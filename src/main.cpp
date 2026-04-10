#include "main.h"
#include "ui/screen_manager.hpp"

#include <atomic>
#include <cctype>
#include <string>

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// ── Chassis constructor ───────────────────────────────────────────────────────
// Update ports and wheel diameter/RPM to match your robot.
ez::Drive chassis(
  {RobotConfig::LEFT_DRIVE_PORTS[0], RobotConfig::LEFT_DRIVE_PORTS[1], RobotConfig::LEFT_DRIVE_PORTS[2]},
  {RobotConfig::RIGHT_DRIVE_PORTS[0], RobotConfig::RIGHT_DRIVE_PORTS[1], RobotConfig::RIGHT_DRIVE_PORTS[2]},
  RobotConfig::IMU_PORT,
    3.25,               // Wheel Diameter (4" wheels without screw holes ≈ 4.125)
    450.0);              // Wheel RPM = cartridge RPM * (motor gear / wheel gear)

// Keep the lateral tracker reversed so EZ's global frame stays +X = right,
// +Y = forward with this sensor mounting.
//ez::tracking_wheel horiz_tracker(-16, 2.0, -1.771654);

enum class IntakeMode {
  Off,
  Loading,
  ScoreHigh,
  ScoreMid,
  ScoreLow,
};

static constexpr std::uint32_t SCORE_MID_INTAKE_DELAY_MS = 300;
static constexpr int kDriverControlDeadband = 10;
static std::atomic_bool s_keep_top_after_auton = false;

static bool driver_control_started() {
  return std::abs(master.get_analog(ANALOG_LEFT_Y)) > kDriverControlDeadband ||
         std::abs(master.get_analog(ANALOG_RIGHT_X)) > kDriverControlDeadband ||
         master.get_digital(DIGITAL_R1) ||
         master.get_digital(DIGITAL_R2) ||
         master.get_digital(DIGITAL_L1) ||
         master.get_digital(DIGITAL_L2) ||
         master.get_digital(DIGITAL_UP) ||
         master.get_digital(DIGITAL_DOWN) ||
         master.get_digital(DIGITAL_LEFT) ||
         master.get_digital(DIGITAL_RIGHT) ||
         master.get_digital(DIGITAL_A) ||
         master.get_digital(DIGITAL_B) ||
         master.get_digital(DIGITAL_X) ||
         master.get_digital(DIGITAL_Y);
}

// ── Screen task ───────────────────────────────────────────────────────────────
static bool s_brain_ui_ready = false;
static constexpr bool kUseEzLcdDebug = false;
static std::atomic_bool s_practice_auton_running = false;

static void screen_task_fn() {
  while (true) {
    if (!s_brain_ui_ready) {
      pros::delay(ez::util::DELAY_TIME);
      continue;
    }

    ScreenManager::ViewModel vm;

    // Odom from EZ-Template
    vm.odomX     = static_cast<float>(chassis.odom_x_get());
    vm.odomY     = static_cast<float>(chassis.odom_y_get());
    vm.odomTheta = static_cast<float>(chassis.odom_theta_get());

    // Selector state (EZ-Template selector)
    vm.autonName = "None";
    vm.autonIndex = 0;
    vm.autonCount = ez::as::auton_selector.auton_count;
    if (vm.autonCount > 0 &&
        ez::as::auton_selector.auton_page_current >= 0 &&
        ez::as::auton_selector.auton_page_current < static_cast<int>(ez::as::auton_selector.Autons.size())) {
      vm.autonIndex = ez::as::auton_selector.auton_page_current;
      vm.autonName  = ez::as::auton_selector.Autons[vm.autonIndex].Name;
    }

    // Battery
    vm.batteryPct   = static_cast<float>(pros::battery::get_capacity());
    vm.batteryVolts = static_cast<float>(pros::battery::get_voltage()) / 1000.0f;

    // Motor temps — find hottest drive motor
    float maxTemp = 0.0f;
    std::string hotName;
    int motorIdx = 0;
    for (auto& m : chassis.left_motors) {
      float t = static_cast<float>(m.get_temperature());
      if (t > maxTemp) { maxTemp = t; hotName = "L" + std::to_string(motorIdx); }
      motorIdx++;
    }
    motorIdx = 0;
    for (auto& m : chassis.right_motors) {
      float t = static_cast<float>(m.get_temperature());
      if (t > maxTemp) { maxTemp = t; hotName = "R" + std::to_string(motorIdx); }
      motorIdx++;
    }
    // Also check intake motors
    for (int i = 0; i < intakeMotors.size(); i++) {
      float t = static_cast<float>(intakeMotors.get_temperature(i));
      if (t > maxTemp) { maxTemp = t; hotName = "Intake"; }
    }
    vm.motorTempMax = maxTemp;
    vm.hotMotor     = hotName;

    // Status
    vm.imuCalibrated = chassis.drive_imu_calibrated();
    vm.compConnected = pros::competition::is_connected();
    if (!vm.imuCalibrated) {
      vm.status = "IMU calibrating...";
    } else if (!vm.compConnected) {
      vm.status = "Practice mode";
    } else {
      vm.status = "Competition ready";
    }

    if (!pros::competition::is_connected() || chassis.pid_tuner_enabled()) {
      ScreenManager::render(vm);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task screenTask(screen_task_fn);

/**
 * Runs initialization code.
 */
void initialize() {
  s_brain_ui_ready = false;
  s_keep_top_after_auton = false;
  top.set_value(false);
  selector.set_value(true);
  tongue.set_value(false);
  wing.set_value(false);
  // Show boot screen
  ScreenManager::renderBoot(0.0f, "Starting...");
  pros::delay(500);

  ScreenManager::renderBoot(0.1f, "Configuring chassis...");

  // ── Tracking wheel setup ───────────────────────────────────────────────────
  //chassis.odom_tracker_back_set(&horiz_tracker);

  // ── Chassis PID + behavior settings ───────────────────────────────────────
  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_drive_activebrake_set(RobotConfig::DRIVER_ACTIVE_BRAKE_POWER);
  chassis.opcontrol_curve_default_set(RobotConfig::DRIVER_FORWARD_CURVE_T,
                                      RobotConfig::DRIVER_TURN_CURVE_T);

  default_constants();

  // ── Auton selector init (EZ-Template) ─────────────────────────────────────
  ScreenManager::renderBoot(0.3f, "Loading autons...");
  using Cat = ScreenManager::AutonCategory;
  ez::as::auton_selector.autons_add({
      // ── Match autons ──────────────────────────────────────────────────────
      {"Left 7 wing",    left7wing},
      {"Left 43",        left43},
      {"Big Bertha",     rightActualAWP},
      {"Right AWP",      rightAWP},
      {"Red Positive",   red_positive_auton},
      {"Red Negative",   red_negative_auton},
      {"Blue Positive",  blue_positive_auton},
      {"Blue Negative",  blue_negative_auton},
      // ── Skills ────────────────────────────────────────────────────────────
      {"Skills",         skills},
      {"crap skills",    shitty_skills},
      // ── Test / tuning ─────────────────────────────────────────────────────
      {"Turn Test",      turn_example},
      {"Odom Test",      odom_drive_example},
      {"RAMSETE Move",   ramsete_move_example},
      {"LTV Move",       ltv_move_example},
      {"RAMSETE Path",   ramsete_path_example},
      {"LTV Path",       ltv_path_example},
      {"RAMSETE+PID",    ramsete_with_pid_example},
  });

  // Register category metadata (must match autons_add order above)
  ScreenManager::setAutonEntries({
      {"Left 7 wing",    Cat::MATCH},
      {"Left 43",        Cat::MATCH},
      {"Big Bertha",     Cat::MATCH},
      {"Right AWP",      Cat::MATCH},
      {"Red Positive",   Cat::MATCH},
      {"Red Negative",   Cat::MATCH},
      {"Blue Positive",  Cat::MATCH},
      {"Blue Negative",  Cat::MATCH},
      {"Skills",         Cat::SKILLS},
      {"crap skills",    Cat::SKILLS},
      {"Turn Test",      Cat::TEST},
      {"Odom Test",      Cat::TEST},
      {"RAMSETE Move",   Cat::TEST},
      {"LTV Move",       Cat::TEST},
      {"RAMSETE Path",   Cat::TEST},
      {"LTV Path",       Cat::TEST},
      {"RAMSETE+PID",    Cat::TEST},
  });

  // Load EZ-Template selector state from SD without starting the default LLEMU UI.
  ez::as::auton_selector_initialize();

  // ── Chassis + IMU init ────────────────────────────────────────────────────
  ScreenManager::renderBoot(0.8f, "Calibrating IMU...");
  chassis.initialize();

  // ── Final screen manager init ─────────────────────────────────────────────
  ScreenManager::renderBoot(1.0f, "Ready");
  pros::delay(400);
  ScreenManager::init();
  s_brain_ui_ready = true;

  // Rumble: '.' = calibrated, '---' = failed
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

}

/**
 * Runs while robot is disabled.
 */
void disabled() {
  // . . .
}

/**
 * Runs before autonomous when connected to Field Management System.
 */
void competition_initialize() {
  // . . .
}

/**
 * Autonomous.
 */
void autonomous() {
  s_keep_top_after_auton = false;
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  // Route to whichever auton EZ-Template selector chose
  ez::as::auton_selector.selected_auton_call();
  top.set_value(true);
  s_keep_top_after_auton = true;
}

/**
 * Tracker helper for ez_screen_task.
 */
void screen_print_tracker(ez::tracking_wheel* tracker, std::string name, int line) {
  std::string val = "", width = "";
  if (tracker != nullptr) {
    val   = name + " tracker: " + util::to_string_with_precision(tracker->get());
    width = "  width: "  + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  ez::screen_print(val + width, line);
}

/**
 * EZ-Template debug screen task — shown only when not at competition.
 */
void ez_screen_task() {
  while (true) {
    if (!kUseEzLcdDebug) {
      pros::delay(ez::util::DELAY_TIME);
      continue;
    }

    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                           "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                           "\na: " + util::to_string_with_precision(chassis.odom_theta_get()), 1);
          screen_print_tracker(chassis.odom_tracker_left,  "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back,  "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    } else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * EZ-Template extras: PID tuner + run auton from opcontrol.
 */
void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    if (!pros::competition::is_connected() &&
        !s_practice_auton_running.load() &&
        master.get_digital(DIGITAL_B) &&
        master.get_digital_new_press(DIGITAL_DOWN)) {
      s_practice_auton_running = true;
      pros::Task([] {
        pros::motor_brake_mode_e_t pref = chassis.drive_brake_get();
        autonomous();
        chassis.drive_brake_set(pref);
        s_practice_auton_running = false;
      }, "Practice Auton");
    }

    chassis.pid_tuner_iterate();
  } else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Operator control.
 */
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  IntakeMode intakeMode = IntakeMode::Off;
  IntakeMode previousIntakeMode = IntakeMode::Off;
  std::uint32_t scoreMidEnteredMs = 0;
  bool wingState = false;
  bool tongueState = false;
  bool infoScreenBrakeDisabled = false;
  bool preserveAutonTop = s_keep_top_after_auton.load();

  selector.set_value(true);
  wing.set_value(wingState);
  tongue.set_value(tongueState);

  while (true) {
    ez_template_extras();

    if (s_practice_auton_running.load()) {
      pros::delay(ez::util::DELAY_TIME);
      continue;
    }

    const bool shouldDisableActiveBrake =
        !pros::competition::is_connected() && ScreenManager::isInfoPageActive();
    if (shouldDisableActiveBrake != infoScreenBrakeDisabled) {
      infoScreenBrakeDisabled = shouldDisableActiveBrake;
      chassis.opcontrol_drive_activebrake_set(
          infoScreenBrakeDisabled ? 0.0 : RobotConfig::DRIVER_ACTIVE_BRAKE_POWER);
    }

    // ── Drive mode (pick one) ──────────────────────────────────────────────
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Split-arcade (recommended)
    // chassis.opcontrol_arcade_standard(ez::SINGLE);
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);
    // chassis.opcontrol_tank();

    if (preserveAutonTop && driver_control_started()) {
      preserveAutonTop = false;
      s_keep_top_after_auton = false;
    }

    int intakeSpeed = 0;
    bool selectMode = true;
    bool upMode = false;

    if (master.get_digital_new_press(DIGITAL_R2)) {
      intakeMode = (intakeMode == IntakeMode::Loading) ? IntakeMode::Off : IntakeMode::Loading;
    }
    if (master.get_digital_new_press(DIGITAL_L2)) {
      intakeMode = (intakeMode == IntakeMode::ScoreHigh) ? IntakeMode::Off : IntakeMode::ScoreHigh;
    }
    if (master.get_digital_new_press(DIGITAL_R1)) {
      intakeMode = (intakeMode == IntakeMode::ScoreMid) ? IntakeMode::Off : IntakeMode::ScoreMid;
    }
    if (master.get_digital_new_press(DIGITAL_L1)) {
      intakeMode = (intakeMode == IntakeMode::ScoreLow) ? IntakeMode::Off : IntakeMode::ScoreLow;
    }

    if (intakeMode != previousIntakeMode) {
      if (intakeMode == IntakeMode::ScoreMid) {
        scoreMidEnteredMs = pros::millis();
      }
      previousIntakeMode = intakeMode;
    }

    switch (intakeMode) {
      case IntakeMode::Loading:
        intakeSpeed = 127;
        selectMode = true;
        upMode = false;
        break;
      case IntakeMode::ScoreHigh:
        intakeSpeed = 127;
        selectMode = true;
        upMode = true;
        break;
      case IntakeMode::ScoreMid:
        intakeSpeed = (pros::millis() - scoreMidEnteredMs >= SCORE_MID_INTAKE_DELAY_MS) ? 127 : 0;
        selectMode = false;
        break;
      case IntakeMode::ScoreLow:
        intakeSpeed = -80;
        break;
      case IntakeMode::Off:
      default:
        intakeSpeed = 0;
        break;
    }

    intakeMotors.move(intakeSpeed);
    selector.set_value(selectMode);
    if (!preserveAutonTop) {
      top.set_value(upMode);
    }

    if (master.get_digital_new_press(DIGITAL_B)) {
      wingState = !wingState;
    }
    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      tongueState = !tongueState;
    }

    wing.set_value(wingState);
    tongue.set_value(tongueState);

    pros::delay(ez::util::DELAY_TIME);
  }
}
