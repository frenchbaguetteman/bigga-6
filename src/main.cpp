#include "main.h"
#include "ui/screen_manager.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// ── Chassis constructor ───────────────────────────────────────────────────────
// Update ports and wheel diameter/RPM to match your robot.
ez::Drive chassis(
  {-11, -12, -14},
  {18, 19, 20},
  16,
    3.25,               // Wheel Diameter (4" wheels without screw holes ≈ 4.125)
    450.0);              // Wheel RPM = cartridge RPM * (motor gear / wheel gear)

// Keep the lateral tracker reversed so EZ's global frame stays +X = right,
// +Y = forward with this sensor mounting.
//ez::tracking_wheel horiz_tracker(-16, 2.0, -1.771654);

static bool s_brain_ui_ready = false;
static std::atomic_bool s_practice_auton_running = false;
static std::atomic_bool s_keep_top_after_auton = false;

namespace {

enum class IntakeMode {
  Off,
  Loading,
  ScoreHigh,
  ScoreMid,
  ScoreLow,
};

using AutonCategory = ScreenManager::AutonCategory;
using AutonRoutine = void (*)();

struct AutonDefinition {
  const char* name;
  AutonRoutine routine;
  AutonCategory category;
};

struct HottestMotor {
  float temperature = 0.0f;
  char name[16] = "";
};

struct IntakeCommand {
  int speed = 0;
  bool selectMode = true;
  bool topMode = false;
};

constexpr std::uint32_t kScoreMidIntakeDelayMs = 300;
constexpr int kDriverControlDeadband = 10;
constexpr double kDriverActiveBrake = 1.5;
constexpr const char* kNoSdDefaultAutonName = "LTV Path";
constexpr std::array<AutonDefinition, 13> kAutonCatalog{{
    {"Left 7 wing", left7wing, AutonCategory::MATCH},
    {"Left 43", left43, AutonCategory::MATCH},
    {"Big Bertha", rightActualAWP, AutonCategory::MATCH},
    {"Right AWP", rightAWP, AutonCategory::MATCH},
    {"Skills", skills, AutonCategory::SKILLS},
    {"crap skills", shitty_skills, AutonCategory::SKILLS},
    {"Turn Test", turn_example, AutonCategory::TEST},
    {"Odom Test", odom_drive_example, AutonCategory::TEST},
    {"RAMSETE Move", ramsete_move_example, AutonCategory::TEST},
    {"LTV Move", ltv_move_example, AutonCategory::TEST},
    {"RAMSETE Path", ramsete_path_example, AutonCategory::TEST},
    {"LTV Path", ltv_path_example, AutonCategory::TEST},
    {"RAMSETE+PID", ramsete_with_pid_example, AutonCategory::TEST},
}};

int no_sd_default_auton_index() {
  for (int i = 0; i < static_cast<int>(kAutonCatalog.size()); ++i) {
    if (std::strcmp(kAutonCatalog[i].name, kNoSdDefaultAutonName) == 0) {
      return i;
    }
  }
  return 0;
}

void apply_no_sd_default_auton() {
  if (ez::util::SD_CARD_ACTIVE) return;

  const int count = ez::as::auton_selector.auton_count;
  if (count <= 0) return;

  int index = no_sd_default_auton_index();
  if (index < 0 || index >= count) {
    index = 0;
  }

  ez::as::auton_selector.last_auton_page_current = index;
  ez::as::auton_selector.auton_page_current = index;
}

template <std::size_t N>
void copy_text(char (&dest)[N], const char* src) {
  std::snprintf(dest, N, "%s", src ? src : "");
}

void reset_actuators() {
  top.set_value(false);
  selector.set_value(true);
  tongue.set_value(false);
  wing.set_value(false);
}

void reset_autonomous_state() {
  s_keep_top_after_auton = false;
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE);
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
}

template <typename MotorContainer>
void scan_hottest_motor_group(HottestMotor& hottest,
                              MotorContainer& motors,
                              const char* prefix) {
  int index = 0;
  for (auto& motor : motors) {
    const float temperature = static_cast<float>(motor.get_temperature());
    if (temperature > hottest.temperature) {
      hottest.temperature = temperature;
      std::snprintf(hottest.name, sizeof(hottest.name), "%s%d", prefix, index);
    }
    ++index;
  }
}

HottestMotor hottest_motor() {
  HottestMotor hottest;
  scan_hottest_motor_group(hottest, chassis.left_motors, "L");
  scan_hottest_motor_group(hottest, chassis.right_motors, "R");

  for (int i = 0; i < intakeMotors.size(); ++i) {
    const float temperature = static_cast<float>(intakeMotors.get_temperature(i));
    if (temperature > hottest.temperature) {
      hottest.temperature = temperature;
      copy_text(hottest.name, "Intake");
    }
  }

  return hottest;
}

void populate_auton_selection(ScreenManager::ViewModel& vm) {
  copy_text(vm.autonName, "None");
  vm.autonIndex = 0;

  const auto& selectorState = ez::as::auton_selector;
  vm.autonCount = selectorState.auton_count;

  if (vm.autonCount <= 0) {
    return;
  }

  const int current = selectorState.auton_page_current;
  if (current < 0 || current >= static_cast<int>(selectorState.Autons.size())) {
    return;
  }

  vm.autonIndex = current;
  copy_text(vm.autonName, selectorState.Autons[current].Name.c_str());
}

const char* competition_status(bool imu_calibrated, bool competition_connected) {
  if (!imu_calibrated) {
    return "IMU calibrating...";
  }
  if (!competition_connected) {
    return "Practice mode";
  }
  return "Competition ready";
}

ScreenManager::ViewModel build_view_model() {
  ScreenManager::ViewModel vm;

  vm.odomX = static_cast<float>(chassis.odom_x_get());
  vm.odomY = static_cast<float>(chassis.odom_y_get());
  vm.odomTheta = static_cast<float>(chassis.odom_theta_get());
  populate_auton_selection(vm);

  vm.batteryPct = static_cast<float>(pros::battery::get_capacity());
  vm.batteryVolts = static_cast<float>(pros::battery::get_voltage()) / 1000.0f;

  const HottestMotor hottest = hottest_motor();
  vm.motorTempMax = hottest.temperature;
  copy_text(vm.hotMotor, hottest.name);

  vm.imuCalibrated = chassis.drive_imu_calibrated();
  vm.compConnected = pros::competition::is_connected();
  copy_text(vm.status, competition_status(vm.imuCalibrated, vm.compConnected));

  return vm;
}

bool should_render_screen() {
  return !pros::competition::is_connected() || chassis.pid_tuner_enabled();
}

std::vector<ez::Auton> selector_autons() {
  std::vector<ez::Auton> autons;
  autons.reserve(kAutonCatalog.size());
  for (const auto& auton : kAutonCatalog) {
    autons.emplace_back(auton.name, auton.routine);
  }
  return autons;
}

std::vector<ScreenManager::AutonEntry> screen_autons() {
  std::vector<ScreenManager::AutonEntry> entries;
  entries.reserve(kAutonCatalog.size());
  for (const auto& auton : kAutonCatalog) {
    entries.push_back({auton.name, auton.category});
  }
  return entries;
}

void register_autons() {
  ez::as::auton_selector.autons_add(selector_autons());
  ScreenManager::setAutonEntries(screen_autons());
}

bool driver_control_started() {
  if (std::abs(master.get_analog(ANALOG_LEFT_Y)) > kDriverControlDeadband ||
      std::abs(master.get_analog(ANALOG_RIGHT_X)) > kDriverControlDeadband) {
    return true;
  }

  for (auto button : {DIGITAL_R1, DIGITAL_R2, DIGITAL_L1, DIGITAL_L2,
                      DIGITAL_UP, DIGITAL_DOWN, DIGITAL_LEFT, DIGITAL_RIGHT,
                      DIGITAL_A, DIGITAL_B, DIGITAL_X, DIGITAL_Y}) {
    if (master.get_digital(button)) {
      return true;
    }
  }

  return false;
}

void toggle_intake_mode(IntakeMode& current_mode,
                        pros::controller_digital_e_t button,
                        IntakeMode target_mode) {
  if (master.get_digital_new_press(button)) {
    current_mode = (current_mode == target_mode) ? IntakeMode::Off : target_mode;
  }
}

void update_intake_mode(IntakeMode& intake_mode) {
  toggle_intake_mode(intake_mode, DIGITAL_R2, IntakeMode::Loading);
  toggle_intake_mode(intake_mode, DIGITAL_L2, IntakeMode::ScoreHigh);
  toggle_intake_mode(intake_mode, DIGITAL_R1, IntakeMode::ScoreMid);
  toggle_intake_mode(intake_mode, DIGITAL_L1, IntakeMode::ScoreLow);
}

IntakeCommand intake_command_for_mode(IntakeMode mode,
                                      std::uint32_t score_mid_entered_ms) {
  switch (mode) {
    case IntakeMode::Loading:
      return {.speed = 127, .selectMode = true, .topMode = false};
    case IntakeMode::ScoreHigh:
      return {.speed = 127, .selectMode = true, .topMode = true};
    case IntakeMode::ScoreMid:
      return {.speed = (pros::millis() - score_mid_entered_ms >= kScoreMidIntakeDelayMs) ? 127 : 0,
              .selectMode = false,
              .topMode = false};
    case IntakeMode::ScoreLow:
      return {.speed = -80, .selectMode = true, .topMode = false};
    case IntakeMode::Off:
    default:
      return {};
  }
}

void apply_intake_command(const IntakeCommand& command, bool preserve_auton_top) {
  intakeMotors.move(command.speed);
  selector.set_value(command.selectMode);
  if (!preserve_auton_top) {
    top.set_value(command.topMode);
  }
}

}  // namespace

static void screen_task_fn() {
  while (true) {
    if (!s_brain_ui_ready) {
      pros::delay(ez::util::DELAY_TIME);
      continue;
    }

    if (should_render_screen()) {
      ScreenManager::render(build_view_model());
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
  reset_actuators();
  // Show boot screen
  ScreenManager::renderBoot(0.0f, "Starting...");
  pros::delay(500);

  ScreenManager::renderBoot(0.1f, "Configuring chassis...");

  // ── Tracking wheel setup ───────────────────────────────────────────────────
  //chassis.odom_tracker_back_set(&horiz_tracker);

  // ── Chassis PID + behavior settings ───────────────────────────────────────
  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_drive_activebrake_set(1.5);
  chassis.opcontrol_curve_default_set(5.0, 5.0);

  default_constants();

  // ── Auton selector init (EZ-Template) ─────────────────────────────────────
  ScreenManager::renderBoot(0.3f, "Loading autons...");
  register_autons();
  apply_no_sd_default_auton();

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
  reset_autonomous_state();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  ez::as::auton_selector.selected_auton_call();
  top.set_value(true);
  s_keep_top_after_auton = true;
}

/**
 * EZ-Template extras: PID tuner + run auton from opcontrol.
 */
void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    if (!s_practice_auton_running.load() &&
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

    const bool competitionConnected = pros::competition::is_connected();
    const bool shouldDisableActiveBrake =
        !competitionConnected && ScreenManager::isInfoPageActive();
    if (shouldDisableActiveBrake != infoScreenBrakeDisabled) {
      infoScreenBrakeDisabled = shouldDisableActiveBrake;
      chassis.opcontrol_drive_activebrake_set(
          infoScreenBrakeDisabled ? 0.0 : kDriverActiveBrake);
    }

    chassis.opcontrol_arcade_standard(ez::SPLIT);

    if (preserveAutonTop && driver_control_started()) {
      preserveAutonTop = false;
      s_keep_top_after_auton = false;
    }

    update_intake_mode(intakeMode);

    if (intakeMode != previousIntakeMode) {
      if (intakeMode == IntakeMode::ScoreMid) {
        scoreMidEnteredMs = pros::millis();
      }
      previousIntakeMode = intakeMode;
    }

    apply_intake_command(
        intake_command_for_mode(intakeMode, scoreMidEnteredMs),
        preserveAutonTop);

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
