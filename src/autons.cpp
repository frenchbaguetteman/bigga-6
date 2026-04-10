#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/motors.h"
#include "subsystems.hpp"

#include <cmath>
#include <cstdint>

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

namespace {

void wait_for_min_duration(std::uint32_t start_ms,
                           std::uint32_t min_duration_ms) {
  const std::uint32_t elapsed_ms = pros::millis() - start_ms;
  if (elapsed_ms < min_duration_ms) {
    pros::delay(min_duration_ms - elapsed_ms);
  }
}

void run_intake_antijam_task(std::uint32_t startup_delay_ms,
                             bool engage_tongue) {
  pros::delay(startup_delay_ms);
  if (engage_tongue) {
    tongue.set_value(true);
  }

  while (true) {
    if (intakeMotors.get_efficiency() < 3 && !intakeMotors.is_over_temp()) {
      intakeMotors.move(-127);
      pros::delay(200);
      intakeMotors.move(127);
      pros::delay(200);
    }
    pros::delay(20);
  }
}

}  // namespace

static void reset_local_odom() {
  tongue.set_value(false);
  wing.set_value(false);
  top.set_value(false);
  selector.set_value(true);
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
}

class ScopedOdomFeedback {
 public:
  explicit ScopedOdomFeedback(ez::e_odom_feedback feedback)
      : previous_(chassis.odom_feedback_get()) {
    chassis.odom_feedback_set(feedback);
  }

  ScopedOdomFeedback(const ScopedOdomFeedback&) = delete;
  ScopedOdomFeedback& operator=(const ScopedOdomFeedback&) = delete;

  ~ScopedOdomFeedback() {
    chassis.odom_feedback_set(previous_);
  }

 private:
  ez::e_odom_feedback previous_;
};

///
// Constants
///
void default_constants() {
  // Re-scaled from the old 4.125"/420 RPM chassis model to 3.25"/450 RPM.
  chassis.pid_drive_constants_set(13.27, 0.3, 210.78, 5);
  chassis.pid_heading_constants_set(3.0, 0.1, 28.0, 8);
  chassis.pid_turn_constants_set(3.56, 0.3, 23.7, 6);
  chassis.pid_swing_constants_set(7.11, 0.06, 77.0, 8);
  chassis.pid_odom_angular_constants_set(7.70, 0.2, 62.2, 8);
  chassis.pid_odom_boomerang_constants_set(6.87, 0.2, 38.5, 8);

  // Exit conditions
  chassis.pid_turn_exit_condition_set(75_ms, 0.6_deg, 150_ms, 2_deg, 180_ms, 300_ms);
  chassis.pid_swing_exit_condition_set(75_ms, 0.6_deg, 150_ms, 2_deg, 180_ms, 300_ms);
  chassis.pid_drive_exit_condition_set(75_ms, 0.35_in, 150_ms, 1.25_in, 180_ms, 300_ms);
  chassis.pid_odom_turn_exit_condition_set(75_ms, 0.75_deg, 150_ms, 2.5_deg, 220_ms, 350_ms);
  chassis.pid_odom_drive_exit_condition_set(75_ms, 0.40_in, 150_ms, 1.40_in, 220_ms, 350_ms);
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_swing_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(2.4_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.5);

  chassis.odom_look_ahead_set(3.8_in);         // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(6_in);   // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.4);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void left43() {
  reset_local_odom();
  pros::Task intakeTaskAuton([] { run_intake_antijam_task(800, true); });
  chassis.pid_odom_set({{{0_in, 19_in}, fwd, DRIVE_SPEED},
                        {{-18.5_in, 27_in, -90_deg}, fwd, 80}},
                       true);
  chassis.pid_wait();
  const uint32_t tiny_turn_start_ms = pros::millis();
  chassis.pid_turn_set(-91.5_deg, TURN_SPEED);
  chassis.pid_wait();
  wait_for_min_duration(tiny_turn_start_ms, 350);
  chassis.pid_drive_set(-31.5_in, DRIVE_SPEED/1.5, true);
  chassis.pid_wait();
  top.set_value(true);
  pros::delay(2000);
  tongue.set_value(false);
  intakeTaskAuton.remove();
  intakeMotors.move(0);
  chassis.pid_drive_set(6.3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  top.set_value(false);
  pros::delay(100);
  chassis.drive_set(-127, -127);
  pros::delay(400);
  chassis.drive_set(0, 0);
  pros::delay(100);

  chassis.pid_drive_set(18.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  intakeMotors.move(127);
  chassis.pid_drive_set(24.6_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  tongue.set_value(true);
  chassis.pid_drive_set(12.0_in, DRIVE_SPEED / 4, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-13.6_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-7.9_in);
  intakeMotors.move(0);
  chassis.pid_wait();
  selector.set_value(false);
  chassis.pid_wait_until(-12.0_in);
  intakeMotors.move(127);
  pros::delay(2000);
  intakeMotors.move(0);
}



void left7wing() {
  reset_local_odom();
  pros::Task intakeTaskAuton([] { run_intake_antijam_task(500, false); });
  intakeMotors.move(127);
  chassis.pid_drive_set(2_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-15_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(22.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(15_in);
  tongue.set_value(true);
  chassis.pid_wait();
  chassis.pid_turn_set(-133_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(37_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(14_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  const uint32_t tiny_turn_start_ms = pros::millis();
  chassis.pid_turn_set(-181.5_deg, TURN_SPEED);
  chassis.pid_wait();
  wait_for_min_duration(tiny_turn_start_ms, 900);
  chassis.drive_angle_set(-181.5);
  chassis.pid_drive_set(-33.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-30.5_in);
  top.set_value(true);
  chassis.pid_wait();
    pros::delay(1500);
  tongue.set_value(false);
  chassis.pid_drive_set(8.4_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(8.8_in, DRIVE_SPEED/2, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void rightAWP(){
  reset_local_odom();
  pros::Task intakeTaskAuton([] { run_intake_antijam_task(800, true); });
  chassis.pid_odom_set({{{0_in, 19_in}, fwd, DRIVE_SPEED},
                        {{19_in, 27_in, 90_deg}, fwd, 80}},
                       true);
  chassis.pid_wait();
  const uint32_t tiny_turn_start_ms = pros::millis();
  chassis.pid_turn_set(91_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  wait_for_min_duration(tiny_turn_start_ms, 350);
  chassis.pid_drive_set(-31.5_in, DRIVE_SPEED/1.5, true);
  chassis.pid_wait_until(-27.5_in);
    top.set_value(true);
  chassis.pid_wait();
  pros::delay(500);
  tongue.set_value(false);
  intakeTaskAuton.remove();

  pros::Task post_goal_release_task([&] {
    pros::delay(150);
    intakeMotors.move(-127);
    pros::delay(200);
    top.set_value(false);
    intakeMotors.move(80);
  });
  /*ez::pose reference_origin = chassis.odom_pose_get();
  const double reference_origin_rad = ez::util::to_rad(reference_origin.theta);
  auto wrap_angle = [](double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
  };
  auto local_to_global = [&](double local_x, double local_y, double local_theta) {
    return ez::pose{
      reference_origin.x + (local_x * std::cos(reference_origin_rad)) + (local_y * std::sin(reference_origin_rad)),
      reference_origin.y - (local_x * std::sin(reference_origin_rad)) + (local_y * std::cos(reference_origin_rad)),
      wrap_angle(reference_origin.theta + local_theta),
    };
  };
  const ez::pose center_setup = local_to_global(0.0, 4.0, 0.0);
  const ez::pose center_balls = local_to_global(23.0, -13.0, 133.0);
  const ez::pose mid_goal = local_to_global(35, -24.5, 133.0);
  chassis.pid_odom_set({{{center_setup.x, center_setup.y, center_setup.theta}, fwd, DRIVE_SPEED},
                        {{center_balls.x, center_balls.y, center_balls.theta}, fwd, DRIVE_SPEED},
                        {{mid_goal.x, mid_goal.y, mid_goal.theta}, fwd, DRIVE_SPEED / 3}},
                       true);
  chassis.pid_wait_until_index(1);
  top.set_value(false);
  chassis.pid_wait();*/
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-135.5_deg, TURN_SPEED);
  chassis.pid_wait();
  top.set_value(0);
  intakeMotors.move(127);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(23_in, DRIVE_SPEED / 2, true);
  chassis.pid_wait_quick();


  
  intakeMotors.move(-70);
  pros::delay(2000);
  intakeMotors.move(127);

  chassis.pid_drive_set(-14_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intakeMotors.move(127);
  chassis.pid_drive_set(46_in, DRIVE_SPEED, true);
  
  chassis.pid_wait_until(30_in);
  tongue.set_value(1);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-20_in, DRIVE_SPEED/2, true);
  intakeMotors.move(127);
  chassis.pid_wait_until(-10_in);
  selector.set_value(false);
  chassis.pid_wait();
  
  pros::delay(5500);
  
}

void rightActualAWP(){
  reset_local_odom();
  pros::Task intakeTaskAuton([] { run_intake_antijam_task(800, true); });
  chassis.pid_odom_set({{{0_in, 19_in}, fwd, DRIVE_SPEED},
                        {{18.5_in, 27_in, 90_deg}, fwd, 80}},
                       true);
  chassis.pid_wait();
  const uint32_t tiny_turn_start_ms = pros::millis();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  wait_for_min_duration(tiny_turn_start_ms, 350);
  chassis.pid_drive_set(-31.5_in, DRIVE_SPEED/1.5, true);
  chassis.pid_wait_until(-27.5_in);
    top.set_value(true);
  chassis.pid_wait();
  pros::delay(350);
  tongue.set_value(false);
  intakeTaskAuton.remove();

  pros::Task post_goal_release_task([&] {
    pros::delay(150);
    intakeMotors.move(-127);
    pros::delay(200);
    top.set_value(false);
    intakeMotors.move(127);
  });
  //chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  ez::pose reference_origin = chassis.odom_pose_get();
  const double reference_origin_rad = ez::util::to_rad(reference_origin.theta);
  auto wrap_angle = [](double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
  };
  auto local_to_global = [&](double local_x, double local_y, double local_theta) {
    return ez::pose{
      reference_origin.x + (local_x * std::cos(reference_origin_rad)) + (local_y * std::sin(reference_origin_rad)),
      reference_origin.y - (local_x * std::sin(reference_origin_rad)) + (local_y * std::cos(reference_origin_rad)),
      wrap_angle(reference_origin.theta + local_theta),
    };
  };
  const ez::pose center_setup = local_to_global(0.0, 4.0, 0.0);
  const ez::pose center_balls = local_to_global(20.0, -7, 90);
  const ez::pose mid_goal = local_to_global(30, -7, 90);
  chassis.pid_odom_set({{{center_setup.x, center_setup.y, center_setup.theta}, fwd, DRIVE_SPEED},
                        {{center_balls.x, center_balls.y}, fwd, static_cast<int>(DRIVE_SPEED/1.5)},
                        {{mid_goal.x, mid_goal.y, mid_goal.theta}, fwd, DRIVE_SPEED / 2}},
                       true);
  chassis.pid_wait_until_index(1);
  top.set_value(0);
  chassis.pid_wait_quick_chain();
  chassis.drive_angle_set(180);
  chassis.pid_drive_set(40.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(20_in);
  tongue.set_value(1);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-22_in, DRIVE_SPEED/2, true);
  intakeMotors.move(127);
  chassis.pid_wait_until(-15_in);
  intakeMotors.move(80);
  selector.set_value(false);
  chassis.pid_wait();
  pros::delay(150);
  pros::Task anti_spill_task([&] {
    intakeMotors.move(-127);
    pros::delay(100);
    intakeMotors.move(0);
    selector.set_value(true);
  });

  intakeMotors.move(127);
  tongue.set_value(true);

  chassis.pid_drive_set(55_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  intakeMotors.move(127);
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(10);
  chassis.drive_set(127, 127);
  pros::delay(600);
  chassis.drive_angle_set(90_deg);
  chassis.pid_drive_set(-33.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-28.5_in);
  top.set_value(true);
  chassis.pid_wait();
  pros::delay(5000);


  

}

void skills() {
  reset_local_odom();
  pros::Task intakeTaskAuton([] { run_intake_antijam_task(800, true); });
  chassis.pid_odom_set({{{0_in, 19_in}, fwd, DRIVE_SPEED},
                        {{-18.5_in, 27_in, -90_deg}, fwd, 80}},
                       true);
  chassis.pid_wait();
  pros::delay(3000);
  intakeTaskAuton.remove();
  intakeMotors.move(0);
  tongue.set_value(false);

  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  chassis.pid_turn_set(60_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(27_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(65_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(15.7_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-23_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  intakeMotors.move(127);
  top.set_value(true);
  tongue.set_value(1);   
  pros::delay(3000);
  chassis.pid_turn_set(91.5_deg, TURN_SPEED);  
  chassis.pid_wait();                                                                                                                                                                                                                                                
  chassis.pid_drive_set(33_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  top.set_value(false);
  pros::delay(2000);
  chassis.pid_turn_set(91.5_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-33_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  top.set_value(true);
  pros::delay(3000);
  

  chassis.pid_drive_set(6.3_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  top.set_value(false);
  pros::delay(100);
  chassis.drive_set(-127, -127);
  pros::delay(400);
  chassis.drive_set(0, 0);
  pros::delay(100);
  intakeMotors.move(0);

  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(97_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(28_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  intakeMotors.move(127);
  pros::delay(2000);
  intakeMotors.move(0);

  tongue.set_value(0);


  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  wing.set_value(1);
  
  chassis.pid_turn_set(-60_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(65_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(19_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-24.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  intakeMotors.move(127);
  top.set_value(true);
  pros::delay(3000);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  tongue.set_value(1);                                                                                                                                                                                                                                                     
  chassis.pid_drive_set(33_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  top.set_value(false);
  pros::delay(2000);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-33_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  top.set_value(true);
  pros::delay(3000);

  chassis.pid_turn_set(-75_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(50_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  chassis.pid_wait();



  }

void shitty_skills() {
  reset_local_odom();
  intakeMotors.move(-127);
  top.set_value(1);
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(65_in, DRIVE_SPEED);
  chassis.pid_wait_until(40);
  chassis.drive_set(127, 127);
  pros::delay(500);
  chassis.drive_set(0,0);
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_BRAKE);
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// RAMSETE single-move example
///
void ramsete_move_example() {
  reset_local_odom();
  {
    ScopedOdomFeedback ramsete(ez::RAMSETE_FEEDBACK);
    chassis.pid_odom_set({{0_in, 24_in, 0_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();

    chassis.pid_odom_set({{18_in, 36_in, 45_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();

    chassis.pid_odom_set({{0_in, 0_in, 180_deg}, rev, DRIVE_SPEED}, true);
    chassis.pid_wait();
  }
}

///
// LTV single-move example
///
void ltv_move_example() {
  reset_local_odom();
  {
    ScopedOdomFeedback ltv(ez::LTV_FEEDBACK);
    chassis.pid_odom_set({{0_in, 24_in, 0_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();

    chassis.pid_odom_set({{12_in, 36_in, 45_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();

    chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED}, true);
    chassis.pid_wait();
  }
}

///
// RAMSETE multi-waypoint path example
///
void ramsete_path_example() {
  reset_local_odom();
  {
    ScopedOdomFeedback ramsete(ez::RAMSETE_FEEDBACK);
    chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                          {{24_in, 24_in}, fwd, DRIVE_SPEED},
                          {{24_in, 48_in, 0_deg}, fwd, 80}},
                         true);
    chassis.pid_wait();
  }
}

///
// LTV multi-waypoint path example
///
void ltv_path_example() {
  reset_local_odom();
  {
    ScopedOdomFeedback ltv(ez::LTV_FEEDBACK);
    chassis.pid_odom_set({{{8_in, 16_in}, fwd, DRIVE_SPEED},
                          {{-8_in, 32_in}, fwd, DRIVE_SPEED},
                          {{0_in, 48_in, 0_deg}, fwd, 80}},
                         true);
    chassis.pid_wait();

    chassis.pid_odom_set({{0_in, 0_in, 180_deg}, rev, DRIVE_SPEED}, true);
    chassis.pid_wait();
  }
}

///
// Mixed: RAMSETE path then PID turn (shows interop with EZ-Template)
///
void ramsete_with_pid_example() {
  reset_local_odom();

  {
    ScopedOdomFeedback ramsete(ez::RAMSETE_FEEDBACK);
    chassis.pid_odom_set({{18_in, 30_in, 0_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();
  }

  // Switch back to PID for a precise turn
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  {
    ScopedOdomFeedback ramsete(ez::RAMSETE_FEEDBACK);
    chassis.pid_odom_set({{36_in, 30_in, 90_deg}, fwd, DRIVE_SPEED}, true);
    chassis.pid_wait();
  }
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

// ─────────────────────────────────────────────────────────────────────────────
// Competition autonomous routines
// Replace the bodies below with your actual paths for each routine.
// ─────────────────────────────────────────────────────────────────────────────

void red_positive_auton() {
  // TODO: Add red-positive-side routine
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void red_negative_auton() {
  // TODO: Add red-negative-side routine
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void blue_positive_auton() {
  // TODO: Add blue-positive-side routine
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void blue_negative_auton() {
  // TODO: Add blue-negative-side routine
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}


// ─────────────────────────────────────────────────────────────────────────────
// Dispatcher: use EZ-Template selector directly
// ─────────────────────────────────────────────────────────────────────────────
void run_selected_auton() {
  ez::as::auton_selector.selected_auton_call();
}
