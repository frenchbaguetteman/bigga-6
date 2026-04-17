#include "EZ-Template/util.hpp"
#include "main.h"

#include <cmath>
#include <cstdint>

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
constexpr int DRIVE_SPEED = 110;
constexpr int TURN_SPEED = 90;

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

double wrap_angle_degrees(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle <= -180.0) angle += 360.0;
  return angle;
}

ez::pose local_pose_from(const ez::pose& origin,
                         double local_x,
                         double local_y,
                         double local_theta) {
  const double origin_rad = ez::util::to_rad(origin.theta);
  return ez::pose{
      origin.x + (local_x * std::cos(origin_rad)) + (local_y * std::sin(origin_rad)),
      origin.y - (local_x * std::sin(origin_rad)) + (local_y * std::cos(origin_rad)),
      wrap_angle_degrees(origin.theta + local_theta),
  };
}

void reset_local_odom() {
  tongue.set_value(false);
  wing.set_value(false);
  top.set_value(false);
  selector.set_value(true);
  chassis.pid_targets_reset();
  chassis.drive_mode_set(ez::DISABLE);
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
}

}  // namespace

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
  ez::pose reference_origin = chassis.odom_pose_get();
  const ez::pose center_setup = local_pose_from(reference_origin, 0.0, 4.0, 0.0);
  const ez::pose center_balls = local_pose_from(reference_origin, 20.0, -7.0, 90.0);
  const ez::pose mid_goal = local_pose_from(reference_origin, 30.0, -7.0, 90.0);
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
  reset_local_odom();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.
  reset_local_odom();

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
                          {{0_in, 0_in, 0_deg}, fwd, 80}},
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
    chassis.pid_odom_set({{{8_in, 16_in}, fwd, 60},
                          {{-8_in, 32_in}, fwd, 60},
                          {{0_in, 48_in, 0_deg}, fwd, 55}},
                         true);
    chassis.pid_wait();
    
    //chassis.pid_odom_set({{0_in, 0_in, 180_deg}, rev, 60}, true);
    //chassis.pid_wait();
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
