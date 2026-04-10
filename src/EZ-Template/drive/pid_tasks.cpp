/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "controllers/ltv_controller.hpp"
#include "controllers/ramsete_controller.hpp"
#include "robot_config.hpp"
#include "pros/misc.hpp"

#include <algorithm>
#include <cmath>

using namespace ez;

namespace {

constexpr double kMaxDriveMillivolts = 12000.0;

struct TrackingControllerResult {
  double linear_inps = 0.0;
  double angular_radps = 0.0;
  float err_fwd = 0.0f;
  float err_lat = 0.0f;
  float err_hdg = 0.0f;
};

struct WheelVelocityState {
  double left_inps = 0.0;
  double right_inps = 0.0;
};

struct TrackingVoltageCommand {
  double left_vel_error_mps = 0.0;
  double right_vel_error_mps = 0.0;
  double ff_left_mv = 0.0;
  double ff_right_mv = 0.0;
  double fb_left_mv = 0.0;
  double fb_right_mv = 0.0;
  int left_cmd = 0;
  int right_cmd = 0;
};

double clamp_symmetric(double value, double limit) {
  return std::clamp(value, -limit, limit);
}

std::pair<double, double> limit_wheel_speeds(double left_inps,
                                             double right_inps) {
  const double max_scale = std::max(
      {1.0, std::fabs(left_inps) / RobotConfig::MAX_SPEED_INPS,
       std::fabs(right_inps) / RobotConfig::MAX_SPEED_INPS});
  return {left_inps / max_scale, right_inps / max_scale};
}

int millivolts_to_drive_command(double millivolts) {
  const double clipped = clamp_symmetric(millivolts, kMaxDriveMillivolts);
  return static_cast<int>(std::round(clipped * (127.0 / kMaxDriveMillivolts)));
}

double average_motor_rpm(const std::vector<pros::Motor>& motors) {
  if (motors.empty()) {
    return 0.0;
  }

  double sum_rpm = 0.0;
  for (const auto& motor : motors) {
    sum_rpm += motor.get_actual_velocity();
  }
  return sum_rpm / static_cast<double>(motors.size());
}

double motor_rpm_to_wheel_inps(double motor_rpm, double wheel_diameter_in,
                               double gear_ratio) {
  const double safe_ratio = std::max(std::fabs(gear_ratio), 1e-6);
  return (motor_rpm * wheel_diameter_in * M_PI) / (60.0 * safe_ratio);
}

WheelVelocityState limit_tracking_wheel_speeds(
    TrackingControllerResult& controller_result) {
  const double half_track = RobotConfig::TRACK_WIDTH_IN / 2.0;
  auto [left_inps, right_inps] = limit_wheel_speeds(
      controller_result.linear_inps + (controller_result.angular_radps * half_track),
      controller_result.linear_inps - (controller_result.angular_radps * half_track));

  controller_result.linear_inps = (left_inps + right_inps) / 2.0;
  controller_result.angular_radps =
      (left_inps - right_inps) / RobotConfig::TRACK_WIDTH_IN;
  return {left_inps, right_inps};
}

}  // namespace

// File-scoped controller instances shared by all odom tasks.
static LtvController     s_ltv;
static RamseteController s_ramsete;
static int s_ctrl_log_counter = 0;

void Drive::ez_auto_task() {
  while (true) {
    // Run odom
    ez_tracking_task();

    // Autonomous PID
    switch (drive_mode_get()) {
      case DRIVE:
        drive_pid_task();
        break;
      case TURN ... TURN_TO_POINT:
        turn_pid_task();
        break;
      case SWING:
        swing_pid_task();
        break;
      case POINT_TO_POINT:
        ptp_task();
        break;
      case PURE_PURSUIT:
        pp_task();
        break;
      case DISABLE:
        break;
      default:
        break;
    }

    // This is used to reset sensors for active braking
    util::AUTON_RAN = drive_mode_get() != DISABLE;

    pros::delay(ez::util::DELAY_TIME);
  }
}

// Drive PID task
void Drive::drive_pid_task() {
  // Compute PID
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  headingPID.compute(drive_imu_get());

  // Compute slew
  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());

  // Left and Right outputs
  double l_drive_out = leftPID.output;
  double r_drive_out = rightPID.output;

  // Scale leftPID and rightPID to slew (if slew is disabled, it returns max_speed)
  double max_slew_out = fmax(slew_left.output(), slew_right.output());
  double faster_side = fmax(fabs(l_drive_out), fabs(r_drive_out));
  if (faster_side > max_slew_out) {
    l_drive_out *= (max_slew_out / faster_side);
    r_drive_out *= (max_slew_out / faster_side);
  }

  // Toggle heading
  double imu_out = heading_on ? headingPID.output : 0;

  // Combine heading and drive
  double l_out = l_drive_out + imu_out;
  double r_out = r_drive_out - imu_out;

  // Vector scaling when combining drive and imu
  max_slew_out = fmax(slew_left.output(), slew_right.output());
  faster_side = fmax(fabs(l_out), fabs(r_out));
  if (faster_side > max_slew_out) {
    l_out *= (max_slew_out / faster_side);
    r_out *= (max_slew_out / faster_side);
  }

  // Set motors
  if (drive_toggle)
    private_drive_set(l_out, r_out);
}

// Turn PID task
void Drive::turn_pid_task() {
  // Compute PID if it's a normal turn
  if (mode == TURN) {
    turnPID.compute(drive_imu_get());
  }
  // Compute PID if we're turning to point
  else {
    double a_target = util::absolute_angle_to_point(point_to_face[!ptf1_running], odom_pose_get());  // Calculate the point for angle to face
    a_target = new_turn_target_compute(a_target, odom_imu_start, current_angle_behavior);
    double error = util::wrap_angle(a_target - odom_theta_get());
    turnPID.compute_error(error, odom_theta_get());
  }

  // Compute slew
  slew_turn.iterate(drive_imu_get());

  // Clip gyroPID to max speed
  double gyro_out = util::clamp(turnPID.output, slew_turn.output(), -slew_turn.output());

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (turnPID.constants.ki != 0 && (fabs(turnPID.target_get()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
    if (pid_turn_min_get() != 0)
      gyro_out = util::clamp(gyro_out, pid_turn_min_get(), -pid_turn_min_get());
  }

  // Set motors
  if (drive_toggle)
    private_drive_set(gyro_out, -gyro_out);
}

// Swing PID task
void Drive::swing_pid_task() {
  // Compute PID
  swingPID.compute(drive_imu_get());
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  // Compute slew
  double current = slew_swing_using_angle ? drive_imu_get() : (current_swing == LEFT_SWING ? drive_sensor_left() : drive_sensor_right());
  slew_swing.iterate(current);

  // Clip swingPID to max speed
  double swing_out = util::clamp(swingPID.output, slew_swing.output(), -slew_swing.output());

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (swingPID.constants.ki != 0 && (fabs(swingPID.target_get()) > swingPID.constants.start_i && fabs(swingPID.error) < swingPID.constants.start_i)) {
    if (pid_swing_min_get() != 0)
      swing_out = util::clamp(swing_out, pid_swing_min_get(), -pid_swing_min_get());
  }

  // Set the motors powers, and decide what to do with the "still" side of the drive
  double opposite_output = 0;
  double scale = swing_out / max_speed;
  if (drive_toggle) {
    // Check if left or right swing, then set motors accordingly
    if (current_swing == LEFT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? rightPID.output : (swing_opposite_speed * scale);
      private_drive_set(swing_out, opposite_output);
    } else if (current_swing == RIGHT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? leftPID.output : -(swing_opposite_speed * scale);
      private_drive_set(opposite_output, -swing_out);
    }
  }
}

// Odom To Point Task
void Drive::ptp_task() {
  if (odom_feedback_type != PID_FEEDBACK) {
    odom_reference_task();
    return;
  }

  // Compute slew
  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());
  double max_slew_out = fmax(slew_left.output(), slew_right.output());

  // Decide if we've past the target or not
  double temp_target = is_past_target(odom_target, odom_pose_get());        // Use this instead of distance formula to fix impossible movements
  int dir = (current_drive_direction == REV ? -1 : 1);                      // If we're going backwards, add a -1
  int flipped = util::sgn(temp_target) != util::sgn(past_target) ? -1 : 1;  // Check if we've flipped directions to what we started

  // Compute xy PID
  new_current_fake += xy_delta_fake * ((dir * flipped));  // Create a "current sensor value" for the PID to calculate off of
  xyPID.compute_error(fabs(temp_target) * dir * flipped, new_current_fake);

  // Compute angle
  pose ptf = point_to_face[!ptf1_running];
  double a_target = util::absolute_angle_to_point(ptf, odom_pose_get());  // Calculate the point for angle to face
  a_target = new_turn_target_compute(a_target, odom_imu_start, current_angle_behavior);
  double wrapped_a_target = util::wrap_angle(a_target - odom_theta_get());
  current_a_odomPID.compute_error(wrapped_a_target, odom_theta_get());

  // Prioritize turning by scaling xy_out down
  double xy_out = xyPID.output;
  xy_out = util::clamp(xy_out, max_slew_out);
  double scale = 1.0 - ((1.0 - cos(util::to_rad(current_a_odomPID.error))) / odom_turn_bias_amount);
  scale = util::clamp(scale, 1.0, 0.0);
  if (odom_turn_bias_enabled())
    xy_out *= scale;
  double a_out = current_a_odomPID.output;

  // Scale xy_out and a_out to max speed
  double faster_side = fmax(fabs(xy_out), fabs(a_out));
  if (faster_side > max_slew_out) {
    xy_out *= (max_slew_out / faster_side);
    a_out *= (max_slew_out / faster_side);
  }

  // Combine heading and drive
  double l_out = xy_out + a_out;
  double r_out = xy_out - a_out;

  // Vector scaling when combining drive and imu
  faster_side = fmax(fabs(l_out), fabs(r_out));
  if (faster_side > max_slew_out) {
    l_out *= (max_slew_out / faster_side);
    r_out *= (max_slew_out / faster_side);
  }

  // Set motors
  if (drive_toggle)
    private_drive_set(l_out, r_out);

  // This is for wait_until
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());
}

void Drive::odom_reference_task() {
  if (odom_reference_states.empty()) {
    reset_odom_reference();
    if (drive_toggle) {
      private_drive_set(0, 0);
    }
    leftPID.compute(drive_sensor_left());
    rightPID.compute(drive_sensor_right());
    return;
  }

  const odom_reference_state ref = sample_odom_reference_state();
  const pose current = odom_pose_get();
  const double avg_progress =
      ((drive_sensor_left() - l_start) + (drive_sensor_right() - r_start)) /
      2.0;
  const double translation_error =
      util::distance_to_point(ref.target_pose, current);
  const double heading_error =
      util::wrap_angle(ref.target_pose.theta - odom_theta_get());

  current_drive_direction = ref.drive_direction;
  odom_target = ref.target_pose;

  xyPID.compute_error(translation_error, avg_progress);
  current_a_odomPID.compute_error(heading_error, odom_theta_get());

  const auto calculate_tracking_command = [&]() {
    const float cx = static_cast<float>(current.x);
    const float cy = static_cast<float>(current.y);
    const float ct = static_cast<float>(current.theta);
    const float tx = static_cast<float>(ref.target_pose.x);
    const float ty = static_cast<float>(ref.target_pose.y);
    const float tt = static_cast<float>(ref.target_pose.theta);
    const float v_ref = static_cast<float>(ref.linear_velocity);
    const float omega_ref_deg =
        static_cast<float>(ref.angular_velocity * RobotConfig::RAD_TO_DEG);

    TrackingControllerResult result;
    if (odom_feedback_type == LTV_FEEDBACK) {
      const auto cmd = s_ltv.calculateChassisSpeeds(cx, cy, ct, tx, ty, tt,
                                                    v_ref, omega_ref_deg);
      const auto& e = s_ltv.lastError();
      result.err_fwd = e[0];
      result.err_lat = e[1];
      result.err_hdg = e[2];
      result.linear_inps = cmd.linear;
      result.angular_radps = cmd.angular;
      return result;
    }

    const auto cmd = s_ramsete.calculateChassisSpeeds(cx, cy, ct, tx, ty, tt,
                                                      v_ref, omega_ref_deg);
    result.err_fwd = s_ramsete.lastEx();
    result.err_lat = s_ramsete.lastEy();
    result.err_hdg = s_ramsete.lastEth();
    result.linear_inps = cmd.linear;
    result.angular_radps = cmd.angular;
    return result;
  };

  const auto update_filtered_wheel_velocity_state =
      [&](std::uint32_t now_ms, double& dt_s) {
    dt_s = util::DELAY_TIME / 1000.0;

    const WheelVelocityState raw{
        motor_rpm_to_wheel_inps(average_motor_rpm(left_motors), WHEEL_DIAMETER,
                                RATIO),
        motor_rpm_to_wheel_inps(average_motor_rpm(right_motors), WHEEL_DIAMETER,
                                RATIO)};

    if (odom_reference_feedback_valid && now_ms > odom_reference_last_ms) {
      dt_s = std::max(1e-3, (now_ms - odom_reference_last_ms) / 1000.0);
      constexpr double kAlpha = RobotConfig::TRACKING_VEL_MEAS_ALPHA;
      odom_reference_left_vel =
          (kAlpha * raw.left_inps) + ((1.0 - kAlpha) * odom_reference_left_vel);
      odom_reference_right_vel =
          (kAlpha * raw.right_inps) + ((1.0 - kAlpha) * odom_reference_right_vel);
    } else {
      odom_reference_left_vel = raw.left_inps;
      odom_reference_right_vel = raw.right_inps;
    }

    return WheelVelocityState{odom_reference_left_vel, odom_reference_right_vel};
  };

  const auto calculate_tracking_voltage_command =
      [&](const TrackingControllerResult& controller_result,
          const WheelVelocityState& desired_wheels,
          const WheelVelocityState& measured_wheels, double dt_s) {
    const double cmd_linear_accel =
        odom_reference_feedback_valid
            ? (controller_result.linear_inps - odom_reference_prev_cmd_v) / dt_s
            : ref.linear_acceleration;
    const double cmd_angular_accel =
        odom_reference_feedback_valid
            ? (controller_result.angular_radps - odom_reference_prev_cmd_omega) /
                  dt_s
            : ref.angular_acceleration;

    const auto [ff_left_mv, ff_right_mv] =
        RobotConfig::drivetrainFeedforward_mV(
            static_cast<float>(controller_result.linear_inps *
                               RobotConfig::IN_TO_M),
            static_cast<float>(controller_result.angular_radps),
            static_cast<float>(cmd_linear_accel * RobotConfig::IN_TO_M),
            static_cast<float>(cmd_angular_accel));

    const double desired_left_mps =
        desired_wheels.left_inps * RobotConfig::IN_TO_M;
    const double desired_right_mps =
        desired_wheels.right_inps * RobotConfig::IN_TO_M;
    const double measured_left_mps =
        measured_wheels.left_inps * RobotConfig::IN_TO_M;
    const double measured_right_mps =
        measured_wheels.right_inps * RobotConfig::IN_TO_M;

    TrackingVoltageCommand result;
    result.left_vel_error_mps = desired_left_mps - measured_left_mps;
    result.right_vel_error_mps = desired_right_mps - measured_right_mps;

    const double left_error_rate =
        odom_reference_feedback_valid
            ? (result.left_vel_error_mps - odom_reference_prev_left_err) / dt_s
            : 0.0;
    const double right_error_rate =
        odom_reference_feedback_valid
            ? (result.right_vel_error_mps - odom_reference_prev_right_err) / dt_s
            : 0.0;

    result.ff_left_mv = ff_left_mv;
    result.ff_right_mv = ff_right_mv;
    result.fb_left_mv = clamp_symmetric(
        (RobotConfig::TRACKING_VEL_KP_MV_PER_MPS * result.left_vel_error_mps) +
            (RobotConfig::TRACKING_VEL_KD_MV_PER_MPS2 * left_error_rate),
        RobotConfig::TRACKING_VEL_MAX_CORRECTION_MV);
    result.fb_right_mv = clamp_symmetric(
        (RobotConfig::TRACKING_VEL_KP_MV_PER_MPS * result.right_vel_error_mps) +
            (RobotConfig::TRACKING_VEL_KD_MV_PER_MPS2 * right_error_rate),
        RobotConfig::TRACKING_VEL_MAX_CORRECTION_MV);

    const double left_out_mv = clamp_symmetric(
        result.ff_left_mv + result.fb_left_mv, kMaxDriveMillivolts);
    const double right_out_mv = clamp_symmetric(
        result.ff_right_mv + result.fb_right_mv, kMaxDriveMillivolts);
    result.left_cmd = millivolts_to_drive_command(left_out_mv);
    result.right_cmd = millivolts_to_drive_command(right_out_mv);
    return result;
  };

  auto controller_result = calculate_tracking_command();
  const auto desired_wheels = limit_tracking_wheel_speeds(controller_result);

  const std::uint32_t now_ms = pros::millis();
  double dt_s = util::DELAY_TIME / 1000.0;
  const auto measured_wheels = update_filtered_wheel_velocity_state(now_ms, dt_s);
  const auto voltage_command = calculate_tracking_voltage_command(
      controller_result, desired_wheels, measured_wheels, dt_s);

  odom_reference_prev_cmd_v = controller_result.linear_inps;
  odom_reference_prev_cmd_omega = controller_result.angular_radps;
  odom_reference_prev_left_err = voltage_command.left_vel_error_mps;
  odom_reference_prev_right_err = voltage_command.right_vel_error_mps;
  odom_reference_last_ms = now_ms;
  odom_reference_feedback_valid = true;

  if (print_toggle && s_ctrl_log_counter++ % 10 == 0) {
    const char* tag = (odom_feedback_type == LTV_FEEDBACK) ? "LTV" : "RAMSETE";
    const float cx = static_cast<float>(current.x);
    const float cy = static_cast<float>(current.y);
    const float ct = static_cast<float>(current.theta);
    const float tx = static_cast<float>(ref.target_pose.x);
    const float ty = static_cast<float>(ref.target_pose.y);
    const float tt = static_cast<float>(ref.target_pose.theta);
    const float v_ref = static_cast<float>(ref.linear_velocity);
    printf(
        "  %s cur:(%.2f, %.2f, %.1f) ref:(%.2f, %.2f, %.1f) "
        "t:%.2f s:%.2f vRef:%.2f wRef:%.4f k:%.4f "
        "err:(fwd %.3f, lat %.3f, hdg %.4f) v:%.2f w:%.4f "
        "wheel:(des %.2f/%.2f, meas %.2f/%.2f) mv:(ff %.0f/%.0f, fb %.0f/%.0f) cmd:(%d, %d)\n",
        tag, cx, cy, ct, tx, ty, tt, ref.time, ref.distance, v_ref,
        static_cast<float>(ref.angular_velocity), static_cast<float>(ref.curvature),
        controller_result.err_fwd, controller_result.err_lat,
        controller_result.err_hdg, controller_result.linear_inps,
        controller_result.angular_radps, desired_wheels.left_inps,
        desired_wheels.right_inps, measured_wheels.left_inps,
        measured_wheels.right_inps, voltage_command.ff_left_mv,
        voltage_command.ff_right_mv, voltage_command.fb_left_mv,
        voltage_command.fb_right_mv, voltage_command.left_cmd,
        voltage_command.right_cmd);
  }

  if (drive_toggle) {
    private_drive_set(voltage_command.left_cmd, voltage_command.right_cmd);
  }

  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());
}

void Drive::boomerang_task() {
  int target_index = pp_index;
  pose target = pp_movements[target_index].target;
  pose current = odom_pose_get();

  int dir = current_drive_direction == REV ? -1 : 1;

  double h = util::distance_to_point(target, current) * odom_boomerang_dlead_get();
  double max = max_boomerang_distance;
  h = h > max ? max : h;
  h *= dir;

  pose temp = util::vector_off_point(-h, pp_movements[target_index].target);
  temp.theta = target.theta;

  if (util::distance_to_point(target, current) < odom_look_ahead_get() / 2.0) {
    temp = target;
  }

  if (print_toggle) {
    bool segment_changed = boomerang_log_index != target_index;
    if (segment_changed) {
      boomerang_log_index = target_index;
      boomerang_log_counter = 0;
    }

    if (segment_changed || boomerang_log_counter++ % 10 == 0) {
      double final_dist = util::distance_to_point(target, current);
      double carrot_dist = util::distance_to_point(temp, current);
      double heading_target = headingPID.target_get();
      double heading_error = util::wrap_angle(heading_target - odom_theta_get());
      printf("  Boomerang Track[%d/%d] cur:(%.2f, %.2f, %.2f) carrot:(%.2f, %.2f, %.2f) final:(%.2f, %.2f, %.2f) dist:(final %.2f, carrot %.2f) h:%.2f heading:(tar %.2f, err %.2f) xy_err:%.2f\n",
             target_index,
             static_cast<int>(pp_movements.size()) - 1,
             current.x,
             current.y,
             current.theta,
             temp.x,
             temp.y,
             temp.theta,
             target.x,
             target.y,
             target.theta,
             final_dist,
             carrot_dist,
             h,
             heading_target,
             heading_error,
             xyPID.error);
    }
  }

  if (odom_target.x != temp.x || odom_target.y != temp.y) {
    bool slew_on = slew_left.enabled() || slew_right.enabled();
    raw_pid_odom_ptp_set({temp, pp_movements[target_index].drive_direction, pp_movements[target_index].max_xy_speed}, slew_on);
  }

  ptp_task();
}

void Drive::pp_task() {
  if (pp_movements.empty()) {
    reset_odom_reference();
    drive_mode_set(DISABLE);
    return;
  }

  if (odom_feedback_type != PID_FEEDBACK) {
    while (pp_index < static_cast<int>(pp_movements.size()) - 1 &&
           fabs(util::distance_to_point(pp_movements[pp_index].target,
                                        odom_pose_get())) <
               odom_look_ahead_get()) {
      pp_index++;
    }

    odom_target = pp_movements[pp_index].target;
    current_drive_direction = pp_movements[pp_index].drive_direction;

    odom_reference_task();
    return;
  }

  if (fabs(util::distance_to_point(pp_movements[pp_index].target, odom_pose_get())) < odom_look_ahead_get()) {
    if (pp_index < pp_movements.size() - 1) {
      pp_index++;
      bool slew_on = current_slew_on && (slew_left.enabled() || slew_right.enabled());
      raw_pid_odom_ptp_set(pp_movements[pp_index], slew_on);
    }
  }

  if (pp_movements[pp_index].target.theta != ANGLE_NOT_SET) {
    boomerang_task();
  } else {
    ptp_task();
  }
}
