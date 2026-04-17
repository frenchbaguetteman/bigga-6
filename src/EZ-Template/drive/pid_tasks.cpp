/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "controllers/ltv_controller.hpp"
#include "controllers/ramsete_controller.hpp"
#include "pros/misc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace ez;

namespace {

constexpr double kMaxDriveMillivolts = 12000.0;
constexpr double kInToM             = 0.0254;
constexpr double kTrackWidthIn      = 11.40;
constexpr double kMaxSpeedInps      = 76.576321;
constexpr double kVelMeasAlpha      = 0.35;
constexpr double kVelKp             = 1300.0;
constexpr double kVelKd             = 0.0;
constexpr double kVelMaxCorrectionMv = 2500.0;
constexpr std::uint32_t kOdomTraceFlushPeriod = 25;
constexpr float  kFfKs              = 1100.0f;
constexpr float  kFfKv              = 6200.0f;
constexpr float  kFfKa              = 400.0f;
constexpr float  kRadToDeg          = 180.0f / 3.14159265f;
constexpr double kDegToRad          = 3.14159265358979323846 / 180.0;

// --- LTV final-pose settler tuning V2 (verified in tools/sim/ltv_sim.py) ---
// Voltage-domain turn-drive-turn recovery with:
//   * Locked phase-1 heading (prevents bearing-chase spinning)
//   * Angular-rate damping (Kd term)
//   * Moderate max voltage (4 V, not 6 V)
//   * Wider phase-1→2 transition (10°, not 2°)
//   * Min kick gated on omega (prevents boost during deceleration)
constexpr double kSettleKpHdgMvPerRad       = 12000.0;
constexpr double kSettleKdMvPerRadps        = 500.0;
constexpr double kSettleKpFwdMvPerIn        = 900.0;
constexpr double kSettleMinTurnKickMv       = 1400.0;
constexpr double kSettleOmegaKickThreshDeg  = 15.0;  // min kick only when |omega| < this
constexpr double kSettleMaxTurnMv           = 4000.0;
constexpr double kSettleMaxFwdMv            = 2500.0;
constexpr double kSettleFwdMinMv            = 1500.0;
constexpr double kSettleHdgDeadbandDeg      = 0.3;
constexpr double kSettleApproachXyTriggerIn = 1.0;
constexpr double kSettlePhase1TolDeg        = 10.0;
constexpr double kSettleXyTolIn             = 0.75;
constexpr double kSettleAngTolDeg           = 1.2;
constexpr double kSettleStableS             = 0.15;
constexpr double kSettleMaxS                = 5.0;
constexpr double kSettleHoldS               = 0.05;

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
      {1.0, std::fabs(left_inps) / kMaxSpeedInps,
       std::fabs(right_inps) / kMaxSpeedInps});
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
    const TrackingControllerResult& controller_result) {
  const double half_track = kTrackWidthIn / 2.0;
  auto [left_inps, right_inps] = limit_wheel_speeds(
      controller_result.linear_inps + (controller_result.angular_radps * half_track),
      controller_result.linear_inps - (controller_result.angular_radps * half_track));
  return {left_inps, right_inps};
}

const char* odom_feedback_name(e_odom_feedback feedback) {
  switch (feedback) {
    case LTV_FEEDBACK:
      return "LTV";
    case RAMSETE_FEEDBACK:
      return "RAMSETE";
    case PID_FEEDBACK:
    default:
      return "PID";
  }
}

const char* odom_mode_name(e_mode mode) {
  switch (mode) {
    case PURE_PURSUIT:
      return "PURE_PURSUIT";
    case POINT_TO_POINT:
      return "POINT_TO_POINT";
    default:
      return "OTHER";
  }
}

const char* drive_direction_name(drive_directions direction) {
  return direction == REV ? "REV" : "FWD";
}

double trace_nan() {
  return std::numeric_limits<double>::quiet_NaN();
}

ltv::Pose2d make_ltv_pose(double ez_x, double ez_y, double ez_theta_deg) {
  return {ez_y, -ez_x, -ez_theta_deg * kDegToRad};
}

}  // namespace

// File-scoped controller instances shared by all odom tasks.
// Q/R tuned for VEX: state tolerances in inches/radians, control effort in
// in/s and rad/s. maxVelocity just above physical max (kMaxSpeedInps ≈ 76.58)
// to keep the gain table compact without clipping.
static LTVUnicycleController s_ltv{
    {0.5, 0.5, 0.1},
    {20.0, 2.0},
    util::DELAY_TIME / 1000.0,
    100.0};
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

  // --- LTV final-pose settler V2 (damped turn-drive-turn) -----------------
  // After the reference trajectory time elapses, the standard LTV+FF
  // pipeline produces commands inside the drivetrain's static-friction
  // deadband near zero velocity.  Engage a voltage-domain controller with:
  //   * Angular-rate damping (Kd) to prevent overshooting turns
  //   * Locked phase-1 heading to prevent bearing-chase spinning
  //   * Moderate max voltage (4 V) to keep angular momentum manageable
  //   * Min kick gated on omega to allow proper deceleration
  if (odom_feedback_type == LTV_FEEDBACK && !odom_reference_states.empty()) {
    const double total_time_s = odom_reference_states.back().time;
    const double elapsed_s =
        (pros::millis() - odom_reference_start_ms) / 1000.0;

    if (elapsed_s >= total_time_s + kSettleHoldS) {
      if (!odom_settle_active) {
        odom_settle_active = true;
        odom_settle_phase = 0;
        odom_settle_start_ms = pros::millis();
        odom_settle_stable_accum_s = 0.0;
        odom_settle_final_target_theta_deg =
            odom_reference_states.back().target_pose.theta;
        odom_settle_locked_bearing_deg = 0.0;
        odom_settle_prev_theta_deg = current.theta;
      }

      const pose final_target = odom_reference_states.back().target_pose;
      const double dx = final_target.x - current.x;
      const double dy = final_target.y - current.y;
      const double xy_err_in = std::hypot(dx, dy);
      // EZ convention: heading 0°=+Y, CW+, so atan2(dx, dy)
      const double angle_to_target_deg =
          util::wrap_angle(std::atan2(dx, dy) * kRadToDeg);
      const double hdg_err_final_deg =
          util::wrap_angle(odom_settle_final_target_theta_deg - current.theta);

      // Angular rate estimation (deg/s)
      const double omega_degps =
          util::wrap_angle(current.theta - odom_settle_prev_theta_deg) /
          (util::DELAY_TIME / 1000.0);

      // --- Phase transitions ---
      // Phase 0 → 1: heading aligned to final, xy still large
      if (odom_settle_phase == 0 &&
          std::fabs(hdg_err_final_deg) < kSettleAngTolDeg * 1.5 &&
          xy_err_in > kSettleApproachXyTriggerIn) {
        odom_settle_phase = 1;
        odom_settle_locked_bearing_deg = angle_to_target_deg;
      }

      // Phase 1 → 2: heading within tolerance of LOCKED bearing
      if (odom_settle_phase == 1) {
        const double err_to_locked =
            util::wrap_angle(odom_settle_locked_bearing_deg - current.theta);
        if (std::fabs(err_to_locked) < kSettlePhase1TolDeg) {
          odom_settle_phase = 2;
        }
      } else if (odom_settle_phase == 2) {
        if (xy_err_in < 0.5) {
          odom_settle_phase = 3;
        }
      } else if (odom_settle_phase == 3) {
        if (std::fabs(hdg_err_final_deg) < kSettleAngTolDeg) {
          odom_settle_phase = 4;
        }
      }

      // Select target heading and forward flag.
      double drive_heading_deg;
      bool want_drive_forward;
      if (odom_settle_phase == 1) {
        drive_heading_deg = odom_settle_locked_bearing_deg;  // LOCKED
        want_drive_forward = false;
      } else if (odom_settle_phase == 2) {
        drive_heading_deg = angle_to_target_deg;  // live bearing OK when close
        want_drive_forward = true;
      } else {
        drive_heading_deg = odom_settle_final_target_theta_deg;
        want_drive_forward = false;
      }

      // --- P + D heading controller ---
      const double err_hdg_drive_deg =
          util::wrap_angle(drive_heading_deg - current.theta);
      const double err_hdg_drive_rad = err_hdg_drive_deg * kDegToRad;
      const double omega_radps = omega_degps * kDegToRad;

      // P term
      double v_p = kSettleKpHdgMvPerRad * err_hdg_drive_rad;
      // D term (angular rate damping — always active)
      const double v_d = -kSettleKdMvPerRadps * omega_radps;

      // Static friction kick: only when nearly stationary (prevents boosting
      // during deceleration which caused the V1 spinning)
      if (std::fabs(err_hdg_drive_deg) > kSettleHdgDeadbandDeg &&
          std::fabs(omega_degps) < kSettleOmegaKickThreshDeg) {
        if (std::fabs(v_p) < kSettleMinTurnKickMv) {
          v_p = std::copysign(kSettleMinTurnKickMv, v_p);
        }
      } else if (std::fabs(err_hdg_drive_deg) <= kSettleHdgDeadbandDeg) {
        v_p = 0.0;  // deadband suppresses P
      }

      double v_turn_mv = clamp_symmetric(v_p + v_d, kSettleMaxTurnMv);

      // Forward drive (only in phase 2, when heading is roughly aligned)
      double v_fwd_mv = 0.0;
      if (want_drive_forward && std::fabs(err_hdg_drive_deg) < 20.0) {
        v_fwd_mv = kSettleKpFwdMvPerIn * xy_err_in;
        v_fwd_mv = std::copysign(
            std::max(std::fabs(v_fwd_mv), kSettleFwdMinMv), 1.0);
        v_fwd_mv = std::min(v_fwd_mv, kSettleMaxFwdMv);
      }

      // Stability accumulator
      const double dt_s = util::DELAY_TIME / 1000.0;
      if (xy_err_in < kSettleXyTolIn &&
          std::fabs(hdg_err_final_deg) < kSettleAngTolDeg) {
        odom_settle_stable_accum_s += dt_s;
      } else {
        odom_settle_stable_accum_s = 0.0;
      }

      const double settle_elapsed_s =
          (pros::millis() - odom_settle_start_ms) / 1000.0;
      const bool settle_done =
          odom_settle_stable_accum_s >= kSettleStableS ||
          settle_elapsed_s >= kSettleMaxS;

      double left_out_mv = v_fwd_mv + v_turn_mv;
      double right_out_mv = v_fwd_mv - v_turn_mv;
      if (settle_done) {
        left_out_mv = 0.0;
        right_out_mv = 0.0;
        odom_settle_phase = 4;
      }

      const int left_cmd = millivolts_to_drive_command(left_out_mv);
      const int right_cmd = millivolts_to_drive_command(right_out_mv);

      if (print_toggle && s_ctrl_log_counter++ % 10 == 0) {
        printf(
            "  LTV-settle ph:%d cur:(%.2f, %.2f, %.1f) tgt:(%.2f, %.2f, %.1f) "
            "xy:%.3f hdg:%.2f w:%.1f mv:(L %.0f, R %.0f) stable:%.2fs%s\n",
            odom_settle_phase, current.x, current.y, current.theta,
            final_target.x, final_target.y,
            odom_settle_final_target_theta_deg, xy_err_in, hdg_err_final_deg,
            omega_degps, left_out_mv, right_out_mv,
            odom_settle_stable_accum_s, settle_done ? " DONE" : "");
      }

      current_drive_direction = ref.drive_direction;
      odom_target = final_target;
      odom_settle_prev_theta_deg = current.theta;
      odom_reference_last_ms = pros::millis();

      if (drive_toggle) {
        private_drive_set(left_cmd, right_cmd);
      }

      leftPID.compute(drive_sensor_left());
      rightPID.compute(drive_sensor_right());
      return;
    }
  }
  // ------------------------------------------------------------------------

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
    TrackingControllerResult result;
    if (odom_feedback_type == LTV_FEEDBACK) {
      const auto current_pose = make_ltv_pose(current.x, current.y, current.theta);
      const auto ref_pose =
          make_ltv_pose(ref.target_pose.x, ref.target_pose.y, ref.target_pose.theta);
      // make_ltv_pose() flips to LTV's CCW-positive convention; angular
      // velocity and omega output must be flipped to match.
      const auto cmd = s_ltv.Calculate(current_pose, ref_pose, ref.linear_velocity,
                                       -ref.angular_velocity);
      const auto& pose_error = s_ltv.PoseError();

      result.err_fwd = static_cast<float>(pose_error.X());
      result.err_lat = static_cast<float>(-pose_error.Y());
      result.err_hdg = static_cast<float>(-pose_error.Rotation().Radians());
      result.linear_inps = cmd.vx;
      result.angular_radps = -cmd.omega;
      return result;
    }

    const float cx = static_cast<float>(current.x);
    const float cy = static_cast<float>(current.y);
    const float ct = static_cast<float>(current.theta);
    const float tx = static_cast<float>(ref.target_pose.x);
    const float ty = static_cast<float>(ref.target_pose.y);
    const float tt = static_cast<float>(ref.target_pose.theta);
    const float v_ref = static_cast<float>(ref.linear_velocity);
    const float omega_ref_deg =
        static_cast<float>(ref.angular_velocity * kRadToDeg);
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
      constexpr double kAlpha = kVelMeasAlpha;
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

    // Inline drivetrain feedforward
    auto ffOneSide = [](float vel, float acc) -> float {
      const float sign = (vel > 0.0f) ? 1.0f : ((vel < 0.0f) ? -1.0f : 0.0f);
      return kFfKs * sign + kFfKv * vel + kFfKa * acc;
    };
    const float halfTrackM = static_cast<float>(kTrackWidthIn * kInToM) / 2.0f;
    const float vMps   = static_cast<float>(controller_result.linear_inps * kInToM);
    const float omRad  = static_cast<float>(controller_result.angular_radps);
    const float aMps2  = static_cast<float>(cmd_linear_accel * kInToM);
    const float alRad2 = static_cast<float>(cmd_angular_accel);
    const float ff_left_mv  = ffOneSide(vMps + omRad * halfTrackM, aMps2 + alRad2 * halfTrackM);
    const float ff_right_mv = ffOneSide(vMps - omRad * halfTrackM, aMps2 - alRad2 * halfTrackM);

    const double desired_left_mps =
        desired_wheels.left_inps * kInToM;
    const double desired_right_mps =
        desired_wheels.right_inps * kInToM;
    const double measured_left_mps =
        measured_wheels.left_inps * kInToM;
    const double measured_right_mps =
        measured_wheels.right_inps * kInToM;

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
        (kVelKp * result.left_vel_error_mps) +
            (kVelKd * left_error_rate),
        kVelMaxCorrectionMv);
    result.fb_right_mv = clamp_symmetric(
        (kVelKp * result.right_vel_error_mps) +
            (kVelKd * right_error_rate),
        kVelMaxCorrectionMv);

    const double left_out_mv = clamp_symmetric(
        result.ff_left_mv + result.fb_left_mv, kMaxDriveMillivolts);
    const double right_out_mv = clamp_symmetric(
        result.ff_right_mv + result.fb_right_mv, kMaxDriveMillivolts);
    result.left_cmd = millivolts_to_drive_command(left_out_mv);
    result.right_cmd = millivolts_to_drive_command(right_out_mv);
    return result;
  };

  auto controller_result = calculate_tracking_command();
  const std::uint32_t now_ms = pros::millis();
  auto limited_controller_result = controller_result;
  const auto desired_wheels = limit_tracking_wheel_speeds(limited_controller_result);

  double dt_s = util::DELAY_TIME / 1000.0;
  const auto measured_wheels = update_filtered_wheel_velocity_state(now_ms, dt_s);
  const auto voltage_command = calculate_tracking_voltage_command(
      limited_controller_result, desired_wheels, measured_wheels, dt_s);

  odom_reference_prev_cmd_v = limited_controller_result.linear_inps;
  odom_reference_prev_cmd_omega = limited_controller_result.angular_radps;
  odom_reference_prev_left_err = voltage_command.left_vel_error_mps;
  odom_reference_prev_right_err = voltage_command.right_vel_error_mps;
  odom_reference_last_ms = now_ms;
  odom_reference_feedback_valid = true;

  const char* controller_tag = odom_feedback_name(odom_feedback_type);
  const char* mode_tag = odom_mode_name(mode);
  const char* direction_tag = drive_direction_name(ref.drive_direction);
  const double trace_elapsed_s =
      (now_ms - odom_reference_start_ms) / 1000.0;
  const double nan = trace_nan();

  double ramsete_k = nan;
  double ramsete_sinc = nan;
  double ltv_k00 = nan;
  double ltv_k01 = nan;
  double ltv_k02 = nan;
  double ltv_k10 = nan;
  double ltv_k11 = nan;
  double ltv_k12 = nan;
  double ltv_corr_v = nan;
  double ltv_corr_omega = nan;

  if (odom_feedback_type != LTV_FEEDBACK) {
    ramsete_k = s_ramsete.lastK();
    ramsete_sinc = s_ramsete.lastSinc();
  }

  if (odom_trace_file != nullptr) {
    std::fprintf(
        odom_trace_file,
        "TRACE,%u,%u,%s,%s,%s,%u,%.6f,%d,%zu,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n",
        odom_trace_session_id, odom_trace_segment_id, controller_tag,
        mode_tag, direction_tag, now_ms, trace_elapsed_s, -1,
        odom_reference_states.size(), current.x, current.y, current.theta,
        ref.target_pose.x, ref.target_pose.y, ref.target_pose.theta, ref.time,
        ref.distance, ref.linear_velocity, ref.angular_velocity,
        ref.linear_acceleration, ref.angular_acceleration, ref.curvature,
        translation_error, heading_error, controller_result.err_fwd,
        controller_result.err_lat, controller_result.err_hdg,
        controller_result.linear_inps, controller_result.angular_radps,
        ramsete_k, ramsete_sinc, ltv_k00, ltv_k01, ltv_k02, ltv_k10, ltv_k11,
        ltv_k12, ltv_corr_v, ltv_corr_omega, desired_wheels.left_inps,
        desired_wheels.right_inps, measured_wheels.left_inps,
        measured_wheels.right_inps, voltage_command.ff_left_mv,
        voltage_command.ff_right_mv, voltage_command.fb_left_mv,
        voltage_command.fb_right_mv, voltage_command.left_cmd,
        voltage_command.right_cmd);

    if (++odom_trace_rows_since_flush % kOdomTraceFlushPeriod == 0) {
      std::fflush(odom_trace_file);
    }
  }

  if (print_toggle && s_ctrl_log_counter++ % 10 == 0) {
    const float cx = static_cast<float>(current.x);
    const float cy = static_cast<float>(current.y);
    const float ct = static_cast<float>(current.theta);
    const float tx = static_cast<float>(ref.target_pose.x);
    const float ty = static_cast<float>(ref.target_pose.y);
    const float tt = static_cast<float>(ref.target_pose.theta);
    const float v_ref = static_cast<float>(ref.linear_velocity);
    if (odom_feedback_type == LTV_FEEDBACK) {
      printf(
          "  %s cur:(%.2f, %.2f, %.1f) ref:(%.2f, %.2f, %.1f) "
          "t:%.2f s:%.2f curv:%.4f vRef:%.2f wRef:%.4f "
          "err:(fwd %.3f, lat %.3f, hdg %.4f) cmd:(v %.2f, w %.4f) "
          "wheel:(des %.2f/%.2f, meas %.2f/%.2f) "
          "mv:(ff %.0f/%.0f, fb %.0f/%.0f) cmd:(%d, %d)\n",
          controller_tag, cx, cy, ct, tx, ty, tt, ref.time, ref.distance,
          static_cast<float>(ref.curvature), v_ref,
          static_cast<float>(ref.angular_velocity), controller_result.err_fwd,
          controller_result.err_lat, controller_result.err_hdg,
          controller_result.linear_inps, controller_result.angular_radps,
          desired_wheels.left_inps, desired_wheels.right_inps,
          measured_wheels.left_inps, measured_wheels.right_inps,
          voltage_command.ff_left_mv, voltage_command.ff_right_mv,
          voltage_command.fb_left_mv, voltage_command.fb_right_mv,
          voltage_command.left_cmd, voltage_command.right_cmd);
    } else {
      printf(
          "  %s cur:(%.2f, %.2f, %.1f) ref:(%.2f, %.2f, %.1f) "
          "t:%.2f s:%.2f curv:%.4f vRef:%.2f wRef:%.4f "
          "err:(fwd %.3f, lat %.3f, hdg %.4f) cmd:(v %.2f, w %.4f) "
          "ram:(k %.4f, sinc %.4f) wheel:(des %.2f/%.2f, meas %.2f/%.2f) "
          "mv:(ff %.0f/%.0f, fb %.0f/%.0f) cmd:(%d, %d)\n",
          controller_tag, cx, cy, ct, tx, ty, tt, ref.time, ref.distance,
          static_cast<float>(ref.curvature), v_ref,
          static_cast<float>(ref.angular_velocity), controller_result.err_fwd,
          controller_result.err_lat, controller_result.err_hdg,
          controller_result.linear_inps, controller_result.angular_radps,
          static_cast<float>(ramsete_k), static_cast<float>(ramsete_sinc),
          desired_wheels.left_inps, desired_wheels.right_inps,
          measured_wheels.left_inps, measured_wheels.right_inps,
          voltage_command.ff_left_mv, voltage_command.ff_right_mv,
          voltage_command.fb_left_mv, voltage_command.fb_right_mv,
          voltage_command.left_cmd, voltage_command.right_cmd);
    }
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
