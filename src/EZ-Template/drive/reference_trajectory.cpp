/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "pathing/reference_planner.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <vector>

using namespace ez;

namespace {

constexpr double kMinReferenceSampleRange = 1e-4;
constexpr const char* kOdomTraceFilePath = "/usd/odom_trace.csv";
constexpr std::uint32_t kOdomTraceFlushPeriod = 25;
constexpr double kTrackingHeadingBlendMinIn = 10.0;

reference_planner::DriveDirection to_reference_direction(
    drive_directions direction) {
  return direction == REV ? reference_planner::DriveDirection::Reverse
                          : reference_planner::DriveDirection::Forward;
}

drive_directions to_ez_direction(
    reference_planner::DriveDirection direction) {
  return direction == reference_planner::DriveDirection::Reverse ? REV : FWD;
}

double speed_limit_inps(int max_xy_speed) {
  const double speed_scale =
      std::clamp(static_cast<double>(max_xy_speed) / 127.0, 0.0, 1.0);
  return 76.576321 * speed_scale;
}

reference_planner::Waypoint to_reference_waypoint(const odom& movement) {
  return {{movement.target.x, movement.target.y, movement.target.theta},
          to_reference_direction(movement.drive_direction),
          speed_limit_inps(movement.max_xy_speed),
          movement.target.theta != ANGLE_NOT_SET};
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

const char* odom_mode_name(e_mode mode, std::size_t movement_count) {
  if (mode == PURE_PURSUIT || movement_count > 1) {
    return "PURE_PURSUIT";
  }
  return "POINT_TO_POINT";
}

const char* drive_direction_name(drive_directions direction) {
  return direction == REV ? "REV" : "FWD";
}

double trace_nan() {
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

void Drive::reset_odom_reference() {
  odom_reference_states.clear();
  odom_reference_start_ms = 0;
  odom_reference_last_ms = 0;
  odom_reference_left_vel = 0.0;
  odom_reference_right_vel = 0.0;
  odom_reference_prev_cmd_v = 0.0;
  odom_reference_prev_cmd_omega = 0.0;
  odom_reference_prev_left_err = 0.0;
  odom_reference_prev_right_err = 0.0;
  odom_reference_feedback_valid = false;
  odom_settle_active = false;
  odom_settle_phase = 0;
  odom_settle_start_ms = 0;
  odom_settle_stable_accum_s = 0.0;
  odom_settle_final_target_theta_deg = 0.0;
  odom_settle_locked_bearing_deg = 0.0;
  odom_settle_prev_theta_deg = 0.0;
}

void Drive::begin_odom_reference_trace_segment(pose start_pose,
                                               std::size_t movement_count,
                                               std::size_t state_count) {
  if (odom_feedback_type == PID_FEEDBACK) {
    return;
  }

  if (!ez::util::SD_CARD_ACTIVE) {
    if (!odom_trace_warning_printed && print_toggle) {
      printf("  Full odom trace skipped: no SD card for %s\n",
             kOdomTraceFilePath);
      odom_trace_warning_printed = true;
    }
    return;
  }

  if (!odom_trace_session_active || odom_trace_file == nullptr) {
    const char* open_mode = odom_trace_session_id == 0 ? "w" : "a";
    odom_trace_file = std::fopen(kOdomTraceFilePath, open_mode);
    if (odom_trace_file == nullptr) {
      if (!odom_trace_warning_printed && print_toggle) {
        printf("  Full odom trace failed: couldn't open %s\n",
               kOdomTraceFilePath);
        odom_trace_warning_printed = true;
      }
      return;
    }

    if (odom_trace_session_id == 0) {
      std::fprintf(
          odom_trace_file,
          "row_type,session,segment,controller,mode,direction,now_ms,elapsed_s,"
          "plan_index,plan_count,current_x,current_y,current_theta,ref_x,ref_y,"
          "ref_theta,ref_time_s,ref_distance_in,ref_linear_inps,"
          "ref_angular_radps,ref_linear_accel_inps2,"
          "ref_angular_accel_radps2,ref_curvature_rad_per_in,"
          "translation_error_in,heading_error_deg,controller_err_fwd_in,"
          "controller_err_lat_in,controller_err_hdg_rad,cmd_linear_inps,"
          "cmd_angular_radps,ramsete_k,ramsete_sinc,ltv_k00,ltv_k01,ltv_k02,"
          "ltv_k10,ltv_k11,ltv_k12,ltv_corr_v,ltv_corr_omega,"
          "desired_left_inps,desired_right_inps,measured_left_inps,"
          "measured_right_inps,ff_left_mv,ff_right_mv,fb_left_mv,fb_right_mv,"
          "left_cmd,right_cmd\n");
    }

    ++odom_trace_session_id;
    odom_trace_rows_since_flush = 0;
    odom_trace_session_active = true;

    if (print_toggle) {
      printf("  %s full trace -> %s (session %u)\n",
             odom_feedback_name(odom_feedback_type), kOdomTraceFilePath,
             odom_trace_session_id);
    }
  }

  ++odom_trace_segment_id;

  if (print_toggle) {
    printf("  %s trace segment %u: %zu planned states from (%.2f, %.2f, %.2f)\n",
           odom_feedback_name(odom_feedback_type), odom_trace_segment_id,
           state_count, start_pose.x, start_pose.y, start_pose.theta);
  }
}

void Drive::end_odom_reference_trace(const char* reason) {
  if (odom_trace_file != nullptr) {
    std::fflush(odom_trace_file);
    std::fclose(odom_trace_file);
    odom_trace_file = nullptr;
  }

  if (print_toggle && odom_trace_session_active && reason != nullptr) {
    printf("  Odom trace closed: %s\n", reason);
  }

  odom_trace_segment_id = 0;
  odom_trace_rows_since_flush = 0;
  odom_trace_session_active = false;
}

void Drive::build_odom_reference_states(const std::vector<odom>& imovements,
                                        pose start_pose) {
  reset_odom_reference();

  if (imovements.empty()) {
    return;
  }

  std::vector<reference_planner::Waypoint> waypoints;
  waypoints.reserve(imovements.size());
  for (const auto& movement : imovements) {
    waypoints.push_back(to_reference_waypoint(movement));
  }

  reference_planner::BuildConfig config;
  config.sample_spacing_in = odom_path_spacing_get();
  config.heading_blend_distance_in =
      odom_feedback_get() == PID_FEEDBACK
          ? odom_look_ahead_get()
          : std::max(kTrackingHeadingBlendMinIn,
                     odom_look_ahead_get() * 3.0);
  config.track_width_in = 11.338583;
  config.max_acceleration_inps2 =
      current_slew_on ? 118.11024
                      : 118.11024 * 4.0;
  config.start_pose_has_heading = start_pose.theta != ANGLE_NOT_SET;

  const auto planned_states = reference_planner::build_reference_states(
      waypoints, {start_pose.x, start_pose.y, start_pose.theta}, config);

  const auto to_odom_reference_state =
      [](const reference_planner::State& state) -> odom_reference_state {
    return {{state.target_pose.x, state.target_pose.y, state.target_pose.theta_deg},
            state.linear_velocity,
            state.angular_velocity,
            state.linear_acceleration,
            state.angular_acceleration,
            state.curvature,
            state.time,
            state.distance,
            to_ez_direction(state.direction)};
  };

  odom_reference_states.reserve(planned_states.size());
  for (const auto& state : planned_states) {
    odom_reference_states.push_back(to_odom_reference_state(state));
  }

  begin_odom_reference_trace_segment(start_pose, imovements.size(),
                                     odom_reference_states.size());

  if (odom_trace_file != nullptr) {
    const char* controller_tag = odom_feedback_name(odom_feedback_type);
    const char* mode_tag = odom_mode_name(mode, imovements.size());
    const double nan = trace_nan();
    const std::uint32_t now_ms = pros::millis();

    for (std::size_t i = 0; i < odom_reference_states.size(); ++i) {
      const auto& state = odom_reference_states[i];
      std::fprintf(
          odom_trace_file,
          "PLAN,%u,%u,%s,%s,%s,%u,%.6f,%d,%zu,"
          "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
          "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
          "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n",
          odom_trace_session_id, odom_trace_segment_id, controller_tag,
          mode_tag, drive_direction_name(state.drive_direction), now_ms,
          state.time, static_cast<int>(i),
          odom_reference_states.size(), nan, nan, nan,
          state.target_pose.x, state.target_pose.y, state.target_pose.theta,
          state.time, state.distance, state.linear_velocity,
          state.angular_velocity, state.linear_acceleration,
          state.angular_acceleration, state.curvature, nan, nan, nan, nan,
          nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan,
          nan, nan, nan, nan, nan, nan, nan, nan, -1, -1);

      if (++odom_trace_rows_since_flush % kOdomTraceFlushPeriod == 0) {
        std::fflush(odom_trace_file);
      }
    }
  }

  if (odom_trace_file != nullptr) {
    std::fflush(odom_trace_file);
    odom_trace_rows_since_flush = 0;
  }

  odom_reference_start_ms = pros::millis();
}

void Drive::rebuild_odom_reference_for_current_motion() {
  if (odom_feedback_get() == PID_FEEDBACK) {
    reset_odom_reference();
    return;
  }

  const pose current_pose = odom_pose_get();

  if (mode == PURE_PURSUIT && !pp_movements.empty()) {
    const int safe_index =
        std::clamp(pp_index, 0, static_cast<int>(pp_movements.size()) - 1);
    std::vector<odom> remaining(pp_movements.begin() + safe_index,
                                pp_movements.end());
    build_odom_reference_states(remaining, current_pose);
    return;
  }

  if (mode == POINT_TO_POINT && active_odom_motion.max_xy_speed > 0) {
    build_odom_reference_states({active_odom_motion}, current_pose);
    return;
  }

  reset_odom_reference();
}

Drive::odom_reference_state Drive::sample_odom_reference_state() const {
  if (odom_reference_states.empty()) {
    return {};
  }

  const auto interpolate_reference_state =
      [](const odom_reference_state& lo, const odom_reference_state& hi,
         double elapsed_s) -> odom_reference_state {
    const double range = hi.time - lo.time;
    const double alpha =
        range > kMinReferenceSampleRange ? (elapsed_s - lo.time) / range : 0.0;

    odom_reference_state sample = lo;
    sample.target_pose.x =
        lo.target_pose.x + ((hi.target_pose.x - lo.target_pose.x) * alpha);
    sample.target_pose.y =
        lo.target_pose.y + ((hi.target_pose.y - lo.target_pose.y) * alpha);
    sample.target_pose.theta =
        lo.target_pose.theta +
        (util::wrap_angle(hi.target_pose.theta - lo.target_pose.theta) * alpha);
    sample.linear_velocity =
        lo.linear_velocity + ((hi.linear_velocity - lo.linear_velocity) * alpha);
    sample.angular_velocity =
        lo.angular_velocity +
        ((hi.angular_velocity - lo.angular_velocity) * alpha);
    sample.linear_acceleration =
        lo.linear_acceleration +
        ((hi.linear_acceleration - lo.linear_acceleration) * alpha);
    sample.angular_acceleration =
        lo.angular_acceleration +
        ((hi.angular_acceleration - lo.angular_acceleration) * alpha);
    sample.curvature = lo.curvature + ((hi.curvature - lo.curvature) * alpha);
    sample.time = elapsed_s;
    sample.distance = lo.distance + ((hi.distance - lo.distance) * alpha);
    sample.drive_direction =
        alpha < 0.5 ? lo.drive_direction : hi.drive_direction;
    return sample;
  };

  const double elapsed_s =
      (pros::millis() - odom_reference_start_ms) / 1000.0;

  if (elapsed_s <= 0.0 || odom_reference_states.size() == 1) {
    return odom_reference_states.front();
  }

  if (elapsed_s >= odom_reference_states.back().time) {
    return odom_reference_states.back();
  }

  auto upper = std::upper_bound(
      odom_reference_states.begin(), odom_reference_states.end(), elapsed_s,
      [](double value, const odom_reference_state& state) {
        return value < state.time;
      });

  if (upper == odom_reference_states.begin()) {
    return *upper;
  }

  return interpolate_reference_state(*(upper - 1), *upper, elapsed_s);
}
