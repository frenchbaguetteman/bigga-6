/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "pathing/reference_planner.hpp"
#include "robot_config.hpp"

#include <algorithm>
#include <vector>

using namespace ez;

namespace {

constexpr double kMinReferenceSampleRange = 1e-4;

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
  return RobotConfig::MAX_SPEED_INPS * speed_scale;
}

reference_planner::Waypoint to_reference_waypoint(const odom& movement) {
  return {{movement.target.x, movement.target.y, movement.target.theta},
          to_reference_direction(movement.drive_direction),
          speed_limit_inps(movement.max_xy_speed),
          movement.target.theta != ANGLE_NOT_SET};
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
  config.heading_blend_distance_in = odom_look_ahead_get();
  config.track_width_in = RobotConfig::TRACK_WIDTH_IN;
  config.max_acceleration_inps2 =
      current_slew_on ? RobotConfig::MAX_ACCELERATION_INPS2
                      : RobotConfig::MAX_ACCELERATION_INPS2 * 4.0;
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
