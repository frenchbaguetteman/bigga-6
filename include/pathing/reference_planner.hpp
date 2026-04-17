#pragma once


#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace reference_planner {

enum class DriveDirection { Forward = 0, Reverse = 1 };

struct Pose {
  double x = 0.0;
  double y = 0.0;
  double theta_deg = 0.0;
};

struct Waypoint {
  Pose target{};
  DriveDirection direction = DriveDirection::Forward;
  double max_velocity_inps = 0.0;
  bool has_explicit_heading = false;
};

struct State {
  Pose target_pose{};
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  double linear_acceleration = 0.0;
  double angular_acceleration = 0.0;
  double curvature = 0.0;
  double time = 0.0;
  double distance = 0.0;
  DriveDirection direction = DriveDirection::Forward;
};

struct BuildConfig {
  double sample_spacing_in = 0.5;
  double heading_blend_distance_in = 7.0;
  double track_width_in = 11.338583;
  double max_acceleration_inps2 = 118.11024;
  bool start_pose_has_heading = true;
};

namespace detail {

inline constexpr double kMinPointSeparationIn = 1e-4;
inline constexpr double kMinVelocityEpsilon = 1e-4;
inline constexpr double kMinSpacingIn = 0.25;
inline constexpr double kMaxSpacingIn = 1.0;
inline constexpr double kHeadingBlendFloorIn = 6.0;

inline double wrap_angle_deg(double theta_deg) {
  while (theta_deg > 180.0) theta_deg -= 360.0;
  while (theta_deg < -180.0) theta_deg += 360.0;
  return theta_deg;
}

inline double distance_to_point(const Pose& target, const Pose& current) {
  return std::hypot(target.x - current.x, target.y - current.y);
}

inline double absolute_angle_to_point(const Pose& target, const Pose& current) {
  return std::atan2(target.x - current.x, target.y - current.y) *
         (180.0 / M_PI);
}

inline double lerp(double a, double b, double t) {
  return a + ((b - a) * t);
}

inline double lerp_angle_deg(double a, double b, double t) {
  return a + (wrap_angle_deg(b - a) * t);
}

inline double segment_heading_deg(const Pose& start, const Pose& end,
                                  DriveDirection dir) {
  double heading = absolute_angle_to_point(end, start);
  if (dir == DriveDirection::Reverse) {
    heading = wrap_angle_deg(heading + 180.0);
  }
  return heading;
}

inline double wheel_limited_velocity(double linear_limit_inps,
                                     double curvature_rad_per_in,
                                     double track_width_in) {
  const double half_track = track_width_in / 2.0;
  const double left_scale =
      std::fabs(1.0 + (curvature_rad_per_in * half_track));
  const double right_scale =
      std::fabs(1.0 - (curvature_rad_per_in * half_track));
  return linear_limit_inps / std::max({1.0, left_scale, right_scale});
}

}  // namespace detail

inline std::vector<State> build_reference_states(
    const std::vector<Waypoint>& input_waypoints, const Pose& start_pose,
    const BuildConfig& config = BuildConfig{}) {
  using namespace detail;

  std::vector<State> states;
  if (input_waypoints.empty()) {
    return states;
  }

  std::vector<Waypoint> waypoints;
  waypoints.reserve(input_waypoints.size() + 1);
  waypoints.push_back({start_pose, input_waypoints.front().direction,
                       input_waypoints.front().max_velocity_inps,
                       config.start_pose_has_heading});

  for (const auto& waypoint : input_waypoints) {
    if (!waypoints.empty() &&
        distance_to_point(waypoints.back().target, waypoint.target) <
            kMinPointSeparationIn) {
      waypoints.back().direction = waypoint.direction;
      waypoints.back().max_velocity_inps = waypoint.max_velocity_inps;
      if (waypoint.has_explicit_heading) {
        waypoints.back().target.theta_deg = waypoint.target.theta_deg;
        waypoints.back().has_explicit_heading = true;
      }
      continue;
    }

    waypoints.push_back(waypoint);
  }

  if (waypoints.size() == 1) {
    states.push_back({{start_pose.x, start_pose.y, start_pose.theta_deg},
                      0.0,
                      0.0,
                      0.0,
                      0.0,
                      0.0,
                      0.0,
                      0.0,
                      input_waypoints.front().direction});
    return states;
  }

  const double sample_spacing =
      std::clamp(config.sample_spacing_in, kMinSpacingIn, kMaxSpacingIn);
  const double heading_blend_distance =
      std::max(kHeadingBlendFloorIn, config.heading_blend_distance_in);
  const double max_accel_inps2 =
      std::max(kMinVelocityEpsilon, std::fabs(config.max_acceleration_inps2));

  std::vector<double> speed_limits;
  std::vector<int> direction_signs;

  double initial_heading = start_pose.theta_deg;
  if (!config.start_pose_has_heading) {
    initial_heading = segment_heading_deg(waypoints[0].target, waypoints[1].target,
                                          waypoints[1].direction);
  }

  states.push_back({{start_pose.x, start_pose.y, initial_heading},
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    waypoints[1].direction});
  speed_limits.push_back(waypoints[1].max_velocity_inps);
  direction_signs.push_back(waypoints[1].direction == DriveDirection::Reverse
                                ? -1
                                : 1);

  double cumulative_distance = 0.0;

  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const auto& start = waypoints[i];
    const auto& end = waypoints[i + 1];
    const double segment_length = distance_to_point(end.target, start.target);

    if (segment_length < kMinPointSeparationIn) {
      continue;
    }

    const int steps =
        std::max(1, static_cast<int>(std::ceil(segment_length / sample_spacing)));
    const double tangent_heading =
        segment_heading_deg(start.target, end.target, end.direction);
    const double start_heading =
        start.has_explicit_heading ? start.target.theta_deg : tangent_heading;
    const double end_heading =
        end.has_explicit_heading ? end.target.theta_deg : tangent_heading;
    const double blend_distance =
        std::min(segment_length, heading_blend_distance);

    for (int step = 1; step <= steps; ++step) {
      const double local_distance =
          segment_length * static_cast<double>(step) / steps;
      const double alpha = local_distance / segment_length;

      Pose sample{
          lerp(start.target.x, end.target.x, alpha),
          lerp(start.target.y, end.target.y, alpha),
          tangent_heading,
      };

      if (start.has_explicit_heading && local_distance < blend_distance) {
        const double blend = 1.0 - (local_distance / blend_distance);
        sample.theta_deg = lerp_angle_deg(sample.theta_deg, start_heading, blend);
      }

      if (end.has_explicit_heading &&
          (segment_length - local_distance) < blend_distance) {
        const double blend =
            1.0 - ((segment_length - local_distance) / blend_distance);
        sample.theta_deg = lerp_angle_deg(sample.theta_deg, end_heading, blend);
      }

      cumulative_distance += distance_to_point(sample, states.back().target_pose);
      states.push_back({sample,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        cumulative_distance,
                        end.direction});
      speed_limits.push_back(end.max_velocity_inps);
      direction_signs.push_back(end.direction == DriveDirection::Reverse ? -1 : 1);
    }
  }

  if (states.size() == 1) {
    return states;
  }

  for (std::size_t i = 0; i < states.size(); ++i) {
    const std::size_t lo = (i == 0) ? 0 : i - 1;
    const std::size_t hi = std::min(i + 1, states.size() - 1);
    const double distance_delta = states[hi].distance - states[lo].distance;

    double curvature = 0.0;
    if (distance_delta > kMinPointSeparationIn) {
      const double heading_delta =
          wrap_angle_deg(states[hi].target_pose.theta_deg -
                         states[lo].target_pose.theta_deg) *
          (M_PI / 180.0);
      curvature = heading_delta / distance_delta;
    }

    states[i].curvature = curvature;
    speed_limits[i] = wheel_limited_velocity(speed_limits[i], curvature,
                                             config.track_width_in);
  }

  std::vector<double> velocity_profile(states.size(), 0.0);
  std::size_t block_start = 0;

  while (block_start < states.size()) {
    std::size_t block_end = block_start;
    while (block_end + 1 < states.size() &&
           direction_signs[block_end + 1] == direction_signs[block_start]) {
      ++block_end;
    }

    velocity_profile[block_start] = 0.0;
    for (std::size_t i = block_start + 1; i <= block_end; ++i) {
      const double ds = states[i].distance - states[i - 1].distance;
      const double accel_limit = std::sqrt(
          std::max(0.0, velocity_profile[i - 1] * velocity_profile[i - 1] +
                            (2.0 * max_accel_inps2 * ds)));
      velocity_profile[i] = std::min(speed_limits[i], accel_limit);
    }

    velocity_profile[block_end] = 0.0;
    for (std::size_t i = block_end; i-- > block_start;) {
      const double ds = states[i + 1].distance - states[i].distance;
      const double accel_limit = std::sqrt(
          std::max(0.0, velocity_profile[i + 1] * velocity_profile[i + 1] +
                            (2.0 * max_accel_inps2 * ds)));
      velocity_profile[i] = std::min(velocity_profile[i], accel_limit);
    }

    const double sign = static_cast<double>(direction_signs[block_start]);
    for (std::size_t i = block_start; i <= block_end; ++i) {
      states[i].linear_velocity = velocity_profile[i] * sign;
      states[i].angular_velocity =
          states[i].linear_velocity * states[i].curvature;
    }

    block_start = block_end + 1;
  }

  states.front().time = 0.0;
  for (std::size_t i = 1; i < states.size(); ++i) {
    const double ds = states[i].distance - states[i - 1].distance;
    const double v0 = std::fabs(states[i - 1].linear_velocity);
    const double v1 = std::fabs(states[i].linear_velocity);
    const double velocity_sum = v0 + v1;
    const double dt = velocity_sum > kMinVelocityEpsilon
                          ? (2.0 * ds / velocity_sum)
                          : (ds / std::max(speed_limits[i], 1.0));
    states[i].time = states[i - 1].time + dt;
  }

  // Recompute angular_velocity from the actual heading/time finite difference.
  // v·κ underspecifies ω during deceleration near an endpoint whose final
  // heading was added by the blend: ω vanishes while heading is still
  // sweeping, which leaves controllers without a feedforward (and zeroes
  // RAMSETE's gain k).
  for (std::size_t i = 0; i < states.size(); ++i) {
    const std::size_t lo = (i == 0) ? 0 : i - 1;
    const std::size_t hi = std::min(i + 1, states.size() - 1);
    const double dt = states[hi].time - states[lo].time;
    if (dt > kMinVelocityEpsilon) {
      const double dtheta_rad =
          wrap_angle_deg(states[hi].target_pose.theta_deg -
                         states[lo].target_pose.theta_deg) *
          (M_PI / 180.0);
      states[i].angular_velocity = dtheta_rad / dt;
    }
  }

  for (std::size_t i = 0; i < states.size(); ++i) {
    const std::size_t lo = (i == 0) ? 0 : i - 1;
    const std::size_t hi = std::min(i + 1, states.size() - 1);
    const double dt = states[hi].time - states[lo].time;

    if (dt > kMinVelocityEpsilon) {
      states[i].linear_acceleration =
          (states[hi].linear_velocity - states[lo].linear_velocity) / dt;
      states[i].angular_acceleration =
          (states[hi].angular_velocity - states[lo].angular_velocity) / dt;
    }
  }

  return states;
}

inline State sample_reference_state(const std::vector<State>& states,
                                    double elapsed_s) {
  using namespace detail;

  if (states.empty()) {
    return {};
  }

  if (elapsed_s <= 0.0 || states.size() == 1) {
    return states.front();
  }

  if (elapsed_s >= states.back().time) {
    return states.back();
  }

  auto upper = std::upper_bound(
      states.begin(), states.end(), elapsed_s,
      [](double value, const State& state) { return value < state.time; });

  if (upper == states.begin()) {
    return *upper;
  }

  const auto& hi = *upper;
  const auto& lo = *(upper - 1);
  const double range = hi.time - lo.time;
  const double alpha = range > kMinVelocityEpsilon
                           ? (elapsed_s - lo.time) / range
                           : 0.0;

  State sample = lo;
  sample.target_pose.x = lerp(lo.target_pose.x, hi.target_pose.x, alpha);
  sample.target_pose.y = lerp(lo.target_pose.y, hi.target_pose.y, alpha);
  sample.target_pose.theta_deg =
      lerp_angle_deg(lo.target_pose.theta_deg, hi.target_pose.theta_deg, alpha);
  sample.linear_velocity = lerp(lo.linear_velocity, hi.linear_velocity, alpha);
  sample.angular_velocity =
      lerp(lo.angular_velocity, hi.angular_velocity, alpha);
  sample.linear_acceleration =
      lerp(lo.linear_acceleration, hi.linear_acceleration, alpha);
  sample.angular_acceleration =
      lerp(lo.angular_acceleration, hi.angular_acceleration, alpha);
  sample.curvature = lerp(lo.curvature, hi.curvature, alpha);
  sample.time = elapsed_s;
  sample.distance = lerp(lo.distance, hi.distance, alpha);
  sample.direction = alpha < 0.5 ? lo.direction : hi.direction;
  return sample;
}

}  // namespace reference_planner
