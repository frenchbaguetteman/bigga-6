#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

namespace RobotConfig {

// ── Shared conversion helpers (ported from bigga4 config.h) ─────────────────
inline constexpr float IN_TO_M    = 0.0254f;
inline constexpr float INCH_TO_M  = IN_TO_M;
inline constexpr float CM_TO_IN   = 1.0f / 2.54f;
inline constexpr float PI_F       = 3.14159265358979323846f;
inline constexpr float DEG_TO_RAD = PI_F / 180.0f;
inline constexpr float RAD_TO_DEG = 180.0f / PI_F;

inline float wrapAngleRadians(float angleRad) {
  return std::atan2(std::sin(angleRad), std::cos(angleRad));
}

inline float wrapDegreesSigned(float degrees) {
  while (degrees <= -180.0f) degrees += 360.0f;
  while (degrees > 180.0f) degrees -= 360.0f;
  return degrees;
}

inline float wrapDegreesPositive(float degrees) {
  while (degrees < 0.0f) degrees += 360.0f;
  while (degrees >= 360.0f) degrees -= 360.0f;
  return degrees;
}

inline float gpsHeadingDegToInternalRad(float compassDeg) {
  return wrapDegreesSigned(90.0f - compassDeg) * DEG_TO_RAD;
}

inline float internalRadToGpsHeadingDeg(float internalRad) {
  const float internalDeg = wrapDegreesSigned(internalRad * RAD_TO_DEG);
  return wrapDegreesPositive(90.0f - internalDeg);
}

// ── Modes ────────────────────────────────────────────────────────────────────
enum class StartupPoseMode {
  ConfiguredStartPoseOnly,
  GPSXYPlusIMUHeading,
  FullGPSInit,
};

enum class LocalizationFusionMode {
  Local,
  Global,
};

// ── Physical geometry ────────────────────────────────────────────────────────
inline constexpr float DRIVE_RADIUS_IN = 1.625f;
inline constexpr float ODOM_RADIUS_IN  = 1.375f;
inline constexpr float TRACK_WIDTH_IN  = 11.338583f;
inline constexpr float WHEEL_BASE_IN   = 10.15748f;
inline constexpr float DRIVE_MOTOR_REVS_PER_WHEEL_REV = 4.0f / 3.0f;
inline constexpr float DRIVE_WHEEL_REVS_PER_MOTOR_REV = 1.0f / DRIVE_MOTOR_REVS_PER_WHEEL_REV;

// ── Noise model ──────────────────────────────────────────────────────────────
inline constexpr float DRIVE_NOISE_IN   = 0.15748f;
inline constexpr float ANGLE_NOISE_DEG  = 0.572958f;

// ── Drivetrain / sensor ports from bigga4 ───────────────────────────────────
inline constexpr std::array<std::int8_t, 3> LEFT_DRIVE_PORTS  = {-11, -15, -14};
inline constexpr std::array<std::int8_t, 3> RIGHT_DRIVE_PORTS = { 10,  17,  20};
inline constexpr std::array<std::int8_t, 2> INTAKE_PORTS      = {-6, 8};

inline constexpr int IMU_PORT = 13;

inline constexpr int  VERTICAL_TRACKING_PORT       = 0;
inline constexpr int  HORIZONTAL_TRACKING_PORT     = 16;
inline constexpr bool VERTICAL_TRACKING_REVERSED   = false;
inline constexpr bool HORIZONTAL_TRACKING_REVERSED = false;
inline constexpr float HORIZONTAL_TRACKING_WHEEL_DIAMETER_IN = 2.0f;
inline constexpr float LATERAL_WHEEL_OFFSET_IN     = -1.771654f;

// ── GPS / localization ───────────────────────────────────────────────────────
inline constexpr int   GPS_PORT = 3;
inline constexpr float GPS_HEADING_OFFSET_DEG = 90.0f;
inline constexpr float GPS_FIELD_ROTATION_DEG = -90.0f;

inline constexpr float GPS_OFFSET_X_IN = 0.0f;
inline constexpr float GPS_OFFSET_Y_IN = 0.0f;

inline constexpr float GPS_MAX_ERROR_M          = 0.50f;
inline constexpr float GPS_RUNTIME_MAX_ERROR_M  = 0.25f;
inline constexpr float GPS_CORRECTION_MAX_IN    = 36.0f;
inline constexpr float GPS_CORRECTION_STEP_IN   = 0.25f;
inline constexpr int   GPS_STABLE_SAMPLES       = 6;
inline constexpr std::uint32_t GPS_MAX_WAIT_MS  = 10000;

inline constexpr StartupPoseMode STARTUP_POSE_MODE = StartupPoseMode::GPSXYPlusIMUHeading;
inline constexpr float DEFAULT_IMU_INIT_ANGLE_DEG  = 180.0f;
inline constexpr float START_POSE_X_IN             = 0.0f;
inline constexpr float START_POSE_Y_IN             = 0.0f;
inline constexpr float START_POSE_THETA_DEG        = 180.0f;

// ── Driver-control shaping / active brake ───────────────────────────────────
inline constexpr float DRIVER_JOYSTICK_DEADBAND             = 5.0f;
inline constexpr float DRIVER_FORWARD_CURVE_T               = 5.0f;
inline constexpr float DRIVER_TURN_CURVE_T                  = 5.0f;
inline constexpr bool  DRIVER_ACTIVE_BRAKE_ENABLED          = true;
inline constexpr float DRIVER_ACTIVE_BRAKE_POWER            = 1.5f;
inline constexpr float DRIVER_ACTIVE_BRAKE_KP               = 0.035f;
inline constexpr float DRIVER_ACTIVE_BRAKE_STICK_DEADBAND   = 6.0f;
inline constexpr float DRIVER_ACTIVE_BRAKE_POS_DEADBAND_DEG = 6.0f;
inline constexpr float DRIVER_ACTIVE_BRAKE_OUTPUT_DEADBAND  = 2.0f;
inline constexpr std::uint32_t DRIVER_ACTIVE_BRAKE_DELAY_MS = 120;

// ── Pneumatic (ADI) ports ───────────────────────────────────────────────────
inline constexpr char TOP_PORT    = 'A';
inline constexpr char SELECT_PORT = 'B';
inline constexpr char TONGUE_PORT = 'C';
inline constexpr char WING_PORT   = 'D';

// ── Localization fusion controls ─────────────────────────────────────────────
inline constexpr bool  MCL_ENABLED                         = true;
inline constexpr int   NUM_PARTICLES                       = 500;
inline constexpr float FIELD_HALF_SIZE_IN                  = 70.2f;
inline constexpr float MAX_DISTANCE_SINCE_UPDATE_IN        = 0.787402f;
inline constexpr int   MAX_UPDATE_INTERVAL_MS              = 50;
inline constexpr float PF_STATIONARY_DEADBAND_IN           = 0.118110f;

inline constexpr float LOC_FUSION_STILLNESS_DEADBAND_IN    = 0.20f;
inline constexpr float LOC_GPS_RUNTIME_ERROR_MAX_IN        = 8.0f;
inline constexpr float LOC_GPS_CORRECTION_MAX_IN           = 36.0f;
inline constexpr float LOC_GPS_CORRECTION_STEP_IN          = 0.25f;
inline constexpr float LOC_GPS_CORRECTION_DEADBAND_IN      = 0.12f;
inline constexpr float LOC_GPS_STABILITY_WINDOW_IN         = 2.0f;
inline constexpr int   LOC_GPS_STABLE_SAMPLE_COUNT         = 4;
inline constexpr float LOC_LOCAL_MODE_CORRECTION_WINDOW_IN = 6.0f;
inline constexpr int   LOC_MCL_MIN_ACTIVE_SENSORS          = 3;
inline constexpr float LOC_MCL_CORRECTION_MAX_IN           = 18.0f;
inline constexpr float LOC_MCL_CORRECTION_STEP_IN          = 0.04f;
inline constexpr float LOC_MCL_CORRECTION_DEADBAND_IN      = 0.10f;
inline constexpr float LOC_MCL_CORRECTION_JUMP_REJECT_IN   = 3.5f;
inline constexpr float LOC_MCL_MIN_ESS_RATIO               = 0.22f;
inline constexpr LocalizationFusionMode LOC_FUSION_DEFAULT_MODE = LocalizationFusionMode::Local;

// ── Speed / acceleration limits ──────────────────────────────────────────────
inline constexpr float MAX_SPEED_INPS         = 76.576321f;
inline constexpr float MAX_ACCELERATION_INPS2 = 118.11024f;
inline constexpr float MAX_ANGULAR_VEL_DEGPS  = 572.9578f;

inline constexpr float INTAKE_PID_KP   = 1.0f;
inline constexpr float INTAKE_PID_KI   = 0.0f;
inline constexpr float INTAKE_PID_KD   = 0.0f;
inline constexpr float INTAKE_PID_ICAP = 0.0f;

// ── RAMSETE path-following parameters ────────────────────────────────────────
inline constexpr float RAMSETE_ZETA = 0.7f;
// β_inches = β_meters × (m/in)² — equivalent to the standard β=2.0 in metric
inline constexpr float RAMSETE_BETA = 2.0f * IN_TO_M * IN_TO_M;

// ── LTV unicycle parameters ──────────────────────────────────────────────────
inline constexpr float LTV_Q_X           = 1.0f;
inline constexpr float LTV_Q_Y           = 1.0f;
inline constexpr float LTV_Q_THETA       = 10.0f;
inline constexpr float LTV_R_V           = 1.0f;
inline constexpr float LTV_R_OMEGA       = 1.0f;
// Match EZ-Template's 10 ms autonomous control loop.
inline constexpr float LTV_DT_S          = 0.01f;
inline constexpr float LTV_MAX_VEL_INPS  = MAX_SPEED_INPS;
// Match WPILib's 0.01 m/s lookup-table spacing, converted to inches/s.
inline constexpr float LTV_LOOKUP_STEP   = 0.01f / IN_TO_M;
inline constexpr float LTV_TERMINAL_SCALE = 1.0f;

// ── Drivetrain feedforward model ─────────────────────────────────────────────
inline constexpr float FF_kS = 1100.0f;   // mV
inline constexpr float FF_kV = 5200.0f;   // mV·s/m
inline constexpr float FF_kA = 400.0f;    // mV·s²/m
inline constexpr float TRACKING_VEL_KP_MV_PER_MPS = 900.0f;
inline constexpr float TRACKING_VEL_KD_MV_PER_MPS2 = 0.0f;
inline constexpr float TRACKING_VEL_MAX_CORRECTION_MV = 2500.0f;
inline constexpr float TRACKING_VEL_MEAS_ALPHA = 0.35f;

inline std::pair<float, float> drivetrainFeedforward_mV(
    float v_mps, float omega_radps, float a_mps2 = 0.0f, float alpha_radps2 = 0.0f) {
  const float trackWidthM = TRACK_WIDTH_IN * IN_TO_M;
  // EZ-Template uses CW-positive heading/omega, so positive omega means the
  // left side should run faster than the right side.
  const float vLeft  = v_mps + omega_radps * trackWidthM / 2.0f;
  const float vRight = v_mps - omega_radps * trackWidthM / 2.0f;
  const float aLeft  = a_mps2 + alpha_radps2 * trackWidthM / 2.0f;
  const float aRight = a_mps2 - alpha_radps2 * trackWidthM / 2.0f;

  auto ff = [](float vel, float acc) -> float {
    const float sign = (vel > 0.0f) ? 1.0f : ((vel < 0.0f) ? -1.0f : 0.0f);
    return FF_kS * sign + FF_kV * vel + FF_kA * acc;
  };

  return {ff(vLeft, aLeft), ff(vRight, aRight)};
}

}  // namespace RobotConfig
