#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "robot_config.hpp"

#include <algorithm>

// ── Chassis (defined in main.cpp) ─────────────────────────────────────────────
extern ez::Drive chassis;

// ── Motors ────────────────────────────────────────────────────────────────────
// Intake subsystem (ported lightweight wrapper from bigga4-with-okapi).
inline pros::MotorGroup intakeMotors(
	{RobotConfig::INTAKE_PORTS[0], RobotConfig::INTAKE_PORTS[1]},
	pros::v5::MotorGears::green,
	pros::v5::MotorUnits::degrees);


// ── Pneumatics ────────────────────────────────────────────────────────────────
// Shared named ADI outputs (ported from bigga4-with-okapi naming).
inline pros::adi::DigitalOut selector(RobotConfig::SELECT_PORT, false);
inline pros::adi::DigitalOut top(RobotConfig::TOP_PORT, false);
inline pros::adi::DigitalOut tongue(RobotConfig::TONGUE_PORT, false);
inline pros::adi::DigitalOut wing(RobotConfig::WING_PORT, false);

// ── GPS Configuration ─────────────────────────────────────────────────────────
// Set GPS_ENABLED to true if a GPS sensor is installed.
inline constexpr bool  GPS_ENABLED            = true;
inline constexpr int   GPS_PORT               = RobotConfig::GPS_PORT;
inline constexpr float GPS_HEADING_OFFSET_DEG = RobotConfig::GPS_HEADING_OFFSET_DEG;
inline constexpr float GPS_FIELD_ROTATION_DEG = RobotConfig::GPS_FIELD_ROTATION_DEG;
inline constexpr float GPS_MOUNT_OFFSET_X_IN  = RobotConfig::GPS_OFFSET_X_IN;
inline constexpr float GPS_MOUNT_OFFSET_Y_IN  = RobotConfig::GPS_OFFSET_Y_IN;
inline constexpr float GPS_MAX_ERROR_M        = RobotConfig::GPS_MAX_ERROR_M;
inline constexpr float GPS_RUNTIME_MAX_ERROR_M = RobotConfig::GPS_RUNTIME_MAX_ERROR_M;
inline constexpr float GPS_CORRECTION_MAX_IN   = RobotConfig::GPS_CORRECTION_MAX_IN;
inline constexpr float GPS_CORRECTION_STEP_IN  = RobotConfig::GPS_CORRECTION_STEP_IN;
inline constexpr int   GPS_STABLE_SAMPLES      = RobotConfig::GPS_STABLE_SAMPLES;
inline constexpr uint32_t GPS_MAX_WAIT_MS      = RobotConfig::GPS_MAX_WAIT_MS;

// ── Drive hardware constants (for LTV / Ramsete) ──────────────────────────────
inline constexpr float DRIVE_TRACK_WIDTH_IN = RobotConfig::TRACK_WIDTH_IN;
inline constexpr float DRIVE_MAX_VEL_INPS   = RobotConfig::MAX_SPEED_INPS;

inline constexpr float RAMSETE_ZETA = RobotConfig::RAMSETE_ZETA;
inline constexpr float RAMSETE_BETA = RobotConfig::RAMSETE_BETA;
