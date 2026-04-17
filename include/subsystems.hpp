#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"


// ── Chassis (defined in main.cpp) ─────────────────────────────────────────────
extern ez::Drive chassis;

// ── Motors ────────────────────────────────────────────────────────────────────
// Intake subsystem (ported lightweight wrapper from bigga4-with-okapi).
inline pros::MotorGroup intakeMotors(
	{15, -17},
	pros::v5::MotorGears::blue,
	pros::v5::MotorUnits::degrees);


// ── Pneumatics ────────────────────────────────────────────────────────────────
// Shared named ADI outputs (ported from bigga4-with-okapi naming).
inline pros::adi::DigitalOut selector('B', false);
inline pros::adi::DigitalOut top('A', false);
inline pros::adi::DigitalOut tongue('C', false);
inline pros::adi::DigitalOut wing('D', false);
