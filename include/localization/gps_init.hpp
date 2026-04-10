/**
 * @file gps_init.hpp
 * VEX GPS sensor initialization and coordinate conversion for EZ-Template.
 *
 * The VEX GPS API uses:
 *   - position: metres, field-centre origin
 *   - heading:  compass degrees  (0 = north, CW positive)
 *
 * EZ-Template odom uses:
 *   - position: inches, user-defined origin (whatever you set at boot)
 *   - theta:    degrees, 0 = robot starting angle, increasing depends on setup
 *
 * This utility polls the GPS sensor at startup, converts to inches, and seeds
 * `chassis.odom_xyt_set()`.  It also provides a runtime helper to apply
 * a gentle GPS correction to the chassis odom pose.
 *
 * Usage in initialize():
 *   #include "localization/gps_init.hpp"
 *   GpsInit::Config cfg;
 *   cfg.port = 3;
 *   GpsInit::init(chassis, cfg);   // waits up to 5 s then seeds odom
 *
 * Usage in a continuous correction loop:
 *   GpsInit::applyCorrection(chassis, cfg);   // call ~every 100 ms
 */
#pragma once

#include "EZ-Template/api.hpp"
#include "robot_config.hpp"
#include "pros/gps.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <optional>

namespace GpsInit {

// ── Configuration ─────────────────────────────────────────────────────────────
struct Config {
    int   port             = RobotConfig::GPS_PORT;      ///< GPS smart port
    float headingOffsetDeg = RobotConfig::GPS_HEADING_OFFSET_DEG;
    float fieldRotationDeg = RobotConfig::GPS_FIELD_ROTATION_DEG;
    float mountOffsetXIn   = RobotConfig::GPS_OFFSET_X_IN;
    float mountOffsetYIn   = RobotConfig::GPS_OFFSET_Y_IN;
    float maxErrorM        = RobotConfig::GPS_MAX_ERROR_M;
    int   stableSamples    = RobotConfig::GPS_STABLE_SAMPLES;
    uint32_t maxWaitMs     = RobotConfig::GPS_MAX_WAIT_MS;
    uint32_t pollMs        = 100;    ///< poll interval (ms)
    float runtimeMaxErrM   = RobotConfig::GPS_RUNTIME_MAX_ERROR_M;
    float correctionMaxIn  = RobotConfig::GPS_CORRECTION_MAX_IN;
    float correctionStepIn = RobotConfig::GPS_CORRECTION_STEP_IN;
};

// ── Internal helpers ──────────────────────────────────────────────────────────
namespace detail {

inline float compassToEzDeg(float compassDeg, float fieldRotDeg) {
    // VEX GPS compass (0=north, CW) → typical math heading (0=east, CCW)
    // EZ-Template theta usually starts at 0 and the IMU tracks deltas.
    // We return the heading in the same "math angle" convention EZ-Template uses
    // for odom_xyt_set: counterclockwise positive, 0° = robot starting forward.
    // For seeding purposes, convert compass → CCW math degrees:
    //   math = (90 - compass) with field rotation applied
    float raw = 90.0f - compassDeg + fieldRotDeg;
    while (raw >  180.0f) raw -= 360.0f;
    while (raw <= -180.0f) raw += 360.0f;
    return raw;
}

inline void rotatePoint(float xIn, float yIn, float rotDeg,
                         float& outX, float& outY) {
    float rad = rotDeg * (3.14159265f / 180.0f);
    float c = std::cos(rad), s = std::sin(rad);
    outX = xIn * c - yIn * s;
    outY = xIn * s + yIn * c;
}

struct GpsReading {
    float xIn, yIn, headingDeg, errorM;
    bool valid;
};

inline GpsReading readGps(pros::Gps& gps, const Config& cfg) {
    auto pos = gps.get_position();
    float rawXM = static_cast<float>(pos.x);
    float rawYM = static_cast<float>(pos.y);
    float errM  = static_cast<float>(gps.get_error());
    float hdg   = static_cast<float>(gps.get_heading()) - cfg.headingOffsetDeg;

    // Check validity
    if (!std::isfinite(rawXM) || !std::isfinite(rawYM) ||
        !std::isfinite(errM)  || !std::isfinite(hdg)   ||
        errM < 0.0f) {
        return {0, 0, 0, -1, false};
    }

    // Convert metres → inches
    float xIn = rawXM * 39.3701f;
    float yIn = rawYM * 39.3701f;

    // Apply field-frame rotation
    float rxIn, ryIn;
    rotatePoint(xIn, yIn, cfg.fieldRotationDeg, rxIn, ryIn);

    // Apply mount offset (sensor → robot centre) in robot frame
    float headRad = (compassToEzDeg(hdg, cfg.fieldRotationDeg)) * (3.14159265f / 180.0f);
    float cx  = std::cos(headRad), sx = std::sin(headRad);
    float rcX = rxIn - (cfg.mountOffsetXIn * cx - cfg.mountOffsetYIn * sx);
    float rcY = ryIn - (cfg.mountOffsetXIn * sx + cfg.mountOffsetYIn * cx);

    return {rcX, rcY, compassToEzDeg(hdg, cfg.fieldRotationDeg), errM, true};
}

}  // namespace detail

// ── Startup seeding ────────────────────────────────────────────────────────────
/**
 * Poll GPS at startup and seed EZ-Template odometry.
 * Blocks until @p stableSamples consecutive good readings are received
 * (within @p maxErrorM) or @p maxWaitMs elapses.
 *
 * @param chassis  reference to the global ez::Drive chassis object
 * @param cfg      GPS configuration
 * @param progressCb  optional callback(0..1) for init-screen progress bar
 * @return true if seeded successfully, false if timed out
 */
inline bool init(ez::Drive& chassis, const Config& cfg,
                 void (*progressCb)(float) = nullptr) {
    pros::Gps gps(cfg.port);
    int stable = 0;
    uint32_t start = pros::millis();

    while (pros::millis() - start < cfg.maxWaitMs) {
        float elapsed = static_cast<float>(pros::millis() - start);
        float progress = elapsed / static_cast<float>(cfg.maxWaitMs);
        if (progressCb) progressCb(progress);

        auto r = detail::readGps(gps, cfg);
        if (r.valid && r.errorM <= cfg.maxErrorM) {
            ++stable;
            if (stable >= cfg.stableSamples) {
                // Seed odom: use GPS x/y + GPS heading
                chassis.odom_xyt_set(r.xIn * okapi::inch,
                                     r.yIn * okapi::inch,
                                     r.headingDeg * okapi::degree);
                return true;
            }
        } else {
            stable = 0;
        }
        pros::delay(cfg.pollMs);
    }
    return false;  // timed out — odom will start from configured default
}

// ── Runtime correction ────────────────────────────────────────────────────────
/**
 * Read GPS and gently nudge odom towards GPS position.
 * Call this in a background task approximately every 100–200 ms.
 *
 * @return true if a correction was applied
 */
inline bool applyCorrection(ez::Drive& chassis, const Config& cfg) {
    pros::Gps gps(cfg.port);
    auto r = detail::readGps(gps, cfg);
    if (!r.valid || r.errorM > cfg.runtimeMaxErrM) return false;

    float cx = static_cast<float>(chassis.odom_x_get());
    float cy = static_cast<float>(chassis.odom_y_get());

    float dx = r.xIn - cx;
    float dy = r.yIn - cy;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 0.5f || dist > cfg.correctionMaxIn) return false;

    float step = std::min(dist, cfg.correctionStepIn);
    float nx = cx + dx * (step / dist);
    float ny = cy + dy * (step / dist);

    float currTheta = static_cast<float>(chassis.odom_theta_get());
    chassis.odom_xyt_set(nx * okapi::inch, ny * okapi::inch,
                         currTheta * okapi::degree);
    return true;
}

/**
 * Convenience: read GPS and return pose in inches/degrees.
 * Returns std::nullopt if GPS is invalid or error exceeds maxErrorM.
 */
struct GpsPose { float x, y, theta, errorM; };
inline std::optional<GpsPose> readPose(const Config& cfg) {
    pros::Gps gps(cfg.port);
    auto r = detail::readGps(gps, cfg);
    if (!r.valid || r.errorM > cfg.maxErrorM) return std::nullopt;
    return GpsPose{r.xIn, r.yIn, r.headingDeg, r.errorM};
}

}  // namespace GpsInit
