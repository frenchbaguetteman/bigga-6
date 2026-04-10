/**
 * @file ramsete_controller.hpp
 * RAMSETE nonlinear path-following controller.
 *
 * Implements the canonical RAMSETE equations from bigga-4 without Eigen.
 *
 *   k  = 2ζ √(ω_d² + β·v_d²)
 *   v  = v_d·cos(e_θ) + k·e_x
 *   ω  = ω_d + k·e_θ + β·v_d·sinc(e_θ)·e_y
 *
 * Accepts EZ-Template convention inputs: inches, degrees (0°=+Y, CW positive).
 *
 * Usage:
 *   RamseteController ctrl;  // or with custom zeta/beta/track_width
 *   auto [left, right] = ctrl.calculate(cx, cy, ctheta,
 *                                        dx, dy, dtheta,
 *                                        vRef_inps, omegaRef_radps);
 *   chassis.drive_set(left, right);
 */
#pragma once

#include "robot_config.hpp"

#include <cmath>
#include <algorithm>

// ── sinc function ─────────────────────────────────────────────────────────────
namespace ramsete_detail {
inline float sinc(float x) {
    if (std::fabs(x) < 1e-4f) return 1.0f - (x * x) / 6.0f;
    return std::sin(x) / x;
}
inline float wrapAngle(float a) {
    while (a >  3.14159265f) a -= 6.28318530f;
    while (a < -3.14159265f) a += 6.28318530f;
    return a;
}
}  // namespace ramsete_detail

// ── RamseteController ─────────────────────────────────────────────────────────
class RamseteController {
public:
    struct DiffSpeeds { int left, right; };  // [-127, 127] motor commands
    struct ChassisCommand { float linear, angular; };

    /**
     * @param zeta         damping  coefficient  (0 < ζ < 1, recommended ~0.7)
     * @param beta         aggressiveness param  (β > 0, recommended 2–10)
     * @param maxVelInps   top speed in in/s     (used for normalisation)
     * @param trackWidthIn drivetrain track width in inches
     */
    explicit RamseteController(float zeta         = RobotConfig::RAMSETE_ZETA,
                               float beta         = RobotConfig::RAMSETE_BETA,
                               float maxVelInps   = RobotConfig::MAX_SPEED_INPS,
                               float trackWidthIn = RobotConfig::TRACK_WIDTH_IN)
        : m_zeta(zeta), m_beta(beta),
          m_maxVelInps(maxVelInps), m_trackWidthIn(trackWidthIn) {}

    /**
     * Compute left/right motor commands.
     *
     * @param cx/cy/ctheta   current pose — inches / degrees (EZ-Template odom)
     * @param dx/dy/dtheta   desired  pose — inches / degrees
     * @param vRef           desired linear velocity  (in/s)
     * @param omegaRef       desired angular velocity (deg/s)
     */
    ChassisCommand calculateChassisSpeeds(float cx, float cy, float ctheta,
                                          float dx, float dy, float dtheta,
                                          float vRef, float omegaRef) {
        using namespace ramsete_detail;

        // Convert EZ-Template degrees to radians for internal math
        constexpr float kDeg2Rad = 3.14159265f / 180.0f;
        float ctRad       = ctheta   * kDeg2Rad;
        float omegaRefRad = omegaRef * kDeg2Rad;

        // Error in robot frame  (EZ convention: 0°=+Y, CW positive)
        float sinT = std::sin(ctRad);
        float cosT = std::cos(ctRad);
        float dxW  = dx - cx;
        float dyW  = dy - cy;
        m_ex    =  sinT * dxW + cosT * dyW;       // forward error
        m_ey    =  cosT * dxW - sinT * dyW;       // lateral error (right +)
        m_eth   = wrapAngle((dtheta - ctheta) * kDeg2Rad);

        // Gain k
        float k = 2.0f * m_zeta *
                  std::sqrt(omegaRefRad * omegaRefRad + m_beta * vRef * vRef);

        // RAMSETE commanded velocities
        float v     = vRef        * std::cos(m_eth) + k * m_ex;
        float omega = omegaRefRad + k * m_eth + m_beta * vRef * sinc(m_eth) * m_ey;

        m_lastV     = v;
        m_lastOmega = omega;

        return {v, omega};
    }

    DiffSpeeds calculate(float cx, float cy, float ctheta,
                         float dx, float dy, float dtheta,
                         float vRef, float omegaRef) {
        const auto cmd = calculateChassisSpeeds(cx, cy, ctheta, dx, dy, dtheta,
                                                vRef, omegaRef);
        return toMotorCommands(cmd.linear, cmd.angular);
    }

    float lastV()     const { return m_lastV; }
    float lastOmega() const { return m_lastOmega; }
    float lastEx()    const { return m_ex; }
    float lastEy()    const { return m_ey; }
    float lastEth()   const { return m_eth; }

private:
    float m_zeta, m_beta, m_maxVelInps, m_trackWidthIn;
    float m_ex = 0, m_ey = 0, m_eth = 0;
    float m_lastV = 0, m_lastOmega = 0;

    DiffSpeeds toMotorCommands(float v, float omega) const {
        float tw2  = m_trackWidthIn / 2.0f;
        // CW-positive convention: positive ω → left faster, right slower
        float vL   = v + omega * tw2;
        float vR   = v - omega * tw2;
        float norm = std::max({1.0f, std::fabs(vL) / m_maxVelInps,
                                     std::fabs(vR) / m_maxVelInps});
        int left  = static_cast<int>(std::round(127.0f * vL /
                                    (m_maxVelInps * norm)));
        int right = static_cast<int>(std::round(127.0f * vR /
                                    (m_maxVelInps * norm)));
        left  = std::max(-127, std::min(127, left));
        right = std::max(-127, std::min(127, right));
        return {left, right};
    }
};
