/**
 * @file ltv_controller.hpp
 * LTV (Linear Time-Varying) unicycle path-tracking controller.
 *
 * Implements the LQR-style feedback law from bigga-4 without Eigen.
 * All matrix math is done with fixed-size std::array<> types.
 *
 * State error  e = [x_err, y_err, θ_err]ᵀ  (robot frame, inches / radians internal)
 *
 * Accepts EZ-Template convention inputs: inches, degrees (0°=+Y, CW positive).
 * Control      u = [Δv, Δω]ᵀ
 *
 * Usage:
 *   LtvController ctrl;
 *   // In a loop:
 *   auto [leftPct, rightPct] = ctrl.calculate(cx, cy, ct,
 *                                              dx, dy, dt,
 *                                              vRef, omegaRef);
 *   chassis.drive_set(leftPct, rightPct);
 */
#pragma once

#include "robot_config.hpp"

#include <array>
#include <cmath>
#include <vector>
#include <algorithm>


namespace ltv {

// ── Tiny fixed-size matrix types ─────────────────────────────────────────────
using Vec2  = std::array<float, 2>;
using Vec3  = std::array<float, 3>;
using Mat22 = std::array<std::array<float, 2>, 2>;
using Mat33 = std::array<std::array<float, 3>, 3>;
using Mat32 = std::array<std::array<float, 2>, 3>;  // 3 rows, 2 cols
using Mat23 = std::array<std::array<float, 3>, 2>;  // 2 rows, 3 cols (gain K)

// ── Matrix helpers ────────────────────────────────────────────────────────────
inline Mat33 mat33Identity() {
    return {{{1,0,0},{0,1,0},{0,0,1}}};
}
inline Mat22 mat22Identity() {
    return {{{1,0},{0,1}}};
}

inline Mat33 mat33Add(const Mat33& A, const Mat33& B) {
    Mat33 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}
inline Mat33 mat33Scale(float s, const Mat33& A) {
    Mat33 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            C[i][j] = s * A[i][j];
    return C;
}
inline Mat33 mat33Mul(const Mat33& A, const Mat33& B) {
    Mat33 C{};
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            for (int j = 0; j < 3; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}
inline Mat33 mat33Transpose(const Mat33& A) {
    Mat33 T{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            T[i][j] = A[j][i];
    return T;
}
inline Mat32 mat33MulMat32(const Mat33& A, const Mat32& B) {
    Mat32 C{};
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            for (int j = 0; j < 2; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}
inline Mat23 mat32TransposeMulMat33(const Mat32& B, const Mat33& A) {
    // B^T (2x3) * A (3x3) = (2x3)
    Mat23 C{};
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 3; ++k)
            for (int j = 0; j < 3; ++j)
                C[i][j] += B[k][i] * A[k][j];
    return C;
}
inline Mat22 mat32TransposeMulMat32(const Mat32& B) {
    // B^T (2x3) * B (3x2) = (2x2)
    Mat22 C{};
    for (int k = 0; k < 3; ++k)
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                C[i][j] += B[k][i] * B[k][j];
    return C;
}
inline Mat23 mat23MulMat33(const Mat23& K, const Mat33& A) {
    Mat23 C{};
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 3; ++k)
            for (int j = 0; j < 3; ++j)
                C[i][j] += K[i][k] * A[k][j];
    return C;
}
inline Mat33 mat33Symmetrize(const Mat33& A) {
    Mat33 S{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            S[i][j] = 0.5f * (A[i][j] + A[j][i]);
    return S;
}
inline Vec2 mat23MulVec3(const Mat23& K, const Vec3& e) {
    Vec2 u{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            u[i] += K[i][j] * e[j];
    return u;
}
inline Vec3 mat32MulVec2(const Mat32& B, const Vec2& u) {
    Vec3 y{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 2; ++j)
            y[i] += B[i][j] * u[j];
    return y;
}
inline Mat22 mat22Add(const Mat22& A, const Mat22& B) {
    return {{{A[0][0]+B[0][0], A[0][1]+B[0][1]},
             {A[1][0]+B[1][0], A[1][1]+B[1][1]}}};
}
inline Mat22 mat22Invert(const Mat22& A) {
    float det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (std::fabs(det) < 1e-9f)
        return {{{1,0},{0,1}}};  // fallback
    float invDet = 1.0f / det;
    return {{{ A[1][1]*invDet, -A[0][1]*invDet},
             {-A[1][0]*invDet,  A[0][0]*invDet}}};
}
inline Mat23 mat22MulMat23(const Mat22& A, const Mat23& B) {
    Mat23 C{};
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 2; ++k)
            for (int j = 0; j < 3; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}
// (2x3) * (3x2) = (2x2)
inline Mat22 mat23MulMat32(const Mat23& A, const Mat32& B) {
    Mat22 C{};
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 3; ++k)
            for (int j = 0; j < 2; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}
// A (3x3) - B*C (3x3)
inline Mat33 mat33Sub(const Mat33& A, const Mat33& B) {
    Mat33 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            C[i][j] = A[i][j] - B[i][j];
    return C;
}
inline float mat33MaxAbsDiff(const Mat33& A, const Mat33& B) {
    float m = 0.0f;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m = std::max(m, std::fabs(A[i][j] - B[i][j]));
    return m;
}
// A^T * P * A
inline Mat33 mat33TPA(const Mat33& A, const Mat33& P) {
    return mat33Mul(mat33Transpose(A), mat33Mul(P, A));
}
// A^T * P * B  → (3x3)^T*(3x3)*(3x2) = (3x2) then B^T*P*B skipped, compute directly
inline Mat32 mat33TPAb(const Mat33& A, const Mat33& P, const Mat32& B) {
    // (A^T * P) first
    Mat33 ATP = mat33Mul(mat33Transpose(A), P);
    return mat33MulMat32(ATP, B);
}

// ── Linearized unicycle discrete model ───────────────────────────────────────
inline Mat33 makeAd(float v, float dt) {
    Mat33 A = mat33Identity();
    A[1][2] = v * dt;
    return A;
}
inline Mat32 makeBd(float v, float dt) {
    Mat32 B{};
    B[0][0] = dt;
    B[1][1] = 0.5f * v * dt * dt;
    B[2][1] = dt;
    return B;
}
inline Mat33 makeDiag3(const Vec3& d) {
    return {{{d[0],0,0},{0,d[1],0},{0,0,d[2]}}};
}
inline Mat22 makeDiag2(const Vec2& d) {
    return {{{d[0],0},{0,d[1]}}};
}

// ── Discrete Riccati iteration ────────────────────────────────────────────────
inline Mat33 solveRiccati(const Mat33& Ad, const Mat32& Bd,
                          const Mat33& Q,  const Mat22& R,
                          int maxIter = 500, float tol = 1e-5f) {
    Mat33 P = Q;
    for (int iter = 0; iter < maxIter; ++iter) {
        // lhs = R + Bd^T * P * Bd  (2x2)
        // P*Bd (3x2)
        Mat32 PBd{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 3; ++k)
                    PBd[i][j] += P[i][k] * Bd[k][j];
        // Bd^T * PBd (2x2)
        Mat22 BtPBd{};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 3; ++k)
                    BtPBd[i][j] += Bd[k][i] * PBd[k][j];
        Mat22 lhs = mat22Add(R, BtPBd);
        Mat22 lhsInv = mat22Invert(lhs);

        // K = lhsInv * Bd^T * P * Ad
        // Bd^T * P * Ad = Bd^T * (P*Ad)
        Mat33 PAd{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    PAd[i][j] += P[i][k] * Ad[k][j];
        Mat23 BtPAd{};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    BtPAd[i][j] += Bd[k][i] * PAd[k][j];
        Mat23 K = mat22MulMat23(lhsInv, BtPAd);

        // Ad - Bd * K (3x3)
        Mat32 BdArr = Bd;
        Mat33 BdK{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 2; ++k)
                    BdK[i][j] += BdArr[i][k] * K[k][j];
        Mat33 AdMinusBdK = mat33Sub(Ad, BdK);

        // P_next = Ad^T * P * (Ad - Bd*K) + Q
        Mat33 tmp{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    tmp[i][j] += Ad[k][i] * P[k][j];  // Ad^T * P → tmp
        Mat33 tmp2{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    tmp2[i][j] += tmp[i][k] * AdMinusBdK[k][j];
        Mat33 Pnext = mat33Add(tmp2, Q);
        Pnext = mat33Symmetrize(Pnext);

        if (mat33MaxAbsDiff(Pnext, P) <= tol) return Pnext;
        P = Pnext;
    }
    return P;
}

inline Mat23 solveGain(float v, const Vec3& q, const Vec2& r, float dt) {
    // Match WPILib's near-zero velocity guard, converted from m/s to in/s.
    constexpr float kMinV = 1e-4f / RobotConfig::IN_TO_M;
    if (std::fabs(v) < kMinV)
        v = (v < 0.0f) ? -kMinV : kMinV;

    Mat33 Ad = makeAd(v, dt);
    Mat32 Bd = makeBd(v, dt);
    Mat33 Q  = makeDiag3(q);
    Mat22 R  = makeDiag2(r);
    Mat33 P  = solveRiccati(Ad, Bd, Q, R);

    // K = (R + Bd^T*P*Bd)^{-1} * Bd^T * P * Ad
    Mat32 PBd{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 3; ++k)
                PBd[i][j] += P[i][k] * Bd[k][j];
    Mat22 BtPBd{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 3; ++k)
                BtPBd[i][j] += Bd[k][i] * PBd[k][j];
    Mat22 lhsInv = mat22Invert(mat22Add(R, BtPBd));

    Mat33 PAd{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                PAd[i][j] += P[i][k] * Ad[k][j];
    Mat23 BtPAd{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                BtPAd[i][j] += Bd[k][i] * PAd[k][j];

    return mat22MulMat23(lhsInv, BtPAd);
}

// ── Wrap angle to [-π, π] ─────────────────────────────────────────────────────
inline float wrapAngle(float a) {
    while (a >  3.14159265f) a -= 6.28318530f;
    while (a < -3.14159265f) a += 6.28318530f;
    return a;
}

}  // namespace ltv

// ── Shared structs (outside class to avoid C++ aggregate init limitation) ────
struct LtvDiffSpeeds { int left, right; };  // [-127, 127] motor commands

struct LtvConfig {
    ltv::Vec3  q            = {RobotConfig::LTV_Q_X, RobotConfig::LTV_Q_Y, RobotConfig::LTV_Q_THETA};
    ltv::Vec2  r            = {RobotConfig::LTV_R_V, RobotConfig::LTV_R_OMEGA};
    float      dt           = RobotConfig::LTV_DT_S;
    float      maxVelInps   = RobotConfig::LTV_MAX_VEL_INPS;
    float      lookupStep   = RobotConfig::LTV_LOOKUP_STEP;
    float      trackWidthIn = RobotConfig::TRACK_WIDTH_IN;
};

// ── LtvController ─────────────────────────────────────────────────────────────
class LtvController {
public:
    using DiffSpeeds = LtvDiffSpeeds;
    using Config     = LtvConfig;
    struct ChassisCommand { float linear, angular; };

    explicit LtvController(LtvConfig cfg = LtvConfig{}) : m_cfg(cfg) {
        buildLookupTable();
    }

    /**
     * Calculate motor commands.
     * @param cx/cy/ctheta  current pose — inches / degrees (EZ-Template odom)
     * @param dx/dy/dtheta  desired pose — inches / degrees
     * @param vRef          desired linear velocity (in/s)
     * @param omegaRef      desired angular velocity (deg/s)
     */
    ChassisCommand calculateChassisSpeeds(float cx, float cy, float ctheta,
                                          float dx, float dy, float dtheta,
                                          float vRef, float omegaRef) {
        // Convert EZ-Template degrees to radians for internal math
        constexpr float kDeg2Rad = 3.14159265f / 180.0f;
        float ctRad       = ctheta   * kDeg2Rad;
        float omegaRefRad = omegaRef * kDeg2Rad;

        // Error in robot frame  (EZ convention: 0°=+Y, CW positive)
        float sinT = std::sin(ctRad);
        float cosT = std::cos(ctRad);
        float dxW  = dx - cx;
        float dyW  = dy - cy;
        m_lastErr[0] =  sinT * dxW + cosT * dyW;   // forward error
        m_lastErr[1] =  cosT * dxW - sinT * dyW;   // lateral error (right +)
        m_lastErr[2] = ltv::wrapAngle((dtheta - ctheta) * kDeg2Rad);

        ltv::Mat23 K    = gainForVelocity(vRef);
        ltv::Vec2  corr = ltv::mat23MulVec3(K, m_lastErr);

        float v     = vRef        + corr[0];
        float omega = omegaRefRad + corr[1];

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

    const ltv::Vec3& lastError() const { return m_lastErr; }
    float lastV()     const { return m_lastV; }
    float lastOmega() const { return m_lastOmega; }

private:
    Config      m_cfg;
    ltv::Vec3   m_lastErr{};
    float       m_lastV = 0.0f, m_lastOmega = 0.0f;

    std::vector<float>       m_velSamples;
    std::vector<ltv::Mat23>  m_gainSamples;

    void buildLookupTable() {
        m_velSamples.clear();
        m_gainSamples.clear();
        const float step = std::max(1e-4f / RobotConfig::IN_TO_M,
                                    std::fabs(m_cfg.lookupStep));
        int n = std::max(2, static_cast<int>(
            std::ceil(2.0f * m_cfg.maxVelInps / step)) + 1);
        m_velSamples.reserve(n);
        m_gainSamples.reserve(n);
        for (int i = 0; i < n; ++i) {
            float v = -m_cfg.maxVelInps + i * step;
            if (i == n - 1 || v > m_cfg.maxVelInps) v = m_cfg.maxVelInps;
            m_velSamples.push_back(v);
            m_gainSamples.push_back(
                ltv::solveGain(v, m_cfg.q, m_cfg.r, m_cfg.dt));
        }
    }

    ltv::Mat23 gainForVelocity(float v) const {
        if (m_velSamples.empty())
            return ltv::solveGain(v, m_cfg.q, m_cfg.r, m_cfg.dt);
        if (v <= m_velSamples.front()) return m_gainSamples.front();
        if (v >= m_velSamples.back())  return m_gainSamples.back();

        auto it = std::lower_bound(m_velSamples.begin(), m_velSamples.end(), v);
        size_t hi = static_cast<size_t>(it - m_velSamples.begin());
        size_t lo = hi - 1;
        float range = m_velSamples[hi] - m_velSamples[lo];
        float alpha = (range < 1e-6f) ? 0.0f :
                      (v - m_velSamples[lo]) / range;

        ltv::Mat23 K{};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 3; ++j)
                K[i][j] = (1.0f - alpha) * m_gainSamples[lo][i][j]
                           + alpha       * m_gainSamples[hi][i][j];
        return K;
    }

    DiffSpeeds toMotorCommands(float v, float omega) const {
        float tw2  = m_cfg.trackWidthIn / 2.0f;
        // CW-positive convention: positive ω → left faster, right slower
        float vL   = v + omega * tw2;
        float vR   = v - omega * tw2;
        float norm = std::max({1.0f, std::fabs(vL)/m_cfg.maxVelInps,
                                     std::fabs(vR)/m_cfg.maxVelInps});
        int left  = static_cast<int>(std::round(127.0f * vL /
                                    (m_cfg.maxVelInps * norm)));
        int right = static_cast<int>(std::round(127.0f * vR /
                                    (m_cfg.maxVelInps * norm)));
        left  = std::max(-127, std::min(127, left));
        right = std::max(-127, std::min(127, right));
        return {left, right};
    }
};
