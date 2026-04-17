/**
 * @file ltv_controller.hpp
 *
 * Local units-adapted port of WPILib's frc::LTVUnicycleController.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ltv {

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;
using Mat22 = std::array<std::array<double, 2>, 2>;
using Mat32 = std::array<std::array<double, 2>, 3>;
using Mat23 = std::array<std::array<double, 3>, 2>;
using Mat33 = std::array<std::array<double, 3>, 3>;

inline constexpr double kInPerMeter = 1.0 / 0.0254;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDeg2Rad = kPi / 180.0;

inline constexpr double kDefaultQxIn = 0.0625 * kInPerMeter;
inline constexpr double kDefaultQyIn = 0.1250 * kInPerMeter;
inline constexpr double kDefaultQthetaRad = 2.0;
inline constexpr double kDefaultRvInps = 1.0 * kInPerMeter;
inline constexpr double kDefaultRwRadps = 2.0;
inline constexpr double kVelocityStepInps = 0.01 * kInPerMeter;
inline constexpr double kNearZeroVelInps = 1e-4 * kInPerMeter;
inline constexpr double kMaxVelocityUpperInps = 15.0 * kInPerMeter;
inline constexpr double kWpilibDefaultMaxVelInps = 9.0 * kInPerMeter;

// Minimum |velocity| used when looking up a gain. Below this, the linearized
// y/heading coupling (A[1][2] = v) is too weak to produce meaningful
// cross-track correction, so the lookup clamps the effective velocity while
// preserving sign. Does not affect how the gain table is built.
inline constexpr double kMinLookupVelInps = 6.0;

struct Translation2d {
  double x = 0.0;
  double y = 0.0;

  double X() const { return x; }
  double Y() const { return y; }
};

struct Rotation2d {
  double radians = 0.0;

  double Radians() const { return radians; }
};

inline double WrapAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

  double X() const { return x; }
  double Y() const { return y; }
  Translation2d Translation() const { return {x, y}; }
  Rotation2d Rotation() const { return {theta}; }

  Pose2d RelativeTo(const Pose2d& other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    const double c = std::cos(other.theta);
    const double s = std::sin(other.theta);
    return {c * dx + s * dy, -s * dx + c * dy, WrapAngle(theta - other.theta)};
  }
};

struct ChassisSpeeds {
  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
};

struct TrajectoryState {
  Pose2d pose;
  double velocity = 0.0;
  double curvature = 0.0;
};

inline Mat33 Mat33Identity() {
  return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}

inline Mat33 Mat33Add(const Mat33& A, const Mat33& B) {
  Mat33 result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result[i][j] = A[i][j] + B[i][j];
    }
  }
  return result;
}

inline Mat33 Mat33Sub(const Mat33& A, const Mat33& B) {
  Mat33 result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result[i][j] = A[i][j] - B[i][j];
    }
  }
  return result;
}

inline Mat33 Mat33Mul(const Mat33& A, const Mat33& B) {
  Mat33 result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return result;
}

inline Mat32 Mat33Mul32(const Mat33& A, const Mat32& B) {
  Mat32 result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 3; ++k) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return result;
}

inline Mat23 Mat23Mul33(const Mat23& A, const Mat33& B) {
  Mat23 result{};
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return result;
}

inline Mat33 Mat32Mul23(const Mat32& A, const Mat23& B) {
  Mat33 result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 2; ++k) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return result;
}

inline Mat22 Mat23Mul32(const Mat23& A, const Mat32& B) {
  Mat22 result{};
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 3; ++k) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return result;
}

inline Mat33 Mat33Transpose(const Mat33& A) {
  Mat33 result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result[i][j] = A[j][i];
    }
  }
  return result;
}

inline double Mat33FrobeniusNorm(const Mat33& A) {
  double sum = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      sum += A[i][j] * A[i][j];
    }
  }
  return std::sqrt(sum);
}

inline Mat22 Mat22Invert(const Mat22& A) {
  const double det = (A[0][0] * A[1][1]) - (A[0][1] * A[1][0]);
  if (std::fabs(det) < 1e-12) {
    throw std::domain_error("LTVUnicycleController: singular 2x2 matrix");
  }
  const double inv_det = 1.0 / det;
  return {{{A[1][1] * inv_det, -A[0][1] * inv_det},
           {-A[1][0] * inv_det, A[0][0] * inv_det}}};
}

inline Mat33 Mat33Invert(const Mat33& A) {
  const double a00 = A[0][0];
  const double a01 = A[0][1];
  const double a02 = A[0][2];
  const double a10 = A[1][0];
  const double a11 = A[1][1];
  const double a12 = A[1][2];
  const double a20 = A[2][0];
  const double a21 = A[2][1];
  const double a22 = A[2][2];

  const double c00 = (a11 * a22) - (a12 * a21);
  const double c01 = -((a10 * a22) - (a12 * a20));
  const double c02 = (a10 * a21) - (a11 * a20);
  const double c10 = -((a01 * a22) - (a02 * a21));
  const double c11 = (a00 * a22) - (a02 * a20);
  const double c12 = -((a00 * a21) - (a01 * a20));
  const double c20 = (a01 * a12) - (a02 * a11);
  const double c21 = -((a00 * a12) - (a02 * a10));
  const double c22 = (a00 * a11) - (a01 * a10);

  const double det = (a00 * c00) + (a01 * c01) + (a02 * c02);
  if (std::fabs(det) < 1e-12) {
    throw std::domain_error("LTVUnicycleController: singular 3x3 matrix");
  }

  const double inv_det = 1.0 / det;
  return {{{c00 * inv_det, c10 * inv_det, c20 * inv_det},
           {c01 * inv_det, c11 * inv_det, c21 * inv_det},
           {c02 * inv_det, c12 * inv_det, c22 * inv_det}}};
}

inline Vec2 Mat23MulVec3(const Mat23& A, const Vec3& x) {
  Vec2 result{};
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      result[i] += A[i][j] * x[j];
    }
  }
  return result;
}

inline Mat33 MakeCostMatrix3(const Vec3& tolerances) {
  Mat33 result{};
  for (int i = 0; i < 3; ++i) {
    result[i][i] = 1.0 / (tolerances[i] * tolerances[i]);
  }
  return result;
}

inline Mat22 MakeCostMatrix2(const Vec2& tolerances) {
  Mat22 result{};
  for (int i = 0; i < 2; ++i) {
    result[i][i] = 1.0 / (tolerances[i] * tolerances[i]);
  }
  return result;
}

inline Mat33 MakeDiscA(double velocity, double dt) {
  Mat33 A = Mat33Identity();
  A[1][2] = velocity * dt;
  return A;
}

inline Mat32 MakeDiscB(double velocity, double dt) {
  Mat32 B{};
  B[0][0] = dt;
  B[1][1] = 0.5 * velocity * dt * dt;
  B[2][1] = dt;
  return B;
}

inline Mat33 DetailDARE(const Mat33& A, const Mat32& B, const Mat33& Q,
                        const Mat22& R) {
  Mat33 A_k = A;

  Mat32 BRinv{};
  const Mat22 R_inv = Mat22Invert(R);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 2; ++k) {
        BRinv[i][j] += B[i][k] * R_inv[k][j];
      }
    }
  }

  Mat33 G_k{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 2; ++k) {
        G_k[i][j] += BRinv[i][k] * B[j][k];
      }
    }
  }

  Mat33 H_k{};
  Mat33 H_k1 = Q;

  do {
    H_k = H_k1;

    const Mat33 W = Mat33Add(Mat33Identity(), Mat33Mul(G_k, H_k));
    const Mat33 W_inv = Mat33Invert(W);
    const Mat33 V_1 = Mat33Mul(W_inv, A_k);
    const Mat33 V_2 = Mat33Mul(W_inv, G_k);

    G_k = Mat33Add(G_k, Mat33Mul(Mat33Mul(A_k, V_2), Mat33Transpose(A_k)));
    H_k1 = Mat33Add(H_k, Mat33Mul(Mat33Mul(Mat33Transpose(V_1), H_k), A_k));
    A_k = Mat33Mul(A_k, V_1);
  } while (Mat33FrobeniusNorm(Mat33Sub(H_k1, H_k)) >
           1e-10 * Mat33FrobeniusNorm(H_k1));

  return H_k1;
}

inline Mat23 SolveGainAtVelocity(double velocity, const Mat33& Q,
                                 const Mat22& R, double dt) {
  const double A_y_heading =
      std::fabs(velocity) < kNearZeroVelInps ? kNearZeroVelInps : velocity;

  const Mat33 discA = MakeDiscA(A_y_heading, dt);
  const Mat32 discB = MakeDiscB(A_y_heading, dt);
  const Mat33 S = DetailDARE(discA, discB, Q, R);

  const Mat23 discB_T = {{
      {discB[0][0], discB[1][0], discB[2][0]},
      {discB[0][1], discB[1][1], discB[2][1]},
  }};

  const Mat23 BtS = Mat23Mul33(discB_T, S);
  const Mat22 lhs = [](
                         const Mat22& a,
                         const Mat22& b) {
    Mat22 result{};
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        result[i][j] = a[i][j] + b[i][j];
      }
    }
    return result;
  }(Mat23Mul32(BtS, discB), R);

  const Mat22 lhs_inv = Mat22Invert(lhs);
  const Mat23 rhs = Mat23Mul33(BtS, discA);

  Mat23 K{};
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 2; ++k) {
        K[i][j] += lhs_inv[i][k] * rhs[k][j];
      }
    }
  }
  return K;
}

}  // namespace ltv

class LTVUnicycleController {
 public:
  explicit LTVUnicycleController(
      double dt, double maxVelocity = ltv::kWpilibDefaultMaxVelInps)
      : LTVUnicycleController({ltv::kDefaultQxIn, ltv::kDefaultQyIn,
                               ltv::kDefaultQthetaRad},
                              {ltv::kDefaultRvInps, ltv::kDefaultRwRadps}, dt,
                              maxVelocity) {}

  LTVUnicycleController(const ltv::Vec3& Qelems, const ltv::Vec2& Relems,
                        double dt, double maxVelocity = ltv::kWpilibDefaultMaxVelInps) {
    if (maxVelocity <= 0.0) {
      throw std::domain_error("Max velocity must be greater than 0 m/s.");
    }
    if (maxVelocity >= ltv::kMaxVelocityUpperInps) {
      throw std::domain_error("Max velocity must be less than 15 m/s.");
    }

    const ltv::Mat33 Q = ltv::MakeCostMatrix3(Qelems);
    const ltv::Mat22 R = ltv::MakeCostMatrix2(Relems);

    for (double velocity = -maxVelocity; velocity < maxVelocity;
         velocity += ltv::kVelocityStepInps) {
      m_velocities.push_back(velocity);
      m_gains.push_back(ltv::SolveGainAtVelocity(velocity, Q, R, dt));
    }

    m_poseTolerance = {0.5, 0.5, 0.05};
  }

  const ltv::Pose2d& PoseError() const { return m_poseError; }

  bool AtReference() const {
    const auto& eTranslate = m_poseError.Translation();
    const auto& eRotate = m_poseError.Rotation();
    const auto& tolTranslate = m_poseTolerance.Translation();
    const auto& tolRotate = m_poseTolerance.Rotation();

    return std::fabs(eTranslate.X()) < tolTranslate.X() &&
           std::fabs(eTranslate.Y()) < tolTranslate.Y() &&
           std::fabs(eRotate.Radians()) < tolRotate.Radians();
  }

  void SetTolerance(const ltv::Pose2d& poseTolerance) {
    m_poseTolerance = poseTolerance;
  }

  ltv::ChassisSpeeds Calculate(const ltv::Pose2d& currentPose,
                               const ltv::Pose2d& poseRef,
                               double linearVelocityRef,
                               double angularVelocityRef) {
    if (!m_enabled) {
      return {linearVelocityRef, 0.0, angularVelocityRef};
    }

    m_poseError = poseRef.RelativeTo(currentPose);

    const auto& K = GainForVelocity(linearVelocityRef);
    const ltv::Vec3 e{m_poseError.X(), m_poseError.Y(),
                      m_poseError.Rotation().Radians()};
    const ltv::Vec2 u = ltv::Mat23MulVec3(K, e);

    return {linearVelocityRef + u[0], 0.0, angularVelocityRef + u[1]};
  }

  ltv::ChassisSpeeds Calculate(const ltv::Pose2d& currentPose,
                               const ltv::TrajectoryState& desiredState) {
    return Calculate(currentPose, desiredState.pose, desiredState.velocity,
                     desiredState.velocity * desiredState.curvature);
  }

  void SetEnabled(bool enabled) { m_enabled = enabled; }

 private:
  const ltv::Mat23& GainForVelocity(double velocity) const {
    // Clamp the lookup velocity away from zero so low-speed gains retain
    // cross-track authority (the gain table itself is not modified).
    if (std::fabs(velocity) < ltv::kMinLookupVelInps) {
      velocity = velocity < 0.0 ? -ltv::kMinLookupVelInps
                                : ltv::kMinLookupVelInps;
    }

    if (velocity <= m_velocities.front()) {
      return m_gains.front();
    }
    if (velocity >= m_velocities.back()) {
      return m_gains.back();
    }

    const auto it =
        std::lower_bound(m_velocities.begin(), m_velocities.end(), velocity);
    const std::size_t hi = static_cast<std::size_t>(it - m_velocities.begin());
    const std::size_t lo = hi - 1;

    const double lower = m_velocities[lo];
    const double upper = m_velocities[hi];
    const double t = (velocity - lower) / (upper - lower);

    m_interpolatedGain = {};
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 3; ++j) {
        m_interpolatedGain[i][j] =
            ((1.0 - t) * m_gains[lo][i][j]) + (t * m_gains[hi][i][j]);
      }
    }
    return m_interpolatedGain;
  }

  std::vector<double> m_velocities;
  std::vector<ltv::Mat23> m_gains;
  mutable ltv::Mat23 m_interpolatedGain{};
  ltv::Pose2d m_poseError;
  ltv::Pose2d m_poseTolerance;
  bool m_enabled = true;
};
