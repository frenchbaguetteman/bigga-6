"""
Closed-loop simulation of the C++ LTV unicycle controller against the
nonlinear unicycle plant.

Convention: 0 deg = +Y axis, CW-positive heading (matches EZ-Template /
``include/controllers/ltv_controller.hpp``).

Purpose
-------
This script is intended to be used as a regression test for the LTV
controller math. It simulates the closed loop until the tracking errors
have settled to a small steady-state value, defined as:

    * Position error (Euclidean norm of the (x, y) error) < 1.5 inches
    * Heading error                                       < 3 degrees

Both conditions must hold simultaneously for at least
``STEADY_STATE_HOLD_STEPS`` consecutive timesteps (= 0.2 s at dt = 0.01 s)
before convergence is declared. This filters out transient zero-crossings
and noisy single-step dips below the threshold.

If steady state is not reached within ``SOFT_TIMEOUT_S`` the script prints
a warning and keeps simulating (up to ``HARD_TIMEOUT_S``) so that the
behaviour can still be inspected. The script only exits successfully when
both tolerances are satisfied for the full hold window.

Run:  python3 tools/ltv_sim.py
"""

from __future__ import annotations

import math
import sys

import numpy as np


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
IN_TO_M = 0.0254
M_TO_IN = 1.0 / IN_TO_M

dt = 0.01                       # simulation timestep (s) -- matches LTV_DT_S
v_ref_in_s = 24.0               # reference forward velocity (in/s)
omega_ref = 0.0                 # reference angular velocity (rad/s, CW+)

# LQR cost matrices -- mirror the defaults baked into ltv_controller.hpp
Q = np.diag([1.0, 1.0, 10.0])
R = np.diag([1.0, 1.0])

# Convergence tolerances (steady-state acceptance criteria)
POS_TOL_IN   = 1.5              # inches
HEAD_TOL_DEG = 3.0              # degrees
STEADY_STATE_HOLD_STEPS = 20    # 20 * dt = 0.2 s of sustained tolerance

# Time bounds
SOFT_TIMEOUT_S = 10.0           # warn if not converged by this point
HARD_TIMEOUT_S = 30.0           # absolute give-up time (still warns + exits)


# ---------------------------------------------------------------------------
# LTV controller (Python re-implementation of the C++ math)
# ---------------------------------------------------------------------------
def Ad(v_m_s: float) -> np.ndarray:
    """Discrete A matrix (exact ZOH, A_c is nilpotent)."""
    A = np.eye(3)
    A[1, 2] = v_m_s * dt
    return A


def Bd(v_m_s: float) -> np.ndarray:
    """Discrete B matrix (exact ZOH, A_c is nilpotent)."""
    return np.array([
        [dt,  0.0],
        [0.0, 0.5 * v_m_s * dt * dt],
        [0.0, dt],
    ])


def dare(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray,
         tol: float = 1e-9, max_iters: int = 10_000) -> np.ndarray:
    """Solve the discrete algebraic Riccati equation by iteration."""
    P = Q.copy()
    for _ in range(max_iters):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        P_next = A.T @ P @ (A - B @ K) + Q
        P_next = 0.5 * (P_next + P_next.T)        # symmetrize
        if np.max(np.abs(P_next - P)) < tol:
            return P_next
        P = P_next
    return P


def K_for_velocity(v_m_s: float) -> np.ndarray:
    """LQR feedback gain K(v). Floors |v| to avoid an uncontrollable plant."""
    if v_m_s == 0.0:
        v_m_s = 1e-4
    elif abs(v_m_s) < 1e-4:
        v_m_s = math.copysign(1e-4, v_m_s)
    A = Ad(v_m_s)
    B = Bd(v_m_s)
    P = dare(A, B, Q, R)
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


# ---------------------------------------------------------------------------
# Nonlinear unicycle plant (EZ-Template convention: 0 deg = +Y, CW+)
# ---------------------------------------------------------------------------
def plant_step(pose: np.ndarray, v_m_s: float, omega: float) -> np.ndarray:
    x, y, th = pose
    return np.array([
        x + v_m_s * math.sin(th) * dt,
        y + v_m_s * math.cos(th) * dt,
        th + omega * dt,
    ])


def reference(t: float, v_ref_m_s: float) -> np.ndarray:
    """Straight-line reference along +Y at x = 0, heading 0 (= +Y)."""
    return np.array([0.0, v_ref_m_s * t, 0.0])


def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------------
def main() -> int:
    # Initial pose: 6 in off in x, 15 deg of heading error -- a stress test
    # well outside the linearization neighborhood.
    pose = np.array([6.0 * IN_TO_M, 0.0, math.radians(15.0)])
    v_ref_m_s = v_ref_in_s * IN_TO_M

    # Pre-compute the (constant-vref) gain once; gain-schedule lookup would
    # only matter if v_ref changed during the run.
    K = K_for_velocity(v_ref_m_s)

    print("=" * 68)
    print("LTV closed-loop convergence simulation")
    print("=" * 68)
    print(f"  dt                  = {dt*1000:.1f} ms")
    print(f"  v_ref               = {v_ref_in_s:.2f} in/s")
    print(f"  initial pose (in,in,deg) = "
          f"({pose[0]*M_TO_IN:.3f}, {pose[1]*M_TO_IN:.3f}, "
          f"{math.degrees(pose[2]):.3f})")
    print(f"  position tolerance  = {POS_TOL_IN:.2f} in")
    print(f"  heading  tolerance  = {HEAD_TOL_DEG:.2f} deg")
    print(f"  must hold for       = {STEADY_STATE_HOLD_STEPS} steps "
          f"({STEADY_STATE_HOLD_STEPS*dt:.2f} s)")
    print(f"  soft timeout        = {SOFT_TIMEOUT_S:.1f} s")
    print(f"  hard timeout        = {HARD_TIMEOUT_S:.1f} s")
    print("-" * 68)

    in_tol_streak = 0
    converged_iter = -1
    converged_time = -1.0
    soft_warned = False

    pos_err_in = float("inf")
    head_err_deg = float("inf")

    max_iters = int(HARD_TIMEOUT_S / dt) + 1

    for k in range(max_iters):
        t = k * dt
        x_ref = reference(t, v_ref_m_s)

        # Robot-frame error (matches the C++ code exactly).
        s = math.sin(pose[2])
        c = math.cos(pose[2])
        dxW = x_ref[0] - pose[0]
        dyW = x_ref[1] - pose[1]
        e_fwd = s * dxW + c * dyW
        e_lat = c * dxW - s * dyW
        e_th  = wrap_angle(x_ref[2] - pose[2])
        e = np.array([e_fwd, e_lat, e_th])

        # u = u_ref + K e   (sign convention as in ltv_controller.hpp)
        corr = K @ e
        v_cmd     = v_ref_m_s + corr[0]
        omega_cmd = omega_ref + corr[1]

        pose = plant_step(pose, v_cmd, omega_cmd)

        # World-frame position / heading error vs. the reference at this t.
        # We use this for the steady-state acceptance test because it is the
        # quantity an operator actually cares about.
        pos_err_in   = math.hypot(dxW, dyW) * M_TO_IN
        head_err_deg = abs(math.degrees(e_th))

        if pos_err_in < POS_TOL_IN and head_err_deg < HEAD_TOL_DEG:
            in_tol_streak += 1
            if (in_tol_streak == STEADY_STATE_HOLD_STEPS
                    and converged_iter < 0):
                converged_iter = k
                converged_time = t
        else:
            if converged_iter >= 0:
                # We previously thought we converged but the error spiked
                # back out of the band -- retract success and keep looking.
                print(f"  [retract] error left tolerance band at "
                      f"t = {t:.3f} s; resuming search.")
                converged_iter = -1
                converged_time = -1.0
            in_tol_streak = 0

        if (not soft_warned) and t >= SOFT_TIMEOUT_S and converged_iter < 0:
            print(f"  [warn] not yet converged after {SOFT_TIMEOUT_S:.1f} s "
                  f"(pos_err = {pos_err_in:.3f} in, "
                  f"head_err = {head_err_deg:.3f} deg) -- continuing.")
            soft_warned = True

        # Exit the moment we have a sustained streak satisfying both
        # tolerances. The streak counter enforces the 0.2 s hold window.
        if (converged_iter >= 0
                and in_tol_streak >= STEADY_STATE_HOLD_STEPS):
            print("-" * 68)
            print(f"  [ok] steady state reached")
            print(f"       iteration       = {converged_iter}")
            print(f"       time            = {converged_time:.3f} s")
            print(f"       final pos error = {pos_err_in:.4f} in")
            print(f"       final head err  = {head_err_deg:.4f} deg")
            print(f"       held for        = {in_tol_streak} steps "
                  f"({in_tol_streak*dt:.2f} s)")
            print("=" * 68)
            print("LTV CONVERGENCE: PASS")
            return 0

    # Hard timeout reached without sustained convergence.
    print("-" * 68)
    print(f"  [fail] hard timeout ({HARD_TIMEOUT_S:.1f} s) reached without "
          f"sustained convergence.")
    print(f"         last pos error  = {pos_err_in:.4f} in")
    print(f"         last head err   = {head_err_deg:.4f} deg")
    print(f"         best streak     = {in_tol_streak} steps")
    print("=" * 68)
    print("LTV CONVERGENCE: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
