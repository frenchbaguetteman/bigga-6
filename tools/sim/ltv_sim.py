"""
LTV + feedforward + differential-drive simulator.

Goal: reproduce the large steady-state error observed in the PROS terminal log
for ``ltv_path_example`` and validate fixes.

The simulator mirrors the on-robot stack:
  * ``reference_planner.hpp``  -> build_reference_states
  * ``ltv_controller.hpp``     -> LTV unicycle with velocity-scheduled gains
  * ``pid_tasks.cpp``          -> ff/fb wheel voltage pipeline + 10 ms loop

Robot dynamics per wheel:
    V = kS * sign(v) + kV * v + kA * a     (in mV with v in m/s)
rearranged, with voltage-limited bang bang, gives a = (V - kS*sgn(v) - kV*v)/kA.
A tiny viscous term (``kV``) dominates for the linear response; at rest the
static break-away term (``kS``) creates a deadband.

All angles here use the EZ-Template convention (0 deg = +Y axis, CW positive)
to match the on-robot code paths exactly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.linalg import solve_discrete_are


# ---------------------------------------------------------------------------
# Constants (match pid_tasks.cpp)
# ---------------------------------------------------------------------------
DT = 0.010                     # s, control loop period
IN_PER_M = 1.0 / 0.0254
TRACK_WIDTH_IN = 11.40
HALF_TRACK_IN = TRACK_WIDTH_IN / 2.0
MAX_SPEED_INPS = 76.576321
MAX_DRIVE_MV = 12000.0

# Feedforward / velocity PI (per wheel, SI units, mV output)
KS_MV = 1100.0
KV_MV_PER_MPS = 6200.0
KA_MV_PER_MPS2 = 400.0
KVEL_KP = 1300.0
KVEL_KD = 0.0
KVEL_MAX_CORR_MV = 2500.0
VEL_MEAS_ALPHA = 0.35

# LTV cost weights (match s_ltv init)
LTV_Q = (0.5, 0.5, 0.1)        # x_in, y_in, theta_rad
LTV_R = (20.0, 2.0)            # v_inps, w_radps
LTV_MAX_VEL_INPS = 100.0       # build gain table up to this
LTV_MIN_LOOKUP_VEL_INPS = 6.0
LTV_NEAR_ZERO_VEL_INPS = 1e-4 * IN_PER_M

# Reference planner (match reference_trajectory.cpp defaults used for LTV)
REF_SAMPLE_SPACING_IN = 0.5
REF_HEADING_BLEND_IN = 21.0    # = max(10.0, 7.0 * 3.0)  (look_ahead=7" default)
REF_TRACK_WIDTH_IN = 11.338583
REF_MAX_ACCEL_INPS2 = 118.11024  # slew on


# ---------------------------------------------------------------------------
# Small helpers: angle wrap
# ---------------------------------------------------------------------------
def wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def wrap_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


# ---------------------------------------------------------------------------
# Reference planner (Python port of reference_planner.hpp)
# ---------------------------------------------------------------------------
@dataclass
class Waypoint:
    x: float
    y: float
    theta_deg: float = 0.0
    has_explicit_heading: bool = False
    direction: int = +1            # +1 fwd, -1 reverse
    max_vel_inps: float = MAX_SPEED_INPS


@dataclass
class RefState:
    x: float
    y: float
    theta_deg: float
    v: float = 0.0                  # in/s (signed by direction)
    w: float = 0.0                  # rad/s
    a: float = 0.0                  # in/s^2
    alpha: float = 0.0              # rad/s^2
    curvature: float = 0.0
    t: float = 0.0
    s: float = 0.0                  # cumulative distance (in)
    direction: int = +1


def _seg_heading_deg(a: Waypoint, b: Waypoint, direction: int) -> float:
    head = math.degrees(math.atan2(b.x - a.x, b.y - a.y))
    if direction < 0:
        head = wrap_deg(head + 180.0)
    return head


def _wheel_limited_vel(limit: float, curvature: float, track: float) -> float:
    half = track / 2.0
    lscale = abs(1.0 + curvature * half)
    rscale = abs(1.0 - curvature * half)
    return limit / max(1.0, lscale, rscale)


def build_reference_states(
    waypoints: Sequence[Waypoint],
    start: Waypoint,
    *,
    spacing_in: float = REF_SAMPLE_SPACING_IN,
    heading_blend_in: float = REF_HEADING_BLEND_IN,
    track_width_in: float = REF_TRACK_WIDTH_IN,
    max_accel_inps2: float = REF_MAX_ACCEL_INPS2,
    smooth_heading_window_in: float = 0.0,
) -> List[RefState]:
    wps: List[Waypoint] = [
        Waypoint(
            start.x, start.y, start.theta_deg,
            has_explicit_heading=True,
            direction=waypoints[0].direction,
            max_vel_inps=waypoints[0].max_vel_inps,
        )
    ]
    for wp in waypoints:
        if wps and math.hypot(wps[-1].x - wp.x, wps[-1].y - wp.y) < 1e-4:
            wps[-1].direction = wp.direction
            wps[-1].max_vel_inps = wp.max_vel_inps
            if wp.has_explicit_heading:
                wps[-1].theta_deg = wp.theta_deg
                wps[-1].has_explicit_heading = True
            continue
        wps.append(wp)

    if len(wps) <= 1:
        return [RefState(start.x, start.y, start.theta_deg)]

    spacing = min(max(spacing_in, 0.25), 1.0)
    blend = max(6.0, heading_blend_in)
    accel = max(1e-4, abs(max_accel_inps2))

    states: List[RefState] = []
    speed_limits: List[float] = []
    dirs: List[int] = []

    states.append(RefState(start.x, start.y, start.theta_deg,
                           direction=wps[1].direction))
    speed_limits.append(wps[1].max_vel_inps)
    dirs.append(wps[1].direction)

    s_cum = 0.0
    for i in range(len(wps) - 1):
        a, b = wps[i], wps[i + 1]
        seg_len = math.hypot(b.x - a.x, b.y - a.y)
        if seg_len < 1e-4:
            continue
        steps = max(1, int(math.ceil(seg_len / spacing)))
        tan = _seg_heading_deg(a, b, b.direction)
        start_h = a.theta_deg if a.has_explicit_heading else tan
        end_h = b.theta_deg if b.has_explicit_heading else tan
        blend_d = min(seg_len, blend)

        for step in range(1, steps + 1):
            local = seg_len * step / steps
            alpha = local / seg_len
            x = a.x + (b.x - a.x) * alpha
            y = a.y + (b.y - a.y) * alpha
            th = tan
            if a.has_explicit_heading and local < blend_d:
                w = 1.0 - (local / blend_d)
                th = th + wrap_deg(start_h - th) * w
            if b.has_explicit_heading and (seg_len - local) < blend_d:
                w = 1.0 - ((seg_len - local) / blend_d)
                th = th + wrap_deg(end_h - th) * w
            prev = states[-1]
            s_cum += math.hypot(x - prev.x, y - prev.y)
            states.append(RefState(x, y, th, direction=b.direction, s=s_cum))
            speed_limits.append(b.max_vel_inps)
            dirs.append(b.direction)

    # Curvature (finite-diff heading vs. arclength) & wheel-limited speed
    # Optionally smooth heading first -- kills the curvature spikes that
    # occur at segment kinks between waypoints without explicit heading.
    # The smoother preserves the first and last state headings exactly and
    # tapers the window within ``half_win`` of either endpoint so the user's
    # explicit end heading is never moved.
    if smooth_heading_window_in > 0.0 and len(states) >= 3:
        arcs = [s.s for s in states]
        s_total = arcs[-1]
        # Unwrap headings so the smoother doesn't jump across +-180.
        unwrapped = [states[0].theta_deg]
        for k in range(1, len(states)):
            prev = unwrapped[-1]
            unwrapped.append(prev + wrap_deg(states[k].theta_deg - prev))
        half_win = smooth_heading_window_in
        smoothed = [0.0] * len(states)
        for i in range(len(states)):
            # Effective window shrinks toward 0 at both ends (preserve start
            # and end headings exactly).
            win_i = min(half_win, arcs[i], max(0.0, s_total - arcs[i]))
            if win_i < 1e-6:
                smoothed[i] = unwrapped[i]
                continue
            w_sum = 0.0
            val_sum = 0.0
            for j in range(len(states)):
                ds = abs(arcs[j] - arcs[i])
                if ds > win_i:
                    continue
                w = 0.5 * (1.0 + math.cos(math.pi * ds / win_i))
                w_sum += w
                val_sum += w * unwrapped[j]
            smoothed[i] = val_sum / w_sum if w_sum > 0.0 else unwrapped[i]
        for i in range(len(states)):
            states[i].theta_deg = wrap_deg(smoothed[i])

    # Curvature (finite-diff heading vs. arclength) & wheel-limited speed
    for i in range(len(states)):
        lo = i - 1 if i > 0 else 0
        hi = min(i + 1, len(states) - 1)
        ds = states[hi].s - states[lo].s
        curvature = 0.0
        if ds > 1e-4:
            dth_rad = math.radians(wrap_deg(
                states[hi].theta_deg - states[lo].theta_deg))
            curvature = dth_rad / ds
        states[i].curvature = curvature
        speed_limits[i] = _wheel_limited_vel(
            speed_limits[i], curvature, track_width_in)

    # Trapezoidal velocity profile per direction block, 0 at endpoints
    profile = [0.0] * len(states)
    i = 0
    while i < len(states):
        j = i
        while j + 1 < len(states) and dirs[j + 1] == dirs[i]:
            j += 1
        profile[i] = 0.0
        for k in range(i + 1, j + 1):
            ds = states[k].s - states[k - 1].s
            limit = math.sqrt(max(0.0, profile[k - 1] ** 2 + 2.0 * accel * ds))
            profile[k] = min(speed_limits[k], limit)
        profile[j] = 0.0
        for k in range(j - 1, i - 1, -1):
            ds = states[k + 1].s - states[k].s
            limit = math.sqrt(max(0.0, profile[k + 1] ** 2 + 2.0 * accel * ds))
            profile[k] = min(profile[k], limit)
        sign = float(dirs[i])
        for k in range(i, j + 1):
            states[k].v = profile[k] * sign
            states[k].w = states[k].v * states[k].curvature
        i = j + 1

    # Integrate time
    for i in range(1, len(states)):
        ds = states[i].s - states[i - 1].s
        v0 = abs(states[i - 1].v)
        v1 = abs(states[i].v)
        vsum = v0 + v1
        dt = 2.0 * ds / vsum if vsum > 1e-4 else ds / max(speed_limits[i], 1.0)
        states[i].t = states[i - 1].t + dt

    # Recompute w from dtheta/dt (match planner)
    for i in range(len(states)):
        lo = i - 1 if i > 0 else 0
        hi = min(i + 1, len(states) - 1)
        dt = states[hi].t - states[lo].t
        if dt > 1e-4:
            dth = math.radians(wrap_deg(
                states[hi].theta_deg - states[lo].theta_deg))
            states[i].w = dth / dt

    # Accels
    for i in range(len(states)):
        lo = i - 1 if i > 0 else 0
        hi = min(i + 1, len(states) - 1)
        dt = states[hi].t - states[lo].t
        if dt > 1e-4:
            states[i].a = (states[hi].v - states[lo].v) / dt
            states[i].alpha = (states[hi].w - states[lo].w) / dt

    return states


def sample_reference(states: List[RefState], t: float) -> RefState:
    if not states:
        return RefState(0, 0, 0)
    if t <= 0.0 or len(states) == 1:
        return states[0]
    if t >= states[-1].t:
        return states[-1]
    # linear scan is fine for sim speed
    for i in range(1, len(states)):
        if states[i].t >= t:
            lo, hi = states[i - 1], states[i]
            rng = hi.t - lo.t
            a = (t - lo.t) / rng if rng > 1e-4 else 0.0
            out = RefState(
                lo.x + (hi.x - lo.x) * a,
                lo.y + (hi.y - lo.y) * a,
                lo.theta_deg + wrap_deg(hi.theta_deg - lo.theta_deg) * a,
                v=lo.v + (hi.v - lo.v) * a,
                w=lo.w + (hi.w - lo.w) * a,
                a=lo.a + (hi.a - lo.a) * a,
                alpha=lo.alpha + (hi.alpha - lo.alpha) * a,
                curvature=lo.curvature + (hi.curvature - lo.curvature) * a,
                t=t,
                s=lo.s + (hi.s - lo.s) * a,
                direction=lo.direction if a < 0.5 else hi.direction,
            )
            return out
    return states[-1]


# ---------------------------------------------------------------------------
# LTV controller (DARE from scipy) -- operates in LTV frame (CCW+)
# ---------------------------------------------------------------------------
class LTVUnicycle:
    def __init__(self, Q=LTV_Q, R=LTV_R, dt=DT,
                 v_max_inps=LTV_MAX_VEL_INPS,
                 v_step_inps=0.01 * IN_PER_M):
        self.dt = dt
        self.v_min_lookup = LTV_MIN_LOOKUP_VEL_INPS
        Qm = np.diag([1.0 / q ** 2 for q in Q])
        Rm = np.diag([1.0 / r ** 2 for r in R])
        self.vels: List[float] = []
        self.gains: List[np.ndarray] = []
        v = -v_max_inps
        while v < v_max_inps:
            self.vels.append(v)
            self.gains.append(self._solve_K(v, Qm, Rm, dt))
            v += v_step_inps
        self.vels_np = np.array(self.vels)

    @staticmethod
    def _solve_K(v: float, Q: np.ndarray, R: np.ndarray, dt: float) -> np.ndarray:
        vA = v if abs(v) >= LTV_NEAR_ZERO_VEL_INPS else math.copysign(LTV_NEAR_ZERO_VEL_INPS, v or 1.0)
        A = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, vA * dt],
                      [0.0, 0.0, 1.0]])
        B = np.array([[dt, 0.0],
                      [0.0, 0.5 * vA * dt * dt],
                      [0.0, dt]])
        try:
            S = solve_discrete_are(A, B, Q, R)
        except np.linalg.LinAlgError:
            S = Q.copy()
        K = np.linalg.solve(R + B.T @ S @ B, B.T @ S @ A)
        return K

    def gain(self, v_ref: float) -> np.ndarray:
        vq = v_ref
        if abs(vq) < self.v_min_lookup:
            vq = math.copysign(self.v_min_lookup, vq if vq != 0 else 1.0)
        if vq <= self.vels_np[0]:
            return self.gains[0]
        if vq >= self.vels_np[-1]:
            return self.gains[-1]
        idx = int(np.searchsorted(self.vels_np, vq))
        lo, hi = idx - 1, idx
        t = (vq - self.vels_np[lo]) / (self.vels_np[hi] - self.vels_np[lo])
        return (1 - t) * self.gains[lo] + t * self.gains[hi]

    def calculate(self, cur_pose_ltv: Tuple[float, float, float],
                  ref_pose_ltv: Tuple[float, float, float],
                  v_ref: float, w_ref: float) -> Tuple[float, float, np.ndarray]:
        cx, cy, ct = cur_pose_ltv
        rx, ry, rt = ref_pose_ltv
        dx, dy = rx - cx, ry - cy
        c, s = math.cos(ct), math.sin(ct)
        ex = c * dx + s * dy
        ey = -s * dx + c * dy
        eth = wrap_rad(rt - ct)
        K = self.gain(v_ref)
        u = K @ np.array([ex, ey, eth])
        return v_ref + u[0], w_ref + u[1], np.array([ex, ey, eth])


# ---------------------------------------------------------------------------
# Robot dynamics: per-wheel v_dot = (V - kS*sign(v) - kV*v) / kA with deadband
# ---------------------------------------------------------------------------
@dataclass
class SimConfig:
    # bot dynamics
    ks_mv: float = KS_MV
    kv_mv_per_mps: float = KV_MV_PER_MPS
    ka_mv_per_mps2: float = KA_MV_PER_MPS2
    track_width_in: float = TRACK_WIDTH_IN
    # Realism: motor response lag (1st-order low-pass on voltage, models
    # the V5 motor's internal controller latency).
    motor_lag_tau_s: float = 0.02
    # Realism: left/right motor mismatch (fraction, e.g. 0.05 = 5%).
    # Left kV multiplied by (1 - mismatch/2), right by (1 + mismatch/2),
    # so the left side is slightly faster for a given voltage.  This causes
    # position drift during pure-spin commands, matching real robot behaviour.
    motor_mismatch_pct: float = 0.05
    # pipeline
    ff_ks_mv: float = KS_MV
    ff_kv_mv_per_mps: float = KV_MV_PER_MPS
    ff_ka_mv_per_mps2: float = KA_MV_PER_MPS2
    kvel_kp: float = KVEL_KP
    kvel_kd: float = KVEL_KD
    max_correction_mv: float = KVEL_MAX_CORR_MV
    vel_meas_alpha: float = VEL_MEAS_ALPHA
    # settling-phase fix (disabled = current behaviour)
    settle_enabled: bool = False
    settle_hold_s: float = 0.8           # trajectory tail padded with hold-pose
    settle_max_s: float = 1.2            # hard cap on extra settling time
    settle_xy_tol_in: float = 0.75
    settle_ang_tol_deg: float = 1.5
    settle_stable_s: float = 0.15
    # Disable noisy FF differentiators (based on per-tick cmd deltas) when the
    # commanded velocity is nearly zero.  The d(cmd)/dt term just amplifies
    # feedback chatter into ka/alpha mV and fights the velocity PI.
    suppress_ff_deriv_low_v: bool = False
    ff_deriv_v_cut_inps: float = 6.0
    ff_deriv_w_cut_radps: float = 1.5
    # Snap to first-order shape of commanded velocity: low-pass v_cmd / w_cmd
    # before they feed the FF chain.  Tau in seconds, 0 = no filter.
    cmd_filter_tau_s: float = 0.0
    # Pose-space heading boost while in the holding tail (rad/s per rad err).
    low_v_heading_boost: float = 0.0
    # --- Voltage-domain settler (V2: damped turn-drive-turn) ---------------
    # Dedicated settling controller, bypasses LTV+FF velocity pipeline.
    # Turn-drive-turn with angular-rate damping and locked phase-1 heading
    # to prevent the runaway spinning observed on the real robot with V1.
    settle_use_voltage_ctrl: bool = False
    settle_kp_hdg_mv_per_rad: float = 12000.0    # P gain for heading
    settle_kd_mv_per_radps: float = 500.0         # D gain (angular rate damping)
    settle_hdg_deadband_deg: float = 0.3
    settle_min_turn_kick_mv: float = 1400.0       # >= kS + margin
    settle_omega_kick_thresh_degps: float = 15.0   # only apply min kick below this rate
    settle_max_turn_mv: float = 4000.0            # max turn millivolts (was 6000)
    settle_max_fwd_mv: float = 2500.0             # max forward millivolts
    settle_fwd_min_mv: float = 1500.0             # min fwd mV (above kS)
    settle_kp_fwd_mv_per_in: float = 900.0
    # Approach pass: turn-drive-turn
    settle_enable_approach: bool = False
    settle_approach_xy_trigger_in: float = 1.0
    settle_phase1_tol_deg: float = 10.0           # heading tolerance for ph1->ph2 (was 2°)
    # Smooth the reference heading across segment kinks (cosine window).
    ref_heading_smooth_in: float = 0.0


@dataclass
class SimState:
    x: float = 0.0
    y: float = 0.0
    theta_deg: float = 0.0          # CUMULATIVE (not wrapped), like real odom
    v_left_mps: float = 0.0         # true wheel velocity
    v_right_mps: float = 0.0
    v_left_meas: float = 0.0        # filtered measurement
    v_right_meas: float = 0.0
    prev_cmd_v_inps: float = 0.0
    prev_cmd_w_radps: float = 0.0
    prev_left_err_mps: float = 0.0
    prev_right_err_mps: float = 0.0
    # Motor voltage lag (effective voltage seen by wheel model)
    v_eff_left_mv: float = 0.0
    v_eff_right_mv: float = 0.0
    # Settler angular-rate tracking
    prev_theta_deg: float = 0.0


def make_ltv_pose(x_ez, y_ez, th_ez_deg):
    # LTV frame: x = y_ez, y = -x_ez, theta = -theta_ez (CCW+)
    return (y_ez, -x_ez, -math.radians(th_ez_deg))


def mv_to_cmd127(mv: float) -> int:
    mv = max(-MAX_DRIVE_MV, min(MAX_DRIVE_MV, mv))
    return int(round(mv * (127.0 / MAX_DRIVE_MV)))


def cmd127_to_mv(cmd: int) -> float:
    return max(-127, min(127, cmd)) * (MAX_DRIVE_MV / 127.0)


def run_simulation(
    waypoints: Sequence[Waypoint],
    start: Waypoint,
    *,
    config: Optional[SimConfig] = None,
    extra_settle_time_s: float = 1.2,
    verbose_every: int = 0,
) -> dict:
    cfg = config or SimConfig()
    states = build_reference_states(
        waypoints, start,
        smooth_heading_window_in=cfg.ref_heading_smooth_in)
    total_time_s = states[-1].t
    ltv = LTVUnicycle()

    sim = SimState(x=start.x, y=start.y, theta_deg=start.theta_deg,
                   prev_theta_deg=start.theta_deg)
    log = []
    settle_stable_accum = 0.0
    finished = False

    t_end_tracking = total_time_s + (cfg.settle_hold_s if cfg.settle_enabled else 0.0)
    t_max_sim = t_end_tracking + extra_settle_time_s + (
        cfg.settle_max_s if cfg.settle_enabled else 0.0)

    # Low-pass state for optional cmd filtering
    v_cmd_filt = 0.0
    w_cmd_filt = 0.0

    step = 0
    prev_cmd_valid = False
    t = 0.0
    final_ref = states[-1]
    # Approach state machine for damped turn-drive-turn recovery (V2)
    approach_phase = 0   # 0 hold/heading-only, 1 turn-to-target, 2 drive, 3 turn-to-final, 4 done
    approach_locked_bearing_deg = 0.0     # LOCKED at phase-1 entry (no recompute)
    final_target_heading_deg = states[-1].theta_deg
    while t <= t_max_sim:
        # 1) reference sample
        traj_time = total_time_s
        in_traj = t < traj_time
        in_hold = (not in_traj) and (t < t_end_tracking) and cfg.settle_enabled
        if in_traj:
            ref = sample_reference(states, t)
        else:
            # hold the last target, zero feedforward velocity
            ref = RefState(final_ref.x, final_ref.y, final_ref.theta_deg,
                           v=0.0, w=0.0, a=0.0, alpha=0.0,
                           curvature=0.0, t=t, s=final_ref.s,
                           direction=final_ref.direction)

        # 2) Tracking controller.
        cur_ltv = make_ltv_pose(sim.x, sim.y, sim.theta_deg)
        ref_ltv = make_ltv_pose(ref.x, ref.y, ref.theta_deg)
        v_cmd_inps, w_cmd_radps_ltv, e_ltv = ltv.calculate(
            cur_ltv, ref_ltv, ref.v, -ref.w)
        w_cmd_radps = -w_cmd_radps_ltv   # flip back to EZ CW+ convention
        err_fwd = e_ltv[0]
        err_lat = -e_ltv[1]
        err_hdg = -e_ltv[2]

        use_voltage_settle = (
            cfg.settle_enabled
            and cfg.settle_use_voltage_ctrl
            and not in_traj
        )

        if use_voltage_settle:
            # --- V2 settler: damped turn-drive-turn with locked heading ---
            target_x = final_ref.x
            target_y = final_ref.y
            dx_world = target_x - sim.x
            dy_world = target_y - sim.y
            xy_err_now = math.hypot(dx_world, dy_world)

            # Bearing from robot to target (EZ: 0°=+Y, CW+)
            angle_to_target_deg = wrap_deg(math.degrees(
                math.atan2(dx_world, dy_world)))

            hdg_err_final_deg = wrap_deg(
                final_target_heading_deg - sim.theta_deg)
            hdg_err_final_rad = math.radians(hdg_err_final_deg)

            # Angular rate estimation (deg/s)
            omega_degps = wrap_deg(sim.theta_deg - sim.prev_theta_deg) / DT

            # --- Phase transitions ---
            # Phase 0 → 1: heading roughly aligned to final, xy still large
            if (cfg.settle_enable_approach
                    and approach_phase == 0
                    and abs(hdg_err_final_deg) < cfg.settle_ang_tol_deg * 1.5
                    and xy_err_now > cfg.settle_approach_xy_trigger_in):
                approach_phase = 1
                # LOCK the bearing at phase-1 entry (prevents chasing)
                approach_locked_bearing_deg = angle_to_target_deg

            # Phase 1 → 2: heading within tolerance of LOCKED bearing
            if approach_phase == 1:
                err_to_locked_deg = wrap_deg(
                    approach_locked_bearing_deg - sim.theta_deg)
                if abs(err_to_locked_deg) < cfg.settle_phase1_tol_deg:
                    approach_phase = 2

            # Phase 2 → 3: close enough to target
            elif approach_phase == 2:
                if xy_err_now < 0.5:
                    approach_phase = 3

            # Phase 3 → 4: heading aligned to final
            elif approach_phase == 3:
                if abs(hdg_err_final_deg) < cfg.settle_ang_tol_deg:
                    approach_phase = 4

            # --- Select target heading and forward flag ---
            if approach_phase == 1:
                drive_heading_deg = approach_locked_bearing_deg  # LOCKED
                want_drive_forward = False
            elif approach_phase == 2:
                drive_heading_deg = angle_to_target_deg  # live bearing OK (robot is close)
                want_drive_forward = True
            else:
                drive_heading_deg = final_target_heading_deg
                want_drive_forward = False

            # --- P + D heading controller ---
            err_hdg_drive_deg = wrap_deg(drive_heading_deg - sim.theta_deg)
            err_hdg_drive_rad = math.radians(err_hdg_drive_deg)
            omega_radps = math.radians(omega_degps)

            # P term
            v_p = cfg.settle_kp_hdg_mv_per_rad * err_hdg_drive_rad
            # D term (angular rate damping — always active)
            v_d = -cfg.settle_kd_mv_per_radps * omega_radps

            # Static friction kick: only apply when robot is nearly stationary
            # (avoids boosting during deceleration)
            if (abs(err_hdg_drive_deg) > cfg.settle_hdg_deadband_deg and
                    abs(omega_degps) < cfg.settle_omega_kick_thresh_degps):
                if abs(v_p) < cfg.settle_min_turn_kick_mv:
                    v_p = math.copysign(cfg.settle_min_turn_kick_mv, v_p)
            elif abs(err_hdg_drive_deg) <= cfg.settle_hdg_deadband_deg:
                v_p = 0.0  # deadband suppresses P

            v_turn_mv = max(-cfg.settle_max_turn_mv,
                            min(cfg.settle_max_turn_mv, v_p + v_d))

            # --- Forward drive ---
            v_fwd_mv = 0.0
            if want_drive_forward and abs(err_hdg_drive_deg) < 20.0:
                v_fwd_mv = cfg.settle_kp_fwd_mv_per_in * xy_err_now
                v_fwd_mv = math.copysign(
                    max(abs(v_fwd_mv), cfg.settle_fwd_min_mv), 1.0)
                v_fwd_mv = min(v_fwd_mv, cfg.settle_max_fwd_mv)

            left_out_mv = v_fwd_mv + v_turn_mv
            right_out_mv = v_fwd_mv - v_turn_mv

            ff_left_mv = left_out_mv
            ff_right_mv = right_out_mv
            fb_left = 0.0
            fb_right = 0.0
            vl_des_inps = 0.0
            vr_des_inps = 0.0
            v_cmd_filt = 0.0
            w_cmd_filt = 0.0
            err_hdg = -math.radians(wrap_deg(
                final_target_heading_deg - sim.theta_deg))
        else:
            # Optional: extra heading authority when nearly stopped at final target
            if (not in_traj) and cfg.low_v_heading_boost > 0:
                w_cmd_radps += cfg.low_v_heading_boost * err_hdg

            # Optional: low-pass the commanded v/w to smooth FF derivatives
            if cfg.cmd_filter_tau_s > 0.0:
                alpha_cmd = DT / (cfg.cmd_filter_tau_s + DT)
                v_cmd_filt = v_cmd_filt + alpha_cmd * (v_cmd_inps - v_cmd_filt)
                w_cmd_filt = w_cmd_filt + alpha_cmd * (w_cmd_radps - w_cmd_filt)
            else:
                v_cmd_filt = v_cmd_inps
                w_cmd_filt = w_cmd_radps

            # 3) wheel-speed limit (on filtered commands)
            half_track = cfg.track_width_in / 2.0
            vl_des_inps = v_cmd_filt + w_cmd_filt * half_track
            vr_des_inps = v_cmd_filt - w_cmd_filt * half_track
            scale = max(1.0, abs(vl_des_inps) / MAX_SPEED_INPS,
                        abs(vr_des_inps) / MAX_SPEED_INPS)
            vl_des_inps /= scale
            vr_des_inps /= scale

            # 4) feedforward + feedback on measured wheel velocities
            dt = DT
            if prev_cmd_valid:
                lin_acc = (v_cmd_filt - sim.prev_cmd_v_inps) / dt
                ang_acc = (w_cmd_filt - sim.prev_cmd_w_radps) / dt
            else:
                lin_acc = ref.a
                ang_acc = ref.alpha

            if cfg.suppress_ff_deriv_low_v:
                if (abs(v_cmd_filt) < cfg.ff_deriv_v_cut_inps and
                        abs(w_cmd_filt) < cfg.ff_deriv_w_cut_radps):
                    lin_acc = 0.0
                    ang_acc = 0.0

            v_mps = v_cmd_filt * 0.0254
            om_rad = w_cmd_filt
            a_mps2 = lin_acc * 0.0254
            al_rad2 = ang_acc
            half_m = cfg.track_width_in * 0.0254 / 2.0
            vl_mps = v_mps + om_rad * half_m
            vr_mps = v_mps - om_rad * half_m
            al_left = a_mps2 + al_rad2 * half_m
            al_right = a_mps2 - al_rad2 * half_m

            def ff_side(v, a):
                ks_sign = (1.0 if v > 0 else (-1.0 if v < 0 else 0.0))
                return cfg.ff_ks_mv * ks_sign + cfg.ff_kv_mv_per_mps * v + cfg.ff_ka_mv_per_mps2 * a

            ff_left_mv = ff_side(vl_mps, al_left)
            ff_right_mv = ff_side(vr_mps, al_right)

            # filtered measurement
            if prev_cmd_valid:
                sim.v_left_meas = (cfg.vel_meas_alpha * sim.v_left_mps +
                                   (1 - cfg.vel_meas_alpha) * sim.v_left_meas)
                sim.v_right_meas = (cfg.vel_meas_alpha * sim.v_right_mps +
                                    (1 - cfg.vel_meas_alpha) * sim.v_right_meas)
            else:
                sim.v_left_meas = sim.v_left_mps
                sim.v_right_meas = sim.v_right_mps

            des_left_mps = vl_des_inps * 0.0254
            des_right_mps = vr_des_inps * 0.0254
            err_l = des_left_mps - sim.v_left_meas
            err_r = des_right_mps - sim.v_right_meas
            derr_l = (err_l - sim.prev_left_err_mps) / dt if prev_cmd_valid else 0.0
            derr_r = (err_r - sim.prev_right_err_mps) / dt if prev_cmd_valid else 0.0
            fb_left = max(-cfg.max_correction_mv, min(cfg.max_correction_mv,
                          cfg.kvel_kp * err_l + cfg.kvel_kd * derr_l))
            fb_right = max(-cfg.max_correction_mv, min(cfg.max_correction_mv,
                           cfg.kvel_kp * err_r + cfg.kvel_kd * derr_r))

            left_out_mv = max(-MAX_DRIVE_MV, min(MAX_DRIVE_MV, ff_left_mv + fb_left))
            right_out_mv = max(-MAX_DRIVE_MV, min(MAX_DRIVE_MV, ff_right_mv + fb_right))

        # 5) quantize and simulate
        left_out_mv = max(-MAX_DRIVE_MV, min(MAX_DRIVE_MV, left_out_mv))
        right_out_mv = max(-MAX_DRIVE_MV, min(MAX_DRIVE_MV, right_out_mv))
        left_cmd = mv_to_cmd127(left_out_mv)
        right_cmd = mv_to_cmd127(right_out_mv)
        V_left_cmd = cmd127_to_mv(left_cmd)
        V_right_cmd = cmd127_to_mv(right_cmd)
        dt = DT

        # Motor voltage lag (1st-order low-pass models V5 internal controller)
        if cfg.motor_lag_tau_s > 0:
            alpha_lag = dt / (cfg.motor_lag_tau_s + dt)
            sim.v_eff_left_mv += alpha_lag * (V_left_cmd - sim.v_eff_left_mv)
            sim.v_eff_right_mv += alpha_lag * (V_right_cmd - sim.v_eff_right_mv)
        else:
            sim.v_eff_left_mv = V_left_cmd
            sim.v_eff_right_mv = V_right_cmd

        # Motor mismatch: left side runs slightly faster (lower effective kV)
        kv_left = cfg.kv_mv_per_mps * (1.0 - cfg.motor_mismatch_pct / 2.0)
        kv_right = cfg.kv_mv_per_mps * (1.0 + cfg.motor_mismatch_pct / 2.0)

        # 5b) wheel dynamics: v_dot = (V_eff - ks*sign(v) - kv*v) / ka
        def wheel_deriv(V, v, kv):
            if abs(v) < 1e-4 and abs(V) < cfg.ks_mv:
                return 0.0
            ks_term = cfg.ks_mv * (1.0 if v > 1e-4 else (-1.0 if v < -1e-4 else math.copysign(1.0, V)))
            return (V - ks_term - kv * v) / cfg.ka_mv_per_mps2

        # RK2 mid-point for stability at low ka
        al = wheel_deriv(sim.v_eff_left_mv, sim.v_left_mps, kv_left)
        ar = wheel_deriv(sim.v_eff_right_mv, sim.v_right_mps, kv_right)
        vl_mid = sim.v_left_mps + 0.5 * dt * al
        vr_mid = sim.v_right_mps + 0.5 * dt * ar
        al2 = wheel_deriv(sim.v_eff_left_mv, vl_mid, kv_left)
        ar2 = wheel_deriv(sim.v_eff_right_mv, vr_mid, kv_right)
        sim.v_left_mps += dt * al2
        sim.v_right_mps += dt * ar2

        # 6) pose integration: EZ convention (0 deg = +Y, CW+)
        v_bot_mps = 0.5 * (sim.v_left_mps + sim.v_right_mps)
        w_bot_rad = (sim.v_left_mps - sim.v_right_mps) / (cfg.track_width_in * 0.0254)  # CW+
        v_bot_inps = v_bot_mps * IN_PER_M
        th_rad = math.radians(sim.theta_deg)
        # In EZ frame (0 deg = +Y, CW+): dx/dt = v*sin(th), dy/dt = v*cos(th)
        sim.prev_theta_deg = sim.theta_deg
        sim.x += v_bot_inps * math.sin(th_rad) * dt
        sim.y += v_bot_inps * math.cos(th_rad) * dt
        sim.theta_deg += math.degrees(w_bot_rad) * dt  # cumulative (not wrapped)

        # 7) update carry-overs
        sim.prev_cmd_v_inps = v_cmd_filt
        sim.prev_cmd_w_radps = w_cmd_filt
        if not use_voltage_settle:
            sim.prev_left_err_mps = err_l
            sim.prev_right_err_mps = err_r
        prev_cmd_valid = True

        # 8) log
        xy_err = math.hypot(final_ref.x - sim.x, final_ref.y - sim.y)
        hdg_err_deg = abs(wrap_deg(final_ref.theta_deg - sim.theta_deg))
        log.append(dict(
            t=t, x=sim.x, y=sim.y, th=sim.theta_deg,
            rx=ref.x, ry=ref.y, rth=ref.theta_deg,
            vref=ref.v, wref=ref.w,
            cmd_v=v_cmd_inps, cmd_w=w_cmd_radps,
            err_fwd=err_fwd, err_lat=err_lat, err_hdg=err_hdg,
            des_l=vl_des_inps, des_r=vr_des_inps,
            meas_l=sim.v_left_mps * IN_PER_M, meas_r=sim.v_right_mps * IN_PER_M,
            ff_l=ff_left_mv, ff_r=ff_right_mv,
            fb_l=fb_left, fb_r=fb_right,
            cmd_left=left_cmd, cmd_right=right_cmd,
            xy_err=xy_err, hdg_err_deg=hdg_err_deg,
            in_traj=in_traj,
        ))

        # 9) early-exit condition when settling converges
        if cfg.settle_enabled and not in_traj:
            if (xy_err < cfg.settle_xy_tol_in and
                    hdg_err_deg < cfg.settle_ang_tol_deg):
                settle_stable_accum += dt
                if settle_stable_accum >= cfg.settle_stable_s:
                    finished = True
                    break
            else:
                settle_stable_accum = 0.0

        if verbose_every and step % verbose_every == 0:
            print(f"  t={t:.2f} pos=({sim.x:.2f},{sim.y:.2f},{sim.theta_deg:.1f}) "
                  f"ref=({ref.x:.2f},{ref.y:.2f},{ref.theta_deg:.1f}) "
                  f"xy_err={xy_err:.2f} hdg_err={hdg_err_deg:.2f}")

        t += DT
        step += 1

        # stop-when-tracking-done behaviour (matches current firmware)
        if not cfg.settle_enabled and not in_traj:
            # firmware exits immediately at total_time_s; but we run a tiny
            # tail (extra_settle_time_s) purely for observation.
            if t - traj_time > extra_settle_time_s:
                break

    final_xy = math.hypot(final_ref.x - sim.x, final_ref.y - sim.y)
    final_hdg = abs(wrap_deg(final_ref.theta_deg - sim.theta_deg))
    return dict(
        log=log,
        total_time_s=total_time_s,
        final_xy_err_in=final_xy,
        final_hdg_err_deg=final_hdg,
        final_pose=(sim.x, sim.y, sim.theta_deg),
        target_pose=(final_ref.x, final_ref.y, final_ref.theta_deg),
        sim_stopped_at_s=t,
        finished_within_settling=finished,
        plan=states,
    )


# ---------------------------------------------------------------------------
# Scenario from the terminal log: ltv_path_example
#     (0,0,0) -> (8,16)@60 -> (-8,32)@60 -> (0,48,0°)@55
# ---------------------------------------------------------------------------
LTV_PATH_START = Waypoint(0.0, 0.0, 0.0, has_explicit_heading=True)
LTV_PATH_WAYPOINTS = [
    Waypoint(8.0, 16.0, 0.0, has_explicit_heading=False, direction=+1,
             max_vel_inps=MAX_SPEED_INPS * (60.0 / 127.0)),
    Waypoint(-8.0, 32.0, 0.0, has_explicit_heading=False, direction=+1,
             max_vel_inps=MAX_SPEED_INPS * (60.0 / 127.0)),
    Waypoint(0.0, 48.0, 0.0, has_explicit_heading=True, direction=+1,
             max_vel_inps=MAX_SPEED_INPS * (55.0 / 127.0)),
]


def run_scenario(label: str, config: Optional[SimConfig] = None,
                 verbose: bool = False) -> dict:
    result = run_simulation(
        LTV_PATH_WAYPOINTS, LTV_PATH_START,
        config=config,
        verbose_every=10 if verbose else 0,
    )
    print(f"[{label}] traj_time={result['total_time_s']:.2f}s "
          f"sim_end={result['sim_stopped_at_s']:.2f}s "
          f"final_xy_err={result['final_xy_err_in']:.3f} in  "
          f"final_hdg_err={result['final_hdg_err_deg']:.2f} deg  "
          f"pose={tuple(round(p,2) for p in result['final_pose'])}  "
          f"target={tuple(round(p,2) for p in result['target_pose'])}")
    return result


if __name__ == "__main__":
    print("=== baseline (current firmware behaviour) ===")
    run_scenario("baseline", SimConfig(settle_enabled=False))

    print("\n=== V1 settler (spinning, matches real robot failure) ===")
    # Use V1-style high gains to reproduce the spinning the user saw.
    # Motor lag + mismatch make this match the real robot's drift.
    run_scenario("V1-spin", SimConfig(
        settle_enabled=True,
        settle_hold_s=0.05,
        settle_max_s=3.5,
        settle_xy_tol_in=0.75,
        settle_ang_tol_deg=1.2,
        settle_stable_s=0.15,
        settle_use_voltage_ctrl=True,
        settle_enable_approach=True,
        settle_approach_xy_trigger_in=1.0,
        # V1 parameters that caused the spinning:
        settle_kp_hdg_mv_per_rad=14000.0,
        settle_kd_mv_per_radps=0.0,        # no damping!
        settle_max_turn_mv=6000.0,          # way too aggressive
        settle_omega_kick_thresh_degps=999.0,  # always apply min kick
        settle_phase1_tol_deg=2.0,          # too tight transition
    ))

    print("\n=== V2 settler (damped turn-drive-turn, fixed) ===")
    run_scenario("V2-fixed", SimConfig(
        settle_enabled=True,
        settle_hold_s=0.05,
        settle_max_s=5.0,
        settle_xy_tol_in=0.75,
        settle_ang_tol_deg=1.2,
        settle_stable_s=0.15,
        settle_use_voltage_ctrl=True,
        settle_enable_approach=True,
        settle_approach_xy_trigger_in=1.0,
    ))
