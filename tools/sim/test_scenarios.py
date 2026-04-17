"""Robustness test of the LTV + voltage-settler fix across several path shapes."""
from ltv_sim import (
    Waypoint, SimConfig, run_simulation,
    LTV_PATH_WAYPOINTS, LTV_PATH_START, MAX_SPEED_INPS,
)


def vmax(pct: float) -> float:
    return MAX_SPEED_INPS * (pct / 127.0)


SCENARIOS = {
    "ltv_path": (LTV_PATH_WAYPOINTS, LTV_PATH_START),
    "single_fwd_24": (
        [Waypoint(0.0, 24.0, 0.0, True, +1, vmax(110))],
        Waypoint(0.0, 0.0, 0.0, True),
    ),
    "diag_to_45": (
        [Waypoint(12.0, 36.0, 45.0, True, +1, vmax(110))],
        Waypoint(0.0, 0.0, 0.0, True),
    ),
    "return_home": (
        [Waypoint(0.0, 0.0, 0.0, True, -1, vmax(110))],
        Waypoint(12.0, 36.0, 45.0, True),
    ),
    "sharp_turn_90": (
        [Waypoint(24.0, 0.0, 90.0, True, +1, vmax(80))],
        Waypoint(0.0, 0.0, 0.0, True),
    ),
}


def run_case(name, wps, start):
    base = run_simulation(
        wps, start,
        config=SimConfig(settle_enabled=False),
        extra_settle_time_s=0.5,
    )
    fix = run_simulation(
        wps, start,
        config=SimConfig(
            settle_enabled=True,
            settle_hold_s=0.05,
            settle_max_s=5.0,
            settle_xy_tol_in=0.75,
            settle_ang_tol_deg=1.2,
            settle_stable_s=0.15,
            settle_use_voltage_ctrl=True,
            settle_enable_approach=True,
            settle_approach_xy_trigger_in=1.0,
            motor_lag_tau_s=0.02,
            motor_mismatch_pct=0.05,
        ),
        extra_settle_time_s=0.5,
    )

    def fmt(r, tag):
        p = r["final_pose"]
        return (
            f"  {tag}: xy={r['final_xy_err_in']:5.3f}in "
            f"hdg={r['final_hdg_err_deg']:6.2f}deg "
            f"t_end={r['sim_stopped_at_s']:4.2f}s "
            f"pose=({p[0]:6.2f}, {p[1]:6.2f}, {p[2]:6.2f})"
        )

    print(f"[{name}]")
    print(fmt(base, "base"))
    print(fmt(fix, "fix "))
    ok = fix["final_xy_err_in"] < 1.5 and abs(fix["final_hdg_err_deg"]) < 3.0
    print(f"  -> {'PASS' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    results = [run_case(n, *s) for n, s in SCENARIOS.items()]
    print()
    print(f"Summary: {sum(results)}/{len(results)} scenarios within 1.5\" / 3° spec.")
