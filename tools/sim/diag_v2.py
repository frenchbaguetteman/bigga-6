"""Diagnostic: trace V2 settle phase step-by-step."""
import math, sys
sys.path.insert(0, ".")
from ltv_sim import *

cfg = SimConfig(
    settle_enabled=True,
    settle_hold_s=0.05,
    settle_max_s=5.0,
    settle_xy_tol_in=0.75,
    settle_ang_tol_deg=1.2,
    settle_stable_s=0.15,
    settle_use_voltage_ctrl=True,
    settle_enable_approach=True,
    settle_approach_xy_trigger_in=1.0,
)

result = run_simulation(LTV_PATH_WAYPOINTS, LTV_PATH_START, config=cfg)

print(f"traj={result['total_time_s']:.2f}s end={result['sim_stopped_at_s']:.2f}s "
      f"xy={result['final_xy_err_in']:.3f} hdg={result['final_hdg_err_deg']:.2f}")

traj_t = result['total_time_s']
count = 0
for e in result['log']:
    if e['t'] < traj_t + 0.04:
        continue
    count += 1
    if count % 10 == 1:
        h2f = wrap_deg(0.0 - e['th'])
        print(f"  t={e['t']:.3f} ({e['x']:7.2f},{e['y']:6.2f},{e['th']:7.1f}) "
              f"xy={e['xy_err']:.3f} h2f={h2f:6.1f} "
              f"cmd=({e['cmd_left']:4d},{e['cmd_right']:4d}) "
              f"ff=({e['ff_l']:6.0f},{e['ff_r']:6.0f})")
