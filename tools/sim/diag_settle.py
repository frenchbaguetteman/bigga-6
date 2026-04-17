"""Diagnostic: trace the settle phase step-by-step to compare with real robot log."""
import math, sys
sys.path.insert(0, ".")
from ltv_sim import *

# Run the FIXED config and log every 10th settling tick (matching real robot's 100ms print)
cfg = SimConfig(
    settle_enabled=True,
    settle_hold_s=0.05,
    settle_max_s=3.5,
    settle_xy_tol_in=0.75,
    settle_ang_tol_deg=1.2,
    settle_stable_s=0.15,
    settle_use_voltage_ctrl=True,
    settle_enable_approach=True,
    settle_approach_xy_trigger_in=1.0,
)

result = run_simulation(
    LTV_PATH_WAYPOINTS, LTV_PATH_START,
    config=cfg,
    extra_settle_time_s=0.5,
)

print(f"traj_time={result['total_time_s']:.2f}s  "
      f"sim_end={result['sim_stopped_at_s']:.2f}s  "
      f"final_xy={result['final_xy_err_in']:.3f}  "
      f"final_hdg={result['final_hdg_err_deg']:.2f}")

# Print every 10th step during the settle phase
traj_t = result['total_time_s']
count = 0
for entry in result['log']:
    if entry['t'] < traj_t + 0.04:
        continue
    count += 1
    if count % 10 == 1:
        print(f"  t={entry['t']:.3f} pos=({entry['x']:6.2f}, {entry['y']:6.2f}, {entry['th']:6.1f}) "
              f"xy={entry['xy_err']:.3f} hdg={entry['hdg_err_deg']:.2f} "
              f"cmd=({entry['cmd_left']:4d}, {entry['cmd_right']:4d}) "
              f"ff=({entry['ff_l']:7.0f}, {entry['ff_r']:7.0f})")
