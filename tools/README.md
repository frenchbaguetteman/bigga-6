# tools/

Developer-only utilities for the `bigga-6` project. Nothing in here is
compiled into firmware; the scripts run on a host machine with Python 3.

> **Build isolation.** The PROS build only compiles sources under `src/`
> (see `SRCDIR` in `Makefile` and the `CSRC`/`CXXSRC` rules in
> `common.mk`), and the template export only ships specific headers from
> `include/`. Files under `tools/` are therefore ignored by `pros make`
> and `pros upload` and cannot affect the robot binary. Please keep this
> property: do not place any `.c`, `.cpp`, `.cc`, `.c++`, `.s`, or `.S`
> files in this directory.

---

## `ltv_sim.py` — LTV controller convergence regression test

`ltv_sim.py` simulates the closed-loop behaviour of the C++ LTV unicycle
controller (`include/controllers/ltv_controller.hpp`) against a nonlinear
unicycle plant. It is intended as a **regression test for the controller
math**: if anyone changes the discretization, DARE iteration, gain
schedule, or sign convention in a way that breaks closed-loop stability,
this script will fail loudly.

### What it checks

The script runs the closed loop until the tracking errors have settled to
a small steady state, defined as:

| Quantity                                       | Tolerance       |
| ---------------------------------------------- | --------------- |
| Position error: `hypot(x_ref - x, y_ref - y)`  | **< 1.5 inches** |
| Heading error:  `|theta_ref - theta|`          | **< 3 degrees**  |

Both conditions must hold simultaneously for **at least 20 consecutive
timesteps** (0.2 s at `dt = 10 ms`) before convergence is declared. The
hold window filters out transient zero-crossings — a single-step dip
under the threshold does not count. If the error later spikes back out
of the tolerance band, the success is retracted and the search resumes.

The script keeps simulating up to a hard timeout of 30 s, prints a
warning at 10 s if it has not yet converged, and only exits with status
`0` once the sustained tolerance has actually been met.

### Usage

```bash
# from the repo root
python3 tools/ltv_sim.py
```

Requires `numpy` (`pip install numpy`). No plotting libraries are
required — the script is purely numeric so it can run headless in CI.

The exit code is suitable for scripting:

* `0` — controller reached and held the steady-state tolerances. PASS.
* `1` — hard timeout (30 s) reached without sustained convergence. FAIL.

### Typical output

For the default scenario (start 6 in off in x with 15° heading error,
straight-line reference along +Y at 24 in/s), you should see something
like:

```
====================================================================
LTV closed-loop convergence simulation
====================================================================
  dt                  = 10.0 ms
  v_ref               = 24.00 in/s
  initial pose (in,in,deg) = (6.000, 0.000, 15.000)
  position tolerance  = 1.50 in
  heading  tolerance  = 3.00 deg
  must hold for       = 20 steps (0.20 s)
  soft timeout        = 10.0 s
  hard timeout        = 30.0 s
--------------------------------------------------------------------
--------------------------------------------------------------------
  [ok] steady state reached
       iteration       = 916
       time            = 9.160 s
       final pos error = 1.4446 in
       final head err  = 0.6661 deg
       held for        = 20 steps (0.20 s)
====================================================================
LTV CONVERGENCE: PASS
```

Numbers will vary slightly with NumPy / BLAS versions, but the final
status line should always read `LTV CONVERGENCE: PASS`. The position
error ends just under 1.5 in because the reference is moving — the
controller's tracking lag against a constant-velocity reference is what
the steady-state band is sized to absorb. The heading error converges
much faster (it has no moving-target component) and ends well under 1°.

If the soft timeout fires you will see an extra line like

```
  [warn] not yet converged after 10.0 s (pos_err = 1.812 in, ...) -- continuing.
```

before either a later `[ok]` or a final `[fail]`.

### Tuning knobs

All knobs live near the top of `tools/ltv_sim.py`:

* `dt`, `v_ref_in_s`, `omega_ref` — plant and reference parameters.
* `Q`, `R` — LQR cost matrices (must match the C++ defaults).
* `POS_TOL_IN`, `HEAD_TOL_DEG` — steady-state acceptance band.
* `STEADY_STATE_HOLD_STEPS` — number of consecutive in-tolerance steps
  required (filters out transients).
* `SOFT_TIMEOUT_S`, `HARD_TIMEOUT_S` — when to warn vs. when to give up.

Changing the initial pose (search for the `pose = np.array(...)` line in
`main()`) lets you stress-test from different starting offsets.
