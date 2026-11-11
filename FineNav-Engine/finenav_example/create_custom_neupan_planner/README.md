# NeuPAN plugin safety and fault handling

All adaptation is implemented in this directory; FineNav core and libneupan are unchanged.

- Observation validity is explicit. Missing messages, stale receipt/message timestamps,
  absent frames, malformed clouds and failed acquisition-time TF lookups stop motion.
  A fresh, valid observation containing no obstacle returns can still track a path.
- Callbacks cache immutable messages only. Configuration, filtering and TF conversion
  run on the planning worker; a mutex protects message handoff and a source generation
  rejects callbacks from a replaced subscription.
- Range filtering uses distance to the robot in the world frame. The minimum range,
  scan stride and max_points affect optimizer input only. Full finite points in the
  configured height band and maximum range remain available for collision checking.
  Robot self-returns must be removed upstream (as the simulator converter does).
- An independent rectangular footprint check covers the next command interval and
  braking motion against the full collision cloud, including the measured velocity.
  Optimizer decimation cannot bypass this gate. Terminal rotation also uses the full cloud.
- Every unsolved cycle commands zero immediately. solver_fail_grace now determines
  when to clear the warm start, and never permits executing an unsolved nominal.
- failure_timeout (default 10 seconds) escalates continuous observation/model/solver/
  collision failures to FineNav FAULTED. Terminal yaw progress is also bounded.
  Recovery-budget exhaustion sends zero once, then faults on the next cycle. Existing
  FineNav behavior-tree recovery handles the fault; a new episode resets the watchdogs.
- The packaged default model is resolved relative to the loaded shared library:
  ../share/finenav_example/models/diff_0p7x0p6.bin from the library directory.
  dune_checkpoint still overrides it. No build-machine model path is embedded.

The footprint gate assumes static observed obstacles and the configured acceleration
bounds. It does not provide global routing or dynamic-obstacle prediction. Actual control
period and physical braking limits must match the configuration.

## Verification

Build the finenav_example package after these source changes. Its plugin tests cover
observation loss, full-cloud collision gating, braking, terminal yaw stalls, exhausted
recovery, actual OSQP failure with a nonzero fallback, timestamped TF, world-frame range
filtering, malformed clouds, source replacement and concurrent message/configuration work.

For a standalone build against installed workspace dependencies:

```bash
source /home/affirm/neupan_reproduction_ws/install/setup.bash
cmake -S finenav_example -B /tmp/neupan-build -DBUILD_TESTING=ON
cmake --build /tmp/neupan-build --target test_goal_alignment test_goal_progress test_planner_safety test_neupan_plugin test_neupan_world_view -j2
ctest --test-dir /tmp/neupan-build/create_custom_neupan_planner --output-on-failure
```

Run these commands from FineNav-Engine. Tests set their library search paths to the
new build so an existing workspace installation cannot load an older plugin ABI.

## Early obstacle bypass

Single-goal mode now checks the forward goal corridor before solving. When it is
blocked within bypass_lookahead (default 2.5 m), the plugin evaluates left/right
references with two offset waypoints. Every segment is checked against the full
collision cloud using the robot circumradius plus bypass_margin (default 0.10 m).
It commits to the shortest feasible route, with a small initial-heading cost,
and keeps that route until it has passed the obstacle and can safely rejoin.
Path replacement preserves the optimizer warm start. Multi-pose upstream paths
are never overridden; the existing footprint gate and failure handling still apply.

This removes the need to wait for a 3–6 second stuck timeout before choosing a
side around a small static obstacle. If no safe candidate fits, it leaves the
reference unchanged and relies on the existing safe stop/recovery path. It does
not infer a global route through unobserved space or force the robot forward.

The startup parameters proactive_bypass, bypass_lookahead and bypass_margin are
read-only during an episode. The existing speed and collision thresholds are
unchanged. test_obstacle_bypass checks geometry, side commitment, safe rejoin and
rejection of blocked candidates; the plugin regression simulates passing a
cylinder with immediate steering and no reversing commands.
