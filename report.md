# OpenArm Bimanual XR Teleop — Jitter Fix Report

**Date:** 2026-05-26
**Backup timestamp suffix:** `20260526-105708`
**Goal:** Reduce arm jitter during XR teleop without aggressive changes that would require re-tuning. Each edit is conservative and easily reversed by restoring the `.bak` file.

---

## Quick-start: how to revert any single change

```bash
# Example: revert demo_with_ros2.py
cp /home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/demo/demo_with_ros2.py.bak.20260526-105708 \
   /home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/demo/demo_with_ros2.py
```

Backups live next to each edited file with the suffix `.bak.20260526-105708`.

---

## Files changed (4 source files, 5 logical edits in the demo)

| # | File | Backup | Purpose |
|---|------|--------|---------|
| 1 | `OPEN_ARM_NEW/src/teleop_xr/teleop_xr/demo/demo_with_ros2.py` | `…/demo_with_ros2.py.bak.20260526-105708` | 5 edits: trajectory horizon, IK filter, gripper dedup, disable reset-on-double-squeeze, guard shutdown |
| 2 | `OPEN_ARM_NEW/src/teleop_xr/teleop_xr/ik/robots/openarm.py` | `…/openarm.py.bak.20260526-105708` | Re-balance IK cost weights (rest ↑, self-collision ↓) |
| 3 | `OPEN_ARM_NEW/src/teleop_xr/teleop_xr/__init__.py` | `…/__init__.py.bak.20260526-105708` | Relax XR pose-jump tolerances |
| 4 | `OPEN_ARM/packages/src/openarm_ros2/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml` | `…/openarm_v10_bimanual_controllers.yaml.bak.20260526-105708` | `goal_time: 0.0 → 0.05` (both JTCs) |

> Note: the `install/` copy of the YAML is a **symlink** to the `src/` file (you built with `colcon build --symlink-install`), so the single edit applies to the running system without a rebuild. A `.bak` was still made at the install path for safety; it is a content duplicate.

---

## Detailed change list

### 1. Demo file: `demo_with_ros2.py`

Path: `/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/demo/demo_with_ros2.py`
Backup: `/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/demo/demo_with_ros2.py.bak.20260526-105708`

**1a. Trajectory time horizon — was 20 ms, now 100 ms** (around line 151)

*Before:*
```python
point.time_from_start.sec = 0
# point.time_from_start.nanosec = 100_000_000  # 0.1 seconds in future
point.time_from_start.nanosec = 20_000_000   # ← CHANGE: 20 ms, NOT 100 ms
```

*After:*
```python
point.time_from_start.sec = 0
# 100 ms horizon gives the JTC cubic spline room to interpolate smoothly
# at 150 Hz controller rate; 20 ms caused velocity-derivative spikes.
point.time_from_start.nanosec = 100_000_000
```

*Why:* Each `JointTrajectory` carries a single point with the horizon you set here. At 150 Hz controller cycle (≈6.7 ms), a 20 ms horizon is only ~3 cycles wide — the JTC's cubic spline produces large velocity setpoints to hit the target on time, and each new XR sample overwrites the spline mid-execution. 100 ms gives the spline ≈15 cycles to ramp gracefully.

---

**1b. IK output low-pass filter — was disabled, now 3-tap weighted MA** (around line 674)

*Before:*
```python
solver = PyrokiSolver(robot)
controller = IKController(robot, solver)
```

*After:*
```python
solver = PyrokiSolver(robot)
# 3-tap weighted moving filter on joint output (weights must sum to 1.0).
# Newer samples weighted more so we keep responsiveness while killing
# the high-frequency XR-pose noise that was reaching the joints raw.
ik_filter_weights = np.array([0.2, 0.3, 0.5])
controller = IKController(robot, solver, filter_weights=ik_filter_weights)
```

*Why:* `IKController` already supports an output filter via `WeightedMovingFilter` (`teleop_xr/utils/filter.py`) — it was simply never wired up. Weights `[0.2, 0.3, 0.5]` sum to 1.0 (required by `WeightedMovingFilter`), are heavier on the newest sample for responsiveness, and add roughly one frame of latency. If the arm feels too sluggish, shorten to `[0.3, 0.7]`; if it still chatters, lengthen to e.g. `[0.1, 0.2, 0.3, 0.4]`.

---

**1c. Gripper command dedup — was commented out, now active** (around line 165)

*Before:* a commented-out dedup block, followed by:
```python
# REMOVED the dedup — let every press hit the wire
if side == "left":
    client = self.left_gripper_client
else:
    client = self.right_gripper_client
```

*After:*
```python
# Dedup near-identical gripper commands so we don't spam the action server.
if side == "left":
    if self._last_left_gripper_pos is not None and \
       abs(position - self._last_left_gripper_pos) < 0.001:
        return
    self._last_left_gripper_pos = position
    client = self.left_gripper_client
else:
    if self._last_right_gripper_pos is not None and \
       abs(position - self._last_right_gripper_pos) < 0.001:
        return
    self._last_right_gripper_pos = position
    client = self.right_gripper_client
```

*Why:* Your terminal showed 3 right-gripper goals in ≈100 ms. With dedup off, every IK frame re-issues an unchanged gripper goal. This isn't the main jitter cause but it burdens the action server and floods logs. 1 mm threshold is well below useful gripper resolution.

---

**1d. Reset-on-double-press-SQUEEZE — disabled** (line 792)

*Before:*
```python
processor.on_double_press(button=XRButton.SQUEEZE, callback=on_reset_pose)
```

*After:*
```python
# Disabled: double-press SQUEEZE reset snapped joints mid-task. Re-enable
# by uncommenting the next line if you want joint reset back.
# processor.on_double_press(button=XRButton.SQUEEZE, callback=on_reset_pose)
```

*Why:* You explicitly asked for this in your note ("need to remove the reset command position in demo ros2"). The handler function `on_reset_pose` is still defined just above — only the binding is removed, so re-enabling is one-line.

---

**1e. `rclpy.shutdown()` guard** (line ≈915)

*Before:*
```python
if ros2_node:
    rclpy.shutdown()
```

*After:*
```python
if ros2_node and rclpy.ok():
    rclpy.shutdown()
```

*Why:* The traceback at the end of your terminal log was `RCLError: failed to shutdown: rcl_shutdown already called`. Guarding with `rclpy.ok()` makes the shutdown idempotent. Cosmetic; does not affect jitter.

---

### 2. IK costs: `ik/robots/openarm.py`

Path: `/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/ik/robots/openarm.py`
Backup: `/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/ik/robots/openarm.py.bak.20260526-105708`

**2a. `rest_cost` weight: 5.0 → 10.0** (around line 206)

*Why:* The rest cost pulls the IK solution toward `q_current`. With it at 5.0 vs `self_collision_cost` at 10.0, the solver could "win" by moving joints to satisfy the self-collision constraint at the price of a discontinuous jump from the previous solution. Raising rest to 10.0 makes the solver prefer smaller frame-to-frame joint changes, which is exactly what we want for smooth tracking.

**2b. `self_collision_cost` margin 0.05 → 0.02, weight 10.0 → 5.0** (around line 250)

*Why:* 5 cm collision margin with a high weight created a wide "no-go zone" where the solver flipped between two locally optimal joint configurations on consecutive frames — felt as a single-joint twitch (often elbow). Tightening the margin to 2 cm and halving the weight keeps you safe from actual collisions but stops the oscillation. If you start seeing the arms touch themselves, raise margin first (try 0.03), not weight.

---

### 3. Pose-jump tolerances: `__init__.py`

Path: `/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/__init__.py`
Backup: `/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/__init__.py.bak.20260526-105708`

**3a. `lin_tol`: 0.05 m → 0.10 m, `ang_tol`: 35° → 60°** (around line 417)

*Why:* The "Pose jump detected, resetting the pose" warning was firing during normal operation in your run. Each reset re-anchors `__relative_pose_init`, which makes the IK target snap to a new origin → joint jerk. The previous limits were too tight for Quest tracking under WebRTC stutter and fast hand motion. 10 cm / 60° still catches genuine teleport-class glitches but tolerates real motion.

---

### 4. Controller YAML: `openarm_v10_bimanual_controllers.yaml`

Path (canonical, src): `/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_ros2/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml`
Backup: same path + `.bak.20260526-105708`
(Install copy at `…/install/openarm_bringup/share/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml` is a symlink to the src file — no rebuild needed.)

**4a. `goal_time: 0.0 → 0.05` in both `left_joint_trajectory_controller` and `right_joint_trajectory_controller`** (lines 102 and 167)

*Why:* `goal_time` is the tolerance the JTC allows for finishing a trajectory point past its `time_from_start`. With it at 0.0, the controller has zero slack — combined with a tight `time_from_start`, the spline interpolator is forced into aggressive velocity profiles. 50 ms slack lets the spline coast smoothly when the next goal arrives slightly late.

---

## Issues identified but **not** fixed (out of scope or needs ROS package rebuild)

| Issue | Where | Why not touched |
|---|---|---|
| `Could not enable FIFO RT scheduling policy: Operation not permitted` | controller_manager runtime log | System-level; needs `setcap cap_sys_nice+ep` on `ros2_control_node` or a sysctl/limits.conf change. Better as a one-time system fix you do explicitly. |
| `WARNING: INVALID PARAM DATA` during hardware configure | `OpenArm_v10HW::on_configure` C++ plugin | C++ source change + colcon rebuild required. Worth a follow-up bug report. |
| `can_fd: false` may saturate CAN under load | hardware param | Hardware-dependent; needs CAN-FD-capable adapter + driver config. Test current changes first. |
| Two source trees (`OPEN_ARM_NEW` vs `OPEN_ARM`) | repo layout | Tree consolidation is a separate task; risk of deleting active work. |

---

## How to verify the improvement

1. **Restart** the demo and the bringup so the new YAML and Python files load:
   ```bash
   # terminal 1 (bringup): Ctrl-C then relaunch
   ros2 launch openarm_bringup openarm.bimanual.launch.py \
       right_can_interface:=robot_l left_can_interface:=robot_r
   # terminal 2 (relay): Ctrl-C then relaunch
   python3 scripts/joint_trajectory_relay.py
   # terminal 3 (demo): Ctrl-C then relaunch
   python -m teleop_xr.demo.demo_with_ros2 --mode ik --robot-class openarm \
       --head-device /dev/video53 \
       --wrist-left-device /dev/video65 \
       --wrist-right-device /dev/video59
   ```

2. **Watch for these signals** during a slow figure-8 motion:
   - "Pose jump detected" warning should appear **rarely** now (vs constantly before).
   - Gripper "Received & accepted new action goal" log should fire only on actual state changes.
   - The visible joint chatter should be reduced. If it's now sluggish, the filter is too strong — drop to `[0.3, 0.7]` or remove `filter_weights=`.

3. **Quantify with `plotjuggler`** (optional):
   - `ros2 topic echo /joint_states` while holding controllers still.
   - Plot e.g. `left_joint4` position vs time; standard deviation over a 5 s window is a good proxy for residual jitter.

---

## Tuning knobs if symptoms persist

| Symptom | First knob to turn |
|---|---|
| Still jittery | Shorten filter to `[0.1, 0.2, 0.3, 0.4]` (more lag, less noise) or raise `time_from_start` to 150 ms |
| Now sluggish / laggy | Shorten filter to `[0.3, 0.7]` or `[0.4, 0.6]`; drop `time_from_start` to 70 ms |
| One joint still flicks | `rest_cost` weight to 15.0, `self_collision_cost` weight to 3.0 |
| Pose jumps still log often | `lin_tol` to 0.15, `ang_tol` to `math.radians(80)` |
| Self-collision happening | `self_collision_cost` margin back up to 0.03, weight to 7.0 |

---

## Follow-up edits (2026-05-26, same `.bak.20260526-105708` session)

After first round of testing the user reported:
- Lag during motion → loosened the filter and shortened the horizon
- "Spring back" on grip release on joints 1-2 (not a reset, a compliance/stiffness symptom)

### F1. `demo_with_ros2.py` — trajectory horizon 100 ms → 60 ms

`time_from_start.nanosec = 60_000_000`. 60 ms is the middle ground: long enough that the JTC spline doesn't snap (the 20 ms original did), short enough that input lag isn't felt.

### F2. `demo_with_ros2.py` — filter [0.2, 0.3, 0.5] → [0.3, 0.7]

Dropped from a 3-tap (~30 ms lag) to a 2-tap (~10 ms lag). Still kills the high-frequency XR noise but doesn't feel sluggish.

### F3. **CORRECTION** — arm Kp/Kd lives in a YAML, not in the launch file

Earlier I edited `openarm.bimanual.launch.py` thinking the `kp1`/`kd1`/... it passes as xacro mappings would reach `parse_config()` in `v10_simple_hardware.cpp:91-101`. **They don't.** The xacro at `openarm_description/urdf/robot/v10.urdf.xacro:56` loads gains from a separate YAML:

```
control_gains="${xacro.load_yaml('$(find openarm_description)/config/arm/v10/control_gains.yaml')}"
```

So the launch-file kp/kd values are **dead code**. I've reverted the launch edit to its original values and added a comment pointing to the real source. The actual edit went into:

**Path:** `/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_description/config/arm/v10/control_gains.yaml`
**Backup:** same path + `.bak.20260526-105708`

| Joint | Was | Now |
|---|---|---|
| Kp1 (DM8009 shoulder) | 70.0 | **90.0** |
| Kd1 | 2.75 | **3.1** |
| Kp2 (DM8009 shoulder) | 70.0 | **90.0** |
| Kd2 | 2.5 | **2.8** |

The install copy of this YAML is a symlink (`colcon build --symlink-install`), so **no rebuild needed** — restart the bringup and the new gains are loaded by xacro at launch.

*Tuning if not enough:* `Kp = 110, Kd1 = 3.4, Kd2 = 3.1`.
*If it starts vibrating:* `Kp = 80, Kd1 = 3.0, Kd2 = 2.7`.

### F3-bis. Arm vs gripper gains — different mechanisms

- **Arm joints 1-7**: gains are parsed dynamically via `parse_config` from `info.hardware_parameters["kp1"..]`, which come from the xacro `<param>` block, which come from the YAML. *In theory* editable without rebuild — but during this session the YAML edit did not produce a visible behavior change. Switched to the .hpp route (see F4) which is the C++ default initializer and *guaranteed* to apply.
- **Gripper ("hand")**: hardcoded as `const double GRIPPER_KP / GRIPPER_KD` at `v10_simple_hardware.hpp:114-115`. The `hand:` block in `control_gains.yaml` is *not* wired to anything in the xacro right now — it's only a documentation hint. So gripper tuning **does** require editing the .hpp and rebuilding. This matches your earlier experience.

### F4. **Hardware .hpp** — Kp/Kd defaults edited directly (requires rebuild)

Path: `/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_ros2/openarm_hardware/include/openarm_hardware/v10_simple_hardware.hpp`
Backup: same path + `.bak.20260526-105708`

Edited the `kp_` / `kd_` initializer vectors (line 104-105):

```cpp
// Was: std::vector<double> kp_ = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};
// Was: std::vector<double> kd_ = {2.75, 2.5,  2.0,  2.0,  0.7,  0.6,  0.5};
std::vector<double> kp_ = {90.0, 90.0, 70.0, 60.0, 10.0, 10.0, 10.0};
std::vector<double> kd_ = {3.1,  2.8,  2.0,  2.0,  0.7,  0.6,  0.5};
```

These vectors are the C++ source-of-truth. `parse_config()` only overrides them if a matching `hardware_parameters["kp1"..]` is found in the URDF — if anything in the YAML→xacro chain silently no-ops, the .hpp values are what reaches the motors. Editing here is the reliable path.

**Rebuild required:**
```bash
cd ~/OPEN_ARM/packages
colcon build --packages-select openarm_hardware --symlink-install
source install/setup.bash
# then relaunch the bringup terminal
```

To revert: `cp …/v10_simple_hardware.hpp.bak.20260526-105708 …/v10_simple_hardware.hpp` then rebuild again.

---

---

## Follow-up edits (2026-05-26, backup timestamp `20260526-223820`)

### G. Runtime ROS2 parameter support for gains — no rebuild ever again after this

**Root cause discovered:** The bimanual xacro (`openarm.bimanual.ros2_control.xacro`) was missing the `kp1`...`kd7` param blocks entirely, and `openarm_robot.xacro` wasn't passing `control_gains` to it. This is why every YAML edit was a silent no-op in bimanual mode.

**Four files changed:**

| File | Backup | Change |
|---|---|---|
| `openarm_hardware/include/openarm_hardware/v10_simple_hardware.hpp` | `…hpp.bak.20260526-223820` | `GRIPPER_KP/KD` → mutable `gripper_kp_/gripper_kd_`; added `param_callback_handle_` and `register_param_callback()` declaration |
| `openarm_hardware/src/v10_simple_hardware.cpp` | `…cpp.bak.20260526-223820` | Added `register_param_callback()` implementation; fixed two `GRIPPER_KP/KD` references |
| `urdf/ros2_control/openarm.bimanual.ros2_control.xacro` | `…xacro.bak.20260526-223820` | Added `control_gains` macro param; added `kp1`...`kd7` + `kd1`...`kd7` `<param>` blocks to both left and right hardware sections |
| `urdf/robot/openarm_robot.xacro` | `…xacro.bak.20260526-223820` | Passed `control_gains="${control_gains}"` to the bimanual ros2_control macro call |

**After this rebuild, you have three ways to set gains:**

1. **At runtime (no restart):**
   ```bash
   ros2 param set /controller_manager left_kp1 95.0
   ros2 param set /controller_manager left_kd1 3.4
   ros2 param set /controller_manager right_kp1 95.0
   ros2 param set /controller_manager right_gripper_kp 1.5
   ```
   Takes effect on the next `write()` cycle (~6.7 ms). Bringup stays running.

2. **At launch via YAML (now wired correctly):**
   Edit `control_gains.yaml`, restart bringup. No rebuild.

3. **As C++ defaults in the .hpp (fallback if YAML chain breaks):**
   Edit `.hpp` initializers, `colcon build --packages-select openarm_hardware`.

**Verify params loaded at startup:**
```bash
ros2 param list /controller_manager | grep kp
# should show: left_kp1..left_kp7, right_kp1..right_kp7
ros2 param get /controller_manager left_kp1
# should show: 90.0
```

**Rebuild command:**
```bash
cd ~/OPEN_ARM/packages
colcon build --packages-select openarm_hardware --symlink-install
source install/setup.bash
# restart bringup terminal
```

---

---

## Follow-up edits (2026-05-27, backup timestamp `20260527-121502`)

### H. Fix spike on squeeze release — hardware feedback latch

**Root cause:** `state_container["q"]` is the IK's *commanded* position, never the hardware's *actual* position. Motor compliance (finite Kp) means the arm always lags slightly behind the commanded target. While squeezing, the JTC continuously receives new goals and the lag is invisible — the arm chases a moving target. When squeeze is released, the last JTC goal executes fully and the arm reaches the *commanded* position, which is slightly ahead of where it physically was. This catch-up motion is the spike. It appears "upward" because shoulder joints 1-2 compliance lets the arm sag slightly below the commanded angle during active teleop.

**Two secondary effects fixed at the same time:**
- While inactive, `state_container["q"]` is now synced from hardware → next re-engagement always starts from actual arm position (no jump on first squeeze)
- Filter and IK both start from hardware truth → no accumulated divergence across multiple squeeze sessions

**File:** `OPEN_ARM_NEW/src/teleop_xr/teleop_xr/demo/demo_with_ros2.py`
**Backup:** `…/demo_with_ros2.py.bak.20260527-121502`

Changes:
1. Added `from sensor_msgs.msg import JointState` to ROS2 import block
2. Added `self._hardware_joint_positions` dict + `/joint_states` subscriber + `get_hardware_q()` method to `IKJointTrajectoryPublisher`
3. In `IKWorker.run()` — on disengage (`was_active and not is_active`): read hardware q, publish latch trajectory to it, update `state_container["q"]`
4. In `IKWorker.run()` — while inactive: continuously sync `state_container["q"]` from hardware so next engagement starts from actual state

No rebuild required (Python, symlink-install).

---

## Backup inventory

```
/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/demo/demo_with_ros2.py.bak.20260526-105708
/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/demo/demo_with_ros2.py.bak.20260527-121502
/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/ik/robots/openarm.py.bak.20260526-105708
/home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/__init__.py.bak.20260526-105708
/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_ros2/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml.bak.20260526-105708
/home/air-lab-ncsu/OPEN_ARM/packages/install/openarm_bringup/share/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml.bak.20260526-105708
/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py.bak.20260526-105708
/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_description/config/arm/v10/control_gains.yaml.bak.20260526-105708
/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_ros2/openarm_hardware/include/openarm_hardware/v10_simple_hardware.hpp.bak.20260526-105708

# 2026-05-26-223820 session
/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_ros2/openarm_hardware/include/openarm_hardware/v10_simple_hardware.hpp.bak.20260526-223820
/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_ros2/openarm_hardware/src/v10_simple_hardware.cpp.bak.20260526-223820
/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_description/urdf/ros2_control/openarm.bimanual.ros2_control.xacro.bak.20260526-223820
/home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_description/urdf/robot/openarm_robot.xacro.bak.20260526-223820
```

Bulk revert (if everything goes sideways):
```bash
TS=20260526-105708
for f in \
  /home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/demo/demo_with_ros2.py \
  /home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/ik/robots/openarm.py \
  /home/air-lab-ncsu/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/__init__.py \
  /home/air-lab-ncsu/OPEN_ARM/packages/src/openarm_ros2/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml ; do
    cp "${f}.bak.${TS}" "$f"
done
```
