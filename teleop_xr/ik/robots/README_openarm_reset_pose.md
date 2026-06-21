# OpenArm reset / default pose (teleop_xr IK)

This documents the OpenArm bimanual **default (reset) configuration** returned by
`OpenArm.get_default_config()` in `openarm.py`, used by the teleop_xr IK demo:

```bash
python -m teleop_xr.demo.demo_with_ros2 --mode ik --robot-class openarm \
    --head-device /dev/video53 \
    --wrist-left-device /dev/video65 --wrist-right-device /dev/video59
```

This is the pose the arms snap to on reset / when IK seeds the solver
(`get_default_config`). It must match the pose the bimanual bringup drives the
real arms to, otherwise teleop and bringup disagree about "home".

## Change 2026-06-18 — left arm now mirrors the right

**Before:** the active `default_pose` had the **right** arm at the recording home
pose but the **left** arm at an ad-hoc placeholder
(`joint1=0.3, joint2=-0.1, joint4=0.1`, rest 0) that was **not** a mirror of the
right arm — so the bimanual reset was asymmetric.

**After:** the left arm is the mirror of the right recording home pose, matching
the bimanual launch (`openarm.bimanual.launch.py` with `execute_pose:=true`) in
the `~/OPEN_ARM/packages` workspace.

### Pose values

| joint | right (home) | MIRROR | left (= mirror) |
|-------|--------------|--------|-----------------|
| 1 | 0.192  | −1 | −0.192 |
| 2 | 0.750  | −1 | −0.750 |
| 3 | −0.688 | −1 | 0.688  |
| 4 | 0.964  | +1 | 0.964  |
| 5 | 0.000  | −1 | 0.000  |
| 6 | 0.683  | −1 | −0.683 |
| 7 | 0.930  | −1 | −0.930 |

Fingers (`*_finger_joint1/2`) stay at `0.0`.

`MIRROR = {1:-1, 2:-1, 3:-1, 4:+1, 5:-1, 6:-1, 7:-1}` — joints whose axis lies in
the sagittal mirror plane flip sign; the elbow (j4) keeps its sign. j7 also flips
(the URDF already reflects its axis); this was **verified in RViz** — with +1 the
two grippers rolled opposite ways. This is the same mirror map used by
`left_arm_sequenced_pose.py` in the `~/OPEN_ARM/packages` bringup workspace.

> Source of truth for the right-arm home pose: `FINAL` in
> `openarm_bringup/scripts/right_arm_sequenced_pose.py`. If you retune that, update
> both the right values **and** the left mirror here.

## Editing notes

- The previous `default_pose` block was **commented out, not deleted**, directly
  above the active one in `openarm.py` (look for "PREVIOUS default_pose").
- Two earlier experimental poses were already commented out above it and are left
  as-is.

## Backup

A timestamped backup of `openarm.py` was made before this edit:

```
openarm.py.bak.20260618-212010
```

(Earlier backups from prior sessions also exist in this folder:
`openarm.py.bak`, `openarm.py.bak.20260526-105708`,
`openarm.py.bak.20260615-182337`, `openarm.py.bak.20260615-183828`.)

### Rollback

```bash
cd ~/OPEN_ARM_NEW/src/teleop_xr/teleop_xr/ik/robots
cp openarm.py.bak.20260618-212010 openarm.py
```

## Notes / caveats

- `get_default_config` warns this is a **safe working configuration** (bent arms
  avoid the joint4 singularity); the recording home pose keeps j4 bent (0.964), so
  it stays well clear of the all-zero singularity.
- As with the bringup pose, the symmetric shape is correct in RViz, but on real
  hardware watch for arm-to-arm collision when both arms fold toward center.
