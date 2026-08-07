# Real-H1 Quest teleop with measured plate lock

## Context

Palletizing moves from Isaac Sim to the real H1 over ROS2. First hardware milestone: pick up and
hold a box by squeezing it between the two flat EE plates, under Quest teleop.

Two findings from exploration reshape the work:

- `~/IsaacLab-Arena/isaaclab_arena/scripts/h1_quest_teleop.py` **contains no Isaac/IsaacLab code**
  (`grep isaaclab` → comments only). It is a plain ROS2 node in system Python 3.10 publishing
  `/lowcmd` + `/base_cmd` and subscribing `/lowstate`. The real robot's `h1_ros2_node.py`
  (`backend:=xapi`) serves that exact interface. So `ArmIK`, `QuestInput`, `KeyboardInput`,
  `FullBodyFK` and the joint/limit tables are reusable **as-is** — import them, don't fork them
  (`h1_place_planner.py` already does `import h1_quest_teleop as hqt`).
- Sim-only coupling is confined to: `/box_lock` `/respawn_box` `/head_cam_*` publishers, the ZMQ
  `BasePoseListener` on :15559, sim-clock pacing (`_sim_dt_s`), and `solve_place_trajectory`.
  All four get dropped.

In sim the box lock was a kinematic weld to the right plate (`h1_box_lock.py`, no physics). On
hardware the lock must be **measured**: FK the two plate poses, compare their separation to the
box width, and only then switch to right-arm-only control with the left arm slaved.

## Decisions (confirmed with user)

| | |
|---|---|
| Deliverable | new script in `~/lerobot`, sim script untouched |
| Lock trigger | live gap readout always shown; **A button engages only if gap is in tolerance** |
| Follow mode | rigid 6-DoF relative transform frozen at lock |
| Geometry | empirical calibration mode → JSON; URDF value is only a seed |

Why calibration is not optional: the plate contact-face offset disagrees between sources —
`h1_with_ee.urdf:786` puts the plate slab faces at ee-link-frame `y = ±0.060 / ±0.080`, while
`ISAACLAB_ARENA_SETUP.md:260-275` calls the support face the extreme-y surface (±0.0803). That
±20 mm spread exceeds the grip clearance.

## Files

| Path | Action |
|---|---|
| `~/lerobot/tools/h1_real_teleop.py` | **new** — the node |
| `~/lerobot/config/h1_plate_calibration.json` | **new** — written by `--calibrate` |
| `~/lerobot/tests/tools/test_h1_real_teleop.py` | **new** — pure-numpy SE(3) tests |
| `~/IsaacLab-Arena/.../h1_quest_teleop.py` | read-only, imported |
| `~/topstar_ros2/.../topstar_h1/h1_ros2_node.py` | read-only, imported (mount transforms) |
| `~/topstar_ros2/.../topstar_h1/{joint_defs,h1_drive_example}.py` | read-only, imported |

## 1. Skeleton and reuse

`tools/h1_real_teleop.py`, system Python 3.10, run after `source ~/topstar_ros2/setup.sh`.

- Copy `_bootstrap_placo()` verbatim from `h1_quest_teleop.py:140-170` (must run before any
  `import placo`; it `execve`s with the cmeel `LD_LIBRARY_PATH`).
- `sys.path.insert(0, "~/IsaacLab-Arena/isaaclab_arena/scripts")`, then
  `import h1_quest_teleop as hqt`. Reuse: `hqt.ArmIK`, `hqt.FullBodyFK`, `hqt.QuestInput`,
  `hqt.KeyboardInput`, `hqt._js`, `hqt._clamp_arm`, `hqt._resolve_little_top_urdf`,
  `hqt._ARM_LIMITS`, `hqt._TORSO_HEAD_LIMITS_HW`, `hqt._HW_TO_URDF_SIGN`, `hqt._N_JOINTS`,
  `hqt._RIGHT_ARM_IDX`, `hqt._LEFT_ARM_IDX`, `hqt._{RIGHT,LEFT}_EE_LINK_URDF`,
  `hqt._H1_FULL_URDF_PATH`, the `_DPOS_CLIP`/`_DROT_CLIP`/rate/base-speed constants.
- `from topstar_h1.h1_ros2_node import _T_BODY_TO_RIGHT_ARM, _T_BODY_TO_LEFT_ARM`
  (`h1_ros2_node.py:94-107`) — single source of truth, no re-derivation.
  `from topstar_h1.h1_drive_example import make_lowcmd`.
- ROS2 node mirroring `h1_quest_teleop.run_node`, minus the sim publishers, minus
  `BasePoseListener`, minus `_step_place_playback`/`_trigger_place_trajectory`. `dt = 1/rate`
  from **wall clock** (`time.monotonic()` delta), not `_sim_dt_s`.
- Keep: `/lowcmd` pub + `/lowstate` sub (QoS BEST_EFFORT / KEEP_LAST / depth 1 — must match
  `h1_ros2_node.py:182-191`), `_sync_targets` on first state and on B+Y, joystick torso/head
  integration, `/base_cmd` on RG. Log once at startup that `/base_cmd` is a **no-op on the xapi
  backend** (`backends/xapi.py:68-69`) — the chassis has its own commercial controller.

## 2. `PlateGauge`

Wraps one `hqt.FullBodyFK` (`h1_with_ee.urdf` — the only model carrying plate geometry).

```
update(q_hw_18)  -> set_hw(); T_r = ee_T(_RIGHT_EE_LINK_URDF); T_l = ee_T(_LEFT_EE_LINK_URDF)
gap()            -> ||t_l - t_r|| - (offset_r + offset_l)      # offsets from calibration JSON
tilt_deg()       -> angle between the two plate normals (each EE link's local y axis), 180° = parallel
lateral_m()      -> component of (t_l - t_r) perpendicular to the right plate normal
```

Both `Robot_*_Hand_6_Link` poses come from the same model, so they are already in a common frame
— no cross-frame algebra needed here. FK tracks the Hand_6 link origin; the plate sits at
`+0.03 m` in z off it (`h1_with_ee.urdf:813-817, 849-853`), which the calibrated offset absorbs.

## 3. Calibration mode

`--calibrate --box-width 0.42 [--samples 50]`

- Node subscribes `/lowstate`, **publishes nothing**. Prints at 5 Hz:
  `origin_dist`, `gap@offset=0`, `tilt_deg`, `lateral_m`.
- Operator jogs with `h1_upper_body_jog.py` (or a separate teleop instance) until the plates are
  snug on a physically measured box, then presses `c`.
- Averages `--samples` ticks, writes `config/h1_plate_calibration.json`:
  `{box_width_m, origin_dist_m, plate_offset_m: (origin_dist - box_width)/2, tilt_deg,
    lateral_m, samples, q_hw, urdf, timestamp}`.
- Only the **sum** of the two offsets is observable; split symmetrically and say so in the file.

## 4. Lock state machine

State: `self._locked: bool`, `self._grip_G: np.ndarray | None`.

Readiness each tick, from the **measured** `/lowstate` q (not the command):

```
ready = |gap - box_width| <= --lock-tol            (default 0.015 m)
    and |180 - tilt_deg|  <= --lock-max-tilt-deg   (default 8.0)
    and lateral_m         <= --lock-max-lateral    (default 0.05 m)
```

- **A edge, not locked**: engage iff `ready`; otherwise log which predicate failed and stay
  unlocked. No silent mode change.
- **A edge, locked**: release.
- While locked, monitor gap drift; warn past `--lock-drift-warn` (default 0.02 m) but **never
  auto-release** — releasing mid-carry drops the box.
- `--force-lock` engages regardless of readiness, for the in-air validation step (§7).

## 5. Locked control — right drives, left follows

Work entirely in the two `little_top` arm-mount frames; no full-body FK in the control loop.
Both mounts are children of `Robot_Body_Rotation_Link`, so the mount-to-mount transform is
**independent of torso lift and pitch**:

```
K = inv(_T_BODY_TO_LEFT_ARM) @ _T_BODY_TO_RIGHT_ARM        # constant, computed once
```

With `A_r`, `A_l` the right/left EE poses in their own mount frames (exactly what
`ArmIK.target` / `ArmIK.model.get_T_world_frame("end_effector")` return):

```
at lock:   G = inv(A_r) @ inv(K) @ A_l                     # frozen grip transform
per tick:  A_l = K @ A_r @ G
```

(Check: `K A_r G = K A_r inv(A_r) inv(K) A_l = A_l`.)

Tick order while locked:

1. Left controller deltas **discarded**; right controller deltas applied full 6-DoF via
   `self.right.apply_delta(r_dpos, r_drot)` (unchanged clips `_DPOS_CLIP` / `_DROT_CLIP`).
2. `q_right = self.right.solve()`.
3. `A_r = self.right.model.get_T_world_frame("end_effector")` — the **IK-achieved** pose, not the
   target. Same rationale as `solve_place_trajectory` (`h1_quest_teleop.py:1318-1327`): the
   physical constraint is between the two real plates, so if the right arm saturates a joint
   limit the left must follow where it actually went, keeping the grip rigid.
4. `self.left.target = K @ A_r @ G`; `q_left = self.left.solve()`.
5. Log `self.{left,right}.fk_error_mm()` — a rising left error means the follow pose is drifting
   out of the left arm's reachable set.

Unlocked behaviour is unchanged from the sim script (both controllers independent).

This replaces the sim carry mode (`h1_quest_teleop.py:2336-2346`), which pushed one raw
translation through both axis maps and dropped all rotation — it kept boxes level but could not
reorient them.

## 6. Hardware safety (new — sim never needed these)

- **Pre-flight limit check, do this first.** `hqt._ARM_LIMITS[5]` was widened to ±2.356 rad on
  2026-07-22 on the strength of `h1_with_ee.urdf`, against `little_top.urdf`'s declared
  ±(1.536/0.436) (see the comment block at `h1_quest_teleop.py:220-243`). Reconcile against
  `topstar_h1.joint_defs.H1_JOINT_LIMITS_MJ` before any hardware run and expose
  `--arm-limits {little_top,full,joint_defs}`, defaulting to the **narrow** table on hardware.
  A clamp mismatch already snapped a real wrist at node startup once.
- **Rate limit**: clamp per-joint `|q_cmd[i] - q_cmd_prev[i]|` to `--max-step-rad` (default
  0.02 rad/tick ≈ 1 rad/s at 50 Hz) before `make_lowcmd`. `h1_upper_body.set_joints` schedules
  waypoints with a 5-cycle lookahead and has no rate guard of its own.
- **Startup**: publish nothing until the first `/lowstate` arrives and `_sync_targets()` has
  seeded both `ArmIK`s from the measured pose (already the sim behaviour — keep it).
- **`--no-publish`** dry-run flag: full pipeline, `/lowcmd` suppressed.
- **Keyboard `space`**: latch the last commanded pose and stop integrating deltas (soft hold).
- `dq` / `tau_est` are hardcoded to 0 in `h1_ros2_node.py:550-557`, so there is no effort-based
  grip confirmation available. FK gap is the only signal — state this in the docstring.

## 7. Verification

Offline, no robot:

1. `--test-gauge` — load `h1_with_ee.urdf`, feed `hqt._CARRY_Q_HW_BY_GRIP["end"]`, assert the
   computed plate separation ≈ **0.440 m** (the documented `end`-profile value,
   `h1_held_box_start.py:101`). Repeat for `"side"` → **0.416 m**. Strong check of the gauge
   against numbers derived independently in sim.
2. `tests/tools/test_h1_real_teleop.py` — pure numpy, no placo: random SE(3) `A_r`, `A_l`, `K`;
   assert `K @ A_r @ (inv(A_r) @ inv(K) @ A_l) == A_l`; assert invariance of the recovered
   relative pose when `A_r` is perturbed; assert the rate limiter never exceeds `max_step`.

On the robot (`ros2 launch topstar_ros2_example h1_sim.launch.py backend:=xapi`):

3. FK cross-check — call the existing `get_arm_ik`/`get_arm_fk` services
   (`h1_ros2_node.py:275-276`, `transform_base_link` output) at the current pose and compare
   against `PlateGauge`'s two link poses. Should agree to ~1 mm; a mismatch means the URDF or
   sign convention is wrong before anything moves.
4. `--no-publish --calibrate` — jog by hand with `h1_upper_body_jog.py`, confirm the printed gap
   tracks a tape-measured plate gap across several poses, then calibrate on the real box.
5. **In-air follow test** — `--force-lock` with no box, plates ~0.5 m apart. Move the right
   controller slowly in translation *and* rotation; verify the left arm mirrors and the measured
   gap stays constant to within a few mm. Validates the `K`/`G` algebra with nothing to crush.
6. Approach and hold — open plates, drive in with both controllers, watch the readout go
   `READY`, press A, lift. Confirm `LOCK` holds and gap drift stays inside the warn threshold.

## Explicitly out of scope

Scripted place trajectory, pallet plan / `--test-plan-place`, `/respawn_box`, head-cam zoom/target,
`--held_box_start`, and any LeRobot↔ROS2 policy adapter. The gap noted in exploration — that
`TopstarH1Robot` / `H1SeatEnv` / EXPO-FT all speak the sim's ZMQ JSON and have no hardware path —
is real but is a separate piece of work.

