# IsaacLab Arena + LeRobot 0.4.1 Setup Guide

This document describes how to run LeRobot 0.4.1 policy evaluation against IsaacLab Arena environments (e.g., GR1 microwave task) and the fixes required to make it work.

---

## Environment

| Component | Version / Path |
|-----------|---------------|
| Conda env | `env_isaaclab` at `~/miniforge3/envs/env_isaaclab` |
| Python | 3.11 |
| LeRobot | `/home/top/lerobot` branch `branch-v0.4.1` |
| IsaacLab-Arena | `/home/top/IsaacLab-Arena` — `isaaclab_arena 1.0.0` |
| IsaacLab | 2.3.2 |
| IsaacSim | 5.1.0 |
| numpy | 1.26.0 (hard pin — IsaacSim requires this exact version) |
| pink | 2.1.0 (IK solver) |
| daqp | (QP solver used by pink) |

---

## Eval Commands

### SmolVLA (`nvidia/smolvla-arena-gr1-microwave`)

```bash
conda activate env_isaaclab
cd /home/top/lerobot

lerobot-eval \
    --policy.path=nvidia/smolvla-arena-gr1-microwave \
    --env.type=isaaclab_arena \
    --env.hub_path=nvidia/isaaclab-arena-envs \
    --rename_map='{"observation.images.robot_pov_cam_rgb": "observation.images.robot_pov_cam"}' \
    --policy.device=cuda \
    --env.environment=gr1_microwave \
    --env.embodiment=gr1_pink \
    --env.object=mustard_bottle \
    --env.headless=false \
    --env.enable_cameras=true \
    --env.video=true \
    --env.video_length=10 \
    --env.video_interval=15 \
    --env.state_keys=robot_joint_pos \
    --env.camera_keys=robot_pov_cam_rgb \
    --trust_remote_code=True \
    --eval.batch_size=1
```

### Pi05 (`nvidia/pi05-arena-gr1-microwave`)

Same as above but with `torch.compile` disabled (required for Pi05) and different `--policy.path`:

```bash
TORCH_COMPILE_DISABLE=1 TORCHINDUCTOR_DISABLE=1 lerobot-eval \
    --policy.path=nvidia/pi05-arena-gr1-microwave \
    --env.type=isaaclab_arena \
    --env.hub_path=nvidia/isaaclab-arena-envs \
    --rename_map='{"observation.images.robot_pov_cam_rgb": "observation.images.robot_pov_cam"}' \
    --policy.device=cuda \
    --env.environment=gr1_microwave \
    --env.embodiment=gr1_pink \
    --env.object=mustard_bottle \
    --env.headless=false \
    --env.enable_cameras=true \
    --env.video=true \
    --env.video_length=10 \
    --env.video_interval=15 \
    --env.state_keys=robot_joint_pos \
    --env.camera_keys=robot_pov_cam_rgb \
    --trust_remote_code=True \
    --eval.batch_size=1
```

---

## Verified Results

| Policy | Task | Embodiment | Success Rate | Episodes |
|--------|------|------------|-------------|----------|
| SmolVLA | gr1_microwave | gr1_pink | **100%** | 50 |

---

## LeRobot Changes

The following files in `/home/top/lerobot/src/lerobot/` were modified or added to support IsaacLab Arena:

### New: `IsaacLabArenaObsWrapper` (`envs/utils.py`)

IsaacLab Arena returns observations in a nested dict:
```python
{"policy": {term: tensor}, "camera_obs": {cam_key: tensor}}
```

The wrapper converts this to LeRobot's flat format:
- `state_keys` terms (e.g., `robot_joint_pos`) → concatenated into `observation.state`
- Other `obs["policy"]` terms → `observation.<term>` (passed through)
- Camera images → converted NHWC→NCHW, uint8→float [0,1], keyed as `observation.images.<key>`

### `envs/configs.py`
Added `IsaacLabArenaEnv` dataclass registered as `"isaaclab_arena"`. Fields include:
- `hub_path`: HuggingFace Hub path for the Arena env package
- `environment`: e.g., `"gr1_microwave"`
- `embodiment`: `"gr1_pink"` (PINK IK EEF control) or `"gr1_joint"` (direct joint control)
- `state_keys`: observation terms to concatenate into `observation.state`
- `camera_keys`: camera observation keys

### `envs/factory.py`
Wraps the Hub-loaded Arena env with `IsaacLabArenaObsWrapper`.

### `configs/eval.py` and `scripts/lerobot_eval.py`
Added `trust_remote_code: bool = False` parameter, passed through to `make_env` and HuggingFace Hub model loading.

### `policies/smolvla/configuration_smolvla.py` and `policies/pi05/configuration_pi05.py`
Added:
```python
from typing import Any
rtc_config: Any | None = None
```
Fixes `DecodingError: The fields 'rtc_config' are not valid for SmolVLAConfig/Pi05Config` when loading model config from Hub.

### `pyproject.toml`
Updated `huggingface-hub` version constraint: `>=0.34.2,<2.0.0`.

---

## Critical Bug Fixes in IsaacLab (site-packages)

These fixes are applied to files under `~/miniforge3/envs/env_isaaclab/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/controllers/pink_ik/`.

**Symptom before fix:** Robot arms completely still every episode — only finger joints moved. 0% task success.

**Root cause:** The PINK IK integration was written against an older version of the `pink` library. Two API mismatches caused every IK computation to silently fail (caught by a broad `except (AssertionError, Exception)` with `show_ik_warnings=False`), returning the current joint positions unchanged every step.

---

### Fix 1 — `pink_kinematics_configuration.py` line 178

**File:** `.../controllers/pink_ik/pink_kinematics_configuration.py`

**Problem:** `PinkKinematicsConfiguration.check_limits` called the parent class with `safety_break` as a positional argument:
```python
def check_limits(self, tol: float = 1e-6, safety_break: bool = True) -> None:
    if safety_break:
        super().check_limits(tol, safety_break)  # BUG: passes bool as 2nd positional arg
```
Pink 2.1.0's `Configuration.check_limits(self, tol)` only accepts one argument (`tol`). Passing `safety_break` as a third positional argument raises:
```
TypeError: Configuration.check_limits() takes from 1 to 2 positional arguments but 3 were given
```

**Fix:**
```python
def check_limits(self, tol: float = 1e-6, safety_break: bool = True) -> None:
    if safety_break:
        super().check_limits(tol)  # FIXED: tol only
```

---

### Fix 2 — `pink_ik.py` compute method

**File:** `.../controllers/pink_ik/pink_ik.py`

**Problem:** `solve_ik` was called with an extra `safety_break` keyword argument:
```python
velocity = solve_ik(
    self.pink_configuration,
    self.cfg.variable_input_tasks + self.cfg.fixed_input_tasks,
    dt,
    solver="daqp",
    safety_break=self.cfg.fail_on_joint_limit_violation,  # BUG
)
```
Pink 2.1.0's `solve_ik(configuration, tasks, dt, solver, damping, **kwargs)` forwards `**kwargs` directly to the QP solver (`qpsolvers.solve_problem`). The `daqp` solver does not accept `safety_break` and raises:
```
TypeError: solve() got an unexpected keyword argument 'safety_break'
```

**Fix:** Remove the `safety_break` kwarg entirely:
```python
velocity = solve_ik(
    self.pink_configuration,
    self.cfg.variable_input_tasks + self.cfg.fixed_input_tasks,
    dt,
    solver="daqp",
)
```

---

## Key Architecture Notes

### `gr1_pink` embodiment action space (36-dim)

```
dims  0-2:   left EEF position (world frame, meters)
dims  3-6:   left EEF quaternion (world frame, wxyz)
dims  7-9:   right EEF position (world frame, meters)
dims 10-13:  right EEF quaternion (world frame, wxyz)
dims 14-35:  hand joint positions (22 joints, applied directly — bypass IK)
```

The `PinkInverseKinematicsAction` transforms EEF targets from world frame to base_link frame before passing to the PINK IK solver, which computes arm joint positions via differential IK.

### Robot pose in Arena vs. pick-place env

In IsaacLab Arena's `gr1_microwave` environment, the robot base_link/pelvis is placed at world `z=0` (set by `set_initial_pose(position_xyz=(-0.4, 0.0, 0.0))`). This differs from the pick-place environment where the pelvis is at `z≈0.93m`. Policy EEF targets are in world frame and have been trained for the Arena z=0 setup.

### Why IK errors were invisible

`GR1T2ActionsCfg` sets `show_ik_warnings=False` and `fail_on_joint_limit_violation=False`. The `PinkIKController.compute()` method catches all exceptions silently when `show_ik_warnings=False`:
```python
except (AssertionError, Exception) as e:
    if self.cfg.show_ik_warnings:
        print(...)
    return torch.tensor(curr_controlled_joint_pos, ...)  # returns current positions unchanged
```
This made both bugs completely invisible — the robot appeared to receive actions but never moved.

### SmolVLA observation mapping

SmolVLA's `prepare_state()` uses only `observation.state` (54-dim `robot_joint_pos`), ignoring other normalized state features even if they are present in the preprocessor config. The `--env.state_keys=robot_joint_pos` flag maps the 54-dim joint position vector to `observation.state`.

---

## H1 Palletize Scene (IsaacLab Arena)

### Key paths

| Resource | Path |
|----------|------|
| Scene bundle | `~/h1_box_scene_bundle/` |
| Scene USD | `~/h1_box_scene_bundle/scenes/collected/h1_box_scene_edit.usd` |
| H1 URDF | `~/h1_box_scene_bundle/robot/h1/h1_with_ee.urdf` |
| Isaac bridge reference | `~/h1_box_scene_bundle/scripts/isaac_bridge/h1_box_scene_isaac_sim.py` |
| MuJoCo model prep | `~/topstar_mujoco/simulate_python/prepare_h1_model.py` |
| Jog GUI | `~/topstar_ros2/example/src/src/h1/topstar_h1/h1_upper_body_jog.py` |
| ROS2 setup script | `~/topstar_ros2/setup_local.sh` |

### Files added/modified in IsaacLab-Arena (`/home/top/IsaacLab-Arena`)

| File | Purpose |
|------|---------|
| `isaaclab_arena/embodiments/topstar_h1/__init__.py` | Empty init |
| `isaaclab_arena/embodiments/topstar_h1/topstar_h1.py` | `TopstarH1Embodiment` — URDF ArticulationCfg, 18-joint action space, PD gains |
| `isaaclab_arena/assets/background_library.py` (+`WarehouseBoxBackground`) | Loads `h1_box_scene_edit.usd` as static background |
| `isaaclab_arena/tasks/palletize_task.py` | `PalletizeTask` — timeout termination, optional box metric |
| `isaaclab_arena/examples/example_environments/h1_palletize_environment.py` | `H1PalletizeEnvironment` — `h1_palletize` CLI name |
| `isaaclab_arena/examples/example_environments/cli.py` | Registers `h1_palletize` |
| `isaaclab_arena/embodiments/__init__.py` | Wraps gr1t2/g1 imports in try/except for pinocchio |
| `isaaclab_arena/utils/usd_pose_helpers.py` | Added no-defaultPrim fallback to world frame |
| `isaaclab_arena/scripts/teleop.py` | Fixed: `getattr(args_cli,"task","")`, `teleop_devices is not None`, action shape mismatch fallback |
| `isaaclab_arena/scripts/teleop_h1_zmq.py` | **H1 teleop — Isaac side** (env_isaaclab Python 3.11) |
| `isaaclab_arena/scripts/h1_arena_ros2_bridge.py` | **H1 teleop — ROS2 bridge** (system Python 3.10) |
| `fix_usd_default_prim.py` (repo root, run once) | Sets `defaultPrim=/World` on scene USD — already run, do not re-run |

### End-effector meshes (L-lip removed, 2026-07-07)

The original EE STLs (`left/right_hand_rebased.stl`) end in an L-shaped angle
bracket (a ~2 mm leg bolted on the plate top face at x 0.137→0.162, vertical lip
at x≈0.160 protruding ~7 mm past the box-support face). The lip made box grasping
require perfect alignment with the box bottom edge, so it was removed:

- The bracket is a separate connected component in each STL (896 faces; the only
  component protruding past the support plane |y| > 0.082 with x > 0.13).
- `left/right_hand_rebased_flat.stl` = original minus that component. Originals kept.
- `h1_with_ee.urdf` visual + collision now reference the `_flat` meshes.
- The plate support face is the extreme-y surface (left: y=−0.0803, right: +0.0805
  in EE-link frame); plate runs to x=0.16. The last ~2.5 cm (x 0.137→0.16) is
  recessed ~5 mm (was under the bracket) — boxes rest on the main face.
- IsaacLab regenerates the cached robot USD automatically (URDF hash changed).

### Cardboard box physics (2026-07-07)

`/World/Boxes/cardbox_0` in `h1_box_scene_edit.usd` originally had only
`PhysicsCollisionAPI` (boundingCube) — a static collider. `PhysicsRigidBodyAPI` +
`PhysicsMassAPI` (mass=1.0 kg placeholder) were applied so it can be held/placed.
Backup of the pre-edit USD: `h1_box_scene_edit.usd.bak-no-rigidbody`.

### Scene layout (world frame, measured from USD)

| Item | Position | Notes |
|------|----------|-------|
| Floor | z = −1.18 | everywhere |
| Robot spawn | (0.0, 0.4, −1.145), yaw −90° | hw-forward faces +y toward the conveyor |
| Cardbox | (−0.03, +2.18), bottom z=−0.06 | on conveyor, ~1.12 m above floor |
| Pallet | x −1.96…−1.16, y −0.70…+0.51 | on a dolly to the robot's left; top z=−0.67 (0.51 m above floor). Moved 2026-07-15 another 0.5 m away from the conveyor (dolly center (−1.55, 0.40) → (−1.55, −0.10)); backup `h1_box_scene_edit.usd.bak-pallet-y040`. (Earlier move 2026-07-10 from the aisle spot y≈−5.4, backup `.bak-pallet-behind`.) |
| Box size | 0.42 × 0.35 × 0.35 m | true size of SM_CardBoxA_01 (2026-07-18: earlier 0.42 × 0.40 × 0.40 / 0.42 × 0.395 × 0.395 figures were the AABB of the box while its spawn was tilted ~7.5°; it now spawns level) |

### Robot config details

- **URDF**: `~/h1_box_scene_bundle/robot/h1/h1_with_ee.urdf`, `fix_base=False` (mobile base since 2026-07-09; `--fix_base` CLI flag pins it), `merge_fixed_joints=True`
- **Spawn pose**: `pos=(0.0, 0.4, -1.145)`, `rot=(0.70711, 0, 0, -0.70711)` (yaw=−90°)
  - Source: `H1_SPAWN_X/Y/Z` defaults in `launch_h1_with_ee_box_scene_bridge_local.sh`
- **Action space**: 18 upper-body joints via `JointPositionActionCfg` with **`preserve_order=True`**
  - `preserve_order=True` is **critical** — the default `False` sorts joints by articulation index,
    scrambling the arm joint order vs. H1JointIndex
- **Observations** (shape): `actions (18,)`, `robot_joint_pos (26,)`, `robot_root_pos (3,)`, `robot_root_rot (4,)`

### HW ↔ URDF sign convention (18 slots)

Source: `h1_box_scene_isaac_sim.py` for slots 0/1/3, `prepare_h1_model.py` axis-flip notes for slots 14/16.

```
slot  0  torso_lift        -1  (hw axis anti-parallel to URDF)
slot  1  torso_pitch       -1
slot  2  head_yaw          +1  (no flip in Isaac Sim; MuJoCo model differs)
slot  3  head_pitch        -1
slots 4-13  right arm + left shoulder/elbow_yaw   +1
slot 14  l_elbow           -1  URDF axis="0 0 -1"; hw expects "0 0 1"
slot 15  l_wrist_yaw       +1
slot 16  l_wrist_pitch     -1  URDF axis="0 0 -1"; hw expects "0 0 1"
slot 17  l_wrist_roll      +1
```

Slots 14 and 16 are fixed in the MuJoCo model by `prepare_h1_model.py` (rewrites URDF axis).
For Isaac Sim the raw URDF is loaded, so the sign flip must be applied at runtime in `teleop_h1_zmq.py`.

### Mobile base (swerve drive, 2026-07-09)

The four wheel modules (steer + drive joint each) are driven by
`isaaclab_arena/embodiments/topstar_h1/h1_base_controller.py`
(`H1SwerveBaseController`) — an Isaac port of the MuJoCo
`topstar_ros2 .../topstar_h1/h1_base.py`. Steer joints take position targets and
drive joints velocity targets (implicit PD actuators), so the MuJoCo torque-level
machinery (overspeed latches, torque slew, wheel fault isolation) is not needed.
Ported features: chassis command clipping, accel slew, 0.25 s command watchdog,
swerve IK with shortest-path steer flip, idle hold — plus, since 2026-07-10, the
MuJoCo *transition shaping* (the first port omitted it and turn-then-forward could
tip the robot over — reproduced as 0.32 rad tilt spikes with 26 rad/s wheel bursts):
- steer position target rate-limited to `TOPSTAR_H1_STEER_TARGET_RATE` (1.2 rad/s;
  MuJoCo's 0.15 bounds torque coupling, which implicit PD doesn't have),
- drive velocity target scaled by cos²(steer error) × 0.25/(0.25+|steer rate|)
  (`TOPSTAR_H1_DRIVE_ENABLE_STEER_RATE`) — a misaligned/swinging module gets ~zero
  target and is braked by the 40 N·m·s/rad actuator damping,
- attitude guard: base tilt > `TOPSTAR_H1_MAX_SAFE_TILT` (0.35 rad) zeroes drive
  and holds steer.
Feel trade-off: turns/crab engage ~0.8 s late while modules swing to their new
heading (same as MuJoCo, which takes ~6 s). Verified by
`h1_base_turn_forward_test.py`: worst tilt 0.32 → 0.0000 rad on the tip maneuver;
`h1_base_drive_smoke_test.py` numbers unchanged except the delayed turn onset.

Command sources (all equivalent, hardware-forward convention):

```bash
# ROS2 (with bridge + teleop running):
ros2 run topstar_ros2_example h1_send_velocity --vx 0.2 --hold     # → /base_cmd Twist
# JSON command file (no ROS2 needed; same tool as MuJoCo):
python3 ~/topstar_mujoco/simulate_python/send_h1_velocity.py --vx 0.2 --duration 3
```

Kinematic conventions (derived from `h1_with_ee.urdf`, verified by
`h1_base_drive_smoke_test.py`):
- Steer joints all have axis (0,0,−1) → steer target = −atan2(v_y, v_x) in base frame
  (verified by the steer probe: module heading = −q on all four).
- Drive sign +1 on right wheels (1,3), −1 on left (2,4) — same as the MuJoCo table.
  Verified with `h1_wheel_axis_probe.py` (spins each wheel alone and reads the wheel
  body's world angular-velocity axis, which is slip-independent).
- Wheel collision simplified in the URDF (backup `h1_with_ee.urdf.bak-wheel-mesh-collision`):
  drive links use an analytic sphere r=0.0625 (the STL is a 43-component dual-wheel
  caster whose convex decomposition rolls badly), steer forks use a clearance box that
  stops 40 mm above ground. Visual meshes unchanged.
- EE plate collision simplified in the URDF (2026-07-14, backup
  `h1_with_ee.urdf.bak-ee-mesh-collision`): the `right/left_ee_link` collision was the
  full 8.7 MB hand STL under convex decomposition, whose hulls swallow the thin
  pressing plate — plates visibly sank into the cardbox sides. Replaced with exact
  primitives measured from the STL: a 20 mm plate slab (0.200×0.020×0.162 m, faces
  exactly at link-frame y=±0.060/±0.080) + a coarse bracket box that stays 2 mm behind
  the plate's inner face so only the plate can touch the box. Verified in the
  converted USD (exact Cube colliders, mesh collision gone) + gravity-hold test PASS.
  Visual meshes unchanged. The cardbox side is fine: `boundingCube` approximation
  matching its visual bounds (0.42×0.35×0.35 m).
- **Scene gotcha**: the conveyor's collision (walls/rollers/frame) blocks the way
  ~0.27 m ahead of the spawn in world +y. hw-forward (vx>0) drives INTO it from spawn;
  open floor is hw-backward (vx<0, world −y) and to the robot's left (world −x, where
  the pallet now sits). A "mysteriously stuck at 1/3 speed" base usually means it is
  wedged against scene collision — wheels then show wild slip-stick rates.
- Verified performance (smoke test): 0.300 m/s straight-line (yaw drift <0.001 rad
  over 1 m), rotate-in-place ω=0.5 rad/s with 2.7 mm center drift, lateral crab
  0.08 m/s with steer at ±90°, 4.3 cm stopping coast via 0.25 s watchdog + idle hold.
- `command_vx/vy_sign = −1` (hardware forward ↔ URDF base frame), overridable via
  `TOPSTAR_H1_CMD_VX_SIGN` / `TOPSTAR_H1_CMD_VY_SIGN`; speed/accel limits use the
  same `TOPSTAR_H1_*` env vars as MuJoCo.
- Wheel radius 0.0625 m; wheel modules at (±0.22, ±0.165) m.

`teleop_h1_zmq.py` also now: handles ZMQ `{"type":"basecmd"}` messages, publishes
`base_pos`/`base_quat` in the state payload, and stretches `episode_length_s` to
24 h by default (fixes the silent 30 s auto-reset that zeroed joints during teleop;
pass an explicit `--episode_length_s` to restore a timeout).

Headless verification:
```bash
python -u isaaclab_arena/scripts/h1_base_drive_smoke_test.py --headless \
    --disable_pinocchio h1_palletize --episode_length_s 300
# turn-then-forward tip regression (PASS = worst tilt < 0.08 rad):
python -u isaaclab_arena/scripts/h1_base_turn_forward_test.py --headless \
    --disable_pinocchio h1_palletize --episode_length_s 300
# upper-body gravity hold (PASS = lift sag < 3 mm, arm sag < 0.02 rad):
python -u isaaclab_arena/scripts/h1_upper_gain_hold_test.py --headless \
    --disable_pinocchio h1_palletize --episode_length_s 300
# box lock weld (PASS = <10 mm steady drift, box carried, free after release):
python -u isaaclab_arena/scripts/h1_box_lock_test.py --headless \
    --disable_pinocchio h1_palletize --episode_length_s 300
# held-box start state (PASS = carry pose held, box centered/clear, weld
# survives a drive toward the pallet, pallet at its moved position):
python -u isaaclab_arena/scripts/h1_held_box_start_test.py --headless \
    --disable_pinocchio h1_palletize --episode_length_s 300
# box-pool respawn (PASS = each respawn welds a FRESH box at the grip,
# released + parked boxes untouched):
python -u isaaclab_arena/scripts/h1_box_pool_respawn_test.py --headless \
    --disable_pinocchio h1_palletize --episode_length_s 300 --held_box_grip end
```

### Upper-body PD gains (re-tuned 2026-07-11)

The bundle gains (= real-robot `H1_DEFAULT_KP`) are far too soft for the URDF
masses in Isaac: torso_lift (Kp 5000 N/m carrying ~21 kg of upper body) sagged
**42 mm** under gravity, torso_pitch drooped 0.015 rad when leaned forward, and
the arms couldn't squeeze a box between the EE plates (grip force = Kp ×
penetration).  `topstar_h1.py` now uses: torso_lift 200000/4000, torso_pitch
30000/800, shoulder 10000/200, elbow 10000/150, wrist 3000/60
(stiffness/damping, damping ≈ critical).  Verified with
`h1_upper_gain_hold_test.py`: lift sag 41.6 → 1.0 mm, pitch 0.015 → 0.0013 rad,
worst arm sag 0.0008 rad; base tip regression still PASS (tilt 0.0000 rad).
Gotcha for hand-written test poses: `l_elbow` (slot 14) and `l_wrist_pitch`
(slot 16) have URDF axis `0 0 -1`, so their URDF targets are the *negated*
right-arm values (`teleop_h1_zmq.py::_HW_TO_URDF` handles this for teleop).

### Teleop architecture

ROS2 (system Python 3.10) and IsaacSim (conda env_isaaclab Python 3.11) cannot share a process.
ZMQ bridges them — same pattern as `h1_box_scene_isaac_sim.py` + `h1_isaac_ros2_bridge.py`.

```
h1_upper_body_jog  ──/lowcmd──▶  h1_arena_ros2_bridge  ──ZMQ:15558──▶  teleop_h1_zmq (Isaac)
h1_upper_body_jog  ◀─/lowstate─  h1_arena_ros2_bridge  ◀─ZMQ:15559──  teleop_h1_zmq (Isaac)
```

- Port **15558**: Isaac binds PULL, bridge connects PUSH — joint commands `{"q": [...]}`
- Port **15559**: Isaac binds PUSH, bridge connects PULL — joint state `{"q": [...], "dq": [...]}`
- Ports 15555/15556 are used by the original bundle's Isaac Sim script — do not conflict.

### Running (3 terminals)

**Argument ordering (argparse subparsers, easy to trip on):** `h1_palletize`
is a subparser command, not just a positional string — anything registered
by `H1PalletizeEnvironment.add_cli_args()` (`--episode_length_s`,
`--fix_base`, `--held_box_start`, `--held_box_grip`, `--show_pattern_markers`,
`--pattern_nx/ny/layers`, `--dump_head_cam*`) MUST come **after**
`h1_palletize`. Anything registered by `AppLauncher`/the top-level parser
(`--enable_cameras`, `--disable_pinocchio`, `--num_envs`, `--seed`, `--mimic`,
`--headless`) MUST come **before** it — putting `--enable_cameras` after
`h1_palletize` is a parse error (`unrecognized arguments`), not silently
ignored.

**Terminal 1 — Isaac Arena (env_isaaclab Python 3.11):**
```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate env_isaaclab
cd ~/IsaacLab-Arena
python isaaclab_arena/scripts/teleop_h1_zmq.py --disable_pinocchio h1_palletize --episode_length_s 100000
# held-box start (in front of the pallet, box welded in the grip; pair the
# quest node with --box-lock-init).  NOTE: the flag goes AFTER h1_palletize —
# it lives on the environment subparser:
python isaaclab_arena/scripts/teleop_h1_zmq.py --disable_pinocchio h1_palletize --episode_length_s 100000 --held_box_start --held_box_grip end
# head camera check: --enable_cameras is top-level (BEFORE h1_palletize),
# --dump_head_cam is on the subparser (AFTER):
python isaaclab_arena/scripts/teleop_h1_zmq.py --disable_pinocchio --enable_cameras h1_palletize --dump_head_cam
```

**Terminal 2 — ROS2 bridge (system Python 3.10):**
```bash
source ~/topstar_ros2/setup_local.sh
python3 ~/IsaacLab-Arena/isaaclab_arena/scripts/h1_arena_ros2_bridge.py
```

**Terminal 3 — Jog GUI:**
```bash
source ~/topstar_ros2/setup_local.sh
python3 ~/topstar_ros2/example/src/src/h1/topstar_h1/h1_upper_body_jog.py
```

**Terminal 3 (alternative) — Meta Quest teleop (added 2026-07-10):**
```bash
source ~/topstar_ros2/setup_local.sh
python3 ~/IsaacLab-Arena/isaaclab_arena/scripts/h1_quest_teleop.py --swap-hands
```
- Quest deltas → per-arm **placo** IK (`little_top.urdf`, same solver/URDF as the H1 ROS2 node's `get_arm_ik` and the real-robot lerobot4 `topstar_human` driver) → 18-slot hw `/lowcmd`; joysticks → torso (left) / head (right); **hold right grip** = base mode (left JS → vx/vy, right JS x → −ω → `/base_cmd`; arm deltas
  frozen while held; was A until 2026-07-11 — same thumb as the right JS); **A** toggles box lock (2026-07-14): Isaac welds the cardbox to the right EE plate (`h1_box_lock.py` kinematic follow via `/box_lock` → ZMQ boxlock); while locked the right trigger translates BOTH arm targets in parallel (box orientation frozen — boxes stay level), composes with base mode for carrying; press A again to release (box falls free for placing); **B+Y** re-syncs IK targets to robot state; `--test-ik` runs the placo pipeline offline (no Quest).
- **Single-publisher discipline on `/base_cmd`** (found 2026-07-14): the jog GUI
  (`h1_upper_body_jog.py`) used to publish zero twists unconditionally at 50 Hz
  even with publishing disabled — merely having it open interleaved zeros with
  the quest node's commands, so the base controller's target flip-flopped
  0↔cmd and the slewed velocity dithered below the idle threshold (log
  signature: repeated `driving vx=-0.027` → `entering idle hold`, no motion).
  Fixed: the GUI publishes only while enabled AND actively driving (plus one
  explicit zero on the falling edge). Note the topstar_ros2_example package
  installs as a *copy* — after editing src, re-sync the file under
  `example/install/.../dist-packages/topstar_h1/` (done) or rebuild.
  `h1_zmq_drive_probe.py` can impersonate the bridge (basecmd → :15558, reads
  base_pos from :15559) to test the Isaac-side base path in isolation.
- Elbow-flip guard (2026-07-14): the solver-side elbow upper limit is capped at
  `H1_QUEST_ELBOW_MAX` (default −0.05 rad) so IK can never enter the wrong-way
  bend branch, plus an elbow-only placo joints task (weight
  `H1_QUEST_POSTURE_WEIGHT`, default 0.001, 0 disables) biases the null space
  toward the synced bend. Gotcha: a full 7-joint posture task fights EE
  tracking badly (joints-task error is ~rad vs the frame task's meters — 0.02
  cost 104 mm, elbow-only 0.001 costs 2.4 mm in `--test-ik`).
- placo 0.9.23 installed via `pip3 install --user placo` (cmeel layout; the script self-bootstraps `LD_LIBRARY_PATH`/`PYTHONPATH` by re-exec, mirroring `h1_sim.launch.py::_placo_env`). **Gotcha**: seed joints must be clamped inside URDF limits before solving or the QP is infeasible; solver needs  `add_regularization_task(1e-4)` on 0.9.23 or it takes wild steps.
- Live use needs `adb` (not installed yet: `sudo apt install adb`) + the oculus_reader APK (`com.rail.oculus.teleop`) on the headset, connected via USB or `adb tcpip`.  `pure-python-adb` already installed.

**`--disable_pinocchio` is required for H1:**
- System libstdc++ only has `CXXABI_1.3.13`; pinocchio needs `CXXABI_1.3.15`
- H1 uses `JointPositionActionCfg` — pinocchio is not needed
- If pinocchio is ever needed (GR1T2/G1): `LD_PRELOAD=$HOME/miniforge3/envs/env_isaaclab/lib/libstdc++.so.6`

### Pattern generator + wireframe target markers (2026-07-15)

Week-1 deliverable from `PALLETIZING_PROJECT_PLAN.md` ("Implement the pattern
generator: box dims + pallet dims -> per-box target pose, neighbor mask,
approach direction. Unit-test full-pallet layouts."):

- **`~/topstar_ros2/example/src/src/h1/topstar_h1/palletize_pattern.py`** --
  pure stdlib (no ROS2/Isaac dependency), so it is unit-tested standalone and
  reusable by the real `place_box` action server later. `generate_pattern(box,
  pallet, nx, ny, n_layers, box_yaw_deg=...)` returns a `PlacedBox` per cell:
  `center_world`, `quat_wxyz`, `neighbor_class` (`NeighborClass`: NONE / X / Y
  / CORNER -- which already-placed lateral neighbor(s) exist at that cell's
  placement time in row-major raster fill order, matching the plan's "four
  discrete modes total"), and `approach_direction` (unit vector in the
  pallet's local xy frame, toward the already-placed neighbor(s), falling
  back to a caller-supplied default for the first box of a layer). Raises
  `ValueError` with the exact overflow amount if the requested `nx*ny` grid
  does not fit the pallet footprint. 23 tests in
  `test_palletize_pattern.py` (`python3 -m unittest test_palletize_pattern -v`
  from that directory).
- **`isaaclab_arena/embodiments/topstar_h1/h1_pattern_markers.py`** -- Isaac
  side. Imports `palletize_pattern` via a `sys.path` bootstrap (no ROS2
  sourcing needed, the module has zero ROS2 deps). `draw_pattern_markers(nx,
  ny, n_layers, env_origin)` computes the pattern and draws a 12-edge
  wireframe box outline per cell via `isaacsim.util.debug_draw`
  (`_debug_draw.acquire_debug_draw_interface().draw_lines(...)`; this
  extension is enabled by default in `isaaclab.python.kit`, no
  `enable_extension()` call needed). Edge color encodes `neighbor_class`: red
  = none, yellow = -x neighbor, blue = -y neighbor, green = corner. Also
  prints a per-cell table (order/row/col/layer/neighbor/center/approach) to
  the console.
  - **Default pattern is 2x2x2, not 2x3x2**: `cardbox_0` is 0.420 x 0.395 x
    0.395 m and the pallet footprint is only 0.80 x 1.21 m. The defaults yaw
    every box 90 deg (`_BOX_YAW_DEG = 90.0`, matching the `--held_box_grip
    "end"` convention) so the box's short 0.395 m edge faces the pallet's
    tight 0.80 m x-extent (2 * 0.395 = 0.79 <= 0.80). A 2x3 grid in that
    orientation needs 1.26 m of pallet y-extent and the pallet only offers
    1.21 m -- `generate_pattern` correctly rejects it (verified); pass
    `--pattern_ny 3` yourself to see the exact rejection message, or narrow
    `pallet.gap`/widen the USD pallet prop first if a 2x3x2 demo is wanted.
  - Wired into `teleop_h1_zmq.py` as `--show_pattern_markers` (plus
    `--pattern_nx`/`--pattern_ny`/`--pattern_layers`, defaults 2/2/2) on the
    `h1_palletize` subparser, drawn once right after `env.reset()` -- same
    place as `--held_box_start`. Usage: run teleop as usual and add
    `--show_pattern_markers` after `h1_palletize`; the wireframes are static
    (the pallet doesn't move) so they don't need to be redrawn per frame.

### Head camera — Orbbec Gemini 335L on Robot_Head_Tonod_Link (DONE, verified live 2026-07-16)

`isaaclab_arena/embodiments/topstar_h1/topstar_h1.py` now defines
`TopstarH1CameraCfg` / `build_topstar_h1_camera_cfg()`, wired into
`TopstarH1Embodiment.__init__` so `self.camera_config` is always populated
(previously `None`). Only takes effect when the environment is launched with
`--enable_cameras` (`EmbodimentBase.get_scene_cfg()`/`get_observation_cfg()`
only merge `camera_config` in when that flag is set).

- **Asset**: `~/h1_box_scene_bundle/robot/h1/sensors/orbbec_gemini_335L.usd`
  -- a copy of `~/Downloads/orbbec_gemini_335L.usd` (moved next to the other
  H1 assets so it doesn't depend on Downloads being kept around). Inspected
  with `pxr` (only available in the `env_isaaclab6` conda env, not
  `env_isaaclab`): it's a full mesh (housing/bracket/cover-glass) plus three
  real, already-authored `UsdGeom.Camera` prims with baked intrinsics —
  `camera_rgb/Camera_rgb` (focal 1.81mm, aperture 3.84x2.40mm, clip
  0.005-100000), `camera_ir_left/{Camera_ir_left,Camera_Depth_Left}`,
  `camera_ir_right/Camera_ir_right`. Only the RGB one is wired up so far.
- **Two scene entities** in `TopstarH1CameraCfg`:
  - `head_camera_mount` (`AssetBaseCfg`) references the whole USD as a static
    child prim at `{ENV_REGEX_NS}/Robot/Robot_Head_Tonod_Link/orbbec_gemini_335L`
    — since `InteractiveScene` spawns `AssetBaseCfg` entries as a plain local
    xformOp translate/orient on that exact prim path (see
    `interactive_scene.py` `_add_entities_from_cfg`, `AssetBaseCfg` branch),
    nesting it under the head link's own prim makes it move rigidly with the
    head (same trick `GR1T2CameraCfg`/`G1CameraCfg` use for their synthetic
    POV cameras — no physics joint involved, purely USD parent/child).
  - `robot_head_cam` (`CameraCfg`) has `spawn=None` and points its
    `prim_path` directly at the pre-existing `.../camera_rgb/Camera_rgb`
    prim inside that referenced USD, so IsaacLab attaches the sensor to the
    device's real intrinsics instead of spawning a synthetic
    `PinholeCameraCfg` (confirmed in `isaaclab/sensors/camera/camera.py`:
    `spawn is None` skips the spawn call and just asserts a prim already
    exists at `cfg.prim_path`). `height=400, width=640` (matches the
    3.84:2.40 aperture ratio), `data_types=["rgb"]`.
- **Bug avoided, not just fixed**: `GR1T2CameraCfg`/`G1CameraCfg` compute
  their camera fields in `__post_init__` reading a private attribute
  (`self._camera_offset`) that the embodiment's `__init__` sets *after*
  constructing the cfg instance (`cfg = XCameraCfg(); cfg._camera_offset =
  offset`). That's dead on arrival: `@configclass` wraps stdlib
  `dataclasses.dataclass`, which calls `__post_init__` synchronously inside
  `__init__`, before the caller's later `setattr` ever runs — so any
  non-default `camera_offset` passed to `GR1T2JointEmbodiment`/`G1*`
  is silently ignored today (only the module-level default ever takes
  effect; nobody's hit it because nobody's passed a different offset that
  mattered). Did **not** replicate this for H1: `build_topstar_h1_camera_cfg
  (camera_offset)` is a plain function called synchronously inside
  `TopstarH1Embodiment.__init__` with the offset already in hand — no
  `__post_init__`, no ordering hazard. `TopstarH1Embodiment(...,
  head_camera_offset=Pose(...))` actually works.
- **Asset fix — canonical xform ops (2026-07-16)**: first live launch failed
  with `Prim ... is not a xformable prim with standard transform operations
  [translate, orient, scale]. Received type: 'Camera'` at
  `.../orbbec_gemini_335L/camera_rgb/Camera_rgb`. `CameraCfg(spawn=None)`
  requires the pre-existing prim to already use the canonical
  `[translate, orient, scale]` op order; the USD's baked `Camera_rgb` prim
  used `xformOp:rotateYXZ` (Euler) instead. Fixed in the asset file itself
  (not at runtime): ran IsaacLab's own
  `isaaclab.sim.utils.transforms.standardize_xform_ops()` against all four
  baked camera prims (RGB + both IR + depth) in
  `~/h1_box_scene_bundle/robot/h1/sensors/orbbec_gemini_335L.usd`, loading
  the module standalone via `importlib` + `pxr` in the `env_isaaclab6` conda
  env (the function only touches `pxr`, no Kit/omni runtime needed, so this
  works without launching Isaac Sim). Preserves the local pose exactly
  (verified: translate `(0,0,0.08)`, 180° about Y unchanged) while rewriting
  the op stack to `[translate, orient, scale]`. One-time fix to the asset on
  disk — no code changes needed since `topstar_h1.py` references the same
  path.
- **Mount pose — verified live (2026-07-16)**: `_DEFAULT_HEAD_CAMERA_OFFSET`
  is now `position_xyz=(-0.147, -0.011, 0.0)`,
  `rotation_wxyz=(0.707107, 0.0, -0.707107, 0.0)` — user-supplied and
  confirmed correct in Isaac Sim, given as translation `(-0.147, -0.011,
  0.0)` + euler xyz degrees `(0, -90, 0)` relative to `Robot_Head_Tonod_Link`
  (a pure -90° rotation about Y converts to that quaternion via
  `q=(cos(t/2),0,sin(t/2),0)`). Supersedes the earlier placeholder guess
  `(-0.14, 0, 0.05)` / identity rotation, which was wrong.
- **`--dump_head_cam`** (`h1_palletize` subparser, `teleop_h1_zmq.py` only):
  periodically saves the `robot_head_cam_rgb` observation to a PNG (default
  `/tmp/topstar_h1_head_cam.png` every 2 s, override with
  `--dump_head_cam_period_s`/`--dump_head_cam_path`) so the mount can be
  checked/tuned by opening the file, without needing a viewport or editing
  code. Requires `--enable_cameras` **before** `h1_palletize` on the command
  line (it's a top-level `AppLauncher` flag, not a subparser one — see
  "Argument ordering" in the Running section, this is what actually broke
  the first time this was tried live); warns once (doesn't crash) if the
  camera observation isn't present. Captures `env.step()`'s obs dict
  (`obs["camera_obs"]["robot_head_cam_rgb"][0]`, uint8 HWC) — previously the
  teleop loop discarded `env.step()`'s return value entirely, now captured
  as `step_result` unconditionally (cheap, just a tuple reference) so the
  dump logic can read it when enabled.

### Scripted place trajectory (2026-07-17, implemented, partially validated)

Week-1 "scripted transport & approach primitive" deliverable from
`~/topstar_ros2/docs/PALLETIZING_PROJECT_PLAN.md`. Full design history and
status in `isaaclab_arena/embodiments/topstar_h1/PLACE_TRAJECTORY_PLAN.md`
(written before implementation, then updated in place as live testing
overturned the original mechanism — read that "Status" section, not just
the plan below it).

**Problem**: manually teleoping the pitch-forward + arm-extend motion to
place a box on the far side of the pallet is hard — pitching drops the held
box (~0.45m across the full pitch range) while the arm's own vertical
authority is only clean for ~10-15cm before a kinematic singularity.

**What was tried and rejected**: staging `torso_lift` + `torso_pitch` as the
reach mechanism (the original approved plan). Offline-tested against the
exact live-measured scenario and it doesn't work — pitching away from the
carry pose's start value made tracking error *worse*, not better (verified
twice: FK sweep, then actual iterative IK convergence). Pure arm reach
plateaus 100-150mm short of the far row regardless.

**What actually works (live-discovered, 2026-07-16/17)**: raising the
**shoulder joint** (`Robot_Right_Hand_base_Joint`, hw slot 4), mirrored to
the left arm to keep the box level, **frees up torso_pitch to help** — from
the un-raised carry pose, pitch doesn't help; after raising the shoulder, it
does. Found by the user via `h1_upper_body_jog.py` (raw per-joint jogging,
not IK — "it's difficult to use IK" for this motion) reaching the far row
live, then confirmed by snooping the live joint state (see "Live introspection"
below) and running it through `FullBodyFK`: the box's right-EE weld point
landed within ~20mm of the far row's depth target.

- **`isaaclab_arena/scripts/h1_quest_teleop.py`** (extended): `FullBodyFK`
  (read-only FK on the full `h1_with_ee.urdf` model, used to measure how
  torso/arm joints move the box), `FullBodyRightArmIK` (frame-task IK on the
  same full model, torso fixed per-solve, only the 7 right-arm joints free —
  used instead of the existing `ArmIK`/`little_top.urdf` because that
  reduced model has no torso joints at all, so it can't represent a torso
  that's actually pitching between stages), `solve_place_trajectory()`
  (stages shoulder_base + pitch scaled from the one validated live data
  point, then a residual IK correction stage), `target_cell_box_pose()`
  (wraps `palletize_pattern.generate_pattern()`), `BasePoseListener` (ZMQ SUB
  for live `base_pos`/`base_quat`). Gotcha hit building `FullBodyRightArmIK`:
  `h1_with_ee.urdf` has `velocity="0"` on every joint (real-robot spec
  placeholders); with `enable_velocity_limits(True)` that freezes every
  joint solid — same documented gotcha as the original carry-pose derivation
  in `h1_held_box_start.py`, same fix (`model.set_velocity_limits(2.0)`).
- **X button** (previously reserved/unused) triggers a live solve+playback
  of the trajectory to `--place-row/--place-col/--place-layer`'s target
  cell, from wherever the base and arms currently are. `--test-plan-place`
  runs the same solve offline (no Quest/ROS2/Isaac) and prints per-stage FK
  error + joint margins.
- **State channel PUSH/PULL → PUB/SUB**: `teleop_h1_zmq.py`'s state socket
  changed from `zmq.PUSH` to `zmq.PUB` (still `.bind()`), and
  `h1_arena_ros2_bridge.py`'s from `zmq.PULL` to `zmq.SUB` (with
  `SUBSCRIBE, b""`) — needed so `h1_quest_teleop.py`'s new `BasePoseListener`
  could subscribe to the same feed independently without splitting messages
  away from the bridge (PUSH/PULL round-robins across connected pullers;
  PUB/SUB gives every subscriber every message). Also surfaces
  `base_pos`/`base_quat`, which the bridge's `/lowstate` relay silently
  dropped (that message type has no room for them) — `h1_quest_teleop.py`
  now reads them directly off the ZMQ feed instead of through ROS2.
- **`place_trajectory_staging.py`** (`~/topstar_ros2/.../topstar_h1/`, pure
  stdlib, 12 unit tests) implements LERP-based torso_lift/pitch staging —
  written for the original (rejected) mechanism, **not used** by
  `solve_place_trajectory` anymore. Left in place, still correct, in case a
  future revision wants staged interpolation again for a different DOF.

**Live introspection technique** (reusable): `teleop_h1_zmq.py`'s state PUB
includes `q` (18-slot hw joint positions) alongside `base_pos`/`base_quat`,
which nothing else was reading. A short SUB-socket snoop script
(`zmq.SUB`, `SUBSCRIBE b""`, connect to port 15559) reads a few live
messages directly — this is how the shoulder-raise discovery got confirmed
numerically instead of staying a qualitative impression. Reading the
USD stage's `Transform` in the Isaac GUI does **not** reflect live physics
state reliably (tried first, didn't update) — use the ZMQ snoop instead.

**Known limitation — not solved this session**: the mechanism fixes the
*row* (depth/x) reach problem but not *column* (lateral/y). The residual IK
correction stage plateaus around ~100mm of remaining error when a target
also needs significant y travel (confirmed: the live-measured success case
itself was 290mm from the exact target cell center, 270mm of it in y). Scope
is deliberately "get the box into the correct row, hand off to manual teleop
for final lateral placement" for now — check the final stage's
`right_err_mm` before trusting a playback result.

**Untested live**: the X-button trigger and PUB/SUB plumbing are
offline-verified only (`--test-plan-place`) — the live session used for
calibration ran the pre-change scripts throughout (no hot-reload). Next live
session: restart both `teleop_h1_zmq.py` and `h1_quest_teleop.py` first.

**Keyboard cell selection + box respawn (2026-07-17, also untested live)**:
`h1_quest_teleop.py` now reads single keypresses from its own terminal
(cbreak mode, background thread, quietly disabled if stdin isn't a TTY) to
augment Quest, since a controller has no natural input for either of these:
`r/R` `c/C` `l/L` adjust the X-button trajectory's target row/col/layer live
(previously fixed at `--place-row/col/layer` until restart); `p` prints the
current target; `n` respawns a fresh box at the held-start carry pose
(arms/torso reset, box re-welded) **without moving the base**, via a new
`/respawn_box` topic threaded through the same ROS2→ZMQ pipe as `/box_lock`.
On the Isaac side, `h1_held_box_start.apply_held_box_start()` gained a
`teleport_base: bool = True` param — `False` skips the root pose/velocity
write; everything else (carry-pose reset, box placement relative to the
*current* root pose, weld engage) is unchanged, since the function already
read the live root pose for the settling step. Lets an operator run repeated
pick/place cycles without restarting the sim. Full detail in
`PLACE_TRAJECTORY_PLAN.md`'s "Update (2026-07-17)" section.

**Bug found and fixed via first live X-button test (2026-07-17)**: left arm
didn't mirror the right during playback, elbow driven near-straight (real
singularity — confirmed via live joint snoop: right elbow hw=-0.7491, left
=-0.0508). Releasing box-lock afterward sent the box flying (left plate had
likely been jammed against it). Root cause: `_RIGHT_TO_LEFT_MIRROR_SIGN` was
derived correctly in URDF convention but applied as-is to hw-convention
arrays — wrong, because the hw↔urdf sign flip differs between arms (right
all +1; left flips elbow + wrist_pitch), which changes the mirror once both
sides are in hw. Correct hw mirror: `[-1,1,-1,+1,-1,+1,-1]` (elbow/wrist_pitch
do NOT flip in hw, unlike urdf) — re-derived and cross-checked against both
carry profiles independently, fixed in `h1_quest_teleop.py`. Not yet
re-tested live.

**Second bug found via next live test, same session**: with the sign fixed,
left arm "was following the right arm initially and then it snapped,
piercing into the box" — box shook violently, got pushed against the
pallet, impact yawed the base. Root cause this time was conceptual, not a
sign error: `box_lock` only welds the box to the **right** EE, so the box's
real position is exactly wherever the right EE goes — mirroring the right
arm's *joint solution* for the residual (arbitrary x/y/z) stage implicitly
assumes the target is y-symmetric, which only held for the one validated
(near-centered) reference case. For an off-center target the right arm
bends however it needs to, and mirroring those joints sends the left plate
to the mirror-*image* location, not "the other side of wherever the box
actually is" — it swings at a phantom target while the real box (rigid on
the right EE) is elsewhere, and they collide. Fixed: the residual stage now
solves the left arm **independently** (new `FullBodyArmIK(side)`,
generalized from the right-only class) toward
`right_actual_ee_pos + grip_offset`, where `grip_offset` (left EE − right
EE, base-local) is measured once at trigger time from the still-valid
grip. Ramp stages keep the joint mirror (correct there — genuinely
symmetric shoulder+pitch motion). `SolvedPlaceStage` gained `left_err_mm`
so both arms' tracking error are visible independently. Offline-verified:
left converges to 0.00mm against its own target while right has a genuine
105mm residual — properly decoupled now. Not yet re-tested live. Full
detail in `PLACE_TRAJECTORY_PLAN.md`'s "Second bug found" section.

**Third bug found (2026-07-17, respawn feature)**: "robot reset to initial
held position and immediately replay the motion sequence and drop the box."
Two separate bugs: (1) `h1_quest_teleop.py`'s own IK targets don't know a
respawn happened, keep publishing stale (pre-respawn) values, so Isaac's PD
controllers visibly drag the fresh carry-pose arms back toward wherever they
last were — looks like a replay. Fixed: `n` now schedules
`self._sync_targets()` ~3s later (`H1_QUEST_RESPAWN_RESYNC_DELAY_S`), after
Isaac's settle sequence finishes. (2) `_box_lock_want["seen_true"]` is a
permanent session-lifetime latch in `teleop_h1_zmq.py`, never reset — so the
guard meant to protect a freshly-engaged weld from a stale `lock=False`
republish was silently inert once any `True` had ever been seen (which
happens almost immediately in any session). If the box had been released
after the previous placement, the Quest node's stale `False` overrode the
respawn's re-engage. Fixed: reset `seen_true = False` alongside `guard =
True` in both the `--held_box_start` and respawn code paths. Neither fix
touches trajectory-solving; offline tests still pass. Not yet re-tested
live — third live-test round this session, treat the whole
respawn+resync+guard interaction as still fragile until confirmed.

**Fourth bug found (2026-07-17, directly reported)**: "box has an even
larger tilt angle [after X] and I cannot teleop it back to the leveled
pose... will roll over that pallet and fall." Design gap, not a small
bug: `solve_place_trajectory` only ever tracked a position target, never
orientation. box_lock welds the box rigidly to the right EE's full pose;
the ramp stages set the shoulder joint directly and re-`seed()`ed every
stage, which re-derives orientation from raw (uncompensated) FK — nothing
corrected for the shoulder rotation also rotating the EE, so tilt
compounded stage over stage. (The user's own description of the validated
technique was "raise via shoulder **but level the box at the same time**"
— the leveling half was never implemented, only the reach half.) Fixed:
each arm's IK is now seeded exactly once (locking orientation), and every
stage — including the pitch ramp, previously direct joint assignment —
tracks position only via `apply_delta`, so the locked orientation persists
and the solver's other joints compensate to hold it. Verified: measured
orientation deviation across the trajectory is now bounded (~12.5° peak,
~7.1° final) instead of unbounded. Does NOT fix the box's pre-existing
~8° baked-in tilt from grip time (separate, predates this session) — only
stops the scripted motion from making it worse. Full detail in
`PLACE_TRAJECTORY_PLAN.md`'s "Fourth bug found" section.

**Fifth bug found (2026-07-17, tilt fix confirmed, respawn still broken)**:
the third bug's scheduled resync (3s later) only fixed this node's own
state after the fact — it never stopped `h1_quest_teleop.py` from
publishing stale `/lowcmd` every tick in the meantime, which
`teleop_h1_zmq.py`'s ZMQ receiver (its own thread, never pauses) applies
unconditionally — overwriting the fresh carry pose within one message
round-trip, long before the timer fires. Fixed: `_tick()` now stops
publishing anything at all for the whole wait window (also drains
`self.quest.poll()` without applying it, to avoid a jump-on-resume). Not
yet re-tested — fifth live-test-driven fix this session. Full detail in
`PLACE_TRAJECTORY_PLAN.md`'s "Fifth bug found" section.

**Sixth bug found (2026-07-17, restarted+retested, still moved)**: node
logs showed `torso=(+0.34,+0.57)` identical before AND right after the
"resync done" message — the fixed 3s delay fired before Isaac's respawn had
actually finished settling (sim slower than assumed), so `_sync_targets()`
locked onto stale `/lowstate` instead of the fresh carry pose (~0.30
pitch, not 0.57), and teleop resumed based on the wrong reference. Fixed:
replaced the fixed delay with actual state matching — new
`_CARRY_Q_HW_BY_GRIP` (both grip profiles, hw convention) + `--held-box-grip`
CLI arg (must match `teleop_h1_zmq.py`'s `--held_box_grip` — note the
different flag spelling). Waits until `/lowstate` actually matches the
expected carry pose (`H1_QUEST_RESPAWN_MATCH_TOL_RAD`, default 0.03 rad),
with `H1_QUEST_RESPAWN_WAIT_TIMEOUT_S` (default 20.0) as a fallback. Not
yet re-tested — sixth live-test-driven fix this session. Full detail in
`PLACE_TRAJECTORY_PLAN.md`'s "Sixth bug found" section.

**Eighth bug found (2026-07-17, geometry review revealed it)**: "for
row:0, col:1, expected pitch-forward only, but it moved the box left as
far as it can — all targets shifted left regardless of column." Traced:
the ramp stages (4 of 5, most of the trajectory) LERPed toward
`ramp_target_pos`, computed purely from the shoulder+pitch schedule via
raw FK — it never read the requested column's y at all, so the ramp made
zero y-progress regardless of column, cramming all column-specific
correction into the single, shortest final stage. Fixed: every stage now
LERPs directly toward the true final target instead of that
column-blind intermediate waypoint. Verified: error now decreases
progressively from stage 1 (was flat for 4 stages, corrected only once at
the end). col:0 remains genuinely far out of reach (~470mm — crossing the
right arm across the body, a real limit, not this bug). Side finding, not
yet fixed: the dry-run's `within_limits` diagnostic uses joint limits
calibrated for a different model than the one actually being solved, so
`R_lim/L_lim=False` in earlier dry-run output was partly a diagnostic
false alarm, not necessarily real. Full detail in
`PLACE_TRAJECTORY_PLAN.md`'s "Eighth bug found" section.

**Ninth issue found (2026-07-17, immediately after)**: "motion seems to
regress, tilt angle gets larger, box interferes with pallet and causes
robot yaw." The eighth fix (LERP every stage to the true target) is
correct but means each stage now covers a much bigger position delta than
before, and holding a fixed orientation through a large single-stage
position change is a genuine kinematic difficulty — measured worst-case
deviation for col=1 went from ~12.5° (fourth-bug baseline) to ~27.6°.
Raising the orientation task's weight again barely moved the number
(confirmed: reachability tradeoff, not a tuning problem). Fix: more,
smaller stages — `n_stages` default 4→12 roughly halves worst-case
deviation (measured across 4/8/12/16 for both columns, diminishing returns
past ~12-16); `H1_QUEST_PLACE_STAGE_DURATION_S` default 1.5→0.5s so total
playback stays ~6s instead of tripling. Doesn't fully fix the hardest
targets (col=0-style still ~26° peak) — real kinematic limit, not
something staging alone resolves. Full detail in `PLACE_TRAJECTORY_PLAN.md`'s
"Ninth issue found" section.

**Tenth issue found (2026-07-17, next live session)**: "box still
interferes with the pallet top and the robot is pushed to turn its
direction" — confirms the collision risk the plan had explicitly flagged as
unchecked. Cause: the ramp stages' position target was a straight-line LERP
from carry position to hover target, with zero notion of clearing anything
in between — it only ever looked safe because the one validated reference
case had a benign height profile. Fix: new `_lift_traverse_descend_pos`
helper reshapes the path into rise-to-transit-height → horizontal traverse
→ descend, instead of a diagonal line, so the box is never descending while
still moving horizontally across the pallet. Transit height =
`max(start_z, target_z) + _PLACE_TRANSIT_CLEARANCE_M` (0.08m default) — a
heuristic margin, since `PalletSpec` doesn't model wall/rim geometry at
all; raise the constant if contact is still seen live. Verified via a
standalone probe (path peaks above both endpoints, never dips below either
while moving horizontally) and `--test-plan-place` (final residual error
unchanged, ~104mm vs ~114mm pre-fix — reach/convergence not regressed).
Full detail in `PLACE_TRAJECTORY_PLAN.md`'s "Tenth issue found" section.
Not yet re-tested live.

**Eleventh issue found (2026-07-18, reported live)**: "box shaking
excessively along scripted trajectory" (suspected plate contact, esp. left
arm, + not enough box-to-pallet-top clearance) and "hold the box initially
with level orientation". Three compounding root causes found and fixed:
(A) the box really was spawned tilted ~7.5° — the "mesh is counter-rotated
inside the prim" note above was wrong (see the corrected TODO 1 below);
spawn quats are now level. (B) `solve_place_trajectory` drove the right EE
LINK ORIGIN to BOX-CENTER targets; measured offset between those points at
the "end" carry pose is (+0.037, −0.300, −0.056) m — so every placement
was ~30 cm off in y (this, not reach, was most of the documented "column
limit"/290 mm reference-case residual) and the real box center rode 56 mm
lower than solved, planting the box bottom into the pallet top at hover.
Targets are now converted to EE space via `_BOX_CENTER_B_BY_GRIP` (offset
captured in the EE local frame at the carry pose, rotated by trigger-time
EE orientation; `solve_place_trajectory` gained a `grip` param wired to
`--held-box-grip`). (C) the ramp stages still joint-MIRRORED the left arm
— invalid ever since the eighth fix made them track non-symmetric targets;
the left plate chased a mirror-image phantom into the welded box (~10 mm
nominal clearance/side). Left arm now solved independently on EVERY stage
toward `right_actual + grip_offset`, backed off `H1_QUEST_PLACE_LEFT_GAP_M`
(default 0.010 m) along the grip axis; `_PLACE_TRANSIT_CLEARANCE_M` raised
0.08 → 0.12. Verified offline: row0/col1 final residual 104 → 13 mm
(per-stage error now actually converges), row1/col1 2.7 mm, col0 470 →
~230 mm (still a genuine cross-body limit — large `right_err_mm` = finish
manually). `h1_held_box_start_test.py --held_box_grip end` re-run headless
with the level spawn: PASS (drift 2.45/3.87 mm, pallet gap 0.323 m).
Follow-up not done: `BoxSpec` dims are still the tilted-AABB values
(0.395/0.42/0.395 vs true 0.42/0.35/0.35) — currently errs in the safe
direction (targets ~22 mm high, loose packing); correct before flush-
seating/ACT work. Full detail in `PLACE_TRAJECTORY_PLAN.md`'s "Eleventh
issue found" section. Not yet re-tested live.

**Twelfth revision (2026-07-18, same day)**: scripted trajectory now uses
live inputs and an honest solver. (1) Base yaw compensated — the trigger
used to discard `base_quat` (translation-only target conversion, silently
assumed the parked identity-yaw heading; any base turn rotated all targets).
(2) `teleop_h1_zmq.py` publishes `box_center`/`box_quat` (true box pose,
world, wxyz) in the ZMQ state payload; the solver uses it for the EE↔box
offset (exact regardless of grip), falling back to the carry-pose-derived
offset for old feeds; trigger log shows source + box tilt (warns >5°).
(3) `FullBodyArmIK` never actually enforced "only this arm's joints free" —
placo recruited torso pitch during every solve, then `set_torso` snapped it
back (0.1→84 mm on identical joints): ALL previous per-stage error numbers
were optimistic phantoms. Non-arm DOFs are now masked. (4) QP oscillation
at the reach boundary (~80 mm between position-good and orientation-good
phases) — convergence now keeps/restores the best state, runs bounded
settle iterations (without which the residual stage never solved at all),
and survives infeasible QPs (elbow-cap-aware retry clamp). (5) Residual
stage frees torso pitch and commands the SOLVED pitch. Offline matrix:
row0/col1 0.48 mm, yaw+15° 0.16 mm, row1 0.00 mm; col0/yaw−20/layer1
remain honestly hard (140–250 mm → finish manually). RESTART
`teleop_h1_zmq.py` to get the box keys on the feed. Full detail in
`PLACE_TRAJECTORY_PLAN.md`'s "Twelfth revision" section. Not yet re-tested
live.

**Box pool for respawns (2026-07-18, user-requested)**: the 'n' respawn now
consumes a FRESH box instead of teleporting the single cardbox back into the
grip (which yanked it off the pallet after a placement). One-time USD edit
`add_cardbox_pool.py` (repo root, run with pxr in `env_isaaclab6` — already
run; backup `h1_box_scene_edit.usd.bak-single-box`) authored
`/World/Boxes/cardbox_pool_1..7` as *internal references* to `cardbox_0`
(mesh+collision+material+rigid-body APIs all compose through), parked level
in a 2×4 grid on the open aisle floor (x −1.30/−1.95, y −4.0…−5.8, the
pallet's pre-2026-07-10 spot). `h1_palletize_environment.py` registers them
as scene entities `cardbox_1..7` via `--num_boxes` (default 8, subparser
arg; 1 = old single-box behavior and the fallback if the USD lacks the pool).
`teleop_h1_zmq.py` keeps a pool cursor: `--held_box_start` consumes box 0,
each respawn releases the weld, swaps it to the next box (new
`H1BoxLock.set_box`, also switches the `box_center` state-feed publish) and
runs the usual carry-pose apply with `teleport_base=False`; exhausted pool →
warning, 'n' ignored (no wraparound — that would grab a placed box). The
operator parks the robot so the fresh box's carry pose doesn't intersect
already-placed boxes. Verified headless: `h1_box_pool_respawn_test.py`
(start + 2 respawn cycles: fresh box welds at the grip target, released
boxes and still-parked pool boxes each move < 30 mm during respawns) and
`h1_held_box_start_test.py` re-PASS with the 8-box scene. Not yet tested
live.

**Placement planner (2026-07-19, user-requested)**: new offline planner
`isaaclab_arena/scripts/h1_place_planner.py` — for every 2×2×2 cell in
placement order, finds the best base PARKING pose (yaw 0, facing −x) and
the best COMMANDED end target = cell + standoff AWAY from already-placed
neighbors (+x / +y, default 0.10 m; closed manually after playback). Two
gates per candidate: (1) geometry — AABB sweep of the carried box (inflated
0.03 m for EE plates + tracking) along the actual lift→traverse→descend
path, ≥ 0.02 m from every placed box, INCLUDING the start carry pose (at
carry height the box rides below the layer-0 tops, which is what forces
layer-1 parking far back); (2) reach — `solve_place_trajectory` run for
real, final right-EE residual ≤ 5 mm. Search: +x standoff (up to 0.55 m
slide-in for layer ≥ 1), start torso lift (≤ 0.45 hw limit), parking depth
d ∈ [0.621, 1.016] (the two live-validated endpoints); base keep-out
x ≥ −0.754 (0.406 m chassis half-depth from base_link.STL vs pallet edge
−1.16). **Results**: all 4 layer-0 cells hit 0.48 mm from the same relative
pose — park at (cell_x+standoff_x+1.016, cell_y+standoff_y−0.0299) — with
0.04–0.13 m swept clearance; layer-1 near row best ~25 mm (d=0.85, 0.55 m
standoff, seat by DRIVING forward — hovering box clears layer-0 tops by
3 cm), far row best ~138 mm (start-clearance pins d=1.016 at layer-1
height → over the reach envelope; manual finish). Extra lift (0.35→0.45)
bought only ~4 mm — layer-1 is pitch-mechanism-limited, not lift-limited.
Writes `~/h1_box_scene_bundle/h1_pallet_plan.json`; `h1_quest_teleop.py`
auto-loads it (`--pallet-plan`, '' disables): cell selection keys print the
recommended parking pose + lift + seat push, X commands the plan's offset
target instead of the raw cell center, and warns if parked > 5 cm/5° or
lift > 0.02 off plan. Regenerate with
`python3 isaaclab_arena/scripts/h1_place_planner.py` (a few minutes; every
candidate goes through the real QP). Not yet tested live.

**Respawn teleports to planned parking (2026-07-19, user-requested)**: with
a plan loaded, 'n' now also TELEPORTS the base to the plan's parking spot
for the CURRENT target cell (select the cell with r/c/l BEFORE 'n') — safe
because the planner's start-pose gate verified the carry pose there clears
every placed box. Teleport, not drive, per user decision: the real robot's
commercial base controller parks reliably, the sim swerve controller
doesn't. Plumbing: `/respawn_box` is now `std_msgs/Float64MultiArray`
(empty = plain respawn/base stays, `[x, y, yaw_rad]` = teleport) — **quest
node and bridge must be restarted together** (topic type change); bridge
adds optional `"base_pose"` to the ZMQ respawn message;
`apply_held_box_start(..., base_pose_w=(x, y, yaw))` does the root write
(z from profile spawn height). Torso lift is NOT teleported — jog to the
plan's lift before X (X-time warning reminds you). Verified headless:
`h1_box_pool_respawn_test.py` respawn cycle 2 now goes through
`base_pose_w` and checks the base lands within 30 mm / 2° of the request.

**Startup parks at the plan too (2026-07-19, user-requested)**:
`teleop_h1_zmq.py --held_box_start` now spawns at the plan's parking spot
for `--start_cell` (new subparser args `--pallet_plan` — same JSON default —
and `--start_cell R C L`, default `0 0 0` = first in placement order)
instead of the fixed profile pose, so the FIRST box needs no manual drive
either; missing plan/cell → fixed pose + warning. To match, the quest
node's `--place-col` default changed 1 → 0 (cell (0,0,0) is the placement
start; col 1 was the historical validated-reference cell). NOTE for bare
`--test-plan-place` runs: the default base x/y is still the COL-1 parking
spot — pass `--place-col 1` (or `--place-base-y -0.5199` for col 0) to
reproduce the 0.48 mm reference; both pairings re-verified 0.48 mm.
Smoke-tested headless: startup log shows "parked at plan cell (0, 0, 0)
(-0.746, -0.520)", box center (-1.470, -0.520, -0.451) = the planner's
predicted carry position, no sim-step errors. Also fixed same day: the
first live run threw `cannot access local variable '_respawn_base_pose'`
every step — `main()` assigned it without declaring it global (the headless
test misses this path since it calls `apply_held_box_start` directly).

### Next phase TODOs

1. **Held-box start state — DONE 2026-07-15 (revised same day, see below).**
   `teleop_h1_zmq.py --held_box_start` (module:
   `isaaclab_arena/embodiments/topstar_h1/h1_held_box_start.py`) teleports the robot to
   (+0.09, −0.10) facing the pallet (−x), poses the arms in a placo-solved symmetric
   carry pose (plates vertical/parallel, faces 0.416 m apart around the 0.395 m box),
   places the box in the grip and engages the box-lock weld — box front face 0.30 m
   from the pallet near edge, box bottom ~2 cm above the pallet top.
   Run the quest node with `--box-lock-init` so its A-button toggle starts ENGAGED
   (its 1 Hz `/box_lock` refresh would otherwise release the weld — the sim side now
   also guards the start weld by ignoring `False` until the publisher has sent one
   `True`). **Restart the quest node whenever the sim restarts** (or press B+Y): a
   node from the previous session keeps streaming its stale /lowcmd targets, which
   drags the arms out of the carry pose while the weld holds the box at the captured
   EE offset → "arms down, box floating in mid-air".
   **Revision 2026-07-15:** the first carry pose (grip center x=−0.32 in the base
   frame) put the box AABB partly INSIDE the robot's own base-cabinet/torso collision
   volume (cabinet front face at x=−0.406, 1.16 m tall) — invisible to the first
   headless test because its "expected center" was computed from the same box-root
   offset bug (see below), so drift/creep looked fine even though the box was
   penetrating the chassis; the live symptom was "box floating far from the robot"
   once teleop moved the arms and the (also-buggy) 2 m offset threw it clear across
   the room. Re-solved with an added self-clearance check (box AABB vs. sampled mesh
   vertices of `base_link` / torso links, >= 15 mm margin): new grip center
   (−0.740, 0.0, +0.704) in the base frame, torso lift 0.35 / pitch 0.30 (hw). Because
   the new grip reaches 0.42 m further forward, `ROBOT_POS_W` moved from x=−0.33 to
   x=+0.09 (robot backs up) to keep the same 0.30 m pallet gap — **do not tune
   `CARRY_Q_URDF` without also re-deriving `ROBOT_POS_W`** from the pallet-gap formula
   in the module (front_face_x − pallet_near_edge_x = +0.30, front_face_x = root_x +
   BOX_CENTER_B.x − 0.21).
   Verified: `h1_held_box_start_test.py` (weld drift 2.5 mm hold / 3.9 mm driving,
   joint err 0.001 rad, zero base creep, plus a release-and-drop check: box falls
   from the grip and settles near the floor rather than teleporting away, ruling out
   a wrong root↔geometry offset).
   Gotchas hit: `h1_with_ee.urdf` declares `velocity="0"` on every joint — placo with
   velocity limits enabled is frozen, without them it explodes; call
   `robot.set_velocity_limits(2.0)` first. The carry pose must respect the QUEST node's
   `_ARM_LIMITS` (tighter than the URDF, e.g. wrist-pitch ≤ 0.436 hw) or `_sync_targets`
   clamps and jumps the arms on attach. Mirror map in hw convention flips joints
   1,3,5,7 (yaws/rolls). cardbox_0's rigid-body root sits at the box BOTTOM CENTER:
   geometry center = root + R_box·(0, 0, 0.175). **CORRECTED 2026-07-18:** the box
   now spawns LEVEL (identity roll for `side`, pure +90° yaw for `end`). The earlier
   claim here — authored world rotation (0.99756, 0.06976, 0, 0) ≈8° roll with "the
   mesh counter-rotated inside the prim, so keep that quat or the visual box appears
   tilted" — was exactly backwards: pxr inspection shows the mesh child has an
   IDENTITY local rotation (points axis-aligned in the root-local frame, bottom at
   local z=0), so keeping the authored quat is what held the box visibly tilted
   ~7.5° (the "pre-existing baked-in tilt" of the fourth place-trajectory bug) and
   hung one bottom edge ~2 cm lower over the pallet. True box size is
   0.42 × 0.35 × 0.35 m — the 0.42 × 0.395 × 0.395 figure used in these notes is the
   AABB of the *tilted* box. TRAP: measuring the prim's own
   translate op against the world bbox misses the /World/Boxes parent xform (y+2.1535)
   and reads as a bogus (0, 2.13, 0.17) offset — that bug put the "held" box 2.1 m to
   the robot's left. Use ComputeLocalToWorldTransform (what ObjectReference does).
   **Two grip profiles (2026-07-15):** `--held_box_grip {side,end}` (default `end` as
   of 2026-07-17 — matches the pattern generator's `box_yaw_deg=90` assumption and the
   scripted place trajectory, the profile actually in use; was `side`) on
   the `h1_palletize` subparser. `side` grips the box's +-y "long" faces (0.42x0.395 m
   each, plates 0.416 m apart around the box's 0.395 m y-extent) -- the pose above.
   `end` yaws the box 90 deg about z and grips its +-x "end" faces instead (0.395x0.395 m
   each, plates 0.440 m apart around the box's 0.42 m x-extent): grip center
   (-0.723, 0.0, +0.701) base frame, same lift 0.35/pitch 0.30, robot at x=+0.06 (a bit
   more forward than `side`'s +0.09, since the narrower cross-section pulls the box's
   world-frame front face closer for the same 0.30 m pallet gap). Both solved with the
   same self-clearance search (scratchpad `carry_pose_ik5.py` for `side`,
   `carry_pose_ik6.py` for `end`) and verified independently with
   `h1_held_box_start_test.py --held_box_grip end` (weld drift 2.5/3.9 mm, joint err
   0.001 rad, zero creep -- same numbers as `side`). The `end` box orientation composes
   a +90 deg z rotation with the same baked-in ~8 deg mesh-roll quat (`quat_mul(YAW90_Z,
   side's box_spawn_quat)`, not applied after) so the small roll correction rotates
   with the box rather than fighting it.
2. **IK-based teleop** — H1 currently uses direct joint position control. EEF-space control
   for policy training needs PINK IK integration (similar to GR1T2 embodiment).
3. **Cameras** — `enable_cameras=False` currently. Add wrist/overhead cameras for vision-based policies.
4. **LeRobot policy eval** — wire `h1_palletize` into `lerobot-eval` with appropriate
   obs/action space mapping via `IsaacLabArenaEnvCfg`.

