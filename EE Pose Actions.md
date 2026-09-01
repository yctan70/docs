# Incremental EE pose as the ACT action space

Status as of 2026-09-01. Supersedes the joint-increment experiment described at the end of this file.

## Why

The policy used to predict seven joint increments per arm. Two of those joints cannot be learned, and the reason is geometric rather than statistical: a 7-DOF arm has a circle of postures that put the plate in the same place, and the demonstrations did not pick the same one twice. Measured on the eight `r2c0L0` demonstrations, at **matched plate position**:

| quantity | scatter across demos | travel during the push |
|---|---|---|
| `r_elbow_yaw` | 13.2 deg | 0.9 deg |
| `r_wrist_pitch` | 7.2 deg | 8.3 deg |
| elbow swivel | 9–12 deg | 5–11 deg |

The policy's per-joint accuracy tracks that ratio exactly. Integrating its predicted increments over the push and comparing against the demonstration:

```
r_shoulder      +0.79     scatter is a third of its motion
r_wrist_roll    +0.46
r_wrist_yaw     +0.38
r_elbow         +0.02
r_wrist_pitch   -0.31     scatter is the whole motion
r_elbow_yaw     -0.90     scatter is fourteen times the motion
```

Negative means it integrates the joint the wrong way. No loss function fixes an inconsistent target, so this is not something more training or a cumulative loss term can reach.

The plate path, by contrast, is tight. At matched push fraction the spread across the same eight demos is 4–12 mm in x, 4–23 mm in z against a 145 mm push — **3–16% of the motion** — and the plate orientation is *constant*: 0.0 deg from episode start to end, in every demonstration of both corner cells.

So express the arm as (EE pose, swivel). Per arm that is 6 + 1 = 7 numbers for 7 joints: exactly determined. Verified, not assumed — targeting one demo's pose and swivel while **seeding from another demo's joints 12.3 deg away** (max 36.5) recovers the target configuration to **0.0000 deg**. The redundancy is removed
by construction, and nothing the operator did is overwritten.

## The composition

```
observation.state  31 channels
  0- 1  torso_lift.pos, torso_pitch.pos
  2- 3  head_yaw.pos, head_pitch.pos
  4-12  r_ee.x/y/z + r11 r21 r31 r12 r22 r32     absolute pose, 6D rotation
 13-21  l_ee.x/y/z + the same six
 22-23  swivel.right, swivel.left                 degrees
 24-26  plate.gap, plate.misalign_deg, plate.lateral
 27-29  mobile_base.x/y/theta
    30  flush.score                               LAST, and must stay last

action  22 channels
  0- 1  torso_lift.pos, torso_pitch.pos           ABSOLUTE
  2- 3  head_yaw.d, head_pitch.d                  increments
  4-10  r_ee.dx dy dz drx dry drz dswivel
 11-17  l_ee. the same seven
 18-20  mobile_base.x/y/theta                     ABSOLUTE
    21  grip.lock                                 ABSOLUTE
```

The 14 arm joints leave the observation because (pose, swivel) replaces them losslessly. `plate_right.x/y/z` also goes: it *was* the right EE position, so keeping both would have been the same numbers twice.

### Conventions, both halves in the BASE frame

Translation is a difference of base-frame positions. Rotation is a base-frame rotation vector, integrated by **left** multiply:

```
D = R_cur @ R_prev.T                 # what the dataset writes
R_new = expm(D) @ R_prev             # how the driver integrates
```

This matches `kinematics.ArmIK.apply_delta`, which already existed:

```python
self.target[:3, 3] += dpos
self.target[:3, :3] = drot @ self.target[:3, :3]
```

so the runtime is that tested function rather than a parallel one, and inherits its `_DPOS_CLIP` / `_DROT_CLIP` guards. The body-frame alternative (`R_prev.T @ R_cur`, right multiply) is equally valid in isolation and was the first cut here. It is wrong only in that it silently disagrees with the integrator — which costs nothing while the plate never turns, and everything the first time it does.

### Why increments and not absolute poses

Orientation has no continuous global chart: quaternions double-cover, rotation vectors wrap at pi, Euler angles gimbal. Any of those puts step discontinuities into the regression target. Consecutive-frame deltas sit near identity, so they are small, smooth and branch-free. The observation keeps **absolute** pose for grounding and uses the 6D representation (first two columns of R) for the same reason: an absolute quaternion *input* carries the sign flip into the network.

### The swivel

Recorded in the observation because IK needs it to recover joints, and carried as an action increment so the runtime can choose. The first plan was to hold it — at r2c0L0 and `r0c1L0` its cross-episode scatter (9–12 deg) exceeds its within-episode travel (5–11 deg), so predicting it looked like learning noise, and `h1-auto-grip --swivel-cell` already snaps and holds the branch at the pick.

But episodes differ: ep0 and ep5 hold the swivel to 1.1 and 1.7 deg while ep20 sweeps 19.5, and holding ep20 at its own median moves a joint 37 deg. The **plate stays exact** — it is a hard target — only the elbow goes elsewhere. Two channels are cheap, so the choice moved to deployment: `--swivel-mode {track,hold,free}`. The drift from the armed branch is reported either way, so  "holding cost nothing" stays a number rather than a claim.

Holding is safe in reachability terms: across 12 episodes, solving the demonstrated plate poses with the swivel held at the episode median gives **0.000 mm** of plate residual and achieves the held swivel exactly.

## Status

Done and verified:

- `lerobot/tools/ee_pose_dataset.py` — three stages, because FK/IK need placo   (ROS2) and the dataset needs lerobot (conda).
- `..._train-eepose` (73 episodes) and `..._holdout-eepose` (14) built.
- Round trip: integrating the increments from the seed reproduces the   demonstrated command pose over 235–581 steps with **0.0000 mm** of drift.
- `policy_action.PoseTargets` — the integrator, with injected solvers so it
  tests without placo, as `GripLockMask` does. 11 tests.
- `policy_driver` — `--action-space`, `--swivel-mode`, `--ik-dt`; seeds on   `cmd=arm` from the measured joints; reports `action_space` and `poses` in status so the eval can refuse a mismatch.
- Suite: 662 passed, 16 skipped (ROS2) plus 13 (conda, the plugin half).
- IK cost: **0.05 ms per solve**, 0.09 ms for both arms, against a 50 ms budget.  The earlier worry about placo in the control loop was unfounded — the offline scipy `least_squares` is ~1000x slower and is not what runs.

- `lowstate_bridge.plate_block` publishes `right_rot` / `left_rot`, the first   two columns of each plate's rotation. Free: `gauge.update` already computed  both transforms for the gap and the misalignment.
- `robot_topstar_h1_real` — `with_ee_pose`, which drops the fourteen arm joints   AND the plate origins (`plate_right.x` *was* the right EE position) and   carries `EE_POSE_KEYS` + swivel instead. A bridge too old to send rotations  gives NaN, not identity.
- `eval_cycle` — knows `ee_pose_increments`, sets the robot's layout from the
  SPEC, forwards the increments untouched, and refuses to start when the driver
  disagrees about what an action is.

The contract between the bridge and the robot spans two environments — the bridge needs `placo`, the plugin needs `lerobot`, and no process has both. So each half asserts the same written column order in its own environment (`test_grip_math.py` and `test_lerobot_plugin.py`), and a transposed mapping fails exactly one of them. That is the reason `ee_pose_from_plate` exists as a seam at all: a transposed rotation still looks like a rotation, and nothing downstream would catch it.

Not done:

- Training in progress (`act_h1_flushoracle_eepose`, 100k steps).
- No offline gate yet on the new checkpoint, and no robot run.
- Nothing is pushed to the controller yet.

## How to use it

Build the datasets (train and holdout separately):

```bash
S=/tmp/eepose
python tools/ee_pose_dataset.py dump  --repo-id <repo> --npz $S/joints.npz
~/h1_palletize/deploy/ros2-python tools/ee_pose_dataset.py fk \
      --npz $S/joints.npz --out $S/poses.npz
python tools/ee_pose_dataset.py build --repo-id <repo> --npz $S/poses.npz --force
```

`build` stamps `meta/info.json` with `action_space: "ee_pose_increments"`, which
is what the runtime interlock keys on.

Train from scratch — never `--policy.path`, which only fills `input_features`
when they are empty:

```bash
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=<repo>-eepose \
  --policy.type=act --policy.chunk_size=20 --policy.n_action_steps=20 \
  --policy.vision_backbone=resnet18 --policy.optimizer_lr=1e-05 \
  --policy.device=cuda --policy.push_to_hub=false \
  --batch_size=8 --steps=100000 --save_freq=20000 \
  --output_dir=outputs/train/<name> --job_name=<name>
```

Run the driver in the ROS2 environment, which is the one with placo:

```bash
~/h1_palletize/deploy/ros2-python -m h1_palletize.cli.policy_driver \
    --no-base-action --action-space ee_pose_increments --swivel-mode track
```

It will log `actions: EE-POSE INCREMENTS, base frame, swivel track`. If it does
not say that, stop: a pose action read as joints commands every arm joint to
roughly zero.

### Gotcha: near-constant action channels

The plate never rotates, so `r_ee.drx` and `r_ee.drz` come out as constant columns (std ~1e-6). LeRobot normalizes with `std + 1e-8`, so per-radian gradient is `1/std` and those channels would have carried ~1e8 times the loss of a real one. `build` floors the action std at `--min-action-std`, default 1e-4 — deliberately **below** the translation channels' own std (3.8e-4 to 7.2e-4) so the floor never de-weights a channel that carries signal. A first attempt at 1e-3 caught all twelve pose channels and would have quietly halved the weight on the translations.

The observation side needs no floor: normalizing an input to unit variance is what normalization is for, and the values sit ~1600x above float32 precision. The loss-side amplification has no analogue for inputs.

## What this is expected to fix

Per-cell conditioning, which is what separated the good cells from the bad ones. Expressing the action in joint space makes the work at r0c0L0 **4.34 rad** of right-arm travel where `r1c1L0` needs **1.04** — four times as much for a larger plate displacement — and the policy delivers about 43% of the demonstrated travel per unit time whatever the cell. That is why `r1c1L0` and `r2c1L0` succeeded and r0c0L0 stalled.

| cell | plate disp | right-arm rad | mm per rad | result on 2026-09-01 |
|---|---|---|---|---|
| `r1c1L0` | 97 mm | 1.04 | 93.2 | very good |
| `r2c1L0` | 133 mm | 2.92 | 45.5 | very good |
| `r0c0L0` | 155 mm | 4.34 | 35.7 | stalled, then slow |
| `r0c1L0` | 142 mm | 5.11 | 27.7 | 60% push |
| `r2c0L0` | 148 mm | 5.47 | 27.0 | 87% push |

In task space the work is 97–155 mm everywhere. The per-cell penalty comes entirely from the action space, and this removes it at the source.

## What it does not fix

The **post-task drift**. The demonstrations end about 2.3 s after the deepest push; the robot runs kept going 8+ seconds past it, and past the last demonstrated frame the policy has no data. It predicts the tail well when it has one — teacher-forced, the tail increment rate matches the demonstration to 0.01662 against 0.01668 rad/step — so this is extrapolation, not a modelling error. An exit gate is still required, and it is per cell: **the push direction itself flips sign** between the corner cells (`r2c0`, `r0c1`: −y) and the flush cells (+y), which is presumably what defeated the earlier attempt at a single global condition.

## Superseded: joint increments

`lerobot/tools/delta_action_dataset.py --mode increment` remains, and `act_h1_flushoracle_incr` is the checkpoint. It works — 87% push depth at r2c0L0 against the absolute policy's 12% — and it is what the runs above used. Its `--mode lead` variant (`a[t] - s[t]`, anchored on the MEASUREMENT) does not work and should not be run: it traps a joint below the servo deadband (2026-08-31 run 1, servo alarm) and slides under the driver's soft-limit clamp, because an anchored target never leaves the measurement's neighborhood. See `project-h1-delta-actions` for the full account.



