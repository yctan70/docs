# H1 flush-place: where the policy stands, and what it cannot do

Status doc written 2026-08-26, at the end of a long build-and-test cycle on the
release/retract automation. Its purpose is to be the starting context for a
conversation about **replacing or retraining the policy**, so it leads with the
failures and the evidence, not with the tooling.

---

## 1. The task, in one paragraph

A Topstar H1 humanoid stands at a fixed park pose beside an 880 x 810 x 110 mm
pallet and places 400 x 250 x 200 mm boxes into a 2 x 3 grid, two layers deep.
The box is already in the hands when the policy starts (`--held_box_start`): the
two end-effector plates pinch it, and while `grip.lock >= 0.5` the left arm is
slaved to the right so the pair moves as one rigid clamp. The policy's job is the
**flush place** -- drive the box down and sideways until it is flush against the
already-placed neighbours, then drop `grip.lock`, which is the signal that the
box has been let go. After that a scripted sequence takes over (left arm up,
torso up, right arm push, stand).

Cells are named `rRcCLL`: rows 0/1/2 across the 810 mm side (3 x 250 mm, 60 mm
slack), columns 0/1 across the 880 mm side (2 x 400 mm, 80 mm slack), layer 0/1.
Placement order is r1c0, r1c1, r2c1, r0c0, r2c0, r0c1. `CORNERS = {r2c0, r0c1}`
are special: there is no neighbour on two sides, so the release happens early
(19-26% into the episode) and the push does the alignment.

Policy: ACT, chunk 20, resnet18, 22-dim action (18 joints + 3 mobile_base +
grip.lock), 30-dim state, 20 fps. Trained on 89 teleop demonstrations of
`topstar/h1-flush-place-lock` recorded 2026-08-14 -> 08-18.

---

## 2. What now works end to end

- **`--auto-release`**: after the policy run, `h1-eval-cycle` shells out to
  `h1-auto-grip --carry-only --exit-when-done` to open the plates, retract, and
  (with `--stand`) unbend the torso. Chained on the four non-corner cells by
  default; corners keep the driver's own stand because their release is
  mid-episode.
- **Geometric release detection** (`--release-on geometry`): the driver watches
  the box centre (midpoint of the two plate origins, in `base_link`) and latches
  a release when it comes within a per-cell horizontal limit of the cell's
  demonstrated seat. This replaced pressing SPACE by hand.
- **`--stop-on-release`** ends the episode at the latch, so the retract starts
  immediately. Refused at startup if the driver is on `--release-on policy`.
- 551 tests pass on the controller (`top@192.168.31.3`, `ros2-humble` container).

Runbook: `docs/EVAL.md`.

---

## 3. The gate, and why its calibration is now suspect

`release_ready()` is **horizontal only**. The vertical (descent) gate was
measured, found not to discriminate, and retired -- the table survives as
`DESCENT_GATE_M` with its provenance. The user's read was right: "the x-y errors
are real, but there is no problem vertically."

The horizontal limit per cell is `CELL_RELEASE_XY_M`, fitted as *the worst
demonstration's distance to that cell's median seat*, rounded up:

```
r0c0L0 0.002   r0c0L1 0.004   r0c1L0 0.017   r0c1L1 0.009
r1c0L1 0.015   r1c1L0 0.019   r1c1L1 0.006   r2c0L0 0.006
r2c0L1 0.015   r2c1L0 0.011   r2c1L1 0.006
```

times `RELEASE_XY_SCALE = 0.7` (overridable with `--release-xy-scale`).

`CELL_SEAT` is the median box centre per cell **in `base_link` at the frame
`grip.lock` falls**. It never references the pallet. That is deliberate -- it
makes the gate independent of pallet calibration -- but it also means the gate
inherits every systematic error the demonstrations had.

### The systematic error we found

Two cells needed hand corrections after tape measurement
(`CELL_SEAT_CORRECTION_M`, base_link dx, dy, dz):

```
r2c1L0  (-0.017, +0.015, 0.0)
r0c0L0  (+0.0118, -0.0175, 0.0)
```

**These are the same physical ~15 mm shift toward pallet +x, seen from opposite
serve sides.** r2c1's gap to the +x 810 edge had shrunk 40 -> 25-30 mm; r0c0's
gap to the -x edge had grown 40 -> 55-60 mm. That is the signature of a pallet
whose calibrated pose is off by ~15 mm along the 880 axis.
`~/hrcb/pallet_pose.json` is dated **2026-08-07** -- before the demonstrations.

**Recommendation for the next session: recalibrate the pallet pose first.** It
would very likely zero both correction rows and shift every cell's seat at once,
and it makes any retraining sit on a correct frame. Doing more per-cell patching
before that is fitting noise.

(One loose end: a ~60 mm discrepancy remains in my pallet-frame arithmetic on the
810 axis. The chassis-pose check validates against the AGV to 3 mm; the box
number does not. Nothing built depends on it -- the gate is entirely `base_link`
-- but it means my pallet-frame numbers should not be trusted until it is found.)

---

## 4. The failures the current policy cannot get past

These are the reasons to consider a new policy.

### 4a. r0c1L1 -- converges, then refuses to let go (4 consecutive failures)

The corner cell r0c1 at layer 1 used to succeed and now does not, across two
checkpoints (`100000_model` and `100000_model_basefix`) and across
`--base-obs demo`. The signature:

- the box **does** converge to the target -- the policy is not lost;
- `grip.lock` stays pinned at **0.99** and never drops;
- the left plate rises 4-6 mm where a success shows 150-180 mm;
- **every state channel is within the success band.** Only the camera image
  differs.

Forcing the release does not help: the user tried it, and "there were clear gaps
between boxes, and the policy got stuck -- it would not go on to the next phase
for left arm raise and right arm push." The post-release phase is not a separate
scripted branch the policy can be pushed into; it is part of the same learned
sequence, and if the policy has not itself decided to release, driving `grip.lock`
low externally leaves it in a state it has never seen.

**This is a visual-conditioning failure.** Something about the scene at r0c1L1
(neighbours present, lighting, or the layer-1 stack height) has drifted outside
what the demonstrations covered, and the policy's release decision is keyed on it.

### 4b. r2c1L0 and r0c0L0 -- stalls at layer 0

Both symmetric layer-0 cells stalled: the policy did not drive the box close to
the target cell at all. r1c1L0 (the centre cell, and the one with the most
demonstrations) was fine. These are the two cells that also needed seat
corrections -- i.e. the two cells whose demonstrations were recorded against a
mis-calibrated pallet. Plausibly the same root cause as the correction: the
demonstrated seat and the visually-correct seat disagree, and at layer 0 (where
the box is low and the neighbour geometry dominates the image) the policy splits
the difference and stops.

### 4c. The gate cuts off a policy that is still improving

The most instructive run: `r2c1L0_20260825-195344`, 498 steps, released at step
490 (98%, t = 25.2 s), `release_rc=0`, `ok=True`. Against the **corrected** seat:

```
approach +6.2 mm    push -0.7 mm    |xy| 6.2 mm      (raw seat: 18.0 mm)
```

The correction's signs are validated -- the taped 17 mm gap became ~6.2 mm. But
the closing history shows the gate fired on a policy that had not finished:

```
  t        error     closing rate
 15.6 s   17.0 mm     5.14 mm/s
 17.6 s   11.3        2.82
 19.6 s   10.1        0.61
 21.6 s   10.2       -0.05
 23.6 s    8.6        0.79
 25.2 s    6.2       <- released
```

Still closing at ~0.9 mm/s with **25 of the 50 s cap unused**. Extrapolating:
~2 s more to 5 mm, ~3 s to 4 mm, ~5 s to 2 mm. The user's read: "the policy could
still do a flush place if the early release is not triggered."

The deeper point: **for a tape-verified cell the fitted limit stops meaning
anything.** `CELL_RELEASE_XY_M["r2c1L0"] = 11 mm` was fitted as the worst
demonstration's distance to the *demonstrated* seat -- which we now know sat
~22 mm from flush. Once a cell has a measured correction, its limit should be a
**spec** (the gap you will accept), not a fitted statistic, and
`RELEASE_XY_SCALE` should not apply to it, for the same reason it does not apply
to an explicit `--release-xy-m`.

Three shapes were offered and none chosen yet:
- (a) per-cell **spec** limits for corrected cells, ~0.004, exempt from the scale;
- (b) release on **convergence** -- inside a looser bound AND closing rate below
  ~0.2 mm/s;
- (c) raise the cap and keep a tight spec limit.

(a) is the smallest change and is roughly what this run was 3 seconds from doing.
(b) is the one that generalises, and is the only one that does not need a new
number per cell.

### 4d. Handoff pose sensitivity (already mitigated, but revealing)

`head_pitch` and `head_yaw` are not set by the scripted pick/carry -- the head
arrives wherever the previous rollout left it. On 2026-08-20 that handed the
policy a head at +0.11 instead of ~+0.49; it released the grip at frame 90
instead of 250 and the whole post-release sequence fired against a spent phase.
At r1c0L1 a handoff `head_yaw` of +0.0140 against six demonstrations all at
-0.0610 was the single largest normalised deviation in the 30-dim observation.

This is now checked (`head_complaints`, `CELL_HEAD_PITCH`, `DEFAULT_TOL_RAD`),
but it says something about the policy worth carrying forward: **it is extremely
sensitive to initial-condition drift that is invisible in the task geometry.**
A ~0.36 rad head offset -- which changes nothing about where the box is --
rewrote its entire timing.

### 4e. The frozen chassis-pose feed (fixed, but it poisoned data)

`robot_topstar_h1_real.py:340` silently falls back to recording `base_target`
instead of the measured base pose when the AGV feed is unavailable, warning once.
This produced identical `mobile_base` triples across three different cells and
park errors of 1381-3602 mm in recorded data. Any episode recorded through that
fallback has a fabricated base channel. Worth auditing which of the 89
demonstrations are affected before retraining on them.

---

## 5. What a better policy would need to fix

In priority order, as the evidence supports:

1. **Release decision is the weak link, not the reach.** In every failure mode
   the box gets close; what breaks is *when* (or whether) `grip.lock` drops.
   A policy that outputs a continuous "flushness" estimate, or that is trained
   with the release as an explicit terminal condition rather than as a 22nd
   action dimension, would be directly testable against the data we now have.

2. **Visual conditioning is fragile per cell.** r0c1L1 fails on the image alone
   with every proprioceptive channel nominal. More cell coverage, augmentation
   over neighbour-stack appearance/lighting, or a representation that does not
   have to re-learn each cell from scratch.

3. **The demonstrations encode a mis-calibrated pallet.** Recalibrate, then
   either re-record or apply a known rigid correction to the recorded seats
   before training. Otherwise the new policy inherits the same ~15 mm bias.

4. **Initial conditions must be part of the training distribution.** The head
   pose lesson generalises: whatever is not reset between episodes must either be
   reset by the harness or varied in the data.

5. **Symmetric cells should not need per-cell tables at all.** The current design
   has 11 fitted limits, 11 head pitches, and a growing correction table. That is
   a sign the policy is memorising cells rather than learning the flush-place
   behaviour. A cell-conditioned or goal-conditioned formulation (target pose in,
   flush behaviour out) is the structural fix.

---

## 6. Open work items carried over

- Choose and build the release-gate shape for corrected cells (see 4c).
- Verify the `r0c0L0` correction signs with one run + tape. (`r2c1L0` is
  validated: taped 17 mm -> 6.2 mm.)
- Recalibrate `~/hrcb/pallet_pose.json` (dated 2026-08-07, pre-demonstrations).
- Find the ~60 mm pallet-frame arithmetic error on the 810 axis.
- Optional: adjust `_PARK_BACK_OFF_M` to move where the box is put rather than
  where the gate looks.
- Optional: record `auto_release` / `release_cells` in `RolloutResult` so the log
  is self-diagnosing.
- Unresolved: the r0c1L1 stall.

---

## 7. Key files

```
src/h1_palletize/handoff.py                  seats, corrections, release gate, head pose
src/h1_palletize/cli/policy_driver.py        GeometryRelease, GripLockMask, TorsoRaise/Ramp
src/h1_palletize/cli/eval_cycle.py           rollout loop, --auto-release chaining
src/h1_palletize/cli/auto_grip.py            scripted pick/carry/release/retract/unbend
src/h1_palletize/eval_rollout.py             RolloutResult
src/h1_palletize/lerobot_plugin/topstar_h1_real/robot_topstar_h1_real.py:340
                                             the base-pose fallback that poisoned data
docs/EVAL.md                                 runbook
```

Three processes: `h1-lowstate-bridge` + `h1-policy-driver` in the ROS2 container,
`h1-eval-cycle` in the `h1policy` conda env, bridged by `deploy/ros2-python`.

Frames: `base_link` -x is forward, +y is the robot's right. The AGV reports
`seer_base`, which is `base_link` yawed 180 degrees. Pallet `ex` = the 880 mm
direction, `ey` = the 810 mm direction. Grips: 400-end (`grip_gap` 0.400) puts
the plates either side of the push axis; 250-end is 0.260.

