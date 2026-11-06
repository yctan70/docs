
# H2 kinematic motion generator (SONIC-style)

Status: P0 and P1 done 2026-11-05 (harness and motion-matching baseline; dataset); P2 done with one gate item open (foot skate), P3 done (closed loop in sim and on the vendor stack). P4 not needed. Next: P5 (hardware). Branch `h2-port`.

## What SONIC's planner does, and what we match

SONIC's planner is a learned model. Every 0.1 s it takes 4 context frames from the reference that is playing now,
plus a command: mode, speed, movement direction, facing, height. It returns about 40 frames at 30 Hz, resampled to
50 Hz and cross-faded into the playing reference over 8 frames. The tracker never sees the planner, only the
reference. Our tracker (`h2_sit_hw`) already works that way. The G1 version of the loop is replayed offline in
`src/realmimic/data/planner_rollouts.py`; the H2 harness copies its structure.

## Decisions

1. Train an H2 generator on our own data; do not run SONIC's G1 planner online. It is a 774 MB G1-specific model (every
   0.1 s we would retarget H2 to G1 and back) and the weights are not downloaded here. Its offline G1 rollouts could add
   squat, kneel and styled modes later, after a G1 to H2 retarget and a check of the NVIDIA Open Model License for
   derived data.
2. Build the harness first, with motion matching as the first generator. The command runner is already a generator; it
   becomes the baseline the learned model has to beat, and the fallback.
3. v1 modes: stand, slow walk, walk, turn in place, back and sideways walks, sit and get-up transitions. Dances are
   discrete move ids (the curated clips). Styles come from BONES-SEED's labels (injured leg, injured torso, hurry).
   No running or crawling in v1.

## Phases and gates

- **P0, harness.** A `PlanSource` interface and the SONIC loop: 10 Hz re-plan, context at the current frame plus 2,
  8-frame cross-fade, hold the last frame. A command script format, and a safety shaper (rated joint speeds, joint
  limits, the firmware ankle clamp, roll and pitch under 1.0 rad). Runs in the runner and offline, like
  `planner_rollouts`. Gate: reproduces the command runner's sim results with zero stops.
- **P1, data.** Windows of 4 context frames plus about 44 future frames from the 15 h library plus dance and floor
  clips. Commands are labeled in hindsight from each clip's own root trajectory; mirror and 0.6 to 1.0x time-stretch
  augmentation; splits by actor. The arm branch must be canonicalized first: about 20 % of clip end frames have the
  left arm on the other solution of a straight arm (shoulder yaw about pi apart, elbow negated), and a regression
  model would average the two into a half-twisted arm. Windows whose elbow cannot be flipped within its range
  (+1.36 rad on the neutral side) are dropped or flagged.
- **P2, model.** A small conditional transformer or MLP with a latent variable (SONIC's sampling seed), exported to
  ONNX, a few ms on CPU. Losses on joints, root trajectory, FK keypoints, foot contacts. Gate: beat motion matching on
  speed-limit excess, foot skate and command-tracking error.
- **P3, closed loop.** Random command scripts of about 60 s in the realmimic sim under the five conditions of the clip
  sweeps (nominal, obs lag, command lag, no-load, noise). Gate: zero stops, tilt under 0.4 rad, rated-speed excess no
  worse than the clips'. Then a few scripts through the vendor stack.
- **P4, tracker fine-tune, only if P3 shows drift.** Mix generator rollouts (10 to 20 %) into the tracker's training
  data, as `generate_planner_rollouts` and `build_training_mix` do for SONIC's.
- **P5, hardware.** `--passive` on the gantry, then the band, then free. Speed capped at 0.3 m/s at first, raised in
  steps.

## Risks

- Generated references are off the tracker's training distribution; the P3 closed-loop gate matters most.
- Floor modes need the ball hands.
- The H2 cannot stand on PD alone, so the runner starts the policy at once.
- The neutral left arm was on the minority branch until `tools/remap_h2_neutral_arm.py`; rerun it after any export that
  rewrites `neutral_motions.npz`.

## P0 (done 2026-11-05)

Everything is numpy-only and lives in single files under `deploy/h2/`, because the runners are copied one file at a time
to `~/topstar_h2/python/` (refresh by hand: `h2_generator.py`, `h2_mimic_gen_runner.py` and `h2_mimic_cmd_runner.py`,
which gained `aligned_segment`).

- `deploy/h2/h2_generator.py`
  - `GenCommand` (mode `idle` or `walk`, vx, vy, wz), `parse_script` (`"0:idle 2:walk,vx=0.5 8:walk,wz=0.6 12:idle"`),
    `RateLimiter`.
  - `GeneratorLoop`: SONIC's loop. A re-plan decision every 0.1 s (always on a mode change; for a moving mode also on a
    velocity change or a 1.0 s timer; `idle` on a 1.5 s timer, our departure), context of 4 frames at 30 Hz ending 2
    rows ahead of the playing row, plan pickup one tick later, 8-row linear cross-fade, hold the last row, timeline
    rebased at each stitch.
  - `SafetyShaper`: joint positions clipped to the hard range, joint speed limited to `--speed-frac` (1.5) x rated from
    the row before the changed stretch, pelvis tilt capped at `--max-rp` (1.0 rad), velocities of changed rows recomputed.
    The firmware ankle clamps are off by default: they belong to the set-points, the tracker was trained with them on
    its targets, and the ankle roll is beyond them in 42 % of the library's frames.
  - `PlanSource` protocol (`plan(Context, GenCommand) -> Plan`), `resample_30_to_50` for a 30 Hz generator, and
    `MotionMatchSource`, the baseline: the command runner's motion matching, returning the upcoming rows unchanged
    (extended by the clip's next frames) when it keeps the playing clip, so a re-plan that changes nothing adds nothing.
  - `rollout`: the loop without a robot.
- `deploy/h2/h2_mimic_gen_runner.py`: the runner (same backends, take-over, stops and parser as the command runner, plus
  the generator options).
- `tools/h2_generator_rollout.py`: offline statistics of a script; `tools/h2_generator_sweep.py`: generator runner against
  command runner in simulation.
- `tests/realmimic/test_h2_generator.py`: 22 tests on synthetic clips.

Gate (sim, `exports/h2_sit_hw`, 4 command profiles x 5 conditions = 20 runs per runner, 30 s each):

| runner | stops | velocity error vx / vy / wz | joint speed over rated | peak | worst roll / pitch |
|---|---|---|---|---|---|
| command runner | 0 | 0.17 / 0.09 / 0.16 | 7.9 % of ticks | 2.57x | 0.31 rad |
| generator runner | 0 | 0.20 / 0.10 / 0.18 | 10.0 % of ticks | 1.84x | 0.26 rad |

Zero stops and a similar velocity error: the gate is met. The generator spends 2 points more of the ticks over the rated
joint speed but with a lower peak. Ablations (same sweep): without the shaper 7.7 % at a peak of 2.74x, so the shaper
trades the peak for more ticks slightly over rated; `--fade 1` 10.4 %; a re-plan timer of 2.0 s 5.7 %. A plan holds 2 s,
so a 2 s timer leaves no margin and is not the default. To look at in P2/P3, with a learned source, not to tune here.

Not done in P0: the `shm` backend (vendor stack) and hardware; facing and height commands; named moves (dances) as
modes; sit and get-up as modes. The plan runner still covers those.

## P1 (done 2026-11-05)

Training-side code is the package `src/realmimic/gen/` (the runtime stays single-file in `deploy/h2/`):

- `rows.py`: the 71-column rows, vectorized quaternions, `resample` (50 to 30 Hz; also the time-stretch, velocities scaled),
  the left-right mirror (`mirror_map`, `mirror_rows`), and the model's features: `encode` / `decode`, 38 per frame (root xy
  in the heading frame of the anchor, z, 6D root rotation in that frame, 29 joints), decoded back to rows with
  central-difference velocities.
- `canon.py`: `ArmBranch`. The shoulder yaw moves by pi and the elbow, wrist pitch and wrist roll are negated for the minority
  branch (left yaw below -1, right above +1), so every arm link up to the hand stays exactly where it is (tested by forward
  kinematics); the hand turns half a turn about its own axis. A flip beyond a joint range is left alone. The branch is
  unimodal afterwards (left yaw peaks at +0.5 to +1.0).
- `windows.py`: `ClipStore`, `WindowSampler`. A window is 4 context frames and 44 future frames at 30 Hz; the label is read off
  the future: average heading-frame velocity (vx, vy, wz), mean pelvis height, a mode (`idle` / `walk` by how the window
  moves; `sit`, `seated`, `get_up`, `dance` by the clip's name), the dance move id, the style (BONES-SEED label), with
  mirror and speed (0.6 to 1.0x) augmentation drawn per window. Windows around an arm-branch jump are left out; a clip that
  ends still is padded by holding its last frame (a plan must be able to stop).
- `tools/build_h2_generator_data.py` builds `data/gen/h2_gen_v0/` (gitignored: `clips.npz`, `index.csv`, `meta.json`,
  `stats.json`); `tools/h2_generator_data_stats.py` prints what is in it. 22 tests in `tests/realmimic/test_gen_data.py`.

What is in it: 7,782 clips, 15.4 h (7,610 training-mix clips plus the 172 retargeted dances that passed the speed filter),
about 381k / 52k / 46k windows at a 0.1 s stride (12.2 / 1.7 / 1.5 h), 3,600 windows/s with augmentation on one core. Modes in
the training windows: idle 28 %, walk 59 %, sit 3 %, seated 7 %, get_up 2 %, dance 2 %. Styles: neutral 79 %, injured
torso 12 %, injured leg 8 %, hurry 0.6 %, old 0.1 % (the last two are too rare to learn).

Decisions made on the way:

- Split by actor (10 % val, 10 % test by a hash of the actor id) for the standing and floor clips. The dances come from 17
  performers repeating the same moves, so an actor split left 19 of the 44 moves without a training clip; they are split by clip
  within the move instead, every move has a training clip.
- 35 % of the left-arm frames and under 1 % of the right-arm frames were on the minority branch; 2 % of the frames are bent
  arms that cannot flip (left alone); 424 clips (5 %) cross the threshold continuously and lose the windows around the jump.
- The mirror was wrong for the arms: `mirror_map` negated roll and yaw only, but the two elbow axes point opposite ways. Every
  mirrored clip of the plan runner's catalog (`right`, `arc_right`, ...) had both elbows bending backwards. Fixed and tested by
  forward kinematics over every leg and arm body; `tools/fix_h2_mirrored_elbows.py` patched the existing bundle.

Left for P2: foot contacts (computed from forward kinematics when the loss needs them), a held-out evaluation set of command
scripts, the facing and height commands, SONIC's other styled modes.

## P2 (done 2026-11-05; the foot-skate gate item is open)

- `src/realmimic/gen/model.py`: a conditional VAE. Input: the 4 context frames (the last one and the three earlier ones as
  differences to it, normalized) and the condition (command vx / vy / wz, pelvis height, one-hots of mode, style and dance
  move). Output: the 44 future frames as an offset from the last context frame, normalized per feature. The latent (32, prior
  N(0, I)) is SONIC's sampling seed. 12.4 M parameters (MLP, 4 residual blocks of width 1024). `Exporter` puts normalization
  and decoding into one ONNX graph (raw context, condition, z in; raw future out).
- `src/realmimic/gen/train.py`, `tools/train_h2_generator.py`: L1 on the normalized offsets (xy x2, the first 4 frames x3), a
  velocity term, KL with a ramp and free bits; windows drawn with mirror (neutral style only) and speed (1, 1, 0.85, 0.7)
  augmentation. 40 epochs take 11 min on the RTX 5090 laptop. `tools/export_h2_generator.py` writes
  `<bundle>/generator/{generator.onnx, deploy.json}`; `deploy/h2/h2_gen_model.py` is the robot side (`LearnedSource`: numpy
  copies of encode / decode, pinned to the training side by a test; the plan is the 44 frames resampled to 73 rows at 50 Hz).
- `deploy/h2/h2_mimic_gen_runner.py --source learned [--model-dir D] [--z-scale S]` runs it; `tools/h2_generator_eval.py`
  compares sources on the reference the loop produces; `tools/h2_generator_sweep.py --gen-arg=--source=learned` runs the
  tracker in the loop. `src/realmimic/gen/fk.py` is a differentiable ankle forward kinematics (checked against MuJoCo to 0.1 mm).

Held-out actors, true command, z = 0 (teacher-forced): joint error 0.032 / 0.049 / 0.058 rad at 0.3 / 0.7 / 1.4 s (holding the
last frame: 0.075 / 0.101 / 0.083), root position error 1.3 / 2.7 / 1.1 cm (12 / 26 / 51 cm).

Gate, on the reference of the four command profiles of the sweep (no robot), z = 0:

| source | vel err vx / vy / wz | over rated | peak | foot skate | below floor |
|---|---|---|---|---|---|
| motion matching | 0.18 / 0.09 / 0.17 | 7.7 % | 1.53x | 0.104 m/s | 0 % |
| learned (v0) | 0.07 / 0.04 / 0.06 | 0.0 % | 0.84x | 0.152 m/s | 0 % |
| the library's own clips | | | | 0.109 m/s | 0 % |

Command tracking and joint-speed excess are far better than motion matching's. **Foot skate is not: 0.152 against 0.104** (0.130
with z = 0.5). It is the plans themselves (0.155 inside a plan, 0.152 after stitching), not the cross-fades. The gait is not
averaged away (knee range 1.05 rad against 1.07 in the library, hip pitch 0.75 against 0.82, the same 0.67 Hz).

Tracker in the loop (sim, `exports/h2_sit_hw`, 4 profiles x 5 conditions = 20 runs, 30 s):

| runner | stops | vel err vx / vy / wz | joints over rated | peak | worst roll / pitch |
|---|---|---|---|---|---|
| command runner | 0 | 0.17 / 0.09 / 0.16 | 7.9 % | 2.57x | 0.31 rad |
| generator, motion matching | 0 | 0.20 / 0.10 / 0.18 | 10.0 % | 1.84x | 0.26 rad |
| generator, learned v0 | 0 | 0.14 / 0.08 / 0.11 | 0.2 % | 1.71x | 0.33 rad |

So the tracker follows the learned references better, and its joints are over the rated speed in 0.2 % of the ticks instead
of 8 to 10 %, even though the reference's feet slide more.

Foot terms in the loss (`--foot-pos / --foot-skate / --foot-pen`, off by default) did not help:

| run | foot terms | teacher-forced skate (target 0.074) | root error at 1.4 s | closed-loop skate | sim vel err vx |
|---|---|---|---|---|---|
| v0 | none | | 1.1 cm | 0.152 | 0.14 |
| v1 | pos 5, skate 1, penetration 5 | 0.094 | 8.7 cm | 0.180 | 0.31 |
| v2 | pos 2, skate 10, penetration 5 | 0.008 | 51 cm (= not moving) | 0.015 | 0.30 |

A skate penalty on the contacts of the target rewards standing still: v2 collapsed to that, and v1 paid for a small gain
inside a plan with worse long-horizon fit. Ideas that do not have that failure, for later: penalize the foot velocity relative to
the target's foot velocity in contact (not its absolute value), a foot-lock pass in the loop's shaper that pins a planted
ankle by adjusting the plan's root, or training on the model's own rollouts.

Not done in P2: the learned source on the vendor stack (P3); facing and height commands; sit, get-up and dance as modes of the
learned runner (the model was trained with them; the runner's `GenCommand` only has idle and walk); the rare styles.

## P3 (done 2026-11-05)

Closed loop: 24 random 60 s command scripts (`tools/h2_generator_sweep.py --random 24 --seed 100 --seconds 60`: idle, forward,
backward, sideways, turning in place, arcs and diagonals in segments of 2 to 6 s, with abrupt changes) x the five conditions
of the clip sweeps (nominal, obs lag, command lag, no-load speed limit, noise) = 120 runs per runner, tracker in the loop.

| runner | stops | vel err vx / vy / wz | joints over rated | peak | worst roll / pitch |
|---|---|---|---|---|---|
| command runner (motion matching, splicing) | 0 | 0.15 / 0.09 / 0.14 | 6.4 % | 3.07x | 0.33 rad |
| generator runner, motion matching | 0 | 0.15 / 0.10 / 0.15 | 7.6 % | 2.77x | 0.30 rad |
| generator runner, learned (v0) | 0 | 0.12 / 0.08 / 0.10 | 0.2 % | 2.85x | 0.36 rad |
| generator runner, learned, from the vendor stand-up pose | 0 | 0.10 / 0.07 / 0.09 | 0.1 % | 1.44x | 0.27 rad |

Gate: zero stops, worst roll / pitch under 0.4 rad, rated-speed excess no worse than the clips' (6.4 to 7.6 %): met.

The vendor stack found what the simulator did not. Two random 60 s scripts (seeds 101 and 102) through the vendor FSM and
bridge with the topstar MuJoCo scene (`exports/h2_sit_hw/vendor_scene.xml`) both ended at 4.4 and 4.8 s, pelvis roll / pitch
past 1.2 rad ("FSM left RL_INFERENCE"). Cause: the FSM's stand-up pose (arms straight down, hip yaw 0, head pitched -0.26 rad;
the head joints are 0.00 in every library frame) is nothing the learned model has seen: as a context its largest z-score
against the training data is 259. The simulator backend starts from a pose near the library's, so the sweep never showed it;
motion matching eases into a library pose and was not affected. The model's first plans moved the reference by 1.3 to 1.75
rad and the robot fell within three seconds.

Fix, in `deploy/h2/h2_mimic_gen_runner.py`: for the learned source the measured pose is held, eased into the bundle's neutral
pose (`--ramp`, 2 s), held again (`--settle`, 0.5 s), and only then does the generator plan, so its first context is in
the distribution. `LearnedSource.ood_max` (needs `ctx_mean` / `ctx_std` in `deploy.json`, written by
`tools/export_h2_generator.py`) records the largest context z-score; the runner prints it at the end (2.4 with the fix).
`--sim-start-pose <json>` starts the simulator backend in a recorded pose; the bundle's `vendor_stand_pose.json` is the
vendor stack's. With it the failure reproduces in the simulator (the learned source falls at 4.8 s, motion matching survives)
and is a regression test (`tests/realmimic/test_h2_gen_runner.py`, skipped without the bundle). After the fix both vendor
sessions run the full 61 s: worst roll / pitch 0.19 and 0.23 rad, no tick with a joint over its rated speed (peak 0.83x and
0.80x), 111 and 121 plans; the 120-run sweep from the vendor pose is the last table row.

P4 (tracker fine-tune on generator rollouts) is not needed: no run drifted off its reference far enough to stop, and
rated-speed excess fell instead of rising.

Not done: hardware (P5); the learned runner only knows idle and walk (sit, get-up and dance exist in the model and the
plan runner but are not modes of `GenCommand` yet); facing and height commands; foot skate of the reference (0.152 m/s
against motion matching's 0.104).

## Slow walking and slow turning (2026-11-05)

Goal from the first hardware-oriented use: adjust the speed continuously and walk or turn very slowly.

`tools/h2_generator_speed_test.py` holds one command for 20 s through the generator runner in simulation (tracker in the loop)
and reports the speed the robot achieved. The first learned model (v0) was good from 0.4 m/s up and for turning, but could not
walk slowly: below 0.3 m/s forward and below 0.4 sideways the reference glided (knee range 0.07 to 0.14 rad, no steps) and the
robot stayed where it was. Cause: the dataset has no stepping gait slower than about 0.3 m/s (the steady walks are 0.7 to 1.3;
BONES-SEED's "slow" and "very slow" walks are 0.3 to 0.6 m/s themselves). Three changes:

1. More data: the slow-walk families that were not in the training mix (steady loops to the side, backward and in arcs,
   0.29 to 0.36 m/s with real stepping): 56 clips extracted from the archive, 30 pass the speed filter (`data/gen/h2_gen_v1`,
   `data/raw/bones_slow`, `data/npz/h2_slow`; the builder picks them up when `data/npz/h2_slow` exists).
2. Training: `--slow-boost 8` (slow clips are 2 % of the windows, 15 % of the draws), speed augmentation down to 0.4x for them and
   0.55x for other walks (`train.py`: `SLOW_SPEEDS`, `WALK_SPEEDS`). Model `s1`; validation errors are the same as v0's.
3. Runtime slow motion (`LearnedSource(slow_floor=0.3)`, runner `--slow-floor`, on by default): a translation slower than the
   floor is asked of the model at the floor speed in that direction, and the plan is played back slower by the ratio; the model's
   context is resampled from the playing reference at the model's own tempo (`Context.history`), without which the model saw a
   shuffling reference and sped it up again. A turn in place is not slowed.

Constant commands, robot speed achieved (m/s or rad/s; the reference's in brackets; v0 -> s1 with slow motion):

| command | forward | backward | sideways | turn |
|---|---|---|---|---|
| 0.1 | 0.00 -> 0.06 (0.11) | 0.00 -> -0.04 | 0.00 -> 0.04 | 0.06 -> 0.07 |
| 0.2 | 0.01 -> 0.13 (0.20) | -0.05 -> -0.12 | 0.01 -> 0.12 | 0.13 -> 0.18 |
| 0.3 | 0.15 -> 0.22 (0.30) | | 0.02 -> 0.21 | 0.24 -> 0.31 |
| 0.4 | 0.36 -> 0.38 | -0.31 -> -0.34 | | |
| 0.6 / 0.5 | 0.58 -> 0.57 | | 0.47 -> 0.41 | 0.50 -> 0.54 |
| 0.8 | 0.79 -> 0.77 | | | 0.83 -> 0.85 |

So the reference follows the command at every speed, with real steps (knee range 0.4 to 0.66 rad at 0.1 to 0.2 m/s), and the
robot realizes about 60 to 65 % of a command below 0.4 m/s (monotonic in the command; a calibration could scale it). Turning
works down to 0.1 rad/s (0.07 achieved).

Checks of the new model with slow motion, from the vendor stack's stand-up pose: 120 random scripts at normal speeds, 0 stops,
worst roll / pitch 0.35 rad, 0.2 % of ticks over the rated joint speed, forward speed error 0.08 (v0: 0.12); 120 random scripts at
0.05 to 0.3, 0 stops, worst roll / pitch 0.25 rad, 0.2 %; one vendor-stack session of a slow script, the full 61 s, worst roll /
pitch 0.21 rad, no tick over rated speed. The shaper clips the ankle roll and pitch a little in slow motion (0.03 to 0.2 rad past
the hard range in 1 to 8 % of the rows): the model is slightly outside its data there.

Direction changes (`tools/h2_generator_transitions.py`, six sudden changes, tracker in the loop): the robot covers half of a change
in about 1.1 s with every runner, and settles in about 2.1 s (learned), 2.2 s (command runner) or 2.8 s (generator with motion
matching). A higher command acceleration (`--cmd-accel` 2 or 4) makes the command runner's half-way 0.7 s but not the learned
generator's (1.08, 1.03 s): the model's own response is the limit, and the reference is four times smoother (peak joint
acceleration p99 11 rad/s2 against 41). Faster switching would need a shorter command horizon in training (the label is the average
velocity over 1.47 s); not tried.

### Shorter command horizon (tried, not adopted)

The command label of a training window is the average velocity over the 44 future frames (1.47 s). `--horizon 20` (0.67 s) was
tried to make the model answer a changed command faster (run `h20`, otherwise as `s1`). Validation fit is the same
(joint L1 0.031/0.046/0.062 rad). In the closed-loop transition test the mean half-way time is 1.11 s against 1.15 s for `s1`,
but 2 of 6 transitions never reached half way and 3 of 6 never settled (`s1`: 6 of 6 reached and settled). No gain, so `s1`
stays deployed. The half-way time of about 1.1 s is therefore not the label horizon: the likely limits are the 8-row fade,
the replan and pickup delay and the tracker's own response. The flag stays (`train_h2_generator.py --horizon`).

