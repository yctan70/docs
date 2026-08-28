# SmolVLA vs ACT on flush-place: the offline bake-off

Written 2026-08-26, as the answer to a question put against
[POLICY_STATUS.md](https://github.com/yctan70/docs/tree/master/H1%20Palletizing%20Policy%20Status.md): *would a pretrained VLA with a stronger
vision backbone fix what the ACT policy cannot do?*

**Short answer: no, and the measurement says something more useful than that.**
Neither architecture generalises the release decision to episodes it has not
seen, and the r0c1L1 failure does not reproduce offline for either of them. The
weak link is not the backbone.

---

## 1. What was compared, and how

Three checkpoints, one held-out set:

| checkpoint | data | steps |
|---|---|---|
| **ACT deployed** (`act_h1_flush_place_lock_r1c0fix_chunk20`) | all 87 episodes | 100k |
| **ACT baseline** (`act_h1_flush_place_split`) | 73-episode train split | 100k |
| **SmolVLA** (`smolvla_h1_flush_place_nl`) | same 73-episode train split | 30k |

The deployed checkpoint is in the table as a **reference, not a baseline**: it
trained on the held-out episodes, so its numbers measure memorisation. The
comparison that decides anything is ACT baseline vs SmolVLA, both trained on the
same 73 episodes and both scored on the 14 they never saw.

`tools/eval_release_offline.py` (in the lerobot checkout) replays each held-out
episode frame by frame through the checkpoint using the *same* three-call load
path and the same `predict_action` that `eval_cycle.py` uses on the controller,
with `n_action_steps` clamped to 10 as the deployment does. It is teacher-forced:
observations come from the demonstration, so it measures the decision at each
state the demonstrator visited, not the drift of a closed loop.

Two setup facts that had to be got right first, both of which silently produce
meaningless numbers:

* **`--dataset.episodes` does not hold anything out.** For a dataset already on
  disk it only filters which files are *downloaded*; `load_hf_dataset` reads all
  of `data/` regardless. `LeRobotDataset(repo, episodes=[10,17])` reports 87
  distinct `episode_index` values and 52307 rows. The splits here are physical,
  built with `dataset_tools.split_dataset`, which also re-aggregates each split's
  normalisation stats.
* **`--policy.path=lerobot/smolvla_base` keeps the SO-100's features** —
  `action (6)` and three `camera1/2/3` at 256x256 — because `make_policy` only
  fills `input_features`/`output_features` when they are empty. It fails the
  visual-consistency check outright here, and without that check would have
  truncated the 22-dim action to 6. The run used a local copy with the features
  re-pointed at this robot; no weights change, since SmolVLA pads state and
  action to 32 either way. That copy also carries processor pipelines, which the
  published `smolvla_base` predates.

The task strings were rewritten from `place r0c1L1` to
`place the box on layer 1, column 1, row 0; this is a corner cell with no
neighbour to push against`, so the language model sees `layer` / `column` / `row`
as tokens shared across cells rather than one opaque blob. ACT ignores the string
entirely, so both models trained on byte-identical data.

## 2. The result

14 held-out episodes. `release_err` is frames between the predicted `grip.lock`
1→0 transition and the demonstrated one; **20 frames = 1.0 s**.

```
run                          never  relerr_med  |relerr|_med  <=10f  l1_joint  lock@gt
ACT deployed (saw holdout)       0        -2.5           3.5  13/14    0.0054    0.223
ACT baseline (73-ep)             0       +19.0          25.5   2/14    0.0152    0.812
SmolVLA 15k  (73-ep)             0        +4.0          31.5   3/14    0.0175    0.582
SmolVLA 30k  (73-ep)             0       +12.5          31.5   1/14    0.0157    0.638
```

Per cell, `release_err` in frames:

```
cell        ACT deployed   ACT baseline    SmolVLA 15k    SmolVLA 30k
r0c0L0               -2.0          +17.5           +8.0          +16.5
r0c0L1              -37.0          +35.0          -52.0          -50.0
r0c1L0               +0.0          +24.0          +41.0          +40.0
r0c1L1               +2.0          +12.0          +37.0          +37.5
r1c0L1               -4.0          +16.0          +26.0          +26.0
r1c1L0               +0.0          +20.0           -3.0          +10.0
r1c1L1               -3.0          +30.0          +20.0          +20.0
r2c0L0               +1.0          -66.0          -25.0          -21.0
r2c0L1               -2.0          -43.0          -74.0          -73.0
r2c1L0               -6.0          +14.0          -51.0          -48.0
r2c1L1               -7.0          +33.0         -147.0         -117.0
```

## 3. What it means

**a. SmolVLA does not beat ACT on the release decision.** 31.5 frames of absolute
median error against 25.5; 1 of 14 episodes within half a second against 2 of 14;
joint L1 a tie at 0.0157 vs 0.0152. On r0c1L1 specifically — the cell section 4a
is about — SmolVLA is *worse*: +37.5 frames against +12.0. Whatever the pretrained
foundation buys, it is not this.

**b. More training will not close it.** SmolVLA's absolute median release error is
**31.5 frames at 15k steps and 31.5 frames at 30k** — flat across the entire
second half of training, while the loss kept falling (0.012 → 0.009). The release
channel had saturated long before the run ended.

**c. The one thing SmolVLA does differently is real but small.** `lock@gt` — the
predicted `grip.lock` at the demonstrated release frame — is 0.638 for SmolVLA
against 0.812 for ACT. The flow-matching head is less collapsed onto "hold" than
ACT's L1 regression, which was the mechanism we expected to matter. It is simply
not enough to change the decision.

**d. Neither architecture generalises the release timing at all.** This is the
finding worth carrying forward. The deployed checkpoint scores 3.5 frames on
episodes it memorised and both honest models score 25-32 frames on episodes they
did not. Section 5.1 said "the release decision is the weak link"; this is that
claim measured, and it is a **data** result, not an architecture one. 73
episodes across 11 cells — six or seven per cell — is not enough for either model
to learn *when* flush has been achieved, as opposed to replaying when it happened
last time.

**e. Section 4a does not reproduce offline, for anything.** Every checkpoint
released on every held-out episode, including both r0c1L1 ones: `never released`
is 0/14 in all four rows. The r0c1L1 stall is therefore not a property of the
policy class. It is a property of an on-robot input distribution that is not in
the demonstrations at all — which is what section 4a suspected, and which no
retrain on this data can fix.

**f. The vision argument, for the record.** SmolVLM2-500M sees 512x512 through
patch-16 with `scale_factor 4` pixel shuffle: 1024 patches → **64 visual tokens**,
about 80x80 px of the original 640x480 frame each. ACT's resnet18 gives 15x20 =
**300** at stride 32. The "better vision network" is semantically stronger and
spatially ~6x coarser, and a flush place is a spatial judgement about a
millimetre seam. That was the a-priori risk; the numbers are consistent with it.

## 4. What to do instead

In the order the evidence supports:

1. **More demonstrations per cell, or a release signal that does not have to be
   learned per cell.** Point (d) is the binding constraint. Section 5.1's own
   suggestion — train the release as an explicit terminal condition, or have the
   policy emit a continuous flushness estimate — is now testable against this
   harness without touching the robot.
2. **Recalibrate the pallet** (section 3). Unchanged, and still the highest-value
   separate task: `~/hrcb/pallet_pose.json` predates the demonstrations.
3. **The geometry gate stays.** Point (d) is also the quantitative case *for*
   `--release-on geometry`: a gate that measures the box against a seat is doing
   the job the policy demonstrably cannot do from 73 episodes. Section 4c's
   choice of gate shape is still the open question, and it matters more than the
   architecture did.
4. **Do not deploy SmolVLA.** It costs 1.2 GB, ~150 ms per chunk refill against
   the driver's 300 ms `max_action_age`, 28 ms of per-step preprocessing against
   a 50 ms loop budget, and a `transformers` + tokenizer-cache dependency on an
   offline controller — for no measured gain. `deploy/seed_smolvla.sh` exists if
   this is ever revisited.

## 5. Artefacts

```
tools/eval_release_offline.py            the replay harness (in ~/lerobot)
tools/rollouts_to_lerobot.py             rollout archive -> LeRobot v3.0
~/h1_policy/split_nl.json                the 73/14 split
~/h1_policy/offline_{act_deployed_split,act_baseline,smolvla_15k,smolvla_30k}.json
~/h1_policy/{train_act_split,train_smolvla_nl}.sh      both ran on jqr@192.168.31.201
~/h1_policy/smolvla_base_h1              smolvla_base with H1 features + processors
deploy/seed_smolvla.sh                   controller prep, unused
```

Datasets (local): `topstar/h1-flush-place-lock-basefix-r1c0fix-nl` and its
`_train` / `_holdout` splits; `topstar/h1-flush-place-rollouts` (52 real rollouts,
50366 steps, no video key — the recorder sampled frames at 1 Hz, now fixed to
every frame).


