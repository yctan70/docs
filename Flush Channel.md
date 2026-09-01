# The flush channel

### Why an observation and not a gate

The classifier's first form could only *end* a rollout by forcing `grip.lock`
low. Two findings from the 2026-08-27 armed runs say that is the wrong shape.

**The seam is often invisible when the decision matters.** The `r1c1L0` box above occluded the very join it would have been judged on. The classifier said 0.0148, correctly, and its score rose monotonically as the box came down (+27.4 mm → 0.0005, +0.5 mm → 0.1455, −6.4 mm → 0.5381). A seam-reading gate cannot fire before the seam exists — but the *trend* was information the whole time, and a binary gate is built to discard it.

**Corners cannot use an external gate at all.** At `r2c0`/`r0c1` the release is a mid-sequence transition; the policy then drives the left-arm raise, torso lift and right-arm push itself, and forcing the lock from outside strands it in an unseen state (POLICY_STATUS §4a). An observation channel has no such restriction — the policy can consult it anywhere in its own sequence.

### What is in the channel

`observation.state[30]`, named `flush.score`, appended after `mobile_base.theta`.

**Not `1 − grip.lock`.** At training time that is an exact copy of the answer the policy must emit at the same timestep. ACT would learn `grip.lock = 1 − flush`, stop reading the image for the release, and key the whole post-release chunk on the channel — and at deployment the channel becomes a noisy probability it has
never seen. `widen_state_flush.py --scores oracle` builds exactly that dataset, labelled as the **leak check** it is, so the failure can be measured instead of argued about.

What goes in is what a scorer actually outputs on each frame, computed offline from the recorded video by `tools/score_flush.py`. Train and deploy then see the same imperfect signal, and since `flush = C(image)` is a function of an input the policy already has, it is a feature rather than a leak.

**Honest limit.** The scorer trains on the same 89 episodes, so this injects *no new information*. The gain is sample efficiency — one densely-supervised scalar against 22 dims from 73 episodes. To add information, the scorer must also train on rollout frames.

### Why it goes last

Index 30, the end. Anything earlier silently breaks two hard-coded offsets that would still produce plausible numbers: `tools/label_flush_lock.py`'s `GAP_INDEX = 18` and `cli/swivel.py`'s `[:18]` slice.

### Corners are scored, never gated

`POSITION_OF_CELL` now covers all eleven policy cells. A channel that is live at seven cells and frozen at four teaches the policy that "frozen" means "corner", which is a cell identifier rather than a flush signal. `configs/preprocess.roi.yaml` has no ROI for the four corner positions, so they are letterboxed whole rather than cropped — worse evidence than the non-corner cells get, and *consistent*
between labelling and deployment, which is the property that matters for an observation.

As a gate they are still refused: `--align-mode gate` disables the monitor on a corner outright, and `--align-mode both --align-arm` scores but does not arm.

### Colour order

The classifier package works in **BGR** throughout (its training frames come from `cv2.imread`; `Ultralytics` reads a numpy `source=` as BGR). Every `LeRobot` camera delivers **RGB**. The deployed monitor was feeding swapped channels.

Measured on three episodes against the time-to-release target rather than assumed: +0.553 (BGR) vs +0.475 (RGB) at r1c1L0, and within 0.02 either way at `r0c0L0` and `r1c1L1`. Real but small — it is **not** the explanation for the 2026-08-27 timeouts. Both paths now convert once, at the boundary, to BGR.

### The pipeline

```bash
# 1. supervision + crops from the recorded video  (env_isaaclab)
python ~/lerobot/tools/score_flush.py \
    --out-targets ~/h1_policy/flush_targets.csv \
    --out-frames  ~/h1_policy/flush_frames --frame-stride 2

# 2. the scorer.  Skip if using the shipped YOLO26n-cls classifier instead.
python ~/box_alignment_rescope/tools/train_flush_regressor.py \
    --frames ~/h1_policy/flush_frames --targets ~/h1_policy/flush_targets.csv \
    --lopo --out ~/h1_policy/flush_reg

# 3. score every frame with it
python ~/lerobot/tools/score_flush.py --model <scorer.pt> \
    --out-scores ~/h1_policy/flush.csv

# 4. widen the dataset 30 -> 31
python ~/lerobot/tools/widen_state_flush.py --scores ~/h1_policy/flush.csv

# 5. retrain ACT FROM SCRATCH on the widened dataset
#    (--policy.type=act, NOT --policy.path: make_policy only fills
#     input_features when they are EMPTY, so a 30-dim checkpoint's config
#     survives and fails at the nn.Linear matmul)

# 6. run it
h1-eval-cycle ... --align-model <scorer.pt> --align-mode observe \
                  --align-preprocess ~/box_alignment_rescope/configs/preprocess.roi.yaml
```

The scorer is installed into `get_observation()` itself, so the number in the state vector describes the exact frame beside it. It runs **every** step at 20 Hz — the channel is an observation, not a windowed vote — while the temporal gate, if `--align-mode both`, is still fed on the original 10 Hz cadence, because its 8-frame window was calibrated there and feeding it at 20 would halve the evidence window while leaving every number in the calibration looking unchanged.

Measured cost: 10.9 ms on the robot against a 50 ms loop, plus ACT's ~10 ms.

### The guards

Three things fail late and quietly without them, so all three are checked before
the robot is touched:

- `check_state_dim` — the checkpoint's state width against the spec's. Without  it, a mismatch is a matmul error from inside `nn.Linear` with a box in the plates.
- `check_state_names` — every name the spec asks for against what the robot publishes. This is the one that actually catches the flush channel: a 31-name spec against a 30-key robot is a bare `KeyError` inside `build_dataset_frame` on the first `step()`, after the chassis has parked. `preflight()` never calls `build_dataset_frame`.
- `Policy.__init__` refuses a 31-dim spec with no `--align-model`. A constant channel normalizes to a column of zeros with no gradient, so nothing complains and the policy leans on it hardest.

The robot itself refuses to `connect()` with `with_flush` on and no scorer installed, for the same reason.

### Verification

- **Offline first, no robot.** `tools/eval_release_offline.py` replays held-out  episodes through the same `predict_action` path. Compare 31-dim against the current 30-dim ACT on release-frame error and never-released count. This is the gate for touching the robot.
- **Leak check.** Train one variant on `--scores oracle`. If release behaviour is unchanged, the policy ignored the channel; if it collapses when the channel is zeroed at inference, the policy depends on it entirely and the channel is doing  all the work.
- **Round-trip.** `widen_state_flush.py` reads the result back and asserts channel 30 equals the scores and channels 0–29 are bit-identical to the source.
- **Stats.** It also asserts `meta/stats.json["observation.state"]["mean"]` has 31 entries — a 30-entry mean against a 31-dim tensor is a broadcast `RuntimeError`  at the first batch.
- **Corners.** Validate on within-run score trajectories, never on two-axis seat distance and never in the pallet frame.

---

## Still open

Three of four 2026-08-27 runs timed out with the box high, and `r2c1L0` never descended at all. With the deadline release those runs would at least place their boxes, but the underlying question stands — whether the place path is failing to complete its descent — because it determines both placement quality and whether more rollouts teach the scorer anything.


