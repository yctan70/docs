

# H1 Palletizing — Part 3 Development Plan (6 weeks + 2-week integration)

**Robot:** Topstar H1 wheeled humanoid (18-DOF upper body: 2×7-DOF arms, torso pitch [−95°, 0°], prismatic torso lift 0.45 m, head pan/tilt; 4-wheel swerve base)
**Owner:** Part 3 (palletize boxes on the pallet). Parts 1 (pick from conveyor) and 2 (navigate to pallet) are developed in parallel by colleagues.
**Period:** 2026-07-06 → 2026-08-28 (6 weeks part-3 development + 2 weeks system integration)

## Scope and approach

Part 3 receives the robot already parked near the pallet and holding a box (parts 1–2 output) and must place the box flush against its neighbors per the pallet pattern.

**Policy architecture (decided in design discussion):**

| Layer | Method | Role |
|---|---|---|
| Pattern planner | Deterministic algorithm | Target pose + neighbor mask (none / x / y / corner) + approach direction for each box |
| Transport & approach | Scripted (park refine, `torso_z` lift, torso pitch, dual-arm Cartesian IK) | Bring box to the pre-place offset pose (~3 cm clear of neighbors) |
| Seating (base policy) | **ACT** (LeRobot/`lerobot4`), canonicalized to target-cell frame, conditioned on neighbor mask; vision + proprioception | Final approach and flush placement, trained from teleop demos |
| Refinement | **EXPO-FT** ([arXiv 2605.25477](https://arxiv.org/abs/2605.25477)): online RL fine-tuning on the real robot — lightweight Gaussian edit policy + Q-function action selection over base/edited candidates, base continues imitation updates on the shared replay buffer, **human-in-the-loop interventions** via the same teleop rig | Close the last-centimeter precision/robustness gap on the deployment distribution |

**Why EXPO-FT instead of sim-trained residual RL:** fine-tuning happens on the real robot with real observations (vision included), so there is no sim-to-real gap and no need to run/approximate the vision-based ACT inside the simulator. The paper reports ~20 min of online data to reach 30/30 on contact-rich tasks starting from a competent IL base — which is exactly why gate G2 (a competent ACT) is a hard prerequisite. Simulation (MuJoCo/Isaac via `isaac_bridge`) is demoted to a supporting role: reach/tipping analysis, safety-envelope validation, and a smoke test of the EXPO-FT training loop before it touches hardware.

**Constraints:** No force/torque sensor is installed at this stage. Contact awareness comes from proprioception — joint positions/velocities and the effort readback already provided by xapi (no new hardware) — as policy observations, and the online reward penalizes effort spikes so the Q-function cannot learn to crush boxes "flush."

**Canonicalization (key to data efficiency):** all seating-policy observations and actions are expressed relative to the commanded target-cell pose, with the neighbor mask as an explicit input. Layer height, pallet side, and stack progress are absorbed by the scripted transport layer, so demos from different layers/positions collapse into the same distribution — four discrete modes total.

**Caveat to validate early:** the EXPO-FT paper uses a π₀.₅ VLA base; the mechanism (imitation-trained chunked base + edit policy + Q-selection + shared buffer) does not depend on that, but running it with an ACT base is our responsibility to validate — hence the week-4 sim smoke test.

---

## Week 1 (Jul 6–10) — Sim scene, pattern planner, scripted scaffolding

**Goal:** Everything the learned policies sit on top of.

- Measure the cell: pallet dims/location, box dims/weight range; agree provisional interface poses with parts 1–2 owners (handover box pose, parking standoff pose).
- Build the MuJoCo scene: pallet, boxes with realistic mass/friction, robot at standoff (used for reach/tipping analysis and the week-4 loop smoke test — not for policy transfer).
- Implement the **pattern generator**: box dims + pallet dims → per-box target pose, neighbor mask, approach direction. Unit-test full-pallet layouts.
- Scripted **transport & approach primitive**: refine park pose → set lift for layer → pitch over pallet → IK to pre-place offset pose. Define the ACT handoff state precisely (pose window + velocity ≈ 0).
- **Reach/stability analysis:** torso pitch is forward-only — map reachable pallet cells per layer per parking pose; check tipping margin with arms extended + max box mass at top layer.
- Define the part-3 ROS2 interface early so colleagues can build against it: `place_box(target_pose, neighbor_mask)` action + result codes.

**Deliverables:** sim scene, tested pattern generator, approach primitive reaching the handoff state in sim, reach map, published interface spec.

## Week 2 (Jul 13–17) — Teleop pipeline and pilot data

**Goal:** End-to-end data path proven: teleop → canonicalized dataset → trained ACT → replayed on robot.

- Bring up `lerobot4` teleop for the seating segment on the real H1 at a mock pallet (part 3 needs no conveyor or navigation — boxes are hand-loaded into the grasp). This rig later doubles as the **EXPO-FT intervention interface**, so invest in its ergonomics now.
- Implement the **canonicalization layer**: record observations/actions in the target-cell frame; attach neighbor mask + offset vector to every episode. This is the highest-leverage code of the project — test it by replaying transformed demos.
- Decide and fix the observation set: head-camera RGB (crop around target cell), joint positions/velocities, xapi effort readback, neighbor mask, offset-to-target.
- Collect a **pilot dataset**: ~10 demos per neighbor class (40 total). Train a first ACT checkpoint; evaluate qualitatively on the robot in the easiest class (no neighbor).

**Exit criteria (Gate G1):** pilot ACT reproduces a no-neighbor and a one-neighbor seating on hardware (any success rate — this gates the pipeline, not the policy). If teleop ergonomics make flush seating hard to demonstrate, fix the teleop mapping now: it degrades both the demos and the later interventions.

## Week 3 (Jul 20–24) — Full data collection and ACT training

**Goal:** The production ACT policy — the base EXPO-FT will fine-tune.

- **Stratified collection:** 40–60 demos per neighbor class (~200 total), randomizing true neighbor position ±1–2 cm around nominal, box size within range, initial offset, and (for upper classes) layer height — coverage by construction, not by luck.
- Data hygiene: discard failed/recovered-by-luck demos or label them; keep a held-out set per class.
- Train ACT (sweep chunk size and learning rate; this scale trains in hours on a single GPU). Evaluate per-class on hardware: success rate, residual gap to neighbors (measured by vision), cycle time.
- In parallel, start the **automated reward and reset infrastructure** EXPO-FT needs (see week 4): a head-camera flushness/success check (gap-to-neighbor measurement of the placed box) and a scripted reset (re-grasp the placed box, return to the handoff pose). Palletizing resets are cheap to script — exploit that.

**Exit criteria (Gate G2):** ACT-only seating ≥ 70% success per class on hardware. Below that, diagnose (data coverage vs. teleop quality vs. observation gaps) and extend collection into week 4 rather than proceeding — EXPO-FT's tens-of-minutes fine-tuning result assumes a competent base; online RL refines a working policy, it does not rescue a broken one.

## Week 4 (Jul 27–31) — EXPO-FT bring-up and first online fine-tuning

**Goal:** The EXPO-FT loop running safely on hardware, fine-tuning the easiest classes.

- **Sim smoke test first:** run the full EXPO-FT loop (actor/learner, replay buffer, edit policy, Q-selection, chunk-overwrite interventions) with the ACT base in MuJoCo. This validates our ACT-base integration — the paper used a π₀.₅ base — and debugs the plumbing where mistakes are free.
- **Reward automation on hardware:** vision-based flushness + placement-pose success signal; effort-spike penalty term (no force sensor — this is what keeps the Q-function from crushing boxes); time penalty.
- **Scripted resets:** un-place (re-grasp the just-placed box), restore the scene, return to handoff pose; randomize neighbor positions between episodes per the canonicalization scheme.
- **Safety governor:** edit-policy output clamp, workspace fence, effort-spike abort, reduced speed. Validate envelopes in sim before enabling online exploration on hardware.
- Begin online fine-tuning on the no-neighbor and single-neighbor classes, with an operator providing **HIL interventions** through the teleop rig when the policy stalls — interventions overwrite the affected timesteps within the action chunk and enter the shared buffer, improving both the base (imitation) and the Q-function (TD).

**Exit criteria (Gate G3):** EXPO-FT loop stable on hardware (no governor aborts from the edit policy, buffer/learner keeping up with the actor), with a measurable success-rate improvement over ACT-only on at least one class.

## Week 5 (Aug 3–7) — Fine-tuning across all classes and layers

**Goal:** Fine-tuned policy meeting the part-3 bar everywhere.

- Extend online fine-tuning to corner class and upper layers (top-layer sessions gated on the week-1 tipping analysis, verified on hardware at low speed).
- Budget intervention operator time deliberately: concentrated intervention early in each class's session, tapering to autonomous as the policy improves — track intervention rate per session as the convergence signal.
- A/B evaluate ACT-only vs. fine-tuned policy per neighbor class per layer (≥ 10 trials per cell) with training frozen; keep the fine-tuned checkpoint only where it wins.
- Freeze a **deployment checkpoint** (base + edit policy + Q-function) and re-verify determinism/latency of the full inference path at cycle speed.

**Exit criteria (Gate G4):** ≥ 90% first-attempt flush success over ≥ 40 hardware placements spanning all classes and layers, no human intervention, on the frozen checkpoint.

## Week 6 (Aug 10–14) — Part-3 hardening and integration readiness

**Goal:** Part 3 works as a black box against its published interface.

- Full-pallet part-3 runs with boxes hand-loaded at the agreed handover pose (mocking parts 1–2): stack complete layers, multiple layers, log every placement. Training stays frozen; the reward-automation vision check is reused as an online **misplacement detector**.
- Failure handling inside part 3: re-approach retry, misplacement detection, abort-and-report codes on the action interface.
- Freeze the interface: final `place_box` action spec, handoff pose window, timing budget per placement. Deliver to colleagues.
- Buffer for slippage from weeks 3–5 (data collection and online fine-tuning always slip).
- Documentation: training/eval runbook (datasets, checkpoints, how to resume fine-tuning, intervention procedure), operator notes.

**Exit criteria (Gate G5, integration readiness):** N consecutive full-layer runs meeting the G4 metric, interface spec frozen and acknowledged by parts 1–2 owners.

## Week 7 (Aug 17–21) — Integration: interfaces and dry runs

- Joint bring-up with parts 1 and 2: verify the real handover state (box-in-grasp pose from part 1, parking pose distribution from part 2) matches what part 3 was trained for. **This is the main integration risk** — if part 2's parking spread exceeds the approach primitive's tolerance, either tighten part 2's visual servoing or widen part 3's tolerance with a short additional EXPO-FT session under the real upstream distribution (this is where online fine-tuning pays off: adaptation is a session, not a retrain cycle).
- End-to-end dry runs at reduced speed: single box through pick → drive → place. Debug state-machine transitions, timeouts, error propagation between the three action servers.
- Re-measure part-3 success under real upstream variability.

## Week 8 (Aug 24–28) — Full-system runs and acceptance

- End-to-end full-pallet runs, increasing to target speed; measure system cycle time and identify the bottleneck stage.
- Endurance: unattended supervised runs (operators watch, don't touch); failure-mode catalog across the whole pipeline with recovery ownership assigned per part.
- Final acceptance: complete pallet palletized end-to-end, ≥ 90% first-attempt flush success, no manual intervention; demo video; final metrics report.

---

## Risks and mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| ACT-only success too low for fine-tuning to be credible (G2) | Medium | Week-2 pilot catches pipeline issues early; stratified data + canonicalization maximize data efficiency; extend collection before escalating |
| EXPO-FT unproven with an ACT base (paper used π₀.₅) | Medium | Week-4 sim smoke test of the full loop before hardware; fallback: distill demos into the paper's base architecture, or fall back to offline DAgger-style top-ups |
| Reward automation errors (vision flushness check mis-scores) | Medium | Calibrate the check against hand-measured gaps in week 3; conservative success threshold; spot-audit episodes during training |
| Reward hacking / unsafe exploration online (crushing boxes flush) | Medium | Effort-spike penalty in reward, edit-policy output clamp, workspace fence, effort-abort governor; envelopes validated in sim first |
| No force sensing → contact only observable via effort readback/vision | Medium | Effort readback is a policy observation; penalty + abort thresholds tuned on week-3 hardware measurements |
| Online fine-tuning consumes more robot/operator time than planned | Medium | Intervention-rate tracking per session gives early convergence signal; per-class opt-out (ship ACT-only where fine-tuning doesn't win) |
| Part 2 parking spread exceeds trained tolerance (integration) | Medium | Handoff window agreed in week 1, measured in week 7; short EXPO-FT adaptation session under the real upstream distribution |
| Top-layer far-side cells unreachable (forward-only torso pitch) | Medium | Week-1 reach map; pattern planned near-side-last or re-park per column |
| Tipping with arms extended + box at top layer | Low–Med | Week-1 sim analysis; hardware top-layer trials gated on it |
| Data collection / fine-tuning weeks slip | High | Week 6 deliberately light; gates G1–G3 force early detection instead of late surprises |

## Out of scope (this phase)

- Parts 1 and 2 development (colleagues; only their interfaces appear here)
- Force/torque sensor installation
- Mixed/unknown box sizes (pattern generator assumes a known size set; online 3D bin packing later)
- Depalletizing; VLA-based policies as the base (ACT is the base; π₀.₅ only as fallback); multi-robot coordination

## References

- EXPO: Stable Reinforcement Learning with Expressive Policies — [arXiv 2507.07986](https://arxiv.org/abs/2507.07986)
- EXPO-FT: sample-efficient online RL fine-tuning of pretrained policies with HIL interventions — [arXiv 2605.25477](https://arxiv.org/abs/2605.25477)
- ACT / ALOHA (Zhao et al., 2023); residual RL (Johannink & Silver et al., 2018) — background for the design discussion

