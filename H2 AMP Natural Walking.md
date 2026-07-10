# H2 AMP Natural Walking — Critical Tuning Summary

**Final policy:** run `2026-07-09_10-07-28_v15_standmask`, checkpoint 27900, deployed as `~/topstar_h2/dist/amp_v3` (verified in MuJoCo 3-terminal test 2026-07-09).
**Capabilities:** natural gait, velocity + yaw tracking (behavioral gain 0.97), stands still at zero command, clean stand↔walk transitions, no falls (98% episode timeout rate).

This document records the tuning changes that actually mattered across v1→v15, in order of discovery. Each was found by eliminating alternatives — the "what didn't matter" sections are as load-bearing as the fixes, since they are what future debugging should *not* revisit.

---

## 1. Self-collision phantom forces blinded all contact rewards (v1–v11)

**Symptom:** `gait_feet_frc` stuck at exactly 0.0000 in every run; `current_air_time` never accumulated; policies converged to flat shuffling.

**Root cause:** the H2 spawn config inherited `enabled_self_collisions=True` from `UnitreeUrdfFileCfg`. The H2 URDF uses raw STL collision meshes whose convex hulls permanently interpenetrate (`knee↔ankle_roll` 12–26 kN, `waist_yaw↔head_pitch` ~40 kN). The foot contact sensor therefore never read below ~12 kN, and `exp(-F²/10000)` ≡ 0.

**Fix** (`topstar_h2.py`): `enabled_self_collisions=False` (TienKung runs the same). Verified by dangling the robot mid-air: all body forces 0; standing feet read ~354–400 N (≈78 kg robot). `feet_too_near` (-2.0, threshold 0.2 m) guards leg crossing instead.

**Lesson:** any foot-contact reward is meaningless until the sensor is verified clean. Diagnostic: teleport root to z=2 m and print all per-body contact forces >1 N.

## 2. Gait structure: clock rewards + phase observation + RSI

AMP style reward alone never produced alternating stepping from a standing start (flat-shuffle local optimum). Three TienKung-ported pieces fixed the bootstrap:

- **Gait clock rewards** (weights 1.0/1.0/0.6): `gait_feet_frc_perio` (no foot force   during scheduled swing), `gait_feet_spd_perio` (still feet during stance),   `gait_feet_frc_support_perio` (weight bearing during stance). Clock: cycle 0.85 s,  air ratio 0.38 per foot, offsets 0.38/0.88 (anti-phase).
- **Gait phase observation** (6 dims: sin/cos φ_L, sin/cos φ_R, air ratios) in policy and critic obs — the policy gets the clock instead of discovering it.
- **Reference State Initialization** (RSI, 50%): episodes start from random expert walk frames, guaranteeing foot-in-air states from iteration 0.

## 3. Discriminator observations must be velocity-blind

**Symptom (v2–v13):** yaw commands ignored entirely — behavioral tracking gain ≈ 0 regardless of reward weights.

The discriminator obs originally included `base_lin_vel`/`base_ang_vel`. With base velocities visible, the discriminator taxed any commanded yaw rate outside the mocap turning distribution. **Fix:** 30-dim root-frame-only disc features — leg joint pos/vel (12+12) + foot positions in the root body frame (6) — and the matching loader flag `include_base_vel=False` (both sides must agree). TienKung's working discriminator is likewise velocity-blind. Foot positions in root frame are what make alternating gait visible to the discriminator (joint angles alone can't distinguish a stride from a shuffle).

## 4. Yaw command shaping (necessary but not sufficient alone)

- **Heading command mode** (`heading_command=True`, stiffness 0.5): yaw rate derived from P-control on heading error rather than directly sampled.
- **`ang_vel_z` clamp ±1.57** (was ±0.5): with ±0.5, a policy that ignores a saturated command still collects exp(−1) = 37% of the yaw reward — "never turn" is a stable  optimum. At ±1.57 ignoring collects ~0.
- **`joint_deviation_legs` −0.15** (was −1.0): natural turning needs ~0.47 rad hip-yaw  L1 (dataset measurement); −1.0 axed exactly the joints turning requires, 7× harder  than TienKung.

A v13 fine-tune *and* a from-scratch run with only these fixes still failed (gain ≈ 0) — they became effective only after fix #6.

## 5. PPO optimizer: adopt TienKung's settings permanently

**Symptom:** action noise std *grew* 0.5 → ~2.0 by 2k iterations in every early run (±0.5 rad joint noise — precise command-conditioned control cannot be expressed).

**Fix** (`H2PPORunnerCfg`): adaptive LR schedule at 1e-3 (was fixed 3e-4), entropy_coef 0.005 (was 0.01), value_loss_coef 1.0 (was 0.25 — a weak critic gives noisy advantages, and entropy wins), max_grad_norm 1.0 (was 0.5). After the fix, std anneals healthily (0.5 → 0.21 by 23k iters). This alone did **not** fix yaw (probe2 proved it), but it is a necessary co-factor — keep permanently. Also aligned penalties to TienKung: action_rate −0.01, dof_pos_limits −2.0, flat_orientation −1.0, plus ermination penalty −200.

## 6. THE decisive fix: bounded LSGAN style reward

**Symptom:** even with fixes #3–#5, yaw tracking plateaued at gain 0.118 (v13_final, style_weight 0.1). A probe with `--style_weight 0.0` gave gain **1.008** — the AMP style reward itself was the blocker.

**Why:** the softplus style reward is *unbounded*. Even at style_weight 0.1 it contributed ~45–65% of the mixed reward, a command-independent signal that diluted the task gradient. TienKung "works at 0.3" because their style reward is LSGAN-bounded: `reward = scale × clamp(1 − 0.25(d−1)², min=0) ∈ [0, scale]`.

**Fix** (amp-rsl-rl `discriminator.py`, new `loss_type="LSGAN"`): discriminator trained with MSE targets expert +1 / policy −1; bounded reward as above with `disc_reward_scale=0.3` and `style_weight=0.1` → style hard-capped at **0.03/step ≈ 12%** of mixed reward.

**Result (v14):** behavioral yaw gain **0.970** with style ON (vs 0.118 softplus), heading error → 0.02 rad, error_vel_xy 0.37 (best to date), style signal retained for gait quality.

## 7. Stand mask: standing still at zero command (v15)

**Symptom:** v14 marched in place at zero command.

**Root cause:** the gait clock never stops, and `gait_feet_frc` paid ~0.76/step more for stepping than for standing (stance force ~400 N → exp(−16) ≈ 0 during every swing window). 20% standing envs and the tanh velocity gate on the style reward didn't help — the clock-reward pressure remained. Inherited from the TienKung recipe (their walk task has the same limitation).

**Fix** (all gated on full 3D command norm < 0.1 — yaw included, since turning in place requires stepping):

- Gait-clock rewards stand-masked: frc → swing window forced to 0; spd/support → both
  feet treated as full stance (standing with planted, still feet becomes the optimum).
- `gait_phase` obs frozen at t=0.06, a double-stance point (both feet in stance for
  t ∈ [0.02, 0.10]); the underlying episode clock keeps running.
- New `stand_still_joint_deviation_l1` (−0.5, legs) pulls the standing pose to default
  instead of a frozen mid-stride crouch.
- `rel_standing_envs` 0.2 → 0.3.

**Training:** fine-tuned from v14 ckpt 23400 (full resume incl. optimizer + disc), ~4.5k iterations sufficed. `stand_still` converged −0.22 → −0.05; tracking *improved* (error_vel_xy 0.335). Verified behaviorally: settles from mid-stride in ~1 s, residual sway 0.05 m/s, no gait-frequency component in frame-motion FFT; stand→walk ramp-up 0.60 s, walk→stand settle 0.72 s, walk tracking 0.493/0.5 m/s.

---

## Methodology lessons

- **Only behavioral tests are trustworthy.** Training metrics lie: `track_ang_vel_z`  ~0.21 normalized and `error_vel_yaw` ~3.9 are both consistent with a policy that  ignores yaw entirely (standing envs + unsaturated P-control commands mask it). Every  yaw conclusion came from `scripts/rsl_rl/yaw_diag.py` (forced heading targets,  measures actual tracking gain/correlation). Same for standing: video + frame-motion  FFT, and `scripts/rsl_rl/transition_diag.py` or mid-episode command switching.
- **Ablate with probes, not intuition.** The style-reward root cause was found by a  binary probe (`--style_weight 0.0`), after reward balance, obs/reward plumbing,  physics, fresh weights, longer training, and the optimizer had each been eliminated  by its own probe.
- **Fine-tuning works when only one regime changes.** v15 (stand mask) fine-tuned  cleanly from v14 because walking/turning were untouched. v13's yaw fine-tune from v12  failed because the no-turn optimum persisted — behavioral failure modes baked into  the weights need from-scratch training (or a reward-structure fix, as it turned out).
- **The reference implementation's *whole* recipe matters.** Every root cause (#1, #3,  #5, #6) was a place where our setup silently differed from TienKung-Lab. Diff against  the proven system first; tune weights second.

## Deployment notes

- Gait-clock policies (v13+) **require** `~/topstar_h2/python/h2_amp_rl_runner.py` —  the older `h2_rl_runner.py` silently feeds zeros for `gait_phase`, which is  permanently out-of-distribution and destabilizes the robot.
- The runner reads the stand mask from deploy.yaml `gait_phase.params`  (`stand_threshold: 0.1`, `stand_phase: 0.06`) and freezes the phase obs on the  *ramped* command, exactly matching training. Bundles without these params (amp_v2)  run unchanged.
- ONNX obs layout is **per-term history** (each term's 5 frames contiguous), not  frame-major. Exported ONNX has fixed batch dim 1.
- Trained command ranges: vx [−0.5, 0.8], vy ±0.3, vyaw ±1.57. Runner convention:  `--kp-scale 2.0` for MuJoCo sim, `1.0` for hardware.

## Key artifacts

| What | Where |
|---|---|
| Final checkpoint | `scripts/rsl_rl/logs/rsl_rl/topstar_h2_velocity_amp/2026-07-09_10-07-28_v15_standmask/model_27900.pt` |
| Deployment bundle | `~/topstar_h2/dist/amp_v3/` (policy.onnx + deploy.yaml) |
| Runner | `~/topstar_h2/python/h2_amp_rl_runner.py` |
| Env config | `source/.../tasks/locomotion/robots/h2/velocity_amp_env_cfg.py` |
| Gait rewards / stand mask | `source/.../tasks/locomotion/mdp/rewards.py`, `observations.py` |
| LSGAN discriminator | `~/amp-rsl-rl/amp_rsl_rl/networks/discriminator.py` |
| Train launcher | `scripts/rsl_rl/train_amp.py` (LSGAN cfg, `include_base_vel=False`) |
| Behavioral tests | `scripts/rsl_rl/yaw_diag.py`, `transition_diag.py`, `test_stand_mask.py` |
| Expert dataset | `~/amp_datasets/h2_walk_v3` (130 segments, incl. h2_stand_*) |




