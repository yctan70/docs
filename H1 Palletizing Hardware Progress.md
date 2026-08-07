# Topstar H1 palletizing — real-hardware progress

Status as of **2026-08-07**. Covers the move off Isaac Sim onto the physical H1 (arms over ROS2) and the Seer AGV chassis (TCP), for the job of palletizing 400 × 250 × 200 mm boxes onto an 880 × 810 mm pallet.

Everything here is on the untracked working tree (`tools/`, `config/`,
`tests/tools/`), branch `branch-v0.4.1`.

---

## 1. Where it stands

The full cycle **exists and is assembled** end to end:

```
park on LM5 → grip a box → drive to the cell → place → let go → back out
```

`tools/h1_pallet_routes.py --execute` runs all six layer-0 cells, gating each stage. What has actually been *executed on the robot* is narrower than that — see §4, which is the section to read before running anything.

The parts are individually proven on hardware; the assembled sequence is not.

---

## 2. The stack

| File | Role | Hardware? |
|---|---|---|
| `tools/h1_real_teleop.py` | Quest teleop, FK plate gauge, plate calibration, scripted place replay | yes |
| `tools/h1_grip_math.py` | mount transforms, `K`, `base_to_mount`, pure numpy | offline lib |
| `tools/h1_held_pose.py` | read-only `/lowstate` snapshot of a held grip | yes |
| `tools/h1_place_path_live.py` | generates place paths from a live reading + tape measurements | yes |
| `tools/h1_auto_grip.py` | teleop-free grip, carry, place, release, clear, retract | yes (through place) |
| `tools/h1_pallet_routes.py` | chassis routing + the whole cycle; shells out to `h1_auto_grip` | chassis only |
| `tests/tools/test_h1_auto_grip.py` | 43 offline geometry/pacing tests | n/a |
| `tests/tools/test_h1_grip_math.py` | SE(3) / mount-transform tests | n/a |

Two machines, two transports, deliberately separate:

- **Arms** — ROS2 `/lowcmd` + `/lowstate` at 50 Hz, system Python 3.10, needs
  `source ~/topstar_ros2/setup.sh`. Launch the robot side with
  `ros2 launch topstar_ros2_example h1_sim.launch.py backend:=xapi`.
- **Chassis** — Seer TCP on `192.168.31.226`, ports 19204–19210, via
  `~/hrcb`'s `SeerComm`. **The ROS2 base backend in `~/hrcb` does not work and
  must not be used.**

---

## 3. Timeline

**2026-08-04 — arms talk, and the mount transforms were wrong.**
`h1_ros2_node.py`'s `_T_BODY_TO_{RIGHT,LEFT}_ARM` used the raw `Robot_*_Hand_base_Joint` origins, but they must map from `little_top.urdf`'s `base_link`, which sits 0.1445 m short in z and is rotated −90° about z. The mount separation is **0.53 m, not 0.241 m**. Proved without a reference model: at mirror-symmetric joints the old constants put the left EE **1.30 m** off its mirror image, and sweeping all 128 sign conventions still left 0.665 m. Fixed in the constants, not the URDF (which is shared by four other programs). The independent cross-check is that `K = inv(T_LEFT) @ T_RIGHT` came out `Rx(π)`, t = (0, 0, −0.53), matching a fit against `h1_with_ee.urdf` to 1.7e-9 over 40 poses per arm.

**2026-08-05 — plate calibration, and the first live place.**
`config/h1_plate_calibration.json` written from 50 live samples: plate offset **0.08939 m per plate** on a 390 mm box, batch spread 0.0 mm. The URDF nominal is 0.0803; the 9 mm difference is real and is why calibration is not optional.

The rate limiter was found to dominate grip drift, not operator speed: `_MAX_STEP_RAD` at 0.02 rad/tick clipped nearly every tick, and clipping is per-joint, so it changed the *direction* of the step and deformed the grip. Measured on a 50 mm / 10° carry: **223 mm drift at 0.02, 14.2 mm at 0.04** (identical to no limiter). Now 0.04.

**First live scripted place aborted 40% into the approach.** 
Grip was 6.87° out of parallel, 11.0 mm lateral, on a 398 mm box; the left arm hit its 5 mm reach limit and the schedule stalled 5 s behind. Not reproducible offline from guessed grip poses, so no confirmed cause. Two responses shipped: `p` now pre-flights the whole run on private solver copies before publishing anything, and paths are aimed at the **box centre** rather than the sim's plate positions.

**2026-08-05 — the pallet was re-planned from a tape measure, not the sim.**
The vertical datum comes from measuring the *held box centre's height*, which makes two unknowns cancel: where the plates sit on the box face (invisible to FK) and base_link's height above the floor. The derivation-based estimate said 0.903 m where the tape said 0.890. Measured 890 mm held → 210 mm seated = a **680 mm drop**.

That drop is only reachable by **bending the torso**. With the grip attitude held and pitch at 0.001 the box bottoms out 280 mm short; torso lift is worth only 70 mm from there. Pitch 1.3–1.6 rad solves every layer-0 cell.

Also settled here: **a reachable endpoint is not a runnable path.** All six endpoints solved, only the three furthest ran, the near ones stalling on RIGHT_ELBOW hitting −1.7802 mid-path while the endpoint itself used −1.53. Two fixes that were measured and do **not** work, recorded so they are not re-proposed: re-posturing the arm through the 7-DoF null space (identical stall percentages — the wrist was a passenger, the elbow is fixed by hand-to-shoulder distance), and making the torso bend lead the descent (broke every cell, including the three that worked).

Path shape had to become `direct` — ease straight from held to hover with the bend ramping throughout. The sim's lift → traverse → descend exists to hop a 0.51 m deck; on a 0.11 m deck it carries at full height with the torso upright and stalls mid-traverse. That change alone took r2c1 from stalled-at-60% to 0.48 mm of gap drift.

Canonical seat: **base (−1.300, 0.000, +0.1946)**, which is where the 1.300 m chassis stand-off comes from. Verified on the published `/lowcmd`: 0.063 mm gap drift, 0.22 mm final error, 0 soft-limit violations in 1704 commands.

**2026-08-06 — teleop taken out of the grip.**
`tools/h1_auto_grip.py` builds a *symmetric* grip from a reference rather than replaying it. The recorded reference is crooked (7.53° skew, 15 mm lateral, 1.82° off level, centre 20 mm off the midline); replaying it reproduces all of
that, which is why every place path since had been squaring the box up on the way down. The derived pose keeps only the grip centre's x/z and the roll about the grip axis, and gives both plates the same rotation matrix — physically mirrored in this URDF — so misalignment is 0.00° by construction.

The in-flight abort test was calibrated twice. The first sweep used 26 starts that all had the plates near z ≈ 0.87 and gave "healthy max 4.75 mm", which the robot's own parked pose exceeded 13× within a day. Redone over 33 starts including the live-pose family:

| | peak lag | peak growth / 3 s | on plan |
|---|---|---|---|
| healthy | med 3.83, **max 118 mm** | med 2.48, max 58 | 33/33 |
| unreachable | med 641, max 650 mm | med 127, max 143 | 0/33 |

Absolute lag separates by 5.4×, growth by only 2.2× with a heavy tail — so the sole in-flight abort is a **300 mm ceiling**, and correctness is decided *on arrival*, where the same populations separate by ~1400×.

Both grips ran on the robot this day — the name is the plate *span*, i.e. the box dimension held across the plates: 400-end (500 → 395 mm across the box's 400 mm axis) and 250-end (350 → 245 mm across its 250 mm axis) — each with a 100 mm
forward carry and a place path down onto the pallet.

**2026-08-07 — chassis, then the whole cycle.**
Chassis identified as **`.226`, not the `.245` in the test defaults** — both answer on 19204, but `.245` is a different robot on a different map with three marks. `.226` carries the five LMs and its live pose matches `~/hrcb/pallet_pose.json` exactly.

`tools/h1_pallet_routes.py` recomputes the six park poses from `T_map_pallet` plus the live station list — nothing transcribed. Independent confirmation: the four pallet-side marks landed within 4–27 cm of poses derived purely from the
pallet transform and the grid, headings within 6°, and a park → cell round trip closed to **85 µm**.

Then the arms were wired in, and the cycle assembled (§5).

---

## 4. Verified on hardware vs. offline only

**This is the section that matters.** Offline here means solver dry runs and unit tests, which have caught real problems but cannot see the robot.

### Run on the robot

- Quest teleop with the measured plate lock, including calibration (50 samples).
- The FK plate gauge against a tape measure.
- `h1_auto_grip` approach → squeeze → carry, both grips.
- `h1_auto_grip` … → place, onto the pallet, at 850 mm and ~890 mm grip heights,   both grips (`--place-path config/h1_place_{paths_carry,250end_850,…}.json`).
- Every place path in `config/` was generated from a **live `/lowstate`   reading** plus tape measurements — `source: "live /lowstate + measured pallet/box, no sim geometry"`.
- Chassis: station list, pose, path nav to marks, and a realtime fine step that   converged **228 → 33 → 5 mm** over three passes.

### Not run on the robot

- **The assembled `h1_pallet_routes --execute` cycle.** Never once, in any  configuration.
- **The asymmetric release** (`--release`), the **clear** slide, and the
  **retract**. All three are new on 2026-08-07 and exist only as solver dry runs  and unit tests.
- **Cells r2c0 and r0c1** — the two with a neighbour already seated. See §6.
- **Any box successfully let go by the robot.** The first attempt (r1c0,  2026-08-07) opened the plates but did not free the box — diagnosed and fixed,  see §6, but the fix itself is unrun.

### Open, unresolved

- **Chassis yaw on the fine step.** During the 2026-08-07 test the yaw error sat  at ~85° across all three passes despite `turn=True`, which would mean the 3056  turn is not taking effect. Not called a bug: a second process was driving the  same chassis at the time (observed navigating LM3 → LM4 while a read-only watch  ran), and each new 3055/3056 cancels the previous. **Needs a clean re-run with  sole control before the first full cycle.**

---

## 5. The cycle as it now runs

Four gated steps per box:

```
step 1  drive to LM5        Enter, then the chassis goes on its own
step 2  pick                3× SPACE; you load and centre the box
step 3  drive to the cell   Enter — the box is in the plates for this leg
step 4  place and let go    5× SPACE: carry, place, release, clear, retract
```

Arm motion runs **out of process**, twice per box, because it needs the ROS2 environment and because the chassis drives between the two halves. `h1_auto_grip` leaves the arms holding their last command when it exits, so the box stays gripped across the gap — that is what makes the split safe.

It is always invoked with `--exit-when-done`. Without it, `q` exits 0 whether the legs finished or one aborted, so the chassis would drive off with a dropped box exactly as readily as with a held one. With it, 0 means clean and 2 means not, and the router stops.

**Vertical bookkeeping.** Grip at 850 mm centre height, lift 40 mm the moment the plates close (clears the support *and* puts the box at the 890 mm the place paths start from), carry 100 mm forward for elbow room, then the place path.
Checked numerically: that chain lands **0.3 mm** (250-end) and **3.6 mm** (400-end) from each path's own recorded `start_box_base`.

**Order and letting go.**

| # | cell | grip | via | occupied side | let go |
|---|---|---|---|---|---|
| 1 | r1c0 | 250-end | LM2 | neither | right |
| 2 | r1c1 | 250-end | LM4 | neither | right |
| 3 | r2c1 | 400-end | LM3 | neither | right |
| 4 | r0c0 | 400-end | LM1 | neither | right |
| 5 | r2c0 | 400-end | LM3 | **left** | right |
| 6 | r0c1 | 400-end | LM1 | **left** | right |

The release opens **25 mm wider than the grip gap** — 245 → **270 mm** on the 250-end grip, 395 → 420 on the 400-end. It is asymmetric: one plate takes 20.5 mm of that travel while the other stops 2 mm off the box face, so the box keeps one face referenced until it drops. Then both plates slide 5 mm sideways at that gap, then the arms back out along the **exact reverse of the place path**, torso standing back up: verified to be the reversed path offset a constant distance purely along the grip axis, zero perpendicular deviation.

The opening is stated as a **gap**, not as a clearance, because that is what the log prints and what the probe measures — ask for 270 and 270 is what you get, no model in between. The 2 mm hold *is* a clearance and so does depend on `--box-extra`. See §6: reading it as plate travel cost the first live release.

Which side gets the 8 mm and which way the slide goes is computed per cell, not fixed. They must agree: the 2 mm plate ends ~0.5 mm off the box, so sliding toward it would shove what was just placed.

**No push is performed.** Every box is dropped where the open-loop chain leaves it, so the seating error can be measured. That total error is what a push policy would have to close.

---

## 6. Known limits

### The first live release failed, and why (2026-08-07, r1c0)

The release leg ran to 100% and the measured gap opened the full commanded 10 mm — 245.3 → 255.2 mm, misalign 0.00°, lateral 0.0 mm, lag flat at 4.8 mm. The arms did exactly what they were told. **The box did not come free.**

The sizing was wrong, not the motion. The plates close to `--grip-gap` 245 on a 250 mm box, so at the grip **each plate is already 2.5 mm inside the box face it is pressing**. The back-offs were being applied as travel from where the plate
was:

| plate | travel | ends up |
|---|---|---|
| right | 8 mm | 5.5 mm off the box |
| left | 2 mm | **0.5 mm still pressed into it** |

A 2 mm back-off against a 2.5 mm half-squeeze never breaks contact. The requirement was "1 or 2 mm *away*"; the implementation gave "1 or 2 mm from where it is", and across a squeeze those differ in sign.

**Fixed** in two steps. First, clearances are now measured **from the box face** and `release_plan` adds the half-squeeze to get the travel. Second — because the computed face position is only as good as `--box-width` and the plate
calibration — the opening itself is now commanded as an **absolute gap** (`--release-to`, or `--release-open` on the router), which is the same quantity the log prints and the probe measures. Operating point is **270 mm on a 250 mm box**: 20.5 mm of travel on the free plate, 4.5 mm on the close one, which keeps its 2 mm of clearance.

`--box-width` stops being a reporting field either way: it still fixes where the close plate stops, so **a box wider than declared holds that plate against it**.

Pinned by two tests, including one that reproduces the old behaviour so it cannot come back.

### Finding the gap the box actually lets go at

The clearance above is computed from `--box-width`, the calibrated plate offset
and FK. All three carry error and **none of it is observable** — there is no
force sensor, so nothing in software can tell you the plates have stopped
touching. The number has to be measured once, on the box and plates you have.

It is cheap to measure, because a release is only an opening and `--carry-only`
starts from wherever the plates are *now*. Open a little, look; if it is still
held, run again wider. Nothing is re-gripped in between:

```bash
# plates are holding a box at 245 mm.  Open to 258 and look.
python3 tools/h1_auto_grip.py --carry-only --grip-gap 0.245 \
    --box-width 0.250 --release-gap 0.258

# still held?  again, wider — it opens further from where it stopped.
python3 tools/h1_auto_grip.py --carry-only --grip-gap 0.245 \
    --box-width 0.250 --release-gap 0.264
```

Use `--release-gap` (symmetric) while probing, so both plates leave together and the reading is the box's effective width rather than an artefact of which side let go first. The gap at which it drops, minus the clearance you want, is the width to use from then on — it absorbs the carton, the plates and the model in one number. That is the same trick the vertical datum uses: measure the thing that makes the unknowns cancel rather than deriving it.

Feed it back as `h1_pallet_routes --box-extra <measured − nominal>`, which raises the assumed width for both grips. Until that is right, "6 mm of clearance" is a computed number, not a real one.

### Other limits

**The cells pack flush, and the plate does not fit between two boxes.** 2 × 400 across 880 and 3 × 250 along 810 both put their slack at the *pallet edges*, nothing between boxes. The plate slab is 20 mm thick and sits outside
the contact face (`h1_with_ee.urdf` right_ee_link collision, faces at y = 0.060/0.080). With a 5 mm squeeze the contact face is 2.5 mm inside the box's nominal face, so the plate body ends up **~17.5 mm inside the neighbour's footprint while gripping**, ~19.5 mm after the release.

So **r2c0 and r0c1 cannot be seated at their nominal centres with a neighbour present.** They need a lateral drop offset plus a push. This is a property of the *descent*, not the retract — the retract adds 2 mm and the clear slide takes 5 mm back. Cells 1–4 have no neighbour on the plate axis and are unaffected.

**Recommendation: run cells 1–4 first and measure, before attempting 5 and 6.**

**No force feedback anywhere.** `/lowstate` hardcodes `dq` and `tau_est` to 0, so the only evidence a box is held is the commanded gap versus its measured width. `--grip-gap` is deliberately a few mm under the box width so closure ends in interference the hardware absorbs. An off-centre box gets shoved sideways rather than gripped; centring by eye at the open gate is the only defence.

**The usable carry envelope is set by the LEFT arm.** An unbiased 6-DoF walk away from the carry pose puts the slaved left plate out of reach ~69% of the time — left shoulder and wrist-yaw bind first. Not the arm the operator is driving.

**Carry travel is a narrow window.** Forward 0.10 m buys 3.7× the elbow room; about 0.15 m is where the arms cannot hold the box out any further. Raising the box instead makes it worse — at +0.05 the right wrist roll goes tight, at +0.10 the grip breaks. Re-measure after any change to the grip pose.

**Head pitch limits are reversed in the sim script's table.**
`hqt._TORSO_HEAD_LIMITS_HW[3]` is the MuJoCo row copied without the sign flip; clamping against it allows commanding 0.21 rad past the hardware max, which `h1_upper_body.set_joints` silently clips. `h1_real_teleop.py` derives its own  table; the sim script's copy is still wrong.

**Upright torso only** for the auto-grip — the operator's own constraint, and the
only regime the abort thresholds are calibrated in.

---

## 7. Running it

```bash
# plan only, no motion — reads the pallet transform and the live station list
python3 tools/h1_pallet_routes.py

# the arms need the ROS2 environment; the router inherits it
source ~/topstar_ros2/setup.sh

# one cell, no neighbours, full cycle
python3 tools/h1_pallet_routes.py --execute --cells r1c0

# print each h1_auto_grip command instead of running it
python3 tools/h1_pallet_routes.py --execute --no-arms

# arms alone, chassis already parked (what was run on 2026-08-06)
python3 tools/h1_auto_grip.py --grip-height 0.850 \
  --open-gap 0.350 --grip-gap 0.245 --box-width 0.250 \
  --carry-forward 0.10 --place-path config/h1_place_250end_850.json

# offline checks, no robot
python3 tools/h1_auto_grip.py --dry-run
python3 -m pytest tests/tools/ -q --noconftest
```

Keys inside `h1_auto_grip`: SPACE proceeds at each gate, `x` freezes a move (SPACE resumes), `q` quits. Quitting and Ctrl-C both stop publishing and leave the arms holding their last command, which is what you want mid-grip.

---

## 8. Next

1. **Re-test the chassis fine step with sole control of `.226`** and settle  whether the 3056 turn works. Blocks the first full cycle.
2. **Run cells 1–4**, one at a time, watching each gate.
3. **Measure where each box actually lands** against its cell centre. The park  error the router prints is only the chassis half; the place path and the  30 mm drop are the rest, and only a tape separates them.
4. **Size the push** from that measurement, then train the policy for r2c0 and  r0c1 — including the lateral drop offset those two need to clear their  neighbour on the way down.
5. Layer 1 is untouched on hardware.



