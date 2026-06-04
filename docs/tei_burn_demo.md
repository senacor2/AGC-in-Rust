# TEI Burn Demonstration

A guided walk-through of the verb/noun keystroke sequence that fires
the SPS engine for the Apollo 8 **trans-Earth injection** (TEI) burn
out of a 111 km lunar parking orbit. Companion to the integration test
`agc-test/tests/phase_tei.rs`.

Audience: live demo of the host-side simulator, or a desk-walkthrough
of the V/N processor when the bench hardware is unavailable.

This demo is the lunar-orbit sibling of [`p40_burn_demo.md`](p40_burn_demo.md).
Where the P40 demo seeds an idealised Earth orbit and fires a small
21 m/s ΔV, the TEI demo places the spacecraft in lunar parking orbit
and executes the historical 1073 m/s prograde burn that sent Apollo 8
home. The V/N sequence is identical; only the state-vector seed and
the magnitude of the ΔV differ.

## Goal

Drive the AGC from FRESH START + lunar-orbit seed to the moment
`hw.engine().sps_enable(true)` is asserted, hold the engine on for the
~5 minutes 53 s of an Apollo 8 TEI burn, and watch the AGC autonomously
cut off the engine when the accumulated inertial ΔV reaches 1073 m/s.

If the live audience can't wait 6 minutes for the burn to complete, the
crew can pick a smaller N81 ΔV (e.g. +21 m/s like the P40 demo) — the
V/N sequence is identical, only the cutoff arrives sooner.

## Simulator dynamics: the SERVICER under Moon gravity

The SERVICER (`average_g_step` in `agc-core/src/navigation/integration.rs`)
dispatches gravity computation by frame:

| `state.csm_state.frame` | Gravity source | Third-body perturbation |
|---|---|---|
| `EarthInertial` | `earth_gravity` (µ_E / r²) | Moon at `moon_pos` |
| `MoonInertial` | `moon_gravity` (µ_M / r²) | Earth at `−moon_pos` |

The V71 P27 uplink protocol (`p27_apply_word` in
`agc-core/src/services/v_n.rs`) forces the frame to `EarthInertial` on
every position / velocity word, so a state vector loaded at lunar-orbit
altitude (~1848 km from origin) would have the SERVICER trying to
evaluate Earth gravity well below `R_EARTH` — nonsense.

For this demo, an extra logical address — **31 = gravity-body selector**
(added with #61) — was wired into P27 so an uplink script can switch
the frame to `MoonInertial` after the state-vector load:

| Value @ address 31 | Effect |
|---|---|
| `1` | `state.csm_state.frame = Frame::EarthInertial` |
| `2` | `state.csm_state.frame = Frame::MoonInertial` |

`scripts/tei_demo.dsky` issues both V71 blocks back-to-back — the
six-word state vector then the single-word frame switch — so the SERVICER
correctly propagates the spacecraft under lunar gravity once the burn
starts.

## State seeded before the burn

Run the prep script from dsky_sim with the `@` key:

```text
scripts/tei_demo.dsky
```

The script does two things and **nothing else** — by design, the crew
sequence below must be typed by the operator at the DSKY so the
audience can watch each keystroke land:

1. **V71 P27 block update, addresses 1..6** — six-word state-vector
   load placing the CSM at `[+1848 km, 0, 0]` with velocity
   `[0, +1629 m/s, 0]`. Round numbers: `R_MOON + 111 km` ≈ 1848 km;
   `√(µ_M / r)` at r = 1.848 × 10⁶ m ≈ 1629 m/s (prograde, +Y).
2. **V71 P27 block update, address 31 = 2** — gravity-body selector
   to `MoonInertial`.

Status row should read `Script loaded: ... (33 words queued)` and the
DSKY position registers reflect the lunar orbit. The PROG indicator
shows **27** during the load and stays at 27 (Update Liaison) afterwards
— normal, the major mode advances on the next V37.

## The V/N sequence (typed manually)

For each step the table lists the keystrokes the crew types and the
AGC effect. Times are mission elapsed time (centiseconds), so
`Met(6000)` is one minute after MET zero.

### Step 1 — (Optional) verify the lunar-orbit seed

```
V 0 6 N 4 4 ENTR
```

The N44 readback should show apogee / perigee both near **111 km**
(R1 ≈ R2). Half-period (R3) reads ≈ 59 min, the period of a low lunar
orbit. If the numbers don't match, the script load didn't take —
check `state.alarm.code` and re-load.

### Step 2 — Select P30 (External-ΔV targeting)

```
V 3 7 ENTR  ← request major-mode change
3 0 ENTR    ← MM = 30
```

| AGC effect |
|---|
| `state.major_mode = 30` |
| `state.dsky.prog = 30`, `noun = 33` (TIG entry cue), `flashing = true` |
| `state.pending_maneuver = None` (any stale solution is discarded) |

### Step 3 — Load TIG = 0 h 1 m 0.00 s (V25 N33)

```
V 2 5 N 3 3 ENTR
0 ENTR              ← R1 = hours
1 ENTR              ← R2 = minutes
0 ENTR              ← R3 = seconds × 100
```

| AGC effect |
|---|
| `state.vn.pending_tig = Some(Met(6_000))`  (1 × 6000 cs) |

One minute is far enough into the future that even an unhurried human
typing pace on the dsky_sim console will not cause the **TIG-in-past**
alarms (210 from P30, 225 from P40) to fire. If you take longer to
load the V/N sequence, bump it: `0 / 2 / 0` for two minutes ahead, or
whatever reading on V16 N65 plus a comfortable buffer.

### Step 4 — Load LVLH ΔV (V25 N81)

```
V 2 5 N 8 1 ENTR
+ 1 0 7 3 ENTR      ← R1 = along-track (S-axis) = +1073 m/s
+ 0 ENTR            ← R2 = radial      (R-axis)
+ 0 ENTR            ← R3 = cross-track (W-axis)
```

The 1073 m/s magnitude comes from Apollo 8 Mission Report
MSC-PA-R-69-1 Table 3-I (3522 ft/s post-flight reconstruction); the
historical TEI was a prograde impulsive burn parallel to the velocity
vector. Because the lunar parking orbit was seeded with velocity
along **+Y**, the LVLH +S axis (along-track) maps to inertial +Y here,
so a positive R1 entry produces a prograde inertial ΔV.

| AGC effect |
|---|
| `noun_81_commit_dv_lvlh` consumes `pending_tig` |
| `p30_load_dv_lvlh` re-orders crew[X,Y,Z] → RSW[Y,X,Z] and calls `apply_external_delta_v` |
| `state.pending_maneuver = Some(maneuver)` with `target_dv_inertial ≈ [0, 1073, 0]` (m/s) and `tig = Met(6_000)` |
| DSKY shows V06 N45 burn-summary readback |

**Knob to twist for time-constrained demos:** type `+ 2 1 ENTR` here
instead of `+ 1 0 7 3 ENTR`. The burn then completes in ~14 s
(matching `p40_burn_demo.md`) rather than ~6 minutes — at the cost of
no longer matching the historical TEI ΔV.

### Step 5 — Select P40 (SPS thrust program)

```
V 3 7 ENTR  ← request major-mode change
4 0 ENTR    ← MM = 40
```

| AGC effect |
|---|
| `validate_pending_maneuver` succeeds (TIG in future, ΔV ≥ 0.5 m/s) |
| `engage_burn` transfers the maneuver into `state.burn`, installs `burn_servicer_exit`, and calls `start_servicer` |
| `dap_init(state, DapMode::Maneuver)` schedules `dap_step` |
| `state.major_mode = 40`, DSKY shows flashing **V50 N99** (engine-arm request) |
| **`state.engine_thrusting` is still `false`** — ignition awaits crew acknowledgement |

### Step 6 — PRO key arms the engine for ignition at TIG

```
PRO
```

| AGC effect |
|---|
| `p40_arm_engine` runs: `state.burn.armed = true`, TVC filter pre-warmed at current trim, V50 N99 cleared |
| Display switches to **V16 N40** (continuous burn-status monitor): R1 = target ΔV, R2 = accumulated ΔV, R3 = remaining ΔV. The dsky_sim render loop refreshes these registers each frame so the operator watches R2 climb toward R1 once the burn starts. |
| `state.dsky.flashing = false` |
| **`state.engine_thrusting` stays `false`.** PRO is the *arming* action; the SPS-enable discrete is held off until TIG. |

Now wait for TIG. Each `dap_step` cycle (every 100 ms) runs an
**ignition gate** that checks `state.burn.armed && state.time >= burn.tig`
— and the moment the mission clock reaches TIG, the gate fires:

| AGC effect at the first dap_step after `state.time >= burn.tig` |
|---|
| `state.engine_thrusting = true` |
| `state.dap_state.mode = Tvc` (TVC steering takes over from Maneuver) |
| `state.burn.armed = false` (gate is one-shot) |

`pump_engine_to_hw` mirrors `engine_thrusting` to `hw.engine.sps_enable(true)`
on the next render frame. The propulsion-panel SPS-thrusting lamp lights;
the simulator's `Spacecraft` integrates Δv on the next `hw.tick(dt)` call.

## Burn execution and autonomous cutoff

With the default `apollo_csm` simulator constants (mass 30 000 kg,
SPS thrust 91 188 N) the acceleration is `91 188 / 30 000 ≈ 3.04 m/s²`.

The SERVICER fires every 2 s, accumulating `~6.08 m/s` of inertial ΔV
per cycle. The cutoff condition is `|accumulated| ≥ |target| − 0.3`:

```
ceil((1073 − 0.3) / 6.08) ≈ 177 SERVICER cycles
                          ≈ 354 s
                          ≈ 5 min 54 s
```

On cycle 177 `burn_servicer_exit`:

- clears `state.burn.burn_active`,
- clears `state.engine_thrusting`,
- drops the SERVICER hook (`servicer_exit = None`),
- transitions the DAP to `AttitudeHold`.

On the following `apply_engine_staging` call, `hw.engine.thrusting`
returns to `false`. The engine has fired for **~5 min 54 s** of mission
elapsed time — closely matching Apollo 8's actual TEI burn duration of
3 min 24 s (the simulator's 3.04 m/s² is conservative; Apollo 8 burned
its lower-mass post-LOI stack with higher specific impulse).

After cutoff the spacecraft is on a hyperbolic departure trajectory
from the Moon (`ε_MCI > 0`, `e > 1`) — readable via V06 N44 against the
post-burn state.

## Demonstration tips

- **Pre-load the script before going on stage.** The state-vector
  reseed via V71 alone takes ~30 keystrokes through the uplink FIFO,
  enough to make the audience restless. Press `@`, type
  `agc-sim/scripts/tei_demo.dsky`, ENTER, and the orbital seed lands
  in under a second. Then start the manual V/N sequence.
- **Watch V25 N33's R3 register format.** It is "seconds × 100" — so
  `0 ENTR` at R3 means *0 seconds*, not 0.01 seconds. For TIG at a
  fractional second, type the centisecond value: e.g. `1 5 0 0` for
  15.00 s.
- The **R2 / R1 reading on V16 N40** is the live progress bar. Watch
  it climb from 0 toward 1073 over the burn — that's the SERVICER
  integrating PIPA pulses cycle by cycle. The simulator's flashing
  ACTY indicator (top-right of the dsky_sim panel) blinks once per
  SERVICER fire, ~every 2 s.
- If `state.alarm.code` lights mid-burn, the most likely culprits are
  **alarm 222** (no PRO before TIG — engine wasn't armed) or
  **alarm 225** (TIG fell into the past during the V/N load). The
  doc's TIG = MET + 1 min was chosen with both in mind; if the load
  takes longer just pick a later TIG.
- Knobs the demo can twist:
  - **Crew-side**: change the N81 R1 ΔV to lengthen / shorten the burn
    (1073 → 21 makes it the 14-s P40 demo). Change the V71 position /
    velocity words to demonstrate other lunar orbits (a 100 nm circular
    orbit needs `+1923` km at addr 1 and `+1596` m/s at addr 5).
  - **Sim-side** (not crew-reachable): `hw.spacecraft.sps_thrust_n` to
    bump acceleration (the historical Apollo 8 TEI burn was only 3 min
    24 s because the post-LOI stack was lighter).

## Why this demo is more useful than P40

Apollo's TLI was a Saturn V S-IVB burn — the CMC was a passive
*observer*. `docs/p40_burn_demo.md` exercises the SPS+DAP+SERVICER
pipeline in isolation but does not correspond to any real Apollo
event. The TEI, by contrast, *was* a CMC-driven SPS burn: P30 + P40
ran the actual sequence demonstrated here, producing the actual ΔV
that sent Apollo 8 home. Walking an audience through the TEI demo
shows the AGC doing something it really did, not a synthetic
exercise.

## File pointers

- Prep script: `agc-sim/scripts/tei_demo.dsky`
- Test mirroring the same burn (with hidden ground-truth oracle):
  `agc-test/tests/phase_tei.rs`
- Lunar-orbit state-vector helpers: `agc-test/tests/phase_tei.rs::pre_tei_sv`
- P30 implementation: `agc-core/src/programs/p30.rs`
- P40 implementation: `agc-core/src/programs/p40_p41.rs`
- V/N processor (V25/V37/V50, V71 P27 with gravity-body selector at
  address 31): `agc-core/src/services/v_n.rs`
- SERVICER under lunar gravity:
  `agc-core/src/navigation/integration.rs::total_gravity`
- Burn state machine and cutoff: `agc-core/src/guidance/maneuver.rs`
