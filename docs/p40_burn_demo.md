# P40 SPS Burn Demonstration

A guided walk-through of the verb/noun keystroke sequence that ignites
the SPS engine for ~15 seconds in `agc-sim`. Companion to the
integration test `agc-test/tests/p40_sps_burn.rs`.

Audience: live demo of the host-side simulator, or a desk-walkthrough
of the V/N processor when the bench hardware is unavailable.

## Goal

Drive the AGC from FRESH START to the moment `hw.engine().sps_enable(true)`
is asserted, hold the engine on for about fifteen seconds, then observe
the AGC autonomously cut it off when the accumulated ΔV reaches the
target — with the simulator generating realistic PIPA pulses the entire
time.

## Simulator dynamics: how the IMU "comes alive"

`SimImu::read_pipa` is a thin destructive read of a pulse counter. By
itself the simulated IMU never produces motion — it would always return
`[0, 0, 0]` and the SERVICER would never observe any thrust.

The simulator's `agc-sim/src/physics.rs` module fills that gap with a
[`Spacecraft`] struct holding mass, SPS thrust and a thrust-direction
unit vector in the IMU platform frame. `SimHardware::tick(dt_seconds)`:

1. Reads `self.engine.thrusting`.
2. While the engine is on, integrates `Δv = (thrust / mass) × dir × dt`
   onto a per-axis sub-quantum residue.
3. Drains the residue as integer PIPA pulses (`PIPA_QUANTUM_M_S = 0.0585`
   m/s/count, equal to the AGC's nominal `PipaCalibration::NOMINAL` scale
   so no crew calibration is needed).
4. Saturating-adds those pulses into `SimImu::pipa`.

After the tick, the AGC's standard `hw.imu().read_pipa()` returns the
new pulses just as it would on real hardware. **No state-patching
gymnastics; the IMU just behaves.**

Default values (Apollo-CSM-like): mass 30 t, thrust 45 kN ⇒ 1.5 m/s²
acceleration. Tests configure `Spacecraft::thrust_dir_platform` to
match the LVLH axis their crew entry exercises.

## State seeded before the burn (entered via V71 / P27)

The real Apollo CMC received its state vector from Mission Control
through the digital uplink — but the uplink itself is just a stream of
V/N keystrokes that PINBALL processes identically to crew DSKY input
(see UPRUPT in `input/AGC Quick Reference.md`). So a state vector load
*is* a V/N sequence; the only difference between crew and ground is the
source of the keystrokes.

Comanche055 dedicates an entire program — **P27 "Update Liaison"** — to
this kind of load, with three extended verbs to drive it:

| Verb | Apollo description |
|---|---|
| **V70** | Update lift-off time (P27) |
| **V71** | Start AGC update; **block address** (P27) |
| V72 | Start AGC update; single address (P27) |
| V73 | Start AGC update; AGC time (P27) |

V71 is the right tool for a state-vector load because it walks the
operator (or the uplink) through three things in sequence: a starting
**address**, the **count** of words to load, and that many signed
**data words** committed at consecutive addresses.

Because the AGC-in-Rust state vector is stored as Rust struct fields
rather than at fixed AGC erasable ECADRs, the V71 address space is
re-mapped to a small simulator-specific table (see `p27_apply_word`
in `agc-core/src/services/v_n.rs`):

| Logical address | AGC field | Crew entry units |
|---|---|---|
| **1** | `state.csm_state.position[0]` | km (× 1000 on commit) |
| **2** | `state.csm_state.position[1]` | km |
| **3** | `state.csm_state.position[2]` | km |
| **4** | `state.csm_state.velocity[0]` | m/s |
| **5** | `state.csm_state.velocity[1]` | m/s |
| **6** | `state.csm_state.velocity[2]` | m/s |

Defaults that *do not* need crew entry:

| Field | Default | Why it just works |
|---|---|---|
| `state.refsmmat` | identity | LVLH along-track maps onto inertial +Y, so the crew's N81 along-track entry lands on the same axis the simulator thrusts along |
| `state.time` | `Met(0)` at FRESH START | Mission clock starts at zero |
| `state.pipa_cal` | `PipaCalibration::NOMINAL` (`scale = 0.0585`) | Matches the simulator's `PIPA_QUANTUM_M_S` so injected pulses scale correctly |
| `hw.spacecraft.thrust_dir_platform` | `[0, 1, 0]` | Aligns simulated SPS thrust with the +Y target ΔV |

`hw.spacecraft` itself has no on-bench analogue — it represents physics
that a real CSM would provide.

## The V/N sequence

For each step the table lists the keystrokes the operator types and the
AGC effect. Times are mission elapsed time (centiseconds), so
`Met(30 000)` is five minutes after MET zero.

### Step 1 — V71 P27 block update — load the full state vector

```
V 7 1 ENTR          ← begin P27 block update
1 ENTR              ← starting address (1 = position[X])
6 ENTR              ← word count        (load all six SV slots)
+ 6 7 7 8 ENTR      ← word @ addr 1: position[X] = +6778 km  (R_Earth + 400 km)
+ 0 ENTR            ← word @ addr 2: position[Y] = 0
+ 0 ENTR            ← word @ addr 3: position[Z] = 0
+ 0 ENTR            ← word @ addr 4: velocity[X] = 0
+ 7 6 6 9 ENTR      ← word @ addr 5: velocity[Y] = +7669 m/s  (≈ circular at 6 778 km)
+ 0 ENTR            ← word @ addr 6: velocity[Z] = 0
```

| AGC effect |
|---|
| PROG indicator changes to **27** (Update Liaison) for the duration of the load |
| DSKY shows **V21 N02** flashing while in P27Address / P27Count / P27Data phases — same cue the real AGC raised for "Specify address whole" |
| `state.csm_state.position = [6_778_000.0, 0.0, 0.0]` (km → m on commit) |
| `state.csm_state.velocity = [0.0, 7669.0, 0.0]` |
| `state.csm_state.frame = EarthInertial` |
| Phase returns to `Idle` after the sixth ENTR; PROG stays at 27 until the next V37 changes it |

After this single sequence the CSM is in a 400 km circular equatorial
orbit with position along inertial +X and velocity along inertial +Y.
The LVLH along-track axis (R1 of N81) now maps onto inertial +Y.

To verify, the operator can read components back via V06 N44 (apogee /
perigee / half-period) — N44 reconstructs the orbit from the freshly
loaded state vector:

```
V 0 6 N 4 4 ENTR    ← R1 = apogee (km), R2 = perigee (km), R3 = half-period (min)
```

For the 400 km circular LEO seeded above, R1 and R2 should both read
≈ 400 (km) and R3 ≈ 46 (min). Apollo's flight-time N44 used
nautical-miles and a `min:s` mixed format ("XXXX.X nmi" / "XXbXX min s")
— the simulator picks plain SI units (km, min) so the registers fit
the DSKY's 5-digit width for any LEO-to-HEO orbit. A real-flight unit
pass will re-encode this in the original Apollo formats once the per-noun
display-format spec lands.

The block load can also be split: V71E 1E 3E loads only the position
triple, V71E 4E 3E only the velocity triple, V71E 2E 1E only
position[Y], and so on. Out-of-range addresses or a count that would
overflow the address space raise OPR ERR and abort the sequence.

### Step 2 — (Optional) check current MET

```
V 1 6 N 6 5 ENTR    ← continuous monitor of mission elapsed time
```

This lets the operator see how much MET has accumulated during the
state-vector load — useful when picking a TIG that is comfortably in
the future. On real hardware the mission clock is also visible on the
panel beside the DSKY.

### Step 3 — Select P30 (External-ΔV targeting)

```
V 3 7 N 3 0 ENTR
```

| AGC effect |
|---|
| `state.major_mode = 30` |
| `state.dsky.prog = 30`, `noun = 33` (TIG entry cue), `flashing = true` |
| `state.pending_maneuver = None` (any stale solution is discarded) |

### Step 4 — Load TIG = 0 h 5 m 0.00 s (V25 N33)

```
V 2 5 N 3 3 ENTR
0 ENTR              ← R1 = hours
5 ENTR              ← R2 = minutes
0 ENTR              ← R3 = seconds × 100
```

| AGC effect |
|---|
| `state.vn.pending_tig = Some(Met(30_000))`  (5 × 6000 cs) |

Five minutes is far enough into the future that even a leisurely human
typing pace on the dsky_sim console will not cause the **TIG-in-past**
alarms (210 from P30, 225 from P40) to fire. If you have already burned
several minutes during the demo, just bump the figure: enter
`0 / 10 / 0` for ten minutes ahead, or whatever reading on V16 N65 plus
a comfortable buffer.

### Step 5 — Load LVLH ΔV = +21 along-track, 0 radial, 0 cross (V25 N81)

```
V 2 5 N 8 1 ENTR
+ 2 1 ENTR          ← R1 = along-track (S-axis)
+ 0 ENTR            ← R2 = radial      (R-axis)
+ 0 ENTR            ← R3 = cross-track (W-axis)
```

| AGC effect |
|---|
| `noun_81_commit_dv_lvlh` consumes `pending_tig` |
| `p30_load_dv_lvlh` re-orders crew[X,Y,Z] → RSW[Y,X,Z] and calls `apply_external_delta_v` |
| `state.pending_maneuver = Some(maneuver)` with `target_dv_inertial ≈ [0, 21, 0]` (m/s) and `tig = Met(30 000)` |
| DSKY shows V06 N45 burn-summary readback |

### Step 6 — Select P40 (SPS thrust program)

```
V 3 7 N 4 0 ENTR
```

| AGC effect |
|---|
| `validate_pending_maneuver` succeeds (TIG in future, ΔV ≥ 0.5 m/s) |
| `engage_burn` transfers the maneuver into `state.burn`, installs `burn_servicer_exit`, and calls `start_servicer` |
| `dap_init(state, DapMode::Maneuver)` schedules `dap_step` |
| `state.major_mode = 40`, DSKY shows flashing **V50 N99** (engine-arm request) |
| **`state.engine_thrusting` is still `false`** — ignition awaits crew acknowledgement |

### Step 7 — PRO key arms the engine for ignition at TIG

```
PRO
```

| AGC effect |
|---|
| `p40_arm_engine` runs: `state.burn.armed = true`, TVC filter pre-warmed at current trim, V50 N99 cleared |
| Display switches to **V16 N40** (continuous burn-status monitor): R1 = target ΔV, R2 = accumulated ΔV, R3 = remaining ΔV. The dsky_sim render loop refreshes these registers each frame so the operator watches R2 climb toward R1 once the burn starts. |
| `state.dsky.flashing = false` |
| **`state.engine_thrusting` stays `false`.** PRO is the *arming* action; the SPS-enable discrete is held off until TIG to avoid burning ΔV early. |

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

This matches the real Apollo TIG-countdown procedure: the crew arms
the engine in the last few seconds before TIG; the AGC commands actual
ignition automatically when the mission clock reaches `tig`.

## Burn execution and autonomous cutoff

Once the engine is armed, the dsky_sim render loop runs a "soft
executive" that mirrors just enough of `agc_core::executive::Executive::run`
to drive the burn to completion:

```rust
hw.tick(dt);                                 // physics: thrust → PIPA pulses
pump_pipa_into_state(&mut state, &mut hw);   // hw.imu.pipa → state.pipa_counts
waitlist_pump.tick(&mut state, &mut hw);     // dispatch dap_step / servicer_task
pump_engine_to_hw(&state, &mut hw);          // engine_thrusting → hw.engine
pump_rcs_to_hw(&mut state, &mut hw);         // RCS jet bitmask → hw.rcs
```

`waitlist_pump` is a small TIME3-style countdown over `state.waitlist`
that fires `dap_step` every 100 ms and `servicer_task` every 2 s. Each
SERVICER cycle:

1. Scales raw counts by `pipa_cal.scale = 0.0585 m/s/count`.
2. Applies REFSMMAT (identity here ⇒ platform = inertial).
3. Integrates the state vector and advances `state.time` by 200 cs.
4. Stages `state.servicer_last_dv_inertial`.
5. Calls the `burn_servicer_exit` hook, which:
   - adds the cycle's ΔV onto `state.burn.accumulated_dv_inertial`,
   - tests `|accumulated| ≥ |target| − 0.3`.

Cycle-by-cycle accumulator (default 1.5 m/s² simulator acceleration,
3.0 m/s of Δv per 2-second cycle ≈ 51 PIPA pulses):

| Cycle | t (s) since ignition | accumulated ΔV (m/s) | ≥ 21 − 0.3 ? |
|---|---|---|---|
| 1 | 2 | ≈ 2.98 | no |
| 2 | 4 | ≈ 5.97 | no |
| 3 | 6 | ≈ 8.95 | no |
| 4 | 8 | ≈ 12.00 | no |
| 5 | 10 | ≈ 14.98 | no |
| 6 | 12 | ≈ 17.96 | no |
| **7** | **14** | **≈ 20.94** | **yes — cutoff** |

(The slight under-counting is the PIPA quantization: 358 integer pulses
across 7 cycles × 0.0585 m/s = 20.943 m/s, with a fractional residue
carried inside `Spacecraft::pipa_residue_m_s` that would be released on
later cycles.)

On cycle 7 `burn_servicer_exit`:

- clears `state.burn.burn_active`,
- clears `state.engine_thrusting`,
- drops the SERVICER hook (`servicer_exit = None`),
- transitions the DAP to `AttitudeHold`.

On the following `apply_engine_staging` call, `hw.engine.thrusting`
returns to `false`. The engine has fired for **14 s** of mission
elapsed time — within the "about 15 s" demonstration target, with the
remainder of the slop given to the 0.3 m/s ΔV cutoff tolerance and the
2-second SERVICER granularity.

## Demonstration tips

- Keep the V/N processor's flashing indicator in mind: V25 N33 leaves
  the display flashing until R3 is loaded, V50 N99 flashes until PRO
  is pressed. If the flash stops unexpectedly, an OPR ERR or an alarm
  has fired — check `state.alarm.code`.
- The interactive simulator (`cargo run -p agc-sim --bin dsky_sim`)
  reflects every staging field on the panel: PROG/VERB/NOUN, the
  R1/R2/R3 registers, the SPS-thrusting indicator, and the gimbal
  angles. Its main loop runs the soft executive
  (`hw.tick` + `pump_pipa_into_state` + `WaitlistPump::tick`
  + `pump_engine_to_hw`) on every frame, so the burn proceeds
  autonomously once the operator presses PRO — no debugger, no scripted
  ticks. R2 of the post-PRO V16 N40 monitor climbs from 0 toward 21 over
  ≈ 14 seconds, then the SPS lamp drops when the SERVICER hits cutoff.
- If the demo audience asks "why exactly 14 s and not 15 s?": because
  the SERVICER is a 2-second cadence task and the engine cuts off at
  the end of the cycle that crosses the ΔV target. Picking a target
  that lands midway between cycles (e.g. 22 m/s) would push cutoff to
  cycle 8 = 16 s — the asymmetry is intrinsic to the AGC's averaging
  design.
- Knobs the demo can twist:
  - **Crew-side**: change the position or velocity words in the V71
    block load to demonstrate other orbits (e.g. a 800 km circular
    orbit needs `+7186` km at addr 1 and `+7459` m/s at addr 5), or
    change the N81 ΔV magnitude to lengthen / shorten the burn.
  - **Sim-side** (not crew-reachable): `hw.spacecraft.sps_thrust_n` to
    vary acceleration, `hw.spacecraft.thrust_dir_platform` to send the
    burn down a different inertial axis (pair with a matching crew
    entry on N81).

## File pointers

- Test: `agc-test/tests/p40_sps_burn.rs`
- Simulator dynamics: `agc-sim/src/physics.rs`
- SimHardware tick wiring: `agc-sim/src/hardware.rs`
- P40 implementation: `agc-core/src/programs/p40_p41.rs`
- P30 targeting and crew load: `agc-core/src/programs/p30.rs`
- V/N processor (key dispatch, V25/V37/V50): `agc-core/src/services/v_n.rs`
- SERVICER cycle and PIPA pipeline: `agc-core/src/services/average_g.rs`
- Burn state machine and cutoff: `agc-core/src/guidance/maneuver.rs`
- Engine staging helper (executive-side, mirrored by the test): `agc-core/src/executive/scheduler.rs`
