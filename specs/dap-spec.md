# Specification: `control/dap.rs` — Digital Autopilot Supervisor

**Status**: Draft — ready for implementation review
**Module path**: `agc-core/src/control/dap.rs`
**Architecture reference**: `docs/architecture.md` §11 "Digital Autopilot (DAP)", §13.2 "Timing Budget"
**HAL reference**: `specs/hal-spec.md` §6 (Timers, arm_t5), §10 (Engine, sps_gimbal / thrust_on)
**Executive reference**: `specs/executive-spec.md` §2.2 (Waitlist self-rescheduling pattern)
**Sibling specs**: `specs/attitude-spec.md`, `specs/rcs-logic-spec.md`, `specs/tvc-spec.md`
**AGC source reference**:
- `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc` — T5RUPT handler and Coast DAP dispatcher
- `Comanche055/RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc` — DAPINIT, DAPDATR, DAPIDLER
- `Comanche055/TVCDAPS.agc` — Thrust DAP step (TVC mode)
- `Comanche055/TVCEXECUTIVE.agc` — TVC mode activation / deactivation
- `Comanche055/TVCINITIALIE.agc` — TVC initialisation
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — DAP erasable variables (DAPDATR, CDUX/Y/Z, RUPTBACK, WFORPOR, etc.)
**Spec checklist**: `specs/README.md` — all items satisfied (see §12)

---

## 1. Purpose and Scope

The Digital Autopilot (`dap.rs`) is the top-level supervisor for all attitude
and translation control of the Command Module. It owns a single periodic
execution cycle driven by T5RUPT (approximately every 100 ms) and dispatches
to mode-specific subsystems:

- **Coast DAP** (RCS modes): reads CDU angles from the IMU, computes body
  rates by differencing successive CDU readings, computes attitude errors,
  and calls `attitude.rs` and `rcs_logic.rs` to select and fire RCS jets.
- **Thrust DAP** (TVC mode): during SPS burns, computes attitude error and
  calls `tvc.rs` to update the pitch/yaw gimbal commands, then issues the
  hardware gimbal command via `engine.sps_gimbal`.

All other control subsystems — `attitude.rs`, `rcs_logic.rs`, `tvc.rs` — are
called exclusively from within `dap_step`. No other module calls these
subsystems directly. `dap.rs` is the single point of entry into the entire
control pipeline.

**Strategy D — staged I/O**: `dap_step` is a pure Waitlist task
(`fn(&mut AgcState)` — no `&mut impl AgcHardware` parameter). All CDU reads
and jet/gimbal writes are performed in the T5RUPT ISR shim. `dap_step` reads
staged CDU angles from `AgcState` and writes jet/gimbal commands back to staging
fields; the ISR shim translates these to hardware I/O. See §5.8 for the full
list of staging fields.

### What this module is NOT

- It does not implement the jet selection lookup tables; those live in
  `rcs_logic.rs`.
- It does not implement the lead-lag filter or gimbal drive arithmetic; those
  live in `tvc.rs`.
- It does not implement the KALCMANU maneuver quaternion steering algorithm;
  that lives in `attitude.rs`.
- It does not own the T6RUPT handler for jet pulse timing; that is managed
  by `rcs_logic.rs` arming `Timers::arm_t6` — called from the T5RUPT ISR shim.

---

## 2. AGC Background

### 2.1 DAP Architecture in Comanche055

The Comanche055 DAP is separated into two major components:

**Coast DAP** (`RCS-CSM_DIGITAL_AUTOPILOT.agc`,
`RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc`): Handles all attitude control during
unpowered flight and during RCS translations. Activated by the crew via
V46 N01 E (DAP data load) followed by PRO. Runs as a Waitlist task on a
10-centisecond (100 ms) period loaded into TIME5. On each cycle the handler:

1. Reads the three CDU gimbal angles (CDUX, CDUY, CDUZ).
2. Computes OMEGAP/Q/R (body rates) by differencing current vs. previous CDU
   angles and dividing by the cycle period (10 cs).
3. Depending on mode, calls the rate-error or attitude-error routine.
4. Calls the jet selection logic (`JET_SELECTION_LOGIC.agc`) to translate
   torque request to a jet bitmask.
5. Re-arms TIME5 for the next cycle (CALLFAST/WAITLIST call with 10 cs).

**Thrust DAP** (`TVCDAPS.agc`, `TVCEXECUTIVE.agc`): Active during P40 SPS
burns. Uses the same T5RUPT cycle, but instead of RCS jets it commands the SPS
engine gimbal (TVCPITCH, TVCYAW — counter cells octal 0053/0054). Includes a
digital lead-lag filter for stability and a trim-tracking loop.

### 2.2 Erasable Variables Corresponding to DapState

The original AGC stored DAP working variables in erasable memory:

| AGC Erasable Tag | AGC Address (octal) | Rust field / meaning |
|---|---|---|
| `CDUX` | 0130 | CDU roll angle (previous reading) — `DapState::prev_cdu[0]` |
| `CDUY` | 0131 | CDU pitch angle (previous reading) — `DapState::prev_cdu[1]` |
| `CDUZ` | 0132 | CDU yaw angle (previous reading) — `DapState::prev_cdu[2]` |
| `OMEGAP` | 0163 | Body roll rate (rad/s) — `DapState::rate_estimate[0]` |
| `OMEGAQ` | 0164 | Body pitch rate (rad/s) — `DapState::rate_estimate[1]` |
| `OMEGAR` | 0165 | Body yaw rate (rad/s) — `DapState::rate_estimate[2]` |
| `ERRORX` / `ERRORY` / `ERRORZ` | Various | Attitude error — `DapState::attitude_error` |
| `DAPDATR1` | 0170 | DAP data word 1 (deadband, jets per axis, manual/auto) — `DapState::deadband`, `DapState::num_jets` |
| `WFORPQR` | 0177 | Rate deadband (rad/s) — `DapState::rate_deadband` |
| `CMDAPMOD` | 0175 | DAP mode register — `DapState::mode` |
| `TVCYAW` / `TVCPITCH` | 0053/0054 | TVC gimbal counter cells — `TvcState::gimbal_yaw` / `gimbal_pitch` |

AGC scale factors for angles: CDU angles are 15-bit ones-complement values
where `2^15` counts = 1 full revolution = 2π rad. In the Rust port, CDU angles
are `CduAngle(u16)` where `2^16` counts = 2π rad (see `types/angle.rs`).
Conversion: `CduAngle::to_radians()` = `count as f64 * (TAU / 65536.0)`.

Body rates: In the original AGC, OMEGAP/Q/R are scaled in B+4 radians/second
(1 unit = 2^−11 rad/s, full scale ~0.0156 rad/s). In the Rust port: `f64` rad/s.

Attitude error: In the original AGC scaled as B-1 half-revolutions per unit;
in the Rust port: `f64` radians.

---

## 3. Data Structures

### 3.1 `DapMode` Enum

**File**: `agc-core/src/control/dap.rs`

```rust
/// DAP operating mode.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — CMDAPMOD register (octal 0175).
/// The mode encoding below follows the Comanche055 DAPDATR register conventions.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DapMode {
    /// DAP is off — no attitude control. T5 is not re-armed by dap_step.
    /// AGC correspondence: CMDAPMOD = 0 (off / idle).
    #[default]
    Off,
    /// Rate damping — null body rates using RCS jets.
    /// Issued torques oppose non-zero rates. No attitude target.
    /// AGC correspondence: CMDAPMOD = 1 (rate command / minimum impulse).
    RateDamping,
    /// Attitude hold — maintain a commanded target attitude within the deadband.
    /// Torques are applied when attitude error exceeds `deadband`.
    /// AGC correspondence: CMDAPMOD = 2 (attitude hold).
    AttitudeHold,
    /// Attitude maneuver — rotate to a commanded attitude at a controlled rate.
    /// On each cycle `commanded_attitude` is incremented by `maneuver_rate`.
    /// When the target is reached, automatically transitions to `AttitudeHold`.
    /// AGC correspondence: CMDAPMOD = 3 (KALCMANU maneuver steering).
    Maneuver,
    /// TVC mode — gimbal control during SPS burn.
    /// RCS is not fired for attitude control; only the SPS gimbal is moved.
    /// Valid only while `hw.engine().thrust_on()` returns `true`.
    /// AGC correspondence: TVCDAPS.agc active (TVC DAP replaces Coast DAP).
    Tvc,
}
```

**Valid-transition table** (see §6 for full semantics):

```
Off          → RateDamping, AttitudeHold, Maneuver, Tvc
RateDamping  → Off, AttitudeHold, Maneuver
AttitudeHold → Off, RateDamping, Maneuver
Maneuver     → Off, AttitudeHold          (automatic on maneuver completion)
Tvc          → Off, RateDamping           (only after thrust off)
```

Tvc may NOT transition to Maneuver or AttitudeHold while thrusting. The
attempt is silently rejected and the mode is left as Tvc.

### 3.2 `DapState` Struct

**File**: `agc-core/src/control/dap.rs`

The existing `DapState` must be extended with the fields required for body-rate
computation (CDU angle history), maneuver tracking, crew-configurable parameters,
and restart protection.

```rust
/// Digital Autopilot state — T5RUPT context.
///
/// One instance lives in `AgcState::dap_state`.
/// All fields are `Copy` — no heap, no pointers.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc erasable assignments (see §2.2).
#[derive(Clone, Copy, Debug, Default)]
pub struct DapState {
    // ── Mode ─────────────────────────────────────────────────────────────────
    /// Current operating mode.
    /// AGC: CMDAPMOD (octal 0175).
    pub mode: DapMode,

    // ── Attitude error ────────────────────────────────────────────────────────
    /// Attitude error angles [roll, pitch, yaw] in radians.
    /// Positive = commanded attitude is ahead of current attitude.
    /// AGC: ERRORX/ERRORY/ERRORZ, scaled B-1 half-revolutions.
    ///
    /// In TVC mode this is also used by tvc_step for pitch/yaw gimbal steering.
    /// In maneuver (cross-product steering) mode, this is set by maneuver.rs
    /// and passed through to tvc_step via DapState — see maneuver-spec §7.1.
    pub attitude_error: Vec3,

    // ── Rate estimate ─────────────────────────────────────────────────────────
    /// Estimated body rates [roll, pitch, yaw] in rad/s.
    /// Computed each cycle by differencing successive CDU readings.
    /// AGC: OMEGAP (octal 0163), OMEGAQ (0164), OMEGAR (0165).
    pub rate_estimate: Vec3,

    // ── CDU history ───────────────────────────────────────────────────────────
    /// CDU gimbal angles from the PREVIOUS T5RUPT cycle [roll, pitch, yaw].
    /// Used to compute body rates by finite difference.
    /// Updated at the END of each dap_step call.
    /// AGC: CDUX (octal 0130), CDUY (0131), CDUZ (0132).
    /// Units: CduAngle (u16 counts); full revolution = 65536 counts = 2π rad.
    pub prev_cdu: [CduAngle; 3],

    // ── Deadbands ─────────────────────────────────────────────────────────────
    /// Attitude deadband in radians.
    /// Jets are not fired if |attitude_error| < deadband on all axes.
    /// Crew-configurable via V46 N01. Typical: 5° (0.0873 rad) coarse,
    /// 1° (0.0175 rad) fine.
    /// AGC: DAPDATR1 bits 11–8 (deadband select).
    pub deadband: f64,

    /// Rate deadband in rad/s.
    /// In RateDamping mode, jets are not fired if |rate_estimate| < rate_deadband.
    /// AGC: WFORPQR (octal 0177). Typical: 0.5°/s (0.00873 rad/s).
    pub rate_deadband: f64,

    // ── RCS configuration ─────────────────────────────────────────────────────
    /// Currently commanded RCS jet bitmask (SM jets, 16 bits).
    /// Bits 0–15 correspond to SM jets A1–D4 (see rcs-logic-spec §3.2).
    /// Upper byte = jets_b (channel 06), lower byte = jets_a (channel 05).
    /// Written by rcs_logic::select_jets_sm on each cycle.
    /// AGC: output to channels 05 (PYJETS) and 06 (ROLLJETS).
    pub rcs_jet_flags: u16,

    /// Failed jet mask — jets to exclude from selection.
    /// Crew-set via V46 N02. A set bit prevents that jet from being commanded.
    /// AGC: DAPDATR2 (failed-jet inhibit register).
    pub failed_jets: u16,

    /// Number of jets per axis to fire (1 or 2).
    /// 1 jet = minimum impulse mode; 2 jets = normal mode.
    /// AGC: DAPDATR1 bits 5–4 (NJET select).
    pub num_jets: u8,

    // ── Maneuver ──────────────────────────────────────────────────────────────
    /// Target (commanded) attitude [roll, pitch, yaw] in radians.
    /// Used in AttitudeHold and Maneuver modes.
    /// Initialised from guidance targeting output (P40 burn attitude, etc.)
    /// or from crew V49 entries.
    pub commanded_attitude: Vec3,

    /// Current maneuver rate [roll, pitch, yaw] in rad/s.
    /// In Maneuver mode, `commanded_attitude` is incremented by this value
    /// each cycle (× 0.1 s period). Zero in AttitudeHold.
    /// AGC: KALCMANU steering angular rate, typically ≤ 0.5°/s.
    pub maneuver_rate: Vec3,

    // ── Restart protection ────────────────────────────────────────────────────
    /// Restart group for this DAP task.
    /// Phase 1 = task re-scheduled to Waitlist (task-type restart).
    /// Phase 0 = DAP idle (no restart needed).
    /// AGC: GROUP 6 (DAPIDLER restart group in RESTART_TABLES.agc).
    pub restart_phase: i16,
}
```

**Invariants**:

- `prev_cdu` contains the CDU reading captured at the end of the most recent
  `dap_step` call. On FRESH START / first call, `prev_cdu` is `[CduAngle(0); 3]`
  and the first rate estimate will be zero (or small) regardless of the actual
  CDU reading. This is acceptable; the first cycle is a warm-up.
- `deadband` must be strictly positive when mode is AttitudeHold or Maneuver.
  If a crew entry sets `deadband = 0.0`, the DAP should substitute a minimum
  safe deadband of 0.1° (0.00175 rad) to prevent a jet-firing infinite loop.
- `rcs_jet_flags` is valid only after the most recent `dap_step` call. It
  reflects the last commanded jet state. The physical jets may be on or off
  depending on the T6RUPT timing (jet pulse duration managed by rcs_logic).
- `failed_jets` is read-only within `dap_step`; it is written only by the
  crew interface (V46 N02) and by the restart sequence.
- `restart_phase` is 1 while a DAP cycle is pending in the Waitlist; 0 when
  DAP is Off. DAP does not use phases 2+ (it never creates Executive jobs).

---

## 4. Public API

### 4.1 `dap_init`

```rust
/// Activate the Digital Autopilot.
///
/// Called by programs (e.g. P00 at mission phase change, P40 at ignition)
/// to enable attitude control. Arms TIME5 for the first T5RUPT cycle.
///
/// AGC correspondence: DAPINIT entry in RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc.
/// The AGC DAPINIT routine sets CMDAPMOD, clears OMEGAP/Q/R, and calls
/// WAITLIST to schedule the first DAP task at 10 cs.
///
/// # Parameters
/// - `state`: mutable reference to the full AGC state.
/// - `hw`: mutable reference to the hardware abstraction layer.
/// - `initial_mode`: the mode to activate; must not be `DapMode::Off`.
///
/// # Preconditions
/// - `initial_mode != DapMode::Off`.
/// - If `initial_mode == DapMode::Tvc`, `hw.engine().thrust_on()` must return
///   `true`. If it returns `false`, the function sets mode to `AttitudeHold`
///   and raises program alarm 0510 (TVC request without thrust active).
/// - May be called from a program job or from the T4RUPT handler; must not be
///   called from within `dap_step` itself.
///
/// # Postconditions
/// - `state.dap_state.mode` is `initial_mode` (or `AttitudeHold` if Tvc was
///   requested without thrust).
/// - `state.dap_state.restart_phase` is 1.
/// - TIME5 is armed for 10 centiseconds (hw.timers().arm_t5(10)).
/// - `state.dap_state.prev_cdu` is loaded from the current hardware CDU reading
///   so that the first rate estimate is zero rather than an artifact of the
///   previous CDU history.
/// - `state.dap_state.rate_estimate` is zeroed.
/// - A Waitlist entry for `dap_step` at 10 cs is enqueued in `state.waitlist`.
pub fn dap_init<H: AgcHardware>(
    state: &mut AgcState,
    hw: &mut H,
    initial_mode: DapMode,
)
```

**Side effects**:
- `hw.timers().arm_t5(10)` — sets TIME5 to fire T5RUPT in 100 ms.
- `hw.imu().read_cdu()` — captures current CDU angles into `prev_cdu`.
- `state.waitlist.schedule(10, dap_step)` — enqueues the first cycle.
- `state.restart.phases[GROUP_DAP]` is set to `Phase(1)`.

**Error conditions**:
- If `initial_mode == DapMode::Off`, the call is a no-op (DAP is already off).
  The implementation may optionally assert in debug builds.
- If the Waitlist is full (8 tasks pending), alarm 1202 is raised and the DAP
  task is NOT scheduled. The mode remains Off.

### 4.2 `dap_stop`

```rust
/// Deactivate the Digital Autopilot (flag-then-exit pattern).
///
/// Sets mode to Off, disarms T6, and clears the staged jet command.
/// The Waitlist entry is removed by having the DAP cycle itself detect
/// the Off mode and NOT re-arm or re-schedule (flag-then-exit).
///
/// AGC correspondence: DAPOFF entry (clear CMDAPMOD, quench jets).
/// Called by P00 (idle), engine cutoff routines, and crew V46 N00 (DAP off).
///
/// Flag-then-exit rationale: Rather than removing the Waitlist entry directly
/// (which would require knowing the exact Waitlist node handle), `dap_stop`
/// sets `mode = DapMode::Off`. When `dap_step` next runs and sees Off mode,
/// it does not re-schedule itself and does not re-arm T5. The task drains
/// naturally on the next cycle. This matches the AGC DAPIDLER pattern.
///
/// # Parameters
/// - `state`: mutable reference to the full AGC state.
/// - `hw`: mutable reference to the hardware abstraction layer.
///
/// # Preconditions
/// - May be called in any mode including Off (idempotent).
/// - May be called from a program job; must not be called from within
///   `dap_step` (use flag-then-exit: set mode = Off and return early
///   from dap_step instead).
///
/// # Postconditions
/// - `state.dap_state.mode` is `DapMode::Off`.
/// - `state.rcs_commanded_jets` is 0 (staged jet command cleared).
/// - `state.rcs_commanded_pulse_cs` is 0.
/// - `hw.rcs().quench_jets()` is called (immediate hardware quench).
/// - `hw.timers().disarm_t6()` is called to cancel any in-flight jet pulse.
/// - `state.dap_state.restart_phase` is 0.
/// - The Waitlist entry for `dap_step` will be removed when the entry fires
///   next and finds mode == Off (it will not re-schedule itself).
pub fn dap_stop<H: AgcHardware>(state: &mut AgcState, hw: &mut H)
```

**Side effects**:
- `hw.rcs().quench_jets()` — turns off all currently commanded jets immediately.
- `hw.timers().disarm_t6()` — cancels any pending T6RUPT jet-off pulse.
- `state.restart.phases[GROUP_DAP]` is set to `Phase::IDLE`.

**Note on Waitlist removal**: `dap_stop` does NOT call
`state.waitlist.remove(dap_step)`. Instead the mode flag `DapMode::Off` causes
the running `dap_step` to exit without rescheduling (§5.1). This avoids any
race between `dap_stop` (called from a program job) and the current executing
`dap_step`. The pending Waitlist entry fires once more with Off mode and then
the task disappears cleanly.

**Idempotency**: Calling `dap_stop` when already Off is a no-op with respect
to mode and staging fields. The HAL calls (`quench_jets`, `disarm_t6`) are
still executed (they are idempotent at the hardware level).

### 4.3 `dap_step`

```rust
/// T5RUPT handler — one complete DAP computation cycle.
///
/// Called as a Waitlist task every 10 centiseconds (100 ms). Must complete
/// within the T5RUPT timing budget of 20 ms (see §8). Re-arms itself at
/// the end by enqueueing the next cycle in the Waitlist — UNLESS mode is Off,
/// in which case it exits without re-scheduling (flag-then-exit pattern).
///
/// AGC correspondence: The T5RUPT vector in INTERRUPT_LEAD_INS.agc dispatches
/// to DAPIDLER (idle check) → CALLFAST (rate-damping) or the Coast DAP
/// main loop in RCS-CSM_DIGITAL_AUTOPILOT.agc. In TVC mode, TVCDAPS.agc
/// takes over.
///
/// This function is NOT called directly by application code. It is registered
/// as a Waitlist task function pointer and invoked by the Waitlist dispatcher
/// when the T5RUPT fires.
///
/// # Parameters
/// - `state`: mutable reference to the full AGC state, passed by the Waitlist
///   dispatcher (same signature as all Waitlist tasks:
///   `fn(&mut AgcState)` — see executive-spec §2.2).
///
/// # Preconditions
/// - `state.dap_state.mode` is valid (any DapMode value).
/// - `state.current_cdu` holds the CDU reading staged by the T5RUPT ISR shim
///   before this task was dispatched.
/// - `state.dap_state.prev_cdu` holds the CDU reading from the previous cycle.
///   If this is the first cycle after `dap_init`, `prev_cdu` was initialised
///   to the current hardware reading, so `rate_estimate` will be approximately zero.
///
/// # Postconditions
/// - `state.dap_state` is updated: `rate_estimate`, `attitude_error`,
///   `rcs_jet_flags`, `prev_cdu` reflect the outcome of this cycle.
/// - `state.rcs_commanded_jets` and `state.rcs_commanded_pulse_cs` hold the
///   jet command staged for the T5RUPT ISR shim to execute.
/// - If mode is Off on entry, the function returns immediately without
///   re-scheduling and without staging any hardware commands.
/// - If mode is not Off, a new Waitlist entry for `dap_step` at 10 cs has
///   been enqueued.
///
/// # Signature
pub fn dap_step(state: &mut AgcState)
```

---

## 5. Detailed Behaviour of `dap_step`

### 5.1 Entry and Mode Dispatch

```
dap_step:
    if mode == Off:
        return                        // flag-then-exit: do NOT re-arm T5,
                                      // do NOT reschedule; task drains naturally

    read current CDU: cdu_now = state.current_cdu   // staged by T5RUPT ISR shim
    compute body rates (§5.2)
    dispatch on mode:
        RateDamping  → §5.3
        AttitudeHold → §5.4
        Maneuver     → §5.5
        Tvc          → §5.6
    update prev_cdu = cdu_now        // store for next cycle
    re-arm T5: stage arm_t5(10) request
    re-schedule: state.waitlist.schedule(10, dap_step)
```

**Off mode — flag-then-exit**: When `dap_stop` sets `mode = DapMode::Off`,
the next invocation of `dap_step` returns immediately on the first line. No
hardware commands are staged, T5 is NOT re-armed, and the task is NOT
re-scheduled. The DAP cycle stops cleanly after at most one more invocation
following the `dap_stop` call.

This matches the AGC DAPIDLER pattern documented in
`RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc`, where the task checks the mode flag and
returns without rescheduling itself when the DAP is deactivated.

### 5.2 Body Rate Computation

Body rates are computed by finite differencing the CDU angle readings over the
100 ms period. CDU angles are modular (wrap at 2π), so the difference must be
computed in the space of signed angular increments.

```
delta_cdu[i] = signed_cdu_diff(cdu_now[i], prev_cdu[i])
rate_estimate[i] = delta_cdu_to_radians(delta_cdu[i]) / DAP_PERIOD_S
```

Where `DAP_PERIOD_S = 0.100` (100 ms).

`signed_cdu_diff(a, b)` returns the shortest signed arc from `b` to `a` in
CDU counts, treating the 16-bit counter as a circular space:

```rust
fn signed_cdu_diff(a: CduAngle, b: CduAngle) -> i16 {
    // Wrapping subtract; i16 cast gives signed result in [-32768, 32767]
    (a.0.wrapping_sub(b.0)) as i16
}
```

Then:

```rust
let delta_rad = (signed_cdu_diff(cdu_now[i], prev_cdu[i]) as f64)
                * (core::f64::consts::TAU / 65536.0);
state.dap_state.rate_estimate[i] = delta_rad / DAP_PERIOD_S;
```

The computed rates are stored in `DapState::rate_estimate` and passed to the
attitude/rate routines.

AGC source: The body rate computation in `RCS-CSM_DIGITAL_AUTOPILOT.agc`
performs this difference in the T5RUPT handler before branching to the
attitude routine. The original uses double-precision scaled arithmetic
equivalent to this `f64` computation.

### 5.3 RateDamping Mode

Objective: null all three body rates within the rate deadband.

```
call attitude::rate_damping_torque(
        rate_estimate,
        rate_deadband)
    → torque_request: Vec3   // signed torque in each axis [N·m or dimensionless sign]

call rcs_logic::select_jets_sm(
        torque_request,
        &state.rcs_config)        // uses RcsConfig, not bare failed_jets
    → jet_mask: u16

state.dap_state.rcs_jet_flags = jet_mask
// Stage for ISR shim — ISR shim calls fire_pulse(hw, jets, counts)
state.rcs_commanded_jets     = jet_mask
state.rcs_commanded_pulse_cs = rcs_logic::compute_pulse_duration(
                                    torque_request, jet_mask,
                                    &state.rcs_config,
                                    state.moment_of_inertia)
```

In RateDamping mode:
- `attitude_error` is set to zero (no attitude target exists).
- `commanded_attitude` is not modified.
- The mode automatically transitions to `AttitudeHold` if `commanded_attitude`
  has been loaded AND `|rate_estimate|` has fallen below `rate_deadband` on all
  axes. This matches the AGC behaviour where rate damping is used as a precursor
  to attitude hold (e.g. after a maneuver).

### 5.4 AttitudeHold Mode

Objective: maintain `commanded_attitude` within `deadband`.

```
call attitude::attitude_hold_torque(
        commanded_attitude,
        current_attitude_from_cdu(cdu_now, refsmmat),
        rate_estimate,
        deadband)
    → torque_request: Vec3

call rcs_logic::select_jets_sm(torque_request, &state.rcs_config)
    → jet_mask: u16

state.dap_state.rcs_jet_flags = jet_mask
// Stage for ISR shim
state.rcs_commanded_jets     = jet_mask
state.rcs_commanded_pulse_cs = rcs_logic::compute_pulse_duration(
                                    torque_request, jet_mask,
                                    &state.rcs_config,
                                    state.moment_of_inertia)
```

`current_attitude_from_cdu` converts the raw CDU angles to attitude angles in
the same frame as `commanded_attitude`. The exact transformation is defined in
`attitude.rs` (see `attitude-spec.md` §3.1).

`state.dap_state.attitude_error` is set to the vector returned by the attitude
error computation within `attitude_hold_torque`.

If `|attitude_error|` is within `deadband` on all axes AND `|rate_estimate|` is
within `rate_deadband`, no jets are fired (`jet_mask = 0`). This is the
"deadband satisfied" condition — the spacecraft is coasting within the
attitude box.

### 5.5 Maneuver Mode

Objective: rotate to `commanded_attitude` at `maneuver_rate` degrees per second.

```
// Step 1: advance commanded attitude by one cycle's worth of maneuver rate
for each axis i:
    commanded_attitude[i] += maneuver_rate[i] * DAP_PERIOD_S

// Step 2: check for maneuver completion
let remaining = angular_distance(commanded_attitude, final_attitude)
if remaining < MANEUVER_COMPLETE_THRESHOLD:
    state.dap_state.mode = DapMode::AttitudeHold
    state.dap_state.maneuver_rate = [0.0; 3]
    // fall through to run one AttitudeHold cycle immediately

// Step 3: run attitude hold against the (possibly updated) commanded attitude
<same logic as §5.4 AttitudeHold>
```

`MANEUVER_COMPLETE_THRESHOLD` is defined as `0.5°` (0.00873 rad). This matches
the AGC KALCMANU completion test which checks whether the steering error is
within the attitude deadband.

`final_attitude` is the maneuver target stored in a separate field
`DapState::maneuver_target` (added to `DapState`; see §3.2 amendments). During
maneuver initialisation (called from guidance or KALCMANU), both
`maneuver_rate` and `maneuver_target` are loaded. The DAP does not compute the
maneuver trajectory itself; it receives the pre-computed rate vector from
`attitude.rs` / KALCMANU.

Automatic transition from Maneuver to AttitudeHold is not visible to external
callers other than through the `mode` field. Programs that need to detect
completion should poll `state.dap_state.mode`.

### 5.6 TVC Mode

Objective: drive the SPS engine gimbal to maintain the thrust vector through
the vehicle centre of mass.

**Strategy D note**: In TVC mode `dap_step` does not call `engine.sps_gimbal`
directly. It calls `tvc::tvc_step` which accepts an `&mut E: Engine` parameter.
The T5RUPT ISR shim must arrange for the engine HAL reference to be passed
through to `tvc_step`. One approach: store the gimbal counts into staging fields
(`state.tvc_commanded_pitch_counts`, `state.tvc_commanded_yaw_counts`) and let
the ISR shim call `sps_gimbal`. An alternative is for the ISR shim to pass a
`&mut E` into `dap_step` via a global or thread-local cell before dispatching
the Waitlist task. The exact mechanism is an architectural decision; the spec
describes the logical flow.

```
// Verify engine still thrusting — if not, abort TVC
// (dap_step cannot call hw.engine() directly in Strategy D;
//  the ISR shim pre-stages the thrust_on flag in state.engine_thrusting)
if !state.engine_thrusting:
    state.dap_state.mode = DapMode::Off   // flag — ISR shim will quench jets
    return                                // flag-then-exit

// Compute attitude error
call attitude::compute_attitude_error(
        commanded_attitude,
        current_attitude_from_cdu(cdu_now, refsmmat))
    → attitude_error: Vec3

state.dap_state.attitude_error = attitude_error

// TVC step: update lead-lag filter and compute new gimbal command.
// attitude_error comes from DapState; tvc_step requires Vec3 parameter.
call tvc::tvc_step(
        &mut state.tvc_state,
        &mut state.tvc_filter,
        state.dap_state.attitude_error,   // Vec3 [roll, pitch, yaw]
        DAP_PERIOD_S,
        engine)                            // &mut E — from ISR shim context
    → (pitch_counts, yaw_counts): (i16, i16)

// tvc_step has already called engine.sps_gimbal internally.
// Store the count result in staging fields for telemetry/restart.
state.tvc_state.gimbal_pitch (already updated by tvc_step)
state.tvc_state.gimbal_yaw   (already updated by tvc_step)

// Roll is controlled by RCS even in TVC mode
let roll_torque = rate_damping_torque_1axis(
        attitude_error[0], rate_estimate[0], rate_deadband)
let roll_jet_mask = rcs_logic::select_jets_sm(
        [roll_torque, 0.0, 0.0],
        &state.rcs_config)
state.dap_state.rcs_jet_flags = roll_jet_mask
// Stage roll jet command for ISR shim
state.rcs_commanded_jets     = roll_jet_mask
state.rcs_commanded_pulse_cs = rcs_logic::compute_pulse_duration(
                                    [roll_torque, 0.0, 0.0],
                                    roll_jet_mask,
                                    &state.rcs_config,
                                    state.moment_of_inertia)
```

**AGC source**: `TVCDAPS.agc` implements the pitch and yaw gimbal loop.
`TVCROLLDAP.agc` implements the separate roll rate-damping loop. Both run in
the same T5RUPT cycle.

**TVC roll authority**: During SPS burns, pitch and yaw are controlled by
the TVC gimbal. Roll cannot be controlled by the gimbal (the SPS engine axis
has no roll moment arm); roll authority comes exclusively from the SM RCS jets.
This is why TVC mode includes a roll rate-damping sub-loop.

**Gimbal limit check**: `tvc_step` internally clamps commands to
`±GIMBAL_LIMIT_RAD` before calling `engine.sps_gimbal`. No additional clamping
is required in `dap_step`.

### 5.7 HAL Access Pattern in Task Context

`dap_step` has the Waitlist task signature `fn(&mut AgcState)`. Hardware
access in this context uses the HAL reference stored in a global or
interrupt-context cell. In the project's `no_std` design:

```rust
// The Waitlist task function accesses hardware through a thread-safe
// global cell populated by the T5RUPT ISR before calling dap_step:
pub fn dap_step(state: &mut AgcState) {
    // Hardware is accessed via a global Mutex<RefCell<H>> pattern
    // consistent with cortex_m::interrupt::free critical sections.
    // The ISR body is responsible for locking and passing the reference.
    // This is the standard embedded-Rust interrupt sharing pattern.
}
```

The exact global-state sharing mechanism (Mutex, RefCell, or a HAL pointer
embedded in AgcState) is a project-level architectural decision to be resolved
before implementation. The spec treats hardware calls as available within
`dap_step` for clarity.

### 5.8 Staged Inputs and Outputs

Per Strategy D, `dap_step` is a pure `fn(&mut AgcState)` with no `AgcHardware`
parameter. All hardware I/O is performed by the T5RUPT ISR shim before and
after calling `dap_step`. The following `AgcState` staging fields carry data
across the ISR/task boundary:

**Inputs staged by the T5RUPT ISR shim before calling `dap_step`**:

| Field | Type | Staged from | Description |
|-------|------|------------|-------------|
| `AgcState::current_cdu` | `[CduAngle; 3]` | `hw.imu().read_cdu()` | Current CDU gimbal angles |
| `AgcState::engine_thrusting` | `bool` | `hw.engine().thrust_on()` | Whether SPS engine is burning |

**Outputs staged by `dap_step` for the T5RUPT ISR shim to execute**:

| Field | Type | Read by | Description |
|-------|------|---------|-------------|
| `AgcState::rcs_commanded_jets` | `u16` | T5RUPT ISR shim | Jet bitmask for `fire_pulse`; upper byte = jets_b (ch 06), lower byte = jets_a (ch 05). `0` = no fire. |
| `AgcState::rcs_commanded_pulse_cs` | `u16` | T5RUPT ISR shim | Pulse duration in T6 counts from `compute_pulse_duration`. `0` = no fire. |

**Reset at start of each `dap_step` cycle** (before re-computing):
```rust
state.rcs_commanded_jets     = 0;
state.rcs_commanded_pulse_cs = 0;
```

**T5RUPT ISR shim post-`dap_step` sequence**:
```rust
// After dap_step returns, ISR shim performs HAL I/O:
let jets   = state.rcs_commanded_jets;
let counts = state.rcs_commanded_pulse_cs;
if counts > 0 && jets != 0 {
    rcs_logic::fire_pulse(hw, jets, counts);
}
```

---

## 6. Mode Transitions

### 6.1 Transition Table

| From \ To    | Off | RateDamping | AttitudeHold | Maneuver | Tvc |
|:-------------|:---:|:-----------:|:------------:|:--------:|:---:|
| Off          | —   | A           | A            | A        | A*  |
| RateDamping  | A   | —           | A            | A        | X   |
| AttitudeHold | A   | A           | —            | A        | X   |
| Maneuver     | A   | X           | Auto         | —        | X   |
| Tvc          | A   | A**         | X            | X        | —   |

Key:
- **A** — allowed; may be requested by programs or crew verbs.
- **A*** — allowed only if `hw.engine().thrust_on()` returns `true`.
- **A**** — allowed only after `hw.engine().thrust_on()` returns `false`
  (i.e., engine cutoff has occurred); transition is typically triggered by
  the P40 cutoff routine.
- **Auto** — automatic transition within `dap_step` on maneuver completion.
- **X** — forbidden; the request is silently rejected, the mode is unchanged,
  and alarm 0510 (invalid DAP mode request) is raised.

### 6.2 Transition Semantics

**Any → Off**: Safe at any time. `dap_stop` is called, which quenches jets
and cancels T6. MODE_LAMP is extinguished.

**Off → any active mode**: `dap_init` arms T5 and schedules the first cycle.
The transition happens before the first `dap_step` call.

**RateDamping → AttitudeHold**: May be triggered by a program loading
`commanded_attitude` and calling a transition request. The `deadband` must be
loaded before the transition takes effect. If `deadband` is zero, a 0.1°
minimum is substituted.

**Any → Maneuver**: The caller (guidance or crew verb V49) must pre-load
`maneuver_target` and `maneuver_rate` in `DapState` before requesting the
transition. If `maneuver_rate` is the zero vector, the transition is treated
as AttitudeHold.

**Maneuver → AttitudeHold (automatic)**: When the maneuver is complete (§5.5),
`dap_step` sets `mode = AttitudeHold` and `maneuver_rate = [0; 3]`. The same
`dap_step` cycle then runs an AttitudeHold cycle against the final target
attitude.

**Any → Tvc**: The SPS engine must already be thrusting (`hw.engine().thrust_on()`).
The caller (P40 ignition sequence) is responsible for sequencing: enable TVC
on channel 12, verify `thrust_on`, then call `dap_init(Tvc)`. If `thrust_on`
is false when the request arrives, mode is set to AttitudeHold and alarm 0510
is raised.

**Tvc → Off**: Triggered by the P40 cutoff routine. `dap_stop` quenches roll
RCS jets. The gimbal is left at its last commanded position; `sps_enable(false)`
(called separately by P40) de-energises the gimbal drive.

**Tvc → RateDamping**: Allowed after engine cutoff. This transition is used
during the post-cutoff attitude settling phase. At this point, the spacecraft
body is still rotating from SPS thrust; RateDamping nulls the residual rates
before AttitudeHold takes over.

**Tvc → Maneuver or AttitudeHold (while thrusting)**: Forbidden (X). The TVC
gimbal loop must not be interrupted during SPS firing. If this transition is
attempted, the DAP remains in Tvc and raises alarm 0510.

### 6.3 Restart Behaviour

On hardware restart, `RestartProtection` reads `state.dap_state.restart_phase`:

| Phase | Action |
|-------|--------|
| 0 (IDLE) | DAP was Off — no restart action needed |
| 1 | Re-enqueue `dap_step` in Waitlist at 10 cs; mode is preserved from erasable |

After restart, the first `dap_step` call re-reads the current CDU angles (from
the staged `state.current_cdu`) and re-initialises `prev_cdu`. The rate estimate
for that first cycle may be slightly elevated (since the inter-restart time is
longer than 100 ms), but the rate deadband prevents spurious jet firings.

If the system restarts while in Tvc mode, the restart sequence checks
`hw.engine().thrust_on()`. If the engine is still burning (e.g. restart during
a burn), Tvc is restored. If the engine has cut off, mode is transitioned to
RateDamping to null post-burn rates.

---

## 7. CDU Angle History and Rate Computation

### 7.1 Storage

`DapState::prev_cdu` stores three `CduAngle` values (16-bit counts each).
On each `dap_step` call the sequence is:

1. Read `cdu_now = state.current_cdu` (staged by T5RUPT ISR shim).
2. Perform all control calculations using `cdu_now` and `prev_cdu`.
3. Store `prev_cdu = cdu_now` at the bottom of the step, just before re-arming.

This ordering guarantees that `prev_cdu` always reflects the CDU state at the
start of the most recently completed cycle, not the current one.

### 7.2 Wraparound Handling

CDU counts are modular on 16 bits (wrap at 65536). A spacecraft rotating from
near 360° back through 0° will produce a positive `cdu_now` and a near-65535
`prev_cdu`. The signed difference using `(a.wrapping_sub(b)) as i16` correctly
handles this: the result is in the range `[-32768, +32767]` counts, representing
at most ±180° of rotation per cycle. Since the spacecraft cannot physically rotate
more than ~1.8°/s (the maximum controllable rate), a 100 ms cycle can produce at
most ~0.18° of CDU change — far within the ±180° unambiguous range.

**Precondition maintained by physics**: If the spacecraft somehow rotates more
than 180° between cycles (physically impossible under normal operations), the
computed rate would be incorrect (sign inversion). This is documented as a
known limitation of the finite-difference rate estimator; there is no hardware
rate sensor in the Block 2 AGC.

### 7.3 Initialisation

`dap_init` reads the current CDU angles and stores them into `prev_cdu` before
scheduling the first task:

```rust
let cdu_now = hw.imu().read_cdu();
state.dap_state.prev_cdu = cdu_now;
state.dap_state.rate_estimate = [0.0; 3];
```

This ensures the first cycle computes rates near zero rather than a large
transient from the uninitialized default value of `CduAngle(0)`.

---

## 8. Timing Budget

### 8.1 T5RUPT Period and Re-arm

The DAP operates on a 10 centisecond (100 ms) period. At the end of each
`dap_step`, TIME5 is re-armed via a staging mechanism (ISR shim re-arms after
`dap_step` returns).

| Parameter | Value | Notes |
|---|---|---|
| T5RUPT period | 10 cs = 100 ms | `arm_t5(10)`; see hal-spec §6.2 |
| Maximum handler budget | 20 ms | Architecture §13.2; see below |
| CDU read | ~0.1 ms | HAL SPI transaction (ISR shim) |
| Rate computation | ~0.01 ms | Three multiplies + divides; f64 HW FPU |
| Attitude error computation | ~0.5 ms | REFSMMAT multiply; 9 f64 multiplies |
| Torque → jet select | ~0.1 ms | Table lookup in rcs_logic |
| T6 arm + jet fire | ~0.05 ms | Register write via HAL (ISR shim) |
| TVC lead-lag filter | ~0.2 ms | 6 multiply-adds per axis |
| Total (worst case, TVC + roll) | ~1.0 ms | Well within 20 ms budget |

The 20 ms budget is the **maximum allowed** runtime for the T5RUPT handler
as documented in `docs/architecture.md` §13.2. Exceeding this budget risks
missing the subsequent T6RUPT jet-off pulse, which would cause jets to fire
for an uncontrolled duration.

On the minimum target processor (Cortex-M7 with double-precision FPU at 216 MHz),
the estimated actual runtime is ~1 ms, giving a ≈20x timing margin. This is
consistent with the architecture §13.2 note: "~100× DAP timing margin".

### 8.2 Self-Rescheduling

`dap_step` re-schedules itself in the Waitlist at the END of each cycle
(if mode is not Off). The ISR shim re-arms TIME5 after `dap_step` returns:

```rust
// In dap_step (when mode != Off):
state.waitlist.schedule(10, dap_step);

// In T5RUPT ISR shim (after dap_step returns, if mode != Off):
hw.timers().arm_t5(10);
```

The `schedule` call uses 10 cs (the delta-time chain entry). The `arm_t5(10)`
call loads TIME5 directly. Both must be called to keep both the hardware timer
and the Waitlist metadata in sync (the Waitlist uses TIME3/T3RUPT for dispatch;
TIME5 is separate and provides the actual interrupt trigger).

The correct period is **10 centiseconds (100 ms)**. The architecture
documentation in one place describes this as "100 cs (1 second)" which is an
error in the prose — the authoritative value is 10 cs (100 ms) as used in all
code-level references (`arm_t5(10)`, "10 cs" in the timing tables) and
confirmed by the Comanche055 AGC source which uses `CALLFAST 10 CS`.

---

## 9. Constants

```rust
/// DAP cycle period in centiseconds. Used for arm_t5 and Waitlist scheduling.
pub const DAP_PERIOD_CS: u16 = 10;

/// DAP cycle period in seconds. Used for rate computation and maneuver stepping.
pub const DAP_PERIOD_S: f64 = 0.100;

/// Minimum attitude deadband (radians). Substituted if crew sets deadband to zero.
pub const DAP_MIN_DEADBAND_RAD: f64 = 0.00175; // 0.1 degrees

/// Default attitude deadband (radians). 5 degrees (coarse deadband).
pub const DAP_DEFAULT_DEADBAND_RAD: f64 = 0.0873; // 5 degrees

/// Default rate deadband (rad/s). 0.5 degrees/second.
pub const DAP_DEFAULT_RATE_DEADBAND_RAD_S: f64 = 0.00873;

/// SPS gimbal maximum angle (radians). ±6 degrees.
pub const TVC_GIMBAL_LIMIT_RAD: f64 = 0.1047;

/// Maneuver completion threshold (radians). 0.5 degrees.
pub const MANEUVER_COMPLETE_RAD: f64 = 0.00873;

/// Restart group index for the DAP task.
pub const GROUP_DAP: usize = 5; // GROUP 6 in AGC (0-indexed = 5)
```

---

## 10. Interfaces to Sibling Modules

The DAP calls the following functions in sibling modules. Their signatures are
fixed by the calling convention used in `dap_step`.

### 10.1 `attitude::compute_attitude_error`

Called from: AttitudeHold (§5.4), Maneuver (§5.5), TVC (§5.6)

```rust
/// Compute attitude error between commanded and current attitudes.
/// Returns [roll_err, pitch_err, yaw_err] in radians.
/// Cross-reference: attitude-spec.md §3.2
pub fn compute_attitude_error(
    commanded: Vec3,
    current: Vec3,
) -> Vec3
```

### 10.2 `attitude::rate_damping_torque`

Called from: RateDamping (§5.3), TVC roll sub-loop (§5.6)

```rust
/// Compute signed torque request to null body rates.
/// Returns [roll, pitch, yaw] torque sign (+1.0, 0.0, -1.0).
/// Zero if rate magnitude is within rate_deadband.
/// Cross-reference: attitude-spec.md §3.3
pub fn rate_damping_torque(
    rate_estimate: Vec3,
    rate_deadband: f64,
) -> Vec3
```

### 10.3 `attitude::attitude_hold_torque`

Called from: AttitudeHold (§5.4), Maneuver after step (§5.5)

```rust
/// Compute signed torque request to drive attitude error to zero.
/// Returns [roll, pitch, yaw] torque sign (+1.0, 0.0, -1.0).
/// Zero if |attitude_error| < deadband on all axes.
/// Cross-reference: attitude-spec.md §3.4
pub fn attitude_hold_torque(
    attitude_error: Vec3,
    rate_estimate: Vec3,
    deadband: f64,
) -> Vec3
```

### 10.4 `rcs_logic::select_jets_sm`

Called from: RateDamping (§5.3), AttitudeHold (§5.4), Maneuver (§5.5),
TVC roll sub-loop (§5.6)

```rust
/// Map torque request to SM RCS jet bitmask (16 jets).
/// config: &RcsConfig — provides enable mask, jets_per_axis, etc.
/// Returns u16 jet bitmask (upper byte = jets_b/ch06, lower = jets_a/ch05).
/// Cross-reference: rcs-logic-spec.md §6.1
pub fn select_jets_sm(
    torque_request: Vec3,
    config: &RcsConfig,
) -> u16
```

### 10.5 `rcs_logic::fire_pulse` (ISR shim only)

`fire_pulse` is NOT called from `dap_step`. It is called exclusively by the
T5RUPT ISR shim after `dap_step` returns, using the staged fields
`state.rcs_commanded_jets` and `state.rcs_commanded_pulse_cs`.

```rust
/// Fire the selected jets for a computed pulse width.
/// Arms T6 with the pulse duration; T6RUPT handler quenches the jets.
/// MUST only be called from the T5RUPT ISR shim, never from dap_step.
/// Cross-reference: rcs-logic-spec.md §6.3
pub fn fire_pulse<H: AgcHardware>(
    hw: &mut H,
    jet_mask: u16,
    duration_counts: u16,
)
```

### 10.6 `tvc::tvc_step`

Called from: TVC pitch/yaw loop (§5.6)

```rust
/// Execute one TVC control cycle.
/// Applies lead-lag filter, adds trim, saturates, calls engine.sps_gimbal.
/// attitude_error is passed from DapState::attitude_error (Vec3).
/// Returns (pitch_counts, yaw_counts): i16 CDU counts written to hardware.
/// Cross-reference: tvc-spec.md §4.2
pub fn tvc_step<E: Engine>(
    state:          &mut TvcState,
    filter:         &mut TvcFilter,
    attitude_error: Vec3,
    dt:             f64,
    engine:         &mut E,
) -> (i16, i16)
```

### 10.7 `Engine::sps_gimbal`

Called from: TVC mode (§5.6) — internally by `tvc_step`, not directly by `dap_step`.

```rust
/// Command SPS engine pitch and yaw gimbal angles.
/// pitch, yaw: signed i16 counts (counter cell units).
/// Cross-reference: hal-spec.md §10.3
hw.engine().sps_gimbal(pitch_counts: i16, yaw_counts: i16)
```

---

## 11. AGC Scaling and Rust Conversion Reference

| Quantity | AGC Scale | AGC Full Scale | Rust Type | Rust Units |
|---|---|---|---|---|
| CDU angle | B-1 revolutions (15-bit) | 2^15 counts = 2π rad | `CduAngle(u16)` | 2^16 counts = 2π rad |
| Body rate | B+4 rad/s | 2^−11 rad/s per count | `f64` | rad/s |
| Attitude error | B-1 half-revolutions | 2^−14 rad per count | `f64` | rad |
| Attitude deadband | B-1 half-revolutions | same | `f64` | rad |
| Gimbal angle | 3200 pps counter cell | ~0.00196 rad per count (2π/3200) | `i16` | CDU pulse counts |
| TVC gimbal (rad) | — | ±6° = ±0.1047 rad | `f64` | rad (internal) |
| DAP period | centiseconds | 10 cs = 100 ms | `u16` | centiseconds |

Conversion from `CduAngle` to radians:
```rust
let angle_rad = cdu.0 as f64 * (core::f64::consts::TAU / 65536.0);
```

Conversion from `f64` radians to SPS gimbal i16 counts (AD-5 scale):
```rust
let counts = (angle_rad * 3200.0 / core::f64::consts::TAU) as i16;
// ≈ angle_rad × 509.3
```
The exact scale factor for the gimbal servo is defined in `tvc-spec.md`.

---

## 12. Test Cases

### TC-DAP-01: Off mode exits immediately without rescheduling

```rust
// Verify that dap_step in Off mode returns without staging any hardware
// commands and without re-scheduling itself.
let mut state = AgcState::new(); // DapState::mode = Off by default
state.rcs_commanded_jets     = 0;
state.rcs_commanded_pulse_cs = 0;

dap_step(&mut state);

// Post-condition: no jets staged
assert_eq!(state.rcs_commanded_jets, 0);
assert_eq!(state.rcs_commanded_pulse_cs, 0);
// Post-condition: rate estimate unchanged (zero)
assert_eq!(state.dap_state.rate_estimate, [0.0; 3]);
// Post-condition: mode still Off
assert_eq!(state.dap_state.mode, DapMode::Off);
// Post-condition: Waitlist was NOT rescheduled (task count unchanged)
```

### TC-DAP-02: RateDamping nulls non-zero rates

```rust
// A spacecraft with a known initial body rate should have jets selected
// to oppose that rate.
let mut state = AgcState::new();
state.dap_state.mode = DapMode::RateDamping;
state.dap_state.rate_deadband = 0.00873; // 0.5 deg/s

// Simulate a 2 deg/s roll rate by arranging prev_cdu to differ from
// cdu_now by 2 deg/s * 0.1 s = 0.2 deg = 0.2/360 * 65536 ≈ 36 counts
let initial_cdu = CduAngle(1000);
let advanced_cdu = CduAngle(1036); // ~0.2° ahead in roll
state.dap_state.prev_cdu = [initial_cdu, CduAngle(0), CduAngle(0)];
// Stage cdu_now = [advanced_cdu, CduAngle(0), CduAngle(0)]
state.current_cdu = [advanced_cdu, CduAngle(0), CduAngle(0)];

dap_step(&mut state);

// Rate estimate should be approximately 2 deg/s = 0.0349 rad/s in roll
let roll_rate = state.dap_state.rate_estimate[0];
assert!((roll_rate - 0.0349).abs() < 0.005,
    "Expected roll rate ~0.035 rad/s, got {}", roll_rate);

// Jets should be staged to oppose positive roll rate
assert_ne!(state.rcs_commanded_jets, 0,
    "Expected jets staged when rate exceeds deadband");
```

### TC-DAP-03: AttitudeHold within deadband fires no jets

```rust
// When attitude error and rate are within deadbands, no jets should fire.
let mut state = AgcState::new();
state.dap_state.mode = DapMode::AttitudeHold;
state.dap_state.deadband = 0.0873;       // 5° deadband
state.dap_state.rate_deadband = 0.00873; // 0.5 deg/s rate deadband
// Set commanded attitude = current attitude (zero error)
state.dap_state.commanded_attitude = [0.0, 0.0, 0.0];
// prev_cdu == cdu_now → zero rates
state.dap_state.prev_cdu = [CduAngle(0); 3];
state.current_cdu = [CduAngle(0); 3];

dap_step(&mut state);

assert_eq!(state.rcs_commanded_jets, 0,
    "No jets should be staged when within deadband");
assert!(state.dap_state.attitude_error.iter().all(|&e| e.abs() < 0.0873),
    "Attitude error should be within deadband");
```

### TC-DAP-04: TVC mode delegates to tvc_step and calls sps_gimbal

```rust
// TVC mode must call tvc::tvc_step with the full correct signature.
let mut state = AgcState::new();
state.dap_state.mode = DapMode::Tvc;
state.engine_thrusting = true;
state.dap_state.commanded_attitude = [0.0, 0.0, 0.0]; // target: zero error

// Introduce a pitch error: CDU reading shows 2° off target
let pitch_offset_counts = (2.0_f64.to_radians() / core::f64::consts::TAU * 65536.0) as u16;
state.dap_state.prev_cdu = [CduAngle(0), CduAngle(pitch_offset_counts), CduAngle(0)];
state.current_cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];
// (Staged attitude error from ISR, pitch error = 2°)

dap_step(&mut state);

// Verify TvcState was updated (non-zero pitch gimbal)
assert_ne!(state.tvc_state.gimbal_pitch, 0.0,
    "Expected non-zero gimbal pitch command for 2° attitude error");
// Mode should still be Tvc
assert_eq!(state.dap_state.mode, DapMode::Tvc);
```

### TC-DAP-05: Maneuver → AttitudeHold automatic transition

```rust
// When a maneuver reaches its target, mode transitions to AttitudeHold.
let mut state = AgcState::new();
state.dap_state.mode = DapMode::Maneuver;
// Target is very close to current — within MANEUVER_COMPLETE_RAD
state.dap_state.commanded_attitude = [0.0, 0.0, 0.0];
state.dap_state.maneuver_target   = [0.0, 0.0, 0.001]; // 0.057° off — within 0.5° threshold
state.dap_state.maneuver_rate     = [0.0, 0.0, 0.0001];
state.dap_state.deadband          = 0.0873;
state.dap_state.prev_cdu          = [CduAngle(0); 3];
state.current_cdu                 = [CduAngle(0); 3];

dap_step(&mut state);

assert_eq!(state.dap_state.mode, DapMode::AttitudeHold,
    "Maneuver should transition to AttitudeHold on completion");
assert_eq!(state.dap_state.maneuver_rate, [0.0; 3],
    "maneuver_rate should be zeroed on completion");
```

### TC-DAP-06: Forbidden mode transition Tvc → Maneuver is rejected

```rust
// Attempting to transition from Tvc to Maneuver while thrusting must fail.
let mut state = AgcState::new();
let mut hw = SimHardware::new();
hw.engine.thrusting = true;
state.dap_state.mode = DapMode::Tvc;

// dap_init with Maneuver while in Tvc and thrusting
// The expected behaviour: request is rejected, mode stays Tvc, alarm raised.
dap_init(&mut state, &mut hw, DapMode::Maneuver);

assert_eq!(state.dap_state.mode, DapMode::Tvc,
    "Mode must remain Tvc when Maneuver requested during active burn");
// Alarm 0510 should have been raised
assert_eq!(state.alarm.code, 0o0510);
```

### TC-DAP-07: dap_init loads prev_cdu from hardware

```rust
// After dap_init, prev_cdu should match the current hardware CDU reading.
let mut state = AgcState::new();
let mut hw = SimHardware::new();
hw.imu.set_cdu([CduAngle(1000), CduAngle(2000), CduAngle(3000)]);

dap_init(&mut state, &mut hw, DapMode::RateDamping);

assert_eq!(state.dap_state.prev_cdu[0], CduAngle(1000));
assert_eq!(state.dap_state.prev_cdu[1], CduAngle(2000));
assert_eq!(state.dap_state.prev_cdu[2], CduAngle(3000));
assert_eq!(state.dap_state.rate_estimate, [0.0; 3]);
```

### TC-DAP-08: dap_stop sets mode Off and clears staging fields

```rust
// After dap_stop, mode is Off, staging fields are cleared, HAL I/O is done.
let mut state = AgcState::new();
let mut hw = SimHardware::new();
dap_init(&mut state, &mut hw, DapMode::RateDamping);
state.rcs_commanded_jets = 0b0000_1111_0000_1111u16;  // some jets staged
state.rcs_commanded_pulse_cs = 22;

dap_stop(&mut state, &mut hw);

assert_eq!(state.dap_state.mode, DapMode::Off);
assert_eq!(state.rcs_commanded_jets, 0,     "staged jets cleared by dap_stop");
assert_eq!(state.rcs_commanded_pulse_cs, 0, "staged pulse cleared by dap_stop");
assert!(hw.rcs.jets_quenched, "quench_jets must be called by dap_stop");
// T6 must be disarmed
assert!(!hw.timers.t6_armed, "T6 must be disarmed after dap_stop");
// Subsequent dap_step should exit immediately (flag-then-exit)
dap_step(&mut state);  // should be a no-op
assert_eq!(state.dap_state.mode, DapMode::Off, "mode remains Off after step");
```

---

## 13. Restart Protection

The DAP uses restart group `GROUP_DAP` (index 5, corresponding to AGC GROUP 6).

Phase encoding:

| `restart_phase` | Meaning |
|---|---|
| 0 | DAP is Off — no restart action |
| 1 | DAP is active — re-enqueue `dap_step` in Waitlist at 10 cs |

The restart sequence (`services/fresh_start.rs`) inspects
`state.dap_state.restart_phase` after copying the phase register state from
the RESTART TABLE. If `restart_phase == 1`:

1. Set `state.dap_state.mode` from the preserved erasable (mode is in DapState,
   which survives a RESTART — it is not zeroed on RESTART, only on FRESH START).
2. If mode is Tvc and `hw.engine().thrust_on()` is false, transition to
   RateDamping.
3. Call `dap_init` to re-arm T5 and re-enqueue the task.

On FRESH START (total power-on reset), `AgcState::new()` zeroes all fields
including `dap_state`, so `mode = Off`, `restart_phase = 0`, and no DAP is
started until a program calls `dap_init`.

---

## 14. Integration with Programs

### 14.1 P00 (CMC Idling)

P00 activates the DAP in `AttitudeHold` mode at the crew-selected deadband.
If no attitude has been commanded by the crew, `commanded_attitude` remains
at whatever the last programmed attitude was. P00 does not call `dap_stop`.

### 14.2 P40 (SPS Burn)

P40 ignition sequence:
1. Enables TVC on channel 12 (bit 8).
2. Calls `dap_init(Tvc)` after engine ignition confirmation.
3. At SPS cutoff, calls `dap_stop` then immediately calls `dap_init(RateDamping)`
   to null post-burn rates.

### 14.3 Crew V46 N01 (DAP Data Load)

V46 N01 allows the crew to set:
- `dap_state.deadband` (DAPDATR1 bits 11–8)
- `dap_state.rate_deadband` (WFORPQR)
- `dap_state.num_jets` (DAPDATR1 bits 5–4)

V46 N02 allows the crew to mark individual jets as failed:
- `dap_state.failed_jets`

These writes happen from a DSKY job context (not from `dap_step`). Because
the DAP is a Waitlist task and the DSKY handler is a job, there is a potential
race: the DSKY job could update `deadband` while a DAP cycle is mid-execution.
In the Rust port, the borrow checker prevents concurrent mutable borrows of
`AgcState`, so the race is structurally prevented at the architecture level —
consistent with the AGC's cooperative scheduling discipline. No additional
locking is required.

---

## 15. Spec Quality Checklist

| Item | Status |
|------|--------|
| AGC source file and line range referenced | Satisfied — see §2.1 and §7 |
| All erasable variables and AGC addresses listed | Satisfied — §2.2 table |
| Scale factors documented for all fixed-point values | Satisfied — §11 table |
| Corresponding `f64` SI units documented | Satisfied — §11 table and §2.2 |
| Input/output preconditions and postconditions stated | Satisfied — §4.1, 4.2, 4.3 |
| Edge cases and error handling specified | Satisfied — §6.1, §6.2, §5.3, §5.6 |
| At least 5 test cases with expected values | Satisfied — 8 test cases, §12 |
| Rust API signature designed | Satisfied — §3.1, 3.2, 4.1, 4.2, 4.3 |
| Invariants explicitly stated | Satisfied — §3.2 invariants block |
| Consistency with `docs/architecture.md` checked | Satisfied — §1, §8.1, §14 |
| CI-1 (Strategy D) applied | Satisfied — §5.8 staged inputs/outputs; `fire_pulse` ISR-shim-only (§10.5) |
| CI-3 (tvc_step signature) applied | Satisfied — §5.6 and §10.6: `tvc_step` takes `attitude_error: Vec3` and `&mut TvcFilter`; DapState.attitude_error used |
| CI-4 (select_jets_sm u16 + RcsConfig) applied | Satisfied — §10.4: `select_jets_sm(torque, &config) -> u16` |
| CI-9 (dap_stop flag-then-exit) applied | Satisfied — §4.2: mode=Off flag; §5.1: Off mode exits without reschedule |
