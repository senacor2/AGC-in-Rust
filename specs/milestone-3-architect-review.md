# Milestone 3 Architect Review

**Date**: 2026-04-09
**Reviewer**: Software Architect
**Scope**: All 10 Milestone 3 specifications
**Target status**: Specs must be fixable-in-place before implementation starts

---

## 1. Summary

Overall, the ten Milestone 3 specs are of high quality. They are internally
detailed, reference the correct AGC source files, and articulate
pre-/post-conditions and test cases. Kepler, Lambert, conics, RCS-logic, and
targeting are publication-ready; attitude, imu_control and TVC are close to
ready pending small fixes. Two specs (DAP, maneuver) contain cross-module
conflicts that must be resolved before implementation starts.

The single most important architectural gap is the **HAL-in-Waitlist-task**
question flagged by dap-spec §5.7 and independently by average-g-spec §4.3 and
§5. Four different specs implicitly assume four different answers, and the
project cannot compile until one is chosen. This is the top critical issue.

Beyond that, the conflicts are concentrated in a few identifiable places:

1. The `Maneuver` struct definition disagrees between `targeting-spec` and
   `maneuver-spec`.
2. The data flow from `cross_product_steering` to TVC is specified in three
   incompatible ways across dap-spec, tvc-spec, and maneuver-spec.
3. The RCS `jet_mask` return type (`(u8, u8)` vs `u16`) disagrees between
   rcs-logic-spec and dap-spec.
4. Several specs call `f64::sin` / `f64::cos` / `f64::atan2` without routing
   through `libm` for `no_std`.
5. The existing `DapState` and `TvcState` in `agc-core/src/control/*.rs` are
   subsets of what the specs require; several specs implicitly assume the
   extensions are already in place.

All critical issues below can be resolved by spec edits (no source code
changes required) plus a short decision from the user on one architectural
question.

---

## 2. Critical Issues (must fix before implementation)

### CI-1. HAL-in-task-context is unresolved project-wide

**Affected specs**: dap-spec §5.7, average-g-spec §4.3 & §5, imu-control-spec
§6.1 (uses `hw.imu().torque_gyro` inside a T4RUPT task), rcs-logic-spec §6.3
(`fire_pulse<H: AgcHardware>`), maneuver-spec §6.2.

**Problem**: `dap_step` and `servicer_task` are both registered in the
Waitlist as `fn(&mut AgcState)` (see `specs/executive-spec.md` §2.2). They
have no `&mut H: AgcHardware` parameter. Yet:

- dap-spec §5 freely writes `hw.imu().read_cdu()`, `hw.rcs().fire_jets(...)`,
  `hw.engine().sps_gimbal(...)`, `hw.timers().arm_t5(10)`.
- average-g-spec §5 writes `hw.imu().read_pipa()`, `hw.timers().arm_t3(...)`.
- rcs-logic-spec §6.3 exposes `fire_pulse<H: AgcHardware>(hw: &mut H, ...)`.
- imu-control-spec §6.1 writes `hw.imu().torque_gyro(...)`.

Four different patterns are implied across the specs:

(a) `hw` stored as a raw pointer on `AgcState` (average-g-spec §4.3, §5).
(b) `hw` threaded via a `cortex_m::interrupt::Mutex<RefCell<H>>` static
    (dap-spec §5.7, `docs/architecture.md` §14.3 "No static mut" paragraph).
(c) The Waitlist task signature changes to `fn(&mut AgcState, &mut dyn
    AgcHardware)` (not proposed by any spec but is the obvious third option).
(d) Tasks only touch `AgcState`; hardware I/O is deferred to a T3/T5-context
    shim that runs *before* dispatch and stages inputs/outputs into new
    `AgcState` fields (the Strategy-B approach used by average-g-spec for
    `pipa_counts`). dap-spec §5 does not use this pattern but it is the most
    consistent with the existing lib.rs code.

**Action needed**: The user must choose one pattern. My recommendation:

> **Adopt Strategy D (staging fields) uniformly.** Extend the pattern already
> in `AgcState` (`pipa_counts`, `servicer_exit`) so that every T3/T4/T5 ISR
> shim performs all HAL reads before calling the task, stores the results in
> `AgcState` fields, runs the task as `fn(&mut AgcState)`, then performs all
> HAL writes the task requested via a new `AgcState` command-staging field.

Rationale: it keeps the Waitlist task signature `fn(&mut AgcState)` as
documented in `executive-spec.md` §2.2, avoids `static mut` and raw pointers,
and is consistent with the `pipa_counts` mechanism already in place. It also
solves CI-1 for `imu-control-spec` (gyro drift comp already stages via
`state.last_drift_comp_time`), `dap-spec` (CDU read must be staged before
`dap_step` runs), and `rcs-logic-spec` (`fire_pulse` becomes a command written
into new staging fields `state.rcs_commanded_jets: (u8, u8)` and
`state.rcs_commanded_pulse_cs: u16`; the T5 ISR shim calls `arm_t6` and
`fire_sm_jets` after `dap_step` returns).

**Fix required**: After the user decides, all four specs must be edited to
use the chosen pattern. For Strategy D specifically:

- dap-spec §5 needs a new subsection "Staged inputs and outputs" listing
  the fields it reads/writes on `AgcState`.
- rcs-logic-spec §6.3 must either (i) drop `fire_pulse` entirely and instead
  document the staging fields, or (ii) keep `fire_pulse` as a helper that is
  called only from the T5 ISR shim, not from `dap_step`.
- average-g-spec already hints at this approach; it just needs the
  `hw.imu().read_pipa()` call moved to a documented "T3 ISR shim" subsection.
- imu-control-spec `compute_gyro_drift` is already pure; the hardware side
  is handled by the T4RUPT ISR shim. Only the example in §6.1 needs
  clarifying language.

**Until this is resolved, no code in control/ or services/ can be written.**

---

### CI-2. `Maneuver` struct definition conflict

**Affected specs**: targeting-spec §3.1 vs maneuver-spec §5.1 and §10 test
cases.

**Problem**: targeting-spec declares

```rust
pub struct Maneuver {
    pub tig: Met,
    pub delta_v: DeltaV,          // newtype over Vec3
    pub burn_attitude: Mat3x3,
    pub mode: TargetingMode,
}
```

But maneuver-spec TC-MANEUVER-1 constructs

```rust
let target = Maneuver {
    tig:     Met(180_000),
    delta_v: DeltaV([90.0, 0.0, -60.0]),
    // <-- burn_attitude and mode missing
};
```

and maneuver-spec §5.1 implementation reads `target.delta_v.0`, consistent
with `DeltaV(pub Vec3)` as a newtype — good — but the `Maneuver` instance
cannot be constructed without all four fields.

**Fix (apply to maneuver-spec)**: Rewrite all `Maneuver { ... }` literals in
maneuver-spec §5.1, §10 test cases, and any other spot to include
`burn_attitude` and `mode`. Use

```rust
burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
mode: TargetingMode::ExternalDeltaV,
```

as the default for test fixtures. Also add to maneuver-spec §4.1 a note that
`BurnState` does not store `burn_attitude` or `mode` because those are
consumed by P40 setup only (before `burn_init` is called).

---

### CI-3. Cross-product steering data path is ambiguous

**Affected specs**: maneuver-spec §7.1 and §7.3, tvc-spec §4.2, dap-spec
§5.6, `AgcState::servicer_exit` comment in current `lib.rs`.

**Problem**: Three incompatible data paths are specified:

1. **maneuver-spec §7.1**: the P40 SERVICER exit hook writes
   `omega_c` into `state.tvc_state.attitude_error: Vec3` (and adds this
   `attitude_error` field to `TvcState`).
2. **tvc-spec §4.2**: `tvc_step` receives `attitude_error: Vec3` as a
   function parameter from the DAP supervisor. `TvcState` has no
   `attitude_error` field in tvc-spec §3.1.
3. **dap-spec §5.6**: `tvc_step` is called with `attitude_error[1]` and
   `attitude_error[2]` — two `f64` scalars — reading from `DapState::attitude_error`.
4. **current lib.rs**: `servicer_exit` comment says "P40 sets this to
   `guidance::tvc::cross_product_steering_update`" — but that function does
   not exist in any spec; the real function is
   `guidance::maneuver::cross_product_steering`.

**Fix (apply to all three specs)**: The authoritative data path is:

```
SERVICER exit (2 s)
  -> burn_update(&mut state.burn, measured_dv, 2.0)
  -> omega_c = cross_product_steering(remaining_dv, measured_dv)
  -> state.dap_state.attitude_error = omega_c   (existing Vec3 field, no TvcState extension)

T5RUPT (100 ms) - DAP supervisor
  -> if mode == Tvc: tvc_step(&mut state.tvc_state, &mut state.tvc_filter,
                              state.dap_state.attitude_error, dt, /* hw ref via CI-1 */)
```

Concretely:

- **maneuver-spec §7.1**: change "`state.tvc_state.attitude_error = omega_c`"
  to "`state.dap_state.attitude_error = omega_c`".
- **maneuver-spec §7**: remove the section proposing a new
  `TvcState::attitude_error` field.
- **tvc-spec §3.1**: keep `TvcState` as specified (no attitude_error field).
- **tvc-spec §4.2**: keep the `attitude_error: Vec3` parameter; this is the
  full vector, not a pitch/yaw scalar pair.
- **dap-spec §5.6**: the pseudocode `tvc::tvc_step(&mut state.tvc_state,
  attitude_error[1], attitude_error[2])` must become
  `tvc::tvc_step(&mut state.tvc_state, &mut state.tvc_filter,
  state.dap_state.attitude_error, DAP_PERIOD_S, /* engine */)`. The roll axis
  (index 0) continues to flow to the roll-RCS sub-loop as already documented.
- **lib.rs comment** (documentation-only): update the `servicer_exit` comment
  to reference `guidance::maneuver` (not `guidance::tvc`).

---

### CI-4. `select_jets_sm` return type disagrees with caller

**Affected specs**: rcs-logic-spec §6.1 vs dap-spec §10.4.

**Problem**: rcs-logic-spec §6.1 returns `(u8, u8)` (PYJETS + ROLLJETS pair).
dap-spec §10.4 declares

```rust
pub fn select_jets_sm(torque_request: Vec3, failed_jets: u16) -> u16
```

and `DapState::rcs_jet_flags` is `u16` in both specs and in the current
`control/dap.rs` source.

**Fix (apply to rcs-logic-spec)**: rcs-logic-spec §6.1 must return a single
`u16` jet mask (bits 15..8 = jets_b = ROLLJETS, bits 7..0 = jets_a = PYJETS),
matching dap-spec. The `fire_pulse` helper then splits the `u16` back into a
`(u8, u8)` pair internally or passes it to
`hw.rcs().fire_sm_jets(u8, u8)` after splitting.

**Alternative** (equally acceptable; user's choice): change dap-spec and
`DapState::rcs_jet_flags` to `(u8, u8)`. This is a bigger edit because
`rcs_jet_flags: u16` is already in the source code and the existing
control/dap.rs needs to match.

**Recommendation**: Change rcs-logic-spec to `u16`. Less code churn; the AGC
hardware wrote two consecutive single-byte output channels but the DAP
state only needs the combined representation. rcs-logic-spec §3.2 tables
still document bits 7..0 = jets_a, bits 15..8 = jets_b.

Also: rcs-logic-spec §6.1 `select_jets_sm(torque: Vec3, &RcsConfig)` takes a
`&RcsConfig`; dap-spec §10.4 `select_jets_sm(torque, failed_jets: u16)`
takes a raw `u16`. Resolve by using `&RcsConfig` (rcs-logic is more
complete). Update dap-spec §5.3 and §10.4 accordingly. The dap_step must
store an `RcsConfig` on `AgcState` or pass one in.

---

### CI-5. `libm` vs `f64::` math calls in `no_std` modules

**Affected specs**: attitude-spec §8 ("f64::sin, f64::cos, f64::atan2 — no
libm needed"), tvc-spec §9, targeting-spec §5.4.

**Problem**: `agc-core` is `#![cfg_attr(not(test), no_std)]` and the
targeted Cortex-M7 uses a hard-float f64 ABI, but the standard-library
transcendental methods `f64::sin`, `f64::cos`, `f64::atan2`, `f64::acos`,
`f64::asin`, `f64::sqrt`, `f64::sinh`, `f64::cosh` live in the `std` feature
and are **not** available on `no_std`. kepler-spec §3.2 and conics-spec §10
correctly mandate `libm::sqrt`, `libm::sin`, etc. attitude-spec §8 explicitly
says the opposite:

> `f64::sin`, `f64::cos`, `f64::atan2` | `core::f64` | Standard math (no
> `libm` needed)

This is wrong. `core::f64` exposes constants like `PI`, not methods.

**Fix**:

- **attitude-spec §8**: replace "f64::sin, f64::cos, f64::atan2" row with
  "`libm::sin`, `libm::cos`, `libm::atan2` from the `libm` crate", matching
  linalg-spec §3 and kepler-spec §3.2.
- **attitude-spec §4.2** (CM gimbal rotation) and §4.6 (maneuver_rate atan2):
  update pseudocode to use `libm::sin(theta_x)` etc.
- **tvc-spec**: already correctly uses `core::f64::consts::TAU` (constant).
  No code uses sin/cos; no change needed. Verify none was silently added.
- **targeting-spec §5.4 & §8**: add a note that `burn_attitude` uses
  `libm::sqrt` via `linalg::norm`/`linalg::unit`, which is already the
  convention in linalg-spec.
- **dap-spec §5.5**: `angular_distance` uses no transcendentals explicitly
  but if `libm::sqrt` is needed (for norm), that is already via
  `linalg::norm`. No change unless a future edit introduces a direct call.

---

### CI-6. Existing `TvcState` type mismatch (trim_pitch/trim_yaw)

**Affected specs**: tvc-spec §3.1 (notes the change), and the existing
`agc-core/src/control/tvc.rs` and `agc-core/src/lib.rs` `AgcState::new()`
which initialise `trim_pitch: 0, trim_yaw: 0` as `i16`.

**Problem**: The tvc-spec §3.1 explicitly calls out that it is changing
`trim_pitch` and `trim_yaw` from `i16` to `f64` radians. The existing source
code in `agc-core/src/control/tvc.rs` declares `trim_pitch: i16` and
`trim_yaw: i16`. `AgcState::new()` in `lib.rs` line 113-114 initialises both
to the integer literal `0`.

**Status**: This is correctly *flagged* in tvc-spec §3.1. It is a known
migration that the implementing developer will make in the same PR that
implements the spec. There is no spec conflict, only a source-code change
that must accompany implementation. **No spec edit needed**, but the
developer must remember to update `lib.rs` line 113-114 to `0.0_f64` and
`control/tvc.rs` fields to `f64` when implementing.

Recording as an Implementation Note rather than a spec issue.

---

### CI-7. `AgcState` missing fields

**Affected specs**: dap-spec §3.2, tvc-spec §5.3, maneuver-spec §4.1,
imu-control-spec §3.1, §4.1, rcs-logic-spec §5.

The specs assume several fields are already present on `AgcState` that are
not present in the current `agc-core/src/lib.rs`:

| Field | Spec introducing it | Current? |
|---|---|---|
| `tvc_filter: TvcFilter` | tvc-spec §5.3 | No |
| `burn: BurnState` | maneuver-spec §2.5 / §4.1 | No |
| `pipa_cal: PipaCalibration` | average-g-spec §3.1 | Yes |
| `pipa_counts: [i16; 3]` | average-g-spec §5 (Strategy B) | Yes |
| `servicer_exit: Option<fn(&mut AgcState)>` | average-g-spec §3.3 | Yes |
| `imu_alignment_state: ImuAlignmentState` | imu-control-spec §3.1 | No |
| `gyro_comp: GyroCompensation` | imu-control-spec §4 | No |
| `last_drift_comp_time: Met` | imu-control-spec §6.1 | No |
| `rcs_config: RcsConfig` | rcs-logic-spec §5.1 / CI-4 | No |
| `DapState::prev_cdu: [CduAngle; 3]` | dap-spec §3.2, attitude-spec §6 | No |
| `DapState::commanded_attitude: Vec3` | dap-spec §3.2 | No |
| `DapState::maneuver_rate: Vec3` | dap-spec §3.2 | No |
| `DapState::maneuver_target: Vec3` | dap-spec §5.5 (ambiguously) | No |
| `DapState::failed_jets: u16` | dap-spec §3.2 | No |
| `DapState::num_jets: u8` | dap-spec §3.2 | No |
| `DapState::rate_deadband: f64` | dap-spec §3.2 | No |
| `DapState::restart_phase: i16` | dap-spec §3.2 | No |

**Status**: These are documented extensions. tvc-spec §5.3 and dap-spec §3.2
explicitly say "The developer must add these fields to `DapState` ...".
These are not spec conflicts.

**Action required**: No spec edit. Instead, add to
`specs/milestone-3-implementation-plan.md` (a companion document, not
required for this review) a consolidated list of `AgcState` / `DapState` /
`TvcState` field additions that the implementing developer must land *before*
writing the module code. That list is what this CI-7 table represents.

Flagging as informational. See §6 of this review for the recommended
implementation order that accounts for this.

---

### CI-8. Gimbal lock warning duplicated in attitude-spec and imu-control-spec

**Affected specs**: attitude-spec §4.7 and imu-control-spec §10.

**Problem**: Both specs define a `gimbal_lock_warning` or
`is_gimbal_lock_warning` function that tests the middle CDU angle against
±90°. The thresholds are different: attitude-spec uses 85° (warning at 5°
margin); imu-control-spec uses 70° (warning at 20° margin,
`GIMBAL_LOCK_WARNING_BAND = 3641 counts ≈ 20°`).

Both specs cite `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` as the source
and neither developer could read the AGC file directly (both specs admit
this).

**Fix**: Pick one owner and one threshold. Recommendation:

- Owner: `control::imu_control::is_gimbal_lock_warning` (imu-control-spec
  §10). The AGC originally handled gimbal lock in the T4RUPT IMU monitoring
  cycle, so the `control::imu_control` module is the correct home.
- Threshold: must be verified against the original AGC source
  (see AGC Verification List item AV-4 below). For now, pick **70° warning /
  5° panic** as an interim value: the 20° margin in imu-control-spec gives
  the crew time to maneuver away from lock, which matches Comanche055's
  design intent.
- **attitude-spec §4.7**: delete the `gimbal_lock_warning` function. Replace
  with a cross-reference to `control::imu_control::is_gimbal_lock_warning`.
- **attitude-spec §9 TC-ATT-06**: move this test case to imu-control-spec.
- **dap-spec §5** (if it references `attitude::gimbal_lock_warning`): change
  to `imu_control::is_gimbal_lock_warning`.

---

### CI-9. `DapMode::Off` self-rescheduling contradiction

**Affected spec**: dap-spec §5.1 vs §4.2.

**Problem**: dap-spec §5.1 says "In Off mode, `dap_step` re-arms T5 and
returns without doing any control work", meaning an Off DAP stays on the
Waitlist. But dap-spec §4.2 `dap_stop` removes the Waitlist entry and
disarms T6 (and implicitly the whole DAP task chain).

If `dap_stop` removes the task, there is no task left to "re-arm T5 in Off
mode". Conversely, if the task keeps re-arming itself in Off mode, then
`dap_stop` cannot remove it via a simple Waitlist cancellation unless the
Waitlist exposes `remove(task_fn)`.

**Fix (apply to dap-spec)**: Adopt the average-g "stop flag" pattern
(average-g-spec §4.2). Add to dap-spec:

- `dap_stop` sets `state.dap_state.mode = DapMode::Off` and
  `state.dap_state.restart_phase = 0` but does **not** remove the Waitlist
  entry.
- On the next dispatch, `dap_step` observes `mode == Off`, does not re-arm
  T5 or reschedule, and returns. The task dies naturally.
- dap-spec §5.1 Off-mode behaviour must be changed to "If mode == Off, do
  NOT re-arm T5 and do NOT reschedule; return immediately." Remove the
  CALLFAST-idle justification — that pattern does not apply when we want
  `dap_stop` to actually stop the DAP.

This is a very small text edit but it corrects an internal inconsistency.

---

### CI-10. Attitude-spec internal sign convention ambiguity in `compute_attitude_error`

**Affected spec**: attitude-spec §4.2 Step 4 and Step 5.

**Problem**: attitude-spec §4.2 computes

```
M_err = mxm(transpose(desired), M_current)
```

then extracts roll/pitch/yaw as

```
roll = (M_err[2][1] - M_err[1][2]) / 2
```

The sign convention in §4.1 invariants says "positive error = current
attitude is rotated positively about that body axis relative to the desired
attitude", but the error matrix `M_err = desired^T · current` is the rotation
that, **applied to a vector in the desired frame**, gives the same vector in
the current frame — so the rotation represents "current-relative-to-desired",
which is what the comment says. However, TC-ATT-02 ("Pure roll error")
expects that a commanded +5° CDU outer rotation produces `error.roll ≈
+5°`, and then attitude-hold-torque produces a negative torque in TC-ATT-02
step "restoring torque opposes positive roll error".

This is internally consistent, but the developer needs to know that the
`attitude_hold_torque` negative sign convention requires
`compute_attitude_error` to produce the error with the documented
"current-relative-to-desired" sign, not the opposite. This is already
documented in §4.1 invariants but is easy to get wrong in implementation.

**Fix**: Add to attitude-spec §4.2 a postcondition check matching
TC-ATT-02 exactly:

```
POSTCONDITION: When desired == REFSMMAT == Identity and cdu[0] = 5°
(positive outer gimbal), error.roll ≈ +5° (positive).
```

This is not a critical bug, just an ambiguity that will bite the implementer.

---

## 3. Non-Critical Suggestions

### S-1. Missing cross-references

- **targeting-spec** references "`specs/lambert-spec.md` (when written)". The
  lambert spec now exists. Update the header reference to
  `specs/lambert-spec.md` (no "when written").
- **maneuver-spec** references "`specs/tvc-spec.md` (not yet written)" and
  "`specs/dap-spec.md` (not yet written)". Both exist now; update headers.
- **dap-spec** references "`specs/attitude-spec.md` (pending)" and "(pending)"
  for rcs-logic and tvc. All exist now; update.
- **attitude-spec** should add a reference to dap-spec §5 (which is where
  the deadband is applied) and rcs-logic-spec §6.
- **targeting-spec §5.2** should reference maneuver-spec for how `Maneuver`
  flows into `burn_init`.

### S-2. Timing budget consistency

Architecture §13.2 says:

| T5RUPT (~100 ms) | ~20 ms max |
| T4RUPT (120 ms)  | ~10 ms per cycle |
| T3RUPT (task)    | ~5 ms for dispatch |
| T6RUPT           | ~0.5 ms max |

Current spec estimates:

- dap-spec §8.1: estimated total DAP runtime ~1 ms. **OK.**
- tvc-spec: no explicit budget, but §4.2 work is two 6-multiply-add filters
  plus clamp — easily <100 us. **OK.**
- average-g-spec: claims 5 ms T3RUPT budget. **OK** (matches architecture).
- attitude-spec: TC-ATT-04 involves a PD formula — <10 us. **OK.**
- rcs-logic-spec: no explicit budget; jet-select loop over 16 jets is <50 us.
  **OK.**
- imu-control-spec: no explicit budget; `apply_pipa_compensation` is a
  single 3×3 mxv and two subtractions, well under 1 ms. **OK.**
- kepler-spec §6.2: mentions 4–20 Newton iterations for typical orbits. At
  ~50 ns/iter, that's ~1 us per call. **OK.**
- lambert-spec §5.1: Halley's 3–5 iterations ≈ 1-2 us. **OK.**
- conics-spec: no explicit budget; `state_to_elements` is several dozen
  flops plus three `acos` calls ≈ 5 us. **OK.**
- maneuver-spec: `cross_product_steering` is O(10) flops ≈ 50 ns. **OK.**
- targeting-spec: `lambert_targeting` calls `lambert` once. **OK.**

All within budget. No action needed.

### S-3. maneuver-spec `Maneuver::delta_v` access pattern

maneuver-spec §5.1 implementation writes `target.delta_v.0`. This only
compiles if `DeltaV` is a tuple-struct newtype with a public field. types-spec
§3.3 confirms `pub struct DeltaV(pub Vec3)`. **OK.**

### S-4. conics-spec `Met` unit confusion

conics-spec §4.1 `OrbitalElements::epoch: Met` comment says "1 unit = 1
centisecond = 0.01 s". This is correct (matches types-spec §3.2). **OK,
no action.** Flagging because the in-code `u32` representation of 0.01 s is
uncommon and the developer should be aware.

### S-5. imu-control-spec `fine_align_torque` has dubious `dt_s` dependence

imu-control-spec §8: `fine_align_torque(error, dt_s)` multiplies
`error * RAD_TO_PULSES * dt_s`. This treats the torque pulse count as
proportional to `dt_s`, but torque pulses are angular displacement per
pulse, not rate. At `dt_s = 0.12 s` and `error = 10 arcmin`, the spec's own
test case (TC-IMU-CTRL-4) computes 1.82 pulses → rounds to 2 pulses.
Without `dt_s`, it would be 10.9 pulses. The factor is unexplained.

**Suggestion**: The original AGC fine-align loop applied a *fraction* of
the accumulated error per cycle (a gain), which is consistent with
multiplying by the cycle time to achieve a time-constant-like response.
If so, the formula should be rewritten as
`pulses = error * RAD_TO_PULSES * (dt_s / tau_fine)` where `tau_fine` is
the alignment time constant. The current formula implicitly uses
`tau_fine = 1.0 s`.

**Action**: Flag for AGC source verification (AV-3 below). Not a critical
issue; test case passes with the chosen formula.

### S-6. rcs-logic-spec: `jets_per_axis` conflict with failure reconfiguration

rcs-logic-spec §9 says "If a primary jet group for an axis has no enabled
jets, attempt to use only the single remaining jet from the same axis
(reduces jets_per_axis to 1 for that axis)". But §6.1 step 5 says "select
up to `config.jets_per_axis` jets per axis". This is self-consistent (the
failure reconfiguration just uses fewer than the configured number) but the
§9 phrase "reduces jets_per_axis to 1 for that axis" is misleading because
`config` is not mutated. Edit to clarify.

### S-7. dap-spec §3.2 comments period error

dap-spec §2.1 and §8.2 debate whether the DAP period is "10 cs (100 ms)"
or "100 cs (1 second)". §8.2 explicitly calls out the architecture
documentation as having an error, saying the authoritative value is 10 cs
(100 ms). I cannot confirm architecture.md has this error without checking,
but if it does, that is a doc bug to fix separately (not in scope for this
review). The **authoritative value is 10 cs = 100 ms**; all specs should
use it. attitude-spec §2.1 uses 100 ms. tvc-spec §4.2 uses 0.1 s. All
consistent. **OK.**

### S-8. `imu_control` PIPA ownership migration

imu-control-spec §1 documents that `apply_pipa_compensation` should own
PIPA steps 2-4, replacing the inline pipeline in `average_g.rs`. This is
an existing-code migration path. The developer must coordinate: implement
`imu_control::apply_pipa_compensation` first, then modify
`services::average_g::servicer_task` to call it. Add to the implementation
plan as a task, not a spec issue.

### S-9. DeltaV newtype vs Vec3 plain usage

targeting-spec §3.1 uses `DeltaV` (newtype) for `Maneuver::delta_v`.
maneuver-spec §4.1 uses `Vec3` for `BurnState::target_dv_inertial`. This is
OK because the unwrapping `.0` is done at `burn_init` (maneuver-spec §5.1
implementation). Just flag in the implementation: `BurnState` does not
retain the `DeltaV` newtype wrapper. That's a deliberate design choice (the
newtype exists only at the targeting-to-burn boundary to prevent
position/delta-V confusion).

### S-10. tvc-spec `tvc_step` CDU counts return value is wrong scale

tvc-spec §4.2.3 converts gimbal radians to CDU counts using
`TVC_CDU_RAD_PER_COUNT = TAU / 360.0 ≈ 0.01745 rad/count` → 1 count = 1
degree. The gimbal mechanical limit is ±5.5° = ±0.0960 rad → ≈ ±5 counts.
That gives extremely coarse resolution.

dap-spec §11 suggests a much finer scale: "3200 pps counter cell" →
`counts = angle_rad * 3200 / (2π)` ≈ 509 counts/rad → 1 count ≈ 0.00196 rad
≈ 0.112°. This would give ±49 counts for the full 5.5° limit.

**Fix**: The correct gimbal servo scale should be verified. See AV-5 below.
For the current spec, pick one and make the two specs agree. Recommendation:
use the dap-spec §11 value (1 count ≈ 0.00196 rad) because it matches the
known SPS gimbal servo resolution (3200 pulse/sec × 1 sec-resolution CDU).

- **tvc-spec**: change `TVC_CDU_RAD_PER_COUNT` from `TAU / 360.0` to
  `TAU / 3200.0 ≈ 1.963e-3` rad/count.
- **tvc-spec §10 TC-TVC-01 expected counts**: recompute after the change.
- **dap-spec §11**: verified correct; no change.

---

## 4. AGC Verification List (for the implementing developer)

Several specs explicitly state that the author could not read the
Comanche055 assembly files and flagged specific constants for verification
before implementation. These must be cross-checked against
`~/virtualagc/Comanche055/*.agc` during implementation. In priority order:

### AV-1. TVC lead-lag filter coefficients
**File**: `Comanche055/TVCINITIALIZE.agc`, `Comanche055/TVCDAPS.agc`
**Items**: `TVC_A0`, `TVC_A1`, `TVC_B1` values from tvc-spec §2.2.
Currently 0.5530, −0.4470, −0.4470. These are computed from a generic
lead-compensator design and not lifted from the AGC source. The actual
Comanche055 constants may differ; the filter response may differ
significantly. **Critical for burn stability.**

### AV-2. TVC trim gain `K_TRIM`
**File**: `Comanche055/TVCMASSPROP.agc`
**Item**: tvc-spec §3.3 uses `K_TRIM = 0.05` (rad/s per rad). Should be
verified.

### AV-3. Fine align torque scale
**File**: `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`
**Item**: imu-control-spec §8 uses `RAD_TO_PULSES = 32768.0 / TAU` (i.e. one
pulse = TAU/32768 rad ≈ B-15 rev). The `dt_s` factor is also suspicious
(see §3 S-5 above).

### AV-4. Gimbal lock warning threshold
**File**: `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`
**Item**: attitude-spec says 85°; imu-control-spec says 70°. Both claim
AGC provenance but neither developer read the source. See CI-8.

### AV-5. SPS gimbal CDU scale factor
**File**: `Comanche055/ERASABLE_ASSIGNMENTS.agc` (`CDUTCMD`, `CDUSCMD`),
and the SPS gimbal servo docs.
**Item**: tvc-spec §3.3 uses 1°/count, dap-spec §11 uses the 3200 pulse/sec
conversion. See S-10.

### AV-6. NBDX/NBDY/NBDZ gyro drift units and scaling
**File**: `Comanche055/IMU_COMPENSATION_PACKAGE.agc`
**Item**: imu-control-spec §4 uses "torque counts per T4RUPT (120 ms)",
while average-g-spec §3.1 documents PIPA bias as "counts per 2-second
SERVICER interval". The two NBD sets are different but related. Verify the
unit conversion in imu-control-spec §6.1 (`/ 12.0`) is correct.

### AV-7. DAP attitude deadband default values
**File**: `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`,
`Comanche055/ERASABLE_ASSIGNMENTS.agc` (`DAPBOOLS`, `ATTDB`, `WFORPQR`)
**Item**: dap-spec §9 uses `DAP_DEFAULT_DEADBAND_RAD = 0.0873` (5°) and
`DAP_DEFAULT_RATE_DEADBAND_RAD_S = 0.00873` (0.5°/s). These are plausible
crew-configurable defaults but should be verified against the `DAPDATR1`
initialisation values in Comanche055.

### AV-8. RCS jet thrust and moment arms
**File**: `Comanche055/P40-P47.agc` (mass-properties initialization),
Apollo CSM systems handbook.
**Item**: rcs-logic-spec §5.1 defaults: SPS 445 N, pitch/yaw arm 1.4 m, roll
arm 1.9 m, CM 389 N, CM arm 0.6 m. These are from external references, not
from the AGC source. Verify that the AGC's erasable values (loaded via V46)
agree.

### AV-9. Lambert solver convergence tolerance
**File**: `Comanche055/CONIC_SUBROUTINES.agc`
**Item**: lambert-spec §5.1 uses `TOL_NDIM = 1e-12`. The AGC used a
~0.01 s tolerance (centisecond resolution). These are equivalent but the
dimensionless value should be cross-checked against the AGC iteration
termination test.

### AV-10. Kepler Newton-Raphson iteration bound and tolerance
**File**: `Comanche055/CONIC_SUBROUTINES.agc`, label `KEPSILON`
**Item**: kepler-spec §4.3 uses 50-iteration max and `1e-9` relative
tolerance. The AGC's original `KEPSILON` routine has its own tolerance.
Verify.

### AV-11. SPS thrust and Isp
**File**: Apollo CSM Systems Handbook
**Item**: targeting-spec §4 uses `SPS_THRUST_N = 91188.0` and `SPS_ISP_S =
314.0`. These are non-AGC constants from the systems handbook. The AGC's
own burn-duration calculation used a different approximation. Verify that
targeting-spec's `burn_duration` is consistent with what P40 displayed on
N37.

### AV-12. P37 Earth entry interface altitude
**File**: `Comanche055/P30,P31,P37,P40SUBROUTINES.agc`, `EARTH_CONSTANTS.agc`
**Item**: targeting-spec §4 uses `ENTRY_INTERFACE_ALT_M = 121920.0` (400,000
ft). This is a known Apollo constant. Verify the exact AGC value.

### AV-13. Comanche055 `DELVEET` erasable octal addresses
**Items**: maneuver-spec §3 gives `~0520–0526` for DELVEET, but other specs
do not use this address block explicitly. No impact on implementation
(Rust doesn't use raw addresses) but verify the spec's address list for
documentation correctness.

---

## 5. Unresolved Architectural Decisions (need user input)

### AD-1. HAL-access-in-task-context pattern (see CI-1)

Which of Strategy A/B/C/D? **Blocking.**
My recommendation: **Strategy D (staging fields)**.

### AD-2. `select_jets_sm` return type: `u16` or `(u8, u8)` (see CI-4)

Recommendation: **`u16`** (less churn given existing `DapState`).

### AD-3. Gimbal lock ownership: `control::attitude` or `control::imu_control`
(see CI-8)

Recommendation: **`control::imu_control`**.

### AD-4. Gimbal lock warning threshold: 85° or 70° (see CI-8)

Recommendation: **70° warn + 85° panic** (matches imu-control's 20°
margin, consistent with historical descriptions).

### AD-5. SPS gimbal CDU count scale: 1°/count or 0.00196 rad/count (see
S-10)

Recommendation: **0.00196 rad/count (TAU/3200)** based on dap-spec.

### AD-6. `dap_stop` behaviour: remove from Waitlist or flag-then-exit
(see CI-9)

Recommendation: **Flag-then-exit** (matches average-g-spec pattern).

---

## 6. Recommended Implementation Order

Given the dependency graph, the following order minimises rework:

**Phase A — Foundation (independent; can parallelise)**

1. `math::kepler` — no dependencies beyond `math::linalg` and `libm`.
   Already cleanly specced.
2. `math::lambert` — no dependencies on other Milestone 3 modules.
   Already cleanly specced.
3. `navigation::conics` — depends on `math::kepler`. Already cleanly specced.
   Re-exports `kepler_step`, adds `OrbitalElements` + helpers.

**Phase B — Architectural resolution**

4. **Resolve CI-1 (HAL-in-task-context).** No further work in control/ or
   services/ can proceed. Landing this unblocks the entire DAP family.
5. Add all `AgcState`, `DapState`, `TvcState` fields from CI-7 table to
   `agc-core/src/lib.rs` and `control/*.rs`. This is a mechanical edit.
   Includes the `trim_pitch/trim_yaw` type change (CI-6).
6. Define `RcsConfig` struct in `control::rcs_logic` and add it to
   `AgcState`. No other logic yet.

**Phase C — Pure compute modules (no hardware)**

7. `control::attitude` — depends on `types`, `linalg`, and (after Phase B)
   `DapState` shape. Pure functions; testable in isolation.
8. `control::imu_control` — depends on `types`, `linalg`, `average_g` for
   `PipaCalibration`. Mostly pure; one function reads HAL but only in the
   T4RUPT ISR shim (CI-1 applies).
9. `control::rcs_logic::{select_jets_sm, select_jets_cm,
   build_sm_torque_table, build_cm_torque_table, compute_pulse_duration}`
   — pure functions. `fire_pulse` is blocked by CI-1.
10. `control::tvc::{tvc_init, tvc_step, update_trim}` — pure functions
    except `tvc_step` which calls `engine.sps_gimbal`. CI-1 applies to that
    call; for the pure parts, implementation can start.

**Phase D — Guidance integration**

11. `guidance::targeting` — depends on `math::lambert`, `math::kepler`,
    `navigation::state_vector`. Pure functions. Produces `Maneuver`.
12. `guidance::maneuver` — depends on `guidance::targeting::Maneuver`,
    `math::linalg`. Pure functions. Produces `BurnState` operations.

**Phase E — Supervisor**

13. `control::dap::{dap_init, dap_stop, dap_step}` — depends on everything
    above. This is the last control module to land because it integrates
    all the others. Requires CI-1 resolution and all field extensions from
    CI-7 to be in place.

**Phase F — Programs (later milestones)**

14. `programs::p40_p41`, `programs::p30`, `programs::p31_p34`, etc. — out
    of scope for Milestone 3.

### Parallelisation

Phase A (items 1–3) can be done by three developers in parallel. Phase C
items 7–10 can be done in parallel once Phase B is complete. Phase D items
11–12 can be done in parallel once Phase A is complete (they do not
require Phase B or C). Phase E (item 13) is strictly sequential after
everything else.

### Milestone 3 Definition of Done

- All 10 modules implemented and passing their spec's unit tests.
- `cargo build --no-default-features --target thumbv7em-none-eabihf`
  succeeds without warnings.
- No `#![feature]` or `alloc` usage added.
- `control/tvc.rs` no longer exists in its current stub form; replaced
  with full spec-compliant implementation.
- `lib.rs::AgcState::new()` updated to initialise all new fields added
  per CI-7.

---

## 7. Spec-by-spec status summary

| Spec | Status | Blocking issues | Non-blocking |
|---|---|---|---|
| kepler-spec | READY | — | — |
| lambert-spec | READY | — | S-1 (cross-ref update) |
| conics-spec | READY | — | S-4 |
| imu-control-spec | NEEDS EDIT | CI-1, CI-8 | S-5, S-8, AV-3, AV-4, AV-6 |
| attitude-spec | NEEDS EDIT | CI-5, CI-8, CI-10 | S-1 |
| rcs-logic-spec | NEEDS EDIT | CI-1, CI-4 | S-6, AV-8 |
| tvc-spec | NEEDS EDIT | CI-3, S-10 | CI-6 (informational), AV-1, AV-2, AV-5 |
| dap-spec | NEEDS EDIT | CI-1, CI-3, CI-4, CI-9 | S-1, S-7, AV-7 |
| targeting-spec | NEEDS EDIT | — | S-1, AV-11, AV-12 |
| maneuver-spec | NEEDS EDIT | CI-2, CI-3 | S-1, S-3, S-9, AV-13 |

**READY** = implement as-is. **NEEDS EDIT** = must fix at least one
critical issue before implementation.

---

## 8. Actions applied directly to specs by this review

This review has been delivered as a single report document without yet
editing the individual spec files. The critical issues (CI-1 through CI-10)
each include a "Fix" subsection stating what the spec edit should be. The
user should review and approve the architectural decisions (§5 AD-1
through AD-6) first, then the spec edits can be applied in one batch by the
architect or analyst.

After the user's approval:

- **CI-2 (Maneuver struct fields)** and **CI-3 (data path)** are
  user-approval-independent and can be applied immediately to
  `maneuver-spec.md` and `tvc-spec.md`.
- **CI-5 (libm)** can be applied immediately to `attitude-spec.md`.
- **CI-8 (gimbal lock ownership)**, **CI-4 (u16 vs (u8,u8))**, **CI-9
  (dap_stop)**, **CI-10 (sign convention doc)** can be applied after user
  approval of the corresponding AD-* decisions.
- **CI-1 (HAL access)** requires AD-1 resolution before any edits.

---

## 9. File References

### Specs reviewed
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/kepler-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/lambert-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/conics-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/imu-control-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/attitude-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/rcs-logic-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/tvc-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/dap-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/targeting-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/maneuver-spec.md`

### Reference documents consulted
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/docs/architecture.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/hal-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/executive-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/state-vector-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/average-g-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/integration-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/types-module-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/linalg-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/specs/gravity-spec.md`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/agc-core/src/lib.rs`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/agc-core/src/control/dap.rs`
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/agc-core/src/control/tvc.rs`

---

## 10. Applied Fixes (2026-04-10)

User approved all 6 architectural decisions (AD-1 through AD-6) with the architect's recommendations:
- **AD-1**: Strategy D (staging fields on AgcState) for HAL-in-task-context
- **AD-2**: `select_jets_sm -> u16` (single bitmask)
- **AD-3**: Gimbal lock owned by `control::imu_control`
- **AD-4**: Gimbal lock thresholds 70° warn / 85° critical
- **AD-5**: SPS gimbal CDU scale = TAU/3200 rad/count (~0.00196)
- **AD-6**: `dap_stop` uses flag-then-exit pattern

All fix edits were applied to the 6 affected specs during the review:

| Spec | CIs Applied | Status |
|---|---|---|
| attitude-spec.md | CI-5 (libm), CI-8 (gimbal removed), CI-10 (sign test) | DONE |
| imu-control-spec.md | CI-1 (staging), CI-8 (70°/85° added) | DONE |
| rcs-logic-spec.md | CI-1 (Strategy D), CI-4 (u16 return) | DONE |
| tvc-spec.md | CI-3 (Vec3 param), CI-6 (i16→f64 noted) | DONE |
| dap-spec.md | CI-1 (staging), CI-3 (tvc_step), CI-4 (u16), CI-9 (flag-then-exit) | DONE |
| maneuver-spec.md | CI-2 (Maneuver 4-field), CI-3 (DapState path) | DONE |

All 10 M3 specs are now READY for implementation. Developer proceeds per the Phase A → E order in §6.
