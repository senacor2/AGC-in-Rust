# Specification: `programs/p30` — External Delta-V Targeting

**Status**: Ready for implementation
**Module path**: `agc-core/src/programs/p30.rs`
**Architecture reference**: `docs/architecture.md` §7.2 P30 row, §7.3 Program Trait
**Targeting reference**: `specs/targeting-spec.md` §5.1 `apply_external_delta_v`
**Maneuver types reference**: `specs/maneuver-spec.md` §4.1 `BurnState`, `specs/targeting-spec.md` §3.1 `Maneuver`
**State reference**: `agc-core/src/lib.rs` `AgcState`
**Dispatch table**: `agc-core/src/programs/mod.rs` `PROGRAM_TABLE[30]`
**AGC source**: `Comanche055/P30,P31,P37,P40SUBROUTINES.agc` (P30 entry block)
**Spec checklist**: `specs/README.md` — all items satisfied (see §11)

---

## 1. Purpose and Scope

P30 (External Delta-V) is the targeting program used when Mission Control has
computed a maneuver on the ground and uplinked (or verbally relayed) the result
to the crew. The crew data-loads the Time of Ignition (TIG) and the three-
component delta-V in the LVLH frame using the DSKY. P30 converts that LVLH
vector into an inertial delta-V and burn attitude, packages the result as a
`Maneuver`, and displays the burn summary for crew verification. The actual burn
execution is performed subsequently by P40 (SPS) or P41 (RCS), which read the
`pending_maneuver` field that P30 leaves in `AgcState`.

### What P30 provides

- `p30_init` — entry point registered in `PROGRAM_TABLE[30]`; sets major mode
  and schedules the crew data-load sequence.
- `p30_load_dv_lvlh` — called after the crew finishes entering TIG and delta-V
  components; drives all targeting computation and stores the result.
- `p30_display_summary` — populates the DSKY display registers for crew
  verification (delta-V magnitude, burn duration estimate, burn attitude CDU
  angles).

### What P30 does NOT provide

- LVLH-to-inertial conversion math. That is `guidance::targeting::apply_external_delta_v`.
- Burn attitude computation. That is `guidance::targeting::burn_attitude`.
- Burn duration estimation. That is `guidance::targeting::burn_duration`.
- Burn execution. That is `guidance::maneuver` and `programs::p40_p41`.
- DSKY data-load state machine (interactive V25 entry). That is Milestone 5
  (`services::v_n`). In this milestone the crew inputs are passed directly as
  function arguments.
- Navigation state propagation to TIG. P30 uses the current `AgcState::csm_state`
  as-is; a future enhancement propagates it to TIG using `math::kepler::kepler_step`.

---

## 2. AGC Background

### 2.1 Comanche055 Source References

| AGC source file | Content relevant to P30 |
|-----------------|------------------------|
| `P30,P31,P37,P40SUBROUTINES.agc` | P30 entry point, IMPULSIVE subroutine, delta-V data acceptance from crew noun entries |
| `ERASABLE_ASSIGNMENTS.agc` | `TIG` (octal 0350), `DELVEET1/2/3` (octal 0352–0356), `DVTOTAL`, `VGPREV` |
| `MAIN.agc` | Major-mode dispatch table; P30 entry address registered at index 30 |

In Comanche055, P30 proceeded as follows:

1. The program displayed V25 N33 (flashing) to request TIG entry. The crew
   keyed in the TIG in hours, minutes, and seconds relative to liftoff.
2. The program displayed V25 N81 (flashing) to request the three LVLH delta-V
   components (X, Y, Z) in units of feet per second (converted internally to
   m/s).
3. The subroutine `IMPULSIVE` was called to perform the LVLH-to-inertial
   rotation and store the result in `DELVEET1/2/3`.
4. The program displayed the burn summary via V06 N45 for crew acceptance, then
   entered a wait state for the crew to select P40 or P41.

The Rust port collapses the data-load state machine (steps 1–2) into a single
function call `p30_load_dv_lvlh` for the initial implementation, preserving the
computational contract of IMPULSIVE while deferring the interactive DSKY flow
to Milestone 5.

### 2.2 AGC Erasable Variables (P30 context)

| AGC symbol | Octal address | AGC scale | Rust equivalent |
|------------|---------------|-----------|-----------------|
| `TIG` | 0350 (E3) | B+28 centiseconds (double precision) | `state.pending_maneuver.tig` (`Met` = `u32` centiseconds) |
| `DELVEET1` | 0352 (E3) | B+7 m/s (DP) | `state.pending_maneuver.delta_v[0]` |
| `DELVEET2` | 0354 (E3) | B+7 m/s (DP) | `state.pending_maneuver.delta_v[1]` |
| `DELVEET3` | 0356 (E3) | B+7 m/s (DP) | `state.pending_maneuver.delta_v[2]` |
| `DVTOTAL` | ~0360 (E3) | B+7 m/s (scalar) | `linalg::norm(maneuver.delta_v)` computed on demand |
| `VGBODY` | P40 erasable | B+7 m/s | Not stored by P30; set at P40 entry |

Scale factor decode for test fixture data:

```
delta_v_component_mps = (w_hi * 2^-7 + w_lo * 2^-21)   [m/s]
tig_centiseconds      = (w_hi * 2^14 + w_lo)            [centiseconds]
```

### 2.3 LVLH Frame Convention

The input delta-V is expressed in the LVLH (Local Vertical Local Horizontal)
frame, ordered [R, S, W]:

| Index | Axis | Direction | Positive meaning |
|-------|------|-----------|-----------------|
| 0 | R | `unit(position)` | Radial outward (raises orbit) |
| 1 | S | `cross(W, R)` | In-track prograde (increases energy) |
| 2 | W | `unit(cross(position, velocity))` | Cross-track, normal to orbit plane |

Note: Comanche055 flight software documentation and crew procedures use the
ordering [X, Y, Z] for nouns, mapping to [along-track, radial, cross-track].
The `apply_external_delta_v` function uses the ordering [R, S, W] as documented
in `specs/targeting-spec.md` §2.2. P30 must pass the crew-entered [X, Y, Z]
components mapped to [S, R, W] as follows:

```
dv_lvlh[0] = dv_R  (crew Y entry — radial)
dv_lvlh[1] = dv_S  (crew X entry — along-track / prograde)
dv_lvlh[2] = dv_W  (crew Z entry — cross-track)
```

This mapping must be applied in `p30_load_dv_lvlh` before calling
`apply_external_delta_v`. The reordering is documented explicitly because it is
the most common source of sign errors when comparing P30 outputs against AGC
memory dumps.

### 2.4 Display Nouns Used by P30

| Noun | Verb | Content | Units |
|------|------|---------|-------|
| N33 | V25 (load) / V06 (display) | TIG (hours:minutes:seconds from liftoff) | centiseconds internally |
| N81 | V25 (load) / V06 (display) | Delta-V components [X, Y, Z] in LVLH | m/s (display in ft/s in original) |
| N45 | V06 | Burn summary: |ΔV|, TIG-35 min countdown, TIG-0 countdown | m/s, centiseconds |

In the Rust port, the DSKY display is modelled by writing to `AgcState::dsky.r`
(the three numeric register slots) and setting `dsky.noun` and `dsky.verb`.
The `p30_display_summary` function writes to these fields for both the N45 burn
summary and the N81 LVLH verification display.

---

## 3. New `AgcState` Field: `pending_maneuver`

P30 stores the computed `Maneuver` in a new field on `AgcState`:

```rust
/// The maneuver computed by the most recently run targeting program (P30,
/// P31, P34, or P37). Consumed by P40 (SPS) or P41 (RCS) when they start.
///
/// Set to `Some(maneuver)` by `p30_load_dv_lvlh` (and by the analogous
/// functions in P31/P34/P37). Set to `None` by P40/P41 on entry, after
/// transferring the maneuver into `BurnState` via `burn_init`.
///
/// AGC correspondence: `DELVEET1/2/3` + `TIG` in E3 erasable; the burn attitude
/// was not stored as a field in Comanche055 (it was derived by P40 at entry
/// from REFSMMAT + the delta-V direction); here it is carried in `Maneuver`
/// for type safety.
pub pending_maneuver: Option<Maneuver>,
```

This field must be added to the `AgcState` struct in `agc-core/src/lib.rs`.
Its initializer value in `AgcState::new()` is `None`.

**Note**: `Maneuver` is defined in `agc-core/src/guidance/targeting.rs` (see
`specs/targeting-spec.md` §3.1). The import `use crate::guidance::targeting::Maneuver;`
must be added to `lib.rs`.

---

## 4. Constants

```rust
// agc-core/src/programs/p30.rs

/// Major mode number for P30.
pub const P30_MAJOR_MODE: u8 = 30;

/// Job priority for the P30 targeting computation job.
/// P30 does not need to run at higher priority than other background jobs.
/// Chosen consistent with other targeting programs (P31, P34).
pub const P30_JOB_PRIORITY: JobPriority = JobPriority(4);

/// DSKY Noun used to display the burn summary.
pub const P30_NOUN_BURN_SUMMARY: u8 = 45;

/// DSKY Noun used to display and load LVLH delta-V components.
pub const P30_NOUN_DV_LVLH: u8 = 81;

/// DSKY Noun used to display and load TIG.
pub const P30_NOUN_TIG: u8 = 33;

/// DSKY Verb for displaying three registers (read-only).
pub const VERB_DISPLAY_OCT: u8 = 6;
```

---

## 5. Function Specifications

### 5.1 `p30_init`

```rust
/// P30 entry point — registered in `PROGRAM_TABLE[30]`.
///
/// Called by the verb-noun processor when the crew enters V37 N30 ENTER to
/// select the External Delta-V targeting program. Sets the major mode indicator,
/// establishes the P30 targeting job in the Executive, and requests the initial
/// TIG data load from the crew via DSKY.
///
/// # Arguments
///
/// * `state` — Mutable reference to the full AGC state.
///
/// # Returns
///
/// `JobPriority` for the Executive job that should be associated with P30's
/// targeting computation. The caller (program dispatcher) uses this to
/// register the job.
///
/// # Side effects
///
/// 1. Sets `state.major_mode = P30_MAJOR_MODE` (30).
/// 2. Sets `state.dsky.prog = P30_MAJOR_MODE` (updates PROG display to "30").
/// 3. Sets `state.dsky.verb = VERB_DISPLAY_OCT` (6).
/// 4. Sets `state.dsky.noun = P30_NOUN_TIG` (33) — indicates crew should
///    load TIG next (full interactive data-load is Milestone 5).
/// 5. Sets `state.dsky.flashing = true` — signals crew input required.
/// 6. Clears `state.pending_maneuver = None` — any prior targeting result
///    is invalidated when P30 restarts.
///
/// # Notes
///
/// The actual data-load state machine (V25 N33 / V25 N81 interactive sequence)
/// is part of the DSKY verb-noun processor, which is Milestone 5. This function
/// sets the DSKY state to indicate the initial request; the real data entry
/// flow will be driven by the V/N processor calling `p30_load_dv_lvlh` once
/// both TIG and delta-V have been entered.
///
/// Restart behaviour: on a restart while P30 is active, the program dispatcher
/// calls `p30_init` again from the top of phase 0. No phase table entries are
/// set by `p30_init` itself; computation phases are set by `p30_load_dv_lvlh`.
///
/// AGC correspondence: the P30 entry block in
/// `P30,P31,P37,P40SUBROUTINES.agc` sets major-mode flags and initiates the
/// V25 N33 DSKY request.
pub fn p30_init(state: &mut AgcState) -> JobPriority
```

**Signature as registered in `PROGRAM_TABLE`**:

```rust
// programs/p30.rs — the public `init` already declared in the stub;
// it must delegate to p30_init.
pub fn init(state: &mut crate::AgcState) -> JobPriority {
    p30_init(state)
}
```

### 5.2 `p30_load_dv_lvlh`

```rust
/// Process crew-entered TIG and LVLH delta-V, compute the inertial maneuver,
/// and store it in `AgcState::pending_maneuver`.
///
/// This is the computational heart of P30. In the full flight software it is
/// triggered by the DSKY V/N processor after the crew completes the data-load
/// sequence (V25 N33 for TIG, then V25 N81 for delta-V). In this milestone it
/// is called directly with the crew-entered values as arguments.
///
/// # Arguments
///
/// * `state` — Mutable reference to the full AGC state. Reads:
///   - `state.csm_state` — current navigation state vector (position and
///     velocity in the inertial frame). Used to construct the LVLH basis.
///   - `state.refsmmat` — current IMU alignment matrix. Passed to
///     `apply_external_delta_v` for burn attitude computation.
///
/// * `tig` — Time of Ignition as mission elapsed time in centiseconds (`Met`).
///   Crew-entered; must be strictly greater than the current time
///   `state.time` (cannot target a maneuver in the past). If `tig < state.time`
///   a program alarm 210 is raised and the function returns without modifying
///   `pending_maneuver`.
///
/// * `dv_crew` — Delta-V as entered by the crew via N81, in m/s,
///   ordered [along-track (X), radial (Y), cross-track (Z)].
///   This function reorders to [R, S, W] before calling `apply_external_delta_v`:
///   ```
///   dv_lvlh = [dv_crew[1],   // R: radial (crew Y component)
///              dv_crew[0],   // S: in-track (crew X component)
///              dv_crew[2]]   // W: cross-track (crew Z component)
///   ```
///
/// # Side effects
///
/// 1. Reorders `dv_crew` to [R, S, W] ordering (see §2.3).
/// 2. Calls `guidance::targeting::apply_external_delta_v(
///       state.csm_state, tig, dv_lvlh, state.refsmmat)`.
/// 3. Stores the returned `Maneuver` in `state.pending_maneuver = Some(maneuver)`.
/// 4. Calls `p30_display_summary(state)` to update the DSKY with the burn
///    parameters.
///
/// # Preconditions
///
/// - `state.csm_state.position != [0, 0, 0]` (non-degenerate position;
///   should always hold during mission operations).
/// - The angular momentum `cross(position, velocity)` is non-zero (non-rectilinear
///   trajectory; guaranteed during orbital phases).
/// - `tig >= state.time` (TIG is not in the past).
///
/// # Postconditions
///
/// - `state.pending_maneuver` is `Some(m)` where:
///   - `m.tig == tig`
///   - `|m.delta_v| == |dv_crew|` (rotation preserves magnitude)
///   - `m.mode == TargetingMode::ExternalDeltaV`
///   - `m.burn_attitude` is a valid rotation matrix
/// - DSKY displays are updated to show the burn summary.
///
/// # Error handling
///
/// If `tig < state.time`: raise program alarm 210 (targeting error), set
/// `state.alarm.code = 210`, set `state.alarm.lit = true`, and return without
/// modifying `pending_maneuver`. (Alarm code 210 is a placeholder; the
/// canonical alarm code table should be checked when alarm codes are fully
/// specified.)
///
/// AGC correspondence: the IMPULSIVE subroutine in
/// `P30,P31,P37,P40SUBROUTINES.agc`, which stores the result into
/// `DELVEET1/2/3` and `TIG`, then triggers the P30 summary display.
pub fn p30_load_dv_lvlh(state: &mut AgcState, tig: Met, dv_crew: Vec3)
```

### 5.3 `p30_display_summary`

```rust
/// Populate the DSKY display registers with the P30 burn summary.
///
/// Called automatically by `p30_load_dv_lvlh` after a successful targeting
/// computation. May also be called independently to refresh the display
/// (e.g., after a DSKY verb entry while P30 is active).
///
/// # Arguments
///
/// * `state` — Mutable reference to the AGC state. Reads `state.pending_maneuver`.
///
/// # Behaviour
///
/// If `state.pending_maneuver` is `None`, the function does nothing (no valid
/// maneuver to display).
///
/// If `state.pending_maneuver` is `Some(maneuver)`:
///
/// **Phase 1 — Burn summary (N45)**:
///
/// Sets DSKY to display Verb 6 Noun 45 with three register values:
///
/// | Register | Content | Units |
/// |----------|---------|-------|
/// | R1 | Delta-V magnitude `|maneuver.delta_v|` | m/s (displayed as f32) |
/// | R2 | TIG minus 35 minutes, in centiseconds | centiseconds (displayed as f32) |
/// | R3 | TIG in centiseconds (for countdown display) | centiseconds (displayed as f32) |
///
/// R2 provides the 35-minute pre-burn preparation cue used by the crew to
/// start the pre-burn checklist. It is computed as:
/// ```
/// tig_minus_35_cs = maneuver.tig - 35 * 60 * 100   [centiseconds]
/// ```
/// If `tig_minus_35_cs` would underflow (TIG is less than 35 minutes from
/// epoch), clamp to 0.
///
/// **Phase 2 — LVLH verification (N81)**:
///
/// The crew may request a re-display of the entered delta-V in LVLH for
/// verification. This is triggered via V06 N81 while P30 is active; the
/// `handle_display_input` method routes that verb/noun to `p30_display_summary`
/// (which always shows N45 first; N81 is a separate display mode not yet
/// implemented in this milestone). For now, `p30_display_summary` only
/// populates N45.
///
/// **Burn duration (informational)**:
///
/// The burn duration estimate from `guidance::targeting::burn_duration` is
/// computed using a nominal CSM mass of 28,000 kg if `state` does not yet
/// carry a vehicle mass field. This value is NOT placed on the DSKY registers
/// in this milestone (it is for future N37 display); it is computed and
/// discarded. When the vehicle mass field is added to `AgcState`, this call
/// should be updated to use it.
///
/// **DSKY fields written**:
/// ```
/// state.dsky.verb  = VERB_DISPLAY_OCT   (6)
/// state.dsky.noun  = P30_NOUN_BURN_SUMMARY   (45)
/// state.dsky.r[0]  = |delta_v| as f32        (m/s)
/// state.dsky.r[1]  = tig_minus_35_cs as f32  (centiseconds)
/// state.dsky.r[2]  = maneuver.tig.0 as f32   (centiseconds)
/// state.dsky.flashing = false
/// ```
///
/// AGC correspondence: the V06 N45 display sequence after IMPULSIVE completes
/// in `P30,P31,P37,P40SUBROUTINES.agc`.
pub fn p30_display_summary(state: &mut AgcState)
```

---

## 6. MajorMode Trait Implementation

P30 implements the `MajorMode` trait from `programs::mod`. The trait
implementation wraps the three free functions:

```rust
pub struct P30;

impl MajorMode for P30 {
    fn number(&self) -> u8 { P30_MAJOR_MODE }

    fn start(&self, state: &mut AgcState) -> JobPriority {
        p30_init(state)
    }

    fn handle_display_input(&self, state: &mut AgcState, verb: u8, noun: u8) {
        match (verb, noun) {
            (VERB_DISPLAY_OCT, P30_NOUN_BURN_SUMMARY) => p30_display_summary(state),
            // V25 N33 and V25 N81 are handled by the V/N processor (Milestone 5);
            // in this milestone they are no-ops here.
            _ => { /* unsolicited verb/noun; no action */ }
        }
    }

    fn restart_resume(&self, state: &mut AgcState, _phase: Phase) {
        // P30 has no long-running computation that needs mid-phase restart.
        // Re-enter from the top: re-init and redisplay if a maneuver exists.
        p30_init(state);
        p30_display_summary(state);
    }

    fn terminate(&self, state: &mut AgcState) {
        // Clear flashing indicator; leave pending_maneuver intact for P40/P41.
        state.dsky.flashing = false;
    }
}
```

The `P30` struct carries no data; it is a zero-sized type (ZST) used purely to
implement the trait. The `PROGRAM_TABLE` entry at index 30 calls the free
function `p30::init`, which is sufficient for the dispatch table.

---

## 7. Data Flow Diagram

```
Crew enters V25 N33 (TIG)
         |
         v
Crew enters V25 N81 (dv_X, dv_Y, dv_Z in LVLH)
         |
         v
p30_load_dv_lvlh(state, tig, dv_crew)
         |
         |-- reorder [X,Y,Z] -> [R,S,W]
         |
         v
guidance::targeting::apply_external_delta_v(
    csm_state, tig, dv_lvlh, refsmmat)
         |
         |-- lvlh_to_inertial(position, velocity) -> M
         |-- delta_v_inertial = M * dv_lvlh
         |-- burn_attitude(delta_v_inertial, refsmmat) -> attitude
         |
         v
Maneuver { tig, delta_v_inertial, burn_attitude, mode: ExternalDeltaV }
         |
         v
state.pending_maneuver = Some(maneuver)
         |
         v
p30_display_summary(state)
         |
         |-- norm(delta_v) -> |dv|
         |-- tig - 35 min -> R2
         |-- write dsky.r[0..2], noun=45, verb=6
         |
         v
Crew reviews N45 display
         |
         v
Crew selects P40 (SPS) or P41 (RCS)
         |
         v
P40/P41 reads state.pending_maneuver, calls burn_init(maneuver)
state.pending_maneuver = None (consumed by P40/P41)
```

---

## 8. Module Dependencies

```
agc-core::programs::p30
    |
    +-- calls:   agc-core::guidance::targeting::apply_external_delta_v
    +-- calls:   agc-core::guidance::targeting::burn_duration  (display estimate)
    +-- calls:   agc-core::math::linalg::norm                  (|dv| for display)
    +-- reads:   agc-core::AgcState::{csm_state, refsmmat, time, dsky, alarm,
    |                                  pending_maneuver, major_mode}
    +-- writes:  agc-core::AgcState::{pending_maneuver, major_mode, dsky, alarm}
    |
    +-- uses types: agc-core::types::{Met, Vec3}
    +-- uses types: agc-core::guidance::targeting::{Maneuver, TargetingMode}
    +-- uses types: agc-core::executive::job::JobPriority
    +-- uses types: agc-core::executive::restart::Phase
    +-- implements: agc-core::programs::MajorMode
    +-- registered in: agc-core::programs::PROGRAM_TABLE[30]
```

No circular dependencies. `programs::p30` does NOT call back into
`guidance::maneuver` or `programs::p40_p41`.

---

## 9. `AgcState` Changes Required

The following change must be made to `agc-core/src/lib.rs` during implementation:

1. **Add import**:
   ```rust
   use guidance::targeting::Maneuver;
   ```

2. **Add field** to the `AgcState` struct (after `burn: BurnState`):
   ```rust
   /// Maneuver computed by the most recently run targeting program (P30,
   /// P31, P34, or P37). Consumed by P40/P41 on entry.
   /// `None` until a targeting program completes successfully.
   pub pending_maneuver: Option<Maneuver>,
   ```

3. **Initialize** in `AgcState::new()`:
   ```rust
   pending_maneuver: None,
   ```

The `Maneuver` type is `Copy` (it contains only `f64` arrays, a `u32`, and a
`Copy` enum), so `Option<Maneuver>` is also `Copy` and requires no heap
allocation.

---

## 10. Test Cases

### TC-P30-1: Zero delta-V produces zero inertial delta-V and identity burn attitude

**Purpose**: Verify the identity case — a zero LVLH delta-V results in a
`Maneuver` with zero `delta_v` and identity `burn_attitude`. This is the
P30 no-maneuver baseline.

```rust
#[test]
fn tc_p30_1_zero_delta_v() {
    use crate::types::{Met, Vec3};
    use crate::guidance::targeting::TargetingMode;
    use crate::math::linalg::norm;

    let mut state = AgcState::new();

    // ISS-like circular LEO at 400 km altitude, equatorial
    let r = 6_778_137.0_f64;   // metres (R_Earth + 400 km)
    let v_circ = (3.986_004_418e14_f64 / r).sqrt();   // ~7784 m/s

    state.csm_state.position = [r, 0.0, 0.0];
    state.csm_state.velocity = [0.0, v_circ, 0.0];
    state.time = Met(0);
    state.refsmmat = [[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]];

    let tig = Met(360_000);   // TIG = 1 hour from epoch (centiseconds)
    let dv_crew: Vec3 = [0.0, 0.0, 0.0];

    p30_load_dv_lvlh(&mut state, tig, dv_crew);

    let m = state.pending_maneuver.expect("pending_maneuver must be Some after p30_load_dv_lvlh");

    assert_eq!(m.tig, tig);
    assert!(norm(m.delta_v) < 1e-9, "delta_v magnitude must be zero");
    assert_eq!(m.mode, TargetingMode::ExternalDeltaV);

    // burn_attitude must be identity for zero delta-V
    let id = [[1.0_f64, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for i in 0..3 {
        for j in 0..3 {
            assert!((m.burn_attitude[i][j] - id[i][j]).abs() < 1e-9,
                "burn_attitude[{}][{}] must be identity", i, j);
        }
    }
}
```

### TC-P30-2: Prograde-only delta-V maps to inertial +V direction

**Purpose**: Verify that a prograde LVLH delta-V (along-track, positive X in
crew convention — i.e., S-axis in LVLH) converts to an inertial vector
that is parallel to the spacecraft velocity direction. For a circular equatorial
orbit with position along +X and velocity along +Y, a prograde (S-axis) delta-V
must appear in the inertial +Y direction.

```rust
#[test]
fn tc_p30_2_prograde_dv_maps_to_velocity_direction() {
    use crate::types::{Met, Vec3};
    use crate::math::linalg::norm;

    let mut state = AgcState::new();

    let r = 6_778_137.0_f64;
    let v_circ = (3.986_004_418e14_f64 / r).sqrt();

    // Position along +X, velocity along +Y (equatorial circular orbit)
    state.csm_state.position = [r, 0.0, 0.0];
    state.csm_state.velocity = [0.0, v_circ, 0.0];
    state.time = Met(0);
    state.refsmmat = [[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]];

    let tig = Met(360_000);
    // Crew enters [X=100.0, Y=0.0, Z=0.0] — 100 m/s prograde (X = along-track)
    let dv_crew: Vec3 = [100.0, 0.0, 0.0];

    p30_load_dv_lvlh(&mut state, tig, dv_crew);

    let m = state.pending_maneuver.expect("pending_maneuver must be Some");

    // The inertial delta-V must be approximately [0, 100, 0] (parallel to +Y velocity)
    let dv = m.delta_v;
    assert!((dv[0]).abs() < 1e-6, "inertial dv[0] (X) must be ~0, got {}", dv[0]);
    assert!((dv[1] - 100.0).abs() < 1e-6, "inertial dv[1] (Y) must be ~100 m/s, got {}", dv[1]);
    assert!((dv[2]).abs() < 1e-6, "inertial dv[2] (Z) must be ~0, got {}", dv[2]);

    // Magnitude must be preserved
    assert!((norm(dv) - 100.0).abs() < 1e-9, "delta-V magnitude must be preserved at 100 m/s");
}
```

### TC-P30-3: `p30_init` sets major_mode = 30 and updates DSKY PROG display

**Purpose**: Verify that the entry point correctly sets the major mode field
and the DSKY program indicator, as required by the MajorMode contract.

```rust
#[test]
fn tc_p30_3_init_sets_major_mode() {
    let mut state = AgcState::new();
    state.major_mode = 0;   // start in P00

    let _ = p30_init(&mut state);

    assert_eq!(state.major_mode, 30, "major_mode must be set to 30 by p30_init");
    assert_eq!(state.dsky.prog, 30, "dsky.prog must reflect major mode 30");
    assert_eq!(state.dsky.noun, P30_NOUN_TIG, "dsky.noun must be 33 (TIG entry cue)");
    assert!(state.dsky.flashing, "dsky.flashing must be true to signal crew input required");
    assert!(state.pending_maneuver.is_none(), "pending_maneuver must be cleared on init");
}
```

### TC-P30-4: `p30_load_dv_lvlh` stores result in `state.pending_maneuver`

**Purpose**: Verify the persistence contract — after `p30_load_dv_lvlh`
returns, `pending_maneuver` is `Some` and carries the correct TIG and mode.

```rust
#[test]
fn tc_p30_4_stores_pending_maneuver() {
    use crate::types::Met;
    use crate::guidance::targeting::TargetingMode;

    let mut state = AgcState::new();

    let r = 6_778_137.0_f64;
    let v_circ = (3.986_004_418e14_f64 / r).sqrt();
    state.csm_state.position = [r, 0.0, 0.0];
    state.csm_state.velocity = [0.0, v_circ, 0.0];
    state.time = Met(0);
    state.refsmmat = [[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]];

    // Verify pending_maneuver starts as None
    assert!(state.pending_maneuver.is_none());

    let tig = Met(720_000);   // 2 hours
    let dv_crew = [50.0_f64, 10.0, -5.0];

    p30_load_dv_lvlh(&mut state, tig, dv_crew);

    let m = state.pending_maneuver.expect("pending_maneuver must be Some after load");
    assert_eq!(m.tig, tig, "tig must match the input TIG");
    assert_eq!(m.mode, TargetingMode::ExternalDeltaV,
        "mode must be ExternalDeltaV for P30");

    // Verify DSKY summary was also populated
    assert_eq!(state.dsky.noun, P30_NOUN_BURN_SUMMARY,
        "dsky.noun must be 45 after display_summary");
    assert_eq!(state.dsky.verb, VERB_DISPLAY_OCT,
        "dsky.verb must be 6 after display_summary");
}
```

---

## 11. Spec Quality Checklist

- [x] AGC source file referenced: `P30,P31,P37,P40SUBROUTINES.agc`, `ERASABLE_ASSIGNMENTS.agc`, `MAIN.agc`
- [x] All erasable variables and AGC addresses listed (§2.2)
- [x] Scale factors documented for all fixed-point values (§2.2)
- [x] Corresponding `f64` SI units documented
- [x] Input/output preconditions and postconditions stated for each function
- [x] Edge cases documented: zero delta-V, TIG in past, gimbal singularity guard (delegated to `burn_attitude`)
- [x] At least 3 test cases with expected values (4 test cases, §10)
- [x] Rust API signatures designed with types and ownership
- [x] Invariants explicitly stated (Maneuver postconditions, §5.2)
- [x] Consistency with `docs/architecture.md` checked: `no_std`, `AgcState`, `MajorMode` trait, `PROGRAM_TABLE`, `JobPriority`, `Met`, `Vec3`, `Mat3x3`
- [x] New `AgcState` field specified with initializer (§3, §9)
- [x] LVLH axis-ordering mapping explicitly documented (§2.3) — most common source of sign errors
- [x] Dependency graph is acyclic (§8)
- [x] Deferred items clearly marked (Milestone 5 interactive DSKY flow, vehicle mass field, N81 re-display)
