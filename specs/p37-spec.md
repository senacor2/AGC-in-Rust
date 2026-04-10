# Specification: `programs/p37` — Return to Earth (Trans-Earth Injection)

**Status**: Ready for implementation
**Module path**: `agc-core/src/programs/p37.rs`
**Architecture reference**: `docs/architecture.md` §7.2 "Programs for the Command Module", §7.3 "Program Trait"
**Targeting reference**: `specs/targeting-spec.md` §5.3 `return_to_earth`, §3.1 `Maneuver`, §3.2 `TargetingMode`
**Navigation reference**: `specs/state-vector-spec.md` §4.1 `Frame`, §4.2 `StateVector`
**Gravity reference**: `specs/gravity-spec.md`; `agc-core/src/navigation/gravity.rs` — `MU_MOON`, `R_EARTH`, `R_SOI_MOON`
**Sibling programs**: `agc-core/src/programs/p30.rs` (same stub pattern), `agc-core/src/programs/p31_p34.rs`
**AGC source reference**: `Comanche055/P30,P37.agc` and `Comanche055/P37,P70.agc`
  - P37 entry point and major-mode setup
  - Noun 45 (V06N45) display for TEI burn summary
  - TEICOMP — TEI delta-V computation using CONIC_SUBROUTINES LAMBERT
  - `Comanche055/ERASABLE_ASSIGNMENTS.agc` — TIG (octal 0350), DELVEET1/2/3 (octal 0352–0356)
**Spec checklist**: `specs/README.md` — all items satisfied (see §12)

---

## 1. Purpose and Scope

P37 (Return to Earth) is the contingency program that computes a Trans-Earth
Injection (TEI) burn from lunar orbit to Earth reentry. The crew can execute
P37 independently of Mission Control — it was central to the Apollo 13 abort
scenario, where the free-return trajectory required a TEI burn computed and
executed without ground support.

P37 operates exclusively while the CSM is in lunar orbit: the current navigation
state must be in `Frame::MoonInertial`. It takes crew-entered targeting parameters
(time of ignition, desired landing site, and coast duration), constructs the Earth
entry interface target vector in the MCI frame, calls
`guidance::targeting::return_to_earth` to solve the Lambert transfer, and stores
the resulting `Maneuver` in `AgcState` for subsequent execution by P40.

### What P37 provides

- `p37_init` — entry point registered in `programs::PROGRAM_TABLE[37]`; sets
  `major_mode = 37` and dispatches the TEI computation job.
- `p37_compute_tei` — constructs the entry target vector from crew inputs and
  calls `return_to_earth`; stores the resulting `Maneuver`.
- `p37_display_summary` — populates the DSKY with the TEI burn summary display
  (V06N45: TIG, delta-V magnitude, burn duration).

### What P37 does NOT provide

- Lambert iteration. That is `math::lambert::lambert` (via `guidance::targeting`).
- Entry corridor verification or flight-path angle iteration. In the initial
  implementation, the first-pass Lambert solution is accepted directly. Iteration
  on TOF to satisfy a flight-path angle constraint is a future enhancement in
  this module.
- SPS burn execution. That is `programs::p40_p41`.
- Lunar ephemeris (Earth position in MCI frame). The initial implementation uses
  a static geometric approximation: Earth at `[-D_EARTH_MOON, 0, 0]` in MCI.
- Actual DSKY keypad I/O (not yet implemented in the Rust port; crew inputs are
  injected directly into `AgcState` fields for testing).

---

## 2. AGC Background

### 2.1 Comanche055 Source References

| AGC source file | Content relevant to this module |
|-----------------|----------------------------------|
| `P30,P37.agc` | P37 entry point, major-mode switch to 37, initial display sequence |
| `P37,P70.agc` | TEICOMP — TEI computation calling LAMBERT, iteration on entry angle; P70 (abort from lunar surface) uses the same targeting routines |
| `CONIC_SUBROUTINES.agc` | LAMBERT universal-variable solver; KEPRTN Kepler propagation |
| `ERASABLE_ASSIGNMENTS.agc` | TIG (octal 0350 E3), DELVEET1/2/3 (octal 0352–0356 E3); TEPHEM (epoch of state vector) |

### 2.2 Relevant AGC Erasable Variables

| AGC symbol | Address (octal) | Scale | Rust equivalent |
|------------|-----------------|-------|-----------------|
| `TIG` | 0350 (E3) | B+28 centiseconds | `Maneuver::tig` (`Met`) |
| `DELVEET1` | 0352 (E3) | B+7 m/s (DP) | `Maneuver::delta_v[0]` |
| `DELVEET2` | 0354 (E3) | B+7 m/s (DP) | `Maneuver::delta_v[1]` |
| `DELVEET3` | 0356 (E3) | B+7 m/s (DP) | `Maneuver::delta_v[2]` |
| `TEPHEM` | 0340 (E3) | B+28 centiseconds | `AgcState::csm_state.epoch` |

In Comanche055, the result of TEICOMP was stored into these same erasable cells
that P30 and P31 use. P37 is therefore **mutually exclusive** with any other
pending maneuver: storing a new `Maneuver` overwrites any previous targeting
solution. This is reflected in the Rust port by the single `AgcState::pending_maneuver`
field (see §4.1).

### 2.3 TEI Mission Profile

A nominal TEI from low lunar orbit (LLO, ~100 km altitude) requires a prograde
burn of approximately 900–1100 m/s. The resulting trans-Earth trajectory has a
time of flight of approximately 60–66 hours from TIG to entry interface. The
entry interface is defined at 121,920 m (400,000 ft) above the Earth's surface.

For Apollo 13, the abort TEI from the free-return trajectory used the LM descent
engine, but the targeting geometry — Lambert from lunar vicinity to Earth entry
corridor — was the same algorithm implemented in P37.

The AGC P37 algorithm iterated on the time of flight until the arrival velocity
at the entry interface made the required flight-path angle (nominally −6.5° for
a skip-entry corridor). In the Rust port, this TOF iteration is deferred: the
initial implementation accepts the first-pass Lambert result. The crew-entered
TOF is used directly without iteration.

### 2.4 Geometric Approximation: Earth Position in MCI

`return_to_earth` in `guidance::targeting` expects the entry target position
expressed in the same frame as the current state vector — in this case, MCI
(Moon-centered inertial). The Earth is not at the MCI origin.

The exact Earth position in MCI at a given epoch requires a planetary ephemeris
lookup (`navigation::planetary`). For the initial P37 implementation, a static
approximation is used: Earth is at `[-D_EARTH_MOON, 0, 0]` in MCI, where
`D_EARTH_MOON = 384_400_000.0` m (mean Earth-Moon distance). This approximation
introduces an error proportional to the Moon's displacement from the mean position,
which is acceptable for a contingency computation but should be replaced with an
ephemeris call in a future iteration.

The entry target position vector in MCI is therefore:

```
earth_center_mci ≈ [-D_EARTH_MOON, 0, 0]

entry_target_mci = earth_center_mci + (R_EARTH + ENTRY_INTERFACE_ALT_M) * unit(landing_direction_mci)
```

For the initial implementation, `landing_direction_mci` defaults to the direction
from the Moon toward Earth (i.e., `[1, 0, 0]` in MCI, pointing toward Earth).
This places the entry target at the sub-Earth point on the entry interface sphere,
which is a valid contingency aim point. Crew-entered latitude/longitude targeting
is a future enhancement.

---

## 3. Data Types

P37 uses the types defined in the targeting and navigation modules. No new types
are introduced.

### 3.1 `AgcState::pending_maneuver` Field

P37 requires `AgcState` to carry a field for the computed-but-not-yet-executed
targeting solution:

```rust
// In agc-core/src/lib.rs — AgcState struct
/// Targeting solution computed by P30, P31, P34, or P37.
/// Consumed by P40 at ignition. None = no pending maneuver.
/// P37 stores a ReturnToEarth maneuver here; P30 stores an ExternalDeltaV maneuver.
/// These programs are mutually exclusive; storing a new result overwrites any prior one.
pub pending_maneuver: Option<Maneuver>,
```

**Note**: `AgcState` does not currently include this field (as of the P37 spec
date). The implementer must add it to `AgcState` in `agc-core/src/lib.rs`, adding
`None` to the `AgcState::new()` initializer, and importing `Maneuver` from
`guidance::targeting`.

### 3.2 Types Used

All types are defined in `agc-core/src/`:

| Type | Definition | Unit |
|------|-----------|------|
| `Vec3` | `[f64; 3]` | metres or m/s |
| `Mat3x3` | `[[f64; 3]; 3]` | dimensionless (rotation) |
| `Met` | `u32` | centiseconds |
| `Maneuver` | struct: `tig`, `delta_v`, `burn_attitude`, `mode` | SI |
| `StateVector` | struct: `position`, `velocity`, `epoch`, `frame` | SI |
| `Frame` | enum: `EarthInertial`, `MoonInertial`, ... | — |
| `JobPriority` | `u8` | — |

---

## 4. Constants

```rust
// agc-core/src/programs/p37.rs

/// Default TIG offset from current MET: 30 minutes in centiseconds.
/// Provides the crew time to verify the solution before committing.
/// AGC: Comanche055 P37 default TIG was mission-specific; 30 min is a
/// reasonable lower bound for a lunar-orbit TEI computation window.
pub const DEFAULT_TEI_TIG_OFFSET_CS: u32 = 180_000;

/// Default TEI time of flight: 60 hours in seconds.
/// Represents the nominal free-return trans-Earth coast duration.
/// Typical Apollo mission values ranged from 57 to 66 hours.
pub const DEFAULT_TEI_TOF_S: f64 = 216_000.0;

/// Earth entry interface altitude above the Earth surface (metres).
/// Matches `guidance::targeting::ENTRY_INTERFACE_ALT_M` = 400,000 ft = 121,920 m.
/// Do not redefine; import from `guidance::targeting`.
pub use crate::guidance::targeting::ENTRY_INTERFACE_ALT_M;

/// Mean Earth-Moon distance (metres). Used for the static Earth position
/// approximation in MCI. See §2.4.
/// Source: IAU nominal value.
pub const D_EARTH_MOON_M: f64 = 384_400_000.0;

/// Minimum acceptable TEI time of flight (seconds): 24 hours.
/// A trans-Earth coast shorter than 24 hours implies an impossibly steep
/// trajectory or an erroneous entry. Guard against crew data-entry errors.
pub const MIN_TEI_TOF_S: f64 = 86_400.0;

/// Maximum acceptable TEI time of flight (seconds): 120 hours.
/// Represents an absolute upper bound; free-return trajectories take at most
/// ~66 hours. Values above 120 hours indicate a crew data-entry error.
pub const MAX_TEI_TOF_S: f64 = 432_000.0;
```

These constants import or re-use values already defined elsewhere:

- `R_EARTH` from `navigation::gravity` (= 6,378,137 m)
- `MU_MOON` from `navigation::gravity` (= 4.902_800_118e12 m³/s²)
- `ENTRY_INTERFACE_ALT_M` from `guidance::targeting` (= 121,920 m)

---

## 5. Function Specifications

### 5.1 `p37_init`

```rust
/// P37 entry point — Return to Earth targeting.
///
/// Called by the major-mode dispatcher when the crew selects P37 via V37 N37 ENTER.
/// Sets `state.major_mode = 37`, computes the default TIG from the current MET,
/// and dispatches the TEI computation.
///
/// # Arguments
///
/// * `state` — Mutable reference to the AGC global state.
///
/// # Returns
///
/// `JobPriority` for the Executive job that will execute the TEI computation.
/// Use priority `0x10` (decimal 16), matching the priority used by targeting
/// programs P30 and P31 (non-time-critical background computation).
///
/// # Behavior
///
/// 1. Assert `state.csm_state.frame == Frame::MoonInertial`. P37 is only valid
///    while the spacecraft is in the Moon's sphere of influence. If the frame is
///    anything else, trigger program alarm (or panic in the initial implementation).
///    See §7 for the alarm policy.
/// 2. Set `state.major_mode = 37`.
/// 3. Set `state.dsky.prog = 37` (PROG display field).
/// 4. Compute the default TIG: `tig = state.time.0.saturating_add(DEFAULT_TEI_TIG_OFFSET_CS)`.
/// 5. Call `p37_compute_tei(state, Met(tig), DEFAULT_TEI_TOF_S)`.
/// 6. Return `JobPriority` = 16 (`0x10`).
///
/// # AGC source
///
/// `Comanche055/P30,P37.agc` — P37 entry, major-mode switch, initial display.
pub fn p37_init(state: &mut AgcState) -> JobPriority
```

The public signature exposed through `PROGRAM_TABLE` is:

```rust
pub fn init(state: &mut crate::AgcState) -> JobPriority
```

This thin wrapper simply calls `p37_init(state)` and returns its result.

### 5.2 `p37_compute_tei`

```rust
/// Compute the Trans-Earth Injection maneuver and store it as the pending burn.
///
/// Constructs the Earth entry target position vector in MCI, then calls
/// `guidance::targeting::return_to_earth` to solve the Lambert transfer from
/// the current CSM state. The resulting `Maneuver` is stored in
/// `state.pending_maneuver`.
///
/// This function performs a single Lambert evaluation (no TOF iteration). The
/// crew can call P37 again with a different TOF to iterate manually. Automatic
/// TOF iteration for flight-path-angle targeting is deferred to a future
/// enhancement.
///
/// # Arguments
///
/// * `state` — Mutable reference to the AGC global state.
///
/// * `tig` — Time of Ignition as mission elapsed time (centiseconds).
///   Determines the epoch at which the Lambert computation is based.
///   Must satisfy `tig.0 >= state.csm_state.epoch.0`.
///
/// * `tof` — Time of Flight from TIG to Earth entry interface (seconds).
///   Must satisfy `MIN_TEI_TOF_S <= tof <= MAX_TEI_TOF_S`.
///   Panic (or alarm) if outside this range; see §7.
///
/// # Algorithm
///
/// 1. Validate `MIN_TEI_TOF_S <= tof <= MAX_TEI_TOF_S`; panic if violated.
/// 2. Validate `state.csm_state.frame == Frame::MoonInertial`; panic if violated.
/// 3. Construct the Earth center position in MCI using the static approximation:
///    ```
///    earth_mci: Vec3 = [-D_EARTH_MOON_M, 0.0, 0.0]
///    ```
/// 4. Compute the entry interface radius:
///    ```
///    r_ei = R_EARTH + ENTRY_INTERFACE_ALT_M   // from gravity.rs and targeting.rs
///    ```
/// 5. Compute the entry target position in MCI (sub-Earth point on entry sphere):
///    ```
///    // Direction from Moon toward Earth in MCI (positive x toward Earth)
///    entry_dir_mci: Vec3 = [1.0, 0.0, 0.0]
///    entry_target_mci: Vec3 = [
///        earth_mci[0] + r_ei * entry_dir_mci[0],
///        earth_mci[1] + r_ei * entry_dir_mci[1],
///        earth_mci[2] + r_ei * entry_dir_mci[2],
///    ]
///    // = [-D_EARTH_MOON_M + r_ei, 0.0, 0.0]
///    ```
///    Note: the sub-Earth point is in the +x MCI direction from the Moon's center,
///    so `entry_target_mci[0]` = `-D_EARTH_MOON_M + r_ei` ≈ -377.9 × 10⁶ m.
///    This is the point on the entry interface sphere closest to the Moon —
///    a physically valid contingency aim point.
/// 6. Propagate the CSM state from `state.csm_state.epoch` to `tig` using
///    `math::kepler::kepler_step` with `MU_MOON`. Store the propagated state as
///    a local `StateVector` with `epoch = tig`.
///    If `tig.0 == state.csm_state.epoch.0`, skip propagation and use
///    `state.csm_state` directly.
/// 7. Call:
///    ```rust
///    let maneuver = guidance::targeting::return_to_earth(
///        state_at_tig,          // propagated CSM state, epoch = tig
///        entry_target_mci,      // target position in MCI
///        tof,                   // TOF in seconds
///        state.refsmmat,        // for burn_attitude computation
///    );
///    ```
/// 8. Store: `state.pending_maneuver = Some(maneuver)`.
/// 9. Call `p37_display_summary(state)`.
///
/// # Preconditions
///
/// - `state.csm_state.frame == Frame::MoonInertial` (checked at entry).
/// - `tig.0 >= state.csm_state.epoch.0` (TIG must not be in the past).
/// - `MIN_TEI_TOF_S <= tof <= MAX_TEI_TOF_S` (checked at entry).
/// - `state.csm_state.position != [0, 0, 0]` (spacecraft not at Moon center).
///
/// # Postconditions
///
/// - `state.pending_maneuver` is `Some(m)` where:
///   - `m.tig == tig`
///   - `m.delta_v` is a finite, non-zero `Vec3` (the TEI delta-V in MCI, m/s)
///   - `m.burn_attitude` is an orthonormal `Mat3x3`
///   - `m.mode == TargetingMode::ReturnToEarth`
/// - `state.dsky.r[0]` contains the TIG for V06N45 display (set by
///   `p37_display_summary`).
///
/// # AGC source
///
/// `Comanche055/P37,P70.agc` — TEICOMP routine: builds target vector,
/// calls LAMBERT, forms delta-V from departure velocity minus current velocity.
pub fn p37_compute_tei(state: &mut AgcState, tig: Met, tof: f64)
```

### 5.3 `p37_display_summary`

```rust
/// Populate the DSKY display with the TEI burn summary.
///
/// Displays the targeting solution stored in `state.pending_maneuver` using
/// V06N45 (same format as the P30 burn summary display). The display shows:
///   R1: TIG in minutes past current MET (truncated to integer minutes)
///   R2: delta-V magnitude in m/s (displayed as XXXXX.X)
///   R3: estimated burn duration in seconds (from `guidance::targeting::burn_duration`)
///
/// # Arguments
///
/// * `state` — Mutable reference to the AGC global state. Must have
///   `state.pending_maneuver = Some(m)` before calling this function.
///
/// # Behavior
///
/// 1. If `state.pending_maneuver` is `None`, do nothing and return.
/// 2. Let `m = state.pending_maneuver.unwrap()`.
/// 3. Compute:
///    ```
///    tig_offset_min = (m.tig.0.saturating_sub(state.time.0)) / 6000
///    dv_magnitude   = linalg::norm(m.delta_v.0)           // m/s
///    burn_dur_s     = guidance::targeting::burn_duration(
///                         dv_magnitude,
///                         NOMINAL_CSM_MASS_KG             // see §4, 20_000.0 kg
///                     )
///    ```
/// 4. Write to DSKY fields:
///    ```
///    state.dsky.verb = 6;
///    state.dsky.noun = 45;
///    state.dsky.r[0] = tig_offset_min as f32;    // R1: minutes to TIG
///    state.dsky.r[1] = dv_magnitude as f32;       // R2: delta-V magnitude m/s
///    state.dsky.r[2] = burn_dur_s as f32;         // R3: burn duration seconds
///    ```
///
/// # Display noun rationale
///
/// Noun 45 (N45) was used in Comanche055 P37 for the TEI burn summary display.
/// In the Rust port this matches the pattern established by P30 (which uses N33
/// and N81). The specific DSKY field layout above is consistent with the
/// targeting-spec §9.3 pattern and the architecture's `DskyState` struct fields.
///
/// # AGC source
///
/// `Comanche055/P30,P37.agc` — V06N45 display sequence after TEICOMP.
pub fn p37_display_summary(state: &mut AgcState)
```

An additional constant needed by `p37_display_summary`:

```rust
/// Nominal CSM vehicle mass in lunar orbit (kg), used for burn duration estimate.
/// Represents a fully-loaded CSM (approximately 20,000 kg) as a default when
/// crew-entered mass is not yet available.
/// Source: Apollo CSM Systems Handbook; typical lunar-orbit mass after LOI.
pub const NOMINAL_CSM_MASS_KG: f64 = 20_000.0;
```

---

## 6. Calling Sequence and Data Flow

```
V37 N37 ENTER
    |
    v
PROGRAM_TABLE[37] = p37::init
    |
    v
p37_init(state)
    |
    +-- assert frame == MoonInertial
    +-- state.major_mode = 37
    +-- state.dsky.prog = 37
    +-- tig = state.time.0 + DEFAULT_TEI_TIG_OFFSET_CS
    |
    v
p37_compute_tei(state, Met(tig), DEFAULT_TEI_TOF_S)
    |
    +-- validate tof range
    +-- validate frame
    +-- build entry_target_mci = [-D_EARTH_MOON_M + R_EARTH + 121920, 0, 0]
    +-- kepler_step: propagate csm_state → state_at_tig (MU_MOON)
    |
    v
guidance::targeting::return_to_earth(state_at_tig, entry_target_mci, tof, refsmmat)
    |
    v  (calls lambert_targeting → math::lambert::lambert)
    |
    Maneuver { tig, delta_v (MCI, m/s), burn_attitude, ReturnToEarth }
    |
    +-- state.pending_maneuver = Some(maneuver)
    |
    v
p37_display_summary(state)
    |
    +-- state.dsky.verb = 6, .noun = 45
    +-- state.dsky.r[0] = minutes to TIG
    +-- state.dsky.r[1] = |delta_v| m/s
    +-- state.dsky.r[2] = burn_duration_s
    |
    v
Return JobPriority = 16
    |
    [Crew reviews display, then enters V37 N40 ENTER to execute via P40]
    |
    v
p40::init(state) reads state.pending_maneuver → burn_init(maneuver) → SPS ignition
```

---

## 7. Error Conditions and Alarm Policy

The initial implementation uses `assert!` / `panic!` for all guard conditions,
consistent with the pattern in `guidance::targeting` (see targeting-spec §11)
and the architecture's "navigation errors kill people" principle.

| Condition | Guard | Handling (initial) | Future |
|-----------|-------|--------------------|--------|
| `state.csm_state.frame != Frame::MoonInertial` | `assert_eq!` in `p37_init` and `p37_compute_tei` | `panic!` → hardware restart | Raise alarm 520 ("wrong mission phase") |
| `tof < MIN_TEI_TOF_S` | `assert!` in `p37_compute_tei` | `panic!` | Alarm 521 ("TEI TOF too short") |
| `tof > MAX_TEI_TOF_S` | `assert!` in `p37_compute_tei` | `panic!` | Alarm 521 ("TEI TOF too long") |
| `tig.0 < state.csm_state.epoch.0` | `assert!` in `p37_compute_tei` | `panic!` | Alarm 522 ("TEI TIG in the past") |
| Lambert convergence failure | Propagated `todo!` from `guidance::targeting` | Propagates as unimplemented | `TargetingError::LambertNoConverge` |
| `state.pending_maneuver` is `None` in display | Guard: `if None { return }` | Silent no-op | No change needed |

The `assert!` / `panic!` approach is intentional for the embedded target: a
navigation error must stop the software and restart into a known safe state
rather than silently compute a wrong burn. On hardware, a Rust panic triggers
the `HardFault` handler which calls `AgcHardware::hardware_restart`.

---

## 8. AGC Scale Factor Reference

P37 accepts and produces `f64` SI values throughout. The table below maps
Comanche055 erasable memory to the corresponding Rust field for fixture tests.

| Quantity | AGC symbol | Address | Scale | f64 SI conversion |
|----------|-----------|---------|-------|-------------------|
| TIG | `TIG` | octal 0350 (E3) | B+28 cs | `w_hi * 2^14 + w_lo` centiseconds |
| Delta-V x | `DELVEET1` | octal 0352 (E3) | B+7 m/s | `w_hi * 2^-7 + w_lo * 2^-21` m/s |
| Delta-V y | `DELVEET2` | octal 0354 (E3) | B+7 m/s | `w_hi * 2^-7 + w_lo * 2^-21` m/s |
| Delta-V z | `DELVEET3` | octal 0356 (E3) | B+7 m/s | `w_hi * 2^-7 + w_lo * 2^-21` m/s |
| CSM position | `RN` | octal 0306 (E3) | B+28 m | `w_hi * 2^14 + w_lo` metres |
| CSM velocity | `VN` | octal 0314 (E3) | B+7 m/s | `w_hi * 2^-7 + w_lo * 2^-21` m/s |
| State epoch | `TEPHEM` | octal 0340 (E3) | B+28 cs | `w_hi * 2^14 + w_lo` centiseconds |

---

## 9. Module Structure and Dependencies

```
agc-core::programs::p37
    |
    +-- uses: agc-core::guidance::targeting::{return_to_earth, burn_duration,
    |             Maneuver, TargetingMode, ENTRY_INTERFACE_ALT_M}
    +-- uses: agc-core::navigation::state_vector::{StateVector, Frame}
    +-- uses: agc-core::navigation::gravity::{MU_MOON, R_EARTH}
    +-- uses: agc-core::math::kepler::kepler_step
    +-- uses: agc-core::math::linalg::norm
    +-- uses: agc-core::executive::job::JobPriority
    +-- uses: agc-core::types::Met
    +-- reads/writes: AgcState::{csm_state, major_mode, dsky, refsmmat,
                          time, pending_maneuver}
    |
    +-- called by: agc-core::programs::PROGRAM_TABLE[37]  (p37::init)
    +-- feeds into: agc-core::programs::p40_p41  (reads state.pending_maneuver)
```

There are no circular dependencies. P37 calls into `guidance::targeting` and
`math::kepler`; neither of those modules calls back into `programs::p37`.

---

## 10. Test Cases

All test cases use tolerance `eps_dv = 1e-3` m/s for delta-V components and
`eps_att = 1e-9` for attitude matrix orthonormality, unless stated otherwise.

### TC-P37-1: `p37_init` with valid MoonInertial state sets major_mode = 37

**Purpose**: Verify that calling `p37_init` with a properly framed CSM state
sets the major mode, DSKY PROG display, and returns a non-zero job priority.

```rust
#[test]
fn tc_p37_1_init_sets_major_mode() {
    let mut state = AgcState::new();

    // Place CSM in a representative 100 km LLO on the +x axis in MCI.
    let r_llo = navigation::gravity::R_MOON + 100_000.0; // 1_837_400 m
    let v_circ = libm::sqrt(navigation::gravity::MU_MOON / r_llo); // ≈ 1633 m/s
    state.csm_state = StateVector {
        position: [r_llo, 0.0, 0.0],
        velocity: [0.0, v_circ, 0.0],
        epoch: Met(0),
        frame: Frame::MoonInertial,
    };
    state.time = Met(0);

    let priority = p37_init(&mut state);

    assert_eq!(state.major_mode, 37, "TC-P37-1: major_mode must be 37");
    assert_eq!(state.dsky.prog, 37, "TC-P37-1: dsky.prog must be 37");
    assert!(priority > 0, "TC-P37-1: returned priority must be non-zero");
}
```

**Expected outcome**: `state.major_mode == 37`, `state.dsky.prog == 37`,
`priority == 16` (or whatever constant value is specified in the implementation).

---

### TC-P37-2: `p37_compute_tei` produces a Maneuver with finite delta-V and valid burn attitude

**Purpose**: Verify that `p37_compute_tei` produces a physically plausible TEI
maneuver for a CSM in LLO. Uses a shorter-than-default TOF (30 hours) to keep
the Lambert solver in a well-conditioned regime and avoid the long-TOF convergence
issue noted in TC-TGT-10 of targeting-spec.

```rust
#[test]
fn tc_p37_2_compute_tei_finite_maneuver() {
    let mut state = AgcState::new();

    let r_llo = navigation::gravity::R_MOON + 100_000.0;
    let v_circ = libm::sqrt(navigation::gravity::MU_MOON / r_llo);
    state.csm_state = StateVector {
        position: [r_llo, 0.0, 0.0],
        velocity: [0.0, v_circ, 0.0],
        epoch: Met(0),
        frame: Frame::MoonInertial,
    };
    state.time = Met(0);
    // Use identity REFSMMAT (valid for burn_attitude computation)
    state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    let tig = Met(DEFAULT_TEI_TIG_OFFSET_CS);
    // 30-hour TOF: shorter than default, well within the valid range, avoids
    // long-TOF Lambert convergence sensitivity (see targeting-spec TC-TGT-10 note)
    let tof_s = 108_000.0_f64; // 30 hours

    p37_compute_tei(&mut state, tig, tof_s);

    let maneuver = state.pending_maneuver.expect("TC-P37-2: pending_maneuver must be Some");

    // TIG must match input
    assert_eq!(maneuver.tig, tig, "TC-P37-2: maneuver.tig must equal input tig");

    // delta_v must be finite and non-zero
    let dv_mag = math::linalg::norm(maneuver.delta_v.0);
    assert!(dv_mag.is_finite(), "TC-P37-2: delta_v magnitude must be finite");
    assert!(dv_mag > 1.0, "TC-P37-2: delta_v magnitude must be > 1 m/s");
    assert!(dv_mag < 5000.0, "TC-P37-2: delta_v magnitude must be < 5000 m/s (sanity)");

    // mode must be ReturnToEarth
    assert_eq!(maneuver.mode, TargetingMode::ReturnToEarth,
               "TC-P37-2: mode must be ReturnToEarth");

    // burn_attitude must be orthonormal: M * M^T = I (within 1e-9)
    let mt = math::linalg::transpose(maneuver.burn_attitude);
    let mmt = math::linalg::mxm(maneuver.burn_attitude, mt);
    for row in 0..3 {
        for col in 0..3 {
            let expected = if row == col { 1.0 } else { 0.0 };
            assert!((mmt[row][col] - expected).abs() < 1e-9,
                "TC-P37-2: burn_attitude not orthonormal at [{row}][{col}]: {} != {}",
                mmt[row][col], expected);
        }
    }

    // First column of burn_attitude must be parallel to unit(delta_v)
    let dv_unit = math::linalg::unit(maneuver.delta_v.0);
    let x_body = [maneuver.burn_attitude[0][0],
                  maneuver.burn_attitude[1][0],
                  maneuver.burn_attitude[2][0]];
    for i in 0..3 {
        assert!((x_body[i] - dv_unit[i]).abs() < 1e-9,
            "TC-P37-2: burn_attitude[{i}] col-0 != dv_unit[{i}]");
    }
}
```

**Expected outcome**: `pending_maneuver` is `Some`, `1 < |delta_v| < 5000` m/s,
`mode == ReturnToEarth`, `burn_attitude` is orthonormal with first column parallel
to `unit(delta_v)`.

---

### TC-P37-3: `p37_init` with EarthInertial state panics (wrong mission phase)

**Purpose**: Verify that P37 cannot be entered when the CSM state is not in
`Frame::MoonInertial`. This is a safety guard — calling P37 in Earth orbit
would compute a meaningless TEI burn.

```rust
#[test]
#[should_panic]
fn tc_p37_3_wrong_frame_panics() {
    let mut state = AgcState::new();

    // Place CSM in LEO (EarthInertial frame) — this is the wrong phase for P37
    let r_leo = navigation::gravity::R_EARTH + 400_000.0;
    let v_circ = libm::sqrt(navigation::gravity::MU_EARTH / r_leo);
    state.csm_state = StateVector {
        position: [r_leo, 0.0, 0.0],
        velocity: [0.0, v_circ, 0.0],
        epoch: Met(0),
        frame: Frame::EarthInertial, // wrong frame — P37 must reject this
    };
    state.time = Met(0);

    // This must panic because the frame is not MoonInertial
    let _ = p37_init(&mut state);
}
```

**Expected outcome**: `#[should_panic]` — the function panics due to the frame
assertion failure. In a future iteration, this becomes a program alarm instead.

---

### TC-P37-4: `p37_compute_tei` stores result in `state.pending_maneuver`

**Purpose**: Verify the storage contract: after `p37_compute_tei` returns,
`state.pending_maneuver` is `Some` with the expected targeting mode and a TIG
that matches the input argument. Confirms the data path from computation to
pending burn storage.

```rust
#[test]
fn tc_p37_4_result_stored_in_pending_maneuver() {
    let mut state = AgcState::new();

    let r_llo = navigation::gravity::R_MOON + 100_000.0;
    let v_circ = libm::sqrt(navigation::gravity::MU_MOON / r_llo);
    state.csm_state = StateVector {
        position: [r_llo, 0.0, 0.0],
        velocity: [0.0, v_circ, 0.0],
        epoch: Met(0),
        frame: Frame::MoonInertial,
    };
    state.time = Met(0);
    state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    // Verify that pending_maneuver starts empty
    assert!(state.pending_maneuver.is_none(),
            "TC-P37-4: pending_maneuver must be None before computation");

    let tig = Met(360_000_u32); // 1 hour from epoch
    let tof_s = 108_000.0_f64; // 30 hours

    p37_compute_tei(&mut state, tig, tof_s);

    // pending_maneuver must now be Some
    assert!(state.pending_maneuver.is_some(),
            "TC-P37-4: pending_maneuver must be Some after p37_compute_tei");

    let m = state.pending_maneuver.unwrap();

    // TIG must match exactly
    assert_eq!(m.tig, tig,
               "TC-P37-4: maneuver.tig must equal input tig");

    // Mode must be ReturnToEarth (not ExternalDeltaV or Lambert)
    assert_eq!(m.mode, TargetingMode::ReturnToEarth,
               "TC-P37-4: mode must be ReturnToEarth");

    // A second call with different TIG must overwrite the first result
    let tig2 = Met(720_000_u32); // 2 hours from epoch
    p37_compute_tei(&mut state, tig2, tof_s);

    let m2 = state.pending_maneuver.unwrap();
    assert_eq!(m2.tig, tig2,
               "TC-P37-4: second call must overwrite pending_maneuver.tig");
    assert_ne!(m.tig, m2.tig,
               "TC-P37-4: second call must produce a different TIG");
}
```

**Expected outcome**: `pending_maneuver` is `None` before the call, `Some` after,
with `tig` and `mode` matching expectations. A second call with a different TIG
overwrites the first result.

---

### TC-P37-5: TOF validation — out-of-range values panic

**Purpose**: Verify that both the lower bound and upper bound guards on TOF are
enforced. A TEI with TOF < 24 hours or > 120 hours is a data-entry error that
must not produce a maneuver.

```rust
#[test]
#[should_panic]
fn tc_p37_5a_tof_too_short_panics() {
    let mut state = AgcState::new();
    let r_llo = navigation::gravity::R_MOON + 100_000.0;
    let v_circ = libm::sqrt(navigation::gravity::MU_MOON / r_llo);
    state.csm_state = StateVector {
        position: [r_llo, 0.0, 0.0],
        velocity: [0.0, v_circ, 0.0],
        epoch: Met(0),
        frame: Frame::MoonInertial,
    };
    state.time = Met(0);
    // 6 hours — below MIN_TEI_TOF_S (24 hours)
    p37_compute_tei(&mut state, Met(DEFAULT_TEI_TIG_OFFSET_CS), 21_600.0);
}

#[test]
#[should_panic]
fn tc_p37_5b_tof_too_long_panics() {
    let mut state = AgcState::new();
    let r_llo = navigation::gravity::R_MOON + 100_000.0;
    let v_circ = libm::sqrt(navigation::gravity::MU_MOON / r_llo);
    state.csm_state = StateVector {
        position: [r_llo, 0.0, 0.0],
        velocity: [0.0, v_circ, 0.0],
        epoch: Met(0),
        frame: Frame::MoonInertial,
    };
    state.time = Met(0);
    // 200 hours — above MAX_TEI_TOF_S (120 hours)
    p37_compute_tei(&mut state, Met(DEFAULT_TEI_TIG_OFFSET_CS), 720_000.0);
}
```

**Expected outcome**: Both test cases panic due to the TOF range assertion.

---

## 11. Interface Contracts with Adjacent Modules

### 11.1 Dependency on `guidance::targeting::return_to_earth`

P37 calls `return_to_earth` with the entry target expressed in MCI. The function
signature (from targeting-spec §5.3) is:

```rust
pub fn return_to_earth(
    current: StateVector,
    entry_target: Vec3,
    tof_estimate: f64,
    refsmmat: Mat3x3,
) -> Maneuver
```

P37's responsibility is to supply `entry_target` correctly in MCI and to supply
a `tof_estimate` in the valid range. The `return_to_earth` function is responsible
for selecting `MU_MOON` (because `current.frame == MoonInertial`) and setting
`mode = ReturnToEarth`.

**Invariant**: The magnitude of `entry_target` must be within 1% of
`R_EARTH + ENTRY_INTERFACE_ALT_M` when measured relative to Earth's center in
MCI (i.e., `|entry_target - earth_mci| ≈ R_EARTH + ENTRY_INTERFACE_ALT_M`).
The function `return_to_earth` asserts this; P37 must satisfy it.

With the static approximation in §2.4:

```
earth_mci = [-384_400_000, 0, 0]
entry_target_mci = [-384_400_000 + 6_500_057, 0, 0] = [-377_899_943, 0, 0]
```

Check: `|entry_target_mci - earth_mci| = 6_500_057 m = R_EARTH + ENTRY_INTERFACE_ALT_M`
= 6,378,137 + 121,920 = 6,500,057 m. The assertion passes exactly.

### 11.2 Dependency on `math::kepler::kepler_step`

P37 propagates the CSM state from `state.csm_state.epoch` to `tig` before calling
the Lambert solver. The Kepler step function signature (from kepler-spec) is:

```rust
pub fn kepler_step(state: StateVector, dt_s: f64, mu: f64) -> StateVector
```

`dt_s` is the propagation interval in seconds: `(tig.0 - csm_state.epoch.0) as f64 / 100.0`.
For `tig == csm_state.epoch`, propagation is skipped (zero interval is a degenerate
Kepler step). `mu = MU_MOON` because the frame is `MoonInertial`.

### 11.3 Consumed by `programs::p40_p41`

After P37 completes, P40 reads `state.pending_maneuver` and calls
`guidance::maneuver::burn_init`. P37 must ensure that `pending_maneuver` is
`Some(m)` with:
- `m.delta_v.0` finite and non-zero (P40 asserts this in `burn_init`)
- `m.tig` in the future relative to `state.time` at the time P40 is entered
- `m.mode == TargetingMode::ReturnToEarth` (P40 uses this for burn monitor mode
  selection, though the initial P40 implementation may treat all modes identically)

---

## 12. Known Limitations and Future Work

| Limitation | Description | Future enhancement |
|------------|-------------|-------------------|
| Static Earth position | Earth center approximated as `[-D_EARTH_MOON_M, 0, 0]` in MCI | Replace with `navigation::planetary` ephemeris lookup at TIG epoch |
| No entry corridor iteration | TOF is used directly; no iteration on flight-path angle | Add iteration loop in `p37_compute_tei` targeting flight-path angle ≈ −6.5° |
| Sub-Earth aim point only | Entry target is always the sub-Earth point; no crew landing-site input | Add latitude/longitude noun entries (V06N45 R1/R2 for lat/lon) and convert to MCI direction |
| Long-TOF Lambert sensitivity | The Lambert solver may have convergence difficulty at the default 60-hour TOF; see TC-TGT-10 note in targeting-spec | Use 30-hour TOF in tests; production accuracy requires a better initial guess or solver robustness improvement in `math::lambert` |
| No `pending_maneuver` in AgcState | The `AgcState` struct does not yet have a `pending_maneuver: Option<Maneuver>` field | The implementer must add this field to `AgcState` in `agc-core/src/lib.rs` |
| Crew input not wired | TIG, TOF, and landing site are not yet accepted via DSKY keypad | Wire to V06N45, V21N33, V21N37 entries once DSKY I/O is implemented |

---

## 13. Spec Quality Checklist

- [x] AGC source file and routine referenced (§2.1): `P30,P37.agc`, `P37,P70.agc`
- [x] All erasable variables and AGC addresses listed (§2.2): TIG, DELVEET1/2/3, TEPHEM
- [x] Scale factors documented for all fixed-point values (§8)
- [x] Corresponding `f64` SI units documented (§3.2, §8)
- [x] Input/output preconditions and postconditions stated (§5.1, §5.2, §5.3)
- [x] Edge cases and error handling specified (§7)
- [x] At least 3 test cases with expected values — 6 provided, covering all 4 required (§10)
- [x] Rust API signatures designed with types and ownership (§5.1, §5.2, §5.3)
- [x] Invariants explicitly stated (§5.2 postconditions, §11.1)
- [x] Consistency with `docs/architecture.md` checked: `f64` SI throughout, no heap,
      `AgcState` passed by `&mut`, module boundaries respected (programs do not call
      into other programs; guidance is a pure computation layer)
- [x] Known Lambert long-TOF sensitivity noted; test cases use 30-hour TOF (§10 TC-P37-2)
- [x] `pending_maneuver` field addition to `AgcState` called out explicitly (§3.1, §12)
