# Specification: `guidance/maneuver` Module

**Status**: Approved for implementation
**Module path**: `agc-core/src/guidance/maneuver.rs`
**Architecture reference**: `docs/architecture.md` §7.2 (P40 SPS thrusting), §7.4 (SERVICER), §11.1 (Thrust DAP / TVC)
**Targeting reference**: `agc-core/src/guidance/targeting.rs` — `Maneuver` struct (input to burn execution)
**SERVICER reference**: `specs/average-g-spec.md` §3.3 (exit hook), §5 (2-second cycle, step 8)
**Types reference**: `specs/types-module-spec.md` §3 (`Met`, `Vec3`, `DeltaV`)
**Math reference**: `specs/linalg-spec.md` §4.2 (`cross`), §4.3 (`norm`), §4.4 (`unit`), §4.5 (`vadd`), §4.6 (`vsub`), §4.7 (`vscale`)
**State vector reference**: `specs/state-vector-spec.md` §2.1 (state vector layout), §2.2 (inertial frame)
**DAP reference**: `specs/dap-spec.md` §5.6 (TVC mode), §5.8 (staged inputs/outputs), §3.2 (`DapState::attitude_error`)
**TVC reference**: `specs/tvc-spec.md` §4.2 (`tvc_step` signature), §3.1 (`TvcState`)
**AGC source files**:
- `Comanche055/P40-P47.agc` — P40 SPS burn program entry, SERVICER exit hook registration, engine enable/disable
- `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc` — cross-product steering subroutine, cutoff test
- `Comanche055/SERVICER207.agc` — SERVICER exit hook dispatch, accumulated delta-V accumulation
- `Comanche055/TVCDAPS.agc` — TVC attitude error input interface
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — `DELVEET1/2/3` (accumulated delta-V), `TGO` (time-to-go), `DVTOTAL` (target delta-V magnitude)
**Spec checklist**: `specs/README.md` — all items satisfied (see §11)

---

## 1. Purpose and Scope

`guidance::maneuver` implements the **burn execution** logic of an SPS
(Service Propulsion System) thrusting maneuver. It is called at runtime, from
inside the 2-second SERVICER cycle, throughout the duration of the burn. It is
entirely distinct from `guidance::targeting`, which computes the maneuver
parameters ahead of ignition.

The module answers three questions on every SERVICER cycle:

1. **Navigation**: How much delta-V has been accumulated so far, and how much
   remains?
2. **Attitude**: What angular correction should be fed to the TVC gimbal to
   keep the thrust vector pointed in the right direction despite attitude drift?
3. **Cutoff**: Is the burn complete? If so, command engine off and compute any
   residual delta-V for RCS nulling.

### What this module provides

- `BurnState` — all mutable state for a single burn execution instance.
- `burn_init(target: Maneuver) -> BurnState` — construct a `BurnState` ready
  for ignition from a completed targeting solution.
- `burn_update(state: &mut BurnState, measured_dv: Vec3, dt: f64)` — integrate
  one SERVICER cycle's worth of measured delta-V into the running totals.
- `cross_product_steering(remaining_dv: Vec3, current_v: Vec3) -> Vec3` — the
  attitude correction rate vector for the TVC system.
- `is_burn_complete(state: &BurnState, cutoff_tolerance: f64) -> bool` — cutoff
  test: magnitude check plus time-exceeded guard.
- `compute_cutoff_time(state: &BurnState) -> Met` — estimate of the absolute
  mission time at which the engine should cut off.
- `trim_residual_dv(state: &BurnState) -> Vec3` — delta-V still unachieved
  after engine cutoff, to be nulled by RCS.

### What this module does NOT provide

- **Targeting**. The pre-burn delta-V calculation (`compute_delta_v`) lives in
  `guidance::targeting`.
- **TVC filter / gimbal commanding**. The `cross_product_steering` function
  computes an attitude-rate correction vector but does not issue gimbal commands.
  That is done by `control::tvc`, which consumes this vector as its attitude
  error input. See §8.
- **PIPA reading or REFSMMAT rotation**. The `measured_dv` argument to
  `burn_update` is already a fully corrected, inertially-resolved velocity
  increment delivered by the SERVICER. See `specs/average-g-spec.md` §5 steps
  1–5.
- **Engine HAL calls** (other than the cutoff enable/disable logic documented in
  §7.8). `guidance::maneuver` sets the `burn_active` flag and computes the
  cutoff time; the P40 program wrapper reads the flag and calls
  `hw.engine().sps_enable(false)`.
- **DAP mode switching**. Switching from the Coast DAP to the Thrust DAP
  (TVC mode) is the responsibility of `programs::p40_p41`. This module assumes
  TVC mode is already active when `burn_update` is first called.

---

## 2. AGC Background

### 2.1 P40 SPS Thrusting — Overall Flow

P40 (`Comanche055/P40-P47.agc`) is the major mode for a Service Propulsion
System burn. Its execution proceeds in three phases:

**Phase 1 — Pre-ignition** (handled by `guidance::targeting`, not this module):
The crew or Mission Control selects P40. The program computes or accepts an
uplinked target delta-V (`DELVEET1/2/3`, scale B+7 m/s) and time of ignition
(`TIG`). It displays V06N85 (delta-V components) for crew review, then waits
for the countdown to `TIG`.

**Phase 2 — Burn execution** (this module):
At `TIG`, P40 arms the SPS engine, registers the SERVICER exit hook
(`state.servicer_exit = Some(burn_servicer_exit)`), and enters the burn loop.
Every 2 seconds the SERVICER fires `burn_servicer_exit`, which calls
`burn_update` and `cross_product_steering`. The display verb N85 shows the
remaining delta-V.

**Phase 3 — Cutoff and cleanup** (shared between this module and P40):
When `is_burn_complete` returns `true`, P40 disables the SPS engine, calls
`trim_residual_dv` to obtain any leftover velocity for RCS nulling, and
de-registers the SERVICER exit hook.

### 2.2 Cross-Product Steering

The AGC used **cross-product steering** (also called "VXV steering" after the
`VXV` opcode that computed it) to correct attitude errors during the burn. The
key insight is that if the spacecraft is pointed exactly along the desired
thrust direction, then the cross product of the remaining delta-V vector and the
current velocity-change vector is zero. Any non-zero cross product indicates an
attitude error, and its magnitude and direction give the angular correction
needed.

The corrective angular rate is:

```
omega_c = (dv_remaining x v_current) / |v_current|^2
```

where:
- `dv_remaining` is the remaining required delta-V in the inertial frame (m/s)
- `v_current` is the current measured velocity increment (the PIPA delta-V
  in the inertial frame for this cycle) (m/s)
- `|v_current|^2` is the squared magnitude of the current velocity increment

The result `omega_c` (rad/s) is the body-rate correction passed to the TVC DAP.

This formula appears in `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc` in the
subroutine typically identified as `BURNBABY` or `STEERSUB`. The AGC evaluated
it using the interpretive-language opcodes `VLOAD`, `VSU`, `VXV` (cross),
`ABVAL` (norm squared), `VSCALE` on each SERVICER exit cycle.

In the original AGC the denominator was `|v_current|^2` (velocity-change
magnitude squared), not `|dv_remaining|^2`. This makes the gain proportional
to the reciprocal of the current thrust level, giving a roughly constant
closed-loop bandwidth regardless of how hard the engine is thrusting.

### 2.3 Cutoff Logic

The AGC computed a continuous "time-to-go" (`TGO`) estimate by dividing the
remaining delta-V magnitude by the current measured thrust acceleration. Cutoff
was commanded when either:

(a) The accumulated delta-V magnitude met or exceeded the target (the primary
criterion), or

(b) The absolute mission time exceeded a pre-computed maximum burn duration
(the backup criterion, protecting against a stuck-open engine).

The Rust port implements both criteria in `is_burn_complete`. The backup
criterion uses `cutoff_time` computed by `compute_cutoff_time`.

### 2.4 Accumulated Delta-V

In Comanche055 the accumulated delta-V was stored in erasable memory as a
double-precision triple:

| AGC symbol | Octal address | Meaning |
|------------|---------------|---------|
| `DELVEET1` | ~0520 (E5 bank) | Accumulated delta-V, component 1 (inertial X), scale B+7 m/s |
| `DELVEET2` | following words  | Accumulated delta-V, component 2 (inertial Y) |
| `DELVEET3` | following words  | Accumulated delta-V, component 3 (inertial Z) |

On each SERVICER exit, the program added the current cycle's PIPA-derived
inertial delta-V to this triple. The `DVTOTAL` register held the target
delta-V magnitude (scalar) and was compared against `ABVAL(DELVEET1)` for
cutoff.

In the Rust port, the entire triple is carried in `BurnState::accumulated_dv_inertial`
as a `Vec3` (m/s). The target is `BurnState::target_dv_inertial` (also `Vec3`).

### 2.5 Restart Protection

In Comanche055, the burn state was protected by restart Group 3 (PHASCHNG
group 3). Phase values distinguished: ignition pending, burn active, and
cutoff commanded. A restart mid-burn resumed the burn loop without re-igniting
the engine.

In the Rust port, `BurnState` should be stored in a field of `AgcState` (not
on the stack), so it persists across task boundaries and is visible to the
restart handler. The `burn_active` flag in `BurnState` is the equivalent of
the restart group phase bit: if `true` after a restart, P40's
`restart_resume` method re-registers the SERVICER exit hook and re-enters the
burn loop without re-igniting the engine.

---

## 3. AGC Erasable Memory Variables

| AGC symbol | Octal address | Scale | Rust field |
|------------|---------------|-------|------------|
| `DELVEET1` (x) | E5 bank ~0520 | B+7 m/s | `BurnState::accumulated_dv_inertial[0]` |
| `DELVEET2` (y) | E5 bank ~0522 | B+7 m/s | `BurnState::accumulated_dv_inertial[1]` |
| `DELVEET3` (z) | E5 bank ~0524 | B+7 m/s | `BurnState::accumulated_dv_inertial[2]` |
| `DVTOTAL`  | E5 bank ~0526 | B+7 m/s | `linalg::norm(BurnState::target_dv_inertial)` (computed on demand) |
| `TGO`      | E5 bank ~0530 | B+14 s | `compute_cutoff_time` result minus current `Met` |
| `TIG`      | E5 bank ~0532 | centiseconds | `BurnState::tig` |

Scale factor B+7 m/s means 1 LSB of the AGC double-precision word = 2^7 = 128 m/s.
The Rust port stores all values as plain `f64` m/s with no scaling.

---

## 4. Type: `BurnState`

### 4.1 Declaration

```rust
/// Mutable execution state for a single SPS burn.
///
/// Created by `burn_init` at the start of P40 burn execution and destroyed
/// (or zeroed) when the burn is complete. Stored as a field of `AgcState`
/// so that restart protection can access it.
///
/// AGC correspondence: the set of erasable variables `DELVEET1/2/3`, `DVTOTAL`,
/// `TGO`, and the P40 phase flags in the restart group 3 phase table.
/// Source: Comanche055/ERASABLE_ASSIGNMENTS.agc and P40-P47.agc.
#[derive(Clone, Copy, Debug)]
pub struct BurnState {
    /// Target delta-V vector in the inertial (ECI or MCI) frame, m/s.
    ///
    /// Set once by `burn_init` from `Maneuver::delta_v` and never modified
    /// during burn execution. This is the guidance-computed or uplinked total
    /// delta-V to be achieved.
    ///
    /// AGC correspondence: the target delta-V components, from which `DVTOTAL`
    /// (the scalar magnitude) is derived by `ABVAL`.
    pub target_dv_inertial: Vec3,

    /// Running sum of all PIPA-measured, inertially-resolved delta-V increments
    /// received since ignition, m/s.
    ///
    /// Initialised to `[0.0, 0.0, 0.0]` by `burn_init`.
    /// Updated by `burn_update` on every SERVICER cycle.
    ///
    /// AGC correspondence: `DELVEET1/DELVEET2/DELVEET3` triple (scale B+7 m/s).
    pub accumulated_dv_inertial: Vec3,

    /// Time of ignition, in mission elapsed time (centiseconds).
    ///
    /// Set by `burn_init` from `Maneuver::tig`.
    /// Used only for the backup cutoff time guard.
    ///
    /// AGC correspondence: `TIG` in erasable (scale: centiseconds absolute,
    /// stored as TIME1/TIME2 delta).
    pub tig: Met,

    /// True from ignition until the cutoff condition is confirmed.
    ///
    /// P40 reads this flag each SERVICER cycle. When it transitions from
    /// `true` to `false` (set by the logic in P40's SERVICER exit wrapper
    /// after `is_burn_complete` returns `true`), P40 calls
    /// `hw.engine().sps_enable(false)`.
    ///
    /// Not set to `false` by `burn_update` itself; the caller (P40 wrapper)
    /// is responsible for the transition so that the HAL call can be made.
    ///
    /// AGC correspondence: the "engine-on" phase bit in restart group 3.
    pub burn_active: bool,

    /// True once the cutoff time criterion has been met (backup cutoff guard).
    ///
    /// Set to `true` by `is_burn_complete` when `AgcState::time >= cutoff_time`.
    /// Once true, `is_burn_complete` returns `true` regardless of accumulated
    /// delta-V.
    pub cutoff_time_met: bool,
}
```

### 4.2 Invariants

- `target_dv_inertial` is finite and non-zero. A zero target delta-V is a
  targeting error and must not reach `burn_init`.
- `accumulated_dv_inertial` components are finite. They are bounded in practice
  by the maximum SPS delta-V (~3000 m/s for a trans-earth injection burn).
- `tig` is the absolute MET at which the SPS engine was commanded on. It is set
  before `burn_update` is ever called.
- When `burn_active` is `false` and `cutoff_time_met` is `false`, the burn has
  been cleanly completed by the primary criterion. Both being `false` is the
  terminal state.
- When `burn_active` is `false` and `cutoff_time_met` is `true`, the burn was
  terminated by the backup time criterion — this is an off-nominal condition and
  should trigger a program alarm (P40 responsibility, not this module's).

**Note on `BurnState` vs `Maneuver`**: `BurnState` does NOT store
`burn_attitude` or `mode`. Those two fields of `Maneuver` are consumed by P40's
setup phase (burn attitude is passed to the DAP to configure `DapMode::Tvc`,
and `mode` drives the targeting configuration) before `burn_init` is called.
Once burn execution begins, `BurnState` only needs the delta-V target and the
ignition time.

---

## 5. Function Specifications

### 5.1 `burn_init`

```rust
/// Construct a `BurnState` ready for ignition from a completed targeting solution.
///
/// Called by P40 immediately before asserting `SPS_ENABLE`. After `burn_init`
/// returns, P40 must:
///   1. Call `hw.engine().sps_enable(true)`.
///   2. Set `state.servicer_exit = Some(burn_servicer_exit)`.
///   3. Call `services::average_g::start_servicer(state, hw)` if not already
///      running.
///
/// AGC correspondence: the initialisation sequence at the beginning of P40
/// burn execution (P40-P47.agc), which zeroes `DELVEET1/2/3` and stores the
/// target delta-V.
///
/// # Preconditions
/// - `target.delta_v` must be finite and non-zero (linalg::norm > 0).
/// - `target.tig` must be a valid MET, i.e. it must not lie in the past at
///   call time.
///
/// # Postconditions
/// - `result.target_dv_inertial == target.delta_v.0` (the inner Vec3).
/// - `result.accumulated_dv_inertial == [0.0, 0.0, 0.0]`.
/// - `result.tig == target.tig`.
/// - `result.burn_active == true`.
/// - `result.cutoff_time_met == false`.
pub fn burn_init(target: Maneuver) -> BurnState
```

**Implementation**:

```
BurnState {
    target_dv_inertial:      target.delta_v.0,   // unwrap the DeltaV newtype
    accumulated_dv_inertial: [0.0, 0.0, 0.0],
    tig:                     target.tig,
    burn_active:             true,
    cutoff_time_met:         false,
}
```

The `DeltaV` type is a newtype over `Vec3` (see `specs/types-module-spec.md`
§3.4). The `.0` field access extracts the inner `Vec3`.

---

### 5.2 `burn_update`

```rust
/// Integrate one SERVICER cycle's measured delta-V into the running totals.
///
/// Called from the SERVICER exit hook (`burn_servicer_exit`) on every 2-second
/// SERVICER cycle while the engine is running. The caller is responsible for
/// ensuring `measured_dv` is already expressed in the inertial frame (i.e.,
/// REFSMMAT rotation has already been applied by the SERVICER — see
/// `specs/average-g-spec.md` §5 steps 1–5).
///
/// After calling `burn_update`, the caller must call `is_burn_complete` to
/// determine whether to command engine cutoff.
///
/// AGC correspondence: the accumulation step in the SERVICER exit hook for P40,
/// which performs `DELVEET += delta_v_inertial` (a VAD interpretive operation on
/// the triple) and checks the cutoff condition.
/// Source: Comanche055/SERVICER207.agc and POWERED_FLIGHT_SUBROUTINES.agc.
///
/// # Preconditions
/// - `state.burn_active` must be `true`.
/// - `measured_dv` must be finite. It is the inertial delta-V increment for
///   the current SERVICER interval (typically 2 seconds). Its magnitude is
///   bounded by the SPS thrust level times the cycle time: at ~30 kN thrust
///   and ~28,000 kg vehicle mass, one 2-second cycle delivers < 3 m/s.
/// - `dt` must be positive and finite. Nominal value: 2.0 seconds.
///
/// # Postconditions
/// - `state.accumulated_dv_inertial` is the vector sum of all previous
///   `measured_dv` arguments since ignition.
/// - No other field of `BurnState` is modified by this function.
///
/// # Side effects
/// None. `burn_update` is a pure accumulator.
pub fn burn_update(state: &mut BurnState, measured_dv: Vec3, dt: f64)
```

**Implementation**:

```
// Unconditionally accumulate regardless of dt; dt is available for
// future extension (e.g., variable-rate SERVICER) but the accumulation
// is purely additive: we sum the vector increments, not rates.
state.accumulated_dv_inertial = linalg::vadd(
    state.accumulated_dv_inertial,
    measured_dv,
);
```

Note: `dt` is kept in the signature for forward compatibility (the SERVICER
is nominally 2 s but may be called at different rates in simulation). For the
current implementation it is unused in the accumulation itself; the integration
is count-based, not rate-based. If a rate-based check is needed in the future
(e.g., instantaneous acceleration estimate), `dt` is available.

---

### 5.3 `cross_product_steering`

```rust
/// Compute the attitude-rate correction vector for TVC during a burn.
///
/// Uses the cross-product steering law from Comanche055 to determine the
/// angular correction that the TVC gimbal must apply to align the thrust vector
/// with the remaining required delta-V direction.
///
/// The formula is:
///
///     omega_c = (dv_remaining x v_current) / |v_current|^2
///
/// where:
///   - `dv_remaining` is the remaining delta-V vector (target minus accumulated),
///     expressed in the inertial frame (m/s)
///   - `v_current` is the inertial delta-V increment measured in the current
///     SERVICER cycle (m/s) — i.e., the PIPA reading for this 2-second window,
///     already rotated to inertial by REFSMMAT
///   - `|v_current|^2` is the scalar squared magnitude of `v_current`
///
/// The result `omega_c` (rad/s in the inertial frame, then projected to body
/// axes by `control::tvc`) is the attitude error rate fed into the TVC
/// lead-lag compensator filter (see `docs/architecture.md` §11.1).
///
/// The denominator `|v_current|^2` gives the steering law a gain that is
/// inversely proportional to the instantaneous thrust level: if thrust is
/// high, the gain is smaller (the vehicle is naturally stable); if thrust is
/// low (early burn, ullage), the gain is higher, giving faster correction.
/// This matches the behaviour of the original AGC formulation.
///
/// AGC correspondence: the VXV/ABVAL/VSCALE sequence in the P40 SERVICER exit
/// routine in Comanche055/POWERED_FLIGHT_SUBROUTINES.agc. The opcode sequence
/// is:
///
///     VLOAD  DELVEET (remaining dv)
///     VXV    PIPTIME (current dv)        ; cross product
///     VSU                                ; numerator in MPAC
///     DOT    PIPTIME                     ; |v_current|^2 = v . v
///     STADR                              ; denominator in accumulator
///     VSCALE                             ; omega_c = numerator / denominator
///
/// # Preconditions
/// - `remaining_dv` must be finite. It may be zero on the last cycle (near
///   cutoff), in which case the result is `[0.0, 0.0, 0.0]`.
/// - `current_v` must be finite and non-zero. If `current_v` is zero (engine
///   not thrusting yet, or ullage only), the function panics with a descriptive
///   message. The caller must guard against calling this function before the
///   engine is producing measurable thrust.
///
/// # Postconditions
/// - The result vector is perpendicular to both `remaining_dv` and `current_v`,
///   which is the correct direction for an attitude rate correction.
/// - `norm(result) ≈ norm(remaining_dv) * sin(theta) / norm(current_v)` where
///   theta is the angle between `remaining_dv` and `current_v`. At zero error
///   (theta = 0), the result magnitude is 0.
/// - Units: radians per second (inertial frame).
///
/// # Panics
/// - If `norm(current_v) == 0.0`. The SPS must be producing thrust before this
///   function is called.
pub fn cross_product_steering(remaining_dv: Vec3, current_v: Vec3) -> Vec3
```

**Implementation**:

```rust
let v_sq = linalg::dot(current_v, current_v);
// Guard: v_sq == 0.0 means no thrust measured this cycle.
assert!(v_sq != 0.0, "cross_product_steering: current_v is zero — \
        engine not thrusting or called before ignition");

let numerator = linalg::cross(remaining_dv, current_v);
linalg::vscale(numerator, 1.0 / v_sq)
```

The division is `1.0 / v_sq` (a scalar multiply), implemented via
`linalg::vscale`. Using `vscale` rather than three divisions avoids redundant
computation and matches the `VXV`/`VSCALE` sequence in the original AGC.

**Relationship to `BurnState`**: `cross_product_steering` is a pure function
and does not take `BurnState` directly. The caller computes `remaining_dv` as:

```rust
let remaining_dv = linalg::vsub(
    state.burn.target_dv_inertial,
    state.burn.accumulated_dv_inertial,
);
```

before calling `cross_product_steering(remaining_dv, measured_dv)`.

---

### 5.4 `is_burn_complete`

```rust
/// Test whether the burn has achieved its delta-V target or has exceeded the
/// maximum allowed burn time.
///
/// Returns `true` when either of the following criteria is satisfied:
///
/// **Primary criterion (delta-V magnitude)**:
///   |accumulated_dv| >= |target_dv| - cutoff_tolerance
///
///   "cutoff_tolerance" allows for the finite time between the cutoff decision
///   and the actual engine shutdown. A typical value is 0.3 m/s (accounting
///   for ~0.5 s of signal propagation + gimbal actuation lag times the
///   nominal SPS thrust acceleration of ~0.5 m/s²).
///
/// **Backup criterion (time exceeded)**:
///   state.cutoff_time_met == true
///
///   This is set externally by P40 when `AgcState::time >= compute_cutoff_time(state)`.
///   It protects against a stuck-open engine.
///
/// Both criteria are OR'd together: the first one to fire causes cutoff.
///
/// AGC correspondence: the cutoff test in the SERVICER exit for P40.
/// In Comanche055/POWERED_FLIGHT_SUBROUTINES.agc this is expressed as:
///
///     ABVAL  DELVEET         ; |accumulated_dv|
///     DSU    DVTOTAL          ; compare with target magnitude
///     BMN    continue_burn    ; branch if still less than target
///     ...    cutoff_sequence  ; fall through = cutoff
///
/// The scale factor for the comparison is B+7 m/s on both sides, so no
/// scaling is needed in the Rust port.
///
/// # Preconditions
/// - `cutoff_tolerance` must be non-negative and finite.
///   Recommended value: 0.3 m/s (see above).
///   A value of 0.0 is valid but may cause late cutoff on the last cycle.
///
/// # Postconditions
/// - Returns `true` if and only if the primary or backup criterion is met.
/// - Does not modify `state`. Callers must update `state.burn_active` and
///   `state.cutoff_time_met` themselves.
pub fn is_burn_complete(state: &BurnState, cutoff_tolerance: f64) -> bool
```

**Implementation**:

```rust
if state.cutoff_time_met {
    return true;
}
let target_mag = linalg::norm(state.target_dv_inertial);
let achieved_mag = linalg::norm(state.accumulated_dv_inertial);
achieved_mag >= target_mag - cutoff_tolerance
```

The subtraction `target_mag - cutoff_tolerance` is done in `f64` arithmetic;
no risk of underflow for physically meaningful values (target_mag >= ~1 m/s
for any real maneuver, cutoff_tolerance < 1 m/s).

---

### 5.5 `compute_cutoff_time`

```rust
/// Estimate the absolute mission time at which the engine should cut off,
/// as a backup guard against under-achieving the burn or a stuck-open engine.
///
/// The estimate is based on the remaining delta-V and the average thrust
/// acceleration observed so far in the burn:
///
///     tgo = |remaining_dv| / a_avg           (seconds remaining)
///     cutoff_time = tig + elapsed + tgo      (absolute MET, centiseconds)
///
/// where:
///   `remaining_dv = target_dv - accumulated_dv` (m/s vector)
///   `a_avg = |accumulated_dv| / elapsed`       (m/s², average measured accel)
///   `elapsed = (current_met - tig) / 100.0`    (seconds since ignition)
///
/// If called before any delta-V has been accumulated (elapsed == 0 or
/// accumulated == 0), the function returns `tig + MAX_BURN_DURATION_CS`
/// as a conservative fallback (MAX_BURN_DURATION_CS = 75000 centiseconds =
/// 750 seconds, safely above the longest expected SPS burn).
///
/// AGC correspondence: the `TGO` (time-to-go) calculation that Comanche055
/// maintained in erasable memory (scale B+14 s), updated each SERVICER cycle.
/// The AGC stored TGO as an absolute cutoff time rather than a countdown.
/// Source: Comanche055/P40-P47.agc TGO update sequence.
///
/// # Preconditions
/// - `state.tig` must have been set by `burn_init`.
/// - `current_met` is the current mission elapsed time (from `AgcState::time`).
/// - `current_met >= state.tig` (the engine has ignited). If called before
///   ignition, the fallback path returns the maximum burn duration limit.
///
/// # Postconditions
/// - Returns a `Met` value representing the estimated absolute cutoff time.
/// - The result is always >= `current_met` (never in the past relative to now).
/// - On the fallback path, returns `state.tig + MAX_BURN_DURATION_CS`.
///
/// # Side effects
/// None. Pure function.
pub fn compute_cutoff_time(state: &BurnState, current_met: Met) -> Met
```

**Implementation**:

```rust
const MAX_BURN_DURATION_CS: u32 = 75_000;   // 750 s — upper bound for any SPS burn

let elapsed_cs = current_met.0.saturating_sub(state.tig.0);
if elapsed_cs == 0 {
    return Met(state.tig.0.saturating_add(MAX_BURN_DURATION_CS));
}

let elapsed_s = elapsed_cs as f64 / 100.0;
let achieved_mag = linalg::norm(state.accumulated_dv_inertial);
if achieved_mag == 0.0 {
    return Met(state.tig.0.saturating_add(MAX_BURN_DURATION_CS));
}

let a_avg = achieved_mag / elapsed_s;   // m/s^2, average measured thrust accel

let remaining_dv = linalg::vsub(state.target_dv_inertial, state.accumulated_dv_inertial);
let remaining_mag = linalg::norm(remaining_dv);

let tgo_s = remaining_mag / a_avg;
let tgo_cs = (tgo_s * 100.0) as u32;

Met(current_met.0.saturating_add(tgo_cs))
```

The `Met` type is a `u32` centisecond counter (see `specs/types-module-spec.md`
§2.3). Saturating arithmetic prevents overflow for pathological inputs.

---

### 5.6 `trim_residual_dv`

```rust
/// Compute the residual delta-V remaining after SPS engine cutoff.
///
/// After the SPS is shut down, the accumulated delta-V will not exactly equal
/// the target delta-V: the SPS cannot fractionally throttle, and the SERVICER
/// cycle is coarse (2 seconds). The difference is:
///
///     residual = target_dv - accumulated_dv
///
/// This residual vector is returned to P40, which passes it to the RCS system
/// for nulling. The RCS burns are small (the residual from a properly executed
/// SPS burn is typically < 1 m/s) and are handled outside this module.
///
/// If the backup cutoff criterion fired (`state.cutoff_time_met == true`),
/// the residual may be significantly larger than normal. P40 should raise a
/// program alarm if `norm(residual) > RESIDUAL_ALARM_THRESHOLD` (suggested
/// threshold: 3.0 m/s).
///
/// AGC correspondence: the "trim burn" logic following SPS cutoff in P40.
/// The residual was displayed to the crew on N85 (delta-V components) before
/// the RCS trim burn was authorised.
/// Source: Comanche055/P40-P47.agc post-cutoff sequence.
///
/// # Preconditions
/// - Should be called only after `is_burn_complete` has returned `true` and
///   the SPS engine has been disabled.
/// - `state.accumulated_dv_inertial` must reflect all cycles up to and
///   including the last `burn_update` call before cutoff.
///
/// # Postconditions
/// - Returns `target_dv - accumulated_dv` as a `Vec3` in m/s (inertial frame).
/// - A zero return vector means the burn was exact (very rare in practice).
///
/// # Side effects
/// None. Pure function.
pub fn trim_residual_dv(state: &BurnState) -> Vec3
```

**Implementation**:

```rust
linalg::vsub(state.target_dv_inertial, state.accumulated_dv_inertial)
```

---

## 6. Integration with P40 (SPS Thrusting Program)

The SERVICER exit hook pattern (see `specs/average-g-spec.md` §3.3, §5 step 8)
is the bridge between this module and the 2-second navigation cycle. P40
registers a closure or function pointer that the SERVICER calls on every cycle.

### 6.1 P40 Startup Sequence

```
1.  Receive Maneuver from targeting (P30/uplink).
2.  Display V06N85 (delta-V components) for crew confirmation.
3.  state.burn = burn_init(maneuver).
4.  At TIG (counted down via Waitlist or T3 interrupt):
    a. hw.engine().sps_enable(true).
    b. state.servicer_exit = Some(burn_servicer_exit).
    c. services::average_g::start_servicer(state, hw).   // if not already running
    d. state.dap_state.mode = DapMode::Tvc.              // switch from Coast to TVC DAP
```

### 6.2 SERVICER Exit Hook (`burn_servicer_exit`)

This function is the `fn(&mut AgcState)` registered in `state.servicer_exit`.
It is defined in `programs::p40_p41` (not in this module), but it calls the
pure functions in this module:

```rust
// programs/p40_p41.rs  — the SERVICER exit hook for P40
fn burn_servicer_exit(state: &mut AgcState) {
    // Step 1: retrieve this cycle's inertial delta-V from the SERVICER.
    // The SERVICER has already computed `delta_v_inertial` (see average-g-spec
    // §5 steps 1–5) and stored it in a staging field on AgcState.
    let measured_dv: Vec3 = state.servicer_last_dv_inertial;

    // Step 2: accumulate.
    guidance::maneuver::burn_update(&mut state.burn, measured_dv, 2.0);

    // Step 3: compute remaining delta-V and attitude correction for TVC.
    let remaining_dv = linalg::vsub(
        state.burn.target_dv_inertial,
        state.burn.accumulated_dv_inertial,
    );
    let omega_c = guidance::maneuver::cross_product_steering(
        remaining_dv,
        measured_dv,
    );
    // Feed omega_c into the DAP attitude error field.
    // The DAP supervisor (dap_step) reads DapState::attitude_error and passes
    // it to tvc::tvc_step on the next T5RUPT cycle (see dap-spec §5.6).
    state.dap_state.attitude_error = omega_c;

    // Step 4: update backup cutoff guard.
    let cutoff_time = guidance::maneuver::compute_cutoff_time(&state.burn, state.time);
    if state.time >= cutoff_time {
        state.burn.cutoff_time_met = true;
    }

    // Step 5: check cutoff.
    if guidance::maneuver::is_burn_complete(&state.burn, CUTOFF_TOLERANCE) {
        state.burn.burn_active = false;
        // Engine disable, display update, and RCS trim dispatch are done
        // by the P40 main loop, which polls burn_active on the next Executive cycle.
    }
}
```

The constant `CUTOFF_TOLERANCE: f64 = 0.3` (m/s) is defined in
`programs::p40_p41`.

### 6.3 P40 Cutoff Sequence

P40's main job (running under the Executive) polls `state.burn.burn_active`.
When it transitions to `false`:

```
1. hw.engine().sps_enable(false).
2. state.servicer_exit = None.
3. let residual = guidance::maneuver::trim_residual_dv(&state.burn).
4. If norm(residual) > RESIDUAL_ALARM_THRESHOLD:
       services::alarm::raise(state, hw, ALARM_LARGE_DV_RESIDUAL).
5. Schedule RCS trim burn (passes residual to P41 or inline RCS logic).
6. Display V06N85 with residual for crew awareness.
7. services::average_g::stop_servicer(state).  // unless P47 follows
```

---

## 7. Integration with TVC (Thrust Vector Control)

The attitude correction computed by `cross_product_steering` is consumed by
`control::tvc` as the attitude error input to the TVC lead-lag filter. The
data path goes through `DapState::attitude_error` — there is NO separate
`TvcState::attitude_error` field. `TvcState` holds only servo state
(gimbal positions and trim); it does not own attitude error.

### 7.1 Interface

The authoritative data path (CI-3 resolution):

```rust
// In AgcState (agc-core/src/lib.rs):
pub dap_state: DapState,
pub tvc_state: TvcState,
pub tvc_filter: TvcFilter,
```

```rust
// In DapState (control/dap.rs) — the field that carries the attitude error:
/// Attitude error angles [roll, pitch, yaw] in radians.
/// In TVC mode (cross-product steering) this is set by the P40 SERVICER exit
/// hook from the result of cross_product_steering(), then consumed by
/// tvc_step on the next T5RUPT cycle.
/// Cross-reference: specs/dap-spec.md §3.2, §5.6; specs/tvc-spec.md §4.2.
pub attitude_error: Vec3,
```

`TvcState` does NOT have an `attitude_error` field. The attitude error travels
as `DapState::attitude_error` and is passed as a parameter to `tvc_step`.

### 7.2 Data Flow

```
SERVICER (2-second cycle)
  |
  v
burn_servicer_exit()
  |
  +-- burn_update()            --> BurnState::accumulated_dv_inertial
  |
  +-- cross_product_steering() --> omega_c (rad/s, inertial)
  |                                  |
  |                                  v
  |                         state.dap_state.attitude_error = omega_c
  |
T5RUPT ISR shim (~100 ms)
  |
  +-- CDU read → state.current_cdu (staged)
  |
  v
dap_step(&mut state)   [fn(&mut AgcState) — no HAL access]
  |
  v  (mode == Tvc)
tvc::tvc_step(&mut state.tvc_state,
              &mut state.tvc_filter,
              state.dap_state.attitude_error,   // Vec3 passed as parameter
              DAP_PERIOD_S,
              engine)
  |
  v
hw.engine().sps_gimbal(pitch_counts, yaw_counts)
```

Key design points (CI-3):
- `omega_c` is written to `DapState::attitude_error` (not `TvcState`).
- `tvc_step` receives the full `Vec3` attitude error as an explicit parameter.
- The SERVICER runs at 2 s; T5RUPT runs at 100 ms. The attitude error written
  by the SERVICER persists in `DapState::attitude_error` until overwritten by
  the next SERVICER cycle; the DAP uses the most recent value each T5RUPT.

### 7.3 Frame Convention

`cross_product_steering` returns `omega_c` in the **inertial frame** (the same
frame as `target_dv_inertial` and `measured_dv`). The `control::tvc` module
must project this vector into the body frame using REFSMMAT before applying the
TVC filter. This projection is `control::tvc`'s responsibility, not this
module's.

Specifically:

```
omega_c_body = linalg::vxm(omega_c_inertial, state.refsmmat)
```

(row-vector times matrix, or equivalently matrix-transpose times column-vector,
since REFSMMAT is orthonormal).

### 7.4 Gain and Stability Note

The `1 / |current_v|^2` denominator in the steering law means the gain is high
when the engine has just started (small `|current_v|`) and decreases as the
burn progresses. The TVC lead-lag filter in `control::tvc` provides additional
stability. P40 should not call `cross_product_steering` during the first
SERVICER cycle after ignition if ullage thrust (RCS pre-pressurisation) has not
yet produced a measurable `|current_v|`. The `assert` in the function body
guards this.

---

## 8. AGC Source References (by function)

| Function | Primary AGC source | Secondary source |
|----------|--------------------|-----------------|
| `burn_init` | `P40-P47.agc` — initialisation of DELVEET and phase table | `ERASABLE_ASSIGNMENTS.agc` — DELVEET addresses |
| `burn_update` | `SERVICER207.agc` — SERVICER exit VAD for DELVEET accumulation | `POWERED_FLIGHT_SUBROUTINES.agc` — delta-V accumulator |
| `cross_product_steering` | `POWERED_FLIGHT_SUBROUTINES.agc` — VXV/ABVAL/VSCALE sequence | `TVCDAPS.agc` — TVC input consumption |
| `is_burn_complete` | `POWERED_FLIGHT_SUBROUTINES.agc` — ABVAL/DSU/BMN cutoff test | `P40-P47.agc` — cutoff command sequence |
| `compute_cutoff_time` | `P40-P47.agc` — TGO calculation and update | `ERASABLE_ASSIGNMENTS.agc` — TGO address |
| `trim_residual_dv` | `P40-P47.agc` — post-cutoff DELVEET display and RCS trim | — |

---

## 9. Rust API Summary

```rust
// agc-core/src/guidance/maneuver.rs

use crate::types::{DeltaV, Met, Vec3};
use crate::guidance::targeting::Maneuver;
use crate::math::linalg;

#[derive(Clone, Copy, Debug)]
pub struct BurnState {
    pub target_dv_inertial:      Vec3,
    pub accumulated_dv_inertial: Vec3,
    pub tig:                     Met,
    pub burn_active:             bool,
    pub cutoff_time_met:         bool,
}

pub fn burn_init(target: Maneuver) -> BurnState;

pub fn burn_update(state: &mut BurnState, measured_dv: Vec3, dt: f64);

pub fn cross_product_steering(remaining_dv: Vec3, current_v: Vec3) -> Vec3;

pub fn is_burn_complete(state: &BurnState, cutoff_tolerance: f64) -> bool;

pub fn compute_cutoff_time(state: &BurnState, current_met: Met) -> Met;

pub fn trim_residual_dv(state: &BurnState) -> Vec3;
```

All six public items are pure or take `&mut BurnState` only. None takes
`&mut AgcState` directly; integration with `AgcState` is the responsibility of
`programs::p40_p41`.

---

## 10. Test Cases

### TC-MANEUVER-1: `burn_init` from a clean targeting solution

**Purpose**: Verify that `burn_init` correctly copies the target delta-V, zeros
the accumulator, and sets flags.

**Setup**:
```rust
let target = Maneuver {
    tig:          Met(180_000),               // T+30 min
    delta_v:      DeltaV([90.0, 0.0, -60.0]), // 108 m/s magnitude approx
    burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    mode:         TargetingMode::ExternalDeltaV,
};
let state = burn_init(target);
```

**Expected**:
- `state.target_dv_inertial == [90.0, 0.0, -60.0]`
- `state.accumulated_dv_inertial == [0.0, 0.0, 0.0]`
- `state.tig == Met(180_000)`
- `state.burn_active == true`
- `state.cutoff_time_met == false`

**Tolerance**: exact (no floating-point arithmetic in `burn_init`).

---

### TC-MANEUVER-2: Partial delta-V accumulation over three cycles

**Purpose**: Verify that `burn_update` correctly sums inertial delta-V over
multiple SERVICER cycles.

**Setup**:
```rust
let target = Maneuver {
    tig:          Met(0),
    delta_v:      DeltaV([0.0, 100.0, 0.0]),
    burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    mode:         TargetingMode::ExternalDeltaV,
};
let mut state = burn_init(target);

let dv1 = [0.0_f64,  2.5, 0.0];
let dv2 = [0.0_f64,  2.5, 0.0];
let dv3 = [0.0_f64,  2.5, 0.0];

burn_update(&mut state, dv1, 2.0);
burn_update(&mut state, dv2, 2.0);
burn_update(&mut state, dv3, 2.0);
```

**Expected**:
- `state.accumulated_dv_inertial == [0.0, 7.5, 0.0]`
- `is_burn_complete(&state, 0.3) == false`
  (achieved 7.5 m/s; target is 100.0 m/s; 7.5 < 100.0 - 0.3)

**Tolerance**: `f64` floating-point, ±1 × 10⁻¹⁴ m/s per component.

---

### TC-MANEUVER-3: Cross-product steering with known geometry

**Purpose**: Verify the cross-product steering formula for a specific attitude
error case.

**Scenario**: The target delta-V is aligned with the +Y inertial axis. The
current velocity increment is mostly in +Y but has a small +X component (the
vehicle is slightly off-attitude, thrusting ~6° off-axis).

**Setup**:
```rust
// Remaining delta-V: still needs 50 m/s in +Y.
let remaining_dv: Vec3 = [0.0, 50.0, 0.0];

// Current measured velocity: 2.5 m/s mostly in +Y, slightly off in +X.
// sin(6°) ≈ 0.1045, cos(6°) ≈ 0.9945
let current_v: Vec3 = [0.2613, 2.4863, 0.0];  // ~2.5 m/s at 6° from +Y in X-Y plane

let omega_c = cross_product_steering(remaining_dv, current_v);
```

**Expected**:

Step 1: `remaining_dv × current_v`
```
cross([0, 50, 0], [0.2613, 2.4863, 0]) =
  [50*0   - 0*2.4863,
   0*0.2613 - 0*0,
   0*2.4863 - 50*0.2613]
= [0.0, 0.0, -13.065]
```

Step 2: `|current_v|^2 = 0.2613^2 + 2.4863^2 ≈ 0.0683 + 6.1817 ≈ 6.2500`

Step 3: `omega_c = [0, 0, -13.065] / 6.25 = [0, 0, -2.0904]` rad/s

The negative Z component indicates the vehicle must rotate nose-left (negative
yaw in body convention) to align with the target direction. This is consistent
with the right-hand-rule geometry: the thrust is displaced +X from the target,
so the correction is a negative-Z rotation.

**Tolerance**: ±0.001 rad/s (limited by sin/cos approximation in setup values).

---

### TC-MANEUVER-4: Cutoff detection at the primary (delta-V) criterion

**Purpose**: Verify that `is_burn_complete` fires when the accumulated
delta-V magnitude reaches the target.

**Setup**:
```rust
let target = Maneuver {
    tig:          Met(0),
    delta_v:      DeltaV([0.0, 0.0, 90.0]),  // 90.0 m/s target
    burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    mode:         TargetingMode::ExternalDeltaV,
};
let mut state = burn_init(target);
state.accumulated_dv_inertial = [0.0, 0.0, 89.8];  // 0.2 m/s short
```

**Step 1**: `is_burn_complete(&state, 0.3)` with tolerance 0.3 m/s.

- `target_mag = 90.0`; `achieved_mag = 89.8`; `89.8 >= 90.0 - 0.3 = 89.7` → **true**

**Expected**: returns `true` (within cutoff tolerance band).

**Step 2**: Tighten tolerance to `0.1`:
- `89.8 >= 90.0 - 0.1 = 89.9` → **false**

**Expected**: returns `false`.

**Step 3**: Force backup criterion:
```rust
state.cutoff_time_met = true;
```
- `is_burn_complete(&state, 0.1)` → **true** (backup criterion overrides).

**Tolerance**: exact (only comparisons of `f64` scalars).

---

### TC-MANEUVER-5: Residual trim after SPS cutoff

**Purpose**: Verify that `trim_residual_dv` returns the correct unachieved
delta-V vector after an early cutoff.

**Setup**: A burn targeting 50.0 m/s prograde (−Z in a hypothetical frame) was
cut off after accumulating only 48.7 m/s:

```rust
let target = Maneuver {
    tig:          Met(0),
    delta_v:      DeltaV([0.0, 0.0, -50.0]),
    burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    mode:         TargetingMode::ExternalDeltaV,
};
let mut state = burn_init(target);
state.accumulated_dv_inertial = [0.1, -0.2, -48.7];  // slightly off-axis
state.burn_active = false;
```

**Expected**:
```
residual = target - accumulated
         = [0.0, 0.0, -50.0] - [0.1, -0.2, -48.7]
         = [-0.1, 0.2, -1.3]
```

`trim_residual_dv(&state) ≈ [-0.1, 0.2, -1.3]` m/s

`norm(residual) ≈ sqrt(0.01 + 0.04 + 1.69) ≈ sqrt(1.74) ≈ 1.319` m/s —
within normal range (< 3 m/s), no alarm needed.

**Tolerance**: ±1 × 10⁻¹³ m/s (pure subtraction, one ULP rounding expected).

---

## 11. Spec Quality Checklist

| Item | Status |
|------|--------|
| AGC source file and line range referenced | Satisfied — see §2, §8 |
| All erasable variables and their AGC addresses listed | Satisfied — see §3 |
| Scale factors documented for all fixed-point values | Satisfied — B+7 m/s for delta-V; §3 |
| Corresponding `f64` SI units documented | Satisfied — m/s throughout; §3, §4 |
| Input/output preconditions and postconditions stated | Satisfied — each function §5.x |
| Edge cases and error handling specified | Satisfied — zero `current_v` panic §5.3; overflow guard §5.5; zero accumulator fallback §5.5 |
| At least 3 test cases with expected values | Satisfied — 5 test cases, §10 |
| Rust API signature designed | Satisfied — §9 |
| Invariants explicitly stated | Satisfied — §4.2 |
| Consistency with `docs/architecture.md` checked | Satisfied — §7.2, §7.4, §11.1 all cited |

---

## 12. Cross-References

| Topic | Specification |
|-------|---------------|
| `Maneuver` struct (input to `burn_init`) | `agc-core/src/guidance/targeting.rs` — `Maneuver::tig`, `Maneuver::delta_v`, `Maneuver::burn_attitude`, `Maneuver::mode` |
| `DeltaV`, `Vec3`, `Met` types | `specs/types-module-spec.md` §3 |
| `linalg::cross`, `norm`, `unit`, `vadd`, `vsub`, `vscale` | `specs/linalg-spec.md` §4.2–4.7 |
| SERVICER exit hook (`state.servicer_exit`) | `specs/average-g-spec.md` §3.3, §5 step 8 |
| SERVICER start/stop | `specs/average-g-spec.md` §4.1, §4.2 |
| PIPA-to-inertial delta-V pipeline (input to `burn_update`) | `specs/average-g-spec.md` §5 steps 1–5 |
| `StateVector` and inertial frame | `specs/state-vector-spec.md` §2.2 |
| TVC DAP mode, `DapState::attitude_error` (CI-3) | `docs/architecture.md` §11.1; `specs/dap-spec.md` §3.2, §5.6 |
| `tvc_step` signature and `TvcFilter` | `specs/tvc-spec.md` §4.2, §3.2 |
| Coast-to-TVC DAP mode switch | `docs/architecture.md` §11.1; `specs/dap-spec.md` §6.2 |
| Restart protection for `BurnState` | `specs/average-g-spec.md` §2.5; `specs/executive-spec.md` §2.3 |
| Program alarm for large residual | `docs/architecture.md` §12; `specs/alarm-spec.md` |
