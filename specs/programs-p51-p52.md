# Functional Specification: P51/P52 — IMU Alignment Programs

## AGC Source Reference

```
AGC source:  Comanche055/P51-P53.agc
Routines:    PROG52 (P52, page 737), R51 (fine align, page 756),
             R52 (AOP, page 743), R55 (gyro torque, page 759),
             CAL53A (coarse align, page 762), PICAPAR (star select, page 752),
             S52.2/S52.3 (gimbal angle compute), CHKSDATA/R54 (star validation)
Supporting:  Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc
             CALCGTA (gyro torque angles, page 1355), ARCTRIG (page 1357),
             CALCGA (CDU driving angles, page 1359), AXISGEN (page 1361)
             Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc
             IMUTEST, CALCGA, IMUCOARS, IMUFINE sequences (page 423)
Lines:       P51-P53.agc pages 737–784
```

---

## Behavior Summary

**P51** is the initial coarse-alignment program used at power-on when no prior
REFSMMAT exists. It selects the REFSMMAT orientation, computes gimbal angles
via S52.2, drives the IMU to coarse align (CAL53A / IMUCOARS), then performs
fine alignment (R51) from two star sightings. P51 is invoked at mission start.

**P52** is the in-flight realignment program. It is invoked whenever gyro drift
has accumulated and the crew wishes to correct the IMU orientation. P52
allows the crew to choose the alignment orientation (preferred / nominal /
REFSMMAT correction), then uses the sextant (optics) to sight two stars,
computes gyro torque angles, and fires the IMU gyros via IMUPULSE. The key
difference from P51: P52 may skip the coarse-align step if the orientation
correction is small (gyro coarse mode via GYCRS label).

**P53** (backup IMU alignment) is out of scope for Milestone 4. Note as
deferred — see P51-P53.agc page 737 header; P53 references not present in
Comanche055 (P53 is Luminary-specific).

Both programs share the underlying star-selection (PICAPAR), star-sighting
(R52/R53), star validation (CHKSDATA/R54), fine alignment (R51), and gyro
torque (R55) subroutines. The Rust implementation models these as helpers
called from the state machine.

---

## State Machine

### P51 — Initial Coarse + Fine Alignment

```
PromptRefsmmat → (crew selects orientation via DSKY N34)
WaitCoarseAlign → (CAL53A/IMUCOARS drives gimbals to THETAD)
WaitStarA → (crew sights star via optics, R52 marks)
WaitStarB → (crew sights second star, R52 marks)
Torque → (AXISGEN, CALCGTA, R55/IMUPULSE sends gyro pulses)
Done / Failed
```

### P52 — In-flight Fine Realignment

```
PromptRefsmmat → (crew selects option: preferred / nominal / REFSMMAT)
WaitStarA → (PICAPAR selects star pair, R52 positions optics, crew marks)
WaitStarB → (crew marks second star)
Torque → (AXISGEN, CALCGTA, R55/IMUPULSE)
Done / Failed
```

---

## Rust API

**Module path:** `agc_core::programs::p51_imu_align`

### Types

```rust
/// Alignment orientation option (AGC: OPTION2 register, P52B).
/// AGC source: P51-P53.agc PROG52 lines: OPTION2 bits select the path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlignOption {
    /// Preferred orientation: XSMD/YSMD/ZSMD set by prior burn program.
    /// AGC: OPTION2 bit 2 set → P52J path (page 739).
    Preferred,
    /// Nominal orientation: computed from current R and V vectors.
    /// AGC: OPTION2 bit 2 clear → P52T path → S52.3 (page 739).
    Nominal,
    /// REFSMMAT correction: corrects drift since last alignment.
    /// AGC: OPTION2 bits 1,0 → P52C path → GYCRS (page 740, gyro coarse).
    RefSmmat,
}

/// Monotonic phase of the IMU alignment state machine.
/// AGC source: Derived from PHASCHNG calls in PROG52, R51, R52 (P51-P53.agc).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum AlignPhase {
    /// Displaying and waiting for crew orientation selection.
    /// AGC label: P52B (GOPERF4R flash on DSKY).
    PromptRefsmmat,
    /// Coarse alignment in progress (P51 only).
    /// AGC label: CAL53A → IMUCOARS → IMUSTALL.
    WaitCoarseAlign,
    /// Waiting for first star sighting mark.
    /// AGC label: R51.2 / R51DSP (V01N70 flash); STARIND = 1.
    WaitStarA,
    /// Waiting for second star sighting mark.
    /// AGC label: R51 inner loop; STARIND = 0.
    WaitStarB,
    /// Computing gyro torque angles and issuing gyro pulses.
    /// AGC label: R55 → CALCGTA → PULSEM → IMUPULSE → IMUSTALL.
    Torque,
    /// Alignment complete. REFSMFLG set; REFSMMAT updated.
    Done,
    /// Alignment failed (gimbal lock, star not available, star data bad).
    /// AGC alarm codes: 405 (no stars), 401 (gimbal lock), 215 (no preferred).
    Failed,
}

/// State for the active P51 or P52 alignment program.
#[derive(Clone, Copy, Debug)]
pub struct P51State {
    /// Current phase.
    pub phase: AlignPhase,
    /// Which program variant is running (true = P51 initial, false = P52 inflight).
    pub is_p51: bool,
    /// Selected alignment orientation option.
    pub option: AlignOption,
    /// Line-of-sight unit vector for first star (in SM frame).
    /// Set when crew completes first mark. AGC erasable: STARSAV1.
    /// Scale: unit vector (f64, no additional scaling).
    pub star_a: Option<[f64; 3]>,
    /// Line-of-sight unit vector for second star (in SM frame).
    /// Set when crew completes second mark. AGC erasable: STARSAV2.
    pub star_b: Option<[f64; 3]>,
    /// Computed gyro torque angles (Y, Z, X gyros) in fractions of revolution.
    /// AGC erasable: IGC (Y gyro), MGC (Z gyro), OGC (X gyro).
    /// Scale: 1.0 = full revolution. Rust: f64.
    pub torque_angles: Option<[f64; 3]>,
    /// Alarm code if `phase == Failed`; 0 if no alarm.
    pub alarm_code: u16,
}
```

### Functions

```rust
/// Enter P51 (initial coarse + fine alignment).
///
/// Sets phase to `PromptRefsmmat`, sets `is_p51 = true`.
/// Calls R02BOTH (IMU status check) via `hw.imu().read_status()`.
/// Clears UPDATFLG and TRACKFLG in `state.flagwrds`.
///
/// Pre-conditions: IMU must be powered (IMODES30 status OK).
/// On IMU failure (alarm 01426/01427 from S61.1 check): set `phase = Failed`.
///
/// AGC source: P51-P53.agc PROG52 entry (page 738-739);
///             TC DOWNFLAG ADRES UPDATFLG; TC DOWNFLAG ADRES TRACKFLG;
///             TC BANKCALL CADR R02BOTH.
pub fn enter_p51(state: &mut AgcState, hw: &mut dyn AgcHardware) -> P51State;

/// Enter P52 (in-flight realignment).
///
/// Sets phase to `PromptRefsmmat`, sets `is_p51 = false`.
/// Identical preamble to P51 (R02BOTH check, flag clears).
/// Skips coarse-align step (no CAL53A); goes directly to star sighting.
///
/// AGC source: P51-P53.agc PROG52 entry — same label used for both P51 and P52;
///             the coarse-align branch is conditioned on OPTION2 and PFRATFLG.
pub fn enter_p52(state: &mut AgcState, hw: &mut dyn AgcHardware) -> P51State;

/// Advance the alignment state machine by one executive cycle.
///
/// Phase transition rules:
///   PromptRefsmmat → WaitCoarseAlign (P51) or WaitStarA (P52):
///     Triggered by `DskyIo::poll_key() == Key::Enter` with valid OPTION2.
///     AGC label: P52B proceed response (GOPERF4R at page 739).
///   WaitCoarseAlign → WaitStarA:
///     Triggered when IMUCOARS completes (IMU stable in coarse mode).
///     AGC label: CAL53A → COARFINE → REFSMFLG set (page 762).
///   WaitStarA → WaitStarB:
///     Triggered by crew mark (R52 mark accepted). Sets `star_a`.
///     AGC label: R51 STARIND=1 → R52 mark → SXTSM stored in STARSAV1.
///   WaitStarB → Torque:
///     Triggered by second crew mark. Sets `star_b`. Calls CHKSDATA (R54)
///     for star-pair validation. If validation fails: → Failed with alarm 405.
///     AGC label: R51 STARIND=0 → CHKSDATA → AXISGEN (page 757-758).
///   Torque → Done:
///     Gyro pulses sent via `hw.imu().torque_gyro()`. REFSMMAT updated.
///     Sets REFSMFLG in `state.flagwrds`. Clears PFRATFLG.
///     AGC label: R55 → CALCGTA → PULSEM → IMUPULSE → IMUSTALL → REFSMFLG set.
///   Any phase → Failed:
///     On gimbal lock (alarm 401), no stars (alarm 405), or IMU reversed (alarm 427).
///
/// Invariants:
///   - `phase` is monotonically non-decreasing.
///   - On entry to `Done`, the IMU controller must be in `FineAligned` state.
///   - `Failed` is a terminal state — tick is a no-op when `phase == Failed`.
///   - Never leaves the IMU in a partially aligned state: either Done (fine) or Failed.
///
/// AGC source: P51-P53.agc full flow from PROG52 through R51 / R55.
pub fn tick(align_state: &mut P51State, state: &mut AgcState, hw: &mut dyn AgcHardware);
```

---

## Key Subroutines (not separately implemented — called from tick)

| Subroutine | AGC file / label | Purpose in Rust |
|---|---|---|
| `picapar()` | P51-P53.agc PICAPAR (page 752) | Select best star pair from catalog given current vehicle orientation and occultation masks |
| `cal53a()` | P51-P53.agc CAL53A (page 762) | Compute desired gimbal angles (S52.2), drive IMU to coarse alignment via `hw.imu()` |
| `calcgta()` | INFLIGHT_ALIGNMENT_ROUTINES.agc CALCGTA (page 1355) | Compute gyro torque angles (IGC, MGC, OGC) from XDC/YDC/ZDC |
| `axisgen()` | INFLIGHT_ALIGNMENT_ROUTINES.agc AXISGEN (page 1361) | Build coordinate transform from two observed star vectors |
| `chksdata()` | P51-P53.agc CHKSDATA (page 760) | Validate star sighting pair angular separation |
| `r55_torque()` | P51-P53.agc R55 (page 759) | Display gyro torque angles (N93), call `hw.imu().torque_gyro()` |

---

## Scale Factors

| Quantity | AGC scale | Rust f64 unit |
|---|---|---|
| Star vector (STARSAV1/2) | Unit vector ÷ 2 (half-unit) | Normalized unit vector [f64; 3] |
| Gimbal angles THETAD | 1's complement, 2^15 counts = full rev | radians: multiply by 2π/32768 |
| Gyro torque (IGC, MGC, OGC) | Fraction of revolution | radians: multiply by 2π |
| REFSMMAT | Half-unit matrix (×0.5) | f64 unit matrix (×2 to denormalize) |
| TSIGHT (alignment time) | B-28 cs | centiseconds as i64 |

---

## Invariants

1. `phase` is monotonically non-decreasing. Backward transitions prohibited.
2. On completion (`Done`), `hw.imu()` must be `FineAligned`. The alignment
   programs do not own the `ImuImpl<State>` typestate directly — they call
   `control::imu_control::ImuController` for state transitions.
3. `Failed` is a terminal state. No further hardware commands are issued.
4. The IMU must never be left in `CoarseAligned` state at program exit.
   If P51 coarse-align completes but fine-align fails, the program must either
   complete fine-align or call `ImuImpl::into_unaligned()`.
5. REFSMMAT must only be overwritten after `Done`. During alignment
   (WaitStarA, WaitStarB, Torque), REFSMMAT holds the pre-alignment matrix.
6. No heap: star catalog lookup uses fixed-size arrays. PICAPAR equivalent
   iterates over an inline star catalog `[StarEntry; 37]` (37 stars in Comanche055).

---

## ImuController Integration

The alignment programs use `control::imu_control::ImuController` (separate spec)
for the coarse→fine typestate transition. The expected call sequence is:

```
P51 WaitCoarseAlign entry:
    imu_controller.begin_coarse_align(hw)
    → ImuImpl<Unaligned>.into_coarse_aligned()
    → hw.imu().set_coarse_align() (writes CHAN12 bit 4)
    → hw.imu().write_cdu_commands(thetad) iteratively

P51 Torque → Done transition:
    imu_controller.begin_fine_align(hw)
    → ImuImpl<CoarseAligned>.into_fine_aligned()
    → hw.imu().write_control(FINE_ALIGN_BITS) (clears CHAN12 bits 4, 5)
    → for each axis: hw.imu().torque_gyro(axis, pulses)
```

---

## DSKY / agc-sim Impact

- `PromptRefsmmat`: Flash N34 (alignment option display) via `DskyIo`.
  `DskyEvent::FlashN34(option_code)`.
- `WaitCoarseAlign`: Display N22 (gimbal angles THETAD) via `DskyIo`.
- `WaitStarA`/`WaitStarB`: Flash N70 (star code) via `DskyIo`.
  `DskyEvent::FlashN70(star_code)`.
- `Torque`: Display N93 (gyro torque angles IGC/MGC/OGC) via `DskyIo`.
- `Done`: Emit `SimLog::info("IMU fine alignment complete")`.
- `Failed`: Illuminate PROG ALARM light; emit alarm code to `SimLog`.
- New `DskyDisplayState` field: `imu_aligned: bool` (set true on Done).
- No new keyboard bindings needed.

---

## Test Cases

| # | Name | Setup | Expected |
|---|---|---|---|
| T1 | P51 entry IMU power-on check | `enter_p51()` with IMU status = OK | `phase == PromptRefsmmat`; flags cleared |
| T2 | P51 IMU status fail | `enter_p51()` with CHAN30 bit 13 (IMU fail) set | `phase == Failed`; alarm 01426 raised |
| T3 | Coarse-align completes | Simulate IMUCOARS completion; call `tick()` | `phase == WaitStarA` |
| T4 | First star sighting accepted | Inject star_a LOS vector; call `tick()` | `phase == WaitStarB`; `star_a == Some(los)` |
| T5 | Star pair validation failure | Inject two stars with separation angle < 40° | `phase == Failed`; alarm 405 raised |
| T6 | P52 full fine-align | `enter_p52()`; inject two valid star sightings; call `tick()` through Torque | `phase == Done`; REFSMMAT updated; IMU in FineAligned state |
| T7 | Gimbal lock during coarse align | Simulate |middle gimbal angle| > 60° during CAL53A | Alarm 401 raised; `phase == Failed` |
| T8 | P52 RefSmmat option (gyro coarse) | Select `AlignOption::RefSmmat`; run to Done | GYCRS path taken; DRIFTFLG cleared; REFSMFLG set |
