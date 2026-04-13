# Spec: IMU Control — Coarse/Fine Alignment Typestate Controller

## AGC Source Reference

```
File:     Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc
Routines: IMUZERO (page 1421) — zero CDU counters and power-down
          IMUCOARS (page 1423) — coarse align entry
          COARS / COARS1 / COARS2 (pages 1423-1425) — CDU drive loop
          SETCOARS (page 1426) — put IMU in coarse align hardware mode
          IMUFINE (page 1427) — fine align mode switch
          IMUFINED / IFAILOK / PFAILOK (page 1428) — fine align completion
          MODEEXIT (page 1422) — general exit from mode switches
          COARSTOL DEC -.01111 (page 1425) — 2° tolerance in half-revolutions

File:     Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc
Routines: COAALIGN (page 428) — coarse align subroutine used by P02
          IMUTEST / IMUBACK (page 423) — IMU performance test entry
          EARTHR / ERTHRVSE (pages 430-431) — Earth-rate compensation during alignment

File:     Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc
Routines: CALCGTA (pages 1355-1356) — compute gyro torque angles from XDC/YDC/ZDC
          ARCTRIG (pages 1357-1358) — atan2 from sin/cos
          CALCGA (pages 1359-1360) — compute CDU drive angles from XNB/XSM frames
          AXISGEN (pages 1361-1362) — two-star coordinate system generation (SMNB method)
Pages:    1355-1364
```

## Behavior Summary

The IMU control module wraps the hardware IMU (`hal::imu::ImuImpl`) in a higher-level state machine that enforces the alignment lifecycle: power-on / unaligned → coarse alignment → fine alignment → operate. The typestate pattern prevents calling fine-alignment methods on an unaligned IMU at compile time.

### IMU alignment states (hardware perspective)

| State | CHAN12 bits | Meaning |
|---|---|---|
| Unaligned | Bit4=0, Bit5=0, Bit6=0 | IMU not in any align mode; gyros spinning, no CDU commanding |
| CoarseAligned | Bit4=1, Bit6=1 | CDU error counters enabled; gimbals driven to THETAD via pulse commands |
| FineAligned | Bit4=0, Bit5=0, Bit6=1 | Zero and coarse bits cleared; gyro torquing active for precision |

### Coarse alignment (IMUCOARS / COARS routine)

Coarse alignment rotates the gimbals to within 2° of the Reference Stable-Member Matrix (REFSMMAT). The hardware loop in `COARS` / `COARS2` (pages 1423-1425):

1. Enables CDU error counters: `CAF BIT6 / EXTEND / WOR CHAN12`.
2. For each CDU axis (inner/middle/outer = indices 2, 1, 0):
   - Computes `THETAD[axis] - CDUX[axis]` (desired minus actual) in ones-complement.
   - Clamps the command to ±COMMAX pulses per iteration.
   - Writes to `CDUXCMD / CDUYCMD / CDUZCMD` (channel 14 pulse command registers).
3. Waits 4 ms (`VARDELAY 2`), then repeats.
4. End-of-command check (`CHKCORS`): verifies each gimbal is within 2° of `THETAD` (`COARSTOL DEC -.01111 = -2° in half-rev units`). If not → alarm 0211 (`COARSERR` label) and fail path (`IMUBAD`).
5. On success → `ENDIMU` → `MODEEXIT`.

The tolerance `COARSTOL DEC -.01111` represents 2° in the AGC's half-revolution angle unit (1 half-revolution = 0.5 rev = 180°; `0.01111 × 180° = 2.0°`).

In Rust the actual CDU driving loop is a HAL responsibility (`ImuImpl<CoarseAligned>::write_cdu_commands`). The `ImuController<CoarseAligned>` holds the target REFSMMAT and the `ImuImpl`. The caller drives the CDU loop from outside (from the Executive job context) by calling `step_coarse_align()` repeatedly, or the developer may implement the loop within a dedicated Waitlist task.

### Fine alignment — SMNB two-vector method (AXISGEN / CALCGTA)

Fine alignment uses star sightings to determine the true orientation of the stable member relative to the navigation base and to compute gyro torquing angles that correct any residual error.

**AXISGEN** (Inflight Alignment Routines, pages 1361-1362) implements the two-star method (SMNB = Stable Member to Nav Base):

Inputs:
- `star_a_sm`: observed direction to star A in stable-member (SM) frame (from optics CDU readings). Stored at `STARAD`.
- `star_b_sm`: observed direction to star B in SM frame. Stored at `STARAD +6`.
- `star_a_desired`: catalog direction to star A in nav-base (NB) or reference frame. Stored at VAC area +6.
- `star_b_desired`: catalog direction to star B in NB frame. Stored at VAC area +12D.

Algorithm (AXISGEN, pages 1361-1362):
1. For each pair (A and B): compute `VA = UNIT(SA × SB)` and `WA = SA × VA` to form an orthonormal triad {SA, VA, WA} in the SM frame, and similarly {SB_desired, VB_desired, WB_desired} in the NB frame.
2. The rotation matrix from SM to NB is: each column of the NB-expressed basis expressed in SM coordinates = `sum(UA_k × UB_k)` for the three orthonormal components.
3. Store the result at `XDC / YDC / ZDC` (desired SM orientation as three half-unit vectors in SM frame).

**CALCGTA** (pages 1355-1356) converts `XDC / YDC / ZDC` (desired SM in current SM coordinates) into three gyro torquing angles (IGC = Y gyro, MGC = Z gyro, OGC = X gyro) via trigonometric decomposition:

```
ZPRIME = UNIT(-XD3, 0, XD1)          # Intermediate axis
IGC    = atan2(ZP1, ZP3)             # Y gyro angle
MGC    = atan2(XD2, COS_MGC)         # Z gyro angle
OGC    = atan2(ZP · YDC, ZP · ZDC)  # X gyro angle
```

Where each angle is computed by `ARCTRIG` (pages 1357-1358), which dispatches to ARCSIN or ARCCOS depending on which quadrant gives better numerical accuracy (|sin| > sin(45°) → use ARCCOS, else ARCSIN).

In Rust: the `fine_align_with_stars()` method computes the gyro-torque angles analytically (no interpreter loops) and returns an `ImuController<FineAligned>`. The caller applies the torque angles via `ImuIo::torque_gyro()`. The math maps exactly to the CALCGTA sequence.

### CALCGA — CDU drive angles for coarse alignment

`CALCGA` (pages 1359-1360) computes the three CDU desired angles (`THETAD`) from the navigation-base orientation (`XNB / YNB / ZNB`) and the desired stable-member orientation (`XSM / YSM / ZSM`), both expressed in the same reference frame (ECI / REFSMMAT). This is the function called by `COAALIGN` in IMU_CALIBRATION_AND_ALIGNMENT.agc (page 428).

In Rust: `coarse_align_to(refsmmat)` stores `refsmmat` in the controller and computes `THETAD` from it. The REFSMMAT columns are the `XSM / YSM / ZSM` vectors; the navigation-base orientation `XNB / YNB / ZNB` is fixed (identity in the body frame relative to the stable member). CALCGA's gimbal-lock check (`|MGC| > 60°` → alarm 0401, `GLOKFAIL` flag) is reproduced as a `GimbalLock` error variant.

### Power-down / STBY → Unaligned

The `IMUZERO` routine (page 1421) zeros all CDU counters and clears CHAN12 bits. This is the transition path when:
- `IMODES30` bit 9 is not set (IMU not in OPERATE mode).
- Hardware reports a cage command (CAGETSTJ check).
- STBY mode is entered.

In Rust: `power_down()` on any `ImuController<State>` returns `ImuController<Unaligned>`.

## Rust API

Module path: `agc_core::control::imu_control`

```rust
use crate::hal::imu::{CoarseAligned, FineAligned, ImuImpl, ImuIo, Unaligned};
use crate::types::{Mat3x3, Vec3};

/// Error returned when coarse alignment fails.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoarseAlignError {
    /// Gimbals did not converge to within 2° of THETAD.
    ///
    /// AGC: alarm 0211 at COARSERR label (IMU_MODE_SWITCHING_ROUTINES.agc page 1425).
    ToleranceExceeded,
    /// IMU is in gimbal lock (|middle gimbal angle| > 60°).
    ///
    /// AGC: alarm 0401 at GIMLOCK1 label (INFLIGHT_ALIGNMENT_ROUTINES.agc page 1360).
    GimbalLock,
}

/// Error returned when fine alignment fails.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FineAlignError {
    /// Two star vectors are nearly collinear; AXISGEN cannot form a valid triad.
    ///
    /// AGC: AXISGEN degeneracy (INFLIGHT_ALIGNMENT_ROUTINES.agc page 1361).
    CollinearStars,
    /// Gyro torquing angle computation overflowed or produced NaN.
    NumericalError,
}

/// IMU alignment controller with typestate tracking.
///
/// `State` is one of `Unaligned`, `CoarseAligned`, `FineAligned` from `hal::imu`.
/// The compiler enforces that fine-alignment methods cannot be called before
/// coarse alignment, and that the IMU cannot be used for navigation before
/// fine alignment.
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc alignment state machine.
pub struct ImuController<State> {
    /// The underlying bare-metal IMU implementation.
    imu: ImuImpl<State>,
    /// The target REFSMMAT for coarse alignment.
    ///
    /// Set by `coarse_align_to`; used to compute THETAD values.
    /// AGC: REFSMMAT erasable, 18 words (Comanche055/ERASABLE_ASSIGNMENTS.agc).
    refsmmat: Mat3x3,
}

impl ImuController<Unaligned> {
    /// Construct a new controller in the Unaligned state.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO entry.
    pub fn new(imu: ImuImpl<Unaligned>) -> Self;

    /// Begin coarse alignment to the given REFSMMAT.
    ///
    /// Transitions the IMU to CoarseAligned hardware mode (sets CHAN12 bit 4 and
    /// bit 6) and stores `refsmmat` as the target orientation.
    ///
    /// The actual CDU drive loop (COARS routine) is separate: the caller must
    /// call `step_coarse_align()` on the returned `ImuController<CoarseAligned>`
    /// until it returns `Ok(true)` (converged) or `Err(CoarseAlignError)`.
    ///
    /// Computes `THETAD` (desired CDU angles) from `refsmmat` via the CALCGA
    /// algorithm (INFLIGHT_ALIGNMENT_ROUTINES.agc pages 1359-1360).
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUCOARS (page 1423),
    ///             SETCOARS (page 1426).
    ///             Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc CALCGA (page 1359).
    pub fn coarse_align_to(self, refsmmat: Mat3x3) -> ImuController<CoarseAligned>;
}

impl ImuController<CoarseAligned> {
    /// Execute one iteration of the CDU drive loop (COARS / COARS2).
    ///
    /// Reads current CDU angles from the IMU, computes the error vs THETAD,
    /// clamps to COMMAX, and issues CDU pulse commands via `ImuIo::write_cdu_commands`.
    ///
    /// Returns:
    ///   `Ok(true)`  — all three axes are within 2° of THETAD (COARSTOL check passed).
    ///   `Ok(false)` — iteration issued pulses; call again in 4 ms.
    ///   `Err(CoarseAlignError::GimbalLock)` — middle gimbal > 60° (alarm 0401).
    ///   `Err(CoarseAlignError::ToleranceExceeded)` — end-of-command check failed (alarm 0211).
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc COARS (page 1423),
    ///             COARS2 (page 1424), CHKCORS (page 1425).
    pub fn step_coarse_align(&mut self) -> Result<bool, CoarseAlignError>;

    /// Transition to fine alignment after coarse align completes.
    ///
    /// Clears CHAN12 bits 4 (coarse) and 5 (zero); enables fine align mode.
    /// The IMU will remain in fine align until `power_down()` is called.
    ///
    /// This method consumes `self` and produces `ImuController<FineAligned>`.
    /// The caller must have previously verified `step_coarse_align() == Ok(true)`.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUFINE (page 1427).
    pub fn fine_align_with_stars(
        self,
        star_a_sm: Vec3,
        star_b_sm: Vec3,
        star_a_desired: Vec3,
        star_b_desired: Vec3,
    ) -> Result<ImuController<FineAligned>, FineAlignError>;

    /// Power down from coarse aligned state.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO → MODEEXIT.
    pub fn power_down(self) -> ImuController<Unaligned>;
}

impl ImuController<FineAligned> {
    /// Power down from fine aligned state (e.g., IMU fail, STBY).
    ///
    /// Zeros all CDU counters and clears CHAN12 bits.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO (page 1421),
    ///             MODABORT path.
    pub fn power_down(self) -> ImuController<Unaligned>;

    /// Access the underlying ImuIo for navigation use (read CDU, read PIPA).
    ///
    /// Only available in FineAligned state — navigation must not use IMU data
    /// before fine alignment is complete.
    pub fn imu(&self) -> &dyn ImuIo;

    /// Access the underlying ImuIo mutably (for torque_gyro, read_pipa).
    pub fn imu_mut(&mut self) -> &mut dyn ImuIo;
}
```

### Internal: AXISGEN two-vector method

The following describes the algorithm used inside `fine_align_with_stars()`. This is not a separate public function — it is implemented within the `ImuController<CoarseAligned>` method.

```
Inputs:
  SA = star_a_sm   (unit vector, observed, in SM frame)
  SB = star_b_sm   (unit vector, observed, in SM frame)
  SB_d = star_b_desired  (unit vector, catalog, in reference/NB frame)
  SA_d = star_a_desired  (unit vector, catalog, in reference/NB frame)

Step 1 — Build SM orthonormal triad:
  VA = UNIT(SA × SB)
  WA = SA × VA

Step 2 — Build NB orthonormal triad:
  VB = UNIT(SA_d × SB_d)
  WB = SA_d × VB

Step 3 — Form rotation matrix R (SM → NB):
  R = SA_d ⊗ SA + VB ⊗ VA + WB ⊗ WA
  (outer-product sum: each column of R is how the SM basis vector is expressed in NB)

Step 4 — Store R columns as XDC / YDC / ZDC (desired SM orientation in SM coords)

Step 5 — CALCGTA: compute gyro torque angles IGC / MGC / OGC from XDC/YDC/ZDC
  ZPRIME = UNIT(-XD3, 0, XD1)
  IGC = atan2(ZPRIME[0], ZPRIME[2])   # Y gyro
  MGC = atan2(XDC[1], cos(IGC))       # Z gyro  
  OGC = atan2(ZPRIME · YDC, ZPRIME · ZDC)  # X gyro

Step 6 — Apply torque: call imu.torque_gyro(axis, angle_to_pulses(OGC/IGC/MGC))
```

Collinear check: if `|SA × SB| < 1e-6`, the stars are too close together to form a valid triad → `Err(FineAlignError::CollinearStars)`.

### Scale factors

| Quantity | Rust unit | AGC scale | Notes |
|---|---|---|---|
| CDU angles | `CduAngle` (i16) | 2¹⁵ counts = 1 rev | Raw hardware counts |
| THETAD | `CduAngle` | same | Desired CDU angle |
| Gyro torque IGC/MGC/OGC | f64 (revolutions) | B-1 (half-rev) | Fraction of revolution |
| Gyro torque pulses | i16 | ~0.00088° per pulse (varies) | Hardware-specific |
| Alignment tolerance | 2° | `COARSTOL = -.01111 half-rev` | 0.01111 × 180° = 2° |
| Star vectors | Vec3 (f64) | B-1 (half-unit in AGC) | Unit vectors |

### Restart safety

- Coarse alignment (IMUCOARS) is protected by a Waitlist task (`COARS` runs as a 4 ms Waitlist task in the AGC). In Rust: `step_coarse_align()` is designed to be called from a Waitlist callback, not from a busy loop.
- `fine_align_with_stars()` is a short atomic computation; no restart phase protection required for the computation itself. However, if it is called as part of a larger alignment program (P52), the outer program must set Group 4 phase protection.
- `power_down()` is always safe to call and is idempotent in effect.

## Invariants

1. **Typestate completeness**: the Rust type system prevents:
   - Calling `fine_align_with_stars()` from `ImuController<Unaligned>`.
   - Accessing `imu()` for navigation from `ImuController<Unaligned>` or `ImuController<CoarseAligned>`.
   - These are compile-time errors, not runtime checks.
2. `step_coarse_align()` returns `Err(CoarseAlignError::ToleranceExceeded)` if a full drive sequence completes but gimbals remain outside 2°.
3. `fine_align_with_stars()` returns `Err(FineAlignError::CollinearStars)` for stars separated by less than ~3° (|cross product| < 1e-6 in unit-vector space).
4. No heap allocation. All intermediate matrices and vectors are stack-allocated `[f64; 3]` and `[[f64; 3]; 3]` arrays.
5. No blocking spin-waits. `step_coarse_align()` returns immediately; the Waitlist reschedules it.
6. The `power_down()` transition always succeeds: `ImuController<S>` → `ImuController<Unaligned>` for any `S`.

## Test Cases

### TC-IMU-1: Coarse alignment to identity REFSMMAT
```
Setup:    let imu = ImuImpl::<Unaligned>::new();
          imu.inject_cdus([CduAngle(0), CduAngle(0), CduAngle(0)]);
          let ctrl = ImuController::new(imu)
                       .coarse_align_to(IDENTITY_MAT3);
Action:   let result = ctrl.step_coarse_align();
Expected: result == Ok(true)     // already at THETAD=0, within 2°
```

### TC-IMU-2: Two-vector fine alignment — collinear stars rejected
```
Setup:    let star_a = [1.0_f64, 0.0, 0.0];
          let star_b = [1.0_f64, 0.0, 0.0];   // same direction as star_a
          ... (coarse aligned controller)
Action:   let result = ctrl.fine_align_with_stars(star_a, star_b, star_a, star_b);
Expected: result == Err(FineAlignError::CollinearStars)
```

### TC-IMU-3: Two-vector fine alignment — orthogonal stars produce valid rotation
```
Setup:    star_a_sm = [1.0, 0.0, 0.0]  (observed in SM)
          star_b_sm = [0.0, 1.0, 0.0]
          star_a_desired = [1.0, 0.0, 0.0]   (catalog, in NB)
          star_b_desired = [0.0, 1.0, 0.0]
          (perfect alignment: SM == NB, no rotation needed)
Action:   let result = ctrl.fine_align_with_stars(star_a_sm, star_b_sm,
                           star_a_desired, star_b_desired);
Expected: result.is_ok() == true
          let fine_ctrl = result.unwrap();
          // Gyro torque angles should all be ≈ 0 (no rotation needed)
          // The returned controller is of type ImuController<FineAligned>
```

### TC-IMU-4: Power-down cycle — Unaligned → CoarseAligned → Unaligned
```
Setup:    let ctrl_u = ImuController::new(ImuImpl::<Unaligned>::new());
Action:   let ctrl_c = ctrl_u.coarse_align_to(IDENTITY_MAT3);
          let ctrl_u2 = ctrl_c.power_down();
Expected: // ctrl_u2 has type ImuController<Unaligned>
          // The following would be compile errors (verifiable in test):
          //   ctrl_c.imu()   <- moved into power_down
          //   ctrl_u.imu()   <- ImuController<Unaligned> has no imu() method
```

### TC-IMU-5: Coarse align tolerance — 2° threshold
```
Setup:    CDU reads [CduAngle(0), CduAngle(0), CduAngle(0)]
          THETAD from REFSMMAT that differs by exactly 2.0° on one axis.
          2° in CduAngle units = round(2/360 × 2^15) = round(181.5) = 182 counts.
Action:   let result = ctrl.step_coarse_align();
          (drive loop runs; check is applied after drive settles)
Expected: result == Ok(true) if actual error ≤ 2°
          result == Err(CoarseAlignError::ToleranceExceeded) if > 2°
Note:     COARSTOL DEC -.01111 = -0.01111 half-rev = -2.0° (from AGC source page 1425).
```

## agc-sim Impact

- `SimHardware`: add `imu_state: ImuAlignmentState` enum with three variants corresponding to the three typestate markers, displayed in the Mission State panel.
- `DskyState`: add `no_att_light: bool` (rendered as "NO ATT" annunciator, driven by the `NOATTOFF` routine called during coarse → fine transition).
- `SimLog`: emit `log::info!("IMU coarse align complete")` from `coarse_align_to()`, `log::info!("IMU fine align complete, gyro corrections: IGC={:.4} MGC={:.4} OGC={:.4}")` from `fine_align_with_stars()`.
- `agc_sim` DSKY panel: render a FINE ALIGN status indicator in the lights row (on when `ImuController<FineAligned>` is active).
- No new keyboard bindings required; P52 (IMU alignment program, handled by the parallel analyst) will drive ImuController transitions via V37 N52.
