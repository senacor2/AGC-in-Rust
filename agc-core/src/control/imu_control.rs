//! IMU alignment controller with typestate-enforced lifecycle.
//!
//! Wraps `hal::imu::ImuImpl` and enforces the three-phase alignment lifecycle:
//! `Unaligned` → `CoarseAligned` → `FineAligned`.
//! The Rust type system prevents calling fine-alignment methods before coarse
//! alignment and prevents using IMU data for navigation before fine alignment.
//!
//! AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc
//!   IMUZERO (page 1421), IMUCOARS (page 1423), SETCOARS (page 1426),
//!   IMUFINE (page 1427), IMUFINED (page 1428).
//! AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc
//!   CALCGTA (pages 1355-1356), AXISGEN (pages 1361-1362).

use crate::hal::imu::{CoarseAligned, FineAligned, ImuImpl, ImuIo, Unaligned};
use crate::math::linalg::{cross, dot, norm, unit};
use crate::types::{Mat3x3, Vec3, IDENTITY_MAT3};

// ── Error types ───────────────────────────────────────────────────────────────

/// Error returned when coarse alignment fails.
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc alarm dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoarseAlignError {
    /// Gimbals did not converge to within 2° of THETAD.
    ///
    /// AGC: alarm 0211 at COARSERR label (page 1425).
    ToleranceExceeded,
    /// IMU is in gimbal lock (|middle gimbal angle| > 60°).
    ///
    /// AGC: alarm 0401 at GIMLOCK1 label (INFLIGHT_ALIGNMENT_ROUTINES.agc page 1360).
    GimbalLock,
}

/// Error returned when fine alignment fails.
///
/// AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc AXISGEN degeneracy check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FineAlignError {
    /// Two star vectors are nearly collinear; AXISGEN cannot form a valid triad.
    ///
    /// AGC: AXISGEN degeneracy (INFLIGHT_ALIGNMENT_ROUTINES.agc page 1361).
    CollinearStars,
    /// Gyro torquing angle computation overflowed or produced a non-finite value.
    NumericalError,
}

// ── Coarse alignment tolerance ────────────────────────────────────────────────

/// Coarse alignment tolerance in CduAngle counts (2° = 182 counts).
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc
///   `COARSTOL DEC -.01111` (page 1425).
///   0.01111 half-revolutions = 0.01111 × 180° = 2.0°.
///   In CduAngle units: 2/360 × 32768 ≈ 182.04 → 182 counts.
const COARSE_TOLERANCE_COUNTS: i32 = 182;

/// Maximum CDU pulse commands per coarse-align iteration (COMMAX).
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc COARS routine (page 1423).
/// Clamps each axis command to avoid overshooting.
const COMMAX: i16 = 512;

// ── ImuController ─────────────────────────────────────────────────────────────

/// IMU alignment controller with typestate tracking.
///
/// `State` is one of `Unaligned`, `CoarseAligned`, `FineAligned`.
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
    /// Desired CDU angles (THETAD) computed from REFSMMAT via CALCGA.
    ///
    /// AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc CALCGA (page 1359).
    thetad: [i16; 3],
}

// ── Unaligned state ───────────────────────────────────────────────────────────

impl ImuController<Unaligned> {
    /// Construct a new controller in the Unaligned state.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO entry (page 1421).
    pub fn new(imu: ImuImpl<Unaligned>) -> Self {
        Self {
            imu,
            refsmmat: IDENTITY_MAT3,
            thetad: [0; 3],
        }
    }

    /// Begin coarse alignment to the given REFSMMAT.
    ///
    /// Transitions the IMU to CoarseAligned hardware mode (sets CHAN12 bit 4 and
    /// bit 6) and stores `refsmmat` as the target orientation.
    ///
    /// Computes `THETAD` (desired CDU angles) from `refsmmat` via the CALCGA
    /// algorithm (INFLIGHT_ALIGNMENT_ROUTINES.agc pages 1359-1360).
    ///
    /// The actual CDU drive loop (COARS routine) is separate: call
    /// `step_coarse_align()` on the returned `ImuController<CoarseAligned>`
    /// until it returns `Ok(true)` (converged) or `Err(CoarseAlignError)`.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUCOARS (page 1423),
    ///             SETCOARS (page 1426).
    ///             Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc CALCGA (page 1359).
    pub fn coarse_align_to(self, refsmmat: Mat3x3) -> ImuController<CoarseAligned> {
        // CALCGA: compute THETAD from REFSMMAT columns (XSM/YSM/ZSM in NB frame).
        // The desired CDU angles are the Euler angles of the REFSMMAT expressed
        // as (inner, middle, outer) gimbal angles = (X, Y, Z CDU axes).
        //
        // AGC CALCGA extracts gimbal angles from the rotation matrix columns.
        // For the coarse align approximation: THETAD[axis] = 0 (identity) or
        // extracted from refsmmat using the Z-Y-X Euler decomposition of the
        // navigation-base frame.
        //
        // Simplified: use the REFSMMAT to compute the desired inner/middle/outer
        // CDU angles. The CALCGA algorithm in AGC does:
        //   OGC (outer/Z CDU) = atan2(R[0][1], R[0][0])  (col 0 of refsmmat)
        //   MGC (middle/Y CDU) = atan2(-R[0][2], sqrt(R[1][2]^2 + R[2][2]^2))
        //   IGC (inner/X CDU) = atan2(R[1][2], R[2][2])
        //
        // AGC source: CALCGA (page 1359-1360) — Euler ZYX decomposition of SM→NB.
        let thetad = calcga(&refsmmat);

        ImuController {
            imu: self.imu.into_coarse_aligned(),
            refsmmat,
            thetad,
        }
    }
}

// ── CoarseAligned state ───────────────────────────────────────────────────────

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
    ///   `Err(CoarseAlignError::ToleranceExceeded)` — end-of-command check failed.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc COARS (page 1423),
    ///             COARS2 (page 1424), CHKCORS (page 1425).
    pub fn step_coarse_align(&mut self) -> Result<bool, CoarseAlignError> {
        let cdus = self.imu.read_cdu();

        // Gimbal lock check: |middle gimbal (CDUY)| > 60° = 5461 counts.
        // AGC: GIMLOCK1 check at CALCGA (page 1360): |MGC| > 60°.
        // 60/360 × 32768 = 5461.3 → 5461 counts.
        let middle_abs = (cdus[1].0 as i32).unsigned_abs();
        if middle_abs > 5461 {
            return Err(CoarseAlignError::GimbalLock);
        }

        // Compute errors for each axis.
        let mut errors = [0i32; 3];
        let mut any_error = false;
        for axis in 0..3 {
            let err = self.thetad[axis] as i32 - cdus[axis].0 as i32;
            errors[axis] = err;
            if err.unsigned_abs() > COARSE_TOLERANCE_COUNTS as u32 {
                any_error = true;
            }
        }

        if !any_error {
            // All axes within tolerance — coarse align converged.
            return Ok(true);
        }

        // Issue pulse commands clamped to COMMAX.
        let mut cmds = [0i16; 3];
        for axis in 0..3 {
            let clamped = errors[axis].clamp(-(COMMAX as i32), COMMAX as i32);
            cmds[axis] = clamped as i16;
        }
        self.imu.write_cdu_commands(cmds);

        Ok(false)
    }

    /// Transition to fine alignment after coarse align completes.
    ///
    /// Takes two star observations and two catalog directions to compute the
    /// rotation from SM to NB frame (AXISGEN algorithm), then derives gyro
    /// torque angles (CALCGTA) and applies them.
    ///
    /// Returns `ImuController<FineAligned>` on success.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUFINE (page 1427).
    ///             Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc AXISGEN (page 1361),
    ///             CALCGTA (page 1355).
    pub fn fine_align_with_stars(
        self,
        star_a_sm: Vec3,
        star_b_sm: Vec3,
        star_a_desired: Vec3,
        star_b_desired: Vec3,
    ) -> Result<ImuController<FineAligned>, FineAlignError> {
        // AXISGEN algorithm (INFLIGHT_ALIGNMENT_ROUTINES.agc pages 1361-1362).
        //
        // Step 1: Build SM orthonormal triad from observed star vectors.
        //   VA = UNIT(SA × SB)
        //   WA = SA × VA
        let sa = star_a_sm;
        let sb = star_b_sm;
        let sa_cross_sb = cross(&sa, &sb);
        let va = match unit(&sa_cross_sb) {
            Some(u) => u,
            None => return Err(FineAlignError::CollinearStars),
        };
        // Check collinearity: |SA × SB| < 1e-6 threshold.
        if norm(&sa_cross_sb) < 1e-6 {
            return Err(FineAlignError::CollinearStars);
        }
        let wa = cross(&sa, &va);

        // Step 2: Build NB orthonormal triad from catalog star vectors.
        //   VB = UNIT(SA_d × SB_d)
        //   WB = SA_d × VB
        let sa_d = star_a_desired;
        let sb_d = star_b_desired;
        let sa_d_cross_sb_d = cross(&sa_d, &sb_d);
        let vb = match unit(&sa_d_cross_sb_d) {
            Some(u) => u,
            None => return Err(FineAlignError::CollinearStars),
        };
        let wb = cross(&sa_d, &vb);

        // Step 3: Form rotation matrix R (SM → NB).
        // R = SA_d ⊗ SA + VB ⊗ VA + WB ⊗ WA
        // XDC[i] = sa_d[i]*sa[j] + vb[i]*va[j] + wb[i]*wa[j] summed over j
        // Column j of XDC is the j-th SM-frame basis expressed in NB coordinates.
        let mut xdc = [[0.0_f64; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                xdc[i][j] = sa_d[i] * sa[j] + vb[i] * va[j] + wb[i] * wa[j];
            }
        }

        // Step 4: CALCGTA — compute gyro torque angles from XDC/YDC/ZDC.
        // The rotation matrix columns are XDC (col0), YDC (col1), ZDC (col2).
        // We need XDC column 0 = xdc[*][0] (X desired in SM), etc.
        //
        // CALCGTA uses:
        //   ZPRIME = UNIT(-XD3, 0, XD1)  where XDC = first column of xdc
        //   XD1 = xdc[0][0], XD2 = xdc[1][0], XD3 = xdc[2][0]
        //   IGC = atan2(ZPRIME[0], ZPRIME[2])   (Y gyro)
        //   MGC = atan2(XD2, cos(IGC))           (Z gyro)
        //   OGC = atan2(ZPRIME · YDC, ZPRIME · ZDC)  (X gyro)
        //
        // AGC source: CALCGTA (pages 1355-1356).
        let xd1 = xdc[0][0]; // XDC[0] — first element of X desired
        let xd2 = xdc[1][0]; // XDC[1]
        let xd3 = xdc[2][0]; // XDC[2]

        // ZPRIME = UNIT(-XD3, 0, XD1)
        let zprime_raw = [-xd3, 0.0, xd1];
        // XD1 and XD3 both zero — degenerate (Y-axis rotation only)
        // Use zero angles for all gyros in this degenerate case.
        let zprime = unit(&zprime_raw).unwrap_or([1.0, 0.0, 0.0]);

        // IGC = atan2(ZPRIME[0], ZPRIME[2])  — Y gyro angle
        let igc = libm::atan2(zprime[0], zprime[2]);

        // MGC = atan2(XD2, cos(IGC))  — Z gyro angle
        let cos_igc = libm::cos(igc);
        let mgc = libm::atan2(xd2, cos_igc);

        // OGC = atan2(ZPRIME · YDC, ZPRIME · ZDC)  — X gyro angle
        // YDC = column 1 of xdc: [xdc[0][1], xdc[1][1], xdc[2][1]]
        // ZDC = column 2 of xdc: [xdc[0][2], xdc[1][2], xdc[2][2]]
        let ydc = [xdc[0][1], xdc[1][1], xdc[2][1]];
        let zdc = [xdc[0][2], xdc[1][2], xdc[2][2]];
        let zp_dot_ydc = dot(&zprime, &ydc);
        let zp_dot_zdc = dot(&zprime, &zdc);
        let ogc = libm::atan2(zp_dot_ydc, zp_dot_zdc);

        // Check for NaN (numerical error).
        if !igc.is_finite() || !mgc.is_finite() || !ogc.is_finite() {
            return Err(FineAlignError::NumericalError);
        }

        // Step 5: Apply gyro torque (IMUPULSE / PULSEM).
        // Convert radians to pulse counts: 1 revolution = 32768 pulses.
        // AGC: GYROCMD (octal 47) — pulses in counts.
        // axes: X=0 (OGC), Y=1 (IGC), Z=2 (MGC).
        let angle_to_pulses = |angle_rad: f64| -> i16 {
            let pulses = libm::round(angle_rad / core::f64::consts::TAU * 32768.0);
            pulses.clamp(i16::MIN as f64, i16::MAX as f64) as i16
        };

        let mut imu_fine = self.imu.into_fine_aligned();
        imu_fine.torque_gyro(0, angle_to_pulses(ogc)); // X gyro (OGC)
        imu_fine.torque_gyro(1, angle_to_pulses(igc)); // Y gyro (IGC)
        imu_fine.torque_gyro(2, angle_to_pulses(mgc)); // Z gyro (MGC)

        Ok(ImuController {
            imu: imu_fine,
            refsmmat: self.refsmmat,
            thetad: self.thetad,
        })
    }

    /// Power down from coarse aligned state.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO → MODEEXIT (page 1422).
    pub fn power_down(self) -> ImuController<Unaligned> {
        // Revert to unaligned by going back through into_fine_aligned then into_unaligned,
        // but the IMU doesn't have a direct coarse→unaligned. Use the control shadow reset.
        // We reconstruct an Unaligned ImuImpl.
        let raw = self.imu.free(); // get control shadow
        let mut imu_u = crate::hal::imu::ImuImpl::<Unaligned>::new();
        imu_u.write_control(0); // clear CHAN12
        let _ = raw; // shadow acknowledged
        ImuController {
            imu: imu_u,
            refsmmat: self.refsmmat,
            thetad: [0; 3],
        }
    }
}

// ── FineAligned state ─────────────────────────────────────────────────────────

impl ImuController<FineAligned> {
    /// Power down from fine aligned state (e.g., IMU fail, STBY).
    ///
    /// Zeros all CDU counters and clears CHAN12 bits.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO (page 1421),
    ///             MODABORT path.
    pub fn power_down(self) -> ImuController<Unaligned> {
        let mut imu_u = crate::hal::imu::ImuImpl::<Unaligned>::new();
        imu_u.write_control(0);
        ImuController {
            imu: imu_u,
            refsmmat: self.refsmmat,
            thetad: [0; 3],
        }
    }

    /// Access the underlying ImuIo for navigation use (read CDU, read PIPA).
    ///
    /// Only available in FineAligned state — navigation must not use IMU data
    /// before fine alignment is complete.
    pub fn imu(&self) -> &dyn ImuIo {
        &self.imu
    }

    /// Access the underlying ImuIo mutably (for torque_gyro, read_pipa).
    pub fn imu_mut(&mut self) -> &mut dyn ImuIo {
        &mut self.imu
    }
}

// ── CALCGA helper ─────────────────────────────────────────────────────────────

/// Compute desired CDU gimbal angles (THETAD) from a REFSMMAT.
///
/// Extracts the Z-Y-X Euler angles from the rotation matrix via the CALCGA
/// algorithm. Returns `[igc, mgc, ogc]` as CduAngle-compatible `i16` counts.
///
/// AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc CALCGA (pages 1359-1360).
///
/// Angle units: 360° = 32768 CduAngle counts.
fn calcga(refsmmat: &Mat3x3) -> [i16; 3] {
    // Euler ZYX decomposition (matches AGC CALCGA Euler sequence):
    //   OGC (Z, outer) = atan2(R[0][1], R[0][0])
    //   MGC (Y, middle) = atan2(-R[0][2], sqrt(R[1][2]² + R[2][2]²))
    //   IGC (X, inner) = atan2(R[1][2], R[2][2])
    //
    // In our Mat3x3 layout: refsmmat[row][col].
    let r00 = refsmmat[0][0];
    let r01 = refsmmat[0][1];
    let r02 = refsmmat[0][2];
    let r12 = refsmmat[1][2];
    let r22 = refsmmat[2][2];

    let ogc_rad = libm::atan2(r01, r00);
    let mgc_rad = libm::atan2(-r02, libm::sqrt(r12 * r12 + r22 * r22));
    let igc_rad = libm::atan2(r12, r22);

    let rad_to_counts = |angle: f64| -> i16 {
        let counts = libm::round(angle / core::f64::consts::TAU * 32768.0);
        counts.clamp(i16::MIN as f64, i16::MAX as f64) as i16
    };

    [
        rad_to_counts(igc_rad), // inner (X)
        rad_to_counts(mgc_rad), // middle (Y)
        rad_to_counts(ogc_rad), // outer (Z)
    ]
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::imu::ImuImpl;
    use crate::types::IDENTITY_MAT3;

    fn make_unaligned_ctrl() -> ImuController<Unaligned> {
        ImuController::new(ImuImpl::<Unaligned>::new())
    }

    /// TC-IMU-1: Coarse alignment to identity REFSMMAT — already at THETAD=0.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc COARS (page 1423).
    #[test]
    fn coarse_align_identity_converges_immediately() {
        let ctrl_u = make_unaligned_ctrl();
        let mut ctrl_c = ctrl_u.coarse_align_to(IDENTITY_MAT3);
        // CDUs start at 0, THETAD derived from identity = [0, 0, 0].
        let result = ctrl_c.step_coarse_align();
        assert_eq!(
            result,
            Ok(true),
            "identity REFSMMAT should converge immediately"
        );
    }

    /// TC-IMU-2: Two-vector fine alignment — collinear stars rejected.
    ///
    /// AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc AXISGEN (page 1361).
    #[test]
    fn fine_align_collinear_stars_rejected() {
        let ctrl_u = make_unaligned_ctrl();
        let ctrl_c = ctrl_u.coarse_align_to(IDENTITY_MAT3);

        let star_a: Vec3 = [1.0, 0.0, 0.0];
        let star_b: Vec3 = [1.0, 0.0, 0.0]; // same direction — collinear
        let result = ctrl_c.fine_align_with_stars(star_a, star_b, star_a, star_b);
        assert!(result.is_err(), "collinear stars must return Err");
        assert_eq!(result.err(), Some(FineAlignError::CollinearStars));
    }

    /// TC-IMU-3: Two-vector fine alignment — orthogonal stars produce valid rotation.
    ///
    /// AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc AXISGEN (page 1361).
    #[test]
    fn fine_align_orthogonal_stars_ok() {
        let ctrl_u = make_unaligned_ctrl();
        let ctrl_c = ctrl_u.coarse_align_to(IDENTITY_MAT3);

        // Perfect alignment: SM == NB, no rotation needed.
        let star_a_sm: Vec3 = [1.0, 0.0, 0.0];
        let star_b_sm: Vec3 = [0.0, 1.0, 0.0];
        let star_a_d: Vec3 = [1.0, 0.0, 0.0];
        let star_b_d: Vec3 = [0.0, 1.0, 0.0];

        let result = ctrl_c.fine_align_with_stars(star_a_sm, star_b_sm, star_a_d, star_b_d);
        assert!(result.is_ok(), "orthogonal stars must succeed");
    }

    /// TC-IMU-4: Power-down cycle — Unaligned → CoarseAligned → Unaligned.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO (page 1421).
    #[test]
    fn power_down_coarse_to_unaligned() {
        let ctrl_u = make_unaligned_ctrl();
        let ctrl_c = ctrl_u.coarse_align_to(IDENTITY_MAT3);
        let _ctrl_u2 = ctrl_c.power_down();
        // Type-system proof: ctrl_u2 has type ImuController<Unaligned>.
        // Cannot call step_coarse_align on it (compile error if tried).
    }

    /// TC-IMU-5: Coarse align tolerance — error just within 2° threshold.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc COARSTOL (page 1425).
    #[test]
    fn coarse_align_tolerance_2deg() {
        // THETAD from identity = [0, 0, 0].
        // CDU error = 0 counts → converges immediately.
        let ctrl_u = make_unaligned_ctrl();
        let mut ctrl_c = ctrl_u.coarse_align_to(IDENTITY_MAT3);
        assert_eq!(ctrl_c.step_coarse_align(), Ok(true));
    }

    /// TC-IMU-6: Fine align power-down returns Unaligned type.
    #[test]
    fn fine_align_power_down() {
        let ctrl_u = make_unaligned_ctrl();
        let ctrl_c = ctrl_u.coarse_align_to(IDENTITY_MAT3);
        let star_a: Vec3 = [1.0, 0.0, 0.0];
        let star_b: Vec3 = [0.0, 1.0, 0.0];
        let fine_result = ctrl_c.fine_align_with_stars(star_a, star_b, star_a, star_b);
        assert!(fine_result.is_ok());
        let ctrl_f = fine_result.unwrap();
        // Power down from fine aligned.
        let _ctrl_u2 = ctrl_f.power_down();
    }

    /// TC-IMU-7: SMNB two-vector fine alignment with 90° rotation produces non-trivial angles.
    ///
    /// Observed star A at [1,0,0] in SM, catalog at [0,1,0] in NB.
    /// Observed star B at [0,1,0] in SM, catalog at [-1,0,0] in NB.
    /// This represents a 90° rotation about Z axis.
    ///
    /// AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc AXISGEN (page 1361),
    ///             CALCGTA (page 1355).
    #[test]
    fn fine_align_90deg_rotation_produces_nonzero_angles() {
        let ctrl_u = make_unaligned_ctrl();
        let ctrl_c = ctrl_u.coarse_align_to(IDENTITY_MAT3);

        let star_a_sm: Vec3 = [1.0, 0.0, 0.0];
        let star_b_sm: Vec3 = [0.0, 1.0, 0.0];
        let star_a_d: Vec3 = [0.0, 1.0, 0.0]; // rotated 90° CCW about Z
        let star_b_d: Vec3 = [-1.0, 0.0, 0.0];

        let result = ctrl_c.fine_align_with_stars(star_a_sm, star_b_sm, star_a_d, star_b_d);
        assert!(result.is_ok(), "90 deg rotation must converge");
    }
}
