//! P51 — IMU Orientation Determination / P52 — IMU Realign.
//!
//! Crew sights two stars through the optics, marks their unit vectors in both
//! ECI and stable-member frames, and the program computes a new REFSMMAT via
//! the two-vector determination method (SMNB).  Fine alignment follows.
//!
//! AGC source: P51-P53.agc, IMU_CALIBRATION_AND_ALIGNMENT.agc.

use crate::control::imu_control::ImuControl;
use crate::math::linalg::{cross, norm, unit};
use crate::navigation::state_vector::Refsmmat;
use crate::types::{Mat3x3, Vec3};

/// Program number for IMU orientation determination.
pub const P51_NUMBER: u8 = 51;

/// Program number for IMU realignment.
pub const P52_NUMBER: u8 = 52;

/// P51/P52 execution phases.
///
/// AGC source: P51-P53.agc — P52 phase sequencing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum P51Phase {
    /// Crew selects star pair for alignment.
    StarSelect,
    /// Mark on first star (crew uses optics).
    Mark1,
    /// Mark on second star.
    Mark2,
    /// Computing new REFSMMAT from star sightings.
    Computing,
    /// Displaying results (gimbal angles, gyro torquing).
    DisplayResult,
    /// Executing fine align (gyro torquing).
    FineAlign,
    /// Complete.
    Complete,
}

/// Persistent P51/P52 state.
#[derive(Clone, Debug)]
pub struct P51State {
    /// Current execution phase.
    pub phase: P51Phase,
    /// First star unit vector in ECI frame (catalog).
    pub star1_eci: Vec3,
    /// Second star unit vector in ECI frame (catalog).
    pub star2_eci: Vec3,
    /// First star measurement in stable-member frame (from optics mark).
    pub star1_sm: Vec3,
    /// Second star measurement in stable-member frame (from optics mark).
    pub star2_sm: Vec3,
    /// Computed new REFSMMAT (set after `compute_refsmmat`).
    pub new_refsmmat: Option<Refsmmat>,
    /// Gyro torquing angles derived from alignment (radians per axis).
    ///
    /// AGC source: IMU_CALIBRATION_AND_ALIGNMENT.agc — FINEALGN torquing angles.
    pub gyro_torque: [f64; 3],
}

impl P51State {
    /// Construct a new, uninitialised P51 state.
    pub const fn new() -> Self {
        Self {
            phase: P51Phase::StarSelect,
            star1_eci: [0.0; 3],
            star2_eci: [0.0; 3],
            star1_sm: [0.0; 3],
            star2_sm: [0.0; 3],
            new_refsmmat: None,
            gyro_torque: [0.0; 3],
        }
    }

    /// Set star sighting data from crew optics marks.
    ///
    /// `s1_eci` / `s2_eci` are unit vectors from the star catalog in ECI.
    /// `s1_sm` / `s2_sm` are the corresponding measurements in stable-member
    /// frame as read from the CDU/optics.
    ///
    /// Advances phase to `Computing`.
    ///
    /// AGC source: P51-P53.agc — MARKS subroutine, star pair data loading.
    pub fn set_star_marks(&mut self, s1_eci: Vec3, s1_sm: Vec3, s2_eci: Vec3, s2_sm: Vec3) {
        self.star1_eci = s1_eci;
        self.star2_eci = s2_eci;
        self.star1_sm = s1_sm;
        self.star2_sm = s2_sm;
        self.phase = P51Phase::Computing;
    }

    /// Compute REFSMMAT from two star sightings using the SMNB two-vector
    /// determination method.
    ///
    /// The algorithm builds an orthonormal triad from each pair of vectors and
    /// then forms the rotation matrix between the two triads:
    ///
    /// 1. Triad from ECI vectors: `e1 = unit(s1)`, `e3 = unit(s1 × s2)`,
    ///    `e2 = e3 × e1`.
    /// 2. Same triad from SM vectors.
    /// 3. REFSMMAT = R_sm_triad · R_eci_triad^T.
    ///
    /// On success sets `new_refsmmat` and advances phase to `DisplayResult`.
    /// On degenerate input (collinear stars) leaves phase unchanged.
    ///
    /// AGC source: IMU_CALIBRATION_AND_ALIGNMENT.agc — SMNB routine.
    pub fn compute_refsmmat(&mut self) {
        // Build ECI orthonormal triad.
        let e1_eci = unit(&self.star1_eci);
        let cross_eci = cross(&self.star1_eci, &self.star2_eci);
        if norm(&cross_eci) < 1e-10 {
            // Stars too close together; cannot determine orientation.
            return;
        }
        let e3_eci = unit(&cross_eci);
        let e2_eci = cross(&e3_eci, &e1_eci);

        // Build SM orthonormal triad.
        let e1_sm = unit(&self.star1_sm);
        let cross_sm = cross(&self.star1_sm, &self.star2_sm);
        if norm(&cross_sm) < 1e-10 {
            return;
        }
        let e3_sm = unit(&cross_sm);
        let e2_sm = cross(&e3_sm, &e1_sm);

        // R_eci_triad: columns are e1_eci, e2_eci, e3_eci (ECI → triad).
        // R_sm_triad:  columns are e1_sm,  e2_sm,  e3_sm.
        //
        // REFSMMAT maps ECI → SM, so:
        //   REFSMMAT = R_sm_triad · R_eci_triad^T
        //
        // Written element-wise to avoid heap allocation.
        let mut mat: Mat3x3 = [[0.0; 3]; 3];
        let eci_triad = [e1_eci, e2_eci, e3_eci]; // columns of R_eci_triad
        let sm_triad = [e1_sm, e2_sm, e3_sm]; // columns of R_sm_triad

        // mat[i][j] = sum_k R_sm[i][k] * R_eci[j][k]
        //           = sum_k sm_triad[k][i] * eci_triad[k][j]
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0_f64;
                for k in 0..3 {
                    s += sm_triad[k][i] * eci_triad[k][j];
                }
                mat[i][j] = s;
            }
        }

        // Compute gyro torquing angles as the small-angle approximation of
        // the residual rotation relative to the current SM orientation.
        // For simplicity we use the skew-symmetric part of (mat - I).
        self.gyro_torque[0] = mat[2][1] - mat[1][2];
        self.gyro_torque[1] = mat[0][2] - mat[2][0];
        self.gyro_torque[2] = mat[1][0] - mat[0][1];

        self.new_refsmmat = Some(Refsmmat(mat));
        self.phase = P51Phase::DisplayResult;
    }

    /// Return a reference to the computed REFSMMAT, or `None` if not yet computed.
    pub fn get_refsmmat(&self) -> Option<&Refsmmat> {
        self.new_refsmmat.as_ref()
    }

    /// Apply the computed REFSMMAT to `imu` and start fine alignment.
    ///
    /// AGC source: IMU_CALIBRATION_AND_ALIGNMENT.agc — FINEALGN entry.
    pub fn apply_to_imu(&mut self, imu: &mut ImuControl) {
        if let Some(rsm) = self.new_refsmmat {
            imu.start_fine_align(rsm.0);
            self.phase = P51Phase::FineAlign;
        }
    }

    /// Mark fine align complete (called after ImuControl reports done).
    pub fn finalize(&mut self) {
        self.phase = P51Phase::Complete;
    }
}

impl Default for P51State {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::norm;

    #[test]
    fn new_state_starts_at_star_select() {
        let s = P51State::new();
        assert_eq!(s.phase, P51Phase::StarSelect);
        assert!(s.new_refsmmat.is_none());
    }

    #[test]
    fn set_star_marks_populates_vectors() {
        let mut s = P51State::new();
        let s1e = [1.0, 0.0, 0.0];
        let s1m = [0.0, 1.0, 0.0];
        let s2e = [0.0, 1.0, 0.0];
        let s2m = [0.0, 0.0, 1.0];
        s.set_star_marks(s1e, s1m, s2e, s2m);
        assert_eq!(s.star1_eci, s1e);
        assert_eq!(s.star1_sm, s1m);
        assert_eq!(s.star2_eci, s2e);
        assert_eq!(s.star2_sm, s2m);
        assert_eq!(s.phase, P51Phase::Computing);
    }

    #[test]
    fn compute_refsmmat_orthogonal_stars_produces_valid_matrix() {
        let mut s = P51State::new();
        // ECI: X and Y axes; SM: Y and Z axes.
        // Expected REFSMMAT rotates X→Y, Y→Z, Z→X (cyclic permutation).
        s.set_star_marks(
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        );
        s.compute_refsmmat();
        assert_eq!(s.phase, P51Phase::DisplayResult);
        let rsm = s.get_refsmmat().expect("REFSMMAT must be set");

        // Each row of a rotation matrix must be a unit vector.
        for row in rsm.0.iter() {
            let n = norm(row);
            assert!((n - 1.0).abs() < 1e-10, "row norm={}", n);
        }
    }

    #[test]
    fn refsmmat_columns_are_unit_vectors() {
        let mut s = P51State::new();
        s.set_star_marks(
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        );
        s.compute_refsmmat();
        let rsm = s.get_refsmmat().unwrap();
        let m = rsm.0;
        // Extract columns.
        for j in 0..3 {
            let col = [m[0][j], m[1][j], m[2][j]];
            let n = norm(&col);
            assert!((n - 1.0).abs() < 1e-10, "col {} norm={}", j, n);
        }
    }

    #[test]
    fn collinear_stars_does_not_set_refsmmat() {
        let mut s = P51State::new();
        // Same vector for both stars → cross product is zero.
        s.set_star_marks(
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        );
        s.compute_refsmmat();
        assert!(s.new_refsmmat.is_none());
    }

    /// Verify identity alignment: when both triads are identical REFSMMAT ≈ I.
    #[test]
    fn identity_alignment_produces_identity_refsmmat() {
        let mut s = P51State::new();
        s.set_star_marks(
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        s.compute_refsmmat();
        let rsm = s.get_refsmmat().expect("must compute");
        // Should be close to the identity matrix.
        let identity: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (rsm.0[i][j] - identity[i][j]).abs() < 1e-10,
                    "mat[{}][{}]={} expected {}",
                    i,
                    j,
                    rsm.0[i][j],
                    identity[i][j]
                );
            }
        }
    }
}
