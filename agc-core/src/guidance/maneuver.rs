//! Maneuver execution: velocity-to-be-gained and cross-product steering.
//!
//! AGC source: P40-P47.agc — S40.8/S40.9 (burn monitoring,
//! CROSS_PRODUCT_STEERING).

use crate::math::linalg::{add, cross, norm, sub, unit};
use crate::types::Vec3;

/// Cutoff threshold: burn is complete when |VG| < this value (m/s).
///
/// AGC source: P40-P47.agc — cutoff logic.
pub const VG_CUTOFF_THRESHOLD: f64 = 0.3;

/// State during an active burn.
#[derive(Clone, Copy, Debug)]
pub struct BurnState {
    /// Velocity-to-be-gained (VG) in ECI (m/s).
    /// Decremented each SERVICER cycle by the measured delta-V.
    pub vg: Vec3,
    /// Unit thrust direction (ECI).
    pub ut: Vec3,
    /// Whether the burn is complete (VG magnitude below threshold).
    pub complete: bool,
    /// Accumulated delta-V during the burn (m/s).
    pub dv_accumulated: Vec3,
}

impl BurnState {
    /// Initialize for a new burn with the given delta-V target.
    ///
    /// AGC source: P40-P47.agc — P40CSM initialization, S40.1 (VGTIG).
    pub fn new(delta_v: &Vec3) -> Self {
        Self {
            vg: *delta_v,
            ut: unit(delta_v),
            complete: false,
            dv_accumulated: [0.0; 3],
        }
    }

    /// Update VG by subtracting the measured delta-V from one SERVICER cycle.
    /// Sets `complete = true` if |VG| drops below the cutoff threshold.
    ///
    /// AGC source: P40-P47.agc — UPDATEVG, cutoff logic.
    pub fn update_vg(&mut self, measured_dv: &Vec3) {
        self.vg = sub(&self.vg, measured_dv);
        self.dv_accumulated = add(&self.dv_accumulated, measured_dv);
        if norm(&self.vg) < VG_CUTOFF_THRESHOLD {
            self.complete = true;
        }
    }

    /// Magnitude of remaining velocity-to-be-gained.
    pub fn vg_magnitude(&self) -> f64 {
        norm(&self.vg)
    }
}

/// Cross-product steering: compute attitude error from thrust direction and VG.
///
/// The cross product of the unit thrust vector (UT) with the unit VG vector
/// gives the rotation axis; its magnitude equals the sine of the angular error.
///
/// Returns attitude error [roll, pitch, yaw] in radians.
/// Roll is not controlled by VG steering and is returned as 0.0.
///
/// AGC source: P40-P47.agc — CROSS_PRODUCT_STEERING / S40.8.
pub fn cross_product_steering(ut: &Vec3, vg: &Vec3) -> [f64; 3] {
    let vg_mag = norm(vg);
    if vg_mag < 1e-30 {
        return [0.0; 3];
    }
    let ut_unit = unit(ut);
    let vg_unit = unit(vg);
    // Cross product gives the rotation axis scaled by sin(angle_error)
    let cp = cross(&ut_unit, &vg_unit);
    // cp magnitude = sin(theta); for small angles this approximates theta
    let sin_theta = norm(&cp);
    // Determine sign from cross product direction; use atan2 for correctness
    let cos_theta = crate::math::linalg::dot(&ut_unit, &vg_unit);
    let angle = libm::atan2(sin_theta, cos_theta);
    // Map cross product components to pitch and yaw; roll is uncontrolled
    // The steering error vector magnitude is the angle; direction from cp
    let scale = if sin_theta > 1e-30 { angle / sin_theta } else { 1.0 };
    [0.0, cp[1] * scale, cp[2] * scale]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::norm;

    #[test]
    fn burn_state_new_initializes_vg() {
        let dv = [100.0_f64, 0.0, 0.0];
        let bs = BurnState::new(&dv);
        assert_eq!(bs.vg, dv);
        assert!((bs.ut[0] - 1.0).abs() < 1e-14);
        assert!(!bs.complete);
        assert_eq!(bs.dv_accumulated, [0.0; 3]);
    }

    #[test]
    fn update_vg_decrements_and_detects_completion() {
        let dv = [10.0, 0.0, 0.0];
        let mut bs = BurnState::new(&dv);
        // Subtract most of VG; leaves 0.5 m/s which is above threshold
        bs.update_vg(&[9.5, 0.0, 0.0]);
        assert!(!bs.complete, "should not be complete yet, vg={}", bs.vg_magnitude());
        assert!((bs.vg[0] - 0.5).abs() < 1e-12, "vg[0]={}", bs.vg[0]);
        // Subtract the remainder; drops below 0.3 m/s cutoff
        bs.update_vg(&[0.5, 0.0, 0.0]);
        assert!(bs.complete, "should be complete");
        assert!(bs.vg_magnitude() < VG_CUTOFF_THRESHOLD);
        // Accumulated should equal original dv
        assert!((bs.dv_accumulated[0] - 10.0).abs() < 1e-12);
    }

    #[test]
    fn cross_product_steering_aligned_returns_zero() {
        // When UT and VG point in the same direction, error should be zero
        let ut = [1.0, 0.0, 0.0];
        let vg = [500.0, 0.0, 0.0];
        let err = cross_product_steering(&ut, &vg);
        assert!(err[0].abs() < 1e-14, "roll={}", err[0]);
        assert!(err[1].abs() < 1e-14, "pitch={}", err[1]);
        assert!(err[2].abs() < 1e-14, "yaw={}", err[2]);
    }

    #[test]
    fn cross_product_steering_90_degree_misalignment() {
        // UT along X, VG along Y => 90° error, cross product along −Z
        let ut = [1.0, 0.0, 0.0];
        let vg = [0.0, 500.0, 0.0];
        let err = cross_product_steering(&ut, &vg);
        // pitch (index 1) should reflect the ~90° error
        let error_mag = norm(&[err[0], err[1], err[2]]);
        let expected = core::f64::consts::FRAC_PI_2; // π/2
        assert!(
            (error_mag - expected).abs() < 1e-10,
            "error_mag={} expected={}",
            error_mag,
            expected
        );
    }
}
