use super::vector::Vec3;

/// CDU (Coupling Data Unit) gimbal angle from the IMU or optics.
///
/// Stored as a signed 16-bit count. The full `i16` range wraps over one
/// revolution: `i16::MIN` = -180°, `0` = 0°, `i16::MAX` ≈ +180° − 1 LSB.
/// 1 LSB = 360° / 2^16 ≈ 0.0055°.
///
/// This is the Rust analogue of the AGC's i15 signed-fraction CDU encoding,
/// in the same way `f64` replaces the AGC's double-precision fixed point.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct CduAngle(pub i16);

impl CduAngle {
    /// Convert to radians in the range [-π, +π).
    pub fn to_radians(self) -> f64 {
        (self.0 as f64) * (core::f64::consts::TAU / 65536.0)
    }

    /// Convert to degrees in the range [-180, +180).
    pub fn to_degrees(self) -> f64 {
        self.to_radians() * (180.0 / core::f64::consts::PI)
    }
}

impl core::fmt::Debug for CduAngle {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "CduAngle({:.4}°)", self.to_degrees())
    }
}

/// Mission elapsed time in centiseconds (1/100 s).
/// Integer counter; wraps after ~497 days.
/// Convert to f64 seconds only at call sites that need floating-point math.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Debug)]
pub struct Met(pub u32);

impl Met {
    /// Convert to seconds as f64 for use in navigation math.
    pub fn to_seconds(self) -> f64 {
        self.0 as f64 / 100.0
    }

    /// Construct from f64 seconds (truncates to nearest centisecond toward zero).
    /// Precondition: s >= 0.0 and s * 100.0 <= u32::MAX as f64.
    pub fn from_seconds(s: f64) -> Self {
        Met((s * 100.0) as u32)
    }

    /// Elapsed centiseconds from an earlier Met to self.
    /// Uses wrapping subtraction to handle the rare counter wrap-around.
    pub fn elapsed_since(self, earlier: Met) -> u32 {
        self.0.wrapping_sub(earlier.0)
    }
}

/// Delta-V vector in metres per second.
#[derive(Clone, Copy, Default, Debug)]
pub struct DeltaV(pub Vec3);

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{PI, TAU};

    const EPS: f64 = 1e-10;

    // ── CduAngle tests (TC-CDU-1 through TC-CDU-5) ──────────────────────────

    #[test]
    fn tc_cdu_1_zero() {
        assert!((CduAngle(0).to_radians() - 0.0).abs() < EPS);
        assert!((CduAngle(0).to_degrees() - 0.0).abs() < EPS);
    }

    #[test]
    fn tc_cdu_2_plus_quarter_rev() {
        // +90° = +2^14 counts
        assert!((CduAngle(16384).to_radians() - PI / 2.0).abs() < EPS);
        assert!((CduAngle(16384).to_degrees() - 90.0).abs() < 1e-8);
    }

    #[test]
    fn tc_cdu_3_minus_half_rev() {
        // i16::MIN = -32768 = -180° exact
        assert!((CduAngle(i16::MIN).to_radians() - (-PI)).abs() < EPS);
        assert!((CduAngle(i16::MIN).to_degrees() - (-180.0)).abs() < 1e-8);
    }

    #[test]
    fn tc_cdu_4_minus_quarter_rev() {
        // -90° = -2^14 counts
        assert!((CduAngle(-16384).to_radians() - (-PI / 2.0)).abs() < EPS);
        assert!((CduAngle(-16384).to_degrees() - (-90.0)).abs() < 1e-8);
    }

    #[test]
    fn tc_cdu_5_minus_one_lsb() {
        // CduAngle(-1) = -1 * TAU/65536 ≈ -0.0055°
        let expected_rad = -TAU / 65536.0;
        assert!((CduAngle(-1).to_radians() - expected_rad).abs() < EPS);
        assert!((CduAngle(-1).to_degrees() - (-360.0 / 65536.0)).abs() < 1e-6);
    }

    #[test]
    fn tc_cdu_6_plus_max_count() {
        // i16::MAX = 32767 ≈ +180° − 1 LSB
        let expected_rad = TAU * (32767.0 / 65536.0);
        assert!((CduAngle(i16::MAX).to_radians() - expected_rad).abs() < EPS);
    }

    // ── Met tests (TC-MET-1 through TC-MET-7) ───────────────────────────────

    #[test]
    fn tc_met_1_zero() {
        assert_eq!(Met(0).to_seconds(), 0.0);
    }

    #[test]
    fn tc_met_2_one_second() {
        assert_eq!(Met(100).to_seconds(), 1.0);
    }

    #[test]
    fn tc_met_3_one_day() {
        assert_eq!(Met(8_640_000).to_seconds(), 86400.0);
    }

    #[test]
    fn tc_met_4_from_seconds() {
        assert_eq!(Met::from_seconds(1.5).0, 150);
    }

    #[test]
    fn tc_met_5_truncation() {
        // 0.007 s * 100 = 0.7, truncated to 0
        assert_eq!(Met::from_seconds(0.007).0, 0);
    }

    #[test]
    fn tc_met_6_elapsed() {
        assert_eq!(Met(5).elapsed_since(Met(3)), 2);
    }

    #[test]
    fn tc_met_7_elapsed_wrapping() {
        // Met(1) - Met(0xFFFFFFFF) with wrapping = 2
        assert_eq!(Met(1).elapsed_since(Met(0xFFFFFFFF)), 2);
    }

    // ── DeltaV tests (TC-DV-1 through TC-DV-4) ──────────────────────────────

    #[test]
    fn tc_dv_1_zero() {
        let dv = DeltaV([0.0, 0.0, 0.0]);
        assert_eq!(dv.0, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn tc_dv_2_single_axis() {
        let dv = DeltaV([100.0, 0.0, 0.0]);
        assert_eq!(dv.0[0], 100.0);
    }

    #[test]
    fn tc_dv_3_agc_fixed_point_import() {
        // AGC DP pair: w_hi = 512, w_lo = 0, scale B+7
        // f64 = (512 / 16384.0) * 128.0 = 4.0 m/s
        let w_hi: f64 = 512.0;
        let dv_x = (w_hi / 16384.0) * 128.0;
        assert!((dv_x - 4.0).abs() < 1e-14);
    }

    #[test]
    fn tc_dv_4_realistic_burn() {
        // Apollo 11 LOI ≈ 867 m/s
        let dv = DeltaV([867.0, 0.0, 0.0]);
        let mag = libm::sqrt(dv.0[0] * dv.0[0] + dv.0[1] * dv.0[1] + dv.0[2] * dv.0[2]);
        assert!((mag - 867.0).abs() < 1e-10);
    }
}
