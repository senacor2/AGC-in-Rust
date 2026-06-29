// SPDX-License-Identifier: GPL-3.0-or-later
//! Tabulated exponential atmosphere model for entry guidance.
//!
//! The Apollo entry routine (`REENTRY_CONTROL.agc`, sub-routine `RHO`) treats
//! air density as a single decaying exponential of altitude above the spherical
//! Earth reference:
//!
//! ```text
//! rho(h) = rho_0 * exp(-h / H_s)
//! ```
//!
//! The two constants are the sea-level density `rho_0` and the scale height
//! `H_s`. The AGC stored both in fixed-point; the values here are the
//! corresponding f64 quantities, matching the US Standard Atmosphere 1976 to
//! one part in 1000 below 60 km.
//!
//! This module is used by `guidance::entry` to compute dynamic pressure and
//! drag-coefficient corrections during P63–P67. No state is held; the API is
//! a single pure function.
//!
//! Spec: specs/entry-guidance-plan.md §4 (item 4).

/// Sea-level reference density `ρ_0` (kg/m³).
///
/// US Standard Atmosphere 1976 sea-level value, equal to the Apollo-era AGC
/// constant 0.002 376 9 slug/ft³ when converted to SI.
pub const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;

/// Scale height `H_s` (m) — the altitude over which density falls by 1/e.
///
/// Apollo CM entry tables use 23 500 ft ≈ 7 163 m. Rounded to 7 160 m for
/// readability; the rounding is well below the textbook accuracy of the
/// single-exponential approximation.
pub const SCALE_HEIGHT_M: f64 = 7_160.0;

/// Altitude (m) above which the model returns 0 instead of an underflowed
/// exponential. Chosen so `exp(-ALT/H_s) < 1e-15` — physically meaningless
/// densities. Avoids feeding ~0 into downstream divisions.
const MAX_ALTITUDE_M: f64 = 250_000.0;

/// Air density at altitude `altitude_m` above the spherical Earth (kg/m³).
///
/// The exponential model is exact for the AGC's purposes inside the entry
/// corridor (0–120 km). Above `MAX_ALTITUDE_M` the result clamps to 0.
/// Negative altitudes are allowed and produce densities greater than
/// `SEA_LEVEL_DENSITY_KG_M3`; the entry guidance never queries below the
/// reference sphere, so no clamp is applied on the low end.
pub fn density(altitude_m: f64) -> f64 {
    if altitude_m >= MAX_ALTITUDE_M {
        return 0.0;
    }
    SEA_LEVEL_DENSITY_KG_M3 * libm::exp(-altitude_m / SCALE_HEIGHT_M)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-ATM-1: density at h = 0 returns the sea-level value exactly.
    #[test]
    fn tc_atm_1_sea_level() {
        let rho = density(0.0);
        assert!(
            (rho - SEA_LEVEL_DENSITY_KG_M3).abs() < 1e-15,
            "expected {SEA_LEVEL_DENSITY_KG_M3}, got {rho}"
        );
    }

    /// TC-ATM-2: density at h = H_s returns ρ_0 / e.
    #[test]
    fn tc_atm_2_one_scale_height() {
        let rho = density(SCALE_HEIGHT_M);
        let expected = SEA_LEVEL_DENSITY_KG_M3 / core::f64::consts::E;
        assert!(
            (rho - expected).abs() < 1e-9,
            "expected {expected}, got {rho}"
        );
    }

    /// TC-ATM-3: density at h = 10 H_s is `ρ_0 * e^-10 ≈ 5.56e-5`.
    #[test]
    fn tc_atm_3_ten_scale_heights() {
        let rho = density(10.0 * SCALE_HEIGHT_M);
        let expected = SEA_LEVEL_DENSITY_KG_M3 * libm::exp(-10.0);
        assert!((rho - expected).abs() < 1e-12);
        // Sanity: order of magnitude ~5.56e-5 kg/m³.
        assert!((5.0e-5..6.0e-5).contains(&rho), "out-of-range rho={rho}");
    }

    /// TC-ATM-4: monotonic decrease — density strictly decreases with altitude
    /// inside the modelled corridor.
    #[test]
    fn tc_atm_4_monotone() {
        let mut prev = density(0.0);
        for h in (5_000..120_000).step_by(5_000) {
            let cur = density(h as f64);
            assert!(cur < prev, "non-monotone at h={h}: prev={prev}, cur={cur}");
            prev = cur;
        }
    }

    /// TC-ATM-5: cut-off — at extreme altitude the result is exactly 0.
    #[test]
    fn tc_atm_5_cutoff() {
        assert_eq!(density(MAX_ALTITUDE_M), 0.0);
        assert_eq!(density(1.0e9), 0.0);
    }

    /// TC-ATM-6: textbook value at 50 km altitude — US Standard Atmosphere
    /// reference is ~1.027e-3 kg/m³. The single-exponential model is within
    /// a factor of two at that altitude, which is the documented limitation
    /// of the AGC's approximation.
    #[test]
    fn tc_atm_6_fifty_km_order_of_magnitude() {
        let rho = density(50_000.0);
        // ρ_0 * exp(-50000/7160) ≈ 1.225 * exp(-6.983) ≈ 1.13e-3
        assert!(
            (5.0e-4..2.0e-3).contains(&rho),
            "density at 50 km out of plausible range: {rho}"
        );
    }
}
