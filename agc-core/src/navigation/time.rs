//! Time conversion utilities and Earth-rotation constants.
//!
//! The AGC uses the Greenwich Hour Angle of Aries (GHA) — the angle between
//! the Greenwich meridian and the vernal-equinox direction — to bridge the
//! inertial and Earth-fixed frames. GHA is parameterised by a one-time
//! epoch value (`gha_epoch_rad`, set by ground uplink and survives FRESH
//! START) and the sidereal rotation rate `OMEGA_EARTH`.
//!
//! Spec: specs/gmst-ecef-plan.md §2.

use crate::types::Met;

/// Earth sidereal rotation rate (rad/s). IAU standard value.
///
/// Used to compute the Greenwich Hour Angle at any mission elapsed time:
/// `gha(t) = gha_epoch_rad + OMEGA_EARTH * t`.
///
/// Spec: specs/gmst-ecef-plan.md §3 — moved from `programs/p21.rs` so a
/// single canonical definition serves P21, P22, P23 and entry guidance.
pub const OMEGA_EARTH: f64 = 7.292_115_085_5e-5;

/// Convert mission elapsed time to the Greenwich Hour Angle of Aries (rad).
///
/// `gha_epoch_rad` is the GHA at MET = 0, supplied by ground uplink
/// (`AgcState.gha_epoch_rad`). The result is unbounded — call sites that
/// feed the angle into `sin` / `cos` do not need normalisation, but call
/// sites that compare angles directly should reduce modulo 2π themselves.
///
/// Spec: specs/gmst-ecef-plan.md §2, §5.
pub fn met_to_gha(t: Met, gha_epoch_rad: f64) -> f64 {
    gha_epoch_rad + OMEGA_EARTH * t.to_seconds()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-TIME-1: at MET=0 the GHA is the epoch angle itself.
    #[test]
    fn tc_time_1_met_zero_is_epoch() {
        assert_eq!(met_to_gha(Met(0), 0.0), 0.0);
        assert_eq!(met_to_gha(Met(0), 1.234), 1.234);
    }

    /// TC-TIME-2: one sidereal day of MET advances GHA by `OMEGA_EARTH * 86400`.
    ///
    /// MET is centiseconds, so 86 400 s = 8 640 000 centiseconds.
    #[test]
    fn tc_time_2_sidereal_day_advance() {
        let gha = met_to_gha(Met(86_400 * 100), 0.0);
        let expected = OMEGA_EARTH * 86_400.0;
        assert!(
            (gha - expected).abs() < 1e-12,
            "expected {expected}, got {gha}"
        );
    }

    /// TC-TIME-3: linearity in `gha_epoch_rad` — adding a constant to the epoch
    /// shifts the result by the same constant.
    #[test]
    fn tc_time_3_linearity_in_epoch() {
        let t = Met(500_000);
        let a = met_to_gha(t, 0.0);
        let b = met_to_gha(t, 1.5);
        assert!((b - a - 1.5).abs() < 1e-14);
    }

    /// TC-TIME-4: linearity in MET — doubling the elapsed time doubles the
    /// rotation contribution (with `gha_epoch_rad = 0`).
    #[test]
    fn tc_time_4_linearity_in_met() {
        let a = met_to_gha(Met(100_000), 0.0);
        let b = met_to_gha(Met(200_000), 0.0);
        assert!((b - 2.0 * a).abs() < 1e-14);
    }
}
