//! TVC (Thrust Vector Control) gimbal steering — PITCHDAP / YAWDAP.
//!
//! Controls the SPS engine gimbal actuators during a propulsion burn.
//! Implements a simplified proportional-derivative law in place of the AGC's
//! 6th-order cascade FWDFLTR, as permitted by ADR-001 (no interpretive-language VM).
//!
//! # Known deviation from AGC
//!
//! The AGC's PITCHDAP and YAWDAP implement a 6th-order cascade IIR filter
//! (2 or 3 biquad stages, coefficient tables N10..N10+14, `TVCDAPS.agc`
//! lines 512-578). This Rust port replaces that filter with a PD law:
//!   `raw_cmd = k_p * error + k_d * rate`
//! This is sufficient for simulation and structural testing, but would need the
//! full filter for flight-representative dynamic behaviour. A production port
//! should restore the biquad cascade from the coefficient tables.
//!
//! AGC source: Comanche055/TVCDAPS.agc
//!   PITCHDAP, YAWDAP, DAPINIT, ERRORLIM, ACTLIM, FWDFLTR, PRECOMP (pages 961-978).
//! AGC source: Comanche055/TVCEXECUTIVE.agc
//!   TVCEXEC, VARGAINS, GAINCHNG (pages 945-950).
//! AGC source: Comanche055/TVCROLLDAP.agc
//!   ROLLDAP, ROLLOGIC, DURATION (pages 984-998).

use crate::control::attitude::AttitudeError;
use crate::control::constants::TVC_ACTSAT_RAD;

/// TVC actuator command saturation limit in degrees.
///
/// AGC source: Comanche055/TVCDAPS.agc p. 978:
///   `ACTSAT DEC 253  # ACTUATOR LIMIT (6 DEG), SC.AT 1ASCREV`.
///   253 ASCREV × 85.41 arcsec/ASCREV = 21,609 arcsec = 6.002° ≈ 6°.
pub const ACTSAT_DEG: f64 = 6.0;

/// TVC actuator command saturation limit in radians.
///
/// ACTSAT_DEG converted: 6° × π/180 = 0.104_719_755 rad.
///
/// AGC source: Comanche055/TVCDAPS.agc `ACTSAT DEC 253`.
pub const ACTSAT_RAD: f64 = TVC_ACTSAT_RAD;

/// TVC roll DAP deadband in degrees.
///
/// AGC source: Comanche055/TVCROLLDAP.agc functional description p. 984:
///   "MAINTAIN OGA WITHIN 5 DEG DEADBND OF OGAD".
pub const TVC_ROLL_DEADBAND_DEG: f64 = 5.0;

/// TVC roll DAP minimum jet firing time in milliseconds.
///
/// AGC source: Comanche055/TVCROLLDAP.agc functional description p. 984:
///   "MINIMUM JET FIRING TIME = 15 MS".
pub const TVC_ROLL_MIN_FIRE_MS: f64 = 15.0;

/// TVC actuator command scaling: 1 ASCREV = 85.41 arcseconds per bit.
///
/// AGC source: Comanche055/TVCDAPS.agc p. 978 note:
///   "1 ASCREV (ACTUATOR CMD SCALING) = 85.41 ARCSEC/BIT".
pub const ASCREV_ARCSEC: f64 = 85.41;

/// TVC filter and control gains for pitch or yaw.
///
/// Gains vary with vehicle mass and configuration (CSM-only vs CSM+LEM).
/// Updated every 10 seconds during the burn by TVCEXECUTIVE / MASSPROP / S40.15.
///
/// AGC source: Comanche055/TVCEXECUTIVE.agc VARGAINS / GAINCHNG (p. 947);
///             Comanche055/TVCDAPS.agc OPTVARK (VARK gain, p. 973),
///             ACTLIM (ACTSAT limit, p. 972).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TvcGains {
    /// Proportional gain (corresponds to VARK in AGC).
    ///
    /// Units: dimensionless (actuator-command units per body-axis rate-error unit).
    /// AGC source: Comanche055/TVCDAPS.agc OPTVARK: `MP VARK`, scaled 1/(8 ASCREV).
    pub k_p: f64,

    /// Derivative gain (inverse-bandwidth, seconds).
    ///
    /// Corresponds to 1/CONACC in the AGC roll DAP; analogous damping in pitch/yaw.
    /// Units: seconds (inverse of angular acceleration per unit command).
    /// AGC source: Comanche055/TVCROLLDAP.agc `1/CONACC SC.AT B+9 SEC²/REV`.
    pub k_d: f64,

    /// Actuator command saturation limit in degrees.
    ///
    /// Always initialised to `ACTSAT_DEG` (6.0°).
    /// AGC source: Comanche055/TVCDAPS.agc ACTSAT = 253 ASCREV = 6°.
    pub limit_deg: f64,
}

impl TvcGains {
    /// Default gains (CSM-only, nominal mass at ignition).
    ///
    /// Developer must update from MASSPROP output before use in flight.
    /// Placeholder values k_p=1.0, k_d=1.0 give unit response for testing.
    pub const NOMINAL: Self = Self {
        k_p: 1.0, // placeholder; actual value from VARK pad-load
        k_d: 1.0, // placeholder; actual value from 1/CONACC
        limit_deg: ACTSAT_DEG,
    };
}

/// Compute pitch and yaw gimbal angle commands for one TVC DAP cycle.
///
/// Implements the attitude-error integration → filter → saturation chain of
/// PITCHDAP and YAWDAP.  The 6th-order cascade filter (FWDFLTR, N10..N10+14
/// coefficient tables in TVCDAPS.agc) is simplified to a PD law for this port
/// (ADR-001: interpretive language replaced by plain f64 functions).
///
/// # Simplification note
///
/// The full AGC filter chain (FWDFLTR cascades with PRECOMP nodes) is
/// intentionally omitted per the project design decision ADR-001. A production
/// flight implementation would restore the biquad cascade from the TVCDAPS.agc
/// N10..N10+14 tables. This simplification does not affect the interface contract
/// (saturation bounds, gain scheduling API, roll-axis exclusion).
///
/// Roll axis is NOT handled here — return values are `(pitch_cmd_rad, yaw_cmd_rad)`.
/// Roll is deferred to the RCS jet selector using `phase_plane_decision` with
/// `deadband = TVC_ROLL_DEADBAND_RAD`.
///
/// Output is saturated symmetrically at ±gains.limit_deg converted to radians.
///   `pitch_cmd_rad` and `yaw_cmd_rad` are in `[−ACTSAT_RAD, +ACTSAT_RAD]`.
///
/// # Invariants
/// - Output is always finite.
/// - `|pitch_cmd_rad| <= ACTSAT_RAD` and `|yaw_cmd_rad| <= ACTSAT_RAD` always hold.
/// - `gains.limit_deg > 0.0` is required; `debug_assert` enforces this.
/// - NaN inputs produce `(0.0, 0.0)`.
///
/// AGC source: Comanche055/TVCDAPS.agc
///   PINTEGRL (body pitch rate error integration, p. 963),
///   PERORLIM → ERRORLIM (input limiter, p. 971),
///   PFORWARD → FWDFLTR → OPTVARK (filter + gain, pp. 964,972-973),
///   POFFSET (trim correction, p. 964),
///   PACLIM → ACTLIM (output saturation, pp. 964,971-972),
///   YINTEGRL ... YACLIM (identical for yaw, pp. 967-968).
///
/// Units:
///   `error.pitch` / `error.yaw` — radians (body frame)
///   `rate.pitch`  / `rate.yaw`  — radians/second (body frame)
///   `gains.k_p`                 — dimensionless
///   `gains.k_d`                 — seconds
///   `gains.limit_deg`           — degrees (converted internally)
///   Return: (pitch_rad, yaw_rad) — radians, saturated at ±ACTSAT_RAD
pub fn steer(error: &AttitudeError, rate: &AttitudeError, gains: &TvcGains) -> (f64, f64) {
    debug_assert!(gains.limit_deg > 0.0, "limit_deg must be positive");

    // Guard NaN inputs
    let safe = |x: f64| if x.is_finite() { x } else { 0.0 };
    let ep = safe(error.pitch);
    let ey = safe(error.yaw);
    let rp = safe(rate.pitch);
    let ry = safe(rate.yaw);

    let limit_deg = if gains.limit_deg > 0.0 {
        gains.limit_deg
    } else {
        ACTSAT_DEG
    };

    // PD law: raw = k_p * error + k_d * rate
    // AGC: PFORWARD → FWDFLTR → OPTVARK (simplified here per ADR-001)
    let raw_pitch = gains.k_p * ep + gains.k_d * rp;
    let raw_yaw = gains.k_p * ey + gains.k_d * ry;

    // ACTLIM: saturate at ±limit_deg (converted to radians)
    // AGC source: TVCDAPS.agc PACLIM → ACTLIM (pp. 964, 971-972)
    let limit_rad = limit_deg * core::f64::consts::PI / 180.0;
    let pitch_cmd = raw_pitch.clamp(-limit_rad, limit_rad);
    let yaw_cmd = raw_yaw.clamp(-limit_rad, limit_rad);

    (pitch_cmd, yaw_cmd)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-TVC-1: Zero error → zero command.
    #[test]
    fn zero_error_zero_command() {
        let (p, y) = steer(
            &AttitudeError::ZERO,
            &AttitudeError::ZERO,
            &TvcGains::NOMINAL,
        );
        assert_eq!(p, 0.0, "pitch_cmd should be 0");
        assert_eq!(y, 0.0, "yaw_cmd should be 0");
    }

    /// TC-TVC-2: Positive pitch error → positive pitch gimbal (unsaturated).
    ///
    /// error.pitch = 0.05 rad ≈ 2.87°; k_p=1.0, k_d=0.0; limit_deg=6.0.
    /// Expected: pitch_cmd = 0.05 rad (unsaturated, within ±ACTSAT_RAD).
    #[test]
    fn positive_pitch_error_positive_command() {
        let error = AttitudeError {
            pitch: 0.05,
            yaw: 0.0,
            roll: 0.0,
        };
        let gains = TvcGains {
            k_p: 1.0,
            k_d: 0.0,
            limit_deg: ACTSAT_DEG,
        };
        let (p, y) = steer(&error, &AttitudeError::ZERO, &gains);
        assert!((p - 0.05).abs() < 1e-12, "pitch_cmd = {p} expected 0.05");
        assert_eq!(y, 0.0, "yaw should be 0");
    }

    /// TC-TVC-3: Saturation at limit for large error.
    ///
    /// error.pitch = 0.5 rad ≈ 28.6° >> 6°; k_p=1.0.
    /// Expected: |pitch_cmd| == ACTSAT_RAD.
    #[test]
    fn saturation_at_actsat() {
        let error = AttitudeError {
            pitch: 0.5,
            yaw: 0.0,
            roll: 0.0,
        };
        let gains = TvcGains {
            k_p: 1.0,
            k_d: 0.0,
            limit_deg: ACTSAT_DEG,
        };
        let (p, _y) = steer(&error, &AttitudeError::ZERO, &gains);
        assert!(
            (p.abs() - ACTSAT_RAD).abs() < 1e-10,
            "pitch_cmd = {p} expected ±ACTSAT_RAD = {ACTSAT_RAD}"
        );
        assert!(p > 0.0, "positive error must give positive command");
    }

    /// TC-TVC-4: Gain scheduling — k_p=2 gives twice the output of k_p=1.
    #[test]
    fn gain_doubles_response() {
        let error = AttitudeError {
            pitch: 0.02,
            yaw: 0.0,
            roll: 0.0,
        };
        let gains1 = TvcGains {
            k_p: 1.0,
            k_d: 0.0,
            limit_deg: ACTSAT_DEG,
        };
        let gains2 = TvcGains {
            k_p: 2.0,
            k_d: 0.0,
            limit_deg: ACTSAT_DEG,
        };
        let (p1, _) = steer(&error, &AttitudeError::ZERO, &gains1);
        let (p2, _) = steer(&error, &AttitudeError::ZERO, &gains2);
        assert!(
            (p2 - 2.0 * p1).abs() < 1e-14,
            "k_p=2 result {p2} should be 2×k_p=1 result {p1}"
        );
    }

    /// TC-TVC-5: ACTSAT_RAD constant is correct (6° in radians).
    #[test]
    fn actsat_rad_is_six_degrees() {
        let six_deg_rad = 6.0_f64 * core::f64::consts::PI / 180.0;
        assert!(
            (ACTSAT_RAD - six_deg_rad).abs() < 1e-14,
            "ACTSAT_RAD = {ACTSAT_RAD} expected {six_deg_rad}"
        );
    }
}
