//! Control-subsystem constants shared across DAP, RCS, TVC, and attitude modules.
//!
//! All constants are cited from the AGC source files listed below.

use crate::types::Met;

/// T5RUPT period for the RCS DAP, centiseconds.
///
/// Phase 1 (RCSATT rate filter) = 20 ms + Phase 2 (T5PHASE2 jet select) = 80 ms = 100 ms total.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
///   `DELTATT  = OCT 37770`  (Phase 2: 80 ms = −8 in TIME5 units of 10 ms)
///   `DELTATT2 = OCT 37776`  (Phase 1: 20 ms = −2 in TIME5 units of 10 ms)
///   Comment in REDAP (line 578):
///   `"PHASE 1 (RATEFILTER) BEGINS CYCLING 100 MS FROM NOW AND EVERY 100MS THEREAFTER"`.
///
/// 10 centiseconds = 100 ms.
pub const T5RUPT_PERIOD_CS: u32 = 10;

/// T5RUPT period as a `Met` (centiseconds).
///
/// Same as `T5RUPT_PERIOD_CS` but wrapped in the `Met` newtype for use
/// with timer scheduling.
///
/// AGC source: same as `T5RUPT_PERIOD_CS`.
pub const T5RUPT_PERIOD: Met = Met::from_centiseconds(T5RUPT_PERIOD_CS);

/// Default RCS attitude-hold deadband, radians.
///
/// Corresponds to approximately 0.3° (6 mrad). The exact value decoded from
/// `DBTABLE` by `S41.2` is crew-selectable; this constant represents the
/// minimum attitude-hold deadband per the AGC program description.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc `S41.2` call at REDAP (line 551).
///             Per O'Brien pp. 312–334: default attitude-hold deadband ≈ 0.3°.
///
/// TODO(M4): decode DBTABLE from S41.2 when that source is available.
pub const DEADBAND_DEFAULT_RAD: f64 = 0.005_236; // ≈ 0.3° in radians

/// Phase-plane switching surface slope.
///
/// The linear switching variable is `s = error + SLOPE * rate`.  SLOPE = 0.24
/// corresponds to 0.6 rad/s gain (the comment in the AGC source reads "SLOPE = 0.6/SEC").
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc `REDAP` initialisation
///   (line 573): `CAF =.24 / TS SLOPE`; label REDAP (page 1010).
/// Referenced also in Comanche055/TVCROLLDAP.agc ROLLOGIC switching line (pp. 987–988).
pub const SLOPE: f64 = 0.24;

/// TVC actuator command saturation limit, radians.
///
/// 6° × π/180 = 0.104_719_755 rad. The AGC stores this as 253 ASCREV counts.
///
/// AGC source: Comanche055/TVCDAPS.agc p. 978 constants block:
///   `ACTSAT DEC 253  # ACTUATOR LIMIT (6 DEG), SC.AT 1ASCREV`.
///   253 × 85.41 arcsec/ASCREV = 21 609 arcsec ≈ 6.002°.
pub const TVC_ACTSAT_RAD: f64 = 0.104_719_755_119_659_77; // 6° in radians

/// TVC roll DAP deadband, radians.
///
/// 5° in radians. Used for the roll-axis phase-plane decision in TVC mode.
///
/// AGC source: Comanche055/TVCROLLDAP.agc functional description p. 984:
///   "MAINTAIN OGA WITHIN 5 DEG DEADBND OF OGAD".
pub const TVC_ROLL_DEADBAND_RAD: f64 = 5.0 * core::f64::consts::PI / 180.0;

/// Cross-product steering gain K_PRIME (dimensionless).
///
/// Scales the angular rate command produced by the cross-product steering law:
///   `omega_cmd = K_PRIME * (unit(delta_v) × unit(VG))`
///
/// Starting value is 1.0; tuning for M4+.
///
/// AGC source: Comanche055/P40-P47.agc `XPRODUCT` routine (S40.8, page 721-722);
///             constant `KPRIMEDT` on page 744.
pub const K_PRIME: f64 = 1.0;
