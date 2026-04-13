//! Navigation constants used by gravity models and the SERVICER integrator.
//!
//! All values are cited from `docs/agc-reference-constants.md` which derives
//! them from `Comanche055/ORBITAL_INTEGRATION.agc` and `SERVICER207.agc`.
//!
//! AGC source: Comanche055/ORBITAL_INTEGRATION.agc (MUEARTH, MUM, RSPHERE, J2REQSQ)
//!             Comanche055/SERVICER207.agc           (KPIP1, -MAXDELV, CAF 2SECS)
//!             Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc (ERAD)

/// Earth gravitational parameter (GM), m³/s².
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc` `MUEARTH 2DEC* 3.986032 E10 B-36*`.
/// Value from `docs/agc-reference-constants.md`.
pub const MU_EARTH: f64 = 3.986_032e14;

/// Moon gravitational parameter (GM), m³/s².
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc` `MUM = MUEARTH -2`
///             `2DEC* 4.9027780 E8 B-30*`.
/// Value from `docs/agc-reference-constants.md`.
pub const MU_MOON: f64 = 4.902_778e12;

/// Earth equatorial radius, metres.
///
/// AGC source: `Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc`
///             `ERAD 2DEC 6373338 B-29 # PAD RADIUS`.
/// Value from `docs/agc-reference-constants.md`.
pub const RE_EARTH: f64 = 6_373_338.0;

/// Earth J2 oblateness coefficient (dimensionless).
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc` `J2REQSQ 2DEC* 1.75501139 E21 B-72*`.
///             The J2 value is extracted from the J2*RE^2 product context.
/// Value from `docs/agc-reference-constants.md`.
pub const J2_EARTH: f64 = 1.082_626_68e-3;

/// Sphere of influence radius (Earth-Moon boundary), metres.
///
/// When the vehicle's ECI distance from Earth exceeds RSPHERE, the primary
/// gravitational body switches to the Moon.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc` `RSPHERE 2DEC 64373.76 E3 B-29`
///             = 64 373 760 m = 64 373.76 km.
pub const RSPHERE: f64 = 64_373_760.0;

/// Singularity guard: minimum |r| below which gravity returns zero, metres.
///
/// Prevents division-by-zero in the point-mass gravity formula.
/// Value from `docs/agc-reference-constants.md` point-mass guard.
pub const R_MIN_GUARD: f64 = 1.0;

/// SERVICER cycle period, seconds.
///
/// AGC source: `Comanche055/SERVICER207.agc` `CAF 2SECS` — Waitlist reschedule delay.
pub const CYCLE_DT: f64 = 2.0;

/// SERVICER cycle period, centiseconds (200 cs = 2 s).
///
/// AGC source: `Comanche055/SERVICER207.agc` `2SEC(22) = 200 B-22 cs`.
pub const CYCLE_DT_CS: u32 = 200;

/// PIPA scale factor: metres per second per count.
///
/// AGC source: `Comanche055/SERVICER207.agc` `KPIP1 2DEC 0.074880`.
/// Converting: 0.074880 (in B-7 m/cs) × 2^-7 × 100 cs/s = 0.05850 m/s/count.
/// Comment in source: `# 1 PULSE = 5.85 CM/SEC`.
pub const KPIP1: f64 = 0.0585;

/// PIPA saturation limit (absolute value), pulse counts per 2-second cycle.
///
/// If |count| >= PIPA_MAX_COUNTS on any axis, the SERVICER raises alarm 00205
/// and skips the CALCRVG integration step.
///
/// AGC source: `Comanche055/SERVICER207.agc` `-MAXDELV DEC -6398`.
pub const PIPA_MAX_COUNTS: i16 = 6398;
