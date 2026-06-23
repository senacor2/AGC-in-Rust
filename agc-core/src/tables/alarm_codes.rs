//! Program alarm code definitions.

/// Executive overflow — no free job slots (the Apollo 11 "1202" alarm).
pub const EXEC_OVERFLOW: u16 = 1202;
/// Executive overflow — no free VAC areas. Unused: the interpretive language
/// is eliminated in this port (ADR-001), so no VAC pool exists and FINDVAC
/// has no Rust equivalent. Retained for reference only.
#[allow(dead_code)]
pub const NO_VAC: u16 = 1210;
/// Waitlist overflow — no free task slots.
pub const WAITLIST_OVERFLOW: u16 = 1211;
/// IMU not aligned (REFSMMAT invalid).
pub const IMU_NOT_ALIGNED: u16 = 0o210;
/// Celestial body too close to Sun for optical sighting.
pub const BODY_TOO_CLOSE_TO_SUN: u16 = 0o206;
/// Optics error — mark rejected. Raised by P22/P23 when a sighting fails
/// the editing test (sigma > limit).
#[allow(dead_code)]
pub const OPTICS_MARK_REJECTED: u16 = 0o220;
/// Navigation integration failed to converge.
pub const NAV_NO_CONVERGE: u16 = 0o401;
/// Invalid orbit (sub-parabolic or degenerate conic).
pub const INVALID_ORBIT: u16 = 0o404;
/// Uplink too fast — ground sent another keystroke while the V/N
/// processor was still in `OprErr` (or, on hardware, the UPRUPT FIFO
/// overran). Mission Control is expected to send RSET before retrying.
/// AGC convention: octal 1106.
pub const UPLINK_TOO_FAST: u16 = 0o1106;

// ── P22 (Orbital Navigation / Landmark Tracking) ──────────────────────────────
//
// AGC source: alarms raised by the P22 measurement-incorporation pipeline.
// These three are also reused by P20/P21/P23 (the #114/#115 sweep removed the
// per-program duplicate `const`s that previously shadowed them).

/// CSM state-vector frame is not a navigation frame (only `StableMember`
/// triggers this; both ECI and MCI are valid for landmark tracking).
/// Reused by P20 and P23.
pub const FRAME_MISMATCH: u16 = 0o00400;
/// No valid CSM state vector (epoch == 0). Raised at P22 init. Reused by
/// P21 and P23.
pub const NO_CSM_SV: u16 = 0o01420;
/// CSM W-matrix diagonal went negative (loss of positive definiteness).
/// Reused by P20 and P23 for their rendezvous/cislunar W-matrix guard.
pub const CSM_W_OVERFLOW: u16 = 0o01421;
/// Five consecutive landmark marks rejected by the 3-sigma gate (P22).
pub const LANDMARK_REJECT: u16 = 0o01422;
/// Landmark index out of range (0 or > 8) supplied to P22.
pub const BAD_LANDMARK_INDEX: u16 = 0o01424;
/// CSM-to-landmark slant range below the safety floor (P22).
pub const LANDMARK_RANGE_ZERO: u16 = 0o01425;

// ── Per-program alarm codes ───────────────────────────────────────────────────
//
// Centralised here per the architecture doc (§ "Constant tables") and issue
// #115. Names are program-scoped so codes that share a value across programs
// stay distinct symbols. NOTE: a few octal values are intentionally reused
// across unrelated programs (e.g. 0o01430–0o01432 appear in both P23 and P29);
// these are pre-existing collisions preserved by the centralisation sweep, not
// introduced by it.

// P01/P02 — gyrocompass alignment.
/// P02 invoked from a gyrocompass state that forbids it.
pub const ALARM_GYROCOMPASS_WRONG_STATE: u16 = 235;

// P11 — earth-orbit insertion monitor.
/// P11 state vector is hyperbolic (e ≥ 1).
pub const ALARM_P11_HYPERBOLIC_ORBIT: u16 = 229;
/// P11 entered with the CSM state vector in the wrong frame.
pub const ALARM_P11_WRONG_FRAME: u16 = 230;

// P15 — TLI / trans-lunar injection targeting.
/// P15 entered with the CSM state vector in the wrong frame.
pub const ALARM_P15_WRONG_FRAME: u16 = 236;
/// P15 trajectory solution is hyperbolic.
pub const ALARM_P15_HYPERBOLIC: u16 = 237;

// P20 — rendezvous navigation.
/// P20 radar mark requested but no tracking source available.
pub const ALARM_P20_NO_RADAR: u16 = 0o00404;
/// P20 mark rejected by the editing gate and the crew override expired.
pub const ALARM_P20_REJECT_OVERRIDE: u16 = 0o00405;

// P23 — cislunar star/horizon navigation.
/// P23 lost star lock during a sighting.
pub const ALARM_P23_NO_STAR_LOCK: u16 = 0o01426;
/// P23 measured a geometrically invalid star/horizon angle.
pub const ALARM_P23_BAD_ANGLE: u16 = 0o01427;
/// P23 sighting body is too close to the line of sight.
pub const ALARM_P23_TOO_CLOSE_TO_BODY: u16 = 0o01430;
/// P23 mark rejected by the editing gate and the crew override expired.
pub const ALARM_P23_REJECT_OVERRIDE: u16 = 0o01431;
/// P23 star/horizon slant range below the safety floor.
pub const ALARM_P23_LANDMARK_RANGE_ZERO: u16 = 0o01432;

// P29 — geodetic-target time-of-event solver.
/// P29 has no valid CSM state vector.
pub const ALARM_P29_NO_CSM_SV: u16 = 0o01430;
/// P29 orbit is hyperbolic.
pub const ALARM_P29_HYPERBOLIC: u16 = 0o01431;
/// P29 solver failed to converge.
pub const ALARM_P29_NO_CONV: u16 = 0o01432;

// P30 — external-ΔV targeting.
/// P30 TIG is in the past.
pub const ALARM_P30_TIG_IN_PAST: u16 = 210;

// P31 — rendezvous-final targeting.
/// P31 has no valid target.
pub const ALARM_P31_NO_TARGET: u16 = 0o01434;
/// P31 targeting failed to converge.
pub const ALARM_P31_NOT_CONVERGED: u16 = 0o01435;

// P32 — coelliptic-sequence targeting.
/// P32 has no valid target.
pub const ALARM_P32_NO_TARGET: u16 = 0o01436;
/// P32 geometry is degenerate.
pub const ALARM_P32_DEGENERATE: u16 = 0o01437;

// P33 — constant-differential-height targeting.
/// P33 has no valid target.
pub const ALARM_P33_NO_TARGET: u16 = 0o01440;
/// P33 has no staged TIG.
pub const ALARM_P33_NO_TIG: u16 = 0o01441;
/// P33 target is stale.
pub const ALARM_P33_STALE_TARGET: u16 = 0o01442;
/// P33 geometry is degenerate.
pub const ALARM_P33_DEGENERATE: u16 = 0o01443;
/// P33 Lambert solver failed.
pub const ALARM_P33_LAMBERT: u16 = 0o01444;

// P34 — transfer-phase-initiation targeting (shares P33's solver alarms).
/// P34 target is closer than the safety floor.
pub const ALARM_P34_TOO_CLOSE: u16 = 0o01445;

// P37 — return-to-earth targeting.
/// P37 time-of-flight is out of range.
pub const ALARM_P37_BAD_TOF: u16 = 1410;
/// P37 entered with the state vector in the wrong frame.
pub const ALARM_P37_WRONG_FRAME: u16 = 1411;

// P40/P41 — SPS / RCS powered-flight.
/// P40/P41 armed with no pending maneuver.
pub const ALARM_P40_NO_PENDING_MANEUVER: u16 = 224;
/// P40/P41 TIG is in the past.
pub const ALARM_P40_TIG_IN_PAST: u16 = 225;
/// P40/P41 ΔV is below the minimum burn threshold.
pub const ALARM_P40_DV_TOO_SMALL: u16 = 226;
/// P40 burn too small for the SPS regime.
pub const ALARM_P40_WRONG_REGIME: u16 = 227;
/// P41 burn too large for the RCS regime.
pub const ALARM_P41_WRONG_REGIME: u16 = 228;

// P51/P52 — IMU alignment.
/// Two selected stars are collinear; TRIAD cannot build a basis.
pub const ALARM_COLLINEAR_STARS: u16 = 220;
/// P52 invoked while the platform is still caged.
pub const ALARM_PLATFORM_CAGED: u16 = 221;

// P61–P67 — entry guidance.
/// P62 entered in the wrong phase.
pub const ALARM_P62_WRONG_PHASE: u16 = 231;
/// P63 entered in the wrong phase.
pub const ALARM_P63_WRONG_PHASE: u16 = 232;
/// P64 entered before its phase was reached.
pub const ALARM_P64_EARLY: u16 = 233;
/// P67 entered in the wrong phase.
pub const ALARM_P67_WRONG_PHASE: u16 = 234;

// V/N processor.
/// V25 N81 (ΔV load) entered without a prior TIG load.
pub const ALARM_DV_LOAD_WITHOUT_TIG: u16 = 240;

// ── Call-site tags (ADRES field on AlarmState) ────────────────────────────────
//
// Each module/program that raises an alarm is assigned a small octal tag.
// The tag is passed to `AlarmState::raise(code, adres)` and displayed by
// V05N08 R1 so the crew (and Mission Control) can identify *where* the
// alarm fired in addition to *what* code was raised.
//
// AGC erasable correspondence: ADRES (the ADRES field of the alarm-raise
// macro in `ALARM_AND_ABORT.agc`). The real AGC stored a 12-bit BBANK+ECADR
// here; we use a synthetic small integer scheme because the Rust port has
// no bank addressing.

pub const SITE_NONE: u16 = 0o0;
pub const SITE_EXECUTIVE: u16 = 0o1;
pub const SITE_WAITLIST: u16 = 0o2;
pub const SITE_UPLINK: u16 = 0o3;
pub const SITE_AVG_G: u16 = 0o4;
pub const SITE_FRESH_START: u16 = 0o5;
pub const SITE_POODOO: u16 = 0o6;

pub const SITE_P01_P02: u16 = 0o11;
pub const SITE_P11: u16 = 0o12;
pub const SITE_P15: u16 = 0o13;
pub const SITE_P20: u16 = 0o14;
pub const SITE_P21: u16 = 0o15;
pub const SITE_P22: u16 = 0o16;
pub const SITE_P23: u16 = 0o17;
pub const SITE_P29: u16 = 0o20;
pub const SITE_P30: u16 = 0o21;
pub const SITE_P31: u16 = 0o22;
pub const SITE_P32: u16 = 0o23;
pub const SITE_P33: u16 = 0o24;
pub const SITE_P34: u16 = 0o25;
pub const SITE_P37: u16 = 0o26;
pub const SITE_P40_P41: u16 = 0o27;
pub const SITE_P51_P52: u16 = 0o30;
pub const SITE_P61_P67: u16 = 0o31;
pub const SITE_VN: u16 = 0o32;
