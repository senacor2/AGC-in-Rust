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
// p20/p21/p23 still define local duplicates of FRAME_MISMATCH and NO_CSM_SV
// pending a follow-up sweep — see PR #114 description.

/// CSM state-vector frame is not a navigation frame (only `StableMember`
/// triggers this; both ECI and MCI are valid for landmark tracking).
pub const FRAME_MISMATCH: u16 = 0o00400;
/// No valid CSM state vector (epoch == 0). Raised at P22 init.
pub const NO_CSM_SV: u16 = 0o01420;
/// P22 W-matrix diagonal went negative (loss of positive definiteness).
pub const CSM_W_OVERFLOW: u16 = 0o01421;
/// Five consecutive landmark marks rejected by the 3-sigma gate (P22).
pub const LANDMARK_REJECT: u16 = 0o01422;
/// Landmark index out of range (0 or > 8) supplied to P22.
pub const BAD_LANDMARK_INDEX: u16 = 0o01424;
/// CSM-to-landmark slant range below the safety floor (P22).
pub const LANDMARK_RANGE_ZERO: u16 = 0o01425;

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
