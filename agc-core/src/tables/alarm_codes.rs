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
