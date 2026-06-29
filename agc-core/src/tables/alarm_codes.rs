// SPDX-License-Identifier: GPL-3.0-or-later
//! Program alarm code definitions.
//!
//! Alarm codes are 5-octal-digit values displayed on the DSKY PROG register
//! (V05N09 R1..R3 shows the 3-deep FIFO). Codes fall into two disjoint sets:
//!
//! * **Faithful AGC codes** — values lifted directly from the Apollo Guidance
//!   Computer Quick Reference (Command Module Program Alarms). The meaning in
//!   this re-implementation matches the historical meaning.
//! * **Synthetic codes** — alarms for conditions specific to this Rust port
//!   that do not have an AGC equivalent. All synthetic codes live in the
//!   reserved range `0o70000..=0o77777`, comfortably above the highest
//!   official code (`0o31211`) and inside `u16`. A leading "7…" on the DSKY
//!   instantly identifies a synthetic alarm.
//!
//! The split is enforced by `alarm_code_audit` tests at the bottom of this
//! file (synthetic ∩ official = ∅, all faithful codes appear in the official
//! list, no two named constants share a value).

// ── §1. Faithful codes (real AGC alarm codes, keep) ───────────────────────────
//
// Each of these matches the Quick Reference both in value and in meaning.

/// Executive overflow — no core sets (the famous Apollo 11 "1202" alarm).
/// AGC code: `31202`.
pub const EXEC_OVERFLOW: u16 = 0o31202;

/// Executive overflow — no free VAC areas. Unused: the interpretive language
/// is eliminated in this port (ADR-001), so no VAC pool exists and FINDVAC
/// has no Rust equivalent. Retained for reference only. AGC code: `31201`.
#[allow(dead_code)]
pub const NO_VAC: u16 = 0o31201;

/// Waitlist overflow — too many tasks. AGC code: `31203`.
pub const WAITLIST_OVERFLOW: u16 = 0o31203;

/// IMU not aligned (REFSMMAT). AGC code: `00220` (R02 / P51).
pub const IMU_NOT_ALIGNED: u16 = 0o00220;

/// Uplink too fast — ground sent another keystroke while the V/N processor
/// was still in `OprErr` (or, on hardware, the UPRUPT FIFO overran).
/// Mission Control is expected to send RSET before retrying. AGC code: `01106`.
pub const UPLINK_TOO_FAST: u16 = 0o01106;

// ── §2. Synthetic codes (project-invented, range 0o70000..=0o77777) ───────────
//
// Layout:
//   0o70xxx — cross-program / shared utilities
//   0o71xxx — P01..P15
//   0o72xxx — P20..P29
//   0o73xxx — P30..P37
//   0o74xxx — P40..P41
//   0o75xxx — P51..P52
//   0o76xxx — P61..P67
//   0o77xxx — V/N processor

// 0o70xxx — cross-program / shared.

/// Optical sighting refused because the target body lies too close to the Sun.
pub const BODY_TOO_CLOSE_TO_SUN: u16 = 0o70001;

/// Optics error — mark rejected. Raised by P22/P23 when a sighting fails
/// the editing test (sigma > limit).
#[allow(dead_code)]
pub const OPTICS_MARK_REJECTED: u16 = 0o70002;

/// Navigation integration failed to converge.
pub const NAV_NO_CONVERGE: u16 = 0o70003;

/// Invalid orbit (sub-parabolic or degenerate conic).
pub const INVALID_ORBIT: u16 = 0o70004;

/// CSM state-vector frame is not a navigation frame (only `StableMember`
/// triggers this; both ECI and MCI are valid for landmark tracking).
/// Shared by P20, P22, P23.
pub const FRAME_MISMATCH: u16 = 0o70005;

/// No valid CSM state vector (epoch == 0). Shared by P21, P22, P23.
pub const NO_CSM_SV: u16 = 0o70006;

/// CSM W-matrix diagonal went negative (loss of positive definiteness).
/// Shared by P20, P22, P23.
pub const CSM_W_OVERFLOW: u16 = 0o70007;

// 0o71xxx — P01..P15.

/// P02 invoked from a gyrocompass state that forbids it.
pub const ALARM_GYROCOMPASS_WRONG_STATE: u16 = 0o71001;
/// P11 state vector is hyperbolic (e ≥ 1).
pub const ALARM_P11_HYPERBOLIC_ORBIT: u16 = 0o71002;
/// P11 entered with the CSM state vector in the wrong frame.
pub const ALARM_P11_WRONG_FRAME: u16 = 0o71003;
/// P15 entered with the CSM state vector in the wrong frame.
pub const ALARM_P15_WRONG_FRAME: u16 = 0o71004;
/// P15 trajectory solution is hyperbolic.
pub const ALARM_P15_HYPERBOLIC: u16 = 0o71005;

// 0o72xxx — P20..P29.

/// P20 radar mark requested but no tracking source available.
pub const ALARM_P20_NO_RADAR: u16 = 0o72001;
/// P20 mark rejected by the editing gate and the crew override expired.
pub const ALARM_P20_REJECT_OVERRIDE: u16 = 0o72002;
/// Five consecutive landmark marks rejected by the 3-sigma gate (P22).
pub const LANDMARK_REJECT: u16 = 0o72003;
/// Landmark index out of range (0 or > 8) supplied to P22.
pub const BAD_LANDMARK_INDEX: u16 = 0o72004;
/// CSM-to-landmark slant range below the safety floor (P22).
pub const LANDMARK_RANGE_ZERO: u16 = 0o72005;
/// P23 lost star lock during a sighting.
pub const ALARM_P23_NO_STAR_LOCK: u16 = 0o72006;
/// P23 measured a geometrically invalid star/horizon angle.
pub const ALARM_P23_BAD_ANGLE: u16 = 0o72007;
/// P23 sighting body is too close to the line of sight.
pub const ALARM_P23_TOO_CLOSE_TO_BODY: u16 = 0o72010;
/// P23 mark rejected by the editing gate and the crew override expired.
pub const ALARM_P23_REJECT_OVERRIDE: u16 = 0o72011;
/// P23 star/horizon slant range below the safety floor.
pub const ALARM_P23_LANDMARK_RANGE_ZERO: u16 = 0o72012;
/// P29 has no valid CSM state vector.
pub const ALARM_P29_NO_CSM_SV: u16 = 0o72013;
/// P29 orbit is hyperbolic.
pub const ALARM_P29_HYPERBOLIC: u16 = 0o72014;
/// P29 solver failed to converge.
pub const ALARM_P29_NO_CONV: u16 = 0o72015;

// 0o73xxx — P30..P37.

/// P30 TIG is in the past.
pub const ALARM_P30_TIG_IN_PAST: u16 = 0o73001;
/// P31 has no valid target.
pub const ALARM_P31_NO_TARGET: u16 = 0o73002;
/// P31 targeting failed to converge.
pub const ALARM_P31_NOT_CONVERGED: u16 = 0o73003;
/// P32 has no valid target.
pub const ALARM_P32_NO_TARGET: u16 = 0o73004;
/// P32 geometry is degenerate.
pub const ALARM_P32_DEGENERATE: u16 = 0o73005;
/// P33 has no valid target.
pub const ALARM_P33_NO_TARGET: u16 = 0o73006;
/// P33 has no staged TIG.
pub const ALARM_P33_NO_TIG: u16 = 0o73007;
/// P33 target is stale.
pub const ALARM_P33_STALE_TARGET: u16 = 0o73010;
/// P33 geometry is degenerate.
pub const ALARM_P33_DEGENERATE: u16 = 0o73011;
/// P33 Lambert solver failed.
pub const ALARM_P33_LAMBERT: u16 = 0o73012;
/// P34 target is closer than the safety floor.
pub const ALARM_P34_TOO_CLOSE: u16 = 0o73013;
/// P37 time-of-flight is out of range.
pub const ALARM_P37_BAD_TOF: u16 = 0o73014;
/// P37 entered with the state vector in the wrong frame.
pub const ALARM_P37_WRONG_FRAME: u16 = 0o73015;

// 0o74xxx — P40/P41.

/// P40/P41 armed with no pending maneuver.
pub const ALARM_P40_NO_PENDING_MANEUVER: u16 = 0o74001;
/// P40/P41 TIG is in the past.
pub const ALARM_P40_TIG_IN_PAST: u16 = 0o74002;
/// P40/P41 ΔV is below the minimum burn threshold.
pub const ALARM_P40_DV_TOO_SMALL: u16 = 0o74003;
/// P40 burn too small for the SPS regime.
pub const ALARM_P40_WRONG_REGIME: u16 = 0o74004;
/// P41 burn too large for the RCS regime.
pub const ALARM_P41_WRONG_REGIME: u16 = 0o74005;

// 0o75xxx — P51/P52.

/// Two selected stars are collinear; TRIAD cannot build a basis.
pub const ALARM_COLLINEAR_STARS: u16 = 0o75001;
/// P52 invoked while the platform is still caged.
pub const ALARM_PLATFORM_CAGED: u16 = 0o75002;

// 0o76xxx — P61..P67.

/// P62 entered in the wrong phase.
pub const ALARM_P62_WRONG_PHASE: u16 = 0o76001;
/// P63 entered in the wrong phase.
pub const ALARM_P63_WRONG_PHASE: u16 = 0o76002;
/// P64 entered before its phase was reached.
pub const ALARM_P64_EARLY: u16 = 0o76003;
/// P67 entered in the wrong phase.
pub const ALARM_P67_WRONG_PHASE: u16 = 0o76004;

// 0o77xxx — V/N processor.

/// V25 N81 (ΔV load) entered without a prior TIG load.
pub const ALARM_DV_LOAD_WITHOUT_TIG: u16 = 0o77001;

// ── §3. Call-site tags (ADRES field on AlarmState) ────────────────────────────
//
// Each module/program that raises an alarm is assigned a small octal tag.
// The tag is passed to `AlarmState::raise(code, adres)` and displayed by
// V05N08 R1 so the crew (and Mission Control) can identify *where* the
// alarm fired in addition to *what* code was raised.
//
// AGC erasable correspondence: ADRES (the ADRES field of the alarm-raise
// macro in `ALARM_AND_ABORT.agc`). The real AGC stored a 12-bit BBANK+ECADR
// here; we use a synthetic small integer scheme because the Rust port has
// no bank addressing. Call-site tags share the namespace with alarm codes
// only via the display register; they are not themselves alarm codes and
// are not range-restricted.

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

// ── §4. Audit (guard tests for #182) ──────────────────────────────────────────

#[cfg(test)]
mod alarm_code_audit {
    use super::*;

    /// Reserved synthetic-alarm range.
    const SYNTHETIC_RANGE: std::ops::RangeInclusive<u16> = 0o70000..=0o77777;

    /// All officially-defined AGC alarm codes from
    /// `input/AGC Quick Reference.md` → "Command Module Program Alarms".
    /// Kept sorted by value. When the Quick Reference is updated, mirror
    /// the change here.
    const OFFICIAL_AGC_CODES: &[u16] = &[
        0o00110, 0o00113, 0o00114, 0o00115, 0o00116, 0o00117, 0o00120, 0o00121,
        0o00205, 0o00206, 0o00207, 0o00210, 0o00211, 0o00212, 0o00213, 0o00214,
        0o00217, 0o00220, 0o00401, 0o00402, 0o00404, 0o00405, 0o00406, 0o00421,
        0o00600, 0o00601, 0o00602, 0o00603, 0o00604, 0o00605, 0o00606, 0o00611,
        0o00612, 0o00613, 0o00777, 0o01102, 0o01105, 0o01106, 0o01107, 0o01301,
        0o01407, 0o01426, 0o01427, 0o01520, 0o01600, 0o01601, 0o01703, 0o03777,
        0o04777, 0o07777, 0o10777, 0o13777, 0o14777, 0o20430, 0o20607, 0o20610,
        0o21204, 0o21206, 0o21210, 0o21302, 0o21501, 0o21502, 0o21521, 0o31104,
        0o31201, 0o31202, 0o31203, 0o31211,
    ];

    /// Codes this re-implementation keeps in real-AGC space because the
    /// meaning is faithful to the Quick Reference.
    const FAITHFUL_WHITELIST: &[(&str, u16)] = &[
        ("EXEC_OVERFLOW", EXEC_OVERFLOW),
        ("NO_VAC", NO_VAC),
        ("WAITLIST_OVERFLOW", WAITLIST_OVERFLOW),
        ("IMU_NOT_ALIGNED", IMU_NOT_ALIGNED),
        ("UPLINK_TOO_FAST", UPLINK_TOO_FAST),
    ];

    /// Every project-invented (synthetic) alarm constant.
    const SYNTHETIC_CODES: &[(&str, u16)] = &[
        ("BODY_TOO_CLOSE_TO_SUN", BODY_TOO_CLOSE_TO_SUN),
        ("OPTICS_MARK_REJECTED", OPTICS_MARK_REJECTED),
        ("NAV_NO_CONVERGE", NAV_NO_CONVERGE),
        ("INVALID_ORBIT", INVALID_ORBIT),
        ("FRAME_MISMATCH", FRAME_MISMATCH),
        ("NO_CSM_SV", NO_CSM_SV),
        ("CSM_W_OVERFLOW", CSM_W_OVERFLOW),
        ("ALARM_GYROCOMPASS_WRONG_STATE", ALARM_GYROCOMPASS_WRONG_STATE),
        ("ALARM_P11_HYPERBOLIC_ORBIT", ALARM_P11_HYPERBOLIC_ORBIT),
        ("ALARM_P11_WRONG_FRAME", ALARM_P11_WRONG_FRAME),
        ("ALARM_P15_WRONG_FRAME", ALARM_P15_WRONG_FRAME),
        ("ALARM_P15_HYPERBOLIC", ALARM_P15_HYPERBOLIC),
        ("ALARM_P20_NO_RADAR", ALARM_P20_NO_RADAR),
        ("ALARM_P20_REJECT_OVERRIDE", ALARM_P20_REJECT_OVERRIDE),
        ("LANDMARK_REJECT", LANDMARK_REJECT),
        ("BAD_LANDMARK_INDEX", BAD_LANDMARK_INDEX),
        ("LANDMARK_RANGE_ZERO", LANDMARK_RANGE_ZERO),
        ("ALARM_P23_NO_STAR_LOCK", ALARM_P23_NO_STAR_LOCK),
        ("ALARM_P23_BAD_ANGLE", ALARM_P23_BAD_ANGLE),
        ("ALARM_P23_TOO_CLOSE_TO_BODY", ALARM_P23_TOO_CLOSE_TO_BODY),
        ("ALARM_P23_REJECT_OVERRIDE", ALARM_P23_REJECT_OVERRIDE),
        ("ALARM_P23_LANDMARK_RANGE_ZERO", ALARM_P23_LANDMARK_RANGE_ZERO),
        ("ALARM_P29_NO_CSM_SV", ALARM_P29_NO_CSM_SV),
        ("ALARM_P29_HYPERBOLIC", ALARM_P29_HYPERBOLIC),
        ("ALARM_P29_NO_CONV", ALARM_P29_NO_CONV),
        ("ALARM_P30_TIG_IN_PAST", ALARM_P30_TIG_IN_PAST),
        ("ALARM_P31_NO_TARGET", ALARM_P31_NO_TARGET),
        ("ALARM_P31_NOT_CONVERGED", ALARM_P31_NOT_CONVERGED),
        ("ALARM_P32_NO_TARGET", ALARM_P32_NO_TARGET),
        ("ALARM_P32_DEGENERATE", ALARM_P32_DEGENERATE),
        ("ALARM_P33_NO_TARGET", ALARM_P33_NO_TARGET),
        ("ALARM_P33_NO_TIG", ALARM_P33_NO_TIG),
        ("ALARM_P33_STALE_TARGET", ALARM_P33_STALE_TARGET),
        ("ALARM_P33_DEGENERATE", ALARM_P33_DEGENERATE),
        ("ALARM_P33_LAMBERT", ALARM_P33_LAMBERT),
        ("ALARM_P34_TOO_CLOSE", ALARM_P34_TOO_CLOSE),
        ("ALARM_P37_BAD_TOF", ALARM_P37_BAD_TOF),
        ("ALARM_P37_WRONG_FRAME", ALARM_P37_WRONG_FRAME),
        ("ALARM_P40_NO_PENDING_MANEUVER", ALARM_P40_NO_PENDING_MANEUVER),
        ("ALARM_P40_TIG_IN_PAST", ALARM_P40_TIG_IN_PAST),
        ("ALARM_P40_DV_TOO_SMALL", ALARM_P40_DV_TOO_SMALL),
        ("ALARM_P40_WRONG_REGIME", ALARM_P40_WRONG_REGIME),
        ("ALARM_P41_WRONG_REGIME", ALARM_P41_WRONG_REGIME),
        ("ALARM_COLLINEAR_STARS", ALARM_COLLINEAR_STARS),
        ("ALARM_PLATFORM_CAGED", ALARM_PLATFORM_CAGED),
        ("ALARM_P62_WRONG_PHASE", ALARM_P62_WRONG_PHASE),
        ("ALARM_P63_WRONG_PHASE", ALARM_P63_WRONG_PHASE),
        ("ALARM_P64_EARLY", ALARM_P64_EARLY),
        ("ALARM_P67_WRONG_PHASE", ALARM_P67_WRONG_PHASE),
        ("ALARM_DV_LOAD_WITHOUT_TIG", ALARM_DV_LOAD_WITHOUT_TIG),
    ];

    /// Every synthetic code must fall inside the reserved range.
    #[test]
    fn synthetic_codes_in_reserved_range() {
        for (name, code) in SYNTHETIC_CODES {
            assert!(
                SYNTHETIC_RANGE.contains(code),
                "{name} = 0o{code:o} is outside the synthetic range 0o70000..=0o77777"
            );
        }
    }

    /// Synthetic codes must not collide with any official AGC code.
    #[test]
    fn synthetic_codes_disjoint_from_official() {
        for (name, code) in SYNTHETIC_CODES {
            assert!(
                !OFFICIAL_AGC_CODES.contains(code),
                "{name} = 0o{code:o} collides with an official AGC code"
            );
        }
    }

    /// Faithful-whitelist entries must each appear in the official list.
    #[test]
    fn faithful_codes_appear_in_official() {
        for (name, code) in FAITHFUL_WHITELIST {
            assert!(
                OFFICIAL_AGC_CODES.contains(code),
                "{name} = 0o{code:o} is on the faithful whitelist \
                 but not in OFFICIAL_AGC_CODES"
            );
        }
    }

    /// No two named alarm constants share a value (resolves #182 §C).
    #[test]
    fn no_internal_collisions() {
        let mut all: Vec<(u16, &str)> = FAITHFUL_WHITELIST
            .iter()
            .chain(SYNTHETIC_CODES.iter())
            .map(|(name, code)| (*code, *name))
            .collect();
        all.sort_by_key(|(code, _)| *code);
        for win in all.windows(2) {
            assert_ne!(
                win[0].0, win[1].0,
                "duplicate alarm code 0o{:o}: {} and {}",
                win[0].0, win[0].1, win[1].1
            );
        }
    }

    /// Sanity: OFFICIAL_AGC_CODES is stored sorted (helps human review).
    #[test]
    fn official_codes_sorted() {
        for win in OFFICIAL_AGC_CODES.windows(2) {
            assert!(
                win[0] < win[1],
                "OFFICIAL_AGC_CODES unsorted at 0o{:o}, 0o{:o}",
                win[0], win[1]
            );
        }
    }
}
