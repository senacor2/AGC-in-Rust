//! RCS jet selection logic (JETSLECT / PWORD / YWORD / RWORD).
//!
//! Translates per-axis jet decisions (`JetDecision`) into a pair of channel
//! words (`JetCommand`) using the PYTABLE / RTABLE lookup approach from the
//! AGC's JET_SELECTION_LOGIC.agc.  Implements the no-failure, no-translation
//! common case (see spec §Restrictions).
//!
//! AGC source: Comanche055/JET_SELECTION_LOGIC.agc
//!   JETSLECT, PWORD, YWORD, RWORD, TABPCOM, TABYCOM, TABRCOM,
//!   PITCHTIM, YAWTIME, ROLLTIME, T6SETUP, T6START (pages 1039-1062).
//! AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
//!   ZEROJET (minimum impulse T6 setup, pages 1015-1016).

use crate::control::attitude::JetDecision;
use crate::hal::rcs::{JetCommand, ACRJETS_MASK, BDRJETS_MASK, PJETS_MASK, YJETS_MASK};
use crate::types::Met;

/// Minimum RCS jet impulse duration: 14 ms expressed in TIME6 counts.
///
/// 1 TIME6 count ≈ 0.625 ms; 14 ms / 0.625 ms = 22.4 → 23 counts.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc `=14MS DEC 23` (line 568).
///             Comment at line 462: "TO INSURE THAT JETS ARE NOT FIRED FOR
///             LESS THAN A MINIMUM IMPULSE (14MS)".
///             Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc ZEROJET `CAF =+14MS TS TIME6`.
pub const MIN_IMPULSE_TIME6_COUNTS: u32 = 23;

/// Minimum RCS jet impulse duration in centiseconds (rounded up: 2 cs = 20 ms).
///
/// 14 ms rounds up to 2 centiseconds for Waitlist scheduling.
/// The true value is 14 ms; 2 cs (20 ms) is the nearest centisecond above.
///
/// AGC source: same as `MIN_IMPULSE_TIME6_COUNTS`.
pub const MIN_IMPULSE_CS: u32 = 2;

/// Pitch/yaw jet lookup table (PYTABLE), 15 entries.
///
/// Indexed by (rotation_cmd × 3 + translation_cmd), where:
///   rotation_cmd: 0 = none, 1 = positive, 2 = negative
///   translation_cmd: 0 = no-trans, 1 = +X trans, 2 = −X trans
/// Entries 9-11: A(B) quad failed; entries 12-14: C(D) quad failed.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc PYTABLE (lines 172-186).
const PYTABLE: [u16; 15] = [
    0o00000, 0o05125, 0o05252, // no-fail, no-trans: none/+/-
    0o00231, 0o02421, 0o02610, // no-fail, +X trans
    0o00146, 0o02504, 0o02442, // no-fail, -X trans
    0o00000, 0o02421, 0o02442, // A(B) quad failed
    0o00000, 0o02504, 0o02610, // C(D) quad failed
];

/// Roll jet lookup table (RTABLE), 15 entries.
///
/// Same indexing as PYTABLE. Bits are split by ACRJETS / BDRJETS masks.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc RTABLE (lines 388-410).
const RTABLE: [u16; 15] = [
    0o11000, 0o22125, 0o00252, // no-fail, no-trans: none/+/-
    0o11231, 0o15421, 0o04610, // no-fail, +Y(Z) trans
    0o11146, 0o15504, 0o04442, // no-fail, -Y(Z) trans
    0o11000, 0o15504, 0o04610, // A(B) quad failed
    0o11000, 0o15421, 0o04442, // C(D) quad failed
];

// Verify channel masks match HAL at compile time.
const _: () = assert!(
    PJETS_MASK & YJETS_MASK == 0,
    "PJETS and YJETS masks must be disjoint"
);

/// Translate per-axis jet decisions into a channel word pair (`JetCommand`).
///
/// Implements the PYTABLE / RTABLE lookup for the no-failure, no-translation
/// case (the common flight case outside manual translation commands or quad failures).
///
/// Output:
///   `JetCommand::pitch_yaw` — written to channel 5 (PYJETS)
///   `JetCommand::roll`      — written to channel 6 (ROLLJETS)
///
/// All-None → `JetCommand::OFF` (both channels zero).
///
/// # Invariants
/// - No heap, no blocking, deterministic table lookup.
/// - Pitch and yaw bits never overlap (PJETS and YJETS masks are disjoint).
/// - Roll bits are placed in channel 6 only.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc
///   PWORD (PYTABLE pitch lookup, p. 1039),
///   YWORD (PYTABLE yaw lookup, p. 1041),
///   RWORD (RTABLE roll lookup, p. 1043),
///   TABPCOM / TABYCOM / TABRCOM entry points,
///   T6START channel write sequence (p. 1061).
pub fn select_jets(pitch: JetDecision, yaw: JetDecision, roll: JetDecision) -> JetCommand {
    // Map JetDecision to PYTABLE index (0 = none, 1 = +, 2 = -)
    let pitch_idx = decision_to_idx(pitch);
    let yaw_idx = decision_to_idx(yaw);
    let roll_idx = decision_to_idx(roll);

    // No-failure, no-translation: first three entries (offset 0..2)
    let pword_raw = PYTABLE[pitch_idx];
    let yword_raw = PYTABLE[yaw_idx];

    // Mask and combine pitch+yaw into channel 5
    let pitch_yaw = (pword_raw & PJETS_MASK) | (yword_raw & YJETS_MASK);

    // Roll: RTABLE[0] = 0o11000 encodes an AGC null-rotation attitude-hold pair,
    // but for the "None" decision (no torque command) we must output zero jets.
    // AGC invariant: JETSLECT does NOT fire roll jets when no roll correction is needed;
    // the 0o11000 entry is only used internally for ZEROJET duration gating.
    // Therefore, for roll=None, force roll_ch6 = 0.
    let roll_ch6 = if roll == JetDecision::None {
        0u16
    } else {
        let rword_raw = RTABLE[roll_idx];
        (rword_raw & ACRJETS_MASK) | (rword_raw & BDRJETS_MASK)
    };

    JetCommand {
        pitch_yaw,
        roll: roll_ch6,
    }
}

/// Return the minimum jet impulse duration as a `Met` value.
///
/// Callers use this to set T6 timers and ensure no jet is fired for less than
/// 14 ms (rounded up to the nearest centisecond).
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc `=14MS DEC 23` (line 568).
pub fn min_impulse_duration() -> Met {
    Met::from_centiseconds(MIN_IMPULSE_CS)
}

/// Convert a `JetDecision` to a zero-based table index (0=none, 1=pos, 2=neg).
#[inline]
fn decision_to_idx(d: JetDecision) -> usize {
    match d {
        JetDecision::None => 0,
        JetDecision::Positive => 1,
        JetDecision::Negative => 2,
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::attitude::JetDecision;
    use crate::hal::rcs::{ACRJETS_MASK, PJETS_MASK, YJETS_MASK};

    /// TC-RCS-1: All None → zero command (OFF).
    #[test]
    fn all_none_gives_off() {
        let cmd = select_jets(JetDecision::None, JetDecision::None, JetDecision::None);
        assert_eq!(cmd.pitch_yaw, 0, "pitch_yaw should be 0");
        assert_eq!(cmd.roll, 0, "roll should be 0");
    }

    /// TC-RCS-2: Pitch Positive → expected PYJETS bits.
    ///
    /// PYTABLE[1] = 0o05125; 0o05125 & PJETS_MASK (0o01417) = 0o01025.
    #[test]
    fn pitch_positive_sets_pjets() {
        let cmd = select_jets(JetDecision::Positive, JetDecision::None, JetDecision::None);
        let expected_pjets = 0o05125u16 & PJETS_MASK;
        assert_eq!(
            cmd.pitch_yaw & PJETS_MASK,
            expected_pjets,
            "pitch_yaw PJETS bits: got {:04o} expected {:04o}",
            cmd.pitch_yaw & PJETS_MASK,
            expected_pjets
        );
        assert_eq!(cmd.roll, 0, "roll should be 0 for pitch-only command");
    }

    /// TC-RCS-3: Roll Negative → expected ROLLJETS bits.
    ///
    /// RTABLE[2] = 0o00252; 0o00252 & ACRJETS_MASK (0o03760) = 0o00240.
    #[test]
    fn roll_negative_sets_rolljets() {
        let cmd = select_jets(JetDecision::None, JetDecision::None, JetDecision::Negative);
        let expected_acrjets = 0o00252u16 & ACRJETS_MASK;
        assert_eq!(
            cmd.roll & ACRJETS_MASK,
            expected_acrjets,
            "roll ACRJETS bits: got {:04o} expected {:04o}",
            cmd.roll & ACRJETS_MASK,
            expected_acrjets
        );
        assert_eq!(
            cmd.pitch_yaw, 0,
            "pitch_yaw should be 0 for roll-only command"
        );
    }

    /// TC-RCS-4: All three Positive → combined bits set in both channels.
    #[test]
    fn all_positive_sets_all_channels() {
        let cmd = select_jets(
            JetDecision::Positive,
            JetDecision::Positive,
            JetDecision::Positive,
        );
        assert_ne!(cmd.pitch_yaw & PJETS_MASK, 0, "PJETS bits should be set");
        assert_ne!(cmd.pitch_yaw & YJETS_MASK, 0, "YJETS bits should be set");
        assert_ne!(cmd.roll & ACRJETS_MASK, 0, "ACRJETS bits should be set");
    }

    /// TC-RCS-5: Pitch and yaw bits are always disjoint.
    #[test]
    fn pitch_yaw_bits_disjoint() {
        // This is a compile-time assertion, but verify at runtime too
        assert_eq!(PJETS_MASK & YJETS_MASK, 0, "masks must be disjoint");
    }

    /// TC-RCS-6: min_impulse_duration returns 2 centiseconds.
    #[test]
    fn min_impulse_is_2cs() {
        let m = min_impulse_duration();
        assert_eq!(m.as_centiseconds(), MIN_IMPULSE_CS);
    }
}
