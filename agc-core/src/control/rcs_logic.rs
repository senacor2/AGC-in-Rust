//! RCS jet selection logic.
//!
//! Maps torque and translation commands to SM RCS jet bitmasks ready for
//! `Rcs::fire_sm_jets`. The CSM SM RCS has four quads (A/B/C/D), each with
//! four jets, for sixteen jets total.
//!
//! Bit layout of `SmJetMask` (u16):
//! ```text
//!  Quad A: bits  0-3  (A1=0, A2=1, A3=2, A4=3)
//!  Quad B: bits  4-7  (B1=4, B2=5, B3=6, B4=7)
//!  Quad C: bits  8-11 (C1=8, C2=9, C3=10, C4=11)
//!  Quad D: bits 12-15 (D1=12, D2=13, D3=14, D4=15)
//! ```
//!
//! Rotation jet pairs follow the AGC topology summary in
//! JET_SELECTION_LOGIC.agc (JETADR table):
//!
//! | Axis  | + torque jets | − torque jets |
//! |-------|---------------|---------------|
//! | Roll  | A4, C2        | A2, C4        |
//! | Pitch | B4, D2        | B2, D4        |
//! | Yaw   | A1, C3        | A3, C1        |
//!
//! Translation jet pairs (Z-axis is along the SPS thrust axis):
//!
//! | Axis | + direction | − direction |
//! |------|-------------|-------------|
//! | X    | B1, D3      | B3, D1      |
//! | Y    | A3, C1      | A1, C3      |
//! | Z    | B2, D4      | B4, D2      |
//!
//! AGC source: JET_SELECTION_LOGIC.agc — JETSLECT, rotation/translation tables.

use crate::hal::rcs::SmJetMask;

// ── Bit positions ─────────────────────────────────────────────────────────────
const A1: u16 = 1 << 0;
const A2: u16 = 1 << 1;
const A3: u16 = 1 << 2;
const A4: u16 = 1 << 3;
const B1: u16 = 1 << 4;
const B2: u16 = 1 << 5;
const B3: u16 = 1 << 6;
const B4: u16 = 1 << 7;
const C1: u16 = 1 << 8;
const C2: u16 = 1 << 9;
const C3: u16 = 1 << 10;
const C4: u16 = 1 << 11;
const D1: u16 = 1 << 12;
const D2: u16 = 1 << 13;
const D3: u16 = 1 << 14;
const D4: u16 = 1 << 15;

// ── Rotation jet masks ────────────────────────────────────────────────────────
const ROLL_POS: SmJetMask = A4 | C2; // +roll torque
const ROLL_NEG: SmJetMask = A2 | C4; // −roll torque
const PITCH_POS: SmJetMask = B4 | D2; // +pitch torque
const PITCH_NEG: SmJetMask = B2 | D4; // −pitch torque
const YAW_POS: SmJetMask = A1 | C3; // +yaw torque
const YAW_NEG: SmJetMask = A3 | C1; // −yaw torque

// ── Translation jet masks ─────────────────────────────────────────────────────
const TRANS_X_POS: SmJetMask = B1 | D3;
const TRANS_X_NEG: SmJetMask = B3 | D1;
const TRANS_Y_POS: SmJetMask = A3 | C1;
const TRANS_Y_NEG: SmJetMask = A1 | C3;
const TRANS_Z_POS: SmJetMask = B2 | D4;
const TRANS_Z_NEG: SmJetMask = B4 | D2;

/// Standard jet firing duration for a minimum impulse bit (14 ms).
///
/// AGC source: JET_SELECTION_LOGIC.agc — T6 minimum-impulse timing constant.
pub const MIN_IMPULSE_MS: u16 = 14;

/// Torque command per axis: `-1` (negative torque), `0` (none), `+1` (positive torque).
///
/// Produced by the phase-plane logic and consumed by `select_rotation_jets`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TorqueCommand {
    pub roll: i8,
    pub pitch: i8,
    pub yaw: i8,
}

/// Select SM RCS jets for rotation commands.
///
/// Returns the combined jet mask for all active axes. Conflicting demands on
/// the same axis are resolved by the phase-plane upstream; this function only
/// ORs the individual axis masks together.
///
/// AGC source: JET_SELECTION_LOGIC.agc — JETSLECT rotation table.
pub fn select_rotation_jets(cmd: &TorqueCommand) -> SmJetMask {
    let mut mask: SmJetMask = 0;
    mask |= axis_mask(cmd.roll, ROLL_POS, ROLL_NEG);
    mask |= axis_mask(cmd.pitch, PITCH_POS, PITCH_NEG);
    mask |= axis_mask(cmd.yaw, YAW_POS, YAW_NEG);
    mask
}

/// Select SM RCS jets for translation commands.
///
/// `x`, `y`, `z` are signed translation demands: `+1` positive, `-1` negative,
/// `0` none.
///
/// AGC source: JET_SELECTION_LOGIC.agc — JETSLECT translation table.
pub fn select_translation_jets(x: i8, y: i8, z: i8) -> SmJetMask {
    let mut mask: SmJetMask = 0;
    mask |= axis_mask(x, TRANS_X_POS, TRANS_X_NEG);
    mask |= axis_mask(y, TRANS_Y_POS, TRANS_Y_NEG);
    mask |= axis_mask(z, TRANS_Z_POS, TRANS_Z_NEG);
    mask
}

/// Helper: return `pos_mask`, `neg_mask`, or `0` depending on command sign.
#[inline]
fn axis_mask(cmd: i8, pos_mask: SmJetMask, neg_mask: SmJetMask) -> SmJetMask {
    match cmd.signum() {
        1 => pos_mask,
        -1 => neg_mask,
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_command_no_jets() {
        let cmd = TorqueCommand::default();
        assert_eq!(select_rotation_jets(&cmd), 0, "no command → no jets");
    }

    #[test]
    fn pure_positive_roll() {
        let cmd = TorqueCommand { roll: 1, pitch: 0, yaw: 0 };
        assert_eq!(select_rotation_jets(&cmd), ROLL_POS);
    }

    #[test]
    fn pure_negative_roll() {
        let cmd = TorqueCommand { roll: -1, pitch: 0, yaw: 0 };
        assert_eq!(select_rotation_jets(&cmd), ROLL_NEG);
    }

    #[test]
    fn pure_positive_pitch() {
        let cmd = TorqueCommand { roll: 0, pitch: 1, yaw: 0 };
        assert_eq!(select_rotation_jets(&cmd), PITCH_POS);
    }

    #[test]
    fn pure_negative_pitch() {
        let cmd = TorqueCommand { roll: 0, pitch: -1, yaw: 0 };
        assert_eq!(select_rotation_jets(&cmd), PITCH_NEG);
    }

    #[test]
    fn pure_positive_yaw() {
        let cmd = TorqueCommand { roll: 0, pitch: 0, yaw: 1 };
        assert_eq!(select_rotation_jets(&cmd), YAW_POS);
    }

    #[test]
    fn pure_negative_yaw() {
        let cmd = TorqueCommand { roll: 0, pitch: 0, yaw: -1 };
        assert_eq!(select_rotation_jets(&cmd), YAW_NEG);
    }

    #[test]
    fn combined_roll_pitch_ors_masks() {
        let cmd = TorqueCommand { roll: 1, pitch: 1, yaw: 0 };
        let expected = ROLL_POS | PITCH_POS;
        assert_eq!(select_rotation_jets(&cmd), expected);
    }

    #[test]
    fn translation_zero_no_jets() {
        assert_eq!(select_translation_jets(0, 0, 0), 0);
    }

    #[test]
    fn translation_positive_x() {
        assert_eq!(select_translation_jets(1, 0, 0), TRANS_X_POS);
    }

    #[test]
    fn translation_negative_z() {
        assert_eq!(select_translation_jets(0, 0, -1), TRANS_Z_NEG);
    }

    #[test]
    fn rotation_masks_are_disjoint_per_axis() {
        // Positive and negative masks for the same axis must not share bits.
        assert_eq!(ROLL_POS & ROLL_NEG, 0, "roll masks overlap");
        assert_eq!(PITCH_POS & PITCH_NEG, 0, "pitch masks overlap");
        assert_eq!(YAW_POS & YAW_NEG, 0, "yaw masks overlap");
    }
}
