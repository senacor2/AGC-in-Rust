//! Verb dispatch table — maps verb codes to categories and actions.
//!
//! Verbs are grouped into displays, monitors, loads, special functions, and
//! extended verbs following the structure of the original PINBALL tables.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — VBRTEFN1/2/3 tables.

// ---------------------------------------------------------------------------
// Verb constants
// ---------------------------------------------------------------------------

/// V01 — Display octal, component 1.
pub const V01_DISPLAY_OCTAL_COMP1: u8 = 1;
/// V04 — Display octal, components 1-3.
pub const V04_DISPLAY_OCTAL_COMP3: u8 = 4;
/// V06 — Display decimal.
pub const V06_DISPLAY_DECIMAL: u8 = 6;
/// V11 — Monitor octal, component 1.
pub const V11_MONITOR_OCTAL_COMP1: u8 = 11;
/// V16 — Monitor decimal.
pub const V16_MONITOR_DECIMAL: u8 = 16;
/// V21 — Load component 1.
pub const V21_LOAD_COMP1: u8 = 21;
/// V22 — Load component 2.
pub const V22_LOAD_COMP2: u8 = 22;
/// V24 — Load components 1, 2, and 3.
pub const V24_LOAD_COMP1_2_3: u8 = 24;
/// V25 — Load components 1, 2, and 3 (same as V24 per AGC).
pub const V25_LOAD_COMP1_2_3: u8 = 25;
/// V32 — Recycle (rerun current program phase).
pub const V32_RECYCLE: u8 = 32;
/// V33 — Proceed without data.
pub const V33_PROCEED_NO_DATA: u8 = 33;
/// V34 — Terminate current activity.
pub const V34_TERMINATE: u8 = 34;
/// V35 — Test lights (lamp test).
pub const V35_TEST_LIGHTS: u8 = 35;
/// V36 — Fresh start.
pub const V36_FRESH_START: u8 = 36;
/// V37 — Change major mode (program number follows in noun field).
pub const V37_CHANGE_PROGRAM: u8 = 37;
/// V50 — Please perform (crew response requested).
pub const V50_PLEASE_PERFORM: u8 = 50;
/// V69 — Cause restart (IMU re-align sequence).
pub const V69_CAUSE_RESTART: u8 = 69;
/// V82 — Display orbital parameters.
pub const V82_ORB_PARAMS: u8 = 82;

// ---------------------------------------------------------------------------
// Verb categories
// ---------------------------------------------------------------------------

/// High-level classification for a verb code.
///
/// The categories follow the groupings described in
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc and the AGC Assembly and Operation
/// Information document.
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — VBRTEFN1/2/3.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerbCategory {
    /// V01-V05: Display data in registers (octal or component select).
    Display,
    /// V06: Display data in decimal.
    DecimalDisplay,
    /// V11-V15: Monitor data (auto-updating display, once per second).
    Monitor,
    /// V16: Monitor data in decimal.
    MonitorDecimal,
    /// V21-V25: Load data (crew enters values into registers).
    Load,
    /// V32-V36, V69: Extended special-function verbs.
    Extended,
    /// V37: Change major mode / program.
    ChangeMajorMode,
    /// V50-V58: "Please" verbs (crew action requested).
    Please,
    /// Verb code not recognised.
    Invalid,
}

/// Classify a verb code into its [`VerbCategory`].
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — VBRTEFN1/2/3 dispatch.
pub fn classify_verb(verb: u8) -> VerbCategory {
    match verb {
        1..=5 => VerbCategory::Display,
        6 => VerbCategory::DecimalDisplay,
        11..=15 => VerbCategory::Monitor,
        16 => VerbCategory::MonitorDecimal,
        21..=25 => VerbCategory::Load,
        32..=36 | 69 => VerbCategory::Extended,
        37 => VerbCategory::ChangeMajorMode,
        50..=58 => VerbCategory::Please,
        _ => VerbCategory::Invalid,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_verb_decimal_display() {
        assert_eq!(classify_verb(6), VerbCategory::DecimalDisplay);
    }

    #[test]
    fn classify_verb_change_major_mode() {
        assert_eq!(classify_verb(37), VerbCategory::ChangeMajorMode);
    }

    #[test]
    fn classify_verb_invalid() {
        assert_eq!(classify_verb(99), VerbCategory::Invalid);
    }

    #[test]
    fn classify_verb_display_range() {
        for v in 1u8..=5 {
            assert_eq!(classify_verb(v), VerbCategory::Display, "verb {v}");
        }
    }

    #[test]
    fn classify_verb_load_range() {
        for v in [21u8, 22, 24, 25] {
            assert_eq!(classify_verb(v), VerbCategory::Load, "verb {v}");
        }
    }

    #[test]
    fn classify_verb_please_range() {
        for v in 50u8..=58 {
            assert_eq!(classify_verb(v), VerbCategory::Please, "verb {v}");
        }
    }

    #[test]
    fn classify_verb_extended() {
        assert_eq!(classify_verb(V32_RECYCLE), VerbCategory::Extended);
        assert_eq!(classify_verb(V36_FRESH_START), VerbCategory::Extended);
        assert_eq!(classify_verb(V69_CAUSE_RESTART), VerbCategory::Extended);
    }

    #[test]
    fn v82_is_invalid_category() {
        // V82 is an extended display verb outside the standard PINBALL table ranges.
        assert_eq!(classify_verb(82), VerbCategory::Invalid);
    }
}
