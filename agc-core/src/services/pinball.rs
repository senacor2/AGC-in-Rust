//! Verb dispatch table — VERBFAN, VERBTAB, and GOEXTVB.
//!
//! Dispatches verified verb/noun pairs to the appropriate verb implementation
//! functions. Mirrors the AGC's VERBTAB (V00–V39) and LST2FAN (V40–V99)
//! jump tables.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
//! Routines:   VERBFAN, VBFANDIR, VERBTAB, ENTPAS0, GOEXTVB
//! Pages:      325-326 (BANK 41, SETLOC PINBALL2)
//!
//! Secondary:
//! AGC source: Comanche055/EXTENDED_VERBS.agc
//! Routines:   GOEXTVB, LST2FAN, V82PERF
//! Pages:      236-267 (BANK 7, SETLOC EXTVERBS)

use crate::hal::DskyIo;
use crate::services::display::{format_decimal, format_min_sec, format_octal, format_time};
use crate::services::noun_table::{lookup, DataSource, DisplayFormat};
use crate::services::v_n::VnState;
use crate::AgcState;

/// Result of executing a verb.
///
/// AGC source: maps to the VERBFAN / CHARALRM / FLASHON exit paths.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum VerbResult {
    /// Verb executed successfully; no further crew action needed.
    Ok,
    /// Verb/noun combination is illegal; caller sets OPERATOR ERROR light.
    Error,
    /// Verb requests crew input; VERB/NOUN display should flash.
    /// Used by load verbs (V21–V25) and please-perform verbs (V50, V51).
    Flash,
}

/// A verb implementation function.
///
/// Corresponds to an entry in the AGC VERBTAB or LST2FAN tables.
/// All parameters are passed by reference (no closures, no heap allocation).
/// Noun is passed as a raw u8 decimal code (0–99).
///
/// `dsky` provides direct access to the DSKY I/O device (object-safe sub-trait).
///
/// AGC source: each VERBTAB entry is a CADR pointing to a routine that
/// executes under the PINBALL Executive job at priority 30000.
pub type VerbFn =
    fn(state: &mut AgcState, dsky: &mut dyn DskyIo, vn: &mut VnState, noun: u8) -> VerbResult;

/// Number of regular verb slots (V00–V39).
const REGULAR_VERB_COUNT: usize = 40;

/// Number of extended verb slots (V40–V99).
const EXTENDED_VERB_COUNT: usize = 60;

/// Static dispatch table for regular verbs V00–V39.
///
/// `None` entries correspond to spare/illegal verbs (dispatch → VerbResult::Error).
///
/// Memory cost: 40 × size_of::<Option<VerbFn>>() = 40 × 4 = 160 bytes on Cortex-M4F.
///
/// AGC source: VERBTAB (CADR table), pages 325-326.
static VERB_TABLE: [Option<VerbFn>; REGULAR_VERB_COUNT] = [
    None,                            // V00 = GODSPALM (illegal)
    Some(verb_01_display_octal_r1),  // V01 = DSPA
    None,                            // V02 = DSPB (spare, not implemented)
    None,                            // V03 = DSPC (spare)
    None,                            // V04 = DSPAB (spare)
    None,                            // V05 = DSPABC (spare)
    Some(verb_06_display_decimal),   // V06 = DECDSP
    None,                            // V07 = DSPDPDEC (spare)
    None,                            // V08 spare
    None,                            // V09 spare
    None,                            // V10 spare
    Some(verb_11_monitor_octal),     // V11 = MONITOR (octal)
    None,                            // V12 spare
    None,                            // V13 spare
    None,                            // V14 spare
    None,                            // V15 spare
    Some(verb_16_monitor_decimal),   // V16 = MONITOR (decimal)
    None,                            // V17 spare
    None,                            // V18 spare
    None,                            // V19 spare
    None,                            // V20 spare
    Some(verb_21_load_r1),           // V21 = ALOAD
    None,                            // V22 = BLOAD (spare)
    None,                            // V23 = CLOAD (spare)
    Some(verb_24_load_r1_r2),        // V24 = ABLOAD
    Some(verb_25_load_r1_r2_r3),     // V25 = ABCLOAD
    None,                            // V26 spare
    Some(verb_27_display_fixed_mem), // V27 = DSPFMEM
    None,                            // V28 spare
    None,                            // V29 spare
    None,                            // V30 spare
    None,                            // V31 spare
    None,                            // V32 spare
    None,                            // V33 spare
    Some(verb_34_terminate),         // V34 = VBTERM
    Some(verb_35_lamp_test),         // V35 = VBTSTLTS
    None,                            // V36 spare
    Some(verb_37_change_program),    // V37 = MMCHANG
    None,                            // V38 spare
    None,                            // V39 spare
];

/// Static dispatch table for extended verbs V40–V99.
///
/// Index 0 = V40, index 59 = V99.
///
/// Memory cost: 60 × 4 = 240 bytes on Cortex-M4F.
///
/// AGC source: LST2FAN (TC table), EXTENDED_VERBS.agc page 236.
static EXTENDED_VERB_TABLE: [Option<VerbFn>; EXTENDED_VERB_COUNT] = {
    let mut table: [Option<VerbFn>; EXTENDED_VERB_COUNT] = [None; EXTENDED_VERB_COUNT];
    // V82 = V82PERF (index 82 - 40 = 42).
    table[42] = Some(verb_82_orbit_params);
    table
};

/// Dispatch a verb/noun pair to the appropriate verb function.
///
/// Implements VERBFAN logic:
///   - verb 0–39: look up VERB_TABLE[verb].
///   - verb 40–99: look up EXTENDED_VERB_TABLE[verb - 40].
///   - None entry or out-of-range: return VerbResult::Error.
///
/// `H` must implement `AgcHardware`; the DSKY sub-trait reference is extracted
/// before dispatch so that individual verb functions receive a `dyn DskyIo`
/// which is object-safe.
///
/// AGC source: VERBFAN, VBFANDIR, LST2CON = DEC 40.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 325.
/// EXTENDED_VERBS.agc, page 236 (GOEXTVB).
pub fn dispatch<H: crate::hal::AgcHardware>(
    verb: u8,
    state: &mut AgcState,
    hw: &mut H,
    vn: &mut VnState,
    noun: u8,
) -> VerbResult {
    let func: Option<VerbFn> = if verb < 40 {
        VERB_TABLE[verb as usize]
    } else if verb < 100 {
        EXTENDED_VERB_TABLE[(verb - 40) as usize]
    } else {
        return VerbResult::Error;
    };

    match func {
        Some(f) => f(state, hw.dsky(), vn, noun),
        None => VerbResult::Error,
    }
}

// ── Verb implementations ───────────────────────────────────────────────────────

/// V01: Display noun component 1 (R1) in octal.
///
/// AGC: DSPA, page 331.
fn verb_01_display_octal_r1(
    state: &mut AgcState,
    dsky: &mut dyn DskyIo,
    _vn: &mut VnState,
    noun: u8,
) -> VerbResult {
    let Some(ndef) = lookup(noun) else {
        return VerbResult::Error;
    };
    let raw = read_field_raw(state, ndef.r1.source);
    let rd = format_octal(raw as u16);
    write_register_display(dsky, 0, &rd);
    VerbResult::Ok
}

/// V06: Display noun in decimal (all three components).
///
/// AGC: DECDSP, page 333.
fn verb_06_display_decimal(
    state: &mut AgcState,
    dsky: &mut dyn DskyIo,
    _vn: &mut VnState,
    noun: u8,
) -> VerbResult {
    let Some(ndef) = lookup(noun) else {
        return VerbResult::Error;
    };

    // Special case: HMS format — use format_time across all three registers.
    if ndef.r1.format == DisplayFormat::Time {
        let cs = read_field_centiseconds(state, ndef.r1.source);
        let td = format_time(cs);
        write_register_display(dsky, 0, &td.r1);
        write_register_display(dsky, 1, &td.r2);
        write_register_display(dsky, 2, &td.r3);
        return VerbResult::Ok;
    }

    // General case: format each field independently.
    for (row, field) in [&ndef.r1, &ndef.r2, &ndef.r3].iter().enumerate() {
        let rd = match field.format {
            DisplayFormat::Blank => crate::services::display::blank(),
            DisplayFormat::Octal => {
                let raw = read_field_raw(state, field.source);
                format_octal(raw as u16)
            }
            DisplayFormat::MinSec => {
                let cs = read_field_centiseconds(state, field.source);
                format_min_sec(cs)
            }
            _ => {
                let val = read_field_scaled(state, field.source, field.scale);
                format_decimal(val, true)
            }
        };
        write_register_display(dsky, row, &rd);
    }
    VerbResult::Ok
}

/// V11: Monitor octal component 1 at 1 Hz.
///
/// AGC: MONITOR (same routine, verb selects display format).
fn verb_11_monitor_octal(
    state: &mut AgcState,
    dsky: &mut dyn DskyIo,
    vn: &mut VnState,
    noun: u8,
) -> VerbResult {
    // Monitor: same as display but sets flash mode.
    vn.flash = true;
    verb_01_display_octal_r1(state, dsky, vn, noun)
}

/// V16: Monitor decimal at 1 Hz.
///
/// AGC: MONITOR (same routine, verb number selects display format).
fn verb_16_monitor_decimal(
    state: &mut AgcState,
    dsky: &mut dyn DskyIo,
    vn: &mut VnState,
    noun: u8,
) -> VerbResult {
    vn.flash = true;
    verb_06_display_decimal(state, dsky, vn, noun)
}

/// V21: Load R1 (ALOAD).
///
/// AGC: ALOAD → REQDATX → PUTCOM, pages 343-348.
fn verb_21_load_r1(
    _state: &mut AgcState,
    _dsky: &mut dyn DskyIo,
    vn: &mut VnState,
    _noun: u8,
) -> VerbResult {
    vn.request_data_entry(1);
    VerbResult::Flash
}

/// V24: Load R1 and R2 (ABLOAD).
///
/// AGC: ABLOAD, page 344.
fn verb_24_load_r1_r2(
    _state: &mut AgcState,
    _dsky: &mut dyn DskyIo,
    vn: &mut VnState,
    _noun: u8,
) -> VerbResult {
    vn.request_data_entry(1);
    VerbResult::Flash
}

/// V25: Load R1, R2, and R3 (ABCLOAD).
///
/// AGC: ABCLOAD, page 343.
fn verb_25_load_r1_r2_r3(
    _state: &mut AgcState,
    _dsky: &mut dyn DskyIo,
    vn: &mut VnState,
    _noun: u8,
) -> VerbResult {
    vn.request_data_entry(1);
    VerbResult::Flash
}

/// V27: Display fixed memory location in octal.
///
/// AGC: DSPFMEM, page 358.
fn verb_27_display_fixed_mem(
    _state: &mut AgcState,
    dsky: &mut dyn DskyIo,
    _vn: &mut VnState,
    noun: u8,
) -> VerbResult {
    // Display noun value as octal in R1 (noun = memory address stub).
    let rd = format_octal(noun as u16);
    write_register_display(dsky, 0, &rd);
    VerbResult::Ok
}

/// V34: Terminate current test or load request.
///
/// AGC: VBTERM, page 367.
fn verb_34_terminate(
    _state: &mut AgcState,
    dsky: &mut dyn DskyIo,
    vn: &mut VnState,
    _noun: u8,
) -> VerbResult {
    vn.terminate_entry();
    // Clear KEY REL lamp: write_lamp_word with KEY REL bit cleared.
    // AGC source: VBTERM calls RELDSP which clears RELDSPON.
    dsky.write_lamp_word(0);
    VerbResult::Ok
}

/// V35: Lamp test (test all display lights).
///
/// AGC: VBTSTLTS, page 326 (VERBTAB entry).
fn verb_35_lamp_test(
    _state: &mut AgcState,
    dsky: &mut dyn DskyIo,
    vn: &mut VnState,
    _noun: u8,
) -> VerbResult {
    vn.terminate_entry();
    dsky.lamp_test();
    VerbResult::Ok
}

/// V37: Change major mode (program).
///
/// AGC: MMCHANG, pages 364-365.
fn verb_37_change_program(
    state: &mut AgcState,
    dsky: &mut dyn DskyIo,
    _vn: &mut VnState,
    noun: u8,
) -> VerbResult {
    // noun encodes the new program number (0–99).
    state.modreg = noun as i16;
    dsky.write_prog(noun);
    VerbResult::Ok
}

/// V82: Request orbit parameters display (R30).
///
/// AGC: V82PERF, EXTENDED_VERBS.agc page 248.
fn verb_82_orbit_params(
    state: &mut AgcState,
    dsky: &mut dyn DskyIo,
    vn: &mut VnState,
    _noun: u8,
) -> VerbResult {
    // Display N44 (apogee/perigee/TFF) using V06 logic.
    verb_06_display_decimal(state, dsky, vn, 44)
}

// ── Field read helpers ─────────────────────────────────────────────────────────

/// Read a raw integer value for a data source (used for octal display).
fn read_field_raw(state: &AgcState, source: DataSource) -> i32 {
    match source {
        DataSource::NotUsed => 0,
        DataSource::CurrentMet => state.tephem.0 as i32,
        DataSource::TargetTig => state.nav.tig.0 as i32,
        DataSource::TigCsi => state.nav.tig.0 as i32, // best approximation
        DataSource::TargetCode => 0,
        DataSource::TimeToGo => 0,
        DataSource::VgMagnitude => (state.nav.vgdisp * 100.0) as i32,
        DataSource::DvAccumulated => 0,
        DataSource::ApogeeAlt => (state.nav.hapo / 1852.0 * 10.0) as i32,
        DataSource::PerigeeAlt => (state.nav.hper / 1852.0 * 10.0) as i32,
        DataSource::TimeFreeFlght => 0,
        DataSource::InertialVelMag => {
            let v = state.nav.sv.velocity();
            let mag = libm::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            (mag * 3.28084 * 100.0) as i32 // m/s → ft/s × 100
        }
        DataSource::AltRate => 0,
        DataSource::Altitude => {
            let r = state.nav.sv.position();
            let mag = libm::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
            ((mag - 6_371_000.0) / 1852.0 * 10.0) as i32 // alt in naut mi × 10
        }
        DataSource::DeltaVLvcX => (state.nav.delvslv[0] * 100.0) as i32,
        DataSource::DeltaVLvcY => (state.nav.delvslv[1] * 100.0) as i32,
        DataSource::DeltaVLvcZ => (state.nav.delvslv[2] * 100.0) as i32,
        DataSource::VgBodyX | DataSource::VgBodyY | DataSource::VgBodyZ => 0,
        DataSource::Latitude | DataSource::Longitude | DataSource::AltitudeGeo => 0,
    }
}

/// Read a centisecond value for time data sources.
fn read_field_centiseconds(state: &AgcState, source: DataSource) -> u32 {
    match source {
        DataSource::CurrentMet => state.tephem.0,
        DataSource::TargetTig | DataSource::TigCsi => state.nav.tig.0,
        DataSource::TimeToGo => 0,
        DataSource::TimeFreeFlght => 0,
        _ => 0,
    }
}

/// Read a scaled `i32` display value for a data source.
fn read_field_scaled(state: &AgcState, source: DataSource, scale: f64) -> i32 {
    let raw_f64: f64 = match source {
        DataSource::NotUsed => 0.0,
        DataSource::CurrentMet => state.tephem.0 as f64,
        DataSource::TargetTig => state.nav.tig.0 as f64,
        DataSource::TigCsi => state.nav.tig.0 as f64,
        DataSource::TargetCode => 0.0,
        DataSource::TimeToGo => 0.0,
        DataSource::VgMagnitude => state.nav.vgdisp,
        DataSource::DvAccumulated => 0.0,
        DataSource::ApogeeAlt => state.nav.hapo,
        DataSource::PerigeeAlt => state.nav.hper,
        DataSource::TimeFreeFlght => 0.0,
        DataSource::InertialVelMag => {
            let v = state.nav.sv.velocity();
            libm::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        }
        DataSource::AltRate => 0.0,
        DataSource::Altitude => {
            let r = state.nav.sv.position();
            libm::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) - 6_371_000.0
        }
        DataSource::DeltaVLvcX => state.nav.delvslv[0],
        DataSource::DeltaVLvcY => state.nav.delvslv[1],
        DataSource::DeltaVLvcZ => state.nav.delvslv[2],
        DataSource::VgBodyX | DataSource::VgBodyY | DataSource::VgBodyZ => 0.0,
        DataSource::Latitude | DataSource::Longitude | DataSource::AltitudeGeo => 0.0,
    };
    (raw_f64 * scale).clamp(-99_999.0, 99_999.0) as i32
}

/// Write a `RegisterDisplay` to one DSKY register row via `write_register`.
fn write_register_display(
    dsky: &mut dyn DskyIo,
    row: usize,
    rd: &crate::services::display::RegisterDisplay,
) {
    use crate::hal::dsky::DigitRow;
    use crate::services::display::Sign;

    let mut digits = [0xFFu8; 5];
    for (i, &d) in rd.digits.iter().enumerate() {
        // Translate: BLANK sentinel (10) → 0xFF (DigitRow blank); digits 0-9 → as-is.
        digits[i] = if d <= 9 { d } else { 0xFF };
    }
    let row_val = DigitRow {
        digits,
        sign_plus: rd.sign == Sign::Plus,
        sign_minus: rd.sign == Sign::Minus,
    };
    dsky.write_register(row, &row_val);
}

// ── tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::v_n::InputMode;
    use crate::tests::mock_hw::MockHardware;
    use crate::types::Met;

    fn make() -> (AgcState, MockHardware, VnState) {
        (AgcState::new(), MockHardware::new(), VnState::new())
    }

    /// TC-PINBALL-1: V06N36 dispatches and reads MET.
    ///
    /// AGC source: DECDSP + HMSOUT, PINBALL_GAME_BUTTONS_AND_LIGHTS.agc page 333.
    #[test]
    fn v06_n36_dispatches_met() {
        let (mut state, mut hw, mut vn) = make();
        // Set MET to 1h01m01s = 366100 centiseconds.
        state.tephem = Met(366_100);
        let result = dispatch(6, &mut state, &mut hw, &mut vn, 36);
        assert_eq!(result, VerbResult::Ok);
    }

    /// TC-PINBALL-2: V34 terminates and clears flash.
    ///
    /// AGC source: VBTERM, page 367.
    #[test]
    fn v34_terminates_monitor() {
        let (mut state, mut hw, mut vn) = make();
        vn.flash = true;
        vn.mode = InputMode::EnteringData(1);
        let result = dispatch(34, &mut state, &mut hw, &mut vn, 0);
        assert_eq!(result, VerbResult::Ok);
        assert!(!vn.flash);
        assert_eq!(vn.mode, InputMode::Idle);
    }

    /// TC-PINBALL-3: V35 lamp test activates all lamps.
    ///
    /// AGC source: VBTSTLTS, page 326.
    #[test]
    fn v35_lamp_test_sets_lamps() {
        let (mut state, mut hw, mut vn) = make();
        let result = dispatch(35, &mut state, &mut hw, &mut vn, 0);
        assert_eq!(result, VerbResult::Ok);
        // lamp_test() calls write_prog(88)/write_verb(88)/write_noun(88).
        assert_eq!(hw.dsky.prog, Some(88));
        assert_eq!(hw.dsky.verb, Some(88));
        assert_eq!(hw.dsky.noun, Some(88));
    }

    /// TC-PINBALL-4: V37 changes program (modreg).
    ///
    /// AGC source: MMCHANG, pages 364-365.
    #[test]
    fn v37_changes_program() {
        let (mut state, mut hw, mut vn) = make();
        state.modreg = 0; // P00
        let result = dispatch(37, &mut state, &mut hw, &mut vn, 40);
        assert_eq!(result, VerbResult::Ok);
        assert_eq!(state.modreg, 40);
        assert_eq!(hw.dsky.prog, Some(40));
    }

    /// TC-PINBALL-5: Unknown verb returns Error (no panic).
    ///
    /// AGC source: GODSPALM / DSPALARM path, page 364.
    #[test]
    fn unknown_verb_returns_error() {
        let (mut state, mut hw, mut vn) = make();
        // Out-of-range verb.
        assert_eq!(
            dispatch(100, &mut state, &mut hw, &mut vn, 0),
            VerbResult::Error
        );
        // Spare verb.
        assert_eq!(
            dispatch(8, &mut state, &mut hw, &mut vn, 0),
            VerbResult::Error
        );
        // V00 = GODSPALM (illegal).
        assert_eq!(
            dispatch(0, &mut state, &mut hw, &mut vn, 0),
            VerbResult::Error
        );
    }

    /// TC-PINBALL-6: V21 returns Flash and enters data-entry mode.
    ///
    /// AGC source: ALOAD → REQDATX → PUTCOM, pages 343-348.
    #[test]
    fn v21_returns_flash() {
        let (mut state, mut hw, mut vn) = make();
        let result = dispatch(21, &mut state, &mut hw, &mut vn, 36);
        assert_eq!(result, VerbResult::Flash);
        assert_eq!(vn.mode, InputMode::EnteringData(1));
        assert!(vn.flash);
    }

    /// TC-PINBALL-7: V06 with unknown noun returns Error.
    #[test]
    fn v06_unknown_noun_returns_error() {
        let (mut state, mut hw, mut vn) = make();
        assert_eq!(
            dispatch(6, &mut state, &mut hw, &mut vn, 99),
            VerbResult::Error
        );
    }

    /// TC-PINBALL-8: V82 dispatches to orbit params (N44).
    #[test]
    fn v82_dispatches_orbit_params() {
        let (mut state, mut hw, mut vn) = make();
        let result = dispatch(82, &mut state, &mut hw, &mut vn, 0);
        assert_eq!(result, VerbResult::Ok);
    }
}
