//! Integration tests for the PINBALL verb/noun dispatch system.
//!
//! Tests the full pipeline: VnState::char_in → dispatch → AgcState effects.
//!
//! AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (VERBFAN, VERBTAB).

use agc_core::services::pinball::{dispatch, VerbResult};
use agc_core::services::v_n::{CharResult, InputMode, VnState};
use agc_core::AgcState;
use agc_sim::SimHardware;

/// Helper: type a sequence of AGC key codes into a VnState.
fn type_keys(vn: &mut VnState, keys: &[u8]) -> Vec<CharResult> {
    keys.iter().map(|&k| vn.char_in(k)).collect()
}

/// Helper: type a key and collect one result.
fn key(vn: &mut VnState, code: u8) -> CharResult {
    vn.char_in(code)
}

/// AGC key code constants.
const VERB_KEY: u8 = 17;
const NOUN_KEY: u8 = 31;
const ENTER_KEY: u8 = 28;
const CLR_KEY: u8 = 30;
const DIGIT_0: u8 = 16; // code 16 = digit 0
const DIGIT_3: u8 = 3;
const DIGIT_4: u8 = 4;
const DIGIT_5: u8 = 5;
const DIGIT_6: u8 = 6;
const DIGIT_7: u8 = 7;
const DIGIT_9: u8 = 9;

/// Test 1: V37N00 ENTER dispatches V37N00 → transitions program to P00.
///
/// AGC source: MMCHANG (page 364-365), V37 = change major mode.
#[test]
fn test_v37n00_transitions_to_p00() {
    let mut vn = VnState::new();
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();

    // V key, then '3', '7'.
    assert_eq!(key(&mut vn, VERB_KEY), CharResult::Accepted);
    assert_eq!(key(&mut vn, DIGIT_3), CharResult::Accepted);
    assert_eq!(key(&mut vn, DIGIT_7), CharResult::Accepted);
    assert_eq!(vn.verb_buf, Some(37));

    // N key, then '0', '0'.
    assert_eq!(key(&mut vn, NOUN_KEY), CharResult::Accepted);
    assert_eq!(key(&mut vn, DIGIT_0), CharResult::Accepted);
    assert_eq!(key(&mut vn, DIGIT_0), CharResult::Accepted);
    assert_eq!(vn.noun_buf, Some(0));

    // ENTER → Complete(37, 0).
    let result = key(&mut vn, ENTER_KEY);
    assert_eq!(result, CharResult::Complete(37, 0));

    // Dispatch V37 with noun=0.
    let vr = dispatch(37, &mut state, &mut hw, &mut vn, 0);
    assert_eq!(vr, VerbResult::Ok);
    assert_eq!(state.modreg, 0); // P00
}

/// Test 2: V06N36 ENTER dispatches V06N36 → display shows current MET.
///
/// AGC source: DECDSP + HMSOUT, PINBALL_GAME_BUTTONS_AND_LIGHTS.agc page 333.
#[test]
fn test_v06n36_displays_met() {
    let mut vn = VnState::new();
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();

    // Set MET to 1h00m00s = 360000 centiseconds.
    state.tephem = agc_core::types::Met(360_000);

    // Type V06.
    key(&mut vn, VERB_KEY);
    key(&mut vn, DIGIT_0);
    key(&mut vn, DIGIT_6);
    // Type N36.
    key(&mut vn, NOUN_KEY);
    key(&mut vn, DIGIT_3);
    key(&mut vn, DIGIT_6);

    // ENTER → Complete(6, 36).
    let result = key(&mut vn, ENTER_KEY);
    assert_eq!(result, CharResult::Complete(6, 36));

    // Dispatch V06N36.
    let vr = dispatch(6, &mut state, &mut hw, &mut vn, 36);
    assert_eq!(vr, VerbResult::Ok);
    // R1 should have been written (1 hour).
    assert!(hw.dsky.display.r1.is_some());
    let r1 = hw.dsky.display.r1.unwrap();
    // 1 hour = digit 00001 in display = value 1.
    assert_eq!(r1, 1);
}

/// Test 3: V35 ENTER → V35 lamp test activates all lamps.
///
/// AGC source: VBTSTLTS, page 326.
#[test]
fn test_v35_lamp_test() {
    let mut vn = VnState::new();
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();

    // Type V35.
    key(&mut vn, VERB_KEY);
    key(&mut vn, DIGIT_3);
    key(&mut vn, DIGIT_5);
    assert_eq!(vn.verb_buf, Some(35));

    // ENTER → Complete(35, 0) — noun defaults to 0.
    let result = key(&mut vn, ENTER_KEY);
    assert_eq!(result, CharResult::Complete(35, 0));

    // Dispatch V35.
    let vr = dispatch(35, &mut state, &mut hw, &mut vn, 0);
    assert_eq!(vr, VerbResult::Ok);

    // lamp_test() should have called write_prog(88)/write_verb(88)/write_noun(88).
    assert_eq!(hw.dsky.display.prog, Some(88));
    assert_eq!(hw.dsky.display.verb, Some(88));
    assert_eq!(hw.dsky.display.noun, Some(88));
}

/// Test 4: V99 ENTER → unknown verb → VerbResult::Error (no panic).
///
/// AGC source: GODSPALM / DSPALARM path.
#[test]
fn test_v99_unknown_verb_error() {
    let mut vn = VnState::new();
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();

    // Type V99 (out-of-range for our table).
    key(&mut vn, VERB_KEY);
    key(&mut vn, DIGIT_9);
    key(&mut vn, DIGIT_9);
    assert_eq!(vn.verb_buf, Some(99));

    // ENTER → Complete(99, 0).
    let result = key(&mut vn, ENTER_KEY);
    assert_eq!(result, CharResult::Complete(99, 0));

    // Dispatch V99 → Error (not in table, no panic).
    let vr = dispatch(99, &mut state, &mut hw, &mut vn, 0);
    assert_eq!(vr, VerbResult::Error);
}

/// Test 5: CLR resets input; ENTR on empty buffer is rejected.
///
/// AGC source: CLEAR, LEGALTST, CHARALRM paths.
#[test]
fn test_clr_resets_input_enter_on_empty_rejected() {
    let mut vn = VnState::new();

    // Enter data-entry mode.
    vn.request_data_entry(1);
    assert_eq!(vn.mode, InputMode::EnteringData(1));

    // Type a digit.
    key(&mut vn, DIGIT_4);
    assert_eq!(vn.digit_count, 1);

    // CLR resets the current register.
    vn.clr_press();
    assert_eq!(vn.digit_count, 0);
    assert!(vn.data_buf[0].is_none());

    // In Idle mode, ENTER with no verb_buf → Rejected.
    vn.mode = InputMode::Idle;
    vn.verb_buf = None;
    let result = key(&mut vn, ENTER_KEY);
    assert_eq!(result, CharResult::Rejected);
}

/// Test 6: V08 (spare verb) → VerbResult::Error.
///
/// AGC source: V08 is GODSPALM in Comanche055.
#[test]
fn test_spare_verb_error() {
    let mut vn = VnState::new();
    let mut state = AgcState::new();
    let mut hw = SimHardware::new_headless();
    let vr = dispatch(8, &mut state, &mut hw, &mut vn, 0);
    assert_eq!(vr, VerbResult::Error);
}
