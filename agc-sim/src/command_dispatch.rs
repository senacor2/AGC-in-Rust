//! Crossterm key-event dispatcher for the AGC simulator.
//!
//! Routes `crossterm::event::KeyEvent` values to the AGC-core PINBALL verb/noun
//! state machine or to simulator meta-controls (quit, scenario switch, F-keys,
//! time multiplier).
//!
//! DSKY key mappings follow the Comanche055 key-code table:
//!   AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 313.

use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};

use agc_core::hal::dsky::Key;
use agc_core::services::pinball::{dispatch, VerbResult};
use agc_core::services::v_n::{CharResult, VnState};
use agc_core::AgcState;

use crate::sim_hardware::SimHardware;

/// Outcome of dispatching a single key event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DispatchOutcome {
    /// The user requested to quit the simulator (`q` or Ctrl-C).
    Quit,
    /// The key was handled normally; no DSKY keystroke was generated.
    Continue,
    /// A DSKY keystroke was queued.
    KeyQueued(Key),
    /// The key was not recognised.
    Unhandled,
}

/// Map a `crossterm::KeyCode` to an AGC 5-bit key code, if applicable.
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
/// DSKY key code table (page 313) — 18 physical keys.
///
/// Returns the AGC numeric key code (not the `Key` enum) for direct
/// forwarding to `VnState::char_in`.
fn map_key_to_agc_code(code: KeyCode) -> Option<(Key, u8)> {
    match code {
        KeyCode::Char('0') => Some((Key::Zero, 16)), // code 16 = digit 0
        KeyCode::Char('1') => Some((Key::One, 1)),
        KeyCode::Char('2') => Some((Key::Two, 2)),
        KeyCode::Char('3') => Some((Key::Three, 3)),
        KeyCode::Char('4') => Some((Key::Four, 4)),
        KeyCode::Char('5') => Some((Key::Five, 5)),
        KeyCode::Char('6') => Some((Key::Six, 6)),
        KeyCode::Char('7') => Some((Key::Seven, 7)),
        KeyCode::Char('8') => Some((Key::Eight, 8)),
        KeyCode::Char('9') => Some((Key::Nine, 9)),
        // v = VERB (17), n = NOUN (31), Enter = ENTER (28),
        // c = CLR (30), r = KEY REL (25), + = plus (26), - = minus (27).
        KeyCode::Char('v') | KeyCode::Char('V') => Some((Key::Verb, 17)),
        KeyCode::Char('n') | KeyCode::Char('N') => Some((Key::Noun, 31)),
        KeyCode::Enter => Some((Key::Enter, 28)),
        KeyCode::Char('c') | KeyCode::Char('C') => Some((Key::Clear, 30)),
        KeyCode::Char('r') | KeyCode::Char('R') => Some((Key::KeyRel, 25)),
        KeyCode::Char('+') => Some((Key::Plus, 26)),
        KeyCode::Char('-') => Some((Key::Minus, 27)),
        _ => None,
    }
}

/// Dispatch a single `KeyEvent` to the simulator.
///
/// Routes key events through `VnState::char_in` and handles simulator
/// meta-controls (quit, scenario switch, time multiplier).
///
/// When ENTER completes a verb/noun pair, calls `pinball::dispatch` and
/// propagates the result to the mission log.
///
/// Returns `DispatchOutcome` to tell the main loop what happened.
///
/// # Key bindings
///
/// | Key | Action |
/// |-----|--------|
/// | `q` | Quit |
/// | Ctrl-C | Quit |
/// | `0`-`9`, `v`, `n`, Enter, `c`, `r`, `+`, `-` | DSKY key → VnState::char_in |
/// | F1 | Switch to 'launch' scenario |
/// | F2 | Switch to 'burn' scenario |
/// | F3 | Switch to 'free' scenario |
/// | `=` | Increase time multiplier (up to 16×) |
/// | `_` | Decrease time multiplier (down to 0.25×) |
pub fn handle_key_event(event: KeyEvent, hw: &mut SimHardware) -> DispatchOutcome {
    // Ctrl-C always quits.
    if event.modifiers.contains(KeyModifiers::CONTROL) {
        if let KeyCode::Char('c') = event.code {
            return DispatchOutcome::Quit;
        }
    }

    match event.code {
        KeyCode::Char('q') | KeyCode::Char('Q') => DispatchOutcome::Quit,

        // Time multiplier controls: '=' increases, '_' decreases.
        KeyCode::Char('=') => {
            hw.time_multiplier = (hw.time_multiplier * 2.0).min(16.0);
            hw.log
                .info(format!("time multiplier: {:.2}×", hw.time_multiplier));
            DispatchOutcome::Continue
        }
        KeyCode::Char('_') => {
            hw.time_multiplier = (hw.time_multiplier / 2.0).max(0.25);
            hw.log
                .info(format!("time multiplier: {:.2}×", hw.time_multiplier));
            DispatchOutcome::Continue
        }

        // F1/F2/F3: scenario switching (handled by the caller via return value).
        // We log the switch and let the main loop re-initialise.
        KeyCode::F(1) => {
            hw.log.info("F1 pressed — scenario: launch");
            DispatchOutcome::Continue
        }
        KeyCode::F(2) => {
            hw.log.info("F2 pressed — scenario: burn");
            DispatchOutcome::Continue
        }
        KeyCode::F(3) => {
            hw.log.info("F3 pressed — scenario: free");
            DispatchOutcome::Continue
        }

        code => {
            if let Some((dsky_key, agc_code)) = map_key_to_agc_code(code) {
                process_dsky_key(hw, dsky_key, agc_code)
            } else {
                DispatchOutcome::Unhandled
            }
        }
    }
}

/// Process one DSKY key through the VnState machine and optionally dispatch.
///
/// AGC source: CHARIN, ENTPAS0, VERBFAN path.
fn process_dsky_key(hw: &mut SimHardware, dsky_key: Key, agc_code: u8) -> DispatchOutcome {
    // Route through VnState::char_in.
    // We need to borrow vn and agc_state separately from hw.
    // Use a local clone trick: extract vn, process, put back.
    let result = hw.vn.char_in(agc_code);

    // Log the key event.
    hw.log
        .info(format!("VN  key=0x{:02X}  result={:?}", agc_code, result));

    // Update display state mirrors.
    hw.dsky.display.flash_vn = hw.vn.flash;

    match result {
        CharResult::Accepted => {
            // Update verb/noun display.
            if let Some(v) = hw.vn.verb_buf {
                hw.dsky.display.verb = Some(v);
            }
            if let Some(n) = hw.vn.noun_buf {
                hw.dsky.display.noun = Some(n);
            }
            hw.dsky.display.error_light = false;
            DispatchOutcome::KeyQueued(dsky_key)
        }
        CharResult::Rejected => {
            // Set OPERATOR ERROR light.
            hw.dsky.display.error_light = true;
            hw.dsky.display.oprerr = true;
            hw.log.warn(format!(
                "DSKY key 0x{:02X} rejected — OPERATOR ERROR",
                agc_code
            ));
            DispatchOutcome::KeyQueued(dsky_key)
        }
        CharResult::Complete(verb, noun) => {
            // Update displays before dispatch.
            hw.dsky.display.verb = Some(verb);
            hw.dsky.display.noun = Some(noun);
            hw.dsky.display.error_light = false;

            // Dispatch via VERBFAN.
            // We need to pass agc_state, the hardware, and vn separately to
            // dispatch<H: AgcHardware>. The borrow checker requires that
            // agc_state and vn are disjoint from the hw borrow.
            //
            // Strategy: move agc_state and vn out of hw temporarily, call
            // dispatch with hw (the hardware sub-traits), then move them back.
            let mut vn_tmp = std::mem::replace(&mut hw.vn, VnState::new());
            let mut state_tmp = std::mem::replace(&mut hw.agc_state, AgcState::new());
            let verb_result = dispatch(verb, &mut state_tmp, hw, &mut vn_tmp, noun);
            hw.agc_state = state_tmp;
            hw.vn = vn_tmp;

            hw.log.info(format!(
                "VERB  v={:02}  n={:02}  result={:?}",
                verb, noun, verb_result
            ));

            match verb_result {
                VerbResult::Flash => {
                    hw.dsky.display.flash_vn = true;
                }
                VerbResult::Error => {
                    hw.dsky.display.error_light = true;
                    hw.dsky.display.oprerr = true;
                }
                VerbResult::Ok => {
                    hw.dsky.display.flash_vn = false;
                    hw.dsky.display.error_light = false;
                }
            }

            // Sync prog display from modreg.
            let modreg = hw.agc_state.modreg;
            if modreg >= 0 {
                hw.dsky.display.prog = Some(modreg as u8);
            }

            DispatchOutcome::KeyQueued(dsky_key)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crossterm::event::{KeyCode, KeyEvent, KeyEventKind, KeyEventState, KeyModifiers};

    fn key(code: KeyCode) -> KeyEvent {
        KeyEvent {
            code,
            modifiers: KeyModifiers::empty(),
            kind: KeyEventKind::Press,
            state: KeyEventState::empty(),
        }
    }

    fn ctrl(code: KeyCode) -> KeyEvent {
        KeyEvent {
            code,
            modifiers: KeyModifiers::CONTROL,
            kind: KeyEventKind::Press,
            state: KeyEventState::empty(),
        }
    }

    #[test]
    fn quit_on_q() {
        let mut hw = SimHardware::new_headless();
        assert_eq!(
            handle_key_event(key(KeyCode::Char('q')), &mut hw),
            DispatchOutcome::Quit
        );
    }

    #[test]
    fn quit_on_ctrl_c() {
        let mut hw = SimHardware::new_headless();
        assert_eq!(
            handle_key_event(ctrl(KeyCode::Char('c')), &mut hw),
            DispatchOutcome::Quit
        );
    }

    #[test]
    fn digit_keys_route_through_vn_state() {
        let mut hw = SimHardware::new_headless();
        // Press VERB then digit '3', '4' — should reach verb_buf = Some(34).
        handle_key_event(key(KeyCode::Char('v')), &mut hw);
        handle_key_event(key(KeyCode::Char('3')), &mut hw);
        handle_key_event(key(KeyCode::Char('4')), &mut hw);
        assert_eq!(hw.vn.verb_buf, Some(34));
        assert_eq!(hw.dsky.display.verb, Some(34));
    }

    #[test]
    fn verb_noun_enter_produces_complete() {
        let mut hw = SimHardware::new_headless();
        // V37N00 ENTER → changes program to P00.
        handle_key_event(key(KeyCode::Char('v')), &mut hw);
        handle_key_event(key(KeyCode::Char('3')), &mut hw);
        handle_key_event(key(KeyCode::Char('7')), &mut hw);
        handle_key_event(key(KeyCode::Char('n')), &mut hw);
        handle_key_event(key(KeyCode::Char('0')), &mut hw);
        handle_key_event(key(KeyCode::Char('0')), &mut hw);
        let result = handle_key_event(key(KeyCode::Enter), &mut hw);
        assert_eq!(result, DispatchOutcome::KeyQueued(Key::Enter));
        // modreg should be 0 after V37N00 dispatches.
        assert_eq!(hw.agc_state.modreg, 0);
    }

    #[test]
    fn f1_f2_f3_are_continue() {
        let mut hw = SimHardware::new_headless();
        assert_eq!(
            handle_key_event(key(KeyCode::F(1)), &mut hw),
            DispatchOutcome::Continue
        );
        assert_eq!(
            handle_key_event(key(KeyCode::F(2)), &mut hw),
            DispatchOutcome::Continue
        );
        assert_eq!(
            handle_key_event(key(KeyCode::F(3)), &mut hw),
            DispatchOutcome::Continue
        );
    }

    #[test]
    fn unknown_key_is_unhandled() {
        let mut hw = SimHardware::new_headless();
        assert_eq!(
            handle_key_event(key(KeyCode::Tab), &mut hw),
            DispatchOutcome::Unhandled
        );
    }

    #[test]
    fn time_multiplier_increases_on_equals() {
        let mut hw = SimHardware::new_headless();
        assert_eq!(hw.time_multiplier, 1.0);
        handle_key_event(key(KeyCode::Char('=')), &mut hw);
        assert_eq!(hw.time_multiplier, 2.0);
        handle_key_event(key(KeyCode::Char('=')), &mut hw);
        assert_eq!(hw.time_multiplier, 4.0);
    }

    #[test]
    fn time_multiplier_decreases_on_underscore() {
        let mut hw = SimHardware::new_headless();
        handle_key_event(key(KeyCode::Char('_')), &mut hw);
        assert_eq!(hw.time_multiplier, 0.5);
    }

    #[test]
    fn time_multiplier_clamped() {
        let mut hw = SimHardware::new_headless();
        for _ in 0..10 {
            handle_key_event(key(KeyCode::Char('=')), &mut hw);
        }
        assert_eq!(hw.time_multiplier, 16.0);
        for _ in 0..10 {
            handle_key_event(key(KeyCode::Char('_')), &mut hw);
        }
        assert_eq!(hw.time_multiplier, 0.25);
    }
}
