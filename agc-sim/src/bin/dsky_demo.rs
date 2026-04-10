//! Interactive DSKY demo.
//!
//! Run with:
//!   cargo run -p agc-sim --bin dsky_demo
//!
//! Uses a local VerbNoun state machine that mirrors `agc_core::services::v_n`.
//! The real PINBALL processor lives in `agc_core::services::v_n::VerbNounState`
//! with full display formatting in `agc_core::services::display` and verb
//! dispatch in `agc_core::services::pinball`.

use std::io;
use std::time::Duration;

use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{backend::CrosstermBackend, Terminal};

use agc_core::hal::dsky::DskyKey;
use agc_sim::{dsky_terminal, SimHardware};

// ── Local VerbNounState fallback ──────────────────────────────────────────────
//
// Mirrors the API that `agc_core::services::v_n` will eventually export.
// Replace this block with:
//   use agc_core::services::v_n::{VerbNounState, VerbNounAction};
// once the module is available.

/// Which field is currently being entered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EntryField {
    Verb,
    Noun,
}

/// Actions produced by `VerbNounState::process_key`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerbNounAction {
    /// V/N confirmed with ENTER — carry out the requested program step.
    Execute { verb: u8, noun: u8 },
    /// PRO (Proceed) key pressed.
    Proceed,
    /// RSET key pressed — caller should clear alarms.
    Reset,
    /// Invalid entry (e.g. ENTER with fewer than 2 digits buffered).
    Error,
    /// No action of interest to the outer loop.
    None,
}

/// Verb/Noun digit-entry state machine.
///
/// Tracks the currently displayed PROG/VERB/NOUN digits, the active entry
/// field, and a two-digit accumulator.  `flash_tick` drives COMP_ACTY
/// blinking so the UI can signal pending input.
pub struct VerbNounState {
    /// Currently displayed program number (0–99).
    pub prog: u8,
    /// Currently displayed verb (0–99).
    pub verb: u8,
    /// Currently displayed noun (0–99).
    pub noun: u8,
    /// True while VERB/NOUN flash state is "on".
    pub flash: bool,

    field: Option<EntryField>,
    /// Digit buffer: at most two entries, each 0–9.
    buf: [u8; 2],
    buf_len: u8,
}

impl VerbNounState {
    pub const fn new() -> Self {
        Self {
            prog: 0,
            verb: 37,
            noun: 0,
            flash: false,
            field: None,
            buf: [0; 2],
            buf_len: 0,
        }
    }

    /// Force the active program number (e.g. after a V37 EXECUTE).
    pub fn set_prog(&mut self, p: u8) {
        self.prog = p;
    }

    /// Toggle the flash state — call every ~500 ms from the event loop.
    pub fn flash_tick(&mut self) {
        self.flash = !self.flash;
    }

    /// Feed one DSKY key into the state machine and return the resulting action.
    pub fn process_key(&mut self, key: DskyKey) -> VerbNounAction {
        match key {
            DskyKey::Verb => {
                self.field = Some(EntryField::Verb);
                self.buf_len = 0;
                VerbNounAction::None
            }
            DskyKey::Noun => {
                self.field = Some(EntryField::Noun);
                self.buf_len = 0;
                VerbNounAction::None
            }
            DskyKey::Clear => {
                self.buf_len = 0;
                VerbNounAction::None
            }
            DskyKey::Enter => {
                if self.buf_len == 2 {
                    let value = self.buf[0] * 10 + self.buf[1];
                    match self.field {
                        Some(EntryField::Verb) => self.verb = value,
                        Some(EntryField::Noun) => self.noun = value,
                        None => return VerbNounAction::Error,
                    }
                    self.field = None;
                    self.buf_len = 0;
                    VerbNounAction::Execute {
                        verb: self.verb,
                        noun: self.noun,
                    }
                } else {
                    VerbNounAction::Error
                }
            }
            DskyKey::ProceED => VerbNounAction::Proceed,
            DskyKey::Reset => {
                self.field = None;
                self.buf_len = 0;
                VerbNounAction::Reset
            }
            digit => {
                if let Some(d) = dsky_key_digit(digit) {
                    if self.buf_len < 2 && self.field.is_some() {
                        self.buf[self.buf_len as usize] = d;
                        self.buf_len += 1;
                        // Show the digit being typed in the relevant display field.
                        let partial = self.buf[0] * 10 + if self.buf_len == 2 { self.buf[1] } else { 0 };
                        match self.field {
                            Some(EntryField::Verb) => self.verb = partial,
                            Some(EntryField::Noun) => self.noun = partial,
                            None => {}
                        }
                    }
                }
                VerbNounAction::None
            }
        }
    }
}

fn dsky_key_digit(key: DskyKey) -> Option<u8> {
    match key {
        DskyKey::Zero => Some(0),
        DskyKey::One => Some(1),
        DskyKey::Two => Some(2),
        DskyKey::Three => Some(3),
        DskyKey::Four => Some(4),
        DskyKey::Five => Some(5),
        DskyKey::Six => Some(6),
        DskyKey::Seven => Some(7),
        DskyKey::Eight => Some(8),
        DskyKey::Nine => Some(9),
        _ => None,
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() -> io::Result<()> {
    std::panic::set_hook(Box::new(|info| {
        let _ = disable_raw_mode();
        let _ = execute!(io::stdout(), LeaveAlternateScreen);
        eprintln!("\r\nPanic: {info}");
    }));

    let result = run();

    let _ = disable_raw_mode();
    let _ = execute!(io::stdout(), LeaveAlternateScreen);

    result
}

fn run() -> io::Result<()> {
    let mut hw = SimHardware::new();
    let mut vn_state = VerbNounState::new();
    vn_state.set_prog(0); // P00 — CMC idle

    // Reflect initial VN state onto hardware display.
    hw.dsky.display.prog = [vn_state.prog / 10, vn_state.prog % 10];
    hw.dsky.display.verb = [vn_state.verb / 10, vn_state.verb % 10];
    hw.dsky.display.noun = [vn_state.noun / 10, vn_state.noun % 10];

    hw.log.info("AGC-in-Rust DSKY demo started");
    hw.log.info("P00 — CMC IDLE  (V37N00 — select major mode)");
    hw.log.info("V for VERB  N for NOUN  0-9 digits  ENTER confirm");
    hw.log.info("DEL clear  P proceed  R reset  Q quit");

    enable_raw_mode()?;
    execute!(io::stdout(), EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;

    let mut flash_counter: u8 = 0;

    loop {
        let display = hw.display_snapshot();
        terminal.draw(|f| dsky_terminal::render(f, display, &hw.log))?;

        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(k) = event::read()? {
                if k.kind != KeyEventKind::Press {
                    continue;
                }
                match k.code {
                    KeyCode::Esc | KeyCode::Char('q') => break,
                    code => {
                        if let Some(dsky_key) = map_key(code) {
                            hw.dsky_push_key(dsky_key);
                            let action = vn_state.process_key(dsky_key);

                            // Mirror VN state to hardware display registers.
                            hw.dsky.display.prog =
                                [vn_state.prog / 10, vn_state.prog % 10];
                            hw.dsky.display.verb =
                                [vn_state.verb / 10, vn_state.verb % 10];
                            hw.dsky.display.noun =
                                [vn_state.noun / 10, vn_state.noun % 10];

                            handle_action(&mut hw, &mut vn_state, action);
                        }
                    }
                }
            }
        }

        // Flash tick: toggle every ~500 ms (10 × 50 ms polls).
        flash_counter += 1;
        if flash_counter >= 10 {
            vn_state.flash_tick();
            flash_counter = 0;
        }
    }

    Ok(())
}

/// Dispatch a `VerbNounAction` produced by the state machine.
fn handle_action(hw: &mut SimHardware, vn_state: &mut VerbNounState, action: VerbNounAction) {
    match action {
        VerbNounAction::Execute { verb, noun } => {
            hw.log.info(format!("V{verb:02}N{noun:02} EXECUTE"));
            match verb {
                35 => {
                    // V35: lamp test
                    hw.dsky.display.lights.prog_alarm = true;
                    hw.dsky.display.lights.gimbal_lock = true;
                    hw.dsky.display.lights.opr_err = true;
                    hw.log.info("V35 — LAMP TEST");
                }
                37 => {
                    // V37: change major mode program
                    vn_state.set_prog(noun);
                    hw.dsky.display.prog = [noun / 10, noun % 10];
                    hw.log.info(format!("V37N{noun:02} — PROGRAM {noun:02}"));
                }
                82 => {
                    // V82: orbital parameters display request
                    hw.log.info("V82 — REQUEST ORB PARAMS");
                }
                _ => {
                    hw.log
                        .info(format!("V{verb:02}N{noun:02} — not implemented"));
                }
            }
        }
        VerbNounAction::Proceed => hw.log.info("PRO — PROCEED"),
        VerbNounAction::Reset => {
            hw.dsky.display.lights.prog_alarm = false;
            hw.dsky.display.lights.opr_err = false;
            hw.dsky.display.lights.gimbal_lock = false;
            hw.log.warn("RSET — alarms cleared");
        }
        VerbNounAction::Error => {
            hw.dsky.display.lights.opr_err = true;
            hw.log.warn("OPR ERR — invalid entry");
        }
        VerbNounAction::None => {}
    }
}

// ── Key mapping ───────────────────────────────────────────────────────────────

fn map_key(code: KeyCode) -> Option<DskyKey> {
    Some(match code {
        KeyCode::Char('v') => DskyKey::Verb,
        KeyCode::Char('n') => DskyKey::Noun,
        KeyCode::Enter => DskyKey::Enter,
        KeyCode::Char('+') => DskyKey::Plus,
        KeyCode::Char('-') => DskyKey::Minus,
        KeyCode::Char('0') => DskyKey::Zero,
        KeyCode::Char('1') => DskyKey::One,
        KeyCode::Char('2') => DskyKey::Two,
        KeyCode::Char('3') => DskyKey::Three,
        KeyCode::Char('4') => DskyKey::Four,
        KeyCode::Char('5') => DskyKey::Five,
        KeyCode::Char('6') => DskyKey::Six,
        KeyCode::Char('7') => DskyKey::Seven,
        KeyCode::Char('8') => DskyKey::Eight,
        KeyCode::Char('9') => DskyKey::Nine,
        KeyCode::Delete | KeyCode::Backspace => DskyKey::Clear,
        KeyCode::Char('p') => DskyKey::ProceED,
        KeyCode::Char('r') => DskyKey::Reset,
        KeyCode::Char('k') => DskyKey::KeyRel,
        _ => return None,
    })
}
