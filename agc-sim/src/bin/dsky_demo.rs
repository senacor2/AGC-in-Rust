//! Interactive DSKY demo.
//!
//! Run with:
//!   cargo run -p agc-sim --bin dsky_demo

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

fn main() -> io::Result<()> {
    // Restore terminal on panic so the shell is never left broken.
    std::panic::set_hook(Box::new(|info| {
        let _ = disable_raw_mode();
        let _ = execute!(io::stdout(), LeaveAlternateScreen);
        eprintln!("\r\nPanic: {info}");
    }));

    let result = run();

    // Always restore even on clean exit.
    let _ = disable_raw_mode();
    let _ = execute!(io::stdout(), LeaveAlternateScreen);

    result
}

fn run() -> io::Result<()> {
    let mut hw = SimHardware::new();

    // Initial display: P00 / V37 / N00 (CMC idle — select major mode).
    hw.dsky.display.prog = [0, 0];
    hw.dsky.display.verb = [3, 7];
    hw.dsky.display.noun = [0, 0];

    hw.log.info("AGC-in-Rust DSKY demo started");
    hw.log.info("P00 — CMC IDLE");
    hw.log.info("V for VERB  N for NOUN  0-9 digits");
    hw.log.info("ENTER confirm  DEL clear  Q quit");

    enable_raw_mode()?;
    execute!(io::stdout(), EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;

    // Verb/Noun digit entry state.
    let mut mode: Option<&'static str> = None;
    let mut buf = String::new();

    loop {
        let display = hw.display_snapshot();
        terminal.draw(|f| dsky_terminal::render(f, display, &hw.log))?;

        // Block up to 50 ms waiting for a key — standard ratatui pattern.
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
                            handle_key(&mut hw, dsky_key, &mut mode, &mut buf);
                        }
                    }
                }
            }
        }
    }

    Ok(())
}

/// Map a PC keycode to a DSKY key, if applicable.
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

fn handle_key(
    hw: &mut SimHardware,
    key: DskyKey,
    mode: &mut Option<&'static str>,
    buf: &mut String,
) {
    match key {
        DskyKey::Verb => {
            *mode = Some("VERB");
            buf.clear();
            hw.log.info("VERB ▸ enter 2 digits");
        }
        DskyKey::Noun => {
            *mode = Some("NOUN");
            buf.clear();
            hw.log.info("NOUN ▸ enter 2 digits");
        }
        DskyKey::Zero => push_digit(hw, mode, buf, 0),
        DskyKey::One => push_digit(hw, mode, buf, 1),
        DskyKey::Two => push_digit(hw, mode, buf, 2),
        DskyKey::Three => push_digit(hw, mode, buf, 3),
        DskyKey::Four => push_digit(hw, mode, buf, 4),
        DskyKey::Five => push_digit(hw, mode, buf, 5),
        DskyKey::Six => push_digit(hw, mode, buf, 6),
        DskyKey::Seven => push_digit(hw, mode, buf, 7),
        DskyKey::Eight => push_digit(hw, mode, buf, 8),
        DskyKey::Nine => push_digit(hw, mode, buf, 9),
        DskyKey::Clear => {
            buf.clear();
            hw.log.info("CLR");
        }
        DskyKey::Enter => {
            if buf.len() == 2 {
                let d0 = buf.as_bytes()[0] - b'0';
                let d1 = buf.as_bytes()[1] - b'0';
                match *mode {
                    Some("VERB") => {
                        hw.dsky.display.verb = [d0, d1];
                        hw.log.info(format!("VERB → {}{}", d0, d1));
                    }
                    Some("NOUN") => {
                        hw.dsky.display.noun = [d0, d1];
                        hw.log.info(format!("NOUN → {}{}", d0, d1));
                    }
                    _ => hw.log.warn("ENTR with no active mode"),
                }
            } else {
                hw.log
                    .warn(format!("ENTR: need 2 digits, got {}", buf.len()));
            }
            *mode = None;
            buf.clear();
        }
        DskyKey::Reset => {
            hw.dsky.display.prog = [0, 0];
            hw.dsky.display.verb = [0, 0];
            hw.dsky.display.noun = [0, 0];
            hw.dsky.display.lights.prog_alarm = false;
            *mode = None;
            buf.clear();
            hw.log.warn("RSET — display cleared");
        }
        DskyKey::ProceED => hw.log.info("PRO"),
        DskyKey::KeyRel => hw.log.info("KEY REL"),
        _ => {}
    }
}

fn push_digit(hw: &mut SimHardware, mode: &mut Option<&'static str>, buf: &mut String, d: u8) {
    if buf.len() < 2 {
        buf.push((b'0' + d) as char);
        let label = mode.unwrap_or("?");
        hw.log.io(format!("{} ← {} (buf: {})", label, d, buf));
    }
}
