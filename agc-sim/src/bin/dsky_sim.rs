//! Interactive terminal DSKY simulator.
//!
//! Runs a host-side copy of `agc_core::AgcState`, drives the V/N
//! processor with real keyboard input, and redraws the DSKY panel
//! at ~20 Hz. Mission Elapsed Time (MET) advances from wall clock.
//!
//! Usage:
//! ```text
//!   cargo run -p agc-sim --bin dsky_sim
//! ```
//!
//! Key bindings: see the status line. `q` or `Ctrl-C` to quit.

use std::io::{self, Write};
use std::time::{Duration, Instant};

use agc_core::services::pinball::decode_dsky;
use agc_core::services::v_n::feed_key;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::dsky_ui::{key_from_code, render};

use crossterm::{
    cursor::{Hide, Show},
    event::{self, Event, KeyCode, KeyEvent, KeyModifiers},
    execute,
    terminal::{
        disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
    },
};

/// Render cadence (approximately 20 Hz).
const FRAME: Duration = Duration::from_millis(50);

/// Flash toggle period (VERB/NOUN blink).
const FLASH_PERIOD: Duration = Duration::from_millis(500);

fn main() -> io::Result<()> {
    let mut stdout = io::stdout();
    enable_raw_mode()?;
    execute!(stdout, EnterAlternateScreen, Hide)?;

    let result = run(&mut stdout);

    // Always restore the terminal even on error.
    execute!(stdout, Show, LeaveAlternateScreen)?;
    disable_raw_mode()?;
    result
}

fn run<W: Write>(out: &mut W) -> io::Result<()> {
    let mut state = AgcState::new();
    let start = Instant::now();
    let mut last_frame = Instant::now();
    let mut flash_on = true;
    let mut last_flash = Instant::now();
    let mut status = String::from("Ready");

    loop {
        // Drain any pending keyboard events.
        while event::poll(Duration::from_millis(0))? {
            if let Event::Key(KeyEvent {
                code, modifiers, ..
            }) = event::read()?
            {
                // Ctrl-C quits.
                if code == KeyCode::Char('c') && modifiers.contains(KeyModifiers::CONTROL) {
                    return Ok(());
                }
                // Plain 'q' quits (but allow 'Q' → RSET? No — 'r' is RSET, 'q' is quit).
                if matches!(code, KeyCode::Char('q') | KeyCode::Char('Q')) {
                    return Ok(());
                }
                if let Some(key) = key_from_code(code) {
                    feed_key(&mut state, key);
                    status = format!("Key: {:?}", key);
                } else if code == KeyCode::Esc {
                    return Ok(());
                }
            }
        }

        // Advance MET from wall clock.
        let elapsed_cs = (start.elapsed().as_millis() / 10) as u32;
        state.time = Met(elapsed_cs);

        // Toggle VERB/NOUN flashing.
        if last_flash.elapsed() >= FLASH_PERIOD {
            flash_on = !flash_on;
            last_flash = Instant::now();
        }

        // Redraw at ~20 Hz.
        if last_frame.elapsed() >= FRAME {
            let frame = decode_dsky(&state.dsky);
            render(out, (1, 1), &frame, elapsed_cs as u64, &status, flash_on)?;
            last_frame = Instant::now();
        }

        // Brief sleep to avoid pegging a core.
        std::thread::sleep(Duration::from_millis(5));
    }
}
