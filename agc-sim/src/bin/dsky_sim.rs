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

use agc_core::hal::Timers as _;
use agc_core::services::pinball::decode_dsky;
use agc_core::services::v_n::feed_key;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::dsky_ui::{key_from_code, render, PropulsionFrame};
use agc_sim::hardware::SimHardware;

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
    let mut hw = SimHardware::new();
    let mut last_frame = Instant::now();
    let mut flash_on = true;
    let mut last_flash = Instant::now();
    let mut status = String::from("Ready");

    loop {
        // Read MET from the HAL timer (single source of truth).
        state.time = Met(hw.timers.mission_time());

        // Drain any pending keyboard events.  Snapshot state.time before
        // processing keys so we can detect if a noun commit (V25 N36/N65/N24)
        // changed it.
        let time_before_keys = state.time;
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

        // If a noun commit changed state.time, rebase the HAL timer.
        if state.time != time_before_keys {
            hw.timers.set_time(state.time.0);
        }

        // Toggle VERB/NOUN flashing.
        if last_flash.elapsed() >= FLASH_PERIOD {
            flash_on = !flash_on;
            last_flash = Instant::now();
        }

        // Redraw at ~20 Hz.
        if last_frame.elapsed() >= FRAME {
            agc_core::services::v_n::refresh_monitor_display(&mut state);
            let frame = decode_dsky(&state.dsky);

            // Build propulsion frame from hardware state.
            let (vis_sm, vis_cm) = hw.rcs.drain_visual();
            let prop = PropulsionFrame {
                sm_jets: vis_sm,
                cm_jets: vis_cm,
                sps_thrusting: hw.engine.thrusting,
                gimbal_pitch_deg: hw.engine.gimbal_pitch as f32 * (360.0 / 3200.0),
                gimbal_yaw_deg: hw.engine.gimbal_yaw as f32 * (360.0 / 3200.0),
            };

            render(out, (1, 1), &frame, Some(&prop), state.time.0 as u64, &status, flash_on)?;
            last_frame = Instant::now();
        }

        // Brief sleep to avoid pegging a core.
        std::thread::sleep(Duration::from_millis(5));
    }
}
