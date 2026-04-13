//! `agc_sim` binary — ratatui TUI main loop for the AGC simulator.
//!
//! Usage: `agc_sim [--scenario {launch,burn,free}]`
//!
//! Key bindings:
//!   q / Ctrl-C          — quit
//!   0-9, v, n, Enter    — DSKY numeric keys / VERB / NOUN / ENTER
//!   c, r, +, -          — CLR / KEY REL / plus / minus
//!   F1, F2, F3          — switch scenario (launch / burn / free)
//!   =                   — increase time multiplier (up to 16×)
//!   _                   — decrease time multiplier (down to 0.25×)

use std::{
    io::{self, IsTerminal},
    time::Duration,
};

use agc_core::{
    navigation::state_vector::StateVector,
    services::v_n::VnState,
    types::{Met, Vec3},
    AgcState,
};
use agc_sim::{command_dispatch::handle_key_event, DispatchOutcome, SimHardware};
use crossterm::{
    event::{self, Event},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{backend::CrosstermBackend, Terminal};

/// Scenario selected via `--scenario`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scenario {
    Launch,
    Burn,
    Free,
}

impl Scenario {
    /// Human-readable label for the log.
    fn label(self) -> &'static str {
        match self {
            Scenario::Launch => "launch",
            Scenario::Burn => "burn",
            Scenario::Free => "free",
        }
    }
}

/// Parse `--scenario {launch,burn,free}` from `std::env::args`.
///
/// Returns `Scenario::Free` if the flag is absent or the value is unrecognised.
fn parse_scenario() -> Scenario {
    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        if args[i] == "--scenario" {
            if let Some(val) = args.get(i + 1) {
                return match val.as_str() {
                    "launch" => Scenario::Launch,
                    "burn" => Scenario::Burn,
                    "free" => Scenario::Free,
                    other => {
                        eprintln!(
                            "agc_sim: unknown scenario '{}'; defaulting to 'free'",
                            other
                        );
                        Scenario::Free
                    }
                };
            }
        }
        i += 1;
    }
    Scenario::Free
}

/// Initialise `SimHardware` for the 'launch' scenario.
///
/// Seeds P11 Earth orbit insertion monitor with a LEO state vector.
/// A 200 km circular LEO: r = 6571 km, v ≈ 7784 m/s prograde.
///
/// AGC source: Comanche055/P11.agc — VHHDOT display, TLIFTOFF.
fn init_launch(hw: &mut SimHardware) {
    hw.agc_state = AgcState::new();
    hw.vn = VnState::new();

    // Seed a 200 km circular LEO state vector (ECI, metres/m/s).
    let r_leo: Vec3 = [6_571_000.0, 0.0, 0.0]; // 200 km alt
    let v_leo: Vec3 = [0.0, 7_784.0, 0.0]; // ~circular LEO velocity
    hw.agc_state.nav.sv = StateVector::new(r_leo, v_leo, Met(0));

    // Enter P11.
    hw.agc_state.modreg = 11;
    hw.dsky.display.prog = Some(11);
    hw.dsky.display.verb = None;
    hw.dsky.display.noun = None;
    hw.log
        .info("SCENARIO: launch — P11 Earth orbit monitor, LEO 200 km seeded");
}

/// Initialise `SimHardware` for the 'burn' scenario.
///
/// Seeds P40 with a 50 m/s prograde delta-V target via V37N40.
/// Uses the same LEO state vector as the launch scenario.
///
/// AGC source: Comanche055/P40-P47.agc — SPS burn execution.
fn init_burn(hw: &mut SimHardware) {
    hw.agc_state = AgcState::new();
    hw.vn = VnState::new();

    let r_leo: Vec3 = [6_571_000.0, 0.0, 0.0];
    let v_leo: Vec3 = [0.0, 7_784.0, 0.0];
    hw.agc_state.nav.sv = StateVector::new(r_leo, v_leo, Met(0));

    // 50 m/s prograde delta-V in LVLH (velocity direction).
    hw.agc_state.nav.delvslv = [0.0, 50.0, 0.0];
    hw.agc_state.nav.vgdisp = 50.0;

    // Enter P40.
    hw.agc_state.modreg = 40;
    hw.dsky.display.prog = Some(40);
    hw.dsky.display.verb = Some(6);
    hw.dsky.display.noun = Some(40);
    hw.log
        .info("SCENARIO: burn — P40 50 m/s prograde SPS burn auto-armed");
}

/// Initialise `SimHardware` for the 'free' scenario.
///
/// Minimal P00 idle, freeform keyboard input.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc POOH.
fn init_free(hw: &mut SimHardware) {
    hw.agc_state = AgcState::new();
    hw.vn = VnState::new();

    // Enter P00: set modreg to 0 and show "00" in PROG.
    // AGC source: Comanche055/FRESH_START_AND_RESTART.agc POOH.
    hw.agc_state.modreg = 0;
    hw.agc_state.restart.set_phase(1, 4, false, 0);
    hw.dsky.display.prog = Some(0);
    hw.dsky.display.verb = None;
    hw.dsky.display.noun = None;
    hw.log.info("SCENARIO: free — P00 idle, freeform mode");
}

/// Apply the selected scenario to `SimHardware`.
fn apply_scenario(scenario: Scenario, hw: &mut SimHardware) {
    match scenario {
        Scenario::Launch => init_launch(hw),
        Scenario::Burn => init_burn(hw),
        Scenario::Free => init_free(hw),
    }
}

fn main() -> io::Result<()> {
    // If stdin is not a TTY (CI / pipe), print a friendly message and exit.
    if !io::stdin().is_terminal() {
        let scenario = parse_scenario();
        println!(
            "agc_sim: stdin is not a TTY — headless mode, scenario='{}', exiting cleanly.",
            scenario.label()
        );
        return Ok(());
    }

    let scenario = parse_scenario();

    // ── Hardware and log ──────────────────────────────────────────────────────
    let mut hw = SimHardware::new_headless();
    apply_scenario(scenario, &mut hw);
    hw.log
        .info(format!("agc_sim started — scenario: {}", scenario.label()));

    // ── Terminal setup ────────────────────────────────────────────────────────
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;

    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;
    terminal.clear()?;

    // ── Main render loop (~20 Hz) ─────────────────────────────────────────────
    let tick = Duration::from_millis(50); // 20 Hz
    let result = run_loop(&mut terminal, &mut hw, tick);

    // ── Terminal teardown (always runs, even on error) ────────────────────────
    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;

    if let Err(e) = result {
        eprintln!("agc_sim error: {e}");
        return Err(e);
    }

    println!("agc_sim: exited cleanly.");
    Ok(())
}

/// Run the main event loop until the user requests quit or an error occurs.
fn run_loop(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    hw: &mut SimHardware,
    tick: Duration,
) -> io::Result<()> {
    let mut current_scenario = parse_scenario();

    loop {
        // Render frame.
        terminal.draw(|frame| {
            agc_sim::dsky_terminal::render(frame, hw);
        })?;

        // Poll for events with a timeout equal to one tick.
        if event::poll(tick)? {
            match event::read()? {
                Event::Key(key_event) => match handle_key_event(key_event, hw) {
                    DispatchOutcome::Quit => return Ok(()),
                    DispatchOutcome::KeyQueued(k) => {
                        hw.log.info(format!("DSKY key: {k:?}"));
                    }
                    DispatchOutcome::Continue => {
                        // Check for F-key scenario switches.
                        // The F-key presses are logged in handle_key_event; we check
                        // the last log entry to detect them (simple approach).
                        if let Some(last) = hw.log.lines().last() {
                            if last.text.contains("scenario: launch")
                                && current_scenario != Scenario::Launch
                            {
                                current_scenario = Scenario::Launch;
                                init_launch(hw);
                            } else if last.text.contains("scenario: burn")
                                && current_scenario != Scenario::Burn
                            {
                                current_scenario = Scenario::Burn;
                                init_burn(hw);
                            } else if last.text.contains("scenario: free")
                                && current_scenario != Scenario::Free
                            {
                                current_scenario = Scenario::Free;
                                init_free(hw);
                            }
                        }
                    }
                    DispatchOutcome::Unhandled => {}
                },
                // Resize events are handled automatically by ratatui on the
                // next draw call; no explicit action needed.
                Event::Resize(_, _) => {}
                _ => {}
            }
        }
    }
}
