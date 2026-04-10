//! Unified AGC simulator — DSKY + navigation + mission timeline.
//!
//! Run with:
//!   cargo run -p agc-sim --bin agc_sim
//!   cargo run -p agc-sim --bin agc_sim -- --scenario launch
//!   cargo run -p agc-sim --bin agc_sim -- --scenario burn
//!   cargo run -p agc-sim --bin agc_sim -- --scenario free

use std::io;
use std::time::Duration;

use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{backend::CrosstermBackend, Terminal};

use agc_core::guidance::maneuver::BurnState;
use agc_core::hal::dsky::DskyKey;
use agc_core::navigation::state_vector::Refsmmat;
use agc_core::services::average_g::{average_g, AverageGState, CYCLE_DT};

use agc_sim::{
    command_dispatch::{dispatch, CommandAction},
    mission::{Mission, Scenario},
    noun_display::registers_for,
    unified_terminal::{self, build_dsky_display, UnifiedSnapshot},
    SimHardware,
};

// ── Local VerbNoun state machine ──────────────────────────────────────────────

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
    /// Invalid entry.
    Error,
    /// No action of interest to the outer loop.
    None,
}

/// Verb/Noun digit-entry state machine.
pub struct VerbNounState {
    pub prog: u8,
    pub verb: u8,
    pub noun: u8,
    pub flash: bool,

    field: Option<EntryField>,
    buf: [u8; 2],
    buf_len: u8,
}

impl Default for VerbNounState {
    fn default() -> Self {
        Self::new()
    }
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

    pub fn set_prog(&mut self, p: u8) {
        self.prog = p;
    }

    pub fn flash_tick(&mut self) {
        self.flash = !self.flash;
    }

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
                        let partial =
                            self.buf[0] * 10 + if self.buf_len == 2 { self.buf[1] } else { 0 };
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

// ── Key mapping ───────────────────────────────────────────────────────────────

fn map_key(code: KeyCode) -> Option<DskyKey> {
    Some(match code {
        KeyCode::Char('v') => DskyKey::Verb,
        KeyCode::Char('n') => DskyKey::Noun,
        KeyCode::Enter => DskyKey::Enter,
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

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() -> io::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let scenario = args
        .iter()
        .position(|a| a == "--scenario")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| Scenario::parse(s))
        .unwrap_or(Scenario::FreeFlight);

    std::panic::set_hook(Box::new(|info| {
        let _ = disable_raw_mode();
        let _ = execute!(io::stdout(), LeaveAlternateScreen);
        eprintln!("\r\nPanic: {info}");
    }));

    let result = run(scenario);
    let _ = disable_raw_mode();
    let _ = execute!(io::stdout(), LeaveAlternateScreen);
    result
}

fn run(scenario: Scenario) -> io::Result<()> {
    let mut hw = SimHardware::new();
    let mut mission = Mission::new(scenario);
    let mut vn = VerbNounState::new();
    vn.set_prog(mission.active_prog);
    if mission.default_verb != 0 {
        vn.verb = mission.default_verb;
    }
    if mission.default_noun != 0 {
        vn.noun = mission.default_noun;
    }

    let mut avg_g = AverageGState::new();
    let mut burn: Option<BurnState> = None;
    let mut total_dv_ms = 0.0_f64;
    let mut lamp_test_frames = 0u32;
    let mut flash_counter = 0u32;
    let mut comp_acty_counter = 0u32;
    let mut time_factor = 1u32;

    hw.log
        .info(format!("agc-sim started — scenario: {}", scenario.label()));
    hw.log
        .info("V/N keys: VERB, NOUN, 0-9, ENTER, CLR, PRO, RSET, KEY REL");
    hw.log
        .info("F1=Launch F2=Burn F3=Free  [+]/[-] time×  [Q]uit");

    enable_raw_mode()?;
    execute!(io::stdout(), EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;

    loop {
        // 1. Advance navigation by `time_factor` SERVICER cycles
        for _ in 0..time_factor {
            let pipa_counts: [i16; 3] = if let Some(ref bs) = burn {
                if !bs.complete {
                    let vg_mag = bs.vg_magnitude();
                    if vg_mag > 0.01 {
                        let accel = 91_188.544_f64 / 20_000.0;
                        let dv_cycle = accel * CYCLE_DT;
                        let counts_per_ms = 1.0 / 0.0585;
                        let ux = bs.vg[0] / vg_mag;
                        let uy = bs.vg[1] / vg_mag;
                        let uz = bs.vg[2] / vg_mag;
                        [
                            (ux * dv_cycle * counts_per_ms) as i16,
                            (uy * dv_cycle * counts_per_ms) as i16,
                            (uz * dv_cycle * counts_per_ms) as i16,
                        ]
                    } else {
                        [0; 3]
                    }
                } else {
                    [0; 3]
                }
            } else {
                [0; 3]
            };

            let result = average_g(
                &mission.sv,
                pipa_counts,
                &Refsmmat::IDENTITY,
                CYCLE_DT,
                &mut avg_g,
            );
            mission.sv = result.sv;

            if let Some(ref mut bs) = burn {
                if !bs.complete {
                    let dv = result.delta_v_total.0;
                    bs.update_vg(&dv);
                    let mag = (dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]).sqrt();
                    total_dv_ms += mag;
                    if bs.complete {
                        hw.log
                            .info(format!("BURN complete — ΣΔV = {:.3} m/s", total_dv_ms));
                        mission.active_prog = 0;
                        vn.set_prog(0);
                    }
                }
            }
        }

        // 2. Populate DSKY registers from Noun
        let regs = registers_for(vn.noun, &mission, None);

        // 3. Build DskyDisplayState
        let dsky = build_dsky_display(
            vn.prog,
            vn.verb,
            vn.noun,
            &regs,
            &hw.dsky.display.lights,
            lamp_test_frames > 0,
            comp_acty_counter,
        );

        // 4. Build UnifiedSnapshot
        let met_s = mission.sv.t.0 as f64 / 100.0;
        let orbit_fraction = (met_s % mission.period_s) / mission.period_s;
        let vg_mag = burn
            .as_ref()
            .filter(|b| !b.complete)
            .map(|b| b.vg_magnitude());
        let tgo_s = vg_mag.map(|m| m / (91_188.544_f64 / 20_000.0));
        let dap_mode: &'static str = if burn.as_ref().is_some_and(|b| !b.complete) {
            "BURN"
        } else {
            "FREE"
        };
        let engine_firing = burn.as_ref().is_some_and(|b| !b.complete);

        let snap = UnifiedSnapshot {
            dsky: &dsky,
            mission: &mission,
            log: &hw.log,
            dap_mode,
            engine_firing,
            vg_mag,
            tgo_s,
            total_dv_ms,
            orbit_fraction,
        };

        terminal.draw(|f| unified_terminal::render(f, &snap))?;

        // 5. Tick helpers
        flash_counter += 1;
        if flash_counter >= 10 {
            vn.flash_tick();
            flash_counter = 0;
        }
        comp_acty_counter = comp_acty_counter.wrapping_add(1);
        lamp_test_frames = lamp_test_frames.saturating_sub(1);

        // 6. Poll input
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(k) = event::read()? {
                if k.kind != KeyEventKind::Press {
                    continue;
                }
                match k.code {
                    KeyCode::Esc | KeyCode::Char('q') | KeyCode::Char('Q') => break,
                    KeyCode::F(1) => {
                        mission = Mission::new(Scenario::LaunchMonitor);
                        vn = VerbNounState::new();
                        vn.set_prog(mission.active_prog);
                        if mission.default_verb != 0 {
                            vn.verb = mission.default_verb;
                        }
                        if mission.default_noun != 0 {
                            vn.noun = mission.default_noun;
                        }
                        burn = None;
                        total_dv_ms = 0.0;
                        avg_g = AverageGState::new();
                        hw.log.info("F1 — Launch Monitor scenario loaded");
                    }
                    KeyCode::F(2) => {
                        mission = Mission::new(Scenario::TargetedBurn);
                        vn = VerbNounState::new();
                        vn.set_prog(mission.active_prog);
                        burn = None;
                        total_dv_ms = 0.0;
                        avg_g = AverageGState::new();
                        hw.log.info("F2 — Targeted Burn scenario loaded");
                    }
                    KeyCode::F(3) => {
                        mission = Mission::new(Scenario::FreeFlight);
                        vn = VerbNounState::new();
                        vn.set_prog(mission.active_prog);
                        burn = None;
                        total_dv_ms = 0.0;
                        avg_g = AverageGState::new();
                        hw.log.info("F3 — Free Flight scenario loaded");
                    }
                    KeyCode::Char('+') | KeyCode::Char('=') => {
                        time_factor = (time_factor * 2).min(512);
                        hw.log.info(format!("time× {}", time_factor));
                    }
                    KeyCode::Char('-') => {
                        time_factor = (time_factor / 2).max(1);
                        hw.log.info(format!("time× {}", time_factor));
                    }
                    code => {
                        if let Some(dsky_key) = map_key(code) {
                            hw.dsky_push_key(dsky_key);
                            let action = vn.process_key(dsky_key);
                            match action {
                                VerbNounAction::Execute { verb, noun } => {
                                    if let CommandAction::LampTest =
                                        dispatch(verb, noun, &mut mission, &mut burn, &mut hw.log)
                                    {
                                        lamp_test_frames = 100;
                                    }
                                    vn.set_prog(mission.active_prog);
                                }
                                VerbNounAction::Proceed => hw.log.info("PRO — Proceed"),
                                VerbNounAction::Reset => {
                                    hw.dsky.display.lights.prog_alarm = false;
                                    hw.dsky.display.lights.opr_err = false;
                                    hw.log.warn("RSET — alarms cleared");
                                }
                                VerbNounAction::Error => {
                                    hw.dsky.display.lights.opr_err = true;
                                    hw.log.warn("OPR ERR");
                                }
                                VerbNounAction::None => {}
                            }
                        }
                    }
                }
            }
        }
    }

    Ok(())
}
