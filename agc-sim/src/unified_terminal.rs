//! Unified three-panel TUI: DSKY | Mission | Log (bottom).
//!
//! Uses only BOLD / DIM / REVERSED modifiers — terminal-agnostic.

use ratatui::{
    layout::{Constraint, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph},
    Frame,
};

use crate::dsky_state::{DataRegister, DskyDisplayState, Sign};
use crate::mission::{Mission, Scenario};
use crate::sim_log::{LogLevel, SimLog};

// ── Shared styles ─────────────────────────────────────────────────────────────

fn normal() -> Style {
    Style::default()
}

fn bold() -> Style {
    Style::default().add_modifier(Modifier::BOLD)
}

fn dim() -> Style {
    Style::default().add_modifier(Modifier::DIM)
}

fn lit() -> Style {
    Style::default()
        .add_modifier(Modifier::REVERSED)
        .add_modifier(Modifier::BOLD)
}

// ── Public snapshot type ──────────────────────────────────────────────────────

/// Snapshot passed to the unified renderer each frame.
pub struct UnifiedSnapshot<'a> {
    pub dsky: &'a DskyDisplayState,
    pub mission: &'a Mission,
    pub log: &'a SimLog,
    /// DAP mode label: "FREE" / "HOLD" / "BURN" / "MANV".
    pub dap_mode: &'static str,
    /// Whether engine is firing.
    pub engine_firing: bool,
    /// VG magnitude (m/s), None if no burn.
    pub vg_mag: Option<f64>,
    /// TGO estimate (s).
    pub tgo_s: Option<f64>,
    /// Cumulative delta-V (m/s).
    pub total_dv_ms: f64,
    /// Orbit fraction [0..1).
    pub orbit_fraction: f64,
}

// ── Top-level entry ───────────────────────────────────────────────────────────

pub fn render(f: &mut Frame, snap: &UnifiedSnapshot) {
    let area = f.area();
    // Compact layout — works on ~16-row VS Code terminals.
    // Log gets 5 rows; top panels share the rest.
    let log_rows = if area.height >= 22 { 7 } else { 5 };
    let rows =
        Layout::vertical([Constraint::Min(10), Constraint::Length(log_rows)]).split(area);
    // DSKY left (44 cols), mission right (rest)
    let cols = Layout::horizontal([Constraint::Length(44), Constraint::Min(54)]).split(rows[0]);

    render_dsky_panel(f, snap.dsky, cols[0]);
    render_mission_panel(f, snap, cols[1]);
    render_log_panel(f, snap.log, rows[1]);
}

// ── DSKY panel ────────────────────────────────────────────────────────────────

fn render_dsky_panel(f: &mut Frame, d: &DskyDisplayState, area: Rect) {
    let outer = Block::default()
        .title(" APOLLO GUIDANCE COMPUTER ")
        .borders(Borders::ALL)
        .border_style(bold());
    let inner = outer.inner(area);
    f.render_widget(outer, area);

    // Compact layout: 9 content rows fit in ~11-row terminals.
    let rows = Layout::vertical([
        Constraint::Length(1), // [0] lights row 1: UPLNK TEMP GBL NO-ATT
        Constraint::Length(1), // [1] lights row 2: STBY PROG KEY-R RSTRT OPR-E TRKR
        Constraint::Length(1), // [2] COMP ACTY + PROG
        Constraint::Length(1), // [3] VERB + NOUN
        Constraint::Length(1), // [4] R1
        Constraint::Length(1), // [5] R2
        Constraint::Length(1), // [6] R3
        Constraint::Length(1), // [7] key hints row 1
        Constraint::Length(1), // [8] key hints row 2
        Constraint::Min(0),
    ])
    .split(inner);

    render_light_row_1(f, &d.lights, rows[0]);
    render_light_row_2(f, &d.lights, rows[1]);
    render_comp_prog(f, d, rows[2]);
    render_verb_noun(f, d, rows[3]);
    render_reg_line(f, "R1", &d.r1, rows[4]);
    render_reg_line(f, "R2", &d.r2, rows[5]);
    render_reg_line(f, "R3", &d.r3, rows[6]);
    render_dsky_keys1(f, rows[7]);
    render_dsky_keys2(f, rows[8]);
}

fn light_span(label: &'static str, on: bool) -> Span<'static> {
    if on {
        Span::styled(format!("[{label}] "), lit())
    } else {
        Span::styled(format!(" {label}  "), dim())
    }
}

fn render_light_row_1(f: &mut Frame, l: &crate::dsky_state::Lights, area: Rect) {
    let line = Line::from(vec![
        light_span("UPLNK", l.uplink_acty),
        light_span("TEMP", l.temp),
        light_span("GBL", l.gimbal_lock),
        light_span("NOATT", l.no_att),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_light_row_2(f: &mut Frame, l: &crate::dsky_state::Lights, area: Rect) {
    let line = Line::from(vec![
        light_span("STBY", l.stby),
        light_span("PROG", l.prog_alarm),
        light_span("KREL", l.key_rel),
        light_span("RSTR", l.restart),
        light_span("OPER", l.opr_err),
        light_span("TRKR", l.tracker),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn fmt_pair(digits: [u8; 2]) -> String {
    digits
        .iter()
        .map(|&d| if d > 9 { ' ' } else { (b'0' + d) as char })
        .collect()
}

fn render_comp_prog(f: &mut Frame, d: &DskyDisplayState, area: Rect) {
    // COMP ACTY indicator + inline PROG display.
    let comp = if d.lights.comp_acty { "●" } else { " " };
    let comp_style = if d.lights.comp_acty { lit() } else { dim() };
    let line = Line::from(vec![
        Span::styled("COMP ACTY ", dim()),
        Span::styled(format!("[{comp}]"), comp_style),
        Span::styled("    PROG  ", dim()),
        Span::styled(fmt_pair(d.prog), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_verb_noun(f: &mut Frame, d: &DskyDisplayState, area: Rect) {
    let line = Line::from(vec![
        Span::styled("VERB ", dim()),
        Span::styled(fmt_pair(d.verb), bold()),
        Span::styled("          NOUN  ", dim()),
        Span::styled(fmt_pair(d.noun), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_reg_line(f: &mut Frame, label: &'static str, reg: &DataRegister, area: Rect) {
    let line = Line::from(vec![
        Span::styled(format!("{label}  "), dim()),
        Span::styled(reg.to_display_string(), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_dsky_keys1(f: &mut Frame, area: Rect) {
    f.render_widget(
        Paragraph::new(Span::styled("[V]erb [N]oun [0-9] [+/-]", normal())),
        area,
    );
}

fn render_dsky_keys2(f: &mut Frame, area: Rect) {
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled("[Enter] [Clr] [P]ro [R]set  ", normal()),
            Span::styled("[K]eyRel  [Q]uit", dim()),
        ])),
        area,
    );
}

// ── Mission panel ─────────────────────────────────────────────────────────────

fn render_mission_panel(f: &mut Frame, snap: &UnifiedSnapshot, area: Rect) {
    let outer = Block::default()
        .title(" MISSION STATE ")
        .borders(Borders::ALL)
        .border_style(bold());
    let inner = outer.inner(area);
    f.render_widget(outer, area);

    // 14 content rows — single-line everything, no nested borders.
    let rows = Layout::vertical([
        Constraint::Length(1), // [0]  MET + PHASE
        Constraint::Length(1), // [1]  SCEN
        Constraint::Length(1), // [2]  X position/velocity
        Constraint::Length(1), // [3]  Y position/velocity
        Constraint::Length(1), // [4]  Z position/velocity
        Constraint::Length(1), // [5]  ALT/SPD
        Constraint::Length(1), // [6]  SMA/ECC
        Constraint::Length(1), // [7]  APO/PER
        Constraint::Length(1), // [8]  ORB progress + period
        Constraint::Length(1), // [9]  DAP + ENG
        Constraint::Length(1), // [10] VG / TGO / ΣΔV
        Constraint::Length(1), // [11] scenario F1
        Constraint::Length(1), // [12] scenario F2
        Constraint::Length(1), // [13] scenario F3
        Constraint::Min(0),
    ])
    .split(inner);

    // Row 0: MET + PHASE on one line
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled("MET ", dim()),
            Span::styled(snap.mission.format_met(), bold()),
            Span::styled("  ", normal()),
            Span::styled(snap.mission.phase_label(), bold()),
        ])),
        rows[0],
    );

    // Row 1: SCEN
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled("SCEN ", dim()),
            Span::styled(snap.mission.scenario.label(), normal()),
        ])),
        rows[1],
    );

    // Rows 2-4: X / Y / Z position + velocity
    render_xyz_row(f, "X", snap.mission.sv.r[0], snap.mission.sv.v[0], rows[2]);
    render_xyz_row(f, "Y", snap.mission.sv.r[1], snap.mission.sv.v[1], rows[3]);
    render_xyz_row(f, "Z", snap.mission.sv.r[2], snap.mission.sv.v[2], rows[4]);

    // Rows 5-8: derived orbital quantities
    render_alt_spd(f, snap.mission, rows[5]);
    render_sma_ecc(f, snap.mission, rows[6]);
    render_apo_per(f, snap.mission, rows[7]);
    render_orb_progress(f, snap, rows[8]);

    // Rows 9-10: DAP / burn state (no box)
    render_dap_line(f, snap, rows[9]);
    render_burn_line(f, snap, rows[10]);

    // Rows 11-13: scenarios (no box)
    let active = snap.mission.scenario;
    render_scenario_line(f, Scenario::LaunchMonitor, "[F1] Launch Monitor", active, rows[11]);
    render_scenario_line(f, Scenario::TargetedBurn, "[F2] Targeted Burn ", active, rows[12]);
    render_scenario_line(f, Scenario::FreeFlight, "[F3] Free Flight   ", active, rows[13]);
}

fn render_xyz_row(f: &mut Frame, axis: &'static str, r: f64, v: f64, area: Rect) {
    let line = Line::from(vec![
        Span::styled(format!("{axis} "), dim()),
        Span::styled(format!("{:+11.3} km", r / 1000.0), bold()),
        Span::styled("  ", normal()),
        Span::styled(format!("{:+10.3} m/s", v), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_dap_line(f: &mut Frame, snap: &UnifiedSnapshot, area: Rect) {
    let eng_span = if snap.engine_firing {
        Span::styled("ENG [● FIRING]", lit())
    } else {
        Span::styled("ENG [ OFF  ]  ", dim())
    };
    let dap_style = if snap.engine_firing { lit() } else { bold() };
    let line = Line::from(vec![
        Span::styled("DAP ", dim()),
        Span::styled(format!("[{:^5}]", snap.dap_mode), dap_style),
        Span::styled("  ", normal()),
        eng_span,
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_burn_line(f: &mut Frame, snap: &UnifiedSnapshot, area: Rect) {
    let vg_str = match snap.vg_mag {
        Some(m) => format!("{:+7.2}", m),
        None => "  ----".to_string(),
    };
    let tgo_str = match snap.tgo_s {
        Some(t) => format!("{:4.1}", t),
        None => " ---".to_string(),
    };
    let line = Line::from(vec![
        Span::styled("VG ", dim()),
        Span::styled(format!("{vg_str} m/s"), bold()),
        Span::styled("  TGO ", dim()),
        Span::styled(format!("{tgo_str} s"), bold()),
        Span::styled("  ΣΔV ", dim()),
        Span::styled(format!("{:+7.2} m/s", snap.total_dv_ms), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_scenario_line(
    f: &mut Frame,
    this: Scenario,
    label: &'static str,
    active: Scenario,
    area: Rect,
) {
    let line = if this == active {
        Line::from(vec![
            Span::styled(label, bold()),
            Span::styled("  \u{2190} active", lit()),
        ])
    } else {
        Line::from(Span::styled(label, dim()))
    };
    f.render_widget(Paragraph::new(line), area);
}

fn render_alt_spd(f: &mut Frame, mission: &Mission, area: Rect) {
    use agc_core::navigation::gravity::RE_EARTH;
    let r = &mission.sv.r;
    let v = &mission.sv.v;
    let r_mag = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
    let speed = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let alt_km = (r_mag - RE_EARTH) / 1000.0;
    let line = Line::from(vec![
        Span::styled("ALT ", dim()),
        Span::styled(format!("{:+8.3} km", alt_km), bold()),
        Span::styled("  SPD ", dim()),
        Span::styled(format!("{:.3} m/s", speed), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_sma_ecc(f: &mut Frame, mission: &Mission, area: Rect) {
    use agc_core::navigation::conics::rv_to_elements;
    use agc_core::navigation::gravity::MU_EARTH;
    let el = rv_to_elements(&mission.sv.r, &mission.sv.v, MU_EARTH);
    let line = Line::from(vec![
        Span::styled("SMA ", dim()),
        Span::styled(format!("{:8.1} km", el.sma / 1000.0), bold()),
        Span::styled("   ECC ", dim()),
        Span::styled(format!("{:.4}", el.ecc), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_apo_per(f: &mut Frame, mission: &Mission, area: Rect) {
    use agc_core::navigation::conics::{apsides, rv_to_elements};
    use agc_core::navigation::gravity::{MU_EARTH, RE_EARTH};
    let el = rv_to_elements(&mission.sv.r, &mission.sv.v, MU_EARTH);
    let (peri_r, apo_r) = apsides(el.sma, el.ecc);
    let apo_km = (apo_r - RE_EARTH) / 1000.0;
    let per_km = (peri_r - RE_EARTH) / 1000.0;
    let apo_str = if el.ecc >= 1.0 || !apo_km.is_finite() {
        "     ---".to_string()
    } else {
        format!("{:+8.1}", apo_km)
    };
    let line = Line::from(vec![
        Span::styled("APO ", dim()),
        Span::styled(format!("{} km", apo_str), bold()),
        Span::styled("  PER ", dim()),
        Span::styled(format!("{:+8.1} km", per_km), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_orb_progress(f: &mut Frame, snap: &UnifiedSnapshot, area: Rect) {
    let pct = snap.orbit_fraction * 100.0;
    let bar_width = 14usize;
    let filled = (snap.orbit_fraction * bar_width as f64) as usize;
    let bar: String = (0..bar_width)
        .map(|i| if i < filled { '█' } else { '░' })
        .collect();
    let line = Line::from(vec![
        Span::styled("ORB ", dim()),
        Span::styled(bar, normal()),
        Span::styled(format!(" {:5.1}%", pct), bold()),
        Span::styled("  T ", dim()),
        Span::styled(format!("{:.0}s", snap.mission.period_s), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

// ── Log panel ─────────────────────────────────────────────────────────────────

fn render_log_panel(f: &mut Frame, log: &SimLog, area: Rect) {
    let outer = Block::default()
        .title(" MISSION LOG ")
        .borders(Borders::ALL)
        .border_style(dim());
    let inner = outer.inner(area);
    f.render_widget(outer, area);

    let visible = inner.height as usize;
    let items: Vec<ListItem> = log
        .tail(visible)
        .iter()
        .map(|e| {
            let level_style = match e.level {
                LogLevel::Alarm => Style::default()
                    .add_modifier(Modifier::BOLD)
                    .add_modifier(Modifier::REVERSED),
                LogLevel::Warn => Style::default().add_modifier(Modifier::BOLD),
                LogLevel::Io => Style::default(),
                LogLevel::Info => dim(),
            };
            ListItem::new(Line::from(vec![
                Span::styled(format!("{:>5} ", e.tick), dim()),
                Span::styled(format!("{} ", e.level.label()), level_style),
                Span::styled(e.message.clone(), normal()),
            ]))
        })
        .collect();

    f.render_widget(List::new(items), inner);
}

// ── Helper: build DskyDisplayState from VN state + computed regs ──────────────

use crate::dsky_state::Lights;
use crate::noun_display::RegValue;

/// Build a `DskyDisplayState` snapshot from the current VN state and registers.
///
/// When `lamp_test` is true all lights are forced on.
/// `comp_acty_counter % 10 < 5` drives the blinking comp-acty indicator.
pub fn build_dsky_display(
    prog: u8,
    verb: u8,
    noun: u8,
    regs: &[RegValue; 3],
    hw_lights: &Lights,
    lamp_test: bool,
    comp_acty_counter: u32,
) -> DskyDisplayState {
    let comp_acty = lamp_test || (comp_acty_counter % 10 < 5);

    let lights = if lamp_test {
        Lights {
            uplink_acty: true,
            temp: true,
            gimbal_lock: true,
            prog_alarm: true,
            key_rel: true,
            opr_err: true,
            comp_acty: true,
            no_att: true,
            stby: true,
            restart: true,
            tracker: true,
            alt: true,
            vel: true,
        }
    } else {
        Lights {
            comp_acty,
            uplink_acty: hw_lights.uplink_acty,
            temp: hw_lights.temp,
            gimbal_lock: hw_lights.gimbal_lock,
            prog_alarm: hw_lights.prog_alarm,
            key_rel: hw_lights.key_rel,
            opr_err: hw_lights.opr_err,
            no_att: hw_lights.no_att,
            stby: hw_lights.stby,
            restart: hw_lights.restart,
            tracker: hw_lights.tracker,
            alt: hw_lights.alt,
            vel: hw_lights.vel,
        }
    };

    DskyDisplayState {
        prog: [prog / 10, prog % 10],
        verb: [verb / 10, verb % 10],
        noun: [noun / 10, noun % 10],
        r1: reg_value_to_data_register(&regs[0]),
        r2: reg_value_to_data_register(&regs[1]),
        r3: reg_value_to_data_register(&regs[2]),
        lights,
    }
}

fn reg_value_to_data_register(rv: &RegValue) -> DataRegister {
    if rv.blank {
        return DataRegister::BLANK;
    }
    let sign = if rv.sign >= 0 {
        Sign::Plus
    } else {
        Sign::Minus
    };
    let v = rv.value.min(99_999);
    let digits = [
        ((v / 10_000) % 10) as u8,
        ((v / 1_000) % 10) as u8,
        ((v / 100) % 10) as u8,
        ((v / 10) % 10) as u8,
        (v % 10) as u8,
    ];
    DataRegister { sign, digits }
}
