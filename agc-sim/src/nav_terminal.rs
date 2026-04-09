//! Navigation state ratatui renderer.
//!
//! Renders a live orbital state vector panel alongside the simulation log.
//! Used by `nav_demo` to visualise SERVICER (AVERAGE G) cycles.

use ratatui::{
    layout::{Constraint, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph},
    Frame,
};

use crate::sim_log::{LogLevel, SimLog};

// ── Styles ────────────────────────────────────────────────────────────────────

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

fn normal() -> Style {
    Style::default()
}

// ── Public types ──────────────────────────────────────────────────────────────

/// Snapshot of navigation state for one render frame.
pub struct NavSnapshot {
    /// MET in centiseconds.
    pub met_cs: u32,
    /// SERVICER cycles completed.
    pub cycle: u64,
    /// ECI position in metres.
    pub r_m: [f64; 3],
    /// ECI velocity in m/s.
    pub v_ms: [f64; 3],
    /// Altitude above Earth surface in metres (r − R_earth).
    pub altitude_m: f64,
    /// Orbital speed |v| in m/s.
    pub speed_ms: f64,
    /// Fractional progress through current orbit [0.0, 1.0).
    pub orbit_fraction: f64,
    /// Estimated orbital period in seconds.
    pub period_s: f64,
    /// Last injected PIPA counts.
    pub pipa: [i16; 3],
    /// Cumulative delta-V magnitude from all PIPA burns so far (m/s).
    pub total_dv_ms: f64,
    /// Whether any alarm is currently raised.
    pub alarm_lit: bool,
    /// Last alarm code if any (octal string like "01202").
    pub alarm_code: Option<String>,
    /// Simulated time acceleration factor (1 = real-time SERVICER cadence).
    pub time_factor: u32,
    /// Semi-major axis (km) — from conics module.
    pub sma_km: f64,
    /// Eccentricity.
    pub ecc: f64,
    /// Inclination (degrees).
    pub inc_deg: f64,
    /// Apoapsis altitude (km). `f64::NAN` if not applicable (hyperbolic).
    pub apo_alt_km: f64,
    /// Periapsis altitude (km).
    pub peri_alt_km: f64,
    /// DAP mode label (e.g., "FREE", "HOLD", "BURN").
    pub dap_mode: &'static str,
    /// Whether a burn is in progress.
    pub burn_active: bool,
    /// Velocity-to-be-gained magnitude (m/s), 0 if no burn.
    pub vg_mag: f64,
    /// Active program number (0, 11, 30, 37, 40, 51, 61-67).
    pub active_prog: u8,
    /// P40 burn phase label (e.g., "ATTMVR", "COUNT", "ULLAGE", "BURN", "CUTOFF", "DONE").
    pub burn_phase: &'static str,
    /// Entry phase label for P61-P67 (e.g., "P61 PREP", "P63 INIT", "P66 BANK", etc.).
    pub entry_phase: &'static str,
}

// ── Top-level entry ───────────────────────────────────────────────────────────

/// Draw one frame: navigation panel on the left, event log on the right.
pub fn render(f: &mut Frame, nav: &NavSnapshot, log: &SimLog) {
    let area = f.area();
    let cols = Layout::horizontal([Constraint::Length(48), Constraint::Min(24)]).split(area);
    render_nav(f, nav, cols[0]);
    render_log(f, log, cols[1]);
}

// ── Navigation panel ──────────────────────────────────────────────────────────

fn render_nav(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    let outer = Block::default()
        .title(" SERVICER — AVERAGE G NAVIGATION ")
        .borders(Borders::ALL)
        .border_style(bold());
    let inner = outer.inner(area);
    f.render_widget(outer, area);

    let rows = Layout::vertical([
        Constraint::Length(1), // [0]  MET / cycle
        Constraint::Length(1), // [1]  spacer
        Constraint::Length(1), // [2]  column headers
        Constraint::Length(1), // [3]  X row
        Constraint::Length(1), // [4]  Y row
        Constraint::Length(1), // [5]  Z row
        Constraint::Length(1), // [6]  spacer
        Constraint::Length(1), // [7]  altitude / speed
        Constraint::Length(1), // [8]  orbit fraction
        Constraint::Length(1), // [9]  orbital elements row 1 (SMA/ECC/INC)
        Constraint::Length(1), // [10] orbital elements row 2 (APO/PERI/DAP) or burn
        Constraint::Length(1), // [11] program + phase + DAP combined row
        Constraint::Length(1), // [12] spacer
        Constraint::Length(1), // [13] PIPA / alarm
        Constraint::Length(1), // [14] cumulative delta-V
        Constraint::Length(1), // [15] time factor
        Constraint::Length(1), // [16] spacer
        Constraint::Length(1), // [17] keys row 1
        Constraint::Length(1), // [18] keys row 2
        Constraint::Min(0),
    ])
    .split(inner);

    render_met_line(f, nav, rows[0]);
    render_vector_headers(f, rows[2]);
    render_vector_row(f, "X", nav.r_m[0] / 1000.0, nav.v_ms[0], rows[3]);
    render_vector_row(f, "Y", nav.r_m[1] / 1000.0, nav.v_ms[1], rows[4]);
    render_vector_row(f, "Z", nav.r_m[2] / 1000.0, nav.v_ms[2], rows[5]);
    render_alt_speed(f, nav, rows[7]);
    render_orbit_progress(f, nav, rows[8]);
    render_orbital_elements_row1(f, nav, rows[9]);
    render_orbital_elements_row2(f, nav, rows[10]);
    render_prog_dap(f, nav, rows[11]);
    render_pipa_alarm(f, nav, rows[13]);
    render_total_dv(f, nav, rows[14]);
    render_time_factor(f, nav, rows[15]);
    render_keys1(f, rows[17]);
    render_keys2(f, rows[18]);
}

fn render_met_line(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    let met_s = nav.met_cs / 100;
    let met_cs = nav.met_cs % 100;
    let line = Line::from(vec![
        Span::styled("MET ", dim()),
        Span::styled(format!("{:+06}s.{:02}", met_s, met_cs), bold()),
        Span::styled("  cycle ", dim()),
        Span::styled(format!("{}", nav.cycle), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_vector_headers(f: &mut Frame, area: Rect) {
    let cols = Layout::horizontal([
        Constraint::Length(3),  // axis
        Constraint::Length(14), // position
        Constraint::Length(3),  // gap
        Constraint::Min(0),     // velocity
    ])
    .split(area);
    f.render_widget(Paragraph::new(Span::styled("   ", normal())), cols[0]);
    f.render_widget(
        Paragraph::new(Span::styled("POSITION (km)", dim())),
        cols[1],
    );
    f.render_widget(Paragraph::new(Span::styled("   ", normal())), cols[2]);
    f.render_widget(
        Paragraph::new(Span::styled("VELOCITY (m/s)", dim())),
        cols[3],
    );
}

fn render_vector_row(f: &mut Frame, axis: &str, pos_km: f64, vel_ms: f64, area: Rect) {
    let cols = Layout::horizontal([
        Constraint::Length(3),  // axis label
        Constraint::Length(14), // position
        Constraint::Length(3),  // gap
        Constraint::Min(0),     // velocity
    ])
    .split(area);
    f.render_widget(
        Paragraph::new(Span::styled(format!("{} ", axis), dim())),
        cols[0],
    );
    f.render_widget(
        Paragraph::new(Span::styled(format!("{:+12.3}", pos_km), bold())),
        cols[1],
    );
    f.render_widget(Paragraph::new(Span::styled("   ", normal())), cols[2]);
    f.render_widget(
        Paragraph::new(Span::styled(format!("{:+12.3}", vel_ms), bold())),
        cols[3],
    );
}

fn render_alt_speed(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    let line = Line::from(vec![
        Span::styled("ALT ", dim()),
        Span::styled(format!("{:+9.3} km", nav.altitude_m / 1000.0), bold()),
        Span::styled("  SPD ", dim()),
        Span::styled(format!("{:8.3} m/s", nav.speed_ms), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_orbit_progress(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    let pct = nav.orbit_fraction * 100.0;
    let bar_width = 20usize;
    let filled = (nav.orbit_fraction * bar_width as f64) as usize;
    let bar: String = (0..bar_width)
        .map(|i| if i < filled { '█' } else { '░' })
        .collect();
    let line = Line::from(vec![
        Span::styled("ORB ", dim()),
        Span::styled(bar, normal()),
        Span::styled(format!(" {:5.1}%", pct), bold()),
        Span::styled(format!("  T={:.0}s", nav.period_s), dim()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn fmt_alt_km(alt_km: f64) -> String {
    if alt_km.is_nan() || alt_km.is_infinite() {
        "     ---".to_string()
    } else {
        format!("{:+8.1}", alt_km)
    }
}

fn render_orbital_elements_row1(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    let line = Line::from(vec![
        Span::styled("SMA ", dim()),
        Span::styled(format!("{:8.1} km", nav.sma_km), bold()),
        Span::styled("  ECC ", dim()),
        Span::styled(format!("{:.3}", nav.ecc), bold()),
        Span::styled("  INC ", dim()),
        Span::styled(format!("{:5.1}°", nav.inc_deg), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_orbital_elements_row2(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    if nav.burn_active {
        let line = Line::from(vec![
            Span::styled("BURN VG ", dim()),
            Span::styled(format!("{:+8.3} m/s", nav.vg_mag), bold()),
            Span::styled("  DAP ", dim()),
            Span::styled(nav.dap_mode, lit()),
        ]);
        f.render_widget(Paragraph::new(line), area);
    } else {
        let line = Line::from(vec![
            Span::styled("APO ", dim()),
            Span::styled(format!("{} km", fmt_alt_km(nav.apo_alt_km)), bold()),
            Span::styled("  PER ", dim()),
            Span::styled(format!("{} km", fmt_alt_km(nav.peri_alt_km)), bold()),
            Span::styled("  DAP ", dim()),
            Span::styled(nav.dap_mode, normal()),
        ]);
        f.render_widget(Paragraph::new(line), area);
    }
}

fn render_prog_dap(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    let prog_label = format!("P{:02}", nav.active_prog);

    let phase_span = if !nav.entry_phase.is_empty() {
        // Entry guidance active (P61-P67).
        Span::styled(nav.entry_phase, lit())
    } else if nav.burn_active {
        // P40 burn in progress — show phase and VG.
        Span::styled(
            format!("{}  VG {:+.1} m/s", nav.burn_phase, nav.vg_mag),
            lit(),
        )
    } else if nav.burn_phase != "IDLE" && !nav.burn_phase.is_empty() {
        // Phase set but burn not yet active (e.g., ATTMVR).
        Span::styled(nav.burn_phase, bold())
    } else {
        Span::styled("IDLE", dim())
    };

    let line = Line::from(vec![
        Span::styled("PROG ", dim()),
        Span::styled(prog_label, bold()),
        Span::styled("  ", normal()),
        phase_span,
        Span::styled("  DAP ", dim()),
        Span::styled(nav.dap_mode, if nav.burn_active { lit() } else { normal() }),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_pipa_alarm(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    let alarm_span = if nav.alarm_lit {
        let code = nav.alarm_code.as_deref().unwrap_or("????");
        Span::styled(format!("ALARM {}", code), lit())
    } else {
        Span::styled("ALARM -----", dim())
    };
    let line = Line::from(vec![
        Span::styled("PIPA ", dim()),
        Span::styled(
            format!("[{:+4} {:+4} {:+4}]", nav.pipa[0], nav.pipa[1], nav.pipa[2]),
            bold(),
        ),
        Span::styled("  ", normal()),
        alarm_span,
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_total_dv(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    let line = Line::from(vec![
        Span::styled("ΣΔV ", dim()),
        Span::styled(format!("{:+9.3} m/s", nav.total_dv_ms), bold()),
        Span::styled("  (cumulative burns)", dim()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_time_factor(f: &mut Frame, nav: &NavSnapshot, area: Rect) {
    let line = Line::from(vec![
        Span::styled("TIME×", dim()),
        Span::styled(format!("{:4}", nav.time_factor), bold()),
        Span::styled("  (cycles per frame)", dim()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_keys1(f: &mut Frame, area: Rect) {
    let line = Line::from(vec![Span::styled(
        "X/x Y/y Z/z → ±PIPA  [B]urn +50 m/s",
        normal(),
    )]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_keys2(f: &mut Frame, area: Rect) {
    let line = Line::from(vec![
        Span::styled("[+]/[-] time×  [C]lear  [0]P00 [1]P11 [3]P30  ", normal()),
        Span::styled("[Q]uit", dim()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

// ── Log panel ─────────────────────────────────────────────────────────────────

fn render_log(f: &mut Frame, log: &SimLog, area: Rect) {
    let outer = Block::default()
        .title(" STATE LOG ")
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
