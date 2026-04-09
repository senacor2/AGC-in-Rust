//! DSKY ratatui renderer.
//!
//! Provides a single `render` function that draws one frame into a ratatui
//! `Frame`. Terminal lifecycle (setup / teardown / event loop) is owned by
//! the caller (`dsky_demo`) so there is no hidden state here.

use ratatui::{
    layout::{Constraint, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph},
    Frame,
};

use crate::dsky_state::{DataRegister, DskyDisplayState, Lights};
use crate::sim_log::{LogLevel, SimLog};

// ── Shared styles ─────────────────────────────────────────────────────────────

/// Normal body text — inherits terminal foreground so it works on any bg.
fn normal() -> Style {
    Style::default()
}

/// Bold text for important values.
fn bold() -> Style {
    Style::default().add_modifier(Modifier::BOLD)
}

/// Dimmed text for labels and off-state items.
fn dim() -> Style {
    Style::default().add_modifier(Modifier::DIM)
}

/// Reversed video for active/lit indicators.
fn lit() -> Style {
    Style::default()
        .add_modifier(Modifier::REVERSED)
        .add_modifier(Modifier::BOLD)
}

// ── Top-level entry point ─────────────────────────────────────────────────────

/// Draw one frame: DSKY panel on the left, state log on the right.
pub fn render(f: &mut Frame, display: &DskyDisplayState, log: &SimLog) {
    let area = f.area();
    let cols = Layout::horizontal([Constraint::Length(46), Constraint::Min(24)]).split(area);
    render_dsky(f, display, cols[0]);
    render_log(f, log, cols[1]);
}

// ── DSKY panel ────────────────────────────────────────────────────────────────

fn render_dsky(f: &mut Frame, d: &DskyDisplayState, area: Rect) {
    let outer = Block::default()
        .title(" APOLLO GUIDANCE COMPUTER ")
        .borders(Borders::ALL)
        .border_style(bold());
    let inner = outer.inner(area);
    f.render_widget(outer, area);

    let rows = Layout::vertical([
        Constraint::Length(1), // lights row 1
        Constraint::Length(1), // lights row 2
        Constraint::Length(1), // spacer
        Constraint::Length(1), // PROG / VERB / NOUN labels
        Constraint::Length(1), // PROG / VERB / NOUN values
        Constraint::Length(1), // spacer
        Constraint::Length(1), // R1
        Constraint::Length(1), // R2
        Constraint::Length(1), // R3
        Constraint::Length(1), // spacer
        Constraint::Length(1), // keys row 1
        Constraint::Length(1), // keys row 2
        Constraint::Min(0),
    ])
    .split(inner);

    render_lights_row1(f, &d.lights, rows[0]);
    render_lights_row2(f, &d.lights, rows[1]);
    render_pvn_labels(f, rows[3]);
    render_pvn_values(f, d, rows[4]);
    render_register(f, "R1", &d.r1, rows[6]);
    render_register(f, "R2", &d.r2, rows[7]);
    render_register(f, "R3", &d.r3, rows[8]);
    render_keys1(f, rows[10]);
    render_keys2(f, rows[11]);
}

fn light(label: &'static str, on: bool) -> Span<'static> {
    if on {
        Span::styled(format!("[{label}] "), lit())
    } else {
        Span::styled(format!(" {label}  "), dim())
    }
}

fn render_lights_row1(f: &mut Frame, l: &Lights, area: Rect) {
    let line = Line::from(vec![
        light("UPLINK", l.uplink_acty),
        light("TEMP", l.temp),
        light("GIMBAL", l.gimbal_lock),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_lights_row2(f: &mut Frame, l: &Lights, area: Rect) {
    let line = Line::from(vec![
        light("PROG", l.prog_alarm),
        light("KEY REL", l.key_rel),
        light("OPR ERR", l.opr_err),
        light("COMP", l.comp_acty),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn fmt_pair(digits: [u8; 2]) -> String {
    digits
        .iter()
        .map(|&d| if d > 9 { ' ' } else { (b'0' + d) as char })
        .collect()
}

fn render_pvn_labels(f: &mut Frame, area: Rect) {
    let cols = Layout::horizontal([
        Constraint::Length(15),
        Constraint::Length(15),
        Constraint::Min(0),
    ])
    .split(area);
    f.render_widget(Paragraph::new(Span::styled("  PROG", dim())), cols[0]);
    f.render_widget(Paragraph::new(Span::styled("  VERB", dim())), cols[1]);
    f.render_widget(Paragraph::new(Span::styled("  NOUN", dim())), cols[2]);
}

fn render_pvn_values(f: &mut Frame, d: &DskyDisplayState, area: Rect) {
    let cols = Layout::horizontal([
        Constraint::Length(15),
        Constraint::Length(15),
        Constraint::Min(0),
    ])
    .split(area);
    f.render_widget(
        Paragraph::new(Span::styled(format!("  [ {} ]", fmt_pair(d.prog)), bold())),
        cols[0],
    );
    f.render_widget(
        Paragraph::new(Span::styled(format!("  [ {} ]", fmt_pair(d.verb)), bold())),
        cols[1],
    );
    f.render_widget(
        Paragraph::new(Span::styled(format!("  [ {} ]", fmt_pair(d.noun)), bold())),
        cols[2],
    );
}

fn render_register(f: &mut Frame, label: &'static str, reg: &DataRegister, area: Rect) {
    let line = Line::from(vec![
        Span::styled(format!("{label}  "), dim()),
        Span::styled(reg.to_display_string(), bold()),
    ]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_keys1(f: &mut Frame, area: Rect) {
    let line = Line::from(vec![Span::styled(
        "[V]erb [N]oun [+] [-] [0-9] [Del]CLR",
        normal(),
    )]);
    f.render_widget(Paragraph::new(line), area);
}

fn render_keys2(f: &mut Frame, area: Rect) {
    let line = Line::from(vec![
        Span::styled("[P]ro [Enter] [R]set [K]eyRel", normal()),
        Span::styled("  [Q]uit", dim()),
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
