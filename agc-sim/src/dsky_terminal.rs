//! Ratatui render helpers for the AGC simulator TUI.
//!
//! Three-panel layout:
//! - Top-left: DSKY panel (PROG/VERB/NOUN, R1-R3, lamp indicators).
//! - Top-right: Mission State panel ("NO NAV STATE" for Milestone 1).
//! - Bottom (full width): Mission Log panel.
//!
//! All panels are stubs for Milestone 1.  Navigation state wiring happens in
//! Milestone 2; verb/noun state machines happen in Milestone 5.

use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph},
    Frame,
};

use crate::sim_hardware::SimHardware;
use crate::{dsky_state::DskyDisplayState, sim_log::LogLevel};

/// Render the complete three-panel TUI frame.
///
/// Called once per render tick (~20 Hz) from the main loop.
pub fn render(frame: &mut Frame, hw: &SimHardware) {
    let area = frame.area();

    // ── Layout: top row splits left/right; bottom row is full-width log ───────
    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(60), Constraint::Percentage(40)])
        .split(area);

    let top_cols = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(45), Constraint::Percentage(55)])
        .split(rows[0]);

    render_dsky(frame, top_cols[0], &hw.dsky.display);
    render_mission_state(frame, top_cols[1], hw);
    render_log(frame, rows[1], hw);
}

/// Render the DSKY panel.
///
/// Displays PROG/VERB/NOUN fields, R1-R3 registers, and lamp status row.
fn render_dsky(frame: &mut Frame, area: Rect, state: &DskyDisplayState) {
    let block = Block::default()
        .title(" DSKY ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Yellow));

    let inner = block.inner(area);
    frame.render_widget(block, area);

    // Format PROG/VERB/NOUN fields.
    let prog_str = format_two_digit(state.prog);
    let verb_str = format_two_digit(state.verb);
    let noun_str = format_two_digit(state.noun);

    // Format R1/R2/R3.
    let r1_str = format_register(state.r1);
    let r2_str = format_register(state.r2);
    let r3_str = format_register(state.r3);

    // Lamp row.
    let lamp_line = build_lamp_line(state);

    let lines = vec![
        Line::from(vec![
            Span::styled("PROG: ", Style::default().add_modifier(Modifier::BOLD)),
            Span::styled(prog_str, Style::default().fg(Color::Green)),
            Span::raw("  "),
            Span::styled("VERB: ", Style::default().add_modifier(Modifier::BOLD)),
            Span::styled(verb_str, Style::default().fg(Color::Green)),
            Span::raw("  "),
            Span::styled("NOUN: ", Style::default().add_modifier(Modifier::BOLD)),
            Span::styled(noun_str, Style::default().fg(Color::Green)),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("R1: ", Style::default().add_modifier(Modifier::BOLD)),
            Span::styled(r1_str, Style::default().fg(Color::Cyan)),
        ]),
        Line::from(vec![
            Span::styled("R2: ", Style::default().add_modifier(Modifier::BOLD)),
            Span::styled(r2_str, Style::default().fg(Color::Cyan)),
        ]),
        Line::from(vec![
            Span::styled("R3: ", Style::default().add_modifier(Modifier::BOLD)),
            Span::styled(r3_str, Style::default().fg(Color::Cyan)),
        ]),
        Line::from(""),
        lamp_line,
    ];

    let para = Paragraph::new(lines);
    frame.render_widget(para, inner);
}

/// Format a two-digit PROG/VERB/NOUN field.
fn format_two_digit(value: Option<u8>) -> String {
    match value {
        Some(v) => format!("{:02}", v),
        None => "  ".to_string(),
    }
}

/// Format a signed register value (±NNNNN) or blank.
fn format_register(value: Option<i32>) -> String {
    match value {
        Some(v) => format!("{:+06}", v),
        None => "      ".to_string(),
    }
}

/// Build the lamp status line from `DskyDisplayState` boolean fields.
fn build_lamp_line(state: &DskyDisplayState) -> Line<'static> {
    let lamp = |label: &'static str, on: bool| -> Span<'static> {
        if on {
            Span::styled(
                format!("[{label}] "),
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            )
        } else {
            Span::styled(format!("[{label}] "), Style::default().fg(Color::DarkGray))
        }
    };

    // OPRERR shows either oprerr (lamp-word driven) or error_light (VnState driven).
    let oprerr = state.oprerr || state.error_light;
    // KEY REL shows key_rel (lamp-word driven) or key_rel_light (VnState driven).
    let keyrel = state.key_rel || state.key_rel_light;
    // FLASH is shown when verb/noun is flashing (load verb waiting for input).
    let flash = state.flash_vn;

    Line::from(vec![
        lamp("PROG", state.prog_light),
        lamp("COMPACTY", state.comp_acty),
        lamp("KEYREL", keyrel),
        lamp("OPRERR", oprerr),
        lamp("FLASH", flash),
        lamp("NO ATT", state.no_att),
        lamp("GIMBAL LK", state.gimbal_lock),
        lamp("RESTART", state.restart),
        lamp("TRACKER", state.tracker),
        lamp("UPLINK", state.uplink_acty),
        lamp("TEMP", state.temp),
    ])
}

/// Render the Mission State panel.
///
/// Shows current program (modreg), MET, and time multiplier.
fn render_mission_state(frame: &mut Frame, area: Rect, hw: &SimHardware) {
    let block = Block::default()
        .title(" Mission State ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan));

    let inner = block.inner(area);
    frame.render_widget(block, area);

    let prog_str = if hw.agc_state.modreg >= 0 {
        format!("P{:02}", hw.agc_state.modreg)
    } else {
        "P--".to_string()
    };
    let verb_str = hw
        .vn
        .verb_buf
        .map(|v| format!("V{:02}", v))
        .unwrap_or_else(|| "V--".to_string());
    let noun_str = hw
        .vn
        .noun_buf
        .map(|n| format!("N{:02}", n))
        .unwrap_or_else(|| "N--".to_string());
    let met_cs = hw.agc_state.tephem.0;
    let met_secs = met_cs / 100;
    let met_str = format!(
        "MET {:02}:{:02}:{:02}",
        met_secs / 3600,
        (met_secs % 3600) / 60,
        met_secs % 60
    );
    let tmx_str = format!("TMX {:.2}x", hw.time_multiplier);

    let lines = vec![
        Line::from(vec![
            Span::styled("PROG: ", Style::default().add_modifier(Modifier::BOLD)),
            Span::styled(prog_str, Style::default().fg(Color::Green)),
            Span::raw("  "),
            Span::styled(verb_str, Style::default().fg(Color::Yellow)),
            Span::raw("  "),
            Span::styled(noun_str, Style::default().fg(Color::Yellow)),
        ]),
        Line::from(vec![Span::styled(
            met_str,
            Style::default().fg(Color::Cyan),
        )]),
        Line::from(vec![Span::styled(
            tmx_str,
            Style::default().fg(Color::DarkGray),
        )]),
        Line::from(""),
        Line::from(vec![Span::styled(
            "Keys: v=VERB n=NOUN Enter=ENTR c=CLR r=KEYREL +/- sign",
            Style::default().fg(Color::DarkGray),
        )]),
        Line::from(vec![Span::styled(
            "      q=quit F1=launch F2=burn F3=free =/_ time",
            Style::default().fg(Color::DarkGray),
        )]),
    ];

    let para = Paragraph::new(lines);
    frame.render_widget(para, inner);
}

/// Render the Mission Log panel (bottom, full width).
///
/// Shows the most recent log entries from `SimLog`, newest at the bottom.
fn render_log(frame: &mut Frame, area: Rect, hw: &SimHardware) {
    let block = Block::default()
        .title(" Mission Log ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Blue));

    let inner = block.inner(area);
    frame.render_widget(block, area);

    // Collect the last N lines that fit in the panel height.
    let max_lines = inner.height as usize;
    let entries: Vec<_> = hw.log.lines().collect();
    let start = entries.len().saturating_sub(max_lines);
    let visible = &entries[start..];

    let items: Vec<ListItem> = visible
        .iter()
        .map(|entry| {
            let (level_str, level_color) = match entry.level {
                LogLevel::Info => ("INFO", Color::Green),
                LogLevel::Warn => ("WARN", Color::Yellow),
                LogLevel::Error => ("ERR ", Color::Red),
            };
            let elapsed = entry.timestamp.elapsed();
            let secs = elapsed.as_secs();
            let line = Line::from(vec![
                Span::styled(
                    format!("[{:>6}s] ", secs),
                    Style::default().fg(Color::DarkGray),
                ),
                Span::styled(format!("[{level_str}] "), Style::default().fg(level_color)),
                Span::raw(entry.text.clone()),
            ]);
            ListItem::new(line)
        })
        .collect();

    let list = List::new(items);
    frame.render_widget(list, inner);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_two_digit_blank() {
        assert_eq!(format_two_digit(None), "  ");
    }

    #[test]
    fn format_two_digit_value() {
        assert_eq!(format_two_digit(Some(7)), "07");
        assert_eq!(format_two_digit(Some(37)), "37");
    }

    #[test]
    fn format_register_blank() {
        assert_eq!(format_register(None), "      ");
    }

    #[test]
    fn format_register_positive() {
        assert_eq!(format_register(Some(12345)), "+12345");
    }

    #[test]
    fn format_register_negative() {
        assert_eq!(format_register(Some(-99)), "-00099");
    }
}
