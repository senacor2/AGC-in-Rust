//! Terminal-based DSKY user interface for interactive simulation.
//!
//! Renders a `DskyFrame` to the terminal in a layout faithful to the
//! Block 2 DSKY (Figure 39 of O'Brien, "The Apollo Guidance Computer").
//!
//! Layout (66 columns wide):
//!
//! - Top-left: 2×7 indicator-lamp panel
//!   (UPLINK ACTY/TEMP, NO ATT/GIMBAL LOCK, STBY/PROG, KEY REL/RESTART,
//!   OPR ERR/TRACKER, and two spare cells)
//! - Top-right: display panel
//!   (COMP ACTY + PROG, VERB + NOUN, R1, R2, R3)
//! - Bottom: 7-column keyboard
//!   (VERB/NOUN | +/-/0 | 7/4/1 | 8/5/2 | 9/6/3 | CLR/PRO/KEYREL | ENTR/RSET)
//!
//! No raw-mode setup here — the binary main loop owns terminal state.
//! This module only writes ANSI sequences to the supplied writer.

use std::io::{self, Write};

use agc_core::services::pinball::{DskyFrame, Lamps, Register, TwoDigit};
use agc_core::services::v_n::Key;
use crossterm::{
    cursor::MoveTo,
    event::KeyCode,
    queue,
    style::{Color, Print, ResetColor, SetForegroundColor},
};

/// Total rendered width in columns.
pub const WIDTH: u16 = 66;

/// Total rendered height in rows (display + keyboard + propulsion + status).
pub const HEIGHT: u16 = 39;

// ── Colours ───────────────────────────────────────────────────────────────────

/// Colour used for active 7-segment digits, VERB/NOUN labels, lamp text.
const ACTIVE: Color = Color::White;
/// Colour used for inactive lamps, panel frames, legends.
const DIM: Color = Color::DarkGrey;
/// Accent colour used for the MET counter.
const ACCENT: Color = Color::Grey;
/// Colour for firing RCS jets.
const JET_FIRE: Color = Color::Green;
/// Colour for SPS thrust indicator.
const SPS_FIRE: Color = Color::Red;

// ── Propulsion frame ─────────────────────────────────────────────────────────

/// Snapshot of propulsion state for rendering.
pub struct PropulsionFrame {
    /// SM RCS jet bitmask (sticky visual — see `SimRcs::drain_visual`).
    pub sm_jets: u16,
    /// CM RCS jet bitmask (sticky visual).
    pub cm_jets: u16,
    /// SPS engine on/off.
    pub sps_thrusting: bool,
    /// SPS gimbal pitch in degrees.
    pub gimbal_pitch_deg: f32,
    /// SPS gimbal yaw in degrees.
    pub gimbal_yaw_deg: f32,
}

// ── Public entry point ───────────────────────────────────────────────────────

/// Render one full frame of the DSKY to `out`.
///
/// The caller is responsible for having placed the terminal in raw mode
/// and, if desired, an alternate screen. `origin` is the top-left corner
/// of the rendered panel.
///
/// When `propulsion` is `Some`, the propulsion panel is drawn below the
/// keyboard and the status line is shifted down.
pub fn render<W: Write>(
    out: &mut W,
    origin: (u16, u16),
    frame: &DskyFrame,
    propulsion: Option<&PropulsionFrame>,
    met_cs: u64,
    status: &str,
    flash_on: bool,
) -> io::Result<()> {
    let (ox, oy) = origin;

    draw_lamp_panel(out, ox, oy, &frame.lamps, frame.lamp_test)?;
    draw_display_panel(out, ox + 32, oy, frame, flash_on)?;
    draw_keyboard(out, ox, oy + 17)?;

    if let Some(prop) = propulsion {
        draw_propulsion_panel(out, ox, oy + 27, prop)?;
        draw_status(out, ox, oy + 36, met_cs, status)?;
    } else {
        draw_status(out, ox, oy + 27, met_cs, status)?;
    }

    queue!(out, ResetColor)?;
    out.flush()
}

// ── Lamp panel (left) ─────────────────────────────────────────────────────────

/// Lamp grid: (label, lit?). Pairs of (left, right) rows, top-to-bottom.
fn lamp_grid(lamps: &Lamps, lamp_test: bool) -> [[(&'static str, bool); 2]; 7] {
    let on = |b: bool| lamp_test || b;
    [
        [
            ("UPLINK ACTY", on(lamps.uplink_activity)),
            ("TEMP", on(lamps.temp)),
        ],
        [
            ("NO ATT", on(lamps.no_att)),
            ("GIMBAL LOCK", on(lamps.gimbal_lock)),
        ],
        [("STBY", on(lamps.stby)), ("PROG", on(lamps.prog_alarm))],
        [
            ("KEY REL", on(lamps.key_rel)),
            ("RESTART", on(lamps.restart)),
        ],
        [
            ("OPR ERR", on(lamps.opr_err)),
            ("TRACKER", on(lamps.tracker)),
        ],
        [("", false), ("", false)],
        [("", false), ("", false)],
    ]
}

fn draw_lamp_panel<W: Write>(
    out: &mut W,
    ox: u16,
    oy: u16,
    lamps: &Lamps,
    lamp_test: bool,
) -> io::Result<()> {
    let grid = lamp_grid(lamps, lamp_test);

    queue!(out, SetForegroundColor(DIM))?;
    queue!(out, MoveTo(ox, oy), Print("┌─────────────┬─────────────┐"))?;
    for row in 0..7 {
        let y = oy + 1 + row * 2;
        queue!(out, MoveTo(ox, y), Print("│             │             │"))?;
        queue!(
            out,
            MoveTo(ox, y + 1),
            Print("├─────────────┼─────────────┤")
        )?;
    }
    // Extra blank row to align with the 17-row display panel.
    queue!(
        out,
        MoveTo(ox, oy + 15),
        Print("│             │             │")
    )?;
    queue!(
        out,
        MoveTo(ox, oy + 16),
        Print("└─────────────┴─────────────┘")
    )?;

    for (row_idx, row) in grid.iter().enumerate() {
        let y = oy + 1 + (row_idx as u16) * 2;
        for (col_idx, (label, lit)) in row.iter().enumerate() {
            let x = ox + 1 + (col_idx as u16) * 14;
            let color = if *lit { ACTIVE } else { DIM };
            queue!(out, SetForegroundColor(color))?;
            // Labels are centred in a 13-col cell.
            let padded = centre(label, 13);
            queue!(out, MoveTo(x, y), Print(padded))?;
        }
    }
    Ok(())
}

// ── Display panel (right) ────────────────────────────────────────────────────

fn draw_display_panel<W: Write>(
    out: &mut W,
    ox: u16,
    oy: u16,
    frame: &DskyFrame,
    flash_on: bool,
) -> io::Result<()> {
    queue!(out, SetForegroundColor(DIM))?;
    // Outer frame (31 cols wide, 17 rows tall, matching the lamp panel height).
    queue!(
        out,
        MoveTo(ox, oy),
        Print("┌──────────────┬──────────────┐")
    )?;
    for row in 1..=3 {
        queue!(
            out,
            MoveTo(ox, oy + row),
            Print("│              │              │")
        )?;
    }
    queue!(
        out,
        MoveTo(ox, oy + 4),
        Print("├──────────────┼──────────────┤")
    )?;
    for row in 5..=7 {
        queue!(
            out,
            MoveTo(ox, oy + row),
            Print("│              │              │")
        )?;
    }
    queue!(
        out,
        MoveTo(ox, oy + 8),
        Print("├──────────────┴──────────────┤")
    )?;
    for row in 9..=10 {
        queue!(
            out,
            MoveTo(ox, oy + row),
            Print("│                             │")
        )?;
    }
    queue!(
        out,
        MoveTo(ox, oy + 11),
        Print("├─────────────────────────────┤")
    )?;
    for row in 12..=13 {
        queue!(
            out,
            MoveTo(ox, oy + row),
            Print("│                             │")
        )?;
    }
    queue!(
        out,
        MoveTo(ox, oy + 14),
        Print("├─────────────────────────────┤")
    )?;
    queue!(
        out,
        MoveTo(ox, oy + 15),
        Print("│                             │")
    )?;
    queue!(
        out,
        MoveTo(ox, oy + 16),
        Print("└─────────────────────────────┘")
    )?;

    // Row 0: COMP ACTY lamp | PROG label
    queue!(
        out,
        SetForegroundColor(if frame.lamps.comp_acty { ACTIVE } else { DIM })
    )?;
    queue!(out, MoveTo(ox + 2, oy + 1), Print("  COMP  "))?;
    queue!(out, MoveTo(ox + 2, oy + 2), Print("  ACTY  "))?;

    queue!(out, SetForegroundColor(DIM))?;
    queue!(out, MoveTo(ox + 17, oy + 1), Print("    PROG    "))?;
    queue!(out, SetForegroundColor(ACTIVE))?;
    queue!(
        out,
        MoveTo(ox + 17, oy + 2),
        Print(format!("     {}     ", two_digit(&frame.prog)))
    )?;

    // Row 5–7: VERB | NOUN, labels may flash
    let vn_color = if frame.flashing && !flash_on {
        DIM
    } else {
        ACTIVE
    };
    queue!(out, SetForegroundColor(DIM))?;
    queue!(out, MoveTo(ox + 2, oy + 5), Print("    VERB    "))?;
    queue!(out, MoveTo(ox + 17, oy + 5), Print("    NOUN    "))?;
    queue!(out, SetForegroundColor(vn_color))?;
    queue!(
        out,
        MoveTo(ox + 2, oy + 6),
        Print(format!("     {}     ", two_digit(&frame.verb)))
    )?;
    queue!(
        out,
        MoveTo(ox + 17, oy + 6),
        Print(format!("     {}     ", two_digit(&frame.noun)))
    )?;

    // Rows 9–10, 12–13, 15 → R1, R2, R3 (two rows per register box)
    draw_register(out, ox, oy + 9, "R1", &frame.r1)?;
    draw_register(out, ox, oy + 12, "R2", &frame.r2)?;
    draw_register(out, ox, oy + 15, "R3", &frame.r3)?;

    Ok(())
}

fn draw_register<W: Write>(
    out: &mut W,
    ox: u16,
    y: u16,
    label: &str,
    reg: &Register,
) -> io::Result<()> {
    queue!(out, SetForegroundColor(DIM))?;
    queue!(out, MoveTo(ox + 2, y), Print(label))?;

    let sign_ch = match reg.sign {
        1 => '+',
        -1 => '-',
        _ => ' ',
    };
    queue!(out, SetForegroundColor(ACTIVE))?;
    queue!(
        out,
        MoveTo(ox + 8, y),
        Print(format!(
            "{}{}{}{}{}{}",
            sign_ch, reg.digits[0], reg.digits[1], reg.digits[2], reg.digits[3], reg.digits[4],
        ))
    )?;
    if reg.overflow {
        queue!(
            out,
            SetForegroundColor(DIM),
            MoveTo(ox + 17, y),
            Print("[OVF]")
        )?;
    }
    Ok(())
}

// ── Keyboard panel (bottom) ──────────────────────────────────────────────────

fn draw_keyboard<W: Write>(out: &mut W, ox: u16, oy: u16) -> io::Result<()> {
    queue!(out, SetForegroundColor(DIM))?;
    let rows = [
        "  ┌────┐  ┌───┐ ┌───┐ ┌───┐ ┌───┐  ┌─────┐        ┌──────┐  ",
        "  │VERB│  │ + │ │ 7 │ │ 8 │ │ 9 │  │ CLR │        │ ENTR │  ",
        "  └────┘  └───┘ └───┘ └───┘ └───┘  └─────┘        └──────┘  ",
        "  ┌────┐  ┌───┐ ┌───┐ ┌───┐ ┌───┐  ┌─────┐        ┌──────┐  ",
        "  │NOUN│  │ - │ │ 4 │ │ 5 │ │ 6 │  │ PRO │        │ RSET │  ",
        "  └────┘  └───┘ └───┘ └───┘ └───┘  └─────┘        └──────┘  ",
        "          ┌───┐ ┌───┐ ┌───┐ ┌───┐  ┌─────┐                  ",
        "          │ 0 │ │ 1 │ │ 2 │ │ 3 │  │ KEY │                  ",
        "          └───┘ └───┘ └───┘ └───┘  │ REL │                  ",
        "                                   └─────┘                  ",
    ];
    for (i, row) in rows.iter().enumerate() {
        queue!(out, MoveTo(ox, oy + i as u16), Print(*row))?;
    }
    Ok(())
}

// ── Propulsion panel ─────────────────────────────────────────────────────────

/// Jet indicator: `●` if firing, `○` if idle.
fn jet_char(firing: bool) -> char {
    if firing {
        '●'
    } else {
        '○'
    }
}

/// Return the colour for a jet indicator.
fn jet_color(firing: bool) -> Color {
    if firing {
        JET_FIRE
    } else {
        DIM
    }
}

/// Draw a single jet indicator at the given position.
fn draw_jet<W: Write>(out: &mut W, x: u16, y: u16, label: &str, firing: bool) -> io::Result<()> {
    queue!(out, SetForegroundColor(jet_color(firing)))?;
    queue!(out, MoveTo(x, y), Print(label))?;
    queue!(out, Print(jet_char(firing)))?;
    Ok(())
}

/// Draw a single jet indicator with the label after the indicator.
fn draw_jet_rev<W: Write>(
    out: &mut W,
    x: u16,
    y: u16,
    label: &str,
    firing: bool,
) -> io::Result<()> {
    queue!(out, SetForegroundColor(jet_color(firing)))?;
    queue!(out, MoveTo(x, y), Print(jet_char(firing)))?;
    queue!(out, Print(label))?;
    Ok(())
}

fn draw_propulsion_panel<W: Write>(
    out: &mut W,
    ox: u16,
    oy: u16,
    prop: &PropulsionFrame,
) -> io::Result<()> {
    let div = 31u16; // vertical divider column (relative to ox)

    // ── Border ───────────────────────────────────────────────────────────────
    queue!(out, SetForegroundColor(DIM))?;
    // Top border with title
    queue!(out, MoveTo(ox, oy), Print("┌─PROPULSION"))?;
    for _ in 12..div {
        queue!(out, Print("─"))?;
    }
    queue!(out, Print("┬"))?;
    for _ in (div + 1)..65 {
        queue!(out, Print("─"))?;
    }
    queue!(out, Print("┐"))?;

    // Content rows (7 rows)
    for row in 1..=7 {
        let y = oy + row;
        queue!(out, MoveTo(ox, y), Print("│"))?;
        // Fill left half with spaces
        for _ in 1..div {
            queue!(out, Print(" "))?;
        }
        queue!(out, Print("│"))?;
        // Fill right half with spaces
        for _ in (div + 1)..65 {
            queue!(out, Print(" "))?;
        }
        // Right edge is at column 65
        queue!(out, MoveTo(ox + 65, y), Print("│"))?;
    }

    // Bottom border
    queue!(out, MoveTo(ox, oy + 8))?;
    queue!(out, Print("└"))?;
    for _ in 1..div {
        queue!(out, Print("─"))?;
    }
    queue!(out, Print("┴"))?;
    for _ in (div + 1)..65 {
        queue!(out, Print("─"))?;
    }
    queue!(out, Print("┘"))?;

    // ── Left half: SM RCS diamond layout ─────────────────────────────────────
    // Bit assignments (from rcs_logic.rs):
    //  0=B4  1=B3  2=B2  3=B1  4=A4  5=A3  6=A2  7=A1
    //  8=D4  9=D3  10=D2 11=D1 12=C4 13=C3 14=C2 15=C1
    let j = |bit: u8| -> bool { prop.sm_jets & (1u16 << bit) != 0 };

    // Quad A (top) — row 1-2
    queue!(out, SetForegroundColor(DIM))?;
    queue!(out, MoveTo(ox + 11, oy + 1), Print("[A]"))?;
    // Row 2: A4 A2 · A1 A3
    draw_jet(out, ox + 5, oy + 2, "A4", j(4))?;
    draw_jet(out, ox + 9, oy + 2, "A2", j(6))?;
    queue!(
        out,
        SetForegroundColor(DIM),
        MoveTo(ox + 12, oy + 2),
        Print("·")
    )?;
    draw_jet_rev(out, ox + 14, oy + 2, "A1", j(7))?;
    draw_jet_rev(out, ox + 18, oy + 2, "A3", j(5))?;

    // Quad labels D and B — row 3
    queue!(out, SetForegroundColor(DIM))?;
    queue!(out, MoveTo(ox + 1, oy + 3), Print("[D]"))?;
    queue!(out, MoveTo(ox + 21, oy + 3), Print("[B]"))?;

    // Quad D (left) — row 4
    draw_jet(out, ox + 1, oy + 4, "D4", j(8))?;
    draw_jet(out, ox + 5, oy + 4, "D3", j(9))?;
    queue!(
        out,
        SetForegroundColor(DIM),
        MoveTo(ox + 12, oy + 4),
        Print("·")
    )?;
    draw_jet_rev(out, ox + 14, oy + 4, "D1", j(11))?;
    draw_jet_rev(out, ox + 18, oy + 4, "D2", j(10))?;

    // Quad B (right) — row 5
    draw_jet(out, ox + 1, oy + 5, "B2", j(2))?;
    draw_jet(out, ox + 5, oy + 5, "B1", j(3))?;
    queue!(
        out,
        SetForegroundColor(DIM),
        MoveTo(ox + 12, oy + 5),
        Print("·")
    )?;
    draw_jet_rev(out, ox + 14, oy + 5, "B3", j(1))?;
    draw_jet_rev(out, ox + 18, oy + 5, "B4", j(0))?;

    // Quad C (bottom) — row 6-7
    draw_jet(out, ox + 5, oy + 6, "C4", j(12))?;
    draw_jet(out, ox + 9, oy + 6, "C2", j(14))?;
    queue!(
        out,
        SetForegroundColor(DIM),
        MoveTo(ox + 12, oy + 6),
        Print("·")
    )?;
    draw_jet_rev(out, ox + 14, oy + 6, "C1", j(15))?;
    draw_jet_rev(out, ox + 18, oy + 6, "C3", j(13))?;
    queue!(out, SetForegroundColor(DIM))?;
    queue!(out, MoveTo(ox + 11, oy + 7), Print("[C]"))?;

    // ── Right half: SPS engine ───────────────────────────────────────────────
    let rx = ox + div + 2; // right half starting x

    // Row 1: SPS status
    if prop.sps_thrusting {
        queue!(out, SetForegroundColor(SPS_FIRE))?;
        queue!(
            out,
            MoveTo(rx, oy + 1),
            Print("SPS: \u{2588}\u{2588} THRUST \u{2588}\u{2588}")
        )?;
    } else {
        queue!(out, SetForegroundColor(DIM))?;
        queue!(out, MoveTo(rx, oy + 1), Print("SPS: OFF"))?;
    }

    // Row 2: Gimbal readout
    queue!(out, SetForegroundColor(ACTIVE))?;
    queue!(
        out,
        MoveTo(rx, oy + 2),
        Print(format!(
            "Gimbal P:{:+05.1}\u{00b0} Y:{:+05.1}\u{00b0}",
            prop.gimbal_pitch_deg, prop.gimbal_yaw_deg
        ))
    )?;

    // Rows 4-6: Nozzle glyph
    if prop.sps_thrusting {
        queue!(out, SetForegroundColor(SPS_FIRE))?;
        queue!(
            out,
            MoveTo(rx + 5, oy + 4),
            Print("\u{2571}\u{2593}\u{2593}\u{2593}\u{2593}\u{2572}")
        )?;
        queue!(
            out,
            MoveTo(rx + 4, oy + 5),
            Print("\u{2571}\u{2593}\u{2593}\u{2593}\u{2593}\u{2593}\u{2593}\u{2572}")
        )?;
        queue!(
            out,
            MoveTo(rx + 3, oy + 6),
            Print(
                "\u{2571}\u{2593}\u{2593}\u{2593}\u{2593}\u{2593}\u{2593}\u{2593}\u{2593}\u{2572}"
            )
        )?;
    } else {
        queue!(out, SetForegroundColor(DIM))?;
        queue!(out, MoveTo(rx + 5, oy + 4), Print("\u{2571}    \u{2572}"))?;
        queue!(out, MoveTo(rx + 4, oy + 5), Print("\u{2571}      \u{2572}"))?;
        queue!(
            out,
            MoveTo(rx + 3, oy + 6),
            Print("\u{2571}        \u{2572}")
        )?;
    }

    Ok(())
}

// ── Status line ──────────────────────────────────────────────────────────────

fn draw_status<W: Write>(
    out: &mut W,
    ox: u16,
    oy: u16,
    met_cs: u64,
    status: &str,
) -> io::Result<()> {
    let total_s = met_cs / 100;
    let h = total_s / 3600;
    let m = (total_s % 3600) / 60;
    let s = total_s % 60;
    queue!(out, SetForegroundColor(ACCENT))?;
    queue!(
        out,
        MoveTo(ox, oy),
        Print(format!("  MET: {:03}:{:02}:{:02}   ", h, m, s))
    )?;
    queue!(out, SetForegroundColor(DIM))?;
    // Pad/truncate status to 34 columns so stale text is overwritten.
    let s = if status.len() > 34 {
        status[..34].to_string()
    } else {
        format!("{:<34}", status)
    };
    queue!(out, Print(s))?;
    queue!(out, MoveTo(ox, oy + 1))?;
    queue!(out, SetForegroundColor(DIM))?;
    queue!(
        out,
        Print("  Keys: V N 0-9 + - E(ntr) P(ro) C(lr) R(set) K(rel)  Q=quit")
    )?;
    Ok(())
}

// ── Helpers ──────────────────────────────────────────────────────────────────

fn two_digit(td: &TwoDigit) -> String {
    format!("{}{}", td.tens, td.units)
}

fn centre(s: &str, width: usize) -> String {
    if s.len() >= width {
        return s[..width].to_string();
    }
    let pad = width - s.len();
    let left = pad / 2;
    let right = pad - left;
    format!("{}{}{}", " ".repeat(left), s, " ".repeat(right))
}

// ── Key mapping ──────────────────────────────────────────────────────────────

/// Convert a crossterm `KeyCode` to a DSKY `Key`.
///
/// Returns `None` for keys that are not bound.
pub fn key_from_code(code: KeyCode) -> Option<Key> {
    match code {
        KeyCode::Char(c) => match c {
            '0'..='9' => Some(Key::Digit(c as u8 - b'0')),
            'v' | 'V' => Some(Key::Verb),
            'n' | 'N' => Some(Key::Noun),
            '+' => Some(Key::Plus),
            '-' => Some(Key::Minus),
            'e' | 'E' => Some(Key::Entr),
            'p' | 'P' => Some(Key::Pro),
            'c' | 'C' => Some(Key::Clr),
            'r' | 'R' => Some(Key::Rset),
            'k' | 'K' => Some(Key::KeyRel),
            _ => None,
        },
        KeyCode::Enter => Some(Key::Entr),
        _ => None,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centre_pads_symmetrically() {
        assert_eq!(centre("AB", 6), "  AB  ");
        assert_eq!(centre("A", 5), "  A  ");
    }

    #[test]
    fn two_digit_formats() {
        let td = TwoDigit { tens: 0, units: 6 };
        assert_eq!(two_digit(&td), "06");
    }

    #[test]
    fn key_from_code_maps_digits() {
        assert_eq!(key_from_code(KeyCode::Char('5')), Some(Key::Digit(5)));
        assert_eq!(key_from_code(KeyCode::Char('0')), Some(Key::Digit(0)));
    }

    #[test]
    fn key_from_code_maps_commands() {
        assert_eq!(key_from_code(KeyCode::Char('v')), Some(Key::Verb));
        assert_eq!(key_from_code(KeyCode::Char('N')), Some(Key::Noun));
        assert_eq!(key_from_code(KeyCode::Char('+')), Some(Key::Plus));
        assert_eq!(key_from_code(KeyCode::Char('-')), Some(Key::Minus));
        assert_eq!(key_from_code(KeyCode::Char('e')), Some(Key::Entr));
        assert_eq!(key_from_code(KeyCode::Enter), Some(Key::Entr));
        assert_eq!(key_from_code(KeyCode::Char('p')), Some(Key::Pro));
        assert_eq!(key_from_code(KeyCode::Char('c')), Some(Key::Clr));
        assert_eq!(key_from_code(KeyCode::Char('r')), Some(Key::Rset));
        assert_eq!(key_from_code(KeyCode::Char('k')), Some(Key::KeyRel));
    }

    #[test]
    fn key_from_code_ignores_unbound() {
        assert_eq!(key_from_code(KeyCode::Char('x')), None);
        assert_eq!(key_from_code(KeyCode::Tab), None);
    }
}
