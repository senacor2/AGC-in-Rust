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
/// Caution-class indicator lamps (yellow) — O'Brien §7.2.
const CAUTION: Color = Color::Yellow;
/// Warning-class indicator lamps (red) — O'Brien §7.2.
const WARNING: Color = Color::Red;
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
///
/// `alarm_code` is the most recent program-alarm code (`AlarmState::code`).
/// It is rendered as a footer in the display panel's bottom border only
/// while the PROG alarm lamp is lit (`frame.lamps.prog_alarm`).
// A terminal-render entry point: each argument is an independent display
// input, so bundling them into a struct would not aid readability.
#[allow(clippy::too_many_arguments)]
pub fn render<W: Write>(
    out: &mut W,
    origin: (u16, u16),
    frame: &DskyFrame,
    propulsion: Option<&PropulsionFrame>,
    met_cs: u64,
    status: &str,
    flash_on: bool,
    alarm_code: u16,
) -> io::Result<()> {
    let (ox, oy) = origin;

    draw_lamp_panel(out, ox, oy, &frame.lamps, frame.lamp_test)?;
    draw_display_panel(out, ox + 32, oy, frame, flash_on, alarm_code)?;
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

/// Lamp grid: (label, lit?, lit-colour). Pairs of (left, right) rows,
/// top-to-bottom. The colour distinguishes *caution* (yellow) from
/// *warning* (red) per Frank O'Brien §7.2; unlit lamps render in [`DIM`].
fn lamp_grid(lamps: &Lamps, lamp_test: bool) -> [[(&'static str, bool, Color); 2]; 7] {
    let on = |b: bool| lamp_test || b;
    [
        [
            ("UPLINK ACTY", on(lamps.uplink_activity), CAUTION),
            ("TEMP", on(lamps.temp), CAUTION),
        ],
        [
            ("NO ATT", on(lamps.no_att), WARNING),
            ("GIMBAL LOCK", on(lamps.gimbal_lock), WARNING),
        ],
        [
            ("STBY", on(lamps.stby), CAUTION),
            ("PROG", on(lamps.prog_alarm), WARNING),
        ],
        [
            ("KEY REL", on(lamps.key_rel), CAUTION),
            ("RESTART", on(lamps.restart), CAUTION),
        ],
        [
            ("OPR ERR", on(lamps.opr_err), CAUTION),
            ("TRACKER", on(lamps.tracker), CAUTION),
        ],
        [("", false, DIM), ("", false, DIM)],
        [("", false, DIM), ("", false, DIM)],
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
        for (col_idx, (label, lit, lit_color)) in row.iter().enumerate() {
            let x = ox + 1 + (col_idx as u16) * 14;
            let color = if *lit { *lit_color } else { DIM };
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
    alarm_code: u16,
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
    // Bottom border doubles as the alarm-code footer when the PROG alarm
    // is lit; the code is embedded in the border (like the PROPULSION
    // title) and the whole line is drawn in the warning colour.
    let footer_code = frame.lamps.prog_alarm.then_some(alarm_code);
    if footer_code.is_some() {
        queue!(out, SetForegroundColor(WARNING))?;
    }
    queue!(
        out,
        MoveTo(ox, oy + 16),
        Print(display_bottom_border(footer_code))
    )?;
    queue!(out, SetForegroundColor(DIM))?;

    // Row 0: COMP ACTY lamp | PROG label
    queue!(
        out,
        SetForegroundColor(if frame.lamps.comp_acty { CAUTION } else { DIM })
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

/// Marker prefix `dsky_sim.rs::main` sets on `status` while the user is
/// typing a script filename after pressing `@`. `draw_status` detects this
/// prefix and switches to the wide-prompt layout (#59).
const PROMPT_PREFIX: &str = "Script file: ";

/// Width of the status row in characters. Matches the propulsion panel
/// above it (`draw_propulsion_panel` spans 65 cols).
const PROMPT_LINE_WIDTH: usize = 65;

/// Characters of the buffer that fit on the first line, after the
/// `"  Script file: "` label (2 leading spaces + 13-char prefix).
const PROMPT_LINE_1_CONTENT: usize = PROMPT_LINE_WIDTH - PROMPT_PREFIX.len() - 2;

/// Continuation marker on the second line when the buffer overflows line 1.
const PROMPT_LINE_2_LABEL: &str = "  ↳ ";

/// Characters of the buffer that fit on the second line.
/// `PROMPT_LINE_2_LABEL` is "  ↳ " — 4 display columns in a monospace
/// terminal (2 spaces + arrow + 1 space).
const PROMPT_LINE_2_CONTENT: usize = PROMPT_LINE_WIDTH - 4;

/// Split the user's typed filename buffer into two lines for the
/// wide-prompt layout. Line 1 fits up to [`PROMPT_LINE_1_CONTENT`]
/// characters; any overflow goes to line 2, itself truncated at
/// [`PROMPT_LINE_2_CONTENT`].
fn split_prompt_buffer(buf: &str) -> (String, String) {
    if buf.len() <= PROMPT_LINE_1_CONTENT {
        return (buf.to_string(), String::new());
    }
    let (a, rest) = buf.split_at(PROMPT_LINE_1_CONTENT);
    let b = if rest.len() > PROMPT_LINE_2_CONTENT {
        &rest[..PROMPT_LINE_2_CONTENT]
    } else {
        rest
    };
    (a.to_string(), b.to_string())
}

fn draw_status<W: Write>(
    out: &mut W,
    ox: u16,
    oy: u16,
    met_cs: u64,
    status: &str,
) -> io::Result<()> {
    // While the user is typing a script filename (the `@`-prompt in
    // dsky_sim.rs sets `status = "Script file: <buf>"`), we drop the MET
    // prefix and repurpose the keys-hint line for the input field. The
    // status row's natural width spans the propulsion panel (65 cols), so
    // we can show ~50 chars on line 1 and wrap the rest onto line 2,
    // accommodating filenames up to ~110 chars before another truncation
    // would kick in (#59).
    if let Some(buf) = status.strip_prefix(PROMPT_PREFIX) {
        let (line_1, line_2) = split_prompt_buffer(buf);

        queue!(out, SetForegroundColor(ACCENT))?;
        queue!(out, MoveTo(ox, oy), Print(format!("  {}", PROMPT_PREFIX)))?;
        queue!(out, SetForegroundColor(DIM))?;

        // Line 1: prompt + first chunk + cursor (when there's no wrap),
        // padded to clear stale text.
        let cursor = if line_2.is_empty() { "_" } else { "" };
        let l1 = format!("{}{}", line_1, cursor);
        let l1_padded = format!("{:<width$}", l1, width = PROMPT_LINE_1_CONTENT);
        queue!(out, Print(l1_padded))?;

        // Line 2: continuation if the buffer overflowed line 1, otherwise
        // the ENTER/ESC hint.
        queue!(out, MoveTo(ox, oy + 1))?;
        if line_2.is_empty() {
            queue!(
                out,
                Print(format!(
                    "{:<width$}",
                    "  [ENTER=load  ESC=cancel]",
                    width = PROMPT_LINE_WIDTH
                ))
            )?;
        } else {
            queue!(out, SetForegroundColor(ACCENT))?;
            queue!(out, Print(PROMPT_LINE_2_LABEL))?;
            queue!(out, SetForegroundColor(DIM))?;
            let l2_padded =
                format!("{:<width$}_", line_2, width = PROMPT_LINE_2_CONTENT - 1);
            queue!(out, Print(l2_padded))?;
        }
        return Ok(());
    }

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

/// Build the display-panel bottom border (31 display columns wide).
///
/// With `alarm_code = Some(code)`, the code is embedded as an `ALM nnnnn`
/// footer centred in the border — mirroring the V05N09 alarm display, where
/// the code value is shown as a 5-digit decimal. With `None`, a plain border
/// is returned. A `u16` is always ≤ 5 digits, so the footer always fits.
fn display_bottom_border(alarm_code: Option<u16>) -> String {
    /// Inner width of the display panel (between the corner glyphs).
    const INNER: usize = 29;
    match alarm_code {
        None => format!("└{}┘", "─".repeat(INNER)),
        Some(code) => {
            let label = format!(" ALM {:05} ", code);
            let dashes = INNER.saturating_sub(label.chars().count());
            let left = dashes / 2;
            let right = dashes - left;
            format!("└{}{}{}┘", "─".repeat(left), label, "─".repeat(right))
        }
    }
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

    /// A blank register (no sign, all-zero digits, no overflow).
    fn blank_register() -> Register {
        Register {
            sign: 0,
            digits: [0; 5],
            overflow: false,
        }
    }

    /// A minimal `DskyFrame` with all lamps dark, used as a test fixture.
    fn blank_frame() -> DskyFrame {
        DskyFrame {
            prog: TwoDigit { tens: 0, units: 0 },
            verb: TwoDigit { tens: 0, units: 0 },
            noun: TwoDigit { tens: 0, units: 0 },
            r1: blank_register(),
            r2: blank_register(),
            r3: blank_register(),
            lamps: Lamps {
                uplink_activity: false,
                no_att: false,
                stby: false,
                key_rel: false,
                opr_err: false,
                restart: false,
                gimbal_lock: false,
                temp: false,
                prog_alarm: false,
                comp_acty: false,
                tracker: false,
            },
            lamp_test: false,
            flashing: false,
        }
    }

    /// TC-#139-1: the plain bottom border is 31 display columns wide and has
    /// no embedded text.
    #[test]
    fn display_bottom_border_plain() {
        let s = display_bottom_border(None);
        assert_eq!(s, "└─────────────────────────────┘");
        assert_eq!(s.chars().count(), 31);
    }

    /// TC-#139-2: a synthetic alarm code is embedded as a 5-digit decimal
    /// `ALM nnnnn` footer, centred, with the border still 31 columns wide.
    #[test]
    fn display_bottom_border_with_alarm_code() {
        let s = display_bottom_border(Some(1202));
        // 29-col inner = 18 dashes split 9/9 around the 11-char " ALM 01202 ".
        let expected = format!("└{}{}{}┘", "─".repeat(9), " ALM 01202 ", "─".repeat(9));
        assert_eq!(s, expected);
        assert_eq!(s.chars().count(), 31);
        assert!(s.contains(" ALM 01202 "));
    }

    /// TC-#139-3: codes wider than five digits are reduced modulo 100000 so
    /// the footer never overflows the border.
    #[test]
    fn display_bottom_border_clamps_wide_code() {
        let s = display_bottom_border(Some(u16::MAX)); // 65535
        assert_eq!(s.chars().count(), 31);
        assert!(s.contains(" ALM 65535 "));
    }

    /// TC-#139-4: rendering a frame with the PROG alarm lit emits the footer
    /// text; a dark PROG alarm does not.
    #[test]
    fn render_emits_alarm_footer_only_when_lit() {
        let mut frame = blank_frame();

        let mut buf = Vec::new();
        render(&mut buf, (1, 1), &frame, None, 0, "", true, 1202).unwrap();
        assert!(
            !String::from_utf8_lossy(&buf).contains("ALM 01202"),
            "footer must be absent while the PROG alarm is dark"
        );

        frame.lamps.prog_alarm = true;
        let mut buf = Vec::new();
        render(&mut buf, (1, 1), &frame, None, 0, "", true, 1202).unwrap();
        assert!(
            String::from_utf8_lossy(&buf).contains("ALM 01202"),
            "footer must appear once the PROG alarm is lit"
        );
    }

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

    /// TC-#59-1: short filename fits on line 1, line 2 empty.
    #[test]
    fn split_prompt_buffer_short_fits_line_1() {
        let (l1, l2) = split_prompt_buffer("scripts/v71.dsky");
        assert_eq!(l1, "scripts/v71.dsky");
        assert!(l2.is_empty());
    }

    /// TC-#59-2: filename at the line-1 capacity still fits with no wrap.
    #[test]
    fn split_prompt_buffer_at_line_1_capacity_no_wrap() {
        let s: String = "a".repeat(PROMPT_LINE_1_CONTENT);
        let (l1, l2) = split_prompt_buffer(&s);
        assert_eq!(l1.len(), PROMPT_LINE_1_CONTENT);
        assert!(l2.is_empty());
    }

    /// TC-#59-3: filename one longer than line-1 wraps; the leftover goes
    /// onto line 2.
    #[test]
    fn split_prompt_buffer_overflows_to_line_2() {
        let mut s = "a".repeat(PROMPT_LINE_1_CONTENT);
        s.push('B');
        let (l1, l2) = split_prompt_buffer(&s);
        assert_eq!(l1.len(), PROMPT_LINE_1_CONTENT);
        assert_eq!(l2, "B");
    }

    /// TC-#59-4: the user's reported case (`scripts/v71_reseed_sample.dsky`,
    /// 30 chars) renders fully on line 1 with the new layout.
    #[test]
    fn split_prompt_buffer_reported_case_visible() {
        let (l1, l2) = split_prompt_buffer("scripts/v71_reseed_sample.dsky");
        assert_eq!(l1, "scripts/v71_reseed_sample.dsky");
        assert!(l2.is_empty());
        assert!(
            l1.len() < PROMPT_LINE_1_CONTENT,
            "user's example must fit on line 1; len={}, cap={PROMPT_LINE_1_CONTENT}",
            l1.len()
        );
    }
}
