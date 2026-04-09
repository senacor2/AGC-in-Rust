//! Navigation simulation demo — SERVICER (AVERAGE G) visualiser.
//!
//! Runs the Milestone 2 navigation pipeline live:
//!   • Initialises a 200 km LEO circular orbit state vector.
//!   • Steps the SERVICER (AVERAGE G) cycle every frame.
//!   • Lets you inject PIPA counts (simulated thruster pulses) and watch the
//!     orbit change in real time.
//!
//! Run with:
//!   cargo run -p agc-sim --bin nav_demo

use std::io;
use std::time::Duration;

use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{backend::CrosstermBackend, Terminal};

use agc_core::navigation::gravity::MU_EARTH;
use agc_core::navigation::state_vector::{Frame, Refsmmat, StateVector};
use agc_core::services::average_g::{average_g, AverageGState, CYCLE_DT};
use agc_core::types::Met;
use agc_sim::{nav_terminal, nav_terminal::NavSnapshot, SimHardware};

/// Earth pad radius used for altitude display (m).
///
/// AGC source: LATITUDE_LONGITUDE_SUBROUTINES.agc — `ERAD 2DEC 6373338 B-29 # PAD RADIUS`
/// Based on the Fischer ellipsoid (a=6378166 m, b=6356784 m). The commonly-cited
/// WGS84 mean radius (6371 km) is wrong for AGC altitude displays.
const R_EARTH_M: f64 = 6_373_338.0;

fn main() -> io::Result<()> {
    std::panic::set_hook(Box::new(|info| {
        let _ = disable_raw_mode();
        let _ = execute!(io::stdout(), LeaveAlternateScreen);
        eprintln!("\r\nPanic: {info}");
    }));

    let result = run();

    let _ = disable_raw_mode();
    let _ = execute!(io::stdout(), LeaveAlternateScreen);

    result
}

fn run() -> io::Result<()> {
    let mut hw = SimHardware::new();

    // 200 km LEO circular orbit.
    let r0 = 6_578_000.0_f64; // R_earth + 200 km
    let v_circ = f64::sqrt(MU_EARTH / r0);
    let period_s = 2.0 * core::f64::consts::PI * r0 / v_circ;

    let mut sv = StateVector {
        frame: Frame::Eci,
        r: [r0, 0.0, 0.0],
        v: [0.0, v_circ, 0.0],
        t: Met::ZERO,
    };

    let mut cycle: u64 = 0;
    let mut pipa: [i16; 3] = [0; 3];
    let mut time_factor: u32 = 1; // cycles per render frame
    let mut total_dv_ms: f64 = 0.0; // cumulative PIPA delta-V (m/s)
    let mut avg_g_state = AverageGState::new(); // predictor-corrector carry-over state

    hw.log.info("AGC-in-Rust — SERVICER / AVERAGE G demo");
    hw.log
        .info(format!("LEO 200 km  v_circ={:.1} m/s", v_circ));
    hw.log
        .info(format!("Orbital period T={:.0} s", period_s));
    hw.log.info("X/x Y/y Z/z = ±100 PIPA  +/- = time×  Q = quit");

    enable_raw_mode()?;
    execute!(io::stdout(), EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;

    loop {
        // ── Advance navigation ─────────────────────────────────────────────
        for _ in 0..time_factor {
            let pipa_i16: [i16; 3] = pipa;
            // Run one SERVICER cycle (AVERAGE G).
            let result = average_g(&sv, pipa_i16, &Refsmmat::IDENTITY, CYCLE_DT, &mut avg_g_state);
            sv = result.sv;
            cycle += 1;

            // Log delta-V when PIPA was non-zero.
            if pipa[0] != 0 || pipa[1] != 0 || pipa[2] != 0 {
                let dv = result.delta_v_total.magnitude();
                total_dv_ms += dv;
                hw.log.io(format!(
                    "cycle {} ΔV={:.3} m/s  ΣΔV={:.3} m/s",
                    cycle, dv, total_dv_ms
                ));
                // Drain PIPA after applying (one-shot per render frame).
                pipa = [0; 3];
            }
        }

        // ── Derived quantities ─────────────────────────────────────────────
        let r_norm = f64::sqrt(sv.r[0] * sv.r[0] + sv.r[1] * sv.r[1] + sv.r[2] * sv.r[2]);
        let speed = f64::sqrt(sv.v[0] * sv.v[0] + sv.v[1] * sv.v[1] + sv.v[2] * sv.v[2]);
        let altitude = r_norm - R_EARTH_M;

        // Orbit fraction: derived from MET so the bar advances 0→100% continuously.
        // acos-based angle is bounded to [0, π] and would oscillate; MET is monotonic.
        let met_s = sv.t.0 as f64 / 100.0;
        let orbit_fraction = (met_s % period_s) / period_s;

        // Alarm state: check if gravity singularity would trigger (altitude < 0).
        let alarm_lit = altitude < 0.0;
        let alarm_code = if alarm_lit {
            Some("IMPACT".to_string())
        } else {
            None
        };

        // ── Render ─────────────────────────────────────────────────────────
        let snap = NavSnapshot {
            met_cs: sv.t.0,
            cycle,
            r_m: sv.r,
            v_ms: sv.v,
            altitude_m: altitude,
            speed_ms: speed,
            orbit_fraction,
            period_s,
            pipa,
            total_dv_ms,
            alarm_lit,
            alarm_code,
            time_factor,
        };
        terminal.draw(|f| nav_terminal::render(f, &snap, &hw.log))?;

        // ── Input ──────────────────────────────────────────────────────────
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(k) = event::read()? {
                if k.kind != KeyEventKind::Press {
                    continue;
                }
                match k.code {
                    KeyCode::Esc | KeyCode::Char('q') => break,

                    // PIPA injection — 100 counts per press = 5.85 m/s.
                    KeyCode::Char('X') => {
                        pipa[0] += 100;
                        hw.log.info(format!("PIPA +X {} (+5.85 m/s)", pipa[0]));
                    }
                    KeyCode::Char('x') => {
                        pipa[0] -= 100;
                        hw.log.info(format!("PIPA -X {} (-5.85 m/s)", pipa[0]));
                    }
                    KeyCode::Char('Y') => {
                        pipa[1] += 100;
                        hw.log.info(format!("PIPA +Y {} (+5.85 m/s)", pipa[1]));
                    }
                    KeyCode::Char('y') => {
                        pipa[1] -= 100;
                        hw.log.info(format!("PIPA -Y {} (-5.85 m/s)", pipa[1]));
                    }
                    KeyCode::Char('Z') => {
                        pipa[2] += 100;
                        hw.log.info(format!("PIPA +Z {} (+5.85 m/s)", pipa[2]));
                    }
                    KeyCode::Char('z') => {
                        pipa[2] -= 100;
                        hw.log.info(format!("PIPA -Z {} (-5.85 m/s)", pipa[2]));
                    }

                    // Time acceleration.
                    KeyCode::Char('+') | KeyCode::Char('=') => {
                        time_factor = (time_factor * 2).min(512);
                        hw.log.info(format!("time× {}", time_factor));
                    }
                    KeyCode::Char('-') => {
                        time_factor = (time_factor / 2).max(1);
                        hw.log.info(format!("time× {}", time_factor));
                    }

                    // Clear pending PIPA.
                    KeyCode::Char('c') | KeyCode::Char('C') => {
                        pipa = [0; 3];
                        hw.log.info("PIPA cleared");
                    }

                    _ => {}
                }
            }
        }
    }

    Ok(())
}
