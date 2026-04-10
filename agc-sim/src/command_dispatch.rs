//! Verb/Noun dispatcher — maps crew-entered V/N commands to mission actions.
//!
//! AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc — VBRTEFN tables,
//! EXTENDED_VERBS.agc.

use crate::mission::Mission;
use crate::sim_log::SimLog;
use agc_core::guidance::maneuver::BurnState;
use agc_core::math::linalg::{norm, scale as vec_scale};

/// Action requested by an executed V/N command.
#[derive(Clone, Debug)]
pub enum CommandAction {
    /// Lamp test — set all lights on for ~5 seconds.
    LampTest,
    /// Change to program N.
    ChangeProgram(u8),
    /// Start a pre-canned 50 m/s prograde burn (presentation shortcut).
    StartCannedBurn,
    /// Terminate running program (V34).
    Terminate,
    /// No observable effect; just latch the new verb/noun.
    DisplayOnly,
    /// Invalid combination — sets OPR ERR.
    Error,
}

/// Dispatch a V/N pair that was just confirmed by ENTER.
///
/// This is the mission-level dispatcher (on top of PINBALL's verb classification).
/// Keeps the AGC-fidelity story while allowing presentation-friendly shortcuts
/// (e.g. V37N40 auto-loads a 50 m/s burn for the demo).
pub fn dispatch(
    verb: u8,
    noun: u8,
    mission: &mut Mission,
    burn: &mut Option<BurnState>,
    log: &mut SimLog,
) -> CommandAction {
    match verb {
        35 => {
            log.info("V35 — LAMP TEST");
            CommandAction::LampTest
        }
        37 => {
            // V37Nxx — change major mode
            mission.active_prog = noun;
            log.info(format!(
                "V37N{:02} — entering {}",
                noun,
                mission.phase_label()
            ));
            if noun == 40 {
                // Auto-load a 50 m/s prograde burn for the demo.
                let speed = norm(&mission.sv.v);
                if speed > 1.0 {
                    let dv = vec_scale(&mission.sv.v, 50.0 / speed);
                    *burn = Some(BurnState::new(&dv));
                    log.info("P40: BurnState armed 50.000 m/s prograde");
                    return CommandAction::StartCannedBurn;
                }
            }
            CommandAction::ChangeProgram(noun)
        }
        34 => {
            log.warn(format!(
                "V34 — terminating program P{:02}",
                mission.active_prog
            ));
            mission.active_prog = 0;
            *burn = None;
            CommandAction::Terminate
        }
        6 | 16 => {
            // V06/V16 display/monitor — just latch the noun.
            log.info(format!("V{:02}N{:02} — display/monitor", verb, noun));
            CommandAction::DisplayOnly
        }
        _ => {
            log.warn(format!("V{:02}N{:02} — not implemented", verb, noun));
            CommandAction::Error
        }
    }
}
