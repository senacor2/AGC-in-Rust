//! P40 — SPS Thrusting / P41 — RCS Thrusting.
//!
//! Executes a maneuver plan computed by P30 or P37.  The program drives the
//! burn through attitude maneuver, countdown, ullage, active burn, and cutoff.
//!
//! AGC source: P40-P47.agc — P40CSM, S40.8, S40.9.

use crate::control::dap::DapMode;
use crate::guidance::maneuver::{BurnState, VG_CUTOFF_THRESHOLD};
use crate::guidance::targeting::ManeuverPlan;
use crate::math::linalg::norm;
use crate::navigation::state_vector::StateVector;

/// Program number for SPS thrusting.
pub const P40_NUMBER: u8 = 40;

/// Program number for RCS thrusting.
pub const P41_NUMBER: u8 = 41;

/// Ullage duration for an SPS burn (seconds).
///
/// AGC source: P40-P47.agc — S40.135: "7 SEC ULLAGE TO GO, PLUS 0.96 SEC
/// FROM PIPTIME". Nominal ullage is 7 seconds.
pub const SPS_ULLAGE_DURATION_S: f64 = 7.0;

/// Ullage duration for an RCS-only burn (seconds).
///
/// AGC source: P40-P47.agc — P41 uses shorter ullage.
pub const RCS_ULLAGE_DURATION_S: f64 = 2.0;

/// Attitude maneuver window: time budget before TIG for slewing (seconds).
///
/// AGC source: P40-P47.agc — R60CSM attitude maneuver called well before TIG.
pub const ATTITUDE_MANEUVER_WINDOW_S: f64 = 900.0;

/// P40/P41 execution phases.
///
/// AGC source: P40-P47.agc — P40CSM phase sequencing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum P40Phase {
    /// Waiting for attitude maneuver to burn attitude (R60).
    AttitudeManeuver,
    /// Countdown to TIG (displaying TTOGO).
    Countdown,
    /// Ullage — RCS jets firing to settle propellant.
    Ullage,
    /// Burn active — SPS/RCS firing, VG tracking.
    Burning,
    /// Engine cutoff, post-burn residuals.
    Cutoff,
    /// Complete.
    Complete,
}

/// Actions P40/P41 requests from the executive each cycle.
///
/// AGC source: P40-P47.agc — engine-on/off discrete outputs.
#[derive(Clone, Copy, Debug)]
pub struct P40Action {
    /// Requested DAP mode for this cycle.
    pub dap_mode: DapMode,
    /// Whether the main engine (SPS or RCS thrusters) should be firing.
    pub engine_on: bool,
    /// Whether ullage RCS should be on.
    pub ullage_on: bool,
    /// True when the program has finished (cutoff complete).
    pub complete: bool,
}

/// Persistent P40/P41 state.
#[derive(Clone, Debug)]
pub struct P40State {
    /// Current execution phase.
    pub phase: P40Phase,
    /// Maneuver plan computed by P30/P37.
    pub plan: ManeuverPlan,
    /// Active burn state (VG tracking).
    pub burn: BurnState,
    /// True when using SPS (P40); false when RCS-only (P41).
    pub use_sps: bool,
    /// Seconds until TIG (time-to-go).
    pub ttogo_s: f64,
    /// Ullage duration remaining (seconds).
    pub ullage_remaining_s: f64,
}

impl P40State {
    /// Initialise P40/P41 from a maneuver plan.
    ///
    /// `use_sps` selects SPS (P40) vs. RCS-only (P41).
    ///
    /// AGC source: P40-P47.agc — P40CSM initialisation, FENG / F constant loading.
    pub fn new(plan: ManeuverPlan, use_sps: bool) -> Self {
        let ullage = if use_sps {
            SPS_ULLAGE_DURATION_S
        } else {
            RCS_ULLAGE_DURATION_S
        };
        Self {
            phase: P40Phase::AttitudeManeuver,
            burn: BurnState::new(&plan.delta_v_eci),
            ttogo_s: plan.burn_time_s + ullage + ATTITUDE_MANEUVER_WINDOW_S,
            ullage_remaining_s: ullage,
            plan,
            use_sps,
        }
    }

    /// Advance one SERVICER cycle (~2 s).
    ///
    /// Drives the phase state machine and returns the action set the executive
    /// should apply.  The caller must pass the current state vector so that VG
    /// can be tracked once the burn is active.
    ///
    /// AGC source: P40-P47.agc — S40.8 burn monitor, S40.9 cutoff logic,
    /// UPDATEVG delta-V integration.
    pub fn cycle(&mut self, sv: &StateVector, dt: f64) -> P40Action {
        match self.phase {
            P40Phase::AttitudeManeuver => {
                self.ttogo_s -= dt;
                // Transition to Countdown once attitude maneuver window expires.
                if self.ttogo_s <= (self.plan.burn_time_s + self.ullage_remaining_s) {
                    self.phase = P40Phase::Countdown;
                }
                P40Action {
                    dap_mode: DapMode::Maneuver,
                    engine_on: false,
                    ullage_on: false,
                    complete: false,
                }
            }
            P40Phase::Countdown => {
                self.ttogo_s -= dt;
                if self.ttogo_s <= self.ullage_remaining_s {
                    self.phase = P40Phase::Ullage;
                }
                P40Action {
                    dap_mode: DapMode::AttitudeHold,
                    engine_on: false,
                    ullage_on: false,
                    complete: false,
                }
            }
            P40Phase::Ullage => {
                self.ullage_remaining_s -= dt;
                if self.ullage_remaining_s <= 0.0 {
                    self.phase = P40Phase::Burning;
                }
                P40Action {
                    dap_mode: DapMode::AttitudeHold,
                    engine_on: false,
                    ullage_on: true,
                    complete: false,
                }
            }
            P40Phase::Burning => {
                // Approximate measured delta-V from thrust over one cycle.
                // Real hardware reads PIPAs; here we derive from the plan.
                let dv_rate = if self.plan.burn_time_s > 0.0 {
                    self.plan.delta_v_mag / self.plan.burn_time_s
                } else {
                    0.0
                };
                let dv_step = dv_rate * dt;
                // Apply delta-V along current VG direction, clamped so we do
                // not overshoot (which would oscillate VG around zero).
                let vg_mag = norm(&self.burn.vg);
                let clamped_step = if dv_step > vg_mag { vg_mag } else { dv_step };
                let measured_dv = if vg_mag > 1e-30 {
                    let scale = clamped_step / vg_mag;
                    [
                        self.burn.vg[0] * scale,
                        self.burn.vg[1] * scale,
                        self.burn.vg[2] * scale,
                    ]
                } else {
                    [0.0; 3]
                };
                self.burn.update_vg(&measured_dv);
                let _ = sv; // state vector available for future use (e.g. guidance)
                if self.burn.complete || norm(&self.burn.vg) < VG_CUTOFF_THRESHOLD {
                    self.phase = P40Phase::Cutoff;
                }
                P40Action {
                    dap_mode: DapMode::AttitudeHold,
                    engine_on: true,
                    ullage_on: false,
                    complete: false,
                }
            }
            P40Phase::Cutoff => {
                self.phase = P40Phase::Complete;
                P40Action {
                    dap_mode: DapMode::AttitudeHold,
                    engine_on: false,
                    ullage_on: false,
                    complete: true,
                }
            }
            P40Phase::Complete => P40Action {
                dap_mode: DapMode::FreeDrift,
                engine_on: false,
                ullage_on: false,
                complete: true,
            },
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;

    fn dummy_sv() -> StateVector {
        StateVector {
            frame: Frame::Eci,
            r: [6_571_000.0, 0.0, 0.0],
            v: [0.0, 7_788.0, 0.0],
            t: Met(0),
        }
    }

    fn simple_plan() -> ManeuverPlan {
        ManeuverPlan {
            tig_cs: 1000,
            delta_v_eci: [100.0, 0.0, 0.0],
            delta_v_mag: 100.0,
            burn_time_s: 22.0,
        }
    }

    #[test]
    fn new_state_starts_in_attitude_maneuver() {
        let state = P40State::new(simple_plan(), true);
        assert_eq!(state.phase, P40Phase::AttitudeManeuver);
    }

    #[test]
    fn cycle_advances_through_phases() {
        let mut state = P40State::new(simple_plan(), true);
        let sv = dummy_sv();

        // Drive quickly through AttitudeManeuver phase.
        while state.phase == P40Phase::AttitudeManeuver {
            state.cycle(&sv, 60.0);
        }
        assert_eq!(state.phase, P40Phase::Countdown);

        // Drive through Countdown.
        while state.phase == P40Phase::Countdown {
            state.cycle(&sv, 1.0);
        }
        assert_eq!(state.phase, P40Phase::Ullage);

        // Drive through Ullage.
        while state.phase == P40Phase::Ullage {
            state.cycle(&sv, 1.0);
        }
        assert_eq!(state.phase, P40Phase::Burning);
    }

    #[test]
    fn burn_completes_when_vg_below_threshold() {
        let plan = ManeuverPlan {
            tig_cs: 0,
            delta_v_eci: [10.0, 0.0, 0.0],
            delta_v_mag: 10.0,
            burn_time_s: 5.0,
        };
        let mut state = P40State::new(plan, true);
        let sv = dummy_sv();

        // Skip to Burning phase.
        state.phase = P40Phase::Burning;

        // Cycle until we reach Complete (bounded).
        let mut action = state.cycle(&sv, 2.0);
        for _ in 0..100 {
            if action.complete {
                break;
            }
            action = state.cycle(&sv, 2.0);
        }
        assert!(action.complete, "burn should be complete, phase={:?}", state.phase);
    }

    #[test]
    fn rcs_burn_uses_shorter_ullage() {
        let state_sps = P40State::new(simple_plan(), true);
        let state_rcs = P40State::new(simple_plan(), false);
        assert!(state_sps.ullage_remaining_s > state_rcs.ullage_remaining_s);
    }
}
