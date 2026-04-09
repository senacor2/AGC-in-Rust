//! P61–P67 — Entry guidance programs.
//!
//! Covers the complete entry sequence from pre-entry preparation through
//! drogue deployment.  Entry guidance phases correspond directly to the AGC
//! program numbers P61–P67.
//!
//! AGC source: P61-P67.agc, ENTRY_LEXICON.agc.

use crate::math::linalg::norm;
use crate::navigation::state_vector::StateVector;

/// Entry interface altitude (m).
///
/// 400,000 ft = 121,920 m.
///
/// AGC source: P61-P67.agc — EI (entry interface) altitude used to trigger
/// entry guidance activation.
pub const ENTRY_INTERFACE_M: f64 = 121_920.0;

/// G-load threshold (Earth g's) that triggers constant-g guidance (P64).
///
/// AGC source: P61-P67.agc — 0.05 g entry detection.
pub const G_LOAD_ENTRY_THRESHOLD: f64 = 0.05;

/// G-load threshold indicating skip-up (P65).
///
/// AGC source: P61-P67.agc — skip-up detection, ~0.2 g.
pub const G_LOAD_SKIP_THRESHOLD: f64 = 0.2;

/// Velocity threshold for P67 final entry (m/s).
///
/// AGC source: P61-P67.agc — "P67 if V < 27000 FPS when .2G occurs".
/// 27,000 ft/s = 8,229.6 m/s. P67 is velocity-gated, not g-load-gated.
pub const P67_VELOCITY_THRESHOLD: f64 = 8_229.6;

/// Drogue deploy altitude (m).  ~24,000 ft = 7,315 m.
///
/// AGC source: P61-P67.agc — P67 drogue conditions.
pub const DROGUE_ALTITUDE_M: f64 = 7_315.0;

/// Standard gravity (m/s²).
const G0: f64 = 9.80665;

/// Earth mean radius (m).
pub const R_EARTH_M: f64 = 6_371_000.0;

/// Entry guidance phases (one per program P61–P67).
///
/// AGC source: P61-P67.agc — program phase sequencing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EntryPhase {
    /// P61: Entry preparation — monitoring coast to EI.
    P61Prep,
    /// P62: CM/SM separation (jettison service module).
    P62Separation,
    /// P63: Entry initialization — g-load monitoring, blackout.
    P63Init,
    /// P64: Post-blackout — constant-g entry.
    P64ConstantG,
    /// P65: Skip-up guidance (if g > skip threshold after initial entry).
    P65SkipUp,
    /// P66: Bank angle steering to landing target.
    P66BankSteer,
    /// P67: Final entry — drogue deploy conditions.
    P67Final,
    /// Complete — chutes deployed.
    Complete,
}

/// Persistent entry guidance state.
#[derive(Clone, Debug)]
pub struct EntryState {
    /// Current entry phase.
    pub phase: EntryPhase,
    /// Current altitude above Earth centre minus Earth radius (m).
    pub altitude_m: f64,
    /// Current g-load (Earth g's).
    pub g_load: f64,
    /// Commanded bank angle (rad).
    pub bank_angle_cmd: f64,
    /// Range to target landing site (m).
    pub range_to_target_m: f64,
    /// Whether CM/SM separation has occurred.
    pub separated: bool,
    /// Drag acceleration magnitude (m/s²).
    pub drag_accel: f64,
    /// Current inertial speed (m/s) — used for P67 velocity-gated transition.
    pub speed_ms: f64,
}

impl EntryState {
    /// Construct a new entry state, ready for P61 monitoring.
    pub const fn new() -> Self {
        Self {
            phase: EntryPhase::P61Prep,
            altitude_m: 0.0,
            g_load: 0.0,
            bank_angle_cmd: 0.0,
            range_to_target_m: 0.0,
            separated: false,
            drag_accel: 0.0,
            speed_ms: 0.0,
        }
    }

    /// Update entry state from current state vector and sensed acceleration.
    ///
    /// `sensed_accel` is the magnitude of non-gravitational (drag) acceleration
    /// in m/s².  `r_earth` is the Earth radius at the current sub-vehicle point
    /// in metres; pass `R_EARTH_M` for a spherical Earth approximation.
    ///
    /// AGC source: ENTRY_LEXICON.agc — CM/DLOAD, CM/KENTRY altitude and g
    /// computation.
    pub fn update(&mut self, sv: &StateVector, sensed_accel: f64, r_earth: f64) {
        let r_mag = norm(&sv.r);
        self.altitude_m = r_mag - r_earth;
        self.speed_ms = norm(&sv.v);
        self.drag_accel = sensed_accel;
        self.g_load = sensed_accel / G0;
        // Range-to-target is maintained externally; leave unchanged here.
    }

    /// Get the commanded bank angle for the current phase.
    ///
    /// In phases before P64 the bank angle is zero (lift vector up).
    /// In P64–P66 the magnitude is determined by the entry guidance equations;
    /// this implementation returns the stored `bank_angle_cmd`.
    ///
    /// AGC source: P61-P67.agc — bank angle logic per phase.
    pub fn bank_angle(&self) -> f64 {
        match self.phase {
            EntryPhase::P61Prep | EntryPhase::P62Separation | EntryPhase::P63Init => 0.0,
            _ => self.bank_angle_cmd,
        }
    }

    /// Check for phase transitions based on current conditions.
    ///
    /// This function must be called after `update` each cycle.
    ///
    /// AGC source: P61-P67.agc — phase transition logic.
    pub fn check_transitions(&mut self) {
        match self.phase {
            EntryPhase::P61Prep => {
                // Transition to separation once below entry interface.
                if self.altitude_m <= ENTRY_INTERFACE_M && !self.separated {
                    self.phase = EntryPhase::P62Separation;
                } else if self.altitude_m <= ENTRY_INTERFACE_M && self.separated {
                    self.phase = EntryPhase::P63Init;
                }
            }
            EntryPhase::P62Separation => {
                if self.separated {
                    self.phase = EntryPhase::P63Init;
                }
            }
            EntryPhase::P63Init => {
                if self.g_load >= G_LOAD_ENTRY_THRESHOLD {
                    if self.g_load >= G_LOAD_SKIP_THRESHOLD {
                        self.phase = EntryPhase::P65SkipUp;
                    } else {
                        self.phase = EntryPhase::P64ConstantG;
                    }
                }
            }
            EntryPhase::P64ConstantG => {
                // AGC: P64 → P66 when range within target and drag conditions met.
                // Simplified: transition when g > 0.2 (skip threshold).
                if self.g_load >= G_LOAD_SKIP_THRESHOLD {
                    self.phase = EntryPhase::P66BankSteer;
                }
            }
            EntryPhase::P65SkipUp => {
                // After skip-up, g-load decreases; resume bank steering.
                if self.g_load < G_LOAD_ENTRY_THRESHOLD {
                    self.phase = EntryPhase::P66BankSteer;
                }
            }
            EntryPhase::P66BankSteer => {
                // AGC: P67 is velocity-gated — "P67 if V < 27000 FPS when .2G occurs".
                // 27,000 ft/s = 8,229.6 m/s.
                if self.speed_ms < P67_VELOCITY_THRESHOLD {
                    self.phase = EntryPhase::P67Final;
                }
            }
            EntryPhase::P67Final => {
                // Drogue deploy — below altitude threshold.
                if self.altitude_m <= DROGUE_ALTITUDE_M {
                    self.phase = EntryPhase::Complete;
                }
            }
            EntryPhase::Complete => {}
        }
    }
}

impl Default for EntryState {
    fn default() -> Self {
        Self::new()
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

    fn sv_at_altitude(alt_m: f64) -> StateVector {
        StateVector {
            frame: Frame::Eci,
            r: [R_EARTH_M + alt_m, 0.0, 0.0],
            v: [0.0, 7_000.0, 0.0],
            t: Met(0),
        }
    }

    #[test]
    fn new_state_starts_at_p61_prep() {
        let s = EntryState::new();
        assert_eq!(s.phase, EntryPhase::P61Prep);
    }

    #[test]
    fn update_high_altitude_stays_in_p61() {
        let mut s = EntryState::new();
        let sv = sv_at_altitude(200_000.0); // above EI
        s.update(&sv, 0.0, R_EARTH_M);
        s.check_transitions();
        assert_eq!(s.phase, EntryPhase::P61Prep);
        assert!((s.altitude_m - 200_000.0).abs() < 1.0);
    }

    #[test]
    fn update_below_ei_with_separation_transitions_to_p63() {
        let mut s = EntryState::new();
        s.separated = true;
        let sv = sv_at_altitude(100_000.0); // below EI
        s.update(&sv, 0.0, R_EARTH_M);
        s.check_transitions();
        assert_eq!(s.phase, EntryPhase::P63Init);
    }

    #[test]
    fn high_g_load_triggers_p64() {
        let mut s = EntryState::new();
        s.phase = EntryPhase::P63Init;
        let sv = sv_at_altitude(80_000.0);
        // 0.1 g drag — above entry threshold, below skip threshold.
        s.update(&sv, 0.1 * 9.80665, R_EARTH_M);
        s.check_transitions();
        assert_eq!(s.phase, EntryPhase::P64ConstantG);
    }

    #[test]
    fn skip_threshold_triggers_p65() {
        let mut s = EntryState::new();
        s.phase = EntryPhase::P63Init;
        let sv = sv_at_altitude(70_000.0);
        // 0.5 g — above skip threshold.
        s.update(&sv, 0.5 * 9.80665, R_EARTH_M);
        s.check_transitions();
        assert_eq!(s.phase, EntryPhase::P65SkipUp);
    }

    #[test]
    fn bank_angle_zero_before_p64() {
        let s = EntryState::new(); // P61Prep
        assert_eq!(s.bank_angle(), 0.0);
    }

    #[test]
    fn bank_angle_returns_cmd_in_p66() {
        let mut s = EntryState::new();
        s.phase = EntryPhase::P66BankSteer;
        s.bank_angle_cmd = 1.2;
        assert!((s.bank_angle() - 1.2).abs() < 1e-15);
    }
}
