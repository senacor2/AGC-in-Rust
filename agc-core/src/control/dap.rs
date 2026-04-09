//! CSM Digital Autopilot supervisor.
//!
//! The DAP runs on T5RUPT (every 100 ms). It manages three modes: attitude
//! hold, rate damping, and free drift, plus a maneuver mode for slewing to a
//! target attitude.
//!
//! AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — T5 interrupt, RCSATT routine.

use crate::control::attitude::{attitude_error, phase_plane};
use crate::control::rcs_logic::{select_rotation_jets, TorqueCommand, MIN_IMPULSE_MS};
use crate::hal::rcs::Rcs;

/// DAP operating mode.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — DAPDATR1 mode word.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DapMode {
    /// Free drift — no active control, jets off.
    FreeDrift,
    /// Rate damping — null body rates only.
    RateDamp,
    /// Attitude hold — maintain current orientation.
    AttitudeHold,
    /// Maneuver — rotating to a target attitude.
    Maneuver,
}

/// DAP configuration parameters.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — DAPDATR1/DAPDATR2 data words.
#[derive(Clone, Copy, Debug)]
pub struct DapConfig {
    /// Deadband width in radians (typically 5° wide or 0.5° fine).
    pub deadband: f64,
    /// Rate limit in rad/s above which attitude hold fires jets.
    pub rate_limit: f64,
    /// DAP cycle period in seconds (T5RUPT = 100 ms = 0.1 s).
    pub dt: f64,
}

impl Default for DapConfig {
    fn default() -> Self {
        Self {
            deadband: 5.0 * (core::f64::consts::PI / 180.0), // 5° wide deadband
            rate_limit: 0.035,                                 // ~2°/s
            dt: 0.1,                                           // 100 ms T5RUPT period
        }
    }
}

/// Persistent DAP state updated each T5RUPT cycle.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — RCSATT, DAPDATR storage.
pub struct DapState {
    /// Current operating mode.
    pub mode: DapMode,
    /// DAP tuning parameters.
    pub config: DapConfig,
    /// Commanded attitude angles (roll, pitch, yaw) in radians.
    pub attitude_cmd: [f64; 3],
    /// Estimated body rates (roll, pitch, yaw) in rad/s.
    pub body_rates: [f64; 3],
    /// CDU readings from the previous T5 cycle used for rate estimation.
    prev_cdu: [f64; 3],
}

impl DapState {
    /// Construct initial DAP state with default configuration.
    pub const fn new() -> Self {
        Self {
            mode: DapMode::FreeDrift,
            config: DapConfig {
                deadband: 5.0 * (core::f64::consts::PI / 180.0),
                rate_limit: 0.035,
                dt: 0.1,
            },
            attitude_cmd: [0.0; 3],
            body_rates: [0.0; 3],
            prev_cdu: [0.0; 3],
        }
    }

    /// Update body-rate estimates from new CDU readings.
    ///
    /// Rate is approximated as the first-order finite difference of CDU angle
    /// divided by the DAP cycle time. Angle difference is wrapped to [-π, π]
    /// to handle CDU counter rollover.
    ///
    /// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — rate estimation from CDU delta.
    pub fn update_rates(&mut self, current_cdu: &[f64; 3]) {
        let dt = self.config.dt;
        for i in 0..3 {
            let mut delta = current_cdu[i] - self.prev_cdu[i];
            // Wrap delta to [-π, π] to handle CDU counter rollover.
            delta = wrap_angle(delta);
            self.body_rates[i] = delta / dt;
            self.prev_cdu[i] = current_cdu[i];
        }
    }

    /// Set a new commanded attitude (roll, pitch, yaw in radians).
    pub fn set_attitude_cmd(&mut self, cmd: [f64; 3]) {
        self.attitude_cmd = cmd;
    }

    /// Transition to a new DAP mode.
    pub fn set_mode(&mut self, mode: DapMode) {
        self.mode = mode;
    }
}

impl Default for DapState {
    fn default() -> Self {
        Self::new()
    }
}

/// Execute one T5RUPT DAP cycle.
///
/// Reads current CDU angles, updates rate estimates, computes attitude error,
/// and fires the appropriate RCS jets through `rcs`. In `FreeDrift` mode all
/// jets are cut off and no computation is performed.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — T5 interrupt handler, RCSATT.
pub fn dap_tick<R: Rcs>(state: &mut DapState, current_cdu: &[f64; 3], rcs: &mut R) {
    if state.mode == DapMode::FreeDrift {
        rcs.all_jets_off();
        return;
    }

    state.update_rates(current_cdu);

    match state.mode {
        DapMode::FreeDrift => {
            // Already handled above.
        }
        DapMode::RateDamp => {
            // Drive body rates to zero; ignore attitude error.
            let cmd = torque_from_rates(&state.body_rates, state.config.rate_limit);
            let mask = select_rotation_jets(&cmd);
            if mask != 0 {
                rcs.fire_sm_jets(mask, MIN_IMPULSE_MS);
            } else {
                rcs.all_jets_off();
            }
        }
        DapMode::AttitudeHold | DapMode::Maneuver => {
            let error = attitude_error(current_cdu, &state.attitude_cmd);
            let mut cmd = TorqueCommand::default();
            cmd.roll = phase_plane(
                error[0],
                state.body_rates[0],
                state.config.deadband,
                state.config.rate_limit,
            );
            cmd.pitch = phase_plane(
                error[1],
                state.body_rates[1],
                state.config.deadband,
                state.config.rate_limit,
            );
            cmd.yaw = phase_plane(
                error[2],
                state.body_rates[2],
                state.config.deadband,
                state.config.rate_limit,
            );
            let mask = select_rotation_jets(&cmd);
            if mask != 0 {
                rcs.fire_sm_jets(mask, MIN_IMPULSE_MS);
            } else {
                rcs.all_jets_off();
            }
        }
    }
}

/// Determine torque commands purely from body rates (rate-damp mode).
///
/// Fires jets opposing any rate that exceeds the rate limit threshold.
fn torque_from_rates(rates: &[f64; 3], rate_limit: f64) -> TorqueCommand {
    let sign = |v: f64| -> i8 {
        if v > rate_limit {
            -1 // oppose positive rate
        } else if v < -rate_limit {
            1 // oppose negative rate
        } else {
            0
        }
    };
    TorqueCommand {
        roll: sign(rates[0]),
        pitch: sign(rates[1]),
        yaw: sign(rates[2]),
    }
}

/// Wrap angle to [-π, π].
#[inline]
fn wrap_angle(mut a: f64) -> f64 {
    use core::f64::consts::PI;
    use libm::fmod;
    // fmod brings into (-2π, 2π); final shift brings to (-π, π].
    a = fmod(a + PI, 2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal no-op RCS for testing.
    struct NullRcs {
        last_sm_mask: u16,
        jets_off_called: bool,
    }
    impl NullRcs {
        fn new() -> Self {
            Self {
                last_sm_mask: 0,
                jets_off_called: false,
            }
        }
    }
    impl Rcs for NullRcs {
        fn fire_sm_jets(&mut self, mask: u16, _duration_ms: u16) {
            self.last_sm_mask = mask;
        }
        fn fire_cm_jets(&mut self, _mask: u16, _duration_ms: u16) {}
        fn all_jets_off(&mut self) {
            self.jets_off_called = true;
            self.last_sm_mask = 0;
        }
        fn sm_jets_firing(&self) -> u16 {
            self.last_sm_mask
        }
        fn cm_jets_firing(&self) -> u16 {
            0
        }
    }

    #[test]
    fn free_drift_cuts_jets() {
        let mut state = DapState::new();
        state.set_mode(DapMode::FreeDrift);
        let mut rcs = NullRcs::new();
        dap_tick(&mut state, &[0.1, 0.0, 0.0], &mut rcs);
        assert!(rcs.jets_off_called, "free drift must cut all jets");
    }

    #[test]
    fn mode_transition() {
        let mut state = DapState::new();
        assert_eq!(state.mode, DapMode::FreeDrift);
        state.set_mode(DapMode::AttitudeHold);
        assert_eq!(state.mode, DapMode::AttitudeHold);
        state.set_mode(DapMode::RateDamp);
        assert_eq!(state.mode, DapMode::RateDamp);
    }

    #[test]
    fn rate_estimation_from_cdu_changes() {
        let mut state = DapState::new();
        // Simulate 0.1 rad change in roll over one 100 ms cycle → 1.0 rad/s.
        state.prev_cdu = [0.0; 3];
        state.update_rates(&[0.1, 0.0, 0.0]);
        let diff = (state.body_rates[0] - 1.0).abs();
        assert!(diff < 1e-10, "roll rate should be 1.0 rad/s, got {}", state.body_rates[0]);
    }

    #[test]
    fn rate_damp_fires_opposing_jets() {
        let mut state = DapState::new();
        state.set_mode(DapMode::RateDamp);
        // prev_cdu starts at [0.0; 3] (from DapState::new).
        // Pass current_cdu = [1.0, 0.0, 0.0]: dap_tick calls update_rates internally,
        // estimating a positive roll rate of 1.0/0.1 = 10 rad/s.
        let mut rcs = NullRcs::new();
        dap_tick(&mut state, &[1.0, 0.0, 0.0], &mut rcs);
        // Rate-damp must fire jets opposing the positive roll rate.
        assert_ne!(rcs.last_sm_mask, 0, "rate damp should fire jets to oppose rate");
    }
}
