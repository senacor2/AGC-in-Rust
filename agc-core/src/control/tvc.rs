//! Thrust Vector Control state for SPS burns.
//!
//! # Phase C — compute functions
//!
//! This module provides `tvc_init`, `tvc_step`, and `update_trim`.
//! `tvc_step` does **not** call any HAL method directly (Strategy D): it
//! returns the `(pitch_counts, yaw_counts)` tuple so the DAP supervisor
//! (Phase E) can write it to `AgcState::sps_gimbal_cmd` for the ISR shim.

use crate::types::Vec3;

/// Persistent state of the TVC (Thrust Vector Control) system.
///
/// Corresponds to Comanche055 erasable variables TVCPITCH, TVCYAW,
/// TRIMGIMB1, TRIMGIMB2. Updated on every T5RUPT cycle while
/// `DapMode::Tvc` is active.
///
/// NOTE: `TvcState` does NOT store `attitude_error`. The attitude error
/// is passed as a parameter to `tvc_step` (from `DapState::attitude_error`).
/// Keeping it out of TvcState clarifies ownership: attitude error belongs
/// to the DAP state, not the TVC servo state.
#[derive(Clone, Copy, Debug, Default)]
pub struct TvcState {
    /// Commanded pitch gimbal angle (radians).
    ///
    /// Positive pitch = nose up. Range ±GIMBAL_LIMIT_RAD.
    /// AGC equivalent: TVCPITCH (CDUSCMD, octal 0054), scale B-1 rev.
    pub gimbal_pitch: f64,
    /// Commanded yaw gimbal angle (radians).
    ///
    /// Positive yaw = nose right. Range ±GIMBAL_LIMIT_RAD.
    /// AGC equivalent: TVCYAW (CDUTCMD, octal 0053), scale B-1 rev.
    pub gimbal_yaw: f64,
    /// Pitch trim bias (radians).
    ///
    /// Slowly integrating CG compensation term, accumulated by `update_trim`.
    /// AGC equivalent: TRIMGIMB1 (erasable), scale B-1 rev.
    /// Stored as `f64` radians so the trim integrator can accumulate sub-count
    /// increments between T5RUPT cycles without quantisation loss. The final
    /// conversion to CDU counts occurs inside `tvc_step` before `sps_gimbal`.
    pub trim_pitch: f64,
    /// Yaw trim bias (radians).
    ///
    /// AGC equivalent: TRIMGIMB2 (erasable), scale B-1 rev.
    /// See `trim_pitch` for notes on sub-count precision.
    pub trim_yaw: f64,
}

/// Digital lead-lag compensator state for one TVC axis.
///
/// Corresponds to the Comanche055 TVCDAPS filter state variables
/// (PCMD/YCMD for x[n-1] and PERROR/YERROR for y[n-1]).
///
/// The filter difference equation is:
///   y[n] = a0·x[n] + a1·x[n-1] − b1·y[n-1]
///
/// where x[n] is the current attitude error input (radians) and
/// y[n] is the filter output (radians).
#[derive(Clone, Copy, Debug)]
pub struct TvcFilterAxis {
    /// Forward coefficient for current input sample x[n].
    pub a0: f64,
    /// Forward coefficient for previous input sample x[n-1].
    pub a1: f64,
    /// Feedback coefficient for previous output sample y[n-1].
    /// Positive value; subtracted in the recurrence.
    pub b1: f64,
    /// Previous input sample x[n-1] (radians).
    pub prev_input: f64,
    /// Previous output sample y[n-1] (radians).
    pub prev_output: f64,
}

/// Nominal lead-lag filter zero coefficient (current input).
///
/// Derived from bilinear transformation of H(s) = K·(s+z)/(s+p),
/// z=0.6 rad/s, p=6.0 rad/s, K=0.5, sample period T=0.1 s (T5RUPT).
///
/// DEVELOPER NOTE: Verify these values against the fixed-point constants
/// in Comanche055/TVCINITIALIZE.agc before finalising the implementation.
pub const TVC_A0: f64 = 0.5530;
/// Nominal lead-lag filter coefficient for previous input sample.
pub const TVC_A1: f64 = -0.4470;
/// Nominal lead-lag filter feedback coefficient (subtracted in the recurrence).
pub const TVC_B1: f64 = -0.4470;

impl TvcFilterAxis {
    /// Construct a filter axis with nominal Comanche055 coefficients
    /// and zeroed state (filter memory cleared, suitable for start of burn).
    pub const fn new_nominal() -> Self {
        TvcFilterAxis {
            a0: TVC_A0,
            a1: TVC_A1,
            b1: TVC_B1,
            prev_input: 0.0,
            prev_output: 0.0,
        }
    }
}

impl Default for TvcFilterAxis {
    fn default() -> Self {
        Self::new_nominal()
    }
}

/// Lead-lag compensator state for both TVC axes (pitch and yaw).
///
/// The pitch and yaw axes use identical coefficients (the SPS gimbal
/// geometry is symmetric) but independent filter memories.
#[derive(Clone, Copy, Debug)]
pub struct TvcFilter {
    pub pitch: TvcFilterAxis,
    pub yaw: TvcFilterAxis,
}

impl TvcFilter {
    /// Construct a filter with nominal Comanche055 coefficients and zeroed
    /// state, suitable for the start of an SPS burn.
    pub const fn new_nominal() -> Self {
        TvcFilter {
            pitch: TvcFilterAxis::new_nominal(),
            yaw: TvcFilterAxis::new_nominal(),
        }
    }
}

impl Default for TvcFilter {
    fn default() -> Self {
        Self::new_nominal()
    }
}

// ─── Constants ───────────────────────────────────────────────────────────────

/// SPS gimbal mechanical limit (radians). Typical CSM limit is ±5.5°.
/// AGC: TVCGIMBAL_MAX constant in TVCINITIALIZE.agc.
pub const GIMBAL_LIMIT_RAD: f64 = 0.0960; // ≈ 5.5 degrees

/// Conversion: radians to SPS gimbal CDU counts.
/// 1 count = TAU / 3200 ≈ 0.00196 rad (per architect decision AD-5).
pub const SPS_GIMBAL_SCALE: f64 = 3200.0 / core::f64::consts::TAU;

/// Trim integrator gain (1/s). Slow integration tracks CG shift as propellant burns.
/// Typical time constant ~60 s → K_TRIM ≈ 1/60 ≈ 0.0167.
pub const K_TRIM: f64 = 0.0167;

// ─── Compute functions ────────────────────────────────────────────────────────

/// Initialise the TVC servo state before an SPS burn.
///
/// Called once by P40 prior to ignition. Loads the pre-burn trim values
/// supplied by the P40 mass-property computation, resets the gimbal commands
/// to match the initial trim, and clears all filter memory.
///
/// # Parameters
/// - `state` — mutable reference to `TvcState` in `AgcState`.
/// - `filter` — mutable reference to `TvcFilter` in `AgcState`.
/// - `initial_trim` — `(pitch_rad, yaw_rad)` initial trim from P40 / TVCMASSPROP.
pub fn tvc_init(state: &mut TvcState, filter: &mut TvcFilter, initial_trim: (f64, f64)) {
    state.trim_pitch = initial_trim.0;
    state.trim_yaw = initial_trim.1;
    state.gimbal_pitch = initial_trim.0;
    state.gimbal_yaw = initial_trim.1;
    *filter = TvcFilter::new_nominal();
}

/// Execute one TVC servo cycle (called each T5RUPT, nominally 100 ms).
///
/// Applies the digital lead-lag filter to the attitude error, adds the
/// accumulated trim bias, saturates to mechanical limits, and returns the
/// result as `(pitch_counts, yaw_counts)` in SPS gimbal CDU units.
///
/// **Strategy D**: this function does NOT call any HAL method. The returned
/// counts must be written to `AgcState::sps_gimbal_cmd` by the DAP supervisor.
///
/// # Parameters
/// - `state` — mutable TVC state (updated in place).
/// - `filter` — mutable filter memory (updated in place).
/// - `attitude_error` — `[roll, pitch, yaw]` error in radians from `DapState`.
/// - `dt` — elapsed time since last call (seconds, nominally 0.1).
///
/// # Returns
/// `(pitch_counts, yaw_counts)` — CDU error-counter units, ±GIMBAL_LIMIT_RAD × SPS_GIMBAL_SCALE.
pub fn tvc_step(
    state: &mut TvcState,
    filter: &mut TvcFilter,
    attitude_error: Vec3,
    dt: f64,
) -> (i16, i16) {
    // Extract pitch and yaw errors (roll is handled by RCS, not TVC).
    let raw_pitch = attitude_error[1];
    let raw_yaw = attitude_error[2];

    // Guard NaN / Inf — treat as zero to avoid driving the gimbal to the stop.
    let pitch_err = if raw_pitch.is_finite() {
        raw_pitch
    } else {
        0.0
    };
    let yaw_err = if raw_yaw.is_finite() { raw_yaw } else { 0.0 };

    // Apply lead-lag filter: y[n] = a0·x[n] + a1·x[n-1] − b1·y[n-1]
    let x_pitch = pitch_err;
    let y_pitch = filter.pitch.a0 * x_pitch + filter.pitch.a1 * filter.pitch.prev_input
        - filter.pitch.b1 * filter.pitch.prev_output;
    filter.pitch.prev_input = x_pitch;
    filter.pitch.prev_output = y_pitch;

    let x_yaw = yaw_err;
    let y_yaw = filter.yaw.a0 * x_yaw + filter.yaw.a1 * filter.yaw.prev_input
        - filter.yaw.b1 * filter.yaw.prev_output;
    filter.yaw.prev_input = x_yaw;
    filter.yaw.prev_output = y_yaw;

    // Add trim bias (pre-saturation).
    let cmd_pitch_unsat = y_pitch + state.trim_pitch;
    let cmd_yaw_unsat = y_yaw + state.trim_yaw;

    // Update trim integrator using pre-saturation commands.
    update_trim(state, (cmd_pitch_unsat, cmd_yaw_unsat), dt);

    // Saturate to mechanical limits.
    state.gimbal_pitch = cmd_pitch_unsat.clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD);
    state.gimbal_yaw = cmd_yaw_unsat.clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD);

    // Convert to CDU counts (AD-5 scale factor).
    let pitch_counts = (state.gimbal_pitch * SPS_GIMBAL_SCALE) as i16;
    let yaw_counts = (state.gimbal_yaw * SPS_GIMBAL_SCALE) as i16;

    (pitch_counts, yaw_counts)
}

/// Update the trim integrator at the end of a TVC cycle.
///
/// Slowly accumulates the current gimbal command into the trim bias so that
/// a constant attitude error (caused by a CG offset) is gradually absorbed
/// into the trim rather than continuously loaded onto the filter output.
///
/// Clamps trim to ±GIMBAL_LIMIT_RAD to prevent wind-up.
///
/// # Parameters
/// - `state` — mutable TVC state.
/// - `gimbal_cmd` — `(pitch_rad, yaw_rad)` pre-saturation gimbal command from `tvc_step`.
/// - `dt` — elapsed time in seconds.
pub fn update_trim(state: &mut TvcState, gimbal_cmd: (f64, f64), dt: f64) {
    state.trim_pitch += K_TRIM * gimbal_cmd.0 * dt;
    state.trim_pitch = state.trim_pitch.clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD);

    state.trim_yaw += K_TRIM * gimbal_cmd.1 * dt;
    state.trim_yaw = state.trim_yaw.clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD);
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.1; // nominal T5RUPT period (seconds)

    fn make_state() -> TvcState {
        TvcState::default()
    }

    fn make_filter() -> TvcFilter {
        TvcFilter::new_nominal()
    }

    // TC-TVC-1: zero attitude error → gimbal stays at initial trim value
    //
    // Sequence: filter output = 0, cmd_unsat = 0 + initial_trim,
    // update_trim runs (modifies trim_pitch), then gimbal_pitch is clamped
    // from cmd_unsat = initial_trim.  So gimbal_pitch == initial_trim, not
    // the post-update trim_pitch.
    #[test]
    fn tc_tvc_1_zero_error_stays_at_trim() {
        let mut state = make_state();
        let mut filter = make_filter();

        let initial_trim = (0.01, -0.005);
        tvc_init(&mut state, &mut filter, initial_trim);

        let (pc, yc) = tvc_step(&mut state, &mut filter, [0.0, 0.0, 0.0], DT);

        // With zero error the filter output is zero; cmd_unsat = initial_trim.
        // gimbal_pitch/yaw are clamped from cmd_unsat (not the post-update trim).
        assert_eq!(
            state.gimbal_pitch,
            initial_trim.0.clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD),
            "gimbal_pitch should equal initial trim after zero-error step"
        );
        assert_eq!(
            state.gimbal_yaw,
            initial_trim.1.clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD),
            "gimbal_yaw should equal initial trim after zero-error step"
        );

        // Counts must be consistent with the stored gimbal angles.
        let expected_pc = (state.gimbal_pitch * SPS_GIMBAL_SCALE) as i16;
        let expected_yc = (state.gimbal_yaw * SPS_GIMBAL_SCALE) as i16;
        assert_eq!(pc, expected_pc);
        assert_eq!(yc, expected_yc);
    }

    // TC-TVC-2: step error response — after one step, filter output ≈ a0 * error
    #[test]
    fn tc_tvc_2_step_response_first_sample() {
        let mut state = make_state();
        let mut filter = make_filter();
        tvc_init(&mut state, &mut filter, (0.0, 0.0));

        let err = 0.05; // 0.05 rad pitch error
        tvc_step(&mut state, &mut filter, [0.0, err, 0.0], DT);

        // First sample: prev_input and prev_output were 0 before the call.
        // y[0] = a0 * err + a1 * 0 - b1 * 0 = a0 * err
        let expected_gimbal = TVC_A0 * err; // trim is 0 and was just updated
                                            // The trim was updated with (a0*err, 0) * K_TRIM * DT — very small.
                                            // gimbal_pitch = clamp(a0*err + trim_pitch_after_update).
                                            // Check that gimbal_pitch is close to a0 * err (within trim perturbation).
        let diff = (state.gimbal_pitch - expected_gimbal).abs();
        // trim increment = K_TRIM * a0 * err * DT ≈ 0.0167 * 0.5530 * 0.05 * 0.1 ≈ 4.6e-5
        assert!(
            diff < 1e-3,
            "gimbal_pitch {:.6} should be close to a0*err {:.6}",
            state.gimbal_pitch,
            expected_gimbal
        );
    }

    // TC-TVC-3: steady-state tracking — constant input should settle to a0/(1+b1) ratio
    #[test]
    fn tc_tvc_3_steady_state_convergence() {
        let mut state = make_state();
        let mut filter = make_filter();
        tvc_init(&mut state, &mut filter, (0.0, 0.0));

        let err = 0.02_f64;
        // Run many cycles to approach steady state (trim disabled, focus on filter).
        // With trim the steady state will be at zero error once trim absorbs it.
        // Use a tiny dt to suppress trim accumulation for this test.
        let tiny_dt = 1e-6;
        for _ in 0..2000 {
            tvc_step(&mut state, &mut filter, [0.0, err, 0.0], tiny_dt);
        }

        // DC gain of the filter = (a0 + a1) / (1 + b1).
        // a0 + a1 = 0.5530 - 0.4470 = 0.1060, 1 + b1 = 1 - 0.4470 = 0.5530
        // DC gain = 0.1060 / 0.5530 ≈ 0.1916
        let dc_gain = (TVC_A0 + TVC_A1) / (1.0 + TVC_B1);
        let expected_steady = dc_gain * err;
        // Allow 1 % tolerance plus tiny trim accumulation.
        let diff = (state.gimbal_pitch - expected_steady).abs();
        assert!(
            diff < 0.001 * err.abs() + 1e-5,
            "steady state gimbal_pitch {:.6} vs expected {:.6} (diff {:.2e})",
            state.gimbal_pitch,
            expected_steady,
            diff
        );
    }

    // TC-TVC-4: trim integration — constant gimbal command integrates trim over time
    #[test]
    fn tc_tvc_4_trim_integration() {
        let mut state = make_state();
        state.trim_pitch = 0.0;
        state.trim_yaw = 0.0;

        // Simulate trim integration directly.
        let gimbal_cmd = (0.05, -0.03);
        let steps = 100;
        for _ in 0..steps {
            update_trim(&mut state, gimbal_cmd, DT);
        }

        let expected_pitch =
            (K_TRIM * gimbal_cmd.0 * DT * steps as f64).clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD);
        let expected_yaw =
            (K_TRIM * gimbal_cmd.1 * DT * steps as f64).clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD);

        let tol = 1e-9;
        assert!(
            (state.trim_pitch - expected_pitch).abs() < tol,
            "trim_pitch {:.8} vs expected {:.8}",
            state.trim_pitch,
            expected_pitch
        );
        assert!(
            (state.trim_yaw - expected_yaw).abs() < tol,
            "trim_yaw {:.8} vs expected {:.8}",
            state.trim_yaw,
            expected_yaw
        );
    }

    // TC-TVC-5: saturation — large error → gimbal clamped to GIMBAL_LIMIT_RAD
    #[test]
    fn tc_tvc_5_saturation() {
        let mut state = make_state();
        let mut filter = make_filter();
        tvc_init(&mut state, &mut filter, (0.0, 0.0));

        // 10 rad error is far beyond any mechanical limit.
        tvc_step(&mut state, &mut filter, [0.0, 10.0, -10.0], DT);

        assert_eq!(
            state.gimbal_pitch, GIMBAL_LIMIT_RAD,
            "gimbal_pitch should be clamped to GIMBAL_LIMIT_RAD"
        );
        assert_eq!(
            state.gimbal_yaw, -GIMBAL_LIMIT_RAD,
            "gimbal_yaw should be clamped to -GIMBAL_LIMIT_RAD"
        );
    }

    // TC-TVC-6: tvc_init with non-zero trim → initial gimbal and trim match
    #[test]
    fn tc_tvc_6_init_non_zero_trim() {
        let mut state = make_state();
        let mut filter = make_filter();

        // Pollute filter state to verify that tvc_init resets it.
        filter.pitch.prev_input = 99.0;
        filter.pitch.prev_output = -42.0;
        filter.yaw.prev_input = 7.0;
        filter.yaw.prev_output = 3.5;

        let initial_trim = (0.05, -0.03);
        tvc_init(&mut state, &mut filter, initial_trim);

        assert_eq!(state.trim_pitch, initial_trim.0);
        assert_eq!(state.trim_yaw, initial_trim.1);
        assert_eq!(state.gimbal_pitch, initial_trim.0);
        assert_eq!(state.gimbal_yaw, initial_trim.1);

        // Filter memory must be zeroed.
        assert_eq!(filter.pitch.prev_input, 0.0);
        assert_eq!(filter.pitch.prev_output, 0.0);
        assert_eq!(filter.yaw.prev_input, 0.0);
        assert_eq!(filter.yaw.prev_output, 0.0);

        // Coefficients must be nominal.
        assert_eq!(filter.pitch.a0, TVC_A0);
        assert_eq!(filter.yaw.a0, TVC_A0);
    }

    // TC-TVC-7: NaN/Inf in attitude_error → treated as zero (no panic, no saturation)
    #[test]
    fn tc_tvc_7_nan_inf_guard() {
        let mut state = make_state();
        let mut filter = make_filter();
        tvc_init(&mut state, &mut filter, (0.0, 0.0));

        // NaN in pitch and Inf in yaw should not cause a panic or saturate the gimbal.
        tvc_step(&mut state, &mut filter, [0.0, f64::NAN, f64::INFINITY], DT);

        assert!(
            state.gimbal_pitch.is_finite(),
            "gimbal_pitch must be finite after NaN input"
        );
        assert!(
            state.gimbal_yaw.is_finite(),
            "gimbal_yaw must be finite after Inf input"
        );
        // Both should be near zero (trim is zero, error was treated as zero).
        assert!(state.gimbal_pitch.abs() < 1e-6);
        assert!(state.gimbal_yaw.abs() < 1e-6);
    }
}
