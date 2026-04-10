//! Thrust Vector Control state for SPS burns.

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
