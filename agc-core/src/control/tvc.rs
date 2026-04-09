//! Thrust Vector Control (TVC) for SPS engine burns.
//!
//! During an SPS burn the TVC DAP steers the engine gimbal to maintain the
//! desired attitude. The outer loop (`tvc_executive_update`) runs every 0.5 s;
//! the inner pitch/yaw DAP filter runs every 0.1 s on T5RUPT.
//!
//! AGC source: TVCEXECUTIVE.agc — TVCEXEC outer loop.
//!             TVCDAPS.agc — TVC pitch/yaw DAP filter.

/// TVC state maintained across DAP cycles during an SPS burn.
///
/// AGC source: TVCEXECUTIVE.agc — TVC variable storage (TVCPITCH, TVCYAW, etc.)
pub struct TvcState {
    /// Commanded pitch gimbal angle (rad).
    pub pitch_cmd: f64,
    /// Commanded yaw gimbal angle (rad).
    pub yaw_cmd: f64,
    /// Current pitch gimbal trim offset (rad).
    pub pitch_trim: f64,
    /// Current yaw gimbal trim offset (rad).
    pub yaw_trim: f64,
    /// Proportional gain applied to attitude error (updated by TVCEXECUTIVE).
    pub gain: f64,
    /// Whether TVC is active (cleared when engine is not firing).
    pub active: bool,
}

impl TvcState {
    /// Construct initial (inactive) TVC state.
    pub const fn new() -> Self {
        Self {
            pitch_cmd: 0.0,
            pitch_trim: 0.0,
            yaw_cmd: 0.0,
            yaw_trim: 0.0,
            gain: 1.0,
            active: false,
        }
    }
}

impl Default for TvcState {
    fn default() -> Self {
        Self::new()
    }
}

/// Gimbal deflection limit (±6°) in radians.
///
/// The SPS gimbal hardware is limited to ±6° per axis.
/// AGC source: TVCEXECUTIVE.agc — gimbal limit check.
const GIMBAL_LIMIT_RAD: f64 = 6.0 * (core::f64::consts::PI / 180.0);

/// Scale factor: CDU counts per radian for the TVC gimbal.
///
/// `command_gimbal` uses i16 counts where 1 count ≈ 0.01° (from engine.rs).
/// 1 rad = 180/π ° → counts = rad * (180/π) / 0.01 = rad * 18000/π
const RAD_TO_GIMBAL_COUNTS: f64 = 18000.0 / core::f64::consts::PI;

/// Compute TVC gimbal commands from attitude error.
///
/// Applies the proportional gain stored in `state`, adds the trim offset, and
/// saturates at the hardware gimbal limit. Returns `(pitch_counts, yaw_counts)`
/// for `Engine::command_gimbal`.
///
/// If TVC is not active (engine not firing) both outputs are zero.
///
/// AGC source: TVCDAPS.agc — pitch/yaw DAP filter, TVCPITCH/TVCYAW computation.
pub fn tvc_command(error_pitch: f64, error_yaw: f64, state: &TvcState) -> (i16, i16) {
    if !state.active {
        return (0, 0);
    }

    let pitch_rad = clamp(
        state.gain * error_pitch + state.pitch_trim,
        -GIMBAL_LIMIT_RAD,
        GIMBAL_LIMIT_RAD,
    );
    let yaw_rad = clamp(
        state.gain * error_yaw + state.yaw_trim,
        -GIMBAL_LIMIT_RAD,
        GIMBAL_LIMIT_RAD,
    );

    let pitch_counts = (pitch_rad * RAD_TO_GIMBAL_COUNTS) as i16;
    let yaw_counts = (yaw_rad * RAD_TO_GIMBAL_COUNTS) as i16;

    (pitch_counts, yaw_counts)
}

/// TVC executive: update gain and trim from vehicle mass.
///
/// Called every 0.5 s during SPS burns. As propellant is consumed the vehicle
/// mass decreases; gain is scaled inversely with mass (heavier vehicle needs
/// less gimbal deflection per unit error to achieve the same angular
/// acceleration, so the simple model scales gain linearly with reference mass
/// divided by current mass).
///
/// `vehicle_mass_kg` — current estimated vehicle mass in kg.
///
/// AGC source: TVCEXECUTIVE.agc — TVCEXEC gain/trim update.
pub fn tvc_executive_update(state: &mut TvcState, vehicle_mass_kg: f64) {
    // Reference mass ~28 800 kg (CSM fully loaded at TLI).
    const REFERENCE_MASS_KG: f64 = 28_800.0;
    // Clamp mass to a sensible minimum to avoid division by near-zero.
    const MIN_MASS_KG: f64 = 10_000.0;

    let mass = if vehicle_mass_kg < MIN_MASS_KG {
        MIN_MASS_KG
    } else {
        vehicle_mass_kg
    };

    state.gain = REFERENCE_MASS_KG / mass;
}

/// Clamp `v` to `[lo, hi]`.
#[inline]
fn clamp(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inactive_tvc_returns_zero() {
        let state = TvcState::new();
        assert!(!state.active);
        let (p, y) = tvc_command(1.0, 1.0, &state);
        assert_eq!(p, 0, "inactive TVC pitch must be zero");
        assert_eq!(y, 0, "inactive TVC yaw must be zero");
    }

    #[test]
    fn zero_error_zero_output() {
        let mut state = TvcState::new();
        state.active = true;
        let (p, y) = tvc_command(0.0, 0.0, &state);
        assert_eq!(p, 0);
        assert_eq!(y, 0);
    }

    #[test]
    fn gimbal_command_scales_correctly() {
        let mut state = TvcState::new();
        state.active = true;
        state.gain = 1.0;
        // 1° pitch error → should produce ~ (180/π)/0.01 / 180 * 100 counts ≈ 100 counts.
        let one_deg = core::f64::consts::PI / 180.0;
        let (p, _) = tvc_command(one_deg, 0.0, &state);
        // 1° = 100 counts (since 1 count = 0.01°).
        assert!((p - 100).abs() <= 1, "1° should be ~100 counts, got {p}");
    }

    #[test]
    fn gimbal_saturates_at_limit() {
        let mut state = TvcState::new();
        state.active = true;
        state.gain = 1.0;
        // Feed a huge error; output must be clamped to ±6° in counts.
        let max_counts = (GIMBAL_LIMIT_RAD * RAD_TO_GIMBAL_COUNTS) as i16;
        let (p, y) = tvc_command(1.0, -1.0, &state);
        assert_eq!(p, max_counts, "pitch must saturate at +6°");
        assert_eq!(y, -max_counts, "yaw must saturate at -6°");
    }

    #[test]
    fn tvc_executive_updates_gain() {
        let mut state = TvcState::new();
        // At reference mass gain should be ~1.
        tvc_executive_update(&mut state, 28_800.0);
        let diff = (state.gain - 1.0).abs();
        assert!(diff < 1e-10, "gain at reference mass should be 1.0, got {}", state.gain);
    }

    #[test]
    fn tvc_executive_increases_gain_lighter_vehicle() {
        let mut state = TvcState::new();
        tvc_executive_update(&mut state, 14_400.0); // half reference mass
        let diff = (state.gain - 2.0).abs();
        assert!(diff < 1e-10, "gain at half mass should be 2.0, got {}", state.gain);
    }

    #[test]
    fn trim_offset_shifts_output() {
        let mut state = TvcState::new();
        state.active = true;
        state.gain = 1.0;
        state.pitch_trim = core::f64::consts::PI / 180.0; // 1° trim
        let (p, _) = tvc_command(0.0, 0.0, &state);
        // Should command 1° trim → ~100 counts.
        assert!((p - 100).abs() <= 1, "trim offset should shift output ~100 counts, got {p}");
    }
}
