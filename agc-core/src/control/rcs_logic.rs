//! RCS jet selection logic.
//!
//! Maps desired torque commands to individual jet bitmasks for the SM RCS
//! (16 jets, 4 quads) and CM RCS (12 jets, 2 rings). Handles failed jets.

/// Configuration for the RCS jet selection logic.
///
/// Loaded from DAP data entered by the crew via V46/V48, or set to defaults
/// at FRESH START. Corresponds to the AGC erasable cells DAPBOOLS, NJETMAN,
/// and the DAP configuration words in ERASABLE_ASSIGNMENTS.agc.
///
/// All floating-point fields are SI (radians, rad/s, metres, Newtons).
/// No AGC fixed-point scaling is applied here.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (DAPBOOLS area)
#[derive(Clone, Copy, Debug)]
pub struct RcsConfig {
    /// Bitmask of SM jets that are enabled. Bit layout matches rcs-logic-spec §3.2:
    /// bits 15–8 = jets_b (channel 06), bits 7–0 = jets_a (channel 05).
    /// A `1` means the jet is enabled; a `0` means it is failed or crew-disabled.
    /// Default (FRESH START): 0xFFFF (all 16 jets enabled).
    pub sm_jet_enable_mask: u16,

    /// Bitmask of CM jets that are enabled. Bits 11–0 correspond to the CM
    /// jet table in rcs-logic-spec §3.3. Default: 0x0FFF (all 12 jets enabled).
    pub cm_jet_enable_mask: u16,

    /// Attitude deadband in radians. If the attitude error is below this
    /// value the DAP issues no torque command. Typical value: 0.5°–2.0°.
    /// Corresponds to AGC erasable ATTDB (attitude deadband).
    pub attitude_deadband_rad: f64,

    /// Rate deadband in rad/s. Body rates below this threshold are treated
    /// as zero for the purpose of rate-damping jet selection.
    /// Typical value: 0.1–0.5 °/s.
    /// Corresponds to AGC erasable RATEDB.
    pub rate_deadband_rad_s: f64,

    /// Minimum pulse duration in T6 counts (1 count = 0.625 ms).
    /// Pulses shorter than this value are rounded up to `min_pulse_counts`.
    /// Pulses that would be shorter than half this value are discarded (no fire).
    /// Original AGC value: 22 counts = 13.75 ms ≈ 14 ms.
    pub min_pulse_counts: u16,

    /// Maximum pulse duration in T6 counts. Pulses longer than this are
    /// clamped. Practical limit for a single DAP cycle: 160 counts (100 ms).
    pub max_pulse_counts: u16,

    /// Number of jets to use per axis for normal (two-jet) mode.
    /// Values: 1 (minimum impulse / low-rate maneuver) or 2 (standard).
    /// Set by crew via V46 DSKY entry. Corresponds to AGC NJETMAN variable.
    pub jets_per_axis: u8,

    /// SM RCS nominal thrust per jet, in Newtons. Used to construct the
    /// torque contribution table. Default: 445.0 N.
    pub sm_thrust_n: f64,

    /// SM RCS pitch/yaw moment arm, in metres (distance from body X axis
    /// to pitch/yaw jet thrust line). Default: 1.4 m.
    pub sm_pitch_yaw_arm_m: f64,

    /// SM RCS roll moment arm, in metres (tangential distance from body X
    /// axis to roll jet thrust line). Default: 1.9 m.
    pub sm_roll_arm_m: f64,

    /// CM RCS nominal thrust per jet, in Newtons. Used during entry.
    /// Default: 389.0 N (87.5 lbf CM RCS).
    pub cm_thrust_n: f64,

    /// CM RCS pitch/yaw moment arm, in metres. Default: 0.6 m.
    pub cm_arm_m: f64,
}

impl RcsConfig {
    /// Nominal CSM configuration at FRESH START.
    ///
    /// All 16 SM jets and all 12 CM jets enabled; deadbands and pulse
    /// durations set to the original AGC defaults.
    pub const NOMINAL: Self = Self {
        sm_jet_enable_mask: 0xFFFF,
        cm_jet_enable_mask: 0x0FFF,
        // 0.5° attitude deadband and 0.2°/s rate deadband expressed in radians.
        // These are compile-time approximations; runtime code should use
        // libm::sin/cos for high-precision conversions.
        attitude_deadband_rad: 0.008_727, // ≈ 0.5°
        rate_deadband_rad_s: 0.003_491,   // ≈ 0.2°/s
        min_pulse_counts: 22,             // 13.75 ms
        max_pulse_counts: 160,            // 100 ms (one DAP cycle)
        jets_per_axis: 2,
        sm_thrust_n: 445.0,
        sm_pitch_yaw_arm_m: 1.4,
        sm_roll_arm_m: 1.9,
        cm_thrust_n: 389.0,
        cm_arm_m: 0.6,
    };
}

impl Default for RcsConfig {
    fn default() -> Self {
        Self::NOMINAL
    }
}
