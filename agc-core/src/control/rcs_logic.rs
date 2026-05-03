//! RCS jet selection logic.
//!
//! Maps desired torque commands to individual jet bitmasks for the SM RCS
//! (16 jets, 4 quads) and CM RCS (12 jets, 2 rings). Handles failed jets.
//!
//! AGC source: Comanche055/JET_SELECTION_LOGIC.agc
//! AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc

use crate::hal::{rcs::Rcs, timers::Timers, AgcHardware};
use crate::math::linalg::{dot, norm};
use crate::types::Vec3;

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

// ── Torque contribution tables ────────────────────────────────────────────────
//
// These compile-time constants use unit-normalised torque direction vectors.
// The actual torque magnitude is thrust_n × moment_arm, applied at compute
// time via `build_sm_torque_table` / `build_cm_torque_table`.
//
// The layout below matches the canonical bit assignment from rcs-logic-spec §3.2:
//   index 0  = bit 0  (B4, -Yaw)
//   index 1  = bit 1  (B3, +Yaw)
//   index 2  = bit 2  (B2, -Pitch)
//   index 3  = bit 3  (B1, +Pitch)
//   index 4  = bit 4  (A4, -Yaw)
//   index 5  = bit 5  (A3, +Yaw)
//   index 6  = bit 6  (A2, -Pitch)
//   index 7  = bit 7  (A1, +Pitch)
//   index 8  = bit 8  (D4, -Roll redundant)
//   index 9  = bit 9  (D3, +Roll redundant)
//   index 10 = bit 10 (D2, -Roll)
//   index 11 = bit 11 (D1, +Roll)
//   index 12 = bit 12 (C4, -Yaw)
//   index 13 = bit 13 (C3, +Yaw)
//   index 14 = bit 14 (C2, -Pitch)
//   index 15 = bit 15 (C1, +Pitch)
//
// Axis convention (body frame):
//   [0] = X = roll axis  (positive forward through hatch)
//   [1] = Y = pitch axis (positive out CM window side)
//   [2] = Z = yaw axis   (completes right-hand frame)
//
// The real AGC had a more complex geometry with canted jet angles, but these
// unit-axis vectors are a working approximation that preserves sign correctness
// and enables correct jet selection per the spec algorithm.

/// Torque contribution unit-direction vectors for the 16 SM RCS jets (body frame).
/// Index matches bit position in the u16 jet mask (bit 0 = jet 0 = index 0).
/// Actual torque magnitude = thrust_n × moment_arm; see `build_sm_torque_table`.
pub const SM_JET_TORQUES: [Vec3; 16] = [
    [0.0, 0.0, -1.0], // bit  0: B4  -Yaw
    [0.0, 0.0, 1.0],  // bit  1: B3  +Yaw
    [0.0, -1.0, 0.0], // bit  2: B2  -Pitch
    [0.0, 1.0, 0.0],  // bit  3: B1  +Pitch
    [0.0, 0.0, -1.0], // bit  4: A4  -Yaw
    [0.0, 0.0, 1.0],  // bit  5: A3  +Yaw
    [0.0, -1.0, 0.0], // bit  6: A2  -Pitch
    [0.0, 1.0, 0.0],  // bit  7: A1  +Pitch
    [-1.0, 0.0, 0.0], // bit  8: D4  -Roll (redundant)
    [1.0, 0.0, 0.0],  // bit  9: D3  +Roll (redundant)
    [-1.0, 0.0, 0.0], // bit 10: D2  -Roll
    [1.0, 0.0, 0.0],  // bit 11: D1  +Roll
    [0.0, 0.0, -1.0], // bit 12: C4  -Yaw  (quad C)
    [0.0, 0.0, 1.0],  // bit 13: C3  +Yaw  (quad C)
    [0.0, -1.0, 0.0], // bit 14: C2  -Pitch (quad C)
    [0.0, 1.0, 0.0],  // bit 15: C1  +Pitch (quad C)
];

/// Torque contribution unit-direction vectors for the 12 CM RCS jets (body frame).
/// Index matches bit position 0–11 in the u16 jet mask. Bits 15–12 are always 0.
///
/// CM jet bit assignments (rcs-logic-spec §3.3):
///   bit 11: F1 Fwd +Pitch   bit 10: F2 Fwd -Pitch
///   bit  9: F3 Fwd +Yaw     bit  8: F4 Fwd -Yaw
///   bit  7: F5 Fwd +Roll    bit  6: F6 Fwd -Roll
///   bit  5: A1 Aft +Pitch   bit  4: A2 Aft -Pitch
///   bit  3: A3 Aft +Yaw     bit  2: A4 Aft -Yaw
///   bit  1: A5 Aft +Roll    bit  0: A6 Aft -Roll
pub const CM_JET_TORQUES: [Vec3; 12] = [
    [-1.0, 0.0, 0.0], // bit  0: A6 Aft -Roll
    [1.0, 0.0, 0.0],  // bit  1: A5 Aft +Roll
    [0.0, 0.0, -1.0], // bit  2: A4 Aft -Yaw
    [0.0, 0.0, 1.0],  // bit  3: A3 Aft +Yaw
    [0.0, -1.0, 0.0], // bit  4: A2 Aft -Pitch
    [0.0, 1.0, 0.0],  // bit  5: A1 Aft +Pitch
    [-1.0, 0.0, 0.0], // bit  6: F6 Fwd -Roll
    [1.0, 0.0, 0.0],  // bit  7: F5 Fwd +Roll
    [0.0, 0.0, -1.0], // bit  8: F4 Fwd -Yaw
    [0.0, 0.0, 1.0],  // bit  9: F3 Fwd +Yaw
    [0.0, -1.0, 0.0], // bit 10: F2 Fwd -Pitch
    [0.0, 1.0, 0.0],  // bit 11: F1 Fwd +Pitch
];

// ── Table builders ────────────────────────────────────────────────────────────

/// Construct the SM jet torque contribution table (N·m, body frame) from `config`.
///
/// Returns a `[Vec3; 16]` where index `i` is the torque vector of the jet
/// corresponding to bit `i` of the u16 combined jet mask (§3.2).
///
/// Called once during `RcsConfig` construction; recompute if config changes.
pub fn build_sm_torque_table(config: &RcsConfig) -> [Vec3; 16] {
    let mut table = [[0.0f64; 3]; 16];
    for i in 0..16 {
        let dir = SM_JET_TORQUES[i];
        // X component uses roll moment arm; Y/Z use pitch-yaw arm.
        let arm = if dir[0] != 0.0 {
            config.sm_roll_arm_m
        } else {
            config.sm_pitch_yaw_arm_m
        };
        let mag = config.sm_thrust_n * arm;
        table[i] = [dir[0] * mag, dir[1] * mag, dir[2] * mag];
    }
    table
}

/// Construct the CM jet torque contribution table (N·m, body frame) from `config`.
///
/// Returns a `[Vec3; 12]` where index `i` corresponds to bit `i` of the u16 mask.
pub fn build_cm_torque_table(config: &RcsConfig) -> [Vec3; 12] {
    let mut table = [[0.0f64; 3]; 12];
    let mag = config.cm_thrust_n * config.cm_arm_m;
    for i in 0..12 {
        let dir = CM_JET_TORQUES[i];
        table[i] = [dir[0] * mag, dir[1] * mag, dir[2] * mag];
    }
    table
}

// ── Generic jet-selection helper ──────────────────────────────────────────────

/// Select jets from a torque table given an enable mask and a torque command.
///
/// Returns a bitmask with at most `jets_per_axis` bits set per axis
/// (roll=X, pitch=Y, yaw=Z). An axis is considered "active" if its torque
/// component exceeds 15% of the total command magnitude, preventing
/// unnecessary cross-coupling firings for nearly-single-axis commands.
///
/// Jets disabled in `enable_mask` are never selected.
fn select_jets_generic<const N: usize>(
    torque_cmd: Vec3,
    torque_table: &[Vec3; N],
    enable_mask: u16,
    jets_per_axis: u8,
) -> u16 {
    let total_mag = norm(torque_cmd);
    if total_mag < 1e-10 {
        return 0;
    }

    // Coupling threshold: an axis is "inactive" if |component| < 15% of total.
    let threshold = total_mag * 0.15;

    let jets_per_axis = match jets_per_axis {
        1 => 1usize,
        _ => 2usize, // default to 2; non-1/2 values fall back to 2 here
    };

    // For each of the three axes, collect candidate jets with a positive dot
    // product, then select the top `jets_per_axis` by score.
    let axis_active = [
        torque_cmd[0].abs() >= threshold, // roll  (X)
        torque_cmd[1].abs() >= threshold, // pitch (Y)
        torque_cmd[2].abs() >= threshold, // yaw   (Z)
    ];

    let mut result: u16 = 0;

    for axis in 0..3 {
        if !axis_active[axis] {
            continue;
        }

        // Gather candidates: dot product of jet torque with full torque_cmd > 0,
        // and only along this axis' contribution (positive score = helpful for
        // the axis sign requested).
        // Score is the projection of the jet's torque onto the command vector.
        let mut candidates: [(usize, f64); 16] = [(0, 0.0); 16];
        let mut n_cands = 0usize;

        for (i, &jet_torque) in torque_table.iter().enumerate().take(N) {
            if enable_mask & (1u16 << i) == 0 {
                continue; // jet disabled
            }
            // Only accept jets that contribute positively on this specific axis.
            let axis_contribution = jet_torque[axis] * torque_cmd[axis];
            if axis_contribution > 0.0 {
                let score = dot(jet_torque, torque_cmd);
                if score > 0.0 {
                    candidates[n_cands] = (i, score);
                    n_cands += 1;
                }
            }
        }

        // Sort by score descending (simple insertion sort — N is tiny).
        for i in 1..n_cands {
            let mut j = i;
            while j > 0 && candidates[j].1 > candidates[j - 1].1 {
                candidates.swap(j, j - 1);
                j -= 1;
            }
        }

        // Select up to jets_per_axis jets.
        let count = n_cands.min(jets_per_axis);
        for cand in candidates.iter().take(count) {
            result |= 1u16 << cand.0;
        }
    }

    result
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Select SM RCS jets for a desired torque command.
///
/// Returns a 16-bit jet bitmask encoding the selected jets:
///   - bits 15–8: jets_b — AGC output channel 06 (ROLLJETS)
///   - bits  7–0: jets_a — AGC output channel 05 (PYJETS)
///
/// The T5RUPT ISR shim splits this u16 when calling `fire_sm_jets`:
/// ```text
///   jets_a = (result & 0x00FF) as u8;   // channel 05
///   jets_b = (result >> 8)    as u8;    // channel 06
/// ```
///
/// Returns `0x0000` if `torque_cmd` is the zero vector or if all
/// contributing jets are disabled in `config`.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc
pub fn select_jets_sm(torque_cmd: Vec3, config: &RcsConfig) -> u16 {
    let table = build_sm_torque_table(config);
    select_jets_generic(
        torque_cmd,
        &table,
        config.sm_jet_enable_mask,
        config.jets_per_axis,
    )
}

/// Select CM RCS jets for a desired torque command (entry phase only).
///
/// Returns a 12-bit jet bitmask for `fire_cm_jets`. Bits 15–12 are always 0.
/// Returns 0 if `torque_cmd` is zero or all relevant jets are disabled.
///
/// The CM RCS is used only after SM/CM separation.
///
/// AGC source: Comanche055/CM_ENTRY_DIGITAL_AUTOPILOT.agc
pub fn select_jets_cm(torque_cmd: Vec3, config: &RcsConfig) -> u16 {
    let table = build_cm_torque_table(config);
    let raw = select_jets_generic(
        torque_cmd,
        &table,
        config.cm_jet_enable_mask,
        config.jets_per_axis,
    );
    // Enforce postcondition: bits 15–12 must always be zero.
    raw & 0x0FFF
}

/// Compute the T6 pulse duration for a jet firing.
///
/// `torque_cmd` is the desired torque vector (N·m).
/// `jet_mask` is the u16 jet bitmask returned by `select_jets_sm` or
/// `select_jets_cm`.
/// `moment_of_inertia` holds the principal body-frame inertia components
/// `[Ixx, Iyy, Izz]` in kg·m².
///
/// Returns a count value for `arm_t6`: duration = counts × 0.625 ms.
/// Returns 0 if the computed duration is below half the minimum pulse
/// threshold (pulse should be discarded — caller must not fire).
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc (minimum impulse logic)
pub fn compute_pulse_duration(
    torque_cmd: Vec3,
    jet_mask: u16,
    config: &RcsConfig,
    moment_of_inertia: Vec3, // [Ixx, Iyy, Izz] in kg·m²
) -> u16 {
    // Step 1: sum the effective torque produced by the selected jets.
    let sm_table = build_sm_torque_table(config);
    let mut tau_eff: Vec3 = [0.0; 3];
    for (i, t) in sm_table.iter().enumerate() {
        if jet_mask & (1u16 << i) != 0 {
            tau_eff[0] += t[0];
            tau_eff[1] += t[1];
            tau_eff[2] += t[2];
        }
    }

    // Step 2: find the dominant axis (largest |torque_cmd| component).
    let magnitudes = [
        torque_cmd[0].abs(),
        torque_cmd[1].abs(),
        torque_cmd[2].abs(),
    ];
    let dominant_axis = if magnitudes[0] >= magnitudes[1] && magnitudes[0] >= magnitudes[2] {
        0
    } else if magnitudes[1] >= magnitudes[2] {
        1
    } else {
        2
    };

    let i_axis = moment_of_inertia[dominant_axis];
    let tau_axis = tau_eff[dominant_axis];

    // Guard against zero effective torque on the dominant axis (all jets failed
    // or jet_mask == 0). Return 0 — no fire.
    if tau_axis.abs() < 1e-10 {
        return 0;
    }

    // Step 4: t_fire = |torque_cmd[axis]| * I_axis / |tau_eff[axis]|
    let t_fire = magnitudes[dominant_axis] * i_axis / tau_axis.abs();

    // Step 5: convert to T6 counts (1 count = 0.000625 s).
    let counts_f = t_fire / 0.000_625;
    let counts_rounded = libm::round(counts_f) as u64;

    // Step 6: apply minimum/maximum limits.
    let half_min = (config.min_pulse_counts / 2) as u64;
    if counts_rounded < half_min {
        return 0; // discard — too short to fire
    }

    if counts_rounded < config.min_pulse_counts as u64 {
        config.min_pulse_counts
    } else if counts_rounded > config.max_pulse_counts as u64 {
        config.max_pulse_counts
    } else {
        counts_rounded as u16
    }
}

/// Arm T6 and fire the specified SM RCS jets as a single atomic sequence.
///
/// **ISR-shim only — do NOT call from Waitlist tasks (Strategy D).**
///
/// This function arms the T6 timer first, then immediately writes the jet
/// mask to channels 05/06. The T6RUPT handler is responsible for calling
/// `hw.rcs().quench_all()` to terminate the pulse.
///
/// **Ordering invariant**: `arm_t6` must complete before `fire_sm_jets`.
/// This matches the AGC's instruction-level ordering in the PYJETS/ROLLJETS
/// output sequence.
///
/// Precondition: `duration_counts >= 1`. Callers must check the return value
/// of `compute_pulse_duration` and skip this call if it returns 0.
///
/// The `jet_mask` parameter is the u16 returned by `select_jets_sm`:
///   - bits 15–8: jets_b → `fire_sm_jets` second arg (channel 06)
///   - bits  7–0: jets_a → `fire_sm_jets` first arg  (channel 05)
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc (PYJETS/ROLLJETS
/// output sequence with channel 13 T6 enable).
pub fn fire_pulse<H: AgcHardware>(hw: &mut H, jet_mask: u16, duration_counts: u16) {
    if jet_mask == 0 || duration_counts == 0 {
        return;
    }
    let jets_a = (jet_mask & 0x00FF) as u8; // channel 05 (PYJETS)
    let jets_b = ((jet_mask >> 8) & 0xFF) as u8; // channel 06 (ROLLJETS)
                                                 // Arm T6 FIRST, then fire jets — mandatory ordering per spec §2 and §6.3.
    hw.timers().arm_t6(duration_counts);
    hw.rcs().fire_sm_jets(jets_a, jets_b);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── TC-RCS-1: zero torque → no jets selected ─────────────────────────────

    #[test]
    fn tc_rcs_1_zero_torque_sm() {
        let config = RcsConfig::NOMINAL;
        let mask = select_jets_sm([0.0, 0.0, 0.0], &config);
        assert_eq!(mask, 0x0000, "zero torque must return empty mask");
    }

    #[test]
    fn tc_rcs_1_zero_torque_cm() {
        let config = RcsConfig::NOMINAL;
        let mask = select_jets_cm([0.0, 0.0, 0.0], &config);
        assert_eq!(mask, 0x0000, "zero torque must return empty mask for CM");
    }

    // ── TC-RCS-2: pure +roll torque → selects jets with +roll contribution ───

    #[test]
    fn tc_rcs_2_pure_positive_roll() {
        let config = RcsConfig::NOMINAL;
        // Pure +X torque → should select D1 (bit 11) and/or D3 (bit 9)
        let mask = select_jets_sm([1.0, 0.0, 0.0], &config);
        assert_ne!(mask, 0, "non-zero roll torque must select at least one jet");
        // All selected jets must be +roll jets (bits 9 and 11 only for +roll).
        let positive_roll_bits: u16 = (1 << 9) | (1 << 11);
        assert_eq!(
            mask & !positive_roll_bits,
            0,
            "only +roll jet bits (9, 11) should be set for pure +X torque, got {:#06x}",
            mask
        );
        // At least one +roll jet should be selected.
        assert_ne!(mask & positive_roll_bits, 0);
    }

    #[test]
    fn tc_rcs_2_pure_negative_roll() {
        let config = RcsConfig::NOMINAL;
        // Pure -X torque → should select D2 (bit 10) and/or D4 (bit 8)
        let mask = select_jets_sm([-1.0, 0.0, 0.0], &config);
        assert_ne!(mask, 0);
        let negative_roll_bits: u16 = (1 << 8) | (1 << 10);
        assert_eq!(
            mask & !negative_roll_bits,
            0,
            "only -roll jet bits (8, 10) should be set for pure -X torque, got {:#06x}",
            mask
        );
    }

    #[test]
    fn tc_rcs_2_pure_positive_pitch() {
        let config = RcsConfig::NOMINAL;
        // Pure +Y torque → A1 (bit 7), B1 (bit 3), C1 (bit 15)
        let mask = select_jets_sm([0.0, 1.0, 0.0], &config);
        assert_ne!(mask, 0);
        let positive_pitch_bits: u16 = (1 << 3) | (1 << 7) | (1 << 15);
        assert_eq!(
            mask & !positive_pitch_bits,
            0,
            "only +pitch jet bits (3, 7, 15) expected for pure +Y torque, got {:#06x}",
            mask
        );
    }

    #[test]
    fn tc_rcs_2_pure_positive_yaw() {
        let config = RcsConfig::NOMINAL;
        // Pure +Z torque → B3 (bit 1), A3 (bit 5), C3 (bit 13)
        let mask = select_jets_sm([0.0, 0.0, 1.0], &config);
        assert_ne!(mask, 0);
        let positive_yaw_bits: u16 = (1 << 1) | (1 << 5) | (1 << 13);
        assert_eq!(
            mask & !positive_yaw_bits,
            0,
            "only +yaw jet bits (1, 5, 13) expected for pure +Z torque, got {:#06x}",
            mask
        );
    }

    // ── TC-RCS-3: jet failure — disabled jet must not appear in result ────────

    #[test]
    fn tc_rcs_3_failed_jet_excluded() {
        let mut config = RcsConfig::NOMINAL;
        // Disable +roll jet D1 (bit 11) and D3 (bit 9).
        // Only the redundant pair is available — but both are disabled too.
        // Actually let us disable only one: bit 11 (D1).
        config.sm_jet_enable_mask = !(1u16 << 11);

        let mask = select_jets_sm([1.0, 0.0, 0.0], &config);
        // Bit 11 (D1) must NOT be in the result.
        assert_eq!(
            mask & (1 << 11),
            0,
            "disabled jet (bit 11) must not appear in selection, got {:#06x}",
            mask
        );
        // D3 (bit 9) should still be available.
        assert_ne!(
            mask & (1 << 9),
            0,
            "D3 (bit 9) should be selected as the only +roll jet"
        );
    }

    #[test]
    fn tc_rcs_3_all_roll_jets_disabled() {
        let mut config = RcsConfig::NOMINAL;
        // Disable all four roll jets (bits 8, 9, 10, 11).
        config.sm_jet_enable_mask = !((1u16 << 8) | (1 << 9) | (1 << 10) | (1 << 11));

        // Pure roll command — no jets can be selected.
        let mask = select_jets_sm([1.0, 0.0, 0.0], &config);
        assert_eq!(
            mask, 0,
            "no usable roll jets → must return 0, got {:#06x}",
            mask
        );
    }

    // ── TC-RCS-4: compute_pulse_duration in [min, max] ────────────────────────

    #[test]
    fn tc_rcs_4_pulse_duration_in_range() {
        let config = RcsConfig::NOMINAL;
        // Reasonable torque command and inertia — should produce a duration in range.
        let torque_cmd: Vec3 = [0.0, 100.0, 0.0]; // 100 N·m pitch
        let inertia: Vec3 = [80_000.0, 100_000.0, 90_000.0]; // kg·m²
        let jet_mask = select_jets_sm(torque_cmd, &config);
        assert_ne!(jet_mask, 0);

        let counts = compute_pulse_duration(torque_cmd, jet_mask, &config, inertia);
        assert_ne!(
            counts, 0,
            "reasonable torque must produce a non-zero pulse duration"
        );
        assert!(
            counts >= config.min_pulse_counts && counts <= config.max_pulse_counts,
            "pulse duration {} not in [{}, {}]",
            counts,
            config.min_pulse_counts,
            config.max_pulse_counts
        );
    }

    // ── TC-RCS-5: tiny torque → pulse discarded (returns 0) ──────────────────

    #[test]
    fn tc_rcs_5_tiny_torque_discarded() {
        let config = RcsConfig::NOMINAL;
        // Very small torque. For discard: t_fire < (min_pulse_counts/2) * 0.000625
        // = 11 * 0.000625 = 0.006875 s. With tau_eff ≈ 623 N·m and small inertia,
        // torque_needed < 0.006875 * 623 / 1.0 ≈ 4.28 N·m requires tiny inertia.
        // Use small inertia = 1.0 kg·m² and very small torque = 0.001 N·m:
        // t_fire = 0.001 * 1.0 / 623 ≈ 1.6e-6 s → 0.00257 counts → 0 after rounding.
        let torque_cmd: Vec3 = [0.0, 0.001, 0.0]; // 0.001 N·m
        let inertia: Vec3 = [1.0, 1.0, 1.0]; // tiny inertia → tiny impulse
        let jet_mask = select_jets_sm(torque_cmd, &config);
        let counts = compute_pulse_duration(torque_cmd, jet_mask, &config, inertia);
        assert_eq!(
            counts, 0,
            "tiny torque must be discarded (return 0), got {}",
            counts
        );
    }

    // ── TC-RCS-6: u16 mask split into jets_a / jets_b ────────────────────────

    #[test]
    fn tc_rcs_6_mask_split_lower_byte() {
        // Verify that the lower byte (bits 7-0) maps to jets_a correctly.
        let jet_mask: u16 = 0b_1010_0101_0110_1001; // 0xA569
        let jets_a = (jet_mask & 0x00FF) as u8;
        let jets_b = ((jet_mask >> 8) & 0xFF) as u8;
        assert_eq!(jets_a, 0x69, "lower byte should be 0x69");
        assert_eq!(jets_b, 0xA5, "upper byte should be 0xA5");
    }

    #[test]
    fn tc_rcs_6_mask_split_round_trip() {
        // Reconstructing the u16 from the two bytes must yield the original.
        let original: u16 = 0xBEEF;
        let jets_a = (original & 0x00FF) as u8;
        let jets_b = ((original >> 8) & 0xFF) as u8;
        let reconstructed = (jets_a as u16) | ((jets_b as u16) << 8);
        assert_eq!(reconstructed, original, "round-trip split must be lossless");
    }

    #[test]
    fn tc_rcs_6_select_sm_mask_only_uses_16_bits() {
        // select_jets_sm must never set bits above bit 15. The u16 return type
        // enforces this structurally; this test exists to document the intent.
        let config = RcsConfig::NOMINAL;
        let _: u16 = select_jets_sm([1.0, 1.0, 1.0], &config);
    }

    // ── TC-RCS-7: CM jet selection uses cm_jet_enable_mask, bits 15-12 = 0 ───

    #[test]
    fn tc_rcs_7_cm_returns_12_bit_mask() {
        let config = RcsConfig::NOMINAL;
        let mask = select_jets_cm([0.0, 1.0, 0.0], &config);
        assert_eq!(
            mask & 0xF000,
            0,
            "CM mask must have bits 15-12 cleared, got {:#06x}",
            mask
        );
        assert_ne!(mask, 0, "non-zero torque should produce a non-zero CM mask");
    }

    #[test]
    fn tc_rcs_7_cm_uses_cm_enable_mask() {
        let mut config = RcsConfig::NOMINAL;
        // Disable all CM jets.
        config.cm_jet_enable_mask = 0x0000;
        let mask = select_jets_cm([0.0, 1.0, 0.0], &config);
        assert_eq!(
            mask, 0,
            "all CM jets disabled → must return 0, got {:#06x}",
            mask
        );
    }

    #[test]
    fn tc_rcs_7_cm_disable_single_jet() {
        let mut config = RcsConfig::NOMINAL;
        // Disable CM bit 11 (F1, +Pitch) and bit 5 (A1, +Pitch) — two of the +pitch jets.
        config.cm_jet_enable_mask = 0x0FFF & !((1u16 << 11) | (1u16 << 5));

        let mask = select_jets_cm([0.0, 1.0, 0.0], &config);
        assert_eq!(mask & (1 << 11), 0, "disabled CM bit 11 must not appear");
        assert_eq!(mask & (1 << 5), 0, "disabled CM bit 5 must not appear");
        // Bits 15-12 still clear.
        assert_eq!(mask & 0xF000, 0);
    }

    // ── Additional edge-case tests ────────────────────────────────────────────

    #[test]
    fn tc_rcs_no_fire_when_jet_mask_zero() {
        // compute_pulse_duration with jet_mask=0 → tau_eff is zero → returns 0.
        let config = RcsConfig::NOMINAL;
        let counts = compute_pulse_duration(
            [0.0, 100.0, 0.0],
            0u16,
            &config,
            [80_000.0, 100_000.0, 90_000.0],
        );
        assert_eq!(counts, 0, "zero jet_mask must return 0 pulse duration");
    }

    #[test]
    fn tc_rcs_pulse_clamped_to_max() {
        // Very large torque command: duration should be clamped to max_pulse_counts.
        let config = RcsConfig::NOMINAL;
        // Use an enormous inertia to force a very long duration.
        let inertia: Vec3 = [1e12, 1e12, 1e12];
        let torque_cmd: Vec3 = [0.0, 1.0, 0.0];
        let jet_mask = select_jets_sm(torque_cmd, &config);
        let counts = compute_pulse_duration(torque_cmd, jet_mask, &config, inertia);
        assert_eq!(
            counts, config.max_pulse_counts,
            "very large inertia must clamp to max_pulse_counts"
        );
    }
}
