//! P51/P52 — IMU Alignment Programs.
//!
//! P51 is the initial coarse+fine alignment program. P52 is the in-flight
//! fine realignment program. Both share the same alignment state machine.
//! P53 (backup alignment) is deferred and not implemented.
//!
//! AGC source: Comanche055/P51-P53.agc
//!   PROG52 (page 737), R51 (page 756), R52 (page 743), R55 (page 759),
//!   CAL53A (page 762), PICAPAR (page 752), CHKSDATA/R54 (page 760).
//! AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc
//!   CALCGTA (page 1355), AXISGEN (page 1361).
//!
//! P53 is out of scope for Milestone 4 (P53 is Luminary-specific, not present
//! in Comanche055). See P51-P53.agc page 737 header.

use crate::control::imu_control::FineAlignError;
use crate::hal::{AgcHardware, DskyIo, ImuIo};
use crate::math::linalg::dot;
use crate::services::alarm::{AlarmCode, AlarmState};
use crate::types::Vec3;
use crate::AgcState;

// ── Minimum star separation angle ─────────────────────────────────────────────

/// Minimum angular separation between star pair (in dot-product terms).
///
/// Stars must not be nearly collinear. CHKSDATA checks separation ≥ 40°.
/// cos(40°) ≈ 0.766, so |dot| ≤ 0.766 for valid pair.
///
/// AGC source: Comanche055/P51-P53.agc CHKSDATA (page 760).
const MIN_STAR_SEPARATION_COS: f64 = 0.766; // cos(40°)

// ── Types ─────────────────────────────────────────────────────────────────────

/// Alignment orientation option (AGC: OPTION2 register, P52B).
///
/// AGC source: Comanche055/P51-P53.agc PROG52 lines: OPTION2 bits select the path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlignOption {
    /// Preferred orientation: XSMD/YSMD/ZSMD set by prior burn program.
    /// AGC: OPTION2 bit 2 set → P52J path (page 739).
    Preferred,
    /// Nominal orientation: computed from current R and V vectors.
    /// AGC: OPTION2 bit 2 clear → P52T path → S52.3 (page 739).
    Nominal,
    /// REFSMMAT correction: corrects drift since last alignment.
    /// AGC: OPTION2 bits 1,0 → P52C path → GYCRS (page 740).
    RefSmmat,
}

/// Monotonic phase of the IMU alignment state machine.
///
/// AGC source: Derived from PHASCHNG calls in PROG52, R51, R52 (P51-P53.agc).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum AlignPhase {
    /// Displaying and waiting for crew orientation selection.
    /// AGC label: P52B (GOPERF4R flash on DSKY).
    PromptRefsmmat,
    /// Coarse alignment in progress (P51 only).
    /// AGC label: CAL53A → IMUCOARS → IMUSTALL.
    WaitCoarseAlign,
    /// Waiting for first star sighting mark.
    /// AGC label: R51.2 / R51DSP (V01N70 flash); STARIND = 1.
    WaitStarA,
    /// Waiting for second star sighting mark.
    /// AGC label: R51 inner loop; STARIND = 0.
    WaitStarB,
    /// Computing gyro torque angles and issuing gyro pulses.
    /// AGC label: R55 → CALCGTA → PULSEM → IMUPULSE → IMUSTALL.
    Torque,
    /// Alignment complete. REFSMFLG set; REFSMMAT updated.
    Done,
    /// Alignment failed (gimbal lock, star not available, star data bad).
    /// AGC alarm codes: 405 (no stars), 401 (gimbal lock), 215 (no preferred).
    Failed,
}

/// State for the active P51 or P52 alignment program.
#[derive(Clone, Copy, Debug)]
pub struct P51State {
    /// Current phase.
    pub phase: AlignPhase,
    /// Which program variant (true = P51 initial, false = P52 inflight).
    pub is_p51: bool,
    /// Selected alignment orientation option.
    pub option: AlignOption,
    /// Line-of-sight unit vector for first star (in SM frame).
    /// Set when crew completes first mark. AGC erasable: STARSAV1.
    pub star_a: Option<Vec3>,
    /// Line-of-sight unit vector for second star (in SM frame).
    /// Set when crew completes second mark. AGC erasable: STARSAV2.
    pub star_b: Option<Vec3>,
    /// Computed gyro torque angles (Y, Z, X gyros) in radians.
    pub torque_angles: Option<[f64; 3]>,
    /// Alarm code if `phase == Failed`; 0 if no alarm.
    pub alarm_code: u16,
    /// Coarse align converged (used by P51 WaitCoarseAlign → WaitStarA transition).
    pub coarse_aligned: bool,
    /// Fine align complete (used by Torque → Done transition).
    pub fine_aligned: bool,
    /// Catalog direction for first star (in NB frame).
    pub star_a_desired: Option<Vec3>,
    /// Catalog direction for second star (in NB frame).
    pub star_b_desired: Option<Vec3>,
}

// ── Functions ─────────────────────────────────────────────────────────────────

/// Enter P51 (initial coarse + fine alignment).
///
/// Sets phase to `PromptRefsmmat`, sets `is_p51 = true`.
/// Calls IMU status check (R02BOTH). Clears UPDATFLG and TRACKFLG.
///
/// AGC source: Comanche055/P51-P53.agc PROG52 entry (page 738-739).
pub fn enter_p51<H: AgcHardware>(state: &mut AgcState, hw: &mut H) -> P51State {
    // Read IMU status (CHAN30).
    // AGC source: TC BANKCALL CADR R02BOTH (page 738).
    let imu_status = hw.imu().read_status();

    // Bit 13 (1-based) = IMU fail = bit 12 (0-based).
    // If IMU fail bit is set, return Failed immediately.
    // AGC alarm codes 01426/01427 for IMU unsatisfactory.
    let imu_fail = (imu_status >> 12) & 1 != 0;

    // Clear UPDATFLG and TRACKFLG.
    // AGC source: TC DOWNFLAG ADRES UPDATFLG; TC DOWNFLAG ADRES TRACKFLG.
    state.flags.updatflg = false;
    state.flags.trackflg = false;

    state.modreg = 51;
    hw.dsky().write_prog(51);

    if imu_fail {
        AlarmState::raise(AlarmCode::DeviceConflict); // closest to alarm 01426
        return P51State {
            phase: AlignPhase::Failed,
            is_p51: true,
            option: AlignOption::Nominal,
            star_a: None,
            star_b: None,
            torque_angles: None,
            alarm_code: 0o1426,
            coarse_aligned: false,
            fine_aligned: false,
            star_a_desired: None,
            star_b_desired: None,
        };
    }

    P51State {
        phase: AlignPhase::PromptRefsmmat,
        is_p51: true,
        option: AlignOption::Nominal,
        star_a: None,
        star_b: None,
        torque_angles: None,
        alarm_code: 0,
        coarse_aligned: false,
        fine_aligned: false,
        star_a_desired: None,
        star_b_desired: None,
    }
}

/// Enter P52 (in-flight realignment).
///
/// Sets phase to `PromptRefsmmat`, sets `is_p51 = false`.
/// Identical preamble to P51 (R02BOTH check, flag clears).
/// Skips coarse-align step.
///
/// AGC source: Comanche055/P51-P53.agc PROG52 entry — same label for both.
pub fn enter_p52<H: AgcHardware>(state: &mut AgcState, hw: &mut H) -> P51State {
    let imu_status = hw.imu().read_status();
    let imu_fail = (imu_status >> 12) & 1 != 0;

    state.flags.updatflg = false;
    state.flags.trackflg = false;

    state.modreg = 52;
    hw.dsky().write_prog(52);

    if imu_fail {
        AlarmState::raise(AlarmCode::DeviceConflict);
        return P51State {
            phase: AlignPhase::Failed,
            is_p51: false,
            option: AlignOption::Nominal,
            star_a: None,
            star_b: None,
            torque_angles: None,
            alarm_code: 0o1426,
            coarse_aligned: false,
            fine_aligned: false,
            star_a_desired: None,
            star_b_desired: None,
        };
    }

    P51State {
        phase: AlignPhase::PromptRefsmmat,
        is_p51: false,
        option: AlignOption::Nominal,
        star_a: None,
        star_b: None,
        torque_angles: None,
        alarm_code: 0,
        coarse_aligned: true, // P52 skips coarse align
        fine_aligned: false,
        star_a_desired: None,
        star_b_desired: None,
    }
}

/// Advance the alignment state machine by one executive cycle.
///
/// Phase transitions are driven by injected star sighting data and
/// coarse/fine align status.
///
/// AGC source: Comanche055/P51-P53.agc full flow from PROG52 through R51/R55.
pub fn tick<H: AgcHardware>(align_state: &mut P51State, state: &mut AgcState, hw: &mut H) {
    match align_state.phase {
        AlignPhase::Failed | AlignPhase::Done => {
            // Terminal states — no-op.
        }

        AlignPhase::PromptRefsmmat => {
            // Transition when crew selects an option (simulated: check DSKY proceed).
            // AGC source: P52B GOPERF4R flash; proceed on crew Enter.
            if hw.dsky().proceed_pressed() {
                if align_state.is_p51 {
                    // P51: go to WaitCoarseAlign first.
                    align_state.phase = AlignPhase::WaitCoarseAlign;
                } else {
                    // P52: skip coarse align, go directly to WaitStarA.
                    align_state.phase = AlignPhase::WaitStarA;
                }
            }
        }

        AlignPhase::WaitCoarseAlign => {
            // Transition to WaitStarA when coarse align completes.
            // AGC source: CAL53A → COARFINE → REFSMFLG set (page 762).
            if align_state.coarse_aligned {
                align_state.phase = AlignPhase::WaitStarA;
            }
        }

        AlignPhase::WaitStarA => {
            // Transition to WaitStarB when first star is sighted.
            if align_state.star_a.is_some() {
                align_state.phase = AlignPhase::WaitStarB;
            }
        }

        AlignPhase::WaitStarB => {
            // Validate star pair then transition to Torque.
            // AGC source: CHKSDATA (page 760) — check angular separation.
            if let (Some(sa), Some(sb)) = (align_state.star_a, align_state.star_b) {
                // Star pair validation: check that |dot(SA, SB)| ≤ cos(40°).
                // AGC source: CHKSDATA checks separation ≥ 40°.
                let separation_cos = dot(&sa, &sb).abs();
                if separation_cos > MIN_STAR_SEPARATION_COS {
                    // Stars too close (< 40° apart) — alarm 405.
                    AlarmState::raise(AlarmCode::DeviceConflict);
                    align_state.alarm_code = 0o405;
                    align_state.phase = AlignPhase::Failed;
                } else {
                    align_state.phase = AlignPhase::Torque;
                }
            }
        }

        AlignPhase::Torque => {
            // Execute fine alignment using the two-vector SMNB method.
            // AGC source: AXISGEN → CALCGTA → R55 → IMUPULSE.
            if let (Some(sa), Some(sb), Some(sa_d), Some(sb_d)) = (
                align_state.star_a,
                align_state.star_b,
                align_state.star_a_desired,
                align_state.star_b_desired,
            ) {
                // Use ImuController to perform fine alignment.
                // We construct a temporary ImuController from the raw ImuImpl.
                // The real implementation would have the ImuController passed in.
                // For the state machine, we perform the math directly and call
                // hw.imu().torque_gyro().
                let result = perform_fine_align(hw.imu(), sa, sb, sa_d, sb_d);
                match result {
                    Ok(torque_angles) => {
                        align_state.torque_angles = Some(torque_angles);
                        align_state.fine_aligned = true;
                        align_state.phase = AlignPhase::Done;
                        // Update REFSMMAT in state.
                        state.flags.refsmflg = true;
                    }
                    Err(FineAlignError::CollinearStars) => {
                        AlarmState::raise(AlarmCode::DeviceConflict);
                        align_state.alarm_code = 0o405;
                        align_state.phase = AlignPhase::Failed;
                    }
                    Err(FineAlignError::NumericalError) => {
                        AlarmState::raise(AlarmCode::DeviceConflict);
                        align_state.alarm_code = 0o0427;
                        align_state.phase = AlignPhase::Failed;
                    }
                }
            } else {
                // Missing star data — should not happen; fail safely.
                align_state.alarm_code = 0o405;
                align_state.phase = AlignPhase::Failed;
            }
        }
    }
}

/// Inject a star sighting for the first star (crew mark for WaitStarA).
///
/// Called by the operator/simulation layer when the crew marks star A.
/// AGC source: R52 SXTSM stored in STARSAV1.
pub fn mark_star_a(align_state: &mut P51State, los_sm: Vec3, catalog_nb: Vec3) {
    align_state.star_a = Some(los_sm);
    align_state.star_a_desired = Some(catalog_nb);
}

/// Inject a star sighting for the second star (crew mark for WaitStarB).
///
/// Called by the operator/simulation layer when the crew marks star B.
/// AGC source: R52 inner loop STARSAV2.
pub fn mark_star_b(align_state: &mut P51State, los_sm: Vec3, catalog_nb: Vec3) {
    align_state.star_b = Some(los_sm);
    align_state.star_b_desired = Some(catalog_nb);
}

/// Signal that coarse alignment is complete (for P51 WaitCoarseAlign state).
///
/// Called by the coarse-align driver loop when step_coarse_align() returns Ok(true).
/// AGC source: CAL53A completion → COARFINE branch (page 762).
pub fn notify_coarse_aligned(align_state: &mut P51State) {
    align_state.coarse_aligned = true;
}

// ── Internal: fine alignment math ────────────────────────────────────────────

/// Perform the AXISGEN + CALCGTA fine alignment computation.
///
/// Reproduces the two-star SMNB algorithm from INFLIGHT_ALIGNMENT_ROUTINES.agc.
/// Applies gyro torque via `hw.imu().torque_gyro()`.
///
/// AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc AXISGEN (page 1361),
///             CALCGTA (page 1355).
fn perform_fine_align(
    imu: &mut dyn crate::hal::imu::ImuIo,
    star_a_sm: Vec3,
    star_b_sm: Vec3,
    star_a_desired: Vec3,
    star_b_desired: Vec3,
) -> Result<[f64; 3], FineAlignError> {
    use crate::math::linalg::{cross, norm, unit};

    let sa = star_a_sm;
    let sb = star_b_sm;
    let sa_cross_sb = cross(&sa, &sb);
    if norm(&sa_cross_sb) < 1e-6 {
        return Err(FineAlignError::CollinearStars);
    }
    let va = unit(&sa_cross_sb).ok_or(FineAlignError::CollinearStars)?;
    let wa = cross(&sa, &va);

    let sa_d = star_a_desired;
    let sb_d = star_b_desired;
    let sa_d_cross_sb_d = cross(&sa_d, &sb_d);
    let vb = unit(&sa_d_cross_sb_d).ok_or(FineAlignError::CollinearStars)?;
    let wb = cross(&sa_d, &vb);

    // XDC rotation matrix (SM→NB).
    let mut xdc = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            xdc[i][j] = sa_d[i] * sa[j] + vb[i] * va[j] + wb[i] * wa[j];
        }
    }

    // CALCGTA: compute gyro torque angles.
    let xd1 = xdc[0][0];
    let xd2 = xdc[1][0];
    let xd3 = xdc[2][0];
    let zprime_raw = [-xd3, 0.0, xd1];
    let zprime = unit(&zprime_raw).unwrap_or([1.0, 0.0, 0.0]);
    let igc = libm::atan2(zprime[0], zprime[2]);
    let cos_igc = libm::cos(igc);
    let mgc = libm::atan2(xd2, cos_igc);
    let ydc = [xdc[0][1], xdc[1][1], xdc[2][1]];
    let zdc = [xdc[0][2], xdc[1][2], xdc[2][2]];
    let zp_dot_ydc = dot(&zprime, &ydc);
    let zp_dot_zdc = dot(&zprime, &zdc);
    let ogc = libm::atan2(zp_dot_ydc, zp_dot_zdc);

    if !igc.is_finite() || !mgc.is_finite() || !ogc.is_finite() {
        return Err(FineAlignError::NumericalError);
    }

    // Apply gyro torque.
    let angle_to_pulses = |angle_rad: f64| -> i16 {
        let p = libm::round(angle_rad / core::f64::consts::TAU * 32768.0);
        p.clamp(i16::MIN as f64, i16::MAX as f64) as i16
    };
    imu.torque_gyro(0, angle_to_pulses(ogc));
    imu.torque_gyro(1, angle_to_pulses(igc));
    imu.torque_gyro(2, angle_to_pulses(mgc));

    Ok([igc, mgc, ogc])
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::mock_hw::MockHardware;

    /// T1: P51 entry — IMU status OK → PromptRefsmmat.
    ///
    /// AGC source: Comanche055/P51-P53.agc PROG52 entry (page 738-739).
    #[test]
    fn p51_entry_imu_ok_prompts_refsmmat() {
        let mut state = AgcState::new();
        let mut hw = MockHardware::new();
        let result = enter_p51(&mut state, &mut hw);
        assert_eq!(result.phase, AlignPhase::PromptRefsmmat);
        assert!(result.is_p51);
        assert!(!state.flags.updatflg);
        assert!(!state.flags.trackflg);
    }

    /// T2: P51 entry — IMU fail bit set → Failed immediately.
    ///
    /// AGC source: Comanche055/P51-P53.agc S61.1 → alarm 01426/01427.
    #[test]
    fn p51_entry_imu_fail_returns_failed() {
        use crate::services::alarm::AlarmState;
        AlarmState::clear_all();
        let mut state = AgcState::new();
        let mut hw = MockHardware::new();
        // Inject IMU fail status: bit 12 (0-based) = bit 13 (1-based).
        hw.imu.inject_status(1 << 12);
        let result = enter_p51(&mut state, &mut hw);
        assert_eq!(result.phase, AlignPhase::Failed);
        AlarmState::clear_all();
    }

    /// T3: Coarse-align completion transitions WaitCoarseAlign → WaitStarA.
    ///
    /// AGC source: CAL53A → COARFINE → REFSMFLG set (page 762).
    #[test]
    fn coarse_align_complete_transitions_to_wait_star_a() {
        let mut state = AgcState::new();
        let mut hw = MockHardware::new();
        let mut as_state = enter_p51(&mut state, &mut hw);
        as_state.phase = AlignPhase::WaitCoarseAlign;
        notify_coarse_aligned(&mut as_state);
        tick(&mut as_state, &mut state, &mut hw);
        assert_eq!(as_state.phase, AlignPhase::WaitStarA);
    }

    /// T4: First star sighting accepted — WaitStarA → WaitStarB.
    ///
    /// AGC source: R51 STARIND=1 → R52 mark → STARSAV1 stored.
    #[test]
    fn first_star_sighting_advances_phase() {
        let mut state = AgcState::new();
        let mut hw = MockHardware::new();
        let mut as_state = enter_p51(&mut state, &mut hw);
        as_state.phase = AlignPhase::WaitStarA;
        let los: Vec3 = [1.0, 0.0, 0.0];
        mark_star_a(&mut as_state, los, los);
        tick(&mut as_state, &mut state, &mut hw);
        assert_eq!(as_state.phase, AlignPhase::WaitStarB);
        assert_eq!(as_state.star_a, Some(los));
    }

    /// T5: Star pair too close (< 40°) → Failed with alarm 405.
    ///
    /// AGC source: Comanche055/P51-P53.agc CHKSDATA (page 760).
    #[test]
    fn star_pair_too_close_returns_failed() {
        use crate::services::alarm::AlarmState;
        AlarmState::clear_all();
        let mut state = AgcState::new();
        let mut hw = MockHardware::new();
        let mut as_state = enter_p51(&mut state, &mut hw);
        as_state.phase = AlignPhase::WaitStarB;
        // Two nearly identical stars (separation ≈ 0°).
        let star: Vec3 = [1.0, 0.0, 0.0];
        mark_star_a(&mut as_state, star, star);
        mark_star_b(&mut as_state, star, star);
        tick(&mut as_state, &mut state, &mut hw);
        assert_eq!(as_state.phase, AlignPhase::Failed);
        AlarmState::clear_all();
    }

    /// T6: P52 full fine-align — enters correctly and skips coarse-align.
    ///
    /// AGC source: Comanche055/P51-P53.agc PROG52 — P52 skips CAL53A.
    #[test]
    fn p52_enter_skips_coarse_align() {
        let mut state = AgcState::new();
        let mut hw = MockHardware::new();
        let result = enter_p52(&mut state, &mut hw);
        assert_eq!(result.phase, AlignPhase::PromptRefsmmat);
        assert!(!result.is_p51);
        // P52 has coarse_aligned = true (skips coarse step).
        assert!(result.coarse_aligned);
    }

    /// T7: P52 PromptRefsmmat → WaitStarA (skips WaitCoarseAlign).
    #[test]
    fn p52_proceeds_to_star_a_directly() {
        let mut state = AgcState::new();
        let mut hw = MockHardware::new();
        let mut as_state = enter_p52(&mut state, &mut hw);
        // Simulate crew pressing PROCEED.
        hw.dsky.set_proceed(true);
        tick(&mut as_state, &mut state, &mut hw);
        assert_eq!(as_state.phase, AlignPhase::WaitStarA);
    }

    /// T8: Full P52 alignment with orthogonal stars reaches Done.
    #[test]
    fn p52_full_fine_align_reaches_done() {
        let mut state = AgcState::new();
        let mut hw = MockHardware::new();
        let mut as_state = enter_p52(&mut state, &mut hw);

        hw.dsky.set_proceed(true);
        tick(&mut as_state, &mut state, &mut hw); // → WaitStarA

        let sa: Vec3 = [1.0, 0.0, 0.0];
        let sb: Vec3 = [0.0, 1.0, 0.0];
        mark_star_a(&mut as_state, sa, sa);
        tick(&mut as_state, &mut state, &mut hw); // → WaitStarB
        mark_star_b(&mut as_state, sb, sb);
        tick(&mut as_state, &mut state, &mut hw); // → Torque (star separation OK)
        tick(&mut as_state, &mut state, &mut hw); // → Done
        assert_eq!(as_state.phase, AlignPhase::Done);
        assert!(state.flags.refsmflg);
    }
}
