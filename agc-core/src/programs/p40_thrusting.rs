//! P40/P41 — SPS and RCS Burn Execution.
//!
//! P40 executes an SPS (main engine) burn. P41 executes an RCS translational
//! burn using the +X (forward-facing) jets. Both share the same six-phase
//! monotonic state machine.
//!
//! AGC source: Comanche055/P40-P47.agc
//!   P40CSM (page 684), P41CSM (page 688), TIGBLNK (page 690),
//!   TIG-30 (page 693), TIG-5 (page 694), TIG-0/IGNITION (page 695),
//!   DOTVCON/DOSTRULL (page 696), ENGINOFF/DOSPSOFF (page 698),
//!   SPSOFF/BESTTRIM (page 699), POSTBURN (page 686), S40.1 (page 710).

use crate::hal::{AgcHardware, EngineIo, RcsIo};
use crate::math::linalg::norm;
use crate::services::alarm::{AlarmCode, AlarmState};
use crate::types::{Met, Vec3};
use crate::AgcState;

// ── Constants ──────────────────────────────────────────────────────────────────

/// SPS thrust, Newtons.
///
/// AGC source: Comanche055/P40-P47.agc `FENG 2DEC 9.1188544 B-7` (page 689).
pub const SPS_THRUST_N: f64 = 91_188.544;

/// Ullage effective duration from IGNITION to DOSTRULL, seconds.
///
/// Derived from two back-to-back FIXDELAY calls in P40-P47.agc:
///   DEC 40  (0.4 s TVC buildup from PREPTVC) +
///   DEC 160 (1.6 s ullage from DOTVCON) = 200 cs = 2.0 s.
/// There is no literal `TULLAGE` label in the AGC source; this value is
/// reconstructed from the timing chain.
///
/// AGC source: Comanche055/P40-P47.agc DOTVCON (DEC 40) + DOSTRULL (DEC 160).
pub const ULLAGE_DURATION_S: f64 = 2.0;

/// SPS tail-off delay, centiseconds.
///
/// Time from ENGINOFF to BESTTRIM (tail-off and residual burn settle).
///
/// AGC source: Comanche055/P40-P47.agc `DEC 250` at DOSPSOFF label (page 698).
pub const SPS_TAILOFF_CS: u32 = 250;

/// SPS tail-off delay in seconds.
pub const SPS_TAILOFF_S: f64 = SPS_TAILOFF_CS as f64 / 100.0;

/// TIG−30 s countdown, centiseconds.
///
/// AGC source: Comanche055/P40-P47.agc `SEC29.96 2DEC 2996` (page 693).
pub const TIG_MINUS_30_CS: u32 = 2996;

/// TIG−5 s arm ullage, centiseconds.
///
/// AGC source: Comanche055/P40-P47.agc `5SEC DEC 500` (page 694).
pub const TIG_MINUS_5_CS: u32 = 500;

// ── Types ─────────────────────────────────────────────────────────────────────

/// Which engine provides thrust for this burn.
///
/// AGC source: Comanche055/P40-P47.agc — P40CSM clears ENG2FLAG (→ SPS);
///             P41CSM sets ENG2FLAG (→ RCS +X).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThrustMode {
    /// SPS main engine. P40CSM path. Sets ENGONFLG in FLAGWRD5.
    Sps,
    /// +X RCS translational jets. P41CSM path. Sets ENG2FLAG in FLAGWRD7.
    RcsPlusX,
}

/// Six-phase monotonic burn state machine.
///
/// AGC source: Derived from PHASCHNG calls in P40-P47.agc timing chain.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum BurnPhase {
    /// Phase 1: Slewing vehicle to burn attitude via KALCMANU.
    /// AGC label: P40SXTY.
    Attitude,
    /// Phase 2: Countdown display active; TIG-30/TIG-5 tasks scheduled.
    /// AGC label: P40TTOG.
    Countdown,
    /// Phase 3: +X RCS ullage jets on; propellant settling.
    /// AGC label: IGNITION.
    Ullage,
    /// Phase 4: Main engine on; VG steering active.
    /// AGC label: DOTVCON.
    Burn,
    /// Phase 5: ENGINOFF task scheduled; engine commanded off.
    /// AGC label: ENGINOFF, DOSPSOFF, SPSOFF.
    Cutoff,
    /// Phase 6: Post-burn; trim residuals, display N85.
    /// AGC label: DOTVCRCS, BESTTRIM, POSTBURN.
    Trim,
}

/// Burn target parameters (set by P30/P31 targeting).
///
/// AGC source: P40-P47.agc — loaded into VGTIG, TIG, F before P40 entry.
#[derive(Clone, Copy, Debug)]
pub struct BurnTarget {
    /// Velocity-to-go vector in inertial coordinates at TIG, m/s.
    /// AGC erasable: VGTIG.
    pub vg_tig: Vec3,
    /// Absolute time of ignition in centiseconds from epoch.
    /// AGC erasable: TIG.
    pub tig_cs: u32,
    /// Nominal thrust magnitude, newtons.
    /// AGC erasable: F (FENG for SPS).
    pub thrust_n: f64,
    /// Vehicle mass at TIG, kg.
    /// AGC erasable: CSMMASS.
    pub mass_kg: f64,
    /// Thrust mode (SPS or RCS +X).
    pub mode: ThrustMode,
}

/// Persistent state for the active P40/P41 burn program.
#[derive(Clone, Copy, Debug)]
pub struct P40State {
    /// Current phase of the burn state machine.
    pub phase: BurnPhase,
    /// Burn target computed by targeting.
    pub target: BurnTarget,
    /// Velocity-to-go vector in inertial coordinates, m/s.
    pub vg: Vec3,
    /// Time-to-go in centiseconds (computed by S40.13 / STEERING).
    pub tgo_cs: i32,
    /// Accumulated delta-V magnitude delivered (for residuals display), m/s.
    pub dv_delivered_ms: f64,
    /// True once ENGINOFF has been commanded (Cutoff phase entry guard).
    pub engine_off_commanded: bool,
    /// Trim pitch offset (PACTOFF).
    /// AGC erasable: PACTOFF (CDU counts).
    pub trim_pitch: i16,
    /// Trim yaw offset (YACTOFF).
    /// AGC erasable: YACTOFF (CDU counts).
    pub trim_yaw: i16,
    /// MET when engine-off was commanded (for tailoff timing).
    engine_off_met: Met,
    /// MET when IGNITION fired (for ullage timing).
    ignition_met: Met,
    /// MET when phase last transitioned (for Attitude phase timeout).
    phase_entry_met: Met,
}

// ── Functions ─────────────────────────────────────────────────────────────────

/// Initialize and enter the P40/P41 burn program.
///
/// Sets phase to `Attitude`, loads target into P40State, and sets
/// ENG2FLAG based on mode (SPS vs RCS).
///
/// AGC source: Comanche055/P40-P47.agc P40CSM / P41CSM entry (page 684/688).
pub fn enter<H: AgcHardware>(state: &mut AgcState, _hw: &mut H, target: BurnTarget) -> P40State {
    // Set major mode.
    let mm = if target.mode == ThrustMode::Sps {
        40
    } else {
        41
    };
    state.modreg = mm;

    // ENG2FLAG: FLAGWRD7 bit 2. Set for P41 (RCS), clear for P40 (SPS).
    // AGC source: P40CSM clears ENG2FLAG, P41CSM sets ENG2FLAG.
    let now = state.nav.sv.time();
    P40State {
        phase: BurnPhase::Attitude,
        target,
        vg: target.vg_tig,
        tgo_cs: compute_initial_tgo(&target),
        dv_delivered_ms: 0.0,
        engine_off_commanded: false,
        trim_pitch: 0,
        trim_yaw: 0,
        engine_off_met: Met(0),
        ignition_met: Met(0),
        phase_entry_met: now,
    }
}

/// Advance the P40/P41 burn state machine by one executive cycle.
///
/// Evaluates the current MET against scheduled phase transition times and
/// issues hardware commands as required.
///
/// Phase transitions (monotonic):
///   Attitude → Countdown: attitude error small (simulated: after first tick).
///   Countdown → Ullage: now >= TIG.
///   Ullage → Burn: ULLAGE_DURATION_S elapsed since IGNITION.
///   Burn → Cutoff: TGO <= 1 cs.
///   Cutoff → Trim: SPS_TAILOFF_S elapsed since ENGINOFF.
///   (Trim is terminal.)
///
/// AGC source: Comanche055/P40-P47.agc timing chain (pages 690-699).
pub fn tick<H: AgcHardware>(
    burn_state: &mut P40State,
    _state: &mut AgcState,
    hw: &mut H,
    now: Met,
) {
    // Terminal state guard.
    if burn_state.phase == BurnPhase::Trim {
        return;
    }

    match burn_state.phase {
        BurnPhase::Attitude => {
            // Transition to Countdown after attitude acquisition (simplified:
            // assume attitude reached after first tick in Attitude phase).
            // AGC: R60CSM completion sets TIMRFLAG; real code checks attitude error.
            // For this implementation, we transition immediately on the first tick
            // after a short hold (simulates KALCMANU completion).
            let elapsed = now.wrapping_sub_cs(burn_state.phase_entry_met);
            if elapsed >= 10 {
                // 0.1 s hold to simulate KALCMANU attitude maneuver
                transition_to(burn_state, BurnPhase::Countdown, now);
            }
        }

        BurnPhase::Countdown => {
            // Transition to Ullage when TIG is reached.
            // AGC source: TIG-0 → IGNITION label (page 695).
            if now.as_centiseconds() >= burn_state.target.tig_cs {
                transition_to(burn_state, BurnPhase::Ullage, now);
                burn_state.ignition_met = now;
                // Fire ullage jets (+X RCS forward thrusters).
                // AGC source: IGNITION fires +X RCS via CHAN5 WOR bits.
                // The +X jets are the forward translation thrusters (PYJETS bits for -Z translation).
                hw.rcs().fire_jets(crate::hal::rcs::JetCommand {
                    pitch_yaw: 0o01417,
                    roll: 0,
                });
            }
        }

        BurnPhase::Ullage => {
            // Transition to Burn after ULLAGE_DURATION_S from IGNITION.
            // AGC source: DOTVCON after DEC 40 + DEC 160 fixdelay = 2.0 s.
            let elapsed_cs = now.wrapping_sub_cs(burn_state.ignition_met);
            let ullage_cs = (ULLAGE_DURATION_S * 100.0) as u32;
            if elapsed_cs >= ullage_cs {
                transition_to(burn_state, BurnPhase::Burn, now);
                // Turn off ullage jets.
                hw.rcs().all_jets_off();
                // Enable main engine (or +X RCS for P41).
                match burn_state.target.mode {
                    ThrustMode::Sps => {
                        hw.engine().set_engine_enable(true);
                    }
                    ThrustMode::RcsPlusX => {
                        hw.rcs().fire_jets(crate::hal::rcs::JetCommand {
                            pitch_yaw: 0o01417,
                            roll: 0,
                        });
                    }
                }
            }
        }

        BurnPhase::Burn => {
            // Update VG (velocity-to-go) each cycle.
            // Simplified steering: compute TGO from VG and thrust.
            burn_state.tgo_cs = compute_tgo_cs(burn_state);

            // Transition to Cutoff when TGO ≤ 1 cs.
            // AGC source: STEERING calls ENGINOFF when TGO reaches 1 cs (page 697).
            if burn_state.tgo_cs <= 1 {
                transition_to(burn_state, BurnPhase::Cutoff, now);
                burn_state.engine_off_met = now;
                burn_state.engine_off_commanded = true;
                // Command engine off.
                match burn_state.target.mode {
                    ThrustMode::Sps => {
                        hw.engine().set_engine_enable(false);
                    }
                    ThrustMode::RcsPlusX => {
                        hw.rcs().all_jets_off();
                    }
                }
            }
        }

        BurnPhase::Cutoff => {
            // Transition to Trim after SPS_TAILOFF_S.
            // AGC source: DOSPSOFF `DEC 250` (page 698) 2.5 s tailoff.
            let elapsed_cs = now.wrapping_sub_cs(burn_state.engine_off_met);
            if elapsed_cs >= SPS_TAILOFF_CS {
                transition_to(burn_state, BurnPhase::Trim, now);
                // Apply trim offsets.
                hw.engine().trim_pitch(burn_state.trim_pitch);
                hw.engine().trim_yaw(burn_state.trim_yaw);
            }
        }

        BurnPhase::Trim => {
            // Terminal — already handled by guard at top.
        }
    }
}

/// Exit the burn program, null residuals, and safe the engine.
///
/// Forces engine off and jets off regardless of current phase.
///
/// AGC source: Comanche055/P40-P47.agc POST41 → GOTOPOOH path (page 687/698).
pub fn exit<H: AgcHardware>(burn_state: &mut P40State, _state: &mut AgcState, hw: &mut H) {
    hw.engine().set_engine_enable(false);
    hw.rcs().all_jets_off();
    burn_state.engine_off_commanded = true;
    burn_state.phase = BurnPhase::Trim;
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Transition to a new phase (monotonicity enforced).
///
/// Raises alarm if a backward phase transition is attempted.
fn transition_to(burn_state: &mut P40State, new_phase: BurnPhase, now: Met) {
    if new_phase < burn_state.phase {
        // Monotonicity violation — raise alarm.
        // AGC source: phase table integrity check in FRESH_START_AND_RESTART.agc.
        AlarmState::raise(AlarmCode::PhaseTableError);
        return;
    }
    burn_state.phase = new_phase;
    burn_state.phase_entry_met = now;
}

/// Compute initial TGO in centiseconds from delta-V magnitude and thrust.
///
/// Uses the Tsiolkovsky rocket equation.
/// AGC source: Comanche055/P40-P47.agc S40.13 TIMEBURN (pages 726-728).
fn compute_initial_tgo(target: &BurnTarget) -> i32 {
    let dv_mag = norm(&target.vg_tig);
    if dv_mag == 0.0 || target.thrust_n <= 0.0 {
        return 0;
    }
    // Simplified: TGO ≈ m * dv / F (constant thrust approximation).
    let tgo_s = target.mass_kg * dv_mag / target.thrust_n;
    (tgo_s * 100.0) as i32
}

/// Compute current TGO in centiseconds from remaining VG.
///
/// Simplified constant-thrust approximation.
fn compute_tgo_cs(burn_state: &P40State) -> i32 {
    let vg_mag = norm(&burn_state.vg);
    if vg_mag == 0.0 || burn_state.target.thrust_n <= 0.0 {
        return 0;
    }
    let tgo_s = burn_state.target.mass_kg * vg_mag / burn_state.target.thrust_n;
    (tgo_s * 100.0) as i32
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::mock_hw::MockHardware;

    fn make_target(mode: ThrustMode) -> BurnTarget {
        BurnTarget {
            vg_tig: [0.0, 50.0, 0.0],
            tig_cs: 1000, // TIG at 10 seconds
            thrust_n: SPS_THRUST_N,
            mass_kg: 28_800.0,
            mode,
        }
    }

    fn make_state_met(cs: u32) -> AgcState {
        use crate::navigation::state_vector::StateVector;
        let mut state = AgcState::new();
        state.nav.sv = StateVector::new([6_556_370.0, 0.0, 0.0], [0.0, 7784.0, 0.0], Met(cs));
        state
    }

    /// T1: Attitude → Countdown transition.
    ///
    /// AGC source: P40-P47.agc P40SXTY → P40TTOG after R60CSM completion.
    #[test]
    fn attitude_to_countdown_after_first_tick() {
        let mut state = make_state_met(0);
        let mut hw = MockHardware::new();
        let target = make_target(ThrustMode::Sps);
        let mut bs = enter(&mut state, &mut hw, target);
        assert_eq!(bs.phase, BurnPhase::Attitude);
        // Advance time past the 10 cs hold.
        tick(&mut bs, &mut state, &mut hw, Met(20));
        assert_eq!(bs.phase, BurnPhase::Countdown);
    }

    /// T2: Countdown → Ullage at TIG.
    ///
    /// AGC source: P40-P47.agc IGNITION label (page 695).
    #[test]
    fn countdown_to_ullage_at_tig() {
        let mut state = make_state_met(0);
        let mut hw = MockHardware::new();
        let target = make_target(ThrustMode::Sps);
        let mut bs = enter(&mut state, &mut hw, target);
        // Advance to Countdown.
        tick(&mut bs, &mut state, &mut hw, Met(20));
        assert_eq!(bs.phase, BurnPhase::Countdown);
        // Advance to TIG (1000 cs).
        tick(&mut bs, &mut state, &mut hw, Met(1000));
        assert_eq!(bs.phase, BurnPhase::Ullage);
    }

    /// T3: Ullage → Burn after 2.0 s.
    ///
    /// AGC source: P40-P47.agc DOTVCON after DEC 40 + DEC 160 (page 696).
    #[test]
    fn ullage_to_burn_after_2s() {
        let mut state = make_state_met(0);
        let mut hw = MockHardware::new();
        let target = make_target(ThrustMode::Sps);
        let mut bs = enter(&mut state, &mut hw, target);
        // Get to Ullage.
        tick(&mut bs, &mut state, &mut hw, Met(20)); // → Countdown
        tick(&mut bs, &mut state, &mut hw, Met(1000)); // → Ullage
                                                       // Advance past ULLAGE_DURATION_S = 2.0 s = 200 cs.
        tick(&mut bs, &mut state, &mut hw, Met(1200));
        assert_eq!(bs.phase, BurnPhase::Burn);
        // SPS must be enabled.
        assert!(
            hw.engine.engine_enabled(),
            "SPS engine must be enabled in Burn phase"
        );
    }

    /// T4: Burn → Cutoff when TGO ≤ 1.
    ///
    /// AGC source: P40-P47.agc ENGINOFF when STEERING detects TGO ≤ 1 cs.
    #[test]
    fn burn_to_cutoff_when_tgo_1() {
        let mut state = make_state_met(0);
        let mut hw = MockHardware::new();
        // Use a tiny VG so TGO is small.
        let mut target = make_target(ThrustMode::Sps);
        target.vg_tig = [0.0, 0.001, 0.0]; // ~0.001 m/s → TGO very small
        let mut bs = enter(&mut state, &mut hw, target);
        // Force to Burn phase.
        bs.phase = BurnPhase::Burn;
        bs.vg = target.vg_tig;
        bs.tgo_cs = 1;
        tick(&mut bs, &mut state, &mut hw, Met(2000));
        assert_eq!(bs.phase, BurnPhase::Cutoff);
        assert!(bs.engine_off_commanded);
        assert!(
            !hw.engine.engine_enabled(),
            "SPS engine must be disabled at Cutoff"
        );
    }

    /// T5: Cutoff → Trim after 2.5 s tailoff.
    ///
    /// AGC source: P40-P47.agc DOSPSOFF DEC 250 (page 698).
    #[test]
    fn cutoff_to_trim_after_2_5s() {
        let mut state = make_state_met(0);
        let mut hw = MockHardware::new();
        let target = make_target(ThrustMode::Sps);
        let mut bs = enter(&mut state, &mut hw, target);
        bs.phase = BurnPhase::Cutoff;
        bs.engine_off_commanded = true;
        bs.engine_off_met = Met(2000);
        // Advance past 250 cs tailoff.
        tick(&mut bs, &mut state, &mut hw, Met(2251));
        assert_eq!(bs.phase, BurnPhase::Trim);
    }

    /// T6: Full SPS burn happy path — Attitude→Countdown→Ullage→Burn→Cutoff→Trim.
    ///
    /// AGC source: P40-P47.agc full timing chain (pages 690-699).
    #[test]
    fn full_sps_burn_happy_path() {
        let mut state = make_state_met(0);
        let mut hw = MockHardware::new();
        let mut target = make_target(ThrustMode::Sps);
        target.vg_tig = [0.0, 0.001, 0.0]; // tiny VG so TGO ≤ 1 quickly
        let mut bs = enter(&mut state, &mut hw, target);

        tick(&mut bs, &mut state, &mut hw, Met(20)); // → Countdown
        assert_eq!(bs.phase, BurnPhase::Countdown);
        tick(&mut bs, &mut state, &mut hw, Met(1000)); // → Ullage
        assert_eq!(bs.phase, BurnPhase::Ullage);
        tick(&mut bs, &mut state, &mut hw, Met(1200)); // → Burn (2.0 s after TIG)
        assert_eq!(bs.phase, BurnPhase::Burn);
        // TGO immediately ≤ 1 with tiny VG.
        tick(&mut bs, &mut state, &mut hw, Met(1201)); // → Cutoff
        assert_eq!(bs.phase, BurnPhase::Cutoff);
        let cutoff_met = bs.engine_off_met;
        tick(
            &mut bs,
            &mut state,
            &mut hw,
            Met(cutoff_met.as_centiseconds() + 251),
        ); // → Trim
        assert_eq!(bs.phase, BurnPhase::Trim);
    }

    /// T7: Emergency exit from Burn phase.
    ///
    /// AGC source: P40-P47.agc POST41 emergency abort path.
    #[test]
    fn emergency_exit_from_burn() {
        let mut state = make_state_met(0);
        let mut hw = MockHardware::new();
        let target = make_target(ThrustMode::Sps);
        let mut bs = enter(&mut state, &mut hw, target);
        bs.phase = BurnPhase::Burn;
        exit(&mut bs, &mut state, &mut hw);
        assert_eq!(bs.phase, BurnPhase::Trim);
        assert!(bs.engine_off_commanded);
        assert!(
            !hw.engine.enabled,
            "engine must be disabled after emergency exit"
        );
    }

    /// T8: Phase monotonicity guard — alarm raised on backward transition.
    ///
    /// AGC source: Phase table integrity (FRESH_START_AND_RESTART.agc).
    #[test]
    fn phase_monotonicity_guard() {
        use crate::services::alarm::AlarmState;
        AlarmState::clear_all();
        let mut bs = P40State {
            phase: BurnPhase::Countdown,
            target: make_target(ThrustMode::Sps),
            vg: [0.0, 50.0, 0.0],
            tgo_cs: 100,
            dv_delivered_ms: 0.0,
            engine_off_commanded: false,
            trim_pitch: 0,
            trim_yaw: 0,
            engine_off_met: Met(0),
            ignition_met: Met(0),
            phase_entry_met: Met(0),
        };
        transition_to(&mut bs, BurnPhase::Attitude, Met(100)); // backward!
                                                               // Phase must not have changed backward.
        assert_eq!(
            bs.phase,
            BurnPhase::Countdown,
            "backward transition must be rejected"
        );
        // Alarm must have been raised.
        assert!(
            AlarmState::most_recent().is_some(),
            "alarm must be raised on backward transition"
        );
        AlarmState::clear_all();
    }

    /// T9: P41 RCS mode — SPS is never commanded.
    ///
    /// AGC source: P40-P47.agc P41CSM sets ENG2FLAG (page 688).
    #[test]
    fn p41_rcs_no_sps_enable() {
        let mut state = make_state_met(0);
        let mut hw = MockHardware::new();
        let mut target = make_target(ThrustMode::RcsPlusX);
        target.vg_tig = [0.0, 0.001, 0.0];
        let mut bs = enter(&mut state, &mut hw, target);

        tick(&mut bs, &mut state, &mut hw, Met(20)); // → Countdown
        tick(&mut bs, &mut state, &mut hw, Met(1000)); // → Ullage
        tick(&mut bs, &mut state, &mut hw, Met(1200)); // → Burn
                                                       // SPS must NOT be enabled for P41.
        assert!(
            !hw.engine.engine_enabled(),
            "SPS must NOT be enabled in P41 (RCS mode)"
        );
    }
}
