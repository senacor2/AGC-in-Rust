// Integration tests for P40/P41 burn execution (six-phase state machine).
//
// Exercises the complete SPS and RCS burn sequence end-to-end using SimHardware.
// Tests cover: happy-path SPS burn, RCS (P41) variant, and mid-burn cutoff.
//
// AGC source: Comanche055/P40-P47.agc P40CSM/P41CSM timing chain (pages 684-699).

use agc_core::hal::{AgcHardware, EngineIo, RcsIo};
use agc_core::{
    navigation::state_vector::StateVector,
    programs::p40_thrusting::{
        enter, exit, tick, BurnPhase, BurnTarget, ThrustMode, SPS_TAILOFF_CS, ULLAGE_DURATION_S,
    },
    types::Met,
    AgcState,
};
use agc_sim::SimHardware;

fn make_target(mode: ThrustMode, tig_cs: u32) -> BurnTarget {
    BurnTarget {
        vg_tig: [0.0, 50.0, 0.0],
        tig_cs,
        thrust_n: agc_core::programs::p40_thrusting::SPS_THRUST_N,
        mass_kg: 28_800.0,
        mode,
    }
}

fn make_agc_state(cs: u32) -> AgcState {
    let r = 6_556_370.0_f64;
    let v = 7_784.0_f64;
    let sv = StateVector::new([r, 0.0, 0.0], [0.0, v, 0.0], Met(cs));
    let mut state = AgcState::new();
    state.nav.sv = sv;
    state
}

// ── TC-P40-IT-01: Full SPS burn — Attitude → Trim ────────────────────────────

/// Happy-path SPS burn sequences through all six phases without alarm.
///
/// AGC source: Comanche055/P40-P47.agc timing chain (pages 684-699).
#[test]
fn sps_full_burn_reaches_trim_phase() {
    let mut state = make_agc_state(0);
    let mut hw = SimHardware::new_headless();
    let tig_cs: u32 = 1000;
    let mut target = make_target(ThrustMode::Sps, tig_cs);
    // Use tiny VG so TGO → 0 quickly after ignition.
    target.vg_tig = [0.0, 0.001, 0.0];

    let mut bs = enter(&mut state, &mut hw, target);
    assert_eq!(bs.phase, BurnPhase::Attitude, "must start in Attitude");

    // 1. Attitude → Countdown (after 10 cs hold).
    tick(&mut bs, &mut state, &mut hw, Met(20));
    assert_eq!(bs.phase, BurnPhase::Countdown);
    assert_eq!(state.modreg, 40, "P40 modreg must be 40 in Countdown");

    // 2. Countdown → Ullage (at TIG).
    tick(&mut bs, &mut state, &mut hw, Met(tig_cs));
    assert_eq!(bs.phase, BurnPhase::Ullage);

    // 3. Ullage → Burn (after ULLAGE_DURATION_S = 2.0 s = 200 cs).
    let ullage_end = tig_cs + (ULLAGE_DURATION_S * 100.0) as u32 + 1;
    tick(&mut bs, &mut state, &mut hw, Met(ullage_end));
    assert_eq!(bs.phase, BurnPhase::Burn);
    assert!(
        hw.engine().engine_enabled(),
        "SPS must be enabled at Burn entry"
    );

    // 4. Burn → Cutoff (tiny VG → TGO ≤ 1 immediately).
    tick(&mut bs, &mut state, &mut hw, Met(ullage_end + 1));
    assert_eq!(bs.phase, BurnPhase::Cutoff);
    assert!(
        bs.engine_off_commanded,
        "engine-off must be commanded at Cutoff"
    );
    assert!(
        !hw.engine().engine_enabled(),
        "SPS must be disabled at Cutoff"
    );

    // 5. Cutoff → Trim (after 2.5 s = 250 cs tailoff).
    // Advance well past the tailoff window from the last known time.
    let post_cutoff = ullage_end + 2 + SPS_TAILOFF_CS + 10;
    tick(&mut bs, &mut state, &mut hw, Met(post_cutoff));
    assert_eq!(bs.phase, BurnPhase::Trim, "must reach Trim after tailoff");
}

// ── TC-P40-IT-02: P41 RCS mode — SPS never engaged ──────────────────────────

/// P41 (RCS +X) burn sequences through phases without ever enabling the SPS.
///
/// AGC source: Comanche055/P40-P47.agc P41CSM sets ENG2FLAG (page 688).
#[test]
fn p41_rcs_sps_never_enabled() {
    let mut state = make_agc_state(0);
    let mut hw = SimHardware::new_headless();
    let tig_cs: u32 = 500;
    let mut target = make_target(ThrustMode::RcsPlusX, tig_cs);
    target.vg_tig = [0.0, 0.001, 0.0];

    let mut bs = enter(&mut state, &mut hw, target);

    // Drive through all phases.
    tick(&mut bs, &mut state, &mut hw, Met(20)); // → Countdown
    tick(&mut bs, &mut state, &mut hw, Met(tig_cs)); // → Ullage
    let ullage_end = tig_cs + (ULLAGE_DURATION_S * 100.0) as u32 + 1;
    tick(&mut bs, &mut state, &mut hw, Met(ullage_end)); // → Burn

    // SPS must NEVER be enabled for P41.
    assert!(
        !hw.engine().engine_enabled(),
        "SPS must NOT be enabled for P41 (RCS-only mode)"
    );

    // RCS +X jets must have been commanded.
    assert_ne!(
        hw.rcs().current_command().pitch_yaw,
        0,
        "RCS +X jets must be commanded during P41 ullage/burn"
    );
}

// ── TC-P40-IT-03: Emergency abort from Burn ──────────────────────────────────

/// `exit()` during Burn forces engine cutoff and transitions to Trim.
///
/// AGC source: Comanche055/P40-P47.agc POST41 abort path.
#[test]
fn emergency_abort_from_burn_disables_engine() {
    let mut state = make_agc_state(0);
    let mut hw = SimHardware::new_headless();
    let target = make_target(ThrustMode::Sps, 500);

    let mut bs = enter(&mut state, &mut hw, target);
    // Manually place in Burn state with engine on.
    bs.phase = BurnPhase::Burn;
    bs.engine_off_commanded = false;
    hw.engine().set_engine_enable(true);

    exit(&mut bs, &mut state, &mut hw);

    assert_eq!(bs.phase, BurnPhase::Trim, "abort must reach Trim");
    assert!(
        bs.engine_off_commanded,
        "engine-off must be commanded on abort"
    );
    assert!(!hw.engine().engine_enabled(), "SPS must be off after abort");
}
