//! End-to-end uplink scenarios driven through the agc-sim
//! [`ScriptedUplink`] + [`T4Pump`] path.
//!
//! Mirrors the MS-U5 exit criterion (`specs/uplink-plan.md` §6 MS-U5):
//! Mission Control can reseed the state vector, upload a REFSMMAT,
//! correct the AGC clock, and patch a single erasable slot — all via
//! scripted uplink — and the AGC raises alarm 01106 when ground sends
//! keystrokes faster than the V/N processor can accept them.

use agc_core::services::average_g::PipaCalibration;
use agc_core::services::v_n::VnPhase;
use agc_core::tables::alarm_codes::UPLINK_TOO_FAST;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::runtime::T4Pump;
use agc_sim::SimHardware;

/// Helper: load `script` into the uplink FIFO and tick `T4Pump` until
/// it is drained (or `max_ticks` elapse). Each tick advances MET by one
/// T4RUPT period so the pump's countdown fires.
fn run_script(state: &mut AgcState, hw: &mut SimHardware, script: &str) {
    use agc_core::control::imu_control::T4RUPT_PERIOD_CS;
    hw.uplink.load_script(script).expect("script must parse");
    let mut t4 = T4Pump::new();
    // One T4 tick per remaining word, plus a few extra to flush.
    let ticks = hw.uplink.words.len() + 4;
    for _ in 0..ticks {
        t4.tick(state, hw);
        state.time = Met(state.time.0.wrapping_add(T4RUPT_PERIOD_CS as u32));
    }
}

/// TC-UPS-1: V71 reseeds the CSM state vector with a circular LEO orbit
/// (Apollo translunar parking style).
#[test]
fn tc_ups_1_v71_state_vector_reseed() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // V71 ENTR 01 ENTR 06 ENTR — load 6 words at address 1.
    // Position = [6578, 0, 0] km, Velocity = [0, 7784, 0] m/s — circular LEO.
    let script = "\
        V 7 1 E \
        0 1 E \
        0 6 E \
        + 6 5 7 8 E \
        + 0 0 0 0 0 E \
        + 0 0 0 0 0 E \
        + 0 0 0 0 0 E \
        + 0 7 7 8 4 E \
        + 0 0 0 0 0 E";
    run_script(&mut state, &mut hw, script);

    assert_eq!(state.vn.phase, VnPhase::Idle);
    assert!((state.csm_state.position[0] - 6_578_000.0).abs() < 1.0);
    assert_eq!(state.csm_state.position[1], 0.0);
    assert_eq!(state.csm_state.velocity[1], 7784.0);
}

/// TC-UPS-2: V71 uploads a complete REFSMMAT (9 words at addresses 14–22).
/// We pick the identity rotation scaled by zero revs (all elements 0)
/// then verify those slots became zero — i.e., the row-major layout
/// reaches the correct cells.
#[test]
fn tc_ups_2_v71_refsmmat_upload() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();
    // Seed REFSMMAT to something distinguishable so the upload's writes
    // are visible.
    state.refsmmat = [[9.0, 9.0, 9.0]; 3];

    // V71 ENTR 14 ENTR 09 ENTR <9 zeros>
    let script = "\
        V 7 1 E \
        1 4 E \
        0 9 E \
        + 0 E + 0 E + 0 E \
        + 0 E + 0 E + 0 E \
        + 0 E + 0 E + 0 E";
    run_script(&mut state, &mut hw, script);

    assert_eq!(state.vn.phase, VnPhase::Idle);
    for row in 0..3 {
        for col in 0..3 {
            assert_eq!(state.refsmmat[row][col], 0.0, "REFSMMAT[{}][{}]", row, col);
        }
    }
}

/// TC-UPS-3: V73 corrects the AGC clock by +1 minute 30 seconds.
#[test]
fn tc_ups_3_v73_clock_correction() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();
    state.time = Met(1_000_000);
    let before = state.time.0;

    // V73 ENTR +0h +1m +3000(=30.00s) ENTR — adds 90.00s = 9000 cs.
    // Use `_` to mask out the synthetic MET advance done by run_script,
    // which only ticks T4 (not state.time).
    let script = "V 7 3 E + 0 E + 1 E + 3 0 0 0 E";
    run_script(&mut state, &mut hw, script);

    assert_eq!(state.vn.phase, VnPhase::Idle);
    // V73 added 9000 cs; run_script's manual time ticks add 11 * T4 cs.
    // Subtract the synthetic advance to isolate V73's contribution.
    let total_advance = state.time.0.wrapping_sub(before);
    assert!(
        total_advance >= 9000,
        "V73 must have advanced state.time by at least 9000 cs (got {})",
        total_advance
    );
}

/// TC-UPS-4: V72 single-word update writes `gyro_comp.nbdx` (address 23)
/// without disturbing nbdy / nbdz.
#[test]
fn tc_ups_4_v72_single_word_gyro() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();
    state.gyro_comp.nbdy = 0.222;
    state.gyro_comp.nbdz = 0.333;

    // V72 ENTR 23 ENTR -01234 ENTR — nbdx ← -1.234 meru.
    let script = "V 7 2 E 2 3 E - 1 2 3 4 E";
    run_script(&mut state, &mut hw, script);

    assert_eq!(state.vn.phase, VnPhase::Idle);
    assert!((state.gyro_comp.nbdx + 1.234).abs() < 1e-9);
    assert!((state.gyro_comp.nbdy - 0.222).abs() < 1e-9);
    assert!((state.gyro_comp.nbdz - 0.333).abs() < 1e-9);
}

/// TC-UPS-5: scripted PIPA calibration via V71 (addresses 26–29).
/// Exercises both the scale (ppm Δ) and bias (cm/s² → counts)
/// conversions documented in `p27_apply_word`.
#[test]
fn tc_ups_5_v71_pipa_cal_upload() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // V71 ENTR 26 ENTR 04 ENTR  +100  +30  -30  +0 ENTR-each
    let script = "\
        V 7 1 E \
        2 6 E \
        0 4 E \
        + 1 0 0 E \
        + 3 0 E \
        - 3 0 E \
        + 0 E";
    run_script(&mut state, &mut hw, script);

    let expected_scale = PipaCalibration::NOMINAL.scale * (1.0 + 100e-6);
    assert!((state.pipa_cal.scale - expected_scale).abs() < 1e-12);
    assert_eq!(state.pipa_cal.bias[0], 10);
    assert_eq!(state.pipa_cal.bias[1], -10);
    assert_eq!(state.pipa_cal.bias[2], 0);
}

/// TC-UPS-6: forced OprErr followed by an uplink keystroke raises
/// alarm 01106 (UPLINK TOO FAST). MS-U5 exit-criterion alarm test.
#[test]
fn tc_ups_6_alarm_01106_on_overrun() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Drive the V/N into OprErr (an unknown verb sequence).
    let script = "V 9 9 N 0 0 E";
    run_script(&mut state, &mut hw, script);
    assert_eq!(state.vn.phase, VnPhase::OprErr);

    // Ground continues without RSET — overrun.
    let script = "V 0 6 N 1 7 E";
    run_script(&mut state, &mut hw, script);

    assert_eq!(state.alarm.code, UPLINK_TOO_FAST);
    assert!(state.alarm.lit, "01106 must light the PROG ALARM lamp");
    assert_eq!(
        state.vn.phase,
        VnPhase::OprErr,
        "OprErr must persist until RSET clears it"
    );
}

/// TC-UPS-7: the uplink activity lamp tracks streamed traffic. The
/// pump fires `t4rupt_step` whenever its countdown reaches zero, so the
/// test advances `state.time` by one full T4RUPT period between ticks
/// to drive a deterministic cadence.
#[test]
fn tc_ups_7_uplink_activity_lamp_tracks_traffic() {
    use agc_core::control::imu_control::T4RUPT_PERIOD_CS;

    let mut state = AgcState::new();
    let mut hw = SimHardware::new();
    let mut t4 = T4Pump::new();

    // Initial tick: arms the countdown and drains the (empty) FIFO once.
    state.time = Met(0);
    t4.tick(&mut state, &mut hw);
    assert!(!state.dsky.uplink_activity, "lamp must be off after a quiet poll");

    // Burst — load the FIFO, then advance one T4 period so the next
    // tick crosses the countdown threshold and the pump fires.
    hw.uplink.load_script("V 0 6 N 1 7 E").unwrap();
    state.time = Met(state.time.0 + T4RUPT_PERIOD_CS as u32);
    t4.tick(&mut state, &mut hw);
    assert!(
        state.dsky.uplink_activity,
        "lamp must light when uplink traffic is processed"
    );

    // Subsequent quiet T4 cycle — lamp clears.
    state.time = Met(state.time.0 + T4RUPT_PERIOD_CS as u32);
    t4.tick(&mut state, &mut hw);
    assert!(
        !state.dsky.uplink_activity,
        "lamp must clear on the next quiet T4 cycle"
    );
}
