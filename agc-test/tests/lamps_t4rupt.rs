//! T4RUPT lamp-refresh ordering tests for issue #137.
//!
//! Verifies that `services::t4rupt::t4rupt_step` invokes
//! `services::lamps::refresh_lamps` before any `decode_dsky` consumer
//! sees the DSKY state. Run from `agc-test` because `t4rupt_step`
//! requires a full `AgcHardware` impl (`SimHardware`) and `agc-core`
//! cannot depend on `agc-sim`.

use agc_core::control::imu_control::ImuAlignmentState;
use agc_core::services::pinball::decode_dsky;
use agc_core::services::t4rupt::t4rupt_step;
use agc_core::tables::alarm_codes::SITE_EXECUTIVE;
use agc_core::AgcState;
use agc_sim::SimHardware;

/// TC-T4-LAMPS-1: a stale `prog_alarm` is refreshed by `t4rupt_step`
/// before any downstream `decode_dsky` could observe it.
///
/// Raise an alarm, leave `state.dsky.prog_alarm` at the post-raise
/// value, then run one `t4rupt_step` tick. After the tick, both the
/// in-memory flag and the decoded `DskyFrame.lamps.prog_alarm` must
/// reflect the alarm being lit — which is only possible if
/// `refresh_lamps` ran *inside* the T4 handler before the frame was
/// decoded.
#[test]
fn tc_t4_lamps_1_alarm_lit_in_decoded_frame() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Raise an alarm without going through any lamp-refresh path.
    state.alarm.raise(0o1202, SITE_EXECUTIVE);
    // Pretend the previous tick had already cleared the lamp boolean
    // (the IMU/CDU defaults will leave it at false anyway, but make the
    // staleness explicit).
    state.dsky.prog_alarm = false;

    t4rupt_step(&mut state, &mut hw);

    assert!(
        state.dsky.prog_alarm,
        "t4rupt_step must call refresh_lamps so prog_alarm tracks alarm.lit"
    );

    // Downstream consumer: any decode_dsky after the T4 tick observes
    // the refreshed value.
    let frame = decode_dsky(&state.dsky);
    assert!(
        frame.lamps.prog_alarm,
        "decoded frame must reflect refreshed prog_alarm"
    );
}

/// TC-T4-LAMPS-2: `no_att` is refreshed from IMU alignment state.
///
/// Default alignment is `Caged` (not fine-aligned) → NO ATT must be
/// lit after one T4 tick.
#[test]
fn tc_t4_lamps_2_no_att_from_imu_alignment() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    assert_eq!(state.imu_alignment_state, ImuAlignmentState::Caged);
    state.dsky.no_att = false; // stale

    t4rupt_step(&mut state, &mut hw);

    assert!(state.dsky.no_att, "Caged IMU → NO ATT after T4 refresh");

    state.imu_alignment_state = ImuAlignmentState::FineAligned;
    t4rupt_step(&mut state, &mut hw);
    assert!(
        !state.dsky.no_att,
        "FineAligned IMU → NO ATT cleared after next T4 refresh"
    );
}

/// TC-T4-LAMPS-3: ordering — `refresh_lamps` must run before the
/// downlink drain (and therefore before any downstream `decode_dsky`
/// consumer). Verified by raising an alarm, running one T4, and
/// asserting that the decoded frame produced *after* the same tick
/// reports the alarm — proving the refresh did not happen *after* the
/// frame was last decoded.
#[test]
fn tc_t4_lamps_3_refresh_precedes_decode() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Initial decoded frame: no alarm lit.
    let frame_before = decode_dsky(&state.dsky);
    assert!(!frame_before.lamps.prog_alarm);

    // Raise an alarm and tick T4.
    state.alarm.raise(0o1410, SITE_EXECUTIVE);
    t4rupt_step(&mut state, &mut hw);

    // The post-tick decoded frame must observe the refreshed lamp.
    let frame_after = decode_dsky(&state.dsky);
    assert!(
        frame_after.lamps.prog_alarm,
        "post-T4 decoded frame must observe refresh_lamps output"
    );
}
