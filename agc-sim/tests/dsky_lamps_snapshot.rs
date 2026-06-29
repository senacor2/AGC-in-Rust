// SPDX-License-Identifier: GPL-3.0-or-later
//! M-D.4 — DSKY indicator-lamp snapshot tests (issue #140).
//!
//! Drives five scenarios and asserts on the **rendered** `DskyFrame`
//! (the output of `decode_dsky`, plus the `dsky_ui` footer for the alarm
//! case). Each scenario exercises one lamp wired by the M-D.1
//! `services::lamps::refresh_lamps` driver pass.
//!
//! The tests run every scenario through `t4rupt_step`, which is the path
//! that calls `refresh_lamps` before the frame is decoded. This is
//! deliberate: it makes the suite a **regression guard** for M-D.1 —
//! deleting the `refresh_lamps(state)` call from `t4rupt_step` leaves
//! every lamp boolean at its stale FRESH-START default, so the
//! `gimbal_lock` / `prog_alarm` / `tracker` / `key_rel` assertions below
//! all fail.
//!
//! Lives in `agc-sim` (not `agc-core`) because `t4rupt_step` needs a full
//! `AgcHardware` impl (`SimHardware`) and the alarm-footer assertion uses
//! `agc_sim::dsky_ui::render`.

use agc_core::navigation::state_vector::Frame;
use agc_core::programs::p22::p22_init;
use agc_core::services::lamps::LAMP_TEST_DURATION_TICKS;
use agc_core::services::pinball::{decode_dsky, DskyFrame};
use agc_core::services::t4rupt::t4rupt_step;
use agc_core::services::v_n::{feed_key, Key};
use agc_core::tables::alarm_codes::{SITE_EXECUTIVE, SITE_P22};
use agc_core::types::{CduAngle, Met};
use agc_core::AgcState;
use agc_sim::dsky_ui::render;
use agc_sim::SimHardware;

/// +90° middle-gimbal CDU count — the centre of the gimbal-lock band.
const PLUS_NINETY_DEG: i16 = 16384;

// ── Helpers ────────────────────────────────────────────────────────────────────

/// Feed a digit key.
fn d(n: u8) -> Key {
    Key::Digit(n)
}

/// Feed a sequence of keystrokes through the V/N processor.
fn feed(state: &mut AgcState, keys: &[Key]) {
    for &k in keys {
        feed_key(state, k);
    }
}

/// Run one T4RUPT tick (which calls `refresh_lamps`) and return the
/// freshly decoded DSKY frame.
fn frame_after_t4(state: &mut AgcState, hw: &mut SimHardware) -> DskyFrame {
    t4rupt_step(state, hw);
    decode_dsky(&state.dsky)
}

// ── 1. Gimbal march into critical → gimbal_lock ─────────────────────────────────

/// Marching the middle gimbal into the ±90° critical band lights
/// `gimbal_lock`; a safe attitude leaves it dark.
#[test]
fn gimbal_lock_lights_when_middle_gimbal_critical() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Safe attitude: middle gimbal at 0° → lamp dark.
    state.current_cdu = [CduAngle(0); 3];
    let frame = frame_after_t4(&mut state, &mut hw);
    assert!(
        !frame.lamps.gimbal_lock,
        "gimbal_lock must be dark at a safe middle-gimbal angle"
    );

    // March the middle gimbal (index 2) into the critical band at +90°.
    state.current_cdu[2] = CduAngle(PLUS_NINETY_DEG);
    let frame = frame_after_t4(&mut state, &mut hw);
    assert!(
        frame.lamps.gimbal_lock,
        "gimbal_lock must light once the middle gimbal reaches the ±90° band"
    );
}

// ── 2. Raise an 1107 alarm → prog_alarm + footer "01107" ────────────────────────

/// Raising alarm 1107 lights `prog_alarm`, and the `dsky_ui` footer
/// renders the code as a zero-padded `01107`.
#[test]
fn prog_alarm_lights_and_footer_shows_code() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    state.alarm.raise(1107, SITE_EXECUTIVE);
    let frame = frame_after_t4(&mut state, &mut hw);

    assert!(
        frame.lamps.prog_alarm,
        "prog_alarm must light while an alarm is lit"
    );

    // Render the frame and confirm the alarm-code footer shows 01107.
    let mut buf = Vec::new();
    render(
        &mut buf,
        (1, 1),
        &frame,
        None,
        state.time.0 as u64,
        "",
        true,
        state.alarm.code(),
    )
    .expect("render must succeed");
    let rendered = String::from_utf8_lossy(&buf);
    assert!(
        rendered.contains("ALM 01107"),
        "alarm-code footer must read 'ALM 01107'; rendered output was:\n{rendered}"
    );
}

// ── 3. V35 → all lamps light for ~5 s, then revert ──────────────────────────────

/// V35 lights every lamp (via `lamp_test`), and `refresh_lamps`
/// auto-reverts the test after the ~5 s window.
#[test]
fn v35_lamp_test_lights_all_then_reverts() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // V35 E — lamp test.
    feed(&mut state, &[Key::Verb, d(3), d(5), Key::Entr]);

    let frame = frame_after_t4(&mut state, &mut hw);
    assert!(
        frame.lamp_test,
        "V35 must engage the lamp test so dsky_ui lights every lamp"
    );

    // Run out the ~5 s lamp-test window (one tick already elapsed above).
    for _ in 0..LAMP_TEST_DURATION_TICKS {
        t4rupt_step(&mut state, &mut hw);
    }
    let frame = decode_dsky(&state.dsky);
    assert!(
        !frame.lamp_test,
        "lamp test must auto-revert after ~5 s (LAMP_TEST_DURATION_TICKS T4 passes)"
    );
}

// ── 4. Enter P22, open a sextant mark window → tracker ──────────────────────────

/// P22 entry opens a nav-mark tracking window, which lights `tracker`.
#[test]
fn tracker_lights_while_p22_tracking_active() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Before P22: no nav-mark loop active → tracker dark.
    let frame = frame_after_t4(&mut state, &mut hw);
    assert!(
        !frame.lamps.tracker,
        "tracker must be dark with no nav-mark loop active"
    );

    // P22 requires a navigation frame, a non-zero CSM epoch, and a real
    // position/velocity (init computes a sub-satellite point).
    state.csm_state.position = [7_000_000.0, 0.0, 0.0];
    state.csm_state.velocity = [0.0, 7546.0, 0.0];
    state.csm_state.frame = Frame::EarthInertial;
    state.csm_state.epoch = Met(360_000);
    p22_init(&mut state);
    assert_eq!(
        state.alarm.code(),
        0,
        "P22 init must not alarm with a valid CSM state vector (site {SITE_P22})"
    );
    assert!(
        state.csm_nav.tracking_active,
        "P22 init must open the tracking window"
    );

    let frame = frame_after_t4(&mut state, &mut hw);
    assert!(
        frame.lamps.tracker,
        "tracker must light while P22 tracking is active"
    );
}

// ── 5. V21 mid-entry → key_rel between digits ───────────────────────────────────

/// A V21 load that is mid data-entry (the V/N processor is holding the
/// DSKY waiting for digits) lights `key_rel`; completing the load clears
/// it.
#[test]
fn key_rel_lights_during_v21_data_entry() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // V21 N33 E — start a single-register load; the processor now waits
    // for data (VnPhase::EnteringData).
    feed(
        &mut state,
        &[Key::Verb, d(2), d(1), Key::Noun, d(3), d(3), Key::Entr],
    );

    let frame = frame_after_t4(&mut state, &mut hw);
    assert!(
        frame.lamps.key_rel,
        "key_rel must light while a V21 load is awaiting data"
    );

    // Complete the load: +00002 E → back to Idle → key_rel clears.
    feed(
        &mut state,
        &[Key::Plus, d(0), d(0), d(0), d(0), d(2), Key::Entr],
    );
    let frame = frame_after_t4(&mut state, &mut hw);
    assert!(
        !frame.lamps.key_rel,
        "key_rel must clear once the V21 load completes"
    );
}
