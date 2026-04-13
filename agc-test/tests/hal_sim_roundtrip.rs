// AGC behaviour: specs/hal.md
//
// Integration tests for the HAL trait contract verification.
// Each test exercises a specific HAL sub-trait through SimHardware, confirming
// that the sim correctly implements the AgcHardware super-trait interface.
//
// Unit tests for each SimHardware component exist in agc-sim/src/sim_hardware.rs.
// These tests focus on roundtrip correctness from the perspective of the flight
// software that calls the HAL — i.e., write via trait → read back the same value.

use agc_core::{
    hal::{AgcHardware, DskyIo, ImuIo, RcsIo},
    types::CduAngle,
};
use agc_sim::SimHardware;

// ── Test 1: DskyIo — key queue FIFO order ────────────────────────────────────

#[test]
fn dsky_key_queue_fifo_via_hal_trait() {
    // Spec: specs/hal.md §5 DskyIo
    // "read_key() returns the next keyboard keypress in FIFO order."
    // Cross-module: keys are pushed into SimDsky.display.key_queue (agc-sim),
    // and read back through the DskyIo trait (agc-core interface).

    use agc_core::hal::dsky::Key;

    let mut hw = SimHardware::new_headless();

    // Enqueue 3 keys through the test back-door (DskyDisplayState::enqueue_key).
    hw.dsky.display.enqueue_key(Key::Verb);
    hw.dsky.display.enqueue_key(Key::One);
    hw.dsky.display.enqueue_key(Key::Enter);

    // Read them back through the HAL DskyIo::read_key() in FIFO order.
    assert_eq!(
        hw.dsky().read_key(),
        Some(Key::Verb),
        "first key must be Verb"
    );
    assert_eq!(
        hw.dsky().read_key(),
        Some(Key::One),
        "second key must be One"
    );
    assert_eq!(
        hw.dsky().read_key(),
        Some(Key::Enter),
        "third key must be Enter"
    );

    // Queue now empty.
    assert_eq!(
        hw.dsky().read_key(),
        None,
        "queue must be empty after draining"
    );
}

// ── Test 2: DskyIo — write_relay is accepted without panic ───────────────────

#[test]
fn dsky_write_relay_no_panic() {
    // Spec: specs/hal.md §5 DskyIo "write_relay — relay word to channel OUT0"
    // write_relay is a no-op in Milestone 1 (full decoding deferred), but must
    // not panic on any input.

    use agc_core::hal::dsky::RelayWord;

    let mut hw = SimHardware::new_headless();

    // Write several relay words, including edge-case 0x0000 and 0x7FFF.
    hw.dsky().write_relay(RelayWord(0x0000));
    hw.dsky().write_relay(RelayWord(0x7FFF));
    hw.dsky().write_relay(RelayWord(0xFFFF));
    // No assertion needed — verifying it does not panic is the contract.
}

// ── Test 3: ImuIo — CDU angles roundtrip via test hook + trait read ───────────

#[test]
fn imu_cdu_angles_roundtrip_via_hal_trait() {
    // Spec: specs/hal.md §6 ImuIo — "CDU angle read through ImuIo::read_cdu()"
    // Cross-module: CDU values are injected via the SimImu public cdus field,
    // then read back through ImuIo::read_cdu().

    let mut hw = SimHardware::new_headless();

    // Expected angles: 0°, 90°, 180°, 270° as CduAngle counts.
    // CduAngle: 65536 counts = 360°, so:
    //   0°   = 0
    //   90°  = 16384
    //   180° = 32768
    //   270° = 49152
    let angles = [
        CduAngle(0),     // 0°
        CduAngle(16384), // 90°
        CduAngle(32768), // 180°
        CduAngle(49152), // 270°
    ];

    for &expected_angle in &angles {
        // Inject via test helper — set all 3 CDU axes to the same angle.
        hw.imu.inject_cdus([expected_angle; 3]);

        // Read back through the HAL trait.
        let readback = hw.imu().read_cdu();
        for (axis, &val) in readback.iter().enumerate() {
            assert_eq!(
                val, expected_angle,
                "CDU axis {} roundtrip failed for angle {:?}",
                axis, expected_angle
            );
        }
    }
}

// ── Test 4: ImuIo — CduAngle::to_radians for canonical angles ────────────────

#[test]
fn cdu_angle_to_radians_canonical_values() {
    // Spec: specs/hal.md §3 IMU Typestate Types — CduAngle scale factor
    // "65536 counts = 2π radians"
    // Cross-module: CduAngle (agc-core/src/types/angle.rs) is used through
    //               ImuIo::read_cdu() returned values.

    let tolerance = 1e-4; // radians

    // 0° → 0 rad
    let zero = CduAngle(0).to_radians();
    assert!(
        zero.abs() < tolerance,
        "0 counts must be ~0 rad, got {zero}"
    );

    // 90° → π/2 rad (16384 counts)
    let quarter = CduAngle(16384).to_radians();
    let expected_quarter = core::f64::consts::FRAC_PI_2;
    assert!(
        (quarter - expected_quarter).abs() < tolerance,
        "16384 counts must be ~π/2 rad, got {quarter}"
    );

    // 180° → π rad (32768 counts)
    let half = CduAngle(32768).to_radians();
    let expected_half = core::f64::consts::PI;
    assert!(
        (half - expected_half).abs() < tolerance,
        "32768 counts must be ~π rad, got {half}"
    );

    // 270° → 3π/2 rad (49152 counts)
    let three_quarter = CduAngle(49152).to_radians();
    let expected_three_quarter = 3.0 * core::f64::consts::FRAC_PI_2;
    assert!(
        (three_quarter - expected_three_quarter).abs() < tolerance,
        "49152 counts must be ~3π/2 rad, got {three_quarter}"
    );
}

// ── Test 5: ImuIo — read_pipa clears the PIPA accumulators ───────────────────

#[test]
fn imu_read_pipa_clears_on_read() {
    // Spec: specs/hal.md §6 ImuIo "read_and_clear_pipa: accumulate PIPA counts,
    //       read through the trait, verify the read clears"
    // Cross-module: PIPA counts injected via SimImu.pipas test back-door;
    //               cleared automatically by ImuIo::read_pipa().

    let mut hw = SimHardware::new_headless();

    // Inject PIPA counts via test helper.
    hw.imu.inject_pipas([42, -17, 100]);

    // First read via HAL trait must return the injected values.
    let counts = hw.imu().read_pipa();
    assert_eq!(
        counts,
        [42, -17, 100],
        "first read must return injected PIPA counts"
    );

    // Second read must return zeros (PIPA accumulators cleared on read).
    // Spec: specs/hal.md — ImuIo::read_pipa clears the accumulator
    let cleared = hw.imu().read_pipa();
    assert_eq!(cleared, [0, 0, 0], "PIPA must be cleared after first read");
}

// ── Test 6: RcsIo — jet-on commands recorded in sim log ──────────────────────

#[test]
fn rcs_jet_commands_recorded_via_hal_trait() {
    // Spec: specs/hal.md §9 RcsIo — jet command output
    // Cross-module: RcsIo::write_channel5/6 flows through SimRcs into
    //               SimHardware, readable back through current_command().

    let mut hw = SimHardware::new_headless();

    // Issue pitch/yaw jet command via the HAL trait.
    // Spec: specs/hal.md CHAN5 (PYJETS) = pitch + yaw jets
    hw.rcs().write_channel5(0b1010_1010_1010_1010_u16);

    assert_eq!(
        hw.rcs.current_command().pitch_yaw,
        0b1010_1010_1010_1010_u16,
        "write_channel5 must set pitch_yaw bits"
    );

    // Issue roll jet command.
    // Spec: specs/hal.md CHAN6 (ROLLJETS) = roll jets
    hw.rcs().write_channel6(0b0101_0101_0101_0101_u16);

    assert_eq!(
        hw.rcs.current_command().roll,
        0b0101_0101_0101_0101_u16,
        "write_channel6 must set roll bits"
    );

    // all_jets_off clears both channels.
    hw.rcs().all_jets_off();
    assert_eq!(hw.rcs.current_command().pitch_yaw, 0);
    assert_eq!(hw.rcs.current_command().roll, 0);
}

// ── Test 7: Timers — set_t3 and read_t3 roundtrip ────────────────────────────

#[test]
fn timers_t3_set_and_read_roundtrip() {
    // Spec: specs/hal.md §TIME3 "Waitlist timer (T3RUPT when overflows to 0)"
    // Cross-module: Timers::set_t3() via AgcHardware → read back via Timers::read_t3().

    use agc_core::hal::Timers;

    let mut hw = SimHardware::new_headless();

    // Set T3 to a specific value (simulating STARTSUB POSMAX init).
    let posmax: u16 = 0o37777;
    hw.timers().set_t3(posmax);
    assert_eq!(
        hw.timers().read_t3(),
        posmax,
        "T3 must read back the value set"
    );

    // Set to a small countdown value.
    hw.timers().set_t3(10);
    assert_eq!(hw.timers().read_t3(), 10, "T3 must read back 10");

    // Set to 0 (expired state).
    hw.timers().set_t3(0);
    assert_eq!(hw.timers().read_t3(), 0, "T3 must read back 0");
}
