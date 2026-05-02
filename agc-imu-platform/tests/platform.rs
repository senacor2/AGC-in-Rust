use agc_imu_platform::{PlatformEmulator, UnitQuaternion, CDU_PULSE_RAD, GYRO_PULSE_RAD};

fn assert_near(a: f64, b: f64, tol: f64, label: &str) {
    assert!(
        (a - b).abs() < tol,
        "{label}: got {a:.10}, expected {b:.10}, diff {:.3e}",
        (a - b).abs()
    );
}

/// Zero gyro + zero accel: attitude stays at identity, PIPA stays at zero.
#[test]
fn stationary() {
    let mut p = PlatformEmulator::caged();
    p.uncage(UnitQuaternion::IDENTITY);
    for _ in 0..100 {
        p.tick([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.01);
    }
    let cdu = p.read_cdu();
    assert_eq!(cdu, [0u16, 0, 0], "stationary CDU");
    let pipa = p.read_pipa();
    assert_eq!(pipa.0, [0i16, 0, 0], "stationary PIPA");
}

/// Identity attitude + accel_z = 9.81 m/s for 1 second → pipa_z ≈ 167.
///
/// Expected: 9.81 / 0.0585 ≈ 167.69 → read_pipa returns 167.
#[test]
fn pure_z_accel() {
    let mut p = PlatformEmulator::caged();
    p.uncage(UnitQuaternion::IDENTITY);
    // 100 ticks of 10 ms each = 1 second total
    for _ in 0..100 {
        p.tick([0.0, 0.0, 0.0], [0.0, 0.0, 9.81], 0.01);
    }
    // Check accumulator before draining (fractional part)
    let expected_accum = 9.81 / 0.0585; // ≈ 167.69
    assert_near(p.pipa_accum[2], expected_accum, 1e-6, "pure_z_accel accum");
    let pipa = p.read_pipa();
    // Integer part: 167
    assert_eq!(pipa.0[2], 167i16, "pure_z_accel z count");
    assert_eq!(pipa.0[0], 0i16, "pure_z_accel x count");
    assert_eq!(pipa.0[1], 0i16, "pure_z_accel y count");
}

/// Two consecutive read_pipa on stationary emulator: second call returns [0,0,0].
#[test]
fn read_pipa_resets() {
    let mut p = PlatformEmulator::caged();
    p.uncage(UnitQuaternion::IDENTITY);
    // Accumulate something
    p.tick([0.0, 0.0, 0.0], [0.0, 0.0, 9.81], 1.0);
    let first = p.read_pipa();
    assert_ne!(first.0[2], 0i16, "first read should be non-zero");
    let second = p.read_pipa();
    // After drain the remaining fraction is < 1.0, so truncation gives 0
    assert_eq!(second.0, [0i16, 0, 0], "read_pipa_resets: second call");
}

/// 8192 gyro pulses on axis 0 (X) = 90°. CDU[0] should read ≈ 16384.
///
/// 8192 pulses × GYRO_PULSE_RAD = 8192 × TAU/32768 = TAU/4 = π/2.
/// π/2 in CDU counts: (π/2) / CDU_PULSE_RAD = (π/2) / (TAU/65536) = 16384.
#[test]
fn torque_gyro_x_quarter_rev() {
    let mut p = PlatformEmulator::caged();
    p.uncage(UnitQuaternion::IDENTITY);
    p.torque_gyro(0, 8192);
    let cdu = p.read_cdu();
    // Allow ±1 count of rounding
    let delta = (cdu[0] as i32 - 16384_i32).unsigned_abs();
    assert!(
        delta <= 1,
        "torque_gyro_x_quarter_rev: CDU[0]={}, expected ≈16384",
        cdu[0]
    );
}

/// coarse_align with a single-axis command → CDU readback matches within 1 count.
///
/// Single-axis rotation avoids ZYX Euler cross-coupling. Applied independently:
///   Axis 1 (pitch / inner): 2048 gyro pulses.
///   2048 × (GYRO_PULSE_RAD / CDU_PULSE_RAD) = 2048 × 2 = 4096 CDU counts.
///
/// Each axis is tested in isolation using fresh emulator instances so that the
/// ZYX Euler decomposition sees only a single non-zero angle.
#[test]
fn coarse_align_arbitrary() {
    // CDU counts = gyro_pulses * GYRO_PULSE_RAD / CDU_PULSE_RAD = gyro_pulses * 2
    let ratio = GYRO_PULSE_RAD / CDU_PULSE_RAD; // 65536 / 32768 = 2.0

    // Axis 0 (roll / outer) only
    let mut p0 = PlatformEmulator::caged();
    p0.uncage(UnitQuaternion::IDENTITY);
    p0.coarse_align([1024, 0, 0]);
    let cdu0 = p0.read_cdu();
    let expected0 = (1024.0 * ratio) as i32 as i16 as u16;
    let d0 = (cdu0[0] as i32 - expected0 as i32).unsigned_abs();
    assert!(
        d0 <= 1,
        "coarse_align axis-0: CDU[0]={}, expected≈{}",
        cdu0[0],
        expected0
    );

    // Axis 1 (pitch / inner) only
    let mut p1 = PlatformEmulator::caged();
    p1.uncage(UnitQuaternion::IDENTITY);
    p1.coarse_align([0, 2048, 0]);
    let cdu1 = p1.read_cdu();
    let expected1 = (2048.0 * ratio) as i32 as i16 as u16;
    let d1 = (cdu1[1] as i32 - expected1 as i32).unsigned_abs();
    assert!(
        d1 <= 1,
        "coarse_align axis-1: CDU[1]={}, expected≈{}",
        cdu1[1],
        expected1
    );

    // Axis 2 (yaw / middle): zero command → CDU unchanged
    let mut p2 = PlatformEmulator::caged();
    p2.uncage(UnitQuaternion::IDENTITY);
    p2.coarse_align([0, 0, 0]);
    let cdu2 = p2.read_cdu();
    assert_eq!(
        cdu2,
        [0u16, 0, 0],
        "coarse_align zero command leaves CDU at zero"
    );
}

/// With caged = true, ticks with arbitrary inputs leave attitude at identity and PIPA at zero.
#[test]
fn caged_no_integration() {
    let mut p = PlatformEmulator::caged();
    assert!(p.caged, "should be caged initially");
    for _ in 0..100 {
        p.tick([10.0, 20.0, 30.0], [1.0, 2.0, 9.81], 0.01);
    }
    // Attitude must remain identity
    assert_eq!(
        p.attitude,
        UnitQuaternion::IDENTITY,
        "caged: attitude must be identity"
    );
    // PIPA accumulator must remain zero
    assert_eq!(
        p.pipa_accum,
        [0.0f64, 0.0, 0.0],
        "caged: pipa_accum must be zero"
    );
    // CDU must read zeros
    let cdu = p.read_cdu();
    assert_eq!(cdu, [0u16, 0, 0], "caged: CDU must be zero");
}
