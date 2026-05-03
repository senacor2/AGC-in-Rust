use agc_imu_platform::UnitQuaternion;
use core::f64::consts::{FRAC_PI_2, PI, TAU};

const EPS12: f64 = 1e-12;
const EPS9: f64 = 1e-9;

fn assert_near(a: f64, b: f64, tol: f64, label: &str) {
    assert!(
        (a - b).abs() < tol,
        "{label}: got {a:.15}, expected {b:.15}, diff {:.3e}",
        (a - b).abs()
    );
}

fn assert_vec_near(a: [f64; 3], b: [f64; 3], tol: f64, label: &str) {
    for i in 0..3 {
        assert!(
            (a[i] - b[i]).abs() < tol,
            "{label}[{i}]: got {:.15}, expected {:.15}, diff {:.3e}",
            a[i],
            b[i],
            (a[i] - b[i]).abs()
        );
    }
}

fn quat_is_identity(q: UnitQuaternion, tol: f64, label: &str) {
    assert_near(q.w, 1.0, tol, &format!("{label}.w"));
    assert_near(q.x, 0.0, tol, &format!("{label}.x"));
    assert_near(q.y, 0.0, tol, &format!("{label}.y"));
    assert_near(q.z, 0.0, tol, &format!("{label}.z"));
}

fn some_unit_quats() -> [UnitQuaternion; 4] {
    [
        UnitQuaternion::from_axis_angle([1.0, 0.0, 0.0], 0.7),
        UnitQuaternion::from_axis_angle([0.0, 1.0, 0.0], 1.2),
        UnitQuaternion::from_axis_angle([0.0, 0.0, 1.0], -0.5),
        UnitQuaternion::from_euler_zyx(0.3, -0.4, 1.1),
    ]
}

/// IDENTITY · q == q and q · IDENTITY == q.
#[test]
fn identity_compose() {
    for q in some_unit_quats() {
        let left = UnitQuaternion::IDENTITY * (q);
        let right = q * (UnitQuaternion::IDENTITY);
        assert_near(left.w, q.w, EPS12, "identity_compose left.w");
        assert_near(left.x, q.x, EPS12, "identity_compose left.x");
        assert_near(left.y, q.y, EPS12, "identity_compose left.y");
        assert_near(left.z, q.z, EPS12, "identity_compose left.z");
        assert_near(right.w, q.w, EPS12, "identity_compose right.w");
        assert_near(right.x, q.x, EPS12, "identity_compose right.x");
        assert_near(right.y, q.y, EPS12, "identity_compose right.y");
        assert_near(right.z, q.z, EPS12, "identity_compose right.z");
    }
}

/// q · q⁻¹ ≈ IDENTITY for a variety of unit quaternions.
#[test]
fn inverse_round_trip() {
    for q in some_unit_quats() {
        let prod = q * (q.inverse());
        quat_is_identity(prod, EPS12, "inverse_round_trip");
    }
}

/// Rotating (0,1,0) by 90° about X → (0,0,1).
#[test]
fn from_axis_angle_x_90() {
    let q = UnitQuaternion::from_axis_angle([1.0, 0.0, 0.0], FRAC_PI_2);
    let v = q.rotate_vec([0.0, 1.0, 0.0]);
    assert_vec_near(v, [0.0, 0.0, 1.0], 1e-14, "from_axis_angle_x_90");
}

/// Rotating (1,0,0) by 90° about Y → (0,0,-1).
#[test]
fn from_axis_angle_y_90() {
    let q = UnitQuaternion::from_axis_angle([0.0, 1.0, 0.0], FRAC_PI_2);
    let v = q.rotate_vec([1.0, 0.0, 0.0]);
    assert_vec_near(v, [0.0, 0.0, -1.0], 1e-14, "from_axis_angle_y_90");
}

/// from_rotation_vector with small |rv| matches from_axis_angle to 1e-12.
#[test]
fn from_rotation_vector_small_angle() {
    let rv = [1e-5_f64, 0.0, 0.0];
    let theta = 1e-5_f64;
    let q_rv = UnitQuaternion::from_rotation_vector(rv);
    let q_aa = UnitQuaternion::from_axis_angle([1.0, 0.0, 0.0], theta);
    assert_near(q_rv.w, q_aa.w, EPS12, "small_angle.w");
    assert_near(q_rv.x, q_aa.x, EPS12, "small_angle.x");
    assert_near(q_rv.y, q_aa.y, EPS12, "small_angle.y");
    assert_near(q_rv.z, q_aa.z, EPS12, "small_angle.z");
}

/// 100 integration steps of ω=(1,0,0) rad/s with dt=π/100 rotates (0,0,1) to ≈(0,0,-1).
#[test]
fn integrate_constant_rate_about_x() {
    let omega = [1.0_f64, 0.0, 0.0];
    let dt = PI / 100.0;
    let mut q = UnitQuaternion::IDENTITY;
    for _ in 0..100 {
        q = q.integrate(omega, dt);
    }
    let v = q.rotate_vec([0.0, 0.0, 1.0]);
    // After π rotation about X: (0,0,1) → (0,0,-1)
    assert_near(v[0], 0.0, 1e-10, "integrate_x.x");
    assert_near(v[1], 0.0, 1e-10, "integrate_x.y");
    assert_near(v[2], -1.0, 1e-10, "integrate_x.z");
}

/// Euler triple → quaternion → Euler matches to 1e-9 (away from gimbal lock).
#[test]
fn euler_zyx_round_trip() {
    let cases: [[f64; 3]; 4] = [
        [0.3, 0.4, -0.7],
        [-1.0, 0.2, 1.5],
        [TAU / 8.0, -0.1, 0.6],
        [0.0, 0.0, 0.0],
    ];
    for c in &cases {
        let [roll, pitch, yaw] = *c;
        let q = UnitQuaternion::from_euler_zyx(roll, pitch, yaw);
        let [r2, p2, y2] = q.to_euler_zyx();
        assert_near(r2, roll, EPS9, "euler_round_trip roll");
        assert_near(p2, pitch, EPS9, "euler_round_trip pitch");
        assert_near(y2, yaw, EPS9, "euler_round_trip yaw");
    }
}

/// At pitch = π/2, to_euler_zyx sets yaw = 0 and roll is well-defined.
#[test]
fn gimbal_lock_singularity() {
    let q = UnitQuaternion::from_euler_zyx(0.5, FRAC_PI_2, 0.8);
    let [roll, pitch, yaw] = q.to_euler_zyx();
    // Yaw must be clamped to 0 at singularity.
    assert_near(yaw, 0.0, EPS9, "gimbal_lock yaw == 0");
    // Pitch is reconstructed via asin; at the singularity the quaternion product
    // introduces floating-point rounding so we allow 1e-7 rad (≈0.006°) here.
    assert_near(pitch, FRAC_PI_2, 1e-7, "gimbal_lock pitch == π/2");
    // Roll is well-defined (not NaN/inf).
    assert!(roll.is_finite(), "gimbal_lock roll must be finite");
    // The resulting quaternion must represent the same rotation as the input.
    // Round-trip via rotate_vec on an arbitrary vector.
    let q2 = UnitQuaternion::from_euler_zyx(roll, pitch, 0.0);
    let v = [1.0_f64, 2.0, 3.0];
    let v1 = q.rotate_vec(v);
    let v2 = q2.rotate_vec(v);
    // The small residual (~2e-8) is from f64 precision when sinp clamps to exactly 1.0
    // while the original quaternion encodes pitch slightly above π/2.
    assert_vec_near(v1, v2, 1e-7, "gimbal_lock rotation preserved");
}

// ── from_two_unit_vectors tests ───────────────────────────────────────────────

/// from == to → IDENTITY (within 1e-12).
#[test]
fn from_two_unit_vectors_identity() {
    let v = [1.0_f64, 0.0, 0.0];
    let q = UnitQuaternion::from_two_unit_vectors(v, v);
    quat_is_identity(q, EPS12, "from_two_unit_vectors_identity");
}

/// (1,0,0) → (0,1,0): 90° rotation about Z.
#[test]
fn from_two_unit_vectors_orthogonal() {
    let from = [1.0_f64, 0.0, 0.0];
    let to = [0.0_f64, 1.0, 0.0];
    let q = UnitQuaternion::from_two_unit_vectors(from, to);
    // q applied to `from` should give `to`.
    let result = q.rotate_vec(from);
    assert_vec_near(result, to, EPS9, "from_two_unit_vectors_orthogonal");
    // Must be a 90° rotation about Z: w = cos(45°), z = sin(45°), x = y = 0.
    let half_sqrt2 = (0.5_f64).sqrt();
    assert_near(q.w, half_sqrt2, EPS9, "orthogonal q.w");
    assert_near(q.x, 0.0, EPS9, "orthogonal q.x");
    assert_near(q.y, 0.0, EPS9, "orthogonal q.y");
    assert_near(q.z, half_sqrt2, EPS9, "orthogonal q.z");
}

/// (1,0,0) → (-1,0,0): antiparallel — rotated `from` lands on `to` within 1e-9.
#[test]
fn from_two_unit_vectors_antiparallel() {
    let from = [1.0_f64, 0.0, 0.0];
    let to = [-1.0_f64, 0.0, 0.0];
    let q = UnitQuaternion::from_two_unit_vectors(from, to);
    let result = q.rotate_vec(from);
    assert_vec_near(result, to, EPS9, "from_two_unit_vectors_antiparallel");
}

/// Arbitrary unit vectors: rotated `from` matches `to` to 1e-9.
#[test]
fn from_two_unit_vectors_arbitrary() {
    // from = normalise(1, 2, 3)
    let mag_f = (1.0_f64 + 4.0 + 9.0_f64).sqrt();
    let from = [1.0 / mag_f, 2.0 / mag_f, 3.0 / mag_f];
    // to = normalise(-3, 1, -1)
    let mag_t = (9.0_f64 + 1.0 + 1.0_f64).sqrt();
    let to = [-3.0 / mag_t, 1.0 / mag_t, -1.0 / mag_t];
    let q = UnitQuaternion::from_two_unit_vectors(from, to);
    let result = q.rotate_vec(from);
    assert_vec_near(result, to, EPS9, "from_two_unit_vectors_arbitrary");
}
