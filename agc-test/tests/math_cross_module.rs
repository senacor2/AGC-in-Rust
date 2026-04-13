// Spec: specs/math-linalg.md §2 Rust API, §7 TC-LINALG-07, TC-LINALG-08
//       specs/math-trig.md §2 Rust API
//       specs/navigation-state-vector.md §Coordinate Frames (REFSMMAT usage)
//
// Integration tests for math layer cross-module wiring.
// Verifies that trig + linalg type-system boundaries are correctly connected:
//   - linalg::rotx applied to a vector uses trig implicitly (libm::sin/cos).
//   - CduAngle::to_radians() feeds correctly into math::trig::sin.
//   - REFSMMAT-style round-trip: cross/unit → mxv → transpose → identity.
//
// No global alarm state is touched; parallel execution is safe (no #[serial]).

use agc_core::{
    math::{
        linalg::{add, cross, dot, mxm, mxv, norm, rotx, scale, transpose, unit},
        trig::{asin_clamped, cos, sin},
    },
    types::CduAngle,
};
use core::f64::consts::PI;

// ── Test 1: rotx(π/2) rotates [0,1,0] into [0,0,1] ──────────────────────────

/// Spec: specs/math-linalg.md §TC-LINALG-07 (rotation matrix orthogonality)
///              §TC-LINALG-08 (90° rotation)
///       specs/math-trig.md §TC-TRIG-01 (sin/cos values)
///
/// rotx(π/2) = | 1   0    0  |
///              | 0   0  -1  |
///              | 0   1   0  |
///
/// Rotating [0, 1, 0]:
///   result[0] = 1*0 + 0*1 + 0*0 = 0
///   result[1] = 0*0 + 0*1 + (-1)*0 = 0
///   result[2] = 0*0 + 1*1 + 0*0 = 1
///
/// Expected: [0, 0, 1] within 1e-12.
/// This verifies that linalg::rotx and math::trig are wired together correctly
/// (rotx calls libm::sin/cos internally, same as math::trig wrappers).
#[test]
fn rotx_quarter_turn_wires_trig_and_linalg() {
    // Spec: specs/math-linalg.md §TC-LINALG-08
    //       specs/math-trig.md §TC-TRIG-01
    let rx90 = rotx(PI / 2.0);
    let v_in = [0.0_f64, 1.0, 0.0];
    let v_out = mxv(&rx90, &v_in);

    // Expected: [0, 0, 1]
    assert!(
        v_out[0].abs() < 1e-12,
        "rotx(π/2) · [0,1,0] x-component should be 0, got {}",
        v_out[0]
    );
    assert!(
        v_out[1].abs() < 1e-12,
        "rotx(π/2) · [0,1,0] y-component should be 0, got {}",
        v_out[1]
    );
    assert!(
        (v_out[2] - 1.0).abs() < 1e-12,
        "rotx(π/2) · [0,1,0] z-component should be 1, got {}",
        v_out[2]
    );
}

// ── Test 2: CduAngle round-trip through trig ─────────────────────────────────

/// Spec: specs/math-trig.md §TC-TRIG-01, §TC-TRIG-02 (Pythagorean identity)
///       specs/navigation-state-vector.md §Coordinate Frames (CDU → radians pipeline)
///       docs/agc-reference-constants.md (CDU counts: 65536 per revolution)
///
/// CduAngle (u16, 65536 counts per revolution) → to_radians() → math::trig::sin
/// verifies the type-boundary wiring is consistent.
///
/// Test: CduAngle representing π/4 (90°/2 = 8192 counts = 45°):
///   sin(π/4) = cos(π/4) = √2/2 ≈ 0.7071
///   sin²(θ) + cos²(θ) = 1.0  (Pythagorean identity, within 2e-15)
#[test]
fn cdu_angle_to_radians_feeds_trig_correctly() {
    // Spec: specs/math-trig.md §TC-TRIG-02 — Pythagorean identity
    //       CDU scale: 65536 counts = 2π radians (docs/agc-reference-constants.md)
    //
    // CduAngle(16384) = 16384/65536 * 2π = π/2 radians (90 degrees)
    // CduAngle(8192)  =  8192/65536 * 2π = π/4 radians (45 degrees)
    let angle_45 = CduAngle::from_counts(8192); // 45 degrees
    let theta = angle_45.to_radians(); // should be ≈ π/4

    // Verify conversion is close to π/4
    assert!(
        (theta - PI / 4.0).abs() < 1e-4,
        "CduAngle(8192) should be ≈ π/4 = {:.6} radians, got {theta:.6}",
        PI / 4.0
    );

    // Feed into math::trig::sin and ::cos
    let s = sin(theta);
    let c = cos(theta);

    // sin(π/4) ≈ 0.7071 = √2/2
    let sqrt2_over_2 = (2.0_f64).sqrt() / 2.0;
    assert!(
        (s - sqrt2_over_2).abs() < 1e-4,
        "sin(CduAngle(8192).to_radians()) should be ≈ √2/2, got {s:.6}"
    );

    // Pythagorean identity: sin² + cos² = 1.0
    let pythagorean = s * s + c * c;
    assert!(
        (pythagorean - 1.0).abs() < 2e-15,
        "sin²(θ) + cos²(θ) = {pythagorean:.15e} must be 1.0 within 2e-15"
    );

    // Also verify asin_clamped round-trip: asin(sin(θ)) ≈ θ for θ in [-π/2, π/2]
    let theta_recovered = asin_clamped(s);
    assert!(
        (theta_recovered - theta).abs() < 1e-10,
        "asin(sin(θ)) round-trip: θ = {theta:.8}, recovered = {theta_recovered:.8}"
    );
}

// ── Test 3: REFSMMAT round-trip — cross/unit → mxv → transpose → identity ────

/// Spec: specs/math-linalg.md §TC-LINALG-07, TC-LINALG-06
///       specs/navigation-state-vector.md §Coordinate Frames (REFSMMAT = SM → ECI rotation)
///
/// Build a 3×3 rotation matrix (REFSMMAT analogue) from three orthonormal basis vectors
/// constructed via cross and unit. Then:
///   1. Transform a test vector v via M.
///   2. Inverse-transform (using transpose, since M is orthogonal) to recover v.
///   3. Assert round-trip identity within 1e-10.
///
/// This end-to-end test verifies that cross, unit, mxv, and transpose are all
/// correctly connected in the math layer, matching the CALCRVG usage of REFSMMAT
/// (SERVICER207.agc: `VXM VSL1 REFSMMAT` = transpose(REFSMMAT) × DELV).
#[test]
fn refsmmat_roundtrip_via_cross_unit_mxv_transpose() {
    // Spec: specs/math-linalg.md §TC-LINALG-07 (orthogonality of rotation matrix)
    //       specs/navigation-state-vector.md §Coordinate Frames
    //
    // Build an orthonormal basis from a known rotation:
    //   col_x = unit([1, 1, 0]) (45° in xy-plane)
    //   col_z = unit(cross(col_x, [0, 0, 1]))
    //   col_y = cross(col_z, col_x)  (complete right-hand set)

    let raw_x = [1.0_f64, 1.0, 0.0]; // not unit yet
    let col_x = unit(&raw_x).expect("raw_x is non-zero, unit must succeed");

    // col_z = unit(col_x × ẑ)  — perpendicular to col_x in the xy-plane but along z
    // Actually: col_x × ẑ where ẑ = [0,0,1]
    // = [1/√2, 1/√2, 0] × [0, 0, 1]
    // = [1/√2*1 - 0*0, 0*0 - 1/√2*1, 1/√2*0 - 1/√2*0]
    // = [1/√2, -1/√2, 0]  — this is in the xy-plane, perpendicular to col_x
    // We want something with a z-component, so use [0,0,1] cross col_x instead.
    let z_hat = [0.0_f64, 0.0, 1.0];
    let raw_y = cross(&z_hat, &col_x); // perpendicular to both col_x and ẑ
    let col_y = unit(&raw_y).expect("cross product of non-parallel vectors must be non-zero");

    // col_z = col_x × col_y  (right-hand rule)
    let col_z = cross(&col_x, &col_y);
    // col_z should already be unit length (cross of two unit vectors in an orthonormal set)
    let col_z_norm = norm(&col_z);
    assert!(
        (col_z_norm - 1.0).abs() < 1e-12,
        "col_z from cross of orthonormal pair should have unit norm, got {col_z_norm}"
    );

    // Build the rotation matrix M (column-major stored as row-major Mat3x3):
    // M = [col_x | col_y | col_z]  (columns of M are the basis vectors)
    // Row-major form: M[i][j] = basis_j[i]
    let m: [[f64; 3]; 3] = [
        [col_x[0], col_y[0], col_z[0]], // row 0
        [col_x[1], col_y[1], col_z[1]], // row 1
        [col_x[2], col_y[2], col_z[2]], // row 2
    ];

    // Verify M is orthogonal: M × M^T = I  (within 1e-12)
    let mmt = mxm(&m, &transpose(&m));
    for i in 0..3 {
        for j in 0..3 {
            let expected = if i == j { 1.0 } else { 0.0 };
            assert!(
                (mmt[i][j] - expected).abs() < 1e-12,
                "M·M^T[{i}][{j}] = {}, expected {expected} (orthogonality check)",
                mmt[i][j]
            );
        }
    }

    // Pick a test vector (in ECI frame)
    let v_eci = [100.0_f64, 200.0, 300.0]; // arbitrary test vector

    // Forward transform: v_sm = M × v_eci  (rotate to Stable-Member frame)
    let v_sm = mxv(&m, &v_eci);

    // Inverse transform: v_eci_recovered = M^T × v_sm  (VXM semantics in AGC)
    // Spec: specs/math-linalg.md §VXM = M^T × v
    let v_eci_recovered = mxv(&transpose(&m), &v_sm);

    // Round-trip must recover original vector within 1e-10
    // Spec: specs/math-linalg.md §TC-LINALG-07 (orthogonality invariant)
    for i in 0..3 {
        assert!(
            (v_eci_recovered[i] - v_eci[i]).abs() < 1e-10,
            "round-trip component {i}: original = {}, recovered = {}, error = {:.3e}",
            v_eci[i],
            v_eci_recovered[i],
            (v_eci_recovered[i] - v_eci[i]).abs()
        );
    }
}

// ── Bonus Test: norm and dot consistency via linalg ───────────────────────────

/// Spec: specs/math-linalg.md §TC-LINALG-02, TC-LINALG-03
///
/// Verifies that dot(v, v) == norm_sq(v) and norm(v) == sqrt(dot(v, v))
/// when operating through the public API — a simple cross-module consistency check.
#[test]
fn norm_and_dot_consistency() {
    // Spec: specs/math-linalg.md §TC-LINALG-03
    let v = [3.0_f64, 4.0, 5.0];
    let expected_norm_sq = 9.0 + 16.0 + 25.0; // 50.0
    let expected_norm = 50.0_f64.sqrt();

    let dot_self = dot(&v, &v);
    let n = norm(&v);

    assert!(
        (dot_self - expected_norm_sq).abs() < 1e-12,
        "dot(v, v) = {dot_self}, expected {expected_norm_sq}"
    );
    assert!(
        (n - expected_norm).abs() < 1e-12,
        "norm(v) = {n}, expected {expected_norm}"
    );
    let dot_sqrt = dot_self.sqrt();
    assert!(
        (n - dot_sqrt).abs() < 1e-12,
        "norm(v) != sqrt(dot(v,v)): {n} vs {dot_sqrt}"
    );

    // scale + add round-trip: add(scale(v, 2), scale(v, -1)) == v
    let v2 = scale(&v, 2.0);
    let vm1 = scale(&v, -1.0);
    let sum = add(&v2, &vm1);
    for i in 0..3 {
        assert!(
            (sum[i] - v[i]).abs() < 1e-15,
            "add(2v, -v)[{i}] = {}, expected {}",
            sum[i],
            v[i]
        );
    }
}
