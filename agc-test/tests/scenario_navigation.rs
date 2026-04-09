//! Navigation scenario tests.
//!
//! These tests validate the orbital integration, gravity models, and the
//! AVERAGE G SERVICER cycle against expected physics.

use agc_core::math::linalg::norm;
use agc_core::navigation::gravity::MU_EARTH;
use agc_core::navigation::integration::propagate;
use agc_core::navigation::state_vector::{Frame, Refsmmat, StateVector};
use agc_core::services::average_g::{average_g, AverageGState, CYCLE_DT};
use agc_core::types::Met;
use core::f64::consts::PI;

/// Thin sqrt wrapper so agc-test does not need a direct libm dependency.
fn sqrt(x: f64) -> f64 {
    x.sqrt()
}

/// Thin fabs wrapper.
fn fabs(x: f64) -> f64 {
    x.abs()
}

fn earth_accel_no_j2(
    r: &agc_core::types::Vec3,
    _v: &agc_core::types::Vec3,
) -> agc_core::types::Vec3 {
    agc_core::navigation::gravity::point_mass(r, MU_EARTH)
}

fn circular_sv() -> StateVector {
    let r0 = 6_578_000.0_f64;
    let v_circ = sqrt(MU_EARTH / r0);
    StateVector {
        frame: Frame::Eci,
        r: [r0, 0.0, 0.0],
        v: [0.0, v_circ, 0.0],
        t: Met::ZERO,
    }
}

/// AGC source: ORBITAL_INTEGRATION.agc — Cowell integrator energy conservation.
///
/// Propagate a circular orbit through half a period and verify the orbital
/// radius is conserved within 100 m (energy conservation criterion).
#[test]
fn circular_orbit_energy_conservation() {
    let sv0 = circular_sv();
    let r0 = norm(&sv0.r);
    let v_circ = norm(&sv0.v);
    let t_orbit = 2.0 * PI * r0 / v_circ;

    let sv_half = propagate(&sv0, t_orbit / 2.0, 10.0, earth_accel_no_j2);
    let r_final = norm(&sv_half.r);

    let dr = fabs(r_final - r0);
    assert!(
        dr < 100.0,
        "orbital radius drifted {dr:.3} m over half period (limit 100 m)"
    );
}

/// AGC source: SERVICER207.agc — zero PIPA step advances state vector.
///
/// With zero PIPA inputs, gravity alone must move the spacecraft — the
/// position after one 2-second cycle must differ from the initial position.
#[test]
fn average_g_zero_pipa_advances_position() {
    let sv = circular_sv();
    let result = average_g(&sv, [0, 0, 0], &Refsmmat::IDENTITY, CYCLE_DT, &mut AverageGState::new());

    let displacement = norm(&[
        result.sv.r[0] - sv.r[0],
        result.sv.r[1] - sv.r[1],
        result.sv.r[2] - sv.r[2],
    ]);

    assert!(
        displacement > 1.0,
        "position should advance by >1 m after 2-second cycle, got {displacement:.3} m"
    );
    assert_eq!(result.sv.t.0, 200, "MET should advance by 200 centiseconds");
}

/// AGC source: SERVICER207.agc — PIPA delta-V is rotated by REFSMMAT.
///
/// PIPA counts in the stable-member +x axis must be rotated by REFSMMAT into
/// ECI before being added to the velocity. With a 90° Z-rotation, stable-member
/// +x maps to ECI +y.
#[test]
fn average_g_pipa_rotated_by_refsmmat() {
    let sv = circular_sv();

    // Identity REFSMMAT: stable-member +x → ECI +x.
    let result_identity = average_g(&sv, [100, 0, 0], &Refsmmat::IDENTITY, CYCLE_DT, &mut AverageGState::new());
    let dv_identity = result_identity.delta_v_total.0;
    assert!(dv_identity[0] > 0.0, "identity: dv[0] should be +x");
    assert!(
        dv_identity[1].abs() < 1e-10,
        "identity: dv[1] should be zero"
    );

    // REFSMMAT where ECI +y maps to SM +x → sm_to_eci maps SM +x to ECI +y.
    let rot_z: [[f64; 3]; 3] = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
    let result_rotated = average_g(&sv, [100, 0, 0], &Refsmmat(rot_z), CYCLE_DT, &mut AverageGState::new());
    let dv_rotated = result_rotated.delta_v_total.0;
    assert!(
        dv_rotated[0].abs() < 1e-10,
        "rotated: dv[0] should be ~0, got {}",
        dv_rotated[0]
    );
    assert!(
        dv_rotated[1] > 0.0,
        "rotated: dv[1] should be +y, got {}",
        dv_rotated[1]
    );
}
