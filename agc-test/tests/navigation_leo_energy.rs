// Spec: specs/navigation-integration.md §Test 1 — Energy conservation over one LEO orbit
//       specs/navigation-integration.md §Test Cases
//       docs/agc-reference-constants.md — MU_EARTH, RE_EARTH cited below
//
// Integration tests for orbital energy conservation through `navigation::integration::propagate`.
// These tests exercise the full RK4 propagator end-to-end, verifying the cross-module chain:
//   constants → integration → StateVector  (point-mass gravity for clean energy invariant)
//
// The energy conservation test uses point-mass-only gravity (no J2) because:
//   - J2 creates a non-conservative perturbation; the orbital energy in a J2 field
//     oscillates at the nodal precession frequency and is not conserved over one orbit.
//   - The unit test in integration.rs also uses point-mass only for the 1e-6 tolerance.
//   - A separate gravity_sanity test covers J2 magnitude and direction.
//
// No global alarm state is touched; parallel execution is safe (no #[serial]).

use agc_core::{
    navigation::{
        constants::{MU_EARTH, RE_EARTH},
        integration::propagate,
        state_vector::StateVector,
    },
    types::{Met, Vec3},
};

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Point-mass gravity (no J2): a = -(MU_EARTH / |r|³) · r  (m/s²)
///
/// Used instead of earth_gravity (which includes J2) because J2 makes energy
/// non-constant over one orbit — incompatible with the 1e-6 tolerance.
fn point_mass_grav(r: &Vec3, _t: Met) -> Vec3 {
    use agc_core::math::linalg::norm;
    let rm = norm(r);
    if rm < 1.0 {
        return [0.0; 3];
    }
    let coeff = -MU_EARTH / (rm * rm * rm);
    [r[0] * coeff, r[1] * coeff, r[2] * coeff]
}

/// Specific orbital energy: E = v²/2 - MU/r  (m²/s²)
fn specific_energy(state: &StateVector) -> f64 {
    let v = state.speed();
    let r = state.radius();
    0.5 * v * v - MU_EARTH / r
}

/// Orbital period for a circular orbit at radius `r_m` (seconds).
fn circular_period(r_m: f64) -> f64 {
    use std::f64::consts::PI;
    2.0 * PI * r_m.powf(1.5) / MU_EARTH.sqrt()
}

// ── Test 1: Energy conservation over one LEO period ──────────────────────────

/// Spec: specs/navigation-integration.md §Test 1 / §Invariants
///       docs/agc-reference-constants.md MU_EARTH = 3.986_032e14 m³/s²
///                                        RE_EARTH = 6_373_338.0 m
///
/// 185 km circular LEO (altitude chosen to match the spec §Test 1 example).
///   r = RE_EARTH + 185_000 m
///   v = sqrt(MU_EARTH / r) purely tangential in equatorial plane
///
/// Propagated for one full orbital period with dt = 2 s (matching SERVICER CYCLE_DT).
/// Gravity: point-mass only (no J2) to satisfy the conservative-energy invariant.
///
/// Primary tolerance: |ΔE / E₀| < 1e-6 (specs/navigation-integration.md §Invariants)
/// Secondary tolerance: |r₁| within 1 km of |r₀|.
#[test]
fn leo_energy_conservation_one_orbit() {
    // Spec: specs/navigation-integration.md §Test 1
    // r = RE_EARTH + 185_000 m, v = sqrt(MU_EARTH / r) tangential
    let r_mag = RE_EARTH + 185_000.0; // metres: 185 km altitude
    let v_circ = (MU_EARTH / r_mag).sqrt(); // m/s: circular orbital speed

    let r0: Vec3 = [r_mag, 0.0, 0.0];
    let v0: Vec3 = [0.0, v_circ, 0.0];
    let state0 = StateVector::new(r0, v0, Met(0));

    let e0 = specific_energy(&state0);

    // T = 2π·√(r³/MU_EARTH)
    let t_orbit = circular_period(r_mag);

    // dt = 2 s (SERVICER CYCLE_DT from docs/agc-reference-constants.md)
    let dt = 2.0_f64;
    let n_steps = (t_orbit / dt).ceil() as usize;

    let mut state = state0;
    for _ in 0..n_steps {
        state = propagate(&state, dt, &point_mass_grav);
    }

    let e1 = specific_energy(&state);

    // Tolerance: 1e-6 relative (specs/navigation-integration.md §Invariants)
    // RK4 at dt=2s accumulates ~10^-7 per orbit → well within 1e-6.
    let rel_err = (e1 - e0).abs() / e0.abs();
    assert!(
        rel_err < 1e-6,
        "One-orbit energy conservation: |ΔE/E₀| = {rel_err:.3e} (must be < 1e-6)"
    );

    // Position radius must not drift more than 1 km over one orbit.
    let r0_mag = state0.radius();
    let r1_mag = state.radius();
    let dr = (r1_mag - r0_mag).abs();
    assert!(
        dr < 1000.0,
        "Position radius drift: |r₁ - r₀| = {dr:.1} m (must be < 1000 m = 1 km)"
    );
}

// ── Test 2: Energy conservation over 10 LEO orbits ───────────────────────────

/// Spec: specs/navigation-integration.md §Test 1 (10-orbit extension)
///       docs/agc-reference-constants.md — RK4 drift documented as acceptable
///
/// Same 185 km LEO propagated for 10 full orbital periods.
/// Tolerance loosened to 1e-4 relative because RK4 truncation error accumulates
/// linearly: ~2e-7 per orbit × 10 = ~2e-6, well within 1e-4.
/// The looser bound ensures the test remains stable under accumulated
/// floating-point error from ~27 000 RK4 steps.
///
/// Purpose: validates integrator numerical stability over extended coast phases
/// (e.g., LEO coast before TLI burn).
#[test]
fn leo_energy_conservation_ten_orbits() {
    // Spec: specs/navigation-integration.md §Test 1 (10-orbit extension)
    // Tolerance: 1e-4 relative — see comment above for rationale.
    let r_mag = RE_EARTH + 185_000.0;
    let v_circ = (MU_EARTH / r_mag).sqrt();

    let r0: Vec3 = [r_mag, 0.0, 0.0];
    let v0: Vec3 = [0.0, v_circ, 0.0];
    let state0 = StateVector::new(r0, v0, Met(0));

    let e0 = specific_energy(&state0);

    let t_orbit = circular_period(r_mag);
    let dt = 2.0_f64;
    let n_steps = (10.0 * t_orbit / dt).ceil() as usize;

    let mut state = state0;
    for _ in 0..n_steps {
        state = propagate(&state, dt, &point_mass_grav);
    }

    let e1 = specific_energy(&state);

    // Tolerance: 1e-4 relative (RK4 drift over 10 orbits, dt=2s)
    let rel_err = (e1 - e0).abs() / e0.abs();
    assert!(
        rel_err < 1e-4,
        "Ten-orbit energy conservation: |ΔE/E₀| = {rel_err:.3e} (must be < 1e-4)"
    );
}
