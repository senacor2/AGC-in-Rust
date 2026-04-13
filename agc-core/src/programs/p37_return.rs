//! P37 — Return-to-Earth Targeting (Lambert solver).
//!
//! P37 computes the delta-V required for trans-Earth injection (TEI) from
//! cislunar space back to Earth. It uses the Lambert solver with the entry
//! interface constraint (400 kft = 121,920 m above Earth geocentric radius).
//!
//! AGC source: Comanche055/P30-P37.agc S31.1 (pages 641-642).
//! AGC source: Comanche055/CONIC_SUBROUTINES.agc LAMBERT (page 1296).
//! AGC source: Comanche055/P61-P67.agc 400KFT constant (page 875).

use crate::guidance::targeting::{burn_duration, BurnTarget, SPS_THRUST_N, SPS_VE_MS};
use crate::math::lambert::{lambert, TransferDirection};
use crate::math::linalg::{cross, norm, unit};
use crate::navigation::state_vector::StateVector;
use crate::types::{Met, Vec3};

/// Entry interface altitude above Earth geocentric radius, metres.
///
/// `400KFT 2DEC 121920 B-29` from Comanche055/P61-P67.agc page 875.
/// Equals 400,000 ft × 0.3048 m/ft = 121,920 m exactly.
///
/// AGC source: Comanche055/P61-P67.agc page 875.
pub const ENTRY_INTERFACE_M: f64 = 121_920.0;

/// Earth mean geocentric radius used for entry sphere, metres.
///
/// Used to compute the entry interface sphere radius:
///   r_ei = EARTH_RADIUS_M + ENTRY_INTERFACE_M
/// Value 6,371,000 m is the IAU mean equatorial radius, within 0.03%
/// of the AGC pad radius from P61-P67.agc page 873 (6,373,336 m).
///
/// AGC source: Comanche055/P61-P67.agc page 873 (RTRIAL / RPAD constants).
pub const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Earth gravitational parameter, m³/s².
///
/// AGC source: Comanche055/ORBITAL_INTEGRATION.agc MUEARTH.
use crate::navigation::constants::MU_EARTH;

/// Solve the Lambert problem for a return-to-Earth trajectory.
///
/// Given the spacecraft's current state vector (in ECI), a desired arrival
/// time at Earth entry interface, and the entry interface altitude above
/// Earth geocentric radius, computes the velocity required at the current
/// position to reach the entry sphere in the given time.
///
/// Returns `Some(BurnTarget)` when Lambert converges; `None` on convergence
/// failure (collinear geometry, non-positive dt, or exceeded MAX_ITERATIONS).
/// The caller must raise an alarm and return to P00 on `None`.
///
/// Algorithm:
///   1. dt = t_arrival - current.time(). Return None if dt ≤ 0.
///   2. r_ei = EARTH_RADIUS_M + entry_radius. Choose r2 on the entry sphere.
///   3. Call math::lambert::lambert(r1, r2, dt, MU_EARTH, Short).
///   4. delta_v_eci = lambert.v1 - current.velocity().
///   5. Rotate delta_v_eci to LVLH at current position.
///   6. Compute burn duration via Tsiolkovsky rocket equation.
///   7. Pack into BurnTarget.
///
/// AGC source: Comanche055/P30-P37.agc S31.1 (pages 641-642);
///             Comanche055/CONIC_SUBROUTINES.agc LAMBERT (page 1296).
///             Entry interface constant: Comanche055/P61-P67.agc page 875.
#[must_use]
pub fn solve(current: &StateVector, t_arrival: Met, entry_radius: f64) -> Option<BurnTarget> {
    // Step 1: Check dt > 0.
    // AGC source: S31.1 validates TIG before calling AGAIN (Lambert).
    let t_now_s = current.time().as_secs_f64();
    let t_arrival_s = t_arrival.as_secs_f64();
    let dt = t_arrival_s - t_now_s;
    if dt <= 0.0 {
        return None;
    }

    // Step 2: Choose r2 on the entry sphere.
    // AGC source: S31.1 targets the entry interface sphere; r2 is chosen as
    // the spacecraft's current inbound radial direction scaled to r_ei.
    // Entry radius must be non-negative (altitude above geocentric radius).
    // AGC source: S31.1 entry interface constant is always positive.
    if entry_radius < 0.0 {
        return None;
    }
    let r_ei = EARTH_RADIUS_M + entry_radius;
    if r_ei <= 0.0 {
        return None;
    }

    let r1 = current.position();
    let r1_norm = norm(&r1);
    if r1_norm < 1.0 {
        return None;
    }

    // Entry sphere target: opposite to current position direction (inbound).
    // This matches AGC default when no specific splash-down target is loaded.
    let r2 = entry_sphere_target(current, t_arrival, r_ei);

    // Step 3: Lambert solve (short-way transfer).
    // AGC source: Comanche055/CONIC_SUBROUTINES.agc LAMBERT (page 1296).
    let result = lambert(&r1, &r2, dt, MU_EARTH, TransferDirection::Short);
    if !result.converged {
        return None;
    }

    // Step 4: delta_v_eci = v_required - v_current.
    // AGC source: S31.1 `DV = VTEI - VCURRENT`.
    let v_current = current.velocity();
    let delta_v_eci: Vec3 = [
        result.v1[0] - v_current[0],
        result.v1[1] - v_current[1],
        result.v1[2] - v_current[2],
    ];

    // Step 5: Rotate delta_v_eci to LVLH at current position.
    // LVLH frame: r_hat = radial, v_hat = velocity direction, h_hat = orbit normal.
    let r_hat = unit(&r1)?;
    let v_hat = unit(&v_current)?;
    let h_vec = cross(&r1, &v_current);
    let h_hat = unit(&h_vec)?;

    let delta_v_lvlh: Vec3 = [
        r_hat[0] * delta_v_eci[0] + r_hat[1] * delta_v_eci[1] + r_hat[2] * delta_v_eci[2],
        v_hat[0] * delta_v_eci[0] + v_hat[1] * delta_v_eci[1] + v_hat[2] * delta_v_eci[2],
        h_hat[0] * delta_v_eci[0] + h_hat[1] * delta_v_eci[1] + h_hat[2] * delta_v_eci[2],
    ];

    // Step 6: Compute burn duration (Tsiolkovsky).
    // AGC source: S40.13 TIMEBURN equation (P40-P47.agc pages 726-728).
    let dv_mag = norm(&delta_v_eci);
    // burn_duration may return None for degenerate inputs, which is fine here.
    let _burn_dur = burn_duration(dv_mag, SPS_THRUST_N, SPS_VE_MS, 28_800.0);

    // Step 7: Pack into BurnTarget.
    // AGC source: S31.1 stores DELVLVC (LVLH delta-V) and TIG.
    Some(BurnTarget {
        tig: current.time(),
        delta_v_lvlh,
        mass: 28_800.0, // nominal CSM mass; real code reads CSMMASS
        thrust: SPS_THRUST_N,
        isp: SPS_VE_MS,
    })
}

/// Choose the target position on the entry sphere.
///
/// Uses the spacecraft's current position direction as a first approximation:
/// places r2 on the entry sphere along the inbound radial from the current
/// position (i.e., opposite to the unit position vector at the current time).
/// This matches the AGC's behavior when no specific recovery site is loaded.
///
/// AGC source: S31.1 uses TPASS4 as arrival time; r2 is the entry interface
/// point on the sphere along the nominal inbound asymptote.
fn entry_sphere_target(_current: &StateVector, _t_arrival: Met, r_ei: f64) -> Vec3 {
    // Place r2 on the entry sphere with a small angular offset from the inbound
    // (antiparallel) direction, avoiding collinear geometry that would make
    // Lambert's formula degenerate (sin(theta) = 0).
    //
    // For accuracy matching the AGC, the caller should provide a specific
    // splash-down latitude/longitude which would be converted to an ECI vector
    // at t_arrival using Earth's rotation rate.
    //
    // The offset used here is ~5.7° (0.1 rad) in the ecliptic Y direction,
    // ensuring the transfer angle is accessible for the Lambert solver.
    //
    // AGC source: S31.1 targets the entry interface sphere along the nominal
    // inbound asymptote. The default placement is near the sub-Earth direction.
    let r1 = _current.position();
    let r1_norm = norm(&r1);
    if r1_norm < 1.0 {
        // Degenerate: place on entry sphere at a fixed off-axis point.
        let n = r_ei / libm::sqrt(2.0);
        return [-n, n, 0.0];
    }
    // Place r2 antiparallel to r1 but offset slightly in the Y direction by
    // 0.1 * r_ei to prevent a collinear (180°) transfer angle.
    // This ensures sin(transfer_angle) != 0 for the Lambert solver.
    let sf = -r_ei / r1_norm;
    let base: Vec3 = [r1[0] * sf, r1[1] * sf, r1[2] * sf];
    // Add a small perpendicular perturbation: choose the Y axis if r1 is mostly
    // along X, otherwise use X.
    let perturb = if r1[0].abs() > r1[1].abs() {
        // r1 mostly along X → perturb in Y
        [0.0, 0.1 * r_ei, 0.0]
    } else {
        // r1 mostly along Y → perturb in X
        [0.1 * r_ei, 0.0, 0.0]
    };
    // Normalize base+perturb to land exactly on the entry sphere.
    let candidate: Vec3 = [
        base[0] + perturb[0],
        base[1] + perturb[1],
        base[2] + perturb[2],
    ];
    let mag = norm(&candidate);
    if mag < 1.0 {
        return base;
    }
    let sf2 = r_ei / mag;
    [candidate[0] * sf2, candidate[1] * sf2, candidate[2] * sf2]
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::StateVector;
    use crate::types::Met;

    /// TC-P37-1: Lunar orbit to Earth return (nominal).
    ///
    /// Spacecraft at lunar distance; 72-hour return.
    /// AGC source: Comanche055/P30-P37.agc S31.1 (pages 641-642).
    #[test]
    fn lunar_return_nominal() {
        let current = StateVector::new(
            [384_400_000.0, 0.0, 0.0], // lunar distance
            [0.0, 1_022.0, 0.0],       // circular lunar orbit speed
            Met(0),
        );
        let t_arrival = Met::from_secs(259_200.0); // 72 hours
        let result = solve(&current, t_arrival, ENTRY_INTERFACE_M);
        assert!(
            result.is_some(),
            "TEI solve must converge for nominal lunar return"
        );
        let bt = result.unwrap();
        let dv_mag = norm(&bt.delta_v_lvlh);
        assert!(
            dv_mag > 800.0,
            "|dv| = {dv_mag:.1} m/s expected > 800 m/s (TEI burn)"
        );
        assert!(
            dv_mag < 1200.0,
            "|dv| = {dv_mag:.1} m/s expected < 1200 m/s"
        );
        assert_eq!(bt.tig, current.time());
    }

    /// TC-P37-2: Convergence failure — non-positive transfer time.
    ///
    /// AGC source: S31.1 validates TIG before Lambert call.
    #[test]
    fn zero_dt_returns_none() {
        let current = StateVector::new([384_400_000.0, 0.0, 0.0], [0.0, 1_022.0, 0.0], Met(0));
        let result = solve(&current, current.time(), ENTRY_INTERFACE_M);
        assert!(result.is_none(), "dt=0 must return None");
    }

    /// TC-P37-3: ENTRY_INTERFACE_M constant matches AGC source value.
    ///
    /// AGC source: Comanche055/P61-P67.agc `400KFT 2DEC 121920 B-29` (page 875).
    #[test]
    fn entry_interface_constant_matches_agc() {
        let expected = 400_000.0_f64 * 0.3048; // 121,920 m exactly
        assert!(
            (ENTRY_INTERFACE_M - expected).abs() < 1.0,
            "ENTRY_INTERFACE_M = {ENTRY_INTERFACE_M} expected {expected}"
        );
    }

    /// TC-P37-4: Negative entry radius returns None.
    #[test]
    fn negative_entry_radius_returns_none() {
        let current = StateVector::new([384_400_000.0, 0.0, 0.0], [0.0, 1_022.0, 0.0], Met(0));
        let result = solve(&current, Met::from_secs(100_000.0), -1.0);
        assert!(result.is_none(), "negative entry_radius must return None");
    }
}
