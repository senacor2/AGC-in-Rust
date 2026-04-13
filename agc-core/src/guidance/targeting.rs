//! Guidance targeting: burn-time prediction and VG vector computation.
//!
//! Answers the question "given a desired velocity change and a time to execute it,
//! what is the initial VG vector and how long will the burn last?"
//!
//! Implements the P30 external delta-V path (S30.1 DELVSLV→DELVSIN rotation) and
//! the Tsiolkovsky TGO computation (S40.13 TIMEBURN).
//!
//! AGC source: Comanche055/P30-P37.agc
//!   P30, S30.1 (pages 635-641), S31.1 (pages 641-642).
//! AGC source: Comanche055/P40-P47.agc
//!   P40CSM (pages 684-685), S40.1 (pages 709-712), S40.13 (pages 726-728).
//!   Constants block (page 689): FENG, 2VEXHUST, EMDOT.

use crate::math::kepler::kepler;
use crate::math::linalg::{cross, unit};
use crate::navigation::constants::MU_EARTH;
use crate::navigation::state_vector::StateVector;
use crate::types::{Met, Vec3};

/// SPS main engine thrust, Newtons.
///
/// AGC source: Comanche055/P40-P47.agc
///   `FENG 2DEC 9.1188544 B-7` (page 689): stored in millions of Newtons × 10⁴.
///   Value: 9.1188544 × 10⁴ N = 91 188.544 N.
pub const SPS_THRUST_N: f64 = 91_188.544;

/// SPS effective exhaust velocity, m/s.
///
/// AGC source: Comanche055/P40-P47.agc
///   `2VEXHUST 2DEC 63.020792 B-7` (page 744): stores 2 × v_e in m/cs.
///   2 × v_e = 63.020792 m/cs × (1/100 s/cs × 100 cs/s) ... actually:
///   63.020792 m/cs × 100 cs/s = 6302.0792 m/s → that is 2×v_e.
///   Therefore v_e = 3151.0396 m/s.
///   Note: `2VEXHUST` encodes twice the exhaust velocity; this constant is
///   the true v_e = 2VEXHUST / 2.
pub const SPS_VE_MS: f64 = 3_151.039_6;

/// Standard gravitational acceleration at Earth's surface (g₀), m/s².
///
/// Used to convert exhaust velocity to specific impulse (Isp).
pub const G0_MS2: f64 = 9.806_65;

/// SPS specific impulse derived from exhaust velocity, seconds.
///
/// Isp = v_e / g₀ = 3151.0396 / 9.80665 ≈ 321.3 s.
///
/// AGC source: Implicit in P40-P47.agc constants (page 689).
pub const SPS_ISP_S: f64 = SPS_VE_MS / G0_MS2;

/// Burn targeting parameters for a single SPS or RCS maneuver.
///
/// AGC equivalents: TIG (B+28 cs), DELVSIN/DELVSLV (B+7 m/cs vector),
/// WEIGHT/G (B+16 kg), F (B+7 M-Newtons), 2VEXHUST/2 (B+7 m/cs).
///
/// AGC source: Comanche055/P40-P47.agc S40.1 erasable initialisation block (pages 709-710).
pub struct BurnTarget {
    /// Time of ignition, mission elapsed time.
    ///
    /// AGC: TIG, DP B+28 centiseconds.
    pub tig: Met,

    /// Desired delta-V in LVLH (Local Vertical / Local Horizontal) frame, m/s.
    ///
    /// LVLH convention: +X = radial outward, +Y = velocity direction, +Z = orbit-normal.
    /// Rotated to ECI by `predict_vg_at_ignition` via the LOMAT construction.
    ///
    /// AGC: DELVSLV (B+7 m/cs); rotated to ECI by S30.1 → DELVSIN.
    pub delta_v_lvlh: Vec3,

    /// Vehicle mass at TIG, kg.
    ///
    /// AGC: WEIGHT/G, SP B+16 kg.
    pub mass: f64,

    /// Engine thrust, Newtons.
    ///
    /// AGC: F (FENG for SPS = 91 188.544 N, stored B+7 M-Newtons/E4).
    pub thrust: f64,

    /// Exhaust velocity, m/s (true v_e = 2VEXHUST / 2).
    ///
    /// AGC: `2VEXHUST 2DEC 63.020792 B-7` (page 744); stored as twice v_e.
    /// Rust stores the true v_e = 3151.0396 m/s.
    /// Named `isp` in the struct to match spec wording but contains v_e (m/s).
    pub isp: f64, // exhaust velocity in m/s (v_e = Isp * g0)
}

/// Predicted burn duration via the Tsiolkovsky rocket equation.
///
/// Returns `Some(seconds)` for valid inputs; `None` when any of `mass`, `thrust`,
/// or `exhaust_velocity` is ≤ 0.0 (alarm conditions in the AGC — S40.13 assumed
/// valid WEIGHT/G and F at call time).
///
/// Formula:
///   mdot    = thrust / exhaust_velocity
///   t_burn  = (m0 / mdot) * (1 − exp(−|delta_v| / v_e))
///
/// The AGC S40.13 uses a piecewise linear approximation to avoid interpreter overflow;
/// this Rust function uses the exact closed-form expression, valid for all burn durations.
///
/// For `delta_v_mag == 0.0`, returns `Some(0.0)`.
///
/// AGC source: Comanche055/P40-P47.agc, S40.13 routine, pages 726-728.
///
/// Inputs:
///   `delta_v_mag`       — required speed change, m/s
///   `thrust`            — engine thrust, N
///   `exhaust_velocity`  — effective exhaust velocity (v_e), m/s
///   `mass`              — initial vehicle mass at TIG, kg
/// Output: burn duration in seconds.
#[must_use]
pub fn burn_duration(
    delta_v_mag: f64,
    thrust: f64,
    exhaust_velocity: f64,
    mass: f64,
) -> Option<f64> {
    if mass <= 0.0 || thrust <= 0.0 || exhaust_velocity <= 0.0 {
        return None;
    }
    if delta_v_mag == 0.0 {
        return Some(0.0);
    }
    // mass flow rate: mdot = F / v_e
    let mdot = thrust / exhaust_velocity;
    // Tsiolkovsky: t = (m0 / mdot) * (1 − exp(−dv / v_e))
    let exp_term = libm::exp(-delta_v_mag / exhaust_velocity);
    let t = (mass / mdot) * (1.0 - exp_term);
    Some(t)
}

/// Velocity-to-be-gained (VG) vector at TIG in the ECI frame.
///
/// For P30 external delta-V burns (`XDELVFLG = 1`):
///   Propagates `current` state forward to `target.tig`, constructs the
///   LVLH-to-ECI rotation matrix at TIG (LOMAT), and rotates
///   `target.delta_v_lvlh` to ECI.
///   This mirrors S30.1 (DELVSLV → DELVSIN via LOMAT).
///
/// For Lambert / aimpoint burns (`XDELVFLG = 0`, not this function):
///   Use `math::lambert::lambert` and pass its result directly as VG.
///
/// Returns the VG vector in m/s (ECI), identical in meaning to AGC VGTIG (B+7 m/cs).
/// Result is always a finite vector for finite inputs (no `unwrap`).
///
/// AGC source: Comanche055/P30-P37.agc, S30.1 (DELVSLV → DELVSIN), pages 639-640.
///             Comanche055/P40-P47.agc, S40.1 delta-V path, pages 709-711.
///
/// Inputs: current state vector (ECI), burn target.
/// Output: VG in ECI frame, m/s.
#[must_use]
pub fn predict_vg_at_ignition(current: &StateVector, target: &BurnTarget) -> Vec3 {
    // Propagate state to TIG via Kepler
    let dt_to_tig = {
        let tig_s = target.tig.as_secs_f64();
        let now_s = current.time().as_secs_f64();
        tig_s - now_s
    };

    let (r_tig, v_tig) = if dt_to_tig.abs() < 0.01 {
        // Already at TIG (within 10 ms)
        (current.position(), current.velocity())
    } else {
        let kep = kepler(
            &current.position(),
            &current.velocity(),
            dt_to_tig,
            MU_EARTH,
        );
        if kep.converged {
            (kep.r, kep.v)
        } else {
            // Kepler did not converge: use current state as best approximation
            (current.position(), current.velocity())
        }
    };

    // Construct LVLH-to-ECI rotation matrix at TIG
    // LVLH frame: +X = radial outward (r-hat), +Y = velocity direction (v-hat),
    //             +Z = orbit normal (h-hat, Z = X × Y, or r × v direction)
    //
    // AGC: LOMAT is the Local Orientation Matrix computed at TIG by the LOMAT routine
    // (P30-P37.agc page 639). The three columns of LOMAT are:
    //   Col 0: unit(r)         — radial (LVLH +X)
    //   Col 1: unit(v)         — velocity (LVLH +Y)
    //   Col 2: unit(r × v)     — orbit normal (LVLH +Z)
    // DELVSLV in LVLH = LOMAT × DELVSLV in ECI, so
    // DELVSIN (ECI) = LOMAT^T × DELVSLV (if LOMAT maps ECI→LVLH)
    // or equivalently DELVSIN = LOMAT × DELVSLV if LOMAT maps LVLH→ECI.
    //
    // The convention here: LOMAT transforms LVLH components to ECI components.
    // Row i of LOMAT = i-th ECI coordinate of LVLH axis i.
    // DELVSIN[i] = sum_j LOMAT[i][j] * DELVSLV[j]

    let r_hat = match unit(&r_tig) {
        Some(u) => u,
        None => return target.delta_v_lvlh, // degenerate: return as-is
    };
    let v_hat = match unit(&v_tig) {
        Some(u) => u,
        None => return target.delta_v_lvlh,
    };
    let h_hat = {
        let h = cross(&r_tig, &v_tig);
        match unit(&h) {
            Some(u) => u,
            None => return target.delta_v_lvlh,
        }
    };

    // LOMAT (3×3, row-major): LOMAT[eci_component][lvlh_axis]
    // Col 0 = r_hat (radial), Col 1 = v_hat (velocity), Col 2 = h_hat (normal)
    let dv = &target.delta_v_lvlh;
    // DELVSIN = r_hat * dv[0] + v_hat * dv[1] + h_hat * dv[2]
    [
        r_hat[0] * dv[0] + v_hat[0] * dv[1] + h_hat[0] * dv[2],
        r_hat[1] * dv[0] + v_hat[1] * dv[1] + h_hat[1] * dv[2],
        r_hat[2] * dv[0] + v_hat[2] * dv[1] + h_hat[2] * dv[2],
    ]
}

/// Canonical SPS engine constants from Comanche055.
///
/// Returns `(thrust_N, exhaust_velocity_m_s)`.
///
/// - `thrust_N`: 91 188.544 N
///   AGC: P40-P47.agc `FENG 2DEC 9.1188544 B-7` (page 689).
/// - `exhaust_velocity_m_s`: 3 151.039_6 m/s
///   AGC: P40-P47.agc `2VEXHUST 2DEC 63.020792 B-7` (page 744);
///   stored as twice v_e; this function returns the true v_e = 63.020792/2 m/cs × 100 cs/s.
///
/// AGC source: Comanche055/P40-P47.agc, constants block, pages 689, 744.
#[must_use]
pub fn sps_constants() -> (f64, f64) {
    (SPS_THRUST_N, SPS_VE_MS)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::norm;
    use crate::navigation::constants::MU_EARTH;
    use crate::navigation::state_vector::StateVector;
    use crate::types::Met;

    /// TC-T1: 50 m/s prograde SPS burn duration (hand-computed).
    ///
    /// mdot = 91188.544 / 3151.0396 ≈ 28.941 kg/s
    /// t = (28800 / 28.941) * (1 - exp(-50 / 3151.04)) ≈ 14.4 s
    #[test]
    fn burn_duration_50ms_sps() {
        let (thrust, ve) = sps_constants();
        let mass = 28_800.0_f64;
        let dv = 50.0_f64;

        let t = burn_duration(dv, thrust, ve, mass).expect("should return Some");

        let mdot = thrust / ve;
        let expected = (mass / mdot) * (1.0 - libm::exp(-dv / ve));
        assert!((t - expected).abs() < 0.1, "t = {t} expected ≈ {expected}");
        // Rough sanity: should be around 14 s
        assert!(t > 10.0 && t < 20.0, "t = {t} s");
    }

    /// TC-T2: Zero delta-V → Some(0.0).
    #[test]
    fn burn_duration_zero_dv() {
        let (thrust, ve) = sps_constants();
        let result = burn_duration(0.0, thrust, ve, 28_800.0);
        assert_eq!(result, Some(0.0));
    }

    /// TC-T3: Short burn (3 m/s) — exact formula is numerically stable.
    ///
    /// mdot = 91188.544 / 3151.0396 ≈ 28.941 kg/s
    /// t = (28800 / 28.941) * (1 - exp(-3 / 3151.04)) ≈ 0.844 s
    #[test]
    fn burn_duration_short_burn() {
        let (thrust, ve) = sps_constants();
        let mass = 28_800.0_f64;
        let dv = 3.0_f64;

        let t = burn_duration(dv, thrust, ve, mass).expect("should return Some");
        let mdot = thrust / ve;
        let expected = (mass / mdot) * (1.0 - libm::exp(-dv / ve));
        assert!((t - expected).abs() < 0.001, "t = {t} expected {expected}");
    }

    /// TC-T4: LVLH-to-ECI rotation — prograde burn stays prograde.
    ///
    /// Circular orbit at 185 km; prograde LVLH burn (Y=50, X=Z=0).
    /// Expected: VG ≈ 50 m/s along velocity vector.
    #[test]
    fn predict_vg_prograde_burn() {
        let r_185 = 6_556_370.0_f64;
        let v_c = libm::sqrt(MU_EARTH / r_185);

        let r0 = [r_185, 0.0, 0.0];
        let v0 = [0.0, v_c, 0.0]; // prograde (Y direction)
        let state = StateVector::new(r0, v0, Met::from_centiseconds(0));

        // Prograde burn: LVLH +Y = velocity direction → ECI = [0, 1, 0] * 50 m/s
        let target = BurnTarget {
            tig: Met::from_centiseconds(0), // burn now
            delta_v_lvlh: [0.0, 50.0, 0.0],
            mass: 28_800.0,
            thrust: SPS_THRUST_N,
            isp: SPS_VE_MS,
        };

        let vg = predict_vg_at_ignition(&state, &target);
        let vg_mag = norm(&vg);

        // Magnitude should be ≈ 50 m/s
        assert!((vg_mag - 50.0).abs() < 0.01, "|VG| = {vg_mag}");

        // For a prograde circular orbit at X=r, V=[0,v,0]:
        // r_hat = [1,0,0], v_hat = [0,1,0], h_hat = [0,0,1]
        // DELVSLV = [0,50,0] → DELVSIN = 0*[1,0,0] + 50*[0,1,0] + 0*[0,0,1] = [0,50,0]
        assert!(vg[0].abs() < 0.01, "radial component = {}", vg[0]);
        assert!(
            (vg[1] - 50.0).abs() < 0.01,
            "velocity component = {}",
            vg[1]
        );
        assert!(vg[2].abs() < 0.01, "normal component = {}", vg[2]);
    }

    /// TC-T5: Invalid inputs to burn_duration → None.
    #[test]
    fn burn_duration_invalid_inputs() {
        let (thrust, ve) = sps_constants();
        assert!(
            burn_duration(50.0, thrust, ve, -1.0).is_none(),
            "negative mass"
        );
        assert!(
            burn_duration(50.0, 0.0, ve, 28800.0).is_none(),
            "zero thrust"
        );
        assert!(
            burn_duration(50.0, thrust, 0.0, 28800.0).is_none(),
            "zero ve"
        );
        assert!(
            burn_duration(50.0, thrust, -1.0, 28800.0).is_none(),
            "negative ve"
        );
    }

    /// TC-T6: sps_constants matches AGC source values.
    #[test]
    fn sps_constants_agc_values() {
        let (thrust, ve) = sps_constants();
        assert!((thrust - 91_188.544).abs() < 0.001, "thrust = {thrust}");
        assert!((ve - 3_151.039_6).abs() < 0.001, "ve = {ve}");
    }
}
