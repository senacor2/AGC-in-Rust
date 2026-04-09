//! Maneuver targeting: compute required delta-V and time of ignition.
//!
//! AGC source: P30-P37.agc — P30 External Delta-V.

use crate::math::linalg::{norm, sub};
use crate::navigation::state_vector::StateVector;
use crate::types::Vec3;

/// SPS engine thrust (Newtons).
///
/// AGC source: P40-P47.agc — `FENG 2DEC 9.1188544 B-7 # SPS THRUST (20500LBS)`.
/// 20,500 lbf = 91,188.544 N.
pub const SPS_THRUST_N: f64 = 91_188.544;

/// SPS exhaust velocity (m/s), derived from AGC `2VEXHUST`.
///
/// AGC source: P40-P47.agc — `2VEXHUST 2DEC 63.020792 B-7`.
/// Exhaust velocity = 63.020792 / 2 m/cs × 100 cs/s = 3151.0396 m/s.
/// Isp = Ve / g0 = 3151.04 / 9.80665 ≈ 321.3 s.
pub const SPS_VE_MS: f64 = 3_151.0396;

/// SPS specific impulse (seconds), derived from `2VEXHUST`.
///
/// AGC source: P40-P47.agc — `2VEXHUST 2DEC 63.020792 B-7`.
/// Isp = Ve / g0 = 3151.04 / 9.80665 ≈ 321.3 s.
pub const SPS_ISP_S: f64 = 321.3;

/// Standard gravity for Isp conversion (m/s²).
pub const G0: f64 = 9.80665;

/// A planned maneuver: when to burn and how much.
#[derive(Clone, Copy, Debug)]
pub struct ManeuverPlan {
    /// Time of ignition (MET centiseconds).
    pub tig_cs: u32,
    /// Required delta-V vector in ECI frame (m/s).
    pub delta_v_eci: Vec3,
    /// Delta-V magnitude (m/s).
    pub delta_v_mag: f64,
    /// Estimated burn time (seconds), based on thrust/mass.
    pub burn_time_s: f64,
}

/// Compute delta-V required to change from current orbit to target orbit.
///
/// Given the current state vector and a target velocity at the same time,
/// returns the velocity change needed: `v_target − v_current`.
///
/// AGC source: P30-P37.agc — S40.1 (VGTIG computation).
pub fn compute_delta_v(sv_current: &StateVector, v_target: &Vec3) -> Vec3 {
    sub(v_target, &sv_current.v)
}

/// Estimate burn time from delta-V magnitude, vehicle mass, and thrust.
///
/// Uses the simplified constant-thrust approximation: `dt ≈ m * |dv| / F`.
/// The full Tsiolkovsky rocket equation is:
///   `dt = (m * Isp * g0 / F) * (1 − exp(−|dv| / (Isp * g0)))`
/// For the typical SPS burn (|dv| << Isp*g0), both forms yield similar results.
///
/// Returns 0.0 if thrust_n is zero to avoid division by zero.
///
/// AGC source: P40-P47.agc — S40.8 burn time computation.
pub fn estimate_burn_time(delta_v_mag: f64, mass_kg: f64, thrust_n: f64) -> f64 {
    if thrust_n <= 0.0 {
        return 0.0;
    }
    let effective_exhaust_velocity = SPS_VE_MS;
    // Full Tsiolkovsky form for accuracy
    let exponent = -delta_v_mag / effective_exhaust_velocity;
    (mass_kg * effective_exhaust_velocity / thrust_n) * (1.0 - libm::exp(exponent))
}

/// Create a maneuver plan from current state, target velocity, and vehicle mass.
///
/// AGC source: P30-P37.agc — P30 External Delta-V, S40.1 and S40.8.
pub fn plan_maneuver(
    sv: &StateVector,
    v_target: &Vec3,
    mass_kg: f64,
    thrust_n: f64,
) -> ManeuverPlan {
    let delta_v_eci = compute_delta_v(sv, v_target);
    let delta_v_mag = norm(&delta_v_eci);
    let burn_time_s = estimate_burn_time(delta_v_mag, mass_kg, thrust_n);
    ManeuverPlan {
        tig_cs: sv.t.0,
        delta_v_eci,
        delta_v_mag,
        burn_time_s,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;

    fn make_sv(v: Vec3) -> StateVector {
        StateVector {
            frame: Frame::Eci,
            r: [0.0; 3],
            v,
            t: Met(100),
        }
    }

    #[test]
    fn compute_delta_v_correct_vector() {
        let sv = make_sv([7500.0, 0.0, 0.0]);
        let v_target = [7600.0, 50.0, 0.0];
        let dv = compute_delta_v(&sv, &v_target);
        assert!((dv[0] - 100.0).abs() < 1e-10, "dv[0]={}", dv[0]);
        assert!((dv[1] - 50.0).abs() < 1e-10, "dv[1]={}", dv[1]);
        assert!(dv[2].abs() < 1e-10, "dv[2]={}", dv[2]);
    }

    #[test]
    fn estimate_burn_time_sps_100ms() {
        // 100 m/s burn, 20_000 kg, SPS thrust
        let dt = estimate_burn_time(100.0, 20_000.0, SPS_THRUST_N);
        // Simple sanity: dt ≈ m * dv / F = 20000 * 100 / 91188 ≈ 21.9 s
        assert!(dt > 20.0 && dt < 25.0, "burn_time={}", dt);
    }

    #[test]
    fn plan_maneuver_fields() {
        let sv = make_sv([7500.0, 0.0, 0.0]);
        let v_target = [7600.0, 0.0, 0.0];
        let plan = plan_maneuver(&sv, &v_target, 20_000.0, SPS_THRUST_N);
        assert_eq!(plan.tig_cs, 100);
        assert!((plan.delta_v_mag - 100.0).abs() < 1e-10);
        assert!((plan.delta_v_eci[0] - 100.0).abs() < 1e-10);
        assert!(plan.burn_time_s > 0.0);
    }

    #[test]
    fn estimate_burn_time_zero_thrust_returns_zero() {
        let dt = estimate_burn_time(100.0, 20_000.0, 0.0);
        assert_eq!(dt, 0.0);
    }
}
