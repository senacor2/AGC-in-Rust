//! P21 — Ground-Track Determination.
//!
//! Display-only program. Given a target GET, propagates the CSM state vector
//! forward (or backward) to that time using `math::kepler::kepler_step`, then
//! computes the sub-satellite point — the geographic point on the Earth's surface
//! directly below the CSM at that instant. The result (geocentric latitude,
//! longitude, and altitude above the spherical Earth reference) is written to
//! the DSKY via Verb 06 Noun 43.
//!
//! P21 makes no measurements, performs no state updates, and does not reschedule
//! itself. It is a one-shot computation triggered by a crew request.
//!
//! Mission context: P21 is useful for predicting ground-station contact windows,
//! verifying the insertion orbit, or planning landmark-tracking sessions.
//!
//! AGC source: Comanche055/P20-P25.agc (P21 entry sequence),
//!             Comanche055/R60,R62.agc (ground-track subroutines),
//!             Comanche055/LAT-LONG_SUBROUTINES.agc
//! Spec: specs/p21_p22-spec.md §1.1, §4.1, §6.1

use crate::executive::job::JobPriority;
use crate::math::kepler::kepler_step;
use crate::math::linalg::norm;
use crate::navigation::gravity::MU_EARTH;
use crate::navigation::state_vector::inertial_to_earth_fixed;
use crate::navigation::time::OMEGA_EARTH;
use crate::types::Vec3;
use crate::AgcState;

// ── Constants ──────────────────────────────────────────────────────────────────

/// Major mode number for P21.
/// Spec: p21_p22-spec.md §4.1
pub const P21_MAJOR_MODE: u8 = 21;

/// Job priority for P21.
/// One level below P22 (periodic) because P21 is one-shot and non-time-critical.
/// Spec: p21_p22-spec.md §4.1
pub const P21_PRIORITY: JobPriority = 7;

/// Mean Earth radius used by P21 (spherical approximation, m).
/// Source: IAU 2012, WGS84 mean equatorial radius rounded to 1 m.
/// Note: `navigation::gravity::R_EARTH` is the WGS84 equatorial (semi-major) radius
/// (6_378_137 m) used for J2 computations. P21 uses the mean spherical radius
/// per the AGC convention (O'Brien p. 303).
/// Spec: p21_p22-spec.md §4.1
pub const R_EARTH: f64 = 6_371_000.0; // m

// ── Alarm codes ─────────────────────────────────────────────────────────────────

/// Alarm 01420 (octal): no valid CSM state vector (epoch == 0).
const ALARM_NO_CSM_SV: u16 = 0o01420;

// ── Result type ────────────────────────────────────────────────────────────────

/// Result of a P21 ground-track computation.
///
/// Spec: p21_p22-spec.md §4.1
#[derive(Clone, Copy, Debug)]
pub struct GroundTrackResult {
    /// Geocentric latitude at the target GET (rad). Range `[-π/2, +π/2]`.
    /// Positive north.
    pub lat_rad: f64,
    /// Longitude at the target GET (rad). Range `(-π, +π]`.
    /// Positive east, measured from the IERS reference meridian.
    pub lon_rad: f64,
    /// Altitude above the spherical Earth reference (m).
    /// Reference sphere radius: `R_EARTH`.
    pub alt_m: f64,
}

// ── Entry point ────────────────────────────────────────────────────────────────

/// Entry point for P21 (Ground-Track Determination).
/// Registered in `PROGRAM_TABLE[21]`.
///
/// Sets `state.major_mode = 21`. Computes the current sub-satellite point using
/// `state.csm_state` and `state.time` as both the epoch and target GET (i.e.,
/// the current ground track), then displays the result via V06 N43.
///
/// # Phase 3 simplification
/// Full crew-driven GET entry (Noun 34 prompt / wait) is deferred until the
/// V/N data-entry layer for Noun 34 is wired up. In Phase 3, `p21_init` performs
/// the ground-track computation for the **current** time (`target_get_s = state.time`)
/// and displays it. Crew-initiated future-GET queries should call
/// `p21_compute_ground_track` directly with the desired GET.
///
/// # Preconditions
/// - `state.csm_state.epoch` must be non-zero; otherwise alarm 01420 is raised
///   and the program returns without a computation.
/// - `state.gha_epoch_rad` must have been set by uplink or crew entry.
///
/// # Post-conditions
/// - `state.major_mode == 21`
/// - `state.dsky.prog == 21`
/// - DSKY registers set to lat/lon/alt at the current GET (if no alarm).
/// - No periodic Waitlist hook installed.
///
/// Spec: p21_p22-spec.md §4.1
pub fn p21_init(state: &mut AgcState) -> JobPriority {
    state.major_mode = P21_MAJOR_MODE;
    state.dsky.prog = P21_MAJOR_MODE;

    // Precondition: non-zero CSM epoch.
    if state.csm_state.epoch.to_seconds() == 0.0 {
        state.alarm.code = ALARM_NO_CSM_SV;
        state.alarm.lit = true;
        state.dsky.verb = 6;
        state.dsky.noun = 43;
        state.dsky.r[0] = 0.0;
        state.dsky.r[1] = 0.0;
        state.dsky.r[2] = 0.0;
        return P21_PRIORITY;
    }

    let epoch_s = state.csm_state.epoch.to_seconds();
    let target_get_s = state.time.to_seconds();
    let csm_pos = state.csm_state.position;
    let csm_vel = state.csm_state.velocity;
    let gha_epoch = state.gha_epoch_rad;

    let result = p21_compute_ground_track(csm_pos, csm_vel, epoch_s, target_get_s, gha_epoch);

    // Display via V06 N43.
    // R1: latitude  (degrees × 100, cast to f32)
    // R2: longitude (degrees × 100, cast to f32)
    // R3: altitude  (km × 10, cast to f32)
    const RAD_TO_DEG: f64 = 180.0 / core::f64::consts::PI;
    state.dsky.verb = 6;
    state.dsky.noun = 43;
    state.dsky.r[0] = (result.lat_rad * RAD_TO_DEG * 100.0) as f32;
    state.dsky.r[1] = (result.lon_rad * RAD_TO_DEG * 100.0) as f32;
    state.dsky.r[2] = (result.alt_m / 100.0) as f32; // km × 10
    state.dsky.flashing = false;

    P21_PRIORITY
}

// ── Core computation ───────────────────────────────────────────────────────────

/// Compute the sub-satellite point for the CSM at the given target GET.
///
/// This is the pure-computation core of P21. It is separated from `p21_init`
/// so that unit tests can exercise it directly without a full `AgcState`.
///
/// # Arguments
/// - `csm_pos`: CSM inertial position at the known epoch (m, ECI).
/// - `csm_vel`: CSM inertial velocity at the known epoch (m/s, ECI).
/// - `epoch_s`: GET of the known epoch (s).
/// - `target_get_s`: GET at which the sub-satellite point is requested (s).
/// - `gha_epoch_rad`: Greenwich Hour Angle at GET = 0 (rad).
///
/// # Returns
/// `GroundTrackResult` containing geocentric latitude, longitude, and altitude.
///
/// # Panics
/// Panics if `norm(csm_pos) == 0` (CSM at Earth centre — physically impossible).
///
/// Spec: p21_p22-spec.md §6.1
pub fn p21_compute_ground_track(
    csm_pos: Vec3,
    csm_vel: Vec3,
    epoch_s: f64,
    target_get_s: f64,
    gha_epoch_rad: f64,
) -> GroundTrackResult {
    // Step 1 — Propagate CSM state to target GET.
    // delta_t may be negative (backward propagation is valid).
    // When delta_t == 0, skip kepler_step (kepler_step panics on zero dt).
    let delta_t = target_get_s - epoch_s;
    let (pos_t, _vel_t) = if delta_t == 0.0 {
        (csm_pos, csm_vel)
    } else {
        kepler_step(csm_pos, csm_vel, delta_t, MU_EARTH)
    };

    // Step 2 — Compute GHA at target GET. The transform takes the angle in
    // radians and uses cos/sin internally, so no modulo-2π reduction needed.
    let gha = gha_epoch_rad + OMEGA_EARTH * target_get_s;

    // Step 3 — Rotate inertial position to Earth-fixed frame (Rz(+gha)).
    let pos_ef: Vec3 = inertial_to_earth_fixed(pos_t, gha);

    // Step 4 — Extract geocentric latitude, longitude, and altitude.
    let r_mag = norm(pos_ef);
    // norm == 0 is physically impossible in orbit; panic is appropriate.
    assert!(
        r_mag > 0.0,
        "P21: CSM position magnitude is zero — physically impossible"
    );

    let lat = libm::asin(pos_ef[2] / r_mag);
    let lon = libm::atan2(pos_ef[1], pos_ef[0]);
    let alt = r_mag - R_EARTH;

    // Step 5 — Pack result.
    GroundTrackResult {
        lat_rad: lat,
        lon_rad: lon,
        alt_m: alt,
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::time::OMEGA_EARTH;

    // ── TC-P21-1: Circular LEO — same-epoch query returns current position ────

    /// TC-P21-1: Verify that `target_get_s == epoch_s` skips propagation and
    /// correctly converts the ECI position to Earth-fixed lat/lon/alt.
    #[test]
    fn tc_p21_1_same_epoch_no_propagation() {
        // Input: equatorial orbit at 300 km altitude, Greenwich meridian on ECI X at GET=0.
        let csm_pos: Vec3 = [6_671_000.0, 0.0, 0.0];
        let csm_vel: Vec3 = [0.0, 7726.0, 0.0];
        let epoch_s = 1000.0_f64;
        let target_get_s = 1000.0_f64; // same epoch — no propagation
        let gha_epoch_rad = 0.0_f64;

        let result =
            p21_compute_ground_track(csm_pos, csm_vel, epoch_s, target_get_s, gha_epoch_rad);

        // GHA at GET=1000 s = OMEGA_EARTH * 1000 ≈ 0.07292 rad.
        // The Earth has rotated east by that amount, so the sub-satellite longitude
        // is -OMEGA_EARTH * 1000 rad (the point is now west of Greenwich).
        let expected_lon = -(OMEGA_EARTH * 1000.0);

        // lat ≈ 0 (equatorial orbit)
        assert!(
            libm::fabs(result.lat_rad) < 1e-6,
            "lat_rad should be ~0 for equatorial orbit; got {}",
            result.lat_rad
        );

        // alt ≈ 300 km (r_mag - R_EARTH)
        assert!(
            libm::fabs(result.alt_m - 300_000.0) < 10.0,
            "alt_m should be ~300_000 m; got {}",
            result.alt_m
        );

        // Longitude equals minus the Earth rotation angle over 1000 s.
        assert!(
            libm::fabs(result.lon_rad - expected_lon) < 1e-4,
            "lon_rad should equal -(OMEGA_EARTH*1000) = {}; got {}",
            expected_lon,
            result.lon_rad
        );
    }

    // ── TC-P21-2: Quarter-orbit propagation — equatorial orbit ───────────────

    /// TC-P21-2: Verify that a quarter-orbit propagation correctly advances
    /// the inertial position by 90° and yields the expected Earth-fixed lat/lon.
    #[test]
    fn tc_p21_2_quarter_orbit_propagation() {
        // Circular equatorial LEO at 300 km altitude. Compute v_circ exactly
        // from mu/r rather than using a rounded literal; 7726 m/s gives a
        // slightly elliptical orbit and the altitude drops ~7 km over T/4.
        let r = 6_671_000.0_f64;
        let mu = crate::navigation::gravity::MU_EARTH;
        let v_circ = libm::sqrt(mu / r);
        let csm_pos: Vec3 = [r, 0.0, 0.0];
        let csm_vel: Vec3 = [0.0, v_circ, 0.0];
        let epoch_s = 0.0_f64;
        let gha_epoch_rad = 0.0_f64;

        // Compute quarter period: T = 2π * sqrt(r³/μ).
        let period = 2.0 * core::f64::consts::PI * libm::sqrt(r * r * r / mu);
        let t_quarter = period / 4.0;

        let result = p21_compute_ground_track(csm_pos, csm_vel, epoch_s, t_quarter, gha_epoch_rad);

        // After a quarter orbit the spacecraft is at approximately +Y ECI.
        // Equatorial orbit → lat should remain near 0.
        assert!(
            libm::fabs(result.lat_rad) < 0.001,
            "lat_rad should be ~0 for equatorial orbit; got {}",
            result.lat_rad
        );

        // Altitude should remain ~300 km (circular orbit).
        assert!(
            libm::fabs(result.alt_m - 300_000.0) < 1000.0,
            "alt_m should be ~300_000 m after quarter orbit; got {}",
            result.alt_m
        );

        // Longitude: after a quarter orbit (≈90° in ECI) minus Earth rotation (~5.67°),
        // we expect lon ≈ π/2 - OMEGA_EARTH * t_quarter ≈ 1.471 rad.
        let expected_lon = core::f64::consts::PI / 2.0 - OMEGA_EARTH * t_quarter;
        assert!(
            libm::fabs(result.lon_rad - expected_lon) < 0.01,
            "lon_rad should be ~{} rad; got {}",
            expected_lon,
            result.lon_rad
        );
    }

    // ── TC-P21-3: High-inclination orbit — non-zero latitude ─────────────────

    /// TC-P21-3: Verify that a high-inclination (ISS-like 51.6°) orbit at the
    /// northernmost point produces the correct sub-satellite latitude.
    #[test]
    fn tc_p21_3_high_inclination_latitude() {
        // ISS-like orbit: 400 km altitude, inclination 51.6°.
        let r = 6_771_000.0_f64;
        let inc_rad = 51.6_f64 * core::f64::consts::PI / 180.0; // 0.9006 rad

        // At the northernmost point of the ground track:
        // csm_pos[2] = r * sin(inc), and the spacecraft is at the top of the orbital plane.
        let csm_pos_z = r * libm::sin(inc_rad); // ≈ 5_299_000 m
                                                // X component chosen so that norm(csm_pos) = r.
        let csm_pos_x = libm::sqrt(r * r - csm_pos_z * csm_pos_z);
        let csm_pos: Vec3 = [csm_pos_x, 0.0, csm_pos_z];

        // Use a dummy velocity (no propagation since target_get_s == epoch_s).
        let csm_vel: Vec3 = [0.0, 5000.0, 0.0];
        let epoch_s = 0.0_f64;
        let target_get_s = 0.0_f64; // same epoch — no propagation
        let gha_epoch_rad = 0.0_f64;

        let result =
            p21_compute_ground_track(csm_pos, csm_vel, epoch_s, target_get_s, gha_epoch_rad);

        // Expected latitude: asin(csm_pos_z / r) = asin(sin(inc)) = inc.
        let expected_lat = libm::asin(csm_pos_z / r);
        assert!(
            libm::fabs(result.lat_rad - expected_lat) < 0.01,
            "lat_rad should be ≈ asin({}/{}) = {}; got {}",
            csm_pos_z,
            r,
            expected_lat,
            result.lat_rad
        );
    }
}
