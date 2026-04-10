//! JSON fixture loading for navigation accuracy tests.
//!
//! Each public function loads one of the committed fixture files from the
//! `agc-test/fixtures/` directory and deserialises it into a typed struct.
//!
//! The fixture files are read at test time using `include_str!` so that the
//! paths are resolved relative to the source tree root at compile time — no
//! runtime `std::fs` path resolution is needed, and tests remain runnable
//! regardless of the working directory from which `cargo test` is invoked.

use serde::Deserialize;

// ── Shared sub-types ──────────────────────────────────────────────────────────

/// A 3-component vector deserialized from a JSON `[x, y, z]` array.
pub type Vec3Json = [f64; 3];

/// A 3×3 matrix deserialized from a JSON `[[r0c0..], [r1c0..], [r2c0..]]` array.
pub type Mat3x3Json = [[f64; 3]; 3];

// ── GravityCase ───────────────────────────────────────────────────────────────

/// One gravity test vector from `gravity_cases.json`.
///
/// Each case specifies a spacecraft position, the gravitating body, and the
/// analytically computed expected gravitational acceleration, together with an
/// acceptance tolerance.
#[derive(Debug, Deserialize)]
pub struct GravityCase {
    /// Human-readable identifier for the test case.
    pub name: String,

    /// Explanation of how the expected value was computed.
    pub description: String,

    /// Spacecraft position in metres, expressed in `frame`.
    pub position_m: Vec3Json,

    /// Coordinate frame: `"ECI"` (Earth-Centred Inertial) or `"MCI"` (Moon-Centred Inertial).
    pub frame: String,

    /// Gravitating body: `"earth"` or `"moon"`.
    pub body: String,

    /// Expected gravitational acceleration in m/s².
    pub expected_accel_m_s2: Vec3Json,

    /// Per-component acceptance tolerance in m/s².
    pub tolerance_m_s2: f64,
}

/// Deserialize all gravity test cases from `agc-test/fixtures/gravity_cases.json`.
pub fn load_gravity_cases() -> Vec<GravityCase> {
    let json = include_str!("../fixtures/gravity_cases.json");
    serde_json::from_str(json)
        .expect("Failed to parse gravity_cases.json — check fixture syntax")
}

// ── ServicerCase ──────────────────────────────────────────────────────────────

/// State vector sub-structure used in `ServicerCase` and `OrbitCase`.
#[derive(Debug, Deserialize)]
pub struct StateVectorJson {
    /// Position in metres `[x, y, z]`.
    pub position_m: Vec3Json,

    /// Velocity in m/s `[vx, vy, vz]`.
    pub velocity_m_s: Vec3Json,

    /// Mission elapsed time in centiseconds.
    pub epoch_cs: u64,

    /// Coordinate frame: `"EarthInertial"` or `"MoonInertial"`.
    pub frame: String,
}

/// PIPA calibration constants for a servicer case.
#[derive(Debug, Deserialize)]
pub struct PipaCalJson {
    /// Scale factor in m/s per raw PIPA count.
    pub scale: f64,

    /// Bias in counts per 2-second cycle `[bx, by, bz]`.
    pub bias: [i16; 3],

    /// 3×3 misalignment matrix (identity for nominal calibration).
    pub misalignment: Mat3x3Json,
}

/// One SERVICER integration test case from `servicer_cycle_cases.json`.
#[derive(Debug, Deserialize)]
pub struct ServicerCase {
    /// Human-readable identifier.
    pub name: String,

    /// Description of the scenario and how expected values were derived.
    pub description: String,

    /// Initial state vector before the first SERVICER cycle.
    pub initial_state: StateVectorJson,

    /// Sequence of raw PIPA counts injected at each cycle: `[[px, py, pz], ...]`.
    pub pipa_sequence: Vec<[i16; 3]>,

    /// PIPA calibration to use for all cycles.
    pub pipa_cal: PipaCalJson,

    /// REFSMMAT (Reference-to-Stable-Member matrix) applied to PIPA delta-V.
    pub refsmmat: Mat3x3Json,

    /// Moon ECI position used in the integration (m).
    pub moon_pos_m: Vec3Json,

    /// Expected state vector after all PIPA cycles complete.
    pub expected_final_state: StateVectorJson,

    /// Acceptance tolerance for each position component (m).
    pub position_tolerance_m: f64,

    /// Acceptance tolerance for each velocity component (m/s).
    pub velocity_tolerance_m_s: f64,

    /// Number of SERVICER cycles to run (must equal `pipa_sequence.len()`).
    pub num_cycles: usize,

    /// Optional human-readable note on the tolerance rationale.
    pub note: String,
}

/// Deserialize all SERVICER test cases from `agc-test/fixtures/servicer_cycle_cases.json`.
pub fn load_servicer_cases() -> Vec<ServicerCase> {
    let json = include_str!("../fixtures/servicer_cycle_cases.json");
    serde_json::from_str(json)
        .expect("Failed to parse servicer_cycle_cases.json — check fixture syntax")
}

// ── OrbitCase ─────────────────────────────────────────────────────────────────

/// One orbit propagation test case from `orbit_propagation_cases.json`.
#[derive(Debug, Deserialize)]
pub struct OrbitCase {
    /// Human-readable identifier.
    pub name: String,

    /// Description of the scenario and how expected values were derived.
    pub description: String,

    /// Initial state vector.
    pub initial_state: StateVectorJson,

    /// Propagation interval in seconds.
    pub dt_s: f64,

    /// Moon ECI position at the start of propagation (m).
    pub moon_pos_m: Vec3Json,

    /// Expected state vector after `dt_s` seconds.
    pub expected_state: StateVectorJson,

    /// Acceptance tolerance for each position component (m).
    pub position_tolerance_m: f64,

    /// Acceptance tolerance for each velocity component (m/s).
    pub velocity_tolerance_m_s: f64,

    /// Optional human-readable note on the tolerance rationale.
    pub note: String,
}

/// Deserialize all orbit propagation test cases from `agc-test/fixtures/orbit_propagation_cases.json`.
pub fn load_orbit_cases() -> Vec<OrbitCase> {
    let json = include_str!("../fixtures/orbit_propagation_cases.json");
    serde_json::from_str(json)
        .expect("Failed to parse orbit_propagation_cases.json — check fixture syntax")
}

// ── LambertCase ──────────────────────────────────────────────────────────────

/// One Lambert solver test case from `lambert_cases.json`.
#[derive(Debug, Deserialize)]
pub struct LambertCase {
    /// Human-readable identifier.
    pub name: String,

    /// Explanation of how the expected values were computed.
    pub description: String,

    /// Initial position vector in metres `[x, y, z]`.
    pub r1_m: Vec3Json,

    /// Final position vector in metres `[x, y, z]`.
    pub r2_m: Vec3Json,

    /// Transfer time of flight in seconds.
    pub tof_s: f64,

    /// Gravitational parameter of the central body (m³/s²).
    pub mu_m3_s2: f64,

    /// Transfer direction: `true` = prograde, `false` = retrograde.
    pub prograde: bool,

    /// Expected departure velocity at `r1` (m/s).
    pub expected_v1_m_s: Vec3Json,

    /// Expected arrival velocity at `r2` (m/s).
    pub expected_v2_m_s: Vec3Json,

    /// Per-component acceptance tolerance for velocities (m/s).
    pub velocity_tolerance_m_s: f64,

    /// Optional human-readable note on the tolerance rationale.
    pub note: String,
}

/// Deserialize all Lambert test cases from `agc-test/fixtures/lambert_cases.json`.
pub fn load_lambert_cases() -> Vec<LambertCase> {
    let json = include_str!("../fixtures/lambert_cases.json");
    serde_json::from_str(json)
        .expect("Failed to parse lambert_cases.json — check fixture syntax")
}

// ── KeplerCase ────────────────────────────────────────────────────────────────

/// One Kepler propagation test case from `kepler_cases.json`.
#[derive(Debug, Deserialize)]
pub struct KeplerCase {
    /// Human-readable identifier.
    pub name: String,

    /// Explanation of how the expected values were computed.
    pub description: String,

    /// Initial position vector in metres `[x, y, z]`.
    pub initial_position_m: Vec3Json,

    /// Initial velocity vector in m/s `[vx, vy, vz]`.
    pub initial_velocity_m_s: Vec3Json,

    /// Propagation interval in seconds.
    pub dt_s: f64,

    /// Gravitational parameter of the central body (m³/s²).
    pub mu_m3_s2: f64,

    /// Expected position vector at `t0 + dt_s` (m).
    pub expected_position_m: Vec3Json,

    /// Expected velocity vector at `t0 + dt_s` (m/s).
    pub expected_velocity_m_s: Vec3Json,

    /// Per-component acceptance tolerance for position (m).
    pub position_tolerance_m: f64,

    /// Per-component acceptance tolerance for velocity (m/s).
    pub velocity_tolerance_m_s: f64,

    /// Optional human-readable note on the tolerance rationale.
    pub note: String,
}

/// Deserialize all Kepler propagation test cases from `agc-test/fixtures/kepler_cases.json`.
pub fn load_kepler_cases() -> Vec<KeplerCase> {
    let json = include_str!("../fixtures/kepler_cases.json");
    serde_json::from_str(json)
        .expect("Failed to parse kepler_cases.json — check fixture syntax")
}

// ── KalmanCase ────────────────────────────────────────────────────────────────

/// One scalar Kalman update test case from `kalman_cases.json`.
#[derive(Debug, Deserialize)]
pub struct KalmanCase {
    /// Human-readable identifier.
    pub name: String,

    /// Explanation of how the expected values were computed.
    pub description: String,

    /// Initial 6-element state vector before the update.
    pub initial_x: [f64; 6],

    /// Initial 6×6 covariance matrix before the update.
    pub initial_w: [[f64; 6]; 6],

    /// Measurement sensitivity row (Jacobian H row).
    pub b: [f64; 6],

    /// Scalar measurement residual `z_observed - z_predicted`.
    pub residual: f64,

    /// Measurement noise variance for this mark type.
    pub sigma_sq: f64,

    /// Expected `UpdateOutcome` as a string: `"Accepted"`, `"Rejected"`, or `"AcceptedWOverflow"`.
    pub expected_outcome: String,

    /// Expected 6-element state vector after the update.
    pub expected_x_after: [f64; 6],

    /// Expected 6×6 covariance matrix after the update.
    pub expected_w_after: [[f64; 6]; 6],

    /// Per-element acceptance tolerance for state components.
    pub state_tolerance: f64,

    /// Per-element acceptance tolerance for covariance components.
    pub covariance_tolerance: f64,

    /// Optional human-readable note on the tolerance rationale.
    pub note: String,
}

/// Deserialize all scalar Kalman update test cases from `agc-test/fixtures/kalman_cases.json`.
pub fn load_kalman_cases() -> Vec<KalmanCase> {
    let json = include_str!("../fixtures/kalman_cases.json");
    serde_json::from_str(json)
        .expect("Failed to parse kalman_cases.json — check fixture syntax")
}

// ── Smoke tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify that gravity_cases.json parses without error and contains
    /// at least one case with the expected field structure.
    #[test]
    fn load_gravity_cases_parses() {
        let cases = load_gravity_cases();
        assert!(!cases.is_empty(), "gravity_cases.json must not be empty");
        for c in &cases {
            assert!(!c.name.is_empty(), "Every gravity case must have a name");
            assert!(
                c.frame == "ECI" || c.frame == "MCI",
                "Frame must be ECI or MCI, got: {}",
                c.frame
            );
            assert!(
                c.body == "earth" || c.body == "moon",
                "Body must be earth or moon, got: {}",
                c.body
            );
            assert!(c.tolerance_m_s2 > 0.0, "Tolerance must be positive");
        }
    }

    /// Verify that servicer_cycle_cases.json parses without error.
    #[test]
    fn load_servicer_cases_parses() {
        let cases = load_servicer_cases();
        assert!(!cases.is_empty(), "servicer_cycle_cases.json must not be empty");
        for c in &cases {
            assert!(!c.name.is_empty(), "Every servicer case must have a name");
            assert!(
                c.pipa_sequence.len() == c.num_cycles,
                "pipa_sequence.len() ({}) must equal num_cycles ({}) in case '{}'",
                c.pipa_sequence.len(), c.num_cycles, c.name
            );
            assert!(c.position_tolerance_m > 0.0, "Position tolerance must be positive");
            assert!(c.velocity_tolerance_m_s > 0.0, "Velocity tolerance must be positive");
        }
    }

    /// Verify that orbit_propagation_cases.json parses without error.
    #[test]
    fn load_orbit_cases_parses() {
        let cases = load_orbit_cases();
        assert!(!cases.is_empty(), "orbit_propagation_cases.json must not be empty");
        for c in &cases {
            assert!(!c.name.is_empty(), "Every orbit case must have a name");
            assert!(c.dt_s > 0.0, "dt_s must be positive");
            assert!(c.position_tolerance_m > 0.0, "Position tolerance must be positive");
            assert!(c.velocity_tolerance_m_s > 0.0, "Velocity tolerance must be positive");
        }
    }

    /// Verify that lambert_cases.json parses without error and contains
    /// at least one case with the expected field structure.
    #[test]
    fn load_lambert_cases_parses() {
        let cases = load_lambert_cases();
        assert!(!cases.is_empty(), "lambert_cases.json must not be empty");
        for c in &cases {
            assert!(!c.name.is_empty(), "Every Lambert case must have a name");
            assert!(c.tof_s > 0.0, "tof_s must be positive");
            assert!(c.mu_m3_s2 > 0.0, "mu_m3_s2 must be positive");
            assert!(c.velocity_tolerance_m_s > 0.0, "velocity_tolerance_m_s must be positive");
        }
    }

    /// Verify that kepler_cases.json parses without error and contains
    /// at least one case with the expected field structure.
    #[test]
    fn load_kepler_cases_parses() {
        let cases = load_kepler_cases();
        assert!(!cases.is_empty(), "kepler_cases.json must not be empty");
        for c in &cases {
            assert!(!c.name.is_empty(), "Every Kepler case must have a name");
            assert!(c.dt_s > 0.0, "dt_s must be positive");
            assert!(c.mu_m3_s2 > 0.0, "mu_m3_s2 must be positive");
            assert!(c.position_tolerance_m > 0.0, "position_tolerance_m must be positive");
            assert!(c.velocity_tolerance_m_s > 0.0, "velocity_tolerance_m_s must be positive");
        }
    }

    /// Verify that kalman_cases.json parses without error and contains
    /// at least one case with the expected field structure.
    #[test]
    fn load_kalman_cases_parses() {
        let cases = load_kalman_cases();
        assert!(!cases.is_empty(), "kalman_cases.json must not be empty");
        for c in &cases {
            assert!(!c.name.is_empty(), "Every Kalman case must have a name");
            assert!(
                c.expected_outcome == "Accepted"
                    || c.expected_outcome == "Rejected"
                    || c.expected_outcome == "AcceptedWOverflow",
                "expected_outcome must be Accepted, Rejected, or AcceptedWOverflow; got: {}",
                c.expected_outcome
            );
        }
    }
}
