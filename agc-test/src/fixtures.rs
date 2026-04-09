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
}
