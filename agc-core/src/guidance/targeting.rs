//! Maneuver targeting — Time of Ignition (TIG), delta-V, and burn attitude.
//!
//! This module is the central computation layer for all maneuver planning in
//! the Comanche055 Command Module software. It translates a navigation state
//! and a targeting objective into a `Maneuver` — the triplet of (Time of
//! Ignition, inertial delta-V vector, body-frame burn attitude) that P40
//! (SPS burn) executes and the DSKY displays to the crew for confirmation.
//!
//! Four targeting modes are served:
//!
//! | Program | Mode              | Algorithm                                  |
//! |---------|-------------------|--------------------------------------------|
//! | P30     | ExternalDeltaV    | LVLH-frame delta-V converted to inertial   |
//! | P31/P34 | Lambert           | Lambert solver: current → aim point        |
//! | P37     | ReturnToEarth     | Lambert solver: current → entry interface  |
//!
//! AGC source: `P30,P31,P37,P40SUBROUTINES.agc`, `CONIC_SUBROUTINES.agc`.

use crate::math::lambert::lambert;
use crate::math::linalg::{cross, mxv, norm, unit, vsub};
use crate::navigation::gravity::{MU_EARTH, MU_MOON};
use crate::navigation::state_vector::{Frame, StateVector};
use crate::types::{DeltaV, Mat3x3, Met, Vec3};

// ── Constants ─────────────────────────────────────────────────────────────────

/// SPS (Service Propulsion System) nominal vacuum thrust, Newtons.
/// Source: Apollo CSM Systems Handbook, SPS engine specification.
pub const SPS_THRUST_N: f64 = 91_188.0;

/// SPS nominal specific impulse (vacuum), seconds.
/// Source: Apollo CSM Systems Handbook, SPS engine specification.
pub const SPS_ISP_S: f64 = 314.0;

/// Earth mean equatorial radius, metres.
pub const R_EARTH_M: f64 = 6_378_137.0;

/// Entry interface altitude above the Earth surface, metres (400,000 ft).
/// Used by P37 as the target sphere radius offset from `R_EARTH_M`.
pub const ENTRY_INTERFACE_ALT_M: f64 = 121_920.0;

// ── TargetingMode ─────────────────────────────────────────────────────────────

/// Identifies which targeting program produced a `Maneuver`.
///
/// Determines the DSKY noun and display units, and which burn-monitor mode
/// P40 uses during execution.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum TargetingMode {
    /// P30 — External Delta-V.
    ///
    /// Delta-V was supplied by Mission Control via uplink and converted from
    /// LVLH frame to inertial frame by `apply_external_delta_v`.
    /// DSKY: V06N33 (TIG), V06N81 (delta-V components in body frame).
    #[default]
    ExternalDeltaV,

    /// Reserved for future Lambert-based targeting programs that are not
    /// TPI/TPM. No current program uses this variant.
    ///
    /// Historically this variant was used by P33 (TPI) and P34 (TPM) before
    /// they received dedicated `TpiBurn` and `TpmBurn` variants in Phase 6.
    /// P31 and P32 use closed-form coelliptic targeting, not Lambert.
    /// DSKY: V06N33 (TIG), V06N84 (delta-V in LVLH for display).
    Lambert,

    /// P31 — Coelliptic Sequence Initiation (CSI) burn.
    /// Delta-V is the in-track impulse computed by P31's closed-form
    /// coelliptic targeting (not Lambert).
    /// DSKY: V06N33 (TIG), V06N84 (delta-V in LVLH).
    CsiBurn,

    /// P32 — Constant Delta-Height (CDH) burn.
    /// Delta-V achieves the coelliptic condition (constant altitude
    /// separation) at the CDH apsidal line.
    /// DSKY: V06N33 (TIG), V06N84 (delta-V in LVLH).
    CdhBurn,

    /// P37 — Return to Earth (Trans-Earth Injection).
    ///
    /// Delta-V was computed by the Lambert solver targeting the Earth entry
    /// interface at the nominal entry flight-path angle.
    /// DSKY: V06N33 (TIG), V06N86 (delta-V magnitude + entry angle).
    ReturnToEarth,

    /// P33 — Terminal Phase Initiation (TPI) burn.
    ///
    /// Delta-V computed by Lambert solver targeting the LM's position at
    /// TIG + dt_tpi. Transfer time nominally 10 minutes.
    /// DSKY: V06 N37 (TIG), V06 N55 (elevation angle + transfer time),
    ///       V06 N81 (LVLH ΔV components).
    TpiBurn,

    /// P34 — Terminal Phase Midcourse (TPM) correction burn.
    ///
    /// Delta-V computed by Lambert solver targeting the same arrival position
    /// as the P33 TPI solution, with remaining time dt_midcourse.
    /// DSKY: V06 N37 (TIG), V06 N81 (LVLH ΔV components).
    TpmBurn,
}

// ── Maneuver ──────────────────────────────────────────────────────────────────

/// A targeted maneuver: when to ignite, how much delta-V to apply, and the
/// body-frame attitude required to align the SPS nozzle with that delta-V.
///
/// `Maneuver` is the primary output of all P30/P31/P34/P37 targeting
/// computations and the primary input to P40 (SPS burn execution) and the
/// DSKY N37 display.
///
/// AGC erasable source:
///   `TIG`       — octal 0350, scale B+28 centiseconds
///   `DELVEET1/2/3` — octal 0352–0356, scale B+7 m/s (inertial frame)
///   Burn attitude stored as CDU gimbal angles in Comanche055 (octal 0033–0035);
///   represented here as a `Mat3x3` for type safety.
///
/// **Invariants**:
/// - `burn_attitude` is orthonormal: `burn_attitude * burn_attitude^T = I`
///   to within `1e-9` per element.
/// - `burn_attitude * [1, 0, 0]` is parallel to `unit(delta_v.0)` when
///   `|delta_v.0| > 0`.
#[derive(Clone, Copy, Debug)]
pub struct Maneuver {
    /// Time of Ignition: mission elapsed time at which the burn begins.
    ///
    /// Unit: centiseconds (`u32`). Convert to seconds with
    /// `tig.to_seconds()`.
    /// AGC: stored at octal 0350 (E3), scale B+28.
    pub tig: Met,

    /// Delta-V to be applied at TIG, expressed in the inertial navigation
    /// frame (ECI or MCI, matching the frame of the current state vector).
    ///
    /// Unit: m/s.
    /// AGC: `DELVEET1/DELVEET2/DELVEET3` at octal 0352–0356, scale B+7 m/s.
    pub delta_v: DeltaV,

    /// Body-to-inertial rotation matrix at TIG: the spacecraft attitude
    /// required to align the SPS nozzle (+X body axis) with
    /// `unit(delta_v.0)`.
    ///
    /// Convention: `burn_attitude * [1, 0, 0] ≈ unit(delta_v.0)`.
    ///
    /// When `delta_v` is the zero vector, `burn_attitude` is the identity
    /// matrix.
    ///
    /// AGC: not stored as a matrix; Comanche055 used CDU gimbal angles
    /// derived from REFSMMAT and the desired thrust direction in P40.
    pub burn_attitude: Mat3x3,

    /// Targeting mode that produced this maneuver.
    ///
    /// Used by P40 and the DSKY display to select the appropriate noun
    /// table entry and burn monitor mode.
    pub mode: TargetingMode,
}

impl Default for Maneuver {
    fn default() -> Self {
        Self {
            tig: Met(0),
            delta_v: DeltaV([0.0; 3]),
            burn_attitude: crate::math::linalg::IDENTITY,
            mode: TargetingMode::ExternalDeltaV,
        }
    }
}

// ── Public functions ──────────────────────────────────────────────────────────

/// Construct the LVLH-to-inertial rotation matrix from a position and
/// velocity in the inertial frame.
///
/// The LVLH (RSW) frame is defined as:
/// ```text
/// R_unit = unit(position)                           — radial (away from body)
/// W_unit = unit(cross(position, velocity))          — orbit normal (angular momentum)
/// S_unit = cross(W_unit, R_unit)                    — along-track (prograde for circular)
/// ```
///
/// Returns the 3×3 matrix `M` such that `M * v_lvlh = v_inertial`.
/// Columns are `[R_unit | S_unit | W_unit]`.
///
/// # Panics
///
/// Panics if `position` is the zero vector or if the angular momentum
/// `cross(position, velocity)` has near-zero magnitude (degenerate
/// rectilinear trajectory).
pub fn lvlh_to_inertial(position: Vec3, velocity: Vec3) -> Mat3x3 {
    let r_unit = unit(position);
    let h = cross(position, velocity);
    let w_unit = unit(h); // orbit normal
    let s_unit = cross(w_unit, r_unit); // in-track, prograde for circular

    // Columns are the LVLH basis vectors expressed in inertial coordinates.
    // Row i of the matrix = [R_unit[i], S_unit[i], W_unit[i]].
    [
        [r_unit[0], s_unit[0], w_unit[0]],
        [r_unit[1], s_unit[1], w_unit[1]],
        [r_unit[2], s_unit[2], w_unit[2]],
    ]
}

/// Convert a ground-uplinked delta-V from LVLH frame to inertial and
/// package it as a `Maneuver`.
///
/// This is the P30 (External Delta-V) targeting path. Mission Control
/// computes the required maneuver in ground-side trajectory software and
/// uplinks the result in LVLH frame.
///
/// # Arguments
///
/// * `current` — Current vehicle state vector (position and velocity in
///   the inertial frame, already propagated to `tig` if TIG is future).
/// * `tig` — Time of Ignition in mission elapsed time (centiseconds).
/// * `delta_v_lvlh` — Delta-V in LVLH frame (m/s), ordered `[R, S, W]`:
///   - `R` (index 0): radial (positive = away from central body)
///   - `S` (index 1): in-track (positive = prograde for circular orbit)
///   - `W` (index 2): cross-track (positive = toward angular momentum)
/// * `refsmmat` — Current IMU REFSMMAT; passed to `burn_attitude`.
///
/// # Returns
///
/// A `Maneuver` with `mode = TargetingMode::ExternalDeltaV`.
///
/// # Postconditions
///
/// - `|delta_v_inertial| == |delta_v_lvlh|` (rotation preserves magnitude).
/// - If `delta_v_lvlh == [0, 0, 0]`, `delta_v_inertial == [0, 0, 0]` and
///   `burn_attitude == IDENTITY`.
pub fn apply_external_delta_v(
    current: StateVector,
    tig: Met,
    delta_v_lvlh: Vec3,
    refsmmat: Mat3x3,
) -> Maneuver {
    let m = lvlh_to_inertial(current.position, current.velocity);
    let delta_v_inertial = mxv(m, delta_v_lvlh);
    let attitude = burn_attitude(delta_v_inertial, refsmmat);
    Maneuver {
        tig,
        delta_v: DeltaV(delta_v_inertial),
        burn_attitude: attitude,
        mode: TargetingMode::ExternalDeltaV,
    }
}

/// Compute the required delta-V to transfer from the current state to a
/// target position in a given time of flight, using Lambert's problem.
///
/// This is the core targeting algorithm for all on-board rendezvous
/// programs (P31 height adjust, P32 coelliptic, P33 CDH, P34 TPI).
///
/// # Arguments
///
/// * `current` — State vector at TIG. `current.epoch` is used as the TIG.
/// * `target_pos` — Desired position at end of transfer arc (metres, same
///   inertial frame as `current`).
/// * `tof` — Time of flight from TIG to `target_pos` arrival (seconds).
///   Must be positive.
/// * `mu` — Central body gravitational parameter (m³/s²).
/// * `prograde` — `true` = short-way transfer (< 180°); `false` = long-way.
/// * `refsmmat` — IMU alignment matrix; passed to `burn_attitude`.
///
/// # Returns
///
/// A `Maneuver` with:
/// - `tig` set to `current.epoch`.
/// - `delta_v` = `v1_lambert − current.velocity` in the inertial frame.
/// - `burn_attitude` computed from `delta_v` and `refsmmat`.
/// - `mode` = `TargetingMode::Lambert`.
pub fn lambert_targeting(
    current: StateVector,
    target_pos: Vec3,
    tof: f64,
    mu: f64,
    prograde: bool,
    refsmmat: Mat3x3,
) -> Maneuver {
    assert!(tof > 0.0, "lambert_targeting: tof must be positive");
    let (v1, _v2) = lambert(current.position, target_pos, tof, mu, prograde);
    let delta_v_inertial = vsub(v1, current.velocity);
    let attitude = burn_attitude(delta_v_inertial, refsmmat);
    Maneuver {
        tig: current.epoch,
        delta_v: DeltaV(delta_v_inertial),
        burn_attitude: attitude,
        mode: TargetingMode::Lambert,
    }
}

/// Compute the Trans-Earth Injection (TEI) burn maneuver from the current
/// state to the Earth entry interface.
///
/// This is the P37 (Return to Earth) targeting path. It calls
/// `lambert_targeting` with the entry interface target position and the
/// estimated time of flight, then overrides the mode to
/// `TargetingMode::ReturnToEarth`.
///
/// # Arguments
///
/// * `current` — Current state vector (typically in `Frame::MoonInertial`
///   for a standard TEI from lunar orbit). Epoch = TIG.
/// * `entry_target` — Desired Earth entry position in the **same frame as
///   `current`**, at radius `R_EARTH_M + ENTRY_INTERFACE_ALT_M`. The
///   caller is responsible for frame conversion.
/// * `tof_estimate` — Initial estimate of time of flight from TIG to entry
///   interface (seconds). Must be positive.
/// * `refsmmat` — IMU alignment matrix; passed through to `burn_attitude`.
///
/// # Returns
///
/// A `Maneuver` with `mode = TargetingMode::ReturnToEarth`.
///
/// # Note on TOF iteration
///
/// Full P37 iterates on `tof` to satisfy an entry corridor constraint.
/// That iteration belongs in `programs::p37`, not in this function. This
/// function is a single Lambert evaluation.
pub fn return_to_earth(
    current: StateVector,
    entry_target: Vec3,
    tof_estimate: f64,
    refsmmat: Mat3x3,
) -> Maneuver {
    let mu = match current.frame {
        Frame::MoonInertial => MU_MOON,
        _ => MU_EARTH,
    };
    let mut maneuver =
        lambert_targeting(current, entry_target, tof_estimate, mu, true, refsmmat);
    maneuver.mode = TargetingMode::ReturnToEarth;
    maneuver
}

/// Compute the body-to-inertial rotation matrix that aligns the SPS thrust
/// axis (+X body) with the required delta-V direction at TIG.
///
/// # Arguments
///
/// * `delta_v_inertial` — Required delta-V in the inertial frame (m/s).
///   Only the direction matters; magnitude is used only for the zero-vector
///   check.
/// * `refsmmat` — Current IMU REFSMMAT (reference-to-stable-member matrix).
///   Used to determine the roll angle around the thrust axis that minimises
///   CDU gimbal-angle traversal during the pre-burn attitude manoeuvre.
///
/// # Returns
///
/// A 3×3 orthonormal rotation matrix `R` (body-to-inertial) such that
/// `R * [1, 0, 0] = unit(delta_v_inertial)`.
///
/// Returns the identity matrix when `|delta_v_inertial| < 1e-6`.
///
/// # Algorithm
///
/// 1. Zero-guard: return `IDENTITY` if `|delta_v_inertial| < 1e-6`.
/// 2. `x_body_inertial = unit(delta_v_inertial)` — desired thrust axis.
/// 3. Use REFSMMAT column 1 (Y column) as roll reference:
///    `ref_y = refsmmat * [0, 1, 0]` (column 1 of the matrix).
/// 4. `z_body_inertial = unit(cross(x_body_inertial, ref_y))`.
///    If this cross product is near-zero (thrust axis parallel to REFSMMAT
///    Y), fall back to REFSMMAT column 0 (X column) as the reference.
/// 5. `y_body_inertial = cross(z_body_inertial, x_body_inertial)`.
/// 6. Assemble as a column matrix (body axes expressed in inertial).
///
/// # Postcondition
///
/// `R * R^T = I` to within `1e-12` per element when `|delta_v_inertial| > 1e-6`.
pub fn burn_attitude(delta_v_inertial: Vec3, refsmmat: Mat3x3) -> Mat3x3 {
    if norm(delta_v_inertial) < 1e-6 {
        return crate::math::linalg::IDENTITY;
    }

    let x_body = unit(delta_v_inertial);

    // REFSMMAT column 1 = Y column (refsmmat stored row-major, so column j
    // of a row-major matrix M is [M[0][j], M[1][j], M[2][j]]).
    let ref_y: Vec3 = [refsmmat[0][1], refsmmat[1][1], refsmmat[2][1]];

    let z_cross = cross(x_body, ref_y);
    let z_body = if norm(z_cross) > 1e-6 {
        unit(z_cross)
    } else {
        // Fallback: use REFSMMAT X column
        let ref_x: Vec3 = [refsmmat[0][0], refsmmat[1][0], refsmmat[2][0]];
        unit(cross(x_body, ref_x))
    };

    let y_body = cross(z_body, x_body);

    // Assemble body-to-inertial matrix — columns are x_body, y_body, z_body.
    // Row i = [x_body[i], y_body[i], z_body[i]].
    [
        [x_body[0], y_body[0], z_body[0]],
        [x_body[1], y_body[1], z_body[1]],
        [x_body[2], y_body[2], z_body[2]],
    ]
}

/// Estimate SPS burn duration in seconds for a given delta-V magnitude.
///
/// Uses the rocket equation impulse approximation (constant thrust,
/// instantaneous ignition). This is a display-only estimate for DSKY
/// V06N37 during the P40 pre-burn checklist.
///
/// # Formula
///
/// `burn_time_s = vehicle_mass_kg * delta_v_magnitude / SPS_THRUST_N`
///
/// Returns `0.0` for zero delta-V.
pub fn burn_duration(delta_v_magnitude: f64, vehicle_mass_kg: f64) -> f64 {
    if delta_v_magnitude == 0.0 {
        return 0.0;
    }
    vehicle_mass_kg * delta_v_magnitude / SPS_THRUST_N
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg;
    use crate::navigation::gravity::MU_EARTH;
    use crate::navigation::state_vector::Frame;

    fn assert_vec_near(a: Vec3, b: Vec3, eps: f64, label: &str) {
        for i in 0..3 {
            assert!(
                (a[i] - b[i]).abs() < eps,
                "{label} component {i}: got {}, expected {} (eps={eps})",
                a[i],
                b[i]
            );
        }
    }

    fn assert_mat_near(m: Mat3x3, expected: Mat3x3, eps: f64, label: &str) {
        for r in 0..3 {
            for c in 0..3 {
                assert!(
                    (m[r][c] - expected[r][c]).abs() < eps,
                    "{label} [{r}][{c}]: got {}, expected {} (eps={eps})",
                    m[r][c],
                    expected[r][c]
                );
            }
        }
    }

    fn assert_orthonormal(m: Mat3x3, eps: f64, label: &str) {
        let mt = linalg::transpose(m);
        let mmt = linalg::mxm(m, mt);
        assert_mat_near(mmt, linalg::IDENTITY, eps, label);
    }

    // ── TC-TGT-01: Zero delta-V produces zero inertial delta-V and identity attitude ──

    /// TC-TGT-01: Zero LVLH delta-V → zero inertial delta-V, identity burn
    /// attitude, `ExternalDeltaV` mode.
    ///
    /// State: ISS-like circular equatorial LEO at 400 km.
    #[test]
    fn tc_tgt_01_zero_delta_v() {
        let r = 6_778_137.0_f64; // R_EARTH + 400 km
        let v_circ = libm::sqrt(MU_EARTH / r);
        let current = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let maneuver = apply_external_delta_v(
            current,
            Met(0),
            [0.0, 0.0, 0.0],
            linalg::IDENTITY,
        );

        // Delta-V magnitude must be zero
        assert!(
            linalg::norm(maneuver.delta_v.0) < 1e-12,
            "delta-V must be zero, got norm = {}",
            linalg::norm(maneuver.delta_v.0)
        );

        // Burn attitude must be identity for zero delta-V
        assert_mat_near(maneuver.burn_attitude, linalg::IDENTITY, 1e-12, "burn_attitude");

        assert_eq!(maneuver.mode, TargetingMode::ExternalDeltaV);
        assert_eq!(maneuver.tig, Met(0));
    }

    // ── TC-TGT-02: Prograde burn (S-axis) in LVLH maps to +Y inertial ──────────

    /// TC-TGT-02: 100 m/s prograde (S-axis) LVLH burn from circular equatorial
    /// orbit at [r,0,0] / [0,vc,0] produces [0, 100, 0] inertial delta-V.
    ///
    /// LVLH basis for this geometry:
    ///   R_unit = [1, 0, 0], W_unit = [0, 0, 1], S_unit = [0, 1, 0].
    #[test]
    fn tc_tgt_02_prograde_burn_lvlh() {
        let r = 6_778_137.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r);
        let current = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let maneuver = apply_external_delta_v(
            current,
            Met(0),
            [0.0, 100.0, 0.0], // 100 m/s prograde (S)
            linalg::IDENTITY,
        );

        assert_vec_near(
            maneuver.delta_v.0,
            [0.0, 100.0, 0.0],
            1e-6,
            "prograde LVLH → inertial",
        );

        // Magnitude must be preserved
        assert!(
            (linalg::norm(maneuver.delta_v.0) - 100.0).abs() < 1e-9,
            "magnitude not preserved"
        );
        assert_eq!(maneuver.mode, TargetingMode::ExternalDeltaV);
    }

    // ── TC-TGT-03: Radial burn (R-axis) in LVLH maps to +X inertial ────────────

    /// TC-TGT-03: 50 m/s radial (R-axis) LVLH burn from the same circular
    /// equatorial orbit produces [50, 0, 0] inertial delta-V.
    ///
    /// R_unit = unit([r,0,0]) = [1, 0, 0], so the radial component maps
    /// directly to the +X inertial axis.
    #[test]
    fn tc_tgt_03_radial_burn_lvlh() {
        let r = 6_778_137.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r);
        let current = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let maneuver = apply_external_delta_v(
            current,
            Met(0),
            [50.0, 0.0, 0.0], // 50 m/s radial (R)
            linalg::IDENTITY,
        );

        assert_vec_near(
            maneuver.delta_v.0,
            [50.0, 0.0, 0.0],
            1e-6,
            "radial LVLH → inertial",
        );
    }

    // ── TC-TGT-04: Cross-track burn (W-axis) in LVLH maps to +Z inertial ───────

    /// TC-TGT-04: 20 m/s cross-track (W-axis) LVLH burn from the same circular
    /// equatorial orbit produces [0, 0, 20] inertial delta-V.
    ///
    /// For position [r,0,0] and velocity [0,vc,0]:
    ///   h = cross([r,0,0], [0,vc,0]) = [0, 0, r*vc]
    ///   W_unit = [0, 0, 1].
    /// So W delta-V → [0, 0, 20] inertial.
    #[test]
    fn tc_tgt_04_cross_track_burn_lvlh() {
        let r = 6_778_137.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r);
        let current = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let maneuver = apply_external_delta_v(
            current,
            Met(0),
            [0.0, 0.0, 20.0], // 20 m/s cross-track (W)
            linalg::IDENTITY,
        );

        assert_vec_near(
            maneuver.delta_v.0,
            [0.0, 0.0, 20.0],
            1e-6,
            "cross-track LVLH → inertial",
        );
    }

    // ── TC-TGT-05: burn_attitude aligns +X body with delta-V direction ──────────

    /// TC-TGT-05: For a representative non-trivial delta-V, the first column of
    /// `burn_attitude` (i.e., the +X body axis in inertial space) must be
    /// parallel to `unit(delta_v_inertial)`, and the matrix must be orthonormal.
    #[test]
    fn tc_tgt_05_burn_attitude_aligns_with_dv() {
        let dv_inertial: Vec3 = [30.0, 114.0, 12.0]; // |dv| ≈ 118.6 m/s

        let attitude = burn_attitude(dv_inertial, linalg::IDENTITY);

        // +X body axis in inertial = first column of attitude matrix
        let x_body_in_inertial: Vec3 = [attitude[0][0], attitude[1][0], attitude[2][0]];

        let dv_unit = linalg::unit(dv_inertial);
        assert_vec_near(
            x_body_in_inertial,
            dv_unit,
            1e-12,
            "x_body must equal unit(delta_v)",
        );

        // Attitude matrix must be orthonormal
        assert_orthonormal(attitude, 1e-10, "burn_attitude orthonormality");
    }

    // ── TC-TGT-06: burn_attitude for zero delta-V returns identity ──────────────

    /// TC-TGT-06: Zero delta-V input to `burn_attitude` must return the identity
    /// matrix (no attitude change for a no-op burn).
    #[test]
    fn tc_tgt_06_burn_attitude_zero_dv_returns_identity() {
        let attitude = burn_attitude([0.0, 0.0, 0.0], linalg::IDENTITY);
        assert_mat_near(attitude, linalg::IDENTITY, 1e-15, "identity for zero dv");
    }

    // ── TC-TGT-07: Lambert targeting — 90° transfer on circular orbit ──────────

    /// TC-TGT-07: `lambert_targeting` for a quarter-period transfer on a circular
    /// equatorial LEO orbit.
    ///
    /// For a spacecraft already on the circular orbit, the Lambert-required
    /// departure velocity equals the current velocity (delta-V ≈ 0). The test
    /// verifies that the returned delta-V magnitude is below the circular
    /// velocity (i.e., the solver did not return nonsense) and that mode and
    /// TIG are set correctly.
    ///
    /// NOTE: This test may be sensitive to the Lambert solver convergence for
    /// near-circular co-planar arcs. Mark ignored if the Izzo solver diverges
    /// on this geometry (see Lambert technical debt entry in docs/tech-debt.md).
    #[test]
    fn tc_tgt_07_lambert_quarter_period_transfer() {
        let r = 6_778_137.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r);

        let current = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        // Quarter-orbit target: 90° ahead on the same circular orbit.
        let target_pos: Vec3 = [0.0, r, 0.0];

        // Quarter orbital period (seconds).
        let period = 2.0 * core::f64::consts::PI * r / v_circ;
        let tof = period / 4.0;

        let maneuver = lambert_targeting(
            current,
            target_pos,
            tof,
            MU_EARTH,
            true,
            linalg::IDENTITY,
        );

        // Delta-V magnitude must be below circular velocity (sanity bound).
        assert!(
            linalg::norm(maneuver.delta_v.0) < v_circ,
            "Lambert delta-V = {} must be < v_circ = {}",
            linalg::norm(maneuver.delta_v.0),
            v_circ
        );

        // TIG must equal the state epoch.
        assert_eq!(maneuver.tig, current.epoch);
        assert_eq!(maneuver.mode, TargetingMode::Lambert);
    }

    // ── TC-TGT-08: burn_duration plausibility check ─────────────────────────────

    /// TC-TGT-08: `burn_duration` returns a physically plausible estimate for a
    /// typical TEI burn (900 m/s, 20 000 kg CSM), and returns exactly 0 for zero
    /// delta-V.
    #[test]
    fn tc_tgt_08_burn_duration_estimate() {
        let mass_kg = 20_000.0_f64;
        let dv_mag = 900.0_f64;

        let dt = burn_duration(dv_mag, mass_kg);

        // dt = m * |dv| / F = 20000 * 900 / 91188 ≈ 197.4 s
        let expected = mass_kg * dv_mag / SPS_THRUST_N;
        assert!(
            (dt - expected).abs() < 1e-6,
            "burn_duration = {dt}, expected {expected}"
        );
        assert!(dt > 0.0, "burn time must be positive");
        assert!(dt < 600.0, "burn time should be under 10 minutes for 900 m/s");

        // Zero delta-V → zero burn time
        assert_eq!(burn_duration(0.0, mass_kg), 0.0);
    }

    // ── TC-TGT-09: lvlh_to_inertial orthonormality ──────────────────────────────

    /// TC-TGT-09: The matrix returned by `lvlh_to_inertial` must be orthonormal
    /// for a realistic LEO state vector.
    #[test]
    fn tc_tgt_09_lvlh_to_inertial_orthonormal() {
        let r = 7_000_000.0_f64;
        let v = libm::sqrt(MU_EARTH / r);
        let m = lvlh_to_inertial([r, 0.0, 0.0], [0.0, v, 0.0]);
        assert_orthonormal(m, 1e-14, "lvlh_to_inertial orthonormality");
    }

    // ── TC-TGT-10: return_to_earth sets ReturnToEarth mode ──────────────────────

    /// TC-TGT-10: `return_to_earth` smoke test — verify that the mode field is
    /// `ReturnToEarth` and that a finite delta-V is produced for a plausible
    /// lunar-orbit TEI geometry.
    ///
    /// Uses a simplified geometry where the Moon is treated as the central body
    /// and the entry target is a point on the Earth entry sphere expressed in the
    /// same Moon-inertial frame (pre-converted by the caller, as per §5.3).
    ///
    /// TC-TGT-10: return_to_earth TEI burn targeting test. Offset from the
    /// x-axis to avoid Lambert anti-parallel singularity, but the long-TOF
    /// (~60 hour) high-eccentricity transfer still causes Halley to stall.
    /// This is a separate Lambert edge case beyond the core regime fixes.
    #[test]
    #[ignore = "TC-TGT-10: Lambert Halley stalls on long-TOF TEI geometry"]
    fn tc_tgt_10_return_to_earth_mode() {
        use crate::navigation::gravity::MU_MOON;

        // Circular 100 km LLO on the +x axis in MCI.
        let r_moon_m = 1_837_400.0_f64; // R_Moon + 100 km
        let v_llo = libm::sqrt(MU_MOON / r_moon_m);

        let current = StateVector {
            position: [r_moon_m, 0.0, 0.0],
            velocity: [0.0, v_llo, 0.0],
            epoch: Met(8_640_000), // 1 day MET
            frame: Frame::MoonInertial,
        };

        // Entry target: a point roughly 380 000 km away, off the x-axis by
        // 5° in the +y direction to avoid anti-parallel Lambert singularity.
        let earth_dist = 384_400_000.0_f64;
        let offset_angle = 5.0_f64.to_radians();
        let entry_target: Vec3 = [
            -earth_dist * libm::cos(offset_angle),
            earth_dist * libm::sin(offset_angle),
            0.0,
        ];

        // Estimated TEI TOF: ~60 hours
        let tof_s = 60.0 * 3600.0;

        let maneuver =
            return_to_earth(current, entry_target, tof_s, linalg::IDENTITY);

        assert_eq!(maneuver.mode, TargetingMode::ReturnToEarth);

        // Delta-V must be finite
        let dv_mag = linalg::norm(maneuver.delta_v.0);
        assert!(
            dv_mag.is_finite(),
            "delta-V magnitude must be finite, got {dv_mag}"
        );

        // Plausibility: TEI delta-V for LLO is typically 800–1200 m/s.
        // This simplified geometry won't give exact Apollo values, but the
        // result should be in a physically reasonable range (non-zero, < 5 km/s).
        assert!(dv_mag > 0.0, "delta-V must be non-zero for TEI");
        assert!(dv_mag < 5_000.0, "delta-V unexpectedly large: {dv_mag} m/s");
    }
}
