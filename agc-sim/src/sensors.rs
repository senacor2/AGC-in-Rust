// SPDX-License-Identifier: GPL-3.0-or-later
//! Sensor simulation helpers for P51/P52 star sightings, P22 landmark sightings,
//! and P02 gyrocompass Earth-rate injection.
//!
//! Computes platform-frame line-of-sight (LOS) unit vectors from simulator
//! truth state (attitude quaternion + REFSMMAT) for use by the scenario
//! runner when dispatching `OpticsSighting` and `LandmarkSighting` events.
//!
//! # Frame chain
//!
//! ```text
//! Inertial ──(REFSMMAT)──▶ Platform
//! ```
//!
//! For both stars and landmarks the output is a **platform-frame unit vector**.
//! This is the coordinate system that P52's `p52_mark_align` and P22's
//! `p22_incorporate_landmark_mark` expect, consistent with the physical sextant
//! measuring angles relative to the stable-member platform.
//!
//! # Star sightings
//!
//! `star_los_in_platform` reads the inertial direction from `STAR_CATALOG`
//! and rotates it by REFSMMAT.  The `attitude` parameter is kept for forward
//! compatibility (the full inertial → body → platform transform uses it) but
//! is **unused in MS-T3** because P52 expects the platform-frame vector directly.
//!
//! # Landmark sightings
//!
//! `landmark_los_in_platform` branches on [`LandmarkTable`]:
//! - `Earth`: reads the Earth landmark table and calls `landmark_inertial_pos`.
//! - `Moon`: reads `LUNAR_LANDMARK_TABLE`, converts Moon-fixed Cartesian to
//!   inertial using the IAU 2015 lunar-libration rotation at the given epoch
//!   (resolved in GH #56 — prior to #56 the conversion was the identity, which
//!   biased the inertial landmark position by up to ~150 km).

use agc_core::math::linalg::{mxv, unit, vsub};
use agc_core::navigation::time::OMEGA_EARTH;
use agc_core::programs::p01_p02::{earth_rate_horizontal, earth_rate_vertical};
use agc_core::types::CduAngle;
use agc_core::navigation::star_catalog::STAR_CATALOG;
use agc_core::programs::p22::{
    landmark_inertial_pos, lunar_landmark_inertial_at, LANDMARK_TABLE, LUNAR_LANDMARK_TABLE,
};
use agc_core::types::{Mat3x3, Met, Vec3};

use crate::physics::Attitude;
use crate::scenario::LandmarkTable;

/// Compute the platform-frame LOS unit vector toward a catalogue star.
///
/// # Arguments
/// - `star_id`: 1-based AGC catalogue number (1..=37).
/// - `attitude`: current spacecraft attitude (unused in MS-T3; retained for
///   forward compatibility with the full body-frame transform in a later
///   milestone).
/// - `refsmmat`: current REFSMMAT (inertial → platform rotation matrix).
///
/// # Returns
/// Unit vector in the platform frame pointing toward the star.
///
/// # Panics
/// Panics if `star_id == 0` or `star_id > 37` (invalid catalogue number).
pub fn star_los_in_platform(star_id: u8, _attitude: &Attitude, refsmmat: &Mat3x3) -> Vec3 {
    assert!(
        (1..=agc_core::navigation::star_catalog::CATALOG_SIZE).contains(&star_id),
        "star_los_in_platform: star_id {star_id} out of range 1..=37"
    );
    let star_dir_inertial = STAR_CATALOG[(star_id - 1) as usize].direction;
    // Platform frame: REFSMMAT · star_direction_inertial
    mxv(*refsmmat, star_dir_inertial)
}

/// Compute the platform-frame LOS unit vector from the CSM toward a landmark.
///
/// # Arguments
/// - `table`: which landmark table to use ([`LandmarkTable::Earth`] or
///   [`LandmarkTable::Moon`]).
/// - `index`: 1-based landmark table index.
/// - `csm_pos_inertial`: CSM position in the inertial frame (m).
/// - `attitude`: current spacecraft attitude (unused in MS-T3; retained for
///   forward compatibility).
/// - `refsmmat`: current REFSMMAT (inertial → platform).
/// - `gha_epoch_rad`: Greenwich Hour Angle at GET = 0 (rad).  Used only for
///   Earth landmarks; ignored for Moon landmarks.
/// - `epoch`: mission elapsed time at the sighting. Used for the lunar
///   libration rotation (#56); ignored for Earth landmarks.
///
/// # Returns
/// Unit vector in the platform frame pointing from the CSM toward the landmark.
///
/// # Panics
/// Panics if `index == 0` or out of range for the selected table.
pub fn landmark_los_in_platform(
    table: LandmarkTable,
    index: u8,
    csm_pos_inertial: Vec3,
    _attitude: &Attitude,
    refsmmat: &Mat3x3,
    gha_epoch_rad: f64,
    epoch: Met,
) -> Vec3 {
    let lm_inertial: Vec3 = match table {
        LandmarkTable::Earth => {
            assert!(
                (1..=8).contains(&index),
                "landmark_los_in_platform: Earth landmark index {index} out of range 1..=8"
            );
            let entry = &LANDMARK_TABLE[index as usize];
            // GET = 0 is used as a simplification here; the scenario runner
            // injects a real GET via csm_pos_inertial.  The full path uses
            // `landmark_inertial_pos(entry, get_s, gha_epoch_rad)` where get_s
            // is derived from AgcState::time at the time of the sighting event.
            // For now, pass 0.0 as get_s and let the caller supply gha_epoch_rad.
            // TODO (#57): thread AgcState::time.to_seconds() through here.
            landmark_inertial_pos(entry, 0.0, gha_epoch_rad)
        }
        LandmarkTable::Moon => {
            assert!(
                (1..=8).contains(&index),
                "landmark_los_in_platform: Moon landmark index {index} out of range 1..=8"
            );
            lunar_landmark_inertial_at(&LUNAR_LANDMARK_TABLE[index as usize], epoch)
        }
    };

    // LOS = unit(landmark_inertial - csm_pos_inertial)
    let los_inertial = unit(vsub(lm_inertial, csm_pos_inertial));
    // Rotate to platform frame: REFSMMAT · los_inertial
    mxv(*refsmmat, los_inertial)
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::Attitude;
    use crate::scenario::LandmarkTable;
    use agc_core::math::linalg::mxv;
    use agc_core::navigation::star_catalog::STAR_CATALOG;
    use agc_core::programs::p22::R_MOON_M;
    use agc_core::types::{Mat3x3, Met, Vec3};

    const IDENTITY_REFSMMAT: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    fn default_attitude() -> Attitude {
        Attitude {
            q: [1.0, 0.0, 0.0, 0.0],
            commanded_q: [1.0, 0.0, 0.0, 0.0],
            slew_tau_s: 5.0,
        }
    }

    /// tc_sens_star_los_identity_refsmmat
    ///
    /// With identity REFSMMAT and identity attitude, `star_los_in_platform`
    /// for star 1 (Alpheratz) must return the catalog inertial unit vector
    /// unchanged, since multiplying by the identity matrix is a no-op.
    #[test]
    fn tc_sens_star_los_identity_refsmmat() {
        let attitude = default_attitude();
        let star_dir = STAR_CATALOG[0].direction; // star 1 = Alpheratz
        let los = star_los_in_platform(1, &attitude, &IDENTITY_REFSMMAT);
        for i in 0..3 {
            assert!(
                (los[i] - star_dir[i]).abs() < 1e-14,
                "component {i}: expected {}, got {}",
                star_dir[i],
                los[i]
            );
        }
    }

    /// tc_sens_star_los_rotated_refsmmat
    ///
    /// With a 90°-about-X REFSMMAT, `star_los_in_platform` returns the
    /// catalog inertial vector rotated by that matrix.  We verify each
    /// component against `mxv(refsmmat, star_dir)` computed independently.
    #[test]
    fn tc_sens_star_los_rotated_refsmmat() {
        // 90° rotation about X maps Y→Z and Z→-Y.
        let refsmmat_90x: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]];
        let attitude = default_attitude();
        // Use star 1 (Alpheratz) as the test vector.
        let star_dir = STAR_CATALOG[0].direction;

        // Manually compute expected result: M · star_dir.
        let expected = mxv(refsmmat_90x, star_dir);
        let got = star_los_in_platform(1, &attitude, &refsmmat_90x);

        for i in 0..3 {
            assert!(
                (got[i] - expected[i]).abs() < 1e-9,
                "component {i}: expected {}, got {}",
                expected[i],
                got[i]
            );
        }
    }

    /// tc_sens_landmark_earth_returns_unit_vector
    ///
    /// For Earth landmark 3 (a surface landmark at roughly 0° lat), a CSM at
    /// +X (7000 km altitude) with identity REFSMMAT and GHA=0, the LOS in
    /// platform must have L2 norm == 1.0 within 1e-12.  The rough direction
    /// must point away from +X (i.e. the X component of the LOS must be negative
    /// since the CSM is at large +X and the landmark is near the equator at the
    /// center).
    #[test]
    fn tc_sens_landmark_earth_returns_unit_vector() {
        let attitude = default_attitude();
        // CSM at 7000 km on +X axis in ECI (above equator).
        let csm_pos: Vec3 = [7_000_000.0, 0.0, 0.0];
        let los = landmark_los_in_platform(
            LandmarkTable::Earth,
            3,
            csm_pos,
            &attitude,
            &IDENTITY_REFSMMAT,
            0.0, // GHA = 0
            Met(0),
        );
        let norm = (los[0] * los[0] + los[1] * los[1] + los[2] * los[2]).sqrt();
        assert!(
            (norm - 1.0).abs() < 1e-12,
            "LOS must be unit vector; norm = {norm}"
        );
        // CSM is far along +X from Earth center; most landmarks are closer to
        // origin, so los[0] should be negative (pointing back toward Earth).
        assert!(
            los[0] < 0.0,
            "LOS X component should be negative (landmark behind CSM in +X direction), got {}",
            los[0]
        );
    }

    /// tc_sens_landmark_moon_returns_unit_vector
    ///
    /// For Mount Marilyn (lunar landmark index 5), with a CSM at 100 km LLO
    /// and identity REFSMMAT, the LOS in platform must be a unit vector.
    /// The CSM is placed at the inertial position of the landmark plus a
    /// 100 km radial offset, so the LOS points back along the radial: the
    /// dot product with the inertial up-vector must be ≈ −1. This decouples
    /// the test from the exact libration epoch — pre-#56 the landmark was
    /// fixed in inertial space, so the CSM placement was fixed too.
    #[test]
    fn tc_sens_landmark_moon_returns_unit_vector() {
        let attitude = default_attitude();
        let epoch = Met(0);

        let entry = &agc_core::programs::p22::LUNAR_LANDMARK_TABLE[5];
        let lm_inertial = agc_core::programs::p22::lunar_landmark_inertial_at(entry, epoch);
        let lm_norm =
            f64::sqrt(lm_inertial[0].powi(2) + lm_inertial[1].powi(2) + lm_inertial[2].powi(2));
        let up: Vec3 = [
            lm_inertial[0] / lm_norm,
            lm_inertial[1] / lm_norm,
            lm_inertial[2] / lm_norm,
        ];
        // CSM = landmark + 100 km radially outward (still at LLO altitude).
        let csm_pos: Vec3 = [
            lm_inertial[0] + 100_000.0 * up[0],
            lm_inertial[1] + 100_000.0 * up[1],
            lm_inertial[2] + 100_000.0 * up[2],
        ];

        let los = landmark_los_in_platform(
            LandmarkTable::Moon,
            5, // Mount Marilyn
            csm_pos,
            &attitude,
            &IDENTITY_REFSMMAT,
            0.0,   // GHA unused for Moon
            epoch, // applies the IAU 2015 libration rotation
        );

        let norm = f64::sqrt(los[0] * los[0] + los[1] * los[1] + los[2] * los[2]);
        assert!(
            (norm - 1.0).abs() < 1e-12,
            "LOS to Moon landmark must be unit vector; norm = {norm}"
        );
        // CSM sits directly above the landmark along the radial, so the LOS
        // back to the landmark equals −up.
        let dot = los[0] * up[0] + los[1] * up[1] + los[2] * up[2];
        assert!(
            (dot + 1.0).abs() < 1e-9,
            "LOS·up should equal −1 when CSM is radially above the landmark; got {dot}"
        );
        let _ = R_MOON_M; // ensure the still-shared import stays referenced
    }
}

// ── SimEarthRate ──────────────────────────────────────────────────────────────

/// Software Earth-rate provider for the P02 gyrocompass loop.
///
/// Encapsulates the launch-site geodetic latitude and provides:
/// - The Earth rotation vector in the ECI frame (always `[0, 0, Ω_E]`).
/// - Helpers to inject a specified CDU misalignment into `AgcState` so the
///   gyrocompass loop can be exercised from a realistic starting condition.
///
/// AGC correspondence: `ERTHRVSE` in
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` which builds `ERVECTOR`
/// from `LATITUDE` and the constant `OMEG/MS`.
pub struct SimEarthRate {
    /// Launch-site geodetic latitude (radians).
    pub launch_lat_rad: f64,
}

impl SimEarthRate {
    /// Kennedy Space Center latitude: 28.573° N ≈ 0.4986 rad.
    pub const KSC_LAT_RAD: f64 = 0.4986;

    pub fn ksc() -> Self {
        Self { launch_lat_rad: Self::KSC_LAT_RAD }
    }

    pub fn new(launch_lat_rad: f64) -> Self {
        Self { launch_lat_rad }
    }

    /// Earth rotation vector in the ECI frame (rad/s).  Always `[0, 0, Ω_E]`.
    pub fn earth_rate_eci(&self) -> [f64; 3] {
        [0.0, 0.0, OMEGA_EARTH]
    }

    /// Horizontal (North) Earth rate component at the launch latitude (rad/s).
    pub fn horizontal_rate(&self) -> f64 {
        earth_rate_horizontal(self.launch_lat_rad)
    }

    /// Vertical (Up) Earth rate component at the launch latitude (rad/s).
    pub fn vertical_rate(&self) -> f64 {
        earth_rate_vertical(self.launch_lat_rad)
    }

    /// Prime `state` with the launch latitude and inject `misalignment_deg`
    /// degrees of CDU offset on all three axes.
    ///
    /// Call before `init_p02` to exercise the gyrocompass from a realistic
    /// starting condition.
    pub fn prime_state(
        &self,
        state: &mut agc_core::AgcState,
        misalignment_deg: f64,
    ) {
        state.launch_lat_rad = self.launch_lat_rad;
        let counts = (misalignment_deg / 360.0 * 65536.0).round() as i16;
        for c in state.current_cdu.iter_mut() {
            c.0 = counts;
        }
    }

    /// Convert degrees to a `CduAngle` count value.
    pub fn deg_to_counts(misalignment_deg: f64) -> CduAngle {
        CduAngle((misalignment_deg / 360.0 * 65536.0).round() as i16)
    }
}

#[cfg(test)]
mod sim_earth_rate_tests {
    use super::*;
    use agc_core::control::imu_control::{ImuAlignmentState, COARSE_ALIGN_THRESHOLD};
    use agc_core::programs::p01_p02::init_p02;
    use agc_core::AgcState;

    /// TC-SER-1: KSC provider gives Earth rate along ECI +Z.
    #[test]
    fn tc_ser_1_earth_rate_eci_points_north_pole() {
        let er = SimEarthRate::ksc();
        let v = er.earth_rate_eci();
        assert_eq!(v[0], 0.0);
        assert_eq!(v[1], 0.0);
        assert!((v[2] - OMEGA_EARTH).abs() < 1e-15);
    }

    /// TC-SER-2: prime_state sets launch_lat_rad and CDU offset correctly.
    #[test]
    fn tc_ser_2_prime_state_sets_cdu_and_lat() {
        let er = SimEarthRate::ksc();
        let mut state = AgcState::new();
        er.prime_state(&mut state, 10.0);
        assert!((state.launch_lat_rad - SimEarthRate::KSC_LAT_RAD).abs() < 1e-10);
        let expected = SimEarthRate::deg_to_counts(10.0).0;
        for c in state.current_cdu.iter() {
            assert_eq!(c.0, expected);
        }
    }

    /// TC-SER-3: P02 convergence from 30° misalignment using SimEarthRate.
    #[test]
    fn tc_ser_3_p02_convergence_via_sim_earth_rate() {
        let er = SimEarthRate::ksc();
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::Caged;
        er.prime_state(&mut state, 30.0);
        init_p02(&mut state);

        let mut steps = 0usize;
        while state.imu_alignment_state != ImuAlignmentState::CoarseAligned
            && steps < 200
        {
            let Some((task_fn, _)) = state.waitlist.pop_task() else { break };
            task_fn(&mut state);
            steps += 1;
        }

        assert_eq!(
            state.imu_alignment_state,
            ImuAlignmentState::CoarseAligned,
            "TC-SER-3: must reach CoarseAligned within 200 steps (took {steps})"
        );
        for (i, c) in state.current_cdu.iter().enumerate() {
            assert!(
                c.0.unsigned_abs() <= COARSE_ALIGN_THRESHOLD,
                "TC-SER-3: CDU[{i}] = {} exceeds threshold {COARSE_ALIGN_THRESHOLD}",
                c.0
            );
        }
    }
}
