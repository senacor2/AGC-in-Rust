//! Sensor simulation helpers for P51/P52 star sightings and P22 landmark sightings.
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
//!   inertial (libration deferred to GH #56; Moon-fixed ≡ MCI in MS-T3).

use agc_core::math::linalg::{mxv, unit, vsub};
use agc_core::navigation::landmarks::{LunarLandmarkEntry, LUNAR_LANDMARK_TABLE, R_MOON_M};
use agc_core::navigation::star_catalog::STAR_CATALOG;
use agc_core::programs::p22::{landmark_inertial_pos, LANDMARK_TABLE};
use agc_core::types::{Mat3x3, Vec3};

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
pub fn star_los_in_platform(
    star_id: u8,
    _attitude: &Attitude,
    refsmmat: &Mat3x3,
) -> Vec3 {
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
            lunar_landmark_inertial(&LUNAR_LANDMARK_TABLE[index as usize])
        }
    };

    // LOS = unit(landmark_inertial - csm_pos_inertial)
    let los_inertial = unit(vsub(lm_inertial, csm_pos_inertial));
    // Rotate to platform frame: REFSMMAT · los_inertial
    mxv(*refsmmat, los_inertial)
}

/// Convert a lunar landmark (selenographic) to a Moon-inertial Cartesian position.
///
/// In MS-T3 libration is deferred (GH #56), so Moon-fixed ≡ MCI.
/// The transform is the standard spherical-to-Cartesian:
///
/// ```text
/// r = R_MOON_M + alt_m
/// x = r · cos(lat) · cos(lon)
/// y = r · cos(lat) · sin(lon)
/// z = r · sin(lat)
/// ```
fn lunar_landmark_inertial(entry: &LunarLandmarkEntry) -> Vec3 {
    let r = R_MOON_M + entry.alt_m;
    let cos_lat = entry.lat_rad.cos();
    let sin_lat = entry.lat_rad.sin();
    let cos_lon = entry.lon_rad.cos();
    let sin_lon = entry.lon_rad.sin();
    [r * cos_lat * cos_lon, r * cos_lat * sin_lon, r * sin_lat]
}
