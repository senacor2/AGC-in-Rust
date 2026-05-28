//! Lunar landmark table for P22 lunar-surface landmark tracking.
//!
//! Provides the compile-time selenographic landmark table and the mean lunar
//! radius constant used by the lunar landmark LOS computation in the simulator
//! and by P22's frame-aware landmark dispatch.
//!
//! Coordinate sources are the IAU/USGS lunar gazetteer and the NASSP (APOLLO)
//! scenario landmark tables.  All coordinates are selenographic (Moon-fixed),
//! latitude positive north, longitude positive east.
//!
//! AGC source: no direct AGC counterpart; the lunar landmark table was loaded
//! via uplink in the real system.  This Rust implementation uses a compile-time
//! constant table for Phase 3, consistent with the Earth landmark table in
//! `programs::p22`.

use core::f64::consts::PI;

/// Mean lunar radius (m).
///
/// IAU 2015 value: 1,737.4 km.
/// Used to convert selenographic (lat, lon, alt) to Moon-fixed Cartesian.
pub const R_MOON_M: f64 = 1_737_400.0;

/// One entry in the lunar landmark table.
///
/// Coordinates are selenographic: latitude positive north, longitude positive
/// east of the prime meridian.  `alt_m` is metres above the mean sphere of
/// radius [`R_MOON_M`]; 0.0 for surface-level landmarks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LunarLandmarkEntry {
    /// Short human-readable name (empty string for unused index-0 slot).
    pub name: &'static str,
    /// Selenographic latitude (rad).  Positive north.
    pub lat_rad: f64,
    /// Selenographic longitude (rad).  Positive east.
    pub lon_rad: f64,
    /// Altitude above mean lunar radius (m).  Typically 0.0.
    pub alt_m: f64,
}

impl LunarLandmarkEntry {
    /// Construct from degrees; evaluated at compile time via inline arithmetic.
    const fn from_deg(name: &'static str, lat_deg: f64, lon_deg: f64, alt_m: f64) -> Self {
        Self {
            name,
            lat_rad: lat_deg * PI / 180.0,
            lon_rad: lon_deg * PI / 180.0,
            alt_m,
        }
    }
}

/// Compile-time lunar landmark table.
///
/// The table is 1-indexed to match the Earth landmark table in `programs::p22`
/// and the DSKY crew interface: index 0 is an unused sentinel, indices 1..=8
/// are the real entries.
///
/// # Sources
///
/// | Index | Name                 | Source                                                        |
/// |-------|----------------------|---------------------------------------------------------------|
/// | 1     | Tycho               | IAU/USGS Lunar Gazetteer of Planetary Nomenclature           |
/// | 2     | Copernicus          | IAU/USGS Lunar Gazetteer of Planetary Nomenclature           |
/// | 3     | Censorinus          | IAU/USGS Lunar Gazetteer of Planetary Nomenclature           |
/// | 4     | Maskelyne F         | IAU; matches NASSP "Wash Basin" landmark                     |
/// | 5     | Mount Marilyn       | Crew nickname (Apollo 8); IAU designation Secchi θ           |
/// | 6     | Boot Hill           | Crew nickname (Apollo 8); near Secchi-region rilles          |
/// | 7     | Sidewinder Rille    | Crew landmark (Apollo 8); sinuous rille W of Maskelyne       |
/// | 8     | Aristarchus         | IAU/USGS Lunar Gazetteer of Planetary Nomenclature           |
pub const LUNAR_LANDMARK_TABLE: [LunarLandmarkEntry; 9] = [
    // Index 0 — unused (DSKY is 1-indexed, parity with Earth table).
    LunarLandmarkEntry {
        name: "",
        lat_rad: 0.0,
        lon_rad: 0.0,
        alt_m: 0.0,
    },
    // Index 1 — Tycho crater.
    // Source: IAU/USGS Lunar Gazetteer: −43.31° lat, −11.36° lon.
    LunarLandmarkEntry::from_deg("Tycho", -43.31, -11.36, 0.0),
    // Index 2 — Copernicus crater.
    // Source: IAU/USGS Lunar Gazetteer: +9.62° lat, −20.08° lon.
    LunarLandmarkEntry::from_deg("Copernicus", 9.62, -20.08, 0.0),
    // Index 3 — Censorinus crater.
    // Source: IAU/USGS Lunar Gazetteer: −0.40° lat, +32.69° lon.
    LunarLandmarkEntry::from_deg("Censorinus", -0.40, 32.69, 0.0),
    // Index 4 — Maskelyne F ("Wash Basin").
    // Source: IAU; matches NASSP Apollo scenario landmark: +1.40° lat, +35.05° lon.
    LunarLandmarkEntry::from_deg("Maskelyne F", 1.40, 35.05, 0.0),
    // Index 5 — Mount Marilyn (crew nickname, Apollo 8); IAU Secchi θ.
    // Source: NASSP Apollo 8 scenario: +1.23° lat, +40.01° lon.
    LunarLandmarkEntry::from_deg("Mount Marilyn", 1.23, 40.01, 0.0),
    // Index 6 — Boot Hill (crew nickname, Apollo 8); near Secchi rilles.
    // Source: NASSP Apollo 8 scenario: +0.59° lat, +30.25° lon.
    LunarLandmarkEntry::from_deg("Boot Hill", 0.59, 30.25, 0.0),
    // Index 7 — Sidewinder Rille (Apollo 8 crew landmark).
    // Source: NASSP Apollo 8 scenario: +0.05° lat, +28.08° lon.
    LunarLandmarkEntry::from_deg("Sidewinder Rille", 0.05, 28.08, 0.0),
    // Index 8 — Aristarchus crater.
    // Source: IAU/USGS Lunar Gazetteer: +23.70° lat, −47.50° lon.
    LunarLandmarkEntry::from_deg("Aristarchus", 23.70, -47.50, 0.0),
];

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// tc_lm_table_length_is_9
    ///
    /// The table must contain 9 entries: index 0 (unused sentinel) plus
    /// indices 1..=8 (the eight real lunar landmarks).
    #[test]
    fn tc_lm_table_length_is_9() {
        assert_eq!(LUNAR_LANDMARK_TABLE.len(), 9);
    }

    /// tc_lm_index_zero_is_empty
    ///
    /// Index 0 is the unused DSKY sentinel: name must be empty and all
    /// coordinates must be 0.0.
    #[test]
    fn tc_lm_index_zero_is_empty() {
        let entry = &LUNAR_LANDMARK_TABLE[0];
        assert!(
            entry.name.is_empty(),
            "index 0 name should be empty, got {:?}",
            entry.name
        );
        assert_eq!(entry.lat_rad, 0.0, "index 0 lat_rad should be 0.0");
        assert_eq!(entry.lon_rad, 0.0, "index 0 lon_rad should be 0.0");
        assert_eq!(entry.alt_m, 0.0, "index 0 alt_m should be 0.0");
    }

    /// tc_lm_named_entries_have_finite_coords
    ///
    /// For indices 1..=8, all `lat_rad`, `lon_rad`, and `alt_m` must be finite,
    /// `|lat_rad| <= PI/2`, and `|lon_rad| <= PI`.
    #[test]
    fn tc_lm_named_entries_have_finite_coords() {
        for i in 1..=8usize {
            let entry = &LUNAR_LANDMARK_TABLE[i];
            assert!(
                entry.lat_rad.is_finite(),
                "index {i} ({}) lat_rad is not finite",
                entry.name
            );
            assert!(
                entry.lon_rad.is_finite(),
                "index {i} ({}) lon_rad is not finite",
                entry.name
            );
            assert!(
                entry.alt_m.is_finite(),
                "index {i} ({}) alt_m is not finite",
                entry.name
            );
            assert!(
                entry.lat_rad.abs() <= PI / 2.0,
                "index {i} ({}) lat_rad = {} exceeds PI/2",
                entry.name,
                entry.lat_rad
            );
            assert!(
                entry.lon_rad.abs() <= PI,
                "index {i} ({}) lon_rad = {} exceeds PI",
                entry.name,
                entry.lon_rad
            );
        }
    }

    /// tc_lm_specific_landmarks_within_expected_regions
    ///
    /// Regression pins for well-known landmarks.  Bounds are intentionally loose
    /// (~3° slack) to catch gross errors (wrong hemisphere, inverted sign) while
    /// allowing minor source discrepancies.
    ///
    /// - Tycho (index 1):       lat_rad ∈ (-0.78, -0.74)  ≈ -45° to -42°
    /// - Copernicus (index 2):  lon_rad ∈ (-0.36, -0.34)  ≈ -21° to -19°
    /// - Mount Marilyn (index 5): lat_rad ∈ (0.02, 0.03)  ≈ +1.1° to +1.7°
    #[test]
    fn tc_lm_specific_landmarks_within_expected_regions() {
        // Tycho: expected ≈ -43.31° → -0.7558 rad
        let tycho = &LUNAR_LANDMARK_TABLE[1];
        assert_eq!(tycho.name, "Tycho");
        assert!(
            tycho.lat_rad > -0.78 && tycho.lat_rad < -0.74,
            "Tycho lat_rad = {} not in (-0.78, -0.74)",
            tycho.lat_rad
        );

        // Copernicus: expected ≈ -20.08° → -0.3504 rad
        let copernicus = &LUNAR_LANDMARK_TABLE[2];
        assert_eq!(copernicus.name, "Copernicus");
        assert!(
            copernicus.lon_rad > -0.36 && copernicus.lon_rad < -0.34,
            "Copernicus lon_rad = {} not in (-0.36, -0.34)",
            copernicus.lon_rad
        );

        // Mount Marilyn: expected ≈ +1.23° → 0.02147 rad
        let marilyn = &LUNAR_LANDMARK_TABLE[5];
        assert_eq!(marilyn.name, "Mount Marilyn");
        assert!(
            marilyn.lat_rad > 0.02 && marilyn.lat_rad < 0.03,
            "Mount Marilyn lat_rad = {} not in (0.02, 0.03)",
            marilyn.lat_rad
        );
    }
}
