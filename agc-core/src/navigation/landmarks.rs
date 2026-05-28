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
