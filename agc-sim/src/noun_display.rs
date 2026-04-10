//! Noun-driven register population.
//!
//! Given an active noun code and the current mission state, compute the
//! three DSKY register values that should be displayed. Matches the AGC
//! noun table from PINBALL_NOUN_TABLES.agc (subset relevant to the demo).

use crate::mission::Mission;
use agc_core::navigation::conics::{apsides, rv_to_elements};
use agc_core::navigation::gravity::{MU_EARTH, RE_EARTH};

/// Value for one register: optional sign, integer, number of digits.
#[derive(Clone, Copy, Debug)]
pub struct RegValue {
    pub sign: i8,   // +1, -1, or 0 (blank)
    pub value: u32, // 0..99999
    pub blank: bool,
}

impl RegValue {
    pub const BLANK: Self = Self {
        sign: 0,
        value: 0,
        blank: true,
    };

    pub fn from_i32(v: i32) -> Self {
        Self {
            sign: if v >= 0 { 1 } else { -1 },
            value: v.unsigned_abs().min(99_999),
            blank: false,
        }
    }
}

/// Populate R1/R2/R3 based on the active noun.
///
/// Supported nouns (Milestone 5 demo subset):
/// - N33: time of ignition (hours / min / sec)
/// - N36: AGC clock (MET)
/// - N44: apoapsis / periapsis altitude / period (km, km, min)
/// - N62: inertial velocity / altitude rate / altitude (m/s, m/s, m)
/// - N76: desired downrange velocity / radial velocity / crossrange
/// - N85: velocity-to-be-gained body frame (m/s x 3)
/// - N00: blank all
///
/// For an unknown noun returns [BLANK; 3].
///
/// AGC source: PINBALL_NOUN_TABLES.agc — NNADTAB definitions.
pub fn registers_for(noun: u8, mission: &Mission, vg_body_ms: Option<[f64; 3]>) -> [RegValue; 3] {
    match noun {
        0 => [RegValue::BLANK; 3],
        33 => {
            // TIG: hours / min / sec (demo: TIG = MET + 60 s)
            let tig_s = mission.sv.t.0 / 100 + 60;
            let h = (tig_s / 3600) as i32;
            let m = ((tig_s / 60) % 60) as i32;
            let s = (tig_s % 60) as i32;
            [
                RegValue::from_i32(h),
                RegValue::from_i32(m),
                RegValue::from_i32(s),
            ]
        }
        36 => {
            // MET hours/min/sec
            let total_s = mission.sv.t.0 / 100;
            let h = (total_s / 3600) as i32;
            let m = ((total_s / 60) % 60) as i32;
            let s = (total_s % 60) as i32;
            [
                RegValue::from_i32(h),
                RegValue::from_i32(m),
                RegValue::from_i32(s),
            ]
        }
        44 => {
            // Apoapsis km, periapsis km, period (centiseconds)
            let el = rv_to_elements(&mission.sv.r, &mission.sv.v, MU_EARTH);
            let (peri_r, apo_r) = apsides(el.sma, el.ecc);
            let apo_km = ((apo_r - RE_EARTH) / 100.0) as i32; // tenths of km
            let per_km = ((peri_r - RE_EARTH) / 100.0) as i32;
            let period_cs = if el.sma.is_finite() && el.sma > 0.0 {
                (2.0 * std::f64::consts::PI * (el.sma.powi(3) / MU_EARTH).sqrt() * 100.0) as i32
            } else {
                0
            };
            [
                RegValue::from_i32(apo_km),
                RegValue::from_i32(per_km),
                RegValue::from_i32(period_cs),
            ]
        }
        62 => {
            // Inertial velocity (m/s × 10) / altitude rate (m/s × 10) / altitude (m)
            let v = &mission.sv.v;
            let speed = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            let r = &mission.sv.r;
            let r_mag = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
            let h_dot = (r[0] * v[0] + r[1] * v[1] + r[2] * v[2]) / r_mag;
            let alt = r_mag - RE_EARTH;
            [
                RegValue::from_i32((speed * 10.0) as i32),
                RegValue::from_i32((h_dot * 10.0) as i32),
                RegValue::from_i32(alt as i32),
            ]
        }
        85 => {
            // VG body frame x/y/z × 10 (tenths of m/s)
            match vg_body_ms {
                Some(vg) => [
                    RegValue::from_i32((vg[0] * 10.0) as i32),
                    RegValue::from_i32((vg[1] * 10.0) as i32),
                    RegValue::from_i32((vg[2] * 10.0) as i32),
                ],
                None => [RegValue::BLANK; 3],
            }
        }
        _ => [RegValue::BLANK; 3],
    }
}
