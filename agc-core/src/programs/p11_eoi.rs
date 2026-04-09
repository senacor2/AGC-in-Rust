//! P11 — Earth Orbit Insertion Monitor.
//!
//! Initiated at liftoff (from P02 gyrocompass when the liftoff discrete is
//! received). The AGC P11 zeros the CMC clock, updates TEPHEM, initialises
//! SERVICER at PREREAD1, and monitors N62 (inertial velocity, altitude rate,
//! altitude above pad). Once above 300,000 ft it also displays apogee/perigee.
//!
//! This Rust implementation is simplified: it computes apoapsis/periapsis
//! altitudes from the current state vector via `rv_to_elements` and makes
//! them available for display. The liftoff epoch zeroing, REFSMMAT
//! computation, and SERVICER initialisation are handled elsewhere.
//!
//! AGC source: Comanche055/P11.agc — EOIMON / P11 entry point.

use crate::navigation::conics::{apsides, rv_to_elements, OrbitalElements};
use crate::navigation::state_vector::StateVector;

/// Program number for Earth Orbit Insertion Monitor.
pub const PROG_NUMBER: u8 = 11;

/// P11 program state.
///
/// Updated once per SERVICER cycle with the current navigation state vector.
///
/// AGC source: P11.agc — EOIMON, uses CSMCONIC/LEMCONIC results stored in
/// RATT/VATT erasable variables.
#[derive(Clone, Copy, Debug)]
pub struct P11State {
    /// Current orbital elements (populated after the first `update` call).
    pub elements: Option<OrbitalElements>,
    /// Apoapsis altitude above the reference sphere (m).
    pub ha: f64,
    /// Periapsis altitude above the reference sphere (m).
    pub hp: f64,
    /// Time elapsed since the program was started (s).
    pub elapsed_s: f64,
}

impl P11State {
    /// Construct a new P11 state with no orbital data.
    pub const fn new() -> Self {
        Self {
            elements: None,
            ha: 0.0,
            hp: 0.0,
            elapsed_s: 0.0,
        }
    }

    /// Update P11 with the current navigation state vector.
    ///
    /// Recomputes orbital elements and updates `ha`/`hp` altitudes.
    /// `r_body` is the reference body's equatorial radius (m); the altitude
    /// is defined as the apsis radius minus that value.
    ///
    /// AGC source: P11.agc — EOIMON calls CSMCONIC to propagate the state
    /// vector and then extracts apogee/perigee from the resulting elements.
    pub fn update(&mut self, sv: &StateVector, r_body: f64, mu: f64) {
        let el = rv_to_elements(&sv.r, &sv.v, mu);
        let (r_periapsis, r_apoapsis) = apsides(el.sma, el.ecc);
        self.hp = r_periapsis - r_body;
        self.ha = r_apoapsis - r_body;
        self.elements = Some(el);
    }

    /// Advance the elapsed-time counter by `dt` seconds.
    pub fn tick(&mut self, dt: f64) {
        self.elapsed_s += dt;
    }
}

impl Default for P11State {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::gravity::{MU_EARTH, RE_EARTH};
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;

    fn make_sv(r: [f64; 3], v: [f64; 3]) -> StateVector {
        StateVector { frame: Frame::Eci, r, v, t: Met(0) }
    }

    /// Circular orbit at 200 km altitude: ha ≈ hp ≈ 200 km.
    #[test]
    fn circular_orbit_ha_hp_approx_equal() {
        let alt = 200_000.0_f64;
        let r_mag = RE_EARTH + alt;
        let v_circ = libm::sqrt(MU_EARTH / r_mag);
        let sv = make_sv([r_mag, 0.0, 0.0], [0.0, v_circ, 0.0]);

        let mut p11 = P11State::new();
        p11.update(&sv, RE_EARTH, MU_EARTH);

        assert!(
            (p11.ha - alt).abs() < 1.0,
            "ha = {} m, expected ≈ {} m",
            p11.ha,
            alt
        );
        assert!(
            (p11.hp - alt).abs() < 1.0,
            "hp = {} m, expected ≈ {} m",
            p11.hp,
            alt
        );
        assert!(p11.elements.is_some());
    }

    /// Elliptic orbit (200 km × 400 km): ha > hp.
    #[test]
    fn elliptic_orbit_ha_greater_than_hp() {
        let r_p = RE_EARTH + 200_000.0;
        let r_a = RE_EARTH + 400_000.0;
        let a = 0.5 * (r_p + r_a);
        let ecc = (r_a - r_p) / (r_a + r_p);
        // Velocity at periapsis from vis-viva.
        let v_p = libm::sqrt(MU_EARTH * (1.0 + ecc) / (a * (1.0 - ecc)));

        let sv = make_sv([r_p, 0.0, 0.0], [0.0, v_p, 0.0]);

        let mut p11 = P11State::new();
        p11.update(&sv, RE_EARTH, MU_EARTH);

        assert!(
            p11.ha > p11.hp,
            "expected ha ({}) > hp ({})",
            p11.ha,
            p11.hp
        );
        assert!(
            (p11.hp - 200_000.0).abs() < 100.0,
            "hp = {}, expected ≈ 200 000 m",
            p11.hp
        );
        assert!(
            (p11.ha - 400_000.0).abs() < 100.0,
            "ha = {}, expected ≈ 400 000 m",
            p11.ha
        );
    }

    #[test]
    fn tick_advances_elapsed_time() {
        let mut p11 = P11State::new();
        p11.tick(10.0);
        p11.tick(5.0);
        assert!((p11.elapsed_s - 15.0).abs() < 1e-10);
    }
}
