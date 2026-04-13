//! Navigation state vector: position, velocity, time, and gravity half-step term.
//!
//! The `StateVector` struct bundles the ECI position-velocity pair (RN, VN) with
//! its time tag (PIPTIME) and the saved half-step gravity term (GDT/2) required
//! by the predictor-corrector integration scheme.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//!   `RN ERASE +5` (position, B-29 m),
//!   `VN ERASE +5` (velocity, B-7 m/cs),
//!   `PIPTIME ERASE +1` (time, centiseconds),
//!   `GDT/2 EQUALS PIPTIME +2` (half-step gravity term),
//!   `PBODY ERASE` (primary body selector),
//!   `MOONFLAG = 003D` (Moon integration flag, flag word 0 bit 12).
//!
//! Comanche055/ORBITAL_INTEGRATION.agc: KEPPREP, RECTIFY, ORIGCHNG, DIFEQ+0/+1/+2.
//! Comanche055/SERVICER207.agc: CALCRVG, CALCGRAV, NORMLIZE.

use crate::math::linalg::norm;
use crate::types::{Met, Vec3, ZERO_VEC3};

/// Navigation state vector in Earth-Centred Inertial (ECI) coordinates.
///
/// Bundles position (metres), velocity (m/s), and time (MET centiseconds).
/// Also carries `gdt_over_2`, the half-step gravity term saved by the
/// predictor-corrector integrator for use in the next cycle.
///
/// AGC equivalents:
///   RN        — `ERASABLE_ASSIGNMENTS.agc`, `RN ERASE +5`
///   VN        — `ERASABLE_ASSIGNMENTS.agc`, `VN ERASE +5`
///   PIPTIME   — `ERASABLE_ASSIGNMENTS.agc`, `PIPTIME ERASE +1`
///   GDT/2     — `ERASABLE_ASSIGNMENTS.agc`, `GDT/2 EQUALS PIPTIME +2`
///
/// Position scale: SI metres (f64).
/// Velocity scale: SI m/s (f64).
/// Time: `Met` (centiseconds).
///
/// // M2 approximation: GDT/2 and GOBL/2 folded into a single `gdt_over_2` field;
/// // M3 may need to split for restart atomicity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateVector {
    /// Position in ECI frame, metres.
    position: Vec3,
    /// Velocity in ECI frame, m/s.
    velocity: Vec3,
    /// Mission elapsed time of this state, centiseconds.
    time: Met,
    /// Half-step gravity acceleration × dt, m/s.
    ///
    /// Equals `a_gravity(position) * dt / 2` from the previous SERVICER cycle.
    /// Zero on first cycle (NORMLIZE initialises GDT/2 via CALCGRAV).
    ///
    /// AGC: `GDT/2 EQUALS PIPTIME +2`
    ///
    /// // M2 approximation: GDT/2 and GOBL/2 folded; M3 may need to split for
    /// // restart atomicity.
    gdt_over_2: Vec3,
}

impl StateVector {
    /// Construct a new state vector from position, velocity, and time.
    ///
    /// `gdt_over_2` is initialised to the zero vector; it is set by the
    /// SERVICER on the first call to CALCGRAV (NORMLIZE routine).
    ///
    /// AGC: NORMLIZE calls CALCGRAV to initialise GDT/2.
    ///
    /// Input: position in metres (ECI), velocity in m/s (ECI), time in centiseconds.
    pub fn new(position: Vec3, velocity: Vec3, time: Met) -> Self {
        Self {
            position,
            velocity,
            time,
            gdt_over_2: ZERO_VEC3,
        }
    }

    /// Construct with an explicit `gdt_over_2` (used when restoring from
    /// erasable memory after a restart, or when the integrator needs to
    /// preserve the predictor term).
    ///
    /// AGC: RELOADSV restores RN, VN, GDT/2 from erasable on restart.
    ///
    /// Input: position in metres (ECI), velocity in m/s (ECI),
    ///        time in centiseconds, gdt_over_2 in m/s.
    pub fn with_gdt(position: Vec3, velocity: Vec3, time: Met, gdt_over_2: Vec3) -> Self {
        Self {
            position,
            velocity,
            time,
            gdt_over_2,
        }
    }

    /// Position vector, ECI, metres.
    #[inline]
    pub fn position(&self) -> Vec3 {
        self.position
    }

    /// Velocity vector, ECI, m/s.
    #[inline]
    pub fn velocity(&self) -> Vec3 {
        self.velocity
    }

    /// Mission elapsed time of this state, centiseconds.
    #[inline]
    pub fn time(&self) -> Met {
        self.time
    }

    /// Half-step gravity term saved from the previous integration cycle, m/s.
    ///
    /// Zero on a freshly constructed state (before NORMLIZE).
    #[inline]
    pub fn gdt_over_2(&self) -> Vec3 {
        self.gdt_over_2
    }

    /// Return a new `StateVector` with position replaced; all other fields unchanged.
    #[inline]
    pub fn with_position(self, r: Vec3) -> Self {
        Self {
            position: r,
            ..self
        }
    }

    /// Return a new `StateVector` with velocity replaced; all other fields unchanged.
    #[inline]
    pub fn with_velocity(self, v: Vec3) -> Self {
        Self {
            velocity: v,
            ..self
        }
    }

    /// Return a new `StateVector` with time replaced; all other fields unchanged.
    #[inline]
    pub fn with_time(self, t: Met) -> Self {
        Self { time: t, ..self }
    }

    /// Return a new `StateVector` with `gdt_over_2` replaced.
    #[inline]
    pub fn with_gdt_over_2(self, gdt: Vec3) -> Self {
        Self {
            gdt_over_2: gdt,
            ..self
        }
    }

    /// Euclidean magnitude of the position vector, metres.
    ///
    /// Convenience wrapper around `math::linalg::norm`.
    /// Callers must ensure `|position| > R_MIN_GUARD` before computing gravity.
    #[inline]
    pub fn radius(&self) -> f64 {
        norm(&self.position)
    }

    /// Euclidean magnitude of the velocity vector, m/s.
    #[inline]
    pub fn speed(&self) -> f64 {
        norm(&self.velocity)
    }
}

/// The gravitational primary body for integration.
///
/// AGC: `MOONFLAG` bit in flag word 0 (`ERASABLE_ASSIGNMENTS.agc` `MOONFLAG = 003D`).
/// AGC: `PBODY` erasable register (`ERASABLE_ASSIGNMENTS.agc` `PBODY ERASE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrimaryBody {
    /// Earth-centred integration (MOONFLAG clear).
    Earth,
    /// Moon-centred integration (MOONFLAG set).
    Moon,
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Met;

    /// Test 1 — Round-trip accessor consistency.
    #[test]
    fn accessor_round_trip() {
        let r = [6_578_000.0_f64, 0.0, 0.0];
        let v = [0.0_f64, 7_784.0, 0.0];
        let state = StateVector::new(r, v, Met::from_centiseconds(0));

        assert_eq!(state.position(), r);
        assert_eq!(state.velocity(), v);
        assert_eq!(state.time(), Met(0));
        assert_eq!(state.gdt_over_2(), ZERO_VEC3);
        assert!(
            (state.radius() - 6_578_000.0).abs() < 1e-6,
            "radius={}",
            state.radius()
        );
        assert!(
            (state.speed() - 7_784.0).abs() < 1e-6,
            "speed={}",
            state.speed()
        );
    }

    /// Test 2 — Immutable builder pattern.
    #[test]
    fn builder_pattern() {
        let s0 = StateVector::new([1.0_f64, 0.0, 0.0], [0.0_f64, 1.0, 0.0], Met(100));
        let s1 = s0.with_position([2.0_f64, 0.0, 0.0]);
        // s0 is Copy, so it is unchanged
        assert_eq!(s0.position(), [1.0, 0.0, 0.0]);
        assert_eq!(s1.position(), [2.0, 0.0, 0.0]);
        assert_eq!(s1.velocity(), [0.0, 1.0, 0.0]); // velocity unchanged
        assert_eq!(s1.time(), Met(100)); // time unchanged
    }

    /// Test 3 — with_gdt constructor preserves gdt_over_2.
    #[test]
    fn with_gdt_constructor() {
        let gdt = [-9.8_f64, 0.0, 0.0];
        let sv = StateVector::with_gdt(
            [6_578_000.0_f64, 0.0, 0.0],
            [0.0_f64, 7784.0, 0.0],
            Met(0),
            gdt,
        );
        assert_eq!(sv.gdt_over_2(), gdt);
        // Round-trip via with_gdt_over_2
        let gdt2 = [0.0_f64, 1.0, 2.0];
        let sv2 = sv.with_gdt_over_2(gdt2);
        assert_eq!(sv2.gdt_over_2(), gdt2);
        assert_eq!(sv.gdt_over_2(), gdt); // original unchanged (Copy)
    }
}
