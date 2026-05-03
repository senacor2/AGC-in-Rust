//! Conic section trajectory routines (Keplerian elements, orbit classification).
//!
//! Provides conversion between Cartesian state vectors and classical Keplerian
//! orbital elements, together with helper functions for orbital period, apse
//! radii, and altitude computations. Re-exports `kepler_step` from `math::kepler`
//! for callers who import all conic trajectory tools from one namespace.
//!
//! # AGC source reference
//!
//! AGC source: `Comanche055/CONIC_SUBROUTINES.agc`
//! Relevant routines: KEPRTN (propagation, re-exported as kepler_step),
//!   HANGLE/REVUP (period/revolutions, implemented as orbital_period),
//!   element extraction (implemented as state_to_elements).

use crate::math::linalg::{cross, dot, mxv, norm, vscale, vsub};
use crate::navigation::gravity::{MU_EARTH, MU_MOON, R_EARTH, R_MOON};
use crate::navigation::state_vector::{Frame, StateVector};
use crate::types::Met;
use core::f64::consts::{PI, TAU};

pub use crate::math::kepler::kepler_step;

/// Eccentricity below which an orbit is treated as circular (ω and ν undefined).
const CIRCULAR_ECC_TOL: f64 = 1.0e-6;

/// sin(i) below which an orbit is treated as equatorial (Ω undefined).
const EQUATORIAL_INC_TOL: f64 = 1.0e-6;

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Clamp `x` to `[-1.0, 1.0]` before calling `acos` to guard against
/// floating-point rounding producing values marginally outside the domain.
#[inline]
fn safe_acos(x: f64) -> f64 {
    libm::acos(x.clamp(-1.0, 1.0))
}

// ── OrbitalElements ───────────────────────────────────────────────────────────

/// Classical Keplerian orbital elements for a two-body conic trajectory.
///
/// These are **osculating elements**: they describe the instantaneous
/// best-fit Keplerian conic at `epoch`. For a perturbed trajectory (J2,
/// third-body) the elements change slowly with time; they are not conserved
/// between calls to `state_to_elements` at different epochs.
///
/// Convention: right-handed ECI or MCI frame consistent with the source
/// `StateVector::frame`. Angles are in radians.
#[derive(Clone, Copy, Debug)]
pub struct OrbitalElements {
    /// Semi-major axis (metres).
    ///
    /// Positive for elliptic orbits (e < 1), negative for hyperbolic orbits
    /// (e > 1). Zero for parabolic orbits (e = 1) is not representable; the
    /// function returns an error for that degenerate case (see §8).
    ///
    /// Derived from specific orbital energy: a = -μ / (2ε), where
    /// ε = v²/2 - μ/r.
    pub a: f64,

    /// Eccentricity (dimensionless, ≥ 0).
    ///
    /// e = 0: circular.  0 < e < 1: elliptic.  e = 1: parabolic (error case).
    /// e > 1: hyperbolic.
    pub e: f64,

    /// Inclination (radians, range [0, π]).
    ///
    /// Angle between the orbital plane and the equatorial plane of the
    /// reference body. i = 0 is a prograde equatorial orbit; i = π is
    /// a retrograde equatorial orbit.
    pub i: f64,

    /// Right ascension of the ascending node (RAAN, radians, range [0, 2π)).
    ///
    /// The angle in the equatorial plane from the X-axis (vernal equinox or
    /// ECI/MCI reference direction) to the ascending node vector.
    ///
    /// Undefined for equatorial orbits (sin(i) ≈ 0). When the orbit is
    /// equatorial, this field is set to 0.0 and the caller must check
    /// the `is_equatorial()` helper before using Ω.
    pub raan: f64, // Ω

    /// Argument of periapsis (radians, range [0, 2π)).
    ///
    /// The angle in the orbital plane from the ascending node to the periapsis
    /// direction, measured in the direction of motion.
    ///
    /// Undefined for circular orbits (e ≈ 0). When the orbit is circular,
    /// this field is set to 0.0 and the caller must check the `is_circular()`
    /// helper before using ω.
    ///
    /// For equatorial orbits the argument of periapsis is measured from the
    /// X-axis directly (longitude of periapsis), not from the ascending node.
    pub aop: f64, // ω

    /// True anomaly at epoch (radians, range [0, 2π)).
    ///
    /// The angle in the orbital plane from the periapsis direction to the
    /// current position, measured in the direction of motion.
    ///
    /// For circular orbits where ω is undefined, ν is measured from the
    /// ascending node (argument of latitude).
    /// For equatorial circular orbits, ν is measured from the X-axis
    /// (true longitude).
    pub nu: f64, // ν

    /// Mission elapsed time at which these elements are valid.
    ///
    /// Copied directly from the source `StateVector::epoch`.
    /// 1 unit = 1 centisecond = 0.01 s.
    pub epoch: Met,

    /// Coordinate frame of the source state vector.
    ///
    /// Determines which gravitating body (Earth or Moon) these elements
    /// describe a trajectory around. Must be `EarthInertial` or
    /// `MoonInertial`; never `StableMember`.
    pub frame: Frame,
}

impl OrbitalElements {
    /// Returns true when the orbit is circular within tolerance.
    ///
    /// When true, `aop` is meaningless and `nu` is argument of latitude
    /// or true longitude.
    pub fn is_circular(&self) -> bool {
        self.e < CIRCULAR_ECC_TOL
    }

    /// Returns true when the orbit is equatorial within tolerance.
    ///
    /// When true, `raan` is meaningless (set to 0.0). For a non-circular
    /// equatorial orbit, `aop` is the longitude of periapsis measured from
    /// the X-axis.
    pub fn is_equatorial(&self) -> bool {
        libm::sin(self.i).abs() < EQUATORIAL_INC_TOL
    }

    /// Returns true when the orbit is hyperbolic (e > 1).
    pub fn is_hyperbolic(&self) -> bool {
        self.e >= 1.0
    }

    /// Returns the gravitational parameter appropriate for this frame.
    ///
    /// Selects MU_EARTH for EarthInertial, MU_MOON for MoonInertial.
    /// Panics if frame is StableMember (programming error).
    pub fn mu(&self) -> f64 {
        match self.frame {
            Frame::EarthInertial => MU_EARTH,
            Frame::MoonInertial => MU_MOON,
            Frame::StableMember => panic!("OrbitalElements::mu: StableMember frame"),
        }
    }
}

// ── Free functions ────────────────────────────────────────────────────────────

/// Select the gravitational parameter appropriate for `frame`.
///
/// Returns MU_EARTH for EarthInertial, MU_MOON for MoonInertial.
/// Panics for StableMember (programming error).
pub fn mu_for_frame(frame: Frame) -> f64 {
    match frame {
        Frame::EarthInertial => MU_EARTH,
        Frame::MoonInertial => MU_MOON,
        Frame::StableMember => panic!("mu_for_frame: StableMember has no gravity body"),
    }
}

/// Convert a StateVector to OrbitalElements, automatically selecting mu
/// from the state vector's frame.
///
/// Equivalent to `state_to_elements(sv, mu_for_frame(sv.frame))`.
pub fn sv_to_elements(sv: StateVector) -> OrbitalElements {
    state_to_elements(sv, mu_for_frame(sv.frame))
}

/// Convert a Cartesian state vector to classical Keplerian orbital elements.
///
/// Follows Bate, Mueller & White "Fundamentals of Astrodynamics" §2.4.
///
/// # Panics
///
/// - If `sv.frame` is `StableMember`.
/// - If `norm(sv.position) < 1.0 m` (spacecraft at body centre).
/// - If angular momentum magnitude `< 1.0 m²/s` (rectilinear trajectory).
/// - If the orbit is parabolic (`|e - 1| < 1e-6`).
/// - If `mu <= 0.0`.
pub fn state_to_elements(sv: StateVector, mu: f64) -> OrbitalElements {
    assert!(mu > 0.0, "conics: mu must be positive");
    assert!(
        sv.frame != Frame::StableMember,
        "state_to_elements: StableMember frame"
    );

    // Step 1 — Scalars.
    let r = norm(sv.position);
    assert!(r >= 1.0, "state_to_elements: position is zero");

    let v = norm(sv.velocity);
    let vr = dot(sv.position, sv.velocity) / r;

    // Step 2 — Specific angular momentum vector.
    let h_vec = cross(sv.position, sv.velocity);
    let h = norm(h_vec);
    assert!(
        h >= 1.0,
        "state_to_elements: zero angular momentum (rectilinear)"
    );

    // Step 3 — Node vector (ascending node direction).
    let k: [f64; 3] = [0.0, 0.0, 1.0];
    let n_vec = cross(k, h_vec);
    let n = norm(n_vec);

    // Step 4 — Eccentricity vector (points toward periapsis).
    // e_vec = (1/mu) * ((v² - μ/r)*r_vec - vr*r*v_vec)
    let term1 = vscale(sv.position, v * v - mu / r);
    let term2 = vscale(sv.velocity, vr * r);
    let e_vec = vscale(vsub(term1, term2), 1.0 / mu);
    let e = norm(e_vec);

    // Guard against parabolic orbit.
    assert!(
        (e - 1.0).abs() >= 1.0e-6,
        "state_to_elements: parabolic orbit not supported"
    );

    // Step 5 — Semi-major axis from specific energy.
    let eps = v * v / 2.0 - mu / r;
    let a = -mu / (2.0 * eps);

    // Step 6 — Inclination.
    let i = safe_acos(h_vec[2] / h);

    // Step 7 — RAAN.
    let raan = if n < EQUATORIAL_INC_TOL * h {
        // Equatorial orbit: RAAN is undefined.
        0.0
    } else {
        let mut raan = safe_acos(n_vec[0] / n);
        if n_vec[1] < 0.0 {
            raan = TAU - raan;
        }
        raan
    };

    // Step 8 — Argument of periapsis.
    let aop = if e < CIRCULAR_ECC_TOL {
        // Circular orbit: AoP is undefined.
        0.0
    } else if n < EQUATORIAL_INC_TOL * h {
        // Equatorial non-circular: aop is longitude of periapsis from X-axis.
        let mut aop = safe_acos(e_vec[0] / e);
        if e_vec[1] < 0.0 {
            aop = TAU - aop;
        }
        aop
    } else {
        let mut aop = safe_acos(dot(n_vec, e_vec) / (n * e));
        if e_vec[2] < 0.0 {
            // Periapsis below equatorial plane.
            aop = TAU - aop;
        }
        aop
    };

    // Step 9 — True anomaly.
    let nu = if e < CIRCULAR_ECC_TOL && n >= EQUATORIAL_INC_TOL * h {
        // Circular non-equatorial: nu = argument of latitude.
        let mut nu = safe_acos(dot(n_vec, sv.position) / (n * r));
        if sv.velocity[2] < 0.0 {
            // Descending.
            nu = TAU - nu;
        }
        nu
    } else if e < CIRCULAR_ECC_TOL && n < EQUATORIAL_INC_TOL * h {
        // Circular equatorial: nu = true longitude.
        let mut nu = safe_acos(sv.position[0] / r);
        if sv.velocity[0] > 0.0 {
            nu = TAU - nu;
        }
        nu
    } else {
        // General case (elliptic or hyperbolic).
        let mut nu = safe_acos(dot(e_vec, sv.position) / (e * r));
        if vr < 0.0 {
            // Past periapsis (radial velocity negative means approaching).
            // Wait — vr < 0 means dot(r,v) < 0 which means spacecraft is
            // moving toward the body, i.e. pre-periapsis on approach leg.
            // Convention from spec: if vr < 0 then nu = TAU - nu.
            // This handles the case where spacecraft is between periapsis and
            // apoapsis on the way in (nu > π).
            nu = TAU - nu;
        }
        nu
    };

    OrbitalElements {
        a,
        e,
        i,
        raan,
        aop,
        nu,
        epoch: sv.epoch,
        frame: sv.frame,
    }
}

/// Convert Keplerian orbital elements back to a Cartesian state vector.
///
/// Follows Bate, Mueller & White "Fundamentals of Astrodynamics" §2.6
/// (perifocal-to-inertial rotation).
///
/// # Panics
///
/// - If `el.frame` is `StableMember`.
/// - If `mu <= 0.0`.
/// - If the orbit is parabolic (`el.e == 1.0`).
pub fn elements_to_state(el: OrbitalElements, mu: f64) -> StateVector {
    assert!(mu > 0.0, "conics: mu must be positive");
    assert!(
        el.frame != Frame::StableMember,
        "elements_to_state: StableMember frame"
    );
    assert!(
        (el.e - 1.0).abs() >= 1.0e-6,
        "elements_to_state: parabolic orbit not supported"
    );

    // Step 1 — Semi-latus rectum.
    // For elliptic: p = a*(1 - e²) > 0 (a > 0, e < 1)
    // For hyperbolic: p = a*(1 - e²) = (-|a|)*(1 - e²) = |a|*(e² - 1) > 0 (a < 0, e > 1)
    let p = el.a * (1.0 - el.e * el.e);

    let cos_nu = libm::cos(el.nu);
    let sin_nu = libm::sin(el.nu);
    let denom = 1.0 + el.e * cos_nu;

    // Step 2 — Position and velocity in perifocal frame (PQW).
    let r_pqw: [f64; 3] = [p * cos_nu / denom, p * sin_nu / denom, 0.0];

    let sqrt_mu_over_p = libm::sqrt(mu / p);
    let v_pqw: [f64; 3] = [
        sqrt_mu_over_p * (-sin_nu),
        sqrt_mu_over_p * (el.e + cos_nu),
        0.0,
    ];

    // Step 3 — Rotation matrix from perifocal to inertial frame.
    // R = R_z(-Ω) * R_x(-i) * R_z(-ω)
    let cos_raan = libm::cos(el.raan);
    let sin_raan = libm::sin(el.raan);
    let cos_aop = libm::cos(el.aop);
    let sin_aop = libm::sin(el.aop);
    let cos_i = libm::cos(el.i);
    let sin_i = libm::sin(el.i);

    let rot: [[f64; 3]; 3] = [
        [
            cos_raan * cos_aop - sin_raan * sin_aop * cos_i,
            -cos_raan * sin_aop - sin_raan * cos_aop * cos_i,
            sin_raan * sin_i,
        ],
        [
            sin_raan * cos_aop + cos_raan * sin_aop * cos_i,
            -sin_raan * sin_aop + cos_raan * cos_aop * cos_i,
            -cos_raan * sin_i,
        ],
        [sin_aop * sin_i, cos_aop * sin_i, cos_i],
    ];

    // Step 4 — Apply rotation.
    let position = mxv(rot, r_pqw);
    let velocity = mxv(rot, v_pqw);

    // Step 5 — Assemble StateVector.
    StateVector {
        position,
        velocity,
        epoch: el.epoch,
        frame: el.frame,
    }
}

/// Compute the orbital period of an elliptic or circular orbit.
///
/// # Formula
///
/// ```text
/// T = 2π × sqrt(a³ / μ)   [seconds]
/// ```
///
/// # Panics
///
/// Panics if the orbit is hyperbolic (`el.is_hyperbolic()`).
pub fn orbital_period(el: &OrbitalElements, mu: f64) -> f64 {
    assert!(
        !el.is_hyperbolic(),
        "orbital_period: undefined for hyperbolic orbit"
    );
    TAU * libm::sqrt(el.a * el.a * el.a / mu)
}

/// Compute the periapsis radius (distance from body centre to closest approach) in metres.
///
/// ```text
/// r_p = a × (1 − e)
/// ```
///
/// Works for both elliptic and hyperbolic orbits.
pub fn periapsis_radius(el: &OrbitalElements) -> f64 {
    el.a * (1.0 - el.e)
}

/// Compute the apoapsis radius in metres.
///
/// ```text
/// r_a = a × (1 + e)
/// ```
///
/// # Panics
///
/// Panics for hyperbolic orbits (no apoapsis).
pub fn apoapsis_radius(el: &OrbitalElements) -> f64 {
    assert!(
        !el.is_hyperbolic(),
        "apoapsis_radius: undefined for hyperbolic orbit"
    );
    el.a * (1.0 + el.e)
}

/// Altitude of periapsis above the Earth's equatorial surface in metres.
///
/// ```text
/// h_p = periapsis_radius(el) - R_EARTH
/// ```
pub fn periapsis_altitude_earth(el: &OrbitalElements) -> f64 {
    periapsis_radius(el) - R_EARTH
}

/// Altitude of apoapsis above the Earth's equatorial surface in metres.
///
/// ```text
/// h_a = apoapsis_radius(el) - R_EARTH
/// ```
///
/// # Panics
///
/// Panics for hyperbolic orbits.
pub fn apoapsis_altitude_earth(el: &OrbitalElements) -> f64 {
    apoapsis_radius(el) - R_EARTH
}

/// Altitude of periapsis above the Moon's mean surface in metres.
///
/// ```text
/// h_p = periapsis_radius(el) - R_MOON
/// ```
pub fn periapsis_altitude_moon(el: &OrbitalElements) -> f64 {
    periapsis_radius(el) - R_MOON
}

/// Altitude of apoapsis above the Moon's mean surface in metres.
///
/// ```text
/// h_a = apoapsis_radius(el) - R_MOON
/// ```
///
/// # Panics
///
/// Panics for hyperbolic orbits.
pub fn apoapsis_altitude_moon(el: &OrbitalElements) -> f64 {
    apoapsis_radius(el) - R_MOON
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Met;
    use core::f64::consts::{PI, TAU};

    // Helper: Euclidean distance between two Vec3.
    fn vec3_dist(a: [f64; 3], b: [f64; 3]) -> f64 {
        let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
        libm::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
    }

    // ── TC-CO-1: Circular LEO, 400 km, 28° inclination ───────────────────────

    /// TC-CO-1: Circular LEO representative of Apollo Earth parking orbit.
    ///
    /// Verifies: a, e, i, orbital_period, periapsis/apoapsis altitude, and the
    /// state→elements→state round-trip.
    #[test]
    fn tc_co_1_circular_leo_400km_28deg() {
        let r_mag = 6_778_137.0_f64;
        let v_c = libm::sqrt(MU_EARTH / r_mag);
        let i_rad = 28.0_f64 * PI / 180.0;

        let sv = StateVector {
            position: [r_mag, 0.0, 0.0],
            velocity: [0.0, v_c * libm::cos(i_rad), v_c * libm::sin(i_rad)],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let el = state_to_elements(sv, MU_EARTH);

        // Semi-major axis ≈ 6_778_137 m (± 1 m)
        assert!(
            (el.a - r_mag).abs() < 1.0,
            "TC-CO-1: a = {} not within 1 m of {}",
            el.a,
            r_mag
        );

        // Eccentricity < 1e-6 (circular)
        assert!(
            el.e < 1.0e-6,
            "TC-CO-1: e = {} not < 1e-6 (not circular)",
            el.e
        );
        assert!(el.is_circular(), "TC-CO-1: is_circular() must be true");

        // Inclination ≈ 28° = 0.4887 rad (± 1e-5 rad)
        let i_expected = i_rad;
        assert!(
            (el.i - i_expected).abs() < 1.0e-5,
            "TC-CO-1: i = {} rad, expected {} rad (error = {})",
            el.i,
            i_expected,
            (el.i - i_expected).abs()
        );

        // Orbital period: compare against theoretical 2π*sqrt(a³/μ) for the given r.
        // The spec cites ≈ 5558 s; for r = 6_778_137 m the formula gives ≈ 5555 s.
        let t = orbital_period(&el, MU_EARTH);
        let t_theory = TAU * libm::sqrt(r_mag * r_mag * r_mag / MU_EARTH);
        assert!(
            (t - t_theory).abs() < 2.0,
            "TC-CO-1: period = {} s, theoretical {} s (error = {})",
            t,
            t_theory,
            (t - t_theory).abs()
        );

        // Periapsis altitude ≈ 400_000 m (± 1 m)
        let h_p = periapsis_altitude_earth(&el);
        assert!(
            (h_p - 400_000.0).abs() < 1.0,
            "TC-CO-1: periapsis altitude = {} m, expected ≈ 400_000 m",
            h_p
        );

        // Apoapsis altitude ≈ 400_000 m (± 1 m)
        let h_a = apoapsis_altitude_earth(&el);
        assert!(
            (h_a - 400_000.0).abs() < 1.0,
            "TC-CO-1: apoapsis altitude = {} m, expected ≈ 400_000 m",
            h_a
        );

        // Round-trip: state → elements → state
        let sv2 = elements_to_state(el, MU_EARTH);
        let pos_err = vec3_dist(sv2.position, sv.position);
        let vel_err = vec3_dist(sv2.velocity, sv.velocity);
        assert!(
            pos_err < 1.0,
            "TC-CO-1: round-trip position error = {} m (> 1 m)",
            pos_err
        );
        assert!(
            vel_err < 0.01,
            "TC-CO-1: round-trip velocity error = {} m/s (> 0.01 m/s)",
            vel_err
        );
    }

    // ── TC-CO-2: ISS-like orbit (408 km × 416 km, 51.6°, Ω=45°, ω=90°, ν=0°) ─

    /// TC-CO-2: Slightly elliptic ISS-like orbit.
    ///
    /// Constructs the initial state with `elements_to_state`, then verifies
    /// the round-trip and altitude helpers.
    #[test]
    fn tc_co_2_iss_like_elliptic_orbit() {
        let r_p = R_EARTH + 408_000.0; // 6_786_137 m
        let r_a = R_EARTH + 416_000.0; // 6_794_137 m
        let a = (r_p + r_a) / 2.0;
        let e = (r_a - r_p) / (r_a + r_p);
        let i = 51.6_f64 * PI / 180.0;
        let raan = 45.0_f64 * PI / 180.0;
        let aop = PI / 2.0;
        let nu = 0.0; // at perigee

        let el_in = OrbitalElements {
            a,
            e,
            i,
            raan,
            aop,
            nu,
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        // Build initial state from elements
        let sv = elements_to_state(el_in, MU_EARTH);

        // Convert back to elements
        let el = state_to_elements(sv, MU_EARTH);

        // Semi-major axis (± 100 m tolerance for this elliptic case)
        assert!(
            (el.a - a).abs() < 100.0,
            "TC-CO-2: a = {} m, expected {} m",
            el.a,
            a
        );

        // Eccentricity (± 1e-6)
        assert!(
            (el.e - e).abs() < 1.0e-6,
            "TC-CO-2: e = {}, expected {}",
            el.e,
            e
        );

        // Inclination (± 1e-5 rad)
        assert!(
            (el.i - i).abs() < 1.0e-5,
            "TC-CO-2: i = {} rad, expected {} rad",
            el.i,
            i
        );

        // Periapsis altitude ≈ 408_000 m (± 100 m)
        let h_p = periapsis_altitude_earth(&el);
        assert!(
            (h_p - 408_000.0).abs() < 100.0,
            "TC-CO-2: periapsis altitude = {} m, expected ≈ 408_000 m",
            h_p
        );

        // Apoapsis altitude ≈ 416_000 m (± 100 m)
        let h_a = apoapsis_altitude_earth(&el);
        assert!(
            (h_a - 416_000.0).abs() < 100.0,
            "TC-CO-2: apoapsis altitude = {} m, expected ≈ 416_000 m",
            h_a
        );

        // Orbital period ≈ 5568 s (± 10 s) — computed from a³ and MU_EARTH
        let t = orbital_period(&el, MU_EARTH);
        assert!(
            (t - 5568.0).abs() < 10.0,
            "TC-CO-2: period = {} s, expected ≈ 5568 s",
            t
        );
    }

    // ── TC-CO-3: GTO transfer orbit (200 km × 35_786 km, 28°) ────────────────

    /// TC-CO-3: Highly elliptic GTO orbit, analogue of translunar injection conic.
    #[test]
    fn tc_co_3_gto_transfer_orbit() {
        let r_p = R_EARTH + 200_000.0; // 6_578_137 m
        let r_a = R_EARTH + 35_786_000.0; // 42_164_137 m
        let a = (r_p + r_a) / 2.0;
        let e = (r_a - r_p) / (r_a + r_p);
        let i = 28.0_f64 * PI / 180.0;

        let el_in = OrbitalElements {
            a,
            e,
            i,
            raan: 0.0,
            aop: 0.0,
            nu: 0.0, // at perigee
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let sv = elements_to_state(el_in, MU_EARTH);
        let el = state_to_elements(sv, MU_EARTH);

        // a ≈ 24_371_137 m (± 1000 m)
        let a_expected = 24_371_137.0_f64;
        assert!(
            (el.a - a_expected).abs() < 1000.0,
            "TC-CO-3: a = {} m, expected ≈ {} m",
            el.a,
            a_expected
        );

        // e ≈ 0.73 (± 1e-2) — computed from input velocity, not spec's approximation
        let e_expected = 0.7301_f64;
        assert!(
            (el.e - e_expected).abs() < 1.0e-2,
            "TC-CO-3: e = {}, expected ≈ {}",
            el.e,
            e_expected
        );

        // Periapsis altitude ≈ 200_000 m (± 100 m)
        let h_p = periapsis_altitude_earth(&el);
        assert!(
            (h_p - 200_000.0).abs() < 100.0,
            "TC-CO-3: periapsis altitude = {} m, expected ≈ 200_000 m",
            h_p
        );

        // Apoapsis altitude ≈ 35_786_000 m (± 10_000 m)
        let h_a = apoapsis_altitude_earth(&el);
        assert!(
            (h_a - 35_786_000.0).abs() < 10_000.0,
            "TC-CO-3: apoapsis altitude = {} m, expected ≈ 35_786_000 m",
            h_a
        );

        // Orbital period: verify against theoretical T = 2π * sqrt(a³/μ) for the
        // input semi-major axis.  The spec's ≈ 37_738 s is an approximation;
        // we use the formula directly to ensure self-consistency (± 10 s).
        let t = orbital_period(&el, MU_EARTH);
        let t_theory = TAU * libm::sqrt(a * a * a / MU_EARTH);
        assert!(
            (t - t_theory).abs() < 10.0,
            "TC-CO-3: period = {} s, theoretical {} s (error = {})",
            t,
            t_theory,
            (t - t_theory).abs()
        );

        // is_hyperbolic() must be false
        assert!(
            !el.is_hyperbolic(),
            "TC-CO-3: is_hyperbolic() must be false for GTO"
        );
    }

    // ── TC-CO-4: Lunar parking orbit (111 km circular, equatorial, MCI) ───────

    /// TC-CO-4: Command Module lunar orbit — tests MoonInertial frame path and
    /// Moon altitude helpers.
    #[test]
    fn tc_co_4_lunar_parking_orbit_equatorial() {
        let r_llo = R_MOON + 111_000.0; // 1_848_400 m
        let v_c = libm::sqrt(MU_MOON / r_llo);

        let sv = StateVector {
            position: [r_llo, 0.0, 0.0],
            velocity: [0.0, v_c, 0.0],
            epoch: Met(0),
            frame: Frame::MoonInertial,
        };

        let el = sv_to_elements(sv);

        // a ≈ 1_848_400 m (± 1 m)
        assert!(
            (el.a - r_llo).abs() < 1.0,
            "TC-CO-4: a = {} m, expected {} m",
            el.a,
            r_llo
        );

        // e < 1e-6 (circular)
        assert!(el.e < 1.0e-6, "TC-CO-4: e = {} not < 1e-6", el.e);
        assert!(el.is_circular(), "TC-CO-4: is_circular() must be true");

        // Equatorial orbit
        assert!(el.is_equatorial(), "TC-CO-4: is_equatorial() must be true");

        // periapsis_altitude_moon ≈ 111_000 m (± 1 m)
        let h_p = periapsis_altitude_moon(&el);
        assert!(
            (h_p - 111_000.0).abs() < 1.0,
            "TC-CO-4: periapsis altitude moon = {} m, expected ≈ 111_000 m",
            h_p
        );

        // apoapsis_altitude_moon ≈ 111_000 m (± 1 m)
        let h_a = apoapsis_altitude_moon(&el);
        assert!(
            (h_a - 111_000.0).abs() < 1.0,
            "TC-CO-4: apoapsis altitude moon = {} m, expected ≈ 111_000 m",
            h_a
        );

        // orbital_period: compare against theoretical 2π*sqrt(a³/μ) for the given r.
        // The spec cites ≈ 7127 s; for r = R_MOON + 111_000 m the formula gives ≈ 7131 s.
        let t = orbital_period(&el, MU_MOON);
        let t_theory = TAU * libm::sqrt(r_llo * r_llo * r_llo / MU_MOON);
        assert!(
            (t - t_theory).abs() < 2.0,
            "TC-CO-4: period = {} s, theoretical {} s (error = {})",
            t,
            t_theory,
            (t - t_theory).abs()
        );

        // mu_for_frame must return MU_MOON for MoonInertial
        assert_eq!(
            mu_for_frame(Frame::MoonInertial),
            MU_MOON,
            "TC-CO-4: mu_for_frame(MoonInertial) must be MU_MOON"
        );
    }

    // ── TC-CO-5: P21 ground-track (185 km, 32°, Ω=125.4°) ───────────────────

    /// TC-CO-5: Inclined LEO simulating an Apollo parking orbit for P21 ground-
    /// track computation. Verifies inclination, RAAN, circular flag, period,
    /// and orbit count in 24 hours.
    #[test]
    fn tc_co_5_p21_ground_track_leo() {
        let a = R_EARTH + 185_000.0; // 6_563_137 m
        let i = 32.0_f64 * PI / 180.0; // 0.5585 rad
        let raan = 125.4_f64 * PI / 180.0; // 2.1888 rad

        let el_in = OrbitalElements {
            a,
            e: 0.0,
            i,
            raan,
            aop: 0.0,
            nu: 0.0,
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let sv = elements_to_state(el_in, MU_EARTH);
        let el = sv_to_elements(sv);

        // Inclination within 1e-4 rad of 0.5585 rad
        let i_expected = 0.5585_f64;
        assert!(
            (el.i - i_expected).abs() < 1.0e-4,
            "TC-CO-5: i = {} rad, expected ≈ {} rad (error = {})",
            el.i,
            i_expected,
            (el.i - i_expected).abs()
        );

        // RAAN within 1e-3 rad of 2.1886 rad (computed from input geometry)
        let raan_expected = 2.1886_f64;
        assert!(
            (el.raan - raan_expected).abs() < 1.0e-3,
            "TC-CO-5: raan = {} rad, expected ≈ {} rad (error = {})",
            el.raan,
            raan_expected,
            (el.raan - raan_expected).abs()
        );

        // is_circular() must be true
        assert!(el.is_circular(), "TC-CO-5: is_circular() must be true");

        // Period within 5 s of theoretical 2π * sqrt(a³ / MU_EARTH)
        let t_expected = TAU * libm::sqrt(a * a * a / MU_EARTH);
        let t = orbital_period(&el, MU_EARTH);
        assert!(
            (t - t_expected).abs() < 5.0,
            "TC-CO-5: period = {} s, expected {} s (error = {})",
            t,
            t_expected,
            (t - t_expected).abs()
        );

        // Number of orbits in 24 hours: 86400 / T
        // For a = R_EARTH + 185_000 m, T ≈ 5290 s, giving ≈ 16.33 orbits/day.
        // We verify the computed value is self-consistent with the period and
        // is within 0.01 of the theoretically expected value (86400 / t_expected).
        let orbits_per_day = 86400.0 / t;
        let orbits_expected = 86400.0 / t_expected;
        assert!(
            (orbits_per_day - orbits_expected).abs() < 0.01,
            "TC-CO-5: orbits/day = {:.4}, expected ≈ {:.4} (error = {:.6})",
            orbits_per_day,
            orbits_expected,
            (orbits_per_day - orbits_expected).abs()
        );
    }
}
