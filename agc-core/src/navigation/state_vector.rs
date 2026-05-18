use crate::navigation::time::OMEGA_EARTH;
use crate::types::{Met, Vec3};

/// Coordinate frame in which a state vector is expressed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Frame {
    /// Earth-centered inertial (ECI).
    ///
    /// Used during Earth orbit, trans-lunar coast before the sphere of influence,
    /// and trans-Earth coast after the SOI crossing on the return leg.
    /// Origin: Earth's centre of mass.
    /// Axes: non-rotating; X toward vernal equinox, Z toward North Celestial Pole.
    EarthInertial,

    /// Moon-centered inertial (MCI).
    ///
    /// Used when the spacecraft is within the Moon's sphere of influence:
    /// lunar orbit, descent, ascent, transearth injection, and cislunar coast
    /// after SOI crossing.
    /// Origin: Moon's centre of mass.
    /// Axes: parallel to ECI axes at the reference epoch (non-rotating).
    MoonInertial,

    /// Stable-member (IMU platform) frame.
    ///
    /// The frame of the gyroscopically stabilized platform. PIPA accelerometer
    /// counts are produced in this frame and must be rotated to an inertial frame
    /// via REFSMMAT before being applied to the state vector.
    /// This frame should not appear on a stored `StateVector`; it is used only
    /// transiently within the SERVICER loop.
    StableMember,
}

/// Position and velocity of a vehicle at a given epoch.
#[derive(Clone, Copy, Debug)]
pub struct StateVector {
    /// Position in metres, expressed in `frame`.
    ///
    /// Components: `[x, y, z]` with origin at the body specified by `frame`.
    /// Scale: SI metres (`f64`). AGC fixed-point scale: B+28 m (1 DP LSB ≈ 1 m).
    pub position: Vec3,

    /// Velocity in metres per second, expressed in `frame`.
    ///
    /// Components: `[vx, vy, vz]`.
    /// Scale: SI m/s (`f64`). AGC fixed-point scale: B+7 m/s (1 DP LSB ≈ 7.6×10⁻⁴ m/s).
    pub velocity: Vec3,

    /// Mission elapsed time at which this state is valid.
    ///
    /// Corresponds to AGC `TEPHEM` (the epoch of the `RN`/`VN` pair).
    /// One unit = 1 centisecond = 0.01 s.
    pub epoch: Met,

    /// Coordinate frame in which `position` and `velocity` are expressed.
    ///
    /// Must be consistent with the primary gravitating body: `EarthInertial`
    /// pairs with `navigation::gravity::earth_gravity`; `MoonInertial` pairs
    /// with `navigation::gravity::moon_gravity`.
    pub frame: Frame,
}

impl StateVector {
    /// A zeroed state vector in the Earth inertial frame at MET = 0.
    ///
    /// Used to initialize fields in `AgcState` at startup and after FRESH START.
    /// The zero position places the origin at Earth's centre, which is not a
    /// physically reachable spacecraft position, so any code path that uses
    /// `ZERO` for real navigation has a bug. Callers must not use `ZERO` as
    /// a valid state without first setting all fields to meaningful values.
    pub const ZERO: Self = Self {
        position: [0.0; 3],
        velocity: [0.0; 3],
        epoch: Met(0),
        frame: Frame::EarthInertial,
    };

    /// Check invariants in debug builds.
    /// Panics in debug mode if any invariant is violated.
    /// No-op in release mode.
    pub fn debug_assert_valid(&self) {
        debug_assert!(
            self.position.iter().all(|x| x.is_finite()),
            "StateVector position contains NaN or Inf"
        );
        debug_assert!(
            self.velocity.iter().all(|v| v.is_finite()),
            "StateVector velocity contains NaN or Inf"
        );
        debug_assert!(
            self.frame == Frame::EarthInertial || self.frame == Frame::MoonInertial,
            "StateVector stored with StableMember frame"
        );
    }
}

// ── Inertial ↔ Earth-fixed transforms ──────────────────────────────────────────
//
// The Earth-fixed (ECEF) frame is transient — we never store a `StateVector`
// in it. These helpers do the rotation on the fly. The angle `gha_rad`
// (Greenwich Hour Angle of Aries) comes from
// `navigation::time::met_to_gha(t, state.gha_epoch_rad)`.
//
// Convention: the ECI→ECEF rotation is `Rz(+gha)` — at `gha = π/2` the inertial
// x-axis points along the -ECEF y-axis (Earth has rotated east by 90°). The
// inverse `Rz(-gha)` rotates ECEF→ECI.
//
// Spec: specs/gmst-ecef-plan.md §5; AGC source LAT-LONG_SUBROUTINES.agc.

/// Rotate an inertial (ECI) position vector to the Earth-fixed (ECEF) frame.
///
/// Applies `Rz(+gha_rad)`. At `gha = 0` this is the identity. At `gha = π/2`
/// the input `[1, 0, 0]` maps to `[0, -1, 0]`.
pub fn inertial_to_earth_fixed(pos: Vec3, gha_rad: f64) -> Vec3 {
    let c = libm::cos(gha_rad);
    let s = libm::sin(gha_rad);
    [
        pos[0] * c + pos[1] * s,
        -pos[0] * s + pos[1] * c,
        pos[2],
    ]
}

/// Rotate an Earth-fixed (ECEF) position vector to the inertial (ECI) frame.
///
/// Applies `Rz(-gha_rad)`. Inverse of [`inertial_to_earth_fixed`].
pub fn earth_fixed_to_inertial(pos: Vec3, gha_rad: f64) -> Vec3 {
    let c = libm::cos(gha_rad);
    let s = libm::sin(gha_rad);
    [
        pos[0] * c - pos[1] * s,
        pos[0] * s + pos[1] * c,
        pos[2],
    ]
}

/// Rotate an inertial velocity to the Earth-fixed frame.
///
/// `pos` and `vel` are the ECI position and velocity respectively. The result
/// is the velocity as observed in the rotating (Earth-fixed) frame, which
/// requires subtracting the Earth-rotation term `ω × r` before rotating.
pub fn inertial_to_earth_fixed_vel(pos: Vec3, vel: Vec3, gha_rad: f64) -> Vec3 {
    let v_rel = [
        vel[0] + OMEGA_EARTH * pos[1],
        vel[1] - OMEGA_EARTH * pos[0],
        vel[2],
    ];
    inertial_to_earth_fixed(v_rel, gha_rad)
}

/// Rotate an Earth-fixed velocity to the inertial frame.
///
/// `pos` and `vel` are the ECEF position and velocity respectively. The result
/// is the velocity as observed in the inertial frame, which adds back the
/// Earth-rotation term `ω × r_eci` after rotating.
pub fn earth_fixed_to_inertial_vel(pos: Vec3, vel: Vec3, gha_rad: f64) -> Vec3 {
    let r_eci = earth_fixed_to_inertial(pos, gha_rad);
    let v_rot = earth_fixed_to_inertial(vel, gha_rad);
    [
        v_rot[0] - OMEGA_EARTH * r_eci[1],
        v_rot[1] + OMEGA_EARTH * r_eci[0],
        v_rot[2],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-SV-1: Construction and field access.
    ///
    /// Verify that a `StateVector` constructed from explicit field values
    /// stores and retrieves all components correctly.
    #[test]
    fn tc_sv_1_construction_and_field_access() {
        let sv = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7_546.05, 0.0],
            epoch: Met(100_000), // 1000 s MET
            frame: Frame::EarthInertial,
        };
        assert_eq!(sv.position[0], 7_000_000.0);
        assert_eq!(sv.position[1], 0.0);
        assert_eq!(sv.position[2], 0.0);
        assert_eq!(sv.velocity[1], 7_546.05);
        assert_eq!(sv.epoch, Met(100_000));
        assert_eq!(sv.frame, Frame::EarthInertial);
    }

    /// TC-SV-2: Frame annotation correctness — ECI and MCI distinct.
    ///
    /// Verify that `Frame::EarthInertial` and `Frame::MoonInertial` are not
    /// equal, that a state vector retains the assigned frame, and that the
    /// `Copy` derive preserves the frame.
    #[test]
    fn tc_sv_2_frame_annotation_and_copy_semantics() {
        let sv_eci = StateVector {
            frame: Frame::EarthInertial,
            ..StateVector::ZERO
        };
        let sv_mci = StateVector {
            frame: Frame::MoonInertial,
            ..StateVector::ZERO
        };

        assert_ne!(sv_eci.frame, sv_mci.frame);

        // Copy semantics: assignment does not alias
        let sv_copy = sv_eci;
        assert_eq!(sv_copy.frame, Frame::EarthInertial);
    }

    /// TC-SV-3: Low Earth Orbit sanity check (ISS-like orbit).
    ///
    /// Verify that a known-good LEO state vector has a position norm above
    /// Earth's surface and below GEO, and that its velocity is within 1% of
    /// the theoretical circular velocity at that radius.
    #[test]
    fn tc_sv_3_leo_orbit_sanity_check() {
        let sv = StateVector {
            position: [6_781_000.0, 0.0, 0.0],
            velocity: [0.0, 7_660.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let r = (sv.position[0].powi(2) + sv.position[1].powi(2) + sv.position[2].powi(2)).sqrt();
        assert!(r > 6_371_000.0, "Position inside Earth");
        assert!(r < 42_164_000.0, "Position beyond GEO");

        let mu: f64 = 3.986_004_418e14;
        let v_circular = (mu / r).sqrt();
        let v_actual =
            (sv.velocity[0].powi(2) + sv.velocity[1].powi(2) + sv.velocity[2].powi(2)).sqrt();
        let relative_error = (v_actual - v_circular).abs() / v_circular;
        assert!(
            relative_error < 0.01,
            "Velocity deviates from circular by more than 1%: relative_error = {relative_error}"
        );
    }

    /// TC-SV-4: Lunar orbit state vector.
    ///
    /// Verify that a realistic lunar orbit state vector is stored in
    /// `MoonInertial` frame and that the velocity is within 1% of the
    /// theoretical circular velocity at that radius.
    #[test]
    fn tc_sv_4_lunar_orbit_state_vector() {
        let sv = StateVector {
            position: [1_837_400.0, 0.0, 0.0],
            velocity: [0.0, 1_633.0, 0.0],
            epoch: Met(8_640_000), // 1 day into mission
            frame: Frame::MoonInertial,
        };
        assert_eq!(sv.frame, Frame::MoonInertial);

        let r = sv.position[0].abs(); // on-axis, simplified
        assert!(r > 1_737_400.0, "Position inside Moon");
        assert!(r < 2_000_000.0, "Position unrealistically far from Moon");

        let mu_moon: f64 = 4.902_800_118e12;
        let v_circular = (mu_moon / r).sqrt();
        let v_actual = sv.velocity[1].abs();
        let relative_error = (v_actual - v_circular).abs() / v_circular;
        assert!(
            relative_error < 0.01,
            "Lunar orbit velocity error > 1%: relative_error = {relative_error}"
        );
    }

    /// TC-SV-5: State vector at sphere-of-influence boundary.
    ///
    /// Verify that the `Frame` field correctly differentiates states at the
    /// same physical location depending on which side of the SOI the spacecraft
    /// is on, and that both representations satisfy the relevant position-norm
    /// invariants.
    #[test]
    fn tc_sv_5_sphere_of_influence_boundary() {
        // ECI frame: spacecraft 318,000 km from Earth (just inside SOI from Earth side)
        let sv_eci = StateVector {
            position: [3.18e8, 0.0, 0.0], // 318,000 km along X
            velocity: [0.0, 830.0, 0.0],  // approximate cislunar velocity
            epoch: Met(25_920_000),       // ~3 days MET
            frame: Frame::EarthInertial,
        };

        // MCI frame: same location expressed from Moon's centre
        // Moon is at ~384,400 km from Earth on X-axis at this epoch (simplified)
        // so Moon-relative position ≈ 318,000 - 384,400 = -66,400 km
        let sv_mci = StateVector {
            position: [-6.64e7, 0.0, 0.0], // 66,400 km from Moon, opposite direction
            velocity: [0.0, 830.0 - 1022.0, 0.0], // relative to Moon's ~1022 m/s orbital velocity
            epoch: Met(25_920_000),
            frame: Frame::MoonInertial,
        };

        assert_eq!(sv_eci.frame, Frame::EarthInertial);
        assert_eq!(sv_mci.frame, Frame::MoonInertial);
        assert_ne!(sv_eci.frame, sv_mci.frame);

        // ECI norm: cislunar distance, must be above Earth radius
        let r_eci = sv_eci.position[0].abs();
        assert!(r_eci > 6_371_000.0, "ECI position inside Earth");
        assert!(r_eci < 4.0e8, "ECI position beyond Earth-Moon distance");

        // MCI norm: should be near the SOI radius (~66,100 km)
        let r_mci = sv_mci.position[0].abs();
        assert!(r_mci > 1_737_400.0, "MCI position inside Moon");
        assert!(r_mci < 1.0e8, "MCI position unrealistically large");
    }

    /// TC-SV-6: ZERO constant properties.
    ///
    /// Verify the guaranteed properties of `StateVector::ZERO`.
    #[test]
    fn tc_sv_6_zero_constant_properties() {
        let z = StateVector::ZERO;
        assert_eq!(z.position, [0.0_f64; 3]);
        assert_eq!(z.velocity, [0.0_f64; 3]);
        assert_eq!(z.epoch, Met(0));
        assert_eq!(z.frame, Frame::EarthInertial);

        // ZERO must be Copy (should compile without .clone())
        let _z2 = z;
        let _z3 = z;
    }

    /// TC-SV-7: AGC fixed-point round-trip (velocity encoding).
    ///
    /// Verify that a known AGC double-precision velocity word pair converts
    /// to the expected `f64` m/s value within one LSB.
    ///
    /// Encoding: B+7 (scale factor 2^7 = 128 m/s).
    /// `velocity_mps = w_hi × 2^-7 + w_lo × 2^-21`
    #[test]
    fn tc_sv_7_agc_fixed_point_velocity_round_trip() {
        // Convert a representative AGC DP pair to f64 velocity
        let w_hi: i16 = 11100;
        let w_lo: i16 = 0;
        let velocity_mps = (w_hi as f64) * 2.0_f64.powi(-7) + (w_lo as f64) * 2.0_f64.powi(-21);
        let expected = 86.71875_f64;
        assert!(
            (velocity_mps - expected).abs() < 1e-6,
            "Velocity conversion error: got {velocity_mps}, expected {expected}"
        );
    }

    // ── Frame transform tests ──────────────────────────────────────────────────

    const R_EARTH_TEST: f64 = 6_371_000.0;

    /// TC-FRAME-1: `inertial_to_earth_fixed` at `gha = 0` is the identity.
    #[test]
    fn tc_frame_1_identity_at_zero_gha() {
        let pos = [1_234_567.0, -987_654.0, 6_543_210.0];
        let out = inertial_to_earth_fixed(pos, 0.0);
        assert!((out[0] - pos[0]).abs() < 1e-9);
        assert!((out[1] - pos[1]).abs() < 1e-9);
        assert!((out[2] - pos[2]).abs() < 1e-9);
    }

    /// TC-FRAME-2: at `gha = π/2`, the unit X axis rotates to `[0, -1, 0]`.
    #[test]
    fn tc_frame_2_quarter_revolution() {
        let out = inertial_to_earth_fixed([1.0, 0.0, 0.0], core::f64::consts::FRAC_PI_2);
        assert!(out[0].abs() < 1e-15, "x should be 0, got {}", out[0]);
        assert!((out[1] - -1.0).abs() < 1e-15, "y should be -1, got {}", out[1]);
        assert!(out[2].abs() < 1e-15, "z should be 0, got {}", out[2]);
    }

    /// TC-FRAME-3: round-trip ECI → ECEF → ECI returns the input within 1e-14.
    #[test]
    fn tc_frame_3_position_round_trip() {
        let pos = [4_321_000.0, -2_109_876.0, 5_432_100.0];
        let gha = 1.234_567_8;
        let ef = inertial_to_earth_fixed(pos, gha);
        let back = earth_fixed_to_inertial(ef, gha);
        for i in 0..3 {
            let err = (back[i] - pos[i]).abs();
            assert!(err < 1e-8, "round-trip error component {i}: {err}");
            // Relative error within 1e-14 of the original magnitude.
            let rel = err / pos[i].abs().max(1.0);
            assert!(rel < 1e-14, "relative round-trip error {i}: {rel}");
        }
    }

    /// TC-FRAME-4: a point stationary in ECEF on the equator at `r = R_earth`
    /// has ECI velocity magnitude `OMEGA_EARTH * R_earth ≈ 465 m/s`. At
    /// `gha = 0` the direction is along ECI +y.
    #[test]
    fn tc_frame_4_stationary_ecef_point_eci_velocity() {
        let r_ef = [R_EARTH_TEST, 0.0, 0.0];
        let v_ef = [0.0, 0.0, 0.0];
        let v_eci = earth_fixed_to_inertial_vel(r_ef, v_ef, 0.0);

        let expected_mag = OMEGA_EARTH * R_EARTH_TEST;
        assert!(v_eci[0].abs() < 1e-9, "v_eci x should be ~0, got {}", v_eci[0]);
        assert!(
            (v_eci[1] - expected_mag).abs() < 1e-9,
            "v_eci y should be {expected_mag}, got {}",
            v_eci[1]
        );
        assert!(v_eci[2].abs() < 1e-9, "v_eci z should be ~0, got {}", v_eci[2]);
        // Magnitude ≈ 465 m/s sanity range.
        assert!(
            (expected_mag - 465.1).abs() < 1.0,
            "expected magnitude near 465 m/s, got {expected_mag}"
        );
    }

    /// TC-FRAME-5: velocity round-trip ECI → ECEF → ECI recovers the input,
    /// including the `ω × r` cross-term.
    #[test]
    fn tc_frame_5_velocity_round_trip() {
        let r_eci = [7_000_000.0, 1_500_000.0, -250_000.0];
        let v_eci = [-200.0, 7_500.0, 100.0];
        let gha = 0.4321;

        let r_ef = inertial_to_earth_fixed(r_eci, gha);
        let v_ef = inertial_to_earth_fixed_vel(r_eci, v_eci, gha);
        let v_back = earth_fixed_to_inertial_vel(r_ef, v_ef, gha);

        for i in 0..3 {
            let err = (v_back[i] - v_eci[i]).abs();
            assert!(err < 1e-8, "velocity round-trip error component {i}: {err}");
        }
    }
}
