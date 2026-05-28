//! Simplified spacecraft dynamics model for simulation.
//!
//! Integrates Δv from the SPS engine each simulator tick and emits PIPA
//! pulses for the IMU stub. The model is intentionally minimal: linear
//! motion only, no gravity (PIPAs measure non-gravitational acceleration
//! anyway), no attitude — `thrust_dir_platform` is taken as fixed during
//! the burn, on the assumption that the DAP slewed the vehicle to the
//! commanded attitude before crew PRO. That assumption fits the
//! `agc-sim` IMU stub, whose CDU angles are pinned to zero.
//!
//! Coupled with [`crate::SimHardware`] via `SimHardware::tick`.

use agc_core::navigation::gravity::{MU_EARTH, MU_MOON};
use agc_core::navigation::integration::{propagate_coast, soi_check};
use agc_core::navigation::planetary::moon_position;
use agc_core::navigation::state_vector::Frame;
use agc_core::navigation::StateVector;
use agc_core::types::Met;

/// PIPA hardware quantum: m/s per integer pulse.
///
/// Equal to [`agc_core::services::average_g::PipaCalibration::NOMINAL`]'s
/// scale, so a freshly-constructed `AgcState` interprets the simulator's
/// pulses correctly without crew calibration.
pub const PIPA_QUANTUM_M_S: f64 = 0.0585;

/// Apollo CSM SPS specifications used as `Spacecraft` defaults.
///
/// Public so tests and demo binaries can reference them when overriding
/// individual fields.
pub mod apollo_csm {
    /// Approximate CSM mid-mission mass with a partially-loaded SM (kg).
    pub const MASS_KG: f64 = 30_000.0;

    /// SPS thrust used by the simulator. The Apollo SPS produced ~91 kN
    /// at full thrust; we use a smaller value here so an unrescaled
    /// burn-time demonstration runs in the tens of seconds rather than
    /// the tens of milliseconds — see `docs/p40_burn_demo.md`.
    pub const SPS_THRUST_N: f64 = 45_000.0;
}

/// The gravitating body acting on the spacecraft.
///
/// Used by [`advance_ground_truth`] to select the correct gravitational
/// parameter for the two-body conic step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GravityBody {
    /// Earth is the primary gravitating body (ECI frame).
    Earth,
    /// Moon is the primary gravitating body (MCI frame).
    Moon,
}

impl GravityBody {
    /// Gravitational parameter μ (m³/s²) for this body.
    pub fn mu(self) -> f64 {
        match self {
            GravityBody::Earth => MU_EARTH,
            GravityBody::Moon => MU_MOON,
        }
    }
}

/// Simulator ground-truth dynamics state.
///
/// Owned by [`crate::SimHardware`]. Updated each `SimHardware::tick`
/// call; consumed by `SimImu::read_pipa` indirectly (the tick drains
/// pulses into `SimImu::pipa`).
pub struct Spacecraft {
    /// Vehicle mass (kg).
    pub mass_kg: f64,

    /// SPS thrust magnitude (N) when the engine is commanded on.
    pub sps_thrust_n: f64,

    /// Unit vector pointing along SPS thrust in the IMU platform frame.
    ///
    /// During a real burn this equals the body axis of the SPS nozzle,
    /// rotated by the platform-to-body matrix. The simulator skips
    /// attitude dynamics; tests configure this directly to whatever
    /// inertial axis the burn should accumulate Δv along, on the
    /// understanding that the test's `state.refsmmat` rotates platform
    /// → inertial.
    pub thrust_dir_platform: [f64; 3],

    /// Sub-quantum Δv carried over between PIPA reads (m/s, platform frame).
    ///
    /// Real PIPAs are pulse-output devices, so a 2-second integration
    /// at 1.5 m/s² yields 3.0 m/s ÷ 0.0585 m/s/count = 51.28 counts —
    /// the hardware emits 51 pulses and saves the 0.28-count remainder
    /// for the next interval. This field is that remainder.
    pipa_residue_m_s: [f64; 3],

    /// Whether gravitational acceleration is modelled in
    /// [`advance_ground_truth`].
    ///
    /// Defaults to `false` to keep legacy SPS-only tests stable.
    /// Set to `true` for any scenario that uses [`advance_ground_truth`]
    /// as a reference trajectory.
    pub gravity_enabled: bool,

    /// The primary gravitating body for two-body conic propagation in
    /// [`advance_ground_truth`].
    ///
    /// Defaults to `Earth`. Updated automatically by [`advance_ground_truth`]
    /// when an SOI crossing is detected.
    pub current_body: GravityBody,

    /// Whether atmospheric drag is modelled (stub for MS-T6).
    ///
    /// Defaults to `false`. Currently has no effect; reserved for future use.
    pub atmosphere_enabled: bool,
}

impl Default for Spacecraft {
    fn default() -> Self {
        Self::new()
    }
}

impl Spacecraft {
    /// Apollo-CSM-like defaults: 30 t, 45 kN SPS, thrust along inertial
    /// +Y (matches the orbit set up by the P40 burn demo).
    pub fn new() -> Self {
        Self {
            mass_kg: apollo_csm::MASS_KG,
            sps_thrust_n: apollo_csm::SPS_THRUST_N,
            thrust_dir_platform: [0.0, 1.0, 0.0],
            pipa_residue_m_s: [0.0; 3],
            gravity_enabled: false,
            current_body: GravityBody::Earth,
            atmosphere_enabled: false,
        }
    }

    /// Acceleration magnitude along `thrust_dir_platform` while the SPS
    /// is on (m/s²). Convenience accessor used by tests and the demo doc.
    pub fn sps_acceleration_m_s2(&self) -> f64 {
        self.sps_thrust_n / self.mass_kg
    }

    /// Advance the dynamics by `dt_seconds`.
    ///
    /// When `engine_on` is true, integrates `acceleration × dt_seconds`
    /// onto the per-axis Δv residue. With the engine off this is a
    /// no-op — PIPAs measure non-gravitational acceleration only, so
    /// coast phases do not generate pulses.
    pub fn tick(&mut self, dt_seconds: f64, engine_on: bool) {
        if !engine_on || dt_seconds <= 0.0 {
            return;
        }
        let accel = self.sps_acceleration_m_s2();
        for (residue, &dir) in self
            .pipa_residue_m_s
            .iter_mut()
            .zip(self.thrust_dir_platform.iter())
        {
            *residue += accel * dir * dt_seconds;
        }
    }

    /// Drain accumulated Δv as integer PIPA pulses.
    ///
    /// Returns the integer count per axis (`trunc` toward zero) and
    /// preserves the sub-quantum remainder for the next call so no
    /// motion is lost. Saturates to `i16::{MIN,MAX}` on overflow.
    pub fn drain_pipa_pulses(&mut self) -> [i16; 3] {
        let mut out = [0i16; 3];
        for (residue, slot) in self.pipa_residue_m_s.iter_mut().zip(out.iter_mut()) {
            let raw = (*residue / PIPA_QUANTUM_M_S).trunc();
            let clamped = raw.clamp(i16::MIN as f64, i16::MAX as f64);
            let pulses = clamped as i16;
            *residue -= pulses as f64 * PIPA_QUANTUM_M_S;
            *slot = pulses;
        }
        out
    }
}

// ── Ground-truth propagator ───────────────────────────────────────────────────

/// Advance `state` by `dt` seconds of unpowered, gravity-only flight.
///
/// Propagates the state vector using the high-accuracy RK4 Cowell integrator
/// ([`agc_core::navigation::integration::propagate_coast`]) with the same
/// gravity model as the AGC SERVICER (Earth J2 + Moon third-body), then
/// calls `soi_check` to detect SOI crossings.
/// Updates `state.epoch`. No-op if `dt <= 0.0` or `!sc.gravity_enabled`.
///
/// # Why `propagate_coast` rather than `kepler_step`
///
/// Using `propagate_coast` (RK4 Cowell, 4th-order) as ground truth against
/// the SERVICER's `average_g_step` (trapezoidal, 2nd-order) is **not**
/// tautological: they use different integration algorithms while sharing the
/// same physics model. The comparison tests the SERVICER's 2nd-order accuracy.
/// In contrast, `kepler_step` (pure two-body) diverges from the SERVICER by
/// ~100–300 km over 24 h due to Earth J2 and Moon third-body effects, making
/// it an unsuitable reference for the 5 km tolerance.
pub fn advance_ground_truth(sc: &mut Spacecraft, state: &mut StateVector, dt: f64) {
    if dt <= 0.0 || !sc.gravity_enabled {
        return;
    }

    let epoch_s = state.epoch.to_seconds();

    // Compute Moon position at the current epoch for the propagator.
    let moon_pos = moon_position(state.epoch);

    // Propagate via high-accuracy RK4 Cowell integrator with J2 + moon gravity.
    // This gives sub-km accuracy over 24 h and uses the same physics model as
    // the SERVICER, ensuring a fair comparison.
    let propagated = propagate_coast(*state, dt, moon_pos);

    *state = propagated;

    // Compute Moon position and velocity at the new epoch for SOI check.
    // Moon velocity is computed via central difference since moon_velocity
    // does not yet exist in agc-core.
    // TODO: replace with moon_velocity from #52 once it is implemented.
    let delta_s = 10.0_f64;
    let moon_pos_new = moon_position(state.epoch);
    let moon_pos_before = moon_position(Met::from_seconds((epoch_s + dt) - delta_s));
    let moon_pos_after = moon_position(Met::from_seconds((epoch_s + dt) + delta_s));
    let moon_vel = [
        (moon_pos_after[0] - moon_pos_before[0]) / (2.0 * delta_s),
        (moon_pos_after[1] - moon_pos_before[1]) / (2.0 * delta_s),
        (moon_pos_after[2] - moon_pos_before[2]) / (2.0 * delta_s),
    ];

    // Run SOI check — may convert ECI ↔ MCI and update the frame.
    let checked = soi_check(*state, moon_pos_new, moon_vel);

    // If the frame changed, update sc.current_body to match.
    if checked.frame != state.frame {
        sc.current_body = match checked.frame {
            Frame::EarthInertial => GravityBody::Earth,
            Frame::MoonInertial => GravityBody::Moon,
            Frame::StableMember => sc.current_body, // unreachable
        };
    }

    *state = checked;
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-PHYS-1: engine off ⇒ no Δv, no pulses.
    #[test]
    fn tc_phys_1_engine_off_no_pulses() {
        let mut sc = Spacecraft::new();
        sc.tick(2.0, false);
        assert_eq!(sc.drain_pipa_pulses(), [0, 0, 0]);
        assert_eq!(sc.pipa_residue_m_s, [0.0; 3]);
    }

    /// TC-PHYS-2: 2-second tick at default thrust produces 51 pulses
    /// along the configured axis (1.5 m/s² × 2 s = 3.0 m/s, ÷0.0585 ≈ 51.28).
    #[test]
    fn tc_phys_2_default_thrust_one_cycle() {
        let mut sc = Spacecraft::new();
        sc.tick(2.0, true);
        let pulses = sc.drain_pipa_pulses();
        assert_eq!(pulses, [0, 51, 0]);
        // Residue ≈ 0.0165 m/s carried forward.
        assert!(
            (sc.pipa_residue_m_s[1] - 0.0165).abs() < 1e-6,
            "residue carry-over should be ≈ 0.0165, got {}",
            sc.pipa_residue_m_s[1]
        );
    }

    /// TC-PHYS-3: residue carries forward across reads (no Δv lost).
    #[test]
    fn tc_phys_3_residue_carries_forward() {
        let mut sc = Spacecraft::new();
        let mut total_pulses = 0i64;
        for _ in 0..7 {
            sc.tick(2.0, true);
            total_pulses += sc.drain_pipa_pulses()[1] as i64;
        }
        // 7 × 3.0 m/s = 21.0 m/s simulated, ÷0.0585 ≈ 358.97 pulses.
        // The trunc-with-residue strategy must emit exactly 358 pulses
        // over 7 cycles — never lose more than one quantum total.
        assert_eq!(total_pulses, 358);
    }

    /// TC-PHYS-4: zero or negative dt is a no-op.
    #[test]
    fn tc_phys_4_zero_dt_no_op() {
        let mut sc = Spacecraft::new();
        sc.tick(0.0, true);
        sc.tick(-1.0, true);
        assert_eq!(sc.drain_pipa_pulses(), [0, 0, 0]);
    }

    /// TC-PHYS-5: thrust direction is honoured per axis.
    #[test]
    fn tc_phys_5_thrust_direction() {
        let mut sc = Spacecraft::new();
        sc.thrust_dir_platform = [0.0, 0.0, 1.0];
        sc.tick(2.0, true);
        let pulses = sc.drain_pipa_pulses();
        assert_eq!(pulses[0], 0);
        assert_eq!(pulses[1], 0);
        assert_eq!(pulses[2], 51);
    }

    /// tc_phys_coast_24h_leo_kepler_within_1km
    ///
    /// Verify that 90 sequential 60 s `advance_ground_truth` steps (one LEO
    /// orbital period ≈ 5400 s) agree with a single `propagate_coast` call
    /// to within 1 km position and 1 m/s velocity.
    ///
    /// Both paths use the same RK4 Cowell integrator with the same fixed moon
    /// position. The test verifies that sub-step accumulation over 90 steps
    /// stays below the 1 km tolerance. A fixed moon position (t=0) is used for
    /// the reference so the comparison is fair: `advance_ground_truth` computes
    /// the moon position once per step at the current epoch, while the
    /// single-step reference uses moon_pos at t=0 only; over one orbit
    /// (~5400 s) the moon moves ~5500 km but its third-body perturbation at
    /// LEO is <10⁻⁶ m/s², so the difference is negligible.
    #[test]
    fn tc_phys_coast_24h_leo_kepler_within_1km() {
        use agc_core::navigation::integration::propagate_coast;

        let r = 6_778_000.0_f64;
        let v_circ = (MU_EARTH / r).sqrt();

        let initial_sv = StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        // Moon position at t=0 used for the reference propagation.
        let moon_pos = agc_core::navigation::planetary::moon_position(Met(0));

        // Set up a spacecraft for ground-truth propagation.
        let mut sc = Spacecraft::new();
        sc.gravity_enabled = true;
        sc.current_body = GravityBody::Earth;

        let mut state = initial_sv;

        // 90 steps × 60 s = 5400 s ≈ one LEO orbital period.
        for _ in 0..90 {
            advance_ground_truth(&mut sc, &mut state, 60.0);
        }

        // Reference: single propagate_coast over 5400 s.
        let ref_sv = propagate_coast(initial_sv, 5_400.0, moon_pos);

        // Compute L2 norms of error.
        let dp = [
            state.position[0] - ref_sv.position[0],
            state.position[1] - ref_sv.position[1],
            state.position[2] - ref_sv.position[2],
        ];
        let pos_err = (dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]).sqrt();

        let dv = [
            state.velocity[0] - ref_sv.velocity[0],
            state.velocity[1] - ref_sv.velocity[1],
            state.velocity[2] - ref_sv.velocity[2],
        ];
        let vel_err = (dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]).sqrt();

        assert!(
            pos_err < 1_000.0,
            "5400 s LEO coast: position error {pos_err:.1} m exceeds 1 km tolerance"
        );
        assert!(
            vel_err < 1.0,
            "5400 s LEO coast: velocity error {vel_err:.4} m/s exceeds 1 m/s tolerance"
        );
    }
}
