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
}
