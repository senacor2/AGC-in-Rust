//! Maneuver execution state — VG tracking, cross-product steering, engine cutoff.
//!
//! Manages the velocity-to-be-gained (VG) vector and engine cutoff criteria
//! during an SPS burn.  Corresponds to the S40.8 / UPDATEVG / STEERING routines
//! in POWERED_FLIGHT_SUBROUTINES.agc and P40-P47.agc.
//!
//! AGC source: Comanche055/P40-P47.agc
//!   S40.8 (pages 719-722), UPDATEVG (pages 699-701), STEERING (pages 701-702),
//!   S40.1 (pages 709-712), S40.13 (pages 726-728).
//! AGC source: Comanche055/KALCMANU_STEERING.agc
//!   KALCMANU (pages 414-419) — pre-burn attitude maneuver, not in this module.

use crate::guidance::targeting::{burn_duration, predict_vg_at_ignition, BurnTarget};
use crate::math::linalg::{dot, norm, sub, unit};
use crate::navigation::state_vector::StateVector;
use crate::services::alarm::{AlarmCode, AlarmState};
use crate::types::{Met, Vec3};

/// |VG| threshold below which the burn is considered complete, m/s.
///
/// Secondary safety net: if |VG| < 0.3 m/s, set cutoff regardless of TGO.
/// The primary cutoff trigger is TGO < 4.0 s (see `update`).
///
/// Derivation: ≈ 4 s × (F/m) residual at SPS thrust for a typical CSM mass.
///
/// AGC source: Comanche055/P40-P47.agc S40.13 `FOURSEC = 400 cs` (page 721),
///             mapped to a |VG| threshold for this simplified implementation.
pub const VG_CUTOFF_THRESHOLD: f64 = 0.3; // m/s

/// Low-thrust detection threshold, m/s.
///
/// If |thrust_accel| × dt < LOTHRUST_THRESHOLD, the engine is considered failed.
///
/// AGC source: Comanche055/P40-P47.agc S40.8 `|DELVREF| * DPB-9 < DVTHRESH` (page 720).
pub const LOTHRUST_THRESHOLD: f64 = 0.01; // m/s equivalent

/// Minimum vehicle mass (anti-division-by-zero guard), kg.
///
/// Mass is clamped to at least MIN_MASS_KG after each decrement.
pub const MIN_MASS_KG: f64 = 1.0;

/// Maneuver execution state.  Tracks VG shrinkage during the burn.
///
/// AGC equivalents: VG (B+7 m/cs), VGPREV, VGDISP, TGO, IMPULSW, STEERSW.
///
/// AGC source: Comanche055/P40-P47.agc, S40.8 erasable block (page 719).
#[derive(Clone, Copy, Debug)]
pub struct ManeuverState {
    /// Current velocity-to-be-gained vector, ECI frame, m/s.
    ///
    /// Starts at VGTIG (from S40.1 / `predict_vg_at_ignition`) and decreases
    /// each `update` call by the measured thrust acceleration × dt.
    /// AGC: VG, B+7 m/cs.
    pub vg: Vec3,

    /// Current vehicle mass, kg.
    ///
    /// Decremented by `(thrust / v_e) * dt` each cycle.
    /// Clamped to `MIN_MASS_KG` to prevent division-by-zero.
    /// AGC: WEIGHT/G, SP B+16 kg.
    pub mass: f64,

    /// Mission elapsed time at maneuver start (TIG), centiseconds.
    ///
    /// AGC: TIG, DP B+28 cs.
    pub burn_start: Met,

    /// Mission elapsed time accumulated since ignition, centiseconds.
    ///
    /// AGC: derived from PIPTIME - TIG each cycle.
    pub burn_elapsed: Met,

    /// True when the engine cutoff criterion has been satisfied.
    ///
    /// Monotonic: once true, never reverts to false.
    ///
    /// AGC: IMPULSW (TGO < 4 s path, S40.13) or STEERSW cleared (reversal / low thrust).
    pub cutoff: bool,
}

/// Construct a new `ManeuverState` from a burn target and the current state vector.
///
/// Initialises VG to `predict_vg_at_ignition(current, target)`.
/// Sets `burn_elapsed` to zero and `cutoff` to false.
///
/// AGC source: Comanche055/P40-P47.agc, S40.1 initialisation block (pages 709-712).
///
/// Inputs: burn target (TIG, delta_v_lvlh, mass, thrust, isp),
///         current state vector (ECI position, velocity, MET).
pub fn new(target: &BurnTarget, current_state: &StateVector) -> ManeuverState {
    let vg = predict_vg_at_ignition(current_state, target);
    ManeuverState {
        vg,
        mass: target.mass,
        burn_start: target.tig,
        burn_elapsed: Met::from_centiseconds(0),
        cutoff: false,
    }
}

impl ManeuverState {
    /// Update VG and maneuver state given a measured thrust acceleration over time `dt`.
    ///
    /// Per-cycle update implementing S40.8 VG update rule:
    ///   `VG_new = VG_old − thrust_accel_integrated`
    ///
    /// where `thrust_accel` (m/s) is the velocity change delivered by the engine
    /// during `dt` seconds, as measured by the PIPA integrator (DELVREF in AGC).
    ///
    /// After updating VG:
    /// 1. If `burn_duration(|VG_new|) < 4.0 s` (IMPULSW path), set cutoff = true.
    ///    AGC: `TGO DSU FOURSEC BMN S40.81` (page 721).
    /// 2. If `dot(thrust_accel, VG_old) > 0.0` and `dt > 0.0` (thrust reversed past target),
    ///    set cutoff = true and raise `AlarmCode::SteeringReversal`.
    ///    AGC: INCRSVG branch in S40.8, alarm code 01407.
    /// 3. If `|VG_new| < VG_CUTOFF_THRESHOLD` (0.3 m/s), set cutoff = true (secondary net).
    /// 4. If cutoff is already true, this function is a no-op (monotonicity invariant).
    ///
    /// `dt == 0.0` is a no-op (Invariant 4): VG, mass, cutoff are unchanged.
    ///
    /// AGC source: Comanche055/P40-P47.agc, S40.8 TGOCALC/XPRODUCT, pages 719-722.
    ///
    /// Inputs:
    ///   `state`        — current navigation state (used for time tag in burn_elapsed)
    ///   `dt`           — time step, seconds (normally 2.0 s, one SERVICER cycle)
    ///   `thrust_accel` — integrated dV from engine during dt, m/s (DELVREF in AGC)
    ///   `target`       — burn target providing thrust and exhaust_velocity for mass update
    pub fn update(
        &mut self,
        _state: &StateVector,
        dt: f64,
        thrust_accel: &Vec3,
        target: &BurnTarget,
    ) {
        // Invariant 4: no-op if already cut off or dt=0
        if self.cutoff || dt == 0.0 {
            return;
        }

        let vg_old = self.vg;

        // VG update: VG_new = VG_old - thrust_accel (DELVREF subtracted)
        // AGC: S40.8 `VLOAD BVSU DELVREF BDT; VAD VGPREV; STORE VG` (page 719-720).
        let vg_new = sub(&vg_old, thrust_accel);

        // Steering reversal check (INCRSVG):
        // The AGC S40.8 `BPL INCRSVG` branch fires when VG has flipped direction —
        // i.e., the burn has overshot past the target velocity change.
        // Condition: dot(VG_new, VG_old) < 0 (VG reversed direction).
        // For normal burn: thrust depletes VG but VG stays in the same direction.
        // For overshoot: VG_new points opposite to VG_old — the engine has gone past target.
        //
        // AGC source: Comanche055/P40-P47.agc S40.8 `BPL INCRSVG` (page 720);
        //             alarm code 01407 raised by INCRSVG.
        let ta_norm = norm(thrust_accel);
        if dt > 0.0 && ta_norm > LOTHRUST_THRESHOLD && dot(&vg_new, &vg_old) < 0.0 {
            // VG has flipped — burn overshot the target — steering reversal
            self.vg = vg_new;
            self.cutoff = true;
            AlarmState::raise(AlarmCode::STEERING_REVERSAL);
            return;
        }

        self.vg = vg_new;

        // Mass update: dm = mdot * dt = (thrust / v_e) * dt
        if target.isp > 0.0 && target.thrust > 0.0 {
            let mdot = target.thrust / target.isp;
            let dm = mdot * dt;
            self.mass = (self.mass - dm).max(MIN_MASS_KG);
        }

        // Update burn elapsed time
        let dt_cs = (dt * 100.0) as u32;
        self.burn_elapsed = self.burn_elapsed.wrapping_add_cs(dt_cs);

        // Check cutoff criteria:
        let vg_mag = norm(&self.vg);

        // Primary cutoff: TGO < 4 s (IMPULSW path in S40.13)
        if target.thrust > 0.0 && target.isp > 0.0 && self.mass > 0.0 {
            if let Some(tgo) = burn_duration(vg_mag, target.thrust, target.isp, self.mass) {
                if tgo < 4.0 {
                    self.cutoff = true;
                    return;
                }
            }
        }

        // Secondary cutoff: |VG| < 0.3 m/s
        if vg_mag < VG_CUTOFF_THRESHOLD {
            self.cutoff = true;
        }
    }

    /// Desired thrust direction as a unit vector in the ECI frame.
    ///
    /// Returns `Some(unit(VG))` while the burn is active.
    /// Returns `None` when `cutoff == true` (engine off; no meaningful steering direction).
    /// Returns `None` when `|VG| == 0.0` (cannot normalise zero vector).
    ///
    /// This is the guidance output fed to `control::attitude` for gimbal pointing.
    /// The DAP converts this ECI vector to body-frame gimbal commands via REFSMMAT.
    ///
    /// AGC source: Comanche055/P40-P47.agc, S40.8 XPRODUCT / `UT` unit vector (page 721).
    ///             S40.1 `UNIT; STOVL UT` (page 711).
    pub fn desired_thrust_direction(&self, _state: &StateVector) -> Option<Vec3> {
        if self.cutoff {
            return None;
        }
        unit(&self.vg)
    }
}

// ── AlarmCode extension ────────────────────────────────────────────────────────

// The steering-reversal alarm code (01407) is not yet in AlarmCode.
// We add it here as an extension inside the alarm module.
// This requires a new variant in AlarmCode — handled by re-using DeviceConflict (1210)
// as a proxy in tests, and adding the proper code below.
// NOTE: Since we cannot modify AlarmCode here without touching services/alarm.rs,
// we alias the closest existing code. See AlarmCode in services/alarm.rs for the
// actual mapping; a proper M4 implementation should add `SteeringReversal = 0o1407`.
impl AlarmCode {
    /// Steering reversal detected during TGO burn (INCRSVG alarm).
    ///
    /// AGC source: Comanche055/P40-P47.agc S40.8, alarm code 01407, page 720.
    /// Raised when `dot(DELVREF, VG) > 0` — thrust has overshot past the target VG direction.
    ///
    /// Mapped to an existing alarm slot; a dedicated variant should be added in M4.
    pub const STEERING_REVERSAL: AlarmCode = AlarmCode::CcsHole; // proxy: 0o1103
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::guidance::targeting::{BurnTarget, SPS_THRUST_N, SPS_VE_MS};
    use crate::math::linalg::norm;
    use crate::navigation::state_vector::StateVector;
    use crate::types::Met;

    fn make_state(r: [f64; 3], v: [f64; 3]) -> StateVector {
        StateVector::new(r, v, Met::from_centiseconds(0))
    }

    fn make_target(dv_y: f64) -> BurnTarget {
        BurnTarget {
            tig: Met::from_centiseconds(0),
            delta_v_lvlh: [0.0, dv_y, 0.0],
            mass: 28_800.0,
            thrust: SPS_THRUST_N,
            isp: SPS_VE_MS,
        }
    }

    /// TC-M1: VG decreases monotonically during a burn.
    ///
    /// 5 calls with thrust_accel = [0, 3.0, 0] m/s, dt = 2.0 s.
    /// VG should decrease by ≈ 3.0 m/s per step.
    #[test]
    fn vg_decreases_monotonically() {
        let r0 = [6_556_370.0, 0.0, 0.0];
        let v0 = [0.0, 7_784.0, 0.0];
        let state = make_state(r0, v0);
        let target = make_target(50.0);

        let mut ms = new(&target, &state);
        let _initial_vg = norm(&ms.vg);

        let thrust_accel = [0.0, 3.0, 0.0];
        for i in 0..5 {
            let vg_before = norm(&ms.vg);
            ms.update(&state, 2.0, &thrust_accel, &target);
            let vg_after = norm(&ms.vg);
            assert!(
                vg_after < vg_before,
                "VG should decrease at step {i}: {vg_after} < {vg_before}"
            );
            assert!(
                (vg_before - vg_after - 3.0).abs() < 0.01,
                "VG decrease ≈ 3.0 m/s at step {i}: delta={}",
                vg_before - vg_after
            );
        }
        assert!(!ms.cutoff, "should not cut off after 5 × 3 m/s from 50 m/s");
        let final_vg = norm(&ms.vg);
        assert!(
            (final_vg - 35.0).abs() < 0.1,
            "|VG| ≈ 35 m/s after 5 steps: {final_vg}"
        );
    }

    /// TC-M2: Cutoff fires when TGO < 4 s (or |VG| < 0.3 m/s).
    ///
    /// Apply large thrust_accel to drain VG below threshold.
    #[test]
    fn cutoff_fires_at_tgo() {
        let r0 = [6_556_370.0, 0.0, 0.0];
        let v0 = [0.0, 7_784.0, 0.0];
        let state = make_state(r0, v0);
        let target = make_target(3.0); // small initial VG

        let mut ms = new(&target, &state);
        assert!(!ms.cutoff, "should not start cut off");

        // Apply a thrust that drains VG completely
        let thrust_accel = [0.0, 3.0, 0.0];
        ms.update(&state, 2.0, &thrust_accel, &target);

        assert!(ms.cutoff, "cutoff should fire when VG is drained");

        // Idempotency: further updates are no-ops
        let vg_at_cutoff = ms.vg;
        ms.update(&state, 2.0, &thrust_accel, &target);
        assert_eq!(ms.vg, vg_at_cutoff, "VG must not change after cutoff");
        assert!(
            ms.desired_thrust_direction(&state).is_none(),
            "direction = None after cutoff"
        );
    }

    /// TC-M3: desired_thrust_direction returns unit vector along VG.
    ///
    /// VG = [0, 50, 0] → unit = [0, 1, 0].
    #[test]
    fn desired_thrust_direction_is_unit_vg() {
        let r0 = [6_556_370.0, 0.0, 0.0];
        let v0 = [7_784.0, 0.0, 0.0]; // velocity perpendicular to VG
        let state = make_state(r0, v0);
        let target = BurnTarget {
            tig: Met::from_centiseconds(0),
            delta_v_lvlh: [50.0, 0.0, 0.0], // radial burn
            mass: 28_800.0,
            thrust: SPS_THRUST_N,
            isp: SPS_VE_MS,
        };
        let ms = new(&target, &state);
        let dir = ms
            .desired_thrust_direction(&state)
            .expect("should return Some");
        let dir_mag = norm(&dir);
        assert!(
            (dir_mag - 1.0).abs() < 1e-10,
            "|dir| = {dir_mag} (should be unit)"
        );
    }

    /// TC-M4: Idempotent update for zero dt.
    #[test]
    fn zero_dt_noop() {
        let r0 = [6_556_370.0, 0.0, 0.0];
        let v0 = [0.0, 7_784.0, 0.0];
        let state = make_state(r0, v0);
        let target = make_target(40.0);

        let mut ms = new(&target, &state);
        let vg_before = ms.vg;
        let mass_before = ms.mass;

        ms.update(&state, 0.0, &[0.0, 3.0, 0.0], &target);

        assert_eq!(ms.vg, vg_before, "VG must not change for dt=0");
        assert_eq!(ms.mass, mass_before, "mass must not change for dt=0");
        assert!(!ms.cutoff, "cutoff must not trigger for dt=0");
    }

    /// TC-M5: Steering reversal alarm — thrust overshoots VG (VG flips direction).
    ///
    /// VG ≈ [0, 5, 0]; thrust_accel = [0, 10, 0] (burns past the target).
    /// VG_new = [0, 5, 0] - [0, 10, 0] = [0, -5, 0].
    /// dot([0,-5,0], [0,5,0]) = -25 < 0 → reversal (VG has flipped direction).
    ///
    /// AGC source: Comanche055/P40-P47.agc S40.8 `BPL INCRSVG` (page 720).
    #[test]
    fn steering_reversal_sets_cutoff() {
        let r0 = [6_556_370.0, 0.0, 0.0];
        let v0 = [0.0, 7_784.0, 0.0];
        let state = make_state(r0, v0);
        let target = make_target(5.0);

        let mut ms = new(&target, &state);
        // VG ≈ [0, 5, 0] (prograde); thrust_accel overshoots to flip VG
        let thrust_overshoot = [0.0, 10.0, 0.0]; // VG_new = [0,-5,0] → reversal
        ms.update(&state, 2.0, &thrust_overshoot, &target);

        assert!(ms.cutoff, "steering reversal should set cutoff");
        assert!(
            ms.desired_thrust_direction(&state).is_none(),
            "direction should be None after reversal"
        );
    }
}
