//! P30 — External Delta-V program.
//!
//! Accepts crew-entered target parameters (time of ignition, desired delta-V)
//! via the DSKY and computes a `ManeuverPlan` for handoff to P40/P41 (SPS burn
//! execution programs).
//!
//! The crew enters the desired velocity components at TIG using Verb 25 Noun 33.
//! P30 then computes VGTIG (velocity-to-gain at ignition) and burn time.
//!
//! AGC source: Comanche055/P30-P37.agc — P30CSM entry, S40.1 (VGTIG
//! computation), S40.8 (burn-time computation).

use crate::guidance::targeting::{plan_maneuver, SPS_THRUST_N};
use crate::navigation::state_vector::StateVector;
use crate::types::Vec3;

/// Program number for External Delta-V.
pub const PROG_NUMBER: u8 = 30;

/// P30 execution phases (restart-safe state machine).
///
/// AGC source: P30-P37.agc — the phase variable is stored in erasable memory
/// (PHASENUM) so that a CMC restart can resume at the correct point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum P30Phase {
    /// Waiting for crew to enter target parameters via DSKY.
    AwaitInput,
    /// Target parameters received; computing burn parameters.
    Computing,
    /// Results computed; waiting for crew PRO (proceed) to hand off to P40.
    DisplayResult,
    /// Computation complete — plan is ready for P40 handoff.
    Complete,
}

/// P30 program state.
///
/// AGC source: P30-P37.agc — P30CSM uses TIG, DELVEET3 (target delta-V),
/// VGTIG, and TTOGO erasable variables.
#[derive(Clone, Copy, Debug)]
pub struct P30State {
    /// Current execution phase.
    pub phase: P30Phase,
    /// Time of ignition entered by crew (MET centiseconds).
    pub tig_cs: u32,
    /// Target delta-V entered by crew (ECI frame, m/s).
    pub target_dv: Vec3,
    /// Computed maneuver plan (available after `compute`).
    pub plan: Option<ManeuverPlan>,
}

use crate::guidance::targeting::ManeuverPlan;

impl P30State {
    /// Construct a new P30 state awaiting crew input.
    pub const fn new() -> Self {
        Self {
            phase: P30Phase::AwaitInput,
            tig_cs: 0,
            target_dv: [0.0; 3],
            plan: None,
        }
    }

    /// Accept target parameters entered by the crew via DSKY V25N33.
    ///
    /// Transitions the phase to `Computing` so that the next call to
    /// [`compute`](Self::compute) processes the new target.
    ///
    /// AGC source: P30-P37.agc — crew enters TIG via Noun 33 and ΔV components
    /// via Noun 81; these are written to TIG and DELVEET3 erasable.
    pub fn set_target(&mut self, tig_cs: u32, dv: Vec3) {
        self.tig_cs = tig_cs;
        self.target_dv = dv;
        self.phase = P30Phase::Computing;
    }

    /// Compute the maneuver plan from the current state vector and crew target.
    ///
    /// The target velocity at TIG is `sv.v + target_dv`, i.e. `target_dv` is
    /// the crew-specified VGTIG (velocity-to-gain).  `plan_maneuver` wraps the
    /// delta-V together with Tsiolkovsky burn-time estimation.
    ///
    /// Silently returns without updating the plan if called outside the
    /// `Computing` phase.
    ///
    /// AGC source: P30-P37.agc — S40.1 computes VGTIG = DELVEET3 − VATT;
    /// S40.8 computes TTOGO (time to go, i.e. burn duration).
    pub fn compute(&mut self, sv: &StateVector, mass_kg: f64) {
        if self.phase != P30Phase::Computing {
            return;
        }
        // target_dv is the desired ΔV, so the target velocity at the burn
        // epoch is v_current + target_dv.
        let v_target: Vec3 = [
            sv.v[0] + self.target_dv[0],
            sv.v[1] + self.target_dv[1],
            sv.v[2] + self.target_dv[2],
        ];
        let mut p = plan_maneuver(sv, &v_target, mass_kg, SPS_THRUST_N);
        // Override the TIG with the crew-entered value.
        p.tig_cs = self.tig_cs;
        self.plan = Some(p);
        self.phase = P30Phase::DisplayResult;
    }

    /// Acknowledge crew PRO key — transition to `Complete` so the executive
    /// can hand off to P40.
    ///
    /// Has no effect unless in `DisplayResult` phase.
    ///
    /// AGC source: P30-P37.agc — crew presses PRO to proceed to P40 burn
    /// execution after reviewing the displayed VGTIG and burn time.
    pub fn proceed(&mut self) {
        if self.phase == P30Phase::DisplayResult {
            self.phase = P30Phase::Complete;
        }
    }

    /// Return a reference to the computed plan.
    ///
    /// Returns `None` if `compute` has not yet been called successfully.
    pub fn get_plan(&self) -> Option<&ManeuverPlan> {
        self.plan.as_ref()
    }
}

impl Default for P30State {
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
    use crate::math::linalg::norm;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;

    fn make_sv(v: Vec3) -> StateVector {
        let r_mag = 6_578_137.0_f64;
        StateVector {
            frame: Frame::Eci,
            r: [r_mag, 0.0, 0.0],
            v,
            t: Met(100),
        }
    }

    /// set_target must transition the phase to Computing.
    #[test]
    fn set_target_transitions_to_computing() {
        let mut p30 = P30State::new();
        assert_eq!(p30.phase, P30Phase::AwaitInput);
        p30.set_target(5_000, [100.0, 0.0, 0.0]);
        assert_eq!(p30.phase, P30Phase::Computing);
    }

    /// compute produces a ManeuverPlan with the correct delta-V magnitude.
    #[test]
    fn compute_produces_correct_delta_v() {
        let dv_in: Vec3 = [100.0, 50.0, 0.0];
        let expected_mag = norm(&dv_in);
        let sv = make_sv([7500.0, 0.0, 0.0]);
        let mut p30 = P30State::new();
        p30.set_target(5_000, dv_in);
        p30.compute(&sv, 20_000.0);
        let plan = p30.get_plan().expect("plan must exist after compute");
        assert!(
            (plan.delta_v_mag - expected_mag).abs() < 1e-6,
            "dv_mag = {}, expected = {}",
            plan.delta_v_mag,
            expected_mag
        );
    }

    /// After compute, phase transitions to DisplayResult.
    #[test]
    fn compute_transitions_to_display_result() {
        let sv = make_sv([7500.0, 0.0, 0.0]);
        let mut p30 = P30State::new();
        p30.set_target(1_000, [50.0, 0.0, 0.0]);
        p30.compute(&sv, 20_000.0);
        assert_eq!(p30.phase, P30Phase::DisplayResult);
    }

    /// proceed transitions from DisplayResult to Complete.
    #[test]
    fn proceed_transitions_to_complete() {
        let sv = make_sv([7500.0, 0.0, 0.0]);
        let mut p30 = P30State::new();
        p30.set_target(1_000, [50.0, 0.0, 0.0]);
        p30.compute(&sv, 20_000.0);
        p30.proceed();
        assert_eq!(p30.phase, P30Phase::Complete);
        assert!(p30.get_plan().is_some());
    }

    /// compute outside Computing phase is a no-op.
    #[test]
    fn compute_noop_outside_computing_phase() {
        let sv = make_sv([7500.0, 0.0, 0.0]);
        let mut p30 = P30State::new();
        // Phase is AwaitInput — compute should do nothing.
        p30.compute(&sv, 20_000.0);
        assert!(p30.get_plan().is_none());
        assert_eq!(p30.phase, P30Phase::AwaitInput);
    }

    /// TIG override: plan.tig_cs must equal the crew-entered value.
    #[test]
    fn tig_is_overridden_by_crew_input() {
        let sv = make_sv([7500.0, 0.0, 0.0]);
        let mut p30 = P30State::new();
        p30.set_target(99_999, [50.0, 0.0, 0.0]);
        p30.compute(&sv, 20_000.0);
        let plan = p30.get_plan().unwrap();
        assert_eq!(plan.tig_cs, 99_999);
    }
}
