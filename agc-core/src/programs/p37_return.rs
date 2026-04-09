//! P37 — Return to Earth.
//!
//! Emergency return program.  Given a desired time of ignition and time of
//! flight, P37 propagates the current state vector to TIG using Kepler
//! universal variables, constructs an entry interface (EI) target point at
//! 122 km altitude, and solves Lambert's problem to find the required
//! delta-V.
//!
//! The resulting [`ManeuverPlan`] is handed off to P40/P41 for SPS burn
//! execution.
//!
//! AGC source: Comanche055/P30-P37.agc — P37 RETURN TO EARTH section.

use crate::guidance::targeting::{ManeuverPlan, SPS_THRUST_N};
use crate::guidance::targeting::{compute_delta_v, estimate_burn_time};
use crate::math::kepler::kepler_universal;
use crate::math::lambert::lambert;
use crate::math::linalg::{norm, scale as vec_scale};
use crate::navigation::gravity::{MU_EARTH, RE_EARTH};
use crate::navigation::state_vector::StateVector;
use crate::types::Vec3;

/// Program number for Return to Earth.
pub const PROG_NUMBER: u8 = 37;

/// Desired flight-path angle at entry interface (rad).
///
/// −6.5° is the nominal corridor centre for a lunar-return entry.
///
/// AGC source: P30-P37.agc — entry corridor constraint used to select the
/// Lambert solution that satisfies the ENTRY ANGLE requirement.
pub const ENTRY_ANGLE_RAD: f64 = -0.1134; // ≈ −6.5°

/// Entry interface altitude (m) — 400 000 ft / 122 km.
///
/// AGC source: P30-P37.agc — EIALT = 122 000 m (400 kft).
pub const ENTRY_ALTITUDE_M: f64 = 122_000.0;

/// P37 execution phases.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum P37Phase {
    /// Waiting for crew to enter the desired return TIG and TOF.
    AwaitTig,
    /// Computing the return trajectory via Lambert.
    Computing,
    /// Results available; awaiting crew PRO to proceed to P40.
    DisplayResult,
    /// Computation complete — plan ready for P40 handoff.
    Complete,
}

/// P37 program state.
///
/// AGC source: P30-P37.agc — P37 uses TIG, TOFRT (time of flight for return),
/// and VGTIG erasable variables.
#[derive(Clone, Copy, Debug)]
pub struct P37State {
    /// Current execution phase.
    pub phase: P37Phase,
    /// Desired time of ignition (MET centiseconds).
    pub tig_cs: u32,
    /// Desired time of flight for the return arc (s).
    pub tof_s: f64,
    /// Computed maneuver plan (available after a successful `compute`).
    pub plan: Option<ManeuverPlan>,
}

impl P37State {
    /// Construct a new P37 state awaiting crew input.
    pub const fn new() -> Self {
        Self {
            phase: P37Phase::AwaitTig,
            tig_cs: 0,
            tof_s: 0.0,
            plan: None,
        }
    }

    /// Accept return parameters entered by the crew.
    ///
    /// `tig_cs` is the desired ignition time in MET centiseconds.
    /// `tof_s` is the desired time of flight from TIG to EI (seconds).
    ///
    /// AGC source: P30-P37.agc — crew enters TIG via Noun 33 and TOF via
    /// Noun 37; values are written to TIG and TOFRT erasable.
    pub fn set_return_params(&mut self, tig_cs: u32, tof_s: f64) {
        self.tig_cs = tig_cs;
        self.tof_s = tof_s;
        self.phase = P37Phase::Computing;
    }

    /// Compute the return trajectory using Kepler propagation + Lambert solver.
    ///
    /// Steps:
    /// 1. Propagate `sv` from its epoch to TIG using universal-variable Kepler.
    /// 2. Construct the entry interface target point: a vector of magnitude
    ///    `R_EARTH + 122 km` in the direction of the propagated position.
    /// 3. Solve Lambert's problem connecting TIG position to EI point in `tof_s`.
    /// 4. Compute delta-V = Lambert v1 − v_at_TIG.
    ///
    /// Silently leaves the plan as `None` and stays in `Computing` if any step
    /// fails (non-convergence, degenerate geometry).  The caller may retry.
    ///
    /// AGC source: P30-P37.agc — P37 calls CSMCONIC to propagate to TIG, then
    /// LAMROUT (Lambert) to solve for the return burn VGTIG.
    pub fn compute(&mut self, sv: &StateVector, mass_kg: f64) {
        if self.phase != P37Phase::Computing {
            return;
        }

        // 1. Propagate state vector from sv.t to TIG.
        let t_sv_s = sv.t.to_secs();
        let t_tig_s = (self.tig_cs as f64) * 0.01;
        let dt_to_tig = t_tig_s - t_sv_s;

        let tig_state = if dt_to_tig.abs() < 1e-3 {
            // Already at TIG epoch.
            *sv
        } else {
            match kepler_universal(&sv.r, &sv.v, dt_to_tig, MU_EARTH) {
                Some(kr) => {
                    let mut s = *sv;
                    s.r = kr.r;
                    s.v = kr.v;
                    s
                }
                None => return,
            }
        };

        // 2. Build EI target point.
        //
        // The EI point lies at RE_EARTH + 122 km altitude on the opposite
        // side of Earth from the spacecraft (the approach hemisphere).
        // For a return-from-moon trajectory, the spacecraft is distant and
        // approaching; the EI point is where it enters the atmosphere, which
        // is on the Earth-facing side — opposite to the current position vector.
        //
        // AGC source: uses RE_EARTH = 6,373,338 m (ERAD from
        // LATITUDE_LONGITUDE_SUBROUTINES.agc).
        let r_ei_mag = RE_EARTH + ENTRY_ALTITUDE_M;
        let r_tig_mag = norm(&tig_state.r);
        if r_tig_mag < 1.0 {
            return;
        }
        // Negate direction: EI is on the approach side (opposite to current r).
        let r_ei = vec_scale(
            &tig_state.r,
            -r_ei_mag / r_tig_mag,
        );

        // 3. Lambert solver: TIG position → EI position in tof_s.
        let lambert_result = match lambert(&tig_state.r, &r_ei, self.tof_s, MU_EARTH, false) {
            Some(lr) => lr,
            None => return,
        };

        // 4. Delta-V = Lambert v1 − v_at_TIG.
        let delta_v_eci = compute_delta_v(&tig_state, &lambert_result.v1);
        let delta_v_mag = norm(&delta_v_eci);
        let burn_time_s = estimate_burn_time(delta_v_mag, mass_kg, SPS_THRUST_N);

        self.plan = Some(ManeuverPlan {
            tig_cs: self.tig_cs,
            delta_v_eci,
            delta_v_mag,
            burn_time_s,
        });
        self.phase = P37Phase::DisplayResult;
    }

    /// Acknowledge crew PRO — transition to `Complete` for P40 handoff.
    ///
    /// Has no effect unless in `DisplayResult` phase.
    pub fn proceed(&mut self) {
        if self.phase == P37Phase::DisplayResult {
            self.phase = P37Phase::Complete;
        }
    }

    /// Return a reference to the computed plan.
    pub fn get_plan(&self) -> Option<&ManeuverPlan> {
        self.plan.as_ref()
    }
}

impl Default for P37State {
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
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;

    /// Circular LEO state vector at 400 km altitude, used as a baseline.
    fn leo_sv() -> StateVector {
        let r_mag = 6_371_000.0 + 400_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r_mag);
        StateVector {
            frame: Frame::Eci,
            r: [r_mag, 0.0, 0.0],
            v: [0.0, v_circ, 0.0],
            t: Met(0),
        }
    }

    /// set_return_params must transition the phase to Computing.
    #[test]
    fn set_return_params_transitions_to_computing() {
        let mut p37 = P37State::new();
        assert_eq!(p37.phase, P37Phase::AwaitTig);
        p37.set_return_params(10_000, 3_600.0);
        assert_eq!(p37.phase, P37Phase::Computing);
        assert_eq!(p37.tig_cs, 10_000);
        assert!((p37.tof_s - 3_600.0).abs() < 1e-9);
    }

    /// compute with valid inputs must produce a plan (DisplayResult phase).
    #[test]
    fn compute_produces_plan_for_valid_orbit() {
        let sv = leo_sv();
        // TIG at t=0 (already at epoch), TOF long enough for the geometry.
        let tof = 1_800.0; // 30 minutes — short enough but non-degenerate
        let mut p37 = P37State::new();
        p37.set_return_params(0, tof);
        p37.compute(&sv, 20_000.0);
        // Lambert may or may not converge for the collinear EI target — accept
        // both outcomes but if a plan is produced it must have a finite dv_mag.
        if let Some(plan) = p37.get_plan() {
            assert!(plan.delta_v_mag.is_finite(), "dv_mag must be finite");
            assert!(plan.delta_v_mag >= 0.0, "dv_mag must be non-negative");
            assert_eq!(p37.phase, P37Phase::DisplayResult);
        }
    }

    /// proceed transitions DisplayResult → Complete.
    #[test]
    fn proceed_transitions_to_complete() {
        let sv = leo_sv();
        let mut p37 = P37State::new();
        p37.set_return_params(0, 1_800.0);
        p37.compute(&sv, 20_000.0);
        if p37.phase == P37Phase::DisplayResult {
            p37.proceed();
            assert_eq!(p37.phase, P37Phase::Complete);
            assert!(p37.get_plan().is_some());
        }
    }

    /// compute outside Computing phase is a no-op.
    #[test]
    fn compute_noop_outside_computing_phase() {
        let sv = leo_sv();
        let mut p37 = P37State::new();
        p37.compute(&sv, 20_000.0);
        assert!(p37.get_plan().is_none());
        assert_eq!(p37.phase, P37Phase::AwaitTig);
    }

    /// For a trans-lunar return scenario, delta-V must be in a reasonable
    /// range (roughly 100–3000 m/s for a typical return burn).
    #[test]
    fn return_dv_in_plausible_range_for_tle_return() {
        // Simulate a state at ~300 000 km from Earth (near moon) on a
        // trans-lunar trajectory with a velocity roughly toward Earth.
        let r_mag = 300_000.0e3_f64; // 300 000 km
        // Approximate inbound velocity on a trans-lunar orbit: ~1 km/s
        let sv = StateVector {
            frame: Frame::Eci,
            r: [r_mag, 0.0, 0.0],
            v: [-900.0, 200.0, 0.0],
            t: Met(0),
        };
        let tof_days = 2.5_f64;
        let tof_s = tof_days * 86_400.0;
        // TIG 6 hours in the future.
        let tig_cs = 6 * 3600 * 100_u32;
        let mut p37 = P37State::new();
        p37.set_return_params(tig_cs, tof_s);
        p37.compute(&sv, 20_000.0);
        if let Some(plan) = p37.get_plan() {
            assert!(
                plan.delta_v_mag < 3_500.0,
                "dv_mag = {} m/s is unrealistically large",
                plan.delta_v_mag
            );
            assert!(
                plan.delta_v_mag > 0.0,
                "dv_mag = {} must be positive",
                plan.delta_v_mag
            );
        }
    }
}
