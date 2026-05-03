//! P34 — Terminal Phase Midcourse (TPM) correction targeting.
//!
//! Computes the midcourse correction burn during the TPI transfer that
//! brings the chaser back to the **same** arrival position established by
//! the P33 TPI solution, using the Lambert solver. The burn is expressed as
//! a `Maneuver` in `state.pending_maneuver` with `mode = TargetingMode::TpmBurn`.
//!
//! P34 reads `state.tpi_arrival_epoch` (written by P33) to derive the
//! remaining transfer time.
//!
//! Spec: specs/p33_p34-spec.md §1.3, §4.3, §5.2, §5.4
//! AGC source: Comanche055/P34,P35,P74,P75.agc (TPM entry sequence)

use crate::executive::job::JobPriority;
use crate::guidance::targeting::{burn_attitude, lvlh_to_inertial, Maneuver, TargetingMode};
use crate::math::kepler::kepler_step;
use crate::math::linalg::{mxv, norm, vsub};
use crate::navigation::gravity::MU_EARTH;
use crate::programs::p33::{
    compute_lambert_intercept, InterceptError, ALARM_P33_DEGENERATE, ALARM_P33_LAMBERT,
    ALARM_P33_NO_TARGET, ALARM_P33_NO_TIG, ALARM_P33_STALE_TARGET, TPI_MIN_TOF_S,
    TPI_STALE_TARGET_CS,
};
use crate::types::{DeltaV, Met, Vec3};
use crate::AgcState;

// ── Constants ──────────────────────────────────────────────────────────────────

/// Major mode number for P34.
pub const P34_MAJOR_MODE: u8 = 34;

/// Job priority for P34.
pub const P34_PRIORITY: JobPriority = 10;

/// Minimum chaser-target range (m) below which P34 refuses to compute a
/// midcourse correction. If the two vehicles are this close, a midcourse
/// is physically meaningless and the crew should switch to braking.
///
/// 100 m corresponds to the final approach range where braking takes over.
pub const TPM_MIN_RANGE_M: f64 = 100.0;

// ── Alarm codes ────────────────────────────────────────────────────────────────
//
// P34 shares alarm codes 01440–01444 with P33 (same conditions).
// Alarm 01445 is P34-specific.

/// Alarm 01445 (octal): chaser already within `TPM_MIN_RANGE_M` of target
/// (P34 is meaningless at this range; braking P47 should be used instead).
pub(crate) const ALARM_P34_TOO_CLOSE: u16 = 0o01445;

// ── Entry point registered in PROGRAM_TABLE ────────────────────────────────────

/// Entry point for P34 (Terminal Phase Midcourse).
/// Registered in PROGRAM_TABLE[34].
///
/// Sets `state.major_mode = 34`. Reads `state.tpi_arrival_epoch` to
/// determine the scheduled arrival epoch, then calls
/// `compute_lambert_intercept` with the remaining transfer time
/// `dt_midcourse = tpi_arrival_epoch - now_s`. Stores the result in
/// `state.pending_maneuver` with `mode = TargetingMode::TpmBurn`.
///
/// # Preconditions
/// - `state.rendezvous_nav.target_pos` must be non-zero; otherwise alarm
///   01440.
/// - `state.tpi_arrival_epoch` must be `Some(_)` (P33 must have run);
///   otherwise alarm 01441.
/// - `state.vn.pending_tig` must be `Some(_)`; otherwise alarm 01441.
/// - `dt_midcourse` must be ≥ `TPI_MIN_TOF_S`; otherwise alarm 01443.
/// - Chaser-to-target range must be ≥ `TPM_MIN_RANGE_M`; otherwise alarm
///   01445.
///
/// # Post-conditions (success)
/// - `state.major_mode == 34`
/// - `state.dsky.prog == 34`
/// - `state.pending_maneuver == Some(tpm_maneuver)` where
///   `tpm_maneuver.mode == TargetingMode::TpmBurn`
/// - `state.tpi_arrival_epoch` is NOT modified.
///
/// # Alarms
/// - 01440: target state zero.
/// - 01441: `tpi_arrival_epoch` is None or `pending_tig` is None.
/// - 01442: target state stale (non-fatal warning).
/// - 01443: `dt_midcourse < TPI_MIN_TOF_S` or TIG in the past.
/// - 01444: Lambert non-convergence or anti-parallel vectors.
/// - 01445: chaser within `TPM_MIN_RANGE_M` of target.
///
/// Spec: specs/p33_p34-spec.md §4.3, §5.4
pub fn p34_init(state: &mut AgcState) -> JobPriority {
    // Step 0 — Guard checks.
    if norm(state.rendezvous_nav.target_pos) < 1.0 {
        state.alarm.code = ALARM_P33_NO_TARGET;
        state.alarm.lit = true;
        return P34_PRIORITY;
    }

    let arrival_epoch_s = match state.tpi_arrival_epoch {
        Some(t) => t,
        None => {
            state.alarm.code = ALARM_P33_NO_TIG;
            state.alarm.lit = true;
            return P34_PRIORITY;
        }
    };

    let tig_cs = match state.vn.pending_tig.take() {
        Some(t) => t,
        None => {
            state.alarm.code = ALARM_P33_NO_TIG;
            state.alarm.lit = true;
            return P34_PRIORITY;
        }
    };

    // Step 1 — Staleness check (non-fatal).
    let target_epoch_cs = (state.rendezvous_nav.target_epoch * 100.0) as u64;
    let tig_cs_u64 = tig_cs.0 as u64;
    if tig_cs_u64 > target_epoch_cs && (tig_cs_u64 - target_epoch_cs) > TPI_STALE_TARGET_CS {
        state.alarm.code = ALARM_P33_STALE_TARGET;
        state.alarm.lit = true;
        // Non-fatal: proceed with computation.
    }

    // Step 2 — Set major mode.
    state.major_mode = P34_MAJOR_MODE;
    state.dsky.prog = P34_MAJOR_MODE;
    state.dsky.verb = 6;
    state.dsky.noun = 37;
    state.dsky.flashing = true;

    // Step 3 — Compute dt_midcourse.
    let tig_s = tig_cs.to_seconds();
    let dt_midcourse = arrival_epoch_s - tig_s;
    if dt_midcourse < TPI_MIN_TOF_S {
        state.alarm.code = ALARM_P33_DEGENERATE;
        state.alarm.lit = true;
        return P34_PRIORITY;
    }

    // Step 4 — Range check.
    let range_vec: Vec3 = vsub(state.rendezvous_nav.target_pos, state.csm_state.position);
    let range_m = norm(range_vec);
    if range_m < TPM_MIN_RANGE_M {
        state.alarm.code = ALARM_P34_TOO_CLOSE;
        state.alarm.lit = true;
        return P34_PRIORITY;
    }

    // Step 5 — Propagate chaser to P34 TIG.
    let chaser_epoch_s = state.csm_state.epoch.to_seconds();
    let dt_chaser = tig_s - chaser_epoch_s;
    if dt_chaser < 0.0 {
        state.alarm.code = ALARM_P33_DEGENERATE;
        state.alarm.lit = true;
        return P34_PRIORITY;
    }
    let (r_c_tig, v_c_tig) = kepler_step(
        state.csm_state.position,
        state.csm_state.velocity,
        dt_chaser,
        MU_EARTH,
    );

    // Step 6 — Propagate target to arrival epoch (the original P33 arrival).
    let dt_target_to_arrive = arrival_epoch_s - state.rendezvous_nav.target_epoch;
    let (r_t_arrive, _) = kepler_step(
        state.rendezvous_nav.target_pos,
        state.rendezvous_nav.target_vel,
        dt_target_to_arrive,
        MU_EARTH,
    );

    // Step 7 — Compute Lambert intercept.
    match compute_lambert_intercept(r_c_tig, v_c_tig, r_t_arrive, dt_midcourse, MU_EARTH) {
        Err(InterceptError::DegenerateState) => {
            state.alarm.code = ALARM_P33_NO_TARGET;
            state.alarm.lit = true;
            return P34_PRIORITY;
        }
        Err(InterceptError::InvalidTimeInterval) | Err(InterceptError::DegenerateGeometry) => {
            state.alarm.code = ALARM_P33_DEGENERATE;
            state.alarm.lit = true;
            return P34_PRIORITY;
        }
        Err(InterceptError::AntiParallelVectors) | Err(InterceptError::LambertNotConverged) => {
            state.alarm.code = ALARM_P33_LAMBERT;
            state.alarm.lit = true;
            return P34_PRIORITY;
        }
        Ok(res) => {
            // Step 8 — Build Maneuver.
            let dv_inertial = mxv(lvlh_to_inertial(r_c_tig, v_c_tig), res.dv_lvlh);
            let attitude = burn_attitude(dv_inertial, state.refsmmat);

            state.pending_maneuver = Some(Maneuver {
                tig: Met::from_seconds(tig_s),
                delta_v: DeltaV(dv_inertial),
                burn_attitude: attitude,
                mode: TargetingMode::TpmBurn,
            });

            // Step 9 — DSKY display. tpi_arrival_epoch is NOT modified.
            state.dsky.verb = 6;
            state.dsky.noun = 81;
            state.dsky.r[0] = res.dv_lvlh[0] as f32;
            state.dsky.r[1] = res.dv_lvlh[1] as f32;
            state.dsky.r[2] = res.dv_lvlh[2] as f32;
            state.dsky.flashing = false;
        }
    }

    P34_PRIORITY
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::kepler::kepler_step;
    use crate::math::lambert::lambert;
    use crate::navigation::gravity::MU_EARTH;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::programs::p33::{compute_lambert_intercept, ALARM_P33_DEGENERATE, ALARM_P33_NO_TIG};
    use crate::types::{Met, Vec3};
    use crate::AgcState;

    const R_LEO: f64 = 6_778_000.0; // 400 km circular LEO radius (m)

    fn v_circ(r: f64) -> f64 {
        libm::sqrt(MU_EARTH / r)
    }

    /// Set up a state with chaser and target for P34 tests.
    fn make_state(
        r_c: Vec3,
        v_c: Vec3,
        r_t: Vec3,
        v_t: Vec3,
        chaser_epoch_s: f64,
        target_epoch_s: f64,
    ) -> AgcState {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: r_c,
            velocity: v_c,
            epoch: Met::from_seconds(chaser_epoch_s),
            frame: Frame::EarthInertial,
        };
        state.rendezvous_nav.target_pos = r_t;
        state.rendezvous_nav.target_vel = v_t;
        state.rendezvous_nav.target_epoch = target_epoch_s;
        state.time = Met::from_seconds(chaser_epoch_s);
        state
    }

    // ── TC-P34-1: near-zero midcourse correction (chaser on correct Lambert track)
    //
    // 1. Start with TC-P33-1 geometry.
    // 2. Get Lambert departure velocity.
    // 3. Propagate chaser (post-CSI velocity) forward 300 s (halfway).
    // 4. Call compute_lambert_intercept with propagated state and tof = 300 s.
    // 5. dv_mag < 1e-3 m/s (chaser is on the ideal trajectory).
    #[test]
    fn tc_p34_1_near_zero_midcourse() {
        let r_c0: Vec3 = [R_LEO, 0.0, 0.0];
        let vc = v_circ(R_LEO);
        let r_t0: Vec3 = [R_LEO, 10_000.0, 0.0];
        let v_t0: Vec3 = [0.0, vc, 0.0];
        let tof_full = 600.0_f64;

        // Arrival point: propagate target to TIG + tof_full.
        let (r_t_arrive, _) = kepler_step(r_t0, v_t0, tof_full, MU_EARTH);

        // Get Lambert departure velocity for the full transfer.
        let (v_depart, _) = lambert(r_c0, r_t_arrive, tof_full, MU_EARTH, true);

        // Propagate chaser forward 300 s on the Lambert arc.
        let half_tof = 300.0_f64;
        let (r_c_mid, v_c_mid) = kepler_step(r_c0, v_depart, half_tof, MU_EARTH);

        // Remaining transfer time to the same arrival point.
        let tof_remaining = tof_full - half_tof;

        let result =
            compute_lambert_intercept(r_c_mid, v_c_mid, r_t_arrive, tof_remaining, MU_EARTH)
                .expect("TC-P34-1: compute_lambert_intercept should succeed");

        // The Lambert solver's TOL_NDIM ≈ 1e-5 (non-dimensional) translates
        // to ~1 mm/s residual in this geometry after propagating through half
        // the transfer. 10 mm/s is a generous bound that still distinguishes
        // the "ideal track" case (< 10 mm/s) from the perturbed case (TC-P34-2,
        // ≥ 0.1 m/s).
        assert!(
            result.dv_mag < 1.0e-2,
            "TC-P34-1: dv_mag = {} m/s, expected < 1e-2 m/s (chaser on ideal track)",
            result.dv_mag
        );
    }

    // ── TC-P34-2: midcourse correction for a 1 m/s in-track perturbation ─────
    //
    // Same setup as TC-P34-1 but after propagating to the midpoint, perturb the
    // chaser velocity by +1 m/s in the Y direction. The correction should be
    // non-trivial but within physical range: dv_mag in [0.1, 5.0] m/s.
    #[test]
    fn tc_p34_2_midcourse_for_1ms_perturbation() {
        let r_c0: Vec3 = [R_LEO, 0.0, 0.0];
        let vc = v_circ(R_LEO);
        let r_t0: Vec3 = [R_LEO, 10_000.0, 0.0];
        let v_t0: Vec3 = [0.0, vc, 0.0];
        let tof_full = 600.0_f64;

        let (r_t_arrive, _) = kepler_step(r_t0, v_t0, tof_full, MU_EARTH);
        let (v_depart, _) = lambert(r_c0, r_t_arrive, tof_full, MU_EARTH, true);

        let half_tof = 300.0_f64;
        let (r_c_mid, v_c_mid) = kepler_step(r_c0, v_depart, half_tof, MU_EARTH);

        // Perturb: +1 m/s in-track (Y direction).
        let v_c_perturbed: Vec3 = [v_c_mid[0], v_c_mid[1] + 1.0, v_c_mid[2]];

        let tof_remaining = tof_full - half_tof;

        let result =
            compute_lambert_intercept(r_c_mid, v_c_perturbed, r_t_arrive, tof_remaining, MU_EARTH)
                .expect("TC-P34-2: compute_lambert_intercept should succeed");

        assert!(
            result.dv_mag >= 0.1 && result.dv_mag <= 5.0,
            "TC-P34-2: dv_mag = {} m/s, expected in [0.1, 5.0] m/s for 1 m/s perturbation",
            result.dv_mag
        );
    }

    // ── TC-P34-3: P34 without prior P33 (tpi_arrival_epoch == None) → alarm ──
    //
    // p34_init must raise alarm 01441 (ALARM_P33_NO_TIG) and leave
    // pending_maneuver unchanged when tpi_arrival_epoch is None.
    #[test]
    fn tc_p34_3_no_prior_p33_alarms() {
        let r_c: Vec3 = [R_LEO, 0.0, 0.0];
        let vc = v_circ(R_LEO);
        let v_c: Vec3 = [0.0, vc, 0.0];
        // Target 10 km away so the target-pos guard passes.
        let r_t: Vec3 = [R_LEO, 10_000.0, 0.0];
        let v_t: Vec3 = [0.0, vc, 0.0];

        let epoch_s = 1000.0_f64;
        let mut state = make_state(r_c, v_c, r_t, v_t, epoch_s, epoch_s);
        // tpi_arrival_epoch is None (P33 never ran).
        state.vn.pending_tig = Some(Met::from_seconds(epoch_s));

        p34_init(&mut state);

        assert!(state.alarm.lit, "TC-P34-3: alarm must be lit");
        assert_eq!(
            state.alarm.code, ALARM_P33_NO_TIG,
            "TC-P34-3: alarm code should be ALARM_P33_NO_TIG (0o01441)"
        );
        assert!(
            state.pending_maneuver.is_none(),
            "TC-P34-3: pending_maneuver must remain None"
        );
    }

    // ── TC-P34-4: dt_midcourse too small (arrival already passed) → alarm ────
    //
    // Set tpi_arrival_epoch just 5 s after the P34 TIG so dt_midcourse = 5 < 60.
    // p34_init must raise alarm 01443 (ALARM_P33_DEGENERATE).
    #[test]
    fn tc_p34_4_arrival_already_passed_alarms() {
        let r_c: Vec3 = [R_LEO, 0.0, 0.0];
        let vc = v_circ(R_LEO);
        let v_c: Vec3 = [0.0, vc, 0.0];
        let r_t: Vec3 = [R_LEO, 10_000.0, 0.0];
        let v_t: Vec3 = [0.0, vc, 0.0];

        let epoch_s = 1000.0_f64;
        let tig_s = 95.0_f64; // P34 TIG
        let arrival_s = 100.0_f64; // only 5 s after TIG — below TPI_MIN_TOF_S

        let mut state = make_state(r_c, v_c, r_t, v_t, epoch_s, epoch_s);
        // Set chaser epoch to tig_s so dt_chaser = 0.
        state.csm_state.epoch = Met::from_seconds(tig_s);
        state.tpi_arrival_epoch = Some(arrival_s);
        state.vn.pending_tig = Some(Met::from_seconds(tig_s));

        p34_init(&mut state);

        assert!(state.alarm.lit, "TC-P34-4: alarm must be lit");
        assert_eq!(
            state.alarm.code, ALARM_P33_DEGENERATE,
            "TC-P34-4: alarm code should be ALARM_P33_DEGENERATE (0o01443)"
        );
        assert!(
            state.pending_maneuver.is_none(),
            "TC-P34-4: pending_maneuver must remain None"
        );
    }

    // ── TC-P34-5: chaser within TPM_MIN_RANGE_M of target → alarm ────────────
    //
    // Place target at only 50 m from chaser. p34_init must raise alarm 01445
    // (ALARM_P34_TOO_CLOSE) before attempting Lambert.
    #[test]
    fn tc_p34_5_too_close_alarms() {
        let r_c: Vec3 = [R_LEO, 0.0, 0.0];
        let vc = v_circ(R_LEO);
        let v_c: Vec3 = [0.0, vc, 0.0];
        // Target only 50 m away (below TPM_MIN_RANGE_M = 100 m).
        let r_t: Vec3 = [R_LEO, 50.0, 0.0];
        let v_t: Vec3 = [0.0, vc, 0.0];

        let epoch_s = 1000.0_f64;
        let arrival_s = epoch_s + 600.0;

        let mut state = make_state(r_c, v_c, r_t, v_t, epoch_s, epoch_s);
        state.tpi_arrival_epoch = Some(arrival_s);
        state.vn.pending_tig = Some(Met::from_seconds(epoch_s));

        p34_init(&mut state);

        assert!(state.alarm.lit, "TC-P34-5: alarm must be lit");
        assert_eq!(
            state.alarm.code, ALARM_P34_TOO_CLOSE,
            "TC-P34-5: alarm code should be ALARM_P34_TOO_CLOSE (0o01445)"
        );
        assert!(
            state.pending_maneuver.is_none(),
            "TC-P34-5: pending_maneuver must remain None"
        );
    }
}
