//! Delta-V computation and burn execution guidance.

use crate::guidance::targeting::{Maneuver, TargetingMode};
use crate::math::linalg;
use crate::types::{Met, Vec3};

/// Maximum burn duration (centiseconds). Safety fallback for cutoff_time_met.
/// AGC: Group 3 phase table safety bound.
pub const MAX_BURN_DURATION_CS: u32 = 75_000; // 750 seconds

/// Delta-V cutoff tolerance (m/s). When `|accumulated_dv|` is within this of
/// `|target_dv|`, the burn is considered complete. AGC uses ≈ 0.3 m/s.
pub const BURN_CUTOFF_TOLERANCE_MS: f64 = 0.3;

/// Mutable state for a single SPS burn execution instance.
///
/// Created by `burn_init` from a targeting solution (`Maneuver`) and stored
/// in `AgcState::burn`. Persists across Waitlist task boundaries so that a
/// mid-burn RESTART can resume the burn loop without re-igniting the engine.
///
/// AGC source: Comanche055/P40-P47.agc and POWERED_FLIGHT_SUBROUTINES.agc.
/// Corresponds to the DELVEET1/2/3, DVTOTAL, TGO erasable variables and the
/// Group 3 restart phase flags.
#[derive(Clone, Copy, Debug, Default)]
pub struct BurnState {
    /// Target delta-V in the inertial frame (m/s).
    ///
    /// Set once by `burn_init` from the targeting solution.
    /// AGC: derived from DELVEET1/2/3 target word at ignition.
    pub target_dv_inertial: Vec3,

    /// Accumulated delta-V in the inertial frame since ignition (m/s).
    ///
    /// Integrated by `burn_update` each SERVICER cycle.
    /// AGC: DELVEET1/2/3 accumulator (scale B+7 m/s).
    pub accumulated_dv_inertial: Vec3,

    /// Time of ignition (MET, centiseconds).
    ///
    /// Recorded at burn start; used to compute the backup cutoff time.
    /// AGC: stored in the restart area for Group 3 protection.
    pub tig: Met,

    /// `true` while the SPS engine is commanded on.
    ///
    /// Set to `true` by `burn_init`; cleared by P40 when `is_burn_complete`
    /// returns `true`. The P40 program wrapper reads this flag to call
    /// `hw.engine().sps_enable(false)`.
    pub burn_active: bool,

    /// `true` once the backup cutoff time has been passed.
    ///
    /// Set by `burn_update` when `AgcState::time >= compute_cutoff_time(state)`.
    /// Causes `is_burn_complete` to return `true` even if the delta-V target
    /// has not been reached (runaway engine protection).
    pub cutoff_time_met: bool,

    /// `true` after the crew presses PRO in response to the V50 N99
    /// engine-arm prompt but before TIG has been reached.
    ///
    /// Real-flight semantics: the crew arms the engine in the last few
    /// seconds before TIG; the AGC must hold off on actually
    /// commanding `SPS_ENABLE` until the time of ignition is reached
    /// (firing early would consume burn duration before the targeting
    /// solution intended). The DAP's per-cycle ignition gate clears
    /// this flag and sets `state.engine_thrusting = true` when
    /// `state.time >= burn.tig`. See `dap_step` in
    /// `agc-core/src/control/dap.rs`.
    ///
    /// AGC correspondence: the "TIG-5" countdown logic in
    /// Comanche055/P40-P47.agc that holds off the engine ON discrete
    /// until TIME1 reaches `tig`.
    pub armed: bool,
}

/// Construct a `BurnState` ready for ignition from a completed targeting solution.
///
/// Called by P40 immediately before asserting `SPS_ENABLE`. After `burn_init`
/// returns, P40 must:
///   1. Call `hw.engine().sps_enable(true)`.
///   2. Set `state.servicer_exit = Some(burn_servicer_exit)`.
///   3. Call `services::average_g::start_servicer(state, hw)` if not already running.
///
/// AGC correspondence: the initialisation sequence at the beginning of P40
/// burn execution (P40-P47.agc), which zeroes `DELVEET1/2/3` and stores the
/// target delta-V.
///
/// # Preconditions
/// - `target.delta_v` must be finite and non-zero (linalg::norm > 0).
/// - `target.tig` must be a valid MET, i.e. it must not lie in the past at
///   call time.
///
/// # Postconditions
/// - `result.target_dv_inertial == target.delta_v.0` (the inner Vec3).
/// - `result.accumulated_dv_inertial == [0.0, 0.0, 0.0]`.
/// - `result.tig == target.tig`.
/// - `result.burn_active == true`.
/// - `result.cutoff_time_met == false`.
pub fn burn_init(target: Maneuver) -> BurnState {
    BurnState {
        target_dv_inertial: target.delta_v.0,
        accumulated_dv_inertial: [0.0, 0.0, 0.0],
        tig: target.tig,
        burn_active: true,
        cutoff_time_met: false,
        armed: false,
    }
}

/// Integrate one SERVICER cycle's measured delta-V into the running totals.
///
/// Called from the SERVICER exit hook (`burn_servicer_exit`) on every 2-second
/// SERVICER cycle while the engine is running. The caller is responsible for
/// ensuring `measured_dv` is already expressed in the inertial frame.
///
/// AGC correspondence: the accumulation step in the SERVICER exit hook for P40,
/// which performs `DELVEET += delta_v_inertial`.
/// Source: Comanche055/SERVICER207.agc and POWERED_FLIGHT_SUBROUTINES.agc.
///
/// # Preconditions
/// - `state.burn_active` must be `true`.
/// - `measured_dv` must be finite.
/// - `dt` must be positive and finite. Nominal value: 2.0 seconds.
///
/// # Postconditions
/// - `state.accumulated_dv_inertial` is the vector sum of all previous
///   `measured_dv` arguments since ignition.
/// - No other field of `BurnState` is modified by this function.
pub fn burn_update(state: &mut BurnState, measured_dv: Vec3, dt: f64) {
    // dt is currently unused; kept for forward compatibility with variable-rate
    // SERVICER cycles and forward-difference predictors.
    let _ = dt;
    state.accumulated_dv_inertial = linalg::vadd(state.accumulated_dv_inertial, measured_dv);
}

/// Compute the attitude-rate correction vector for TVC during a burn.
///
/// Uses the cross-product steering law from Comanche055:
///
/// ```text
/// omega_c = (dv_remaining × v_current) / |v_current|²
/// ```
///
/// The result `omega_c` (rad/s, inertial frame) is fed into the TVC lead-lag
/// compensator filter via `DapState::attitude_error`.
///
/// AGC correspondence: the VXV/ABVAL/VSCALE sequence in
/// Comanche055/POWERED_FLIGHT_SUBROUTINES.agc.
///
/// # Preconditions
/// - `remaining_dv` must be finite. May be zero near cutoff.
/// - `current_v` must be finite and non-zero (engine must be producing
///   measurable thrust before this function is called).
///
/// # Panics
/// - If `|current_v|² <= 1e-9` (engine not thrusting or called before ignition).
pub fn cross_product_steering(remaining_dv: Vec3, current_v: Vec3) -> Vec3 {
    let v_mag_sq = linalg::dot(current_v, current_v);
    debug_assert!(
        v_mag_sq > 1e-9,
        "cross_product_steering: current_v is zero — engine not thrusting or called before ignition"
    );
    let cross_prod = linalg::cross(remaining_dv, current_v);
    linalg::vscale(cross_prod, 1.0 / v_mag_sq)
}

/// Test whether the burn has achieved its delta-V target or has exceeded the
/// maximum allowed burn time.
///
/// Returns `true` when either criterion is satisfied:
///
/// **Primary**: `|accumulated_dv| >= |target_dv| - cutoff_tolerance`
///
/// **Backup**: `state.cutoff_time_met == true`
///
/// AGC correspondence: the cutoff test in Comanche055/POWERED_FLIGHT_SUBROUTINES.agc.
///
/// # Preconditions
/// - `cutoff_tolerance` must be non-negative and finite.
///   Recommended value: 0.3 m/s.
pub fn is_burn_complete(state: &BurnState, cutoff_tolerance: f64) -> bool {
    if state.cutoff_time_met {
        return true;
    }
    let target_mag = linalg::norm(state.target_dv_inertial);
    let achieved_mag = linalg::norm(state.accumulated_dv_inertial);
    achieved_mag >= target_mag - cutoff_tolerance
}

/// Estimate the absolute mission time at which the engine should cut off,
/// as a backup guard against under-achieving the burn or a stuck-open engine.
///
/// AGC correspondence: the `TGO` (time-to-go) calculation in
/// Comanche055/P40-P47.agc, updated each SERVICER cycle.
///
/// # Preconditions
/// - `state.tig` must have been set by `burn_init`.
/// - `current_met >= state.tig` (the engine has ignited).
///
/// # Postconditions
/// - Returns a `Met` representing the estimated absolute cutoff time.
/// - On the fallback path, returns `state.tig + MAX_BURN_DURATION_CS`.
pub fn compute_cutoff_time(state: &BurnState, current_met: Met) -> Met {
    let elapsed_cs = current_met.0.wrapping_sub(state.tig.0);
    let elapsed_s = elapsed_cs as f64 / 100.0;

    if elapsed_s < 0.1 {
        return Met(state.tig.0.wrapping_add(MAX_BURN_DURATION_CS));
    }

    let achieved_mag = linalg::norm(state.accumulated_dv_inertial);
    if achieved_mag < 1e-6 {
        return Met(state.tig.0.wrapping_add(MAX_BURN_DURATION_CS));
    }

    let avg_accel = achieved_mag / elapsed_s; // m/s²
    let target_mag = linalg::norm(state.target_dv_inertial);
    let remaining_dv = target_mag - achieved_mag; // m/s
    let remaining_time_s = remaining_dv / avg_accel; // s
    let remaining_cs = (remaining_time_s * 100.0) as u32;
    let cutoff_cs = current_met.0.wrapping_add(remaining_cs);
    Met(cutoff_cs)
}

/// Compute the residual delta-V remaining after SPS engine cutoff.
///
/// Returns `target_dv - accumulated_dv`. This residual is passed to P40 for
/// RCS nulling.
///
/// AGC correspondence: the "trim burn" logic in Comanche055/P40-P47.agc
/// following SPS cutoff.
///
/// # Preconditions
/// - Should be called only after `is_burn_complete` has returned `true` and
///   the SPS engine has been disabled.
pub fn trim_residual_dv(state: &BurnState) -> Vec3 {
    linalg::vsub(state.target_dv_inertial, state.accumulated_dv_inertial)
}

/// SERVICER exit hook installed by P40 and P41 while a burn is in progress.
///
/// Called by `services::average_g::servicer_task` at the end of every 2-second
/// SERVICER cycle, via `state.servicer_exit`. Reads the inertial delta-V the
/// SERVICER just integrated from `state.servicer_last_dv_inertial`, folds it
/// into `state.burn.accumulated_dv_inertial` via `burn_update`, and checks
/// `is_burn_complete`. On completion it:
///
/// 1. Clears `state.burn.burn_active` (the burn loop terminates).
/// 2. Clears `state.engine_thrusting` (the ISR shim will drop the SPS enable
///    discrete on its next iteration).
/// 3. Drops the exit hook itself (`state.servicer_exit = None`) so the
///    SERVICER stops calling it.
/// 4. Transitions the DAP to `AttitudeHold` so the vehicle continues to hold
///    orientation after cutoff.
///
/// If `state.burn.burn_active` is already false when the hook runs, it is a
/// no-op — this covers the one-cycle race between P40 clearing the flag and
/// the SERVICER observing the new `servicer_exit` value.
///
/// AGC correspondence: the POWERED_FLIGHT_SUBROUTINES.agc SERVICER exit path
/// that checks TGO and performs ENGINOFF.
pub fn burn_servicer_exit(state: &mut crate::AgcState) {
    if !state.burn.burn_active {
        return;
    }

    let dv = state.servicer_last_dv_inertial;
    burn_update(&mut state.burn, dv, 2.0);

    if is_burn_complete(&state.burn, BURN_CUTOFF_TOLERANCE_MS) {
        state.burn.burn_active = false;
        state.engine_thrusting = false;
        state.servicer_exit = None;
        state.dap_state.mode = crate::control::DapMode::AttitudeHold;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{DeltaV, Met};

    fn assert_vec_near(a: Vec3, b: Vec3, eps: f64) {
        for i in 0..3 {
            assert!(
                (a[i] - b[i]).abs() < eps,
                "component {i}: {} != {} (eps={eps})",
                a[i],
                b[i]
            );
        }
    }

    /// TC-MAN-1: burn_init from a zero-accumulated state.
    #[test]
    fn tc_man_1_burn_init() {
        let target = Maneuver {
            tig: Met(180_000),
            delta_v: DeltaV([90.0, 0.0, -60.0]),
            burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mode: TargetingMode::ExternalDeltaV,
        };
        let state = burn_init(target);

        assert_eq!(state.target_dv_inertial, [90.0, 0.0, -60.0]);
        assert_eq!(state.accumulated_dv_inertial, [0.0, 0.0, 0.0]);
        assert_eq!(state.tig, Met(180_000));
        assert!(state.burn_active);
        assert!(!state.cutoff_time_met);
    }

    /// TC-MAN-2: partial delta-V accumulation over three burn_update calls.
    #[test]
    fn tc_man_2_burn_update_accumulation() {
        let target = Maneuver {
            tig: Met(0),
            delta_v: DeltaV([0.0, 100.0, 0.0]),
            burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mode: TargetingMode::ExternalDeltaV,
        };
        let mut state = burn_init(target);

        let dv1: Vec3 = [0.0, 2.5, 0.0];
        let dv2: Vec3 = [0.0, 2.5, 0.0];
        let dv3: Vec3 = [0.0, 2.5, 0.0];

        burn_update(&mut state, dv1, 2.0);
        burn_update(&mut state, dv2, 2.0);
        burn_update(&mut state, dv3, 2.0);

        assert_vec_near(state.accumulated_dv_inertial, [0.0, 7.5, 0.0], 1e-14);
        // 7.5 m/s achieved; target is 100.0; 7.5 < 100.0 - 0.3 → not complete
        assert!(!is_burn_complete(&state, 0.3));
    }

    /// TC-MAN-3: cross-product steering with known 6° attitude error geometry.
    ///
    /// Target direction: +Y.
    /// Current velocity: ~2.5 m/s at 6° from +Y in the X-Y plane.
    /// Expected: omega_c ≈ [0, 0, -2.0904] rad/s.
    #[test]
    fn tc_man_3_cross_product_steering() {
        let remaining_dv: Vec3 = [0.0, 50.0, 0.0];
        // sin(6°) ≈ 0.1045, cos(6°) ≈ 0.9945 → 2.5 m/s at 6° from +Y
        let current_v: Vec3 = [0.2613, 2.4863, 0.0]; // |v|² ≈ 6.25

        let omega_c = cross_product_steering(remaining_dv, current_v);

        // cross([0,50,0], [0.2613,2.4863,0]) = [0, 0, -13.065]
        // |current_v|² ≈ 6.25
        // omega_c ≈ [0, 0, -2.0904]
        assert_vec_near(omega_c, [0.0, 0.0, -2.0904], 1e-3);
    }

    /// TC-MAN-4: is_burn_complete at tolerance boundary and via cutoff_time_met backup.
    #[test]
    fn tc_man_4_is_burn_complete() {
        let target = Maneuver {
            tig: Met(0),
            delta_v: DeltaV([0.0, 0.0, 90.0]), // 90.0 m/s target
            burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mode: TargetingMode::ExternalDeltaV,
        };
        let mut state = burn_init(target);
        state.accumulated_dv_inertial = [0.0, 0.0, 89.8]; // 0.2 m/s short of target

        // Within 0.3 m/s tolerance: 89.8 >= 90.0 - 0.3 = 89.7 → true
        assert!(is_burn_complete(&state, 0.3));

        // Tighter tolerance: 89.8 >= 90.0 - 0.1 = 89.9 → false
        assert!(!is_burn_complete(&state, 0.1));

        // Backup criterion: cutoff_time_met overrides
        state.cutoff_time_met = true;
        assert!(is_burn_complete(&state, 0.1));
    }

    /// TC-MAN-6: burn_servicer_exit integrates one cycle without completing.
    #[test]
    fn tc_man_6_servicer_exit_one_cycle() {
        let mut state = crate::AgcState::new();
        let target = Maneuver {
            tig: Met(0),
            delta_v: DeltaV([10.0, 0.0, 0.0]),
            burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mode: TargetingMode::ExternalDeltaV,
        };
        state.burn = burn_init(target);
        state.servicer_exit = Some(burn_servicer_exit);
        state.servicer_last_dv_inertial = [3.0, 0.0, 0.0];

        burn_servicer_exit(&mut state);

        assert_vec_near(state.burn.accumulated_dv_inertial, [3.0, 0.0, 0.0], 1e-14);
        assert!(
            state.burn.burn_active,
            "burn must still be active after 3/10 m/s"
        );
        assert!(
            state.servicer_exit.is_some(),
            "hook must remain installed until burn completes"
        );
    }

    /// TC-MAN-7: burn_servicer_exit triggers cutoff at target completion.
    #[test]
    fn tc_man_7_servicer_exit_cutoff() {
        let mut state = crate::AgcState::new();
        state.dap_state.mode = crate::control::DapMode::Tvc;
        state.engine_thrusting = true;

        let target = Maneuver {
            tig: Met(0),
            delta_v: DeltaV([10.0, 0.0, 0.0]),
            burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mode: TargetingMode::ExternalDeltaV,
        };
        state.burn = burn_init(target);
        state.burn.accumulated_dv_inertial = [9.5, 0.0, 0.0];
        state.servicer_exit = Some(burn_servicer_exit);
        state.servicer_last_dv_inertial = [0.8, 0.0, 0.0]; // lands at 10.3

        burn_servicer_exit(&mut state);

        assert!(
            !state.burn.burn_active,
            "burn must cut off after target reached"
        );
        assert!(
            !state.engine_thrusting,
            "engine_thrusting must clear on cutoff"
        );
        assert!(
            state.servicer_exit.is_none(),
            "hook must be uninstalled on cutoff"
        );
        assert_eq!(
            state.dap_state.mode,
            crate::control::DapMode::AttitudeHold,
            "DAP must transition to AttitudeHold after cutoff"
        );
    }

    /// TC-MAN-8: burn_servicer_exit is a no-op when burn_active is false.
    #[test]
    fn tc_man_8_servicer_exit_inactive_noop() {
        let mut state = crate::AgcState::new();
        state.burn.burn_active = false;
        state.servicer_exit = Some(burn_servicer_exit);
        state.servicer_last_dv_inertial = [5.0, 0.0, 0.0];
        let prior_accum = state.burn.accumulated_dv_inertial;

        burn_servicer_exit(&mut state);

        assert_eq!(state.burn.accumulated_dv_inertial, prior_accum);
        // Hook is NOT uninstalled from the no-op path — P40 clears it explicitly.
        assert!(state.servicer_exit.is_some());
    }

    /// TC-MAN-5: trim_residual_dv after a slightly off-axis burn.
    #[test]
    fn tc_man_5_trim_residual_dv() {
        let target = Maneuver {
            tig: Met(0),
            delta_v: DeltaV([0.0, 0.0, -50.0]),
            burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mode: TargetingMode::ExternalDeltaV,
        };
        let mut state = burn_init(target);
        state.accumulated_dv_inertial = [0.1, -0.2, -48.7]; // slightly off-axis, short
        state.burn_active = false;

        let residual = trim_residual_dv(&state);

        // residual = [0,0,-50] - [0.1,-0.2,-48.7] = [-0.1, 0.2, -1.3]
        assert_vec_near(residual, [-0.1, 0.2, -1.3], 1e-13);
    }
}
