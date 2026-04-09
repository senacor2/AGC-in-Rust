//! SERVICER — the 2-second navigation cycle (Average-G integration).
//!
//! Reads PIPA delta-V counts from the staging field `AgcState::pipa_counts`
//! (written by the T3RUPT handler or hardware shim before dispatch — Strategy B
//! from the spec §6.3), compensates them for calibration errors, rotates the
//! resulting delta-V into the inertial reference frame via REFSMMAT, and drives
//! `navigation::integration::average_g_step`.
//!
//! # Hardware access design choice
//!
//! Waitlist tasks have the signature `fn(&mut AgcState)` — no hardware parameter.
//! This module uses **Strategy B** (spec §6.3): the T3RUPT handler reads the PIPA
//! counters and stores the raw counts in `AgcState::pipa_counts` before calling
//! `Waitlist::dispatch`. `servicer_task` reads from that staging field, so it
//! needs no direct hardware access. Rescheduling only needs `Waitlist::schedule`,
//! which is available via `state.waitlist`; the T3RUPT handler is responsible for
//! calling `hw.timers().arm_t3` after `dispatch` returns the next delta.
//!
//! # AGC source references
//!
//! - `Comanche055/AVERAGE_G_INTEGRATOR.agc` — SERVICER entry point, PIPA read
//! - `Comanche055/ERASABLE_ASSIGNMENTS.agc` — PIPAX/Y/Z addresses, NBDX/Y/Z
//! - `Comanche055/INTEGRATION_INITIALIZATION.agc` — body selection, PIPA scale

use crate::executive::{ScheduleResult, GROUP_2};
use crate::executive::restart::Phase;
use crate::math::linalg::mxv;
use crate::navigation::integration::average_g_step;
use crate::types::Vec3;
use crate::AgcState;

// ── PipaCalibration ───────────────────────────────────────────────────────────

/// PIPA (accelerometer) calibration constants.
///
/// In Comanche055 these are stored in erasable memory (E1 bank) and loaded
/// from the fixed-memory constant tables at program start or updated by uplink.
///
/// AGC source: `Comanche055/AVERAGE_G_INTEGRATOR.agc` — NBDX/NBDY/NBDZ and
/// `1/PIPADT` constant entries.
#[derive(Clone, Copy, Debug)]
pub struct PipaCalibration {
    /// PIPA scale factor: metres per second per raw count.
    ///
    /// Nominal value ≈ 0.0585 m/s/count.
    /// AGC name: 1/PIPADT (inverse of PIPA delta-time constant).
    /// Stored as double-precision fixed-point in erasable; converted to f64 here.
    pub scale: f64,

    /// Bias (zero-offset drift) in counts per 2-second interval for each axis.
    ///
    /// AGC names: NBDX (index 0), NBDY (index 1), NBDZ (index 2).
    /// These are subtracted from the raw counts before scaling.
    /// Units: counts per 2-second SERVICER interval (not counts/second).
    /// Nominal value: 0 (perfectly calibrated instrument).
    /// Typical flight value: small integer, order 1–5 counts/interval.
    pub bias: [i16; 3],

    /// PIPA misalignment compensation matrix.
    ///
    /// A 3×3 matrix applied after bias removal and scale-factor multiplication.
    /// The diagonal is 1.0; off-diagonal elements are small (< 1×10⁻³ rad).
    /// For a perfectly aligned instrument this is the identity matrix.
    /// AGC source: AVERAGE_G_INTEGRATOR.agc misalignment table entries.
    pub misalignment: [[f64; 3]; 3],
}

impl PipaCalibration {
    /// Nominal (uncalibrated) constants. Used at FRESH START.
    pub const NOMINAL: Self = Self {
        scale: 0.0585,          // m/s per count, approximate
        bias: [0, 0, 0],        // no bias correction
        misalignment: [         // identity (no misalignment correction)
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
    };
}

// ── SERVICER active flag ──────────────────────────────────────────────────────

/// Bit position in `flagwords[0]` that indicates the SERVICER is active.
/// Set by `start_servicer`; cleared by `stop_servicer`.
/// AGC correspondence: AVEGFLAG or equivalent bit in FLAGWRD0.
pub const SERVICER_ACTIVE_BIT: u8 = 0;

#[inline]
fn is_servicer_active(state: &AgcState) -> bool {
    (state.flagwords[0] >> SERVICER_ACTIVE_BIT) & 1 != 0
}

#[inline]
fn set_servicer_active(state: &mut AgcState, active: bool) {
    if active {
        state.flagwords[0] |= 1 << SERVICER_ACTIVE_BIT;
    } else {
        state.flagwords[0] &= !(1u16 << SERVICER_ACTIVE_BIT);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Schedule the first SERVICER (Average-G) Waitlist task.
///
/// Safe to call when the SERVICER is already running (idempotent: the active-flag
/// check prevents double-scheduling). Sets restart group 2 to `Phase(1)` so that
/// a hardware restart re-queues the task.
///
/// The caller (T3RUPT handler or test harness) is responsible for arming the
/// timer if `Waitlist::schedule` returns `ScheduleResult::OkReloadT3`.
///
/// AGC correspondence: the WAITLIST call for "SERVICER in 2 seconds" made at
/// the entry points of the navigation programs in Comanche055.
pub fn start_servicer(state: &mut AgcState) {
    if is_servicer_active(state) {
        return; // already running; idempotent
    }
    set_servicer_active(state, true);
    state.restart.set_phase(GROUP_2, Phase(1));

    // Schedule the first cycle. The T3RUPT handler must arm TIME3 if OkReloadT3.
    // In the simulation / test environment the caller handles the arm_t3 call.
    let _result = state.waitlist.schedule(200, servicer_task);
    // In a real system: match _result { OkReloadT3(d) => hw.timers().arm_t3(d), _ => {} }
}

/// Cancel the SERVICER (Average-G) task.
///
/// Sets the stop flag so the next time `servicer_task` runs it does not reschedule
/// itself. Any currently-pending Waitlist entry fires one final time, then terminates
/// gracefully.
///
/// Safe to call when the SERVICER is not running (idempotent).
///
/// AGC correspondence: clearing AVEGFLAG in program exit sequences in Comanche055.
pub fn stop_servicer(state: &mut AgcState) {
    set_servicer_active(state, false);
    state.restart.set_phase(GROUP_2, Phase::IDLE);
    state.servicer_exit = None;
}

/// The 2-second SERVICER Waitlist task.
///
/// Registered in the Waitlist via `fn(&mut AgcState)`. Executes the full
/// PIPA compensation pipeline, integrates the state vector, optionally calls
/// the program-specific exit hook, then reschedules itself if the active flag
/// is still set.
///
/// Steps:
/// 1. Check `servicer_active` — if false, terminate without rescheduling.
/// 2. Read PIPA counts from `state.pipa_counts` staging field.
/// 3. Apply bias correction (subtract `pipa_cal.bias`).
/// 4. Apply scale factor → delta-V in m/s (stable-member frame).
/// 5. Apply misalignment correction matrix.
/// 6. Rotate to inertial frame via REFSMMAT.
/// 7. Integrate state vector: `average_g_step(csm_state, dv_inertial, 2.0, moon_pos)`.
/// 8. Write back new state vector and advance `state.time`.
/// 9. Call program-specific exit hook (if set).
/// 10. Reschedule at 200 cs.
pub fn servicer_task(state: &mut AgcState) {
    // Step 1 — check active flag; bail if stopped.
    if !is_servicer_active(state) {
        state.restart.set_phase(GROUP_2, Phase::IDLE);
        return;
    }

    // Step 2 — read raw PIPA counts from staging field.
    // Written by T3RUPT handler (Strategy B) before Waitlist::dispatch.
    let raw: [i16; 3] = state.pipa_counts;

    // PIPA overflow check: if any axis is at i16::MAX the counter wrapped.
    // In that case zero out the count for this cycle (do not corrupt navigation).
    let raw_checked: [i16; 3] = [
        if raw[0].abs() == i16::MAX { 0 } else { raw[0] },
        if raw[1].abs() == i16::MAX { 0 } else { raw[1] },
        if raw[2].abs() == i16::MAX { 0 } else { raw[2] },
    ];

    // Step 3 — apply bias correction in i32 to avoid overflow.
    let cal = state.pipa_cal;
    let biased: [i32; 3] = [
        raw_checked[0] as i32 - cal.bias[0] as i32,
        raw_checked[1] as i32 - cal.bias[1] as i32,
        raw_checked[2] as i32 - cal.bias[2] as i32,
    ];

    // Step 4 — apply scale factor (m/s per count).
    let scaled: Vec3 = [
        biased[0] as f64 * cal.scale,
        biased[1] as f64 * cal.scale,
        biased[2] as f64 * cal.scale,
    ];

    // Step 5 — apply misalignment matrix (3×3, near identity).
    let delta_v_platform: Vec3 = mxv(cal.misalignment, scaled);

    // Step 6 — rotate to inertial frame via REFSMMAT.
    let delta_v_inertial: Vec3 = mxv(state.refsmmat, delta_v_platform);

    // Step 7 — integrate the state vector.
    // Moon position: use a simplified placeholder until navigation::planetary is
    // implemented. The value [3.844e8, 0, 0] places the Moon at its mean distance
    // on the X axis; the third-body perturbation is small at LEO/LLO so the
    // approximation is adequate for initial validation.
    // TODO: replace with navigation::planetary::moon_position(state.csm_state.epoch)
    let moon_pos: Vec3 = [3.844e8, 0.0, 0.0];

    let new_sv = average_g_step(state.csm_state, delta_v_inertial, 2.0, moon_pos);

    // Step 8 — write back with restart protection.
    state.restart.set_phase(GROUP_2, Phase(-1)); // mid-update guard
    state.csm_state = new_sv;
    state.time = new_sv.epoch;
    state.restart.set_phase(GROUP_2, Phase(1));  // cycle complete

    // Step 9 — call program-specific exit hook (e.g., P40 TVC steering).
    if let Some(exit_fn) = state.servicer_exit {
        exit_fn(state);
    }

    // Step 10 — reschedule at 200 cs (2.000 s) if still active.
    if is_servicer_active(state) {
        match state.waitlist.schedule(200, servicer_task) {
            ScheduleResult::OkReloadT3(_delta) => {
                // T3RUPT handler must call hw.timers().arm_t3(delta).
                // In Strategy B this is handled externally.
            }
            ScheduleResult::Ok => {}
            ScheduleResult::Full => {
                // Waitlist full — stop the SERVICER (alarm 1211 would be raised
                // by the real system; omitted here as services::alarm depends on hw).
                set_servicer_active(state, false);
                state.restart.set_phase(GROUP_2, Phase::IDLE);
            }
        }
    } else {
        // stop_servicer was called during this cycle.
        state.restart.set_phase(GROUP_2, Phase::IDLE);
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;

    fn assert_near(a: f64, b: f64, tol: f64, label: &str) {
        assert!(
            (a - b).abs() < tol,
            "{label}: got {a:.6}, expected {b:.6}, tolerance {tol:.2e}"
        );
    }

    // ── TC-AG-1: Zero bias — raw counts map directly to delta-V ──────────────

    /// TC-AG-1: With zero bias, identity misalignment, identity REFSMMAT, and
    /// scale = 1.0 m/s/count, the PIPA counts [10, -5, 3] produce a delta-V
    /// of exactly [10, -5, 3] m/s in the inertial frame.
    ///
    /// We verify this by driving `servicer_task` directly through the staging
    /// field and inspecting the velocity change (gravity contribution is small
    /// but non-zero; we isolate the delta-V by comparing before/after with a
    /// zero-gravity reference).
    #[test]
    fn tc_ag_1_zero_bias_raw_counts_as_delta_v() {
        let mut state = AgcState::new();

        // Nominal calibration with scale = 1.0 for easy arithmetic.
        state.pipa_cal = PipaCalibration {
            scale: 1.0,
            bias: [0, 0, 0],
            misalignment: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        };
        // Identity REFSMMAT: platform = inertial.
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        // Put the spacecraft in an orbit so gravity is finite.
        state.csm_state = StateVector {
            position: [0.0, 0.0, 7_000_000.0],
            velocity: [7500.0, 0.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        let v_before = state.csm_state.velocity;

        // Inject PIPA counts.
        state.pipa_counts = [10, -5, 3];

        // Activate and run one SERVICER cycle.
        start_servicer(&mut state);
        // Simulate the dispatch: run servicer_task directly
        // (in the real system, the T3RUPT handler calls waitlist.dispatch).
        servicer_task(&mut state);

        // To isolate the PIPA contribution, run a second reference cycle with
        // zero PIPA counts from the same initial state.
        let mut ref_state = AgcState::new();
        ref_state.pipa_cal = state.pipa_cal;
        ref_state.refsmmat = state.refsmmat;
        ref_state.csm_state = StateVector {
            position: [0.0, 0.0, 7_000_000.0],
            velocity: [7500.0, 0.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        ref_state.pipa_counts = [0, 0, 0];
        set_servicer_active(&mut ref_state, true);
        ref_state.restart.set_phase(GROUP_2, Phase(1));
        servicer_task(&mut ref_state);

        // The difference between the two results isolates the delta-V contribution.
        let dv_x = state.csm_state.velocity[0] - ref_state.csm_state.velocity[0];
        let dv_y = state.csm_state.velocity[1] - ref_state.csm_state.velocity[1];
        let dv_z = state.csm_state.velocity[2] - ref_state.csm_state.velocity[2];
        assert_near(dv_x, 10.0, 1e-3, "TC-AG-1: delta-V X (10 m/s)");
        assert_near(dv_y, -5.0, 1e-3, "TC-AG-1: delta-V Y (-5 m/s)");
        assert_near(dv_z, 3.0, 1e-3, "TC-AG-1: delta-V Z (3 m/s)");
    }

    // ── TC-AG-3: REFSMMAT rotation ────────────────────────────────────────────

    /// TC-AG-3: A 90° rotation REFSMMAT correctly rotates the platform delta-V
    /// to the inertial frame.
    ///
    /// With REFSMMAT rotating platform-X to inertial-Y (90° about Z), PIPA
    /// counts [1, 0, 0] (after scale=1) should produce delta-V [0, 1, 0] in
    /// the inertial frame.
    #[test]
    fn tc_ag_3_refsmmat_rotation() {
        let mut state = AgcState::new();

        state.pipa_cal = PipaCalibration {
            scale: 1.0,
            bias: [0, 0, 0],
            misalignment: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        };
        // 90° rotation about Z: platform-X → inertial-Y, platform-Y → inertial-(-X)
        state.refsmmat = [
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0],
        ];

        // Use a position along X so gravity is purely along -X.
        // That way we can isolate the Y contribution from PIPA.
        state.csm_state = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        let v_before = state.csm_state.velocity;

        // PIPA count of [1, 0, 0] in the platform frame.
        state.pipa_counts = [1, 0, 0];
        start_servicer(&mut state);
        servicer_task(&mut state);

        // Isolate delta-V by comparing against a zero-PIPA reference run.
        let mut ref_state = AgcState::new();
        ref_state.pipa_cal = PipaCalibration {
            scale: 1.0,
            bias: [0, 0, 0],
            misalignment: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        };
        ref_state.refsmmat = state.refsmmat;
        ref_state.csm_state = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        ref_state.pipa_counts = [0, 0, 0];
        set_servicer_active(&mut ref_state, true);
        ref_state.restart.set_phase(GROUP_2, Phase(1));
        servicer_task(&mut ref_state);

        // REFSMMAT rotates platform-X → inertial-Y. So [1,0,0] platform → [0,1,0] inertial.
        let dv_y = state.csm_state.velocity[1] - ref_state.csm_state.velocity[1];
        let dv_x = state.csm_state.velocity[0] - ref_state.csm_state.velocity[0];
        assert_near(dv_y, 1.0, 1e-3, "TC-AG-3: rotated delta-V Y = 1 m/s");
        assert_near(dv_x, 0.0, 1e-3, "TC-AG-3: rotated delta-V X = 0 m/s");
    }

    // ── TC-AG-4: Free-fall propagation ────────────────────────────────────────

    /// TC-AG-4: With zero PIPA counts (no thrust), the SERVICER integrates
    /// the state vector under gravity only. After one 2-second cycle, the
    /// velocity change should match the Average-G gravity integration.
    #[test]
    fn tc_ag_4_free_fall_propagation() {
        use crate::navigation::gravity::earth_gravity;

        let mut state = AgcState::new();

        state.pipa_cal = PipaCalibration::NOMINAL;
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        // Spacecraft at rest directly above Earth: free-fall along -X.
        let r0 = 7_000_000.0_f64;
        state.csm_state = StateVector {
            position: [r0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        state.pipa_counts = [0, 0, 0];

        start_servicer(&mut state);
        servicer_task(&mut state);

        // With zero PIPA and moon_pos=[0,0,0] placeholder the integration is
        // pure gravity. Verify the velocity is approximately g_x * dt.
        let g0 = earth_gravity([r0, 0.0, 0.0]);
        // After 2s, velocity ≈ g0 * dt (trapezoidal, but g barely changes)
        let expected_vx = g0[0] * 2.0;
        assert_near(
            state.csm_state.velocity[0],
            expected_vx,
            0.1,
            "TC-AG-4: free-fall velocity[0]"
        );
        // Y and Z should remain near zero.
        assert_near(state.csm_state.velocity[1], 0.0, 1e-6, "TC-AG-4: velocity[1]");
        assert_near(state.csm_state.velocity[2], 0.0, 1e-6, "TC-AG-4: velocity[2]");
    }

    // ── Lifecycle: start/stop flag behaviour ──────────────────────────────────

    /// Verify that start_servicer is idempotent and stop_servicer clears the flag.
    #[test]
    fn tc_ag_lifecycle_start_stop() {
        let mut state = AgcState::new();
        assert!(!is_servicer_active(&state), "should be inactive initially");

        start_servicer(&mut state);
        assert!(is_servicer_active(&state), "should be active after start");
        assert_eq!(state.waitlist.len(), 1, "one waitlist entry after start");

        // Second start must be idempotent.
        start_servicer(&mut state);
        assert_eq!(state.waitlist.len(), 1, "idempotent: still one entry");

        stop_servicer(&mut state);
        assert!(!is_servicer_active(&state), "should be inactive after stop");
        assert!(state.servicer_exit.is_none(), "exit hook cleared by stop");
    }

    /// Verify that servicer_task does not reschedule itself after stop_servicer.
    #[test]
    fn tc_ag_task_stops_when_inactive() {
        let mut state = AgcState::new();
        state.pipa_counts = [0, 0, 0];
        state.csm_state = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        // Start, then immediately stop before the task runs.
        start_servicer(&mut state);
        stop_servicer(&mut state);

        // The Waitlist still has one pending entry (from start_servicer), but
        // when it fires it will find the flag clear and not reschedule.
        let len_before = state.waitlist.len();
        servicer_task(&mut state);
        // After one run, the task should NOT have added another entry.
        // The waitlist count stays the same (entry was already there from start;
        // servicer_task does not call schedule when inactive).
        // Since we call servicer_task directly (not via dispatch), the waitlist
        // still has the original entry. No new entry added.
        assert_eq!(
            state.waitlist.len(),
            len_before,
            "inactive servicer must not add a new entry"
        );
    }
}
