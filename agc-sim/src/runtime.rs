//! "Soft executive" helpers for host-side simulation.
//!
//! On the bare-metal target, [`agc_core::executive::Executive::run`]
//! drives everything: it pre-reads CDU/PIPA before each waitlist
//! dispatch, mirrors `state.engine_thrusting` and `state.rcs_commanded_*`
//! through the HAL, and reloads T3 after every task. That entry point
//! never returns and pulls in the `AgcHardware` trait machinery, so the
//! hosted [`crate::SimHardware`] simulator cannot call it.
//!
//! This module re-creates just the parts of `Executive::run` that a
//! host integration test or interactive simulator (`dsky_sim`) needs to
//! make the AGC actually do something:
//!
//! - [`pump_pipa_into_state`] — drain `SimImu::pipa` into
//!   `AgcState::pipa_counts` (the foreground PIPA accumulator).
//! - [`pump_engine_to_hw`] / [`pump_rcs_to_hw`] / [`pump_secs_to_hw`] —
//!   mirror staging fields to the simulated hardware, the same translation
//!   `process_engine_staging` / `process_rcs_staging` / `process_secs_staging`
//!   perform on the bare-metal scheduler.
//! - [`WaitlistPump`] — dispatches waitlist tasks at their mission-time
//!   deadlines, reading PIPA/CDU before each task to mirror what
//!   `Executive::run` does on T3RUPT.
//! - [`DapPump`] — invokes `dap_step` every `DAP_PERIOD_CS`, mirroring
//!   the bare-metal scheduler's T5_PENDING branch (ADR-022). Without
//!   this, `dap_state.mode != Off` has no consumer and the P40 TIG
//!   ignition gate inside `dap_step` never fires.
//!
//! Tests are free to wire these in any order; `dsky_sim` calls them
//! from its render-loop tick.

use agc_core::control::DapMode;
use agc_core::hal::{AgcHardware, Engine, Imu, Rcs, Secs};
use agc_core::math::linalg::mxm;
use agc_core::math::quaternion::quat_from_mat3x3;
use agc_core::types::Met;
use agc_core::AgcState;

use crate::physics::Attitude;
use crate::SimHardware;

/// Mirror of the executive's foreground PIPA accumulator: drain
/// `hw.imu().read_pipa()` and saturating-add into `state.pipa_counts`.
///
/// `read_pipa` is destructive — it returns the pulses that have
/// arrived since the last call and resets the hardware counter — so
/// this function should be called once per simulation tick and once
/// before each waitlist dispatch (the bare-metal scheduler does the
/// same on its main loop).
pub fn pump_pipa_into_state(state: &mut AgcState, hw: &mut SimHardware) {
    let pulses = hw.imu().read_pipa();
    for (acc, &p) in state.pipa_counts.iter_mut().zip(pulses.iter()) {
        *acc = acc.saturating_add(p);
    }
}

/// Mirror of `process_engine_staging` from the bare-metal scheduler:
/// command the SPS on/off and apply the staged gimbal counts.
pub fn pump_engine_to_hw(state: &AgcState, hw: &mut SimHardware) {
    if state.engine_thrusting {
        hw.engine().sps_enable(true);
        let (pitch, yaw) = state.sps_gimbal_cmd;
        hw.engine().sps_gimbal(pitch, yaw);
    } else {
        hw.engine().sps_enable(false);
    }
}

/// Mirror of `process_rcs_staging` from the bare-metal scheduler: if
/// the DAP staged a non-zero jet bitmask + pulse duration, fire the
/// jets and clear the staging. Pulse duration goes from centiseconds
/// to T6 counts (1 count = 0.625 ms ⇒ multiply by 16).
pub fn pump_rcs_to_hw(state: &mut AgcState, hw: &mut SimHardware) {
    if state.rcs_commanded_jets != 0 && state.rcs_commanded_pulse_cs != 0 {
        let jets_a = (state.rcs_commanded_jets & 0xFF) as u8;
        let jets_b = ((state.rcs_commanded_jets >> 8) & 0xFF) as u8;
        hw.rcs().fire_sm_jets(jets_a, jets_b);
        // T6 unit conversion is left to the hardware in this stub —
        // the simulator's RCS jet model has no pulse-duration timer
        // beyond `quench_all`, so the dispatch is sufficient for visuals.
        state.rcs_commanded_jets = 0;
        state.rcs_commanded_pulse_cs = 0;
    }
}

/// Mirror of `process_secs_staging` from the bare-metal scheduler:
/// consume the SECS pyro staging flags and call the corresponding HAL
/// methods exactly once per stage. The flags are edge-triggered —
/// `init_p62` sets `csm_separation_pending`, `init_p67` sets
/// `drogue_deploy_pending`, and this pump resets them after the HAL
/// call so the foreground loop does not keep re-issuing the discrete.
///
/// Without this pump the scenario runner's `do_tick` leaves the staging
/// flags raised forever and the simulated SECS never observes the
/// command — `SimSecs::csm_separation_fired` stays `false` even though
/// P62 advanced. See `agc_core::executive::scheduler::process_secs_staging`
/// for the bare-metal twin.
pub fn pump_secs_to_hw(state: &mut AgcState, hw: &mut SimHardware) {
    if state.csm_separation_pending {
        hw.secs().fire_csm_separation();
        state.csm_separation_pending = false;
    }
    if state.drogue_deploy_pending {
        hw.secs().deploy_drogue();
        state.drogue_deploy_pending = false;
    }
}

/// Translate the AGC DAP's commanded attitude (Euler gimbal angles) into a
/// quaternion target for `Spacecraft::attitude.commanded_q`.
///
/// Reads `state.dap_state.commanded_attitude` and `state.refsmmat`; computes
/// `M_target = refsmmat · gimbal_matrix_from_euler(commanded_attitude)` and
/// converts to a quaternion.  The result is written into `attitude.commanded_q`.
///
/// No-op when `state.dap_state.mode` is `Off` or `RateDamping` (no
/// commanded attitude in those modes), and no-op when `state.dap_state.mode`
/// is `EntryRoll(_)` (pitch/yaw aerodynamically stable during entry; out of
/// scope for #55).
pub fn bridge_dap_to_commanded_q(state: &AgcState, attitude: &mut Attitude) {
    use agc_core::control::attitude::gimbal_matrix_from_euler;

    match state.dap_state.mode {
        DapMode::Off | DapMode::RateDamping | DapMode::EntryRoll(_) => return,
        DapMode::AttitudeHold | DapMode::Maneuver | DapMode::Tvc => {}
    }

    let m_gimbal = gimbal_matrix_from_euler(state.dap_state.commanded_attitude);
    let m_target = mxm(state.refsmmat, m_gimbal);
    attitude.commanded_q = quat_from_mat3x3(m_target);
}

/// Pumps the waitlist at its mission-time cadence.
///
/// Tracks an elapsed-time-driven countdown for the waitlist head,
/// mirroring the bare-metal AGC's TIME3 register: each call to
/// [`WaitlistPump::tick`] subtracts the centiseconds elapsed since the
/// previous tick, and when the countdown reaches zero (or below) the
/// pump dispatches the head, refreshes PIPA / CDU staging, and reloads
/// from the next entry's delta — preserving any overshoot so the
/// average cadence stays correct across slow render frames.
///
/// **Usage:** call [`WaitlistPump::tick`] every render-loop iteration,
/// AFTER `feed_key` has been drained. Anything scheduled inside that
/// frame (e.g. `start_servicer` invoked from `init_p40` running inside
/// `feed_key`) is observed by the same tick that handles the
/// scheduling, so the head countdown arms with the full delta no
/// matter when within the frame the schedule call happened.
pub struct WaitlistPump {
    /// Mission-elapsed time at the previous tick, used to compute
    /// elapsed centiseconds. `None` until the first tick.
    last_tick_met: Option<Met>,
    /// Centiseconds remaining until the waitlist head fires.
    ///
    /// `None` means either the pump has not seen the head yet (initial
    /// state) or the waitlist was empty at the previous tick. A tick
    /// that finds the waitlist non-empty arms this from
    /// `front_delta()`. May be negative briefly while the pump is
    /// catching up across a slow frame; subsequent dispatches preserve
    /// the negative remainder to maintain cadence.
    head_remaining_cs: Option<i32>,
}

impl Default for WaitlistPump {
    fn default() -> Self {
        Self::new()
    }
}

impl WaitlistPump {
    pub const fn new() -> Self {
        Self {
            last_tick_met: None,
            head_remaining_cs: None,
        }
    }

    /// Advance the waitlist by `cs` centiseconds without dispatching tasks.
    ///
    /// Used by the coast loop to account for bulk time advancement (the
    /// remainder of an outer step after the 200 cs SERVICER window) without
    /// paying for per-tick SERVICER invocations.
    ///
    /// Both the internal countdown and `last_tick_met` are updated atomically
    /// so that the next `tick()` call does not re-account for the skipped
    /// interval. The countdown is clamped to zero from below to prevent the
    /// SERVICER catch-up cascade that would otherwise fire multiple times at
    /// the start of the following inner loop.
    pub fn skip_cs(&mut self, cs: u32, state: &AgcState) {
        // Mirror the time advance in last_tick_met so the next tick() call
        // computes elapsed_cs = 0 for the skipped interval.
        self.last_tick_met = Some(state.time);
        if let Some(rem) = self.head_remaining_cs.as_mut() {
            *rem = rem.saturating_sub(cs as i32).max(0);
        }
    }

    /// Advance the pump by one render-loop iteration.
    ///
    /// `last_tick_met` is updated **after** the dispatch loop so that any
    /// `state.time` change made by a dispatched task (e.g. the SERVICER
    /// writing `state.time = new_sv.epoch`) is accounted for in the next
    /// tick's elapsed-time calculation.  Without this ordering, a task that
    /// sets `state.time` backwards (or to a different epoch) would cause the
    /// following tick to see a 0- or negative-elapsed interval, silently
    /// delaying subsequent dispatches.
    pub fn tick(&mut self, state: &mut AgcState, hw: &mut SimHardware) {
        let now = state.time;
        let prev = self.last_tick_met.unwrap_or(now);
        let elapsed_cs = now.0.wrapping_sub(prev.0) as i32;

        // Decrement an active countdown by the elapsed time.
        if let Some(rem) = self.head_remaining_cs.as_mut() {
            *rem = rem.saturating_sub(elapsed_cs);
        }

        // Arm the countdown if the pump has no active head and the
        // waitlist has work.  Subtract the elapsed time that already
        // passed this tick so the head fires at the correct absolute
        // time, not `front_delta()` ticks *after* this tick.  Without
        // this subtraction, the first tick after the pump is armed (or
        // re-armed after the waitlist was empty) silently "wastes" one
        // tick's worth of elapsed time, causing every task to fire one
        // tick late.
        if self.head_remaining_cs.is_none() {
            if let Some(cs) = state.waitlist.front_delta() {
                self.head_remaining_cs = Some(cs as i32 - elapsed_cs);
            }
        }

        // Dispatch every task whose countdown has expired.
        while let Some(rem) = self.head_remaining_cs {
            if rem > 0 {
                break;
            }
            // Mirror Executive::run's pre-dispatch staging refresh.
            state.current_cdu = hw.imu().read_cdu();
            pump_pipa_into_state(state, hw);

            match state.waitlist.pop_task() {
                Some((task, _next_delta)) => {
                    agc_core::services::lamps::note_pinball_activity(state);
                    task(state);
                    self.head_remaining_cs = state
                        .waitlist
                        .front_delta()
                        // Add to the (possibly negative) remainder to
                        // preserve cadence across catch-up dispatches.
                        .map(|cs| rem.saturating_add(cs as i32));
                }
                None => {
                    self.head_remaining_cs = None;
                    break;
                }
            }
        }

        // Record the post-dispatch state.time so that if a task changed
        // state.time (e.g. SERVICER epoch write), the next tick's elapsed
        // calculation starts from the task's new time value, not the
        // pre-dispatch value.
        self.last_tick_met = Some(state.time);
    }
}

/// Pumps the DAP at its 100 ms cadence (mirrors the bare-metal T5_PENDING
/// branch, ADR-022).
///
/// On the real board, TIM4 raises T5_PENDING every `DAP_PERIOD_CS`
/// centiseconds and the scheduler calls `dap_step`. In the sim there are
/// no interrupts, so this pump tracks an internal countdown in
/// mission-time centiseconds and invokes `dap_step` whenever it expires.
///
/// The pump observes the `Off → !Off` transition automatically: as soon
/// as `state.dap_state.mode != Off` (set by `dap_init`), the countdown
/// arms; when it reaches zero, the pump pre-reads CDU and calls
/// `dap_step`, then re-arms for the next cycle. When `dap_step` sets
/// `mode = Off` (e.g. via `dap_stop`), the countdown is dropped and no
/// further `dap_step` calls are made.
///
/// **Usage:** call [`DapPump::tick`] every render-loop iteration, AFTER
/// `pump_pipa_into_state` (so CDU is fresh) and BEFORE `pump_engine_to_hw`
/// / `pump_rcs_to_hw` (so the simulated hardware sees the staging fields
/// `dap_step` just wrote).
pub struct DapPump {
    last_tick_met: Option<Met>,
    /// Centiseconds until the next `dap_step` invocation.
    /// `None` while `dap_state.mode == Off`.
    cycle_remaining_cs: Option<i32>,
}

impl Default for DapPump {
    fn default() -> Self {
        Self::new()
    }
}

impl DapPump {
    pub const fn new() -> Self {
        Self {
            last_tick_met: None,
            cycle_remaining_cs: None,
        }
    }

    /// Advance the pump by one render-loop iteration.
    ///
    /// `attitude` is an optional reference to the simulator's [`Attitude`]
    /// state.  When `Some`, the bridge [`bridge_dap_to_commanded_q`] is
    /// invoked after each `dap_step` call to translate the DAP's Euler
    /// commanded attitude into the quaternion representation used by the
    /// physics model.  Pass `None` in unit tests that do not need attitude
    /// tracking.
    pub fn tick(
        &mut self,
        state: &mut AgcState,
        hw: &mut SimHardware,
        mut attitude: Option<&mut Attitude>,
    ) {
        use agc_core::control::dap::{dap_step, DAP_PERIOD_CS};

        let now = state.time;
        let prev = self.last_tick_met.unwrap_or(now);
        let elapsed_cs = now.0.wrapping_sub(prev.0) as i32;
        self.last_tick_met = Some(now);

        // DAP is off: nothing to do; drop any pending countdown.
        if state.dap_state.mode == DapMode::Off {
            self.cycle_remaining_cs = None;
            return;
        }

        // Off → !Off transition: arm the first cycle. First dap_step
        // fires DAP_PERIOD_CS centiseconds from now, matching the
        // bare-metal scheduler's behaviour of arming TIM4 and waiting
        // for it to expire.
        if self.cycle_remaining_cs.is_none() {
            self.cycle_remaining_cs = Some(DAP_PERIOD_CS as i32);
            return;
        }

        // Decrement countdown.
        if let Some(rem) = self.cycle_remaining_cs.as_mut() {
            *rem = rem.saturating_sub(elapsed_cs);
        }

        // Fire dap_step for every expired cycle (catch-up across slow
        // frames), preserving overshoot to keep average cadence honest.
        // The `attitude` reborrow via `&mut attitude` inside the loop
        // gives a fresh `&mut Attitude` each iteration without
        // consuming the outer Option.
        while let Some(rem) = self.cycle_remaining_cs {
            if rem > 0 {
                break;
            }
            // Mirror the bare-metal T5 branch: refresh CDU staging.
            state.current_cdu = hw.imu().read_cdu();
            dap_step(state);
            if let Some(att) = &mut attitude {
                bridge_dap_to_commanded_q(state, att);
            }
            // dap_step may have turned the DAP off.
            if state.dap_state.mode == DapMode::Off {
                self.cycle_remaining_cs = None;
                break;
            }
            self.cycle_remaining_cs = Some(rem.saturating_add(DAP_PERIOD_CS as i32));
        }
    }
}

// ── T4Pump (T4RUPT, ~120 ms cadence) ─────────────────────────────────────────

/// Pumps the T4RUPT handler at its 120 ms cadence (mirrors the
/// bare-metal `T4_PENDING` branch of `executive::scheduler::Executive::run`).
///
/// Today the handler drains the uplink FIFO into the V/N processor; later
/// milestones will fold in the DSKY frame emit and downlink list output.
/// On the real hardware TIM3 fires every `T4RUPT_PERIOD_CS` centiseconds;
/// in the sim there are no interrupts, so this pump tracks an internal
/// countdown in mission-time centiseconds and invokes
/// [`agc_core::services::t4rupt::t4rupt_step`] whenever it expires.
///
/// **Usage:** call [`T4Pump::tick`] every render-loop iteration. The pump
/// is safe to call before the waitlist pump because `t4rupt_step` only
/// touches HAL FIFOs and V/N state, neither of which interferes with
/// scheduled tasks.
pub struct T4Pump {
    last_tick_met: Option<Met>,
    /// Centiseconds until the next `t4rupt_step` invocation.
    remaining_cs: i32,
}

impl Default for T4Pump {
    fn default() -> Self {
        Self::new()
    }
}

impl T4Pump {
    pub const fn new() -> Self {
        Self {
            last_tick_met: None,
            remaining_cs: 0,
        }
    }

    /// Advance the pump by one render-loop iteration.
    pub fn tick(&mut self, state: &mut AgcState, hw: &mut SimHardware) {
        use agc_core::control::imu_control::T4RUPT_PERIOD_CS;
        use agc_core::services::t4rupt::t4rupt_step;

        let now = state.time;
        let prev = self.last_tick_met.unwrap_or(now);
        let elapsed_cs = now.0.wrapping_sub(prev.0) as i32;
        self.last_tick_met = Some(now);

        self.remaining_cs = self.remaining_cs.saturating_sub(elapsed_cs);

        // Catch up across slow frames, preserving any overshoot so the
        // average cadence stays honest.
        while self.remaining_cs <= 0 {
            t4rupt_step(state, hw);
            self.remaining_cs = self.remaining_cs.saturating_add(T4RUPT_PERIOD_CS as i32);
        }
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple recording task — sets a sentinel via flagwords[0].
    fn record_call(state: &mut AgcState) {
        state.flagwords[0] = state.flagwords[0].wrapping_add(1);
    }

    /// TC-PUMP-1: a task scheduled for 100 cs fires once mission time
    /// reaches that deadline, and only once.
    #[test]
    fn tc_pump_1_dispatch_at_deadline() {
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut pump = WaitlistPump::new();
        state.time = Met(0);

        let r = state.waitlist.schedule(100, record_call);
        assert!(matches!(
            r,
            agc_core::executive::ScheduleResult::OkReloadT3(100)
        ));
        // Tick at the same MET as the schedule arms the head countdown.
        pump.tick(&mut state, &mut hw);
        assert_eq!(state.flagwords[0], 0);

        // Halfway to the deadline: still nothing.
        state.time = Met(50);
        pump.tick(&mut state, &mut hw);
        assert_eq!(state.flagwords[0], 0);

        // At the deadline: fires exactly once.
        state.time = Met(100);
        pump.tick(&mut state, &mut hw);
        assert_eq!(state.flagwords[0], 1);

        // Subsequent ticks with no new tasks must not refire.
        state.time = Met(200);
        pump.tick(&mut state, &mut hw);
        assert_eq!(state.flagwords[0], 1);
    }

    /// TC-PUMP-2: multiple back-to-back tasks fire in delta-order.
    #[test]
    fn tc_pump_2_back_to_back_dispatch() {
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut pump = WaitlistPump::new();
        state.time = Met(0);

        state.waitlist.schedule(10, record_call);
        state.waitlist.schedule(20, record_call);
        state.waitlist.schedule(30, record_call);
        pump.tick(&mut state, &mut hw); // arm at scheduling time

        // Advance well past all deadlines and tick once.
        state.time = Met(100);
        pump.tick(&mut state, &mut hw);
        assert_eq!(state.flagwords[0], 3, "all three tasks must dispatch");
    }

    /// TC-PUMP-3: pump drains PIPA pulses into state.pipa_counts before
    /// each dispatch so the SERVICER's destructive read sees them.
    #[test]
    fn tc_pump_3_pipa_drained_before_dispatch() {
        fn snapshot_pipa(state: &mut AgcState) {
            // Stash the staging value into flagwords for inspection.
            state.flagwords[1] = state.pipa_counts[1] as u16;
        }

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut pump = WaitlistPump::new();
        state.time = Met(0);

        // Pre-load some PIPA pulses on the simulated hardware.
        hw.imu.pipa = [0, 42, 0];

        state.waitlist.schedule(10, snapshot_pipa);
        pump.tick(&mut state, &mut hw); // arm

        state.time = Met(20);
        pump.tick(&mut state, &mut hw);

        assert_eq!(
            state.flagwords[1], 42,
            "pump must drain PIPA into state.pipa_counts before dispatch"
        );
    }

    /// TC-PUMP-4: pump_engine_to_hw mirrors engine_thrusting.
    #[test]
    fn tc_pump_4_engine_mirror() {
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();

        state.engine_thrusting = true;
        state.sps_gimbal_cmd = (10, -5);
        pump_engine_to_hw(&state, &mut hw);
        assert!(hw.engine.thrusting);
        assert_eq!(hw.engine.gimbal_pitch, 10);
        assert_eq!(hw.engine.gimbal_yaw, -5);

        state.engine_thrusting = false;
        pump_engine_to_hw(&state, &mut hw);
        assert!(!hw.engine.thrusting);
    }

    /// TC-PUMP-5: a self-rescheduling task keeps firing on cadence.
    #[test]
    fn tc_pump_5_self_rescheduling_cadence() {
        fn periodic(state: &mut AgcState) {
            state.flagwords[0] = state.flagwords[0].wrapping_add(1);
            // Reschedule self in 100 cs.
            state.waitlist.schedule(100, periodic);
        }

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut pump = WaitlistPump::new();
        state.time = Met(0);

        state.waitlist.schedule(100, periodic);
        pump.tick(&mut state, &mut hw); // arm

        // Step forward 350 cs — should see 3 firings (at 100, 200, 300 cs).
        state.time = Met(350);
        pump.tick(&mut state, &mut hw);
        assert_eq!(
            state.flagwords[0], 3,
            "periodic task must fire on cadence; got {}",
            state.flagwords[0]
        );
    }

    // ── Suppress dead-code warnings for the trait import. ────────────────
    #[allow(unused)]
    fn _imports_ok() {
        // Keep the Rcs import alive even if pump_rcs_to_hw is unused in tests.
        let _: fn(&mut AgcState, &mut SimHardware) = pump_rcs_to_hw;
    }

    /// TC-PUMP-SECS-1: `init_p62` stages the CM/SM separation pyro,
    /// `pump_secs_to_hw` consumes the flag exactly once, and subsequent
    /// pump cycles must not re-fire the pyro. This is the unit-level
    /// "exactly once" check that backs the end-to-end SM-sep assertion in
    /// `phase_entry` (issue #124 acceptance criterion).
    #[test]
    fn tc_pump_secs_1_csm_separation_fires_exactly_once() {
        use agc_core::programs::p61_p67::{init_p61, init_p62};

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();

        // Walk to P62 from preparation — init_p62 must stage the pyro.
        init_p61(&mut state);
        init_p62(&mut state);
        assert!(
            state.csm_separation_pending,
            "init_p62 must raise the CM/SM separation staging flag"
        );
        assert_eq!(
            hw.secs.csm_separation_fire_count, 0,
            "fixture: no HAL call yet"
        );

        // One pump cycle drives the discrete through the HAL.
        pump_secs_to_hw(&mut state, &mut hw);
        assert!(
            !state.csm_separation_pending,
            "pump must consume the staging flag"
        );
        assert_eq!(
            hw.secs.csm_separation_fire_count, 1,
            "pyro must fire exactly once after one pump cycle"
        );

        // Many subsequent pump cycles must NOT re-fire — the pyro is
        // one-shot and `init_p62` must not re-stage it.
        for _ in 0..50 {
            pump_secs_to_hw(&mut state, &mut hw);
        }
        assert_eq!(
            hw.secs.csm_separation_fire_count, 1,
            "pyro must remain at exactly one firing across 50 pump cycles"
        );
    }

    /// TC-PUMP-SECS-2: `pump_secs_to_hw` is a no-op when nothing is
    /// staged. Guards against accidental edge-triggering on the empty
    /// path (which would fire the pyro on every cycle).
    #[test]
    fn tc_pump_secs_2_no_staging_is_noop() {
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();

        assert!(!state.csm_separation_pending);
        assert!(!state.drogue_deploy_pending);

        for _ in 0..10 {
            pump_secs_to_hw(&mut state, &mut hw);
        }
        assert_eq!(hw.secs.csm_separation_fire_count, 0);
        assert_eq!(hw.secs.drogue_fire_count, 0);
    }

    /// TC-DAP-PUMP-1: dap_step fires at the 100 ms cadence once the DAP
    /// is no longer Off. Without this pump (or the equivalent bare-metal
    /// T5_PENDING branch), `dap_step` is never called and P40's TIG
    /// ignition gate inside it never fires — the regression that
    /// motivated this test.
    #[test]
    fn tc_dap_pump_1_tig_arms_engine_thrusting() {
        use agc_core::control::dap::DAP_PERIOD_CS;
        use agc_core::control::DapMode;
        use agc_core::guidance::maneuver::BurnState;

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut dap_pump = DapPump::new();

        state.time = Met(0);
        state.dap_state.mode = DapMode::Maneuver;
        // Burn armed for ignition at MET 50 cs.
        state.burn = BurnState {
            target_dv_inertial: [10.0, 0.0, 0.0],
            accumulated_dv_inertial: [0.0; 3],
            tig: Met(50),
            burn_active: false,
            cutoff_time_met: false,
            armed: true,
        };

        // First tick at MET=0: arms the cycle, no dap_step yet.
        dap_pump.tick(&mut state, &mut hw, None);
        assert!(
            !state.engine_thrusting,
            "no dap_step has run yet -- engine must still be off"
        );

        // Advance past TIG and past one DAP cycle.
        state.time = Met((DAP_PERIOD_CS as u32) + 50);
        dap_pump.tick(&mut state, &mut hw, None);

        assert!(
            state.engine_thrusting,
            "dap_step at MET={} should have fired the TIG-arm gate (tig={})",
            state.time.0, 50
        );
        assert_eq!(
            state.dap_state.mode,
            DapMode::Tvc,
            "TIG ignition must promote DAP to Tvc mode"
        );
        assert!(!state.burn.armed, "TIG ignition must clear burn.armed");
    }

    /// TC-T4-PUMP-1: T4Pump fires t4rupt_step at the T4RUPT cadence and
    /// the resulting uplink drain reaches the V/N state machine.
    #[test]
    fn tc_t4_pump_1_uplink_drained_at_cadence() {
        use agc_core::control::imu_control::T4RUPT_PERIOD_CS;
        use agc_core::services::v_n::{Key, VnPhase};

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut t4 = T4Pump::new();

        state.time = Met(0);
        // Queue VERB-71-ENTR on the uplink FIFO via the canonical key codes.
        hw.uplink.push_key(Key::Verb);
        hw.uplink.push_key(Key::Digit(7));
        hw.uplink.push_key(Key::Digit(1));
        hw.uplink.push_key(Key::Entr);

        // First tick at MET=0 fires the initial t4rupt_step (remaining = 0).
        t4.tick(&mut state, &mut hw);

        assert!(
            matches!(state.vn.phase, VnPhase::P27Address { .. }),
            "T4Pump must drain uplink → V/N; phase = {:?}",
            state.vn.phase
        );

        // No further work — advance past one full T4 period; nothing must
        // change because the FIFO is empty.
        state.time = Met(T4RUPT_PERIOD_CS);
        t4.tick(&mut state, &mut hw);
        assert!(matches!(state.vn.phase, VnPhase::P27Address { .. }));
    }

    // ── bridge_dap_to_commanded_q ─────────────────────────────────────────────

    /// tc_rt_bridge_dap_off_no_change
    ///
    /// When `dap_state.mode == Off`, `bridge_dap_to_commanded_q` must leave
    /// `attitude.commanded_q` unchanged.
    #[test]
    fn tc_rt_bridge_dap_off_no_change() {
        use crate::physics::Attitude;
        use agc_core::control::DapMode;

        let mut state = AgcState::new();
        state.dap_state.mode = DapMode::Off;

        let identity_q = [1.0_f64, 0.0, 0.0, 0.0];
        let mut attitude = Attitude {
            q: identity_q,
            commanded_q: identity_q,
            slew_tau_s: 5.0,
        };

        bridge_dap_to_commanded_q(&state, &mut attitude);

        assert_eq!(
            attitude.commanded_q, identity_q,
            "commanded_q must not change when DAP mode is Off"
        );
    }

    /// tc_rt_bridge_rate_damping_no_change
    ///
    /// When `dap_state.mode == RateDamping`, `bridge_dap_to_commanded_q` must
    /// leave `attitude.commanded_q` unchanged (no commanded attitude in this mode).
    #[test]
    fn tc_rt_bridge_rate_damping_no_change() {
        use crate::physics::Attitude;
        use agc_core::control::DapMode;

        let mut state = AgcState::new();
        state.dap_state.mode = DapMode::RateDamping;

        let identity_q = [1.0_f64, 0.0, 0.0, 0.0];
        let mut attitude = Attitude {
            q: identity_q,
            commanded_q: identity_q,
            slew_tau_s: 5.0,
        };

        bridge_dap_to_commanded_q(&state, &mut attitude);

        assert_eq!(
            attitude.commanded_q, identity_q,
            "commanded_q must not change when DAP mode is RateDamping"
        );
    }

    /// tc_rt_bridge_entry_roll_no_change
    ///
    /// When `dap_state.mode == EntryRoll(_)`, `bridge_dap_to_commanded_q` must
    /// leave `attitude.commanded_q` unchanged (entry roll out of scope for #55).
    #[test]
    fn tc_rt_bridge_entry_roll_no_change() {
        use crate::physics::Attitude;
        use agc_core::control::DapMode;

        let mut state = AgcState::new();
        state.dap_state.mode = DapMode::EntryRoll(0.5);

        let identity_q = [1.0_f64, 0.0, 0.0, 0.0];
        let mut attitude = Attitude {
            q: identity_q,
            commanded_q: identity_q,
            slew_tau_s: 5.0,
        };

        bridge_dap_to_commanded_q(&state, &mut attitude);

        assert_eq!(
            attitude.commanded_q, identity_q,
            "commanded_q must not change when DAP mode is EntryRoll"
        );
    }

    /// tc_rt_bridge_attitude_hold_identity_inputs
    ///
    /// `AttitudeHold` mode with `commanded_attitude = [0, 0, 0]` and identity
    /// REFSMMAT: `M_target = I · I = I`, so `commanded_q` must equal the
    /// identity quaternion `[1, 0, 0, 0]` within 1e-12.
    #[test]
    fn tc_rt_bridge_attitude_hold_identity_inputs() {
        use crate::physics::Attitude;
        use agc_core::control::DapMode;

        let mut state = AgcState::new();
        state.dap_state.mode = DapMode::AttitudeHold;
        state.dap_state.commanded_attitude = [0.0, 0.0, 0.0];
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        let mut attitude = Attitude {
            // Set to something non-identity so a no-op would be detectable.
            commanded_q: [0.0, 1.0, 0.0, 0.0],
            ..Attitude::default()
        };

        bridge_dap_to_commanded_q(&state, &mut attitude);

        let q = attitude.commanded_q;
        let eps = 1e-12;
        assert!((q[0] - 1.0).abs() < eps, "w should be 1.0, got {}", q[0]);
        assert!(q[1].abs() < eps, "x should be 0.0, got {}", q[1]);
        assert!(q[2].abs() < eps, "y should be 0.0, got {}", q[2]);
        assert!(q[3].abs() < eps, "z should be 0.0, got {}", q[3]);
    }

    /// tc_rt_bridge_attitude_hold_90deg_roll
    ///
    /// `AttitudeHold` mode with `commanded_attitude = [PI/2, 0, 0]` and identity
    /// REFSMMAT: `M_target = Rx(PI/2)`.  The resulting `commanded_q` must equal
    /// `[cos(PI/4), sin(PI/4), 0, 0]` within 1e-12 (90° rotation about +X).
    #[test]
    fn tc_rt_bridge_attitude_hold_90deg_roll() {
        use crate::physics::Attitude;
        use agc_core::control::DapMode;
        use core::f64::consts::{FRAC_PI_2, FRAC_PI_4};

        let mut state = AgcState::new();
        state.dap_state.mode = DapMode::AttitudeHold;
        state.dap_state.commanded_attitude = [FRAC_PI_2, 0.0, 0.0];
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        let mut attitude = Attitude::default();
        bridge_dap_to_commanded_q(&state, &mut attitude);

        let q = attitude.commanded_q;
        let eps = 1e-12;

        // Norm must be 1.
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!(
            (norm - 1.0).abs() < eps,
            "quaternion must be unit norm, got {norm}"
        );

        // Axis-angle check: 90° about +X → w = cos(PI/4), x = sin(PI/4), y = z = 0.
        assert!(
            (q[0] - FRAC_PI_4.cos()).abs() < eps,
            "w should be cos(PI/4) = {}, got {}",
            FRAC_PI_4.cos(),
            q[0]
        );
        assert!(
            (q[1] - FRAC_PI_4.sin()).abs() < eps,
            "x should be sin(PI/4) = {}, got {}",
            FRAC_PI_4.sin(),
            q[1]
        );
        assert!(q[2].abs() < eps, "y should be 0, got {}", q[2]);
        assert!(q[3].abs() < eps, "z should be 0, got {}", q[3]);
    }

    /// TC-DAP-PUMP-2: pump is a no-op while mode == Off and does not
    /// arm a countdown that would later fire spuriously.
    #[test]
    fn tc_dap_pump_2_off_is_inert() {
        use agc_core::control::DapMode;

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut dap_pump = DapPump::new();

        state.dap_state.mode = DapMode::Off;
        state.time = Met(0);
        dap_pump.tick(&mut state, &mut hw, None);

        state.time = Met(10_000);
        dap_pump.tick(&mut state, &mut hw, None);

        // No fields the pump would have written touched.
        assert_eq!(state.rcs_commanded_jets, 0);
        assert_eq!(state.rcs_commanded_pulse_cs, 0);
        assert!(!state.engine_thrusting);
    }
}
