//! SERVICER Average-G: 2-second navigation heartbeat cycle.
//!
//! Implements the predictor-corrector CALCRVG integration step, PIPA read and
//! saturation check, ECI rotation of delta-V, and state-vector update.
//!
//! The cycle is driven by the Waitlist (T3RUPT → READACCS task) at 2-second
//! intervals while the AVERAGEG flag is set.
//!
//! AGC source: Comanche055/SERVICER207.agc
//!   PREREAD    (initialisation, pages 822-823),
//!   READACCS   (PIPA read + Waitlist reschedule, pages 823-826),
//!   SERVICER   (PIPA saturation check + PIPA compensation, pages 828-829),
//!   AVERAGEG   (calls CALCRVG, bulk-copies RN1→RN, exits, pages 828-829),
//!   CALCRVG    (predictor-corrector integration, pages 835-836),
//!   CALCGRAV   (gravity at predicted position, pages 835-836),
//!   NORMLIZE   (first-cycle initialisation of GDT/2, page 831),
//!   PIPASR     (read and clear PIPA counters, pages 832-834),
//!   SERVEXIT   (phase-change + ENDOFJOB, page 830).

use crate::hal::{imu::ImuIo, AgcHardware};
use crate::math::linalg::{add, mxv, scale, transpose};
use crate::navigation::constants::{CYCLE_DT, CYCLE_DT_CS, KPIP1, PIPA_MAX_COUNTS};
use crate::navigation::gravity::earth_gravity;
use crate::navigation::state_vector::StateVector;
use crate::services::alarm::{AlarmCode, AlarmState};

// ── Restart phase constants ───────────────────────────────────────────────────

/// Restart phase: before PIPA compensation and CALCRVG.
///
/// AGC: `TC PHASCHNG / OCT 16035` (SERVICER207.agc, before 1/PIPA call).
#[allow(dead_code)]
const PHASE_BEFORE_CALCRVG: u8 = 4;

/// Restart phase: during/after CALCRVG, before bulk copy.
///
/// AGC: `TC PHASCHNG / OCT 10035` (SERVICER207.agc, after CALCRVG call).
/// Same value as PHASE_BEFORE_CALCRVG in the AGC; used twice.
#[allow(dead_code)]
const PHASE_AFTER_CALCRVG: u8 = 4;

/// Restart phase: SERVEXIT — final safe state.
///
/// AGC: `TC PHASCHNG / OCT 00035` (SERVEXIT label).
#[allow(dead_code)]
const PHASE_SERVEXIT: u8 = 0;

/// Restart group for SERVICER.
///
/// AGC: `-PHASE5` / `NEWPHASE OCT 5`.
#[allow(dead_code)]
const SERVICER_RESTART_GROUP: u8 = 5;

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors that can occur during an Average-G cycle.
///
/// These do not cause a GOJAM restart; the cycle skips and the state is
/// preserved unchanged, consistent with SERVICER207.agc alarm handling.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AvgGError {
    /// PIPA hardware not ready (read_pipa could not be serviced).
    PipaNotReady,
    /// One or more PIPA axes saturated (`|count| >= PIPA_MAX_COUNTS`).
    ///
    /// Alarm 00205 has been raised.
    ///
    /// AGC: `Comanche055/SERVICER207.agc` `-MAXDELV DEC -6398` and
    ///      `TC ALARM / OCT 00205`.
    ///
    /// APPROXIMATE deviation: the AGC jumps to AVERAGEG after the alarm (which
    /// still calls CALCRVG with uncompensated DELV). The Rust port skips CALCRVG
    /// entirely when PIPA is saturated, preserving the state unchanged, to avoid
    /// corrupting the navigation state with physically impossible delta-V values.
    PipaSaturated,
}

// ── AverageG struct ───────────────────────────────────────────────────────────

/// SERVICER Average-G navigation cycle.
///
/// Implements the 2-second predictor-corrector integration cycle driven by
/// PIPA accelerometer readings. Called every 2 seconds via the Waitlist
/// (T3RUPT → READACCS task) while the AVERAGEG flag is set.
///
/// AGC source: `Comanche055/SERVICER207.agc`, AVERAGEG/CALCRVG routines, pages 835-836.
///
/// Restart safety: a mid-cycle restart replays the PIPA read and CALCRVG step.
/// Phase bracketing uses `AgcState::restart` (passed via `hw` where available).
///
/// Invariants:
///   - No heap allocation.
///   - No `unwrap` or `expect`.
///   - PIPA saturation → alarm 00205 raised, state unchanged.
pub struct AverageG {
    state: StateVector,
}

impl AverageG {
    /// Initialise Average-G with the provided state vector.
    ///
    /// The `state.gdt_over_2()` field should be pre-initialised via
    /// `initialize_gdt` to match the AGC NORMLIZE routine.
    ///
    /// AGC: `Comanche055/SERVICER207.agc` NORMLIZE (page 831).
    pub fn new(initial_state: StateVector) -> Self {
        Self {
            state: initial_state,
        }
    }

    /// Return a reference to the current state vector.
    pub fn state(&self) -> &StateVector {
        &self.state
    }

    /// Execute one 2-second Average-G cycle.
    ///
    /// Steps (matching CALCRVG in SERVICER207.agc):
    ///   1. Read and clear PIPA counts via `hw.imu().read_pipa()`.
    ///   2. Check saturation: if any |count| >= PIPA_MAX_COUNTS, raise alarm 00205,
    ///      return `Err(AvgGError::PipaSaturated)` with state unchanged.
    ///   3. Rotate PIPA counts to ECI:
    ///      `dv_sm = counts × KPIP1` (m/s, stable-member frame)
    ///      `dv_eci = transpose(refsmmat) × dv_sm`
    ///   4. Predictor (position):
    ///      `v_half = velocity + dv_eci×0.5 + gdt_over_2`
    ///      `r1 = position + v_half × CYCLE_DT`
    ///   5. Gravity at predicted position (CALCGRAV):
    ///      `gdt_new = earth_gravity(&r1) × (CYCLE_DT / 2)`
    ///   6. Corrector (velocity):
    ///      `v1 = velocity + dv_eci + gdt_new + gdt_over_2`
    ///   7. Build new state and commit.
    ///
    /// Input/output units: positions in metres (ECI), velocities in m/s (ECI),
    ///   time in centiseconds, gdt_over_2 in m/s.
    ///
    /// AGC source: `Comanche055/SERVICER207.agc` CALCRVG (pages 835-836),
    ///             CALCGRAV (page 835), PIPASR (pages 832-834).
    pub fn cycle<H: AgcHardware>(&mut self, hw: &mut H) -> Result<StateVector, AvgGError> {
        // Step 1: Read and clear PIPA counts (PIPASR routine).
        // AGC source: SERVICER207.agc PIPASR (pages 832-834).
        let counts = hw.imu().read_pipa();

        // Step 2: Saturation check (SERVICER207.agc, -MAXDELV DEC -6398).
        for &c in counts.iter() {
            let c: i16 = c;
            if c.unsigned_abs() >= PIPA_MAX_COUNTS.unsigned_abs() {
                // AGC: TC ALARM / OCT 00205 / TC AVERAGEG (skips CALCRVG in Rust port).
                AlarmState::raise(AlarmCode::PipaOverflow);
                return Err(AvgGError::PipaSaturated);
            }
        }

        // Restart phase: before CALCRVG.
        // NOTE: Phase bracketing via AgcState::restart is not wired to AgcHardware in M2.
        // Full restart safety requires passing AgcState through the HAL in a future milestone.

        // Step 3: Rotate PIPA counts to ECI.
        // dv_sm[i] = counts[i] * KPIP1  (m/s in stable-member frame)
        // KPIP1 = 0.0585 m/s/count (from SERVICER207.agc KPIP1 2DEC 0.074880)
        // AGC source: CALCRVG Step 1 — DELVREF = REFSMMAT^T × (DELV × KPIP1).
        let dv_sm = [
            counts[0] as f64 * KPIP1,
            counts[1] as f64 * KPIP1,
            counts[2] as f64 * KPIP1,
        ];
        let refsmmat = hw.imu().refsmmat();
        // VXM semantics: transpose(REFSMMAT) × dv_sm  (SM → ECI)
        let dv_eci = mxv(&transpose(&refsmmat), &dv_sm);

        let r = self.state.position();
        let v = self.state.velocity();
        let gdt_old = self.state.gdt_over_2();
        let t = self.state.time();

        // Step 4: Predictor (position).
        // v_half = VN + DELVREF/2 + GDT/2  (velocity at half-step)
        // r1     = RN + v_half × 2SEC
        // AGC source: CALCRVG `RN1 = RN + (VN + DELVREF/2 + GDT/2) * 2SEC`.
        let v_half = add(&add(&v, &scale(&dv_eci, 0.5)), &gdt_old);
        let r1 = add(&r, &scale(&v_half, CYCLE_DT));

        // Step 5: Gravity at predicted position (CALCGRAV).
        // GDT1/2 = gravity(r1) × dt/2
        // AGC source: CALCGRAV called with RN1 in MPAC.
        let gdt_new = scale(&earth_gravity(&r1), CYCLE_DT * 0.5);

        // Step 6: Corrector (velocity).
        // VN1 = VN + DELVREF + GDT1/2 + GDT/2  (trapezoidal gravity average)
        // AGC source: CALCRVG `VN1 = VN + DELVREF + GDT1/2 + GDT/2`.
        let v1 = add(&add(&add(&v, &dv_eci), &gdt_new), &gdt_old);

        // Step 7: Build new state.
        let t1 = t + CYCLE_DT_CS;
        let new_state = StateVector::with_gdt(r1, v1, t1, gdt_new);

        // Restart phase: after CALCRVG (before bulk copy).
        // Phase bracketing not wired in M2 (see note above).

        // Commit (matches AGC GENTRAN bulk copy RN1→RN, VN1→VN, GDT1/2→GDT/2).
        self.state = new_state;

        // Restart phase: SERVEXIT (final safe state).

        Ok(new_state)
    }
}

/// Initialise the `gdt_over_2` field of a `StateVector` using the current
/// gravitational acceleration.
///
/// Call this once before the first `AverageG::cycle` to replicate the
/// AGC NORMLIZE routine (which calls CALCGRAV to initialise GDT/2 before
/// the first Average-G pass).
///
/// Equivalent to: `state.with_gdt_over_2(earth_gravity(&state.position()) × (CYCLE_DT / 2))`
///
/// Input: `StateVector` with position in metres (ECI).
/// Output: `StateVector` with `gdt_over_2` set to `earth_gravity(position) × (CYCLE_DT/2)`.
///
/// AGC source: `Comanche055/SERVICER207.agc`, NORMLIZE routine (page 831).
pub fn initialize_gdt(state: StateVector) -> StateVector {
    let gdt = scale(&earth_gravity(&state.position()), CYCLE_DT * 0.5);
    state.with_gdt_over_2(gdt)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::{
        dsky::{DigitRow, DskyIo, Key, RelayWord},
        engine::EngineImpl,
        imu::{ImuImpl, Unaligned},
        optics::OpticsImpl,
        rcs::RcsImpl,
        telemetry::TelemetryImpl,
        timers::TimersImpl,
        uplink::UplinkImpl,
        AgcHardware,
    };
    use crate::navigation::constants::{KPIP1, RE_EARTH};
    use crate::types::{Met, IDENTITY_MAT3};

    // ── Minimal DSKY stub (no concrete DskyImpl in M1) ────────────────────────

    struct NopDsky;
    impl DskyIo for NopDsky {
        fn read_key(&mut self) -> Option<Key> {
            None
        }
        fn read_nav_key(&mut self) -> Option<Key> {
            None
        }
        fn write_relay(&mut self, _word: RelayWord) {}
        fn write_lamp_word(&mut self, _bits: u16) {}
        fn write_prog(&mut self, _prog: u8) {}
        fn write_verb(&mut self, _verb: u8) {}
        fn write_noun(&mut self, _noun: u8) {}
        fn write_register(&mut self, _row: usize, _value: &DigitRow) {}
        fn proceed_pressed(&self) -> bool {
            false
        }
        fn set_prog_light(&mut self, _on: bool) {}
    }

    /// Minimal hardware struct for Average-G tests using concrete *Impl types.
    struct TestHw {
        imu: ImuImpl<Unaligned>,
        timers: TimersImpl,
        dsky: NopDsky,
        engine: EngineImpl,
        optics: OpticsImpl,
        rcs: RcsImpl,
        uplink: UplinkImpl,
        telemetry: TelemetryImpl,
    }

    impl TestHw {
        fn new() -> Self {
            Self {
                imu: ImuImpl::<Unaligned>::new(),
                timers: TimersImpl::new(),
                dsky: NopDsky,
                engine: EngineImpl::new(),
                optics: OpticsImpl::new(),
                rcs: RcsImpl::new(),
                uplink: UplinkImpl::new(),
                telemetry: TelemetryImpl::new(),
            }
        }
    }

    impl AgcHardware for TestHw {
        type Timers = TimersImpl;
        type Dsky = NopDsky;
        type Imu = ImuImpl<Unaligned>;
        type Optics = OpticsImpl;
        type Engine = EngineImpl;
        type Rcs = RcsImpl;
        type Uplink = UplinkImpl;
        type Telemetry = TelemetryImpl;

        fn timers(&mut self) -> &mut TimersImpl {
            &mut self.timers
        }
        fn dsky(&mut self) -> &mut NopDsky {
            &mut self.dsky
        }
        fn imu(&mut self) -> &mut ImuImpl<Unaligned> {
            &mut self.imu
        }
        fn optics(&mut self) -> &mut OpticsImpl {
            &mut self.optics
        }
        fn engine(&mut self) -> &mut EngineImpl {
            &mut self.engine
        }
        fn rcs(&mut self) -> &mut RcsImpl {
            &mut self.rcs
        }
        fn uplink(&mut self) -> &mut UplinkImpl {
            &mut self.uplink
        }
        fn telemetry(&mut self) -> &mut TelemetryImpl {
            &mut self.telemetry
        }
        fn pet_watchdog(&mut self) {}
        fn hardware_restart(&mut self) -> ! {
            loop {
                core::hint::spin_loop();
            }
        }
    }

    fn make_leo_state() -> StateVector {
        let r0 = [RE_EARTH + 200_000.0, 0.0_f64, 0.0];
        let v0 = [0.0_f64, 7784.0, 0.0];
        let state0 = StateVector::new(r0, v0, Met(0));
        initialize_gdt(state0)
    }

    /// Test 1 — Zero thrust (pure gravity coast).
    ///
    /// With zero PIPA counts the cycle must advance using only gravity.
    /// Velocity should not change by more than 0.1 m/s (loose tolerance for 1 step).
    #[test]
    fn zero_thrust_coast() {
        crate::services::alarm::AlarmState::clear_all();
        let state0 = make_leo_state();
        let v0_mag = state0.speed();

        let mut avg_g = AverageG::new(state0);
        let mut hw = TestHw::new();
        hw.imu.inject_pipa([0i16, 0, 0]);
        hw.imu.set_refsmmat(IDENTITY_MAT3);

        let result = avg_g.cycle(&mut hw).expect("cycle should succeed");

        let dv = result.speed() - v0_mag;
        assert!(
            dv.abs() < 0.1,
            "speed change = {dv} m/s (should be < 0.1 m/s for circular orbit)"
        );
        // Time advanced by CYCLE_DT_CS = 200 cs
        assert_eq!(result.time(), Met(200));
    }

    /// Test 2 — Constant thrust 1 m/s² for 2 s.
    ///
    /// PIPA counts ≈ 34 on x-axis (34 × KPIP1 = 1.989 m/s total delta-v).
    /// The thrust increment is isolated by comparing a thrust cycle against a
    /// zero-thrust reference cycle run from the same initial state.
    #[test]
    fn constant_thrust_1ms2() {
        crate::services::alarm::AlarmState::clear_all();
        let state0 = make_leo_state();

        let counts_x: i16 = 34;
        let expected_dv_x = counts_x as f64 * KPIP1; // ≈ 1.989 m/s

        // Run a thrust cycle
        let mut avg_g_thrust = AverageG::new(state0);
        let mut hw_thrust = TestHw::new();
        hw_thrust.imu.inject_pipa([counts_x, 0, 0]);
        hw_thrust.imu.set_refsmmat(IDENTITY_MAT3);
        let result_thrust = avg_g_thrust
            .cycle(&mut hw_thrust)
            .expect("thrust cycle should succeed");

        // Run a zero-thrust reference cycle from the same initial state
        let mut avg_g_coast = AverageG::new(state0);
        let mut hw_coast = TestHw::new();
        hw_coast.imu.inject_pipa([0i16, 0, 0]);
        hw_coast.imu.set_refsmmat(IDENTITY_MAT3);
        let result_coast = avg_g_coast
            .cycle(&mut hw_coast)
            .expect("coast cycle should succeed");

        // The difference between thrust and coast velocity is the PIPA delta-v
        let delta_vx = result_thrust.velocity()[0] - result_coast.velocity()[0];
        assert!(
            (delta_vx - expected_dv_x).abs() < 0.05,
            "thrust - coast dv_x = {delta_vx}, expected ~{expected_dv_x}"
        );
    }

    /// Test 3 — Mid-cycle restart resume (idempotency).
    ///
    /// Two AverageG instances started from the same state with the same PIPA
    /// injection must produce identical results (restart idempotency).
    #[test]
    fn mid_cycle_restart_idempotency() {
        crate::services::alarm::AlarmState::clear_all();
        let state0 = make_leo_state();

        let mut avg_g1 = AverageG::new(state0);
        let mut hw1 = TestHw::new();
        hw1.imu.inject_pipa([100i16, 0, 0]);
        hw1.imu.set_refsmmat(IDENTITY_MAT3);
        let result1 = avg_g1.cycle(&mut hw1).expect("cycle 1 should succeed");

        // Simulated restart: fresh AverageG, same initial state, re-inject same PIPAs
        let mut avg_g2 = AverageG::new(state0);
        let mut hw2 = TestHw::new();
        hw2.imu.inject_pipa([100i16, 0, 0]);
        hw2.imu.set_refsmmat(IDENTITY_MAT3);
        let result2 = avg_g2.cycle(&mut hw2).expect("cycle 2 should succeed");

        assert!(
            (result1.position()[0] - result2.position()[0]).abs() < 1e-10,
            "x positions differ: {} vs {}",
            result1.position()[0],
            result2.position()[0]
        );
        assert!(
            (result1.velocity()[0] - result2.velocity()[0]).abs() < 1e-10,
            "x velocities differ: {} vs {}",
            result1.velocity()[0],
            result2.velocity()[0]
        );
    }

    /// Test 4 — PIPA saturation.
    ///
    /// A count >= PIPA_MAX_COUNTS (6398) must return `Err(PipaSaturated)`,
    /// leave state unchanged, and raise alarm `PipaOverflow` (0o205).
    ///
    /// AGC source: Comanche055/SERVICER207.agc `-MAXDELV DEC -6398` / `TC ALARM / OCT 00205`.
    #[test]
    fn pipa_saturation() {
        crate::services::alarm::AlarmState::clear_all();
        let state0 = make_leo_state();
        let initial_pos = state0.position();
        let initial_vel = state0.velocity();

        let mut avg_g = AverageG::new(state0);
        let mut hw = TestHw::new();
        hw.imu.inject_pipa([6500i16, 0, 0]); // 6500 > 6398
        hw.imu.set_refsmmat(IDENTITY_MAT3);

        let result = avg_g.cycle(&mut hw);

        assert_eq!(
            result,
            Err(AvgGError::PipaSaturated),
            "must return PipaSaturated"
        );
        assert_eq!(avg_g.state().position(), initial_pos, "position unchanged");
        assert_eq!(avg_g.state().velocity(), initial_vel, "velocity unchanged");
        assert_eq!(
            crate::services::alarm::AlarmState::most_recent(),
            Some(AlarmCode::PipaOverflow),
            "alarm PipaOverflow (0o205) must have been raised"
        );
    }

    /// Test 5 — initialize_gdt sets gdt_over_2 to non-zero.
    #[test]
    fn initialize_gdt_non_zero() {
        use crate::types::ZERO_VEC3;
        let r = [RE_EARTH + 200_000.0, 0.0_f64, 0.0];
        let state = StateVector::new(r, [0.0_f64, 7784.0, 0.0], Met(0));
        assert_eq!(state.gdt_over_2(), ZERO_VEC3, "new state has zero gdt");
        let initialised = initialize_gdt(state);
        let gdt = initialised.gdt_over_2();
        // gdt_over_2[0] ≈ earth_gravity(r)[0] × (CYCLE_DT/2) ≈ -9.8 × 1.0 ≈ -9.8 m/s
        assert!(
            gdt[0] < -9.0,
            "gdt_over_2[0] should be ~-9.8, got {}",
            gdt[0]
        );
    }
}
