//! Digital Autopilot (DAP) supervisor — top-level control-cycle driver.
//!
//! Manages the T5RUPT-driven RCS and TVC control cycles, mode transitions,
//! and the FRESHDAP / REDAP initialization sequences.
//!
//! AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
//!   RCSATT, REDORCS, SETT5, FRESHDAP, REDAP, T5PHASE2, ZEROJET (pages 1002-1024).
//! AGC source: Comanche055/TVCEXECUTIVE.agc
//!   TVCEXEC, VARGAINS, ROLLPREP (pages 945-950).
//! AGC source: Comanche055/TVCDAPS.agc
//!   PITCHDAP, YAWDAP, DAPINIT, ACTLIM (pages 961-978).

use crate::control::attitude::{compute_error, phase_plane_decision, AttitudeError, JetDecision};
use crate::control::constants::{DEADBAND_DEFAULT_RAD, TVC_ROLL_DEADBAND_RAD};
use crate::control::rcs_logic::select_jets;
use crate::control::tvc::{steer, TvcGains};
use crate::hal::engine::EngineIo;
use crate::hal::rcs::RcsIo;
use crate::hal::{AgcHardware, ImuIo};
use crate::services::alarm::{AlarmCode, AlarmState};
use crate::types::CduAngle;
use crate::types::Mat3x3;
use crate::types::IDENTITY_MAT3;

/// Target attitude for the DAP, expressed as three CDU commanded angles.
///
/// Corresponds to CDUXD/CDUYD/CDUZD erasable registers.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc DCDUINCR routine.
#[derive(Clone, Copy, Debug, Default)]
pub struct AttitudeTarget {
    /// Commanded CDU X (inner gimbal / pitch axis).
    pub x: CduAngle,
    /// Commanded CDU Y (middle gimbal / yaw axis).
    pub y: CduAngle,
    /// Commanded CDU Z (outer gimbal / roll axis).
    pub z: CduAngle,
}

/// DAP operating mode.
///
/// Mirrors the T5PHASE + FLAGWRD6 state machine in Comanche055.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc (T5PHASE encoding, pp. 1002-1003);
///             TVCEXECUTIVE.agc TVCEXFIN (TVC termination).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DapMode {
    /// No autopilot activity. T5RUPT not hooked; jets all off.
    Idle,
    /// RCS attitude hold / auto-maneuver. Driven by T5RUPT at 100 ms.
    Rcs,
    /// TVC (SPS burn) mode. Pitch/yaw driven by T5 tasks; roll by Waitlist.
    Tvc,
}

/// Digital Autopilot supervisor state.
///
/// All fields are `Copy` / statically sized — no heap.
/// Shared-mutable access (ISR ↔ foreground) must be wrapped in
/// `cortex_m::interrupt::Mutex<RefCell<Dap>>` by the caller.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc (T5PHASE, HOLDFLAG, ERRORX/Y/Z).
#[derive(Clone, Copy, Debug)]
pub struct Dap {
    /// Current DAP operating mode.
    pub mode: DapMode,

    /// Commanded attitude (CDUXD/CDUYD/CDUZD).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc CDUXD/CDUYD/CDUZD registers.
    pub target: AttitudeTarget,

    /// T5 phase counter. Mirrors T5PHASE erasable.
    ///
    /// Positive → FRESHDAP, zero → Phase2, negative → Phase1/REDAP.
    ///
    /// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc T5PHASE encoding (pp. 1002-1003).
    pub t5_phase: i16,

    /// HOLDFLAG: positive = attitude hold, negative = auto steer.
    ///
    /// AGC source: HOLDFLAG erasable, RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1007.
    pub hold_flag: i16,

    /// Cumulative attitude errors (ERRORX/Y/Z), in raw error units (radians internally).
    ///
    /// AGC source: ERRORX/ERRORY/ERRORZ, RCS-CSM_DIGITAL_AUTOPILOT.agc MERUPDAT (p. 1020).
    pub error: [i16; 3],

    /// Desired attitude rotation matrix (REFSMMAT × target rotation).
    ///
    /// Set by KALCMANU / guidance layer before calling `t5rupt_tick`.
    /// Initialized to identity (no rotation from REFSMMAT).
    pub desired_mat: Mat3x3,

    /// TVC control gains.  Used in Tvc mode.
    pub tvc_gains: TvcGains,

    /// Last computed attitude error (radians), for rate estimation.
    pub last_error: AttitudeError,
}

impl Dap {
    /// Construct an idle DAP state (power-on default).
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc — all control registers zeroed.
    pub const fn new() -> Self {
        Self {
            mode: DapMode::Idle,
            target: AttitudeTarget {
                x: CduAngle(0),
                y: CduAngle(0),
                z: CduAngle(0),
            },
            t5_phase: 0,
            hold_flag: 1, // positive = attitude hold (default)
            error: [0; 3],
            desired_mat: IDENTITY_MAT3,
            tvc_gains: TvcGains::NOMINAL,
            last_error: AttitudeError::ZERO,
        }
    }
}

impl Default for Dap {
    fn default() -> Self {
        Self::new()
    }
}

/// Process one T5RUPT tick of the DAP.
///
/// Must complete in under 1 ms (no blocking, no heap, no spin-wait).
/// Called from an ISR context — caller must hold the Mutex critical section.
///
/// In **RCS mode**: reads CDU angles, computes attitude error, delegates to
/// `rcs_logic::select_jets`, and fires `hw.rcs().fire_jets()`.
/// In **TVC mode**: computes gimbal commands via `tvc::steer` and calls
/// `hw.engine().set_gimbal_angles`.
/// In **Idle mode**: this function is a no-op.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
///   RCSATT (Phase 1), T5PHASE2 (Phase 2), JETSLECT handoff.
pub fn t5rupt_tick<H: AgcHardware>(dap: &mut Dap, hw: &mut H) {
    match dap.mode {
        DapMode::Idle => {
            // Idle: no autopilot activity (NORATE path in AGC)
        }

        DapMode::Rcs => {
            // Read CDU angles
            let cdus = hw.imu().read_cdu();

            // Check IMU status (CHAN30): bit 13 = IMU fail (GOJAM path)
            let status = hw.imu().read_status();
            if status & (1 << 13) != 0 {
                // IMU tilt detected — raise alarm and return without firing jets
                AlarmState::raise(AlarmCode::DeviceConflict);
                hw.rcs().all_jets_off();
                return;
            }

            // Compute attitude error (MERUPDAT / GETAKS)
            let att_err = compute_error(&cdus, &dap.desired_mat);

            // Rate estimate: finite-difference of error (placeholder 0 when no history)
            // In a full implementation, DRHO from the rate filter would be used.
            // Here we use a simplified zero-rate estimate, which gives attitude-hold only.
            let rate = AttitudeError::ZERO;

            // Phase-plane switching (T5PHASE2 / JETS label)
            let pitch_dec = phase_plane_decision(att_err.pitch, rate.pitch, DEADBAND_DEFAULT_RAD);
            let yaw_dec = phase_plane_decision(att_err.yaw, rate.yaw, DEADBAND_DEFAULT_RAD);
            let roll_dec = phase_plane_decision(att_err.roll, rate.roll, DEADBAND_DEFAULT_RAD);

            // Convert to channel words and fire jets (T6START)
            let cmd = select_jets(pitch_dec, yaw_dec, roll_dec);
            hw.rcs().fire_jets(cmd);

            // Store error for next cycle rate estimation
            dap.last_error = att_err;

            // Update integer error registers (ERRORX/Y/Z) — scale: π/32768 rad/count
            let scale = 32768.0 / core::f64::consts::PI;
            dap.error[0] = (att_err.pitch * scale).clamp(-32768.0, 32767.0) as i16;
            dap.error[1] = (att_err.yaw * scale).clamp(-32768.0, 32767.0) as i16;
            dap.error[2] = (att_err.roll * scale).clamp(-32768.0, 32767.0) as i16;
        }

        DapMode::Tvc => {
            // TVC mode: drive gimbal actuators for pitch and yaw.
            // Roll is handled via RCS jets (TVC roll DAP, TVCROLLDAP.agc).

            let cdus = hw.imu().read_cdu();
            let att_err = compute_error(&cdus, &dap.desired_mat);
            let rate = AttitudeError::ZERO; // simplified: use zero rate

            // Compute gimbal commands (PITCHDAP / YAWDAP simplified to PD law)
            let (pitch_cmd_rad, yaw_cmd_rad) = steer(&att_err, &rate, &dap.tvc_gains);

            // Write to engine gimbal (TVCPITCH / TVCYAW)
            // Convert radians to CDU delta counts: 1 CDU count = 2π/65536 rad
            let rad_to_counts = 65536.0 / core::f64::consts::TAU;
            let pitch_delta = (pitch_cmd_rad * rad_to_counts) as i16;
            let yaw_delta = (yaw_cmd_rad * rad_to_counts) as i16;
            hw.engine().trim_pitch(pitch_delta);
            hw.engine().trim_yaw(yaw_delta);

            // Roll via RCS (TVCROLLDAP / ROLLOGIC — phase-plane with 5° deadband)
            let roll_dec = phase_plane_decision(att_err.roll, rate.roll, TVC_ROLL_DEADBAND_RAD);
            let roll_cmd = select_jets(JetDecision::None, JetDecision::None, roll_dec);
            hw.rcs().fire_jets(roll_cmd);

            dap.last_error = att_err;
        }
    }
}

/// Change the DAP operating mode.
///
/// Idle → Rcs: runs FRESHDAP initialisation (zeros rate filter variables,
///   sets T5PHASE = 0, hooks T5RUPT to RCSATT).
/// Rcs → Tvc: arms PITCHDAP/YAWDAP T5 chain; ROLLDAP scheduled via Waitlist.
/// Any → Idle: calls hw.rcs().all_jets_off(); unhooks T5RUPT.
///
/// AGC source: FRESHDAP (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1014),
///             TVCEXEC (TVCEXECUTIVE.agc p. 946),
///             TVCEXFIN (TVCEXECUTIVE.agc p. 949).
pub fn set_mode<H: AgcHardware>(dap: &mut Dap, mode: DapMode, hw: &mut H) {
    match mode {
        DapMode::Idle => {
            // Any → Idle: TVCEXFIN path: clear FLAGWRD6 bits 15,14
            dap.mode = DapMode::Idle;
            dap.t5_phase = 0;
            hw.rcs().all_jets_off();
        }
        DapMode::Rcs => {
            // Idle/Tvc → Rcs: run FRESHDAP initialisation
            // AGC: FRESHDAP zeros DRHO/1/2, ADOT/1/2, sets T5PHASE=0
            dap.mode = DapMode::Rcs;
            dap.t5_phase = 0; // Phase 2 startup
            dap.error = [0; 3];
            dap.last_error = AttitudeError::ZERO;
            // Jets off at transition (ZEROJET)
            hw.rcs().all_jets_off();
        }
        DapMode::Tvc => {
            // Rcs → Tvc: arm TVC T5 chain (TVCDAPON → TVCINIT4)
            dap.mode = DapMode::Tvc;
            dap.t5_phase = 0;
            dap.error = [0; 3];
            dap.last_error = AttitudeError::ZERO;
        }
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::rcs::{RcsImpl, RcsIo};

    // Minimal mock hardware for DAP tests.
    // We need a type that implements AgcHardware; use the full set of impls from HAL.
    use crate::hal::{
        dsky::{DigitRow, DskyIo, Key, RelayWord},
        engine::EngineImpl,
        imu::ImuImpl,
        imu::Unaligned,
        optics::OpticsImpl,
        telemetry::TelemetryImpl,
        timers::TimersImpl,
        uplink::UplinkImpl,
    };

    /// Minimal in-test DSKY stub: satisfies DskyIo with no-op implementations.
    struct StubDsky;

    impl DskyIo for StubDsky {
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

    struct MockHw {
        imu: ImuImpl<Unaligned>,
        rcs: RcsImpl,
        engine: EngineImpl,
        timers: TimersImpl,
        dsky: StubDsky,
        optics: OpticsImpl,
        uplink: UplinkImpl,
        telemetry: TelemetryImpl,
    }

    impl MockHw {
        fn new() -> Self {
            Self {
                imu: ImuImpl::new(),
                rcs: RcsImpl::new(),
                engine: EngineImpl::new(),
                timers: TimersImpl::new(),
                dsky: StubDsky,
                optics: OpticsImpl::new(),
                uplink: UplinkImpl::new(),
                telemetry: TelemetryImpl::new(),
            }
        }
    }

    impl AgcHardware for MockHw {
        type Timers = TimersImpl;
        type Dsky = StubDsky;
        type Imu = ImuImpl<Unaligned>;
        type Optics = OpticsImpl;
        type Engine = EngineImpl;
        type Rcs = RcsImpl;
        type Uplink = UplinkImpl;
        type Telemetry = TelemetryImpl;

        fn timers(&mut self) -> &mut Self::Timers {
            &mut self.timers
        }
        fn dsky(&mut self) -> &mut Self::Dsky {
            &mut self.dsky
        }
        fn imu(&mut self) -> &mut Self::Imu {
            &mut self.imu
        }
        fn optics(&mut self) -> &mut Self::Optics {
            &mut self.optics
        }
        fn engine(&mut self) -> &mut Self::Engine {
            &mut self.engine
        }
        fn rcs(&mut self) -> &mut Self::Rcs {
            &mut self.rcs
        }
        fn uplink(&mut self) -> &mut Self::Uplink {
            &mut self.uplink
        }
        fn telemetry(&mut self) -> &mut Self::Telemetry {
            &mut self.telemetry
        }
        fn pet_watchdog(&mut self) {}
        fn hardware_restart(&mut self) -> ! {
            loop {}
        }
    }

    /// TC-DAP-1: Mode transition Idle → Rcs.
    ///
    /// After set_mode(Rcs), dap.mode == Rcs, dap.t5_phase == 0, jets OFF.
    #[test]
    fn idle_to_rcs_transition() {
        let mut dap = Dap::new();
        let mut hw = MockHw::new();

        set_mode(&mut dap, DapMode::Rcs, &mut hw);

        assert_eq!(dap.mode, DapMode::Rcs, "mode must be Rcs after transition");
        assert_eq!(dap.t5_phase, 0, "t5_phase must be 0 (Phase 2 startup)");
        let cmd = hw.rcs().current_command();
        assert_eq!(cmd.pitch_yaw, 0, "jets must be off at Rcs entry");
        assert_eq!(cmd.roll, 0, "jets must be off at Rcs entry");
    }

    /// TC-DAP-2: Idle mode tick is a no-op.
    ///
    /// JetCommand remains OFF, no CDU read attempted.
    #[test]
    fn idle_mode_noop() {
        let mut dap = Dap::new();
        assert_eq!(dap.mode, DapMode::Idle);
        let mut hw = MockHw::new();

        t5rupt_tick(&mut dap, &mut hw);

        let cmd = hw.rcs().current_command();
        assert_eq!(cmd.pitch_yaw, 0, "no jets in Idle mode");
        assert_eq!(cmd.roll, 0, "no jets in Idle mode");
    }

    /// TC-DAP-3: RCS tick with positive pitch error fires pitch jets.
    ///
    /// Set desired_mat to a rotation that creates a positive pitch error (> deadband)
    /// vs CDU at zero.  Expected: pitch_yaw has PJETS bits set, roll = 0.
    #[test]
    fn rcs_tick_positive_pitch_fires_jets() {
        use crate::math::linalg::roty;

        let mut dap = Dap::new();
        let mut hw = MockHw::new();

        // Set desired attitude: 1° pitch rotation (above deadband of 0.3°)
        dap.desired_mat = roty(1.0_f64.to_radians());
        set_mode(&mut dap, DapMode::Rcs, &mut hw);

        t5rupt_tick(&mut dap, &mut hw);

        let cmd = hw.rcs().current_command();
        // Some pitch bits should be set (non-zero PJETS portion)
        assert_ne!(
            cmd.pitch_yaw & crate::hal::rcs::PJETS_MASK,
            0,
            "pitch jets should fire for positive pitch error"
        );
        assert_eq!(cmd.roll, 0, "roll should not fire for pure pitch error");
    }

    /// TC-DAP-4: TVC tick drives gimbal and does not write to pitch_yaw jets (only roll).
    ///
    /// In Tvc mode, engine trim is updated (not checked here since EngineImpl is minimal)
    /// and rcs().current_command() gets only roll bits (pitch/yaw go to gimbal).
    #[test]
    fn tvc_mode_drives_gimbal_not_pyjets() {
        use crate::math::linalg::roty;

        let mut dap = Dap::new();
        let mut hw = MockHw::new();

        // Small pitch error in TVC mode
        dap.desired_mat = roty(2.0_f64.to_radians());
        set_mode(&mut dap, DapMode::Tvc, &mut hw);

        t5rupt_tick(&mut dap, &mut hw);

        // In TVC mode, pitch/yaw go to engine gimbal (not channel 5 PYJETS)
        // Roll goes to RCS. Here roll error is zero so roll bits should be 0.
        let cmd = hw.rcs().current_command();
        assert_eq!(
            cmd.pitch_yaw & crate::hal::rcs::PJETS_MASK,
            0,
            "PYJETS should not fire in TVC mode (pitch/yaw go to gimbal)"
        );
    }
}
