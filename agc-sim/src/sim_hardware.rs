//! `SimHardware` — host-side implementation of all AGC HAL sub-traits.
//!
//! This struct provides a fully software-modelled peripheral set that the
//! flight software can drive without any real hardware or TTY.  It is the
//! implementation used by all integration tests and the `agc_sim` binary.
//!
//! The IMU starts in `Unaligned` state.  Call `sim_coarse_align()` /
//! `sim_fine_align()` on the `SimImu` to advance the typestate.

use agc_core::hal::{
    dsky::{DigitRow, DskyIo, Key, RelayWord},
    engine::EngineIo,
    imu::{CoarseAligned, FineAligned, ImuIo, Unaligned},
    optics::OpticsIo,
    rcs::{JetCommand, RcsIo},
    telemetry::TelemetryIo,
    timers::Timers,
    uplink::UplinkIo,
    AgcHardware,
};
use agc_core::services::v_n::VnState;
use agc_core::types::{CduAngle, Mat3x3, IDENTITY_MAT3};
use agc_core::AgcState;

use crate::{dsky_state::DskyDisplayState, sim_log::SimLog};

// ── SimImu ────────────────────────────────────────────────────────────────────

/// Simulated IMU with typestate alignment tracking.
///
/// `State` is one of `Unaligned`, `CoarseAligned`, `FineAligned`.
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc (alignment state machine).
pub struct SimImu<State> {
    /// Channel 12 control register shadow (last written value).
    control_shadow: u16,
    /// CDU gimbal angles — test code drives these directly.
    cdus: [CduAngle; 3],
    /// PIPA delta-V accumulators; read_pipa clears them.
    pipas: [i16; 3],
    /// Channel 30 status shadow.
    status: u16,
    /// Reference Stable-Member Matrix (SM → ECI rotation).
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc `REFSMMAT ERASE +17D`.
    refsmmat: Mat3x3,
    _state: core::marker::PhantomData<State>,
}

impl SimImu<Unaligned> {
    /// Construct in the `Unaligned` state with all registers zeroed.
    pub fn new() -> Self {
        Self {
            control_shadow: 0,
            cdus: [CduAngle(0); 3],
            pipas: [0; 3],
            status: 0,
            refsmmat: IDENTITY_MAT3,
            _state: core::marker::PhantomData,
        }
    }

    /// Advance to `CoarseAligned` state (sets CHAN12 bit 4).
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUCOARS, page 1423.
    pub fn coarse_align(mut self) -> SimImu<CoarseAligned> {
        self.control_shadow |= 1 << 3; // bit 4 (1-based) = bit 3 (0-based)
        SimImu {
            control_shadow: self.control_shadow,
            cdus: self.cdus,
            pipas: self.pipas,
            status: self.status,
            refsmmat: self.refsmmat,
            _state: core::marker::PhantomData,
        }
    }

    /// Release the raw control shadow (C-FREE).
    pub fn free(self) -> u16 {
        self.control_shadow
    }
}

impl Default for SimImu<Unaligned> {
    fn default() -> Self {
        Self::new()
    }
}

impl SimImu<CoarseAligned> {
    /// Advance to `FineAligned` state (clears coarse and zero bits).
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUFINE, page 1427.
    pub fn fine_align(mut self) -> SimImu<FineAligned> {
        self.control_shadow &= !((1 << 3) | (1 << 4));
        SimImu {
            control_shadow: self.control_shadow,
            cdus: self.cdus,
            pipas: self.pipas,
            status: self.status,
            refsmmat: self.refsmmat,
            _state: core::marker::PhantomData,
        }
    }

    /// Release the raw control shadow (C-FREE).
    pub fn free(self) -> u16 {
        self.control_shadow
    }
}

impl SimImu<FineAligned> {
    /// Revert to `Unaligned` (e.g. on simulated IMU fail).
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc MODABORT path.
    pub fn into_unaligned(mut self) -> SimImu<Unaligned> {
        self.control_shadow = 0;
        SimImu {
            control_shadow: 0,
            cdus: self.cdus,
            pipas: self.pipas,
            status: self.status,
            refsmmat: self.refsmmat,
            _state: core::marker::PhantomData,
        }
    }

    /// Release the raw control shadow (C-FREE).
    pub fn free(self) -> u16 {
        self.control_shadow
    }
}

/// Implement `ImuIo` for all three alignment states via a macro.
macro_rules! impl_sim_imu_io {
    ($state:ty) => {
        impl ImuIo for SimImu<$state> {
            fn read_cdu(&self) -> [CduAngle; 3] {
                self.cdus
            }

            fn read_pipa(&mut self) -> [i16; 3] {
                let counts = self.pipas;
                self.pipas = [0; 3];
                counts
            }

            fn torque_gyro(&mut self, _axis: usize, _pulses: i16) {}

            fn read_status(&self) -> u16 {
                self.status
            }

            fn write_control(&mut self, bits: u16) {
                self.control_shadow = bits;
            }

            fn write_cdu_commands(&mut self, _cmds: [i16; 3]) {}

            fn coarse_align_active(&self) -> bool {
                (self.control_shadow >> 3) & 1 != 0
            }

            fn set_coarse_align(&mut self) {
                self.control_shadow |= 1 << 3;
            }

            fn refsmmat(&self) -> Mat3x3 {
                self.refsmmat
            }
        }
    };
}

impl_sim_imu_io!(Unaligned);
impl_sim_imu_io!(CoarseAligned);
impl_sim_imu_io!(FineAligned);

/// Test-injection helpers for SimImu (all alignment states).
///
/// These methods allow integration tests in `agc-test` to set CDU angles and
/// PIPA counts directly without going through the HAL write path, mirroring
/// the back-door access pattern used in agc-sim's own unit tests.
macro_rules! impl_sim_imu_test_helpers {
    ($state:ty) => {
        impl SimImu<$state> {
            /// Inject CDU gimbal angles for all three axes directly.
            ///
            /// Used by integration tests to set up expected CDU state before
            /// reading back through `ImuIo::read_cdu()`.
            pub fn inject_cdus(&mut self, cdus: [CduAngle; 3]) {
                self.cdus = cdus;
            }

            /// Inject PIPA delta-V accumulator counts for all three axes.
            ///
            /// Used by integration tests to set up expected PIPA state before
            /// reading back through `ImuIo::read_pipa()`.
            pub fn inject_pipas(&mut self, pipas: [i16; 3]) {
                self.pipas = pipas;
            }

            /// Set the REFSMMAT for simulator injection and integration tests.
            ///
            /// AGC: REFSMMAT is loaded during P52 IMU alignment.
            pub fn set_refsmmat(&mut self, m: Mat3x3) {
                self.refsmmat = m;
            }
        }
    };
}

/// Test-injection helper for the Channel 30 status register (all alignment states).
macro_rules! impl_sim_imu_status_inject {
    ($state:ty) => {
        impl SimImu<$state> {
            /// Inject a raw Channel 30 status word for testing alarm/failure paths.
            ///
            /// Bit 12 (0-based) = IMU fail; bit 9 = coarse-align complete, etc.
            /// Used by `p51_imu_align` tests to trigger the IMU-fail branch.
            pub fn inject_status(&mut self, status: u16) {
                self.status = status;
            }
        }
    };
}

impl_sim_imu_test_helpers!(Unaligned);
impl_sim_imu_test_helpers!(CoarseAligned);
impl_sim_imu_test_helpers!(FineAligned);

impl_sim_imu_status_inject!(Unaligned);
impl_sim_imu_status_inject!(CoarseAligned);
impl_sim_imu_status_inject!(FineAligned);

// ── SimDsky ───────────────────────────────────────────────────────────────────

/// Simulated DSKY peripheral.
///
/// Stores display state internally and forwards reads/writes to `DskyDisplayState`.
/// Key presses are injected through `DskyDisplayState::enqueue_key`.
pub struct SimDsky {
    /// Shared display state (the TUI reads from this).
    pub display: DskyDisplayState,
    /// Proceed button latch.
    proceed: bool,
}

impl SimDsky {
    /// Construct with blank display and no pending keys.
    pub fn new() -> Self {
        Self {
            display: DskyDisplayState::blank(),
            proceed: false,
        }
    }

    /// Release (C-FREE) — returns display state.
    pub fn free(self) -> DskyDisplayState {
        self.display
    }

    /// Set or clear the simulated PROCEED button latch.
    ///
    /// Used by integration tests to simulate crew PROCEED key presses.
    pub fn set_proceed(&mut self, pressed: bool) {
        self.proceed = pressed;
    }
}

impl Default for SimDsky {
    fn default() -> Self {
        Self::new()
    }
}

impl DskyIo for SimDsky {
    fn read_key(&mut self) -> Option<Key> {
        self.display.dequeue_key()
    }

    fn read_nav_key(&mut self) -> Option<Key> {
        // Nav DSKY shares the same key queue in the sim.
        None
    }

    fn write_relay(&mut self, _word: RelayWord) {
        // Full relay-word decoding is deferred to Milestone 5 (verb/noun dispatch).
    }

    fn write_lamp_word(&mut self, bits: u16) {
        self.display.apply_lamp_word(bits);
    }

    fn write_prog(&mut self, prog: u8) {
        self.display.prog = Some(prog);
    }

    fn write_verb(&mut self, verb: u8) {
        self.display.verb = Some(verb);
    }

    fn write_noun(&mut self, noun: u8) {
        self.display.noun = Some(noun);
    }

    fn write_register(&mut self, row: usize, value: &DigitRow) {
        // Decode sign and 5-digit value from DigitRow.
        let mut magnitude: i32 = 0;
        let mut valid = true;
        for &d in &value.digits {
            if d == 0xFF {
                valid = false;
                break;
            }
            magnitude = magnitude * 10 + d as i32;
        }
        let v = if valid {
            let signed = if value.sign_minus {
                -magnitude
            } else {
                magnitude
            };
            Some(signed)
        } else {
            None
        };
        match row {
            0 => self.display.r1 = v,
            1 => self.display.r2 = v,
            2 => self.display.r3 = v,
            _ => {}
        }
    }

    fn proceed_pressed(&self) -> bool {
        self.proceed
    }

    fn set_prog_light(&mut self, on: bool) {
        self.display.prog_light = on;
    }

    fn blank_display(&mut self) {
        self.display.prog = None;
        self.display.verb = None;
        self.display.noun = None;
    }
}

// ── SimOptics ─────────────────────────────────────────────────────────────────

/// Simulated optics CDU.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc CDUT/CDUS, CDUTCMD/CDUSCMD.
pub struct SimOptics {
    shaft: CduAngle,
    trunnion: CduAngle,
    chan14_shadow: u16,
}

impl SimOptics {
    /// Construct with zeroed positions.
    pub fn new() -> Self {
        Self {
            shaft: CduAngle(0),
            trunnion: CduAngle(0),
            chan14_shadow: 0,
        }
    }

    /// Release (C-FREE).
    pub fn free(self) -> u16 {
        self.chan14_shadow
    }
}

impl Default for SimOptics {
    fn default() -> Self {
        Self::new()
    }
}

impl OpticsIo for SimOptics {
    fn read_shaft(&self) -> CduAngle {
        self.shaft
    }

    fn read_trunnion(&self) -> CduAngle {
        self.trunnion
    }

    fn drive_shaft(&mut self, delta: i16) {
        self.shaft = CduAngle(self.shaft.0.wrapping_add(delta as u16));
    }

    fn drive_trunnion(&mut self, delta: i16) {
        self.trunnion = CduAngle(self.trunnion.0.wrapping_add(delta as u16));
    }

    fn write_chan14(&mut self, bits: u16) {
        self.chan14_shadow = bits;
    }
}

// ── SimEngine ─────────────────────────────────────────────────────────────────

/// Simulated SPS engine.
///
/// AGC source: Comanche055/P40-P47.agc engine control.
pub struct SimEngine {
    enabled: bool,
    tvc_pitch: CduAngle,
    tvc_yaw: CduAngle,
    dsalmout: u16,
}

impl SimEngine {
    /// Construct with engine disabled, gimbals at zero.
    pub fn new() -> Self {
        Self {
            enabled: false,
            tvc_pitch: CduAngle(0),
            tvc_yaw: CduAngle(0),
            dsalmout: 0,
        }
    }

    /// Release (C-FREE).
    pub fn free(self) -> u16 {
        self.dsalmout
    }
}

impl Default for SimEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl EngineIo for SimEngine {
    fn set_engine_enable(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    fn engine_enabled(&self) -> bool {
        self.enabled
    }

    fn trim_pitch(&mut self, delta: i16) {
        self.tvc_pitch = CduAngle(self.tvc_pitch.0.wrapping_add(delta as u16));
    }

    fn trim_yaw(&mut self, delta: i16) {
        self.tvc_yaw = CduAngle(self.tvc_yaw.0.wrapping_add(delta as u16));
    }

    fn read_tvc_pitch(&self) -> CduAngle {
        self.tvc_pitch
    }

    fn read_tvc_yaw(&self) -> CduAngle {
        self.tvc_yaw
    }

    fn write_dsalmout(&mut self, bits: u16) {
        self.dsalmout = bits;
    }
}

// ── SimRcs ────────────────────────────────────────────────────────────────────

/// Simulated RCS jet controller.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc.
pub struct SimRcs {
    command: JetCommand,
}

impl SimRcs {
    /// Construct with all jets off.
    pub fn new() -> Self {
        Self {
            command: JetCommand::OFF,
        }
    }

    /// Release (C-FREE).
    pub fn free(self) -> JetCommand {
        self.command
    }
}

impl Default for SimRcs {
    fn default() -> Self {
        Self::new()
    }
}

impl RcsIo for SimRcs {
    fn fire_jets(&mut self, cmd: JetCommand) {
        self.command = cmd;
    }

    fn all_jets_off(&mut self) {
        self.command = JetCommand::OFF;
    }

    fn current_command(&self) -> JetCommand {
        self.command
    }

    fn write_channel5(&mut self, word: u16) {
        self.command.pitch_yaw = word;
    }

    fn write_channel6(&mut self, word: u16) {
        self.command.roll = word;
    }
}

// ── SimTimers ─────────────────────────────────────────────────────────────────

/// Simulated timer registers.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc TIME3-TIME6.
pub struct SimTimers {
    t3: u16,
    t4: u16,
    t5: u16,
    t6: u16,
    t3_enabled: bool,
}

impl SimTimers {
    /// Construct at POSMAX (no immediate interrupt).
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSUB timer init.
    pub fn new() -> Self {
        use agc_core::hal::timers::{POSMAX, T4_INIT, T5_INIT};
        Self {
            t3: POSMAX,
            t4: T4_INIT,
            t5: T5_INIT,
            t6: POSMAX,
            t3_enabled: false,
        }
    }

    /// Release (C-FREE).
    pub fn free(self) -> (u16, u16, u16, u16) {
        (self.t3, self.t4, self.t5, self.t6)
    }
}

impl Default for SimTimers {
    fn default() -> Self {
        Self::new()
    }
}

impl Timers for SimTimers {
    fn set_t3(&mut self, centiseconds: u16) {
        self.t3 = centiseconds;
    }

    fn set_t4(&mut self, centiseconds: u16) {
        self.t4 = centiseconds;
    }

    fn set_t5(&mut self, centiseconds: u16) {
        self.t5 = centiseconds;
    }

    fn set_t6(&mut self, centiseconds: u16) {
        self.t6 = centiseconds;
    }

    fn read_t3(&self) -> u16 {
        self.t3
    }

    fn read_t4(&self) -> u16 {
        self.t4
    }

    fn read_t5(&self) -> u16 {
        self.t5
    }

    fn read_t6(&self) -> u16 {
        self.t6
    }

    fn disable_t3(&mut self) {
        self.t3_enabled = false;
    }

    fn enable_t3(&mut self) {
        self.t3_enabled = true;
    }
}

// ── SimUplink ─────────────────────────────────────────────────────────────────

/// Simulated uplink receiver.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc INLINK = octal 45.
pub struct SimUplink {
    pending: Option<u8>,
    overrun: bool,
}

impl SimUplink {
    /// Construct with no pending word.
    pub fn new() -> Self {
        Self {
            pending: None,
            overrun: false,
        }
    }

    /// Release (C-FREE).
    pub fn free(self) -> Option<u8> {
        self.pending
    }
}

impl Default for SimUplink {
    fn default() -> Self {
        Self::new()
    }
}

impl UplinkIo for SimUplink {
    fn read_uplink_word(&mut self) -> Option<u8> {
        self.pending.take()
    }

    fn uplink_overrun(&self) -> bool {
        self.overrun
    }

    fn clear_overrun(&mut self) {
        self.overrun = false;
    }
}

// ── SimTelemetry ──────────────────────────────────────────────────────────────

/// Simulated PCM telemetry downlink.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc DNTM1/DNTM2 (octal 34-35).
pub struct SimTelemetry {
    last_word1: u16,
    last_word2: u16,
    overrun: bool,
    chan13: u16,
}

impl SimTelemetry {
    /// Construct with no pending words.
    pub fn new() -> Self {
        Self {
            last_word1: 0,
            last_word2: 0,
            overrun: false,
            chan13: 0,
        }
    }

    /// Release (C-FREE).
    pub fn free(self) -> (u16, u16) {
        (self.last_word1, self.last_word2)
    }
}

impl Default for SimTelemetry {
    fn default() -> Self {
        Self::new()
    }
}

impl TelemetryIo for SimTelemetry {
    fn write_downlink_pair(&mut self, word1: u16, word2: u16) {
        self.last_word1 = word1;
        self.last_word2 = word2;
    }

    fn downlink_overrun(&self) -> bool {
        self.overrun
    }

    fn clear_overrun(&mut self) {
        self.overrun = false;
    }

    fn write_chan13(&mut self, bits: u16) {
        self.chan13 = bits;
    }
}

// ── SimHardware ───────────────────────────────────────────────────────────────

/// Complete simulated hardware platform.
///
/// Holds all 8 simulated peripherals and the mission log.  The IMU is kept in
/// `SimImu<Unaligned>` for the `AgcHardware` implementation; use
/// `SimHardware::new_headless()` in tests.
///
/// The IMU starts in `Unaligned`; call `hw.imu().set_coarse_align()` or
/// use `fresh_start()` to advance the alignment state.
pub struct SimHardware {
    /// Simulated timers.
    pub timers: SimTimers,
    /// Simulated DSKY (display + key queue).
    pub dsky: SimDsky,
    /// Simulated IMU (always `Unaligned` for the trait object; typestate
    /// transitions are made via the concrete `SimImu` methods before embedding).
    pub imu: SimImu<Unaligned>,
    /// Simulated optics.
    pub optics: SimOptics,
    /// Simulated SPS engine.
    pub engine: SimEngine,
    /// Simulated RCS.
    pub rcs: SimRcs,
    /// Simulated uplink.
    pub uplink: SimUplink,
    /// Simulated telemetry downlink.
    pub telemetry: SimTelemetry,
    /// Mission log — all I/O events are appended here.
    pub log: SimLog,
    /// Watchdog pet counter (incremented each `pet_watchdog` call).
    pub watchdog_count: u64,
    /// AGC erasable state (verb/noun state machine, navigation, programs).
    pub agc_state: AgcState,
    /// PINBALL verb/noun keyboard state machine.
    pub vn: VnState,
    /// Current simulation time multiplier (0.25× to 16×, default 1×).
    pub time_multiplier: f64,
}

impl SimHardware {
    /// Construct a headless `SimHardware` suitable for tests and CI.
    ///
    /// All peripherals start in a safe, zeroed state.  No TTY is required.
    pub fn new_headless() -> Self {
        Self {
            timers: SimTimers::new(),
            dsky: SimDsky::new(),
            imu: SimImu::new(),
            optics: SimOptics::new(),
            engine: SimEngine::new(),
            rcs: SimRcs::new(),
            uplink: SimUplink::new(),
            telemetry: SimTelemetry::new(),
            log: SimLog::new(),
            watchdog_count: 0,
            agc_state: AgcState::new(),
            vn: VnState::new(),
            time_multiplier: 1.0,
        }
    }
}

impl AgcHardware for SimHardware {
    type Timers = SimTimers;
    type Dsky = SimDsky;
    type Imu = SimImu<Unaligned>;
    type Optics = SimOptics;
    type Engine = SimEngine;
    type Rcs = SimRcs;
    type Uplink = SimUplink;
    type Telemetry = SimTelemetry;

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

    fn pet_watchdog(&mut self) {
        self.watchdog_count = self.watchdog_count.saturating_add(1);
        self.log.info("watchdog pet");
    }

    fn hardware_restart(&mut self) -> ! {
        self.log
            .error("hardware_restart triggered — sim cannot continue");
        // In the simulator, we panic to unwind test infrastructure.
        // Real target spins until the hardware watchdog fires.
        panic!("SimHardware::hardware_restart")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use agc_core::hal::AgcHardware;

    #[test]
    fn headless_construction() {
        let hw = SimHardware::new_headless();
        assert_eq!(hw.watchdog_count, 0);
        assert!(!hw.engine.engine_enabled());
        assert_eq!(hw.rcs.current_command().pitch_yaw, 0);
    }

    #[test]
    fn pet_watchdog_increments_counter() {
        let mut hw = SimHardware::new_headless();
        hw.pet_watchdog();
        hw.pet_watchdog();
        assert_eq!(hw.watchdog_count, 2);
    }

    #[test]
    fn dsky_key_queue_roundtrip() {
        let mut hw = SimHardware::new_headless();
        hw.dsky.display.enqueue_key(Key::Enter);
        assert_eq!(hw.dsky().read_key(), Some(Key::Enter));
        assert_eq!(hw.dsky().read_key(), None);
    }

    #[test]
    fn pipa_read_and_clear() {
        let mut hw = SimHardware::new_headless();
        // Inject PIPA counts directly (test back-door).
        hw.imu.pipas = [10, -5, 3];
        let counts = hw.imu().read_pipa();
        assert_eq!(counts, [10, -5, 3]);
        // Second read must return zeros (cleared).
        assert_eq!(hw.imu().read_pipa(), [0, 0, 0]);
    }

    #[test]
    fn coarse_align_set_via_imu_io() {
        let mut hw = SimHardware::new_headless();
        assert!(!hw.imu().coarse_align_active());
        hw.imu().set_coarse_align();
        assert!(hw.imu().coarse_align_active());
    }

    #[test]
    fn sim_imu_typestate_transitions() {
        let imu: SimImu<Unaligned> = SimImu::new();
        let imu_ca = imu.coarse_align();
        assert!(imu_ca.coarse_align_active());
        let imu_fa = imu_ca.fine_align();
        assert!(!imu_fa.coarse_align_active());
    }

    #[test]
    fn rcs_jets_off_on_write_channel() {
        let mut hw = SimHardware::new_headless();
        hw.rcs().write_channel5(0xABCD);
        assert_eq!(hw.rcs.command.pitch_yaw, 0xABCD);
        hw.rcs().all_jets_off();
        assert_eq!(hw.rcs.command.pitch_yaw, 0);
    }
}
