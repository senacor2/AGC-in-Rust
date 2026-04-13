//! Minimal mock hardware for unit tests inside `agc-core`.
//!
//! Used by program unit tests that need an `AgcHardware` implementation but
//! cannot depend on `agc-sim` (circular crate dependency).
//! All methods are no-ops or store state in fixed-size fields.

use crate::hal::{
    dsky::{DigitRow, DskyIo, Key, RelayWord},
    engine::EngineIo,
    imu::ImuIo,
    optics::OpticsIo,
    rcs::{JetCommand, RcsIo},
    telemetry::TelemetryIo,
    timers::Timers,
    uplink::UplinkIo,
    AgcHardware,
};
use crate::types::{CduAngle, Mat3x3, IDENTITY_MAT3};

// ── MockDsky ──────────────────────────────────────────────────────────────────

pub struct MockDsky {
    pub prog: Option<u8>,
    pub verb: Option<u8>,
    pub noun: Option<u8>,
    pub r1: Option<i32>,
    pub r2: Option<i32>,
    pub r3: Option<i32>,
    pub proceed: bool,
}

impl MockDsky {
    pub fn new() -> Self {
        Self {
            prog: None,
            verb: None,
            noun: None,
            r1: None,
            r2: None,
            r3: None,
            proceed: false,
        }
    }

    pub fn set_proceed(&mut self, v: bool) {
        self.proceed = v;
    }
}

impl DskyIo for MockDsky {
    fn read_key(&mut self) -> Option<Key> {
        None
    }
    fn read_nav_key(&mut self) -> Option<Key> {
        None
    }
    fn write_relay(&mut self, _w: RelayWord) {}
    fn write_lamp_word(&mut self, _b: u16) {}
    fn write_prog(&mut self, p: u8) {
        self.prog = Some(p);
    }
    fn write_verb(&mut self, v: u8) {
        self.verb = Some(v);
    }
    fn write_noun(&mut self, n: u8) {
        self.noun = Some(n);
    }
    fn write_register(&mut self, row: usize, value: &DigitRow) {
        // Skip leading blank (0xFF) digits; accumulate the rest.
        let mut mag: i32 = 0;
        let mut any_digit = false;
        for &d in &value.digits {
            if d == 0xFF {
                continue;
            } // leading blank — skip
            any_digit = true;
            mag = mag * 10 + d as i32;
        }
        let v = if any_digit {
            Some(if value.sign_minus { -mag } else { mag })
        } else {
            None
        };
        match row {
            0 => self.r1 = v,
            1 => self.r2 = v,
            2 => self.r3 = v,
            _ => {}
        }
    }
    fn proceed_pressed(&self) -> bool {
        self.proceed
    }
    fn set_prog_light(&mut self, _on: bool) {}
    fn blank_display(&mut self) {
        self.prog = None;
        self.verb = None;
        self.noun = None;
    }
}

// ── MockImu ───────────────────────────────────────────────────────────────────

pub struct MockImu {
    pub cdus: [CduAngle; 3],
    pub pipas: [i16; 3],
    pub status: u16,
    pub coarse_align_bit: bool,
    pub torque_calls: u32,
}

impl MockImu {
    pub fn new() -> Self {
        Self {
            cdus: [CduAngle(0); 3],
            pipas: [0; 3],
            status: 0,
            coarse_align_bit: false,
            torque_calls: 0,
        }
    }

    pub fn inject_status(&mut self, s: u16) {
        self.status = s;
    }
}

impl ImuIo for MockImu {
    fn read_cdu(&self) -> [CduAngle; 3] {
        self.cdus
    }
    fn read_pipa(&mut self) -> [i16; 3] {
        let v = self.pipas;
        self.pipas = [0; 3];
        v
    }
    fn torque_gyro(&mut self, _axis: usize, _pulses: i16) {
        self.torque_calls += 1;
    }
    fn read_status(&self) -> u16 {
        self.status
    }
    fn write_control(&mut self, _bits: u16) {}
    fn write_cdu_commands(&mut self, _cmds: [i16; 3]) {}
    fn coarse_align_active(&self) -> bool {
        self.coarse_align_bit
    }
    fn set_coarse_align(&mut self) {
        self.coarse_align_bit = true;
    }
    fn refsmmat(&self) -> Mat3x3 {
        IDENTITY_MAT3
    }
}

// MockImu implements ImuIo directly; no typestate wrapper needed for tests.

// ── MockEngine ────────────────────────────────────────────────────────────────

pub struct MockEngine {
    pub enabled: bool,
}

impl MockEngine {
    pub fn new() -> Self {
        Self { enabled: false }
    }
}

impl EngineIo for MockEngine {
    fn set_engine_enable(&mut self, e: bool) {
        self.enabled = e;
    }
    fn engine_enabled(&self) -> bool {
        self.enabled
    }
    fn trim_pitch(&mut self, _d: i16) {}
    fn trim_yaw(&mut self, _d: i16) {}
    fn read_tvc_pitch(&self) -> CduAngle {
        CduAngle(0)
    }
    fn read_tvc_yaw(&self) -> CduAngle {
        CduAngle(0)
    }
    fn write_dsalmout(&mut self, _b: u16) {}
}

// ── MockRcs ───────────────────────────────────────────────────────────────────

pub struct MockRcs {
    pub command: JetCommand,
}

impl MockRcs {
    pub fn new() -> Self {
        Self {
            command: JetCommand::OFF,
        }
    }
}

impl RcsIo for MockRcs {
    fn fire_jets(&mut self, cmd: JetCommand) {
        self.command = cmd;
    }
    fn all_jets_off(&mut self) {
        self.command = JetCommand::OFF;
    }
    fn current_command(&self) -> JetCommand {
        self.command
    }
    fn write_channel5(&mut self, w: u16) {
        self.command.pitch_yaw = w;
    }
    fn write_channel6(&mut self, w: u16) {
        self.command.roll = w;
    }
}

// ── MockTimers ────────────────────────────────────────────────────────────────

pub struct MockTimers {
    pub t3: u16,
}
impl MockTimers {
    pub fn new() -> Self {
        Self { t3: 0xFFFF }
    }
}
impl Timers for MockTimers {
    fn set_t3(&mut self, v: u16) {
        self.t3 = v;
    }
    fn set_t4(&mut self, _v: u16) {}
    fn set_t5(&mut self, _v: u16) {}
    fn set_t6(&mut self, _v: u16) {}
    fn read_t3(&self) -> u16 {
        self.t3
    }
    fn read_t4(&self) -> u16 {
        0
    }
    fn read_t5(&self) -> u16 {
        0
    }
    fn read_t6(&self) -> u16 {
        0
    }
    fn disable_t3(&mut self) {}
    fn enable_t3(&mut self) {}
}

// ── MockOptics ────────────────────────────────────────────────────────────────

pub struct MockOptics;
impl OpticsIo for MockOptics {
    fn read_shaft(&self) -> CduAngle {
        CduAngle(0)
    }
    fn read_trunnion(&self) -> CduAngle {
        CduAngle(0)
    }
    fn drive_shaft(&mut self, _d: i16) {}
    fn drive_trunnion(&mut self, _d: i16) {}
    fn write_chan14(&mut self, _b: u16) {}
}

// ── MockUplink ────────────────────────────────────────────────────────────────

pub struct MockUplink;
impl UplinkIo for MockUplink {
    fn read_uplink_word(&mut self) -> Option<u8> {
        None
    }
    fn uplink_overrun(&self) -> bool {
        false
    }
    fn clear_overrun(&mut self) {}
}

// ── MockTelemetry ─────────────────────────────────────────────────────────────

pub struct MockTelemetry;
impl TelemetryIo for MockTelemetry {
    fn write_downlink_pair(&mut self, _w1: u16, _w2: u16) {}
    fn downlink_overrun(&self) -> bool {
        false
    }
    fn clear_overrun(&mut self) {}
    fn write_chan13(&mut self, _b: u16) {}
}

// ── MockHardware ──────────────────────────────────────────────────────────────

/// Complete mock hardware for unit tests in `agc-core`.
pub struct MockHardware {
    pub dsky: MockDsky,
    pub imu: MockImu,
    pub engine: MockEngine,
    pub rcs: MockRcs,
    pub timers: MockTimers,
    pub optics: MockOptics,
    pub uplink: MockUplink,
    pub telemetry: MockTelemetry,
}

impl MockHardware {
    pub fn new() -> Self {
        Self {
            dsky: MockDsky::new(),
            imu: MockImu::new(),
            engine: MockEngine::new(),
            rcs: MockRcs::new(),
            timers: MockTimers::new(),
            optics: MockOptics,
            uplink: MockUplink,
            telemetry: MockTelemetry,
        }
    }
}

impl AgcHardware for MockHardware {
    type Timers = MockTimers;
    type Dsky = MockDsky;
    type Imu = MockImu;
    type Optics = MockOptics;
    type Engine = MockEngine;
    type Rcs = MockRcs;
    type Uplink = MockUplink;
    type Telemetry = MockTelemetry;

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
        panic!("MockHardware::hardware_restart")
    }
}
