//! Simulated AgcHardware implementation for host testing.

use agc_core::hal::{
    AgcHardware, Dsky, Engine, Imu, Optics, Rcs, Telemetry, Timers, Uplink,
    dsky::Lamp,
};
use agc_core::types::CduAngle;

// ── Sub-system stubs ──────────────────────────────────────────────────────────

pub struct SimTimers { pub mission_time_cs: u32 }
pub struct SimDsky   { pub keys: std::collections::VecDeque<u8> }
pub struct SimImu    { pub pipa: [i16; 3], pub cdu: [CduAngle; 3] }
pub struct SimOptics { pub trunnion: CduAngle, pub shaft: CduAngle }
pub struct SimEngine { pub thrusting: bool }
pub struct SimRcs;
pub struct SimUplink  { pub words: std::collections::VecDeque<u16> }
pub struct SimTelemetry { pub log: Vec<u16> }

// ── Trait implementations ─────────────────────────────────────────────────────

impl Timers for SimTimers {
    fn arm_t3(&mut self, _cs: u16) {}
    fn arm_t5(&mut self, _cs: u16) {}
    fn arm_t6(&mut self, _counts: u16) {}
    fn disarm_t6(&mut self) {}
    fn mission_time(&self) -> u32 { self.mission_time_cs }
}

impl Dsky for SimDsky {
    fn write_row(&mut self, _row: u8, _data: u16) {}
    fn clear_row(&mut self, _row: u8) {}
    fn set_lamp(&mut self, _lamp: Lamp, _on: bool) {}
    fn set_flash(&mut self, _on: bool) {}
    fn read_key(&mut self) -> Option<u8> { self.keys.pop_front() }
}

impl Imu for SimImu {
    fn read_pipa(&mut self) -> [i16; 3] {
        let counts = self.pipa;
        self.pipa = [0; 3];
        counts
    }
    fn read_cdu(&self) -> [CduAngle; 3] { self.cdu }
    fn torque_gyro(&mut self, _axis: usize, _pulses: i16) {}
    fn coarse_align(&mut self, _commands: [i16; 3]) {}
    fn is_caged(&self) -> bool { false }
}

impl Optics for SimOptics {
    fn trunnion_angle(&self) -> CduAngle { self.trunnion }
    fn shaft_angle(&self) -> CduAngle { self.shaft }
    fn drive(&mut self, _trunnion: i16, _shaft: i16) {}
    fn mark_pressed(&self) -> bool { false }
}

impl Engine for SimEngine {
    fn sps_enable(&mut self, on: bool) { self.thrusting = on; }
    fn sps_gimbal(&mut self, _pitch: i16, _yaw: i16) {}
    fn thrust_on(&self) -> bool { self.thrusting }
}

impl Rcs for SimRcs {
    fn fire_sm_jets(&mut self, _a: u8, _b: u8) {}
    fn fire_cm_jets(&mut self, _jets: u16) {}
    fn quench_all(&mut self) {}
}

impl Uplink for SimUplink {
    fn read_word(&mut self) -> Option<u16> { self.words.pop_front() }
}

impl Telemetry for SimTelemetry {
    fn send_word(&mut self, word: u16) { self.log.push(word); }
}

// ── Top-level SimHardware ─────────────────────────────────────────────────────

pub struct SimHardware {
    pub timers:    SimTimers,
    pub dsky:      SimDsky,
    pub imu:       SimImu,
    pub optics:    SimOptics,
    pub engine:    SimEngine,
    pub rcs:       SimRcs,
    pub uplink:    SimUplink,
    pub telemetry: SimTelemetry,
}

impl SimHardware {
    pub fn new() -> Self {
        Self {
            timers:    SimTimers { mission_time_cs: 0 },
            dsky:      SimDsky   { keys: Default::default() },
            imu:       SimImu    { pipa: [0; 3], cdu: [CduAngle(0); 3] },
            optics:    SimOptics { trunnion: CduAngle(0), shaft: CduAngle(0) },
            engine:    SimEngine { thrusting: false },
            rcs:       SimRcs,
            uplink:    SimUplink  { words: Default::default() },
            telemetry: SimTelemetry { log: Vec::new() },
        }
    }
}

impl AgcHardware for SimHardware {
    type Timers    = SimTimers;
    type Dsky      = SimDsky;
    type Imu       = SimImu;
    type Optics    = SimOptics;
    type Engine    = SimEngine;
    type Rcs       = SimRcs;
    type Uplink    = SimUplink;
    type Telemetry = SimTelemetry;

    fn timers(&mut self)    -> &mut SimTimers    { &mut self.timers }
    fn dsky(&mut self)      -> &mut SimDsky      { &mut self.dsky }
    fn imu(&mut self)       -> &mut SimImu       { &mut self.imu }
    fn optics(&mut self)    -> &mut SimOptics    { &mut self.optics }
    fn engine(&mut self)    -> &mut SimEngine    { &mut self.engine }
    fn rcs(&mut self)       -> &mut SimRcs       { &mut self.rcs }
    fn uplink(&mut self)    -> &mut SimUplink    { &mut self.uplink }
    fn telemetry(&mut self) -> &mut SimTelemetry { &mut self.telemetry }

    fn pet_watchdog(&mut self) { /* no-op in simulation */ }

    fn hardware_restart(&mut self) -> ! {
        panic!("SimHardware: hardware_restart triggered")
    }
}
