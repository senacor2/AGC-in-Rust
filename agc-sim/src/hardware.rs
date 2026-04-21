//! Simulated AgcHardware implementation for host testing.

use std::time::Instant;

use agc_core::hal::{
    AgcHardware, Dsky, Engine, Imu, Optics, Rcs, Telemetry, Timers, Uplink,
    dsky::Lamp,
};
use agc_core::types::CduAngle;

// ── Sub-system stubs ──────────────────────────────────────────────────────────

/// Simulated mission timer.  Tracks a `base_cs` value and an `epoch`
/// instant; `mission_time()` returns `base_cs + elapsed_since_epoch`.
/// Calling `set_time()` rebases the clock so crew clock-sets (V25 N36 /
/// N65) are respected and the timer keeps advancing from the new value.
pub struct SimTimers {
    base_cs: u32,
    epoch: Instant,
}

impl SimTimers {
    pub fn new() -> Self {
        Self { base_cs: 0, epoch: Instant::now() }
    }

    /// Set the mission clock to an absolute value.  The timer continues
    /// to advance from this new base at wall-clock rate.
    pub fn set_time(&mut self, cs: u32) {
        self.base_cs = cs;
        self.epoch = Instant::now();
    }
}
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
    fn mission_time(&self) -> u32 {
        let elapsed = (self.epoch.elapsed().as_millis() / 10) as u32;
        self.base_cs.wrapping_add(elapsed)
    }
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
            timers:    SimTimers::new(),
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

#[cfg(test)]
mod tests {
    use super::*;
    use agc_core::hal::{Dsky, Engine, Imu, Optics, Rcs, Telemetry, Timers, Uplink};
    use agc_core::hal::dsky::Lamp;
    use agc_core::types::CduAngle;

    // ── Timers (TC-TIMERS-01 through TC-TIMERS-03) ──────────────────────────

    #[test]
    fn tc_timers_01_arm_t3_no_panic() {
        let mut hw = SimHardware::new();
        hw.timers().arm_t3(100);
        // mission_time starts near 0 (within a few ms of construction).
        assert!(hw.timers().mission_time() < 10);
    }

    #[test]
    fn tc_timers_02_mission_time_set() {
        let mut hw = SimHardware::new();
        hw.timers.set_time(54321);
        // Should read back ≈ 54321 (plus a few ms elapsed).
        let t = hw.timers().mission_time();
        assert!(t >= 54321 && t < 54321 + 10, "expected ~54321, got {t}");
    }

    #[test]
    fn tc_timers_03_disarm_t6_idempotent() {
        let mut hw = SimHardware::new();
        hw.timers().disarm_t6();
        hw.timers().disarm_t6();
    }

    // ── DSKY (TC-DSKY-01 through TC-DSKY-03) ────────────────────────────────

    #[test]
    fn tc_dsky_01_read_key_empty() {
        let mut hw = SimHardware::new();
        assert_eq!(hw.dsky().read_key(), None);
    }

    #[test]
    fn tc_dsky_02_read_key_fifo() {
        let mut hw = SimHardware::new();
        hw.dsky.keys.push_back(25); // ENTER key code
        hw.dsky.keys.push_back(31); // VERB key code
        assert_eq!(hw.dsky().read_key(), Some(25));
        assert_eq!(hw.dsky().read_key(), Some(31));
        assert_eq!(hw.dsky().read_key(), None);
    }

    #[test]
    fn tc_dsky_03_write_row_no_panic() {
        let mut hw = SimHardware::new();
        hw.dsky().write_row(1, 0x7FF);
        hw.dsky().clear_row(1);
        hw.dsky().set_lamp(Lamp::ProgAlarm, true);
        hw.dsky().set_flash(true);
    }

    // ── IMU (TC-IMU-01 through TC-IMU-03) ───────────────────────────────────

    #[test]
    fn tc_imu_01_read_pipa_destructive() {
        let mut hw = SimHardware::new();
        hw.imu.pipa = [100, -50, 25];
        let counts = hw.imu().read_pipa();
        assert_eq!(counts, [100, -50, 25]);
        assert_eq!(hw.imu().read_pipa(), [0, 0, 0]); // cleared
    }

    #[test]
    fn tc_imu_02_read_cdu_non_destructive() {
        let mut hw = SimHardware::new();
        hw.imu.cdu = [CduAngle(8192), CduAngle(0), CduAngle(16384)];
        let first = hw.imu().read_cdu();
        let second = hw.imu().read_cdu();
        assert_eq!(first, second);
        assert_eq!(first[0].0, 8192);
    }

    #[test]
    fn tc_imu_03_torque_gyro_no_side_effects() {
        let mut hw = SimHardware::new();
        hw.imu.cdu = [CduAngle(1000), CduAngle(2000), CduAngle(3000)];
        hw.imu().torque_gyro(0, 512);
        hw.imu().torque_gyro(1, -256);
        hw.imu().torque_gyro(2, 1);
        assert_eq!(hw.imu().read_cdu()[0].0, 1000);
    }

    // ── Optics (TC-OPTICS-01 through TC-OPTICS-03) ──────────────────────────

    #[test]
    fn tc_optics_01_initial_angles() {
        let mut hw = SimHardware::new();
        assert_eq!(hw.optics().trunnion_angle().0, 0);
        assert_eq!(hw.optics().shaft_angle().0, 0);
    }

    #[test]
    fn tc_optics_02_injected_angles() {
        let mut hw = SimHardware::new();
        hw.optics.trunnion = CduAngle(4096);
        hw.optics.shaft = CduAngle(32768);
        assert_eq!(hw.optics().trunnion_angle().0, 4096);
        assert_eq!(hw.optics().shaft_angle().0, 32768);
    }

    #[test]
    fn tc_optics_03_drive_no_panic() {
        let mut hw = SimHardware::new();
        hw.optics().drive(100, -200);
        assert!(!hw.optics().mark_pressed());
    }

    // ── Engine (TC-ENGINE-01 through TC-ENGINE-03) ──────────────────────────

    #[test]
    fn tc_engine_01_toggle_thrust() {
        let mut hw = SimHardware::new();
        assert!(!hw.engine().thrust_on());
        hw.engine().sps_enable(true);
        assert!(hw.engine().thrust_on());
        hw.engine().sps_enable(false);
        assert!(!hw.engine().thrust_on());
    }

    #[test]
    fn tc_engine_02_gimbal_no_thrust_change() {
        let mut hw = SimHardware::new();
        hw.engine().sps_enable(true);
        hw.engine().sps_gimbal(100, -50);
        assert!(hw.engine().thrust_on());
    }

    #[test]
    fn tc_engine_03_initial_state() {
        let hw = SimHardware::new();
        assert!(!hw.engine.thrusting);
    }

    // ── RCS (TC-RCS-01 through TC-RCS-03) ───────────────────────────────────

    #[test]
    fn tc_rcs_01_quench_idempotent() {
        let mut hw = SimHardware::new();
        hw.rcs().quench_all();
        hw.rcs().quench_all();
    }

    #[test]
    fn tc_rcs_02_fire_sm_jets() {
        let mut hw = SimHardware::new();
        hw.rcs().fire_sm_jets(0b1010_0101, 0b0101_1010);
        hw.rcs().fire_sm_jets(0x00, 0x00);
    }

    #[test]
    fn tc_rcs_03_fire_cm_jets() {
        let mut hw = SimHardware::new();
        hw.rcs().fire_cm_jets(0b0000_1111_1111);
        hw.rcs().quench_all();
    }

    // ── Uplink (TC-UPLINK-01 through TC-UPLINK-03) ──────────────────────────

    #[test]
    fn tc_uplink_01_empty() {
        let mut hw = SimHardware::new();
        assert_eq!(hw.uplink().read_word(), None);
    }

    #[test]
    fn tc_uplink_02_fifo() {
        let mut hw = SimHardware::new();
        hw.uplink.words.push_back(0x1234);
        hw.uplink.words.push_back(0x5678);
        assert_eq!(hw.uplink().read_word(), Some(0x1234));
        assert_eq!(hw.uplink().read_word(), Some(0x5678));
        assert_eq!(hw.uplink().read_word(), None);
    }

    #[test]
    fn tc_uplink_03_single_word() {
        let mut hw = SimHardware::new();
        hw.uplink.words.push_back(0xABCD);
        assert_eq!(hw.uplink().read_word(), Some(0xABCD));
    }

    // ── Telemetry (TC-TELEM-01 through TC-TELEM-03) ─────────────────────────

    #[test]
    fn tc_telem_01_send_word_logged() {
        let mut hw = SimHardware::new();
        hw.telemetry().send_word(0x1111);
        hw.telemetry().send_word(0x2222);
        assert_eq!(hw.telemetry.log, vec![0x1111, 0x2222]);
    }

    #[test]
    fn tc_telem_02_initial_log_empty() {
        let hw = SimHardware::new();
        assert!(hw.telemetry.log.is_empty());
    }

    #[test]
    fn tc_telem_03_send_multiple() {
        let mut hw = SimHardware::new();
        for i in 0..10 {
            hw.telemetry().send_word(i);
        }
        assert_eq!(hw.telemetry.log.len(), 10);
    }

    // ── AgcHardware (TC-HW-01 through TC-HW-02) ────────────────────────────

    #[test]
    fn tc_hw_01_pet_watchdog_noop() {
        let mut hw = SimHardware::new();
        hw.pet_watchdog(); // must not panic
    }

    #[test]
    #[should_panic(expected = "hardware_restart")]
    fn tc_hw_02_hardware_restart_panics() {
        let mut hw = SimHardware::new();
        hw.hardware_restart();
    }
}
