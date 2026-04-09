//! Simulated hardware implementation of `AgcHardware`.
//!
//! `SimHardware` drives an optional `DskyTerminal` (TUI) and a `SimLog` that
//! records every state change. When the terminal is attached, every relay-word
//! write, light change, alarm, and input keystroke is reflected live in the UI.

use agc_core::hal::dsky::DskyKey;
use agc_core::hal::rcs::{CmJetMask, SmJetMask};
use agc_core::hal::{AgcHardware, Dsky, Engine, Imu, Optics, Rcs, Telemetry, Timers, Uplink};
use agc_core::types::CduAngle;

use crate::dsky_state::{DataRegister, DskyDisplayState, Sign};
use crate::sim_log::SimLog;

// ── Simulated Timers ─────────────────────────────────────────────────────────

pub struct SimTimers {
    t3_cs: u16,
    t5_cs: u16,
    t6_cs: Option<u16>,
}

impl SimTimers {
    const fn new() -> Self {
        Self {
            t3_cs: 0,
            t5_cs: 0,
            t6_cs: None,
        }
    }
}

impl Timers for SimTimers {
    fn arm_t3(&mut self, centiseconds: u16) {
        self.t3_cs = centiseconds;
    }
    fn read_t3(&self) -> u16 {
        self.t3_cs
    }
    fn arm_t5(&mut self, centiseconds: u16) {
        self.t5_cs = centiseconds;
    }
    fn arm_t6(&mut self, centiseconds: u16) {
        self.t6_cs = Some(centiseconds);
    }
    fn disarm_t6(&mut self) {
        self.t6_cs = None;
    }
}

// ── Simulated DSKY ───────────────────────────────────────────────────────────

pub struct SimDsky {
    /// Current decoded display state (updated on every relay-word write).
    pub display: DskyDisplayState,
    /// Pending keystrokes queued for the AGC to read.
    pending_keys: [Option<DskyKey>; 8],
    key_head: usize,
    key_tail: usize,
}

impl SimDsky {
    const fn new() -> Self {
        Self {
            display: DskyDisplayState {
                prog: [0xFF, 0xFF],
                verb: [0xFF, 0xFF],
                noun: [0xFF, 0xFF],
                r1: DataRegister::BLANK,
                r2: DataRegister::BLANK,
                r3: DataRegister::BLANK,
                lights: crate::dsky_state::Lights {
                    uplink_acty: false,
                    temp: false,
                    gimbal_lock: false,
                    prog_alarm: false,
                    key_rel: false,
                    opr_err: false,
                    comp_acty: false,
                    no_att: false,
                    stby: false,
                    restart: false,
                    tracker: false,
                    alt: false,
                    vel: false,
                },
            },
            pending_keys: [None; 8],
            key_head: 0,
            key_tail: 0,
        }
    }

    /// Enqueue a key press for the AGC to read.
    pub fn push_key(&mut self, key: DskyKey) {
        self.pending_keys[self.key_tail % 8] = Some(key);
        self.key_tail = (self.key_tail + 1) % 8;
    }

    /// Decode a raw relay word into display fields.
    ///
    /// The AGC channel 010 bit layout encodes which digit position is being
    /// updated. This simplified decoder updates specific fields based on the
    /// upper 4 bits as a field selector; a full implementation would follow
    /// the T4RUPT relay matrix scan sequence.
    fn decode_relay_word(&mut self, word: u16, log: &mut SimLog) {
        let field = (word >> 11) & 0x1F;
        let d1 = ((word >> 5) & 0x1F) as u8;
        let d2 = (word & 0x1F) as u8;

        // Clamp digits: AGC uses 0–9 in BCD; values > 9 mean blank.
        let digit = |v: u8| if v <= 9 { v } else { 0xFF };

        match field {
            0x0B => {
                self.display.prog = [digit(d1), digit(d2)];
                log.io(format!("DSKY PROG ← {}{}", d1, d2));
            }
            0x0A => {
                self.display.verb = [digit(d1), digit(d2)];
                log.io(format!("DSKY VERB ← {}{}", d1, d2));
            }
            0x09 => {
                self.display.noun = [digit(d1), digit(d2)];
                log.io(format!("DSKY NOUN ← {}{}", d1, d2));
            }
            0x01 => {
                // R1 sign + digits 1–2
                self.display.r1.sign = if d1 & 0x01 != 0 {
                    Sign::Plus
                } else {
                    Sign::Minus
                };
                self.display.r1.digits[0] = digit(d2);
                log.io(format!("DSKY R1 sign+d1 ← {:02X}", word));
            }
            0x02 => {
                self.display.r1.digits[1] = digit(d1);
                self.display.r1.digits[2] = digit(d2);
            }
            0x03 => {
                self.display.r1.digits[3] = digit(d1);
                self.display.r1.digits[4] = digit(d2);
            }
            0x04 => {
                self.display.r2.sign = if d1 & 0x01 != 0 {
                    Sign::Plus
                } else {
                    Sign::Minus
                };
                self.display.r2.digits[0] = digit(d2);
                log.io(format!("DSKY R2 sign+d1 ← {:02X}", word));
            }
            0x05 => {
                self.display.r2.digits[1] = digit(d1);
                self.display.r2.digits[2] = digit(d2);
            }
            0x06 => {
                self.display.r2.digits[3] = digit(d1);
                self.display.r2.digits[4] = digit(d2);
            }
            0x07 => {
                self.display.r3.sign = if d1 & 0x01 != 0 {
                    Sign::Plus
                } else {
                    Sign::Minus
                };
                self.display.r3.digits[0] = digit(d2);
                log.io(format!("DSKY R3 sign+d1 ← {:02X}", word));
            }
            0x08 => {
                self.display.r3.digits[1] = digit(d1);
                self.display.r3.digits[2] = digit(d2);
            }
            0x0C => {
                self.display.r3.digits[3] = digit(d1);
                self.display.r3.digits[4] = digit(d2);
            }
            _ => {}
        }
    }
}

impl Dsky for SimDsky {
    fn write_relay_word(&mut self, _word: u16) {
        // Relay word decoding requires access to the log; handled via
        // SimHardware::dsky_write_relay which has both.
    }

    fn set_comp_acty(&mut self, on: bool) {
        self.display.lights.comp_acty = on;
    }
    fn set_uplink_acty(&mut self, on: bool) {
        self.display.lights.uplink_acty = on;
    }
    fn set_temp_light(&mut self, on: bool) {
        self.display.lights.temp = on;
    }
    fn set_gimbal_lock(&mut self, on: bool) {
        self.display.lights.gimbal_lock = on;
    }
    fn set_prog_alarm(&mut self, on: bool) {
        self.display.lights.prog_alarm = on;
    }
    fn set_key_rel(&mut self, on: bool) {
        self.display.lights.key_rel = on;
    }
    fn set_opr_err(&mut self, on: bool) {
        self.display.lights.opr_err = on;
    }

    fn read_key(&mut self) -> Option<DskyKey> {
        if self.key_head == self.key_tail {
            return None;
        }
        let key = self.pending_keys[self.key_head % 8];
        self.key_head = (self.key_head + 1) % 8;
        key
    }
}

// ── Simulated IMU ─────────────────────────────────────────────────────────────

pub struct SimImu {
    pub pipa_counts: [i16; 3],
    pub cdu_angles: [CduAngle; 3],
    pub temperature: f32,
}

impl SimImu {
    const fn new() -> Self {
        Self {
            pipa_counts: [0; 3],
            cdu_angles: [CduAngle::ZERO; 3],
            temperature: 70.0,
        }
    }
}

impl Imu for SimImu {
    fn read_pipa(&mut self) -> [i16; 3] {
        let counts = self.pipa_counts;
        self.pipa_counts = [0; 3];
        counts
    }
    fn read_cdu(&self) -> [CduAngle; 3] {
        self.cdu_angles
    }
    fn torque_gyro(&mut self, axis: usize, pulses: i16) {
        if axis < 3 {
            let cur = self.cdu_angles[axis].0 as i32;
            self.cdu_angles[axis] = CduAngle((cur + pulses as i32).rem_euclid(32768) as u16);
        }
    }
    fn set_caged(&mut self, _caged: bool) {}
    fn read_temperature(&self) -> f32 {
        self.temperature
    }
}

// ── Simulated Optics ──────────────────────────────────────────────────────────

pub struct SimOptics {
    shaft: CduAngle,
    trunnion: CduAngle,
    mark_pending: bool,
}

impl SimOptics {
    const fn new() -> Self {
        Self {
            shaft: CduAngle::ZERO,
            trunnion: CduAngle::ZERO,
            mark_pending: false,
        }
    }
}

impl Optics for SimOptics {
    fn read_shaft(&self) -> CduAngle {
        self.shaft
    }
    fn read_trunnion(&self) -> CduAngle {
        self.trunnion
    }
    fn drive_shaft(&mut self, target: CduAngle) {
        self.shaft = target;
    }
    fn drive_trunnion(&mut self, target: CduAngle) {
        self.trunnion = target;
    }
    fn set_zero_optics(&mut self, _enabled: bool) {}
    fn mark_pending(&self) -> bool {
        self.mark_pending
    }
    fn clear_mark(&mut self) {
        self.mark_pending = false;
    }
}

// ── Simulated Engine ──────────────────────────────────────────────────────────

pub struct SimEngine {
    pub armed: bool,
    pub firing: bool,
    pub gimbal: [CduAngle; 2],
}

impl SimEngine {
    const fn new() -> Self {
        Self {
            armed: false,
            firing: false,
            gimbal: [CduAngle::ZERO; 2],
        }
    }
}

impl Engine for SimEngine {
    fn set_engine_arm(&mut self, armed: bool) {
        self.armed = armed;
    }
    fn ignite(&mut self) {
        if self.armed {
            self.firing = true;
        }
    }
    fn cutoff(&mut self) {
        self.firing = false;
    }
    fn command_gimbal(&mut self, pitch: i16, yaw: i16) {
        self.gimbal[0] = CduAngle(pitch as u16);
        self.gimbal[1] = CduAngle(yaw as u16);
    }
    fn read_gimbal(&self) -> [CduAngle; 2] {
        self.gimbal
    }
    fn is_firing(&self) -> bool {
        self.firing
    }
}

// ── Simulated RCS ─────────────────────────────────────────────────────────────

pub struct SimRcs {
    pub sm_firing: SmJetMask,
    pub cm_firing: CmJetMask,
}

impl SimRcs {
    const fn new() -> Self {
        Self {
            sm_firing: 0,
            cm_firing: 0,
        }
    }
}

impl Rcs for SimRcs {
    fn fire_sm_jets(&mut self, mask: SmJetMask, _duration_ms: u16) {
        self.sm_firing |= mask;
    }
    fn fire_cm_jets(&mut self, mask: CmJetMask, _duration_ms: u16) {
        self.cm_firing |= mask;
    }
    fn all_jets_off(&mut self) {
        self.sm_firing = 0;
        self.cm_firing = 0;
    }
    fn sm_jets_firing(&self) -> SmJetMask {
        self.sm_firing
    }
    fn cm_jets_firing(&self) -> CmJetMask {
        self.cm_firing
    }
}

// ── Simulated Uplink ──────────────────────────────────────────────────────────

pub struct SimUplink {
    buffer: [Option<u16>; 8],
    head: usize,
    tail: usize,
}

impl SimUplink {
    const fn new() -> Self {
        Self {
            buffer: [None; 8],
            head: 0,
            tail: 0,
        }
    }

    pub fn push_word(&mut self, word: u16) {
        self.buffer[self.tail % 8] = Some(word);
        self.tail = (self.tail + 1) % 8;
    }
}

impl Uplink for SimUplink {
    fn word_available(&self) -> bool {
        self.head != self.tail
    }
    fn read_word(&mut self) -> Option<u16> {
        if self.head == self.tail {
            return None;
        }
        let w = self.buffer[self.head % 8];
        self.head = (self.head + 1) % 8;
        w
    }
    fn buffered_count(&self) -> u8 {
        ((self.tail + 8 - self.head) % 8) as u8
    }
}

// ── Simulated Telemetry ───────────────────────────────────────────────────────

pub struct SimTelemetry {
    pub words: Vec<u16>,
}

impl SimTelemetry {
    fn new() -> Self {
        Self { words: Vec::new() }
    }
}

impl Telemetry for SimTelemetry {
    fn ready(&self) -> bool {
        true
    }
    fn write_word(&mut self, word: u16) {
        self.words.push(word);
    }
}

// ── SimHardware (composite) ───────────────────────────────────────────────────

/// Simulated AGC hardware for host-side testing and interactive simulation.
///
/// Attach a `DskyTerminal` via `SimHardware::new_with_terminal()` to get a
/// live TUI. Use `SimHardware::new()` for headless unit tests.
pub struct SimHardware {
    pub timers: SimTimers,
    pub dsky: SimDsky,
    pub imu: SimImu,
    pub optics: SimOptics,
    pub engine: SimEngine,
    pub rcs: SimRcs,
    pub uplink: SimUplink,
    pub telemetry: SimTelemetry,
    /// Event log — append to this to record state changes.
    pub log: SimLog,
    pub watchdog_pets: u32,
}

impl SimHardware {
    pub fn new() -> Self {
        let mut hw = Self {
            timers: SimTimers::new(),
            dsky: SimDsky::new(),
            imu: SimImu::new(),
            optics: SimOptics::new(),
            engine: SimEngine::new(),
            rcs: SimRcs::new(),
            uplink: SimUplink::new(),
            telemetry: SimTelemetry::new(),
            log: SimLog::new(),
            watchdog_pets: 0,
        };
        hw.log.info("SimHardware initialised");
        hw
    }

    /// Write a relay word to the DSKY and decode it into the display state.
    pub fn dsky_write_relay(&mut self, word: u16) {
        let log = &mut self.log;
        self.dsky.decode_relay_word(word, log);
    }

    /// Push a key into the DSKY queue and log it.
    pub fn dsky_push_key(&mut self, key: DskyKey) {
        self.log.io(format!("KEY → {:?}", key));
        self.dsky.push_key(key);
    }

    /// Inject PIPA counts (simulated accelerometer pulse) and log it.
    pub fn inject_pipa(&mut self, x: i16, y: i16, z: i16) {
        self.imu.pipa_counts[0] += x;
        self.imu.pipa_counts[1] += y;
        self.imu.pipa_counts[2] += z;
        if x != 0 || y != 0 || z != 0 {
            self.log.io(format!("PIPA ({:+}, {:+}, {:+})", x, y, z));
        }
    }

    /// Log an alarm code being raised.
    pub fn log_alarm(&mut self, code: u16) {
        self.log.alarm(format!("ALARM {:04o}", code));
        self.dsky.display.lights.prog_alarm = true;
    }

    /// Snapshot of the current DSKY display state (for the TUI renderer).
    pub fn display_snapshot(&self) -> &DskyDisplayState {
        &self.dsky.display
    }
}

impl Default for SimHardware {
    fn default() -> Self {
        Self::new()
    }
}

impl AgcHardware for SimHardware {
    type Timers = SimTimers;
    type Dsky = SimDsky;
    type Imu = SimImu;
    type Optics = SimOptics;
    type Engine = SimEngine;
    type Rcs = SimRcs;
    type Uplink = SimUplink;
    type Telemetry = SimTelemetry;

    fn timers(&mut self) -> &mut SimTimers {
        &mut self.timers
    }
    fn dsky(&mut self) -> &mut SimDsky {
        &mut self.dsky
    }
    fn imu(&mut self) -> &mut SimImu {
        &mut self.imu
    }
    fn optics(&mut self) -> &mut SimOptics {
        &mut self.optics
    }
    fn engine(&mut self) -> &mut SimEngine {
        &mut self.engine
    }
    fn rcs(&mut self) -> &mut SimRcs {
        &mut self.rcs
    }
    fn uplink(&mut self) -> &mut SimUplink {
        &mut self.uplink
    }
    fn telemetry(&mut self) -> &mut SimTelemetry {
        &mut self.telemetry
    }

    fn pet_watchdog(&mut self) {
        self.watchdog_pets += 1;
    }

    fn hardware_restart(&mut self) -> ! {
        self.log.alarm("HARDWARE RESTART");
        panic!("SimHardware: hardware restart triggered")
    }
}
