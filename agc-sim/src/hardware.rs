//! Simulated AgcHardware implementation for host testing.

use std::time::Instant;

use agc_core::hal::{
    dsky::Lamp, AgcHardware, Dsky, Engine, Imu, Optics, Rcs, Secs, Telemetry, Timers,
};
use agc_core::types::CduAngle;

use crate::physics::Spacecraft;
use crate::uplink::ScriptedUplink;

// ── Sub-system stubs ──────────────────────────────────────────────────────────

/// Simulated mission timer.  Tracks a `base_cs` value and an `epoch`
/// instant; `mission_time()` returns `base_cs + elapsed_since_epoch`.
/// Calling `set_time()` rebases the clock so crew clock-sets (V25 N36 /
/// N65) are respected and the timer keeps advancing from the new value.
pub struct SimTimers {
    base_cs: u32,
    epoch: Instant,
}

impl Default for SimTimers {
    fn default() -> Self {
        Self::new()
    }
}

impl SimTimers {
    pub fn new() -> Self {
        Self {
            base_cs: 0,
            epoch: Instant::now(),
        }
    }

    /// Set the mission clock to an absolute value.  The timer continues
    /// to advance from this new base at wall-clock rate.
    pub fn set_time(&mut self, cs: u32) {
        self.base_cs = cs;
        self.epoch = Instant::now();
    }
}
pub struct SimDsky {
    pub keys: std::collections::VecDeque<u8>,
    /// Recorded `set_lamp(lamp, on)` invocations in dispatch order.
    ///
    /// Populated by [`Dsky::set_lamp`] whenever
    /// `services::pinball::emit_dsky_to_hw` walks the lamp table. The
    /// sim's interactive UI does not depend on these events — it reads
    /// the lamp truth via `decode_dsky(&state.dsky)` — so the recorder
    /// exists purely as a debug-log / test surface (#138).
    ///
    /// Tests drain the queue with [`SimDsky::drain_lamp_events`] after
    /// running enough sim ticks (and a manual `emit_dsky_to_hw`) to
    /// drive the expected lamp transitions.
    pub lamp_events: std::collections::VecDeque<(Lamp, bool)>,
}

impl SimDsky {
    /// Take and clear every recorded `set_lamp` event since the last
    /// drain. Events are returned in dispatch order
    /// (oldest → newest).
    pub fn drain_lamp_events(&mut self) -> Vec<(Lamp, bool)> {
        self.lamp_events.drain(..).collect()
    }
}
pub struct SimImu {
    pub pipa: [i16; 3],
    pub cdu: [CduAngle; 3],
}
pub struct SimOptics {
    pub trunnion: CduAngle,
    pub shaft: CduAngle,
    /// MARK button latched state. Set to `true` by the scenario runner
    /// when a CDU-driven sighting fires; the sextant-interrupt handler
    /// in `agc_core::control::sextant` resets it via `clear_mark` after
    /// consuming the press, so one keystroke maps to one dispatch (#57).
    pub mark_pressed: bool,
}

impl SimOptics {
    /// Latch the optics CDU at the given shaft/trunnion angles and assert
    /// the MARK edge. Mirrors the hardware behaviour of the crew pressing
    /// MARK while the optics are pointed at a star or landmark.
    pub fn press_mark(&mut self, shaft: CduAngle, trunnion: CduAngle) {
        self.shaft = shaft;
        self.trunnion = trunnion;
        self.mark_pressed = true;
    }
}
pub struct SimEngine {
    pub thrusting: bool,
    pub gimbal_pitch: i16,
    pub gimbal_yaw: i16,
}
pub struct SimRcs {
    /// Current hardware jet state (cleared by quench_all).
    pub sm_jets: u16,
    pub cm_jets: u16,
    /// Sticky visual accumulator — ORs in every firing between render frames.
    pub visual_sm_jets: u16,
    pub visual_cm_jets: u16,
}
/// Simulated MSFN telemetry downlink sink.
///
/// Words are always appended to `log` for test inspection.  When
/// `file` is `Some`, each word is also written as two big-endian bytes
/// to a timestamped binary file — one word per two-byte write, 50 pairs
/// per second at the DOWNRUPT cadence.
///
/// Open a file sink with [`SimTelemetry::with_file`].
pub struct SimTelemetry {
    pub log: Vec<u16>,
    file: Option<std::io::BufWriter<std::fs::File>>,
}

impl SimTelemetry {
    /// Create a memory-only sink (default for tests).
    pub fn new() -> Self {
        Self { log: Vec::new(), file: None }
    }

    /// Create a sink that additionally streams downlink words to a
    /// timestamped binary file `downlink_<unix_secs>.bin` in `dir`.
    ///
    /// Each 15-bit AGC word is written as 2 bytes big-endian.
    /// The file is flushed automatically when `SimTelemetry` is dropped.
    ///
    /// # Errors
    /// Returns `Err` if the file cannot be created.
    pub fn with_file(dir: &std::path::Path) -> std::io::Result<Self> {
        let ts = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();
        let path = dir.join(format!("downlink_{ts}.bin"));
        let f = std::fs::File::create(path)?;
        Ok(Self { log: Vec::new(), file: Some(std::io::BufWriter::new(f)) })
    }
}

impl Default for SimTelemetry {
    fn default() -> Self { Self::new() }
}
/// Simulated Sequential Events Control System pyro driver.
///
/// Each pyro is modelled as a latched `*_fired` flag plus a call counter.
/// The pyros are idempotent at the hardware level — counting just lets
/// tests verify the AGC didn't keep re-issuing the command on subsequent
/// cycles.
#[derive(Default)]
pub struct SimSecs {
    pub drogue_fired: bool,
    pub drogue_fire_count: u32,
    pub csm_separation_fired: bool,
    pub csm_separation_fire_count: u32,
}

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
    fn set_lamp(&mut self, lamp: Lamp, on: bool) {
        self.lamp_events.push_back((lamp, on));
    }
    fn set_flash(&mut self, _on: bool) {}
    fn read_key(&mut self) -> Option<u8> {
        self.keys.pop_front()
    }
}

impl Imu for SimImu {
    fn read_pipa(&mut self) -> [i16; 3] {
        let counts = self.pipa;
        self.pipa = [0; 3];
        counts
    }
    fn read_cdu(&self) -> [CduAngle; 3] {
        self.cdu
    }
    fn torque_gyro(&mut self, _axis: usize, _pulses: i16) {}
    fn coarse_align(&mut self, _commands: [i16; 3]) {}
    fn is_caged(&self) -> bool {
        false
    }
}

impl Optics for SimOptics {
    fn trunnion_angle(&self) -> CduAngle {
        self.trunnion
    }
    fn shaft_angle(&self) -> CduAngle {
        self.shaft
    }
    fn drive(&mut self, _trunnion: i16, _shaft: i16) {}
    fn mark_pressed(&self) -> bool {
        self.mark_pressed
    }
    fn clear_mark(&mut self) {
        self.mark_pressed = false;
    }
}

impl Engine for SimEngine {
    fn sps_enable(&mut self, on: bool) {
        self.thrusting = on;
    }
    fn sps_gimbal(&mut self, pitch: i16, yaw: i16) {
        self.gimbal_pitch = pitch;
        self.gimbal_yaw = yaw;
    }
    fn thrust_on(&self) -> bool {
        self.thrusting
    }
}

impl Rcs for SimRcs {
    fn fire_sm_jets(&mut self, a: u8, b: u8) {
        self.sm_jets = (b as u16) << 8 | (a as u16);
        self.visual_sm_jets |= self.sm_jets;
    }
    fn fire_cm_jets(&mut self, jets: u16) {
        self.cm_jets = jets & 0x0FFF;
        self.visual_cm_jets |= self.cm_jets;
    }
    fn quench_all(&mut self) {
        self.sm_jets = 0;
        self.cm_jets = 0;
    }
}

impl SimRcs {
    /// Read and clear the visual jet accumulators. Call once per render frame.
    /// Returns `(sm_jets, cm_jets)` representing all jets that fired since the
    /// last drain, then resets the accumulators to the current hardware state.
    pub fn drain_visual(&mut self) -> (u16, u16) {
        let sm = self.visual_sm_jets;
        let cm = self.visual_cm_jets;
        self.visual_sm_jets = self.sm_jets;
        self.visual_cm_jets = self.cm_jets;
        (sm, cm)
    }
}

impl Telemetry for SimTelemetry {
    fn send_word(&mut self, word: u16) {
        self.log.push(word);
        if let Some(ref mut f) = self.file {
            use std::io::Write as _;
            let _ = f.write_all(&word.to_be_bytes());
        }
    }
}

impl Secs for SimSecs {
    fn deploy_drogue(&mut self) {
        self.drogue_fired = true;
        self.drogue_fire_count = self.drogue_fire_count.saturating_add(1);
    }
    fn fire_csm_separation(&mut self) {
        self.csm_separation_fired = true;
        self.csm_separation_fire_count = self.csm_separation_fire_count.saturating_add(1);
    }
}

// ── Top-level SimHardware ─────────────────────────────────────────────────────

pub struct SimHardware {
    pub timers: SimTimers,
    pub dsky: SimDsky,
    pub imu: SimImu,
    pub optics: SimOptics,
    pub engine: SimEngine,
    pub rcs: SimRcs,
    pub secs: SimSecs,
    pub uplink: ScriptedUplink,
    pub telemetry: SimTelemetry,
    /// Ground-truth spacecraft dynamics. Drives the IMU's PIPA pulse
    /// stream when the SPS is commanded on.
    pub spacecraft: Spacecraft,
}

impl Default for SimHardware {
    fn default() -> Self {
        Self::new()
    }
}

impl SimHardware {
    pub fn new() -> Self {
        Self {
            timers: SimTimers::new(),
            dsky: SimDsky {
                keys: Default::default(),
                lamp_events: Default::default(),
            },
            imu: SimImu {
                pipa: [0; 3],
                cdu: [CduAngle(0); 3],
            },
            optics: SimOptics {
                trunnion: CduAngle(0),
                shaft: CduAngle(0),
                mark_pressed: false,
            },
            engine: SimEngine {
                thrusting: false,
                gimbal_pitch: 0,
                gimbal_yaw: 0,
            },
            rcs: SimRcs {
                sm_jets: 0,
                cm_jets: 0,
                visual_sm_jets: 0,
                visual_cm_jets: 0,
            },
            secs: SimSecs::default(),
            uplink: ScriptedUplink::new(),
            telemetry: SimTelemetry::new(),
            spacecraft: Spacecraft::new(),
        }
    }

    /// Open a timestamped MSFN downlink log file in `dir`.
    ///
    /// Replaces the current `SimTelemetry` sink with one that streams every
    /// downlink word to `downlink_<unix_secs>.bin` in addition to the
    /// in-memory `log`.  Call once after construction, before the run loop.
    ///
    /// # Errors
    /// Returns `Err` if the file cannot be created.
    pub fn open_downlink_log(
        &mut self,
        dir: &std::path::Path,
    ) -> std::io::Result<()> {
        self.telemetry = SimTelemetry::with_file(dir)?;
        Ok(())
    }

    /// Advance the simulator by `dt_seconds`.
    ///
    /// Reads `self.engine.thrusting`, integrates Δv on the
    /// [`Spacecraft`], drains accumulated PIPA pulses, and
    /// saturating-adds them into `self.imu.pipa` so the next
    /// `Imu::read_pipa` call returns them like real hardware would.
    /// Coast phases (engine off) leave the IMU pulse counters
    /// untouched — PIPAs are non-gravitational accelerometers.
    pub fn tick(&mut self, dt_seconds: f64) {
        let engine_on = self.engine.thrusting;
        self.spacecraft.tick(dt_seconds, engine_on);
        let pulses = self.spacecraft.drain_pipa_pulses();
        for (acc, &p) in self.imu.pipa.iter_mut().zip(pulses.iter()) {
            *acc = acc.saturating_add(p);
        }
    }
}

impl AgcHardware for SimHardware {
    type Timers = SimTimers;
    type Dsky = SimDsky;
    type Imu = SimImu;
    type Optics = SimOptics;
    type Engine = SimEngine;
    type Rcs = SimRcs;
    type Secs = SimSecs;
    type Uplink = ScriptedUplink;
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
    fn secs(&mut self) -> &mut SimSecs {
        &mut self.secs
    }
    fn engine(&mut self) -> &mut SimEngine {
        &mut self.engine
    }
    fn rcs(&mut self) -> &mut SimRcs {
        &mut self.rcs
    }
    fn uplink(&mut self) -> &mut ScriptedUplink {
        &mut self.uplink
    }
    fn telemetry(&mut self) -> &mut SimTelemetry {
        &mut self.telemetry
    }

    fn pet_watchdog(&mut self) { /* no-op in simulation */
    }

    fn hardware_restart(&mut self) -> ! {
        panic!("SimHardware: hardware_restart triggered")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use agc_core::hal::dsky::Lamp;
    use agc_core::hal::{Dsky, Engine, Imu, Optics, Rcs, Secs, Telemetry, Timers, Uplink};
    // NB: `ScriptedUplink` lives in `crate::uplink` and is re-exported at
    // the crate root; the `Uplink` import above brings the trait into
    // scope so `hw.uplink().read_word()` resolves.
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
        assert!((54321..54321 + 10).contains(&t), "expected ~54321, got {t}");
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

    /// TC-DSKY-04: `set_lamp` records every call into `lamp_events`,
    /// `drain_lamp_events` returns them in dispatch order and resets
    /// the queue.
    #[test]
    fn tc_dsky_04_set_lamp_records_events() {
        let mut hw = SimHardware::new();
        hw.dsky().set_lamp(Lamp::ProgAlarm, true);
        hw.dsky().set_lamp(Lamp::NoAtt, true);
        hw.dsky().set_lamp(Lamp::ProgAlarm, false);

        let events = hw.dsky.drain_lamp_events();
        assert_eq!(
            events,
            vec![
                (Lamp::ProgAlarm, true),
                (Lamp::NoAtt, true),
                (Lamp::ProgAlarm, false),
            ]
        );
        assert!(
            hw.dsky.drain_lamp_events().is_empty(),
            "second drain must return nothing"
        );
    }

    /// TC-DSKY-05: end-to-end SERVICER → refresh_lamps →
    /// `emit_dsky_to_hw` → `set_lamp` recorder.
    ///
    /// V46 ENTR brings up the SERVICER. The keystroke alone bumps
    /// `pinball_ticks` (latching COMP ACTY for the next T4 window),
    /// and the IMU is at its FRESH-START default of `Caged` (so NO ATT
    /// must be lit). After one T4Pump tick + a manual
    /// `emit_dsky_to_hw`, the recorded events must reflect both lamps
    /// being driven on, with PROG ALARM staying off (no alarm raised).
    #[test]
    fn tc_dsky_05_servicer_drives_recorded_lamp_events() {
        use crate::runtime::T4Pump;
        use agc_core::services::pinball::{decode_dsky, emit_dsky_to_hw};
        use agc_core::services::v_n::{feed_key, Key};
        use agc_core::AgcState;

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut t4 = T4Pump::new();

        // Drive V46 ENTR through the V/N processor — establishes the
        // SERVICER cycle and bumps the PINBALL tick counter.
        for k in [Key::Verb, Key::Digit(4), Key::Digit(6), Key::Entr] {
            feed_key(&mut state, k);
        }

        // One T4 tick fires t4rupt_step → refresh_lamps. Sim ticks do
        // not themselves call emit_dsky_to_hw (the interactive UI reads
        // via decode_dsky), so drive that step manually here — exactly
        // what the test/debug surface is intended for.
        t4.tick(&mut state, &mut hw);
        let frame = decode_dsky(&state.dsky);
        emit_dsky_to_hw(&frame, hw.dsky());

        let events = hw.dsky.drain_lamp_events();

        // Each lamp must appear exactly once per emit.
        assert_eq!(
            events.len(),
            10,
            "emit_dsky_to_hw must drive all 10 HAL lamps; got {events:?}"
        );

        let find = |lamp: Lamp| -> Option<bool> {
            events.iter().find(|(l, _)| *l == lamp).map(|(_, on)| *on)
        };

        assert_eq!(find(Lamp::NoAtt), Some(true), "Caged IMU → NO ATT lit");
        assert_eq!(
            find(Lamp::CompActy),
            Some(true),
            "PINBALL keystroke → COMP ACTY latched"
        );
        assert_eq!(
            find(Lamp::ProgAlarm),
            Some(false),
            "no alarm raised → PROG ALARM off"
        );
        assert_eq!(
            find(Lamp::Stby),
            Some(false),
            "no P06 entry → STBY off"
        );
        assert_eq!(
            find(Lamp::GimbalLock),
            Some(false),
            "CDU at 0° → GIMBAL LOCK off"
        );
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
        // 4096 = +22.5°; -32768 = -180° (i16::MIN, the same physical angle as the
        // pre-i16-migration test value 32768).
        hw.optics.trunnion = CduAngle(4096);
        hw.optics.shaft = CduAngle(i16::MIN);
        assert_eq!(hw.optics().trunnion_angle().0, 4096);
        assert_eq!(hw.optics().shaft_angle().0, i16::MIN);
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
        assert_eq!(hw.engine.gimbal_pitch, 0);
        assert_eq!(hw.engine.gimbal_yaw, 0);
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

    // ── SECS (TC-SECS-01 through TC-SECS-02) ────────────────────────────────

    /// TC-SECS-01: `deploy_drogue` sets the fired flag and bumps the counter.
    /// Repeated calls keep counting (idempotent at the hardware level but the
    /// counter lets tests assert single-shot semantics from the AGC side).
    #[test]
    fn tc_secs_01_deploy_drogue_counts() {
        let mut hw = SimHardware::new();
        assert!(!hw.secs.drogue_fired);
        assert_eq!(hw.secs.drogue_fire_count, 0);
        hw.secs().deploy_drogue();
        assert!(hw.secs.drogue_fired);
        assert_eq!(hw.secs.drogue_fire_count, 1);
        hw.secs().deploy_drogue();
        assert_eq!(hw.secs.drogue_fire_count, 2);
    }

    /// TC-SECS-02: initial state is no-fire, count zero (both pyros).
    #[test]
    fn tc_secs_02_initial_state() {
        let hw = SimHardware::new();
        assert!(!hw.secs.drogue_fired);
        assert_eq!(hw.secs.drogue_fire_count, 0);
        assert!(!hw.secs.csm_separation_fired);
        assert_eq!(hw.secs.csm_separation_fire_count, 0);
    }

    /// TC-SECS-03: `fire_csm_separation` sets the fired flag and bumps its
    /// counter, mirroring the drogue path. Independent of `deploy_drogue`.
    #[test]
    fn tc_secs_03_fire_csm_separation_counts() {
        let mut hw = SimHardware::new();
        hw.secs().fire_csm_separation();
        assert!(hw.secs.csm_separation_fired);
        assert_eq!(hw.secs.csm_separation_fire_count, 1);
        // Drogue must not have moved.
        assert!(!hw.secs.drogue_fired);
        assert_eq!(hw.secs.drogue_fire_count, 0);
        // Re-firing keeps counting (idempotent at hardware level).
        hw.secs().fire_csm_separation();
        assert_eq!(hw.secs.csm_separation_fire_count, 2);
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
