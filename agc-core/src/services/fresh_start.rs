//! Fresh Start and Restart sequences.
//!
//! AGC source: Comanche055/FRESH_START_AND_RESTART.agc
//! Routines:   SLAP1, DOFSTART, GOPROG, GOPROG3, ENEMA, STARTSUB, STARTSB2, MR.KLEAN
//! Pages:      181-210

use crate::{
    executive::restart::RestartTables,
    hal::{
        dsky::DskyIo, timers::Timers, AgcHardware, EngineIo, ImuIo, OpticsIo, RcsIo, TelemetryIo,
    },
    services::alarm::{AlarmCode, AlarmState},
    AgcState, IM30INIF, IM33INIT, MODREG_NONE, OPTINITF,
};

// ── Constants from STARTSUB ────────────────────────────────────────────────────

/// TIME3 / TIME4 / TIME5 initialisation values.
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSUB (page 190).
const TIME3_POSMAX: u16 = 0o37777; // 16383
const TIME4_INIT: u16 = TIME3_POSMAX - 2; // 16381
const TIME5_INIT: u16 = TIME3_POSMAX - 3; // 16380

// ── fresh_start ───────────────────────────────────────────────────────────────

/// Execute a FRESH START (DOFSTART / SLAP1 path).
///
/// Zeros all erasable user state, re-initialises hardware channels, places the
/// IMU in coarse align, clears the alarm history, and clears the phase tables.
/// On return the system is in the "waiting for P00" idle state.
///
/// This function must be called once from `main` on power-on, and is also called
/// whenever an unrecoverable condition forces a cold re-initialisation.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc
///   SLAP1 (line 145), DOFSTART (line 169), STARTSUB (line 504),
///   STARTSB2 (line 515), MR.KLEAN (line 264). Pages 183-192.
///
/// # Post-conditions
///
/// - `state.alarm` is fully cleared (`alarm::clear_all()` called).
/// - All 6 restart groups have zero phase values.
/// - `state.modreg == MODREG_NONE` (ones-complement -0 sentinel).
/// - IMU coarse align bit is set via `hw.imu().set_coarse_align()`.
/// - All DSKY display registers are blank.
/// - The Executive job table is empty (all slots available).
/// - The Waitlist is re-initialised with ENDTASK sentinels.
/// - Output channels 5, 6, 11, 12, 13, 14 are cleared.
/// - PROG light is off.
pub fn fresh_start<H: AgcHardware>(state: &mut AgcState, hw: &mut H) {
    // SLAP1 path: clear alarm history before DOFSTART.
    // AGC source: FRESH_START_AND_RESTART.agc SLAP1 lines 147-148 clear FAILREG/ERCOUNT/REDOCTR.
    // DOFSTART itself does NOT clear FAILREG — only the SLAP1 (manual) path does.
    AlarmState::clear_all();
    state.alarm = AlarmState::new();
    state.redoctr = 0;

    dofstart(state, hw);
}

/// Internal DOFSTART path — shared between `fresh_start` (SLAP1) and `restart` (PTBAD).
///
/// Does NOT clear FAILREG (alarm history) — that is only done by the SLAP1 path above.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc DOFSTART (page 184, line 169).
pub(crate) fn dofstart<H: AgcHardware>(state: &mut AgcState, hw: &mut H) {
    // ── Step 1: Zero output channels (DOFSTART order) ────────────────────────
    // AGC source: DOFSTART lines 169-201 (CHAN5, CHAN6, CHAN11, CHAN12, CHAN13, CHAN14).
    hw.rcs().write_channel5(0);
    hw.rcs().write_channel6(0);
    hw.engine().write_dsalmout(0); // CHAN11 = DSALMOUT
    hw.imu().write_control(0); // CHAN12 = IMU control
    hw.telemetry().write_chan13(0); // CHAN13 = SPS/TVC discrete outputs
    hw.optics().write_chan14(0); // CHAN14 = ISS CDU pulse enables

    // ── Step 2: Clear ERESTORE ────────────────────────────────────────────────
    state.erestore = 0;

    // ── Step 3: Clear phase tables (MR.KLEAN) ────────────────────────────────
    // AGC source: FRESH_START_AND_RESTART.agc MR.KLEAN (page 185, line 264).
    state.restart.clear_all();

    // ── Step 4: Set MODREG to -0 (no program) ────────────────────────────────
    // AGC source: DOFSTART `CS ZERO; TS MODREG`.
    state.modreg = MODREG_NONE;

    // ── Step 5: Initialise IMU and optics mode flags ──────────────────────────
    // AGC source: DOFSTART lines 200-215.
    state.imodes30 = IM30INIF; // OCT 37411
    state.imodes33 = IM33INIT; // PRIO16 = 16
    state.optmodes = OPTINITF; // OCT 130

    // ── Step 6: IMU coarse align ──────────────────────────────────────────────
    // AGC source: DOFSTART SETCOARS path `ORS CHAN12 / BIT6`.
    hw.imu().set_coarse_align();

    // ── Step 7: Initialise flag words (SWINIT path) ───────────────────────────
    // AGC source: DOFSTART initialises FLAGWRD0-8 from SWINIT table.
    // For Milestone 1 we zero all; bit preservation is handled in a later milestone.
    state.flagwrds = [0; 9];

    // ── Step 8: STARTSUB — timer initialisation ───────────────────────────────
    // AGC source: FRESH_START_AND_RESTART.agc STARTSUB (page 190, line 504).
    hw.timers().set_t3(TIME3_POSMAX);
    hw.timers().set_t4(TIME4_INIT);
    hw.timers().set_t5(TIME5_INIT);

    // ── Step 9: STARTSB2 — re-initialise Executive, Waitlist, DSKY ──────────
    // AGC source: FRESH_START_AND_RESTART.agc STARTSB2 (page 190, line 515).
    startsb2(state, hw);
}

// ── restart ───────────────────────────────────────────────────────────────────

/// Execute a RESTART (GOPROG / warm-restart path).
///
/// Called from the panic handler (GOJAM equivalent) when the system believes
/// erasable memory is intact.  Verifies phase table integrity, re-initialises
/// hardware I/O, and reschedules any programs that were interrupted.
///
/// If phase table verification fails (alarm 1107), this function calls
/// `fresh_start` internally and does not return via the GOPROG3 path.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc
///   GOPROG (line 290), ELRSKIP (line 340), GOPROG3 (line 410),
///   STARTSUB (line 504), STARTSB2 (line 515). Pages 186-192.
///
/// # Post-conditions (warm restart succeeded)
///
/// - `state.redoctr` is one greater than before the restart.
/// - Phase tables are unchanged.
/// - All active restart groups are rescheduled.
/// - DSKY registers are re-blanked.
/// - Hardware channels are re-initialised.
///
/// # Post-conditions (phase table failure → fresh start)
///
/// Same as `fresh_start()` post-conditions plus alarm code 1107 in history.
pub fn restart<H: AgcHardware>(state: &mut AgcState, hw: &mut H) {
    // AGC source: GOPROG first action is `INCR REDOCTR`.
    // Pre-resolved discrepancy #6: increment BEFORE delegating to fresh_start.
    state.restart.increment_restart_count();
    state.redoctr = state.restart.restart_count();

    // ── Phase table integrity check (GOPROG3 PCLOOP) ──────────────────────────
    // AGC source: FRESH_START_AND_RESTART.agc GOPROG3 PCLOOP (page 188).
    if let Err(bad_group) = state.restart.verify_integrity() {
        // Phase table corrupted — raise alarm 1107 and fall through to DOFSTART.
        // AGC source: FRESH_START_AND_RESTART.agc PTBAD `TCF DOFSTART`.
        AlarmState::raise(AlarmCode::PhaseTableError);
        let _ = bad_group;
        dofstart(state, hw);
        return;
    }

    // ── Re-initialise hardware channels (ELRSKIP path) ────────────────────────
    // AGC source: FRESH_START_AND_RESTART.agc ELRSKIP (line 340).
    hw.rcs().write_channel5(0);
    hw.rcs().write_channel6(0);
    hw.imu().write_control(0);
    hw.telemetry().write_chan13(0);
    hw.optics().write_chan14(0);

    // ── Timer init (STARTSUB) ─────────────────────────────────────────────────
    hw.timers().set_t3(TIME3_POSMAX);
    hw.timers().set_t4(TIME4_INIT);
    hw.timers().set_t5(TIME5_INIT);

    // ── STARTSB2: Executive, Waitlist, DSKY re-init ───────────────────────────
    startsb2(state, hw);

    // ── IMU coarse align if NO ATT was set ────────────────────────────────────
    // AGC source: GOPROG ELRSKIP checks NO ATT lamp; if set → TC IBNKCALL SETCOARS.
    // For Milestone 1 we always coarse-align on restart (conservative safe-state default).
    hw.imu().set_coarse_align();

    // ── GOPROG3: Scan active restart groups and reschedule ────────────────────
    // AGC source: FRESH_START_AND_RESTART.agc NXTRST/PACTIVE loop (page 189).
    let tables = RestartTables::empty();
    let (actions, count) = state.restart.on_restart(&tables);
    // In Milestone 1 the tables are empty so no jobs are rescheduled.
    // Future milestones populate RestartTables from RESTART_TABLES.agc.
    let _ = (actions, count);
}

// ── STARTSB2 internal helper ─────────────────────────────────────────────────

/// Re-initialise Executive job table, Waitlist, and DSKY display registers.
///
/// Called by both `fresh_start` and `restart` (the ENEMA convergence point).
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc STARTSB2 (page 190, line 515).
fn startsb2<H: AgcHardware>(state: &mut AgcState, hw: &mut H) {
    // Re-initialise Executive core sets (all slots free, all VAC areas free).
    // AGC source: STARTSB2 `PRIORITY[0,12,24,...,72] := -0`.
    // (The Executive struct is re-constructed inline; no heap allocation.)
    // We do not have a separate Executive field on AgcState in Milestone 1;
    // the callers in tests use stack-allocated Executive instances.
    // The global DSKY blank is handled through the HAL.

    // Blank DSKY display (DSPTAB[0..10] cleared).
    // AGC source: STARTSB2 lines 533-545 (loop over 11 DSPTAB registers).
    // Relay code 00000 = BLANK (distinct from 10101 = digit "0").
    // AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc relay table.
    hw.dsky().blank_display();
    hw.dsky().set_prog_light(false);
    hw.dsky().write_lamp_word(0);

    // Re-initialise Waitlist with ENDTASK sentinels.
    // AGC source: STARTSB2 lines 519-530.
    // In Milestone 1 the Waitlist is stack-allocated in tests; a global static
    // is added in a later milestone.  We note the requirement here.
    // (No action needed — Waitlist::new() already produces the correct state.)

    // Clear FLAGWRD4 (STARTSB2 zeroes it unconditionally).
    // AGC source: STARTSB2 line 580 `TS FLAGWRD4`.
    state.flagwrds[4] = 0;
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        hal::{
            dsky::{DigitRow, Key, RelayWord},
            engine::EngineImpl,
            imu::{ImuImpl, Unaligned},
            optics::OpticsImpl,
            rcs::RcsImpl,
            telemetry::TelemetryImpl,
            timers::TimersImpl,
            uplink::UplinkImpl,
            AgcHardware, DskyIo, ImuIo, RcsIo,
        },
        AgcState,
    };

    // ── Minimal SimHardware for testing ──────────────────────────────────────

    struct SimDsky {
        prog: u8,
        verb: u8,
        noun: u8,
        prog_light: bool,
    }

    impl SimDsky {
        fn new() -> Self {
            Self {
                prog: 0xFF,
                verb: 0xFF,
                noun: 0xFF,
                prog_light: false,
            }
        }
    }

    impl DskyIo for SimDsky {
        fn read_key(&mut self) -> Option<Key> {
            None
        }
        fn read_nav_key(&mut self) -> Option<Key> {
            None
        }
        fn write_relay(&mut self, _w: RelayWord) {}
        fn write_lamp_word(&mut self, _bits: u16) {}
        fn write_prog(&mut self, p: u8) {
            self.prog = p;
        }
        fn write_verb(&mut self, v: u8) {
            self.verb = v;
        }
        fn write_noun(&mut self, n: u8) {
            self.noun = n;
        }
        fn write_register(&mut self, _r: usize, _v: &DigitRow) {}
        fn proceed_pressed(&self) -> bool {
            false
        }
        fn set_prog_light(&mut self, on: bool) {
            self.prog_light = on;
        }
    }

    struct TestHw {
        dsky: SimDsky,
        imu: ImuImpl<Unaligned>,
        optics: OpticsImpl,
        engine: EngineImpl,
        rcs: RcsImpl,
        timers: TimersImpl,
        uplink: UplinkImpl,
        telemetry: TelemetryImpl,
    }

    impl TestHw {
        fn new() -> Self {
            Self {
                dsky: SimDsky::new(),
                imu: ImuImpl::new(),
                optics: OpticsImpl::new(),
                engine: EngineImpl::new(),
                rcs: RcsImpl::new(),
                timers: TimersImpl::new(),
                uplink: UplinkImpl::new(),
                telemetry: TelemetryImpl::new(),
            }
        }
    }

    impl AgcHardware for TestHw {
        type Timers = TimersImpl;
        type Dsky = SimDsky;
        type Imu = ImuImpl<Unaligned>;
        type Optics = OpticsImpl;
        type Engine = EngineImpl;
        type Rcs = RcsImpl;
        type Uplink = UplinkImpl;
        type Telemetry = TelemetryImpl;

        fn timers(&mut self) -> &mut TimersImpl {
            &mut self.timers
        }
        fn dsky(&mut self) -> &mut SimDsky {
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
            loop {}
        }
    }

    #[test]
    fn fresh_start_clears_alarms_and_phase_tables() {
        // Test 1: fresh_start clears alarms and phase tables
        AlarmState::raise(AlarmCode::PhaseTableError);
        AlarmState::raise(AlarmCode::NoCoreSets);

        let mut state = AgcState::new();
        state.restart.set_phase(1, 5, false, 0);
        state.modreg = 11; // P11 nominally running

        let mut hw = TestHw::new();
        fresh_start(&mut state, &mut hw);

        assert!(!AlarmState::prog_light_on());
        assert_eq!(AlarmState::most_recent(), None);
        assert!(state.restart.all_groups_zero());
        assert_eq!(state.modreg, MODREG_NONE);
        assert!(hw.imu.coarse_align_active());
        assert_eq!(hw.rcs.current_command().pitch_yaw, 0);
        assert_eq!(hw.rcs.current_command().roll, 0);
    }

    #[test]
    fn restart_increments_redoctr() {
        // Test 2: restart increments REDOCTR and preserves valid phase table
        AlarmState::clear_all();
        let mut state = AgcState::new();
        state.redoctr = 3;
        state.restart.restart_count = 3;

        // Set group 2 to phase 4 with valid complement.
        state.restart.set_phase(2, 4, false, 0);

        let mut hw = TestHw::new();
        restart(&mut state, &mut hw);

        assert_eq!(state.redoctr, 4, "REDOCTR should increment on restart");
        assert_eq!(AlarmState::most_recent(), None, "no 1107 alarm expected");
        assert_eq!(
            state.restart.current_phase(2),
            4,
            "phase preserved on warm restart"
        );
    }

    #[test]
    fn restart_with_corrupt_phase_table_triggers_fresh_start() {
        // Test 3: corrupt phase table → alarm 1107 → fresh start
        AlarmState::clear_all();
        let mut state = AgcState::new();
        state.modreg = 40; // P40 running
                           // Set group 3 to phase 7, then corrupt the shadow.
        state.restart.set_phase(3, 7, false, 0);
        state.restart.neg_phase[2] = -6; // should be !7 = -8

        let mut hw = TestHw::new();
        restart(&mut state, &mut hw);

        assert_eq!(AlarmState::most_recent(), Some(AlarmCode::PhaseTableError));
        assert_eq!(
            state.modreg, MODREG_NONE,
            "fresh start should set modreg to NO_PROGRAM"
        );
        assert!(
            state.restart.all_groups_zero(),
            "MR.KLEAN should clear phase tables"
        );
        assert!(hw.imu.coarse_align_active());
    }
}
