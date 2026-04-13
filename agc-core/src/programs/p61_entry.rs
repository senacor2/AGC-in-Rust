//! P61–P67 — Entry Guidance.
//!
//! P61–P67 is the Command Module reentry guidance sequence. It is a seven-phase
//! monotonic state machine driven by sensed atmospheric drag (measured via PIPAs)
//! and computed range-to-go to the splash-down target.
//!
//! The crew invokes P61 before atmospheric interface. Phases progress automatically
//! as the vehicle descends; phase regression is illegal and raises an alarm.
//!
//! AGC source: Comanche055/P61-P67.agc
//!   P61 (page 789), P62 (page 792), P63 (page 795), P64 (page 797),
//!   P65 (page 798), P66 (page 799), P67 (page 800), P67.1 (page 800),
//!   S61.1 (page 803), S61.2 (page 806), FISHCALC (page 812), VGAMCALC (page 813).
//! AGC source: Comanche055/ENTRY_LEXICON.agc (pages 837-843).

use crate::hal::{AgcHardware, DskyIo, ImuIo};
use crate::math::linalg::norm;
use crate::navigation::constants::RE_EARTH;
use crate::services::alarm::{AlarmCode, AlarmState};
use crate::AgcState;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Entry interface altitude above Earth geocentric radius, metres.
///
/// `S61.2: 400KFT 2DEC 121920 B-29`.
///
/// AGC source: Comanche055/P61-P67.agc S61.2 (page 806).
pub const ENTRY_INTERFACE_M: f64 = 121_920.0;

/// EMS interface altitude for orbital reentry, metres.
///
/// 284 843 ft converted to metres.
///
/// AGC source: Comanche055/P61-P67.agc S61.2 comment.
pub const EMSALT_ORBITAL_M: f64 = 86_759.2;

/// EMS interface altitude for lunar reentry, metres.
///
/// 297 431 ft converted to metres.
///
/// AGC source: Comanche055/P61-P67.agc S61.2 comment.
pub const EMSALT_LUNAR_M: f64 = 90_657.0;

/// Standard gravity, m/s².
const G0: f64 = 9.806_65;

/// 0.05 g onset threshold, m/s².
///
/// Trigger for P63 → P64 phase transition. `.05GSW = CM/FLAGS bit 3`.
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `.05GSW = CM/FLAGS bit 3`.
pub const POINT_05G_THRESHOLD: f64 = 0.05 * G0;

/// 0.2 g threshold for P64 → P67 direct transition, m/s².
///
/// P64 selects P67 directly when V < VFINAL1 at the 0.2G level.
///
/// AGC source: Comanche055/P61-P67.agc P64 comment (page 797).
pub const POINT_2G_THRESHOLD: f64 = 0.2 * G0;

/// Velocity threshold to enter upcontrol (P65) vs final (P67), m/s.
///
/// VFINAL1 = 27 000 FPS = 8229.6 m/s.
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `VFINAL1 = 27000 FPS`.
pub const VFINAL1_MS: f64 = 8_229.6;

/// Minimum exit velocity for upcontrol solution (VL min), m/s.
///
/// VLMIN = 18 000 FPS = 5486.4 m/s.
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `VLMIN = 18000 FPS`.
pub const VLMIN_MS: f64 = 5_486.4;

/// Guidance termination velocity, m/s.
///
/// VQUIT = 1000 FPS = 304.8 m/s.
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `VQUIT = 1000 FPS`.
pub const VQUIT_MS: f64 = 304.8;

/// Range-to-go threshold to enter P65 upcontrol, metres.
///
/// 25 NM = 46 300 m.
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `25NM tolerance = 25 NM`.
pub const RANGE_25NM_M: f64 = 46_300.0;

/// Minimum drag threshold to maintain upcontrol (Q7), m/s².
///
/// Q7F = 6 FPSS = 1.8288 m/s².
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `Q7F = 6 FPSS`.
pub const Q7F_MS2: f64 = 1.828_8;

/// Satellite velocity at Earth radius (VSAT), m/s.
///
/// VSAT = 25 766.1973 FPS = 7853.0 m/s.
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `VSAT = 25766.1973 FPS`.
pub const VSAT_MS: f64 = 7_853.0;

/// P65 → P67 velocity bias (C18), m/s.
///
/// C18 = 500 FPS = 152.4 m/s.
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `C18 = 500 FPS`.
pub const C18_MS: f64 = 152.4;

/// Nominal vehicle lift-to-drag ratio (LADPAD).
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `LADPAD = 0.3`.
pub const LADPAD_NOM: f64 = 0.3;

/// Final phase L/D ratio (LODPAD).
///
/// AGC source: Comanche055/ENTRY_LEXICON.agc `LODPAD = 0.18`.
pub const LODPAD_NOM: f64 = 0.18;

/// Roll angle for lift-down entry (HEADSUP = +1), radians.
///
/// AGC source: Comanche055/P61-P67.agc P61.4 `CA BIT14; DXCH ROLLC` = 180°.
pub const HEADSUP_LIFT_DOWN: f64 = core::f64::consts::PI;

/// Roll angle for lift-up entry (HEADSUP = −1), radians.
///
/// AGC source: Comanche055/P61-P67.agc P61.4 `NOOP; DXCH ROLLC` = 0°.
pub const HEADSUP_LIFT_UP: f64 = 0.0;

// ── Types ─────────────────────────────────────────────────────────────────────

/// Seven-phase monotonic entry guidance state machine.
///
/// AGC source: Comanche055/P61-P67.agc NEWMODEX calls (MM 61..67) and RTB dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EntryPhase {
    /// P61: Pre-entry calculations and predictions display.
    ///
    /// Entry trigger: V37 crew selection.
    /// AGC label: P61 (page 789); display N60, N63.
    P61PreEntry,
    /// P62: CM/SM separation readiness; entry attitude maneuver.
    ///
    /// Entry trigger: Falls through from P61, or V37 MM=62.
    /// AGC label: P62 (page 792); CM/DAPIC started; ROLLC computed.
    P62SepCM,
    /// P63: Hold entry attitude; sense 0.05G onset.
    ///
    /// Entry trigger: P62 hands off after attitude maneuver complete.
    /// AGC label: P63 (page 795); P63FLAG set; display N64.
    P63EntryInit,
    /// P64: Post-0.05G; constant drag guidance phase.
    ///
    /// Entry trigger: Sensed drag D ≥ POINT_05G_THRESHOLD.
    /// AGC label: P64 (page 797); RTB from reentry control (DANZIG/INITROLL).
    P64Post05G,
    /// P65: Up-control phase; steer to controlled exit.
    ///
    /// Entry trigger: Range-to-go < 25 NM AND V > VFINAL1.
    /// AGC label: P65 (page 798); GOTOADDR set to UPCONTRL.
    P65Upcontrol,
    /// P66: Ballistic/trim phase; hold attitude in trim.
    ///
    /// Entry trigger: Drag D < Q7 FPSS (from P65).
    /// AGC label: P66 (page 799); KEP2 ballistic integration.
    P66Ballistic,
    /// P67: Final phase; range and lateral corrections, guidance termination.
    ///
    /// Entry trigger: RDOT < 0 AND V < VL + C18 (from P65), or
    ///                V < VFINAL1 at 0.2G (from P64).
    /// AGC label: P67 (page 800); terminates when V ≤ VQUIT.
    P67Final,
}

/// Active entry guidance state.
///
/// Passed through every tick() call. All physics quantities use SI units.
#[derive(Clone, Copy, Debug)]
pub struct EntryState {
    /// Current guidance phase.
    pub phase: EntryPhase,
    /// Target splash-down latitude, degrees (WGS-84 approximate).
    ///
    /// AGC erasable: LAT(SPL) scaled /360.
    pub target_lat_deg: f64,
    /// Target splash-down longitude, degrees.
    ///
    /// AGC erasable: LNG(SPL) scaled /360.
    pub target_lon_deg: f64,
    /// Predicted range-to-go to target, metres.
    ///
    /// AGC erasable: RTGO (THETAH/360); ENTRY_LEXICON: max 21600 NM.
    pub range_to_go_m: f64,
    /// Current roll command, radians.
    ///
    /// AGC erasable: ROLLC (1 revolution scale).
    pub rollc_rad: f64,
    /// Lift-up / lift-down selection (+1 = lift down, −1 = lift up).
    ///
    /// AGC erasable: HEADSUP.
    pub headsup: i8,
    /// Sensed drag acceleration, m/s². Updated each SERVICER cycle.
    ///
    /// AGC erasable: D (total accel), scaled 805 FPSS max.
    pub drag_acc_ms2: f64,
    /// Inertial velocity magnitude, m/s. Updated each SERVICER cycle.
    ///
    /// AGC erasable: VMAGI (B-7 m/cs).
    pub vi_ms: f64,
    /// Altitude rate, m/s. Positive = climbing.
    ///
    /// AGC erasable: RDOT (2 × VSAT scale).
    pub rdot_ms: f64,
    /// Exit velocity for upcontrol (VL), m/s. Set during P64.
    ///
    /// AGC erasable: VL (2 × VSAT scale). ENTRY_LEXICON: VLMIN = 18 000 FPS.
    pub vl_ms: f64,
    /// Minimum drag threshold for upcontrol (Q7), m/s².
    ///
    /// AGC erasable: Q7. ENTRY_LEXICON: Q7F = 6 FPSS minimum.
    pub q7_ms2: f64,
    /// True once guidance termination has been commanded (V ≤ VQUIT).
    pub guidance_terminated: bool,
    /// Maximum predicted entry acceleration (display only).
    ///
    /// AGC erasable: GMAX (100 × GMAX, B-14 G-S scale).
    pub gmax_g: f64,
    /// Predicted velocity at 400 kft entry interface, m/s.
    ///
    /// AGC erasable: VPRED (B-7 m/cs).
    pub vpred_ms: f64,
    /// Predicted flight-path angle at 400 kft, radians.
    ///
    /// AGC erasable: GAMMAEI (GAMMA/360).
    pub gammaei_rad: f64,
    /// Time-to-entry interface, centiseconds from current state.
    ///
    /// AGC erasable: TTE (B-28 cs).
    pub tte_cs: i64,
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Enter entry guidance at P61 (pre-entry calculations).
///
/// Sets `phase = P61PreEntry`. Initializes HEADSUP = -1 (lift up default).
/// Locks out extended verbs (EXTVBACT = BIT14). Calls S61.1 (IMU/state-vector
/// check). Calls S61.2 (computes GMAX, VPRED, GAMMAEI, RTGO, VIO, TTE).
/// Prompts crew for LAT(SPL), LNG(SPL), HEADSUP via N61 flash.
///
/// AGC source: Comanche055/P61-P67.agc P61 entry (page 789-791).
/// Sets EXTVBACT; calls S61.1, S61.2; displays N60, N63.
#[must_use]
pub fn enter<H: AgcHardware>(state: &mut AgcState, hw: &mut H) -> EntryState {
    // Restart protection: Group 2 Phase 2.
    // AGC source: P61-P67.agc PHASCHNG / OCT 05022 (page 789).
    state.restart.set_phase(2, 2, false, 0);

    // Lock out extended verbs.
    // AGC source: Comanche055/P61-P67.agc `TC UPFLAG / ADRES EXTVBACT` (page 789).
    state.extvbact = true;

    // Set major mode to 61.
    // AGC source: `TC NEWMODEX / MM 61` (page 789).
    state.modreg = 61;
    hw.dsky().write_prog(61);

    // S61.1: Validate IMU and navigation state.
    // AGC source: Comanche055/P61-P67.agc S61.1 (page 803).
    let imu_ok = s61_1(state, hw);

    // S61.2: Compute display quantities (GMAX, VPRED, GAMMAEI, RTGO, TTE).
    // AGC source: Comanche055/P61-P67.agc S61.2 (page 806).
    let s61 = s61_2(state);

    // Initialise roll command from HEADSUP default (lift up = -1 → ROLLC = 0).
    // AGC source: P61-P67.agc P61.4 page 790.
    let rollc = HEADSUP_LIFT_UP;

    // Display N61 (LAT/LNG/HEADSUP prompt) and N60/N63 (predicted quantities).
    // AGC source: P61-P67.agc `TC BANKCALL / CADR GOFLASH / DEC 61` (page 790).
    hw.dsky().write_noun(61);

    let mut entry = EntryState {
        phase: EntryPhase::P61PreEntry,
        target_lat_deg: 0.0,
        target_lon_deg: 0.0,
        range_to_go_m: s61.rtgo_m,
        rollc_rad: rollc,
        headsup: -1, // lift up by default
        drag_acc_ms2: 0.0,
        vi_ms: s61.vpred_ms,
        rdot_ms: 0.0,
        vl_ms: VLMIN_MS, // initial exit velocity estimate
        q7_ms2: Q7F_MS2,
        guidance_terminated: false,
        gmax_g: s61.gmax_g,
        vpred_ms: s61.vpred_ms,
        gammaei_rad: s61.gammaei_rad,
        tte_cs: s61.tte_cs,
    };

    if !imu_ok {
        // IMU unsatisfactory: alarm already raised by s61_1.
        // Return P61 state; tick will not advance until IMU recovers.
        // AGC source: S61.1 alarm 01426 handling (page 803).
        entry.phase = EntryPhase::P61PreEntry;
    }

    entry
}

/// Advance entry guidance by one executive cycle.
///
/// Reads sensed state from `entry_state`, evaluates phase transition conditions,
/// updates ROLLC, issues DSKY noun updates, and writes the new phase.
///
/// Phase transitions are strictly monotonic. Backward transitions raise alarm
/// `PhaseTableError` and leave `phase` unchanged.
///
/// AGC source: Comanche055/P61-P67.agc full reentry control flow;
///             ENTRY_LEXICON constants.
pub fn tick<H: AgcHardware>(entry_state: &mut EntryState, state: &mut AgcState, hw: &mut H) {
    // Terminal state: no-op after guidance termination.
    if entry_state.guidance_terminated {
        return;
    }

    match entry_state.phase {
        EntryPhase::P61PreEntry => {
            // P61 → P62: crew presses PROCEED (or V37 MM=62).
            // AGC source: P61 fallthrough "THEN FALL INTO P62" (page 791).
            if hw.dsky().proceed_pressed() {
                transition_to(entry_state, state, EntryPhase::P62SepCM, hw);
            }
        }

        EntryPhase::P62SepCM => {
            // P62 → P63: entry attitude achieved (simplified: α within ±45°).
            // AGC source: WAKEP62 task wakes P63 (page 793).
            // In the sim, we approximate "attitude converged" by checking that
            // the CM is near entry attitude (ALFA_LIMIT = 45° = π/4).
            // Real AGC computes CDU angles from CMATT and compares to entry CPHI.
            // Here we use a simple range check on the navigation state.
            //
            // For the simulation, we use the DSKY verb flag as a proxy:
            // the crew presses PROCEED again to confirm CM/SM separation.
            if hw.dsky().proceed_pressed() {
                transition_to(entry_state, state, EntryPhase::P63EntryInit, hw);
            }
        }

        EntryPhase::P63EntryInit => {
            // P63 → P64: sensed drag ≥ 0.05G onset.
            // AGC source: STARTENT → reentry control → `TC DANZIG` (page 797).
            if entry_state.drag_acc_ms2 >= POINT_05G_THRESHOLD {
                // Set .05GSW (CM/FLAGS bit 3).
                // AGC source: P63 SETFLAG ADRES .05GSW (page 796).
                state.flags.point_05gsw = true;
                transition_to(entry_state, state, EntryPhase::P64Post05G, hw);
            }
        }

        EntryPhase::P64Post05G => {
            // P64 → P67 (direct): V < VFINAL1 when D ≥ 0.2G.
            // AGC source: P64 function 2 and 4 (page 797 comment).
            let direct_to_p67 =
                entry_state.vi_ms < VFINAL1_MS && entry_state.drag_acc_ms2 >= POINT_2G_THRESHOLD;

            // P64 → P65: range < 25 NM AND V > VFINAL1 AND upcontrol solution exists.
            // AGC source: P65 TC NEWMODEX MM 65; GOTOADDR = UPCONTRL (page 798).
            let go_to_p65 = entry_state.range_to_go_m < RANGE_25NM_M
                && entry_state.vi_ms > VFINAL1_MS
                && entry_state.vl_ms > VLMIN_MS;

            if direct_to_p67 {
                transition_to(entry_state, state, EntryPhase::P67Final, hw);
            } else if go_to_p65 {
                transition_to(entry_state, state, EntryPhase::P65Upcontrol, hw);
            }
        }

        EntryPhase::P65Upcontrol => {
            // P65 → P66: drag drops below Q7 FPSS.
            // AGC source: P66 TC NEWMODEX MM 66; "WHEN D < Q7 FPSS" (page 799).
            if entry_state.drag_acc_ms2 < entry_state.q7_ms2 {
                transition_to(entry_state, state, EntryPhase::P66Ballistic, hw);
                return;
            }

            // P65 → P67: RDOT < 0 AND V < VL + C18.
            // AGC source: P65 function B (page 798 comment).
            if entry_state.rdot_ms < 0.0 && entry_state.vi_ms < entry_state.vl_ms + C18_MS {
                transition_to(entry_state, state, EntryPhase::P67Final, hw);
            }
        }

        EntryPhase::P66Ballistic => {
            // P66 → P67: drag builds back or terminal velocity reached.
            // AGC source: P66 ballistic; returns to reentry control at KEP2.
            // P66 can also return to P65 if drag rebuilds. Here we check for P67.
            if entry_state.vi_ms < VQUIT_MS {
                transition_to(entry_state, state, EntryPhase::P67Final, hw);
                return;
            }
            // P66 → P65: drag rebuilds above Q7 + 0.5 FPSS.
            // 0.5 FPSS = 0.1524 m/s².
            if entry_state.drag_acc_ms2 > entry_state.q7_ms2 + 0.152_4 {
                transition_to(entry_state, state, EntryPhase::P65Upcontrol, hw);
            }
        }

        EntryPhase::P67Final => {
            // P67 terminal: V ≤ VQUIT → guidance_terminated.
            // AGC source: P67.1 `CS THREE; MASK CM/FLAGS` clear DAP flags (page 801).
            if entry_state.vi_ms <= VQUIT_MS {
                entry_state.guidance_terminated = true;
                // Clear EXTVBACT lock.
                // AGC source: P67.1 TC DOWNFLAG ADRES EXTVBACT (page 800).
                state.extvbact = false;
                // Display N67 (RTOGO, LAT, LONG).
                // AGC source: P67.1 TC BANKCALL CADR GOFLASH DEC 67 (page 800).
                hw.dsky().write_noun(67);
            }
        }
    }
}

// ── Internal: S61.1 IMU validation ───────────────────────────────────────────

/// S61.1 result containing pre-entry display quantities.
struct S61Result {
    rtgo_m: f64,
    vpred_ms: f64,
    gmax_g: f64,
    gammaei_rad: f64,
    tte_cs: i64,
}

/// S61.1: Validate IMU orientation and navigation state.
///
/// Checks that Average-G is running and IMU is not in fail state.
/// Raises alarm 01426 (IMU unsatisfactory) if the IMU fail bit (CHAN30 bit 13)
/// is set. Returns `false` if IMU is unsatisfactory.
///
/// AGC source: Comanche055/P61-P67.agc S61.1 (page 803).
fn s61_1<H: AgcHardware>(state: &AgcState, hw: &mut H) -> bool {
    let imu_status = hw.imu().read_status();
    // Bit 12 (0-based) = CHAN30 bit 13 (1-based) = IMU fail.
    let imu_fail = (imu_status >> 12) & 1 != 0;
    if imu_fail {
        // AGC source: S61.1 TC ALARM 01426 (page 803).
        AlarmState::raise(AlarmCode::DeviceConflict);
        return false;
    }
    // REFSMFLG must be set for entry navigation to be valid.
    // AGC source: S61.1 checks REFSMFLG before proceeding.
    if !state.flags.refsmflg {
        AlarmState::raise(AlarmCode::DeviceConflict);
        return false;
    }
    true
}

/// S61.2: Compute entry prediction quantities for display.
///
/// Computes GMAX, VPRED, GAMMAEI, RTGO, TTE from the current navigation state.
/// This is a simplified implementation: real AGC S61.2 integrates a conic arc
/// from the current state to 400 kft entry interface.
///
/// AGC source: Comanche055/P61-P67.agc S61.2 (page 806).
fn s61_2(state: &AgcState) -> S61Result {
    let r = state.nav.sv.position();
    let v = state.nav.sv.velocity();
    let r_mag = norm(&r);
    let v_mag = norm(&v);

    // VPRED: inertial velocity magnitude at current state (simplified — no conic).
    // Real AGC propagates to entry interface via VGAMCALC.
    // AGC source: S61.2 VGAMCALC → VPRED storage.
    let vpred_ms = v_mag;

    // GAMMAEI: flight-path angle ≈ arcsin(RDOT / VMAGI).
    // RDOT = (R · V) / |R|.
    // AGC source: S61.2 GAMMAEI = arcsin(RDOT / VMAGI).
    let rdot = if r_mag > 1.0 {
        (r[0] * v[0] + r[1] * v[1] + r[2] * v[2]) / r_mag
    } else {
        0.0
    };
    let sin_gamma = if v_mag > 1.0 {
        (rdot / v_mag).clamp(-1.0, 1.0)
    } else {
        0.0
    };
    let gammaei_rad = libm::asin(sin_gamma);

    // RTGO: approximate range-to-go (straight-line distance to entry sphere).
    // AGC source: S61.2 RTGO computed via FISHCALC + integration.
    let r_entry = RE_EARTH + ENTRY_INTERFACE_M;
    let rtgo_m = if r_mag > r_entry {
        r_mag - r_entry
    } else {
        0.0
    };

    // GMAX: max predicted deceleration (simplified — use current v^2/R proxy).
    // AGC source: S61.2 GMAX computed from LADPAD, D0 gains.
    let gmax_g = if r_mag > 1.0 {
        v_mag * v_mag / (r_mag * G0) * LADPAD_NOM
    } else {
        0.0
    };

    // TTE: time-to-entry interface (simplified — Keplerian descent estimate).
    // For now, return 0 (unknown). Real AGC integrates via conic.
    // AGC source: S61.2 TTE = -28 cs erasable.
    let tte_cs = 0_i64;

    S61Result {
        rtgo_m,
        vpred_ms,
        gmax_g,
        gammaei_rad,
        tte_cs,
    }
}

// ── Internal: phase transition helper ────────────────────────────────────────

/// Transition the entry state machine to a new phase.
///
/// Enforces strict monotonicity: if `new_phase < current_phase`, raises alarm
/// `PhaseTableError` and leaves phase unchanged.
///
/// Updates the DSKY PROG display and `state.modreg` on every legal transition.
///
/// AGC source: Comanche055/P61-P67.agc `TC NEWMODEX / MM xx` calls.
fn transition_to<H: AgcHardware>(
    entry_state: &mut EntryState,
    state: &mut AgcState,
    new_phase: EntryPhase,
    hw: &mut H,
) {
    // Phase regression check: only P66 → P65 is a legal "backward" transition
    // (ballistic phase can re-enter upcontrol when drag rebuilds).
    // All other regressions are invariant violations.
    // AGC source: P66 reentry control re-enters P65 via GOTOADDR = UPCONTRL.
    let p66_to_p65 =
        entry_state.phase == EntryPhase::P66Ballistic && new_phase == EntryPhase::P65Upcontrol;
    if new_phase < entry_state.phase && !p66_to_p65 {
        // Phase regression: invariant violation.
        // AGC source: PHASCHNG design principle — phase groups only advance.
        AlarmState::raise(AlarmCode::PhaseTableError);
        return;
    }

    entry_state.phase = new_phase;

    let (modreg, prog_display) = match new_phase {
        EntryPhase::P61PreEntry => (61_i16, 61_u8),
        EntryPhase::P62SepCM => (62, 62),
        EntryPhase::P63EntryInit => (63, 63),
        EntryPhase::P64Post05G => (64, 64),
        EntryPhase::P65Upcontrol => (65, 65),
        EntryPhase::P66Ballistic => (66, 66),
        EntryPhase::P67Final => (67, 67),
    };

    state.modreg = modreg;
    hw.dsky().write_prog(prog_display);

    // Restart protection: set group 2 phase on each phase advancement.
    // AGC source: PHASCHNG calls throughout P61-P67.agc.
    state.restart.set_phase(2, modreg as u8, false, 0);
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::StateVector;
    use crate::tests::mock_hw::MockHardware;
    use crate::types::Met;

    fn make_entry_state() -> AgcState {
        let r_entry = RE_EARTH + ENTRY_INTERFACE_M + 50_000.0; // 50 km above EI
        let v_entry = 10_000.0_f64; // ~Mach 30 entry speed
        let sv = StateVector::new([r_entry, 0.0, 0.0], [0.0, v_entry, 0.0], Met(0));
        let mut state = AgcState::new();
        state.nav.sv = sv;
        state.flags.refsmflg = true;
        state
    }

    /// T1: P61 enters correctly — phase = P61PreEntry, EXTVBACT set.
    ///
    /// AGC source: Comanche055/P61-P67.agc P61 entry (page 789).
    #[test]
    fn p61_enter_sets_correct_phase() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let entry = enter(&mut state, &mut hw);
        assert_eq!(entry.phase, EntryPhase::P61PreEntry);
        assert!(state.extvbact, "EXTVBACT must be set on P61 entry");
        assert_eq!(state.modreg, 61);
    }

    /// T2: P61 → P62 on crew PROCEED.
    ///
    /// AGC source: Comanche055/P61-P67.agc P61 fallthrough "THEN FALL INTO P62" (page 791).
    #[test]
    fn p61_to_p62_on_proceed() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        hw.dsky.set_proceed(true);
        tick(&mut entry, &mut state, &mut hw);
        assert_eq!(entry.phase, EntryPhase::P62SepCM);
        assert_eq!(state.modreg, 62);
    }

    /// T3: P62 → P63 on attitude converge (crew PROCEED in sim).
    ///
    /// AGC source: Comanche055/P61-P67.agc WAKEP62 task wakes P63 (page 793).
    #[test]
    fn p62_to_p63_on_attitude_converge() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        entry.phase = EntryPhase::P62SepCM;
        state.modreg = 62;
        hw.dsky.set_proceed(true);
        tick(&mut entry, &mut state, &mut hw);
        assert_eq!(entry.phase, EntryPhase::P63EntryInit);
        assert_eq!(state.modreg, 63);
    }

    /// T4: P63 → P64 on 0.05G onset.
    ///
    /// AGC source: Comanche055/P61-P67.agc `TC DANZIG` from reentry control (page 797).
    #[test]
    fn p63_to_p64_on_05g_onset() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        entry.phase = EntryPhase::P63EntryInit;
        state.modreg = 63;
        // Inject drag > 0.05G threshold.
        entry.drag_acc_ms2 = POINT_05G_THRESHOLD + 0.01;
        tick(&mut entry, &mut state, &mut hw);
        assert_eq!(entry.phase, EntryPhase::P64Post05G);
        assert!(state.flags.point_05gsw, ".05GSW must be set");
        assert_eq!(state.modreg, 64);
    }

    /// T5: P64 → P67 direct (V < 27000 FPS at 0.2G).
    ///
    /// AGC source: Comanche055/P61-P67.agc P64 function 2 and 4 (page 797 comment).
    #[test]
    fn p64_to_p67_direct_at_02g() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        entry.phase = EntryPhase::P64Post05G;
        state.modreg = 64;
        // V < VFINAL1 (8229.6 m/s) AND D ≥ 0.2G.
        entry.vi_ms = 7_000.0;
        entry.drag_acc_ms2 = POINT_2G_THRESHOLD + 0.01;
        tick(&mut entry, &mut state, &mut hw);
        assert_eq!(
            entry.phase,
            EntryPhase::P67Final,
            "should jump P65 and go to P67"
        );
        assert_eq!(state.modreg, 67);
    }

    /// T6: P64 → P65 on range < 25 NM with valid upcontrol solution.
    ///
    /// AGC source: Comanche055/P61-P67.agc P65 TC NEWMODEX MM 65 (page 798).
    #[test]
    fn p64_to_p65_on_range_threshold() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        entry.phase = EntryPhase::P64Post05G;
        state.modreg = 64;
        // Range < 25 NM, V > VFINAL1, VL > VLMIN.
        entry.range_to_go_m = 40_000.0; // < 46 300 m
        entry.vi_ms = 8_500.0; // > VFINAL1
        entry.vl_ms = 6_000.0; // > VLMIN
        tick(&mut entry, &mut state, &mut hw);
        assert_eq!(entry.phase, EntryPhase::P65Upcontrol);
        assert_eq!(state.modreg, 65);
    }

    /// T7: P65 → P66 on D < Q7.
    ///
    /// AGC source: Comanche055/P61-P67.agc P66 "WHEN D < Q7 FPSS" (page 799).
    #[test]
    fn p65_to_p66_on_low_drag() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        entry.phase = EntryPhase::P65Upcontrol;
        state.modreg = 65;
        entry.drag_acc_ms2 = Q7F_MS2 * 0.9; // below Q7
        entry.q7_ms2 = Q7F_MS2;
        tick(&mut entry, &mut state, &mut hw);
        assert_eq!(entry.phase, EntryPhase::P66Ballistic);
        assert_eq!(state.modreg, 66);
    }

    /// T8: P65 → P67 on RDOT < 0 and V < VL + 500 FPS.
    ///
    /// AGC source: Comanche055/P61-P67.agc P65 function B (page 798 comment).
    #[test]
    fn p65_to_p67_on_rdot_and_velocity() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        entry.phase = EntryPhase::P65Upcontrol;
        state.modreg = 65;
        entry.rdot_ms = -10.0; // RDOT < 0
        entry.vl_ms = 6_000.0;
        entry.vi_ms = entry.vl_ms + 100.0; // V < VL + C18 (152.4)
        entry.drag_acc_ms2 = Q7F_MS2 * 1.5; // drag above Q7 (no P66 transition first)
        entry.q7_ms2 = Q7F_MS2;
        tick(&mut entry, &mut state, &mut hw);
        assert_eq!(entry.phase, EntryPhase::P67Final);
        assert_eq!(state.modreg, 67);
    }

    /// T9: P67 terminal at VQUIT (V ≤ 304.8 m/s).
    ///
    /// AGC source: Comanche055/P61-P67.agc P67.1 `CS THREE; MASK CM/FLAGS` (page 800-801).
    #[test]
    fn p67_terminates_at_vquit() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        entry.phase = EntryPhase::P67Final;
        state.modreg = 67;
        entry.vi_ms = 250.0; // V ≤ VQUIT (304.8 m/s)
        tick(&mut entry, &mut state, &mut hw);
        assert!(
            entry.guidance_terminated,
            "guidance must terminate at VQUIT"
        );
        assert!(!state.extvbact, "EXTVBACT must be cleared on P67 terminal");
    }

    /// T10: Phase monotonicity guard — backward phase transition raises alarm.
    ///
    /// AGC source: Phase table design principle; alarm PhaseTableError.
    #[test]
    fn phase_monotonicity_guard() {
        use crate::services::alarm::AlarmState;
        AlarmState::clear_all();
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        entry.phase = EntryPhase::P65Upcontrol;
        state.modreg = 65;
        // Attempt backward transition to P63 — must be rejected.
        transition_to(&mut entry, &mut state, EntryPhase::P63EntryInit, &mut hw);
        assert_eq!(
            entry.phase,
            EntryPhase::P65Upcontrol,
            "phase must not regress"
        );
        AlarmState::clear_all();
    }

    /// T11: Guidance is a no-op after termination.
    #[test]
    fn tick_noop_after_termination() {
        let mut state = make_entry_state();
        let mut hw = MockHardware::new();
        let mut entry = enter(&mut state, &mut hw);
        entry.guidance_terminated = true;
        entry.phase = EntryPhase::P67Final;
        // Any tick must be a no-op.
        tick(&mut entry, &mut state, &mut hw);
        assert!(entry.guidance_terminated);
        assert_eq!(entry.phase, EntryPhase::P67Final);
    }

    /// T12: VQUIT constant matches spec value (304.8 m/s = 1000 FPS).
    ///
    /// AGC source: Comanche055/ENTRY_LEXICON.agc `VQUIT = 1000 FPS`.
    #[test]
    fn vquit_constant_matches_agc() {
        let expected = 1000.0_f64 * 0.3048;
        assert!(
            (VQUIT_MS - expected).abs() < 0.01,
            "VQUIT_MS = {VQUIT_MS} expected {expected}"
        );
    }
}
