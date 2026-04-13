// Integration tests for P61–P67 entry guidance sequence.
//
// Exercises the complete entry guidance state machine end-to-end using SimHardware.
// Tests cover: nominal lunar return entry sequence and terminal guidance at VQUIT.
//
// AGC source: Comanche055/P61-P67.agc P61-P67 guidance sequence (pages 789-818).
// AGC source: Comanche055/ENTRY_LEXICON.agc (constants and scale factors).

use agc_core::{
    navigation::constants::RE_EARTH,
    navigation::state_vector::StateVector,
    programs::p61_entry::{
        enter, tick, EntryPhase, ENTRY_INTERFACE_M, POINT_05G_THRESHOLD, POINT_2G_THRESHOLD,
        Q7F_MS2, RANGE_25NM_M, VFINAL1_MS, VLMIN_MS, VQUIT_MS,
    },
    types::Met,
    AgcState,
};
use agc_sim::SimHardware;

fn make_entry_agc_state() -> AgcState {
    // Vehicle at 50 km above entry interface, moving at entry speed.
    let r = RE_EARTH + ENTRY_INTERFACE_M + 50_000.0;
    let v = 10_000.0_f64; // ~Mach 30
    let sv = StateVector::new([r, 0.0, 0.0], [0.0, v, 0.0], Met(0));
    let mut state = AgcState::new();
    state.nav.sv = sv;
    state.flags.refsmflg = true;
    state
}

// ── TC-P61-IT-01: Full nominal entry — P61 → P62 → P63 → P64 → P67 ───────────

/// Drives the entry state machine through the full nominal sequence from
/// P61PreEntry to P67Final via P64 direct path (V < VFINAL1 at 0.2G).
///
/// AGC source: Comanche055/P61-P67.agc phases P61-P64-P67 (pages 789-800).
#[test]
fn nominal_entry_sequence_p61_to_p67_direct() {
    let mut state = make_entry_agc_state();
    let mut hw = SimHardware::new_headless();

    // Enter P61.
    let mut entry = enter(&mut state, &mut hw);
    assert_eq!(
        entry.phase,
        EntryPhase::P61PreEntry,
        "must start in P61PreEntry"
    );
    assert!(state.extvbact, "EXTVBACT must be set on P61 entry");
    assert_eq!(state.modreg, 61);

    // P61 → P62: crew PROCEED.
    hw.dsky.set_proceed(true);
    tick(&mut entry, &mut state, &mut hw);
    assert_eq!(entry.phase, EntryPhase::P62SepCM);
    assert_eq!(state.modreg, 62);

    // P62 → P63: attitude achieved (PROCEED in sim).
    tick(&mut entry, &mut state, &mut hw);
    assert_eq!(entry.phase, EntryPhase::P63EntryInit);
    assert_eq!(state.modreg, 63);

    // P63 → P64: 0.05G onset.
    entry.drag_acc_ms2 = POINT_05G_THRESHOLD + 0.01;
    hw.dsky.set_proceed(false); // clear proceed — no longer needed
    tick(&mut entry, &mut state, &mut hw);
    assert_eq!(entry.phase, EntryPhase::P64Post05G);
    assert!(state.flags.point_05gsw, ".05GSW flag must be set");
    assert_eq!(state.modreg, 64);

    // P64 → P67 direct: V < VFINAL1 at 0.2G.
    entry.vi_ms = VFINAL1_MS - 500.0; // below threshold
    entry.drag_acc_ms2 = POINT_2G_THRESHOLD + 0.01;
    tick(&mut entry, &mut state, &mut hw);
    assert_eq!(
        entry.phase,
        EntryPhase::P67Final,
        "must skip P65 and go direct to P67"
    );
    assert_eq!(state.modreg, 67);

    // P67 terminal at VQUIT.
    entry.vi_ms = VQUIT_MS - 10.0;
    tick(&mut entry, &mut state, &mut hw);
    assert!(
        entry.guidance_terminated,
        "guidance must terminate at VQUIT"
    );
    assert!(!state.extvbact, "EXTVBACT must be cleared on P67 terminal");
}

// ── TC-P61-IT-02: Entry via upcontrol — P64 → P65 → P66 → P67 ───────────────

/// When range < 25 NM and V > VFINAL1, P64 advances to P65 (upcontrol).
/// P65 then progresses to P66 (ballistic) when drag drops, then to P67.
///
/// AGC source: Comanche055/P61-P67.agc P65 UPCONTRL → P66 → P67 (pages 798-800).
#[test]
fn entry_via_upcontrol_path_p64_to_p67() {
    let mut state = make_entry_agc_state();
    let mut hw = SimHardware::new_headless();

    let mut entry = enter(&mut state, &mut hw);

    // Drive to P64 quickly.
    hw.dsky.set_proceed(true);
    tick(&mut entry, &mut state, &mut hw); // P61 → P62
    tick(&mut entry, &mut state, &mut hw); // P62 → P63
    entry.drag_acc_ms2 = POINT_05G_THRESHOLD + 0.1;
    hw.dsky.set_proceed(false);
    tick(&mut entry, &mut state, &mut hw); // P63 → P64
    assert_eq!(entry.phase, EntryPhase::P64Post05G);

    // Set up conditions for P64 → P65.
    entry.range_to_go_m = RANGE_25NM_M - 1_000.0; // < 25 NM
    entry.vi_ms = VFINAL1_MS + 500.0; // > VFINAL1
    entry.vl_ms = VLMIN_MS + 1_000.0; // > VLMIN (upcontrol exists)
    entry.drag_acc_ms2 = POINT_05G_THRESHOLD + 0.1; // below 0.2G (no direct-P67 trigger)
    tick(&mut entry, &mut state, &mut hw); // P64 → P65
    assert_eq!(entry.phase, EntryPhase::P65Upcontrol);
    assert_eq!(state.modreg, 65);

    // P65 → P66: drag drops below Q7.
    entry.drag_acc_ms2 = Q7F_MS2 * 0.8;
    entry.q7_ms2 = Q7F_MS2;
    entry.rdot_ms = 10.0; // positive RDOT — no P67 trigger yet
    tick(&mut entry, &mut state, &mut hw); // P65 → P66
    assert_eq!(entry.phase, EntryPhase::P66Ballistic);
    assert_eq!(state.modreg, 66);

    // P66 → P65: drag rebuilds (re-enters upcontrol).
    entry.drag_acc_ms2 = Q7F_MS2 + 0.5; // above Q7 + 0.5 FPSS threshold
    entry.vi_ms = VQUIT_MS + 500.0; // V > VQUIT — no P67 terminal
    tick(&mut entry, &mut state, &mut hw); // P66 → P65
    assert_eq!(entry.phase, EntryPhase::P65Upcontrol);

    // P65 → P67: RDOT < 0 and V < VL + 500.
    entry.rdot_ms = -5.0;
    entry.vi_ms = entry.vl_ms + 50.0; // V < VL + C18 (152.4)
    entry.drag_acc_ms2 = Q7F_MS2 * 2.0; // drag above Q7 (no P66 first)
    tick(&mut entry, &mut state, &mut hw); // P65 → P67
    assert_eq!(entry.phase, EntryPhase::P67Final);

    // P67 terminal.
    entry.vi_ms = VQUIT_MS / 2.0;
    tick(&mut entry, &mut state, &mut hw);
    assert!(entry.guidance_terminated);
}
