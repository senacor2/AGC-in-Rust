//! DSKY indicator-lamp driver pass.
//!
//! `refresh_lamps` is invoked from `t4rupt_step` (and from the bare-metal
//! `Executive::run` T4 handler) before `decode_dsky`. It refreshes every
//! lamp boolean in `state.dsky` so the DSKY frame emitted to hardware
//! reflects the AGC's current internal condition rather than the stale
//! defaults left in place by FRESH START.
//!
//! Driving conditions (per #137):
//!
//! | Lamp              | Source                                                       |
//! |-------------------|--------------------------------------------------------------|
//! | `comp_acty`       | ≥1 PINBALL or Waitlist tick since the previous refresh       |
//! | `no_att`          | `state.imu_alignment_state != ImuAlignmentState::FineAligned`|
//! | `gimbal_lock`     | `imu_control::is_gimbal_lock_critical(&state.current_cdu)`   |
//! | `key_rel`         | V/N processor is mid-load (multi-keystroke entry in progress)|
//! | `prog_alarm`      | `state.alarm.lit`                                            |
//! | `tracker`         | any nav-mark loop (P20/P22/P23) is awaiting a sextant MARK   |
//! | `restart_flag`    | latched by `fresh_start::restart`, cleared by V37 dispatch   |
//! | `temp`            | deferred — no temperature HAL in agc-sim                     |
//! | `stby`            | set by P06, cleared by V37 transition to non-P06             |
//! | `uplink_activity` | maintained by `services::uplink::poll_uplink`                |
//!
//! `note_pinball_activity` is called from the V/N processor (`feed_key`)
//! and from each Waitlist task dispatch site (bare-metal scheduler and
//! `agc-sim`'s `WaitlistPump`) so `comp_acty` can latch a one-T4 window
//! after any scheduler activity.

use crate::control::imu_control::{is_gimbal_lock_critical, ImuAlignmentState};
use crate::services::v_n::VnPhase;
use crate::AgcState;

/// Record one PINBALL/Waitlist tick. Called from `feed_key` and from
/// each Waitlist dispatch site. Wraps on overflow.
#[inline]
pub fn note_pinball_activity(state: &mut AgcState) {
    state.pinball_ticks = state.pinball_ticks.wrapping_add(1);
}

/// Refresh every lamp boolean in `state.dsky` to reflect current
/// internal AGC state.
///
/// The original issue suggested a `(state, hw)` signature so a future
/// `temp` lamp driver could query a temperature HAL — that hardware
/// surface does not exist in agc-sim today, so the parameter is omitted
/// and will be re-added when `temp` becomes a real driver.
pub fn refresh_lamps(state: &mut AgcState) {
    // comp_acty: latch true for one T4 window after any PINBALL/Waitlist tick.
    let ticks = state.pinball_ticks;
    state.dsky.comp_acty = ticks != state.dsky.last_pinball_ticks_seen;
    state.dsky.last_pinball_ticks_seen = ticks;

    // no_att: IMU is not fine-aligned (either caged or coarse-aligned).
    state.dsky.no_att = state.imu_alignment_state != ImuAlignmentState::FineAligned;

    // gimbal_lock: middle gimbal is within 5° of ±90°.
    state.dsky.gimbal_lock = is_gimbal_lock_critical(&state.current_cdu);

    // key_rel: a multi-keystroke load is awaiting input. Verb/Noun entry
    // and OprErr do not count — those are normal cue states, not "the
    // system needs the DSKY but the crew has it tied up".
    state.dsky.key_rel = matches!(
        state.vn.phase,
        VnPhase::EnteringData { .. }
            | VnPhase::EnteringMajorMode { .. }
            | VnPhase::P27Address { .. }
            | VnPhase::P27Count { .. }
            | VnPhase::P27Data { .. }
            | VnPhase::P27SingleAddress { .. }
            | VnPhase::P27SingleData { .. }
            | VnPhase::P27Time { .. }
    );

    // prog_alarm: PROG alarm lamp follows the alarm state machine.
    state.dsky.prog_alarm = state.alarm.lit;

    // tracker: lit while a nav-mark loop is active (P20 rendezvous, P22
    // landmark, P23 cislunar). The real AGC's TRACKER lamp watched the
    // optics MARK window directly; here we use the nav-tracking flag as
    // the best available proxy until an explicit mark-window state exists.
    state.dsky.tracker =
        state.rendezvous_nav.tracking_active || state.csm_nav.tracking_active;

    // temp: deferred — no temperature HAL surface exists today.
    // stby / restart_flag / uplink_activity: owned by their setters
    // (P06, fresh_start::restart + V37 dispatch, uplink poll).
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::CduAngle;

    /// TC-LAMPS-COMPACTY-1: with no scheduler activity, comp_acty stays off
    /// across consecutive refreshes.
    #[test]
    fn tc_lamps_compacty_1_no_activity() {
        let mut state = AgcState::new();
        refresh_lamps(&mut state);
        assert!(!state.dsky.comp_acty);
        refresh_lamps(&mut state);
        assert!(!state.dsky.comp_acty);
    }

    /// TC-LAMPS-COMPACTY-2: a single activity tick latches comp_acty on for
    /// exactly one refresh window, then clears.
    #[test]
    fn tc_lamps_compacty_2_latches_one_window() {
        let mut state = AgcState::new();

        note_pinball_activity(&mut state);
        refresh_lamps(&mut state);
        assert!(state.dsky.comp_acty, "tick should latch comp_acty on");

        refresh_lamps(&mut state);
        assert!(!state.dsky.comp_acty, "comp_acty should clear after one window");
    }

    /// TC-LAMPS-COMPACTY-3: multiple ticks within a window collapse to one
    /// latched window; activity in the next window re-latches.
    #[test]
    fn tc_lamps_compacty_3_relatches_on_new_activity() {
        let mut state = AgcState::new();

        note_pinball_activity(&mut state);
        note_pinball_activity(&mut state);
        refresh_lamps(&mut state);
        assert!(state.dsky.comp_acty);

        refresh_lamps(&mut state);
        assert!(!state.dsky.comp_acty);

        note_pinball_activity(&mut state);
        refresh_lamps(&mut state);
        assert!(state.dsky.comp_acty, "new activity should re-latch");
    }

    /// TC-LAMPS-NOATT-1: no_att lit when IMU is Caged or CoarseAligned.
    #[test]
    fn tc_lamps_noatt_1_unaligned() {
        let mut state = AgcState::new();

        state.imu_alignment_state = ImuAlignmentState::Caged;
        refresh_lamps(&mut state);
        assert!(state.dsky.no_att, "Caged → NO ATT");

        state.imu_alignment_state = ImuAlignmentState::CoarseAligned;
        refresh_lamps(&mut state);
        assert!(state.dsky.no_att, "CoarseAligned → NO ATT");
    }

    /// TC-LAMPS-NOATT-2: no_att off when IMU is fine-aligned.
    #[test]
    fn tc_lamps_noatt_2_fine_aligned() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::FineAligned;
        refresh_lamps(&mut state);
        assert!(!state.dsky.no_att);
    }

    /// TC-LAMPS-GIMBAL-1: gimbal_lock follows the critical-band predicate
    /// on the middle (CDUZ) gimbal angle.
    #[test]
    fn tc_lamps_gimbal_1_critical_band() {
        let mut state = AgcState::new();

        state.current_cdu = [CduAngle(0); 3];
        refresh_lamps(&mut state);
        assert!(!state.dsky.gimbal_lock);

        state.current_cdu[2] = CduAngle(16384); // +90°
        refresh_lamps(&mut state);
        assert!(state.dsky.gimbal_lock);
    }

    /// TC-LAMPS-KEYREL-1: key_rel lit only when a multi-keystroke load is
    /// awaiting input. Idle, OprErr, EnteringVerb, EnteringNoun do not
    /// raise key_rel — those are normal entry cues.
    #[test]
    fn tc_lamps_keyrel_1_load_phases() {
        let mut state = AgcState::new();

        refresh_lamps(&mut state);
        assert!(!state.dsky.key_rel);

        state.vn.phase = VnPhase::EnteringMajorMode { digits: 1, buf: 4 };
        refresh_lamps(&mut state);
        assert!(state.dsky.key_rel, "V37 MM entry should light KEY REL");

        state.vn.phase = VnPhase::EnteringData {
            verb: 21,
            noun: 33,
            reg_index: 0,
            total_regs: 3,
            sign: 1,
            digits: 0,
            buf: 0,
            committed: [0.0; 3],
        };
        refresh_lamps(&mut state);
        assert!(state.dsky.key_rel, "V25 data entry should light KEY REL");

        state.vn.phase = VnPhase::Idle;
        refresh_lamps(&mut state);
        assert!(!state.dsky.key_rel);
    }

    /// TC-LAMPS-KEYREL-2: EnteringVerb and EnteringNoun are normal entry
    /// cues — they must NOT raise key_rel.
    #[test]
    fn tc_lamps_keyrel_2_verb_noun_entry_skipped() {
        let mut state = AgcState::new();

        state.vn.phase = VnPhase::EnteringVerb { digits: 1, buf: 3 };
        refresh_lamps(&mut state);
        assert!(!state.dsky.key_rel, "EnteringVerb must not raise KEY REL");

        state.vn.phase = VnPhase::EnteringNoun {
            verb: 6,
            digits: 1,
            buf: 4,
        };
        refresh_lamps(&mut state);
        assert!(!state.dsky.key_rel, "EnteringNoun must not raise KEY REL");

        state.vn.phase = VnPhase::OprErr;
        refresh_lamps(&mut state);
        assert!(!state.dsky.key_rel, "OprErr must not raise KEY REL");
    }

    /// TC-LAMPS-PROGALARM-1: prog_alarm mirrors `state.alarm.lit`.
    #[test]
    fn tc_lamps_progalarm_1_follows_alarm_lit() {
        let mut state = AgcState::new();

        refresh_lamps(&mut state);
        assert!(!state.dsky.prog_alarm);

        state.alarm.raise(0o1202, crate::tables::alarm_codes::SITE_EXECUTIVE);
        refresh_lamps(&mut state);
        assert!(state.dsky.prog_alarm);

        state.alarm.reset();
        refresh_lamps(&mut state);
        assert!(!state.dsky.prog_alarm);
    }

    /// TC-LAMPS-TRACKER-1: tracker lit while any nav-mark loop is active.
    #[test]
    fn tc_lamps_tracker_1_nav_tracking_active() {
        let mut state = AgcState::new();

        refresh_lamps(&mut state);
        assert!(!state.dsky.tracker);

        state.csm_nav.tracking_active = true;
        refresh_lamps(&mut state);
        assert!(state.dsky.tracker, "P22/P23 active → TRACKER lit");

        state.csm_nav.tracking_active = false;
        state.rendezvous_nav.tracking_active = true;
        refresh_lamps(&mut state);
        assert!(state.dsky.tracker, "P20 active → TRACKER lit");

        state.rendezvous_nav.tracking_active = false;
        refresh_lamps(&mut state);
        assert!(!state.dsky.tracker);
    }

    /// TC-LAMPS-RESTART-OWNED: refresh_lamps does NOT touch restart_flag —
    /// it is owned by fresh_start::restart (sets) and V37 dispatch (clears).
    #[test]
    fn tc_lamps_restart_owned_externally() {
        let mut state = AgcState::new();
        state.dsky.restart_flag = true;
        refresh_lamps(&mut state);
        assert!(state.dsky.restart_flag, "refresh_lamps must not clear restart_flag");
    }

    /// TC-LAMPS-STBY-OWNED: refresh_lamps does NOT touch stby — P06 sets it
    /// and the V37 dispatch in `services::v_n` clears it.
    #[test]
    fn tc_lamps_stby_owned_externally() {
        let mut state = AgcState::new();
        state.dsky.stby = true;
        refresh_lamps(&mut state);
        assert!(state.dsky.stby, "refresh_lamps must not clear stby");
    }

    /// TC-LAMPS-UPLINK-PRESERVED: uplink_activity is owned by the uplink
    /// poll path — refresh_lamps must not clear it.
    #[test]
    fn tc_lamps_uplink_preserved() {
        let mut state = AgcState::new();
        state.dsky.uplink_activity = true;
        refresh_lamps(&mut state);
        assert!(state.dsky.uplink_activity);
    }
}
