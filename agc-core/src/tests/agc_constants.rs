//! AGC constant assertion tests.
//!
//! These tests lock Rust constants to their AGC Comanche055 source values.
//! If a constant is accidentally changed, `cargo test` catches it immediately.
//!
//! Reference: docs/agc-reference-constants.md

use crate::executive::{MAX_JOBS, MAX_WAITLIST_TASKS};
use crate::executive::restart::NUM_RESTART_GROUPS;
use crate::navigation::gravity::{J2_EARTH, MU_EARTH, MU_MOON, RE_EARTH};
use crate::services::average_g::{CYCLE_DT, PIPA_SCALE};

// ── Physical constants (ORBITAL_INTEGRATION.agc) ─────────────────────────────

#[test]
fn agc_mu_earth() {
    // ORBITAL_INTEGRATION.agc: MUEARTH 3.986032 E10 B-36 (scaled to m³/s²)
    assert_eq!(MU_EARTH, 3.986_032e14);
}

#[test]
fn agc_mu_moon() {
    // ORBITAL_INTEGRATION.agc: approximately 4.9027780×10¹² m³/s²
    assert_eq!(MU_MOON, 4.902_778e12);
}

#[test]
fn agc_re_earth() {
    // ORBITAL_INTEGRATION.agc: RSPHERE ~6373.338 km
    assert_eq!(RE_EARTH, 6_373_338.0);
}

#[test]
fn agc_j2_earth() {
    // ORBITAL_INTEGRATION.agc: J2 term coefficient
    assert_eq!(J2_EARTH, 1.082_626_68e-3);
}

// ── SERVICER constants (SERVICER207.agc) ─────────────────────────────────────

#[test]
fn agc_pipa_scale_kpip1() {
    // SERVICER207.agc: KPIP1 2DEC 0.074880 # 1 PULSE = 5.85 CM/SEC
    // 5.85 cm/s = 0.0585 m/s
    assert_eq!(PIPA_SCALE, 0.0585);
}

#[test]
fn agc_servicer_cycle_dt() {
    // SERVICER207.agc: CAF 2SECS; TC WAITLIST; 2CADR READACCS
    // "recur every 2 seconds"
    assert_eq!(CYCLE_DT, 2.0);
}

// ── Scheduler constants ─────────────────────────────────────────────────────

#[test]
fn agc_max_jobs_core_set() {
    // EXECUTIVE.agc: 7-slot CORE SET table
    assert_eq!(MAX_JOBS, 7);
}

#[test]
fn agc_max_waitlist_tasks() {
    // WAITLIST.agc: "9 TASKS MAXIMUM"; LST2 ERASE +17D
    assert_eq!(MAX_WAITLIST_TASKS, 9);
}

#[test]
fn agc_num_restart_groups() {
    // FRESH_START_AND_RESTART.agc: NUMGRPS EQUALS FIVE
    assert_eq!(NUM_RESTART_GROUPS, 5);
}

// ── DSKY key codes (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc) ─────────────────────

#[test]
fn agc_dsky_key_codes() {
    use crate::hal::dsky::DskyKey;

    // Octal table from PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
    assert_eq!(DskyKey::Verb as u8, 0o21);    // OCT 21
    assert_eq!(DskyKey::Noun as u8, 0o37);    // OCT 37
    assert_eq!(DskyKey::Enter as u8, 0o34);   // OCT 34
    assert_eq!(DskyKey::Clear as u8, 0o36);   // OCT 36
    assert_eq!(DskyKey::Reset as u8, 0o22);   // OCT 22
    assert_eq!(DskyKey::KeyRel as u8, 0o31);  // OCT 31
    assert_eq!(DskyKey::Plus as u8, 0o32);    // OCT 32
    assert_eq!(DskyKey::Minus as u8, 0o33);   // OCT 33
    assert_eq!(DskyKey::Zero as u8, 0o20);    // OCT 20
    assert_eq!(DskyKey::One as u8, 0o01);     // OCT 01
    assert_eq!(DskyKey::Two as u8, 0o02);     // OCT 02
    assert_eq!(DskyKey::Three as u8, 0o03);   // OCT 03
    assert_eq!(DskyKey::Four as u8, 0o04);    // OCT 04
    assert_eq!(DskyKey::Five as u8, 0o05);    // OCT 05
    assert_eq!(DskyKey::Six as u8, 0o06);     // OCT 06
    assert_eq!(DskyKey::Seven as u8, 0o07);   // OCT 07
    assert_eq!(DskyKey::Eight as u8, 0o10);   // OCT 10
    assert_eq!(DskyKey::Nine as u8, 0o11);    // OCT 11
}

// ── Alarm codes (ALARM_AND_ABORT.agc) ────────────────────────────────────────

#[test]
fn agc_alarm_codes() {
    use crate::services::alarm::AlarmCode;

    assert_eq!(AlarmCode::ExecutiveOverflow as u16, 1202);
    assert_eq!(AlarmCode::WaitlistOverflow as u16, 1210);
    assert_eq!(AlarmCode::ErasableChecksum as u16, 1211);
}
