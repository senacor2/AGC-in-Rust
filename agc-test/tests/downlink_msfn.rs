// SPDX-License-Identifier: GPL-3.0-or-later
//! MSFN downlink format tests — compares the Rust CMCSTADL encoder output
//! against the analytically-derived `downlink_fresh_start.json` fixture.
//!
//! ## Fixture provenance
//!
//! The fixture was computed analytically for a `AgcState::new()` (fresh-start)
//! state and committed alongside this test.  It encodes the MSFN word-pair
//! values that the Comanche055 DOWN-TELEMETRY program would produce for the
//! same state.
//!
//! A `capture_downlink` binary (`--features vagc-capture`) can regenerate the
//! fixture from a live yaAGC session, once the yaAGC hostname-resolution issue
//! on the development machine is resolved (see `fixtures/downlink_fresh_start.json`).

use agc_core::services::downlink::{build_cmcstadl, CMCSTADL_ID, LOWIDCOD};
use agc_core::AgcState;
use serde::Deserialize;

// ── Fixture types ─────────────────────────────────────────────────────────────

#[derive(Debug, Deserialize)]
struct PairExpectation {
    index: usize,
    w34: u16,
    w35: u16,
    #[allow(dead_code)]
    comment: Option<String>,
}

#[derive(Debug, Deserialize)]
struct DownlinkFixture {
    #[allow(dead_code)]
    description: String,
    pairs: Vec<PairExpectation>,
}

fn load_fixture() -> DownlinkFixture {
    let json = include_str!("../fixtures/downlink_fresh_start.json");
    serde_json::from_str(json).expect("fixture parse failed")
}

// ── Tests ─────────────────────────────────────────────────────────────────────

/// TC-MSFN-1: CMCSTADL_ID and LOWIDCOD constants match the Comanche055 spec.
///
/// LOWIDCOD = octal 77340 = decimal 32480 = 0x7EE0.
#[test]
fn tc_msfn_1_constants() {
    assert_eq!(LOWIDCOD, 0x7EE0, "LOWIDCOD must be octal 77340 = 0x7EE0");
    // CMCSTADL_ID must be a non-zero value distinct from LOWIDCOD.
    assert_ne!(CMCSTADL_ID, 0, "CMCSTADL_ID must be non-zero");
    assert_ne!(CMCSTADL_ID, LOWIDCOD, "ID and sync words must differ");
}

/// TC-MSFN-2: Fresh-start downlist matches the analytically-derived fixture.
///
/// Every pair listed in the fixture must appear at the correct buffer index
/// with matching word values.  Pairs not listed in the fixture are not checked
/// (they may be zero or carry implementation-specific values not covered by the
/// analytical computation).
#[test]
fn tc_msfn_2_fresh_start_matches_fixture() {
    let fixture = load_fixture();
    let state = AgcState::new();
    let buf = build_cmcstadl(&state);

    for exp in &fixture.pairs {
        let got_w34 = buf[2 * exp.index];
        let got_w35 = buf[2 * exp.index + 1];
        assert_eq!(
            got_w34, exp.w34,
            "pair {} channel-34: expected 0x{:04X}, got 0x{:04X}",
            exp.index, exp.w34, got_w34
        );
        assert_eq!(
            got_w35, exp.w35,
            "pair {} channel-35: expected 0x{:04X}, got 0x{:04X}",
            exp.index, exp.w35, got_w35
        );
    }
}

/// TC-MSFN-3: ID/sync pair (index 0) is (CMCSTADL_ID, LOWIDCOD).
#[test]
fn tc_msfn_3_id_sync_pair() {
    let state = AgcState::new();
    let buf = build_cmcstadl(&state);
    assert_eq!(buf[0], CMCSTADL_ID, "w34 of pair 0 must be CMCSTADL_ID");
    assert_eq!(buf[1], LOWIDCOD, "w35 of pair 0 must be LOWIDCOD");
}

/// TC-MSFN-4: TIME2/TIME1 at pair 50 reflect mission elapsed time.
#[test]
fn tc_msfn_4_time_pair() {
    use agc_core::types::Met;
    use agc_core::services::downlink::encode_time;

    let mut state = AgcState::new();
    state.time = Met(50_000); // 500 seconds

    let buf = build_cmcstadl(&state);
    let (expected_t2, expected_t1) = encode_time(50_000);

    assert_eq!(buf[2 * 50], expected_t2, "TIME2 must match encode_time high (pair 50)");
    assert_eq!(buf[2 * 50 + 1], expected_t1, "TIME1 must match encode_time low (pair 50)");
}

/// TC-MSFN-5: REFSMMAT identity diagonal encodes to near-full-scale positive.
///
/// REFSMMAT is at pairs 33–38. Diagonal M[0][0] (pair 33) and M[1][1] (pair 37)
/// both encode to (0x3FFF, 0x3FFE) in DP B-0 scale.
#[test]
fn tc_msfn_5_refsmmat_identity_diagonal() {
    let state = AgcState::new();
    let buf = build_cmcstadl(&state);

    let (h33, l33) = (buf[66], buf[67]); // pair 33 = M[0][0]
    assert_eq!(h33, 0x3FFF, "REFSMMAT[0][0] high must be 0x3FFF");
    assert_eq!(l33, 0x3FFE, "REFSMMAT[0][0] low must be 0x3FFE");

    let (h37, l37) = (buf[74], buf[75]); // pair 37 = M[1][1]
    assert_eq!(h37, 0x3FFF, "REFSMMAT[1][1] high must be 0x3FFF");
    assert_eq!(l37, 0x3FFE, "REFSMMAT[1][1] low must be 0x3FFE");
}

/// TC-MSFN-6: REFSMMAT off-diagonal elements (0.0) encode to zero.
#[test]
fn tc_msfn_6_refsmmat_offdiagonal_zero() {
    let state = AgcState::new();
    let buf = build_cmcstadl(&state);

    // Pairs 34, 35, 36, 38 are off-diagonal = 0.0
    for idx in [34, 35, 36, 38] {
        let (h, l) = (buf[2 * idx], buf[2 * idx + 1]);
        assert_eq!(h, 0, "REFSMMAT off-diagonal pair {idx} high must be 0");
        assert_eq!(l, 0, "REFSMMAT off-diagonal pair {idx} low must be 0");
    }
}

/// TC-MSFN-7: DownlinkDriver produces a full 100-pair cycle without panic.
///
/// Smoke-test that running 100 steps through the driver does not panic or
/// produce out-of-range word values.
#[test]
fn tc_msfn_7_driver_cycle_no_panic() {
    use agc_core::services::downlink::{downlink_step, DownlinkDriver};
    use agc_core::hal::Telemetry;

    struct Collector(Vec<u16>);
    impl Telemetry for Collector {
        fn send_word(&mut self, w: u16) {
            assert!(w <= 0x7FFF, "word 0x{w:04X} exceeds 15-bit range");
            self.0.push(w);
        }
    }

    let state = AgcState::new();
    let mut driver = DownlinkDriver::new();
    let mut col = Collector(Vec::new());

    for _ in 0..100 {
        downlink_step(&mut driver, &state, &mut col);
    }

    assert_eq!(col.0.len(), 200, "100 pairs × 2 words = 200 words");
    assert_eq!(driver.pair_index, 0, "index must reset after full cycle");
    // ID pair must be present at positions 0/1.
    assert_eq!(col.0[0], CMCSTADL_ID);
    assert_eq!(col.0[1], LOWIDCOD);
}
