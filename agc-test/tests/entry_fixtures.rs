//! CI-visible tests that load JSON fixtures captured by the developer-
//! only `capture_*` binaries and validate them against agc-core's
//! entry-guidance implementation.
//!
//! These tests do NOT need yaAGC at run time — the JSON files are
//! committed and contain the captured AGC outputs. Re-capturing
//! requires running the appropriate `cargo run --features vagc-capture
//! --bin capture_<routine>` (see `agc-test/README.md`).
//!
//! ## Phase-3 scaffold scope
//!
//! The current `huntest_cases.json` captures a LEWD/DLEWD round-trip
//! through yaAGC's erasable memory without actually exercising HUNTEST
//! (that's gated on MS-E7b DSKY scripting). The test below therefore
//! verifies the **round-trip identity** — expected = inputs to within
//! the captured tolerance. When real HUNTEST captures land in MS-E3b,
//! the test logic will switch to driving `compute_ld_command` and
//! comparing its outputs to `expected`.

use std::collections::BTreeMap;

use serde::Deserialize;

const HUNTEST_FIXTURE_JSON: &str =
    include_str!("../fixtures/entry/huntest_cases.json");

#[derive(Debug, Deserialize)]
struct FixtureFile {
    #[allow(dead_code)]
    source: String,
    cases: Vec<FixtureCase>,
}

#[derive(Debug, Deserialize)]
struct FixtureCase {
    name: String,
    #[allow(dead_code)]
    description: String,
    inputs: BTreeMap<String, f64>,
    expected: BTreeMap<String, f64>,
    tolerance: BTreeMap<String, f64>,
}

/// TC-EFX-HUNTEST-ROUND-TRIP: every captured HUNTEST case satisfies the
/// "expected ≈ inputs" round-trip identity, since the Phase-3 scaffold
/// does not yet drive the AGC through HUNTEST.
///
/// When MS-E3b lands the real captures, this test name will be retired
/// and the assertion logic will switch to driving
/// `agc_core::guidance::entry::compute_ld_command` with `inputs` and
/// asserting its result matches `expected`.
#[test]
fn huntest_fixtures_round_trip() {
    let fixture: FixtureFile = serde_json::from_str(HUNTEST_FIXTURE_JSON)
        .expect("huntest_cases.json must deserialize");

    assert!(
        !fixture.cases.is_empty(),
        "huntest_cases.json must contain at least one case"
    );

    for case in &fixture.cases {
        for (var, input_value) in &case.inputs {
            let expected = case.expected.get(var).copied().unwrap_or_else(|| {
                panic!(
                    "case '{}' has input '{}' but no matching expected output — \
                     this scaffold test requires every input to be re-read as an \
                     output for the round-trip identity check",
                    case.name, var
                )
            });
            let tol = case.tolerance.get(var).copied().unwrap_or(1.0e-3);
            assert!(
                (expected - input_value).abs() < tol,
                "case '{}' variable '{}': expected ({}) and input ({}) differ by more than tolerance {}",
                case.name,
                var,
                expected,
                input_value,
                tol
            );
        }
    }
}
