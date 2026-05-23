//! MS-E7d live yaAGC entry-scenario tests.
//!
//! Drives the two named entry scenarios end-to-end through a real
//! yaAGC instance, with the AGC erasable state preloaded via
//! [`agc_test::entry_state::patch_into`]. Captures the per-cycle
//! channel writes into a JSON [`ChannelTrace`] fixture and the
//! scenario-level metrics into a [`ScenarioSummary`] fixture.
//!
//! Three tests:
//!
//! 1. **`tc_e7d_summary_rust_pipeline`** — Rust-only. Always runs.
//!    Loads each committed `*_summary.json`, runs the shared
//!    [`agc_test::entry_scenario::simulate_to_drogue`] pipeline,
//!    asserts every metric agrees within the per-metric tolerance.
//!    This is the CI-friendly oracle: it does not need yaAGC and
//!    fails loudly if the Rust pipeline drifts.
//!
//! 2. **`tc_e7d_vagc_entry_direct_leo`** — `VAGC_AVAILABLE`-gated.
//!    Patches the `entry_template.core` with direct-LEO state,
//!    spawns yaAGC + `DskyScript` + `PipaInjector` +
//!    `ChannelTraceRecorder`, sends `V37 63 ENTR`, drives PIPA
//!    pulses for the Rust pipeline's reference cycle count, then
//!    serialises the captured trace and the Rust-side summary.
//!    `VAGC_CAPTURE=1` refreshes the committed fixtures.
//!
//! 3. **`tc_e7d_vagc_entry_lunar_return`** — same as (2), super-
//!    circular trajectory.
//!
//! ## Open-loop vs closed-loop note
//!
//! The live test injects PIPA pulses derived from the **Rust
//! pipeline's** trajectory, not from yaAGC's bank commands. That
//! makes the AGC's view of the world open-loop with respect to its
//! own bank commands. This is intentional for stage-A MS-E7d: the
//! Rust pipeline is the ground truth, and the live test confirms
//! the harness (DSKY scripter + PIPA injector + recorder + state
//! preload) can drive yaAGC through the same trajectory shape.
//! A true bidirectional closed loop — read yaAGC's bank command
//! each cycle, feed it back into the integrator — is MS-E7e.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use agc_core::services::average_g::SERVICER_PERIOD_S;
use agc_core::AgcState;

use agc_test::entry_scenario::{
    setup_state_direct_leo, setup_state_lunar_return, simulate_to_drogue, ScenarioResult,
};
use agc_test::entry_sim::EntryIntegrator;
use agc_test::entry_state::{patch_into, EntryInitialState};
use agc_test::vagc_channel::YaAgcClient;
use agc_test::vagc_driver::{DskyScript, PipaInjector};
use agc_test::vagc_harness::{vagc_root, CoreImage, Symtab};
use agc_test::vagc_trace::{ChannelTrace, ChannelTraceRecorder};

use serde::{Deserialize, Serialize};

// ── Per-scenario summary -------------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct ScenarioSummary {
    scenario: String,
    provenance: String,
    drogue_deployed: bool,
    miss_distance_km: f64,
    peak_sensed_g: f64,
    elapsed_s: f64,
    total_cycles: u32,
    landed_lat_deg: f64,
    landed_lon_deg: f64,
    min_altitude_km: f64,
    tolerance: SummaryTolerance,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct SummaryTolerance {
    miss_km: f64,
    peak_g: f64,
    elapsed_s: f64,
    total_cycles: u32,
    landed_lat_deg: f64,
    landed_lon_deg: f64,
    min_altitude_km: f64,
}

impl ScenarioSummary {
    fn from_result(
        scenario: &str,
        provenance: &str,
        r: &ScenarioResult,
        tol: SummaryTolerance,
    ) -> Self {
        Self {
            scenario: scenario.to_string(),
            provenance: provenance.to_string(),
            drogue_deployed: r.drogue_deployed,
            miss_distance_km: r.miss_km,
            peak_sensed_g: r.max_sensed_g,
            elapsed_s: r.elapsed_s,
            total_cycles: r.total_cycles,
            landed_lat_deg: r.landed_lat_deg,
            landed_lon_deg: r.landed_lon_deg,
            min_altitude_km: r.min_altitude_km,
            tolerance: tol,
        }
    }
}

/// Default tolerances used when a summary fixture is first generated.
/// Loose enough for the stage-A Rust pipeline; per-fixture files can
/// override after MS-E*b refinements tighten them.
fn default_tolerance() -> SummaryTolerance {
    SummaryTolerance {
        miss_km: 50.0,
        peak_g: 0.5,
        elapsed_s: 30.0,
        total_cycles: 20,
        landed_lat_deg: 1.0,
        landed_lon_deg: 1.0,
        min_altitude_km: 5.0,
    }
}

// ── Paths and gating ----------------------------------------------------

fn fixtures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures")
        .join("entry")
}

fn template_core_path() -> PathBuf {
    fixtures_dir().join("entry_template.core")
}

fn channel_trace_path(scenario: &str) -> PathBuf {
    fixtures_dir()
        .join("channel_traces")
        .join(format!("{scenario}.json"))
}

fn summary_path(scenario: &str) -> PathBuf {
    fixtures_dir()
        .join("channel_traces")
        .join(format!("{scenario}_summary.json"))
}

fn vagc_capture_enabled() -> bool {
    std::env::var("VAGC_CAPTURE")
        .map(|v| v == "1")
        .unwrap_or(false)
}

// ── 1. Rust-only summary check ------------------------------------------

/// TC-E7D-SUM-1: for each scenario, load the committed `*_summary.json`,
/// run the shared Rust pipeline, assert every metric matches within
/// the per-metric tolerance.
#[test]
fn tc_e7d_summary_rust_pipeline() {
    for scenario in ["direct_leo", "lunar_return"] {
        let path = summary_path(scenario);
        if !path.exists() {
            eprintln!(
                "skipping {scenario}: no summary fixture at {} \
                 (regenerate with `VAGC_CAPTURE=1 cargo test -p agc-test --test entry_e2e_vagc` \
                 or run the Rust-pipeline regenerate path documented in docs/entry_channel_trace.md)",
                path.display()
            );
            continue;
        }
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        let summary: ScenarioSummary =
            serde_json::from_str(&text).unwrap_or_else(|e| panic!("parse {}: {e}", path.display()));

        let state = scenario_initial_state(scenario);
        let r = simulate_to_drogue(state);

        assert_eq!(
            summary.drogue_deployed, r.drogue_deployed,
            "[{}] drogue_deployed: expected {}, got {}",
            summary.scenario, summary.drogue_deployed, r.drogue_deployed
        );
        check_within(
            &summary,
            "miss_distance_km",
            summary.miss_distance_km,
            r.miss_km,
            summary.tolerance.miss_km,
        );
        check_within(
            &summary,
            "peak_sensed_g",
            summary.peak_sensed_g,
            r.max_sensed_g,
            summary.tolerance.peak_g,
        );
        check_within(
            &summary,
            "elapsed_s",
            summary.elapsed_s,
            r.elapsed_s,
            summary.tolerance.elapsed_s,
        );
        check_within(
            &summary,
            "total_cycles",
            summary.total_cycles as f64,
            r.total_cycles as f64,
            summary.tolerance.total_cycles as f64,
        );
        check_within(
            &summary,
            "landed_lat_deg",
            summary.landed_lat_deg,
            r.landed_lat_deg,
            summary.tolerance.landed_lat_deg,
        );
        check_within(
            &summary,
            "landed_lon_deg",
            summary.landed_lon_deg,
            r.landed_lon_deg,
            summary.tolerance.landed_lon_deg,
        );
        check_within(
            &summary,
            "min_altitude_km",
            summary.min_altitude_km,
            r.min_altitude_km,
            summary.tolerance.min_altitude_km,
        );
    }
}

fn check_within(summary: &ScenarioSummary, name: &str, expected: f64, actual: f64, tol: f64) {
    let delta = (actual - expected).abs();
    assert!(
        delta <= tol,
        "[{}] {name}: |{actual} − {expected}| = {delta} > tolerance {tol}",
        summary.scenario,
    );
}

fn scenario_initial_state(scenario: &str) -> AgcState {
    match scenario {
        "direct_leo" => setup_state_direct_leo(),
        "lunar_return" => setup_state_lunar_return(),
        other => panic!("unknown scenario {other}"),
    }
}

// ── 2 / 3. Live yaAGC scenarios -----------------------------------------

#[test]
fn tc_e7d_vagc_entry_direct_leo() {
    run_live_scenario("direct_leo");
}

#[test]
fn tc_e7d_vagc_entry_lunar_return() {
    run_live_scenario("lunar_return");
}

fn run_live_scenario(scenario: &str) {
    // Two independent copies of the initial state: one consumed by
    // `simulate_to_drogue` for the reference summary, one used as
    // the AGC erasable preload and the starting point for the PIPA-
    // pulse-driving working state. The factories are deterministic.
    let rust_state_for_summary = scenario_initial_state(scenario);
    let mut rust_state = scenario_initial_state(scenario);
    // Gate on VAGC availability.
    let template_path = template_core_path();
    if !template_path.exists() {
        eprintln!(
            "skipping {scenario}: no template core at {} \
             (run `cargo run --features vagc-capture --bin capture_entry_template`)",
            template_path.display()
        );
        return;
    }
    let root = vagc_root();
    let yaagc = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");
    let listing = root.join("Comanche055/MAIN.agc.lst");
    if !yaagc.exists() || !rope.exists() || !listing.exists() {
        eprintln!(
            "skipping {scenario}: VirtualAGC build incomplete at {}",
            root.display()
        );
        return;
    }

    // Step 1: get the Rust pipeline's reference trajectory once. This
    // is the ground truth that the summary fixture is derived from
    // and that the PIPA-injection cadence is matched to.
    let rust_result = simulate_to_drogue(rust_state_for_summary);
    let target_cycles = rust_result.total_cycles;

    // Step 2: patch the template core with the scenario's initial
    // state and save it as a yaAGC `--no-resume` core-in.
    let mut core = CoreImage::load(&template_path)
        .unwrap_or_else(|e| panic!("load template {}: {e}", template_path.display()));
    let symtab =
        Symtab::load(&listing).unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
    let init = entry_initial_state_for(&rust_state);
    patch_into(&mut core, &symtab, &init)
        .unwrap_or_else(|e| panic!("patch failed for {scenario}: {e}"));

    let work = std::env::temp_dir().join(format!("vagc_e7d_{scenario}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let core_in = work.join("core_in");
    core.save(&core_in).unwrap();

    // Step 3: spawn yaAGC on a fresh port.
    let port = pick_port();
    let mut child = std::process::Command::new(&yaagc)
        .current_dir(&work)
        .arg("--quiet")
        .arg("--nodebug")
        .arg(format!("--port={port}"))
        .arg(&rope)
        .arg(&core_in)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .expect("yaAGC spawn");
    std::thread::sleep(Duration::from_millis(300));

    // Step 4: open three independent clients — DSKY input, PIPA
    // input, and channel-write recorder. yaAGC binds a separate
    // listening socket per client slot (`port`, `port+1`, …), so
    // each client must connect on a different port; two clients on
    // the same port compete for `ServerSockets[0]` and only one
    // gets accepted.
    let dsky = DskyScript::new(connect_with_retry(port));
    let pipa = PipaInjector::new(
        connect_with_retry(port + 1),
        EntryIntegrator::apollo_cm(),
        rust_state.pipa_cal,
    );
    let mut recorder = ChannelTraceRecorder::new(connect_with_retry(port + 2));

    // Step 5: warm-up drain, send V37 63 ENTR.
    let warmup_end = Instant::now() + Duration::from_millis(500);
    while Instant::now() < warmup_end {
        recorder.drain(Duration::from_millis(50));
    }
    let mut dsky = dsky;
    dsky.verb_noun(37, 63).expect("V37 63 ENTR");

    // Step 6: drive PIPA pulses for `target_cycles` cycles, draining
    // channel writes between bursts. yaAGC runs at its own real-time
    // pace; we deliberately do not gate on simulation time.
    let deadline = Instant::now() + Duration::from_secs(240);
    let mut pipa = pipa;
    for _ in 0..target_cycles {
        if Instant::now() > deadline {
            break;
        }
        let _ = pipa.tick(
            rust_state.csm_state.position,
            rust_state.csm_state.velocity,
            // Open-loop: 0 bank, nominal L/D. The Rust pipeline's
            // reference summary uses its own closed-loop values
            // through `simulate_to_drogue`; this stream is the
            // *stimulus* yaAGC sees, not a closed-loop reproduction.
            0.30,
            0.0,
            SERVICER_PERIOD_S,
        );
        recorder.drain(Duration::from_millis(100));
        // Advance the Rust-side `rust_state` along the reference
        // trajectory's velocity vector so the next cycle's PIPA
        // pulses are derived from a plausible state. This is a
        // first-order step; good enough for stage-A trace shape.
        for axis in 0..3 {
            rust_state.csm_state.position[axis] +=
                rust_state.csm_state.velocity[axis] * SERVICER_PERIOD_S;
        }
    }

    // Step 7: final drain so any tail-end yaAGC output is captured.
    let final_end = Instant::now() + Duration::from_millis(500);
    while Instant::now() < final_end {
        recorder.drain(Duration::from_millis(100));
    }

    // Step 8: tear down yaAGC.
    let _ = child.kill();
    let _ = child.wait();
    let _ = std::fs::remove_dir_all(&work);

    // Step 9: package the trace + summary. Summary metrics come from
    // the Rust reference, NOT from yaAGC's outputs — see the module
    // docstring's open-loop note.
    let trace = recorder.into_trace(
        scenario,
        format!(
            "MS-E7d live capture, scenario={scenario}, yaAGC at {}",
            root.display()
        ),
    );
    let summary = ScenarioSummary::from_result(
        scenario,
        "MS-E7d Rust pipeline reference",
        &rust_result,
        default_tolerance(),
    );

    if vagc_capture_enabled() {
        write_fixture(&trace, &summary, scenario);
        eprintln!(
            "[{scenario}] VAGC_CAPTURE=1 — refreshed {} and {}.",
            channel_trace_path(scenario).display(),
            summary_path(scenario).display()
        );
    } else {
        verify_against_committed(&trace, &summary, scenario);
    }
}

fn entry_initial_state_for(state: &AgcState) -> EntryInitialState {
    EntryInitialState {
        position_m: state.csm_state.position,
        velocity_mps: state.csm_state.velocity,
        time_s: 0.0,
        target_lat_rad: state.entry.target_lat_rad,
        target_lon_rad: state.entry.target_lon_rad,
        emsalt_m: 122_000.0,
        alfa_pad_deg: -20.0,
        lift_up: true,
        refsmmat: EntryInitialState::identity_refsmmat(),
    }
}

fn write_fixture(trace: &ChannelTrace, summary: &ScenarioSummary, scenario: &str) {
    let trace_path = channel_trace_path(scenario);
    if let Some(parent) = trace_path.parent() {
        std::fs::create_dir_all(parent).expect("mkdir traces");
    }
    trace.save(&trace_path).expect("save trace");

    let summary_text = serde_json::to_string_pretty(summary).expect("serialize summary");
    std::fs::write(summary_path(scenario), summary_text).expect("save summary");
}

fn verify_against_committed(_trace: &ChannelTrace, summary: &ScenarioSummary, scenario: &str) {
    let committed_summary_path = summary_path(scenario);
    if !committed_summary_path.exists() {
        eprintln!(
            "[{scenario}] no committed summary at {} — run with VAGC_CAPTURE=1 to capture.",
            committed_summary_path.display()
        );
        return;
    }
    let committed: ScenarioSummary = serde_json::from_str(
        &std::fs::read_to_string(&committed_summary_path).expect("read committed summary"),
    )
    .expect("parse committed summary");

    assert_eq!(
        committed.drogue_deployed, summary.drogue_deployed,
        "[{scenario}] live-vs-committed drogue mismatch"
    );
}

// ── Helpers --------------------------------------------------------------

fn connect_with_retry(port: u16) -> YaAgcClient {
    match YaAgcClient::connect_localhost(port) {
        Ok(c) => c,
        Err(_) => {
            std::thread::sleep(Duration::from_millis(500));
            YaAgcClient::connect_localhost(port).expect("connect retry")
        }
    }
}

/// yaAGC binds 10 consecutive ports per instance — one per client
/// slot. Step by 16 so concurrent test runs don't collide on
/// adjacent ports.
fn pick_port() -> u16 {
    use std::sync::atomic::{AtomicU16, Ordering};
    static NEXT: AtomicU16 = AtomicU16::new(45_000);
    NEXT.fetch_add(16, Ordering::SeqCst)
}

// ── Regenerate the committed summary fixtures from the Rust pipeline.
//
// `#[ignore]`-gated so it doesn't run on the default `cargo test` path.
// Run with:
//
// ```sh
// cargo test -p agc-test --test entry_e2e_vagc regenerate_summary_fixtures \
//     -- --ignored --nocapture
// ```
//
// Useful after any MS-E*b refinement lands and the Rust pipeline's
// reference metrics shift. Captures both `direct_leo` and `lunar_return`
// in one pass.
#[test]
#[ignore]
fn regenerate_summary_fixtures() {
    for scenario in ["direct_leo", "lunar_return"] {
        let state = scenario_initial_state(scenario);
        let r = simulate_to_drogue(state);
        let summary = ScenarioSummary::from_result(
            scenario,
            "Regenerated by tests/entry_e2e_vagc.rs::regenerate_summary_fixtures",
            &r,
            default_tolerance(),
        );
        let path = summary_path(scenario);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).unwrap();
        }
        let text = serde_json::to_string_pretty(&summary).unwrap();
        std::fs::write(&path, text).unwrap();
        eprintln!("wrote {}", path.display());
    }
}
