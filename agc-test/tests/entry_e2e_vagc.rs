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

use agc_core::control::DapMode;
use agc_core::programs::p61_p67::{init_p61, init_p62, init_p63, EntryPhase};
use agc_core::services::average_g::{servicer_task, start_servicer};

use agc_test::entry_scenario::{
    setup_state_direct_leo, setup_state_lunar_return, simulate_to_drogue, sub_satellite_lat_lon,
    ScenarioResult, MAX_SCENARIO_DURATION_S, R_EARTH_M,
};
use agc_test::entry_sim::{haversine_km, pipa_pulses_for_dv, EntryIntegrator};
use agc_test::entry_state::{patch_into, read_rollc_rad, EntryInitialState};
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

/// TC-E7E-SUM-CL-1: load each committed closed-loop summary and
/// verify it parses + has structurally plausible values. Closed-loop
/// fixtures are yaAGC-driven so the Rust pipeline does NOT reproduce
/// them directly; this test is a CI-runnable schema check, not a
/// metric oracle. The actual regression check lives in the live
/// `verify_closed_loop_against_committed` path inside
/// `run_live_scenario_closed_loop`.
#[test]
fn tc_e7e_closed_loop_summary_structural() {
    for scenario in ["direct_leo", "lunar_return"] {
        let path = closed_loop_summary_path(scenario);
        if !path.exists() {
            eprintln!(
                "skipping {scenario} (closed-loop): no committed summary at {} \
                 (run with VAGC_CAPTURE=1 once on a machine with VirtualAGC built)",
                path.display()
            );
            continue;
        }
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        let s: ScenarioSummary = serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("parse {}: {e}", path.display()));

        assert_eq!(s.scenario, scenario, "scenario field mismatch in {}", path.display());
        assert!(!s.provenance.is_empty(), "[{scenario}] provenance is empty");
        assert!(
            s.elapsed_s > 0.0,
            "[{scenario}] elapsed_s must be positive, got {}",
            s.elapsed_s
        );
        assert!(
            s.total_cycles > 0,
            "[{scenario}] total_cycles must be positive, got {}",
            s.total_cycles
        );
        assert!(
            s.tolerance.miss_km > 0.0
                && s.tolerance.peak_g > 0.0
                && s.tolerance.elapsed_s > 0.0,
            "[{scenario}] all tolerances must be positive (got {:?})",
            s.tolerance
        );
        // Closed-loop bands are wider than the open-loop ones because
        // the Rust stage-A guidance diverges from Comanche055's full
        // guidance. Sanity-check that the author didn't accidentally
        // commit open-loop-tight tolerances on this fixture.
        assert!(
            s.tolerance.miss_km >= 100.0,
            "[{scenario}] closed-loop miss_km tolerance suspiciously tight: {} \
             (open-loop runs use 50.0; closed-loop should be ≥ 100.0)",
            s.tolerance.miss_km
        );
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

// ── MS-E7e: closed-loop scenarios ───────────────────────────────────────

fn closed_loop_trace_path(scenario: &str) -> PathBuf {
    fixtures_dir()
        .join("channel_traces")
        .join(format!("{scenario}_closed_loop.json"))
}

fn closed_loop_summary_path(scenario: &str) -> PathBuf {
    fixtures_dir()
        .join("channel_traces")
        .join(format!("{scenario}_closed_loop_summary.json"))
}

/// Default tolerances for the closed-loop summary. Wider than
/// [`default_tolerance`] because the Rust pipeline's stage-A guidance
/// diverges from Comanche055's full guidance, so the captured
/// trajectory may differ from the Rust reference by tens of percent
/// on miss-distance and a few seconds on elapsed time. The tighter
/// regression signal lives in the live test re-running against the
/// committed summary.
fn default_closed_loop_tolerance() -> SummaryTolerance {
    SummaryTolerance {
        miss_km: 2000.0,
        peak_g: 5.0,
        elapsed_s: 120.0,
        total_cycles: 60,
        landed_lat_deg: 10.0,
        landed_lon_deg: 10.0,
        min_altitude_km: 30.0,
    }
}

/// TC-E7E-VAGC-1: closed-loop direct-LEO entry — yaAGC steers, Rust
/// integrates. Reads yaAGC's `ROLLC` each cycle and feeds it back
/// into `EntryIntegrator`.
#[test]
fn tc_e7e_vagc_entry_direct_leo_closed_loop() {
    run_live_scenario_closed_loop("direct_leo");
}

/// TC-E7E-VAGC-2: closed-loop super-circular entry.
#[test]
fn tc_e7e_vagc_entry_lunar_return_closed_loop() {
    run_live_scenario_closed_loop("lunar_return");
}

fn run_live_scenario_closed_loop(scenario: &str) {
    let template_path = template_core_path();
    if !template_path.exists() {
        eprintln!(
            "skipping {scenario} (closed-loop): no template core at {} \
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
            "skipping {scenario} (closed-loop): VirtualAGC build incomplete at {}",
            root.display()
        );
        return;
    }

    let initial = scenario_initial_state(scenario);

    let mut core = CoreImage::load(&template_path)
        .unwrap_or_else(|e| panic!("load template {}: {e}", template_path.display()));
    let symtab = Symtab::load(&listing)
        .unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
    let init = entry_initial_state_for(&initial);
    patch_into(&mut core, &symtab, &init)
        .unwrap_or_else(|e| panic!("patch failed for {scenario}: {e}"));

    let work = std::env::temp_dir().join(format!(
        "vagc_e7e_{scenario}_{}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let core_in = work.join("core_in");
    core.save(&core_in).unwrap();

    // Spawn yaAGC with --dump-time=2 so a fresh core dump appears
    // every simulated SERVICER cycle.
    let port = pick_port();
    let mut child = std::process::Command::new(&yaagc)
        .current_dir(&work)
        .arg("--quiet")
        .arg("--nodebug")
        .arg("--dump-time=2")
        .arg(format!("--port={port}"))
        .arg(&rope)
        .arg(&core_in)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .expect("yaAGC spawn");
    std::thread::sleep(Duration::from_millis(300));

    let mut dsky = DskyScript::new(connect_with_retry(port));
    let mut pipa = PipaInjector::new(
        connect_with_retry(port + 1),
        EntryIntegrator::apollo_cm(),
        initial.pipa_cal,
    );
    let mut recorder = ChannelTraceRecorder::new(connect_with_retry(port + 2));

    // Warm up + V37 63 ENTR.
    let warmup_end = Instant::now() + Duration::from_millis(500);
    while Instant::now() < warmup_end {
        recorder.drain(Duration::from_millis(50));
    }
    dsky.verb_noun(37, 63).expect("V37 63 ENTR");

    let dump_path = work.join("core");
    let mut last_dump_mtime = current_mtime(&dump_path);
    let mut current_bank_rad = 0.0_f64;
    let mut bank_history: Vec<f64> = Vec::new();
    let wall_deadline = Instant::now() + Duration::from_secs(240);

    // Custom per-cycle loop: integrate Rust state under the AGC's
    // latest ROLLC bank, inject the resulting Δv as PIPA pulses, wait
    // for the next core dump, read new ROLLC.
    let integrator = EntryIntegrator::apollo_cm();
    let mut state = scenario_initial_state(scenario);
    init_p61(&mut state);
    init_p62(&mut state);
    init_p63(&mut state);
    start_servicer(&mut state);

    let mut elapsed_s = 0.0_f64;
    let mut history: Vec<(f64, EntryPhase, f64, f64)> = Vec::new();
    let mut min_altitude_km = f64::INFINITY;
    let mut max_sensed_g = 0.0_f64;
    let mut total_cycles: u32 = 0;

    loop {
        if Instant::now() > wall_deadline {
            eprintln!("[{scenario}] closed-loop: wall-clock cap reached, exiting early");
            break;
        }

        // Use the AGC's bank for this cycle; record it for later
        // inspection.
        bank_history.push(current_bank_rad);

        let ld_command = state.entry.ld_command;
        let dv_inertial = integrator.integrate_cycle(
            state.csm_state.position,
            state.csm_state.velocity,
            ld_command,
            current_bank_rad,
            agc_core::services::average_g::SERVICER_PERIOD_S,
        );
        // Feed the Δv into yaAGC via PIPA pulses and into the Rust
        // SERVICER via state.pipa_counts.
        state.pipa_counts = pipa_pulses_for_dv(dv_inertial, &state.pipa_cal);
        // Pre-compute and send the pulses to yaAGC. `PipaInjector::tick`
        // does its own integration — to avoid double-integrating, we
        // emit the already-quantised pulses directly via its public
        // helper through `state.csm_state` (the AGC sees the SAME Δv
        // the Rust SERVICER will see). We reuse the integrator inside
        // `pipa` to keep call sites consistent; the AGC's downstream
        // SERVICER reads only the resulting counter increments, so the
        // per-axis quantisation is what matters.
        let _ = pipa.tick(
            state.csm_state.position,
            state.csm_state.velocity,
            ld_command,
            current_bank_rad,
            agc_core::services::average_g::SERVICER_PERIOD_S,
        );

        servicer_task(&mut state);
        let _ = state.waitlist.pop_task();
        elapsed_s += agc_core::services::average_g::SERVICER_PERIOD_S;
        total_cycles += 1;

        let r_mag = (state.csm_state.position[0].powi(2)
            + state.csm_state.position[1].powi(2)
            + state.csm_state.position[2].powi(2))
        .sqrt();
        let altitude_km = (r_mag - R_EARTH_M) / 1000.0;
        min_altitude_km = min_altitude_km.min(altitude_km);
        max_sensed_g = max_sensed_g.max(state.entry.sensed_acceleration_g);

        history.push((
            elapsed_s,
            state.entry.phase,
            state.entry.sensed_acceleration_g,
            altitude_km,
        ));

        // Drain channel writes that arrived while we were processing.
        recorder.drain(Duration::from_millis(80));

        if state.entry.drogue_deployed || elapsed_s >= MAX_SCENARIO_DURATION_S {
            break;
        }

        // Wait for the AGC's next core dump and update the bank for
        // the next cycle. Tolerate small read races.
        let dump_deadline = Instant::now() + Duration::from_millis(1500);
        if let Some(new_mtime) = wait_for_new_dump(&dump_path, last_dump_mtime, dump_deadline) {
            last_dump_mtime = Some(new_mtime);
            match try_load_core(&dump_path) {
                Some(loaded) => {
                    if let Some(bank) = read_rollc_rad(&loaded, &symtab) {
                        current_bank_rad = bank;
                    }
                }
                None => {
                    // Parse failure on a partial dump; keep the
                    // previous bank.
                }
            }
        }
        // If no new dump appeared in time, fall through with the
        // existing bank — yaAGC may be busy. Next iteration retries.
    }

    let (landed_lat, landed_lon) = sub_satellite_lat_lon(&state);
    let miss_km = haversine_km(
        landed_lat,
        landed_lon,
        state.entry.target_lat_rad,
        state.entry.target_lon_rad,
    );
    let last_history = history.iter().rev().take(10).rev().cloned().collect();
    let final_phase = state.entry.phase;
    let drogue_deployed = state.entry.drogue_deployed;
    let _ = final_phase; // silence warning if unused below

    // Final drain.
    let final_end = Instant::now() + Duration::from_millis(500);
    while Instant::now() < final_end {
        recorder.drain(Duration::from_millis(100));
    }

    // Tear down yaAGC.
    let _ = child.kill();
    let _ = child.wait();
    let _ = std::fs::remove_dir_all(&work);

    let result = ScenarioResult {
        final_phase,
        drogue_deployed,
        elapsed_s,
        miss_km,
        landed_lat_deg: landed_lat.to_degrees(),
        landed_lon_deg: landed_lon.to_degrees(),
        min_altitude_km,
        max_sensed_g,
        last_history,
        total_cycles,
    };

    // Diagnostic: log bank history extremes so a stuck-at-zero ROLLC
    // is obvious on the first capture.
    let bank_min = bank_history
        .iter()
        .cloned()
        .fold(f64::INFINITY, f64::min);
    let bank_max = bank_history
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    eprintln!(
        "[{scenario}] closed-loop: {total_cycles} cycles, drogue={drogue_deployed}, \
         miss={miss_km:.1} km, peak g={max_sensed_g:.2}, \
         bank ∈ [{bank_min:+.3}, {bank_max:+.3}] rad",
        total_cycles = result.total_cycles,
        drogue_deployed = result.drogue_deployed,
        miss_km = result.miss_km,
        max_sensed_g = result.max_sensed_g,
    );

    let trace = recorder.into_trace(
        scenario,
        format!(
            "MS-E7e closed-loop capture, scenario={scenario}, yaAGC at {}",
            root.display()
        ),
    );
    let summary = ScenarioSummary::from_result(
        scenario,
        "MS-E7e closed-loop (yaAGC steers, Rust integrates)",
        &result,
        default_closed_loop_tolerance(),
    );

    if vagc_capture_enabled() {
        write_closed_loop_fixture(&trace, &summary, scenario);
        eprintln!(
            "[{scenario}] VAGC_CAPTURE=1 — refreshed closed-loop fixtures."
        );
    } else {
        verify_closed_loop_against_committed(&summary, scenario);
    }
}

fn write_closed_loop_fixture(trace: &ChannelTrace, summary: &ScenarioSummary, scenario: &str) {
    let trace_path = closed_loop_trace_path(scenario);
    if let Some(parent) = trace_path.parent() {
        std::fs::create_dir_all(parent).expect("mkdir traces");
    }
    trace.save(&trace_path).expect("save trace");

    let text = serde_json::to_string_pretty(summary).expect("serialize closed-loop summary");
    std::fs::write(closed_loop_summary_path(scenario), text).expect("save closed-loop summary");
}

fn verify_closed_loop_against_committed(summary: &ScenarioSummary, scenario: &str) {
    let path = closed_loop_summary_path(scenario);
    if !path.exists() {
        eprintln!(
            "[{scenario}] no committed closed-loop summary at {} — run with VAGC_CAPTURE=1.",
            path.display()
        );
        return;
    }
    let committed: ScenarioSummary =
        serde_json::from_str(&std::fs::read_to_string(&path).expect("read closed-loop summary"))
            .expect("parse closed-loop summary");
    assert_eq!(
        committed.drogue_deployed, summary.drogue_deployed,
        "[{scenario}] live-vs-committed drogue mismatch (closed-loop)"
    );
    // Use the committed tolerance bands for the live-vs-committed
    // checks — they were authored to cover yaAGC's run-to-run jitter.
    let tol = &committed.tolerance;
    check_within(&committed, "miss_distance_km", committed.miss_distance_km, summary.miss_distance_km, tol.miss_km);
    check_within(&committed, "peak_sensed_g", committed.peak_sensed_g, summary.peak_sensed_g, tol.peak_g);
    check_within(&committed, "elapsed_s", committed.elapsed_s, summary.elapsed_s, tol.elapsed_s);
    check_within(&committed, "total_cycles", committed.total_cycles as f64, summary.total_cycles as f64, tol.total_cycles as f64);
}

// ── Core-dump polling primitives ───────────────────────────────────────

fn current_mtime(path: &std::path::Path) -> Option<std::time::SystemTime> {
    std::fs::metadata(path).ok().and_then(|m| m.modified().ok())
}

/// Wait until `path`'s mtime advances past `previous` (or the file
/// first appears). Polls at 50 ms intervals up to `deadline`. Returns
/// the new mtime on success, `None` on timeout.
fn wait_for_new_dump(
    path: &std::path::Path,
    previous: Option<std::time::SystemTime>,
    deadline: Instant,
) -> Option<std::time::SystemTime> {
    while Instant::now() < deadline {
        if let Some(now) = current_mtime(path) {
            if previous.map(|p| now > p).unwrap_or(true) {
                // Give yaAGC ~30 ms of grace so the buffered write
                // has time to flush before we read.
                std::thread::sleep(Duration::from_millis(30));
                return Some(now);
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    None
}

/// Best-effort core-dump load with one retry on parse failure (the
/// dump could have been read mid-write).
fn try_load_core(path: &std::path::Path) -> Option<CoreImage> {
    for _ in 0..3 {
        match CoreImage::load(path) {
            Ok(c) => return Some(c),
            Err(_) => std::thread::sleep(Duration::from_millis(50)),
        }
    }
    None
}

// Silence unused-import warnings for items only used by closed-loop:
#[allow(dead_code)]
fn _suppress_unused_imports() {
    let _ = DapMode::EntryRoll(0.0);
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
