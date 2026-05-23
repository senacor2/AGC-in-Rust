//! MS-E7 end-to-end entry scenarios — Rust-only path.
//!
//! Drives the AGC entry-guidance pipeline (P61–P67) through a complete
//! atmospheric entry, using the [`agc_test::entry_sim::EntryIntegrator`]
//! (via [`agc_test::entry_scenario::simulate_to_drogue`]) to produce
//! realistic sensed Δv that the SERVICER ingests via the PIPA path.
//! The AGC's `entry_servicer_exit` hook computes the sensed-g, R-dot,
//! range-to-go, and dispatches the closed-loop math each 2-s cycle.
//!
//! The shared runner now lives in `agc-test/src/entry_scenario.rs` so
//! the live yaAGC variant (`tests/entry_e2e_vagc.rs`, MS-E7d) can call
//! the same Rust pipeline with no risk of drift on initial conditions
//! or integration sub-step.
//!
//! Stage A scope (this file):
//! - `entry_direct_leo` — direct entry from a 200 km circular orbit.
//! - `entry_lunar_return` — translunar-return entry per MS-E7b.
//! - `#[ignore]` footprint regenerator over the FPA grid.

use agc_core::programs::p61_p67::EntryPhase;
use agc_core::AgcState;
use agc_test::entry_scenario::{
    setup_state_direct_leo, setup_state_lunar_return, simulate_to_drogue, ScenarioResult,
    MAX_SCENARIO_DURATION_S,
};

/// Direct-LEO miss-distance threshold (km). The original MS-E7 exit
/// criterion is ~25 nmi ≈ 46 km. Stage A inherits the cumulative effect
/// of every "stage A simplification" from the preceding milestones:
///
/// - MS-E3 / MS-E3b: DHOOK correction omitted (`GAMMAL = GAMMAL1`).
/// - MS-E4 / MS-E4b: `F1 = FACTOR` gain compression set to 1; no
///   DOWNCNTL or CONSTD branch.
/// - MS-E6 / MS-E6b: PREDICT3's F1 = ∂Range/∂D and F2 = ∂Range/∂RDOT
///   sensitivity terms approximated as zero; no GLIMITER deceleration
///   limiter (the CM peaks at ~6 g without it).
/// - MS-E7 stage A: no Earth-rotation correction (`v_rel = v_inertial`).
///
/// Each of those tightens by 100–200 km in their respective MS-E*b
/// follow-ups. Stage A accepts up to **1000 km** miss; the assertion
/// here is "the pipeline runs end-to-end without diverging", not
/// "the AGC lands within nautical-mile accuracy" — that's the original
/// MS-E7 exit criterion which the *b* milestones will earn back.
const MISS_DISTANCE_DIRECT_LEO_KM: f64 = 1_000.0;

/// Lunar-return miss-distance threshold (km). Lunar return is the
/// harder trajectory: V ≈ 11 km/s super-circular, perigee well below
/// the atmosphere, requiring the P65 skip-phase UPCONTRL feedback law
/// to fly up out of the dense atmosphere and re-enter at a lower
/// energy. Stage-A simplifications hit harder here than in direct LEO:
/// - The Skip phase is exercised, and our `F1 = 1` simplification in
///   `upcontrol_step` produces a SKIPPER feedback that's coarser than
///   the AGC's gain-compressed form.
/// - The trajectory hits peak deceleration far above GMAX/2 = 4 g
///   (no GLIMITER deferred to MS-E6b means no L/D clamping there).
///
/// **3000 km** is "pipeline doesn't diverge / spacecraft lands
/// somewhere on Earth". Tightens to ~250–500 km once MS-E3b/E4b/E6b
/// land their fixture-validated refinements.
const MISS_DISTANCE_LUNAR_RETURN_KM: f64 = 3_000.0;

/// `entry_direct_leo` — direct entry from a 200 km LEO trajectory.
#[test]
fn entry_direct_leo() {
    let state = setup_state_direct_leo();
    run_entry_scenario("direct_leo", state, MISS_DISTANCE_DIRECT_LEO_KM);
}

/// `entry_lunar_return` — translunar-return entry from the documented
/// MS-E7b initial conditions.
#[test]
fn entry_lunar_return() {
    let state = setup_state_lunar_return();
    run_entry_scenario("lunar_return", state, MISS_DISTANCE_LUNAR_RETURN_KM);
}

/// Run one complete entry scenario through the AGC + integrator and
/// assert the miss-distance acceptance criterion.
fn run_entry_scenario(name: &str, state: AgcState, miss_threshold_km: f64) {
    let r = simulate_to_drogue(state);

    assert!(
        r.drogue_deployed,
        "[{name}] scenario did not reach drogue deploy within \
         {MAX_SCENARIO_DURATION_S} s — phase={:?}, peak g={:.3}, \
         min alt={:.1} km\nlast 10 cycles:\n{}",
        r.final_phase,
        r.max_sensed_g,
        r.min_altitude_km,
        r.last_history
            .iter()
            .map(|(t, p, g, h)| format!("  t={t:.1}s phase={p:?} g={g:.3} h={h:.1}km"))
            .collect::<Vec<_>>()
            .join("\n")
    );

    assert_eq!(
        r.final_phase,
        EntryPhase::Final,
        "[{name}] drogue deploy must land us in Final phase"
    );

    eprintln!(
        "[{name}] drogue at t={:.1}s, miss = {:.1} km (threshold {:.0} km)",
        r.elapsed_s, r.miss_km, miss_threshold_km
    );
    assert!(
        r.miss_km < miss_threshold_km,
        "[{name}] miss distance {:.1} km exceeds {miss_threshold_km} km threshold\n  \
         landed: lat={:.4} lon={:.4}\n  \
         elapsed: {:.1} s",
        r.miss_km,
        r.landed_lat_deg,
        r.landed_lon_deg,
        r.elapsed_s,
    );
}

// ── Footprint sweep (regenerator, #[ignore]) ───────────────────────────────

/// Regenerate `docs/entry_footprint.md` by sweeping the flight-path
/// angle from −5.5° to −7.5° in 0.25° steps for both the direct-LEO and
/// lunar-return scenarios. Records drogue time, miss distance, minimum
/// altitude, and peak sensed-g per cell.
///
/// `#[ignore]`-gated because it takes ~30–60 s wall-clock for the
/// 18-cell sweep — too slow for normal `cargo test`. Run with:
///
/// ```sh
/// cargo test -p agc-test --test entry_e2e regenerate_footprint_table \
///     -- --ignored --nocapture
/// ```
///
/// The committed Markdown table is the baseline; refinements landing
/// in #32 / #33 / #34 (MS-E*b) should tighten the miss-distance
/// column. After landing any such refinement, re-run this test and
/// commit the updated table.
#[test]
#[ignore]
fn regenerate_footprint_table() {
    use agc_test::entry_scenario::make_initial_state;

    let fpa_grid: Vec<f64> = (0..=8).map(|i| -5.5 - 0.25 * i as f64).collect();

    let mut rows = Vec::new();
    for fpa in &fpa_grid {
        let leo = simulate_to_drogue(make_initial_state(7_900.0, *fpa, 20.0));
        rows.push(("direct_leo", *fpa, leo));
        let lunar = simulate_to_drogue(make_initial_state(11_000.0, *fpa, 45.0));
        rows.push(("lunar_return", *fpa, lunar));
    }

    let markdown = render_footprint_markdown(&rows);
    let out_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("docs")
        .join("entry_footprint.md");
    std::fs::write(&out_path, markdown)
        .unwrap_or_else(|e| panic!("cannot write {}: {}", out_path.display(), e));
    eprintln!("wrote {}", out_path.display());
}

fn render_footprint_markdown(rows: &[(&str, f64, ScenarioResult)]) -> String {
    let mut s = String::new();
    s.push_str("# Entry Guidance Footprint Sweep\n\n");
    s.push_str(
        "Generated by `cargo test -p agc-test --test entry_e2e \
         regenerate_footprint_table -- --ignored --nocapture`.\n\n",
    );
    s.push_str(
        "Each row records the result of running one closed-loop entry \
         scenario (P61→P67) end-to-end through the AGC + `EntryIntegrator`. \
         The flight-path angle (FPA) is varied; all other initial \
         conditions stay fixed per `setup_state_direct_leo` / \
         `setup_state_lunar_return`.\n\n",
    );
    s.push_str(
        "This is the **stage-A** baseline. Miss distances are expected \
         to tighten as MS-E3b (#32), MS-E4b (#33), and MS-E6b (#34) land \
         their fixture-validated refinements. Re-run the sweep and \
         commit an updated table after each landing.\n\n",
    );

    // Split rows by scenario name into two tables for readability.
    for scenario in ["direct_leo", "lunar_return"] {
        let title = match scenario {
            "direct_leo" => "Direct LEO (V = 7900 m/s at interface)",
            "lunar_return" => "Lunar Return (V = 11 000 m/s at interface)",
            _ => unreachable!(),
        };
        s.push_str(&format!("## {title}\n\n"));
        s.push_str(
            "| FPA (°) | Drogue at | Drogue? | Miss (km) | Min alt (km) | Peak g | Final phase |\n",
        );
        s.push_str("|---|---|---|---|---|---|---|\n");

        for (name, fpa, r) in rows.iter().filter(|(n, _, _)| *n == scenario) {
            let drogue_marker = if r.drogue_deployed { "✓" } else { "—" };
            s.push_str(&format!(
                "| {:>+6.2} | {:>6.1} s | {} | {:>7.1} | {:>7.1} | {:>5.2} | {:?} |\n",
                fpa,
                r.elapsed_s,
                drogue_marker,
                r.miss_km,
                r.min_altitude_km,
                r.max_sensed_g,
                r.final_phase,
            ));
            // Silence the unused-binding warning when sweep grows.
            let _ = name;
        }
        s.push('\n');
    }
    s
}
