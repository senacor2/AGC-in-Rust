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
//! Scenarios:
//! - `entry_direct_leo` — direct entry from a 200 km circular orbit.
//! - `entry_lunar_return` — translunar-return entry per MS-E7b.
//! - `#[ignore]` footprint regenerator over the FPA grid.

use agc_core::programs::p61_p67::EntryPhase;
use agc_core::AgcState;
use agc_test::entry_scenario::{
    setup_state_direct_leo, setup_state_lunar_return, simulate_to_drogue, ScenarioResult,
    MAX_SCENARIO_DURATION_S,
};

/// Direct-LEO miss-distance threshold (km). Tightened in #82 around the
/// post-#87 achieved baseline (~680 km) with ~17 % headroom so floating-
/// point noise and minor algorithm tweaks don't flip the test.
///
/// All three physical-model gaps identified by the orbital-mechanics
/// review have landed: #85 (GLIMITER scope), #86 (CONSTD divergence),
/// #87 (Earth-rotation correction in the integrators). Direct-LEO is a
/// synthetic scenario (no historical Apollo direct-LEO entry exists),
/// so the gate is sized for regression detection, not historical
/// accuracy.
const MISS_DISTANCE_DIRECT_LEO_KM: f64 = 800.0;

/// Lunar-return miss-distance threshold (km). Lunar return is the
/// harder trajectory: V ≈ 11 km/s super-circular, perigee well below
/// the atmosphere, requiring the P65 skip-phase UPCONTRL feedback law
/// to fly up out of the dense atmosphere and re-enter at a lower
/// energy. Tightened in #82 around the post-#87 achieved baseline
/// (~111 km) with ~80 % headroom.
///
/// Apollo 8 actual splashdown miss was ~4.6 km; the simulator is now
/// within ~25 × historical accuracy. Tighter gates would over-constrain
/// against the remaining simplifications (no J2, simplified DAP bank
/// dynamics, exponential atmosphere).
const MISS_DISTANCE_LUNAR_RETURN_KM: f64 = 200.0;

/// Peak-g acceptance band for `entry_direct_leo` (#83). Synthetic direct
/// LEO at FPA = −6° currently peaks at ~9.1 g; ~25 % headroom catches
/// L/D-sign bugs and ballistic regressions.
const PEAK_G_BAND_DIRECT_LEO: (f64, f64) = (7.0, 11.0);

/// Peak-g acceptance band for `entry_lunar_return` (#83). Lunar return at
/// FPA = −6.48° currently peaks at ~10.4 g — higher than Apollo 8 actual
/// (6.84 g) because the simulator runs a steeper trajectory shape than
/// the historical one.
const PEAK_G_BAND_LUNAR_RETURN: (f64, f64) = (8.5, 12.5);

/// Peak Sutton–Graves heat-flux band for `entry_direct_leo` (#96, MW/m²).
/// Synthetic direct LEO at FPA = −6° peaks at ~0.80 MW/m²; ~25 % headroom.
const PEAK_HEAT_BAND_DIRECT_LEO: (f64, f64) = (0.5, 1.2);

/// Peak Sutton–Graves heat-flux band for `entry_lunar_return` (#96, MW/m²).
/// Lunar return at FPA = −6.48° peaks at ~1.88 MW/m² — well below Apollo
/// 8 actual (~4.77 MW/m²) because the simulator's peak heating occurs at
/// higher altitude / lower density than the historical trajectory.
const PEAK_HEAT_BAND_LUNAR_RETURN: (f64, f64) = (1.3, 2.5);

/// `entry_direct_leo` — direct entry from a 200 km LEO trajectory.
#[test]
fn entry_direct_leo() {
    let state = setup_state_direct_leo();
    run_entry_scenario(
        "direct_leo",
        state,
        MISS_DISTANCE_DIRECT_LEO_KM,
        PEAK_G_BAND_DIRECT_LEO,
        PEAK_HEAT_BAND_DIRECT_LEO,
    );
}

/// `entry_lunar_return` — translunar-return entry from the documented
/// MS-E7b initial conditions.
#[test]
fn entry_lunar_return() {
    let state = setup_state_lunar_return();
    run_entry_scenario(
        "lunar_return",
        state,
        MISS_DISTANCE_LUNAR_RETURN_KM,
        PEAK_G_BAND_LUNAR_RETURN,
        PEAK_HEAT_BAND_LUNAR_RETURN,
    );
}

/// Run one complete entry scenario through the AGC + integrator and
/// assert the miss-distance, peak-g, and peak-heating acceptance criteria.
fn run_entry_scenario(
    name: &str,
    state: AgcState,
    miss_threshold_km: f64,
    peak_g_band: (f64, f64),
    peak_heat_band: (f64, f64),
) {
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

    // #83: peak-g shape check — catches trajectory regressions (L/D-sign
    // bug, ballistic entry, runaway lift) that the miss-distance gate
    // alone may not see.
    let (min_g, max_g) = peak_g_band;
    eprintln!("[{name}] peak g = {:.2} (band [{min_g:.2}, {max_g:.2}])", r.max_sensed_g);
    assert!(
        (min_g..=max_g).contains(&r.max_sensed_g),
        "[{name}] peak sensed g = {:.2} outside [{min_g:.2}, {max_g:.2}] g",
        r.max_sensed_g,
    );

    // #96: peak Sutton–Graves stagnation-point heat flux. Pairs with the
    // peak-g check above to constrain the trajectory's thermal shape.
    let (min_q, max_q) = peak_heat_band;
    eprintln!(
        "[{name}] peak q̇ = {:.2} MW/m² (band [{min_q:.2}, {max_q:.2}])",
        r.max_heating_rate_mw_m2,
    );
    assert!(
        (min_q..=max_q).contains(&r.max_heating_rate_mw_m2),
        "[{name}] peak Sutton–Graves heat flux = {:.2} MW/m² outside \
         [{min_q:.2}, {max_q:.2}] MW/m²",
        r.max_heating_rate_mw_m2,
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
/// The committed Markdown table is the production baseline now that
/// #85 (GLIMITER scope), #86 (CONSTD divergence routing), #87 (Earth
/// rotation), and #82 (gate tightening) have landed. Re-run after any
/// future entry-guidance change and commit the updated table.
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
        "Production baseline. The entry-guidance physics chain — \
         GLIMITER scope (#85), CONSTD divergence routing (#86), and \
         Earth-rotation correction (#87) — is in place. Miss-distance \
         gates were tightened around these baselines in #82. Re-run \
         this sweep after any future entry-guidance change and commit \
         the updated table.\n\n",
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
            "| FPA (°) | Drogue at | Drogue? | Miss (km) | Min alt (km) | Peak g | Peak q̇ (MW/m²) | Final phase |\n",
        );
        s.push_str("|---|---|---|---|---|---|---|---|\n");

        for (name, fpa, r) in rows.iter().filter(|(n, _, _)| *n == scenario) {
            let drogue_marker = if r.drogue_deployed { "✓" } else { "—" };
            s.push_str(&format!(
                "| {:>+6.2} | {:>6.1} s | {} | {:>7.1} | {:>7.1} | {:>5.2} | {:>5.2} | {:?} |\n",
                fpa,
                r.elapsed_s,
                drogue_marker,
                r.miss_km,
                r.min_altitude_km,
                r.max_sensed_g,
                r.max_heating_rate_mw_m2,
                r.final_phase,
            ));
            // Silence the unused-binding warning when sweep grows.
            let _ = name;
        }
        s.push('\n');
    }
    s
}
