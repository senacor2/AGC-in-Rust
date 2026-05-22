//! MS-E7 end-to-end entry scenarios.
//!
//! Drives the AGC entry-guidance pipeline (P61–P67) through a complete
//! atmospheric entry, using the [`entry_sim::EntryIntegrator`] to produce
//! realistic sensed Δv that the SERVICER ingests via the PIPA path. The
//! AGC's `entry_servicer_exit` hook computes the sensed-g, R-dot, range-
//! to-go, and dispatches the closed-loop math each 2-s cycle.
//!
//! Stage A scope (this file):
//! - `entry_direct_leo` — direct entry from a 200 km circular orbit. No
//!   skip required; the pipeline runs Entry → Skip (transient) → Final →
//!   drogue deploy on the velocity threshold.
//!
//! Deferred to MS-E7b:
//! - Lunar-return entry (V ≈ 11 km/s at interface, P65 skip required).
//! - Entry-FPA / azimuth footprint sweep.
//! - VirtualAGC channel-trace comparison (gated by `VAGC_AVAILABLE=1`).

use agc_core::control::DapMode;
use agc_core::programs::p61_p67::{init_p61, init_p62, init_p63, EntryPhase};
use agc_core::services::average_g::{servicer_task, start_servicer, SERVICER_PERIOD_S};
use agc_core::types::Met;
use agc_core::AgcState;
use agc_test::entry_sim::{haversine_km, pipa_pulses_for_dv, EntryIntegrator};

/// Earth equatorial radius (m) used for entry-interface geometry.
const R_EARTH_M: f64 = 6_371_000.0;

/// Hard upper bound on scenario duration (s). Apollo entries land in
/// ~7–10 minutes from interface; we allow 20 to be defensive without
/// turning a hang into an infinite loop.
const MAX_SCENARIO_DURATION_S: f64 = 20.0 * 60.0;

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
///
/// Initial conditions at entry interface (122 km altitude):
/// - Position: `(R_E + 122 km, 0, 0)` ECI — lat = 0, lon = 0
/// - Velocity: `7900 m/s` at flight-path-angle −6° (descending), heading +Y
/// - Target: lat = 0, lon = 20° east (Pacific equator, ~2225 km downrange)
///
/// The test runs the full pipeline (P61 → P62 → P63 → SERVICER ticks →
/// Entry → Skip → Final → drogue) and asserts the spacecraft lands within
/// the documented miss-distance band of the target.
#[test]
fn entry_direct_leo() {
    let state = setup_state_direct_leo();
    run_entry_scenario("direct_leo", state, MISS_DISTANCE_DIRECT_LEO_KM);
}

/// `entry_lunar_return` — translunar-return entry from the documented
/// MS-E7b initial conditions.
///
/// Initial conditions at entry interface:
/// - Position: `(R_E + 122 km, 0, 0)` ECI — lat = 0, lon = 0
/// - Velocity: `11 000 m/s` at flight-path-angle −6° (Apollo lunar-
///   return corridor), heading +Y. The orbit is highly elliptical
///   (a ≈ 221 000 km, e ≈ 0.971, perigee well below the surface).
/// - Target: lat = 0, lon = 45° east (Pacific equator, ~5000 km
///   downrange).
///
/// Unlike `entry_direct_leo`, this trajectory **requires the P65
/// Skip phase**: at V = 11 km/s the spacecraft would peak well above
/// 20 g if it just plunged in, so HUNTEST must converge and UPCONTRL
/// must lift the trajectory up. Exercises the full MS-E4 SKIPPER
/// feedback law end-to-end.
///
/// The stage-A miss-distance threshold is 3000 km — large because
/// SKIPPER's `F1 = 1` simplification and the missing GLIMITER both
/// hit this trajectory harder than the direct-LEO scenario. The
/// threshold tightens to ~250–500 km once MS-E3b/E4b/E6b land.
#[test]
fn entry_lunar_return() {
    let state = setup_state_lunar_return();
    run_entry_scenario("lunar_return", state, MISS_DISTANCE_LUNAR_RETURN_KM);
}

// ── scenario runner ─────────────────────────────────────────────────────────

/// Run one complete entry scenario through the AGC + integrator and assert
/// the miss-distance acceptance criterion.
///
/// `state` must already have the initial CSM state vector, target
/// landing coordinates, MET, and GHA-epoch populated. This helper takes
/// care of `init_p61 → init_p62 → init_p63 → start_servicer` and then
/// pumps `servicer_task` until drogue deploys.
fn run_entry_scenario(name: &str, mut state: AgcState, miss_threshold_km: f64) {
    let integrator = EntryIntegrator::apollo_cm();

    init_p61(&mut state);
    init_p62(&mut state);
    init_p63(&mut state);
    assert_eq!(
        state.entry.phase,
        EntryPhase::PreEntry,
        "[{name}] fixture: P63 must leave phase=PreEntry"
    );

    // Kick off the SERVICER. servicer_task runs on each cycle.
    start_servicer(&mut state);

    let mut elapsed_s = 0.0;
    let mut history: Vec<(f64, EntryPhase, f64, f64)> = Vec::new();
    loop {
        // Read the bank command the DAP is currently holding.
        let bank_rad = match state.dap_state.mode {
            DapMode::EntryRoll(b) => b,
            _ => 0.0,
        };
        let ld_command = state.entry.ld_command;

        // Advance the dynamics by one SERVICER cycle.
        let dv_inertial = integrator.integrate_cycle(
            state.csm_state.position,
            state.csm_state.velocity,
            ld_command,
            bank_rad,
            SERVICER_PERIOD_S,
        );

        // Quantise into PIPA pulses for the SERVICER's normal pipeline.
        // REFSMMAT defaults to identity → platform = inertial.
        state.pipa_counts = pipa_pulses_for_dv(dv_inertial, &state.pipa_cal);

        // Run one SERVICER cycle: integrates state vector (with gravity),
        // computes sensed_acceleration_g, dispatches the closed-loop
        // L/D law per current phase, updates DAP, runs select_phase.
        servicer_task(&mut state);
        // Drain the self-reschedule (real flight uses T3RUPT / agc-sim's
        // WaitlistPump; this is the equivalent in a stripped-down test
        // loop).
        let _ = state.waitlist.pop_task();
        elapsed_s += SERVICER_PERIOD_S;

        let r_mag = (state.csm_state.position[0].powi(2)
            + state.csm_state.position[1].powi(2)
            + state.csm_state.position[2].powi(2))
        .sqrt();
        let altitude_km = (r_mag - R_EARTH_M) / 1000.0;
        history.push((
            elapsed_s,
            state.entry.phase,
            state.entry.sensed_acceleration_g,
            altitude_km,
        ));

        if state.entry.drogue_deployed {
            break;
        }
        assert!(
            elapsed_s < MAX_SCENARIO_DURATION_S,
            "[{name}] scenario did not reach drogue deploy within \
             {MAX_SCENARIO_DURATION_S} s — phase={:?}, sensed_g={:.3}, \
             altitude={altitude_km:.1} km\nlast 10 cycles:\n{}",
            state.entry.phase,
            state.entry.sensed_acceleration_g,
            history
                .iter()
                .rev()
                .take(10)
                .map(|(t, p, g, h)| format!("  t={t:.1}s phase={p:?} g={g:.3} h={h:.1}km"))
                .collect::<Vec<_>>()
                .join("\n")
        );
    }

    assert_eq!(
        state.entry.phase,
        EntryPhase::Final,
        "[{name}] drogue deploy must land us in Final phase"
    );

    let (landed_lat, landed_lon) = sub_satellite_lat_lon(&state);
    let miss_km = haversine_km(
        landed_lat,
        landed_lon,
        state.entry.target_lat_rad,
        state.entry.target_lon_rad,
    );
    eprintln!(
        "[{name}] drogue at t={:.1}s, miss = {:.1} km (threshold {:.0} km)",
        elapsed_s, miss_km, miss_threshold_km
    );
    assert!(
        miss_km < miss_threshold_km,
        "[{name}] miss distance {miss_km:.1} km exceeds {miss_threshold_km} km threshold\n  \
         target: lat={:.4} lon={:.4}\n  \
         landed: lat={:.4} lon={:.4}\n  \
         elapsed: {elapsed_s:.1} s",
        state.entry.target_lat_rad.to_degrees(),
        state.entry.target_lon_rad.to_degrees(),
        landed_lat.to_degrees(),
        landed_lon.to_degrees(),
    );
}

// ── initial-state factories ─────────────────────────────────────────────────

/// Build an entry-interface state vector at `(lat=0, lon=0, alt=122 km)`
/// with the given inertial speed and flight-path angle, heading +Y, and
/// the target at `target_lon_deg_east` on the equator.
fn make_initial_state(speed_mps: f64, fpa_deg: f64, target_lon_deg_east: f64) -> AgcState {
    let mut state = AgcState::new();
    state.time = Met(0);
    state.gha_epoch_rad = 0.0; // GHA = 0 at MET = 0 → ECI ≡ ECEF

    let r0 = R_EARTH_M + 122_000.0;
    let fpa = fpa_deg.to_radians();
    state.csm_state.position = [r0, 0.0, 0.0];
    state.csm_state.velocity = [speed_mps * fpa.sin(), speed_mps * fpa.cos(), 0.0];

    state.entry.target_lat_rad = 0.0;
    state.entry.target_lon_rad = target_lon_deg_east.to_radians();
    state
}

/// Direct-LEO initial state — 200 km circular orbit deorbited to FPA=-6°.
/// V = 7900 m/s (slightly super-circular at interface altitude).
/// Target 20° east ≈ 2226 km downrange.
fn setup_state_direct_leo() -> AgcState {
    make_initial_state(7_900.0, -6.0, 20.0)
}

/// Lunar-return initial state — translunar-return entry per
/// `specs/entry-guidance-plan.md` §5 MS-E7.
/// V = 11 000 m/s super-circular; orbit highly elliptical, perigee well
/// below the surface (a ≈ 221 000 km, e ≈ 0.971). Target 45° east ≈
/// 5004 km downrange (Pacific splashdown).
fn setup_state_lunar_return() -> AgcState {
    make_initial_state(11_000.0, -6.0, 45.0)
}

fn sub_satellite_lat_lon(state: &AgcState) -> (f64, f64) {
    let pos = state.csm_state.position;
    let r = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
    let lat = (pos[2] / r).asin();
    let lon = pos[1].atan2(pos[0]);
    (lat, lon)
}
