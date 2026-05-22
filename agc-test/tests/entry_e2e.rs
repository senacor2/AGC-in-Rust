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

/// Miss-distance acceptance threshold (km). The original MS-E7 exit
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
const MISS_DISTANCE_THRESHOLD_KM: f64 = 1_000.0;

/// `entry_direct_leo` — direct entry from a 200 km LEO trajectory.
///
/// Initial conditions at entry interface (122 km altitude):
/// - Position: `(R_E + 122 km, 0, 0)` ECI — lat = 0, lon = 0
/// - Velocity: `7800 m/s` at flight-path-angle −1.5° (descending), heading +Y
/// - Target: lat = 0, lon = 20° east (Pacific equator, ~2225 km downrange)
///
/// The test runs the full pipeline (P61 → P62 → P63 → SERVICER ticks →
/// Entry → Skip → Final → drogue) and asserts the spacecraft lands within
/// the documented miss-distance band of the target.
#[test]
fn entry_direct_leo() {
    let mut state = setup_initial_state();
    let integrator = EntryIntegrator::apollo_cm();

    // Drive the AGC through the entry-prep sequence.
    init_p61(&mut state);
    init_p62(&mut state);
    init_p63(&mut state);
    assert_eq!(state.entry.phase, EntryPhase::PreEntry, "fixture");

    // Phase transitions (PreEntry → Entry → Skip → Final) happen
    // automatically inside entry_servicer_exit each cycle once the 0.05g
    // threshold trips — matches the AGC's behaviour in flight.

    // Kick off the SERVICER. servicer_task runs on each cycle.
    start_servicer(&mut state);

    let mut elapsed_s = 0.0;
    let mut history: Vec<(f64, EntryPhase, f64, f64)> = Vec::new();
    loop {
        // Read the bank command the DAP is currently holding.
        let bank_rad = match state.dap_state.mode {
            DapMode::EntryRoll(b) => b,
            _ => 0.0, // before threshold trip, no entry roll yet
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

        // Quantise into PIPA pulses and stage them where the SERVICER
        // reads. REFSMMAT defaults to identity → platform = inertial.
        state.pipa_counts = pipa_pulses_for_dv(dv_inertial, &state.pipa_cal);

        // Run one SERVICER cycle: integrates state vector (with gravity),
        // computes sensed_acceleration_g, dispatches the closed-loop
        // L/D law per current phase, updates DAP, runs select_phase.
        servicer_task(&mut state);
        // servicer_task reschedules itself into the Waitlist for the next
        // 2-s cycle. The test drives `servicer_task` directly, so we drain
        // the reschedule to prevent the Waitlist from overflowing (which
        // would otherwise clear the active flag and silently freeze the
        // simulation). The real flight stack drains via T3RUPT / agc-sim's
        // `WaitlistPump`; this manual pop is the equivalent in a stripped-
        // down test loop.
        let _ = state.waitlist.pop_task();
        elapsed_s += SERVICER_PERIOD_S;

        // Record trajectory for diagnostics on failure.
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
            "scenario did not reach drogue deploy within {MAX_SCENARIO_DURATION_S} s — \
             phase={:?}, sensed_g={:.3}, altitude={altitude_km:.1} km\nlast 10 cycles:\n{}",
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

    // Drogue deployed. Verify miss distance.
    assert_eq!(
        state.entry.phase,
        EntryPhase::Final,
        "drogue deploy must land us in Final phase"
    );

    // Compute the landing point in (lat, lon) and compare to the target.
    let (landed_lat, landed_lon) = sub_satellite_lat_lon(&state);
    let miss_km = haversine_km(
        landed_lat,
        landed_lon,
        state.entry.target_lat_rad,
        state.entry.target_lon_rad,
    );
    assert!(
        miss_km < MISS_DISTANCE_THRESHOLD_KM,
        "miss distance {miss_km:.1} km exceeds {MISS_DISTANCE_THRESHOLD_KM} km threshold\n  \
         target: lat={:.4} lon={:.4}\n  \
         landed: lat={:.4} lon={:.4}\n  \
         elapsed: {elapsed_s:.1} s",
        state.entry.target_lat_rad.to_degrees(),
        state.entry.target_lon_rad.to_degrees(),
        landed_lat.to_degrees(),
        landed_lon.to_degrees(),
    );
}

// ── helpers ──────────────────────────────────────────────────────────────────

fn setup_initial_state() -> AgcState {
    let mut state = AgcState::new();
    state.time = Met(0);
    state.gha_epoch_rad = 0.0; // GHA = 0 at MET = 0 → ECI ≡ ECEF

    // Initial inertial state at entry interface: place CM at lon=0 lat=0
    // on the +X axis, 122 km altitude, moving +Y with a downward radial
    // component. FPA = -6.0° matches the nominal Apollo CM entry corridor
    // (real flights used -6.5° to -7.5°). Velocity slightly above circular
    // orbital at the interface altitude.
    let r0 = R_EARTH_M + 122_000.0;
    let v0 = 7900.0_f64;
    let fpa = -6.0_f64.to_radians();
    state.csm_state.position = [r0, 0.0, 0.0];
    state.csm_state.velocity = [v0 * fpa.sin(), v0 * fpa.cos(), 0.0];

    // Target on the equator, 20° east of the entry interface.
    state.entry.target_lat_rad = 0.0;
    state.entry.target_lon_rad = 20.0_f64.to_radians();

    // PreEntry-side bookkeeping: phase will be set by init_p61/p62/p63.
    state
}

fn sub_satellite_lat_lon(state: &AgcState) -> (f64, f64) {
    let pos = state.csm_state.position;
    let r = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
    let lat = (pos[2] / r).asin();
    let lon = pos[1].atan2(pos[0]);
    (lat, lon)
}
