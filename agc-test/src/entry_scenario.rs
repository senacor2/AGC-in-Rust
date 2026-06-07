//! Shared entry-scenario runner for the MS-E7 / MS-E7d test suites.
//!
//! Hosts the Rust-only closed-loop entry pipeline (`simulate_to_drogue`)
//! and the canonical [`ScenarioInitialState`] factories for the two
//! named scenarios. Both `tests/entry_e2e.rs` (the all-Rust path) and
//! `tests/entry_e2e_vagc.rs` (the live yaAGC path) call into this
//! module so the two test files cannot drift apart on initial
//! conditions or the integration sub-step.
//!
//! Refactored out of `tests/entry_e2e.rs` for MS-E7d (see
//! `docs/entry_channel_trace.md`).

use agc_core::control::DapMode;
use agc_core::programs::p61_p67::{init_p61, init_p62, init_p63, EntryPhase};
use agc_core::services::average_g::{servicer_task, start_servicer, SERVICER_PERIOD_S};
use agc_core::types::Met;
use agc_core::AgcState;

use crate::entry_sim::{haversine_km, pipa_pulses_for_dv, EntryIntegrator};

/// Earth equatorial radius (m), shared across the entry pipeline tests.
pub const R_EARTH_M: f64 = 6_371_000.0;

/// Hard upper bound on scenario duration (s). Apollo entries land in
/// ~7–10 minutes from interface; we allow 20 minutes to be defensive
/// without turning a hang into an infinite loop.
pub const MAX_SCENARIO_DURATION_S: f64 = 20.0 * 60.0;

/// Diagnostics for one closed-loop entry trajectory.
#[derive(Clone, Debug)]
pub struct ScenarioResult {
    /// Phase the AGC ended in.
    pub final_phase: EntryPhase,
    /// Whether `state.entry.drogue_deployed` is set at exit.
    pub drogue_deployed: bool,
    /// Elapsed simulation seconds at exit.
    pub elapsed_s: f64,
    /// Great-circle miss distance from the configured target (km).
    pub miss_km: f64,
    /// Landed sub-satellite latitude (degrees, east positive).
    pub landed_lat_deg: f64,
    /// Landed sub-satellite longitude (degrees, east positive).
    pub landed_lon_deg: f64,
    /// Minimum altitude reached during the run (km above mean sea level).
    pub min_altitude_km: f64,
    /// Peak sensed-g recorded during the run.
    pub max_sensed_g: f64,
    /// Peak Sutton–Graves stagnation-point heat flux observed during the
    /// run (MW/m²). Apollo 8 actual was ~4.77 MW/m² (#96).
    pub max_heating_rate_mw_m2: f64,
    /// Last 10 cycles of `(t, phase, sensed_g, alt_km)`. Useful in
    /// failure messages — `tests/entry_e2e.rs` formats it into the
    /// assertion text.
    pub last_history: Vec<(f64, EntryPhase, f64, f64)>,
    /// Total number of SERVICER cycles executed.
    pub total_cycles: u32,
}

/// Simulate one entry scenario all the way to drogue deploy (or the
/// [`MAX_SCENARIO_DURATION_S`] timeout). The per-cycle bank is read
/// from the AGC's own DAP state (`DapMode::EntryRoll(_)` defaults to
/// 0 otherwise). No assertions — returns the diagnostics for the
/// caller to inspect.
///
/// `state` must already have the initial CSM state vector, target
/// landing coordinates, MET, and GHA-epoch populated. This helper
/// takes care of `init_p61` → `init_p62` → `init_p63` → `start_servicer`
/// and then ticks the SERVICER loop until exit.
pub fn simulate_to_drogue(state: AgcState) -> ScenarioResult {
    simulate_to_drogue_with_bank(state, |s| match s.dap_state.mode {
        DapMode::EntryRoll(b) => b,
        _ => 0.0,
    })
}

/// Same as [`simulate_to_drogue`] but the per-cycle bank is supplied
/// by the caller's `bank_source` closure. Used by the MS-E7e
/// closed-loop live test to feed yaAGC's `ROLLC` back into the
/// integrator each cycle.
///
/// The closure is invoked **before** each cycle's integration, with a
/// `&AgcState` view of the current spacecraft and AGC state. The
/// returned bank (radians, 0 = lift up, positive = right-bank) is
/// passed straight to [`EntryIntegrator::integrate_cycle`].
pub fn simulate_to_drogue_with_bank<F>(mut state: AgcState, mut bank_source: F) -> ScenarioResult
where
    F: FnMut(&AgcState) -> f64,
{
    let integrator = EntryIntegrator::apollo_cm();

    init_p61(&mut state);
    init_p62(&mut state);
    init_p63(&mut state);
    debug_assert_eq!(state.entry.phase, EntryPhase::PreEntry);

    start_servicer(&mut state);

    let mut elapsed_s = 0.0;
    let mut history: Vec<(f64, EntryPhase, f64, f64)> = Vec::new();
    let mut min_altitude_km = f64::INFINITY;
    let mut max_sensed_g = 0.0_f64;
    let mut max_heat_flux_w_m2 = 0.0_f64;
    let mut total_cycles: u32 = 0;

    loop {
        let bank_rad = bank_source(&state);
        let ld_command = state.entry.ld_command;

        let diag = integrator.integrate_cycle_with_diag(
            state.csm_state.position,
            state.csm_state.velocity,
            ld_command,
            bank_rad,
            SERVICER_PERIOD_S,
        );
        max_heat_flux_w_m2 = max_heat_flux_w_m2.max(diag.peak_heat_flux_w_m2);
        state.pipa_counts = pipa_pulses_for_dv(diag.sensed_dv, &state.pipa_cal);

        servicer_task(&mut state);
        let _ = state.waitlist.pop_task();
        elapsed_s += SERVICER_PERIOD_S;
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

        if state.entry.drogue_deployed || elapsed_s >= MAX_SCENARIO_DURATION_S {
            break;
        }
    }

    let (landed_lat, landed_lon) = sub_satellite_lat_lon(&state);
    let miss_km = haversine_km(
        landed_lat,
        landed_lon,
        state.entry.target_lat_rad,
        state.entry.target_lon_rad,
    );
    let last_history = history.iter().rev().take(10).rev().cloned().collect();

    ScenarioResult {
        final_phase: state.entry.phase,
        drogue_deployed: state.entry.drogue_deployed,
        elapsed_s,
        miss_km,
        landed_lat_deg: landed_lat.to_degrees(),
        landed_lon_deg: landed_lon.to_degrees(),
        min_altitude_km,
        max_sensed_g,
        max_heating_rate_mw_m2: max_heat_flux_w_m2 / 1.0e6,
        last_history,
        total_cycles,
    }
}

/// Build an entry-interface state vector at `(lat=0, lon=0, alt=122 km)`
/// with the given inertial speed and flight-path angle, heading +Y, and
/// the target at `target_lon_deg_east` on the equator.
pub fn make_initial_state(speed_mps: f64, fpa_deg: f64, target_lon_deg_east: f64) -> AgcState {
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

/// Direct-LEO initial state — synthetic 200 km circular orbit deorbited to
/// FPA = −6°. V = 7900 m/s (slightly super-circular at interface altitude).
/// Target 20° east ≈ 2226 km downrange.
///
/// This is a **synthetic** scenario — no historical Apollo mission ever flew
/// a direct LEO entry. The −6° FPA is therefore not "wrong" per Apollo 8
/// Mission Report; it is a plausible operational value. Kept distinct from
/// the lunar-return setup (which uses Apollo 8 actual −6.48°, see #81).
pub fn setup_state_direct_leo() -> AgcState {
    make_initial_state(7_900.0, -6.0, 20.0)
}

/// Lunar-return initial state — translunar-return entry, Apollo 8 actual
/// entry-interface conditions per Apollo 8 Mission Report NASA TM X-65500
/// Table 3-I:
///
/// - V_EI = 11 000 m/s (Mission Report: 10 825.4 m/s — using 11 000 here as
///   slightly conservative on peak g; ~1.6 % above historical).
/// - FPA_EI = **−6.48°** — Apollo 8 actual; the half-width of the Chapman
///   entry corridor is ≈ ±1.0°, so the previously-used −6.0° gave away
///   roughly half a corridor of trajectory shape (orbital-mechanics review
///   of MS-T7, GitHub issue #81).
/// - Target lon = 45° E (≈ 5004 km downrange Pacific) — synthetic equatorial
///   target; not Apollo 8's actual splashdown (8°N 165°W). The entry
///   guidance is frame-agnostic, so an equatorial setup exercises the same
///   control law.
pub fn setup_state_lunar_return() -> AgcState {
    make_initial_state(11_000.0, -6.48, 45.0)
}

/// Sub-satellite latitude and longitude (radians, east positive) of
/// the current `csm_state.position` projected onto the spherical Earth.
pub fn sub_satellite_lat_lon(state: &AgcState) -> (f64, f64) {
    let pos = state.csm_state.position;
    let r = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
    let lat = (pos[2] / r).asin();
    let lon = pos[1].atan2(pos[0]);
    (lat, lon)
}

/// Drive the V37 P61 → P62 → P63 sequence and coast through entry on the
/// supplied `state` + `hw`. Asserts drogue deploys within `miss_km_tol` km of
/// the target landing site already stored in `state.entry`. If `peak_g_band`
/// is supplied, also asserts the peak sensed g during the coast falls
/// inside the given `(min_g, max_g)` band (#83). If `peak_heat_band` is
/// supplied, asserts the peak Sutton–Graves heat flux falls inside the
/// given `(min_mw_m2, max_mw_m2)` band (#96).
///
/// Inverts the ownership compared to the inline driver in
/// `agc-test/tests/phase_entry.rs::run_entry_phase`: here the caller owns
/// `state` and `hw` so the entry phase can be chained after another mission
/// phase (MS-T7 full mission walkthrough).
pub fn run_entry_phase_scenario(
    state: &mut AgcState,
    hw: &mut agc_sim::SimHardware,
    miss_km_tol: f64,
    peak_g_band: Option<(f64, f64)>,
    peak_heat_band: Option<(f64, f64)>,
) {
    use agc_core::services::average_g::start_servicer;
    use agc_core::services::v_n::Key;
    use agc_sim::scenario::SimDuration;
    use agc_sim::{run_scenario, ScenarioBuilder};

    // ── Phase 1: V37 E61 → V37 E62 → V37 E63 ─────────────────────────────────
    let phase1 = ScenarioBuilder::new("phase_entry/select_p61_p62_p63")
        .comment("entry phase: V37 E61 → V37 E62 → V37 E63")
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(6),
            Key::Digit(1),
            Key::Entr,
        ])
        .expect_major_mode(61)
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(6),
            Key::Digit(2),
            Key::Entr,
        ])
        .expect_major_mode(62)
        // P62 must have staged the SECS CM/SM separation pyro discrete
        // and the do_tick pump must have consumed the flag exactly once.
        .expect_csm_separation_fire_count(1)
        .keys(&[
            Key::Verb,
            Key::Digit(3),
            Key::Digit(7),
            Key::Entr,
            Key::Digit(6),
            Key::Digit(3),
            Key::Entr,
        ])
        .expect_major_mode(63)
        // After advancing to P63 the SECS pyro count must still be 1 —
        // the staging flag is one-shot, not re-armed by P63.
        .expect_csm_separation_fire_count(1)
        .build();
    run_scenario(&phase1, state, hw);

    assert_eq!(
        state.entry.phase,
        agc_core::programs::p61_p67::EntryPhase::PreEntry,
        "init_p63 must leave entry phase = PreEntry"
    );
    assert!(
        state.servicer_exit.is_some(),
        "init_p63 must install entry_servicer_exit"
    );

    // ── Phase 2: SERVICER + atmosphere coast ─────────────────────────────────
    state.csm_state.epoch = state.time;
    start_servicer(state);

    let mut phase2_builder = ScenarioBuilder::new("phase_entry/coast")
        .comment("coast through entry — atmosphere + bank flow on")
        .seed_ground_truth(state.csm_state)
        .enable_atmosphere()
        .advance_coast(SimDuration::seconds(MAX_SCENARIO_DURATION_S as u32))
        .expect_drogue_within(miss_km_tol);
    if let Some((min_g, max_g)) = peak_g_band {
        phase2_builder = phase2_builder.expect_peak_g_in(min_g, max_g);
    }
    if let Some((min_q, max_q)) = peak_heat_band {
        phase2_builder = phase2_builder.expect_peak_heating_in(min_q, max_q);
    }
    let phase2 = phase2_builder.build();
    run_scenario(&phase2, state, hw);

    assert_eq!(
        state.entry.phase,
        agc_core::programs::p61_p67::EntryPhase::Final,
        "entry must end in Final phase"
    );
    assert_eq!(state.alarm.code, 0, "no AGC alarms during entry");
}
