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
    /// Last 10 cycles of `(t, phase, sensed_g, alt_km)`. Useful in
    /// failure messages — `tests/entry_e2e.rs` formats it into the
    /// assertion text.
    pub last_history: Vec<(f64, EntryPhase, f64, f64)>,
    /// Total number of SERVICER cycles executed.
    pub total_cycles: u32,
}

/// Simulate one entry scenario all the way to drogue deploy (or the
/// [`MAX_SCENARIO_DURATION_S`] timeout). No assertions — returns the
/// diagnostics for the caller to inspect.
///
/// `state` must already have the initial CSM state vector, target
/// landing coordinates, MET, and GHA-epoch populated. This helper
/// takes care of `init_p61` → `init_p62` → `init_p63` → `start_servicer`
/// and then ticks the SERVICER loop until exit.
pub fn simulate_to_drogue(mut state: AgcState) -> ScenarioResult {
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
    let mut total_cycles: u32 = 0;

    loop {
        let bank_rad = match state.dap_state.mode {
            DapMode::EntryRoll(b) => b,
            _ => 0.0,
        };
        let ld_command = state.entry.ld_command;

        let dv_inertial = integrator.integrate_cycle(
            state.csm_state.position,
            state.csm_state.velocity,
            ld_command,
            bank_rad,
            SERVICER_PERIOD_S,
        );
        state.pipa_counts = pipa_pulses_for_dv(dv_inertial, &state.pipa_cal);

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

/// Direct-LEO initial state — 200 km circular orbit deorbited to FPA = −6°.
/// V = 7900 m/s (slightly super-circular at interface altitude).
/// Target 20° east ≈ 2226 km downrange.
pub fn setup_state_direct_leo() -> AgcState {
    make_initial_state(7_900.0, -6.0, 20.0)
}

/// Lunar-return initial state — translunar-return entry per
/// `specs/entry-guidance-plan.md` §5 MS-E7. V = 11 000 m/s super-
/// circular; orbit highly elliptical, perigee well below the surface
/// (a ≈ 221 000 km, e ≈ 0.971). Target 45° east ≈ 5004 km downrange
/// (Pacific splashdown).
pub fn setup_state_lunar_return() -> AgcState {
    make_initial_state(11_000.0, -6.0, 45.0)
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
