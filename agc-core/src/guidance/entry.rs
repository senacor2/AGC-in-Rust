//! CM entry guidance — closed-loop P64 / P65 / P67 math (MS-E3 ..).
//!
//! AGC source: `Comanche055/REENTRY_CONTROL.agc`. ENTRY_LEXICON.agc names are
//! preserved in comments and variable names where it aids cross-reference.
//!
//! ## Scope of this module (MS-E3)
//!
//! Four pure functions form the closed-loop step that runs every SERVICER
//! cycle once the 0.05g threshold has tripped:
//!
//! 1. [`predict_range`] — predicted total range from current state to the
//!    point where `V = VFINAL`.
//! 2. [`compute_ld_command`] — HUNTEST/INITROLL Newton iteration on the
//!    up-control reference L/D (`LEWD`); returns this cycle's vertical L/D
//!    command and the iteration state needed for the next cycle.
//! 3. [`resolve_roll`] — convert vertical-L/D command to commanded bank
//!    angle (sign chosen from cross-range error).
//! 4. [`select_phase`] — decide whether to stay in P64 (`EntryPhase::Entry`),
//!    branch to P67 (`Final`) on terminal velocity, or fall through to P66
//!    (`Ballistic`) on a diverged range prediction.
//!
//! Skip-out (P65 UPCONTRL) is deferred to MS-E4.
//!
//! ## Prediction model (MS-E3)
//!
//! The AGC's HUNTEST predicts total range as the analytic sum
//! `ASKEP + ASP1 + ASPUP + ASP3 + ASPDWN` (see REENTRY_CONTROL.agc:654–732).
//! Implementing the full five-component sum requires faithfully reproducing
//! the HUNTEST internal variables (`VL`, `GAMMAL`, `VBARS`, `A0`, `FACT1`,
//! `FACT2`, `Q7`, `DHOOK` …), each with its own AGC fixed-point scale factor.
//! That implementation effort is what we'd expect to validate against a
//! `huntest_cases.json` fixture, and the fixture path is deferred to
//! **MS-E3a** per the user's milestone-scope decision.
//!
//! For MS-E3 we use the AGC's other range model: the **reference profile
//! table** (REENTRY_CONTROL.agc:1410–1467), which the AGC consumes in the
//! final phase (`PREDICT3` block, line 1139) as a tabulated lookup. The
//! `RTOGO` column gives reference range at each velocity sample; the
//! `DRANGE/D(L/D)` column provides the linear sensitivity that ties the
//! prediction to the iterated `LEWD`. The combined formula is
//!
//! ```text
//! ASP = RTOGO(V) + (LEWD − LAD) · DRANGE/D(L/D)(V)
//! ```
//!
//! Both columns are 100 % from `REENTRY_CONTROL.agc`, only the consumer
//! changes. MS-E6 will replace this with the analytic HUNTEST sum when the
//! VirtualAGC HUNTEST fixtures land in MS-E3a — the function signature stays
//! the same, so the swap is a one-file change.

use crate::guidance::entry_tables::{
    lookup_reference, LAD_NOMINAL, LD_CMIN_RATIO, LEWD_INIT, DLEWD_INIT, RANGE_ERR_THRESHOLD_KM,
    VFINAL1_MPS,
};
use crate::navigation::state_vector::inertial_to_earth_fixed;
use crate::navigation::time::met_to_gha;
use crate::programs::p21::R_EARTH;
use crate::programs::p61_p67::EntryPhase;
use crate::AgcState;

/// Nautical miles to kilometres.
///
/// AGC's range tables are in nautical miles; SI conversion is exact at
/// 1 nm = 1.852 km.
pub const NM_TO_KM: f64 = 1.852;

// ── Public API ─────────────────────────────────────────────────────────────────

/// Result of one HUNTEST iteration.
///
/// `compute_ld_command` is a pure function (`&AgcState → LdUpdate`); the
/// SERVICER call-site copies the four updated fields back into `EntryState`.
#[derive(Clone, Copy, Debug)]
pub struct LdUpdate {
    /// Vertical L/D command for this cycle (saturated to `[-LAD, LAD]`).
    pub ld_command: f64,
    /// Updated `LEWD` to persist for the next cycle.
    pub lewd_new: f64,
    /// Updated `DLEWD` (HUNTEST iteration step) for next cycle.
    pub dlewd_new: f64,
    /// `DIFFOLD` for next cycle = the current downrange error (km).
    pub diffold_new_km: f64,
}

/// Predicted total range (km) from the current state to the point where the
/// vehicle decelerates to `VFINAL`.
///
/// Sums the AGC reference profile range (`RTOGO`) plus the L/D-sensitivity
/// correction `(LEWD − LAD) · DRANGE/D(L/D)` — see module-level docs.
///
/// AGC equivalents: REENTRY_CONTROL.agc:1426–1438 (RTOGO column),
/// 1455–1467 (DRANGE/D(L/D) column), 1271 (LEWD), 469 (LAD).
pub fn predict_range(state: &AgcState) -> f64 {
    let v = velocity_mps(state);
    let p = lookup_reference(v);
    let lewd = if state.entry.hunt_initialized {
        state.entry.lewd_ref
    } else {
        LEWD_INIT
    };
    let range_nm = p.range_to_go_nm + (lewd - LAD_NOMINAL) * p.drange_dld_nm;
    range_nm * NM_TO_KM
}

/// Run one HUNTEST iteration and return the new vertical L/D command and
/// updated iteration state.
///
/// Implements the Newton-style step at REENTRY_CONTROL.agc:744–760:
/// ```text
/// DLEWD = DLEWD · DIFF / (DIFFOLD − DIFF)
/// if LEWD + DLEWD < 0:  DLEWD = −LEWD / 2          (line 797 — LEWDPTR clamp)
/// LEWD  = LEWD + DLEWD
/// ```
/// `DIFF` is the downrange error `target_range − predicted_range` in km.
/// On the **first** SERVICER cycle in `EntryPhase::Entry`, the iteration is
/// initialised from `FOREHUNT` (line 861): `LEWD = LEWD_INIT`,
/// `DLEWD = DLEWD_INIT`, `DIFFOLD = 0`.
///
/// The returned `ld_command` is the new `LEWD` saturated to
/// `[−LAD_NOMINAL, +LAD_NOMINAL]` to mirror the AGC `GLIMITER` / LIMITL/D
/// post-clamp (line 1247) — the L/D physically cannot exceed the vehicle's
/// nominal max.
pub fn compute_ld_command(state: &AgcState) -> LdUpdate {
    let diff_km = state.entry.target_range_km - state.entry.predicted_range_km;

    let (lewd_prev, dlewd_prev, diffold) = if state.entry.hunt_initialized {
        (
            state.entry.lewd_ref,
            state.entry.dlewd,
            state.entry.diffold_km,
        )
    } else {
        // FOREHUNT init (REENTRY_CONTROL.agc:861). DIFFOLD starts at 0 so the
        // first cycle behaves as if there is no prior estimate (Newton step
        // collapses to the initial DLEWD).
        (LEWD_INIT, DLEWD_INIT, 0.0)
    };

    // Newton iteration step: DLEWD · DIFF / (DIFFOLD − DIFF).
    // Tiny-denominator guard mirrors the AGC's `BOV` overflow check at
    // line 758 (LEWDOVFL fallback): if the iteration would overflow, freeze
    // DLEWD at its previous value instead.
    let denom = diffold - diff_km;
    let dlewd_step = if denom.abs() > 1e-6 {
        dlewd_prev * diff_km / denom
    } else {
        dlewd_prev
    };

    // LEWDPTR clamp (line 797): if the proposed LEWD would go negative,
    // halve the step instead.
    let dlewd_clamped = if lewd_prev + dlewd_step < 0.0 {
        -lewd_prev * 0.5
    } else {
        dlewd_step
    };

    let lewd_new_raw = lewd_prev + dlewd_clamped;
    // GLIMITER/LIMITL/D post-clamp at line 1271–1285.
    let ld_command = lewd_new_raw.clamp(-LAD_NOMINAL, LAD_NOMINAL);

    LdUpdate {
        ld_command,
        lewd_new: lewd_new_raw,
        dlewd_new: dlewd_clamped,
        diffold_new_km: diff_km,
    }
}

/// Convert a vertical L/D command to a commanded bank angle (rad).
///
/// AGC equivalent: REENTRY_CONTROL.agc:1308 (`L355` block).
/// ```text
/// ROLLC = acos( clamp(L/D₁ / LAD, −1, +1) )
/// ```
/// The magnitude comes from the ratio `ld_cmd / LAD`. The **sign** is set by
/// the lateral switch `L353` (line 1296): we bank toward whichever side
/// reduces the cross-range error. Apollo convention: positive bank = right
/// roll. `crossrange_km > 0` means we are right of the great-circle track to
/// target, so we bank **left** (negative sign).
///
/// The hysteresis band `L/DCMINR = LAD · cos(15°)` (line 1614) prevents tiny
/// cross-range errors from triggering bank reversals.
pub fn resolve_roll(state: &AgcState, ld_cmd: f64) -> f64 {
    let ratio = (ld_cmd / LAD_NOMINAL).clamp(-1.0, 1.0);
    let magnitude = libm::acos(ratio);

    // Lateral switch: bank toward the trajectory plane unless |cross-range|
    // is inside the LD_CMIN_RATIO hysteresis band (in km-equivalent through
    // the great-circle relation `crossrange_km ≈ LAD * R_EARTH_km * angle`,
    // we simplify to a fixed band).
    let crossrange_band_km = LD_CMIN_RATIO * R_EARTH * 0.001 * 1.0e-3; // ~6.1e-3 km
    if state.entry.crossrange_km.abs() < crossrange_band_km {
        magnitude // hold sign-positive (matches AGC trim-attitude default)
    } else if state.entry.crossrange_km > 0.0 {
        -magnitude
    } else {
        magnitude
    }
}

/// Decide the next entry-guidance phase.
///
/// Returns `Some(next_phase)` to request a transition, or `None` to stay in
/// `Entry`. Three outcomes:
/// - `Some(EntryPhase::Final)` once `V < VFINAL1` — terminal velocity has
///   been reached, hand off to P67 / PREDICT3 (REENTRY_CONTROL.agc:431).
/// - `Some(EntryPhase::Ballistic)` if the predicted range diverges from the
///   actual range to target by more than `RANGE_ERR_THRESHOLD_KM` — the
///   guidance has lost track of the trajectory, fall through to P66.
/// - `None` while nominal closed-loop guidance can continue.
///
/// Skip-out (P65 UPCONTRL) detection is deferred to MS-E4.
pub fn select_phase(state: &AgcState) -> Option<EntryPhase> {
    let v = velocity_mps(state);
    if v < VFINAL1_MPS {
        return Some(EntryPhase::Final);
    }
    let range_err_km =
        (state.entry.target_range_km - state.entry.predicted_range_km).abs();
    if range_err_km > RANGE_ERR_THRESHOLD_KM {
        return Some(EntryPhase::Ballistic);
    }
    None
}

// ── Helpers (private) ──────────────────────────────────────────────────────────

/// Inertial speed `|v|` (m/s).
fn velocity_mps(state: &AgcState) -> f64 {
    let v = state.csm_state.velocity;
    libm::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

/// Sub-satellite latitude / longitude (rad) under the current ECI position.
///
/// Uses the same GHA → ECEF rotation as `compute_range_to_go_km` so target
/// position and cross-range share a consistent frame.
fn sub_satellite_lat_lon(state: &AgcState) -> Option<(f64, f64)> {
    let pos = state.csm_state.position;
    let r2 = pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2];
    if r2 == 0.0 {
        return None;
    }
    let r = libm::sqrt(r2);
    let gha = met_to_gha(state.time, state.gha_epoch_rad);
    let pos_ef = inertial_to_earth_fixed(pos, gha);
    let lat = libm::asin(pos_ef[2] / r);
    let lon = libm::atan2(pos_ef[1], pos_ef[0]);
    Some((lat, lon))
}

/// Cross-range distance (km) from the current sub-satellite point to the
/// great-circle through the target, **signed**: positive = right of the
/// north-pointing direction to the target.
///
/// For MS-E3 we use the small-angle approximation
/// `crossrange ≈ R_EARTH · sin(Δlon) · cos(target_lat)`,
/// where `Δlon = current_lon − target_lon`. This is accurate to better than
/// 1 % inside the entry corridor (last 1000 km) where the bank command is
/// applied.
pub(crate) fn crossrange_km(state: &AgcState) -> f64 {
    let Some((_lat, lon)) = sub_satellite_lat_lon(state) else {
        return 0.0;
    };
    let dlon = lon - state.entry.target_lon_rad;
    R_EARTH * libm::sin(dlon) * libm::cos(state.entry.target_lat_rad) * 1.0e-3
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::guidance::entry_tables::REFERENCE_PROFILE;
    use crate::programs::p61_p67::EntryPhase;

    /// Slowest and fastest sampled velocities in the AGC reference profile,
    /// exposed for tests that sweep across the table.
    const REFERENCE_PROFILE_V_FIRST: f64 = REFERENCE_PROFILE[0].velocity_mps;
    const REFERENCE_PROFILE_V_LAST: f64 =
        REFERENCE_PROFILE[REFERENCE_PROFILE.len() - 1].velocity_mps;

    /// Common fixture: CSM at 6500 km radius on +X, velocity along +Y, target
    /// on the equator at the sub-satellite longitude (range-to-go = 0).
    fn fixture(v_mps: f64) -> AgcState {
        let mut state = AgcState::new();
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [0.0, v_mps, 0.0];
        state.entry.phase = EntryPhase::Entry;
        state.entry.sensed_acceleration_g = 0.1;
        // Pre-stage MS-E2 values so MS-E3 functions read consistent state.
        state.entry.target_range_km = 1_500.0; // ~810 nm — typical 5g entry
        state.entry.predicted_range_km = predict_range(&state);
        state
    }

    // ── predict_range ─────────────────────────────────────────────────────────

    /// TC-MSE3-PR-1: predicted range at the slowest table sample (~303 m/s,
    /// near drogue) is a small number (≤ 20 km), matching `RTOGO[0]` ≈ 2.2 nm.
    #[test]
    fn tc_mse3_pr_1_at_min_table_v_minimal() {
        let state = fixture(303.0);
        let rng = predict_range(&state);
        // RTOGO at i=0 = .0008067 · 2700 nm ≈ 2.18 nm ≈ 4.03 km; LEWD-LAD
        // correction is small at the endpoint.
        assert!(
            rng < 20.0,
            "predicted range near drogue should be ≤ 20 km, got {rng} km"
        );
    }

    /// TC-MSE3-PR-2: predicted range at a hyper-velocity entry (~10 km/s) is
    /// large, matching `RTOGO[12]` ≈ 794 nm.
    #[test]
    fn tc_mse3_pr_2_at_high_v_large() {
        let state = fixture(10_000.0);
        let rng = predict_range(&state);
        // Interpolation between i=11 (7163 m/s, RTOGO ≈ 643 nm) and i=12
        // (10 668 m/s, RTOGO ≈ 794 nm) plus the LEWD correction lands around
        // ~1000 km. Wide band — this is a sanity-of-shape check, not a
        // numerical fixture (deferred to MS-E3a).
        assert!(
            (500.0..2_500.0).contains(&rng),
            "predicted range at 10 km/s out of expected band: {rng} km"
        );
    }

    /// TC-MSE3-PR-3: predicted range is monotone non-decreasing in V.
    #[test]
    fn tc_mse3_pr_3_monotonic_in_v() {
        let v_lo = REFERENCE_PROFILE_V_FIRST;
        let v_hi = REFERENCE_PROFILE_V_LAST;
        let mut prev = predict_range(&fixture(v_lo));
        for step in 1..=10 {
            let v = v_lo + (v_hi - v_lo) * (step as f64) / 10.0;
            let cur = predict_range(&fixture(v));
            assert!(cur >= prev, "non-monotone at v={v}: prev={prev}, cur={cur}");
            prev = cur;
        }
    }

    /// TC-MSE3-PR-4: increasing LEWD increases predicted range (positive
    /// sensitivity DRANGE/D(L/D) in the AGC table).
    #[test]
    fn tc_mse3_pr_4_ld_sensitivity() {
        let mut state = fixture(7_500.0);
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = 0.10;
        let r_lo = predict_range(&state);
        state.entry.lewd_ref = 0.25;
        let r_hi = predict_range(&state);
        assert!(
            r_hi > r_lo,
            "higher LEWD must predict longer range, got r_lo={r_lo} r_hi={r_hi}"
        );
    }

    // ── compute_ld_command ────────────────────────────────────────────────────

    /// TC-MSE3-LD-1: first cycle (FOREHUNT init) uses LEWD_INIT and DLEWD_INIT.
    #[test]
    fn tc_mse3_ld_1_first_cycle_uses_forehunt_init() {
        let mut state = fixture(7_500.0);
        state.entry.hunt_initialized = false;
        // Synthesize a small DIFF: target slightly bigger than prediction.
        state.entry.target_range_km = state.entry.predicted_range_km + 50.0;

        let upd = compute_ld_command(&state);
        // DIFFOLD=0, so denom = -DIFF, step = DLEWD_INIT · DIFF/(-DIFF) = -DLEWD_INIT
        // = 0.05. Then LEWD_new = LEWD_INIT + 0.05 = 0.20.
        assert!(
            (upd.lewd_new - 0.20).abs() < 1e-9,
            "expected lewd_new = 0.20, got {}",
            upd.lewd_new
        );
        assert!(
            (upd.diffold_new_km - 50.0).abs() < 1e-9,
            "expected DIFFOLD = 50 km, got {}",
            upd.diffold_new_km
        );
    }

    /// TC-MSE3-LD-2: Newton update with a known (DIFF, DIFFOLD, DLEWD) triple
    /// matches the AGC formula DLEWD · DIFF / (DIFFOLD − DIFF).
    #[test]
    fn tc_mse3_ld_2_newton_update() {
        let mut state = fixture(7_500.0);
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = 0.20;
        state.entry.dlewd = 0.05;
        state.entry.diffold_km = 100.0;
        state.entry.target_range_km = state.entry.predicted_range_km + 40.0;

        let upd = compute_ld_command(&state);
        // DIFF = 40, DIFFOLD = 100. Step = 0.05 · 40 / (100 − 40) = 0.0333…
        let expected_step = 0.05 * 40.0 / 60.0;
        assert!(
            (upd.dlewd_new - expected_step).abs() < 1e-9,
            "expected DLEWD = {expected_step}, got {}",
            upd.dlewd_new
        );
        assert!(
            (upd.lewd_new - (0.20 + expected_step)).abs() < 1e-9,
            "lewd_new mismatch: {}",
            upd.lewd_new
        );
    }

    /// TC-MSE3-LD-3: output saturates to LAD_NOMINAL when the iteration
    /// would otherwise drive L/D past the vehicle maximum.
    #[test]
    fn tc_mse3_ld_3_saturates_to_lad() {
        let mut state = fixture(9_000.0);
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = 0.25;
        state.entry.dlewd = 0.20;
        // Force a converging Newton step of +0.20: DIFFOLD=200, DIFF=100,
        // denom=100, step = 0.20 · 100 / 100 = 0.20. lewd_new_raw = 0.45 > LAD.
        state.entry.diffold_km = 200.0;
        state.entry.target_range_km = state.entry.predicted_range_km + 100.0;

        let upd = compute_ld_command(&state);
        assert!(
            upd.lewd_new > LAD_NOMINAL,
            "lewd_new unclamped should exceed LAD, got {}",
            upd.lewd_new
        );
        assert!(
            (upd.ld_command - LAD_NOMINAL).abs() < 1e-12,
            "ld_command must clamp to LAD, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE3-LD-4: negative LEWD clamp (LEWDPTR) — if the proposed step
    /// would drive LEWD below zero, the step is replaced by `-LEWD/2`.
    #[test]
    fn tc_mse3_ld_4_lewdptr_clamp() {
        let mut state = fixture(7_500.0);
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = 0.10;
        state.entry.dlewd = -0.5; // huge negative step
        state.entry.diffold_km = -100.0;
        state.entry.target_range_km = state.entry.predicted_range_km - 50.0; // DIFF = -50

        let upd = compute_ld_command(&state);
        // raw step = -0.5 · (-50) / (-100 − (-50)) = -0.5 · -50 / -50 = -0.5
        // LEWD + step = 0.10 - 0.5 = -0.4 < 0 → step replaced by -LEWD/2 = -0.05
        assert!(
            (upd.dlewd_new - (-0.05)).abs() < 1e-9,
            "expected DLEWD_new = -0.05 after LEWDPTR clamp, got {}",
            upd.dlewd_new
        );
        assert!(
            (upd.lewd_new - 0.05).abs() < 1e-9,
            "expected lewd_new = 0.05, got {}",
            upd.lewd_new
        );
    }

    // ── resolve_roll ──────────────────────────────────────────────────────────

    /// TC-MSE3-RR-1: at L/D = LAD (max lift up), commanded bank = 0 rad.
    #[test]
    fn tc_mse3_rr_1_max_lift_zero_bank() {
        let state = fixture(7_500.0);
        let bank = resolve_roll(&state, LAD_NOMINAL);
        assert!(bank.abs() < 1e-9, "expected bank = 0, got {bank} rad");
    }

    /// TC-MSE3-RR-2: at L/D = 0 (knife-edge), commanded |bank| = π/2.
    #[test]
    fn tc_mse3_rr_2_knife_edge_90deg() {
        let state = fixture(7_500.0);
        let bank = resolve_roll(&state, 0.0);
        assert!(
            (bank.abs() - core::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "expected |bank| = π/2, got {bank} rad"
        );
    }

    /// TC-MSE3-RR-3: at L/D = -LAD (max lift down), commanded |bank| = π.
    #[test]
    fn tc_mse3_rr_3_full_lift_down_180deg() {
        let state = fixture(7_500.0);
        let bank = resolve_roll(&state, -LAD_NOMINAL);
        assert!(
            (bank.abs() - core::f64::consts::PI).abs() < 1e-9,
            "expected |bank| = π, got {bank} rad"
        );
    }

    /// TC-MSE3-RR-4: sign convention — positive cross-range gives negative
    /// bank, negative cross-range gives positive bank.
    #[test]
    fn tc_mse3_rr_4_sign_from_crossrange() {
        let mut state = fixture(7_500.0);
        state.entry.crossrange_km = 50.0; // right of track
        let b_pos = resolve_roll(&state, 0.0);
        assert!(b_pos < 0.0, "expected negative bank, got {b_pos}");

        state.entry.crossrange_km = -50.0; // left of track
        let b_neg = resolve_roll(&state, 0.0);
        assert!(b_neg > 0.0, "expected positive bank, got {b_neg}");
    }

    // ── select_phase ──────────────────────────────────────────────────────────

    /// TC-MSE3-SP-1: below VFINAL1, transition to Final.
    #[test]
    fn tc_mse3_sp_1_below_vfinal1_to_final() {
        let state = fixture(VFINAL1_MPS - 100.0);
        assert_eq!(select_phase(&state), Some(EntryPhase::Final));
    }

    /// TC-MSE3-SP-2: above VFINAL1 with small range error stays in Entry
    /// (None = no transition).
    #[test]
    fn tc_mse3_sp_2_nominal_no_transition() {
        let mut state = fixture(VFINAL1_MPS + 500.0);
        // Make sure target_range matches predicted_range — DIFF ≈ 0.
        state.entry.target_range_km = state.entry.predicted_range_km;
        assert_eq!(select_phase(&state), None);
    }

    /// TC-MSE3-SP-3: large divergence transitions to Ballistic.
    #[test]
    fn tc_mse3_sp_3_diverged_to_ballistic() {
        let mut state = fixture(VFINAL1_MPS + 500.0);
        // Force a 1500-km range error — well above 500-km threshold.
        state.entry.target_range_km = state.entry.predicted_range_km + 1_500.0;
        assert_eq!(select_phase(&state), Some(EntryPhase::Ballistic));
    }

    // ── crossrange helper ─────────────────────────────────────────────────────

    /// TC-MSE3-CR-1: zero cross-range at target sub-satellite longitude.
    #[test]
    fn tc_mse3_cr_1_zero_at_target() {
        let mut state = AgcState::new();
        state.csm_state.position = [6_500_000.0, 0.0, 0.0]; // lat=0, lon=0
        state.csm_state.velocity = [0.0, 7_500.0, 0.0];
        // target_lat_rad = 0, target_lon_rad = 0 by default.
        let cr = crossrange_km(&state);
        assert!(cr.abs() < 1e-6, "expected 0 crossrange, got {cr} km");
    }

    /// TC-MSE3-CR-2: positive cross-range for 1° eastward of target.
    #[test]
    fn tc_mse3_cr_2_positive_east() {
        let mut state = AgcState::new();
        // CSM at lon = +1°, target at lon = 0 → CSM is east of target →
        // positive crossrange (right of north-pointing direction).
        let lon = 1.0_f64.to_radians();
        state.csm_state.position = [6_500_000.0 * libm::cos(lon), 6_500_000.0 * libm::sin(lon), 0.0];
        let cr = crossrange_km(&state);
        // Expected ≈ R_EARTH · sin(1°) · 1 ≈ 111 km
        assert!(cr > 100.0 && cr < 120.0, "expected ~111 km, got {cr} km");
    }
}
