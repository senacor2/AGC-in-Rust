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
//! ## Prediction model
//!
//! [`predict_range`] implements the AGC's `RANGER` block at
//! `REENTRY_CONTROL.agc:500–732` as a faithful SI translation:
//!
//! 1. Set up HUNTEST intermediates: `V₁`, `A₀`, `ALP`, `FACT1`, `FACT2`,
//!    `VL`, `VBARS`, `GAMMAL1`, `GAMMAL` (lines 500–650). The second-order
//!    `DHOOK / AHOOKDV` correction (lines 616–639) is approximated as
//!    `GAMMAL = GAMMAL1`; including the full correction is part of the
//!    VirtualAGC fixture-match pass (see [`predict_range_table`] for the
//!    PREDICT3 lookup we cross-validate against).
//! 2. RANGER (lines 654–732):
//!    `ASP = ASKEP + ASP1 + ASPUP + ASP3 + ASPDWN`
//!    - `ASKEP` — Kepler-arc range from pull-out to entry interface.
//!    - `ASP1`  — final-phase range, linear in `VL`.
//!    - `ASPUP` — up-phase range, logarithmic in `V₁²·Q7 / (VBARS·A₀)`.
//!    - `ASP3`  — γ-correction, linear in `GAMMAL`.
//!    - `ASPDWN`— pull-out range, `KC3·RDOT·V/(A₀·LAD)`.
//! 3. Sum is in revolutions of the Earth (`1 rev = 21 600 NM ≈ 40 003 km`)
//!    and is converted to km on output.
//!
//! [`predict_range_table`] is the AGC `PREDICT3` lookup
//! (`REENTRY_CONTROL.agc:1369–1467`) kept as a private cross-check oracle.
//! Both methods compute the same physical quantity by different routes.

use crate::guidance::entry_tables::{
    lookup_reference, C12_NM, DLEWD_INIT, FPSS_805_MPS2, KC3_NM_PER_M2_PER_S2, LAD_NOMINAL,
    LD_CMIN_RATIO, LEWD_INIT, Q2_NM, Q3_NM_PER_MPS, Q5_NM_PER_RAD, Q6_RAD, Q7F_AGC,
    RANGE_ERR_THRESHOLD_KM, TWO_C1_HS_AGC, VFINAL1_MPS, VLMIN_MPS, VQUIT_MPS, VSAT_MPS,
};
use crate::programs::p61_p67::G0_MPS2;
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
/// Translates `REENTRY_CONTROL.agc:500–732` (HUNTEST + RANGER) into SI.
///
/// **Numerical limitations** (will tighten in the MS-E3a fixture-match pass):
/// - The `DHOOK / AHOOKDV` correction (AGC lines 616–639) is omitted —
///   `GAMMAL = GAMMAL1` here. This is a second-order accuracy term.
/// - When the HUNTEST setup produces a degenerate intermediate (e.g.
///   `1 − ALP ≈ 0`, `VL < VLMIN`, `Q7·FACT2 + ALP < 0`), the function falls
///   back to [`predict_range_table`] so the SERVICER cycle always returns
///   a finite, monotone-in-V prediction.
pub fn predict_range(state: &AgcState) -> f64 {
    let v = velocity_mps(state);
    let rdot = state.entry.r_dot_mps;
    let lewd = if state.entry.hunt_initialized {
        state.entry.lewd_ref
    } else {
        LEWD_INIT
    };
    let lad = LAD_NOMINAL;
    // `D` (drag, AGC line 502): convert sensed-g to AGC's `805 FPSS` scale.
    let d_agc = state.entry.sensed_acceleration_g * G0_MPS2 / FPSS_805_MPS2;

    // Velocities in AGC-stored scaling `V / (2·VSAT)`, dimensionless.
    let v_n = v / (2.0 * VSAT_MPS);
    let rdot_n = rdot / (2.0 * VSAT_MPS);
    if v_n < 1e-6 {
        return predict_range_table(state);
    }

    // TEM1B = LAD if RDOT<0 else LEWD (REENTRY_CONTROL.agc:507–510).
    let tem1b = if rdot < 0.0 { lad } else { lewd };
    if tem1b.abs() < 1e-6 {
        return predict_range_table(state);
    }

    // V1 = V + RDOT/TEM1B (line 513).
    let v1_n = v_n + rdot_n / tem1b;

    // A0 = (V1/V)² · (D + RDOT² / (TEM1B · 2·C1·HS))  (lines 519–528).
    let v1_over_v = v1_n / v_n;
    let a0_agc =
        v1_over_v * v1_over_v * (d_agc + rdot_n * rdot_n / (tem1b * TWO_C1_HS_AGC));

    // V1LEAD: if L/D < 0, V1 −= VQUIT (lines 537–545).
    let v1_n_after_lead = if state.entry.ld_command < 0.0 {
        v1_n - VQUIT_MPS / (2.0 * VSAT_MPS)
    } else {
        v1_n
    };
    if v1_n_after_lead.abs() < 1e-6 {
        return predict_range_table(state);
    }

    // ALP = 2·C1·HS · A0 / (LEWD · V1²)  (lines 547–556).
    let alp = TWO_C1_HS_AGC * a0_agc / lewd / (v1_n_after_lead * v1_n_after_lead);

    // FACT1 = V1 / (1 − ALP)  (line 558–561).
    if (1.0 - alp).abs() < 1e-6 {
        return predict_range_table(state);
    }
    let fact1 = v1_n_after_lead / (1.0 - alp);

    // FACT2 = ALP·(ALP − 1) / A0  (line 564–569).
    if a0_agc.abs() < 1e-6 {
        return predict_range_table(state);
    }
    let fact2 = alp * (alp - 1.0) / a0_agc;

    // VL = FACT1 · (1 − sqrt(Q7·FACT2 + ALP))  (line 571–578).
    let inner = Q7F_AGC * fact2 + alp;
    if inner < 0.0 {
        return predict_range_table(state);
    }
    let vl_n = fact1 * (1.0 - libm::sqrt(inner));
    let vl_real_mps = vl_n * 2.0 * VSAT_MPS;
    if vl_real_mps < VLMIN_MPS || vl_n.abs() < 1e-6 {
        return predict_range_table(state);
    }

    // VBARS = stored VL² = (real VL / 2·VSAT)² · 4 → in AGC-stored.
    let vbars = vl_n * vl_n;

    // GAMMAL1 = LEWD · (V1 − VL) / VL  (line 580–585).
    let gammal1 = lewd * (v1_n_after_lead - vl_n) / vl_n;
    if gammal1.abs() < 1e-9 {
        return predict_range_table(state);
    }

    // DHOOK / AHOOKDV correction skipped — GAMMAL = GAMMAL1 (see fn docs).
    let gammal = gammal1;

    // ── RANGER ────────────────────────────────────────────────────────────────
    // COSG/2 = (1 − GAMMAL²) / 2  (truncated Taylor, line 654–657).
    let cosg_over_2 = 0.5 * (1.0 - gammal * gammal);

    // E/4 = sqrt( (VBARS − 1/2) · VBARS · (COSG/2)² · 4 + 1/16 )  (line 660–668).
    let bracket =
        (vbars - 0.5) * vbars * cosg_over_2 * cosg_over_2 * 4.0 + 1.0 / 16.0;
    if bracket <= 0.0 {
        return predict_range_table(state);
    }
    let e_over_4 = libm::sqrt(bracket);
    if e_over_4 < 1e-9 {
        return predict_range_table(state);
    }

    // ASKEP/2 = arcsin(VBARS · COSG/2 · GAMMAL / (E/4))  (line 671–676).
    // SL1 (×2) gives ASKEP. Convert AGC-stored revolutions (where 1 = full
    // revolution) to the same revolution unit we sum below.
    let arg = (vbars * cosg_over_2 * gammal / e_over_4).clamp(-1.0, 1.0);
    let askep_rev = libm::asin(arg) / core::f64::consts::PI;

    // ASP1 = Q2 + Q3·VL  (line 680). Q2_NM and Q3_NM_PER_MPS in nm.
    let asp1_rev = (Q2_NM + Q3_NM_PER_MPS * vl_real_mps) / 21_600.0;

    // ASPUP = −C12 · log(V1²·Q7 / (VBARS·A0)) / GAMMAL1  (line 688–699).
    let log_arg = (v1_n_after_lead * v1_n_after_lead * Q7F_AGC
        / (vbars * a0_agc))
        .abs()
        .max(1e-12);
    let aspup_rev = -C12_NM * libm::log(log_arg) / gammal1 / 21_600.0;

    // ASPDWN = KC3 · RDOT · V / (A0 · LAD)  (line 701–710).
    let a0_real_mps2 = a0_agc * FPSS_805_MPS2;
    let aspdwn_nm = if a0_real_mps2.abs() < 1e-3 {
        0.0
    } else {
        KC3_NM_PER_M2_PER_S2 * rdot * v / (a0_real_mps2 * lad)
    };
    let aspdwn_rev = aspdwn_nm / 21_600.0;

    // ASP3 = Q5 · (Q6 − GAMMAL)  (line 712–717).
    let asp3_rev = Q5_NM_PER_RAD * (Q6_RAD - gammal) / 21_600.0;

    // Total = sum (revolutions) → km. 1 rev = 2π·R_EARTH.
    let asp_rev = askep_rev + asp1_rev + aspup_rev + asp3_rev + aspdwn_rev;
    let asp_km = asp_rev * 2.0 * core::f64::consts::PI * R_EARTH * 1.0e-3;

    // If the analytic result is patently unphysical (negative, ridiculous),
    // fall back to the table. This guards downstream
    // `compute_ld_command` / `select_phase` against NaN propagation.
    if !asp_km.is_finite() || !(0.0..=100_000.0).contains(&asp_km) {
        return predict_range_table(state);
    }
    asp_km
}

/// AGC `PREDICT3` tabulated range prediction (REENTRY_CONTROL.agc:1369–1467).
///
/// Cross-check oracle for [`predict_range`]. Used directly by P67 final-phase
/// guidance (MS-E6), and by [`predict_range`] as a graceful-degradation
/// fallback when the analytic HUNTEST setup produces a degenerate
/// intermediate.
///
/// Formula:
/// ```text
/// ASP = RTOGO(V) + (LEWD − LAD) · DRANGE/D(L/D)(V)
/// ```
pub(crate) fn predict_range_table(state: &AgcState) -> f64 {
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

    /// TC-MSE3A-PR-1: the analytic 5-component sum and the PREDICT3 table
    /// lookup agree in order of magnitude at hyper-velocity entry.
    ///
    /// They are different algorithms targeting the same physical quantity;
    /// agreement to a factor of ~4 (analytic is allowed to be 4× larger or
    /// smaller than the table) is enough to assert "same ballpark". The
    /// tolerance will be tightened to ~1% when VAGC fixtures land in
    /// Stage B of MS-E3a.
    #[test]
    fn tc_mse3a_pr_1_analytic_vs_table_at_high_v() {
        let v_samples = [VFINAL1_MPS + 200.0, 9_000.0, 10_000.0];
        for &v in &v_samples {
            let mut state = fixture(v);
            // Use a steeper descent than the default fixture so the analytic
            // HUNTEST setup yields a well-defined VL > VLMIN.
            state.csm_state.position = [6_500_000.0, 0.0, 0.0];
            state.csm_state.velocity = [-200.0, v, 0.0];
            state.entry.r_dot_mps = -200.0;
            state.entry.hunt_initialized = true;
            state.entry.lewd_ref = LEWD_INIT;

            let r_analytic = predict_range(&state);
            let r_table = predict_range_table(&state);
            assert!(
                r_analytic > 0.0 && r_table > 0.0,
                "both ranges must be positive at v={v}: analytic={r_analytic}, table={r_table}"
            );
            let ratio = r_analytic / r_table;
            assert!(
                (0.25..=4.0).contains(&ratio),
                "analytic vs table ratio out of band at v={v}: r_analytic={r_analytic} r_table={r_table} ratio={ratio}"
            );
        }
    }

    /// TC-MSE3A-PR-2: explicit degenerate-input fallback to the table.
    ///
    /// At velocities below VLMIN the HUNTEST setup cannot produce a valid VL,
    /// and `predict_range` is expected to fall back to `predict_range_table`.
    /// We verify the fallback returns identical values for a sub-VLMIN state.
    #[test]
    fn tc_mse3a_pr_2_fallback_below_vlmin() {
        let state = fixture(VLMIN_MPS - 500.0);
        let r_analytic = predict_range(&state);
        let r_table = predict_range_table(&state);
        assert!(
            (r_analytic - r_table).abs() < 1e-9,
            "below VLMIN, analytic must delegate to table: analytic={r_analytic} table={r_table}"
        );
    }

    /// TC-MSE3A-PR-3: NaN-guard. Any pathological intermediate (zero LEWD,
    /// 1 − ALP ≈ 0, etc.) must not propagate NaN to the SERVICER cycle.
    #[test]
    fn tc_mse3a_pr_3_no_nan() {
        // Sweep pathological inputs and verify all are finite.
        for &v in &[
            0.0,
            100.0,
            VLMIN_MPS,
            VFINAL1_MPS,
            9_000.0,
            10_500.0,
            20_000.0,
        ] {
            for &lewd in &[0.0, 1e-9, 0.05, LEWD_INIT, 0.30, -0.10] {
                let mut state = fixture(v);
                state.entry.hunt_initialized = true;
                state.entry.lewd_ref = lewd;
                let r = predict_range(&state);
                assert!(
                    r.is_finite(),
                    "predict_range produced non-finite at v={v}, lewd={lewd}: {r}"
                );
            }
        }
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
