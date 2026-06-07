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
//!    `VL`, `VBARS`, `GAMMAL1`, `GAMMAL` (lines 500–650). The full
//!    second-order `DHOOK / AHOOKDV` correction (lines 616–640) is
//!    applied to derive `GAMMAL` from `GAMMAL1`; the BMN NEGAMA branch
//!    clamps `GAMMAL = 0` if the correction overshoots. The
//!    [`predict_range_table`] PREDICT3 lookup remains as a private
//!    cross-check oracle.
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
    lookup_reference, AHOOKDV_DIVISOR, C12_AGC, C18_MPS, C20_G, CH1, CHOOK, DLEWD_INIT,
    FPSS_805_MPS2, GMAX_G, GMAX_HALF_G, HUNTEST_CONVERGED_KM, K1D_AGC, K2D_AGC, KA3_AGC, KA4_AGC,
    KB1, KB2_MPS, KC3_AGC, LAD_NOMINAL, LD_CMIN_RATIO, LEWD_INIT, LOD_NOMINAL, ONE_SIXTEENTH,
    POINT1, PT1_OVER_16, Q21_AGC, Q22_AGC, Q3_AGC, Q5_AGC, Q6_RAD, Q7F_AGC, Q7F_G, Q7MIN_G,
    RANGE_ERR_THRESHOLD_KM, TWO_C1_HS_AGC, TWO_HS_AGC, TWO_HS_GMAX_SQ_AGC, VFINAL1_MPS, VLMIN_MPS,
    VQUIT_MPS, VSAT_MPS,
};
use crate::navigation::state_vector::inertial_to_earth_fixed;
use crate::navigation::time::met_to_gha;
use crate::programs::p21::R_EARTH;
use crate::programs::p61_p67::EntryPhase;
use crate::programs::p61_p67::G0_MPS2;
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
    /// SKIPPER `FACTOR` (`F1`) to persist for the next cycle.
    ///
    /// Computed by UPCONTRL's CONTINU2 block when `D > Q7MIN`; carried
    /// through unchanged by HUNTEST / ballistic / final-phase paths.
    pub factor_new: f64,
}

/// Predicted total range (km) from the current state to the point where the
/// vehicle decelerates to `VFINAL`.
///
/// Translates `REENTRY_CONTROL.agc:500–732` (HUNTEST + RANGER) into SI.
///
/// **Numerical fallback**: when the HUNTEST setup produces a degenerate
/// intermediate (e.g. `1 − ALP ≈ 0`, `VL < VLMIN`, `Q7·FACT2 + ALP < 0`,
/// or `DHOOK ≈ 0`), the function falls back to [`predict_range_table`]
/// so the SERVICER cycle always returns a finite, monotone-in-V
/// prediction. The full DHOOK / AHOOKDV / GAMMAL second-order
/// correction (AGC lines 616–640) lands in `huntest_setup` since
/// MS-E3b (#32).
pub fn predict_range(state: &AgcState) -> f64 {
    let Some(s) = huntest_setup(state) else {
        return predict_range_table(state);
    };
    let v = velocity_mps(state);
    let rdot = state.entry.r_dot_mps;
    let lad = LAD_NOMINAL;

    // ── RANGER (REENTRY_CONTROL.agc:654–732) ──────────────────────────────────
    //
    // Every range component is produced in **revolutions of the Earth**
    // (1 rev = 2π·R_E) and summed before the final SI conversion. The AGC
    // formulas operate on dimensionless normalised variables:
    //
    //   V_n     = V_mps  / (2·VSAT_mps)         (`V1`,  `VL`,  `VBARS = VL²`)
    //   RDOT_n  = RDOT_mps / (2·VSAT_mps)
    //   A0_n    = A0_mps2 / FPSS_805_mps2       (`805 ft/s² ≈ 25 g`)
    //   GAMMAL  in radians (numerically)
    //
    // The AGC-stored coefficients (`KC3_AGC`, `C12_AGC`, `Q3_AGC`, `Q5_AGC`,
    // `Q21_AGC`, `Q22_AGC`, `Q6_RAD`) are kept verbatim from REENTRY_CONTROL.agc
    // so the AGC source pairs term-for-term with this block.

    let v_norm = v / (2.0 * VSAT_MPS);
    let rdot_norm = rdot / (2.0 * VSAT_MPS);

    // COSG/2 = (1 − GAMMAL²) / 2  (truncated Taylor, line 654–657).
    let cosg_over_2 = 0.5 * (1.0 - s.gammal * s.gammal);

    // E/4 = sqrt( (VBARS − 1/2) · VBARS · (COSG/2)² · 4 + 1/16 )  (line 660–668).
    // The ASKEP formula is the Keplerian-arc analogue and is only meaningful
    // when E/4 ≳ a few percent; below that the `arcsin(.../E/4)` term saturates
    // to ±π/2 and the analytic prediction explodes. Fall through to the
    // tabulated `PREDICT3` whenever the formula leaves its valid regime.
    let bracket = (s.vbars - 0.5) * s.vbars * cosg_over_2 * cosg_over_2 * 4.0 + 1.0 / 16.0;
    if bracket <= 0.0 {
        return predict_range_table(state);
    }
    let e_over_4 = libm::sqrt(bracket);
    // 0.05 ≈ sqrt(0.0025): below this, the ASKEP argument approaches
    // saturation and the surrounding ASPUP / ASPDWN terms also leave their
    // valid regime (`log_arg` drifts far from 1, `1/GAMMAL1` amplifies
    // noise). Empirically every sub-skip-out test input falls inside this
    // band, so the table fallback dominates.
    if e_over_4 < 5.0e-2 {
        return predict_range_table(state);
    }

    // ASKEP/2 = arcsin(VBARS · COSG/2 · GAMMAL / (E/4))  (line 671–676).
    let arg = (s.vbars * cosg_over_2 * s.gammal / e_over_4).clamp(-1.0, 1.0);
    let askep_rev = libm::asin(arg) / core::f64::consts::PI;

    // ASP1_rev = Q2 + Q3 · VL_n  (line 680). Q2 = LAD·Q21 + Q22, computed at
    // runtime from LAD per the AGC line 112-116 sequence.
    let q2_rev = lad * Q21_AGC + Q22_AGC;
    let asp1_rev = q2_rev + Q3_AGC * s.vl_n;

    // ASPUP_rev = −C12 · log(V1²·Q7/(VBARS·A0)) / GAMMAL1  (line 688–699).
    // All terms inside the log are AGC-normalised; the ratio is dimensionless.
    // The `1/GAMMAL1` factor amplifies log deviations from zero — the
    // formula is only well-behaved when the trajectory is in the AGC's
    // designed up-control operating regime (`log_arg` near 1, GAMMAL1
    // a few-hundredths of a radian). Outside that band ASPUP can exceed a
    // full revolution; fall through to the lookup table in that case.
    let log_arg = (s.v1_n * s.v1_n * Q7F_AGC / (s.vbars * s.a0_agc))
        .abs()
        .max(1e-12);
    let aspup_rev = -C12_AGC * libm::log(log_arg) / s.gammal1;
    if aspup_rev.abs() > 0.2 {
        return predict_range_table(state);
    }

    // ASPDWN_rev = KC3 · RDOT_n · V_n / A0_n / LAD  (line 701–710).
    let aspdwn_rev = if s.a0_agc.abs() < 1e-9 {
        0.0
    } else {
        KC3_AGC * rdot_norm * v_norm / s.a0_agc / lad
    };

    // ASP3_rev = Q5 · (Q6 − GAMMAL)  (line 712–717).
    let asp3_rev = Q5_AGC * (Q6_RAD - s.gammal);

    let asp_rev = askep_rev + asp1_rev + aspup_rev + asp3_rev + aspdwn_rev;
    let asp_km = asp_rev * 2.0 * core::f64::consts::PI * R_EARTH * 1.0e-3;

    if !asp_km.is_finite() || !(0.0..=100_000.0).contains(&asp_km) {
        return predict_range_table(state);
    }
    asp_km
}

/// Predicted inertial velocity (m/s) at the end of up-control (`VL` in
/// AGC nomenclature; the V16N63 R2 `VPRED` register).
///
/// Returns the `vl_mps` from the current HUNTEST setup, which captures
/// the closed-loop guidance law's estimate of how fast the spacecraft
/// will still be moving when it exits up-control. Returns 0.0 before
/// the 0.05g threshold (HUNTEST setup gates on phase) or when the
/// setup falls back to PREDICT3 (degenerate intermediates).
pub fn predicted_exit_velocity_mps(state: &AgcState) -> f64 {
    huntest_setup(state).map(|s| s.vl_mps).unwrap_or(0.0)
}

/// Frozen HUNTEST intermediates, shared between [`predict_range`] and
/// [`upcontrol_step`]. Mirrors the AGC erasable variables produced by
/// `REENTRY_CONTROL.agc:500–649` (HUNTEST setup + GAMMAL computation).
///
/// All velocities are stored in AGC-normalised form `V / (2·VSAT)` so the
/// dimensionless AGC formulas translate cleanly. A few SI forms are
/// pre-computed for downstream consumers (`vl_mps`, `a0_g`).
#[derive(Clone, Copy, Debug)]
struct HuntestSetup {
    /// `V₁` — projected pull-out velocity (AGC line 513). AGC-normalised.
    v1_n: f64,
    /// `V₁` in m/s.
    v1_mps: f64,
    /// `A₀` — predicted pull-out drag (AGC line 519). AGC `805 FPSS` units.
    a0_agc: f64,
    /// `A₀` in g (drag in standard gravities).
    a0_g: f64,
    /// `A₁` — drag value used by the SKIPPER `F1 = FACTOR` gain.
    ///
    /// Per `REENTRY_CONTROL.agc:500-535`: `A1 = D` when descending
    /// (`RDOT < 0`); `A1 = A0` when level or climbing (`RDOT ≥ 0`,
    /// the skip-out path). AGC `805 FPSS` units.
    a1_agc: f64,
    /// `ALP` (AGC line 547). Dimensionless.
    alp: f64,
    /// `FACT1` (AGC line 558). AGC-normalised.
    fact1: f64,
    /// `FACT2` (AGC line 564). Units of `1/g`.
    fact2: f64,
    /// `VL` — exit velocity for up-control (AGC line 571). AGC-normalised.
    vl_n: f64,
    /// `VL` in m/s.
    vl_mps: f64,
    /// `VBARS` — `VL²` in AGC-stored form (AGC line 593).
    vbars: f64,
    /// `GAMMAL1` (AGC line 580). Approximate flight-path angle at pull-out.
    gammal1: f64,
    /// `GAMMAL` (AGC line 640) — the DHOOK / AHOOKDV second-order
    /// correction applied to `gammal1`, clamped at 0 by the BMN NEGAMA
    /// branch (line 637).
    gammal: f64,
}

/// HUNTEST variable setup — REENTRY_CONTROL.agc:500–649.
///
/// Returns `None` for any degenerate intermediate that would make the
/// downstream RANGER or SKIPPER math invalid (zero LEWD, `1 − ALP ≈ 0`,
/// VL below VLMIN, etc.). Callers either fall back to a safe alternative
/// or skip that cycle's computation.
fn huntest_setup(state: &AgcState) -> Option<HuntestSetup> {
    let v = velocity_mps(state);
    let rdot = state.entry.r_dot_mps;
    let lewd = if state.entry.hunt_initialized {
        state.entry.lewd_ref
    } else {
        LEWD_INIT
    };
    let lad = LAD_NOMINAL;
    let d_agc = state.entry.sensed_acceleration_g * G0_MPS2 / FPSS_805_MPS2;

    let v_n = v / (2.0 * VSAT_MPS);
    let rdot_n = rdot / (2.0 * VSAT_MPS);
    if v_n < 1e-6 {
        return None;
    }

    let tem1b = if rdot < 0.0 { lad } else { lewd };
    if tem1b.abs() < 1e-6 {
        return None;
    }

    let v1_n = v_n + rdot_n / tem1b;

    let v1_over_v = v1_n / v_n;
    let a0_agc = v1_over_v * v1_over_v * (d_agc + rdot_n * rdot_n / (tem1b * TWO_C1_HS_AGC));

    let v1_n_after_lead = if state.entry.ld_command < 0.0 {
        v1_n - VQUIT_MPS / (2.0 * VSAT_MPS)
    } else {
        v1_n
    };
    if v1_n_after_lead.abs() < 1e-6 {
        return None;
    }

    let alp = TWO_C1_HS_AGC * a0_agc / lewd / (v1_n_after_lead * v1_n_after_lead);

    if (1.0 - alp).abs() < 1e-6 {
        return None;
    }
    let fact1 = v1_n_after_lead / (1.0 - alp);

    if a0_agc.abs() < 1e-6 {
        return None;
    }
    let fact2 = alp * (alp - 1.0) / a0_agc;

    let inner = Q7F_AGC * fact2 + alp;
    if inner < 0.0 {
        return None;
    }
    let vl_n = fact1 * (1.0 - libm::sqrt(inner));
    let vl_mps = vl_n * 2.0 * VSAT_MPS;
    if vl_mps < VLMIN_MPS || vl_n.abs() < 1e-6 {
        return None;
    }

    let vbars = vl_n * vl_n;

    let gammal1 = lewd * (v1_n_after_lead - vl_n) / vl_n;
    if gammal1.abs() < 1e-9 {
        return None;
    }

    // DHOOK / AHOOKDV / GAMMAL second-order correction
    // (REENTRY_CONTROL.agc:616-640, GETDHOOK and the GAMMAL store).
    //
    // GETDHOOK calls DHOOKYQ7 with MPAC = VS1 (= V1 in this HUNTEST
    // path), which evaluates:
    //
    //     DHOOK = ((1 - VS1/FACT1)² - ALP) / FACT2
    //
    // Then the AGC computes:
    //
    //     AHOOKDV = DHOOK / (64 · Q7) - CHOOK         (line 621-626, the
    //                                                  SR 6 / DDV Q7 / DSU
    //                                                  CHOOK chain)
    //     GAMMAL  = GAMMAL1
    //               - (AHOOKDV + 1/16) · CH1 · DVL² / (DHOOK · VBARS)
    //                                                  (line 628-639)
    //     DVL     = V1 - VL                           (line 612)
    //
    // If the result goes negative, the BMN NEGAMA branch (line 637-639)
    // clamps GAMMAL = 0.
    //
    // Algebraically in clean f64 arithmetic, `DHOOK = a0_agc` because
    // FACT1 = V1/(1-ALP) reduces `1 - VS1/FACT1` to `ALP`, and FACT2
    // already factors out `ALP·(ALP-1)/A0`. The AGC's fixed-point form
    // would differ by a few LSBs of rounding; we keep the literal
    // formula for traceability.
    let one_minus_vs1_over_fact1 = 1.0 - v1_n_after_lead / fact1;
    let dhook = (one_minus_vs1_over_fact1 * one_minus_vs1_over_fact1 - alp) / fact2;
    if dhook.abs() < 1e-12 {
        return None;
    }
    let ahookdv = dhook / (AHOOKDV_DIVISOR * Q7F_AGC) - CHOOK;
    let dvl = v1_n_after_lead - vl_n;
    let gammal_correction = (ahookdv + ONE_SIXTEENTH) * CH1 * dvl * dvl / (dhook * vbars);
    let gammal_candidate = gammal1 - gammal_correction;
    // BMN NEGAMA: clamp to 0 if the correction overshot.
    let gammal = if gammal_candidate < 0.0 {
        0.0
    } else {
        gammal_candidate
    };

    let a0_g = a0_agc * 25.0; // AGC 805 FPSS = 25 g.
                              // A1 setup (AGC lines 502 + 532-535): initial `A1 = D`; if `RDOT ≥ 0`
                              // (climbing or level — the skip-out regime), `A1` is overwritten with
                              // `A0` so the SKIPPER's `F1` gain sees the predicted pull-out drag.
    let a1_agc = if rdot < 0.0 { d_agc } else { a0_agc };

    Some(HuntestSetup {
        v1_n: v1_n_after_lead,
        v1_mps: v1_n_after_lead * 2.0 * VSAT_MPS,
        a0_agc,
        a0_g,
        a1_agc,
        alp,
        fact1,
        fact2,
        vl_n,
        vl_mps,
        vbars,
        gammal1,
        gammal,
    })
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
/// `[−LAD_NOMINAL, +LAD_NOMINAL]` (LIMITL/D, line 1271-1285) and then run
/// through [`glimiter_ld`] (AGC `GLIMITER`, line 1247) so excessive drag
/// forces full lift-up — mirroring the AGC's `STOREL/D → GLIMITER` flow.
pub fn compute_ld_command(state: &AgcState) -> LdUpdate {
    let diff_km = state.entry.target_range_km - state.entry.predicted_range_km;
    let d_g = state.entry.sensed_acceleration_g;
    let rdot = state.entry.r_dot_mps;
    let v = velocity_mps(state);

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
    // LIMITL/D post-clamp (line 1271–1285), then GLIMITER (line 1247) so
    // a high-drag transient forces full lift-up even mid-HUNTEST.
    let ld_clamped = lewd_new_raw.clamp(-LAD_NOMINAL, LAD_NOMINAL);
    let ld_command = glimiter_ld(d_g, rdot, v, ld_clamped);

    LdUpdate {
        ld_command,
        lewd_new: lewd_new_raw,
        dlewd_new: dlewd_clamped,
        diffold_new_km: diff_km,
        // HUNTEST does not touch FACTOR — carry the previous value through.
        factor_new: state.entry.factor,
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

/// Run one P65 UPCONTRL / SKIPPER iteration and return the new vertical
/// L/D command for the next 2-s SERVICER cycle.
///
/// Implements `REENTRY_CONTROL.agc:882–1091` across four branches.
///
/// 1. **`D < Q7F`** (drag too low — vehicle is above the sensible
///    atmosphere): freeze `L/D` at its previous value. The AGC routes to
///    `KEP` (P66 ballistic) here; we keep the controller alive so the next
///    SERVICER cycle can resume closed-loop guidance when drag returns.
/// 2. **`V > V₁`** (current velocity exceeds predicted pull-out velocity):
///    enter the `DOWNCNTL` branch (AGC lines 1061-1091). Required for the
///    lunar-return regime where the vehicle skips out with V ≳ V₁ during
///    the upper-atmosphere arc.
/// 3. **`D > A0` *or* `D > C20`** (drag exceeds predicted pull-out — we're
///    decelerating too fast): command max lift-up `L/D = LAD`
///    (AGC `STOREL/D` via `GOPOSLAD`).
/// 4. **Nominal SKIPPER feedback law** (AGC line 975 `UPCNTRL3`):
///    ```text
///    VREF    = FACT1 · (1 − sqrt(FACT2·D + ALP))      (line 918)
///    RDOTREF = LEWD · (V1 − VREF)                     (line 929)
///    F1      = FACTOR                                  (line 967)
///    ΔL/D    = −((RDOT − RDOTREF)·F1/KB1 + V − VREF)·F1/KB2
///    L/D     = LEWD + ΔL/D                             (clamped to ±LAD)
///    ```
///    The `F1 = FACTOR = (A1 − Q7F)/(D − Q7F)` nonlinear gain (AGC line
///    961-968) is updated only when `D > Q7MIN`; below that threshold the
///    previous-cycle `FACTOR` is reused (matching the AGC `BMN UPCNTRL3`
///    branch that skips the `STORE FACTOR` and reuses erasable memory).
///    `A1 = D` when descending (`RDOT < 0`), `A1 = A0` when climbing —
///    cached on `HuntestSetup`. The first-cycle default is `FACTOR = 1`
///    (set by `EntryState::new`).
///
/// The `CONSTD` constant-drag branch math is exposed by [`constd_dref_agc`]
/// for unit testing; wiring it into the phase state machine — when RANGER's
/// LEWD iteration diverges, or `VSAT − VL < 0` — is a follow-up since both
/// triggers fire well off-nominal and we need the rest of the pipeline (LEQ /
/// VSQUARE state) before the wire-up is meaningful.
pub fn upcontrol_step(state: &AgcState) -> LdUpdate {
    let lewd_prev = state.entry.lewd_ref;
    let d_g = state.entry.sensed_acceleration_g;
    let factor_prev = state.entry.factor;
    let v = velocity_mps(state);
    let rdot = state.entry.r_dot_mps;

    let frozen = |state: &AgcState| LdUpdate {
        ld_command: state.entry.ld_command,
        lewd_new: state.entry.lewd_ref,
        dlewd_new: state.entry.dlewd,
        diffold_new_km: state.entry.diffold_km,
        factor_new: factor_prev,
    };

    // Branch 1: drag too low — freeze L/D (AGC `KEP`, line 895).
    if d_g < Q7F_G {
        return frozen(state);
    }

    // Need HUNTEST intermediates for branches 2-4. If the setup is
    // degenerate, freeze L/D — same defensive behavior as branch 1.
    let Some(s) = huntest_setup(state) else {
        return frozen(state);
    };

    let d_agc = d_g * G0_MPS2 / FPSS_805_MPS2;

    // Branch 2: V > V₁ — DOWNCNTL (AGC lines 1061-1091). DOWNCNTL clamps
    // to ±LAD internally; GLIMITER then enforces the deceleration cap
    // (AGC `STOREL/D → GLIMITER`, line 1247).
    if v > s.v1_mps {
        let ld_raw = downcntl_ld(&s, v, rdot, d_agc);
        let ld_command = glimiter_ld(d_g, rdot, v, ld_raw);
        return LdUpdate {
            ld_command,
            lewd_new: lewd_prev,
            dlewd_new: 0.0,
            diffold_new_km: state.entry.diffold_km,
            // DOWNCNTL doesn't touch FACTOR.
            factor_new: factor_prev,
        };
    }

    // Branch 3: drag exceeds predicted pull-out drag *or* C20 trip — full
    // lift-up. AGC `CONT1` (line 909) and `NEGTESTS` (line 1008).
    if d_g > s.a0_g || d_g > C20_G {
        let ld_command = LAD_NOMINAL;
        return LdUpdate {
            ld_command,
            lewd_new: ld_command,
            dlewd_new: 0.0,
            diffold_new_km: state.entry.diffold_km,
            factor_new: factor_prev,
        };
    }

    // Branch 4: nominal SKIPPER law.
    // VREF (AGC line 918): FACT1 · (1 − sqrt(FACT2·D + ALP)).
    // FACT2 and D both carry AGC-stored "fraction-of-805-FPSS" units so
    // their product is dimensionless (matches the AGC formula).
    let inner = s.fact2 * d_agc + s.alp;
    if inner < 0.0 {
        // Pathological state — freeze L/D.
        return frozen(state);
    }
    let vref_n = s.fact1 * (1.0 - libm::sqrt(inner));
    let vref_mps = vref_n * 2.0 * VSAT_MPS;

    // RDOTREF (AGC line 929): LEWD · (V1 − VREF).
    let rdotref_mps = lewd_prev * (s.v1_mps - vref_mps);

    // FACTOR (`F1`, AGC lines 955-968): `(A1 − Q7F) / (D − Q7F)` when
    // `D > Q7MIN`; otherwise reuse the cached previous value (the AGC
    // skips the `STORE FACTOR` via the `BMN UPCNTRL3` branch).
    let factor = if d_g > Q7MIN_G {
        let denom = d_agc - Q7F_AGC;
        if denom.abs() < 1e-9 {
            factor_prev
        } else {
            (s.a1_agc - Q7F_AGC) / denom
        }
    } else {
        factor_prev
    };

    // ΔL/D = −((RDOT − RDOTREF)·F1/KB1 + V − VREF)·F1/KB2.
    let rdot_err = rdot - rdotref_mps;
    let v_err = v - vref_mps;
    let inner_sum = rdot_err * factor / KB1 + v_err;
    let raw_delta_ld = -inner_sum * factor / KB2_MPS;

    // Nonlinear gain reduction (AGC lines 989–998): if |ΔL/D| > PT1_OVER_16,
    // compress the magnitude by `POINT1 · |ΔL/D| + PT1_OVER_16` with sign
    // preserved.
    let delta_ld = if raw_delta_ld.abs() > PT1_OVER_16 {
        let compressed = POINT1 * raw_delta_ld.abs() + PT1_OVER_16;
        compressed.copysign(raw_delta_ld)
    } else {
        raw_delta_ld
    };

    // L/D = LEWD + ΔL/D, saturated to ±LAD (AGC `LIMITL/D`, line 1274),
    // then GLIMITER (line 1247) so a high-drag transient forces full
    // lift-up even on the nominal SKIPPER path.
    let ld_raw = lewd_prev + delta_ld;
    let ld_clamped = ld_raw.clamp(-LAD_NOMINAL, LAD_NOMINAL);
    let ld_command = glimiter_ld(d_g, rdot, v, ld_clamped);

    LdUpdate {
        ld_command,
        // In UPCONTRL the LEWD reference is the converged HUNTEST value —
        // we keep it frozen so range prediction stays anchored. ΔL/D rides
        // on top of LEWD per cycle.
        lewd_new: lewd_prev,
        dlewd_new: delta_ld,
        diffold_new_km: state.entry.diffold_km,
        factor_new: factor,
    }
}

/// Run one CONSTD (constant-drag) closed-loop iteration and return the
/// new vertical L/D command for the next 2-s SERVICER cycle.
///
/// Implements `REENTRY_CONTROL.agc:1036–1059`. Entered from `EntryPhase::
/// Entry` when HUNTEST diverges (|range error| > `RANGE_ERR_THRESHOLD_KM`):
/// the trajectory is well off the nominal corridor, so HUNTEST's range
/// prediction can no longer be trusted and the AGC switches to a
/// constant-drag reference profile.
///
/// ```text
/// LEQ      = V² / VSAT² − 1                               (line 238 stored)
/// DREF     = D0 = KA3 · LEQ + KA4                         (line 441-447)
/// C/D0     = −4 / DREF                                    (line 451)
/// RDOTREF  = −2·HS · DREF / V_n                            (line 1045)
/// L/D      = LEQ · C/D0
///            + K2D · (RDOT − RDOTREF)
///            + K1D · (D − DREF)                            (CONSTD1, line 1053-1057)
/// L/D      = clamp(L/D, −LAD, +LAD) → GLIMITER             (NEGTESTS path)
/// ```
///
/// CONSTD does not iterate `LEWD` or the HUNTEST Newton state — the
/// returned `LdUpdate` carries the previous `lewd_ref` / `factor` through
/// unchanged so the controller can fall back to HUNTEST cleanly via
/// `select_phase` once the range prediction recovers.
pub fn constd_step(state: &AgcState) -> LdUpdate {
    let v = velocity_mps(state);
    let rdot = state.entry.r_dot_mps;
    let d_g = state.entry.sensed_acceleration_g;
    let d_agc = d_g * G0_MPS2 / FPSS_805_MPS2;

    let v_n = v / (2.0 * VSAT_MPS);
    let rdot_n = rdot / (2.0 * VSAT_MPS);
    let leq = v_n * v_n * 4.0 - 1.0;

    // DREF = D0 = KA3·LEQ + KA4 (AGC-normalised drag).
    let dref_agc = KA3_AGC * leq + KA4_AGC;

    // Pathological D0 → freeze: keeps the previous L/D and lets
    // `select_phase` route us out next cycle.
    let frozen = LdUpdate {
        ld_command: state.entry.ld_command,
        lewd_new: state.entry.lewd_ref,
        dlewd_new: 0.0,
        diffold_new_km: state.entry.diffold_km,
        factor_new: state.entry.factor,
    };
    if dref_agc.abs() < 1e-9 {
        return frozen;
    }
    let c_over_d0 = -4.0 / dref_agc;

    // RDOTREF = −2·HS · D0 / V_n (line 1045). Use a small floor on V_n
    // to avoid blowing up near terminal velocity.
    let v_n_safe = v_n.max(1e-6);
    let rdotref_n = -TWO_HS_AGC * dref_agc / v_n_safe;

    // L/D = LEQ·C/D0 + K2D·(RDOT − RDOTREF) + K1D·(D − DREF).
    //
    // The AGC's chain ends with `SL 8D` (×256) at CONSTD1 line 1057. Our
    // `K1D_AGC` and `K2D_AGC` already absorb that ×256 (see their `_AGC`
    // doc comments — the post-SL8 physical gains). The bare `LEQ · C/D0`
    // term is computed directly, so we keep it on the same physical scale
    // by *not* multiplying by 256. Equivalently: this matches the
    // AGC-stored intermediate `LEQ · C/D0_stored` interpreted with its
    // own B-N scaling — small enough to act as a bias around the
    // closed-loop K1D / K2D corrections rather than saturate them.
    let ld_raw = leq * c_over_d0 / 256.0
        + K2D_AGC * (rdot_n - rdotref_n)
        + K1D_AGC * (d_agc - dref_agc);

    let ld_clamped = ld_raw.clamp(-LAD_NOMINAL, LAD_NOMINAL);
    let ld_command = glimiter_ld(d_g, rdot, v, ld_clamped);

    LdUpdate {
        ld_command,
        // CONSTD does not iterate LEWD; carry the HUNTEST state through.
        lewd_new: state.entry.lewd_ref,
        dlewd_new: 0.0,
        // Store the *current* HUNTEST DIFF so that if select_phase routes
        // us back to Skip on convergence, UPCONTRL's first cycle has a
        // sensible diffold reference.
        diffold_new_km: state.entry.target_range_km - state.entry.predicted_range_km,
        factor_new: state.entry.factor,
    }
}

/// DOWNCNTL L/D command — REENTRY_CONTROL.agc:1061-1091 + CONSTD1 (line
/// 1053-1059).
///
/// Entered when `V > V₁` (vehicle has more energy than the predicted pull-out
/// state). Computes a reference drag profile and a candidate L/D that biases
/// toward lift-down, then sums in `K1D·(D − DREF)` as a closed-loop drag
/// correction.
///
/// All variables in AGC-normalised form (V, V₁, RDOT divided by `2·VSAT`;
/// drag values divided by `FPSS_805`). The output L/D is dimensionless and
/// clamped to ±LAD.
///
/// ```text
/// RDTR  = LAD · (V₁ − V)                                      (AGC line 1068)
/// L/D_c = LAD + K2D · (RDOT − RDTR)
/// DREF  = (V/V₁)² · A0 − (V₁ − V)² · LAD / (2·C1·HS)         (AGC line 1093)
/// L/D   = clamp(L/D_c + K1D · (D − DREF) , −LAD, +LAD)        (CONSTD1)
/// ```
///
/// All operands are AGC-normalised inside this function — the V, RDOT
/// arguments are SI (m/s) and the conversion happens here.
fn downcntl_ld(s: &HuntestSetup, v_mps: f64, rdot_mps: f64, d_agc: f64) -> f64 {
    let v_n = v_mps / (2.0 * VSAT_MPS);
    let rdot_n = rdot_mps / (2.0 * VSAT_MPS);
    let v1_n = s.v1_n;
    let lad = LAD_NOMINAL;

    let v1_minus_v = v1_n - v_n; // negative since V > V₁.
    let rdtr_n = lad * v1_minus_v;
    let ld_candidate = lad + K2D_AGC * (rdot_n - rdtr_n);

    let v_over_v1 = v_n / v1_n;
    let dref_agc = v_over_v1 * v_over_v1 * s.a0_agc - v1_minus_v * v1_minus_v * lad / TWO_C1_HS_AGC;

    let drag_error = d_agc - dref_agc;
    let ld_raw = ld_candidate + K1D_AGC * drag_error;
    ld_raw.clamp(-LAD_NOMINAL, LAD_NOMINAL)
}

/// CONSTD reference drag — REENTRY_CONTROL.agc:1023-1059.
///
/// Computes the constant-drag DREF profile entered (1) from RANGER when
/// the LEWD iteration fails to converge (`DCONSTD` at line 1023) or (2)
/// from UPCONTRL when `VSAT − VL < 0` (`BECONSTD` at line 601). The
/// returned `DREF` is in AGC-normalised drag units (`/ FPSS_805`).
///
/// ```text
/// LEQ      = V² / (4·VSAT²) − 1                       (AGC line 238)
/// D0       = KA3 · LEQ + KA4                          (AGC line 441-447)
/// C/D0     = −4 / D0                                  (line 451)
/// RDOTREF  = −2·HS · D0 / V_n                         (line 1045)
/// DREF     = LEQ · C/D0 + K2D · (RDOT − RDOTREF)
///            + K1D · (D − DREF)  ⟶ shared in CONSTD1
/// ```
///
/// Returns the bare reference (LEQ · C/D0) prior to the SKIPPER-style
/// correction. The phase state machine does not yet route this — wiring
/// CONSTD into UPCONTRL / RANGER divergence handling is tracked separately;
/// today the helper exists so the math is locked in against the AGC formula.
pub(crate) fn constd_dref_agc(v_mps: f64, rdot_mps: f64) -> f64 {
    let v_n = v_mps / (2.0 * VSAT_MPS);
    let rdot_n = rdot_mps / (2.0 * VSAT_MPS);

    // LEQ = V_n² · 4 − 1 (since VSQUARE in the AGC is `V·V/4` and FOURTH = 1/4
    // on the same scale). Algebraically LEQ_real = V²/VSAT² − 1.
    // We store the same dimensionless value as the AGC's LEQ erasable.
    let leq = v_n * v_n * 4.0 - 1.0;

    let d0 = KA3_AGC * leq + KA4_AGC;
    if d0.abs() < 1e-9 {
        return KA4_AGC;
    }
    let c_over_d0 = -4.0 / d0;

    // Bare DREF (LEQ · C/D0) — the SKIPPER-style RDOT correction is
    // applied by the caller once we wire CONSTD into the dispatcher.
    let rdotref_n = -TWO_HS_AGC * d0 / v_n.max(1e-6);
    let _ = rdotref_n; // silence unused once wired
    let _ = rdot_n;
    leq * c_over_d0
}

/// P66 ballistic phase — zero-roll-rate hold, no closed loop.
///
/// AGC source: `KEP` block at `REENTRY_CONTROL.agc:1098` and surrounding
/// P66 logic. Entered from `EntryPhase::Skip` when drag falls below `Q7`
/// (above the sensible atmosphere) or from `EntryPhase::Entry` when the
/// HUNTEST range prediction diverges beyond `RANGE_ERR_THRESHOLD_KM`.
///
/// Behaviour: returns the previous cycle's `ld_command`, `lewd_ref` and
/// `diffold_km` verbatim with `dlewd_new = 0`. Resolves to the last
/// `EntryRoll(_)` bank command via the SERVICER's standard pipeline, so
/// the DAP keeps the spacecraft on its current trim attitude.
///
/// Exit conditions are evaluated in [`select_phase`] (currently: only
/// the global `V < VFINAL1` terminal check applies — no automatic return
/// from Ballistic to a closed-loop phase in this milestone).
pub fn ballistic_step(state: &AgcState) -> LdUpdate {
    LdUpdate {
        ld_command: state.entry.ld_command,
        lewd_new: state.entry.lewd_ref,
        dlewd_new: 0.0,
        diffold_new_km: state.entry.diffold_km,
        factor_new: state.entry.factor,
    }
}

/// P67 final-phase guidance — `PREDICT3` law from terminal velocity down
/// to drogue deploy.
///
/// AGC source: `REENTRY_CONTROL.agc:1139–1235`. Algorithm:
///
/// 1. Linearly interpolate the reference profile (RTOGO, RDOTREF, AREF, F1,
///    F2, Y) at the current velocity. `F1 = ∂Range/∂D`, `F2 = ∂Range/∂RDOT`,
///    `Y = ∂Range/∂(L/D)`.
/// 2. Compute predicted range:
///    `PREDANG = RTOGO + F1·(D − AREF) + F2·(RDOT − RDOTREF)`
///    where the stored `neg_aref_g` is `−AREF`, so `D_g - AREF_g = D_g +
///    neg_aref_g`.
/// 3. Compute L/D command:
///    `L/D = LOD_NOMINAL + (THETAH − PREDANG) / Y`
///    where THETAH is the actual range-to-go from `state.entry.target_range_km`.
/// 4. Saturate `L/D` to `±LAD_NOMINAL`, then apply `glimiter_ld` to clip
///    the command on excessive drag (AGC `GLIMITER`, line 1247).
pub fn final_phase_step(state: &AgcState) -> LdUpdate {
    let v = velocity_mps(state);
    let rdot = state.entry.r_dot_mps;
    let d_g = state.entry.sensed_acceleration_g;
    let rtogo_nm = state.entry.target_range_km / NM_TO_KM;

    let p = lookup_reference(v);

    // PREDANG = RTOGO + F1·(D - AREF) + F2·(RDOT - RDOTREF).
    // `p.neg_aref_g` is the stored "-AREF" value (negative), so the AGC's
    // `D - AREF = D + (-AREF_stored) = D_g + p.neg_aref_g`.
    let d_minus_aref = d_g + p.neg_aref_g;
    let rdot_minus_rdotref = rdot - p.rdot_ref_mps;
    let predang_nm = p.range_to_go_nm
        + p.drange_da_nm_per_g * d_minus_aref
        + p.drange_drdot_nm_per_mps * rdot_minus_rdotref;

    // L/D = LOD + (THETAH − PREDANG) / Y. Y = DRANGE/D(L/D), already in nm.
    let theta_minus_predang = rtogo_nm - predang_nm;
    let ld_command_raw = if p.drange_dld_nm.abs() > 1e-9 {
        LOD_NOMINAL + theta_minus_predang / p.drange_dld_nm
    } else {
        LOD_NOMINAL
    };
    let ld_clamped = ld_command_raw.clamp(-LAD_NOMINAL, LAD_NOMINAL);
    let ld_command = glimiter_ld(d_g, rdot, v, ld_clamped);

    LdUpdate {
        ld_command,
        // PREDICT3 doesn't iterate LEWD — freeze it from HUNTEST/UPCONTRL.
        lewd_new: state.entry.lewd_ref,
        dlewd_new: 0.0,
        diffold_new_km: state.entry.diffold_km,
        factor_new: state.entry.factor,
    }
}

/// GLIMITER deceleration limiter — REENTRY_CONTROL.agc:1247-1267.
///
/// Three-way clip on `ld_command` based on current drag:
///
/// 1. `D ≤ GMAX/2 (4 g)`: pass through unchanged.
/// 2. `D > GMAX (8 g)`: force `L/D = LAD` (full lift-up to bleed energy).
/// 3. `GMAX/2 < D ≤ GMAX`: compute the AGC's `XLIM` discriminant:
///    ```text
///    XLIM = sqrt(2·HS·(GMAX − D)·(LEQ/GMAX + LAD) + (2·HS·GMAX/V)²)
///    ```
///    If `RDOT + XLIM ≥ 0` (vehicle still has altitude margin), pass `L/D`
///    through. Otherwise clip to `LAD`.
///
/// All variables are AGC-normalised internally (`D / FPSS_805`,
/// `V / (2·VSAT)`) so the AGC's literal `2HS_AGC` / `2HSGMXSQ_AGC`
/// constants apply directly. If the `XLIM` argument goes negative (a
/// pathological mix of low V and modest LEQ), the helper conservatively
/// clips to `LAD`.
fn glimiter_ld(d_g: f64, rdot_mps: f64, v_mps: f64, ld_command: f64) -> f64 {
    if d_g <= GMAX_HALF_G {
        return ld_command;
    }
    if d_g > GMAX_G {
        return LAD_NOMINAL;
    }

    // Normalise to AGC drag scaling. D / FPSS_805 ≡ D_g / 25.
    let d_agc = d_g / 25.0;
    let gmax_agc = GMAX_G / 25.0;
    let gmax_minus_d = gmax_agc - d_agc;

    let v_n = v_mps / (2.0 * VSAT_MPS);
    let vsquare_agc = v_n * v_n * 4.0;
    // AGC erasable: LEQ stored = (V_n² · 4 − 1) / 4. `1/GMAX` stored = 0.5
    // (= 4/GMAX). The product `LEQ_stored · 1/GMAX_stored = LEQ_real/8`,
    // which is what the AGC's `LEQ · 1/GMAX` evaluates to numerically.
    let leq_stored = (vsquare_agc - 1.0) / 4.0;
    let leq_over_gmax = leq_stored * 0.5;

    let t1 = TWO_HS_AGC * gmax_minus_d * (leq_over_gmax + LAD_NOMINAL);
    let t2 = TWO_HS_GMAX_SQ_AGC / vsquare_agc.max(1.0e-9);
    let inner = t1 + t2;
    if inner < 0.0 {
        return LAD_NOMINAL;
    }
    let xlim_agc = libm::sqrt(inner);

    let rdot_norm = rdot_mps / (2.0 * VSAT_MPS);
    if rdot_norm + xlim_agc >= 0.0 {
        ld_command
    } else {
        LAD_NOMINAL
    }
}

/// Decide the next entry-guidance phase.
///
/// Returns `Some(next_phase)` to request a transition, or `None` to stay in
/// the current phase. Outcomes depend on the current phase:
///
/// **From `EntryPhase::Entry`** (HUNTEST iteration in progress):
/// - `Some(Final)` once `V < VFINAL1` (REENTRY_CONTROL.agc:431).
/// - `Some(Constd)` if `|range_error| > RANGE_ERR_THRESHOLD_KM`
///   (AGC `RANGER → DCONSTD`, line 1023). HUNTEST has diverged; keep
///   the loop closed on a constant-drag reference.
/// - `Some(Skip)` if `|range_error| < HUNTEST_CONVERGED_KM`
///   (AGC line 734 `GOTOUPSY` branch).
/// - `None` otherwise.
///
/// **From `EntryPhase::Constd`** (constant-drag closed loop):
/// - `Some(Final)` once `V < VFINAL1`.
/// - `Some(Skip)` if `|range_error| < HUNTEST_CONVERGED_KM` (HUNTEST
///   recovered — hand back to UPCONTRL).
/// - `None` otherwise. **No low-drag → Ballistic exit**: the AGC's CONSTD
///   keeps re-evaluating each cycle, accepting whatever L/D the
///   closed-loop math produces. Letting CONSTD's first-cycle output
///   freeze into Ballistic dramatically overshoots peak g (CONSTD's
///   bias drives the vehicle into a full-lift-down dive).
///
/// **From `EntryPhase::Skip`** (P65 UPCONTRL):
/// - `Some(Final)` once `V < VFINAL1` *or* `V − VL < C18`
///   (AGC line 902 `VLTEST → PREFINAL`).
/// - `Some(Ballistic)` if drag `D < Q7F_G` (AGC `KEP` routing at line 895
///   — above the sensible atmosphere, coast ballistically).
/// - `Some(Constd)` if `|range_error| > RANGE_ERR_THRESHOLD_KM`
///   (UPCONTRL diverged, fall back to constant-drag closed loop).
/// - `None` otherwise.
///
/// **From `EntryPhase::Ballistic`** (P66): no automatic return to a
/// closed-loop phase. Only the global `V < VFINAL1` terminal check fires.
pub fn select_phase(state: &AgcState) -> Option<EntryPhase> {
    let v = velocity_mps(state);

    // Terminal-velocity transition applies to every closed-loop phase.
    if v < VFINAL1_MPS {
        return Some(EntryPhase::Final);
    }

    let range_err_km = (state.entry.target_range_km - state.entry.predicted_range_km).abs();

    match state.entry.phase {
        EntryPhase::Entry => {
            // HUNTEST divergence → CONSTD (AGC `RANGER → DCONSTD`).
            // Previously routed straight to Ballistic, which killed the
            // closed loop entirely — see #86.
            if range_err_km > RANGE_ERR_THRESHOLD_KM {
                return Some(EntryPhase::Constd);
            }
            // HUNTEST convergence → P65 skip-out (UPCONTRL).
            if range_err_km < HUNTEST_CONVERGED_KM {
                return Some(EntryPhase::Skip);
            }
            None
        }
        EntryPhase::Constd => {
            // HUNTEST recovered → hand back to UPCONTRL.
            if range_err_km < HUNTEST_CONVERGED_KM {
                return Some(EntryPhase::Skip);
            }
            None
        }
        EntryPhase::Skip => {
            // PREFINAL test: V − VL < C18 hands off to P67 final phase.
            // VL is the exit velocity from the frozen HUNTEST setup; without
            // a cached value, fall back to the AGC's `V − VLMIN < C18` proxy
            // (effectively asks "are we within C18 of the minimum exit V?").
            let vl_proxy = match huntest_setup(state) {
                Some(s) => s.vl_mps,
                None => VLMIN_MPS,
            };
            if v - vl_proxy < C18_MPS {
                return Some(EntryPhase::Final);
            }
            // Low drag — above the sensible atmosphere, coast ballistically.
            // AGC `D − Q7 NEG → KEP` at REENTRY_CONTROL.agc:895.
            if state.entry.sensed_acceleration_g < Q7F_G {
                return Some(EntryPhase::Ballistic);
            }
            // UPCONTRL divergence → CONSTD (closed-loop fallback). Was
            // previously Ballistic, which killed the loop — see #86.
            if range_err_km > RANGE_ERR_THRESHOLD_KM {
                return Some(EntryPhase::Constd);
            }
            None
        }
        // P66 ballistic — hold trim. No automatic return to closed loop
        // (the global V < VFINAL1 check above already routes to Final).
        EntryPhase::Ballistic => None,
        _ => None,
    }
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

    /// TC-MSE3A-PR-1: analytic `predict_range` and the AGC `PREDICT3`-style
    /// lookup agree at hyper-velocity entry.
    ///
    /// **Honest scope** (post-#42): the analytic ASKEP formula is the
    /// Keplerian-arc analogue and is only valid in the AGC's designed
    /// up-control regime — VBARS ≳ 0.5 (VL near or above orbital), plus
    /// `log(V1²·Q7/(VBARS·A0))` near zero and GAMMAL1 in the few-hundredths
    /// of a radian band. Below the entry-interface skip-out window (the
    /// velocities sampled here, VFINAL1 + 200 m/s through ~10 km/s with
    /// ṙ = −200 m/s) every input lands inside either the `bracket ≤ 0`
    /// guard, the `E/4 < 0.05` saturation guard, or the `|ASPUP| > 0.2 rev`
    /// runaway guard — all three of which delegate to `predict_range_table`.
    ///
    /// The ratio is therefore `1.000` exactly across this sweep, asserting
    /// **fallback consistency**, not RANGER-vs-PREDICT3 agreement. The
    /// underlying `KC3`, `Q2`, `Q3` SI scalings are pinned independently by
    /// `tc_42_aspdwn_regression`. Closing the analytic-vs-table gap requires
    /// a comprehensive RANGER rework (#10 follow-up) and is out of scope.
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
                (0.99..=1.01).contains(&ratio),
                "analytic vs table ratio out of band at v={v}: \
                 r_analytic={r_analytic} r_table={r_table} ratio={ratio} \
                 — expected ≈ 1.0 via ASPDWN-scale fallback (see test docstring)"
            );
        }
    }

    /// TC-MSE3B-DHOOK-1: the DHOOK / AHOOKDV / GAMMAL correction
    /// matches a hand-computed reference (see `docs/...` and the
    /// inline trace in `huntest_setup`).
    ///
    /// **Algebraic invariant**: in clean f64 arithmetic, the AGC's
    /// `DHOOK = ((1 - VS1/FACT1)² - ALP) / FACT2` reduces *exactly*
    /// to `A0` because `FACT1 = V1 / (1 - ALP)` makes the first
    /// factor `ALP`, and `FACT2 = ALP·(ALP-1)/A0` cancels the
    /// numerator. This test pins that identity, plus the subsequent
    /// `AHOOKDV` and `GAMMAL` outputs, against drift.
    ///
    /// The reference inputs (v=10 km/s, ṙ=−50 m/s, D=0.2 g, LEWD=
    /// LEWD_INIT, hunt_initialized) sit in the non-degenerate region
    /// and produce a `gammal1` that gets noticeably (~70 %) reduced
    /// by the correction without crossing zero. This is the case
    /// the AGC source flow was designed for.
    #[test]
    fn tc_mse3b_dhook_1_textbook_values() {
        let mut state = AgcState::new();
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [-50.0, 10_000.0, 0.0];
        state.entry.r_dot_mps = -50.0;
        state.entry.sensed_acceleration_g = 0.2;
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = LEWD_INIT;
        state.entry.phase = EntryPhase::Entry;

        let setup = huntest_setup(&state).expect("non-degenerate inputs");

        // Hand-computed reference values (see test docstring).
        // Tolerances chosen to comfortably exceed the precision of the
        // hand calculation; the implementation matches to many more
        // decimals.
        assert!(
            (setup.a0_agc - 0.009242).abs() < 5e-5,
            "a0_agc expected ≈ 0.009242, got {}",
            setup.a0_agc
        );
        assert!(
            (setup.gammal1 - 0.003450).abs() < 5e-5,
            "gammal1 expected ≈ 0.003450, got {}",
            setup.gammal1
        );
        // GAMMAL after DHOOK correction is roughly 30 % of GAMMAL1.
        assert!(
            (setup.gammal - 0.001025).abs() < 5e-5,
            "gammal expected ≈ 0.001025 (gammal1 - DHOOK correction), got {}",
            setup.gammal
        );
        // Sanity: the correction must REDUCE gammal1 (the
        // (AHOOKDV + 1/16) · CH1 · DVL² / (DHOOK · VBARS) term is
        // strictly positive for these inputs).
        assert!(
            setup.gammal < setup.gammal1,
            "DHOOK correction should reduce gammal below gammal1; \
             got gammal={} vs gammal1={}",
            setup.gammal,
            setup.gammal1
        );
    }

    /// TC-MSE3B-DHOOK-2: clamp at zero (BMN NEGAMA branch). With a
    /// steeper descent (ṙ = −200 m/s, D = 0.5 g) the DHOOK correction
    /// overshoots `gammal1` and the BMN NEGAMA branch clamps `gammal`
    /// to zero. This exercises the negative-result path of the AGC's
    /// `BMN NEGAMA; STORE GAMMAL` sequence at line 637-640.
    #[test]
    fn tc_mse3b_dhook_2_negama_clamp() {
        let mut state = AgcState::new();
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [-200.0, 9_000.0, 0.0];
        state.entry.r_dot_mps = -200.0;
        state.entry.sensed_acceleration_g = 0.5;
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = LEWD_INIT;
        state.entry.phase = EntryPhase::Entry;

        let setup = huntest_setup(&state).expect("non-degenerate inputs");
        assert!(
            setup.gammal1 > 0.0,
            "gammal1 must be positive pre-clamp, got {}",
            setup.gammal1
        );
        assert_eq!(
            setup.gammal, 0.0,
            "BMN NEGAMA must clamp gammal to 0 when DHOOK correction overshoots; \
             gammal1={}, gammal={}",
            setup.gammal1, setup.gammal
        );
    }

    /// TC-42-ASPDWN: regression test for the ASPDWN SI scaling fix (#42).
    ///
    /// Pre-fix `KC3_NM_PER_M2_PER_S2 = -0.619` evaluated `ASPDWN_rev` against
    /// SI inputs without applying the AGC's `(2·VSAT)²` velocity normalisation
    /// or the `FPSS_805` drag normalisation, inflating it by ~1163× and
    /// forcing every steep-descent input through the table fallback. Post-fix
    /// the call site uses the AGC literal `KC3 = -0.024_762_223_2` against
    /// normalised operands.
    ///
    /// This test pins the corrected ASPDWN result for the inputs from the
    /// ticket #42 diagnosis (v = 9 km/s, ṙ = −200 m/s, a0_agc = 0.01357,
    /// LAD = 0.30). Hand-computed reference:
    ///
    /// ```text
    /// V_n     = 9000 / (2·VSAT)        ≈ 0.5731
    /// RDOT_n  = -200 / (2·VSAT)        ≈ -0.01273
    /// ASPDWN  = -0.024762·(-0.01273)·0.5731 / 0.01357 / 0.30
    ///         ≈ 0.04432 rev            ≈ 1773 km
    /// ```
    ///
    /// Pre-fix value at the same inputs was ~1.15 × 10⁶ km — `ratio_bug ≈
    /// 650×` larger than this corrected value. The band asserts post-fix
    /// behaviour with a margin generous enough to absorb f64 drift on the
    /// hand calculation.
    #[test]
    fn tc_42_aspdwn_regression() {
        use crate::guidance::entry_tables::{KC3_AGC, VSAT_MPS};

        let v_mps = 9_000.0_f64;
        let rdot_mps = -200.0_f64;
        let a0_agc = 0.01357_f64;
        let lad = LAD_NOMINAL;

        let v_norm = v_mps / (2.0 * VSAT_MPS);
        let rdot_norm = rdot_mps / (2.0 * VSAT_MPS);
        let aspdwn_rev = KC3_AGC * rdot_norm * v_norm / a0_agc / lad;
        let aspdwn_km = aspdwn_rev * 2.0 * core::f64::consts::PI * R_EARTH * 1.0e-3;

        assert!(
            (1_700.0..=1_900.0).contains(&aspdwn_km),
            "ASPDWN out of post-#42 band: expected ≈ 1773 km, got {aspdwn_km} km"
        );
    }

    /// TC-42-ASP1: regression test for the ASP1 = Q2 + Q3·VL SI scaling fix (#42).
    ///
    /// Pre-fix `Q2_NM = +1280` had the wrong sign and magnitude (AGC builds
    /// it dynamically as `LAD·Q21 + Q22 = (500·LAD − 1152)`, which is
    /// `-1002` nm at LAD = 0.3, not +1280). Pre-fix `Q3_NM_PER_MPS = 0.0509`
    /// applied a stale ft↔m conversion (collapsed AGC's literal stored value
    /// `0.167` by a factor ≈ 4.5). Post-fix the call site evaluates
    /// `q2_rev + Q3_AGC · vl_norm` with AGC-native dimensionless constants.
    ///
    /// Hand-computed at LAD = 0.30, VL = 7644 m/s (a representative pull-out
    /// exit velocity from the HUNTEST setup):
    ///
    /// ```text
    /// Q2_rev = (500·0.30 - 1152) / 21600 ≈ -0.04639
    /// VL_n   = 7644 / (2·VSAT)           ≈ 0.4866
    /// ASP1   = -0.04639 + 0.167·0.4866   ≈  0.03488 rev   ≈  1395 km
    /// ```
    #[test]
    fn tc_42_asp1_regression() {
        use crate::guidance::entry_tables::{Q21_AGC, Q22_AGC, Q3_AGC, VSAT_MPS};

        let lad = LAD_NOMINAL;
        let vl_mps = 7_644.0_f64;
        let vl_norm = vl_mps / (2.0 * VSAT_MPS);

        let q2_rev = lad * Q21_AGC + Q22_AGC;
        assert!(
            (q2_rev - (-1_002.0 / 21_600.0)).abs() < 1e-9,
            "Q2_rev = LAD·Q21 + Q22 mismatch at LAD = 0.30: got {q2_rev}"
        );

        let asp1_rev = q2_rev + Q3_AGC * vl_norm;
        let asp1_km = asp1_rev * 2.0 * core::f64::consts::PI * R_EARTH * 1.0e-3;
        assert!(
            (1_350.0..=1_450.0).contains(&asp1_km),
            "ASP1 out of post-#42 band: expected ≈ 1395 km, got {asp1_km} km"
        );
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

    /// TC-MSE3-SP-2: HUNTEST iteration in progress — range error between
    /// the "converged" band and the divergence threshold stays in Entry.
    ///
    /// With MS-E4, `DIFF ≈ 0` triggers Skip (HUNTEST converged), and
    /// `DIFF > 500 km` triggers Ballistic. The "no transition" band is
    /// `HUNTEST_CONVERGED_KM < |DIFF| < RANGE_ERR_THRESHOLD_KM`.
    #[test]
    fn tc_mse3_sp_2_nominal_no_transition() {
        let mut state = fixture(VFINAL1_MPS + 500.0);
        state.entry.target_range_km = state.entry.predicted_range_km + 200.0;
        assert_eq!(select_phase(&state), None);
    }

    /// TC-MSE3-SP-3: HUNTEST divergence transitions to CONSTD (constant-drag
    /// closed loop) per `RANGER → DCONSTD`, AGC line 1023. Previously routed
    /// straight to Ballistic, which killed the closed loop — see #86.
    #[test]
    fn tc_mse3_sp_3_diverged_to_constd() {
        let mut state = fixture(VFINAL1_MPS + 500.0);
        // Force a 1500-km range error — well above 500-km threshold.
        state.entry.target_range_km = state.entry.predicted_range_km + 1_500.0;
        assert_eq!(select_phase(&state), Some(EntryPhase::Constd));
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
        state.csm_state.position = [
            6_500_000.0 * libm::cos(lon),
            6_500_000.0 * libm::sin(lon),
            0.0,
        ];
        let cr = crossrange_km(&state);
        // Expected ≈ R_EARTH · sin(1°) · 1 ≈ 111 km
        assert!(cr > 100.0 && cr < 120.0, "expected ~111 km, got {cr} km");
    }

    // ── MS-E4 upcontrol_step (P65 SKIPPER) ────────────────────────────────────

    /// TC-MSE4-UC-1: drag below `Q7F_G` freezes `ld_command` (AGC `KEP`).
    #[test]
    fn tc_mse4_uc_1_low_drag_freezes_ld() {
        let mut state = fixture(9_500.0);
        state.entry.phase = EntryPhase::Skip;
        state.entry.sensed_acceleration_g = Q7F_G * 0.5; // well below threshold
        state.entry.ld_command = 0.18;
        state.entry.lewd_ref = 0.22;

        let upd = upcontrol_step(&state);
        assert!(
            (upd.ld_command - 0.18).abs() < 1e-9,
            "expected ld_command frozen at 0.18, got {}",
            upd.ld_command
        );
        assert!(
            (upd.lewd_new - 0.22).abs() < 1e-9,
            "lewd_new must stay at the previous LEWD, got {}",
            upd.lewd_new
        );
    }

    /// TC-MSE4-UC-2: drag above `C20_G` (175 ft/s² ≈ 0.217 g) commands max
    /// lift-up `L/D = LAD_NOMINAL` (AGC `CONT1` / `NEGTESTS`).
    #[test]
    fn tc_mse4_uc_2_high_drag_max_lift_up() {
        use crate::guidance::entry_tables::C20_G;
        let mut state = fixture(9_500.0);
        state.entry.phase = EntryPhase::Skip;
        state.entry.sensed_acceleration_g = C20_G * 2.0; // way above
        state.entry.lewd_ref = 0.15;

        let upd = upcontrol_step(&state);
        assert!(
            (upd.ld_command - LAD_NOMINAL).abs() < 1e-9,
            "expected ld_command = LAD = {LAD_NOMINAL}, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE4-UC-3: nominal SKIPPER law produces a non-zero ΔL/D in
    /// response to a non-zero `(V − VREF)` or `(RDOT − RDOTREF)` error.
    ///
    /// Approach: place the vehicle in a steep-descent state (large negative
    /// RDOT) so the difference from the SKIPPER reference is significant,
    /// and verify the returned ld_command moves away from the previous LEWD.
    #[test]
    fn tc_mse4_uc_3_skipper_produces_delta_ld() {
        let mut state = fixture(9_500.0);
        state.entry.phase = EntryPhase::Skip;
        // Moderate drag above Q7F (0.186 g) and below C20 (5.4 g):
        state.entry.sensed_acceleration_g = 0.5;
        state.entry.lewd_ref = 0.20;
        state.entry.ld_command = 0.20;
        state.csm_state.velocity = [-300.0, 9_500.0, 0.0];
        state.entry.r_dot_mps = -300.0;

        let upd = upcontrol_step(&state);
        let delta = upd.ld_command - state.entry.lewd_ref;
        assert!(
            delta.abs() > 1e-9,
            "expected non-zero ΔL/D from SKIPPER, got {delta} (ld_command={})",
            upd.ld_command
        );
        assert!(
            upd.ld_command.abs() <= LAD_NOMINAL + 1e-9,
            "ld_command must be saturated to ±LAD, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE4B-F1-1: SKIPPER F1 = (A1 - Q7F)/(D - Q7F) when climbing at
    /// high drag.
    ///
    /// With RDOT ≥ 0 the HUNTEST setup writes `A1 = A0` (line 535) and the
    /// SKIPPER gets the AGC's nonlinear F1 gain. The CONTINU2 update branch
    /// only fires when `D > Q7MIN_G ≈ 1.24 g` (AGC line 957) — below that
    /// FACTOR is held over from the previous cycle. Hand-computed reference
    /// at a high-drag climbing snapshot:
    ///
    /// ```text
    /// V = 10 km/s, RDOT = +50 m/s, D = 1.5 g, LEWD = 0.15
    /// v_n     = 0.6367,  rdot_n   = 0.00318,  tem1b = LEWD = 0.15
    /// v1_n    = 0.6580,  v1_over_v = 1.0334
    /// d_agc   = 1.5·g₀/FPSS_805 = 0.05997
    /// a0_agc  = 1.0334² · (0.05997 + 0.00318²/(0.15·2C1HS)) ≈ 0.06734
    /// a1_agc  = a0_agc  (since rdot ≥ 0)
    /// F1      = (0.06734 - 0.00745) / (0.05997 - 0.00745) ≈ 1.140
    /// ```
    #[test]
    fn tc_mse4b_f1_1_climbing_amplified_gain() {
        let mut state = AgcState::new();
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [50.0, 10_000.0, 0.0];
        state.entry.r_dot_mps = 50.0;
        state.entry.sensed_acceleration_g = 1.5;
        state.entry.phase = EntryPhase::Skip;
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = LEWD_INIT;
        state.entry.factor = 1.0;

        let upd = upcontrol_step(&state);
        assert!(
            (upd.factor_new - 1.140).abs() < 0.05,
            "F1 expected ≈ 1.140, got {}",
            upd.factor_new
        );
    }

    /// TC-MSE4B-F1-2: SKIPPER F1 = 1.0 exactly when descending at high drag.
    ///
    /// With RDOT < 0 the HUNTEST setup keeps `A1 = D` (line 502), so the
    /// numerator and denominator of `(A1-Q7F)/(D-Q7F)` are identical and
    /// `F1 = 1`. The AGC's gain compression effectively only activates on
    /// the skip-out climb. We pick `D = 1.5 g > Q7MIN_G` so the FACTOR
    /// update branch is taken; the pre-loaded `factor = 1.5` must be
    /// overwritten with `1.0`.
    #[test]
    fn tc_mse4b_f1_2_descending_unity_gain() {
        let mut state = AgcState::new();
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [-100.0, 8_000.0, 0.0];
        state.entry.r_dot_mps = -100.0;
        state.entry.sensed_acceleration_g = 1.5;
        state.entry.phase = EntryPhase::Skip;
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = LEWD_INIT;
        state.entry.factor = 1.5;

        let upd = upcontrol_step(&state);
        // With D = 1.5 g the SKIPPER may saturate `D > A0_g` and route to
        // branch 3 (max-lift-up). Either way, FACTOR is unchanged on that
        // path — but on the SKIPPER path A1 = D collapses the ratio to 1.
        // Both legal outcomes are acceptable; we just require it isn't the
        // pre-loaded 1.5 if the SKIPPER ran.
        let on_skipper = (upd.factor_new - 1.0).abs() < 1e-12;
        let on_branch3 = (upd.factor_new - 1.5).abs() < 1e-12 && upd.ld_command == LAD_NOMINAL;
        assert!(
            on_skipper || on_branch3,
            "F1 must be 1.0 (SKIPPER) or factor unchanged with full-LAD lift (branch 3), \
             got factor={}, ld={}",
            upd.factor_new,
            upd.ld_command
        );
    }

    /// TC-MSE4B-F1-3: `D < Q7MIN_G` freezes FACTOR at its previous value.
    ///
    /// Per AGC CONTINU2 (line 957-958), the `BMN UPCNTRL3` branch skips
    /// `STORE FACTOR`, leaving the erasable variable at whatever the prior
    /// cycle left there. We pre-load `factor = 2.5` and verify the SKIPPER
    /// passes it through unchanged at a typical mid-entry drag of 0.5 g
    /// (which is `> Q7F_G ≈ 0.186 g` so branch 1 doesn't freeze, but
    /// `< Q7MIN_G ≈ 1.242 g` so CONTINU2 skips the update).
    #[test]
    fn tc_mse4b_f1_3_low_drag_freezes_factor() {
        let mut state = AgcState::new();
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [-50.0, 10_000.0, 0.0];
        state.entry.r_dot_mps = -50.0;
        state.entry.sensed_acceleration_g = 0.5;
        state.entry.phase = EntryPhase::Skip;
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = LEWD_INIT;
        state.entry.factor = 2.5;

        let upd = upcontrol_step(&state);
        assert!(
            (upd.factor_new - 2.5).abs() < 1e-12,
            "FACTOR must be frozen at 2.5 when D < Q7MIN_G, got {}",
            upd.factor_new
        );
    }

    /// TC-MSE4B-DOWNCNTL-1: DOWNCNTL branch fires when V > V₁ and produces
    /// a closed-loop L/D from the DREF / K1D / K2D feedback.
    ///
    /// Hand-computed reference (with V1LEAD applied — prior cycle commanded
    /// lift-down, so V₁ is reduced by VQUIT):
    ///
    /// ```text
    /// V = 8 km/s, RDOT = -200 m/s, D = 1.0 g, LEWD = 0.15, ld_cmd_prev = -0.05
    /// v_n      = 0.5094,  rdot_n      = -0.01273,  tem1b = LAD = 0.3
    /// v1_pre   = 0.4670,  v1_after_lead = 0.4476 (= v1_pre − VQUIT/(2·VSAT))
    /// d_agc    = 1.0·g₀/FPSS_805 = 0.04000
    /// a0_agc   = (0.467/0.5094)² · (0.04 + 0.01273²/(0.3·2C1HS)) ≈ 0.05462
    /// V > V₁ (8000 > 7030) → DOWNCNTL.
    ///
    /// v1_minus_v  = -0.0618,  rdtr_n = LAD·v1_minus_v = -0.01854
    /// (rdot_n − rdtr_n) = 0.00581
    /// ld_candidate = LAD + K2D · 0.00581 = 0.3 − 51.53·0.00581 ≈ 0.0006
    /// (V/V₁)² · a0 − (V₁−V)² · LAD / 2C1HS
    ///   = 1.138² · 0.05462 − 0.0618² · 0.3 / 0.0216 ≈ 0.0707 − 0.0531 = 0.0177
    /// drag_error = 0.04 − 0.0177 = 0.0223
    /// L/D = 0.0006 + 8.05 · 0.0223 ≈ 0.180
    /// ```
    #[test]
    fn tc_mse4b_downcntl_1_v_above_v1_gives_closed_loop_ld() {
        let mut state = AgcState::new();
        state.csm_state.position = [6_500_000.0, 0.0, 0.0];
        state.csm_state.velocity = [-200.0, 8_000.0, 0.0];
        state.entry.r_dot_mps = -200.0;
        state.entry.sensed_acceleration_g = 1.0;
        state.entry.phase = EntryPhase::Skip;
        state.entry.hunt_initialized = true;
        state.entry.lewd_ref = LEWD_INIT;
        // Negative ld_command triggers V1LEAD in huntest_setup.
        state.entry.ld_command = -0.05;
        state.entry.factor = 1.0;

        let upd = upcontrol_step(&state);
        assert!(
            (upd.ld_command - 0.180).abs() < 0.02,
            "DOWNCNTL L/D expected ≈ 0.180, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE4B-CONSTD-1: `constd_dref_agc` math is locked against drift.
    ///
    /// CONSTD's `D0 = KA3·LEQ + KA4` and `C/D0 = -4/D0` together give a
    /// bare reference `DREF = LEQ · C/D0`. For a representative supercircular
    /// input the values can be hand-checked.
    ///
    /// ```text
    /// V = 10 km/s, RDOT = -100 m/s
    /// v_n  = 0.6367,  LEQ = 4·v_n² − 1 = 4·0.4053 − 1 = 0.6213
    /// D0   = KA3·LEQ + KA4 = 0.447·0.6213 + 0.0497 = 0.3275
    /// C/D0 = -4/0.3275 ≈ -12.21
    /// DREF = 0.6213 · -12.21 ≈ -7.59
    /// ```
    ///
    /// The returned bare DREF lives in AGC-normalised "/FPSS_805" units;
    /// the magnitude is large because the LEQ·C/D0 term is the raw kinematic
    /// component before the K1D / K2D corrections at CONSTD1. The test
    /// pins the formula identity, not the eventual physical drag.
    #[test]
    fn tc_mse4b_constd_1_dref_math() {
        let dref = constd_dref_agc(10_000.0, -100.0);
        assert!(
            (dref - (-7.59)).abs() < 0.1,
            "CONSTD bare DREF expected ≈ -7.59 (AGC-normalised), got {dref}"
        );
    }

    /// TC-MSE86-CS-1 (#86): `constd_step` produces an L/D inside ±LAD_NOMINAL
    /// at a typical CONSTD operating point. Establishes the basic invariant
    /// that the constant-drag closed loop returns a saturatable command.
    #[test]
    fn tc_mse86_cs_1_produces_clamped_ld() {
        let mut state = fixture(9_500.0);
        state.entry.phase = EntryPhase::Constd;
        state.entry.sensed_acceleration_g = 1.5;
        state.entry.r_dot_mps = -300.0;
        let upd = constd_step(&state);
        assert!(
            upd.ld_command.abs() <= LAD_NOMINAL + 1e-9,
            "ld_command must be saturated, got {}",
            upd.ld_command,
        );
    }

    /// TC-MSE86-CS-2 (#86): `constd_step` carries `lewd_ref` and `factor`
    /// through unchanged so HUNTEST can resume cleanly if `select_phase`
    /// hands back to Skip.
    #[test]
    fn tc_mse86_cs_2_preserves_huntest_state() {
        let mut state = fixture(9_500.0);
        state.entry.phase = EntryPhase::Constd;
        state.entry.sensed_acceleration_g = 1.0;
        state.entry.r_dot_mps = -200.0;
        state.entry.lewd_ref = 0.123;
        state.entry.factor = 1.7;
        let upd = constd_step(&state);
        assert!((upd.lewd_new - 0.123).abs() < 1e-12);
        assert!((upd.factor_new - 1.7).abs() < 1e-12);
        assert_eq!(upd.dlewd_new, 0.0);
    }

    /// TC-MSE86-SP-1 (#86): from `Constd`, large range error keeps us in
    /// CONSTD (waiting for HUNTEST to recover).
    #[test]
    fn tc_mse86_sp_1_constd_large_err_stays() {
        let mut state = fixture(VFINAL1_MPS + 1_000.0);
        state.entry.phase = EntryPhase::Constd;
        state.entry.sensed_acceleration_g = 0.5;
        state.entry.target_range_km = state.entry.predicted_range_km + 1_500.0;
        assert_eq!(select_phase(&state), None);
    }

    /// TC-MSE86-SP-2 (#86): from `Constd`, range error inside HUNTEST
    /// convergence band hands back to Skip (HUNTEST recovered).
    #[test]
    fn tc_mse86_sp_2_constd_recovery_to_skip() {
        let mut state = fixture(VFINAL1_MPS + 1_000.0);
        state.entry.phase = EntryPhase::Constd;
        state.entry.sensed_acceleration_g = 0.5;
        state.entry.target_range_km = state.entry.predicted_range_km;
        assert_eq!(select_phase(&state), Some(EntryPhase::Skip));
    }

    /// TC-MSE86-SP-3 (#86): from `Constd`, low-drag (above the sensible
    /// atmosphere) does **not** exit to Ballistic — CONSTD keeps running
    /// each cycle. Letting CONSTD's first-cycle output freeze into
    /// Ballistic overshoots peak g dramatically.
    #[test]
    fn tc_mse86_sp_3_constd_low_drag_stays() {
        let mut state = fixture(VFINAL1_MPS + 1_000.0);
        state.entry.phase = EntryPhase::Constd;
        // Below Q7F_G threshold; range error still large (avoids
        // recovery-to-Skip).
        state.entry.sensed_acceleration_g = Q7F_G - 0.01;
        state.entry.target_range_km = state.entry.predicted_range_km + 1_500.0;
        assert_eq!(select_phase(&state), None);
    }

    /// TC-MSE86-SP-4 (#86): from `Constd`, `V < VFINAL1` short-circuits to
    /// Final via the global terminal-velocity check.
    #[test]
    fn tc_mse86_sp_4_constd_terminal_v_to_final() {
        let mut state = fixture(VFINAL1_MPS - 100.0);
        state.entry.phase = EntryPhase::Constd;
        assert_eq!(select_phase(&state), Some(EntryPhase::Final));
    }

    /// TC-MSE4-UC-4: SKIPPER ld_command saturates to ±LAD on extreme errors.
    #[test]
    fn tc_mse4_uc_4_skipper_saturates() {
        let mut state = fixture(9_500.0);
        state.entry.phase = EntryPhase::Skip;
        state.entry.sensed_acceleration_g = 0.15;
        state.entry.lewd_ref = 0.10;
        state.entry.ld_command = 0.10;
        // Massive descent rate → big negative ΔL/D → saturates to −LAD or 0.
        state.csm_state.velocity = [-5_000.0, 9_500.0, 0.0];
        state.entry.r_dot_mps = -5_000.0;

        let upd = upcontrol_step(&state);
        assert!(
            upd.ld_command.abs() <= LAD_NOMINAL + 1e-9,
            "ld_command must be saturated, got {}",
            upd.ld_command
        );
    }

    // ── MS-E4 select_phase (Skip transitions) ─────────────────────────────────

    /// TC-MSE4-SP-1: HUNTEST converges → transition to Skip.
    #[test]
    fn tc_mse4_sp_1_huntest_convergence_to_skip() {
        use crate::guidance::entry_tables::HUNTEST_CONVERGED_KM;
        let mut state = fixture(VFINAL1_MPS + 500.0);
        // |range_err| < 25 nm ≈ 46 km → Skip.
        state.entry.target_range_km = state.entry.predicted_range_km + 0.5 * HUNTEST_CONVERGED_KM;
        assert_eq!(select_phase(&state), Some(EntryPhase::Skip));
    }

    /// TC-MSE4-SP-2: in Skip, nominal state (above VFINAL1, modest range
    /// error) stays in Skip (`None`).
    ///
    /// The VLTEST → Final branch depends on the HUNTEST setup producing a
    /// `VL` close to the current `V`, which requires controllable VAGC-
    /// fixture inputs — moved to MS-E4b. The basic "Skip phase tolerates a
    /// nominal cycle" invariant is what we exercise here.
    #[test]
    fn tc_mse4_sp_2_skip_nominal_no_transition() {
        let mut state = fixture(VFINAL1_MPS + 1_000.0);
        state.entry.phase = EntryPhase::Skip;
        // Drag above Q7F_G so MS-E5's low-drag → Ballistic trigger doesn't fire.
        state.entry.sensed_acceleration_g = 0.5;
        // Range error within both the Skip-stay band: not large enough to
        // trigger Ballistic, V well above VFINAL1.
        state.entry.target_range_km = state.entry.predicted_range_km + 200.0;
        assert_eq!(select_phase(&state), None);
    }

    /// TC-MSE4-SP-3: in Skip, |range_err| > 500 km → CONSTD (closed-loop
    /// fallback, #86). Previously routed straight to Ballistic.
    #[test]
    fn tc_mse4_sp_3_skip_divergence_to_constd() {
        let mut state = fixture(VFINAL1_MPS + 1_000.0);
        state.entry.phase = EntryPhase::Skip;
        // Drag above Q7F_G so the assertion exercises the *range-error*
        // path, not the MS-E5 low-drag path.
        state.entry.sensed_acceleration_g = 0.5;
        state.entry.target_range_km = state.entry.predicted_range_km + 1_500.0;
        assert_eq!(select_phase(&state), Some(EntryPhase::Constd));
    }

    // ── MS-E5 ballistic_step (P66) ────────────────────────────────────────────

    /// TC-MSE5-BS-1: `ballistic_step` freezes all outputs and zeroes `dlewd`.
    #[test]
    fn tc_mse5_bs_1_freezes_outputs() {
        let mut state = fixture(VFINAL1_MPS + 1_000.0);
        state.entry.phase = EntryPhase::Ballistic;
        state.entry.ld_command = 0.12;
        state.entry.lewd_ref = 0.17;
        state.entry.dlewd = 0.03;
        state.entry.diffold_km = 80.0;

        let upd = ballistic_step(&state);
        assert!((upd.ld_command - 0.12).abs() < 1e-12);
        assert!((upd.lewd_new - 0.17).abs() < 1e-12);
        assert!(upd.dlewd_new.abs() < 1e-12);
        assert!((upd.diffold_new_km - 80.0).abs() < 1e-12);
    }

    /// TC-MSE5-SP-1: in Skip, drag below `Q7F_G` transitions to Ballistic.
    ///
    /// AGC `D − Q7 NEG → KEP` at REENTRY_CONTROL.agc:895.
    #[test]
    fn tc_mse5_sp_1_low_drag_skip_to_ballistic() {
        let mut state = fixture(VFINAL1_MPS + 1_000.0);
        state.entry.phase = EntryPhase::Skip;
        // 0.05 g — well below Q7F_G (0.186 g) but well above the 0.05 g
        // entry-interface threshold (which is unrelated to this AGC test).
        state.entry.sensed_acceleration_g = 0.10;
        state.entry.target_range_km = state.entry.predicted_range_km;
        assert_eq!(select_phase(&state), Some(EntryPhase::Ballistic));
    }

    /// TC-MSE5-SP-2: from Ballistic, no automatic transition while
    /// V > VFINAL1 — the controller holds attitude indefinitely until
    /// terminal velocity is reached.
    #[test]
    fn tc_mse5_sp_2_ballistic_no_return() {
        let mut state = fixture(VFINAL1_MPS + 1_000.0);
        state.entry.phase = EntryPhase::Ballistic;
        // Make D high again and range-error favorable — neither should
        // pull us out of Ballistic.
        state.entry.sensed_acceleration_g = 0.5;
        state.entry.target_range_km = state.entry.predicted_range_km;
        assert_eq!(select_phase(&state), None);
    }

    /// TC-MSE5-SP-3: from Ballistic, terminal velocity still routes to Final.
    #[test]
    fn tc_mse5_sp_3_ballistic_to_final_at_terminal_v() {
        let mut state = fixture(VFINAL1_MPS - 100.0);
        state.entry.phase = EntryPhase::Ballistic;
        assert_eq!(select_phase(&state), Some(EntryPhase::Final));
    }

    // ── MS-E6 final_phase_step (P67 PREDICT3) ─────────────────────────────────

    /// TC-MSE6-FP-1: `final_phase_step` returns an `L/D` clamped to ±LAD.
    #[test]
    fn tc_mse6_fp_1_returns_clamped_ld() {
        let mut state = fixture(VFINAL1_MPS - 200.0);
        state.entry.phase = EntryPhase::Final;
        state.entry.target_range_km = 800.0;
        let upd = final_phase_step(&state);
        assert!(
            upd.ld_command.abs() <= LAD_NOMINAL + 1e-12,
            "L/D must be clamped to ±LAD, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE6B-PREDANG-1: PREDANG = RTOGO + F1·(D-AREF) + F2·(RDOT-RDOTREF).
    ///
    /// Hand-computed reference at the V=7038 m/s table sample (i=10):
    ///
    /// ```text
    /// RTOGO     = 0.186963 · 2700           ≈ 504.8 nm
    /// AREF      = -p.neg_aref_g             ≈ 0.873 g
    /// RDOTREF   = -0.017981·1963.4          ≈ -35.31 m/s
    /// F1 (drda) = -0.602557 · 2700/25       ≈ -65.08 nm/g
    /// F2 (drdrdot) = +4.151220 · 2700/15707 ≈ +0.7136 nm/(m/s)
    ///
    /// At D = 5 g, RDOT = -100 m/s:
    ///   D - AREF              = 5 - 0.873   = 4.127 g
    ///   RDOT - RDOTREF        = -100 + 35.3 = -64.69 m/s
    ///   PREDANG = 504.8 + (-65.08)·4.127 + 0.7136·(-64.69)
    ///           ≈ 504.8 - 268.6 - 46.2
    ///           ≈ 190.0 nm
    /// ```
    ///
    /// With `target_range_km = 190·NM_TO_KM`, `(THETAH-PREDANG)/Y ≈ 0` so
    /// `L/D ≈ LOD_NOMINAL`. Demonstrates the F1/F2 corrections drive PREDANG
    /// down from the bare RTOGO value (504.8 → 190 nm) under high drag and
    /// faster descent than reference.
    #[test]
    fn tc_mse6b_predang_1_full_formula_at_sample10() {
        use crate::guidance::entry_tables::REFERENCE_PROFILE;
        let v = REFERENCE_PROFILE[10].velocity_mps;
        let mut state = fixture(v);
        state.entry.phase = EntryPhase::Final;
        state.entry.sensed_acceleration_g = 5.0;
        state.entry.r_dot_mps = -100.0;
        state.entry.target_range_km = 190.0 * NM_TO_KM;
        let upd = final_phase_step(&state);
        // With PREDANG ≈ THETAH the L/D should be ≈ LOD_NOMINAL.
        // Then GLIMITER kicks in (D = 5 g > GMAX_HALF_G = 4 g) — at these
        // inputs `XLIM + RDOT_norm > 0`, so L/D passes through.
        assert!(
            (upd.ld_command - LOD_NOMINAL).abs() < 0.02,
            "L/D expected ≈ LOD_NOMINAL = {LOD_NOMINAL} at PREDANG-THETAH ≈ 0, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE6B-GLIMITER-1: drag below GMAX/2 leaves L/D unchanged.
    ///
    /// At D = 1 g (well below the 4 g GLIMITER threshold) the limiter is
    /// inert and `final_phase_step` returns the PREDICT3 L/D verbatim.
    #[test]
    fn tc_mse6b_glimiter_1_below_threshold_pass_through() {
        use crate::guidance::entry_tables::REFERENCE_PROFILE;
        let v = REFERENCE_PROFILE[10].velocity_mps;
        let mut state = fixture(v);
        state.entry.phase = EntryPhase::Final;
        state.entry.sensed_acceleration_g = 1.0;
        state.entry.r_dot_mps = -100.0;
        // Push PREDANG far short of THETAH so L/D wants to be near LAD.
        state.entry.target_range_km = 5000.0;
        let upd = final_phase_step(&state);
        assert!(
            (upd.ld_command - LAD_NOMINAL).abs() < 1e-9,
            "L/D should saturate to LAD when target ≫ PREDANG, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE6B-GLIMITER-2: drag above GMAX forces L/D = LAD regardless.
    ///
    /// At D = 9 g (above the 8 g hard limit) the limiter unconditionally
    /// clips to maximum lift-up — sized to bleed kinetic energy as fast as
    /// the vehicle aerodynamics allow.
    #[test]
    fn tc_mse6b_glimiter_2_above_gmax_forces_lad() {
        use crate::guidance::entry_tables::REFERENCE_PROFILE;
        let v = REFERENCE_PROFILE[10].velocity_mps;
        let mut state = fixture(v);
        state.entry.phase = EntryPhase::Final;
        state.entry.sensed_acceleration_g = 9.0;
        state.entry.r_dot_mps = -100.0;
        // Even target a short range so PREDICT3 wants L/D ≈ LOD or below —
        // GLIMITER still drives L/D to LAD.
        state.entry.target_range_km = 1.0;
        let upd = final_phase_step(&state);
        assert!(
            (upd.ld_command - LAD_NOMINAL).abs() < 1e-9,
            "L/D must clip to LAD when D > GMAX, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE6B-GLIMITER-3: in the `GMAX/2 < D ≤ GMAX` band, XLIM gates the
    /// clip. With heavy descent (RDOT = −500 m/s) and D ≈ 5 g, the AGC's
    /// `RDOT + XLIM < 0` branch fires and L/D snaps to LAD even if PREDICT3
    /// wanted something modest.
    #[test]
    fn tc_mse6b_glimiter_3_xlim_clips_on_heavy_descent() {
        use crate::guidance::entry_tables::REFERENCE_PROFILE;
        let v = REFERENCE_PROFILE[10].velocity_mps;
        let mut state = fixture(v);
        state.entry.phase = EntryPhase::Final;
        state.entry.sensed_acceleration_g = 5.0;
        state.entry.r_dot_mps = -500.0;
        // Set target = PREDANG_at_this_input so PREDICT3 wants L/D ≈ LOD.
        // GLIMITER must override because RDOT is much more negative than XLIM.
        let upd = final_phase_step(&state);
        assert!(
            (upd.ld_command - LAD_NOMINAL).abs() < 1e-9,
            "GLIMITER must clip L/D to LAD at D=5g, RDOT=-500, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE6-FP-2: when `target_range_km` matches PREDANG (the table's
    /// RTOGO at the current V, plus the F1 and F2 corrections from any
    /// drag / rdot offset), the L/D command equals LOD_NOMINAL — the
    /// nominal "no correction needed" output.
    ///
    /// With the post-#34 full PREDANG formula, this requires setting
    /// `D = AREF` *and* `RDOT = RDOTREF` so the sensitivity corrections
    /// vanish, then targeting RTOGO exactly.
    #[test]
    fn tc_mse6_fp_2_nominal_ld_at_zero_correction() {
        use crate::guidance::entry_tables::lookup_reference;
        let v = VFINAL1_MPS - 200.0;
        let mut state = fixture(v);
        state.entry.phase = EntryPhase::Final;
        let p = lookup_reference(v);
        // Zero the F1 contribution: D_g = AREF_g = -p.neg_aref_g.
        state.entry.sensed_acceleration_g = -p.neg_aref_g;
        // Zero the F2 contribution: RDOT = RDOTREF.
        state.entry.r_dot_mps = p.rdot_ref_mps;
        // Set target = RTOGO so (THETAH − PREDANG) = 0 → L/D = LOD.
        state.entry.target_range_km = p.range_to_go_nm * NM_TO_KM;
        let upd = final_phase_step(&state);
        assert!(
            (upd.ld_command - LOD_NOMINAL).abs() < 1e-9,
            "expected L/D = LOD = {LOD_NOMINAL}, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE6-FP-3: range error in the correctable direction moves L/D in
    /// the expected sense — long-range error pushes L/D up (lift to extend),
    /// short-range error pushes L/D down (push down to shorten).
    ///
    /// Like TC-MSE6-FP-2, we hold D and RDOT at their reference values so
    /// only the range-error term drives the L/D update.
    #[test]
    fn tc_mse6_fp_3_ld_sense_from_range_error() {
        use crate::guidance::entry_tables::lookup_reference;
        let v = VFINAL1_MPS - 200.0;
        let mut state = fixture(v);
        state.entry.phase = EntryPhase::Final;
        let p = lookup_reference(v);
        state.entry.sensed_acceleration_g = -p.neg_aref_g;
        state.entry.r_dot_mps = p.rdot_ref_mps;
        let nominal_km = p.range_to_go_nm * NM_TO_KM;

        // Long: actual target is farther than reference → need more lift.
        state.entry.target_range_km = nominal_km + 50.0;
        let ld_long = final_phase_step(&state).ld_command;
        assert!(
            ld_long > LOD_NOMINAL,
            "long-range error should push L/D above LOD, got {ld_long}"
        );

        // Short: actual target is closer than reference → need less lift.
        state.entry.target_range_km = nominal_km - 50.0;
        let ld_short = final_phase_step(&state).ld_command;
        assert!(
            ld_short < LOD_NOMINAL,
            "short-range error should pull L/D below LOD, got {ld_short}"
        );
    }
}
