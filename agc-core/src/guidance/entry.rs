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
    lookup_reference, C12_NM, C18_MPS, C20_G, DLEWD_INIT, FPSS_805_MPS2, HUNTEST_CONVERGED_KM,
    KB1, KB2_MPS, KC3_NM_PER_M2_PER_S2, LAD_NOMINAL, LD_CMIN_RATIO, LEWD_INIT, LOD_NOMINAL, POINT1,
    PT1_OVER_16, Q2_NM, Q3_NM_PER_MPS, Q5_NM_PER_RAD, Q6_RAD, Q7F_AGC, Q7F_G,
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
    let Some(s) = huntest_setup(state) else {
        return predict_range_table(state);
    };
    let v = velocity_mps(state);
    let rdot = state.entry.r_dot_mps;
    let lad = LAD_NOMINAL;

    // ── RANGER (REENTRY_CONTROL.agc:654–732) ──────────────────────────────────
    // COSG/2 = (1 − GAMMAL²) / 2  (truncated Taylor, line 654–657).
    let cosg_over_2 = 0.5 * (1.0 - s.gammal * s.gammal);

    // E/4 = sqrt( (VBARS − 1/2) · VBARS · (COSG/2)² · 4 + 1/16 )  (line 660–668).
    let bracket =
        (s.vbars - 0.5) * s.vbars * cosg_over_2 * cosg_over_2 * 4.0 + 1.0 / 16.0;
    if bracket <= 0.0 {
        return predict_range_table(state);
    }
    let e_over_4 = libm::sqrt(bracket);
    if e_over_4 < 1e-9 {
        return predict_range_table(state);
    }

    // ASKEP/2 = arcsin(VBARS · COSG/2 · GAMMAL / (E/4))  (line 671–676).
    let arg = (s.vbars * cosg_over_2 * s.gammal / e_over_4).clamp(-1.0, 1.0);
    let askep_rev = libm::asin(arg) / core::f64::consts::PI;

    // ASP1 = Q2 + Q3·VL  (line 680).
    let asp1_rev = (Q2_NM + Q3_NM_PER_MPS * s.vl_mps) / 21_600.0;

    // ASPUP = −C12 · log(V1²·Q7 / (VBARS·A0)) / GAMMAL1  (line 688–699).
    let log_arg = (s.v1_n * s.v1_n * Q7F_AGC / (s.vbars * s.a0_agc))
        .abs()
        .max(1e-12);
    let aspup_rev = -C12_NM * libm::log(log_arg) / s.gammal1 / 21_600.0;

    // ASPDWN = KC3 · RDOT · V / (A0 · LAD)  (line 701–710).
    let a0_real_mps2 = s.a0_agc * FPSS_805_MPS2;
    let aspdwn_nm = if a0_real_mps2.abs() < 1e-3 {
        0.0
    } else {
        KC3_NM_PER_M2_PER_S2 * rdot * v / (a0_real_mps2 * lad)
    };
    let aspdwn_rev = aspdwn_nm / 21_600.0;

    // ASP3 = Q5 · (Q6 − GAMMAL)  (line 712–717).
    let asp3_rev = Q5_NM_PER_RAD * (Q6_RAD - s.gammal) / 21_600.0;

    let asp_rev = askep_rev + asp1_rev + aspup_rev + asp3_rev + aspdwn_rev;
    let asp_km = asp_rev * 2.0 * core::f64::consts::PI * R_EARTH * 1.0e-3;

    if !asp_km.is_finite() || !(0.0..=100_000.0).contains(&asp_km) {
        return predict_range_table(state);
    }
    asp_km
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
    /// `GAMMAL` (AGC line 640). DHOOK correction omitted in stage A —
    /// equals `gammal1`.
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
    let a0_agc =
        v1_over_v * v1_over_v * (d_agc + rdot_n * rdot_n / (tem1b * TWO_C1_HS_AGC));

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

    // DHOOK / AHOOKDV correction skipped — GAMMAL = GAMMAL1 (stage A).
    let gammal = gammal1;

    let a0_g = a0_agc * 25.0; // AGC 805 FPSS = 25 g.

    Some(HuntestSetup {
        v1_n: v1_n_after_lead,
        v1_mps: v1_n_after_lead * 2.0 * VSAT_MPS,
        a0_agc,
        a0_g,
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

/// Run one P65 UPCONTRL / SKIPPER iteration and return the new vertical
/// L/D command for the next 2-s SERVICER cycle.
///
/// Implements `REENTRY_CONTROL.agc:882–1020` in three branches:
///
/// 1. **`D < Q7F`** (drag too low — vehicle is above the sensible
///    atmosphere): freeze `L/D` at its previous value. The AGC routes to
///    `KEP` (P66 ballistic) here; we keep the controller alive so the next
///    SERVICER cycle can resume closed-loop guidance when drag returns.
/// 2. **`D > A0` *or* `D > C20`** (drag exceeds predicted pull-out — we're
///    decelerating too fast): command max lift-up `L/D = LAD`
///    (AGC `STOREL/D` via `GOPOSLAD`).
/// 3. **Nominal SKIPPER feedback law** (AGC line 975 `UPCNTRL3`):
///    ```text
///    VREF    = FACT1 · (1 − sqrt(FACT2·D + ALP))      (line 918)
///    RDOTREF = LEWD · (V1 − VREF)                     (line 929)
///    ΔL/D    = −((RDOT − RDOTREF)·F1/KB1 + V − VREF)·F1/KB2
///    L/D     = LEWD + ΔL/D                             (clamped to ±LAD)
///    ```
///    The `F1 = FACTOR = (A1 − Q7)/(D − Q7)` nonlinear gain is approximated
///    by `F1 = 1` in stage A — the gain compression at large
///    deceleration is a refinement deferred until VAGC fixtures (MS-E4b).
///
/// The `DOWNCNTL` branch (V > V₁, line 1061) and the `CONSTD` constant-drag
/// branch (line 1036) are not yet implemented; both are MS-E4b scope.
pub fn upcontrol_step(state: &AgcState) -> LdUpdate {
    let lewd_prev = state.entry.lewd_ref;
    let d_g = state.entry.sensed_acceleration_g;
    let v = velocity_mps(state);
    let rdot = state.entry.r_dot_mps;

    // Branch 1: drag too low — freeze L/D (AGC `KEP`, line 895).
    if d_g < Q7F_G {
        return LdUpdate {
            ld_command: state.entry.ld_command,
            lewd_new: lewd_prev,
            dlewd_new: state.entry.dlewd,
            diffold_new_km: state.entry.diffold_km,
        };
    }

    // Need HUNTEST intermediates for branches 2 & 3. If the setup is
    // degenerate, freeze L/D — same defensive behavior as branch 1.
    let Some(s) = huntest_setup(state) else {
        return LdUpdate {
            ld_command: state.entry.ld_command,
            lewd_new: lewd_prev,
            dlewd_new: state.entry.dlewd,
            diffold_new_km: state.entry.diffold_km,
        };
    };

    // Branch 2: drag exceeds predicted pull-out drag *or* C20 trip — full
    // lift-up. AGC `CONT1` (line 909) and `NEGTESTS` (line 1008).
    if d_g > s.a0_g || d_g > C20_G {
        let ld_command = LAD_NOMINAL;
        return LdUpdate {
            ld_command,
            lewd_new: ld_command,
            dlewd_new: 0.0,
            diffold_new_km: state.entry.diffold_km,
        };
    }

    // Branch 3: nominal SKIPPER law.
    // VREF (AGC line 918): FACT1 · (1 − sqrt(FACT2·D + ALP)).
    // FACT2 and D both carry AGC-stored "fraction-of-805-FPSS" units so
    // their product is dimensionless (matches the AGC formula).
    let d_agc = d_g * G0_MPS2 / FPSS_805_MPS2;
    let inner = s.fact2 * d_agc + s.alp;
    if inner < 0.0 {
        // Pathological state — freeze L/D.
        return LdUpdate {
            ld_command: state.entry.ld_command,
            lewd_new: lewd_prev,
            dlewd_new: state.entry.dlewd,
            diffold_new_km: state.entry.diffold_km,
        };
    }
    let vref_n = s.fact1 * (1.0 - libm::sqrt(inner));
    let vref_mps = vref_n * 2.0 * VSAT_MPS;

    // RDOTREF (AGC line 929): LEWD · (V1 − VREF).
    let rdotref_mps = lewd_prev * (s.v1_mps - vref_mps);

    // FACTOR (`F1`, AGC line 967): (A1 − Q7) / (D − Q7), bounded.
    // Stage A simplification: F1 = 1 (gain compression deferred). See doc.
    let factor = 1.0;

    // ΔL/D = −((RDOT − RDOTREF)·F1/KB1 + V − VREF)·F1/KB2.
    let rdot_err = rdot - rdotref_mps;
    let v_err = v - vref_mps;
    let inner_sum = rdot_err * factor / KB1 + v_err;
    let raw_delta_ld = -inner_sum * factor / KB2_MPS;

    // Nonlinear gain reduction (AGC lines 989–998): if |ΔL/D| > PT1_OVER_16,
    // compress the magnitude by `POINT1 · |ΔL/D| + PT1_OVER_16` with sign
    // preserved. Stage A applies a clamp-only version since the AGC's
    // exact compression curve will be validated in MS-E4b.
    let delta_ld = if raw_delta_ld.abs() > PT1_OVER_16 {
        let compressed = POINT1 * raw_delta_ld.abs() + PT1_OVER_16;
        compressed.copysign(raw_delta_ld)
    } else {
        raw_delta_ld
    };

    // L/D = LEWD + ΔL/D, saturated to ±LAD (AGC `LIMITL/D`, line 1274).
    let ld_raw = lewd_prev + delta_ld;
    let ld_command = ld_raw.clamp(-LAD_NOMINAL, LAD_NOMINAL);

    LdUpdate {
        ld_command,
        // In UPCONTRL the LEWD reference is the converged HUNTEST value —
        // we keep it frozen so range prediction stays anchored. ΔL/D rides
        // on top of LEWD per cycle.
        lewd_new: lewd_prev,
        dlewd_new: delta_ld,
        diffold_new_km: state.entry.diffold_km,
    }
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
    }
}

/// P67 final-phase guidance — `PREDICT3` law from terminal velocity down
/// to drogue deploy.
///
/// AGC source: `REENTRY_CONTROL.agc:1139–1235`. Algorithm:
///
/// 1. Linearly interpolate the reference profile (RTOGO, RDOTREF, F1, F2,
///    Y) at the current velocity. `F1 = ∂Range/∂A`, `F2 = ∂Range/∂RDOT`,
///    `Y = ∂Range/∂(L/D)`. The MS-E3 table has RTOGO, RDOTREF and Y; for
///    F1 and F2 we use the sensitivity columns at AGC lines 1383–1408.
/// 2. Compute predicted range:
///    `PREDANG = RTOGO + F1·(D − AREF) + F2·(RDOT − RDOTREF)`
///    where `AREF = REFERENCE_PROFILE[..].neg_aref_g` (already in g units,
///    sign carries through).
/// 3. Compute L/D command:
///    `L/D = LOD_NOMINAL + (THETAH − PREDANG) / Y`
///    where THETAH is the actual range-to-go from `state.entry.target_range_km`.
/// 4. Saturate `L/D` to `±LAD_NOMINAL`. GLIMITER (line 1247, `D > GMAX/2 →
///    clip`) is deferred to MS-E6b.
///
/// Stage A simplification: F1 and F2 are approximated as zero — the
/// dominant range-tracking term is the `Y · (THETAH − RTOGO)` correction.
/// Including the analytic F1, F2 sensitivities needs the additional table
/// columns (DRANGE/DA, DRANGE/DRDOT at AGC lines 1383, 1397) which we'll
/// add when the VAGC fixture-match pass lands in MS-E6b.
pub fn final_phase_step(state: &AgcState) -> LdUpdate {
    let v = velocity_mps(state);
    let rtogo_nm = state.entry.target_range_km / NM_TO_KM;

    let p = lookup_reference(v);

    // PREDANG (predicted range in nm) — F1 and F2 approximated as 0 for
    // stage A; the RTOGO column itself anchors the prediction.
    let predang_nm = p.range_to_go_nm;

    // L/D = LOD + (THETAH − PREDANG) / Y. Y = DRANGE/D(L/D), already in nm.
    let theta_minus_predang = rtogo_nm - predang_nm;
    let ld_command_raw = if p.drange_dld_nm.abs() > 1e-9 {
        LOD_NOMINAL + theta_minus_predang / p.drange_dld_nm
    } else {
        LOD_NOMINAL
    };
    let ld_command = ld_command_raw.clamp(-LAD_NOMINAL, LAD_NOMINAL);

    LdUpdate {
        ld_command,
        // PREDICT3 doesn't iterate LEWD — freeze it from HUNTEST/UPCONTRL.
        lewd_new: state.entry.lewd_ref,
        dlewd_new: 0.0,
        diffold_new_km: state.entry.diffold_km,
    }
}

/// Decide the next entry-guidance phase.
///
/// Returns `Some(next_phase)` to request a transition, or `None` to stay in
/// the current phase. Outcomes depend on the current phase:
///
/// **From `EntryPhase::Entry`** (HUNTEST iteration in progress):
/// - `Some(Final)` once `V < VFINAL1` (REENTRY_CONTROL.agc:431).
/// - `Some(Ballistic)` if `|range_error| > RANGE_ERR_THRESHOLD_KM`.
/// - `Some(Skip)` if `|range_error| < HUNTEST_CONVERGED_KM`
///   (AGC line 734 `GOTOUPSY` branch).
/// - `None` otherwise.
///
/// **From `EntryPhase::Skip`** (P65 UPCONTRL):
/// - `Some(Final)` once `V < VFINAL1` *or* `V − VL < C18`
///   (AGC line 902 `VLTEST → PREFINAL`).
/// - `Some(Ballistic)` if drag `D < Q7F_G` (AGC `KEP` routing at line 895
///   — above the sensible atmosphere, coast ballistically).
/// - `Some(Ballistic)` if `|range_error| > RANGE_ERR_THRESHOLD_KM`.
/// - `None` otherwise.
///
/// **From `EntryPhase::Ballistic`** (P66): no automatic return to a
/// closed-loop phase. Only the global `V < VFINAL1` terminal check fires.
pub fn select_phase(state: &AgcState) -> Option<EntryPhase> {
    let v = velocity_mps(state);

    // Terminal-velocity transition applies in both Entry and Skip phases.
    if v < VFINAL1_MPS {
        return Some(EntryPhase::Final);
    }

    let range_err_km =
        (state.entry.target_range_km - state.entry.predicted_range_km).abs();

    match state.entry.phase {
        EntryPhase::Entry => {
            // HUNTEST divergence → P66 ballistic.
            if range_err_km > RANGE_ERR_THRESHOLD_KM {
                return Some(EntryPhase::Ballistic);
            }
            // HUNTEST convergence → P65 skip-out (UPCONTRL).
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
            // Persistent divergence → P66 ballistic.
            if range_err_km > RANGE_ERR_THRESHOLD_KM {
                return Some(EntryPhase::Ballistic);
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
        state.entry.target_range_km =
            state.entry.predicted_range_km + 0.5 * HUNTEST_CONVERGED_KM;
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

    /// TC-MSE4-SP-3: in Skip, |range_err| > 500 km → Ballistic.
    #[test]
    fn tc_mse4_sp_3_skip_divergence_to_ballistic() {
        let mut state = fixture(VFINAL1_MPS + 1_000.0);
        state.entry.phase = EntryPhase::Skip;
        // Drag above Q7F_G so the assertion exercises the *range-error*
        // path, not the MS-E5 low-drag path.
        state.entry.sensed_acceleration_g = 0.5;
        state.entry.target_range_km = state.entry.predicted_range_km + 1_500.0;
        assert_eq!(select_phase(&state), Some(EntryPhase::Ballistic));
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

    /// TC-MSE6-FP-2: when `target_range_km` matches the table's RTOGO at
    /// the current V, the L/D command equals LOD_NOMINAL — the nominal
    /// "no correction needed" output.
    #[test]
    fn tc_mse6_fp_2_nominal_ld_at_zero_correction() {
        use crate::guidance::entry_tables::lookup_reference;
        let v = VFINAL1_MPS - 200.0;
        let mut state = fixture(v);
        state.entry.phase = EntryPhase::Final;
        let p = lookup_reference(v);
        // Set target = RTOGO so (THETAH − PREDANG) = 0 → L/D = LOD.
        state.entry.target_range_km = p.range_to_go_nm * NM_TO_KM;
        let upd = final_phase_step(&state);
        assert!(
            (upd.ld_command - LOD_NOMINAL).abs() < 1e-12,
            "expected L/D = LOD = {LOD_NOMINAL}, got {}",
            upd.ld_command
        );
    }

    /// TC-MSE6-FP-3: range error in the correctable direction moves L/D in
    /// the expected sense — long-range error pushes L/D up (lift to extend),
    /// short-range error pushes L/D down (push down to shorten).
    #[test]
    fn tc_mse6_fp_3_ld_sense_from_range_error() {
        use crate::guidance::entry_tables::lookup_reference;
        let v = VFINAL1_MPS - 200.0;
        let mut state = fixture(v);
        state.entry.phase = EntryPhase::Final;
        let p = lookup_reference(v);
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
