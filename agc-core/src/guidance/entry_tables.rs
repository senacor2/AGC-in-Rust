//! Reference profile and named constants for the P64 / P65 / P67 entry
//! guidance law (REENTRY_CONTROL.agc).
//!
//! Each constant is doc-commented with:
//! - the original AGC label (uppercase identifier from REENTRY_CONTROL.agc)
//! - the AGC scaling note (`B`-form fixed-point)
//! - the SI value used here, with the conversion source
//!
//! Conversions:
//! - 1 ft         = 0.304_8 m
//! - 1 nmi        = 1.852 km
//! - g_AGC        = 32.2 ft/s² = 9.815 m/s² (AGC convention; the live SI value
//!   `9.806_65 m/s²` is in `programs::p61_p67::G0_MPS2`. The two differ by
//!   ~0.1 % — well below the 1 % we expect from the single-exponential
//!   atmosphere model.)
//! - VSAT_AGC     = 25 766.197 ft/s = 7 853.547 m/s
//! - R_E_AGC      = 21 202 900 ft   = 6 462 643.92 m (the Apollo-era figure;
//!   we deliberately use the modern mean radius 6 371 000 m via
//!   `programs::p21::R_EARTH` for navigation, and call out where the two
//!   values appear separately).
//!
//! All values are SI f64. No fixed-point arithmetic at runtime.

/// Nominal vehicle maximum lift-to-drag ratio (dimensionless).
///
/// AGC: `LADPAD` (ENTRY_LEXICON.agc:139) / `LAD` (REENTRY_CONTROL.agc:469).
/// Scaling: `1`. SI: `0.30`.
pub const LAD_NOMINAL: f64 = 0.30;

/// Nominal final-phase L/D (dimensionless).
///
/// AGC: `LODPAD` (ENTRY_LEXICON.agc:149) / `LOD`. Scaling: `1`. SI: `0.18`.
pub const LOD_NOMINAL: f64 = 0.18;

/// Initial value of up-control reference L/D (dimensionless).
///
/// AGC: `LEWD1` (REENTRY_CONTROL.agc:1497) — `2DEC .15`. Scaling: `1`.
pub const LEWD_INIT: f64 = 0.15;

/// Initial value of the HUNTEST L/D iteration step (dimensionless).
///
/// AGC: `DLEWD0` (REENTRY_CONTROL.agc:1503) — `2DEC -.05`. Scaling: `1`.
pub const DLEWD_INIT: f64 = -0.05;

/// L/D below which the lateral-switch threshold uses the reduced cosine band.
///
/// AGC: `L/DCMINR` (ENTRY_LEXICON.agc:156) = `LAD · cos(15°)`. Scaling: `1`.
pub const LD_CMIN_RATIO: f64 = LAD_NOMINAL * 0.965; // `COS15` line 1614

/// Earth-orbit circular velocity reference (m/s).
///
/// AGC: `VSAT = 25 766.197 ft/s` (REENTRY_CONTROL.agc:1493 comment).
/// SI: `25 766.197 · 0.304_8 = 7 853.5168 m/s`.
pub const VSAT_MPS: f64 = 7_853.516_8;

/// "Final-phase exit" velocity above which `init_p67`/PREDICT3 must wait (m/s).
///
/// AGC: `VFINAL1` (REENTRY_CONTROL.agc:1597) — `2DEC .523942273 (B0 in 2VS)`.
/// SI: `0.523_942_273 · 2 · VSAT_MPS = 8 229.6 m/s ≈ 27 000 ft/s`.
pub const VFINAL1_MPS: f64 = 2.0 * VSAT_MPS * 0.523_942_273;

/// Velocity at which the final-phase guidance switches to PREDICT3 (m/s).
///
/// AGC: `VFINAL` (REENTRY_CONTROL.agc:1595) — `26 600 ft/s`.
pub const VFINAL_MPS: f64 = 2.0 * VSAT_MPS * 0.516_180_16;

/// Minimum exit velocity (m/s); HUNTEST falls through to PREFINAL below this.
///
/// AGC: `VLMIN` (REENTRY_CONTROL.agc:1530) — `2DEC .34929485` (18 000 ft/s).
pub const VLMIN_MPS: f64 = 2.0 * VSAT_MPS * 0.349_294_85;

/// Up-control acceleration scaling (g) — gate on `D` (drag in g).
///
/// AGC: `KA2 = .008` scaled `805 FPSS` (REENTRY_CONTROL.agc:1601).
/// Algebra: `KA2_unscaled = .008 · 805 ft/s² = 6.44 ft/s² = 0.2 g`.
pub const KA2_G: f64 = 0.2;

/// Maximum allowed deceleration before `GLIMITER` engages (g, AGC `GMAX/2`).
///
/// AGC: `GMAX/2 = .16` scaled `2DEC` (REENTRY_CONTROL.agc:1505) — `8 g / 2`.
pub const GMAX_HALF_G: f64 = 8.0 / 2.0;

/// Lateral-angle bias term (rad) — half-nautical-mile dead-band.
///
/// AGC: `LATBIAS = .00003 (4 REV)` (REENTRY_CONTROL.agc:1561) ≈ `1.88e-4 rad`.
pub const LAT_BIAS_RAD: f64 = 0.000_03 * 4.0 * core::f64::consts::PI;

/// Range-error threshold (km) — exceeding this drops to `EntryPhase::Ballistic`.
///
/// **MS-E3 design choice**, not directly from AGC. See
/// `specs/entry-guidance-plan.md` §5 MS-E3. Selected by user during planning.
pub const RANGE_ERR_THRESHOLD_KM: f64 = 500.0;

/// Final-phase range curve fit — constant term `Q2 = 21600 NM scale = .151`.
///
/// AGC: `Q2` (REENTRY_CONTROL.agc:1516 area). Computed at runtime from `LAD`
/// in the AGC; we use a single nominal value. Units: nautical miles.
pub const Q2_NM: f64 = 1280.0; // ≈ Q2(LAD = 0.3); fits the AGC nominal.

/// Final-phase range curve fit — slope term `Q3` (dim `nm·s/m`).
///
/// AGC: `Q3 = 2DEC .167003132` (REENTRY_CONTROL.agc:1516) scaled
/// `.07 · 2VS / 21600` = `0.07 · 2 · 25766.2 / 21600 ≈ 0.167003`. The constant
/// is therefore `0.07 nm·s/(m·rad) · ...`. We store the per-m/s slope.
pub const Q3_NM_PER_MPS: f64 = 0.07 * 2.0 * VSAT_MPS / 21_600.0;

/// Gamma-correction range coefficient `Q5` (nautical miles per rad of γ).
///
/// AGC: `Q5 = .326388889` scaled `.3 · 23500 / 21600`. The scaled value gives
/// `Q5 · 21600 NM = 7050 NM`. We store this scaled-out coefficient.
pub const Q5_NM_PER_RAD: f64 = 7_050.0;

/// Gamma-correction range zero offset `Q6` (rad of γ).
///
/// AGC: `Q6 = .0349` (REENTRY_CONTROL.agc:1520) ≈ `2 deg`.
pub const Q6_RAD: f64 = 0.034_9;

/// Down-control range constant `KC3` (nautical miles · s²/m²).
///
/// AGC: `KC3 = -.0247622232` scaled `-(4 VS · VS / 2π · 805 · R_E)`. The
/// underlying expression is `-4·VS²/(2π·805·R_E)`; we store the SI value
/// `-4·VSAT²/(2π·g₀·R_E)` (nm·s²/m²).
pub const KC3_NM_PER_M2_PER_S2: f64 = -4.0 * VSAT_MPS * VSAT_MPS
    / (2.0 * core::f64::consts::PI * 9.815 * 6_462_643.92);

/// Up-range scaling constant `C12` (nautical miles, per natural-log unit).
///
/// AGC: `C12 = 2DEC .00684572901` scaled `32 · 28500 / (R_E_AGC · 2π)`.
/// We store the scaled-out value in nm; the log term is dimensionless.
pub const C12_NM: f64 = 0.006_845_729_01 * 21_600.0;

/// Pre-tabulated point along the entry reference profile (velocity sample).
///
/// Source: REENTRY_CONTROL.agc lines 1412–1467 — four parallel columns
/// stored in the AGC's `RDOTREF`, `RTOGO`, `-AREF`, and `DRANGE/D(L/D)` tables.
///
/// Units (after conversion):
/// - `velocity_mps` — m/s (linearly spaced from `VFINAL_MPS` upward).
/// - `rdot_ref_mps` — m/s (negative = descending).
/// - `range_to_go_nm` — nautical miles.
/// - `neg_aref_g` — g (always negative; deceleration).
/// - `drange_dld_nm` — nautical miles per unit L/D (∂Range/∂(L/D)).
#[derive(Clone, Copy, Debug)]
pub struct ReferencePoint {
    /// Sample velocity (m/s).
    pub velocity_mps: f64,
    /// Reference altitude rate (m/s).
    pub rdot_ref_mps: f64,
    /// Reference range remaining (nm).
    pub range_to_go_nm: f64,
    /// Reference deceleration (g, negative).
    pub neg_aref_g: f64,
    /// Sensitivity of range to L/D (nm per unit L/D).
    pub drange_dld_nm: f64,
}

/// Velocity scaling factor used by `VREFER` at REENTRY_CONTROL.agc:1369 —
/// `V / 51 532.3946 ft/s` = `V / (2 · VSAT)`. SI: `2 · VSAT_MPS m/s`.
const V_SCALE_MPS: f64 = 2.0 * VSAT_MPS;

/// Velocity column scaling for `-AREF` (line 1440): the AGC stores it as
/// `-AREF / 805 ft/s²`. Convert to g via `805 ft/s² / 32.2 ft/s² ≈ 25 g`.
const AREF_SCALE_G: f64 = 805.0 / 32.2;

/// Velocity column scaling for `RTOGO` and `DRANGE/D(L/D)` (lines 1426, 1455):
/// AGC stores both as `value / 2700 NM`. Multiply by 2700.
const RANGE_SCALE_NM: f64 = 2700.0;

/// Velocity column scaling for `RDOTREF` (line 1412):
/// `8 · RDOT / 2VS` → divide by 8, multiply by `2 · VSAT`.
const RDOT_SCALE_MPS: f64 = 2.0 * VSAT_MPS / 8.0;

/// AGC reference profile (REENTRY_CONTROL.agc:1369–1467).
///
/// 13 sample points. The independent variable is `VREFER` at line 1369. The
/// four columns we use are RDOTREF (line 1412), RTOGO (1426), -AREF (1440)
/// and DRANGE/D(L/D) (1455). The AGC also stores DRANGE/DA and DRANGE/DRDOT
/// columns (1383, 1397) which we don't currently consume — they are needed
/// only when the analytic HUNTEST `PREDICT3` correction is enabled (MS-E6).
///
/// **Velocity ordering**: the table runs from slow (drogue-deploy regime,
/// ~300 m/s at sample 0) up to lunar-return entry interface (~10 668 m/s at
/// sample 12 — explicitly tagged "HIGH VELOCITY FOR SAFETY" at AGC line
/// 1381). Sample 11 corresponds to ~23 500 ft/s ≈ 7 163 m/s, just below VSAT.
pub const REFERENCE_PROFILE: [ReferencePoint; 13] = [
    ReferencePoint {
        // i = 0 — VREFER = .019288 → 994 ft/s = 303 m/s.
        velocity_mps: 0.019_288 * V_SCALE_MPS,
        rdot_ref_mps: -0.013_400_1 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.000_806_7 * RANGE_SCALE_NM,
        neg_aref_g: -0.051_099 * AREF_SCALE_G,
        drange_dld_nm: 0.004_491 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 1 — VREFER = .040809 → 2103 ft/s = 641 m/s.
        velocity_mps: 0.040_809 * V_SCALE_MPS,
        rdot_ref_mps: -0.013_947 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.003_296_3 * RANGE_SCALE_NM,
        neg_aref_g: -0.074_534 * AREF_SCALE_G,
        drange_dld_nm: 0.008_081 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 2 — VREFER = .076107 → 3922 ft/s = 1195 m/s.
        velocity_mps: 0.076_107 * V_SCALE_MPS,
        rdot_ref_mps: -0.013_462 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.008_185_2 * RANGE_SCALE_NM,
        neg_aref_g: -0.101_242 * AREF_SCALE_G,
        drange_dld_nm: 0.016_030 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 3 — VREFER = .122156 → 6295 ft/s = 1918 m/s.
        velocity_mps: 0.122_156 * V_SCALE_MPS,
        rdot_ref_mps: -0.011_813 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.017_148 * RANGE_SCALE_NM,
        neg_aref_g: -0.116_646 * AREF_SCALE_G,
        drange_dld_nm: 0.035_815 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 4 — VREFER = .165546 → 8531 ft/s = 2600 m/s.
        velocity_mps: 0.165_546 * V_SCALE_MPS,
        rdot_ref_mps: -0.009_563_1 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.027_926 * RANGE_SCALE_NM,
        neg_aref_g: -0.122_360 * AREF_SCALE_G,
        drange_dld_nm: 0.069_422 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 5 — VREFER = .196012 → 10101 ft/s = 3079 m/s.
        velocity_mps: 0.196_012 * V_SCALE_MPS,
        rdot_ref_mps: -0.008_069_46 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.037 * RANGE_SCALE_NM,
        neg_aref_g: -0.127_081 * AREF_SCALE_G,
        drange_dld_nm: 0.104_519 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 6 — VREFER = .271945 → 14013 ft/s = 4271 m/s.
        velocity_mps: 0.271_945 * V_SCALE_MPS,
        rdot_ref_mps: -0.006_828 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.063_298 * RANGE_SCALE_NM,
        neg_aref_g: -0.147_453 * AREF_SCALE_G,
        drange_dld_nm: 0.122 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 7 — VREFER = .309533 → 15951 ft/s = 4863 m/s.
        velocity_mps: 0.309_533 * V_SCALE_MPS,
        rdot_ref_mps: -0.008_069_46 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.077_889 * RANGE_SCALE_NM,
        neg_aref_g: -0.155_528 * AREF_SCALE_G,
        drange_dld_nm: 0.172_407 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 8 — VREFER = .356222 → 18356 ft/s = 5595 m/s.
        velocity_mps: 0.356_222 * V_SCALE_MPS,
        rdot_ref_mps: -0.010_979_1 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.098_815 * RANGE_SCALE_NM,
        neg_aref_g: -0.149_565 * AREF_SCALE_G,
        drange_dld_nm: 0.252_852 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 9 — VREFER = .404192 → 20828 ft/s = 6349 m/s.
        velocity_mps: 0.404_192 * V_SCALE_MPS,
        rdot_ref_mps: -0.015_149_8 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.127_519 * RANGE_SCALE_NM,
        neg_aref_g: -0.118_509 * AREF_SCALE_G,
        drange_dld_nm: 0.363_148 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 10 — VREFER = .448067 → 23089 ft/s = 7038 m/s.
        velocity_mps: 0.448_067 * V_SCALE_MPS,
        rdot_ref_mps: -0.017_981_7 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.186_963 * RANGE_SCALE_NM,
        neg_aref_g: -0.034_907 * AREF_SCALE_G,
        drange_dld_nm: 0.512_963 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 11 — VREFER = .456023 → 23500 ft/s = 7163 m/s.
        velocity_mps: 0.456_023 * V_SCALE_MPS,
        rdot_ref_mps: -0.015_906_1 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.238_148 * RANGE_SCALE_NM,
        neg_aref_g: -0.007_950 * AREF_SCALE_G,
        drange_dld_nm: 0.558_519 * RANGE_SCALE_NM,
    },
    ReferencePoint {
        // i = 12 — VREFER = .67918 → 34999 ft/s = 10 668 m/s.
        // AGC line 1381 comment: "HIGH VELOCITY FOR SAFETY".
        velocity_mps: 0.679_18 * V_SCALE_MPS,
        rdot_ref_mps: -0.015_906_1 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.294_185_185 * RANGE_SCALE_NM,
        neg_aref_g: -0.007_950 * AREF_SCALE_G,
        drange_dld_nm: 0.558_519 * RANGE_SCALE_NM,
    },
];

/// Linearly interpolate the reference profile at the given velocity.
///
/// Returns the bracketing-row interpolation for `velocity_mps`. For inputs
/// outside `[VFINAL_MPS, VSAT_MPS]` the closest endpoint is returned (no
/// extrapolation — the AGC profile is only valid inside its sampled band).
pub fn lookup_reference(velocity_mps: f64) -> ReferencePoint {
    let n = REFERENCE_PROFILE.len();
    if velocity_mps <= REFERENCE_PROFILE[0].velocity_mps {
        return REFERENCE_PROFILE[0];
    }
    if velocity_mps >= REFERENCE_PROFILE[n - 1].velocity_mps {
        return REFERENCE_PROFILE[n - 1];
    }
    let mut hi = 1;
    while hi < n && REFERENCE_PROFILE[hi].velocity_mps < velocity_mps {
        hi += 1;
    }
    let lo = hi - 1;
    let p0 = REFERENCE_PROFILE[lo];
    let p1 = REFERENCE_PROFILE[hi];
    let span = p1.velocity_mps - p0.velocity_mps;
    let t = if span > 0.0 {
        (velocity_mps - p0.velocity_mps) / span
    } else {
        0.0
    };
    // Exact-endpoint short-circuits avoid floating-point drift of ~1e-12
    // when the query velocity lands precisely on a table sample.
    if t == 0.0 {
        return p0;
    }
    if t == 1.0 {
        return p1;
    }
    ReferencePoint {
        velocity_mps,
        rdot_ref_mps: p0.rdot_ref_mps + t * (p1.rdot_ref_mps - p0.rdot_ref_mps),
        range_to_go_nm: p0.range_to_go_nm + t * (p1.range_to_go_nm - p0.range_to_go_nm),
        neg_aref_g: p0.neg_aref_g + t * (p1.neg_aref_g - p0.neg_aref_g),
        drange_dld_nm: p0.drange_dld_nm + t * (p1.drange_dld_nm - p0.drange_dld_nm),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-TAB-1: the table covers from sub-sonic (~300 m/s, sample 0) up
    /// through hyper-velocity entry (~10 668 m/s, sample 12). The exact
    /// endpoints are derived from the AGC `VREFER` scaled values at
    /// `REENTRY_CONTROL.agc:1369` and `1381`.
    #[test]
    fn tc_tab_1_table_endpoints() {
        let first = REFERENCE_PROFILE[0];
        let last = REFERENCE_PROFILE[REFERENCE_PROFILE.len() - 1];
        // Sample 0: 0.019288 · 2·VSAT ≈ 303 m/s.
        assert!(
            (first.velocity_mps - 0.019_288 * V_SCALE_MPS).abs() < 1e-9,
            "first sample velocity off"
        );
        // Sample 12: 0.67918 · 2·VSAT ≈ 10 668 m/s.
        assert!(
            (last.velocity_mps - 0.679_18 * V_SCALE_MPS).abs() < 1e-9,
            "last sample velocity off"
        );
    }

    /// TC-TAB-2: lookup at exactly a sample velocity returns the sample.
    #[test]
    fn tc_tab_2_lookup_exact_sample() {
        let p = lookup_reference(REFERENCE_PROFILE[5].velocity_mps);
        let q = REFERENCE_PROFILE[5];
        assert!((p.rdot_ref_mps - q.rdot_ref_mps).abs() < 1e-9);
        assert!((p.range_to_go_nm - q.range_to_go_nm).abs() < 1e-9);
    }

    /// TC-TAB-3: lookup at the midpoint of two samples returns the average.
    #[test]
    fn tc_tab_3_lookup_midpoint() {
        let a = REFERENCE_PROFILE[3];
        let b = REFERENCE_PROFILE[4];
        let mid = (a.velocity_mps + b.velocity_mps) * 0.5;
        let p = lookup_reference(mid);
        let expected_rtogo = 0.5 * (a.range_to_go_nm + b.range_to_go_nm);
        assert!(
            (p.range_to_go_nm - expected_rtogo).abs() < 1e-6,
            "expected {expected_rtogo} nm, got {} nm",
            p.range_to_go_nm
        );
    }

    /// TC-TAB-4: out-of-band velocities clamp to the endpoints.
    #[test]
    fn tc_tab_4_clamp_oob() {
        let first = REFERENCE_PROFILE[0];
        let last = REFERENCE_PROFILE[REFERENCE_PROFILE.len() - 1];
        let lo = lookup_reference(0.0);
        let hi = lookup_reference(1.0e9);
        assert!((lo.velocity_mps - first.velocity_mps).abs() < 1e-9);
        assert!((hi.velocity_mps - last.velocity_mps).abs() < 1e-9);
    }

    /// TC-TAB-5: rangeToGo is strictly non-decreasing in velocity (more energy
    /// = more range to fly before decelerating to VFINAL).
    #[test]
    fn tc_tab_5_rtogo_monotone() {
        for w in REFERENCE_PROFILE.windows(2) {
            assert!(
                w[1].range_to_go_nm >= w[0].range_to_go_nm,
                "range_to_go decreased between v={} and v={}",
                w[0].velocity_mps,
                w[1].velocity_mps
            );
        }
    }

    /// TC-TAB-6: VFINAL1 > VFINAL > VLMIN, in the order REENTRY_CONTROL.agc
    /// asserts. Constant-time check so the build fails if the AGC scale
    /// factors are ever fat-fingered.
    #[test]
    fn tc_tab_6_velocity_ordering() {
        const _: () = assert!(VFINAL1_MPS > VFINAL_MPS);
        const _: () = assert!(VFINAL_MPS > VLMIN_MPS);
    }
}
