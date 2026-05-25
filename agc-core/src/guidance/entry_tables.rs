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

/// "V quit" velocity (m/s) below which HUNTEST steering stops.
///
/// AGC: `VQUIT` (REENTRY_CONTROL.agc:1539) — `2DEC .019405269` (1000 ft/s).
pub const VQUIT_MPS: f64 = 2.0 * VSAT_MPS * 0.019_405_269;

/// AGC drag-scaling constant — `2·C1·HS` in the stored fixed-point form.
///
/// AGC: `2C1HS` (REENTRY_CONTROL.agc:1589) — `2DEC .0215983264`. Algebra:
/// `2·1.25·28500·805 / (2·VSAT_ft/s)²`. Used dimensionless against
/// AGC-scaled velocity and acceleration variables.
pub const TWO_C1_HS_AGC: f64 = 0.021_598_326_4;

/// Initial HUNTEST minimum drag for up-control (AGC `Q7F`).
///
/// AGC: `Q7F` (REENTRY_CONTROL.agc:1522) — `2DEC .0074534161` (6 ft/s² /
/// 805 ft/s² = `0.186 g`). Drives `VL` via the up-control fit.
pub const Q7F_AGC: f64 = 0.007_453_416_1;

/// `Q7F` expressed in g (= `Q7F_AGC · 25`). UPCONTRL's `D < Q7 → KEP`
/// branch (REENTRY_CONTROL.agc:895) compares against this threshold.
pub const Q7F_G: f64 = Q7F_AGC * 25.0;

/// HUNTEST `CHOOK` constant — base of the `AHOOKDV` correction
/// (REENTRY_CONTROL.agc:1567 `2DEC 1 B-6` = `1 / 2⁶` = `1/64`).
pub const CHOOK: f64 = 1.0 / 64.0;

/// HUNTEST `CH1` constant — scales the `(AHOOKDV + 1/16) · DVL² / DHOOK / VBARS`
/// correction subtracted from `GAMMAL1` (REENTRY_CONTROL.agc:1571
/// `2DEC .32 B1` = `0.32 · 2` = `0.64`).
pub const CH1: f64 = 0.64;

/// HUNTEST `1/16TH` constant — added to `AHOOKDV` before the `CH1` scale,
/// representing the "1 + AHOOK·DVL" term in the (AGC-fixed-point)
/// correction (REENTRY_CONTROL.agc:1491 `DP2(-4)` = `2⁻⁴` = `0.0625`).
pub const ONE_SIXTEENTH: f64 = 1.0 / 16.0;

/// HUNTEST `AHOOKDV` divisor — equals `2⁶` (the `SR 6` in
/// REENTRY_CONTROL.agc:621 trace; divides `DHOOK` before the `Q7`
/// normalisation produces the dimensionless `AHOOKDV`).
pub const AHOOKDV_DIVISOR: f64 = 64.0;

/// AGC's "1 g" reference acceleration — 32.2 ft/s² in SI = 9.81456 m/s².
///
/// Used to convert between `sensed_acceleration_g` (which uses the modern
/// SI `G0_MPS2 = 9.806_65`) and the AGC's `D / 805 FPSS` scaling. The
/// 0.1 % difference is well below the accuracy of the entry guidance.
pub const G_AGC_MPS2: f64 = 32.2 * 0.304_8;

/// AGC's `805 FPSS` drag scale-factor (m/s²). Equals `25 · G_AGC_MPS2`.
pub const FPSS_805_MPS2: f64 = 25.0 * G_AGC_MPS2;

/// Up-control acceleration scaling (g) — gate on `D` (drag in g).
///
/// AGC: `KA2 = .008` scaled `805 FPSS` (REENTRY_CONTROL.agc:1601).
/// Algebra: `KA2_unscaled = .008 · 805 ft/s² = 6.44 ft/s² = 0.2 g`.
pub const KA2_G: f64 = 0.2;

/// Maximum allowed deceleration before `GLIMITER` engages (g, AGC `GMAX/2`).
///
/// AGC: `GMAX/2 = .16` scaled `2DEC` (REENTRY_CONTROL.agc:1505) — `8 g / 2`.
pub const GMAX_HALF_G: f64 = 8.0 / 2.0;

/// Hard upper deceleration limit (g) — `GMAX` in REENTRY_CONTROL.agc.
/// Above this `GLIMITER` unconditionally clips the L/D command to LAD.
pub const GMAX_G: f64 = 8.0;

/// `2HSGMXSQ` constant — `(2·HS·GMAX)² / (2·VS)⁴` in AGC scaling.
///
/// AGC: `2HSGMXSQ = 2DEC .0000305717` (REENTRY_CONTROL.agc:1583) =
/// `(2·28500·8·32.2 / (4·VS·VS))²`. Algebraically `(2HS_AGC · GMAX_AGC)²`:
/// `(0.017_278_661_1 · 0.32)² ≈ 0.0000305717`. Used in the `2HSGMXSQ /
/// VSQUARE_AGC` term of GLIMITER's `XLIM` (line 1262).
pub const TWO_HS_GMAX_SQ_AGC: f64 = 0.000_030_571_7;

/// Lateral-angle bias term (rad) — half-nautical-mile dead-band.
///
/// AGC: `LATBIAS = .00003 (4 REV)` (REENTRY_CONTROL.agc:1561) ≈ `1.88e-4 rad`.
pub const LAT_BIAS_RAD: f64 = 0.000_03 * 4.0 * core::f64::consts::PI;

/// Range-error threshold (km) — exceeding this drops to `EntryPhase::Ballistic`.
///
/// **MS-E3 design choice**, not directly from AGC. See
/// `specs/entry-guidance-plan.md` §5 MS-E3. Selected by user during planning.
pub const RANGE_ERR_THRESHOLD_KM: f64 = 500.0;

/// HUNTEST-converged range error (km). Below this, HUNTEST stops iterating
/// and the controller transitions to `EntryPhase::Skip` (P65 UPCONTRL).
///
/// AGC: `25NM` (REENTRY_CONTROL.agc:1545) — `2DEC .0011574074` ≈ `25/21600`,
/// applied at line 734 as `IF ABS(THETAH−ASP) − 25NM NEG, GO TO UPSY`.
/// SI: `25 · 1.852 km ≈ 46.3 km`.
pub const HUNTEST_CONVERGED_KM: f64 = 25.0 * 1.852;

/// Velocity band (m/s) for the V−VL−C18 PREFINAL transition test.
///
/// AGC: `C18` (REENTRY_CONTROL.agc:1510) — `2DEC .0097026346` (500/2VS) =
/// 500 ft/s. SI: `500 · 0.3048 ≈ 152.4 m/s`.
pub const C18_MPS: f64 = 500.0 * 0.304_8;

/// Drag threshold (g) above which UPCONTRL commands max lift-up `L/D = LAD`.
///
/// AGC: `C20` (REENTRY_CONTROL.agc:1541) — `175 ft/s²` = `175/805` in
/// 805-FPSS scaling = `0.217 g`. Comment: "LIFT UP IF ABOVE C20".
pub const C20_G: f64 = 175.0 / 805.0 * 25.0;

/// Drag threshold (g) above which UPCONTRL suppresses the lateral switch.
///
/// AGC: `C21` (REENTRY_CONTROL.agc:1543) — `140 ft/s²` ≈ `0.174 g`.
pub const C21_G: f64 = 140.0 / 805.0 * 25.0;

/// Minimum drag (g) below which UPCONTRL branches to KEP (P66 ballistic).
///
/// AGC: `Q7MIN` (REENTRY_CONTROL.agc:1609) `= KA4 = 40/805 = 0.049689`,
/// stored as `2DEC .049689441` — equivalent to `40 ft/s²` ≈ `1.243 m/s²`
/// ≈ `0.127 g`. Note that `Q7` itself (a HUNTEST-iterated variable) and
/// `Q7MIN` are distinct values; in REENTRY_CONTROL.agc `Q7MIN` is the
/// floor used in UPCONTRL's CONTINU2 block.
pub const Q7MIN_G: f64 = 40.0 / 805.0 * 25.0;

/// SKIPPER feedback gain `KB1` (dimensionless).
///
/// AGC: `1/KB1` (REENTRY_CONTROL.agc:1535) — `2DEC .29411765` = `1/3.4`.
/// We store the divisor `KB1 = 3.4`.
pub const KB1: f64 = 3.4;

/// SKIPPER feedback gain `KB2` in SI (m/s).
///
/// AGC: `-1/KB2` (REENTRY_CONTROL.agc:1537) declared as
/// `-1/(0.0034 · 2·VS_ft/s)`. Multiplying by the velocity normalisation
/// `2·VSAT_mps` gives the SI feedback gain.
/// `KB2_MPS = 0.0034 · 2 · VSAT_MPS ≈ 53.4 m/s`.
pub const KB2_MPS: f64 = 0.003_4 * 2.0 * VSAT_MPS;

/// `PT1/16` (REENTRY_CONTROL.agc:1591) — `0.1 · 2^-4 = 0.00625`. Nonlinear
/// SKIPPER gain-reduction threshold.
pub const PT1_OVER_16: f64 = 0.1 * (1.0 / 16.0);

/// `POINT1` (REENTRY_CONTROL.agc:1499) — `0.1`. Used as the linear-gain
/// shoulder in the SKIPPER nonlinear gain reducer.
pub const POINT1: f64 = 0.1;

/// Final-phase range curve fit — `Q21` coefficient (rev per L/D).
///
/// AGC: `Q21 = 2DEC .0231481481` (REENTRY_CONTROL.agc:1526) — `500/21600`.
/// Used at line 113-116 to build `Q2 = LAD · Q21 + Q22` (rev), where Q2 is
/// the LAD-dependent constant term of `ASP1 = Q2 + Q3·VL`.
pub const Q21_AGC: f64 = 500.0 / 21_600.0;

/// Final-phase range curve fit — `Q22` constant (rev).
///
/// AGC: `Q22 = 2DEC -.053333333` (REENTRY_CONTROL.agc:1528) — `-1152/21600`.
/// Pairs with [`Q21_AGC`] to compute `Q2 = LAD · Q21 + Q22`.
pub const Q22_AGC: f64 = -1152.0 / 21_600.0;

/// Final-phase range curve fit — slope term `Q3` (rev per VL_normalised).
///
/// AGC: `Q3 = 2DEC .167003132` (REENTRY_CONTROL.agc:1516) — the literal
/// `.07 · 2VS_ft/s / 21600`. Pairs with `VL_normalised = VL_mps / (2·VSAT)`
/// to give a contribution in revolutions: `ASP1_rev = Q2_rev + Q3 · VL_n`.
pub const Q3_AGC: f64 = 0.167_003_132;

/// Gamma-correction range coefficient `Q5` (rev per rad of γ).
///
/// AGC: `Q5 = 2DEC .326388889` (REENTRY_CONTROL.agc:1518) — `.3 · 23500/21600`.
/// Pairs directly with `(Q6 - GAMMAL)` in radians to give rev.
pub const Q5_AGC: f64 = 0.326_388_889;

/// Gamma-correction range zero offset `Q6` (rad of γ).
///
/// AGC: `Q6 = 2DEC .0349` (REENTRY_CONTROL.agc:1520) ≈ `2 deg`.
pub const Q6_RAD: f64 = 0.034_9;

/// Down-control range constant `KC3` — AGC-native dimensionless value.
///
/// AGC: `KC3 = 2DEC -.0247622232` (REENTRY_CONTROL.agc:1573) defined as
/// `-(4·VS²/(2π·805·R_E))`. The formula at line 701-710 evaluates
/// `ASPDWN_rev = KC3 · RDOT · V / A0 / LAD` against the AGC-normalised
/// operands `RDOT/(2·VSAT)`, `V/(2·VSAT)`, `A0/FPSS_805`. The result is in
/// revolutions of the Earth (1 rev = 2π·R_E). Stored here as the literal
/// AGC value; the call site applies the velocity / drag normalisations.
///
/// Prior versions of this constant absorbed the normalisations into the SI
/// value but did so wrongly (used `g₀` instead of `FPSS_805` and dropped the
/// `(2·VSAT)²` term), inflating `ASPDWN` by a factor of ~1163 and forcing
/// every steep-descent input through the table fallback in `predict_range`.
/// See ticket #42 for the full diagnosis.
pub const KC3_AGC: f64 = -0.024_762_223_2;

/// Up-range scaling constant `C12` — AGC-native dimensionless value (rev / log-unit).
///
/// AGC: `C12 = 2DEC .00684572901` (REENTRY_CONTROL.agc:1533) — derived
/// `32 · 28500 / (R_E_AGC · 2π)`. Pairs with `GAMMAL1` in radians in the
/// `ASPUP_rev = -C12 · log(...) / GAMMAL1` evaluation.
pub const C12_AGC: f64 = 0.006_845_729_01;

/// DOWNCNTL / CONSTD feedback gain `K1D` — physical (post-SL8) form.
///
/// AGC erasable: `K1D = 2DEC .0314453125` (REENTRY_CONTROL.agc:1547) =
/// `C16·805/256` where C16 = 0.01. The `/256` is undone by the AGC's
/// `SL 8D` shift at the end of CONSTD1 (line 1057). We bake the `·256`
/// into the constant so SI call sites use it directly:
/// `K1D · (D_AGC − DREF_AGC)` against AGC-normalised drag.
///
/// Numeric: `0.0314453125 · 256 = 8.05 = 0.01 · 805`.
pub const K1D_AGC: f64 = 8.05;

/// DOWNCNTL / CONSTD feedback gain `K2D` — physical (post-SL8) form.
///
/// AGC erasable: `K2D = 2DEC -.201298418` (REENTRY_CONTROL.agc:1549) =
/// `-C17·2·VS/256` where C17 = 0.001 and `2·VS` is in ft/s. Same SL8
/// deferral as [`K1D_AGC`]. Numeric: `-0.201298418 · 256 ≈ -51.532` —
/// which equals `-0.001 · 2·VSAT_ft/s`. Applied to AGC-normalised
/// `(RDOT − RDTR) = (rdot_n − lad·(v1_n−v_n))`, the contribution to L/D
/// is `-0.00328 · rdot_mps`-equivalent (the `2·VSAT` normalisation
/// cancels).
pub const K2D_AGC: f64 = -51.532_395_008;

/// CONSTD reference-drag coefficient `2HS` — AGC-native dimensionless value.
///
/// AGC: `2HS = 2DEC .0172786611` (REENTRY_CONTROL.agc:1581) =
/// `2·28500·25·32.2 / (4·VS·VS)`. Pairs with `D0/V` in CONSTD's
/// `RDOTREF = -2·HS·D0/V` (line 1045).
pub const TWO_HS_AGC: f64 = 0.017_278_661_1;

/// D0 initialiser `KA3` — AGC-native dimensionless value.
///
/// AGC: `KA3 = 2DEC .44720497` (REENTRY_CONTROL.agc:1603) = `90·4/805`.
/// Used in `D0 = KA3·LEQ + KA4` (line 441-447), the equilibrium-drag fit
/// that drives CONSTD's reference profile.
pub const KA3_AGC: f64 = 0.447_204_97;

/// D0 floor `KA4` — AGC-native dimensionless value.
///
/// AGC: `KA4 = 2DEC .049689441` (REENTRY_CONTROL.agc:1605) = `40/805` ≈
/// 0.127 g. Same constant as `Q7MIN_G` reused in a different role: here
/// it's the additive floor of the `D0 = KA3·LEQ + KA4` equilibrium drag.
pub const KA4_AGC: f64 = 0.049_689_441;

/// Pre-tabulated point along the entry reference profile (velocity sample).
///
/// Source: REENTRY_CONTROL.agc lines 1369–1467 — six parallel columns
/// stored in the AGC's `VREFER`, `DRANGE/DA`, `DRANGE/DRDOT`, `RDOTREF`,
/// `RTOGO`, `-AREF`, and `DRANGE/D(L/D)` tables.
///
/// Units (after conversion):
/// - `velocity_mps` — m/s (linearly spaced from `VFINAL_MPS` upward).
/// - `rdot_ref_mps` — m/s (negative = descending).
/// - `range_to_go_nm` — nautical miles.
/// - `neg_aref_g` — g (always negative; deceleration, stored as `−AREF`).
/// - `drange_dld_nm` — nautical miles per unit L/D (`∂Range/∂(L/D)`).
/// - `drange_da_nm_per_g` — nautical miles per g of drag (`∂Range/∂D`,
///   the AGC `DRANGE/DA` column; values are negative since more drag
///   shortens the trajectory).
/// - `drange_drdot_nm_per_mps` — nautical miles per m/s of altitude rate
///   (`∂Range/∂RDOT`, derived from the AGC `−DRANGE/DRDOT` column with
///   sign flipped; values are positive since less descent extends range).
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
    /// Sensitivity of range to drag (nm per g of drag).
    pub drange_da_nm_per_g: f64,
    /// Sensitivity of range to altitude rate (nm per m/s of RDOT).
    pub drange_drdot_nm_per_mps: f64,
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

/// Velocity column scaling for `DRANGE/DA` (line 1383): the AGC stores
/// `DRDA / (2700/805)` — so multiplying the stored value by `2700/805` recovers
/// the AGC `∂PREDANG_stored / ∂D_stored`, which is `∂Range_nm / ∂D_AGC_dimless`.
/// To convert that to `∂Range_nm / ∂D_g`, divide by `25` (since `D_AGC = D_g/25`).
/// Net: `stored × 2700/(805/32.2 × 25)` ≡ `stored × 2700/25` in nm per g.
const DRDA_SCALE_NM_PER_G: f64 = RANGE_SCALE_NM / AREF_SCALE_G;

/// Velocity column scaling for `−DRANGE/DRDOT` (line 1397): the AGC stores
/// `−DR/DRDOT · (2VS/8) · 2700` with implicit B-3 (×2⁻³). Empirical conversion
/// from stored to `∂Range_nm / ∂RDOT_mps`:
/// `stored × −2700 / (2·VSAT_mps)`. Sign flip absorbs the `−DR/DRDOT`
/// column-name negative, plus the B-3 (×8) factor cancels with the 8
/// pre-shift applied to `RDOT` in the AGC's interpretive arithmetic
/// (DDOUBL DDOUBL DDOUBL at REENTRY_CONTROL.agc:1196-1198).
const DRDRDOT_SCALE_NM_PER_MPS: f64 = -RANGE_SCALE_NM / (2.0 * VSAT_MPS);

/// AGC reference profile (REENTRY_CONTROL.agc:1369–1467).
///
/// 13 sample points. The independent variable is `VREFER` at line 1369. Six
/// parallel columns: DRANGE/DA (1383), DRANGE/DRDOT (1397), RDOTREF (1412),
/// RTOGO (1426), -AREF (1440), DRANGE/D(L/D) (1455). All converted to SI
/// at table-construction time so PREDICT3 reads physical quantities.
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
        drange_da_nm_per_g: -0.010_337 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -0.047_859_9 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 1 — VREFER = .040809 → 2103 ft/s = 641 m/s.
        velocity_mps: 0.040_809 * V_SCALE_MPS,
        rdot_ref_mps: -0.013_947 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.003_296_3 * RANGE_SCALE_NM,
        neg_aref_g: -0.074_534 * AREF_SCALE_G,
        drange_dld_nm: 0.008_081 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.016_550 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -0.068_366_3 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 2 — VREFER = .076107 → 3922 ft/s = 1195 m/s.
        velocity_mps: 0.076_107 * V_SCALE_MPS,
        rdot_ref_mps: -0.013_462 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.008_185_2 * RANGE_SCALE_NM,
        neg_aref_g: -0.101_242 * AREF_SCALE_G,
        drange_dld_nm: 0.016_030 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.026_935 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -0.134_346_8 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 3 — VREFER = .122156 → 6295 ft/s = 1918 m/s.
        velocity_mps: 0.122_156 * V_SCALE_MPS,
        rdot_ref_mps: -0.011_813 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.017_148 * RANGE_SCALE_NM,
        neg_aref_g: -0.116_646 * AREF_SCALE_G,
        drange_dld_nm: 0.035_815 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.042_039 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -0.275_984_6 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 4 — VREFER = .165546 → 8531 ft/s = 2600 m/s.
        velocity_mps: 0.165_546 * V_SCALE_MPS,
        rdot_ref_mps: -0.009_563_1 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.027_926 * RANGE_SCALE_NM,
        neg_aref_g: -0.122_360 * AREF_SCALE_G,
        drange_dld_nm: 0.069_422 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.058_974 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -0.473_143_7 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 5 — VREFER = .196012 → 10101 ft/s = 3079 m/s.
        velocity_mps: 0.196_012 * V_SCALE_MPS,
        rdot_ref_mps: -0.008_069_46 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.037 * RANGE_SCALE_NM,
        neg_aref_g: -0.127_081 * AREF_SCALE_G,
        drange_dld_nm: 0.104_519 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.070_721 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -0.647_208_7 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 6 — VREFER = .271945 → 14013 ft/s = 4271 m/s.
        velocity_mps: 0.271_945 * V_SCALE_MPS,
        rdot_ref_mps: -0.006_828 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.063_298 * RANGE_SCALE_NM,
        neg_aref_g: -0.147_453 * AREF_SCALE_G,
        drange_dld_nm: 0.122 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.098_538 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -1.171_693 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 7 — VREFER = .309533 → 15951 ft/s = 4863 m/s.
        velocity_mps: 0.309_533 * V_SCALE_MPS,
        rdot_ref_mps: -0.008_069_46 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.077_889 * RANGE_SCALE_NM,
        neg_aref_g: -0.155_528 * AREF_SCALE_G,
        drange_dld_nm: 0.172_407 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.107_482 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -1.466_382 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 8 — VREFER = .356222 → 18356 ft/s = 5595 m/s.
        velocity_mps: 0.356_222 * V_SCALE_MPS,
        rdot_ref_mps: -0.010_979_1 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.098_815 * RANGE_SCALE_NM,
        neg_aref_g: -0.149_565 * AREF_SCALE_G,
        drange_dld_nm: 0.252_852 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.147_762 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -1.905_171 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 9 — VREFER = .404192 → 20828 ft/s = 6349 m/s.
        velocity_mps: 0.404_192 * V_SCALE_MPS,
        rdot_ref_mps: -0.015_149_8 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.127_519 * RANGE_SCALE_NM,
        neg_aref_g: -0.118_509 * AREF_SCALE_G,
        drange_dld_nm: 0.363_148 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.193_289 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -2.547_990 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 10 — VREFER = .448067 → 23089 ft/s = 7038 m/s.
        velocity_mps: 0.448_067 * V_SCALE_MPS,
        rdot_ref_mps: -0.017_981_7 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.186_963 * RANGE_SCALE_NM,
        neg_aref_g: -0.034_907 * AREF_SCALE_G,
        drange_dld_nm: 0.512_963 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.602_557 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -4.151_220 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 11 — VREFER = .456023 → 23500 ft/s = 7163 m/s.
        velocity_mps: 0.456_023 * V_SCALE_MPS,
        rdot_ref_mps: -0.015_906_1 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.238_148 * RANGE_SCALE_NM,
        neg_aref_g: -0.007_950 * AREF_SCALE_G,
        drange_dld_nm: 0.558_519 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.999_99 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -5.813_617 * DRDRDOT_SCALE_NM_PER_MPS,
    },
    ReferencePoint {
        // i = 12 — VREFER = .67918 → 34999 ft/s = 10 668 m/s.
        // AGC line 1381 comment: "HIGH VELOCITY FOR SAFETY".
        velocity_mps: 0.679_18 * V_SCALE_MPS,
        rdot_ref_mps: -0.015_906_1 * RDOT_SCALE_MPS,
        range_to_go_nm: 0.294_185_185 * RANGE_SCALE_NM,
        neg_aref_g: -0.007_950 * AREF_SCALE_G,
        drange_dld_nm: 0.558_519 * RANGE_SCALE_NM,
        drange_da_nm_per_g: -0.999_99 * DRDA_SCALE_NM_PER_G,
        drange_drdot_nm_per_mps: -5.813_617 * DRDRDOT_SCALE_NM_PER_MPS,
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
        drange_da_nm_per_g: p0.drange_da_nm_per_g
            + t * (p1.drange_da_nm_per_g - p0.drange_da_nm_per_g),
        drange_drdot_nm_per_mps: p0.drange_drdot_nm_per_mps
            + t * (p1.drange_drdot_nm_per_mps - p0.drange_drdot_nm_per_mps),
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
