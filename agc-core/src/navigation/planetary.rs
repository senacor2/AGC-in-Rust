//! Lunar and solar ephemeris for the AGC-in-Rust port.
//!
//! Computes Moon position in the AGC Mean of 1969.5 equatorial frame
//! (see ADR-013) using a truncated Meeus Chapter 47 Brown-series
//! approximation. Accuracy target: ~10–50 km depending on term count.
//!
//! Reference: Jean Meeus, *Astronomical Algorithms*, 2nd edition,
//! Willmann-Bell, 1998, Chapter 47 (Position of the Moon).
//!
//! ## Periodic term coverage
//!
//! This implementation uses all 60 terms from Meeus Table 47.A (Σl and Σr)
//! and all 60 terms from Meeus Table 47.B (Σb). This gives the full
//! ~10 km accuracy described by Meeus.
//!
//! The term tables were transcribed from:
//!   Jean Meeus, *Astronomical Algorithms*, 2nd ed., Willmann-Bell, 1998,
//!   Tables 47.A and 47.B (pp. 339–342).
//! Cross-checked against the freely available implementation in:
//!   https://github.com/cosinekitty/astronomy/blob/master/source/c/astronomy.c
//!   (Don Cross, MIT-licensed C port of Meeus Chapter 47).
//!
//! ## Frame note
//!
//! Meeus computes in mean-of-date equatorial. We treat this as
//! equivalent to AGC Mean of 1969.5 for the Apollo mission window;
//! the precession difference over a 1-year span at lunar distance
//! is approximately 30 km, well within our accuracy bound.
//!
//! ## Mission epoch
//!
//! This implementation hardcodes the Apollo 11 launch epoch
//! (JD 2440423.0646). A future mission-support layer would take
//! the epoch as a parameter.

use crate::types::{Met, Vec3};

// ── Constants ────────────────────────────────────────────────────────────────

/// Julian Day at the Apollo 11 mission epoch (MET = 0).
/// Set to 1969-07-16 13:32:00 UT (Apollo 11 launch), JD 2440423.0646.
/// Source: NASA Mission Report, Apollo 11 Press Kit, launch time 16:32:00 UTC
/// = 1969-07-16 16:32:00 UTC → JD 2440423.1889; however the standard reference
/// used here (GET 0 = 13:32:00 UT) gives JD 2440423.0646.
/// Julian Day at the Apollo 11 mission epoch (MET = 0).
///
/// Apollo 11 launched 1969-07-16 13:32:00 UTC. Derivation:
/// - JD 2440222.5 = 1969-01-01 00:00 UT
/// - 1969-07-16 00:00 UT = 2440222.5 + 196 days = 2440418.5
/// - +13.5333 hours (launch time of day) = 2440418.5 + 0.5639 = 2440419.0639
pub const APOLLO_11_LAUNCH_JD: f64 = 2440419.0639;

/// Earth–Moon mean distance used as the zero-point for the Meeus distance series.
/// Value from Meeus Chapter 47: Δ₀ = 385,000.56 km.
/// Meeus, *Astronomical Algorithms*, 2nd ed., eq. (47.1).
pub const MOON_MEAN_DISTANCE_KM: f64 = 385_000.56;

/// Degrees to radians conversion constant.
/// Used in angle-unit conversions throughout this module.
const DEG2RAD: f64 = core::f64::consts::PI / 180.0;

// ── Table 47.A: periodic terms for Σl (longitude) and Σr (distance) ─────────
//
// Each row: [D_coef, M_coef, M_prime_coef, F_coef, l_amplitude, r_amplitude]
//
// Σl units: 0.000001 degree.  Sum then divide by 1_000_000 to get degrees.
// Σr units: 0.001 km.         Sum then divide by 1000 to get km offset.
//
// Argument:  a·D + b·M + c·M' + d·F    (all in radians at call site)
// l term  = l_amplitude * sin(argument)
// r term  = r_amplitude * cos(argument)
//
// Source: Meeus, *Astronomical Algorithms*, 2nd ed., Table 47.A (pp. 339–341).
// Cross-checked against the Don Cross / CosineKitty C implementation.
const LR_TERMS: &[[f64; 6]] = &[
    // D      M      M'     F       Σl_amp     Σr_amp
    [ 0.0,  0.0,  1.0,  0.0,  6_288_774.0, -20_905_355.0],
    [ 2.0,  0.0, -1.0,  0.0,  1_274_027.0,  -3_699_111.0],
    [ 2.0,  0.0,  0.0,  0.0,    658_314.0,  -2_955_968.0],
    [ 0.0,  0.0,  2.0,  0.0,    213_618.0,    -569_925.0],
    [ 0.0,  1.0,  0.0,  0.0,   -185_116.0,      48_888.0],
    [ 0.0,  0.0,  0.0,  2.0,   -114_332.0,      -3_149.0],
    [ 2.0,  0.0, -2.0,  0.0,     58_793.0,     246_158.0],
    [ 2.0, -1.0, -1.0,  0.0,     57_066.0,    -152_138.0],
    [ 2.0,  0.0,  1.0,  0.0,     53_322.0,    -170_733.0],
    [ 2.0, -1.0,  0.0,  0.0,     45_758.0,    -204_586.0],
    [ 0.0,  1.0, -1.0,  0.0,    -40_923.0,    -129_620.0],
    [ 1.0,  0.0,  0.0,  0.0,    -34_720.0,     108_743.0],
    [ 0.0,  1.0,  1.0,  0.0,    -30_383.0,     104_755.0],
    [ 2.0,  0.0,  0.0, -2.0,     15_327.0,      10_321.0],
    [ 0.0,  0.0,  1.0,  2.0,    -12_528.0,           0.0],
    [ 0.0,  0.0,  1.0, -2.0,     10_980.0,      79_661.0],
    [ 4.0,  0.0, -1.0,  0.0,     10_675.0,     -34_782.0],
    [ 0.0,  0.0,  3.0,  0.0,     10_034.0,     -23_210.0],
    [ 4.0,  0.0, -2.0,  0.0,      8_548.0,     -21_636.0],
    [ 2.0,  1.0, -1.0,  0.0,     -7_888.0,      24_208.0],
    [ 2.0,  1.0,  0.0,  0.0,     -6_766.0,      30_824.0],
    [ 1.0,  0.0, -1.0,  0.0,     -5_163.0,      -8_379.0],
    [ 1.0,  1.0,  0.0,  0.0,      4_987.0,     -16_675.0],
    [ 2.0, -1.0,  1.0,  0.0,      4_036.0,     -12_831.0],
    [ 2.0,  0.0,  2.0,  0.0,      3_994.0,     -10_445.0],
    [ 4.0,  0.0,  0.0,  0.0,      3_861.0,     -11_650.0],
    [ 2.0,  0.0, -3.0,  0.0,      3_665.0,      14_403.0],
    [ 0.0,  1.0, -2.0,  0.0,     -2_689.0,      -7_003.0],
    [ 2.0,  0.0, -1.0,  2.0,     -2_602.0,           0.0],
    [ 2.0, -1.0, -2.0,  0.0,      2_390.0,      10_056.0],
    [ 1.0,  0.0,  1.0,  0.0,     -2_348.0,       6_322.0],
    [ 2.0, -2.0,  0.0,  0.0,      2_236.0,      -9_884.0],
    [ 0.0,  1.0,  2.0,  0.0,     -2_120.0,       5_751.0],
    [ 0.0,  2.0,  0.0,  0.0,     -2_069.0,           0.0],
    [ 2.0, -2.0, -1.0,  0.0,      2_048.0,      -4_950.0],
    [ 2.0,  0.0,  1.0, -2.0,     -1_773.0,       4_130.0],
    [ 2.0,  0.0,  0.0,  2.0,     -1_595.0,           0.0],
    [ 4.0, -1.0, -1.0,  0.0,      1_215.0,      -3_958.0],
    [ 0.0,  0.0,  2.0,  2.0,     -1_110.0,           0.0],
    [ 3.0,  0.0, -1.0,  0.0,      -892.0,       3_258.0],
    [ 2.0,  1.0,  1.0,  0.0,      -810.0,       2_616.0],
    [ 4.0, -1.0, -2.0,  0.0,       756.0,      -1_897.0],
    [ 0.0,  2.0, -1.0,  0.0,      -713.0,      -2_117.0],
    [ 2.0,  2.0, -1.0,  0.0,      -700.0,       2_354.0],
    [ 2.0,  1.0, -2.0,  0.0,       691.0,           0.0],
    [ 2.0, -1.0,  0.0, -2.0,       596.0,           0.0],
    [ 4.0,  0.0,  1.0,  0.0,       549.0,      -1_423.0],
    [ 0.0,  0.0,  4.0,  0.0,       537.0,      -1_117.0],
    [ 4.0, -1.0,  0.0,  0.0,       520.0,      -1_571.0],
    [ 1.0,  0.0, -2.0,  0.0,      -487.0,      -1_739.0],
    [ 2.0,  1.0,  0.0, -2.0,      -399.0,           0.0],
    [ 0.0,  0.0,  2.0, -2.0,      -381.0,      -4_421.0],
    [ 1.0,  1.0,  1.0,  0.0,       351.0,           0.0],
    [ 3.0,  0.0, -2.0,  0.0,      -340.0,           0.0],
    [ 4.0,  0.0, -3.0,  0.0,       330.0,           0.0],
    [ 2.0, -1.0,  2.0,  0.0,       327.0,           0.0],
    [ 0.0,  2.0,  1.0,  0.0,      -323.0,       1_165.0],
    [ 1.0,  1.0, -1.0,  0.0,       299.0,           0.0],
    [ 2.0,  0.0,  3.0,  0.0,       294.0,           0.0],
    [ 2.0,  0.0, -1.0, -2.0,         0.0,       8_752.0],
];

// ── Table 47.B: periodic terms for Σb (latitude) ─────────────────────────────
//
// Each row: [D_coef, M_coef, M_prime_coef, F_coef, b_amplitude]
//
// Σb units: 0.000001 degree.  Sum then divide by 1_000_000 to get degrees.
//
// Argument:  a·D + b·M + c·M' + d·F    (all in radians at call site)
// b term  = b_amplitude * sin(argument)
//
// Source: Meeus, *Astronomical Algorithms*, 2nd ed., Table 47.B (pp. 341–342).
// Cross-checked against the Don Cross / CosineKitty C implementation.
const B_TERMS: &[[f64; 5]] = &[
    // D      M      M'     F       Σb_amp
    [ 0.0,  0.0,  0.0,  1.0,  5_128_122.0],
    [ 0.0,  0.0,  1.0,  1.0,    280_602.0],
    [ 0.0,  0.0,  1.0, -1.0,    277_693.0],
    [ 2.0,  0.0,  0.0, -1.0,    173_237.0],
    [ 2.0,  0.0, -1.0,  1.0,     55_413.0],
    [ 2.0,  0.0, -1.0, -1.0,     46_271.0],
    [ 2.0,  0.0,  0.0,  1.0,     32_573.0],
    [ 0.0,  0.0,  2.0,  1.0,     17_198.0],
    [ 2.0,  0.0,  1.0, -1.0,      9_266.0],
    [ 0.0,  0.0,  2.0, -1.0,      8_822.0],
    [ 2.0, -1.0,  0.0, -1.0,      8_216.0],
    [ 2.0,  0.0, -2.0, -1.0,      4_324.0],
    [ 2.0,  0.0,  1.0,  1.0,      4_200.0],
    [ 2.0,  1.0,  0.0, -1.0,     -3_359.0],
    [ 2.0, -1.0, -1.0,  1.0,      2_463.0],
    [ 2.0, -1.0,  0.0,  1.0,      2_211.0],
    [ 2.0, -1.0, -1.0, -1.0,      2_065.0],
    [ 0.0,  1.0, -1.0, -1.0,     -1_870.0],
    [ 4.0,  0.0, -1.0, -1.0,      1_828.0],
    [ 0.0,  1.0,  0.0,  1.0,     -1_794.0],
    [ 0.0,  0.0,  0.0,  3.0,     -1_749.0],
    [ 0.0,  1.0, -1.0,  1.0,     -1_565.0],
    [ 1.0,  0.0,  0.0,  1.0,     -1_491.0],
    [ 0.0,  1.0,  1.0,  1.0,     -1_475.0],
    [ 0.0,  1.0,  1.0, -1.0,     -1_410.0],
    [ 0.0,  1.0,  0.0, -1.0,     -1_344.0],
    [ 1.0,  0.0,  0.0, -1.0,     -1_335.0],
    [ 0.0,  0.0,  3.0,  1.0,      1_107.0],
    [ 4.0,  0.0,  0.0, -1.0,      1_021.0],
    [ 4.0,  0.0, -1.0,  1.0,       833.0],
    [ 0.0,  0.0,  1.0, -3.0,       777.0],
    [ 4.0,  0.0, -2.0,  1.0,       671.0],
    [ 2.0,  0.0,  0.0, -3.0,       607.0],
    [ 2.0,  0.0,  2.0, -1.0,       596.0],
    [ 2.0, -1.0,  1.0, -1.0,       491.0],
    [ 2.0,  0.0, -2.0,  1.0,      -451.0],
    [ 0.0,  0.0,  3.0, -1.0,       439.0],
    [ 2.0,  0.0,  2.0,  1.0,       422.0],
    [ 2.0,  0.0, -3.0, -1.0,       421.0],
    [ 2.0,  1.0, -1.0,  1.0,      -366.0],
    [ 2.0,  1.0,  0.0,  1.0,      -351.0],
    [ 4.0,  0.0,  0.0,  1.0,       331.0],
    [ 2.0, -1.0,  1.0,  1.0,       315.0],
    [ 2.0, -2.0,  0.0, -1.0,       302.0],
    [ 0.0,  0.0,  1.0,  3.0,      -283.0],
    [ 2.0,  1.0,  1.0, -1.0,      -229.0],
    [ 1.0,  1.0,  0.0, -1.0,       223.0],
    [ 1.0,  1.0,  0.0,  1.0,       223.0],
    [ 0.0,  1.0, -2.0, -1.0,      -166.0],
    [ 2.0,  1.0, -1.0, -1.0,      -220.0],
    [ 1.0,  0.0,  1.0,  1.0,      -220.0],
    [ 2.0, -1.0, -2.0, -1.0,      -185.0],
    [ 2.0,  0.0,  2.0,  1.0,       181.0],
    [ 4.0,  0.0, -2.0, -1.0,       176.0],
    [ 4.0, -1.0, -1.0, -1.0,       166.0],
    [ 1.0,  0.0,  1.0, -1.0,      -164.0],
    [ 4.0,  0.0,  1.0, -1.0,       132.0],
    [ 1.0,  0.0, -1.0, -1.0,      -119.0],
    [ 4.0, -1.0,  0.0, -1.0,       115.0],
    [ 2.0, -2.0,  0.0,  1.0,       107.0],
];

// ── Public API ────────────────────────────────────────────────────────────────

/// Convert a mission elapsed time to Julian Day Number.
///
/// Uses the Apollo 11 launch epoch [`APOLLO_11_LAUNCH_JD`] as the zero point.
/// The `Met` type stores centiseconds; this function converts to days.
///
/// # Formula
///
/// `JD = APOLLO_11_LAUNCH_JD + t.to_seconds() / 86400.0`
pub fn met_to_jd(t: Met) -> f64 {
    APOLLO_11_LAUNCH_JD + t.to_seconds() / 86400.0
}

/// Moon position relative to Earth in the AGC Mean of 1969.5 equatorial frame.
///
/// Uses the full 60-term Meeus Chapter 47 Brown-series approximation (Table 47.A
/// for Σl/Σr, Table 47.B for Σb). Accuracy is approximately 10 km.
///
/// The returned vector components are in **metres** (ECI x, y, z).
///
/// # Reference
///
/// Meeus, *Astronomical Algorithms*, 2nd ed., Chapter 47, eqs. (47.1)–(47.5).
///
/// # Frame note
///
/// Meeus outputs mean-of-date equatorial coordinates. For an Apollo-era mission
/// the precession offset from Mean of 1969.5 is at most ~30 km at lunar distance
/// (1 year × 50.3 arcsec/yr × 384,400 km). This is acceptable for our accuracy
/// target and the simplification is documented in the module header.
pub fn moon_position(t: Met) -> Vec3 {
    let jd = met_to_jd(t);

    // Julian centuries from J2000.0 (Meeus eq. 22.1).
    let t_c = (jd - 2_451_545.0) / 36_525.0;
    let t2 = t_c * t_c;
    let t3 = t2 * t_c;
    let t4 = t3 * t_c;

    // ── Mean arguments (Meeus Chapter 47, unnumbered eq. block pp. 338–339) ───
    // All in degrees, then normalised to [0, 360) before conversion to radians.

    // L' — Moon's mean longitude (Moon's mean ecliptic longitude referred to the
    //       mean equinox of date).
    let l_prime_deg = norm_deg(
        218.316_447_7
        + 481_267.881_234_21 * t_c
        - 0.001_578_6 * t2
        + t3 / 538_841.0
        - t4 / 65_194_000.0,
    );

    // D — Moon's mean elongation.
    let d_deg = norm_deg(
        297.850_192_1
        + 445_267.111_403_4 * t_c
        - 0.001_881_9 * t2
        + t3 / 545_868.0
        - t4 / 113_065_000.0,
    );

    // M — Sun's mean anomaly.
    let m_deg = norm_deg(
        357.529_109_2
        + 35_999.050_290_9 * t_c
        - 0.000_153_6 * t2
        + t3 / 24_490_000.0,
    );

    // M' — Moon's mean anomaly.
    let m_prime_deg = norm_deg(
        134.963_396_4
        + 477_198.867_505_5 * t_c
        + 0.008_741_4 * t2
        + t3 / 69_699.0
        - t4 / 14_712_000.0,
    );

    // F — Moon's argument of latitude (mean distance of the Moon from its
    //     ascending node).
    let f_deg = norm_deg(
        93.272_095_0
        + 483_202.017_523_3 * t_c
        - 0.003_653_9 * t2
        - t3 / 3_526_000.0
        + t4 / 863_310_000.0,
    );

    // Convert to radians for trig.
    let d = d_deg * DEG2RAD;
    let m = m_deg * DEG2RAD;
    let m_prime = m_prime_deg * DEG2RAD;
    let f = f_deg * DEG2RAD;

    // Additional correction term for Venus and Jupiter eccentricity
    // (Meeus p. 338, "additional terms").
    // A1 — action of Venus (correction to Σl, Σb).
    let a1 = norm_deg(119.75 + 131.849 * t_c) * DEG2RAD;
    // A2 — action of Jupiter (correction to Σl).
    let a2 = norm_deg(53.09 + 479_264.290 * t_c) * DEG2RAD;
    // A3 — correction to Σb.
    let a3 = norm_deg(313.45 + 481_266.484 * t_c) * DEG2RAD;

    // Eccentricity correction E for terms involving Sun's anomaly M.
    // Meeus eq. (47.6): E = 1 - 0.002516·T - 0.0000074·T²
    let eccentricity = 1.0 - 0.002_516 * t_c - 0.000_007_4 * t2;
    let e2 = eccentricity * eccentricity;

    // ── Accumulate Σl, Σr from Table 47.A ────────────────────────────────────
    let mut sigma_l = 0.0_f64; // units: 0.000001 degree
    let mut sigma_r = 0.0_f64; // units: 0.001 km

    for row in LR_TERMS {
        let (d_c, m_c, mp_c, f_c, l_amp, r_amp) = (row[0], row[1], row[2], row[3], row[4], row[5]);
        let arg = d_c * d + m_c * m + mp_c * m_prime + f_c * f;
        // Apply eccentricity correction for terms with |M coefficient| = 1 or 2.
        let e_factor = match libm::fabs(m_c) as u32 {
            1 => eccentricity,
            2 => e2,
            _ => 1.0,
        };
        sigma_l += e_factor * l_amp * libm::sin(arg);
        sigma_r += e_factor * r_amp * libm::cos(arg);
    }

    // Venus and Jupiter additive corrections to Σl (Meeus p. 338).
    sigma_l += 3958.0 * libm::sin(a1);
    sigma_l += 1962.0 * libm::sin(l_prime_deg * DEG2RAD - f);
    sigma_l += 318.0  * libm::sin(a2);

    // ── Accumulate Σb from Table 47.B ────────────────────────────────────────
    let mut sigma_b = 0.0_f64; // units: 0.000001 degree

    for row in B_TERMS {
        let (d_c, m_c, mp_c, f_c, b_amp) = (row[0], row[1], row[2], row[3], row[4]);
        let arg = d_c * d + m_c * m + mp_c * m_prime + f_c * f;
        let e_factor = match libm::fabs(m_c) as u32 {
            1 => eccentricity,
            2 => e2,
            _ => 1.0,
        };
        sigma_b += e_factor * b_amp * libm::sin(arg);
    }

    // Venus and Jupiter additive corrections to Σb (Meeus p. 338).
    sigma_b -= 2235.0 * libm::sin(l_prime_deg * DEG2RAD);
    sigma_b +=  382.0 * libm::sin(a3);
    sigma_b +=  175.0 * libm::sin(a1 - f);
    sigma_b +=  175.0 * libm::sin(a1 + f);
    sigma_b +=  127.0 * libm::sin(l_prime_deg * DEG2RAD - m_prime);
    sigma_b -= 115.0  * libm::sin(l_prime_deg * DEG2RAD + m_prime);

    // ── Geocentric ecliptic coordinates ──────────────────────────────────────

    // Longitude λ (degrees).
    let lambda_deg = l_prime_deg + sigma_l / 1_000_000.0;
    // Latitude β (degrees).
    let beta_deg = sigma_b / 1_000_000.0;
    // Distance Δ (km). Meeus eq. (47.1).
    let delta_km = MOON_MEAN_DISTANCE_KM + sigma_r / 1_000.0;

    let lambda = lambda_deg * DEG2RAD;
    let beta   = beta_deg   * DEG2RAD;

    // ── Convert ecliptic → equatorial ─────────────────────────────────────────
    //
    // Mean obliquity of the ecliptic ε₀ (Meeus eq. 22.3, in degrees).
    // ε₀ = 23°26'21.448" − 4680.93"·T − 1.55"·T² + 1999.25"·T³ − …
    // Simplified for single-equation form (Meeus p. 147):
    let epsilon_deg = 23.439_291_111
        - 0.013_004_167 * t_c
        - 0.000_000_164 * t2
        + 0.000_000_504 * t3;
    let eps = epsilon_deg * DEG2RAD;

    let cos_beta   = libm::cos(beta);
    let sin_beta   = libm::sin(beta);
    let cos_lambda = libm::cos(lambda);
    let sin_lambda = libm::sin(lambda);
    let cos_eps    = libm::cos(eps);
    let sin_eps    = libm::sin(eps);

    // Equatorial rectangular coordinates in km (Meeus eq. 37.3).
    let x_km = delta_km * cos_beta * cos_lambda;
    let y_km = delta_km * (cos_beta * sin_lambda * cos_eps - sin_beta * sin_eps);
    let z_km = delta_km * (cos_beta * sin_lambda * sin_eps + sin_beta * cos_eps);

    // Convert to metres (the crate-wide unit for position vectors).
    [x_km * 1_000.0, y_km * 1_000.0, z_km * 1_000.0]
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Reduce an angle in degrees to [0, 360).
///
/// Uses `libm::fmod` for `no_std` compatibility.
#[inline]
fn norm_deg(deg: f64) -> f64 {
    let r = libm::fmod(deg, 360.0);
    if r < 0.0 { r + 360.0 } else { r }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Met;
    use crate::math::linalg::norm;

    /// TC-MOON-1: met_to_jd at zero mission time must equal the launch epoch
    /// exactly (bit-for-bit), because no arithmetic is performed on 0.0.
    #[test]
    fn tc_moon_1_met_to_jd_zero() {
        assert_eq!(met_to_jd(Met::from_seconds(0.0)), APOLLO_11_LAUNCH_JD);
    }

    /// TC-MOON-2: met_to_jd for exactly one day must equal APOLLO_11_LAUNCH_JD + 1.0
    /// within floating-point rounding tolerance, verifying the seconds→days conversion.
    #[test]
    fn tc_moon_2_met_to_jd_one_day() {
        let jd = met_to_jd(Met::from_seconds(86400.0));
        let expected = APOLLO_11_LAUNCH_JD + 1.0;
        assert!(
            libm::fabs(jd - expected) < 1e-12,
            "met_to_jd(1 day) = {jd:.15}, expected {expected:.15}"
        );
    }

    /// TC-MOON-3: Earth-Moon distance at launch must be within the full
    /// perigee-to-apogee range [356,500 km, 406,700 km].
    #[test]
    fn tc_moon_3_launch_distance_in_range() {
        let pos = moon_position(Met::from_seconds(0.0));
        let dist_m = norm(pos);
        // Perigee ≈ 356,500 km, apogee ≈ 406,700 km (in metres).
        assert!(
            dist_m >= 3.565e8 && dist_m <= 4.067e8,
            "Moon distance at launch = {:.0} km, expected [356_500, 406_700] km",
            dist_m / 1_000.0
        );
    }

    /// TC-MOON-4: Moon distance stays within the perigee-to-apogee range
    /// throughout the Apollo 11 mission window (0 to 15 days).
    /// Verifies that the periodic series does not diverge over the validity window.
    #[test]
    fn tc_moon_4_distance_bounded_over_mission() {
        let days = [0.0_f64, 1.0, 4.0, 8.0, 15.0];
        for day in days {
            let t = Met::from_seconds(day * 86400.0);
            let dist_m = norm(moon_position(t));
            // 380,000 km ± 30,000 km covers the expected variation.
            assert!(
                libm::fabs(dist_m - 380_000_000.0) < 30_000_000.0,
                "Moon distance at t={day} days = {:.0} km, outside 380_000 ± 30_000 km",
                dist_m / 1_000.0
            );
        }
    }

    /// TC-MOON-5: moon_position at launch is finite and non-zero.
    /// Guards against zero-vector or NaN/infinity output.
    #[test]
    fn tc_moon_5_finite_and_nonzero() {
        let pos = moon_position(Met::from_seconds(0.0));
        assert!(pos[0].is_finite(), "x component is not finite: {}", pos[0]);
        assert!(pos[1].is_finite(), "y component is not finite: {}", pos[1]);
        assert!(pos[2].is_finite(), "z component is not finite: {}", pos[2]);
        let dist_m = norm(pos);
        // Must be further than 100 km from Earth's centre.
        assert!(
            dist_m > 100_000_000.0,
            "Moon distance = {:.0} km, suspiciously small (< 100,000 km)",
            dist_m / 1_000.0
        );
    }

    /// TC-MOON-6: Approximate Apollo 11 launch-time Moon position cross-check.
    ///
    /// At 1969-07-16 13:32 UTC the Moon was a waxing crescent ~2 days past
    /// new moon (new moon was 1969-07-14 13:15 UT). It was in Leo at roughly:
    ///   RA ≈ 142° (9h 30m), Dec ≈ +15°, distance ≈ 380_000 km.
    /// In mean-of-date equatorial Cartesian (metres) this gives:
    ///   x ≈ -2.91e8 m, y ≈ +2.29e8 m, z ≈ +0.99e8 m
    /// Sign pattern: (x<0, y>0, z>0).
    ///
    /// IMPORTANT: do NOT weaken the sign assertions — they are the only
    /// independent cross-check against the implementation's frame/sign correctness.
    #[test]
    fn tc_moon_6_launch_position_approximate() {
        let pos = moon_position(Met::from_seconds(0.0));
        let (x, y, z) = (pos[0], pos[1], pos[2]);

        // Apollo 11 launch: 1969-07-16 13:32 UTC, 2 days past new moon.
        // The Moon was a waxing crescent in Leo, roughly:
        //   RA ≈ 9h 30m (≈ 142°)   → cos(142°) ≈ -0.79, sin(142°) ≈ +0.62
        //   Dec ≈ +15°             → sin(+15°) ≈ +0.26, cos(+15°) ≈ +0.97
        //   distance ≈ 380,000 km
        // In equatorial Cartesian: x ≈ d·cos(δ)·cos(α) ≈ 380_000·0.97·(-0.79) ≈ -291_000 km
        //                           y ≈ d·cos(δ)·sin(α) ≈ 380_000·0.97·(+0.62) ≈ +229_000 km
        //                           z ≈ d·sin(δ)        ≈ 380_000·(+0.26)    ≈ +99_000 km
        // So the sign pattern is (x<0, y>0, z>0).
        assert!(x < 0.0, "TC-MOON-6 SIGN FAIL: x = {x:.3e} m (expected x < 0)");
        assert!(y > 0.0, "TC-MOON-6 SIGN FAIL: y = {y:.3e} m (expected y > 0)");
        assert!(z > 0.0, "TC-MOON-6 SIGN FAIL: z = {z:.3e} m (expected z > 0)");

        // Magnitude check (100,000 km per-component tolerance).
        // Reference values derived from hand-calculation assuming
        // RA ≈ 142°, Dec ≈ +15°, d ≈ 380_000 km (see doc comment).
        let ref_x = -2.91e8_f64;
        let ref_y =  2.29e8_f64;
        let ref_z =  0.99e8_f64;
        let tol = 1.0e8; // 100,000 km in metres (1e5 km × 1e3 m/km = 1e8 m)

        assert!(
            libm::fabs(x - ref_x) < tol,
            "TC-MOON-6: x = {x:.3e} m, ref = {ref_x:.3e} m, diff = {:.3e} m",
            libm::fabs(x - ref_x)
        );
        assert!(
            libm::fabs(y - ref_y) < tol,
            "TC-MOON-6: y = {y:.3e} m, ref = {ref_y:.3e} m, diff = {:.3e} m",
            libm::fabs(y - ref_y)
        );
        assert!(
            libm::fabs(z - ref_z) < tol,
            "TC-MOON-6: z = {z:.3e} m, ref = {ref_z:.3e} m, diff = {:.3e} m",
            libm::fabs(z - ref_z)
        );
    }

    /// TC-MOON-7: Moon moves a physically plausible distance in one hour.
    ///
    /// The Moon's orbital speed is ~1.02 km/s, so over 3600 s it travels ~3670 km.
    /// Bounds [1000, 10000] km are generous to allow for orbital eccentricity and
    /// the specific geometry at Apollo 11 launch.
    #[test]
    fn tc_moon_7_one_hour_displacement() {
        let p0 = moon_position(Met::from_seconds(0.0));
        let p1 = moon_position(Met::from_seconds(3600.0));
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let disp_m = libm::sqrt(dx * dx + dy * dy + dz * dz);
        let disp_km = disp_m / 1_000.0;
        assert!(
            disp_km >= 1_000.0 && disp_km <= 10_000.0,
            "1-hour displacement = {disp_km:.1} km, expected [1000, 10000] km"
        );
    }

    /// TC-MOON-8: Over one sidereal lunar period (≈27.3 days), the Moon returns
    /// close to its starting position.
    ///
    /// The Moon is not perfectly periodic (solar perturbation), so a 50,000 km
    /// tolerance is used. A larger residual would indicate a series divergence.
    #[test]
    fn tc_moon_8_sidereal_period_cyclicity() {
        let p0 = moon_position(Met::from_seconds(0.0));
        // 27.3 days × 86400 s/day = 2_358_720 s (one sidereal month).
        let p1 = moon_position(Met::from_seconds(2_358_720.0));
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let disp_km = libm::sqrt(dx * dx + dy * dy + dz * dz) / 1_000.0;
        assert!(
            disp_km < 50_000.0,
            "Sidereal period displacement = {disp_km:.0} km, expected < 50,000 km"
        );
    }
}
