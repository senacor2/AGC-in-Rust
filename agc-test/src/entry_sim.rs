//! 3DOF atmospheric-entry integrator for end-to-end MS-E7 scenarios.
//!
//! Generates the sensed (non-gravitational) Δv that the SERVICER's PIPA
//! pipeline would otherwise receive from real flight hardware. The
//! `average_g_step` integrator inside `services::average_g::servicer_task`
//! handles gravity propagation, so this module only needs to model:
//!
//! - Atmospheric drag (`F_drag = −½·ρ·v² ·C_D ·A` along −v_rel)
//! - Aerodynamic lift (`F_lift = drag_mag · L/D`), rotated about the velocity
//!   vector by the bank angle commanded by the DAP (`DapMode::EntryRoll`).
//!
//! Force model choices (per MS-E7 planning):
//! - Point-mass `μ/r²` gravity. The actual gravity integration lives in
//!   the SERVICER; we don't apply it here, only consume the AGC's CSM state.
//! - Exponential atmosphere from [`agc_core::navigation::atmosphere::density`]
//!   (sea-level density × `exp(−h/H_s)`, `H_s = 7160 m`).
//! - Earth rotation modelled: `v_rel = v_inertial − ω⊕ × r` (#87).
//!   `ω⊕ = 7.292 115e-5 rad/s` about the ECI +Z axis. The atmosphere
//!   co-rotates with the surface, so drag and lift forces are computed
//!   against `v_rel`, not `v_inertial`. At an equatorial 32° azimuth
//!   entry the velocity slip is ~470 m/s; the downrange component
//!   propagates into the miss-distance.
//! - **No** J2 oblateness: irrelevant on the ~7-minute entry profile.
//!
//! ## Sub-stepping
//!
//! The SERVICER cycle is 2 s; atmospheric forces during peak entry can vary
//! by an order of magnitude in 2 s. The integrator sub-steps internally at
//! `SUB_STEP_S = 0.1 s` (20 sub-steps per SERVICER cycle) and accumulates
//! Δv across the sub-steps. RK2 (midpoint) is used inside each sub-step.
//!
//! ## Quantising to PIPA pulses
//!
//! `pipa_pulses_for_dv` converts a 3-vector of inertial Δv (m/s) into the
//! `[i16; 3]` PIPA counts that the SERVICER's foreground accumulator
//! deposits into `AgcState::pipa_counts`. Inverse of the AGC's
//! scale × misalignment × REFSMMAT chain; for the default
//! `REFSMMAT = I` and identity misalignment, the inverse is the trivial
//! `count = round(dv / scale)`.

use agc_core::services::average_g::PipaCalibration;
use agc_core::types::Vec3;

/// Standard gravitational parameter for Earth (m³/s²) — matches
/// `agc_core::navigation::gravity::MU_EARTH`.
const MU_EARTH: f64 = 3.986_004_418e14;

/// Earth equatorial radius (m) — matches `agc_core::programs::p21::R_EARTH`.
const R_EARTH: f64 = 6_371_000.0;

/// Integrator sub-step (s). 20 sub-steps per 2-s SERVICER cycle.
pub const SUB_STEP_S: f64 = 0.1;

/// Earth sidereal rotation rate (rad/s). Re-exported from agc-core so
/// the integrator, the AGC's drogue-deploy trigger, and the GHA chain
/// all share one value.
use agc_core::navigation::time::OMEGA_EARTH as OMEGA_EARTH_RAD_S;

/// Apollo CM mass after CSM separation (kg). Mid-range of the 5500–6000 kg
/// historical band.
pub const APOLLO_CM_MASS_KG: f64 = 5_800.0;

/// Apollo CM reference area (m²). Heat-shield diameter ≈ 3.91 m, area
/// `π·(D/2)² ≈ 12.0 m²`.
pub const APOLLO_CM_AREA_M2: f64 = 12.0;

/// Apollo CM hypersonic drag coefficient (dimensionless). Standard textbook
/// value for the blunt CM geometry at Mach 20+.
pub const APOLLO_CM_CD: f64 = 1.3;

/// Apollo CSM L/D magnitude (dimensionless). The vehicle's lift coefficient
/// is fixed; bank angle rotates the lift vector around the velocity axis.
/// Matches `agc_core::guidance::entry_tables::LAD_NOMINAL = 0.30`.
const LAD_LIFT: f64 = 0.30;

/// Apollo CM heat-shield effective nose radius (m). Used in the
/// Sutton–Graves stagnation-point heat-flux correlation. The CM forebody
/// is a spherical cap with curvature radius ≈ 4.69 m; this matches the
/// value cited in Detra & Hidalgo (1961) and Apollo entry-trajectory
/// literature.
const APOLLO_CM_NOSE_RADIUS_M: f64 = 4.69;

/// Sutton–Graves stagnation-point heat-flux coefficient `K` for Earth
/// air (kg^½ · m⁻¹). `q̇ = K · √(ρ / R_n) · v³` where `q̇` is in W/m²,
/// `ρ` in kg/m³, `R_n` in m, and `v` in m/s. Sutton & Graves (NASA TR
/// R-376, 1971) — Apollo-era reference constant.
const SUTTON_GRAVES_K: f64 = 1.7415e-4;

/// Diagnostics returned alongside the sensed Δv from
/// [`EntryIntegrator::integrate_cycle_with_diag`].
#[derive(Clone, Copy, Debug, Default)]
pub struct IntegrationDiag {
    /// Inertial sensed Δv accumulated over the cycle (m/s).
    pub sensed_dv: Vec3,
    /// Peak Sutton–Graves stagnation-point heat flux observed during the
    /// cycle (W/m²). Sampled twice per RK2 sub-step.
    pub peak_heat_flux_w_m2: f64,
}

/// Sutton–Graves stagnation-point heat flux for the Apollo CM at a given
/// inertial state. Computes `q̇ = K · √(ρ / R_n) · v_rel³` (Sutton & Graves
/// NASA TR R-376), where `v_rel` is the Earth-relative velocity (the
/// atmosphere co-rotates with the surface, so heat-flux drives off the
/// freestream velocity in the air-mass frame — same `v_rel` the drag and
/// lift forces use in `acceleration`).
///
/// Returns 0 below the sensible atmosphere (ρ ≈ 0) or at near-zero
/// relative velocity. Units: W/m². Divide by `1.0e6` for MW/m².
pub fn heat_flux_w_m2(position: Vec3, velocity_eci: Vec3) -> f64 {
    let r_mag = vec3_norm(position);
    if r_mag < 1.0 {
        return 0.0;
    }
    let omega_cross_r = [
        -OMEGA_EARTH_RAD_S * position[1],
        OMEGA_EARTH_RAD_S * position[0],
        0.0,
    ];
    let v_rel = vec3_sub(velocity_eci, omega_cross_r);
    let v_rel_mag = vec3_norm(v_rel);
    if v_rel_mag < 1.0 {
        return 0.0;
    }
    let altitude = r_mag - R_EARTH;
    let rho = agc_core::navigation::atmosphere::density(altitude);
    if rho <= 0.0 {
        return 0.0;
    }
    SUTTON_GRAVES_K * (rho / APOLLO_CM_NOSE_RADIUS_M).sqrt() * v_rel_mag.powi(3)
}

/// 3DOF entry integrator state.
#[derive(Clone, Copy, Debug)]
pub struct EntryIntegrator {
    /// Vehicle mass (kg).
    pub mass_kg: f64,
    /// Reference area (m²).
    pub ref_area_m2: f64,
    /// Hypersonic drag coefficient.
    pub cd: f64,
}

impl Default for EntryIntegrator {
    fn default() -> Self {
        Self::apollo_cm()
    }
}

impl EntryIntegrator {
    /// Apollo CM defaults: 5800 kg, 12 m², C_D = 1.3.
    pub const fn apollo_cm() -> Self {
        Self {
            mass_kg: APOLLO_CM_MASS_KG,
            ref_area_m2: APOLLO_CM_AREA_M2,
            cd: APOLLO_CM_CD,
        }
    }

    /// Integrate the spacecraft through one SERVICER cycle (2 s) and return
    /// the accumulated **sensed** Δv in the inertial frame (m/s).
    ///
    /// Inputs:
    /// - `position` — current ECI position (m); read-only.
    /// - `velocity` — current ECI velocity (m/s); read-only.
    /// - `ld_command` — vertical L/D currently commanded by AGC entry guidance.
    /// - `bank_rad` — bank angle from `DapMode::EntryRoll(_)`; 0 = lift up.
    /// - `dt_s` — total interval to integrate (usually `SERVICER_PERIOD_S = 2.0`).
    ///
    /// Internally sub-steps at `SUB_STEP_S`. Gravity is **not** applied —
    /// the AGC's `average_g_step` handles gravity propagation. Position
    /// and velocity advance during the sub-steps under combined gravity +
    /// drag + lift so the *forces* are evaluated at locally correct state;
    /// only the sensed (drag + lift) Δv is returned.
    pub fn integrate_cycle(
        &self,
        position: Vec3,
        velocity: Vec3,
        ld_command: f64,
        bank_rad: f64,
        dt_s: f64,
    ) -> Vec3 {
        self.integrate_cycle_with_diag(position, velocity, ld_command, bank_rad, dt_s)
            .sensed_dv
    }

    /// Same as [`integrate_cycle`] but also returns diagnostics from the
    /// sub-step loop — currently the peak Sutton–Graves stagnation-point
    /// heat flux observed (#96). Callers that need shape assertions on the
    /// trajectory (peak g, peak heating) use this variant; callers that
    /// only need sensed Δv keep the simpler [`integrate_cycle`].
    pub fn integrate_cycle_with_diag(
        &self,
        position: Vec3,
        velocity: Vec3,
        ld_command: f64,
        bank_rad: f64,
        dt_s: f64,
    ) -> IntegrationDiag {
        let mut pos = position;
        let mut vel = velocity;
        let mut sensed_dv: Vec3 = [0.0; 3];
        let mut peak_heat_flux_w_m2 = 0.0_f64;

        let n_substeps = ((dt_s / SUB_STEP_S).round() as usize).max(1);
        let h = dt_s / n_substeps as f64;

        for _ in 0..n_substeps {
            // RK2 midpoint: evaluate full accel at start, advance half-step,
            // evaluate again at midpoint, use midpoint accel for the full step.
            let (a_full_0, a_sensed_0) = self.acceleration(pos, vel, ld_command, bank_rad);
            // Sample heat flux at the sub-step start point.
            peak_heat_flux_w_m2 = peak_heat_flux_w_m2.max(heat_flux_w_m2(pos, vel));
            let pos_mid = vec3_add(pos, vec3_scale(vel, h * 0.5));
            let vel_mid = vec3_add(vel, vec3_scale(a_full_0, h * 0.5));
            let (a_full_mid, a_sensed_mid) =
                self.acceleration(pos_mid, vel_mid, ld_command, bank_rad);
            peak_heat_flux_w_m2 = peak_heat_flux_w_m2.max(heat_flux_w_m2(pos_mid, vel_mid));

            pos = vec3_add(pos, vec3_scale(vel_mid, h));
            vel = vec3_add(vel, vec3_scale(a_full_mid, h));

            // Sensed Δv: average of start and midpoint sensed accel.
            let a_sensed_avg = vec3_scale(vec3_add(a_sensed_0, a_sensed_mid), 0.5);
            sensed_dv = vec3_add(sensed_dv, vec3_scale(a_sensed_avg, h));
        }

        IntegrationDiag {
            sensed_dv,
            peak_heat_flux_w_m2,
        }
    }

    /// Compute `(full_accel, sensed_accel)` at the given state. `full_accel`
    /// is used internally to propagate the integrator's working copy of
    /// position/velocity; `sensed_accel` is the non-gravitational part that
    /// the AGC observes via PIPAs.
    fn acceleration(
        &self,
        position: Vec3,
        velocity: Vec3,
        ld_command: f64,
        bank_rad: f64,
    ) -> (Vec3, Vec3) {
        let r_mag = vec3_norm(position);
        let r_hat = vec3_scale(position, 1.0 / r_mag);

        // Gravity (used only to propagate the working copy; the AGC's
        // SERVICER applies its own gravity model).
        let g_mag = MU_EARTH / (r_mag * r_mag);
        let a_grav = vec3_scale(r_hat, -g_mag);

        // Earth-rotation-corrected velocity (#87): the atmosphere co-rotates
        // with the surface at `ω⊕ = OMEGA_EARTH_RAD_S` about ECI +Z, so
        // drag/lift act against `v_rel = v_inertial − ω × r`. Closed form
        // for `ω = [0, 0, ω⊕]`: `ω × r = [−ω·r_y, +ω·r_x, 0]`.
        let omega_cross_r = [
            -OMEGA_EARTH_RAD_S * position[1],
            OMEGA_EARTH_RAD_S * position[0],
            0.0,
        ];
        let v_rel = vec3_sub(velocity, omega_cross_r);
        let v_rel_mag = vec3_norm(v_rel);

        // Aerodynamic forces require non-zero relative velocity.
        if v_rel_mag < 1.0 {
            return (a_grav, [0.0; 3]);
        }
        let v_hat = vec3_scale(v_rel, 1.0 / v_rel_mag);

        let altitude = r_mag - R_EARTH;
        let rho = agc_core::navigation::atmosphere::density(altitude);

        // Drag magnitude (m/s²): F/m = ½·ρ·v_rel²·C_D·A / m
        let drag_mag =
            0.5 * rho * v_rel_mag * v_rel_mag * self.cd * self.ref_area_m2 / self.mass_kg;
        let a_drag = vec3_scale(v_hat, -drag_mag);

        // Lift direction frame:
        //   up_hat   = component of r_hat perpendicular to v_hat ("away from Earth")
        //   right_hat = v_hat × up_hat (right-hand rule, positive crossrange)
        let v_dot_r = vec3_dot(v_hat, r_hat);
        let up_raw = vec3_sub(r_hat, vec3_scale(v_hat, v_dot_r));
        let up_mag = vec3_norm(up_raw);
        let (up_hat, right_hat) = if up_mag > 1e-6 {
            let up = vec3_scale(up_raw, 1.0 / up_mag);
            let right = vec3_cross(v_hat, up);
            (up, right)
        } else {
            // Pure radial velocity (degenerate); pick an arbitrary plane.
            ([0.0, 0.0, 1.0], [0.0, 1.0, 0.0])
        };
        // Lift direction: cos(bank)·up + sin(bank)·right.
        // Bank convention: 0 = lift up (Apollo default), positive = right-bank.
        let cos_b = bank_rad.cos();
        let sin_b = bank_rad.sin();
        let lift_dir = vec3_add(vec3_scale(up_hat, cos_b), vec3_scale(right_hat, sin_b));
        // Apollo CSM has a fixed L/D magnitude (= LAD_NOMINAL). The bank
        // angle rotates the lift vector around the velocity axis; the
        // vertical projection equals `LAD·cos(bank)`. `ld_command` is the
        // signed vertical L/D (= LAD·cos(bank) by construction in
        // `resolve_roll`), so we use it ONLY through `lift_dir` and not
        // multiplicatively — multiplying again would double-count the
        // sign and flip lift-down into lift-up for negative L/D commands
        // (#86 CONSTD path).
        let _ = ld_command;
        let a_lift = vec3_scale(lift_dir, drag_mag * LAD_LIFT);

        let a_sensed = vec3_add(a_drag, a_lift);
        let a_full = vec3_add(a_grav, a_sensed);
        (a_full, a_sensed)
    }
}

/// Quantise an inertial Δv into `[i16; 3]` PIPA counts.
///
/// Assumes `REFSMMAT = I` (identity), so inertial = platform. Uses the
/// `pipa_cal.scale` (m/s per count) for the per-axis division; the
/// misalignment matrix is assumed near-identity. The returned counts saturate
/// to `i16::{MIN,MAX}` on overflow (matches the AGC's hardware quantum
/// behaviour).
pub fn pipa_pulses_for_dv(dv_inertial: Vec3, pipa_cal: &PipaCalibration) -> [i16; 3] {
    let mut out = [0i16; 3];
    for (i, &dv) in dv_inertial.iter().enumerate() {
        let raw = (dv / pipa_cal.scale).round();
        let clamped = raw.clamp(i16::MIN as f64, i16::MAX as f64);
        out[i] = clamped as i16;
    }
    out
}

/// Great-circle range (km) between two (lat, lon) points on the spherical Earth.
///
/// Haversine formula, same convention as `programs::p61_p67::compute_range_to_go_km`.
pub fn haversine_km(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = lat2 - lat1;
    let dlon = lon2 - lon1;
    let sd_lat = (dlat * 0.5).sin();
    let sd_lon = (dlon * 0.5).sin();
    let a = sd_lat * sd_lat + lat1.cos() * lat2.cos() * sd_lon * sd_lon;
    let c = 2.0 * a.sqrt().atan2((1.0 - a).max(0.0).sqrt());
    R_EARTH * c / 1000.0
}

// ── tiny inline vec helpers — avoids a dep on glam for two test files ───────

#[inline]
fn vec3_add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
#[inline]
fn vec3_sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
#[inline]
fn vec3_scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
#[inline]
fn vec3_dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
fn vec3_norm(a: Vec3) -> f64 {
    vec3_dot(a, a).sqrt()
}
#[inline]
fn vec3_cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use agc_core::services::average_g::PipaCalibration;

    /// TC-ESIM-1: in vacuum (h > 250 km), sensed Δv is exactly zero.
    #[test]
    fn tc_esim_1_vacuum_no_sensed_dv() {
        let integ = EntryIntegrator::apollo_cm();
        let dv = integ.integrate_cycle(
            [R_EARTH + 300_000.0, 0.0, 0.0],
            [0.0, 7800.0, 0.0],
            0.30,
            0.0,
            2.0,
        );
        let mag = vec3_norm(dv);
        assert!(
            mag < 1e-6,
            "expected ~0 sensed Δv at h=300 km, got {mag} m/s"
        );
    }

    /// TC-ESIM-2: at peak entry density (~50 km altitude, V≈7800 m/s)
    /// sensed Δv is significant — drag deceleration of several g.
    #[test]
    fn tc_esim_2_peak_entry_decelerates() {
        let integ = EntryIntegrator::apollo_cm();
        // V along +Y, position on +X, descending at FPA = 0 for max drag.
        let dv = integ.integrate_cycle(
            [R_EARTH + 50_000.0, 0.0, 0.0],
            [0.0, 7800.0, 0.0],
            0.30,
            0.0,
            2.0,
        );
        // Drag is along -V (−Y). At 50 km without bank-management the CM
        // can hit several g of deceleration; over 2 s that's tens to a few
        // hundred m/s. Plausible band: 10–500 m/s.
        assert!(
            dv[1] < -1.0,
            "expected significant deceleration on +Y, got dv={dv:?}"
        );
        assert!(
            (10.0..500.0).contains(&(-dv[1])),
            "expected |dv| in 10..500 m/s range, got {dv:?}"
        );
    }

    /// TC-ESIM-3: bank=0 lift points away from Earth (+ radial).
    #[test]
    fn tc_esim_3_bank_zero_lift_radial() {
        let integ = EntryIntegrator::apollo_cm();
        let dv = integ.integrate_cycle(
            [R_EARTH + 60_000.0, 0.0, 0.0],
            [0.0, 7800.0, 0.0],
            0.30, // positive L/D
            0.0,  // bank = 0 → lift up
            2.0,
        );
        // The X component (radial) should be positive — lift pushes away
        // from Earth, partially offsetting the drag and gravity.
        assert!(
            dv[0] > 0.0,
            "expected positive radial component with bank=0 + L/D>0, got dv={dv:?}"
        );
    }

    /// TC-ESIM-4: PIPA quantisation round-trips through the nominal scale.
    #[test]
    fn tc_esim_4_pipa_quantisation() {
        let cal = PipaCalibration::NOMINAL;
        // 10 counts × 0.0585 m/s = 0.585 m/s exactly.
        let pulses = pipa_pulses_for_dv([0.585, 0.0, -1.17], &cal);
        assert_eq!(pulses, [10, 0, -20]);
    }

    /// TC-ESIM-5: haversine returns 0 for coincident points and ~111 km
    /// for 1° offsets on the equator.
    #[test]
    fn tc_esim_5_haversine() {
        assert!(haversine_km(0.0, 0.0, 0.0, 0.0).abs() < 1e-6);
        let d = haversine_km(0.0, 0.0, 0.0, 1.0_f64.to_radians());
        assert!(
            (110.0..112.0).contains(&d),
            "expected ~111 km for 1° on equator, got {d}"
        );
    }
}
