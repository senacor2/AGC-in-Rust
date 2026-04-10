//! P33 — Terminal Phase Initiation (TPI) targeting.
//!
//! Computes the TPI burn that carries the chaser (CSM) to the target's (LM)
//! position at TIG + dt_tpi using the Lambert solver. The burn is expressed as
//! a `Maneuver` in `state.pending_maneuver` with `mode = TargetingMode::TpiBurn`.
//!
//! Also stores `state.tpi_arrival_epoch` for subsequent P34 (TPM) use.
//!
//! Spec: specs/p33_p34-spec.md §1.2, §4.2, §5.2, §5.3
//! AGC source: Comanche055/P34,P35,P74,P75.agc (TPI entry and Lambert call)

use crate::executive::job::JobPriority;
use crate::guidance::targeting::{burn_attitude, lvlh_to_inertial, Maneuver, TargetingMode};
use crate::math::kepler::kepler_step;
use crate::math::lambert::lambert;
use crate::math::linalg::{cross, dot, mxv, norm, unit, vsub};
use crate::navigation::gravity::MU_EARTH;
use crate::types::{DeltaV, Met, Vec3};
use crate::AgcState;

// ── Constants ──────────────────────────────────────────────────────────────────

/// Major mode number for P33.
pub const P33_MAJOR_MODE: u8 = 33;

/// Job priority for P33. Same as P31/P32 (foreground targeting programs).
pub const P33_PRIORITY: JobPriority = 10;

/// Default TPI-to-intercept transfer time (seconds).
/// Apollo nominally used 10 minutes. The crew may supply a different value
/// via V06 N55 (see §6). This constant is used when no crew entry is made.
///
/// AGC erasable: DTTPI, scale B+17 s.
pub const TPI_DEFAULT_TRANSFER_TIME_S: f64 = 600.0;

/// Minimum chaser-target separation (m) below which the geometry is
/// considered collinear or degenerate for Lambert purposes.
/// If `norm(r_t_arrive - r_c_tig) < TPI_MIN_SEPARATION_M`, alarm 01443.
pub const TPI_MIN_SEPARATION_M: f64 = 1_000.0;

/// Minimum transfer time (s) accepted by P33/P34. Protects the Lambert
/// solver against zero or near-zero TOF inputs.
pub const TPI_MIN_TOF_S: f64 = 60.0;

/// Staleness limit for the target state (centiseconds).
/// If `(tig_cs - target_epoch_cs) > TPI_STALE_TARGET_CS`, alarm 01442 is
/// raised (non-fatal: computation proceeds but DSKY shows staleness warning).
/// 30 minutes = 180_000 cs.
pub const TPI_STALE_TARGET_CS: u64 = 180_000;

/// Desired elevation angle of target above chaser local horizontal at TIG
/// (radians). Apollo used 27.45° = 0.4793 rad (130 mils).
/// Used only for display — P33 does not iterate TIG to match this angle.
pub const TPI_NOMINAL_ELEVATION_RAD: f64 = 0.4793;

// ── Alarm codes ────────────────────────────────────────────────────────────────
//
// Collision analysis (grep ALARM_ across programs/):
//   p20.rs:  0o01421, 0o00404, 0o00405, 0o00400
//   p22.rs:  0o01420, 0o01421, 0o01422, 0o01424, 0o01425, 0o00400
//   p23.rs:  0o01420, 0o01421, 0o01426, 0o01427, 0o01430, 0o01431, 0o01432
//   p30.rs:  210 (decimal)
//   p31.rs:  0o01434, 0o01435
//   p32.rs:  0o01436, 0o01437
//
// P33/P34 use codes 0o01440–0o01445 (no collision).

/// Alarm 01440 (octal): target state is zero — P20 never ran or radar lost.
pub(crate) const ALARM_P33_NO_TARGET: u16 = 0o01440;

/// Alarm 01441 (octal): required input not available (pending_tig is None for
/// P33; tpi_arrival_epoch is None for P34).
pub(crate) const ALARM_P33_NO_TIG: u16 = 0o01441;

/// Alarm 01442 (octal): target state epoch is stale (non-fatal warning).
pub(crate) const ALARM_P33_STALE_TARGET: u16 = 0o01442;

/// Alarm 01443 (octal): degenerate geometry or invalid time interval.
pub(crate) const ALARM_P33_DEGENERATE: u16 = 0o01443;

/// Alarm 01444 (octal): Lambert non-convergence or anti-parallel vectors.
pub(crate) const ALARM_P33_LAMBERT: u16 = 0o01444;

// ── Result and error types ─────────────────────────────────────────────────────

/// Result of a successful Lambert intercept computation.
///
/// Spec: specs/p33_p34-spec.md §4.4
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InterceptResult {
    /// Delta-V in the LVLH frame at the burn TIG (m/s).
    /// [0] = R (radial), [1] = S (in-track), [2] = W (cross-track).
    pub dv_lvlh: Vec3,

    /// Delta-V magnitude (m/s). Equal to `norm(dv_inertial)`.
    pub dv_mag: f64,
}

/// Error conditions for a Lambert intercept computation.
///
/// Spec: specs/p33_p34-spec.md §4.4
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InterceptError {
    /// Chaser or target position vector is zero or below LEO altitude.
    DegenerateState,
    /// Transfer time is below `TPI_MIN_TOF_S` or non-positive.
    InvalidTimeInterval,
    /// Departure and arrival positions are within `TPI_MIN_SEPARATION_M`
    /// (collinear or same-point geometry; Lambert is undefined).
    DegenerateGeometry,
    /// r_c and r_t_arrive are anti-parallel (180° transfer with no
    /// defined plane; Lambert panics if called; rejected by pre-validation).
    AntiParallelVectors,
    /// Lambert solver did not converge (Halley iteration exhausted).
    /// This maps to the AGC PROGRAM ALARM displayed on the DSKY.
    LambertNotConverged,
}

// ── Entry point registered in PROGRAM_TABLE ────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[33]`.
/// Delegates to `p33_init` using `TPI_DEFAULT_TRANSFER_TIME_S`.
pub fn p33_init(state: &mut AgcState) -> JobPriority {
    p33_run(state, TPI_DEFAULT_TRANSFER_TIME_S)
}

// ── Core function ─────────────────────────────────────────────────────────────

/// Entry point for P33 (Terminal Phase Initiation).
/// Registered in PROGRAM_TABLE[33].
///
/// Sets `state.major_mode = 33`. Reads `state.vn.pending_tig` for the TPI
/// TIG. Calls `compute_lambert_intercept` and displays the result via V06 N81.
/// Stores the result in `state.pending_maneuver` with
/// `mode = TargetingMode::TpiBurn`. Also stores
/// `state.tpi_arrival_epoch = Some(arrival_epoch_s)` for subsequent P34 use.
///
/// # Preconditions
/// - `state.rendezvous_nav.target_pos` must be non-zero; otherwise alarm
///   01440.
/// - `state.vn.pending_tig` must be `Some(_)`; otherwise alarm 01441.
///
/// # Post-conditions (success)
/// - `state.major_mode == 33`
/// - `state.dsky.prog == 33`
/// - `state.pending_maneuver == Some(tpi_maneuver)` where
///   `tpi_maneuver.mode == TargetingMode::TpiBurn`
/// - `state.tpi_arrival_epoch == Some(arrival_s)` where
///   `arrival_s = tig_s + dt_tpi`
/// - DSKY displays ΔV via V06 N81 and elevation angle via V06 N55.
///
/// # Post-conditions (alarm)
/// - `state.pending_maneuver` is unchanged.
/// - `state.tpi_arrival_epoch` is unchanged.
///
/// # Alarms
/// - 01440: target state is zero.
/// - 01441: `pending_tig` is None (no TIG entered by crew).
/// - 01442: target state stale (non-fatal; proceeds but displays warning).
/// - 01443: degenerate geometry or invalid time interval.
/// - 01444: Lambert non-convergence or anti-parallel vectors.
///
/// Spec: specs/p33_p34-spec.md §4.2, §5.3
pub fn p33_run(state: &mut AgcState, dt_tpi: f64) -> JobPriority {
    // Step 0 — Guard checks.
    if norm(state.rendezvous_nav.target_pos) < 1.0 {
        state.alarm.code = ALARM_P33_NO_TARGET;
        state.alarm.lit = true;
        return P33_PRIORITY;
    }

    let tig_cs = match state.vn.pending_tig.take() {
        Some(t) => t,
        None => {
            state.alarm.code = ALARM_P33_NO_TIG;
            state.alarm.lit = true;
            return P33_PRIORITY;
        }
    };

    // Step 1 — Staleness check (non-fatal).
    let target_epoch_cs = (state.rendezvous_nav.target_epoch * 100.0) as u64;
    let tig_cs_u64 = tig_cs.0 as u64;
    if tig_cs_u64 > target_epoch_cs
        && (tig_cs_u64 - target_epoch_cs) > TPI_STALE_TARGET_CS
    {
        state.alarm.code = ALARM_P33_STALE_TARGET;
        state.alarm.lit = true;
        // Non-fatal: proceed with computation.
    }

    // Step 2 — Set major mode.
    state.major_mode = P33_MAJOR_MODE;
    state.dsky.prog = P33_MAJOR_MODE;
    state.dsky.verb = 6;
    state.dsky.noun = 37;
    state.dsky.flashing = true;

    // Step 3 — Propagate chaser to TIG.
    let tig_s = tig_cs.to_seconds();
    let chaser_epoch_s = state.csm_state.epoch.to_seconds();
    let dt_chaser = tig_s - chaser_epoch_s;
    if dt_chaser < 0.0 {
        state.alarm.code = ALARM_P33_DEGENERATE;
        state.alarm.lit = true;
        return P33_PRIORITY;
    }
    let (r_c_tig, v_c_tig) = kepler_step(
        state.csm_state.position,
        state.csm_state.velocity,
        dt_chaser,
        MU_EARTH,
    );

    // Step 4 — Compute elevation angle at TIG (display only).
    let target_epoch_s = state.rendezvous_nav.target_epoch;
    let dt_target_to_tig = tig_s - target_epoch_s;
    let (r_t_tig, _) = kepler_step(
        state.rendezvous_nav.target_pos,
        state.rendezvous_nav.target_vel,
        dt_target_to_tig,
        MU_EARTH,
    );
    let elev_rad = elevation_angle(r_c_tig, r_t_tig);

    // Step 5 — Transfer time: use the provided dt_tpi.

    // Step 6 — Propagate target to arrival epoch.
    let arrival_epoch_s = tig_s + dt_tpi;
    let dt_target_to_arrive = arrival_epoch_s - target_epoch_s;
    let (r_t_arrive, _) = kepler_step(
        state.rendezvous_nav.target_pos,
        state.rendezvous_nav.target_vel,
        dt_target_to_arrive,
        MU_EARTH,
    );

    // Step 7 — Compute Lambert intercept.
    match compute_lambert_intercept(r_c_tig, v_c_tig, r_t_arrive, dt_tpi, MU_EARTH) {
        Err(InterceptError::DegenerateState) => {
            state.alarm.code = ALARM_P33_NO_TARGET;
            state.alarm.lit = true;
            return P33_PRIORITY;
        }
        Err(InterceptError::InvalidTimeInterval) | Err(InterceptError::DegenerateGeometry) => {
            state.alarm.code = ALARM_P33_DEGENERATE;
            state.alarm.lit = true;
            return P33_PRIORITY;
        }
        Err(InterceptError::AntiParallelVectors) | Err(InterceptError::LambertNotConverged) => {
            state.alarm.code = ALARM_P33_LAMBERT;
            state.alarm.lit = true;
            return P33_PRIORITY;
        }
        Ok(res) => {
            // Step 8 — Build Maneuver.
            let dv_inertial = mxv(lvlh_to_inertial(r_c_tig, v_c_tig), res.dv_lvlh);
            let attitude = burn_attitude(dv_inertial, state.refsmmat);

            state.pending_maneuver = Some(Maneuver {
                tig: Met::from_seconds(tig_s),
                delta_v: DeltaV(dv_inertial),
                burn_attitude: attitude,
                mode: TargetingMode::TpiBurn,
            });

            // Step 9 — Store arrival epoch.
            state.tpi_arrival_epoch = Some(arrival_epoch_s);

            // Step 10 — DSKY display.
            // Show elevation angle and transfer time via V06 N55.
            state.dsky.verb = 6;
            state.dsky.noun = 55;
            state.dsky.r[0] = (elev_rad / 0.001) as f32; // mils
            state.dsky.r[1] = (dt_tpi / 60.0) as f32;    // minutes
            state.dsky.flashing = false;
            // Show ΔV via V06 N81.
            state.dsky.verb = 6;
            state.dsky.noun = 81;
            state.dsky.r[0] = res.dv_lvlh[0] as f32;
            state.dsky.r[1] = res.dv_lvlh[1] as f32;
            state.dsky.r[2] = res.dv_lvlh[2] as f32;
            state.dsky.flashing = false;
        }
    }

    P33_PRIORITY
}

// ── Pure-computation helpers ───────────────────────────────────────────────────

/// Compute a Lambert-based intercept burn given chaser and target state
/// at the burn epoch and a transfer time.
///
/// This is the pure-math core used by both P33 and P34. It does not touch
/// `AgcState` and is safe to call from tests without a full state.
///
/// # Arguments
/// - `r_c`: Chaser position at burn TIG (m, inertial ECI).
///   Caller must propagate with `kepler_step` from current epoch to TIG
///   before calling.
/// - `v_c`: Chaser velocity at burn TIG (m/s, inertial ECI).
/// - `r_t_arrive`: Target position at the intended arrival epoch (m,
///   inertial ECI). Caller must propagate the target state with `kepler_step`
///   to `tig_s + tof` before calling. For P34 this is
///   `tpi_arrival_epoch`, not `tig_s + new_tof`.
/// - `tof`: Transfer time from TIG to arrival (s). Must satisfy
///   `tof >= TPI_MIN_TOF_S`.
/// - `mu`: Gravitational parameter (m³/s²). Use `MU_EARTH`.
///
/// # Returns
/// `Ok(InterceptResult)` on success, `Err(InterceptError)` on failure.
///
/// # Algorithm
/// See specs/p33_p34-spec.md §5.2 for step-by-step detail.
pub fn compute_lambert_intercept(
    r_c: Vec3,
    v_c: Vec3,
    r_t_arrive: Vec3,
    tof: f64,
    mu: f64,
) -> Result<InterceptResult, InterceptError> {
    // Step 0 — Validate inputs.
    if norm(r_c) < 1.0e6 {
        return Err(InterceptError::DegenerateState);
    }
    if norm(r_t_arrive) < 1.0e6 {
        return Err(InterceptError::DegenerateState);
    }
    if tof < TPI_MIN_TOF_S {
        return Err(InterceptError::InvalidTimeInterval);
    }
    if mu <= 0.0 {
        return Err(InterceptError::DegenerateState);
    }

    validate_lambert_inputs(r_c, r_t_arrive, tof, mu)?;

    // Step 1 — Call Lambert solver.
    let (v_c_required, _v_arrive) = lambert(r_c, r_t_arrive, tof, mu, true);

    // Step 2 — Compute inertial ΔV.
    let dv_inertial = vsub(v_c_required, v_c);
    let dv_mag = norm(dv_inertial);

    // Step 3 — Convert to LVLH at TIG.
    let r_hat = unit(r_c);
    let h_vec = cross(r_c, v_c);
    let w_hat = unit(h_vec);
    let s_hat = cross(w_hat, r_hat);

    let dv_lvlh: Vec3 = [
        dot(dv_inertial, r_hat),
        dot(dv_inertial, s_hat),
        dot(dv_inertial, w_hat),
    ];

    // Step 4 — Return result.
    Ok(InterceptResult { dv_lvlh, dv_mag })
}

/// Validate inputs to `math::lambert::lambert` before calling the solver.
///
/// The Lambert solver (`math::lambert::lambert`) panics on degenerate inputs
/// (anti-parallel vectors, zero-length vectors, zero or negative TOF/mu).
/// Calling this function first converts those panic conditions into
/// structured `Err` variants, enabling graceful alarm handling.
///
/// # Checks performed
/// 1. `norm(r1) > 1e6 m` and `norm(r2) > 1e6 m`
/// 2. `tof > 0.0`
/// 3. `mu > 0.0`
/// 4. `norm(r2 - r1) > TPI_MIN_SEPARATION_M` (non-zero chord)
/// 5. Anti-parallel check: cross-product magnitude below threshold
///
/// # Returns
/// `Ok(())` if all preconditions are satisfied.
/// `Err(InterceptError)` if any precondition fails.
///
/// Spec: specs/p33_p34-spec.md §4.5
pub fn validate_lambert_inputs(
    r1: Vec3,
    r2: Vec3,
    tof: f64,
    mu: f64,
) -> Result<(), InterceptError> {
    if norm(r1) < 1.0e6 {
        return Err(InterceptError::DegenerateState);
    }
    if norm(r2) < 1.0e6 {
        return Err(InterceptError::DegenerateState);
    }
    if tof < TPI_MIN_TOF_S {
        return Err(InterceptError::InvalidTimeInterval);
    }
    if mu <= 0.0 {
        return Err(InterceptError::DegenerateState);
    }
    let sep = norm(vsub(r2, r1));
    if sep < TPI_MIN_SEPARATION_M {
        return Err(InterceptError::DegenerateGeometry);
    }
    // Anti-parallel check: cross-product magnitude < 1e-6 implies nearly collinear.
    let r1_hat = unit(r1);
    let r2_hat = unit(r2);
    let cross_mag = norm(cross(r1_hat, r2_hat));
    if cross_mag < 1.0e-6 {
        return Err(InterceptError::AntiParallelVectors);
    }

    Ok(())
}

/// Compute the elevation angle of the target above the chaser's local
/// horizontal at a given epoch.
///
/// The elevation angle E is the angle between the line-of-sight vector
/// (r_t - r_c) and the chaser's local horizontal plane.
///
/// E = asin( dot(unit(r_t - r_c), unit(r_c)) )
///
/// Positive E means the target is above the chaser's horizon.
///
/// # Arguments
/// - `r_c`: Chaser position (m, inertial).
/// - `r_t`: Target position at the same epoch (m, inertial).
///
/// # Returns
/// Elevation angle in radians, range [-π/2, +π/2].
/// Returns 0.0 if `norm(r_t - r_c) < 1.0` (same position; no defined LOS).
///
/// AGC display: shown via V06 N55 R1, in mils.
///
/// Spec: specs/p33_p34-spec.md §4.6, §5.3 step 4
pub(crate) fn elevation_angle(r_c: Vec3, r_t: Vec3) -> f64 {
    let los = vsub(r_t, r_c);
    if norm(los) < 1.0 {
        return 0.0;
    }
    let los_hat = unit(los);
    let r_c_hat = unit(r_c);
    libm::asin(dot(los_hat, r_c_hat).clamp(-1.0, 1.0))
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::kepler::kepler_step;
    use crate::math::lambert::lambert;
    use crate::navigation::gravity::MU_EARTH;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::AgcState;

    // Radius of a 400 km circular LEO.
    const R_LEO: f64 = 6_778_000.0; // m

    /// Build a minimal AgcState with chaser and target at the given epoch.
    fn make_state(
        r_c: Vec3,
        v_c: Vec3,
        r_t: Vec3,
        v_t: Vec3,
        epoch_s: f64,
    ) -> AgcState {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: r_c,
            velocity: v_c,
            epoch: Met::from_seconds(epoch_s),
            frame: Frame::EarthInertial,
        };
        state.rendezvous_nav.target_pos = r_t;
        state.rendezvous_nav.target_vel = v_t;
        state.rendezvous_nav.target_epoch = epoch_s;
        state.time = Met::from_seconds(epoch_s);
        state.vn.pending_tig = Some(Met::from_seconds(epoch_s));
        state
    }

    /// Circular LEO orbital speed.
    fn v_circ(r: f64) -> f64 {
        libm::sqrt(MU_EARTH / r)
    }

    // ── TC-P33-1: circular coplanar intercept analytical baseline ────────────
    //
    // Chaser on 400 km circular LEO, target 10 km in-track ahead, 10-minute
    // transfer. ΔV magnitude should be in [0.5, 20] m/s (broad tolerance
    // because exact value depends on Lambert convergence). Cross-track
    // component should be < 0.01 m/s (coplanar geometry).
    #[test]
    fn tc_p33_1_circular_coplanar_intercept() {
        let r_c: Vec3 = [R_LEO, 0.0, 0.0];
        let vc = v_circ(R_LEO);
        let v_c: Vec3 = [0.0, vc, 0.0];

        // Target 10 km in-track ahead — approximately [R_LEO, 10_000, 0] in
        // the in-plane direction of the circular orbit.
        let r_t: Vec3 = [R_LEO, 10_000.0, 0.0];
        let v_t: Vec3 = [0.0, vc, 0.0];

        let tof = 600.0; // 10 minutes

        // Propagate target to arrival epoch (TIG + tof).
        let (r_t_arrive, _) = kepler_step(r_t, v_t, tof, MU_EARTH);

        let result = compute_lambert_intercept(r_c, v_c, r_t_arrive, tof, MU_EARTH)
            .expect("TC-P33-1: compute_lambert_intercept should succeed");

        assert!(
            result.dv_mag >= 0.5 && result.dv_mag <= 20.0,
            "TC-P33-1: dv_mag = {} m/s, expected in [0.5, 20] m/s",
            result.dv_mag
        );
        // Coplanar: W-axis (cross-track) component must be negligible.
        assert!(
            result.dv_lvlh[2].abs() < 0.01,
            "TC-P33-1: cross-track dv = {} m/s, expected < 0.01 m/s (coplanar)",
            result.dv_lvlh[2]
        );
    }

    // ── TC-P33-2: zero ΔV case (chaser already on Lambert departure velocity) ─
    //
    // After getting the Lambert departure velocity for TC-P33-1's geometry,
    // set the chaser's velocity to that value. The resulting ΔV should be
    // essentially zero (< 1e-6 m/s).
    #[test]
    fn tc_p33_2_zero_dv_case() {
        let r_c: Vec3 = [R_LEO, 0.0, 0.0];
        let vc = v_circ(R_LEO);

        let r_t: Vec3 = [R_LEO, 10_000.0, 0.0];
        let v_t: Vec3 = [0.0, vc, 0.0];
        let tof = 600.0;

        // Propagate target to arrival epoch.
        let (r_t_arrive, _) = kepler_step(r_t, v_t, tof, MU_EARTH);

        // Get the Lambert departure velocity.
        let (v_required, _) = lambert(r_c, r_t_arrive, tof, MU_EARTH, true);

        // Set chaser velocity exactly to what Lambert requires — ΔV should vanish.
        let result = compute_lambert_intercept(r_c, v_required, r_t_arrive, tof, MU_EARTH)
            .expect("TC-P33-2: compute_lambert_intercept should succeed");

        assert!(
            result.dv_mag < 1.0e-6,
            "TC-P33-2: dv_mag = {} m/s, expected < 1e-6 m/s (zero ΔV case)",
            result.dv_mag
        );
    }

    // ── TC-P33-3: degenerate target (zero r_t_arrive) → Err(DegenerateState) ──
    #[test]
    fn tc_p33_3_zero_target_position() {
        let r_c: Vec3 = [R_LEO, 0.0, 0.0];
        let v_c: Vec3 = [0.0, v_circ(R_LEO), 0.0];
        let r_t_zero: Vec3 = [0.0, 0.0, 0.0]; // below 1e6 m threshold

        let result = compute_lambert_intercept(r_c, v_c, r_t_zero, 600.0, MU_EARTH);
        assert_eq!(
            result,
            Err(InterceptError::DegenerateState),
            "TC-P33-3: zero target should produce DegenerateState"
        );
    }

    // ── TC-P33-4: anti-parallel r1/r2 → Err(AntiParallelVectors) ────────────
    //
    // Place the arrival point exactly opposite the chaser (180° apart, same
    // orbital radius). validate_lambert_inputs should catch this.
    #[test]
    fn tc_p33_4_anti_parallel_vectors() {
        let r_c: Vec3 = [R_LEO, 0.0, 0.0];
        let v_c: Vec3 = [0.0, v_circ(R_LEO), 0.0];
        let r_t_anti: Vec3 = [-R_LEO, 0.0, 0.0]; // exactly opposite

        let result = compute_lambert_intercept(r_c, v_c, r_t_anti, 600.0, MU_EARTH);
        assert_eq!(
            result,
            Err(InterceptError::AntiParallelVectors),
            "TC-P33-4: anti-parallel vectors should produce AntiParallelVectors"
        );
    }

    // ── TC-P33-5: invalid TOF (< TPI_MIN_TOF_S) → Err(InvalidTimeInterval) ──
    #[test]
    fn tc_p33_5_tof_too_short() {
        let r_c: Vec3 = [R_LEO, 0.0, 0.0];
        let v_c: Vec3 = [0.0, v_circ(R_LEO), 0.0];
        let r_t: Vec3 = [R_LEO, 10_000.0, 0.0];

        let result = compute_lambert_intercept(r_c, v_c, r_t, TPI_MIN_TOF_S - 1.0, MU_EARTH);
        assert_eq!(
            result,
            Err(InterceptError::InvalidTimeInterval),
            "TC-P33-5: TOF below minimum should produce InvalidTimeInterval"
        );
    }

    // ── TC-P33-6: integration test through p33_run with zero target state ────
    //
    // When target_pos is the zero vector, p33_run must raise alarm 01440
    // (ALARM_P33_NO_TARGET), leave pending_maneuver unchanged, and leave
    // tpi_arrival_epoch as None.
    #[test]
    fn tc_p33_6_p33_run_zero_target_alarms() {
        let mut state = AgcState::new();
        // target_pos remains [0,0,0] (default).
        // Set a valid TIG so we reach the target-pos check.
        state.vn.pending_tig = Some(Met::from_seconds(1000.0));

        p33_run(&mut state, TPI_DEFAULT_TRANSFER_TIME_S);

        assert!(
            state.alarm.lit,
            "TC-P33-6: alarm must be lit for zero target"
        );
        assert_eq!(
            state.alarm.code, ALARM_P33_NO_TARGET,
            "TC-P33-6: alarm code should be ALARM_P33_NO_TARGET (0o01440)"
        );
        assert!(
            state.pending_maneuver.is_none(),
            "TC-P33-6: pending_maneuver must remain None on alarm"
        );
        assert!(
            state.tpi_arrival_epoch.is_none(),
            "TC-P33-6: tpi_arrival_epoch must remain None on alarm"
        );
    }

    // ── TC-P33-7: elevation_angle for coplanar geometry → 0.0 rad ─────────
    //
    // Geometry: chaser at [R_LEO, 0, 0], target at [R_LEO, 10_000, 0].
    //   los     = [0, 10_000, 0]   → los_hat = [0, 1, 0]
    //   r_c_hat = [1, 0, 0]
    //   sin(E)  = dot([0,1,0], [1,0,0]) = 0  → E = 0.0 rad
    #[test]
    fn tc_p33_7_elevation_angle_coplanar() {
        let r_c: Vec3 = [R_LEO, 0.0, 0.0];
        let r_t: Vec3 = [R_LEO, 10_000.0, 0.0];

        let elev = elevation_angle(r_c, r_t);
        assert!(
            libm::fabs(elev) < 1.0e-6,
            "TC-P33-7: elevation_angle = {} rad, expected ≈ 0.0 (target directly ahead)",
            elev
        );
    }
}
