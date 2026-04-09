//! AVERAGE G navigation cycle (SERVICER).
//!
//! Runs every 2 seconds, driven by T3RUPT via the Waitlist. Each cycle:
//!   1. Converts PIPA counts to body-frame delta-V.
//!   2. Rotates delta-V to ECI via REFSMMAT.
//!   3. Predicts new position using velocity + half-thrust + half-gravity from
//!      the **previous** cycle (Störmer-Verlet predictor step — CALCRVG).
//!   4. Evaluates fresh gravity at the predicted position (CALCGRAV).
//!   5. Corrects velocity: v_new = v + dv_eci + a_new * dt.
//!   6. Saves GDT/2 (half the new gravity impulse) for the next cycle.
//!   7. Advances the MET.
//!
//! The predictor-corrector structure matches the AGC CALCRVG/CALCGRAV sequence
//! in SERVICER207.agc. The first cycle is a warm-up: `AverageGState::new()`
//! initialises `prev_gdt_half` to zero, so position accuracy on cycle 1 is
//! first-order; accuracy is second-order from cycle 2 onward.
//!
//! AGC source: Comanche055/SERVICER207.agc — AVERAGE G / CALCRVG / CALCGRAV.

use crate::math::linalg::{add, scale};
use crate::navigation::gravity::{point_mass, MU_EARTH};
use crate::navigation::state_vector::{Refsmmat, StateVector};
use crate::types::{DeltaV, Vec3};

/// Duration of one SERVICER cycle in seconds.
pub const CYCLE_DT: f64 = 2.0;

/// PIPA scale factor: metres per second per count.
///
/// AGC source: SERVICER207.agc — KPIP1 constant: "1 PULSE = 5.85 CM/SEC".
/// 1 PIPA count = 0.0585 m/s in the stable-member frame.
pub const PIPA_SCALE: f64 = 0.0585;

/// Persistent state carried between AVERAGE G cycles.
///
/// The AGC CALCRVG predictor step uses the gravitational impulse `GDT/2`
/// (half of `a_grav * dt`) from the **previous** cycle to predict the new
/// position before evaluating gravity anew. This struct holds that value.
///
/// AGC source: SERVICER207.agc — `GDT/2` erasable variable.
#[derive(Clone, Copy, Debug)]
pub struct AverageGState {
    /// `a_grav * dt / 2` from the previous cycle (ECI, m/s).
    /// Initialised to zero; first cycle is a warm-up (first-order accuracy).
    pub prev_gdt_half: [f64; 3],
}

impl AverageGState {
    pub const fn new() -> Self {
        Self {
            prev_gdt_half: [0.0; 3],
        }
    }
}

impl Default for AverageGState {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of one AVERAGE G cycle.
#[derive(Clone, Copy, Debug)]
pub struct AverageGResult {
    /// Updated state vector.
    pub sv: StateVector,
    /// Accumulated delta-V this cycle in ECI m/s.
    pub delta_v_total: DeltaV,
}

/// Run one AVERAGE G cycle.
///
/// - `sv`: current state vector (ECI, m, m/s)
/// - `pipa_counts`: [x, y, z] PIPA counts from the IMU (stable-member frame, i16 each)
/// - `refsmmat`: current ECI-to-stable-member rotation matrix
/// - `dt`: cycle duration in seconds (normally `CYCLE_DT = 2.0`)
/// - `state`: mutable carry-over state (`prev_gdt_half` from previous cycle)
///
/// AGC source: Comanche055/SERVICER207.agc — AVERAGE G / CALCRVG / CALCGRAV.
pub fn average_g(
    sv: &StateVector,
    pipa_counts: [i16; 3],
    refsmmat: &Refsmmat,
    dt: f64,
    state: &mut AverageGState,
) -> AverageGResult {
    // Step 1: Convert PIPA counts to stable-member-frame delta-V.
    let dv_sm: Vec3 = [
        pipa_counts[0] as f64 * PIPA_SCALE,
        pipa_counts[1] as f64 * PIPA_SCALE,
        pipa_counts[2] as f64 * PIPA_SCALE,
    ];

    // Step 2: Rotate stable-member delta-V to ECI.
    let dv_eci = refsmmat.sm_to_eci(&dv_sm);

    // Step 3: Position predictor — CALCRVG.
    //
    // AGC source: SERVICER207.agc — CALCRVG:
    //   RN1 = RN + (VN + dv/2 + GDT/2_old) * 2sec
    //
    // `GDT/2_old` is half the gravitational impulse (a_grav * dt / 2) saved
    // from the **previous** cycle. Using old gravity for the position step and
    // fresh gravity (evaluated at RN1) for the velocity corrector is the
    // Störmer-Verlet (leapfrog) predictor-corrector scheme.
    let dv_half = scale(&dv_eci, 0.5);
    let v_pred = add(&add(&sv.v, &dv_half), &state.prev_gdt_half);
    let r_new = add(&sv.r, &scale(&v_pred, dt));

    // Step 4: Fresh gravity at predicted position — CALCGRAV.
    //
    // AGC source: SERVICER207.agc — CALCGRAV called on RN1.
    let a_new = point_mass(&r_new, MU_EARTH);

    // Step 5: Velocity corrector.
    //
    // AGC source: SERVICER207.agc — post-CALCGRAV:
    //   VN1 = VN + dv + GDT1   (three VAD: dv + 2 * GDT1/2)
    let v_new = add(&add(&sv.v, &dv_eci), &scale(&a_new, dt));

    // Step 6: Save GDT1/2 for the next cycle.
    //
    // AGC source: SERVICER207.agc — GDT/2 erasable store.
    state.prev_gdt_half = scale(&a_new, dt * 0.5);

    // Step 7: Advance MET by dt centiseconds (rounded to avoid truncation error
    // on sub-centisecond final steps from `propagate`).
    let dt_cs = libm::round(dt * 100.0) as u32;
    let t_new = sv.t.advance(dt_cs);

    AverageGResult {
        sv: StateVector {
            frame: sv.frame,
            r: r_new,
            v: v_new,
            t: t_new,
        },
        delta_v_total: DeltaV(dv_eci),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::linalg::norm;
    use crate::navigation::gravity::MU_EARTH;
    use crate::navigation::state_vector::Frame;
    use crate::types::Met;

    fn circular_orbit_sv() -> StateVector {
        let r0 = 6_578_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r0);
        StateVector {
            frame: Frame::Eci,
            r: [r0, 0.0, 0.0],
            v: [0.0, v_circ, 0.0],
            t: Met::ZERO,
        }
    }

    #[test]
    fn zero_pipa_advances_position() {
        let sv = circular_orbit_sv();
        let result = average_g(&sv, [0, 0, 0], &Refsmmat::IDENTITY, CYCLE_DT, &mut AverageGState::new());
        // Position must change.
        assert!(
            norm(&[
                result.sv.r[0] - sv.r[0],
                result.sv.r[1] - sv.r[1],
                result.sv.r[2] - sv.r[2],
            ]) > 1.0,
            "position should advance by more than 1 m"
        );
        // MET should advance by 200 centiseconds (2 s).
        assert_eq!(result.sv.t.0, 200);
    }

    #[test]
    fn nonzero_pipa_increases_velocity_in_expected_direction() {
        let sv = circular_orbit_sv();
        // Apply 100 counts in the +x stable-member direction (identity REFSMMAT → ECI +x).
        let result = average_g(&sv, [100, 0, 0], &Refsmmat::IDENTITY, CYCLE_DT, &mut AverageGState::new());
        let dv = result.delta_v_total.0;
        // delta-V should be in +x ECI direction.
        assert!(dv[0] > 0.0, "dv[0] = {}", dv[0]);
        assert_eq!(dv[1], 0.0);
        assert_eq!(dv[2], 0.0);
        // Magnitude check: 100 counts * 0.0585 m/s = 5.85 m/s.
        assert!((result.delta_v_total.magnitude() - 5.85).abs() < 1e-10);
    }

    #[test]
    fn pipa_rotated_by_refsmmat() {
        let sv = circular_orbit_sv();
        // REFSMMAT that maps ECI +y to SM +x (row 0 = [0,1,0]).
        // sm_to_eci is the transpose, so SM +x → ECI +y.
        // REFSMMAT (eci_to_sm): row0=[0,1,0], row1=[-1,0,0], row2=[0,0,1]
        // Transpose (sm_to_eci): col0=[0,-1,0] → SM[1,0,0] maps to ECI[0,1,0] ✓
        let refsmmat: [[f64; 3]; 3] = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let rsm = Refsmmat(refsmmat);
        // PIPA 100 counts in stable-member +x → should appear in ECI +y.
        let result = average_g(&sv, [100, 0, 0], &rsm, CYCLE_DT, &mut AverageGState::new());
        let dv = result.delta_v_total.0;
        assert!(dv[0].abs() < 1e-10, "dv[0] = {}", dv[0]);
        assert!((dv[1] - 5.85).abs() < 1e-10, "dv[1] = {}", dv[1]);
    }
}
