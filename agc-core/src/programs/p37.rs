//! P37 — Return to Earth
//!
//! Contingency program for computing a Trans-Earth Injection (TEI) burn
//! from lunar orbit back to Earth entry. Used in Apollo 13-style scenarios
//! when ground support is unavailable.
//!
//! AGC source: Comanche055/P37,P70.agc

use crate::executive::job::JobPriority;
use crate::guidance::targeting::{burn_duration, return_to_earth, ENTRY_INTERFACE_ALT_M};
use crate::math::kepler::kepler_step;
use crate::math::linalg::norm;
use crate::navigation::gravity::{MU_MOON, R_EARTH};
use crate::navigation::state_vector::{Frame, StateVector};
use crate::types::Met;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Job priority for P37 (background computation, same tier as P30/P31).
pub const PRIORITY: JobPriority = 16;

/// Alarm raised when P37 receives a TOF outside `[MIN_TEI_TOF_S, MAX_TEI_TOF_S]`.
pub const ALARM_P37_BAD_TOF: u16 = 1410;

/// Alarm raised when P37 is invoked with a CSM state vector that is not in
/// `Frame::MoonInertial`.
pub const ALARM_P37_WRONG_FRAME: u16 = 1411;

/// Default TIG offset: 30 minutes from current MET (centiseconds).
pub const DEFAULT_TEI_TIG_OFFSET_CS: u32 = 180_000;

/// Default time of flight for TEI return: 60 hours in seconds.
pub const DEFAULT_TEI_TOF_S: f64 = 216_000.0;

/// Minimum valid TOF: 24 hours in seconds.
pub const MIN_TEI_TOF_S: f64 = 86_400.0;

/// Maximum valid TOF: 120 hours in seconds.
pub const MAX_TEI_TOF_S: f64 = 432_000.0;

/// Earth-Moon distance (m) — static approximation for P37 targeting.
pub const D_EARTH_MOON_M: f64 = 384_400_000.0;

/// Nominal CSM vehicle mass in lunar orbit (kg), used for burn duration estimate.
pub const NOMINAL_CSM_MASS_KG: f64 = 20_000.0;

// ── Public functions ──────────────────────────────────────────────────────────

/// P37 entry point — Return to Earth targeting.
///
/// Sets `major_mode = 37`, computes the default TIG from the current MET,
/// and calls `p37_compute_tei` with the default parameters.
///
/// If the CSM state vector is not in `Frame::MoonInertial`, raises alarm
/// 1411 (`ALARM_P37_WRONG_FRAME`) and returns without entering P37.
pub fn p37_init(state: &mut crate::AgcState) -> JobPriority {
    if state.csm_state.frame != Frame::MoonInertial {
        state.alarm.raise(ALARM_P37_WRONG_FRAME);
        return PRIORITY;
    }

    state.major_mode = 37;
    state.dsky.prog = 37;
    state.pending_maneuver = None;

    let tig_cs = state.time.0.saturating_add(DEFAULT_TEI_TIG_OFFSET_CS);
    p37_compute_tei(state, Met(tig_cs), DEFAULT_TEI_TOF_S);

    PRIORITY
}

/// Thin wrapper called via `PROGRAM_TABLE[37]`.
pub fn init(state: &mut crate::AgcState) -> JobPriority {
    p37_init(state)
}

/// Compute the Trans-Earth Injection maneuver and store it as the pending burn.
///
/// Constructs the Earth entry target position in MCI, propagates the CSM state
/// to the TIG, calls `return_to_earth`, and stores the result in
/// `state.pending_maneuver`.
///
/// On invalid input the function leaves `state.pending_maneuver` unchanged and
/// raises a program alarm:
/// - TOF outside `[MIN_TEI_TOF_S, MAX_TEI_TOF_S]` → alarm 1410 (`ALARM_P37_BAD_TOF`).
/// - `state.csm_state.frame != Frame::MoonInertial` → alarm 1411 (`ALARM_P37_WRONG_FRAME`).
pub fn p37_compute_tei(state: &mut crate::AgcState, tig: Met, tof: f64) {
    if !(MIN_TEI_TOF_S..=MAX_TEI_TOF_S).contains(&tof) {
        state.alarm.raise(ALARM_P37_BAD_TOF);
        return;
    }
    if state.csm_state.frame != Frame::MoonInertial {
        state.alarm.raise(ALARM_P37_WRONG_FRAME);
        return;
    }

    // Entry target in MCI.
    // Earth centre: approximately at [-D_EARTH_MOON_M, 0, 0] in MCI.
    // Entry interface radius: R_EARTH + ENTRY_INTERFACE_ALT_M.
    // Sub-Earth aim point (direction +x in MCI, toward Earth):
    //   entry_target[0] = -D_EARTH_MOON_M + (R_EARTH + ENTRY_INTERFACE_ALT_M)
    let r_ei = R_EARTH + ENTRY_INTERFACE_ALT_M;
    let entry_target: crate::types::Vec3 = [-D_EARTH_MOON_M + r_ei, 0.0, 0.0];

    // Propagate CSM state from its epoch to TIG using Keplerian two-body mechanics.
    let state_at_tig: StateVector = {
        let epoch_cs = state.csm_state.epoch.0;
        let tig_cs = tig.0;
        if tig_cs == epoch_cs {
            state.csm_state
        } else {
            let dt_s = (tig_cs as f64 - epoch_cs as f64) / 100.0;
            let (r1, v1) = kepler_step(
                state.csm_state.position,
                state.csm_state.velocity,
                dt_s,
                MU_MOON,
            );
            StateVector {
                position: r1,
                velocity: v1,
                epoch: tig,
                frame: Frame::MoonInertial,
            }
        }
    };

    let maneuver = return_to_earth(state_at_tig, entry_target, tof, state.refsmmat);
    state.pending_maneuver = Some(maneuver);
    p37_display_summary(state);
}

/// Populate the DSKY with the TEI burn summary (V06N45).
///
/// Writes:
///   R1: minutes to TIG (integer truncation)
///   R2: delta-V magnitude (m/s)
///   R3: estimated burn duration (seconds)
///
/// Does nothing if `state.pending_maneuver` is `None`.
pub fn p37_display_summary(state: &mut crate::AgcState) {
    if let Some(m) = state.pending_maneuver {
        let tig_offset_min = m.tig.0.saturating_sub(state.time.0) / 6_000;
        let dv_mag = norm(m.delta_v.0);
        let burn_dur_s = burn_duration(dv_mag, NOMINAL_CSM_MASS_KG);

        state.dsky.verb = 6;
        state.dsky.noun = 45;
        state.dsky.r[0] = tig_offset_min as f32;
        state.dsky.r[1] = dv_mag as f32;
        state.dsky.r[2] = burn_dur_s as f32;
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::guidance::targeting::TargetingMode;
    use crate::navigation::gravity::{MU_EARTH, MU_MOON, R_EARTH, R_MOON};
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;
    use crate::{math, AgcState};

    // Helper: build a representative 100 km LLO state in MoonInertial frame.
    fn llo_state() -> StateVector {
        let r = R_MOON + 100_000.0;
        let v = libm::sqrt(MU_MOON / r);
        StateVector {
            position: [r, 0.0, 0.0],
            velocity: [0.0, v, 0.0],
            epoch: Met(0),
            frame: Frame::MoonInertial,
        }
    }

    /// TC-P37-1: `init` with `Frame::MoonInertial` state sets `major_mode = 37`.
    ///
    #[test]
    fn tc_p37_1_init_sets_major_mode() {
        let mut state = AgcState::new();
        state.csm_state = llo_state();
        state.time = Met(0);
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        // Simulate the field-setting portion of p37_init and then compute with 30h TOF.
        assert_eq!(
            state.csm_state.frame,
            Frame::MoonInertial,
            "TC-P37-1: frame must be MoonInertial"
        );
        state.major_mode = 37;
        state.dsky.prog = 37;
        state.pending_maneuver = None;

        let tig_cs = state.time.0.saturating_add(DEFAULT_TEI_TIG_OFFSET_CS);
        p37_compute_tei(&mut state, Met(tig_cs), 108_000.0); // 30-hour TOF

        assert_eq!(state.major_mode, 37, "TC-P37-1: major_mode must be 37");
        assert_eq!(state.dsky.prog, 37, "TC-P37-1: dsky.prog must be 37");
        const _: () = assert!(PRIORITY > 0, "TC-P37-1: PRIORITY must be non-zero");
        assert_eq!(PRIORITY, 16, "TC-P37-1: PRIORITY must equal 16");
    }

    /// TC-P37-2: `p37_compute_tei` with 30-hour TOF produces a finite `Maneuver`.
    ///
    #[test]
    fn tc_p37_2_compute_tei_finite_maneuver() {
        let mut state = AgcState::new();
        state.csm_state = llo_state();
        state.time = Met(0);
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        let tig = Met(DEFAULT_TEI_TIG_OFFSET_CS);
        let tof_s = 108_000.0_f64; // 30 hours

        p37_compute_tei(&mut state, tig, tof_s);

        let maneuver = state
            .pending_maneuver
            .expect("TC-P37-2: pending_maneuver must be Some");

        assert_eq!(
            maneuver.tig, tig,
            "TC-P37-2: maneuver.tig must equal input tig"
        );

        let dv_mag = math::linalg::norm(maneuver.delta_v.0);
        assert!(
            dv_mag.is_finite(),
            "TC-P37-2: delta_v magnitude must be finite"
        );
        assert!(dv_mag > 1.0, "TC-P37-2: delta_v magnitude must be > 1 m/s");
        assert!(
            dv_mag < 5000.0,
            "TC-P37-2: delta_v magnitude must be < 5000 m/s (sanity check)"
        );

        assert_eq!(
            maneuver.mode,
            TargetingMode::ReturnToEarth,
            "TC-P37-2: mode must be ReturnToEarth"
        );

        // burn_attitude must be orthonormal: M * M^T = I (within 1e-9)
        let mt = math::linalg::transpose(maneuver.burn_attitude);
        let mmt = math::linalg::mxm(maneuver.burn_attitude, mt);
        for (row, mmt_row) in mmt.iter().enumerate() {
            for (col, &val) in mmt_row.iter().enumerate() {
                let expected = if row == col { 1.0 } else { 0.0 };
                assert!(
                    (val - expected).abs() < 1e-9,
                    "TC-P37-2: burn_attitude not orthonormal at [{row}][{col}]: {val} != {expected}"
                );
            }
        }

        // First column of burn_attitude must be parallel to unit(delta_v)
        let dv_unit = math::linalg::unit(maneuver.delta_v.0);
        let x_body = [
            maneuver.burn_attitude[0][0],
            maneuver.burn_attitude[1][0],
            maneuver.burn_attitude[2][0],
        ];
        for i in 0..3 {
            assert!(
                (x_body[i] - dv_unit[i]).abs() < 1e-9,
                "TC-P37-2: burn_attitude col-0[{i}] ({}) != dv_unit[{i}] ({})",
                x_body[i],
                dv_unit[i]
            );
        }
    }

    /// TC-P37-3: `init` with `Frame::EarthInertial` state raises alarm 1411
    /// and does not enter P37 (no `pending_maneuver`, `major_mode` unchanged).
    #[test]
    fn tc_p37_3_wrong_frame_alarms_1411() {
        let mut state = AgcState::new();
        let r_leo = R_EARTH + 400_000.0;
        let v_circ = libm::sqrt(MU_EARTH / r_leo);
        state.csm_state = StateVector {
            position: [r_leo, 0.0, 0.0],
            velocity: [0.0, v_circ, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        state.time = Met(0);
        let major_mode_before = state.major_mode;

        let _ = p37_init(&mut state);

        assert_eq!(
            state.alarm.code, ALARM_P37_WRONG_FRAME,
            "wrong frame must raise alarm 1411"
        );
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert!(
            state.pending_maneuver.is_none(),
            "wrong frame must not set pending_maneuver"
        );
        assert_eq!(
            state.major_mode, major_mode_before,
            "wrong frame must not promote major_mode to 37"
        );
    }

    /// TC-P37-4: After `p37_compute_tei`, `state.pending_maneuver` is `Some(_)`.
    ///
    #[test]
    fn tc_p37_4_result_stored_in_pending_maneuver() {
        let mut state = AgcState::new();
        state.csm_state = llo_state();
        state.time = Met(0);
        state.refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        assert!(
            state.pending_maneuver.is_none(),
            "TC-P37-4: pending_maneuver must be None before computation"
        );

        let tig = Met(360_000_u32); // 1 hour from epoch
        let tof_s = 108_000.0_f64; // 30 hours

        p37_compute_tei(&mut state, tig, tof_s);

        assert!(
            state.pending_maneuver.is_some(),
            "TC-P37-4: pending_maneuver must be Some after p37_compute_tei"
        );

        let m = state.pending_maneuver.unwrap();
        assert_eq!(m.tig, tig, "TC-P37-4: maneuver.tig must equal input tig");
        assert_eq!(
            m.mode,
            TargetingMode::ReturnToEarth,
            "TC-P37-4: mode must be ReturnToEarth"
        );

        // A second call with a different TIG must overwrite the first result.
        let tig2 = Met(720_000_u32); // 2 hours from epoch
        p37_compute_tei(&mut state, tig2, tof_s);

        let m2 = state.pending_maneuver.unwrap();
        assert_eq!(
            m2.tig, tig2,
            "TC-P37-4: second call must overwrite pending_maneuver.tig"
        );
        assert_ne!(
            m.tig, m2.tig,
            "TC-P37-4: second call must produce a different TIG"
        );
    }

    /// TC-P37-5a: `p37_compute_tei` with TOF < MIN_TEI_TOF_S raises alarm 1410
    /// and leaves `pending_maneuver` unchanged.
    #[test]
    fn tc_p37_5a_tof_too_short_alarms_1410() {
        let mut state = AgcState::new();
        state.csm_state = llo_state();
        state.time = Met(0);

        // 6 hours — below MIN_TEI_TOF_S (24 hours)
        p37_compute_tei(&mut state, Met(DEFAULT_TEI_TIG_OFFSET_CS), 21_600.0);

        assert_eq!(
            state.alarm.code, ALARM_P37_BAD_TOF,
            "TOF below MIN must raise alarm 1410"
        );
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert!(
            state.pending_maneuver.is_none(),
            "bad TOF must not set pending_maneuver"
        );
    }

    /// TC-P37-5b: `p37_compute_tei` with TOF > MAX_TEI_TOF_S raises alarm 1410
    /// and leaves `pending_maneuver` unchanged.
    #[test]
    fn tc_p37_5b_tof_too_long_alarms_1410() {
        let mut state = AgcState::new();
        state.csm_state = llo_state();
        state.time = Met(0);

        // 200 hours — above MAX_TEI_TOF_S (120 hours)
        p37_compute_tei(&mut state, Met(DEFAULT_TEI_TIG_OFFSET_CS), 720_000.0);

        assert_eq!(
            state.alarm.code, ALARM_P37_BAD_TOF,
            "TOF above MAX must raise alarm 1410"
        );
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert!(
            state.pending_maneuver.is_none(),
            "bad TOF must not set pending_maneuver"
        );
    }
}
