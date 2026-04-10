//! P51 — IMU Orientation Determination
//! P52 — IMU Realignment
//!
//! These two programs sequence the TRIAD REFSMMAT construction from
//! `control::imu_control` and transition the platform alignment state.
//!
//! They are thin layers over `refsmmat_from_star_sightings`; the interactive
//! optics MARK loop and the automatic CDU coarse-align drive are Milestone 5
//! items. Test harnesses call `pXX_mark_align` directly with pre-computed
//! inertial-frame and platform-frame star vectors.
//!
//! AGC source: Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc (P51, P52 entry),
//!             Comanche055/R51.agc, Comanche055/R52.agc.

use crate::control::imu_control::{refsmmat_from_star_sightings, ImuAlignmentState};
use crate::executive::job::JobPriority;
use crate::types::Vec3;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Major mode number for P51.
pub const P51_MAJOR_MODE: u8 = 51;

/// Major mode number for P52.
pub const P52_MAJOR_MODE: u8 = 52;

/// Background job priority for alignment programs.
pub const PRIORITY: JobPriority = 8;

/// DSKY verb for three-register display.
const VERB_DISPLAY: u8 = 6;

/// DSKY noun used as the star-code entry cue (N70).
const NOUN_STAR_CODE: u8 = 70;

/// DSKY noun used for the REFSMMAT-determined confirmation display (N93).
const NOUN_REFSMMAT_OK: u8 = 93;

// ── Program alarms ────────────────────────────────────────────────────────────

/// Program alarm: star vectors collinear → TRIAD cannot build a basis.
const ALARM_COLLINEAR_STARS: u16 = 220;

/// Program alarm: P52 invoked while the platform is still caged.
const ALARM_PLATFORM_CAGED: u16 = 221;

// ── Entry points registered in PROGRAM_TABLE ──────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[51]`.
pub fn init_p51(state: &mut crate::AgcState) -> JobPriority {
    p51_init(state)
}

/// Entry point registered in `PROGRAM_TABLE[52]`.
pub fn init_p52(state: &mut crate::AgcState) -> JobPriority {
    p52_init(state)
}

// ── P51 ───────────────────────────────────────────────────────────────────────

/// P51 — IMU Orientation Determination initialisation.
///
/// Called on V37E51E. Sets the DSKY to cue the crew for star selection
/// (flashing V06N70). Does not modify `state.refsmmat` or
/// `state.imu_alignment_state`.
pub fn p51_init(state: &mut crate::AgcState) -> JobPriority {
    state.major_mode = P51_MAJOR_MODE;
    state.dsky.prog = P51_MAJOR_MODE;
    state.dsky.verb = VERB_DISPLAY;
    state.dsky.noun = NOUN_STAR_CODE;
    state.dsky.flashing = true;
    PRIORITY
}

/// Complete a P51 alignment using two star sightings.
///
/// Computes a new REFSMMAT via the TRIAD method and commits it to
/// `state.refsmmat`. On success, transitions `imu_alignment_state` to
/// `CoarseAligned`. On collinear-star failure, raises program alarm 220
/// and leaves state unchanged.
///
/// `s*_inertial` are the catalogue-known inertial-frame unit vectors for
/// the two selected stars; `s*_platform` are the measured body/platform-frame
/// vectors as reported by the sextant.
pub fn p51_mark_align(
    state: &mut crate::AgcState,
    s1_inertial: Vec3,
    s2_inertial: Vec3,
    s1_platform: Vec3,
    s2_platform: Vec3,
) {
    match refsmmat_from_star_sightings(s1_inertial, s2_inertial, s1_platform, s2_platform) {
        Some(m) => {
            state.refsmmat = m;
            state.imu_alignment_state = ImuAlignmentState::CoarseAligned;
            state.dsky.flashing = false;
            state.dsky.verb = VERB_DISPLAY;
            state.dsky.noun = NOUN_REFSMMAT_OK;
            state.dsky.r[0] = 1.0;
        }
        None => {
            state.alarm.code = ALARM_COLLINEAR_STARS;
            state.alarm.lit = true;
        }
    }
}

// ── P52 ───────────────────────────────────────────────────────────────────────

/// P52 — IMU Realignment initialisation.
///
/// Must be called with `imu_alignment_state` at `CoarseAligned` or
/// `FineAligned`. If the platform is still `Caged`, raises program alarm
/// 221 and does not change the major mode.
pub fn p52_init(state: &mut crate::AgcState) -> JobPriority {
    if state.imu_alignment_state == ImuAlignmentState::Caged {
        state.alarm.code = ALARM_PLATFORM_CAGED;
        state.alarm.lit = true;
        return PRIORITY;
    }

    state.major_mode = P52_MAJOR_MODE;
    state.dsky.prog = P52_MAJOR_MODE;
    state.dsky.verb = VERB_DISPLAY;
    state.dsky.noun = NOUN_STAR_CODE;
    state.dsky.flashing = true;
    PRIORITY
}

/// Complete a P52 realignment using two star sightings.
///
/// Identical to `p51_mark_align` except that the success path transitions
/// `imu_alignment_state` to `FineAligned` rather than `CoarseAligned`.
pub fn p52_mark_align(
    state: &mut crate::AgcState,
    s1_inertial: Vec3,
    s2_inertial: Vec3,
    s1_platform: Vec3,
    s2_platform: Vec3,
) {
    match refsmmat_from_star_sightings(s1_inertial, s2_inertial, s1_platform, s2_platform) {
        Some(m) => {
            state.refsmmat = m;
            state.imu_alignment_state = ImuAlignmentState::FineAligned;
            state.dsky.flashing = false;
            state.dsky.verb = VERB_DISPLAY;
            state.dsky.noun = NOUN_REFSMMAT_OK;
            state.dsky.r[0] = 1.0;
        }
        None => {
            state.alarm.code = ALARM_COLLINEAR_STARS;
            state.alarm.lit = true;
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    const E1: Vec3 = [1.0, 0.0, 0.0];
    const E2: Vec3 = [0.0, 1.0, 0.0];

    /// TC-P51-1: `init_p51` sets major_mode = 51 and DSKY cue.
    #[test]
    fn tc_p51_1_init_sets_major_mode() {
        let mut state = AgcState::new();
        let prio = init_p51(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(state.major_mode, 51);
        assert_eq!(state.dsky.prog, 51);
        assert_eq!(state.dsky.verb, VERB_DISPLAY);
        assert_eq!(state.dsky.noun, NOUN_STAR_CODE);
        assert!(state.dsky.flashing, "P51 must flash for MARK acquisition");
    }

    /// TC-P51-2: `p51_mark_align` with identity star triads produces identity REFSMMAT
    /// and transitions Caged → CoarseAligned.
    #[test]
    fn tc_p51_2_identity_alignment_from_caged() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::Caged;

        p51_mark_align(&mut state, E1, E2, E1, E2);

        // REFSMMAT must be identity.
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (state.refsmmat[i][j] - expected).abs() < 1e-12,
                    "refsmmat[{i}][{j}] = {} (expected {expected})",
                    state.refsmmat[i][j]
                );
            }
        }
        assert_eq!(state.imu_alignment_state, ImuAlignmentState::CoarseAligned);
        assert!(!state.dsky.flashing, "flashing must clear after successful mark");
        assert_eq!(state.dsky.noun, NOUN_REFSMMAT_OK);
        assert_eq!(state.alarm.code, 0, "no alarm on success");
    }

    /// TC-P51-3: `p51_mark_align` with collinear stars raises alarm 220 and
    /// preserves REFSMMAT and alignment state.
    #[test]
    fn tc_p51_3_collinear_stars_alarm() {
        let mut state = AgcState::new();
        let prior_refsmmat = state.refsmmat; // default identity
        state.imu_alignment_state = ImuAlignmentState::Caged;

        p51_mark_align(&mut state, E1, E1, E1, E1);

        assert_eq!(state.alarm.code, ALARM_COLLINEAR_STARS);
        assert!(state.alarm.lit);
        assert_eq!(
            state.imu_alignment_state,
            ImuAlignmentState::Caged,
            "alignment state must not advance on collinear error"
        );
        assert_eq!(
            state.refsmmat, prior_refsmmat,
            "refsmmat must be preserved on collinear error"
        );
    }

    /// TC-P51-4: non-identity but orthogonal measurement produces an
    /// orthonormal REFSMMAT (R · Rᵀ = I).
    #[test]
    fn tc_p51_4_orthonormal_refsmmat() {
        let mut state = AgcState::new();
        // Inertial stars: +x, +y
        // Platform measurements: +y, -x  (a 90° rotation about +z)
        let s1_iner: Vec3 = [1.0, 0.0, 0.0];
        let s2_iner: Vec3 = [0.0, 1.0, 0.0];
        let s1_plat: Vec3 = [0.0, 1.0, 0.0];
        let s2_plat: Vec3 = [-1.0, 0.0, 0.0];

        p51_mark_align(&mut state, s1_iner, s2_iner, s1_plat, s2_plat);

        let r = state.refsmmat;
        // Orthonormality check R · Rᵀ = I.
        for i in 0..3 {
            for j in 0..3 {
                let mut dot = 0.0;
                for k in 0..3 {
                    dot += r[i][k] * r[j][k];
                }
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (dot - expected).abs() < 1e-12,
                    "orthonormality [{i}][{j}] = {dot}"
                );
            }
        }
        assert_eq!(state.imu_alignment_state, ImuAlignmentState::CoarseAligned);
    }

    /// TC-P52-1: `init_p52` from CoarseAligned sets major_mode = 52.
    #[test]
    fn tc_p52_1_init_from_coarse_aligned() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::CoarseAligned;

        let prio = init_p52(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(state.major_mode, 52);
        assert_eq!(state.dsky.prog, 52);
        assert_eq!(state.alarm.code, 0, "no alarm from a valid init_p52");
    }

    /// TC-P52-2: `init_p52` from Caged raises alarm 221 and does not advance
    /// the major mode.
    #[test]
    fn tc_p52_2_init_from_caged_alarms() {
        let mut state = AgcState::new();
        state.major_mode = 0;
        state.imu_alignment_state = ImuAlignmentState::Caged;

        init_p52(&mut state);

        assert_eq!(state.alarm.code, ALARM_PLATFORM_CAGED);
        assert!(state.alarm.lit);
        assert_ne!(
            state.major_mode, 52,
            "major_mode must not advance when P52 is rejected"
        );
    }

    /// TC-P52-3: `p52_mark_align` transitions CoarseAligned → FineAligned.
    #[test]
    fn tc_p52_3_coarse_to_fine_transition() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::CoarseAligned;

        p52_mark_align(&mut state, E1, E2, E1, E2);

        assert_eq!(state.imu_alignment_state, ImuAlignmentState::FineAligned);
        assert_eq!(state.alarm.code, 0);
    }

    /// TC-P52-4: `p52_mark_align` with collinear stars preserves prior REFSMMAT
    /// and prior alignment state.
    #[test]
    fn tc_p52_4_collinear_preserves_refsmmat() {
        let mut state = AgcState::new();
        // Seed a non-identity REFSMMAT.
        let prior = [
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        state.refsmmat = prior;
        state.imu_alignment_state = ImuAlignmentState::FineAligned;

        p52_mark_align(&mut state, E1, E1, E1, E1);

        assert_eq!(state.alarm.code, ALARM_COLLINEAR_STARS);
        assert_eq!(state.refsmmat, prior, "refsmmat must survive collinear error");
        assert_eq!(
            state.imu_alignment_state,
            ImuAlignmentState::FineAligned,
            "alignment state must survive collinear error"
        );
    }
}
