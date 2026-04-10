//! P01 — Pre-launch IMU Initialisation.
//! P02 — Gyrocompassing.
//!
//! Book-keeping programs executed on the launch pad before ignition. P01
//! cages the inertial platform; P02 subsequently runs the gyrocompass
//! loop that aligns the platform to local horizontal and the Earth
//! rotation vector.
//!
//! The real AGC P02 executes a multi-minute closed-loop gyrocompass
//! algorithm using the PIPA sense of `g` and the gyro torquing loop to
//! null misalignment. Phase 6 models this as an instantaneous state
//! transition because there is no HAL earth-rate source yet; the
//! lifecycle contract (Caged → CoarseAligned via P02) is what matters.
//!
//! AGC source: Comanche055/PRELAUNCH_INITIALIZATION.agc.

use crate::control::imu_control::ImuAlignmentState;
use crate::executive::job::JobPriority;

// ── Constants ─────────────────────────────────────────────────────────────────

pub const P01_MAJOR_MODE: u8 = 1;
pub const P02_MAJOR_MODE: u8 = 2;

/// Pre-launch job priority (both P01 and P02).
pub const PRIORITY: JobPriority = 3;

const VERB_DISPLAY: u8 = 6;
const NOUN_PRELAUNCH: u8 = 68;

/// Program alarm: P02 invoked from a non-Caged alignment state.
const ALARM_GYROCOMPASS_WRONG_STATE: u16 = 235;

// ── P01 ───────────────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[1]`.
pub fn init_p01(state: &mut crate::AgcState) -> JobPriority {
    state.major_mode = P01_MAJOR_MODE;
    state.dsky.prog = P01_MAJOR_MODE;
    state.dsky.verb = VERB_DISPLAY;
    state.dsky.noun = NOUN_PRELAUNCH;
    state.dsky.flashing = false;

    // Cage the platform regardless of prior state.
    state.imu_alignment_state = ImuAlignmentState::Caged;

    PRIORITY
}

// ── P02 ───────────────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[2]`.
pub fn init_p02(state: &mut crate::AgcState) -> JobPriority {
    if state.imu_alignment_state != ImuAlignmentState::Caged {
        state.alarm.code = ALARM_GYROCOMPASS_WRONG_STATE;
        state.alarm.lit = true;
        // soft alarm — continue so the crew can observe the transition
    }

    // Simulate a successful gyrocompass.
    state.imu_alignment_state = ImuAlignmentState::CoarseAligned;

    state.major_mode = P02_MAJOR_MODE;
    state.dsky.prog = P02_MAJOR_MODE;
    state.dsky.verb = VERB_DISPLAY;
    state.dsky.noun = NOUN_PRELAUNCH;
    state.dsky.flashing = false;

    PRIORITY
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    /// TC-P01-1: `init_p01` sets major_mode = 1 and cages the platform.
    #[test]
    fn tc_p01_1_sets_caged() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::FineAligned;

        let prio = init_p01(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(state.major_mode, P01_MAJOR_MODE);
        assert_eq!(state.dsky.prog, P01_MAJOR_MODE);
        assert_eq!(state.imu_alignment_state, ImuAlignmentState::Caged);
        assert_eq!(state.alarm.code, 0);
    }

    /// TC-P01-2: P01 forces Caged even from CoarseAligned.
    #[test]
    fn tc_p01_2_forces_cage_from_coarse() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::CoarseAligned;

        init_p01(&mut state);

        assert_eq!(state.imu_alignment_state, ImuAlignmentState::Caged);
    }

    /// TC-P02-1: `init_p02` from Caged transitions to CoarseAligned.
    #[test]
    fn tc_p02_1_from_caged() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::Caged;

        init_p02(&mut state);

        assert_eq!(state.major_mode, P02_MAJOR_MODE);
        assert_eq!(state.imu_alignment_state, ImuAlignmentState::CoarseAligned);
        assert_eq!(state.alarm.code, 0);
    }

    /// TC-P02-2: `init_p02` from FineAligned raises alarm 235 but still advances.
    #[test]
    fn tc_p02_2_from_fine_aligned_alarm() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::FineAligned;

        init_p02(&mut state);

        assert_eq!(state.alarm.code, ALARM_GYROCOMPASS_WRONG_STATE);
        assert!(state.alarm.lit);
        assert_eq!(state.major_mode, P02_MAJOR_MODE);
    }
}
