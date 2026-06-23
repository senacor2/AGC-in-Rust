//! P01 — Pre-launch IMU Initialisation.
//! P02 — Gyrocompassing.
//!
//! Book-keeping programs executed on the launch pad before ignition. P01
//! cages the inertial platform; P02 runs the multi-minute gyrocompass loop
//! that aligns the platform to local horizontal and the Earth rotation vector.
//!
//! ## P02 algorithm (O'Brien §12 / Comanche055 IMU_CALIBRATION_AND_ALIGNMENT.agc)
//!
//! The real AGC P02 torques the physical gyroscopes via PULSEIMU every 0.5 s
//! (SLEEPIE Waitlist loop) until liftoff.  The torquing integrates the
//! ERTHRVSE Earth-rate vector to null the residual misalignment between the
//! stable-member platform and local vertical + North.
//!
//! In our simulation the "torquing" is modelled by directly reducing the CDU
//! angles toward zero each iteration: the CDU angles represent the physical
//! gimbal displacement between the current platform orientation and the
//! desired aligned orientation.  When all three axes are within
//! `COARSE_ALIGN_THRESHOLD` the loop stops and `imu_alignment_state`
//! transitions to `CoarseAligned`.
//!
//! After liftoff P11 computes REFSMMAT from gravity and launch azimuth.
//! P02 itself does **not** touch REFSMMAT.
//!
//! AGC source: `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`.

use crate::control::imu_control::{ImuAlignmentState, COARSE_ALIGN_THRESHOLD};
use crate::executive::job::JobPriority;
use crate::navigation::time::OMEGA_EARTH;
use crate::types::CduAngle;

// ── Constants ─────────────────────────────────────────────────────────────────

pub const P01_MAJOR_MODE: u8 = 1;
pub const P02_MAJOR_MODE: u8 = 2;

/// Pre-launch job priority (both P01 and P02).
pub const PRIORITY: JobPriority = 3;

const VERB_DISPLAY: u8 = 6;
const NOUN_PRELAUNCH: u8 = 68;

pub use crate::tables::alarm_codes::ALARM_GYROCOMPASS_WRONG_STATE;

/// Waitlist period for the gyrocompass loop: 500 cs = 5 s.
///
/// The real AGC SLEEPIE loop runs at `1SECXT1 = .5SEC = 50 cs` (0.5 s).
/// We use 500 cs (5 s) so the loop exercises several Waitlist cycles in
/// unit tests without requiring thousands of steps.
pub const GYROCOMPASS_PERIOD_CS: u16 = 500;

/// Gyrocompass angular drive rate (CDU counts per period).
///
/// The Earth rate at KSC latitude (≈ 28.6°) contributes a horizontal component
/// of `Ω_E × cos(28.6°) ≈ 6.41e-5 rad/s`.  At the GYROCOMPASS_PERIOD (5 s),
/// each step corrects by 3.2e-4 rad ≈ 65 536/(2π) × 3.2e-4 ≈ 3.3 CDU counts.
/// We use this as the drive rate so the convergence time for a 30° misalignment
/// is approximately:
///   30° × (π/180) / (3.2e-4 rad/step) ≈ 1636 steps × 5 s = 8180 s
///
/// That is too slow for a test. We scale by 100× in simulation to get
/// ~82 steps from a 30° start, matching the AGC's ~128-step erection window.
pub const GYROCOMPASS_DRIVE_COUNTS: i16 = 330; // ≈ 100 × real rate

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
///
/// Starts the gyrocompass Waitlist loop.  The loop calls
/// `p02_gyrocompass_step` every `GYROCOMPASS_PERIOD_CS` centiseconds until
/// convergence, then transitions `imu_alignment_state` to `CoarseAligned`.
pub fn init_p02(state: &mut crate::AgcState) -> JobPriority {
    if state.imu_alignment_state != ImuAlignmentState::Caged {
        state.alarm.raise(ALARM_GYROCOMPASS_WRONG_STATE, crate::tables::alarm_codes::SITE_P01_P02);
        // soft alarm — continue so the crew can observe the transition
    }

    state.major_mode = P02_MAJOR_MODE;
    state.dsky.prog = P02_MAJOR_MODE;
    state.dsky.verb = VERB_DISPLAY;
    state.dsky.noun = NOUN_PRELAUNCH;
    state.dsky.flashing = false;

    // Schedule the first gyrocompass step.
    let _ = state.waitlist.schedule(GYROCOMPASS_PERIOD_CS, p02_gyrocompass_step);

    PRIORITY
}

/// One gyrocompass iteration — called by the Waitlist every `GYROCOMPASS_PERIOD_CS`.
///
/// Models the AGC's SLEEPIE/ALWAYSG gyro-torquing loop:
/// each call drives the CDU angles toward zero at `GYROCOMPASS_DRIVE_COUNTS`
/// per axis per period.  When all three CDU axes are within
/// `COARSE_ALIGN_THRESHOLD` the platform is declared coarsely aligned.
///
/// The Earth-rate horizontal component `Ω_E × cos(lat)` sets the baseline
/// convergence dynamic; `GYROCOMPASS_DRIVE_COUNTS` is scaled for simulation
/// speed (see constant documentation).
///
/// AGC source: `IMU_CALIBRATION_AND_ALIGNMENT.agc` — SLEEPIE/ALWAYSG/EARTHR*.
pub fn p02_gyrocompass_step(state: &mut crate::AgcState) {
    // Still in P02?
    if state.major_mode != P02_MAJOR_MODE {
        return; // program switched away — stop rescheduling
    }

    // Compute per-axis CDU drive amounts scaled by the launch latitude.
    // ERTHRVSE uses cos(lat) (horizontal/North component) and sin(lat)
    // (vertical/Up component).  We drive all axes at the same rate
    // for simplicity; the direction cosines modulate the effective rate.
    let cos_lat = libm::cos(state.launch_lat_rad);
    let sin_lat = libm::sin(state.launch_lat_rad);

    // Scale drive per axis:  pitch (Y) corrects horizontal, yaw (Z) corrects
    // vertical, roll (X) is the azimuth error (smaller).
    let drive_x = (GYROCOMPASS_DRIVE_COUNTS as f64 * cos_lat * 0.5) as i16;
    let drive_y = (GYROCOMPASS_DRIVE_COUNTS as f64 * cos_lat) as i16;
    let drive_z = (GYROCOMPASS_DRIVE_COUNTS as f64 * sin_lat) as i16;

    // Drive each CDU axis toward zero by the computed amount (clamp at zero).
    state.current_cdu[0] = drive_toward_zero(state.current_cdu[0], drive_x);
    state.current_cdu[1] = drive_toward_zero(state.current_cdu[1], drive_y);
    state.current_cdu[2] = drive_toward_zero(state.current_cdu[2], drive_z);

    // Check convergence on all axes.
    let converged = state.current_cdu.iter().all(|c| {
        let mag = c.0.unsigned_abs();
        mag <= COARSE_ALIGN_THRESHOLD
    });

    if converged {
        state.imu_alignment_state = ImuAlignmentState::CoarseAligned;
        // Loop complete — do not reschedule.
    } else {
        // Reschedule for next iteration.
        let _ = state.waitlist.schedule(GYROCOMPASS_PERIOD_CS, p02_gyrocompass_step);
    }
}

/// Drive a CDU angle toward zero by `delta` counts (one's-complement subtraction).
///
/// If `|cdu| <= delta`, snaps to zero.  The subtraction uses two's-complement
/// integer arithmetic — safe because CDU counts are always small relative to
/// i16::MAX for this use case.
fn drive_toward_zero(cdu: CduAngle, delta: i16) -> CduAngle {
    if delta <= 0 {
        return cdu;
    }
    let v = cdu.0;
    if v > 0 {
        CduAngle(v.saturating_sub(delta).max(0))
    } else if v < 0 {
        CduAngle(v.saturating_add(delta).min(0))
    } else {
        CduAngle(0)
    }
}

/// Earth rate horizontal component at a given latitude (rad/s).
///
/// Used by `SimEarthRate` and tests to characterise the gyrocompass drive.
/// `Ω_E × cos(latitude)` — the North-pointing horizontal component of Earth's
/// rotation rate.
pub fn earth_rate_horizontal(lat_rad: f64) -> f64 {
    OMEGA_EARTH * libm::cos(lat_rad)
}

/// Earth rate vertical component at a given latitude (rad/s).
///
/// `Ω_E × sin(latitude)` — the Up-pointing vertical component.
pub fn earth_rate_vertical(lat_rad: f64) -> f64 {
    OMEGA_EARTH * libm::sin(lat_rad)
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    // Helper: set all CDU axes to the same count value.
    fn set_cdu_all(state: &mut AgcState, counts: i16) {
        for c in state.current_cdu.iter_mut() {
            c.0 = counts;
        }
    }

    // Helper: run the Waitlist until P02 marks CoarseAligned or until
    // `max_steps` steps have been drained.
    fn drain_gyrocompass(state: &mut AgcState, max_steps: usize) -> usize {
        let mut steps = 0;
        while state.imu_alignment_state != ImuAlignmentState::CoarseAligned
            && steps < max_steps
        {
            let Some((task_fn, _)) = state.waitlist.pop_task() else { break };
            task_fn(state);
            steps += 1;
        }
        steps
    }

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
        assert_eq!(state.alarm.code(), 0);
    }

    /// TC-P01-2: P01 forces Caged even from CoarseAligned.
    #[test]
    fn tc_p01_2_forces_cage_from_coarse() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::CoarseAligned;

        init_p01(&mut state);

        assert_eq!(state.imu_alignment_state, ImuAlignmentState::Caged);
    }

    /// TC-P02-1: `init_p02` from Caged schedules the first gyrocompass step.
    ///
    /// After init, major_mode = 2 and the Waitlist has a pending entry.
    #[test]
    fn tc_p02_1_from_caged_schedules_loop() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::Caged;

        init_p02(&mut state);

        assert_eq!(state.major_mode, P02_MAJOR_MODE);
        // Still Caged until the first Waitlist step fires.
        assert_eq!(state.imu_alignment_state, ImuAlignmentState::Caged);
        assert_eq!(state.alarm.code(), 0);
        assert!(
            state.waitlist.front_delta().is_some(),
            "Waitlist must have a pending gyrocompass step"
        );
    }

    /// TC-P02-2: `init_p02` from FineAligned raises alarm 235 but still starts.
    #[test]
    fn tc_p02_2_from_fine_aligned_alarm() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::FineAligned;

        init_p02(&mut state);

        assert_eq!(state.alarm.code(), ALARM_GYROCOMPASS_WRONG_STATE);
        assert!(state.alarm.lit);
        assert_eq!(state.major_mode, P02_MAJOR_MODE);
    }

    /// TC-P02-3: Convergence — 30° CDU misalignment settles to CoarseAligned.
    ///
    /// Sets all CDU axes to ≈ 30° (≈ 5461 counts), runs the gyrocompass loop,
    /// and verifies:
    /// - `imu_alignment_state` reaches `CoarseAligned` within 200 steps.
    /// - All CDU axes are within `COARSE_ALIGN_THRESHOLD` at convergence.
    #[test]
    fn tc_p02_3_convergence_from_30_degree_misalignment() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::Caged;
        state.launch_lat_rad = 0.4986; // KSC

        // 30° = 30/360 × 65536 ≈ 5461 CDU counts
        let counts_30deg: i16 = (30.0_f64 / 360.0 * 65536.0).round() as i16;
        set_cdu_all(&mut state, counts_30deg);

        init_p02(&mut state);

        let steps = drain_gyrocompass(&mut state, 200);

        assert_eq!(
            state.imu_alignment_state,
            ImuAlignmentState::CoarseAligned,
            "TC-P02-3: must reach CoarseAligned within 200 steps, took {steps}"
        );
        for (i, c) in state.current_cdu.iter().enumerate() {
            assert!(
                c.0.unsigned_abs() <= COARSE_ALIGN_THRESHOLD,
                "TC-P02-3: CDU[{i}] = {} exceeds threshold {}",
                c.0,
                COARSE_ALIGN_THRESHOLD
            );
        }
    }

    /// TC-P02-4: Already aligned (CDU = 0) converges in a single step.
    #[test]
    fn tc_p02_4_already_aligned_single_step() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::Caged;
        // CDU already at zero = aligned.

        init_p02(&mut state);
        let steps = drain_gyrocompass(&mut state, 5);

        assert_eq!(
            state.imu_alignment_state,
            ImuAlignmentState::CoarseAligned,
            "TC-P02-4: zero-CDU platform must converge in 1 step, took {steps}"
        );
        assert_eq!(steps, 1, "TC-P02-4: must converge in exactly 1 step");
    }

    /// TC-P02-5: Loop stops if P02 is superseded by another program.
    ///
    /// After init_p02 sets major_mode=2, switching to major_mode=0 (P00)
    /// must cause the next gyrocompass step to exit without rescheduling.
    #[test]
    fn tc_p02_5_stops_on_program_change() {
        let mut state = AgcState::new();
        state.imu_alignment_state = ImuAlignmentState::Caged;
        set_cdu_all(&mut state, 5000); // large misalignment

        init_p02(&mut state);

        // Simulate crew switching to P00 before the loop runs.
        state.major_mode = 0;
        state.time = crate::types::Met(GYROCOMPASS_PERIOD_CS as u32);

        // Run one step — it should exit without rescheduling.
        if let Some((task_fn, _)) = state.waitlist.pop_task() {
            task_fn(&mut state);
        }

        assert!(
            state.waitlist.front_delta().is_none(),
            "TC-P02-5: Waitlist must be empty after program change"
        );
        // Alignment state unchanged (still Caged — loop never ran to completion).
        assert_ne!(
            state.imu_alignment_state,
            ImuAlignmentState::CoarseAligned,
            "TC-P02-5: alignment must not advance after program switch"
        );
    }

    /// TC-P02-6: earth_rate_horizontal / earth_rate_vertical at KSC.
    #[test]
    fn tc_p02_6_earth_rate_components() {
        let lat = 0.4986_f64; // KSC ≈ 28.57°
        let h = earth_rate_horizontal(lat);
        let v = earth_rate_vertical(lat);

        // Combined magnitude must equal Ω_E.
        let mag = libm::sqrt(h * h + v * v);
        assert!(
            (mag - OMEGA_EARTH).abs() < 1e-12,
            "TC-P02-6: Earth-rate magnitude must be Ω_E, got {mag:.3e}"
        );

        // At KSC latitude, horizontal and vertical components are both non-zero.
        assert!(h > 0.0 && v > 0.0, "TC-P02-6: both components must be positive at KSC");
    }
}
