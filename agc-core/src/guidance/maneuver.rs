//! Delta-V computation and burn execution guidance.

use crate::types::{Met, Vec3};

/// Mutable state for a single SPS burn execution instance.
///
/// Created by `burn_init` from a targeting solution (`Maneuver`) and stored
/// in `AgcState::burn`. Persists across Waitlist task boundaries so that a
/// mid-burn RESTART can resume the burn loop without re-igniting the engine.
///
/// AGC source: Comanche055/P40-P47.agc and POWERED_FLIGHT_SUBROUTINES.agc.
/// Corresponds to the DELVEET1/2/3, DVTOTAL, TGO erasable variables and the
/// Group 3 restart phase flags.
#[derive(Clone, Copy, Debug, Default)]
pub struct BurnState {
    /// Target delta-V in the inertial frame (m/s).
    ///
    /// Set once by `burn_init` from the targeting solution.
    /// AGC: derived from DELVEET1/2/3 target word at ignition.
    pub target_dv_inertial: Vec3,

    /// Accumulated delta-V in the inertial frame since ignition (m/s).
    ///
    /// Integrated by `burn_update` each SERVICER cycle.
    /// AGC: DELVEET1/2/3 accumulator (scale B+7 m/s).
    pub accumulated_dv_inertial: Vec3,

    /// Time of ignition (MET, centiseconds).
    ///
    /// Recorded at burn start; used to compute the backup cutoff time.
    /// AGC: stored in the restart area for Group 3 protection.
    pub tig: Met,

    /// `true` while the SPS engine is commanded on.
    ///
    /// Set to `true` by `burn_init`; cleared by P40 when `is_burn_complete`
    /// returns `true`. The P40 program wrapper reads this flag to call
    /// `hw.engine().sps_enable(false)`.
    pub burn_active: bool,

    /// `true` once the backup cutoff time has been passed.
    ///
    /// Set by `burn_update` when `AgcState::time >= compute_cutoff_time(state)`.
    /// Causes `is_burn_complete` to return `true` even if the delta-V target
    /// has not been reached (runaway engine protection).
    pub cutoff_time_met: bool,
}
