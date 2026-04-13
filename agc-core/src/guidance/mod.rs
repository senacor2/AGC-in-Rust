//! Guidance subsystem: burn targeting and maneuver execution.
//!
//! Provides the two-phase guidance architecture for powered maneuvers:
//!
//! 1. **Targeting** (`targeting`): before ignition — compute VG at TIG and predict TGO
//!    (S30.1 / S40.1 / S40.13 path).
//! 2. **Maneuver execution** (`maneuver`): during burn — shrink VG by measured dV, apply
//!    cross-product steering law, detect engine cutoff (S40.8 / STEERING path).
//!
//! KALCMANU pre-burn attitude maneuver is deferred to Milestone 4.
//!
//! AGC source: Comanche055/P30-P37.agc (S30.1, S31.1, pages 635-643)
//!             Comanche055/P40-P47.agc (S40.1, S40.8, S40.13, pages 709-728)
//!             Comanche055/KALCMANU_STEERING.agc (pages 414-419, M4)

pub mod maneuver;
pub mod targeting;

pub use maneuver::{
    new as new_maneuver, ManeuverState, LOTHRUST_THRESHOLD, MIN_MASS_KG, VG_CUTOFF_THRESHOLD,
};
pub use targeting::{
    burn_duration, predict_vg_at_ignition, sps_constants, BurnTarget, G0_MS2, SPS_ISP_S,
    SPS_THRUST_N, SPS_VE_MS,
};
