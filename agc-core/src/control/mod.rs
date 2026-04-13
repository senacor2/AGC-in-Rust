//! Control subsystem: DAP supervisor, attitude computation, RCS jet selection, TVC gimbal steering.
//!
//! This module implements the Digital Autopilot (DAP) control cycle driven by
//! T5RUPT at 100 ms intervals in RCS mode and by self-perpetuating T5 tasks in
//! TVC mode. The RCS uses bang-bang phase-plane switching; TVC uses a simplified
//! PD law in place of the AGC's 6th-order cascade filter (ADR-001).
//!
//! # Module hierarchy
//!
//! - `constants` — shared AGC-cited constants (T5RUPT period, deadband, slopes)
//! - `attitude`  — attitude error computation and phase-plane switching function
//! - `rcs_logic` — PYTABLE/RTABLE jet selection (JETSLECT implementation)
//! - `tvc`       — TVC gimbal steering (PITCHDAP/YAWDAP simplified PD law)
//! - `dap`       — DAP supervisor: mode state machine + `t5rupt_tick`
//!
//! AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc (pages 1002-1024)
//!             Comanche055/JET_SELECTION_LOGIC.agc (pages 1039-1062)
//!             Comanche055/TVCDAPS.agc (pages 961-978)
//!             Comanche055/TVCEXECUTIVE.agc (pages 945-950)
//!             Comanche055/TVCROLLDAP.agc (pages 984-998)

pub mod attitude;
pub mod constants;
pub mod dap;
pub mod imu_control;
pub mod rcs_logic;
pub mod tvc;

pub use attitude::{compute_error, phase_plane_decision, AttitudeError, JetDecision};
pub use constants::{
    DEADBAND_DEFAULT_RAD, K_PRIME, SLOPE, T5RUPT_PERIOD, T5RUPT_PERIOD_CS, TVC_ACTSAT_RAD,
    TVC_ROLL_DEADBAND_RAD,
};
pub use dap::{set_mode, t5rupt_tick, AttitudeTarget, Dap, DapMode};
pub use rcs_logic::{min_impulse_duration, select_jets, MIN_IMPULSE_CS, MIN_IMPULSE_TIME6_COUNTS};
pub use tvc::{steer, TvcGains, ACTSAT_DEG, ACTSAT_RAD};
