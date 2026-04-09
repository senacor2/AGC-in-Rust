//! Guidance and control: Digital Autopilot, attitude control, RCS logic, TVC.
//!
//! AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc, JET_SELECTION_LOGIC.agc,
//! TVCEXECUTIVE.agc, TVCDAPS.agc.

pub mod attitude;
pub mod dap;
pub mod imu_control;
pub mod rcs_logic;
pub mod tvc;
