//! Simulated HAL implementation for host-side testing and simulation.
//!
//! Provides:
//! - `SimHardware` — full `AgcHardware` impl for unit tests and integration tests
//! - `DskyTerminal` — ratatui TUI with live DSKY display and keyboard input
//! - `SimLog` — ring-buffer event log shown in the TUI sidebar

pub mod dsky_state;
pub mod dsky_terminal;
pub mod hardware;
pub mod nav_terminal;
pub mod sim_log;

pub use dsky_state::DskyDisplayState;
pub use hardware::SimHardware;
pub use sim_log::SimLog;
