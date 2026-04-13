//! `agc-sim`: host-side simulator crate for the AGC-in-Rust project.
//!
//! Provides:
//! - `SimHardware` — implements `AgcHardware` (and all 8 sub-traits) in pure software.
//! - `SimLog` — fixed-capacity ring buffer for the Mission Log panel.
//! - `DskyDisplayState` — DSKY display model consumed by the TUI renderer.
//! - `dsky_terminal` — ratatui render helpers (three-panel layout).
//! - `command_dispatch` — crossterm key-event router.
//!
//! The binary (`agc_sim`) lives in `src/bin/agc_sim.rs` and wires these
//! components together into a 20 Hz render loop.

pub mod command_dispatch;
pub mod dsky_state;
pub mod dsky_terminal;
pub mod sim_hardware;
pub mod sim_log;

pub use command_dispatch::{handle_key_event, DispatchOutcome};
pub use dsky_state::DskyDisplayState;
pub use sim_hardware::SimHardware;
pub use sim_log::{LogLevel, SimLog};

pub use agc_core::services::pinball::{dispatch as verb_dispatch, VerbResult};
pub use agc_core::services::v_n::{CharResult, VnState};
