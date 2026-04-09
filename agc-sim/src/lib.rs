//! agc-sim — host-side simulator for agc-core.
//!
//! Provides a software implementation of [`agc_core::hal::AgcHardware`]
//! backed by a simplified spacecraft dynamics model. Used for integration
//! tests, scenario playback, and interactive DSKY simulation.

pub mod dsky_ui;
pub mod hardware;
pub mod physics;
pub mod scenario;

pub use hardware::SimHardware;
