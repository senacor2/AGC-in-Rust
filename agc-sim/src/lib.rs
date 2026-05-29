//! agc-sim — host-side simulator for agc-core.
//!
//! Provides a software implementation of [`agc_core::hal::AgcHardware`]
//! backed by a simplified spacecraft dynamics model. Used for integration
//! tests, scenario playback, and interactive DSKY simulation.

pub mod dsky_ui;
pub mod hardware;
pub mod physics;
pub mod runtime;
pub mod scenario;
pub mod sensors;
pub mod uplink;

pub use hardware::SimHardware;
pub use physics::{Attitude, GravityBody};
pub use runtime::bridge_dap_to_commanded_q;
pub use scenario::{
    run_scenario, DskyExpect, Event, LandmarkTable, Scenario, ScenarioBuilder, SeedStateSpec,
    SimDuration,
};
pub use sensors::{landmark_los_in_platform, star_los_in_platform};
pub use uplink::ScriptedUplink;
