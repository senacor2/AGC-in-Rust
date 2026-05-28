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
pub mod uplink;

pub use hardware::SimHardware;
pub use physics::GravityBody;
pub use scenario::{
    run_scenario, DskyExpect, Event, LandmarkTable, Scenario, ScenarioBuilder, SeedStateSpec,
    SimDuration,
};
pub use uplink::ScriptedUplink;
