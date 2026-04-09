//! Hardware Abstraction Layer.
//!
//! `AgcHardware` is the boundary between the flight software and the physical
//! (or simulated) machine. The flight software only ever calls these traits;
//! it never touches memory-mapped registers directly.
//!
//! In bare-metal builds the impl wraps `embedded-hal` peripherals.
//! In simulation (`agc-sim`) the impl is a software model.
//!
//! AGC source: INTERRUPT_LEAD_INS.agc, channel definitions.

pub mod dsky;
pub mod engine;
pub mod imu;
pub mod optics;
pub mod rcs;
pub mod telemetry;
pub mod timers;
pub mod uplink;

pub use dsky::Dsky;
pub use engine::Engine;
pub use imu::Imu;
pub use optics::Optics;
pub use rcs::Rcs;
pub use telemetry::Telemetry;
pub use timers::Timers;
pub use uplink::Uplink;

/// Complete hardware bound required by the flight software.
///
/// Each associated type is a focused sub-trait; the bare-metal implementation
/// wires these to `embedded-hal` peripherals.
///
/// AGC source: INTERRUPT_LEAD_INS.agc — hardware channel and interrupt wiring.
pub trait AgcHardware {
    type Timers: Timers;
    type Dsky: Dsky;
    type Imu: Imu;
    type Optics: Optics;
    type Engine: Engine;
    type Rcs: Rcs;
    type Uplink: Uplink;
    type Telemetry: Telemetry;

    fn timers(&mut self) -> &mut Self::Timers;
    fn dsky(&mut self) -> &mut Self::Dsky;
    fn imu(&mut self) -> &mut Self::Imu;
    fn optics(&mut self) -> &mut Self::Optics;
    fn engine(&mut self) -> &mut Self::Engine;
    fn rcs(&mut self) -> &mut Self::Rcs;
    fn uplink(&mut self) -> &mut Self::Uplink;
    fn telemetry(&mut self) -> &mut Self::Telemetry;

    /// Reset the night-watchman (hardware watchdog) timer.
    ///
    /// Must be called at least once per Executive loop iteration (~1.28 s).
    /// Failure triggers a hardware restart.
    ///
    /// AGC source: EXECUTIVE.agc — NEWJOB bit clears the watchdog counter.
    fn pet_watchdog(&mut self);

    /// Trigger an immediate hardware restart. Never returns.
    ///
    /// In bare-metal builds this calls SCB::sys_reset().
    /// In simulation it panics (acceptable: simulation is std).
    fn hardware_restart(&mut self) -> !;
}
