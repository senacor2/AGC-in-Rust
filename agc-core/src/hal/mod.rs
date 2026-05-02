pub mod dsky;
pub mod engine;
pub mod imu;
pub mod interrupts;
pub mod optics;
pub mod rcs;
pub mod runtime;
pub mod telemetry;
pub mod timers;
pub mod uplink;

pub use dsky::{Dsky, Lamp};
pub use engine::Engine;
pub use imu::Imu;
pub use interrupts::Interrupt;
pub use optics::Optics;
pub use rcs::Rcs;
pub use telemetry::Telemetry;
pub use timers::Timers;
pub use uplink::Uplink;

/// Master hardware abstraction trait.
///
/// The flight software is generic over `H: AgcHardware`. The bare-metal
/// implementation wires the associated types to `embedded-hal` peripherals
/// for a specific MCU. The simulator (`agc-sim`) provides a host-side
/// implementation backed by a software dynamics model.
///
/// Rules for implementors (see architecture §4.1):
/// - Peripheral side effects (jet firing, display relay timing) are
///   encapsulated here; the flight software never touches hardware registers.
/// - The `embedded-hal` v1 traits are used *inside* implementations, not
///   exposed to the flight software.
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

    /// Reset the hardware watchdog timer. Must be called at least once
    /// every ~1.28 s from the Executive main loop (night-watchman).
    fn pet_watchdog(&mut self);

    /// Trigger an immediate hardware restart (equivalent to GOJAM).
    /// Called from the panic handler and from software-initiated restarts.
    fn hardware_restart(&mut self) -> !;
}
