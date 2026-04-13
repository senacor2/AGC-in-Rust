//! Hardware Abstraction Layer for the AGC Rust port.
//!
//! The HAL is the sole boundary between the flight software and physical hardware.
//! All peripheral access must go through the `AgcHardware` super-trait and its
//! sub-traits.  No peripheral register or hardware address may appear outside `hal/`.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (channel assignments, pages 41-42)
//!             docs/architecture.md §4 — HAL design, sub-trait structure, typestate conventions.

pub mod dsky;
pub mod engine;
pub mod imu;
pub mod optics;
pub mod rcs;
pub mod telemetry;
pub mod timers;
pub mod uplink;

pub use dsky::{DigitRow, DskyIo, Key, RelayWord};
pub use engine::EngineIo;
pub use imu::{CoarseAligned, FineAligned, ImuImpl, ImuIo, Unaligned};
pub use optics::OpticsIo;
pub use rcs::{JetCommand, RcsIo};
pub use telemetry::TelemetryIo;
pub use timers::Timers;
pub use uplink::UplinkIo;

/// Bound that the flight software requires of the platform.
///
/// The bare-metal implementation wires each associated type to an `embedded-hal`
/// peripheral wrapper.  The `agc-sim` implementation wires each associated type
/// to a software model.
///
/// All mutable access to hardware goes through this trait and its sub-traits.
/// No peripheral register access is permitted outside `hal/`.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (all channel assignments).
pub trait AgcHardware {
    /// Hardware timer type (T3/T4/T5/T6).
    type Timers: Timers;
    /// DSKY display/keyboard type.
    type Dsky: DskyIo;
    /// IMU CDU/PIPA/gyro type.
    type Imu: ImuIo;
    /// Optics shaft/trunnion type.
    type Optics: OpticsIo;
    /// SPS engine type.
    type Engine: EngineIo;
    /// RCS jet type.
    type Rcs: RcsIo;
    /// Uplink receiver type.
    type Uplink: UplinkIo;
    /// Telemetry downlink type.
    type Telemetry: TelemetryIo;

    /// Obtain a mutable reference to the timers subsystem.
    fn timers(&mut self) -> &mut Self::Timers;

    /// Obtain a mutable reference to the DSKY subsystem.
    fn dsky(&mut self) -> &mut Self::Dsky;

    /// Obtain a mutable reference to the IMU subsystem.
    fn imu(&mut self) -> &mut Self::Imu;

    /// Obtain a mutable reference to the optics subsystem.
    fn optics(&mut self) -> &mut Self::Optics;

    /// Obtain a mutable reference to the engine subsystem.
    fn engine(&mut self) -> &mut Self::Engine;

    /// Obtain a mutable reference to the RCS subsystem.
    fn rcs(&mut self) -> &mut Self::Rcs;

    /// Obtain a mutable reference to the uplink receiver.
    fn uplink(&mut self) -> &mut Self::Uplink;

    /// Obtain a mutable reference to the telemetry downlink.
    fn telemetry(&mut self) -> &mut Self::Telemetry;

    /// Reset the night-watchman (hardware watchdog) timer.
    ///
    /// Must be called at least once per Executive loop iteration.
    /// If not called within ~1.28 s, the hardware triggers a restart.
    ///
    /// AGC source: Comanche055/EXECUTIVE.agc DUMMYJOB / main loop watchdog.
    fn pet_watchdog(&mut self);

    /// Trigger an immediate hardware restart.
    ///
    /// Called by the alarm system on unrecoverable failure (GOJAM equivalent).
    ///
    /// AGC source: Comanche055/ALARM_AND_ABORT.agc CURTAINS routine.
    fn hardware_restart(&mut self) -> !;
}
