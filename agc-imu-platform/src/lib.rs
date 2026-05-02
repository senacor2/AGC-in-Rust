#![no_std]

pub mod platform;
pub mod quat;

pub use platform::{PipaCounts, PlatformEmulator};
pub use quat::UnitQuaternion;

/// Pulses per (m/s). Nominal PIPA scale: 1 pulse ≈ 0.0585 m/s.
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc — PIPADT constant.
pub const PIPA_SCALE: f64 = 1.0 / 0.0585;

/// Radians per gyro torque pulse (B-15 rev scale).
/// 1 pulse = TAU / 32768 rad ≈ 1.9175e-4 rad.
/// Distinct from CDU_PULSE_RAD: gyro pulses use 15-bit range over a full revolution,
/// CDU counts use 16-bit range.
pub const GYRO_PULSE_RAD: f64 = core::f64::consts::TAU / 32768.0;

/// Radians per CDU count (B-1 rev scale, 16-bit full revolution).
/// 1 count = TAU / 65536 rad ≈ 9.587e-5 rad.
pub const CDU_PULSE_RAD: f64 = core::f64::consts::TAU / 65536.0;
