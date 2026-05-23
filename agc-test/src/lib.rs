//! AGC-test utility library.
//!
//! Provides:
//! - [`agc_convert`] — AGC fixed-point word ↔ `f64` conversion utilities.
//! - [`fixtures`] — JSON fixture loading for navigation accuracy tests.
//! - [`entry_sim`] — 3DOF atmospheric-entry integrator for end-to-end scenarios.
//! - [`vagc_harness`] — yaAGC core-dump + symbol-table I/O for the
//!   routine-level entry fixture-capture harness.
//! - [`vagc_channel`] — TCP client for yaAGC's channel-word socket
//!   protocol; foundation for the end-to-end channel-trace tests.
//! - [`vagc_driver`] — DSKY keypress + PIPA pulse drivers over the
//!   yaAGC channel protocol.
//! - [`vagc_trace`] — channel-write recorder, JSON fixture format,
//!   and Rust-side comparator for MS-E7c.

pub mod agc_convert;
pub mod entry_scenario;
pub mod entry_sim;
pub mod entry_state;
pub mod fixtures;
pub mod vagc_channel;
pub mod vagc_driver;
pub mod vagc_harness;
pub mod vagc_trace;
