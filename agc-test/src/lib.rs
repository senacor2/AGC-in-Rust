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

pub mod agc_convert;
pub mod entry_sim;
pub mod fixtures;
pub mod vagc_channel;
pub mod vagc_harness;
