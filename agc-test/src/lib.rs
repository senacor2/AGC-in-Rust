//! AGC-test utility library.
//!
//! Provides:
//! - [`agc_convert`] — AGC fixed-point word ↔ `f64` conversion utilities.
//! - [`fixtures`] — JSON fixture loading for navigation accuracy tests.
//! - [`entry_sim`] — 3DOF atmospheric-entry integrator for end-to-end scenarios.

pub mod agc_convert;
pub mod entry_sim;
pub mod fixtures;
