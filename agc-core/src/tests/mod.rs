//! Test helpers and mock implementations for `agc-core` unit tests.
//!
//! These modules are only compiled during `cargo test`.
//! They provide a minimal no-heap `MockHardware` that avoids the circular
//! dependency that would arise from pulling in `agc-sim` here.

pub mod mock_hw;
