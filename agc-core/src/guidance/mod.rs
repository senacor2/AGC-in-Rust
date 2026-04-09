//! Guidance: targeting, maneuver planning, and burn steering.
//!
//! AGC source: P30-P37.agc (targeting), P40-P47.agc (burn execution),
//! KALCMANU_STEERING.agc (attitude maneuver steering).

pub mod maneuver;
pub mod targeting;
