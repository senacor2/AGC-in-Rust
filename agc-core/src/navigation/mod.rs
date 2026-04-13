//! Navigation layer: state vectors, gravity models, and orbital integration.
//!
//! This module provides the building blocks for the 2-second SERVICER Average-G
//! cycle and for longer-arc orbital propagation.
//!
//! AGC source: Comanche055/SERVICER207.agc (CALCRVG, CALCGRAV, NORMLIZE)
//!             Comanche055/ORBITAL_INTEGRATION.agc (DIFEQ+0/+1/+2, OBLATE, RECTIFY)
//!             Comanche055/ERASABLE_ASSIGNMENTS.agc (RN, VN, PIPTIME, GDT/2, PBODY)

pub mod conics;
pub mod constants;
pub mod gravity;
pub mod integration;
pub mod state_vector;

pub use state_vector::{PrimaryBody, StateVector};
