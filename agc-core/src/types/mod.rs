//! Physical-quantity newtypes for the AGC Rust port.
//!
//! All types compile with `#![no_std]` and use no heap allocation.
//! Every public item documents its unit and scale factor.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (special registers, CDU/PIPA
//! assignments, pages 39-41); docs/architecture.md §3.2-3.3;
//! docs/agc-reference-constants.md (scale-factor table).

pub mod angle;
pub mod delta_v;
pub mod matrix;
pub mod time;
pub mod vector;

pub use angle::CduAngle;
pub use matrix::{mat3, mat_mat_mul, mat_vec_mul, transpose, Mat3x3, IDENTITY_MAT3, ZERO_MAT3};
pub use time::Met;
pub use vector::{vec3, DeltaV, Vec3, PIPA_SCALE, ZERO_VEC3};
