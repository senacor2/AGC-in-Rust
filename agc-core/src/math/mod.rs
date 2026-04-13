//! Mathematical primitives for the AGC navigation stack.
//!
//! This module provides pure, no-alloc, no_std-safe building blocks used
//! throughout the navigation, guidance, and control layers.
//!
//! AGC source: Comanche055/CONIC_SUBROUTINES.agc (GEOM, KEPLERN, GETX, LAMROUT)
//!             Comanche055/SERVICER207.agc (CALCGRAV, CALCRVG, NORMLIZE)
//!             Comanche055/ORBITAL_INTEGRATION.agc (OBLATE, INTGRATE, DIFEQ0)
//!             Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc (CALCGA)

pub mod kepler;
pub mod lambert;
pub mod linalg;
pub mod trig;
