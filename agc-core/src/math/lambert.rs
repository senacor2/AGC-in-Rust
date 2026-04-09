//! Lambert's problem — transfer orbit between two position vectors in a given time.
//!
//! Implements the Lambert targeting algorithm that underlies P31, P34, and
//! the general targeting routines.

use crate::types::Vec3;

/// Solve Lambert's problem: given initial position `r1`, final position `r2`,
/// transfer time `tof` (seconds), and central body parameter `mu` (m³/s²),
/// return the departure velocity at `r1` and arrival velocity at `r2`.
///
/// `prograde` selects the short-way (true) or long-way (false) solution.
pub fn lambert(r1: Vec3, r2: Vec3, tof: f64, mu: f64, prograde: bool) -> (Vec3, Vec3) {
    let _ = (r1, r2, tof, mu, prograde);
    todo!("Lambert solver (Izzo or Gooding method)")
}
