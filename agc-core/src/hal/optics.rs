//! Optics sub-trait: CM optics shaft/trunnion control.
//!
//! AGC source: P20-P25.agc — star sighting routines.

use crate::types::CduAngle;

/// CM optics shaft and trunnion interface.
///
/// The Command Module optics (sextant + scanning telescope) are driven by two
/// CDU-driven axes: shaft (azimuth) and trunnion (elevation).
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc — OPTY/OPTX optics CDU counters.
pub trait Optics {
    /// Read the current shaft (azimuth) CDU angle.
    fn read_shaft(&self) -> CduAngle;

    /// Read the current trunnion (elevation) CDU angle.
    fn read_trunnion(&self) -> CduAngle;

    /// Drive optics shaft to a target angle (coarse positioning).
    fn drive_shaft(&mut self, target: CduAngle);

    /// Drive optics trunnion to a target angle.
    fn drive_trunnion(&mut self, target: CduAngle);

    /// Enable or disable the optics zero (AOT) mode.
    fn set_zero_optics(&mut self, enabled: bool);

    /// True if a star mark (astronaut button press) is pending.
    fn mark_pending(&self) -> bool;

    /// Clear the pending mark.
    fn clear_mark(&mut self);
}
