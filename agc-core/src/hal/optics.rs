use crate::types::CduAngle;

/// CM optics (sextant / telescope) interface.
pub trait Optics {
    /// Read the trunnion CDU angle.
    fn trunnion_angle(&self) -> CduAngle;

    /// Read the shaft CDU angle.
    fn shaft_angle(&self) -> CduAngle;

    /// Command optics drive (position mode).
    /// `trunnion` and `shaft` are signed rate commands in CDU counts/s.
    fn drive(&mut self, trunnion: i16, shaft: i16);

    /// Return true if the optics mark button is pressed.
    fn mark_pressed(&self) -> bool;

    /// Clear the MARK edge after the sextant-interrupt handler has consumed
    /// it. Required so a single keystroke fires exactly one program-level
    /// dispatch. Hardware adapters that latch MARK as a one-shot can leave
    /// this as the default no-op; software-modelled adapters
    /// (e.g. [`SimOptics`](../../agc_sim/hardware/struct.SimOptics.html))
    /// must reset their stored bool here.
    fn clear_mark(&mut self) {}
}
