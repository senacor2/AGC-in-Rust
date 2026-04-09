//! RCS (Reaction Control System) jet sub-trait.
//!
//! AGC source: JET_SELECTION_LOGIC.agc — 16-jet SM RCS, 12-jet CM RCS.

/// RCS jet mask for the Service Module (16 jets, bits 0–15).
/// Bit numbering follows AGC channel 030/031 layout.
pub type SmJetMask = u16;

/// RCS jet mask for the Command Module (12 jets, bits 0–11).
/// Bit numbering follows AGC channel 032/033 layout.
pub type CmJetMask = u16;

/// RCS jet on/off command interface.
///
/// The AGC fires RCS jets by writing bitmasks to hardware I/O channels.
/// Each bit corresponds to one solenoid valve. The HAL translates these
/// masks to the appropriate GPIO/PWM outputs.
///
/// AGC source: JET_SELECTION_LOGIC.agc — JETADR table, channel 030–033 writes.
pub trait Rcs {
    /// Fire SM RCS jets specified by `mask` for `duration_ms` milliseconds.
    ///
    /// The HAL must enforce minimum impulse (~14 ms) and maximum single-fire
    /// duration constraints.
    fn fire_sm_jets(&mut self, mask: SmJetMask, duration_ms: u16);

    /// Fire CM RCS jets specified by `mask` for `duration_ms` milliseconds.
    fn fire_cm_jets(&mut self, mask: CmJetMask, duration_ms: u16);

    /// Immediately cut off all RCS jets.
    fn all_jets_off(&mut self);

    /// Read the current SM jet firing status (which jets are currently on).
    fn sm_jets_firing(&self) -> SmJetMask;

    /// Read the current CM jet firing status.
    fn cm_jets_firing(&self) -> CmJetMask;
}
