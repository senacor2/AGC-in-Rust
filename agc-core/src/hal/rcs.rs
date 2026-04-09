/// Reaction Control System jet interface.
///
/// Jets are addressed by bitmask. The SM RCS has 16 jets (two 8-bit words);
/// the CM RCS has 12 jets. The mapping of bitmask positions to physical jets
/// is defined in `control::rcs_logic`.
pub trait Rcs {
    /// Command SM RCS jets. `jets_a` and `jets_b` are 8-bit masks for
    /// the two RCS jet groups (corresponding to the AGC's two output words).
    fn fire_sm_jets(&mut self, jets_a: u8, jets_b: u8);

    /// Command CM RCS jets (used during entry).
    fn fire_cm_jets(&mut self, jets: u16);

    /// Turn off all RCS jets immediately (called by T6RUPT at end of pulse).
    fn quench_all(&mut self);
}
