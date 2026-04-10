//! DSKY display state (PINBALL system).
//!
//! Maintains the shadow copy of the DSKY display. The T4RUPT handler reads
//! this state and writes changed rows to the hardware via the `Dsky` HAL trait.

/// Current DSKY display state.
#[derive(Clone, Copy, Debug, Default)]
pub struct DskyState {
    /// Currently displayed PROG (major mode) number.
    pub prog: u8,
    /// Currently active VERB code.
    pub verb: u8,
    /// Currently active NOUN code.
    pub noun: u8,
    /// Data registers R1, R2, R3 (displayed as 5-digit signed decimal).
    pub r: [f32; 3],
    /// Verb/Noun flash (crew input request).
    pub flashing: bool,
    /// Indicator lamp states.
    pub uplink_activity: bool,
    pub no_att: bool,
    pub stby: bool,
    pub key_rel: bool,
    pub opr_err: bool,
    pub restart_flag: bool,
    pub gimbal_lock: bool,
    pub temp: bool,
    pub prog_alarm: bool,
    pub comp_acty: bool,
    /// Set by V35 (lamp test). The T4RUPT display shim reads this and
    /// drives every indicator lamp on for one cycle, then clears it.
    pub lamp_test_active: bool,
}
