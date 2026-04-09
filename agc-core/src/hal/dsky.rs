//! DSKY (Display/Keyboard) sub-trait.
//!
//! AGC source: T4RUPT_PROGRAM.agc — relay matrix timing; PINBALL_GAME_BUTTONS_AND_LIGHTS.agc.

/// DSKY key codes received from the keyboard matrix.
///
/// Values are the 5-bit keyboard codes from the DSKY relay matrix, as defined
/// in PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (octal table). Each octal code is
/// listed next to the variant for traceability.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum DskyKey {
    Verb    = 0x11, // OCT 21
    Noun    = 0x1F, // OCT 37  (was wrong: 0x31)
    Plus    = 0x1A, // OCT 32
    Minus   = 0x1B, // OCT 33
    Zero    = 0x10, // OCT 20  (was wrong: 0x00)
    One     = 0x01, // OCT 01
    Two     = 0x02, // OCT 02
    Three   = 0x03, // OCT 03
    Four    = 0x04, // OCT 04
    Five    = 0x05, // OCT 05
    Six     = 0x06, // OCT 06
    Seven   = 0x07, // OCT 07
    Eight   = 0x08, // OCT 10
    Nine    = 0x09, // OCT 11
    Clear   = 0x1E, // OCT 36  (was wrong: 0x18)
    ProceED = 0x14, // OCT 24  (unverified in PINBALL; assigned distinct from KeyRel)
    Enter   = 0x1C, // OCT 34
    Reset   = 0x12, // OCT 22  (was wrong: 0x3A)
    KeyRel  = 0x19, // OCT 31  (was wrong: 0x16)
}

/// DSKY interface: display output and keyboard input.
///
/// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, T4RUPT_PROGRAM.agc.
pub trait Dsky {
    /// Write a raw 14-bit relay word to the DSKY display matrix.
    ///
    /// The relay word format is defined by the channel 010 bit layout:
    /// bits encode which digit position and which 7-segment segments to light.
    ///
    /// AGC source: T4RUPT_PROGRAM.agc — relay matrix scan timing.
    fn write_relay_word(&mut self, word: u16);

    /// Turn the COMP ACTY (computer activity) light on or off.
    fn set_comp_acty(&mut self, on: bool);

    /// Set the UPLINK ACTY light.
    fn set_uplink_acty(&mut self, on: bool);

    /// Set the TEMP light (IMU temperature warning).
    fn set_temp_light(&mut self, on: bool);

    /// Set the GIMBAL LOCK warning light.
    fn set_gimbal_lock(&mut self, on: bool);

    /// Set the PROG alarm light.
    fn set_prog_alarm(&mut self, on: bool);

    /// Set the KEY REL (key release request) light.
    fn set_key_rel(&mut self, on: bool);

    /// Set the OPR ERR (operator error) light.
    fn set_opr_err(&mut self, on: bool);

    /// Poll for a pending keyboard key. Returns `None` if no key pressed.
    fn read_key(&mut self) -> Option<DskyKey>;
}
