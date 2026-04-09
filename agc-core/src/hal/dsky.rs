/// DSKY (Display and Keyboard) hardware interface.
///
/// The display uses a row-select relay matrix; timing constraints (20 ms
/// hold per row) are enforced inside the implementation, not by callers.
/// The flight software calls `write_row` / `set_lamp` and the HAL handles
/// the relay sequencing.
pub trait Dsky {
    /// Write a display row.
    /// `row` selects the relay row (1–14); `data` contains the segment bits.
    fn write_row(&mut self, row: u8, data: u16);

    /// Clear a display row (turn all segments off for that row).
    fn clear_row(&mut self, row: u8);

    /// Set or clear an indicator lamp (PROG alarm, GIMBAL LOCK, NO ATT, etc.).
    fn set_lamp(&mut self, lamp: Lamp, on: bool);

    /// Enable or disable the flashing VERB/NOUN indicators (crew input request).
    fn set_flash(&mut self, on: bool);

    /// Return the next pending keypress code, or `None` if no key has been pressed.
    /// The 5-bit code corresponds to the DSKY key matrix encoding.
    fn read_key(&mut self) -> Option<u8>;
}

/// DSKY indicator lamps.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Lamp {
    UplinkActivity,
    NoAtt,
    Stby,
    KeyRel,
    OprErr,
    Restart,
    GimbalLock,
    Temp,
    ProgAlarm,
    CompActy,
}
