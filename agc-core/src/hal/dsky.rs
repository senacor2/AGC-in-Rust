/// DSKY (Display and Keyboard) hardware interface.
///
/// Per ADR-019 the bridge uses a per-field row encoding (not the original
/// AGC relay matrix). 21 rows total: rows 0–2 carry PROG/VERB/NOUN, rows
/// 3–8 carry R1 (sign + 5 digits), rows 9–14 R2, rows 15–20 R3. Digits
/// are raw BCD (0x0–0x9); 0xF blanks a digit. All 21 rows are re-emitted
/// on every T4RUPT (every 120 ms). Indicator lamps go through `set_lamp`,
/// VERB/NOUN flashing through `set_flash`.
pub trait Dsky {
    /// Write a display row.
    /// `row` selects the field row (0–20); `data` packs the BCD digits
    /// (tens in bits 7–4, units in bits 3–0; 0xF means blank).
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
