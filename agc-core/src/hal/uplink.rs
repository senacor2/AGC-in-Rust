// SPDX-License-Identifier: GPL-3.0-or-later
/// Ground uplink receiver interface.
pub trait Uplink {
    /// Return the next received uplink word, or `None` if the buffer is empty.
    fn read_word(&mut self) -> Option<u16>;
}
