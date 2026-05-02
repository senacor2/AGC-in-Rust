//! CRC-16/CCITT: poly=0x1021, init=0xFFFF, no reflection, no xorout.

// Convenience wrapper used in tests; callers that need streaming use Crc16 directly.
#[expect(
    dead_code,
    reason = "public helper; used in tests and by external callers"
)]
pub fn crc16_ccitt(bytes: &[u8]) -> u16 {
    let mut state = Crc16::new();
    for &b in bytes {
        state.update(b);
    }
    state.finish()
}

#[derive(Clone, Copy)]
pub struct Crc16(u16);

impl Crc16 {
    pub const fn new() -> Self {
        Self(0xFFFF)
    }

    pub fn update(&mut self, byte: u8) {
        let mut crc = self.0;
        crc ^= (byte as u16) << 8;
        let mut i = 0u8;
        while i < 8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            i += 1;
        }
        self.0 = crc;
    }

    pub fn finish(self) -> u16 {
        self.0
    }
}
