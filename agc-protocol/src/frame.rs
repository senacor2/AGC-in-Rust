use crate::crc::Crc16;
use crate::msg::{DecodeError, EncodeError, Msg};

pub const STX: u8 = 0xFE;
pub const MAX_PAYLOAD: usize = 247;
/// STX(1) + LEN(1) + SEQ(1) + TYPE(1) + payload(247) + CRC_LO(1) + CRC_HI(1).
pub const MAX_FRAME: usize = 252;

/// Encode `msg` into `out`, return total bytes written.
pub fn encode(msg: &Msg, seq: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
    const HDR: usize = 4; // STX + LEN + SEQ + TYPE
    const TRL: usize = 2; // CRC_LO + CRC_HI

    if out.len() < HDR + TRL {
        return Err(EncodeError::BufferTooSmall);
    }
    let out_len = out.len();
    let payload_len = msg.encode_payload(&mut out[HDR..out_len - TRL])?;
    let total = HDR + payload_len + TRL;
    if out.len() < total {
        return Err(EncodeError::BufferTooSmall);
    }

    out[0] = STX;
    out[1] = payload_len as u8;
    out[2] = seq;
    out[3] = msg.type_byte();

    // CRC over LEN + SEQ + TYPE + payload (bytes at indices 1..HDR+payload_len).
    let mut crc = Crc16::new();
    for b in &out[1..HDR + payload_len] {
        crc.update(*b);
    }
    let checksum = crc.finish();
    out[HDR + payload_len] = checksum as u8; // CRC_LO
    out[HDR + payload_len + 1] = (checksum >> 8) as u8; // CRC_HI

    Ok(total)
}

// ---- decoder ----------------------------------------------------------------

/// Byte-at-a-time decoder state.
#[derive(Clone, Copy)]
enum Phase {
    /// Waiting for STX.
    Idle,
    /// Got STX, waiting for LEN.
    WaitLen,
    /// Got LEN, waiting for SEQ.
    WaitSeq { len: u8 },
    /// Got SEQ, waiting for TYPE.
    WaitType { len: u8, seq: u8 },
    /// Got TYPE; collecting payload bytes (received < len).
    CollectPayload {
        len: u8,
        seq: u8,
        type_byte: u8,
        received: u8,
    },
    /// All payload bytes received, waiting for CRC_LO.
    WaitCrcLo { len: u8, seq: u8, type_byte: u8 },
    /// Got CRC_LO, waiting for CRC_HI.
    WaitCrcHi {
        len: u8,
        seq: u8,
        type_byte: u8,
        crc_lo: u8,
    },
}

pub struct FrameDecoder {
    phase: Phase,
    buf: [u8; MAX_PAYLOAD],
    crc: Crc16,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    pub const fn new() -> Self {
        Self {
            phase: Phase::Idle,
            buf: [0u8; MAX_PAYLOAD],
            crc: Crc16::new(),
        }
    }

    pub fn reset(&mut self) {
        self.phase = Phase::Idle;
        self.crc = Crc16::new();
    }

    /// Feed one byte into the decoder.
    /// On `Error`, the decoder auto-resets to `Idle` so the next `STX` starts fresh.
    /// STX (0xFE) mid-frame is treated as the start of a new frame, enabling
    /// resynchronisation after a truncated or garbled frame.
    pub fn push(&mut self, b: u8) -> DecodeStatus {
        // STX mid-frame: abandon the current frame and start fresh.
        if b == STX && !matches!(self.phase, Phase::Idle | Phase::WaitLen) {
            self.crc = Crc16::new();
            self.phase = Phase::WaitLen;
            return DecodeStatus::NeedMore;
        }

        match self.phase {
            Phase::Idle => {
                if b == STX {
                    self.crc = Crc16::new();
                    self.phase = Phase::WaitLen;
                }
                // non-STX bytes are silently dropped in Idle
                DecodeStatus::NeedMore
            }

            Phase::WaitLen => {
                if b as usize > MAX_PAYLOAD {
                    self.reset();
                    return DecodeStatus::Error(DecodeError::BadLength);
                }
                self.crc.update(b);
                self.phase = Phase::WaitSeq { len: b };
                DecodeStatus::NeedMore
            }

            Phase::WaitSeq { len } => {
                self.crc.update(b);
                self.phase = Phase::WaitType { len, seq: b };
                DecodeStatus::NeedMore
            }

            Phase::WaitType { len, seq } => {
                self.crc.update(b);
                let type_byte = b;
                if len == 0 {
                    self.phase = Phase::WaitCrcLo {
                        len,
                        seq,
                        type_byte,
                    };
                } else {
                    self.phase = Phase::CollectPayload {
                        len,
                        seq,
                        type_byte,
                        received: 0,
                    };
                }
                DecodeStatus::NeedMore
            }

            Phase::CollectPayload {
                len,
                seq,
                type_byte,
                received,
            } => {
                self.crc.update(b);
                self.buf[received as usize] = b;
                let next = received + 1;
                if next == len {
                    self.phase = Phase::WaitCrcLo {
                        len,
                        seq,
                        type_byte,
                    };
                } else {
                    self.phase = Phase::CollectPayload {
                        len,
                        seq,
                        type_byte,
                        received: next,
                    };
                }
                DecodeStatus::NeedMore
            }

            Phase::WaitCrcLo {
                len,
                seq,
                type_byte,
            } => {
                self.phase = Phase::WaitCrcHi {
                    len,
                    seq,
                    type_byte,
                    crc_lo: b,
                };
                DecodeStatus::NeedMore
            }

            Phase::WaitCrcHi {
                len,
                seq,
                type_byte,
                crc_lo,
            } => {
                let received_crc = (crc_lo as u16) | ((b as u16) << 8);
                let computed = self.crc.finish();

                // Reset before parsing so buf[] data is still intact.
                self.phase = Phase::Idle;
                self.crc = Crc16::new();

                if computed != received_crc {
                    return DecodeStatus::Error(DecodeError::BadCrc);
                }

                let payload = &self.buf[..len as usize];
                match Msg::decode_payload(type_byte, payload) {
                    Ok(msg) => DecodeStatus::Ready { msg, seq },
                    Err(e) => DecodeStatus::Error(e),
                }
            }
        }
    }
}

pub enum DecodeStatus {
    NeedMore,
    Ready { msg: Msg, seq: u8 },
    Error(DecodeError),
}
