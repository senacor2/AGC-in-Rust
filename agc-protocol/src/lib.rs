#![no_std]

mod crc;
mod frame;
mod msg;

pub use frame::{encode, DecodeStatus, FrameDecoder, MAX_FRAME, MAX_PAYLOAD, STX};
pub use msg::{DecodeError, EncodeError, Msg};

/// Wire-protocol version negotiated during the Hello / HelloAck handshake.
pub const PROTO_VERSION: u8 = 1;
