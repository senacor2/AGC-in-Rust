/// All messages that can travel over the bridge link.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Msg {
    // AGC → bridge
    DskyWriteRow { row: u8, data: u16 },
    DskyClearRow { row: u8 },
    DskySetLamp { lamp: u8, on: u8 },
    DskySetFlash { on: u8 },
    OpticsDrive { trunnion: i16, shaft: i16 },
    EngineSpsEnable { on: u8 },
    EngineSpsGimbal { pitch: i16, yaw: i16 },
    RcsFireSm { jets_a: u8, jets_b: u8 },
    RcsFireCm { jets: u16 },
    RcsQuenchAll,
    TelemetryWord { word: u16 },
    AgcHeartbeat { mission_time_cs: u32 },
    // bridge → AGC
    DskyKey { code: u8, dsky: u8 },
    OpticsCdu { trunnion: u16, shaft: u16 },
    OpticsMark,
    EngineThrustOn { on: u8 },
    UplinkWord { word: u16 },
    BridgeHeartbeat { uptime_ms: u32 },
    Hello { proto_version: u8 },
    HelloAck { proto_version: u8 },
    // both directions
    Error { code: u8, ctx: u8 },
}

impl Msg {
    pub fn type_byte(&self) -> u8 {
        match self {
            Msg::DskyWriteRow { .. } => 0x10,
            Msg::DskyClearRow { .. } => 0x11,
            Msg::DskySetLamp { .. } => 0x12,
            Msg::DskySetFlash { .. } => 0x13,
            Msg::OpticsDrive { .. } => 0x20,
            Msg::EngineSpsEnable { .. } => 0x30,
            Msg::EngineSpsGimbal { .. } => 0x31,
            Msg::RcsFireSm { .. } => 0x40,
            Msg::RcsFireCm { .. } => 0x41,
            Msg::RcsQuenchAll => 0x42,
            Msg::TelemetryWord { .. } => 0x4A,
            Msg::AgcHeartbeat { .. } => 0x70,
            Msg::DskyKey { .. } => 0x80,
            Msg::OpticsCdu { .. } => 0xA0,
            Msg::OpticsMark => 0xA1,
            Msg::EngineThrustOn { .. } => 0xB0,
            Msg::UplinkWord { .. } => 0xC0,
            Msg::BridgeHeartbeat { .. } => 0xD0,
            Msg::Hello { .. } => 0xE0,
            Msg::HelloAck { .. } => 0xE1,
            Msg::Error { .. } => 0xEF,
        }
    }

    /// Write payload bytes into `out`, return byte count.
    pub fn encode_payload(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let needed = self.payload_len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        match *self {
            Msg::DskyWriteRow { row, data } => {
                out[0] = row;
                out[1..3].copy_from_slice(&data.to_le_bytes());
            }
            Msg::DskyClearRow { row } => {
                out[0] = row;
            }
            Msg::DskySetLamp { lamp, on } => {
                out[0] = lamp;
                out[1] = on;
            }
            Msg::DskySetFlash { on } => {
                out[0] = on;
            }
            Msg::OpticsDrive { trunnion, shaft } => {
                out[0..2].copy_from_slice(&trunnion.to_le_bytes());
                out[2..4].copy_from_slice(&shaft.to_le_bytes());
            }
            Msg::EngineSpsEnable { on } => {
                out[0] = on;
            }
            Msg::EngineSpsGimbal { pitch, yaw } => {
                out[0..2].copy_from_slice(&pitch.to_le_bytes());
                out[2..4].copy_from_slice(&yaw.to_le_bytes());
            }
            Msg::RcsFireSm { jets_a, jets_b } => {
                out[0] = jets_a;
                out[1] = jets_b;
            }
            Msg::RcsFireCm { jets } => {
                out[0..2].copy_from_slice(&jets.to_le_bytes());
            }
            Msg::RcsQuenchAll => {}
            Msg::TelemetryWord { word } => {
                out[0..2].copy_from_slice(&word.to_le_bytes());
            }
            Msg::AgcHeartbeat { mission_time_cs } => {
                out[0..4].copy_from_slice(&mission_time_cs.to_le_bytes());
            }
            Msg::DskyKey { code, dsky } => {
                out[0] = code;
                out[1] = dsky;
            }
            Msg::OpticsCdu { trunnion, shaft } => {
                out[0..2].copy_from_slice(&trunnion.to_le_bytes());
                out[2..4].copy_from_slice(&shaft.to_le_bytes());
            }
            Msg::OpticsMark => {}
            Msg::EngineThrustOn { on } => {
                out[0] = on;
            }
            Msg::UplinkWord { word } => {
                out[0..2].copy_from_slice(&word.to_le_bytes());
            }
            Msg::BridgeHeartbeat { uptime_ms } => {
                out[0..4].copy_from_slice(&uptime_ms.to_le_bytes());
            }
            Msg::Hello { proto_version } => {
                out[0] = proto_version;
            }
            Msg::HelloAck { proto_version } => {
                out[0] = proto_version;
            }
            Msg::Error { code, ctx } => {
                out[0] = code;
                out[1] = ctx;
            }
        }
        Ok(needed)
    }

    pub fn decode_payload(type_byte: u8, payload: &[u8]) -> Result<Msg, DecodeError> {
        let msg = match type_byte {
            0x10 => {
                check_len(payload, 3)?;
                Msg::DskyWriteRow {
                    row: payload[0],
                    data: u16_le(&payload[1..]),
                }
            }
            0x11 => {
                check_len(payload, 1)?;
                Msg::DskyClearRow { row: payload[0] }
            }
            0x12 => {
                check_len(payload, 2)?;
                Msg::DskySetLamp {
                    lamp: payload[0],
                    on: payload[1],
                }
            }
            0x13 => {
                check_len(payload, 1)?;
                Msg::DskySetFlash { on: payload[0] }
            }
            0x20 => {
                check_len(payload, 4)?;
                Msg::OpticsDrive {
                    trunnion: i16_le(&payload[0..]),
                    shaft: i16_le(&payload[2..]),
                }
            }
            0x30 => {
                check_len(payload, 1)?;
                Msg::EngineSpsEnable { on: payload[0] }
            }
            0x31 => {
                check_len(payload, 4)?;
                Msg::EngineSpsGimbal {
                    pitch: i16_le(&payload[0..]),
                    yaw: i16_le(&payload[2..]),
                }
            }
            0x40 => {
                check_len(payload, 2)?;
                Msg::RcsFireSm {
                    jets_a: payload[0],
                    jets_b: payload[1],
                }
            }
            0x41 => {
                check_len(payload, 2)?;
                Msg::RcsFireCm {
                    jets: u16_le(payload),
                }
            }
            0x42 => {
                check_len(payload, 0)?;
                Msg::RcsQuenchAll
            }
            0x4A => {
                check_len(payload, 2)?;
                Msg::TelemetryWord {
                    word: u16_le(payload),
                }
            }
            0x70 => {
                check_len(payload, 4)?;
                Msg::AgcHeartbeat {
                    mission_time_cs: u32_le(payload),
                }
            }
            0x80 => {
                check_len(payload, 2)?;
                Msg::DskyKey {
                    code: payload[0],
                    dsky: payload[1],
                }
            }
            0xA0 => {
                check_len(payload, 4)?;
                Msg::OpticsCdu {
                    trunnion: u16_le(&payload[0..]),
                    shaft: u16_le(&payload[2..]),
                }
            }
            0xA1 => {
                check_len(payload, 0)?;
                Msg::OpticsMark
            }
            0xB0 => {
                check_len(payload, 1)?;
                Msg::EngineThrustOn { on: payload[0] }
            }
            0xC0 => {
                check_len(payload, 2)?;
                Msg::UplinkWord {
                    word: u16_le(payload),
                }
            }
            0xD0 => {
                check_len(payload, 4)?;
                Msg::BridgeHeartbeat {
                    uptime_ms: u32_le(payload),
                }
            }
            0xE0 => {
                check_len(payload, 1)?;
                Msg::Hello {
                    proto_version: payload[0],
                }
            }
            0xE1 => {
                check_len(payload, 1)?;
                Msg::HelloAck {
                    proto_version: payload[0],
                }
            }
            0xEF => {
                check_len(payload, 2)?;
                Msg::Error {
                    code: payload[0],
                    ctx: payload[1],
                }
            }
            _ => return Err(DecodeError::UnknownType),
        };
        Ok(msg)
    }

    fn payload_len(&self) -> usize {
        match self {
            Msg::DskyWriteRow { .. } => 3,
            Msg::DskyClearRow { .. } => 1,
            Msg::DskySetLamp { .. } => 2,
            Msg::DskySetFlash { .. } => 1,
            Msg::OpticsDrive { .. } => 4,
            Msg::EngineSpsEnable { .. } => 1,
            Msg::EngineSpsGimbal { .. } => 4,
            Msg::RcsFireSm { .. } => 2,
            Msg::RcsFireCm { .. } => 2,
            Msg::RcsQuenchAll => 0,
            Msg::TelemetryWord { .. } => 2,
            Msg::AgcHeartbeat { .. } => 4,
            Msg::DskyKey { .. } => 2,
            Msg::OpticsCdu { .. } => 4,
            Msg::OpticsMark => 0,
            Msg::EngineThrustOn { .. } => 1,
            Msg::UplinkWord { .. } => 2,
            Msg::BridgeHeartbeat { .. } => 4,
            Msg::Hello { .. } => 1,
            Msg::HelloAck { .. } => 1,
            Msg::Error { .. } => 2,
        }
    }
}

fn check_len(payload: &[u8], expected: usize) -> Result<(), DecodeError> {
    if payload.len() == expected {
        Ok(())
    } else {
        Err(DecodeError::BadLength)
    }
}

fn u16_le(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

fn i16_le(b: &[u8]) -> i16 {
    i16::from_le_bytes([b[0], b[1]])
}

fn u32_le(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EncodeError {
    BufferTooSmall,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DecodeError {
    UnknownType,
    BadLength,
    BadCrc,
}
