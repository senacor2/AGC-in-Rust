use agc_protocol::{encode, DecodeError, DecodeStatus, FrameDecoder, Msg, MAX_FRAME};

/// Feed every byte of `frame[..len]` into a fresh decoder; return the final status.
fn decode_all(frame: &[u8], len: usize) -> DecodeStatus {
    let mut dec = FrameDecoder::new();
    let mut status = DecodeStatus::NeedMore;
    for &b in &frame[..len] {
        status = dec.push(b);
    }
    status
}

/// Encode `msg` with the given `seq` and decode it byte-by-byte.
fn roundtrip(msg: Msg, seq: u8) {
    let mut buf = [0u8; MAX_FRAME];
    let len = encode(&msg, seq, &mut buf).expect("encode failed");
    match decode_all(&buf, len) {
        DecodeStatus::Ready {
            msg: got,
            seq: got_seq,
        } => {
            assert_eq!(got, msg, "msg mismatch");
            assert_eq!(got_seq, seq, "seq mismatch");
        }
        DecodeStatus::Error(e) => panic!("decode error: {e:?}"),
        DecodeStatus::NeedMore => panic!("decoder never completed"),
    }
}

// ---- roundtrip for every variant -------------------------------------------

#[test]
fn rt_dsky_write_row() {
    roundtrip(
        Msg::DskyWriteRow {
            row: 3,
            data: 0xABCD,
        },
        0x01,
    );
}
#[test]
fn rt_dsky_clear_row() {
    roundtrip(Msg::DskyClearRow { row: 7 }, 0x02);
}
#[test]
fn rt_dsky_set_lamp() {
    roundtrip(Msg::DskySetLamp { lamp: 2, on: 1 }, 0x03);
}
#[test]
fn rt_dsky_set_flash() {
    roundtrip(Msg::DskySetFlash { on: 0 }, 0x04);
}
#[test]
fn rt_optics_drive() {
    roundtrip(
        Msg::OpticsDrive {
            trunnion: -100,
            shaft: 200,
        },
        0x05,
    );
}
#[test]
fn rt_engine_sps_enable() {
    roundtrip(Msg::EngineSpsEnable { on: 1 }, 0x06);
}
#[test]
fn rt_engine_sps_gimbal() {
    roundtrip(
        Msg::EngineSpsGimbal {
            pitch: -32768,
            yaw: 32767,
        },
        0x07,
    );
}
#[test]
fn rt_rcs_fire_sm() {
    roundtrip(
        Msg::RcsFireSm {
            jets_a: 0xFF,
            jets_b: 0x0F,
        },
        0x08,
    );
}
#[test]
fn rt_rcs_fire_cm() {
    roundtrip(Msg::RcsFireCm { jets: 0x0FFF }, 0x09);
}
#[test]
fn rt_rcs_quench_all() {
    roundtrip(Msg::RcsQuenchAll, 0x0A);
}
#[test]
fn rt_telemetry_word() {
    roundtrip(Msg::TelemetryWord { word: 0x1234 }, 0x0B);
}
#[test]
fn rt_agc_heartbeat() {
    roundtrip(
        Msg::AgcHeartbeat {
            mission_time_cs: 0x0102_0304,
        },
        0x0C,
    );
}
#[test]
fn rt_dsky_key() {
    roundtrip(
        Msg::DskyKey {
            code: 0x1F,
            dsky: 0x00,
        },
        0x0D,
    );
}
#[test]
fn rt_optics_cdu() {
    roundtrip(
        Msg::OpticsCdu {
            trunnion: 0xAAAAu16 as i16,
            shaft: 0x5555,
        },
        0x0E,
    );
}
#[test]
fn rt_optics_mark() {
    roundtrip(Msg::OpticsMark, 0x0F);
}
#[test]
fn rt_engine_thrust_on() {
    roundtrip(Msg::EngineThrustOn { on: 1 }, 0x10);
}
#[test]
fn rt_uplink_word() {
    roundtrip(Msg::UplinkWord { word: 0xBEEF }, 0x11);
}
#[test]
fn rt_bridge_heartbeat() {
    roundtrip(
        Msg::BridgeHeartbeat {
            uptime_ms: 0xDEAD_BEEF,
        },
        0x12,
    );
}
#[test]
fn rt_hello() {
    roundtrip(Msg::Hello { proto_version: 1 }, 0x13);
}
#[test]
fn rt_hello_ack() {
    roundtrip(Msg::HelloAck { proto_version: 1 }, 0x14);
}
#[test]
fn rt_error() {
    roundtrip(
        Msg::Error {
            code: 0x01,
            ctx: 0x02,
        },
        0x15,
    );
}

// ---- sequence numbers ------------------------------------------------------

#[test]
fn seq_wraps_zero_and_max() {
    roundtrip(Msg::RcsQuenchAll, 0x00);
    roundtrip(Msg::RcsQuenchAll, 0xFF);
}

// ---- CRC corruption --------------------------------------------------------

#[test]
fn crc_corruption_returns_bad_crc_and_next_frame_still_decodes() {
    let msg = Msg::TelemetryWord { word: 0xCAFE };
    let seq = 0x42u8;
    let mut buf = [0u8; MAX_FRAME];
    let len = encode(&msg, seq, &mut buf).unwrap();

    // Flip a payload byte (index 4 = first payload byte).
    buf[4] ^= 0xFF;

    let mut dec = FrameDecoder::new();
    let mut last = DecodeStatus::NeedMore;
    for &b in &buf[..len] {
        last = dec.push(b);
    }
    assert!(
        matches!(last, DecodeStatus::Error(DecodeError::BadCrc)),
        "expected BadCrc, got something else"
    );

    // After the error the decoder must accept a fresh valid frame.
    let msg2 = Msg::DskySetFlash { on: 1 };
    let seq2 = 0x43u8;
    let mut buf2 = [0u8; MAX_FRAME];
    let len2 = encode(&msg2, seq2, &mut buf2).unwrap();
    let result = decode_all(&buf2, len2);
    assert!(
        matches!(result, DecodeStatus::Ready { .. }),
        "decoder did not recover after BadCrc"
    );
}

// ---- trailing garbage ------------------------------------------------------

#[test]
fn trailing_garbage_is_skipped() {
    let msg = Msg::AgcHeartbeat {
        mission_time_cs: 999,
    };
    let seq = 0x20u8;
    let mut frame = [0u8; MAX_FRAME];
    let len = encode(&msg, seq, &mut frame).unwrap();

    let mut dec = FrameDecoder::new();
    // Two garbage bytes before the frame.
    dec.push(0xAB);
    dec.push(0xCD);

    let mut last = DecodeStatus::NeedMore;
    for &b in &frame[..len] {
        last = dec.push(b);
    }
    assert!(
        matches!(
            last,
            DecodeStatus::Ready {
                msg: Msg::AgcHeartbeat {
                    mission_time_cs: 999
                },
                seq: 0x20
            }
        ),
        "expected ready with heartbeat"
    );
}

// ---- truncated frame followed by valid frame -------------------------------

#[test]
fn truncated_then_valid_frame() {
    let msg1 = Msg::DskyWriteRow {
        row: 1,
        data: 0x1234,
    };
    let msg2 = Msg::DskyWriteRow {
        row: 2,
        data: 0x5678,
    };
    let mut buf1 = [0u8; MAX_FRAME];
    let mut buf2 = [0u8; MAX_FRAME];
    let len1 = encode(&msg1, 1, &mut buf1).unwrap();
    let len2 = encode(&msg2, 2, &mut buf2).unwrap();

    let mut dec = FrameDecoder::new();

    // Feed only the first half of frame1 (truncated — no CRC).
    for &b in &buf1[..len1 / 2] {
        dec.push(b);
    }

    // Now feed the complete second frame.
    let mut last = DecodeStatus::NeedMore;
    for &b in &buf2[..len2] {
        last = dec.push(b);
    }

    assert!(
        matches!(
            last,
            DecodeStatus::Ready {
                msg: Msg::DskyWriteRow {
                    row: 2,
                    data: 0x5678
                },
                seq: 2
            }
        ),
        "expected second frame after truncated first"
    );
}
