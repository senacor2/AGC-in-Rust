# External Peripheral Protocol

## Overview

Note: the IMU is local to the AGC MCU (BMI088 over SPI3) and is **not** routed through the bridge. All other peripherals are.

The AGC firmware (`agc-board-nucleo-f767`) communicates with external
peripherals (DSKY, sextant/optics, engines, RCS) through a bridge MCU over
USART6.  The wire protocol is defined in the `agc-protocol` crate
(`agc-protocol/src/`).

**Reference bridge implementation**: `agc-bridge-pico` — an RP2040 (Raspberry
Pi Pico) stub bridge that implements the full wire protocol, performs the
Hello/HelloAck handshake, forwards USB-console keystrokes as DSKY keys, and
emits synthetic heartbeat + OpticsCdu traffic.  See `agc-bridge-pico/README.md`
for wiring and flash instructions.

```
┌─────────────────┐  USART6 460800 8N1  ┌──────────────────────┐
│ Nucleo-F767ZI   │◄──────────────────►│ Raspberry Pi Pico     │
│ (AGC firmware)  │  PC6=TX / PC7=RX   │ (agc-bridge-pico)    │
└─────────────────┘  GPIO0=TX/GPIO1=RX └──────────────────────┘
                                                │  USB CDC-ACM
                                          developer console
                                     (future: DSKY / optics / engines / RCS)
```

---

## Wire Frame Format

Every message is wrapped in a frame:

```
┌──────┬─────┬─────┬───────┬─────────────────┬────────┬────────┐
│ STX  │ LEN │ SEQ │ TYPE  │ PAYLOAD (0–247B) │ CRC_LO │ CRC_HI │
│ 0xFE │  1B │  1B │  1B   │   LEN bytes      │   1B   │   1B   │
└──────┴─────┴─────┴───────┴─────────────────┴────────┴────────┘
```

| Field   | Size | Description                                            |
|---------|------|--------------------------------------------------------|
| STX     | 1    | Frame start sentinel — always `0xFE`                   |
| LEN     | 1    | Payload length in bytes (0–247)                        |
| SEQ     | 1    | Outbound sequence counter (wraps at 255→0)             |
| TYPE    | 1    | Message type byte (see table below)                    |
| PAYLOAD | LEN  | Message-specific payload (little-endian multi-byte)   |
| CRC_LO  | 1    | Low byte of CRC-16 over LEN+SEQ+TYPE+PAYLOAD          |
| CRC_HI  | 1    | High byte of CRC-16 (little-endian)                   |

Maximum frame size: 252 bytes.

### CRC Algorithm

CRC-16 (polynomial 0x1021, initial value 0x0000, no input/output reflection).
Covers bytes from LEN through end of PAYLOAD (STX and the two CRC bytes are
excluded).

### STX-in-Payload Caveat

If a payload byte happens to equal `0xFE` (the STX sentinel), the decoder
treats it as the start of a new frame and **drops the current frame**.
The CRC check will fail on the partial follow-on frame, which is then also
dropped.  Recovery occurs on the next valid STX.

**Mitigation**: the bridge firmware must implement application-level
acknowledgement and retransmission for safety-critical messages (RCS quench,
SPS enable/disable).  A payload byte equalling STX is statistically rare
(probability ≈ 1/256 per byte), but the bridge should log frame-drop events
for diagnostics.

---

## Message Types

### AGC → Bridge

| Type byte | Message          | Payload layout                               |
|-----------|------------------|----------------------------------------------|
| `0x10`    | DskyWriteRow     | row (u8), data (u16 LE)                      |
| `0x11`    | DskyClearRow     | row (u8)                                     |
| `0x12`    | DskySetLamp      | lamp (u8), on (u8: 0=off, 1=on)              |
| `0x13`    | DskySetFlash     | on (u8: 0=off, 1=on)                         |
| `0x20`    | OpticsDrive      | trunnion (i16 LE), shaft (i16 LE)            |
| `0x30`    | EngineSpsEnable  | on (u8: 0=cutoff, 1=ignition)                |
| `0x31`    | EngineSpsGimbal  | pitch (i16 LE), yaw (i16 LE)                 |
| `0x40`    | RcsFireSm        | jets_a (u8), jets_b (u8)                     |
| `0x41`    | RcsFireCm        | jets (u16 LE)                                |
| `0x42`    | RcsQuenchAll     | (no payload)                                 |
| `0x4A`    | TelemetryWord    | word (u16 LE)                                |
| `0x70`    | AgcHeartbeat     | mission_time_cs (u32 LE)                     |
| `0xE1`    | HelloAck         | proto_version (u8)                           |
| `0xEF`    | Error            | code (u8), ctx (u8)                          |

### Bridge → AGC

| Type byte | Message          | Payload layout                               |
|-----------|------------------|----------------------------------------------|
| `0x80`    | DskyKey          | code (u8: 5-bit AGC key), dsky (u8: 0=main)  |
| `0xA0`    | OpticsCdu        | trunnion (u16 LE), shaft (u16 LE)            |
| `0xA1`    | OpticsMark       | (no payload)                                 |
| `0xB0`    | EngineThrustOn   | on (u8: 0=off, 1=on)                         |
| `0xC0`    | UplinkWord       | word (u16 LE)                                |
| `0xD0`    | BridgeHeartbeat  | uptime_ms (u32 LE)                           |
| `0xE0`    | Hello            | proto_version (u8)                           |
| `0xEF`    | Error            | code (u8), ctx (u8)                          |

### Lamp Byte Encoding (DskySetLamp)

| Value | Lamp            |
|-------|-----------------|
| 0     | UplinkActivity  |
| 1     | NoAtt           |
| 2     | Stby            |
| 3     | KeyRel          |
| 4     | OprErr          |
| 5     | Restart         |
| 6     | GimbalLock      |
| 7     | Temp            |
| 8     | ProgAlarm       |
| 9     | CompActy        |

---

## Startup Handshake

1. Bridge sends `Hello { proto_version: 1 }`.
2. AGC sends `HelloAck { proto_version: 1 }`.

If versions do not match, the receiving side sends `Error { code: 0x01, ctx: proto_version }` and closes the link.

---

## Heartbeat Policy

- **AGC → bridge**: `AgcHeartbeat { mission_time_cs }` sent once per second
  from the idle loop.  Bridge firmware uses this to detect AGC liveness.
- **Bridge → AGC**: `BridgeHeartbeat { uptime_ms }` sent once per second.
  AGC firmware stores the last received `uptime_ms` in
  `BridgeState.last_bridge_heartbeat_ms`.  If the value is not updated for
  more than 3 seconds, downstream code may raise an alarm (not yet wired).

---

## RCS Jet Quench Semantics

The bridge firmware **must** implement a hardware-side jet-quench timeout.
If the bridge does not receive a new `RcsFireSm` or `RcsFireCm` message within
10 ms of the last fire command, it must automatically de-energise all jets
(`quench_all` locally).

Rationale: a dropped `RcsQuenchAll` frame (STX-in-payload or CRC failure)
would leave jets open indefinitely.  The bridge-side timeout is the
safety backstop independent of the link.

---

## DSKY Display Row Encoding

**Note**: this encoding is the project's own design — it is NOT the original AGC relay matrix (which packed 5 digits + sign across 12 relay coils per row in an SC-prefix scheme). The per-row, per-field layout is optimised for the bridge to render directly without any further decoding.

Each `DskyWriteRow { row, data }` message carries one logical DSKY field. There are 21 rows total (rows 0–20):

| Row | Field           | `data` encoding                                           |
|-----|-----------------|-----------------------------------------------------------|
| 0   | PROG            | bits 7–4 = tens digit, bits 3–0 = units digit            |
| 1   | VERB            | same as PROG                                             |
| 2   | NOUN            | same as PROG                                             |
| 3   | R1 sign         | 0 = blank, 1 = '+', 2 = '−'                              |
| 4   | R1 digit 0 (MS) | bits 3–0 = digit value 0x0–0x9; 0xF = blank              |
| 5   | R1 digit 1      | same                                                     |
| 6   | R1 digit 2      | same                                                     |
| 7   | R1 digit 3      | same                                                     |
| 8   | R1 digit 4 (LS) | same                                                     |
| 9   | R2 sign         | same as row 3                                            |
| 10  | R2 digit 0 (MS) | same as row 4                                            |
| 11  | R2 digit 1      | same                                                     |
| 12  | R2 digit 2      | same                                                     |
| 13  | R2 digit 3      | same                                                     |
| 14  | R2 digit 4 (LS) | same                                                     |
| 15  | R3 sign         | same as row 3                                            |
| 16  | R3 digit 0 (MS) | same as row 4                                            |
| 17  | R3 digit 1      | same                                                     |
| 18  | R3 digit 2      | same                                                     |
| 19  | R3 digit 3      | same                                                     |
| 20  | R3 digit 4 (LS) | same                                                     |

All 21 rows are emitted on every T4RUPT (every 120 ms) as an atomic snapshot of the current display state. Indicator lamps are NOT in the row stream — they travel through `DskySetLamp` messages (type `0x12`). The V/N flashing indicator travels through `DskySetFlash` (type `0x13`), also emitted once per T4RUPT immediately after the row stream.

**Rationale (ADR-019)**: A per-row design costs 21 frames per refresh versus the original relay matrix's 14, but it allows the bridge renderer to update individual fields without holding a complete display shadow. The encoding is trivially decodable in C, MicroPython, or any bridge firmware.

---

## Protocol Version

Current protocol version: **1** (constant `PROTO_VERSION` in `agc-protocol/src/lib.rs`).

---

## Bridge Firmware Quickref

Minimum bridge firmware must:

1. Open UART at **460800 baud, 8N1**.
2. On power-on, send `Hello { proto_version: 1 }`.
3. Await `HelloAck`; if not received within 5 s, retry.
4. On each DSKY keypress, send `DskyKey { code: <5-bit AGC code>, dsky: 0 }`.
5. Every 10 ms, send `OpticsCdu { trunnion, shaft }` with the latest CDU angles.
6. On optics mark button press, send `OpticsMark`.
7. On SPS thrust-on discrete change, send `EngineThrustOn { on }`.
8. On uplink word received, send `UplinkWord { word }`.
9. Every 1 s, send `BridgeHeartbeat { uptime_ms }`.
10. On `RcsFireSm` or `RcsFireCm`, energise the requested jets and start the
    10 ms hardware quench timer.
11. On `RcsQuenchAll` (or quench timer expiry), de-energise all jets.
12. On `EngineSpsEnable { on: 1 }`, close the SPS ignition relay.
13. On `EngineSpsGimbal { pitch, yaw }`, drive the TVC actuators.
14. On `DskyWriteRow`, `DskyClearRow`, `DskySetLamp`, `DskySetFlash`,
    update the DSKY display hardware accordingly.
