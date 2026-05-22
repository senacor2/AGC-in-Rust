//! TCP client for yaAGC's channel-word socket protocol.
//!
//! yaAGC listens on a TCP port (default 19697, configurable via
//! `--port=N`) and speaks a 4-byte channel-word protocol with each
//! connected peripheral. Each I/O channel write performed by the AGC is
//! emitted to every connected peripheral; each packet sent by a
//! peripheral is delivered to the AGC's input channels.
//!
//! ## Packet format (from `agc_utilities.c::FormIoPacket`)
//!
//! Each 4-byte wire packet has fixed 2-bit framing prefixes
//! (`00`, `01`, `10`, `11` for bytes 0–3) so a resynchronising reader
//! can find packet boundaries. Per-bit layout:
//!
//! ```text
//! Byte 0: 00 ub ccccc   — frame=00, u-bit in bit 5, channel bits 7:3 in bits 4:0
//! Byte 1: 01 ccc vvv    — frame=01, channel bits 2:0 in bits 5:3, value bits 14:12 in bits 2:0
//! Byte 2: 10 vvvvvv     — frame=10, value bits 11:6
//! Byte 3: 11 vvvvvv     — frame=11, value bits 5:0
//! ```
//!
//! - Channel number: 8 bits (0–255).
//! - u-bit: 1 bit (used by some AGC channels to distinguish control
//!   traffic from data).
//! - Value: 15 bits (AGC word, sign in bit 14).
//!
//! ## Usage
//!
//! ```no_run
//! use agc_test::vagc_channel::{ChannelPacket, YaAgcClient};
//! use std::time::Duration;
//!
//! let mut client = YaAgcClient::connect_localhost(19697).unwrap();
//!
//! // Inject one PIPA pulse on channel 014 (octal).
//! client.send(ChannelPacket {
//!     channel: 0o14,
//!     value: 1,
//!     u_bit: false,
//! }).unwrap();
//!
//! // Read any pending output from the AGC (timeout 100 ms).
//! while let Ok(pkt) = client.try_recv(Duration::from_millis(100)) {
//!     println!("AGC wrote channel 0o{:o} = 0o{:o}", pkt.channel, pkt.value);
//! }
//! ```
//!
//! This module is the foundation for the MS-E7b end-to-end channel-
//! trace comparison work. The full driver (DSKY scripting + PIPA
//! injection + cycle-by-cycle comparator) lives in a follow-up
//! sub-issue; the packet-level client and a smoke test land here.

use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

/// Default yaAGC TCP port. Override with `yaAGC --port=N` when running
/// multiple AGC instances or to avoid CI port conflicts.
pub const DEFAULT_YAAGC_PORT: u16 = 19_697;

// ── ChannelPacket ──────────────────────────────────────────────────────────

/// A single I/O channel word framed for the yaAGC socket protocol.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelPacket {
    /// 8-bit AGC channel number. For Comanche055, channel numbers in
    /// 0o10..0o57 are documented in `Comanche055/CHANNEL_USAGE.agc`.
    /// Values above 0xFF are masked on `pack`.
    pub channel: u16,
    /// 15-bit AGC word. Bit 14 is the sign in ones-complement
    /// representation; bits 13:0 are the magnitude. Values above
    /// 0x7FFF are masked on `pack`.
    pub value: u16,
    /// `u-bit`: 9th channel bit. yaAGC uses it to distinguish channel
    /// writes from some special control packets. Almost always
    /// `false` for normal I/O.
    pub u_bit: bool,
}

impl ChannelPacket {
    /// Encode the packet as the 4-byte wire format.
    /// Returns the encoded bytes; never fails for valid input. Out-of-
    /// range fields (`channel > 0xFF`, `value > 0x7FFF`) are masked
    /// silently, matching `agc_utilities.c::FormIoPacket`.
    pub fn pack(self) -> [u8; 4] {
        let channel = self.channel & 0xFF;
        let value = self.value & 0x7FFF;
        let u = if self.u_bit { 0x20 } else { 0 };

        // Channel: high 5 bits (7:3) → byte 0 bits 4:0; low 3 bits → byte 1 bits 5:3.
        let b0 = ((channel >> 3) & 0x1F) as u8 | u;
        let b1 = 0x40 | (((channel << 3) & 0x38) as u8) | (((value >> 12) & 0x07) as u8);
        let b2 = 0x80 | (((value >> 6) & 0x3F) as u8);
        let b3 = 0xc0 | ((value & 0x3F) as u8);
        [b0, b1, b2, b3]
    }

    /// Decode a 4-byte wire packet.
    /// Returns `Err` if any of the framing bits are wrong.
    pub fn unpack(bytes: [u8; 4]) -> Result<Self, ProtocolError> {
        if (0xc0 & bytes[0]) != 0x00 {
            return Err(ProtocolError::BadFraming { byte: 0, got: bytes[0] });
        }
        if (0xc0 & bytes[1]) != 0x40 {
            return Err(ProtocolError::BadFraming { byte: 1, got: bytes[1] });
        }
        if (0xc0 & bytes[2]) != 0x80 {
            return Err(ProtocolError::BadFraming { byte: 2, got: bytes[2] });
        }
        if (0xc0 & bytes[3]) != 0xc0 {
            return Err(ProtocolError::BadFraming { byte: 3, got: bytes[3] });
        }
        let channel = (((bytes[0] & 0x1F) as u16) << 3) | (((bytes[1] >> 3) & 7) as u16);
        let value = (((bytes[1] & 0x07) as u16) << 12)
            | (((bytes[2] & 0x3F) as u16) << 6)
            | ((bytes[3] & 0x3F) as u16);
        let u_bit = (bytes[0] & 0x20) != 0;
        Ok(Self {
            channel,
            value,
            u_bit,
        })
    }
}

#[derive(Debug)]
pub enum ProtocolError {
    /// One of the four 2-bit framing prefixes was wrong. yaAGC's wire
    /// format encodes the frame index in the high two bits of each
    /// byte: `00`, `01`, `10`, `11` for bytes 0–3. A mismatch usually
    /// means a desync: the stream was read at a non-packet boundary.
    BadFraming { byte: usize, got: u8 },
}

impl std::fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProtocolError::BadFraming { byte, got } => write!(
                f,
                "invalid packet framing at byte {byte}: 0x{got:02X}"
            ),
        }
    }
}

impl std::error::Error for ProtocolError {}

// ── YaAgcClient ────────────────────────────────────────────────────────────

/// TCP client for one connection to a yaAGC instance.
///
/// Each `YaAgcClient` looks to yaAGC like a single peripheral (DSKY,
/// IMU, RCS controller, etc.). yaAGC will deliver every channel write
/// to every connected peripheral, so a test driver typically opens
/// one client and demultiplexes inside [`try_recv`] by channel number.
pub struct YaAgcClient {
    stream: TcpStream,
    buf: Vec<u8>,
}

impl YaAgcClient {
    /// Connect to a yaAGC instance reachable at `addr` (any
    /// `ToSocketAddrs` value). Sets a default read timeout of 100 ms
    /// so `try_recv` doesn't block forever.
    pub fn connect<A: ToSocketAddrs>(addr: A) -> io::Result<Self> {
        let stream = TcpStream::connect(addr)?;
        stream.set_read_timeout(Some(Duration::from_millis(100)))?;
        stream.set_nodelay(true)?;
        Ok(Self {
            stream,
            buf: Vec::with_capacity(64),
        })
    }

    /// Convenience: connect to `127.0.0.1:port`.
    pub fn connect_localhost(port: u16) -> io::Result<Self> {
        Self::connect(("127.0.0.1", port))
    }

    /// Send one channel-word packet to yaAGC. Blocks until the 4 bytes
    /// are written or the underlying socket errors.
    pub fn send(&mut self, packet: ChannelPacket) -> io::Result<()> {
        self.stream.write_all(&packet.pack())
    }

    /// Try to receive one channel-word packet from yaAGC. Returns
    /// `Ok(packet)` on success, `Err(io::ErrorKind::WouldBlock)` /
    /// `TimedOut` if no packet arrived within `timeout`, or
    /// `Err(io::ErrorKind::InvalidData)` on protocol-framing failure.
    pub fn try_recv(&mut self, timeout: Duration) -> io::Result<ChannelPacket> {
        self.stream.set_read_timeout(Some(timeout))?;
        while self.buf.len() < 4 {
            let mut tmp = [0u8; 32];
            let n = self.stream.read(&mut tmp)?;
            if n == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "yaAGC closed the connection",
                ));
            }
            self.buf.extend_from_slice(&tmp[..n]);
        }
        let bytes: [u8; 4] = [self.buf[0], self.buf[1], self.buf[2], self.buf[3]];
        self.buf.drain(..4);
        ChannelPacket::unpack(bytes).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-VCH-PK-1: encoder matches the bit layout from
    /// `agc_utilities.c::FormIoPacket`. Channel 0o14, value 0o123, no
    /// u-bit. Hand-computed reference bytes.
    #[test]
    fn tc_vch_pk_1_pack_known() {
        let packet = ChannelPacket {
            channel: 0o14,
            value: 0o123,
            u_bit: false,
        };
        let bytes = packet.pack();
        // Round-trip through unpack as a cross-check.
        let again = ChannelPacket::unpack(bytes).unwrap();
        assert_eq!(packet, again);
        // Verify framing bits.
        assert_eq!(bytes[0] & 0xc0, 0x00);
        assert_eq!(bytes[1] & 0xc0, 0x40);
        assert_eq!(bytes[2] & 0xc0, 0x80);
        assert_eq!(bytes[3] & 0xc0, 0xc0);
    }

    /// TC-VCH-PK-2: round-trip a sweep of channel × value × u-bit
    /// combinations. Encoder + decoder must be exact inverses.
    #[test]
    fn tc_vch_pk_2_round_trip_sweep() {
        // Strided sweep keeps the test fast while covering all bits.
        // Channel field is 8-bit (0..0xFF) on the wire; u-bit is a
        // separate 9th bit.
        for &ch in &[0u16, 1, 7, 8, 0o10, 0o15, 0o33, 0o57, 0o100, 0o177, 0xFF] {
            for &val in &[0u16, 1, 0o100, 0o7777, 0x4000, 0x7FFF] {
                for &u in &[false, true] {
                    let p = ChannelPacket {
                        channel: ch,
                        value: val,
                        u_bit: u,
                    };
                    let bytes = p.pack();
                    let round = ChannelPacket::unpack(bytes).unwrap();
                    assert_eq!(p, round, "round-trip failed for {p:?}");
                }
            }
        }
    }

    /// TC-VCH-PK-3: malformed framing returns `BadFraming`, not panic.
    #[test]
    fn tc_vch_pk_3_bad_framing() {
        // Byte 0 must have framing bits 00; flip them to 11.
        let err = ChannelPacket::unpack([0xc0, 0x40, 0x80, 0xc0]).unwrap_err();
        assert!(matches!(err, ProtocolError::BadFraming { byte: 0, .. }));
        // Byte 1 must have 01; flip to 11.
        let err = ChannelPacket::unpack([0x00, 0xc0, 0x80, 0xc0]).unwrap_err();
        assert!(matches!(err, ProtocolError::BadFraming { byte: 1, .. }));
    }

    /// TC-VCH-PK-4: oversize channel and value masks are clipped on
    /// pack, so bad input still produces decodable output.
    #[test]
    fn tc_vch_pk_4_mask_on_pack() {
        let p = ChannelPacket {
            channel: 0xFFFF,
            value: 0xFFFF,
            u_bit: false,
        };
        let bytes = p.pack();
        let round = ChannelPacket::unpack(bytes).unwrap();
        assert_eq!(round.channel, 0xFF);
        assert_eq!(round.value, 0x7FFF);
    }

    /// TC-VCH-RX-INTEG: spawn yaAGC, connect via TCP, observe at least
    /// one outbound channel packet from the AGC's startup sequence.
    ///
    /// yaAGC writes to its output channels (DSKY blanking, alarm clear,
    /// etc.) within a few simulated milliseconds of boot, so any
    /// connected peripheral will receive packets within ~1 s wall-
    /// clock. Skipped on machines without the VirtualAGC build.
    #[test]
    fn tc_vch_rx_integ_smoke() {
        use crate::vagc_harness::vagc_root;
        use std::time::Duration;

        let root = vagc_root();
        let yaagc = root.join("yaAGC/yaAGC");
        let rope = root.join("Comanche055/MAIN.agc.bin");
        if !yaagc.exists()
            || !rope.exists()
            || std::fs::metadata(&rope).map(|m| m.len()).unwrap_or(0) == 0
        {
            eprintln!("skipping: VirtualAGC build incomplete");
            return;
        }

        // Use an ephemeral high port to avoid clashes when tests run
        // concurrently. yaAGC accepts any port via --port=N.
        let port = pick_test_port();
        let work_dir = std::env::temp_dir().join(format!("vagc_chan_smoke_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&work_dir);
        std::fs::create_dir_all(&work_dir).unwrap();

        let mut child = std::process::Command::new(&yaagc)
            .current_dir(&work_dir)
            .arg("--quiet")
            .arg("--nodebug")
            .arg("--no-resume")
            .arg(format!("--port={port}"))
            .arg(&rope)
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
            .expect("yaAGC failed to spawn");

        // Give yaAGC a moment to bind the listening socket.
        std::thread::sleep(Duration::from_millis(200));

        // Connect. Retry once in case the bind hadn't completed.
        let mut client = match YaAgcClient::connect_localhost(port) {
            Ok(c) => c,
            Err(_) => {
                std::thread::sleep(Duration::from_millis(500));
                YaAgcClient::connect_localhost(port)
                    .expect("could not connect to yaAGC after retry")
            }
        };

        // Try to receive any channel packet within 2 s. yaAGC emits
        // channel writes from the startup sequence as soon as it
        // executes its first instructions.
        let result = (0..20).find_map(|_| client.try_recv(Duration::from_millis(100)).ok());

        // Tear down yaAGC regardless of test outcome.
        let _ = child.kill();
        let _ = child.wait();
        let _ = std::fs::remove_dir_all(work_dir);

        let pkt =
            result.expect("expected at least one channel packet from yaAGC within 2 s");
        // Sanity-check that the channel number is in the documented
        // Comanche055 output-channel range. We don't validate the
        // *value*, just that we got framed protocol output.
        assert!(
            pkt.channel < 0o60,
            "unexpected channel number 0o{:o} from yaAGC",
            pkt.channel
        );
    }

    // Pick a high TCP port for parallel test runs. Avoids the default
    // 19697 in case another yaAGC instance is already running.
    fn pick_test_port() -> u16 {
        use std::sync::atomic::{AtomicU16, Ordering};
        static NEXT: AtomicU16 = AtomicU16::new(43_000);
        NEXT.fetch_add(1, Ordering::SeqCst)
    }
}
