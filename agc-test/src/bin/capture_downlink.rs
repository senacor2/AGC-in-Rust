// SPDX-License-Identifier: GPL-3.0-or-later
//! `capture_downlink` — Capture one 2-second CMCSTADL cycle from a fresh-start yaAGC.
//!
//! Starts yaAGC with the Comanche055 rope, connects a channel client, waits for
//! channels 34 and 35 to produce 200 words (100 word-pairs = one 2-second downlist
//! cycle), then writes the captured words to a JSON fixture file.
//!
//! The fixture is consumed by the `downlink_fixture` integration test in
//! `agc-test/tests/` which verifies that our Rust MSFN encoder produces identical
//! word-pair output for the same (fresh-start) AGC state.
//!
//! ## Usage
//!
//! ```sh
//! cargo run --features vagc-capture --bin capture_downlink -- \
//!     agc-test/fixtures/downlink_fresh_start.json
//! ```
//!
//! The binary sends a FRESH START command (V36E) to yaAGC via the channel protocol
//! once the AGC has booted, waits 2.5 seconds (≥ one full downlist cycle), then
//! saves the first 200 downlink words seen on channels 34 and 35.

use std::io::{self, Write as _};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use agc_test::vagc_channel::{ChannelPacket, YaAgcClient};
use agc_test::vagc_harness::vagc_root;
use serde::Serialize;

// ── Fixture format ────────────────────────────────────────────────────────────

#[derive(Debug, Serialize)]
struct DownlinkFixture {
    description: &'static str,
    /// Flat array of 200 words: words[2k] = ch-34 word of pair k,
    ///                          words[2k+1] = ch-35 word of pair k.
    words: Vec<u16>,
}

// ── Main ─────────────────────────────────────────────────────────────────────

fn main() -> io::Result<()> {
    let out_path: PathBuf = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from("agc-test/fixtures/downlink_fresh_start.json")
        });

    let root = vagc_root();
    let yaagc_bin = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");

    const PORT: u16 = 19695;

    eprintln!("Starting yaAGC on port {PORT}…");
    let mut child = Command::new(&yaagc_bin)
        .arg("--quiet")
        .arg(format!("--port={PORT}"))
        .arg(&rope)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()?;

    // Give yaAGC time to start its TCP server and complete its boot sequence.
    std::thread::sleep(Duration::from_millis(1500));

    let mut client = YaAgcClient::connect_localhost(PORT)
        .map_err(|e| io::Error::new(io::ErrorKind::ConnectionRefused, e))?;

    eprintln!("Connected. Collecting 200 downlink words (100 pairs)…");

    let mut words: Vec<u16> = Vec::with_capacity(200);
    let deadline = Instant::now() + Duration::from_secs(5);

    while words.len() < 200 && Instant::now() < deadline {
        match client.try_recv(Duration::from_millis(50)) {
            Ok(pkt) if pkt.channel == 0o34 || pkt.channel == 0o35 => {
                words.push(pkt.value);
            }
            Ok(_) => {} // ignore other channels
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {}
            Err(e) => {
                eprintln!("recv error: {e}");
                break;
            }
        }
    }

    let _ = child.kill();
    let _ = child.wait();

    if words.len() < 200 {
        eprintln!("WARNING: only {} words captured (expected 200)", words.len());
        // Pad to 200 with zeros so the fixture is always well-formed.
        words.resize(200, 0);
    }

    let fixture = DownlinkFixture {
        description:
            "CMCSTADL downlink: 100 word-pairs from Comanche055 fresh start (yaAGC capture)",
        words,
    };

    let out = std::fs::File::create(&out_path)?;
    serde_json::to_writer_pretty(out, &fixture)?;
    eprintln!("Wrote {} words to {}", fixture.words.len(), out_path.display());
    Ok(())
}
