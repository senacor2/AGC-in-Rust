//! MS-E7c channel-trace integration tests.
//!
//! Three tiers:
//!
//! 1. **`tc_e7c_fixture_load_smoke`** — Rust-only. Always runs. Loads
//!    a committed JSON trace fixture, exercises the recorder/
//!    comparator API end-to-end against itself. Validates the on-disk
//!    fixture format and the comparator's "identical traces match"
//!    invariant without needing a local VirtualAGC build.
//!
//! 2. **`tc_e7c_vagc_recorder_startup`** — `VAGC_AVAILABLE`-gated.
//!    Spawns yaAGC, drains its boot-time channel writes into a
//!    [`ChannelTraceRecorder`], saves the result as JSON, and reloads
//!    it. Asserts the recorder produced a parseable trace with at
//!    least some output channel activity.
//!
//! 3. **`tc_e7c_vagc_dsky_keypress`** — `VAGC_AVAILABLE`-gated.
//!    Spawns yaAGC, connects two clients (sender + recorder), sends
//!    V35 ENTR (DSKY lamp test) via [`DskyScript`], and asserts the
//!    recorder captured the keypress packet on channel `0o15`. (V35
//!    is the AGC's lamp-test diagnostic verb — safe to issue in P00
//!    idle and visible in the channel-015 echo back to every
//!    peripheral.)
//!
//! The full `entry_direct_leo` / `entry_lunar_return` end-to-end drive
//! through yaAGC is **not** included in this milestone — it requires a
//! pre-staged AGC erasable state (REFSMMAT + state vector + target
//! site) that has no harness today. That work is the natural follow-on
//! milestone; see [`docs/entry_channel_trace.md`].

use std::path::PathBuf;
use std::time::{Duration, Instant};

use agc_test::vagc_channel::YaAgcClient;
use agc_test::vagc_driver::{DskyScript, CHAN_KEYIN};
use agc_test::vagc_harness::vagc_root;
use agc_test::vagc_trace::{compare, ChannelTrace, ChannelTraceRecorder, CompareTolerance};

/// Path to the committed smoke fixture used by the Rust-only test.
fn fixture_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures")
        .join("entry")
        .join("channel_traces")
        .join("entry_startup_smoke.json")
}

/// Spawn yaAGC for an integration test. Returns the child process and
/// the TCP port it's bound to, or `None` if the VirtualAGC build is
/// missing (caller should skip).
fn spawn_yaagc(port: u16) -> Option<(std::process::Child, std::path::PathBuf)> {
    let root = vagc_root();
    let yaagc = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");
    if !yaagc.exists()
        || !rope.exists()
        || std::fs::metadata(&rope).map(|m| m.len()).unwrap_or(0) == 0
    {
        eprintln!(
            "skipping: VirtualAGC build incomplete at {} (run agc-test/scripts/assemble_comanche055.sh)",
            root.display()
        );
        return None;
    }

    let work_dir = std::env::temp_dir().join(format!(
        "vagc_e7c_{}_{}_{}",
        port,
        std::process::id(),
        rand_suffix()
    ));
    let _ = std::fs::remove_dir_all(&work_dir);
    std::fs::create_dir_all(&work_dir).unwrap();

    let child = std::process::Command::new(&yaagc)
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
    Some((child, work_dir))
}

/// Allocate a unique high TCP port for parallel test runs.
fn pick_test_port() -> u16 {
    use std::sync::atomic::{AtomicU16, Ordering};
    static NEXT: AtomicU16 = AtomicU16::new(44_000);
    NEXT.fetch_add(1, Ordering::SeqCst)
}

/// Best-effort random suffix without pulling in `rand`. Uses the
/// nanosecond fraction of the current time, which is good enough to
/// keep parallel test work-dirs from colliding.
fn rand_suffix() -> u128 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0)
}

/// Connect with one retry to forgive yaAGC's bind/listen race.
fn connect(port: u16) -> YaAgcClient {
    match YaAgcClient::connect_localhost(port) {
        Ok(c) => c,
        Err(_) => {
            std::thread::sleep(Duration::from_millis(500));
            YaAgcClient::connect_localhost(port).expect("could not connect to yaAGC after retry")
        }
    }
}

/// Tear down a spawned yaAGC.
fn kill(mut child: std::process::Child, work_dir: std::path::PathBuf) {
    let _ = child.kill();
    let _ = child.wait();
    let _ = std::fs::remove_dir_all(work_dir);
}

// ── 1. Rust-only fixture smoke ────────────────────────────────────────────

/// TC-E7C-FIX-1: load the committed channel-trace fixture, exercise
/// the comparator pipeline against it, and verify structural
/// invariants. Always runs — no yaAGC needed.
#[test]
fn tc_e7c_fixture_load_smoke() {
    let path = fixture_path();
    let trace = ChannelTrace::load(&path)
        .unwrap_or_else(|e| panic!("could not load fixture {}: {}", path.display(), e));

    assert!(
        !trace.scenario.is_empty(),
        "fixture scenario field is empty"
    );
    assert!(
        !trace.events.is_empty(),
        "fixture has no events — write at least one event so the comparator path is exercised"
    );

    // Monotonic timestamps — the recorder appends in chronological
    // order so any out-of-order fixture is a data error.
    let mut last_t = 0u32;
    for ev in &trace.events {
        assert!(
            ev.t_ms >= last_t,
            "fixture timestamps must be monotonically non-decreasing (got {} after {})",
            ev.t_ms,
            last_t
        );
        last_t = ev.t_ms;
    }

    // Self-compare must produce an exact match under default tolerance.
    let report = compare(&trace, &trace, &CompareTolerance::default());
    assert!(
        report.is_match(),
        "fixture should self-match under default tolerance; got differences: {:?}",
        report.differences
    );
}

// ── 2. yaAGC startup smoke ────────────────────────────────────────────────

/// TC-E7C-VAGC-REC-1: capture yaAGC's startup channel writes into a
/// [`ChannelTraceRecorder`], round-trip through JSON, and assert the
/// resulting trace has plausible content.
#[test]
fn tc_e7c_vagc_recorder_startup() {
    let port = pick_test_port();
    let Some((child, work_dir)) = spawn_yaagc(port) else {
        return; // VirtualAGC not available — skip.
    };

    std::thread::sleep(Duration::from_millis(200));
    let client = connect(port);
    let mut recorder = ChannelTraceRecorder::new(client);

    // Capture 1.5 s of startup output. yaAGC emits many channel writes
    // during PINBALL's first lamp-blanking pass, so this should land
    // dozens of events on a healthy build.
    let deadline = Instant::now() + Duration::from_millis(1500);
    while Instant::now() < deadline {
        recorder.drain(Duration::from_millis(100));
    }

    let captured = recorder.len();
    let trace = recorder.into_trace(
        "vagc_startup",
        format!("yaAGC startup capture from {}", vagc_root().display()),
    );

    kill(child, work_dir);

    // Round-trip through JSON to validate the on-disk format.
    let tmp_path = std::env::temp_dir().join(format!(
        "e7c_startup_{}_{}.json",
        std::process::id(),
        rand_suffix()
    ));
    trace.save(&tmp_path).expect("save trace");
    let reloaded = ChannelTrace::load(&tmp_path).expect("reload trace");
    let _ = std::fs::remove_file(&tmp_path);

    assert_eq!(reloaded, trace, "trace did not survive JSON round trip");
    assert!(
        captured >= 5,
        "expected at least a handful of startup writes, got {captured}"
    );
    // Every captured event must be on a documented channel range. The
    // upper bound is generous (yaAGC's NUM_CHANNELS = 512, but valid
    // AGC output channels live in 0o00..0o57 plus the fictitious
    // 0o173 for INLINK).
    for ev in &trace.events {
        assert!(
            ev.channel < 0o200,
            "captured impossible channel 0o{:o} = {}",
            ev.channel,
            ev.channel
        );
        assert!(
            ev.value <= 0x7FFF,
            "value 0x{:X} exceeds the 15-bit AGC word",
            ev.value
        );
    }
}

// ── 3. DskyScript keypress smoke ──────────────────────────────────────────

/// TC-E7C-VAGC-DSKY-1: send `V35 ENTR` via [`DskyScript`] and capture
/// the resulting channel-015 echo on a second client.
///
/// yaAGC delivers every channel write — including writes that came from
/// a peripheral itself — to every connected peripheral. So the
/// recorder's view of the keypress confirms that:
/// - The `DskyScript` produced a well-formed packet.
/// - yaAGC accepted and acknowledged it.
/// - The packet was framed correctly enough for the recorder to
///   decode it back out.
///
/// V35 is the AGC lamp-test diagnostic verb; it requires no prior
/// state and is safe to issue while the AGC is idle in P00.
#[test]
fn tc_e7c_vagc_dsky_keypress() {
    let port = pick_test_port();
    let Some((child, work_dir)) = spawn_yaagc(port) else {
        return;
    };

    std::thread::sleep(Duration::from_millis(200));

    // Two parallel connections — yaAGC echoes the keypress back to
    // both, so we can observe what we just sent.
    let sender_client = connect(port);
    let recorder_client = connect(port);
    let mut sender = DskyScript::new(sender_client);
    let mut recorder = ChannelTraceRecorder::new(recorder_client);

    // Drain ~500 ms of startup so the AGC has finished its boot
    // sequence before we start typing. Without this the recorder's
    // capture is dominated by lamp-blanking writes and the keypress
    // gets lost in the noise of TC-VAGC-REC-1.
    let warmup = Instant::now() + Duration::from_millis(500);
    while Instant::now() < warmup {
        recorder.drain(Duration::from_millis(50));
    }
    let pre_keypress_len = recorder.len();

    // Type V35E. Each press is one channel-015 packet.
    sender.verb(35).expect("send V35");
    sender.enter().expect("send ENTR");

    // Drain another 500 ms — long enough for the keypress packets to
    // round-trip and for the AGC to respond.
    let deadline = Instant::now() + Duration::from_millis(500);
    while Instant::now() < deadline {
        recorder.drain(Duration::from_millis(50));
    }

    let new_events = &recorder.events()[pre_keypress_len..];
    let key_events: Vec<_> = new_events
        .iter()
        .filter(|e| e.channel == CHAN_KEYIN)
        .collect();

    kill(child, work_dir);

    // We sent V (0o21), digit-3 (0o03), digit-5 (0o05), ENTR (0o34).
    // The recorder should see all four codes.
    assert!(
        key_events.len() >= 4,
        "expected ≥4 channel-015 echoes after V35E, got {} (new events: {:?})",
        key_events.len(),
        new_events
    );
    let codes: Vec<u16> = key_events.iter().map(|e| e.value).collect();
    for expected in [0o21, 0o03, 0o05, 0o34] {
        assert!(
            codes.contains(&expected),
            "expected keycode 0o{:o} in captured echoes {:?}",
            expected,
            codes
        );
    }
}
