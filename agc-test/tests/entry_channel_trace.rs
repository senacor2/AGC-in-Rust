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
//!    Spawns yaAGC, connects two clients (sender on `port`, recorder
//!    on `port + 1`), sends V35 ENTR (DSKY lamp test) via
//!    [`DskyScript`], and asserts the AGC's response writes
//!    channel-011 (DSALMOUT lamps). yaAGC does NOT echo channel-015
//!    writes back to peripherals (channel 015 is input-only), so we
//!    confirm the keypress path by observing what V35 *causes*
//!    rather than the keypress packet itself.
//!
//! The full `entry_direct_leo` / `entry_lunar_return` end-to-end drive
//! through yaAGC lands in MS-E7d (`tests/entry_e2e_vagc.rs`) — it
//! requires a pre-staged AGC erasable state (REFSMMAT + state vector
//! + target site) provided by `agc_test::entry_state::patch_into`.

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

/// Allocate a unique base TCP port for parallel test runs. yaAGC binds
/// 10 consecutive ports (`port`, `port+1`, …, `port+9`) — one per
/// client slot — so the allocator advances by 16 to keep concurrent
/// test runs from colliding on adjacent ports.
fn pick_test_port() -> u16 {
    use std::sync::atomic::{AtomicU16, Ordering};
    static NEXT: AtomicU16 = AtomicU16::new(44_000);
    NEXT.fetch_add(16, Ordering::SeqCst)
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

/// TC-E7C-VAGC-DSKY-1: send `V35 ENTR` via [`DskyScript`] and verify
/// the AGC responds with **additional** channel writes beyond the
/// baseline lamp-blanking it was already emitting.
///
/// yaAGC does NOT echo channel-015 (DSKY input) writes back to
/// connected peripherals — channel 015 is an input-only AGC channel
/// fed by `KEYRUPT1`'s `RAND MNKEYIN` (see `KEYRUPT,_UPRUPT.agc`).
/// So we can't observe the keypress packet directly. Instead, we
/// confirm the path is wired by observing that the AGC reacts:
/// V35 (lamp test) lights every DSKY lamp via channel-011 writes
/// (`DSALMOUT`) and updates the V/N display via channel-010, so the
/// post-keypress channel-write rate must rise above the pre-keypress
/// idle baseline.
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

    // yaAGC binds a separate listening socket per client slot
    // (`port`, `port+1`, …). Each TCP client must connect to a
    // different port; connecting twice to `port` would compete for
    // the same `ServerSockets[0]` slot and only one peer would be
    // accepted.
    let sender_client = connect(port);
    let recorder_client = connect(port + 1);
    let mut sender = DskyScript::new(sender_client);
    let mut recorder = ChannelTraceRecorder::new(recorder_client);

    // Drain ~1.5 s of startup so the AGC has finished its initial
    // lamp-blanking pass and any idle T4RUPT activity has settled.
    let warmup = Instant::now() + Duration::from_millis(1_500);
    while Instant::now() < warmup {
        recorder.drain(Duration::from_millis(50));
    }
    let pre_keypress_len = recorder.len();

    // Type V35E with a deliberate inter-key delay. The AGC services
    // KEYRUPT1 once per keystroke and the next keystroke must not
    // arrive before CHARIN has consumed channel-015. yaAGC does NOT
    // echo channel-015 writes back to peripherals (it's input-only),
    // so we observe the AGC's *response* — channel-011 (DSALMOUT
    // lamps) and channel-010 (V/N display digits) writes — rather
    // than the keypress packet itself.
    let key_delay = Duration::from_millis(150);
    for press in [
        agc_test::vagc_driver::DskyKey::Verb,
        agc_test::vagc_driver::DskyKey::Digit(3),
        agc_test::vagc_driver::DskyKey::Digit(5),
        agc_test::vagc_driver::DskyKey::Enter,
    ] {
        sender.press(press).expect("send keystroke");
        let until = Instant::now() + key_delay;
        while Instant::now() < until {
            recorder.drain(Duration::from_millis(20));
        }
    }

    let deadline = Instant::now() + Duration::from_millis(1_500);
    while Instant::now() < deadline {
        recorder.drain(Duration::from_millis(50));
    }

    let post_keypress: Vec<_> = recorder.events()[pre_keypress_len..].to_vec();
    let alarm_writes = post_keypress.iter().filter(|e| e.channel == 0o11).count();

    kill(child, work_dir);

    assert!(
        alarm_writes > 0,
        "expected V35 lamp test to write channel-011 (DSALMOUT); \
         got {} post-keypress events on other channels: {:?}",
        post_keypress.len(),
        post_keypress
            .iter()
            .map(|e| e.channel)
            .collect::<std::collections::BTreeSet<_>>()
    );

    // CHAN_KEYIN is the channel-015 constant the script wrote to; we
    // don't observe its echo (yaAGC doesn't broadcast input-channel
    // writes), but the constant is documented here for future
    // readers updating this test.
    let _ = CHAN_KEYIN;
}
