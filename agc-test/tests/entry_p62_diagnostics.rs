//! MS-E7i Experiment A — snapshot the AGC's parked state after
//! `V37 ENTR 62 ENTR` and again after `V33 ENTR`, to confirm or refute
//! the V33 / TESTNN / POODOO hypothesis from issue #45.
//!
//! Hypothesis (Comanche055 source citations):
//!
//! 1. V33 (decimal 27) < `LOWVERB` (28), so TESTVB at
//!    `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc:897-901` falls into TESTNN
//!    instead of dispatching to VBPROC via VERBFAN.
//! 2. `NOUNREG` is zero after the `V37 62 ENTR` sequence (REQMM zeros
//!    it at `:2845`). `NNADTAB[0] = OCT 00000  # NOT IN USE`
//!    (`PINBALL_NOUN_TABLES.agc:195`). TESTNN at `:911` does
//!    `CCS NNADTEM` with +0 and jumps to `TC GODSPALM`.
//! 3. `ENTRET` (= `DOTINC` erasable cell at
//!    `ERASABLE_ASSIGNMENTS.agc:1577,1580`) holds `TC NVSUBEND` after
//!    DISPLAY_INTERFACE's NVSUB call at `:3113`. DSPALARM at
//!    `:2770-2785` matches that and falls into `TC POODOO; OCT 01501`
//!    — a software restart that re-asserts `MODREG = 062`,
//!    `VERBREG = 045`.
//!
//! Experiment design:
//!
//! - Spawn yaAGC with `--dump-time=1` and `--inhibit-alarms`, stderr
//!   captured to a buffer.
//! - Drive `V37 ENTR 62 ENTR`, wait for the AGC to park at GOFLASH idle
//!   (~1.5 s wall-clock), then capture **Snapshot A** of erasable.
//! - Send `V33 ENTR`, wait for the next dump, then capture
//!   **Snapshot B**.
//! - Decode `DOTINC`, `NOUNREG`, `CADRSTOR`, `MODREG`, `VERBREG`,
//!   `MMNUMBER`, `REQRET`, `LOADSTAT`, and `FAILREG` (the alarm
//!   register) for both snapshots, plus the fixed-memory addresses of
//!   `ENDOFJOB` and `NVSUBEND` for off-line decoding of `DOTINC`.
//! - Scan captured yaAGC stderr for `01501` / `POODOO` / `ALARM` /
//!   `RESTART` markers.
//! - Serialize the full record as JSON under
//!   `agc-test/fixtures/entry/diagnostics/p62_parked_state.json` (or
//!   refresh it when `VAGC_CAPTURE=1`).
//!
//! Test gating: same pattern as the other MS-E7 live tests. Skips
//! cleanly when the VirtualAGC checkout, yaAGC binary, listing, or
//! `entry_template.core` fixture is missing.

use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use agc_test::entry_scenario::setup_state_direct_leo;
use agc_test::entry_state::{patch_into, EntryInitialState};
use agc_test::vagc_channel::YaAgcClient;
use agc_test::vagc_driver::DskyScript;
use agc_test::vagc_harness::{vagc_root, AgcAddress, CoreImage, Symtab};

use serde::{Deserialize, Serialize};

// ── Test gating helpers (parallel to entry_e2e_vagc.rs) ─────────────────────

fn fixtures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures")
        .join("entry")
}

fn template_core_path() -> PathBuf {
    fixtures_dir().join("entry_template.core")
}

fn diagnostics_dir() -> PathBuf {
    fixtures_dir().join("diagnostics")
}

fn snapshot_path() -> PathBuf {
    diagnostics_dir().join("p62_parked_state.json")
}

fn vagc_capture_enabled() -> bool {
    std::env::var("VAGC_CAPTURE")
        .map(|v| v == "1")
        .unwrap_or(false)
}

fn connect_with_retry(port: u16) -> YaAgcClient {
    match YaAgcClient::connect_localhost(port) {
        Ok(c) => c,
        Err(_) => {
            std::thread::sleep(Duration::from_millis(500));
            YaAgcClient::connect_localhost(port).expect("connect retry")
        }
    }
}

fn pick_port() -> u16 {
    use std::sync::atomic::{AtomicU16, Ordering};
    static NEXT: AtomicU16 = AtomicU16::new(46_500);
    NEXT.fetch_add(16, Ordering::SeqCst)
}

fn current_mtime(path: &Path) -> Option<std::time::SystemTime> {
    std::fs::metadata(path).ok().and_then(|m| m.modified().ok())
}

fn wait_for_new_dump(
    path: &Path,
    previous: Option<std::time::SystemTime>,
    deadline: Instant,
) -> Option<std::time::SystemTime> {
    while Instant::now() < deadline {
        if let Some(now) = current_mtime(path) {
            if previous.map(|p| now > p).unwrap_or(true) {
                std::thread::sleep(Duration::from_millis(30));
                return Some(now);
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    None
}

fn try_load_core(path: &Path) -> Option<CoreImage> {
    for _ in 0..3 {
        match CoreImage::load(path) {
            Ok(c) => return Some(c),
            Err(_) => std::thread::sleep(Duration::from_millis(80)),
        }
    }
    None
}

// ── Snapshot record ─────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
struct ReadVal {
    /// AGC erasable bank index (0..8).
    bank: u8,
    /// AGC erasable offset within the bank (0..256, decimal).
    offset: u16,
    /// Raw 15-bit AGC word, formatted as 5-octal-digit string.
    octal: String,
    /// Same word as decimal, for convenience.
    decimal: u16,
}

impl ReadVal {
    fn read(core: &CoreImage, symtab: &Symtab, name: &str) -> Option<Self> {
        let addr = symtab.get(name)?;
        let AgcAddress::Erasable { bank, offset } = addr else {
            return None;
        };
        let word = core.read_sp(addr)?;
        Some(Self {
            bank,
            offset,
            octal: format!("0o{:05o}", word),
            decimal: word,
        })
    }
}

/// Fixed-memory address record — used so the human reader can decode
/// raw `DOTINC` values against known TC targets.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
struct FixedAddr {
    bank: u8,
    offset: u16,
    /// "BB,OOOO" representation (yaYUL listing form).
    listing: String,
}

impl FixedAddr {
    fn lookup(symtab: &Symtab, name: &str) -> Option<Self> {
        let addr = symtab.get(name)?;
        let AgcAddress::Fixed { bank, offset } = addr else {
            return None;
        };
        Some(Self {
            bank,
            offset,
            listing: format!("{:02o},{:04o}", bank, offset),
        })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
struct ParkedSnapshot {
    /// Number of fresh core-dump mtimes observed before this snapshot
    /// was captured. 0 = no dump yet; >=1 = a real dump landed.
    dumps_seen: u32,
    /// How long the harness waited for `MODREG = 0o076` to appear, in
    /// milliseconds. `None` when the snapshot is unconditional (e.g.,
    /// post-V33 we don't gate).
    settle_wait_ms: Option<u32>,
    /// `true` if the harness observed `MODREG = 0o076` before reading.
    settled_in_p62: bool,
    modreg: Option<ReadVal>,
    verbreg: Option<ReadVal>,
    nounreg: Option<ReadVal>,
    mmnumber: Option<ReadVal>,
    /// `DOTINC` aliases `ENTRET`; one erasable cell, two names.
    dotinc_entret: Option<ReadVal>,
    /// Pretty interpretation of `dotinc_entret.octal` against the known
    /// `TC ENDOFJOB` (`0o05217`) and `TC NVSUBEND` (`0o04216`) encodings
    /// pulled from the listing.
    dotinc_decoded: String,
    cadrstor: Option<ReadVal>,
    reqret: Option<ReadVal>,
    loadstat: Option<ReadVal>,
    /// `FAILREG ERASE +2` — 3 consecutive words at FAILREG..FAILREG+2.
    failreg: Vec<Option<ReadVal>>,
    /// Phase-table cells (`PHASE1`..`PHASE6`).
    phases: Vec<Option<ReadVal>>,
    /// Complement-of-phase cells (`-PHASE1`..`-PHASE6`).
    neg_phases: Vec<Option<ReadVal>>,
    /// Per-phase-pair `PHASE XOR -PHASE` result, formatted as octal.
    /// AGC restart-check at `FRESH_START_AND_RESTART.agc:419-429`
    /// requires this to be `0o77777` (all-ones, ones-complement -0).
    /// Anything else triggers alarm `0o01107` (PHASE TABLE FAILURE)
    /// and a fresh start.
    phase_xor: Vec<String>,
    /// Other scheduler cells worth eyeballing.
    newjob: Option<ReadVal>,
    extvbact: Option<ReadVal>,
    flagwrd4: Option<ReadVal>,
    flagwrd5: Option<ReadVal>,
    flagwrd6: Option<ReadVal>,
    flagwrd7: Option<ReadVal>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
struct StderrScan {
    /// Total stderr lines captured.
    total_lines: u32,
    /// Lines containing the literal "01501" (POODOO alarm code).
    matched_01501: Vec<String>,
    /// Lines containing "POODOO".
    matched_poodoo: Vec<String>,
    /// Lines containing "ALARM" (case-insensitive).
    matched_alarm: Vec<String>,
    /// Lines containing "RESTART".
    matched_restart: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
struct ChannelScan {
    /// Number of writes observed on channel 010 (MODE/PROG display).
    chan_010: u32,
    /// Number of writes observed on channel 011 (CAUTION/STATUS lamps).
    chan_011: u32,
    /// Number of writes observed on channel 013 (V/N display, FLASH bit).
    chan_013: u32,
    /// Most recent channel-011 value seen (octal).
    last_chan_011_octal: Option<String>,
    /// Most recent channel-013 value seen (octal).
    last_chan_013_octal: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
struct ExperimentARecord {
    schema_version: u32,
    description: String,
    /// Pre-V33 parked-state snapshot, taken after the AGC settles into
    /// `GOFLASH V06N61`.
    snapshot_a_pre_v33: ParkedSnapshot,
    /// Post-V33 snapshot, taken ~1 s after `V33 ENTR` is sent.
    snapshot_b_post_v33: ParkedSnapshot,
    /// Stderr lines observed across the whole run.
    stderr_full: StderrScan,
    /// Channel writes observed between V37 and the post-V33 snapshot.
    channels: ChannelScan,
    /// `ENDOFJOB` fixed address — for off-line decoding of `DOTINC`.
    addr_endofjob: Option<FixedAddr>,
    /// `NVSUBEND` fixed address — for off-line decoding of `DOTINC`.
    addr_nvsubend: Option<FixedAddr>,
    /// `POODOO` fixed address.
    addr_poodoo: Option<FixedAddr>,
    /// Hypothesis evaluation, computed from the snapshots above.
    hypothesis: HypothesisOutcome,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
struct HypothesisOutcome {
    /// `MODREG == 0o076` in snapshot A (AGC really is parked in P62).
    a_modreg_p62: bool,
    /// `CADRSTOR != 0` in snapshot A (AGC really is at ENDIDLE).
    a_cadrstor_nonzero: bool,
    /// `NOUNREG == 0` in snapshot A. Required by the TESTNN-fails-on-
    /// noun-zero branch of the hypothesis.
    a_nounreg_zero: bool,
    /// `true` if `DOTINC = 0o04216 = TC NVSUBEND` in snapshot A. The
    /// smoking-gun observable for the V33 / DSPALARM / POODOO path.
    a_entret_is_tc_nvsubend: bool,
    /// `MODREG` value in snapshot B (post-V33). 0o076 + the restart-
    /// table effects = "POODOO fired and restored MODREG to 062".
    b_modreg_octal: String,
    /// `VERBREG` value in snapshot B. 0o045 + the restart effects =
    /// POODOO restored V37 leftovers.
    b_verbreg_octal: String,
    /// `true` if V33 ENTR did NOT advance the AGC out of P62 — the
    /// observable failure of the wake path that motivates this issue.
    v33_did_not_advance: bool,
    /// Whether any "01501" stderr line was seen.
    stderr_01501_seen: bool,
    /// Whether any FAILREG slot holds the phase-table-failure alarm
    /// (`0o01107`). Documents a SEPARATE preload-validity bug surfaced
    /// during the experiment.
    failreg_holds_01107: bool,
    /// `true` if all six phase pairs satisfy the complement invariant
    /// (`-PHASEi XOR PHASEi == 0o77777`). Required for the restart
    /// check at `FRESH_START_AND_RESTART.agc:419-429` to pass.
    a_phase_table_complement_ok: bool,
}

// ── Stderr capture thread ───────────────────────────────────────────────────

fn spawn_stderr_capture(stderr: std::process::ChildStderr) -> Arc<Mutex<Vec<String>>> {
    let buf = Arc::new(Mutex::new(Vec::<String>::new()));
    let buf_clone = Arc::clone(&buf);
    thread::spawn(move || {
        let reader = BufReader::new(stderr);
        for line in reader.lines().map_while(Result::ok) {
            if let Ok(mut g) = buf_clone.lock() {
                g.push(line);
            }
        }
    });
    buf
}

fn scan_stderr(lines: &[String]) -> StderrScan {
    let mut s = StderrScan {
        total_lines: lines.len() as u32,
        matched_01501: Vec::new(),
        matched_poodoo: Vec::new(),
        matched_alarm: Vec::new(),
        matched_restart: Vec::new(),
    };
    for line in lines {
        if line.contains("01501") {
            s.matched_01501.push(line.clone());
        }
        if line.contains("POODOO") {
            s.matched_poodoo.push(line.clone());
        }
        if line.to_uppercase().contains("ALARM") {
            s.matched_alarm.push(line.clone());
        }
        if line.to_uppercase().contains("RESTART") {
            s.matched_restart.push(line.clone());
        }
    }
    s
}

// ── Channel-write capture (light, foreground-polling) ───────────────────────

#[derive(Default)]
struct ChannelObs {
    chan_010: u32,
    chan_011: u32,
    chan_013: u32,
    last_011: Option<u16>,
    last_013: Option<u16>,
}

impl ChannelObs {
    fn into_scan(self) -> ChannelScan {
        ChannelScan {
            chan_010: self.chan_010,
            chan_011: self.chan_011,
            chan_013: self.chan_013,
            last_chan_011_octal: self.last_011.map(|v| format!("0o{:05o}", v)),
            last_chan_013_octal: self.last_013.map(|v| format!("0o{:05o}", v)),
        }
    }
}

fn drain_channels(client: &mut YaAgcClient, dur: Duration, obs: &mut ChannelObs) {
    let deadline = Instant::now() + dur;
    while Instant::now() < deadline {
        match client.try_recv(Duration::from_millis(50)) {
            Ok(pkt) => match pkt.channel {
                0o10 => obs.chan_010 += 1,
                0o11 => {
                    obs.chan_011 += 1;
                    obs.last_011 = Some(pkt.value);
                }
                0o13 => {
                    obs.chan_013 += 1;
                    obs.last_013 = Some(pkt.value);
                }
                _ => {}
            },
            Err(_) => break,
        }
    }
}

// ── Snapshot helper ─────────────────────────────────────────────────────────

/// Decode a `DOTINC`/`ENTRET` value against the two known TC targets
/// (`TC ENDOFJOB` = `0o05217`, `TC NVSUBEND` = `0o04216`) extracted
/// from the Comanche055 listing. Returns a human-readable label.
fn decode_dotinc(word: u16) -> &'static str {
    match word {
        0o05217 => "TC ENDOFJOB (safe — DSPALARM falls to FALTON/ENDOFJOB)",
        0o04216 => "TC NVSUBEND (POODOO path — DSPALARM falls to TC POODOO; OCT 01501)",
        0 => "0 (uninitialised)",
        _ => "unknown encoding",
    }
}

/// Block II AGC encoding of `TC NVSUBEND` (= `0o04216`). Pulled from
/// the Comanche055 listing — every `TC NVSUBEND` line encodes to this
/// raw word (`PINBALL_GAME__BUTTONS_AND_LIGHTS.agc:3133, 3139,` etc.).
const TC_NVSUBEND_WORD: u16 = 0o04216;

fn build_snapshot(
    core: &CoreImage,
    symtab: &Symtab,
    dumps_seen: u32,
    settle_wait_ms: Option<u32>,
    settled_in_p62: bool,
) -> ParkedSnapshot {
    let mut failreg = Vec::with_capacity(3);
    if let Some(AgcAddress::Erasable { bank, offset }) = symtab.get("FAILREG") {
        for i in 0..3u16 {
            let addr = AgcAddress::Erasable {
                bank,
                offset: offset + i,
            };
            let raw = core.read_sp(addr);
            failreg.push(raw.map(|w| ReadVal {
                bank,
                offset: offset + i,
                octal: format!("0o{:05o}", w),
                decimal: w,
            }));
        }
    }
    let dotinc = ReadVal::read(core, symtab, "DOTINC");
    let dotinc_decoded = dotinc
        .as_ref()
        .map(|r| decode_dotinc(r.decimal).to_string())
        .unwrap_or_else(|| "n/a".to_string());

    // Phase table: 6 pairs of (-PHASEn, PHASEn) at consecutive
    // addresses. -PHASEn XOR PHASEn must equal 0o77777 (-0) for the
    // restart-check at FRESH_START_AND_RESTART.agc:419-429 to pass.
    let mut phases = Vec::with_capacity(6);
    let mut neg_phases = Vec::with_capacity(6);
    let mut phase_xor = Vec::with_capacity(6);
    for i in 1..=6u8 {
        let pname = format!("PHASE{}", i);
        let nname = format!("-PHASE{}", i);
        let p = ReadVal::read(core, symtab, &pname);
        let n = ReadVal::read(core, symtab, &nname);
        let x = match (p.as_ref(), n.as_ref()) {
            (Some(pv), Some(nv)) => format!("0o{:05o}", pv.decimal ^ nv.decimal),
            _ => "n/a".to_string(),
        };
        phases.push(p);
        neg_phases.push(n);
        phase_xor.push(x);
    }

    ParkedSnapshot {
        dumps_seen,
        settle_wait_ms,
        settled_in_p62,
        modreg: ReadVal::read(core, symtab, "MODREG"),
        verbreg: ReadVal::read(core, symtab, "VERBREG"),
        nounreg: ReadVal::read(core, symtab, "NOUNREG"),
        mmnumber: ReadVal::read(core, symtab, "MMNUMBER"),
        dotinc_entret: dotinc,
        dotinc_decoded,
        cadrstor: ReadVal::read(core, symtab, "CADRSTOR"),
        reqret: ReadVal::read(core, symtab, "REQRET"),
        loadstat: ReadVal::read(core, symtab, "LOADSTAT"),
        failreg,
        phases,
        neg_phases,
        phase_xor,
        newjob: ReadVal::read(core, symtab, "NEWJOB"),
        extvbact: ReadVal::read(core, symtab, "EXTVBACT"),
        flagwrd4: ReadVal::read(core, symtab, "FLAGWRD4"),
        flagwrd5: ReadVal::read(core, symtab, "FLAGWRD5"),
        flagwrd6: ReadVal::read(core, symtab, "FLAGWRD6"),
        flagwrd7: ReadVal::read(core, symtab, "FLAGWRD7"),
    }
}

/// Poll the dump file every 200 ms for up to `max_ms`, reloading on
/// each fresh mtime. Returns `(core, dumps_seen, settle_wait_ms,
/// settled_in_p62)` as soon as a dump arrives that shows
/// `MODREG = 0o076`, or after `max_ms` if no such dump appears.
fn wait_for_p62_parked(
    dump_path: &Path,
    symtab: &Symtab,
    initial_mtime: Option<std::time::SystemTime>,
    max_ms: u32,
) -> (Option<CoreImage>, u32, u32, bool) {
    let start = Instant::now();
    let deadline = start + Duration::from_millis(max_ms as u64);
    let mut last_mtime = initial_mtime;
    let mut latest_core: Option<CoreImage> = None;
    let mut dumps_seen = 0u32;
    let mut settled = false;
    while Instant::now() < deadline {
        if let Some(new_mtime) = wait_for_new_dump(
            dump_path,
            last_mtime,
            (start + Duration::from_millis(max_ms as u64)).min(Instant::now() + Duration::from_millis(400)),
        ) {
            last_mtime = Some(new_mtime);
            if let Some(c) = try_load_core(dump_path) {
                dumps_seen += 1;
                let modreg = symtab.get("MODREG").and_then(|a| c.read_sp(a)).unwrap_or(0);
                latest_core = Some(c);
                if modreg == 0o076 {
                    settled = true;
                    break;
                }
            }
        }
    }
    let waited_ms = start.elapsed().as_millis() as u32;
    (latest_core, dumps_seen, waited_ms, settled)
}

// ── The experiment ──────────────────────────────────────────────────────────

#[test]
fn tc_e7i_a_parked_state_snapshot() {
    let template_path = template_core_path();
    if !template_path.exists() {
        eprintln!(
            "skipping: no template core at {} \
             (run `cargo run --features vagc-capture --bin capture_entry_template`)",
            template_path.display()
        );
        return;
    }
    let root = vagc_root();
    let yaagc = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");
    let listing = root.join("Comanche055/MAIN.agc.lst");
    if !yaagc.exists() || !rope.exists() || !listing.exists() {
        eprintln!(
            "skipping: VirtualAGC build incomplete at {}",
            root.display()
        );
        return;
    }

    // Step 1: patch the template with the direct-LEO state so the AGC
    // boots into a realistic entry preflight. This matches the
    // patching the closed-loop test does.
    let mut core = CoreImage::load(&template_path)
        .unwrap_or_else(|e| panic!("load template {}: {e}", template_path.display()));
    let symtab = Symtab::load(&listing)
        .unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
    let rust_state = setup_state_direct_leo();
    let init = EntryInitialState {
        position_m: rust_state.csm_state.position,
        velocity_mps: rust_state.csm_state.velocity,
        time_s: 0.0,
        target_lat_rad: rust_state.entry.target_lat_rad,
        target_lon_rad: rust_state.entry.target_lon_rad,
        emsalt_m: 122_000.0,
        alfa_pad_deg: -20.0,
        lift_up: true,
        refsmmat: EntryInitialState::identity_refsmmat(),
        cmdapmod: -1,
    };
    patch_into(&mut core, &symtab, &init).expect("patch_into ok");

    let work = std::env::temp_dir().join(format!("vagc_e7i_a_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let core_in = work.join("core_in");
    core.save(&core_in).unwrap();

    // Step 2: spawn yaAGC. stderr is captured (not nulled) so we can
    // scan for alarm markers. `--inhibit-alarms` suppresses the
    // *host*-checked alarms (Night Watchman / Rupt Lock / TC Trap);
    // software-internal AGC alarms like POODOO are unaffected.
    let port = pick_port();
    let mut child = std::process::Command::new(&yaagc)
        .current_dir(&work)
        .arg("--quiet")
        .arg("--nodebug")
        .arg("--inhibit-alarms")
        .arg("--dump-time=1")
        .arg(format!("--port={port}"))
        .arg(&rope)
        .arg(&core_in)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("yaAGC spawn");

    let stderr = child.stderr.take().expect("piped stderr");
    let stderr_buf = spawn_stderr_capture(stderr);

    std::thread::sleep(Duration::from_millis(300));

    // Step 3: open DSKY input + a channel-write listener.
    let mut dsky = DskyScript::new(connect_with_retry(port));
    let mut listener = connect_with_retry(port + 1);
    let mut obs = ChannelObs::default();

    // Step 4: warm-up — drain startup channel traffic.
    drain_channels(&mut listener, Duration::from_millis(500), &mut obs);

    // Step 5: V37 ENTR 62 ENTR — enter P62. Then **poll** the dump
    // file until a dump shows `MODREG = 0o076` (AGC truly parked at
    // GOFLASH). A fixed 1.8 s wait is unreliable: yaAGC's wall-clock
    // pace varies, and the dump file is rewritten every 1 simulated
    // second, so we need to track mtimes rather than guess the wait.
    let dump_path = work.join("core");
    let mtime_pre = current_mtime(&dump_path);
    dsky.verb_major_mode(62).expect("V37 ENTR 62 ENTR");
    // Background channel drain while we poll. 6 s budget is generous;
    // typical settle is 2–3 s wall.
    let (core_a, dumps_seen_a, settle_a_ms, settled_a) = {
        let drain_until =
            Instant::now() + std::time::Duration::from_millis(150);
        drain_channels(&mut listener, drain_until - Instant::now(), &mut obs);
        wait_for_p62_parked(&dump_path, &symtab, mtime_pre, 6_000)
    };
    // Drain any remaining channel traffic in a short tail.
    drain_channels(&mut listener, Duration::from_millis(200), &mut obs);

    // Step 6: V33 ENTR — the keystroke that should advance P62 →
    // ROLLC init → P63 but (per the hypothesis) instead triggers
    // POODOO and a software restart back to P62.
    let mtime_after_a = current_mtime(&dump_path);
    dsky.proceed().expect("V33 ENTR send");
    drain_channels(&mut listener, Duration::from_millis(1_800), &mut obs);

    // For snapshot B we don't gate on MODREG (could be anything after
    // POODOO restart); just take the next dump.
    let dumps_seen_b = if wait_for_new_dump(
        &dump_path,
        mtime_after_a,
        Instant::now() + Duration::from_secs(3),
    )
    .is_some()
    {
        dumps_seen_a + 1
    } else {
        dumps_seen_a
    };
    let core_b = try_load_core(&dump_path);
    drain_channels(&mut listener, Duration::from_millis(200), &mut obs);

    // Step 7: tear down yaAGC. Stderr is then fully flushed by the
    // capture thread (BufReader closes on EOF).
    let _ = child.kill();
    let _ = child.wait();
    std::thread::sleep(Duration::from_millis(150));

    let stderr_lines = {
        let g = stderr_buf.lock().expect("stderr buf lock");
        g.clone()
    };
    let stderr_full = scan_stderr(&stderr_lines);

    // Step 8: build the experiment record.
    let snapshot_a = if let Some(c) = &core_a {
        build_snapshot(c, &symtab, dumps_seen_a, Some(settle_a_ms), settled_a)
    } else {
        // No dump landed — leave fields empty so the JSON still
        // serialises and the assertions catch it.
        build_snapshot(
            &CoreImage::empty(),
            &symtab,
            dumps_seen_a,
            Some(settle_a_ms),
            settled_a,
        )
    };
    let snapshot_b = if let Some(c) = &core_b {
        build_snapshot(c, &symtab, dumps_seen_b, None, true)
    } else {
        build_snapshot(&CoreImage::empty(), &symtab, dumps_seen_b, None, false)
    };

    let a_modreg_p62 = snapshot_a
        .modreg
        .as_ref()
        .map(|r| r.decimal == 0o076)
        .unwrap_or(false);
    let a_cadrstor_nonzero = snapshot_a
        .cadrstor
        .as_ref()
        .map(|r| r.decimal != 0)
        .unwrap_or(false);
    let a_nounreg_zero = snapshot_a
        .nounreg
        .as_ref()
        .map(|r| r.decimal == 0)
        .unwrap_or(false);
    let a_entret_is_tc_nvsubend = snapshot_a
        .dotinc_entret
        .as_ref()
        .map(|r| r.decimal == TC_NVSUBEND_WORD)
        .unwrap_or(false);
    let b_modreg_octal = snapshot_b
        .modreg
        .as_ref()
        .map(|r| r.octal.clone())
        .unwrap_or_else(|| "n/a".to_string());
    let b_verbreg_octal = snapshot_b
        .verbreg
        .as_ref()
        .map(|r| r.octal.clone())
        .unwrap_or_else(|| "n/a".to_string());
    let v33_did_not_advance = snapshot_b
        .modreg
        .as_ref()
        .map(|r| r.decimal == 0o076)
        .unwrap_or(false);
    let stderr_01501_seen = !stderr_full.matched_01501.is_empty();
    let failreg_holds_01107 = snapshot_a
        .failreg
        .iter()
        .chain(snapshot_b.failreg.iter())
        .flatten()
        .any(|r| r.decimal == 0o01107);
    // Phase-table invariant: every pair must XOR to 0o77777.
    let a_phase_table_complement_ok = !snapshot_a.phase_xor.is_empty()
        && snapshot_a
            .phase_xor
            .iter()
            .all(|s| s == "0o77777");

    let record = ExperimentARecord {
        schema_version: 1,
        description: format!(
            "MS-E7i Experiment A — snapshot of AGC erasable state \
             after V37 ENTR 62 ENTR and again after V33 ENTR. \
             Captured against VirtualAGC at {}.",
            root.display()
        ),
        snapshot_a_pre_v33: snapshot_a,
        snapshot_b_post_v33: snapshot_b,
        stderr_full,
        channels: obs.into_scan(),
        addr_endofjob: FixedAddr::lookup(&symtab, "ENDOFJOB"),
        addr_nvsubend: FixedAddr::lookup(&symtab, "NVSUBEND"),
        addr_poodoo: FixedAddr::lookup(&symtab, "POODOO"),
        hypothesis: HypothesisOutcome {
            a_modreg_p62,
            a_cadrstor_nonzero,
            a_nounreg_zero,
            a_entret_is_tc_nvsubend,
            b_modreg_octal,
            b_verbreg_octal,
            v33_did_not_advance,
            stderr_01501_seen,
            failreg_holds_01107,
            a_phase_table_complement_ok,
        },
    };

    // Step 9: clean up the work dir.
    let _ = std::fs::remove_dir_all(&work);

    // Step 10: write or verify the JSON fixture.
    let snapshot = snapshot_path();
    if vagc_capture_enabled() {
        std::fs::create_dir_all(diagnostics_dir()).expect("mkdir diagnostics");
        let text = serde_json::to_string_pretty(&record).expect("serialize record");
        std::fs::write(&snapshot, text).expect("write snapshot");
        eprintln!(
            "[ms-e7i-a] VAGC_CAPTURE=1 — refreshed {}",
            snapshot.display()
        );
    } else if snapshot.exists() {
        let text = std::fs::read_to_string(&snapshot).expect("read snapshot");
        let committed: ExperimentARecord =
            serde_json::from_str(&text).expect("parse committed snapshot");
        // Pin the *hypothesis outcome* (not the full record — yaAGC
        // run-to-run jitter may shake other fields).
        assert_eq!(
            committed.hypothesis, record.hypothesis,
            "hypothesis outcome diverged from committed fixture; \
             re-run with VAGC_CAPTURE=1 to refresh after deliberate \
             changes"
        );
    } else {
        eprintln!(
            "[ms-e7i-a] no committed snapshot at {} — first run; \
             VAGC_CAPTURE=1 to capture",
            snapshot.display()
        );
    }

    // Step 11: report what we learned, regardless of pin status.
    eprintln!(
        "[ms-e7i-a] snapshot A: parked_p62={} cadrstor_nonzero={} \
         nounreg_zero={} entret_is_tc_nvsubend={} settle_ms={} dumps_seen={}",
        record.hypothesis.a_modreg_p62,
        record.hypothesis.a_cadrstor_nonzero,
        record.hypothesis.a_nounreg_zero,
        record.hypothesis.a_entret_is_tc_nvsubend,
        record.snapshot_a_pre_v33.settle_wait_ms.unwrap_or(0),
        record.snapshot_a_pre_v33.dumps_seen,
    );
    eprintln!(
        "[ms-e7i-a] snapshot B post-V33: MODREG={} VERBREG={} \
         v33_did_not_advance={} stderr_01501={} failreg_holds_01107={} \
         phase_table_ok={}",
        record.hypothesis.b_modreg_octal,
        record.hypothesis.b_verbreg_octal,
        record.hypothesis.v33_did_not_advance,
        record.hypothesis.stderr_01501_seen,
        record.hypothesis.failreg_holds_01107,
        record.hypothesis.a_phase_table_complement_ok,
    );
    eprintln!(
        "[ms-e7i-a] phase_xor (must all be 0o77777): {:?}",
        record.snapshot_a_pre_v33.phase_xor
    );
    if let Some(d) = &record.snapshot_a_pre_v33.dotinc_entret {
        eprintln!(
            "[ms-e7i-a] DOTINC at snapshot A = {} ({})",
            d.octal, record.snapshot_a_pre_v33.dotinc_decoded
        );
    }
}
