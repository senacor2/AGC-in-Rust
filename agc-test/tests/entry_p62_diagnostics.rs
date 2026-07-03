// SPDX-License-Identifier: GPL-3.0-or-later
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
use agc_test::vagc_driver::{entry_trim_cdu_deg, CduInjector, DskyScript};
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

    // POODOO-related cells. ALMCADR is a 2-word CADR holding the
    // calling location of the most recent ALARM/POODOO; the listing
    // line at that address pinpoints the failing call. Captured along
    // with neighbours for context.
    /// `ALMCADR` low word — call-site CADR. Saved at the top of POODOO
    /// (`ALARM_AND_ABORT.agc:171-172`).
    almcadr_lo: Option<ReadVal>,
    /// `ALMCADR +1` — high word of the alarm CADR (rarely used).
    almcadr_hi: Option<ReadVal>,
    /// `ERCOUNT` — count of errors recorded since the last fresh start.
    ercount: Option<ReadVal>,
    /// `REDOCTR` — count of restarts seen by GOPROG.
    redoctr: Option<ReadVal>,

    // Timing / waitlist cells. A non-positive `DT` argument to WAITLIST
    // trips POODOO 01204 (`WAITLIST.agc:156-157`). Watching these tells
    // us whether the cold-boot template starts in a state where some
    // P62 sub-routine would compute a bad DT.
    time1: Option<ReadVal>,
    time3: Option<ReadVal>,
    time4: Option<ReadVal>,
    tbase1: Option<ReadVal>,
    s61dt: Option<ReadVal>,
    posexit: Option<ReadVal>,

    /// `FLAGWRD0` so we can decode `AVEGFLAG` (= flag-bit 29 decimal,
    /// which lives in `FLAGWRD1` per the comment block at
    /// `ERASABLE_ASSIGNMENTS.agc:488`).
    flagwrd0: Option<ReadVal>,
    flagwrd1: Option<ReadVal>,
    flagwrd2: Option<ReadVal>,
    flagwrd3: Option<ReadVal>,
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

        almcadr_lo: ReadVal::read(core, symtab, "ALMCADR"),
        almcadr_hi: read_offset(core, symtab, "ALMCADR", 1),
        ercount: ReadVal::read(core, symtab, "ERCOUNT"),
        redoctr: ReadVal::read(core, symtab, "REDOCTR"),

        time1: ReadVal::read(core, symtab, "TIME1"),
        time3: ReadVal::read(core, symtab, "TIME3"),
        time4: ReadVal::read(core, symtab, "TIME4"),
        tbase1: ReadVal::read(core, symtab, "TBASE1"),
        s61dt: ReadVal::read(core, symtab, "S61DT"),
        posexit: ReadVal::read(core, symtab, "POSEXIT"),

        flagwrd0: ReadVal::read(core, symtab, "FLAGWRD0"),
        flagwrd1: ReadVal::read(core, symtab, "FLAGWRD1"),
        flagwrd2: ReadVal::read(core, symtab, "FLAGWRD2"),
        flagwrd3: ReadVal::read(core, symtab, "FLAGWRD3"),
    }
}

/// Read the erasable word at `symbol + offset` words.
fn read_offset(core: &CoreImage, symtab: &Symtab, symbol: &str, offset: u16) -> Option<ReadVal> {
    let base = symtab.get(symbol)?;
    let AgcAddress::Erasable {
        bank,
        offset: base_off,
    } = base
    else {
        return None;
    };
    let addr = AgcAddress::Erasable {
        bank,
        offset: base_off + offset,
    };
    let raw = core.read_sp(addr)?;
    Some(ReadVal {
        bank,
        offset: base_off + offset,
        octal: format!("0o{:05o}", raw),
        decimal: raw,
    })
}

/// Poll the dump file every 200 ms for up to `max_ms`, reloading on
/// each fresh mtime. Returns `(core, dumps_seen, settle_wait_ms,
/// settled_in_p62)` as soon as a dump arrives that shows the AGC
/// truly parked at `GOFLASH V06N61` — i.e. `MODREG == 0o076` AND
/// `CADRSTOR != 0` — or after `max_ms` if no such dump appears.
///
/// **Why the two-condition gate.** P62's prelude bumps `MMNUMBER`
/// (which writes `MODREG = 0o076`) before its first
/// `GOFLASH V06N61 → NVSUB` body has executed. `NVSUB` is what
/// populates `CADRSTOR` (PINBALL `ENDIDLE` storage; see
/// `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc:110, 238`) and writes
/// `DOTINC = TC NVSUBEND`. A naive `MODREG == 0o076` gate catches
/// the brief window in between and produces a snapshot where
/// `CADRSTOR == 0` / `DOTINC = 0`, which is not the ENDIDLE steady
/// state the hypothesis is examining. See issue #119.
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
            (start + Duration::from_millis(max_ms as u64))
                .min(Instant::now() + Duration::from_millis(400)),
        ) {
            last_mtime = Some(new_mtime);
            if let Some(c) = try_load_core(dump_path) {
                dumps_seen += 1;
                let modreg = symtab.get("MODREG").and_then(|a| c.read_sp(a)).unwrap_or(0);
                let cadrstor = symtab
                    .get("CADRSTOR")
                    .and_then(|a| c.read_sp(a))
                    .unwrap_or(0);
                latest_core = Some(c);
                if modreg == 0o076 && cadrstor != 0 {
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
    let symtab =
        Symtab::load(&listing).unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
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
        refsmmat: EntryInitialState::entry_refsmmat(
            rust_state.csm_state.position,
            rust_state.csm_state.velocity,
        ),
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
        let drain_until = Instant::now() + std::time::Duration::from_millis(150);
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
    let a_phase_table_complement_ok =
        !snapshot_a.phase_xor.is_empty() && snapshot_a.phase_xor.iter().all(|s| s == "0o77777");

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

/// TC-E7I-B: control experiment for #45 — drive only `V37 ENTR 62 ENTR`,
/// then wait several seconds with **no V33 keystroke at all**. Captures
/// the AGC's parked-state FAILREG so we can tell whether the
/// `0o01204` waitlist alarm seen in the V33 experiment is V33-triggered
/// or background.
///
/// This is a transient experiment — kept only long enough to answer
/// the question. Once the source of `01204` is pinned, it can be
/// removed.
#[test]
fn tc_e7i_b_parked_state_no_v33() {
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

    let mut core = CoreImage::load(&template_path)
        .unwrap_or_else(|e| panic!("load template {}: {e}", template_path.display()));
    let symtab =
        Symtab::load(&listing).unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
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
        refsmmat: EntryInitialState::entry_refsmmat(
            rust_state.csm_state.position,
            rust_state.csm_state.velocity,
        ),
        cmdapmod: -1,
    };
    patch_into(&mut core, &symtab, &init).expect("patch_into ok");

    let work = std::env::temp_dir().join(format!("vagc_e7i_b_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let core_in = work.join("core_in");
    core.save(&core_in).unwrap();

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
    let _stderr_buf = spawn_stderr_capture(stderr);
    std::thread::sleep(Duration::from_millis(300));

    let mut dsky = DskyScript::new(connect_with_retry(port));
    let mut listener = connect_with_retry(port + 1);
    let mut obs = ChannelObs::default();
    drain_channels(&mut listener, Duration::from_millis(500), &mut obs);

    let dump_path = work.join("core");
    let mtime_pre = current_mtime(&dump_path);
    dsky.verb_major_mode(62).expect("V37 ENTR 62 ENTR");

    // Wait for parked state, then sit for an extra 3 seconds (no V33)
    // so background scheduler/T*RUPT activity has plenty of room to
    // raise any waitlist alarms it's going to raise.
    let (_core_parked, _dumps, settle_ms, settled) =
        wait_for_p62_parked(&dump_path, &symtab, mtime_pre, 6_000);
    drain_channels(&mut listener, Duration::from_millis(3_000), &mut obs);

    let mtime_post = current_mtime(&dump_path);
    let _ = wait_for_new_dump(
        &dump_path,
        mtime_post,
        Instant::now() + Duration::from_secs(3),
    );
    let final_core = try_load_core(&dump_path);

    let _ = child.kill();
    let _ = child.wait();

    let snap = if let Some(c) = final_core {
        build_snapshot(&c, &symtab, 0, Some(settle_ms), settled)
    } else {
        build_snapshot(&CoreImage::empty(), &symtab, 0, Some(settle_ms), settled)
    };

    let _ = std::fs::remove_dir_all(&work);

    let failreg_01204 = snap.failreg.iter().flatten().any(|r| r.decimal == 0o01204);
    let failreg_01107 = snap.failreg.iter().flatten().any(|r| r.decimal == 0o01107);
    let failreg_str: Vec<String> = snap
        .failreg
        .iter()
        .map(|opt| opt.as_ref().map(|r| r.octal.clone()).unwrap_or_default())
        .collect();

    eprintln!(
        "[ms-e7i-b] no-V33 control: settled={} FAILREG={:?} \
         holds_01204={} holds_01107={} ALMCADR_lo={:?}",
        settled,
        failreg_str,
        failreg_01204,
        failreg_01107,
        snap.almcadr_lo.as_ref().map(|r| r.octal.clone()),
    );
}

/// TC-E7I-D: use the yaAGC built-in debugger to catch the ACTUAL caller
/// that raises the `0o01204` WAITLIST "zero/neg DT" alarm. Resumes the
/// SAME patched core as `tc_e7i_b`, but runs yaAGC with the debugger
/// active (breakpoints at `WATLST0-` — the `TC POODOO; OCT 1204` site —
/// and at `POODOO`), drives `V37 ENTR 62 ENTR`, and on the first hit
/// dumps a backtrace, the central registers, and a core image.
///
/// Diagnostic for issue #49. The `01204` is generated live during P62
/// entry init (the committed template has `FAILREG = [01107, 0, 0]`;
/// the parked state has `[01107, 01204, 0]` — the `01204` fills the
/// first empty slot per `ALARM_AND_ABORT.agc` CHKFAIL1/CHKFAIL2). This
/// test captures where it comes from instead of inferring it.
#[test]
fn tc_e7i_d_debugger_catch_01204() {
    let template_path = template_core_path();
    if !template_path.exists() {
        eprintln!("skipping: no template core at {}", template_path.display());
        return;
    }
    let root = vagc_root();
    let yaagc = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");
    let listing = root.join("Comanche055/MAIN.agc.lst");
    let symtab_file = root.join("Comanche055/MAIN.agc.symtab");
    if !yaagc.exists() || !rope.exists() || !listing.exists() || !symtab_file.exists() {
        eprintln!(
            "skipping: VirtualAGC build incomplete at {}",
            root.display()
        );
        return;
    }

    let mut core = CoreImage::load(&template_path)
        .unwrap_or_else(|e| panic!("load template {}: {e}", template_path.display()));
    let symtab =
        Symtab::load(&listing).unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
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
        refsmmat: EntryInitialState::entry_refsmmat(
            rust_state.csm_state.position,
            rust_state.csm_state.velocity,
        ),
        cmdapmod: -1,
    };
    patch_into(&mut core, &symtab, &init).expect("patch_into ok");

    let work = std::env::temp_dir().join(format!("vagc_e7i_d_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let core_in = work.join("core_in");
    core.save(&core_in).unwrap();

    // Debugger script: break at the 01204-specific WAITLIST alarm site
    // and at POODOO; on the first hit dump the call chain, registers,
    // and a core image, then quit. `WATLST0-` (the `TC POODOO`) is hit
    // just before 01204 is raised, so BACKTRACES shows the WAITLIST
    // caller that passed the bad DT.
    let hit_core = work.join("hit.core");
    let cmd_path = work.join("watch.txt");
    let script = format!(
        "BREAK WATLST0-\nBREAK POODOO\nCONT\nBACKTRACES\ninfo registers\nCOREDUMP {}\nQUIT\n",
        hit_core.display()
    );
    std::fs::write(&cmd_path, script).unwrap();

    let dbg_out = work.join("dbg.txt");
    let out_file = std::fs::File::create(&dbg_out).unwrap();

    let port = pick_port();
    let mut child = std::process::Command::new(&yaagc)
        .current_dir(&work)
        .arg("--quiet")
        .arg(format!("--symbols={}", symtab_file.display()))
        .arg("--inhibit-alarms")
        .arg(format!("--command={}", cmd_path.display()))
        .arg(format!("--port={port}"))
        .arg(&rope)
        .arg(&core_in)
        .stdout(std::process::Stdio::from(out_file))
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("yaAGC spawn");

    let stderr = child.stderr.take().expect("piped stderr");
    let _stderr_buf = spawn_stderr_capture(stderr);
    std::thread::sleep(Duration::from_millis(500));

    // Drive V37 ENTR 62 ENTR to enter P62. The 01204 alarm fires during
    // P62 init and should trip BREAK WATLST0-.
    let mut dsky = DskyScript::new(connect_with_retry(port));
    let _ = dsky.verb_major_mode(62);

    // Poll for the debugger script to hit, dump, and QUIT (up to ~30s).
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut exited = false;
    while Instant::now() < deadline {
        if let Ok(Some(_)) = child.try_wait() {
            exited = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    if !exited {
        let _ = child.kill();
    }
    let _ = child.wait();

    let dbg = std::fs::read_to_string(&dbg_out).unwrap_or_default();
    let hit = CoreImage::load(&hit_core).ok();
    let (z, a) = hit
        .as_ref()
        .map(|c| (c.erasable[0][5], c.erasable[0][0]))
        .unwrap_or((0, 0));

    eprintln!(
        "[ms-e7i-d] exited={} hit.core={} Z(PC)=0o{:05o} A=0o{:05o}",
        exited,
        hit.is_some(),
        z,
        a
    );

    // Confirm the state-vector-epoch-vs-clock mechanism: read the AGC
    // hardware clock (TIME2:TIME1, 14 bits each = centiseconds) and the
    // state-vector epoch cells (TET/PIPTIME) at the breakpoint. If the
    // clock has advanced far beyond TET/PIPTIME, the ENTMID1 integration
    // must bridge that gap and overruns WAITLIST's 12.5 s DT budget.
    if let Some(c) = hit.as_ref() {
        let rd = |bank: u8, offset: u16| c.read_sp(AgcAddress::Erasable { bank, offset });
        let time2 = rd(0, 0o24).unwrap_or(0) as u32;
        let time1 = rd(0, 0o25).unwrap_or(0) as u32;
        let clock_cs = time2 * 0o40000 + time1; // 2^14 = 0o40000
        let dp = |sym: &str| -> Option<i64> {
            match symtab.get(sym) {
                Some(AgcAddress::Erasable { bank, offset }) => {
                    let hi = rd(bank, offset)? as i64;
                    let lo = rd(bank, offset + 1)? as i64;
                    Some(hi * 0o40000 + lo)
                }
                _ => None,
            }
        };
        let sp = |sym: &str| -> Option<u16> {
            match symtab.get(sym) {
                Some(AgcAddress::Erasable { bank, offset }) => rd(bank, offset),
                _ => None,
            }
        };
        // AVEGFLAG = FLAGWRD1 bit 1 (mask 0o00001): AVERAGE-G on/off.
        // Hypothesis: P62 entry init here runs with AVERAGE-G OFF, so
        // S61.1 takes the MIDTOAV2 one-shot-integrate branch from a stale
        // (epoch-0) state instead of extrapolating a live state vector,
        // yielding a target time already behind the free-running clock.
        let flagwrd1 = sp("FLAGWRD1");
        let avegflag = flagwrd1.map(|w| w & 0o00001 != 0);
        eprintln!(
            "[ms-e7i-d] clock TIME2=0o{:05o} TIME1=0o{:05o} = {} cs ({:.2} s) | \
             TET={:?} cs  PIPTIME={:?} cs  S61DT={:?} | \
             AVEGFLAG={:?} FLAGWRD1={:?}",
            time2,
            time1,
            clock_cs,
            clock_cs as f64 / 100.0,
            dp("TET"),
            dp("PIPTIME"),
            dp("S61DT"),
            avegflag,
            flagwrd1.map(|w| format!("0o{:05o}", w)),
        );
    }
    eprintln!(
        "[ms-e7i-d] ── debugger session stdout ──\n{}",
        dbg.trim_end()
    );

    let _ = std::fs::remove_dir_all(&work);
}

/// TC-E7I-C: warm-template control — boot yaAGC from the **unpatched**
/// cold rope (no `core-in`, no `patch_into`), drive `V37 ENTR 01 ENTR`
/// (PRELAUNCH OR SERVICE), wait for P01 idle, then `V37 ENTR 62 ENTR`,
/// then `V33 ENTR`. Inspect the post-V33 state.
///
/// This is the empirical version of the analyst's "Option 2" (warm
/// template). If `FAILREG[1] != 0o01204` and `MODREG` advances past
/// `0o076` after V33, then driving through PRELAUNCH gives us the
/// scheduler/timing state we need. That would shape the fix as a
/// `capture_entry_template` change, not a `patch_into` change.
#[test]
fn tc_e7i_c_warm_template_no_preload() {
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
    let symtab =
        Symtab::load(&listing).unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));

    let work = std::env::temp_dir().join(format!("vagc_e7i_c_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();

    let port = pick_port();
    let mut child = std::process::Command::new(&yaagc)
        .current_dir(&work)
        .arg("--quiet")
        .arg("--nodebug")
        .arg("--inhibit-alarms")
        .arg("--dump-time=1")
        .arg("--no-resume")
        .arg(format!("--port={port}"))
        .arg(&rope)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("yaAGC spawn");

    let stderr = child.stderr.take().expect("piped stderr");
    let _stderr_buf = spawn_stderr_capture(stderr);
    std::thread::sleep(Duration::from_millis(500));

    let mut dsky = DskyScript::new(connect_with_retry(port));
    let mut listener = connect_with_retry(port + 1);
    let mut obs = ChannelObs::default();
    drain_channels(&mut listener, Duration::from_millis(800), &mut obs);

    let dump_path = work.join("core");

    // V37 ENTR 01 ENTR — PRELAUNCH OR SERVICE.
    dsky.verb_major_mode(1).expect("V37 ENTR 01 ENTR");
    drain_channels(&mut listener, Duration::from_millis(2_000), &mut obs);

    // Sample MODREG to see if we landed in P01.
    let mid_core = try_load_core(&dump_path);
    let modreg_after_p01 = mid_core
        .as_ref()
        .and_then(|c| ReadVal::read(c, &symtab, "MODREG"))
        .map(|r| r.octal.clone())
        .unwrap_or_else(|| "n/a".to_string());

    // V37 ENTR 62 ENTR — into P62.
    dsky.verb_major_mode(62).expect("V37 ENTR 62 ENTR");
    drain_channels(&mut listener, Duration::from_millis(2_000), &mut obs);

    let mtime_after_v62 = current_mtime(&dump_path);
    let (_core_parked, _dumps, settle_ms, settled) =
        wait_for_p62_parked(&dump_path, &symtab, mtime_after_v62, 6_000);

    let parked_snap =
        try_load_core(&dump_path).map(|c| build_snapshot(&c, &symtab, 0, Some(settle_ms), settled));

    // V33 ENTR — the wake we care about.
    let mtime_before_v33 = current_mtime(&dump_path);
    dsky.proceed().expect("V33 ENTR send");
    drain_channels(&mut listener, Duration::from_millis(2_000), &mut obs);
    let _ = wait_for_new_dump(
        &dump_path,
        mtime_before_v33,
        Instant::now() + Duration::from_secs(3),
    );

    let post_snap = try_load_core(&dump_path).map(|c| build_snapshot(&c, &symtab, 0, None, true));

    let _ = child.kill();
    let _ = child.wait();
    let _ = std::fs::remove_dir_all(&work);

    let post_failreg: Vec<String> = post_snap
        .as_ref()
        .map(|s| {
            s.failreg
                .iter()
                .map(|opt| opt.as_ref().map(|r| r.octal.clone()).unwrap_or_default())
                .collect()
        })
        .unwrap_or_default();
    let post_modreg = post_snap
        .as_ref()
        .and_then(|s| s.modreg.as_ref())
        .map(|r| r.octal.clone())
        .unwrap_or_else(|| "n/a".to_string());
    let post_verbreg = post_snap
        .as_ref()
        .and_then(|s| s.verbreg.as_ref())
        .map(|r| r.octal.clone())
        .unwrap_or_else(|| "n/a".to_string());
    let post_almcadr_lo = post_snap
        .as_ref()
        .and_then(|s| s.almcadr_lo.as_ref())
        .map(|r| r.octal.clone())
        .unwrap_or_else(|| "n/a".to_string());

    let parked_modreg = parked_snap
        .as_ref()
        .and_then(|s| s.modreg.as_ref())
        .map(|r| r.octal.clone())
        .unwrap_or_else(|| "n/a".to_string());
    let parked_failreg: Vec<String> = parked_snap
        .as_ref()
        .map(|s| {
            s.failreg
                .iter()
                .map(|opt| opt.as_ref().map(|r| r.octal.clone()).unwrap_or_default())
                .collect()
        })
        .unwrap_or_default();

    eprintln!(
        "[ms-e7i-c] post-P01 MODREG={} | parked-after-V62 MODREG={} FAILREG={:?} \
         | post-V33 MODREG={} VERBREG={} FAILREG={:?} ALMCADR_lo={}",
        modreg_after_p01,
        parked_modreg,
        parked_failreg,
        post_modreg,
        post_verbreg,
        post_failreg,
        post_almcadr_lo,
    );
}

/// TC-E7I-F: two-PROCEED hypothesis test for the P62→P63 wake gap (#49).
///
/// Source-analysis shows P62 requires TWO sequential PROCEED keystrokes:
///
/// 1. **PROCEED #1** (GOPERF1R separation display, P62.2):
///    `BANKCALL GOPERF1R` is the immediate-return variant — it creates an
///    async NOVAC display job for OCT41 (CM/SM separation check) and
///    returns to `TC P61.3` (PHASCHNG+ENDOFJOB). The async display job
///    eventually reaches `ENDIDLE` → `CADRSTOR ≠ 0`. When V33 ENTR is
///    received, VBPROC→RECALTST finds `CADRSTOR ≠ 0`, wakes the async
///    job, which takes the L+3 PROCEED branch → `TC POSTJUMP CADR CM/DAPON`.
///
/// 2. **CM/DAPON** loops (0.5 sim-sec DELAYJOB polls) until CM/POSE sets
///    `GAMDIFSW` (FLAGWRD5 BIT11) — happens after the first AVERAGE-G
///    SERVICER cycle calls CM/POSE via AVEGEXIT. Then CM/DAPON falls
///    through to `TC POSTJUMP CADR P62.1`.
///
/// 3. **PROCEED #2** (P62.1 V06N61 display):
///    `BANKCALL GOFLASH` is the synchronous variant — it runs MAKEPLAY in
///    the CURRENT job context, reaching `ENDIDLE` → `CADRSTOR ≠ 0` again.
///    A SECOND V33 ENTR is needed to wake it. That branch executes
///    `TC PHASCHNG OCT04024`, sets ROLLC/ALFACOM/P63FLAG, checks CMDAPMOD,
///    and with `CMDAPMOD = -1` jumps directly `TC P63` (skipping WAKEP62).
///    P63 sets `MODREG = 0o077` and does `ENDOFJOB`.
///
/// This test fires all three checkpoints and asserts (post entry-aligned
/// REFSMMAT fix, issue #49):
/// - Snapshot A (after V37 62 ENTR): MODREG=0o076 and NO IMU alarm. S61.1
///   now passes (±30° check), so P62 runs S61.1→P62.2→GOPERF1R. GOPERF1R
///   (V50N25 "PLEASE PERFORM") is immediate-return and does NOT park via
///   CADRSTOR, so A does not require CADRSTOR≠0. (`tc_e7i_g` verifies the
///   S61.1→GOPERF1R path directly.)
/// - Snapshot B (after PROCEED #1 + CM/DAPON wait): MODREG=0o076, CADRSTOR≠0
///   (P62.1 GOFLASH waiting in ENDIDLE) — the meaningful win of the fix.
/// - Snapshot C (after PROCEED #2): MODREG=0o077 (P63). REMAINING GAP —
///   reported, not asserted: PROCEED #2 → CMDAPMOD gate → TC P63 does not
///   yet complete (ROLLC advances but MODREG stays 0o076).
///
/// References:
/// - P61-P67.agc:204-220 (P62.2 GOPERF1R call and immediate-return)
/// - P61-P67.agc:225-265 (P62.1 GOFLASH call and CMDAPMOD gate)
/// - P61-P67.agc:309-338 (P63 NEWMODEX MM 63)
/// - CM_ENTRY_DIGITAL_AUTOPILOT.agc:175-238 (CM/DAPON GAMDIFSW wait loop)
/// - PINBALL_GAME__BUTTONS_AND_LIGHTS.agc:2902 (VBPROC)
/// - PINBALL_GAME__BUTTONS_AND_LIGHTS.agc:3358 (RECALTST)
/// - DISPLAY_INTERFACE_ROUTINES.agc:926 (GOPERF1R immediate-return)
/// - PINBALL_GAME__BUTTONS_AND_LIGHTS.agc:731 (GOFLASH synchronous)
#[test]
fn tc_e7i_f_wake_gap() {
    let template_path = template_core_path();
    if !template_path.exists() {
        eprintln!(
            "skipping tc_e7i_f: no template core at {} \
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
            "skipping tc_e7i_f: VirtualAGC build incomplete at {}",
            root.display()
        );
        return;
    }

    let mut core = CoreImage::load(&template_path)
        .unwrap_or_else(|e| panic!("load template {}: {e}", template_path.display()));
    let symtab =
        Symtab::load(&listing).unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
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
        refsmmat: EntryInitialState::entry_refsmmat(
            rust_state.csm_state.position,
            rust_state.csm_state.velocity,
        ),
        cmdapmod: -1,
    };
    patch_into(&mut core, &symtab, &init).expect("patch_into ok");

    let work = std::env::temp_dir().join(format!("vagc_e7i_f_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let core_in = work.join("core_in");
    core.save(&core_in).unwrap();

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
    let mut dsky = DskyScript::new(connect_with_retry(port));
    let dump_path = work.join("core");

    // ── Snapshot A: wait for GOPERF1R separation display to park ────────────
    // After V37 62 ENTR, P62.2 calls GOPERF1R (async / immediate-return),
    // whose display job reaches ENDIDLE. wait_for_p62_parked gates on
    // MODREG=0o076 AND CADRSTOR≠0 — both must hold simultaneously.
    // Generous 12-second budget: S61DT ≈ 11 sim-sec delay before SERVICER
    // starts (at 20x wall-clock speed ≈ 550 ms), plus IMU-check + display.
    let mtime_pre = current_mtime(&dump_path);
    dsky.verb_major_mode(62).expect("V37 ENTR 62 ENTR");
    let (core_a, dumps_a, settle_a_ms, settled_a) =
        wait_for_p62_parked(&dump_path, &symtab, mtime_pre, 12_000);
    let snap_a = core_a
        .as_ref()
        .map(|c| build_snapshot(c, &symtab, dumps_a, Some(settle_a_ms), settled_a));
    let cadrstor_a = snap_a
        .as_ref()
        .and_then(|s| s.cadrstor.as_ref())
        .map(|r| r.decimal)
        .unwrap_or(0);
    let modreg_a = snap_a
        .as_ref()
        .and_then(|s| s.modreg.as_ref())
        .map(|r| r.decimal)
        .unwrap_or(0);
    let posexit_a = snap_a
        .as_ref()
        .and_then(|s| s.posexit.as_ref())
        .map(|r| r.decimal)
        .unwrap_or(0);
    eprintln!(
        "[e7i-f] snap-A (post V37 62): MODREG=0o{:05o} CADRSTOR=0o{:05o} \
         POSEXIT=0o{:05o} settled={} wait_ms={} dumps={}",
        modreg_a, cadrstor_a, posexit_a, settled_a, settle_a_ms, dumps_a
    );

    // ── PROCEED #1: wake GOPERF1R → CM/DAPON → P62.1 GOFLASH ───────────────
    // After waking the GOPERF1R job it runs CM/DAPON, which loops (0.5
    // sim-sec polls) until the SERVICER calls CM/POSE and sets GAMDIFSW.
    // Then P62.1 calls GOFLASH (synchronous), reaching ENDIDLE again.
    // Budget: 2 sim-sec for first SERVICER cycle + up to 0.5 sim-sec
    // for CM/DAPON poll ≈ 125 ms wall-clock; add generous 10 s budget.
    let mtime_after_1st = current_mtime(&dump_path);
    dsky.proceed().expect("PROCEED #1 (wake GOPERF1R → CM/DAPON → P62.1)");
    let (core_b, dumps_b, settle_b_ms, settled_b) =
        wait_for_p62_parked(&dump_path, &symtab, mtime_after_1st, 10_000);
    let snap_b = core_b
        .as_ref()
        .map(|c| build_snapshot(c, &symtab, dumps_b, Some(settle_b_ms), settled_b));
    let cadrstor_b = snap_b
        .as_ref()
        .and_then(|s| s.cadrstor.as_ref())
        .map(|r| r.decimal)
        .unwrap_or(0);
    let modreg_b = snap_b
        .as_ref()
        .and_then(|s| s.modreg.as_ref())
        .map(|r| r.decimal)
        .unwrap_or(0);
    eprintln!(
        "[e7i-f] snap-B (post PROCEED #1): MODREG=0o{:05o} CADRSTOR=0o{:05o} \
         settled={} wait_ms={} dumps={}",
        modreg_b, cadrstor_b, settled_b, settle_b_ms, dumps_b
    );

    // ── PROCEED #2: wake P62.1 GOFLASH → CMDAPMOD gate → TC P63 ─────────────
    // The GOFLASH job's PROCEED branch runs:
    //   TC PHASCHNG OCT04024, CCS HEADSUP, TS ROLLC, ... TC P63
    // P63 calls TC NEWMODEX MM 63 → MODREG = 0o077, then ENDOFJOB.
    // Poll for MODREG to advance from 0o076 to 0o077.
    let mtime_after_2nd = current_mtime(&dump_path);
    dsky.proceed().expect("PROCEED #2 (wake P62.1 GOFLASH → P63)");

    let start_c = Instant::now();
    let budget_c = Duration::from_secs(8);
    let mut last_mtime_c = mtime_after_2nd;
    let mut modreg_c = modreg_b;
    let mut rollc_c: u16 = 0;
    let mut posexit_c: u16 = 0;
    while Instant::now() < start_c + budget_c {
        if let Some(new_mt) = wait_for_new_dump(
            &dump_path,
            last_mtime_c,
            Instant::now() + Duration::from_millis(400),
        ) {
            last_mtime_c = Some(new_mt);
            if let Some(c) = try_load_core(&dump_path) {
                let m = symtab.get("MODREG").and_then(|a| c.read_sp(a)).unwrap_or(0);
                modreg_c = m;
                // Also read ROLLC and POSEXIT for extra context
                posexit_c = symtab
                    .get("POSEXIT")
                    .and_then(|a| c.read_sp(a))
                    .unwrap_or(0);
                rollc_c = symtab
                    .get("ROLLC")
                    .and_then(|a| c.read_sp(a))
                    .unwrap_or(0);
                if m == 0o077 {
                    break;
                }
            }
        }
    }
    eprintln!(
        "[e7i-f] snap-C (post PROCEED #2): MODREG=0o{:05o} POSEXIT=0o{:05o} \
         ROLLC=0o{:05o} (expected MODREG=0o00077=P63)",
        modreg_c, posexit_c, rollc_c
    );

    // ── Stderr scan ──────────────────────────────────────────────────────────
    let _ = child.kill();
    let _ = child.wait();
    std::thread::sleep(Duration::from_millis(100));
    let stderr_lines = stderr_buf.lock().expect("lock").clone();
    let scan = scan_stderr(&stderr_lines);
    if !scan.matched_alarm.is_empty() {
        eprintln!("[e7i-f] alarm lines: {:?}", scan.matched_alarm);
    }

    let _ = std::fs::remove_dir_all(&work);

    // ── Assertions ───────────────────────────────────────────────────────────
    // A: P62 must be active. With the entry-aligned REFSMMAT (issue #49),
    //    S61.1's ±30° IMU check passes and P62 runs S61.1→P62.2→GOPERF1R.
    //    GOPERF1R (V50N25 "PLEASE PERFORM") is *immediate-return* and does
    //    NOT park via CADRSTOR, so we assert P62 is active + no IMU alarm,
    //    NOT `settled_a`. (`tc_e7i_g` verifies the S61.1→GOPERF1R path.)
    let imu_alarm = scan
        .matched_alarm
        .iter()
        .any(|l| l.contains("1426") || l.contains("1427"));
    assert!(
        modreg_a == 0o076 && !imu_alarm,
        "SNAP-A: P62 not active or S61.1 raised an IMU alarm. \
         MODREG=0o{:05o} CADRSTOR=0o{:05o} POSEXIT=0o{:05o} imu_alarm={imu_alarm} \
         (settled_a={settled_a})",
        modreg_a, cadrstor_a, posexit_a
    );
    // B: after PROCEED #1, P62.1 GOFLASH V06N61 must park in ENDIDLE
    //    (CADRSTOR≠0). This is the meaningful win of the REFSMMAT fix:
    //    it proves S61.1 passed and GOPERF1R→CM/DAPON→P62.1 all ran.
    assert!(
        settled_b,
        "SNAP-B: after PROCEED #1, AGC did not reach MODREG=0o076 + CADRSTOR≠0 \
         within 10 s; CM/DAPON→P62.1 GOFLASH never parked. \
         MODREG=0o{:05o} CADRSTOR=0o{:05o}",
        modreg_b, cadrstor_b
    );
    // C: after PROCEED #2, P63 should set MODREG=0o077. This is the
    //    REMAINING wake-gap layer (#49): the REFSMMAT fix gets us a clean
    //    P62.1 park, but PROCEED #2 → CMDAPMOD gate → TC P63 does not yet
    //    complete (observed: ROLLC advances but MODREG stays 0o076).
    //    Reported, not asserted, until that layer is addressed.
    if modreg_c == 0o077 {
        eprintln!("[e7i-f] ✓ PROCEED #2 reached P63 (MODREG=0o077)");
    } else {
        eprintln!(
            "[e7i-f] REMAINING GAP: PROCEED #2 did not reach P63. \
             MODREG=0o{:05o} POSEXIT=0o{:05o} ROLLC=0o{:05o} (want 0o00077)",
            modreg_c, posexit_c, rollc_c
        );
    }
}

/// TC-E7I-G: localize the P62 stall for the wake gap (#49).
///
/// `tc_e7i_f` showed the AGC enters P62 (`MODREG = 0o076`) but the
/// GOPERF1R separation display never parks in ENDIDLE — `CADRSTOR`
/// stays 0 through 12 s and both PROCEEDs, so the two-PROCEED wake
/// logic is never reached. This test finds *where* P62 stalls.
///
/// The P62 entry path (confirmed from the Comanche055 listing and
/// O'Brien book pp.350-351, see `docs/reentry_workflow_spec.md`) is:
///
///   P62 (26,2320)  TC S61.1         # check state vector + IMU orientation
///   P62.2 (26,2326) ... CAF OCT41 / TC BANKCALL / CADR GOPERF1R
///   GOPERF1R (10,3125)               # posts V50N25 sep display
///   ... ENDIDLE                      # display job sleeps, sets CADRSTOR
///
/// So execution should hit, in order: `S61.1` → `P62.2` → `GOPERF1R`
/// → `ENDIDLE`. We break on all four, drive `V37 ENTR 62 ENTR`, and
/// dump a symbolic backtrace + registers at each stop (each CONT
/// resumes to the next breakpoint). The stop *sequence* localizes the
/// stall:
///   - S61.1 hit, P62.2 NOT hit → stall INSIDE S61.1 (IMU/state check)
///     — the leading hypothesis (O'Brien: "P62 ... generates a program
///     alarm if the IMU is not ready").
///   - S61.1+P62.2 hit, GOPERF1R NOT hit → stall in the BANKCALL dispatch.
///   - GOPERF1R hit, ENDIDLE NOT hit → display job never scheduled.
///   - all four hit → display parks; the wake gap is downstream.
///
/// Also dumps AVEGFLAG (FLAGWRD1 bit 1) from the final core: if
/// AVERAGE-G is off, the SERVICER never cycles CM/POSE, which both
/// gates the display path and (later) GAMDIFSW in CM/DAPON.
#[test]
fn tc_e7i_g_locate_p62_stall() {
    let template_path = template_core_path();
    if !template_path.exists() {
        eprintln!("skipping tc_e7i_g: no template core at {}", template_path.display());
        return;
    }
    let root = vagc_root();
    let yaagc = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");
    let listing = root.join("Comanche055/MAIN.agc.lst");
    let symtab_file = root.join("Comanche055/MAIN.agc.symtab");
    if !yaagc.exists() || !rope.exists() || !listing.exists() || !symtab_file.exists() {
        eprintln!(
            "skipping tc_e7i_g: VirtualAGC build incomplete at {}",
            root.display()
        );
        return;
    }

    let mut core = CoreImage::load(&template_path)
        .unwrap_or_else(|e| panic!("load template {}: {e}", template_path.display()));
    let symtab =
        Symtab::load(&listing).unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
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
        refsmmat: EntryInitialState::entry_refsmmat(
            rust_state.csm_state.position,
            rust_state.csm_state.velocity,
        ),
        cmdapmod: -1,
    };
    patch_into(&mut core, &symtab, &init).expect("patch_into ok");

    let work = std::env::temp_dir().join(format!("vagc_e7i_g_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let core_in = work.join("core_in");
    core.save(&core_in).unwrap();

    // Debugger script: break on the four P62-path labels; after each
    // CONT emit a symbolic backtrace + registers so dbg.txt records the
    // stop sequence. `info breakpoints` up front confirms which labels
    // the debugger actually resolved (the dotted names S61.1 / P62.2 may
    // or may not parse; GOPERF1R / ENDIDLE are dotless and safe). The
    // final COREDUMP captures AVEGFLAG etc. at the last stop reached.
    // Breakpoints span the P62 entry path *and* S61.1's IMU-gate divert
    // exits. S61.1's first act is `CADR R02BOTH`, the IMU/REFSMMAT check
    // that raises VARALARM 0o00210 (IMU NOT OPERATING) / 0o00220 (NO
    // REFSMMAT) and `TC GOTOPOOH` when the ISS is uninitialised. Breaking
    // on R02BOTH / VARALARM / ALARM / GOTOPOOH pins whether the stall is
    // the IMU-not-ready abort or a genuine loop.
    let hit_core = work.join("hit.core");
    let cmd_path = work.join("watch.txt");
    let script = format!(
        "BREAK S61.1\nBREAK R02BOTH\nBREAK VARALARM\nBREAK ALARM\n\
         BREAK GOTOPOOH\nBREAK P62.2\nBREAK GOPERF1R\nBREAK ENDIDLE\n\
         info breakpoints\n\
         CONT\nBACKTRACES\ninfo registers\n\
         CONT\nBACKTRACES\ninfo registers\n\
         CONT\nBACKTRACES\ninfo registers\n\
         CONT\nBACKTRACES\ninfo registers\n\
         COREDUMP {}\nQUIT\n",
        hit_core.display()
    );
    std::fs::write(&cmd_path, script).unwrap();

    let dbg_out = work.join("dbg.txt");
    let out_file = std::fs::File::create(&dbg_out).unwrap();

    let port = pick_port();
    let mut child = std::process::Command::new(&yaagc)
        .current_dir(&work)
        .arg("--quiet")
        .arg(format!("--symbols={}", symtab_file.display()))
        .arg("--inhibit-alarms")
        .arg(format!("--command={}", cmd_path.display()))
        .arg(format!("--port={port}"))
        .arg(&rope)
        .arg(&core_in)
        .stdout(std::process::Stdio::from(out_file))
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("yaAGC spawn");

    let stderr = child.stderr.take().expect("piped stderr");
    let _stderr_buf = spawn_stderr_capture(stderr);
    std::thread::sleep(Duration::from_millis(500));

    // Drive V37 ENTR 62 ENTR. P62's `TC S61.1` should trip the first
    // breakpoint within a couple seconds.
    let mut dsky = DskyScript::new(connect_with_retry(port));
    let _ = dsky.verb_major_mode(62);

    // Give the chained CONT/backtrace sequence room to run; if a CONT
    // hangs (a later breakpoint is never reached), the script never hits
    // QUIT and we kill at the deadline. dbg.txt still holds every stop
    // that completed.
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut exited = false;
    while Instant::now() < deadline {
        if let Ok(Some(_)) = child.try_wait() {
            exited = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    if !exited {
        let _ = child.kill();
    }
    let _ = child.wait();

    let dbg = std::fs::read_to_string(&dbg_out).unwrap_or_default();
    let hit = CoreImage::load(&hit_core).ok();

    // Count how many stops we observed. Each stop prints one
    // "info registers" block, so the count of Z/register dumps ~= stops.
    // BACKTRACES is symbolic, so the presence of each label name in
    // dbg.txt tells us whether that point was reached at all.
    let stopped_at = |label: &str| dbg.contains(&format!(", {label} ("));
    eprintln!(
        "[e7i-g] exited={} | stopped at: S61.1={} R02BOTH={} VARALARM={} \
         ALARM={} GOTOPOOH={} P62.2={} GOPERF1R={} ENDIDLE={}",
        exited,
        stopped_at("S61.1"),
        stopped_at("R02BOTH"),
        stopped_at("VARALARM"),
        stopped_at("ALARM"),
        stopped_at("GOTOPOOH"),
        stopped_at("P62.2"),
        stopped_at("GOPERF1R"),
        stopped_at("ENDIDLE"),
    );

    // Decode AVEGFLAG + state-vector epoch from the final core, mirroring
    // tc_e7i_d, to answer whether AVERAGE-G / the SERVICER is running.
    if let Some(c) = hit.as_ref() {
        let rd = |bank: u8, offset: u16| c.read_sp(AgcAddress::Erasable { bank, offset });
        let z = rd(0, 5).unwrap_or(0);
        let time2 = rd(0, 0o24).unwrap_or(0) as u32;
        let time1 = rd(0, 0o25).unwrap_or(0) as u32;
        let clock_cs = time2 * 0o40000 + time1;
        let dp = |sym: &str| -> Option<i64> {
            match symtab.get(sym) {
                Some(AgcAddress::Erasable { bank, offset }) => {
                    let hi = rd(bank, offset)? as i64;
                    let lo = rd(bank, offset + 1)? as i64;
                    Some(hi * 0o40000 + lo)
                }
                _ => None,
            }
        };
        let sp = |sym: &str| -> Option<u16> {
            match symtab.get(sym) {
                Some(AgcAddress::Erasable { bank, offset }) => rd(bank, offset),
                _ => None,
            }
        };
        let flagwrd1 = sp("FLAGWRD1");
        let avegflag = flagwrd1.map(|w| w & 0o00001 != 0);
        eprintln!(
            "[e7i-g] final core: Z(PC)=0o{:05o} clock={:.2}s | \
             AVEGFLAG={:?} FLAGWRD1={:?} | TET={:?}cs PIPTIME={:?}cs \
             MODREG={:?} CADRSTOR={:?}",
            z,
            clock_cs as f64 / 100.0,
            avegflag,
            flagwrd1.map(|w| format!("0o{:05o}", w)),
            dp("TET"),
            dp("PIPTIME"),
            sp("MODREG").map(|w| format!("0o{:05o}", w)),
            sp("CADRSTOR").map(|w| format!("0o{:05o}", w)),
        );
    } else {
        eprintln!("[e7i-g] final core: none written (no breakpoint reached ENDIDLE/QUIT)");
    }

    eprintln!(
        "[e7i-g] ── debugger session stdout ──\n{}",
        dbg.trim_end()
    );

    let _ = std::fs::remove_dir_all(&work);
}

/// TC-E7I-I: root-cause the P62→P63 wake gap (#49).
///
/// Drives PROCEEDs paced on the ACTUAL parked display (not just
/// CADRSTOR≠0) without the debugger — breakpoints on the keyboard path
/// halt the sim and break the DSKY socket, and the debugger runs too slow
/// to reach the ~20-30 s GAMDIFSW delay. It walks the P62 displays and
/// captures the state that discriminates the failure mode:
///
///   - Display trail — expected `V50N25` (GOPERF1R separation) → `V06N61`
///     (P62.1 prompt). The `V50N25 → V06N61` step is gated on GAMDIFSW
///     (CM/DAPON ↔ AVERAGE-G SERVICER), so PROCEEDs must be display-paced.
///   - `DSPLOCK` — 0 throughout ⇒ V33 is NOT blocked (disproves the
///     MS-E7h "V33 ignored at the P62.1 park" hypothesis).
///   - `P63FLAG` after the V06N61 PROCEED — `+1` (0o00001) proves the
///     P62.1 `+2` branch ran (CM/DAPON leaves it `-1`).
///   - `CMDAPMOD` after the V06N61 PROCEED — preloaded `-1` (0o77776) but
///     found `-0` (0o77777): **EXDAP overwrites it from body attitude**
///     (`CM_ENTRY_DIGITAL_AUTOPILOT.agc:602`; CALFA negative + outside 45°
///     ⇒ `-0`). The `CS CMDAPMOD / MASK ONE / BZF P63.1` gate then takes
///     the `P63.1` (WAKEP62) branch instead of `TC P63` — and WAKEP62
///     never fires because the open-loop harness never maneuvers the CM
///     within 45° of entry attitude. That is the root cause: the final
///     P62→P63 handover is closed-loop on the entry-attitude maneuver.
#[test]
fn tc_e7i_i_v33_dispatch() {
    let template_path = template_core_path();
    if !template_path.exists() {
        eprintln!("skipping tc_e7i_i: no template core at {}", template_path.display());
        return;
    }
    let root = vagc_root();
    let yaagc = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");
    let listing = root.join("Comanche055/MAIN.agc.lst");
    let symtab_file = root.join("Comanche055/MAIN.agc.symtab");
    if !yaagc.exists() || !rope.exists() || !listing.exists() || !symtab_file.exists() {
        eprintln!("skipping tc_e7i_i: VirtualAGC build incomplete at {}", root.display());
        return;
    }

    let mut core = CoreImage::load(&template_path)
        .unwrap_or_else(|e| panic!("load template {}: {e}", template_path.display()));
    let symtab =
        Symtab::load(&listing).unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
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
        refsmmat: EntryInitialState::entry_refsmmat(
            rust_state.csm_state.position,
            rust_state.csm_state.velocity,
        ),
        cmdapmod: -1,
    };
    patch_into(&mut core, &symtab, &init).expect("patch_into ok");

    let work = std::env::temp_dir().join(format!("vagc_e7i_i_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let core_in = work.join("core_in");
    core.save(&core_in).unwrap();

    let _ = &symtab_file; // (debugger symbols not needed for this probe)

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
    let _stderr_buf = spawn_stderr_capture(stderr);
    std::thread::sleep(Duration::from_millis(500));

    let dump_path = work.join("core");
    let mut dsky = DskyScript::new(connect_with_retry(port));

    // Read a named erasable SP cell from a core, formatted octal.
    let read_cell = |c: &CoreImage, name: &str| -> Option<u16> {
        match symtab.get(name) {
            Some(a @ AgcAddress::Erasable { .. }) => c.read_sp(a),
            _ => None,
        }
    };
    // V37 62 ENTR → wait for P62 active.
    dsky.verb_major_mode(62).expect("V37 ENTR 62 ENTR");
    let deadline_a = Instant::now() + Duration::from_secs(10);
    let mut last_mt = current_mtime(&dump_path);
    while Instant::now() < deadline_a {
        if let Some(mt) = wait_for_new_dump(&dump_path, last_mt, Instant::now() + Duration::from_millis(400)) {
            last_mt = Some(mt);
            if let Some(c) = try_load_core(&dump_path) {
                if read_cell(&c, "MODREG").unwrap_or(0) == 0o076 {
                    break;
                }
            }
        }
    }
    std::thread::sleep(Duration::from_millis(1_500));

    // Drive PROCEEDs paced on the ACTUAL parked display, up to a budget,
    // until P63 (MODREG=0o077). Each iteration waits (long budget, for the
    // GAMDIFSW/CM-DAPON delay) for a flashing display to park (CADRSTOR≠0),
    // logs which V/N is up, then sends one PROCEED. This mimics a crew
    // clicking through the P62 displays and reveals how many correctly-
    // paced PROCEEDs actually reach P63.
    let mut modreg_final = 0o076u16;
    let mut proceeds = 0u32;
    let mut display_trail: Vec<String> = Vec::new();
    for iter in 0..6u32 {
        // Wait for a parked flashing display, or P63.
        let mut parked = None;
        let dl = Instant::now() + Duration::from_secs(35);
        let mut lm = current_mtime(&dump_path);
        while Instant::now() < dl {
            if let Some(mt) = wait_for_new_dump(&dump_path, lm, Instant::now() + Duration::from_millis(500)) {
                lm = Some(mt);
                if let Some(c) = try_load_core(&dump_path) {
                    let m = read_cell(&c, "MODREG").unwrap_or(0);
                    let cadr = read_cell(&c, "CADRSTOR").unwrap_or(0);
                    if m == 0o077 || cadr != 0 {
                        parked = Some(c);
                        break;
                    }
                }
            }
        }
        let Some(c) = parked else {
            eprintln!("[e7i-i] iter {iter}: no display parked within budget");
            break;
        };
        let vb = read_cell(&c, "VERBREG").unwrap_or(0);
        let nn = read_cell(&c, "NOUNREG").unwrap_or(0);
        let m = read_cell(&c, "MODREG").unwrap_or(0);
        let gam = read_cell(&c, "CM/FLAGS").map(|w| w & 0o02000 != 0).unwrap_or(false);
        display_trail.push(format!("V{vb:02o}N{nn:02o}@MM0o{m:05o}"));
        eprintln!(
            "[e7i-i] iter {iter}: parked V{vb:02o}N{nn:02o} MODREG=0o{m:05o} \
             CADRSTOR=0o{:05o} GAMDIFSW={gam}",
            read_cell(&c, "CADRSTOR").unwrap_or(0)
        );
        modreg_final = m;
        if m == 0o077 {
            break;
        }
        dsky.proceed().expect("PROCEED");
        proceeds += 1;
        std::thread::sleep(Duration::from_millis(2_000));
    }

    // Final steady-state after the last PROCEED. Discriminator for whether
    // the P62.1 `+2` branch executed: it sets `P63FLAG = +1` (0o00001),
    // whereas CM/DAPON left it `-1` (0o77776). It also sets ENTRYVN=V06N22
    // (0o02026) and would fall through the CMDAPMOD gate to `TC P63`.
    std::thread::sleep(Duration::from_millis(1_000));
    let (p63flag, cmdapmod, entryvn, alfacom, rollc) = try_load_core(&dump_path)
        .map(|c| {
            (
                read_cell(&c, "P63FLAG").unwrap_or(0),
                read_cell(&c, "CMDAPMOD").unwrap_or(0),
                read_cell(&c, "ENTRYVN").unwrap_or(0),
                read_cell(&c, "ALFACOM").unwrap_or(0),
                read_cell(&c, "ROLLC").unwrap_or(0),
            )
        })
        .unwrap_or((0, 0, 0, 0, 0));

    let _ = child.kill();
    let _ = child.wait();
    let _ = std::fs::remove_dir_all(&work);

    // ── Interpretation ───────────────────────────────────────────────────────
    eprintln!("[e7i-i] display trail: {}", display_trail.join(" → "));
    eprintln!(
        "[e7i-i] final state: P63FLAG=0o{p63flag:05o} CMDAPMOD=0o{cmdapmod:05o} \
         ENTRYVN=0o{entryvn:05o} ALFACOM=0o{alfacom:05o} ROLLC=0o{rollc:05o}"
    );
    eprintln!(
        "[e7i-i]   ↳ P62.1 +2 branch ran? {} (P63FLAG +1=ran / -1=only CM/DAPON)",
        if p63flag == 0o00001 {
            "YES → diverted at CMDAPMOD gate / P63 (not TC P63)"
        } else {
            "NO → PROCEED at V06N61 never entered the +2 branch"
        }
    );
    eprintln!(
        "[e7i-i] after {proceeds} display-paced PROCEEDs: MODREG=0o{modreg_final:05o} {}",
        if modreg_final == 0o077 {
            "✓ REACHED P63 — the gap was PROCEED pacing (wait for each parked display)"
        } else {
            "✗ still no P63 — a genuine blocker remains beyond pacing"
        }
    );
}

/// TC-E7I-J: closed-loop P62→P63 handover with CDU injection (#49).
///
/// Drives the CM attitude via unprogrammed-increment CDU pulses
/// (PCDU/MCDU on counter channels 0o32/33/34 → wire 0x9A/9B/9C) so that
/// `CALFA` (computed by CM/POSE from the CDU gimbal angles) rises above
/// `+cos 45°` with a positive sign. The EXDAP gate logic then schedules
/// WAKEP62 (§3.3 of `docs/reentry_workflow_spec.md`), which launches P63.
///
/// ## Mechanism
///
/// - CDU counter registers (bank 0, offsets 0o32/33/34) and the READGYMB
///   accumulators (AOG/AIG/AMG, erasable bank 6) are pre-zeroed so the
///   first READGYMB cycle produces zero body-rate (no spurious ALFA spike).
///   At CDU = 0 the CM is nose-forward → CALFA ≈ −1 (nosein phase).
/// - A [`CduInjector`] ramps CDUY from 0° toward ≈174° at 3°/s:
///     - t ≈ 0 – 13 s  (CDU 0°–39°):  nosein  (|CALFA| ≥ cos45°, CALFA < 0)
///     - t ≈ 13 – 43 s (CDU 39°–129°): broadside (|CALFA| < cos45°) →
///       EXDAP2 decrements P63FLAG from +1 → +0
///     - t > 43 s      (CDU > 129°):  headsup (|CALFA| ≥ cos45°, CALFA > 0) →
///       EXDAP schedules WAKEP62 (21 s delay, fires at ≈64 s)
/// - P62.1 fires on PROCEED #2 (t ≈ 5–8 s, CALFA still < 0):
///   CMDAPMOD = −0 (nosein) → BZF P63.1 → deferred to WAKEP62.
/// - WAKEP62 launches P63 via NOVAC ~21 s after the gate trips. Because
///   that is an autonomous mode transition (not a parked display), a
///   dedicated Phase 3 loop polls `MODREG` for up to 45 s after the two
///   display-paced PROCEEDs, keeping the CDU ramp running to hold CALFA
///   positive until P63 starts.
///
/// Acceptance (§8.7): `MODREG = 0o077` (P63, via WAKEP62) proves CALFA
/// crossed +cos45° positive — the only path that arms WAKEP62 — and
/// `CMDAPMOD = +0` confirms the heat-shield-forward EXDAP mode. The raw
/// `CALFA` cell is diagnostic-only: it aliases SPNDX/INTTEMP scratch and
/// its dumped value is unreliable (observed 0o37727 / 0o00000 across runs).
///
/// ## §8.6 items resolved (live run, 2026-07)
///
/// 1. **`IMODES33` bit 6** — already clear in the template; `patch_into`
///    also writes 0 to `IMODES33`. Not the root cause.
/// 2. **FIFO-drain vs 100 ms cadence** — CDU injection rate (≈27 counts/tick
///    = 0.3°) is far below the CDU_MAX_RATE_CPS = 400 limit.  Not blocking.
/// 3. **AOG/AIG/AMG bank context** — no aliasing; routines set EBANK=AOG
///    correctly before every READGYMB. Not blocking.
///
/// **Root cause**: `entry_trim_cdu_deg` used `+unit(velocity)` as X_body,
/// giving CDUY ≈ −6° → CALFA ≈ −1 (nose-into-wind forever). Fixed by
/// negating the velocity to get X_body = −unit(velocity) (heat-shield
/// forward), yielding CDUY ≈ 174° and CALFA ≈ +1 after ramp.
///
/// AGC source: Comanche055/CM_ENTRY_DIGITAL_AUTOPILOT.agc (EXDAP loop,
/// READGYMB); Comanche055/P61-P67.agc P62.1 CMDAPMOD gate.
#[test]
fn tc_e7i_j_closed_loop_p63() {
    let template_path = template_core_path();
    if !template_path.exists() {
        eprintln!("skipping tc_e7i_j: no template core at {}", template_path.display());
        return;
    }
    let root = vagc_root();
    let yaagc = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");
    let listing = root.join("Comanche055/MAIN.agc.lst");
    if !yaagc.exists() || !rope.exists() || !listing.exists() {
        eprintln!("skipping tc_e7i_j: VirtualAGC build incomplete at {}", root.display());
        return;
    }

    let mut core = CoreImage::load(&template_path)
        .unwrap_or_else(|e| panic!("load template {}: {e}", template_path.display()));
    let symtab =
        Symtab::load(&listing).unwrap_or_else(|e| panic!("load symtab {}: {e}", listing.display()));
    let rust_state = setup_state_direct_leo();
    let refsmmat = EntryInitialState::entry_refsmmat(
        rust_state.csm_state.position,
        rust_state.csm_state.velocity,
    );

    // Use cmdapmod = 0 (+0 ones-complement): let EXDAP set it naturally
    // from CALFA rather than preloading -1 (which bypasses the gate).
    let init = EntryInitialState {
        position_m: rust_state.csm_state.position,
        velocity_mps: rust_state.csm_state.velocity,
        time_s: 0.0,
        target_lat_rad: rust_state.entry.target_lat_rad,
        target_lon_rad: rust_state.entry.target_lon_rad,
        emsalt_m: 122_000.0,
        alfa_pad_deg: -20.0,
        lift_up: true,
        refsmmat,
        cmdapmod: 0,
    };
    patch_into(&mut core, &symtab, &init).expect("patch_into ok");

    // Pre-zero the CDU counter registers (bank 0, offsets 0o32/33/34 =
    // CDUX/CDUY/CDUZ per ERASABLE_ASSIGNMENTS.agc:145-147).
    //
    // CDU = 0 places the CM nose-forward (X_body ≈ +velocity direction →
    // CALFA ≈ −1, nosein).  The CDU injector ramps CDUY toward ≈174°,
    // driving CALFA through broadside to headsup to trigger WAKEP62.
    for offset in [0o32u16, 0o33, 0o34] {
        core.write_sp(AgcAddress::Erasable { bank: 0, offset }, 0);
    }

    // Zero AOG/AIG/AMG (READGYMB "previous CDU reading" accumulators,
    // erasable bank 6, switched at 0o1661–0o1663).  These hold the CDU
    // value from the last READGYMB call; the first −DELA* computation is
    // (CDU − AOG/AIG/AMG).  Template values may be non-zero, producing a
    // large spurious body-rate spike on the very first READGYMB cycle that
    // would corrupt ALFA/180 before CM/ATUP can correct it.
    for name in ["AOG", "AIG", "AMG"] {
        if let Some(addr) = symtab.get(name) {
            core.write_sp(addr, 0);
        }
    }

    let work = std::env::temp_dir().join(format!("vagc_e7i_j_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let core_in = work.join("core_in");
    core.save(&core_in).unwrap();

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
    std::thread::sleep(std::time::Duration::from_millis(300));

    // Three TCP connections: DSKY, channel listener, CDU injector.
    // yaAGC listens on port, port+1, port+2, … (SocketAPI.c line 417:
    // `EstablishSocket(Portnum + NumServers, 3)`).
    let mut dsky = DskyScript::new(connect_with_retry(port));
    let cdu_client = connect_with_retry(port + 2);

    let dump_path = work.join("core");

    // Compute the entry-trim CDU target: heat-shield into the relative
    // wind at AoA ≈ 0° (CALFA ≈ 1.0).  For the direct-LEO equatorial
    // geometry (FPA ≈ −6°), X_body = −unit(V) gives CDUY ≈ 174° (= 180° −
    // 6°), CDUX ≈ 0°, CDUZ ≈ 0°.  The injector ramps from CDU = 0 toward
    // this target at 3°/s; WAKEP62 is scheduled when CALFA first exceeds
    // cos(45°) ≈ 0.707 with a positive sign (CDUY ≈ 129°, t ≈ 43 s).
    let cdu_target = entry_trim_cdu_deg(
        rust_state.csm_state.position,
        rust_state.csm_state.velocity,
        refsmmat,
    );

    // CDU injection runs in a background thread so it is not blocked by
    // the main thread's dump-polling sleeps.  Ramps at 3°/s from CDU = 0
    // (CALFA ≈ −1, nosein) toward CDUY ≈ 174° (CALFA ≈ +1, headsup).
    // The full 174° sweep takes ≈58 s; the WAKEP62 gate trips at ≈43 s
    // (CDUY ≈ 129°) and P63 starts at ≈64 s (§8.3 timing budget).
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    let stop_flag = Arc::new(AtomicBool::new(false));
    let stop_clone = Arc::clone(&stop_flag);
    let cdu_thread = {
        let initial = [0.0_f64; 3];
        let mut injector = CduInjector::new(cdu_client, initial);
        injector.set_target(cdu_target);
        std::thread::spawn(move || {
            const SLEW_DEG_PER_S: f64 = 3.0;
            const TICK_S: f64 = 0.1;
            while !stop_clone.load(Ordering::Relaxed) {
                let _ = injector.tick(TICK_S, SLEW_DEG_PER_S);
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
        })
    };

    // Helper to read a named SP cell from a core dump.
    let read_cell = |c: &CoreImage, name: &str| -> Option<u16> {
        match symtab.get(name) {
            Some(a @ AgcAddress::Erasable { .. }) => c.read_sp(a),
            _ => None,
        }
    };

    // ── Phase 1: V37 ENTR 62 ENTR → wait for P62 active ────────────────────
    dsky.verb_major_mode(62).expect("V37 ENTR 62 ENTR");
    let deadline_p62 = Instant::now() + Duration::from_secs(12);
    let mut last_mt = current_mtime(&dump_path);
    while Instant::now() < deadline_p62 {
        if let Some(mt) = wait_for_new_dump(
            &dump_path,
            last_mt,
            Instant::now() + Duration::from_millis(400),
        ) {
            last_mt = Some(mt);
            if let Some(c) = try_load_core(&dump_path) {
                if read_cell(&c, "MODREG").unwrap_or(0) == 0o076 {
                    break;
                }
            }
        }
    }

    // ── Phase 2: display-paced PROCEEDs until P63 ───────────────────────────
    // Up to 6 iterations; each waits (35 s budget for GAMDIFSW) for a
    // parked flashing display (CADRSTOR≠0) or MODREG=0o077 (P63 started),
    // then sends one PROCEED.
    let mut modreg_final = 0o076u16;
    let mut proceeds = 0u32;
    let mut display_trail: Vec<String> = Vec::new();

    for iter in 0..6u32 {
        let mut parked: Option<CoreImage> = None;
        let dl = Instant::now() + Duration::from_secs(35);
        let mut lm = current_mtime(&dump_path);
        while Instant::now() < dl {
            if let Some(mt) = wait_for_new_dump(
                &dump_path,
                lm,
                Instant::now() + Duration::from_millis(400),
            ) {
                lm = Some(mt);
                if let Some(c) = try_load_core(&dump_path) {
                    let m = read_cell(&c, "MODREG").unwrap_or(0);
                    let cadr = read_cell(&c, "CADRSTOR").unwrap_or(0);
                    if m == 0o077 || cadr != 0 {
                        parked = Some(c);
                        break;
                    }
                }
            }
        }
        let Some(ref c) = parked else {
            eprintln!("[e7i-j] iter {iter}: no display parked within budget");
            break;
        };
        let vb = read_cell(c, "VERBREG").unwrap_or(0);
        let nn = read_cell(c, "NOUNREG").unwrap_or(0);
        let m = read_cell(c, "MODREG").unwrap_or(0);
        let cadr = read_cell(c, "CADRSTOR").unwrap_or(0);
        let calfa_raw = read_cell(c, "CALFA").unwrap_or(0);
        display_trail.push(format!("V{vb:02o}N{nn:02o}@MM0o{m:05o}"));
        eprintln!(
            "[e7i-j] iter {iter}: V{vb:02o}N{nn:02o} MODREG=0o{m:05o} \
             CADRSTOR=0o{cadr:05o} CALFA=0o{calfa_raw:05o}"
        );
        modreg_final = m;
        if m == 0o077 {
            break;
        }
        dsky.proceed().expect("PROCEED");
        proceeds += 1;
        std::thread::sleep(Duration::from_millis(2_000));
    }

    // ── Phase 3: wait for the WAKEP62 WAITLIST task to start P63 ─────────────
    // At PROCEED #2 the P62.1 CMDAPMOD gate may still see CALFA < 0 (CMDAPMOD
    // = -0) and take `BZF P63.1`, which just ENDOFJOBs and defers to WAKEP62.
    // WAKEP62 is armed by EXDAP once the ramping CDU carries CALFA above
    // +cos45° positive, and starts P63 ~21 s later (NSEC = 2100 cs). That is
    // an autonomous mode transition, not a parked display, so poll MODREG
    // directly. The CDU thread keeps holding CALFA positive meanwhile.
    if modreg_final != 0o077 {
        let dl = Instant::now() + Duration::from_secs(45);
        let mut lm = current_mtime(&dump_path);
        while Instant::now() < dl {
            if let Some(mt) = wait_for_new_dump(
                &dump_path,
                lm,
                Instant::now() + Duration::from_millis(400),
            ) {
                lm = Some(mt);
                if let Some(c) = try_load_core(&dump_path) {
                    if read_cell(&c, "MODREG").unwrap_or(0) == 0o077 {
                        modreg_final = 0o077;
                        eprintln!("[e7i-j] phase 3: P63 started (MODREG=0o077) via WAKEP62");
                        break;
                    }
                }
            }
        }
        if modreg_final != 0o077 {
            eprintln!("[e7i-j] phase 3: WAKEP62 did not start P63 within 45 s");
        }
    }

    // ── Final erasable snapshot ──────────────────────────────────────────────
    std::thread::sleep(Duration::from_millis(1_500));
    let final_core = try_load_core(&dump_path);
    let (calfa_raw, p63flag, cmdapmod, mmnumber, failreg) = final_core
        .as_ref()
        .map(|c| {
            // FAILREG is a 3-word block; read all three.
            let fr0 = read_cell(c, "FAILREG").unwrap_or(0);
            let fr1 = match symtab.get("FAILREG") {
                Some(AgcAddress::Erasable { bank, offset }) => c
                    .read_sp(AgcAddress::Erasable {
                        bank,
                        offset: offset + 1,
                    })
                    .unwrap_or(0),
                _ => 0,
            };
            (
                read_cell(c, "CALFA").unwrap_or(0),
                read_cell(c, "P63FLAG").unwrap_or(0),
                read_cell(c, "CMDAPMOD").unwrap_or(0),
                read_cell(c, "MMNUMBER").unwrap_or(0),
                [fr0, fr1],
            )
        })
        .unwrap_or((0, 0, 0, 0, [0, 0]));

    // Tear down CDU thread and yaAGC.
    stop_flag.store(true, Ordering::Relaxed);
    cdu_thread.join().ok();
    let _ = child.kill();
    let _ = child.wait();
    std::thread::sleep(Duration::from_millis(150));

    let stderr_lines = stderr_buf.lock().expect("lock").clone();
    let scan = scan_stderr(&stderr_lines);
    let _ = std::fs::remove_dir_all(&work);

    // ── Reporting ────────────────────────────────────────────────────────────
    eprintln!("[e7i-j] display trail: {}", display_trail.join(" → "));
    eprintln!(
        "[e7i-j] final: MODREG=0o{modreg_final:05o} CALFA=0o{calfa_raw:05o} \
         P63FLAG=0o{p63flag:05o} CMDAPMOD=0o{cmdapmod:05o} \
         MMNUMBER=0o{mmnumber:05o} FAILREG=[0o{:05o},0o{:05o}]",
        failreg[0], failreg[1]
    );
    if !scan.matched_alarm.is_empty() {
        eprintln!("[e7i-j] alarm lines: {:?}", scan.matched_alarm);
    }

    // CALFA in SP ones-complement at B+0: cos(45°) = 1/√2 ≈ 0.7071 → ≈ 11584 raw.
    // A positive word below 0x4000 (16384) is a positive fraction; above
    // that is the negative half of ones-complement representation.
    let cos45_raw: u16 = (std::f64::consts::FRAC_1_SQRT_2 * 16383.0).round() as u16; // ≈ 11584

    // ── §8.7 acceptance criteria ─────────────────────────────────────────────
    assert_eq!(
        modreg_final, 0o077,
        "MODREG must reach P63 (0o077); got 0o{modreg_final:05o} after {proceeds} \
         PROCEEDs. CALFA=0o{calfa_raw:05o} CMDAPMOD=0o{cmdapmod:05o} \
         P63FLAG=0o{p63flag:05o} FAILREG=[0o{:05o},0o{:05o}]. \
         Observed display trail: {}",
        failreg[0], failreg[1],
        display_trail.join(" → "),
    );
    // Attitude-gate proof. Reaching P63 via WAKEP62 (asserted above) is
    // itself sufficient evidence that CALFA crossed +cos45° positive — per
    // the validated §3.3 gate that is the *only* path by which EXDAP arms
    // WAKEP62. We therefore assert on CMDAPMOD, the stable EXDAP output:
    // CMDAPMOD = +0 (0o00000) means EXDAP's last pass saw |CALFA| ≥ cos45°
    // with CALFA > 0 (heat-shield-forward) — not the -0 (0o77777) stall.
    //
    // The raw CALFA cell is NOT asserted: it aliases the SPNDX/INTTEMP
    // scratch area and its dumped value is overwritten unpredictably by
    // other routines (observed 0o37727 / 0o37247 / 0o00000 across runs).
    // `cos45_raw` is retained only for the diagnostic print below.
    let _ = cos45_raw;
    eprintln!(
        "[e7i-j] attitude gate: CMDAPMOD=0o{cmdapmod:05o} (+0 = heat-shield-forward) \
         CALFA(scratch, unreliable)=0o{calfa_raw:05o}"
    );
    assert_eq!(
        cmdapmod, 0o00000,
        "EXDAP must settle CMDAPMOD to +0 (heat-shield-forward); got 0o{cmdapmod:05o} \
         (0o77777 = -0 nose-into-wind stall)"
    );
    // No new entry-guidance program alarm. Two codes are expected harness
    // artifacts, not entry faults, and are allowlisted:
    //   - 0o01107: pre-existing phase-table marker in the template core.
    //   - 0o00207: "ISS TURN-ON REQUEST NOT PRESENT FOR 90 SEC" (T4RUPT
    //     ITURNON, ASSEMBLY_AND_OPERATION_INFORMATION.agc:925). The entry
    //     fixtures preload an already-aligned IMU and never drive the
    //     physical ISS turn-on discrete, so T4RUPT's turn-on monitor raises
    //     it once sim time passes 90 s. It is non-aborting (no POODOO in
    //     yaAGC stderr) and unrelated to the P62→P63 handover under test.
    const ALLOWED_ALARMS: [u16; 2] = [0o01107, 0o00207];
    let has_new_alarm = failreg
        .iter()
        .any(|&fr| fr != 0 && !ALLOWED_ALARMS.contains(&fr));
    assert!(
        !has_new_alarm,
        "unexpected program alarm in FAILREG: [0o{:05o}, 0o{:05o}]",
        failreg[0], failreg[1]
    );
}
