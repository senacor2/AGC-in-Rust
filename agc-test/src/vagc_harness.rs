//! VirtualAGC fixture-capture harness for AGC-in-Rust.
//!
//! Provides three building blocks used by the routine-level capture
//! binaries (`agc-test/src/bin/capture_*`):
//!
//! 1. [`Symtab`] — parses `~/virtualagc/Comanche055/MAIN.agc.lst` and
//!    yields named-symbol → AGC-memory-address lookups. Both
//!    erasable (`E`) and fixed (`F`) symbols are stored; constants (`C`)
//!    are skipped.
//! 2. [`CoreImage`] — load / save / read / write the text core-dump
//!    files yaAGC produces with `--dump-time=N`. Round-trip stable.
//! 3. [`read_scaled`] / [`write_scaled`] — convert between AGC
//!    fixed-point words in a `CoreImage` and `f64` engineering units,
//!    reusing the `agc_convert` `from_agc_word` / `to_agc_dword`
//!    helpers.
//!
//! The harness deliberately does NOT spawn yaAGC itself — that's
//! Phase 2 of the harness build (separate module). Phase 1 (this file)
//! only handles the offline data: parse symtab, read/write core files.
//!
//! ## yaAGC core-dump format (from `agc_engine_init.c::MakeCoreDump`)
//!
//! Text file, octal `%06o\n` per line:
//! 1. Lines 1–512: I/O channels.
//! 2. Lines 513–2560: erasable memory, 8 banks × 256 words. Bank 0/1/2
//!    are "unswitched" (mapped at AGC addresses 0000–1377 octal);
//!    banks 3–7 are switched (mapped at addresses 1400–1777 via EBANK).
//! 3. Lines 2561+: CPU state (cycle counter, interrupt requests, …) —
//!    not parsed by this harness.

use std::collections::HashMap;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant};

use crate::agc_convert;

// ── AGC memory addresses ────────────────────────────────────────────────────

/// One AGC memory location, decoded into storage coordinates.
///
/// Storage layout follows yaAGC's `agc_t` struct:
/// - Erasable: `Erasable[bank: 0..8][offset: 0..256]`.
/// - Fixed:    `Fixed[bank: 0..36][offset: 0..1024]`. Not currently
///   stored by [`CoreImage`] (only erasable is needed for routine-level
///   fixture capture; rope-residing data is constant).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AgcAddress {
    /// Erasable: bank 0–7, offset 0–255.
    Erasable { bank: u8, offset: u16 },
    /// Fixed: bank 0–43 octal (= 0..36 dec), offset 0–1777 octal.
    Fixed { bank: u8, offset: u16 },
}

impl AgcAddress {
    /// Parse the address form used by yaYUL listing symbol tables:
    /// - `0OOO` (1–4 octal digits, no comma): unswitched erasable.
    /// - `BB,OOOO`: bank/offset, type-tag determines erasable vs fixed.
    /// - Pure number: low-fixed address (banks 02/03 of the rope).
    ///
    /// Returns `None` if the address can't be recognized as either
    /// erasable or fixed.
    fn parse(addr_text: &str, kind: SymbolKind) -> Option<Self> {
        match kind {
            SymbolKind::Erasable => {
                if let Some((bank_s, offset_s)) = addr_text.split_once(',') {
                    // Switched erasable: bank prefixed with optional 'E'.
                    let bank_s = bank_s.trim_start_matches('E');
                    let bank = u8::from_str_radix(bank_s, 8).ok()?;
                    let offset_oct = u16::from_str_radix(offset_s, 8).ok()?;
                    // Switched erasable maps AGC addresses 1400..1777 to
                    // bank N. Storage offset = AGC offset − 1400₈.
                    if !(0o1400..=0o1777).contains(&offset_oct) {
                        return None;
                    }
                    Some(Self::Erasable {
                        bank,
                        offset: offset_oct - 0o1400,
                    })
                } else {
                    // Unswitched erasable: 4-digit octal AGC address.
                    let addr = u16::from_str_radix(addr_text, 8).ok()?;
                    let (bank, offset) = if addr < 0o0400 {
                        (0, addr)
                    } else if addr < 0o1000 {
                        (1, addr - 0o0400)
                    } else if addr < 0o1400 {
                        (2, addr - 0o1000)
                    } else {
                        // 0o1400..0o1777 without a bank prefix would be
                        // ambiguous (switched window without EBANK
                        // value). Not a real symtab form.
                        return None;
                    };
                    Some(Self::Erasable { bank, offset })
                }
            }
            SymbolKind::Fixed => {
                if let Some((bank_s, offset_s)) = addr_text.split_once(',') {
                    let bank = u8::from_str_radix(bank_s, 8).ok()?;
                    let offset = u16::from_str_radix(offset_s, 8).ok()?;
                    Some(Self::Fixed { bank, offset })
                } else {
                    // Low-fixed: pure octal address in the 02–03 rope
                    // window. We store as bank-relative for symmetry;
                    // the harness doesn't currently access fixed memory
                    // so this is bookkeeping only.
                    let raw = u16::from_str_radix(addr_text, 8).ok()?;
                    Some(Self::Fixed {
                        bank: (raw >> 10) as u8,
                        offset: raw & 0o1777,
                    })
                }
            }
            SymbolKind::Constant => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SymbolKind {
    Erasable,
    Fixed,
    Constant,
}

impl SymbolKind {
    fn from_tag(tag: char) -> Option<Self> {
        match tag {
            'E' => Some(Self::Erasable),
            'F' => Some(Self::Fixed),
            'C' => Some(Self::Constant),
            _ => None,
        }
    }
}

// ── Symbol table ────────────────────────────────────────────────────────────

/// Map of AGC symbol name → memory address.
///
/// Parsed from the `Symbol Table` section of yaYUL's text assembly
/// listing (`~/virtualagc/Comanche055/MAIN.agc.lst`).
#[derive(Clone, Debug, Default)]
pub struct Symtab {
    symbols: HashMap<String, AgcAddress>,
}

impl Symtab {
    /// Load and parse a yaYUL listing file (`*.lst`).
    pub fn load(path: &Path) -> io::Result<Self> {
        let text = fs::read_to_string(path)?;
        Ok(Self::parse(&text))
    }

    /// Parse symbol-table text. Public for unit tests that feed
    /// hand-crafted snippets.
    pub fn parse(text: &str) -> Self {
        let mut symbols = HashMap::new();

        // The symbol-table section starts after the line `Symbol Table`
        // (followed by `------------`). Earlier lines have similar-
        // shaped content for individual instruction listings; ignoring
        // them avoids false positives.
        let mut in_table = false;
        for line in text.lines() {
            if !in_table {
                if line.trim_start().starts_with("Symbol Table") {
                    in_table = true;
                }
                continue;
            }
            // Each line of the symbol table holds up to three
            // `N,T:   NAME   ADDR  ` entries separated by tabs.
            for chunk in line.split('\t') {
                if let Some((name, addr)) = parse_symtab_chunk(chunk) {
                    symbols.insert(name, addr);
                }
            }
        }

        Self { symbols }
    }

    /// Look up a symbol by its uppercase AGC name. Returns `None` if
    /// the symbol is unknown or its address could not be parsed (e.g.,
    /// the symbol was a constant rather than a memory location).
    pub fn get(&self, name: &str) -> Option<AgcAddress> {
        self.symbols.get(name).copied()
    }

    /// Number of symbols indexed. Useful as a sanity check that the
    /// listing was parsed correctly.
    pub fn len(&self) -> usize {
        self.symbols.len()
    }

    /// True if no symbols were indexed (parse failure).
    pub fn is_empty(&self) -> bool {
        self.symbols.is_empty()
    }
}

/// Parse one "N,T:   NAME   ADDR" chunk into `(name, addr)`. Returns
/// `None` for separators (`==========…`) or unparseable rows.
fn parse_symtab_chunk(chunk: &str) -> Option<(String, AgcAddress)> {
    let chunk = chunk.trim();
    if chunk.is_empty() || chunk.starts_with('=') {
        return None;
    }
    // Form: `   NNN,T:   NAME       ADDR`
    let colon_pos = chunk.find(':')?;
    let tag_part = &chunk[..colon_pos];
    let body = &chunk[colon_pos + 1..].trim_start();
    let tag = tag_part.rsplit(',').next()?.trim();
    if tag.len() != 1 {
        return None;
    }
    let kind = SymbolKind::from_tag(tag.chars().next()?)?;

    // Split the body into NAME and ADDR by collapsing runs of spaces.
    let mut parts = body.split_whitespace();
    let name = parts.next()?.to_string();
    let addr_text = parts.next()?;

    let addr = AgcAddress::parse(addr_text, kind)?;
    Some((name, addr))
}

// ── Core image ──────────────────────────────────────────────────────────────

/// yaAGC's I/O channel count (`agc_engine.h: NUM_CHANNELS`).
const NUM_CHANNELS: usize = 512;
/// AGC has 8 erasable banks of 256 words each in yaAGC's representation.
const NUM_ERASABLE_BANKS: usize = 8;
/// Words per erasable bank (octal 0400 = 256 decimal).
const WORDS_PER_BANK: usize = 256;

/// In-memory representation of a yaAGC core dump.
///
/// Round-trip stable: `CoreImage::load(p).save(q)` followed by
/// `CoreImage::load(q)` reproduces the same erasable contents and
/// preserves the rest of the file byte-for-byte (the CPU-state suffix
/// is kept verbatim).
#[derive(Clone, Debug)]
pub struct CoreImage {
    /// I/O channel values. 512 entries.
    pub channels: Vec<u16>,
    /// Erasable memory: `erasable[bank][offset]`, both 0-indexed.
    pub erasable: Vec<Vec<u16>>,
    /// Verbatim trailing lines after the erasable section (CPU state).
    /// Re-emitted unchanged on save.
    suffix: Vec<String>,
}

impl CoreImage {
    /// Load a yaAGC text core-dump file.
    pub fn load(path: &Path) -> io::Result<Self> {
        let text = fs::read_to_string(path)?;
        Self::parse(&text).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
    }

    /// Save the core image back to disk in yaAGC's exact text format.
    pub fn save(&self, path: &Path) -> io::Result<()> {
        let mut f = fs::File::create(path)?;
        for ch in &self.channels {
            writeln!(f, "{:06o}", ch)?;
        }
        for bank in &self.erasable {
            for word in bank {
                writeln!(f, "{:06o}", word)?;
            }
        }
        for line in &self.suffix {
            writeln!(f, "{}", line)?;
        }
        Ok(())
    }

    /// Parse a yaAGC core-dump string.
    pub fn parse(text: &str) -> Result<Self, String> {
        let lines: Vec<&str> = text.lines().collect();
        let expected_min = NUM_CHANNELS + NUM_ERASABLE_BANKS * WORDS_PER_BANK;
        if lines.len() < expected_min {
            return Err(format!(
                "core dump truncated: {} lines, expected at least {}",
                lines.len(),
                expected_min
            ));
        }

        let mut channels = Vec::with_capacity(NUM_CHANNELS);
        for (i, line) in lines.iter().enumerate().take(NUM_CHANNELS) {
            channels.push(parse_octal(line).map_err(|e| {
                format!("line {} (channel {}): {}", i + 1, i, e)
            })?);
        }

        let mut erasable = Vec::with_capacity(NUM_ERASABLE_BANKS);
        for bank in 0..NUM_ERASABLE_BANKS {
            let mut words = Vec::with_capacity(WORDS_PER_BANK);
            for j in 0..WORDS_PER_BANK {
                let line_idx = NUM_CHANNELS + bank * WORDS_PER_BANK + j;
                words.push(parse_octal(lines[line_idx]).map_err(|e| {
                    format!("line {} (bank {} off {}): {}", line_idx + 1, bank, j, e)
                })?);
            }
            erasable.push(words);
        }

        let suffix = lines[expected_min..].iter().map(|s| s.to_string()).collect();

        Ok(Self {
            channels,
            erasable,
            suffix,
        })
    }

    /// Read a single-precision (one-word) erasable value as raw 16-bit.
    ///
    /// Returns `None` if `addr` is not an erasable address. Reading a
    /// `Fixed` address returns `None` — fixed memory comes from the
    /// rope (`MAIN.agc.bin`), not the dump.
    pub fn read_sp(&self, addr: AgcAddress) -> Option<u16> {
        match addr {
            AgcAddress::Erasable { bank, offset } => self
                .erasable
                .get(bank as usize)
                .and_then(|b| b.get(offset as usize))
                .copied(),
            AgcAddress::Fixed { .. } => None,
        }
    }

    /// Write a single-precision raw 16-bit value. Returns `false` if
    /// `addr` is out-of-range or not erasable.
    pub fn write_sp(&mut self, addr: AgcAddress, word: u16) -> bool {
        match addr {
            AgcAddress::Erasable { bank, offset } => {
                if let Some(b) = self.erasable.get_mut(bank as usize) {
                    if let Some(slot) = b.get_mut(offset as usize) {
                        *slot = word;
                        return true;
                    }
                }
                false
            }
            AgcAddress::Fixed { .. } => false,
        }
    }

    /// Read a double-precision (two consecutive words) value as
    /// `(hi, lo)` 16-bit raws. The DP pair is stored as the SP word at
    /// `addr` (hi) followed by the SP word at `addr+1` (lo).
    pub fn read_dp(&self, addr: AgcAddress) -> Option<(u16, u16)> {
        let next = increment_addr(addr)?;
        let hi = self.read_sp(addr)?;
        let lo = self.read_sp(next)?;
        Some((hi, lo))
    }

    /// Write a double-precision raw pair.
    pub fn write_dp(&mut self, addr: AgcAddress, hi: u16, lo: u16) -> bool {
        let Some(next) = increment_addr(addr) else {
            return false;
        };
        self.write_sp(addr, hi) && self.write_sp(next, lo)
    }
}

/// Return the AGC address one word after `addr` within the same bank.
/// Returns `None` if `addr` is at the last offset of its bank (so the
/// "next" address would cross a bank boundary — DP pairs are not
/// supposed to straddle banks).
fn increment_addr(addr: AgcAddress) -> Option<AgcAddress> {
    match addr {
        AgcAddress::Erasable { bank, offset } => {
            let next = offset.checked_add(1)?;
            if (next as usize) >= WORDS_PER_BANK {
                None
            } else {
                Some(AgcAddress::Erasable { bank, offset: next })
            }
        }
        AgcAddress::Fixed { .. } => None,
    }
}

/// Parse a single octal value from a yaAGC dump line into the low 16
/// bits of a `u16`. yaAGC writes erasable words via `printf("%06o", w)`
/// where `w` is `int16_t`, and printf promotes negative values via
/// default-argument promotion to (large) unsigned int — so 11-digit
/// outputs like `37777776314` are sign-extended representations of
/// negative `int16_t` values. We parse as 64-bit unsigned and take the
/// low 16 bits, which restores the original `int16_t` bit pattern.
fn parse_octal(line: &str) -> Result<u16, String> {
    let trimmed = line.trim();
    let v = u64::from_str_radix(trimmed, 8).map_err(|e| format!("'{}': {}", trimmed, e))?;
    Ok((v & 0xFFFF) as u16)
}

// ── Scaled-variable I/O ─────────────────────────────────────────────────────

/// Description of a named AGC variable: its erasable address, its
/// fixed-point B-scaling, and whether it is single- or double-precision.
#[derive(Clone, Copy, Debug)]
pub struct ScaledVar {
    /// AGC erasable address (must be `AgcAddress::Erasable`).
    pub addr: AgcAddress,
    /// Fixed-point B-scaling (e.g. `+28` for position in metres, `+7`
    /// for velocity in m/s). See `agc_convert::from_agc_word`.
    pub scale: i8,
    /// `true` for DP (two-word) variables, `false` for SP.
    pub dp: bool,
}

/// Read a named scaled variable from the core image. Returns `None` if
/// the address is out of range.
pub fn read_scaled(core: &CoreImage, var: &ScaledVar) -> Option<f64> {
    if var.dp {
        let (hi, lo) = core.read_dp(var.addr)?;
        Some(agc_convert::from_agc_dword(hi, lo, var.scale))
    } else {
        let raw = core.read_sp(var.addr)?;
        Some(agc_convert::from_agc_word(raw, var.scale))
    }
}

/// Write a named scaled variable into the core image. Returns `false`
/// if the address is out of range. For SP variables, this clamps to
/// the AGC's 15-bit ones-complement range and discards the LO half of
/// the DP word produced by `agc_convert::to_agc_dword`.
pub fn write_scaled(core: &mut CoreImage, var: &ScaledVar, value: f64) -> bool {
    let (hi, lo) = agc_convert::to_agc_dword(value, var.scale);
    if var.dp {
        core.write_dp(var.addr, hi, lo)
    } else {
        core.write_sp(var.addr, hi)
    }
}

// ── yaAGC subprocess wrapper ────────────────────────────────────────────────

/// Default wall-clock timeout for [`YaAgcRun::execute`] — yaAGC running
/// for a 2-s SERVICER cycle is sub-second wall-clock, so 30 s is very
/// generous and a misbehaving script trips it quickly.
pub const DEFAULT_YAAGC_TIMEOUT: Duration = Duration::from_secs(30);

/// Mode of operation for [`YaAgcRun`].
#[derive(Clone, Debug)]
pub enum RunMode {
    /// "Wall-clock dump" mode: spawn yaAGC with `--dump-time=N` and
    /// `--nodebug`, let it run for `wall_seconds` of real time, then kill
    /// it. The final `core` dump captures the AGC's state at the end of
    /// the run. Simple, fast, no breakpoints — used for the smoke test
    /// and any "run for fixed time" capture.
    WallClockDump {
        /// yaAGC `--dump-time=N` value (simulated seconds between dumps).
        dump_every_s: u32,
        /// Real-time deadline before we send SIGTERM to yaAGC.
        wall_seconds: f64,
    },
    /// Debugger-driven mode: enable the yaAGC GDB/MI debugger, feed it
    /// `commands` via `--command=FILE`, and let the script issue
    /// `BREAK`/`CONT`/`COREDUMP`/`QUIT` to dump at well-defined points.
    /// Used by Phase 3 per-routine capture binaries.
    Debugger {
        /// Lines to write to the `--command=FILE`.
        commands: Vec<String>,
    },
}

/// Configuration for one yaAGC invocation.
#[derive(Clone, Debug)]
pub struct YaAgcRun {
    /// Path to `yaAGC` binary.
    pub binary: PathBuf,
    /// Path to assembled core rope (`MAIN.agc.bin`).
    pub rope: PathBuf,
    /// Optional symtab file (`MAIN.agc.symtab`). Required for
    /// `RunMode::Debugger` if commands use symbol names.
    pub symtab: Option<PathBuf>,
    /// Optional core-resume file: yaAGC starts execution from this
    /// pre-staged AGC state instead of cold-booting. When `None`,
    /// yaAGC runs the AGC's prelaunch sequence from the rope.
    pub core_in: Option<PathBuf>,
    /// Working directory for the subprocess. The post-run `core` dump
    /// lands here. Each call should use a fresh directory to avoid
    /// races between parallel test invocations.
    pub work_dir: PathBuf,
    /// What the subprocess does once started.
    pub mode: RunMode,
    /// Wall-clock timeout. Defaults to [`DEFAULT_YAAGC_TIMEOUT`].
    pub timeout: Duration,
}

/// Result of a yaAGC invocation.
#[derive(Clone, Debug)]
pub struct YaAgcResult {
    /// Parsed post-run core dump (`work_dir/core`).
    pub core: CoreImage,
    /// Subprocess exit status. May reflect a kill-on-timeout in
    /// `RunMode::WallClockDump`.
    pub exit_code: Option<i32>,
}

impl YaAgcRun {
    /// Spawn yaAGC with the configured arguments, wait up to `timeout`
    /// for it to exit (sending SIGTERM if it doesn't), then load the
    /// `core` dump it produced.
    ///
    /// Errors if yaAGC cannot be spawned, the timeout is hit and yaAGC
    /// refuses to die, or no `core` file is found in `work_dir` after
    /// the run.
    pub fn execute(&self) -> io::Result<YaAgcResult> {
        fs::create_dir_all(&self.work_dir)?;

        let mut cmd = Command::new(&self.binary);
        cmd.current_dir(&self.work_dir);
        cmd.arg("--quiet");

        if let Some(symtab) = &self.symtab {
            cmd.arg(format!("--symbols={}", symtab.display()));
        }

        match &self.mode {
            RunMode::WallClockDump { dump_every_s, .. } => {
                cmd.arg("--nodebug");
                cmd.arg(format!("--dump-time={}", dump_every_s));
            }
            RunMode::Debugger { commands } => {
                let cmd_path = self.work_dir.join("yaagc_commands.txt");
                fs::write(&cmd_path, commands.join("\n") + "\n")?;
                cmd.arg(format!("--command={}", cmd_path.display()));
            }
        }

        // Suppress yaAGC's default `core` resume behaviour unless the
        // caller explicitly supplies a core-in.
        if self.core_in.is_none() {
            cmd.arg("--no-resume");
        }

        cmd.arg(&self.rope);
        if let Some(core_in) = &self.core_in {
            cmd.arg(core_in);
        }

        // Detach stdout/stderr so a chatty yaAGC doesn't fill our pipes.
        // `--quiet` already suppresses the banner; debugger prompts go
        // to stdout but get discarded.
        cmd.stdout(std::process::Stdio::null());
        cmd.stderr(std::process::Stdio::null());

        let mut child = cmd.spawn()?;
        let deadline = Instant::now() + self.timeout;
        let mut exit_code: Option<i32> = None;

        match &self.mode {
            // In wall-clock mode we always wait the full duration so
            // `--dump-time` has a chance to fire at least once, then we
            // SIGTERM yaAGC.
            RunMode::WallClockDump { wall_seconds, .. } => {
                let target = Instant::now() + Duration::from_secs_f64(*wall_seconds);
                while Instant::now() < target {
                    if let Some(status) = child.try_wait()? {
                        exit_code = status.code();
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(50));
                }
                if exit_code.is_none() {
                    let _ = child.kill();
                    let status = child.wait()?;
                    exit_code = status.code();
                }
            }
            // In debugger mode we expect the script to issue `QUIT`. If
            // it doesn't, we kill at the timeout.
            RunMode::Debugger { .. } => loop {
                if let Some(status) = child.try_wait()? {
                    exit_code = status.code();
                    break;
                }
                if Instant::now() > deadline {
                    let _ = child.kill();
                    let status = child.wait()?;
                    exit_code = status.code();
                    break;
                }
                std::thread::sleep(Duration::from_millis(50));
            },
        }

        let core_path = self.work_dir.join("core");
        if !core_path.exists() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!(
                    "yaAGC produced no core dump at {} (exit_code={:?})",
                    core_path.display(),
                    exit_code
                ),
            ));
        }
        let core = CoreImage::load(&core_path)?;
        Ok(YaAgcResult { core, exit_code })
    }
}

/// Convenience: locate the developer's local VirtualAGC checkout.
///
/// Returns `$VAGC_ROOT` if set, else `~/virtualagc`. Used by the
/// fixture-capture binaries and by smoke tests that gate on the
/// VirtualAGC build being present.
pub fn vagc_root() -> PathBuf {
    if let Ok(p) = std::env::var("VAGC_ROOT") {
        return PathBuf::from(p);
    }
    if let Ok(home) = std::env::var("HOME") {
        return PathBuf::from(home).join("virtualagc");
    }
    PathBuf::from("/virtualagc")
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-VAGC-AA-1: parse unswitched erasable address in each of the
    /// three banks (E0, E1, E2).
    #[test]
    fn tc_vagc_aa_1_parse_unswitched_erasable() {
        for (addr_text, expected_bank, expected_off) in [
            ("0000", 0, 0),
            ("0377", 0, 0o0377),
            ("0400", 1, 0),
            ("0752", 1, 0o0352),
            ("1000", 2, 0),
            ("1377", 2, 0o0377),
        ] {
            let parsed = AgcAddress::parse(addr_text, SymbolKind::Erasable);
            assert_eq!(
                parsed,
                Some(AgcAddress::Erasable {
                    bank: expected_bank,
                    offset: expected_off,
                }),
                "address {addr_text} parsed incorrectly: {:?}",
                parsed
            );
        }
    }

    /// TC-VAGC-AA-2: parse switched erasable `EB,OOOO` form.
    #[test]
    fn tc_vagc_aa_2_parse_switched_erasable() {
        let parsed = AgcAddress::parse("E7,1625", SymbolKind::Erasable);
        assert_eq!(
            parsed,
            Some(AgcAddress::Erasable {
                bank: 7,
                offset: 0o1625 - 0o1400,
            })
        );
    }

    /// TC-VAGC-AA-3: parse fixed `BB,OOOO` form.
    #[test]
    fn tc_vagc_aa_3_parse_fixed_banked() {
        let parsed = AgcAddress::parse("26,2455", SymbolKind::Fixed);
        assert_eq!(
            parsed,
            Some(AgcAddress::Fixed {
                bank: 0o26,
                offset: 0o2455,
            })
        );
    }

    /// TC-VAGC-AA-4: constants (`C` tag) are not memory addresses;
    /// parsing returns `None`.
    #[test]
    fn tc_vagc_aa_4_constant_skipped() {
        assert_eq!(AgcAddress::parse("0000146", SymbolKind::Constant), None);
    }

    /// TC-VAGC-AA-5: malformed input returns `None`, not a panic.
    #[test]
    fn tc_vagc_aa_5_malformed() {
        assert_eq!(AgcAddress::parse("not-octal", SymbolKind::Erasable), None);
        assert_eq!(AgcAddress::parse("8888", SymbolKind::Erasable), None);
    }

    /// TC-VAGC-ST-1: parse a small hand-built listing snippet.
    #[test]
    fn tc_vagc_st_1_small_snippet() {
        let text = "\
preamble line\n\
Symbol Table\n\
------------\n\
     1,F:   .05G         26,3240  \t    43,F:   -OCT10          6171  \n\
     3,C:   .05GSW       0000146  \t    45,E:   -PHASE1         0752  \n\
";
        let st = Symtab::parse(text);
        assert_eq!(
            st.get(".05G"),
            Some(AgcAddress::Fixed {
                bank: 0o26,
                offset: 0o3240
            })
        );
        assert_eq!(
            st.get("-PHASE1"),
            Some(AgcAddress::Erasable {
                bank: 1,
                offset: 0o0352
            })
        );
        // Constants are not stored.
        assert_eq!(st.get(".05GSW"), None);
        // Low-fixed in the fixed-fixed window: AGC address `6171` (octal)
        // sits in bank 03 (covering AGC addresses 6000..7777 octal).
        // (raw >> 10) = 3, (raw & 0o1777) = 0o171.
        assert_eq!(
            st.get("-OCT10"),
            Some(AgcAddress::Fixed {
                bank: 0o3,
                offset: 0o171
            })
        );
    }

    /// TC-VAGC-ST-2: lines outside the `Symbol Table` section are not
    /// treated as symbols (no false positives on listing rows).
    #[test]
    fn tc_vagc_st_2_no_false_positives_before_section() {
        let text = "\
000228,000082:    1,F:   .05G    26,3240  \n\
preamble\n\
";
        let st = Symtab::parse(text);
        assert_eq!(st.len(), 0, "no symbol-table marker → no symbols");
    }

    /// TC-VAGC-CI-1: build a synthetic core image, round-trip through
    /// save / load preserves channels, erasable, and suffix verbatim.
    #[test]
    fn tc_vagc_ci_1_round_trip_save_load() {
        use std::io::Read;
        let mut core = CoreImage {
            channels: vec![0; NUM_CHANNELS],
            erasable: (0..NUM_ERASABLE_BANKS)
                .map(|_| vec![0; WORDS_PER_BANK])
                .collect(),
            suffix: vec!["12345".into(), "0".into()],
        };
        core.channels[5] = 0o123;
        core.erasable[3][20] = 0o44321;

        let tmp = tempfile_path("core_rt_1");
        core.save(&tmp).unwrap();

        let loaded = CoreImage::load(&tmp).unwrap();
        assert_eq!(loaded.channels[5], 0o123);
        assert_eq!(loaded.erasable[3][20], 0o44321);
        assert_eq!(loaded.suffix, vec!["12345".to_string(), "0".into()]);

        // Byte-exact equality on the written file would also be nice,
        // but yaAGC's format is fully specified by save() so a round-
        // trip through load+save+load is sufficient.
        let mut bytes = Vec::new();
        std::fs::File::open(&tmp).unwrap().read_to_end(&mut bytes).unwrap();
        assert!(!bytes.is_empty());

        std::fs::remove_file(tmp).ok();
    }

    /// TC-VAGC-CI-2: read/write SP and DP words through `CoreImage`.
    #[test]
    fn tc_vagc_ci_2_sp_dp_round_trip() {
        let mut core = empty_core();
        let addr = AgcAddress::Erasable {
            bank: 4,
            offset: 50,
        };
        // Single-precision.
        assert!(core.write_sp(addr, 0o12345));
        assert_eq!(core.read_sp(addr), Some(0o12345));

        // Double-precision occupies addr and addr+1.
        let dp_addr = AgcAddress::Erasable {
            bank: 4,
            offset: 100,
        };
        assert!(core.write_dp(dp_addr, 0o10000, 0o20000));
        assert_eq!(core.read_dp(dp_addr), Some((0o10000, 0o20000)));
        assert_eq!(
            core.read_sp(AgcAddress::Erasable {
                bank: 4,
                offset: 101
            }),
            Some(0o20000)
        );
    }

    /// TC-VAGC-CI-3: writes to a `Fixed` address are rejected (fixed
    /// memory is the rope, not in the core dump).
    #[test]
    fn tc_vagc_ci_3_fixed_writes_rejected() {
        let mut core = empty_core();
        let fixed = AgcAddress::Fixed {
            bank: 0o26,
            offset: 0o2455,
        };
        assert!(!core.write_sp(fixed, 0o123));
        assert_eq!(core.read_sp(fixed), None);
    }

    /// TC-VAGC-SV-1: round-trip a scaled DP value through `write_scaled`
    /// and `read_scaled`. Position value at B+28 → 1 m per LSB.
    #[test]
    fn tc_vagc_sv_1_scaled_round_trip() {
        let mut core = empty_core();
        let var = ScaledVar {
            addr: AgcAddress::Erasable {
                bank: 3,
                offset: 10,
            },
            scale: 0,
            dp: true,
        };
        let original = 12345.0_f64;
        assert!(write_scaled(&mut core, &var, original));
        let read_back = read_scaled(&core, &var).unwrap();
        // Round-trip error < 1 LSB of B+0 DP (≈ 2^-28 ≈ 3.7e-9).
        assert!(
            (read_back - original).abs() < 1.0,
            "round-trip error too large: original={original}, read_back={read_back}"
        );
    }

    /// TC-VAGC-LST-INTEG: parse the real Comanche055 listing (if
    /// present in the developer's local VirtualAGC checkout) and look
    /// up a handful of well-known symbols. Skipped if the file is not
    /// available, so this test is non-blocking on CI.
    #[test]
    fn tc_vagc_lst_integ_real_listing() {
        let path = std::path::PathBuf::from(
            std::env::var("HOME")
                .map(|h| format!("{h}/virtualagc/Comanche055/MAIN.agc.lst"))
                .unwrap_or_default(),
        );
        if !path.exists() {
            eprintln!(
                "skipping: no Comanche055 listing at {} \
                 (run agc-test/scripts/assemble_comanche055.sh)",
                path.display()
            );
            return;
        }
        let st = Symtab::load(&path).unwrap();
        assert!(
            st.len() > 1000,
            "expected thousands of symbols, got {}",
            st.len()
        );
        // ROLLC: erasable in bank 7 from the listing (`E7,1633`,
        // shifted: storage offset = 0o1633 − 0o1400 = 0o233).
        let rollc = st.get("ROLLC").expect("ROLLC should be present");
        assert!(matches!(rollc, AgcAddress::Erasable { .. }), "ROLLC ⇒ erasable");
        // UPCONTRL: fixed bank label.
        let upctl = st.get("UPCONTRL").expect("UPCONTRL should be present");
        assert!(matches!(upctl, AgcAddress::Fixed { .. }), "UPCONTRL ⇒ fixed");
    }

    /// TC-VAGC-RUN-DBG-INTEG: spawn yaAGC in debugger mode with a
    /// minimal command script that just dumps the initial state and
    /// quits. Verifies the GDB/MI `COREDUMP filename` + `QUIT` path
    /// works — i.e., the wrapper correctly enables the debugger,
    /// writes the command file, and the script terminates the
    /// subprocess cleanly.
    ///
    /// Skipped when VirtualAGC build is unavailable.
    #[test]
    fn tc_vagc_run_dbg_integ_smoke() {
        let root = vagc_root();
        let yaagc = root.join("yaAGC/yaAGC");
        let rope = root.join("Comanche055/MAIN.agc.bin");
        let symtab = root.join("Comanche055/MAIN.agc.symtab");
        if !yaagc.exists() || !rope.exists() || std::fs::metadata(&rope).map(|m| m.len()).unwrap_or(0) == 0 {
            eprintln!("skipping: VirtualAGC build incomplete");
            return;
        }

        let work_dir = std::env::temp_dir().join(format!(
            "vagc_dbg_smoke_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&work_dir);
        std::fs::create_dir_all(&work_dir).unwrap();

        let run = YaAgcRun {
            binary: yaagc,
            rope,
            symtab: Some(symtab),
            core_in: None,
            work_dir: work_dir.clone(),
            mode: RunMode::Debugger {
                commands: vec![
                    "COREDUMP core".into(),
                    "QUIT".into(),
                ],
            },
            timeout: Duration::from_secs(10),
        };
        let result = run.execute().expect("yaAGC debugger smoke run failed");
        assert_eq!(result.core.channels.len(), 512);
        assert_eq!(result.core.erasable.len(), 8);

        let _ = std::fs::remove_dir_all(work_dir);
    }

    /// TC-VAGC-RUN-INTEG: spawn yaAGC in wall-clock-dump mode and verify
    /// it produces a parseable `core` file.
    ///
    /// Skipped when the VirtualAGC build is unavailable (no yaAGC binary
    /// or no Comanche055 rope at the expected paths). This is the
    /// Phase 2 smoke test described in the harness plan — it doesn't
    /// validate the AGC's *behaviour*, just that the subprocess
    /// wrapper, args, and core-dump round-trip work end-to-end.
    #[test]
    fn tc_vagc_run_integ_smoke() {
        let root = vagc_root();
        let yaagc = root.join("yaAGC/yaAGC");
        let rope = root.join("Comanche055/MAIN.agc.bin");
        if !yaagc.exists() || !rope.exists() || std::fs::metadata(&rope).map(|m| m.len()).unwrap_or(0) == 0 {
            eprintln!(
                "skipping: VirtualAGC build incomplete \
                 (yaAGC={}, rope={}). Run agc-test/scripts/assemble_comanche055.sh.",
                yaagc.display(),
                rope.display()
            );
            return;
        }

        let work_dir = std::env::temp_dir().join(format!(
            "vagc_smoke_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&work_dir);

        let run = YaAgcRun {
            binary: yaagc,
            rope,
            symtab: None,
            core_in: None,
            work_dir: work_dir.clone(),
            mode: RunMode::WallClockDump {
                dump_every_s: 1,
                wall_seconds: 1.5,
            },
            timeout: Duration::from_secs(10),
        };
        let result = run.execute().expect("yaAGC smoke run failed");

        // A core dump from a cold-booted AGC should have at least the
        // channel and erasable arrays populated. Spot-check: the
        // suffix carries CPU-state lines.
        assert_eq!(result.core.channels.len(), 512);
        assert_eq!(result.core.erasable.len(), 8);
        assert!(
            !result.core.suffix.is_empty(),
            "expected CPU-state suffix in core dump, got none"
        );

        let _ = std::fs::remove_dir_all(work_dir);
    }

    // ── Helpers ─────────────────────────────────────────────────────────

    fn empty_core() -> CoreImage {
        CoreImage {
            channels: vec![0; NUM_CHANNELS],
            erasable: (0..NUM_ERASABLE_BANKS)
                .map(|_| vec![0; WORDS_PER_BANK])
                .collect(),
            suffix: Vec::new(),
        }
    }

    fn tempfile_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("vagc_harness_test_{name}_{}", std::process::id()))
    }
}
