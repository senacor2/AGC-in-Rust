//! `capture_entry_template` — produce the binary template core dump
//! that the MS-E7d live yaAGC entry tests resume from.
//!
//! The template captures yaAGC's state after a brief cold boot, before
//! any DSKY interaction. The MS-E7d test then patches it with the
//! scenario-specific initial conditions via
//! [`agc_test::entry_state::patch_into`] and uses the result as the
//! `--no-resume` core-in.
//!
//! Pattern mirrors `capture_template_core` in
//! `agc-test/src/bin/capture_huntest.rs`.
//!
//! ## Usage
//!
//! ```sh
//! # One-time bootstrap (Phase 0).
//! bash agc-test/scripts/assemble_comanche055.sh
//!
//! # Capture / refresh the template core.
//! cargo run --features vagc-capture --bin capture_entry_template
//! ```
//!
//! Writes to `agc-test/fixtures/entry/entry_template.core` (resolved
//! relative to the agc-test crate root). The file is text-format
//! (yaAGC's `--dump-time` output) and round-trip stable through
//! `vagc_harness::CoreImage::load`.

use std::path::PathBuf;
use std::time::Duration;

use agc_test::vagc_harness::{vagc_root, CoreImage, RunMode, YaAgcRun};

fn main() {
    let root = vagc_root();
    let yaagc = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");
    let symtab = root.join("Comanche055/MAIN.agc.symtab");

    if !yaagc.exists() || !rope.exists() {
        die(&format!(
            "VirtualAGC build incomplete at {} — run agc-test/scripts/assemble_comanche055.sh",
            root.display()
        ));
    }

    let work = std::env::temp_dir().join(format!("capture_entry_template_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);

    let run = YaAgcRun {
        binary: yaagc.clone(),
        rope: rope.clone(),
        symtab: if symtab.exists() { Some(symtab) } else { None },
        core_in: None,
        work_dir: work.clone(),
        mode: RunMode::WallClockDump {
            dump_every_s: 1,
            wall_seconds: 3.0,
        },
        timeout: Duration::from_secs(15),
    };

    eprintln!("Cold-booting yaAGC for 3.0 s …");
    let result = run
        .execute()
        .unwrap_or_else(|e| die(&format!("yaAGC cold-boot failed: {e}")));

    // Sanity-check the dump before committing it: 512 channels and 8
    // erasable banks must be present, and at least one channel write
    // should have happened (lamp-blanking on boot).
    let core: CoreImage = result.core;
    if core.channels.len() != 512 || core.erasable.len() != 8 {
        die("captured core dump has wrong shape — yaAGC build mismatch?");
    }

    let out_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures")
        .join("entry")
        .join("entry_template.core");
    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent)
            .unwrap_or_else(|e| die(&format!("cannot create {}: {e}", parent.display())));
    }
    core.save(&out_path)
        .unwrap_or_else(|e| die(&format!("cannot save {}: {e}", out_path.display())));

    let _ = std::fs::remove_dir_all(work);
    eprintln!(
        "Wrote {} ({} channels, {} erasable banks).",
        out_path.display(),
        core.channels.len(),
        core.erasable.len()
    );
}

fn die(msg: &str) -> ! {
    eprintln!("error: {msg}");
    std::process::exit(1);
}
