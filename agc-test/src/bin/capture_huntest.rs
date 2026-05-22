//! `capture_huntest` — Phase-3 scaffold for HUNTEST fixture capture.
//!
//! Reads a TOML case-list, patches the input erasable variables into a
//! freshly-booted yaAGC core image, runs yaAGC for one SERVICER cycle,
//! reads back the output variables, and writes a `huntest_cases.json`
//! fixture file consumable by `agc-test/tests/entry_fixtures.rs`.
//!
//! ## Phase-3 scaffold caveat
//!
//! This binary builds and exercises the full pipeline (TOML → patch →
//! yaAGC → dump → JSON), but **does not yet drive the AGC to actually
//! execute HUNTEST**. Reaching HUNTEST in flight requires the DSKY +
//! PIPA scripting infrastructure that is the subject of issue #35
//! (MS-E7b). Without it, the AGC is in its prelaunch state when we
//! patch erasable, so the "outputs" we read back are mostly equal to
//! the inputs (no transformation happens). The committed fixture file
//! reflects this — it round-trips inputs through yaAGC's storage but
//! does not exercise HUNTEST math.
//!
//! When the MS-E7b DSKY-scripting harness lands, the loop body
//! `capture_one_case` swaps to the real "drive AGC through P63 →
//! threshold → P64 → HUNTEST → dump" sequence. The TOML/JSON formats
//! and the surrounding plumbing stay the same.
//!
//! ## Usage
//!
//! ```sh
//! # One-time bootstrap (Phase 0).
//! bash agc-test/scripts/assemble_comanche055.sh
//!
//! # Run the capture.
//! cargo run --features vagc-capture --bin capture_huntest -- \
//!     agc-test/fixtures/entry/huntest_inputs.toml \
//!     agc-test/fixtures/entry/huntest_cases.json
//! ```

use std::path::{Path, PathBuf};
use std::time::Duration;

use agc_test::vagc_harness::{
    read_scaled, vagc_root, write_scaled, CoreImage, RunMode, ScaledVar, Symtab, YaAgcRun,
};

use serde::{Deserialize, Serialize};

// ── TOML input format ───────────────────────────────────────────────────────

#[derive(Debug, Deserialize)]
struct CaptureConfig {
    /// Per-variable metadata: AGC symbol name, B-scale, SP/DP.
    variables: Vec<VarDef>,
    /// Input case list.
    cases: Vec<CaseInput>,
}

#[derive(Debug, Deserialize)]
struct VarDef {
    /// AGC symbol name (must exist in MAIN.agc.symtab as an erasable).
    name: String,
    /// B-scale exponent (e.g., +28 for DP position metres, +7 for DP
    /// velocity m/s, 0 for dimensionless ratios).
    scale: i8,
    /// `true` for double-precision (two-word) variables.
    dp: bool,
    /// `true` if this variable is an **input** (we write it before the
    /// run). `false` for an **output** (we read it after).
    input: bool,
}

#[derive(Debug, Deserialize)]
struct CaseInput {
    /// Human-readable identifier; used as the `name` in the JSON fixture.
    name: String,
    /// Long-form description, mirrored into the JSON.
    description: String,
    /// Input values keyed by variable name. Variables not listed retain
    /// their template-core-file value (typically zero on cold boot).
    inputs: std::collections::HashMap<String, f64>,
    /// Per-output tolerance (absolute). If a tolerance isn't given,
    /// defaults to 1e-3.
    #[serde(default)]
    tolerance: std::collections::HashMap<String, f64>,
}

// ── JSON output format ──────────────────────────────────────────────────────

#[derive(Debug, Serialize)]
struct CapturedFixture {
    /// Provenance: VirtualAGC commit, yaAGC version, capture date.
    source: String,
    /// Captured cases.
    cases: Vec<CapturedCase>,
}

#[derive(Debug, Serialize)]
struct CapturedCase {
    name: String,
    description: String,
    inputs: std::collections::BTreeMap<String, f64>,
    expected: std::collections::BTreeMap<String, f64>,
    tolerance: std::collections::BTreeMap<String, f64>,
}

// ── Entry point ─────────────────────────────────────────────────────────────

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: {} <inputs.toml> <out_cases.json>", args[0]);
        std::process::exit(2);
    }
    let input_path = PathBuf::from(&args[1]);
    let output_path = PathBuf::from(&args[2]);

    let config_text = std::fs::read_to_string(&input_path)
        .unwrap_or_else(|e| die(&format!("cannot read {}: {}", input_path.display(), e)));
    let config: CaptureConfig = toml::from_str(&config_text)
        .unwrap_or_else(|e| die(&format!("cannot parse TOML: {}", e)));

    let root = vagc_root();
    let yaagc = root.join("yaAGC/yaAGC");
    let rope = root.join("Comanche055/MAIN.agc.bin");
    let symtab_path = root.join("Comanche055/MAIN.agc.symtab");
    let listing = root.join("Comanche055/MAIN.agc.lst");

    if !yaagc.exists() || !rope.exists() || !listing.exists() {
        die(&format!(
            "VirtualAGC build incomplete at {} — run agc-test/scripts/assemble_comanche055.sh",
            root.display()
        ));
    }

    let symtab = Symtab::load(&listing)
        .unwrap_or_else(|e| die(&format!("cannot parse {}: {}", listing.display(), e)));
    eprintln!("Loaded symbol table: {} symbols.", symtab.len());

    // Resolve every variable's address before running any case. Catches
    // typos early, before we spend time invoking yaAGC.
    let resolved = resolve_variables(&config, &symtab);
    eprintln!("Resolved {} variables from TOML.", resolved.len());

    // Capture a baseline (template) core file via a cold yaAGC boot.
    // Per-case patching starts from this template.
    let template = capture_template_core(&yaagc, &rope, &symtab_path);
    eprintln!(
        "Captured template core: {} channels, {} erasable banks.",
        template.core.channels.len(),
        template.core.erasable.len()
    );

    let mut out = CapturedFixture {
        source: format!(
            "VirtualAGC + Comanche055, captured by capture_huntest on {}",
            chrono_today()
        ),
        cases: Vec::with_capacity(config.cases.len()),
    };

    for case in &config.cases {
        eprintln!("Capturing case: {}", case.name);
        let captured = capture_one_case(case, &resolved, &template.core, &yaagc, &rope, &symtab_path);
        out.cases.push(captured);
    }

    let json = serde_json::to_string_pretty(&out).unwrap();
    std::fs::write(&output_path, json + "\n")
        .unwrap_or_else(|e| die(&format!("cannot write {}: {}", output_path.display(), e)));
    eprintln!(
        "Wrote {} cases to {}.",
        out.cases.len(),
        output_path.display()
    );
}

// ── Helpers ─────────────────────────────────────────────────────────────────

struct ResolvedVar {
    def: VarDef,
    scaled: ScaledVar,
}

fn resolve_variables(config: &CaptureConfig, symtab: &Symtab) -> Vec<ResolvedVar> {
    let mut out = Vec::with_capacity(config.variables.len());
    for v in &config.variables {
        let addr = symtab.get(&v.name).unwrap_or_else(|| {
            die(&format!(
                "variable '{}' not found in Comanche055 symbol table",
                v.name
            ))
        });
        out.push(ResolvedVar {
            def: VarDef {
                name: v.name.clone(),
                scale: v.scale,
                dp: v.dp,
                input: v.input,
            },
            scaled: ScaledVar {
                addr,
                scale: v.scale,
                dp: v.dp,
            },
        });
    }
    out
}

struct TemplateCore {
    core: CoreImage,
}

fn capture_template_core(yaagc: &Path, rope: &Path, symtab: &Path) -> TemplateCore {
    let work = std::env::temp_dir().join(format!("capture_huntest_tmpl_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&work);
    let run = YaAgcRun {
        binary: yaagc.to_path_buf(),
        rope: rope.to_path_buf(),
        symtab: Some(symtab.to_path_buf()),
        core_in: None,
        work_dir: work.clone(),
        mode: RunMode::WallClockDump {
            dump_every_s: 1,
            wall_seconds: 1.0,
        },
        timeout: Duration::from_secs(10),
    };
    let result = run
        .execute()
        .unwrap_or_else(|e| die(&format!("template-core yaAGC run failed: {}", e)));
    let _ = std::fs::remove_dir_all(work);
    TemplateCore { core: result.core }
}

fn capture_one_case(
    case: &CaseInput,
    resolved: &[ResolvedVar],
    template: &CoreImage,
    yaagc: &Path,
    rope: &Path,
    symtab: &Path,
) -> CapturedCase {
    // Patch the template with this case's inputs.
    let mut patched = template.clone();
    for (input_name, value) in &case.inputs {
        let v = resolved
            .iter()
            .find(|r| &r.def.name == input_name)
            .unwrap_or_else(|| {
                die(&format!(
                    "case '{}' references unknown variable '{}'",
                    case.name, input_name
                ))
            });
        if !v.def.input {
            die(&format!(
                "variable '{}' is declared output-only but case '{}' tries to set it",
                input_name, case.name
            ));
        }
        if !write_scaled(&mut patched, &v.scaled, *value) {
            die(&format!(
                "cannot write '{}' = {} (address out of range?)",
                input_name, value
            ));
        }
    }

    // Save the patched template as a yaAGC core-resume file, then run
    // yaAGC briefly. **Phase-3 scaffold limitation**: the AGC will NOT
    // execute HUNTEST here — we're just round-tripping the patched
    // erasable through yaAGC's I/O. See module docs.
    let work = std::env::temp_dir().join(format!(
        "capture_huntest_case_{}_{}",
        sanitize(&case.name),
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&work);
    std::fs::create_dir_all(&work).unwrap();
    let patched_path = work.join("core_in");
    patched.save(&patched_path).unwrap();

    let run = YaAgcRun {
        binary: yaagc.to_path_buf(),
        rope: rope.to_path_buf(),
        symtab: Some(symtab.to_path_buf()),
        core_in: Some(patched_path),
        work_dir: work.clone(),
        mode: RunMode::WallClockDump {
            dump_every_s: 1,
            wall_seconds: 1.0,
        },
        timeout: Duration::from_secs(10),
    };
    let result = run
        .execute()
        .unwrap_or_else(|e| die(&format!("case '{}' yaAGC run failed: {}", case.name, e)));

    // Read back outputs from the post-run core.
    let mut expected = std::collections::BTreeMap::new();
    for v in resolved.iter().filter(|r| !r.def.input) {
        let val = read_scaled(&result.core, &v.scaled).unwrap_or(0.0);
        expected.insert(v.def.name.clone(), val);
    }

    // Resolve tolerances: per-output if specified, else a default.
    let mut tolerance = std::collections::BTreeMap::new();
    for name in expected.keys() {
        let t = case.tolerance.get(name).copied().unwrap_or(1e-3);
        tolerance.insert(name.clone(), t);
    }

    let _ = std::fs::remove_dir_all(work);

    CapturedCase {
        name: case.name.clone(),
        description: case.description.clone(),
        inputs: case.inputs.iter().map(|(k, v)| (k.clone(), *v)).collect(),
        expected,
        tolerance,
    }
}

fn sanitize(s: &str) -> String {
    s.chars()
        .map(|c| if c.is_alphanumeric() { c } else { '_' })
        .collect()
}

fn chrono_today() -> String {
    // Avoid adding a `chrono` dep for a one-line use. yaAGC builds
    // and the capture timestamp aren't used downstream — they're
    // bookkeeping. Just stamp wall-clock seconds since epoch.
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    format!("UNIX_EPOCH+{secs}s")
}

fn die(msg: &str) -> ! {
    eprintln!("error: {}", msg);
    std::process::exit(1);
}
