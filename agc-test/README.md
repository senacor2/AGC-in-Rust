# agc-test

Test utilities and integration tests for AGC-in-Rust.

## Crate layout

- `src/agc_convert.rs` — AGC fixed-point word ↔ `f64` conversion helpers.
- `src/fixtures.rs` — JSON fixture loaders for navigation accuracy tests.
- `src/entry_sim.rs` — 3DOF atmospheric-entry integrator used by the
  end-to-end MS-E7 scenario.
- `fixtures/*.json` — committed test vectors (Gravity, Kepler, Lambert,
  Servicer, Orbit propagation, Rendezvous, Targeting, Kalman, …).
- `tests/*.rs` — integration tests driving the full AGC stack against
  the simulator: P40 SPS burn, DSKY interaction, restart recovery,
  navigation accuracy, timing compliance, end-to-end entry.

## Optional: VirtualAGC routine-level fixture capture

For the entry-guidance B-track issues (#32 MS-E3b, #33 MS-E4b,
#34 MS-E6b) we drive yaAGC through individual entry routines (HUNTEST,
UPCONTRL, GLIM/PREDICT3), capture the AGC's outputs, and commit them as
JSON fixtures. **CI does not need yaAGC** — the validation tests run
against the committed JSON. Only the fixture *capture* step requires a
local VirtualAGC build.

### One-time setup

```sh
# Prerequisite: VirtualAGC clone at ~/virtualagc/ with yaAGC and yaYUL
# built natively. Following ~/dev/AGC-in-Rust/run-virtualagc.sh sets up
# the Podman path; building the native binaries on macOS is:
#
#   git clone https://github.com/virtualagc/virtualagc ~/virtualagc
#   cd ~/virtualagc && make

# Assemble the Comanche055 rope (one-shot, idempotent).
bash agc-test/scripts/assemble_comanche055.sh
```

This produces three files in `~/virtualagc/Comanche055/`:

- `MAIN.agc.bin` — 73 728-byte core rope, loadable by `yaAGC`.
- `MAIN.agc.symtab` — binary symbol table (11 MB; produced as a side
  effect of yaYUL — the harness parses `MAIN.agc.lst` instead).
- `MAIN.agc.lst` — assembly listing + plain-text symbol table.

The script is idempotent: subsequent invocations only re-assemble when
a `.agc` source has been modified since the last build.

### Capture flow (developer-only)

After the one-time bootstrap above, capture a fixture for a routine
by running its capture binary:

```sh
# HUNTEST (P64 — feeds #32 MS-E3b once DSKY scripting is in place):
cargo run -p agc-test --features vagc-capture --bin capture_huntest -- \
    agc-test/fixtures/entry/huntest_inputs.toml \
    agc-test/fixtures/entry/huntest_cases.json
```

The capture binary:

1. Parses the input TOML (which Comanche055 erasable variables are
   read / written, plus a list of named cases with their input values).
2. Resolves each variable's AGC address via the symbol table.
3. Spawns yaAGC briefly to obtain a baseline core image.
4. For each case: patches the inputs, runs yaAGC for one SERVICER
   cycle, reads back the outputs, writes them to JSON.

The committed JSON file (`fixtures/entry/huntest_cases.json`) is what
CI validates against in `agc-test/tests/entry_fixtures.rs`. CI does not
need yaAGC or the `vagc-capture` feature — it just reads the JSON.

### Phase-3 scaffold caveat

`capture_huntest` currently round-trips inputs through yaAGC's erasable
memory but does **not** drive the AGC to actually execute HUNTEST.
Reaching HUNTEST in flight requires DSKY + PIPA scripting that is
tracked in #35 (MS-E7b). Once that infrastructure lands, the
per-routine binaries flip to "drive AGC through P63 → 0.05g →
HUNTEST → COREDUMP" and the captured JSON values reflect the AGC's
actual computed outputs.

The capture scaffold (TOML format, JSON format, CI loader,
`vagc_harness` Rust library) is in place so that work becomes a small
change to one function in each capture binary, not a from-scratch
build.
