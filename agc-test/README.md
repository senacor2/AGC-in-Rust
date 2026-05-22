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

(Coming with the per-routine capture binaries in #32 / #33 / #34
follow-ups — this README will be expanded then.)
