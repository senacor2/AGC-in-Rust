# Entry-Guidance Channel-Trace Harness (MS-E7c)

Infrastructure for capturing AGC → peripheral channel writes from a
running yaAGC instance, comparing them against a Rust-side trace, and
asserting cycle-by-cycle equivalence. The harness builds on the
packet-level [`vagc_channel`](../agc-test/src/vagc_channel.rs) client
that landed in MS-E7b.

## Modules

| Module | Role |
|---|---|
| `agc-test/src/vagc_channel.rs` | TCP packet client (`YaAgcClient`, `ChannelPacket`). |
| `agc-test/src/vagc_driver.rs` | High-level drivers: `DskyScript`, `PipaInjector`. |
| `agc-test/src/vagc_trace.rs`  | Recorder, JSON fixture format, comparator. |

## Wire-protocol building blocks

### DSKY keypresses

A keypress is a single `ChannelPacket` on AGC channel `0o15` carrying
the 5-bit `LOW5`-masked code that `KEYRUPT1` reads via `RAND MNKEYIN`.
The mapping is recovered from `CHARIN2`'s dispatch table in
`Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc` (pp. 315–316):

| Code (octal) | Key                    |
|--------------|------------------------|
| 01–07        | digits 1–7             |
| 010, 011     | digits 8, 9            |
| 020          | digit 0                |
| 021          | VERB                   |
| 022          | ERROR RESET (RSET)     |
| 031          | KEY RELEASE            |
| 032          | +                      |
| 033          | −                      |
| 034          | ENTR / PRO             |
| 036          | CLEAR                  |
| 037          | NOUN                   |

`DskyScript::verb_noun(35, 0)` therefore emits the six packets
`VERB, 3, 5, NOUN, 0, ENTR` on channel `0o15`.

### PIPA pulses

yaAGC's `SocketAPI.c::ParseIoPacket` treats packets with the high bit
of the channel byte set (`Channel & 0x80`) as **counter-increment**
commands. The low 7 bits select the erasable counter address; the
15-bit value carries the `IncType` discriminant (`0 = PINC`, `2 =
MINC`). For the three PIPA accumulators:

| Counter | Erasable (octal) | Wire `channel` byte |
|---------|------------------|---------------------|
| PIPAX   | 037              | `0o237 = 0xBF`      |
| PIPAY   | 040              | `0o240 = 0xC0`      |
| PIPAZ   | 041              | `0o241 = 0xC1`      |

`PipaInjector::tick` integrates one SERVICER cycle through the
`EntryIntegrator`, quantises the resulting Δv with the existing
`pipa_pulses_for_dv` helper, and emits `|count|` PINC or MINC packets
per axis. Pulses are sent in a single per-cycle burst at the start of
the cycle; yaAGC accumulates them into the counter registers
immediately, and the AGC's foreground SERVICER reads them at the
next 2-s tick.

## Trace JSON format

```jsonc
{
  "scenario": "entry_direct_leo",
  "provenance": "yaAGC <hash>, Comanche055 <hash>, captured 2026-05-23",
  "events": [
    { "t_ms": 0, "channel": 10, "value": 0 },
    ...
  ]
}
```

Timestamps are wall-clock milliseconds from
`ChannelTraceRecorder::new`. The capture order is the time order:
`t_ms` is monotonically non-decreasing. The comparator does not
require strict-greater because yaAGC may emit several packets at the
same millisecond.

## Comparator tolerances

[`CompareTolerance`](../agc-test/src/vagc_trace.rs) classifies channels
into three buckets:

- **Event-exact** (`0o05`, `0o30`–`0o33`): the multiset of `(value)`
  writes must match exactly. Discrete output channels (RCS jets, lamps,
  alarm bits) are crisp digital signals — every write is meaningful.
- **Final-value** (`0o10`–`0o13`): only the most recent value matters.
  The DSKY display channels are rewritten on every T4RUPT (every
  120 ms) as a snapshot of the current display state; comparing every
  intermediate value would be sensitive to wall-clock jitter that has
  no AGC meaning.
- **Ignored**: defaults to empty. Populate per scenario to suppress
  channels with known nondeterminism (e.g., the AGC's
  `--debug-dsky`-only fictitious channels).

Any channel that appears in the trace but is not configured in any of
the three buckets is flagged as `UnconfiguredChannel`. This is a
forcing function — every channel observed must have an explicit
tolerance decision before the test merges.

## Erasable-state preload (MS-E7d)

`agc-test/src/entry_state.rs` encodes a high-level
[`EntryInitialState`] (SI units, ECI frame) into a patched
[`CoreImage`] by writing every Comanche055 erasable variable that
P63 reads: `RN`, `VN`, `TET`, `REFSMMAT`, `LAT(SPL)` / `LNG(SPL)`,
`EMSALT`, `ALFAPAD`, `HEADSUP`, `MODREG`, and the `REFSMFLG` bit
inside `FLAGWRD3`. Each variable's symbol address comes from the
yaYUL listing's symbol table; the B-scaling is documented per-
variable against `Comanche055/ERASABLE_ASSIGNMENTS.agc` and
`agc_convert.rs`.

The patch path uses the existing `YaAgcRun::core_in` plumbing in
`vagc_harness.rs`: cold-boot yaAGC once to capture
`entry_template.core`, then per-test patch that template, save as
`core_in`, and resume yaAGC from it via `--no-resume <core_in>`.
The Rust-side helper is round-trip-tested in
`agc-test/src/entry_state.rs::tests`; live AGC-acceptance
validation happens when `tests/entry_e2e_vagc.rs` runs against a
machine with VirtualAGC built.

### Regenerating the binary template core

```sh
# One-time bootstrap (assembles MAIN.agc.bin etc).
bash agc-test/scripts/assemble_comanche055.sh

# Cold-boot yaAGC and capture entry_template.core.
cargo run --features vagc-capture --bin capture_entry_template
```

The captured file lives at
`agc-test/fixtures/entry/entry_template.core`. It is **not
committed** — too large, fully deterministic given the rope, and
only useful on a machine with the VirtualAGC build present. The
live tests skip cleanly when it is absent.

## Scenario summaries

For each named scenario, a small `*_summary.json` fixture captures
the scenario-level metrics that the Rust pipeline produces — drogue
deployed yes/no, miss distance, peak sensed-g, elapsed seconds,
total SERVICER cycles, final landed lat/lon, minimum altitude — plus
per-metric tolerances. The Rust-only test
`tc_e7d_summary_rust_pipeline` loads each summary and asserts the
shared `simulate_to_drogue` runner matches it within tolerance.
This is the regression oracle: an MS-E*b refinement that shifts the
Rust pipeline's metrics will fail the test until the summary is
regenerated.

```sh
# Regenerate summary fixtures after an MS-E*b refinement.
cargo test -p agc-test --test entry_e2e_vagc regenerate_summary_fixtures \
    -- --ignored --nocapture

# Refresh BOTH the channel trace and the summary from a live yaAGC
# run (requires VirtualAGC built locally + entry_template.core
# captured):
VAGC_CAPTURE=1 cargo test -p agc-test --test entry_e2e_vagc
```

## Out-of-scope for MS-E7d

- **Closed-loop bidirectional drive.** The current live test
  injects PIPA pulses derived from the **Rust** pipeline's
  trajectory, not from yaAGC's bank commands. The AGC's view is
  open-loop with respect to its own steering. A true closed loop —
  read yaAGC's bank-command channel each cycle, feed it back into
  the integrator — is MS-E7e.
- **Faithful Rust-side AGC-display channel projector.** The summary
  JSON oracle satisfies the "matching channel writes within
  tolerance" exit criterion at the scenario-metric level; per-cycle
  DSKY-display equivalence is also MS-E7e territory.

## Test plan

| Test                              | Where                               | Gating          |
|-----------------------------------|-------------------------------------|-----------------|
| `tc_dsky_code_*`                  | `vagc_driver::tests`                | Always.         |
| `tc_pipa_enc_*`                   | `vagc_driver::tests`                | Always.         |
| `tc_trace_io_*`                   | `vagc_trace::tests`                 | Always.         |
| `tc_trace_cmp_*`                  | `vagc_trace::tests`                 | Always.         |
| `tc_es_*`                         | `entry_state::tests`                | Always.         |
| `tc_e7c_fixture_load_smoke`       | `tests/entry_channel_trace.rs`      | Always.         |
| `tc_e7c_vagc_recorder_startup`    | `tests/entry_channel_trace.rs`      | VAGC available. |
| `tc_e7c_vagc_dsky_keypress`       | `tests/entry_channel_trace.rs`      | VAGC available. |
| `tc_e7d_summary_rust_pipeline`    | `tests/entry_e2e_vagc.rs`           | Always.         |
| `tc_e7d_vagc_entry_direct_leo`    | `tests/entry_e2e_vagc.rs`           | VAGC + template.|
| `tc_e7d_vagc_entry_lunar_return`  | `tests/entry_e2e_vagc.rs`           | VAGC + template.|

The VAGC-gated tests look for `vagc_root()/yaAGC/yaAGC` and
`vagc_root()/Comanche055/MAIN.agc.bin`. The MS-E7d live tests
additionally require `agc-test/fixtures/entry/entry_template.core`,
which is generated by `cargo run --features vagc-capture --bin
capture_entry_template`. If any of these are missing, the test
prints a skip message and returns successfully — the same pattern
used elsewhere in the harness.
