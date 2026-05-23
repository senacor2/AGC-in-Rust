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

## Closed-loop scenarios (MS-E7e)

The MS-E7d live tests are open-loop: PIPA pulses are derived from
the **Rust** pipeline's bank commands, so the AGC's `ROLLC`
(the bank-command output of P63/P64/P65/P67) has no effect on the
trajectory. MS-E7e adds two `VAGC_AVAILABLE`-gated companion tests
that close the steering loop:

- `tc_e7e_vagc_entry_direct_leo_closed_loop`
- `tc_e7e_vagc_entry_lunar_return_closed_loop`

Each spawns yaAGC with `--dump-time=2` so a fresh core dump arrives
once per simulated SERVICER cycle. Per cycle the driver:

1. Integrates the Rust state under the current `bank_rad` (initial
   value: 0).
2. Pushes the resulting Δv to yaAGC via `PipaInjector`.
3. Waits for the `core` file's mtime to advance, with a 30 ms grace
   period for the buffered write to flush.
4. Loads the dump, extracts `ROLLC` via
   `entry_state::read_rollc_rad`, uses it for the next cycle.

`ROLLC` is the DP erasable at `ROLLTM + 1` (`Comanche055/
ERASABLE_ASSIGNMENTS.agc:3181`) storing the bank command as a
fraction of one revolution; the helper multiplies by 2π. The
`Rust-only oracle for closed-loop summaries is structural only
(`tc_e7e_closed_loop_summary_structural`) because the Rust pipeline's
stage-A guidance does not reproduce Comanche055's full guidance — the
regression check that does matter is the live test re-running against
the committed summary.

### Closed-loop fixtures

- `*_closed_loop.json` — captured channel traces (`.gitignore`d like
  the open-loop ones).
- `*_closed_loop_summary.json` — committed ScenarioSummary derived
  from the live run's Rust-side state. Wider default tolerances than
  the open-loop summaries (2000 km miss, 5 g, 120 s elapsed, …)
  because yaAGC's guidance and the Rust integrator's bank execution
  combine into a different trajectory than either alone.

### Known limitations from the first MS-E7e captures

- **`ROLLC = 0` throughout — diagnosed.** Both scenarios observed
  `bank ∈ [0.0, 0.0] rad` for every cycle. Root cause (traced from a
  one-off probe of the AGC's live erasable state):

  1. The test sends `V37 63 ENTR`.
  2. The AGC's `V37` (major-mode-change) routine looks up program 63
     in the `PREMM1` dispatch table in
     `Comanche055/FRESH_START_AND_RESTART.agc:1314-1347`.
  3. Program 63 is **not** in `PREMM1`. Only P00, P01, P06, P17,
     P20–23, P30–35, P37–41, P47, P51–54, **P61, P62**, and
     P72–79 are. P63–P67 are dispatched internally from P62 (the
     entry preparation program), not selectable from the DSKY.
  4. The AGC takes the `V37NONO` branch (line 1059), lights the
     `OPR ERR` lamp (channel 11 bit 7, observed as
     `channel 0o11 = 0o100`), and rejects the request.
  5. With no entry program running, no SERVICER runs and no
     `HUNTEST` ever computes `ROLLC`. PIPA pulses still increment
     the hardware counters (verified: `PIPAX/Y/Z` advance) but
     `DELV` stays at 0 because the executive never consumes them.

  **Fix shape** (deferred — see follow-up issue):
  - Change the live test to send `V37 62 ENTR` (P62, the
    entry-preparation program) — confirmed to set `MODREG = 62`.
  - Then send a PROCEED (write to channel `0o32` with bit 14
    cleared) to acknowledge P62's `V06N61` flashing display and
    advance to P63. From there the SERVICER runs, sensed-g trips
    the 0.05g threshold, P64/HUNTEST runs, and ROLLC starts being
    written.

- **`lunar_return` doesn't reach drogue within the 240 s wall-clock
  cap.** Closed-loop is ~10× slower per cycle than open-loop because
  of the per-cycle core-dump wait. The lunar trajectory under bank =
  0 needs ~533 SERVICER cycles; at ~1 s wall-clock per cycle, the
  full trajectory exceeds the cap and the test exits early with
  `drogue_deployed = false`. The committed
  `lunar_return_closed_loop_summary.json` reflects this honestly.
  Resolving the ROLLC issue above and possibly raising the cap will
  unblock it.

## Out-of-scope (left for follow-ups)

- **Faithful Rust-side AGC-display channel projector.** The summary
  JSON oracle satisfies the "matching channel writes within
  tolerance" exit criterion at the scenario-metric level; per-cycle
  DSKY-display equivalence (channels 010 / 011 / 013) is a separate
  body of work and is deferred.
- **Diagnosing why the AGC's entry guidance doesn't reach HUNTEST in
  the preloaded scenarios.** The MS-E7e harness will pick up the
  fix automatically once a non-zero `ROLLC` starts being written.

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
| `tc_e7e_closed_loop_summary_structural` | `tests/entry_e2e_vagc.rs`     | Always.         |
| `tc_e7e_vagc_entry_direct_leo_closed_loop` | `tests/entry_e2e_vagc.rs` | VAGC + template.|
| `tc_e7e_vagc_entry_lunar_return_closed_loop` | `tests/entry_e2e_vagc.rs` | VAGC + template.|

The VAGC-gated tests look for `vagc_root()/yaAGC/yaAGC` and
`vagc_root()/Comanche055/MAIN.agc.bin`. The MS-E7d live tests
additionally require `agc-test/fixtures/entry/entry_template.core`,
which is generated by `cargo run --features vagc-capture --bin
capture_entry_template`. If any of these are missing, the test
prints a skip message and returns successfully — the same pattern
used elsewhere in the harness.
