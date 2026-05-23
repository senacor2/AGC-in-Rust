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

## Scope and follow-on milestone

Out of scope for MS-E7c:

- **Driving yaAGC to drogue deploy on `entry_direct_leo` /
  `entry_lunar_return`.** This requires staging the AGC erasable state
  to the entry-interface configuration (REFSMMAT + ECI state vector +
  target landmark + entry flags) before the first SERVICER tick. The
  `YaAgcRun::core_in` core-resume plumbing exists in `vagc_harness.rs`,
  but no fixture exists that captures a "ready to enter P63" template
  core. Building that template core is the natural follow-on milestone
  — call it MS-E7d.

The current harness gives the next milestone everything it needs:

- a recorder that captures the live yaAGC trace
- a JSON format the trace can be committed in
- a comparator that asserts equivalence
- drivers (`DskyScript`, `PipaInjector`) that emit the stimuli the
  scenario needs once the AGC is in P63.

What's still required for MS-E7d is the **erasable-state preload**:
either a manual V21 / V25 / V52 sequence executed via `DskyScript`
before the integrator starts, or a captured core-resume file produced
by running yaAGC through that sequence once and snapshotting the
result.

## Test plan

| Test                              | Where                               | Gating          |
|-----------------------------------|-------------------------------------|-----------------|
| `tc_dsky_code_*`                  | `vagc_driver::tests`                | Always.         |
| `tc_pipa_enc_*`                   | `vagc_driver::tests`                | Always.         |
| `tc_trace_io_*`                   | `vagc_trace::tests`                 | Always.         |
| `tc_trace_cmp_*`                  | `vagc_trace::tests`                 | Always.         |
| `tc_e7c_fixture_load_smoke`       | `tests/entry_channel_trace.rs`      | Always.         |
| `tc_e7c_vagc_recorder_startup`    | `tests/entry_channel_trace.rs`      | VAGC available. |
| `tc_e7c_vagc_dsky_keypress`       | `tests/entry_channel_trace.rs`      | VAGC available. |

The VAGC-gated tests look for `vagc_root()/yaAGC/yaAGC` and
`vagc_root()/Comanche055/MAIN.agc.bin`. If either is missing, the test
prints a skip message and returns successfully — the same pattern
used elsewhere in the harness.
