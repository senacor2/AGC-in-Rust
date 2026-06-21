# Specification: `services/downlink` Module — MSFN Telemetry Encoder

**Status**: Approved for implementation
**Module path**: `agc-core/src/services/downlink.rs`
**Architecture reference**: `docs/architecture.md` §7 (Services), §11 (Telemetry HAL)
**Related specs**:
- `specs/hal-spec.md` — `Telemetry` sub-trait (`send_word(u16)`).
- `specs/t4rupt-spec.md` — runs `downlink_step` six times per ~120 ms cycle.
- `specs/executive-spec.md` — bare-metal alternative driver (one `downlink_step` per DOWNRUPT).
- `specs/state-vector-spec.md`, `specs/conics-spec.md` — `StateVector` and apo/peri-apsis helpers used in the encoder.
- `specs/alarm-spec.md` — `state.alarm.code()` surfaces in pair 86 (IMODES30/33).
**Glossary cross-reference**: `docs/glossary.md` — DOWNRUPT, CMCSTADL, MSFN, DNTMBUFF, LOWIDCOD, B-scaling.
**AGC source files**:
- `Comanche055/DOWN-TELEMETRY_PROGRAM.agc` — `DODOWNTM`, `DNPHASE2`, the DOWNRUPT ISR.
- `Comanche055/DOWNLINK_LISTS.agc` — `CMCSTADL`, the 100-pair erasable-dump list this module reproduces.
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` line 1714 — `DNTMBUFF ERASE +11D` (the 12-word snapshot buffer).
**Reference**: O'Brien, *The Apollo Guidance Computer* §16.2 — buffer architecture.

---

## 1. Purpose and Scope

`services::downlink` is the **MSFN telemetry encoder** — it converts
the current `AgcState` into the 200-word, 2-second-long CMCSTADL
downlist that ground-station Manned Space Flight Network (MSFN)
expects.

The historical AGC emitted two 15-bit words on output channels 34 and
35 every 20 ms (50 Hz) under the DOWNRUPT ISR. One full
`CMCSTADL` cycle is 100 such word-pairs (= 200 words = 2 s).

This module is the encoder — it computes one word-pair at a time on
demand, with a 12-word snapshot buffer for the four sublists that
need atomic capture. A caller (the bare-metal Executive's DOWNRUPT
ISR or the simulator's T4 shim) drives one `downlink_step` per
DOWNRUPT.

### What this module provides

- AGC fixed-point encoders: `encode_agc15`, `encode_sp`, `encode_dp`,
  `encode_time`.
- Public constants `DOWNLIST_PAIRS = 100`, `DOWNLIST_WORDS = 200`,
  `LOWIDCOD = 0x7EE0`, `CMCSTADL_ID = 0x00FE`.
- `DownlinkDriver` — 28-byte per-cycle state (pair index + 12-word
  snapshot buffer).
- `downlink_step(driver, state, telemetry)` — send one DOWNRUPT
  word-pair.
- `build_cmcstadl(state) -> DownlistBuffer` — host-side test helper
  that drives 100 steps and returns the full 200-word flat array.

### What this module does NOT provide

- **Timing.** No clock, no scheduling. The caller decides when to
  invoke `downlink_step`.
- **The uplink path.** Uplink is `services::uplink`.
- **Output channel routing.** `Telemetry::send_word` abstracts the
  channel 34 / 35 split — the encoder sends words in (34, 35) order
  but the trait impl is free to fan them onto two physical lines.
- **Live data sources not yet tracked.** Many DSPTAB / MARKDOWN /
  CMPOWE07 fields are currently zero — they are placeholders to keep
  the cycle map intact; programs gain "fill this pair" responsibility
  as they grow.

---

## 2. AGC Background

### 2.1 Word format

The AGC uses one's-complement 15-bit words. Bit 14 = sign;
bits 13..0 = magnitude.

| Convention | Bits 14..0 |
|---|---|
| +0 | `0x0000` |
| Largest positive | `0x3FFF` (= +16383) |
| Largest negative | `0x4000` (= one's-complement −16383) |
| −0 | `0x7FFF` (rare but legal) |

A double-precision value uses a *pair* of 15-bit words: bit 14 of
both halves carries the sign, and the lower 14 bits of each half
concatenate to a 28-bit magnitude (max `0x0FFF_FFFF`).

### 2.2 B-scaling

Physical units are normalised to `[−1, +1]` by dividing by `2^B`
where `B` is the per-field exponent in the downlist tables. e.g.
`RN` is `B+29` so a 6.4×10⁸ m geocentric vector divided by `2^29 ≈
5.4×10⁸` lands in range with a few bits of headroom.

### 2.3 DNTMBUFF (snapshot buffer)

The AGC could not afford a 200-word RAM cache — it built each pair on
demand. Four of CMCSTADL's sublists are *snapshot sublists*:
they need an atomic read of several erasables (so the position
vector and the velocity vector in a single cycle come from the same
instant). For those, the first DOWNRUPT of the sublist captures the
data into a 12-word erasable area called `DNTMBUFF` and sends the
"live" pair simultaneously; the subsequent DOWNRUPTs of the sublist
drain `DNTMBUFF`.

12 words = 6 DP pairs is exactly the size of the largest snapshot
sublist (CMPOWE01 / CMPOWE05 — 7 entries × 2 = 14 words minus the live
pair = 12 buffered words).

### 2.4 CMCSTADL cycle map (100 pairs)

```
Pair 0      SENDID — ID + LOWIDCOD
Pairs 1–7   CMPOWE01 snapshot: RN+1/+2/+3/+4/+5, VN/+1/+2/+3/+4/+5, PIPTIME
Pairs 8–12  CMPOWE02 snapshot: CDU + ADOT family
Pairs 13–16 CMPOWE03 regular:  attitude error (THETADX/Y/Z) + RCSFLAGS
Pair  17    TIG/+1
Pair  18    BESTI/BESTJ
Pairs 19–22 MARKDOWN
Pairs 23–26 MARK2DWN
Pairs 27–28 HAPOX (apogee / perigee altitudes)
Pair  29    PACTOFF/YACTOFF
Pairs 30–32 VGTIG
Pairs 33–38 REFSMMAT first 6 DP elements
Pairs 39–49 CMPOWE04: FLAGWRDS 0–9 + DSPTAB
Pair  50    TIME2/TIME1
Pairs 51–57 CMPOWE05 snapshot
Pairs 58–62 CMPOWE02 repeat
Pairs 63–66 CMPOWE03 repeat
Pairs 67–72 CMPOWE06
Pairs 73–75 OGC/IGC/MGC
Pair  76    FLAGWRDS 10+11
Pairs 77–78 TEVENT, LAUNCHAZ
Pair  79    OPTMODES
Pairs 80–93 CMPOWE07
Pairs 94–99 DSPTAB
```

The module's source has the full per-pair table in its header
comment; this spec keeps the summary.

---

## 3. Rust API

### 3.1 Constants

```rust
pub const DOWNLIST_PAIRS: usize = 100;
pub const DOWNLIST_WORDS: usize = 200;
pub const LOWIDCOD:    u16 = 0x7EE0;   // octal 77340 — AGC sync word
pub const CMCSTADL_ID: u16 = 0x00FE;   // erasable-dump ID
const     DNTMBUFF_WORDS: usize = 12;  // private — snapshot-buffer size
```

`LOWIDCOD == 7×4096 + 7×512 + 3×64 + 4×8 = 32480` (TC-DL-13 asserts).

### 3.2 Encoders

```rust
pub fn encode_agc15(normalized: f64) -> u16;
pub fn encode_sp(value: f64, b_scale: i32) -> u16;
pub fn encode_dp(value: f64, b_scale: i32) -> (u16, u16);   // (high, low)
pub fn encode_time(time_cs: u32)            -> (u16, u16);  // (TIME2, TIME1)
```

- `encode_agc15` clamps to `[−1, 1]`, rounds, and outputs a 15-bit
  one's-complement word.
- `encode_sp` divides by `2^b_scale` and forwards to `encode_agc15`.
- `encode_dp` clamps to `[−1 + 2⁻²⁸, 1 − 2⁻²⁸]`, multiplies by
  `268_435_455` (= `2²⁸ − 1`), and splits into two 14-bit halves; the
  sign bit (bit 14) is set on both words for negative values.
- `encode_time` packs a centisecond MET counter into two 14-bit
  halves (lo first into TIME1, hi into TIME2); both bit-14 sign bits
  remain zero because the counter never goes negative.

### 3.3 `DownlinkDriver`

```rust
#[derive(Clone, Copy, Debug)]
pub struct DownlinkDriver {
    pub pair_index: usize,                     // 0..100, wraps
    snapshot_buf:   [u16; DNTMBUFF_WORDS],     // private
}

impl DownlinkDriver {
    pub const fn new() -> Self;
}
impl Default for DownlinkDriver { … }
```

Memory budget: `2*12 + 8 = 32` bytes on 64-bit hosts, 28 bytes on
32-bit (`thumbv7em-none-eabihf`). The AGC's full 200-word cache
equivalent would have been 408 bytes — see the docstring justification.

### 3.4 Public driver entry

```rust
pub fn downlink_step<T: Telemetry>(
    driver: &mut DownlinkDriver,
    state: &AgcState,
    telemetry: &mut T,
);
```

Sends two `Telemetry::send_word` calls (in pair-34, pair-35 order)
and advances `driver.pair_index` modulo 100. Pure: no logging, no
panics, no allocation. `no_std`-safe.

### 3.5 Test helper

```rust
pub type DownlistBuffer = [u16; DOWNLIST_WORDS];
pub fn build_cmcstadl(state: &AgcState) -> DownlistBuffer;
```

Drives a private `Telemetry` collector through 100 steps and returns
the flat 200-word array. Used by `agc-test` integration tests and the
`capture_downlink` fixture tool.

---

## 4. Functional Requirements

### 4.1 `downlink_step`

1. Let `k = driver.pair_index`.
2. Compute `(w34, w35) = compute_pair(driver, state, k)`.
3. `telemetry.send_word(w34); telemetry.send_word(w35);`
4. `driver.pair_index = (k + 1) % DOWNLIST_PAIRS`.

### 4.2 `compute_pair(driver, state, k)`

A 100-arm `match` on `k`. Four classes of arm:

- **SENDID (`k == 0`)** — returns `(CMCSTADL_ID, LOWIDCOD)`.
- **Snapshot-collect arms (`k ∈ {1, 8, 51, 58}`)** — fill
  `driver.snapshot_buf[0..12]` with the rest of the sublist and
  return the live pair.
- **Snapshot-drain arms (immediately after a collect)** — index into
  `driver.snapshot_buf` with `(k − collect_k − 1) * 2` and
  `… + 1`.
- **Live arms** — read directly from `state` and encode.

### 4.3 Snapshot helpers (private)

- `snapshot_cmpowe01` — RN+1..+5, VN/+1..+5, PIPTIME; live = RN/+1.
- `snapshot_cmpowe02` — CDUZ + CDUT (zero) + ADOT family (zero);
  live = CDUX,CDUY.
- `snapshot_cmpowe05` — R-OTHER family (zero); live = (0, 0).

CDU angles are converted from degrees to "revolutions / π" before
B-0 encoding (i.e. radians normalised by π so ±π → ±1).

### 4.4 Live data sources

| Pair | Source | Encoding |
|---|---|---|
| 13, 14, 15, 63, 64, 65, 82, 83 | `dap_state.attitude_error[]` | `encode_agc15(rad / π)` |
| 17 | `pending_maneuver.tig` | `encode_time` |
| 27, 28 | `apoapsis_altitude_earth` / `periapsis_altitude_earth` of `csm_state` (only if `epoch != 0 && frame == EarthInertial` and the orbit is not hyperbolic) | `encode_dp(alt, 29)` |
| 33..38 | `refsmmat` flattened (row-major, first 6 elements) | `encode_dp(v, 0)` |
| 39..43 | `flagwords[0..10]` paired | masked to 15 bits |
| 50 | `time` | `encode_time` |
| 76 | `flagwords[10..12]` | masked to 15 bits |
| 86 | `alarm.code()` | masked to 15 bits, low word zero |
| 94 | `dsky.prog`/`verb`/`noun` | `((prog<<7)|verb, noun)`, each 7-bit |
| 95 | `alarm.lit`, `dsky.opr_err`, `dsky.gimbal_lock`, `dsky.no_att` | bit-packed lamp word, low word zero |

All other pairs are `(0, 0)` — placeholders for fields not yet
tracked in this port (BESTI/J, MARKDOWN, MARK2DWN, VGTIG, DSPTAB
sectors, CMPOWE06/07 hardware register echo).

### 4.5 Cycle drift

`pair_index` is the sole cadence-tracking state. A burst of skipped
DOWNRUPTs simply stretches the 2-second cycle; an over-call
compresses it. The cycle resets every 100 calls. The encoder is
otherwise stateless — there is no "next time you wrap" bookkeeping.

---

## 5. Memory Budget

```
sizeof(DownlinkDriver)  ≤ 32 bytes  (12 × u16 buffer + 1 × usize index)
sizeof(DownlistBuffer)  = 400 bytes (200 × u16, test helper only)
```

No allocation, no globals, no panics, no `unsafe`. Pure
`no_std` + `libm` for `round` and `pow`.

---

## 6. Test Cases

| ID | Coverage |
|---|---|
| TC-DL-1 | `encode_agc15(0.0) == 0x0000` (+0 in one's complement). |
| TC-DL-2 | `encode_agc15(1.0) == 0x3FFF` (max positive). |
| TC-DL-3 | `encode_agc15(-1.0) == 0x4000` (max negative). |
| TC-DL-4 | `encode_agc15(0.5) == 8192`. |
| TC-DL-5 | `encode_sp(v, 0) == encode_agc15(v)` for representative values. |
| TC-DL-6 | `encode_dp(0.0, 28) == (0, 0)`. |
| TC-DL-7 | `encode_dp(1000.0, 28)` round-trips within 1 cs. |
| TC-DL-8 | `encode_time` separates lo/hi 14-bit halves correctly across the 16384-cs wrap. |
| TC-DL-9 | Pair 0 of `build_cmcstadl` is `(CMCSTADL_ID, LOWIDCOD)`. |
| TC-DL-10 | Pair 50 encodes `state.time` via `encode_time`. |
| TC-DL-11 | `build_cmcstadl` returns exactly `DOWNLIST_WORDS` words. |
| TC-DL-12 | `DownlinkDriver::pair_index` cycles 0..99 → 0. |
| TC-DL-13 | `LOWIDCOD == octal 77340`. |
| TC-DL-14 | The CMPOWE01 collect/drain pattern is consistent — pair 1 = live RN/+1, pairs 2..7 = `snapshot_buf[0..11]`. |

---

## 7. Module Layout

```
src/services/downlink.rs
├── pub const DOWNLIST_PAIRS, DOWNLIST_WORDS, LOWIDCOD, CMCSTADL_ID
├── const DNTMBUFF_WORDS
├── pub fn encode_agc15 / encode_sp / encode_dp / encode_time
├── pub struct DownlinkDriver { pair_index, snapshot_buf }
├── impl DownlinkDriver { new } / Default
├── fn snapshot_cmpowe01 / 02 / 05            (private)
├── pub fn downlink_step
├── fn compute_pair                            (private)
├── pub type DownlistBuffer
├── pub fn build_cmcstadl
└── #[cfg(test)] mod tests                     (TC-DL-1..14)
```

---

## 8. Spec Quality Checklist

- [x] One's-complement word format documented (§2.1).
- [x] B-scaling concept documented (§2.2).
- [x] DNTMBUFF rationale (12 words = max snapshot) recorded
      (§2.3, §3.3).
- [x] Cycle map at pair granularity (§2.4, §4.4).
- [x] Each encoder's clamp / round / sign handling specified
      (§3.2, §4.3).
- [x] All public constants given their AGC erasable / octal
      provenance.
- [x] Memory footprint vs full-cache alternative documented (§5).
- [x] Out-of-scope items (timing, channel routing, unwritten DSPTAB
      sectors) listed (§1).
- [x] Test coverage maps to every public function (§6).
