# Specification: `services/uplink` Module — UPRUPT to V/N Processor Shim

**Status**: Approved for implementation
**Module path**: `agc-core/src/services/uplink.rs`
**Architecture reference**: `docs/architecture.md` §7 (Services), §11 (DSKY)
**Related specs**:
- `specs/hal-spec.md` — `Uplink` sub-trait (`read_word(&mut self) -> Option<u16>`).
- `specs/v_n-spec.md` — `Key`, `Key::from_code`, `VnPhase`, `feed_key`. The V/N processor consumes every uplink key as if it had been pressed on the DSKY.
- `specs/t4rupt-spec.md` — calls `poll_uplink` once per ~120 ms tick.
- `specs/alarm-spec.md` — `state.alarm.raise(UPLINK_TOO_FAST, SITE_UPLINK)` on OprErr overrun.
**Implementation plan (historical)**: `specs/uplink-plan.md` — milestone breakdown that drove the implementation. MS-U1 through MS-U5 are landed; this spec supersedes it for the documented behaviour.
**Glossary cross-reference**: `docs/glossary.md` — UPRUPT, UPLINK ACTY, V70/V71/V72/V73, KEYTEMP1.
**AGC source files**:
- `Comanche055/KEYRUPT,_UPRUPT.agc` — the UPRUPT ISR. Validates the redundancy-encoded uplink word, extracts the 5-bit key code, and routes it into the same NSTRT path that KEYRUPT uses for the DSKY.
- `Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` — KEYTEMP1, the 5-bit key code table.

---

## 1. Purpose and Scope

`services::uplink` is the **bridge from the HAL `Uplink` trait to the
V/N processor**. It exposes one public function — `poll_uplink` — that
drains every queued uplink word, decodes each into a `Key`, and feeds
the key into the same `feed_key` entry point that the DSKY keypad
uses. The result is that any ground-uplink sequence (V70, V71, V72,
V73, RSET, raw nouns, …) is observationally identical to the same
crew-typed sequence.

This module also runs three pieces of bookkeeping that the historical
AGC's UPRUPT ISR ran:

- It maintains the **UPLINK ACTY** lamp (`dsky.uplink_activity`) — on
  during a tick that drained at least one word, off on a quiet tick.
- It enforces the **UPLINK TOO FAST** rule — a non-RSET key arriving
  while V/N is locked in `OprErr` raises alarm `0o1106`
  (`UPLINK_TOO_FAST`) and drops the key.
- It validates the **5-bit key code**: bits above 5 are masked off,
  the zero word is treated as "idle line", and any code not in
  KEYTEMP1 is silently dropped (matching the AGC, where bad codes
  never reach NSTRT).

The Apollo redundancy / complement / "uplink too fast" wire-level
protocol is **not** in this module. It is the responsibility of the
bare-metal driver (`agc-board-nucleo-f767`) and of the simulator's
HAL impl. `Uplink::read_word()` is contracted to return
post-validated key codes; `services::uplink` trusts the lower 5 bits.

### What this module provides

- `pub fn key_from_word(word: u16) -> Option<Key>` — pure decoder.
- `pub fn poll_uplink<U: Uplink>(state: &mut AgcState, uplink: &mut U)`.

### What this module does NOT provide

- **The redundancy / complement protocol.** The wire format that the
  ground actually sends — three-of-five voting, complement check — is
  the HAL impl's job (the trait returns a single, validated 5-bit
  code per word).
- **V70/V71/V72/V73 dispatch.** Those verbs are implemented in
  `services::v_n` (`dispatch_verb_noun`, `v71_p27_block_update`,
  `p27_apply_word`, …). This module only hands keys over.
- **Buffering.** The trait owns the FIFO. `poll_uplink` calls
  `read_word()` repeatedly until it returns `None`.

---

## 2. AGC Background

In Comanche055 the UPRUPT interrupt fires when the ground has
transmitted a full uplink word into the AGC's input register. The ISR
in `KEYRUPT,_UPRUPT.agc` validates the redundancy encoding, extracts
the underlying 5-bit DSKY key code, and routes it through the same
`INREAD` / `NSTRT` path that KEYRUPT uses for a crew keypress. The V/N
processor on the other side cannot tell the difference between a
crew-typed key and an uplinked one — that is the whole point.

This Rust port keeps that property. The HAL `Uplink::read_word`
returns a `u16` whose lower 5 bits are a `KEYTEMP1` code; everything
else is reserved. `poll_uplink` is the equivalent of the AGC's "drain
the FIFO" loop and feeds `feed_key` directly.

---

## 3. Rust API

### 3.1 `key_from_word`

```rust
pub fn key_from_word(word: u16) -> Option<Key>;
```

Mask the low 5 bits. Return `None` for the all-zero word (idle line)
and for any code not in KEYTEMP1. Otherwise return
`Key::from_code(code as u8)`.

Pure — no state, no panics.

### 3.2 `poll_uplink`

```rust
pub fn poll_uplink<U: Uplink>(state: &mut AgcState, uplink: &mut U);
```

Drains `uplink.read_word()` to exhaustion. For each word:

1. Set `drained = true` (used at end for the lamp).
2. Decode via `key_from_word`; on `None`, continue.
3. If `state.vn.phase == VnPhase::OprErr` and `key != Key::Rset`,
   raise `UPLINK_TOO_FAST` with `SITE_UPLINK` and drop the key.
4. Otherwise call `feed_key(state, key)`.

After the loop:

5. `state.dsky.uplink_activity = drained`.

`no_std`-safe; the only dependency outside this module is `feed_key`
and the alarm constants.

---

## 4. Functional Requirements

### 4.1 Decoder rules (`key_from_word`)

| Input | Output |
|---|---|
| `word == 0` (any high bits) | `None` (idle line). |
| `word & 0x1F == 0` (high bits set, low 5 = 0) | `None`. |
| `word & 0x1F == code in KEYTEMP1` | `Some(Key::from_code(code))`. |
| `word & 0x1F == code not in KEYTEMP1` | `None` (dropped silently). |

The "drop silently" behaviour mirrors the AGC: bad codes never reach
NSTRT. This is the right default — the ground's transmission frame
is opaque past the validated key code, and noise is more common than
deliberately invalid codes.

### 4.2 Drain rules (`poll_uplink`)

- Drains `read_word` until it returns `None` — no per-call cap. A
  burst of queued uplink words is fully drained in one tick.
- The lamp (`dsky.uplink_activity`) is set iff at least one word was
  drained, and cleared on the first quiet tick. The historical AGC's
  UPLINK ACTY lamp had a similar duty-cycle behaviour — at the 120 ms
  T4 cadence it blinks at roughly the same rate as ground traffic.
- An OprErr overrun (non-RSET key while `phase == OprErr`) raises
  `UPLINK_TOO_FAST` *and* drops the offending key. RSET is the
  documented recovery path and is let through.
- Unknown codes are dropped at the decoder level and never reach the
  OprErr check (they would still consume the read; the lamp will
  reflect activity).

### 4.3 What `feed_key` does next

`feed_key` is the V/N processor's normal entry point. It implements
the full state machine documented in `specs/v_n-spec.md` — digit
accumulation, V/N pairing, V37 major-mode entry, V71/72/73 data load,
RSET, KEY REL, etc. Nothing in this module duplicates that logic; an
uplink RSET goes through the same RSET handler as a panel RSET, an
uplink "V 7 1 ENTR" goes through `dispatch_verb_noun(71, …)` exactly
like a panel sequence.

That property is what TC-UPL-4 asserts.

---

## 5. Test Cases

| ID | Coverage |
|---|---|
| TC-UPL-1 | `key_from_word` decodes representative valid codes: VERB (17), ENTR (28), 0/1/9. |
| TC-UPL-2 | Upper bits are ignored — `0xFFE1 → Digit(1)`, `0x8011 → VERB`. |
| TC-UPL-3 | Zero word and undefined low-5 codes return `None`. Spot-checked codes: 0, 0xFFE0, 10, 15, 29. |
| TC-UPL-4 | A full "V 7 1 ENTR" uplink sequence drives the V/N phase to `P27Address { … }`. End-to-end proof that uplink ≡ crew. |
| TC-UPL-5 | Noise codes interspersed in a "V 0 6 N 4 0 ENTR" sequence are skipped without raising OPR ERR. |
| TC-UPL-6 | `poll_uplink` on an empty FIFO leaves V/N phase unchanged. |
| TC-UPL-7 | `dsky.uplink_activity` toggles correctly across quiet → busy → quiet poll cycles. |
| TC-UPL-8 | An OprErr overrun (VERB while phase = OprErr) raises `UPLINK_TOO_FAST`, drops the key, leaves phase as OprErr; a subsequent RSET clears OprErr normally. |

---

## 6. Dependencies

| Dependency | Used for |
|---|---|
| `crate::hal::Uplink` | The trait being drained. |
| `crate::services::v_n::{Key, VnPhase, feed_key}` | Decode and inject. |
| `crate::tables::alarm_codes::{UPLINK_TOO_FAST, SITE_UPLINK}` | Alarm raise. |

No globals. No allocation. `no_std`.

---

## 7. Module Layout

```
src/services/uplink.rs
├── pub fn key_from_word(word: u16) -> Option<Key>
├── pub fn poll_uplink<U: Uplink>(state, uplink)
└── #[cfg(test)] mod tests              (TC-UPL-1..8)
```

---

## 8. Spec Quality Checklist

- [x] Drop semantics for zero words and unknown codes documented
      (§4.1 table) and matched against the AGC's NSTRT behaviour.
- [x] OprErr overrun rule — alarm + key drop, RSET pass-through —
      stated (§4.2).
- [x] Lamp duty cycle (set on drained tick, clear on quiet tick)
      called out (§4.2).
- [x] "Uplink ≡ crew" invariant (TC-UPL-4) noted as the central
      observational property (§4.3).
- [x] Wire-level protocol explicitly delegated to the HAL impl, not
      to this module (§1, §2).
- [x] Cross-references to `v_n-spec` (for the receiving state
      machine), `t4rupt-spec` (for the caller), `alarm-spec` and the
      `tables::alarm_codes` constants.
- [x] Dependencies enumerated (§6).
- [x] Test coverage spans decoder, drain, lamp, alarm, and the
      end-to-end V71 sequence (§5).
