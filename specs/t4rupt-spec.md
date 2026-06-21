# Specification: `services/t4rupt` Module — Periodic I/O Shim

**Status**: Approved for implementation
**Module path**: `agc-core/src/services/t4rupt.rs`
**Architecture reference**: `docs/architecture.md` §6 (Executive), §7 (Services)
**Related specs**:
- `specs/executive-spec.md` — owns the `Executive::run` loop that calls `t4rupt_step` (bare-metal path).
- `specs/uplink-spec.md` — `poll_uplink` (drained inside `t4rupt_step`).
- `specs/downlink-spec.md` — `downlink_step` and `DownlinkDriver` (drained six times per `t4rupt_step`).
- `specs/hal-spec.md` — `AgcHardware` and the `Uplink` / `Telemetry` sub-traits used here.
**Glossary cross-reference**: `docs/glossary.md` — T4RUPT, DOWNRUPT, UPRUPT.
**AGC source files**:
- `Comanche055/DOWN-TELEMETRY_PROGRAM.agc` — `DODOWNTM` driven by DOWNRUPT every 20 ms (the cadence this shim approximates).
- `Comanche055/KEYRUPT,_UPRUPT.agc` — UPRUPT ISR (the source of the uplink words this shim drains).

---

## 1. Purpose and Scope

`services::t4rupt` is the **periodic-I/O shim** that the
host-side simulator (`agc-sim`) calls every ~120 ms to drive the
uplink and downlink paths without spinning up the full bare-metal
Executive loop. On the bare-metal target the same logic is performed
inline by `executive::scheduler::Executive::run` — this module's
existence is for unit-testable, host-side composition.

The 120 ms cadence approximates the AGC's T4RUPT period; the AGC's
DOWNRUPT actually fires every 20 ms (50 Hz), so the shim runs
`downlink_step` six times per call (6 × 20 ms = 120 ms) to keep the
2-second CMCSTADL cycle on schedule.

### What this module provides

- `DOWNRUPTS_PER_T4: usize = 6` — number of downlink word-pairs
  emitted per tick.
- `t4rupt_step<H: AgcHardware>(state, hw)` — the single entry point.

### What this module does NOT provide

- The DSKY display refresh. That happens elsewhere
  (`services::pinball::decode_dsky` is called by a separate display
  shim — the agc-sim `T4Pump` invokes it directly, the bare-metal
  Executive does so on every cycle).
- Gyro-drift compensation / IMU monitoring. The historical AGC's
  T4RUPT also did periodic IMU checks; in the Rust port these are
  driven by the SERVICER and the `services::imu_control` module, not
  by this shim.
- Any timing primitive. The caller (the simulator's `T4Pump` or the
  Executive's main loop) decides when to invoke `t4rupt_step`.

---

## 2. AGC Background

In Comanche055, T4RUPT was the periodic-I/O interrupt that ran every
~120 ms. Its handler cycled through display update, IMU status
monitoring, uplink, and downlink. DOWNRUPT was a separate, faster
(~20 ms) interrupt dedicated to driving the two downlink output
channels (34 and 35).

The Rust port collapses display refresh into a separate shim and
combines uplink poll + a six-downrupt downlink drain into this single
`t4rupt_step` function. The 6:1 ratio recovers the historical 20 ms
downlink cadence within the 120 ms simulator cycle.

---

## 3. Rust API

### 3.1 Constant

```rust
const DOWNRUPTS_PER_T4: usize = 6;
```

Private — the only use is inside `t4rupt_step`.

### 3.2 Function

```rust
pub fn t4rupt_step<H: AgcHardware>(state: &mut AgcState, hw: &mut H);
```

`no_std`-safe; takes mutable access to both `AgcState` and `H` because
the uplink and telemetry sub-traits are mutable.

---

## 4. Functional Requirements

### 4.1 `t4rupt_step`

In order:

1. **Uplink drain** — call
   `services::uplink::poll_uplink(state, hw.uplink())`. This drains
   every queued uplink word into the V/N state machine, sets the
   `uplink_activity` lamp if any word arrived, and raises alarm 01106
   if a non-RSET key arrives while the V/N is locked in `OprErr`.
   See `specs/uplink-spec.md`.

2. **Downlink emit** — six successive
   `services::downlink::downlink_step(driver, state, hw.telemetry())`
   calls. Each call sends one 30-bit word-pair (two 15-bit words via
   `Telemetry::send_word`) and advances `driver.pair_index` by 1
   modulo 100.

   The driver lives in `state.downlink`. The implementation splits the
   borrow (`let mut driver = state.downlink; downlink_step(&mut driver, state, …); state.downlink = driver;`)
   so the read borrow of `state` and the write borrow of the driver
   do not overlap. This is the same split-borrow pattern used in the
   bare-metal Executive loop.

### 4.2 Cadence assumption

`t4rupt_step` does not consult any clock. The caller is expected to
invoke it every ~120 ms. Drift from that cadence shows up as drift in
the downlink rate; the 2-second cycle resets every 100 calls
(`pair_index` wraps), so persistent under- or over-call simply
stretches or compresses the CMCSTADL cycle but does not corrupt it.

---

## 5. Dependencies

| Dependency | Used for |
|---|---|
| `crate::hal::AgcHardware` | Generic host bound; gives access to `uplink()` and `telemetry()`. |
| `crate::services::downlink::downlink_step` | Drains six pairs per call. |
| `crate::services::uplink::poll_uplink` | Drains the uplink FIFO. |

No global state. No allocation. `no_std` compatible.

---

## 6. Module Layout

```
src/services/t4rupt.rs
├── const DOWNRUPTS_PER_T4: usize = 6
└── pub fn t4rupt_step<H: AgcHardware>(state: &mut AgcState, hw: &mut H)
```

No `#[cfg(test)] mod tests` in the file itself — `t4rupt_step` is
exercised end-to-end through `agc-test/tests/uplink_scenarios.rs` and
`agc-test/tests/full_mission.rs` (downlink stream length check).

---

## 7. Spec Quality Checklist

- [x] T4RUPT vs DOWNRUPT cadence relationship documented (§2, §4.1).
- [x] The single public function has its precondition (called every
      ~120 ms), step ordering, and split-borrow rationale documented
      (§4.1).
- [x] Out-of-scope items (display refresh, gyro-drift comp) explicitly
      listed (§1).
- [x] Dependencies listed (§5).
- [x] Test coverage pointed to consumer integration tests (§6).
