# Uplink (P27) Implementation Plan

**Status**: Approved
**Scope**: Complete the AGC uplink subsystem — V70/V71/V72/V73 verbs that allow Mission Control to update state vector, REFSMMAT, clock, and calibration constants — and wire the UPRUPT path from HAL to the V/N processor.
**Module owner**: `agc-core/src/services/v_n.rs` (V/N extensions), `agc-core/src/services/uplink.rs` (new), `agc-core/src/services/t4rupt.rs` (extend), `agc-sim/src/uplink.rs` (new).
**AGC source files**:
- `Comanche055/PINBALL_NOUN_TABLES.agc` — verb/noun definitions for V70/V71/V72/V73
- `Comanche055/KEYRUPT,_UPRUPT.agc` — UPRUPT interrupt handler
- `Comanche055/P27.agc` (implicit — set via DSKY verb dispatch)

---

## 1. Current state

Significantly more is already in place than the project tracking suggests:

| Component | Status |
|---|---|
| `hal/uplink.rs` — `Uplink` trait | Minimal (`read_word() -> Option<u16>`) but present |
| `services/v_n.rs` — V/N state machine | Substantial (2280 LOC) |
| V71 dispatch (`v71_p27_block_update`) | Implemented with 8 tests (`v_n.rs:651, 874–944`) |
| `VnPhase::P27Address` / `P27Count` / `P27Data` | Implemented |
| Address-space mapping for state-vector slots 1–6 (`p27_apply_word`) | Implemented |
| `dsky.uplink_activity` lamp field | Wired |
| DSKY shows `PROG = 27` during V71 entry | Done |

Missing:

| Gap | Location |
|---|---|
| **V70** (liftoff time update) | Not dispatched in `v_n.rs:643` |
| **V72** (single-address update) | Not dispatched |
| **V73** (AGC time update) | Not dispatched |
| **UPRUPT path** — uplink word → V/N keypress | `services/t4rupt.rs` is a 4-line stub |
| **REFSMMAT uplink** target | `p27_apply_word` address space stops at slot 6 (state vector only) |
| **gha_epoch_rad / gyro_comp / pipa_cal** uplink targets | Absent (only `csm_state` reachable) |
| **liftoff_time** field on `AgcState` | Does not exist |
| **Alarm 01106** ("uplink too fast") | Not raised |
| **agc-sim uplink driver** for scripted tests | Absent |

## 2. Operational priority for the mission

In order of mission-criticality for earth-to-moon-and-back:

1. **State vector** — already addressable via V71; needs validation via the UPRUPT path.
2. **REFSMMAT** — uploadable before LOI and after every IMU realignment.
3. **AGC clock correction** (V73) — drift over a multi-day mission.
4. **Calibration constants** (gyro_comp, pipa_cal) — refinement during cruise.
5. **GHA_epoch** — already noted as an uplink target in `services/backup.rs:32`.
6. **Liftoff time** (V70) — least critical post-flight; included for completeness.

## 3. Design decisions

1. **P27 representation**: implicit. No `programs/p27.rs` is created. `state.major_mode = 27` is set inside the V/N dispatch (as today in `v_n.rs:884`).
2. **Uplink-word format**: agc-core sees clean 5-bit DSKY key codes. The Apollo redundancy / complement / "uplink too fast" protocol is the responsibility of the bare-metal driver (`agc-board-nucleo-f767`) and of the simulator HAL impl. The `Uplink::read_word()` HAL contract returns post-validated key codes (lower 5 bits used, upper bits reserved / zero).
3. **REFSMMAT uplink path**: extend the existing V71/V72 block address space (see §5). No dedicated verb.
4. **`liftoff_time` storage**: new field `liftoff_time: Met` on `AgcState`, surviving FRESH START like `gha_epoch_rad` does (see `services/fresh_start.rs:326`).
5. **VirtualAGC fixtures**: skipped for this subsystem. The V/N processor has its own unit-test coverage and the uplink path is well-defined by the V/N state machine. Capture cost outweighs marginal validation value here.

## 4. Module layout

```
hal/uplink.rs                         (unchanged contract; documentation tightened)
  pub trait Uplink {
      fn read_word(&mut self) -> Option<u16>;
  }
services/uplink.rs                    (new — ~150 LOC)
  pub fn poll_uplink(state, hw)       // called from T4RUPT shim
  fn key_from_word(u16) -> Option<Key> // lower 5 bits → Key
services/t4rupt.rs                    (extend — currently 4-LOC stub)
  pub fn t4rupt_step(state, hw)
                                      // dispatch: DSKY refresh, uplink poll, downlink (later)
services/v_n.rs                       (extend)
  fn v70_liftoff_time_update(state)
  fn v72_single_address_update(state)
  fn v73_agc_time_update(state)
  // new VnPhase variants: P27Time, P27SingleAddress, P27SingleData
  // extend p27_apply_word address space — see §5
agc-sim/src/uplink.rs                 (new)
  pub struct ScriptedUplink            // implements hal::Uplink
                                      // Feeds a pre-loaded keystroke queue
```

## 5. Address space (V71 block / V72 single-word)

`p27_apply_word(address, value)` currently maps addresses 1–6 to `csm_state` position/velocity. Extended map:

| Address | Field | Crew units | AGC erasable correspondence |
|---|---|---|---|
| 1–3 | `csm_state.position[0..3]` | km | RN |
| 4–6 | `csm_state.velocity[0..3]` | m/s | VN |
| 7–9 | `target_state.position[0..3]` | km | RN (other vehicle) |
| 10–12 | `target_state.velocity[0..3]` | m/s | VN (other vehicle) |
| 13 | `gha_epoch_rad` | radians × 1e5 | GHABASE |
| 14–22 | `refsmmat[3×3]` row-major | revolutions × 1e5 (B-1) | REFSMMAT |
| 23–25 | `gyro_comp.{nbdx,nbdy,nbdz}` | meru × 1e3 | NBDX/NBDY/NBDZ |
| 26 | `pipa_cal.scale_factor` | ppm | PIPASCF |
| 27–29 | `pipa_cal.bias[0..3]` | cm/s² | PIPABIAS |
| 30 | `met_offset` (applied to `time`) | centiseconds | (TBD — V73 commit path) |

REFSMMAT scaling note: the AGC stored REFSMMAT in B-1 (half-revolutions). Uplink uses revolutions ×1e5 (signed) so a full ±0.5 rev fits in five decimal digits. The decode multiplies by 2π and writes radians to the rotation matrix element directly.

`P27_MAX_ADDRESS` rises from 6 to 30. Out-of-range addresses raise OPR ERR (existing behaviour). Final boundaries reviewed at MS-U3 implementation.

## 6. Implementation milestones

### MS-U1 — UPRUPT plumbing
- `services/uplink.rs::poll_uplink` drains `hw.uplink().read_word()`, decodes 5-bit key codes, feeds keypresses into `v_n::feed_key`.
- `services/uplink.rs::key_from_word` extracts the 5-bit code and maps to `Key` via the existing `Key::from_code`.
- `services/t4rupt.rs::t4rupt_step` implemented and wired into the T4RUPT ISR shim (replaces the 4-LOC stub).
- `agc-sim/src/uplink.rs::ScriptedUplink` — HAL impl that returns queued words.
- **Exit criterion**: in `agc-sim`, a scripted "V71 ENTR 01 ENTR 06 ENTR …" uplink sequence produces the same `csm_state` as direct `feed_key` calls. Existing V71 tests untouched and passing.

### MS-U2 — V70 + V73 time updates
- Add `liftoff_time: Met` to `AgcState`; preserve across FRESH START (mirror `gha_epoch_rad` handling in `services/fresh_start.rs`).
- New `VnPhase::P27Time` variant for the hours / minutes / seconds × 100 entry pattern (matches existing HMS conversion in `v_n.rs::noun_scale` HMS path).
- Dispatch V70 in `v_n.rs::dispatch_verb_noun` → `v70_liftoff_time_update`.
- Dispatch V73 in `v_n.rs::dispatch_verb_noun` → `v73_agc_time_update` (applies to `state.time`).
- **Exit criterion**: scripted V73 uplink advances `state.time` by the entered amount; V70 stores liftoff time and is recoverable via DSKY display (V06 N16 or similar).

### MS-U3 — Address-space expansion (REFSMMAT, calibration, GHA)
- Extend `p27_apply_word` per §5.
- Bump `P27_MAX_ADDRESS` to 30.
- Document each new address with AGC erasable correspondence in comments.
- REFSMMAT unit conversion: revolutions × 1e5 (signed integer) → radians (f64) via `value * 2π / 1e5`.
- **Exit criterion**: V71 sequence uploads a full REFSMMAT (9 words at addresses 14–22) and the result drives `state.refsmmat` correctly; round-trip test against a hand-computed orientation matrix. Calibration uploads (gyro_comp, pipa_cal) similarly verified.

### MS-U4 — V72 single-address update
- Dispatch V72 in `v_n.rs::dispatch_verb_noun` → `v72_single_address_update`.
- New `VnPhase::P27SingleAddress` and `VnPhase::P27SingleData` phases (single address then single signed word, no count).
- Reuse the §5 address space (no new addresses introduced here).
- **Exit criterion**: scripted V72 uplink updates one slot (e.g., `gyro_comp.nbdx`) without touching neighbours; OPR ERR on bad address.

### MS-U5 — End-to-end uplink scenarios and alarms
- Raise alarm 01106 ("UPLINK TOO FAST") from `poll_uplink` when the HAL produces words faster than the V/N processor can accept them (i.e., a keypress is received while `vn.phase` is mid-transition in a non-input-accepting state, or when the buffer overruns).
- Manage `dsky.uplink_activity` from `poll_uplink`: set when a non-empty word is drained, clear after a quiet T4RUPT cycle.
- New `agc-test/tests/uplink_scenarios.rs`:
  - State-vector reseed mid-cruise (V71, 6 words).
  - REFSMMAT uplink before LOI (V71, 9 words at addresses 14–22).
  - Clock correction via V73.
  - Single-word gyro_comp.nbdx update via V72.
- **Exit criterion**: all four scenarios pass; alarm 01106 fires under a forced overflow test.

## 7. Test strategy

- **Unit tests** in `services/uplink.rs::tests` — `key_from_word` edge cases (valid, unknown key, zero word), `poll_uplink` draining behaviour, alarm 01106 trigger.
- **New V/N tests** in `services/v_n.rs::tests` for V70, V72, V73 phase transitions and the extended address space.
- **Existing V71 tests** untouched and passing.
- **agc-sim integration** — `ScriptedUplink` drives end-to-end tests in `agc-test/tests/uplink_scenarios.rs`.
- **No VirtualAGC fixtures** for this subsystem (see §3 decision 5).

## 8. GitHub issue seed

The milestones map to issues in the `senacor2/AGC-in-Rust` repo under the existing `entry-guidance`-style scheme. New labels needed:

- New label `uplink` (color `#1d76db`) — scopes all issues in this plan.
- Existing labels `milestone`, `infrastructure`, `enhancement` reused.

Proposed issues (one per milestone, plus a parent tracking issue):

| Title | Labels |
|---|---|
| Uplink (P27) — implementation tracking | `uplink`, `milestone` |
| MS-U1: UPRUPT plumbing (HAL → V/N + agc-sim ScriptedUplink) | `uplink`, `milestone`, `infrastructure`, `enhancement` |
| MS-U2: V70 + V73 time updates and `liftoff_time` field | `uplink`, `milestone`, `enhancement` |
| MS-U3: V71/V72 address-space expansion (REFSMMAT, calibration, GHA) | `uplink`, `milestone`, `enhancement` |
| MS-U4: V72 single-address update | `uplink`, `milestone`, `enhancement` |
| MS-U5: End-to-end uplink scenarios and alarm 01106 | `uplink`, `milestone`, `enhancement` |

Each milestone issue links back to the parent and to this plan, lists its exit criterion verbatim, and gets closed only when its tests pass.
