# Specification: `programs/p61_p67` Module — Entry Guidance Programs

**Status**: Approved for implementation (Milestone 4 Phase 5 — skeletons)
**Module path**: `agc-core/src/programs/p61_p67.rs`
**Architecture reference**: `docs/architecture.md` §7.2 (P-programs), §10 (entry guidance)
**AGC source files**:
- `Comanche055/P61-P67.agc` — entry guidance program entry blocks
- `Comanche055/REENTRY_CONTROL.agc` — entry control law
- `Comanche055/ENTRY_LEXICON.agc` — entry-specific state variables

---

## 1. Purpose and Scope

P61–P67 are the family of entry-guidance programs that execute during
Earth atmospheric entry after the Trans-Earth Coast phase. They sequence
the vehicle through entry preparation, CM/SM separation, pre-0.05g
monitoring, closed-loop entry guidance, and final drogue deployment.

**This milestone (MS4 Phase 5) implements only skeletons.** The programs
establish the phase state machine, major-mode/DSKY wiring, and the
inter-program handoff contract. The real entry-guidance math — roll
steering law, lift-to-drag modulation, skip targeting, range prediction
— is a later milestone.

### What this module provides

- `EntryPhase` enum: `Idle → Preparation → Separation → PreEntry → Entry → Final`.
- `EntryState` struct in `AgcState`: current phase + sensed acceleration
  (g units) + stub roll command + stub target range.
- `P61_MAJOR_MODE … P67_MAJOR_MODE` constants.
- `PRIORITY: JobPriority = 10` — one tier above the background monitors.
- `init_p61 … init_p67` entry points.
- `p63_check_threshold` — advances `PreEntry → Entry` when
  `sensed_acceleration_g >= 0.05` (stub driver that tests call directly
  instead of wiring into the SERVICER loop).
- `p67_deploy_drogue` — sets a `drogue_deployed: bool` flag. Drogue
  hardware actuation is a HAL concern out of scope here.

### What this module does NOT provide

- The closed-loop entry guidance law (roll steering, L/D modulation,
  skip targeting, range prediction).
- Real sensed-acceleration integration — `sensed_acceleration_g` is
  written by the test harness, not by the SERVICER.
- CM/SM separation pyrotechnic commands — the HAL SECS interface does
  not yet exist; P62 only updates phase state and DAP mode.
- Drogue and main parachute HAL actuation.
- P65 and P66 (up-skip and ballistic phases).

---

## 2. `EntryState`

```rust
pub struct EntryState {
    pub phase: EntryPhase,              // Default: Idle
    pub sensed_acceleration_g: f64,     // Default: 0.0 — test-harness driven
    pub roll_command_rad: f64,          // Default: 0.0 — stub
    pub target_range_km: f64,           // Default: 0.0 — stub
    pub drogue_deployed: bool,          // Default: false
}
```

Added to `AgcState` as `entry: EntryState`, initialised to `Default::default()`.

---

## 3. Program Alarms

| Code | Trigger                                                    |
|------|------------------------------------------------------------|
| 231  | P62 invoked while entry phase is not `Preparation`.        |
| 232  | P63 invoked while entry phase is not `Separation`.         |
| 233  | P64 invoked while sensed_acceleration_g < 0.05 (pre-0.05g).|
| 234  | P67 invoked while entry phase is not `Entry`.              |

Alarms are "soft" — they set `alarm.code`/`alarm.lit` but do **not**
abort the program. The major mode is still advanced so the crew can
manually override if needed.

---

## 4. Program Behaviours

### 4.1 `init_p61` — Entry Preparation

- `entry.phase = Preparation`
- `major_mode = 61`, `dsky.prog = 61`, `dsky.verb = 6`, `dsky.noun = 61`
- Display `r[0]` = target range (stub 0), `r[1]` = 0, `r[2]` = 0
- No alarm.

### 4.2 `init_p62` — CM/SM Separation

- Alarm 231 if `entry.phase != Preparation` (but still advance).
- `entry.phase = Separation`
- `major_mode = 62`, `dsky.prog = 62`, `dsky.verb = 6`, `dsky.noun = 62`
- `state.pending_maneuver = None` (any stale ΔV is void post-separation).
- Transition DAP to `AttitudeHold` (CM-only RCS control).

### 4.3 `init_p63` — Pre-0.05g Entry Initialisation

- Alarm 232 if `entry.phase != Separation` (but still advance).
- `entry.phase = PreEntry`
- `major_mode = 63`, `dsky.prog = 63`, `dsky.verb = 16`, `dsky.noun = 64`
  (continuously updated entry status)
- `dsky.r[0]` = `entry.sensed_acceleration_g as f32`
- `dsky.r[1]` = 0 (stub)
- `dsky.r[2]` = 0 (stub)

### 4.4 `p63_check_threshold`

Called from tests (and eventually the SERVICER) with the current sensed
acceleration in g units already staged in `state.entry.sensed_acceleration_g`.

- If `entry.phase == PreEntry` and `sensed_acceleration_g >= 0.05`:
  - `entry.phase = Entry`
  - Return `true`.
- Otherwise return `false`.

### 4.5 `init_p64` — Closed-Loop Entry Guidance

- Alarm 233 if `entry.sensed_acceleration_g < 0.05` (early invocation).
- `entry.phase = Entry` (force, even if phase was PreEntry).
- `major_mode = 64`, `dsky.prog = 64`, `dsky.verb = 16`, `dsky.noun = 64`.
- `entry.roll_command_rad = 0.0` — stub.
- Display same triplet as P63.

### 4.6 `init_p67` — Drogue Deploy / Final Phase

- Alarm 234 if `entry.phase != Entry`.
- `entry.phase = Final`
- `major_mode = 67`, `dsky.prog = 67`, `dsky.verb = 6`, `dsky.noun = 67`.
- Call `p67_deploy_drogue(state)`.

### 4.7 `p67_deploy_drogue`

- `entry.drogue_deployed = true`.
- Future: call `hw.secs().deploy_drogue()`.

---

## 5. Test Cases

### TC-P61-1: `init_p61` sets phase = Preparation and major_mode = 61.

### TC-P62-1: `init_p62` from Preparation advances to Separation and clears pending_maneuver.

### TC-P62-2: `init_p62` from Idle raises alarm 231 but still advances.

### TC-P63-1: `init_p63` from Separation advances to PreEntry.

### TC-P63-2: `p63_check_threshold` with g = 0.04 returns false and stays PreEntry.

### TC-P63-3: `p63_check_threshold` with g = 0.08 returns true and advances to Entry.

### TC-P64-1: `init_p64` with g = 0.10 sets phase = Entry, no alarm.

### TC-P64-2: `init_p64` with g = 0.02 raises alarm 233.

### TC-P67-1: `init_p67` from Entry sets phase = Final and drogue_deployed = true.

### TC-P67-2: `init_p67` from Preparation raises alarm 234 but still advances.
