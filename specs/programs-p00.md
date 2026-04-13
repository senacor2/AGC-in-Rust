# Spec: P00 — CMC Idle Program (POOH / DUMMYJOB)

## AGC Source Reference

```
File:     Comanche055/FRESH_START_AND_RESTART.agc
Routines: GOTOPOOH (page 194), POOH (page 200), GOP00FIX (page 195),
          DUMMYJOB (page 1220 in EXECUTIVE.agc)
File:     Comanche055/EXECUTIVE.agc
Routine:  DUMMYJOB
Pages:    1220 (DUMMYJOB idle loop), 194-195 (GOTOPOOH / GOP00FIX)
```

## Behavior Summary

P00 is the CMC idle program — the resting state to which the computer returns whenever no active navigation or guidance program is running. It is also called "POOH" in the source (see: `# DO NOT USE GOPROG2 OR ENEMA WITHOUT CONSULTING POOH PEOPLE`).

### Entry paths

1. **FRESH START / RESTART**: after `DOFSTART` or `GOPROG` completes initialization, control passes to `DUMMYJOB +2` (a `RELINT` instruction) via a `POSTJUMP CADR DUMMYJOB+2`. This is the unconditional landing pad after any complete reset.
2. **V37 N00 ENTER (crew-requested)**: the operator enters Verb 37, Noun 00 (or just Enter on the MM display). `V37` stores `MMNUMBER = 0`, then falls through to `POOH`. The POOH label: releases the DSKY display (`TC RELDSP`), sets the restart register to PRIO5, kills all restart groups except Group 2 (nine-minute integration cycle), stores MMNUMBER into MODREG, and jumps to GOPROG2 (an alias for ENEMA) which reschedules the servicer.
3. **Phase-table failure with MODREG = -0**: at restart, if no restart groups are active and MODREG reads as -0 (i.e., no program was running), `GOTOPOOH` is called directly.

### GOTOPOOH routine

`GOTOPOOH` first sets a restart-protect phase (`TC PHASCHNG / OCT 14`) to mark itself as restartable from the beginning, then calls `POSTJUMP CADR GOP00FIX`. `GOP00FIX` calls `INITSUB`, clears any mark, then flashes `V37 N99` — the "select major mode" prompt — on the DSKY and waits for a crew keypress. On any keypress it loops back on itself, effectively suspending the computer in idle until the crew enters a valid V37 program selection.

### DUMMYJOB (Executive idle loop)

When the Executive's job-scan finds no runnable jobs, it calls `DUMMYJOB` instead of a CHANJOB:
- Sets `NEWJOB = -0` (indicates idling, no active job).
- `RELINT` (re-enables interrupts).
- Turns OFF the Activity (green) light (`CS TWO / WAND DSALMOUT`).
- Spins in `ADVAN`: checks if a `NEWJOB` has appeared. If yes → `NUCHANG2` → dispatches the highest-priority job. If no → cycles back into `ADVAN`.
- Runs the SELFCHK self-test routine (stored as a switched-bank job) on each idle cycle.

DUMMYJOB is **not** a job itself; it is a fall-through subroutine of the Executive. The Rust equivalent is the `executive::scheduler` idle path.

### What P00 does NOT do

- P00 does not command engines, jets, or IMU.
- P00 does not integrate the state vector.
- P00 does not block the IMU T3RUPT / T4RUPT interrupt handlers — they continue.

### Transitions out of P00

The only exit from P00 is a valid `V37 Nxx ENTER` from the crew selecting a non-zero major mode. `V37` validates the MM number against the `PREMM1` allowed-mode table, then schedules `V37XEQ` as a job, clears all restart groups except Group 4 (P37-related), and eventually calls the entry point of the selected program.

The NODO V37 flag (`FLAGWRD2` bit `NODOBIT`) can block a V37 request during critical maneuver phases and causes alarm 1520.

## Rust API

Module path: `agc_core::programs::p00_idle`

```rust
/// Enter the P00 idle state.
///
/// Sets `state.modreg` to `MODREG_NONE` (the P00 sentinel value).
/// Clears the DSKY program display register (PROG light shows "00").
/// Does NOT affect the Executive job table or Waitlist.
/// Does NOT command any hardware.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc
///   GOTOPOOH (page 194), POOH (page 200), GOP00FIX (page 195).
pub fn enter(state: &mut AgcState, hw: &mut dyn AgcHardware);

/// Returns `true` when the current major mode register indicates P00 is active.
///
/// Condition: `state.modreg == MODREG_NONE` (value 0, the zero major-mode).
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc V37 routine at label
///   `ISITP00`: `CA MMNUMBER / EXTEND / BZF ISSERVON`.
pub fn is_active(state: &AgcState) -> bool;
```

### AgcState fields used

| Field | AGC register | Notes |
|---|---|---|
| `state.modreg` | `MODREG` | Holds current major mode (0 = P00) |
| `state.dsky.prog` | `DSPTAB` (program display digits) | Shows "00" during P00 |

### Scale factors

No numerical computation. All values are program-mode selector integers.

### Restart safety

- `GOTOPOOH` calls `TC PHASCHNG / OCT 14` which sets Group 1 phase = 4 and Group 3 phase = 4, protecting the GOTOPOOH → GOP00FIX transition across restarts.
- In Rust: `enter()` must call `state.restart.set_phase(1, 4)` before writing `state.modreg`. The restart handler checks this phase and re-calls `enter()` on restart.

## Invariants

1. After `enter()` returns, `is_active()` returns `true`.
2. `enter()` must not issue any engine or jet commands.
3. `enter()` is idempotent: calling it while already in P00 is a no-op (GOTOPOOH handles this).
4. No heap allocation. No blocking. Callable from the Executive job context.
5. The DSKY V37N99 flash is handled by the pinball/display layer, not by this module. `enter()` only sets `state.modreg` and calls `hw.dsky().set_prog(0)`.

## Test Cases

### TC-P00-1: Enter sets modreg to zero
```
Setup:    state.modreg = 11  (simulating an active P11)
Action:   p00_idle::enter(&mut state, &mut hw)
Expected: state.modreg == 0 (MODREG_NONE)
          p00_idle::is_active(&state) == true
```

### TC-P00-2: is_active is false when another program is active
```
Setup:    state.modreg = 30  (P30 active)
Action:   p00_idle::is_active(&state)
Expected: false
```

### TC-P00-3: Enter is idempotent
```
Setup:    p00_idle::enter(&mut state, &mut hw)   (state is now P00)
Action:   p00_idle::enter(&mut state, &mut hw)   (called again)
Expected: no panic, no alarm, state.modreg == 0,
          is_active(&state) == true
```

## agc-sim Impact

- `DskyState`: `enter()` calls `hw.dsky().set_prog(0)` — renders "00" in the PROG display.
- `SimLog`: emit `log::info!("P00 idle entered")` from `enter()`.
- No new keyboard bindings required; V37 handling already dispatches to this module.
- No new TUI panels required.
