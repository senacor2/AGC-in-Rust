# Specification: `programs/p06` Module — CMC Power-down (P06)

**Status**: Approved for implementation
**Module path**: `agc-core/src/programs/p06.rs`
**Architecture reference**: `docs/architecture.md` §7.2 "Programs for the Command Module"
**Executive reference**: `specs/executive-spec.md` §5 (FRESH START / standby distinction)
**DAP reference**: `specs/dap-spec.md` — `dap_stop` semantics
**SERVICER reference**: `specs/average-g-spec.md` — `stop_servicer`
**Maneuver reference**: `specs/maneuver-spec.md` — `BurnState`, `pending_maneuver`
**AGC source files**:
- `Comanche055/FRESH_START_AND_RESTART.agc` — standby / power-down path

---

## 1. Purpose and Scope

P06 is **CMC Power-down**. It quiesces every active task so the computer
can be placed in a low-power standby state, lights the `STBY` indicator
on the DSKY, and waits for the crew to bring the CMC back up via
`V37 E 00 E` (P00).

Specifically, `init` performs every action required to leave the AGC in
a safe, deterministic state:

- cancels the SERVICER cycle and detaches any servicer-exit hook,
- stops the DAP (clears mode, staging fields, jet commands),
- drops any pending maneuver result,
- quenches the SPS burn state,
- updates the DSKY to display the STBY indicator and the V37 prompt.

P06 has **no continuation** — once `init` returns, the program owns no
running jobs or Waitlist tasks.

### What this module provides

- `P06_MAJOR_MODE: u8 = 6`.
- `PRIORITY: JobPriority = 1` (lowest non-zero priority — once quiesced
  P06 yields to any other job).
- `init(state)` — entry point registered in `PROGRAM_TABLE[6]`.

### What this module does NOT provide

- A repeating standby maintenance job. The CMC sits idle until the crew
  re-enters another program (typically P00).
- Any hardware-level power management (clocking down the IMU heaters,
  disabling discrete drivers, etc.). The `STBY` indicator is the only
  external signal; the rest is up to the spacecraft's primary electrical
  distribution.
- Persistence. P06 does not snapshot the state vector or REFSMMAT to
  any backup store; those fields are left untouched in `AgcState`.
- Restart-protection bookkeeping. After a GOJAM, the FRESH START path
  would re-enter P00, not P06.

---

## 2. AGC Background

In the original CMC, the standby sequence corresponded to the crew
pressing the `STANDBY` switch on Panel 1 (not a DSKY verb at all on
real Apollo missions, though some training references reuse program
number 06 for the equivalent software action). The AGC stopped all
periodic interrupts, retained the contents of erasable memory, and
illuminated the STBY indicator. Wake-up was via the same switch, which
called FRESH START.

In the Rust port, P06 is selectable via `V37 E 06 E` and effects the
same software-side quiescence on a flat-memory model.

---

## 3. Program Alarms

P06 raises no alarms.

---

## 4. Functional Requirements

### 4.1 `init`

In strict order:

| Step | Action | Field(s) modified |
|---|---|---|
| 1 | Cancel SERVICER cycle | (internal to `stop_servicer`) |
| 2 | Detach servicer exit hook | `state.servicer_exit = None` |
| 3 | Stop the DAP | `dap_stop(state)` — clears `dap_state.mode`, staging fields, RCS jet commands |
| 4 | Drop pending maneuver | `state.pending_maneuver = None` |
| 5 | Cancel burn state | `state.burn.burn_active = false` |
| 6 | Quench engine | `state.engine_thrusting = false` |
| 7 | Light STBY indicator | `state.dsky.stby = true` |
| 8 | Display P06 / V37 prompt | `state.dsky.prog = 6`, `verb = 37`, `noun = 0`, `flashing = false`, `comp_acty = false` |
| 9 | Set major mode | `state.major_mode = 6` |

Returns `PRIORITY` (1).

`init` is **idempotent**: calling it twice on the same state leaves the
state unchanged after the second call.

### 4.2 What `init` does NOT touch

- `state.csm_state`, `state.target` — navigation state preserved.
- `state.refsmmat` — alignment preserved.
- `state.imu_alignment_state` — IMU alignment preserved.
- `state.time` — mission clock continues to advance.
- `state.flagwords` — crew-configurable flag bits preserved.
- `state.gha_epoch_rad` — Earth-rotation epoch preserved.

---

## 5. PROGRAM_TABLE Registration

```rust
PROGRAM_TABLE[6] = Some(p06::init);
```

---

## 6. Restart Protection

P06 has no restart group. The standby state is reachable only by
explicit crew action. If a hardware restart occurs during P06, the
FRESH START path enters P00; the crew can re-enter standby with another
`V37 E 06 E`.

---

## 7. Transitions

### Into P06

| Trigger | Source |
|---|---|
| Crew `V37 E 06 E` | V37 handler → `PROGRAM_TABLE[6]` |

### Out of P06

| Trigger | Source |
|---|---|
| Crew `V37 E 00 E` | V37 handler → `PROGRAM_TABLE[0]` (P00). P00's `init` clears `state.dsky.stby` only indirectly via display refresh; the spec for P00 (§4) does not currently override `stby`. Re-establishing operational state is the job of P00. |
| Crew `V37 E xx E` for any other valid program | V37 handler → `PROGRAM_TABLE[xx]` |

---

## 8. Test Cases

The implementation in `agc-core/src/programs/p06.rs::tests` provides:

| ID | What is verified |
|---|---|
| TC-P06-1 | `init` from an active configuration (DAP in TVC mode, servicer_exit installed, `pending_maneuver = Some(...)`, burn active, engine thrusting) leaves every one of those fields quiesced and `dsky.stby = true`. Priority = 1. `major_mode = 6`. |
| TC-P06-2 | `init` is idempotent — two back-to-back calls on a fresh state both leave `major_mode = 6` and `dsky.stby = true`. |

A future audit (issue #159) should add coverage for:
- TC-P06-3 (suggested): `init` preserves `csm_state`, `refsmmat`, `imu_alignment_state`, and `time`.

---

## 9. Spec Quality Checklist

- [x] AGC source file referenced (`FRESH_START_AND_RESTART.agc`).
- [x] All state fields touched by `init` enumerated (§4.1).
- [x] Fields explicitly **not** touched enumerated (§4.2).
- [x] Idempotence stated and tested (§4.1 / §8 TC-P06-2).
- [x] No restart group required — documented (§6).
- [x] PROGRAM_TABLE registration documented (§5).
- [x] Test coverage summarised (§8).
- [x] Consistency with `dap_stop` and `stop_servicer` interfaces confirmed.
