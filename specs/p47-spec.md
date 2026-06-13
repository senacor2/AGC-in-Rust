# Specification: `programs/p47` Module — Thrust Monitor (P47)

**Status**: Approved for implementation
**Module path**: `agc-core/src/programs/p47.rs`
**Architecture reference**: `docs/architecture.md` §7.2 "Programs for the Command Module"
**SERVICER reference**: `specs/average-g-spec.md` §5 (`servicer_exit` hook), §6 (`servicer_last_dv_inertial`)
**Maneuver reference**: `specs/maneuver-spec.md` — `BurnState`, inertial ΔV accumulation
**AGC source files**:
- `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc` — monitoring path

---

## 1. Purpose and Scope

P47 is the **Thrust Monitor**. It is a passive display-only program
that exposes the inertial ΔV the SERVICER integrated during the most
recent 2-second cycle. The crew uses P47 to verify or characterise an
uncommanded or non-nominal thrust event — for example, an RCS jet
sticking, a leak, or a ground-uplinked maneuver that has not yet been
sequenced through P40.

P47 **never commands any actuator**. It does not arm or fire the SPS,
does not configure the DAP, does not switch RCS modes, and does not
touch the burn state machine. Selecting P47 while a real burn is
in progress (P40 active) will overwrite the servicer-exit hook with
`p47_servicer_exit` — which the operator should avoid in flight; in
the Rust port this is the crew's responsibility, the program does not
guard against it.

### What this module provides

- `P47_MAJOR_MODE: u8 = 47`.
- `PRIORITY: JobPriority = 6` — background monitor tier.
- `init(state)` — entry point registered in `PROGRAM_TABLE[47]`.
  Sets the major mode, installs the servicer-exit hook, and performs an
  immediate first display update.
- `p47_update(state)` — pure copy of `state.servicer_last_dv_inertial`
  into the DSKY `r[0..3]` registers.
- `p47_servicer_exit(state)` — SERVICER exit hook (thin wrapper around
  `p47_update`).

### What this module does NOT provide

- Any cutoff logic. P47 just observes; the displayed values do not
  influence the burn state machine.
- LVLH-frame display. The triplet shown is **inertial-frame
  components**, in the same units as the SERVICER stages them. A future
  enhancement could add an LVLH-frame Noun, but the current scope shows
  N83 only.
- Smoothing or filtering. The displayed value is the *most recent
  cycle's* ΔV — it changes at the SERVICER cadence.

---

## 2. AGC Background

Historically, the AGC presented a thrust-monitor display as part of
the powered-flight sub-routine set. The crew could request the
accumulated specific-impulse vector to verify that, for instance, a
post-cutoff RCS settling burn had stopped or that a residual SPS
"chuff" had no measurable effect on velocity.

In the Rust port, the SERVICER stages the most recent integrated ΔV
into `state.servicer_last_dv_inertial` (`Vec3`, m/s, inertial frame) on
every cycle. P47's only job is to surface that staged value on the
DSKY.

---

## 3. DSKY Display

| Verb | Noun | Register | Meaning |
|---|---|---|---|
| 16 (monitor) | 83 (ΔV components) | R1 | `servicer_last_dv_inertial[0]` (m/s) |
|              |                    | R2 | `servicer_last_dv_inertial[1]` (m/s) |
|              |                    | R3 | `servicer_last_dv_inertial[2]` (m/s) |

`flashing = false`. The display refreshes once per SERVICER cycle
(2 s nominally).

---

## 4. Program Alarms

P47 raises no alarms.

---

## 5. Functional Requirements

### 5.1 `init`

1. Set `state.major_mode = 47`, `state.dsky.prog = 47`.
2. Set `state.dsky.verb = 16` (monitor), `state.dsky.noun = 83`
   (ΔV components).
3. Set `state.dsky.flashing = false`.
4. Install the servicer exit hook:
   `state.servicer_exit = Some(p47_servicer_exit)`.
5. Call `p47_update(state)` once so the display reflects the last staged
   ΔV (zero at program start unless a prior SERVICER cycle has run).
6. Return `PRIORITY` (6).

### 5.2 `p47_update`

```rust
let dv = state.servicer_last_dv_inertial;
state.dsky.r[0] = dv[0] as f32;
state.dsky.r[1] = dv[1] as f32;
state.dsky.r[2] = dv[2] as f32;
```

Pure write — no precondition checks. If `servicer_last_dv_inertial` is
zero (no prior SERVICER cycle), the display reads zero across all three
registers, which is the correct semantics.

### 5.3 `p47_servicer_exit`

Thin wrapper that calls `p47_update`. Refreshes the N83 display every
2 s while P47 is active.

---

## 6. PROGRAM_TABLE Registration

```rust
PROGRAM_TABLE[47] = Some(p47::init);
```

---

## 7. Restart Protection

P47 carries no per-program restart state; the display is re-derived
each SERVICER cycle. If a restart re-dispatches P47, `init` re-installs
the servicer-exit hook and displays whatever `servicer_last_dv_inertial`
holds at that moment.

---

## 8. Transitions

### Into P47

| Trigger | Source |
|---|---|
| Crew `V37 E 47 E` | V37 handler → `PROGRAM_TABLE[47]` |

### Out of P47

Any V37 to another major mode replaces P47. The new program's `init`
should install its own `servicer_exit` (or `None`); otherwise the P47
hook continues firing harmlessly until cleared.

---

## 9. Test Cases

The implementation in `agc-core/src/programs/p47.rs::tests` provides:

| ID | What is verified |
|---|---|
| TC-P47-1 | `init` sets `major_mode = 47`, `PROG = 47`, NOUN = 83, VERB = 16, and installs the servicer-exit hook. Priority = 6. |
| TC-P47-2 | After staging `servicer_last_dv_inertial = [1.5, -0.7, 0.3]` and invoking `p47_servicer_exit`, the DSKY r[0]/r[1]/r[2] match within 1e-6. |

A future audit (issue #159) should add coverage for:
- TC-P47-3 (suggested): default display after `init` with no prior SERVICER cycle reads zero.
- TC-P47-4 (suggested): an in-progress P47 hook is replaced when the crew switches to P40 (servicer_exit overwritten to `burn_servicer_exit`).

---

## 10. Spec Quality Checklist

- [x] AGC source file referenced (`POWERED_FLIGHT_SUBROUTINES.agc`).
- [x] DSKY V/N layout documented (§3).
- [x] All state fields touched by `init` enumerated (§5.1).
- [x] No alarms / no actuator commands explicitly stated (§1, §4).
- [x] PROGRAM_TABLE registration documented (§6).
- [x] Test coverage summarised (§9).
- [x] Inertial-vs-LVLH frame convention explicitly noted (§1, §3).
