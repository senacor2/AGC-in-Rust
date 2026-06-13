# Specification: `programs/p15` Module — Trans-Lunar Injection Monitor (P15)

**Status**: Approved for implementation
**Module path**: `agc-core/src/programs/p15.rs`
**Architecture reference**: `docs/architecture.md` §7.2 "Programs for the Command Module"
**Companion spec**: `specs/p11-spec.md` — P11 shares the compute pipeline
**Conics reference**: `specs/conics-spec.md` — `sv_to_elements`, `apoapsis_altitude_earth`, `periapsis_altitude_earth`, `orbital_period`
**State-vector reference**: `specs/state-vector-spec.md` §2.1 (Frame), §3 (StateVector)
**SERVICER reference**: `specs/average-g-spec.md` §5 (`servicer_exit` hook)
**AGC source files**:
- `Comanche055/P11.agc` — shared block for the P11/P15 EOI/TLI monitor

---

## 1. Purpose and Scope

P15 is the **Trans-Lunar Injection Monitor**. It runs during the S-IVB
TLI burn and the early translunar coast, displaying continuously the
current trajectory's apoapsis altitude, periapsis altitude, and half
of the orbital period — the same Verb 16 Noun 44 triplet that P11
maintains.

P15 differs from P11 in only two visible ways:

1. The major-mode number / PROG field is `15` rather than `11`.
2. P15 raises a **distinct alarm code** for a hyperbolic trajectory
   (which is expected after a successful TLI burn) so the crew can
   distinguish the post-TLI condition from a generic P11 hyperbolic
   alarm.

Like P11, P15 commands no actuators — it is a passive monitor. Once
the spacecraft's velocity exceeds escape velocity (i.e. the orbit is
unbounded with respect to Earth), N44's `r1`/`r2`/`r3` registers can no
longer represent the orbit and P15 raises alarm 237 and freezes the
last good display.

### What this module provides

- `P15_MAJOR_MODE: u8 = 15`.
- `PRIORITY: JobPriority = 6` — background monitor tier.
- `init(state)` — entry point registered in `PROGRAM_TABLE[15]`.
  Validates the frame, sets major mode, installs `p15_servicer_exit`,
  and performs an immediate first update.
- `p15_update(state)` — pure recomputation of the N44 display from
  `state.csm_state`. Called by the hook and directly from tests.
- `p15_servicer_exit(state)` — SERVICER exit hook (thin wrapper around
  `p15_update`).

### What this module does NOT provide

- A dedicated km vs metre display. The DSKY registers carry **metres**
  (`f32`); display-format selection is a milestone-deferred concern, see
  `specs/p11-spec.md` §1 for the matching note.
- A true TFF computation. The third register carries
  `orbital_period_seconds / 2` (half-period); a proper Time-From-Fictitious-Perigee
  is a future milestone.
- Frame switching. P15 is bound to `Frame::EarthInertial`. Transition
  into the lunar SOI is the SERVICER's responsibility and is signalled
  by the state vector switching to `Frame::MoonInertial`; the crew is
  expected to V37 into P00 or P23 at that point.

---

## 2. AGC Background

In the original CMC, P11 and P15 shared the same code block (`P11.agc`)
with branch-on-major-mode selecting between the two PROG displays. The
underlying orbital-elements computation was identical; P15 simply
inherited the EOI-monitor's apo/peri readout because it was useful
during the TLI burn too.

The Rust port keeps the same architecture: `p15_update` reuses the
`navigation::conics` helpers (which themselves are the modern
replacement for the AGC's HANGLE/REVUP routines) and only the alarm
codes and PROG number differ between P11 and P15.

---

## 3. Program Alarms

| Code | Trigger |
|---|---|
| 236 (`ALARM_WRONG_FRAME`) | `csm_state.frame != EarthInertial` at `init`. Returns without entering the major mode. |
| 237 (`ALARM_HYPERBOLIC`) | `csm_state` describes a hyperbolic trajectory (`OrbitalElements::is_hyperbolic`). Display is **not** overwritten — the last good apo/peri readout is preserved. |

(P11 raises alarm 229 / 230 for the same two conditions; the distinct
P15 numbers let the crew read alarms code-and-PROG together to know
which monitor flagged.)

---

## 4. Functional Requirements

### 4.1 `init`

1. If `state.csm_state.frame != Frame::EarthInertial`, raise alarm 236
   and return `PRIORITY` without further state changes.
2. Set `state.major_mode = 15`, `state.dsky.prog = 15`.
3. Set `state.dsky.verb = 16` (monitor), `state.dsky.noun = 44`
   (apo/peri/half-period).
4. Set `state.dsky.flashing = false`.
5. Install the servicer exit hook:
   `state.servicer_exit = Some(p15_servicer_exit)`.
6. Call `p15_update(state)` once so the display reflects the current
   orbit at program selection time.
7. Return `PRIORITY` (6).

### 4.2 `p15_update`

1. Convert `state.csm_state` to `OrbitalElements` via `sv_to_elements`.
2. If `elements.is_hyperbolic()`, raise alarm 237 and return without
   modifying the display (preserves last good N44).
3. Otherwise compute:
   - `apo_m = apoapsis_altitude_earth(&elements)` (metres above mean
     Earth radius)
   - `peri_m = periapsis_altitude_earth(&elements)` (metres)
   - `half_period_s = orbital_period(&elements, MU_EARTH) / 2.0`
4. Write to DSKY:
   - `dsky.r[0] = apo_m as f32`
   - `dsky.r[1] = peri_m as f32`
   - `dsky.r[2] = half_period_s as f32`

### 4.3 `p15_servicer_exit`

Thin wrapper that calls `p15_update`. Refreshes the N44 display every
2 s while P15 is active.

---

## 5. PROGRAM_TABLE Registration

```rust
PROGRAM_TABLE[15] = Some(p15::init);
```

---

## 6. Restart Protection

P15 carries no per-program restart state; the display is fully
re-derivable from `state.csm_state` on every SERVICER cycle. If a
restart re-dispatches P15, `init` re-installs the servicer-exit hook
and computes a fresh display.

---

## 7. Transitions

### Into P15

| Trigger | Source |
|---|---|
| Crew `V37 E 15 E` | V37 handler → `PROGRAM_TABLE[15]` |

### Out of P15

Any V37 to another major mode replaces P15. The new program's `init`
should install its own `servicer_exit` (or `None`); otherwise the P15
hook continues firing harmlessly until cleared.

When the trajectory becomes hyperbolic (post-TLI cutoff with sufficient
energy), the crew typically switches to P23 (cislunar navigation) or
P00.

---

## 8. Test Cases

The implementation in `agc-core/src/programs/p15.rs::tests` provides:

| ID | What is verified |
|---|---|
| TC-P15-1 | `init` on a 400 km circular LEO sets `major_mode = 15`, `PROG = 15`, NOUN = 44; populates apogee ≈ perigee ≈ 400 000 m within 10 m; installs the servicer-exit hook. |
| TC-P15-2 | `init` on a `MoonInertial` state vector raises alarm 236 and leaves `major_mode` unchanged. |
| TC-P15-3 | `init` on a hyperbolic trajectory (v > 1.2 × v_escape) raises alarm 237 and **does not** overwrite a pre-existing N44 display (last good values are preserved). |

A future audit (issue #159) should add coverage for:
- TC-P15-4 (suggested): elliptic post-EOI orbit (e.g. 200 × 500 km) reads correct apo/peri values.
- TC-P15-5 (suggested): `p15_servicer_exit` updates the display after a state-vector change.

---

## 9. Spec Quality Checklist

- [x] AGC source file referenced (`P11.agc` shared block).
- [x] All state fields touched by `init` enumerated (§4.1).
- [x] Companion-spec cross-reference to P11 (§1, §2).
- [x] Alarms documented and distinguished from P11's (§3).
- [x] Rust API signatures (§1, §4).
- [x] PROGRAM_TABLE registration documented (§5).
- [x] Test coverage summarised (§8).
- [x] Behaviour on hyperbolic trajectory explicitly defined (§4.2, §3).
