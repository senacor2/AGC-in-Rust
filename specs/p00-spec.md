# Specification: `programs/p00` Module — CMC Idle (P00)

**Status**: Approved for implementation
**Module path**: `agc-core/src/programs/p00.rs`
**Architecture reference**: `docs/architecture.md` §7.2 "Programs for the Command Module", §6 "Restart Protection"
**Executive reference**: `specs/executive-spec.md` §4 (job priorities), §5.1 (FRESH START behaviour)
**DAP reference**: `specs/dap-spec.md` §14.1 (P00 / AttitudeHold interaction)
**SERVICER reference**: `specs/average-g-spec.md` §1 (start/stop lifecycle)
**State-vector reference**: `specs/state-vector-spec.md` §1 (StateVector definition)
**Conics reference**: `specs/conics-spec.md` §1 (on-demand conic propagation during coast)
**AGC source files**:
- `Comanche055/GROUND_TRACKING_DETERMINATION_PROGRAM.agc` — P00 entry block, CMC IDLING label
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — MOONFLAG, SURFFLAG, DAPBOOLS flagword bits
**Spec checklist**: `specs/README.md` — all items satisfied (see §9)

---

## 1. Purpose and Scope

`programs::p00` implements P00 — "CMC Idling" — the lowest-priority background
major mode of the Apollo Guidance Computer. It is the program the CMC enters at
FRESH START and the program the crew selects with V37E00E when no active
guidance task is required.

P00 is intentionally passive. Its init function performs the minimum safe
bookkeeping (set `major_mode`, cancel any active burn, transition the DAP to
attitude hold) and then exits. It does not register a repeating job or Waitlist
task. The AGC's cooperative scheduler idles naturally when no higher-priority
job is present — that quiescent state is the "running" state of P00.

### What this module provides

- `PRIORITY` constant: `JobPriority = 1` (lowest non-zero priority).
- `init(state: &mut AgcState) -> JobPriority` — entry point registered in
  `PROGRAM_TABLE[0]` and called by the V37 handler on V37E00E.

### What this module does NOT provide

- A repeating background job. P00 is passive after `init` returns.
- Explicit SERVICER management. P00 does not call `stop_servicer`; if a prior
  program left the SERVICER running (e.g., P11 orbital nav), it continues
  coasting and self-cancels on the next SERVICER-stop flag transition.
- DSKY formatting. Display output (V16N65 MET monitor) is issued via the
  `services::display` module (M5). The current stub defers this to the display
  milestone; this spec documents intent only.
- Any DAP mode computation. P00 calls `dap_init(AttitudeHold)` to hand off
  attitude control, then departs. DAP logic lives in `control::dap`.

---

## 2. AGC Background

### 2.1 P00 in Comanche055

In the original assembly, the P00 entry block appeared near the top of the
ground-tracking program file under the label `CMC IDLING` (sometimes spelled
`CMCIDL` in surrounding comments). The routine was reached either by a FRESH
START sequence (`FRESH START → GOPROG → P00`) or by the program-select handler
when the crew keyed V37E00E.

The AGC source established the following on entry:
1. Set `MODREG` (major-mode register, octal address 0007) to 00.
2. Called `P00DPSM` — "P00 descent-powered-flight stub" — which was a
   near-null routine that set the DAPBOOLS word to ATTHOLD configuration.
3. Issued `ENDOFJOB` to release the EXEC job slot.

Because P00 had no repeating job, the EXEC loop would find no ready jobs and
spin with the COMPUTER ACTIVITY lamp dark. Navigation continued passively if
the SERVICER was already running (its self-reschedule is independent of P00).

### 2.2 SERVICER and Coasting Flight

P00 is the correct state during coast flight between manoeuvres. Two navigation
modes are possible:

- **Active SERVICER** (entered from a prior navigation program): The 2-second
  SERVICER Waitlist task continues to run, updating `csm_state` via PIPA
  integration. P00 does not interfere with this.
- **Conic propagation** (no prior navigation program, or after SERVICER stops):
  Any program that needs the current state calls `navigation::conics` on demand
  to propagate `csm_state` forward from its last valid epoch. P00 itself makes
  no such call.

### 2.3 Default DSKY Display — V16N65

The crew display during P00 is Verb 16 (monitor), Noun 65 (MET):
- R1: hours (integer, centisecond-derived)
- R2: minutes
- R3: seconds

This is set by requesting a monitor display from `services::display`. The
display module handles refresh; P00 only requests it.

---

## 3. Rust API

### 3.1 Constants

```rust
/// Job priority for P00.
/// Priority 1 is the lowest non-zero value so any other program can preempt.
/// Priority 0 is reserved as the "empty slot" sentinel (executive-spec §4.1).
pub const PRIORITY: JobPriority = 1;
```

### 3.2 Entry Point

```rust
/// P00 — CMC Idle initialisation.
///
/// Called by the V37 program-select handler when the crew keys V37E00E, and
/// by the FRESH START sequence after clearing jobs and Waitlist tasks.
///
/// # Actions performed (in order)
/// 1. Sets `state.major_mode = 0`.
/// 2. Sets `state.burn.burn_active = false` and `state.engine_thrusting = false`
///    to cancel any active SPS burn.
/// 3. Clears `state.servicer_exit` (removes the P40 burn-exit callback if set).
/// 4. Activates the DAP in `AttitudeHold` mode via `control::dap::dap_init`.
/// 5. Requests V16N65 MET monitor display via `services::display` (deferred to M5).
///
/// # Returns
/// `PRIORITY` (1) — the caller (V37 handler) passes this to `executive.create_job`
/// if it needs to schedule a follow-on job. Because P00 has no background work,
/// the returned priority is used only for the (brief) init job itself; once
/// `init` returns, no further job is registered.
pub fn init(state: &mut AgcState) -> JobPriority
```

---

## 4. Behaviour Specification

### 4.1 Preconditions

- `state` is a valid `AgcState` (either freshly zeroed at FRESH START or
  carrying over state from a prior program).
- The call originates from the V37 handler or the FRESH START sequence;
  both guarantee that no job with higher priority is executing concurrently
  (cooperative scheduler).

### 4.2 Step-by-Step Actions

| Step | Field(s) modified | Value | Rationale |
|------|------------------|-------|-----------|
| 1 | `state.major_mode` | `0` | MODREG ← 00; DSKY PROG display shows "00" |
| 2 | `state.burn.burn_active` | `false` | Cancel any P40 burn arm/cutoff loop |
| 3 | `state.engine_thrusting` | `false` | Ensure ISR shim quenches SPS/TVC output |
| 4 | `state.servicer_exit` | `None` | Remove P40 burn-exit hook (safe for SERVICER to run headless) |
| 5 | `state.dap_state.mode` | `DapMode::AttitudeHold` | Via `control::dap::dap_init(state, DapMode::AttitudeHold)` |
| 6 | `state.dsky.prog` | `0` | PROG indicator shows "00" on DSKY |

Step 5 does not modify `state.dap_state.commanded_attitude`. If a prior program
commanded an attitude, P00 holds that attitude. If no attitude was commanded,
the field retains its FRESH START zero (body-frame aligned with inertial frame).

Step 6 is the only DSKY write in `init`. The V16N65 MET monitor display is
requested via `services::display::request_monitor(state, 16, 65)` (deferred to
display milestone M5; a no-op in the current stub).

### 4.3 What P00 Does NOT Do

- Does not call `stop_servicer`. The SERVICER is not P00's responsibility to
  cancel. Programs that started it (P11, P20, P40) are responsible for stopping
  it at their own termination, or it may continue coasting through P00.
- Does not reset `state.csm_state` or `state.target_state`.
- Does not reset `state.refsmmat`.
- Does not reset `state.flagwords` (these carry crew-configurable settings).
- Does not call `dap_stop`. P00 transitions the DAP to AttitudeHold, not off.

---

## 5. PROGRAM_TABLE Registration

`programs::mod::PROGRAM_TABLE[0]` is set to `Some(p00::init)`. The V37 handler
indexes this table by program number and calls the function pointer. No other
registration is required.

---

## 6. Restart Protection

P00 has no restart group and no phase register. After a hardware restart the
FRESH START path calls `p00::init` directly. Because P00 carries no computation
state, there is nothing to checkpoint.

If `state.major_mode` is read after restart and equals 0, the restart handler
infers that P00 was active and re-calls `p00::init` to re-establish DAP mode
and DSKY display.

---

## 7. Transitions

### Into P00

| Trigger | Source |
|---------|--------|
| FRESH START (V36E or power cycle) | `services::fresh_start` |
| Crew V37E00E | `services::v37_handler` → `PROGRAM_TABLE[0]` |
| Completion of P37 (midcourse correction) | P37 calls `PROGRAM_TABLE[0]` on completion |
| Abort from any P-program | Abort handler calls `PROGRAM_TABLE[0]` |

### Out of P00

Any V37ExxE with a valid program number replaces P00. The V37 handler calls
`PROGRAM_TABLE[xx]` which overwrites `state.major_mode` with the new value.
P00 has no `terminate` callback; the new program's `init` is responsible for
establishing its own safe configuration.

---

## 8. DSKY Display (Intent — deferred to M5)

When `services::display` is implemented, `p00::init` shall call:

```
services::display::request_monitor(state, verb=16, noun=65)
```

This requests the DSKY to show a continuously updating readout:
- **R1**: Mission elapsed time — hours  (derived from `state.time`)
- **R2**: Mission elapsed time — minutes
- **R3**: Mission elapsed time — seconds

Display refresh is driven by the display-service monitor cycle, not by P00.

---

## 9. Spec Quality Checklist

- [x] AGC source file referenced (GROUND_TRACKING_DETERMINATION_PROGRAM.agc, CMC IDLING label)
- [x] All state fields touched by `init` listed in §4.2 table
- [x] No fixed-point scaling required (P00 writes no navigation values)
- [x] Preconditions and postconditions stated (§4.1, §4.2)
- [x] Edge cases documented (§4.3 — what P00 does NOT do)
- [x] At least 4 test cases specified (§10)
- [x] Rust API signature with types (§3.2)
- [x] Invariants stated (§4.3)
- [x] Consistency with `docs/architecture.md` §7.2 confirmed
- [x] PROGRAM_TABLE registration documented (§5)

---

## 10. Test Cases

### TC-P00-1: `init` sets `major_mode` to 0

**Precondition**: `state.major_mode` is set to a non-zero value (e.g., 40,
simulating return from P40).

**Action**: Call `p00::init(&mut state)`.

**Expected**: `state.major_mode == 0`.

```rust
#[test]
fn tc_p00_1_sets_major_mode_zero() {
    let mut state = AgcState::new();
    state.major_mode = 40;
    p00::init(&mut state);
    assert_eq!(state.major_mode, 0);
}
```

---

### TC-P00-2: `init` returns `PRIORITY` (1)

**Precondition**: Fresh `AgcState`.

**Action**: Capture the return value of `p00::init(&mut state)`.

**Expected**: Return value equals `p00::PRIORITY` which equals `1`.

```rust
#[test]
fn tc_p00_2_returns_low_priority() {
    let mut state = AgcState::new();
    let prio = p00::init(&mut state);
    assert_eq!(prio, p00::PRIORITY);
    assert_eq!(prio, 1);
}
```

---

### TC-P00-3: `init` cancels an active burn

**Precondition**: `state.burn.burn_active = true` and
`state.engine_thrusting = true`, simulating a mid-burn state entry.

**Action**: Call `p00::init(&mut state)`.

**Expected**:
- `state.burn.burn_active == false`
- `state.engine_thrusting == false`
- `state.servicer_exit == None`

```rust
#[test]
fn tc_p00_3_cancels_active_burn() {
    let mut state = AgcState::new();
    state.burn.burn_active = true;
    state.engine_thrusting = true;
    state.servicer_exit = Some(dummy_exit_fn);
    p00::init(&mut state);
    assert!(!state.burn.burn_active);
    assert!(!state.engine_thrusting);
    assert!(state.servicer_exit.is_none());
}
```

---

### TC-P00-4: `init` leaves navigation state unchanged

**Precondition**: `state.csm_state` and `state.refsmmat` are populated with
non-zero values (simulating a valid nav state carried over from P11).

**Action**: Snapshot `csm_state` and `refsmmat`, then call `p00::init(&mut state)`.

**Expected**: Both fields are bit-for-bit identical after the call.

```rust
#[test]
fn tc_p00_4_leaves_nav_state_unchanged() {
    let mut state = AgcState::new();
    state.csm_state.position = [1.0e6, 2.0e6, 3.0e6];
    state.csm_state.velocity = [100.0, 200.0, 300.0];
    let pos_before = state.csm_state.position;
    let vel_before = state.csm_state.velocity;
    let refsmmat_before = state.refsmmat;
    p00::init(&mut state);
    assert_eq!(state.csm_state.position, pos_before);
    assert_eq!(state.csm_state.velocity, vel_before);
    assert_eq!(state.refsmmat, refsmmat_before);
}
```
