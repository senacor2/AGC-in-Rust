# Functional Specification: AGC Reentry Crew Workflow (P61–P67)

**Status**: Reference document for issue #49 (P62→P63 wake-gap debugging).
**AGC source**: Comanche055 (Apollo 11 Command Module, revision 055, April 1969).

**Primary sources**:
- Comanche055 `.agc` listing (`P61-P67.agc`, `CM_ENTRY_DIGITAL_AUTOPILOT.agc`,
  `CM_BODY_ATTITUDE.agc`, `DISPLAY_INTERFACE_ROUTINES.agc`,
  `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`) — via `input/AGC Symbolic Listing.md`
  and the VirtualAGC tree.
- Frank O'Brien, *The Apollo Guidance Computer — Architecture and Operation*,
  Springer/Praxis. Chapter **"Command Module entry"**, book pp. 344–364.
  (Located at `~/Documents/Digital Editions/The Apollo Guidance Computer.pdf`;
  the path recorded in `CLAUDE.md` was stale.)
- `docs/entry_channel_trace.md` (MS-E7c–i diagnostic record).
- `specs/p61_p67-spec.md` (existing entry-program spec).

> **Line-number caveat.** The `.agc` line numbers below are as reported by the
> analyst reading the VirtualAGC Comanche055 tree; treat them as close pointers,
> not byte-exact citations. The O'Brien page numbers are book-printed page
> numbers (PDF page ≈ book page + 15).

---

## 1. Program chain overview: P61 → P62 → P63 → P64 → P65/P66/P67

| Program | MM | Purpose | Entry condition | Exit condition |
|---------|----|---------|-----------------|----------------|
| P61 | 61 | Entry preparation: preliminary nav, IMU check, EMS init data (GMAX, VPRED, GAMMAEI, RTGO, VIO, TTE); crew confirms LAT/LNG/HEADSUP | Crew V37; ~EI−25 min | PRO on V06N63 → falls through to P62 |
| P62 | 62 | CM/SM separation + preentry maneuver: platform check, separation checklist, orient CM to entry attitude, activate entry DAP | Fall-through from P61 (or V37); ~EI−15 min | Two PROs (see §2); AGC auto-advances to P63 when CM within 45° of entry attitude |
| P63 | 63 | Entry initialization / pre-0.05g: init entry equations, hold entry attitude, sense 0.05g; display V06N64 (G/VI/range) | Internal dispatch from P62 (`TC P63` or WAKEP62 task); ~EI−2 min | REENTRY_CONTROL trips 0.05g → P64 |
| P64 | 64 | Post-0.05g closed-loop guidance: constant-drag then HUNTEST L/D steering; display V06N74 | RTB from REENTRY_CONTROL at 0.05g | HUNTEST → P65; short range/low V → P67 |
| P65 | 65 | Up-control (UPCONTRL/SKIPPER): loft trajectory to meet range; display V16N69 | RTB when UPCONTROL solution reached | drag < 0.2g → P66; low V descending → P67 |
| P66 | 66 | Ballistic phase: three-axis attitude hold while out of sensible atmosphere; display V06N22 | RTB when drag < 0.2g | drag rebuilds to 0.2g → P67 |
| P67 | 67 | Final phase: null range error to splashdown; display V16N66/N67; hand off to SCS at ~65 kft | RTB from P64/P65/P66 | steering terminated ~1000 ft/s → GOTOPOOH |

Only P61 and P62 appear in the V37 `PREMM1` dispatch table
(`FRESH_START_AND_RESTART.agc`). P63–P67 are dispatched internally (via `TC P63`,
`WAKEP62`, or RTB from `REENTRY_CONTROL.agc`). Note (O'Brien, footnote, book p.354):
**altitude is not used by any entry program** — it becomes relevant only after
guidance terminates and the parachutes deploy.

---

## 2. P62 DSKY dialogue — the two-PROCEED sequence (the crux for #49)

Both the Comanche055 source **and** O'Brien's operational account agree: **P62
requires two sequential PRO (V33 ENTR) keystrokes**, and P63 begins only after
the CM's preentry attitude maneuver brings it within 45° of the entry attitude.

### O'Brien's operational account (book p.351)

> "P62 performs yet another check of the platform to ensure that it is operating
> and properly aligned, and **generates a program alarm if the IMU is not ready**."
> … "P62 will display **V50N25, Perform Checklist Item, with Register 1 containing
> 00041₈**, to request that the crew begin the CM/SM separation process." … [after
> separation and the automatic heads-down horizon-check maneuver] … "P62 once again
> displays **V06N61** to confirm the splashdown point and the lift orientation …
> Pressing **PRO** accepts the data and advances the display to **V06N22** with the
> gimbal angles for entry. The CM maneuvers to the entry attitude with the lift
> vector pointing down … **When the CM is within 45 degrees of the intended
> attitude, the AGC automatically advances to P63.**"

So the crew-visible sequence is:

1. **V50N25** (R1 = `00041₈`) "Perform Checklist Item" → crew performs CM/SM
   separation → **PRO #1**.
2. Automatic heads-down horizon-check maneuver; **V06N61** re-displayed → crew
   confirms splashdown/lift → **PRO #2** → display advances to **V06N22**.
3. CM maneuvers toward entry attitude; when within 45°, AGC **auto-advances to P63**
   (P63 may already be running via WAKEP62 — see §3.3).

### Source-level branch points

| Checkpoint | File / label | Line |
|------------|--------------|------|
| `P62.2` block start | `P61-P67.agc` `P62.2` | ~204 |
| Separation value `CAF OCT41` (= `00041₈` = V50N25 R1) | `P61-P67.agc` | ~208 |
| `CADR GOPERF1R` (separation display, immediate-return) | `P61-P67.agc` | ~210 |
| `TC +3` — **PRO #1** callback | `P61-P67.agc` | ~212 |
| `TC P61.3` — GOPERF1R immediate return (P62 job then ENDOFJOBs) | `P61-P67.agc` | ~216 |
| `+3:` `TC POSTJUMP / CADR CM/DAPON` | `P61-P67.agc` | ~218–219 |
| `P62.1` label | `P61-P67.agc` | ~225 |
| `CADR GOFLASH` (V06N61 display, **synchronous**) | `P61-P67.agc` | ~233 |
| `TC +2` — **PRO #2** callback | `P61-P67.agc` | ~235 |
| `PHASCHNG OCT 04024`; set ROLLC/ALFACOM/ENTRYVN/P63FLAG | `P61-P67.agc` | ~238 |
| CMDAPMOD gate: `CS CMDAPMOD / MASK ONE / BZF P63.1` | `P61-P67.agc` | ~260–263 |
| `TC P63` (direct dispatch) | `P61-P67.agc` | ~265 |
| `WAKEP62` (NOVAC P63 via WAITLIST) | `P61-P67.agc` | ~274 |
| `CM/DAPON` entry | `CM_ENTRY_DIGITAL_AUTOPILOT.agc` | ~175 |
| `NOTYET` GAMDIFSW wait loop (0.5 s poll) | `CM_ENTRY_DIGITAL_AUTOPILOT.agc` | ~196–202 |
| `POSTJUMP P62.1` | `CM_ENTRY_DIGITAL_AUTOPILOT.agc` | ~237–238 |
| GAMDIFSW set on first CM/POSE (`CMTR1`) | `CM_BODY_ATTITUDE.agc` | ~210–215 |
| `GOPERF1R` subroutine / immediate-return dispatch | `DISPLAY_INTERFACE_ROUTINES.agc` | ~923, 926 |
| `VBPROC` / `RECALTST` (PRO → JOBWAKE) | `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc` | ~2902, 3358 |

### The two PROs are not symmetric

- **PRO #1** wakes the **NVSUB display job** spawned by GOPERF1R (the original P62
  job already terminated at `P61.3 → ENDOFJOB`, because GOPERF1R is *immediate
  return*). VBPROC → RECALTST finds `CADRSTOR ≠ 0`, `XCH CADRSTOR`, `JOBWAKE`; the
  woken job resolves the PRO callback `TC +3` → `POSTJUMP CM/DAPON`.
- Between the PROs, **CM/DAPON** runs the RCS→entry-DAP handover and then **loops
  at 0.5 s** until **GAMDIFSW** is set, before jumping to P62.1.
- **PRO #2** wakes the **P62.1 job itself** (GOFLASH is *synchronous* — the job
  sleeps in ENDIDLE). It resumes at `TC +2`, sets up ROLLC/ALFACOM/P63FLAG, and
  falls through the CMDAPMOD gate to P63.

---

## 3. Supporting mechanisms

### 3.1 GOPERF1R (immediate return) vs GOFLASH (synchronous)

| Property | GOPERF1R (V50N25 sep display) | GOFLASH (V06N61) |
|----------|------------------------------|------------------|
| Return | Immediate to `CADR+4`; caller continues then ENDOFJOBs | Synchronous — caller suspends in ENDIDLE |
| Who holds CADRSTOR | A spawned NVSUB/NOVAC display job | The now-sleeping calling job |
| Wake path | PRO wakes the display job | PRO wakes the caller |

### 3.2 ENDIDLE / CADRSTOR

`ENDIDLE` is the display-wait primitive: it stores the sleeping job's re-entry
CADR in `CADRSTOR` and does JOBSLEEP. A crew key → `VBPROC` → `RECALTST`: if
`CADRSTOR ≠ 0`, clear it (`XCH CADRSTOR`) and `JOBWAKE`; the woken job branches on
LOADSTAT (+1 = PROCEED, −1 = TERMINATE). **`CADRSTOR ≠ 0` is therefore the
observable that a flashing display is parked and waiting for a keystroke.**

### 3.3 WAKEP62 and the 45° gate

`WAKEP62` is a WAITLIST task scheduled by EXDAP (the extra-atmospheric DAP, ~0.1 s
cadence) once (a) `|CALFA| < cos 45°` (CM within 45° of entry trim), (b) CALFA
negative, and (c) `P63FLAG = +1` (P62.1 re-enables it; CM/DAPON had set it −1 to
block early scheduling). This is the mechanism behind O'Brien's "automatically
advances to P63 when within 45°." The **CMDAPMOD gate** at P62.1 then routes:

| CMDAPMOD | Meaning | Dispatch |
|----------|---------|----------|
| +1 / −0 | within 45° / rate-damp-only | `BZF P63.1` → ENDOFJOB (P63 already started by WAKEP62) |
| −1 / +0 | ≥0.05g / outside 45° | `TC P63` (direct start) |

> **Test note.** `entry_state.rs` preloads `CMDAPMOD = -1`, which forces the direct
> `TC P63` branch (skipping WAKEP62). See `entry_state.rs:73–89`.

### 3.4 GAMDIFSW — first CM/POSE gate (depends on AVERAGE-G / SERVICER)

`GAMDIFSW` = `CM/FLAGS` bit 11, initially 0. It is set on the **first execution of
CM/POSE** (`CMTR1`, `CM_BODY_ATTITUDE.agc:~210–215`) after `CM/DAPIC` starts the
attitude cycle. CM/POSE is invoked from the **AVERAGE-G SERVICER** via `AVEGEXIT`
(P62.2 loads `POSECADR` into `AVEGEXIT`). **CM/DAPON cannot exit to P62.1 until the
SERVICER has run at least one AVERAGE-G cycle.** O'Brien (book p.350–351) confirms
P61 starts "a final integration of the state vector … and Average G, the Servicer
routine used to update the state vector during accelerated flight." If AVERAGE-G is
not cycling, GAMDIFSW is never set and CM/DAPON spins forever.

### 3.5 NODOFLAG — V37 lockout

CM/DAPON sets `NODOFLAG` (`FLAGWRD2` bit 1). Thereafter V37 program changes are
refused (alarm 01520) until a fresh start. Crews cannot re-select a program by hand
once the entry DAP is active.

---

## 4. P61 dialogue (summary)

Three flashing displays, then unconditional fall-through to P62
(O'Brien book p.351; `P61-P67.agc`):

| Step | Display | Content |
|------|---------|---------|
| 1 | V06N61 | LAT(SPL) / LNG(SPL) / HEADSUP |
| 2 | V06N60 | GMAX / VPRED / GAMMAEI |
| 3 | V06N63 | RTGO / VIO / TTE — **PRO here terminates P61 and auto-calls P62** |

P61 also (re)starts AVERAGE-G / the Servicer and does the final state-vector
integration and IMU check (O'Brien book p.350–351).

---

## 5. P63–P67 crew view (from O'Brien, book pp.351–358)

- **P63** (EI−2 min): holds entry attitude, displays **V06N64** (accel/velocity/
  range-to-splash); crew waits for the 0.05g light. AGC expects 0.05g near 300 kft.
- **P64** (post-0.05g): DAP drops to pitch/yaw dampers, CM rolls heads-down; **V06N74**
  (roll/velocity/drag). Constant-drag → **HUNTEST** picks the entry profile (target
  within 25 nm), deciding whether P65/P66 are needed before the mandatory P67.
- **P65** (up-control): rolls heads-up to loft and extend range; **V16N69**.
- **P66** (ballistic): three-axis attitude hold out of atmosphere; **V06N22**.
- **P67** (final phase): nulls range error; **V16N66/N67**; hands control to SCS at
  ~65 kft / ~1000 ft/s; PGNS role ends.

---

## 6. Wake-gap diagnostic model (issue #49) — reconciled with current evidence

### 6.1 What the source model predicts (should happen)

`V37 62 ENTR` → P62.2 → S61.1 (platform/IMU + state check) → CM/DAPIC (starts
attitude cycle, sets `AVEGEXIT = CM/POSE`) → **GOPERF1R posts V50N25 (R1=00041₈)**,
which parks a display job in ENDIDLE (`CADRSTOR ≠ 0`). Crew PRO #1 → CM/DAPON →
(GAMDIFSW wait) → P62.1 GOFLASH V06N61 (`CADRSTOR ≠ 0` again). Crew PRO #2 →
CMDAPMOD gate → `TC P63` → `MODREG = 0o077`.

### 6.2 What is actually observed (2026-07, after the recent seeding fixes)

`tc_e7i_f_wake_gap` (`agc-test/tests/entry_p62_diagnostics.rs`) and the committed
`p62_parked_state.json` fixture both show:

| Observable | Expected (parked at V50N25) | Observed |
|------------|-----------------------------|----------|
| `MODREG` | `0o076` | `0o076` ✅ (P62 entered) |
| `CADRSTOR` | ≠ 0 (display parked) | **`0` ❌ — never parks** |
| `settled_in_p62` | true within budget | **false** (12 s budget) |
| `DOTINC/ENTRET` | `TC NVSUBEND` (`0o04216`) | `0o77776` (−1, "unknown") |

Both PROs then have nothing to wake: after PRO #1 and PRO #2, `CADRSTOR` stays `0`,
`MODREG` stays `0o076`, `ROLLC` stays `0`. `POSEXIT` moves `0 → 0o54401` across the
PROs, showing *some* background SERVICER/position-exit activity but no display.

**Conclusion: the stall is UPSTREAM of the two-PROCEED wake logic.** The two-PROCEED
model in §2 is correct, but P62 never reaches the point of posting the first
(V50N25) separation display, so there is no ENDIDLE/`CADRSTOR` for the PROs to act
on. This is a *change* from the Round-2 (MS-E7i) picture, where `CADRSTOR = 0o021523`
was observed at P62.1 GOFLASH. Clearing the earlier `0o01204` WAITLIST alarm (commit
`cd0eac5`) and the REFSMMAT B-1 scaling (`43a3602`) moved the stall — it did not
close the handover.

### 6.3 Confirmed root cause (probe `tc_e7i_g_locate_p62_stall`)

The debugger probe (`agc-test/tests/entry_p62_diagnostics.rs`, breakpoints on the
P62 path + S61.1's alarm exits, driving `V37 ENTR 62 ENTR`) captured this exact
stop sequence, then **no further progress** (the job never reaches P62.2):

```
Breakpoint 1, S61.1   (P61-P67.agc:579)                    ← P62 calls S61.1
Breakpoint 2, R02BOTH (IMU_MODE_SWITCHING_ROUTINES.agc:914) ← IMU/REFSMMAT check
Breakpoint 4, ALARM   (ALARM_AND_ABORT.agc:57)
   #1 RETRN3 (P61-P67.agc:640)  →  TC ALARM / OCT 01426     ← "IMU UNSATISFACTORY"
```

Key points:

1. **R02BOTH now passes.** `VARALARM` was *not* hit — the earlier `0o00210` "IMU
   NOT OPERATING" / `0o00220` "NO REFSMMAT" abort is gone. The REFSMMAT B-1
   half-scale fix (`43a3602`) and permanent-CSM-block seeding (`cd0eac5`) got the
   ISS-initialised gate to succeed. This is progress from the Round-2 picture.

2. **S61.1's own geometric check fails.** `S61.1A` (P61-P67.agc:604-620) computes
   `UNIT(VN ×(REFSMMAT·RN))` and compares `cos(θ)/2` against `C(30)LIM = 1.0 −
   .5·cos 30°` — a ±30° consistency test between the nav state vector (RN/VN) and
   the stable-member axes defined by REFSMMAT. The angle exceeds the limit, so:
   - `RETRN3` → `TC ALARM / OCT 01426` (**IMU UNSATISFACTORY**), or
   - `RETRN2` → `TC ALARM / OCT 01427` (**IMU REVERSED**).
   The probe observed **`0o01426` via RETRN3**.

3. After the alarm, S61.1 does `CAF V05N09 / BANKCALL GODSPR` (puts up the V05N09
   alarm display) and `DELAYJOB` — it **never returns to let P62 fall through to
   P62.2**. Hence GOPERF1R never posts, `CADRSTOR` stays 0, and the two PROCEEDs
   have nothing to wake. `MODREG` remains `0o076` because it was set by NEWMODEX
   before S61.1 ran, and the alarm is a plain `TC ALARM` (posts FAILREG + returns
   into the DELAY/display loop), not a `GOTOPOOH` abort to P00.

**Root cause: the test's identity REFSMMAT (`EntryInitialState::identity_refsmmat`)
is geometrically inconsistent with the direct-LEO state vector (RN/VN). S61.1's
±30° stable-member consistency test therefore fails with `0o01426`, blocking P62
before the separation display.**

### 6.4 Fix direction

S61.1A checks that the trajectory is consistent with the platform orientation. The
preload must supply a **REFSMMAT aligned to the entry trajectory** (the historical
entry REFSMMAT is built from the state vector at Entry Interface), not the identity
matrix, so `UNIT(VN ×(REFSMMAT·RN))` lands within 30° of the stable-member frame.
Options:

- Compute an entry-aligned REFSMMAT in `entry_state.rs` / `EntryInitialState` from
  the same state vector used for RN/VN, replacing `identity_refsmmat()`.
- Or relax the scenario so RN/VN and the identity REFSMMAT are consistent by
  construction (less realistic).

### 6.5 Fix implemented + result (2026-07-02)

`EntryInitialState::entry_refsmmat(position, velocity)` (in `entry_state.rs`) now
builds the REFSMMAT to the flight IMU-alignment axes (`P51-P53.agc:64-70`):
`Y_SM = unit(V × R)`, `Z_SM = unit(−R)`, `X_SM = unit(Y_SM × Z_SM)`. The live test
call sites use it instead of `identity_refsmmat()`. (An intermediate attempt with
`Y_SM = unit(R × V)` / `Z_SM = unit(R)` had both signs flipped and tripped `0o01427`
"IMU REVERSED" — the correct axes matter.)

Verified against yaAGC:

- **`tc_e7i_g_locate_p62_stall`**: stop sequence is now
  **`S61.1 → R02BOTH → P62.2 → GOPERF1R`** with **no `ALARM`/`VARALARM` stop**. The
  S61.1 IMU-consistency alarm is gone; P62 falls through and posts the GOPERF1R
  separation display.
- **`tc_e7i_f_wake_gap`**: Snapshot **B** (after PROCEED #1) now parks with
  `CADRSTOR = 0o21523`, `settled = true` — the P62.1 GOFLASH V06N61 display reaches
  ENDIDLE. Before the fix, nothing parked at all.

Note: Snapshot A does **not** show `CADRSTOR ≠ 0`, because GOPERF1R (V50N25 "PLEASE
PERFORM") is *immediate-return* and does not park via CADRSTOR — the first ENDIDLE
park is P62.1 after PROCEED #1. `tc_e7i_f`'s Snapshot A gate was corrected
accordingly.

### 6.6 Remaining gap (next work package)

`tc_e7i_f` Snapshot **C**: after PROCEED #2, `ROLLC` advances (`0 → 0o77777`, the
P62.1 PROCEED branch runs) but `MODREG` stays `0o076` — **P63 is not yet reached**.
The `PROCEED #2 → CMDAPMOD gate → TC P63` step does not complete under the preload.
This is the next layer of the wake gap (reported, not asserted, in `tc_e7i_f`).
Candidate causes to probe next (same `tc_e7i_g` debugger pattern, breakpoints on
`P62.1`'s PROCEED branch, the CMDAPMOD gate, `WAKEP62`, and `P63`): the CMDAPMOD/45°
gate, `GAMDIFSW`/CM-DAPON state, or the AVERAGE-G SERVICER not cycling under the
preload.

---

## 7. Caveats

1. **O'Brien** was consulted (book pp. 344–364) — this corrects an earlier note in
   the analyst draft that claimed it was unavailable.
2. `.agc` line numbers are analyst-reported pointers; verify against the tree
   before hard-coding any in tests.
3. The test preloads `CMDAPMOD = -1`, forcing the direct `TC P63` branch; real
   flight starts at `+0`/`+1` and transitions as CALFA enters the 45° window.
