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

### 3.3 WAKEP62 and the 45° entry-attitude window

> **Terminology.** The "45°" here is the **angle-of-attack window** (the CM heat
> shield within 45° of the relative wind), *not* the flight-path-angle entry
> corridor. The gate is on attitude (`CALFA = cos α`), decided by the entry DAP —
> independent of the trajectory corridor P61 checks.

`WAKEP62` is a WAITLIST task scheduled by EXDAP (the extra-atmospheric DAP, ~0.1 s
cadence). **Validated against `CM_ENTRY_DIGITAL_AUTOPILOT.agc` EXDAP (lines
602–624) by the analyst-reengineer and orbital-mechanics agents, 2026-07-02 — the
earlier draft of this section had the sign inverted.** EXDAP arms it **exactly
once**, when all three hold:

- **(a) `|CALFA| ≥ cos 45°` (≈ 0.707)** — the CM is *within 45° of zero angle of
  attack*. Mechanism: `CCS CALFA / AD C45LIM / TS A`, where `C45LIM = 1 − cos45° =
  0.29289`; the add overflows (skipping `TCF EXDAP2`) exactly when `|CALFA| >
  cos45°`. `|CALFA| < cos45°` instead falls to **EXDAP2** → `CMDAPMOD = +1`
  (broadside / rate-damp).
- **(b) `CALFA > 0` (positive)** — heat-shield-forward half. `CCS CALFA / TCF +1`
  continues only for positive CALFA; **negative CALFA branches to `TC EXDAP4`** and
  never schedules.
- **(c) `P63FLAG = ±0`** — the `CCS P63FLAG` single-pass guard (`+1` or `−1` →
  EXDAP4). CM/DAPON preloads `P63FLAG = −1` to block early scheduling; it must be
  cleared to `±0` first. On arming, EXDAP sets `P63FLAG = −1` (`CS ONE / TS
  P63FLAG`) so WAKEP62 fires only once.

Delay `NSEC = 2100 cs = 21 s` ("65°/3°·s⁻¹ = transit time from AoA 45° to trim").
This is the mechanism behind O'Brien's "automatically advances to P63 when within
45°": at the entry trim AoA ≈ −20°, `CALFA = cos(−20°) ≈ +0.94` — positive and
`> cos45°`, so all three conditions are met. **P63 is started only by the WAKEP62
task (`NOVAC 2CADR P63`, `P61-P67.agc:275–277`) on this path** — there is no
synchronous `TC P63` from EXDAP. The **CMDAPMOD gate** at P62.1 then routes:

| CMDAPMOD | Meaning | Dispatch |
|----------|---------|----------|
| +1 | broadside, `\|CALFA\| < cos45°` | `BZF P63.1` → ENDOFJOB (defer to WAKEP62) |
| −0 | `\|CALFA\| ≥ cos45°`, **CALFA < 0** (nose-into-wind) | `BZF P63.1` → ENDOFJOB (defer to WAKEP62) |
| +0 | `\|CALFA\| ≥ cos45°`, **CALFA > 0** (heat-shield-forward) | `TC P63` (direct start) |
| −1 | preload / ≥0.05g | `TC P63` (direct start) |

> The `BZF P63.1` branch only *defers* to WAKEP62; WAKEP62 must actually have been
> (or later be) scheduled by EXDAP for P63 to start. Under the `-0` stall (CALFA
> negative) EXDAP never schedules it — the §6.6 deadlock. When the loop drives CALFA
> positive, EXDAP schedules WAKEP62 and, once `|CALFA| ≥ cos45°` with CALFA > 0
> persists, the direct `+0 → TC P63` path can also fire.

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

### 6.6 PROCEED#2 → P63 gap — root cause (2026-07-02)

Probe `tc_e7i_i_v33_dispatch` (non-debugger; PROCEEDs paced on the actual parked
display, capturing the discriminating erasable state) root-caused this. The
debugger approach was abandoned: breakpoints on the keyboard path (`VBPROC`/
`RECALTST`) halt the sim and break the DSKY socket, and under the debugger the sim
is too slow to reach the ~20–30 s GAMDIFSW delay in reasonable wall-clock.

**Two factors, one a red herring:**

1. **Pacing (partial factor).** The `V50N25` (GOPERF1R separation) → `V06N61`
   (P62.1 prompt) step is gated on **GAMDIFSW** (`CM/FLAGS` bit 11), which CM/DAPON
   waits for and the AVERAGE-G SERVICER sets only ~20–30 s in (`AVEGFLAG` is on the
   whole time). `tc_e7i_f` sent its second PROCEED before that — prematurely. With
   display-paced PROCEEDs, P62 does advance `V50N25 → V06N61`.

2. **Root cause — the P62→P63 handover is closed-loop on the entry-attitude
   maneuver.** With correct pacing, the PROCEED at `V06N61` **does** run the P62.1
   `+2` branch (`P63FLAG = +1` after it; CM/DAPON leaves it `-1`). But it then takes
   the wrong gate arm:
   - `CMDAPMOD` is preloaded `-1` (`0o77776`) — the value that makes
     `CS CMDAPMOD / MASK ONE / BZF P63.1` fall through to `TC P63`. **EXDAP
     overwrites it** every cycle from the body attitude
     (`CM_ENTRY_DIGITAL_AUTOPILOT.agc:579-602`): `|CALFA| ≤ 45° ⇒ +1`; CALFA
     positive/outside `⇒ +0`; **CALFA negative/outside `⇒ -0`** (`0o77777`, "rate
     damp only"). Observed at the gate: `CMDAPMOD = 0o77777` (−0).
   - With `CMDAPMOD = -0`, the gate takes **`BZF P63.1`** — which just
     `PHASCHNG / ENDOFJOB`, expecting `WAKEP62` to have started P63.
   - `WAKEP62` is scheduled by EXDAP only when **`CALFA > +cos45°`** (positive —
     heat shield within 45° of the relative wind) with `P63FLAG = ±0` (see §3.3 for
     the validated conditions; the observed `CMDAPMOD = -0` is precisely the
     `CALFA < 0`, nose-into-wind case that branches to `TC EXDAP4` and never
     schedules). The open-loop harness drives PIPAs (accelerometers) but **not the
     attitude/CDU loop**, so the CM never maneuvers heat-shield-forward, `CALFA`
     stays negative, `WAKEP62` is never scheduled, and P63 never starts. This is the
     deadlock: the P62.1 gate routes `CMDAPMOD = -0 → BZF P63.1` (which waits for
     WAKEP62), while EXDAP refuses to schedule WAKEP62 until `CALFA` goes positive.

**`DSPLOCK = 0` throughout** — V33 is not blocked; the MS-E7h "V33 ignored at the
P62.1 park" hypothesis is disproven. (The `CADRSTOR = 0o21523` seen at both parks is
the common GOFLASH/ENDIDLE re-entry, not a stale-wake bug.)

**Conclusion.** The final P62→P63 step cannot close under a purely open-loop preload
harness. It requires simulating the CM entry-attitude maneuver — entry-DAP thruster
commands → CDU/IMU attitude feedback → `CALFA` within 45° → `WAKEP62` → P63. This is
exactly the trajectory-level closed-loop validation issue #49 tracks as deferred:
the per-routine fixtures do not need it, and closing it means standing up the
attitude-control loop (or injecting CDU angles that walk CALFA into the 45° window).

---

## 7. Caveats

1. **O'Brien** was consulted (book pp. 344–364) — this corrects an earlier note in
   the analyst draft that claimed it was unavailable.
2. `.agc` line numbers are analyst-reported pointers; verify against the tree
   before hard-coding any in tests.
3. The test preloads `CMDAPMOD = -1`, forcing the direct `TC P63` branch; real
   flight starts at `+0`/`+1` and transitions as CALFA enters the 45° window.

---

## 8. Developer-ready spec: closing P62→P63 with a simulated attitude loop

**Validated 2026-07-02** by three agents against Comanche055 + yaAGC:
analyst-reengineer (EXDAP gate logic, §3.3), orbital-mechanics (physics / gimbal
recipe / dynamics), virtualagc-debugger (CDU injection mechanism). This section is
the implementation contract for the developer.

### 8.1 Objective

Drive the simulated CM attitude so that `CALFA` (cos of angle of attack, computed by
CM/POSE from the CDU gimbal angles + the state-vector velocity triad) rises through
`+cos45°` and settles toward the entry trim (AoA ≈ −20°, `CALFA ≈ +0.94`). This
satisfies the §3.3 gate, EXDAP schedules `WAKEP62`, and P63 starts ~21 s later.
The loop must supply **CDU gimbal angles**, not PIPA counts — PIPAs feed AVERAGE-G,
not the attitude gate.

### 8.2 Target body attitude → CDU gimbal-angle recipe (orbital-mechanics)

Desired: heat-shield X-body axis into the relative wind, at the −20° trim. Given the
desired body axes expressed in stable-member (SM) coordinates — where the entry
REFSMMAT is `Y_SM = unit(V×R)`, `Z_SM = unit(−R)`, `X_SM = unit(Y_SM×Z_SM)`
(matches `entry_state.rs::entry_refsmmat`, `P51-P53.agc:64-70`) — the AGC gimbal
angles are the inverse of READGYMB (`X`=outer/OG, `Y`=inner/IG, `Z`=middle/MG):

```
CDUZ (middle, AMG) = arcsin( XB · Y_SM )              # = arcsin(XB_y)
CDUY (inner, AIG)  = atan2( −(XB · Z_SM), XB · X_SM ) # = atan2(−XB_z, XB_x)
CDUX (outer, AOG)  = atan2( −(ZB · Y_SM), YB · Y_SM ) # = atan2(−ZB_y, YB_y)
```

where `XB, YB, ZB` are the desired CM body axes in SM coords. Watch gimbal-lock near
`CDUZ = ±90°` (middle-gimbal); the entry attitude is nowhere near it, but clamp.

### 8.3 Attitude dynamics fidelity (orbital-mechanics)

For the **P62→P63 window only** (pre-0.05g), model:

- **Pure rigid-body RCS** — thruster torque → angular acceleration. **No
  aerodynamics** (below 0.05 g there is no sensible aero moment); the DAP is the
  extra-atmospheric mode.
- Roll authority `A = 9.1°/s²` (`A1 = 4.55`, `CM_ENTRY_DIGITAL_AUTOPILOT.agc`
  roll-DAP constants), `VM = 20`. Pitch/yaw comparable RCS authority.
- **Rate-continuous CDU updates are mandatory.** The DAP derives body rates from
  successive CDU *differences*. A static/step CDU write produces a one-cycle
  spurious rate spike that the DAP fights. The loop must **ramp** CDU angles at a
  physically plausible slew (≤ a few °/s), matching the ~65°-in-21 s nominal
  maneuver (≈3°/s), so `PREL/QREL/RREL` stay bounded.

### 8.4 CDU pulse-injection mechanism (virtualagc-debugger)

The existing `YaAgcClient` (`agc-test/src/vagc_channel.rs`) needs **no changes**.
Inject via the counter/unprogrammed-increment path — build a `CduInjector` in
`vagc_driver.rs` mirroring `PipaInjector`:

| Gimbal | Counter channel | Wire address (`0x80 \| addr`) |
|--------|-----------------|-------------------------------|
| CDUX (outer) | 0o32 | `0x9A` |
| CDUY (inner) | 0o33 | `0x9B` |
| CDUZ (middle) | 0o34 | `0x9C` |

- Packet `value` = **IncType**, not a count: `1` = `+PCDU` (slow, +1 LSB),
  `3` = `−MCDU` (slow, −1 LSB). **Avoid** the fast types `17`/`19` (they pulse at a
  different rate and complicate calibration).
- **LSB = 360° / 32768 ≈ 0.010986° ≈ 39.56 arcsec** → ~91 counts per degree. Emit
  N=round(Δ°·91) increments per update tick, sign via PCDU/MCDU.
- FIFO runs at **400 counts/s** and holds **128 sign-change entries**; keep each
  tick's burst under that and pace ticks so the FIFO drains (see §8.6).
- **Patches required in `patch_into` (`entry_state.rs`)** so CDU reads are
  deterministic: write **`IMODES33 = 0`** (clear bit 6 so `READGYMB` reads the CDU
  counters, not coarse-align) and **`IMODES30 = 0`** (channel-30 CDU-fail / mode
  bits clear). Neither is currently patched.

### 8.5 Control-loop structure

Per tick (align to the DAP's ~0.1 s / the harness's PIPA cadence):

1. Read current CDU angles (or track them internally from cumulative increments).
2. Compute the target attitude for this tick along a ramp from the post-separation
   attitude to the entry trim (§8.2 recipe, interpolated ≈3°/s).
3. Emit PCDU/MCDU bursts (§8.4) to move each CDU toward its target-for-this-tick.
4. Continue driving PIPAs as today (AVERAGE-G must keep cycling so GAMDIFSW sets and
   CM/POSE runs).
5. Let CM/POSE recompute `CALFA`; once `CALFA > +cos45°` positive, EXDAP arms
   WAKEP62; P63 starts ~21 s later.

### 8.6 Open items — RESOLVED (live yaAGC run, 2026-07-03, `tc_e7i_j`)

All three were verified **not** to be the blocker; the real root cause was an
inverted target attitude:

1. **`IMODES33` bit 6** — already clear in the template core; the `IMODES33 = 0`
   patch is harmless but not the cause. READGYMB reads the injected CDU counters.
2. **FIFO-drain vs `READGYMB` cadence** — the ramp injects ≈27 counts/tick (0.3° at
   3°/s / 100 ms), far below the 400 cps FIFO limit; no lag, not blocking.
3. **AOG/AIG/AMG bank switching** — no aliasing; the routines set `EBANK=AOG`
   correctly before each READGYMB.

**Actual root cause:** `entry_trim_cdu_deg` targeted `X_body = +unit(velocity)`
(nose-into-wind → `CALFA ≈ −1`, the permanent `CMDAPMOD = −0` stall). Entry is
**heat-shield-forward**, so the CM flies backward: `X_body = −unit(velocity)`
(CDUY ≈ 174° for the direct-LEO FPA, not −6°). With that correction the ramp
carries CALFA through broadside to `+cos45°` positive, EXDAP arms WAKEP62, and
P63 starts ~21 s later. Also required in the harness: a **Phase 3 poll** of
`MODREG` after the two PROCEEDs (WAKEP62 → P63 is an autonomous transition, not a
parked display), and treating alarm **`0o00207`** ("ISS TURN-ON REQUEST NOT
PRESENT FOR 90 SEC", a preload-harness artifact) as benign.

### 8.7 Acceptance criteria — MET by `tc_e7i_j_closed_loop_p63`

- With the closed-loop `CduInjector` active, the system test drives P62 through both
  PROCEEDs and reaches **`MODREG = 0o077` (P63) via WAKEP62** with no unexpected
  program alarm. ✅ (stable across repeated runs, ~69 s each)
- **`MODREG = P63` is the CALFA-gate proof**: WAKEP62 is armed only when CALFA is
  positive and `≥ +cos45°` (§3.3), so reaching P63 establishes it. The assertion
  uses the stable **`CMDAPMOD = +0`** (heat-shield-forward) EXDAP output; the raw
  `CALFA` cell is diagnostic-only (it aliases SPNDX/INTTEMP scratch — dumped value
  unreliable, observed `0o37727` / `0o00000` across runs).
- The existing open-loop fixtures and REFSMMAT tests remain green (no regression).
