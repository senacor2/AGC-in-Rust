# Draft: A coherent IMU/ISS boot state for the yaAGC entry harness

**Status**: DESIGN DRAFT for issue #49, remaining open item **#3** ("No coherent
IMU/ISS + scheduler boot state"). Not yet implemented; §6 lists what still needs a
live yaAGC run to confirm.

**AGC source**: Comanche055 (Apollo 11 CM, rev 055). yaAGC = VirtualAGC
`yaAGC/yaAGC`.

---

## 1. The problem — an incoherent inertial-subsystem state

The entry fixtures preload erasable that says *"the IMU is powered and aligned"*
(`REFSMFLG = 1`, a valid `REFSMMAT`, seeded gimbal angles). But the **simulated
hardware** disagrees:

- yaAGC initialises the IMU discrete word to `InputChannel[030] = 037777`
  (`agc_engine_init.c:255`).
- **Channel 30 is active-low**: a `1` bit means the signal is *absent*. So in the
  default, **bit 9 (IMU OPERATE) = 1 = operate absent** → the hardware says the IMU
  is *not running*.

The erasable ("aligned") and the hardware ("off") contradict each other. Nothing in
the harness drives channel 30, so the AGC's T4RUPT background monitor eventually
acts on the "off" hardware and raises inertial-subsystem alarms. Observed:

- `0o00207` "ISS TURN-ON REQUEST NOT PRESENT FOR 90 SEC" — allowlisted in
  `tc_e7i_j` as a harness artifact.
- `0o00210` "IMU NOT OPERATING" — seen in the Round-2 PRELAUNCH control (`tc_e7i_c`).

We currently paper over these with `--inhibit-alarms` and allowlists. A *coherent*
boot state removes the contradiction at the source, which is prerequisite for any
real Rust↔yaAGC cross-validation (item #1) and likely dissolves items #4
(`0o01204`) and part of #2 (is the guidance even getting a good state?).

## 2. Why erasable-only patching is not enough

The T4RUPT IMU monitor `IMUMON` (`T4RUPT_PROGRAM.agc:291`) **re-samples channel 30
into `IMODES30` on every 480 ms T4RUPT cycle**. So writing `IMODES30`/`IMODES33`
into the core image is *transient* — the next T4RUPT overwrites it from the (still
default, "IMU off") channel 30. The authoritative state is the **channel**, not the
erasable. A coherent boot state must drive **channel 30** (and keep it driven), with
erasable set consistently to avoid a startup transient.

## 3. The alarm conditions to satisfy (source-exact)

| Alarm | Where | Fires when | Coherent state must ensure |
|-------|-------|-----------|----------------------------|
| `0o00210` IMU NOT OPERATING | `IMU_MODE_SWITCHING_ROUTINES.agc` (mode-switch exit, `CS IMODES30 / MASK BIT9 / CCS A`) | `IMODES30` bit 9 = 1 (operate absent) | **ch30 bit 9 = 0** (IMU OPERATE present) |
| `0o00207` ISS TURN-ON NOT PRESENT 90 s | `T4RUPT_PROGRAM.agc:733` (`ITURNON`) | a turn-on sequence was armed (ch30 bit 14 toggled on) then the request went away for 90 s | **no turn-on transition**: ch30 bit 14 held = 1, `IMODES30` bits 7/8/2 = 0 (not mid-sequence) |
| `0o00213` TURN-ON W/O OPERATE | `T4RUPT_PROGRAM.agc` (`PROCTNON`) | ISS turn-on requested but operate absent | not armed if bit 14 steady = 1 and bit 9 = 0 |
| `0o00212` / ISS-warning VARALARMs (`0oX777`) | IMU mode switch, T4RUPT | IMU/ICDU/PIPA FAIL discretes active | keep all fail bits = 1 (inactive) — the default already does |

Net: the AGC accepts "IMU already up and aligned, no turn-on in progress, no
faults" iff channel 30 presents **operate-present, everything-else-nominal, no
edge on the turn-on line.**

## 4. The coherent target state

### 4.1 Channel 30 (the fix that matters)

Start from the yaAGC default `0o37777` and **clear only bit 9** (IMU OPERATE,
`0o400`), presenting operate-present with every other discrete left in its nominal
(inactive, `=1`) state:

```
CH30_COHERENT = 0o37777 & ~0o400  =  0o37377
```

- bit 9  (IMU OPERATE)        = 0  → present  (clears `0o00210`)
- bit 14 (ISS TURN-ON REQUEST)= 1  → absent, no turn-on edge (avoids `0o00207/213`)
- bit 11 (IMU CAGE)           = 1  → not caged
- bit 15 (IMU TEMP)           = 0  → in default; nominal "temp OK"
- IMU/ICDU/PIPA FAIL bits     = 1  → inactive (no fail / ISS-warning VARALARM)

This value must be **written before the first T4RUPT acts** (i.e. at/adjacent to
boot) and **held** for the run so `IMUMON` keeps sampling "operating."

### 4.2 Erasable, set consistently to avoid a startup transient

| Cell | Fresh-start | Coherent value | Rationale |
|------|-------------|----------------|-----------|
| `IMODES30` | `0o37411` | `0o37011` (fresh & ~`0o400`, bit 9 cleared) | pre-agree with ch30 so no transient alarm before the first `IMUMON` sample; turn-on bits 7/8/2 already 0 in fresh-start |
| `IMODES33` | `0o16000` | derive; **at minimum bit 6 = 0** (READGYMB reads CDU counters) — see note | `tc_e7i_j`/`tc_e7e` currently write `0` wholesale; revisit vs the fresh-start value |
| `REFSMFLG` (flag 47D, FLAGWRD3) | 0 | **1** (REFSMMAT good) | already required by the entry preload; confirm `patch_into` sets it |
| `IMUSE` (flag 7D / IMUSEBIT = BIT8) | 0 | **0** for entry | IMU need not be "in use" by a P20-class program; `REFSMFLG` is the alignment gate |

> **IMODES33 note.** We currently zero `IMODES33` in `patch_into`. That is safe for
> the CDU-read path (bit 6 = 0) but is coarser than the fresh-start `0o16000`; a
> follow-up should confirm no IMODES33-derived monitor (bits 11–13, the DSKY/optics
> discretes at `C33TEST`) misbehaves with the wholesale zero. Low risk for entry.

### 4.3 Do NOT trigger the turn-on sequence

The AGC has a legitimate "fresh start with ISS already in OPERATE" path
(`T4RUPT_PROGRAM.agc:339`) that *still* zeroes the ICDUs and runs a 90 s cage. The
coherent state deliberately avoids entering it: present operate-present as a
**steady level from t=0** (no rising edge on bit 9 or bit 14), and have `IMODES30`
already reflect "operating," so `ITURNON`/`TNONTEST` see no change to process.

## 5. Injection mechanism in the harness

- Channel 30 is an **input channel** (hardware→AGC). yaAGC accepts writes to it over
  the peripheral socket. Add a one-shot (then held) write in the harness setup:
  a `ChannelPacket { channel: 0o30, value: 0o37377, u_bit: false }` through a
  `YaAgcClient` (no `COUNTER_FLAG` — this is a channel level-write, not a counter
  increment). Reuse an existing connection or a dedicated one.
- Sequence: spawn yaAGC → **immediately** write ch30 = `0o37377` (before/at the
  first T4RUPT) → then proceed with the existing DSKY/PIPA/CDU flow. Optionally
  re-assert it periodically (cheap) to guarantee it stays latched.
- Keep the erasable `IMODES30 = 0o37011` / `REFSMFLG = 1` patches in `patch_into` so
  the pre-first-sample window is also coherent.

Suggested home: a `patch_imu_operating(core, symtab)` erasable helper in
`entry_state.rs` plus a `set_imu_operating(&mut client)` channel-write in
`vagc_driver.rs`, called by both `tc_e7i_j` and `run_live_scenario_closed_loop`.

## 6. Open items — need a live yaAGC run to confirm

1. **Channel-30 write path.** Confirm a `ChannelPacket` to channel `0o30` actually
   lands in `InputChannel[030]` (vs. needing the `u_bit`/uninterpreted flag or a
   different opcode in the socket framing). Verify by dumping `IMODES30` after one
   T4RUPT and checking bit 9 = 0.
2. **Exact `0o37377` vs. a fuller value.** Bits 15 (temp) and the fail bits are
   assumed nominal from the default; confirm no caution/ISS-warning fires over a
   long run. Adjust if the temp-lamp or fail logic expects a different level.
3. **`IMODES33` coherent value** — whether wholesale `0` is fine or the fresh-start
   `0o16000` (with bit 6 cleared → `0o16000`, already bit-6-clear) is safer.
4. **Turn-on transient.** Confirm presenting operate-present as a steady level truly
   skips the ICDU-zero / 90 s cage (no `IMODES30` bit 7/8 churn in dumps).
5. **Scheduler coupling (item #4).** Re-check whether `0o01204` (WAITLIST) still
   appears once the IMU state is coherent and `--inhibit-alarms` is *removed* — this
   draft targets IMU/ISS; the WAITLIST alarm may be independent or may clear.

## 7. Success criteria

- Over a full entry run **without `--inhibit-alarms`**, `FAILREG` shows **no**
  `0o00207` / `0o00210` / `0o00213` / ISS-warning VARALARM (the `0o00207` allowlist
  in `tc_e7i_j` can then be *removed*, not just tolerated).
- Core dumps show `IMODES30` bit 9 = 0 (operating) throughout.
- `tc_e7i_j` and the `tc_e7e` closed-loop tests still reach P63 / P64 (no
  regression), now on a coherent — rather than alarm-suppressed — inertial state.

## 8. Implementation attempt — finding (2026-07-03)

The §4/§5 recipe was implemented and run against live yaAGC. **It does not work as
drafted — the naive channel-30 write is counterproductive** — and was reverted. The
failure is instructive and de-risks the next attempt.

**What was tried:** `set_imu_operating` writing `ch30 = 0o37377` right after connect,
plus `IMODES30 = 0o37011` in `patch_into`.

**Result:** a *new* alarm `0o00220` "IMU NOT ALIGNED — NO REFSMMAT" (R02BOTH,
`IMU_MODE_SWITCHING_ROUTINES.agc:914`), and `tc_e7i_j` never reached P63 (stuck at
P62, `FAILREG = [0o01107, 0o00220]`). Strictly **worse** than the original state.

**Root cause — the missing interaction with `REFSMFLG`:**
`patch_into` already preloads `REFSMFLG = 1` (FLAGWRD3 bit 13). In the *original*
harness the IMU reads "off" at the hardware (`ch30` default bit 9 = 1) but
`REFSMFLG` is never disturbed, so R02BOTH (`MASK STATE+3` = REFSMFLG → set →
`R02ZERO`, no alarm) is happy; the only residual is the benign `0o00207`.

The post-boot `ch30` write flips IMU OPERATE (bit 9: 1→0) — an **edge**. The AGC's
IMU-monitor logic processes that edge as "the IMU just came on," which disturbs the
preloaded alignment: by the time R02BOTH runs, `REFSMFLG` reads **clear**, so
R02BOTH takes the `REFSMFLG clear + operating` branch → `0o00220`.

**Key insight (reframes the goal):** the original accommodation — IMU nominally off
at the hardware, `REFSMFLG` preloaded and untouched, `0o00207` allowlisted — is
actually **more coherent for R02BOTH** than driving `ch30`. `0o00207` is a benign,
non-aborting turn-on-monitor artifact; trading it for `0o00220` (which blocks P63)
is a regression.

**What a real fix requires (larger than a single write):** an *edge-free* operating
state, so no IMU-turn-on transition is ever detected:
1. **`ch30` must read "operating" from yaAGC's very first cycle** — before any
   `IMUMON` sample — so there is no 1→0 edge. yaAGC hard-codes
   `InputChannel[030] = 037777` in `agc_engine_init.c:255`; getting `0o37377` in
   from t=0 means either a yaAGC-side change (reference source — out of scope) or a
   write that provably lands before the first T4RUPT (a tight, racy ~480 ms window —
   unverified it's achievable over the socket). **or**
2. **Re-assert `REFSMFLG = 1` after the turn-on sequence settles** — but the harness
   only patches erasable at boot; mid-run erasable writes need a mechanism that does
   not exist yet.

**Recommendation:** treat the `0o00207` allowlist as the pragmatic "coherent-enough"
accommodation for now (it keeps `REFSMFLG` valid, which is what R02BOTH and the
guidance actually depend on). Pursue the edge-free operating state only as a
separately scoped task with live-debugger iteration — and note it may be blocked by
yaAGC's fixed channel-30 initialisation, i.e. it could require a yaAGC-side change
rather than a harness-side one.
