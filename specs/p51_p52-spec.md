# Specification: `programs/p51_p52` Module — IMU Alignment Programs

**Status**: Approved for implementation
**Module path**: `agc-core/src/programs/p51_p52.rs`
**Architecture reference**: `docs/architecture.md` §7.2 "Programs for the Command Module"
**IMU control reference**: `specs/imu-control-spec.md` §3 (REFSMMAT construction), §4 (alignment state machine)
**Types reference**: `specs/types-module-spec.md` §3.1 (`CduAngle`), §3.4 (`Vec3`, `Mat3x3`)
**AGC source files**:
- `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — P51/P52 labels, coarse/fine align loops
- `Comanche055/R51.agc` — IMU orientation determination routine
- `Comanche055/R52.agc` — IMU realignment routine

---

## 1. Purpose and Scope

`programs::p51_p52` implements the two IMU alignment programs used by the
CMC to establish and maintain the orientation of the inertial platform
(stable member) with respect to an inertial reference frame:

- **P51 — IMU Orientation Determination**: runs when the platform is
  caged or has an invalid REFSMMAT. Takes two star sightings, computes a
  new REFSMMAT via the TRIAD method, commits it to `state.refsmmat`, and
  transitions `imu_alignment_state` from `Caged` → `CoarseAligned`.

- **P52 — IMU Realignment**: runs when the platform already has a valid
  fine alignment but has accumulated gyro drift since the last alignment.
  Takes two new star sightings, computes a corrected REFSMMAT, commits it,
  and transitions `imu_alignment_state` → `FineAligned`.

Both programs are thin sequencing layers over the pure functions in
`control::imu_control`. They do not implement the T4RUPT drift-compensation
loop (owned by the T4RUPT ISR shim) nor the interactive optics MARK
acquisition (owned by the V/N processor — Milestone 5).

### What this module provides

- `P51_MAJOR_MODE: u8 = 51`, `P52_MAJOR_MODE: u8 = 52`.
- `PRIORITY: JobPriority = 8` — same background tier as other non-critical
  targeting programs.
- `init_p51`, `init_p52` — entry points registered in `PROGRAM_TABLE`.
- `p51_mark_align`, `p52_mark_align` — accept two star-catalog inertial
  vectors and two measured platform-frame vectors; compute REFSMMAT and
  commit it.

### What this module does NOT provide

- Interactive MARK button polling. The crew-interactive star-sighting loop
  (V01 N71/N72, optics drive, MARK acceptance) is a Milestone 5 item. For
  now, test harnesses call `pXX_mark_align` directly with pre-computed
  inertial and platform vectors.
- T4RUPT gyro drift torquing. That lives in the T4RUPT ISR shim and
  operates independently of the active program.
- Automatic coarse-align CDU drive loop. P51's true behaviour drives the
  CDUs to a target orientation via `coarse_align_step` until converged; we
  assume the stable member is already within the coarse-align envelope when
  `p51_mark_align` is called (i.e., the crew has slewed it manually via
  the optics).

---

## 2. Program Alarms

| Code | Trigger                                        | Behaviour                              |
|------|------------------------------------------------|----------------------------------------|
| 220  | Star vectors collinear (TRIAD returns `None`). | REFSMMAT unchanged, alignment state unchanged. |

---

## 3. Functional Requirements

### 3.1 `init_p51`

On entry:

1. Sets `state.major_mode = 51`.
2. Sets `state.dsky.prog = 51`.
3. Sets `state.dsky.verb = 6` (display).
4. Sets `state.dsky.noun = 70` (Star code / sextant angles — data entry cue).
5. Sets `state.dsky.flashing = true` (awaiting crew MARK).

Does NOT modify `state.refsmmat` or `state.imu_alignment_state` — those
are only touched by a successful `p51_mark_align` call. Returns `PRIORITY`.

### 3.2 `init_p52`

Identical to `init_p51` except:
- `state.major_mode = 52`
- `state.dsky.prog = 52`

Additionally: must only be called when `state.imu_alignment_state` is
`CoarseAligned` or `FineAligned`. If it is `Caged`, sets program alarm
code `221` (platform caged) and does not advance state. (P52 requires an
existing reference to refine; P51 must run first from a caged start.)

### 3.3 `p51_mark_align(state, s1_inertial, s2_inertial, s1_platform, s2_platform)`

1. Call `refsmmat_from_star_sightings(s1_inertial, s2_inertial, s1_platform, s2_platform)`.
2. On `Some(m)`:
   - `state.refsmmat = m`
   - `state.imu_alignment_state = ImuAlignmentState::CoarseAligned`
   - Clear `state.dsky.flashing = false`
   - Display new REFSMMAT determinant confirmation on DSKY (N93 — V06N93
     shows the three star angles; simplified here to setting verb/noun to
     6/93 with `r[0]` = 1.0 as success flag).
3. On `None` (collinear stars):
   - `state.alarm.code = 220`
   - `state.alarm.lit = true`
   - Leave `state.refsmmat` and `state.imu_alignment_state` unchanged.

### 3.4 `p52_mark_align(state, s1_inertial, s2_inertial, s1_platform, s2_platform)`

Identical to `p51_mark_align` except that the successful path sets
`state.imu_alignment_state = ImuAlignmentState::FineAligned` (not
CoarseAligned), reflecting the spec assumption that P52 is a refinement
of an already-coarse-aligned platform.

---

## 4. Test Cases

### TC-P51-1: `init_p51` sets major_mode = 51 and prog = 51.
Construct fresh `AgcState`; call `init_p51`; assert `major_mode == 51`,
`dsky.prog == 51`, `dsky.flashing == true`, return value equals `PRIORITY`.

### TC-P51-2: `p51_mark_align` with orthogonal unit stars produces identity REFSMMAT.
Pass `[1,0,0]/[0,1,0]` as both inertial and platform pairs; assert
`state.refsmmat` equals identity to 1e-12; assert
`imu_alignment_state == CoarseAligned`.

### TC-P51-3: `p51_mark_align` with collinear stars sets alarm 220.
Pass `[1,0,0]/[1,0,0]`; assert `state.alarm.code == 220`,
`state.alarm.lit == true`, and `state.imu_alignment_state` is unchanged
from its pre-call value.

### TC-P51-4: `p51_mark_align` transitions Caged → CoarseAligned.
Set `state.imu_alignment_state = Caged`; run a successful `p51_mark_align`;
assert transition.

### TC-P52-1: `init_p52` sets major_mode = 52 from CoarseAligned state.
Set `imu_alignment_state = CoarseAligned`; call `init_p52`; assert
`major_mode == 52`, `dsky.prog == 52`, no alarm raised.

### TC-P52-2: `init_p52` from Caged state raises alarm 221.
Set `imu_alignment_state = Caged`; call `init_p52`; assert
`state.alarm.code == 221`.

### TC-P52-3: `p52_mark_align` transitions CoarseAligned → FineAligned.
Start from `CoarseAligned`; successful `p52_mark_align`; assert
`imu_alignment_state == FineAligned`.

### TC-P52-4: `p52_mark_align` preserves REFSMMAT on collinear star error.
Pre-seed a known REFSMMAT; call `p52_mark_align` with identical stars;
assert REFSMMAT bytes are unchanged and alarm 220 is set.

---

## 5. Out of Scope (Milestone 5 or later)

- Interactive MARK button polling and the V01 N71/N72 crew loop.
- `coarse_align_step` CDU drive integration — for now, assume the crew
  slews the platform manually before calling `pXX_mark_align`.
- `fine_align_torque` nulling iteration — the T4RUPT handler does this
  continuously once REFSMMAT is set.
- Star-catalog lookup by octal star code (N71 data entry).
- Optics sextant drive (`Optics::drive`) — Milestone 5 + HAL integration.
