# Specification: `programs/p01_p02` Module — Pre-launch IMU Initialisation (P01) and Gyrocompassing (P02)

**Status**: Approved for implementation
**Module path**: `agc-core/src/programs/p01_p02.rs`
**Architecture reference**: `docs/architecture.md` §7.2 "Programs for the Command Module"
**IMU-control reference**: `specs/imu-control-spec.md` — `ImuAlignmentState`, `COARSE_ALIGN_THRESHOLD`
**Executive reference**: `specs/executive-spec.md` §2.2 (Waitlist self-rescheduling pattern)
**Time reference**: `agc-core/src/navigation/time.rs` — `OMEGA_EARTH = 7.292_115_085_5e-5 rad/s`
**O'Brien reference**: §12 "IMU Alignment" — gyrocompassing on the launch pad
**AGC source files**:
- `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — SLEEPIE / ALWAYSG / EARTHR* gyro-torque loop

---

## 1. Purpose and Scope

`programs::p01_p02` implements the two pre-launch IMU programs:

- **P01 — Pre-launch IMU Initialisation.** Cages the inertial platform
  to its mechanical mount, clearing any prior alignment. Entered before
  liftoff so the subsequent gyrocompass (P02) can begin from a known
  starting state.
- **P02 — Gyrocompassing.** Runs a repeating Waitlist loop that
  iteratively torques the IMU gimbals toward local-vertical / local-North
  alignment by integrating the Earth-rate vector. Terminates when all
  three CDU axes fall within `COARSE_ALIGN_THRESHOLD` and transitions
  `imu_alignment_state` to `CoarseAligned`.

These are book-keeping programs. They command no actuators except the
IMU torque drives (modelled by directly driving the CDU angles in the
Rust port — see §2.3), perform no navigation maths beyond the
Earth-rate components, and have no servicer / DAP interactions.

After liftoff, P11 computes the proper `REFSMMAT` from gravity and the
launch azimuth. **P02 itself does not touch `REFSMMAT`.**

### What this module provides

- `P01_MAJOR_MODE: u8 = 1`, `P02_MAJOR_MODE: u8 = 2`.
- `PRIORITY: JobPriority = 3` — pre-launch tier (both programs).
- `GYROCOMPASS_PERIOD_CS: u16 = 500` — Waitlist period (5 s).
- `GYROCOMPASS_DRIVE_COUNTS: i16 = 330` — per-step drive (≈ 100× real
  Earth-rate; see §2.3 for the simulation-speed rationale).
- `ALARM_GYROCOMPASS_WRONG_STATE: u16 = 235`.
- `init_p01(state)` — entry point registered in `PROGRAM_TABLE[1]`.
- `init_p02(state)` — entry point registered in `PROGRAM_TABLE[2]`.
- `p02_gyrocompass_step(state)` — one iteration of the convergence loop;
  scheduled by `init_p02` and re-schedules itself until convergence.
- `earth_rate_horizontal(lat_rad)` / `earth_rate_vertical(lat_rad)` —
  pure helpers for tests and the `SimEarthRate` simulator hook
  (`Ω_E · cos(lat)` and `Ω_E · sin(lat)` respectively).

### What this module does NOT provide

- REFSMMAT computation. That belongs to P11 (post-liftoff) and to P51
  (in-flight realignment).
- Coarse-alignment-to-fine-alignment refinement. P02 leaves the platform
  in `CoarseAligned`; fine alignment is the job of P51 / P52 with the
  optics.
- Direct gyro torque pulse counting. The historical PULSEIMU pipeline is
  abstracted by writing CDU angles directly (`state.current_cdu[i]`),
  which is what the SLEEPIE loop ultimately accomplishes on the AGC.

---

## 2. AGC Background

### 2.1 P01 in Comanche055

In the original assembly the P01 entry block cages the platform: it
sets the IMU mode discrete that mechanically locks the gimbals to the
mount, clears any in-progress alignment, and ENDs OF JOB. After ENDOFJOB
the CMC sits idle waiting for the crew to V37 into P02.

### 2.2 P02 in Comanche055

P02 runs the SLEEPIE / ALWAYSG / EARTHR* gyrocompass loop. Every 0.5 s
(`1SECXT1 = .5SEC = 50 cs`) the AGC calls PULSEIMU with a torque pulse
derived from the Earth-rate vector ERTHRVSE projected onto the platform
axes via the current direction cosines. Convergence is achieved when
the misalignment between the stable member and (local-vertical,
local-North, local-East) falls below the IMU's mechanical resolution.

### 2.3 Simulation model in the Rust port

The Rust port does not model PULSEIMU. Instead, `p02_gyrocompass_step`
drives `state.current_cdu[0..3]` (the IMU outer/inner/middle gimbal
angles in CDU counts) directly toward zero. Convergence is reached when
all three axes fall within `COARSE_ALIGN_THRESHOLD` (see
`specs/imu-control-spec.md` §3).

Per-axis drive at each iteration:

| Axis | Drive value (counts) | Earth-rate component |
|---|---|---|
| X (roll, azimuth error) | `GYROCOMPASS_DRIVE_COUNTS · cos(lat) · 0.5` | (small) |
| Y (pitch, horizontal-North) | `GYROCOMPASS_DRIVE_COUNTS · cos(lat)` | `Ω_E · cos(lat)` |
| Z (yaw, vertical-Up) | `GYROCOMPASS_DRIVE_COUNTS · sin(lat)` | `Ω_E · sin(lat)` |

`GYROCOMPASS_DRIVE_COUNTS = 330` is the per-step drive scaled 100× over
the physical Earth-rate horizontal contribution
(`Ω_E · cos(28.6° KSC) ≈ 6.41 × 10⁻⁵ rad/s ⇒ 3.3 counts at 5 s/step`).
The scale-up keeps the convergence window short enough for unit tests
(~82 steps from a 30° misalignment instead of the ~1636 steps a faithful
real-time loop would take). On real hardware this constant would be 1×
real rate.

### 2.4 Why CDU = 0 means aligned

The CDU angle represents the *displacement* between the current
platform orientation and the desired alignment. When all three CDUs
read zero, the stable member is aligned to local vertical + North + Up
within the resolution of the COARSE_ALIGN_THRESHOLD.

---

## 3. Program Alarms

| Code | Trigger |
|---|---|
| 235 (`ALARM_GYROCOMPASS_WRONG_STATE`) | `init_p02` invoked while `imu_alignment_state != Caged`. Soft alarm — P02 still starts so the crew can observe the transition. |

P01 raises no alarms.

---

## 4. Functional Requirements

### 4.1 `init_p01`

1. Set `state.major_mode = 1`, `state.dsky.prog = 1`.
2. Set `state.dsky.verb = 6` (display), `state.dsky.noun = 68`
   (pre-launch summary).
3. Set `state.dsky.flashing = false`.
4. Set `state.imu_alignment_state = ImuAlignmentState::Caged`
   **unconditionally** — including when the platform was previously
   `FineAligned`. P01 is the crew's "start over" lever.
5. Return `PRIORITY` (3).

### 4.2 `init_p02`

1. If `state.imu_alignment_state != Caged`, raise alarm 235 (soft —
   continue).
2. Set `state.major_mode = 2`, `state.dsky.prog = 2`.
3. Set `state.dsky.verb = 6`, `state.dsky.noun = 68`,
   `state.dsky.flashing = false`.
4. Schedule the first iteration:
   `state.waitlist.schedule(GYROCOMPASS_PERIOD_CS, p02_gyrocompass_step)`.
5. Return `PRIORITY` (3).

P02 does **not** change `imu_alignment_state` itself — that transition
happens only when the loop converges.

### 4.3 `p02_gyrocompass_step` (per-iteration)

1. If `state.major_mode != 2`, exit without rescheduling (crew has
   switched away).
2. Compute `cos_lat = cos(state.launch_lat_rad)` and
   `sin_lat = sin(state.launch_lat_rad)`.
3. Compute per-axis drive (saturating cast to `i16`):
   - `drive_x = round(GYROCOMPASS_DRIVE_COUNTS · cos_lat · 0.5)`
   - `drive_y = round(GYROCOMPASS_DRIVE_COUNTS · cos_lat)`
   - `drive_z = round(GYROCOMPASS_DRIVE_COUNTS · sin_lat)`
4. For each axis `i ∈ {0, 1, 2}`:
   `state.current_cdu[i] = drive_toward_zero(state.current_cdu[i], drive_i)`.
   `drive_toward_zero` saturates at zero (does not overshoot).
5. Compute `converged = all(|cdu[i]| ≤ COARSE_ALIGN_THRESHOLD)`.
6. If converged:
   `state.imu_alignment_state = ImuAlignmentState::CoarseAligned`.
   **Do not** reschedule.
7. Otherwise: reschedule next iteration
   `state.waitlist.schedule(GYROCOMPASS_PERIOD_CS, p02_gyrocompass_step)`.

### 4.4 Helpers

- `earth_rate_horizontal(lat_rad) = OMEGA_EARTH · cos(lat_rad)`.
- `earth_rate_vertical(lat_rad)   = OMEGA_EARTH · sin(lat_rad)`.

Both return `f64` rad/s. Used by the `SimEarthRate` simulator hook and by
characterisation tests. The combined magnitude
`sqrt(horizontal² + vertical²)` equals `OMEGA_EARTH` at every latitude.

---

## 5. PROGRAM_TABLE Registration

```rust
PROGRAM_TABLE[1] = Some(p01_p02::init_p01);
PROGRAM_TABLE[2] = Some(p01_p02::init_p02);
```

---

## 6. Restart Protection

P01 has no restart state (it is one-shot).

P02 self-cancels on `major_mode != 2`, so a restart that re-dispatches
P02 via `FRESH START → V37 02 E` will see the freshly cleared waitlist
and start a new convergence loop from the current CDU values. No phase
register is needed.

---

## 7. Transitions

### Into P01 / P02

| Trigger | Source |
|---|---|
| Crew `V37 E 01 E` | V37 handler → `PROGRAM_TABLE[1]` |
| Crew `V37 E 02 E` | V37 handler → `PROGRAM_TABLE[2]` |
| FRESH START → pre-launch sequence | `services::fresh_start` |

### Out of P02

Any V37 to another major mode causes `p02_gyrocompass_step` to exit at
its next firing without rescheduling (step 1 of §4.3). The Waitlist
becomes empty for P02 within one period.

Convergence sets `imu_alignment_state = CoarseAligned` and stops
self-rescheduling but leaves `major_mode = 2`. The crew is expected to
V37 into P00 (or directly into P11 once liftoff is imminent).

---

## 8. Test Cases

The implementation in `agc-core/src/programs/p01_p02.rs::tests` provides
six representative test cases covering both programs and the loop:

| ID | What is verified |
|---|---|
| TC-P01-1 | `init_p01` sets `major_mode=1`, DSKY shows P01, IMU forced to `Caged` even from `FineAligned`. |
| TC-P01-2 | P01 forces `Caged` from `CoarseAligned`. |
| TC-P02-1 | `init_p02` from `Caged` schedules the first step; alignment stays `Caged` until the first iteration fires. |
| TC-P02-2 | `init_p02` from `FineAligned` raises alarm 235 but still sets `major_mode=2` and schedules the loop. |
| TC-P02-3 | Convergence from a 30° misalignment reaches `CoarseAligned` within 200 steps; all three CDUs end within `COARSE_ALIGN_THRESHOLD`. Latitude = KSC (28.6°). |
| TC-P02-4 | An already-aligned platform (CDU = 0) converges in exactly one step. |
| TC-P02-5 | Switching `major_mode` to 0 mid-loop cancels self-rescheduling; alignment does not advance to `CoarseAligned`. |
| TC-P02-6 | `earth_rate_horizontal` and `earth_rate_vertical` at KSC reproduce `OMEGA_EARTH` in their combined magnitude and are both positive. |

---

## 9. Spec Quality Checklist

- [x] AGC source file referenced (`IMU_CALIBRATION_AND_ALIGNMENT.agc`).
- [x] All state fields touched by `init_p01` and `init_p02` listed (§4.1, §4.2).
- [x] Per-iteration step actions listed (§4.3).
- [x] Alarms documented (§3).
- [x] Simulation-speed scaling (§2.3) honestly noted.
- [x] Rust API constants and signatures (§1, §4).
- [x] PROGRAM_TABLE registration documented (§5).
- [x] At least 4 test cases specified (§8).
- [x] Consistency with `specs/imu-control-spec.md` confirmed.
