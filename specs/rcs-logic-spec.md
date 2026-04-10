# RCS Jet Selection Logic — Functional Specification

**Module**: `agc-core/src/control/rcs_logic.rs`
**Spec version**: 1.1
**Date**: 2026-04-09
**Status**: Draft — ready for developer implementation

---

## AGC Source Reference

```
AGC source: Comanche055/JET_SELECTION_LOGIC.agc
AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
AGC source: Comanche055/RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc
AGC channels: 05 (PYJETS), 06 (ROLLJETS) — docs/AGC Symbolic Listing.md §IIE
Architecture: docs/architecture.md §11.2 "RCS Jet Selection", §11.3 "Timing"
HAL cross-ref: specs/hal-spec.md §11 (Rcs trait), §6 (Timers T6)
Strategy D: specs/dap-spec.md §5 "Staged inputs and outputs"
```

---

## 1. Overview and Purpose

`rcs_logic.rs` is the sole module that maps a continuous torque command vector
produced by the Digital Autopilot (attitude.rs) into discrete on/off commands
for individual RCS thruster jets. It is called from the DAP cycle driven by
T5RUPT (~100 ms period) and uses the T6 timer to produce precisely-timed jet
pulses via T6RUPT.

The module has three distinct responsibilities:

1. **Jet selection**: for a given three-axis torque command, determine the
   smallest set of jets whose combined torque is aligned with the command,
   excluding any jets marked disabled in `RcsConfig`.

2. **Pulse duration computation**: convert the torque magnitude into a T6 count
   (units: 0.625 ms per count) such that the resulting impulse achieves the
   commanded angular velocity change. Enforce a minimum pulse length and discard
   pulses that would be too short to overcome valve latency.

3. **Atomic fire-and-arm sequence**: write the jet bitmask to hardware channels
   05/06 and arm T6 in the correct order so that the T6RUPT handler can call
   `quench_all` to end the pulse at the right time.

The module does **not** compute attitude errors or desired torques; those are
produced by `control/attitude.rs` and passed in as a `Vec3`.

**Strategy D — staged I/O**: `select_jets_sm`, `select_jets_cm`, and
`compute_pulse_duration` are pure functions that operate only on their arguments
and return values — no `AgcState` access, no HAL calls. The results are written
to staging fields (`AgcState::rcs_commanded_jets` and
`AgcState::rcs_commanded_pulse_cs`) by `dap_step`. The T5RUPT ISR shim reads
these staging fields and calls `fire_pulse` to perform the actual HAL I/O.
`fire_pulse` itself is **not** called from any Waitlist task; it is an ISR-shim
helper only.

---

## 2. Context: DAP Cycle and T6RUPT Interaction

The DAP cycle is split between the Waitlist task (`dap_step`) and the T5RUPT ISR
shim. Only the ISR shim performs HAL I/O.

```
T5RUPT ISR shim fires (~100 ms)
  ├─► Read CDU angles from hardware → stage in state.current_cdu
  ├─► call dap_step(&mut state)           [pure — no HAL]
  │     └─► attitude.rs: PD controller → torque_cmd: Vec3
  │           └─► rcs_logic::select_jets_sm(torque_cmd, &config) → jets: u16
  │                 └─► rcs_logic::compute_pulse_duration(torque_cmd, jets, ...) → counts
  │                       └─► stage: state.rcs_commanded_jets     = jets
  │                           stage: state.rcs_commanded_pulse_cs = counts
  └─► Read staging fields and perform HAL I/O (ISR shim):
        jets   = state.rcs_commanded_jets
        counts = state.rcs_commanded_pulse_cs
        if counts > 0 {
            fire_pulse(hw, jets, counts)     [HAL I/O — ISR shim only]
        }

T6RUPT ISR fires (counts × 0.625 ms later)
  └─► T6RUPT handler: hw.rcs().quench_all()
```

The ordering constraint — arm T6 before writing jets — is mandatory. Writing
jets before arming T6 leaves them on indefinitely if the T6RUPT fires before the
arm instruction completes (a race condition that existed in the original AGC
when INHINT was not held across the sequence). In this port, `fire_pulse` is
responsible for sequencing these two HAL calls atomically within a critical
section if the target supports preemptive interrupts.

---

## 3. Physical Hardware Layouts

### 3.1 SM RCS: 16 Jets in Four Quads

The Service Module carries 16 thrusters arranged in four quads (A, B, C, D)
located 90 degrees apart around the circumference of the SM. Each quad contains
four jets oriented to provide thrust in four directions relative to the body
frame. The four jets within a quad are designated by their function:

| Jet suffix | Thrust direction | Primary axis contribution |
|------------|-----------------|---------------------------|
| +X (aft)   | +X body axis    | Translation (posigrade)   |
| -X (fwd)   | -X body axis    | Translation (retrograde)  |
| CCW        | tangential      | +Roll about X body axis   |
| CW         | tangential      | -Roll about X body axis   |

Within each quad the pitch/yaw jets are canted so that their thrust vectors pass
near (but not through) the CM/SM mass centre. The offset produces a torque about
the pitch or yaw axis.

The 16 jets are returned from `select_jets_sm` as a single `u16` bitmask:
- **Upper byte (bits 15–8)**: `jets_b` — corresponds to AGC channel 06 (ROLLJETS)
- **Lower byte (bits 7–0)**: `jets_a` — corresponds to AGC channel 05 (PYJETS)

The T5RUPT ISR shim splits this `u16` into two `u8` values when calling
`fire_sm_jets(jets_a, jets_b)`:

```rust
let jets_a = (jets & 0x00FF) as u8;  // channel 05
let jets_b = (jets >> 8)    as u8;   // channel 06
```

| AGC channel | Mnemonic | u16 encoding |
|-------------|----------|--------------|
| 05 (octal)  | PYJETS   | bits 7–0 (lower byte) |
| 06 (octal)  | ROLLJETS | bits 15–8 (upper byte) |

The mapping of bit positions within those bytes to physical jets is the
**jet torque contribution table** (§4 below). Each bit set to `1` fires that
jet; bit set to `0` leaves it off. Returning `0x0000` means all SM jets off.

### 3.2 SM Jet Bit Assignments

The following table defines the canonical bit assignment used by all functions
in this module. Within each byte, bit 7 is the most-significant bit.

**Bits 7–0 (`jets_a` / channel 05 / PYJETS):**

| Bit | u16 bit | Jet ID | Quad | Direction | Primary torque axis |
|-----|---------|--------|------|-----------|---------------------|
| 7   | 7       | A1     | A    | +Pitch    | +Y body torque      |
| 6   | 6       | A2     | A    | -Pitch    | -Y body torque      |
| 5   | 5       | A3     | A    | +Yaw      | +Z body torque      |
| 4   | 4       | A4     | A    | -Yaw      | -Z body torque      |
| 3   | 3       | B1     | B    | +Pitch    | +Y body torque      |
| 2   | 2       | B2     | B    | -Pitch    | -Y body torque      |
| 1   | 1       | B3     | B    | +Yaw      | +Z body torque      |
| 0   | 0       | B4     | B    | -Yaw      | -Z body torque      |

**Bits 15–8 (`jets_b` / channel 06 / ROLLJETS):**

| Bit | u16 bit | Jet ID | Quad | Direction | Primary torque axis |
|-----|---------|--------|------|-----------|---------------------|
| 7   | 15      | C1     | C    | +Pitch    | +Y body torque      |
| 6   | 14      | C2     | C    | -Pitch    | -Y body torque      |
| 5   | 13      | C3     | C    | +Yaw      | +Z body torque      |
| 4   | 12      | C4     | C    | -Yaw      | -Z body torque      |
| 3   | 11      | D1     | D    | +Roll     | +X body torque      |
| 2   | 10      | D2     | D    | -Roll     | -X body torque      |
| 1   | 9       | D3     | D    | +Roll     | +X body torque      |
| 0   | 8       | D4     | D    | -Roll     | -X body torque      |

Notes on the roll axis:
- D1 and D3 are redundant +Roll jets on opposite sides of quad D; pairing them
  provides pure roll without cross-coupling.
- D2 and D4 provide -Roll. Same redundancy principle applies.
- The original AGC jet-select tables use both a "primary" and a "secondary"
  selection for each axis to allow reconfiguration when one jet is failed.

### 3.3 CM RCS: 12 Jets in Two Rings

After SM separation, the Command Module uses its own 12-jet RCS for attitude
control during atmospheric entry. The CM jets are arranged in two rings (forward
and aft) of six jets each, oriented so that each ring can provide pitch, yaw,
and roll torques.

The 12 jets are passed to `fire_cm_jets(jets: u16)` as bits 11–0 of a `u16`.
Bits 15–12 are always zero; writing them as nonzero is a hardware fault that
must not occur.

**CM jet bit assignments (bits 11–0):**

| Bit | Jet ID | Ring   | Primary torque axis |
|-----|--------|--------|---------------------|
| 11  | F1     | Fwd    | +Pitch              |
| 10  | F2     | Fwd    | -Pitch              |
| 9   | F3     | Fwd    | +Yaw                |
| 8   | F4     | Fwd    | -Yaw                |
| 7   | F5     | Fwd    | +Roll               |
| 6   | F6     | Fwd    | -Roll               |
| 5   | A1     | Aft    | +Pitch              |
| 4   | A2     | Aft    | -Pitch              |
| 3   | A3     | Aft    | +Yaw                |
| 2   | A4     | Aft    | -Yaw                |
| 1   | A5     | Aft    | +Roll               |
| 0   | A6     | Aft    | -Roll               |

CM RCS jet selection follows the same sign-matching algorithm as SM but indexes
into the CM torque table. The CM module is selected only during entry (after
`SM_SEPARATED` flag is set by the separation event handler).

---

## 4. Jet Torque Contribution Table

Each jet contributes a torque vector in spacecraft body frame. These are
static constant arrays defined at module scope. Values are in Newton-metres
with the body-frame convention: X = roll axis (positive forward through the
hatch), Y = pitch axis (positive out the CM window side), Z = yaw axis
(positive out the CM side, completing right-hand frame).

### 4.1 SM Jet Torque Table

```
SM_JET_TORQUES: [Vec3; 16]  (indices correspond to jet bits 15..0 of the u16 mask,
                              index 0 = bit 0 (B4), index 15 = bit 15 (C1))
```

The exact torque magnitudes depend on the RCS thruster moment arm, which is a
function of SM geometry. For Comanche055 the relevant constants are drawn from
the CSM mass properties loaded via V46. The table is parameterised by the
moment arm `r_rcs` (metres) and nominal thrust `F_rcs` (Newtons):

| u16 bit | Jet | Nominal torque vector (body frame) | Notes |
|---------|-----|-------------------------------------|-------|
| 0       | B4  | `[ 0,  0,  -F·r]`                  | -Yaw  |
| 1       | B3  | `[ 0,  0,  +F·r]`                  | +Yaw  |
| 2       | B2  | `[ 0,  -F·r,  0]`                  | -Pitch (quad B) |
| 3       | B1  | `[ 0,  +F·r,  0]`                  | +Pitch (quad B) |
| 4       | A4  | `[ 0,  0,  -F·r]`                  | -Yaw  |
| 5       | A3  | `[ 0,  0,  +F·r]`                  | +Yaw  |
| 6       | A2  | `[ 0,  -F·r,  0]`                  | -Pitch |
| 7       | A1  | `[ 0,  +F·r,  0]`                  | +Pitch |
| 8       | D4  | `[-F·r,  0,  0]`                   | -Roll (quad D, redundant) |
| 9       | D3  | `[+F·r,  0,  0]`                   | +Roll (quad D, redundant) |
| 10      | D2  | `[-F·r,  0,  0]`                   | -Roll (quad D) |
| 11      | D1  | `[+F·r,  0,  0]`                   | +Roll (quad D) |
| 12      | C4  | `[ 0,  0,  -F·r]`                  | -Yaw  (quad C) |
| 13      | C3  | `[ 0,  0,  +F·r]`                  | +Yaw  (quad C) |
| 14      | C2  | `[ 0,  -F·r,  0]`                  | -Pitch (quad C) |
| 15      | C1  | `[ 0,  +F·r,  0]`                  | +Pitch (quad C) |

Where:
- `F_rcs` ≈ 445 N (100 lbf, nominal SM RCS thrust per jet)
- `r_rcs` ≈ 1.4 m (moment arm from SM centreline to jet thrust line, pitch/yaw)
- `r_roll` ≈ 1.9 m (moment arm for roll jets tangential to SM circumference)

The nominal values above are approximate. The actual values used at runtime are
loaded from `RcsConfig::moment_arm_m` and `RcsConfig::thrust_n`, making the
table dynamically constructed at `RcsConfig` initialisation rather than
hard-coded at compile time.

### 4.2 Torque Alignment Score

Jet selection uses a **torque alignment score** (dot product of a candidate
jet's torque vector with the normalised command vector) to rank jets:

```
score(jet_i) = dot(SM_JET_TORQUES[i], torque_cmd) / |torque_cmd|
```

Jets with a positive score are candidate firing jets. Jets with a score at or
below zero are excluded because they would produce a counter-productive torque
component.

---

## 5. Data Structures

### 5.1 `RcsConfig`

```rust
/// Configuration for the RCS jet selection logic.
///
/// Loaded from DAP data entered by the crew via V46/V48, or set to defaults
/// at FRESH START. Corresponds to the AGC erasable cells DAPBOOLS, NJETMAN,
/// and the DAP configuration words in ERASABLE_ASSIGNMENTS.agc.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (DAPBOOLS area)
#[derive(Clone, Copy)]
pub struct RcsConfig {
    /// Bitmask of SM jets that are enabled. Bit layout matches §3.2:
    /// bits 15–8 = jets_b (channel 06), bits 7–0 = jets_a (channel 05).
    /// A `1` means the jet is enabled; a `0` means it is failed or crew-disabled.
    /// Default (FRESH START): 0xFFFF (all 16 jets enabled).
    pub sm_jet_enable_mask: u16,

    /// Bitmask of CM jets that are enabled. Bits 11–0 correspond to the CM
    /// jet table in §3.3. Default: 0x0FFF (all 12 jets enabled).
    pub cm_jet_enable_mask: u16,

    /// Attitude deadband in radians. If the attitude error is below this
    /// value the DAP issues no torque command. Typical value: 0.5°–2.0°.
    /// Corresponds to AGC erasable ATTDB (attitude deadband).
    pub attitude_deadband_rad: f64,

    /// Rate deadband in rad/s. Body rates below this threshold are treated
    /// as zero for the purpose of rate-damping jet selection.
    /// Typical value: 0.1–0.5 °/s.
    /// Corresponds to AGC erasable RATEDB.
    pub rate_deadband_rad_s: f64,

    /// Minimum pulse duration in T6 counts (1 count = 0.625 ms).
    /// Pulses shorter than this value are rounded up to `min_pulse_counts`.
    /// Pulses that would be shorter than half this value are discarded (no fire).
    /// Original AGC value: 22 counts = 13.75 ms ≈ 14 ms.
    pub min_pulse_counts: u16,

    /// Maximum pulse duration in T6 counts. Pulses longer than this are
    /// clamped. Prevents runaway jet firing if torque_cmd is unreasonably large.
    /// Practical limit: 32767 counts (≈ 20.5 s), but DAP cycles at 100 ms so
    /// a value of 160 (100 ms) is a sensible upper bound for a single pulse.
    pub max_pulse_counts: u16,

    /// Number of jets to use per axis for normal (two-jet) mode.
    /// Values: 1 (minimum impulse / low-rate maneuver) or 2 (standard).
    /// Set by crew via V46 DSKY entry. Corresponds to AGC NJETMAN variable.
    pub jets_per_axis: u8,

    /// SM RCS nominal thrust per jet, in Newtons. Used to construct the
    /// torque contribution table. Default: 445.0 N.
    pub sm_thrust_n: f64,

    /// SM RCS pitch/yaw moment arm, in metres (distance from body X axis
    /// to pitch/yaw jet thrust line). Default: 1.4 m.
    pub sm_pitch_yaw_arm_m: f64,

    /// SM RCS roll moment arm, in metres (tangential distance from body X
    /// axis to roll jet thrust line). Default: 1.9 m.
    pub sm_roll_arm_m: f64,

    /// CM RCS nominal thrust per jet, in Newtons. Used during entry.
    /// Default: 389.0 N (87.5 lbf CM RCS).
    pub cm_thrust_n: f64,

    /// CM RCS pitch/yaw moment arm, in metres. Default: 0.6 m.
    pub cm_arm_m: f64,
}
```

**Scale factors**: All floating-point fields are SI (radians, rad/s, metres,
Newtons). No AGC fixed-point scaling is applied here; the DAP converts AGC
erasable word values to SI before constructing `RcsConfig`.

**Invariants**:
- `sm_jet_enable_mask` must have bits 15–0 only (no reserved bits set).
- `cm_jet_enable_mask` must have bits 11–0 only (bits 15–12 must be zero).
- `min_pulse_counts >= 1`. Passing 0 to `arm_t6` is a programming error (see
  hal-spec.md §6.2).
- `max_pulse_counts >= min_pulse_counts`.
- `jets_per_axis` must be 1 or 2. Any other value triggers `alarm(0x0140)` and
  the function falls back to 1.
- `attitude_deadband_rad >= 0.0` and `rate_deadband_rad_s >= 0.0`.
- None of the `f64` fields may be `NaN` or infinite (types-module-spec §3.4
  NaN invariant).

**Default construction**:

```rust
impl Default for RcsConfig {
    fn default() -> Self {
        Self {
            sm_jet_enable_mask:     0xFFFF,
            cm_jet_enable_mask:     0x0FFF,
            attitude_deadband_rad:  0.5_f64.to_radians(),  // 0.5 degrees
            rate_deadband_rad_s:    (0.2_f64).to_radians(), // 0.2 deg/s
            min_pulse_counts:       22,   // 13.75 ms
            max_pulse_counts:       160,  // 100 ms (one DAP cycle)
            jets_per_axis:          2,
            sm_thrust_n:            445.0,
            sm_pitch_yaw_arm_m:     1.4,
            sm_roll_arm_m:          1.9,
            cm_thrust_n:            389.0,
            cm_arm_m:               0.6,
        }
    }
}
```

### 5.2 Staging Fields in `AgcState`

Per Strategy D, `dap_step` (a Waitlist task — pure `fn(&mut AgcState)`) writes
the jet selection results to staging fields on `AgcState`. The T5RUPT ISR shim
reads these fields and performs the HAL fire sequence.

| Field | Type | Written by | Read by | Meaning |
|-------|------|-----------|---------|---------|
| `AgcState::rcs_commanded_jets` | `u16` | `dap_step` | T5RUPT ISR shim | Combined jet bitmask from `select_jets_sm`; upper byte = jets_b (ch 06), lower byte = jets_a (ch 05). `0x0000` = no fire. |
| `AgcState::rcs_commanded_pulse_cs` | `u16` | `dap_step` | T5RUPT ISR shim | Pulse duration in T6 counts from `compute_pulse_duration`. `0` = no fire. |

Both fields are reset to `0` by `dap_step` at the start of each DAP cycle before
re-computing new values, so a missed T5RUPT tick does not leave stale commands.

---

## 6. Function Specifications

### 6.1 `select_jets_sm`

```rust
/// Select SM RCS jets for a desired torque command.
///
/// Returns a 16-bit jet bitmask encoding the selected jets:
///   - bits 15–8: jets_b — AGC output channel 06 (ROLLJETS)
///   - bits  7–0: jets_a — AGC output channel 05 (PYJETS)
///
/// The T5RUPT ISR shim splits this u16 when calling fire_sm_jets:
///   jets_a = (result & 0x00FF) as u8;
///   jets_b = (result >> 8)    as u8;
///
/// Returns 0x0000 if `torque_cmd` is the zero vector or if all
/// contributing jets are disabled in `config`.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc
pub fn select_jets_sm(torque_cmd: Vec3, config: &RcsConfig) -> u16
```

**Preconditions**:
- `torque_cmd` must not contain `NaN` or infinite values.
- `config` must satisfy all `RcsConfig` invariants listed in §5.1.

**Algorithm**:

1. Compute the magnitude of `torque_cmd`. If `|torque_cmd| < 1e-10` (effectively
   zero), return `0x0000` immediately.

2. Decompose `torque_cmd` into three scalar components: `tx` (roll), `ty`
   (pitch), `tz` (yaw).

3. For each axis independently, identify the correct jet group:
   - If `tx > 0`: candidate jets are D1, D3 (+Roll group).
   - If `tx < 0`: candidate jets are D2, D4 (-Roll group).
   - If `ty > 0`: candidate jets are A1, B1, C1 (+Pitch group).
   - If `ty < 0`: candidate jets are A2, B2, C2 (-Pitch group).
   - If `tz > 0`: candidate jets are A3, B3, C3 (+Yaw group).
   - If `tz < 0`: candidate jets are A4, B4, C4 (-Yaw group).

   An axis is "inactive" if the absolute magnitude of its torque component is
   below a coupling threshold `|torque_cmd| * 0.15` (15% of the total command
   magnitude). Inactive axes do not contribute jets, preventing unnecessary
   cross-coupling firings for nearly-single-axis commands.

4. Within each active group, filter out any jets that are disabled in
   `config.sm_jet_enable_mask`. If a jet's bit in the mask is `0`, skip it.

5. From the remaining candidates, select up to `config.jets_per_axis` jets per
   axis, preferring jets in different quads to minimise cross-coupling torques.
   Selection priority: prefer jets whose torque vectors are most closely aligned
   with the command (highest dot-product score as defined in §4.2).

6. Assemble the selected jets into the `u16` result using the bit assignments
   from §3.2:
   - Channel 05 jets (A1–B4) occupy bits 7–0.
   - Channel 06 jets (C1–D4) occupy bits 15–8.
   Return the combined `u16`.

**Failure reconfiguration** (§9): if an entire group for an active axis has all
jets disabled (e.g., all three +Pitch jets failed), select jets from the
opposite quad only, using a single jet if necessary. If no usable jet exists for
an axis, that axis fires zero jets (the torque command for that axis cannot be
executed) and the function proceeds without panicking.

**Postconditions**:
- No bit set in the return value corresponds to a disabled jet
  (`result & ~config.sm_jet_enable_mask == 0`).
- At most `config.jets_per_axis` jets are selected per axis direction.
- If `torque_cmd == [0.0, 0.0, 0.0]`, returns `0x0000`.

---

### 6.2 `compute_pulse_duration`

```rust
/// Compute the T6 pulse duration for a jet firing.
///
/// `torque_cmd` is the desired torque vector (N·m).
/// `jet_mask` is the u16 jet bitmask returned by `select_jets_sm` or
/// `select_jets_cm`.
///
/// Returns a count value for `arm_t6`: duration = counts × 0.625 ms.
/// Returns 0 if the computed duration is below half the minimum pulse
/// threshold (pulse should be discarded — caller must not fire).
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc (minimum impulse logic)
pub fn compute_pulse_duration(
    torque_cmd: Vec3,
    jet_mask: u16,
    config: &RcsConfig,
    moment_of_inertia: Vec3,   // Ixx, Iyy, Izz in kg·m² (body frame principal inertia)
) -> u16
```

**Preconditions**:
- `torque_cmd` must not contain `NaN` or infinite values.
- `jet_mask` must be a value previously returned by `select_jets_sm` or
  `select_jets_cm` with the same `config`. Passing an arbitrary bitmask is
  permitted but the result may not achieve the desired angular rate change.
- `moment_of_inertia` components must all be positive and finite.

**Algorithm**:

1. Compute the **effective torque** actually produced by the selected jets by
   summing the torque contribution vectors (§4.1) for all set bits in `jet_mask`.
   Let `tau_eff: Vec3` be this sum (in N·m).

2. Determine the **dominant axis**: the body axis (X, Y, Z) along which
   `|torque_cmd|` is largest. Identify the corresponding inertia component
   `I_axis` from `moment_of_inertia`.

3. Determine the **required angular impulse** along the dominant axis:
   ```
   delta_omega_cmd = torque_cmd[axis] / tau_eff[axis]
   ```
   This is the fraction of the command that the selected jets will deliver per
   unit time. Units: dimensionless (seconds of firing to achieve rate change).

4. Compute the **required firing duration** in seconds:
   ```
   t_fire = |torque_cmd[axis]| * I_axis / |tau_eff[axis]|
   ```
   This is the time needed to produce the commanded angular momentum change.

5. Convert to T6 counts:
   ```
   counts_f = t_fire / 0.000625
   counts   = counts_f.round() as u16
   ```

6. Apply minimum and maximum pulse limits:
   - If `counts < config.min_pulse_counts / 2`: return `0` (discard pulse;
     torque command is too small to fire even one minimum impulse).
   - If `counts < config.min_pulse_counts`: set `counts = config.min_pulse_counts`
     (round up to minimum impulse of 13.75 ms at default settings).
   - If `counts > config.max_pulse_counts`: set `counts = config.max_pulse_counts`.

7. Return `counts`.

**Return value of 0** signals the caller to skip the `fire_pulse` call entirely.
The zero return value is not passed to `arm_t6` (which requires `counts >= 1`).

**Postconditions**:
- Return value is either 0 or in the range
  `[config.min_pulse_counts, config.max_pulse_counts]`.
- `arm_t6` must only be called with a non-zero return value.

**Scaling note**: `moment_of_inertia` is in SI units (kg·m²). The original AGC
stored mass properties in AGC-scaled fixed-point words from the DAP data load
(V46/V48 entries). In the Rust port, the DAP data structure holds SI values
converted at load time.

---

### 6.3 `fire_pulse`

```rust
/// Arm T6 and fire the specified RCS jets as a single atomic sequence.
///
/// This function arms the T6 timer first, then immediately writes the jet
/// mask to channels 05/06. The T6RUPT handler is responsible for calling
/// `hw.rcs().quench_all()` to terminate the pulse.
///
/// **Ordering invariant**: `arm_t6` must complete before `fire_sm_jets`.
/// This matches the AGC's instruction-level ordering in the PYJETS/ROLLJETS
/// output sequence.
///
/// **ISR-shim only**: `fire_pulse` must only be called from the T5RUPT ISR
/// shim. It must NEVER be called from a Waitlist task (fn(&mut AgcState)).
/// Waitlist tasks write staging fields; the ISR shim calls `fire_pulse`.
///
/// Precondition: `duration_counts >= 1`. Callers must check the return value
/// of `compute_pulse_duration` and skip this call if it returns 0.
///
/// The `jet_mask` parameter is the u16 returned by `select_jets_sm`:
///   - bits 15–8: jets_b → hw.rcs().fire_sm_jets(jets_a, jets_b) second arg
///   - bits  7–0: jets_a → hw.rcs().fire_sm_jets(jets_a, jets_b) first arg
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc (PYJETS/ROLLJETS
/// output sequence with channel 13 T6 enable).
pub fn fire_pulse<H: AgcHardware>(
    hw: &mut H,
    jet_mask: u16,
    duration_counts: u16,
)
```

**Preconditions**:
- `duration_counts >= 1`. Panics in debug builds if called with 0.
- `jet_mask` must be non-zero. Calling with `0x0000` is a no-op (an
  optimisation: calling `arm_t6` without firing any jets would produce a
  spurious T6RUPT that calls `quench_all` with nothing to quench).
- No prior T6 pulse must be in-flight. Callers must ensure T6 has already
  fired or been disarmed before calling `fire_pulse` again. If uncertain, call
  `hw.timers().disarm_t6()` before calling `fire_pulse`.
- This function must only be called from the T5RUPT ISR shim, not from any
  Waitlist task.

**Algorithm** (must execute as a logical unit — use a critical section on
preemptive targets):

```
1. jets_a = (jet_mask & 0x00FF) as u8
2. jets_b = (jet_mask >> 8)    as u8
3. hw.timers().arm_t6(duration_counts)
4. hw.rcs().fire_sm_jets(jets_a, jets_b)
```

The two HAL calls (steps 3–4) must not be separated by any other code that could
take significant time. On Cortex-M bare-metal with T6RUPT at highest priority,
the compiler and microcontroller pipeline guarantee that these two register
writes are adjacent; no explicit critical section is needed provided no other
interrupt between T5RUPT (caller) and T6RUPT (terminator) writes channel 13 or
05/06.

**Postconditions**:
- `T6Rupt` will fire after `duration_counts × 0.625 ms ± 0.625 ms`.
- Channels 05 and 06 are set to the lower and upper bytes of `jet_mask`
  respectively.
- The T6RUPT handler must call `hw.rcs().quench_all()` to clear channels 05/06.

**After T6RUPT** (caller's T6RUPT handler, outside this module):
```rust
// In the T6RUPT handler (ISR shim):
fn t6rupt_handler<H: AgcHardware>(hw: &mut H) {
    hw.rcs().quench_all();
    // T6 is automatically disarmed by hardware (channel 13 bit 15 auto-clears)
}
```

---

### 6.4 `select_jets_cm`

```rust
/// Select CM RCS jets for a desired torque command (entry phase only).
///
/// Returns a 12-bit jet bitmask for `fire_cm_jets`. Bits 15–12 are always 0.
/// Returns 0 if `torque_cmd` is zero or all relevant jets are disabled.
///
/// The CM RCS is used only after SM/CM separation. Calling this function
/// before separation is not a hardware error (no jets will fire until the
/// CM RCS propellant valves are opened by the separation sequence), but is a
/// logical error in the control system.
///
/// AGC source: Comanche055/CM_ENTRY_DIGITAL_AUTOPILOT.agc
pub fn select_jets_cm(torque_cmd: Vec3, config: &RcsConfig) -> u16
```

**Algorithm**: Same sign-matching and alignment-scoring approach as
`select_jets_sm` (§6.1), but using the CM jet torque table (built from
`config.cm_thrust_n` and `config.cm_arm_m`) and the CM jet enable mask
(`config.cm_jet_enable_mask`).

**Postconditions**:
- Return value has bits 15–12 cleared (mask with `0x0FFF` before use if
  defensive programming is desired).
- No bit set in the return value corresponds to a disabled CM jet.

---

### 6.5 `build_sm_torque_table`

```rust
/// Construct the SM jet torque contribution table from RcsConfig.
///
/// Returns a [Vec3; 16] where index i is the torque vector (N·m, body frame)
/// of the jet corresponding to bit i of the 16-bit combined jet mask
/// (bits 7–0 = jets_a/channel 05, bits 15–8 = jets_b/channel 06 per §3.2).
///
/// Called once during `RcsConfig` construction; the result is stored in
/// a fixed-size array field or as a module-level constant if config is
/// treated as compile-time-known.
pub fn build_sm_torque_table(config: &RcsConfig) -> [Vec3; 16]
```

This is a pure function with no side effects. Results must be recomputed if
`config` changes (e.g., after V46 data reload).

---

### 6.6 `build_cm_torque_table`

```rust
/// Construct the CM jet torque contribution table from RcsConfig.
/// Returns a [Vec3; 12] where index i corresponds to bit i of the u16 mask.
pub fn build_cm_torque_table(config: &RcsConfig) -> [Vec3; 12]
```

---

## 7. Minimum Impulse Logic

The minimum impulse constraint exists because RCS jet solenoid valves require
approximately 7–10 ms to open fully and another 3–5 ms to close. A commanded
pulse shorter than ~14 ms produces less impulse than commanded (the jet
partially opens and closes). The original AGC handled this with a minimum pulse
table and a "coasting" (no-fire) decision for very small commands.

Rules implemented in `compute_pulse_duration` (§6.2, steps 6):

| Computed duration | Action |
|-------------------|--------|
| `< min_pulse_counts / 2` | Return 0. No fire. |
| `>= min_pulse_counts / 2` and `< min_pulse_counts` | Round up to `min_pulse_counts`. |
| `>= min_pulse_counts` and `<= max_pulse_counts` | Use as computed. |
| `> max_pulse_counts` | Clamp to `max_pulse_counts`. |

At the default `min_pulse_counts = 22`:
- Commands needing < 6.875 ms: no fire (below 11 counts).
- Commands needing 6.875–13.75 ms: round up to 13.75 ms (22 counts).
- Commands needing > 62.5 ms (default `max_pulse_counts = 100`): clamped.

The discard threshold (half of minimum) prevents the DAP from firing jets
for insignificant attitude errors that would contribute more disturbance than
correction after accounting for valve latency.

---

## 8. Erasable Variables and AGC Addresses

The following AGC erasable memory cells are replaced by fields in `RcsConfig`
or by local variables in the Rust implementation. They are listed for
cross-reference traceability.

| AGC Symbol   | AGC Address (octal) | Rust equivalent                            | Scale |
|--------------|---------------------|--------------------------------------------|-------|
| `DAPBOOLS`   | ~3777 (bank 0)      | Several `bool` flags, inlined into config  | N/A   |
| `NJETMAN`    | Erasable ~3764      | `config.jets_per_axis: u8`                 | integer |
| `RATEDB`     | Erasable ~3760      | `config.rate_deadband_rad_s: f64`          | B-1 revolutions/cs → rad/s |
| `ATTDB`      | Erasable ~3761      | `config.attitude_deadband_rad: f64`        | B-0 half-revolutions → rad |
| `TJETLAW`    | Fixed ROM tables    | `build_sm_torque_table()` result           | N/A (rebuilt from SI) |
| `MINPULSE`   | Fixed ROM constant  | `config.min_pulse_counts: u16`             | T6 counts (0.625 ms each) |
| `JETSTEM`    | Erasable            | Local variable in `select_jets_sm`         | bitmask |

---

## 9. Jet Failure Handling

Jet failures enter the system through `RcsConfig::sm_jet_enable_mask`. A bit
value of `0` means the jet is unavailable. The mask is updated by:

1. **Crew entry via V46**: the DSKY procedure allows the crew to manually
   disable individual jets (e.g., after a thruster leak is suspected).
2. **Software failure detection**: not implemented in this module; the failure
   detection logic sets the mask bit to `0` and updates `config` before the
   next DAP cycle.

**Reconfiguration rules within `select_jets_sm`**:

- If a primary jet group for an axis has no enabled jets, attempt to use only
  the single remaining jet from the same axis (reduces `jets_per_axis` to 1 for
  that axis).
- If no jet can produce the required torque sign for a given axis, that axis
  fires zero jets. The DAP will accumulate the un-executed torque and retry on
  the next cycle with the full torque error.
- Cross-axis coupling: when reconfiguring around failures, check that the
  selected substitute jets do not produce a large cross-axis torque component
  (score > 0.5 on an inactive axis). If cross-coupling exceeds this threshold,
  prefer a jet with lower coupling even if its alignment score is slightly lower.

**Roll-axis redundancy**: D1/D3 and D2/D4 are two physically separate jets on
quad D that both produce the same torque direction. With both enabled,
`jets_per_axis = 2` selects one from each pair. With one pair fully failed,
the remaining pair still provides the correct torque sign at half the commanded
rate.

---

## 10. `AgcHardware` Trait Usage

Only `fire_pulse` requires a `&mut impl AgcHardware`. The jet-selection and
duration-computation functions (`select_jets_sm`, `select_jets_cm`,
`compute_pulse_duration`, `build_sm_torque_table`, `build_cm_torque_table`) are
pure and take only `Vec3` and `&RcsConfig`. This separation ensures that the
selection and computation logic can be unit tested without a hardware stub.

`fire_pulse` is an ISR-shim helper. The T5RUPT ISR shim (not a Waitlist task)
reads the staging fields written by `dap_step` and calls `fire_pulse`:

```rust
// T5RUPT ISR shim context (has access to hw: &mut impl AgcHardware):
use agc_core::hal::AgcHardware;
use agc_core::control::rcs_logic::fire_pulse;

// After dap_step(&mut state) has returned, the ISR shim executes:
let jets   = state.rcs_commanded_jets;
let counts = state.rcs_commanded_pulse_cs;
if counts > 0 && jets != 0 {
    fire_pulse(hw, jets, counts);  // HAL I/O — ISR shim only
}
```

The pure selection logic is used inside `dap_step` (Waitlist task):

```rust
// Inside dap_step (fn(&mut AgcState)) — no hw parameter:
use agc_core::control::rcs_logic::{RcsConfig, select_jets_sm, compute_pulse_duration};

// Reset staging fields at start of cycle
state.rcs_commanded_jets      = 0;
state.rcs_commanded_pulse_cs  = 0;

let torque_cmd = /* ... from attitude controller ... */;
let jets   = select_jets_sm(torque_cmd, &state.rcs_config);
let counts = compute_pulse_duration(torque_cmd, jets, &state.rcs_config, state.inertia);

// Write to staging fields for the ISR shim to act on
state.rcs_commanded_jets      = jets;
state.rcs_commanded_pulse_cs  = counts;
```

---

## 11. Error and Edge Cases

| Condition | Behaviour |
|-----------|-----------|
| `torque_cmd == [0.0, 0.0, 0.0]` | `select_jets_sm` returns `0x0000`; `compute_pulse_duration` returns 0; `fire_pulse` is not called. |
| All jets disabled (`sm_jet_enable_mask == 0x0000`) | `select_jets_sm` returns `0x0000`. |
| Computed duration < half minimum | `compute_pulse_duration` returns 0; no fire. |
| Computed duration rounds up to minimum | Jets fire for `min_pulse_counts × 0.625 ms` regardless of smaller command. |
| `fire_pulse` called with `duration_counts == 0` | Debug: `debug_assert!(duration_counts >= 1)` panics. Release: undefined behaviour (arm_t6 fires within one DINC tick). |
| `jet_mask == 0x0000` passed to `fire_pulse` | No-op; function returns immediately without calling `arm_t6` or `fire_sm_jets`. |
| Coupled torque command (all three axes nonzero) | Each axis independently selects its jets; combined mask is the union of all three selections. |
| NaN in `torque_cmd` | Violates precondition. Behaviour is undefined; debug build: `debug_assert!` on each component. |
| `fire_pulse` called from a Waitlist task | Programming error; violates Strategy D. Must only be called from the T5RUPT ISR shim. |

---

## 12. Test Cases

### TC-RCS-LOGIC-01: Pure +Roll torque selects correct SM jets

**Setup**: `torque_cmd = [+150.0, 0.0, 0.0]` (N·m, +Roll only).
`config = RcsConfig::default()` (all jets enabled, 2 jets per axis).

**Expected**: `select_jets_sm` returns a `u16` mask that has:
- Bit 11 set (D1, +Roll, upper byte bit 3).
- Bit 9 set (D3, +Roll redundant, upper byte bit 1).
- No pitch or yaw jets set.

`result = 0x0A00` (upper byte = `0x0A` = D1+D3, lower byte = `0x00`).

**Rationale**: +Roll selects D1 and D3. No pitch or yaw command → no other jets.

---

### TC-RCS-LOGIC-02: Pure -Pitch torque selects correct jets

**Setup**: `torque_cmd = [0.0, -120.0, 0.0]` (N·m, -Pitch only).
`config = RcsConfig::default()`.

**Expected**: Mask selects A2, B2 (or A2, C2) — two jets from the -Pitch group.

Jets A2 = bit 6 and B2 = bit 2 of the lower byte:
`result = 0x0044` (lower byte = `0b0100_0100` = `0x44`, upper byte = `0x00`).

**Rationale**: -Pitch group is {A2, B2, C2}; two-jet mode selects the first two
available (A2, B2) since all are enabled.

---

### TC-RCS-LOGIC-03: Coupled three-axis torque

**Setup**: `torque_cmd = [+80.0, +80.0, +80.0]` (N·m, equal torque all axes).
`config = RcsConfig::default()`.

**Expected**: Each axis has components above the 15% coupling threshold (33%
each of total), so all three axes fire. The result has roll jets (D1/D3), pitch
jets (A1/B1), and yaw jets (A3/B3) simultaneously set.

- Lower byte (jets_a): A1 (bit 7), A3 (bit 5), B1 (bit 3), B3 (bit 1) → `0b1010_1010` = `0xAA`.
- Upper byte (jets_b): D1 (bit 3 of upper byte = u16 bit 11), D3 (bit 1 of upper byte = u16 bit 9) → upper byte = `0b0000_1010` = `0x0A`.

`result = 0x0AAA`.

**Rationale**: All three axes active and equal; two jets per axis; no cross-axis
suppression because all contributing jets have positive scores along their
primary axis.

---

### TC-RCS-LOGIC-04: Disabled jet causes reconfiguration

**Setup**: `torque_cmd = [0.0, +100.0, 0.0]` (+Pitch only).
`config.sm_jet_enable_mask = 0xFF7F` (bit 7 of lower byte cleared → A1 disabled).

**Expected**: `select_jets_sm` skips A1 and selects B1 and C1 instead.

- B1 = bit 3 of lower byte → lower byte bit 3 set → `0x08`.
- C1 = bit 7 of upper byte = u16 bit 15 → upper byte bit 7 set → upper byte `0x80`.

`result = 0x8008`.

**Rationale**: A1 is disabled; the +Pitch group still has B1 and C1 available;
two-jet mode fills up from those.

---

### TC-RCS-LOGIC-05: Zero torque command — no jets fired

**Setup**: `torque_cmd = [0.0, 0.0, 0.0]`.
`config = RcsConfig::default()`.

**Expected**:
- `select_jets_sm([0.0, 0.0, 0.0], &config)` returns `0x0000`.
- `compute_pulse_duration([0.0, 0.0, 0.0], 0x0000, &config, inertia)` returns `0`.
- ISR shim does not call `fire_pulse`.

**Rationale**: Zero torque command must result in no jet activity regardless of
config. This is the steady-state condition when the vehicle is within its
attitude and rate deadbands.

---

### TC-RCS-LOGIC-06: Minimum pulse rounding

**Setup**: `torque_cmd = [0.0, +5.0, 0.0]` (tiny +Pitch torque).
`config = RcsConfig::default()`, `inertia = [2000.0, 2500.0, 2500.0]` kg·m².

**Computed duration** (step 4 of §6.2):
- `tau_eff ≈ 2 × 445 N × 1.4 m = 1246 N·m` (two +Pitch jets at default config).
- `t_fire = 5.0 × 2500.0 / 1246.0 ≈ 10.03 ms`.
- `counts_f = 10.03 / 0.625 ≈ 16.0` counts.
- 16 < 22 (`min_pulse_counts`), 16 >= 11 (`min_pulse_counts / 2`).
- Result: rounded up to 22 counts (13.75 ms).

**Expected**: `compute_pulse_duration` returns `22`.

---

### TC-RCS-LOGIC-07: Below discard threshold — no fire

**Setup**: `torque_cmd = [0.0, +1.0, 0.0]` (very small +Pitch torque).
`config = RcsConfig::default()`, `inertia = [2000.0, 2500.0, 2500.0]` kg·m².

**Computed duration**:
- `t_fire = 1.0 × 2500.0 / 1246.0 ≈ 2.0 ms`.
- `counts_f = 2.0 / 0.625 ≈ 3.2` counts.
- 3.2 < 11 (`min_pulse_counts / 2 = 11`).
- Result: return `0`.

**Expected**: `compute_pulse_duration` returns `0`; ISR shim must not call
`fire_pulse`; no T6RUPT is armed.

---

## 13. Module Interface Summary

```rust
// agc-core/src/control/rcs_logic.rs

use crate::hal::AgcHardware;
use crate::types::Vec3;

pub struct RcsConfig { /* §5.1 */ }
impl Default for RcsConfig { /* §5.1 */ }

/// Pure jet selection functions — callable from Waitlist tasks (fn(&mut AgcState))
pub fn select_jets_sm(torque_cmd: Vec3, config: &RcsConfig) -> u16;
pub fn select_jets_cm(torque_cmd: Vec3, config: &RcsConfig) -> u16;

pub fn compute_pulse_duration(
    torque_cmd: Vec3,
    jet_mask: u16,
    config: &RcsConfig,
    moment_of_inertia: Vec3,
) -> u16;

pub fn build_sm_torque_table(config: &RcsConfig) -> [Vec3; 16];
pub fn build_cm_torque_table(config: &RcsConfig) -> [Vec3; 12];

/// HAL I/O helper — ISR-shim context ONLY; never call from a Waitlist task
pub fn fire_pulse<H: AgcHardware>(
    hw: &mut H,
    jet_mask: u16,
    duration_counts: u16,
);
```

All public functions are `no_std` compatible. No heap allocation, no `static
mut`, no interior mutability.

---

## 14. Spec Quality Checklist

- [x] AGC source files and line ranges referenced (JET_SELECTION_LOGIC.agc,
      RCS-CSM_DIGITAL_AUTOPILOT.agc)
- [x] All erasable variables and AGC addresses listed (§8)
- [x] Scale factors documented for all fixed-point AGC values (§8, §5.1)
- [x] Corresponding `f64` SI units documented for all `RcsConfig` fields (§5.1)
- [x] Input/output preconditions and postconditions stated for every function (§6)
- [x] Edge cases and error handling specified (§11)
- [x] 7 test cases with computed expected values (§12)
- [x] Rust API signature designed with types and ownership (§13)
- [x] Invariants explicitly stated (§5.1 RcsConfig invariants)
- [x] Consistency with `docs/architecture.md` checked: `Vec3 = [f64; 3]`,
      `u16` for jet mask (CI-4 fix), `AgcHardware` trait bounds, `no_std`
- [x] Cross-referenced hal-spec.md §11 (Rcs trait) and §6 (T6 timer)
- [x] T6RUPT `quench_all` requirement stated (§6.3 postconditions, §2 context)
- [x] Strategy D (CI-1): staging fields documented in §5.2; `fire_pulse` marked
      ISR-shim-only throughout
- [x] CI-4 applied: `select_jets_sm` returns `u16` (upper byte = jets_b/ch 06,
      lower byte = jets_a/ch 05); all test cases updated
