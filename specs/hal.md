# Functional Specification: Hardware Abstraction Layer (`agc-core/src/hal/`)

AGC source references:
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — channel assignments (pages 41-42)
- `Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` — DSKY key codes, relay table (pages 313-315)
- `Comanche055/T4RUPT_PROGRAM.agc` — DSKY relay output (DSPOUT), IMU monitoring (IMUMON)
- `Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc` — IMU state machine (IMUCOARS, IMUFINE, IMUZERO)
- `Comanche055/JET_SELECTION_LOGIC.agc` — RCS 16-jet channel topology (JETSLECT, T6START)
- `docs/architecture.md` §4 — HAL design, sub-trait structure, typestate conventions

---

## 1. Overview

The HAL is the sole boundary between the flight software and physical hardware.
All peripheral access must go through the `AgcHardware` super-trait and its
sub-traits. No peripheral register or hardware address may appear outside `hal/`.

The HAL is split into one focused sub-trait per physical subsystem:

| Sub-trait | Module | Physical subsystem |
|---|---|---|
| `DskyIo` | `hal/dsky.rs` | Display/keyboard unit |
| `ImuIo` | `hal/imu.rs` | Inertial Measurement Unit |
| `OpticsIo` | `hal/optics.rs` | CM optics (sextant/telescope shaft/trunnion) |
| `EngineIo` | `hal/engine.rs` | SPS main engine + gimbal |
| `RcsIo` | `hal/rcs.rs` | RCS jets |
| `Timers` | `hal/timers.rs` | T3/T4/T5/T6 hardware timers |
| `Uplink` | `hal/uplink.rs` | Ground uplink receiver |
| `Telemetry` | `hal/telemetry.rs` | PCM downlink transmitter |

The `AgcHardware` super-trait collects all of these via associated types.

---

## 2. Channel Number Reference

From `Comanche055/ERASABLE_ASSIGNMENTS.agc`, page 41:

| Symbol | Octal | Purpose |
|---|---|---|
| `PYJETS` | 05 | Pitch + yaw jet commands (output) |
| `ROLLJETS` | 06 | Roll jet commands (output) |
| `OUT0` | 10 | DSKY relay output word |
| `DSALMOUT` | 11 | DSKY alarm lamp bits |
| `CHAN12` | 12 | IMU control discrete outputs |
| `CHAN13` | 13 | RCS/TVC discrete outputs |
| `CHAN14` | 14 | ISS CDU pulse commands |
| `MNKEYIN` | 15 | Main DSKY keyboard input (5-bit key code) |
| `NAVKEYIN` | 16 | Nav DSKY keyboard input |
| `CHAN30` | 30 | IMU status bits (input) |
| `CHAN31` | 31 | Hand-controller / discrete inputs |
| `CHAN32` | 32 | Proceed button and other discretes |
| `CHAN33` | 33 | PIPA fail / uplink / downlink flags |
| `DNTM1` | 34 | Downlink telemetry word 1 (output) |
| `DNTM2` | 35 | Downlink telemetry word 2 (output) |

Time registers (counters, not channels):

| Symbol | Octal | Period |
|---|---|---|
| `TIME1` | 25 | 10 ms tick (T3RUPT base) |
| `TIME2` | 24 | Overflow of TIME1 (≈327 s) |
| `TIME3` | 26 | Waitlist timer (T3RUPT when overflows to 0) |
| `TIME4` | 27 | T4RUPT every 120 ms nominal |
| `TIME5` | 30 | T5RUPT DAP cycle (20-100 ms) |
| `TIME6` | 31 | T6RUPT jet timing (14 ms minimum impulse) |

---

## 3. IMU Typestate Types

The IMU requires compile-time enforcement of the alignment state machine.
These marker types are defined in `hal/imu.rs` and used as type parameters:

```rust
/// Marker: IMU has not yet been coarse-aligned.
pub struct Unaligned;

/// Marker: IMU coarse alignment complete; CDU error counters enabled.
/// AGC source: IMUCOARS routine, IMU_MODE_SWITCHING_ROUTINES.agc page 1423.
pub struct CoarseAligned;

/// Marker: IMU fine alignment complete; gyro torque available.
/// AGC source: IMUFINE routine, IMU_MODE_SWITCHING_ROUTINES.agc page 1427.
pub struct FineAligned;
```

These are zero-sized types with no fields. They are used as `PhantomData<State>`
in the bare-metal `ImuImpl<State>` struct. The `ImuIo` trait is implemented for
all three states; methods that require a specific alignment level are only present
on the appropriate `impl ImuImpl<State>` block (not on the trait).

---

## 4. `AgcHardware` Super-Trait

```
Module: agc-core/src/hal/mod.rs
```

```rust
/// Bound that the flight software requires of the platform.
///
/// The bare-metal implementation wires each associated type to an
/// `embedded-hal` peripheral wrapper. The `agc-sim` implementation wires
/// each associated type to a software model.
///
/// All mutable access to hardware goes through this trait and its sub-traits.
/// No peripheral register access is permitted outside `hal/`.
pub trait AgcHardware {
    type Timers: Timers;
    type Dsky: DskyIo;
    type Imu: ImuIo;
    type Optics: OpticsIo;
    type Engine: EngineIo;
    type Rcs: RcsIo;
    type Uplink: UplinkIo;
    type Telemetry: TelemetryIo;

    fn timers(&mut self) -> &mut Self::Timers;
    fn dsky(&mut self) -> &mut Self::Dsky;
    fn imu(&mut self) -> &mut Self::Imu;
    fn optics(&mut self) -> &mut Self::Optics;
    fn engine(&mut self) -> &mut Self::Engine;
    fn rcs(&mut self) -> &mut Self::Rcs;
    fn uplink(&mut self) -> &mut Self::Uplink;
    fn telemetry(&mut self) -> &mut Self::Telemetry;

    /// Reset the night-watchman (hardware watchdog) timer.
    /// Must be called at least once per Executive loop iteration.
    /// If not called within ~1.28 s, the hardware triggers a restart.
    fn pet_watchdog(&mut self);

    /// Trigger an immediate hardware restart.
    /// Called by the alarm system on unrecoverable failure (GOJAM equivalent).
    fn hardware_restart(&mut self) -> !;
}
```

---

## 5. `DskyIo` Sub-Trait

```
AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (pages 307-315, 399+)
            Comanche055/T4RUPT_PROGRAM.agc (DSPOUT, CDRVE, RELTAB, pages 133-138)
Module:     agc-core/src/hal/dsky.rs
Channel:    OUT0 (octal 10) for display relay output
            DSALMOUT (octal 11) for alarm lamp bits
            MNKEYIN (octal 15) for main keyboard input
            NAVKEYIN (octal 16) for nav keyboard input
```

### Behavior Summary

The DSKY communicates via two output channels and two input channels. Display
output uses the DSPTAB buffer mechanism: the T4RUPT routine (DSPOUT) cycles
through an 11-register table, writing one relay word to OUT0 (channel 10) each
time. Alarm lamps are written separately to DSALMOUT (channel 11).

**Output format (OUT0 = channel 10):**

```
Bits 15-12: relay word selector (A, 4 bits)
Bit  11:    special relay (sign, lamp override, etc.)
Bits 10-6:  5-bit relay code for left character of selected pair
Bits 5-1:   5-bit relay code for right character of selected pair
```

Relay word selectors (from PINBALL_GAME_BUTTONS_AND_LIGHTS.agc page 313-314):

| RELAYWD | Display positions |
|---|---|
| 1011 (11) | MD1/MD2 (major mode digits) |
| 1010 (10) | VD1/VD2 (verb digits) |
| 1001 (9) | ND1/ND2 (noun digits) |
| 1000 (8) | R1D1 |
| 0111 (7) | +R1 sign, R1D2/R1D3 |
| 0110 (6) | -R1 sign, R1D4/R1D5 |
| 0101 (5) | +R2 sign, R2D1/R2D2 |
| 0100 (4) | -R2 sign, R2D3/R2D4 |
| 0011 (3) | R2D5/R3D1 |
| 0010 (2) | +R3 sign, R3D2/R3D3 |
| 0001 (1) | -R3 sign, R3D4/R3D5 |

**5-bit relay codes** (from PINBALL_GAME_BUTTONS_AND_LIGHTS.agc page 315):

| Digit | Code |
|---|---|
| blank | 00000 |
| 0 | 10101 |
| 1 | 00011 |
| 2 | 11001 |
| 3 | 11011 |
| 4 | 01111 |
| 5 | 11110 |
| 6 | 11100 |
| 7 | 10011 |
| 8 | 11101 |
| 9 | 11111 |

**Alarm/status lamps (DSALMOUT = channel 11):**

| Bit | Lamp |
|---|---|
| Bit 5 | KEY RELEASE |
| Bit 6 | VERB/NOUN FLASH |
| Bit 7 | OPERATOR ERROR |
| Bit 4 | TEMP (IMU temp out of limits, set by TLIM in T4RUPT) |

**Keyboard input (MNKEYIN = channel 15):**

5-bit key codes from PINBALL_GAME_BUTTONS_AND_LIGHTS.agc page 313:

| Key | Code (octal) | Code (decimal) | Rust variant |
|---|---|---|---|
| 0 | 20 | 16 | `Key::Zero` |
| 1 | 01 | 1 | `Key::One` |
| 2 | 02 | 2 | `Key::Two` |
| 3 | 03 | 3 | `Key::Three` |
| 4 | 04 | 4 | `Key::Four` |
| 5 | 05 | 5 | `Key::Five` |
| 6 | 06 | 6 | `Key::Six` |
| 7 | 07 | 7 | `Key::Seven` |
| 8 | 10 | 8 | `Key::Eight` |
| 9 | 11 | 9 | `Key::Nine` |
| VERB | 21 | 17 | `Key::Verb` |
| ERROR RESET | 22 | 18 | `Key::Reset` |
| KEY RELEASE | 31 | 25 | `Key::KeyRel` |
| + | 32 | 26 | `Key::Plus` |
| - | 33 | 27 | `Key::Minus` |
| ENTER | 34 | 28 | `Key::Enter` |
| CLEAR | 36 | 30 | `Key::Clear` |
| NOUN | 37 | 31 | `Key::Noun` |

Key code 24 (octal) = PRO (Proceed) is read from channel 32 bit 14, not
from the keyboard input channel (see PROCEEDE routine, T4RUPT_PROGRAM.agc page 136).

### `Key` Enum

```rust
/// 5-bit DSKY keyboard key code.
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 313.
/// Input channel: MNKEYIN (octal 15) for main DSKY; NAVKEYIN (octal 16) for nav DSKY.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Key {
    Zero   = 0x10,   // octal 20
    One    = 0x01,
    Two    = 0x02,
    Three  = 0x03,
    Four   = 0x04,
    Five   = 0x05,
    Six    = 0x06,
    Seven  = 0x07,
    Eight  = 0x08,   // octal 10
    Nine   = 0x09,   // octal 11
    Verb   = 0x11,   // octal 21
    Reset  = 0x12,   // octal 22 — ERROR LIGHT RESET
    KeyRel = 0x19,   // octal 31 — KEY RELEASE
    Plus   = 0x1A,   // octal 32
    Minus  = 0x1B,   // octal 33
    Enter  = 0x1C,   // octal 34
    Clear  = 0x1E,   // octal 36
    Noun   = 0x1F,   // octal 37
}
```

### `RelayWord` and `DigitRow`

```rust
/// A packed 15-bit word for channel OUT0 (channel 10).
/// Format: [bits15-12: relay selector][bit11: special][bits10-6: left code][bits5-1: right code]
#[derive(Clone, Copy)]
pub struct RelayWord(pub u16);

impl RelayWord {
    pub fn new(relay_selector: u8, special: bool, left_code: u8, right_code: u8) -> Self;
    pub fn relay_selector(self) -> u8;
    pub fn special(self) -> bool;
    pub fn left_code(self) -> u8;
    pub fn right_code(self) -> u8;
}

/// A decoded display row (verb, noun, major mode, or R1/R2/R3).
/// Each digit is in [0, 9] or 0xFF for blank.
#[derive(Clone, Copy, Debug)]
pub struct DigitRow {
    pub digits: [u8; 5],  // up to 5 digit positions; unused positions = 0xFF
    pub sign_plus: bool,  // R1/R2/R3 sign
    pub sign_minus: bool,
}
```

### Trait Definition

```rust
/// DSKY I/O interface.
///
/// Isolates the flight software from the display relay hardware.
///
/// AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc,
///             Comanche055/T4RUPT_PROGRAM.agc (DSPOUT routine).
/// Channels:   OUT0 (octal 10), DSALMOUT (octal 11),
///             MNKEYIN (octal 15), NAVKEYIN (octal 16).
pub trait DskyIo {
    /// Read the next keyboard keypress, or `None` if no key is pending.
    /// Corresponds to reading MNKEYIN (channel 15) in KEYRUPT1.
    fn read_key(&mut self) -> Option<Key>;

    /// Read from the navigation DSKY keyboard (NAVKEYIN, channel 16).
    fn read_nav_key(&mut self) -> Option<Key>;

    /// Write a relay word to the display (channel OUT0, octal 10).
    /// Called by T4RUPT DSPOUT once per display scan cycle.
    /// The relay word encodes which character pair to update and the
    /// 5-bit segment codes for each character.
    fn write_relay(&mut self, word: RelayWord);

    /// Write the alarm/status lamp bits (channel DSALMOUT, octal 11).
    /// Bit 5 = KEY RELEASE lamp, Bit 6 = VERB/NOUN FLASH, Bit 7 = OPERATOR ERROR,
    /// Bit 4 = TEMP lamp.
    fn write_lamp_word(&mut self, bits: u16);

    /// Set the PROG (major mode) digits on the DSKY panel.
    /// This is a higher-level convenience; implementation writes to DSPTAB[10].
    fn write_prog(&mut self, prog: u8);

    /// Set the VERB digit field.
    fn write_verb(&mut self, verb: u8);

    /// Set the NOUN digit field.
    fn write_noun(&mut self, noun: u8);

    /// Set a register row (R1, R2, or R3). Row index 0=R1, 1=R2, 2=R3.
    fn write_register(&mut self, row: usize, value: &DigitRow);

    /// True if the PROCEED button is currently pressed.
    /// Corresponds to channel 32 bit 14 (PROCEEDE routine, T4RUPT_PROGRAM.agc).
    fn proceed_pressed(&self) -> bool;
}
```

### C-FREE

The bare-metal DSKY implementation (`DskyImpl`) must expose:

```rust
impl DskyImpl {
    /// Release the underlying SPI/GPIO peripheral handles.
    pub fn free(self) -> (SpiPeripheral, GpioPin, ...);
}
```

### C-HAL-TRAITS

The bare-metal `DskyImpl` should implement `embedded_hal::spi::SpiDevice` (or
the appropriate bus trait) for the relay drive circuit.

### agc-sim Impact

- `SimHardware` must implement `DskyIo` using an in-memory `DskyDisplayState`
  struct that the TUI reads on each render tick.
- `DskyDisplayState` fields: `prog: u8`, `verb: u8`, `noun: u8`,
  `r1: DigitRow`, `r2: DigitRow`, `r3: DigitRow`, `lamps: u16`,
  `key_queue: heapless::Deque<Key, 8>` (or equivalent bounded buffer).
- The TUI must render the 5-bit relay codes as human-readable digits.
- No new keyboard bindings beyond the existing `Key` enum variants.

---

## 6. `ImuIo` Sub-Trait

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (CDUX/Y/Z at octal 32-34,
            PIPAX/Y/Z at octal 37-41, GYROCTR/GYROCMD at octal 47)
            Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc (IMUCOARS, IMUFINE,
            IMUZERO, SETCOARS; pages 1420-1432)
            Comanche055/T4RUPT_PROGRAM.agc (IMUMON, pages 139-143)
Module:     agc-core/src/hal/imu.rs
Channels:   CHAN12 (octal 12) for IMU control discretes
            CHAN14 (octal 14) for ISS CDU pulse commands
            CHAN30 (octal 30) for IMU status bits (input)
```

### Behavior Summary

The IMU (Inertial Measurement Unit, also called the ISS) has three operational
modes managed by the software:

1. **Unaligned** (zeroed/caged): IMU powered on but gyros not commanding stable member.
   CDU counters are zeroed. Channel 12 bits 4+6 control zero/coarse/error modes.

2. **Coarse Aligned** (IMUCOARS): CDU error counters enabled (channel 12 bit 6).
   The software iteratively drives COMMAND registers (CDU pulse commands via
   channel 14) to rotate gimbals to the desired THETAD orientation.
   Tolerance: within 2 degrees (COARSTOL = -0.01111 half-revolutions,
   IMU_MODE_SWITCHING_ROUTINES.agc page 1425).

3. **Fine Aligned** (IMUFINE): Zero and coarse discrete bits cleared. DAP enabled.
   Gyro torque commands (GYROCMD, octal 47) available for fine alignment.

The T4RUPT IMUMON routine (480 ms period) monitors channel 30 bits 9/11-15 and
invokes subroutines on changes. IMU fail (bit 13) and CDU fail (bit 12) trigger
warnings.

PIPA (Pulse Integrating Pendulous Accelerometer) counters — PIPAX (37), PIPAY (40),
PIPAZ (41) — are read once every 2 seconds by the SERVICER. Each count = 0.0585
m/s (PIPA_SCALE, docs/agc-reference-constants.md).

### Trait Definition

```rust
/// IMU I/O interface.
///
/// Provides access to CDU angles, PIPA delta-V accumulators, and gyro torque.
/// The bare-metal implementation enforces alignment state via type parameters
/// on the concrete struct (ImuImpl<Unaligned>, ImuImpl<CoarseAligned>,
/// ImuImpl<FineAligned>).
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (CDUX-Z octal 32-34,
///             PIPAX-Z octal 37-41, GYROCMD octal 47).
///             Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc.
pub trait ImuIo {
    /// Read the three IMU CDU gimbal angles (inner/middle/outer axes X/Y/Z).
    ///
    /// Returns [CduAngle; 3] = [CDUX, CDUY, CDUZ].
    /// CDUX = octal 32, CDUY = octal 33, CDUZ = octal 34.
    /// The middle gimbal angle (CDUZ) is monitored for gimbal lock (|angle| > 70°).
    fn read_cdu(&self) -> [CduAngle; 3];

    /// Read and clear the PIPA delta-V counters since the last call.
    ///
    /// Returns [i16; 3] = [PIPAX, PIPAY, PIPAZ], in raw pulse counts.
    /// Scale: 1 count = PIPA_SCALE = 0.0585 m/s (from SERVICER207.agc KPIP1).
    /// PIPAX = octal 37, PIPAY = octal 40, PIPAZ = octal 41.
    ///
    /// Reading clears the accumulators. Must be called from SERVICER (2 s cycle).
    fn read_pipa(&mut self) -> [i16; 3];

    /// Send gyro torque pulses for fine alignment (GYROCMD, octal 47).
    ///
    /// `axis`: 0=X, 1=Y, 2=Z.
    /// `pulses`: signed count of torque pulses to issue. Positive = one direction.
    ///
    /// Only valid when the IMU is in FineAligned state; the bare-metal impl
    /// enforces this via typestate. The trait exposes this method for all states
    /// to allow the abstract interface to be used; callers must ensure alignment
    /// via the type system.
    fn torque_gyro(&mut self, axis: usize, pulses: i16);

    /// Read IMU status bits from channel 30.
    ///
    /// Bit 15: temp in limits, Bit 14: ISS turn-on request,
    /// Bit 13: IMU fail, Bit 12: CDU fail, Bit 11: IMU cage, Bit 9: IMU operate.
    /// Used by IMUMON in T4RUPT_PROGRAM.agc.
    fn read_status(&self) -> u16;

    /// Write IMU control discrete bits to channel 12.
    ///
    /// Bit 4: coarse align enable. Bit 5: ISS CDU zero. Bit 6: error counter enable.
    /// Bit 10: gyro activity inhibit. Bit 15: ISS delay complete.
    /// Used by SETCOARS, IMUZERO, IMUFINE routines.
    fn write_control(&mut self, bits: u16);

    /// Write ISS CDU pulse commands to channel 14 (CDUXCMD/CDUYCMD/CDUZCMD).
    ///
    /// Used during coarse alignment to drive the gimbals to the desired angle.
    fn write_cdu_commands(&mut self, cmds: [i16; 3]);
}
```

### Typestate on Bare-Metal Implementation

The concrete bare-metal struct (not the trait) uses typestate:

```rust
pub struct ImuImpl<State> {
    /* SPI peripheral, GPIO pins, etc. */
    _state: core::marker::PhantomData<State>,
}

impl ImuImpl<Unaligned> {
    pub fn new(/* peripheral handles */) -> Self;
    /// Begin coarse alignment sequence (calls SETCOARS sequence).
    pub fn into_coarse_aligned(self) -> ImuImpl<CoarseAligned>;
    pub fn free(self) -> /* raw peripheral */;
}

impl ImuImpl<CoarseAligned> {
    /// Transition to fine alignment (clears zero and coarse bits).
    pub fn into_fine_aligned(self) -> ImuImpl<FineAligned>;
    pub fn free(self) -> /* raw peripheral */;
}

impl ImuImpl<FineAligned> {
    /// Revert to unaligned (e.g., on IMU fail).
    pub fn into_unaligned(self) -> ImuImpl<Unaligned>;
    pub fn free(self) -> /* raw peripheral */;
}

// All three states implement ImuIo
impl ImuIo for ImuImpl<Unaligned> { /* read_cdu, read_pipa, torque_gyro, ... */ }
impl ImuIo for ImuImpl<CoarseAligned> { /* ... */ }
impl ImuIo for ImuImpl<FineAligned> { /* ... */ }
```

### C-FREE

Each `ImuImpl<State>` exposes `free(self) -> RawPeripheral` that consumes the
wrapper and returns the raw SPI peripheral, allowing it to be passed to another
driver.

### C-HAL-TRAITS

`ImuImpl<State>` should implement `embedded_hal::spi::SpiDevice` for the
SPI bus that connects to the IMU electronics.

### agc-sim Impact

- `SimHardware` holds `SimImu { cdus: [CduAngle; 3], pipas: [i16; 3], status: u16 }`.
- PIPA counts are updated by the physics model each 2-second cycle.
- CDU angles are updated by the attitude dynamics model continuously.
- The TUI Mission State panel displays CDU angles as degrees (X/Y/Z gimbal).
- The `aligned` state of the sim IMU is tracked as a plain enum (no typestate
  needed in the sim; typestate is only for the bare-metal impl).

---

## 7. `OpticsIo` Sub-Trait

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
            CDUT (octal 35, optics trunnion CDU = OPTY)
            CDUS (octal 36, optics shaft CDU = OPTX)
            CDUTCMD (octal 53, trunnion command = OPTYCMD)
            CDUSCMD (octal 54, shaft command = OPTXCMD)
Module:     agc-core/src/hal/optics.rs
```

### Behavior Summary

The CM optics (sextant and telescope) have two CDU axes: shaft (azimuth rotation
of the optics assembly) and trunnion (elevation tilt of the star tracker).
The software reads current positions and writes incremental commands to drive
the optics to a desired pointing direction for star sightings.

In TVC mode (SPS burn), CDUTCMD (octal 53) is aliased as TVCYAW and CDUSCMD
(octal 54) as TVCPITCH — the same register locations are reused for engine gimbal
trim commands. This aliasing is a hardware-level multiplexing decision and must
be reflected in the implementation.

### Trait Definition

```rust
/// Optics shaft/trunnion CDU interface.
///
/// Provides position readback and incremental drive commands for
/// the sextant/telescope optics assembly.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
/// CDUT = octal 35 (trunnion), CDUS = octal 36 (shaft).
/// CDUTCMD = octal 53, CDUSCMD = octal 54.
pub trait OpticsIo {
    /// Read current shaft CDU angle (CDUS, octal 36).
    fn read_shaft(&self) -> CduAngle;

    /// Read current trunnion CDU angle (CDUT, octal 35).
    fn read_trunnion(&self) -> CduAngle;

    /// Command an incremental shaft drive (CDUSCMD, octal 54).
    /// Units: raw CDU pulse count; positive = one direction.
    fn drive_shaft(&mut self, delta: i16);

    /// Command an incremental trunnion drive (CDUTCMD, octal 53).
    /// Units: raw CDU pulse count.
    fn drive_trunnion(&mut self, delta: i16);
}
```

Note: The TVC gimbal trim commands (TVCPITCH, TVCYAW) that share the CDU command
registers are issued through `EngineIo::trim_gimbal`, not through `OpticsIo`.
The hardware implementation must multiplex appropriately; the software must
not call both simultaneously.

### C-FREE / C-HAL-TRAITS

`OpticsImpl` must expose `free(self)` to release the underlying peripheral.
It should implement the applicable `embedded-hal` GPIO or SPI trait for the
CDU drive electronics.

### agc-sim Impact

- `SimHardware` holds `SimOptics { shaft: CduAngle, trunnion: CduAngle }`.
- No TUI panel dedicated to optics for Milestone 1; add a status line in
  Mission State showing shaft/trunnion in degrees if needed for P51/P52 testing.

---

## 8. `EngineIo` Sub-Trait

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
            EMSD (octal 55, engine on/off discrete = THRUST)
            CDUTCMD (octal 53) = TVCYAW (SPS yaw gimbal)
            CDUSCMD (octal 54) = TVCPITCH (SPS pitch gimbal)
            CHAN13 (octal 13) for SPS/TVC discrete outputs
Module:     agc-core/src/hal/engine.rs
```

### Behavior Summary

The SPS (Service Propulsion System) main engine is commanded by:
- An engine on/off discrete (EMSD/THRUST register, octal 55), which gates
  the bipropellant valves.
- Gimbal trim commands (TVCPITCH, TVCYAW) written to the CDU command registers
  (octal 54 and 53) during TVC mode. These drive the engine gimbal actuators
  to steer thrust vector.

Channel 13 (octal 13) carries SPS/TVC discrete outputs. Channel 14 carries ISS
CDU pulse enables; CHAN14 bit 13/14/15 are used for SENDPULS in coarse align
(IMU_MODE_SWITCHING_ROUTINES.agc SENDPULS label).

SPS thrust: 91,188.544 N (from docs/agc-reference-constants.md).
SPS Ve: 3151.0396 m/s.

### Trait Definition

```rust
/// SPS main engine I/O interface.
///
/// Provides engine enable/disable and gimbal trim commands.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
/// EMSD/THRUST = octal 55 (engine on/off).
/// TVCPITCH = CDUSCMD = octal 54 (SPS pitch gimbal).
/// TVCYAW = CDUTCMD = octal 53 (SPS yaw gimbal).
pub trait EngineIo {
    /// Enable (fire) or disable the SPS engine.
    /// Corresponds to setting/clearing EMSD (octal 55).
    fn set_engine_enable(&mut self, enabled: bool);

    /// True if the engine is currently commanded on.
    fn engine_enabled(&self) -> bool;

    /// Command SPS gimbal trim in pitch axis.
    /// Units: CDU pulse counts (signed i16). Positive = pitch up.
    /// Writes to TVCPITCH = CDUSCMD (octal 54).
    fn trim_pitch(&mut self, delta: i16);

    /// Command SPS gimbal trim in yaw axis.
    /// Units: CDU pulse counts (signed i16). Positive = yaw right.
    /// Writes to TVCYAW = CDUTCMD (octal 53).
    fn trim_yaw(&mut self, delta: i16);

    /// Read back the current pitch gimbal position estimate.
    fn read_tvc_pitch(&self) -> CduAngle;

    /// Read back the current yaw gimbal position estimate.
    fn read_tvc_yaw(&self) -> CduAngle;
}
```

### C-FREE / C-HAL-TRAITS

`EngineImpl` must expose `free(self)`. It should implement the applicable
`embedded-hal` GPIO trait for the engine arm/fire discrete.

### agc-sim Impact

- `SimHardware` holds `SimEngine { enabled: bool, tvc_pitch: CduAngle, tvc_yaw: CduAngle }`.
- When `set_engine_enable(true)` is called in the sim, the physics model begins
  applying SPS thrust at `SPS_THRUST_N = 91188.544 N` with `SPS_VE_MS = 3151.0 m/s`.
- The Mission State panel shows ENGINE ON/OFF status and TVC angles.

---

## 9. `RcsIo` Sub-Trait

```
AGC source: Comanche055/JET_SELECTION_LOGIC.agc (JETSLECT, T6START, pages 1039-1062)
            Comanche055/ERASABLE_ASSIGNMENTS.agc
            PYJETS = channel 5 (octal 05) — pitch and yaw jet commands
            ROLLJETS = channel 6 (octal 06) — roll jet commands
Module:     agc-core/src/hal/rcs.rs
Channels:   PYJETS (octal 05), ROLLJETS (octal 06)
```

### Behavior Summary

The CSM has 16 RCS (Reaction Control System) jets arranged in four quads (A, B,
C, D), each quad having 4 jets. The jets fire in two axis groupings:

- **Channel 5 (PYJETS)**: pitch + yaw jet commands. Pitch jets: bits 1-4 (PWORD).
  Yaw jets: bits 5-8 (YWORD). Total: 8 jets addressable via this channel.
- **Channel 6 (ROLLJETS)**: roll jet commands. AC-roll jets: bits 1-5 (RWORD,
  mask ACRJETS = 03760 octal). BD-roll jets: bits 1-5 (mask BDRJETS = 34017 octal).

From JET_SELECTION_LOGIC.agc T6START label (page 1061): RWORD1 is written to
CHAN6; PWORD1 + YWORD1 is written to CHAN5.

The JET_SELECTION_LOGIC computes:
- PWORD1/PWORD2: pitch jet selection words
- YWORD1/YWORD2: yaw jet selection words
- RWORD1/RWORD2: roll jet selection words

These words encode which jets fire and for how long (jet on-time BLAST/BLAST1/BLAST2
in centiseconds, minimum 14 ms from `=14MS` constant). The `RcsIo` trait operates
at the channel-word level, not the individual-jet level; the selection logic
above it computes the correct channel words.

The 16 jets are logically grouped as:
- Quad A (AC pitch/roll): jets A1-A4
- Quad B (BD roll/yaw): jets B1-B4
- Quad C (AC pitch/roll redundant): jets C1-C4
- Quad D (BD roll/yaw redundant): jets D1-D4

Each jet is a 1-bit on/off command within the channel word. From the code
comments in JET_SELECTION_LOGIC.agc, ACRJETS mask = 03760 and BDRJETS = 34017,
PJETS = 01417, YJETS = 06360.

### `JetCommand` Type

```rust
/// A pair of jet channel words for one T6 interval.
///
/// Sent simultaneously to the two RCS output channels.
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc (T6START, page 1061).
/// Channel 5 (PYJETS, octal 05): pitch and yaw jets.
/// Channel 6 (ROLLJETS, octal 06): roll jets.
#[derive(Clone, Copy, Debug, Default)]
pub struct JetCommand {
    /// Channel 5 word: bits encode pitch (PWORD) and yaw (YWORD) jets.
    pub pitch_yaw: u16,
    /// Channel 6 word: bits encode roll jets (RWORD, AC and BD quads).
    pub roll: u16,
}

impl JetCommand {
    pub const OFF: Self = Self { pitch_yaw: 0, roll: 0 };
}
```

### Trait Definition

```rust
/// RCS jet I/O interface.
///
/// Issues 16-jet on/off commands via the two RCS output channels.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc.
/// Channels:   PYJETS (octal 05) — pitch+yaw jets.
///             ROLLJETS (octal 06) — roll jets.
pub trait RcsIo {
    /// Issue a jet command word pair.
    ///
    /// Writes `cmd.pitch_yaw` to channel PYJETS (05) and `cmd.roll` to
    /// channel ROLLJETS (06). The command takes effect immediately and
    /// persists until overwritten by the next call (typically the next
    /// T6RUPT cycle, 14 ms minimum pulse width).
    fn fire_jets(&mut self, cmd: JetCommand);

    /// Turn off all jets (write 0 to both channels).
    fn all_jets_off(&mut self);

    /// Read back the current jet command state (what was last written).
    fn current_command(&self) -> JetCommand;
}
```

### C-FREE / C-HAL-TRAITS

`RcsImpl` must expose `free(self)`. It should implement applicable `embedded-hal`
digital output traits for the jet driver lines.

### agc-sim Impact

- `SimHardware` holds `SimRcs { command: JetCommand }`.
- The physics model reads the current `JetCommand` every T5 cycle (20-100 ms)
  to compute torques about the vehicle's three axes.
- The Mission State panel does not render individual jet states for Milestone 1;
  a single indicator ("RCS ACTIVE") is sufficient.

---

## 10. `Timers` Sub-Trait

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
            TIME3 (octal 26) — Waitlist timer (T3RUPT)
            TIME4 (octal 27) — Periodic I/O timer (T4RUPT)
            TIME5 (octal 30) — DAP timer (T5RUPT)
            TIME6 (octal 31) — Jet timing timer (T6RUPT)
            T4RUPT_PROGRAM.agc: SETTIME4, 20MRUPT = OCT 37776 (16382 decimal)
            JET_SELECTION_LOGIC.agc: DELTATT3 = 16378 (60 ms), DELATT20 = 16382 (20 ms)
Module:     agc-core/src/hal/timers.rs
```

### Behavior Summary

The AGC has four programmable countdown timers. Each timer counts up from a
loaded value toward 0 (in ones-complement, actually toward the overflow). When
the timer overflows, the corresponding interrupt fires.

The Rust port maps each AGC timer to a hardware MCU timer peripheral (e.g.,
STM32 TIM3/TIM4/TIM5/TIM6) configured to interrupt at the appropriate period.

Timer periods:
- T3RUPT (TIME3): fires on Waitlist demand; period is set dynamically by WAITLIST.
  Resolution: 1 centisecond (10 ms tick rate).
- T4RUPT (TIME4): set to 20MRUPT = OCT 37776 = 16382, which in the AGC timer
  scheme corresponds to approximately 120 ms. Reloaded each T4RUPT cycle by
  SETTIME4 in T4RUPT_PROGRAM.agc.
- T5RUPT (TIME5): set by the DAP to 60 ms (DELTATT3 = 16378) or 20 ms
  (DELATT20 = 16382) depending on jet activity.
- T6RUPT (TIME6): set to approximately 14 ms at the start of a jet firing
  sequence (=14MS constant in JET_SELECTION_LOGIC.agc).

### Trait Definition

```rust
/// Hardware timer control interface for AGC interrupt scheduling.
///
/// Maps to TIME3/TIME4/TIME5/TIME6 in the AGC hardware.
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (octal 26-31).
///             Comanche055/T4RUPT_PROGRAM.agc (SETTIME4).
///             Comanche055/JET_SELECTION_LOGIC.agc (DELTATT3, DELATT20, =14MS).
pub trait Timers {
    /// Set the T3 (Waitlist) timer period.
    ///
    /// `centiseconds`: time in centiseconds until T3RUPT fires.
    /// Range: 1..=32767 (fitting the AGC TIME3 15-bit register).
    fn set_t3(&mut self, centiseconds: u16);

    /// Set the T4 (periodic I/O) timer period in centiseconds.
    /// Nominal: 12 cs (120 ms). Reset each T4RUPT cycle.
    fn set_t4(&mut self, centiseconds: u16);

    /// Set the T5 (DAP) timer period in centiseconds.
    /// DAP uses 2 cs (20 ms) or 6 cs (60 ms) depending on activity.
    fn set_t5(&mut self, centiseconds: u16);

    /// Set the T6 (jet timing) timer period in centiseconds.
    /// Minimum: approximately 1-2 cs (14 ms pulse).
    fn set_t6(&mut self, centiseconds: u16);

    /// Read the current T3 count (remaining centiseconds).
    fn read_t3(&self) -> u16;

    /// Read the current T4 count.
    fn read_t4(&self) -> u16;

    /// Read the current T5 count.
    fn read_t5(&self) -> u16;

    /// Read the current T6 count.
    fn read_t6(&self) -> u16;

    /// Disable T3 interrupt (INHINT equivalent for Waitlist timer).
    fn disable_t3(&mut self);

    /// Enable T3 interrupt (RELINT equivalent).
    fn enable_t3(&mut self);
}
```

### C-FREE / C-HAL-TRAITS

`TimersImpl` must expose `free(self)` returning the constituent MCU timer
peripheral handles. It must implement the applicable `embedded-hal` timer traits
(`CountDown`, `Periodic`) for each timer.

### agc-sim Impact

- `SimHardware` simulates timers via `std::time::Instant` or a monotonic counter.
- T3 fires when the Waitlist's next task delta expires.
- T4 fires every 120 ms of simulated time.
- T5/T6 fire as scheduled by the DAP/RCS logic.
- The sim may optionally run at a fixed multiple of real time for testing.

---

## 11. `UplinkIo` Sub-Trait

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
            INLINK (octal 45) — uplink word input register
            CHAN33 bit 11 — uplink too fast (read in C33TEST, T4RUPT_PROGRAM.agc page 146)
Module:     agc-core/src/hal/uplink.rs
```

### Behavior Summary

Uplink words arrive from ground via the radio link. The AGC receives each word
via UPRUPT (interrupt 7). Each word is a 5-bit keyboard code equivalent that
drives PINBALL the same way a keyboard keystroke does (per the PINBALL header
comment, PINBALL_GAME_BUTTONS_AND_LIGHTS.agc page 307).

Channel 33 bit 11 signals "uplink too fast" if words arrive faster than the
AGC can process them; this is monitored by C33TEST in T4RUPT.

### Trait Definition

```rust
/// Uplink receiver I/O interface.
///
/// Provides access to uplink words received from ground via UPRUPT.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc, INLINK = octal 45.
///             Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (uplink = keyboard equivalent).
pub trait UplinkIo {
    /// Read the next uplink word (5-bit code), or `None` if none is pending.
    ///
    /// Corresponds to reading INLINK (octal 45) on UPRUPT.
    fn read_uplink_word(&mut self) -> Option<u8>;

    /// True if the "uplink too fast" flag is set (channel 33 bit 11).
    fn uplink_overrun(&self) -> bool;

    /// Clear the uplink overrun flag.
    fn clear_overrun(&mut self);
}
```

### C-FREE / C-HAL-TRAITS

`UplinkImpl` must expose `free(self)`. It should implement the applicable
`embedded-hal` UART or serial trait for the uplink receiver.

### agc-sim Impact

- `SimHardware` holds `uplink_queue: heapless::Deque<u8, 16>` or a similar
  statically-sized ring buffer.
- The `agc_sim` TUI or scenario runner pushes uplink words via a command
  interface for scripted scenario testing.
- The uplink path allows automated injection of V37N40 (P40 run command) and
  similar program invocations.

---

## 12. `TelemetryIo` Sub-Trait

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
            DNTM1 (octal 34) — downlink telemetry word 1
            DNTM2 (octal 35) — downlink telemetry word 2
            CHAN33 bit 12 — downlink too fast (read in C33TEST)
Module:     agc-core/src/hal/telemetry.rs
```

### Behavior Summary

The AGC sends housekeeping and navigation data to the ground via the PCM
downlink. Two output channels (DNTM1, DNTM2) accept 15-bit data words.
DOWNRUPT (interrupt 8) signals that the telemetry system is ready to accept
the next pair. Channel 33 bit 12 signals overrun.

### Trait Definition

```rust
/// PCM telemetry downlink interface.
///
/// Writes downlink word pairs for transmission to the ground.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc.
/// DNTM1 = octal 34, DNTM2 = octal 35.
pub trait TelemetryIo {
    /// Write a downlink word pair (called on DOWNRUPT).
    ///
    /// `word1`: data for DNTM1 (octal 34).
    /// `word2`: data for DNTM2 (octal 35).
    fn write_downlink_pair(&mut self, word1: u16, word2: u16);

    /// True if the downlink system has signaled "too fast" (channel 33 bit 12).
    fn downlink_overrun(&self) -> bool;

    /// Clear the overrun flag.
    fn clear_overrun(&mut self);
}
```

### C-FREE / C-HAL-TRAITS

`TelemetryImpl` must expose `free(self)`. It should implement the applicable
`embedded-hal` serial or SPI trait for the PCM encoder.

### agc-sim Impact

- `SimHardware` implements `TelemetryIo` by writing word pairs to a
  `heapless::Deque<(u16, u16), 64>` ring buffer.
- The `agc-sim` binary may optionally log telemetry words to stdout or a file
  for post-run analysis.
- For Milestone 1, telemetry output is captured but not decoded.

---

## 13. Invariants Across All Sub-Traits

1. **No blocking**: every HAL method must return promptly (microseconds). Long
   operations use interrupt-driven callbacks, not polling loops.
2. **No heap**: all HAL types must be statically allocated. No `Vec`, `Box`, or
   `String` in any HAL type, including the sim implementation.
3. **No `static mut`**: shared state between interrupt handlers and foreground
   code uses `cortex_m::interrupt::Mutex<RefCell<T>>`. Raw `static mut` is
   forbidden.
4. **C-FREE**: every non-`Copy` HAL wrapper exposes `free(self)` returning the
   raw peripheral. This is mandatory (not optional) for all bare-metal structs.
5. **C-HAL-TRAITS**: bare-metal structs implement all applicable `embedded-hal`
   v1 traits in addition to the custom sub-traits.
6. **C-PIN-STATE**: IMU alignment state is enforced via typestate type parameters
   on `ImuImpl<State>`. No runtime state variable for alignment.
7. **PAC re-export**: the `#[interrupt]` attribute used in ISR definitions must
   come from the device PAC crate's re-export, not from `cortex-m-rt` directly.

---

## 14. Test Cases

### `DskyIo`
1. Encode digit 7 as relay code: `0b10011` (from PINBALL table). Round-trip
   encode/decode: `decode(encode(7))` == 7.
2. Key code for ENTER: `Key::Enter as u8` == 0x1C (octal 34). Reading MNKEYIN
   value 0x1C from sim produces `Some(Key::Enter)`.
3. Lamp bits: `write_lamp_word(0x20)` sets bit 5 (KEY RELEASE) in sim state;
   `write_lamp_word(0x40)` sets bit 6 (VERB/NOUN FLASH).

### `ImuIo`
1. PIPA round-trip: writing [100i16, 0, 0] to sim PIPA state; calling
   `read_pipa()` returns [100, 0, 0] and clears accumulator to [0, 0, 0].
2. CDU read: sim CDUY = CduAngle(16384) (= 90°); `read_cdu()[1]` returns
   `CduAngle(16384)`; `.to_radians()` ≈ π/2.
3. Gimbal lock threshold: `CduAngle::from_radians(71.0_f64.to_radians()).counts()`
   should exceed the gimbal lock threshold (from GLOCKMON: -70DEGS = -0.38888
   half-revolutions ≈ 70° = approximately 12743 counts from center).

### `RcsIo`
1. All-off command: `all_jets_off()` writes (0, 0) to sim; `current_command()`
   returns `JetCommand::OFF`.
2. Pitch jet fire: `fire_jets(JetCommand { pitch_yaw: 0x0F, roll: 0 })` sets
   pitch bits; `current_command().pitch_yaw` == 0x0F.
3. Minimum pulse: a JetCommand must remain active for at least 14 ms (enforced
   by T6RUPT scheduling in jet_selection_logic; the HAL itself does not enforce
   duration).

### `Timers`
1. T4 period: `set_t4(12)` configures a 120 ms timer; `read_t4()` returns a
   value in [0, 12] after the call.
2. T6 minimum: `set_t6(2)` configures the jet pulse timer for approximately
   20 ms; a T6RUPT fires within 20 ms of simulated time.
3. T3 disable/enable round-trip: `disable_t3()` followed by `enable_t3()`
   should not fire a spurious T3RUPT in the sim.

---

## 15. Ambiguities and Open Questions

1. **PRO key channel**: The PRO (Proceed) button is read from channel 32 bit 14
   (PROCEEDE routine, T4RUPT_PROGRAM.agc page 136), not from the keyboard channel.
   It is not represented as a `Key` variant. The `DskyIo::proceed_pressed()` method
   covers this. Confirm with the architect whether PRO should be a separate
   discrete or unified under `read_key()`.

2. **Nav DSKY keyboard**: `NAVKEYIN` (channel 16) is defined in ERASABLE_ASSIGNMENTS.agc
   but its key codes and usage in Comanche055 are not detailed in the files read.
   The `read_nav_key()` method is included for completeness. If Comanche055 does
   not use a nav DSKY, this method can return `None` always.

3. **TVC register aliasing**: CDUTCMD (octal 53) = TVCYAW and CDUSCMD (octal 54) =
   TVCPITCH share register addresses with the optics CDU commands. The HAL design
   has `OpticsIo` and `EngineIo` as separate traits. The bare-metal implementation
   must ensure these are not called simultaneously; the software currently prevents
   this by the fact that TVC is only active during SPS burns when optics tracking
   is not in progress. Document this constraint explicitly in the implementation.

4. **GYROCTR vs GYROCMD**: ERASABLE_ASSIGNMENTS.agc shows `GYROCTR EQUALS 47` and
   then `GYROCMD EQUALS 47` — two names for the same register at octal 47. The
   Rust implementation should use `GYROCMD` as the canonical name (used in
   IMU_MODE_SWITCHING_ROUTINES.agc SETCOARS: `CS ZERO; TS GYROCMD`).

5. **16-jet numbering convention**: JET_SELECTION_LOGIC.agc describes jets in
   terms of quad membership (A/B/C/D) and PYTABLE/RTABLE/YZTABLE channel-word
   bits, not by a simple 1-16 index. The `JetCommand` type above uses the native
   channel-word encoding. If a higher-level API with 1-16 jet numbering is needed,
   it should live in `control/rcs_logic.rs`, not in the HAL.
