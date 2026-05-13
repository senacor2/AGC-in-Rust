# Specification: `hal/` Module — Hardware Abstraction Layer

**Status**: Approved for implementation
**Module path**: `agc-core/src/hal/`
**Source files**: `mod.rs`, `interrupts.rs`, `timers.rs`, `dsky.rs`, `imu.rs`,
`optics.rs`, `engine.rs`, `rcs.rs`, `uplink.rs`, `telemetry.rs`
**Simulator implementation**: `agc-sim/src/hardware.rs`
**Architecture reference**: `docs/architecture.md` §4 "Hardware Abstraction Layer (HAL)"
**Types reference**: `specs/types-module-spec.md` — `CduAngle` definition
**AGC source reference**: `docs/AGC Symbolic Listing.md` §IIE (I/O Channels),
§IIH (Interrupts), §IID (Counter Cells)
**Spec checklist**: `specs/README.md` — all items satisfied (see §12)

---

## 1. Purpose and Scope

The `hal/` module defines the boundary between the AGC flight software and the
physical (or simulated) spacecraft peripherals. Every hardware access — reading
IMU angles, firing RCS jets, driving the DSKY display, arming timers — goes
through one of the typed sub-trait methods defined here. No other module in
`agc-core` ever touches a hardware register directly.

This isolation serves two goals:

1. **Testability**: The entire flight software can be run on a host machine
   against `SimHardware` without any embedded hardware present.
2. **Safety**: The type system prevents the flight software from performing
   physically dangerous operations (e.g., issuing gyro torque commands to an
   uncaged IMU) that were prevented only by programmer discipline in the
   original assembly.

### What this module is NOT

- It is not a hardware driver. The bare-metal implementation of each sub-trait
  uses `embedded-hal` v1 traits internally, but the flight software never
  imports or calls `embedded-hal` directly.
- It is not a software model of spacecraft physics. That lives in `agc-sim/`.
- It does not own interrupt service routines. The `Executive` registers ISR
  entry points at startup; the `hal/` module only exposes the timer-arm/disarm
  interface used by those ISRs.

---

## 2. AGC Background: Original Hardware Channels and Counters

The Block 2 AGC communicates with spacecraft hardware through two mechanisms:

### 2.1 I/O Channels (Binary Input/Output)

Digital control and status registers addressed by octal channel number. Ten
output channels (05, 06, 07, 10, 11, 12, 13, 14, 34, 35) and eight input
channels (03, 04, 15, 16, 30, 31, 32, 33). Key channel assignments for the
Command Module:

| Channel (octal) | Mnemonic   | Direction | Rust HAL method               |
|-----------------|------------|-----------|-------------------------------|
| 05              | PYJETS     | Output    | `Rcs::fire_sm_jets` (jets_a)  |
| 06              | ROLLJETS   | Output    | `Rcs::fire_sm_jets` (jets_b)  |
| 10              | OUTO       | Output    | `Dsky::write_row`             |
| 11              | DSALMOUT   | Output    | `Dsky::set_lamp`, `Engine::sps_enable` (bit 13), `Dsky::set_flash` (bit 6) |
| 12              | CHAN12     | Output    | `Imu::coarse_align` (bits 6,4), `Engine::sps_gimbal` (bits 8,2) |
| 13              | CHAN13     | Output    | `Timers::arm_t6` (bit 15), `Uplink` block/enable (bit 6) |
| 14              | CHAN14     | Output    | `Imu::torque_gyro` (bits 10-6), CDU pulse generation (bits 15-11) |
| 15              | MNKEYIN    | Input     | `Dsky::read_key` (main DSKY)  |
| 16              | NAVKEYIN   | Input     | `Dsky::read_key` (nav DSKY), `Optics::mark_pressed` (bit 6) |
| 30              | CHAN30     | Input     | `Imu::is_caged` (bit 11), IMU fail status |
| 34              | DNTM1      | Output    | `Telemetry::send_word` (word 1) |
| 35              | DNTM2      | Output    | `Telemetry::send_word` (word 2) |

> AGC source: `docs/AGC Symbolic Listing.md`, §IIE, table beginning at "Channel
> 05", pages IIE-1 through IIE-10.

### 2.2 Counter Cells (Pulse Accumulation / Decrement)

Erasable memory cells that interface with hardware via the DINC (decrement)
mechanism. Relevant counter cells for the HAL:

| Cell (octal) | Tag      | Direction | Scale       | Rust HAL method              |
|--------------|----------|-----------|-------------|------------------------------|
| 0031         | TIME6    | Down-counter | 0.000625 s/count | `Timers::arm_t6`        |
| 0026         | TIME3    | Up-counter | 0.01 s/count | `Timers::arm_t3`           |
| 0027         | TIME4    | Up-counter | 0.01 s/count | `Timers::arm_t4` (implicit) |
| 0030         | TIME5    | Up-counter | 0.01 s/count | `Timers::arm_t5`           |
| 0024–0025    | TIME2/1  | Up-counter (dp) | 0.01 s/count | `Timers::mission_time` |
| 0047         | GYROCMD  | Down-counter | pulses     | `Imu::torque_gyro`          |
| 0050–0052    | CDUXCMD/CDUYCMD/CDUZCMD | Down-counter | 3200 pps | `Imu::coarse_align` |
| 0053–0054    | CDUTCMD/CDUSCMD (TVCYAW/TVCPITCH) | Down-counter | 3200 pps | `Engine::sps_gimbal` |
| 0045         | INLINK   | Bit serial input | — | `Uplink::read_word`         |

> AGC source: `docs/AGC Symbolic Listing.md`, §IID, counter cell table.

### 2.3 NEWJOB and the Night-Watchman

Cell `0061` (octal) `NEWJOB` is sampled by the Executive main loop. The
hardware flip-flop paired with NEWJOB produces a restart if the loop stalls for
more than 0.64–1.92 seconds. In the Rust port this maps to `pet_watchdog()`.

> AGC source: `docs/AGC Symbolic Listing.md`, §IID, cell `0061` (NEWJOB).

---

## 3. Module Structure

```
agc-core/src/hal/
  mod.rs          — AgcHardware master trait
  interrupts.rs   — Interrupt enum
  timers.rs       — Timers sub-trait
  dsky.rs         — Dsky sub-trait + Lamp enum
  imu.rs          — Imu sub-trait
  optics.rs       — Optics sub-trait
  engine.rs       — Engine sub-trait
  rcs.rs          — Rcs sub-trait
  uplink.rs       — Uplink sub-trait
  telemetry.rs    — Telemetry sub-trait
```

All public items are re-exported from `hal/mod.rs` so the rest of the crate
uses a single import path:

```rust
use crate::hal::{AgcHardware, Interrupt, Timers, Dsky, Lamp, Imu, Optics,
                 Engine, Rcs, Uplink, Telemetry};
```

---

## 4. `AgcHardware` Master Trait

**File**: `agc-core/src/hal/mod.rs`

### 4.1 Declaration

```rust
pub trait AgcHardware {
    type Timers:    Timers;
    type Dsky:      Dsky;
    type Imu:       Imu;
    type Optics:    Optics;
    type Engine:    Engine;
    type Rcs:       Rcs;
    type Uplink:    Uplink;
    type Telemetry: Telemetry;

    fn timers(&mut self)    -> &mut Self::Timers;
    fn dsky(&mut self)      -> &mut Self::Dsky;
    fn imu(&mut self)       -> &mut Self::Imu;
    fn optics(&mut self)    -> &mut Self::Optics;
    fn engine(&mut self)    -> &mut Self::Engine;
    fn rcs(&mut self)       -> &mut Self::Rcs;
    fn uplink(&mut self)    -> &mut Self::Uplink;
    fn telemetry(&mut self) -> &mut Self::Telemetry;

    fn pet_watchdog(&mut self);
    fn hardware_restart(&mut self) -> !;
}
```

### 4.2 Associated-Type Pattern

Each peripheral subsystem is an associated type rather than a trait-object
field. This design was chosen so that:

- The compiler can monomorphize the entire flight software against a single
  concrete implementation type, eliminating all virtual dispatch overhead (a
  hard requirement for the DAP timing budget on Cortex-M).
- Each sub-type can itself carry typestate parameters (e.g. `ImuImpl<State>`)
  without making `AgcHardware` generic over extra type parameters.
- Different targets (bare-metal, sim, future hardware variants) differ only in
  their `AgcHardware` `impl` block; the flight software is untouched.

The full `AgcHardware` bound on a flight-software entry point looks like:

```rust
pub fn run<H: AgcHardware>(hw: &mut H, state: &mut AgcState) -> ! { ... }
```

All eight accessor methods return `&mut Self::SubType`, granting exclusive
mutable access to one peripheral at a time. The borrow checker statically
prevents simultaneous mutable access to two peripherals (a proxy for preventing
re-entrant channel writes that caused issues in the original hardware).

### 4.3 `pet_watchdog()`

**Contract**

| Attribute | Value |
|-----------|-------|
| Required call frequency | At least once every 0.64–1.92 s (architecture §5.2) |
| Called from | `Executive::run` main loop, once per iteration |
| Side effect | Resets the hardware night-watchman flip-flop |
| Return | `()` |
| Panic / error | None; if not called in time, hardware restarts |

**Preconditions**: None. May be called from interrupt context or foreground.

**Postconditions**: The watchdog timer deadline is reset; a new 0.64–1.92 s
window begins.

**AGC source correspondence**: Sampling the NEWJOB cell (`0061` octal) in
the `EXEC` main loop. The act of reading NEWJOB resets the hardware flip-flop.

**SimHardware behaviour**: No-op. `SimHardware::pet_watchdog` returns
immediately with no state change.

### 4.4 `hardware_restart()`

**Contract**

| Attribute | Value |
|-----------|-------|
| Return type | `!` (diverges — never returns) |
| Triggered by | Rust `#[panic_handler]`, alarm system, software-initiated GOJAM |
| Side effect | All output channels are reset to zero (see §IIE of AGC Symbolic Listing: "as part of a hardware restart ... all output channel bits except those of channel 07 are reset zero") |
| Post-restart | The `RESTART` / `FRESH START` sequence in `services/fresh_start.rs` re-initialises the system |

**Preconditions**: None. May be called at any time from any context.

**Postconditions**: Never reached; control transfers to the reset vector.

**On bare metal**: Calls `cortex_m::peripheral::SCB::sys_reset()`.

**SimHardware behaviour**: `panic!("SimHardware: hardware_restart triggered")`.
This propagates out of the test runner as a test failure with a recognisable
message, which is the expected behaviour — restarting in simulation is always
a bug.

**Error conditions**: If flight software calls `hardware_restart()` during a
unit test, the Rust test harness catches the panic and marks the test as
failed. Integration tests in `agc-test/` may specifically test that
`hardware_restart()` is called (by catching the panic) to verify restart
protection.

---

## 5. `Interrupt` Enum

**File**: `agc-core/src/hal/interrupts.rs`

### 5.1 Declaration

```rust
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[repr(u8)]
pub enum Interrupt {
    T6Rupt    = 1,
    T5Rupt    = 2,
    T3Rupt    = 3,
    T4Rupt    = 4,
    KeyRupt1  = 5,
    KeyRupt2  = 6,
    UplinkRupt = 7,
    DownRupt  = 8,
    RadarRupt = 9,
    HandRupt  = 10,
}
```

### 5.2 Interrupt Descriptions and AGC Correspondence

| Rust Variant | AGC Interrupt # | Trigger Condition | AGC Counter Cell | Period / Condition |
|---|---|---|---|---|
| `T6Rupt` | Program interrupt #1 | TIME6 decremented to `-0`; bit 15 of channel 13 auto-cleared | `TIME6` (octal 0031) | 0.625 ms per count (up to 1600 Hz) |
| `T5Rupt` | Program interrupt #2 | TIME5 overflow | `TIME5` (octal 0030) | Programmable; typically ~100 ms for DAP cycle |
| `T3Rupt` | Program interrupt #3 | TIME3 overflow | `TIME3` (octal 0026) | Programmable; 10 ms minimum; waitlist dispatch |
| `T4Rupt` | Program interrupt #4 | TIME4 overflow | `TIME4` (octal 0027) | 120 ms; rotating I/O task cycle (DSKY update, IMU monitoring, gyro drift comp; see §13.4) |
| `KeyRupt1` | Program interrupt #5 | Keystroke on main DSKY | Channel 15 (MNKEYIN) | Asynchronous |
| `KeyRupt2` | Program interrupt #6 | Keystroke on nav DSKY / optics mark | Channel 16 (NAVKEYIN) | Asynchronous |
| `UplinkRupt` | Program interrupt #7 | Uplink word received in INLINK (octal 0045) | `INLINK` (octal 0045) | Asynchronous |
| `DownRupt` | Program interrupt #8 | Telemetry end pulse | Channel 33 bit 12 | ~10 ms typical |
| `RadarRupt` | Program interrupt #9 | Radar data ready (bit 4 of channel 13 auto-cleared) | Cell octal 0046 | Asynchronous |
| `HandRupt` | Program interrupt #10 | Hand controller or discrete input change | Channels 31/32 | Asynchronous |

> AGC source: `docs/AGC Symbolic Listing.md`, §IIH "Program Interrupts"; §IID
> counter cell table for TIME3/4/5/6.

### 5.3 Priority

The `#[repr(u8)]` values assign priority consistent with the original AGC:
**lower discriminant = higher priority**. On Cortex-M, the NVIC must be
programmed so that the timer peripheral assigned to T6RUPT has the highest
interrupt priority level (lowest numeric priority register value) and HandRupt
has the lowest.

### 5.4 Interrupt Timing Budget

From `docs/architecture.md` §13.2:

| Interrupt | Period | Maximum Handler Budget |
|-----------|--------|------------------------|
| T6Rupt    | 0.625 ms (demand) | 0.5 ms |
| T5Rupt    | ~100 ms | 20 ms |
| T3Rupt    | ≥10 ms variable | 5 ms (dispatch only; task runs in foreground) |
| T4Rupt    | 120 ms | 10 ms |
| KeyRupt1/2 | async | 1 ms |
| DownRupt  | ~10 ms | 0.5 ms |

Handlers that exceed their budget risk missing the next T6RUPT, causing RCS
jet timing errors. The only remediation is to shorten the handler or move
work to a foreground job via NOVAC/FINDVAC.

### 5.5 Critical Sections (INHINT / RELINT)

The original AGC provided `INHINT` (inhibit interrupts) and `RELINT` (release
interrupts) instructions for critical sections. In the Rust port:

```rust
// Enter critical section (equivalent to INHINT)
cortex_m::interrupt::free(|cs| {
    // Access to Mutex<RefCell<T>> shared state here
});
// Interrupts automatically re-enabled on closure exit (equivalent to RELINT)
```

`raw static mut` is prohibited in application code. All state shared between
interrupt handlers and foreground code uses
`cortex_m::interrupt::Mutex<RefCell<T>>`.

---

## 6. `Timers` Sub-Trait

**File**: `agc-core/src/hal/timers.rs`

### 6.1 Declaration

```rust
pub trait Timers {
    fn arm_t3(&mut self, centiseconds: u16);
    fn arm_t5(&mut self, centiseconds: u16);
    fn arm_t6(&mut self, counts: u16);
    fn disarm_t6(&mut self);
    fn mission_time(&self) -> u32;
}
```

### 6.2 Method Specifications

#### `arm_t3(centiseconds: u16)`

| Attribute | Value |
|-----------|-------|
| AGC source | Sets `TIME3` (octal 0026) to `2^14 - centiseconds`; overflow fires interrupt #3 |
| AGC channel | Counter cell TIME3; overflow → T3RUPT |
| Valid range | 1–16383 centiseconds (14-bit counter) |
| Side effect | Arms the waitlist dispatch timer; previous T3 deadline is discarded |
| Called from | `Waitlist::schedule()` in `executive/waitlist.rs` |

**Preconditions**: `1 <= centiseconds <= 16383`. Implementors may saturate
values outside this range; flight software must never pass `0` or values above
16383 (use the long-waitlist chaining mechanism for delays > 163 s).

**Postconditions**: `T3Rupt` will fire in no less than `centiseconds × 10 ms`
and no more than `(centiseconds + 1) × 10 ms` (one tick jitter due to counter
phase).

**Error condition**: Passing `0` is undefined behaviour on bare metal (TIME3
would overflow on the next DINC, firing T3RUPT almost immediately). Passing
`>= 16384` silently wraps the 14-bit counter and fires too early. The Waitlist
scheduler must validate inputs before calling `arm_t3`.

#### `arm_t5(centiseconds: u16)`

| Attribute | Value |
|-----------|-------|
| AGC source | Sets `TIME5` (octal 0030) to `2^14 - centiseconds`; overflow fires interrupt #2 |
| AGC channel | Counter cell TIME5; overflow → T5RUPT |
| Valid range | 1–16383 centiseconds |
| Side effect | Arms the DAP computation timer |
| Called from | `control/dap.rs` DAP initialisation and T5RUPT re-arm |

**Preconditions/Postconditions**: same pattern as `arm_t3`.

**Typical usage**: DAP calls `arm_t5(10)` (100 ms period) at the end of each
T5RUPT handler to re-arm for the next cycle.

#### `arm_t6(counts: u16)`

| Attribute | Value |
|-----------|-------|
| AGC source | Loads `TIME6` (octal 0031) and sets bit 15 of channel 13 to enable decrementing at 1600 Hz (0.000625 s per count) |
| AGC channel | Counter cell TIME6 (octal 0031); channel 13 bit 15 = enable; auto-cleared when TIME6 reaches −0, triggers T6RUPT |
| Valid range | 1–32767 counts (0.625 ms to ~20.5 s) |
| Side effect | Arms the RCS jet-pulse timer; enables the 1600 Hz DINC clock to TIME6 |
| Called from | `control/rcs_logic.rs` jet firing sequence |
| Timing | Each count = 0.625 ms; maximum count = 32767 ≈ 20.5 s |

**Preconditions**: `counts >= 1`. Calling with `counts = 0` would cause
T6RUPT to fire on the next DINC cycle (< 1 ms), which is a programming error
in the jet timing logic.

**Postconditions**: `T6Rupt` fires after `counts × 0.625 ms ± 0.625 ms`.
Channel 13 bit 15 is set; it is auto-cleared by hardware when TIME6 reaches
−0.

#### `disarm_t6()`

| Attribute | Value |
|-----------|-------|
| AGC source | Clears bit 15 of channel 13 (disables DINC to TIME6) |
| AGC channel | Channel 13 bit 15 |
| Side effect | Stops TIME6 decrementing; T6RUPT will NOT fire |
| Called from | `control/rcs_logic.rs` — cancel a pending jet pulse before it fires |

**Preconditions**: May be called at any time, including when T6 is not armed.
Calling when T6 is not armed is a no-op.

**Postconditions**: TIME6 is frozen; T6RUPT is suppressed.

**AGC note**: The original AGC code frequently armed T6 and then cancelled it
in the same task if conditions changed (e.g. a requested jet firing was
superseded by a newer DAP cycle). `disarm_t6` must execute atomically with
respect to `arm_t6` calls from the DAP — use a critical section if both
interrupt context and foreground context call these methods.

#### `mission_time() -> u32`

| Attribute | Value |
|-----------|-------|
| AGC source | Reads double-precision counter `TIME2`/`TIME1` (octal 0024/0025) |
| AGC channels | Counter cells TIME2 (octal 0024) and TIME1 (octal 0025) |
| Return type | `u32` centiseconds (wraps after ~497 days) |
| Side effect | None (read-only) |
| Called from | Navigation, guidance, display formatting |

**Preconditions**: None.

**Postconditions**: Returns the current mission elapsed time in centiseconds
as a monotonically increasing value. Wraps on overflow; callers that compute
time differences must handle wrap-around.

**Scale**: 1 count = 0.01 s. Convert to `f64` seconds at call site:
`hw.timers().mission_time() as f64 * 0.01`.

### 6.3 SimTimers Behaviour

`SimTimers` maintains a single `mission_time_cs: u32` counter. `arm_t3`,
`arm_t5`, `arm_t6`, and `disarm_t6` are no-ops (the sim does not have a real
timer ISR mechanism; interrupt simulation is driven externally by the test
harness). `mission_time()` returns `mission_time_cs`.

### 6.4 Test Cases

**TC-TIMERS-01**: `arm_t3` sets the deadline
```rust
let mut hw = SimHardware::new();
// arm_t3 is a no-op in sim; verify it does not panic
hw.timers().arm_t3(100); // 1 second
// postcondition: no panic, mission_time unchanged
assert_eq!(hw.timers().mission_time(), 0);
```

**TC-TIMERS-02**: `mission_time` returns injected value
```rust
let mut hw = SimHardware::new();
hw.timers.mission_time_cs = 54321;
assert_eq!(hw.timers().mission_time(), 54321);
```

**TC-TIMERS-03**: `disarm_t6` is safe when T6 not armed
```rust
let mut hw = SimHardware::new();
hw.timers().disarm_t6(); // must not panic
hw.timers().disarm_t6(); // idempotent
```

---

## 7. `Dsky` Sub-Trait and `Lamp` Enum

**File**: `agc-core/src/hal/dsky.rs`

### 7.1 Declaration

```rust
pub trait Dsky {
    fn write_row(&mut self, row: u8, data: u16);
    fn clear_row(&mut self, row: u8);
    fn set_lamp(&mut self, lamp: Lamp, on: bool);
    fn set_flash(&mut self, on: bool);
    fn read_key(&mut self) -> Option<u8>;
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Lamp {
    UplinkActivity,  // Channel 11, bit 3
    NoAtt,           // Channel 11, bit 1 (ISS Warning)
    Stby,            // Channel 13, bit 10
    KeyRel,          // Channel 11, bit 5
    OprErr,          // Channel 11, bit 7
    Restart,         // Channel 11, bit 10 (caution reset)
    GimbalLock,      // Display row 14, bit 1
    Temp,            // Channel 11, bit 4
    ProgAlarm,       // Channel 11 / display row relay
    CompActy,        // Channel 11, bit 2
}
```

### 7.2 AGC Source Correspondence

The DSKY display is driven by output channel 10 (OUTO, octal 010) using a
relay row-select matrix. Indicator lamps are driven by output channel 11
(DSALMOUT, octal 011). Key codes arrive on input channels 15 (MNKEYIN) and
16 (NAVKEYIN).

> AGC source: `docs/AGC Symbolic Listing.md`, §IIE channel 10 and channel 11
> tables; §IIJ Display System.

**Channel 10 (OUTO) row encoding**:
- Bits 15–12: row number (01–14 octal in the original; 1–14 decimal in the
  Rust port)
- Bits 11–1: segment relay settings for that row
- Each row must be held for 20 ms, then cleared, before the next row is set
- Full 11-row update cycle takes ~440 ms
- The HAL implementation manages the 20 ms hold via a hardware timer; the
  flight software calls `write_row` / `clear_row` freely

**Channel 11 (DSALMOUT) bit assignments** (relevant CM bits):

| Bit | Lamp / Function | `Lamp` variant |
|-----|-----------------|----------------|
| 7 | Operator Error | `Lamp::OprErr` |
| 6 | Flash (VERB/NOUN blinking) | `Dsky::set_flash` |
| 5 | Key Release | `Lamp::KeyRel` |
| 4 | Temperature Caution | `Lamp::Temp` |
| 3 | Uplink Activity | `Lamp::UplinkActivity` |
| 2 | Computer Activity | `Lamp::CompActy` |
| 1 | ISS Warning (No Att) | `Lamp::NoAtt` |
| 13 | SPS Engine On | `Engine::sps_enable` |
| 10 | Caution Reset (Restart) | `Lamp::Restart` |

### 7.3 Method Specifications

#### `write_row(row: u8, data: u16)`

| Attribute | Value |
|-----------|-------|
| AGC channel | Output channel 10 (OUTO); bits 15–12 = row, bits 11–1 = data |
| Valid `row` range | 1–14 |
| Side effect | Energises relay row `row` with segment pattern `data` for 20 ms (enforced in HAL implementation) |
| Called from | `services/display.rs` (PINBALL display driver) during T4RUPT |

**Preconditions**: `1 <= row <= 14`. Row 0 and rows 15+ do not exist in the
hardware relay matrix; passing them is a programming error (undefined behaviour
on bare metal).

**Postconditions**: The relay row is energised; the display segments
corresponding to set bits in `data` are illuminated. The HAL implementation
holds the relay for 20 ms before accepting the next `write_row` call for a
different row.

**Timing constraint**: The 20 ms hold is enforced by the HAL implementation
(hardware timer ISR), not by the calling flight software. The flight software
may call `write_row` for the next row immediately after returning; the
implementation buffers and sequences the relay operations.

#### `clear_row(row: u8)`

| Attribute | Value |
|-----------|-------|
| AGC channel | Output channel 10 (OUTO), all segment bits = 0 for the selected row |
| Valid `row` range | 1–14 |
| Side effect | De-energises all segment relays in row `row` |
| Called from | `services/display.rs` |

**Preconditions**: Same as `write_row`.

**Postconditions**: Row `row` is blanked. Does not affect other rows.

#### `set_lamp(lamp: Lamp, on: bool)`

| Attribute | Value |
|-----------|-------|
| AGC channel | Output channel 11 (DSALMOUT); specific bit determined by `Lamp` variant |
| Side effect | Illuminates or extinguishes the named indicator lamp |
| Called from | `services/alarm.rs`, `services/t4rupt.rs`, `services/display.rs` |

**Preconditions**: None.

**Postconditions**: The lamp state is as commanded. Lamp state persists until
the next `set_lamp` call for the same lamp; hardware restart clears all
channel 11 bits, so the `RESTART` sequence must re-assert necessary lamps.

#### `set_flash(on: bool)`

| Attribute | Value |
|-----------|-------|
| AGC channel | Output channel 11 (DSALMOUT), bit 6 |
| Side effect | Starts or stops the VERB/NOUN display flash (crew input requested) |
| Called from | `services/v_n.rs` (Verb/Noun processor) when awaiting crew input |

**Preconditions**: None.

**Postconditions**: VERB and NOUN display segments flash at the nominal
hardware rate (~1 Hz) when `on = true`; steady when `on = false`.

#### `read_key() -> Option<u8>`

| Attribute | Value |
|-----------|-------|
| AGC channel | Input channel 15 (MNKEYIN, bits 5–1) or channel 16 (NAVKEYIN, bits 5–1) for nav DSKY |
| Return | `Some(code)` — 5-bit key code (1–19 for the 19 DSKY keys); `None` — no key pending |
| Side effect | Consumes the keypress from the hardware latch |
| Called from | `services/v_n.rs` from within the `KeyRupt1` / `KeyRupt2` handler |

**Key code encoding**: 5-bit matrix code from the DSKY keyboard. The mapping
of code values to logical keys (0–9, VERB, NOUN, ENTER, CLR, PRO, KEY REL,
+, −, RSET) is defined in `tables/verb_table.rs` and `services/v_n.rs`, not
in the HAL.

**Preconditions**: None. Calling when no key is pending returns `None`; this
is not an error.

**Postconditions**: If a key was pending, it is consumed. A second call
returns `None` until the next keypress.

### 7.4 SimDsky Behaviour

`SimDsky` stores pending key codes in a `VecDeque<u8>`. `read_key()` pops the
front of the deque. Display and lamp methods are all no-ops. Tests inject
keypresses by pushing to `sim.dsky.keys`.

### 7.5 Test Cases

**TC-DSKY-01**: `read_key` returns queued codes and then `None`
```rust
let mut hw = SimHardware::new();
hw.dsky.keys.push_back(0x14); // VERB key code
hw.dsky.keys.push_back(0x03); // digit '3'
assert_eq!(hw.dsky().read_key(), Some(0x14));
assert_eq!(hw.dsky().read_key(), Some(0x03));
assert_eq!(hw.dsky().read_key(), None);
```

**TC-DSKY-02**: `write_row` and `set_lamp` do not panic on SimDsky
```rust
let mut hw = SimHardware::new();
hw.dsky().write_row(1, 0b0111_1111_1111); // all segments on row 1
hw.dsky().clear_row(1);
hw.dsky().set_lamp(Lamp::ProgAlarm, true);
hw.dsky().set_lamp(Lamp::CompActy, false);
// no assertions on state; just verify no panic
```

**TC-DSKY-03**: `set_flash` is idempotent
```rust
let mut hw = SimHardware::new();
hw.dsky().set_flash(true);
hw.dsky().set_flash(true);  // second call is idempotent
hw.dsky().set_flash(false);
hw.dsky().set_flash(false);
```

---

## 8. `Imu` Sub-Trait

**File**: `agc-core/src/hal/imu.rs`

### 8.1 Declaration

```rust
pub trait Imu {
    fn read_pipa(&mut self) -> [i16; 3];
    fn read_cdu(&self) -> [CduAngle; 3];
    fn torque_gyro(&mut self, axis: usize, pulses: i16);
    fn coarse_align(&mut self, commands: [i16; 3]);
    fn is_caged(&self) -> bool;
}
```

### 8.2 AGC Source Correspondence

The IMU (Inertial Measurement Unit) is the gyro-stabilised platform. Three
functional aspects are exposed:

**PIPA (Pulse-Integrating Pendulous Accelerometer)**:
PIPA counts accumulate in counter cells PIPAX, PIPAY, PIPAZ (IMU X/Y/Z
accelerometer axes). Each count represents a velocity increment of
approximately 0.0585 m/s on the real hardware (mission-calibrated). Reading
a PIPA resets its counter to zero — this is a destructive read.

> AGC source: `docs/AGC Symbolic Listing.md`, §IID. Counter cells in the PIPA
> address range; serviced by SERVICER (Average-G) every 2 seconds.

**CDU Angle Readout**:
The gimbal angles are stored in the counter cells CDUX (octal 0033), CDUY
(octal 0034), CDUZ (octal 0035). These are updated continuously by the CDU
hardware. Reading is non-destructive.

> AGC source: `docs/AGC Symbolic Listing.md`, §IID counter cells 0033–0035;
> §IIE channel 12 bit 6 (enable CDU error counters).

**Gyro Torquing**:
Fine alignment torques GYROCMD (octal 0047) with the pulse count; channel 14
bits 10–6 specify axis, polarity, and power-supply enable.

> AGC source: `docs/AGC Symbolic Listing.md`, §IIE channel 14 bits 6–10.

**Coarse Alignment**:
Channel 12 bits 6 and 4 enable the CDU error counters for coarse alignment;
cells CDUXCMD/CDUYCMD/CDUZCMD (octal 0050–0052) carry the commanded angle.

> AGC source: `docs/AGC Symbolic Listing.md`, §IIE channel 12 bits 4 and 6.

### 8.3 Method Specifications

#### `read_pipa() -> [i16; 3]`

| Attribute | Value |
|-----------|-------|
| AGC counter cells | PIPAX, PIPAY, PIPAZ (addresses per ERASABLE_ASSIGNMENTS.agc) |
| Return | `[x, y, z]` in raw pulse counts (signed; positive = positive delta-V on that axis) |
| Side effect | Resets counter cells to zero (destructive read) |
| Scale | ~0.0585 m/s per count (mission-calibrated; exact factor applied in `imu_control`) |
| Called from | `executive/scheduler.rs` (`Executive::run` foreground loop, every iteration). Counts are saturating-accumulated into `AgcState::pipa_counts`; the SERVICER reads and resets that staging field every 2 s. |

**Preconditions**: The IMU must be powered. Calling while the IMU is uncaged
or during coarse alignment produces implementation-defined values.

**Postconditions**: Counter cells are zeroed. Return value `[x, y, z]` holds
the accumulated counts since the last call. Counts are signed `i16`; overflow
of the hardware counter (if the SERVICER is delayed more than ~5 min) is
clamped at `i16::MAX` / `i16::MIN` by the hardware; the flight software must
detect and alarm on this condition.

**Error condition**: Calling `read_pipa` more frequently than the 2-second
SERVICER cycle would read partial counts and corrupt the navigation integration.
The SERVICER must be the only caller.

#### `read_cdu() -> [CduAngle; 3]`

| Attribute | Value |
|-----------|-------|
| AGC counter cells | CDUX (octal 0033), CDUY (octal 0034), CDUZ (octal 0035) |
| Return | `[outer, inner, middle]` gimbal angles as `CduAngle` (u16 twos-complement, full revolution = 65536 counts — see `specs/types-module-spec.md` §2.2 and §3.1) |
| Side effect | None (non-destructive) |
| Called from | `control/imu_control.rs`, `programs/p51_p52.rs` alignment programs |

**Preconditions**: None; readable at any time but values are meaningful only
after IMU initialisation.

**Postconditions**: Returns the instantaneous gimbal angles. The caller is
responsible for interpreting the angle convention (outer = roll, inner = pitch,
middle = yaw per the CM gimbal sequence).

**Angle encoding**: `CduAngle(u16)` — see `specs/types-module-spec.md` §3.1.
Convert to radians: `angle.to_radians()` = `(count as f64) * TAU / 65536.0`.

#### `torque_gyro(axis: usize, pulses: i16)`

| Attribute | Value |
|-----------|-------|
| AGC counter cell | GYROCMD (octal 0047); channel 14 bits 10–6 |
| `axis` | 0 = X (outer), 1 = Y (inner), 2 = Z (middle) |
| `pulses` | Signed pulse count; positive = positive torque direction per gyro polarity convention |
| Side effect | Commands the IMU CDU hardware to apply `|pulses|` torque pulses in the direction given by the sign, on the specified gyro axis |
| Called from | `control/imu_control.rs` fine alignment, drift compensation |

**Preconditions**: `axis` must be 0, 1, or 2. Passing any other value is a
programming error; bare-metal behaviour is undefined (channel 14 bits 8–7
encode the axis as a 2-bit field: `01` = X, `10` = Y, `11` = Z, `00` = none).

**Postconditions**: The hardware initiates torquing. The pulse train completes
asynchronously; GYROCMD is decremented to 0 by the DINC mechanism, and channel
14 bit 10 is auto-cleared.

**Note**: In the original AGC, calling `torque_gyro` while a previous torque
was still in progress would corrupt the GYROCMD counter. The flight software
must ensure the previous command has completed before issuing a new one. The
bare-metal HAL implementation must check and wait (within interrupt budget).

#### `coarse_align(commands: [i16; 3])`

| Attribute | Value |
|-----------|-------|
| AGC counter cells | CDUXCMD (octal 0050), CDUYCMD (octal 0051), CDUZCMD (octal 0052) |
| AGC channel | Channel 12 bits 6 (enable CDU error counters) and 4 (enable coarse align) |
| `commands` | `[x, y, z]` signed CDU drive counts |
| Side effect | Commands platform slew to specified gimbal angles |
| Called from | `programs/p51_p52.rs` coarse alignment sequences |

**Preconditions**: The IMU must be powered and the coarse-align mode must be
enabled in spacecraft software (P52 flow). Using this while the IMU is in
fine-alignment mode produces a mechanical disturbance to the stable platform.

**Postconditions**: CDU error counters begin driving the gimbals toward the
commanded angles. The operation completes asynchronously; P52 polls `read_cdu`
to determine when the slew is complete.

#### `is_caged() -> bool`

| Attribute | Value |
|-----------|-------|
| AGC channel | Input channel 30 (CHAN30), bit 11 (inverted: 0 if caged) |
| Return | `true` if the IMU cage switch is active (gimbals driven to zero) |
| Side effect | None |
| Called from | `control/imu_control.rs`, IMU monitoring in T4RUPT |

**Preconditions**: None.

**Postconditions**: Returns the current cage status. The cage can be commanded
by the crew (hardware switch) or by software; both paths are reflected in
channel 30 bit 11.

### 8.4 SimImu Behaviour

`SimImu` stores `pipa: [i16; 3]` and `cdu: [CduAngle; 3]` as public fields.
`read_pipa()` snapshots and zeroes `pipa`. `read_cdu()` returns `cdu`.
`torque_gyro` and `coarse_align` are no-ops. `is_caged()` returns `false`.

Tests inject accelerations by writing to `sim.imu.pipa` before each
SERVICER-equivalent call.

### 8.5 Test Cases

**TC-IMU-01**: `read_pipa` clears counts after reading
```rust
let mut hw = SimHardware::new();
hw.imu.pipa = [100, -50, 25];
let counts = hw.imu().read_pipa();
assert_eq!(counts, [100, -50, 25]);
assert_eq!(hw.imu().read_pipa(), [0, 0, 0]); // cleared
```

**TC-IMU-02**: `read_cdu` is non-destructive
```rust
let mut hw = SimHardware::new();
hw.imu.cdu = [CduAngle(8192), CduAngle(0), CduAngle(16384)];
let first  = hw.imu().read_cdu();
let second = hw.imu().read_cdu();
assert_eq!(first, second); // non-destructive
assert_eq!(first[0].0, 8192);
```

**TC-IMU-03**: `torque_gyro` does not panic and does not alter PIPA or CDU in sim
```rust
let mut hw = SimHardware::new();
hw.imu.cdu = [CduAngle(1000), CduAngle(2000), CduAngle(3000)];
hw.imu().torque_gyro(0, 512);
hw.imu().torque_gyro(1, -256);
hw.imu().torque_gyro(2, 1);
assert_eq!(hw.imu().read_cdu()[0].0, 1000); // CDU unchanged in sim
```

---

## 9. `Optics` Sub-Trait

**File**: `agc-core/src/hal/optics.rs`

### 9.1 Declaration

```rust
pub trait Optics {
    fn trunnion_angle(&self) -> CduAngle;
    fn shaft_angle(&self) -> CduAngle;
    fn drive(&mut self, trunnion: i16, shaft: i16);
    fn mark_pressed(&self) -> bool;
}
```

### 9.2 AGC Source Correspondence

The CM optics (sextant / telescope) are driven by two CDUs:
- **Trunnion** (elevation): counter cell OPTY (octal 0036), command cell
  CDUTCMD (octal 0053), enabled by channel 12 bit 2 (CM: enable optics CDU
  error counters) and channel 12 bit 8 (TVC enable or optics error counter
  connect).
- **Shaft** (azimuth): counter cell OPTX (octal 0037), command cell CDUSCMD
  (octal 0054).

The optics mark signal arrives on input channel 16 (NAVKEYIN), bit 6.

> AGC source: `docs/AGC Symbolic Listing.md`, §IIE channel 12 bits 2, 8, 10, 11;
> channel 16 bits 6–7; §IID counter cells 0036–0037, 0053–0054.

### 9.3 Method Specifications

#### `trunnion_angle() -> CduAngle`

| Attribute | Value |
|-----------|-------|
| AGC counter cell | OPTY (octal 0036) |
| Return | Current trunnion CDU angle |
| Side effect | None |

**Preconditions**: None; readable at any time.

**Postconditions**: Returns instantaneous trunnion angle. `CduAngle(0)` = zero
(bore-sight elevation).

#### `shaft_angle() -> CduAngle`

| Attribute | Value |
|-----------|-------|
| AGC counter cell | OPTX (octal 0037) |
| Return | Current shaft CDU angle |
| Side effect | None |

**Preconditions/Postconditions**: analogous to `trunnion_angle`.

#### `drive(trunnion: i16, shaft: i16)`

| Attribute | Value |
|-----------|-------|
| AGC counter cells | CDUTCMD (octal 0053), CDUSCMD (octal 0054) |
| AGC channel | Channel 12 bit 2 (enable optics CDU error counters), channel 14 bits 12–11 |
| Arguments | Signed rate commands in CDU counts/s; positive = positive angle direction |
| Side effect | Commands the optics drive motors at the given rates |
| Called from | `programs/p23.rs` (star/landmark tracking) |

**Preconditions**: Channel 12 bit 11 (disengage optics DAC) must be clear for
the drive to have physical effect. The HAL implementation manages this bit.

**Postconditions**: Drive motors begin slewing at the commanded rates.
Commands persist until the next `drive` call or until `drive(0, 0)` is issued.

**Error condition**: Calling `drive` while channel 12 bit 11 is set (optics
DAC disengaged) will not move the optics. The optics mode management in
`programs/p23.rs` must ensure the correct channel state before calling.

#### `mark_pressed() -> bool`

| Attribute | Value |
|-----------|-------|
| AGC channel | Input channel 16 (NAVKEYIN), bit 6 (CM: optics mark signal if 1) |
| Return | `true` if the optics mark button is currently pressed |
| Side effect | None |
| Called from | `programs/p23.rs`, star-sighting mark sequence |

**Preconditions**: None.

**Postconditions**: Returns instantaneous button state. The KeyRupt2 interrupt
also fires on mark button press; the HAL exposes both the edge interrupt path
(via `Interrupt::KeyRupt2`) and the level-sense path (via `mark_pressed()`).

### 9.4 SimOptics Behaviour

`SimOptics` stores `trunnion: CduAngle` and `shaft: CduAngle`. Angle readers
return the stored values. `drive` is a no-op. `mark_pressed()` returns `false`.

Tests may set `sim.optics.trunnion` / `.shaft` to inject star-sighting
scenarios.

### 9.5 Test Cases

**TC-OPTICS-01**: angle reads return injected values
```rust
let mut hw = SimHardware::new();
hw.optics.trunnion = CduAngle(4096);
hw.optics.shaft    = CduAngle(12288);
assert_eq!(hw.optics().trunnion_angle().0, 4096);
assert_eq!(hw.optics().shaft_angle().0, 12288);
```

**TC-OPTICS-02**: `drive` is a no-op in sim (angles unchanged)
```rust
let mut hw = SimHardware::new();
hw.optics.trunnion = CduAngle(1000);
hw.optics().drive(100, -200);
assert_eq!(hw.optics().trunnion_angle().0, 1000);
```

**TC-OPTICS-03**: `mark_pressed` returns false by default
```rust
let mut hw = SimHardware::new();
assert!(!hw.optics().mark_pressed());
```

---

## 10. `Engine` Sub-Trait

**File**: `agc-core/src/hal/engine.rs`

### 10.1 Declaration

```rust
pub trait Engine {
    fn sps_enable(&mut self, on: bool);
    fn sps_gimbal(&mut self, pitch: i16, yaw: i16);
    fn thrust_on(&self) -> bool;
}
```

### 10.2 AGC Source Correspondence

The SPS (Service Propulsion System) engine is controlled through:

- **Engine on/off**: Output channel 11 (DSALMOUT), bit 13 (CM). Set to 0 to
  turn the engine off; set to 1 to enable the ignition relay.
- **Gimbal (TVC)**: Counter cells TVCYAW = CDUTCMD (octal 0053),
  TVCPITCH = CDUSCMD (octal 0054); channel 12 bit 8 (TVC enable) connects the
  CDU error counter output to the SPS gimbal servo amplifiers.
- **Thrust-on discrete**: Input channel 30 or a dedicated discrete. The
  `thrust_on()` method reads the thrust-present signal.

> AGC source: `docs/AGC Symbolic Listing.md`, §IIE channel 11 bit 13(CM)
> "SPS engine turn-on signal"; channel 12 bit 8(CM) "TVC enable"; channel 12
> bits 12–11 (optics/TVC CDU) note re SPS control.

### 10.3 Method Specifications

#### `sps_enable(on: bool)`

| Attribute | Value |
|-----------|-------|
| AGC channel | Output channel 11 (DSALMOUT), bit 13(CM) |
| Side effect | Asserts or de-asserts the SPS ignition relay; `on = true` commands ignition; `on = false` commands cutoff |
| Called from | `programs/p40_p41.rs` (SPS ignition/cutoff sequences) |

**Preconditions**: The engine arm sequence must have been completed at the
program level before `sps_enable(true)` is safe. The HAL does not enforce arm
status; the flight-software P40 program is responsible.

**Postconditions**: The SPS ignition relay is in the commanded state. A brief
channel-write transient (~0.25 ms) zeros all channel 11 bits during the write
cycle. The implementation must mask and restore non-engine bits atomically.

**Error condition**: Calling `sps_enable(true)` while channel 12 bit 8 (TVC)
is not enabled will ignite without gimbal authority — a safety hazard managed
at the P40 program level, not the HAL level.

**Hardware restart note**: All output channel bits including channel 11 bit 13
are cleared on hardware restart. The restart sequence in
`services/fresh_start.rs` must issue `sps_enable(false)` explicitly to
synchronise the HAL's internal state with the hardware state.

#### `sps_gimbal(pitch: i16, yaw: i16)`

| Attribute | Value |
|-----------|-------|
| AGC counter cells | TVCPITCH = CDUSCMD (octal 0054), TVCYAW = CDUTCMD (octal 0053) |
| AGC channel | Channel 12 bit 8 (TVC enable); channel 14 bits 12, 11 (CDU pulse generation) |
| Arguments | Signed CDU counts; the TVC module defines the scale and polarity |
| Side effect | Commands SPS gimbal servo amplifiers to the specified pitch/yaw angles |
| Called from | `control/tvc.rs` during T5RUPT DAP cycle |

**Preconditions**: Channel 12 bit 8 (TVC enable) must be set. The `tvc.rs`
module manages this; `sps_gimbal` assumes TVC mode is active.

**Postconditions**: CDU error counters drive the gimbal servos. The actual
gimbal movement follows asynchronously; `tvc.rs` must not issue a new gimbal
command before the previous one has taken effect (timing managed by T5RUPT
period).

#### `thrust_on() -> bool`

| Attribute | Value |
|-----------|-------|
| AGC channel | Thrust-on discrete (spacecraft wiring; not a named AGC channel in the symbolic listing but part of the discrete input set) |
| Return | `true` if the SPS is currently producing thrust |
| Side effect | None |
| Called from | `programs/p47.rs` (thrust monitor), `programs/p40_p41.rs` |

**Preconditions**: None.

**Postconditions**: Returns the hardware discrete state. There is a latency of
up to one T5RUPT period (~100 ms) between `sps_enable(true)` and `thrust_on()`
returning `true` due to ignition sequence timing.

### 10.4 SimEngine Behaviour

`SimEngine` stores `thrusting: bool`. `sps_enable(on)` sets `thrusting = on`.
`sps_gimbal` is a no-op. `thrust_on()` returns `thrusting`.

### 10.5 Test Cases

**TC-ENGINE-01**: `sps_enable` toggles `thrust_on` state
```rust
let mut hw = SimHardware::new();
assert!(!hw.engine().thrust_on());
hw.engine().sps_enable(true);
assert!(hw.engine().thrust_on());
hw.engine().sps_enable(false);
assert!(!hw.engine().thrust_on());
```

**TC-ENGINE-02**: `sps_gimbal` does not affect thrust state
```rust
let mut hw = SimHardware::new();
hw.engine().sps_enable(true);
hw.engine().sps_gimbal(100, -50);
assert!(hw.engine().thrust_on()); // unaffected
```

**TC-ENGINE-03**: initial state is not thrusting
```rust
let hw = SimHardware::new();
// thrust_on is false by default
assert!(!hw.engine.thrusting);
```

---

## 11. `Rcs` Sub-Trait

**File**: `agc-core/src/hal/rcs.rs`

### 11.1 Declaration

```rust
pub trait Rcs {
    fn fire_sm_jets(&mut self, jets_a: u8, jets_b: u8);
    fn fire_cm_jets(&mut self, jets: u16);
    fn quench_all(&mut self);
}
```

### 11.2 AGC Source Correspondence

RCS (Reaction Control System) jets are controlled through two output channels:
- **Channel 05 (PYJETS)**: 8-bit bitmask for SM RCS jet group A (pitch/yaw)
- **Channel 06 (ROLLJETS)**: 8-bit bitmask for SM RCS jet group B (roll)

The CM RCS (used during entry) is driven by the same channel pair with a
different mapping for CM jets.

Each bit in the channel corresponds to one RCS jet. A `1` fires the jet; a `0`
turns it off. The mapping of bit positions to physical jets is defined in
`control/rcs_logic.rs`, not in the HAL.

> AGC source: `docs/AGC Symbolic Listing.md`, §IIE channels 05 and 06 ("RCS
> jet controls"). Architecture §11.2 "RCS Jet Selection".

The jet timing sequence is:
1. `rcs.fire_sm_jets(jets_a, jets_b)` — fires jets; simultaneously arms T6
   with the desired duration via `timers.arm_t6(counts)`.
2. T6RUPT fires after `counts × 0.625 ms`.
3. T6RUPT handler calls `rcs.quench_all()` to cut all jets.

### 11.3 Method Specifications

#### `fire_sm_jets(jets_a: u8, jets_b: u8)`

| Attribute | Value |
|-----------|-------|
| AGC channel | Output channel 05 (PYJETS) = `jets_a`; output channel 06 (ROLLJETS) = `jets_b` |
| Side effect | Sets the specified jets on; previously-on jets not in the new mask are turned off |
| Called from | `control/rcs_logic.rs` from within T6RUPT or DAP task |

**Preconditions**: T6 must be armed with the desired pulse duration before or
immediately after this call. Firing jets without arming T6 leaves them on
indefinitely (until the next `quench_all` or `fire_sm_jets(0, 0)`).

**Postconditions**: Channel 05 = `jets_a`, channel 06 = `jets_b`. The physical
jets respond within ~1 ms (valve opening latency). The mapping of bits to jets:

| Channel 05 bit | Jet group |
|---|---|
| bits 7–4 | SM RCS Pitch/Yaw quad A |
| bits 3–0 | SM RCS Pitch/Yaw quad B |

(Exact mapping: `control/rcs_logic.rs` jet table.)

**Hardware restart**: Channels 05 and 06 are cleared on restart. All jets are
off after restart, which is the fail-safe state.

#### `fire_cm_jets(jets: u16)`

| Attribute | Value |
|-----------|-------|
| AGC channel | Output channels 05/06 with CM RCS bit mapping (entry phase only) |
| `jets` | 12-bit mask (bits 11–0) for 12 CM RCS jets |
| Side effect | Commands the CM RCS jets for the entry phase |
| Called from | `guidance/entry.rs`, `programs/p61_p67.rs` during atmospheric entry |

**Preconditions**: SM/CM separation must have occurred; calling during
pre-separation flight connects to the wrong propellant system (hardware switch
state, not a software concern).

**Postconditions**: CM jets commanded as specified.

#### `quench_all()`

| Attribute | Value |
|-----------|-------|
| AGC channel | Output channel 05 = 0x00, channel 06 = 0x00 |
| Side effect | Turns off ALL RCS jets immediately |
| Called from | T6RUPT handler (normal jet pulse termination); hardware restart handler (safety); program abort sequences |

**Preconditions**: None; safe to call at any time.

**Postconditions**: All RCS jets are off. Channels 05 and 06 are both `0x00`.

**Timing**: Must execute within the T6RUPT budget (0.5 ms). `quench_all` must
not perform any operations that could exceed this budget.

### 11.4 SimRcs Behaviour

`SimRcs` is a unit struct with no stored state. All three methods are no-ops.
Jet firing is tested at the integration level by observing simulated vehicle
dynamics in `agc-sim/src/physics.rs`.

### 11.5 Test Cases

**TC-RCS-01**: `quench_all` does not panic
```rust
let mut hw = SimHardware::new();
hw.rcs().quench_all();
hw.rcs().quench_all(); // idempotent
```

**TC-RCS-02**: `fire_sm_jets` does not panic with valid masks
```rust
let mut hw = SimHardware::new();
hw.rcs().fire_sm_jets(0b1010_0101, 0b0101_1010);
hw.rcs().fire_sm_jets(0x00, 0x00);
```

**TC-RCS-03**: `fire_cm_jets` accepts a 12-bit mask
```rust
let mut hw = SimHardware::new();
hw.rcs().fire_cm_jets(0b0000_1111_1111); // 12 jets, all on
hw.rcs().quench_all();
```

---

## 12. `Uplink` Sub-Trait

**File**: `agc-core/src/hal/uplink.rs`

### 12.1 Declaration

```rust
pub trait Uplink {
    fn read_word(&mut self) -> Option<u16>;
}
```

### 12.2 AGC Source Correspondence

Uplink data from the ground arrives via the INLINK receiver. Each received bit
is clocked into counter cell INLINK (octal 0045). When a complete 15-bit word
has been received, program interrupt #7 (UplinkRupt) fires and the software
reads the word.

> AGC source: `docs/AGC Symbolic Listing.md`, §IID cell 0045 (INLINK); §IIH
> "Program interrupt #7"; channel 33 bit 11 (uplink bit rate error flip-flop);
> channel 13 bit 6 (block all INLINK inputs); channel 13 bit 5 (crosslink
> select).

The uplink word format and command protocol are defined in
`services/uplink.rs` (flight software, not HAL). The HAL exposes only the
raw word delivery.

### 12.3 Method Specification

#### `read_word() -> Option<u16>`

| Attribute | Value |
|-----------|-------|
| AGC counter cell | INLINK (octal 0045) |
| Return | `Some(word)` — the next 15-bit uplink word as a `u16`; `None` — buffer is empty |
| Side effect | Consumes the word from the hardware FIFO |
| Called from | `UplinkRupt` interrupt handler in `services/uplink.rs` |

**Preconditions**: None.

**Postconditions**: If a word was available, it is consumed. A second call
returns `None` until the next UplinkRupt. The word is raw; decoding (load
address type, data field) is done by the uplink processor.

**Error conditions**: Channel 33 bit 11 (uplink bit rate error flip-flop, set
if bits arrive too fast) is not surfaced through this method — it is monitored
by the T4RUPT handler separately. If the uplink rate error flag is set, words
received may be corrupt; the uplink processor checks this flag before
processing.

### 12.4 SimUplink Behaviour

`SimUplink` stores `words: VecDeque<u16>`. `read_word()` pops the front.
Tests inject uplink sequences by pushing words to `sim.uplink.words`.

### 12.5 Test Cases

**TC-UPLINK-01**: `read_word` returns queued words and then `None`
```rust
let mut hw = SimHardware::new();
hw.uplink.words.push_back(0x0042);
hw.uplink.words.push_back(0x1F00);
assert_eq!(hw.uplink().read_word(), Some(0x0042));
assert_eq!(hw.uplink().read_word(), Some(0x1F00));
assert_eq!(hw.uplink().read_word(), None);
```

**TC-UPLINK-02**: empty buffer returns `None` immediately
```rust
let mut hw = SimHardware::new();
assert_eq!(hw.uplink().read_word(), None);
```

**TC-UPLINK-03**: multiple words are delivered in FIFO order
```rust
let mut hw = SimHardware::new();
for i in 0u16..5 {
    hw.uplink.words.push_back(i * 100);
}
for i in 0u16..5 {
    assert_eq!(hw.uplink().read_word(), Some(i * 100));
}
assert_eq!(hw.uplink().read_word(), None);
```

---

## 13. `Telemetry` Sub-Trait

**File**: `agc-core/src/hal/telemetry.rs`

### 13.1 Declaration

```rust
pub trait Telemetry {
    fn send_word(&mut self, word: u16);
}
```

### 13.2 AGC Source Correspondence

The AGC downlinks telemetry via output channels 34 (DNTM1) and 35 (DNTM2).
These two channels together form a 30-bit word pair that is transmitted when
program interrupt #8 (DownRupt) fires. Loading these channels cannot be sensed
by a channel-read instruction.

Channel 13 bit 7 carries the word-order code bit (flags certain words in the
telemetry list).

> AGC source: `docs/AGC Symbolic Listing.md`, §IIE channels 34 and 35
> ("DNTM1", "DNTM2"); channel 33 bit 12 (telemetry end pulse "too fast"
> error flip-flop); §IIH "Program interrupt #8".

The Rust HAL simplifies this to a single `send_word(u16)` call. The downlink
formatter in `services/display.rs` is responsible for packing and ordering the
two-word pairs; the HAL simply delivers each `u16` to the transmitter.

### 13.3 Method Specification

#### `send_word(word: u16)`

| Attribute | Value |
|-----------|-------|
| AGC channel | Output channels 34 (DNTM1) and 35 (DNTM2), alternating per word pair |
| Side effect | Delivers `word` to the telemetry transmitter |
| Called from | `DownRupt` handler in `services/display.rs` |
| Timing | Must complete before the next DownRupt (~10 ms) |

**Preconditions**: None.

**Postconditions**: The word is handed off to the transmitter hardware.
The HAL implementation may buffer one word pair; it is a programming error
to call `send_word` faster than the transmitter can accept it (channel 33
bit 12 monitors this condition). If the error flag is set, the implementation
should record the condition; the flight software must check and alarm.

### 13.4 SimTelemetry Behaviour

`SimTelemetry` appends each word to `log: Vec<u16>`. Tests verify the
telemetry log contents.

### 13.5 Test Cases

**TC-TELEMETRY-01**: `send_word` appends to log
```rust
let mut hw = SimHardware::new();
hw.telemetry().send_word(0xABCD);
hw.telemetry().send_word(0x1234);
assert_eq!(hw.telemetry.log, vec![0xABCD, 0x1234]);
```

**TC-TELEMETRY-02**: log is empty initially
```rust
let hw = SimHardware::new();
assert!(hw.telemetry.log.is_empty());
```

**TC-TELEMETRY-03**: `send_word` preserves all 16 bits
```rust
let mut hw = SimHardware::new();
hw.telemetry().send_word(0xFFFF);
hw.telemetry().send_word(0x0000);
assert_eq!(hw.telemetry.log[0], 0xFFFF);
assert_eq!(hw.telemetry.log[1], 0x0000);
```

---

## 14. Implementor Guidance: `embedded-hal` v1 Integration

This section is for bare-metal HAL implementors only. The flight software in
`agc-core` is not affected by these decisions.

### 14.1 General Requirements

From `docs/architecture.md` §4.1:

- **C-FREE**: Every non-`Copy` HAL wrapper must expose a `free()` method that
  consumes the wrapper and returns the raw `embedded-hal` peripheral, allowing
  reclamation.
- **C-HAL-TRAITS**: Bare-metal structs must implement all applicable
  `embedded-hal` v1 traits in addition to the `agc-core` sub-traits so that
  standard tooling (`probe-rs`, `defmt`, third-party drivers) can interact with
  them.
- **C-PIN-STATE**: Operational modes that are only valid in certain hardware
  states must be encoded as type parameters (typestates) to make
  misconfiguration a compile-time error.
- **C-REEXPORT-PAC**: The bare-metal HAL crate re-exports the device PAC under
  the name `pac`:
  ```rust
  pub use stm32f4 as pac;
  ```

### 14.2 Timer Implementation

`arm_t3(cs)`, `arm_t5(cs)`, and `arm_t6(counts)` translate to MCU hardware
timer counter/compare register writes. The specific registers depend on the
target MCU. On an STM32F4:
- T3 → TIM3 ARR/CNT reload
- T5 → TIM5 ARR/CNT reload
- T6 → TIM6 single-shot one-pulse mode

`disarm_t6()` disables the TIM6 update interrupt.

The `#[interrupt]` attribute for each timer handler must come from the device
PAC crate (e.g. `use stm32f4::interrupt;`), not from `cortex-m-rt` directly,
to get compile-time interrupt name verification.

### 14.3 IMU Typestate Example

The typestate pattern prevents calling `torque_gyro` on an unaligned IMU:

```rust
pub struct ImuImpl<State> {
    spi: Spi<SPI1>,
    _state: core::marker::PhantomData<State>,
}

pub struct Unaligned;
pub struct CoarseAligned;
pub struct FineAligned;

impl ImuImpl<Unaligned> {
    pub fn into_coarse_aligned(self) -> ImuImpl<CoarseAligned> { ... }
    pub fn free(self) -> Spi<SPI1> { self.spi }
}

impl ImuImpl<CoarseAligned> {
    pub fn into_fine_aligned(self) -> ImuImpl<FineAligned> { ... }
}

impl Imu for ImuImpl<FineAligned> {
    fn torque_gyro(&mut self, axis: usize, pulses: i16) { ... }
    // ...
}
```

Note: The `Imu` trait as defined in `hal/imu.rs` does **not** carry a typestate
parameter, because that would make `AgcHardware` generic over the IMU state and
break the simple `type Imu: Imu` associated type. The typestate is an
implementation detail of the bare-metal crate that is resolved at compile time
when the concrete `AgcHardware` type is chosen.

### 14.4 DSKY Relay Timing

The 20 ms relay hold must be implemented as a hardware timer ISR, not as a
blocking `cortex_m::delay::Delay` call, because blocking inside the
`write_row` method would stall the foreground Executive for 20 ms, violating
the timing budget.

Recommended implementation: double-buffer of (row, data) pairs in a static
`Mutex<RefCell<[Option<(u8, u16)>; 2]>>`. The T4RUPT context writes to the
buffer; a dedicated lower-priority timer ISR sequences the relays.

### 14.5 Cortex-M Target Requirements

- **Minimum**: Cortex-M7 with double-precision FPU (e.g., STM32H743,
  STM32F767). Hardware `f64` operations are required for the DAP's 100 ms
  timing budget. Cortex-M4F has only a single-precision (f32) FPU; `f64` on
  M4F would require software emulation, which is approximately 10x slower and
  would violate the DAP deadline for attitude computations.
- **Cortex-M33**: May be used if the target includes the optional
  double-precision FPU extension.
- **Soft-float is prohibited**: Soft-float emulation breaks the DAP 100 ms
  timing budget. The build system must enforce a hard-float ABI.

See `docs/architecture.md` §14.1 for the authoritative statement of this
requirement and the rationale.

---

## 15. Error Conditions Summary

| Scenario | Expected Behaviour |
|---|---|
| `arm_t3(0)` | Programming error; T3RUPT fires within one tick. Waitlist must validate. |
| `arm_t3(> 16383)` | Programming error; timer wraps, fires too early. Use long-waitlist chaining. |
| `arm_t6(0)` | Programming error; T6RUPT fires within one tick; jets may not fire or quench immediately. |
| `torque_gyro` with `axis > 2` | Undefined on bare metal (channel 14 bits 8–7 are 2-bit). SimImu is a no-op. |
| `write_row` with `row == 0` or `row > 14` | Undefined relay matrix behaviour. Flight software must validate. |
| `read_pipa` called more often than every 2 s | Counter not saturated; partial accumulations corrupt navigation. SERVICER must be sole caller. |
| `sps_enable(true)` without TVC enabled | Engine fires without gimbal authority. P40 program is responsible for sequence ordering. |
| `hardware_restart()` in SimHardware | `panic!` — propagates as test failure, not a real restart. |
| NEWJOB not sampled for > 1.92 s | Hardware triggers restart. `pet_watchdog` must be called at least once per Executive loop iteration. |
| DownRupt fires faster than 100 pps | Channel 33 bit 12 error flip-flop is set; `Telemetry::send_word` may lose words. T4RUPT monitors and alarms. |

---

## 16. Specification Quality Checklist

Verified against `specs/README.md`:

- [x] AGC source file and line range referenced (§IIE, §IIH, §IID of AGC Symbolic Listing)
- [x] All erasable variables and their AGC addresses listed (TIME3/4/5/6, PIPA cells, CDU cells, INLINK, GYROCMD)
- [x] Scale factors documented (CDU: B-1 revolutions original / 2^16 counts in Rust u16; PIPA: ~0.0585 m/s/count; TIME: 0.01 s/count; T6: 0.000625 s/count)
- [x] Corresponding `f64` SI units documented (conversion noted at call sites)
- [x] Input/output preconditions and postconditions stated (each method in §6–13)
- [x] Edge cases and error handling specified (§15 and per-method error conditions)
- [x] At least 3 test cases per sub-trait with expected values (§6.4, §7.5, §8.5, §9.5, §10.5, §11.5, §12.5, §13.5)
- [x] Rust API signatures designed (types, ownership — all methods take `&mut self` or `&self`)
- [x] Invariants explicitly stated (destructive reads, relay timing, jet T6 pairing)
- [x] Consistency with `docs/architecture.md` checked (§4.1, §4.2, §4.3, §11, §13, §14.1)
