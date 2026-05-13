# AGC-in-Rust: Software Architecture

## 1. Design Philosophy

This architecture is NOT a cycle-accurate emulator of the Block 2 AGC. It is an
idiomatic Rust reimplementation of the Comanche055 (Command Module) guidance
software that preserves the original's functional behavior, safety properties,
and real-time characteristics while recovering the high-level abstractions that
were lost when the program was written in AGC assembly language.

The key tension in this port is between two goals:

- **Fidelity**: The software must produce the same navigation, guidance, and
  control outputs as the original AGC given the same inputs.
- **Idiom**: The software must use Rust's type system, ownership model, and
  module system to make the design legible, maintainable, and provably safe
  where the original relied on programmer discipline.

Where these conflict, fidelity wins. Navigation errors kill people.

### Governing Constraints

| Constraint | Original AGC | Rust Port |
|---|---|---|
| Memory | 2048 words erasable, 36864 words fixed | Target: `no_std`, static allocation only, no heap |
| CPU | ~85000 additions/sec (11.7us MCT) | Bare-metal embedded target; must meet same deadlines |
| OS | None; software owns the machine | `agc-core` is `#![no_std]`; firmware binary is also `#![no_main]`; we provide the scheduler |
| Errors | Hardware restart restores safe state | Rust panics trigger restart handler; no unwinding |
| Arithmetic | 15-bit ones-complement, fractional | `f64` for navigation/guidance math; `i16`/`u16` for I/O and hardware registers |
| Real-time | Hard deadlines (T3RUPT variable for Waitlist, T4RUPT 120ms cycle, T5RUPT ~100ms for DAP, T6RUPT on-demand for RCS jet timing) | Interrupt-driven with priority; same timing contracts |


## 2. Crate and Module Structure

The project is organized as a Cargo workspace. `agc-core` holds the flight
software as a portable `no_std` library; concrete hardware drivers, the wire
protocol, the IMU platform emulator, the bridge firmware, the host simulator,
and the integration tests each live in their own crate. This split keeps the
flight software independent of any specific MCU and lets host-side tests link
`agc-core` without pulling in bare-metal dependencies.

```
agc-in-rust/                     (workspace root)
  Cargo.toml                     (workspace definition)
  agc-core/                      (the flight software -- #![no_std] library)
    src/
      lib.rs                     (crate root, feature gates, AgcState definition)
      
      types/
        mod.rs
        angle.rs                 (CduAngle: i16 signed counts, full revolution = 2^16; Met centiseconds)
        vector.rs                (Vec3: [f64; 3] -- position, velocity, delta-V)
        matrix.rs                (Mat3x3: [[f64; 3]; 3] -- coordinate transforms)
      
      hal/
        mod.rs                   (AgcHardware trait -- bounds on the embedded-hal impl)
        interrupts.rs            (Interrupt enum, INHINT/RELINT critical section)
        runtime.rs               (Strategy-D foreground drain: T3/T4/T5/T6_PENDING atomic flags, ADR-017)
        timers.rs                (Scheduler timers: T3, T4, T5, T6 period management)
        dsky.rs                  (DSKY 21-row per-field display interface, ADR-019)
        imu.rs                   (IMU CDU angles, gyro torque commands, PIPA reads)
        optics.rs                (CM optics shaft/trunnion drive and readback)
        engine.rs                (SPS enable, gimbal commands)
        rcs.rs                   (RCS jet on/off commands)
        uplink.rs                (Uplink receiver interface)
        telemetry.rs             (Downlink word output)
      
      executive/
        mod.rs                   (Executive system -- job scheduling)
        job.rs                   (Job: priority, state, register set, VAC area)
        waitlist.rs              (Waitlist system -- task scheduling via TIME3)
        scheduler.rs             (EXEC main loop: priority scan, NOVAC/FINDVAC)
        restart.rs               (Restart protection: phase tables, group management)
      
      math/
        mod.rs
        trig.rs                  (sin, cos, asin, acos -- f64 wrappers with AGC domain conventions)
        kepler.rs                (Kepler equation solvers, universal variable)
        lambert.rs               (Lambert's problem -- transfer orbit targeting)
        linalg.rs                (vector/matrix ops: dot, cross, norm, rotate)
      
      navigation/
        mod.rs
        state_vector.rs          (Position/velocity state vectors, coordinate frames)
        integration.rs           (Cowell numerical integration, Encke method)
        gravity.rs               (Earth/Moon gravity models, oblateness)
        conics.rs                (Conic trajectory routines: Kepler, Lambert)
        kalman.rs                (Scalar Kalman update shared by P20/P22 nav)
        star_catalog.rs          (Fixed star table for IMU alignment)
        planetary.rs             (Planetary/lunar ephemeris)
        time.rs                  (Mission elapsed time, ground elapsed time)
      
      guidance/
        mod.rs
        targeting.rs             (Targeting parameters, TIG computation)
        maneuver.rs              (Delta-V computation, burn attitude, cross-product steering)
        midcourse.rs             (Midcourse correction guidance)
        entry.rs                 (CM entry guidance -- skip/ballistic targeting)
        lambert.rs               (Lambert aim point computation)
        rendezvous.rs            (LVLH range/range-rate; shared by P20/P34/P35)
      
      control/
        mod.rs
        dap.rs                   (Digital Autopilot supervisor)
        attitude.rs              (Attitude control: rate damping, attitude hold, maneuver)
        tvc.rs                   (Thrust Vector Control for SPS burns)
        rcs_logic.rs             (RCS jet select logic, minimum impulse)
        imu_control.rs           (IMU coarse/fine align, gyro compensation, drift)
      
      programs/
        mod.rs                   (Major Mode dispatch table)
        p00.rs                   (CMC Idling -- POO)
        p01_p02.rs               (Pre-launch initialization / Gyrocompassing)
        p06.rs                   (Power-down)
        p11.rs                   (Earth orbit insertion monitor)
        p15.rs                   (TLI initiate/cutoff)
        p20.rs                   (Rendezvous navigation -- VHF/optics marks)
        p21.rs                   (Ground track determination)
        p22.rs                   (Orbital landmark navigation)
        p23.rs                   (Cislunar midcourse navigation -- star/landmark)
        p30.rs                   (External Delta-V targeting)
        p31.rs                   (Lambert pre-thrust)
        p32.rs                   (CSI pre-thrust)
        p33.rs                   (CDH pre-thrust)
        p34.rs                   (TPI pre-thrust)
        p37.rs                   (Return to Earth)
        p40_p41.rs               (SPS/RCS thrusting)
        p47.rs                   (Thrust monitor)
        p51_p52.rs               (IMU orientation / realignment)
        p61_p67.rs               (Entry programs -- pre-entry through landing)
      
      services/
        mod.rs
        average_g.rs             (SERVICER: 2-second navigation cycle using PIPAs)
        v_n.rs                   (Verb-Noun processor: DSKY command interpreter)
        display.rs               (display formatting helpers)
        pinball.rs               (DSKY frame encoder -- 21 rows, ADR-019)
        alarm.rs                 (Program alarm system: 1202, 1210, etc.)
        fresh_start.rs           (FRESH START and RESTART sequences)
        backup.rs                (Battery-backed BKPSRAM state for RESTART)
        t4rupt.rs                (T4RUPT periodic I/O: DSKY, IMU monitoring, gyro drift comp)
      
      tables/
        mod.rs
        noun_table.rs            (Noun definitions: addresses, components, scale factors)
        verb_table.rs            (Verb definitions: routine entry points)
        alarm_codes.rs           (Alarm code definitions and severity)
  
  agc-protocol/                  (Bridge wire format -- #![no_std], used by both MCUs)
    src/
      lib.rs                     (re-exports; PROTO_VERSION constant)
      msg.rs                     (Msg enum: every message carried over the link)
      frame.rs                   (STX/length/seq framing, encode/decode)
      crc.rs                     (CRC-16/CCITT: poly=0x1021, init=0xFFFF)
  
  agc-imu-platform/              (Virtual stable platform emulator -- #![no_std])
    src/
      lib.rs                     (CDU_PULSE_RAD, GYRO_PULSE_RAD, PIPA_SCALE constants)
      platform.rs                (PlatformEmulator: gimballed-platform abstraction;
                                  caged/coarse/fine states; destructive PIPA reads)
      quat.rs                    (UnitQuaternion: rotation math for platform attitude)
  
  agc-board-nucleo-f767/         (Bare-metal HAL for Nucleo-F767ZI -- #![no_std])
    src/
      lib.rs                     (Board struct: AgcHardware impl; global singletons
                                  for LINK/BRIDGE/PLATFORM/BMI088/TIMER_HANDLES;
                                  #[panic_handler] -- defmt+sys_reset in dev, sys_reset in rel)
      state.rs                   (BridgeState: cached values from bridge messages;
                                  uplink and key queues; critical-section protected, ADR-008)
      bin/
        agc.rs                   (firmware entry point: clocks, USART6, IWDG, SPI3+BMI088,
                                  TIM2/3/4/5 (T-RUPTs) + TIM7 (IMU 1 kHz), SysTick,
                                  FRESH START / RESTART dispatch)
      link/
        mod.rs
        uart.rs                  (USART6 driver, PC6/PC7, 460800 baud, 8N1)
        dispatch.rs              (Inbound frame -> BridgeState updates, ISR-side)
      local/                     (peripherals owned by the AGC MCU itself)
        mod.rs
        timers.rs                (T3/T4/T5/T6 trait impls backed by STM32 TIM2/3/4/5)
        watchdog.rs              (IWDG wrapper: 1.024 s timeout)
        imu/
          mod.rs                 (BoardImu: Imu trait impl wiring BMI088 + platform emulator)
          bmi088.rs              (SPI3 driver for the accel + gyro dice)
      remote/                    (peripherals reached over the bridge link, ADR-009)
        mod.rs
        dsky.rs                  (Dsky trait impl -- Msg::Dsky* + BridgeState.key_queue)
        optics.rs                (Optics trait impl -- CDU + mark flag from BridgeState)
        engine.rs                (Engine trait impl -- SPS enable + gimbal Msg)
        rcs.rs                   (Rcs trait impl -- jet on/off Msg)
        uplink.rs                (Uplink trait impl -- BridgeState.uplink_queue)
        telemetry.rs             (Telemetry trait impl -- downlink word Msg)
  
  agc-bridge-pico/               (RP2040 bridge firmware -- #![no_std], non-flight helper)
    src/
      main.rs                    (entry point: UART0 link, USB-CDC console, heartbeat)
      link.rs                    (UART0 frame encode/decode at 460800 baud, GPIO0/1)
      console.rs                 (USB CDC-ACM developer console; non-blocking)
      keymap.rs                  (ASCII -> DSKY key code mapping, KEYTEMP1 5-bit codes)
      state.rs                   (BridgeState: tx_seq counter and other shared data)
  
  agc-sim/                       (Host-side simulator -- std allowed)
    src/
      lib.rs
      hardware.rs                (Simulated HAL implementation)
      dsky_ui.rs                 (Terminal-based DSKY simulator)
      scenario.rs                (Mission scenario loader)
      physics.rs                 (Simplified orbital mechanics for testing)
  
  agc-test/                      (Integration test harness)
    src/
      lib.rs
    tests/
      restart_recovery.rs
      navigation_accuracy.rs
      timing_compliance.rs
      dsky_interaction.rs
```


## 3. Type System Design

### 3.1 Numeric Types

The Rust port uses standard Rust numeric primitives throughout. There is no
custom `AgcWord` type.

| Use | Type | Notes |
|-----|------|-------|
| Navigation math (position, velocity, maneuver) | `f64` | Full double-precision; matches original double-word accuracy |
| Attitude / trig computations | `f64` | Matches interpretive-language double precision |
| Mission elapsed time | `u32` (centiseconds) | Integer counter; convert to `f64` seconds only at math call sites |
| Display and alarm values | `f32` | Single precision sufficient for crew displays |
| I/O channel words, counter cells | `u16` | Bit-field access; no arithmetic semantics |
| CDU gimbal angles from hardware | `u16` | Unsigned counts, full revolution = 2^16 counts (see 3.2) |
| Signed hardware quantities (PIPA counts, gyro pulses) | `i16` | Raw hardware units |

The original AGC's ones-complement format and fractional scaling were
constraints of the hardware, not requirements of the guidance algorithms.
Using `f64` preserves the numerical accuracy that mattered (double-precision
state vectors) without the bookkeeping overhead of scale factors and
overflow flags.

### 3.2 Newtypes for Physical Quantities

Physical quantities that have distinct units use newtypes to prevent
unit errors at compile time:

```rust
/// CDU gimbal angle. The original AGC used 15-bit ones-complement values
/// where a full revolution = 2^15 counts (scale factor B-1 revolutions).
/// In our Rust port we use u16 (twos-complement), so a full revolution
/// maps to 2^16 = 65536 counts, giving uniform angular resolution across
/// the full circle.
#[derive(Clone, Copy)]
pub struct CduAngle(pub u16);

impl CduAngle {
    pub fn to_radians(self) -> f64 {
        (self.0 as f64) * (core::f64::consts::TAU / 65536.0)
    }
}

/// Mission elapsed time in centiseconds (integer counter, wraps after ~497 days).
/// Convert to f64 seconds only at call sites that need it for math.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Met(pub u32);

/// Delta-V in meters per second.
#[derive(Clone, Copy)]
pub struct DeltaV(pub Vec3);
```

### 3.3 Vectors and Matrices

```rust
/// 3-component vector (position m, velocity m/s, delta-V m/s, etc.)
pub type Vec3 = [f64; 3];

/// 3x3 rotation matrix (REFSMMAT, coordinate frame transforms)
pub type Mat3x3 = [[f64; 3]; 3];
```

Helper functions for dot product, cross product, norm, and matrix-vector
multiply live in `math::linalg` and are plain `fn` calls, not methods on
wrapper types.


## 4. Hardware Abstraction Layer (HAL)

The HAL isolates the flight software from the target hardware. In production
(bare-metal), the HAL talks to memory-mapped I/O registers. In simulation, it
talks to a software model.

### 4.1 Core Trait

The HAL is split into focused sub-traits, one per physical subsystem. The
`AgcHardware` bound collects them. Each bare-metal implementation is a struct
that wraps the `embedded-hal` peripheral handles for the specific MCU.

```rust
/// Bound that the flight software requires of the platform.
/// Each associated type is a focused sub-trait; the bare-metal impl
/// wires these to `embedded-hal` peripherals.
pub trait AgcHardware {
    type Timers: Timers;
    type Dsky: Dsky;
    type Imu: Imu;
    type Optics: Optics;
    type Engine: Engine;
    type Rcs: Rcs;
    type Uplink: Uplink;
    type Telemetry: Telemetry;

    fn timers(&mut self) -> &mut Self::Timers;
    fn dsky(&mut self) -> &mut Self::Dsky;
    fn imu(&mut self) -> &mut Self::Imu;
    fn optics(&mut self) -> &mut Self::Optics;
    fn engine(&mut self) -> &mut Self::Engine;
    fn rcs(&mut self) -> &mut Self::Rcs;
    fn uplink(&mut self) -> &mut Self::Uplink;
    fn telemetry(&mut self) -> &mut Self::Telemetry;

    /// Reset the night-watchman timer. Call once per Executive loop.
    fn pet_watchdog(&mut self);

    /// Trigger a hardware restart.
    fn hardware_restart(&mut self) -> !;
}
```

Example sub-trait:

```rust
pub trait Imu {
    /// Read accumulated PIPA delta-V counts since last call (x, y, z).
    /// Destructive at the hardware level. Called by `Executive::run` every
    /// foreground iteration; counts are saturating-accumulated into
    /// `AgcState::pipa_counts`, which the SERVICER consumes every 2 s.
    fn read_pipa(&mut self) -> [i16; 3];
    /// Read the three CDU gimbal angles (outer, inner, middle).
    fn read_cdu(&self) -> [CduAngle; 3];
    /// Command gyro torque pulses for fine alignment on the given axis.
    fn torque_gyro(&mut self, axis: usize, pulses: i16);
    /// Command coarse CDU drive angles for platform slew.
    fn coarse_align(&mut self, commands: [i16; 3]);
    /// True if the platform is caged (power-up state).
    fn is_caged(&self) -> bool;
}
```

The Rust Embedded HAL (`embedded-hal` v1) provides the SPI/I2C/GPIO traits
used inside the bare-metal implementations. The flight software never calls
`embedded-hal` directly; it only calls `AgcHardware` sub-traits.

#### HAL Implementation Requirements

Bare-metal structs implementing the sub-traits must follow the Rust Embedded
HAL design patterns:

- **`free()` method** (C-FREE): Every non-`Copy` HAL wrapper must expose a
  `free()` method that consumes the wrapper and returns the raw peripheral,
  allowing it to be reclaimed or passed to other drivers.

- **`embedded-hal` trait impls** (C-HAL-TRAITS): Bare-metal structs must
  implement all applicable `embedded-hal` traits in addition to the custom
  sub-traits, so standard tooling (`probe-rs`, `defmt`, third-party drivers)
  can interact with them.

- **Alignment state as runtime enum** (C-IMU-ALIGN): The IMU's alignment
  lifecycle (Caged → CoarseAligned → FineAligned) is tracked at runtime in
  `AgcState::imu_alignment_state`, not via a typestate parameter on the HAL
  wrapper. The bare-metal `BoardImu` is a zero-sized handle; all state lives
  in `AgcState` (mutated by foreground code) and in the global
  `PLATFORM: Mutex<RefCell<PlatformEmulator>>` (mutated by the TIM7 ISR).
  Gating gyro torque on alignment is enforced by the SERVICER and the
  programs that call it, not by the type system. A future enhancement could
  lift this into a typestate parameter on a new wrapper type — see open
  items in §16.

- **PAC ownership** (C-PAC-LOCAL): The device PAC is a dependency of the board
  crate only (`agc-board-nucleo-f767` pulls in `stm32f7xx-hal`, which re-exports
  the F7 PAC as `stm32f7xx_hal::pac`). `agc-core` is hardware-agnostic by design
  (ADR-005) and has no PAC dependency. The PAC's `#[interrupt]` attribute is
  used directly from the board crate (`stm32f7xx_hal::pac::interrupt`, ADR-010)
  rather than re-exported through a workspace-wide alias.

### 4.2 Interrupt Model

The AGC has 10 program interrupts plus 29 counter interrupts. Program interrupts
are edge-triggered, latched, and serviced in a fixed-priority order. Only one
interrupt can be active at a time.

```rust
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Interrupt {
    /// TIME6 decremented to -0. Used for RCS jet timing.
    T6Rupt = 1,
    /// TIME5 overflow. Digital autopilot computations.
    T5Rupt = 2,
    /// TIME3 overflow. Waitlist task dispatch.
    T3Rupt = 3,
    /// TIME4 overflow. Periodic I/O (DSKY, IMU monitoring).
    T4Rupt = 4,
    /// Keyboard input from main DSKY.
    KeyRupt1 = 5,
    /// Keyboard input from nav DSKY or optics mark.
    KeyRupt2 = 6,
    /// Uplink word received from ground.
    UplinkRupt = 7,
    /// Telemetry end pulse -- ready for next downlink word pair.
    DownRupt = 8,
    /// Radar data ready.
    RadarRupt = 9,
    /// Hand controller / discrete input change.
    HandRupt = 10,
}
```

On Cortex-M, context save/restore is handled automatically by the NVIC
hardware. Each interrupt handler is a plain Rust function. The Executive owns
all interrupt service routines, which it registers at startup via the HAL.

The `#[interrupt]` attribute must be re-exported from the **device PAC crate**
(`stm32f7xx_hal::pac::interrupt` for this project — see C-PAC-LOCAL in §4.1
and ADR-010), not used directly from `cortex-m-rt`. Using the PAC's re-export
causes the compiler to verify that the interrupt name actually exists on the
target device, catching typos at compile time instead of silently producing
an unregistered handler at runtime.

The AGC interrupt names do **not** map trivially onto STM32 timer names. The
+1 offset below (TN-RUPT → TIM(N-1)) is forced by the F7 timer pool: only
TIM2 and TIM5 are 32-bit, and they're claimed by the two T-RUPTs that need
the range (T3 ≤ 163 s) or the tick rate (T6 = 0.625 ms at 108 MHz). The 16-bit
TIM3 and TIM4 take the remaining T4 and T5 slots. The authoritative mapping
table lives in `agc-board-nucleo-f767/src/local/timers.rs`.

```rust
// Correct: use the device crate's re-exported attribute (ADR-010)
use stm32f7xx_hal::pac::interrupt;

#[interrupt]
fn TIM2() {
    // T3RUPT -- Waitlist task dispatch (32-bit, ≤163 s range)
}

#[interrupt]
fn TIM3() {
    // T4RUPT -- periodic 120 ms I/O drain (DSKY, gyro drift comp)
}

#[interrupt]
fn TIM4() {
    // T5RUPT -- 100 ms DAP cycle (ADR-022)
}

#[interrupt]
fn TIM5() {
    // T6RUPT -- RCS jet pulse timer (32-bit @ 108 MHz, 0.625 ms ticks)
}
```

#### HardFault Handler

A `HardFault` handler must always be defined. The `ExceptionFrame` it receives
contains a full register snapshot including the program counter at the moment
of the fault. The handler follows the same GOJAM contract as the panic handler
(ADR-009): debug builds emit the PC/LR/XPSR over RTT (`defmt`) so the cause is
visible to an attached probe; release builds reset immediately with no output.
We never `loop {}` for the debugger — the IWDG watchdog would recover the MCU
in ~1 s anyway, and an immediate reset is closer to the AGC's behaviour.

The handler lives next to the SysTick exception in
`agc-board-nucleo-f767/src/bin/agc.rs`.

```rust
use cortex_m_rt::{exception, ExceptionFrame};

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    #[cfg(debug_assertions)]
    defmt::error!(
        "HardFault: pc={:#x} lr={:#x} xpsr={:#x}",
        ef.pc(),
        ef.lr(),
        ef.xpsr(),
    );
    #[cfg(not(debug_assertions))]
    let _ = ef;
    cortex_m::peripheral::SCB::sys_reset()
}
```

### 4.3 Peripheral Side Effects

The original AGC's I/O channels were hardware registers with physical side
effects (writing a jet channel fired a thruster; writing a display channel
drove relay coils). In the Rust port these side effects are encapsulated inside
each sub-trait implementation. The flight software calls typed methods (`rcs.fire_jets(...)`,
`dsky.write_relay(...)`) and never manipulates hardware registers directly.

The bare-metal HAL implementation translates each method call to the appropriate
MCU peripheral operation (GPIO, SPI, timer compare, etc.) using `embedded-hal`.
Timing constraints (e.g., the DSKY relay must be held for 20ms) are enforced
inside the HAL implementation, not by the calling flight-software code.


## 5. Executive and Waitlist -- The Real-Time Scheduler

The AGC has no operating system. Instead, two cooperating systems manage all
computation:

### 5.1 Waitlist (Tasks)

A **task** is a short, time-critical computation that runs to completion without
yielding. Tasks are scheduled to execute at a specific future time, measured in
centiseconds from "now." The hardware mechanism is TIME3: the software loads
TIME3 with (2^14 - delay_cs), and when TIME3 overflows, T3RUPT fires and the
Waitlist dispatcher runs the task.

The Waitlist maintains a table of pending tasks using a delta-time chain. Each
entry stores the time difference from the previous entry (not an absolute time).
The first entry's delta-time corresponds to the value loaded into TIME3. When
T3RUPT fires, the first entry is dispatched and the next entry's delta-time is
loaded into TIME3.

The original AGC supports 8 concurrent pending tasks (7 in the LST1/LST2
tables plus the one currently being dispatched). Each entry is a pair:
(delta-time, task-address).

```rust
pub const MAX_WAITLIST_TASKS: usize = 8;

pub struct WaitlistEntry {
    /// Time remaining until execution, in centiseconds.
    /// Stored as delta from previous entry (delta-time chain).
    pub delta_time: u16,
    /// Entry point of the task.
    pub task: fn(&mut AgcState),
}

pub struct Waitlist {
    entries: [Option<WaitlistEntry>; MAX_WAITLIST_TASKS],
    /// Number of active entries.
    count: u8,
}
```

Tasks are the highest-priority software activity after interrupt service routines.
A task may:
- Schedule another task (WAITLIST call).
- Establish a job (EXEC call: NOVAC or FINDVAC).
- Modify output channels directly (e.g., fire jets).
- NOT perform long computations or use the interpreter.

In Rust, tasks are function pointers (`fn(&mut AgcState)`). They execute with
interrupts inhibited (INHINT) during the dispatch, then may release interrupts
(RELINT) if they need to allow higher-priority work.

### 5.2 Executive (Jobs)

A **job** is a longer computation that can be preempted by tasks and interrupts.
Jobs run at assigned priorities (higher number = higher priority). The Executive
maintains a table of up to 7 jobs (the CORE SET table). Each job has its own
register save area.

The original AGC had two mechanisms for creating jobs:

- **NOVAC**: Creates a job without a VAC (Vector Accumulator) area. Used for
  jobs that only perform basic machine-language operations.
- **FINDVAC**: Creates a job with a VAC area -- a block of scratch workspace
  originally needed for the interpretive language's push-down list and vector
  accumulators. In the Rust port, since the interpreter is eliminated, FINDVAC
  is equivalent to creating a job with an associated scratch workspace struct
  for intermediate computation results.

If no empty job slots are available when NOVAC or FINDVAC is called, alarm
1202 is raised (see section 5.3).

```rust
pub const MAX_JOBS: usize = 7;

pub struct JobEntry {
    /// Priority. 0 = slot empty. Higher = more important.
    /// The dummy job (idle loop) runs at priority 0.
    pub priority: u8,
    /// The function implementing this job's computation.
    pub entry: fn(&mut AgcState),
    /// Major mode that owns this job (for restart dispatch).
    pub major_mode: u8,
}
// Note: the original AGC distinguished NOVAC vs FINDVAC jobs via a `has_vac`
// flag selecting whether a VAC scratchpad was allocated for the interpretive
// language. The interpreter is eliminated in this port (ADR-001), so no VAC
// pool exists and the field has been removed.
```

The Executive's main loop (`EXEC`) scans the job table for the highest-priority
ready job and runs it. When a job completes or yields (via `CHANG1` to change
priority, or task preemption), the scan repeats.

**The night-watchman timer** must be reset by the Executive main loop at least
once every ~1.28 seconds. If the loop stalls (infinite loop in a job), the
hardware triggers a restart. In the Rust port this is the `pet_watchdog()` call
in `AgcHardware`, wired to a hardware watchdog timer on the MCU.

```rust
impl Executive {
    /// The main scheduling loop. Never returns in normal operation.
    /// This is the "idle loop" that the AGC enters when no jobs are ready.
    pub fn run(&mut self, hw: &mut impl AgcHardware) -> ! {
        loop {
            // Sample NEWJOB -- resets night watchman
            hw.pet_watchdog();  // reset night-watchman timer
            
            if let Some(job_index) = self.find_highest_priority_job() {
                self.dispatch_job(job_index, hw);
            }
            // If no jobs ready, loop (the dummy job).
            // No jobs ready -- COMPUTER ACTIVITY light is off.
        }
    }
}
```

### 5.3 Priority Inversion and the 1202 Alarm

The original AGC famously encountered "Executive overflow" (alarm 1202) during
Apollo 11 when the rendezvous radar inadvertently consumed Executive slots. The
alarm system must be preserved:

- If a NOVAC or FINDVAC call finds no empty job slots, alarm 1202 is raised.
- If a FINDVAC call finds no free VAC areas, alarm 1210 is raised.
- The restart logic recovers by dropping the lowest-priority non-critical job.
- This is NOT a bug to be fixed; it is the correct design. The alarm tells the
  crew and ground that the computer is shedding load.


## 6. Restart Protection

The AGC can restart at any time due to:
- Parity failure in memory readout
- Night watchman timeout (NEWJOB not sampled)
- Software-initiated restart (TC GOJAM)
- Power transient

After restart, the software must return to a safe state without losing critical
navigation data. The mechanism is **restart groups and phase tables**.

### 6.1 Phase Tables

The software is divided into **restart groups** (numbered 1-6 in Comanche055).
Each group maintains a **phase register** that records which step of a multi-step
computation the group has reached. On restart, the RESTART routine reads all
phase registers and re-dispatches each group from its recorded phase.

```rust
pub const NUM_RESTART_GROUPS: usize = 6;

pub struct RestartProtection {
    /// Phase for each restart group. 0 = idle.
    /// Positive odd = re-dispatch as task; positive even = re-dispatch as job.
    /// Negative = restart group from the top of the phase.
    pub phases: [i16; NUM_RESTART_GROUPS],
}
```

### 6.2 Restart-Safe Coding Pattern

In the original AGC assembly, critical computations bracket themselves with
phase-change calls:

```
TC PHASCHNG       ; set phase for group N to value P
... critical work ...
TC PHASCHNG       ; advance phase
```

In Rust, this becomes a structured pattern:

```rust
fn critical_computation(state: &mut AgcState) {
    state.restart.set_phase(GROUP_3, Phase::new(1));
    
    // ... compute step 1 ...
    
    state.restart.set_phase(GROUP_3, Phase::new(3));
    
    // ... compute step 2 ...
    
    state.restart.set_phase(GROUP_3, Phase::IDLE);
}
```

### 6.3 Erasable Memory Protection

Navigation state vectors (position and velocity of CSM and target vehicle) are
the most critical data. They are stored in a protected region of erasable memory
and are designed to survive restarts. The integration routines use a "swap"
protocol: new state is computed into a temporary area, then atomically swapped
into the primary area after the phase register is updated. This ensures that a
restart mid-computation never corrupts the primary state.

### 6.4 FRESH START vs RESTART

The AGC distinguishes between two recovery modes:

- **FRESH START**: Complete reinitialization. All jobs are cleared, all tasks
  are cleared, all phase registers are zeroed. The state vectors and REFSMMAT
  are preserved if they were valid. The computer enters P00 (idle). This is
  invoked by the crew via VERB 36 ENTER or after a prolonged power loss.

- **RESTART**: Partial recovery. Phase registers are read and active
  computations are re-dispatched from their last recorded phase. The Waitlist,
  Executive, and DAP are restarted. Navigation state is preserved. This is the
  normal recovery from a transient event (GOJAM, parity fail, watchdog).

The RESTART sequence must complete within a bounded time (on the order of
100ms) to avoid missing critical DAP deadlines.


## 7. Navigation Programs (Major Modes)

Major modes (also called "programs" -- P00, P01, P11, etc.) represent the
high-level mission phases. The crew selects a major mode by keying VERB 37 NOUN
xx ENTER on the DSKY.

### 7.1 Program Lifecycle

```
VERB 37 ENTER -> V/N Processor -> Major Mode Switch -> Program Entry Point
                                                        |
                                                        v
                                                   [Establish Job]
                                                        |
                                                        v
                                                   [Schedule Tasks]
                                                        |
                                                   [Await Crew Input / Time]
                                                        |
                                                   [Perform Guidance]
                                                        |
                                                   [Display Results]
                                                        |
                                                   [Return to P00 or next P]
```

The V37 handler includes validity checking: not all program transitions are
permitted from all states. For example, entry programs (P61-P67) chain
automatically and cannot be entered individually via V37 mid-sequence.

### 7.2 Programs for the Command Module (Comanche055 Scope)

| Program | Description | Category |
|---------|-------------|----------|
| P00 | CMC Idling | Idle |
| P01 | Pre-launch IMU initialization | Pre-launch |
| P02 | Gyrocompassing | Pre-launch |
| P06 | Power-down | System |
| P11 | Earth orbit insertion monitor | Boost |
| P15 | TLI (Trans-Lunar Injection) monitor | Boost |
| P20 | Rendezvous navigation | Navigation |
| P21 | Ground track determination | Navigation |
| P22 | Orbital navigation (CM) | Navigation |
| P23 | Cislunar midcourse nav (star/landmark) | Navigation |
| P30 | External Delta-V | Targeting |
| P31 | Rendezvous maneuver (height adjust) | Targeting |
| P32 | Coelliptic sequence initiation | Targeting |
| P33 | Constant Delta Height | Targeting |
| P34 | Transfer Phase Initiation | Targeting |
| P37 | Return to Earth | Contingency |
| P40 | SPS thrusting | Maneuver |
| P41 | RCS thrusting | Maneuver |
| P47 | Thrust monitoring | Maneuver |
| P51 | IMU orientation determination | Alignment |
| P52 | IMU realignment | Alignment |
| P61 | Entry - Sixth Body fix (pre-entry preparation) | Entry |
| P62 | CM/SM Separation and Pre-entry Maneuver | Entry |
| P63 | Entry Initialization (0.05g detection) | Entry |
| P64 | Post-0.05g (up-control phase) | Entry |
| P65 | Up-control (ballistic-to-lifting transition) | Entry |
| P66 | Ballistic phase | Entry |
| P67 | Final phase (drogue deploy) | Entry |

### 7.3 Program Trait

Each major mode implements a common trait:

```rust
pub trait MajorMode {
    /// The program number (e.g., 11 for P11).
    fn number(&self) -> u8;
    
    /// Entry point: called when the crew selects this program.
    /// Returns a job priority for the Executive to use.
    fn start(&self, state: &mut AgcState) -> JobPriority;
    
    /// Called if the program needs to handle a DSKY verb/noun
    /// while it is the active major mode.
    fn handle_display_input(&self, state: &mut AgcState, verb: u8, noun: u8);
    
    /// Called on restart to resume from the recorded phase.
    fn restart_resume(&self, state: &mut AgcState, phase: Phase);
    
    /// Cleanup: called when switching away from this major mode.
    fn terminate(&self, state: &mut AgcState);
}
```

### 7.4 SERVICER (Average-G)

The SERVICER is not a major mode but a critical background process that runs
as a repeating task on a 2-second cycle. It reads the PIPA accelerometer
counts, transforms them from platform coordinates to the computational
coordinate frame, and integrates the state vector.

The SERVICER is established by programs that need navigation (P11, P20, P40,
entry programs) and is cancelled when navigation is not needed.

```
Every 2 seconds:
  1. Read PIPA counts (PIPAX, PIPAY, PIPAZ) and reset counters
  2. Apply PIPA compensation (bias, scale factor, misalignment)
  3. Rotate from platform frame to reference frame (REFSMMAT)
  4. Add gravity acceleration (computed from current state vector)
  5. Integrate state vector (position += velocity*dt + 0.5*accel*dt^2, etc.)
  6. Update displays (V06N63 or similar, depending on active program)
  7. Call any program-specific SERVICER exit routine (e.g., cross-product
     steering updates during P40)
  8. Reschedule self for T+2 seconds
```

The gravity computation in step 4 uses the model described in section 9.4.
During coasting flight (no thrusting), the SERVICER may not be active; instead,
the state vector is propagated by the conic integration routines on demand.


## 8. Memory Layout Strategy

### 8.1 Static Allocation Only

All memory is statically allocated. There is no heap, no `alloc` crate, no
`Vec`, no `Box`. Every data structure has a fixed, compile-time-known size.

AGC memory addresses are irrelevant in the Rust port. Hardware registers,
counter cells, and I/O channels are accessed through `AgcHardware` sub-trait
methods; the MCU's memory-mapped register addresses are hidden inside the
HAL implementation. All navigation and scheduler state lives in ordinary
named Rust variables and struct fields.

### 8.2 AgcState -- The Central State Structure

Rather than using global mutable statics scattered across modules, all mutable
state is collected into a single structure that is threaded through the call
hierarchy. This makes the data flow explicit and testable.

The full struct is defined in `agc-core/src/lib.rs`. The excerpt below is
illustrative -- the actual `AgcState` has roughly 30 fields covering TVC
filter state, SPS burn state, IMU alignment lifecycle, RCS staging fields,
PIPA calibration, P20/P22 rendezvous state, entry state, the V/N processor,
and more. Refer to `lib.rs` for the authoritative list.

```rust
pub struct AgcState {
    // ── Scheduler ─────────────────────────────────────────────────────────
    pub executive: Executive,
    pub waitlist: Waitlist,
    pub restart: RestartProtection,

    // ── Navigation ────────────────────────────────────────────────────────
    pub csm_state: StateVector,      // CSM position (m) and velocity (m/s)
    pub target_state: StateVector,   // Target vehicle state (LM or landmark)
    pub refsmmat: Mat3x3,            // Reference stable member matrix
    pub time: Met,                   // Mission elapsed time (centiseconds)

    // ── Guidance and control ──────────────────────────────────────────────
    pub major_mode: u8,              // Current program number
    pub dap_state: DapState,
    pub tvc_state: TvcState,
    pub burn: BurnState,             // SPS burn execution (P40)

    // ── Crew interface, alarms, flags ─────────────────────────────────────
    pub dsky: DskyState,             // Current display, verb, noun, registers
    pub alarm: AlarmState,
    pub flagwords: [u16; 12],        // FLAGWRD0..FLAGWRD11 bit-field words

    // ... (~20 more fields; see agc-core/src/lib.rs)
}
```

### 8.3 Read-Only Data

Constant tables (star catalog, verb/noun tables, alarm code definitions,
trigonometric constants) are declared `static` or `const` in their respective
modules and placed in flash by the linker. The AGC's bank-switching scheme is
not relevant; the Cortex-M flat address space makes all flash uniformly
accessible.


## 9. Navigation Math -- Replacing the Interpreter

The AGC's interpretive language existed to provide double-precision vector and
matrix operations on a 15-bit CPU that had no such instructions. That problem
does not exist on a modern target. The interpreter is **not re-implemented**.

### 9.1 Replacement Strategy

Every computation that was written in the AGC interpretive language is
reimplemented as a plain Rust function using `f64`. The math module provides
the building blocks:

| Interpreter construct | Rust equivalent |
|-----------------------|-----------------|
| `VLOAD`, `VSTORE` | `let v: Vec3 = ...` / assignment |
| `VAD`, `VSU`, `VSCALE` | `linalg::vadd`, `linalg::vsub`, `linalg::vscale` |
| `DOT`, `CROSS` | `linalg::dot`, `linalg::cross` |
| `UNIT` | `linalg::unit` |
| `MXV`, `VXM` | `linalg::mxv`, `linalg::vxm` |
| `SINE`, `COSINE` | `f64::sin`, `f64::cos` |
| `ASIN`, `ACOS` | `f64::asin`, `f64::acos` |
| `SQRT` | `f64::sqrt` |
| `CALL` / `RETURN` | normal Rust function calls |
| Push-down list | Rust call stack |

### 9.2 Function Granularity

The navigation and guidance modules expose functions that correspond
one-to-one to the AGC's interpretive subroutines. For example, the KEPSILON
routine (Kepler equation solver) becomes:

```rust
/// Solve Kepler's equation for the universal variable.
/// Inputs are in SI units (m, m/s). Returns the state at time `dt`.
pub fn kepler_step(r0: Vec3, v0: Vec3, dt: f64, mu: f64) -> (Vec3, Vec3) { ... }
```

The DAP, SERVICER, targeting, and entry guidance functions follow the same
pattern: named Rust functions with `f64` parameters, called directly from
program and task code.

### 9.3 No Interpreter State

There is no `InterpreterState` struct, no MPAC accumulator, no push-down list,
and no mode register. All intermediate values are local variables or function
parameters on the Rust call stack.

### 9.4 Gravity Model

The AGC used a restricted gravity model for computational efficiency:

- **Earth**: Point-mass gravity plus J2 oblateness term (Earth's equatorial
  bulge). Higher-order terms (J3, J4) were not included. The J2 model adds
  a perturbation that depends on the sine of geocentric latitude.

- **Moon**: Point-mass gravity only in the primary model. The Moon's
  non-spherical gravity field (mascons) was not modeled by the AGC; trajectory
  errors from this were corrected by midcourse maneuvers based on ground
  tracking.

- **Third body**: When in the Earth's sphere of influence, the Moon's gravity
  is included as a third-body perturbation (and vice versa). The sphere of
  influence boundary determines which body is primary.

The integration uses Cowell's method (direct integration of the total
acceleration) for the SERVICER/Average-G cycle, and Encke's method (integration
of the deviation from a reference conic) for longer-term coast propagation.


## 10. DSKY Interface and the Verb/Noun System

The DSKY (Display and Keyboard) is the crew's sole interface to the computer.
It consists of:

**Output**: Electroluminescent display with PROG, VERB, NOUN (2 digits each),
R1, R2, R3 (5 digits each, signed), plus indicator lights. The original AGC
panel had UPLINK ACTY, NO ATT, STBY, KEY REL, OPR ERR, TEMP, GIMBAL LOCK,
PROG, RESTART, TRACKER, ALT, VEL, and COMP ACTY. The Rust port's `hal::dsky::Lamp`
enum implements ten of these (UplinkActivity, NoAtt, Stby, KeyRel, OprErr,
Restart, GimbalLock, Temp, ProgAlarm, CompActy); TRACKER lives as a field on
`DskyState` but is not in the `Lamp` enum yet; ALT and VEL are LM-specific and
out of scope for the CSM Comanche055 target.

**Input**: 19 keys (0-9, VERB, NOUN, +, -, ENTER, CLR, PRO, KEY REL, RSET).

### 10.1 Display Driving

The original AGC drove the electroluminescent display via channels 10 and 11
(octal), which actuated a relay matrix. The Rust port replaces the relay
matrix with a per-field bridge-link encoding (ADR-019): 21 rows carry the
PROG/VERB/NOUN fields and the three register sign+digit columns, ten lamp
messages carry the indicator lights, and one flash message controls VERB/NOUN
flashing.

Display updates are emitted by the T4RUPT periodic handler. On each 120 ms
invocation the handler decodes the current `DskyState` into a `DskyFrame` via
`services::pinball::decode_dsky` and pushes the frame to the bridge only when
it differs from the previous one (ADR-020 item 4 — rate limiting). See §13.4
for the full T4 cycle, and `services/pinball.rs` for the row encoding.

### 10.2 Verb/Noun Processing (PINBALL)

The PINBALL system processes crew keyboard input. Verbs specify actions; nouns
specify data items. Common verbs:

| Verb | Meaning |
|------|---------|
| V01 | Display octal in R1 |
| V04 | Display octal in R1, R2 |
| V05 | Display octal in R1, R2, R3 |
| V06 | Display decimal in R1, R2, R3 |
| V16 | Monitor decimal in R1, R2, R3 (auto-refresh) |
| V21 | Load component 1 (into R1) |
| V22 | Load component 2 (into R2) |
| V23 | Load component 3 (into R3) |
| V24 | Load component 1, 2 (into R1, R2) |
| V25 | Load component 1, 2, 3 (into R1, R2, R3) |
| V32 | Recycle (repeat current program display) |
| V33 | Proceed without data input |
| V34 | Terminate current program |
| V35 | Test lights (illuminate all segments) |
| V36 | Fresh start (crew-initiated FRESH START) |
| V37 | Change major mode (program select) |
| V50 | Please perform (request crew action) |
| V82 | Request orbital parameters display |

The Verb/Noun processor is a state machine:

```
Idle -> Verb Digit 1 -> Verb Digit 2 -> Noun Digit 1 -> Noun Digit 2 -> ENTER
                                                                          |
                                                               [Dispatch to verb handler]
```

The PINBALL system supports two concurrent displays: a "normal" display
(driven by the active program) and a "monitor" display (driven by a V16-type
verb). When the crew presses KEY REL, the normal display is released back to
the program. The DSKY has a "KEY REL" indicator light that illuminates when
the program is waiting for the display.

### 10.3 Flashing Display

When the computer needs crew input, it flashes the VERB and NOUN indicators via
`dsky.set_flash(true)`. The crew responds by entering data and pressing ENTER,
or by pressing PRO (proceed without input), or KEY REL (release display for
background use).

### 10.4 Extended Verbs

Verb numbers 40 and above are "extended verbs" that do not use nouns. They are
dispatched through a separate table (ETEFLAG table in the original). Extended
verbs include V46 (establish DAP data), V48 (request DAP data load), V49
(crew-defined maneuver), V82 (orbital parameters request), and others.


## 11. Digital Autopilot (DAP)

The DAP runs on T5RUPT (100 ms attitude-control cycle) and T6RUPT (one-shot
RCS jet pulse timer). Each TIM4 (T5RUPT) interrupt sets `T5_PENDING`; the
Executive's foreground drain pre-reads the CDU, calls `dap_step`, and re-arms
TIM4 when `dap_state.mode != Off` (ADR-017 Strategy D + ADR-022). The DAP is
deliberately on its own hardware interrupt — not on the Waitlist — so a
Waitlist-saturated (1211) condition cannot stop attitude control.

### 11.1 CSM DAP Modes

For the Command Module, the DAP operates in several modes:

**Coast DAP (RCS)**:
- Rate damping: null body rates using RCS jets
- Attitude hold: maintain a target attitude quaternion
- Maneuver: rotate to a commanded attitude at a controlled rate

The Coast DAP runs on the T5RUPT 100ms cycle. At each cycle it reads the CDU
angles, computes body rates by differencing successive CDU readings, compares
against the desired rates or attitude, and issues jet commands.

**Thrust DAP (TVC)**:
- During SPS burns, the DAP controls the SPS engine gimbal through the TVC
  system. Pitch and yaw gimbal angles are commanded via `engine.sps_gimbal(pitch, yaw)`
  to steer the thrust vector through the vehicle center of mass.
- The TVC DAP includes a digital filter (lead-lag compensator) to provide
  stability. The filter processes the attitude error signal before commanding
  the engine gimbal actuators. Filter coefficients are tunable constants stored
  in erasable memory (in our port: fields on `TvcState`).
- TVC also includes trim tracking: as propellant is consumed, the vehicle
  center of mass shifts, and the TVC system adjusts the trim position
  accordingly.

### 11.2 RCS Jet Selection

The jet select logic maps desired torques to individual jet firings. The SM RCS
has 16 jets in 4 quads (A, B, C, D); the CM RCS has 12 jets in 2 rings. During
most of the mission (earth orbit, TLI, cislunar coast, lunar orbit), the SM RCS
is used. The CM RCS is used only during entry, after SM separation.

Jet failures (sensed or commanded off by the crew via V46) are handled by the
jet select logic, which reconfigures to available jets. The DAP configuration
(rate deadband, attitude deadband, number of jets per axis) is set by the crew
via V46/V48 DSKY entries.

Jet commands are issued via `rcs.fire_jets(jet_mask)`.

### 11.3 Timing

T6RUPT provides fine-resolution timing for RCS jet on/off commands. In the
original AGC, TIME6 was decremented at 1600 pulses per second, giving 0.625ms
resolution per count. The DAP arms T6 with the desired jet-on duration via
`timers.arm_t6(counts)`, fires the jets, and the T6RUPT handler calls
`rcs.quench_jets()` to turn them off.

T6 is a one-shot timer: it must be explicitly armed for each jet firing.
Between firings, T6 is disabled (the ENABLE T6 bit in channel 13 must be set
to activate it).


## 12. Error Handling and Alarms

### 12.1 Rust Panic Handler

The `#[panic_handler]` triggers GOJAM (hardware restart). The handler is
profile-specific: debug builds log the panic message via `defmt` (RTT) so the
cause is visible to an attached probe; release builds restart immediately with
no output overhead.

The handler lives in the board crate (`agc-board-nucleo-f767/src/lib.rs`), not
in `agc-core`. `agc-core` is compiled with `std` enabled under `cfg(test)` so
host-side unit tests can use the standard panic; defining `#[panic_handler]`
there would conflict with that. The board crate is `#![no_std]` unconditionally
and is the only crate linked into the firmware binary, so it is the natural
home for the handler.

Do **not** add `panic-probe`, `panic-halt`, or any other panic-handler crate as
a dependency -- only one `#[panic_handler]` is permitted per binary, and those
crates provide one automatically (causing a link error). Their default
behaviour is also wrong for flight: `panic-probe` halts at a `udf` for the
debugger and `panic-halt` spins in `loop {}`. Neither resets, so the immediate
GOJAM the design requires would be replaced by an indeterminate wait for the
IWDG watchdog.

```rust
// dev profile: log the message over RTT, then restart.
// A probe attached via `probe-rs` will receive the defmt frame before reset.
#[cfg(debug_assertions)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("PANIC: {}", defmt::Display2Format(info));
    cortex_m::peripheral::SCB::sys_reset()
}

// release profile: restart immediately; no output, minimal binary size.
#[cfg(not(debug_assertions))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}
```

### 12.2 Program Alarms

The AGC software raises alarms (via the ALARM routine) for non-fatal conditions:

| Code | Meaning |
|------|---------|
| 0206 | Celestial body too close to Sun for sighting |
| 0210 | IMU not aligned (REFSMMAT invalid) |
| 0220 | Optics error -- mark rejected |
| 0401 | Failed to converge (navigation integration) |
| 0404 | Invalid orbit (sub-parabolic, etc.) |
| 1202 | Executive overflow (no free job slots) |
| 1210 | Executive overflow (no free VAC areas) |
| 1211 | Waitlist overflow (no free task slots) |

Alarms display on the DSKY PROG register and may or may not abort the current
program depending on severity.

### 12.3 Design for Robustness in Rust

Rust's type system handles several classes of error that the AGC had to manage
by convention:

- **Array bounds**: Compile-time-fixed arrays with index checks (panic on
  out-of-bounds triggers restart, same as original behavior).
- **Null pointers**: `Option<T>` eliminates null-pointer errors in the job
  table, waitlist, etc.
- **Uninitialized memory**: Rust's ownership system prevents reading
  uninitialized state.
- **Arithmetic overflow**: Rust's integer overflow semantics apply. In debug
  builds, integer overflow panics (triggering a restart). In release builds,
  wrapping arithmetic is used where overflow is possible and expected (e.g.,
  `Met` centisecond counter). Navigation code uses `f64`, which handles
  out-of-range values through IEEE 754 infinity and NaN; callers check for
  these where the result feeds into a safety-critical decision.

However, Rust's safety guarantees do NOT replace the need for restart
protection. A power glitch that corrupts RAM will corrupt Rust data structures
just as it corrupted AGC erasable memory. The phase-table restart mechanism
must be preserved.


## 13. Timing Architecture

### 13.1 Clock Source

The AGC timing model requires several interrupt sources:

| Timer | Period | Use |
|-------|--------|-----|
| T3 (TIME3) | Variable (loaded per Waitlist scheduling) | Waitlist task dispatch (T3RUPT) |
| T4 (TIME4) | 120ms cycle | Periodic I/O: DSKY display, IMU monitoring, gyro drift compensation (T4RUPT) |
| T5 (TIME5) | ~100ms | Digital autopilot cycle (T5RUPT) |
| T6 (TIME6) | On-demand, 0.625ms resolution | RCS jet pulse timing (T6RUPT) |

On the Cortex-M target these are implemented with hardware timer peripherals
configured via `embedded-hal`. The `Timers` sub-trait exposes
`arm_t3(centiseconds)`, `arm_t5(centiseconds)`, and `arm_t6(counts)` methods;
the HAL implementation maps these to MCU timer registers.

### 13.2 Interrupt Timing Budget

The AGC has a strict timing budget. Interrupts must complete quickly to avoid
missing the next event.

| Source | Period | Budget |
|--------|--------|--------|
| T6RUPT | 0.625ms (on demand) | ~0.5ms max |
| T5RUPT | ~100ms | ~20ms max |
| T3RUPT | variable (10ms min) | ~5ms for dispatch, task runs in foreground |
| T4RUPT | 120ms | ~10ms per cycle |
| KeyRupt | async (key press) | ~1ms |
| DownRupt | ~10ms | ~0.5ms |

Tasks and interrupt handlers must not block. If a computation is too long for
a task, it must be established as a job.

### 13.3 WAITLIST Timing

Waitlist tasks are scheduled with centisecond resolution (10ms). The minimum
useful delay is 1 centisecond. The maximum is 16383 centiseconds (~163 seconds)
due to the 14-bit TIME3 counter.

For delays longer than 163 seconds, the "long waitlist" mechanism chains
tasks: a task reschedules itself for 163 seconds repeatedly until the remaining
delay is small enough for a single waitlist call.

### 13.4 T4RUPT Cycle Work

Each 120 ms T4RUPT invocation performs three fixed actions (see
`agc-core/src/executive/scheduler.rs` for the implementation):

1. **Advance MET** by 12 centiseconds (= 120 ms).
2. **Apply gyro drift compensation** — scale `NBDX/NBDY/NBDZ` by the
   centiseconds elapsed since the last drift application and torque the
   platform via `hw.imu().torque_gyro` on each axis where the pulse count
   is non-zero.
3. **Emit the DSKY frame** to the bridge, but only when the decoded frame
   differs from the previous one (ADR-020 item 4 — rate limit). The
   encoding is 21 rows + 10 lamp messages + 1 flash message per ADR-019;
   see `services/pinball.rs`.

DSKY keyboard input is drained every foreground iteration (not gated by
T4), so key latency is bounded by the Executive loop period, not the T4
cycle. The Coast DAP runs on T5RUPT at 100 ms cadence (ADR-022, §11.1),
not on T4. The original AGC's rotating "DSPTAB / WAITLIST" task chain in
T4RUPT is not implemented in the Rust port: the bridge handles all 21
rows per cycle, removing the need for relay-row sequencing, and IMU
monitoring / optics CDU / downlink assembly are open milestones.


## 14. Build System and Embedded Target

### 14.1 Rust Embedded Ecosystem

The project targets the [Rust Embedded](https://github.com/rust-embedded)
ecosystem. Key dependencies:

| Crate | Role |
|-------|------|
| [`cortex-m`](https://github.com/rust-embedded/cortex-m) | Core Cortex-M primitives: interrupt enable/disable, SysTick, `Mutex`, critical sections |
| [`cortex-m-rt`](https://github.com/rust-embedded/cortex-m-rt) | Startup, reset handler, `#[entry]`, `#[exception]` macros |
| [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) | Trait abstractions for GPIO, SPI, I2C, UART (used in `agc-hal`) |
| [`stm32f7xx-hal`](https://github.com/stm32-rs/stm32f7xx-hal) | F7 device HAL + re-exported PAC -- provides `#[interrupt]` attribute with compile-time name verification (used by the board crate only; see C-PAC-LOCAL in §4.1) |
| [`defmt`](https://github.com/knurling-rs/defmt) + [`defmt-rtt`](https://github.com/knurling-rs/defmt) | Efficient structured logging over RTT (development and panic handler; cf. ADR-009) |

The minimum target is **Cortex-M7 with double-precision FPU** (e.g.,
STM32H743, STM32F767) to guarantee hardware `f64` operations within the DAP
timing budget. Cortex-M4F has only a single-precision (f32) FPU; f64 on M4F
would require software emulation, which is approximately 10x slower and would
violate the DAP's 100ms deadline for attitude computations. Cortex-M33 targets
may also be used if they include the optional double-precision FPU extension.

Note: If a future design decision restricts navigation math to `f32` (which
provides ~7 decimal digits -- less than the AGC's double-word ~9 digits),
then Cortex-M4F becomes viable. This would require careful analysis of
numerical accuracy in state vector propagation and is NOT the current baseline.

#### Processor Choice Justification

The target choice is driven by the binding constraint of the Digital Autopilot:
the DAP must complete a full attitude/velocity computation cycle in less than
100 ms, and its math is `f64`-heavy (see §9.4 Gravity Model and §3.1 Numeric
Types). The following comparison against the original AGC and a common
low-cost alternative (ESP8266) shows why Cortex-M7 is the only viable choice.

**Raw clock and instruction rate**

| Processor                   | Master Clock       | Effective Instruction Rate  |
|-----------------------------|--------------------|-----------------------------|
| AGC (Block 2)               | 2.048 MHz          | ~85,000 basic instr/sec (MCT = 11.72 μs) |
| ESP8266 (Tensilica L106)    | 80 MHz (up to 160) | ~80–160 MIPS                |
| Cortex-M7 (STM32H743)       | 480 MHz            | ~960 MIPS (dual-issue)      |

- ESP8266 vs AGC: ~80× faster clock (160× overclocked)
- Cortex-M7 @ 480 MHz vs AGC: ~470× faster clock, ~11,000× faster effective
- Cortex-M7 vs ESP8266: ~3–6× clock, ~6–12× MIPS

**Architecture features relevant to the AGC port**

| Feature                 | AGC             | ESP8266          | Cortex-M7              |
|-------------------------|-----------------|------------------|------------------------|
| Word size               | 15-bit 1's comp | 32-bit RISC      | 32-bit ARM             |
| Hardware multiply       | Yes (46.9 μs)   | Yes (1 cycle)    | Yes (1 cycle)          |
| Hardware divide         | Yes (82.0 μs)   | No (software)    | Yes (2–12 cycles)      |
| **FPU f32**             | None            | None (soft-float)| **Hardware (1–4 cyc)** |
| **FPU f64**             | None            | None (soft-float)| **Hardware (5–20 cyc)**|
| Deterministic timing    | Yes             | No (cache + WiFi)| Yes (ITCM/DTCM)        |
| RAM                     | 2 KB            | ~80 KB           | 1 MB SRAM typical      |
| Flash                   | 36 KB (rope)    | 1–4 MB           | 2 MB typical           |

**Relative performance against the AGC baseline**

| Metric              | AGC              | ESP8266 (160 MHz)   | Cortex-M7 (480 MHz) |
|---------------------|------------------|---------------------|---------------------|
| Raw clock           | 1×               | 78×                 | 234×                |
| Integer MIPS        | 1×               | ~1,900×             | ~11,000×            |
| **f64 throughput**  | **1× (fixed-pt)**| **~2× (soft-float)**| **~5,000× (HW FPU)**|
| DAP timing margin   | ~0× (tight)      | Negative (misses)   | ~100×               |

#### ESP8266 Rejection (recorded so the question does not recur)

The ESP8266's 80–160 MHz clock and ~80 KB RAM superficially look like a
significant upgrade from the AGC. At the raw-integer level it is ~1,900× faster
than the AGC. For our workload it is not viable:

1. **No FPU**: The ESP8266 performs `f32` and `f64` operations entirely in
   software. A soft-float `f64` multiply on the Tensilica L106 takes
   ~50–100 clock cycles, giving a net `f64` throughput of only ~2× the
   original AGC (which used 15-bit fixed-point). The DAP, SERVICER, and
   gravity computations would either miss their 100 ms deadline under
   adversarial conditions or force the project to abandon `f64` for the
   navigation math — re-introducing the scale-factor bookkeeping we
   explicitly designed out (ADR D3).

2. **Non-deterministic timing**: The ESP8266 has an instruction cache and
   shares the CPU with the WiFi stack via higher-priority interrupts. Both
   introduce latency jitter that is hostile to hard real-time scheduling.
   The AGC's cooperative Executive + Waitlist assumes deterministic interrupt
   latency; reproducing that on an ESP8266 would require disabling the WiFi
   stack, losing the main reason someone would pick this part.

3. **Marginal RAM**: The full `AgcState` struct plus stacks plus fixture data
   plus the ESP-IDF system overhead (~40 KB) approaches the 80 KB RAM limit.
   There is no comfortable margin for adding program-specific workspace.

4. **No ecosystem fit**: The project targets the
   [Rust Embedded](https://github.com/rust-embedded) ecosystem (§14.1). The
   ESP8266 is not a Cortex-M and the `cortex-m` / `cortex-m-rt` crates do not
   apply. An alternative Rust toolchain exists (`esp-rs`), but does not share
   the `#[interrupt]`, NVIC, or `embedded-hal` v1 conventions the rest of the
   architecture is built on.

**Conclusion**: Cortex-M7 with double-precision FPU gives ~5,000× the f64
throughput of the original AGC and ~100× DAP timing margin. The ESP32-S3 is
a possible secondary target because it has a hardware FPU; but the ESP8266,
specifically, is rejected and should not be revisited without first
re-opening ADR D3 (native `f64` vs fixed-point arithmetic).

### 14.2 Feature Flags

```toml
[features]
default = ["sim"]
sim = ["std"]              # Host simulation -- std allowed
bare-metal = []            # No std, no heap, hardware target
```

The `agc-core` crate is always `#![no_std]`. The `sim` feature enables
`agc-sim` to link against it with a hosted HAL.

### 14.3 `#![no_std]` Constraints

Running on `cortex-m-rt` with no OS imposes hard rules:

- **No heap**: `alloc` is not used. All data structures are statically sized.
- **No threads**: The execution model is single-threaded + interrupts.
  Raw `static mut` is avoided in application code (the firmware binary's
  `static mut AGC_STATE` is the one allowed exception, justified by a
  SAFETY comment because no ISR touches `AGC_STATE`). All ISR-shared
  mutable state lives in `cortex_m::interrupt::Mutex<RefCell<T>>` (heap-
  free; `Mutex` and `RefCell` are plain stack/static structs with zero
  allocation overhead). Access always goes through `interrupt::free(|cs| ...)`,
  which provides a `CriticalSection` token the compiler requires before
  the `Mutex` will yield its contents. The board crate's ISR-shared
  statics (`agc-board-nucleo-f767/src/lib.rs`):

  | Static | Type | Touched by |
  |---|---|---|
  | `BRIDGE` | `Mutex<RefCell<BridgeState>>` | USART6 ISR (writes), HAL impls (reads) |
  | `LINK` | `Mutex<RefCell<Option<UartLink>>>` | USART6 ISR + foreground sends |
  | `PLATFORM` | `Mutex<RefCell<PlatformEmulator>>` | TIM7 ISR (writes), `BoardImu` reads |
  | `BMI088` | `Mutex<RefCell<Option<Bmi088Driver>>>` | TIM7 ISR only |
  | `TIMER_HANDLES` | `Mutex<RefCell<Option<TimerHandles>>>` | every TIM ISR + `Timers` impl |

  `AgcState` (DAP, alarm, DSKY, executive, …) is NOT in a `Mutex<RefCell>` —
  per Strategy D (ADR-017) it is mutated only by foreground code, so the
  borrow checker enforces exclusion at compile time without a runtime lock.

- **No async/await**: Rust's `async`/`await` and any executor (Tokio, Embassy, etc.)
  are prohibited. All concurrency is expressed exclusively through waitlist tasks
  and executive jobs. These are the only two scheduling primitives. Code that
  needs deferred or concurrent execution must be structured as a task (short,
  time-triggered) or a job (longer, priority-scheduled) -- nothing else.
- **No `f64` soft-float**: The linker must target a hard-float ABI with
  double-precision FPU support. Soft-float `f64` is approximately 10x slower
  and breaks the DAP deadline.
- **No unwinding**: `panic = "abort"` in `Cargo.toml`; panics trigger GOJAM.
- **Stack size**: The entire call stack (navigation programs + DAP) must fit
  in the MCU's RAM. Navigation functions must not recurse deeply. The deepest
  call chain (SERVICER -> orbit integrator -> Kepler solver -> linalg) must be
  bounded and measured.
- **Interrupt vectors**: Defined via `#[interrupt]` re-exported from the device
  PAC crate (`stm32f7xx_hal::pac::interrupt`), not from `cortex-m-rt` directly.
  This gives compile-time verification that the interrupt name exists on the
  target device. The four AGC scheduler interrupts map to STM32F7 timers as
  follows: **T3RUPT → TIM2** (32-bit, for the ≤163 s Waitlist range),
  **T4RUPT → TIM3** (periodic 120 ms), **T5RUPT → TIM4** (100 ms DAP cycle,
  ADR-022), **T6RUPT → TIM5** (32-bit at 108 MHz for the 0.625 ms RCS jet
  tick). The +1 offset is forced
  by hardware: only TIM2 and TIM5 are 32-bit on F7, and they're claimed by the
  two T-RUPTs that need long range (T3) and fast tick (T6). Authoritative
  mapping table: `agc-board-nucleo-f767/src/local/timers.rs`.

### 14.4 Linker Script

```
MEMORY {
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K  /* program + const tables (F767ZI) */
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 512K   /* AgcState + stacks */
}
```

The numbers match the Nucleo-F767ZI (ADR-021). The verbatim script lives in
`agc-board-nucleo-f767/memory.x`.

The `AgcState` struct is placed in a named RAM section so the linker can
verify it fits. Stack size is set conservatively and measured with a watermark
pattern during integration testing.

### 14.5 Testing Strategy

1. **Unit tests** (`#[cfg(test)]`): Run on host. Test individual math functions,
   channel bit-field parsing, waitlist ordering, and program logic.

2. **Integration tests** (`agc-test` crate): Run complete mission scenarios
   against `agc-sim` on the host. Verify navigation accuracy, timing compliance,
   and restart recovery.

3. **Hardware-in-the-loop**: Run on target MCU connected to a simulated
   IMU/DSKY via `probe-rs`. Verify real-time compliance and stack depth.


## 15. Key Architectural Decisions and Trade-offs

### D1: Interpreter Elimination

**Decision**: Implement all navigation and guidance computations as direct Rust
functions using `f64`. Do not re-implement the AGC interpretive language VM.

**Rationale**: The interpreter was a space-saving measure on a 15-bit, 2K-ROM
machine. Its bytecode dispatch, push-down list, and MPAC register add complexity
with no benefit on a modern target. Native `f64` functions are faster, smaller,
type-checked, and directly testable.

**Trade-off**: The translation from interpretive code to Rust requires careful
understanding of each routine's numerical intent. Mitigation: unit tests compare
outputs against VirtualAGC reference runs for known mission profiles.

**Justification from AGC architecture**: The interpreter occupied a significant
portion of the fixed-memory program space and CPU time. O'Brien documents that
each interpretive instruction took multiple MCTs to decode and execute through
the EXEC interpretive dispatch loop. The interpreter maintained substantial
state: MPAC (multi-purpose accumulator), MODE register, LOC/BANKSET (program
counter), and a push-down list (stack) of 8 levels. All of this existed solely
to simulate a double-precision vector/matrix computer on 15-bit hardware. With
native `f64` and a real stack, this entire subsystem -- the dispatch tables,
the address resolution logic, the mode switching, the push-down list overflow
checking -- becomes unnecessary overhead. The only risk is in faithfully
translating the numerical algorithms themselves, which is mitigated by
reference testing.

### D2: Single AgcState Structure

**Decision**: Collect all mutable state into one structure, passed by mutable
reference.

**Rationale**: Eliminates `static mut` and `unsafe` from most of the codebase.
Makes the data flow explicit. Enables unit testing without global state.

**Trade-off**: The single-owner model conflicts with the AGC's concurrent
access patterns (interrupt handlers modify state that foreground jobs also
read). Interrupt handlers are called by hardware and cannot receive function
arguments, so shared state must be in `static` variables. The mechanism is
`cortex_m::interrupt::Mutex<RefCell<T>>` -- a heap-free wrapper (`Mutex` and
`RefCell` are plain structs, zero allocation overhead) that requires a
`CriticalSection` token before yielding access. The token can only be obtained
inside `interrupt::free`, so the compiler statically enforces that shared state
is never accessed outside a critical section. Each piece of state that is
touched by both foreground jobs and interrupt handlers gets its own typed
static, keeping the scope of sharing explicit and minimal.

### D3: Native Types Instead of Ones-Complement

**Decision**: Use `f64` for all navigation and guidance math. Use `i16`/`u16`
for raw hardware values (channel words, counter cells, CDU angles). No custom
`AgcWord` type.

**Rationale**: The AGC's ones-complement arithmetic was a hardware constraint,
not a navigation requirement. `f64` provides 53 bits of mantissa (more than
the AGC's 29-bit double-word precision) and eliminates the entire class of
scale-factor bookkeeping errors. Hardware I/O uses `i16`/`u16` to faithfully
represent the 15-bit register values without arithmetic interpretation.

**Trade-off**: Floating-point requires an FPU for real-time performance.
The minimum viable bare-metal target (Cortex-M7 with DP-FPU) includes hardware
double-precision support. Soft-float emulation is not acceptable for the DAP
timing budget.

### D4: Restart Protection as Explicit State Machine

**Decision**: Preserve the AGC's phase-table restart mechanism rather than
relying on Rust's safety guarantees alone.

**Rationale**: Rust prevents software bugs but not hardware faults (RAM bit
flips from radiation, power glitches). The phase-table mechanism provides
recovery from any cause of restart, including those that corrupt RAM. This is
essential for a safety-critical system.

### D5: HAL Trait for Hardware Isolation

**Decision**: Define hardware interaction through a trait, with separate
implementations for simulation and bare-metal.

**Rationale**: The flight software must be testable on a development host
without hardware. The trait boundary is the natural seam.

**Trade-off**: Trait dispatch adds a vtable indirection on every hardware access.
Mitigation: use monomorphization (generics) rather than trait objects in the
hot path. The compiler eliminates the indirection entirely.

### D6: No Dynamic Memory Allocation

**Decision**: `#![no_std]` with no `alloc` crate. All data structures are
statically sized.

**Rationale**: The original AGC had no heap, and for good reason: heap
fragmentation in a long-running real-time system is a reliability hazard.
Fixed-size structures have deterministic access times and cannot fail to
allocate.

**Trade-off**: Data structure sizes must be determined at compile time. This
is not a problem because the original AGC had the same constraint, and all
table sizes are known.

### D7: Rust Embedded Ecosystem as Target Platform

**Decision**: Target the `rust-embedded` ecosystem (`cortex-m`, `cortex-m-rt`,
`embedded-hal`) with a minimum of Cortex-M7 with double-precision FPU.

**Rationale**: The `rust-embedded` crates provide the standard, well-maintained
foundation for bare-metal Rust. `embedded-hal` traits align directly with the
`agc-hal` abstraction layer. A Cortex-M7 with DP-FPU guarantees hardware
double-precision floating point, which is required to meet the DAP's 100ms
timing budget with `f64` arithmetic. Cortex-M4F only has single-precision
hardware and would require software emulation for `f64`.

**Trade-off**: The minimum target (Cortex-M7) is orders of magnitude more
capable than the original AGC. This is intentional -- the extra headroom allows
use of `f64`, eliminates the need for fixed-point arithmetic gymnastics, and
leaves room for future additions (e.g., a software simulation of the spacecraft
dynamics for testing).

### D8: No Async -- Tasks and Jobs Are the Only Concurrency Primitives

**Decision**: `async`/`await` and any async executor are prohibited in `agc-core`.
All deferred and concurrent execution must use the waitlist (tasks) or the
executive (jobs).

**Rationale**: The AGC's cooperative scheduling model is load-bearing for
restart safety and timing guarantees. An async executor would introduce a
second, hidden scheduler with its own ready queue, wakeup mechanism, and
stack discipline -- undermining the determinism that the phase-table restart
mechanism depends on. Tasks and jobs have explicit priorities, bounded
execution times, and well-defined preemption points; futures do not.

**How to apply**: If a computation needs to be deferred, split it into a
task (if it is short and time-triggered) or a job (if it is longer and
priority-driven). If it needs to wait for crew input, use the
verb/noun flashing protocol and resume via a job that is unblocked by
the DSKY interrupt handler. There is no third option.


## 16. Review Notes

This section records findings from cross-referencing the architecture against
Frank O'Brien's "The Apollo Guidance Computer: Architecture and Operation" and
the AGC Symbolic Listing Information document.

### 16.1 Items Verified as Correct

- Restart group count (6 groups) and phase encoding (odd=task, even=job,
  negative=from-top) match O'Brien Chapter 4.
- Waitlist task limit (7 table entries + 1 dispatching = 8) matches the
  LST1/LST2 table structure documented by O'Brien.
- Executive job slot count (7 core sets) matches O'Brien.
- SERVICER 2-second cycle and PIPA read/compensate/integrate sequence matches
  O'Brien's description of Average-G.
- Program numbers P00-P67 match the Comanche055 (CM) program list.
- T5RUPT 100ms DAP cycle matches O'Brien.
- Night watchman ~1.28 second timeout matches O'Brien.
- RCS jet configuration (16 SM jets in 4 quads, 12 CM jets in 2 rings) matches.

### 16.2 Corrections Applied in This Revision

1. **Cortex-M4F f64 claim (CRITICAL)**: The previous version stated Cortex-M4F
   as the minimum target, claiming it "guarantees a hardware FPU for f64
   operations." This was incorrect. Cortex-M4F has a single-precision (f32)
   FPU only. f64 on M4F requires software emulation at approximately 10x
   penalty. Corrected to require Cortex-M7 with double-precision FPU as the
   minimum target.

2. **T4RUPT timing**: The previous version described T4RUPT as "every 7.5ms
   offset" in the governing constraints table. T4RUPT actually fires on a
   120ms cycle. Corrected throughout.

3. **CDU angle inconsistency**: The type table said "full revolution = 2^15"
   (matching the original AGC's 15-bit ones-complement representation) but the
   code used `TAU / 65536.0` (2^16). Since we use u16 (twos-complement), 2^16
   is correct. Added explanatory comment clarifying the difference from the
   original AGC representation.

4. **Merge conflict**: Resolved the merge conflict in section 14.3 by
   incorporating content from both branches (the Mutex<RefCell<T>> shared state
   pattern AND the no-async prohibition).

5. **FINDVAC vs NOVAC**: Added explanation of the distinction and the 1210
   alarm for VAC area exhaustion.

6. **TVC filter**: Added description of the TVC digital filter (lead-lag
   compensator) and trim tracking, which are critical to the SPS burn control
   loop.

7. **Entry program details**: Expanded P61-P67 from a single line to
   individual entries showing each program's role in the entry sequence.

8. **DSKY verb table**: Added V05, V21-V24, V35, V36, V82 which were missing
   from the original list. These are commonly used verbs documented by O'Brien.

9. **Extended verbs**: Added section 10.4 describing the extended verb
   mechanism (V40+).

10. **Gravity model**: Added section 9.4 documenting the gravity model scope
    (Earth J2 oblateness, Moon point-mass, third-body perturbation).

11. **FRESH START vs RESTART**: Added section 6.4 distinguishing the two
    recovery modes.

12. **T4RUPT task list**: Added section 13.4 describing the rotating task
    list within T4RUPT, including gyro drift compensation (NBDX/NBDY/NBDZ).

13. **SM RCS vs CM RCS**: Added clarification that SM RCS is used during most
    of the mission and CM RCS only during entry after SM separation.

### 16.3 Items Not Verifiable Without PDF Reader

The O'Brien PDF could not be read directly due to missing `poppler-utils` on
this system. The review was conducted based on established knowledge of the
AGC architecture from O'Brien's book, the AGC Symbolic Listing document, and
primary Apollo documentation. A follow-up review with direct PDF access is
recommended to verify:

- Exact T4RUPT task list ordering and number of phases per cycle
- TVC filter coefficient values and filter order
- Complete extended verb table for Comanche055
- Exact alarm code list (there may be additional codes not listed here)
- P37 (Return to Earth) detailed algorithm and its interaction with targeting
