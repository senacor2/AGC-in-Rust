# Architecture Decision Records (ADRs)

Full rationale and trade-offs for each decision are in `docs/architecture.md §15`.
This file is the index and status tracker.

---

## ADR-001: Interpreter Elimination

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Implement all navigation and guidance computations as direct Rust functions using `f64`. Do not re-implement the AGC interpretive language VM.

**Rationale**: The interpreter was a space-saving measure on a 15-bit, 2K-ROM machine. On a Cortex-M4F with hardware FPU, native `f64` functions are faster, smaller, type-checked, and directly testable.

**Trade-off**: Translation from interpretive code requires careful understanding of each routine's numerical intent. Mitigation: unit tests compare outputs against VirtualAGC reference runs. See `docs/testing.md`.

---

## ADR-002: Single AgcState Structure

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Collect all mutable state into one `AgcState` struct, passed by `&mut` reference through foreground code.

**Rationale**: Eliminates `static mut` and `unsafe` from most of the codebase. Makes data flow explicit. Enables unit testing without global state.

**Trade-off**: State shared with interrupt handlers cannot be inside `AgcState` (interrupt handlers are called by hardware and cannot receive function arguments). Such state lives in dedicated `static Mutex<RefCell<T>>` variables. `cortex_m::interrupt::Mutex` is heap-free — it is a plain struct wrapper with zero allocation overhead.

---

## ADR-003: Native Types Instead of Ones-Complement

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: `f64` for all navigation and guidance math. `i16`/`u16` for raw hardware values (channel words, CDU angles, PIPA counts). No custom `AgcWord` type.

**Rationale**: The AGC's ones-complement arithmetic was a hardware constraint, not a navigation requirement. `f64` provides 53 bits of mantissa and eliminates scale-factor bookkeeping errors.

**Trade-off**: Floating-point requires an FPU. All viable Cortex-M targets (M4F and above) include a hardware FPU. Soft-float is not acceptable for the DAP timing budget.

---

## ADR-004: Restart Protection as Explicit State Machine

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Preserve the AGC's phase-table restart mechanism rather than relying on Rust's safety guarantees alone.

**Rationale**: Rust prevents software bugs but not hardware faults (RAM bit flips, power glitches). The phase-table mechanism provides recovery from any cause of restart.

---

## ADR-005: HAL Trait for Hardware Isolation

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Define hardware interaction through the `AgcHardware` trait with focused sub-traits per subsystem. Separate bare-metal and simulation implementations.

**Rationale**: The flight software must be testable on a development host without hardware. The trait boundary is the natural seam.

**Trade-off**: Use monomorphization (generics) not trait objects in the hot path to eliminate vtable overhead.

---

## ADR-006: No Dynamic Memory Allocation

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: `#![no_std]` with no `alloc` crate. All data structures statically sized.

**Rationale**: The original AGC had no heap. Heap fragmentation in a long-running real-time system is a reliability hazard. Fixed-size structures have deterministic access times.

---

## ADR-007: Rust Embedded Ecosystem as Target Platform

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Target the `rust-embedded` ecosystem (`cortex-m`, `cortex-m-rt`, `embedded-hal`) with a minimum of Cortex-M4F.

**Rationale**: `embedded-hal` traits align directly with the `AgcHardware` abstraction layer. Cortex-M4F guarantees a hardware FPU required for the DAP's 100ms timing budget with `f64`.

---

## ADR-008: Mutex\<RefCell\<T\>\> for Interrupt-Shared State (not raw static mut)

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: All mutable state shared between interrupt handlers and foreground code uses `cortex_m::interrupt::Mutex<RefCell<T>>`. Raw `static mut` is forbidden.

**Rationale**: `cortex_m::interrupt::Mutex` is heap-free (a plain struct wrapper around `UnsafeCell`). Its `borrow(cs)` method requires a `CriticalSection` token that can only be obtained inside `interrupt::free`. The compiler therefore statically guarantees that shared state is never accessed outside a critical section — no runtime overhead. See `docs/optimization.md §1`.

---

## ADR-009: Profile-Specific Panic Handler

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Dev builds log `PanicInfo` via semihosting then restart. Release builds restart immediately. No `panic-halt` dependency.

**Rationale**: Silent panics are undebuggable. Only one `#[panic_handler]` is permitted per binary; `panic-halt` would conflict. See `docs/optimization.md §2`.

---

## ADR-010: PAC-Sourced \#\[interrupt\] Attribute

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: The `#[interrupt]` attribute is re-exported from the device PAC crate (e.g., `stm32f4`), not used directly from `cortex-m-rt`.

**Rationale**: Using the PAC's re-export causes the compiler to verify that the interrupt name actually exists on the target device. A typo in an interrupt name with `cortex-m-rt` directly compiles silently but produces an unregistered handler. See `docs/optimization.md §3`.

---

## ADR-013: Navigation Reference Frame — AGC Mean of 1969.5

**Date**: 2026-04-10 | **Status**: Accepted

**Decision**: The port's `Frame::EarthInertial` and `Frame::MoonInertial` use the **AGC Mean of 1969.5 equatorial frame** natively — identical to the frame embedded in `Comanche055/STAR_TABLES.agc` and the Apollo-era ephemeris tables. No precession rotation is applied to star-catalogue or ephemeris data at load time.

**Rationale**: The primary validation strategy for the port is side-by-side comparison against a simulated AGC (e.g. VirtualAGC or the yaAGC emulator). Frame-matching the original hardware eliminates an entire class of "off by 0.4°" discrepancies that would otherwise appear in REFSMMAT operations, sextant mark predictions, and rendezvous targeting. The alternative — declare J2000 and precess all AGC data via IAU 1976 at compile time — introduces a rotation that must itself be independently verified and adds a silent-failure mode if the rotation matrix is wrong. Matching 1969.5 is simpler, auditable, and matches the source.

**Trade-off**: Output state vectors cannot be directly compared against contemporary J2000 ephemeris data (e.g. JPL Horizons, modern TLEs). Any such comparison must apply an explicit IAU 1976 precession to convert between frames. This is acceptable because the primary validation target is the AGC itself, not modern data.

**Affected files**:
- `specs/state-vector-spec.md` §2.2 — already declares "mean-of-1969 inertial frame"; no change needed.
- `specs/p23-spec.md` §1 and §3.2 — updated to reference ADR-013 and replace "J2000" wording.
- `specs/star-catalog-research.md` §6 — identifies the 1969.5 frame as the source-of-truth.
- `navigation/star_catalog.rs` (when populated) — will store the 37 star vectors verbatim from `STAR_TABLES.agc` with no rotation.

**Resolves**: the "navigation reference-frame discrepancy" tech-debt item in `transformation/tasks.md` (previously blocked the star catalogue population work).

---

## ADR-014: Izzo Lambert Solver Instead of AGC Conic Subroutines Port

**Date**: 2026-04-22 | **Status**: Accepted

**Decision**: Implement Lambert's problem using Izzo's (2015) λ-parametrization with Halley iteration rather than porting the universal-variable / Stumpff-function formulation from `Comanche055/CONIC_SUBROUTINES.agc`.

**Rationale**: A direct port of the AGC's Lambert solver is not viable for three reasons:

1. **Fixed-point vs floating-point mismatch.** The AGC code is meticulously tuned for 15-bit single-precision and 28-bit double-precision fixed-point arithmetic — scaling factors, shift counts, and iteration bounds are all specific to that word format. Porting to `f64` would either mean emulating fixed-point quirks (inheriting precision limitations with none of IEEE 754's advantages) or rewriting the numerical core anyway, at which point the "port" is a reimplementation.

2. **The AGC's universal-variable formulation has known numerical difficulties.** Near-parabolic transfers (Stumpff singularity at the elliptic/hyperbolic boundary), near-180° transfers (ill-defined transfer plane), and hyperbolic escapes (under-exercised in the original code, which relied on ground-computed TEI targeting) all require carefully hand-tuned iteration bounds. Those bounds are meaningless outside the original fixed-point context.

3. **Izzo's method is designed for IEEE 754 and solves exactly these problems.** The λ-parametrization unifies elliptic and hyperbolic regimes through a single parameter x with a clean (non-singular) branch at x = 1. Halley iteration provides cubic convergence with fewer iterations. Closed-form initial guesses (Eq. 30, three regimes) guarantee convergence basin entry without mission-specific knowledge. Signed λ handles retrograde transfers cleanly.

**Trade-off**: The implementation cannot be validated line-by-line against the AGC source. Mitigation: the test suite validates against analytical solutions (Hohmann, circular arc), energy conservation invariants, and both elliptic and hyperbolic regimes including retrograde (TC-LAM-1 through TC-LAM-8). This is consistent with ADR-001 (interpreter elimination) and ADR-003 (native types) — the project recreates the AGC's *functionality* in idiomatic Rust, not its bit-level implementation.

**Affected files**:
- `agc-core/src/math/lambert.rs` — Izzo 2015 implementation with Halley iteration
- `specs/lambert-spec.md` — functional specification references Izzo 2015 as the algorithm source

**Reference**: Izzo, D. (2015). *Revisiting Lambert's problem.* Celestial Mechanics and Dynamical Astronomy, 121(1), 1–15. DOI: 10.1007/s10569-014-9587-y

---

---

## ADR-011: Hardware Target — Nucleo-F722ZE (Cortex-M7 @ 216 MHz)

**Date**: 2026-05-02 | **Status**: **Superseded by ADR-021 on 2026-05-03**

> **Note**: The "Rationale" below is preserved as the historical record of
> the original 2026-05-02 decision. It contains a factual error about the
> Cortex-M7 floating-point unit on the STM32F722xx family that ADR-021
> corrects. Read ADR-021 for the current hardware target (Nucleo-F767ZI).

**Decision**: Target the STM32F722ZE on the Nucleo-F722ZE development board.
All bare-metal firmware lives in the `agc-board-nucleo-f722` crate;
the target triple is `thumbv7em-none-eabihf`.

**Rationale**:
- The architecture.md §14.1 mandate requires hardware `f64` (FPU).  The
  Cortex-M7 with double-precision FPU (STM32F7xx) meets this requirement
  directly; the Cortex-M4F (STM32F3/F4) has a single-precision FPU only
  and would require soft-float `f64`, violating the DAP timing budget.
- The STM32F722ZE provides 512 KB flash / 256 KB RAM — sufficient for
  the full `AgcState` (≈ 4 KB) plus the Executive, Waitlist, and navigation
  tables.
- The Nucleo-F722ZE carrier board exposes all required UART pins and has
  a built-in ST-LINK v3 probe, simplifying the development workflow.

**Trade-off**: Nucleo-F722ZE is not radiation-hardened.  For a real flight
computer, a radiation-tolerant part (e.g. STM32H7A3 with ECC SRAM or a
dedicated RHFL part) would be selected.  This is out of scope for this
research implementation.

**Rejected alternatives**:
- STM32F405 / F4 Discovery: Cortex-M4F, single-precision FPU only.
- STM32F3 Discovery: Cortex-M4F with FPU but only 64 KB RAM.
- Generic qemu-cortex-m: no persistent state, unsuitable for integration tests.

---

## ADR-015: External Peripheral Bridge over UART

**Date**: 2026-05-02 | **Status**: Accepted

**Decision**: DSKY, sextant/optics, SPS engine, and RCS jets are located on a
satellite "D1 mini" MCU connected to the Nucleo-F722ZE via USART6 at 460800
baud using the `agc-protocol` framing.  The AGC firmware treats them as remote
peripherals accessed through the bridge link.

**Rationale**:
- Keeps the AGC firmware focused on guidance and navigation.
- The DSKY display relay matrix (20 ms per row) and RCS jet driver circuitry
  require dedicated GPIO/PWM that the D1 mini handles without interfering with
  the AGC scheduling loop.
- Using `agc-protocol` (already implemented and tested) avoids a custom
  wire protocol and enables host-side simulation with the same code path.
- USART6 on PC6/PC7 is available on the Nucleo-F722ZE without conflicting
  with the ST-LINK UART (USART3 on PD8/PD9).

**Trade-off**: A byte equalling `0xFE` (STX) in any payload field causes the
current frame to be dropped and the decoder to resynchronise on the next STX.
Mitigation:
1. CRC-16 on every frame detects corruption and partial drops.
2. The bridge firmware implements a hardware-side 10 ms jet-quench timeout
   independent of the link, so a dropped `RcsQuenchAll` does not leave jets
   energised.
3. For display updates the loss of a single frame is visually imperceptible.
4. A future milestone may add sequence-number gap detection and
   application-level retransmission for safety-critical messages.

**Wire details**: See `docs/external-peripheral-protocol.md`.

---

## ADR-016: BMI088 Strapdown IMU with Gimballed-Platform Emulation

**Date**: 2026-05-02 | **Status**: Accepted

**Decision**: Use a Bosch BMI088 IMU (3.3 V, SPI) as the local IMU on the Nucleo-F722ZE; the bench reference module is the **Adafruit BMI088 breakout (#4836)** because its 2.54 mm headers and on-board 3.3 V regulator make breadboarding straightforward. The Bosch shuttle board is an interchangeable alternative — same silicon, same registers, same Rust driver — but requires a 1.27 mm pitch adapter.

Map the BMI088's strapdown samples to the AGC's gimballed-platform `Imu` trait by emulating a virtual stable platform inside the HAL — quaternion-integrated attitude, body-frame-to-platform-frame accelerometer rotation, PIPA pulse accumulation. The emulation lives in the new `agc-imu-platform` crate (no_std, host-tested).

**Rationale**: The Block-2 AGC's gimballed inertial platform is not commercially available. Modern strapdown IMUs are. The trait can either change (touching every flight-software call site and breaking parity with the AGC source) or the HAL can absorb the translation. The latter is correct because the AGC's gimbal abstractions encode physically-meaningful invariants (stable-member frame, CDU drive pulses, PIPA scale) that should remain visible to flight code regardless of how they are produced. The translation is small (~250 LOC of well-known quaternion math) and host-testable.

**Trade-off**: The emulator's "platform" is virtual — there is no physical isolation between the body and the inertial reference, so disturbance torques on the spacecraft body do not torque the platform via its gimbals (because there are no gimbals). In practice this only changes the failure modes, not the math: gyro drift still translates to attitude error; gimbal-lock geometry is preserved through the Euler extraction; the AGC's NBD compensation works against the strapdown gyros via `Imu::torque_gyro` exactly as it would against the original gyros.

**Alternatives considered — bridge-hosted IMU**: Routing the BMI088 through `agc-bridge-pico` instead of keeping it local on SPI3 was evaluated and rejected. The appeal was a uniform "all peripherals over the bridge" architecture matching DSKY/optics/RCS. Three issues sank it:

1. **Bandwidth.** Streaming raw samples at 1 kHz × ~16 B + 6 B framing ≈ 22 kB/s consumes ≈ 38 % of the 460 800 baud link, leaving little headroom alongside DSKY (21 rows + lamps + flash per T4RUPT), 100 Hz optics CDU, RCS, telemetry, and heartbeats.
2. **Link-jitter on integration `dt`.** Either the AGC integrates raw samples (link queue depth, STX-in-payload reframes, and CRC retransmits become attitude-integration `dt` jitter, which drifts the virtual platform) or the platform emulator moves to the bridge (relocating safety-relevant attitude state into a "stub bridge" and pushing it further from the executive's deterministic ISR cadence).
3. **No bridge FPU.** The `agc-imu-platform` quaternion math is f64. The RP2040 (Cortex-M0+) has no hardware FPU; soft-float at 1 kHz would dominate the bridge's CPU budget.

The IMU stays local. The `Imu` trait already abstracts location, so the architectural inconsistency is cosmetic — flight code does not know or care which peripherals are local vs remote. This also matches the original AGC topology, in which the IMU was a directly-wired peripheral, not on the uplink.

**Affected files**:
- `agc-imu-platform/` (new crate)
- `agc-board-nucleo-f722/src/local/imu/` (BMI088 driver + Imu trait impl)
- `agc-board-nucleo-f722/src/bin/agc.rs` (TIM7 ISR, init sequence)

**References**: BMI088 datasheet rev 1.9 (Bosch document BST-BMI088-DS001-19); `specs/imu-control-spec.md` for the gimballed-platform behaviour the emulator preserves.

---

## ADR-017: Executive runs on bare metal via foreground drain of ISR-posted atomic flags

**Date**: 2026-05-02 | **Status**: Accepted

**Decision**: The AGC's four scheduling interrupts (T3RUPT, T4RUPT, T5RUPT, T6RUPT) are
serviced by four minimal ISRs that each: (1) clear the timer's UIF flag, and (2) set a
`static AtomicBool` in `agc-core::hal::runtime`. The `Executive::run` loop drains these
flags in priority order (T6 → T5 → T3 → T4) between job dispatches. `AgcState` is only
ever mutated by foreground code.

**Motivation**: The original signature `pub fn run(&mut self, state: &mut AgcState, ...)` caused a
split-borrow conflict at the call site: `state.executive` is a field of `AgcState`, so holding
`&mut state.executive` (the `&mut self` receiver) while also passing `&mut state` (required by
every dispatched job) aliases `state`. Refactoring `run` to a free associated function
(`pub fn run<H: AgcHardware>(state: &mut AgcState, hw: &mut H) -> !`) eliminates the conflict by
taking `state` as an ordinary argument — the borrow checker sees `state.executive.*` as a brief
internal borrow within a single iteration, with no overlap with the job dispatch call.

**ISR design**: Putting `AgcState` mutations inside ISRs would require wrapping `AgcState` in a
`Mutex<RefCell<T>>`, preventing the clean `&mut` threading that is the core discipline of ADR-002.
Short ISRs that set `AtomicBool` flags avoid this: they execute in ≤ 5 µs, release the flag with
`Ordering::Release`, and return immediately. The foreground loop drains flags with
`swap(false, Acquire)`, forming a correct Release/Acquire pair.

**Priority ordering**: The drain loop checks flags lowest-discriminant-first (T6=1, T5=2, T3=3,
T4=4) to match the original AGC's interrupt priority. NVIC hardware priorities (0x10–0x40) ensure
ISRs can also preempt each other at the same relative order.

**Latency**: One Executive-loop iteration elapses between an ISR posting a flag and its foreground
action executing. At > 10 kHz idle loop rate this is ≤ 100 µs — well below the T6 demand of
0.625 ms and the T3 minimum of 10 ms.

**AtomicBool vs `extern "Rust"` weak-link**: The original plan described a weak-link pattern
(each `agc_drain_tN()` function implemented by the board crate). AtomicBool flags in
`agc-core::hal::runtime` are simpler: they compile identically on host and bare metal, require no
link-time glue, and are directly readable in unit tests without a mock board crate.

**References**: ADR-002 (single `AgcState`), ADR-008 (`Mutex<RefCell<T>>` for ISR-shared state),
ADR-010 (PAC-sourced `#[interrupt]`).

**Affected files**:
- `agc-core/src/executive/scheduler.rs` — `run` refactored to free associated function; drain loop added
- `agc-core/src/hal/runtime.rs` — new module: T3/T4/T5/T6 `AtomicBool` flags, `T3_TICK_COUNT`, `DEMO_HOOK`
- `agc-core/src/hal/mod.rs` — `pub mod runtime` added
- `agc-board-nucleo-f722/src/local/timers.rs` — full rewrite with real TIM2/3/4/5 register access
- `agc-board-nucleo-f722/src/lib.rs` — `TimerHandles`, `TIMER_HANDLES`, `with_timers` added
- `agc-board-nucleo-f722/src/bin/agc.rs` — TIM ISRs, NVIC init, demo hook, `Executive::run` entry

---

## ADR-018: Phase-5 Flight-Code Wiring Strategy

**Date**: 2026-05-02 | **Status**: Accepted

**Decision**: Wire the real flight code through the ISR-posted AtomicBool flags set up in Phase 3 (ADR-017). The `Executive::run` loop drains the flags in priority order and performs the following work per flag:

- **T6** — `hw.rcs().quench_all()` (jet pulse terminated by hardware timer).
- **T5** — No flight code uses T5 directly in this port; placeholder retained.
- **T3** — Pre-read CDU into `state.current_cdu`, call `state.waitlist.pop_task()`, invoke the popped task, re-arm T3 with the next delta.
- **T4** — Advance MET by 12 cs, compute and apply gyro drift compensation via `compute_gyro_drift` + `hw.imu().torque_gyro`.

After the ISR drain loop, each iteration also: drains the DSKY key queue into `services::v_n::feed_key`, dispatches one Executive job, translates `rcs_commanded_jets`/`rcs_commanded_pulse_cs` staging fields to `hw.rcs().fire_sm_jets` + `hw.timers().arm_t6`, and translates `engine_thrusting`/`sps_gimbal_cmd` to `hw.engine().sps_enable`/`sps_gimbal`. T3 is re-armed lazily: the last-armed value is tracked in a local `Option<u16>`; `arm_t3` register writes are skipped when the waitlist front hasn't changed.

**Why DAP runs on the waitlist (T3) rather than T5**: `dap_step` is a `fn(&mut AgcState)` Waitlist task that reschedules itself at `DAP_PERIOD_CS = 10 cs` on every invocation (ADR-017, Strategy D). T5 therefore has no flight code consumer in this port. Keeping DAP on the waitlist means T5 can be repurposed or retired in a future milestone without touching any DAP code.

**Why PIPA/SERVICER stays out of scope**: `hw.imu().read_pipa()` is a destructive read — the hardware counters are zeroed on access. The SERVICER must accumulate PIPA counts over a precise 2-second window with software integration glue (`services/average_g.rs`). The cadence, accumulator reset, and integration ordering are a separate design concern that must be coordinated with the SERVICER cycle. Putting a destructive read in the T3 path with no accumulator would corrupt navigation.

**Why DSKY display emission stays out of scope**: `services::pinball::decode_dsky` produces a `DskyFrame` (segment bitmasks), but the mapping from `DskyFrame` rows to `hw.dsky().write_row(row, data)` byte encoding is undefined: the row index convention, the GPIO bit ordering on the bridge, and the 20 ms relay hold sequencing are all unspecified. This is a separate milestone with its own design document.

**Affected files**:
- `agc-core/src/executive/scheduler.rs` — `run` rewritten; `process_rcs_staging`, `process_engine_staging` helpers added; HAL trait imports added.
- `agc-core/src/executive/waitlist.rs` — `front_delta()` and `pop_task()` added.
- `agc-core/src/hal/runtime.rs` — `T3_TICK_COUNT`, `DEMO_HOOK`, `register_demo_hook` removed.
- `agc-board-nucleo-f722/src/bin/agc.rs` — `board_demo_tick` removed; TIM2 periodic override removed; DAP bootstrap with `dap_init(AttitudeHold)` added.

---

## ADR-019: DSKY Row Encoding for the Bridge Link

**Date**: 2026-05-03 | **Status**: Accepted

**Decision**: Use a per-row, per-field encoding for `DskyWriteRow` messages rather than porting the original AGC relay matrix. The layout uses 21 rows (rows 0–20): rows 0–2 for PROG/VERB/NOUN (tens nibble in bits 7–4, units in bits 3–0), followed by sign + 5 digits for each of R1 (rows 3–8), R2 (rows 9–14), and R3 (rows 15–20). Digits are raw BCD (0x0–0x9); 0xF indicates a blank digit. Indicator lamps travel through the existing `DskySetLamp` messages; the V/N flash through `DskySetFlash`. All 21 rows are emitted on every T4RUPT (every 120 ms).

**Why not the original relay matrix**: The Block 2 AGC DSKY relay matrix packed 5 digits plus sign across 12 relay coils per row in an SC-prefix scheme (14 rows per full display cycle). Porting that scheme would require the bridge firmware to decode a compact relay-coil bitmask back into digits before rendering — adding complexity with no benefit on modern hardware. The original design was a constraint of 1960s electromechanical relay technology.

**Why 21 rows instead of 14**: The per-field design maps exactly one logical display element (one PROG/VERB/NOUN field, one register sign, or one digit) per row. This simplifies the bridge renderer: each incoming row is a direct field update. The 7-frame overhead (21 vs 14) is negligible at 460 800 baud — the 21 rows plus 10 lamp messages plus the flash message total roughly 24 frames of 6 bytes each = 144 bytes per 120 ms refresh, equivalent to ~1 KB/s, well under 1 % of link bandwidth.

**Trade-off summary**:

| Criterion              | AGC relay matrix (14 rows) | This design (21 rows) |
|------------------------|----------------------------|-----------------------|
| Wire bytes per refresh | ~84 (14 × 6)               | ~126 (21 × 6)         |
| Bridge decode effort   | Non-trivial (coil bitmask) | Trivial (nibble split) |
| Field granularity      | Multi-field per row         | One field per row     |
| Extension cost         | Relay schema must be re-engineered | Add a new row number |

**Affected files**:
- `agc-core/src/services/pinball.rs` — `emit_dsky_to_hw` function
- `agc-core/src/executive/scheduler.rs` — T4 drain calls `emit_dsky_to_hw` + `set_flash`
- `agc-bridge-pico/src/console.rs` — `decode_dsky_row` pretty-printer
- `docs/external-peripheral-protocol.md` — row encoding table

---

## ADR-020: Phase-7 Operational Polish

**Date**: 2026-05-03 | **Status**: Accepted

**Decision**: Six targeted hardening items for the Nucleo-F722ZE firmware:

1. **RCC reset-cause detection** — On every boot, `was_cold_boot()` reads RCC CSR before `dp.RCC.constrain()` consumes the peripheral. PORRSTF or BORRSTF set → cold power-on, call `fresh_start` (all state zeroed). Any other reset (IWDG timeout, software `sys_reset()`, NRST pin, low-power wakeup) → warm restart, call `restart` (navigation state preserved). RMVF written after reading to clear flags for the next boot. This implements the GOPROG behaviour described in `services/fresh_start.rs`.

2. **IWDG timeout** — Changed from prescaler /256, reload 187 (≈ 1.496 s) to prescaler /64, reload 512 (1.024 s). New value is comfortably centred in the 0.64–1.92 s AGC spec window from `specs/hal-spec.md §4.3`, giving more margin against WDT false trips from brief processing spikes without risking watchdog misses.

3. **Gravity-vector initial attitude** — The identity-attitude bootstrap assumed the board was mounted with its Z axis pointing up. Any tilt produced wrong accel bias (the subtracted `−g` was mis-projected). Replacing with `UnitQuaternion::from_two_unit_vectors(g_body_unit, [0,0,1])` derived from the 100-sample gravity mean rotates the platform so +Z aligns with the measured gravity direction, eliminating the tilt-induced bias entirely. Accel bias is then zero. The new `from_two_unit_vectors` constructor is unit-tested with identity, orthogonal, antiparallel, and arbitrary cases.

4. **DSKY refresh rate-limiting** — The T4 drain emitted all 21 rows + 10 lamps + 1 flash every 120 ms regardless of whether anything changed. A `last_dsky_frame: Option<DskyFrame>` local in `Executive::run` caches the last emission; the frame is only pushed to hardware when it differs. This reduces UART traffic by ~90 % in steady state. Trade-off: bridge consumers no longer receive a heartbeat from DSKY rows; `BridgeHeartbeat` (every 200 ms, already implemented) fills that role.

5. **T5/TIM4 retirement** — No flight code uses T5 directly. The `T5_PENDING` AtomicBool and the `arm_t5` trait method are retained at zero cost for future use, but TIM4's update interrupt is no longer enabled in `LocalTimers::init`, the TIM4 ISR is removed from `bin/agc.rs`, and TIM4's NVIC priority and unmask calls are removed. This simplifies the interrupt table and removes a redundant path that duplicates the Waitlist's T3-based dispatch.

6. **Memory layout defmt log** — A one-time `defmt::info!` at boot reports stack region (base, top, size), `.bss` size, and `.data` size using cortex-m-rt linker symbols (`_stack_start`, `_stack_end`, `__sbss`, `__ebss`, `__sdata`, `__edata`). Helps detect linker-script misconfiguration without attaching a debugger.

**Affected files**:
- `agc-board-nucleo-f722/src/bin/agc.rs` — reset-cause dispatch, gravity attitude, memory log, T5 removal
- `agc-board-nucleo-f722/src/local/watchdog.rs` — prescaler/reload corrected to 1.024 s
- `agc-board-nucleo-f722/src/local/timers.rs` — TIM4 UIE not set; ADR-020 comment
- `agc-core/src/executive/scheduler.rs` — T5_PENDING drain removed; DSKY rate-limit added
- `agc-imu-platform/src/quat.rs` — `from_two_unit_vectors` constructor added
- `agc-imu-platform/tests/quat.rs` — 4 new unit tests for `from_two_unit_vectors`

---

## ADR-021: Hardware Target Revision — Nucleo-F767ZI (Cortex-M7 with double-precision FPU)

**Date**: 2026-05-03 | **Status**: Accepted (supersedes ADR-011)

**Decision**: Switch the bare-metal target from the Nucleo-F722ZE to the
**Nucleo-F767ZI** (STM32F767ZIT6, Cortex-M7 @ 216 MHz with hardware
double-precision FPU). The board crate is renamed
`agc-board-nucleo-f722` → `agc-board-nucleo-f767`. Target triple is
unchanged: `thumbv7em-none-eabihf`.

**Why ADR-011 was wrong**: ADR-011 claimed that "the Cortex-M7 with
double-precision FPU (STM32F7xx)" describes the F722. Direct
verification against **DS11853 Rev 9 (July 2022) §2 Description, p.14**
contradicts this:

> "The Cortex®-M7 core features a single floating point unit (SFPU)
> precision which supports Arm® single-precision data-processing
> instructions and data types."

The F722/F723/F730/F732/F733 sub-family ships the
**single-precision-only** Cortex-M7 variant. The SP+DP-FPU variant
("Cortex-M7F-DP") is found on F745/F746/F756/F765/**F767**/F777/F779
and the entire H7 series. `docs/architecture.md` §14.1 (line 1182–1184)
correctly listed STM32F767 and STM32H743 as the DPFPU exemplars from
the start; ADR-011 misread "STM32F7xx" as a uniform DPFPU family.

The original DAP-timing-budget rationale therefore stands but the chip
choice did not honour it: every `f64` op on F722 would have gone through
the `compiler-builtins` soft-float path (the same one we rejected for
the Cortex-M4F), defeating the entire purpose of picking Cortex-M7.

**Why F767ZI specifically**:
- **Cortex-M7 with DP-FPU** — the architecture mandate is met without
  soft-float fallbacks. `f64` math runs at 5–20 cycles per op
  (architecture.md §14 Table 5).
- **Same MB1137 carrier board family** as F722ZE — UM1974 Table 12
  (solder bridges) and Table 21 (ST morpho pinout) both cover F722ZE
  and F767ZI in the *same row*. SB156, CN11 V_BAT pin (pin 33), and
  every BKPSRAM/battery-backup detail from the earlier work
  (project memory `project_battery_backed_bkpsram.md`) carries over
  verbatim. No board-design rework.
- **Headroom**: 2 MB flash (4× F722) and 512 KB system SRAM (2× F722).
  The full `AgcState` fits in either; F767 leaves room for future
  growth without a second migration.
- **probe-rs / stm32f7xx-hal / cortex-m-rt support** all already
  available; only the chip ID and PAC feature flag change.
- **Price delta**: ~€5–10 above F722ZE. Negligible against a months-long
  research effort.

**Rejected alternatives at the revision point**:
- **Stay on F722, accept soft-float `f64`**: contradicts the original
  DAP-budget rationale; would invalidate the headline claim that
  Cortex-M7 was chosen for hardware `f64`. Rejected.
- **Jump to NUCLEO-H743ZI** (Cortex-M7 @ 480 MHz, DPFPU, 2 MB flash,
  1 MB RAM): future-proofs more aggressively but is overkill for a
  research port. The H7 series is also a different reference manual
  (RM0433) with different power architecture and a more complex
  cache-coherency story for DTCM. Rejected as YAGNI; could be revisited
  if a later milestone (e.g. higher-rate guidance loop, full mission
  replay) needs the headroom.

**Migration scope** (this commit):
- `Cargo.toml` workspace member rename
- `agc-board-nucleo-f767/Cargo.toml`: `package.name`,
  `stm32f7xx-hal` feature `stm32f722` → `stm32f767`
- `.cargo/config.toml` (workspace + crate): probe-rs chip ID
  `STM32F722ZETx` → `STM32F767ZITx`
- `agc-board-nucleo-f767/memory.x`: FLASH 512K → 2048K, RAM 256K → 512K
  (both regions are contiguous on F767ZI: SRAM = DTCM 128K + SRAM1 368K
  + SRAM2 16K = 512K starting at `0x2000_0000`)
- All `agc_board_nucleo_f722::` Rust paths in
  `agc-board-nucleo-f767/src/bin/agc.rs` rewritten to `f767`
- Doc-comment references in `src/lib.rs` and `src/link/dispatch.rs`
- README.md, docs/hardware-bom.md, docs/external-peripheral-protocol.md,
  agc-bridge-pico/README.md
- ADR-011 marked Superseded with an in-place note (text preserved as
  historical record)
- Memory entries `project_hardware_target.md`,
  `project_battery_backed_bkpsram.md`, `project_hardware_port_paused.md`,
  `MEMORY.md` index

**Out of scope** (deliberately):
- `transformation/tasks.md` historical milestone records that say
  "agc-board-nucleo-f722 v0.x.0 — 2026-05-02" stay as written. Those
  document past work under its actual name. Active build/run
  instructions in the same file are updated.
- ADR-015, 016, 017, 018, 019, 020 are NOT rewritten — their text
  references the F722-era crate name as a matter of historical record.
  When those ADRs are revisited for unrelated reasons, the names will
  be brought current then.

**Affected files**: see commit message of the migration commit.

---

## Open / Proposed ADRs

| ID | Topic | Status |
|---|---|---|
| ADR-012 | RTIC vs hand-rolled Executive for interrupt scheduling | Proposed — see `docs/optimization.md §1` |
