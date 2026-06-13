# Glossary

A reference of abbreviations and domain terms used across this project's code,
specs, docs, and issues. Aimed at newcomers who know Rust and basic physics but
have not yet absorbed the AGC and Apollo vocabulary.

The glossary is organised in two halves:

- **AGC, computer architecture, and project-specific Rust terms** — covers the
  Block 2 AGC hardware/software, the DSKY, YUL/GAP assembler, downlink
  telemetry, the major-mode programs (P00–P67), and the Rust crates and types
  unique to this port.
- **Orbital mechanics and Apollo mission operations** — covers reference
  frames, orbital elements, trajectory planning, navigation/guidance
  algorithms, entry, time systems, and Apollo mission jargon.

Some terms appear in both halves with different emphasis (e.g. **SPS** as a HAL
sub-trait vs. as a 91 kN hypergolic engine; **REFSMMAT** as an `AgcState`
field vs. as the platform-to-inertial rotation matrix). Read whichever entry
sits in the section you came in from.

## Contents

### AGC, computer, and project
- [AGC Hardware](#agc-hardware)
- [AGC Software & Runtime](#agc-software--runtime)
- [DSKY & Crew Interface](#dsky--crew-interface)
- [Programs (Mission Phases)](#programs-mission-phases)
- [Assembler & Source Conventions](#assembler--source-conventions)
- [Telemetry & I/O](#telemetry--io)
- [Project / Rust Port](#project--rust-port)

### Orbital mechanics & mission operations
- [Reference Frames & Coordinate Systems](#reference-frames--coordinate-systems)
- [Orbital Elements & State](#orbital-elements--state)
- [Trajectory & Burn Planning](#trajectory--burn-planning)
- [Navigation & Guidance Algorithms](#navigation--guidance-algorithms)
- [Entry & Re-entry](#entry--re-entry)
- [Time & Units](#time--units)
- [Apollo Mission Jargon](#apollo-mission-jargon)

---

## AGC Hardware

**A** — Accumulator Register
: The primary 15-bit arithmetic register of the Block 2 AGC (octal address 00000). Almost every ALU instruction reads from or writes to A. In the Rust port there is no A register; the role is played by ordinary Rust local variables.

**ALTM** — Altitude Meter
: Erasable counter cell at octal address 00060, read by the altimeter hardware on the LM. Not used for CM-only (Comanche055) operation but listed in the register table.

**BBANK** — Both-Bank Register
: Composite register (octal 00006) that encodes both the EBANK (erasable bank) and FBANK (fixed bank) selector in a single word; used with the DTCB instruction to switch both banks simultaneously. In the Rust port there is no bank switching; all memory is a flat address space.

**BMAGX / BMAGY / BMAGZ** — Body-Motion Accelerometer (Rate Hand Controller inputs)
: Three counter-cell registers (octal 00042–00044) that accumulate rotation-rate pulse counts from the Rate Hand Controller (RHC). Used by the DAP to read crew stick inputs.

**BRUPT** — B-register Interrupt save
: Erasable cell (octal 00017) that saves the contents of the AGC's internal B register at interrupt time, paired with ZRUPT to save the program counter.

**CDUX / CDUY / CDUZ** — Coupling Data Unit angles (IMU gimbals)
: Three counter-cell registers (octal 00032–00034) that hold the current angular positions of the IMU outer, inner, and middle gimbals respectively, measured in 15-bit two's-complement counts where one full revolution = 2^15 counts. In the Rust port these are exposed as `[CduAngle; 3]` from `hw.imu().read_cdu()`.

**CDUXCMD / CDUYCMD / CDUZCMD** — CDU command outcounters
: Down-counting registers (octal 00050–00052) that the software loads with pulse counts to drive the CDU resolver drives for coarse-alignment slews. Correspond to `Imu::coarse_align` in the HAL.

**CDU** — Coupling Data Unit
: Angle resolver hardware that converts the analog gimbal angle into digital pulse counts and back. One CDU exists for each gimbal axis (X, Y, Z) plus two for the optics shaft and trunnion.

**CYL / CYR** — Cycle Left / Cycle Right
: Special-purpose counter-cell registers (CYR = octal 00020, CYL = octal 00022) used as hardware shift aids. Writing a value to CYR shifts it one bit right; CYL shifts one bit left. Used in bit-manipulation routines.

**EBANK** — Erasable Memory Bank register
: A 3-bit selector register (octal 00003) that chooses which 256-word erasable-memory bank is visible in the CPU's addressable window. The Block 2 AGC has eight erasable banks (E0–E7). Switched with the EBANK= pseudo-instruction in the assembler.

**EDOP** — Edit Interpretive Operation code
: A scratchpad register (octal 00023) used by the AGC interpretive-language dispatch machinery to unpack two interpreter opcodes packed into one 15-bit word.

**FBANK** — Fixed Memory Bank register
: A 5-bit selector register (octal 00004) that chooses which 1024-word fixed-memory bank is mapped into the banked region of the address space. The AGC has 36 fixed banks (B0–B35), giving 36,864 words of rope-core ROM.

**GYROCTR** — Gyro Counter register
: Outcounter register (octal 00047) used to pulse the IMU gyros during torque commands. Corresponds to `Imu::torque_gyro` in the HAL. Also called GYROCMD in the Comanche055 source.

**IMU** — Inertial Measurement Unit
: The stable platform assembly mounted in the CM equipment bay. It contains three gimbals (outer / inner / middle, corresponding to roll / pitch / yaw) that mechanically isolate a "stable member" from vehicle rotation. Three PIPAs and three gyroscopes ride on the stable member. Implemented in the Rust port via the `Imu` HAL sub-trait (`agc-core/src/hal/imu.rs`) backed by a BMI088 IMU chip + `agc-imu-platform` software platform emulator.

**INLINK** — Uplink Input register
: Erasable cell (octal 00045) that receives serial uplink bits from ground via MSFN telemetry. The UPRUPT interrupt fires when a complete word is ready. Corresponds to `Uplink::read_word` in the HAL.

**ISS** — Inertial Subsystem (= IMU assembly)
: The complete inertial navigation hardware package containing the IMU platform, CDU electronics, and PIPA electronics. "ISS Warning" is lamp bit 1 of output channel 11.

**L** — Low-order product register
: Companion register to A (octal 00001) that holds the low-order 15 bits after a multiply instruction (MP) or double-length arithmetic. In the Rust port there is no L register; f64 carries full precision.

**LEMONM / OUTLINK** — Unused registers
: Erasable cells at octal 00056–00057 listed in the register file but reserved / unused in Comanche055.

**MCT** — Memory Cycle Time
: The basic clock period of the Block 2 AGC hardware: 11.72 microseconds (one AGC "machine cycle"). Nearly all instructions execute in 1–3 MCTs. The reference throughput is approximately 85,000 simple instructions per second.

**MPAC** — Multi-Purpose Accumulator
: The main accumulator of the AGC interpretive language. Can hold a single-precision scalar, double-precision scalar, triple-precision scalar, or a three-component double-precision vector (six words). Eliminated in the Rust port; each function uses typed Rust local variables.

**NEWJOB** — Night-watchman counter cell
: Erasable cell at octal 00061. Every read of NEWJOB resets a hardware flip-flop that, if not reset within 0.64–1.92 s, triggers a hardware restart (GOJAM). The Executive main loop samples NEWJOB on every iteration. In the Rust port this maps to `AgcHardware::pet_watchdog()`.

**OUTLINK** — Unused output register
: See LEMONM above.

**PIPAX / PIPAY / PIPAZ** — PIPA axis counter cells
: Three counter-cell registers (octal 00037–00041) that accumulate delta-V pulse counts from the three accelerometers. Destructive read: reading the cell resets its counter. Accessed via `Imu::read_pipa()` in the HAL; staged into `AgcState::pipa_counts` by the Executive foreground loop.

**PIPA** — Pulse-Integrating Pendulous Accelerometer
: A precision accelerometer on each axis of the IMU stable member that generates one pulse for each ~0.0585 m/s of velocity change (the exact scale factor varies slightly by axis). Pulses accumulate in the PIPAX/Y/Z counter cells. The SERVICER reads them every 2 seconds to propagate the state vector.

**Q** — Subroutine Return Register
: CPU register (octal 00002) that holds the return address for TC (Transfer Control) instructions. In the Rust port, function call return addresses are on the Rust call stack.

**RNRAD** — Rendezvous and Landing Radar data register
: Erasable cell (octal 00046). LM only; not used in the CM Comanche055 configuration. Included in the register table for completeness.

**SAMPTIME** — Sampled time registers
: Two erasable cells (octal 00013–00014) that record a snapshot of TIME1 and TIME2 at interrupt entry time for precise time-stamping of events inside interrupt handlers.

**SR** — Shift Right register
: A special-purpose counter-cell (octal 00021) that shifts its contents one bit to the right when read, with sign extension.

**TIME1 / TIME2** — System clock registers
: A 28-bit double-precision up-counter formed by TIME1 (octal 00025, low word) and TIME2 (octal 00024, high word), incremented 100 times per second (every 10 ms). Provides mission elapsed time with ~10 ms resolution. In the Rust port this is `AgcState::time` of type `Met` (centiseconds).

**TIME3** — Waitlist timer
: An up-counter (octal 00026) loaded to `2^14 – delay_cs` that overflows every `delay_cs` centiseconds and fires T3RUPT, dispatching the next Waitlist task. In the Rust port, `Timers::arm_t3(centiseconds)` loads the equivalent value into STM32 TIM2.

**TIME4** — Periodic I/O timer
: An up-counter (octal 00027) whose periodic overflow fires T4RUPT approximately every 120 ms. Used for DSKY update, gyro drift compensation, and IMU status monitoring. In the Rust port, this is STM32 TIM3.

**TIME5** — Digital Autopilot timer
: An up-counter (octal 00030) whose overflow fires T5RUPT approximately every 100 ms. Drives the Coast DAP attitude-control cycle. In the Rust port, this is STM32 TIM4.

**TIME6** — Fine-resolution RCS timer
: A down-counter (octal 00031) that decrements at 1600 Hz (0.625 ms per count). When it reaches zero, T6RUPT fires and quenches the RCS jets. Used for precise jet-on-time control. In the Rust port, this is STM32 TIM5.

**TVCPITCH / TVCYYAW** — TVC command outcounters
: Outcounter registers (octal 00053–00054) used during SPS burns to drive the pitch and yaw gimbal actuators of the engine. Correspond to `Engine::sps_gimbal(pitch, yaw)` in the HAL.

**Z** — Program Counter
: The 12-bit program counter (octal 00005), always pointing to the next instruction to execute. Loaded by TC, TCF, BZF, and BZMF. Saved as ZRUPT on interrupt entry. In the Rust port there is no Z register; the CPU's instruction pointer fills this role.

**ZERO** — Zero register
: A read-only hardware register hardwired to the value 0 (octal 00007). Reading it always returns zero; writing it has no effect.

**ZRUPT / ARUPT / LRUPT / QRUPT / BANKRUPT** — Interrupt context-save registers
: Pairs of erasable cells (octal 00010–00016) that automatically save the contents of A, L, Q, Z, and BBANK at the moment an interrupt occurs, and are restored by the RESUME instruction at interrupt exit. On Cortex-M, the NVIC hardware saves and restores the full register set automatically.

---

## AGC Software & Runtime

**ADRLOC** — Interpreter program counter
: The interpreter's current execution address within the interpretive code stream. Equivalent to Z for machine code. Eliminated in the Rust port (no interpreter VM).

**Average-G** — see SERVICER
: The informal name for the SERVICER navigation task, derived from the averaging of gravitational acceleration over the 2-second integration interval. Implemented in `agc-core/src/services/average_g.rs`.

**BANKCALL** — Bank-switching subroutine call macro
: A common calling convention in the AGC assembler that saves the current bank registers and loads a new FBANK/EBANK pair before transferring control to a routine in a different fixed-memory bank. Unnecessary in the Rust port because the Cortex-M has a flat address space.

**BBCON** — Both Banks CONstant
: A 15-bit assembler-generated constant that encodes both the FBANK (fixed bank number) and EBANK (erasable bank number) into a single word for DXCH/BBANK and DTCF use. Appears in V26 N26 (WAITLIST parameters) and restart tables. In the Rust port, function pointers replace BBCON jump targets.

**CADR** — Complete Address
: An assembler pseudo-instruction that generates a 15-bit absolute fixed-memory address. Used in data tables that hold code addresses. In the Rust port, function pointers (`fn(&mut AgcState)`) replace CADR entries.

**CHANG1** — Change priority (Executive)
: An AGC routine that changes the priority of the currently executing job; the Executive then re-scans to determine whether to preempt. In the Rust port, equivalent to calling `create_job` to re-register with a different priority.

**CORESET** — Core Set table (= Executive job table)
: The fixed 7-entry table in erasable memory where the Executive stores priority and entry address for each active job. Implemented as `Executive::jobs: [JobEntry; MAX_JOBS]` in `agc-core/src/executive/scheduler.rs`.

**CURTAINS** — Unrecoverable-error routine
: An AGC routine called when the software detects an error so severe that continued execution is impossible. Triggers a GOJAM (hardware restart). In the Rust port, unrecoverable errors are expressed as Rust panics, which invoke the `#[panic_handler]` that calls `cortex_m::peripheral::SCB::sys_reset()` (= GOJAM).

**DANZIG** — Interpreter dispatch routine
: The interpreter's central opcode-dispatch subroutine, which decodes the next interpretive instruction pair from the program stream and dispatches it. Named for the city of Danzig (now Gdańsk) — the interpreter operated in reverse-Polish notation; dispatching each step was "the Gateway to Poland." Eliminated in the Rust port.

**DAPBOOLS** — Digital Autopilot Boolean flag word
: A flagword in erasable memory that packages multiple DAP configuration switches (attitude-hold mode, TVC mode, jet-select configuration) into a single 15-bit word. In the Rust port this is `DapState::mode` and `RcsConfig` fields in `AgcState`.

**DP** — Double Precision
: Two consecutive 15-bit AGC words read together as a 30-bit integer, providing ~9 significant decimal digits for navigation math. Most navigation quantities in Comanche055 are stored in DP. In the Rust port, `f64` (53-bit mantissa) replaces DP arithmetic throughout.

**ECADR** — Erasable Complete ADdRess
: A 10-bit absolute address into erasable memory, directly usable by the CPU without bank switching. Range: octal 00000–01777 (1024 words). Appears in V07 N08 (alarm data display). In the Rust port, no ECADR is needed; ordinary Rust references are used.

**ENDOFJOB** — End of job (Executive)
: An AGC macro that clears the current job's entry in the CORESET table, signaling the Executive that the job is complete. In the Rust port, a job is complete simply by returning from its `fn(&mut AgcState)` — the Executive's `run` loop clears the slot after the function returns.

**ERASE** — Assembler variable declaration
: A YUL/GAP pseudo-instruction that reserves one or more words in erasable memory for a variable, analogous to declaring an uninitialized variable. In the Rust port, variables are Rust struct fields in `AgcState`.

**EXEC** — Executive scheduler
: The AGC's cooperative priority scheduler that scans the CORESET table for the highest-priority ready job and dispatches it. Implemented in `agc-core/src/executive/scheduler.rs`. The main loop is `Executive::run`.

**FINDVAC** — Find a VAC area (Executive job creation with scratchpad)
: An AGC call that creates a new job AND allocates a block of scratchpad memory (a VAC area) for the interpretive language. In the Rust port, the interpreter is eliminated so FINDVAC collapses to `Executive::create_job`; no VAC pool exists.

**FLAGWRD** — Flag word
: One of twelve 15-bit bit-field words in erasable memory (FLAGWRD0–FLAGWRD11) used as a compact boolean register file. Individual bits are tested, set, and cleared by the interpreter's BON/BOFF/SET/CLEAR instructions. In the Rust port, stored as `AgcState::flagwords: [u16; 12]`.

**FRESH START** — Complete system reinitialisation
: The recovery mode that zeros all jobs, tasks, phase registers, and navigation state, setting the computer to a known clean state and entering P00. Invoked by the crew via V36 ENTER or after prolonged power loss. In the Rust port, implemented in `agc-core/src/services/fresh_start.rs`.

**GOJAM** — Hardware restart trigger
: Any condition that forces the AGC hardware to restart from address 4000 octal (the RESTART entry point). Triggers include: parity error in memory, night-watchman timeout, software instruction `TC GOJAM`, or power glitch. In the Rust port, `hardware_restart()` in the HAL maps to `cortex_m::peripheral::SCB::sys_reset()`.

**INHINT** — Inhibit Interrupts
: A zero-operand instruction that disables all program interrupts by setting the interrupt-inhibit flip-flop. Must be paired with a matching RELINT. In the Rust port, implemented via `cortex_m::interrupt::disable()` / `critical_section`.

**Job** — Executive-scheduled computation
: A longer computation registered in the CORESET table with a priority and an entry-point address. The Executive dispatches jobs in priority order. Jobs run to completion without yielding; "preemption" occurs between invocations if a higher-priority job appears. In the Rust port, a job is `fn(&mut AgcState)`.

**LST1 / LST2** — Waitlist task tables
: Two 7-word tables in erasable memory that store the delta-time entries of the Waitlist. LST1 holds the time differences; LST2 holds the entry-point addresses. Together they form the Waitlist data structure. In the Rust port, implemented as `Waitlist::entries: [Option<WaitlistEntry>; 8]`.

**MODREG** — Major Mode Register
: Erasable cell that records the currently active program number (0–99). Displayed on the DSKY PROG field. In the Rust port, this is `AgcState::major_mode: u8`.

**NOVAC** — Create a new job without a VAC area
: An AGC call that creates a new job in the CORESET table without allocating a VAC scratchpad (used for jobs that only do machine-code operations, not interpretive language). In the Rust port, collapses to `Executive::create_job`.

**OVFIND** — Overflow indicator
: An interpreter register that is set when an arithmetic operation overflows the double-precision range. Tested by the BOV (Branch on OVerflow) interpreter instruction. Eliminated in the Rust port; IEEE 754 overflow is handled by the `f64` type.

**PHASCHNG** — Phase Change
: An AGC routine called to update a restart-group phase register before and after each step of a multi-step computation, so that a restart can re-enter the computation at the correct step. In the Rust port, implemented as `RestartProtection::set_phase(group, phase)`.

**Phase table** — Restart protection table
: The array of phase registers (one per restart group) that records computation progress. On restart, the RESTART routine reads all phase values and re-dispatches each active group. Six groups are defined. Implemented in `agc-core/src/executive/restart.rs`.

**QPRET** — Interpreter return address register
: Holds the return address for the CALL interpreter instruction, analogous to Q for machine code. The RVQ instruction returns via QPRET. Eliminated in the Rust port.

**RELINT** — Release/Re-enable Interrupts
: A zero-operand instruction that re-enables program interrupts after an INHINT. In the Rust port, the critical-section exit re-enables interrupts.

**RESTART** — Partial system recovery from a transient fault
: Recovery mode that preserves navigation state and REFSMMAT, clears jobs and tasks, then re-dispatches active restart groups from their recorded phases. Used after GOJAM, parity fail, or watchdog timeout. Contrast with FRESH START. Implemented in `agc-core/src/services/fresh_start.rs`.

**Restart group** — Phase-protected computation unit
: One of six numbered groups (1–6) of related computations that together constitute a logical activity (e.g., a navigation program). Each group has its own phase register. The Rust port uses constants `GROUP_1` through `GROUP_6` in `executive/restart.rs`.

**RTB** — Return to Basic
: An interpreter instruction that temporarily exits the interpretive language to call a machine-code subroutine. The machine-code routine returns to the interpreter by jumping to DANZIG. Eliminated in the Rust port; subroutine calls are ordinary Rust function calls.

**SERVICER** — 2-second navigation task (also "Average-G")
: A repeating Waitlist task, established by navigation programs, that runs every 200 centiseconds (2 seconds) to: (1) read PIPA accelerometer counts, (2) apply PIPA compensation, (3) rotate from platform frame to inertial frame via REFSMMAT, (4) call the gravity model, and (5) call `average_g_step` to integrate the state vector. Implemented in `agc-core/src/services/average_g.rs`. Also see `AgcState::servicer_exit`.

**SP** — Single Precision
: A single 15-bit AGC word used for scalars where DP precision is not needed (e.g., display quantities, small angles). In the Rust port, `f32` is used where SP was used; `i16` / `u16` for hardware word fields.

**T3RUPT** — Timer 3 interrupt (Waitlist dispatch)
: The interrupt generated when TIME3 overflows. Its handler dispatches the earliest Waitlist task and reloads TIME3 for the next task. In the Rust port, STM32 TIM2 fires this interrupt.

**T4RUPT** — Timer 4 interrupt (periodic I/O)
: The interrupt generated by TIME4 overflow, nominally every 120 ms. Its handler performs the DSKY display update, gyro drift compensation, and IMU health monitoring. In the Rust port, STM32 TIM3 fires this interrupt.

**T5RUPT** — Timer 5 interrupt (Digital Autopilot)
: The interrupt generated by TIME5 overflow, nominally every 100 ms. Its handler runs the Coast DAP or Thrust DAP cycle. In the Rust port, STM32 TIM4 fires this interrupt.

**T6RUPT** — Timer 6 interrupt (RCS jet pulse timing)
: A one-shot interrupt generated when TIME6 counts down to zero. Its handler quenches the active RCS jets. Provides 0.625 ms pulse resolution. In the Rust port, STM32 TIM5 fires this interrupt.

**Task** — Waitlist-scheduled computation
: A short, time-triggered function that runs to completion and must not block. Tasks are inserted into the Waitlist with a centisecond delay and fired by T3RUPT. In the Rust port, a task is `fn(&mut AgcState)`, stored in `WaitlistEntry::task`.

**TP** — Triple Precision
: Three consecutive 15-bit AGC words used for the highest-accuracy arithmetic (45 bits, ~13 significant decimal digits). Rare; used in a few high-precision constants. Handled by `f64` in the Rust port.

**VAC** — Vector Accumulator (scratchpad block)
: A 43-word block of erasable memory allocated by FINDVAC for the interpretive language's push-down list and multi-word accumulator work areas. The AGC had five VAC areas. Alarm 1210 fired if all were in use. Eliminated in the Rust port because the interpreter is not re-implemented (ADR-001).

**Waitlist** — Time-triggered task scheduler
: The AGC's mechanism for scheduling tasks to execute at a specific future time. Entries are stored as a delta-time chain sorted by execution time. Driven by TIME3/T3RUPT. Maximum 8 concurrent pending tasks. Implemented in `agc-core/src/executive/waitlist.rs`.

**W-matrix** — Navigation covariance matrix
: A state-estimation uncertainty matrix maintained by the rendezvous navigation programs (P20/P22) and updated with each optical mark. Alarm 00421 fires on overflow. The Rust port implements a scalar Kalman update in `navigation/kalman.rs`. See also the orbital-mechanics half for the mathematical definition.

---

## DSKY & Crew Interface

**CLR** — Clear key
: DSKY keyboard key that erases digits entered in the current display field without committing them. Numbered key code 21 (octal 25).

**COMP ACTY** — Computer Activity lamp
: Indicator lamp on the DSKY (bit 2 of output channel 11) that illuminates whenever the Executive is dispatching a job. When dark, the computer is idle (P00 state or waiting for input).

**DSKY** — Display and Keyboard unit
: The crew's sole interface to the AGC. Contains an electroluminescent seven-segment display showing PROG (2 digits), VERB (2 digits), NOUN (2 digits), and three 5-digit registers (R1, R2, R3) with signs, plus 19 keys and approximately 15 indicator lamps. In the Rust port, the `Dsky` HAL sub-trait (`agc-core/src/hal/dsky.rs`) is implemented by the bridge link via `agc-bridge-pico`.

**ENTER** — Enter / Execute key
: DSKY key that submits the currently displayed verb/noun pair or a data entry to the V/N processor for execution. Key code 28 (octal 34).

**FDAI** — Flight Director Attitude Indicator
: The crew's attitude-ball instrument. The AGC drives FDAI needles via output channels that indicate attitude error (difference between commanded and actual attitude). Not a DSKY element, but referenced by DSKY nouns and DAP modes.

**GIMBAL LOCK lamp** — Gimbal-lock warning indicator
: DSKY lamp that illuminates when the middle IMU gimbal is within ~20° of a gimbal lock position (typically middle gimbal angle > 70°). Alarm 401 ("desired angles yield gimbal lock") is the associated program alarm.

**KEY REL** — Keyboard Release lamp and key
: A DSKY lamp (bit 5 of channel 11) that illuminates when the computer has seized the display and is waiting for crew input. The KEY REL key releases the display back to the background monitoring program without cancelling the foreground request.

**KEYRUPT1 / KEYRUPT2** — Keyboard interrupt vectors
: Hardware interrupts generated when the main DSKY (KEYRUPT1, channel 15) or the navigation-panel DSKY (KEYRUPT2, channel 16) delivers a keystroke. The AGC loads the 5-bit key code and queues it for the V/N processor.

**Major Mode** — see Program
: The active guidance program (P00, P11, P40, etc.), displayed as a two-digit octal number on the DSKY PROG field. Changed via V37 ENTER followed by two-digit program number.

**Noun** — DSKY data item selector
: A two-digit decimal number (01–99) typed after pressing the NOUN key on the DSKY. The noun identifies which data item(s) will be displayed or loaded by the current verb. For example, Noun 33 = "Time of Ignition (TIG)." The noun/verb dispatch table is in `agc-core/src/services/v_n.rs`.

**NO ATT lamp** — No Attitude reference lamp
: DSKY indicator that illuminates when the IMU is caged or the REFSMMAT is invalid. Prevents the crew from relying on attitude information that would be meaningless.

**OPR ERR lamp** — Operator Error lamp
: DSKY lamp (bit 7 of channel 11) that illuminates when the crew enters an illegal verb/noun combination or presses a key at the wrong phase of V/N interaction. The crew presses RSET to acknowledge.

**PINBALL** — DSKY verb/noun processing subsystem
: The informal name for the AGC software that handles all keyboard input and drives the DSKY display; taken from the original source file name `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`. In the Rust port, split between `agc-core/src/services/v_n.rs` (V/N state machine) and `agc-core/src/services/pinball.rs` (display formatter).

**PRO** — Proceed key
: DSKY key that causes the computer to proceed without waiting for additional data entry. Used to accept defaults or skip optional inputs. Key code 25 (octal 31).

**PROG** — Program display field (also: PROG alarm lamp)
: Two-digit field on the DSKY showing the currently active major mode number (00–67 for Comanche055). Also the name of the alarm indicator lamp that lights when a non-fatal program alarm is active.

**PROG alarm lamp** — see PROG
: Indicator lamp (bit 9 in `Lamps` struct) that illuminates when the computer has raised a program alarm. The crew reads the alarm code from N09 (R1/R2/R3 = first/second/last alarm code).

**R1 / R2 / R3** — Display registers
: Three five-digit signed decimal display registers on the DSKY. Each is driven by the active verb/noun combination to show navigation data (velocities, angles, times, etc.) to the crew. In the Rust port, stored as `DskyState.r1`, `.r2`, `.r3` (f32 values).

**RESTART lamp** — Restart indicator
: DSKY lamp that illuminates whenever the computer undergoes a RESTART (as opposed to a FRESH START). Stays lit until the crew acknowledges.

**RSET** — Reset key
: DSKY key that clears the OPR ERR and other acknowledging lamp conditions. Does not abort the current program.

**STBY lamp** — Standby indicator
: DSKY lamp (bit 3 of output channel 11) that illuminates when the AGC is in powered-down standby mode (P06 path).

**Uplink Activity lamp** — UPLINK ACTY indicator
: DSKY lamp (bit 3 of output channel 11) that flashes when the AGC is receiving an uplink data stream from the ground. In the Rust port, `DskyState.uplink_activity: bool`.

**V/N processor** — Verb/Noun processor
: The state machine that interprets DSKY keystrokes to form verb/noun pairs, route them to the appropriate handler, and manage the display. Implemented in `agc-core/src/services/v_n.rs`. Entry point: `dispatch_verb_noun`.

**VERB** — DSKY action selector
: A two-digit decimal number (01–99) typed after pressing the VERB key on the DSKY. The verb specifies an action (display, load, request, monitor). Verbs 40–99 are "extended verbs" that do not use a noun. All verbs are dispatched through the match in `services/v_n.rs::dispatch_verb_noun`.

**VnState** — Verb/Noun state machine state
: Rust enum tracking the current phase of V/N input (`Idle`, `VerbDigit1`, `VerbDigit2`, `NounDigit1`, `NounDigit2`, etc.). Stored in `AgcState::dsky` via `services/v_n.rs`. See also VnPhase.

---

## Programs (Mission Phases)

**P00** — CMC Idling
: The default background program. Entered at FRESH START and any time no active guidance task is needed. Turns the DAP to attitude-hold, does not run a repeating job. The Executive idles (COMP ACTY lamp dark). Implemented in `agc-core/src/programs/p00.rs`.

**P01** — Pre-launch IMU Initialization
: Cages the IMU platform to the launch-mount alignment, clears all alignment state, and displays pre-launch status on the DSKY. Entry point to the pre-launch alignment sequence. Implemented in `agc-core/src/programs/p01_p02.rs`.

**P02** — Gyrocompassing
: Runs on the launch pad to determine the azimuth of the IMU stable member by sensing Earth's rotation rate with the gyroscopes. Establishes `imu_alignment_state = CoarseAligned`. Implemented in `agc-core/src/programs/p01_p02.rs`.

**P06** — CMC Power-down
: Quiesces the SERVICER and DAP, drives the DSKY standby lamp, and holds the computer in a minimal-power state while maintaining critical navigation state for a later restart. Implemented in `agc-core/src/programs/p06.rs`.

**P11** — Earth Orbit Insertion Monitor
: A passive monitor that runs during the launch phase from lift-off through Earth orbit insertion (EOI). Displays altitude, inertial velocity, and flight-path angle. Drives the SERVICER for in-flight navigation. Implemented in `agc-core/src/programs/p11.rs`.

**P15** — TLI Monitor (Trans-Lunar Injection initiation/cutoff)
: Monitors the S-IVB third-stage burn that takes the spacecraft from Earth parking orbit to a trans-lunar trajectory. Similar in structure to P11. Implemented in `agc-core/src/programs/p15.rs`.

**P20** — Rendezvous Navigation (Universal Tracking)
: Processes crew optical marks of the LM (or other target) through the sextant, updates the target state vector using a scalar Kalman filter, and maintains range and range-rate displays. Implemented in `agc-core/src/programs/p20.rs`.

**P21** — Ground Track Determination
: Determines the ground track of the CSM orbit to support lunar-landmark and rendezvous navigation. Implemented in `agc-core/src/programs/p21.rs`.

**P22** — Orbital Navigation (landmark tracking)
: Refines the CSM state vector by tracking ground landmarks through the optics. Updates `csm_state` via a W-matrix Kalman filter. Implemented in `agc-core/src/programs/p22.rs`.

**P23** — Cislunar Midcourse Navigation
: Performs star-landmark sightings during the Earth–Moon transit to update the cislunar state vector. Uses the scanning telescope and sextant. Implemented in `agc-core/src/programs/p23.rs`.

**P29** — Time-of-Longitude (Cislunar)
: Computes the time at which the CSM reaches a specified geographic longitude, used for cislunar trajectory verification. Implemented in `agc-core/src/programs/p29.rs`.

**P30** — External Delta-V Targeting
: The targeting program for ground-uplinked maneuvers. The crew enters TIG and delta-V components in the LVLH frame; P30 converts them to an inertial `Maneuver` and stores it in `pending_maneuver` for P40/P41 execution. Implemented in `agc-core/src/programs/p30.rs`.

**P31** — CSM Height-Adjustment Maneuver (HAM)
: Computes a Lambert transfer from the current orbit to adjust the CSM's pericenter (height) above the target orbit. Used for rendezvous phasing. Implemented in `agc-core/src/programs/p31.rs`.

**P32** — Coelliptic Sequence Initiation (CSI)
: Targets the maneuver that places the CSM into a circular orbit coelliptic with the LM, with a specified altitude differential. Part of the standard rendezvous sequence CSI → CDH → TPI. Implemented in `agc-core/src/programs/p32.rs`.

**P33** — Constant Delta Height (CDH)
: Targets the maneuver that circularizes the rendezvous sequence at the specified constant altitude difference from the target. Follows CSI. Implemented in `agc-core/src/programs/p33.rs`.

**P34** — Transfer Phase Initiation (TPI)
: Targets the maneuver that begins the final approach to the LM, using a Lambert arc to close the remaining range. Follows CDH. Implemented in `agc-core/src/programs/p34.rs`.

**P37** — Return to Earth (Abort/TEI targeting)
: Computes a Trans-Earth Injection (TEI) burn to return the CSM to an Earth-entry corridor from anywhere in the Earth–Moon system. Uses a Lambert solver with sphere-of-influence crossings. Implemented in `agc-core/src/programs/p37.rs`.

**P40** — SPS Thrusting
: Executes any maneuver stored in `pending_maneuver` using the Service Propulsion System main engine. Engages TVC DAP, ignites the engine, monitors the burn via the SERVICER exit hook, and performs cutoff when delta-V is achieved. Implemented in `agc-core/src/programs/p40_p41.rs`.

**P41** — RCS Thrusting
: Executes small maneuvers using RCS thrusters only (no SPS). Used for trim burns < ~0.5 m/s. Implemented in `agc-core/src/programs/p40_p41.rs`.

**P47** — Thrust Monitor
: Displays SERVICER-integrated delta-V and time from ignition/cutoff for any active burn without taking control of the vehicle. Implemented in `agc-core/src/programs/p47.rs`.

**P51** — IMU Orientation Determination
: Establishes a new REFSMMAT from scratch by taking two star sightings through the sextant. Uses the TRIAD algorithm to compute the rotation matrix from stable-member to inertial frame. Implemented in `agc-core/src/programs/p51_p52.rs`.

**P52** — IMU Realignment
: Refines or re-establishes the REFSMMAT after gyro drift has accumulated. Requires two star sightings; uses gyro torquing to slew the platform to the desired orientation. Implemented in `agc-core/src/programs/p51_p52.rs`.

**P61** — Entry Preparation (Pre-entry phase)
: The first entry program; performs pre-entry system checks and verifies IMU alignment before commit to atmospheric entry. Transitions to P62 when ready. Implemented in `agc-core/src/programs/p61_p67.rs`.

**P62** — CM/SM Separation and Pre-entry Maneuver
: Commands separation of the Command Module from the Service Module and performs the small attitude maneuver to place the CM heat shield forward for entry. Implemented in `agc-core/src/programs/p61_p67.rs`.

**P63** — Entry Initialization (0.05g detection)
: Monitors sensed acceleration and transitions to closed-loop entry guidance when 0.05g is exceeded. This threshold marks the start of significant aerodynamic forces. Implemented in `agc-core/src/programs/p61_p67.rs`.

**P64** — Post-0.05g Entry (Up-control phase)
: The primary closed-loop entry guidance program. Commands bank angle to modulate lift and steer the CM to the desired landing point. Active from 0.05g through drogue deployment. Implemented in `agc-core/src/programs/p61_p67.rs`.

**P65** — Entry Up-control (Upcontrol phase)
: Continues the bank-angle guidance law for the "upcontrol" portion of the trajectory, transitioning the CM from the initial high-energy phase to the final glide. Implemented in `agc-core/src/programs/p61_p67.rs`.

**P66** — Entry Ballistic Phase
: Ballistic (unguided) entry phase used if the drag level is too high for normal guidance. The CM rolls to zero bank angle and falls unguided. Implemented in `agc-core/src/programs/p61_p67.rs`.

**P67** — Entry Final Phase (Drogue deploy)
: The terminal entry phase. Monitors descent rate and velocity; commands drogue parachute deployment at the appropriate altitude/velocity. Sets `drogue_deploy_pending` in `AgcState`. Implemented in `agc-core/src/programs/p61_p67.rs`.

**TEI** — Trans-Earth Injection
: The burn performed (typically from lunar orbit) to send the CSM back toward Earth. Computed by P37. See also LOI and the orbital-mechanics entry under [Trajectory & Burn Planning](#trajectory--burn-planning).

**TLI** — Trans-Lunar Injection
: The S-IVB third-stage burn that accelerates the Apollo stack from Earth parking orbit to a lunar transfer trajectory. Monitored by P15.

---

## Assembler & Source Conventions

**2DEC** — Double-precision decimal constant pseudo-instruction
: A YUL/GAP pseudo-instruction that encodes a floating-point constant as two consecutive 15-bit words in fixed memory. Used for navigation constants (gravitational parameters, etc.).

**BANK** — Fixed-memory bank selector pseudo-instruction
: Directs the assembler to place subsequent code into the next available word of the current (or specified) fixed-memory bank. Defines the rope-core memory layout of the Comanche055 binary.

**BNKSUM** — Bank checksum pseudo-instruction
: Inserted at the end of each fixed-memory bank to cause the assembler to print the word count used in that bank. Used for memory budget tracking.

**Comanche055** — The Command Module flight software version
: The specific revision of AGC assembly software loaded on Command Module computers for Apollo 10, 11, and related missions. The name follows the MIT/Draper project naming convention (Native American tribe names). This project ports Comanche055 to Rust.

**COUNT / COUNT\*** — Memory usage tracking pseudo-instructions
: Assembler directives that tag the following memory words with a label for post-assembly memory-usage reporting.

**DTCB** — Double Transfer Control switching Both banks
: An implied-address instruction equivalent to `DXCH BBANK` followed by `TCF` to a banked address; used to call subroutines in other fixed-memory banks while simultaneously switching both EBANK and FBANK. In the Rust port, ordinary function calls replace DTCB.

**DTCF** — Double Transfer Control switching the F-bank
: An implied-address instruction equivalent to `DXCH FB` followed by transfer; used to jump to a routine in a different fixed bank while keeping EBANK unchanged.

**EBANK=** — Erasable bank selector pseudo-instruction
: A YUL/GAP assembler directive that sets the current erasable bank number for the assembler's address resolution. Must be set before any reference to bank-specific erasable variables. Not needed in the Rust port; ordinary Rust references work across the flat address space.

**GAP** — General Assembly Program
: The successor to YUL, the assembler used to compile Comanche055. GAP is also sometimes called "MAC" (MIT Assembly Compiler). It processes `.agc` source files into binary rope-core images.

**OCT** — Octal constant pseudo-instruction
: Assembler directive that emits a 15-bit octal constant word into fixed memory. Used for lookup tables, channel masks, and initialization vectors. Octal notation is universal in AGC documentation (e.g., alarm code `01202`).

**SBANK=** — Superbank selector pseudo-instruction
: Indicates that the following code uses the "superbank" (banks 24–27 octal, which are not addressable via FBANK alone). A rarely used feature for large fixed-memory sections.

**SETLOC** — Set location counter
: An assembler pseudo-instruction that sets the assembly location counter to a specific absolute address, forcing subsequent code or data to begin there. Used for fixed entry points (interrupt vectors, restart address).

**SUBRO** — Subroutine declaration
: An assembler pseudo-instruction that records a subroutine name for the assembler's cross-reference. Has no effect on the assembled output.

**YUL** — MIT Apollo assembly language assembler (original version)
: The first AGC assembler, named after Yule tide because development began in late 1961. Replaced by GAP for later missions but the name persists informally for the entire AGC assembly toolchain.

**1DNADR through 6DNADR** — Downlink data address pseudo-instructions
: Assembler pseudo-instructions that emit a 1–6 word downlink data descriptor pointing to a labeled erasable variable. Used to build the downlink list (telemetry format table).

**2FCADR** — Double Fixed Complete ADdRess
: A pseudo-instruction that emits both the FBANK and address of a target routine as a two-word constant, used by DTCF-type calls.

---

## Telemetry & I/O

**BMAG** — Body-Motion Accelerometer (Rate Hand Controller)
: See BMAGX / BMAGY / BMAGZ under [AGC Hardware](#agc-hardware). Three counter-cell inputs from the Rate Hand Controller (RHC) that measure rotational stick deflections.

**Channel 05 (PYJETS)** — SM RCS pitch/yaw jet output channel
: Octal output channel 05; 8-bit jet-command register for four SM quad jets (A-3/4 and C-3/4 jets for +/-X and +/-pitch). Written by the jet-select logic and forwarded to hardware via `Rcs::fire_sm_jets`. In the Rust port, mapped to the low byte of `AgcState::rcs_commanded_jets`.

**Channel 06 (ROLLJETS)** — SM RCS roll jet output channel
: Octal output channel 06; 8-bit jet-command register for SM quad B and D jets (roll and Y-translation). Written alongside channel 05. In the Rust port, mapped to the high byte of `AgcState::rcs_commanded_jets`.

**Channel 11 (DSALMOUT)** — DSKY alarm/lamp output channel
: Octal output channel 11; 15-bit register that controls indicator lamps (bits 1–8), Engine On/Off discrete (bit 13), and flash state (bit 6). Implemented via `Dsky::set_lamp` and `Engine::sps_enable` HAL methods.

**Channel 12 (CHAN12)** — IMU control / TVC enable output channel
: Octal output channel 12; controls Coarse Align Enable (bit 4), Zero IMU CDUs (bit 5), Enable IMU CDU error counters (bit 6), TVC Enable (bit 8), and S-IVB interface bits. Mapped to `Imu::coarse_align` and `Engine::sps_gimbal` HAL methods.

**Channel 13 (CHAN13)** — Timer/uplink control output channel
: Octal output channel 13; contains Enable T6RUPT (bit 15), Block Inlink (bit 6), Downlink Word Order Code (bit 7), Reset Trap bits (12–14). `Timers::arm_t6` sets bit 15.

**Channel 14 (CHAN14)** — Gyro torque and CDU drive output channel
: Octal output channel 14; controls gyro enable/select/sign/activity (bits 6–10) and CDU drive pulses (bits 11–15 for X/Y/Z/T/S CDU axes). Mapped to `Imu::torque_gyro` and CDU command outcounters.

**Channel 30 (CHAN30)** — Discrete input status channel
: Octal input channel 30; 15-bit status register containing IMU Operate (bit 9), IMU Cage (bit 11), IMU CDU Fail (bit 12), IMU Fail (bit 13), SPS Ready (bit 3), and other spacecraft discretes. Read by `Imu::is_caged()`.

**Channel 33 (CHAN33)** — System status input channel
: Octal input channel 33; contains PIPA Fail (bit 13), AGC Warning (bit 14), AGC Oscillator Alarm (bit 15), Uplink/Downlink overrun flags, and CMC Control discrete.

**DOWNRUPT** — Downlink interrupt
: An interrupt generated when the downlink hardware has consumed the previous word pair and is ready for the next. The AGC's DOWNRUPT handler fetches the next two words from the downlink list and writes them to channels 34–35 (DNTM1/DNTM2). In the Rust port, `Telemetry::send_word` in the HAL.

**Downlink** — AGC telemetry to the ground
: The serial bit-stream transmitted from the AGC to Mission Control via the MSFN network. The AGC assembles downlink "words" from erasable-memory values according to a fixed downlink list. Each downlink frame is 100 words; approximately 50 frames per second. In the Rust port, handled by `agc-core/src/services/downlink.rs` and `Telemetry` HAL sub-trait.

**DNTMBUFF** — Downlink telemetry buffer
: A buffer in erasable memory that holds the 12 words of the current downlink supercommutation list entry. In the Rust port, the equivalent is `DownlinkDriver::buf` in `agc-core/src/services/downlink.rs`.

**DNPTR** — Downlink pointer pseudo-instruction
: Assembler pseudo-instruction that emits an indirect downlink descriptor — a pointer to a variable whose address is itself in erasable memory. Used for variables that move during a mission.

**INLINK** — see [INLINK under AGC Hardware](#agc-hardware)
: The uplink input register.

**MARK** — Optics sighting event
: A crew action performed by pressing the MARK button on the navigation panel when the target (star or landmark) is centered in the optics. Generates KEYRUPT2 and records the current shaft/trunnion angles for use by P20/P22/P23/P51/P52 navigation programs.

**MCU** — Mission Control Update (Uplink)
: Generic term for data sent from Mission Control to the AGC via the uplink channel. Contents include state-vector corrections, REFSMMAT updates, time corrections, and calibration constants. Implemented via V70/V71/V72/V73 verbs (P27).

**MSFN** — Manned Space Flight Network
: The global network of ground stations and tracking ships that maintained voice, telemetry, and uplink contact with Apollo. The AGC communicated with MSFN via S-band; data rates were ~51.2 kbit/s (downlink) and ~2 kbit/s (uplink). Orbital-mechanics half cross-references in [Apollo Mission Jargon](#apollo-mission-jargon).

**PAD LOAD** — Pre-flight parameter upload
: Data loaded into the AGC's erasable memory before launch (or during the mission from the ground), containing mission-specific constants: Earth-rate calibration, PIPA scale factors, star catalog epoch, TLI targeting parameters, and similar. In the Rust port, the equivalent is constants in `AgcState` initialized during FRESH START.

**RADARUPT** — Rendezvous and Landing Radar interrupt
: An interrupt generated when the LM rendezvous radar has a new data sample ready in the RNRAD register. LM only; not present in the CM Comanche055 configuration.

**RCS** — Reaction Control System
: The set of small monopropellant thrusters used for attitude control and small velocity changes. The CSM has two RCS systems: 16 jets in 4 quads (A, B, C, D) on the Service Module, and 12 jets in 2 rings on the Command Module (used after SM separation during entry). The HAL sub-trait is `agc-core/src/hal/rcs.rs`.

**REFSMMAT** — Reference to Stable Member MATrix
: A 3×3 rotation matrix (direction cosine matrix) that defines the orientation of the IMU stable member with respect to the inertial (ECI or MCI) reference frame. Updated by P51/P52 alignment and by ground uplink. Stored in `AgcState::refsmmat: Mat3x3`. The SERVICER uses REFSMMAT to rotate PIPA delta-V from platform coordinates to inertial coordinates. See also the orbital-mechanics entry in [Reference Frames](#reference-frames--coordinate-systems).

**SECS** — Sequential Events Controller System
: The pyrotechnic sequencing system that fires explosive bolts for CM/SM separation, parachute deployment, and similar one-shot events. Triggered by the AGC discrete outputs. In the Rust port, `agc-core/src/hal/secs.rs` defines the `Secs` HAL sub-trait.

**SOI** — Sphere of Influence
: The boundary in cislunar space where the AGC switches the primary gravitational body from Earth to Moon (outbound) or Moon to Earth (inbound). The AGC uses a simplified patched-conic SOI based on the ratio of gravitational accelerations. See the orbital-mechanics entry under [Navigation & Guidance Algorithms](#navigation--guidance-algorithms) for the numerical value.

**SPS** — Service Propulsion System
: The large main engine of the Service Module, used for large velocity changes: LOI, TEI, and midcourse corrections. Fixed to the airframe except for a two-axis gimbal. Commands are sent via output channels 11 (engine on/off) and 14/CDUSCMD+CDUTCMD (gimbal). In the Rust port, `Engine` HAL sub-trait in `agc-core/src/hal/engine.rs`.

**UPRUPT** — Uplink interrupt
: An interrupt generated when a complete uplink word has been received in the INLINK register. The UPRUPT handler decodes the 5-bit character and passes it to the V/N processor's uplink queue. In the Rust port, `Uplink::read_word()` in the HAL and the UPRUPT path in `services/t4rupt.rs`.

**VHF** — Very High Frequency (rendezvous ranging)
: Ranging system used by P20 to measure the distance and closing rate between the CSM and LM. The VHF range flag (`VHFRANGE`) enables the P20 Kalman filter to incorporate VHF measurements when the optics are unusable.

---

## Project / Rust Port

**ADR** — Architecture Decision Record
: A numbered note embedded in `docs/architecture.md` that records a specific design choice, its rationale, and known trade-offs (e.g., ADR-001: Interpreter Elimination, ADR-019: DSKY bridge encoding). Referenced throughout the specs as `ADR-NNN`.

**agc-board-nucleo-f767** — Nucleo board firmware crate
: The bare-metal Rust crate that provides the concrete `AgcHardware` implementation for the STM32F767ZI Nucleo-144 development board. Contains the firmware entry point, peripheral initialization, interrupt handlers, and the board-level panic handler (GOJAM). Target triple: `thumbv7em-none-eabihf`.

**agc-bridge-pico** — RP2040 bridge firmware crate
: A Rust crate running on a Raspberry Pi Pico (RP2040) that acts as a peripheral bridge between the AGC MCU and the DSKY, optics, engine, RCS, and ground-link hardware. Communicates with the AGC over USART6 at 460,800 baud using the `agc-protocol` framing. Target triple: `thumbv6m-none-eabi`.

**agc-core** — Flight software library crate
: The portable `#![no_std]` Rust library containing all AGC flight software: scheduler (executive + waitlist), navigation, guidance, control, programs, DSKY services, and hardware abstraction traits. Can be compiled for bare-metal targets or linked by host-side simulation and tests.

**agc-imu-platform** — IMU platform emulator crate
: A `#![no_std]` Rust crate that implements a software model of the gimballed stable platform. Maintains the platform orientation as a unit quaternion, simulates CDU angle readout, accumulates synthetic PIPA counts, and handles gyro torque commands. Used by the `agc-board-nucleo-f767` firmware to emulate the AGC's stable-platform IMU using a strapdown BMI088 sensor.

**agc-protocol** — Bridge wire protocol crate
: A shared `#![no_std]` Rust crate used by both `agc-board-nucleo-f767` and `agc-bridge-pico`. Defines the binary message types (`Msg` enum), the STX/LEN/SEQ/TYPE/PAYLOAD/CRC-16 frame format, and encode/decode functions. Frame start sentinel: `0xFE`.

**agc-sim** — Host-side simulator crate
: A Rust crate (std allowed) that provides a software `AgcHardware` implementation for running `agc-core` on a development host. Contains a terminal-based DSKY UI, simulated physics, and scenario loaders for integration testing.

**agc-test** — Integration test harness crate
: The Rust crate that holds system-level integration tests: `restart_recovery.rs`, `navigation_accuracy.rs`, `timing_compliance.rs`, `dsky_interaction.rs`. Tests run against `agc-sim` on the host.

**AgcHardware** — Hardware abstraction trait
: The master Rust trait that bounds all hardware access. Composed of associated type sub-traits: `Timers`, `Dsky`, `Imu`, `Optics`, `Engine`, `Rcs`, `Uplink`, `Telemetry`. Flight software never calls hardware directly; it always calls through this trait. Defined in `agc-core/src/hal/mod.rs`.

**AgcState** — Central mutable state structure
: The single `struct` that holds all runtime mutable state: scheduler (executive, waitlist, restart), navigation (state vectors, REFSMMAT, MET), guidance (burn state, pending maneuver), control (DAP, TVC, RCS config), crew interface (DSKY, alarm), and IMU (alignment state, gyro compensation, PIPA calibration). Passed by `&mut` reference through the entire call hierarchy. Defined in `agc-core/src/lib.rs`.

**BMI088** — IMU sensor chip used in the Rust hardware target
: The Bosch BMI088 is the commercial 6-axis MEMS IMU (accelerometer + gyroscope) used as the physical sensor in the `agc-board-nucleo-f767` implementation. Its strapdown measurements are fed into `agc-imu-platform` to emulate the gimballed stable platform. Connected via SPI3.

**CduAngle** — CDU gimbal angle newtype
: `pub struct CduAngle(pub u16)` in `agc-core/src/types/angle.rs`. Wraps a CDU angle count where one full revolution = 65,536 counts (2^16 rather than the AGC's 2^15 because the Rust port uses two's-complement `u16`). Provides `to_radians()`.

**DskyFrame** — Decoded DSKY display frame
: A Rust struct in `agc-core/src/services/pinball.rs` that holds a fully decoded snapshot of all DSKY fields (PROG, VERB, NOUN, R1, R2, R3, lamps, flash). Produced by `decode_dsky(&state.dsky)` and transmitted to the bridge by the T4RUPT handler.

**DskyState** — DSKY display state struct
: Rust struct that holds the current display values (`prog`, `verb`, `noun`, `r1`, `r2`, `r3`, lamp booleans, `flashing`). Stored in `AgcState::dsky`. Written by programs and service routines; read by the T4RUPT handler and encoded by `pinball::decode_dsky`.

**JobEntry** — Executive job table entry
: `pub struct JobEntry { priority: JobPriority, entry: fn(&mut AgcState), major_mode: u8 }` in `agc-core/src/executive/job.rs`. Represents one slot in the 7-slot CORESET table. Priority 0 = empty slot.

**JobPriority** — Job priority type
: `pub type JobPriority = u8` in `agc-core/src/executive/job.rs`. Value 0 is the empty-slot sentinel and is illegal as a job priority. Higher values mean higher priority; 255 is the maximum.

**Mat3x3** — 3×3 matrix type
: `pub type Mat3x3 = [[f64; 3]; 3]` in `agc-core/src/types/matrix.rs`. Used for the REFSMMAT and all coordinate-frame rotation matrices.

**Met** — Mission Elapsed Time
: `pub struct Met(pub u32)` in `agc-core/src/types/angle.rs`. Stores MET in centiseconds as a 32-bit unsigned integer (wraps after ~497 days). Converted to `f64` seconds only at math call sites.

**no_std** — Rust standard-library-free build mode
: `agc-core` is compiled with `#![cfg_attr(not(test), no_std)]`, meaning it does not use the Rust standard library on bare-metal targets. No heap (`alloc`), no threads, no OS. Enabled by the `bare-metal` Cargo feature; the `sim` feature re-enables `std` for host-side testing.

**Phase** — Restart protection phase value
: `pub struct Phase(pub i16)` in `agc-core/src/executive/restart.rs`. Encodes the restart-group state: 0 = IDLE, positive even = re-dispatch as Executive job, positive odd = re-dispatch as Waitlist task, negative = restart group from top of phase.

**ScenarioBuilder** — Test scenario API
: A fluent builder in `agc-sim` used by integration tests to seed a known AGC state, configure simulated hardware, and drive a sequence of program inputs. Used extensively in `agc-test/tests/`.

**ScheduleResult** — Waitlist schedule return type
: `pub enum ScheduleResult { OkReloadT3(u16), Ok, Full }` in `agc-core/src/executive/waitlist.rs`. Returns whether the newly inserted task is now the earliest (requiring TIME3 reload), is not the earliest (no reload needed), or the table was full (alarm 1211).

**SimHardware** — Simulated HAL implementation
: The `AgcHardware` implementation in `agc-sim/src/hardware.rs` that provides software models of all peripherals for host-side testing and simulation. No physical hardware required.

**Strategy D** — ISR-to-foreground staging pattern
: The architectural pattern (recorded as ADR-017) in which interrupt service routines (T4RUPT, T5RUPT) do not pass `&mut impl AgcHardware` to flight-software functions. Instead, ISR shims stage hardware values into `AgcState` fields before invoking tasks, and the tasks write command fields back; the ISR shim then issues the hardware I/O. This keeps `fn(&mut AgcState)` signatures on all tasks and makes them unit-testable without hardware.

**Vec3** — 3-component vector type
: `pub type Vec3 = [f64; 3]` in `agc-core/src/types/vector.rs`. Used for position (m), velocity (m/s), delta-V (m/s), and force vectors throughout the navigation, guidance, and control modules.

**VnPhase** — V/N processor input phase enum
: A Rust enum in `agc-core/src/services/v_n.rs` tracking which digit of the V/N sequence is being entered (e.g., `VerbDigit1`, `NounDigit1`, `P27Data`, etc.). The V/N state machine is `VnState`.

**WaitlistEntry** — Waitlist task entry
: `pub struct WaitlistEntry { delta_time: u16, task: fn(&mut AgcState) }` in `agc-core/src/executive/waitlist.rs`. Stores the time delta from the previous entry in the sorted delta-chain, and the task function pointer.

---

## Reference Frames & Coordinate Systems

**Body frame** — Spacecraft body axes
: The frame attached to the vehicle: X = roll, Y = pitch, Z = yaw in the standard CM convention. Used by the DAP for attitude error and by TVC for thrust-vector alignment. State vectors are never persisted in this frame; see `specs/state-vector-spec.md` §2.2.

**BRCS** — Basic Reference Coordinate System
: NASSP / Apollo flight-software term for the Earth-centered inertial frame of date used as the master inertial reference. Equivalent to `Frame::EarthInertial` in the Rust port (`agc-core/src/navigation/state_vector.rs`).

**ECEF** — Earth-Centered Earth-Fixed
: Earth-centered frame that rotates with the Earth (longitude is constant for a fixed surface point). Used for ground-track displays (P21), landmark coordinates (P22, P23), and entry-target site coordinates. Conversions live in `agc-core/src/navigation/state_vector.rs` (`inertial_to_earth_fixed`, `earth_fixed_to_inertial`).

**ECI** — Earth-Centered Inertial
: Non-rotating frame with origin at Earth's center of mass. X-axis points to the vernal equinox, Z-axis to the North Celestial Pole. Primary computational frame during Earth orbit, TLI, translunar coast (before SOI), and post-SOI transearth coast. Represented as `Frame::EarthInertial`.

**Frame** (Rust enum)
: The Rust-port enum that tags every `StateVector` with its coordinate frame: `EarthInertial`, `MoonInertial`, or `StableMember`. Defined in `agc-core/src/navigation/state_vector.rs`; controls gravity-body dispatch and SOI handover.

**GHA of Aries** — Greenwich Hour Angle of Aries
: Angle between the Greenwich meridian and the vernal equinox at a given epoch. The AGC bridges ECI and ECEF with `gha(t) = gha_epoch_rad + OMEGA_EARTH * t`. Implemented as `met_to_gha` in `agc-core/src/navigation/time.rs`. Numerically equivalent to GMST in radians once the epoch is fixed by uplink.

**GMST** — Greenwich Mean Sidereal Time
: Standard astronomical equivalent of GHA of Aries (radians since vernal equinox passed Greenwich). The Rust port uses the GHA formulation (see `specs/gmst-ecef-plan.md`); `met_to_gmst` was a stub and was replaced by `met_to_gha`.

**Hill / CW frame** — Hill / Clohessy-Wiltshire frame
: A target-centred LVLH used in rendezvous analysis. The "rendezvous LVLH" of `agc-core/src/guidance/rendezvous.rs` follows this convention: x = along-track, y = out-of-plane (−h), z = radial-inward. Distinct from the RSW targeting LVLH (`specs/rendezvous-spec.md` §4).

**Inertial frame**
: Any non-rotating frame in which Newton's laws hold without fictitious forces. In this project, "inertial" means ECI (`EarthInertial`) or MCI (`MoonInertial`); the IMU platform frame is *not* inertial in the navigational sense (it has drift) but is treated as instantaneously inertial during a SERVICER cycle.

**LVLH** — Local Vertical Local Horizontal
: Body-attached orbital frame. P30 / `apply_external_delta_v` uses the RSW convention (R = radial outward, S = in-track prograde, W = orbit normal); P20–P23 rendezvous displays use the Hill convention (x = along-track, y = −h, z = radial-inward). The two flavors are reconciled in `specs/rendezvous-spec.md` §4.

**MCI** — Moon-Centered Inertial
: Non-rotating frame with origin at the Moon's center of mass; axes parallel to ECI at the reference epoch. Active inside the lunar SOI (lunar orbit, LOI, lunar coast, TEI before SOI exit). Represented as `Frame::MoonInertial`.

**MCMF** — Moon-Centered Moon-Fixed (selenographic)
: Moon-fixed rotating frame (lunar longitude/latitude). Used by NASSP for lunar landmark coordinates; out of scope for the Comanche055 port (no lunar-surface targeting). Lunar libration model lives in `agc-core/src/navigation/lunar_libration.rs`.

**Mean of 1969.5**
: The AGC's mean-equator-and-equinox frame for the Apollo mission window. The star catalogue (`STAR_TABLES.agc`) and `navigation::star_catalog` direction vectors are expressed in this frame; treated as identical to ECI for the port because precession over the mission timespan is negligible at lunar distance (~30 km error). See ADR-013 referenced in `specs/p23-spec.md`.

**MRCS** — Moon Reference Coordinate System
: NASSP analogue of BRCS centred on the Moon; equivalent to `Frame::MoonInertial`.

**Perifocal frame (PQW)**
: Orbit-attached frame with P-axis to periapsis, Q-axis 90° ahead in the direction of motion, W-axis along angular momentum. Used as an intermediate in `elements_to_state` (`specs/conics-spec.md` §5.2) before rotating to ECI/MCI via the three-Euler-angle (Ω, i, ω) sequence.

**REFSMMAT** — Reference-to-Stable-Member Matrix
: A 3×3 orthonormal rotation matrix that maps platform-frame vectors to inertial frame: `v_inertial = REFSMMAT · v_platform`. Established by IMU alignment programs P51 (TRIAD from two star sightings) and P52 (refinement). Stored as `AgcState::refsmmat`; details in `specs/state-vector-spec.md` §2.4.

**RIC** — Radial / In-track / Cross-track
: Alternate name for the RSW LVLH frame used in some orbital-mechanics literature (Vallado §3.4). Identical to LVLH-RSW.

**RSW** — Radial / In-track / Cross-track LVLH
: The targeting LVLH used by P30 for ground-uplinked ΔV vectors. R = `unit(r)`, S = `unit(cross(W, R))`, W = `unit(cross(r, v))`. Defined in `specs/targeting-spec.md` §2.2 and implemented in `agc-core/src/guidance/targeting.rs::lvlh_to_inertial`.

**Stable-member frame**
: The frame of the IMU's gyroscopically stabilised platform. PIPA accelerometer counts are produced here. Used only transiently inside the SERVICER (rotated to inertial via REFSMMAT before being added to the state vector). The `Frame::StableMember` variant is forbidden on persistent state vectors.

**Vernal equinox**
: The direction along which the celestial equator intersects the ecliptic at the moment of the March equinox; the +X axis of ECI. Reference direction for RAAN.

---

## Orbital Elements & State

**Angular momentum vector (h)**
: `h = r × v`. Conserved in pure two-body motion. Its magnitude sets the semi-latus rectum (`p = h²/μ`); its direction is the orbit-plane normal; its Z-component / |h| gives `cos(i)`. Numerically reconstructed in `state_to_elements` (`agc-core/src/navigation/conics.rs`).

**AoP / ω** — Argument of Periapsis
: Angle in the orbital plane from the ascending node to the periapsis direction, measured in the direction of motion (radians, [0, 2π)). Undefined for circular orbits. Field `OrbitalElements::aop` in `agc-core/src/navigation/conics.rs`.

**Apoapsis** — Apogee (Earth) / Apocynthion (Moon)
: The point of maximum distance from the central body on an elliptic orbit; radius `r_a = a(1+e)`. "Apogee" is Earth-specific; "apocynthion" or "apolune" is the lunar equivalent. Functions `apoapsis_radius`, `apoapsis_altitude_earth`, `apoapsis_altitude_moon` in `conics.rs`.

**Argument of latitude (u)**
: For circular orbits where ω is undefined: angle from the ascending node to the current position in the orbital plane. Used as the substitute for true anomaly in `state_to_elements` when `e < CIRCULAR_ECC_TOL`.

**Ascending node**
: The point where the orbit crosses the equatorial plane from south to north. The unit vector to it is `cross(k̂, ĥ)/|·|`; its right ascension is Ω (RAAN).

**C3** — Characteristic energy
: `C3 = v² − 2μ/r = 2ε`. C3 > 0 means hyperbolic (escape); C3 = 0 parabolic; C3 < 0 bound. Used in escape-trajectory characterization; not stored explicitly in the port but derivable from the specific energy.

**Eccentricity (e)**
: Dimensionless shape parameter: e=0 circular, 0<e<1 elliptic, e=1 parabolic, e>1 hyperbolic. Field `OrbitalElements::e`. Tolerances `CIRCULAR_ECC_TOL = 1e-6` in `conics.rs`.

**Eccentricity vector (e_vec)**
: Vector pointing from the focus to periapsis with magnitude e. `e_vec = ((v² − μ/r)·r − (r·v)·v) / μ`. Used internally by `state_to_elements`; never stored.

**Free-return trajectory**
: A circumlunar trajectory chosen so that, with no further maneuvers, the spacecraft loops around the Moon and is automatically returned to Earth entry corridor. Apollo 8, 10, 11 launched on free-returns then transitioned to a hybrid non-free-return after TLI; Apollo 13 famously had to manoeuvre back onto a free-return after the SM oxygen tank failure.

**Hyperbolic orbit**
: Unbound trajectory with e > 1 and a < 0. Approach and departure asymptotes make angle `ν_∞ = acos(−1/e)` with periapsis. Translunar and transearth coast arcs are hyperbolic with respect to the Moon. The `OrbitalElements::is_hyperbolic()` helper checks `e ≥ 1`.

**Inclination (i)**
: Angle between the orbital plane and the equatorial plane (radians, [0, π]). `i = acos(h_z / |h|)`. i = 0 prograde equatorial, π retrograde equatorial. Field `OrbitalElements::i`.

**Keplerian elements** (classical)
: The six constants {a, e, i, Ω, ω, ν} that define a Keplerian conic in a central gravitational field. Implemented as `OrbitalElements` in `agc-core/src/navigation/conics.rs`. Computed as **osculating** (instantaneous) values from a perturbed state; not conserved between calls.

**Latus rectum / semi-latus rectum (p)**
: `p = a(1 − e²) = h²/μ`. Used by `elements_to_state` to construct perifocal-frame coordinates: `r = p / (1 + e·cos ν)`.

**Mean anomaly (M)**
: Linear-in-time anomaly satisfying Kepler's equation `M = E − e·sin E`. Not stored explicitly in `OrbitalElements`; consumed inside `kepler_step` for time propagation.

**Node vector (n_vec)**
: `n = ẑ × h`. Points toward the ascending node. Zero for equatorial orbits — the `EQUATORIAL_INC_TOL = 1e-6` test in `conics.rs` guards against the resulting singularity.

**Orbital elements** — see Keplerian elements.

**Osculating elements**
: The instantaneous Keplerian elements of the conic tangent to a perturbed trajectory at a given epoch. The Rust port computes osculating elements; mean elements (averaged over perturbation cycles) are not produced.

**Periapsis** — Perigee (Earth) / Pericynthion (Moon)
: The point of closest approach to the central body; radius `r_p = a(1−e)`. "Perigee" Earth, "pericynthion" / "perilune" Moon. The Apollo LOI burns were timed and oriented to occur **at pericynthion**, which minimizes gravity loss for the symmetric arc.

**Period (T)**
: `T = 2π·sqrt(a³/μ)` for elliptic orbits. Function `orbital_period` in `conics.rs`. Hyperbolic orbits have no period — the function panics.

**RAAN / Ω** — Right Ascension of the Ascending Node
: Angle in the equatorial plane from the X-axis (vernal equinox) to the ascending node (radians, [0, 2π)). Undefined for equatorial orbits. Field `OrbitalElements::raan`.

**Semi-major axis (a)**
: `a = −μ / (2ε)` where ε is specific orbital energy. Positive for elliptic, negative for hyperbolic, infinite (undefined) for parabolic. Field `OrbitalElements::a`.

**Specific orbital energy (ε)**
: `ε = v²/2 − μ/r`, conserved in two-body motion. ε < 0 bound, ε = 0 parabolic, ε > 0 hyperbolic. Used in `state_to_elements` to derive `a`.

**State vector**
: Position + velocity + epoch + frame tag — the fundamental navigation datum. The CSM and the rendezvous target each carry one (`AgcState::csm_state`, `AgcState::target`, AGC erasables `RN`/`VN` and `RN1`/`VN1`). See `agc-core/src/navigation/state_vector.rs` and `specs/state-vector-spec.md`.

**TEPHEM**
: AGC erasable holding the epoch of the `RN`/`VN` state vector. Maps to the `epoch: Met` field of `StateVector` in the Rust port.

**True anomaly (ν)**
: Angle from periapsis to the current position in the orbital plane, measured in the direction of motion. Field `OrbitalElements::nu`. For hyperbolic orbits restricted to `|ν| < acos(−1/e)`.

**True longitude / argument of latitude**
: Substitute angles used by `state_to_elements` for circular and/or equatorial orbits where ω and ν are undefined; see `specs/conics-spec.md` §5.1 step 9.

**Vis-viva equation**
: `v² = μ·(2/r − 1/a)`. Relates speed, radius, and semi-major axis. Used as the natural sanity check on Lambert solutions and SERVICER outputs.

**W-matrix**
: 6×6 position-velocity covariance ("information matrix") of the navigation state. The original CMC kept only a reduced 21-DP-word upper-triangular version (`WM` erasable); Mission Control did the full filter. The Rust port stores a full `[[f64; 6]; 6]` in `RendezvousNavState::w_matrix` and `CsmNavState::w_matrix` (`specs/p20-spec.md` §3).

---

## Trajectory & Burn Planning

**Burn attitude**
: 3×3 rotation matrix that aligns the spacecraft body (+X = SPS nozzle) with the required inertial thrust direction at TIG. Computed by `guidance::targeting::burn_attitude`; stored in `Maneuver::burn_attitude`.

**Burn duration**
: Approximate burn time `t_b = m·|ΔV| / F` (impulsive-equivalent approximation, ignoring mass loss). Displayed on V06N37; computed by `guidance::targeting::burn_duration`.

**CDH** — Constant Delta-Height
: Second rendezvous burn (P32). Makes the chaser's orbit coelliptic with the target at constant altitude separation Δh (typically 10 nmi ≈ 18,520 m). Closed-form; no iteration. See `specs/p31_p32-spec.md` §1.3.

**Coast**
: Unpowered flight. Propagated on demand by `propagate_coast` in `navigation::integration` or `kepler_step` in `math::kepler`. The SERVICER is normally stopped during long coasts.

**Coelliptic**
: Two orbits with the same eccentricity vector (magnitude and direction); they maintain a constant altitude difference. The geometry CSI/CDH (P31/P32) targets.

**Cross-product steering** — VXV steering
: Cross-product attitude-correction law used by the AGC during SPS burns. `ω_c = (ΔV_remaining × v_measured) / |v_measured|²`. Drives the TVC gimbal to align thrust with required ΔV direction. Implemented as `cross_product_steering` in `agc-core/src/guidance/maneuver.rs`; the gain `1/|v|²` gives roughly constant closed-loop bandwidth regardless of thrust level.

**CSI** — Coelliptic Sequence Initiation
: First rendezvous burn (P31). In-track (S-axis LVLH) burn that, after N revolutions, leaves the chaser at the correct geometry for CDH. Iterative 1-D Newton solver on ΔV magnitude. `specs/p31_p32-spec.md` §1.2.

**Deboost**
: Generic term for a retrograde burn that lowers an orbit, lowers periapsis, or transitions to a lower altitude. LOI is a deboost from a lunar hyperbolic approach into bound lunar orbit; DOI is a deboost from circular lunar orbit to a 60×8 nm descent orbit (LM only — out of scope here).

**Delta-V (ΔV)**
: Change in velocity produced by a maneuver. Vector quantity in m/s. Carried by the `DeltaV` newtype around `Vec3` (`specs/types-module-spec.md` §3.4). AGC scale B+7 m/s (`DELVEET1/2/3` erasables).

**DOI** — Descent Orbit Insertion
: LM-only burn from 60-nm circular lunar orbit to 60×8 nm phasing orbit. Out of scope for Comanche055 (CM-only port), mentioned in mission timelines.

**External ΔV (EDV)**
: A ΔV computed by Mission Control on the ground, in LVLH coordinates, and uploaded to the AGC for execution. P30's targeting mode (`TargetingMode::ExternalDeltaV`).

**Finite-burn correction**
: Adjustment of an idealized impulsive ΔV to account for the actual finite-duration burn. Apollo's LOI was symmetric about pericynthion specifically to make the finite burn approximately equivalent to an impulsive burn there.

**Gravity loss / cosine loss**
: Velocity wasted by a finite-duration burn because thrust must offset gravity throughout the burn (gravity loss) or because thrust vector is not exactly aligned with the required ΔV direction (cosine loss). Gravity loss scales roughly with (burn_duration)².

**Hohmann transfer**
: Two-impulse minimum-energy transfer between two coplanar circular orbits. 180° transfer arc; degenerate 180° geometry is the Lambert anti-parallel singularity guarded by `COLLINEAR_TOL` in `math/lambert.rs`.

**Impulsive burn**
: Mathematical idealization where the entire ΔV is applied instantaneously. Real burns are finite-duration; the approximation is used in pre-burn targeting (Lambert returns impulsive Δv₁), with corrections applied during execution by cross-product steering.

**LOI** — Lunar Orbit Insertion
: SPS retrograde burn that captures the CSM from a lunar hyperbolic approach into lunar orbit. Apollo 8 LOI-1: ΔV ≈ 914 m/s, ~4 min 7 s burn, centred on pericynthion, producing 60×170 nm orbit. LOI-2 then circularises to ~60×60 nm.

**Maneuver** (Rust struct)
: The output of every targeting program (P30/P31/P34/P37): TIG + inertial ΔV + body-frame burn attitude + targeting mode. Stored in `AgcState::pending_maneuver`. Consumed by P40 (SPS) or P41 (RCS). Defined in `agc-core/src/guidance/targeting.rs`.

**MCC** — Midcourse Correction
: Small trim burn during translunar or transearth coast to correct accumulated trajectory error. Apollo 8 flew MCC-2 (translunar) and MCC-4 (transearth); MCC-1, MCC-3, MCC-5 were not required. Typical ΔV: a few m/s. Targeted by P23 navigation marks + ground-uplinked External ΔV via P30.

**MCC-1 … MCC-7**
: Apollo numbering of planned midcourse correction opportunities. MCC-1..4 nominally between TLI and LOI; MCC-5..7 between TEI and entry. Most are not actually executed; the schedule provides time slots.

**Patched-conic approximation**
: Trajectory model that treats each leg as a Keplerian conic in the current primary body's frame, joined at the SOI boundary. The AGC and the Rust port both use this model (single primary + third-body perturbation), not full N-body integration.

**Pericynthion-centred burn**
: Burn geometry where ignition is set ~T/2 before pericynthion and cutoff ~T/2 after. The symmetric arc cancels most gravity loss; impulsive-at-pericynthion idealization is a good approximation. Standard LOI and DOI geometry.

**P40 burn**
: An SPS burn executed by program P40. The Rust port's burn-execution machinery (`BurnState`, `burn_init`, `burn_update`, `is_burn_complete`, `cross_product_steering`, `trim_residual_dv`) lives in `agc-core/src/guidance/maneuver.rs`; see `docs/p40_burn_demo.md`.

**Plane change**
: Maneuver that rotates the orbit plane (changes inclination or RAAN). W-axis component in an LVLH ΔV. Expensive at high speed (`Δv_plane = 2v·sin(Δi/2)`); usually combined with another burn.

**Prograde / Retrograde**
: Prograde = in the direction of orbital motion (h_z > 0 for an Earth orbit launched from KSC). Retrograde = opposite. The Lambert solver's `prograde: bool` parameter selects the short-way arc (prograde) or long-way arc (retrograde). See `specs/lambert-spec.md` §6.

**Residual ΔV / trim**
: ΔV remaining after SPS cutoff because the engine cannot fractionally throttle and the SERVICER cycle is 2 s coarse. Computed by `trim_residual_dv` (`guidance/maneuver.rs`); nulled by an RCS trim burn. Alarmed if > ~3 m/s.

**SPS** — Service Propulsion System
: The CSM's main hypergolic engine (~91 kN / 20,500 lbf). Drives all major burns (TLI monitoring, LOI, MCC where ΔV > 0.5 m/s, TEI). Controlled by P40 via `hw.engine().sps_enable(·)` and TVC gimbal commands.

**TEI** — Trans-Earth Injection
: Prograde SPS burn from lunar orbit that establishes the trans-earth coast trajectory. Apollo 8: ΔV ≈ 1051 m/s, ~5 min 53 s burn. Computed by P37 (`return_to_earth` Lambert); executed by P40. See `docs/tei_burn_demo.md`.

**TIG** — Time of Ignition
: Absolute MET at which the engine is commanded on. Stored in `Maneuver::tig` and `BurnState::tig`; AGC erasable `TIG` (octal 0350, B+28 centiseconds DP).

**TGO** — Time-to-Go
: Estimated seconds remaining in the current burn. Computed as `|ΔV_remaining| / a_avg` where `a_avg` is the running average thrust acceleration. Used as the backup cutoff guard; see `compute_cutoff_time` in `guidance/maneuver.rs`.

**TLI** — Trans-Lunar Injection
: S-IVB burn from Earth parking orbit that establishes the cislunar trajectory. Apollo 8: T+2:50:41 MET, ΔV ≈ 3047 m/s. Monitored (not commanded) by AGC program P15.

**TPI** — Terminal Phase Initiation
: Third rendezvous burn (P33). Lambert-solver burn to intercept the target after a nominal ~10-minute transfer; crew times TIG by the target elevation angle (nominally 27.45° / 130 mils). `specs/p33_p34-spec.md` §1.2.

**TPM / TPF** — Terminal Phase Midcourse / Terminal Phase Finalize
: Fourth rendezvous phase (P34) — small RCS corrections during the TPI transfer if the chaser drifts off the intercept arc.

**Ullage**
: A small RCS settling burn before SPS ignition to push propellant against the engine intake (so the SPS gets liquid, not gas). Not modelled separately in the port; the cross-product steering's `assert(|current_v|>0)` guards against ignition before measurable thrust.

---

## Navigation & Guidance Algorithms

**Average-G** — Average Gravity integration
: The AGC's powered-flight integration scheme. Two-stage trapezoidal Cowell: gravity is evaluated at the start (`g0`) and the predicted end (`g1`) of each 2-s interval and averaged for the velocity update. Implemented as `average_g_step` in `agc-core/src/navigation/integration.rs`. Driven by the SERVICER (`services::average_g`). Second-order accurate for near-circular orbits.

**Conic propagation**
: Propagation of a state vector along its unperturbed Keplerian conic. Used for coast-phase on-demand updates (no SERVICER cycle). Implemented in `math::kepler::kepler_step` (universal-variable formulation) and re-exported by `navigation::conics`. See `specs/conics-spec.md`.

**Cowell's method**
: Direct numerical integration of the full equations of motion (primary gravity + perturbations summed at each step). Used by the SERVICER (`average_g_step`). Simple but accumulates roundoff error during long coasts because the dominant central-body term dwarfs perturbations.

**Cross-product steering** — see [Trajectory & Burn Planning](#trajectory--burn-planning).

**Encke's method**
: Numerical integration of only the deviation from a reference Keplerian conic, with periodic rectification. Used by the original AGC in `ORBITAL_INTEGRATION.agc` for long coast arcs. The Rust port defers Encke; for current scope (lunar orbit, translunar / transearth), Cowell + `kepler_step` is sufficient (`specs/integration-spec.md` §2.2, §7).

**HUNTEST / INITROLL**
: AGC entry-guidance routines that compute the initial roll command and iterate the reference L/D (`LEWD`) toward zero downrange error. Implemented in `agc-core/src/guidance/entry.rs::compute_ld_command` (MS-E3); AGC source `REENTRY_CONTROL.agc`.

**Kepler propagation / Kepler's equation**
: Time-marched conic-arc propagation by solving Kepler's transcendental equation `M = E − e·sin E` (or the universal-variable equivalent). `math::kepler::kepler_step(r0, v0, dt, mu) -> (r, v)` is the universal-variable propagator.

**Lambert's problem**
: Two-point boundary-value problem: given r₁, r₂, transfer time, and μ, find v₁ and v₂. The conic-targeting foundation. Solved by Izzo's 2015 algorithm in `agc-core/src/math/lambert.rs`; re-exported by `agc-core/src/guidance/lambert.rs`. The AGC source is `Comanche055/CONIC_SUBROUTINES.agc` (`RVIO`/`LAMBERT`).

**Lagrange f & g coefficients**
: Scalar functions of time/anomaly that express future (r, v) as linear combinations of initial (r₀, v₀): `r = f·r₀ + g·v₀`. Used inside the universal-variable Kepler propagator and Lambert iteration.

**Patched-conic** — see [Trajectory & Burn Planning](#trajectory--burn-planning).

**PIPA** — Pulse Integrating Pendulous Accelerometer
: The three accelerometers on the IMU stable member. Each pulse ≈ 0.0585 m/s. PIPA counts are read destructively each SERVICER cycle, compensated (bias, scale, misalignment) by `services::average_g`, and rotated through REFSMMAT into the inertial frame before integration.

**Powered Flight Steering**
: General term for the closed-loop attitude-correction during a burn. In the Apollo AGC this is the cross-product steering law; the result is fed into the TVC DAP. See `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc`.

**Scalar Kalman update**
: The AGC's measurement-incorporation algorithm for P20/P22/P23 navigation marks. A single scalar measurement (range, range-rate, sextant angle) updates the full 6-state covariance and state without inverting a measurement-matrix. Shared algorithm across P20, P22, P23; described in `specs/p20-spec.md` §6 and O'Brien Chapter 11 pp. 318–325.

**SERVICER** (Average-G task)
: The 2-second Waitlist task that runs whenever active navigation is required. Reads PIPA, compensates, rotates to inertial, integrates the state vector via `average_g_step`. Hosts the `servicer_exit` hook through which P40 / entry programs piggyback on the cycle. Lives in `agc-core/src/services/average_g.rs`. AGC source `AVERAGE_G_INTEGRATOR.agc`.

**Sphere of Influence (SOI)**
: Region around a secondary body where its gravity dominates over the primary's. Used as the boundary between Earth-centric and Moon-centric patched-conic legs. Implemented as `R_SOI_MOON = 66_183_000 m` in `agc-core/src/navigation/gravity.rs` (Hill-sphere approximation, ratio (M_moon/M_earth)^(2/5) × a_moon). At the boundary the active `Frame` is switched and position/velocity are re-expressed relative to the new origin (`specs/state-vector-spec.md` §2.3).

**Stumpff functions C(ψ), S(ψ)**
: Series functions used in the universal-variable Kepler formulation to handle elliptic, parabolic, and hyperbolic cases uniformly. Used inside `math::kepler::kepler_step`.

**TRIAD method**
: Closed-form algorithm to construct a rotation matrix from two pairs of unit vectors (two stars sighted in inertial frame and in platform frame). Used by P51 to build a fresh REFSMMAT from caged. Returns `None` if the two star vectors are collinear (alarm 220). Lives in `agc-core/src/control/imu_control.rs`.

**Universal variable (χ, ψ)**
: Single-parameter formulation of Kepler propagation that works seamlessly across elliptic, parabolic, hyperbolic. Used by `math::kepler::kepler_step` and (conceptually) by the Battin Lambert solver underlying the Rust port's Izzo implementation.

---

## Entry & Re-entry

**0.05g threshold**
: Sensed-acceleration threshold (≈ 0.49 m/s²) that marks the transition from P63 (PreEntry monitor) to P64 (closed-loop entry guidance). Constant `ENTRY_THRESHOLD_G` in `agc-core/src/programs/p61_p67.rs`.

**Atmosphere model** — exponential
: `ρ(h) = ρ₀ · exp(−h/H_s)` with ρ₀ = 1.225 kg/m³ and H_s = 7160 m. Used by entry guidance for dynamic pressure and drag corrections. `agc-core/src/navigation/atmosphere.rs`.

**Ballistic phase**
: P66. Open-loop attitude hold (zero roll rate) entered when guidance diverges. `EntryPhase::Ballistic` in `agc-core/src/programs/p61_p67.rs`.

**Bank angle / roll command**
: The CM uses constant L/D modulus and **rolls** to direct the lift vector up (lengthen range) or down (shorten range). Produced by `resolve_roll` in `guidance/entry.rs`; the sign comes from the cross-range error.

**Blackout**
: Communications blackout from ionized plasma sheath at peak entry heating (roughly Mach 25 / 50–80 km altitude). Not modelled directly; observable as the absence of telemetry during this phase.

**CM/SM separation**
: Pyrotechnic separation of the Command Module from the Service Module before entry. Program P62 — the SM is jettisoned, the CM-only RCS takes over attitude control (`DapMode::AttitudeHold`).

**Direct entry / direct return**
: Entry without a skip-out — drag-dominated single pass to drogue deploy. Direct LEO entry at ~7900 m/s at the interface; lunar return at ~11000 m/s typically requires a skip.

**Drogue deploy**
: Deployment of the drogue parachute (program P67 terminal action). Trigger condition (velocity / altitude) drives `entry.drogue_deployed = true` and (future) `hw.secs().deploy_drogue()`.

**Dynamic pressure (q̄)**
: `q̄ = ½·ρ·v²`. Peak dynamic pressure (~150–200 kN/m² lunar return) sets aerodynamic loads. Recorded in the footprint sweep table (`docs/entry_footprint.md`).

**EI** — Entry Interface
: Conventionally the point at 121,920 m (400,000 ft) above the Earth surface — the entry corridor target. Used as the Lambert target by P37 (TEI). Constant `R_EARTH + 121920 m`.

**Entry corridor**
: The narrow band of flight-path angles at EI for which the CM neither skips out (too shallow) nor exceeds load/heat limits (too steep). Apollo nominal: −6.5° ± ~1°.

**EntryPhase** (Rust enum)
: `Idle → Preparation → Separation → PreEntry → Entry → Final` (and `Ballistic` for divergence). Drives P61–P67 sequencing in `agc-core/src/programs/p61_p67.rs`.

**Final phase / PREDICT3 / GLIM**
: P67. Terminal range control: track a reference R-dot vs velocity profile (`PREDICT3`) and limit g-load (`GLIM`); deploy drogue at terminal velocity/altitude. AGC source `REENTRY_CONTROL.agc`.

**Footprint**
: The set of landing locations reachable by varying initial flight-path angle and bank-angle profile at EI. Apollo CM downrange footprint on lunar return is ~1500–3500 nmi from EI. Generated by `cargo test … regenerate_footprint_table` and recorded in `docs/entry_footprint.md`.

**FPA** — Flight-Path Angle
: Angle of the velocity vector below local horizontal. Negative at entry (descending). Apollo nominal entry FPA ≈ −6.5°. Drives the entry-corridor footprint sweep.

**g-load / peak g**
: Sensed deceleration in units of standard gravity (g₀ = 9.80665 m/s²). Direct LEO entry: peak ~8–13 g; lunar return: ~8–15 g depending on FPA. Tracked in `docs/entry_footprint.md`.

**L/D — Lift-to-Drag ratio**
: Aerodynamic efficiency. CM L/D ≈ 0.3 in trim. Constant magnitude; sign/direction modulated by **roll**. `LAD_NOMINAL` and `LOD_NOMINAL` constants in `agc-core/src/guidance/entry_tables.rs`.

**LEWD / DLEWD**
: HUNTEST-iterated reference L/D (`LEWD`) and its step size (`DLEWD`). Per-cycle Newton iteration converging on zero downrange error. Persisted in `EntryState.lewd_ref`/`dlewd`.

**Lift vector**
: The aerodynamic lift force, perpendicular to the velocity vector. Magnitude is fixed by trim attitude; direction is controlled by rolling.

**MAX_ALTITUDE_M**
: 250,000 m cutoff above which the exponential atmosphere returns 0. Prevents downstream divisions by underflow values.

**Range-to-go**
: Great-circle ground distance from current sub-satellite point to target landing site. Computed by `navigate_entry` and `predict_range` in `guidance/entry.rs`; AGC label `ASP` (or `DIFF` for the error against target).

**R-dot**
: Altitude rate (`= v · sin(FPA)`). Used as the controlled variable in the final phase reference profile.

**Reference profile (entry)**
: Tabulated R-dot vs velocity profile that the closed-loop guidance tracks during the final phase. Constants in `agc-core/src/guidance/entry_tables.rs`; AGC source `REENTRY_CONTROL.agc`.

**Skip / skip-out / UPCONTRL**
: Lunar-return entry technique: aerobrake in atmosphere, lift up to exit the atmosphere again, re-enter for a second drag pass. Extends downrange. Program P65 (UPCONTRL); not yet implemented (deferred to MS-E4).

**Splashdown / target line**
: Final landing point in the ocean. The DSKY display register `target_range_km` carries the planned downrange in `EntryState`; cross-range error drives the bank-sign decision.

**VFINAL / VQUIT / VLMIN**
: Terminal-phase velocity reference thresholds in `entry_tables.rs`. VFINAL marks the velocity at which range prediction terminates; VQUIT the closed-loop guidance quit point; VLMIN the lower-bound velocity for the range-arc model.

---

## Time & Units

**B+n scaling**
: AGC double-precision fixed-point scaling convention: a stored DP fraction `f ∈ [−1, 1)` represents the physical value `f · 2^n`. Examples used in this port: position B+28 m (1 LSB ≈ 1 m), velocity B+7 m/s (1 LSB ≈ 7.6 × 10⁻⁴ m/s), TIG B+28 centiseconds DP.

**Centiseconds**
: 0.01-s units. AGC `TIME1`/`TIME2` increments at this resolution; the Rust port's `Met` newtype is a `u32` centisecond counter.

**Epoch**
: The reference instant attached to a state vector or set of orbital elements. Carried by `StateVector::epoch` (an `Met`) and `OrbitalElements::epoch`. AGC erasable `TEPHEM` (octal 0340, 3 DP words).

**Ephemeris time / TDB**
: Time scale used by planetary ephemerides (JPL DE-series, etc.). Not used directly in the AGC; the Rust port's lunar ephemeris (`agc-core/src/navigation/planetary.rs`) treats MET as approximately equivalent over the mission timespan (~30 km precession-equivalent error, well within scope).

**GET** — Ground Elapsed Time
: Time since liftoff, as displayed to the crew/MCC. Used by P21 and P29 for crew-entered target times. Numerically the same as MET in the AGC representation.

**GHA / GHA of Aries** — see [Reference Frames](#reference-frames--coordinate-systems).

**JD** — Julian Day
: Continuous day count from −4712 January 1 noon UT. `APOLLO_11_LAUNCH_JD = 2440419.0639` anchors the lunar ephemeris (`navigation::planetary`).

**Met** (Rust newtype)
: `u32` centisecond counter representing Mission Elapsed Time. Defined in `agc-core/src/types/mod.rs`. Roll-over at ~497 days — far beyond any Apollo mission.

**MET** — Mission Elapsed Time
: Seconds (centiseconds in AGC representation) since liftoff. The fundamental project time axis. Apollo 8 events keyed by MET include TLI at 02:50:41, LOI-1 at 69:08:20, TEI at 89:19:16, entry at 146:46.

**MJD** — Modified Julian Day
: JD − 2,400,000.5. NASSP scenario files use MJD for the simulation epoch (e.g. MJD 40211.36875 for Apollo 8 launch).

**OMEGA_EARTH**
: Earth sidereal rotation rate, `7.292_115_085_5e-5 rad/s`. Defined in `agc-core/src/navigation/time.rs`. Drives GHA evolution and ECI↔ECEF velocity cross-term.

**Sidereal day**
: 86,164.0905 s — one rotation of the Earth relative to inertial space. Distinguished from the 86,400-s solar day.

**UT / UTC**
: Universal Time / Coordinated Universal Time. The wall-clock time scale used for mission planning. Apollo 8 launch: 1968-12-21 12:51:00 UT. The AGC itself works in MET; conversions are done by ground.

---

## Apollo Mission Jargon

**Apollo 8**
: Target mission for MS-T4 walkthrough testing. First crewed lunar-orbit mission (December 1968). Key events: parking orbit insertion T+0:11:35 (185 km × 184 km × 32.5°); TLI T+2:50:41 (ΔV ≈ 3047 m/s); MCC-2 and MCC-4 executed; LOI-1 T+69:08:20 (ΔV ≈ 914 m/s, ~4:07 burn); LOI-2 T+73:35:06 (≈41 m/s circularization); 10 lunar revs; TEI T+89:19:16 (ΔV ≈ 1051 m/s); entry T+146:46. Source: Apollo 8 Mission Report (NASA TM X-65500).

**BURNBABY / STEERSUB**
: Names by which the powered-flight steering subroutine is commonly identified in AGC assembly. Lives in `POWERED_FLIGHT_SUBROUTINES.agc`. Implements cross-product steering.

**CMC** — Command Module Computer
: The AGC installation in the CSM. Comanche055 = the CMC mission program. Distinct from the LGC (LM Guidance Computer) running Luminary.

**CSM** — Command and Service Module
: The combined Command Module + Service Module spacecraft. Carries the SPS engine, RCS, fuel, the AGC running Comanche055, and the crew. State vector stored as `AgcState::csm_state`; AGC erasables `RN`/`VN`.

**Comanche055**
: The mission release of the CMC flight software used as the source-of-truth for this port. Originally flown on Apollo 11 (CMC); the assembler source is on GitHub (`chrislgarry/Apollo-11`).

**CSI / CDH / TPI / TPM** — see [Trajectory & Burn Planning](#trajectory--burn-planning).

**DAP** — Digital AutoPilot
: The attitude-control loop, runs on T5RUPT (~100 ms). Two flavors for CSM: Coast DAP (RCS jets, attitude hold/maneuver) and Thrust DAP (TVC during SPS burns). Implementation in `agc-core/src/control/dap.rs`. Mode driven by `DapState::mode`.

**DELVEET1 / 2 / 3**
: AGC erasable triple holding the inertial delta-V (B+7 m/s DP). For targeting (P30/P31/P34/P37) holds the target ΔV; for execution (P40) accumulates measured ΔV. Maps to `Maneuver::delta_v` / `BurnState::accumulated_dv_inertial`.

**DSKY** — see [DSKY & Crew Interface](#dsky--crew-interface)
: The crew interface to the AGC. Verbs / Nouns / Programs. The surface for all maneuver displays (N33 TIG, N45 burn summary, N81 LVLH ΔV, N44 apo/peri/TFF, N49 mark counters).

**FIDO** — Flight Dynamics Officer
: Mission Control trajectory specialist responsible for the as-flown trajectory and the maneuver pad computations uplinked to the crew via P30. Not implemented in the port; FIDO's outputs arrive as External ΔV / state-vector uplinks.

**Free return** — see [Orbital Elements & State](#orbital-elements--state) → Free-return trajectory.

**Hybrid trajectory**
: A translunar trajectory that departs Earth on a free-return but is biased to a non-free-return after MCC-2. Used by later Apollo missions (11–17) to optimise lunar-orbit plane.

**IMPULSIVE**
: AGC subroutine in `P30,P31,P37,P40SUBROUTINES.agc` that converts a ground-supplied LVLH delta-V into the inertial form P40 needs. Rust analogue: `guidance::targeting::apply_external_delta_v`.

**KEPRTN**
: The AGC's universal-variable Kepler propagator routine in `CONIC_SUBROUTINES.agc`. Rust analogue: `math::kepler::kepler_step`.

**LM** — Lunar Module
: Apollo lunar lander. The rendezvous target during the lunar phase. Its state vector occupies the AGC's `RN1`/`VN1` slot (`AgcState::target` in the Rust port). Out of scope as a controlled vehicle.

**Mark**
: A sextant or radar measurement event marked by the crew via `V54E`. The AGC processes it through R52/R53 (optics) or R22 (radar) to produce a `StarHorizonMark` / `LandmarkMark` / radar range+range-rate sample, which is then incorporated into the W-matrix.

**MSFN** — Manned Space Flight Network
: Ground tracking and communications network. Provides the state-vector uplinks that P23 navigation refines, and accepts the AGC's downlink telemetry.

**Noun**
: DSKY data designator. Each Noun selects a triplet of registers with a fixed scale and meaning. Noun 33 = TIG, 37 = TIG / ΔV / burn time, 44 = apoapsis / periapsis / TFF, 45 = burn summary, 49 = mark counters, 64 = entry status, 81 = LVLH ΔV components, 85 = ΔV components / residuals, 93 = REFSMMAT confirmation.

**P00 .. P67** — Major Modes / Programs
: AGC top-level programs. See [Programs (Mission Phases)](#programs-mission-phases) for the per-program implementation pointers. Summary: P00 idle; P11 EOI monitor; P15 TLI monitor; P20 rendezvous nav; P21 ground track; P22 landmark tracking; P23 cislunar midcourse nav; P27 update liaison (uplink); P29 time-of-longitude; P30 external ΔV; P31 CSI; P32 CDH; P33 TPI; P34 TPM; P37 return-to-Earth (TEI); P40 SPS burn; P41 RCS burn; P47 thrust monitor; P51 IMU orientation determination; P52 IMU realignment; P61 entry prep; P62 CM/SM sep; P63 pre-0.05g monitor; P64 closed-loop entry; P65 up-control (skip); P66 ballistic; P67 final phase / drogue.

**Pad** (data)
: The "PAD" message — a structured block of numbers (TIG, ΔV components, burn duration, attitude, etc.) read up to the crew by CapCom and entered via DSKY into P30. Defined in mission flight rules; not a Rust artifact.

**Phasing**
: Adjustment of orbital period so two vehicles arrive at the same point at the same time. Goal of CSI/CDH/TPI sequence.

**RCS** — Reaction Control System
: Small hypergolic jets used for attitude control (always) and very small ΔV burns (P41). Each CSM SM cluster (quad) has four 100 lbf jets. Modelled by `hal::rcs` and `control::rcs_logic`.

**REFSMMAT** — see [Reference Frames](#reference-frames--coordinate-systems).

**RETRO** — Retrofire Officer
: Mission Control specialist responsible for the deorbit / entry maneuvers and the entry trajectory. Inputs the entry pad. Not represented in the port; the equivalent computation is P37.

**RN / VN** (and RN1 / VN1)
: AGC erasable symbols for the CSM (RN/VN) and target (RN1/VN1) position/velocity 6-vectors. Octal 0306–0321. Maps to `AgcState::csm_state` / `AgcState::target` (positions in B+28 m, velocities in B+7 m/s).

**S-IVB**
: Saturn V third stage, performs orbital insertion and TLI. Its burn is monitored by P15 but not commanded by the AGC.

**SECS** — Sequential Events Control System
: CSM event-controller for pyrotechnic actions (CM/SM sep, drogue, main chutes, RCS jettison). Modelled by `hal/secs.rs`.

**Servicer** — see [Navigation & Guidance Algorithms](#navigation--guidance-algorithms) → SERVICER.

**TPI / TPM / TPF**
: Terminal-Phase Initiation / Terminal-Phase Midcourse / Terminal-Phase Finalize. See [Trajectory & Burn Planning](#trajectory--burn-planning).

**Update Liaison (P27)**
: The AGC program that accepts ground uplink data. V70/V71/V72/V73 verbs drive state-vector and constant updates. The Rust port simulates uplink through `services::v_n::p27_apply_word`; see `docs/p40_burn_demo.md`.

**V37**
: Verb 37 — "Select major mode". `V37E NNE` requests program NN. The Rust dispatch is `PROGRAM_TABLE[NN]` in `agc-core/src/programs/mod.rs`.

**V71**
: Verb 71 — block address update under P27. The mechanism used to load a state vector or change the gravity-body selector in the simulator demos.
