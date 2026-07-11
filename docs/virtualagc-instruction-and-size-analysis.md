<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Comanche055 Instruction-Mix and Binary-Size Analysis

**Scope:** Comanche055 (Apollo 11 Command Module AGC flight rope), as digitized in the
VirtualAGC project's `Comanche055.lst` / `Comanche055.bin`.

**Purpose:** Quantify, per source module, how much of the assembled binary is native
"basic" machine code versus code running on the AGC's software interpreter, how large
each module is in AGC words, and how much binary size the interpreter design saves (or
makes possible in the first place). This report is descriptive analysis of the flight
software as built; it does not itself constrain the Rust port, but it explains why the
original code is organized the way it is, which is directly relevant when deciding which
Comanche055 components map to hand-written Rust "hot path" logic versus higher-level
vector/matrix math libraries in the port.

---

## 1. Methodology

- **Source of truth:** `Comanche055.lst`, the yaYUL assembler listing, cross-referenced
  against yaYUL's own Block-2 opcode tables.
- **Instruction classification.** Every emitted AGC word was classified by the mnemonic
  in the listing's opcode column, against three disjoint yaYUL tables:
  - `OP_BASIC` — the 58 basic machine opcodes/aliases (native Block-2 instructions such
    as `TC`, `CA`, `AD`, `MASK`, `DXCH`, etc.).
  - `InterpreterOpcodesBlock2` (172 entries) plus the four STORE-family variants
    (`STORE`, `STOVL`, `STODL`, `STCALL`) — together `OP_INTERPRETER`, the interpretive
    opcode set executed by the `INTERPRETER.agc` virtual machine.
  - `OP_PSEUDO` / `OP_DOWNLINK` — assembler pseudo-ops that emit data or constant words
    (`DEC`, `OCT`, `2DEC`, `2CADR`, `VN`, download-list entries, etc.), not instructions.
- **Word counting.** Octal-word tokens are counted per listing line. Two-word constant
  pseudo-ops (`2DEC`, `2CADR`, and similar) emit two octal words on a single listing
  line and are counted as 2 words.
- **Interpretive opcode packing.** The AGC interpreter packs up to **two** interpretive
  opcodes into a single 15-bit fixed-memory word (e.g. a listing line reading
  `ITA VLOAD` or `MXV VSL1` holds two opcodes in one word). Each packed opcode is
  counted as one interpretive instruction; a packed pair therefore counts as 2
  instructions occupying 1 word. Operand addresses that follow an interpretive opcode
  (e.g. the erasable address loaded by `VLOAD`) each occupy their own separate word and
  are counted in the `iOpd` (interpretive operand words) column, distinct from the
  `iOpc` (interpretive opcode words) column.
- **Validation.** The parser's independently computed total of emitted fixed words
  (36,504) exactly matches yaYUL's own per-bank occupancy report, summed across all 36
  fixed banks (36,504 of 36,864 words of capacity). Because the two counting methods are
  structurally independent (mnemonic-driven word classification vs. per-bank
  fixed-memory occupancy accounting) and agree exactly, the word totals in this report
  are treated as validated, not estimated.

---

## 2. Validated totals

| Category | Instructions | Words |
|---|---:|---:|
| Basic (machine) instructions | 18,119 | 18,119 |
| Interpretive instructions | 9,875 | 6,550 (opcode words, packed 1.51/word) + 7,055 (operand words) = 13,605 |
| Data / constant words | — | 4,780 |
| **Total used fixed words** | | **36,504** of 36,864 capacity |

- Fixed-memory capacity: 36 fixed banks x 1,024 words = 36,864 words.
- Used: 36,504 words (99.02%). Unused: 360 words (0.98%) across the whole rope.
- Memory share of used fixed memory: **basic 49.6%**, **interpretive 37.3%**, **data
  13.1%**.
- Net effect: the 9,875 interpretive instructions occupy only 37% of used memory, while
  the 18,119 basic instructions — roughly twice as many instructions — occupy 50%. Per
  instruction, interpretive code is markedly denser than native code (see Section 5).

---

## 3. Module inventory (instruction mix and size)

The table below lists every `.agc` source module contributing to Comanche055, sorted by
total emitted words (largest first), as produced by the listing parser. Columns:

- **words** — total fixed-memory words the module contributes to the rope.
- **basicI** — basic (native machine) instruction count.
- **intI** — interpretive instruction count (opcode tokens, packed 2/word where
  applicable).
- **iOpc** — interpretive *opcode* words (after packing).
- **iOpd** — interpretive *operand* words.
- **data** — data/constant words (pseudo-op emitted).
- **%interp** — interpretive instructions as a percentage of all instructions in the
  module, `intI / (basicI + intI)`. A dash (`-`) marks modules that contain no
  instructions at all (pure data/constant tables or non-code source, e.g. banner
  headers, erasable-memory assignment lists).

| Module | words | basicI | intI | iOpc | iOpd | data | %interp |
|---|---:|---:|---:|---:|---:|---:|---:|
| P20-P25.agc | 2,724 | 625 | 1,257 | 884 | 962 | 253 | 67% |
| PINBALL_GAME_BUTTONS_AND_LIGHTS.agc | 2,379 | 2,112 | 0 | 0 | 0 | 267 | 0% |
| INTERPRETER.agc | 2,080 | 2,020 | 0 | 0 | 0 | 60 | 0% |
| P40-P47.agc | 1,667 | 804 | 459 | 308 | 299 | 256 | 36% |
| P51-P53.agc | 1,416 | 450 | 544 | 384 | 421 | 161 | 55% |
| P37_P70.agc | 1,368 | 82 | 854 | 556 | 594 | 136 | 91% |
| REENTRY_CONTROL.agc | 1,172 | 93 | 603 | 378 | 484 | 217 | 87% |
| CONIC_SUBROUTINES.agc | 1,079 | 14 | 724 | 442 | 520 | 103 | 98% |
| IMU_CALIBRATION_AND_ALIGNMENT.agc | 1,077 | 543 | 321 | 233 | 184 | 117 | 37% |
| P34-35_P74-75.agc | 1,065 | 128 | 678 | 448 | 447 | 42 | 84% |
| FRESH_START_AND_RESTART.agc | 893 | 657 | 19 | 12 | 21 | 203 | 3% |
| P32-P33_P72-P73.agc | 882 | 46 | 596 | 378 | 407 | 51 | 93% |
| T4RUPT_PROGRAM.agc | 833 | 773 | 0 | 0 | 0 | 60 | 0% |
| ORBITAL_INTEGRATION.agc | 816 | 6 | 530 | 329 | 398 | 83 | 99% |
| CM_ENTRY_DIGITAL_AUTOPILOT.agc | 813 | 761 | 0 | 0 | 0 | 52 | 0% |
| EXTENDED_VERBS.agc | 703 | 540 | 22 | 17 | 18 | 128 | 4% |
| INTEGRATION_INITIALIZATION.agc | 685 | 130 | 323 | 198 | 315 | 42 | 71% |
| DISPLAY_INTERFACE_ROUTINES.agc | 667 | 609 | 0 | 0 | 0 | 58 | 0% |
| IMU_MODE_SWITCHING_ROUTINES.agc | 656 | 604 | 0 | 0 | 0 | 52 | 0% |
| JET_SELECTION_LOGIC.agc | 629 | 556 | 0 | 0 | 0 | 73 | 0% |
| RCS-CSM_DIGITAL_AUTOPILOT.agc | 611 | 541 | 0 | 0 | 0 | 70 | 0% |
| PINBALL_NOUN_TABLES.agc | 561 | 41 | 0 | 0 | 0 | 520 | 0% |
| P11.agc | 535 | 285 | 163 | 112 | 89 | 49 | 36% |
| P61-P67.agc | 495 | 148 | 195 | 128 | 132 | 87 | 57% |
| TVCDAPS.agc | 489 | 481 | 0 | 0 | 0 | 8 | 0% |
| ANGLFIND.agc | 479 | 20 | 364 | 241 | 205 | 13 | 95% |
| TPI_SEARCH.agc | 443 | 36 | 279 | 181 | 198 | 28 | 89% |
| MEASUREMENT_INCORPORATION.agc | 385 | 21 | 250 | 159 | 196 | 9 | 92% |
| SERVICER207.agc | 375 | 232 | 65 | 41 | 35 | 67 | 22% |
| P30-P37.agc | 374 | 87 | 138 | 96 | 110 | 81 | 61% |
| EXECUTIVE.agc | 337 | 328 | 0 | 0 | 0 | 9 | 0% |
| AUTOMATIC_MANEUVERS.agc | 333 | 318 | 0 | 0 | 0 | 15 | 0% |
| SXTMARK.agc | 315 | 275 | 0 | 0 | 0 | 40 | 0% |
| AGC_BLOCK_TWO_SELF-CHECK.agc | 314 | 299 | 0 | 0 | 0 | 15 | 0% |
| R30.agc | 283 | 98 | 102 | 77 | 79 | 29 | 51% |
| UPDATE_PROGRAM.agc | 280 | 253 | 2 | 2 | 1 | 24 | 1% |
| TVCROLLDAP.agc | 278 | 257 | 0 | 0 | 0 | 21 | 0% |
| RESTART_TABLES.agc | 269 | 32 | 0 | 0 | 0 | 237 | 0% |
| TIME_OF_FREE_FALL.agc | 268 | 2 | 194 | 122 | 115 | 29 | 99% |
| TVCINITIALIZE.agc | 259 | 212 | 0 | 0 | 0 | 47 | 0% |
| CSM_GEOMETRY.agc | 253 | 19 | 128 | 90 | 86 | 58 | 87% |
| WAITLIST.agc | 252 | 224 | 0 | 0 | 0 | 28 | 0% |
| IMU_COMPENSATION_PACKAGE.agc | 247 | 239 | 0 | 0 | 0 | 8 | 0% |
| R60_62.agc | 227 | 76 | 89 | 60 | 65 | 26 | 54% |
| DOWNLINK_LISTS.agc | 226 | 0 | 0 | 0 | 0 | 226 | - |
| STABLE_ORBIT_-_P38-P39.agc | 225 | 62 | 97 | 68 | 84 | 11 | 61% |
| STAR_TABLES.agc | 223 | 0 | 0 | 0 | 0 | 223 | - |
| R31.agc | 218 | 41 | 108 | 78 | 93 | 6 | 72% |
| RTB_OP_CODES.agc | 206 | 173 | 21 | 12 | 13 | 8 | 11% |
| PLANETARY_INERTIAL_ORIENTATION.agc | 204 | 0 | 147 | 94 | 92 | 18 | 100% |
| CM_BODY_ATTITUDE.agc | 195 | 69 | 99 | 69 | 51 | 6 | 59% |
| POWERED_FLIGHT_SUBROUTINES.agc | 187 | 132 | 36 | 29 | 18 | 8 | 21% |
| INFLIGHT_ALIGNMENT_ROUTINES.agc | 186 | 3 | 120 | 89 | 88 | 6 | 98% |
| RESTARTS_ROUTINE.agc | 185 | 176 | 0 | 0 | 0 | 9 | 0% |
| KALCMANU_STEERING.agc | 179 | 149 | 17 | 13 | 8 | 9 | 10% |
| DOWN-TELEMETRY_PROGRAM.agc | 173 | 164 | 0 | 0 | 0 | 9 | 0% |
| LATITUDE_LONGITUDE_SUBROUTINES.agc | 159 | 0 | 114 | 75 | 74 | 10 | 100% |
| PHASE_TABLE_MAINTENANCE.agc | 159 | 150 | 0 | 0 | 0 | 9 | 0% |
| TVCEXECUTIVE.agc | 144 | 136 | 0 | 0 | 0 | 8 | 0% |
| SERVICE_ROUTINES.agc | 123 | 120 | 0 | 0 | 0 | 3 | 0% |
| GROUND_TRACKING_DETERMINATION_PROGRAM.agc | 108 | 18 | 51 | 37 | 42 | 11 | 74% |
| ALARM_AND_ABORT.agc | 108 | 94 | 0 | 0 | 0 | 14 | 0% |
| P76.agc | 96 | 22 | 44 | 35 | 31 | 8 | 67% |
| FIXED_FIXED_CONSTANT_POOL.agc | 94 | 0 | 0 | 0 | 0 | 94 | - |
| TVCMASSPROP.agc | 91 | 61 | 0 | 0 | 0 | 30 | 0% |
| TVCRESTARTS.agc | 90 | 74 | 0 | 0 | 0 | 16 | 0% |
| S-BAND_ANTENNA_FOR_CM.agc | 80 | 16 | 48 | 30 | 30 | 4 | 75% |
| INTER-BANK_COMMUNICATION.agc | 76 | 76 | 0 | 0 | 0 | 0 | 0% |
| TVCSTROKETEST.agc | 73 | 62 | 0 | 0 | 0 | 11 | 0% |
| LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc | 71 | 0 | 57 | 32 | 35 | 4 | 100% |
| KEYRUPT_UPRUPT.agc | 68 | 64 | 0 | 0 | 0 | 4 | 0% |
| INTERRUPT_LEAD_INS.agc | 58 | 49 | 0 | 0 | 0 | 9 | 0% |
| GIMBAL_LOCK_AVOIDANCE.agc | 55 | 0 | 15 | 11 | 14 | 30 | 100% |
| SYSTEM_TEST_STANDARD_LEAD_INS.agc | 36 | 32 | 2 | 2 | 1 | 1 | 6% |
| MYSUBS.agc | 35 | 35 | 0 | 0 | 0 | 0 | 0% |
| INTERPRETIVE_CONSTANTS.agc | 35 | 0 | 0 | 0 | 0 | 35 | - |
| RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc | 33 | 32 | 0 | 0 | 0 | 1 | 0% |
| SINGLE_PRECISION_SUBROUTINES.agc | 32 | 32 | 0 | 0 | 0 | 0 | 0% |
| TAGS_FOR_RELATIVE_SETLOC.agc | 7 | 0 | 0 | 0 | 0 | 7 | - |
| MAIN.agc | 0 | 0 | 0 | 0 | 0 | 0 | - |
| CONTRACT_AND_APPROVALS.agc | 0 | 0 | 0 | 0 | 0 | 0 | - |
| ASSEMBLY_AND_OPERATION_INFORMATION.agc | 0 | 0 | 0 | 0 | 0 | 0 | - |
| ERASABLE_ASSIGNMENTS.agc | 0 | 0 | 0 | 0 | 0 | 0 | - |
| ENTRY_LEXICON.agc | 0 | 0 | 0 | 0 | 0 | 0 | - |
| LUNAR_LANDMARK_SELECTION_FOR_CM.agc | 0 | 0 | 0 | 0 | 0 | 0 | - |
| **TOTAL (85 modules)** | **36,504** | **18,119** | **9,875** | **6,550** | **7,055** | **4,780** | **35%** |

(The six zero-word entries at the bottom are source files that contribute no
fixed-memory content — assembly banner/title pages, the erasable-memory assignment
list, and lunar-landmark tables that are stubbed out of the Comanche (Command Module)
assembly because lunar landing is not flown by the CM.)

---

## 4. Q1 — Instruction mix per module: interpreter-heavy math vs. basic-only real-time code

The `%interp` column above splits the 85 modules into two sharply distinct populations,
with almost no middle ground:

### 4.1 Interpreter-dominated modules (guidance, navigation, targeting math)

| Module | %interp | Role |
|---|---:|---|
| ORBITAL_INTEGRATION.agc | 99% | Encke-method orbit integrator |
| TIME_OF_FREE_FALL.agc | 99% | Free-fall time-of-flight iteration |
| CONIC_SUBROUTINES.agc | 98% | Conic (two-body) orbit math library |
| INFLIGHT_ALIGNMENT_ROUTINES.agc | 98% | IMU inflight star-alignment computation |
| ANGLFIND.agc | 95% | Gimbal-angle solving |
| P32-P33_P72-P73.agc | 93% | P32/P33/P72/P73 rendezvous targeting |
| MEASUREMENT_INCORPORATION.agc | 92% | Kalman-filter-style measurement update |
| P37_P70.agc | 91% | P37 (return-to-earth) / P70 (DAP calc) |
| TPI_SEARCH.agc | 89% | Transfer-phase-initiation search |
| CSM_GEOMETRY.agc | 87% | CM geometric transforms |
| REENTRY_CONTROL.agc | 87% | Entry guidance (Corridor/lift control) |
| P34-35_P74-75.agc | 84% | P34/P35/P74/P75 rendezvous targeting |
| R31.agc | 72% | Rendezvous routine |
| INTEGRATION_INITIALIZATION.agc | 71% | State-vector integration setup |
| GROUND_TRACKING_DETERMINATION_PROGRAM.agc | 74% | Ground-track computation |
| P30-P37.agc | 61% | P30 (external Δv)/targeting |
| STABLE_ORBIT_-_P38-P39.agc | 61% | Stable-orbit rendezvous programs |
| P76.agc | 67% | Target-delta-V update |
| P20-P25.agc | 67% | Rendezvous navigation/tracking |
| P61-P67.agc | 57% | Entry preparation programs |
| P51-P53.agc | 55% | IMU alignment programs |
| P40-P47.agc | 36% | Thrusting maneuver programs (mixed — see below) |

All of these are **guidance, navigation, and targeting math**: numerical integration of
orbital state vectors, conic-section (two-body) trajectory solutions, rendezvous
targeting iterations, star-sighting/IMU alignment geometry, entry guidance corridor
control, and Kalman-style navigation updates. They are dominated (55-99% of
instructions) by interpretive opcodes.

### 4.2 Basic-only modules (executive, real-time, DAP, DSKY, self-check)

The following modules contain **zero** interpretive instructions — every single
instruction is a native basic machine opcode:

| Module | basicI | Role |
|---|---:|---|
| PINBALL_GAME_BUTTONS_AND_LIGHTS.agc | 2,112 | DSKY keyboard/display driver ("Pinball") |
| INTERPRETER.agc | 2,020 | The interpreter virtual machine itself |
| T4RUPT_PROGRAM.agc | 773 | Timer-4 interrupt (DAP servo timing) |
| CM_ENTRY_DIGITAL_AUTOPILOT.agc | 761 | Entry (reentry) digital autopilot |
| DISPLAY_INTERFACE_ROUTINES.agc | 609 | DSKY display formatting/output |
| IMU_MODE_SWITCHING_ROUTINES.agc | 604 | IMU coarse/fine-align mode control |
| JET_SELECTION_LOGIC.agc | 556 | RCS jet on/off selection logic |
| TVCDAPS.agc | 481 | Thrust Vector Control autopilot |
| RCS-CSM_DIGITAL_AUTOPILOT.agc | 541 | RCS attitude-control autopilot |
| EXECUTIVE.agc | 328 | Task scheduler / EXEC |
| AUTOMATIC_MANEUVERS.agc | 318 | Automatic attitude-maneuver sequencer |
| SXTMARK.agc | 275 | Sextant mark-button interrupt handling |
| AGC_BLOCK_TWO_SELF-CHECK.agc | 299 | Power-up/periodic hardware self-test |
| TVCROLLDAP.agc | 257 | TVC roll-axis autopilot |
| TVCINITIALIZE.agc | 212 | TVC initialization |
| WAITLIST.agc | 224 | Delayed-task scheduler (Waitlist) |
| IMU_COMPENSATION_PACKAGE.agc | 239 | IMU error-compensation |
| RESTARTS_ROUTINE.agc | 176 | Restart/power-fail recovery |
| DOWN-TELEMETRY_PROGRAM.agc | 164 | Telemetry downlink formatting |
| PHASE_TABLE_MAINTENANCE.agc | 150 | EXEC job phase-table bookkeeping |
| TVCEXECUTIVE.agc | 136 | TVC job dispatch |
| SERVICE_ROUTINES.agc | 120 | Low-level EXEC/interrupt services |
| ALARM_AND_ABORT.agc | 94 | Program-alarm and abort handling |
| TVCMASSPROP.agc | 61 | TVC mass-properties compensation |
| TVCRESTARTS.agc | 74 | TVC restart handling |
| INTER-BANK_COMMUNICATION.agc | 76 | Cross-bank subroutine call glue |
| TVCSTROKETEST.agc | 62 | TVC gimbal-drive stroke test |
| KEYRUPT_UPRUPT.agc | 64 | DSKY keyboard interrupt |
| INTERRUPT_LEAD_INS.agc | 49 | Interrupt vector table entries |
| MYSUBS.agc | 35 | Misc. low-level subroutines |
| RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc | 32 | RCS DAP job control |
| SINGLE_PRECISION_SUBROUTINES.agc | 32 | Single-precision arithmetic helpers |

This population is exactly the **hard-real-time interrupt, executive, digital
autopilot, DSKY, and self-check** layer of the flight software: the task scheduler
(EXECUTIVE, WAITLIST), the servo/autopilot loops that run every DAP cycle
(CM_ENTRY_DIGITAL_AUTOPILOT, TVCDAPS/TVCROLLDAP, RCS-CSM_DIGITAL_AUTOPILOT,
JET_SELECTION_LOGIC), the timer/keyboard interrupt handlers (T4RUPT_PROGRAM,
KEYRUPT_UPRUPT, SXTMARK), the crew display driver (PINBALL_GAME_BUTTONS_AND_LIGHTS,
DISPLAY_INTERFACE_ROUTINES), IMU mode control, self-check diagnostics, and — notably —
the interpreter engine itself, `INTERPRETER.agc`, which of course cannot be written in
the language it implements.

A handful of modules fall in between (e.g. P40-P47.agc at 36%, IMU_CALIBRATION_AND_ALIGNMENT.agc
at 37%, SERVICER207.agc at 22%, POWERED_FLIGHT_SUBROUTINES.agc at 21%,
KALCMANU_STEERING.agc at 10%): these mix time-critical maneuver sequencing and DAP
interfacing (basic code) with embedded targeting/steering math (interpretive calls into
the math library), which is exactly what the pattern predicts for programs that bridge
the executive/DAP layer and the navigation layer (e.g. P40-P47's thrusting-maneuver
programs must both drive the engine/DAP in real time *and* compute steering targets).

### 4.3 Why the split exists

The AGC interpreter is a **software-implemented virtual machine**, itself written in
~2,020 basic instructions (`INTERPRETER.agc`). It provides:

- Double- and triple-precision fixed-point scalar arithmetic (the AGC's basic word is
  only 15 bits; double precision needs two words, triple precision three).
- Vector (3-component) and 3x3 matrix operations: dot product, cross product,
  matrix-vector multiply, unit-vector normalization.
- Transcendental approximations (sine, cosine, arctangent, square root).
- A compact, stack-oriented instruction encoding (see Section 5) purpose-built to
  minimize the *code size* of these operations, at the cost of *execution time*: each
  interpretive instruction requires the interpreter's fetch-decode-dispatch loop to run
  on top of the basic machine, so interpretive code executes roughly **5-10x slower**
  than equivalent hand-coded basic instructions.

This is precisely the right trade for the two populations identified above:

- **Guidance/navigation/targeting math** (orbit integration, conic solutions,
  rendezvous targeting, alignment geometry, entry guidance) is **not time-critical at
  the instruction level** — a P20-P25 tracking update or a P37 return-to-earth targeting
  solve can take fractions of a second to many seconds of wall-clock time without
  affecting spacecraft safety, and these routines are large, complex, math-heavy, and
  reused across many programs. Interpreter execution slowness is an acceptable price for
  large code-size reduction.
- **Real-time/executive/DAP/DSKY/self-check code** must run within hard deadlines set
  by external timing sources — servo loops driven by T4RUPT at fixed intervals, RCS
  jet on/off decisions that must be issued within a DAP cycle, keyboard/display
  interrupts that must be serviced promptly, and the EXEC/Waitlist scheduler that
  underlies all timing guarantees in the system. None of this code can tolerate a 5-10x
  slowdown, so it is written entirely in basic instructions, even though basic code is
  larger.

---

## 5. Q2 — Module binary sizes and rope memory layout

Comanche055 assembles into a single binary image, `Comanche055.bin` — 73,728 bytes =
36,864 words of 15 (usable) bits each, 2 bytes/word as stored. This is the entire "core
rope" read-only memory of the flight computer's fixed memory, organized as **36 fixed
banks of 1,024 words each**.

### 5.1 Largest modules by word contribution

The three largest modules by far:

| Rank | Module | Words | % of total rope |
|---:|---|---:|---:|
| 1 | P20-P25.agc | 2,724 | 7.4% |
| 2 | PINBALL_GAME_BUTTONS_AND_LIGHTS.agc | 2,379 | 6.5% |
| 3 | INTERPRETER.agc | 2,080 | 5.7% |

These three alone account for **7,183 words — 19.6% of the entire 36,864-word fixed
store**. P20-P25 (rendezvous navigation and tracking programs) is the largest single
module in the rope precisely because it combines a large basic-instruction executive
shell (DSKY interfacing, mode sequencing, radar-mark processing — 625 basic
instructions) with the single biggest interpretive math payload in the system (1,257
interpretive instructions, 884 opcode words + 962 operand words). PINBALL_GAME_BUTTONS_AND_LIGHTS
is the largest all-basic module because it implements the entire DSKY keyboard/display
state machine — verb/noun entry, flashing, component (register) formatting — which is
inherently a real-time, table-driven state machine best done in native code.
INTERPRETER.agc, at 2,080 words (2,020 basic + 60 data, 0 interpretive — it cannot
interpret itself), is the fixed cost of the virtual machine that everything in Section
4.1 depends on.

The full ranked table is given in Section 3 (sorted by the `words` column, largest
first).

### 5.2 Rope fill / memory pressure

Across the full module inventory, **36,504 of 36,864 fixed words are used (99.02%)**,
leaving only **360 words (0.98%) free** in the entire fixed store. At the individual
bank level, per-bank occupancy (from yaYUL's independent bank-occupancy report) ranges
from roughly **1,757 to 1,777 octal words used out of a 2,000-octal-word (1,024-decimal)
capacity per bank — approximately 98% to 100% full**, consistent with the aggregate
99.02% figure.

This is the dominant physical constraint under which Comanche055 was engineered: the
rope memory was fabricated as a fixed, non-reprogrammable core-rope module, and once a
bank filled up there was no "just allocate more RAM" option — code had to be
restructured, moved to another bank, or rewritten more compactly to fit. This is the
direct motivation for Section 6's central finding: the interpreter is not a convenience,
it is what makes the guidance/navigation software *fit* at all.

---

## 6. Q3 — How much does the interpreter reduce binary size?

There are two distinct, and differently certain, size effects. They must be kept
separate: (A) is a direct measurement from the word-count data; (B) is an engineering
estimate based on typical AGC interpretive-vs-basic code-density ratios, not a
measurement.

### 6.1 (A) Directly measured: interpretive opcode packing

The interpreter packs **two interpretive opcodes per 15-bit fixed word** wherever the
listing shows two mnemonics on one line (e.g. `ITA VLOAD`, `MXV VSL1`). This is a purely
mechanical encoding trick — a 7-bit interpretive opcode leaves room for a second one in
the same word — and its effect is directly measurable from the validated word counts:

| Quantity | Value |
|---|---:|
| Interpretive instructions (opcode tokens) | 9,875 |
| Words those opcodes actually occupy (packed, 1.51 opcodes/word) | 6,550 |
| Words they *would* occupy unpacked (1 opcode/word) | 9,875 |
| Interpretive operand words (unaffected by packing) | 7,055 |
| Interpretive region, packed (actual) | 6,550 + 7,055 = **13,605 words** |
| Interpretive region, if unpacked | 9,875 + 7,055 = **16,930 words** |
| **Saving from opcode packing alone** | **3,325 words** |

That saving of 3,325 words is:

- **19.6%** of the interpretive code region (13,605 vs. the 16,930 it would need
  unpacked).
- **9.1%** of *all* used fixed memory (3,325 of 36,504).

This is the direct, quantified answer to "interpretive instructions are smaller — how
does that reduce binary size": the packed 7-bit-opcode encoding alone removes about
3,325 words from the rope — roughly **3.25 of the rope's 36 banks worth of memory** —
independent of any argument about what the interpreter lets you avoid writing in the
first place.

### 6.2 (B) Modeled, larger effect: subroutinization of common math operations

The much bigger size effect is architectural rather than encoding-level. A single
interpretive instruction is not just "smaller than a basic instruction" — it typically
*invokes a shared library routine* that performs an entire double- or triple-precision
arithmetic operation, or a whole vector/matrix operation (dot product, cross product,
matrix-vector multiply, unit-vector normalization, square root, sine/cosine), in one
interpreter-visible step. An interpretive instruction averages about 1.5 words (its
opcode share of a packed word, plus an operand word) to invoke functionality that,
written inline in basic instructions, commonly takes on the order of **10 to 40+ words**
per operation (double-precision add/subtract/multiply/divide, a 3-vector dot or cross
product, a 3x3 matrix-vector multiply, etc., each require multiple basic instructions
per component, with carry/overflow handling for multi-word precision).

This cannot be measured directly from the listing (there is no de-interpreted "basic
equivalent" of Comanche055 to count), so it must be presented as an **explicit
engineering estimate**, not a single false-precision number:

| Assumption: average basic-code expansion per interpretive operation | Basic words needed for 9,875 operations |
|---:|---:|
| 3 basic words/op (conservative low end) | ~29,600 words |
| 5 basic words/op (still conservative) | ~49,400 words |
| 10-40+ words/op (typical for DP arithmetic, vector/matrix ops) | ~99,000 - 395,000+ words |

Even at the conservative low end of this range, expanding the 9,875 interpretive
operations into inline basic code would require on the order of **30,000 to 50,000+
words** — compared to the 13,605 words the same functionality occupies today as
interpretive code. Added to the 18,119 words of basic code and 4,780 words of data that
already occupy the rope, this would push total fixed-memory demand well past the
36,864-word capacity of the entire rope, using the low end of the estimate alone, and
far beyond it at realistic per-operation expansion factors. In short: **the
guidance/navigation/targeting software, as written, would not fit in the CM's fixed
memory without the interpreter.**

Framed as an investment: `INTERPRETER.agc` costs **2,080 words** of fixed memory
(2,020 basic instructions implementing the fetch/decode/dispatch engine and its
arithmetic/vector/matrix/transcendental primitive routines, plus 60 words of constants)
— under 6% of the rope. That roughly 2K-word fixed overhead is what makes the
math-heavy 37% of the rope (13,605 words of interpretive guidance/navigation code)
possible at all within a 36,864-word budget; without it, that same functionality,
written natively, would not fit regardless of how the remaining fixed banks were
allocated.

### 6.3 The classic AGC trade

Both effects reflect the same underlying trade, made deliberately by the original
Colossus/Comanche software team under a hard, fixed memory ceiling: **the interpreter
sacrifices execution speed (roughly 5-10x slower than equivalent basic code) in
exchange for code density**, via (A) a directly-measured ~9% reduction from opcode
packing and (B) a much larger, non-directly-measurable reduction from replacing inline
multi-word arithmetic/vector/matrix sequences with single subroutine-call-like
interpretive instructions. Because the fixed-memory rope was a hard, non-expandable
physical constraint (Section 5.2: 99% full), and because none of the math-heavy
guidance/navigation/targeting code is hard-real-time at the instruction level (Section
4.3), this trade — slow but dense math code, fast but larger real-time/DAP/executive
code — was the correct engineering choice, and it is the reason Comanche055's code is
organized into the two clearly separated populations documented in Section 4.

---

## 7. Caveats / method limitations

- **Packed-pair detection.** A second interpretive mnemonic appearing on the same
  listing line as a first is classified as a packed opcode sharing that word. This
  matches the documented Block-2 interpreter encoding but was not independently
  re-derived from the interpreter's dispatch tables beyond yaYUL's own classification.
- **STORE-family variants** (`STORE`, `STOVL`, `STODL`, `STCALL`) are counted as single
  interpretive opcode tokens (as yaYUL's `OP_INTERPRETER` table treats them), not
  decomposed into sub-operations.
- **Bank guard words.** The 36 per-bank "bugger word" / checksum guard words (one per
  fixed bank, used by the AGC's power-up self-check to validate rope integrity) are
  included in the totals as ordinary data words; they are not separately broken out.
- **Section 6.2's expansion-factor estimate is an engineering estimate, not a
  measurement.** No de-interpreted equivalent of Comanche055 exists to count directly;
  the 3-40+ words/operation range is based on typical AGC basic-instruction sequence
  lengths for multi-precision arithmetic and vector/matrix operations as documented in
  AGC architecture references, not on a word-by-word reconstruction of what an
  all-basic Comanche055 would contain.
- **Module boundaries** follow the `.agc` source file split as assembled by yaYUL; some
  logical "programs" (e.g. P32/P33/P72/P73, P34/P35/P74/P75) are combined into single
  source files by the original authors and are reported here as single rows.

---

## 8. Summary

| Finding | Value |
|---|---|
| Total emitted fixed words | 36,504 / 36,864 (99.02% full) |
| Basic instructions | 18,119 (49.6% of used memory) |
| Interpretive instructions | 9,875 (37.3% of used memory, packed into 6,550 opcode words + 7,055 operand words) |
| Data/constant words | 4,780 (13.1% of used memory) |
| Largest modules | P20-P25 (2,724 w), PINBALL_GAME_BUTTONS_AND_LIGHTS (2,379 w), INTERPRETER (2,080 w) |
| Opcode-packing saving (measured) | 3,325 words (9.1% of used memory) |
| Subroutinization saving (estimated) | Tens of thousands of words — without it, the program would not fit |
| Interpreter engine cost | 2,080 words (INTERPRETER.agc) |
