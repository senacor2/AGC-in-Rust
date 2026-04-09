# Spaceflight Computing — Context & Navigation Index

Two source documents are indexed here for fast lookup. All page numbers refer to **book/document page numbers** as printed on each page (headers/footers), not PDF viewer page numbers.

--- 

## PDFs on File System

| Document | File Location |
|---|---|
`The Apollo Guidance Computer.pdf` | '/Users/Vitaliy.Schreibmann/Downloads/The Apollo Guidance Computer.pdf' |
`Computers in Spaceflight.pdf` | '/Users/Vitaliy.Schreibmann/Downloads/Computers in Spaceflight.pdf' |


---

## PDF Page Offset Guide

To convert a **book page number → PDF viewer page number**, apply the offset:

| Document | File | Offset |
|---|---|---|
| **AGC** | `The Apollo Guidance Computer.pdf` | **book page + 16** = PDF page | 
| **CIS** | `Computers in Spaceflight.pdf` | **book page + 9** = PDF page |

> Example: AGC book p. 37 → PDF p. 53 · CIS book p. 27 → PDF p. 36

---

## Document 1 — The Apollo Guidance Computer: Architecture and Operation

**Author:** Frank O'Brien  
**Publisher:** Springer/Praxis, 2010  
**ISBN:** 978-1-4419-0876-6  
**Total pages:** ~431  
**Scope:** Block II AGC used on all manned Apollo flights. Covers hardware architecture, operating system (Executive), Interpreter language, guidance/navigation fundamentals, and mission programs.

---

### Front Matter

| Section | Book Page | PDF Page |
|---|---|---|
| List of Figures | ix–xii | 25–28 |
| Author's Preface | xiii–xv | 29–31 |
| Acknowledgments | xvii–xviii | 33–34 |

---

### Chapter 0 — The State of the Art (Book pp. 1–9 · PDF pp. 17–25)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| From whence we came: early computing | 1 | 17 |
| Outside the computer room: early computing in aviation and space | 2 | 18 |
| Computing in manned spacecraft (Mercury, Gemini overview) | 3 | 19 |
| Defining computer "power" (clock speed fallacy, AGC comparison) | 4–5 | 20–21 |
| The evolution of computing (assembly language era, transistors) | 7 | 23 |
| Technology acquisition: consumers vs. the aerospace industry | 8–9 | 24–25 |

**Key content:** Contextual prologue explaining why AGC capabilities cannot be judged by modern metrics. Explains technology freeze in aerospace procurement.

---

### Chapter 1 — The AGC Hardware (Book pp. 11–97 · PDF pp. 27–113)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Introduction (CPU, memory, I/O fundamentals) | 11–14 | 27–30 |
| Overview of Chapter 1 | 15–16 | 31–32 |
| **Physical characteristics of the AGC** (size, weight, location) | 16–18 | 32–34 |
| **Properties of number systems** | 18–27 | 34–43 |
| — Binary and octal notation | 18 | 34 |
| — One's complement notation | 20–22 | 36–38 |
| — Floating point / fractional notation (slide rule analogy) | 22–25 | 38–41 |
| — Scaling, precision and accuracy | 25 | 41 |
| **Double precision numbers** | 27 | 43 |
| **FIGMENT** (imaginary computer for teaching memory banking) | 29 | 45 |
| **Instructions: basic units of computer operation** | 29–31 | 45–47 |
| **Memory management** | 31–37 | 47–53 |
| — Core memory (erasable) | 31–35 | 47–51 |
| — Core rope (fixed/read-only) | 35–37 | 51–53 |
| **A tour of low core and central registers** | 37–47 | 53–63 |
| — Accumulator (A), L, Q registers | 37 | 53 |
| — Z (program counter), BB (bank) | ~40 | ~56 |
| — Editing registers (CYR, SR, CYL, EDOP) | ~42 | ~58 |
| **Keeping time: timers and clocks** | 47–51 | 63–67 |
| — Master clock (2.048 MHz) | 47 | 63 |
| — TIME1–TIME6 registers | ~48 | ~64 |
| — T4RUPT timing | ~49 | ~65 |
| **Counters — CDUS (X,Y,Z), OPTS, OPTT and PIPAS (X,Y,Z)** | 51–55 | 67–71 |
| — Inertial Measurement Unit interface | 51 | 67 |
| **Radar, engine and crew interfaces** | 55–59 | 71–75 |
| — Codes for radar data source selection (Fig. 14) | 57 | 73 |
| — LM tapemeters: altitude and altitude rate (Fig. 15) | 59 | 75 |
| **Memory addressing and banking in the AGC** | 59–68 | 75–84 |
| — Erasable storage and bits 10/9 (Fig. 16) | 61 | 77 |
| — Erasable storage banking (Fig. 17) | 62 | 78 |
| — Fixed storage and bits 12/11 (Fig. 18) | 63 | 79 |
| — Fixed storage banking (Fig. 19) | 63 | 79 |
| — View of both erasable and fixed storage (Fig. 20) | 64 | 80 |
| **Interrupt processing** | 68–70 | 84–86 |
| **The instruction set** | 70–87 | 86–103 |
| — AGC instruction format (Fig. 7) | 30 | 46 |
| — Instruction format with quarter code (Fig. 21) | 72 | 88 |
| — AND, OR, XOR logical operations (Fig. 22) | 77 | 93 |
| — TS instruction and overflow processing (Fig. 23) | 79 | 95 |
| — Summing five entries in a table (Fig. 24) | 85 | 101 |
| — Special case instructions (Fig. 25) | 86 | 102 |
| **Communicating with outside world: the I/O system** | 87–97 | 103–113 |
| — Characteristics of counter registers and I/O channels (Fig. 26) | 90 | 106 |
| — The AGC and its I/O devices (Fig. 27) | 91 | 107 |
| — I/O instruction format (Fig. 28) | 92 | 108 |
| — I/O channel usage (Fig. 29) | 93 | 109 |

---

### Chapter 2 — The Executive and Interpreter (Book pp. 99–197 · PDF pp. 115–213)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Introduction to the Executive | 99–101 | 115–117 |
| **Scheduling: preemptive and cooperative multiprogramming** | 101–103 | 117–119 |
| **The Executive** (Core Sets, VAC areas, Waitlist) | 103–123 | 119–139 |
| — Core Set layout (Fig. 30) | 105 | 121 |
| — Core Sets and job scheduling (Fig. 31) | 106 | 122 |
| — Vector Accumulator area layout (Fig. 32) | 107 | 123 |
| — Allocated Core Sets and VAC areas (Fig. 33) | 108 | 124 |
| — Waitlist tables (Fig. 34) | 115 | 131 |
| — Addition of a new waitlist task (Fig. 35) | 116 | 132 |
| — Restart and phase tables (Fig. 36) | 118 | 134 |
| — The +phase/–phase tables (Fig. 37) | 120 | 136 |
| **The astronaut interface: the display and keyboard (DSKY)** | 123–140 | 139–156 |
| — Display and keyboard diagram (Fig. 38) | 124 | 140 |
| — Diagram of the DSKY (Fig. 39) | 126 | 142 |
| — Noun tables (Fig. 40) | 139 | 155 |
| **Telemetry uplink** | 140–143 | 156–159 |
| — Data in INLINK register (Fig. 41) | 141 | 157 |
| — Sample Verb 71 in uplink (Fig. 42) | 142 | 158 |
| **Synchronous I/O processing and T4RUPT** | 143–150 | 159–166 |
| — LM rendezvous radar at top of ascent stage (Fig. 43) | 146 | 162 |
| **High level languages and the Interpreter** | 150–160 | 166–176 |
| — Summing a table (Space Shuttle computer reference, Fig. 44) | 150 | 166 |
| **The Interpreter** | 160–197 | 176–213 |
| — Interpreter half-memories (Fig. 50) | 162 | 178 |
| — Generic interpretive instruction format (Fig. 51) | 164 | 180 |
| — Multipurpose Accumulator and datatype modes (Fig. 52) | 168 | 184 |
| — Interpretive instruction on index register (Fig. 53) | 170 | 186 |
| — Indexing into star catalog table (Fig. 54) | 171 | 187 |
| — GOTO variants (Figs. 55–57) | 172 | 188 |
| — Orientation of vector space on Earth surface (Fig. 58) | 174 | 190 |
| — Flagword operand format (Fig. 59) | 181 | 197 |
| — Scalar and vector short shift formats (Fig. 60) | 182 | 198 |
| — Long shift address word format (Fig. 61) | 183 | 199 |
| — STORE instruction format (Fig. 62) | 187 | 203 |
| — Uses of STORE and STADR (Fig. 63) | 189 | 205 |
| — Computed GOTO example (Fig. 64) | 193 | 209 |

---

### Chapter 3 — The Basics of Guidance and Navigation (Book pp. 199–229 · PDF pp. 215–245)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Hardware unique to guidance and navigation | 199–209 | 215–225 |
| — Schematic of the Inertial Measurement Unit (Fig. 65) | 200 | 216 |
| — Body axes on the CM and LM (Fig. 66) | 201 | 217 |
| — Gimbal lock (Fig. 67) | 201 | 217 |
| — Schematic of a sextant (Fig. 68) | 203 | 219 |
| — Command Module optics systems (Fig. 69) | 205 | 221 |
| — LM Alignment Optical Telescope (Fig. 70) | 206 | 222 |
| — Star sighting through the AOT (Fig. 71) | 208 | 224 |
| **Question 1: Which way is up?** (platform alignment) | 209–217 | 225–233 |
| — Basic reference coordinate system, Earth-centered (Fig. 72) | 211 | 227 |
| — CSM coordinate system (Fig. 73) | 213 | 229 |
| — LM coordinate system (Fig. 74) | 215 | 231 |
| — Two stars creating fixed orientation in space (Fig. 75) | 217 | 233 |
| **Question 2: Where am I?** (cislunar navigation) | 217–222 | 233–238 |
| — Cislunar navigation sighting (Fig. 76) | 219 | 235 |
| — View of horizon and star through sextant (Fig. 77) | 220 | 236 |
| **Question 3: Which way am I going?** (orbital mechanics) | 222–229 | 238–245 |
| — Conic sections (Fig. 78) | 224 | 240 |

---

### Chapter 4 — Mission Programs and Operations (Book pp. 231–363 · PDF pp. 247–379)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| **Introduction** | 231 | 247 |
| **Launch from Earth** | 231–245 | 247–261 |
| — Launch monitor cue card (Fig. 79) | 236 | 252 |
| — TLI cue card (Fig. 80) | 242 | 258 |
| **The lunar landing** | 245–287 | 261–303 |
| — Descent profile: braking, approach, landing phases (Fig. 81) | 245 | 261 |
| — Lunar Module DSKY layout (Fig. 82) | 247 | 263 |
| — Engine gauges and tape instruments (Fig. 83) | 248 | 264 |
| — Landing point designator (Fig. 84) | 250 | 266 |
| — P63 ignition algorithm check (Fig. 114) | 271 | 287 |
| — P64 approach phase cue card (Fig. 129) | 282 | 298 |
| — P66 and lunar contact (Fig. 131) | 283 | 299 |
| **Lunar orbit rendezvous** | 287–312 | 303–328 |
| — Isaac Newton's cannonball experiment (Fig. 134) | 288 | 304 |
| — Orbital planes (Fig. 135) | 289 | 305 |
| — Polar plot from LM rendezvous procedures manual (Fig. 136) | 293 | 309 |
| — Coelliptic rendezvous (Fig. 137) | 295 | 311 |
| — LM ascent cue card (Fig. 138) | 298 | 314 |
| — Direct rendezvous maneuvers (Fig. 139) | 307 | 323 |
| — Alternate LM active rendezvous techniques (Fig. 140) | 311 | 327 |
| **The digital autopilot (DAP)** | 312–334 | 328–350 |
| — Generic phase plane (Fig. 141) | 316 | 332 |
| — DAP phase-plane decision areas (Fig. 142) | 318 | 334 |
| — CSM and LM moment of inertia (Fig. 143) | 320 | 336 |
| — Rotational coupling in the CSM (Fig. 144) | 321 | 337 |
| — LM thruster configuration (Fig. 145) | 326 | 342 |
| **Erasable memory programs** | 334–337 | 350–353 |
| **AGC data uplink and downlink** | 337–341 | 353–357 |
| — Downlist word format (Fig. 148) | 339 | 355 |
| **Command Module entry** | 341–358 | 357–374 |
| — Command Module entry corridor (Fig. 149) | 344 | 360 |
| — Entry maneuvering footprint (Fig. 150) | 344 | 360 |
| — Entry Monitor System panel (Fig. 151) | 346 | 362 |
| — EMS scroll strip (Fig. 152) | 347 | 363 |
| — Entry altitude and range profile (Fig. 155) | 356 | 372 |
| — Abort/Abort Stage pushbuttons in LM (Fig. 156) | 362 | 378 |
| **Computer problems during Apollo 11 and Apollo 14** | 358–363 | 374–379 |

---

### Epilogue (Book pp. 365–367 · PDF pp. 381–383)

---

### Appendixes (Book pp. 369–419 · PDF pp. 385–435)

| Appendix | Content | Book Page | PDF Page |
|---|---|---|---|
| A | AGC instruction set | 369 | 385 |
| B | AGC interrupt vectors | 371 | 387 |
| C | Layout of special and control registers | 372 | 388 |
| D | Command Module I/O channels | 373 | 389 |
| E | Lunar Module I/O channels | 379 | 395 |
| F | Interpreter instruction set | 386 | 402 |
| G | Command Module programs (Major Modes) | 391 | 407 |
| H | Command Module routines | 392 | 408 |
| I | Command Module verbs | 393 | 409 |
| J | Command Module nouns | 395 | 411 |
| K | Command Module program alarms | 401 | 417 |
| L | Lunar Module programs (Major Modes) | 403 | 419 |
| M | Lunar Module routines | 404 | 420 |
| N | Lunar Module verbs | 405 | 421 |
| O | Lunar Module nouns | 407 | 423 |
| P | Lunar Module program alarms | 412 | 428 |
| Q | Command Module and LM downlists | 415 | 431 |
| R | AGC navigation star catalog | 417 | 433 |
| S | Configuring the CSM and LM DAP (Routine 03) | 418 | 434 |

---

### Back Matter (AGC Book)

| Section | Book Page | PDF Page |
|---|---|---|
| Glossary of terms and abbreviations | 421 | 437 |
| Bibliography | 423 | 439 |
| Illustration credits | 427 | 443 |
| Index | 431 | 447 |

---

---

## Document 2 — Computers in Spaceflight: The NASA Experience

**Author:** James E. Tomayko (Wichita State University)  
**Published:** NASA Contractor Report 182505, March 1988  
**Contract:** NASW-3714  
**Total pages:** ~409 (plus appendices)  
**Scope:** Topical history of NASA's computer use from 1958 onward — manned and unmanned spacecraft plus ground support. Three major parts.

---

### Front Matter

| Section | Book Page | PDF Page |
|---|---|---|
| Foreword (Allen Kent & James G. Williams) | vii | 5 |
| Preface (Tomayko) | ix–x | 6–7 |
| Acknowledgements | xi–xii | 8–9 |

---

### Introduction — Computing and Spaceflight: An Introduction (Book pp. 1–5 · PDF pp. 10–14)

| Topic | Book Page | PDF Page |
|---|---|---|
| NASA's use from 1958, from UNIVAC to distributed systems | 1–2 | 10–11 |
| Real-time vs. batch computing requirements | 3–4 | 12–13 |
| On-board vs. ground-based computer requirements | 4–5 | 13–14 |

---

## Part I — Manned Spacecraft Computers (Book pp. 7–134 · PDF pp. 16–143)

**Introduction to Part One** — Mercury, Gemini, Apollo, Skylab, Shuttle overview (Book p. 7 · PDF p. 16)

---

### Chapter 1 — The Gemini Digital Computer: First Machine in Orbit (Book pp. 9–26 · PDF pp. 18–35)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Background: Mercury (no computer) and Gemini requirements | 9–11 | 18–20 |
| — Gemini's 6 mission phases: prelaunch, ascent backup, insertion, catch-up, rendezvous, re-entry | 10 | 19 |
| **HARDWARE** | 11–17 | 20–26 |
| — IBM contract (April 1962, $26.6M), 20 units built | 12 | 21 |
| — Physical specs: 18.9×14.5×12.75 in, 58.98 lbs | 12 | 21 |
| — Core memory: 39-plane × 64×64-bit arrays, 4,096 addresses | 13–14 | 22–23 |
| — Instruction cycle: 140 ms (addition); 420 ms (multiplication) | 13 | 22 |
| — Auxiliary Tape Memory (from Gemini VIII) | 14–15 | 23–24 |
| — Tape capacity: 1,170,000 bits; 6 min to load | 15 | 24 |
| **SOFTWARE** | 16–21 | 25–30 |
| — 16 instructions, assembly language only | 17 | 26 |
| — "Gemini Math Flow" — 9 versions; modularization origin story | 19–20 | 28–29 |
| — Three levels of simulation (CCTS, MVS) | 20–21 | 29–30 |
| **CREW INTERFACES TO THE GEMINI DIGITAL COMPUTER** | 21–24 | 30–33 |
| — Mode switch, MDIU (10-key keyboard + 7-digit register), IVI | 21–22 | 30–31 |
| — Catch-up/rendezvous procedures via MDIU address 83 | 23–24 | 32–33 |
| **THE IMPACT OF THE GEMINI DIGITAL COMPUTER** | 25–26 | 34–35 |
| — "Last of a dying breed" — list of 6 firsts (IBM silicon, core memory, auxiliary memory...) | 25 | 34 |

---

### Chapter 2 — Computers On Board the Apollo Spacecraft (Book pp. 27–64 · PDF pp. 36–73)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| The need for an on-board computer | 27–29 | 36–38 |
| — MIT chosen (August 9, 1961 contract), PGNCS, AGS | 28–31 | 37–40 |
| — 1.5-second signal delay to Moon as primary justification | 28 | 37 |
| MIT chosen as hardware and software contractor | 29–30 | 38–39 |
| **THE APOLLO COMPUTER SYSTEMS** (PGNCS + AGS) | 30–31 | 39–40 |
| **EVOLUTION OF THE HARDWARE: Block I and Block II** | 31–38 | 40–47 |
| — Block I (Polaris-derived) vs. Block II (integrated circuits) | 31 | 40 |
| — NOR gate logic, 4,100 ICs, 2,800 resistors | ~33 | ~42 |
| AGC specs: 15-bit word, 2.048 MHz, 38K total memory | ~34 | ~43 |
| **AGC SOFTWARE** | ~38–54 | ~47–63 |
| — LUMINARY (LM), COLOSSUS (CM) software versions | ~38 | ~47 |
| — Executive (OS), DSKY interface, Verb/Noun paradigm | ~40 | ~49 |
| **ABORT GUIDANCE SYSTEM (AGS)** | ~54–60 | ~63–69 |
| **PGNCS SOFTWARE IN OPERATION** | ~60–64 | ~69–73 |

> Note: CIS Chapter 2 covers Apollo computers from an external/historical perspective; for deep technical detail see AGC book.

---

### Chapter 3 — The Skylab Computer System (Book pp. 65–84 · PDF pp. 74–93)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Dual IBM 4Pi TC-1 computers for attitude control | 65–68 | 74–77 |
| — Apollo Telescope Mount (solar telescope pointing) | ~68 | ~77 |
| Hardware specs: 16-bit, 16K memory, 488 lbs total | ~70 | ~79 |
| Software: ATM, attitude control, systems management | ~72–78 | ~81–87 |
| Computer failures and workarounds during Skylab missions | ~78–84 | ~87–93 |

---

### Chapter 4 — Computers in the Space Shuttle Avionics System (Book pp. 85–134 · PDF pp. 94–143)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Overview: 5 general-purpose AP-101 computers (IBM) | 85–88 | 94–97 |
| — Redundancy: 4 primary + 1 backup (BFS) | ~87 | ~96 |
| Hardware: 16-bit word, 256K memory per CPU, 1.4 MIPS | ~89 | ~98 |
| **HAL/S language** (high-level, real-time) | ~95 | ~104 |
| Software: Primary Avionics Software System (PASS) | ~97–110 | ~106–119 |
| — Major Modes (MM) and Operational Sequences (OPS) | ~98 | ~107 |
| Integrated Multifunction CRT Display System (MEDS) | ~115 | ~124 |
| Software development and verification challenges | ~120–134 | ~129–143 |

---

## Part II — Computers On Board Unmanned Spacecraft (Book pp. 135–204 · PDF pp. 144–213)

**Introduction to Part Two** (Book p. 135 · PDF p. 144)

---

### Chapter 5 — From Sequencers to Computers: Exploring the Moon and the Inner Planets (Book pp. 139–170 · PDF pp. 148–179)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Programmable sequencers (command storage evolution) | 139–142 | 148–151 |
| Ranger spacecraft computers | ~143 | ~152 |
| Surveyor: first true on-board computer for unmanned lunar mission | ~148 | ~157 |
| Mariner series (Mars, Venus flybys) | ~152–160 | ~161–169 |
| — Mariner Mars 1969 flight program (see Appendix IV) | ~160 | ~169 |
| Viking (Mars landers): dual computer system | ~163–170 | ~172–179 |

---

### Chapter 6 — Distributed Computing On Board Voyager and Galileo (Book pp. 171–204 · PDF pp. 180–213)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Voyager: Computer Command Subsystem (CCS) | 171–180 | 180–189 |
| — Flight Data Subsystem (FDS) and Attitude & Articulation Control (AACS) | ~175 | ~184 |
| — Reprogramming Voyager in flight (1980 Saturn encounter) | ~182 | ~191 |
| Galileo spacecraft computers and distributed architecture | ~190–204 | ~199–213 |

---

## Part III — Ground Based Computers for Space Flight Operations (Book pp. 205–298 · PDF pp. 214–307)

**Introduction to Part Three** (Book p. 205 · PDF p. 214)

---

### Chapter 7 — The Evolution of Automated Launch Processing (Book pp. 207–240 · PDF pp. 216–249)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Manual checkout origins (Mercury/Gemini era) | 207–212 | 216–221 |
| Saturn V Launch Processing System (LPS) | ~213–225 | ~222–234 |
| — GOAL language for launch sequencing (see Appendix III) | ~220 | ~229 |
| Shuttle Launch Processing System (LPS/CLCS) | ~226–240 | ~235–249 |

---

### Chapter 8 — Computers in Mission Control (Book pp. 241–268 · PDF pp. 250–277)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Real-Time Computer Complex (RTCC) for Mercury/Gemini | 241–248 | 250–257 |
| — IBM 7090/7094 configuration | ~243 | ~252 |
| Apollo Mission Control RTCC upgrade | ~248–255 | ~257–264 |
| Shuttle Mission Control: distributed minicomputers | ~255–268 | ~264–277 |
| — Integrated Communications System | ~262 | ~271 |

---

### Chapter 9 — Making New Reality: Computers in Simulations and Image Processing (Book pp. 269–298 · PDF pp. 278–307)

| Sub-section | Book Page | PDF Page |
|---|---|---|
| Mission simulators (Gemini, Apollo, Shuttle) | 269–280 | 278–289 |
| Digital image processing origins (Ranger, Mariner) | ~280–290 | ~289–299 |
| — Jet Propulsion Laboratory image processing lab | ~283 | ~292 |
| Landsat and Earth observation image processing | ~290–298 | ~299–307 |

---

### Epilogue — Themes in NASA's Computing Experience (Book pp. 299–302 · PDF pp. 308–311)

Key themes: chronic underestimation of memory needs · increasing reliance on distributed/networked computers · software as a long-duration engineering product · NASA as technology nudger (not revolutionary driver)

---

### Back Matter (CIS)

| Section | Book Page | PDF Page |
|---|---|---|
| Source Notes | 303–362 | 312–371 |
| Bibliographic Note | 363–376 | 372–385 |
| Appendix I: Glossary of Computer Terms | 377–392 | 386–401 |
| Appendix II: HAL/S — A Real-Time Language for Space Flight | 393–398 | 402–407 |
| Appendix III: GOAL — A Language for Launch Processing | 399–402 | 408–411 |
| Appendix IV: Mariner Mars 1969 Flight Program | 403–409 | 412–418 |

---

---

## Cross-Document Topic Index

Quick reference for topics that appear in **both** documents.

| Topic | AGC Book | CIS |
|---|---|---|
| AGC hardware physical specs | Ch.1 p.16–18 | Ch.2 p.~33 |
| AGC memory: erasable + fixed | Ch.1 p.31–37, 59–68 | Ch.2 p.~34 |
| AGC instruction set | Ch.1 p.70–87; App.A p.369 | Ch.2 p.~37 |
| DSKY (Display & Keyboard) | Ch.2 p.123–140 | Ch.2 p.~40 |
| Executive (operating system) | Ch.2 p.99–160 | Ch.2 p.~40 |
| Interpreter language | Ch.2 p.160–197 | Ch.2 p.~44 |
| Inertial Measurement Unit (IMU) | Ch.3 p.199–209 | Ch.2 p.~50 |
| Gimbal lock | Ch.3 p.201 | Ch.2 p.~50 |
| Lunar landing programs (P63/P64/P66) | Ch.4 p.245–287 | Ch.2 p.~57 |
| Lunar orbit rendezvous | Ch.4 p.287–312 | Ch.2 p.~60 |
| Digital autopilot (DAP) | Ch.4 p.312–334 | Ch.2 p.~62 |
| Program alarms / 1202 error (Apollo 11) | Ch.4 p.358–363 | Ch.2 p.~63 |
| Gemini computer (IBM) | AGC Ch.0 p.3–4 | CIS Ch.1 p.9–26 |
| Apollo computer history/context | AGC Ch.0, Author's preface | CIS Ch.2 p.27–64 |
| Skylab computers | Not covered | CIS Ch.3 p.65–84 |
| Space Shuttle computers / HAL/S | AGC Ch.2 p.150 (reference) | CIS Ch.4 p.85–134 |
| Rendezvous techniques | Ch.4 p.287–312 | CIS Ch.1 p.23–24 |
| Software development challenges | AGC preface, Ch.0 | CIS Introduction, Ch.2 |
| Core rope memory | AGC Ch.1 p.35–37 | CIS Ch.2 p.~35 |
| Block I vs. Block II AGC | Not detailed (book covers Block II only) | CIS Ch.2 p.31–38 |
| Voyager / Galileo distributed computing | Not covered | CIS Ch.6 p.171–204 |
| Mission Control computers (RTCC) | Not covered | CIS Ch.8 p.241–268 |
| Launch processing automation | Not covered | CIS Ch.7 p.207–240 |
| Image processing / simulations | Not covered | CIS Ch.9 p.269–298 |

---

## Key Figures Quick Reference (AGC Book)

| Fig. | Description | Book Page | PDF Page |
|---|---|---|---|
| 1 | AGC and DSKY (photo) | 17 | 33 |
| 2 | AGC component locations (diagram) | 18 | 34 |
| 3 | Numeric representation in AGC word | 20 | 36 |
| 7 | AGC instruction format | 30 | 46 |
| 8 | Core memory | 35 | 51 |
| 9 | Schematic of core rope | 36 | 52 |
| 13 | Inertial Measurement Unit | 51 | 67 |
| 20 | View of both erasable and fixed storage | 64 | 80 |
| 21 | Instruction format including quarter code | 72 | 88 |
| 38 | Display and keyboard (DSKY photo) | 124 | 140 |
| 39 | Diagram of the DSKY (annotated) | 126 | 142 |
| 65 | Schematic of the IMU | 200 | 216 |
| 67 | Gimbal lock | 201 | 217 |
| 81 | Descent profile: braking, approach, landing | 245 | 261 |
| 82 | Lunar Module DSKY | 247 | 263 |

---

*Index compiled 2026-04-09 from full visual analysis of both PDFs (pages 1–40 read, full table of contents extracted).*
