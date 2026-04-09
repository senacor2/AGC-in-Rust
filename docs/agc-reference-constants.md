# AGC Reference Constants and Algorithms

Machine-readable reference extracted from the Comanche055 AGC assembler source
and Frank O'Brien's *The Apollo Guidance Computer* (AGC book). Used by the
validation skill to verify Rust implementations without re-fetching sources.

Last validated: 2026-04-09.

---

## Source File Map

Each Rust module maps to one or more AGC assembler files.

| Rust module | AGC source file(s) | Local path |
|---|---|---|
| `services/average_g` | SERVICER207.agc | `docs/agc-source/SERVICER207.agc` |
| `navigation/gravity` | ORBITAL_INTEGRATION.agc | `docs/agc-source/ORBITAL_INTEGRATION.agc` |
| `navigation/integration` | ORBITAL_INTEGRATION.agc | `docs/agc-source/ORBITAL_INTEGRATION.agc` |
| `navigation/state_vector` | ERASABLE_ASSIGNMENTS.agc | `docs/agc-source/ERASABLE_ASSIGNMENTS.agc` |
| `executive/scheduler` | EXECUTIVE.agc | `docs/agc-source/EXECUTIVE.agc` |
| `executive/waitlist` | WAITLIST.agc | `docs/agc-source/WAITLIST.agc` |
| `executive/restart` | FRESH_START_AND_RESTART.agc, RESTART_TABLES.agc | `docs/agc-source/FRESH_START_AND_RESTART.agc` |
| `services/alarm` | ALARM_AND_ABORT.agc | `docs/agc-source/ALARM_AND_ABORT.agc` |
| `hal/dsky` | PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, T4RUPT_PROGRAM.agc | `docs/agc-source/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` |

---

## Physical Constants

All values sourced from ORBITAL_INTEGRATION.agc and LATITUDE_LONGITUDE_SUBROUTINES.agc.
"AGC value" means the constant as encoded in the flight software (may differ from
modern IAU/WGS84 values; the AGC value takes precedence for fidelity).

| Constant | Rust name | Value | Unit | AGC source line |
|---|---|---|---|---|
| Earth GM | `MU_EARTH` | `3.986_032e14` | m^3/s^2 | ORBITAL_INTEGRATION.agc: `MUEARTH 3.986032 E10 B-36` (scaled) |
| Moon GM | `MU_MOON` | `4.902_778e12` | m^3/s^2 | ORBITAL_INTEGRATION.agc |
| Earth equatorial radius | `RE_EARTH` | `6_373_338.0` | m | ORBITAL_INTEGRATION.agc: `RSPHERE` ~6373.338 km |
| Earth pad radius (altitude ref) | `R_EARTH_M` (nav_demo) | `6_373_338.0` | m | LATITUDE_LONGITUDE_SUBROUTINES.agc: `ERAD 2DEC 6373338 B-29 # PAD RADIUS` |
| Earth J2 coefficient | `J2_EARTH` | `1.082_626_68e-3` | dimensionless | ORBITAL_INTEGRATION.agc: J2 term coefficient |
| PIPA scale factor | `PIPA_SCALE` | `0.0585` | m/s per count | SERVICER207.agc: `KPIP1 2DEC 0.074880 # 1 PULSE = 5.85 CM/SEC` |
| SERVICER cycle period | `CYCLE_DT` | `2.0` | seconds | SERVICER207.agc: `CAF 2SECS; TC WAITLIST; 2CADR READACCS` |

---

## Scheduler Constants

| Constant | Rust name | Value | AGC source |
|---|---|---|---|
| CORE SET job table size | `MAX_JOBS` | 7 | EXECUTIVE.agc: 7-slot CORE SET table |
| Waitlist task capacity | `MAX_WAITLIST_TASKS` | 9 | WAITLIST.agc: "9 TASKS MAXIMUM"; `LST2 ERASE +17D` (9 two-word 2CADR pairs) |
| Restart groups | `NUM_RESTART_GROUPS` | 5 | FRESH_START_AND_RESTART.agc: `NUMGRPS EQUALS FIVE` |

---

## Alarm Codes

From ALARM_AND_ABORT.agc:

| Code | Rust variant | Severity | Description |
|---|---|---|---|
| 01202 | `ExecutiveOverflow` | Recoverable | Executive overflow: no empty VAC area (NOVAC table full) |
| 01210 | `WaitlistOverflow` | Recoverable | Waitlist overflow: no empty task slot |
| 01211 | `ErasableChecksum` | Fatal | Erasable memory checksum failure |

---

## DSKY Key Codes

5-bit keyboard codes from PINBALL_GAME_BUTTONS_AND_LIGHTS.agc octal table:

| Key | Octal | Hex | Rust variant | Status |
|---|---|---|---|---|
| VERB | 21 | 0x11 | `Verb` | Confirmed |
| NOUN | 37 | 0x1F | `Noun` | Confirmed |
| ENTR | 34 | 0x1C | `Enter` | Confirmed |
| CLR | 36 | 0x1E | `Clear` | Confirmed |
| RSET | 22 | 0x12 | `Reset` | Confirmed |
| KEY REL | 31 | 0x19 | `KeyRel` | Confirmed |
| PRO | 24 | 0x14 | `ProceED` | Unverified in PINBALL source |
| + | 32 | 0x1A | `Plus` | Confirmed |
| - | 33 | 0x1B | `Minus` | Confirmed |
| 0 | 20 | 0x10 | `Zero` | Confirmed |
| 1 | 01 | 0x01 | `One` | Confirmed |
| 2 | 02 | 0x02 | `Two` | Confirmed |
| 3 | 03 | 0x03 | `Three` | Confirmed |
| 4 | 04 | 0x04 | `Four` | Confirmed |
| 5 | 05 | 0x05 | `Five` | Confirmed |
| 6 | 06 | 0x06 | `Six` | Confirmed |
| 7 | 07 | 0x07 | `Seven` | Confirmed |
| 8 | 10 | 0x08 | `Eight` | Confirmed |
| 9 | 11 | 0x09 | `Nine` | Confirmed |

---

## Algorithms

### AVERAGE G / SERVICER (SERVICER207.agc)

Predictor-corrector (Störmer-Verlet / leapfrog) integration:

```
Input: RN, VN (current state), PIPA counts, REFSMMAT, GDT/2_old (from previous cycle)

1. dv_sm = PIPA * KPIP1                          # stable-member delta-V
2. dv_eci = REFSMMAT^T * dv_sm                   # rotate to ECI
3. RN1 = RN + (VN + dv/2 + GDT/2_old) * dt       # position PREDICTOR (CALCRVG)
4. a_new = point_mass(RN1, MU_EARTH)              # gravity at predicted position (CALCGRAV)
5. VN1 = VN + dv + a_new * dt                     # velocity CORRECTOR
6. GDT/2_new = a_new * dt/2                       # save for next cycle

Output: RN1, VN1, GDT/2_new
```

Key: position uses gravity from the **previous** cycle (`GDT/2_old`); velocity uses
**freshly computed** gravity at the predicted position. First cycle is warm-up
(GDT/2_old = 0).

### Point-mass gravity

```
a = -(mu / |r|^3) * r
```

Singularity guard: if |r| < 1.0 m, return zero vector.

### J2 perturbation

Standard ECI J2 formula using `J2_EARTH`, `MU_EARTH`, `RE_EARTH`. Higher-order
terms (J3, J4) are defined in ORBITAL_INTEGRATION.agc but out of scope for
Milestone 2.

### RK4 integrator (Rust substitution)

The AGC used a second-order Nystrom predictor-corrector in ORBITAL_INTEGRATION.agc.
Rust substitutes a 4th-order Runge-Kutta (RK4) for orbital propagation (not SERVICER).
This achieves same-order truncation error with a cleaner implementation.

---

## AGC Book References (PDF unreadable by tools)

The PDF *The Apollo Guidance Computer* uses a non-standard font encoding that
produces garbled text when extracted programmatically. Key values have been
manually verified and recorded above. For future reference:

| Topic | Book page | PDF page (book + 16) |
|---|---|---|
| PIPA counters, CDU interface | 51-55 | 67-71 |
| Guidance and navigation fundamentals | 199-229 | 215-245 |
| IMU schematic, body axes | 200-201 | 216-217 |
| DSKY layout and operation | 123-140 | 139-156 |
| Executive / Waitlist / Interpreter | 99-197 | 115-213 |
| Restart / phase tables | 118-120 | 134-136 |
| Program alarms (1202 Apollo 11) | 358-363 | 374-379 |
| DAP phase plane | 312-334 | 328-350 |

Full index: `spaceflight_context_index.md`
