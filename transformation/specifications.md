# Specification Tracking

## Core Infrastructure

| Spec | AGC Source (Comanche055) | Status | Notes |
|------|--------------------------|--------|-------|
| `types/` module | `ERASABLE_ASSIGNMENTS.agc` | Not started | CduAngle, Vec3, Mat3x3, Met, DeltaV |
| `AgcHardware` trait + sub-traits | `INTERRUPT_LEAD_INS.agc`, channel definitions | Not started | HAL boundary |
| `Executive` | `EXECUTIVE.agc` | Not started | Job table, 7 slots, NOVAC/FINDVAC |
| `Waitlist` | `WAITLIST.agc` | Not started | Delta-time chain, 8 slots |
| Restart protection | `FRESH_START_AND_RESTART.agc`, `PHASE_TABLE_MAINTENANCE.agc` | Not started | Phase tables, group management |
| Alarm system | `ALARM_AND_ABORT.agc` | Not started | 1202, 1210, 1211 |

## Navigation

| Spec | AGC Source | Status | Notes |
|------|------------|--------|-------|
| `math/linalg` | `INTERPRETER.agc` (VLOAD, DOT, CROSS, etc.) | Not started | Replaces interpretive vector ops |
| `math/trig` | `INTERPRETER.agc` (SINE, COSINE, ASIN) | Not started | f64 wrappers with AGC domain conventions |
| `math/kepler` | `CONIC_SUBROUTINES.agc` (KEPRTN) | Not started | Universal variable Kepler solver |
| `math/lambert` | `CONIC_SUBROUTINES.agc` | Not started | Lambert's problem |
| `navigation/state_vector` | `ERASABLE_ASSIGNMENTS.agc` | Not started | StateVector, coordinate frames |
| `navigation/integration` | `ORBITAL_INTEGRATION.agc` | Not started | Cowell / Encke propagation |
| `navigation/gravity` | `ORBITAL_INTEGRATION.agc` | Not started | Earth/Moon gravity, oblateness |
| `navigation/conics` | `CONIC_SUBROUTINES.agc` | Not started | Conic orbit determination |
| `services/average_g` | `SERVICER207.agc` | Not started | 2-second PIPA integration cycle |

## Guidance and Control

| Spec | AGC Source | Status | Notes |
|------|------------|--------|-------|
| `control/imu_control` | `IMU_MODE_SWITCHING_ROUTINES.agc`, `IMU_CALIBRATION_AND_ALIGNMENT.agc` | Not started | Typestate: Unaligned → Coarse → Fine |
| `control/dap` | `RCS-CSM_DIGITAL_AUTOPILOT.agc` | Not started | T5RUPT driven |
| `control/attitude` | `RCS-CSM_DIGITAL_AUTOPILOT.agc` | Not started | Rate damping, hold, maneuver |
| `control/rcs_logic` | `JET_SELECTION_LOGIC.agc` | Not started | 16-jet SM RCS, 12-jet CM RCS |
| `control/tvc` | `TVCEXECUTIVE.agc` | Not started | SPS gimbal steering |
| `guidance/targeting` | `P30.agc` | Not started | TIG computation |
| `guidance/maneuver` | `P40-P47.agc` | Not started | Delta-V, cross-product steering |
| `guidance/entry` | `REENTRY_CONTROL.agc`, `CM_ENTRY_DIGITAL_AUTOPILOT.agc` | Not started | Skip/ballistic targeting |

## Programs (P-codes)

| Spec | AGC Source | Status |
|------|------------|--------|
| P00 | `P00.agc` | Not started |
| P11 | `P11.agc` | Not started |
| P40/P41 | `P40-P47.agc` | Not started |
| P51/P52 | `P51-P53.agc` | Not started |
| P61–P67 | `P61-P67.agc` | Not started |
| P30 | `P30.agc` | Not started |
| P37 | `P37_P70.agc` | Not started |
| P20–P23 | `P20-P25.agc` | Not started |

## DSKY / Crew Interface

| Spec | AGC Source | Status |
|------|------------|--------|
| `services/v_n` (PINBALL) | `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` | Not started |
| `services/display` | `DISPLAY_INTERFACE_ROUTINES.agc`, `PINBALL_NOUN_TABLES.agc` | Not started |
| `hal/dsky` | `T4RUPT_PROGRAM.agc` (relay matrix timing) | Not started |

## Status Legend

| Status | Meaning |
|--------|---------|
| Not started | Spec file not yet created |
| Spec draft | Spec being written |
| Spec approved | Ready for implementation |
| In progress | Code being written |
| Complete | Implemented, tested, reviewed |
