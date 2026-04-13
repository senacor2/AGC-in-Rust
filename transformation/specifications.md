# Specification Tracking

## Core Infrastructure

| Spec | AGC Source (Comanche055) | Status | Notes |
|------|--------------------------|--------|-------|
| `types/` module | `ERASABLE_ASSIGNMENTS.agc` | Complete | CduAngle, Vec3, Mat3x3, Met, DeltaV |
| `AgcHardware` trait + sub-traits | `INTERRUPT_LEAD_INS.agc`, channel definitions | Complete | HAL boundary; SimHardware in agc-sim |
| `Executive` | `EXECUTIVE.agc` | Complete | Job table, 7 slots, NOVAC/FINDVAC |
| `Waitlist` | `WAITLIST.agc` | Complete | Delta-time chain, 8 slots |
| Restart protection | `FRESH_START_AND_RESTART.agc`, `PHASE_TABLE_MAINTENANCE.agc` | Complete | Phase tables, group management |
| Alarm system | `ALARM_AND_ABORT.agc` | Complete | 1202, 1210, 1211; PipaOverflow (0o205) added for M2 |
| Fresh Start / Restart | `FRESH_START_AND_RESTART.agc` | Complete | SLAP1, DOFSTART, GOPROG, STARTSB2, MR.KLEAN |

## Navigation

| Spec | AGC Source | Status | Notes |
|------|------------|--------|-------|
| `math/linalg` | `INTERPRETER.agc` (VLOAD, DOT, CROSS, etc.) | Complete | dot, cross, norm, unit, scale, add, sub, mxv, mxm, transpose, rotx/y/z |
| `math/trig` | `INTERPRETER.agc` (SINE, COSINE, ASIN) | Complete | sin, cos, tan, asin_clamped, acos_clamped, atan2 |
| `math/kepler` | `CONIC_SUBROUTINES.agc` (KEPRTN) | Complete | Universal-variable Kepler solver; Stumpff C/S functions; integration tests in kepler_lambert_roundtrip.rs |
| `math/lambert` | `CONIC_SUBROUTINES.agc` | Complete | BMW universal-variable bisection Lambert solver; short/long transfer; integration tests in kepler_lambert_roundtrip.rs |
| `navigation/state_vector` | `ERASABLE_ASSIGNMENTS.agc` | Complete | StateVector (r, v, t, gdt/2), PrimaryBody |
| `navigation/integration` | `ORBITAL_INTEGRATION.agc` | Complete | RK4 propagator (M2); TODO(M3): Nystrom predictor-corrector |
| `navigation/gravity` | `ORBITAL_INTEGRATION.agc` | Complete | Earth (point-mass + J2), Moon (point-mass), total_gravity, SOI check |
| `navigation/conics` | `CONIC_SUBROUTINES.agc` | Not started | Conic orbit determination |
| `services/average_g` | `SERVICER207.agc` | Complete | CALCRVG trapezoidal predictor-corrector, PIPA saturation alarm |

## Guidance and Control

| Spec | AGC Source | Status | Notes |
|------|------------|--------|-------|
| `control/imu_control` | `IMU_MODE_SWITCHING_ROUTINES.agc`, `IMU_CALIBRATION_AND_ALIGNMENT.agc` | Complete | Typestate: Unaligned → Coarse → Fine; AXISGEN/CALCGTA torquing angles; integration tests in p51_alignment_scenario.rs |
| `control/dap` | `RCS-CSM_DIGITAL_AUTOPILOT.agc` | Complete | T5RUPT driven; Idle/Rcs mode state machine; integration tests in dap_attitude_scenario.rs |
| `control/attitude` | `RCS-CSM_DIGITAL_AUTOPILOT.agc` | Complete | Phase-plane switching; compute_error; SLOPE=0.24; integration tests in dap_attitude_scenario.rs |
| `control/rcs_logic` | `JET_SELECTION_LOGIC.agc` | Complete | PYTABLE/RTABLE 16-jet lookup; select_jets; min_impulse 14 ms |
| `control/tvc` | `TVCEXECUTIVE.agc` | Complete | Simplified PD TVC law; ±6° saturation; TvcGains |
| `guidance/targeting` | `P30.agc` | Complete | burn_duration (Tsiolkovsky); predict_vg_at_ignition (LOMAT); integration tests in burn_targeting_scenario.rs |
| `guidance/maneuver` | `P40-P47.agc` | Complete | ManeuverState VG tracking; S40.8 cutoff; steering reversal; integration tests in maneuver_crossproduct_steering.rs |
| `guidance/entry` | `REENTRY_CONTROL.agc`, `CM_ENTRY_DIGITAL_AUTOPILOT.agc` | Not started | Skip/ballistic targeting |

## Programs (P-codes)

| Spec | AGC Source | Status |
|------|------------|--------|
| P00 | `P00.agc` | Complete | Idle loop; DSKY blanking; program/noun/verb display |
| P11 | `P11.agc` | Complete | Earth Orbit Insertion monitor; CDU angle display; 2-s tick rate |
| P40/P41 | `P40-P47.agc` | Complete | 6-phase SPS/RCS burn state machine; TIG/ullage/cutoff/trim; integration tests in p40_burn_end_to_end.rs |
| P51/P52 | `P51-P53.agc` | Complete | IMU alignment; AXISGEN star-pair geometry; CALCGTA torquing; integration tests in p51_alignment_scenario.rs |
| P61–P67 | `P61-P67.agc` | Complete | Entry guidance 7-phase state machine; 0.05G/0.2G/upcontrol/ballistic/final; integration tests in p61_entry_sequence.rs |
| P30 | `P30.agc` | Complete | External delta-V burn targeting; DSKY N81 display; integration tests in burn_targeting_scenario.rs |
| P37 | `P37_P70.agc` | Complete | Return-to-Earth Lambert targeting; TEI delta-V; entry interface constraint |
| P20–P23 | `P20-P25.agc` | Not started |

## DSKY / Crew Interface

| Spec | AGC Source | Status | Notes |
|------|------------|--------|-------|
| `services/v_n` (PINBALL) | `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` | Complete | VnState; InputMode; CharResult; char_in/clr_press/key_rel_press/request_data_entry; 9 unit tests |
| `services/display` | `DISPLAY_INTERFACE_ROUTINES.agc`, `PINBALL_NOUN_TABLES.agc` | Complete | RELAY_CODES; format_decimal/octal/time/min_sec; BLANK=10 sentinel; 10 unit tests |
| `services/noun_table` | `PINBALL_NOUN_TABLES.agc` (NNADTAB) | Complete | 11 nouns (N00/N11/N30/N33/N36/N40/N43/N44/N62/N82/N85); DataSource/DisplayFormat/NounDef; lookup(); 6 unit tests |
| `services/pinball` | `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` (VERBFAN) | Complete | VerbFn type; VERB_TABLE/EXTENDED_VERB_TABLE; dispatch<H>; V01/V06/V11/V16/V21/V24/V25/V27/V34/V35/V37/V82; 8 unit tests |
| `hal/dsky` | `T4RUPT_PROGRAM.agc` (relay matrix timing) | Complete | DskyIo trait; lamp_test() default method added (M5) |
| agc-sim TUI wiring | — | Complete | VnState routing; 3 scenarios (launch/burn/free); F1-F3 live switch; =/_ TMX; headless-safe stdin check |
| Integration tests (M5) | — | Complete | pinball_verb_dispatch.rs (6 tests); full_mission_sequence.rs (3 tests); P00→P11→P30→P40→P37→P40→P61 |

## Status Legend

| Status | Meaning |
|--------|---------|
| Not started | Spec file not yet created |
| Spec draft | Spec being written |
| Spec approved | Ready for implementation |
| In progress | Code being written |
| Complete | Implemented, tested, reviewed |
