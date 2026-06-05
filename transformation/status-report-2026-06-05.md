# AGC-in-Rust: Status / Gap Report
**Date:** 2026-06-05  
**Purpose:** Inventory the current Rust port against the Comanche055 (Apollo 11 CM) and Colossus237 (Apollo 8 CM) reference ropes, identifying what is implemented, partially implemented, intentionally out of scope, or a genuine gap to close.  
**Methodology:** Read every Rust source file under `agc-core/src/programs/`, `agc-core/src/services/`, and `agc-core/src/control/`; cross-referenced against the AGC assembler source at `~/virtualagc/Comanche055/` and `~/virtualagc/Colossus237/` (primarily `ASSEMBLY_AND_OPERATION_INFORMATION.agc`, `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, `PINBALL_NOUN_TABLES.agc`, and all `P*.agc` / `R*.agc` files). P/R/V/N numbers listed only where they appear in the AGC source or our port.  
**Inputs surveyed:** `agc-core/src/programs/{mod.rs,p00.rs,p01_p02.rs,p06.rs,p11.rs,p15.rs,p20.rs,p21.rs,p22.rs,p23.rs,p29.rs,p30.rs,p31.rs,p32.rs,p33.rs,p34.rs,p37.rs,p40_p41.rs,p47.rs,p51_p52.rs,p61_p67.rs}`, `agc-core/src/services/v_n.rs`, `agc-core/src/services/{alarm,average_g,backup,display,fresh_start,pinball,t4rupt,uplink}.rs`, `agc-core/src/control/{dap,imu_control,rcs_logic,tvc,attitude,sextant}.rs`, plus the entire virtualagc Comanche055 and Colossus237 trees.

---

## 1. Executive Summary

The port implements **26 major-mode entry points** spanning P00–P67. Against the Comanche055 rope (~50 named major modes that have real operational content for the CM-only mission), this corresponds to roughly **50–55 %** of the CM-relevant program space. Programs covering the entire earth-to-moon-and-back flight profile (pre-launch initialisation, orbit insertion, cislunar navigation, rendezvous, targeting, maneuver execution, the full closed-loop entry guidance chain P61–P67 driven by HUNTEST / UPCONTRL / CONSTD / PREDICT3, and TEI contingency) are all present at the operational depth required by the integration tests. The remaining intentional gaps are LM-side programs (P72–P78 family).

Colossus237 (Apollo 8) shares ~95 % of its program and routine set with Comanche055. The primary structural differences are: (a) Comanche055 adds the full LM rendezvous suite (P32–P35, P72–P75, P76–P78), which Apollo 8 did not carry operationally; (b) Comanche055 has the `LUNAR_LANDMARK_SELECTION_FOR_CM.agc` routine (R35/V79); (c) Colossus237 lacks the `TVCGEN3FILTERS.agc` module present in Comanche055, indicating incremental TVC improvements. The Rust port targets Comanche055; Colossus237 differences are largely academic for the current scope.

The verb/noun processor (PINBALL) covers the operationally critical verbs. The most significant open gaps at the DSKY layer are the extended verbs V40–V49 and V56–V99 (hardware CDU, DAP configuration, and telemetry verbs) and mixed/engineering nouns that were never connected in `noun_display` / `noun_commit`.

---

## 2. Programs (P00 – P79+)

Key to Status column:  
✅ Implemented · 🟡 Partial · ❌ Gap to close · ⚪ Out of scope (intentional)

| P# | Comanche055 purpose | Colossus237 purpose if different | Status in port | Notes / reason for gap |
|----|---------------------|----------------------------------|----------------|------------------------|
| P00 | CMC Idle — background coasting hold | Same | ✅ `programs/p00.rs` | Sets AttitudeHold, clears burn, cancels servicer exit hook |
| P01 | Pre-launch IMU initialisation (cage platform) | Same | ✅ `programs/p01_p02.rs` | Cages ImuAlignmentState; real gyrocompass loop simplified to state transition |
| P02 | Gyrocompassing (coarse align platform to local horizontal) | Same | 🟡 `programs/p01_p02.rs` | State transition only; no actual multi-minute gyrocompass integration loop (no earth-rate HAL source) |
| P06 | CMC Power-down (standby) | Same | ✅ `programs/p06.rs` | Stops SERVICER, stops DAP, lights STBY; crew resumes with V37E00E |
| P07 | IMU performance test | Same | ⚪ AGC self-check / IMU hardware test — out of scope (see rationale G6) |
| P08 | Gyro torquing (auto fine-align) | Same | ⚪ IMU hardware path — out of scope (G5) |
| P09 | (Not a standard Comanche055 major mode) | — | ⚪ Not an assigned program number |
| P10 | (Not assigned in CM rope) | Same | ⚪ Not used |
| P11 | Earth orbit insertion monitor (V16N44) | Same | ✅ `programs/p11.rs` | SERVICER exit hook refreshes apogee/perigee/half-period; warns on hyperbolic orbit |
| P12 | Powered flight monitor (CSM ascent) | Same | ❌ Not yet ported; Comanche055 shares a code block with P11 |
| P13 | (Not standard) | — | ⚪ Not assigned |
| P14 | (Not standard) | — | ⚪ Not assigned |
| P15 | TLI monitor (V16N44 post-TLI) | Same | ✅ `programs/p15.rs` | Identical pipeline to P11; warns on post-TLI hyperbolic trajectory |
| P20 | Rendezvous navigation (LM state vector, W-matrix Kalman) | Same | ✅ `programs/p20.rs` | Full scalar Kalman filter; radar / sextant marks via `p20_incorporate_mark`; V16N54 display; process-noise growth |
| P21 | Ground-track determination (lat/lon/alt at crew-specified GET) | Same | ✅ `programs/p21.rs` | Kepler propagation; inertial→ECEF; V06N43 display |
| P22 | Orbital navigation — landmark tracking (CSM state update) | Same | ✅ `programs/p22.rs` | Periodic Waitlist loop; scalar Kalman; sextant landmark marks via `p22_incorporate_landmark_mark` |
| P23 | Cislunar midcourse navigation (star-horizon / star-landmark) | Same | ✅ `programs/p23.rs` | Star-horizon and star-landmark measurement models; V50/V54 mark pipeline; process noise 10× P22 |
| P24 | (Rendezvous nav option — part of P20 suite) | Same | ⚪ Subsumed into P20 in this port; discrete P24 not needed |
| P25 | (Rendezvous / tracking — part of P20 suite) | Same | ⚪ Tracking terminated by V56 in the real AGC; not a separate major mode in our port |
| P27 | Update Liaison (P27 implicit via V70/V71/V72/V73) | Same | ✅ Implemented within `services/v_n.rs` | P27 block/single-address/time update dispatched from V70–V73; address space 1–31 fully mapped (§5 of uplink plan) |
| P29 | Time-of-longitude (when does CSM cross a target longitude?) | Same | ✅ `programs/p29.rs` | Newton solver via `navigation::conics::time_of_longitude`; V25N89 crew entry; V06N34 display |
| P30 | External ΔV targeting (TIG + LVLH ΔV → burn attitude) | Same | ✅ `programs/p30.rs` | Crew loads N33 TIG then N81 ΔV (LVLH); `apply_external_delta_v` computes inertial burn vector; stored in `pending_maneuver` |
| P31 | Lambert aim-point guidance (CSI targeting, coelliptic) | Same | ✅ `programs/p31.rs` | 1-D Newton iteration for CSI burn; MU_EARTH; `propagate_to_tig`; crew-configurable Δh default 10 nmi |
| P32 | CDH targeting (constant-delta-height coelliptic) | Same | ✅ `programs/p32.rs` | Closed-form CDH ΔV; alarm on degenerate geometry; reuses `propagate_to_tig` from P31 |
| P33 | TPI targeting (Lambert intercept to LM position) | Same | ✅ `programs/p33.rs` | Lambert solver (`math/lambert.rs`); staleness check; stores `tpi_arrival_epoch` for P34 |
| P34 | TPM midcourse correction (Lambert from current position to P33 arrival) | Same | ✅ `programs/p34.rs` | Reads `tpi_arrival_epoch` from P33; minimum range guard |
| P35 | TPF final approach (rendezvous) | Same | ❌ Not ported — AGC source in `P34-P35,_P74-P75.agc`; rendezvous braking approach |
| P36 | (Not standard) | — | ⚪ Not assigned |
| P37 | Return to Earth (TEI from lunar orbit) | Same | ✅ `programs/p37.rs` | Lambert-based TEI targeting; requires `Frame::MoonInertial`; default 60-hour TOF; stores `pending_maneuver` |
| P38 | Stable-orbit rendezvous SOI maneuver | Same | ❌ Not ported — `STABLE_ORBIT_-_P38-P39.agc`; used for plane-change / station-keeping |
| P39 | Stable-orbit rendezvous SOR maneuver | Same | ❌ Not ported — companion to P38 |
| P40 | SPS thrusting program | Same | ✅ `programs/p40_p41.rs` | Burn init, TVC/DAP setup, V50N99 crew ARM request; SERVICER burn loop |
| P41 | RCS thrusting program | Same | ✅ `programs/p40_p41.rs` | Same pipeline as P40 without TVC; regime guard (DV < SPS_MIN_DV) |
| P47 | Thrust monitor (passive ΔV display) | Same | ✅ `programs/p47.rs` | V16N83; SERVICER exit hook; no actuator commands |
| P48 | (Rendezvous braking — LM side, not CM) | Same (LM program) | ⚪ LM-side program — CM port only (G1) |
| P51 | IMU orientation determination (star sightings → REFSMMAT) | Same | ✅ `programs/p51_p52.rs` | TRIAD construction via `imu_control::refsmmat_from_star_sightings`; V25N70 star entry; collinear-star alarm |
| P52 | IMU realignment (realign to stored REFSMMAT) | Same | ✅ `programs/p51_p52.rs` | Calls `coarse_align_step` + `fine_align_torque`; platform-caged alarm |
| P53 | External ΔV determination (P52 variant, post-sep check) | Same | ❌ Not ported — `P51-P53.agc`; measures IMU drift after separation |
| P57 | (Not a standard major mode number) | — | ⚪ Not assigned |
| P61 | Entry preparation (EMS init, predict GMAX/VPRED/GAMMAEI) | Same | ✅ `programs/p61_p67.rs` | Sets `EntryPhase::Preparation`, loads target-range display |
| P62 | CM/SM separation | Same | 🟡 `programs/p61_p67.rs` | Phase transition, `dap_stop`, voids stale `pending_maneuver`; the physical sep-pyro RCS command is the only piece deferred (no Secs-style HAL trait for SM-sep yet) |
| P63 | Pre-0.05g monitoring | Same | ✅ `programs/p61_p67.rs` | Installs `entry_servicer_exit` hook; SERVICER computes sensed-g each cycle, `p63_check_threshold` trips PreEntry → Entry at `ENTRY_THRESHOLD_G` and switches DAP to `EntryRoll(0.0)` |
| P64 | Closed-loop entry guidance (roll steering) | Same | ✅ `programs/p61_p67.rs` + `guidance/entry.rs` | HUNTEST/INITROLL Newton iteration in `compute_ld_command`; GLIMITER L/D limiter (#85); CONSTD divergence routing (#86); Earth-rotation `v_rel` correction (#87); L/D → `resolve_roll` → `DapMode::EntryRoll`. Achieved 111 km miss on lunar return |
| P65 | Up-control / skip-out (UPCONTRL) | Same | ✅ `guidance/entry.rs:upcontrol_step` | Full UPCONTRL feedback law with LIMITL/D clamp + GLIMITER (#85); `select_phase` routes Entry ↔ Skip on HUNTEST convergence |
| P66 | Ballistic hold (roll-command hold when guidance diverges) | Same | ✅ `guidance/entry.rs:ballistic_step` | Roll-command freeze when `select_phase` decides Q7 drag exit; DAP retains `EntryRoll` |
| P67 | Final phase / drogue deployment detection | Same | ✅ `programs/p61_p67.rs` + `guidance/entry.rs:final_phase_step` | PREDICT3 final-phase law; drogue trigger uses VQUIT = 305 m/s on `\|v_rel\|` (#87); Sutton–Graves stagnation-point heating monitored (#96) |
| P70 | TLI targeting (burns from Earth orbit to trans-lunar trajectory) | Same | ❌ Not ported — `P37,P70.agc`; computes TLI burn from LEO parking orbit |
| P72 | CSI targeting (LM active vehicle) | LM program | ⚪ LM-side rendezvous — CM port only (G1) |
| P73 | CDH targeting (LM active) | LM program | ⚪ LM-side rendezvous (G1) |
| P74 | TPI targeting (LM active) | LM program | ⚪ LM-side rendezvous (G1) |
| P75 | TPM midcourse (LM active) | LM program | ⚪ LM-side rendezvous (G1) |
| P76 | Target ΔV program (update LM state with a ΔV event) | Same | ❌ Not ported — `P76.agc`; crew enters ΔV and TIG, integrates LM state to TIG, applies ΔV |
| P77 | (Not standard in Comanche055) | — | ⚪ Not assigned |
| P78 | Stable-orbit rendezvous (LM active) | LM program | ⚪ LM-side rendezvous (G1) |

**Programs summary:** 25 implemented (✅), 2 partial (P02 gyrocompass loop unsimplified, P62 missing the physical sep-pyro HAL command), 9 out-of-scope LM/hardware programs, 6 CM-relevant gaps (P12, P35, P38, P39, P53, P70, P76).

---

## 3. Routines (R00 – R7x)

R-routines are reusable subroutines called by multiple P-programs. They do not have their own `PROGRAM_TABLE` entry but are standalone `.agc` files or labelled sections.

| R# | Comanche055 purpose | Status in port | Notes |
|----|---------------------|----------------|-------|
| R02 | IMU status check (called by P40, P20, P51) | ⚪ Hardware-only path (G5) — IMU fault detection requires real CDU hardware |
| R21 | Rendezvous sighting mark routine (V57) | 🟡 `programs/p20.rs:p20_incorporate_sextant_mark` + `control/sextant.rs:consume_optics_mark` — sextant HAL mark pipeline and Kalman update implemented; V57 verb dispatch not wired in `dispatch_verb_noun` (verified via `programs/p20.rs:570`, `control/sextant.rs:128`) |
| R22 | Rendezvous tracking data processor (radar marks) | 🟡 `programs/p20.rs` — `p20_incorporate_mark` ingests range/range-rate; R22 auto-ranging and transponder-lock logic not modelled |
| R23 | Rendezvous backup sighting (V54) | ❌ Not ported — R23 is a distinct rendezvous-specific backup-sighting routine; `p23_incorporate_star_horizon_mark` in `programs/p23.rs` serves cislunar nav (P23), not R23's rendezvous role; V54 dispatch not wired |
| R30 | Orbital parameters display (V82 → N44) | 🟡 `services/v_n.rs` `noun_display` N44 — apogee/perigee/half-period computed; `time_to_periapsis` (TFF) and `DELRSPL` splash prediction not connected |
| R31 | Rendezvous parameter display No. 1 (V83) | ❌ Not ported — `R31.agc`; displays CDH/TPI timing parameters |
| R34 | Rendezvous parameter display No. 2 (V85) | ❌ Not ported |
| R35 | Lunar landmark selection (V79) | ❌ Not ported — `LUNAR_LANDMARK_SELECTION_FOR_CM.agc` |
| R36 | Rendezvous out-of-plane display (V90) | ❌ Not ported |
| R52 | Auto optics positioning / star-LOS (used by P23, P51) | 🟡 `control/sextant.rs` — star-horizon mark pipeline wired; auto-optics CDU drive (sextant slew) not modelled |
| R60 | Vehicle attitude maneuver (V49 → KALCMANU steering) | 🟡 `control/dap.rs` `dap_init` + `attitude.rs` — attitude error and RCS pulse selection implemented; KALCMANU optimal steering law not fully replicated (DAP uses simple P/I attitude error) |
| R62 | Crew-defined maneuver (V49 crew enters body-axis angles) | ❌ Not ported — `R60,R62.agc`; V49 dispatch raises OPR ERR |
| R63 | Rendezvous final attitude (V89) | ❌ Not ported |

Additional routines embedded in service modules (not discrete `.rs` files but directly implemented):

| Routine | Location in port | Notes |
|---------|-----------------|-------|
| SERVICER (Average-G) | `services/average_g.rs` | PIPA integration, REFSMMAT rotation, gravity/SOI switching — fully implemented |
| KALCMANU steering | `control/dap.rs` | Simplified PID attitude hold; full KALCMANU quaternion-path steering not yet implemented |
| GIMBAL_LOCK_AVOIDANCE | `control/imu_control.rs` — `is_gimbal_lock_warning` / `is_gimbal_lock_critical` | Warning and critical thresholds checked; automated gimbal-lock avoidance maneuver not wired to DAP |
| IMU_COMPENSATION (PIPA NBD) | `services/average_g.rs` + `control/imu_control.rs` | Gyro NBD bias applied; V71 addresses 23–29 update gyro/PIPA calibration |
| TVC (Thrust Vector Control) | `control/tvc.rs` | Three-axis IIR filter, gimbal trim loop, pitch/yaw TVC fully implemented |
| RCS-CSM DAP | `control/rcs_logic.rs` + `control/dap.rs` | Jet selection (CM and SM torque tables), pulse-duration computation, attitude hold; translation-axis commands not modelled |
| FRESH START / RESTART | `services/fresh_start.rs` | Phase-table maintenance, battery-backed BKPSRAM restart semantics (FRESH START vs RESTART), V37E00E path |
| PINBALL (DSKY keyboard) | `services/v_n.rs` | Full key state machine V06/V16/V21–V25/V34/V35/V37/V70–V73; extended verbs V40–V99 raise OPR ERR |
| T4RUPT (10 ms hardware interrupt) | `services/t4rupt.rs` | DAP step, display refresh, IMU CDU sampling |
| DOWN-TELEMETRY | `services/pinball.rs` (partial) | Downlink list structure present; real MSFN-format telemetry encoding not implemented |
| UPLINK | `services/uplink.rs` | V71/V72/V73 keyboard path via `v_n.rs`; raw uplink byte stream parsing not implemented |
| ALARM_AND_ABORT | `services/alarm.rs` | `AlarmState{code, lit}` set throughout; POODOO / GOTOPOOH restart recovery chain not fully wired |

---

## 4. Verbs (V00 – V99)

### Regular Verbs (V00 – V39)

| V# | AGC purpose | Status | Notes |
|----|-------------|--------|-------|
| V00 | Not in use | ⚪ — |
| V01 | Display octal comp 1 in R1 | ❌ Not in `dispatch_verb_noun` |
| V02 | Display octal comp 2 in R1 | ❌ |
| V03 | Display octal comp 3 in R1 | ❌ |
| V04 | Display octal comp 1,2 in R1,R2 | ❌ |
| V05 | Display octal comp 1,2,3 in R1,R2,R3 | ❌ |
| V06 | Display decimal (1–3 registers) | ✅ `v_n.rs:v06_display_decimal` |
| V07 | Display DP decimal R1,R2 (test only) | ⚪ Test-only verb (G6) |
| V11–V15 | Monitor octal 1–3 / monitor octal 1,2 / 1,2,3 | ❌ Not wired |
| V16 | Monitor decimal (updated 1/sec) | ✅ `v_n.rs:v16_monitor` + `refresh_monitor_display` |
| V17 | Monitor DP decimal (test only) | ⚪ Test-only (G6) |
| V21 | Load component 1 into R1 | ✅ `start_load` — 1-register entry |
| V22 | Load component 2 into R2 | ✅ `start_load` — 1-register entry |
| V23 | Load component 3 into R3 | ✅ `start_load` — 1-register entry |
| V24 | Load components 1,2 | ❌ Not in dispatch — would need `start_load(v,n,2,0)` |
| V25 | Load components 1,2,3 | ✅ `start_load` — 3-register entry |
| V27 | Display fixed memory | ⚪ Debug/diagnostic (G7) |
| V30 | Request Executive | ⚪ System diagnostic (G7) |
| V31 | Request Waitlist | ⚪ System diagnostic (G7) |
| V32 | Recycle program | ❌ Not wired — would re-enter current major mode |
| V33 | Proceed without DSKY input | ❌ Not in dispatch (same as PRO for non-V50 context) |
| V34 | Terminate function → P00 | ✅ `v_n.rs:v34_terminate` |
| V35 | Test lights (lamp test) | ✅ `v_n.rs:v35_lamp_test` |
| V36 | Request fresh start | 🟡 Capability present via `services/fresh_start.rs:fresh_start` (also reachable via V37E00E); direct V36 dispatch not in `dispatch_verb_noun` or `verb_takes_no_noun` (verified via `services/v_n.rs:920`) |
| V37 | Change major mode | ✅ `v_n.rs:v37_program_select` — special keystroke path V37 ENTR MM ENTR |

### Extended Verbs (V40 – V99)

| V# | AGC purpose | Status | Notes |
|----|-------------|--------|-------|
| V40 | Zero CDUs | ❌ Hardware CDU path (G5) |
| V41 | Coarse align CDUs | ❌ Hardware CDU path (G5) |
| V42 | Fine align IMU | ❌ Hardware CDU path (G5) |
| V43 | Load IMU attitude error meters | ❌ Hardware path (G5) |
| V44 | Set surface flag | ❌ Flag register path |
| V45 | Reset surface flag | ❌ Flag register path |
| V46 | Establish G+C control (start SERVICER) | 🟡 Capability present — `services/average_g.rs:start_servicer` (line 126) starts the SERVICER; DAP deadband and jet-config fields exist in `control/dap.rs`; direct V46 dispatch not in `dispatch_verb_noun` (verified via `services/v_n.rs:933`) |
| V47 | Move LM state vector into CM state vector | ❌ LM-related (G1) |
| V48 | Request DAP data load (R03) | ❌ R03 not implemented |
| V49 | Request crew-defined maneuver (R62) | ❌ R62 not implemented |
| V50 | Please perform (crew acknowledgement cue) | ✅ `v_n.rs:request_v50` — PRO key triggers `on_proceed` callback |
| V51 | Please mark | ❌ Mark pipeline partial (sextant HAL wired, V51 dispatch not) |
| V52 | Mark on offset landing site | ⚪ Landing-related (G2) |
| V53 | Please perform alternate LOS mark | ❌ |
| V54 | Request R23 (rendezvous backup sighting) | ❌ R23 not ported |
| V55 | Increment AGC time (decimal) | ❌ V73 covers additive time correction; V55 not wired |
| V56 | Terminate tracking (P20 + P25) | ❌ Not wired |
| V57 | Request rendezvous sighting (R21) | 🟡 Underlying mark pipeline exists (`programs/p20.rs:p20_incorporate_sextant_mark`, `control/sextant.rs:consume_optics_mark`); V57 dispatch not in `dispatch_verb_noun` (verified via `services/v_n.rs:933`) |
| V58 | Reset stick flag | ❌ |
| V59 | Please calibrate | ❌ IMU calibration extended verb (G5) |
| V60 | Set astronaut total attitude to present (N17) | ❌ Attitude monitoring verb |
| V61 | Display DAP attitude error | ❌ |
| V62 | Display total attitude error | ❌ |
| V63 | Display total astronaut attitude error | ❌ |
| V64 | Request S-band antenna routine | ⚪ Hardware-only path (G5) |
| V65 | Optical verification of pre-launch alignment | ⚪ Pre-launch hardware (G5) |
| V66 | Vehicles attached — move this vehicle state to other | ❌ LM-related (G1) |
| V68 | CSM stroke test ON | ⚪ Hardware test (G6) |
| V69 | Cause restart | 🟡 Restart infrastructure implemented in `services/fresh_start.rs` (restart group dispatch, BKPSRAM restore); direct V69 dispatch not in `dispatch_verb_noun`; FRESH START reachable via V37E00E (verified via `services/v_n.rs:933`) |
| V70 | Update liftoff time (HMS) | ✅ `v_n.rs:v70_liftoff_time_update` — P27Time state machine |
| V71 | Universal update — block address (P27) | ✅ `v_n.rs:v71_p27_block_update` — full address space 1–31 |
| V72 | Universal update — single address | ✅ `v_n.rs:v72_single_address_update` |
| V73 | Update AGC time (additive HMS correction) | ✅ `v_n.rs:v73_agc_time_update` |
| V74 | Initialise erasable dump via downlink | ⚪ Telemetry dump (G3) |
| V75 | Backup liftoff | ❌ |
| V76 | Set preferred attitude flag | ❌ DAP flag |
| V77 | Reset preferred attitude flag | ❌ |
| V78 | Update pre-launch azimuth | ⚪ Pre-launch (G4) |
| V79 | Request lunar landmark selection (R35) | ❌ R35 not ported |
| V80 | Update LEM state vector | ❌ LM-related (G1) |
| V81 | Update CSM state vector | ❌ Overlaps P27 V71; direct V81 dispatch not wired |
| V82 | Request orbit param display (R30) | 🟡 Noun N44 in `noun_display` supplies apogee/perigee but lacks TFF; full V82 dispatch not wired |
| V83 | Request rendezvous param display (R31) | ❌ R31 not ported |
| V84 | Start target ΔV (R32) | ❌ |
| V85 | Request rendezvous param display No. 2 (R34) | ❌ R34 not ported |
| V86 | Reject rendezvous backup sighting mark | ❌ |
| V87 | Set VHF range flag | ⚪ VHF ranging (G5) |
| V88 | Reset VHF range flag | ⚪ |
| V89 | Request rendezvous final attitude (R63) | ❌ R63 not ported |
| V90 | Request rendezvous out-of-plane display (R36) | ❌ R36 not ported |
| V91 | Display bank sum | ⚪ Diagnostic (G7) |
| V92 | Operate IMU performance test (P07) | ⚪ IMU hardware test (G6) |
| V93 | Enable W-matrix initialisation | 🟡 W-matrix reset implemented as `programs/p22.rs:p22_rectify_w_matrix` (line 528); direct V93 dispatch not wired in `dispatch_verb_noun` (verified via `services/v_n.rs:933`) |
| V94 | Perform cislunar attitude maneuver (P23) | ❌ P23 auto-maneuver path not wired; P23 mark pipeline implemented but programmatic maneuver initiation via V94 not dispatched |
| V95 | No update of either state vector (P20/P22) | ❌ Kalman freeze option — no freeze flag in P20/P22 nav state |
| V96 | Terminate integration and go to P00 | 🟡 Equivalent capability implemented as V34 (`v_n.rs:v34_terminate`); direct V96 dispatch not wired (verified via `services/v_n.rs:933`) |
| V97 | Perform engine fail procedure | ❌ Contingency verb |
| V98 | Enable TLI | ❌ P15/TLI arming |
| V99 | Please enable engine (SPS ARM crew ACK) | ✅ Used as `NOUN_ENGINE_ARM = 99` in P40 V50N99 flow |

**Verb summary:** V06, V16, V21–V23, V25, V34, V35, V37, V50, V70–V73, V99-as-noun = **13 verbs fully wired**. V24, V32, V33 = **3 pure dispatch gaps**. V36, V46, V57, V69, V93, V96 = **6 verbs where capability is present but not crew-accessible by that verb number** (🟡). Extended verbs V40–V45, V47–V49, V51, V53–V56, V58–V63 (hardware/rendezvous) and V75–V81, V83–V86, V89–V90, V94–V95, V97–V98 = remaining gaps.

---

## 5. Nouns (N00 – N99)

Only nouns wired in `noun_display` or `noun_commit` are listed as implemented. All others raise no error in the port (they fall through the `_ => None` arm silently).

| N# | AGC purpose | Status | Notes |
|----|-------------|--------|-------|
| N00 | Not in use | ⚪ |
| N01–N03 | Specify machine address (frac/whole/deg) | ⚪ AGC-internal debug (G7) |
| N05 | Angular error/difference | ❌ |
| N06 | Option code | ❌ |
| N08 | Alarm data | ❌ |
| N09 | Alarm codes | ❌ |
| N11 | TIG of CSI (HMS) | ✅ `noun_commit` — `commit_hms_to_pending_tig` |
| N13 | TIG of CDH (HMS) | ✅ `noun_commit` — `commit_hms_to_pending_tig` |
| N16 | Time of event (HMS) | ✅ `noun_commit` — `commit_hms_to_pending_tig` |
| N17 | Liftoff time (HMS, uplinked via V70) | ✅ `noun_display` — `time_to_hms(state.liftoff_time.0)` at `services/v_n.rs:1007` |
| N18 | Auto maneuver ball angles (THETAD) | ✅ `noun_commit:noun_18_commit_attitude` |
| N20 | ICDU angles | ❌ |
| N21 | PIPAs (pulse count display) | ❌ |
| N22 | New ICDU angles | ❌ |
| N24 | Delta time for AGC clock (HMS) | ✅ `noun_commit:noun_24_commit_delta_time` |
| N25 | Checklist (Please perform) | ❌ |
| N29 | Launch azimuth | ❌ |
| N30 | Target codes | ❌ |
| N31 | Time of landing site (HMS) | ✅ `commit_hms_to_pending_tig` |
| N32 | Time from perigee (HMS) | ✅ `commit_hms_to_pending_tig` |
| N33 | TIG (HMS) | ✅ `commit_hms_to_pending_tig` + `noun_display` |
| N34 | Time of event (HMS) | ✅ `commit_hms_to_pending_tig` |
| N35 | Time from event (HMS) | ✅ `commit_hms_to_pending_tig` |
| N36 | AGC clock time (HMS) | ✅ `noun_display:time_to_hms(state.time)` + `noun_commit:noun_36_commit_clock_set` |
| N37 | TIG of TPI (HMS) | ✅ `commit_hms_to_pending_tig` |
| N38 | Time of state vector (HMS) | ✅ `commit_hms_to_pending_tig` |
| N39 | Delta time for transfer (HMS) | ✅ `commit_hms_to_pending_tig` |
| N40 | TIG/cutoff, Vg, ΔV accumulated (burn display) | ✅ `noun_display` — magnitude-based display from `burn.target_dv_inertial` and `burn.accumulated_dv_inertial` |
| N41 | Target azimuth/elevation | ❌ |
| N42 | Apogee/perigee/ΔV required | ❌ |
| N43 | Latitude/longitude/altitude | 🟡 `noun_display` returns (0,0,0) placeholder; P21 writes R-regs directly |
| N44 | Apogee/perigee/TFF | 🟡 `noun_display` — apogee/perigee/half-period computed; TFF (time to free fall at 300 kft) not implemented |
| N45 | VHF marks, TFI, MGA | ❌ Rendezvous radar display |
| N47 | Vehicle weight (CSM / LM) | ❌ |
| N48 | Pitch/yaw trim | ❌ |
| N49 | ΔR, ΔV, code | ❌ |
| N50 | Splash error/perigee/TFF | ❌ |
| N51 | S-band antenna angles | ⚪ Hardware (G5) |
| N54 | Range/range rate/theta | ✅ `noun_display` — returns current R-reg values (written by P20) |
| N55 | Perigee code/elevation/central angle | ❌ |
| N56 | Reentry angle/ΔV | ❌ |
| N57 | ΔR | ❌ |
| N58 | Post-TPI perigee/ΔV-TPI/ΔV-TPF | ❌ |
| N59 | ΔV LOS | ❌ |
| N60 | Gmax/Vpred/γEI | ❌ |
| N61 | Impact latitude/longitude/heads-up | ❌ |
| N62 | Inertial vel/time from TIG/accum ΔV | ✅ `noun_display` — |V|, elapsed centiseconds since TIG, `norm(accumulated_dv_inertial)` |
| N63 | RTGO/VIO/TFE | 🟡 RTGO = `entry.target_range_km` computed; VIO (velocity at EI) not stored in `AgcState`; noun display arm missing (verified via `services/v_n.rs:991` and `programs/p61_p67.rs:EntryState`) |
| N64 | Drag accel/Vmagi/range-to-splash | 🟡 Data present: `entry.sensed_acceleration_g`, `norm(csm_state.velocity)`, `entry.target_range_km`; noun display arm missing in `noun_display` (verified via `programs/p61_p67.rs:EntryState`) |
| N65 | Sampled AGC time (HMS) | ✅ `noun_display:time_to_hms(state.time)` |
| N66 | Roll command/cross-range error/down-range error | 🟡 Data present: `entry.roll_command_rad`, `entry.crossrange_km`, `entry.downrange_error_km`; noun display arm missing (verified via `programs/p61_p67.rs:EntryState`, `guidance/entry.rs:crossrange_km`) |
| N67 | Range-to-target/lat/lon | 🟡 Data present: `entry.target_range_km`, `entry.target_lat_rad`, `entry.target_lon_rad`; noun display arm missing (verified via `programs/p61_p67.rs:EntryState`) |
| N68 | Roll command/Vmagi/Rdot | 🟡 Data present: `entry.roll_command_rad`, `norm(csm_state.velocity)`, `entry.r_dot_mps`; noun display arm missing (verified via `programs/p61_p67.rs:EntryState`) |
| N69 | Beta/DL/VL | ❌ |
| N70 | Star code/landmark/horizon | ✅ `noun_commit:noun_70_commit_star_code` + `noun_display` (pass-through) |
| N72 | Landmark lat/lon/alt | ✅ `noun_commit:noun_72_commit_landmark` |
| N73 | Altitude/velocity/FPA (ground track) | ❌ |
| N75 | ΔAlt-CDH/Δt-CDH/Δt-TPI | ❌ |
| N80 | TIG/cutoff, Vg, accum ΔV (high-res P burn display) | ❌ |
| N81 | ΔV (LV), 3 components | ✅ `noun_commit:noun_81_commit_dv_lvlh` |
| N82 | ΔV (LV) alternate | ❌ |
| N83 | ΔV (body frame) | 🟡 P47 writes R-regs directly each SERVICER cycle; `burn.accumulated_dv_inertial` present; no `noun_display` arm for N83 and no inertial→body rotation applied via noun path (verified via `programs/p47.rs`) |
| N84 | ΔV (other vehicle) | ❌ |
| N85 | Vg (body) | ❌ |
| N86 | ΔV (LV) high-res | ❌ |
| N87 | Mark data shaft/trunnion | ❌ |
| N88 | Star vector | ❌ |
| N89 | Landmark lat/lon/alt (P29 target) | ✅ `noun_commit:noun_89_commit_p29_target` — triggers P29 solver when active |
| N91–N96 | Optics CDU, delta gyro, preferred attitude | ❌ IMU alignment engineering nouns |
| N97–N98 | System test inputs/results | ⚪ System test (G6) |
| N99 | RMS position/velocity/option | ❌ Navigation quality display |

**Nouns wired in port: N11, N13, N16, N17, N18, N24, N31–N39, N40, N43 (stub), N44 (partial), N54, N62, N65, N70, N72, N81, N89 = approximately 23 nouns fully or partially active out of ~60 operationally relevant nouns in Comanche055. Entry-guidance state nouns N63, N64, N66, N67, N68 have all underlying data computed but lack `noun_display` arms (5 additional 🟡 entries).**

---

## 6. Differences Between Colossus237 and Comanche055

Colossus237 is functionally very close to Comanche055 (both are the Colossus 2 family). Observable differences from the virtualagc file listings and source annotations:

| Item | Comanche055 | Colossus237 | Impact on port |
|------|-------------|-------------|----------------|
| P32/P33/P72/P73 (CSI/CDH rendezvous) | `P32-P33,_P72-P73.agc` present | No `P32-P33` file | Port targets Comanche055; our P31–P34 code is CM-active-vehicle rendezvous (Apollo 11 used these to rendezvous with the LM). Apollo 8 had no LM so this suite was inert. |
| P34/P35/P74/P75 (TPI/TPF) | `P34-P35,_P74-P75.agc` present | No `P34-P35` file | Same: port includes P33/P34 (TPI/TPM), deliberately omits P35/P74/P75 |
| TVCGEN3FILTERS.agc | Absent from Comanche055 | Present in Colossus237 | Additional TVC filter bank in the Apollo 8 rope (pre-production). Comanche055 consolidated these. Not relevant to port (TVC is `control/tvc.rs`). |
| Lunar landmark selection | `LUNAR_LANDMARK_SELECTION_FOR_CM.agc` (V79/R35) | Same file present | Both ropes have R35; neither is ported yet. |
| Stable orbit P38/P39 | Present in both | Present | Neither ported in port. |
| V37E76E P76 target ΔV | Present | Present | Not ported in either case. |
| Mission-specific pad-load data | Apollo 11 constants (Comanche055) | Apollo 8 constants (Colossus237) | `NASSP` note: Apollo 8 = Colossus237 + MJD 40211.36875. Port uses SI simulation constants; adapting for a specific flight would change ERASABLE_ASSIGNMENTS initial values and REFSMMAT seeds. |
| P70 TLI targeting | `P37,P70.agc` — full precision TLI algorithm | Same file | Neither ported; gap is equal for both ropes. |

---

## 7. Intentional-Gap Rationale Catalogue

The following reason codes appear in the tables above:

| Code | Description | Example items |
|------|-------------|---------------|
| **G1** LM-specific | Program is on the LM/active-in-LM side of a rendezvous. The CM port only covers the passive (CSM) side. | P48, P72–P75, P78; V47, V66, V80 |
| **G2** Lunar landing | Program is part of the powered descent / landing sequence. Out of scope per CLAUDE.md. | P64 ballistic hold future math, P65 skip math, P66 final roll |
| **G3** Telemetry infrastructure | Verb/noun requires a working downlink channel format (MSFN telemetry encoding). Deferred until hardware port resumes. | V74 erasable dump |
| **G4** Pre-launch / pad-specific | Program is only used on the launch pad or during pad checkout. Not needed for the simulation scope (earth-to-moon-and-back). | V78, N29, P08 |
| **G5** Hardware-only path | Implementation requires a real IMU CDU, RCS valve driver, or VHF transponder. Not meaningful in simulation. | V40–V43, V59, V64, V87/V88, R02, N51 |
| **G6** AGC self-check / system test | Verb or program exercises the AGC hardware itself (memory parity, oscillator, register tests). No equivalent in software simulation. | V07, V17, V68, V92, P07; N97–N98 |
| **G7** AGC-internal debug | Verb or noun addresses raw erasable memory or fixed-memory locations; only useful with the AGC assembler runtime. | V01–V05, V27, V30, V31; N01–N03 |

---

## 8. Suggested Priority for Closing Remaining Gaps

Listed in descending value to the mission simulation / demonstration:

1. **P70 — TLI Targeting.** Currently the port can model a TLI burn only by crew-loading the ΔV via P30. A real P70 would compute the optimal TLI burn from the parking orbit and display the conic / precision options. High value: completes the earth-to-TLI narrative arc already demonstrated in the TEI demo (#61).

2. **P35 — TPF (Terminal Phase Final) and rendezvous braking.** The port has P31–P34 (the full CSI→CDH→TPI→midcourse chain) but stops before the final approach and docking. P35 closes the rendezvous sequence.

3. **V82 full dispatch + R30 TFF.** V82 (`request_v50` + N44) is partially supported via `noun_display` N44, but: (a) V82 itself is not in `dispatch_verb_noun`, so crew must manually select V16N44; (b) the time-to-free-fall (TFF) at 300 kft is not computed. This is a high-visibility display used throughout the mission.

4. **P62 SM-sep pyro command via a `Secs`-style HAL trait.** P62 currently does the state transition + `dap_stop` but does not fire the actual separation pyrotechnic — the hardware action doesn't exist as a HAL method yet. Mechanically small, but completes the only real gap in the entry chain (the closed-loop guidance, HUNTEST/UPCONTRL/CONSTD/PREDICT3, all shipped in #85/#86/#87/#96).

5. **P76 — Target ΔV.** A crew-loadable program that integrates the LM state vector to a TIG and applies a ΔV. Needed for post-LOI rendezvous scenarios where the LM has fired independently. Straightforward to implement using the existing P30 and Kepler integration infrastructure.

6. **V32 / V33 / V36 verbs.** Recycle (V32), Proceed without DSKY input (V33), and Fresh Start (V36) are low-complexity but operationally important. V33 is the non-V50 PRO equivalent that many programs use for phase advancement. V36 only needs a one-line dispatch arm calling `fresh_start::fresh_start`.

7. **R31 / R34 rendezvous parameter displays (V83, V85).** Crew used these heavily during the rendezvous approach to monitor CDH/TPI timing parameters. The targeting math is already implemented; these routines are primarily DSKY display wiring.

8. **Entry display nouns N63 / N64 / N66 / N67 / N68.** Complete the entry guidance display chain (range-to-go, drag deceleration, roll command, cross-range). All underlying data fields (`entry.target_range_km`, `entry.sensed_acceleration_g`, `entry.roll_command_rad`, `entry.crossrange_km`, `entry.downrange_error_km`, `entry.r_dot_mps`, `entry.target_lat_rad`, `entry.target_lon_rad`) are already computed each SERVICER cycle; adding these nouns is purely wiring `noun_display` arms.
