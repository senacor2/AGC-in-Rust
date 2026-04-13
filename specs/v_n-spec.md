# Specification: `services/v_n` — Verb/Noun Processor (Consolidated)

**Status**: Implemented through Phase 2; Phase 3+ items listed below  
**Module path**: `agc-core/src/services/v_n.rs`  
**Architecture reference**: `docs/architecture.md` §11 (DSKY and Crew Interface)  
**HAL reference**: `specs/hal-spec.md` §6 (`Dsky` sub-trait)  
**Programs reference**: `specs/p00-spec.md` (V37 E00 E destination)  
**AGC source files**:
- `Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`
- `Comanche055/PINBALL_NOUN_TABLES.agc`
- `Comanche055/EXTENDED_VERBS.agc`
- `Comanche055/KEYRUPT,_UPRUPT.agc`

---

## 1. Purpose and Scope

The Verb/Noun (V/N) processor is the crew interface state machine for the DSKY
(Display and Keyboard). It translates keystrokes into Verb/Noun commands and
dispatches them to the appropriate handler — program select, display request,
data load, or control verb.

The historical AGC name for this subsystem is **PINBALL** (from the source file
`PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`).

### Project scope

This port covers the Comanche055 Command Module, earth-to-moon-and-back travel.
Lunar landing is explicitly out of scope. As a consequence:

- All CM navigation programs (P00–P06, P11, P15, P20–P23, P30–P34, P37,
  P40–P41, P47, P51–P52, P61–P67) are in scope.
- LM-only verbs and nouns are excluded.
- Ground test equipment verbs (self-test, memory dump/load for ground use) are
  excluded unless required for crew operations.

---

## 2. Key Codes

The AGC DSKY uses a 5-bit key matrix. Code values from Comanche055
`PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` (KEYTEMP1 table), given in decimal.

The AGC Symbolic Listing (Section IIJ) gives codes in octal; decimal equivalents
are used below for clarity.

| Key         | Code (decimal) | Notes                                          |
|-------------|----------------|------------------------------------------------|
| `0`         | 16             | Octal 20                                       |
| `1`–`9`     | 1–9            | Code equals decimal value                      |
| `VERB`      | 17             | Octal 21                                       |
| `NOUN`      | 31             | Octal 37                                       |
| `+`         | 26             | Octal 32                                       |
| `−`         | 27             | Octal 33                                       |
| `CLR`       | 30             | Octal 36                                       |
| `KEY REL`   | 25             | Octal 31                                       |
| `ENTR`      | 28             | Octal 34                                       |
| `RSET`      | 18             | Octal 22                                       |
| `PRO`       | channel 32 bit 14 | Not a keycode; polled via hardware channel  |

`PRO` is not transmitted through the keycode path in the original AGC — it is
sensed as bit 14 of channel 32 going low. In this port `Key::Pro` is a logical
key that the HAL shim synthesises from that channel. It is handled directly
in `feed_key` outside the normal digit-accumulation flow.

The `Key` enum provides a typed wrapper; `Key::from_code(u8)` maps a raw HAL
keypress to the enum or returns `None` for unknown codes.

---

## 3. State Machine

### 3.1 VnPhase

```rust
pub enum VnPhase {
    /// Nothing in progress — waiting for VERB or a control key.
    Idle,
    /// VERB pressed, accumulating up to two digits.
    EnteringVerb { digits: u8, buf: u8 },
    /// NOUN pressed after verb complete, accumulating up to two digits.
    EnteringNoun { verb: u8, digits: u8, buf: u8 },
    /// Data entry in progress for a V21/V22/V23/V25 load.
    EnteringData {
        verb: u8,
        noun: u8,
        reg_index: u8,   // which register (0, 1, or 2) is being loaded
        total_regs: u8,  // 1 for V21/22/23, 3 for V25
        sign: i8,        // +1 or -1
        digits: u8,      // 0..=5
        buf: u32,        // 0..=99_999
        committed: [f64; 3],
    },
    /// Operator error — awaiting RSET.
    OprErr,
}
```

### 3.2 VnState

```rust
pub struct VnState {
    pub phase: VnPhase,
    /// TIG stashed by V25/V21 N33 while waiting for delta-V.
    pub pending_tig: Option<Met>,
    /// Pending V50 "please perform" request (set by programs, cleared by PRO).
    pub pending_v50: Option<Pending50>,
}
```

### 3.3 Transitions

| Phase              | Key          | Next phase                                      |
|--------------------|--------------|------------------------------------------------|
| Any                | VERB         | `EnteringVerb { digits:0, buf:0 }`             |
| Any                | RSET         | `Idle`; clear `opr_err` lamp                   |
| Any                | CLR          | `Idle`                                         |
| Any                | PRO          | Invoke `pending_v50` callback if set; no-op otherwise |
| Idle               | other        | OprErr                                         |
| EnteringVerb       | digit        | accumulate; OprErr if digits already 2         |
| EnteringVerb (2d)  | NOUN         | `EnteringNoun { verb:buf, digits:0, buf:0 }`   |
| EnteringVerb (<2d) | NOUN         | OprErr                                         |
| EnteringVerb (2d)  | ENTR         | dispatch (noun-less verbs only); else OprErr   |
| EnteringNoun       | digit        | accumulate; OprErr if digits already 2         |
| EnteringNoun (2d)  | ENTR         | dispatch                                       |
| EnteringNoun       | other        | OprErr                                         |
| EnteringData       | digit        | accumulate into `buf`; OprErr if digits >= 5   |
| EnteringData       | `+`          | set sign=+1 (only when digits==0); else OprErr |
| EnteringData       | `−`          | set sign=-1 (only when digits==0); else OprErr |
| EnteringData       | ENTR         | commit register; advance or call `noun_commit` |
| EnteringData       | other        | OprErr                                         |
| OprErr             | RSET         | `Idle`; clear `opr_err`                        |
| OprErr             | any          | stay until RSET                                |

Digit accumulation is decimal. Verb and noun buffers are clamped to 2 digits;
data buffers to 5 digits. After every key `sync_display` mirrors the in-progress
entry into `state.dsky` so the crew sees each keystroke as they type.

### 3.4 Dispatch

```rust
fn dispatch_verb_noun(state: &mut AgcState, verb: u8, noun: u8) {
    match verb {
        6  => v06_display_decimal(state, noun),
        16 => v16_monitor(state, noun),
        21 | 22 | 23 => start_load(state, verb, noun, 1, verb - 21),
        25 => start_load(state, verb, noun, 3, 0),
        34 => v34_terminate(state),
        35 => v35_lamp_test(state),
        37 => v37_program_select(state, noun),
        50 => { /* V50 is raised by programs; crew responds with PRO */ }
        _  => raise_opr_err(state),
    }
}
```

### 3.5 V50 crew acknowledgement

V50 is not typed by the crew — it is raised by a program calling
`request_v50(state, noun, callback)`. This sets `dsky.verb = 50`,
`dsky.noun = noun`, `dsky.flashing = true`, and stores a `Pending50`
in `state.vn.pending_v50`. When the crew presses PRO, `feed_key`
invokes the callback and clears `pending_v50` and `dsky.flashing`.

---

## 4. Complete Verb Inventory

The following table covers all verbs present in Comanche055
`PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` and `EXTENDED_VERBS.agc`.

Legend for "Status" column:
- **Impl** — coded and dispatched in `v_n.rs`
- **Planned** — in scope, not yet implemented
- **Excluded** — intentionally out of scope (reason given)

### 4.1 Regular Verbs (V01–V39)

| Verb | Name / Description                               | Status   | Notes / Reason                                   |
|------|--------------------------------------------------|----------|--------------------------------------------------|
| V01  | Display octal (R1 only)                          | Planned  | Octal display; needed for engineering readouts   |
| V02  | Display octal (R1, R2)                           | Planned  | Octal display                                    |
| V03  | Display octal (R1, R2, R3)                       | Planned  | Octal display                                    |
| V04  | Display octal, double-precision (R1 high, R2 low) | Planned  | DP octal display                                 |
| V05  | Display octal/decimal (R1 only)                  | Planned  | Mixed display                                    |
| V06  | Display decimal (R1, R2, R3)                     | **Impl** | Core display verb; dispatches to `noun_display`  |
| V07  | Display double-precision decimal (R1 high, R2 low) | Planned | DP decimal; used by some nav programs            |
| V11  | Monitor octal (R1)                               | Planned  | Continuous monitor variant of V01                |
| V12  | Monitor octal (R1, R2)                           | Planned  | Continuous monitor variant of V02                |
| V13  | Monitor octal (R1, R2, R3)                       | Planned  | Continuous monitor variant of V03                |
| V14  | Monitor octal, DP                                | Planned  | Continuous monitor variant of V04                |
| V16  | Monitor decimal (R1, R2, R3)                     | **Impl** | Continuous decimal monitor; `refresh_monitor_display` |
| V17  | Monitor double-precision decimal                 | Planned  | Continuous DP decimal monitor                    |
| V21  | Load R1 (decimal, 5 digits)                      | **Impl** | Data entry; starts `EnteringData`                |
| V22  | Load R2 (decimal, 5 digits)                      | **Impl** | Data entry                                       |
| V23  | Load R3 (decimal, 5 digits)                      | **Impl** | Data entry                                       |
| V24  | Load R1 and R2                                   | Planned  | Two-register load; less common than V25          |
| V25  | Load R1, R2, R3 (all three)                      | **Impl** | Primary data-entry verb                          |
| V26  | Load octal into R1                               | Excluded | Octal data entry; not needed for CM flight ops   |
| V27  | Display and load R1, R2, R3 (current values shown first) | Planned | Confirm-then-override pattern used in targeting |
| V32  | Request ground uplink                            | Excluded | Ground-to-AGC uplink; crew-side request handled by uplink system, not needed in Rust port |
| V33  | Proceed without data                             | Planned  | Crew acknowledgement when no data change needed; companion to V50 flow |
| V34  | Terminate program (return to P00)                | **Impl** | Control verb; no noun                            |
| V35  | Lamp test (all lamps on)                         | **Impl** | Control verb; no noun                            |
| V36  | Fresh start                                      | Excluded | Hardware restart sequence; equivalent to power-on; not a normal flight operation |
| V37  | Change major mode (program select)               | **Impl** | Dispatches via `PROGRAM_TABLE[noun]`             |

### 4.2 Extended Verbs (V40–V99, from EXTENDED_VERBS.agc)

| Verb | Name / Description                               | Status   | Notes / Reason                                   |
|------|--------------------------------------------------|----------|--------------------------------------------------|
| V40  | Zero CDU (command CDU angles to zero)            | Excluded | IMU CDU zeroing; ground/test use only            |
| V41  | Set coarse align CDU (gyrocompass)               | Excluded | Ground alignment only; P51/P52 handle in-flight align |
| V42  | Display zero                                     | Excluded | Test utility; no flight use in CM scope          |
| V43  | Display MMBR                                     | Excluded | Memory bank register display; maintenance only   |
| V44  | Freeze PIPA (accelerometer) outputs              | Excluded | Test/cal; not needed in flight ops               |
| V45  | Spare                                            | Excluded | Unused in Comanche055                            |
| V46  | Mark (star/horizon sighting)                     | Planned  | Used by P52 star-sighting alignment; should be wired to `p51_p52` |
| V47  | Gyro torquing (fine alignment)                   | Planned  | Used by P52 alignment; currently P47 program handles; V47 verb signals readiness |
| V48  | Load IMU fine alignment torquing angles (R1, R2, R3) | Planned | P52 fine-align data entry                    |
| V49  | Request voice (not applicable CM)                | Excluded | LM-only voice annunciator                       |
| V50  | Please perform (crew request)                    | **Impl** | Raised by programs; crew responds with PRO; not typed |
| V51  | Mark (optics CDU marks for P20/P22)              | Planned  | Needed for P22 landmark tracking; signals optics mark |
| V52  | Mark rejected                                    | Planned  | Companion to V51; crew rejects a mark           |
| V53  | Optics zero                                      | Excluded | Optics CDU zeroing; maintenance                  |
| V55  | Update AGC with navigation base data             | Excluded | Uplink-related; ground updates only              |
| V56  | Request attitude error display                   | Planned  | Attitude error monitoring; used in burn phases   |
| V57  | Display inertial velocity                        | Planned  | Used during burns for velocity monitoring        |
| V58  | Display CMC ATT (computer attitude)              | Planned  | Attitude display during alignment and burns      |
| V59  | Update LM state vector                           | Excluded | LM state vector; out of scope (no LM in this port) |
| V60  | Display star name (R1 = catalog number)          | Planned  | Used by P52 star identification                  |
| V62  | Apply REFSMMAT (from stored value)               | Planned  | Platform alignment reference matrix apply; P51/P52 |
| V63  | Drift test control                               | Excluded | IMU drift test; ground-only operation            |
| V64  | Designate optics to object (P20)                 | Planned  | Optics auto-designate for P20 rendezvous         |
| V65  | Load thrust level percentage (RCS)               | Excluded | RCS thrust calibration; ground test              |
| V66  | Mark star sighting (optics)                      | Planned  | Used by P23 cislunar navigation                  |
| V67  | Perform automatic attitude maneuver              | Planned  | Used by P40/P41 for pre-burn orientation         |
| V68  | Display time from event                          | Planned  | Time-to-go display for burns                     |
| V69  | Cause restart (CMC restart)                      | Excluded | Deliberately restarts computer; maintenance use only |
| V70  | Set AGC clock (load time from N36)               | Planned  | Clock set operation; crew-callable               |
| V71  | Test lights (alternate lamp test)                | Excluded | Duplicate of V35; V35 is the standard            |
| V72  | Place guidance system in test mode               | Excluded | Hardware test mode; not flight operation         |
| V73  | Return from guidance system test mode            | Excluded | Companion to V72; excluded for same reason       |
| V74  | Spare                                            | Excluded | Not assigned in Comanche055                      |
| V75  | Display W-matrix diagonal (navigation error)     | Planned  | Navigation filter state display; useful for CM nav |
| V76  | Fail gimbal motor                                | Excluded | Hardware test only                               |
| V77  | Spare                                            | Excluded | Not assigned in Comanche055                      |
| V78  | Update IMU with erasable memory values           | Excluded | Ground upload; not a crew flight operation       |
| V79  | Spare                                            | Excluded | Not assigned in Comanche055                      |
| V82  | Request orbit parameters display (N44)           | Planned  | Standard apogee/perigee/period display; V82 N44  |
| V83  | Request orbital velocity display (N62)           | Planned  | Velocity and ΔV display during burns             |
| V85  | Display angle between two vectors                | Planned  | Used in P52 alignment checks                     |
| V86  | Display platform attitude                        | Planned  | FDAI/ball attitude display for burns             |
| V87  | Request LOS (line of sight) display              | Excluded | LM/optics LOS; not needed without LM             |
| V89  | Request lunar surface alignment                  | Excluded | LM surface operations; out of scope              |
| V90  | Display DSKY time to next ignition               | Planned  | Time-to-ignition countdown display               |
| V91  | Display R/R-dot (rendezvous radar)               | Excluded | Rendezvous radar display — P20/P22 write N54 directly; no separate verb needed in this port |
| V93  | Request ISS mode control                         | Excluded | ISS (Inertial Sub-System) mode; maintenance      |
| V96  | Terminate integration                            | Excluded | Navigator integration halt; maintenance/test     |
| V97  | Perform engine on/off (manual)                   | Planned  | Manual SPS engine control; needed for P40 abort  |
| V99  | Astronaut total attitude maneuver (manual)       | Planned  | Manual attitude control via DSKY; used in P47    |

---

## 5. Complete Noun Inventory

Source: `Comanche055/PINBALL_NOUN_TABLES.agc`. The noun table maps noun numbers
to erasable memory locations and scaling; the `noun_display` function in `v_n.rs`
implements the display side (read), and `noun_commit` implements the data-entry
side (write).

Legend for "Display" / "Commit" columns:
- **Impl** — function coded in `v_n.rs`
- **Planned** — in scope, not yet implemented
- **Excluded** — intentionally out of scope

### 5.1 Noun Table (N00–N99)

| Noun | Description                                         | R1 / R2 / R3                                  | Display | Commit | Notes                                       |
|------|-----------------------------------------------------|-----------------------------------------------|---------|--------|---------------------------------------------|
| N01  | Spare                                               | —                                             | Excluded | Excluded | Not used                               |
| N02  | Spare                                               | —                                             | Excluded | Excluded | Not used                               |
| N09  | Alarm codes                                         | Alarm word 1 / word 2 / word 3               | Planned | —      | Read-only; used with V05                    |
| N11  | (Reserved — Comanche)                               | —                                             | Excluded | Excluded |                                            |
| N14  | Desired CMC attitude (platform)                     | R1=roll / R2=pitch / R3=yaw (degrees×100)     | Planned | Planned | Used in P40 pre-burn attitude maneuver     |
| N15  | Delta time (coarse) for P02                        | R1=ΔT hours / R2=ΔT min / R3=ΔT sec×100      | Planned | Planned | Time increment for coarse align             |
| N17  | Star angle difference (sextant–AGC)                 | R1=Δ (arc-sec)                               | Planned | —      | P52 alignment check display                 |
| N18  | Auto maneuver angles (P47)                          | R1=roll / R2=pitch / R3=yaw target (deg×100)  | Planned | Planned | P47 attitude maneuver target               |
| N19  | RCS maneuver attitude                               | R1=roll / R2=pitch / R3=yaw                   | Planned | Planned | Used in P40/P41                             |
| N20  | Icdu (CDU angles)                                   | R1=OGA / R2=MGA / R3=IGA (degrees)           | Planned | —      | Platform gimbal angles                      |
| N22  | New REFSMMAT vector set (R1=angle)                  | R1=angle choice                              | Excluded | Excluded | Internal table selection; ground use       |
| N23  | Compensation angles (fine align)                    | R1 / R2 / R3 (milliradians)                  | Planned | Planned | P52 torquing angles                         |
| N25  | Star code / star occultation                        | R1=star code                                 | Planned | Planned | P51/P52 star selection                      |
| N27  | Rendezvous radar data                               | R1=range / R2=range rate / R3=spare          | Planned | —      | P20 rendezvous; written by radar driver     |
| N30  | Reentry target parameters (P61–P67)                 | R1=range angle / R2=K-factor / R3=RRT       | Planned | —      | Entry guidance display                      |
| N31  | ΔV (LGC to AGC state vector)                        | —                                             | Excluded | Excluded | LM Guidance Computer sync; no LM here     |
| N32  | Height above lunar surface                          | —                                             | Excluded | Excluded | LM-only                                    |
| N33  | TIG (Time of Ignition), GET                         | R1=hours / R2=minutes / R3=seconds×100        | **Impl** | **Impl** | Core burn targeting noun                  |
| N34  | TFI (Time from Ignition to cutoff)                  | R1=hours / R2=minutes / R3=seconds×100        | Planned | Planned | Used in P30/P40; burn duration             |
| N35  | Checklist item (crew procedure step)                | R1=item number                               | Planned | —      | Display only; no write                      |
| N36  | Ground elapsed time (GET)                           | R1=hours / R2=minutes / R3=seconds×100        | **Impl** | —      | Vehicle mission clock; read-only            |
| N37  | Time to next maneuver event                         | R1=hours / R2=minutes / R3=seconds×100        | Planned | —      | Countdown display                           |
| N38  | Time of node (ascending/descending)                 | R1=hours / R2=minutes / R3=seconds×100        | Planned | —      | Orbital mechanics display                   |
| N39  | Latitude and longitude of sub-satellite point       | R1=lat (deg×100) / R2=lon (deg×100) / R3=alt | Planned | —      | Used in P21 ground track                    |
| N40  | Burn display (ΔV magnitudes)                        | R1=target ΔV / R2=accumulated ΔV / R3=remaining ΔV | **Impl** | — | Core burn monitor noun                 |
| N41  | Attitude error (FDAI needles)                       | R1=roll err / R2=pitch err / R3=yaw err (deg×100) | Planned | — | Attitude guidance display              |
| N42  | RCS attitude error (body axes)                      | R1=roll / R2=pitch / R3=yaw                  | Planned | —      | Used during RCS burns                       |
| N43  | Position of LM relative to CSM                     | R1=lat / R2=lon / R3=alt (used also as nav ref) | **Impl** | — | Placeholder; P21 writes directly        |
| N44  | Orbital parameters (apogee / perigee / half-period) | R1=apogee alt (m) / R2=perigee alt (m) / R3=half-period (s) | **Impl** | — | Orbital mechanics; computed from state |
| N45  | Gyro compensation angles                            | R1/R2/R3 (milli-degrees)                     | Planned | Planned | IMU fine alignment                          |
| N46  | Spare                                               | —                                             | Excluded | Excluded |                                            |
| N47  | Reentry trajectory angle                            | R1=flight path angle                         | Planned | —      | P61–P67 entry guidance                      |
| N48  | Reentry delta velocity                              | R1/R2/R3 ΔV components                      | Planned | Planned | Entry ΔV targeting                          |
| N49  | Altitude rate / altitude                            | R1=alt rate (m/s) / R2=altitude (km) / R3=   | Planned | —      | Entry monitoring                            |
| N54  | Range / range rate / theta (P20 rendezvous)         | R1=range (km) / R2=range rate (m/s) / R3=θ   | **Impl** | —      | Written by P20 directly; V16 reads it       |
| N55  | Star ID angle from P52                              | R1=shaft / R2=trunnion (deg×100)             | Planned | —      | Optics angle display                        |
| N56  | Spare                                               | —                                             | Excluded | Excluded |                                            |
| N57  | Desired FDAI attitude                               | R1=roll / R2=pitch / R3=yaw (deg×100)        | Planned | Planned | P40 attitude maneuver target                |
| N58  | ICDU/OCDU angles (optics)                           | R1=shaft / R2=trunnion / R3=                 | Planned | —      | P20/P22 optics CDU                          |
| N59  | Corrector delta-V                                   | R1/R2/R3 (m/s × scale)                      | Planned | —      | Mid-course correction ΔV, P31/P32           |
| N60  | Camera angle (CM optics)                            | R1/R2                                        | Excluded | Excluded | Optics camera; not modelled in this port   |
| N61  | Entry target (reentry angle)                        | R1=target angle                              | Planned | Planned | P61–P67 entry targeting                     |
| N62  | Absolute velocity / time from TIG / accumulated ΔV  | R1=\|V\| (m/s) / R2=time from TIG (s×100) / R3=accum ΔV (m/s) | **Impl** | — | Burn monitoring |
| N63  | Extended reentry corridor parameters                | R1/R2/R3                                     | Planned | —      | P61–P67 corridor display                    |
| N65  | Mission elapsed time (duplicate of N36 format)      | R1=hours / R2=minutes / R3=seconds×100        | **Impl** | —      | Used when N36 context requires a second time channel |
| N67  | Spare                                               | —                                             | Excluded | Excluded |                                            |
| N68  | Optics CDU and shaft/trunnion angles                | R1=OGA / R2=shaft / R3=trunnion (deg×100)    | Planned | —      | P20/P22                                     |
| N69  | FDAI attitude (roll / pitch / yaw)                  | R1=roll / R2=pitch / R3=yaw (deg×100)        | Planned | —      | Platform attitude display                   |
| N70  | Planet or star selection code                       | R1=code                                      | Planned | Planned | P23 cislunar navigation; crew enters code   |
| N71  | IMU calendar time                                   | R1=year+day / R2=hours+min / R3=sec×100      | Planned | Planned | IMU clock set; V70 N71                      |
| N72  | Latitude / longitude / altitude of landmark         | R1=lat / R2=lon / R3=alt                     | Planned | Planned | P22 landmark tracking; crew-entered target  |
| N73  | Number of marks taken (P22)                         | R1=count                                     | Planned | —      | P22 progress display                        |
| N74  | Altitude rate and altitude (reentry)               | R1=alt rate / R2=alt                         | Planned | —      | P61/P63 entry display                       |
| N75  | Velocity to be gained (P37)                        | R1/R2/R3 ΔV (m/s)                           | Planned | —      | P37 return-to-earth ΔV display              |
| N76  | Target ID (for P20/P22 rendezvous)                  | R1=target code                               | Planned | Planned | Rendezvous target selection                 |
| N77  | Delta velocity (inertial, P40/P41)                  | R1/R2/R3 ΔV (m/s)                           | Planned | —      | Burn ΔV components display                  |
| N78  | Spare                                               | —                                             | Excluded | Excluded |                                            |
| N79  | Spare                                               | —                                             | Excluded | Excluded |                                            |
| N80  | Parking orbit / injection parameters (P11)          | R1=altitude / R2=inclination / R3=            | Planned | Planned | P11 earth parking orbit entry               |
| N81  | ΔV (LVLH frame) for targeting (P30)                 | R1=ΔVx / R2=ΔVy / R3=ΔVz (m/s)              | **Impl** | **Impl** | Primary burn targeting noun                |
| N82  | Spare                                               | —                                             | Excluded | Excluded |                                            |
| N84  | Apogee/perigee after maneuver (P30 prediction)      | R1=apo alt / R2=peri alt / R3=TFF            | Planned | —      | Post-burn orbit prediction; P30             |
| N85  | ΔV (VGC) remaining                                  | R1/R2/R3 (m/s)                              | Planned | —      | Used in P40/P41 during burn                 |
| N86  | Crew-selected option code                           | R1=option                                    | Planned | Planned | Used in P20/P30/P37 decision points         |
| N87  | Star code for P51 platform alignment               | R1=star 1 / R2=star 2 / R3=                  | Planned | Planned | P51 star pair entry                         |
| N88  | Preferred REFSMMAT source selection                 | R1=selection code                            | Planned | Planned | Used in P52 to choose which REFSMMAT to use |
| N89  | Spare                                               | —                                             | Excluded | Excluded |                                            |
| N90  | IMU temperature                                     | R1=temperature (°C × 10)                     | Excluded | Excluded | Thermal monitoring; not modelled            |
| N91  | Vehicle altitude and altitude rate (P11)            | R1=alt (km) / R2=alt rate (m/s) / R3=        | Planned | —      | P11 ascent monitoring                       |
| N92  | Spare                                               | —                                             | Excluded | Excluded |                                            |
| N93  | Desired free-fall time (P30/P37)                    | R1=hours / R2=min / R3=sec×100               | Planned | Planned | P37 return-to-earth free-fall duration entry |
| N94  | RCS ΔV (P41)                                       | R1/R2/R3 (m/s)                              | Planned | Planned | P41 RCS burn data entry                     |
| N95  | Reentry parameters (P61)                            | R1/R2/R3                                     | Planned | —      | P61/P67 entry guidance                      |
| N97  | RCS performance (accumulated ΔV)                    | R1=accumulated ΔV                            | Planned | —      | P41 RCS burn monitoring                     |
| N98  | Crew clock for rendezvous TIG estimate              | R1=hours / R2=min / R3=sec×100               | Planned | Planned | P30/P32/P33/P34 rendezvous timing           |
| N99  | Computer activity (self-test result)                | R1=result code                               | Excluded | Excluded | Hardware test; not a flight operation       |

---

## 6. noun_display Mappings (currently implemented)

| Noun | R1                              | R2                             | R3                             | Source / Notes                      |
|------|----------------------------------|--------------------------------|--------------------------------|--------------------------------------|
| N33  | Pending TIG hours                | Pending TIG minutes            | Pending TIG seconds×100        | `state.vn.pending_tig`               |
| N36  | GET hours                        | GET minutes                    | GET seconds×100                | `state.time`                         |
| N40  | Target ΔV magnitude (m/s)       | Accumulated ΔV magnitude (m/s) | Remaining ΔV magnitude (m/s)   | `state.burn.*_dv_inertial`           |
| N43  | 0.0 (placeholder)                | 0.0                            | 0.0                            | P21 writes directly; stub here       |
| N44  | Apogee altitude (m)             | Perigee altitude (m)           | Half-orbital period (s)        | Computed from `state.csm_state`      |
| N54  | Range (current dsky.r[0])       | Range rate (current dsky.r[1]) | Theta (current dsky.r[2])      | P20 writes directly; read-through    |
| N62  | Absolute velocity (m/s)         | Time from TIG (s×100)          | Accumulated ΔV magnitude (m/s) | `state.csm_state.velocity`, `state.burn` |
| N65  | Mission time hours               | Mission time minutes           | Mission time seconds×100       | `state.time` (same as N36)           |
| N81  | ΔVx of pending maneuver (m/s)   | ΔVy                            | ΔVz                            | `state.pending_maneuver`             |

Time display convention: R3 is expressed as SSSCC (seconds×100 + centiseconds
expressed as integer), so 30.45 s → 3045. R1 = whole hours, R2 = whole minutes.

---

## 7. noun_commit Mappings (currently implemented)

| Noun | V21 reg | V22 reg | V23 reg | V25 regs     | Effect                                               |
|------|---------|---------|---------|--------------|------------------------------------------------------|
| N33  | R1=TIG  | —       | —       | R1=TIG only  | Sets `state.vn.pending_tig = Some(Met(value as u32))` |
| N81  | —       | —       | —       | R1/R2/R3=ΔV  | Consumes `pending_tig`; calls `p30_load_dv_lvlh`     |

Note: V21 N33 and V25 N33 both commit through the same `noun_33_commit_tig`
handler. V25 N33 ignores R2/R3 (N33 is a single-component noun in the AGC).

---

## 8. noun_scale Table

| Noun | Scale factor | Units after scaling         | Notes                                    |
|------|--------------|-----------------------------|------------------------------------------|
| N33  | 1.0          | centiseconds (integer)      | Exact integer; TIG in mission elapsed cs |
| N34  | 1.0          | centiseconds (integer)      | Placeholder for TFI                     |
| N81  | 1.0          | m/s (integer)               | Real AGC uses B-7 (≈0.00784 m/s/bit); integer used for test clarity |
| all others | 1.0  | pass-through                | Refine per noun as additional nouns are committed |

Real AGC scaling uses powers of 2 (the "B" notation, e.g. B-7 means × 2⁻⁷).
The Phase 2 simplification of integer units should be refined as each
additional noun's commit handler is implemented.

---

## 9. Gap Analysis — Missing but Needed

The following verb/noun combinations are in scope for the CM earth-to-moon-and-back
mission but not yet implemented. They are listed in priority order based on which
programs already exist and what they need to call.

### 9.1 High priority (blocks existing programs)

| Gap | Reason |
|-----|--------|
| N34 display + commit | P30/P40 need to display and accept TFI (time of burn); currently only TIG (N33) is committed |
| N44 display (apogee/perigee) | Implemented for orbital display but needs `V82 N44` verb shortcut (V82 currently unrouted) |
| N84 display (post-burn orbit prediction) | P30 shows predicted post-burn orbit; `noun_display` needs this noun |
| N77 display (ΔV inertial components) | P40/P41 show burn ΔV components; currently only N81 is in `noun_display` |
| V46 + V66 (mark verbs for P23, P52) | P52 alignment and P23 cislunar navigation both require a "mark" signal |
| N14, N57 commit (attitude targets) | P40/P41 attitude maneuver data entry; verbs V67/V25 N14 sequence |
| N25 commit (star codes, P51/P52) | P51/P52 star selection by crew                                     |

### 9.2 Medium priority (needed for complete mission flow)

| Gap | Reason |
|-----|--------|
| N86 display + commit (option codes) | P20/P30/P37 present option choices to crew; crew selects by V25 N86 |
| N87 commit (star pair P51)         | P51 platform alignment requires star pair entry |
| N88 commit (REFSMMAT selection)    | P52 REFSMMAT choice                             |
| N76 commit (target selection P22)  | P22 landmark tracking requires target ID entry  |
| N72 commit (landmark lat/lon/alt)  | P22 landmark entry                              |
| V33 (proceed without data)         | Crew decline pattern in V50 flows; some programs offer "press V33 to skip" |
| V27 (display then load)            | Used in targeting flows where current value is shown before overwrite |
| N80 commit (P11 orbit parameters)  | P11 earth parking orbit data entry              |

### 9.3 Lower priority (end-of-mission programs)

| Gap | Reason |
|-----|--------|
| N30, N61, N95 display/commit | P61–P67 reentry guidance; relevant but later phase |
| N47, N48, N49, N63, N74 display | Entry trajectory monitoring; P61–P67 |
| V46, V51, V52 (sighting marks) | P20/P22 optics operations; P20 already uses N54 directly |

---

## 10. Intentional Exclusions

| Verb/Noun | Reason for Exclusion |
|-----------|----------------------|
| V01–V05, V11–V14 (octal display verbs) | Not needed for flight-crew operations; all meaningful data is decimal in CM context. May be added later as a debugging aid but have zero flight-mission use cases. |
| V26 (load octal R1) | Octal data entry; no flight use in CM |
| V32 (request uplink) | Ground-to-AGC uplink; uplink path modelled separately |
| V36 (fresh start) | Power-on reset sequence; no interactive path |
| V40–V44 (CDU zero, gyrocompass, test) | Ground alignment/test procedures only |
| V49 (voice) | LM-only |
| V53 (optics zero) | Maintenance |
| V55 (uplink nav base) | Ground data upload |
| V63 (drift test) | Ground IMU calibration |
| V69 (cause restart) | Maintenance; the restart subsystem handles this at the executive level |
| V71–V73 (test mode) | Hardware test; not flight operations |
| V76–V79 (spare/test) | Unassigned or test only |
| V87, V89 (lunar surface) | LM surface operations |
| V91, V93, V96 (radar, ISS, integration halt) | Not needed for flight ops in this port |
| N31 (LGC–AGC ΔV) | LM Guidance Computer sync; no LM in this port |
| N32 (lunar surface height) | LM-only |
| N60 (camera) | Optics camera not modelled |
| N67, N78, N79, N82, N89, N92 (spare) | Not assigned in Comanche055 |
| N90 (IMU temperature) | Thermal monitoring not modelled |
| N99 (self-test) | Hardware self-test |

---

## 11. State Machine Tests

The following test cases are implemented in `v_n.rs` under `#[cfg(test)]`.

### Phase 1 tests (TC-VN-*)

| ID | Description |
|----|-------------|
| TC-VN-1 | `Key::from_code` round trip — all canonical codes map; 255 → None |
| TC-VN-2 | V37 E00 E selects P00 and returns to Idle |
| TC-VN-3 | V37 E30 E selects P30 and leaves `major_mode = 30` |
| TC-VN-4 | V06 N40 E sets `dsky.verb=6`, `dsky.noun=40`, no burn mutation |
| TC-VN-5 | V34 E terminates to P00 |
| TC-VN-6 | V35 E sets `dsky.lamp_test_active = true` |
| TC-VN-7 | Unknown verb raises OPR ERR |
| TC-VN-8 | RSET clears OPR ERR and returns to Idle |
| TC-VN-9 | VERB during EnteringNoun restarts entry |
| TC-VN-10 | CLR from EnteringVerb returns to Idle |
| TC-VN-11 | V37 with unknown program number raises OPR ERR |
| TC-VN-12 | Single-digit verb followed by NOUN raises OPR ERR |

### Phase 2 tests (TC-VND-*)

| ID | Description |
|----|-------------|
| TC-VND-1 | V21 N33 E +100 E → commits TIG = Met(100) |
| TC-VND-2 | V25 N33 E +50000 E → `pending_tig = Some(Met(50_000))` |
| TC-VND-3 | V25 N81 E +100 E +0 E +0 E with prior pending_tig → pending_maneuver is Some |
| TC-VND-4 | V25 N81 without prior N33 TIG → alarm 240, no pending_maneuver |
| TC-VND-5 | Minus sign before first digit: −100 is committed correctly |
| TC-VND-6 | Sign after digit raises OPR ERR |
| TC-VND-7 | Six digits raises OPR ERR |
| TC-VND-8 | CLR during data entry aborts; phase → Idle |
| TC-VND-9 | V21 N33 loads R1 only and commits immediately |
| TC-VND-10 | End-to-end: V25 N33 → V25 N81 → P30 pending_maneuver |

---

## 12. Alarm Codes

| Code | Meaning |
|------|---------|
| 240  | `ALARM_DV_LOAD_WITHOUT_TIG` — V25 N81 entered without a prior V25/V21 N33 TIG load |

---

## 13. AgcState Integration

```rust
// In AgcState:
pub vn: VnState,    // Initialised with VnState::new()
```

The single public entry point is `feed_key(state, key)`, called by the KEYRUPT
ISR shim on bare metal or by the test harness.

`refresh_monitor_display(state)` is called by the periodic display refresh task
(T4RUPT or a 1 Hz background job) to update R1/R2/R3 while V16 is active.

`request_v50(state, noun, on_proceed)` is called by programs that need crew
acknowledgement before a critical operation.
