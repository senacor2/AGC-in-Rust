# agc-sim

Unified three-panel terminal simulator for the AGC-in-Rust project. Built on
[ratatui](https://ratatui.rs) + [crossterm](https://github.com/crossterm-rs/crossterm).

One binary, one DSKY, one mission panel, one event log. Designed for
presentations: drive the AGC with **real Apollo verb/noun keystrokes** and watch
the navigation + guidance + control pipeline respond live.

## Running

```sh
# Default — free flight sandbox (200 km LEO)
cargo run -p agc-sim --bin agc_sim

# Pre-loaded scenarios
cargo run -p agc-sim --bin agc_sim -- --scenario launch   # P11 Launch Monitor
cargo run -p agc-sim --bin agc_sim -- --scenario burn     # P40 SPS burn demo
cargo run -p agc-sim --bin agc_sim -- --scenario free     # Free flight sandbox
```

While running, press **F1 / F2 / F3** to swap scenarios live and **Q** to quit.

## Unified Layout

```
┌ APOLLO GUIDANCE COMPUTER ─────────────┬ MISSION STATE ───────────────────────────┐
│ UPLNK  TEMP  GBL  NOATT                │ MET +000:01:24.37  P40 — SPS Thrust     │
│ STBY  PROG  KREL  RSTR  OPER  TRKR     │ SCEN Targeted Burn                      │
│ COMP ACTY [●]    PROG  40              │ X   +6578.000 km   +0.000 m/s           │
│ VERB 37          NOUN  40              │ Y      +0.000 km +7784.268 m/s          │
│ R1   +00585                             │ Z      +0.000 km   +0.000 m/s          │
│ R2   +00000                             │ ALT  +204.662 km  SPD 7784.268 m/s     │
│ R3   +00000                             │ SMA   6578.0 km   ECC 0.0000            │
│ [V]erb [N]oun [0-9] [+/-]               │ APO   +200.0 km   PER   +200.0 km      │
│ [Enter] [Clr] [P]ro [R]set              │ ORB ████░░░░░░░░░░░░  15.3%  T 5309s   │
│                                         │ DAP [BURN]  ENG [● FIRING]              │
│                                         │ VG +40.88 m/s  TGO  9.0s  ΣΔV +9.12 m/s│
│                                         │ [F1] Launch Monitor                     │
│                                         │ [F2] Targeted Burn ← active             │
│                                         │ [F3] Free Flight                        │
├─────────────────────────────────────────┴─────────────────────────────────────────┤
│ MISSION LOG                                                                        │
│ +000:01:24.35  I/O   KEY → Verb                                                   │
│ +000:01:24.37  PGM   V37N40 — entering P40 — SPS Thrust                            │
│ +000:01:24.37  GNC   P40: BurnState armed 50.000 m/s prograde                      │
│ +000:01:24.38  DAP   FREE → BURN                                                   │
└────────────────────────────────────────────────────────────────────────────────────┘
```

### DSKY (left panel)

Mirrors the physical Apollo DSKY (Fig. 39 in Frank O'Brien's book):

| Section | What it shows |
|---|---|
| **Indicator lights** (2 rows) | UPLNK · TEMP · GBL · NOATT · STBY · PROG · KREL · RSTR · OPER · TRKR — reversed video when lit, dim when off |
| **COMP ACTY** | The "computer active" heartbeat — blinks whenever PINBALL is working |
| **PROG / VERB / NOUN** | Two-digit mode / verb / noun fields |
| **R1 / R2 / R3** | Three 5-digit signed registers. Content depends on the active Noun (see Noun table below) |
| **Key hints** | Your PC keyboard mapping |

### Mission State (right panel)

Live output from the `agc-core` navigation and guidance pipeline:

| Line | Meaning | Source |
|---|---|---|
| **MET** | Mission Elapsed Time (from `StateVector::t`) | `services/average_g` cycles |
| **PHASE** | Active program label (P00, P11, P40, …) | `mission.phase_label()` |
| **X/Y/Z** | ECI position (km) and velocity (m/s) | `StateVector.r` / `.v` |
| **ALT / SPD** | Altitude above `RE_EARTH` (6,373,338 m) and inertial speed | derived |
| **SMA / ECC** | Semi-major axis, eccentricity | `navigation::conics::rv_to_elements` |
| **APO / PER** | Apoapsis / periapsis altitudes | `navigation::conics::apsides` |
| **ORB bar** | Fraction of one orbit completed since MET reset | derived |
| **DAP** | Digital Autopilot mode (`FREE` / `BURN`) | burn state |
| **ENG** | Engine firing indicator | burn state |
| **VG / TGO / ΣΔV** | Velocity-to-be-gained / time-to-go / cumulative delta-V | `guidance::maneuver::BurnState` |

### Mission Log (bottom)

Ring buffer of the last events. Tag legend:

| Tag | Meaning |
|---|---|
| `INFO` | Neutral state change |
| `I/O`  | DSKY key press, relay write, PIPA pulse |
| `PGM`  | Program (P-code) entry/exit |
| `GNC`  | Guidance and navigation computation |
| `DAP`  | Digital Autopilot mode transition |
| `ENG`  | Engine command (ignite / cutoff / gimbal) |
| `WARN` | Non-fatal condition |
| `ALARM`| Program alarm raised (1202, 1210, …) |

## Keyboard Map

### DSKY keys (same as the real Apollo DSKY)

| PC key | DSKY key | Function |
|---|---|---|
| `v` | **VERB** | Begin verb-number entry |
| `n` | **NOUN** | Begin noun-number entry |
| `0`–`9` | digits | Digit entry |
| `+` | `+` | Plus sign |
| `-` | `–` | Minus sign |
| `Enter` | **ENTR** | Confirm entry / execute V/N |
| `Del` / `Backspace` | **CLR** | Clear current entry |
| `p` | **PRO** | Proceed / acknowledge |
| `r` | **RSET** | Reset — clear alarms |
| `k` | **KEY REL** | Key release — hand DSKY back to the background program |

### Simulator controls

| Key | Function |
|---|---|
| `F1` | Load Launch Monitor scenario |
| `F2` | Load Targeted Burn scenario |
| `F3` | Load Free Flight scenario |
| `+` / `=` | Double the time factor (max ×512 cycles per render frame) |
| `-` | Halve the time factor (min ×1) |
| `q` / `Esc` | Quit |

## Implemented Verbs

The PINBALL dispatcher (`command_dispatch.rs` + `agc_core::services::pinball`)
currently honours:

| Verb | Meaning | Effect in agc-sim |
|---|---|---|
| **V06 Nxx** | Display decimal data for noun xx | Latches the noun; the R1/R2/R3 fields update every SERVICER cycle |
| **V16 Nxx** | Monitor decimal (same as V06 but continuous) | Same as V06 in this sim |
| **V34** | Terminate running program | Drops back to P00, cancels any active burn |
| **V35** | Lamp test | All DSKY lights on for ~5 seconds |
| **V37 Nxx** | Change major mode → program P*xx* | Switches to the program; V37N40 auto-arms a 50 m/s prograde burn |

Other verbs log `not implemented` and raise OPR ERR.

## Implemented Nouns

Populated by `noun_display.rs` on every frame from the live state vector:

| Noun | R1 | R2 | R3 | AGC Scaling |
|---|---|---|---|---|
| **N00** | blank | blank | blank | — |
| **N33** | TIG hours | TIG minutes | TIG seconds | MET + 60 s (demo) |
| **N36** | MET hours | MET minutes | MET seconds | from `StateVector::t` |
| **N44** | apoapsis (×0.1 km) | periapsis (×0.1 km) | orbital period (×0.01 s) | from `rv_to_elements` |
| **N62** | inertial velocity (×0.1 m/s) | altitude rate h-dot (×0.1 m/s) | altitude (m) | from `StateVector` |
| **N85** | VG body-frame X (×0.1 m/s) | VG body-frame Y | VG body-frame Z | from `BurnState` |

All other nouns display blank R1/R2/R3.

---

## Six Scenarios You Can Execute

Each scenario assumes you start from `--scenario free` or `--scenario burn` unless
noted. In the real Apollo documentation a verb-noun command is written as
**V37N40E** where `E` means ENTR.

---

### Scenario 1 — Lamp Test (V35)

**Aim:** prove the DSKY is alive before trusting any flight-critical display.
On real Apollo missions this was the first thing the Commander did when taking
control of the computer.

**Keys to press**
| Press | DSKY key | Meaning |
|---|---|---|
| `v` | VERB | begin verb entry |
| `3` | 3 | first digit |
| `5` | 5 | second digit — verb = **35** |
| `Enter` | ENTR | execute |

**Verb meaning:** V35 = **Test Lights** (from the standard AGC verb table).
It has no noun because it operates on the DSKY hardware, not on data.

**Outcome**
- All 10 status lights (UPLNK, TEMP, GBL, NOATT, STBY, PROG, KREL, RSTR, OPER, TRKR) turn **on** for ~5 seconds
- COMP ACTY is forced on
- Mission log: `V35 — LAMP TEST`
- After ~5 seconds every light returns to its prior (off) state

**Before / After**
| | Before | After |
|---|---|---|
| Indicator lights | all off (dim) | all on (reversed) → back to off |
| Program | P00 (unchanged) | P00 (unchanged) |
| State vector | unchanged | unchanged |

---

### Scenario 2 — Read Orbital Parameters (V06N44)

**Aim:** display the spacecraft's current apoapsis, periapsis, and orbital period
without changing the mission state. This is the exact command Neil Armstrong used
to verify parking orbit after insertion.

**Keys to press**
| Press | DSKY key | Meaning |
|---|---|---|
| `v` | VERB | begin verb entry |
| `0` | 0 | first digit |
| `6` | 6 | second digit — verb = **06** (display decimal) |
| `n` | NOUN | begin noun entry |
| `4` | 4 | first digit |
| `4` | 4 | second digit — noun = **44** (apoapsis/periapsis/period) |
| `Enter` | ENTR | execute |

**Verb/Noun meaning:** V06 = **Display decimal** on the three data registers.
N44 = **Apoapsis altitude / Periapsis altitude / Time of free fall** in the
AGC book's Appendix J Command Module noun table.

**Outcome**
- PROG stays at 00; VERB shows 06; NOUN shows 44
- R1 fills with apoapsis altitude in tenths of a km (e.g. `+02000` = 200.0 km)
- R2 fills with periapsis altitude
- R3 fills with orbital period in tenths of a second
- The values update every 2-second SERVICER cycle as the orbit evolves
- Mission log: `V06N44 — display/monitor`

**Before / After**
| | Before | After |
|---|---|---|
| R1 / R2 / R3 | blank | apoapsis / periapsis / period |
| Program | P00 | P00 |
| Orbit | 200 km circular | unchanged — this is a read-only command |

Use `+` to accelerate time and watch R3 (orbital period) stay constant while
R1 and R2 remain identical (the orbit is circular).

---

### Scenario 3 — Live Velocity Readout (V16N62)

**Aim:** set up a **monitor** that continuously updates velocity, altitude rate,
and altitude. On real Apollo this is Fig. 79 cue card — what the crew scanned
every few seconds during launch.

**Keys to press**
| Press | DSKY key | Meaning |
|---|---|---|
| `v` | VERB | begin verb entry |
| `1` | 1 | |
| `6` | 6 | verb = **16** (monitor decimal) |
| `n` | NOUN | begin noun entry |
| `6` | 6 | |
| `2` | 2 | noun = **62** (V/h-dot/h) |
| `Enter` | ENTR | execute |

**Verb/Noun meaning:** V16 = **Monitor decimal** — like V06 but the AGC keeps
updating the display continuously (no need to re-press ENTR). N62 = **Inertial
velocity / altitude rate / altitude above pad**.

**Outcome**
- VERB = 16, NOUN = 62
- R1 shows inertial speed ×10 (e.g. `+77843` = 7,784.3 m/s)
- R2 shows altitude rate h-dot (for a circular orbit this is ~0)
- R3 shows altitude in metres (e.g. `+00204662` → truncated to 5 digits)
- Mission log: `V16N62 — display/monitor`

**Before / After**
| | Before | After |
|---|---|---|
| R1 / R2 / R3 | blank (or previous display) | velocity / h-dot / altitude live |
| Program | P00 | P00 |
| Orbit | unchanged | unchanged — read-only |

Press `+` a few times to fast-forward the orbit. R1 stays near 7,784 m/s but
R2 oscillates as the spacecraft approaches and recedes from Earth centre —
the subtle signature of a slightly elliptical numeric integration orbit.

---

### Scenario 4 — Execute a 50 m/s Prograde SPS Burn (V37N40)

**Aim:** change orbit. This is the marquee presentation — watch the Apollo SPS
engine fire, change the orbit in real time, and see apoapsis rise.

**Start** with `cargo run -p agc-sim --bin agc_sim -- --scenario burn` (or press F2).

**Keys to press**
| Press | DSKY key | Meaning |
|---|---|---|
| `v` | VERB | begin verb entry |
| `3` | 3 | |
| `7` | 7 | verb = **37** (change major mode) |
| `n` | NOUN | begin noun entry |
| `4` | 4 | |
| `0` | 0 | noun = **40** (program number 40) |
| `Enter` | ENTR | execute |

**Verb/Noun meaning:** V37 is the single most important verb on the DSKY —
**"change major mode"**. The noun is the program number you want to switch to.
N40 = P40 = **SPS Thrust**. This demo shortcut auto-loads a 50 m/s prograde
burn at the moment you press ENTR (see `command_dispatch.rs` line 53).

**Outcome step-by-step**
1. PROG → 40, VERB → 37, NOUN → 40
2. DAP transitions from `[FREE]` to `[BURN]` (reversed video, bold)
3. ENG indicator lights up `[● FIRING]`
4. VG starts at `+50.00 m/s` and counts down
5. ΣΔV climbs from 0 toward +50 m/s
6. Trajectory X/Y/Z velocity components start changing
7. SMA grows (energy is being added)
8. APO rises — watch it climb from +200 km toward +305 km
9. PER stays ~constant (burn is at periapsis → apoapsis only rises)
10. After ~11 seconds of sim time VG reaches 0, `BURN complete` is logged, program returns to P00

Speed this up by pressing `+` a few times before or during the burn.

**Before / After**
| | Before | After |
|---|---|---|
| PROG | 00 | 00 (burn finished) |
| DAP | FREE | FREE (burn finished) |
| SMA | 6578.0 km | ~6625 km |
| ECC | 0.0000 | ~0.0072 (now slightly elliptical) |
| APO | +200.0 km | ~+305 km |
| PER | +200.0 km | ~+200 km |
| ΣΔV | 0 | +50.00 m/s |
| Kinetic energy | baseline | +392 kJ/kg (per unit mass) |

**AGC-authentic details**
- SPS thrust: **91,188.544 N** (AGC `FENG 2DEC 9.1188544 B-7`)
- Exhaust velocity: **3,151.04 m/s** (AGC `2VEXHUST 2DEC 63.020792 B-7`)
- PIPA scale: **0.0585 m/s per count** (AGC `KPIP1 2DEC 0.074880`)
- Integration: **Störmer-Verlet predictor-corrector** (SERVICER207.agc CALCRVG)
- Position-update uses gravity from the **previous** cycle; velocity corrector uses fresh gravity at the predicted position

---

### Scenario 5 — Abort a Running Program (V34)

**Aim:** show the crew's emergency stop. V34 is the "terminate" verb — it
drops any running program back to P00 and cancels whatever guidance computation
was in progress. Useful if the crew notices a problem mid-burn.

**Setup:** start the burn from Scenario 4 and, **while the burn is active**,
execute this command.

**Keys to press**
| Press | DSKY key | Meaning |
|---|---|---|
| `v` | VERB | begin verb entry |
| `3` | 3 | |
| `4` | 4 | verb = **34** (terminate) |
| `Enter` | ENTR | execute — V34 takes no noun |

**Verb meaning:** V34 = **Terminate** — the "abort current program" command.
One of the special verbs that operate without a noun (V32 recycle, V33 proceed,
V34 terminate, V35 lamp test, V36 fresh start).

**Outcome**
- Mission log: `V34 — terminating program P40`
- Active program drops to P00
- Burn state is cleared — engine cuts off
- DAP returns to FREE, ENG returns to OFF
- Whatever ΣΔV was accumulated stays (the real burn that happened)
- The orbit is now in whatever intermediate state the burn left it

**Before / After (if you abort halfway through a 50 m/s burn)**
| | Before V34 | After V34 |
|---|---|---|
| PROG | 40 | 00 |
| DAP | BURN | FREE |
| ENG | FIRING | OFF |
| VG | +22.64 m/s (counting down) | `----` (no active burn) |
| ΣΔV | +27.36 m/s | +27.36 m/s (frozen) |
| SMA | rising | frozen at partial value |
| APO | rising | frozen at ~+255 km |

The partial burn is physically real — the spacecraft is now in an orbit it
wouldn't otherwise have occupied.

---

### Scenario 6 — Two-Burn Maneuver: Raise and Circularize

**Aim:** combine two burns to raise both apoapsis and periapsis — the classic
Hohmann transfer departure and arrival. This proves the AGC can execute a
sequence, not just a single burn.

**Keys to press**

First burn (at the original 200 km periapsis — raises apoapsis):
| Press | DSKY key | Meaning |
|---|---|---|
| `v` `3` `7` `n` `4` `0` `Enter` | V37N40 | execute 50 m/s prograde → apoapsis rises to ~305 km |

Wait until `BURN complete` appears in the log. Then press `+` a few times to
fast-forward half an orbit (until you can see ORB progress ~50 %).

Second burn (now at the new apoapsis — raises periapsis to match):
| Press | DSKY key | Meaning |
|---|---|---|
| `v` `3` `7` `n` `4` `0` `Enter` | V37N40 | execute another 50 m/s prograde |

**Why this works:** the first burn is at periapsis and raises apoapsis
(turning the circular orbit into an ellipse). The second burn, at apoapsis,
raises periapsis up to meet the apoapsis, producing a new (approximately)
circular orbit at the higher altitude. This is exactly how Apollo reached
higher parking orbits and how the LM did its rendezvous phasing.

**Outcome**
- After **burn 1**: ECC rises from 0 to ~0.007, APO ~305 km, PER ~200 km
- After **coasting half an orbit**: the spacecraft is at the new apoapsis
- After **burn 2**: ECC drops back toward zero, APO ~305 km, PER also ~305 km
- ΣΔV total: +100 m/s
- Program returns to P00 twice

**Before / After the full maneuver**
| | Before | After burn 1 | After burn 2 |
|---|---|---|---|
| SMA | 6578.0 km | 6625 km | 6678 km |
| ECC | 0.0000 | 0.0072 | ~0.001 |
| APO | +200 km | +305 km | +305 km |
| PER | +200 km | +200 km | +305 km |
| ΣΔV | 0 | +50 m/s | +100 m/s |
| Orbital period | 5309 s | 5376 s | 5442 s |

This is a **real Hohmann transfer** computed live by the AGC — not a
scripted demo. The physics is correct, the energy bookkeeping is correct,
and the orbital-elements output comes straight from `rv_to_elements()` in
`navigation/conics.rs`.

---

## Presentation Playbook

Suggested order for a 10-minute live demo:

1. **Open `--scenario free`** — point at the MET ticking. "This is a 200 km LEO. Every 2 seconds of that clock is one SERVICER cycle, the same cycle Apollo ran."
2. **Scenario 1: V35 lamp test** — "First thing any Apollo crew did. Prove the DSKY works."
3. **Scenario 2: V06N44** — "Now I ask the AGC what orbit we're in." Point at APO/PER.
4. **Scenario 3: V16N62** — "This is what Neil Armstrong watched during launch. Velocity, altitude rate, altitude." Press `+` to run the orbit.
5. **Switch to `--scenario burn` (F2)** — "Now let me actually change the orbit."
6. **Scenario 4: V37N40** — slow, key by key. "V — 3 — 7 — N — 4 — 0 — Enter." Watch DAP turn to BURN, VG count down, APO rise. This is the moment.
7. **Scenario 5: V34 mid-burn (optional)** — "And if I don't like what's happening I can abort." Show the partial burn freezing.
8. **Scenario 6: Hohmann transfer (advanced)** — "Two burns, half an orbit apart, and we've just done a Hohmann transfer — the same maneuver the LM used to rendezvous with the CM."

At every step, point at the mission log. Every entry is a real AGC event
logged by the same code paths that would run on bare-metal hardware.

---

## AGC Authenticity

Every constant in the simulator is locked to the Comanche055 assembler source:

| Constant | AGC value | Source file |
|---|---|---|
| `MU_EARTH` | 3.986032×10¹⁴ m³/s² | `ORBITAL_INTEGRATION.agc` |
| `RE_EARTH` (ERAD) | 6,373,338 m (Fischer ellipsoid) | `LATITUDE_LONGITUDE_SUBROUTINES.agc` |
| `PIPA_SCALE` (KPIP1) | 0.0585 m/s per count | `SERVICER207.agc` |
| `CYCLE_DT` | 2.0 s | `SERVICER207.agc` |
| `SPS_THRUST_N` (FENG) | 91,188.544 N | `P40-P47.agc` |
| `SPS_VE_MS` (2VEXHUST) | 3,151.04 m/s | `P40-P47.agc` |
| DSKY key codes | 18 keys, octal-encoded | `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` |

The full catalogue lives in [`../docs/agc-reference-constants.md`](../docs/agc-reference-constants.md).
Regression tests in [`../agc-core/src/tests/agc_constants.rs`](../agc-core/src/tests/agc_constants.rs) lock these values so any drift fails CI.

---

## Module Layout

| Module | Purpose |
|---|---|
| `mission` | `Mission` + `Scenario` enum, MET formatter, phase labels |
| `noun_display` | Noun → R1/R2/R3 mapping (N00, N33, N36, N44, N62, N85) |
| `command_dispatch` | V/N → mission action (V35 lamp test, V37 program change, V34 terminate) |
| `unified_terminal` | Three-panel ratatui renderer |
| `hardware` | `SimHardware` — full `AgcHardware` impl, safe for unit tests |
| `dsky_state` | `DskyDisplayState` — decoded PROG/VERB/NOUN, registers, lights |
| `sim_log` | `SimLog` — ring buffer used by the mission log panel |
| `bin/agc_sim` | The single executable |

## Using `SimHardware` headless (for tests)

`SimHardware::new()` creates a headless instance with no terminal attached —
safe for CI unit and scenario tests.

```rust
use agc_sim::SimHardware;
use agc_core::AgcState;

let mut hw = SimHardware::new();
let mut state = AgcState::new();

// Inject a PIPA pulse and check the log.
hw.inject_pipa(10, 0, 0);
assert!(hw.log.tail(1)[0].message.contains("PIPA"));
```

The TUI (`unified_terminal`) is only attached by the `agc_sim` binary, so
tests run cleanly in CI without requiring a terminal.
