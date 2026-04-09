# agc-sim

Simulated hardware implementation of the AGC HAL (`AgcHardware`) for host-side
testing and interactive simulation. Includes a terminal-based DSKY display
driven by [ratatui](https://ratatui.rs) and [crossterm](https://github.com/crossterm-rs/crossterm).

## Running the interactive demos

```sh
# DSKY keyboard / display demo
cargo run -p agc-sim --bin dsky_demo

# SERVICER / AVERAGE G navigation demo (Milestone 2)
cargo run -p agc-sim --bin nav_demo
```

---

## TUI Layout

The terminal is split into two panels side by side.

```
╔═ APOLLO GUIDANCE COMPUTER — DSKY ═════════════╦═ STATE LOG ═══════════════════╗
║  ● UPLINK  ○ TEMP  ○ GIMBAL LOCK              ║   0 INFO  SimHardware init    ║
║  ○ PROG    ○ KEY REL  ○ OPR ERR  ● COMP       ║   1 INFO  P00 — CMC IDLE      ║
║                                               ║   2 I/O   KEY → Verb          ║
║  PROG          VERB          NOUN             ║   3 I/O   VERB digit: 3       ║
║  [ 00 ]        [ 37 ]        [ 00 ]           ║   4 I/O   VERB digit: 7       ║
║                                               ║   5 INFO  VERB set to 37      ║
║  R1  +00000                                   ║   6 ALARM ALARM 01202         ║
║  R2  +00000                                   ║                               ║
║  R3  +00000                                   ║                               ║
║                                               ║                               ║
║  [V]erb [N]oun [+] [-] [0-9] [Del]Clear       ║                               ║
║  [P]roceed [Enter] [R]eset [K]eyRel | [Q]uit  ║                               ║
╚═══════════════════════════════════════════════╩═══════════════════════════════╝
```

### Left panel — DSKY

| Section | Content |
|---|---|
| **Indicator lights** (rows 1–2) | Lit yellow when active, dark grey when off. Lights: UPLINK ACTY, TEMP, GIMBAL LOCK, PROG ALARM, KEY REL, OPR ERR, COMP ACTY |
| **PROG / VERB / NOUN** | Two-digit displays for major mode, verb, and noun. Blank (`  `) when the AGC has not written a value yet |
| **R1 / R2 / R3** | Five-digit signed data registers. Sign is `+` or `-`; blank digits shown as spaces |
| **Keyboard hint** | PC key bindings for every DSKY key |

### Right panel — State log

A scrolling ring buffer (last 200 entries) of state-change events emitted by
the simulation. Each line shows a monotonic tick counter, a severity label, and
a message.

| Label | Colour | Meaning |
|---|---|---|
| `INFO ` | Grey | General simulator events |
| `WARN ` | Yellow | Non-fatal conditions |
| `ALARM` | Red | Program alarms (1202, 1210, …) |
| `I/O  ` | Cyan | DSKY relay writes, key presses, PIPA injections |

---

## Navigation Demo (`nav_demo`)

Visualises the Milestone 2 navigation pipeline. Starts a 200 km LEO circular
orbit and runs the SERVICER (AVERAGE G) cycle continuously, updating the
display every render frame.

The integration algorithm faithfully reproduces the AGC's CALCRVG/CALCGRAV
predictor-corrector sequence from SERVICER207.agc:

1. **Position predictor** (CALCRVG): `r_new = r + (v + dv/2 + GDT_old/2) * dt`
   where `GDT_old/2` is half the gravitational impulse from the **previous** cycle
   (Störmer-Verlet / leapfrog scheme).
2. **Gravity evaluation** (CALCGRAV): fresh `point_mass(r_new)` at predicted position.
3. **Velocity corrector**: `v_new = v + dv + a_new * dt`.
4. **State carry-over**: `GDT_new/2 = a_new * dt/2` saved for next cycle.

Altitude is computed using the AGC's Fischer ellipsoid pad radius
`ERAD = 6,373,338 m` (from LATITUDE_LONGITUDE_SUBROUTINES.agc), not
the WGS84 mean radius (6,371 km).

```
╔═ SERVICER — AVERAGE G NAVIGATION ══════════╦═ STATE LOG ════════════════════╗
║ MET +000000s.00  cycle 0                   ║   0 INFO  AGC-in-Rust SERVICER ║
║                                            ║   1 INFO  LEO 200 km  v=7784   ║
║    POSITION (km)    VELOCITY (m/s)         ║   2 INFO  T=5309 s             ║
║ X    +6578.000        +0.000               ║   3 I/O   PIPA +Y 100 (+5.85)  ║
║ Y       +0.000     +7784.268               ║   4 I/O   cycle 1 ΔV=5.850 m/s ║
║ Z       +0.000        +0.000               ║                                ║
║                                            ║                                ║
║ ALT   +204.662 km  SPD  7784.268 m/s       ║                                ║
║ ORB ░░░░░░░░░░░░░░░░░░░░  0.0%  T=5309s    ║                                ║
║                                            ║                                ║
║ PIPA [  +0   +0   +0]  ALARM -----         ║                                ║
║ ΣΔV    +5.850 m/s  (cumulative burns)      ║                                ║
║ TIME×   1  (cycles per frame)              ║                                ║
║                                            ║                                ║
║ X/x Y/y Z/z → ±PIPA (100 cnts = ±5.85 m/s)║                                ║
║ [+]/[-] time×  [C]lear PIPA  [Q]uit        ║                                ║
╚════════════════════════════════════════════╩════════════════════════════════╝
```

### Navigation keyboard map

| PC key | Action |
|---|---|
| `X` | Inject +100 PIPA counts on X axis (+5.85 m/s) |
| `x` | Inject −100 PIPA counts on X axis (−5.85 m/s) |
| `Y` | Inject +100 PIPA counts on Y axis (+5.85 m/s) |
| `y` | Inject −100 PIPA counts on Y axis (−5.85 m/s) |
| `Z` | Inject +100 PIPA counts on Z axis (+5.85 m/s) |
| `z` | Inject −100 PIPA counts on Z axis (−5.85 m/s) |
| `B` | Plan and execute a +50 m/s prograde SPS burn (DAP switches to BURN mode) |
| `+` / `=` | Double the time factor (max ×512 cycles/frame) |
| `-` | Halve the time factor (min ×1) |
| `C` | Clear all pending PIPA counts |
| `Q` / `Esc` | Quit |

**PIPA scale:** 1 count = 0.0585 m/s delta-V (AGC `KPIP1 2DEC 0.074880 # 1 PULSE = 5.85 CM/SEC`
from SERVICER207.agc). 100 counts = 5.85 m/s per press — large enough to produce a
visible orbit change. A prograde burn (inject `Y` at the initial position) raises
the apoapsis; a retrograde burn lowers it. The `ΣΔV` line tracks cumulative burns.
Use `+` to increase the time factor and watch the orbit evolve.

---

## Keyboard Map (DSKY demo)

| PC key | DSKY key | Function |
|---|---|---|
| `V` | VERB | Begin verb-number entry |
| `N` | NOUN | Begin noun-number entry |
| `0` – `9` | 0 – 9 | Digit entry |
| `+` | `+` | Plus sign |
| `-` | `–` | Minus sign |
| `Enter` | ENTR | Confirm entry |
| `Del` / `Backspace` | CLR | Clear current entry |
| `P` | PRO | Proceed (acknowledge / continue) |
| `R` | RSET | Reset — clears PROG/VERB/NOUN and all lights |
| `K` | KEY REL | Key release request |
| `Q` / `Esc` | — | Quit the simulator |

---

## Modules

| Module | Purpose |
|---|---|
| `hardware` | `SimHardware` — full `AgcHardware` impl; headless, safe for unit tests |
| `dsky_state` | `DskyDisplayState` — decoded PROG/VERB/NOUN, three data registers, 13 indicator lights |
| `sim_log` | `SimLog` — 200-entry ring buffer with INFO / WARN / ALARM / I·O levels |
| `dsky_terminal` | ratatui renderer for the DSKY panel; maps PC keys to `DskyKey` |
| `nav_terminal` | ratatui renderer for the navigation state panel (`NavSnapshot`) |

## Using `SimHardware` in tests

`SimHardware::new()` creates a headless instance suitable for unit and scenario
tests. No terminal is required.

```rust
use agc_sim::SimHardware;
use agc_core::AgcState;

let mut hw = SimHardware::new();
let mut state = AgcState::new();

// Inject a PIPA pulse and check the log.
hw.inject_pipa(10, 0, 0);
assert!(hw.log.tail(1)[0].message.contains("PIPA"));
```

Attach the TUI only in the interactive binary (`dsky_demo`); keep tests
headless so they run in CI without a terminal.
