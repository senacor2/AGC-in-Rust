# Hardware Bill of Materials

Reference list for building the bench setup that runs the bare-metal
AGC firmware (`agc-board-nucleo-f767`) plus the Pico stub bridge
(`agc-bridge-pico`).

Prices are typical retail in EUR (early 2026, German retailers — Reichelt,
BerryBase, Eckstein, Mouser DE). Add 10–30 % for boutique retailers like
Adafruit; subtract 30–50 % for direct-from-China.

> Status: planning only — **no order placed**. Update prices and
> suppliers when ordering.

---

## Essential

Required to power on, flash, and exercise the AGC end-to-end with the
stub bridge.

| # | Part | Notes | ~€ |
|---|---|---|---|
| 1 | **STM32 Nucleo-F767ZI** (`NUCLEO-F767ZI`) | The AGC. Cortex-M7 @ 216 MHz **with hardware double-precision FPU**, 2 MB flash, 512 KB RAM. On-board ST-LINK/V2-1 over a single USB micro-B — no separate debugger needed. Pre-soldered morpho headers. Same MB1137 carrier as the F722ZE so all pinouts/solder bridges from UM1974 carry over. ADR-021 (supersedes ADR-011). | 33 |
| 2 | **Raspberry Pi Pico** with pre-soldered headers ("Pico H") | The bridge MCU running `agc-bridge-pico`. RP2040 only (Cortex-M0+); the Pico 2 (RP2350) is a different chip and not supported by our firmware. ADR-015. | 6 |
| 3 | **Adafruit BMI088 breakout (#4836)** | Local IMU. 6-axis strapdown, 3.3 V regulator on board (5 V tolerant), 2.54 mm headers — breadboard-ready. ADR-016. | 17 |
| 4 | USB-A↔micro-B cable (Nucleo) | Often included with the Nucleo-F767ZI; verify before ordering. | 3 |
| 5 | USB-A↔micro-B cable (Pico) | Data-capable, not a charge-only cable. | 3 |
| 6 | Half-size breadboard (≥400 tie points) | Or perfboard if you prefer permanent wiring. | 6 |
| 7 | Dupont jumper wire kit | 40 each of M-M, M-F, F-F, mixed lengths. ~12 wires used in the bench setup. | 6 |
| | **Subtotal** | | **~€69** |

### Wiring summary (11 wires + grounds)

```
Nucleo-F767ZI                      Adafruit BMI088 (#4836)
─────────────                      ───────────────────────
PB3   (SPI3 SCK,    CN7-15)  ───►  SCL
PB4   (SPI3 MISO,   CN7-19)  ◄──   SDO   (combined SDO1/SDO2 on the board)
PB5   (SPI3 MOSI,   CN7-13)  ───►  SDA
PA15  (CS_ACCEL,    CN7-17)  ───►  CS    (accelerometer CS)
PB12  (CS_GYRO,     CN10-16) ───►  CSG   (gyroscope CS)
3V3   (CN8-7)                ───►  VIN   (3.3 V; module also accepts 5 V)
GND   (CN8-11)               ───   GND

Nucleo-F767ZI                      Raspberry Pi Pico
─────────────                      ─────────────────
PC6   (USART6 TX,   CN7-1)   ───►  GP1   (UART0 RX)
PC7   (USART6 RX,   CN7-11)  ◄──   GP0   (UART0 TX)
GND                          ───   GND
```

The Adafruit board labels MISO as `SDO`; both gyro and accel SDO pins
are tied together on the breakout, matching the SPI 3-wire convention
expected by `agc-board-nucleo-f767/src/local/imu/bmi088.rs`. The
accelerometer needs a CS toggle on first access to enter SPI mode — the
driver handles this in `Bmi088Driver::init`.

---

## Strongly recommended — quality of life

| # | Part | Why | ~€ |
|---|---|---|---|
| 8 | **Second Raspberry Pi Pico** flashed as **Picoprobe** | Acts as an SWD debugger for the bridge Pico, enabling `cargo run -p agc-bridge-pico` via `probe-rs`. Without it: UF2 drag-and-drop in BOOTSEL mode (works, slower). Three wires from Picoprobe GP2/GP3/GND → bridge-Pico SWCLK/SWDIO/GND. | 6 |
| 9 | Cheap 8-channel logic analyzer (24 MHz Saleae clone) | Drops the difficulty of debugging UART/SPI by an order of magnitude. Works with PulseView (open-source) and decodes UART, SPI, I²C natively. | 12 |
| 10 | Multimeter | Continuity checks save hours when wiring goes wrong. | 15 |
| | **Subtotal** | | **~€33** |

---

## Optional — Phase 7 physical DSKY (deferred)

Not required for the current state of the project. Listed only so you
know what's coming if you decide to build a real DSKY later.

| Part | Use | ~€ |
|---|---|---|
| 6× MAX7219 4-digit 7-segment LED modules (cascaded SPI) | DSKY display rows | 18 |
| 4×5 momentary tactile-button keypad (or 19 individual buttons) | DSKY keyboard | 5 |
| 2× 74HC595 shift registers + 14 LEDs | Indicator-lamp panel | 5 |
| Perfboard or custom PCB | Mounting | 10–25 |
| Total | | **~€38–53** |

---

## Total budget snapshots

- **Bare minimum to run the current firmware**: ~€69 (essentials only,
  UF2 flashing for the bridge, no logic analyzer).
- **Comfortable bench setup**: ~€102 (essentials + Picoprobe + cheap
  logic analyzer + multimeter).
- **Full kit including Phase 7 DSKY**: ~€140–155 (when you decide to
  build it).

---

## BMI088 module alternatives (for reference)

If you switch suppliers, the Rust driver code is unchanged — same
silicon. Only the connector and CS pin labels differ.

| Module | Connector | Price | Notes |
|---|---|---|---|
| **Adafruit BMI088 (#4836)** ✅ | 2.54 mm | ~€17 | Bench reference. On-board regulator. |
| Bosch shuttle board 3.0 BMI088 | 1.27 mm | ~€32 | Authoritative datasheet pinout. Needs a 1.27→2.54 mm adapter. |
| Generic AliExpress BMI088 breakout | 2.54 mm | ~€8 | Cheapest; verify silkscreen against the Bosch silicon datasheet before wiring. |
