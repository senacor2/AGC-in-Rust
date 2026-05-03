# agc-bridge-pico

RP2040 (Raspberry Pi Pico) bridge firmware for AGC-in-Rust.

Sits between the AGC (Nucleo-F767ZI running `agc-board-nucleo-f767`) and the
absent physical peripherals (DSKY, sextant/optics, engines, RCS).  Phase 4
scope: stub bridge that:

- Speaks the AGC wire protocol (`agc-protocol`) over UART0 at 460800 baud.
- Performs the Hello/HelloAck handshake on startup.
- Sends a `BridgeHeartbeat` every 200 ms (LED toggles as a visible indicator).
- Sends a synthetic `OpticsCdu` stream every 10 ms (slowly drifting angles).
- Exposes a USB-CDC serial console: keystrokes typed on the host are forwarded
  as DSKY key codes; decoded AGC messages are printed as one-line text.

---

## Wiring

```
Nucleo-F767ZI               Raspberry Pi Pico
─────────────────           ─────────────────
PC6 (USART6 TX) ──────────> GPIO1 (UART0 RX)
PC7 (USART6 RX) <────────── GPIO0 (UART0 TX)
GND             ──────────── GND
```

Cross-connect: AGC TX → Pico RX, AGC RX → Pico TX, common ground.

---

## Flash methods

### Via SWD (probe-rs)

```sh
cargo run -p agc-bridge-pico
```

Requires a debug probe (e.g. Raspberry Pi Debug Probe, J-Link, or a second
Pico running `picoprobe`) connected to the Pico's SWD pins.

### Via USB drag-and-drop (UF2)

1. Hold BOOTSEL while plugging in the Pico USB cable.
2. Build the ELF and convert to UF2:
   ```sh
   cargo build --release --target thumbv6m-none-eabi -p agc-bridge-pico
   elf2uf2-rs \
     target/thumbv6m-none-eabi/release/agc-bridge \
     agc-bridge.uf2
   ```
3. Copy `agc-bridge.uf2` to the `RPI-RP2` mass storage drive.

Install `elf2uf2-rs`:
```sh
cargo install elf2uf2-rs
```

---

## USB console keystroke mapping

Connect a serial terminal (e.g. `picocom -b 115200 /dev/tty.usbmodemXXXX`)
to the Pico's USB CDC port.

| Key typed | DSKY function | AGC code |
|-----------|---------------|----------|
| `0`–`9`   | Digit         | 16 / 1–9 |
| `v` / `V` | VERB          | 17       |
| `n` / `N` | NOUN          | 31       |
| `+`       | PLUS          | 26       |
| `-`       | MINUS         | 27       |
| Enter     | ENTR          | 28       |
| `c` / `C` | CLR           | 30       |
| `p` / `P` | PRO           | 25       |
| `k` / `K` | KEYREL        | 25       |
| `r` / `R` | RSET          | 18       |

Code values match `agc_core::services::v_n::Key::from_code` (KEYTEMP1 table,
`Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`).

---

## Expected USB console output

```
AGC bridge starting...
handshake OK
AGC> AGC_HEARTBEAT met=100cs
AGC> DSKY_WRITE_ROW row=10 data=0x1F3
AGC> ENGINE_SPS_GIMBAL pitch=0 yaw=120
```
