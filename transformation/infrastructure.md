# Infrastructure

## Workspace Structure

```
agc-in-rust/                     (workspace root)
  Cargo.toml                     (workspace definition)
  agc-core/                      (flight software — #![no_std], #![no_main])
  agc-sim/                       (host-side simulator — std allowed)
  agc-test/                      (integration test harness)
```

Full module tree: see `docs/architecture.md §2`.

## Dependencies (agc-core)

| Crate | Version | Purpose |
|-------|---------|---------|
| `cortex-m` | 0.7 | Cortex-M primitives: `interrupt::free`, `Mutex`, SysTick |
| `cortex-m-rt` | 0.7 | Startup, reset handler, `#[entry]`, `#[exception]` |
| `embedded-hal` | 1.0 | Trait abstractions for SPI/I2C/GPIO/UART |
| `stm32f4` | latest | Device PAC — `#[interrupt]` with compile-time name verification; re-exported as `pac` |
| `cortex-m-semihosting` | latest | Debug logging to host via probe (dev builds only) |
| `defmt` | latest | Structured logging over probe (dev builds only) |

**Explicitly excluded**: `alloc`, `Vec`, `Box`, `panic-halt`, any RTOS crate.

## Dependencies (agc-sim)

| Crate | Purpose |
|-------|---------|
| `agc-core` (with `sim` feature) | Flight software under test |
| Standard library | Unrestricted in simulator |
| `crossterm` or similar | Terminal-based DSKY simulator |

## Dependencies (agc-test)

| Crate | Purpose |
|-------|---------|
| `agc-core` | Under test |
| `agc-sim` | Provides simulated HAL |
| `serde` / `serde_json` | Fixture file parsing |

## Feature Flags

```toml
[features]
default = ["sim"]
sim = ["std"]        # host simulation — std allowed
bare-metal = []      # no std, no heap, hardware target
```

`agc-core` is always `#![no_std]`. The `sim` feature enables `agc-sim` to link against it.

## Build Targets

| Target | Purpose |
|--------|---------|
| `x86_64` (host) | Development, CI, unit tests |
| `thumbv7em-none-eabihf` | Bare-metal Cortex-M4F/M7F (hard-float ABI) |

The bare-metal build must always be clean. Add to CI:

```sh
cargo build --target thumbv7em-none-eabihf -p agc-core
```

## CI Checks

```sh
cargo fmt -- --check
cargo clippy -- -D warnings
cargo nextest run
cargo build --target thumbv7em-none-eabihf -p agc-core
cargo audit
```

## Hardware-in-the-Loop

- Board: STM32F405 (or STM32F7 — see ADR-011)
- Tooling: `probe-rs run` for flash + RTT log streaming
- Stack depth measurement: watermark pattern, checked in integration tests
- Linker: `flip-link` for stack overflow detection at link time

## Linker Script

```
MEMORY {
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}
```

`AgcState` is placed in a named RAM section; linker verifies it fits.
