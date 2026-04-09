---
name: workspace-maintenance
description: Use when maintaining the AGC-in-Rust workspace — Cargo.toml changes, feature flags, dependency hygiene, linting, formatting, bare-metal target configuration, probe-rs setup, or CI validation.
argument-hint: Describe the Cargo, workspace, dependency, or build maintenance task
---

# Rust Workspace Maintenance — AGC-in-Rust

## When To Use

- Add or adjust workspace members (`agc-core`, `agc-sim`, `agc-test`) or Cargo configuration
- Review dependency hygiene or feature flags
- Fix formatting, linting, or verification workflows
- Maintain embedded target setup (`thumbv7em-none-eabihf`, `probe-rs`, `flip-link`)
- Verify `no_std` compliance has not been broken by a dependency change
- Prepare CI checks

## Procedure

1. Inspect workspace structure and relevant `Cargo.toml` files before making changes.
2. Keep dependency changes minimal. Prefer existing crates and the standard library.
3. **Verify embedded target config**: `.cargo/config.toml` must specify `thumbv7em-none-eabihf` as the default target for `agc-core`, `probe-rs` as the runner, and `flip-link` as the linker wrapper.
4. Treat feature flags as additive. Avoid features that silently remove behavior.
5. `agc-core` is always `#![no_std]`. The `sim` feature enables `agc-sim` to link against it with `std`.
6. Validate with the narrowest command first, then broaden to workspace-level.

## Workspace Layout

```
agc-in-rust/
  Cargo.toml              workspace definition
  agc-core/               #![no_std], #![no_main] — flight software
  agc-sim/                std allowed — simulator + hosted HAL
  agc-test/               integration tests, VirtualAGC fixtures
  .cargo/config.toml      target triple, runner, linker
```

## Feature Flags

```toml
# agc-core/Cargo.toml
[features]
default = ["sim"]
sim = ["std"]        # host simulation
bare-metal = []      # no std, no heap, hardware target
```

## Required Dependencies (agc-core)

| Crate | Role | Note |
|---|---|---|
| `cortex-m` | `Mutex`, `interrupt::free`, SysTick | Required for shared state pattern |
| `cortex-m-rt` | `#[entry]`, `#[exception]` | Startup only; NOT the source of `#[interrupt]` |
| `embedded-hal` v1 | HAL trait abstractions | Used inside bare-metal HAL structs |
| `stm32f4` (or target PAC) | `#[interrupt]` with name verification | Re-exported as `pub use stm32f4 as pac` |
| `cortex-m-semihosting` | Dev-build panic logging | `#[cfg(debug_assertions)]` only |
| `defmt` | Structured probe logging | Dev builds only |

**Explicitly forbidden in agc-core**: `alloc`, `std`, `panic-halt`, any RTOS crate, `Vec`, `Box`.

## `.cargo/config.toml`

```toml
[build]
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip STM32F405RGTx"
rustflags = [
  "-C", "link-arg=-Tlink.x",
  "-C", "link-arg=-flip-link",   # stack overflow detection
]

[profile.release]
opt-level = "s"
lto = true
codegen-units = 1

[profile.dev]
debug = true
```

## Common Checks

```sh
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo build --target thumbv7em-none-eabihf -p agc-core   # bare-metal build must always pass
cargo doc --no-deps -p agc-core
cargo audit
cargo udeps --workspace                                   # when cleaning dependencies
```

## `no_std` Compliance Verification

After adding any new dependency to `agc-core`:

```sh
cargo build --target thumbv7em-none-eabihf -p agc-core --no-default-features
```

If this fails with `error[E0463]: can't find crate for 'std'` on a transitive dependency, that dependency must be pinned to its `default-features = false` variant.

## Delivery Checklist

- [ ] `agc-core` bare-metal build passes after changes
- [ ] No new `std`-only dependency added to `agc-core`
- [ ] No `alloc` or heap-allocating types introduced in `agc-core`
- [ ] Feature flags remain additive
- [ ] `cargo clippy -- -D warnings` clean across workspace
- [ ] `cargo audit` clean
- [ ] No `dbg!`, stray `hprintln!`, or commented-out code left in workspace
