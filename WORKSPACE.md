# Workspace Setup

## Essential Tools

```sh
# Rust targets
rustup target add thumbv7em-none-eabihf   # Cortex-M4F/M7F bare-metal (hard float)

# Cargo tools
cargo install cargo-watch      # auto-recompile on save
cargo install cargo-nextest    # faster test runner
cargo install cargo-audit      # security advisories
cargo install cargo-outdated   # stale dependency check
cargo install cargo-expand     # macro expansion debug
cargo install flip-link        # stack overflow detection at link time

# Embedded tooling
cargo install probe-rs --features cli   # flash, debug, RTT logging over probe
```

## Daily Commands

```sh
# Continuous feedback on host
cargo watch -x check -x test

# Pre-commit gate (must be clean before pushing)
cargo fmt && cargo clippy -- -D warnings && cargo test

# Verify the no_std bare-metal build has not broken
cargo build --target thumbv7em-none-eabihf -p agc-core

# Fast test run
cargo nextest run

# Security check
cargo audit
```

## VS Code

Enable `rust-analyzer.checkOnSave.command = "clippy"` and `editor.formatOnSave = true`.

For embedded debugging, install the **Cortex-Debug** extension and configure `probe-rs` as the GDB server.

## Key Documentation

| File | Purpose |
|---|---|
| `AGENTS.md` | Coding conventions and embedded rules |
| `docs/architecture.md` | Full software architecture and design decisions |
| `docs/testing.md` | VirtualAGC integration test strategy |
| `docs/optimization.md` | Rust Embedded Book compliance gaps |
| `transformation/` | Progress, tasks, ADRs, blockers |
| `specs/` | Component spec templates and filled specs |

## Embedded Build Notes

- Target: `thumbv7em-none-eabihf` (Cortex-M4F, hard-float ABI)
- Soft-float (`thumbv7em-none-eabi`) is **not** acceptable — the DAP timing budget requires hardware FPU
- `flip-link` is used to detect stack overflows at link time (moves stack to bottom of RAM so overflow causes a clean fault)
- `probe-rs run` is used for hardware-in-the-loop testing; it flashes, runs, and streams RTT log output

## Cargo.toml Profile Notes

```toml
[profile.release]
opt-level = "s"      # optimise for size on embedded
lto = true
codegen-units = 1

[profile.dev]
# Leave debug info on for probe-rs / Cortex-Debug
debug = true
```
