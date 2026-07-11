<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Binary-Size Optimization Specification (Board Target)

**Issue:** [#95 — Shrink the size of the binary of the board target](https://github.com/senacor2/AGC-in-Rust/issues/95)
(parent: #99)

**Scope:** The `agc-board-nucleo-f767` binary (`agc`), built for
`thumbv7em-none-eabihf`, which statically links `agc-core` (the Comanche055 flight
software), `agc-protocol`, `agc-imu-platform`, and the STM32F7 HAL. This document
specifies *what* to change and *why*, with measured projections; it does **not** change
any code or configuration.

**Motivation:** The historical Comanche055 core rope occupied 36,864 fixed 15-bit words
(≈ 73,728 bytes as stored, ~34k words of program). We want the Rust port's on-target
footprint to shrink toward that figure so the two can be compared meaningfully, and to
prove the flight software fits comfortably in a small ROM budget. See
`docs/virtualagc-instruction-and-size-analysis.md` for the original-rope breakdown.

---

## 1. Acceptance criteria (from issue #95)

| # | Criterion | Baseline | Status after this spec |
|---|---|---|---|
| 1 | Code size < 128 kB (131,072 B) | **177,656 B** ❌ | **104,572 B** ✅ (opt=z + LTO) |
| 2 | Writable data < 4 kB (4,096 B) | **3,748 B** ✅ | 3,748 B ✅ (already met) |
| 3 | Program still compiles | ✅ | must re-verify |
| 4 | All tests pass | ✅ | must re-verify |
| 5 | No clippy errors/warnings | ✅ | must re-verify |

The dominant gap is criterion 1 (code size). Criterion 2 is already satisfied and is not
at risk from any change proposed here. The remaining work is (a) apply the build-profile
changes, then (b) verify criteria 3–5 still hold.

---

## 2. Baseline measurement (as of this spec)

Build: `cargo build -p agc-board-nucleo-f767 --target thumbv7em-none-eabihf --release`
with the current `[profile.release]` (`panic = "abort"`, `opt-level = "s"`,
`overflow-checks = false`, **no LTO**, default `codegen-units`).

Measured with `llvm-size` from the rustup toolchain
(`$(rustc --print sysroot)/lib/rustlib/*/bin/llvm-size`):

| Section | Bytes | Notes |
|---|---:|---|
| `.text` | 159,012 | executable code |
| `.rodata` | 18,140 | read-only constants (incl. ephemeris / star / noun tables) |
| `.vector_table` | 504 | Cortex-M exception vectors |
| `.data` (init image) | 2,600 | writable statics, dominated by `AGC_STATE` (2,160 B) |
| `.bss` | 124 | zero-init statics |
| `.uninit` | 1,024 | `MaybeUninit` statics (e.g. buffers) |
| **Flash footprint** (`text`+`rodata`+`vectors`+`data`) | **≈ 180,256** | what is programmed into ROM |
| **"Code size"** (Berkeley `text` = `.text`+`.rodata`+`.vector_table`) | **177,656** | the criterion-1 figure |
| **Writable RAM** (`.data`+`.bss`+`.uninit`) | **3,748** | the criterion-2 figure |

Definitions used throughout: **"code size"** = Berkeley `text` column = the read-only
image (`.text`+`.rodata`+`.vector_table`), matching how criterion 1 is stated.
**"Writable data"** = `.data`+`.bss`+`.uninit` = the RAM the program mutates.

### 2.1 Where the code size goes (top symbols, baseline)

`llvm-nm --print-size --size-sort` on the baseline ELF, largest contributors:

- **Float→decimal formatting machinery — ≈ 15 kB, and entirely removable:**
  `core::num::flt2dec::strategy::dragon::format_shortest` (5,348 B), `format_exact`
  (4,758 B), `grisu::format_shortest_opt` (2,080 B), `grisu::CACHED_POW10` (1,296 B),
  `do_count_chars` (1,258 B). This is the generic `{}`/`{:?}`-on-`f64` code pulled in
  transitively; **it is not needed on-target** and is dead once cross-crate inlining can
  see it is unreachable (see §3.1). This is the single largest low-value block.
- **`libm` transcendentals (legitimate, keep):** `pow` (3,008), `fmod` (2,772),
  `rem_pio2_large` (2,680), `rem_pio2` (1,328), `expm1` (1,376), etc. These implement the
  double-precision math the guidance/nav code genuinely needs.
- **Guidance/nav/targeting logic (legitimate, keep — these are the abstractions):**
  `lambert` (5,160), `conics::state_to_elements` (2,848), `planetary::moon_position_at_jd`
  (3,856), `kepler_step` (1,744), the P31/P32/P33 CSI/CDH solvers, entry guidance, etc.
- **HAL one-time cost (mostly unavoidable):** `stm32f7xx_hal::rcc::CFGR::freeze` (3,312) —
  clock-tree setup, runs once at boot.
- **Const data blobs:** two `.Lanon` rodata tables (2,880 + 2,400 B) — ephemeris / star /
  noun-table constants. Legitimate data; not a formatting artifact.

The takeaway: apart from the ~15 kB of float-formatting code, the baseline is *not* full
of obvious waste — most of `.text` is real guidance/nav math and `libm`. The largest
single win is therefore a **build-configuration** change that both removes the formatting
dead code and tightens code generation globally, not a source rewrite.

---

## 3. Recommended changes (tiered, measured)

### Tier 0 — Build profile (highest impact, zero code change) — **REQUIRED**

The current `[profile.release]` uses `opt-level = "s"` with **no LTO** and default
`codegen-units` (16), which prevents cross-crate dead-code elimination and inlining.
Enabling link-time optimization and single-unit codegen lets the linker drop the
float-formatting machinery (§2.1) and de-duplicate/​inline across crate boundaries.

Measured on this workstation (rustc 1.96.0, same source tree, changing **only** the
profile via `CARGO_PROFILE_RELEASE_*` env overrides — no files touched):

| Profile | `text` (code size) | Δ vs baseline | Meets <128 kB? |
|---|---:|---:|---|
| Baseline: `opt="s"`, no LTO, cgu=16 | 177,656 | — | ❌ |
| `opt="s"`, `lto="fat"`, `cgu=1` | 133,280 | −25.0% | ❌ (barely over) |
| **`opt="z"`, `lto="fat"`, `cgu=1`** | **104,572** | **−41.1%** | ✅ (25 kB headroom) |

After `lto="fat"`, the `flt2dec`/`grisu`/`dragon` symbols are **gone** from the image —
confirmed by `llvm-nm`. LTO made them visibly unreachable and the linker removed them.
`writable data` is unchanged by all three profiles (2,600 `.data` + 1,148 `.bss/.uninit`
= 3,748 B), so criterion 2 stays satisfied.

**Specified profile change** (to `[profile.release]` in the workspace `Cargo.toml`):

```toml
[profile.release]
panic         = "abort"   # already set
overflow-checks = false   # already set
opt-level     = "z"       # was "s"  — smallest code
lto           = "fat"     # NEW      — cross-crate DCE + inlining
codegen-units = 1         # NEW      — best size, single translation unit
strip         = true      # NEW      — drop symbols/debuginfo from the ELF (see note)
```

Notes and caveats:

- **`opt-level = "z"` vs `"s"`.** `z` additionally disables loop vectorization and is more
  aggressive about not inlining. Here it saved a further ~28 kB over `s`. Because
  `agc-core` contains **hard-real-time** loops (executive/DAP servicer paths), `z` MUST be
  validated against timing budgets, not just size. If any hot loop regresses past its
  deadline, fall back to `s` (which, with LTO+cgu=1, still gets to 133 kB — over target by
  ~2 kB, closed by the §3.1 String removal) or apply per-function `#[optimize(speed)]`
  (nightly) / hoist the hot loop into its own crate compiled at `opt="s"`. **Default
  recommendation: adopt `z`, then run the timing-compliance tests (see testing skill) and
  the end-to-end mission tests before committing.**
- **`strip = true`** removes symbols/debuginfo from the *ELF file* (baseline ELF is
  406 kB on disk); it does **not** change the flash footprint (`.text`/`.rodata` are
  unaffected). Keep an *unstripped* copy for `defmt`/`probe-rs` decoding — either build
  without `strip` for debugging, or archive the unstripped ELF in CI. `strip` is
  cosmetic w.r.t. criterion 1 but worth having for release artifacts.
- **`lto = "thin"`** is a lower-risk alternative to `"fat"` (faster builds, ~90 % of the
  benefit). If build time becomes painful, measure `thin` and prefer it if it still clears
  128 kB. `"fat"` is specified as the default because build time is not a constraint here
  and it maximizes headroom.
- These changes affect the whole workspace's release profile. The host crates
  (`agc-sim`, `agc-test`, tests) build in the `dev`/`test` profiles and are unaffected;
  `cargo test` continues to run unoptimized. Confirm no host tool relies on
  `--release` speed.

### Tier 1 — Guarantee the String/formatting output is gone — **REQUIRED, mostly already met**

Issue #95 asks to "remove all String output along with the String constants." Findings:

- **`defmt` log strings are already nearly free on-target.** `defmt` interns format
  strings into a non-allocated `.defmt` section linked at address 0 (via `-Tdefmt.x`);
  they are **not** programmed into flash. The board's `defmt::info!/error!` calls
  (`agc.rs`, `lib.rs`, `bmi088.rs`) therefore cost almost nothing in `.text`/`.rodata`.
  There is no need to strip logging to meet the budget, and keeping it preserves
  probe-side observability. **Recommendation: keep `defmt`.**
- **The real string cost was the float-formatting code (§2.1), not string literals.** It
  entered the graph through generic `core::fmt` float paths. Tier 0's LTO already removes
  it. **Action: after adopting Tier 0, re-run `llvm-nm | grep -iE 'flt2dec|grisu|dragon'`
  and assert the result is empty** — this is the concrete "String output removed" check.
- **Avoid reintroducing it.** Do not add `Display`/`Debug` formatting of `f32`/`f64` on
  any *release-reachable* path (e.g. `defmt::Display2Format`/`Debug2Format` wrapping a
  float-bearing type). The existing panic handler already gates the `Display2Format(info)`
  print behind `#[cfg(debug_assertions)]`, so the release panic handler pulls in no
  formatter — keep it that way. `Debug2Format(&e)` on the BMI088 error (`agc.rs:203`) is
  the one release-reachable `Debug` use; verify its error type's `Debug` does not format
  floats (it currently prints enum variants), or gate it behind `debug_assertions` too.
- **Test-only assertion strings** (the `format!`/`{}` uses in `#[cfg(test)]` modules of
  `agc-core`, e.g. `v_n.rs`) compile only into the host test binary, never into the board
  image. **No action needed**; do not spend effort removing them.

### Tier 2 — Source-level code-density improvements — **OPTIONAL (only if headroom is wanted)**

With Tier 0 the target is met with ~25 kB of headroom, so these are *not required* for
#95. They are listed for the "get as close as possible to 34k words" goal and should be
applied **only where they do not sacrifice readability or the reconstructed
abstractions** (a core project value — see CLAUDE.md). Each must be justified by a
measured `llvm-nm`/`cargo-bloat` before/after delta, not applied speculatively.

1. **Prune monomorphization bloat.** Generic functions instantiated over several concrete
   types emit a full copy per type. Audit with `cargo bloat --crates` /
   `cargo llvm-lines`. Where a generic is instantiated many times with the same code
   shape, extract a non-generic inner `fn` taking already-erased arguments (the classic
   "outer generic wrapper, inner monomorphic core" pattern) so only the small wrapper is
   duplicated. Keep the ergonomic generic signature at the call site.
2. **Prefer `f64` + `libm` over mixing in `f32`/`micromath` unless precision allows.** The
   board already links both `libm` and `micromath`. `micromath` (f32) routines are smaller
   *and* faster but lose ~7 decimal digits — unacceptable for orbital integration /
   conics / Lambert, acceptable for some DAP/attitude scalars. **Do not** blanket-swap;
   only move a value to `f32`/`micromath` where the numeric error budget is documented to
   permit it (consult the orbital-mechanics agent). This is a code-*quality* risk, so it
   is opt-in per call site, not a global switch.
3. **Avoid panicking indexing / slicing on release paths.** `a[i]`, `slice[a..b]`,
   `.unwrap()`, and integer-overflow-checked arithmetic each emit a panic/bounds branch
   and (pre-`panic=abort`) a formatter. With `panic=abort` the format is already gone, but
   the *bounds-check branches* remain. Where an index is provably in range, use
   `get_unchecked` **only** behind a checked invariant with a comment (no_std safety rules,
   see development skill), or restructure with iterators (`.iter().zip()`,
   `array::from_fn`) which the optimizer proves bounds-free. Prefer the iterator form for
   readability; reserve `unsafe` for measured hot spots.
4. **De-duplicate near-identical program logic.** The P31/P32/P33 CSI/CDH solvers and the
   P34/P35/P74/P75 family share structure. Where two functions differ only by constants,
   factor the shared body into one helper parameterized by data, not by a generic type
   (avoids re-monomorphization). This mirrors the original rope's subroutinization and
   *improves* abstraction rather than harming it.
5. **Const tables: store scaled integers, format lazily.** The `.Lanon` rodata tables are
   already compact; if any large `f64` table is only ever consumed after scaling, storing
   it as `i32`/`i16` fixed-point (as the original AGC did) halves its footprint. Only worth
   it for tables > ~1 kB; measure first.

### Tier 3 — Rebuild `core`/`compiler-builtins` for size — **OPTIONAL, nightly only**

If sub-100 kB is desired, a nightly toolchain can rebuild the sysroot crates at the same
size profile and elide panic string formatting entirely:

```sh
cargo +nightly build -p agc-board-nucleo-f767 \
  --target thumbv7em-none-eabihf --release \
  -Z build-std=core,alloc \
  -Z build-std-features=panic_immediate_abort
```

`panic_immediate_abort` replaces every panic with a bare `udf`/abort, removing residual
panic-location strings and bounds-check message formatting from `core`. This is a
**nightly** feature and would pin the board build to a nightly toolchain (the project is
currently on stable 1.96.0). **Recommendation: do not adopt for #95** — it complicates the
toolchain story for marginal extra savings now that Tier 0 already clears the budget.
Record it as a documented lever for a future "minimum-rope" experiment.

---

## 4. Risks and how to bound them

| Risk | Mitigation |
|---|---|
| `opt-level="z"` slows a hard-real-time loop past its deadline | Run timing-compliance + end-to-end mission tests (§5) before commit; fall back to `s`+LTO (133 kB, close gap via Tier 1/2) or isolate the hot loop into an `opt="s"` crate. |
| LTO changes numerical results via reassociation/inlining | LTO does not enable fast-math; `libm` results are bit-stable. Re-run the VirtualAGC fixture tests to confirm nav outputs unchanged. |
| `f32`/`micromath` substitution (Tier 2.2) loses precision | Opt-in per call site only, each with a documented error budget vetted by orbital-mechanics; never a global switch. |
| `strip=true` breaks `defmt`/`probe-rs` decode | Keep/archive an unstripped ELF for debugging; ship stripped only as the release artifact. |
| `unsafe` indexing (Tier 2.3) introduces UB | Only behind a checked invariant with a comment; prefer iterator reforms; gate under `debug_assertions` bounds asserts. |

---

## 5. Verification plan (maps to acceptance criteria)

Run after applying Tier 0 (and any optional tier):

1. **Compiles (crit. 3):**
   `cargo build -p agc-board-nucleo-f767 --target thumbv7em-none-eabihf --release`.
2. **Code size < 128 kB (crit. 1):**
   `llvm-size target/thumbv7em-none-eabihf/release/agc` → Berkeley `text` column < 131,072.
   Also record `llvm-size -A` for the section breakdown.
3. **Writable data < 4 kB (crit. 2):**
   `.data` + `.bss` + `.uninit` < 4,096 from `llvm-size -A`.
4. **String/formatter removal confirmed (issue text):**
   `llvm-nm --size-sort target/.../agc | grep -iE 'flt2dec|grisu|dragon|format_shortest|format_exact'`
   returns nothing.
5. **All tests pass (crit. 4):** `cargo test` (host profile — unaffected by the release
   profile change, but re-run to be safe), plus the VirtualAGC fixture tests and the
   end-to-end mission tests, and the timing-compliance checks (testing skill) to validate
   the `opt="z"` choice against real-time deadlines.
6. **Clippy clean (crit. 5):** `cargo clippy` and
   `cargo clippy -p agc-board-nucleo-f767 --target thumbv7em-none-eabihf` → no
   warnings/errors.

Record the before/after `llvm-size` table in the issue #95 closing comment.

---

## 6. Recommendation summary

- **Do Tier 0** (profile: `opt-level="z"`, `lto="fat"`, `codegen-units=1`, `strip=true`).
  Measured result: **code size 177,656 → 104,572 bytes (−41%)**, clearing the 128 kB
  target with ~25 kB headroom, with **no source changes** and **no loss of abstraction**.
  This alone satisfies criteria 1–2.
- **Do Tier 1** as a *verification* step (assert the float-formatting code is gone; keep
  `defmt`; do not add release-reachable float `Display`/`Debug`).
- **Treat Tiers 2–3 as optional** headroom work toward the "close to 34k words" goal,
  applied only per measured benefit and only where readability and the reconstructed
  abstractions are preserved. Precision-sensitive substitutions require orbital-mechanics
  sign-off.
- **Gate the `opt="z"` decision on the real-time timing tests**, since size and latency
  trade off on the hard-real-time paths.

### Comparison to the original rope (context, not a criterion)

The original Comanche055 rope is 36,864 × 15-bit words ≈ 73,728 bytes as stored (~34k
words of program). At 104,572 bytes of Thumb-2 code+rodata the Rust port is ~1.4× the
original's byte count — a fair result given the port carries an f64 `libm`, a vendor HAL,
and reconstructed abstractions the original achieved via a hand-tuned interpreter and
packed 7-bit opcodes (see `docs/virtualagc-instruction-and-size-analysis.md`). Byte-for-
byte parity is not a goal; fitting comfortably in a small ROM with readable code is.
