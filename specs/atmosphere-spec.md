# Specification: `navigation/atmosphere` Module — Exponential Atmosphere Model

**Status**: Approved for implementation
**Module path**: `agc-core/src/navigation/atmosphere.rs`
**Architecture reference**: `docs/architecture.md` §9 (Navigation Math)
**Related specs**: `specs/entry-spec.md` (consumer — dynamic pressure and drag), `specs/p61_p67-spec.md` (entry program sequence), `specs/entry-guidance-plan.md` §4 item 4 (original requirement source).
**Glossary cross-reference**: `docs/glossary.md` — Atmosphere model (exponential), Dynamic pressure, R-dot, Entry corridor.
**AGC source files**:
- `Comanche055/REENTRY_CONTROL.agc` — sub-routine `RHO`, the historical equivalent.

---

## 1. Purpose and Scope

`navigation::atmosphere` provides the single-term **exponential atmosphere
model** used by `guidance::entry` to compute dynamic pressure
(`q̄ = ½ ρ v²`) and the drag-coefficient corrections that drive HUNTEST,
UPCONTRL, and the final-phase reference profile.

The model is the same one the historical AGC used:

```
ρ(h) = ρ₀ · exp(−h / H_s)
```

with sea-level reference density `ρ₀ = 1.225 kg/m³` and scale height
`H_s = 7160 m`. This is the simplest atmosphere model the Apollo entry
guidance can use; the AGC fits the US Standard Atmosphere 1976 to about
one part in 1000 below 60 km and is intentionally a single decaying
exponential — no layered model, no temperature, no wind.

### What this module provides

- `SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225` — `ρ₀` constant.
- `SCALE_HEIGHT_M: f64 = 7_160.0` — `H_s` constant.
- `density(altitude_m: f64) -> f64` — single pure function returning
  density in kg/m³.

### What this module does NOT provide

- Layered atmosphere (US Standard Atmosphere isothermal-layer table).
  The model is single-exponential everywhere.
- Wind, gust, or weather modelling.
- Temperature, pressure, or molecular mass. Density only.
- Mars or any other atmosphere. Earth only.
- Altitude derived from a geodetic latitude — the input is treated as
  altitude above a spherical Earth reference (this is the same
  simplification the Apollo CM entry guidance uses).
- Time integration of density (e.g. for trapped radiation in the
  upper atmosphere). The function is memory-less and altitude-only.

---

## 2. AGC Background

The historical CMC computed density inline inside `REENTRY_CONTROL.agc`
via a single multiply-and-exponentiate sequence (sub-routine `RHO`,
called from the up-control and final-phase legs). Both the sea-level
density and scale height were stored in fixed-point at compile time.

The Rust port keeps the same single-exponential form. The values are
`f64` constants with the same physical meaning as the AGC's fixed-point
constants once they are converted to SI units (the AGC carried
0.002 376 9 slug/ft³ for `ρ₀` and 23 500 ft for `H_s`).

---

## 3. Rust API

### 3.1 Constants

```rust
pub const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;
pub const SCALE_HEIGHT_M: f64           = 7_160.0;
```

A private constant `MAX_ALTITUDE_M = 250_000.0` is the cut-off above
which the function returns `0.0` exactly (see §4.1 below).

### 3.2 Function

```rust
pub fn density(altitude_m: f64) -> f64;
```

Pure, total, no panics, no allocation. Safe for `#![no_std]`.

---

## 4. Functional Requirements

### 4.1 `density(altitude_m) -> f64`

For altitude `h` in metres above the spherical Earth reference:

1. If `h ≥ MAX_ALTITUDE_M (250_000.0)`, return `0.0` exactly.
   Above this altitude `exp(−h/H_s) < 1e-15` and feeding a
   round-to-subnormal value into downstream divisions (e.g. dynamic
   pressure denominators) risks NaN propagation. Clamping to a hard
   zero is safer than letting the exponent underflow continue to
   propagate.
2. Otherwise return `ρ₀ · exp(−h / H_s)` using `libm::exp`.

**Negative altitudes** are *not* clamped on the low end and produce
densities `> ρ₀`. The entry guidance never queries below the reference
sphere, so the unclamped behaviour is fine and removes a branch from
the hot path.

**Postcondition**:
- For `h < 0`, `density(h) > SEA_LEVEL_DENSITY_KG_M3`.
- For `h = 0`, `density(0.0) == SEA_LEVEL_DENSITY_KG_M3` (bit-exact —
  `exp(0) == 1.0`).
- For `0 < h < MAX_ALTITUDE_M`, `density(h)` is strictly decreasing in
  `h`.
- For `h ≥ MAX_ALTITUDE_M`, `density(h) == 0.0` (bit-exact).

---

## 5. Numerical Notes

- `libm::exp` is used so the function is callable from `#![no_std]`
  bare-metal builds and from interrupt handlers.
- The model is exact for the AGC's purposes inside the entry corridor
  (0–120 km). Outside that band (~50 km and higher) the density
  diverges from the US Standard Atmosphere by a factor of order 2,
  which is **documented as the limitation of the AGC's
  approximation**. The Apollo flight rules use this single-exponential
  fit; matching that fit is the right call for the Rust port.
- No alarm or NaN policy is needed because `libm::exp` of any finite
  input is finite (and positive). For `altitude_m = NaN`, the
  comparison `>= MAX_ALTITUDE_M` evaluates to `false`, `libm::exp(NaN)`
  is `NaN`, and `density(NaN) == NaN` — the caller's invariant.

---

## 6. Dependencies

| Dependency | Used for |
|---|---|
| `libm::exp` | The exponential evaluation (`no_std` constraint). |

No dependency on any other `agc-core` module. No state.

---

## 7. Module Layout

```
src/navigation/atmosphere.rs
├── pub const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225
├── pub const SCALE_HEIGHT_M: f64 = 7_160.0
├── const MAX_ALTITUDE_M: f64 = 250_000.0  (private)
├── pub fn density(altitude_m: f64) -> f64
└── #[cfg(test)] mod tests
```

---

## 8. Test Cases

The implementation in `agc-core/src/navigation/atmosphere.rs::tests`
provides the following representative cases:

| ID | What is verified |
|---|---|
| `tc_atm_1_sea_level` | `density(0.0)` returns exactly `SEA_LEVEL_DENSITY_KG_M3` (within `1e-15`). |
| `tc_atm_2_one_scale_height` | `density(H_s)` returns `ρ₀ / e` within `1e-9`. |
| `tc_atm_3_ten_scale_heights` | `density(10 · H_s) ≈ 5.56e-5 kg/m³`; matches `ρ₀ · exp(−10)` within `1e-12`. |
| `tc_atm_4_monotone` | density strictly decreases through the modelled corridor `0 ≤ h ≤ 120 km` (sampled every 5 km). |
| `tc_atm_5_cutoff` | `density(MAX_ALTITUDE_M) == 0.0` and `density(1e9) == 0.0`. |
| `tc_atm_6_fifty_km_order_of_magnitude` | density at 50 km lies in `[5e-4, 2e-3] kg/m³` — the modelled value is ~1.13e-3, within a factor-of-two of the textbook US Standard Atmosphere value (~1.027e-3), documenting the limitation of the single-exponential approximation. |

---

## 9. Spec Quality Checklist

- [x] AGC source counterpart (`REENTRY_CONTROL.agc::RHO`) referenced.
- [x] Single public function specified with preconditions and
      postconditions (§4).
- [x] Cut-off behaviour at `MAX_ALTITUDE_M` documented (§4.1).
- [x] Negative-altitude behaviour explicitly documented (§4.1).
- [x] Limitation of the single-exponential fit honestly noted (§5).
- [x] Dependencies listed (§6).
- [x] Test coverage summarised (§8).
