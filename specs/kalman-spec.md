# Specification: `navigation/kalman` Module — Scalar Kalman Measurement Update

**Status**: Approved for implementation
**Module path**: `agc-core/src/navigation/kalman.rs`
**Architecture reference**: `docs/architecture.md` §9 (Navigation Math)
**Related specs**:
- `specs/p20-spec.md` §6.5–§6.10 — full algorithm derivation in the P20 (Rendezvous Navigation) context.
- `specs/p21_p22-spec.md` §6.2 — P22 (Landmark Tracking) adaptation; same algorithm against the CSM state and CSM covariance.
- `specs/p23-spec.md` — P23 (Cislunar Midcourse Navigation) consumer.
- `specs/state-vector-spec.md` — the 6-vector that this module updates.
**Glossary cross-reference**: `docs/glossary.md` — Scalar Kalman update, W-matrix.
**AGC source files**:
- `Comanche055/MEASUREMENT_INCORPORATION.agc` — the historical equivalent
  (`*MEASUREMENT INCORPORATION ROUTINE*`).
**O'Brien reference**: Chapter 11 pp. 318–325 — derivation of the scalar
  incorporation algorithm and the rationale for the 3-σ rejection gate.

---

## 1. Purpose and Scope

`navigation::kalman` provides a **pure scalar Kalman measurement-update
helper** shared by every program that incorporates a single-scalar
optical or radar mark: P20 (rendezvous), P22 (landmark), P23 (cislunar
star-horizon). The historical AGC's `MEASUREMENT_INCORPORATION.agc`
served the same role.

The function is pure (no global state, no allocation), allocation-free,
and safe for `#![no_std]` bare-metal builds.

The module deliberately lives **outside** any program module so that
P20, P22, and P23 do not have to import each other.

### What this module provides

- `UpdateOutcome` — discriminated enum returned by the update:
  `Accepted`, `Rejected`, `AcceptedWOverflow`.
- `scalar_measurement_update(x, w, b, residual, sigma_sq) -> UpdateOutcome`
  — the algorithm.

### What this module does NOT provide

- Sensitivity-vector (`b`) construction — that is mark-type-specific
  and lives in each program (range, range-rate, sextant shaft/trunnion,
  star-horizon angle, …).
- Residual (`z_observed − z_predicted`) construction — likewise the
  caller's job.
- W-matrix **rectification** for the `AcceptedWOverflow` case — the
  caller decides which rectify routine to call (P20 has one,
  P22 has another). The historical AGC had `INTSTALL` and `INCORP2`
  with the same partitioning.
- Multi-dimensional measurement updates. This is strictly **scalar**
  incorporation; vector measurements are decomposed into a sequence of
  scalar updates by the caller (the same scheme the AGC used).
- Numerical stabilisation beyond the diagonal positive-definiteness
  check (e.g. Joseph form, square-root filtering). The AGC used the
  same simple downdate; the gate + rectification machinery makes it
  acceptably robust for the mission timeline.

---

## 2. AGC Background

The AGC's incorporation routine processed one scalar measurement per
call against a 6-state navigation vector (position + velocity) and a
6×6 covariance (the "W-matrix"). The chain was:

1. Compute Kalman gain from the measurement sensitivity `b` and the
   current covariance.
2. Update state and covariance (the latter via a downdate, not the
   Joseph form).
3. Reject the measurement if the residual was outside `3σ` of the
   predicted innovation.
4. If the downdate broke the positive-definiteness of the covariance,
   rectify before the next cycle.

The Rust port keeps the same partitioning. `scalar_measurement_update`
implements steps 1–3 and signals (via `AcceptedWOverflow`) that step 4
is the caller's responsibility.

---

## 3. Rust API

### 3.1 Types

```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UpdateOutcome {
    /// Mark passed the gate; `x` and `w` updated.
    Accepted,
    /// Residual exceeded 3σ; `x` and `w` unchanged.
    Rejected,
    /// Accepted, but a diagonal entry of `w` went negative.
    /// The caller MUST call its rectify routine before the next update.
    AcceptedWOverflow,
}
```

### 3.2 Function

```rust
pub fn scalar_measurement_update(
    x: &mut [f64; 6],
    w: &mut [[f64; 6]; 6],
    b: [f64; 6],
    residual: f64,
    sigma_sq: f64,
) -> UpdateOutcome;
```

Operates **in place** on `x` (state) and `w` (covariance). Mutates them
only on the `Accepted` and `AcceptedWOverflow` branches; leaves them
bit-exactly unchanged on `Rejected`.

---

## 4. Functional Requirements

### 4.1 Algorithm

The body of `scalar_measurement_update` is the AGC's seven-step scalar
incorporation, translated directly into `f64` arithmetic:

| # | Step | Notes |
|---|---|---|
| 1 | Compute the 6-vector `Wb = W · b`. | Double loop, 36 multiplies. |
| 2 | Compute the innovation variance `S = bᵀ · Wb + σ²`. | Scalar accumulate. |
| 3 | **Gate**: if `|residual| > 3 · √|S|`, return `Rejected`. | Uses `libm::fabs` and `libm::sqrt`; the `|S|` form makes the gate NaN-safe — physically `S ≥ σ² > 0`. |
| 4 | Compute the 6-vector Kalman gain `k = Wb / S`. | Six divides. |
| 5 | State update `x ← x + k · residual`. | Six multiplies. |
| 6 | Covariance downdate `W[i][j] ← W[i][j] − k[i] · k[j] · S` for all `i, j`. | 36 multiplies. |
| 7 | **Positive-definiteness check**: if any `W[i][i] < 0`, return `AcceptedWOverflow`. | Six comparisons. |

If steps 7's check passes, return `Accepted`.

### 4.2 Preconditions

- `w` is symmetric and positive semi-definite before the call. The
  function does not verify this invariant; violating it can produce
  numerical garbage.
- `sigma_sq > 0`. The gate uses `|S|`, so even an accidentally negative
  `sigma_sq` cannot panic; the result is just nonsensical.

### 4.3 Postconditions

| Branch | `x` mutation | `w` mutation | Caller obligation |
|---|---|---|---|
| `Accepted` | Updated (step 5) | Updated (step 6); diagonal still ≥ 0 | None |
| `AcceptedWOverflow` | Updated (step 5) | Updated (step 6); at least one diagonal is negative | Call `rectify_*` before the next update |
| `Rejected` | Unchanged | Unchanged | Optionally log / count the rejection |

---

## 5. Numerical Notes

- The downdate in step 6 is the *direct* form (not Joseph form). The
  AGC accepts the occasional loss of positive-definiteness in exchange
  for cheap arithmetic; the caller handles the loss via rectify.
- The gate is `|residual| > 3 √|S|`, not `> 3 √S`. The absolute value
  makes the gate robust if `S` is non-finite or accidentally negative.
- The function performs no `nan` or `inf` filtering on `residual` or
  `b`. The caller is expected to feed finite values; if it doesn't,
  the result propagates `NaN` through `x` and `w` as in any IEEE-754
  arithmetic.
- All `libm` calls (`sqrt`, `fabs`) are `no_std` compatible.

---

## 6. Dependencies

| Dependency | Used for |
|---|---|
| `libm::sqrt` | 3-σ gate denominator. |
| `libm::fabs` | Residual and `S` magnitudes. |

No dependency on any other `agc-core` module. No state.

---

## 7. Module Layout

```
src/navigation/kalman.rs
├── pub enum UpdateOutcome { Accepted, Rejected, AcceptedWOverflow }
└── pub fn scalar_measurement_update(...) -> UpdateOutcome
```

(No `#[cfg(test)] mod tests` in the file itself — the algorithm is
exercised against representative measurements through the consumer
specs' test cases, see §8.)

---

## 8. Test Cases

Direct unit-test coverage lives in the consumer programs, where the
sensitivity vector `b` and the residual carry physical meaning:

- `agc-core/src/programs/p20.rs::tests` — rendezvous range and
  range-rate updates, 3-σ gate exercise, W-overflow path.
- `agc-core/src/programs/p22.rs::tests` — landmark sextant shaft /
  trunnion angle updates against the CSM state.
- `agc-core/src/programs/p23.rs::tests` — cislunar star-horizon angle
  updates.

A pure-algorithm test file is not required: every code path is
reachable through at least one consumer test. (Issue #159 audit noted
that adding a small synthetic-input test in this module would be
useful regression coverage; it is not blocking.)

---

## 9. Spec Quality Checklist

- [x] AGC source counterpart (`MEASUREMENT_INCORPORATION.agc`)
      referenced.
- [x] Single public function specified with all seven algorithm steps
      (§4.1).
- [x] Each `UpdateOutcome` variant's caller obligation documented
      (§4.3).
- [x] Numerical choices documented (direct downdate, NaN-safe gate)
      (§5).
- [x] No state, no allocation, `no_std` confirmed (§1).
- [x] Dependencies listed (§6).
- [x] Test coverage pointed to the consumer programs (§8).
