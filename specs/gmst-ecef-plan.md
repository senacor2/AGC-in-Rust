# GMST + ECEF Transforms Plan

**Status**: Approved
**Scope**: Centralize the Earth-rotation angle computation and the inertial ↔ Earth-fixed (ECI ↔ ECEF) rotation that today lives duplicated inline in P21 and P22. Fill the unused `met_to_gmst` stub with the correct AGC-convention formula. Add the inverse direction and a velocity transform so entry guidance and ground-track displays can share one tested implementation.
**Module owner**: `agc-core/src/navigation/state_vector.rs` (transform helpers), `agc-core/src/navigation/time.rs` (angle computation + constants).
**Unblocks**: #4 (entry-guidance MS-E1).
**AGC source files**:
- `Comanche055/LAT-LONG_SUBROUTINES.agc` — Earth-fixed ↔ inertial rotation
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — GHABASE definition (`AgcState.gha_epoch_rad`)

---

## 1. Current state

The work is less new code, more consolidation. The AGC-style GHA convention (`gha_epoch + OMEGA_EARTH * t`) is already in use, hard-coded inline in two programs.

| Where | What |
|---|---|
| `navigation/time.rs:14` | `met_to_gmst(Met, launch_jd) -> f64` — `todo!()` stub; currently unused. |
| `navigation/time.rs:10` | `pub const REFERENCE_JD: f64` — unused dead constant. |
| `programs/p21.rs:50` | `pub const OMEGA_EARTH: f64 = 7.292_115_085_5e-5` — should be a navigation constant. |
| `programs/p21.rs:182–193` | Inline `Rz(+gha)` rotating ECI → Earth-fixed: `gha = gha_epoch + OMEGA_EARTH * t`, then matrix-multiply. |
| `programs/p22.rs:559` | Same formula again, used in `landmark_inertial_pos` to rotate Earth-fixed landmark → inertial. |
| `programs/p23.rs:223` | Uses `gha_epoch_rad` in landmark inertial-position computation. |
| `AgcState.gha_epoch_rad` (`lib.rs:214`) | Greenwich Hour Angle of Aries at MET=0; set by uplink, survives FRESH START. |

## 2. Naming and convention

The AGC uses **GHA of Aries** (Greenwich Hour Angle of the vernal equinox), parameterized by the epoch angle `gha_epoch_rad` and the constant rotation rate `OMEGA_EARTH`. This is numerically equivalent to GMST in radians once the epoch alignment is fixed by uplink. The IAU-style `launch_jd`-based GMST is not needed; the AGC formulation is the source of truth.

The function is named `met_to_gha` (not `met_to_gmst`) to match the AGC source and the existing P21/P22 call sites. The `launch_jd` parameter is removed; the function takes `gha_epoch_rad` instead.

## 3. Design decisions (locked)

1. **Function name**: `met_to_gha(t: Met, gha_epoch_rad: f64) -> f64`. Replaces the `met_to_gmst` stub.
2. **Module location**: free functions added directly to `navigation/state_vector.rs`, alongside the `StateVector` type. No new `frames.rs` file.
3. **No `Frame::EarthFixed` variant**: ECEF is transient — computed on the fly without ever storing an ECEF `StateVector`. This matches current usage and avoids stale-state bugs in the gravity / integration code that assume inertial frames.
4. **`OMEGA_EARTH` location**: moved from `programs/p21.rs:50` to `navigation/time.rs`. Re-exported once from `navigation::time::OMEGA_EARTH`.
5. **`REFERENCE_JD` constant**: deleted as dead code.

## 4. Scope

In:
- Canonical Earth-rotation constants (`OMEGA_EARTH`) and angle computation (`met_to_gha`) in `navigation/time.rs`.
- Position and velocity transform helpers (ECI ↔ ECEF) in `navigation/state_vector.rs`.
- Refactor P21, P22, P23 to use the shared helpers; remove duplicate inline rotations.
- Tests including round-trip, identity, and velocity cross-term sanity.

Out (deferred):
- WGS84 ellipsoid lat/lon/alt — current code uses a spherical Earth, and entry guidance can use the same approximation. Add later if accuracy demands.
- Precession / nutation — not modeled by Apollo and not needed for our scope.
- Lunar-fixed frame (MCMF) — no moon-surface targeting in scope.

## 5. Module layout

```
navigation/time.rs                     (extend)
  pub const OMEGA_EARTH: f64 = 7.292_115_085_5e-5;  // moved from p21.rs
  pub fn met_to_gha(t: Met, gha_epoch_rad: f64) -> f64;
  // pub const REFERENCE_JD: f64 — deleted

navigation/state_vector.rs             (extend)
  pub fn inertial_to_earth_fixed(pos: Vec3, gha_rad: f64) -> Vec3;
  pub fn earth_fixed_to_inertial(pos: Vec3, gha_rad: f64) -> Vec3;
  pub fn inertial_to_earth_fixed_vel(pos: Vec3, vel: Vec3, gha_rad: f64) -> Vec3;
  pub fn earth_fixed_to_inertial_vel(pos: Vec3, vel: Vec3, gha_rad: f64) -> Vec3;

programs/p21.rs                        (refactor)
  // OMEGA_EARTH import now `crate::navigation::time::OMEGA_EARTH`
  // p21_compute_ground_track inline rotation collapses to one helper call

programs/p22.rs                        (refactor)
  // landmark_inertial_pos uses earth_fixed_to_inertial

programs/p23.rs                        (refactor if applicable)
```

### Velocity transform note

ECI ↔ ECEF velocity is not just `Rz` — the Earth-rotation cross-term `ω × r` must be added (or subtracted) to get the correct rotating-frame velocity. P21 today only needs position, but entry guidance and any future ground-track-rate display benefit from having both. The helpers are implemented together; the cost is small and the entry plan benefits.

## 6. Implementation milestones

### MS-F1 — Centralize constants and add transform helpers (additive only)
- Move `OMEGA_EARTH` from `programs/p21.rs` to `navigation/time.rs`.
- Delete dead `REFERENCE_JD` constant.
- Implement `met_to_gha(t, gha_epoch_rad) -> f64` (replaces the `todo!()` stub).
- Add four transform helpers in `navigation/state_vector.rs`: `inertial_to_earth_fixed`, `earth_fixed_to_inertial`, `inertial_to_earth_fixed_vel`, `earth_fixed_to_inertial_vel`.
- Unit tests: zero-rotation identity, quarter-revolution sanity, round-trip, velocity cross-term sign and magnitude.
- **Exit criterion**: new helpers pass tests; no callsite changes; all existing tests still pass.

### MS-F2 — Refactor P21 / P22 / P23 to use the helpers
- P21 `p21_compute_ground_track`: replace inline `Rz` (lines ~182–193) with `inertial_to_earth_fixed`. Import `OMEGA_EARTH` from `navigation::time` instead of redefining locally.
- P22 `landmark_inertial_pos` (around `p22.rs:559`): replace inline rotation with `earth_fixed_to_inertial`.
- P23: any remaining inline rotation replaced.
- **Exit criterion**: P21/P22/P23 tests pass without modification (numerical identity); LOC reduction visible; the duplicate `OMEGA_EARTH` definition in `p21.rs` is gone.

## 7. Test strategy

Unit tests in `navigation/state_vector.rs`:
- `inertial_to_earth_fixed` at `gha = 0` is identity.
- `inertial_to_earth_fixed` at `gha = π/2` rotates `[1, 0, 0]` to `[0, -1, 0]` (Earth has rotated east, so the inertial x-axis now lies along the -ECEF-y axis).
- Round-trip `inertial → earth_fixed → inertial` returns the input within 1e-14.
- Velocity transform: a point stationary in ECEF on the equator at `r = R_earth` has ECI velocity magnitude `ω × R_earth ≈ 465 m/s` and direction along ECI +y at `gha = 0`.

Unit tests in `navigation/time.rs`:
- `met_to_gha(Met(0), 0.0) == 0.0`.
- `met_to_gha(Met(86_400 * 100), 0.0)` ≈ `OMEGA_EARTH * 86_400` (one sidereal-day worth of rotation).
- Linearity in both arguments.

Existing tests in `programs/p21.rs::tests` (the TC-P21-* suite) act as integration tests for the refactor — they must pass byte-for-byte.

No new fixtures; everything is closed-form.

## 8. GitHub issue seed

New label `navigation` (color `#fbca04`) — scopes navigation-infrastructure work. Existing labels `milestone`, `infrastructure`, `enhancement` reused.

| Title | Labels |
|---|---|
| GMST + ECEF transforms — implementation tracking | `navigation`, `milestone` |
| MS-F1: Centralize Earth-rotation constants and add ECI↔ECEF helpers | `navigation`, `milestone`, `infrastructure`, `enhancement` |
| MS-F2: Refactor P21 / P22 / P23 to use shared frame helpers | `navigation`, `milestone`, `enhancement` |
