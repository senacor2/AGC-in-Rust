# MS-T5 — Inter-phase handoff integration tests

**Status**: Draft, for developer consumption
**Implements**: GitHub issue #28
**Target file**: `agc-test/tests/handoffs.rs`
**Dependencies**: MS-T4 phase tests landed; `agc-sim::scenario` API stable.

---

## 1. Goal / scope

MS-T4 demonstrated that each Apollo 8 phase tracks the oracle inside its
own checkpoint window. MS-T5 is one layer up: the *transitions* between
phases must not corrupt AGC state. This test file (`handoffs.rs`) adds
four short, self-contained Rust integration tests, each focused on one
boundary and each carrying **explicit state-invariant assertions** in
addition to program-progression checks. The pattern is the same
checkpoint-reseed used in MS-T4: seed a state at a known checkpoint,
drive a minimal program sequence, assert the boundary invariants. No
test should take more than a few seconds of wall-clock time.

The four boundaries are taken verbatim from
`specs/end-to-end-mission-testing-plan.md` §8 / MS-T5 and issue #28:

| # | Boundary                              | Primary risk it guards against                              |
|---|---------------------------------------|-------------------------------------------------------------|
| 1 | P40 cutoff → P00 → V37 next program   | Stale `pending_maneuver`, hook, or DAP state across re-select |
| 2 | P23 marks → P30 targeting             | Nav update done *after* P30 read its inputs                 |
| 3 | SOI crossing (Earth↔Moon, both ways)  | Vector discontinuity / "forgot to subtract v_moon" mass-bug |
| 4 | P52 alignment → next burn             | SERVICER consuming stale REFSMMAT (cache between P52 and SERVICER) |

---

## 2. Conventions used in this spec

- All paths relative to the code base. The developer creates exactly one file:
  `agc-test/tests/handoffs.rs`.
- One `#[test] fn …` per numbered test below (Test 3 is two functions:
  `_outbound` and `_inbound` — keeps each test focused, no parameterisation
  framework needed).
- All tests use `agc_core::AgcState::new()` + `agc_sim::SimHardware::new()`
  + the `ScenarioBuilder` fluent API. No direct waitlist driving except
  where called out.
- Tolerances below are **numeric, not vague**. Any deviation requires
  spec amendment.
- Reuse helpers from the existing phase tests by copy (not by `mod`
  import) — these are integration tests; cross-file coupling is worse
  than 20 duplicated lines.

---

## 3. Test 1 — P40 cutoff → P00 → V37 next program

**Goal**: prove that after a completed SPS burn the AGC returns to a
fully clean state and a subsequent program selection succeeds.

**Why this matters**: P40 installs a `servicer_exit` hook, sets
`burn_active`, consumes `pending_maneuver`, drives the DAP into `Tvc`,
and writes V16 N40 to the DSKY. Each of those fields has its own owner;
a stale value at any one of them would corrupt the next program.

### 3.1 Fixture state

Start from a circular LEO at 6 778 km radius, 7 669 m/s prograde — the
exact same fixture used in `agc-test/tests/p40_sps_burn.rs`. This is
the only handoff test that exercises a real burn end-to-end; reusing
the fixture means the burn dynamics are already well-understood. Burn
target: **21 m/s along-track** (LVLH X), TIG at MET 5 min. No
non-identity REFSMMAT — keep this test orthogonal to Test 4.

### 3.2 Key sequence

The sequence is a trimmed version of `p40_sps_burn.rs` followed by an
explicit P00 selection and a V37→Pxx selection (P30 chosen because it
is the natural "next program" after a burn — the crew typically loads
the next maneuver):

1. V71 P27 block update — seed CSM state vector (helper:
   `ScenarioBuilder::v71_p27_block_update`).
2. V37 E30 E — select P30. Assert `state.major_mode == 30`.
3. V25 N33 ENTR + HMS digits — load TIG = 0h 5m 0.00s.
4. V25 N81 + along-track ΔV — consumes `pending_tig`, calls
   `p30_load_dv_lvlh`. Assert `state.pending_maneuver.is_some()`.
5. V37 E40 E — select P40. Assert `state.major_mode == 40`,
   `state.burn.burn_active == true`, `state.pending_maneuver.is_none()`,
   `state.servicer_exit.is_some()`.
6. PRO — arms the burn (`state.burn.armed == true`).
7. Drive the burn loop to cutoff using the same pattern as
   `p40_sps_burn.rs:226-268` (manual `WaitlistPump`/`DapPump`/`hw.tick`
   loop; the scenario builder cannot jump time inside a powered phase
   because the PIPA accumulator needs `hw.tick` calls). Loop terminates
   when `state.burn.burn_active == false`.
8. **Boundary point #1 — post-cutoff (still major_mode 40)**. Assert
   set in §3.3.
9. V37 E00 E — select P00.
10. **Boundary point #2 — after P00**. Assert set in §3.4.
11. V37 E30 E — select P30 (the second "next program"). This re-runs
    the same handoff a second time, this time from a quiescent P00
    rather than mid-mode-transition. Assert set in §3.5.

### 3.3 Invariants at boundary point #1 (post-P40 cutoff)

| Field                                       | Expected value         |
|---------------------------------------------|------------------------|
| `state.major_mode`                          | `40` (P40 still owns the major-mode until V37) |
| `state.burn.burn_active`                    | `false`                |
| `state.burn.armed`                          | `false`                |
| `state.engine_thrusting`                    | `false`                |
| `hw.engine.thrusting`                       | `false`                |
| `state.servicer_exit.is_none()`             | `true` (P40 must uninstall on cutoff — `p40_sps_burn.rs:301-304` already asserts this) |
| `state.pending_maneuver.is_none()`          | `true` (consumed at P40 init) |
| `state.alarm.code`                          | `0`                    |
| achieved ΔV magnitude (norm of `state.burn.accumulated_dv_inertial`) | within `±5.0 m/s` of target |

### 3.4 Invariants at boundary point #2 (after V37 E00 E)

| Field                                       | Expected value         |
|---------------------------------------------|------------------------|
| `state.major_mode`                          | `0`                    |
| `state.dsky.prog`                           | `0`                    |
| `state.servicer_exit.is_none()`             | `true`                 |
| `state.burn.burn_active`                    | `false`                |
| `state.engine_thrusting`                    | `false`                |
| `state.dap_state.mode`                      | `DapMode::AttitudeHold` (P00 transitions Tvc → AttitudeHold; see `p00.rs:27-49`) |
| `state.csm_state.position` Δ vs. snapshot taken at boundary #1 | `< 1.0 m` (P00 must not perturb nav) |
| `state.csm_state.velocity` Δ vs. snapshot taken at boundary #1 | `< 0.001 m/s` |
| `state.alarm.code`                          | `0`                    |

The position/velocity comparison: snapshot `state.csm_state` to a local
variable between steps 8 and 9, then compare component-by-component
(absolute difference per axis under tolerance).

### 3.5 Invariants at boundary point #3 (after V37 E30)

| Field                                       | Expected value         |
|---------------------------------------------|------------------------|
| `state.major_mode`                          | `30`                   |
| `state.dsky.prog`                           | `30`                   |
| `state.alarm.code`                          | `0`                    |
| `state.servicer_exit.is_none()`             | `true` (still clean after another program switch) |
| `state.pending_maneuver.is_none()`          | `true` (P30 init clears any stale pending maneuver — `p30.rs:73`) |

---

## 4. Test 2 — P23 marks → P30 targeting

**Goal**: prove that a nav correction applied while P23 is active
becomes visible to P30 when the crew transitions to MCC targeting.

**Plumbing note (important)**: P23 marks are not currently driven from
the scenario builder (no `optics_sighting` path to
`p23_incorporate_star_horizon_mark`; the existing star-sighting handler
in `scenario.rs:982-1003` routes pairs to `p52_mark_align`, not to P23).
Driving real P23 marks would require a builder extension. **For this
test, use the uplink path as the proxy for "P23 mark output"**: an
uplinked correction is what a P23 mark would produce after filtering.
This is explicitly called out in §8 (gap list) but is not a blocker —
P23's filter math is unit-tested in `agc-core/src/programs/p23.rs`; the
boundary under test here is P30's *consumption* of the corrected state.

### 4.1 Fixture state

Trans-lunar coast, mid-cislunar — pick a fixture similar to the
translunar phase 3 checkpoint (MET ≈ T+30:00:00, ECI, far from any
SOI). The exact value of the oracle SV does not matter for this test;
the developer can either compute one via `propagate_coast` like
`phase_translunar.rs:325-340` or pick a plausible literal (e.g.
position `[1.5e8, 5e7, 2e7] m`, velocity `[1500, 800, 200] m/s`, frame
`EarthInertial`). Use the latter — this is faster, deterministic, and
the actual oracle accuracy is irrelevant: we only need the values to
flow through the AGC unchanged.

### 4.2 Sequence

1. Seed CSM state to a *known wrong* state vector. Choose a 5 km
   position offset along +X from the oracle:
   `csm_state.position[0] += 5000.0` before `SeedState`. This is the
   "uncorrected" pre-P23 state.
2. V37 E23 E — select P23. Assert `state.major_mode == 23`,
   `state.csm_nav.tracking_active == true`.
3. **Snapshot the uncorrected state**:
   `let uncorrected = state.csm_state;`
4. Apply the nav correction. Use the V71 P27 block-update path with
   the *oracle* (correct) position and velocity components — six words
   at logical address 1. This simulates the ground uplink that would
   normally follow a successful P23 mark. (P27 writes integer km /
   integer m/s — the test should choose oracle component values that
   are integers in those units, e.g. `pos = [150_000_000, 50_000_000,
   20_000_000] m → [150_000, 50_000, 20_000] km` for the P27 frame.
   Check the exact P27 scaling against `v_n.rs:1248` `p27_apply_word`
   when wiring up the digits.)
5. Assert that `state.csm_state` now equals the oracle SV (component-wise
   tolerance `< 1.0 m` on position, `< 1e-6 m/s` on velocity — P27
   stores as integer-scaled fixed point, so a few ULP of rounding is
   possible; widen if the developer finds the rounding floor is larger).
6. V37 E30 E — select P30. Assert `state.major_mode == 30`,
   `state.pending_maneuver.is_none()` (P30 init clears it).
7. V25 N33 ENTR + HMS — load TIG = current MET + 10 min.
8. V25 N81 + LVLH ΔV — load `[+2.35, 0.0, 0.0]` m/s (Apollo 8 MCC-2
   magnitude, prograde). This commits via
   `noun_81_commit_dv_lvlh → p30_load_dv_lvlh`.
9. **Boundary point — pending_maneuver inspection.** Assert set in §4.3.

### 4.3 Invariants

The crux of this test is that `p30_load_dv_lvlh` reads `state.csm_state`
*at the time of call* and converts the LVLH ΔV to inertial using *that*
state. So:

| Invariant                                                                | Tolerance         |
|--------------------------------------------------------------------------|-------------------|
| `state.pending_maneuver.is_some()`                                       | —                 |
| `state.pending_maneuver.tig == TIG_loaded`                               | exact equality    |
| `state.pending_maneuver.mode == TargetingMode::ExternalDeltaV`           | exact equality    |
| `state.pending_maneuver.delta_v` magnitude                                | `2.35 m/s ± 1e-6` |
| `‖state.csm_state.position - uncorrected.position‖` (sanity)              | `≥ 4 999 m`       |

The decisive check is computing the **inertial ΔV that P30 *would have*
produced if it had used the uncorrected state**, calling
`apply_external_delta_v` directly with `uncorrected`, and asserting
that the actual `state.pending_maneuver.delta_v.0` is **not** equal to
that vector (L2 distance ≥ `1.0e-4 m/s`). For a 5 km position offset
the LVLH frame rotates measurably (~3.3e-5 rad), so the inertial ΔV
will differ on the order of `1e-4 m/s` across components — well above
floating-point noise. This catches a regression where P30 cached or
used a stale state.

Implementation hint for the comparison: import
`agc_core::guidance::targeting::apply_external_delta_v` and call
`apply_external_delta_v(uncorrected, tig, dv_rsw, state.refsmmat)`,
remembering the same crew → RSW re-ordering done in `p30_load_dv_lvlh`
(crew `[X,Y,Z] = [along, radial, cross]` → RSW `[Y, X, Z]`).

---

## 5. Test 3 — SOI crossings (outbound and inbound)

**Goal**: prove that crossing the Moon's sphere of influence (a) flips
`csm_state.frame` and (b) preserves the spacecraft's absolute (Earth-
inertial) trajectory across the handover.

**Two `#[test]` functions** — easier to read failures than a parameterised
single test:

- `tc_handoff_soi_outbound_eci_to_mci`
- `tc_handoff_soi_inbound_mci_to_eci`

### 5.1 Critical infrastructure caveat

`navigation::integration::soi_check` is **only called from
`propagate_coast`** (the RK4 oracle path used by
`advance_ground_truth`); it is **not** called from `average_g_step`
(the SERVICER's powered-flight integrator that also runs during
`AdvanceCoast`'s coast-mode loop). Tracked as GH issue #51.

Consequence: if Test 3 runs `advance_coast` and seeds the AGC's
`csm_state` to ECI, the **AGC's own state vector will not flip frames**
across an SOI crossing — only the test runner's `ground_truth` SV
(which is propagated via `advance_ground_truth → propagate_coast`)
will. So Test 3 must **not** rely on `state.csm_state.frame` flipping
after `advance_coast`. Two paths are open:

- **Path A (chosen for this spec)**: drive `propagate_coast`
  *directly* from the test, bypassing the SERVICER. The "system under
  test" is the SOI-handover code (`soi_check`) plus its integration
  inside `propagate_coast`. The AGC state is seeded but the test does
  not run `advance_coast`. This is sufficient for MS-T5's stated goal
  ("frame transitions are clean; gravity body switches; nav stays
  consistent") and avoids exercising the known #51 deficiency.
- **Path B (rejected)**: run `advance_coast` and assert on the ground
  truth's frame instead of the AGC's. This conflates "what the AGC
  knows" with "what the oracle knows" and would silently pass even if
  the SERVICER kept the AGC in the wrong frame.

The test docstring must call out the Path A choice and reference
issue #51 so the gap is not lost.

### 5.2 SOI threshold and Moon ephemeris

- Use `R_SOI_MOON = 66_183_000.0 m` from
  `agc_core::navigation::gravity::R_SOI_MOON`. **Do not** substitute
  the NASSP 9 R⊕ ≈ 57 400 km value (locked AGC choice — see issue
  comments).
- Moon position: `agc_core::navigation::planetary::moon_position(epoch)`.
- Moon velocity: central-difference from `moon_position` with `δ = 10.0
  s`, exactly as `agc_sim/src/physics.rs:285-296` does it. This is the
  only sane choice until `moon_velocity` exists (issue #52).
- Epoch: use **MET = 0** for both directions. The `moon_position`
  model is anchored to the Apollo-11 epoch, but we do not require the
  position to correspond to any specific Apollo 8 GET — we only need
  a consistent Moon vector at a single instant. Using MET=0 avoids
  long propagation and is reproducible.

### 5.3 Fixture state — outbound (Earth → Moon)

Construct an ECI state vector that is *just outside* the SOI and aimed
at the Moon at MET=0:

```text
moon_pos_eci   = moon_position(Met(0))
moon_vel_eci   = (moon_position(Met(10.0)) - moon_position(Met(-10.0))) / 20.0   // central diff, δ=10 s
inbound_dir    = unit(-moon_pos_eci)            // from Moon back toward Earth (sign chosen below)
r_offset_m     = R_SOI_MOON + 1000.0             // 1 km outside SOI
r_eci          = moon_pos_eci - r_offset_m * unit(moon_pos_eci)  // 1 km outside SOI on the Earth side
// Velocity: inbound toward the Moon at 1500 m/s relative motion, plus moon velocity
v_eci          = moon_vel_eci + 1500.0 * unit(moon_pos_eci)      // moving toward the Moon
```

Sanity check the geometry by computing `‖r_eci - moon_pos_eci‖` — it
must be `R_SOI_MOON + 1000.0` (one km outside).

### 5.4 Sequence — outbound

1. Compute `r_eci`, `v_eci`, `moon_pos_eci`, `moon_vel_eci` per §5.3.
2. Build a single-event scenario (no `AdvanceCoast`!): just
   `seed_state().from_state_vector(sv_eci).met(Met(0))...done()`.
   This puts the values into `state.csm_state` and is the entry point
   for everything after.
3. **In the test, drive the propagator directly:**
   ```text
   let dt = 2.0;                                       // one SERVICER cycle equivalent (s)
   let propagated = propagate_coast(sv_eci, dt, moon_pos_eci);  // this calls soi_check internally
   ```
   `dt = 2.0` is long enough to drive the spacecraft from
   `R_SOI_MOON + 1000` to inside the SOI (1500 m/s × 2 s = 3000 m
   inbound; well across the 1000 m margin).
4. Assert on `propagated` directly (the AGC's `state.csm_state` is left
   at its seeded ECI value; that is fine — we are testing the function
   the SERVICER *should* be calling, even if it doesn't today).
5. As a separate sub-check, run a one-step `advance_coast` (with
   `coast_step_cs = 200`) through the scenario runner with
   `seed_ground_truth(sv_eci)`. The scenario runner's `ground_truth`
   will be advanced via `advance_ground_truth → propagate_coast`,
   which **does** flip the frame. The test then reads
   `ctx.ground_truth` — but that field is private. **Skip the runner
   path; assert only on the direct `propagated` value.** The §5.6 gap
   list flags this.

### 5.5 Invariants — outbound

The clean assertion is on the direct return value of
`propagate_coast`. Round-trip the absolute (ECI) coordinates back from
MCI through the Moon ephemeris at the propagated epoch:

```text
moon_pos_at_end = moon_position(propagated.epoch)
moon_vel_at_end = central_diff(moon_position, propagated.epoch, 10.0)
r_eci_recovered = propagated.position + moon_pos_at_end
v_eci_recovered = propagated.velocity + moon_vel_at_end
```

Then assert:

| Invariant                                                       | Tolerance                        |
|-----------------------------------------------------------------|----------------------------------|
| `propagated.frame == Frame::MoonInertial`                       | exact                            |
| `‖propagated.position‖ < R_SOI_MOON`                             | strict (well inside after 2 s)  |
| Position handover continuity (see below)                         | `< 1.0 m`                        |
| Velocity handover continuity (see below)                         | `< 1.0e-3 m/s` (i.e. 1 mm/s)     |

**Computing the handover-continuity reference** (the bug-catcher):

The mass-bug "forgot to subtract `v_moon` during ECI→MCI conversion"
would produce a velocity discrepancy of `‖v_moon‖ ≈ 1018 m/s`. To
detect it with margin, compute what the *pre-handover* SV would be in
ECI just before the crossing by running a parallel propagation that
does NOT cross the SOI:

```text
// Manual one-step propagation in pure ECI, no SOI check.
// Compute gravity at sv_eci, take an Euler step for the same dt.
// We are NOT testing integrator accuracy here — we are testing the
// handover transform. So Euler is fine; the 2 s drift between Euler
// and RK4 over this dt is < 1 cm.
r_eci_no_soi = sv_eci.position + sv_eci.velocity * dt + 0.5 * g_eci * dt * dt
v_eci_no_soi = sv_eci.velocity + g_eci * dt
```

where `g_eci` is computed via `total_gravity(sv_eci.position,
Frame::EarthInertial, moon_pos_eci)`. Compare:

```text
assert  ‖r_eci_recovered - r_eci_no_soi‖ < 1.0
assert  ‖v_eci_recovered - v_eci_no_soi‖ < 1.0e-3
```

This is the per-orbital-mechanics-finding "vector continuity in
absolute coordinates" check. 1 mm/s tolerance is six orders of
magnitude tighter than the mass-bug signature (1018 m/s) and absorbs
the Euler-vs-RK4 disagreement at this dt.

A weaker but still useful safety net: assert `‖v_eci_recovered -
sv_eci.velocity‖ < 0.1 m/s` (it must not drift much in 2 s; gravity at
this distance is ~0.003 m/s²). This is the "did we lose half a moon
velocity" catch.

### 5.6 Fixture state and sequence — inbound

Mirror of §5.3-§5.5: spacecraft just inside the SOI, moving outward
from the Moon:

```text
moon_pos_eci  = moon_position(Met(0))
moon_vel_eci  = central_diff(moon_position, Met(0), 10.0)
r_offset_m    = R_SOI_MOON - 1000.0             // 1 km inside SOI
r_mci         = -r_offset_m * unit(moon_pos_eci)   // 1 km inside SOI, on the Earth side of Moon
v_mci         = -1500.0 * unit(moon_pos_eci)       // 1.5 km/s outbound (Moon-relative, toward Earth)
sv_mci.frame  = Frame::MoonInertial
```

Sequence: identical to §5.4 but feeding `propagate_coast(sv_mci, dt,
moon_pos_eci)`. Expected post: `frame == Frame::EarthInertial`.

Invariants for the inbound case:

| Invariant                                                       | Tolerance                        |
|-----------------------------------------------------------------|----------------------------------|
| `propagated.frame == Frame::EarthInertial`                      | exact                            |
| `‖propagated.position - moon_pos_eci‖ > R_SOI_MOON`              | strict                           |
| Position handover continuity: `‖propagated.position - r_eci_pre_no_soi‖` where `r_eci_pre_no_soi` is `r_mci + moon_pos_eci` Euler-stepped under MCI gravity for `dt` then converted ECI by adding moon_pos | `< 1.0 m` |
| Velocity handover continuity (analogous)                         | `< 1.0e-3 m/s`                   |

The Euler step for the inbound case is taken in MCI under
`total_gravity(sv_mci.position, Frame::MoonInertial, moon_pos_eci)` —
that uses the Moon as primary body. Then convert ECI: `r_eci =
r_mci_stepped + moon_pos_eci`, `v_eci = v_mci_stepped + moon_vel_eci`.

### 5.7 What this test does NOT cover

- SOI crossing during a powered burn (`average_g_step` does not call
  `soi_check` — tracked in issue #51). MS-T5 cannot fix this without
  changing core code; reference issue #51 in the test docstring.
- SOI crossing during the SERVICER's coast-mode integration. Same
  finding as above. The AGC's onboard nav will silently stay in the
  wrong frame across an SOI crossing today; MS-T5 verifies the
  *transform mathematics* are correct, leaving the *invocation gap*
  for #51.
- SOI crossing across an Apollo-mission-realistic geometry. The
  `moon_position` model is anchored to a different epoch than Apollo 8;
  see `phase_translunar.rs` for the long story. MS-T5 only verifies
  the handover mechanics, not real-trajectory continuity.

---

## 6. Test 4 — P52 alignment → next burn

**Goal**: prove that a new REFSMMAT written by P52 is consumed by the
very next SERVICER cycle and feeds correctly into the inertial-frame
ΔV computation.

**Why this matters**: `servicer_task` reads `state.refsmmat` on every
cycle (`average_g.rs:218`). If P52 wrote to a *cached* matrix and
SERVICER kept reading an old one, the inertial ΔV would be wrong.
Per `scenario.rs` doc-comment §3, no caching exists today — this test
locks that property in.

### 6.1 Fixture state

A stable LEO (same as Test 1's fixture: 6 778 km, 7 669 m/s prograde).
The orbit choice doesn't matter for this test; we just need a
non-degenerate state vector and a fresh `AgcState`.

### 6.2 Sequence

1. Seed state. Set `state.imu_alignment_state =
   ImuAlignmentState::FineAligned` directly (the P51→P52 climb is out
   of scope for *this* test; P52's prerequisite is just that the
   platform is at least coarsely aligned).
2. **Snapshot initial REFSMMAT**:
   `let refsmmat_before = state.refsmmat;` (default is identity).
3. Drive P52 via the optics path: `seed_truth_refsmmat(M_TRUTH)` with
   a known non-identity matrix `M_TRUTH` (see §6.4), then two
   `optics_sighting(star_id)` events that pair to a `p52_mark_align`
   call (see `scenario.rs:960-1003`). Pick two non-collinear catalogue
   star indices (e.g. `1` and `10` — the developer should pick by
   checking `STAR_CATALOG` directions and avoiding near-collinearity).
4. **Assert** post-P52:
   - `state.refsmmat != refsmmat_before` (non-trivial change),
   - `state.imu_alignment_state == ImuAlignmentState::FineAligned`,
   - `state.refsmmat` is orthonormal: for each pair `(i, j)`,
     `|sum_k R[i][k] * R[j][k] - δ_ij| < 1e-10`.
5. **Snapshot the new REFSMMAT**:
   `let refsmmat_after_p52 = state.refsmmat;`
6. Set up a P40 burn at a known attitude:
   - `command_attitude(q_burn)` where `q_burn` is identity (vehicle
     X-axis = inertial +X). Snap-and-hold so the simulator's PIPA
     output is along the platform Y-axis (the simulator's default
     `thrust_dir_platform`).
   - Seed `state.pending_maneuver` directly via the V25 N33 / V25 N81
     keystroke path, **or** call `p30_load_dv_lvlh(&mut state,
     tig, dv_crew)` directly from the test (simpler — no DSKY
     state-machine to drive). TIG = current MET + 60 s, crew ΔV =
     `[+5.0, 0.0, 0.0]` (5 m/s along-track) — a small SPS burn well
     above the 0.5 m/s threshold.
   - V37 E40 E + PRO to arm.
7. Walk the burn loop (same pattern as Test 1) **for exactly one
   SERVICER cycle after ignition** (≈ 2 s of engine-on, plus the
   pre-ignition wait). After the first SERVICER cycle past TIG fires,
   stop the loop manually (don't let the burn run to cutoff — we
   only need *one* `servicer_last_dv_inertial` sample).
8. Read `dv_inertial_observed = state.servicer_last_dv_inertial`.
9. **Assert** that `dv_inertial_observed` was rotated through
   `refsmmat_after_p52`, not through the identity (`refsmmat_before`).

### 6.3 Decisive invariant

The cleanest mathematical assertion is:

> The platform-frame ΔV recovered from the inertial ΔV via the new
> REFSMMAT inverse must agree with the simulator's commanded thrust
> direction; the same recovery through the *old* REFSMMAT must
> disagree.

In pseudo-code:

```text
// platform thrust direction is +Y in the simulator's default Spacecraft model
// (see physics.rs: thrust_dir_platform = [0, 1, 0]).
//
// During one SERVICER cycle of 2 s at 1.5 m/s² (default SimHardware SPS),
// the AGC sees ~3 m/s in platform Y (after PIPA quantisation, exact value
// depends on residue carry-over; see §9 open question).
dv_platform_recovered = mxv(transpose(state.refsmmat), dv_inertial_observed)
dv_platform_via_old   = mxv(transpose(refsmmat_before), dv_inertial_observed)
// dv_platform_via_old is the wrong recovery — if REFSMMAT didn't propagate,
// these two would match.
```

Assertions:

| Assertion                                                              | Tolerance              |
|------------------------------------------------------------------------|------------------------|
| `dv_platform_recovered[0].abs()` (off-axis X)                          | `< 0.1 m/s`            |
| `dv_platform_recovered[1]` (on-axis Y)                                 | `> 2.0 m/s` (positive, magnitude in expected range) |
| `dv_platform_recovered[2].abs()` (off-axis Z)                          | `< 0.1 m/s`            |
| `‖dv_platform_recovered - dv_platform_via_old‖` (bug-catcher: must differ) | `≥ 0.5 m/s`        |
| `state.refsmmat == refsmmat_after_p52` (element-wise)                  | exact equality (no SERVICER write-back) |
| `state.alarm.code`                                                     | `0`                    |

The third row (off-axis Z) and the bug-catcher row are the active
parts. If the SERVICER were using the cached old REFSMMAT, the
recovered platform ΔV through the *current* REFSMMAT would not look
like a clean +Y thrust; it would look like a +Y thrust rotated by
`M_TRUTH^T · I` ≠ identity, i.e. with a significant X-component.

### 6.4 Choosing `M_TRUTH`

A 30° rotation about +Z is sufficient — far from identity, easy to
sanity-check, well-conditioned. The developer should write `M_TRUTH`
as a literal:

```text
M_TRUTH = [
  [ cos(30°), -sin(30°), 0.0],
  [ sin(30°),  cos(30°), 0.0],
  [      0.0,       0.0, 1.0],
]  // ≈ [[0.866, -0.500, 0.0], [0.500, 0.866, 0.0], [0.0, 0.0, 1.0]]
```

This is the matrix passed to `seed_truth_refsmmat`. The scenario's
`optics_sighting` machinery rotates the two catalogue-star directions
through `M_TRUTH` to obtain the platform-frame measurements, then
`p52_mark_align` reconstructs `M_TRUTH` via the TRIAD method (which is
exact up to numerical roundoff for well-conditioned inputs). The unit
test `p51_p52::tests::tc_p51_4_orthonormal_refsmmat` already exercises
this construction.

---

## 7. Tolerance summary table

Single reference for all numeric tolerances above, so the developer
can grep:

| Test | Field                                                | Tolerance         |
|------|------------------------------------------------------|-------------------|
| 1    | `csm_state` position drift across P00 (per axis)     | `< 1.0 m`         |
| 1    | `csm_state` velocity drift across P00 (per axis)     | `< 0.001 m/s`     |
| 1    | Achieved burn ΔV magnitude vs target                 | `< 5.0 m/s`       |
| 2    | P27-applied state vs oracle position (per axis)      | `< 1.0 m`         |
| 2    | P27-applied state vs oracle velocity (per axis)      | `< 1.0e-6 m/s`    |
| 2    | `pending_maneuver.delta_v` magnitude                 | `2.35 ± 1.0e-6 m/s` |
| 2    | "What P30 would have produced from uncorrected state" — L2 distance from actual `pending_maneuver.delta_v` | `≥ 1.0e-4 m/s` (must differ) |
| 3    | Position handover continuity (per Euler ref)         | `< 1.0 m`         |
| 3    | Velocity handover continuity (per Euler ref)         | `< 1.0e-3 m/s`    |
| 4    | Off-axis (X, Z) platform ΔV after recovery via new REFSMMAT | `< 0.1 m/s`  |
| 4    | On-axis (Y) platform ΔV after recovery via new REFSMMAT     | `> 2.0 m/s`  |
| 4    | Recovery via old REFSMMAT — distance from recovery via new | `≥ 0.5 m/s` (must differ) |
| 4    | REFSMMAT orthonormality after P52                   | `< 1.0e-10`       |

---

## 8. Gap list

Items the developer may discover are missing. **Do not** speculatively
extend the builder; flag them in code comments and proceed using the
proxy specified.

| Gap                                                                                                  | Severity | Workaround used in this spec |
|------------------------------------------------------------------------------------------------------|----------|-------------------------------|
| `soi_check` is not called from `average_g_step` (issue #51) — AGC stays in stale frame across SOI    | medium   | Test 3 calls `propagate_coast` directly from the test and asserts on its return value. The AGC's `csm_state` is irrelevant for Test 3; that gap is explicitly out-of-scope here. |
| No scenario-builder method to assert `state.csm_state.frame`                                         | minor    | Tests read `state.csm_state.frame` directly via the `&mut state` handle after `run_scenario` returns. |
| No scenario-builder method to assert a REFSMMAT value                                                | minor    | Test 4 reads `state.refsmmat` directly and compares element-wise. |
| `Event::OpticsSighting` always routes to `p52_mark_align` regardless of `major_mode` (see `scenario.rs:993-1001`) | minor    | Test 4 sets `imu_alignment_state = FineAligned` and uses the existing routing — exactly the path the comment in scenario.rs promises is sufficient for now. |
| No `moon_velocity` function in `agc-core` (issue #52)                                                | minor    | Use central-difference of `moon_position` with `δ = 10 s`, exactly as `physics.rs:285-296`. Define a `moon_velocity_central_diff` helper at the top of the test file. |
| P23 mark plumbing not driven from the scenario builder                                                | medium   | Test 2 uses the V71 P27 uplink path as the proxy for "P23 has corrected the state". Explicitly documented in test doc-comment. P23 filter math is unit-tested elsewhere. |
| `Event::SeedState` cannot directly set `imu_alignment_state`                                          | minor    | Tests 1 and 4 set `state.imu_alignment_state` directly via the `&mut state` they hold, between scenario phases. |
| `ctx.ground_truth` is private to the scenario runner                                                 | minor    | Test 3 does not depend on it (uses direct `propagate_coast`). If a future test needs to read it, file an issue to expose a `expect_ground_truth_frame(Frame)` builder method. |

None of these require new builder API in this PR. If any subsequent
test needs them, file a separate issue.

---

## 9. Open questions

1. **Test 2 P27 word layout**: the spec assumes P27 words map cleanly
   to integer-scaled position (km) and velocity (m/s) via
   `p30_load_dv_lvlh` style. The developer must verify this against
   `v_n.rs:1248` (`p27_apply_word`) before finalising the test —
   specifically the exact scaling factor and signedness for each of
   the six words. If the encoding requires fixed-point fractional
   parts beyond ULP rounding, widen Test 2's tolerances to absorb
   that and document the change in a code comment.

2. **Test 4 expected platform ΔV magnitude**: the value "≈ 3.0 m/s in
   one cycle" depends on `SimHardware`'s default SPS acceleration
   (1.5 m/s² × 2 s = 3.0 m/s nominal, less 1 PIPA quantum residue
   carry-forward). The actual per-cycle ΔV after PIPA quantisation may
   be closer to 2.94 m/s (≈50 pulses × 0.0585 m/s/pulse + residue).
   The chosen assertion (`dv_platform_recovered[1] > 2.0 m/s`) is
   generous enough to absorb this; if a tighter bound is desired,
   the developer should run once, observe the value, and lock it
   with a `±0.05 m/s` band. Flag in a code comment.

3. **Test 3 Euler-vs-RK4 drift**: at `dt = 2 s` and gravity
   `≈ 0.003 m/s²` (1 km outside SOI), Euler and RK4 disagree by
   `~ 0.5 * dt^2 * jerk ≈ 1 nm` in position and `~ µm/s` in
   velocity. The 1 m / 1 mm/s tolerance is comfortable. If the
   developer wants to be ultra-rigorous, replace the Euler reference
   step with a direct RK4 step that *bypasses* `soi_check` (i.e.
   reuse `rk4_cowell_step` if it is `pub`, or accept the Euler
   approximation). Confirm before implementation.

---

## 10. Estimated size

- `handoffs.rs`: ~500–650 lines including doc comments and inline
  rationale. About 100 lines per test plus 100 lines of shared helpers
  (`moon_velocity_central_diff`, ECI/MCI fixture builders, ΔV recovery
  helpers).

- Wall-clock test time: each test should complete in `< 2 s`. Test 1
  (the burn loop) is the longest; the others are essentially
  single-scenario runs with arithmetic.
