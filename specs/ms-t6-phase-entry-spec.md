# MS-T6 — Entry-phase end-to-end integration test

**Status**: Draft, for developer consumption
**Implements**: GitHub issue #29
**Target files**:
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/agc-sim/src/physics.rs` (extend)
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/agc-test/tests/phase_entry.rs` (new)

**Dependencies**:
- MS-T4 phase tests landed; `agc_sim::scenario` API stable.
- Entry-guidance #3 / MS-E7 complete (`agc-core::programs::p61_p67`,
  `agc-core::guidance::entry`, `agc-core::navigation::atmosphere`).
- `agc-test::entry_sim::EntryIntegrator` available as the reference aero model.

---

## 1. Goal / scope

Drive the AGC end-to-end through the entry-phase program sequence
`P61 → P62 → P63 → (P64 closed loop) → P67 (drogue)` via the
`ScenarioBuilder` API — using nothing but the **scenario-runner-driven**
pipeline. The simulator's ground-truth propagator
(`agc_sim::physics::advance_ground_truth`) must apply atmospheric drag
and aerodynamic lift to the spacecraft state so that the AGC's PIPA
pipeline observes the sensed-Δv that the entry-guidance closed loop
needs. The bank angle commanded by the AGC (`DapMode::EntryRoll(_)`)
must flow back into the simulator so it shapes the trajectory.

The acceptance criterion is the same shape as the existing `entry_e2e.rs`:
the AGC reaches drogue deploy within the documented footprint
(direct-LEO 1000 km, lunar-return 3000 km) and ends in `EntryPhase::Final`.

**What this test is NOT**: a replacement for `entry_e2e.rs`. That file
exercises the AGC + `EntryIntegrator` pipeline directly (bypassing the
scenario runner). MS-T6 closes the gap by routing the same physics
through the scenario runner so that `phase_entry.rs` chains cleanly into
the MS-T7 full-mission walkthrough.

---

## 2. Conventions used in this spec

- All file paths absolute. The developer adds exactly two source-tree
  changes (one extension, one new test file).
- All tolerances are numeric, not vague. Any deviation requires a spec
  amendment.
- Re-use rather than reimplement: the aerodynamic math already exists in
  `agc-test/src/entry_sim.rs::EntryIntegrator::acceleration`
  (line 144). The Part 1 work is a *relocation + wiring*, not a
  redesign of the force model.

---

## 3. Part 1 — `agc-sim/src/physics.rs` extension

### 3.1 New `Spacecraft` fields

Extend the existing struct (currently declared at
`/Users/Juergen.Schiewe/dev/AGC-in-Rust/agc-sim/src/physics.rs:108-158`):

```rust
pub struct Spacecraft {
    // ... existing fields unchanged ...

    /// Reference area (m²) for the drag / lift forces.
    /// Defaults to `apollo_cm::AREA_M2` (12.0).
    pub ref_area_m2: f64,

    /// Hypersonic drag coefficient. Defaults to `apollo_cm::CD` (1.3).
    pub cd: f64,

    /// Hypersonic vertical L/D ratio. Defaults to `apollo_cm::LD` (0.30).
    /// Used as the **magnitude** of the L/D vector; the AGC's
    /// `state.entry.ld_command` provides the signed *fraction* of this
    /// magnitude actually flown (range `[-1, +1]` after the per-cycle clamp).
    pub ld_hypersonic: f64,

    /// Last commanded bank angle (radians, 0 = lift up, +ve = right-bank).
    ///
    /// Written by `advance_ground_truth` from the AGC's
    /// `DapMode::EntryRoll(_)` — see [`apply_bank_from_agc`]. Exposed
    /// publicly so tests can inspect / override it for unit cases.
    pub bank_rad: f64,
}
```

The `apollo_cm` constant module already exists at
`agc-sim/src/physics.rs:70-79`. Extend it with the three new constants
(copy the values verbatim from `agc-test/src/entry_sim.rs:53-61`):

```rust
pub mod apollo_csm { /* existing */ }

pub mod apollo_cm {
    /// CM mass after CSM separation (kg). Source: entry_sim.rs:53.
    pub const MASS_KG: f64 = 5_800.0;
    /// CM heat-shield reference area (m²). Source: entry_sim.rs:57.
    pub const AREA_M2: f64 = 12.0;
    /// Hypersonic drag coefficient. Source: entry_sim.rs:61.
    pub const CD: f64 = 1.3;
    /// Hypersonic L/D. Matches the `0.30` constant hard-coded in
    /// EntryIntegrator's `tc_esim_*` unit tests (entry_sim.rs:274 etc.).
    pub const LD: f64 = 0.30;
}
```

`Spacecraft::new()` (currently at line 169) must initialise the four new
fields from these constants. `atmosphere_enabled` defaults to `false`
(unchanged), so existing tests remain green.

### 3.2 Aerodynamic force model

Add a free function (or `impl Spacecraft` method) inside
`agc-sim/src/physics.rs`:

```rust
/// Compute the sensed (non-gravitational) acceleration on the
/// spacecraft at the given inertial state.
///
/// Returns the drag + lift acceleration in the **inertial** frame (m/s²).
/// Pure function of inputs — no shared state — so unit tests can call
/// it without constructing a full `Spacecraft`.
///
/// Bank convention: `bank_rad = 0` → lift radially outward (up);
/// `bank_rad = π/2` → lift to the right of velocity. Matches
/// `EntryIntegrator::acceleration` at entry_sim.rs:144-198.
pub fn aero_acceleration_inertial(
    sc: &Spacecraft,
    position_eci: [f64; 3],
    velocity_eci: [f64; 3],
    bank_rad: f64,
) -> [f64; 3];
```

**The body of `aero_acceleration_inertial` is a direct port of
`EntryIntegrator::acceleration` minus the gravity term**, because
gravity is owned by `propagate_coast` (see §3.3). Specifically the
returned vector is the existing `a_sensed = a_drag + a_lift` quantity
computed at lines 170-195 of entry_sim.rs. The developer should:

- copy the body of `EntryIntegrator::acceleration` into the new
  function;
- delete the `a_grav` calculation and the `a_full` return component;
- replace `self.mass_kg / cd / ref_area_m2` with the equivalent
  `sc.*` field reads;
- replace `ld_command` (a function argument in the original) with
  `sc.ld_hypersonic` so the new helper takes a single AGC-side scalar.
  The signed `ld_command` from the AGC flows in via `bank_rad` and the
  sign of the lift fraction (see §3.4 for the wiring detail);
- reuse `agc_core::navigation::atmosphere::density` exactly as today
  (atmosphere.rs:47).

The existing local `vec3_*` helpers in entry_sim.rs:233-260 are not
public. The developer should either (a) re-implement the 6 inline
helpers privately in physics.rs (preferred — matches the
"no cross-file coupling" pattern in MS-T5) or (b) call into
`agc_core::math::linalg::{cross, norm, unit}` which exist
(grep `agc-core/src/math/linalg.rs`). Choose option (b) when the math
crate already provides the operation; option (a) only for the missing
helpers.

### 3.3 Wire forces into `advance_ground_truth`

The current `advance_ground_truth` (physics.rs:267-311) runs an RK4
Cowell step via `propagate_coast`, then handles SOI. For MS-T6 the
function must additionally apply the aerodynamic Δv when
`sc.atmosphere_enabled == true`.

**Force-of-design choice**: there are two viable models. Pick **Model A**:

| Model | How aero is applied | Pro | Con |
|---|---|---|---|
| **A — Operator split** | After `propagate_coast`, add `aero_accel × dt` to `state.velocity`. Position is then nudged by `½·aero_accel·dt²` for second-order accuracy. | One line. Reuses RK4 gravity propagator unchanged. Same operator-split structure the SERVICER's `average_g_step` uses. | Splits gravity + aero across the step. Acceptable when `dt ≤ servicer period`. |
| B — Full RK2 with aero in the derivative | Replace `propagate_coast` with an RK2 sub-stepped integrator that has gravity + drag + lift in its `acceleration()` (mirror of `EntryIntegrator`). | Mechanically identical to EntryIntegrator. | Duplicates entire integrator; requires a parallel gravity model. |

**Mandatory constraint**: `Scenario::coast_step_cs` must be set to **200 cs**
(one SERVICER cycle = 2 s) when the test runs the entry phase. The outer
coast loop in scenario.rs (lines 893-919) advances the ground truth by
`outer_dt_s` between SERVICER cycles; for entry, 2 s sub-stepping
matches the SERVICER cadence and aligns with `EntryIntegrator`'s 2 s
`integrate_cycle` window (entry_sim.rs:105). At 2 s outer-step granularity
Model A's operator-split error is below the existing 1000 km / 3000 km
footprint tolerance — confirmed by the fact that the SERVICER itself uses
the same split (its `average_g_step` is 2nd-order trapezoidal on a 2 s
window per `agc-core::services::average_g`).

**Implementation sketch for Model A** (replaces lines 280-282 of
physics.rs:267):

```rust
let propagated = propagate_coast(*state, dt, moon_pos);
*state = propagated;

if sc.atmosphere_enabled {
    let a_sensed = aero_acceleration_inertial(
        sc,
        state.position,
        state.velocity,
        sc.bank_rad,
    );
    // Velocity kick (Euler-forward on the sensed accel).
    state.velocity[0] += a_sensed[0] * dt;
    state.velocity[1] += a_sensed[1] * dt;
    state.velocity[2] += a_sensed[2] * dt;
    // Second-order position trim — matches the trapezoidal SERVICER.
    let half_dt2 = 0.5 * dt * dt;
    state.position[0] += a_sensed[0] * half_dt2;
    state.position[1] += a_sensed[1] * half_dt2;
    state.position[2] += a_sensed[2] * half_dt2;
}

// SOI check etc. — unchanged.
```

### 3.4 Bank command bridge

The AGC commands bank via the DAP mode `DapMode::EntryRoll(bank_rad)`
(defined at `agc-core/src/control/dap.rs:38`, set inside
`entry_servicer_exit` at `agc-core/src/programs/p61_p67.rs:381`).
The scenario runner already reads it for the existing entry pipeline
(entry_scenario.rs:67-71).

Add a helper in `agc-sim/src/physics.rs`:

```rust
/// Update `sc.bank_rad` from the AGC's DAP state. Called by the
/// scenario runner once per outer coast step before
/// `advance_ground_truth` consumes `sc.bank_rad`.
///
/// - `DapMode::EntryRoll(b)` → `sc.bank_rad = b`
/// - any other DAP mode → leaves `sc.bank_rad` unchanged
///   (the AGC has not entered entry guidance yet; pre-entry default
///   is 0.0, i.e. lift up, set by `Spacecraft::new`).
pub fn apply_bank_from_agc(sc: &mut Spacecraft, state: &AgcState);
```

Wiring point in the scenario runner (`agc-sim/src/scenario.rs`,
`AdvanceCoast` arm starting at line 841):

- **Before** the existing `advance_ground_truth(&mut ctx.spacecraft, gt, outer_dt_s)`
  call at line 899, call `apply_bank_from_agc(&mut ctx.spacecraft, state)`.
- Also wire the AGC's signed L/D into the lift magnitude. The cleanest
  way is to extend `Spacecraft` with a `ld_fraction` field that
  `apply_bank_from_agc` writes from `state.entry.ld_command`, and have
  `aero_acceleration_inertial` multiply `drag_mag * ld_fraction` (so a
  negative `ld_command` produces lift *toward* Earth — the half-lift-down
  case the AGC uses in skip phase). This matches the existing
  `EntryIntegrator::integrate_cycle(ld_command=…)` argument.

**Key state-field references** (developer should cite these in code
comments so future readers can trace the contract):
- AGC bank command: `agc-core/src/control/dap.rs:38` (variant), set in
  `agc-core/src/programs/p61_p67.rs:381` and `:418`.
- AGC L/D command: `agc-core/src/programs/p61_p67.rs:372`
  (`state.entry.ld_command`).
- AGC entry phase: `agc-core/src/programs/p61_p67.rs:58-88`
  (`EntryPhase`).

### 3.5 Unit tests for the physics extension

Co-locate in the existing `#[cfg(test)] mod tests` block of physics.rs.
Mirror the spirit of `entry_sim.rs::tc_esim_*` but on the agc-sim
API surface:

| Test | What it pins |
|---|---|
| `tc_phys_aero_vacuum_no_sensed` | At h ≥ 250 km `aero_acceleration_inertial` returns `[0, 0, 0]` to within 1e-10 m/s². |
| `tc_phys_aero_peak_decel` | At h = 50 km, V = 7800 m/s along +Y, `‖aero_accel‖` lies in 5..50 m/s² (a few g). Drag is anti-velocity (negative-Y component dominant). |
| `tc_phys_aero_bank_zero_lift_radial` | At bank=0, L/D=0.30, lift component on the radial-outward axis is **positive** (matches entry_sim.rs:313 `tc_esim_3`). |
| `tc_phys_advance_ground_truth_aero_disabled_no_change` | With `atmosphere_enabled = false`, an entry-altitude state is bit-identical between the new code and the old `propagate_coast`-only path (regression guard for the existing TLI/coast tests). |
| `tc_phys_apply_bank_from_agc_entry_roll` | After `state.dap_state.mode = DapMode::EntryRoll(0.7)`, calling `apply_bank_from_agc` sets `sc.bank_rad = 0.7`. Other modes leave it untouched. |

No additional fixture data needed; all five tests can run with synthetic
state vectors.

---

## 4. Part 2 — scenario builder updates (minimal)

The existing `ScenarioBuilder` API in
`/Users/Juergen.Schiewe/dev/AGC-in-Rust/agc-sim/src/scenario.rs` is
sufficient for MS-T6 with **one new method**:

```rust
impl ScenarioBuilder {
    /// Enable atmospheric drag + lift in the ground-truth propagator.
    ///
    /// Sets `Spacecraft::atmosphere_enabled = true` and configures the
    /// outer coast-step granularity to 200 cs (one SERVICER cycle).
    /// Must appear after `seed_ground_truth` (it relies on the
    /// `RunContext::spacecraft` having been bound).
    pub fn enable_atmosphere(self) -> Self;
}
```

This requires a new `Event::EnableAtmosphere` variant and a corresponding
arm in `run_scenario`. The arm sets
`ctx.spacecraft.atmosphere_enabled = true` and forces `coast_step_cs = 200`
(overriding the default 6000 cs). Forcing the step size here is safer
than asking the test author to call `.coast_step_cs(200)` manually; the
combination of "atmosphere on" + "60 s outer step" would silently push
miss-distance above the footprint threshold.

**Assertion of drogue deploy / footprint**: the existing
`expect_dsky` / `expect_major_mode` events are not sufficient because
the miss distance is computed via haversine (entry_sim.rs:221) and is
not surfaced through the DSKY. Add **one** new builder method:

```rust
impl ScenarioBuilder {
    /// Assert `state.entry.drogue_deployed == true` and the great-circle
    /// distance from the sub-satellite point to
    /// `(state.entry.target_lat_rad, state.entry.target_lon_rad)` is
    /// below `miss_km_tol`.
    pub fn expect_drogue_within(self, miss_km_tol: f64) -> Self;
}
```

The implementation re-uses `haversine_km` (entry_sim.rs:221) and
`sub_satellite_lat_lon` (entry_scenario.rs:198). Since `agc-sim`
cannot depend on `agc-test`, the implementation duplicates these two
small helpers privately inside `agc-sim/src/scenario.rs` (the same
"no cross-file coupling" rationale as MS-T5).

No further builder additions are needed. In particular, do NOT add:
- `expect_csm_altitude_below` (drogue check covers it),
- per-phase `expect_entry_phase` (drogue + `expect_major_mode(67)` covers it),
- explicit `set_target_landing_site` (the test seeds it directly via
  `state.entry.target_lat_rad = …` after building the AGC state).

---

## 5. Part 3 — `agc-test/tests/phase_entry.rs` test design

### 5.1 File layout

One file, two `#[test]` functions:

| Function | Scenario | Miss-distance threshold |
|---|---|---|
| `tc_phase_entry_direct_leo` | direct entry from 200 km LEO, FPA = −6°, V = 7900 m/s | 1000 km |
| `tc_phase_entry_lunar_return` | translunar return, FPA = −6°, V = 11 000 m/s | 3000 km |

Both reuse the initial-state factories already in `entry_scenario.rs`:
- `setup_state_direct_leo()` (line 183)
- `setup_state_lunar_return()` (line 192)

These factories are `pub` and importable from agc-test. **Do not
duplicate** their bodies.

### 5.2 Test pattern (per-test pseudocode)

```rust
fn run_entry_phase_scenario(
    name: &'static str,
    seed_state: AgcState,
    miss_km_tol: f64,
) {
    let mut state = seed_state;
    let mut hw = SimHardware::new();

    // Build a single scenario that drives P61 → P63 and then waits for the
    // SERVICER to walk through P64/P65/P66 → P67 autonomously.
    let scenario = ScenarioBuilder::new(name)
        // 1. Initial state is already in `state`. Re-seed the ground truth
        //    so the scenario runner has a reference trajectory.
        .seed_ground_truth(state.csm_state)
        // 2. Atmosphere on, outer step pinned to 200 cs (one SERVICER cycle).
        .enable_atmosphere()
        // 3. V37 E61 E — selects P61 (init_p61).
        //    init_p61 sets phase=Preparation, major_mode=61.
        .verb_noun(37).digits(61).enter()
        .expect_major_mode(61)
        // 4. V37 E62 E — selects P62 (init_p62, CM/SM separation).
        .verb_noun(37).digits(62).enter()
        .expect_major_mode(62)
        // 5. V37 E63 E — selects P63 (installs entry_servicer_exit hook).
        .verb_noun(37).digits(63).enter()
        .expect_major_mode(63)
        // 6. Coast forward up to MAX_SCENARIO_DURATION_S (20 min) — the
        //    SERVICER's entry_servicer_exit autonomously drives
        //    PreEntry → Entry → (Skip|Ballistic|Final) → drogue deploy.
        .advance_coast(SimDuration::seconds(20 * 60))
        // 7. Acceptance assertion.
        .expect_drogue_within(miss_km_tol)
        .build();

    run_scenario(&scenario, &mut state, &mut hw);
}

#[test]
fn tc_phase_entry_direct_leo() {
    run_entry_phase_scenario(
        "phase_entry/direct_leo",
        setup_state_direct_leo(),
        1_000.0,
    );
}

#[test]
fn tc_phase_entry_lunar_return() {
    run_entry_phase_scenario(
        "phase_entry/lunar_return",
        setup_state_lunar_return(),
        3_000.0,
    );
}
```

Notes on the keystroke pattern:
- `V37 E61 E`, `V37 E62 E`, `V37 E63 E` use `verb_noun(37)` which
  expands to `Key::Verb, Digit(3), Digit(7)` (scenario.rs:494). The
  `.digits(61).enter()` expansion handles the two-digit program number.
- After V37 E63 E the SERVICER drives the rest. P64 is reached when
  the 0.05g threshold trips inside `p63_check_threshold`
  (p61_p67.rs:411). P67 is reached when HUNTEST predicts converged
  range — both transitions happen inside the SERVICER's
  `entry_servicer_exit` hook (p61_p67.rs:333). **The test never
  manually invokes P64/P65/P66/P67.**
- The advance_coast duration is `MAX_SCENARIO_DURATION_S = 20 min`
  (entry_scenario.rs:27). Real entries finish in 7-10 min; the 20 min
  cap is a hang-guard. The `expect_drogue_within` assertion will fail
  loudly if the AGC didn't reach `Final` within that window.

### 5.3 No manual burn loop

Unlike `p40_sps_burn.rs` (which has the explicit "Walk the remaining
1 s and the full burn at 100 ms granularity" loop at lines 247-269),
entry-phase is entirely SERVICER-driven once `init_p63` installs the
hook. The scenario runner's `AdvanceCoast` event already runs
inner-tick `WaitlistPump` calls at 200 cs cadence — that is the only
loop the SERVICER needs to fire.

The only departure from the default `AdvanceCoast` behaviour is the
outer-step pinning to 200 cs (one SERVICER cycle). At 60 s outer steps
(the default), aero forces would be evaluated once per minute — far
too coarse for the 7-minute entry. `enable_atmosphere` forces 200 cs to
guarantee the aero kick is applied 600 times across a 20-min coast,
matching `EntryIntegrator`'s 1-cycle granularity.

### 5.4 Tick granularity

Inside the 200 cs outer step the inner loop uses the default
`tick_cs = 10` (scenario.rs:398). This produces 20 inner ticks per
outer step, which is consistent with the
`agc-test/src/entry_sim.rs:49` `SUB_STEP_S = 0.1` constant
(20 sub-steps per SERVICER cycle). No `tick_cs` override needed.

---

## 6. Assertion table

| # | Site | Assertion | Source / threshold |
|---|---|---|---|
| 1 | Scenario step 3 | `state.major_mode == 61` | `init_p61` sets `P61_MAJOR_MODE` (p61_p67.rs:20). |
| 2 | Scenario step 4 | `state.major_mode == 62` | `init_p62` sets `P62_MAJOR_MODE` (p61_p67.rs:21). |
| 3 | Scenario step 5 | `state.major_mode == 63` | `init_p63` sets `P63_MAJOR_MODE` (p61_p67.rs:22). |
| 4 | Final | `state.entry.drogue_deployed == true` | `p67_deploy_drogue` (p61_p67.rs:471). |
| 5 | Final | `state.major_mode == 67` | `init_p67` sets `P67_MAJOR_MODE` (p61_p67.rs:24). |
| 6 | Final | great-circle miss ≤ `miss_km_tol` | Haversine on `(target_lat_rad, target_lon_rad)` vs sub-satellite point of `state.csm_state.position`. |

Assertions 1-3 are in-scenario `expect_major_mode` events; assertions
4-6 land in the new `expect_drogue_within` event.

`expect_drogue_within` must produce a diagnostic message in the same
shape as the existing scenario.rs `fail_prefix`:
```
scenario "<name>": event #<idx> (ExpectDrogueWithin) failed at MET …:
  drogue not deployed (phase=…); miss=… km > tol=… km;
  landed lat=… lon=…
```
Include `state.entry.phase`, the haversine distance, and the
sub-satellite lat/lon. Match the verbose tail diagnostic format of
`entry_e2e.rs:84-95`.

---

## 7. Open questions / consultation requests

### 7.1 Apollo 8 fidelity of the initial state vectors

The existing `setup_state_lunar_return()` (entry_scenario.rs:192) uses
**synthetic equatorial conditions**:
- entry interface at `(lat=0°, lon=0°, alt=122 km)`,
- V = 11 000 m/s, FPA = −6°,
- target = `(lat=0°, lon=45°E)` (Pacific notional, ≈5004 km downrange).

The Apollo 8 actual entry interface, per the Apollo 8 Mission Report
MSC-PA-R-69-1:
- V ≈ 10 825 m/s (just below 11 km/s),
- FPA ≈ −6.48°,
- Splashdown 8°N 165°W (Pacific).

**Recommendation**: keep the synthetic equatorial fixture for MS-T6 — it
matches `entry_e2e.rs` exactly, the 3000 km tolerance already absorbs
the geometry difference, and switching to lat=8°N 165°W would force a
re-derivation of the entry-interface state vector (non-trivial because
the existing `make_initial_state` factory is hard-coded to place the
spacecraft on the +X axis). Recording this choice in the test's
module-doc comment is enough.

**Consultation request to orbital-mechanics**: confirm whether the
1000 km direct-LEO and 3000 km lunar-return tolerances inherited from
entry_e2e.rs:44/60 are still adequate when the same physics flows
through the scenario runner's 200 cs outer step (vs. EntryIntegrator's
internal 100 ms RK2 sub-steps). The two integrators are mathematically
similar but not identical (RK4-Cowell + Euler operator-split kick vs.
RK2 midpoint). If the operator-split error is suspected to consume more
than ~10 % of the budget, raise the lunar-return tolerance to 3500 km.

The user has indicated they will route this question to
**orbital-mechanics** before the developer finalises the test. If the
analyst says "use Apollo 8 real splashdown", a follow-up issue should
adapt `make_initial_state` rather than blocking MS-T6.

### 7.2 Operator-split vs. full-RK2 integrator

§3.3 picks the operator-split model (Model A). This is a deliberate
"reuse the gravity propagator we already have" call. If the developer
finds during implementation that the operator-split error pushes the
direct-LEO scenario above its 1000 km tolerance, they should:

1. Tighten the outer step to 100 cs (1 s) in `enable_atmosphere`.
2. Only as a last resort, switch to Model B (full RK2 with combined
   gravity + aero). This would mean replacing the `propagate_coast`
   call in `advance_ground_truth` for the entry branch with an
   `EntryIntegrator`-style RK2 stepper — bigger surgery, raise an issue
   before doing it.

---

## 8. Gap list and rationale

### 8.1 Implemented by this spec

- Aerodynamic drag + lift in `agc-sim::physics`.
- Bank-command flow from AGC `DapMode::EntryRoll(_)` into the
  simulator's ground-truth propagator.
- One new scenario-builder event (`enable_atmosphere`) and one new
  assertion event (`expect_drogue_within`).
- One new test file (`phase_entry.rs`) with two `#[test]` functions
  (direct-LEO + lunar-return).

### 8.2 NOT in scope for MS-T6

- **Earth-rotation correction**: `v_rel = v_inertial`. Same
  simplification as `EntryIntegrator` (entry_sim.rs:17). The ~470 m/s
  equatorial slip is below the 1000 / 3000 km footprint resolution.
- **Apollo 8 real splashdown coordinates** (8°N 165°W). See §7.1.
- **Sphericity / J2 correction during entry**: irrelevant on a 7-minute
  arc. Matches EntryIntegrator's omission.
- **Lunar entry**: Not applicable — entry is always at Earth.
- **Drogue → main-chute → splashdown sequencing**: MS-T6 ends at drogue
  deploy. Anything past that is out of scope for the AGC entirely
  (handled by SECS pyrotechnics on the real vehicle).
- **Backup S/C control by crew**: closed-loop only.

### 8.3 Proxies the test uses

- `expect_drogue_within` re-implements haversine + sub-satellite lat/lon
  locally rather than depending on `agc-test` from `agc-sim` (would
  introduce a circular dep).
- The Apollo-CM aero constants (mass 5800 kg, area 12 m², CD=1.3, L/D=0.30)
  are duplicated between `agc-test/src/entry_sim.rs` and the new
  `agc-sim::physics::apollo_cm` module. A future cleanup could move
  them to `agc-core` (since `agc-core::navigation::atmosphere` already
  lives there), but that's out of scope for MS-T6 — file the issue
  when needed.

### 8.4 Existing tests left untouched

- `agc-test/tests/entry_e2e.rs` — unchanged. Continues to exercise the
  AGC + `EntryIntegrator` pipeline directly. MS-T6 is a parallel
  scenario-runner-driven path, not a replacement.
- `agc-test/tests/entry_e2e_vagc.rs` — unchanged.
- All other `phase_*.rs` tests — unchanged. The new `Spacecraft` fields
  default to "atmosphere off" so coast-only scenarios retain bit-identical
  trajectories. The new `tc_phys_advance_ground_truth_aero_disabled_no_change`
  test in §3.5 is the regression guard.

---

## 9. Acceptance criteria (issue #29 close conditions)

The implementation is complete when:

1. `agc-sim/src/physics.rs` compiles with the four new `Spacecraft`
   fields, the new `apollo_cm` constant module, `aero_acceleration_inertial`,
   `apply_bank_from_agc`, and the five unit tests from §3.5 — all five
   pass.
2. `agc-sim/src/scenario.rs` exposes `ScenarioBuilder::enable_atmosphere`
   and `ScenarioBuilder::expect_drogue_within` and the corresponding
   `Event` variants and `run_scenario` arms.
3. `agc-test/tests/phase_entry.rs` contains the two `#[test]` functions
   in §5 and both pass without `#[ignore]`.
4. `cargo test -p agc-sim` continues to pass (no regressions in
   coast / TLI / LOI / TEI phases).
5. `cargo test -p agc-test --test entry_e2e` continues to pass (parallel
   path unchanged).
6. `cargo clippy --all-targets -- -D warnings` clean.

The original mission-plan MS-T6 exit criterion ("closed-loop entry
scenario lands within target footprint") is satisfied by criterion #3
inheriting the same 1000 / 3000 km thresholds as MS-E7.
