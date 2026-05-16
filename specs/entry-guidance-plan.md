# Entry Guidance Implementation Plan (P63–P67)

**Status**: Approved
**Scope**: Close the largest remaining functional gap in the AGC core — the entry-guidance math that flies the Command Module from 0.05g entry interface to drogue deploy.
**Module owner**: `agc-core/src/guidance/entry.rs` (new content) and `agc-core/src/programs/p61_p67.rs` (thin orchestration).
**AGC source files**:
- `Comanche055/P61-P67.agc` — major-mode entry blocks
- `Comanche055/REENTRY_CONTROL.agc` — entry control law (HUNTEST, INITROLL, UPCONTRL, GLIM, PREDICT3)
- `Comanche055/ENTRY_LEXICON.agc` — entry-specific state variables

---

## 1. Goal and current state

Today `programs/p61_p67.rs` is a phase-state-machine and DSKY-wiring skeleton, as its own header (`programs/p61_p67.rs:3`) and `specs/p61_p67-spec.md:24` both state. The closed-loop math, the SERVICER → entry data path, and the P65 / P66 programs are absent. `guidance/entry.rs` is a 1-line stub.

After this plan lands, the CM can fly closed-loop entry from 0.05g to drogue deploy for both **direct LEO entry** and **lunar-return entry with skip-out**, hitting the target landing point within Apollo-era footprint tolerances in `agc-sim`, with VirtualAGC-captured fixtures validating the inner loops.

## 2. Reference algorithm

The Apollo CM entry guidance is the linearized-feedback scheme from `Comanche055/REENTRY_CONTROL.agc` (documented in O'Brien Ch. 14, Morth NASA TM X-738, and `input/Users Guide to Apollo GNCS.pdf`). Five sub-phases:

| Sub-phase | AGC label | Program | Job |
|---|---|---|---|
| Pre-0.05g coast | INITROLL | P63 | Wait for sensed-g threshold, hold trim attitude |
| Post-0.05g initial roll | HUNTEST | P64 | Compute first roll command, branch to up/final |
| Up-control (skip) | UPCONTRL | P65 | Lift-up to extend range; manage exit conditions |
| Ballistic | BALLISTIC | P66 | Open-loop attitude hold when guidance diverges |
| Final phase | PREDICT3 / GLIM | P67 | Terminal range control using R-dot reference profile, drogue deploy |

Algorithm in plain terms: predict range-to-go assuming a reference profile, compare against actual range-to-target, fold the error and altitude-rate error into a commanded vertical-L/D, then resolve that to a roll angle (sign of bank from cross-range error). Phase transitions are driven by velocity bands and divergence checks.

## 3. Module layout

The math goes in `guidance/entry.rs`. `programs/p61_p67.rs` stays thin and orchestration-only — it sequences phases, drives the DSKY, and dispatches into `guidance::entry` each SERVICER cycle.

```
guidance/entry.rs                     (new content, ~700–900 LOC)
  pub struct EntryRef { … }           // reference-profile table
  pub struct EntryTargets             // target lat/lon, downrange/crossrange
  pub fn navigate_entry(state, dt)    // ECEF range-to-go, R-dot, downrange/crossrange
  pub fn predict_range(state)         // analytic range prediction from current state
  pub fn compute_ld_command(state)    // (L/D)_cmd from range error + R-dot error
  pub fn resolve_roll(state, ld_cmd)  // roll magnitude + cross-range sign flip
  pub fn select_phase(state)          // P64 → P65 / P66 / P67 transition logic
  pub fn upcontrol_step(state)        // P65 skip-out law
  pub fn ballistic_step(state)        // P66 — zero roll rate to DAP
  pub fn final_phase_step(state)      // P67 terminal R-dot vs V tracking
guidance/entry/tables.rs              (reference profile + L/D constants from REENTRY_CONTROL.agc)
programs/p61_p67.rs                   (gates state, calls into guidance::entry from servicer_exit)
```

## 4. Infrastructure prerequisites

These are blockers, in priority order:

1. **Sensed-acceleration path SERVICER → `entry.sensed_acceleration_g`.** Currently test-harness-driven (`specs/p61_p67-spec.md:45`). Wire it from `services/average_g.rs` via the `servicer_exit` hook the AgcState already exposes (`agc-core/src/lib.rs` — `servicer_exit: Option<fn(&mut AgcState)>`). The hook computes |a_sensed| / g0 and stores it before the entry programs read it.
2. **MET → GMST conversion in `navigation/time.rs:16`** (currently `todo!()`). Needed to express the target landing site in inertial coordinates and to project altitude / latitude / longitude from the inertial state vector.
3. **ECEF ↔ inertial transforms.** Short addition to `navigation/state_vector.rs` once GMST is available. Used every entry guidance cycle.
4. **Atmospheric density model.** Tabulated exponential atmosphere (`rho = rho_0 * exp(-h/H_s)`). New file `navigation/atmosphere.rs` (~80 LOC), no deps. Constants from REENTRY_CONTROL.agc.
5. **DAP roll-command path.** Confirm `control/dap.rs` accepts a commanded bank angle in an entry-guidance mode; if not, add a `DapMode::EntryRoll(f64)` variant and a small step function. Verify before adding — the entry-mode hook may already exist.

## 5. Implementation milestones

Each milestone is independently testable. Each maps 1:1 to a GitHub issue (see §8).

### MS-E1 — Plumbing (no entry math)
- SERVICER → `entry.sensed_acceleration_g` (Average-G magnitude / g0).
- Atmospheric density model + ECEF ↔ inertial transforms + GMST.
- DAP roll-command path verified or added.
- **Exit criterion**: existing P61–P67 phase-machine tests still pass; new unit test confirms `sensed_acceleration_g` advances under simulated PIPA inputs in `agc-sim`.

### MS-E2 — P63 (Pre-0.05g monitor)
- Replace the test-only `p63_check_threshold` with a SERVICER-driven check installed via `entry.servicer_exit`.
- Compute and display altitude rate (R-dot) and range-to-go on V16N64.
- Hold trim-attitude roll command (no closed loop yet).
- **Exit criterion**: 0.05g threshold trip on simulated trajectory advances to `Entry` phase; range-to-go display updates each 2-s cycle.

### MS-E3 — P64 (Post-0.05g, HUNTEST / INITROLL)
- `predict_range`, `compute_ld_command`, `resolve_roll`.
- Phase-selector: → P65 (skip), → P67 (final), → P66 (diverged).
- Reference-profile table from REENTRY_CONTROL.agc (AGC fixed-point → f64 conversion documented per constant).
- **Exit criterion**: roll command produced every 2-s cycle; phase selector chooses P65/P67 correctly for steep vs. shallow entries; VirtualAGC fixture match within tolerance for HUNTEST output.

### MS-E4 — P65 (Up-control / UPCONTRL)
- Skip-out guidance: lift-up to extend range, monitor exit velocity and re-entry conditions.
- Hand-off back to P64 (or P66) once exit conditions satisfied.
- **Exit criterion**: lunar-return entry with V ~36000 ft/s skips correctly and returns to closed-loop guidance; VirtualAGC fixture match for UPCONTRL key intermediates.

### MS-E5 — P66 (Ballistic)
- Smallest one. Zero-roll-rate hold, no closed loop.
- Add divergence-detection triggers in P64/P65 that transition to P66.
- **Exit criterion**: forced divergence test selects P66 and freezes roll command.

### MS-E6 — P67 (Final phase, PREDICT3 / GLIM)
- Terminal range control using tabulated R-dot vs. velocity profile.
- Drogue-deploy trigger on velocity / altitude condition (replaces current stub).
- **Exit criterion**: closed-loop sim from 0.05g to 24 km lands within ~25 nmi of target with nominal initial conditions.

### MS-E7 — Integration and validation
- End-to-end scenario in `agc-test` from P61 through P67 drogue.
- Two reference trajectories: direct LEO entry (no skip) and lunar-return entry (with P65 skip).
- Sweep of entry-flight-path-angle and azimuth to map the footprint.
- VirtualAGC end-to-end channel-trace comparison for both trajectories.

## 6. Test strategy

### 6.1 Unit tests (`guidance/entry.rs::tests`)
Per-function golden tests for range prediction, L/D-to-roll conversion, phase-transition logic, atmosphere model, GMST/ECEF transforms. Expected values from O'Brien Ch. 14 closed-form expressions and REENTRY_CONTROL.agc constants.

### 6.2 VirtualAGC fixture capture (Level 1)
For each closed-loop function (HUNTEST, UPCONTRL, GLIM, PREDICT3):
- Drive yaAGC into the relevant entry phase via DSKY scripting.
- Inject a canned state (position, velocity, sensed-acceleration history) over the channel protocol.
- Dump erasable memory after the routine runs (`yaAGC --debug=erasable`).
- Extract inputs and outputs, document AGC fixed-point scale factors, commit as JSON in `agc-test/fixtures/entry/`.
- Rust unit tests load the fixture and assert match within tolerance (typical: 1e-4 on roll command, 1e-3 on range-to-go).

New fixture files (planned):
```
agc-test/fixtures/entry/
  huntest_cases.json        // P64 initial roll, ~6–8 cases across entry-angle sweep
  upcontrol_cases.json      // P65 skip-out, ~4 cases (lunar-return entries only)
  glim_cases.json           // P67 final-phase R-dot tracking, ~6 cases
  predict_range_cases.json  // analytic range prediction, ~10 cases
```

Fixture-capture tooling: extend `agc-test/src/oracle/fixture_capture.rs` with entry-specific scenarios. Capture is a one-shot manual run; tests then run without Podman.

### 6.3 Integration tests (Level 2/3)
End-to-end entry scenarios in `agc-test/tests/entry_e2e.rs`:
- `entry_direct_leo`: P61 → P67 from 200 km circular orbit, no skip, target Pacific splashdown ellipse.
- `entry_lunar_return`: P61 → P67 from translunar-return state vector, P65 skip required, Pacific splashdown.
- Both gated by `VAGC_AVAILABLE=1` for the channel-trace comparison; the Rust-only legs (phase progression, miss distance) run in CI.

### 6.4 Existing tests must still pass
The current `programs/p61_p67.rs::tests` module exercises the phase machine only and must keep passing without modification. New tests for the math live in `guidance/entry.rs::tests`; new end-to-end tests live in `agc-test/tests/`.

## 7. Risks and open decisions

- **Reference profile data.** REENTRY_CONTROL.agc has the tables but in AGC fixed-point. Converting carefully is half the work of MS-E3. Cross-check against `input/Users Guide to Apollo GNCS.pdf`.
- **VirtualAGC scenario scripting.** We have not yet driven yaAGC through P64 in this project. Some up-front time to validate that the channel protocol + erasable dump can reach the entry routines reproducibly — call this out at MS-E1 review.
- **DAP coupling.** Entry guidance produces a roll *angle* command but the CM RCS DAP is typically rate-commanded with attitude-error feedback. Confirm the existing DAP supports a roll-hold/maneuver mode or extend it. Small risk, but verify before MS-E1 closes.
- **`tasks.md` deprecation.** Per project decision, ongoing work tracking moves to GitHub issues. This plan provides the seed issues (§8); `transformation/tasks.md` is frozen as a historical record.

## 8. GitHub issue seed

The milestones map to issues in the `senacor2/AGC-in-Rust` repo. Suggested label scheme:

- New label `entry-guidance` (color `#5319e7`) — scopes all issues in this plan.
- New label `milestone` (color `#0e8a16`) — for MS-E1…E7 tracking issues.
- New label `infrastructure` (color `#bfdadc`) — for prerequisite items inside MS-E1.
- Existing label `enhancement` — also applied to each milestone issue.

Proposed issues (one per milestone, plus a parent tracking issue):

| # | Title | Labels |
|---|---|---|
| Parent | Entry Guidance (P63–P67) — implementation tracking | `entry-guidance`, `milestone` |
| 1 | MS-E1: Entry-guidance plumbing (SERVICER, GMST, atmosphere, DAP) | `entry-guidance`, `milestone`, `infrastructure`, `enhancement` |
| 2 | MS-E2: P63 SERVICER-driven 0.05g monitor and R-dot display | `entry-guidance`, `milestone`, `enhancement` |
| 3 | MS-E3: P64 HUNTEST / INITROLL closed-loop guidance with VirtualAGC fixtures | `entry-guidance`, `milestone`, `enhancement` |
| 4 | MS-E4: P65 UPCONTRL skip-out guidance | `entry-guidance`, `milestone`, `enhancement` |
| 5 | MS-E5: P66 ballistic-phase hold and divergence triggers | `entry-guidance`, `milestone`, `enhancement` |
| 6 | MS-E6: P67 PREDICT3 / GLIM final-phase and drogue deploy | `entry-guidance`, `milestone`, `enhancement` |
| 7 | MS-E7: End-to-end entry scenarios (direct LEO + lunar return) | `entry-guidance`, `milestone`, `enhancement` |

Each milestone issue links back to the parent and to this plan, lists its exit criterion verbatim, and gets closed only when its VirtualAGC fixtures (where applicable) are committed and tests pass.
