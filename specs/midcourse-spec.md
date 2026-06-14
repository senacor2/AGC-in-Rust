# Specification: `guidance/midcourse` Module — Midcourse Correction (Placeholder)

**Status**: Placeholder — see §3 for current implementation pointers
**Module path**: `agc-core/src/guidance/midcourse.rs`
**Related specs**: `specs/p23-spec.md` (cislunar navigation), `specs/p30-spec.md` (external-ΔV targeting), `specs/targeting-spec.md` (`apply_external_delta_v`), `specs/p40_p41-spec.md` (burn execution).
**Glossary cross-reference**: `docs/glossary.md` — MCC, External ΔV, FIDO.

---

## 1. Purpose and Status

`guidance/midcourse.rs` is a **placeholder module**. As of this audit
the file contains only a header doc-comment:

```rust
//! Midcourse correction guidance (P23 cislunar navigation support).
```

No types, functions, or constants are defined in the file. It is
re-exported from `guidance/mod.rs` as `pub mod midcourse;` so that
future midcourse-specific helpers have a natural home and the path
`crate::guidance::midcourse::` can be used in cross-references and
specs without breaking once code lands.

This spec exists to **document that intent**, to inventory where the
project currently handles MCC functionality (so a reader can find it
without grep), and to record the scope the module should grow into.

---

## 2. Mission context: what an MCC needs

An Apollo midcourse correction (MCC) — MCC-1 .. MCC-7 in the standard
numbering — is a small, ground-targeted trim burn during translunar
or transearth coast. The flight-dynamics flow is:

1. **Navigation refinement** (P23 cislunar marks of stars against the
   Earth or Moon limb) tightens the onboard state vector against
   accumulated coast-phase drift.
2. **Ground targeting** (FIDO + RETRO) computes a target maneuver — a
   TIG and an LVLH ΔV vector — and uploads it via P27 / V70..V73.
3. **Crew loads the pad** (P30: V25 N33 = TIG, V25 N81 = LVLH ΔV).
4. **Crew executes the burn** (P40 for SPS, P41 for RCS — MCCs of less
   than ~0.5 m/s are nominally RCS-only).

There is no single "MCC routine" in the historical AGC source either —
the work is spread across P23, P30, P40/P41, and the
`apply_external_delta_v` / `IMPULSIVE` family of conversion helpers.

---

## 3. Where MCC functionality lives today

The current implementation distributes the responsibilities as follows:

| Responsibility | Module | Notes |
|---|---|---|
| Cislunar navigation refinement (the input to an MCC decision) | `agc-core/src/programs/p23.rs` | `specs/p23-spec.md` |
| Ground-uplinked LVLH-ΔV → inertial-Δv conversion | `agc-core/src/guidance/targeting.rs::apply_external_delta_v` and friends | `specs/targeting-spec.md` |
| Crew pad load (TIG + LVLH ΔV) | `agc-core/src/programs/p30.rs` | `specs/p30-spec.md` |
| Burn execution and cutoff | `agc-core/src/programs/p40_p41.rs` (and `guidance/maneuver.rs`) | `specs/p40_p41-spec.md`, `specs/maneuver-spec.md` |

Nothing **mission-critical** for an Apollo-8-style MCC flow needs a
dedicated `midcourse.rs` body today — the four pieces above compose
into a complete MCC capability. The `agc-test/tests/full_mission.rs`
walkthrough exercises this composition end-to-end (MCC-2 translunar
and MCC-4 transearth correction, executed via P30 → P40).

---

## 4. Scope reserved for the module

Future work that would naturally land in `guidance/midcourse.rs`:

- A **trim-burn solver** that takes the current cislunar state vector
  and a target arrival state and returns a Lambert-derived ΔV bias
  for use as an autonomous MCC (no ground uplink needed). The current
  flow assumes ground targeting; an autonomous solver would let the
  AGC self-correct during a long coast.
- A **threshold gate** that decides whether to recommend a midcourse
  based on accumulated post-burn dispersion (e.g. position-error
  growth predicted forward to entry interface). Today this decision
  is FIDO's.
- **Finite-burn correction** specific to small trim burns (the present
  `cross_product_steering` in `guidance/maneuver.rs` is tuned for the
  large SPS burns).

None of the above are scheduled. Issue #160 created this spec as a
placeholder; concrete behaviour will be added when the project takes
on autonomous-MCC support.

---

## 5. Rust API

Currently empty. The module declaration `pub mod midcourse;` in
`agc-core/src/guidance/mod.rs` keeps the path stable so that
cross-references in other specs and in code comments do not break when
code is added.

---

## 6. Test Cases

None — the module has no functional surface to test. The integration
flow it would support is exercised by `agc-test/tests/full_mission.rs`
through the composition described in §3.

---

## 7. Spec Quality Checklist

- [x] Placeholder status explicitly stated (§1).
- [x] Mission context explained so a reader knows what an MCC is (§2).
- [x] Inventory of where MCC functionality currently lives (§3).
- [x] Scope reserved for the module recorded for future work (§4).
- [x] No tests claimed (§6).
