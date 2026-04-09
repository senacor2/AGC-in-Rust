# AGC Validation Skill

Validates Rust implementations against the original AGC Comanche055 assembler
source and reference constants.

## When To Use

- After implementing a new module or milestone
- After modifying constants, algorithms, or integration formulas
- When the user asks to "validate", "verify", or "check against assembler/source"
- Before marking a milestone as complete

## Reference Data (local — no web fetches needed)

| Resource | Path | Purpose |
|---|---|---|
| AGC assembler source | `docs/agc-source/*.agc` | Comanche055 .agc files (downloaded from Apollo-11 GitHub) |
| Reference constants | `docs/agc-reference-constants.md` | Pre-extracted constants, algorithms, key codes, and book page references |
| Source file map | `docs/agc-reference-constants.md` § "Source File Map" | Maps each Rust module to its AGC source file(s) |
| Architecture doc | `docs/architecture.md` | Architecture decisions and constraints |

## Procedure

### 1. Identify scope

Determine which Rust modules to validate. If the user specifies a module, validate
that one. If the user says "validate all" or "validate milestone N", validate all
modules in the milestone.

### 2. Read the reference constants

Read `docs/agc-reference-constants.md` to get the expected values for all constants,
algorithm structures, and key codes relevant to the modules being validated.

### 3. Read the Rust source

For each module being validated, read the Rust source file(s) from `agc-core/src/`.

### 4. Read the AGC assembler source

For each module, read the corresponding `.agc` file from `docs/agc-source/` using
the Source File Map in the reference constants doc. Focus on:
- Named constants (look for `DEC`, `2DEC`, `EQUALS`, `ERASE` directives)
- Algorithm structure (look for subroutine labels, `TC`, `CAF`, `VAD`, `VXSC`)
- Comments that describe expected behavior or cite specific values

### 5. Compare and report

For each item, produce one of:

| Result | Meaning |
|---|---|
| **CONFIRMED** | Rust value/algorithm matches AGC source exactly |
| **WRONG** | Rust value/algorithm differs from AGC source — include both values |
| **APPROXIMATE** | Rust uses a valid numerical substitute (e.g., RK4 for Nystrom) — note the difference |
| **NOT FOUND** | Expected constant/algorithm not found in AGC source — may need a different .agc file |
| **UNVERIFIED** | Cannot confirm from available local sources — note what's missing |

### 6. Output format

```markdown
## Validation Report: [module name]

AGC source: [filename.agc]
Rust source: [path/to/file.rs]

| # | Item | Expected (AGC) | Actual (Rust) | Result |
|---|------|----------------|---------------|--------|
| 1 | MU_EARTH | 3.986032e14 | 3.986032e14 | CONFIRMED |
| 2 | Position update | Störmer-Verlet | Midpoint rule | WRONG |
...

### Issues requiring correction
- [List any WRONG items with recommended fix]

### Items to add to reference constants
- [List any newly discovered constants/algorithms to add to agc-reference-constants.md]
```

### 7. Update reference constants

If the validation discovers new constants or algorithms not yet in
`docs/agc-reference-constants.md`, add them. This grows the reference doc
over time so future validations are cheaper.

## Validation Checklist by Category

### Constants
- [ ] All physical constants match AGC values (not modern IAU/WGS84)
- [ ] All scheduler limits match (MAX_JOBS, MAX_WAITLIST_TASKS, NUM_RESTART_GROUPS)
- [ ] All DSKY key codes match PINBALL octal table
- [ ] All alarm codes match ALARM_AND_ABORT.agc

### Algorithms
- [ ] Integration method matches AGC source (or is documented as a valid substitute)
- [ ] SERVICER uses predictor-corrector with GDT/2 carry-over (not midpoint)
- [ ] Gravity evaluation point matches AGC (predicted position, not old position)
- [ ] PIPA conversion uses KPIP1 scale factor correctly
- [ ] REFSMMAT rotation direction correct (sm_to_eci = transpose of eci_to_sm)

### Architecture
- [ ] No `unwrap()` / `expect()` in production paths (only in tests)
- [ ] No heap allocation in `agc-core` (must be `no_std`)
- [ ] Alarm severity classification matches AGC source
- [ ] Restart group odd/even phase semantics match AGC

## Cost Efficiency Notes

- **Never fetch from the web** — all reference data is local in `docs/agc-source/`
- **Read reference constants first** — most validations can be done by comparing
  Rust values against `docs/agc-reference-constants.md` without reading .agc files
- **Read .agc files only for algorithm structure** — constants are already extracted
- **Use `cargo test` for regression** — constant-assertion tests catch drift automatically
- **Typical token budget**: ~5k tokens for a single-module validation, ~15k for a full milestone
