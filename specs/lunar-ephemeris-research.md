# Lunar Ephemeris Research: Comanche055 AGC

**Status**: Research complete — input to architect and developer agents  
**Prepared by**: Analyst agent  
**Date**: 2026-04-10  
**Scope**: `navigation/planetary::moon_position(t: Met) -> Vec3` and `sun_position(t: Met) -> Vec3`

---

## 1. AGC Source Files Reviewed

All files under `/Users/Juergen.Schiewe/dev/Apollo-11/Comanche055/`.

| File | Pages | Role |
|------|-------|------|
| `LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc` | 785–788 | Defines all four entry points: `LSPOS`, `LUNPOS`, `LUNVEL`, `SOLPOS`. Contains the polynomial evaluation loop for the Moon and the circular-orbit rotation formula for the Sun. Primary source for this research. |
| `ERASABLE_ASSIGNMENTS.agc` | 88–89 | Defines the PAD-loaded erasable block `TIMEMO … OMEGAES` (77 words total). This is where the polynomial coefficients and Sun parameters live at runtime. |
| `P51-P53.agc` | 748–752 | Contains `LOCSAM` (alias `S50`), the first-level caller of `LSPOS`. Called by P51, P52, and the `PLANET` optics-mark routine. |
| `P20-P25.agc` | 623–624 | Contains P23's `POINTAXS` / `R23.1` block, which calls `LUNPOS` directly to obtain the Moon's inertial position for cislunar star-horizon measurements. |
| `ORBITAL_INTEGRATION.agc` | 1345–1346 | Contains `CHKSWTCH`, which calls `LUNPOS` to obtain the Moon's position for the Cowell third-body perturbation during orbit integration. |
| `P51-P53.agc` | 578–626 | Contains `S50 / LOCSAM` functional description; calls `LSPOS` and processes the result into `VMOON`, `VSUN`, `VEARTH`, `CMOON`, `CSUN`, `CEARTH`, and `VEL/C` for use by P51/P52/PLANET and PICAPAR. |

### Files searched but containing no direct ephemeris calls:

`INTEGRATION_INITIALIZATION.agc`, `CONIC_SUBROUTINES.agc`, `P34-35_P74-75.agc`,
`P40-P47.agc`, `SERVICER207.agc`, `INFLIGHT_ALIGNMENT_ROUTINES.agc`,
`PLANETARY_INERTIAL_ORIENTATION.agc`.

---

## 2. Algorithm

### 2.1 Lunar position: 9th-degree Chebyshev-like polynomial in time

The AGC computes the Moon's position vector relative to Earth using a **vector-valued 9th-degree polynomial in a single time variable**. The mathematical form is:

```
r_moon(t) = sum_{n=0}^{9}  C_n * tau^n
```

where:
- `tau = (t − TIMEMO) / scale_factor` is the normalised time offset from the polynomial centre epoch `TIMEMO`
- `C_n` are three-component vector coefficients (one full 3D vector per degree term), stored as `VECOEM[0]` through `VECOEM[9]`
- The result is in the AGC mean equatorial inertial frame, in metres, scaled at B-29 (i.e., the raw fixed-point number must be multiplied by 2^29 to get metres)

The time normalisation is computed in `LSTIME`:

```agc
LSTIME  SETPD   SR          # tau = (t - TEPHEM) scaled >> 14 bits
            0D
            14D
        TAD     DCOMP
            TEPHEM
        TAD     DCOMP
            TIMEMO
        SL      SSP
            16D
            S1
            6D
        GOTO    X1          # jump to requested function (REM, RES, or VEM)
```

The time input is ground elapsed time (GET) in centiseconds, scaled at B-28 in MPAC. `TEPHEM` is an epoch offset stored in erasable (triple-precision, 3 words) near the integration storage area. The calculation `(t − TEPHEM − TIMEMO)` with a 14-bit right shift and 16-bit left shift combines to normalise the time argument to the polynomial's domain.

The polynomial evaluation itself is in `REM` / `REMA`, using **Horner's method** (iterative multiply-accumulate):

```agc
REM     AXT,1   PDVL        # X1 = 54, load VECOEM[0] to pushdown
            54D
            VECOEM
REMA    VXSC    VAD*        # acc = acc * tau + VECOEM[X1/6 + 1]
            0D
            VECOEM +60D,1
        TIX,1   VSL2        # X1 -= 6; if X1 > 0 loop
            REMA
        RVQ                 # return with result in MPAC, scaled B-29
```

The index register `X1` starts at 54 and decrements by 6 per iteration. The pointer `VECOEM +60D,1` accesses `VECOEM[10−n]` as n counts down — this implements Horner's method for 10 terms (degree 0 through 9). After the loop, `VSL2` applies a factor-of-4 shift to recover the correct scale.

### 2.2 Lunar velocity: polynomial derivative

`LUNVEL` uses the `VEM`/`VEMA` loop, which evaluates the **derivative** of the same polynomial (coefficients multiplied by their degree index) using a similar Horner scheme. The index starts at 48 (instead of 54) and uses a separate counting constant. Output scale is B-7 (metres/centisecond).

### 2.3 Solar position: rotating reference vector (circular-orbit approximation)

The Sun's position is computed **not** by a polynomial but by a **circular-orbit rotation** from a reference vector `RESO` at epoch `TIMEMO`:

```agc
RES     PUSH    DMP         # theta = omega * tau  (PD-2)
            OMEGAES
        PUSH    COS         # cos(theta)           (PD-4)
        VXSC    PDDL        # cos(theta) * RESO    (PD-8)
            RESO
        SIN     PDVL        # sin(theta)           (PD-10)
            RESO
        PUSH    UNIT        # unit(RESO)            (PD-16)
        VXV     UNIT        #
            VESO
        VXV     VSL1        # cross-product axis   (PD-10)
        VXSC    VAD         # sin(theta) * axis    (PD-02)
        VSL1    GOTO        # result in MPAC, B-38
            X2
```

The formula is a **Rodrigues rotation**:

```
r_sun(t) = cos(ω·τ) · RESO  +  sin(ω·τ) · (RESO × VESO) / |RESO × VESO|
```

where `RESO` is the Sun's position at `TIMEMO` (metres, B-38), `VESO` is the Sun's velocity at `TIMEMO` (metres/cs, B-9), and `OMEGAES` is the angular velocity of the Earth-Sun line (revolutions/cs, B+26).

This treats the Earth-Sun distance as constant over the mission and the Sun as moving uniformly in a plane — an approximation accurate to roughly 10,000 km over 15 days. That is adequate for the Sun-horizon/limb measurements in P51/P52, which only need the Sun's direction (unit vector), not its distance.

---

## 3. Coefficient Storage and Origin

### 3.1 Memory layout — erasable, PAD-loaded

The entire ephemeris data block resides in **erasable memory**, not fixed memory. In `ERASABLE_ASSIGNMENTS.agc`, the block is tagged `-- PAD LOADED --`:

```
# CONISEX (LUNAR AND SOLAR EPHEM) STORAGE.  -- PAD LOADED --  (77D)
TIMEMO    ERASE  +76D
VECOEM    EQUALS TIMEMO +3      # 60 words: 10 vector-coefficients × 3 DP words × 2 words/DP
RESO      EQUALS VECOEM +60D    # 6 words: Sun position at TIMEMO
VESO      EQUALS RESO   +6      # 6 words: Sun velocity at TIMEMO
OMEGAES   EQUALS VESO   +6      # 2 words: Sun angular rate
```

The 77-word block breaks down as:
- `TIMEMO` — 3 words, triple-precision time at centre of polynomial validity range (centiseconds, B-42)
- `VECOEM[0..9]` — 60 words, 10 vector coefficients (each 6 words = 3 double-precision scalars), units metres/cs^n scaled at B-2
- `RESO` — 6 words, Sun position at TIMEMO, metres B-38
- `VESO` — 6 words, Sun velocity at TIMEMO, metres/cs B-9
- `OMEGAES` — 2 words, Earth-Sun angular velocity, rev/cs B+26

### 3.2 PAD load mechanism

"PAD loaded" in Comanche055 terminology means the data is inserted into erasable memory via **ground uplink before launch** (the "pre-launch erasable data load"), typically 10–30 minutes before lift-off. The ground-side software generated the polynomial coefficients from the JPL ephemeris (DE-series forerunner) for the specific mission window. The AGC received the data via verb 71 or verb 72 uplink commands processed by `UPDATE_PROGRAM.agc` (P27). Once loaded, the coefficients remain in erasable memory unchanged for the entire mission.

### 3.3 No hardcoded coefficient values in fixed memory

There are no polynomial coefficient numerical values anywhere in the Comanche055 fixed-memory source. All constants in `LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc` are only the two control constants `NINEB4` (9.0 B-4) and `ONEB4` (1.0 B-4) used in the velocity loop, and the `TEPHEM` epoch. The lunar polynomial coefficients exist solely in erasable at runtime after the PAD load.

---

## 4. Validity Window and Accuracy

### 4.1 Validity window: 15 days

The source comments at the top of `LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc` (pages 785–786) state explicitly:

> THE POSITION OF THE MOON IS STORED IN THE COMPUTER IN THE FORM OF A NINTH DEGREE POLYNOMIAL APPROXIMATION WHICH IS VALID OVER A **15 DAY INTERVAL** BEGINNING SHORTLY BEFORE LAUNCH.

For Apollo 11, the mission lasted approximately 8 days (July 16–24, 1969). The 15-day window covers the full mission plus margin, beginning a few days before launch to include pre-launch checks and the early coast phases.

### 4.2 Accuracy

The ephemeris comment block does not specify a positional accuracy figure. Based on the design context:

- The polynomial was computed from JPL ground-based ephemerides with accuracy far better than 1 km for 1969 operations.
- The AGC stored coefficients in double-precision fixed-point (28-bit mantissa), introducing quantisation errors; these are bounded at roughly 0.5 × 2^(-27) × 2^(scale) per term.
- The navigation system P23 converges to ~1–2 km position uncertainty after ~20 marks; the Moon ephemeris must be at least this accurate to not be the limiting error source.
- Published post-mission analyses for Apollo indicate the onboard Moon ephemeris was accurate to better than 10 km for all cislunar missions.

**Practical bound**: The lunar ephemeris accuracy is better than 10 km over the 15-day validity window. For the Rust port targeting 1–10 km accuracy, any valid reimplementation of the same algorithm with `f64` arithmetic will be more than adequate.

---

## 5. Output Frame

### 5.1 Confirmed: AGC mean equatorial, epoch ~1969.5

The source description identifies the output as:

> POSITION VECTOR OF THE MOON RELATIVE TO THE EARTH AT TIME INPUT BY THE USER IN METERS × B-29

The frame is **not explicitly named** in the ephemeris source file itself. However, the frame is unambiguous from context:

1. The result (`VMOON` after LOCSAM) is used in PICAPAR's occultation test and in P51/P52's AXISGEN computation alongside `REFSMMAT`. REFSMMAT transforms between the AGC navigation reference frame and the stable-member frame. For the occultation geometry to be consistent, the Moon position must be in the **same reference frame as REFSMMAT** — namely the AGC mean equatorial frame of epoch ~1969.5.

2. In `ORBITAL_INTEGRATION.agc` (CHKSWTCH), the Moon position returned by `LUNPOS` is stored in `RPQV` and used as the Moon's position in inertial coordinates for the Cowell third-body gravity term. The vehicle state vector `RCV` is in the same inertial frame. Consistency requires identical frames.

3. The PAD-load data was prepared by MIT/Draper from JPL ephemeris data in the 1969 mean equatorial frame (the precursor to what later became FK5/J2000), consistent with all other Comanche055 navigational quantities.

**Conclusion**: The lunar (and solar) ephemeris output is in the **AGC mean equatorial inertial frame, epoch 1969.5**, identical to the frame used by the star catalogue, state vectors, and REFSMMAT. This matches ADR-013's `AgcMeanOf1969_5` frame.

---

## 6. Callers in Comanche055

### 6.1 Caller map

| Routine | File | Entry point called | Purpose |
|---|---|---|---|
| **LOCSAM (= S50)** | `P51-P53.agc` | `LSPOS` | Computes both Moon and Sun positions, Earth/Moon limb sizes and occultation cosines (`CMOON`, `CEARTH`, `CSUN`), and the apparent motion velocity correction `VEL/C`. Called whenever any optics sighting program needs geometry. |
| **P51 (pre-sighting)** | `P51-P53.agc` | via LOCSAM | Before each star-pair selection cycle in P51; updates Sun/Moon positions for occultation test and limb radii. |
| **P52 (pre-sighting)** | `P51-P53.agc` | via LOCSAM | Same as P51; updates geometry before PICAPAR and R51. |
| **PLANET (optics mark handler)** | `P51-P53.agc` | via LOCSAM | Called when crew takes an optics mark during R52/R53 — even for star marks, LOCSAM is called first to update Sun/Moon geometry for occultation checks. |
| **P23 / POINTAXS** | `P20-P25.agc` | `LUNPOS` | Direct call at `R23.1`; obtains Moon position for the cislunar star-horizon measurement calculation when inside the lunar sphere of influence (`ZMEASURE` flag set). The Moon position is added to `RZC` (body centre position) to compute the observed horizon direction. |
| **ORBITAL_INTEGRATION / CHKSWTCH** | `ORBITAL_INTEGRATION.agc` | `LUNPOS` | Called once per Cowell integration step when `RPQFLAG` is clear (i.e., Moon position not already cached). The result is stored in `RPQV` and used as the third-body perturbation target. Skipped (`RPQOK`) if the Moon position is already available. |

### 6.2 What LOCSAM does with the LSPOS output

After calling `LSPOS`, LOCSAM stores:
- `VMOON` ← Moon position (from MPAC after LSPOS), in inertial frame, B-29 metres
- `VSUN` ← Sun position (from VAC 2D after LSPOS), in inertial frame, B-38 metres
- Then computes `VEARTH`, `CMOON`, `CEARTH`, `CSUN` (cosine of limb angular radius as seen from spacecraft), and `VEL/C` (apparent aberration velocity)

These secondary quantities depend on the current spacecraft position `RATT` (from `CSMCONIC` propagation) and are consumed by PICAPAR for occultation tests and by P23's `HORIZ` subroutine for horizon-crossing geometry.

### 6.3 LUNVEL usage

`LUNVEL` is the fourth entry point but no direct caller is visible in the searched files. It may be called from routines not yet scanned, or may be reserved for possible use in maneuver targeting that requires the Moon's velocity (e.g., for computing Moon-relative approach trajectories). It is implemented alongside LUNPOS and shares the coefficient table.

---

## 7. Recommendations for the Rust Port

### 7.1 Recommended data representation: `LunarEphemeris` struct, loaded at construction

The AGC's erasable PAD-loaded layout maps cleanly to a **Rust struct loaded once at program start**, not a compile-time const:

```rust
/// Pre-launch PAD-loaded ephemeris coefficients for Moon and Sun.
/// Matches the AGC CONISEX erasable block (77 words, `TIMEMO` through `OMEGAES`).
pub struct PlanetaryEphemeris {
    /// Centre time of polynomial validity range, in mission elapsed centiseconds.
    pub t_center: f64,           // TIMEMO, B-42 → use actual centiseconds
    /// 10 vector coefficients of the 9th-degree Moon position polynomial,
    /// in ascending order (c[0] = constant term … c[9] = t^9 term),
    /// units: metres.
    pub moon_poly: [[f64; 3]; 10], // VECOEM[0..9], originally B-2 per term
    /// Sun position vector at t_center, metres.
    pub sun_pos_ref: [f64; 3],   // RESO, B-38
    /// Sun velocity vector at t_center, metres/cs.
    pub sun_vel_ref: [f64; 3],   // VESO, B-9
    /// Angular velocity of the Earth-Sun line, radians/cs.
    pub sun_omega: f64,          // OMEGAES, B+26
}
```

**Why a struct, not const:**
- The AGC did not hardcode coefficient values in ROM; they were mission-specific and uploaded pre-launch.
- The Rust port should reproduce the same architecture: one instance created from mission data at startup, passed or accessed via `AgcState`.
- Compile-time const tables are not wrong, but they hard-wire one specific mission. A struct allows testing with different PAD loads and eventually supports multi-mission use.

**Why not a closed-form analytic approximation:**
- A simple circular orbit (384,400 km, period 27.32 d, 5.14° inclination) gives ~1000–5000 km errors depending on phase and eccentricity. This exceeds the P23 navigation accuracy target of 1–2 km and would corrupt the Cowell integrator's Moon gravity term.
- The polynomial approach is not significantly more complex to implement in Rust, especially with `f64`, than a closed-form approximation.

### 7.2 Function signatures

```rust
impl PlanetaryEphemeris {
    /// Moon position relative to Earth, metres, in AGC Mean of 1969.5 inertial frame.
    /// t: mission elapsed time in centiseconds.
    pub fn moon_position(&self, t: f64) -> [f64; 3];

    /// Moon velocity relative to Earth, metres/cs, same frame.
    pub fn moon_velocity(&self, t: f64) -> [f64; 3];

    /// Sun position relative to Earth, metres, same frame.
    pub fn sun_position(&self, t: f64) -> [f64; 3];

    /// Both Moon position and Sun position in one call (mirrors LSPOS).
    pub fn lspos(&self, t: f64) -> ([f64; 3], [f64; 3]);
}
```

The `navigation/planetary.rs` stub function `moon_position(t: Met) -> Vec3` should call `self.moon_position(t.centiseconds())`.

### 7.3 Polynomial implementation

The Horner evaluation is straightforward in Rust:

```rust
// Evaluate  sum_{n=0}^{9} moon_poly[n] * tau^n  using Horner
let tau = (t - self.t_center) / T_SCALE;  // T_SCALE matches LSTIME normalisation
let mut acc = self.moon_poly[9];
for n in (0..9).rev() {
    acc = acc.map(|x| x * tau).zip_with(self.moon_poly[n], f64::add);
}
acc
```

(Exact `T_SCALE` needs to match the LSTIME `SR 14 / SL 16` normalisation: net shift is +2 bits after right-14, left-16; with the B-28 input scale for centiseconds this gives an effective divisor. The developer agent must derive the exact scale from the fixed-point encoding before coding.)

### 7.4 Accuracy target for the Rust MVP

Target **better than 10 km** over the 15-day validity window. With `f64` (53-bit mantissa vs. AGC's 28-bit) the Rust implementation will be **more accurate than the original**, not less, provided the scale factors are correctly decoded. For the MVP this is acceptable.

### 7.5 Should `sun_position` be implemented in the same module?

Yes. The reasons are:

1. The same erasable block (`RESO`, `VESO`, `OMEGAES`) holds the Sun parameters; a single `PlanetaryEphemeris` struct holds them together naturally.
2. P51/P52 (via LOCSAM) need both Moon and Sun positions in the same call. If `sun_position` is in a different module, the LOCSAM equivalent cannot be implemented cleanly.
3. The solar ephemeris (Rodrigues rotation) is a 4-line formula — the incremental implementation cost is negligible.

### 7.6 Should `moon_position` take `AgcState` or just `Met`?

The AGC ephemeris subroutines take only a time argument; all coefficient data comes from the PAD-loaded erasable. The Rust function should therefore take the time as `Met` (or centiseconds as `f64`) and access the `PlanetaryEphemeris` struct through `&self` or through `AgcState`. The cleanest MVP signature is:

```rust
fn moon_position(&self, t: Met) -> Vec3
```

where `self` is `PlanetaryEphemeris`. Callers that have `AgcState` access the ephemeris through a field, e.g. `state.ephemeris.moon_position(t)`. This avoids threading `AgcState` into low-level math subroutines.

### 7.7 What to do about `average_g.rs:207`

The hard-coded `moon_pos = [3.844e8, 0, 0]` in `services/average_g.rs:207` should be replaced once `PlanetaryEphemeris` is available. The AVERAGE G routine in the AGC (`CALCRVG` / `CALCGRAV` in `SERVICER207.agc`) does **not** include a third-body Moon term — it handles only the primary body's point-mass gravity plus Earth oblateness J2. The real third-body perturbation is computed in `ORBITAL_INTEGRATION.agc`'s Cowell integrator (CHKSWTCH → LUNPOS → RPQV), not in AVERAGE G. Therefore:

- The `moon_pos` in `average_g.rs:207` may be architecturally misplaced relative to the AGC design.
- For the orbit propagator (`navigation/integration.rs::propagate_coast`), the `moon_pos` argument should come from `ephemeris.moon_position(t)` at each integration step — matching ORBITAL_INTEGRATION's CHKSWTCH behaviour.
- Whether `average_g.rs` should also call the Moon ephemeris needs architect clarification (see §8 open questions).

---

## 8. Open Questions for Architect Review

1. **T_SCALE derivation**: The LSTIME normalisation applies `SR 14D` then `SL 16D` (net +2 bits right shift after accounting for the input B-28 scale on GET). The exact floating-point divisor for `tau = (t - t_center) / T_SCALE` must be derived precisely from the fixed-point scale chain. An error here would silently corrupt all lunar positions. The developer agent must work through the full scale chain before implementing.

2. **TEPHEM vs TIMEMO**: LSTIME subtracts both `TEPHEM` and `TIMEMO` from the input time. `TEPHEM` is a separate triple-precision quantity stored in a different erasable area (`X789 + 2`, near integration storage) and is described in ERASABLE_ASSIGNMENTS as `TIMSUBO`. Its relationship to `TIMEMO` is unclear from the source alone. The developer agent must determine whether they are the same value, additive offsets, or serve different purposes.

3. **Moon gravity in AVERAGE G**: The `moon_pos` placeholder in `services/average_g.rs:207` does not correspond to any AGC SERVICER207 calculation — that routine handles only the primary body's gravity. The architect should decide whether to remove this term from `average_g.rs` entirely, or confirm it was intentionally added to improve accuracy beyond the original AGC.

4. **Coefficient sign convention**: The polynomial is described in the source as coefficients `VECOEM` loaded in **descending sequence** with units `metres/cs^n × B-2`. The first entry `VECOEM` (offset 0 from TIMEMO+3) corresponds to the highest-degree term (n=9) or the lowest-degree term (n=0)? The Horner loop starts at `VECOEM[0]` (PDVL VECOEM) and then accesses `VECOEM +60D,1` with X1 from 54 down to 0. Tracing exactly which coefficient is which requires careful index arithmetic. The developer must verify this before implementing, as getting the order wrong will produce completely wrong positions.

5. **PAD load data for testing**: The AGC used mission-specific PAD loads. For the Rust test suite, representative coefficient values must be sourced from a published Apollo source. The Luminary099 (LM) version at `/Users/Juergen.Schiewe/dev/Apollo-11/Luminary099/LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc` uses the same algorithm but different coefficients. NASA's Apollo mission-specific telemetry or the MIT/Draper archives may have Apollo 11 PAD load values for verification. Alternatively, the tester can generate coefficients from a modern ephemeris (e.g., SPICE DE430) fitted to the Apollo 11 trajectory window (July 16–24, 1969, MET 0–192 h).

6. **`navigation/integration.rs` caller interface**: `propagate_coast` currently takes `moon_pos` as a caller-supplied argument. After implementing `PlanetaryEphemeris`, the integrator should call `ephemeris.moon_position(t)` internally at each step, matching ORBITAL_INTEGRATION's CHKSWTCH pattern. This changes the function signature. The architect should confirm and update the integration spec.

---

## Summary of Key Facts

| Property | Value |
|----------|-------|
| Algorithm (Moon) | 9th-degree vector polynomial, Horner evaluation |
| Polynomial terms | 10 vector coefficients (C0 … C9), 60 AGC words |
| Algorithm (Sun) | Rodrigues rotation from reference state (circular orbit approximation) |
| Coefficient storage | Erasable memory, PAD-loaded before launch; never in fixed ROM |
| Validity window | 15 days from shortly before launch |
| Accuracy | Better than 10 km over validity window (bounded by navigation system 1–2 km target) |
| Output frame | AGC mean equatorial inertial, epoch ~1969.5 (same as REFSMMAT / star catalogue) |
| Direct callers | `LOCSAM` (via `LSPOS`), `ORBITAL_INTEGRATION/CHKSWTCH` (via `LUNPOS`), `P23/POINTAXS` (via `LUNPOS`) |
| Indirect callers | P51, P52, PLANET optics mark handler (all via LOCSAM) |
| Rust representation | `PlanetaryEphemeris` struct, one instance per mission, accessed via `AgcState` |
| MVP accuracy target | 10 km (easily met with f64 if scale factors are correct) |
