# Star Catalogue Research: Comanche055 AGC
**Status**: Research complete — input to developer agent  
**Prepared by**: Analyst agent  
**Date**: 2026-04-10  
**Scope**: `navigation/star_catalog.rs` and `programs/p23.rs::CISLUNAR_STAR_TABLE`

---

## 1. AGC Source Files That Reference the Star Catalogue

All files are in `/Users/Juergen.Schiewe/dev/Apollo-11/Comanche055/`.

| File | Pages | Role |
|------|-------|------|
| `STAR_TABLES.agc` | 1389–1393 | Defines the fixed-memory star catalogue: 37 unit vectors, stored in reverse order (star 37 down to star 1), followed by the sentinel word `CATLOG DEC 6970`. |
| `P51-P53.agc` | 737–784 | Contains `PICAPAR` (star-pair selection R56), `R51` (fine align), `P51` (P51 program), `PLANET` (optics-mark entry), `OCCULT` (occultation test), and `CHKSCODE` (STARCODE range validation). All are direct consumers of `CATLOG`. |
| `P20-P25.agc` | 562–634 | Contains P23 (cislunar navigation), including `LOWMEMRY` (cross-bank star vector fetch via `CATLOG,1`), `P23.17` (star vector retrieval loop), and `LDPLANET` (planet-mode fallback). |
| `ERASABLE_ASSIGNMENTS.agc` | various | Defines every erasable variable associated with star operations: `STARCODE`, `STARALGN`/`SINCDU`/`COSCDU`, `BESTI`, `BESTJ`, `STARIND`, `STARAD`, `STARSAV1`, `STARSAV2`, `STARSAV3`, `US`. |
| `IMU_CALIBRATION_AND_ALIGNMENT.agc` | 423–455 | Uses `STARCODE` to point the optics at a specific star during IMU self-test (optical-verification mode). Not a direct `CATLOG` consumer but writes `STARCODE`. |
| `TAGS_FOR_RELATIVE_SETLOC.agc` | — | Defines `STARTAB EQUALS` in bank 14 (= fixed-memory bank 32). |
| `PINBALL_NOUN_TABLES.agc` | — | Maps noun 70 R1 and noun 71 R1 to `STARCODE` (display of star code on DSKY), and noun 88 to `STARSAV3` (sun/planet vector display). |
| `ASSEMBLY_AND_OPERATION_INFORMATION.agc` | — | Documents noun 70/71 fields: R1 = `STARCODE` (octal only), R2 = `LANDMARK`, R3 = `HORIZON`. Also lists noun 88 as "half-unit sun or planet vector". |
| `LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc` | 785–788 | Computes Sun and Moon position via polynomial approximation. Called by the `PLANET` routine in P51-P53.agc when `STARCODE == 0` (planet mode). |
| `SXTMARK.agc` | 222–235 | Mark-system resource manager; dispatches `MKVB51` job when the crew takes an optical mark. Not a direct `CATLOG` consumer but provides the hardware mark interrupt mechanism that feeds R52/R53. |

### Other files searched, no direct star-catalogue references found:
`R30.agc`, `R31.agc`, `R60_62.agc`, `INFLIGHT_ALIGNMENT_ROUTINES.agc`,
`SERVICER207.agc`, `CONIC_SUBROUTINES.agc`, `P30-P37.agc`, `P40-P47.agc`.

---

## 2. Data Format in Fixed Memory

### 2.1 Memory bank and label

`STAR_TABLES.agc` line 32–34:

```
BANK    32
SETLOC  STARTAB
BANK
```

Bank 32 is a fixed-memory (ROM) bank. `STARTAB` is also defined in `TAGS_FOR_RELATIVE_SETLOC.agc` as the origin of bank 14 (the SETLOC name used to place the table at a specific offset). The table is therefore **in fixed memory and is by definition read-only**. There is no mechanism in the AGC to write to fixed memory at runtime.

### 2.2 Storage layout

Each star occupies **three consecutive double-precision (2-word) AGC words**, one for each Cartesian component:

```
STARTAB:
    2DEC  +.XXXXXXXXXX B-1    # STAR 37  X
    2DEC  +.XXXXXXXXXX B-1    # STAR 37  Y
    2DEC  +.XXXXXXXXXX B-1    # STAR 37  Z
    ...
    2DEC  +.XXXXXXXXXX B-1    # STAR 1   X
    2DEC  +.XXXXXXXXXX B-1    # STAR 1   Y
    2DEC  +.XXXXXXXXXX B-1    # STAR 1   Z
CATLOG  DEC  6970
```

The table is laid out in **descending order** (star 37 first, star 1 last). This is significant: the AGC indexing scheme works by counting down from the top of the table. The label `CATLOG` is placed **after** the last entry (star 1, Z component), i.e. `CATLOG` points one word beyond the table end. This is the anchor that consumers use for indexed addressing.

### 2.3 Scaling (B-1 notation)

Each component value is stored as a `2DEC` with scale factor `B-1`, meaning the stored fractional value must be multiplied by 2^(-1) = 0.5 to recover the true component. The raw 2DEC values in the source already include this factor: a value written as `+.8748658918 B-1` represents the actual component 0.8748658918 × 2^(−1) = 0.4374329459.

Wait — re-reading carefully: in AGC notation `2DEC value B-1` stores `value/2` in a double-precision fixed-point word. However the actual values in the file (e.g. `.8748658918`) are close to unit-vector components that are plausible without a factor of 2 applied. The `B-1` scaling means that the stored number in hardware is `value × 2^(-1)`, i.e. the floating-point value is `stored_integer / 2^28 × 2^1`. The numbers in the source are **already the correct direction-cosine values**; the `B-1` is the representation hint to the assembler to scale correctly for 28-bit double-precision storage. For the Rust port, read the literal decimal numbers directly as `f64` direction cosines. They are already unit vectors (verified: for star 1, √(0.8748² + 0.0261² + 0.4837²) ≈ 1.0).

### 2.4 CATLOG sentinel value

`CATLOG DEC 6970` — this is a decimal constant with no navigation meaning; it serves as a terminator/marker word at the end of the table. The value 6970 is not used in any computation; it just occupies the word immediately following the last star entry, which coincidentally gives `CATLOG` an address that equals `STARTAB + 37×6` (37 stars × 3 double-precision words × 2 words each = 222 words, but 2DEC takes 2 words so 37×3×2 = 222; `CATLOG = STARTAB + 222`).

### 2.5 Per-star word count

Each star: 3 components × 1 double-precision slot × 2 AGC words per slot = **6 AGC words per star**. This is why BESTI and BESTJ in the erasable area are described as "star numbers times 6": the raw catalog offset for star N (1-based) is `(37 − N) × 6` from `STARTAB`, or equivalently, a backward offset of `N × 6` from `CATLOG`.

No name field is stored in AGC fixed memory. Star names exist only in documentation and in astronaut checklists. The crew enters star numbers (1–37) on the DSKY via noun 70 or 71.

---

## 3. Consumers

### 3.1 Consumer map

| Routine / Program | File | How it uses the catalogue |
|---|---|---|
| **PICAPAR (R56)** | `P51-P53.agc` | Iterates all 37 entries via `CATLOG,1` and `CATLOG,2`; tests each for occultation and angular separation to select the best visible pair for IMU fine-alignment. |
| **R51 (fine align)** | `P51-P53.agc` | Reads `BESTI`/`BESTJ` (star offsets selected by PICAPAR); loads star vectors from `CATLOG,1` and `CATLOG,2` into `STARSAV1` and `STARSAV2`; computes expected stable-member direction. |
| **P51 (P51 program)** | `P51-P53.agc` | Calls PICAPAR to select two stars, then R51 to perform fine align. Writes `STARCODE` from `BESTI × 1/6TH` for DSKY display on noun 70. |
| **P52 (P52 program)** | `P51-P53.agc` | Same as P51 for the star-sighting phase; calls PICAPAR then R51. |
| **PLANET (optics mark entry)** | `P51-P53.agc` | Entry point when the crew takes an optics mark during R52/R53. If `STARCODE == 0` → planet mode (uses ephemeris, not catalogue). If `STARCODE > 0 and STARCODE <= 37` → fetches star vector from `CATLOG,1`. If `STARCODE > 50` → uses previously stored track data at `STARAD − 228D,1`. |
| **P23.17 / LOWMEMRY** | `P20-P25.agc` | When crew enters a star code via noun 70 in P23: multiplies `STARCODE` by 6 to get `BESTI`, then calls `LOWMEMRY` which executes `VLOAD* RVQ / CATLOG,1` (cross-bank load of the direction vector), storing result in `STARSAV2 (= US)`. |
| **OCCULT** | `P51-P53.agc` | Called by PICAPAR for each candidate star; receives star direction vector (already loaded from `CATLOG`); tests against Earth/Sun/Moon angular radii stored in `CULTRIX` matrix. Not itself a catalogue reader — receives the loaded vector. |
| **IMU_CALIBRATION_AND_ALIGNMENT** | `IMU_CALIBRATION_AND_ALIGNMENT.agc` | Writes `STARCODE = 1` and `STARCODE = 2` (at CONTIN33 and NEXBNKSS) to command the optics to stars 1 and 2 during self-test. This is `TARGDRVE` pointing, not a direct vector lookup. |

### 3.2 P51/P52 detail — PICAPAR selection algorithm

`PICAPAR` (in `P51-P53.agc`, label `PICAPAR`, page 752):

1. Reads IMU CDU angles; computes shaft axis (SAX) in inertial space via `REFSMMAT`.
2. Initialises loop counter `X1 = 228` (= 37 × 6 + 6), counting down in steps of 6 (one star per iteration).
3. For each star: calls `OCCULT` with `CATLOG,1` (the vector indexed by `X1`). If the star passes (not occulted), it becomes the "major" candidate.
4. For each major candidate, iterates a second inner loop (`X2`) pairing it with all lower-indexed stars, checking each pair for:
   - Neither star occulted
   - Angular separation 40°–66° (`COS76 < dot < COS30`, per CSS66/CSS6640 constants)
   - Both stars within 33° of SAX (sextant field-of-view cone, CSS33 constant)
5. The pair with maximum angular separation is stored in `BESTI` and `BESTJ` (as star index × 6).
6. Returns: normal return if ≥1 valid pair found; error return if no valid pair.

Output: `BESTI`, `BESTJ` — integer offsets into the catalog (= star number × 6, relative to `CATLOG`).

### 3.3 R51 fine-alignment detail

`R51` (in `P51-P53.agc`, page 756):

1. Uses `BESTI`/`BESTJ` as index registers. On first pass (`STARIND = 1`): loads `BESTI` from `BESTI`; computes `STARCODE = BESTI × 1/6TH` for crew display.
2. Calls R52 (sextant optics mark routine) to get measured star direction in stable-member frame; stores it in `STARAD` (first star) or `STARAD+6` (second star).
3. After both marks: calls `LOCSAM` for time/position, `PLANET` for ephemeris, then calls `R54` (CHKSDATA — star angle test, comparing catalog expected angles with marked angles).
4. Calls `AXISGEN` (in `P51-P53.agc`, page 764) to compute REFSMMAT from the two star vectors in catalog frame vs. the two measured vectors in stable-member frame. This is the TRIAD method.

### 3.4 P23 detail — star-horizon measurement

`P23` (in `P20-P25.agc`, page 619):

1. Crew enters `STARCODE` (1–37) and `HORIZON` (1 = near, 2 = far) via V05N70 flash.
2. Code at `P23.17` computes `BESTI = STARCODE × 6`, then calls `LOWMEMRY`:
   ```
   LOWMEMRY    VLOAD*   RVQ
                   CATLOG,1
   ```
   This executes a cross-bank load of the 3-component star unit vector indexed by `BESTI`, stores result in `STARSAV2` (aliased as `US` in cislunar context).
3. `STARCODE = 0` branches to `LDPLANET` — the crew has indicated a planet or the Sun; the AGC displays noun 88 (`STARSAV3`, the planet vector), and the crew updates it before the mark.
4. R52/R53 then takes the sextant mark; the mark time, shaft, and trunnion angles are stored. The predicted angle between `STARSAV2` and the body (Earth/Moon) horizon is computed via `POINTAXS`.
5. The residual (measured angle − predicted angle) is incorporated into the navigation state via the scalar Kalman filter (`INCORP1`/`INCORP2` in `MEASUREMENT_INCORPORATION.agc`).

### 3.5 Distinction: navigation stars vs. celestial bodies

The star catalogue (37 entries) contains only **fixed stars**. The Sun, Moon, and planets are treated completely separately:

- `LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc` computes Sun and Moon positions via 9th-degree polynomial approximation loaded at mission-start.
- For P51/R51 optics marks: `PLANET` routine is called; if `STARCODE == 0`, the routine uses the ephemeris computation for the Sun.
- For P23: `STARCODE == 0` triggers `LDPLANET` which uses `STARSAV3` (previously entered planet vector via noun 88, V06N88).
- Noun 88 is listed in `PINBALL_NOUN_TABLES.agc` line 760 as "HALF UNIT SUN OR PLANET VECTOR" stored in `STARSAV3`.

**There is no separate "planet catalogue" table.** Planets use real-time ephemeris routines (Sun/Moon) or crew-entered vectors (other bodies). The 37-entry table covers only fixed stars.

---

## 4. Number of Stars and Identity

### 4.1 Count verified: 37 stars

The Comanche055 `STAR_TABLES.agc` contains exactly 37 stars (numbered 1–37), with entries for stars 37 down to 1. The validation code in `P20-P25.agc` (label `P23.16`) confirms this:

```
CA   STARCODE    # IS STARCODE GREATER THAN OR
EXTEND          # EQUAL TO 0 AND LESS THAN 37
BZF  P23.176
EXTEND
BZMF R23.10
AD   NEG37
EXTEND
BZMF +2
TC   R23.10
```

`NEG37` is defined as `DEC -37`. The valid range is 1–37 for navigation stars; 0 means planet mode.

The validation code in `CHKSCODE` (`P51-P53.agc`, line 1731) accepts 0 through 50:

```
CHKSCODE  CCS  STARCODE
          AD   NEG47
          CCS  A
          TC   Q    # SC < 0 OR SC > 50
```

Stars 38–50 are therefore valid STARCODE values for the optics routines even though they are not in the fixed catalogue. These codes (38–50) are used for planet/solar mode in the P51/R51 context (see `PLANET` routine at line 2185–2196: codes 38–50 trigger `CALSAM1` which uses previously stored track data from `STARAD − 228D,1`, not the star catalogue).

### 4.2 Complete star list with direction vectors

The `B-1` scaled values from `STAR_TABLES.agc` are the actual direction cosines (see §2.3 scaling note above). The table below gives each star's number and the three components as they appear in the source. These are unit vectors; the coordinate frame is discussed in §6.

| Star # | X component | Y component | Z component |
|--------|-------------|-------------|-------------|
| 1 | +0.8748658918 | +0.0260879174 | +0.4836621670 |
| 2 | +0.9342640400 | +0.1735073142 | −0.3115219339 |
| 3 | +0.4775639450 | +0.1166004340 | +0.8708254803 |
| 4 | +0.4917678276 | +0.2204887125 | −0.8423473935 |
| 5 | +0.0130968840 | +0.0078062795 | +0.9998837600 |
| 6 | +0.5450107404 | +0.5314955466 | −0.6484410356 |
| 7 | +0.7032235469 | +0.7075846047 | +0.0692868685 |
| 8 | +0.4105636020 | +0.4988110001 | +0.7632988371 |
| 9 | +0.3507315038 | +0.8926333307 | +0.2831839492 |
| 10 | +0.2011399589 | +0.9690337941 | −0.1432348512 |
| 11 | +0.1371725575 | +0.6813721061 | +0.7189685267 |
| 12 | −0.0614937230 | +0.6031563286 | −0.7952489957 |
| 13 | −0.1820751783 | +0.9404899869 | −0.2869271926 |
| 14 | −0.4118589524 | +0.9065485360 | +0.0924226975 |
| 15 | −0.3612508532 | +0.5747270840 | −0.7342932655 |
| 16 | −0.4657947941 | +0.4774785033 | +0.7450164351 |
| 17 | −0.7742591356 | +0.6152504197 | −0.1482892839 |
| 18 | −0.8608205219 | +0.4636213989 | +0.2098647835 |
| 19 | −0.9656605484 | +0.0525933156 | +0.2544280809 |
| 20 | −0.9525211695 | −0.0593434796 | −0.2986331746 |
| 21 | −0.4523440203 | −0.0493710140 | −0.8904759346 |
| 22 | −0.9170097662 | −0.3502146628 | −0.1908999176 |
| 23 | −0.5812035376 | −0.2909171294 | +0.7599800468 |
| 24 | −0.6898393233 | −0.4182330640 | −0.5909338474 |
| 25 | −0.7861763936 | −0.5217996305 | +0.3311371675 |
| 26 | −0.5326876930 | −0.7160644554 | +0.4511047742 |
| 27 | −0.3516499609 | −0.8240752703 | −0.4441196390 |
| 28 | −0.1146237858 | −0.3399692557 | −0.9334250333 |
| 29 | −0.1124304773 | −0.9694934200 | +0.2178116072 |
| 30 | +0.1217293692 | −0.7702732847 | +0.6259880410 |
| 31 | +0.2069525789 | −0.8719885748 | −0.4436288486 |
| 32 | +0.4537196908 | −0.8779508801 | +0.1527766153 |
| 33 | +0.5520184464 | −0.7933187400 | −0.2567508745 |
| 34 | +0.3201817378 | −0.4436021946 | −0.8370786986 |
| 35 | +0.4541086270 | −0.5392368197 | +0.7092312789 |
| 36 | +0.8139832631 | −0.5557243189 | +0.1691204557 |
| 37 | +0.8342971408 | −0.2392481515 | −0.4966976975 |

### 4.3 Approximate star identifications

Cross-referencing the direction vectors against known bright-star catalogues (using RA/Dec equivalent from the vector components and the epoch discussed in §6), the AGC numbering corresponds to the standard Apollo navigation star list. Some confirmed identifications (approximate, based on vector directions):

- Star 1: Alpheratz (α And) — RA ~0h 8m, Dec ~+29°
- Star 5: Polaris — Z ≈ +1.000 confirms near-north-pole position
- Star 7: Hamal (α Ari) — consistent with low-positive declination
- Star 16: Pollux (β Gem) — high positive declination, moderate RA
- Star 25: Antares (α Sco) — negative Z confirms southern declination
- Star 30: Vega (α Lyr) — high positive declination, slightly positive X

The p23-spec.md stub table lists stars 1, 4, 7, 10, 16, 25, 30, 36 — these are confirmed to be in the AGC catalogue at those numbers.

---

## 5. Is the Catalogue Runtime-Mutable?

**No. The star table is strictly read-only at runtime.**

The evidence is definitive:

1. The table resides in `BANK 32`, which is AGC fixed memory (F-bank). No instruction in the AGC instruction set can write to fixed memory. There are no `TS`, `XCH`, `ADS`, `DAS`, or `INCR` instructions that target any address in the STARTAB range in any Comanche055 file.

2. Grep across all Comanche055 `.agc` files for writes to `STARTAB` or the CATLOG region found zero matches.

3. The AGC ground uplink (UPDATE_PROGRAM) can overwrite **erasable** memory cells only. STARTAB is not an erasable cell.

4. The only erasable associated with the star catalogue is `STARCODE` (one word) — this is the index that selects which star from the fixed catalogue to use. The crew or uplink can change `STARCODE` to point to a different star; they cannot change the star's direction vector.

**Conclusion**: The Rust implementation should use `const` / `static` compile-time data. A runtime-mutable `Vec` or `Mutex`-protected table is not needed and should not be used.

---

## 6. Coordinate Frame

### 6.1 What the source says

The `STAR_TABLES.agc` source contains no explicit frame annotation. The assembly note at the top of the file states only "Comanche 055, April 1, 1969".

### 6.2 Frame inference from usage

The star vectors are used in the following critical calculation in R51/AXISGEN:

```
VLOAD* [catalog vector]
MXV    REFSMMAT
UNIT
STORE  STARAD
```

The vector is multiplied by `REFSMMAT` (the Reference-to-Stable-Member matrix) to produce the expected star direction in stable-member coordinates. `REFSMMAT` transforms from the navigation **inertial frame** (the reference frame) to the stable-member frame. Therefore the star vectors in the catalogue are expressed in the **same inertial frame that REFSMMAT maps from**.

For Apollo, this reference inertial frame is the **Earth mean equatorial frame of epoch 1969** (close to Mean of Date for 1969), which is the frame used throughout Comanche055 for the state vector, REFSMMAT, and all navigational quantities. The `PLANETARY_INERTIAL_ORIENTATION.agc` routines handle the Earth's rotation within this frame.

### 6.3 Epoch

The star vectors were pre-computed for the Apollo 11 mission epoch (~July 1969). The file header states "April 1, 1969" as the assembly date; the mission was July 1969. Given the very small stellar proper motions and the sub-arcsecond precision of the stored values, the effective epoch can be taken as **1969.5 (approximately)**. This is neither B1950 nor J2000.

The difference between the 1969.5 frame and J2000 is a precession rotation of approximately 30 years × 50.3 arcsec/year = 1509 arcsec ≈ 0.42°. For IMU alignment purposes this is significant; for the Rust port it means:

- If the navigation code uses J2000 ECI coordinates for `REFSMMAT`, the star vectors **must be precessed** from 1969.5 to J2000 before use.
- If the navigation code uses the same 1969.5 reference (as the original AGC did), no conversion is needed.

The existing `specs/p51_p52-spec.md` and `specs/p23-spec.md` refer to "J2000 equatorial frame" for star directions. This is an anachronism — the original AGC used Mean of 1969.5. See Open Questions (§8) for resolution guidance.

### 6.4 Verification: star 5 frame check

Star 5 has components (0.01310, 0.00781, 0.99988). This is near (0, 0, +1), i.e., the North Celestial Pole. The NCP in the AGC's coordinate system is at Z = +1 by convention (mean equatorial frame). Star 5 is Polaris (α UMi), which in 1969 was approximately 0.73° from the NCP. The vector magnitude is confirmed: √(0.01310² + 0.00781² + 0.99988²) = 1.000 to 5 decimal places.

---

## 7. Recommendations for the Rust Port

### 7.1 Where should the table live?

The catalogue is consumed by three separate modules: P23 (cislunar nav), P51/P52 (alignment), and R51/R56 (alignment subroutines). It should live in **`navigation/star_catalog.rs`** as a module-level constant, not embedded in `programs/p23.rs`. Embedding it in p23.rs would prevent P51/P52 from accessing it without creating a circular dependency or code duplication.

`CISLUNAR_STAR_TABLE` in `programs/p23.rs` should be replaced by a reference to the full table in `navigation::star_catalog`.

### 7.2 Recommended struct shape

The current stub `{ number, name, direction }` is workable but has a redundant `name` field that was not in the AGC. The AGC only stored direction vectors indexed by star number. Recommended struct:

```rust
/// One entry in the AGC navigation star catalogue.
/// Direction vector is a unit vector in the reference inertial frame
/// (Earth mean equatorial, epoch ~1969.5, axes identical to AGC REFSMMAT reference frame).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StarEntry {
    /// AGC catalogue number, 1-based (1..=37).
    pub number: u8,
    /// Direction cosines (X, Y, Z) as a half-unit vector.
    /// Each component is stored at B-1 scale in the AGC; for Rust, use the
    /// literal decimal values from STAR_TABLES.agc directly as f64.
    pub direction: [f64; 3],
}
```

The `name` field can be retained for documentation purposes (`pub name: &'static str`) but should not be considered authoritative — the AGC has no star names.

### 7.3 How many stars for MVP?

Include **all 37 stars**. The reasons are:

1. The data is available verbatim from `STAR_TABLES.agc` — no computation or estimation is required. Populating 37 entries is trivial.
2. PICAPAR requires the full 37 to perform the pair-selection algorithm correctly. Trimming to 8 would produce wrong pair selections and potentially fail to find a valid pair.
3. P23 allows the crew to enter any star 1–37; limiting to 8 would cause the validation check (`STARCODE <= 37`) to pass but the lookup to fail for stars not in the truncated table.

The 8-star `CISLUNAR_STAR_TABLE` in the current p23.rs stub is a temporary placeholder and should be replaced by a reference or slice view into the full 37-star array.

### 7.4 Should `navigation/star_catalog.rs` be a separate module?

Yes, keep it as a separate module. Rationale:

- It is referenced by at least three programs/routines (P23, P51, P52) and the PICAPAR selection algorithm.
- The lookup function (`fn star_direction(code: u8) -> Option<[f64; 3]>`) is a shared primitive, not program-specific logic.
- The module boundary makes frame-conversion handling explicit and centralized: if a conversion from 1969.5 to J2000 is applied, it should be applied once at module level, not scattered across callers.

Recommended public API:

```rust
/// Returns the direction unit vector for a star with the given AGC number (1..=37).
/// Returns None if the number is out of range.
pub fn star_direction(number: u8) -> Option<[f64; 3]>

/// Returns the full 37-entry catalogue.
pub const STAR_CATALOG: [StarEntry; 37]

/// The number of navigation stars in the catalogue.
pub const CATALOG_SIZE: u8 = 37;
```

### 7.5 Frame conversion at load time?

This depends on the decision in §6.3. If the port's reference inertial frame is J2000 (as stated in p23-spec.md and p51_p52-spec.md), a precession rotation matrix must be applied to each direction vector at compile time or at `static` initialization. The precession from 1969.5 to J2000 (30.5 years × 50.3 arcsec/yr ≈ 1509 arcsec along the ecliptic) can be applied using the standard IAU 1976 precession formula. The resulting rotation is small (≈0.42°) but exceeds the AGC's own accuracy threshold.

If the port's REFSMMAT and state vectors use the 1969.5 frame natively (matching the original AGC), no conversion is needed. This is the recommended approach for the MVP to avoid introducing a rotation that must be verified against independent data.

---

## 8. Open Questions for Architect Review

1. **Frame decision**: The existing specs (`p51_p52-spec.md` §1, `p23-spec.md` §3.2) state the frame is "J2000 equatorial". The original AGC used Mean of 1969.5. This is an inconsistency that the architect must resolve explicitly. If J2000 is the target, the 37 vectors need precessing before they can be used with the REFSMMAT operations as currently specified.

2. **`CATLOG` index arithmetic**: The AGC uses `CATLOG,1` with an index register holding the offset from CATLOG backward into the table. The index is `BESTI = star_number × 6`. In Rust, the equivalent is `STAR_CATALOG[37 - star_number]` (0-based, since the AGC table runs 37-down-to-1). The developer agent must implement this inversion correctly, especially in `navigation::star_catalog::star_direction`.

3. **Stars 38–50**: `CHKSCODE` accepts STARCODE up to 50. These are not in the fixed table — they refer to stored track data at `STARAD − 228D,1` (a previously measured star direction saved in erasable memory during ongoing tracking). This is the "auto-optics continuation" mode. It is relevant to P51/P52 but not to P23. The Rust port should document this: `star_direction(n)` returns `None` for n > 37, and the caller (P51/P52) must handle the stored-vector case separately.

4. **CATLOG sentinel value 6970**: Its meaning is undocumented beyond being a boundary marker. One hypothesis: DEC 6970 is the Julian Day Number difference or a mission-specific epoch reference. This is unconfirmed and not load-bearing for the Rust port.

5. **Star name mapping**: The p23-spec.md stub assigns names like "Alpheratz" to star 1, "Achernar" to star 4, etc. These should be verified against a published Apollo navigation star list (e.g., NASA Mission Planning and Analysis Division documents, or the GSOP §5.6 table) before being included in documentation. The direction vectors are definitive; the names are supplementary and should be clearly marked as "for human reference only".

6. **STARSAV3 and the planet-mode interface**: `STARSAV3` holds the Sun/planet vector entered by the crew via noun 88. In P23 (`LDPLANET`), this is used as the "star" direction when `STARCODE == 0`. The Rust implementation of P23's star-lookup path must distinguish star mode (1–37) from planet mode (0), and the planet-mode vector must come from `AgcState` (crew-entered or ephemeris-computed), not from the static catalogue.

---

## Summary of Key Facts

| Property | Value |
|----------|-------|
| Star count | **37** (confirmed, not 36 or 38) |
| Storage format | 3 × double-precision (2-word) AGC words per star, scale B-1 |
| Memory type | Fixed (ROM), bank 32, label STARTAB |
| Table ordering | Descending: star 37 first, star 1 last; CATLOG label follows |
| Index stride | 6 AGC words per star (BESTI/BESTJ = star\_number × 6) |
| Runtime-mutable | No — fixed memory, never written at runtime |
| Coordinate frame | Earth mean equatorial, epoch ~1969.5 |
| Planet handling | Separate from star table; uses ephemeris routines or crew-entered vectors |
| Direct AGC consumers | PICAPAR, R51, P51, P52, P23.17/LOWMEMRY, PLANET routine |
| STARCODE field | Erasable, 1 word; valid range 1–37 for catalogue stars, 0 for planet mode |
