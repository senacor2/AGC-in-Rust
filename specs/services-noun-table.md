# Functional Specification: Noun Table (Data Descriptors)

```
AGC source: Comanche055/PINBALL_NOUN_TABLES.agc
Routines:   LODNNTAB, NNADTAB, NNTYPTAB, IDADDTAB, RUTMXTAB, SFINTAB, SFOUTAB
Pages:      268-284 (BANK 06, SETLOC PINBALL3)
```

---

## 1. Behavior Summary

The noun table maps a two-digit decimal noun code to a description of what data to display in R1, R2, and R3. Each noun entry consists of three parallel tables:

- **NNADTAB** (noun address table): one entry per noun, containing the ECADR (erasable-memory address) of the noun's data for normal nouns, or a packed `IDADDREL` pointer for mixed nouns.
- **NNTYPTAB** (noun type table): packed 15-bit word encoding component count, display format (SF routine), and scale factor constant code.
- **IDADDTAB** (indirect address table): for mixed nouns (N40–N99), three consecutive ECADRs for R1, R2, R3 data sources.
- **RUTMXTAB** (SF routine table for mixed nouns): SF routine codes for each component.

### 1.1 Normal vs. Mixed Nouns

- **Normal nouns (N00–N39)**: one ECADR in NNADTAB; components are contiguous erasable words starting at that address.
- **Mixed nouns (N40–N99)**: IDADDTAB holds three independent ECADRs (components may be non-contiguous); RUTMXTAB holds the three SF routine codes.

The AGC distinguishes them by: if noun ≥ 40 (octal 50 = `MIXCON`), it is mixed.

### 1.2 NNTYPTAB Encoding

For normal nouns, `NNTYPTAB[noun]` is a packed 15-bit word `MMMMMNNNNNPPPPP`:
- **HI5 (MMMMM)**: component code number (component count + flags).
- **MID5 (NNNNN)**: SF routine code number (display format).
- **LOW5 (PPPPP)**: SF constant code number (scale factor).

Component code flags (from page 268):
- Bits 0–1: number of components (0 = 1-comp, 1 = 2-comp, 2 = 3-comp).
- Bit 3 (BIT4): decimal-only flag (alarms if used with octal verb).
- Bit 4 (BIT5): no-load flag (cannot be loaded by load verbs).

SF routine code numbers (from page 268):
- 0: octal only
- 1: straight fractional
- 2: CDU degrees (XXX.XX)
- 3: arithmetic SF
- 4: ARITH DP1 (out ×2^14)
- 5: ARITH DP2 (out straight)
- 6: Y-optics degrees
- 7: ARITH DP3 (out shift-left-7)
- 8: HMS (hours:minutes:seconds)
- 9: minutes:seconds (M/S)
- 10: ARITH DP4 (out shift-left-3)
- 11: ARITH1 SF (out ×2^14)
- 12: 2 integers in D1D2, D4D5, D3 blank
- 13: DP straight fractional

### 1.3 Scale Factor Constants (SFOUTAB)

`SFOUTAB` (page 279) provides DP scale factor constants indexed by SF constant code. These are applied by the SF routines to convert raw AGC fixed-point values to display units. In the Rust implementation, these are represented as `f64` scale factors in `FieldDesc.scale`.

Selected scale factor mappings (from SFOUTAB and comments, page 278-280):

| SF const code | Name | Physical scale | AGC source |
|---|---|---|---|
| 0 | Whole, DP time sec | 1 count = 1 (dimensionless/centiseconds) | `SFOUTAB` entry 0 |
| 2 | CDU degrees | 360/2^15 deg/count = 0.010986° | `DEGOUTSF` |
| 3 | DP degrees (90) | 90/2^14 deg/count | `SFOUTAB` entry 3 |
| 4 | DP degrees (360) | 360/2^14 deg/count | `SFOUTAB` entry 4 |
| 9 | VEL2 (ft/s) | 2^14 counts = full scale, SFOUTAB entry 9 | `SFOUTAB` entries 9-10 |
| 10 | VEL3 (ft/s) | VELOCITY3, shift-left-7 | `SFOUTAB` entries 11-12 |
| 8 | POS4 (naut mi) | POSITION4 | `SFOUTAB` entries 7-8 |

For the M5 noun set, the developer is responsible for mapping each `DataSource` to the corresponding `f64` scale factor from `SFOUTAB`.

---

## 2. Selected Noun Set (11 Nouns for M5)

The 11 required nouns are:

| N | Name | Source (NNADTAB / IDADDTAB) | Type | Format | Fields |
|---|---|---|---|---|---|
| N00 | Not in use | `OCT 00000` | Normal | — | blank/blank/blank |
| N11 | TIG of CSI (hrs,min,sec) | `ECADR TCSI` | Normal | HMS | R1=hrs, R2=min, R3=sec |
| N30 | Target codes | `ECADR DSPTEM1` | Normal | 3-comp whole | R1=code1, R2=code2, R3=code3 |
| N33 | Time of ignition (hrs,min,sec) | `ECADR TIG` | Normal | HMS | R1=hrs, R2=min, R3=sec |
| N36 | AGC clock (MET) | `ECADR TIME2` | Normal | HMS | R1=hrs, R2=min, R3=sec |
| N40 | TIG/cutoff, VG, ΔV accum | Mixed; TTOGO, VGDISP, DVTOTAL | Mixed | M/S, VEL3, VEL3 | R1=TGO, R2=VG, R3=ΔV |
| N44 | Apogee, perigee, TFF | Mixed; HAPOX, HPERX, TFF | Mixed | POS4, POS4, M/S | R1=APO, R2=PER, R3=TFF |
| N62 | Inertial vel, alt rate, alt | Mixed; VMAGI, HDOT, ALTI | Mixed | VEL2, VEL2, POS4 | R1=VI, R2=HDOT, R3=H |
| N85 | VG body-frame components | Mixed; VGBODY, VGBODY+2, VGBODY+4 | Mixed | VEL3, VEL3, VEL3 | R1=X, R2=Y, R3=Z |
| N82 | Delta-V (LV frame) | Mixed; DELVLVC, DELVLVC+2, DELVLVC+4 | Mixed | VEL3, VEL3, VEL3 | R1=X, R2=Y, R3=Z |
| N43 | Latitude, longitude, altitude | Mixed; LAT, LONG, ALT | Mixed | DPDEG(360), DPDEG(360), POS4 | R1=lat, R2=long, R3=alt |

**Rationale for N82 and N43**: N82 (delta-V in local-vertical frame) is directly used by P30 (external delta-V) and P40 (thrusting). N43 (lat/long/altitude) is read by P11 (Earth orbit) and P61 (entry). Both are confirmed implemented M4 programs.

### 2.1 Source References per Noun

From `PINBALL_NOUN_TABLES.agc`:

- N00: `NNADTAB[0] = OCT 00000`, `NNTYPTAB[0] = OCT 00000` (page 270, 275).
- N11: `NNADTAB[11] = ECADR TCSI`, `NNTYPTAB[11] = OCT 24400` (3-comp HMS, dec only) (pages 199, 381).
- N30: `NNADTAB[30] = ECADR DSPTEM1`, `NNTYPTAB[30] = OCT 04140` (3-comp whole) (pages 222, 400).
- N33: `NNADTAB[33] = ECADR TIG`, `NNTYPTAB[33] = OCT 24400` (3-comp HMS, dec only) (pages 224, 403).
- N36: `NNADTAB[36] = ECADR TIME2`, `NNTYPTAB[36] = OCT 24400` (3-comp HMS, dec only) (pages 227, 406).
- N40: `NNADTAB[40] = OCT 64000` (mixed, IDADDREL=0), `NNTYPTAB[40] = OCT 24500` (no-load, dec-only, M/S+VEL3+VEL3), `RUTMXTAB[40] = OCT 16351` (M/S,DP3,DP3) (pages 234, 414, 797).
- N44: `NNADTAB[44] = OCT 64014` (mixed, IDADDREL=14), `NNTYPTAB[44] = OCT 00410` (POS4,POS4,M/S, no-load, dec-only), `RUTMXTAB[44] = OCT 22347` (DP3,DP3,M/S) (pages 245, 422, 801).
- N62: `NNADTAB[62] = OCT 24102` (mixed, IDADDREL=102), `NNTYPTAB[62] = OCT 20451` (VEL2,VEL2,POS4, dec-only), `RUTMXTAB[62] = OCT 16512` (DP4,DP4,DP3) (pages 288, 456, 819).
- N82: `NNADTAB[82] = OCT 24176` (mixed, IDADDREL=176), `NNTYPTAB[82] = OCT 24512` (VEL3, dec-only), `RUTMXTAB[82] = OCT 16347` (DP3,DP3,DP3) (pages 338, 488, 840).
- N85: `NNADTAB[85] = OCT 24207` (mixed, IDADDREL=207), `NNTYPTAB[85] = OCT 24512` (VEL3, dec-only), `RUTMXTAB[85] = OCT 16347` (DP3,DP3,DP3) (pages 341, 494, 843).
- N43: `NNADTAB[43] = OCT 24011` (mixed, IDADDREL=011), `NNTYPTAB[43] = OCT 20204` (DPDEG(360),DPDEG(360),POS4, dec-only), `RUTMXTAB[43] = OCT 16512` (DP4,DP4,DP3) (pages 243, 419, 800).

---

## 3. Rust API

### 3.1 Module Path

`agc_core::services::noun_table`

### 3.2 Types

```rust
/// Data source for one field of a noun definition.
///
/// Maps an AGC ECADR or computed value to a Rust accessor.
/// Each variant corresponds to a specific erasable-memory symbol in Comanche055.
/// Mixed nouns use up to three independent DataSource entries (one per component).
///
/// AGC source: NNADTAB and IDADDTAB, PINBALL_NOUN_TABLES.agc pages 186-234, 608-792.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DataSource {
    /// Not in use (noun 0, spare entries).  Display is all-blank.
    NotUsed,
    /// Mission elapsed time (TIME2 erasable register, centiseconds).
    /// AGC ECADR: TIME2.  N36.
    CurrentMet,
    /// Time of ignition (TIG erasable register, centiseconds).
    /// AGC ECADR: TIG.  N33.
    TargetTig,
    /// Time of CSI ignition (TCSI erasable register).
    /// AGC ECADR: TCSI.  N11.
    TigCsi,
    /// Target code display buffer (DSPTEM1).
    /// AGC ECADR: DSPTEM1.  N30.
    TargetCode,
    /// Time to go to event / cutoff (TTOGO, centiseconds).
    /// AGC ECADR: TTOGO.  N40 R1.
    TimeToGo,
    /// Velocity-to-gain magnitude (VGDISP, ft/s scaled).
    /// AGC ECADR: VGDISP.  N40 R2.
    VgMagnitude,
    /// Accumulated ΔV total (DVTOTAL, ft/s scaled).
    /// AGC ECADR: DVTOTAL.  N40 R3.
    DvAccumulated,
    /// Apogee altitude of target orbit (HAPOX, nautical miles).
    /// AGC ECADR: HAPOX.  N44 R1.
    ApogeeAlt,
    /// Perigee altitude of target orbit (HPERX, nautical miles).
    /// AGC ECADR: HPERX.  N44 R2.
    PerigeeAlt,
    /// Time of free flight (TFF, centiseconds, min:sec display).
    /// AGC ECADR: TFF.  N44 R3.
    TimeFreeFlght,
    /// Inertial velocity magnitude (VMAGI, ft/s).
    /// AGC ECADR: VMAGI.  N62 R1.
    InertialVelMag,
    /// Altitude rate (HDOT, ft/s).
    /// AGC ECADR: HDOT.  N62 R2.
    AltRate,
    /// Altitude above pad radius (ALTI, nautical miles).
    /// AGC ECADR: ALTI.  N62 R3.
    Altitude,
    /// Delta-V in local-vertical frame, X component (DELVLVC).
    /// AGC ECADR: DELVLVC.  N82 R1.
    DeltaVLvcX,
    /// Delta-V in local-vertical frame, Y component (DELVLVC+2).
    /// AGC ECADR: DELVLVC+2.  N82 R2.
    DeltaVLvcY,
    /// Delta-V in local-vertical frame, Z component (DELVLVC+4).
    /// AGC ECADR: DELVLVC+4.  N82 R3.
    DeltaVLvcZ,
    /// VG body-frame, X component (VGBODY).
    /// AGC ECADR: VGBODY.  N85 R1.
    VgBodyX,
    /// VG body-frame, Y component (VGBODY+2).
    VgBodyY,
    /// VG body-frame, Z component (VGBODY+4).
    VgBodyZ,
    /// Geodetic latitude (LAT, degrees).
    /// AGC ECADR: LAT.  N43 R1.
    Latitude,
    /// Geodetic longitude (LONG, degrees).
    /// AGC ECADR: LONG.  N43 R2.
    Longitude,
    /// Altitude above reference ellipsoid (ALT, nautical miles).
    /// AGC ECADR: ALT.  N43 R3.
    AltitudeGeo,
}

/// Display format for one field.
///
/// Maps to AGC SF routine codes (NNTYPTAB MID5 field).
/// PINBALL_NOUN_TABLES.agc pages 268-269.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DisplayFormat {
    /// Octal display only (SF routine 0).
    Octal,
    /// Signed decimal, 5 digits (SF routines 1, 3, 11).
    Decimal,
    /// Hours:minutes:seconds across R1/R2/R3 (SF routine 8).
    /// Only valid when all three fields of a noun share this format.
    Time,
    /// Minutes:seconds in one register (SF routine 9).
    MinSec,
    /// DP degrees (360-degree range, SF routine 4).
    DpDegrees360,
    /// Position in nautical miles, POSITION4 scale (SF routine 7, const code 8).
    Position4Nm,
    /// Velocity in ft/s, VELOCITY2 scale (SF routine 5, const code 9).
    Velocity2Fps,
    /// Velocity in ft/s, VELOCITY3 scale (SF routine 7, const code 10).
    Velocity3Fps,
    /// No data; field is blank.
    Blank,
}

/// Descriptor for one register field within a noun.
#[derive(Clone, Copy, Debug)]
pub struct FieldDesc {
    /// Where to read the data value from AgcState.
    pub source: DataSource,
    /// How to format the value for display.
    pub format: DisplayFormat,
    /// Scale factor: multiply the raw `f64` AgcState value by this to obtain
    /// the integer displayed on the DSKY (before digit extraction).
    /// Units depend on DisplayFormat:
    ///   Decimal/Velocity: raw ft/s or m/s → integer × 0.1 or × 1
    ///   Position4Nm: raw metres → displayed nautical miles × 10 (XXXX.X)
    ///   DpDegrees360: raw radians → displayed degrees × 100 (XXX.XX)
    ///   Time/MinSec: raw centiseconds → unscaled (format_time handles internally)
    ///   Blank: 0.0
    pub scale: f64,
}

/// Complete definition of one noun.
#[derive(Clone, Copy, Debug)]
pub struct NounDef {
    /// Noun number (0–99).
    pub noun: u8,
    /// R1 field descriptor.
    pub r1: FieldDesc,
    /// R2 field descriptor.
    pub r2: FieldDesc,
    /// R3 field descriptor.
    pub r3: FieldDesc,
}
```

### 3.3 Noun Table Constant

```rust
/// Number of noun definitions in the M5 table.
const NOUN_TABLE_LEN: usize = 11;

/// Static noun definitions for M5.
/// Ordered by noun number for O(N) linear scan via `lookup`.
///
/// AGC source: NNADTAB, NNTYPTAB, IDADDTAB, RUTMXTAB.
/// PINBALL_NOUN_TABLES.agc pages 270-284.
///
/// Memory cost: 11 × size_of::<NounDef>() ≈ 11 × 64 = 704 bytes (estimated).
/// This is fixed-size ROM data; acceptable for the Cortex-M4F target.
pub const NOUN_TABLE: [NounDef; NOUN_TABLE_LEN] = [
    // N00: Not in use
    // N11: TIG of CSI (hrs,min,sec)
    // N30: Target codes
    // N33: Time of ignition (hrs,min,sec)
    // N36: AGC clock / MET (hrs,min,sec)
    // N40: TGO / VG / ΔV accumulated (M/S, VEL3, VEL3)
    // N43: Latitude, longitude, altitude
    // N44: Apogee, perigee, TFF (POS4, POS4, M/S)
    // N62: Inertial velocity mag, alt rate, altitude
    // N82: Delta-V LV frame (VEL3, VEL3, VEL3)
    // N85: VG body frame (VEL3, VEL3, VEL3)
    // (developer fills in the actual initialiser)
];
```

### 3.4 Lookup Function

```rust
/// Look up a noun definition by noun number.
///
/// Performs a linear scan of NOUN_TABLE (O(11) = O(1) in practice).
/// Linear scan is chosen over index lookup because the noun numbers are
/// sparse (not contiguous), matching the AGC's indexed table with gaps.
///
/// Returns None for noun numbers not present in the M5 table.
/// The caller (verb dispatch) treats None as an OPERATOR ERROR.
///
/// AGC source: LODNNTAB (PINBALL_NOUN_TABLES.agc page 270) performs
/// indexed table lookup; the Rust linear scan is equivalent for 11 entries.
pub fn lookup(noun: u8) -> Option<&'static NounDef>;
```

---

## 4. Scale Factors

| Noun | Field | AGC units | Display format | Rust scale factor |
|---|---|---|---|---|
| N11, N33, N36 | All | Centiseconds (ECADR of DP time word) | HMS | 1.0 (format_time handles) |
| N30 | R1/R2/R3 | Whole integer (DSPTEM1 words) | Decimal | 1.0 |
| N40 R1 | TGO | Centiseconds | M/S | 1.0 |
| N40 R2/R3 | VG, ΔV | ft/s × 2^(−7) (VELOCITY3 AGC scale) | Decimal | 2^7 = 128.0 → display in ft/s |
| N44 R1/R2 | APO/PER | nautical miles × 2^(−3) (POS4) | Decimal | 2^3 = 8.0 → display in naut mi |
| N44 R3 | TFF | Centiseconds | M/S | 1.0 |
| N62 R1/R2 | VI, HDOT | ft/s × 2^(−14) (VELOCITY2 DP scale) | Decimal | 2^14 = 16384.0 |
| N62 R3 | Alt | nautical miles × 2^(−3) (POS4) | Decimal | 8.0 |
| N82/N85 R1/R2/R3 | VG, ΔV | ft/s × 2^(−7) (VELOCITY3) | Decimal | 128.0 |
| N43 R1/R2 | Lat/Long | Radians, DP (DPDEG 360) | Decimal | 360/(2π) × 100 ≈ 5729.58 |
| N43 R3 | Alt | Nautical miles (POS4) | Decimal | 8.0 |

Note: these scale factors are derived from the SFOUTAB entries in `PINBALL_NOUN_TABLES.agc` pages 279-280 and the SF routine descriptions on pages 268-269. The developer must verify each scale against the SFOUTAB double-precision constants during implementation.

---

## 5. Invariants

1. **Table is static (`const`)**: `NOUN_TABLE` is a `const` array in ROM; no heap allocation, no runtime initialisation required.
2. **Lookup is O(11)**: linear scan is acceptable; the AGC's indexed lookup is also O(1) but requires contiguous noun numbers. Linear scan is more maintainable for a sparse set.
3. **Unknown noun returns `None`**: `lookup` never panics. The caller converts `None` to `VerbResult::Error`.
4. **N40 has exactly 3 burn fields**: N40's R1, R2, R3 are all `Some(FieldDesc)` with non-blank DataSource (TGO, VG, ΔV). Any test for the 3-field invariant on N40 must find all three populated.
5. **Blank fields use `DataSource::NotUsed`**: N00 has all three fields with `source = DataSource::NotUsed` and `format = DisplayFormat::Blank`.
6. **No-load nouns (N40, N44)**: the `NNTYPTAB` bit4=1 flag (no-load) is encoded implicitly in the `DisplayFormat` or as a separate `no_load: bool` flag if the developer finds it necessary for load-verb validation. The spec recommends encoding it as a flag in `NounDef` (`pub no_load: bool`) rather than overloading `DisplayFormat`.

---

## 6. Test Cases

### Test 1: N36 returns MET field descriptor

```
lookup(36)
→ Some(NounDef { noun: 36, r1: FieldDesc { source: DataSource::CurrentMet, format: DisplayFormat::Time, .. },
                             r2: FieldDesc { source: DataSource::CurrentMet, format: DisplayFormat::Time, .. },
                             r3: FieldDesc { source: DataSource::CurrentMet, format: DisplayFormat::Time, .. } })
```

(All three fields point to the same MET source; format_time splits hours/min/sec internally.)

### Test 2: Unknown noun returns None

```
lookup(99)  → None   // N99 not in M5 table
lookup(10)  → None   // N10 not in M5 table
lookup(200) → None   // out of range
```

### Test 3: N40 has 3 populated burn fields

```
let n40 = lookup(40).unwrap();
assert!(n40.r1.source != DataSource::NotUsed);  // TGO
assert!(n40.r2.source != DataSource::NotUsed);  // VG magnitude
assert!(n40.r3.source != DataSource::NotUsed);  // ΔV accumulated
assert_eq!(n40.r1.format, DisplayFormat::MinSec);
assert_eq!(n40.r2.format, DisplayFormat::Velocity3Fps);
assert_eq!(n40.r3.format, DisplayFormat::Velocity3Fps);
```

### Test 4: Noun lookup is const-compatible

```
// Must compile: NOUN_TABLE is a `const` item
const _: &NounDef = &NOUN_TABLE[0];
// lookup at runtime:
assert_eq!(lookup(0).map(|n| n.noun), Some(0));
assert_eq!(lookup(36).map(|n| n.noun), Some(36));
```

---

## 7. agc-sim Impact

- `DskyDisplayState`: no new fields required; verb dispatch uses `lookup` to resolve noun data and calls `format_decimal` / `format_time` / `format_octal` from `services::display`.
- `SimLog`: when a noun is resolved, emit `NOUN  n={:02}  r1_src={:?}  r2_src={:?}  r3_src={:?}`.
- `dsky_terminal.rs`: no structural changes; the noun display (ND1/ND2) is already rendered from `VnState.noun_buf`.
- Scenario files: V06N36 (display MET) and V16N44 (monitor orbit params) must be exercisable via the agc-sim keyboard after M5.
