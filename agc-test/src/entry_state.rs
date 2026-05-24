//! Entry-scenario erasable-state preload for the MS-E7d live yaAGC tests.
//!
//! Encodes a high-level [`EntryInitialState`] (SI units, ECI frame)
//! into a patched [`crate::vagc_harness::CoreImage`] so that a yaAGC
//! run resumed from the patched core can execute P63 without first
//! cold-booting through P52 / V21 / V25 / V52. The patched variables
//! are chosen by reading P61 / REENTRY_CONTROL / ENTRY_LEXICON in
//! `~/dev/virtualagc/Comanche055/`:
//!
//! | AGC symbol | Erasable size | Role |
//! |---|---|---|
//! | `RN`        | DP × 3 (6 words)  | CSM ECI position vector |
//! | `VN`        | DP × 3 (6 words)  | CSM ECI velocity vector |
//! | `TET`       | DP × 1 (2 words)  | Time of the state vector (cs) |
//! | `REFSMMAT`  | DP × 9 (18 words) | Stable-member → ECI rotation |
//! | `LAT(SPL)`  | DP × 1            | Target landing latitude (rev) |
//! | `LNG(SPL)`  | DP × 1            | Target landing longitude (rev) |
//! | `EMSALT`    | DP × 1            | 0.05g altitude (m) |
//! | `ALFAPAD`   | SP × 1            | Hypersonic CM trim alpha (180°) |
//! | `HEADSUP`   | SP × 1            | +1 lift-down, −1 lift-up |
//! | `MODREG`    | SP × 1            | Major mode (0 = P00 idle) |
//! | `CMDAPMOD`  | SP × 1            | Entry-DAP mode (-1 = direct P63) |
//! | `FLAGWRD3`  | SP × 1            | OR-in REFSMFLG bit 13 |
//!
//! ## B-scaling
//!
//! The conversion uses [`crate::agc_convert::to_agc_dword`], whose
//! `scale` argument is the **LSB exponent** in the engineering units
//! used by the caller. The mapping from AGC source "B(N)" notation is
//! `scale = N − 28` for DP variables. The scale choices below are
//! documented per-variable against the AGC source. They are
//! **Rust-side round-trip-tested** here in this module, but full
//! AGC-acceptance validation only happens when the live yaAGC test
//! (`tests/entry_e2e_vagc.rs`) runs with a local VirtualAGC build —
//! see `docs/entry_channel_trace.md` for the verification path.

use crate::vagc_harness::{read_scaled, AgcAddress, CoreImage, ScaledVar, Symtab};
use crate::{agc_convert, vagc_harness::write_scaled};

use std::f64::consts::TAU;

/// Initial state describing one entry scenario in SI units.
#[derive(Clone, Debug)]
pub struct EntryInitialState {
    /// ECI position (m). 3-axis components.
    pub position_m: [f64; 3],
    /// ECI velocity (m/s). 3-axis components.
    pub velocity_mps: [f64; 3],
    /// Time of the state vector (s) from the AGC clock's epoch. The
    /// AGC stores time in centiseconds.
    pub time_s: f64,
    /// Target landing latitude (radians, north positive).
    pub target_lat_rad: f64,
    /// Target landing longitude (radians, east positive).
    pub target_lon_rad: f64,
    /// 0.05g monitoring altitude above the Fischer ellipsoid (m).
    /// Apollo nominal: 122 km (400 000 ft). Loaded as a pad constant
    /// in flight; we set it explicitly here.
    pub emsalt_m: f64,
    /// Hypersonic CM trim angle of attack (degrees). Apollo nominal
    /// for the CM is ≈ −20°. The AGC stores it as `alfa / 180°`.
    pub alfa_pad_deg: f64,
    /// `true` = lift up (HEADSUP = −1); `false` = lift down (+1).
    /// Apollo entry default: lift up.
    pub lift_up: bool,
    /// REFSMMAT (3 × 3 stable-member → ECI rotation). Row-major.
    /// Identity matrix when ECI ≡ IMU (the scenario default).
    pub refsmmat: [[f64; 3]; 3],
    /// `CMDAPMOD` — the entry DAP mode selector
    /// (`Comanche055/ERASABLE_ASSIGNMENTS.agc` line 767 = `E6,1700`).
    /// P62 reads this after PROCEED to decide whether to schedule
    /// `WAKEP62` (a task that waits for the body-attitude maneuver to
    /// complete) or jump directly to P63:
    ///
    /// ```text
    /// CS  CMDAPMOD     # P61-P67.agc:260-265
    /// MASK ONE
    /// BZF  P63.1       # taken if CMDAPMOD = -0 or +1  → WAKEP62 path
    ///                  # otherwise (e.g. CMDAPMOD = -1 or +0) → TC P63 direct
    /// ```
    ///
    /// Default `-1` (`0o77776` in 15-bit ones-complement) skips the
    /// attitude-maneuver wait, which the harness does not simulate.
    /// Stored as a raw 15-bit ones-complement word (not scaled).
    ///
    /// **Note (MS-E7g)**: the preload is in place but does not yet
    /// drive a non-zero ROLLC end-to-end — P62's GOFLASH wait for
    /// PROCEED currently doesn't wake on either V33 ENTR or the
    /// hardware PRO discrete. See `docs/entry_channel_trace.md` and
    /// the MS-E7h follow-up.
    pub cmdapmod: i16,
}

impl EntryInitialState {
    /// Build a 3×3 identity matrix for REFSMMAT defaults.
    pub fn identity_refsmmat() -> [[f64; 3]; 3] {
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    }
}

/// Errors patching an [`EntryInitialState`] into a [`CoreImage`].
#[derive(Debug)]
pub enum PatchError {
    /// A required AGC symbol was not found in the symbol table. The
    /// `symbol` field names the missing identifier; the most likely
    /// cause is a stale or wrong assembly listing.
    MissingSymbol { symbol: &'static str },
    /// A required AGC symbol resolved to a fixed-memory address. The
    /// patch path only writes to erasable memory.
    SymbolInFixed { symbol: &'static str },
    /// `write_sp` / `write_dp` returned `false` for one of the patched
    /// variables — usually an out-of-bank offset.
    WriteRejected {
        symbol: &'static str,
        addr: AgcAddress,
    },
}

impl std::fmt::Display for PatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PatchError::MissingSymbol { symbol } => {
                write!(f, "AGC symbol '{symbol}' not found in symbol table")
            }
            PatchError::SymbolInFixed { symbol } => write!(
                f,
                "AGC symbol '{symbol}' resolves to fixed memory — cannot patch"
            ),
            PatchError::WriteRejected { symbol, addr } => {
                write!(f, "core image rejected write to {symbol} @ {:?}", addr)
            }
        }
    }
}

impl std::error::Error for PatchError {}

// ── B-scale constants ──────────────────────────────────────────────────────

/// Position scale: AGC `RN` is "B(6)PRM" with single-precision word at
/// `B+7 m` per `agc_convert::from_agc_word` documentation; DP extends
/// the LSB by 2^14. With our `to_agc_dword(value_m, scale)` convention,
/// `scale = 0` gives 1 LSB = 1 m and full-scale = 2^28 m ≈ 268 Mm —
/// plenty for both LEO and lunar-return entry interfaces.
const SCALE_POSITION_M: i8 = 0;

/// Velocity scale: AGC `VN` is documented at B+7. Working in m/cs
/// (m/s ÷ 100) with `scale = -21` gives 1 LSB ≈ 4.77 × 10⁻⁷ m/cs and
/// full-scale = 2⁷ m/cs = 128 m/cs ≈ 12.8 km/s — covers direct-LEO
/// (7.9 km/s) and lunar-return (11 km/s) by margin.
const SCALE_VELOCITY_M_PER_CS: i8 = -21;

/// Time-of-state-vector scale: AGC `TET` is "B(2)TMP CSECS*2(-28)".
/// With value in cs and `scale = 0`, 1 LSB = 1 cs and full-scale =
/// 2^28 cs ≈ 31 days — sufficient for an entry MET that starts at 0.
const SCALE_TIME_CS: i8 = 0;

/// Generic "fraction of 1" scale (`B+0` DP). 1 LSB = 2^-28.
/// Used for REFSMMAT direction cosines (each element ∈ [−1, 1)) and
/// for the target lat/lon expressed as a fraction of one revolution.
const SCALE_FRACTION: i8 = -28;

/// Altitude scale (m). EMSALT is documented "I(2)PL" with no specific
/// B-scale in `ERASABLE_ASSIGNMENTS.agc:2309`. We use the same scale
/// as RN (B+28 m, scale = 0) for symmetry — 1 LSB = 1 m, full-scale
/// 268 Mm.
const SCALE_ALTITUDE_M: i8 = 0;

/// Alpha-pad scale (single-precision fraction of 180°). AGC: "B(1)PL
/// ALFA TRIM / 180" — SP word stores fraction of 180° directly. With
/// single-precision and `scale = -14`, 1 LSB ≈ 6 × 10⁻⁵ × 180° ≈ 0.01°.
const SCALE_ALFA_FRACTION_SP: i8 = -14;

/// REFSMFLG bit position within FLAGWRD3 (1-indexed from the LSB
/// per the AGC convention; comment in `ERASABLE_ASSIGNMENTS.agc:352`
/// says "BIT 13 FLAG 3"). The set mask is `1 << (13 − 1) = 0x1000`.
const REFSMFLG_BIT_MASK: u16 = 1 << 12;

// ── Closed-loop bank readout (MS-E7e) ──────────────────────────────────────

/// Read the AGC's current entry bank command `ROLLC` (a DP erasable
/// holding `ROLLCOM / 360`) and return the corresponding bank angle in
/// radians, sign convention matching
/// [`crate::entry_sim::EntryIntegrator::integrate_cycle`]'s `bank_rad`
/// argument (0 = lift up, positive = right-bank).
///
/// `ROLLC` is at the AGC symbol "ROLLC" (defined as `ROLLTM + 1` in
/// `Comanche055/ERASABLE_ASSIGNMENTS.agc:3181`). Returns `None` if the
/// symbol is missing from the table (stale assembly listing) or
/// resolves to fixed memory.
pub fn read_rollc_rad(core: &CoreImage, symtab: &Symtab) -> Option<f64> {
    let addr = symtab.get("ROLLC")?;
    if !matches!(addr, AgcAddress::Erasable { .. }) {
        return None;
    }
    let fraction = read_scaled(
        core,
        &ScaledVar {
            addr,
            scale: SCALE_FRACTION,
            dp: true,
        },
    )?;
    Some(fraction * TAU)
}

// ── Patch entry point ──────────────────────────────────────────────────────

/// Patch the AGC erasable state inside `core` with the variables that
/// `EntryInitialState` carries. Returns `Ok(())` on success, or a
/// `PatchError` naming the first symbol that failed to resolve or
/// failed to write.
pub fn patch_into(
    core: &mut CoreImage,
    symtab: &Symtab,
    state: &EntryInitialState,
) -> Result<(), PatchError> {
    // Position: RN (DP × 3 components, value in m).
    write_dp_vec3(core, symtab, "RN", SCALE_POSITION_M, state.position_m)?;

    // Velocity: VN (DP × 3 components, value in m/cs).
    let v_mpercs = [
        state.velocity_mps[0] * 0.01,
        state.velocity_mps[1] * 0.01,
        state.velocity_mps[2] * 0.01,
    ];
    write_dp_vec3(core, symtab, "VN", SCALE_VELOCITY_M_PER_CS, v_mpercs)?;

    // TET (DP × 1, value in cs).
    write_dp_scalar(core, symtab, "TET", SCALE_TIME_CS, state.time_s * 100.0)?;

    // REFSMMAT (DP × 9, row-major identity by default).
    write_refsmmat(core, symtab, &state.refsmmat)?;

    // Target landing site — "LAT(SPL)" / "LNG(SPL)" stored as a
    // fraction of one revolution per `ERASABLE_ASSIGNMENTS.agc:3366`.
    write_dp_scalar(
        core,
        symtab,
        "LAT(SPL)",
        SCALE_FRACTION,
        state.target_lat_rad / TAU,
    )?;
    write_dp_scalar(
        core,
        symtab,
        "LNG(SPL)",
        SCALE_FRACTION,
        state.target_lon_rad / TAU,
    )?;

    // EMSALT (DP × 1, value in m).
    write_dp_scalar(core, symtab, "EMSALT", SCALE_ALTITUDE_M, state.emsalt_m)?;

    // ALFAPAD (SP × 1, value in fraction of 180°).
    write_sp_scalar(
        core,
        symtab,
        "ALFAPAD",
        SCALE_ALFA_FRACTION_SP,
        state.alfa_pad_deg / 180.0,
    )?;

    // HEADSUP (SP × 1, raw count). HEADSUP = −1 means lift up.
    write_sp_raw(
        core,
        symtab,
        "HEADSUP",
        if state.lift_up {
            // −1 as 15-bit ones-complement = 0x7FFE (≡ −1 in i16).
            ones_complement_word(-1)
        } else {
            1
        },
    )?;

    // MODREG (SP × 1, raw count). 0 = P00 idle so V37 ENTR 62 ENTR
    // is accepted on resume.
    write_sp_raw(core, symtab, "MODREG", 0)?;

    // CMDAPMOD (SP × 1, raw ones-complement count). See the
    // `EntryInitialState::cmdapmod` doc-comment for the AGC decision
    // path. Stored as the caller-supplied i16 packed to 15-bit
    // ones-complement.
    write_sp_raw(
        core,
        symtab,
        "CMDAPMOD",
        ones_complement_word(state.cmdapmod),
    )?;

    // FLAGWRD3 — OR-in REFSMFLG bit 13 without disturbing the other
    // 14 flag bits. Read-modify-write.
    let flag_addr = lookup_erasable(symtab, "FLAGWRD3")?;
    let current = core.read_sp(flag_addr).ok_or(PatchError::WriteRejected {
        symbol: "FLAGWRD3",
        addr: flag_addr,
    })?;
    let updated = current | REFSMFLG_BIT_MASK;
    if !core.write_sp(flag_addr, updated) {
        return Err(PatchError::WriteRejected {
            symbol: "FLAGWRD3",
            addr: flag_addr,
        });
    }

    Ok(())
}

// ── Internal helpers ───────────────────────────────────────────────────────

fn lookup_erasable(symtab: &Symtab, symbol: &'static str) -> Result<AgcAddress, PatchError> {
    let addr = symtab
        .get(symbol)
        .ok_or(PatchError::MissingSymbol { symbol })?;
    match addr {
        AgcAddress::Erasable { .. } => Ok(addr),
        AgcAddress::Fixed { .. } => Err(PatchError::SymbolInFixed { symbol }),
    }
}

fn write_dp_scalar(
    core: &mut CoreImage,
    symtab: &Symtab,
    symbol: &'static str,
    scale: i8,
    value: f64,
) -> Result<(), PatchError> {
    let addr = lookup_erasable(symtab, symbol)?;
    let ok = write_scaled(
        core,
        &ScaledVar {
            addr,
            scale,
            dp: true,
        },
        value,
    );
    if !ok {
        return Err(PatchError::WriteRejected { symbol, addr });
    }
    Ok(())
}

fn write_dp_vec3(
    core: &mut CoreImage,
    symtab: &Symtab,
    symbol: &'static str,
    scale: i8,
    value: [f64; 3],
) -> Result<(), PatchError> {
    let base = lookup_erasable(symtab, symbol)?;
    for (i, component) in value.iter().enumerate() {
        let component_addr = bump_dp(base, i)?;
        let ok = write_scaled(
            core,
            &ScaledVar {
                addr: component_addr,
                scale,
                dp: true,
            },
            *component,
        );
        if !ok {
            return Err(PatchError::WriteRejected {
                symbol,
                addr: component_addr,
            });
        }
    }
    Ok(())
}

fn write_sp_scalar(
    core: &mut CoreImage,
    symtab: &Symtab,
    symbol: &'static str,
    scale: i8,
    value: f64,
) -> Result<(), PatchError> {
    let addr = lookup_erasable(symtab, symbol)?;
    let (hi, _) = agc_convert::to_agc_dword(value, scale);
    if !core.write_sp(addr, hi) {
        return Err(PatchError::WriteRejected { symbol, addr });
    }
    Ok(())
}

fn write_sp_raw(
    core: &mut CoreImage,
    symtab: &Symtab,
    symbol: &'static str,
    raw: u16,
) -> Result<(), PatchError> {
    let addr = lookup_erasable(symtab, symbol)?;
    if !core.write_sp(addr, raw & 0x7FFF) {
        return Err(PatchError::WriteRejected { symbol, addr });
    }
    Ok(())
}

fn write_refsmmat(
    core: &mut CoreImage,
    symtab: &Symtab,
    refsmmat: &[[f64; 3]; 3],
) -> Result<(), PatchError> {
    let base = lookup_erasable(symtab, "REFSMMAT")?;
    let mut idx = 0;
    for row in refsmmat.iter() {
        for &cell in row.iter() {
            let cell_addr = bump_dp(base, idx)?;
            let ok = write_scaled(
                core,
                &ScaledVar {
                    addr: cell_addr,
                    scale: SCALE_FRACTION,
                    dp: true,
                },
                // Direction-cosine ±1 saturates at +1 - 2^-28; clamp
                // toward 0 by an epsilon so writing identity does not
                // hit the AGC's ones-complement overflow boundary.
                cell.clamp(-1.0 + 2.0_f64.powi(-28), 1.0 - 2.0_f64.powi(-28)),
            );
            if !ok {
                return Err(PatchError::WriteRejected {
                    symbol: "REFSMMAT",
                    addr: cell_addr,
                });
            }
            idx += 1;
        }
    }
    Ok(())
}

/// Advance `base` by `i` DP cells (two 15-bit words each). Used to
/// walk a multi-component erasable variable like RN, VN, REFSMMAT.
fn bump_dp(base: AgcAddress, i: usize) -> Result<AgcAddress, PatchError> {
    match base {
        AgcAddress::Erasable { bank, offset } => Ok(AgcAddress::Erasable {
            bank,
            offset: offset + (2 * i as u16),
        }),
        AgcAddress::Fixed { .. } => Err(PatchError::SymbolInFixed {
            symbol: "(multi-DP)",
        }),
    }
}

/// Encode a small signed integer as the AGC's 15-bit ones-complement
/// word. Negative numbers use the upper half-range (`0x4001..=0x7FFE`).
fn ones_complement_word(value: i16) -> u16 {
    if value >= 0 {
        value as u16
    } else {
        // Ones-complement: −x ⇒ NOT(x), 15-bit.
        (!(value.unsigned_abs())) & 0x7FFF
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vagc_harness::read_scaled;

    /// Hand-built symbol table covering every variable the patcher
    /// touches. Offsets are arbitrary (within the same bank to keep
    /// `bump_dp` simple) — they only need to be unique so the round-
    /// trip can read each value back.
    fn fixture_symtab() -> Symtab {
        let text = "\
preamble\n\
Symbol Table\n\
------------\n\
     1,F:   stub        04,2000  \n\
     2,E:   RN          E5,1400  \t   3,E:   VN          E5,1414  \n\
     4,E:   TET         E5,1430  \t   5,E:   REFSMMAT    E5,1434  \n\
     6,E:   LAT(SPL)    E5,1500  \t   7,E:   LNG(SPL)    E5,1504  \n\
     8,E:   EMSALT      E5,1510  \t   9,E:   ALFAPAD     E5,1514  \n\
    10,E:   HEADSUP     E5,1516  \t  11,E:   MODREG      E5,1520  \n\
    12,E:   FLAGWRD3    E5,1522  \t  13,E:   ROLLC       E5,1524  \n\
    14,E:   CMDAPMOD    E5,1526  \n\
";
        Symtab::parse(text)
    }

    fn empty_core() -> CoreImage {
        CoreImage::empty()
    }

    fn default_state() -> EntryInitialState {
        EntryInitialState {
            position_m: [6_493_000.0, 0.0, 0.0],
            velocity_mps: [-825.0, 7860.0, 0.0],
            time_s: 0.0,
            target_lat_rad: 0.0,
            target_lon_rad: 20.0_f64.to_radians(),
            emsalt_m: 122_000.0,
            alfa_pad_deg: -20.0,
            lift_up: true,
            refsmmat: EntryInitialState::identity_refsmmat(),
            cmdapmod: -1,
        }
    }

    /// TC-ES-1: patching an empty core succeeds and every written
    /// variable round-trips through `read_scaled` within 1 LSB.
    #[test]
    fn tc_es_1_round_trip_scalar_values() {
        let symtab = fixture_symtab();
        let mut core = empty_core();
        let state = default_state();

        patch_into(&mut core, &symtab, &state).expect("patch_into ok");

        // TET → 0 s → 0 cs.
        let tet = read_scaled(
            &core,
            &ScaledVar {
                addr: symtab.get("TET").unwrap(),
                scale: SCALE_TIME_CS,
                dp: true,
            },
        )
        .unwrap();
        assert!(tet.abs() < 1.0, "TET round-trip: {tet}");

        // EMSALT → 122 km.
        let emsalt = read_scaled(
            &core,
            &ScaledVar {
                addr: symtab.get("EMSALT").unwrap(),
                scale: SCALE_ALTITUDE_M,
                dp: true,
            },
        )
        .unwrap();
        assert!(
            (emsalt - 122_000.0).abs() < 1.0,
            "EMSALT round-trip: {emsalt}"
        );

        // LAT(SPL) → 0 → 0 rev.
        let lat = read_scaled(
            &core,
            &ScaledVar {
                addr: symtab.get("LAT(SPL)").unwrap(),
                scale: SCALE_FRACTION,
                dp: true,
            },
        )
        .unwrap();
        assert!(lat.abs() < 2.0_f64.powi(-28));

        // LNG(SPL) → 20° → 20/360 rev.
        let lng = read_scaled(
            &core,
            &ScaledVar {
                addr: symtab.get("LNG(SPL)").unwrap(),
                scale: SCALE_FRACTION,
                dp: true,
            },
        )
        .unwrap();
        let expected = 20.0_f64.to_radians() / TAU;
        assert!(
            (lng - expected).abs() < 2.0_f64.powi(-27),
            "LNG(SPL) round-trip: got {lng}, expected {expected}"
        );
    }

    /// TC-ES-2: each component of RN and VN sits in its expected DP
    /// slot and round-trips within 1 m and 1 m/s respectively.
    #[test]
    fn tc_es_2_round_trip_vec3_values() {
        let symtab = fixture_symtab();
        let mut core = empty_core();
        let state = default_state();
        patch_into(&mut core, &symtab, &state).unwrap();

        let rn_base = symtab.get("RN").unwrap();
        for (i, expected) in state.position_m.iter().enumerate() {
            let addr = bump_dp(rn_base, i).unwrap();
            let actual = read_scaled(
                &core,
                &ScaledVar {
                    addr,
                    scale: SCALE_POSITION_M,
                    dp: true,
                },
            )
            .unwrap();
            assert!(
                (actual - expected).abs() < 1.0,
                "RN[{i}] round-trip: {actual} vs {expected}"
            );
        }

        let vn_base = symtab.get("VN").unwrap();
        for (i, expected_mps) in state.velocity_mps.iter().enumerate() {
            let addr = bump_dp(vn_base, i).unwrap();
            let actual_mpercs = read_scaled(
                &core,
                &ScaledVar {
                    addr,
                    scale: SCALE_VELOCITY_M_PER_CS,
                    dp: true,
                },
            )
            .unwrap();
            let actual_mps = actual_mpercs * 100.0;
            assert!(
                (actual_mps - expected_mps).abs() < 1.0e-3,
                "VN[{i}] round-trip: {actual_mps} m/s vs {expected_mps} m/s"
            );
        }
    }

    /// TC-ES-3: REFSMMAT cells are written into 9 consecutive DP
    /// slots, each round-tripping near 1 (identity diagonal) or 0
    /// (off-diagonal).
    #[test]
    fn tc_es_3_refsmmat_identity_layout() {
        let symtab = fixture_symtab();
        let mut core = empty_core();
        let state = default_state();
        patch_into(&mut core, &symtab, &state).unwrap();

        let base = symtab.get("REFSMMAT").unwrap();
        for row in 0..3 {
            for col in 0..3 {
                let idx = row * 3 + col;
                let addr = bump_dp(base, idx).unwrap();
                let value = read_scaled(
                    &core,
                    &ScaledVar {
                        addr,
                        scale: SCALE_FRACTION,
                        dp: true,
                    },
                )
                .unwrap();
                let expected = if row == col { 1.0 } else { 0.0 };
                // Direction-cosines are clamped one LSB inside the
                // ones-complement range, so the diagonal lands at
                // 1.0 − 2^−28.
                assert!(
                    (value - expected).abs() < 2.0_f64.powi(-26),
                    "REFSMMAT[{row}][{col}] = {value}, expected ≈ {expected}"
                );
            }
        }
    }

    /// TC-ES-4: REFSMFLG bit 13 is set in FLAGWRD3 after patching,
    /// and the other 14 bits of FLAGWRD3 are left at their pre-patch
    /// values.
    #[test]
    fn tc_es_4_refsmflg_bit_set() {
        let symtab = fixture_symtab();
        let mut core = empty_core();
        // Seed FLAGWRD3 with a non-zero pattern so the OR-merge can be
        // distinguished from a plain overwrite.
        let flag_addr = symtab.get("FLAGWRD3").unwrap();
        core.write_sp(flag_addr, 0o000_111);

        patch_into(&mut core, &symtab, &default_state()).unwrap();

        let updated = core.read_sp(flag_addr).unwrap();
        assert_eq!(
            updated & REFSMFLG_BIT_MASK,
            REFSMFLG_BIT_MASK,
            "REFSMFLG bit not set in {updated:b}"
        );
        assert_eq!(
            updated & 0o000_111,
            0o000_111,
            "non-REFSMFLG bits of FLAGWRD3 were clobbered (got {updated:o})"
        );
    }

    /// TC-ES-5: HEADSUP encodes ±1 as ones-complement (−1 = 0x7FFE).
    #[test]
    fn tc_es_5_headsup_encoding() {
        let symtab = fixture_symtab();
        let mut core = empty_core();
        let mut state = default_state();

        state.lift_up = true;
        patch_into(&mut core, &symtab, &state).unwrap();
        assert_eq!(core.read_sp(symtab.get("HEADSUP").unwrap()), Some(0x7FFE));

        state.lift_up = false;
        patch_into(&mut core, &symtab, &state).unwrap();
        assert_eq!(core.read_sp(symtab.get("HEADSUP").unwrap()), Some(1));
    }

    /// TC-ES-6: MODREG is zeroed by the patch (so V37 is accepted on
    /// the resumed AGC's P00 idle screen).
    #[test]
    fn tc_es_6_modreg_zero() {
        let symtab = fixture_symtab();
        let mut core = empty_core();
        // Pretend a previous run left MODREG = 37 (P37).
        core.write_sp(symtab.get("MODREG").unwrap(), 37);

        patch_into(&mut core, &symtab, &default_state()).unwrap();

        assert_eq!(core.read_sp(symtab.get("MODREG").unwrap()), Some(0));
    }

    /// TC-ES-ROLLC-1: write a known bank fraction into `ROLLC` via
    /// the existing `write_scaled` path, then `read_rollc_rad` recovers
    /// it (in radians) within DP LSB tolerance. The conversion is
    /// `radians = fraction_of_rev * 2π`, so a fraction of 0.25 rev =
    /// 90° = π/2 rad.
    #[test]
    fn tc_es_rollc_1_round_trip() {
        let symtab = fixture_symtab();
        let mut core = empty_core();
        let addr = symtab.get("ROLLC").unwrap();
        // Write 0.25 rev → expect 90° = π/2 rad on readback.
        let scaled = ScaledVar {
            addr,
            scale: SCALE_FRACTION,
            dp: true,
        };
        assert!(write_scaled(&mut core, &scaled, 0.25));

        let bank = read_rollc_rad(&core, &symtab).expect("rollc readback");
        let expected = std::f64::consts::FRAC_PI_2;
        assert!(
            (bank - expected).abs() < 1e-6,
            "ROLLC round-trip: expected {expected}, got {bank}"
        );

        // Negative bank (left bank) round-trips.
        assert!(write_scaled(&mut core, &scaled, -0.125));
        let bank = read_rollc_rad(&core, &symtab).unwrap();
        let expected = -0.25 * std::f64::consts::PI;
        assert!(
            (bank - expected).abs() < 1e-6,
            "ROLLC negative round-trip: expected {expected}, got {bank}"
        );
    }

    /// TC-ES-ROLLC-2: a symbol table without `ROLLC` returns `None`
    /// (catches stale assembly listings).
    #[test]
    fn tc_es_rollc_2_missing_symbol_none() {
        let symtab = Symtab::parse("Symbol Table\n");
        let core = empty_core();
        assert!(read_rollc_rad(&core, &symtab).is_none());
    }

    /// TC-ES-7: a missing symbol surfaces as `MissingSymbol`, not a
    /// silent zero write. Catches symtab drift early.
    #[test]
    fn tc_es_7_missing_symbol_errors() {
        // Empty symtab: every lookup fails.
        let symtab = Symtab::parse("Symbol Table\n");
        let mut core = empty_core();
        let err = patch_into(&mut core, &symtab, &default_state()).unwrap_err();
        match err {
            PatchError::MissingSymbol { symbol } => {
                // First symbol patched is RN.
                assert_eq!(symbol, "RN");
            }
            other => panic!("expected MissingSymbol, got {other:?}"),
        }
    }

    /// TC-ES-CMDAPMOD-1: CMDAPMOD lands at the right address with the
    /// expected 15-bit ones-complement bit pattern for `-1`, `0`, and
    /// `+1`. The AGC's P62 decision branch
    /// (`P61-P67.agc:260-265`) reads this raw word; getting the bit
    /// pattern wrong silently re-routes through the WAKEP62 path.
    #[test]
    fn tc_es_cmdapmod_round_trip() {
        let symtab = fixture_symtab();
        let addr = symtab.get("CMDAPMOD").unwrap();

        for (value, expected_raw) in [(-1_i16, 0o77776_u16), (0, 0), (1, 1)] {
            let mut core = empty_core();
            let mut state = default_state();
            state.cmdapmod = value;
            patch_into(&mut core, &symtab, &state).unwrap();
            let raw = core.read_sp(addr).unwrap();
            assert_eq!(
                raw, expected_raw,
                "CMDAPMOD={value} → expected raw {expected_raw:o}, got {raw:o}"
            );
        }
    }
}

