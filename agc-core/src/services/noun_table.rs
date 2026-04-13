//! Noun table — data descriptors mapping noun codes to AGC state fields.
//!
//! AGC source: Comanche055/PINBALL_NOUN_TABLES.agc
//! Routines:   LODNNTAB, NNADTAB, NNTYPTAB, IDADDTAB, RUTMXTAB, SFINTAB, SFOUTAB
//! Pages:      268-284 (BANK 06, SETLOC PINBALL3)

/// Data source for one field of a noun definition.
///
/// Maps an AGC ECADR or computed value to a Rust accessor.
/// Each variant corresponds to a specific erasable-memory symbol in Comanche055.
/// Mixed nouns use up to three independent DataSource entries (one per component).
///
/// AGC source: NNADTAB and IDADDTAB, PINBALL_NOUN_TABLES.agc pages 186-234, 608-792.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DataSource {
    /// Not in use (noun 0, spare entries). Display is all-blank.
    NotUsed,
    /// Mission elapsed time (TIME2 erasable register, centiseconds).
    /// AGC ECADR: TIME2. N36.
    CurrentMet,
    /// Time of ignition (TIG erasable register, centiseconds).
    /// AGC ECADR: TIG. N33.
    TargetTig,
    /// Time of CSI ignition (TCSI erasable register).
    /// AGC ECADR: TCSI. N11.
    TigCsi,
    /// Target code display buffer (DSPTEM1).
    /// AGC ECADR: DSPTEM1. N30.
    TargetCode,
    /// Time to go to event / cutoff (TTOGO, centiseconds).
    /// AGC ECADR: TTOGO. N40 R1.
    TimeToGo,
    /// Velocity-to-gain magnitude (VGDISP, ft/s scaled).
    /// AGC ECADR: VGDISP. N40 R2.
    VgMagnitude,
    /// Accumulated ΔV total (DVTOTAL, ft/s scaled).
    /// AGC ECADR: DVTOTAL. N40 R3.
    DvAccumulated,
    /// Apogee altitude of target orbit (HAPOX, nautical miles).
    /// AGC ECADR: HAPOX. N44 R1.
    ApogeeAlt,
    /// Perigee altitude of target orbit (HPERX, nautical miles).
    /// AGC ECADR: HPERX. N44 R2.
    PerigeeAlt,
    /// Time of free flight (TFF, centiseconds, min:sec display).
    /// AGC ECADR: TFF. N44 R3.
    TimeFreeFlght,
    /// Inertial velocity magnitude (VMAGI, ft/s).
    /// AGC ECADR: VMAGI. N62 R1.
    InertialVelMag,
    /// Altitude rate (HDOT, ft/s).
    /// AGC ECADR: HDOT. N62 R2.
    AltRate,
    /// Altitude above pad radius (ALTI, nautical miles).
    /// AGC ECADR: ALTI. N62 R3.
    Altitude,
    /// Delta-V in local-vertical frame, X component (DELVLVC).
    /// AGC ECADR: DELVLVC. N82 R1.
    DeltaVLvcX,
    /// Delta-V in local-vertical frame, Y component (DELVLVC+2).
    /// AGC ECADR: DELVLVC+2. N82 R2.
    DeltaVLvcY,
    /// Delta-V in local-vertical frame, Z component (DELVLVC+4).
    /// AGC ECADR: DELVLVC+4. N82 R3.
    DeltaVLvcZ,
    /// VG body-frame, X component (VGBODY).
    /// AGC ECADR: VGBODY. N85 R1.
    VgBodyX,
    /// VG body-frame, Y component (VGBODY+2).
    VgBodyY,
    /// VG body-frame, Z component (VGBODY+4).
    VgBodyZ,
    /// Geodetic latitude (LAT, degrees).
    /// AGC ECADR: LAT. N43 R1.
    Latitude,
    /// Geodetic longitude (LONG, degrees).
    /// AGC ECADR: LONG. N43 R2.
    Longitude,
    /// Altitude above reference ellipsoid (ALT, nautical miles).
    /// AGC ECADR: ALT. N43 R3.
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
///
/// AGC source: NNTYPTAB, SFOUTAB, PINBALL_NOUN_TABLES.agc pages 268-284.
#[derive(Clone, Copy, Debug)]
pub struct FieldDesc {
    /// Where to read the data value from AgcState.
    pub source: DataSource,
    /// How to format the value for display.
    pub format: DisplayFormat,
    /// Scale factor: multiply the raw `f64` AgcState value by this to obtain
    /// the integer displayed on the DSKY (before digit extraction).
    ///
    /// Units depend on DisplayFormat:
    ///   Decimal/Velocity: raw ft/s or m/s → integer × 0.1 or × 1
    ///   Position4Nm: raw metres → displayed nautical miles × 10 (XXXX.X)
    ///   DpDegrees360: raw radians → displayed degrees × 100 (XXX.XX)
    ///   Time/MinSec: raw centiseconds → unscaled (format_time handles internally)
    ///   Blank: 0.0
    ///
    /// TODO: cross-check against SFOUTAB scaling in PINBALL_NOUN_TABLES.agc
    pub scale: f64,
}

/// Complete definition of one noun.
///
/// AGC source: NNADTAB, NNTYPTAB, IDADDTAB. PINBALL_NOUN_TABLES.agc pages 270-284.
#[derive(Clone, Copy, Debug)]
pub struct NounDef {
    /// Noun number (0–99).
    pub noun: u8,
    /// True if this noun cannot be loaded by load verbs (V21–V25).
    ///
    /// Encodes NNTYPTAB bit4 (BIT5 in AGC 1-based) = no-load flag.
    /// AGC source: PINBALL_NOUN_TABLES.agc page 268.
    pub no_load: bool,
    /// R1 field descriptor.
    pub r1: FieldDesc,
    /// R2 field descriptor.
    pub r2: FieldDesc,
    /// R3 field descriptor.
    pub r3: FieldDesc,
}

/// Number of noun definitions in the M5 table.
const NOUN_TABLE_LEN: usize = 11;

/// Blank field descriptor used for unused noun positions.
const BLANK_FIELD: FieldDesc = FieldDesc {
    source: DataSource::NotUsed,
    format: DisplayFormat::Blank,
    scale: 0.0,
};

/// Static noun definitions for M5.
///
/// Ordered by noun number for O(N) linear scan via `lookup`.
///
/// AGC source: NNADTAB, NNTYPTAB, IDADDTAB, RUTMXTAB.
/// PINBALL_NOUN_TABLES.agc pages 270-284.
///
/// Memory cost: 11 × size_of::<NounDef>() ≈ 11 × 64 = 704 bytes (estimated).
/// This is fixed-size ROM data; acceptable for the Cortex-M4F target.
pub const NOUN_TABLE: [NounDef; NOUN_TABLE_LEN] = [
    // N00: Not in use.
    // NNADTAB[0] = OCT 00000, NNTYPTAB[0] = OCT 00000.
    // AGC source: PINBALL_NOUN_TABLES.agc page 270, 275.
    NounDef {
        noun: 0,
        no_load: false,
        r1: BLANK_FIELD,
        r2: BLANK_FIELD,
        r3: BLANK_FIELD,
    },
    // N11: TIG of CSI (hrs, min, sec).
    // NNADTAB[11] = ECADR TCSI, NNTYPTAB[11] = OCT 24400 (3-comp HMS, dec only).
    // AGC source: PINBALL_NOUN_TABLES.agc pages 199, 381.
    NounDef {
        noun: 11,
        no_load: true,
        r1: FieldDesc {
            source: DataSource::TigCsi,
            format: DisplayFormat::Time,
            scale: 1.0,
        },
        r2: FieldDesc {
            source: DataSource::TigCsi,
            format: DisplayFormat::Time,
            scale: 1.0,
        },
        r3: FieldDesc {
            source: DataSource::TigCsi,
            format: DisplayFormat::Time,
            scale: 1.0,
        },
    },
    // N30: Target codes (DSPTEM1).
    // NNADTAB[30] = ECADR DSPTEM1, NNTYPTAB[30] = OCT 04140 (3-comp whole).
    // AGC source: PINBALL_NOUN_TABLES.agc pages 222, 400.
    NounDef {
        noun: 30,
        no_load: false,
        r1: FieldDesc {
            source: DataSource::TargetCode,
            format: DisplayFormat::Decimal,
            scale: 1.0,
        },
        r2: FieldDesc {
            source: DataSource::TargetCode,
            format: DisplayFormat::Decimal,
            scale: 1.0,
        },
        r3: FieldDesc {
            source: DataSource::TargetCode,
            format: DisplayFormat::Decimal,
            scale: 1.0,
        },
    },
    // N33: Time of ignition TIG (hrs, min, sec).
    // NNADTAB[33] = ECADR TIG, NNTYPTAB[33] = OCT 24400.
    // AGC source: PINBALL_NOUN_TABLES.agc pages 224, 403.
    NounDef {
        noun: 33,
        no_load: true,
        r1: FieldDesc {
            source: DataSource::TargetTig,
            format: DisplayFormat::Time,
            scale: 1.0,
        },
        r2: FieldDesc {
            source: DataSource::TargetTig,
            format: DisplayFormat::Time,
            scale: 1.0,
        },
        r3: FieldDesc {
            source: DataSource::TargetTig,
            format: DisplayFormat::Time,
            scale: 1.0,
        },
    },
    // N36: AGC clock / MET (hrs, min, sec).
    // NNADTAB[36] = ECADR TIME2, NNTYPTAB[36] = OCT 24400 (3-comp HMS, dec only).
    // AGC source: PINBALL_NOUN_TABLES.agc pages 227, 406.
    NounDef {
        noun: 36,
        no_load: true,
        r1: FieldDesc {
            source: DataSource::CurrentMet,
            format: DisplayFormat::Time,
            scale: 1.0,
        },
        r2: FieldDesc {
            source: DataSource::CurrentMet,
            format: DisplayFormat::Time,
            scale: 1.0,
        },
        r3: FieldDesc {
            source: DataSource::CurrentMet,
            format: DisplayFormat::Time,
            scale: 1.0,
        },
    },
    // N40: TGO / VG / ΔV accumulated (M/S, VEL3, VEL3). Mixed noun.
    // NNADTAB[40] = OCT 64000, NNTYPTAB[40] = OCT 24500, RUTMXTAB[40] = OCT 16351.
    // AGC source: PINBALL_NOUN_TABLES.agc pages 234, 414, 797.
    // TODO: cross-check against SFOUTAB scaling in PINBALL_NOUN_TABLES.agc
    NounDef {
        noun: 40,
        no_load: true,
        r1: FieldDesc {
            source: DataSource::TimeToGo,
            format: DisplayFormat::MinSec,
            scale: 1.0,
        },
        r2: FieldDesc {
            source: DataSource::VgMagnitude,
            format: DisplayFormat::Velocity3Fps,
            scale: 128.0, // VELOCITY3: 2^7 = 128, ft/s × 2^(-7)
        },
        r3: FieldDesc {
            source: DataSource::DvAccumulated,
            format: DisplayFormat::Velocity3Fps,
            scale: 128.0, // VELOCITY3: 2^7 = 128
        },
    },
    // N43: Latitude, longitude, altitude (DPDEG360, DPDEG360, POS4). Mixed noun.
    // NNADTAB[43] = OCT 24011, NNTYPTAB[43] = OCT 20204, RUTMXTAB[43] = OCT 16512.
    // AGC source: PINBALL_NOUN_TABLES.agc pages 243, 419, 800.
    // TODO: cross-check against SFOUTAB scaling in PINBALL_NOUN_TABLES.agc
    NounDef {
        noun: 43,
        no_load: true,
        r1: FieldDesc {
            source: DataSource::Latitude,
            format: DisplayFormat::DpDegrees360,
            scale: 5729.578, // radians → degrees × 100 = 360/(2π) × 100
        },
        r2: FieldDesc {
            source: DataSource::Longitude,
            format: DisplayFormat::DpDegrees360,
            scale: 5729.578,
        },
        r3: FieldDesc {
            source: DataSource::AltitudeGeo,
            format: DisplayFormat::Position4Nm,
            scale: 8.0, // POS4: 2^3 = 8, naut mi × 2^(-3)
        },
    },
    // N44: Apogee, perigee, TFF (POS4, POS4, M/S). Mixed noun.
    // NNADTAB[44] = OCT 64014, NNTYPTAB[44] = OCT 00410, RUTMXTAB[44] = OCT 22347.
    // AGC source: PINBALL_NOUN_TABLES.agc pages 245, 422, 801.
    // TODO: cross-check against SFOUTAB scaling in PINBALL_NOUN_TABLES.agc
    NounDef {
        noun: 44,
        no_load: true,
        r1: FieldDesc {
            source: DataSource::ApogeeAlt,
            format: DisplayFormat::Position4Nm,
            scale: 8.0, // POS4: 2^3 = 8
        },
        r2: FieldDesc {
            source: DataSource::PerigeeAlt,
            format: DisplayFormat::Position4Nm,
            scale: 8.0,
        },
        r3: FieldDesc {
            source: DataSource::TimeFreeFlght,
            format: DisplayFormat::MinSec,
            scale: 1.0,
        },
    },
    // N62: Inertial velocity mag, alt rate, altitude (VEL2, VEL2, POS4). Mixed noun.
    // NNADTAB[62] = OCT 24102, NNTYPTAB[62] = OCT 20451, RUTMXTAB[62] = OCT 16512.
    // AGC source: PINBALL_NOUN_TABLES.agc pages 288, 456, 819.
    // TODO: cross-check against SFOUTAB scaling in PINBALL_NOUN_TABLES.agc
    NounDef {
        noun: 62,
        no_load: true,
        r1: FieldDesc {
            source: DataSource::InertialVelMag,
            format: DisplayFormat::Velocity2Fps,
            scale: 16384.0, // VELOCITY2: 2^14 = 16384, ft/s × 2^(-14)
        },
        r2: FieldDesc {
            source: DataSource::AltRate,
            format: DisplayFormat::Velocity2Fps,
            scale: 16384.0,
        },
        r3: FieldDesc {
            source: DataSource::Altitude,
            format: DisplayFormat::Position4Nm,
            scale: 8.0,
        },
    },
    // N82: Delta-V LV frame (VEL3, VEL3, VEL3). Mixed noun.
    // NNADTAB[82] = OCT 24176, NNTYPTAB[82] = OCT 24512, RUTMXTAB[82] = OCT 16347.
    // AGC source: PINBALL_NOUN_TABLES.agc pages 338, 488, 840.
    // TODO: cross-check against SFOUTAB scaling in PINBALL_NOUN_TABLES.agc
    NounDef {
        noun: 82,
        no_load: false,
        r1: FieldDesc {
            source: DataSource::DeltaVLvcX,
            format: DisplayFormat::Velocity3Fps,
            scale: 128.0,
        },
        r2: FieldDesc {
            source: DataSource::DeltaVLvcY,
            format: DisplayFormat::Velocity3Fps,
            scale: 128.0,
        },
        r3: FieldDesc {
            source: DataSource::DeltaVLvcZ,
            format: DisplayFormat::Velocity3Fps,
            scale: 128.0,
        },
    },
    // N85: VG body frame (VEL3, VEL3, VEL3). Mixed noun.
    // NNADTAB[85] = OCT 24207, NNTYPTAB[85] = OCT 24512, RUTMXTAB[85] = OCT 16347.
    // AGC source: PINBALL_NOUN_TABLES.agc pages 341, 494, 843.
    // TODO: cross-check against SFOUTAB scaling in PINBALL_NOUN_TABLES.agc
    NounDef {
        noun: 85,
        no_load: false,
        r1: FieldDesc {
            source: DataSource::VgBodyX,
            format: DisplayFormat::Velocity3Fps,
            scale: 128.0,
        },
        r2: FieldDesc {
            source: DataSource::VgBodyY,
            format: DisplayFormat::Velocity3Fps,
            scale: 128.0,
        },
        r3: FieldDesc {
            source: DataSource::VgBodyZ,
            format: DisplayFormat::Velocity3Fps,
            scale: 128.0,
        },
    },
];

/// Look up a noun definition by noun number.
///
/// Performs a linear scan of NOUN_TABLE (O(11) = O(1) in practice).
/// Linear scan is chosen over index lookup because the noun numbers are
/// sparse (not contiguous), matching the AGC's indexed table with gaps.
///
/// Returns `None` for noun numbers not present in the M5 table.
/// The caller (verb dispatch) treats `None` as an OPERATOR ERROR.
///
/// AGC source: LODNNTAB (PINBALL_NOUN_TABLES.agc page 270) performs
/// indexed table lookup; the Rust linear scan is equivalent for 11 entries.
pub fn lookup(noun: u8) -> Option<&'static NounDef> {
    NOUN_TABLE.iter().find(|def| def.noun == noun)
}

// ── tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-NOUN-1: N36 returns MET field descriptor.
    ///
    /// AGC source: NNADTAB[36] = ECADR TIME2, NNTYPTAB[36] HMS format.
    #[test]
    fn n36_returns_met_descriptor() {
        let def = lookup(36);
        assert!(def.is_some());
        let def = def.unwrap();
        assert_eq!(def.noun, 36);
        assert_eq!(def.r1.source, DataSource::CurrentMet);
        assert_eq!(def.r1.format, DisplayFormat::Time);
        assert_eq!(def.r2.source, DataSource::CurrentMet);
        assert_eq!(def.r2.format, DisplayFormat::Time);
        assert_eq!(def.r3.source, DataSource::CurrentMet);
        assert_eq!(def.r3.format, DisplayFormat::Time);
    }

    /// TC-NOUN-2: Unknown nouns return None.
    ///
    /// AGC source: LODNNTAB handles unmapped nouns with ALARM / CHARALRM.
    #[test]
    fn unknown_nouns_return_none() {
        assert!(lookup(99).is_none());
        assert!(lookup(10).is_none());
        assert!(lookup(200).is_none());
        assert!(lookup(1).is_none());
    }

    /// TC-NOUN-3: N40 has 3 populated burn fields.
    ///
    /// AGC source: IDADDTAB N40: TTOGO, VGDISP, DVTOTAL. RUTMXTAB[40] M/S+VEL3+VEL3.
    #[test]
    fn n40_has_three_burn_fields() {
        let n40 = lookup(40).unwrap();
        assert_ne!(n40.r1.source, DataSource::NotUsed);
        assert_ne!(n40.r2.source, DataSource::NotUsed);
        assert_ne!(n40.r3.source, DataSource::NotUsed);
        assert_eq!(n40.r1.format, DisplayFormat::MinSec);
        assert_eq!(n40.r2.format, DisplayFormat::Velocity3Fps);
        assert_eq!(n40.r3.format, DisplayFormat::Velocity3Fps);
    }

    /// TC-NOUN-4: Noun lookup is const-compatible and table is valid.
    ///
    /// AGC source: NOUN_TABLE is ROM data; must compile as const.
    #[test]
    fn noun_lookup_const_compatible() {
        const _: &NounDef = &NOUN_TABLE[0];
        assert_eq!(lookup(0).map(|n| n.noun), Some(0));
        assert_eq!(lookup(36).map(|n| n.noun), Some(36));
    }

    /// TC-NOUN-5: N00 is all-blank.
    #[test]
    fn n00_all_blank() {
        let n00 = lookup(0).unwrap();
        assert_eq!(n00.r1.source, DataSource::NotUsed);
        assert_eq!(n00.r1.format, DisplayFormat::Blank);
        assert_eq!(n00.r2.source, DataSource::NotUsed);
        assert_eq!(n00.r3.source, DataSource::NotUsed);
    }

    /// TC-NOUN-6: N82 has three VEL3 delta-V LVC components.
    #[test]
    fn n82_has_three_deltav_fields() {
        let n82 = lookup(82).unwrap();
        assert_eq!(n82.r1.source, DataSource::DeltaVLvcX);
        assert_eq!(n82.r2.source, DataSource::DeltaVLvcY);
        assert_eq!(n82.r3.source, DataSource::DeltaVLvcZ);
        assert_eq!(n82.r1.format, DisplayFormat::Velocity3Fps);
    }
}
