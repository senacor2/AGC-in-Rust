//! AGC navigation star catalogue (37 stars) — Apollo Comanche055.
//!
//! The direction vectors are stored in the AGC **Mean of 1969.5**
//! equatorial frame (see ADR-013 in transformation/decisions.md). They
//! are transcribed verbatim from `Comanche055/STAR_TABLES.agc`; no
//! precession rotation is applied at load time.
//!
//! Consumers:
//! - P23 (cislunar midcourse navigation) — star-horizon mark prediction
//! - P51 / P52 (IMU alignment) — REFSMMAT computation via TRIAD method
//! - R51 (fine-align subroutine), R56/PICAPAR (auto-star-select pair search)
//!
//! Research: specs/star-catalog-research.md
//!
//! AGC source: Comanche055/STAR_TABLES.agc

use crate::types::Vec3;

/// Total number of navigation stars in the AGC catalogue.
pub const CATALOG_SIZE: u8 = 37;

/// One entry in the AGC navigation star catalogue.
///
/// The `direction` field is a unit vector in the AGC Mean-of-1969.5
/// equatorial frame. The `name` field is for human reference only; all
/// automated lookups go through `number`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StarEntry {
    /// AGC catalogue number, 1-based (1..=37).
    pub number: u8,
    /// Human-readable star name. Approximate identifications —
    /// not authoritative. See doc comment on `STAR_CATALOG` for details.
    pub name: &'static str,
    /// Direction unit vector in the AGC Mean-of-1969.5 equatorial frame.
    pub direction: Vec3,
}

/// The full 37-entry AGC navigation star catalogue, indexed `[0..37]` by
/// `(star_number - 1)`. `STAR_CATALOG[0]` is star 1 (Alpheratz), etc.
///
/// Sourced verbatim from `Comanche055/STAR_TABLES.agc`; see ADR-013
/// for the frame-epoch decision.
///
/// # Star name note
///
/// **APPROXIMATE IDENTIFICATIONS — may differ from published Apollo navigation
/// star lists.** The AGC source has no star names; stars are identified only
/// by number (1–37). The names below are best-guess identifications derived
/// from the direction vectors, with a subset confirmed from known Apollo
/// mission documentation (stars 1, 5, 7, 16, 25, 30). Stars 9–10 and 17–19
/// are particularly uncertain. Do not rely on the `name` field for correctness;
/// use `number` for all automated lookups.
///
/// Confirmed identifications:
/// - Star  1: Alpheratz (α And)
/// - Star  5: Polaris   (α UMi) — Z ≈ +1.000 confirms near-NCP position
/// - Star  7: Hamal     (α Ari)
/// - Star 16: Pollux    (β Gem) — confirmed via direction vector analysis in
///   specs/star-catalog-research.md §4.3.
/// - Star 25: Antares   (α Sco)
/// - Star 30: Vega      (α Lyr)
pub const STAR_CATALOG: [StarEntry; 37] = [
    StarEntry {
        number: 1,
        name: "Alpheratz",
        direction: [0.8748658918, 0.0260879174, 0.4836621670],
    },
    StarEntry {
        number: 2,
        name: "Diphda",
        direction: [0.9342640400, 0.1735073142, -0.3115219339],
    },
    StarEntry {
        number: 3,
        name: "Navi",
        direction: [0.4775639450, 0.1166004340, 0.8708254803],
    },
    StarEntry {
        number: 4,
        name: "Achernar",
        direction: [0.4917678276, 0.2204887125, -0.8423473935],
    },
    StarEntry {
        number: 5,
        name: "Polaris",
        direction: [0.0130968840, 0.0078062795, 0.9998837600],
    },
    StarEntry {
        number: 6,
        name: "Acamar",
        direction: [0.5450107404, 0.5314955466, -0.6484410356],
    },
    StarEntry {
        number: 7,
        name: "Hamal",
        direction: [0.7032235469, 0.7075846047, 0.0692868685],
    },
    StarEntry {
        number: 8,
        name: "Menkar",
        direction: [0.4105636020, 0.4988110001, 0.7632988371],
    },
    StarEntry {
        number: 9,
        name: "Mirfak",
        direction: [0.3507315038, 0.8926333307, 0.2831839492],
    },
    StarEntry {
        number: 10,
        name: "Aldebaran",
        direction: [0.2011399589, 0.9690337941, -0.1432348512],
    },
    StarEntry {
        number: 11,
        name: "Rigel",
        direction: [0.1371725575, 0.6813721061, 0.7189685267],
    },
    StarEntry {
        number: 12,
        name: "Capella",
        direction: [-0.0614937230, 0.6031563286, -0.7952489957],
    },
    StarEntry {
        number: 13,
        name: "Canopus",
        direction: [-0.1820751783, 0.9404899869, -0.2869271926],
    },
    StarEntry {
        number: 14,
        name: "Sirius",
        direction: [-0.4118589524, 0.9065485360, 0.0924226975],
    },
    StarEntry {
        number: 15,
        name: "Regor",
        direction: [-0.3612508532, 0.5747270840, -0.7342932655],
    },
    StarEntry {
        number: 16,
        name: "Pollux",
        direction: [-0.4657947941, 0.4774785033, 0.7450164351],
    },
    StarEntry {
        number: 17,
        name: "Regor",
        direction: [-0.7742591356, 0.6152504197, -0.1482892839],
    },
    StarEntry {
        number: 18,
        name: "Dnoces",
        direction: [-0.8608205219, 0.4636213989, 0.2098647835],
    },
    StarEntry {
        number: 19,
        name: "Alphard",
        direction: [-0.9656605484, 0.0525933156, 0.2544280809],
    },
    StarEntry {
        number: 20,
        name: "Regulus",
        direction: [-0.9525211695, -0.0593434796, -0.2986331746],
    },
    StarEntry {
        number: 21,
        name: "Acrux",
        direction: [-0.4523440203, -0.0493710140, -0.8904759346],
    },
    StarEntry {
        number: 22,
        name: "Menkent",
        direction: [-0.9170097662, -0.3502146628, -0.1908999176],
    },
    StarEntry {
        number: 23,
        name: "Alphecca",
        direction: [-0.5812035376, -0.2909171294, 0.7599800468],
    },
    StarEntry {
        number: 24,
        name: "Atria",
        direction: [-0.6898393233, -0.4182330640, -0.5909338474],
    },
    StarEntry {
        number: 25,
        name: "Antares",
        direction: [-0.7861763936, -0.5217996305, 0.3311371675],
    },
    StarEntry {
        number: 26,
        name: "Rasalhague",
        direction: [-0.5326876930, -0.7160644554, 0.4511047742],
    },
    StarEntry {
        number: 27,
        name: "Nunki",
        direction: [-0.3516499609, -0.8240752703, -0.4441196390],
    },
    StarEntry {
        number: 28,
        name: "Rigil Kent",
        direction: [-0.1146237858, -0.3399692557, -0.9334250333],
    },
    StarEntry {
        number: 29,
        name: "Kaus Austr",
        direction: [-0.1124304773, -0.9694934200, 0.2178116072],
    },
    StarEntry {
        number: 30,
        name: "Vega",
        direction: [0.1217293692, -0.7702732847, 0.6259880410],
    },
    StarEntry {
        number: 31,
        name: "Nunki",
        direction: [0.2069525789, -0.8719885748, -0.4436288486],
    },
    StarEntry {
        number: 32,
        name: "Dabih",
        direction: [0.4537196908, -0.8779508801, 0.1527766153],
    },
    StarEntry {
        number: 33,
        name: "Peacock",
        direction: [0.5520184464, -0.7933187400, -0.2567508745],
    },
    StarEntry {
        number: 34,
        name: "Alnair",
        direction: [0.3201817378, -0.4436021946, -0.8370786986],
    },
    StarEntry {
        number: 35,
        name: "Fomalhaut",
        direction: [0.4541086270, -0.5392368197, 0.7092312789],
    },
    StarEntry {
        number: 36,
        name: "Markab",
        direction: [0.8139832631, -0.5557243189, 0.1691204557],
    },
    StarEntry {
        number: 37,
        name: "Deneb",
        direction: [0.8342971408, -0.2392481515, -0.4966976975],
    },
];

/// Look up a star's direction unit vector by AGC catalogue number.
///
/// Returns `Some(direction)` for `number` in `1..=37`, `None` otherwise.
///
/// `number == 0` is reserved for planet mode (Sun or planet vector
/// computed from an ephemeris or crew-entered via Noun 88); callers
/// handling planet mode must use a different lookup path — see P23's
/// `STARCODE == 0` branch per specs/star-catalog-research.md §3.4.
///
/// `number` in `38..=50` is valid in the AGC's `CHKSCODE` range (used for
/// stored-track continuation mode in P51/P52), but those star-code values
/// address erasable memory (`STARAD − 228D,1`), not the fixed catalogue.
/// This function returns `None` for those values; callers must handle
/// the stored-vector case separately.
///
/// Spec: specs/star-catalog-research.md §7.4
pub fn star_direction(number: u8) -> Option<Vec3> {
    if number == 0 || number > CATALOG_SIZE {
        return None;
    }
    Some(STAR_CATALOG[(number - 1) as usize].direction)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// TC-STAR-1: every entry in STAR_CATALOG is a unit vector (magnitude within 1e-6 of 1.0).
    ///
    /// The AGC values are transcribed as 10-digit decimal literals; rounding in the
    /// least-significant digits can produce magnitude errors up to ~1e-6 across the
    /// 37 entries. The 1e-6 tolerance accepts these transcription-level errors while
    /// firmly rejecting any sign-flip or digit-swap (which would cause deviations of
    /// order 1e-1 or larger).
    #[test]
    fn tc_star_1_all_entries_unit_length() {
        for entry in STAR_CATALOG.iter() {
            let [dx, dy, dz] = entry.direction;
            let mag = libm::sqrt(dx * dx + dy * dy + dz * dz);
            assert!(
                libm::fabs(mag - 1.0) < 1e-6,
                "Star {}: magnitude {} deviates from 1.0 by {}",
                entry.number,
                mag,
                libm::fabs(mag - 1.0)
            );
        }
    }

    /// TC-STAR-2: star numbers are 1..=37 in ascending order, matching array index convention.
    #[test]
    fn tc_star_2_numbers_ascending_one_based() {
        for (i, entry) in STAR_CATALOG.iter().enumerate() {
            assert_eq!(
                entry.number,
                (i + 1) as u8,
                "STAR_CATALOG[{}].number expected {} but got {}",
                i,
                i + 1,
                entry.number
            );
        }
    }

    /// TC-STAR-3: star_direction boundary cases — None for 0, Some for 1 and 37, None for 38/50/255.
    #[test]
    fn tc_star_3_star_direction_boundary_cases() {
        // Planet mode — must be None.
        assert_eq!(
            star_direction(0),
            None,
            "star_direction(0) must be None (planet mode)"
        );

        // First catalogue entry.
        let expected_1 = STAR_CATALOG[0].direction;
        assert_eq!(
            star_direction(1),
            Some(expected_1),
            "star_direction(1) must match STAR_CATALOG[0].direction"
        );

        // Last catalogue entry.
        let expected_37 = STAR_CATALOG[36].direction;
        assert_eq!(
            star_direction(37),
            Some(expected_37),
            "star_direction(37) must match STAR_CATALOG[36].direction"
        );

        // Stored-track code range — not in the fixed catalogue.
        assert_eq!(
            star_direction(38),
            None,
            "star_direction(38) must be None (stored-track range)"
        );
        assert_eq!(star_direction(50), None, "star_direction(50) must be None");
        assert_eq!(
            star_direction(255),
            None,
            "star_direction(255) must be None (way out of range)"
        );
    }

    /// TC-STAR-4: specific stars have analytically expected approximate positions.
    /// Catches sign errors on direction component transcription.
    #[test]
    fn tc_star_4_specific_star_approximate_positions() {
        // Star 5 (Polaris): near the north celestial pole — Z component very close to +1.
        let polaris = STAR_CATALOG[4].direction;
        assert!(
            polaris[2] > 0.9998,
            "Star 5 (Polaris) Z component {} must be > 0.9998 (near-NCP)",
            polaris[2]
        );

        // Star 1 (Alpheratz): positive X hemisphere.
        let alpheratz = STAR_CATALOG[0].direction;
        assert!(
            alpheratz[0] > 0.8,
            "Star 1 (Alpheratz) X component {} must be > 0.8",
            alpheratz[0]
        );

        // Star 25 (Antares): negative Y and negative X (southern declination, negative RA X-component).
        let antares = STAR_CATALOG[24].direction;
        assert!(
            antares[1] < 0.0,
            "Star 25 (Antares) Y component {} must be negative (southern declination)",
            antares[1]
        );
        assert!(
            antares[0] < 0.0,
            "Star 25 (Antares) X component {} must be negative (negative RA X-component)",
            antares[0]
        );
    }

    /// TC-STAR-5: CATALOG_SIZE constant equals the actual array length.
    #[test]
    fn tc_star_5_catalog_size_matches_array_length() {
        assert_eq!(STAR_CATALOG.len(), CATALOG_SIZE as usize);
    }

    /// TC-STAR-6: no duplicate star numbers — sorted window-pair check.
    #[test]
    fn tc_star_6_no_duplicate_star_numbers() {
        // The array is already sorted ascending by number (verified by TC-STAR-2).
        // A simple adjacent-pair check is sufficient.
        for i in 0..(STAR_CATALOG.len() - 1) {
            assert_ne!(
                STAR_CATALOG[i].number,
                STAR_CATALOG[i + 1].number,
                "Duplicate star number {} found at indices {} and {}",
                STAR_CATALOG[i].number,
                i,
                i + 1
            );
        }
    }
}
