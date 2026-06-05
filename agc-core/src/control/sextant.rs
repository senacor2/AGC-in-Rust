//! Sextant geometry and MARK keystroke pipeline (#57).
//!
//! Converts the AGC's CM optics shaft/trunnion CDU angles into a body-frame
//! unit vector, and provides the dispatch helper that the simulator (or any
//! future real-hardware adapter) can call when the optics MARK button is
//! pressed.
//!
//! ## Geometry convention
//!
//! The AGC Command Module sextant has two CDU channels:
//!
//! - **Trunnion (TA)** — tilt of the LOS away from the navigation-base
//!   optical axis. `TA = 0` means the LOS lies along the optical axis;
//!   `TA = 90°` means the LOS is perpendicular to it.
//! - **Shaft (SA)** — rotation of the trunnion deflection around the
//!   optical axis. `SA = 0` puts the trunnion swing into one body plane;
//!   `SA = 90°` puts it into the orthogonal one.
//!
//! For the Rust port we adopt a clean spherical-coordinate convention with
//! the optical axis along the body `+Z` axis. (Comanche055 routes the
//! sensor mount through a fixed offset matrix; we treat that offset as
//! the identity because the closed-loop tests only need a consistent
//! forward/inverse pair, not bit-for-bit AGC fidelity.)
//!
//! ```text
//! los_body = [ sin(TA) · cos(SA),
//!              sin(TA) · sin(SA),
//!              cos(TA) ]
//! ```
//!
//! - `TA = 0`                    → `[0, 0, 1]` (along +Z, the optical axis)
//! - `TA = 90°`, `SA = 0`        → `[1, 0, 0]` (along +X)
//! - `TA = 90°`, `SA = 90°`      → `[0, 1, 0]` (along +Y)
//!
//! The inverse is `TA = acos(los_body[2])`, `SA = atan2(los_body[1], los_body[0])`,
//! consistent with the standard spherical-to-Cartesian mapping.
//!
//! ## MARK pipeline
//!
//! When the crew presses MARK during a star or landmark sighting:
//!
//! 1. The hardware latches the current `shaft_angle` and `trunnion_angle`
//!    CDU values and asserts `mark_pressed`.
//! 2. The sextant interrupt handler ([`consume_optics_mark`]) reads the
//!    CDU registers, converts them to a body-frame unit vector via
//!    [`los_body_from_cdu`], and rotates to the platform frame.
//! 3. The platform-frame LOS is buffered or dispatched to the active
//!    program (P51/P52 for stars, P22 for landmarks).
//! 4. The handler clears `mark_pressed` so the next press is detected
//!    as a fresh edge.
//!
//! ## Attitude
//!
//! For the body → inertial rotation, [`consume_optics_mark`] currently
//! assumes the body frame is identical to the inertial frame (identity
//! attitude). That matches the convention every existing optics test uses
//! today (`star_los_in_platform` in `agc-sim/src/sensors.rs` passes the
//! catalog inertial direction directly into REFSMMAT) and is sufficient
//! to demonstrate the closed-loop CDU pipeline. Threading a real attitude
//! quaternion is a follow-on task; the function returns the platform-frame
//! LOS so a caller in a non-identity-attitude scenario can pre-rotate
//! before the call.

use crate::hal::{AgcHardware, Optics};
use crate::math::linalg::mxv;
use crate::types::{CduAngle, Mat3x3, Vec3};

/// Body-frame line-of-sight unit vector that the sextant is pointing at,
/// given its current shaft and trunnion CDU angles.
///
/// See the module-level docs for the geometry convention.
pub fn los_body_from_cdu(shaft: CduAngle, trunnion: CduAngle) -> Vec3 {
    let sa = shaft.to_radians();
    let ta = trunnion.to_radians();
    let sin_ta = libm::sin(ta);
    let cos_ta = libm::cos(ta);
    let sin_sa = libm::sin(sa);
    let cos_sa = libm::cos(sa);
    [sin_ta * cos_sa, sin_ta * sin_sa, cos_ta]
}

/// CDU shaft/trunnion encoding for a desired body-frame line-of-sight unit
/// vector. Right-inverse of [`los_body_from_cdu`] up to the CDU's 1 LSB
/// (≈ 0.0055°) quantisation.
///
/// The returned `trunnion` lies in `[0, π]` (positive tilt only — the
/// sextant cannot mechanically aim "behind" the optical axis). The returned
/// `shaft` lies in `[-π, π)`.
pub fn cdu_from_los_body(los_body: Vec3) -> (CduAngle, CduAngle) {
    // Trunnion: angle between LOS and optical axis (+Z).
    let z = los_body[2].clamp(-1.0, 1.0);
    let trunnion = CduAngle::from_radians(libm::acos(z));
    // Shaft: azimuth of the in-plane projection (X-Y).
    let shaft = CduAngle::from_radians(libm::atan2(los_body[1], los_body[0]));
    (shaft, trunnion)
}

/// Result of consuming a pending optics MARK.
///
/// The platform-frame LOS lets the caller dispatch the mark to whichever
/// program is active (P51/P52 for stars, P22 for landmarks). The shaft and
/// trunnion angles are returned so loggers / regression captures can pin
/// the CDU values without re-reading the HAL after the consume step has
/// already cleared `mark_pressed`.
#[derive(Clone, Copy, Debug)]
pub struct OpticsMark {
    /// Shaft CDU latched at the moment MARK was pressed.
    pub shaft: CduAngle,
    /// Trunnion CDU latched at the moment MARK was pressed.
    pub trunnion: CduAngle,
    /// Sextant LOS unit vector in the body frame.
    pub los_body: Vec3,
    /// Sextant LOS unit vector rotated to the platform (stable-member) frame
    /// under the identity-attitude assumption (`body ≡ inertial`).
    pub los_platform: Vec3,
}

/// Check the optics MARK input and, if asserted, convert the latched CDU
/// angles into a platform-frame LOS.
///
/// Returns `Some(mark)` on a fresh MARK edge — the input is consumed
/// (`mark_pressed` cleared) so subsequent calls won't re-fire on the same
/// keystroke. Returns `None` when no MARK is pending.
///
/// `refsmmat` is the current inertial→platform rotation (typically read
/// from `state.refsmmat`). Body-frame LOS is rotated directly by REFSMMAT
/// under the identity-attitude assumption documented in the module header.
pub fn consume_optics_mark<H: AgcHardware>(hw: &mut H, refsmmat: &Mat3x3) -> Option<OpticsMark> {
    if !hw.optics().mark_pressed() {
        return None;
    }
    let shaft = hw.optics().shaft_angle();
    let trunnion = hw.optics().trunnion_angle();
    let los_body = los_body_from_cdu(shaft, trunnion);
    // Identity-attitude assumption: body ≡ inertial, so platform = REFSMMAT·body.
    let los_platform = mxv(*refsmmat, los_body);
    clear_mark(hw);
    Some(OpticsMark {
        shaft,
        trunnion,
        los_body,
        los_platform,
    })
}

/// Clear the MARK edge by driving optics with a no-op `drive()` call after
/// the consumer has noted the press. Hardware adapters can override the
/// behaviour by intercepting `drive`; the default sim adapter just resets
/// its own bool when this function is called via [`consume_optics_mark`].
///
/// Kept as a free function so AGC-side users (e.g. the foreground executor)
/// don't have to know which `Optics` impl they're talking to.
fn clear_mark<H: AgcHardware>(hw: &mut H) {
    hw.optics().clear_mark();
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    fn rad_to_cdu(r: f64) -> CduAngle {
        CduAngle::from_radians(r)
    }

    /// TC-SXT-1: forward conversion at the three principal axes.
    ///
    /// - TA = 0  → +Z
    /// - TA = π/2, SA = 0  → +X
    /// - TA = π/2, SA = π/2 → +Y
    #[test]
    fn tc_sxt_1_principal_axes() {
        let los_pz = los_body_from_cdu(rad_to_cdu(0.0), rad_to_cdu(0.0));
        assert!(
            libm::fabs(los_pz[2] - 1.0) < 1e-3 && libm::fabs(los_pz[0]) < 1e-3,
            "TC-SXT-1 (TA=0): expected +Z, got {los_pz:?}"
        );
        let los_px = los_body_from_cdu(rad_to_cdu(0.0), rad_to_cdu(PI / 2.0));
        assert!(
            libm::fabs(los_px[0] - 1.0) < 1e-3 && libm::fabs(los_px[2]) < 1e-3,
            "TC-SXT-1 (SA=0,TA=π/2): expected +X, got {los_px:?}"
        );
        let los_py = los_body_from_cdu(rad_to_cdu(PI / 2.0), rad_to_cdu(PI / 2.0));
        assert!(
            libm::fabs(los_py[1] - 1.0) < 1e-3 && libm::fabs(los_py[2]) < 1e-3,
            "TC-SXT-1 (SA=π/2,TA=π/2): expected +Y, got {los_py:?}"
        );
    }

    /// TC-SXT-2: forward output is a unit vector at arbitrary CDU values.
    #[test]
    fn tc_sxt_2_forward_is_unit_vector() {
        for (sa_deg, ta_deg) in [(0.0, 30.0), (45.0, 60.0), (-90.0, 45.0), (135.0, 10.0)] {
            let los = los_body_from_cdu(
                rad_to_cdu(sa_deg * PI / 180.0),
                rad_to_cdu(ta_deg * PI / 180.0),
            );
            let mag = libm::sqrt(los[0] * los[0] + los[1] * los[1] + los[2] * los[2]);
            assert!(
                libm::fabs(mag - 1.0) < 1e-3,
                "TC-SXT-2 (SA={sa_deg}°, TA={ta_deg}°): |los| = {mag}"
            );
        }
    }

    /// TC-SXT-3: forward / inverse roundtrip is exact within CDU quantisation.
    ///
    /// 1 LSB ≈ 0.0055°. Picking inputs that align with CDU multiples lets us
    /// assert sub-degree accuracy without hitting quantisation drift.
    #[test]
    fn tc_sxt_3_cdu_to_los_to_cdu_roundtrip() {
        // Choose inputs that align with the 1-LSB CDU grid so the roundtrip
        // is limited only by the f64 trig accuracy, not the quantisation
        // imposed by the next encoding step.
        for (sa_count, ta_count) in [(0_i16, 5000), (4096, 1000), (-8192, 16000), (16384, 8000)] {
            let sa_in = CduAngle(sa_count);
            let ta_in = CduAngle(ta_count);
            let los = los_body_from_cdu(sa_in, ta_in);
            let (sa_out, ta_out) = cdu_from_los_body(los);
            // Tolerate ±2 LSB to absorb the cumulative round() drift between
            // the two encode→decode transitions.
            assert!(
                (sa_out.0 - sa_in.0).abs() <= 2,
                "TC-SXT-3 SA roundtrip: in {sa_in:?}, out {sa_out:?}"
            );
            assert!(
                (ta_out.0 - ta_in.0).abs() <= 2,
                "TC-SXT-3 TA roundtrip: in {ta_in:?}, out {ta_out:?}"
            );
        }
    }

    /// TC-SXT-4: arbitrary inertial LOS round-trips through encode → decode
    /// (within CDU quantisation), and the resulting body LOS is unit-length.
    #[test]
    fn tc_sxt_4_los_body_roundtrip() {
        // Pick a few unit vectors in the +Z hemisphere (TA in [0, π]).
        let inputs: [Vec3; 4] = [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.6, 0.8, 0.0],
            // Mix all three.
            {
                let v: Vec3 = [0.3, -0.4, 0.866];
                let n = libm::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                [v[0] / n, v[1] / n, v[2] / n]
            },
        ];
        for los_in in inputs {
            let (sa, ta) = cdu_from_los_body(los_in);
            let los_out = los_body_from_cdu(sa, ta);
            let mag = libm::sqrt(los_out[0].powi(2) + los_out[1].powi(2) + los_out[2].powi(2));
            assert!(
                libm::fabs(mag - 1.0) < 1e-3,
                "TC-SXT-4: |los_out| = {mag} for input {los_in:?}"
            );
            // Inner product close to 1 means the angle is < 1°.
            let dot = los_in[0] * los_out[0] + los_in[1] * los_out[1] + los_in[2] * los_out[2];
            assert!(
                dot > 0.9999,
                "TC-SXT-4: roundtrip angle too large for input {los_in:?}: dot = {dot}"
            );
        }
    }
}
