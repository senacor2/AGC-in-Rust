// SPDX-License-Identifier: GPL-3.0-or-later
//! Interactive sextant for `dsky_sim` (#176; concept in `docs/sextant_ui_concept.md`, #109).
//!
//! Models the CM optics star line as a body-frame line of sight driven by the
//! shaft/trunnion CDU angles ([`SimOptics`]). The crew slews the line onto a
//! star selected via `V25 N70`, then presses MARK; the latched angles become a
//! platform-frame sighting that feeds the P51/P52 keystroke MARK pipeline
//! (`pxx_mark_align`, #175/#177).
//!
//! Frame convention follows the rest of the sim: an **identity attitude**
//! (body ≡ inertial), so the reticle centre `los_body_from_cdu(shaft, trunnion)`
//! and a catalogue star direction are compared directly, and a mark's
//! platform-frame LOS is `REFSMMAT · los_body_from_cdu(...)` — exactly what
//! [`consume_optics_mark`] computes (matching the scenario `OpticsCduMark` path).

use agc_core::control::sextant::{consume_optics_mark, los_body_from_cdu};
use agc_core::navigation::star_catalog::{CATALOG_SIZE, STAR_CATALOG};
use agc_core::programs::p51_p52::pxx_mark_align;
use agc_core::types::{CduAngle, Mat3x3, Vec3};
use agc_core::AgcState;

use crate::hardware::SimHardware;

/// Half-angle of the rendered sextant field of view, degrees. Wider than a real
/// SXT so the crew can find a star from off-centre in a demo.
pub const FOV_HALF_DEG: f64 = 15.0;

/// A mark within this angular offset of the star is flagged "on mark" (display
/// only — marks outside it are still accepted, carrying the pointing error into
/// the alignment; see concept §4).
pub const MARK_TOL_DEG: f64 = 0.5;

/// Coarse slew step (~2°) and fine slew step (~0.2°) in CDU counts
/// (16384 counts = 90°, i.e. ≈182 counts/°).
const COARSE_COUNTS: i16 = 364;
const FINE_COUNTS: i16 = 36;

/// Slew axis for [`MarkSession::slew`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlewAxis {
    ShaftMinus,
    ShaftPlus,
    TrunnionMinus,
    TrunnionPlus,
}

/// A buffered first sighting of a star pair.
#[derive(Clone, Copy)]
struct Sighting {
    inertial: Vec3,
    platform: Vec3,
}

/// The rendered target star within the sextant field.
pub struct TargetStar {
    pub id: u8,
    pub name: &'static str,
    /// Screen offset normalised to the FOV: `(x, y)` in `[-1, 1]` when in field
    /// (may exceed ±1 when the star is outside the field — draw an edge arrow).
    pub screen: (f64, f64),
    pub in_fov: bool,
    pub on_mark: bool,
    pub offset_deg: f64,
}

/// Immutable per-frame view for the renderer.
pub struct SextantFrame {
    pub shaft_deg: f64,
    pub trunnion_deg: f64,
    pub target: Option<TargetStar>,
    /// Marks taken toward the current pair (0, 1).
    pub pair_marks: u8,
    pub status: String,
}

/// Interactive sextant state owned by `dsky_sim`.
pub struct MarkSession {
    /// Truth inertial→platform rotation used to synthesise the sighting the
    /// optics measures. Fixed default per concept §7.4.
    pub truth_refsmmat: Mat3x3,
    first: Option<Sighting>,
    status: String,
}

impl MarkSession {
    /// New session with a given truth REFSMMAT (identity = platform ≡ inertial).
    pub fn new(truth_refsmmat: Mat3x3) -> Self {
        Self {
            truth_refsmmat,
            first: None,
            status: String::from("Select a star: V25 N70 E <code> E, slew onto it, then MARK"),
        }
    }

    /// Number of marks buffered toward the current pair (0 or 1).
    pub fn pair_marks(&self) -> u8 {
        u8::from(self.first.is_some())
    }

    /// The latest status line (mark progress / prompts).
    pub fn status(&self) -> &str {
        &self.status
    }

    /// Nudge the optics shaft or trunnion by a coarse (or `fine`) step.
    pub fn slew(&self, hw: &mut SimHardware, axis: SlewAxis, fine: bool) {
        let step = if fine { FINE_COUNTS } else { COARSE_COUNTS };
        match axis {
            SlewAxis::ShaftMinus => hw.optics.shaft = CduAngle(hw.optics.shaft.0.wrapping_sub(step)),
            SlewAxis::ShaftPlus => hw.optics.shaft = CduAngle(hw.optics.shaft.0.wrapping_add(step)),
            SlewAxis::TrunnionMinus => {
                hw.optics.trunnion = CduAngle(hw.optics.trunnion.0.wrapping_sub(step))
            }
            SlewAxis::TrunnionPlus => {
                hw.optics.trunnion = CduAngle(hw.optics.trunnion.0.wrapping_add(step))
            }
        }
    }

    /// Build the per-frame render view: optics angles + the selected star's
    /// position in the field (if a star code is entered).
    pub fn frame(&self, state: &AgcState, hw: &SimHardware) -> SextantFrame {
        let shaft = hw.optics.shaft;
        let trunnion = hw.optics.trunnion;
        let target = state
            .vn
            .crew_star_code
            .filter(|&id| (1..=CATALOG_SIZE).contains(&id))
            .map(|id| {
                let entry = &STAR_CATALOG[(id - 1) as usize];
                let p = los_body_from_cdu(shaft, trunnion); // reticle centre
                let (screen, offset_deg, in_fov) = project(p, entry.direction);
                TargetStar {
                    id,
                    name: entry.name,
                    screen,
                    in_fov,
                    on_mark: offset_deg <= MARK_TOL_DEG,
                    offset_deg,
                }
            });
        SextantFrame {
            shaft_deg: shaft.to_degrees(),
            trunnion_deg: trunnion.to_degrees(),
            target,
            pair_marks: self.pair_marks(),
            status: self.status.clone(),
        }
    }

    /// Register a MARK for the currently entered star at the current optics
    /// angles. Consumes `vn.crew_star_code`; on the second mark of a pair,
    /// dispatches `pxx_mark_align` (P51/P52 by major mode) and updates status.
    pub fn mark(&mut self, state: &mut AgcState, hw: &mut SimHardware) {
        let Some(star_id) = state.vn.crew_star_code else {
            self.status = String::from("MARK ignored — no star selected (V25 N70 E <code> E first)");
            return;
        };
        if !(1..=CATALOG_SIZE).contains(&star_id) {
            self.status = format!("MARK ignored — star code {star_id} out of range 1..=37");
            return;
        }
        let star_inertial = STAR_CATALOG[(star_id - 1) as usize].direction;

        // Latch the current optics angles and consume the mark → platform LOS.
        hw.optics.press_mark(hw.optics.shaft, hw.optics.trunnion);
        let mark = consume_optics_mark(hw, &self.truth_refsmmat)
            .expect("press_mark just fired, so consume_optics_mark must yield a mark");

        // Consume the crew's selection.
        state.vn.crew_star_code = None;
        let name = STAR_CATALOG[(star_id - 1) as usize].name;

        match self.first.take() {
            None => {
                self.first = Some(Sighting {
                    inertial: star_inertial,
                    platform: mark.los_platform,
                });
                self.status = format!("MARK 1 of 2 — {name} (#{star_id}) buffered; select the next star");
            }
            Some(first) => {
                let before = state.imu_alignment_state;
                pxx_mark_align(
                    state,
                    first.inertial,
                    star_inertial,
                    first.platform,
                    mark.los_platform,
                );
                self.status = if state.alarm.code() != 0 {
                    format!(
                        "MARK 2 of 2 — alarm {:04o} (stars too close?); pair discarded",
                        state.alarm.code()
                    )
                } else {
                    format!(
                        "MARK 2 of 2 — {name} (#{star_id}): {before:?} → {:?}",
                        state.imu_alignment_state
                    )
                };
            }
        }
    }
}

/// Project a target direction `s` onto the reticle centred on `p` (both unit
/// vectors, identity-attitude frame). Returns `((x, y)` normalised to
/// [`FOV_HALF_DEG`], angular offset in degrees, in-field flag).
fn project(p: Vec3, s: Vec3) -> ((f64, f64), f64, bool) {
    let dot = (p[0] * s[0] + p[1] * s[1] + p[2] * s[2]).clamp(-1.0, 1.0);
    let offset_deg = dot.acos().to_degrees();

    // Screen basis perpendicular to p: right = z × p (fallback x × p near poles).
    let mut right = cross([0.0, 0.0, 1.0], p);
    if norm(right) < 1e-6 {
        right = cross([1.0, 0.0, 0.0], p);
    }
    let right = unit(right);
    let up = unit(cross(p, right));

    // Angular offsets along each screen axis (radians → normalised to FOV).
    let ang_x = (s[0] * right[0] + s[1] * right[1] + s[2] * right[2]).atan2(dot);
    let ang_y = (s[0] * up[0] + s[1] * up[1] + s[2] * up[2]).atan2(dot);
    let nx = ang_x.to_degrees() / FOV_HALF_DEG;
    let ny = ang_y.to_degrees() / FOV_HALF_DEG;
    let in_fov = nx.abs() <= 1.0 && ny.abs() <= 1.0 && dot > 0.0;
    ((nx, ny), offset_deg, in_fov)
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(v: Vec3) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn unit(v: Vec3) -> Vec3 {
    let n = norm(v);
    if n < 1e-12 {
        v
    } else {
        [v[0] / n, v[1] / n, v[2] / n]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use agc_core::control::imu_control::ImuAlignmentState;
    use agc_core::control::sextant::cdu_from_los_body;

    const IDENTITY: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    /// Point the optics exactly at a star (via the CDU inverse) and confirm the
    /// projected offset is ~0 and it reads "on mark".
    #[test]
    fn tc_sxt_centered_star_zero_offset() {
        let star = STAR_CATALOG[0].direction; // Alpheratz
        let (shaft, trunnion) = cdu_from_los_body(star);
        let p = los_body_from_cdu(shaft, trunnion);
        let (_screen, offset_deg, in_fov) = project(p, star);
        assert!(offset_deg < 0.1, "centered offset {offset_deg}° should be ~0");
        assert!(in_fov);
    }

    /// Slewing changes the corresponding CDU angle.
    #[test]
    fn tc_sxt_slew_changes_angle() {
        let mut hw = SimHardware::new();
        let s = MarkSession::new(IDENTITY);
        let before = hw.optics.shaft.0;
        s.slew(&mut hw, SlewAxis::ShaftPlus, false);
        assert_eq!(hw.optics.shaft.0, before.wrapping_add(COARSE_COUNTS));
        let t = hw.optics.trunnion.0;
        s.slew(&mut hw, SlewAxis::TrunnionMinus, true);
        assert_eq!(hw.optics.trunnion.0, t.wrapping_sub(FINE_COUNTS));
    }

    /// Two centred keystroke MARKs in P52 drive CoarseAligned → FineAligned.
    #[test]
    fn tc_sxt_two_mark_p52_reaches_fine() {
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut session = MarkSession::new(IDENTITY);
        state.major_mode = 52;
        state.imu_alignment_state = ImuAlignmentState::CoarseAligned;

        // Star 1 (Alpheratz): select via N70, aim optics at it, MARK.
        state.vn.crew_star_code = Some(1);
        let (sa, ta) = cdu_from_los_body(STAR_CATALOG[0].direction);
        hw.optics.shaft = sa;
        hw.optics.trunnion = ta;
        session.mark(&mut state, &mut hw);
        assert_eq!(session.pair_marks(), 1, "first mark buffers");
        assert_eq!(state.vn.crew_star_code, None, "mark consumes the star code");

        // Star 16 (Pollux): select, aim, MARK → pair completes.
        state.vn.crew_star_code = Some(16);
        let (sa, ta) = cdu_from_los_body(STAR_CATALOG[15].direction);
        hw.optics.shaft = sa;
        hw.optics.trunnion = ta;
        session.mark(&mut state, &mut hw);

        assert_eq!(state.alarm.code(), 0, "clean pair, no alarm");
        assert_eq!(
            state.imu_alignment_state,
            ImuAlignmentState::FineAligned,
            "two centred marks in P52 must reach FineAligned"
        );
        assert_eq!(session.pair_marks(), 0, "pair consumed");
    }

    /// MARK with no star selected is a no-op with a helpful status.
    #[test]
    fn tc_sxt_mark_without_star_is_noop() {
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        let mut session = MarkSession::new(IDENTITY);
        state.vn.crew_star_code = None;
        session.mark(&mut state, &mut hw);
        assert_eq!(session.pair_marks(), 0);
    }
}
