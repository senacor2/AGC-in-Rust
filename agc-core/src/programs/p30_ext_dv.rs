//! P30 — External Delta-V Targeting.
//!
//! P30 accepts a Time of Ignition (TIG) and a delta-V vector in LVLH frame
//! from the crew (or uplink), rotates it to ECI via S30.1, computes
//! apogee/perigee altitudes, and stores the BurnTarget for P40.
//!
//! AGC source: Comanche055/P30-P37.agc
//!   P30 (page 636), CNTNUP30 (page 636), PARAM30 (page 637), S30.1 (pages 639-640).
//! AGC source: Comanche055/CONIC_SUBROUTINES.agc
//!   PERIAPO1 — apogee/perigee computation.

use crate::guidance::targeting::{
    predict_vg_at_ignition, BurnTarget as GuidBurnTarget, SPS_THRUST_N, SPS_VE_MS,
};
use crate::hal::{AgcHardware, DskyIo};
use crate::math::linalg::norm;
use crate::navigation::conics::{apoapsis_periapsis, elements_from_state};
use crate::navigation::constants::{MU_EARTH, RE_EARTH};
use crate::types::{Met, Vec3};
use crate::AgcState;

/// Conversion factor: metres per second to feet per second.
const MPS_TO_FPS: f64 = 3.280_840;

/// Conversion factor: metres to nautical miles.
const M_TO_NM: f64 = 1.0 / 1852.0;

/// DSKY Noun for the N42 summary display (apogee/perigee/delta-V).
const N42_NOUN: u8 = 42;

/// Enter P30 External Delta-V Targeting.
///
/// Records TIG and the crew-supplied delta-V vector in LVLH frame into
/// `state.nav`. Sets UPDATFLG and TRACKFLG.
/// Sets restart phase for Group 4.
///
/// AGC source: Comanche055/P30-P37.agc P30/P31 common entry (page 636).
pub fn enter<H: AgcHardware>(state: &mut AgcState, hw: &mut H, tig: Met, delta_v_lvlh: Vec3) {
    // Restart protection: TC PHASCHNG / OCT 00014 (Group 4 phase 4).
    // AGC source: Comanche055/P30-P37.agc P30 entry (page 636).
    state.restart.set_phase(4, 4, false, 0);

    // Set flags.
    // AGC source: `TC UPFLAG / ADRES UPDATFLG` and `TC UPFLAG / ADRES TRACKFLG`.
    state.flags.updatflg = true;
    state.flags.trackflg = true;

    // Store TIG and DELVSLV.
    state.nav.tig = tig;
    state.nav.delvslv = delta_v_lvlh;

    // Set major mode to 30.
    state.modreg = 30;
    hw.dsky().write_prog(30);
}

/// Compute the BurnTarget from stored TIG and DELVSLV.
///
/// Performs the S30.1 algorithm:
///   1. Propagates the current state vector to TIG via Kepler.
///   2. Rotates DELVSLV (LVLH) to DELVSIN (ECI) via the LOMAT construction.
///   3. Computes delta-V magnitude (VGDISP).
///   4. Computes post-burn RTIG + DELVSIN state.
///   5. Calls PERIAPO1 to compute HAPO and HPER.
///   6. Stores the complete BurnTarget into state.
///   7. Sets XDELVFLG.
///
/// AGC source: Comanche055/P30-P37.agc S30.1 (pages 639-640).
pub fn compute_target(state: &mut AgcState) -> GuidBurnTarget {
    let bt_guidance = GuidBurnTarget {
        tig: state.nav.tig,
        delta_v_lvlh: state.nav.delvslv,
        mass: state.nav.mass_kg,
        thrust: SPS_THRUST_N,
        isp: SPS_VE_MS,
    };

    // Step 1+2: Propagate to TIG and rotate DELVSLV to DELVSIN (ECI).
    // AGC source: S30.1 THISPREC → LOMAT → DELVSIN computation.
    let delvsin = predict_vg_at_ignition(&state.nav.sv, &bt_guidance);
    state.nav.delvsin = delvsin;

    // Step 3: Delta-V magnitude.
    let vgdisp = norm(&delvsin);
    state.nav.vgdisp = vgdisp;

    // Step 4: Propagate state to TIG to get RTIG/VTIG.
    // For TIG = now, just use current state.
    let dt_tig = {
        let tig_s = state.nav.tig.as_secs_f64();
        let now_s = state.nav.sv.time().as_secs_f64();
        tig_s - now_s
    };
    let (rtig, vtig) = if dt_tig.abs() < 0.01 {
        (state.nav.sv.position(), state.nav.sv.velocity())
    } else {
        let kep = crate::math::kepler::kepler(
            &state.nav.sv.position(),
            &state.nav.sv.velocity(),
            dt_tig,
            MU_EARTH,
        );
        if kep.converged {
            (kep.r, kep.v)
        } else {
            (state.nav.sv.position(), state.nav.sv.velocity())
        }
    };
    state.nav.rtig = rtig;
    state.nav.vtig = vtig;

    // Step 5: Post-burn velocity = VTIG + DELVSIN.
    let v_post_burn = [
        vtig[0] + delvsin[0],
        vtig[1] + delvsin[1],
        vtig[2] + delvsin[2],
    ];

    // Step 5b: Compute HAPO / HPER via orbital elements.
    // AGC source: S30.1 calls PERIAPO1 with (RTIG, VTIG + DELVSIN).
    if let Some(elements) = elements_from_state(&rtig, &v_post_burn, MU_EARTH) {
        let (apoapsis, periapsis) = apoapsis_periapsis(&elements);
        // Convert from orbital radius to altitude above Earth surface.
        state.nav.hapo = (apoapsis - RE_EARTH).max(0.0);
        state.nav.hper = (periapsis - RE_EARTH).max(0.0);
    }

    // Step 7: Set XDELVFLG.
    state.flags.xdelvflg = true;
    state.flags.updatflg = false; // clear UPDATFLG on completion

    bt_guidance
}

/// Display the P30 computed summary on the DSKY (N42).
///
/// Renders:
///   R1 = HAPO apogee altitude in NM.
///   R2 = HPER perigee altitude in NM.
///   R3 = delta-V magnitude in fps.
/// Issues V06 N42 monitor display.
///
/// AGC source: Comanche055/P30-P37.agc PARAM30 label, page 637.
pub fn display_summary<H: AgcHardware>(state: &AgcState, hw: &mut H) {
    use crate::hal::DskyIo;
    let hapo_nm = libm::round(state.nav.hapo * M_TO_NM) as i32;
    let hper_nm = libm::round(state.nav.hper * M_TO_NM) as i32;
    let vg_fps = libm::round(state.nav.vgdisp * MPS_TO_FPS) as i32;

    hw.dsky().write_noun(N42_NOUN);
    write_signed_register(hw.dsky(), 0, hapo_nm);
    write_signed_register(hw.dsky(), 1, hper_nm);
    write_signed_register(hw.dsky(), 2, vg_fps);
}

fn write_signed_register<D: crate::hal::DskyIo>(dsky: &mut D, row: usize, value: i32) {
    use crate::hal::DigitRow;
    let abs_val = value.unsigned_abs();
    // Extract up to 5 decimal digits without heap allocation.
    let mut digits = [0xFF_u8; 5]; // 0xFF = blank
    let mut n = abs_val;
    let mut pos = 4_usize;
    let mut count = 0_usize;
    loop {
        digits[pos] = (n % 10) as u8;
        n /= 10;
        count += 1;
        if n == 0 || count == 5 {
            break;
        }
        pos -= 1;
    }
    // Blank leading positions (right-justify).
    let first = 5 - count;
    for d in digits.iter_mut().take(first) {
        *d = 0xFF;
    }
    let row_data = DigitRow {
        digits,
        sign_plus: value >= 0,
        sign_minus: value < 0,
    };
    dsky.write_register(row, &row_data);
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::StateVector;
    use crate::tests::mock_hw::MockHardware;
    use crate::types::Met;

    fn make_leo_state() -> AgcState {
        let r_185 = 6_556_370.0_f64;
        let v_c = libm::sqrt(MU_EARTH / r_185);
        let sv = StateVector::new([r_185, 0.0, 0.0], [0.0, v_c, 0.0], Met(0));
        let mut state = AgcState::new();
        state.nav.sv = sv;
        state
    }

    /// TC-P30-1: compute_target rotates prograde LVLH burn to ECI.
    ///
    /// AGC source: Comanche055/P30-P37.agc S30.1 DELVSLV→DELVSIN (page 639).
    #[test]
    fn compute_target_prograde_burn_eci() {
        let mut state = make_leo_state();
        let mut hw = MockHardware::new();
        let tig = Met(0);
        let dv_lvlh: Vec3 = [0.0, 50.0, 0.0]; // prograde
        enter(&mut state, &mut hw, tig, dv_lvlh);
        let bt = compute_target(&mut state);
        // Magnitude ≈ 50 m/s.
        let delvsin_mag = norm(&state.nav.delvsin);
        assert!(
            (delvsin_mag - 50.0).abs() < 1.0,
            "|DELVSIN| = {delvsin_mag:.3} m/s expected ≈50"
        );
        assert!(state.flags.xdelvflg, "XDELVFLG must be set");
        assert_eq!(bt.delta_v_lvlh, dv_lvlh);
    }

    /// TC-P30-2: compute_target produces valid apogee/perigee for prograde burn.
    ///
    /// AGC source: Comanche055/P30-P37.agc S30.1 → PERIAPO1.
    #[test]
    fn compute_target_apogee_raised_by_prograde_burn() {
        let mut state = make_leo_state();
        let mut hw = MockHardware::new();
        enter(&mut state, &mut hw, Met(0), [0.0, 50.0, 0.0]);
        compute_target(&mut state);
        // A prograde burn from circular orbit raises apogee.
        assert!(
            state.nav.hapo > 180_000.0,
            "HAPO = {} m expected > 180 km (some apogee raise)",
            state.nav.hapo
        );
        assert!(
            state.nav.hper < state.nav.hapo,
            "HPER ({}) must be < HAPO ({})",
            state.nav.hper,
            state.nav.hapo
        );
    }

    /// TC-P30-3: display_summary writes correct NM and fps values.
    ///
    /// AGC source: Comanche055/P30-P37.agc PARAM30 (page 637).
    #[test]
    fn display_summary_correct_units() {
        let mut state = make_leo_state();
        state.nav.hapo = 300_000.0;
        state.nav.hper = 185_000.0;
        state.nav.vgdisp = 50.0;
        let mut hw = MockHardware::new();
        display_summary(&state, &mut hw);
        // N42 noun.
        assert_eq!(hw.dsky.noun, Some(42));
        // R1 = 300000 / 1852 ≈ 162 NM.
        let r1 = hw.dsky.r1.unwrap_or(0);
        assert!((r1 - 162).abs() <= 1, "R1 = {r1} expected ≈162 NM");
        // R2 = 185000 / 1852 ≈ 100 NM.
        let r2 = hw.dsky.r2.unwrap_or(0);
        assert!((r2 - 100).abs() <= 1, "R2 = {r2} expected ≈100 NM");
        // R3 = 50 * 3.28084 ≈ 164 fps.
        let r3 = hw.dsky.r3.unwrap_or(0);
        assert!((r3 - 164).abs() <= 1, "R3 = {r3} expected ≈164 fps");
    }

    /// TC-P30-4: enter sets flags correctly.
    #[test]
    fn enter_sets_flags() {
        let mut state = make_leo_state();
        let mut hw = MockHardware::new();
        enter(&mut state, &mut hw, Met(0), [0.0, 50.0, 0.0]);
        assert!(state.flags.updatflg, "UPDATFLG must be set on P30 enter");
        assert!(state.flags.trackflg, "TRACKFLG must be set on P30 enter");
        assert_eq!(state.modreg, 30);
    }
}
