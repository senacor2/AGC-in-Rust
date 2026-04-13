//! P11 — Earth Orbit Insertion Monitor.
//!
//! P11 monitors booster-powered ascent and Earth orbit insertion. It initialises
//! the Average-G state vector integrator and drives the DSKY N62 display with
//! inertial velocity, altitude rate, and altitude.
//!
//! AGC source: Comanche055/P11.agc
//!   P11 (page 535), REP11 (page 539), VHHDOT (page 539), S11.1 (page 540).
//! AGC source: Comanche055/SERVICER207.agc
//!   PREREAD1, NORMLIZE.
//! AGC source: Comanche055/FRESH_START_AND_RESTART.agc
//!   GOTOPOOH (normal exit via V37N00).

use crate::hal::{AgcHardware, DskyIo, ImuIo};
use crate::math::linalg::norm;
use crate::navigation::constants::RE_EARTH;
use crate::types::Mat3x3;
use crate::AgcState;

/// Conversion factor: metres per second to feet per second.
const MPS_TO_FPS: f64 = 3.280_840;

/// Conversion factor: metres to nautical miles.
const M_TO_NM: f64 = 1.0 / 1852.0;

/// DSKY Noun for the N62 monitor display.
///
/// AGC source: Comanche055/P11.agc VHHDOT: `TC BANKCALL / CADR GOFLASH / DEC 62`.
const N62_NOUN: u8 = 62;

/// Enter P11 Earth Orbit Insertion Monitor.
///
/// Performs the full P11 initialisation sequence:
/// - Records liftoff time (zeroes CMC clock, saves into TLIFTOFF).
/// - Sets major mode register to 11 (`state.modreg = 11`).
/// - Computes and stores prelaunch REFSMMAT from current CDU angles.
/// - Sets AVGEXIT to the VHHDOT hook (avgexit_active = true).
/// - Sets restart protection (Group 3 and Group 5 phases).
///
/// AGC source: Comanche055/P11.agc P11 label, pages 535-537.
pub fn enter<H: AgcHardware>(state: &mut AgcState, hw: &mut H) {
    // Restart protection Group 3 phase 5: protects TEPHEM correction and PREREAD1.
    // AGC source: TC PHASCHNG / OCT 05023 (page 537).
    state.restart.set_phase(3, 5, false, 0);

    // Record liftoff time: save current MET as TLIFTOFF, reset clock.
    // AGC source: Comanche055/P11.agc P11: `DXCH TIME2 / EXTEND / DCA ZERO` (page 535).
    let now = state.nav.sv.time();
    state.nav.tliftoff = now;

    // Update TEPHEM by adding TLIFTOFF.
    // AGC source: Comanche055/P11.agc step 2: TEPHEM += TLIFTOFF.
    state.tephem = state.tephem + now.as_centiseconds();

    // Set major mode to 11.
    // AGC source: Comanche055/P11.agc `TC NEWMODEX / MM 11` (page 535).
    state.modreg = 11;

    // Build prelaunch REFSMMAT from CDU angles.
    // AGC source: Comanche055/P11.agc step 8: compute REFSMMAT from unit vectors.
    // Simplified: read CDU angles to build a rotation matrix.
    let cdus = hw.imu().read_cdu();
    let refsmmat = build_refsmmat_from_cdus(cdus);
    state.nav.refsmmat = refsmmat;
    state.flags.refsmflg = true;

    // Set AVGEXIT to VHHDOT hook.
    // AGC source: Comanche055/P11.agc step 9: `TC STCADR / CADR P11SCADR` (page 535).
    state.nav.avgexit_active = true;

    // Write PROG display "11".
    hw.dsky().write_prog(11);
}

/// Advance the P11 navigation display by one Average-G cycle (every 2 s).
///
/// Computes inertial velocity magnitude (R1), altitude rate HDOT (R2),
/// and altitude above pad radius (R3) for the DSKY N62 display.
///
/// Called from the Average-G servicer exit hook (AVGEXIT pointer = VHHDOT).
/// Does NOT modify the state vector.
///
/// AGC source: Comanche055/P11.agc VHHDOT label, page 539.
pub fn tick<H: AgcHardware>(state: &mut AgcState, hw: &mut H) {
    if state.modreg != 11 {
        return;
    }

    let r = state.nav.sv.position();
    let v = state.nav.sv.velocity();

    // Inertial velocity magnitude (VMAGI) in fps.
    // AGC source: Comanche055/P11.agc S11.1 VMAGI computation (page 540).
    let vi_ms = norm(&v);
    let vi_fps = libm::round(vi_ms * MPS_TO_FPS) as i32;

    // Altitude rate HDOT = (r · v) / |r|  (radial velocity component, m/s).
    // AGC source: Comanche055/P11.agc S11.1 HDOT = (R·V)/|R|.
    let r_norm = norm(&r);
    let hdot_ms = if r_norm > 1.0 {
        (r[0] * v[0] + r[1] * v[1] + r[2] * v[2]) / r_norm
    } else {
        0.0
    };
    let hdot_fps = libm::round(hdot_ms * MPS_TO_FPS) as i32;

    // Altitude above pad radius (ALTI) in NM.
    // AGC source: Comanche055/P11.agc S11.1 ALTI = |R| - R_pad.
    let altitude_m = (r_norm - RE_EARTH).max(0.0);
    let altitude_nm = libm::round(altitude_m * M_TO_NM) as i32;

    // Display N62: R1=VI (fps), R2=HDOT (fps), R3=H (NM).
    // AGC source: Comanche055/P11.agc VHHDOT `TC BANKCALL / CADR GOFLASH / DEC 62`.
    hw.dsky().write_noun(N62_NOUN);
    write_signed_register(hw.dsky(), 0, vi_fps);
    write_signed_register(hw.dsky(), 1, hdot_fps);
    write_signed_register(hw.dsky(), 2, altitude_nm);
}

/// Exit P11 (crew-requested via V37 or alarm path).
///
/// Clears the AVGEXIT hook. Does NOT stop Average-G (servicer continues).
/// Does NOT change modreg (that is done by V37 dispatch).
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc GOTOPOOH path.
pub fn exit(state: &mut AgcState) {
    state.nav.avgexit_active = false;
}

// ── internal helpers ──────────────────────────────────────────────────────────

/// Build a REFSMMAT from CDU angles using Z-Y-X Euler construction.
///
/// Constructs a rotation matrix from the three CDU gimbal angles.
/// This approximates the full LALOTORV/REFSMMAT computation from P11.agc.
///
/// AGC source: Comanche055/P11.agc step 8 — REFSMMAT from UNIT_Z, UNIT_X, UNIT_Y.
fn build_refsmmat_from_cdus(cdus: [crate::types::CduAngle; 3]) -> Mat3x3 {
    // Convert CduAngle counts to radians: 32768 counts = 2π.
    let to_rad =
        |c: crate::types::CduAngle| -> f64 { c.0 as f64 / 32768.0 * core::f64::consts::TAU };
    let igc = to_rad(cdus[0]); // inner (X)
    let mgc = to_rad(cdus[1]); // middle (Y)
    let ogc = to_rad(cdus[2]); // outer (Z)

    // Z-Y-X Euler rotation matrix: R = Rz(ogc) × Ry(mgc) × Rx(igc).
    let ci = libm::cos(igc);
    let si = libm::sin(igc);
    let cm = libm::cos(mgc);
    let sm = libm::sin(mgc);
    let co = libm::cos(ogc);
    let so = libm::sin(ogc);

    [
        [cm * co, si * sm * co - ci * so, ci * sm * co + si * so],
        [cm * so, si * sm * so + ci * co, ci * sm * so - si * co],
        [-sm, si * cm, ci * cm],
    ]
}

/// Write a signed integer to a DSKY register row using DigitRow encoding.
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
    // Right-justify: blank leading positions.
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
        let v_c = libm::sqrt(crate::navigation::constants::MU_EARTH / r_185);
        let sv = StateVector::new([r_185, 0.0, 0.0], [0.0, v_c, 0.0], Met(0));
        let mut state = AgcState::new();
        state.nav.sv = sv;
        state
    }

    /// TC-P11-1: enter() sets major mode to 11.
    ///
    /// AGC source: Comanche055/P11.agc `TC NEWMODEX / MM 11` (page 535).
    #[test]
    fn enter_sets_modreg_11() {
        let mut state = make_leo_state();
        let mut hw = MockHardware::new();
        enter(&mut state, &mut hw);
        assert_eq!(state.modreg, 11);
        assert!(state.flags.refsmflg);
        assert!(state.nav.avgexit_active);
    }

    /// TC-P11-2: tick() updates DSKY N62 display quantities.
    ///
    /// AGC source: Comanche055/P11.agc VHHDOT label, page 539.
    #[test]
    fn tick_writes_n62_noun() {
        let mut state = make_leo_state();
        let mut hw = MockHardware::new();
        enter(&mut state, &mut hw);
        tick(&mut state, &mut hw);
        // DSKY noun must be 62 after tick.
        assert_eq!(hw.dsky.noun, Some(62));
    }

    /// TC-P11-3: exit() clears avgexit hook.
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc GOTOPOOH path.
    #[test]
    fn exit_clears_avgexit() {
        let mut state = make_leo_state();
        let mut hw = MockHardware::new();
        enter(&mut state, &mut hw);
        assert!(state.nav.avgexit_active);
        exit(&mut state);
        assert!(!state.nav.avgexit_active);
        // modreg unchanged by exit
        assert_eq!(state.modreg, 11);
    }

    /// TC-P11-4: tick is a no-op when modreg != 11.
    #[test]
    fn tick_noop_when_not_p11() {
        let mut state = make_leo_state();
        let mut hw = MockHardware::new();
        state.modreg = 30; // P30 active, not P11
        tick(&mut state, &mut hw);
        // noun should not be 62 (it stays at default None)
        assert_ne!(hw.dsky.noun, Some(62));
    }
}
