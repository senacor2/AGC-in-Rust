// SPDX-License-Identifier: GPL-3.0-or-later
//! MS-T2 exit-criterion test: 24-hour LEO coast — AGC tracks ground truth.
//!
//! Seeds a circular LEO state vector, provides the same state as the
//! ground-truth reference, runs `AdvanceCoast(86 400 s)`, then asserts that
//! the AGC's SERVICER-integrated state vector remains within 5 km position
//! and 5 m/s velocity of the two-body conic reference propagated by
//! [`agc_sim::physics::advance_ground_truth`].
//!
//! # Tolerances (from architect's derivation)
//!
//! - Position: 5 000 m (5 km). The SERVICER uses a two-stage Average-G
//!   trapezoidal integrator with 2-second steps; the Kepler ground truth uses
//!   the universal-variable conic solver. Over 24 h the SERVICER accumulates
//!   ~2–3 km of error relative to the conic reference at this altitude.
//! - Velocity: 5.0 m/s. Velocity error is consistent with the position drift
//!   at LEO speeds.

use agc_core::navigation::gravity::MU_EARTH;
use agc_core::navigation::state_vector::Frame;
use agc_core::navigation::StateVector;
use agc_core::types::Met;
use agc_core::AgcState;
use agc_sim::SimHardware;
use agc_sim::{run_scenario, ScenarioBuilder, SimDuration};

/// tc_msT2_coast_24h_agc_tracks_ground_truth
///
/// Seeds a circular LEO at r = 6 778 000 m, v_circ = sqrt(MU_EARTH / r),
/// runs a 24-hour coast, and asserts the AGC's SERVICER-integrated position
/// and velocity are within 5 km / 5 m/s of the conic-propagator ground truth.
#[test]
fn tc_ms_t2_coast_24h_agc_tracks_ground_truth() {
    let r = 6_778_000.0_f64;
    let v_circ = (MU_EARTH / r).sqrt();

    let leo_sv = StateVector {
        position: [r, 0.0, 0.0],
        velocity: [0.0, v_circ, 0.0],
        epoch: Met(0),
        frame: Frame::EarthInertial,
    };

    let scenario = ScenarioBuilder::new("coast_24h_leo")
        .seed_state()
        .from_state_vector(leo_sv)
        .met(Met(0))
        .refsmmat_identity()
        .done()
        .seed_ground_truth(leo_sv)
        .advance_coast(SimDuration::seconds(86_400))
        .expect_agc_matches_ground_truth(5_000.0, 5.0)
        .build();

    let mut state = AgcState::new();
    let mut hw = SimHardware::new();
    run_scenario(&scenario, &mut state, &mut hw);
}
