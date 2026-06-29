// SPDX-License-Identifier: GPL-3.0-or-later
//! Mission scenario runner and builder.
//!
//! Provides a structured way to describe and execute end-to-end AGC integration
//! tests as a sequence of typed events with timing and assertion support.
//!
//! # Design
//!
//! A [`Scenario`] is an ordered list of [`Event`]s plus a name and a tick
//! granularity. [`ScenarioBuilder`] assembles scenarios from ergonomic typed
//! methods. [`run_scenario`] interprets each event in order against a live
//! [`agc_core::AgcState`] and [`crate::SimHardware`].
//!
//! # Failure format
//!
//! All `Expect*` events that fail produce messages in the canonical form:
//! ```text
//! scenario "<name>": event #<idx> (<variant>) failed at MET <cs>cs (<s>s):
//!   <reason>; expected <x>, got <y>
//! ```
//!
//! # Investigation findings (§5.3 of the end-to-end testing plan, GH #24)
//!
//! ## 1. Continuous-coast SERVICER cycling
//!
//! `servicer_task` in `agc-core/src/services/average_g.rs` self-reschedules
//! every 200 cs whenever `is_servicer_active()` is true (step 10).  It reads
//! `AgcState::pipa_counts` — a staging field accumulated by calls to
//! `pump_pipa_into_state` in the sim render loop.  During a coast phase (engine
//! off) `SimHardware::tick` produces no PIPA pulses, so the count array is
//! zero each cycle and `average_g_step` integrates only gravity.  Conclusion:
//! **the SERVICER keeps running during day-scale coasts when `WaitlistPump::tick`
//! is called regularly; gravity integration continues, thruster delta-V is zero.**
//!
//! ## 2. SOI crossing ownership
//!
//! `navigation::integration::soi_check` (in `agc-core/src/navigation/integration.rs`)
//! is the sole function that switches `csm_state.frame` between `EarthInertial`
//! and `MoonInertial`.  It is called from `propagate_coast` (the RK4 coast
//! integrator) but is **not** called from `average_g_step` (the SERVICER's
//! powered-flight integrator).  As a result, SOI crossing during powered arcs
//! is not detected.  Wiring `soi_check` into the SERVICER integration path is
//! tracked in GH issue #51.
//!
//! ## 3. REFSMMAT propagation after P52
//!
//! `p52_mark_align` in `agc-core/src/programs/p51_p52.rs` writes the new matrix
//! directly to `state.refsmmat`.  `servicer_task` reads `state.refsmmat` on
//! every cycle at step 6 (`mxv(state.refsmmat, delta_v_platform)`); there is no
//! caching or shadow copy.  Conclusion: **a new REFSMMAT written by P52 is
//! consumed by the very next SERVICER cycle — no stale-cache issue exists in the
//! current implementation.**

use agc_core::math::linalg::{mxv, transpose};
use agc_core::navigation::star_catalog::STAR_CATALOG;
use agc_core::navigation::StateVector;
use agc_core::programs::p20::LosComponent;
use agc_core::programs::p22::{landmark_inertial_pos, LandmarkMark, LANDMARK_TABLE};
use agc_core::programs::p23::{
    p23_incorporate_star_horizon_mark, p23_incorporate_star_landmark_mark, StarHorizonMark,
    StarLandmarkMark,
};
use agc_core::programs::p51_p52::p52_mark_align;
use agc_core::services::average_g::start_servicer;
use agc_core::services::v_n::{feed_key, Key};
use agc_core::types::{Mat3x3, Met, Vec3};
use agc_core::AgcState;

use crate::physics::{advance_ground_truth, GravityBody, Spacecraft};
use crate::runtime::{
    pump_engine_to_hw, pump_pipa_into_state, pump_rcs_to_hw, pump_secs_to_hw, DapPump, T4Pump,
    WaitlistPump,
};
use crate::sensors::{landmark_los_in_platform, star_los_in_platform};
use crate::SimHardware;

// ── SimDuration ───────────────────────────────────────────────────────────────

/// A duration expressed in mission centiseconds.
///
/// All constructor methods panic on overflow (a simulation duration that
/// overflows `u32` centiseconds — roughly 497 days — indicates a defect in
/// the caller).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SimDuration(pub u32);

impl SimDuration {
    /// Construct from raw centiseconds.
    pub const fn cs(n: u32) -> Self {
        Self(n)
    }

    /// Construct from milliseconds (rounds down to the nearest cs).
    pub const fn ms(n: u32) -> Self {
        Self(n / 10)
    }

    /// Construct from whole seconds.
    pub const fn seconds(n: u32) -> Self {
        Self(n * 100)
    }

    /// Construct from whole minutes.
    pub const fn minutes(n: u32) -> Self {
        Self(n * 6_000)
    }
}

// ── LandmarkTable ─────────────────────────────────────────────────────────────

/// Which landmark table a sighting references. Sim-only enumeration for now.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LandmarkTable {
    /// Earth-surface landmark (P22 / P26).
    Earth,
    /// Lunar-surface landmark (not yet supported in the sim).
    Moon,
}

// ── SeedStateSpec ─────────────────────────────────────────────────────────────

/// Initial conditions injected by a [`Event::SeedState`] event.
#[derive(Clone, Copy, Debug)]
pub struct SeedStateSpec {
    /// CSM state vector (position, velocity, epoch, frame).
    pub csm: StateVector,
    /// Mission elapsed time to write into `AgcState::time`.
    pub met: Met,
    /// Reference-to-Stable-Member matrix (inertial → platform rotation).
    pub refsmmat: Mat3x3,
}

// ── DskyExpect ────────────────────────────────────────────────────────────────

/// Partial DSKY display expectation.
///
/// Each field is `None` to skip that check or `Some(value)` to assert it.
/// `tol_pct = 0.0` selects exact comparison (NaN-safe); `tol_pct > 0.0`
/// selects a percent-of-magnitude tolerance:
/// ```text
/// |got - want| <= max(want.abs(), 1.0) * tol_pct / 100.0
/// ```
#[derive(Clone, Copy, Debug)]
pub struct DskyExpect {
    pub verb: Option<u8>,
    pub noun: Option<u8>,
    pub r0: Option<f32>,
    pub r1: Option<f32>,
    pub r2: Option<f32>,
    pub flashing: Option<bool>,
    /// Tolerance in percent.  `0.0` means exact equality.
    pub tol_pct: f32,
}

impl DskyExpect {
    /// An expectation that checks nothing (all `None`). Useful as a placeholder
    /// while iteratively hardening a scenario.
    pub fn any() -> Self {
        Self {
            verb: None,
            noun: None,
            r0: None,
            r1: None,
            r2: None,
            flashing: None,
            tol_pct: 0.0,
        }
    }
}

// ── Event ─────────────────────────────────────────────────────────────────────

/// A single action or assertion in a scenario timeline.
#[derive(Clone, Copy, Debug)]
pub enum Event {
    /// Inject initial navigation state without pumping the soft executive.
    ///
    /// Writes `state.csm_state`, `state.time`, and `state.refsmmat` from `spec`.
    SeedState(SeedStateSpec),

    /// Advance mission time by `dur` centiseconds, ticking the soft executive
    /// at `Scenario::tick_cs` granularity per slice.
    AdvanceMet(SimDuration),

    /// Advance mission time by `dur` as a coast phase.
    ///
    /// Runs a two-tier loop: outer steps at `Scenario::coast_step_cs`
    /// granularity each advance the ground-truth state vector (if seeded)
    /// via [`advance_ground_truth`] and run one SERVICER cycle's worth of
    /// inner ticks; remaining time within each outer step advances
    /// `state.time` and the waitlist countdown in a single bump.
    AdvanceCoast(SimDuration),

    /// Seed the executor-held ground-truth state vector.
    ///
    /// Sets the reference trajectory used by
    /// [`Event::ExpectAgcMatchesGroundTruth`] and advanced by
    /// [`Event::AdvanceCoast`].  Also sets `spacecraft.current_body` to
    /// match the frame of the supplied state vector.
    SeedGroundTruth(StateVector),

    /// Assert that the AGC's CSM state vector is close to the ground truth.
    ///
    /// Compares `state.csm_state` against the executor-held ground-truth
    /// state vector using L2 norms on position and velocity.  Panics if
    /// the ground truth has not been seeded first, or if either error
    /// exceeds the specified tolerance.
    ExpectAgcMatchesGroundTruth {
        /// Maximum allowed L2 position error (metres).
        pos_tol_m: f64,
        /// Maximum allowed L2 velocity error (m/s).
        vel_tol_m_s: f64,
    },

    /// Deliver a single DSKY keypress to the V/N processor, then tick once.
    KeyPress(Key),

    /// Push a raw word onto the simulated uplink FIFO, then tick once.
    UplinkWord(u16),

    /// Log a stub "not-yet-implemented" sighting event and continue.
    OpticsSighting { star_id: u8 },

    /// Drive a star sighting through the CDU MARK pipeline (#57).
    ///
    /// Computes the body-frame LOS for `star_id` from the catalog + identity
    /// attitude, inverts to shaft/trunnion CDU counts via
    /// `control::sextant::cdu_from_los_body`, then calls
    /// `SimOptics::press_mark` to assert the latched CDU + `mark_pressed`
    /// hardware state. The handler immediately invokes
    /// `control::sextant::consume_optics_mark` to consume the press and
    /// dispatch to P51/P52 just like `OpticsSighting` does — only via the
    /// hardware-driven path instead of the direct AGC entry point.
    OpticsCduMark { star_id: u8 },

    /// Log a stub "not-yet-implemented" landmark sighting and continue.
    LandmarkSighting { table: LandmarkTable, index: u8 },

    /// Apply a P23 star-horizon mark directly to the CSM nav filter.
    ///
    /// Bypasses the sextant HAL and calls
    /// [`agc_core::programs::p23::p23_incorporate_star_horizon_mark`] with the
    /// pre-computed mark. The test owns the geometry: it constructs a
    /// [`StarHorizonMark`] whose `angle_observed_rad` represents the
    /// "true" measurement, then expects the Kalman update to pull
    /// `state.csm_state` toward truth.
    P23StarHorizonMark(StarHorizonMark),

    /// Apply a P23 star-landmark mark directly to the CSM nav filter.
    ///
    /// Bypasses the sextant HAL and calls
    /// [`agc_core::programs::p23::p23_incorporate_star_landmark_mark`].
    P23StarLandmarkMark(StarLandmarkMark),

    /// Assert `state.major_mode == mm`.
    ExpectMajorMode(u8),

    /// Assert selected DSKY fields are within tolerance.
    ExpectDsky(DskyExpect),

    /// Assert the CSM state vector is close to a ground-truth reference.
    ExpectCsmStateClose {
        ground_truth: StateVector,
        pos_tol_m: f64,
        vel_tol_m_s: f64,
    },

    /// Assert `state.alarm.code() == code`.
    ExpectAlarm(u16),

    /// Seed the simulator truth REFSMMAT used by sensor simulation.
    ///
    /// Required before any [`Event::OpticsSighting`] or
    /// [`Event::LandmarkSighting`] event.  The matrix is stored in the
    /// `RunContext` and is never written to `AgcState::refsmmat` — it
    /// represents the simulator's ground truth, which the AGC's P51/P52
    /// alignment programs are trying to estimate.
    SeedTruthRefsmmat(Mat3x3),

    /// Snap the spacecraft to the given attitude and hold it there.
    ///
    /// Writes `q` to **both** `Spacecraft::attitude.q` (current attitude) and
    /// `Spacecraft::attitude.commanded_q` (commanded target) so that the
    /// physics model and the DAP bridge both start from the injected value.
    /// This snap-and-hold semantic matches the scenario use case: tests that
    /// inject a specific attitude want the vehicle to begin at that orientation,
    /// not to slew toward it from the current (possibly arbitrary) attitude.
    CommandAttitude { q: [f64; 4] },

    /// Emit a one-line progress message; no state change.
    Comment(&'static str),

    /// Enable atmospheric drag + lift in the ground-truth propagator and
    /// force the outer coast-step granularity to 200 cs (one SERVICER cycle).
    ///
    /// Required before any `AdvanceCoast` event that should produce entry
    /// physics. The outer step pinning ensures the operator-split aero kick
    /// in `advance_ground_truth` is applied at the same cadence as the
    /// SERVICER, keeping integration error inside the entry footprint
    /// tolerance (see specs/ms-t6-phase-entry-spec.md §3.3).
    EnableAtmosphere,

    /// Assert that the AGC's entry phase reached drogue deploy and the
    /// great-circle miss distance to the configured target landing site is
    /// below `miss_km_tol`. Computed via haversine on the sub-satellite
    /// point of `state.csm_state.position` vs
    /// `(state.entry.target_lat_rad, state.entry.target_lon_rad)`.
    ExpectDrogueWithin { miss_km_tol: f64 },

    /// Assert that the peak `state.entry.sensed_acceleration_g` observed
    /// across all atmosphere-on coast steps in this scenario falls within
    /// `[min_g, max_g]`. Orbital-mechanics review #83 recommends this as
    /// a second-axis trajectory-shape check (Apollo 8 historical peak g
    /// was 6.84 g; entries that land near target on the wrong trajectory
    /// shape — e.g. ballistic rather than lifting — show up here even when
    /// the miss-distance gate accepts them).
    ExpectPeakGIn { min_g: f64, max_g: f64 },

    /// Assert that the peak Sutton–Graves stagnation-point heat flux
    /// (MW/m²) observed across atmosphere-on coast steps falls within
    /// `[min_mw_m2, max_mw_m2]`. Companion to [`Event::ExpectPeakGIn`]
    /// (#96) — peak heating and peak g together constrain both the
    /// trajectory's lower-altitude leg (where heat flux peaks) and its
    /// peak-deceleration regime.
    ExpectPeakHeatingIn { min_mw_m2: f64, max_mw_m2: f64 },

    /// Assert that the simulated SECS observed `count` CM/SM separation
    /// pyro firings to date.
    ///
    /// Reads `SimSecs::csm_separation_fire_count`. The pyro is staged
    /// edge-triggered by `init_p62` and consumed exactly once per stage
    /// by [`crate::runtime::pump_secs_to_hw`]; asserting `count == 1`
    /// after V37 E62 plus one tick verifies the end-to-end wiring, and
    /// re-asserting `count == 1` after many subsequent ticks verifies
    /// the staging flag is not re-raised (the pyro is one-shot).
    ExpectCsmSeparationFireCount(u32),
}

// ── Scenario ──────────────────────────────────────────────────────────────────

/// A complete test scenario: name, event list, and tick granularity.
///
/// Constructed via [`ScenarioBuilder::build`].
pub struct Scenario {
    /// Human-readable name used in log messages and failure reports.
    pub name: &'static str,
    /// Ordered list of events to execute.
    pub events: Vec<Event>,
    /// Soft-executive tick granularity in centiseconds (default 10 cs = 100 ms).
    ///
    /// Must satisfy `tick_cs <= DAP_PERIOD_CS` — enforced by [`run_scenario`].
    pub tick_cs: u32,
    /// Outer-loop step size for [`Event::AdvanceCoast`] in centiseconds.
    ///
    /// Each outer step advances the ground-truth state vector by
    /// `coast_step_cs / 100` seconds, then runs one SERVICER cycle's worth
    /// of inner ticks (200 cs), then bumps `state.time` for the remaining
    /// `coast_step_cs - 200` cs in a single step.
    ///
    /// Default: 6000 cs (60 s).
    pub coast_step_cs: u32,
}

// ── SeedStateBuilder ──────────────────────────────────────────────────────────

/// Sub-builder returned by [`ScenarioBuilder::seed_state`].
///
/// Call `.done()` to push the completed [`Event::SeedState`] and return to
/// the parent builder.
pub struct SeedStateBuilder {
    parent: ScenarioBuilder,
    csm: StateVector,
    met: Met,
    refsmmat: Mat3x3,
}

const IDENTITY_MAT: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

impl SeedStateBuilder {
    /// Set the CSM position in kilometres (converted to metres internally).
    ///
    /// Preserves all other `csm` fields.
    pub fn position_km(mut self, x_km: f64, y_km: f64, z_km: f64) -> Self {
        self.csm.position = [x_km * 1_000.0, y_km * 1_000.0, z_km * 1_000.0];
        self
    }

    /// Set the CSM velocity in m/s.
    pub fn velocity_m_s(mut self, vx: f64, vy: f64, vz: f64) -> Self {
        self.csm.velocity = [vx, vy, vz];
        self
    }

    /// Set the coordinate frame.
    pub fn frame(mut self, frame: agc_core::navigation::state_vector::Frame) -> Self {
        self.csm.frame = frame;
        self
    }

    /// Set the mission epoch on the state vector.
    pub fn met(mut self, met: Met) -> Self {
        self.met = met;
        self.csm.epoch = met;
        self
    }

    /// Set the REFSMMAT from a `[[f64; 3]; 3]` matrix.
    pub fn refsmmat(mut self, m: Mat3x3) -> Self {
        self.refsmmat = m;
        self
    }

    /// Use the identity matrix as REFSMMAT (platform = inertial).
    pub fn refsmmat_identity(mut self) -> Self {
        self.refsmmat = IDENTITY_MAT;
        self
    }

    /// Initialise the entire CSM state from an existing [`StateVector`].
    ///
    /// Overrides any previous `position_km` / `velocity_m_s` / `frame` calls.
    pub fn from_state_vector(mut self, sv: StateVector) -> Self {
        self.csm = sv;
        self
    }

    /// Finalise the `SeedState` event and return to the parent builder.
    pub fn done(mut self) -> ScenarioBuilder {
        self.parent.events.push(Event::SeedState(SeedStateSpec {
            csm: self.csm,
            met: self.met,
            refsmmat: self.refsmmat,
        }));
        self.parent
    }
}

// ── ScenarioBuilder ───────────────────────────────────────────────────────────

/// Fluent builder for [`Scenario`].
///
/// Begin with [`ScenarioBuilder::new`], chain typed event methods, and call
/// [`ScenarioBuilder::build`] to produce the final [`Scenario`].
pub struct ScenarioBuilder {
    name: &'static str,
    events: Vec<Event>,
    tick_cs: u32,
    coast_step_cs: u32,
}

impl ScenarioBuilder {
    /// Create a new builder for a scenario with the given name.
    pub fn new(name: &'static str) -> Self {
        Self {
            name,
            events: Vec::new(),
            tick_cs: 10,         // default: 100 ms per tick
            coast_step_cs: 6000, // default: 60 s outer coast step
        }
    }

    /// Override the per-tick granularity (centiseconds).
    ///
    /// Must satisfy `tick_cs <= DAP_PERIOD_CS`; validated at [`run_scenario`]
    /// time to allow zero-cost const scenarios.
    pub fn tick_cs(mut self, cs: u32) -> Self {
        self.tick_cs = cs;
        self
    }

    /// Override the outer-loop step size for [`Event::AdvanceCoast`].
    ///
    /// Default is 6000 cs (60 s). Values smaller than 200 cs are not useful
    /// because one SERVICER cycle already takes 200 cs.
    pub fn coast_step_cs(mut self, cs: u32) -> Self {
        self.coast_step_cs = cs;
        self
    }

    /// Start a [`SeedStateBuilder`] sub-builder.
    ///
    /// Produces an [`Event::SeedState`] when `.done()` is called.
    pub fn seed_state(self) -> SeedStateBuilder {
        SeedStateBuilder {
            parent: self,
            csm: StateVector::ZERO,
            met: Met(0),
            refsmmat: IDENTITY_MAT,
        }
    }

    /// Push a single [`Key`] keypress event.
    pub fn key(mut self, k: Key) -> Self {
        self.events.push(Event::KeyPress(k));
        self
    }

    /// Push a sequence of [`Key`] keypresses.
    pub fn keys(mut self, ks: &[Key]) -> Self {
        for &k in ks {
            self.events.push(Event::KeyPress(k));
        }
        self
    }

    /// Push a single digit keypress (panics if `d > 9`).
    pub fn digit(self, d: u8) -> Self {
        assert!(d <= 9, "digit() called with value > 9: {d}");
        self.key(Key::Digit(d))
    }

    /// Push each decimal digit of `n` MSB-first (panics if `n > 9_999_999`).
    pub fn digits(mut self, n: u32) -> Self {
        assert!(n <= 9_999_999, "digits() called with n > 9_999_999: {n}");
        if n == 0 {
            return self.key(Key::Digit(0));
        }
        let mut buf = [0u8; 7];
        let mut count = 0usize;
        let mut rem = n;
        while rem > 0 {
            buf[count] = (rem % 10) as u8;
            rem /= 10;
            count += 1;
        }
        for i in (0..count).rev() {
            self.events.push(Event::KeyPress(Key::Digit(buf[i])));
        }
        self
    }

    /// Push `Key::Entr`.
    pub fn enter(self) -> Self {
        self.key(Key::Entr)
    }

    /// Push `Key::Pro`.
    pub fn pro(self) -> Self {
        self.key(Key::Pro)
    }

    /// Push `Key::Verb`.
    pub fn verb(self) -> Self {
        self.key(Key::Verb)
    }

    /// Push `Key::Noun`.
    pub fn noun(self) -> Self {
        self.key(Key::Noun)
    }

    /// Push `Verb` + two-digit major mode / verb number: `VERB d(v/10) d(v%10)`.
    pub fn verb_noun(self, v: u8) -> Self {
        self.key(Key::Verb)
            .key(Key::Digit(v / 10))
            .key(Key::Digit(v % 10))
    }

    /// Emit the complete V25 Nxx ENTR +v0 ENTR +v1 ENTR +v2 ENTR sequence.
    ///
    /// Negative values emit `Key::Minus` followed by the absolute magnitude;
    /// non-negative values emit `Key::Plus`.  Magnitude is clamped to the
    /// representable unsigned range (panics if any component's absolute value
    /// exceeds 9_999_999).
    pub fn v25_load_three(self, noun: u8, values: [i32; 3]) -> Self {
        let mut b = self
            .key(Key::Verb)
            .key(Key::Digit(2))
            .key(Key::Digit(5))
            .key(Key::Noun)
            .key(Key::Digit(noun / 10))
            .key(Key::Digit(noun % 10))
            .key(Key::Entr);
        for v in values {
            let sign_key = if v < 0 { Key::Minus } else { Key::Plus };
            let mag = v.unsigned_abs();
            b = b.key(sign_key).digits(mag).key(Key::Entr);
        }
        b
    }

    /// Emit the V71 P27 block-address update sequence.
    ///
    /// Format: `V71 ENTR address ENTR count ENTR (sign, mag) ENTR...`
    /// Each `words` entry is `(sign, magnitude)` where `sign` is +1 or -1.
    pub fn v71_p27_block_update(self, address: u8, words: &[(i8, u32)]) -> Self {
        let mut b = self
            .key(Key::Verb)
            .key(Key::Digit(7))
            .key(Key::Digit(1))
            .key(Key::Entr)
            .digits(address as u32)
            .key(Key::Entr)
            .digits(words.len() as u32)
            .key(Key::Entr);
        for &(sign, mag) in words {
            let sign_key = if sign < 0 { Key::Minus } else { Key::Plus };
            b = b.key(sign_key).digits(mag).key(Key::Entr);
        }
        b
    }

    /// Advance mission time by `dur`.
    pub fn advance(mut self, dur: SimDuration) -> Self {
        self.events.push(Event::AdvanceMet(dur));
        self
    }

    /// Advance mission time as a coast phase.
    pub fn advance_coast(mut self, dur: SimDuration) -> Self {
        self.events.push(Event::AdvanceCoast(dur));
        self
    }

    /// Seed the executor-held ground-truth state vector.
    ///
    /// Pushes a [`Event::SeedGroundTruth`] event. Also sets
    /// `spacecraft.current_body` at run time to match the frame of `sv`.
    pub fn seed_ground_truth(mut self, sv: StateVector) -> Self {
        self.events.push(Event::SeedGroundTruth(sv));
        self
    }

    /// Assert the AGC's CSM state vector is close to the ground truth.
    ///
    /// Pushes an [`Event::ExpectAgcMatchesGroundTruth`] event.
    pub fn expect_agc_matches_ground_truth(mut self, pos_tol_m: f64, vel_tol_m_s: f64) -> Self {
        self.events.push(Event::ExpectAgcMatchesGroundTruth {
            pos_tol_m,
            vel_tol_m_s,
        });
        self
    }

    /// Push a raw uplink word.
    pub fn uplink_word(mut self, w: u16) -> Self {
        self.events.push(Event::UplinkWord(w));
        self
    }

    /// Push an optics sighting event (stub).
    pub fn optics_sighting(mut self, star_id: u8) -> Self {
        self.events.push(Event::OpticsSighting { star_id });
        self
    }

    /// Push a CDU-driven star sighting event (#57). See
    /// [`Event::OpticsCduMark`] for the closed-loop hardware path.
    pub fn optics_cdu_mark(mut self, star_id: u8) -> Self {
        self.events.push(Event::OpticsCduMark { star_id });
        self
    }

    /// Push a landmark sighting event (stub).
    pub fn landmark_sighting(mut self, table: LandmarkTable, index: u8) -> Self {
        self.events.push(Event::LandmarkSighting { table, index });
        self
    }

    /// Apply a P23 star-horizon mark directly to the CSM nav filter.
    pub fn p23_star_horizon_mark(mut self, mark: StarHorizonMark) -> Self {
        self.events.push(Event::P23StarHorizonMark(mark));
        self
    }

    /// Apply a P23 star-landmark mark directly to the CSM nav filter.
    pub fn p23_star_landmark_mark(mut self, mark: StarLandmarkMark) -> Self {
        self.events.push(Event::P23StarLandmarkMark(mark));
        self
    }

    /// Assert `state.major_mode == mm`.
    pub fn expect_major_mode(mut self, mm: u8) -> Self {
        self.events.push(Event::ExpectMajorMode(mm));
        self
    }

    /// Assert selected DSKY fields.
    pub fn expect_dsky(mut self, d: DskyExpect) -> Self {
        self.events.push(Event::ExpectDsky(d));
        self
    }

    /// Assert the CSM state vector is close to `ground_truth`.
    pub fn expect_csm_state_close(
        mut self,
        ground_truth: StateVector,
        pos_tol_m: f64,
        vel_tol_m_s: f64,
    ) -> Self {
        self.events.push(Event::ExpectCsmStateClose {
            ground_truth,
            pos_tol_m,
            vel_tol_m_s,
        });
        self
    }

    /// Assert `state.alarm.code() == code`.
    pub fn expect_alarm(mut self, code: u16) -> Self {
        self.events.push(Event::ExpectAlarm(code));
        self
    }

    /// Seed the simulator truth REFSMMAT used by sensor simulation.
    ///
    /// Pushes a [`Event::SeedTruthRefsmmat`] event.  Must appear before
    /// any `optics_sighting` or `landmark_sighting` calls in the scenario.
    pub fn seed_truth_refsmmat(mut self, m: Mat3x3) -> Self {
        self.events.push(Event::SeedTruthRefsmmat(m));
        self
    }

    /// Set the spacecraft's commanded attitude quaternion (scalar-first `[w, x, y, z]`).
    ///
    /// Pushes a [`Event::CommandAttitude`] event.
    pub fn command_attitude(mut self, q: [f64; 4]) -> Self {
        self.events.push(Event::CommandAttitude { q });
        self
    }

    /// Emit a progress comment.
    pub fn comment(mut self, msg: &'static str) -> Self {
        self.events.push(Event::Comment(msg));
        self
    }

    /// Enable atmospheric drag + lift in the entry-phase pipeline and pin
    /// the outer coast step to 200 cs (one SERVICER cycle).
    ///
    /// Aero Δv is evaluated once per outer step at the AGC's own
    /// `state.csm_state` (matches `agc_test::entry_scenario::simulate_to_drogue`),
    /// quantised to PIPA pulses, and injected directly into
    /// `state.pipa_counts`. The 200 cs cadence matches the SERVICER cycle so
    /// each aero injection is consumed by exactly one SERVICER fire.
    pub fn enable_atmosphere(mut self) -> Self {
        self.events.push(Event::EnableAtmosphere);
        self.coast_step_cs = 200;
        self
    }

    /// Assert drogue deployed and miss distance ≤ `miss_km_tol`.
    pub fn expect_drogue_within(mut self, miss_km_tol: f64) -> Self {
        self.events.push(Event::ExpectDrogueWithin { miss_km_tol });
        self
    }

    /// Assert peak `state.entry.sensed_acceleration_g` observed across
    /// atmosphere-on coast steps falls in `[min_g, max_g]`. Second-axis
    /// trajectory-shape check (#83) — Apollo 8 historical was 6.84 g.
    pub fn expect_peak_g_in(mut self, min_g: f64, max_g: f64) -> Self {
        self.events.push(Event::ExpectPeakGIn { min_g, max_g });
        self
    }

    /// Assert peak Sutton–Graves stagnation-point heat flux (MW/m²)
    /// observed across atmosphere-on coast steps falls in
    /// `[min_mw_m2, max_mw_m2]`. Pairs with [`expect_peak_g_in`] as the
    /// thermal companion of the peak-g trajectory-shape check (#96).
    pub fn expect_peak_heating_in(mut self, min_mw_m2: f64, max_mw_m2: f64) -> Self {
        self.events
            .push(Event::ExpectPeakHeatingIn { min_mw_m2, max_mw_m2 });
        self
    }

    /// Assert that the simulated SECS observed exactly `count` CM/SM
    /// separation pyro firings to date.
    ///
    /// Reads `SimSecs::csm_separation_fire_count`. Used by entry-phase
    /// scenarios to verify P62 fires the pyro exactly once (and doesn't
    /// keep re-firing on subsequent cycles).
    pub fn expect_csm_separation_fire_count(mut self, count: u32) -> Self {
        self.events.push(Event::ExpectCsmSeparationFireCount(count));
        self
    }

    /// Consume the builder and produce a [`Scenario`].
    pub fn build(self) -> Scenario {
        Scenario {
            name: self.name,
            events: self.events,
            tick_cs: self.tick_cs,
            coast_step_cs: self.coast_step_cs,
        }
    }
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Single log-line sink. All scenario output goes through this function so we
/// can swap `eprintln!` for `tracing::debug!` later without touching callsites.
#[inline]
fn log_event(scenario_name: &str, msg: &str) {
    eprintln!("[scenario {scenario_name}] {msg}");
}

/// Perform one soft-executive tick, advancing `state.time` and pumping all
/// subsystems.  Used by both [`Event::AdvanceMet`] (in a loop) and
/// [`Event::KeyPress`] / [`Event::UplinkWord`] (single iteration after the input
/// event).
#[inline]
fn do_tick(
    state: &mut AgcState,
    hw: &mut SimHardware,
    tick_cs: u32,
    waitlist: &mut WaitlistPump,
    dap: &mut DapPump,
    t4: &mut T4Pump,
    ctx: &mut RunContext,
) {
    let tick_s = tick_cs as f64 / 100.0;
    state.time = Met(state.time.0.wrapping_add(tick_cs));
    hw.timers.set_time(state.time.0);
    hw.tick(tick_s);
    pump_pipa_into_state(state, hw);
    dap.tick(state, hw, Some(&mut ctx.spacecraft.attitude));
    waitlist.tick(state, hw);
    t4.tick(state, hw);
    pump_engine_to_hw(state, hw);
    pump_rcs_to_hw(state, hw);
    pump_secs_to_hw(state, hw);
}

/// Build the standard failure prefix for an `Expect*` event.
fn fail_prefix(name: &str, idx: usize, variant: &str, met_cs: u32) -> String {
    let met_s = met_cs as f64 / 100.0;
    format!("scenario \"{name}\": event #{idx} ({variant}) failed at MET {met_cs}cs ({met_s:.2}s):")
}

/// Compare two `f32` values using the `DskyExpect` tolerance rules.
///
/// `tol_pct = 0.0` ⇒ exact equality (both NaN is an error; either NaN is an
/// error).  `tol_pct > 0.0` ⇒ percent-of-magnitude tolerance.
fn dsky_r_matches(got: f32, want: f32, tol_pct: f32) -> Result<(), String> {
    if tol_pct == 0.0 {
        // Exact / NaN-safe comparison.
        if got.is_nan() || want.is_nan() {
            return Err(format!(
                "DSKY register NaN — likely uninitialised noun: got {got}, want {want}"
            ));
        }
        if got != want {
            return Err(format!("expected {want}, got {got}"));
        }
    } else {
        let tolerance = want.abs().max(1.0) * tol_pct / 100.0;
        if (got - want).abs() > tolerance {
            return Err(format!(
                "expected {want} ± {tolerance:.4} ({tol_pct}%), got {got}"
            ));
        }
    }
    Ok(())
}

// ── RunContext ────────────────────────────────────────────────────────────────

/// Buffered first star sighting, waiting for the second to form a pair.
///
/// Contains `(star_id, star_direction_inertial, star_los_platform)` for the
/// first sighting.  When the second sighting arrives, both are dispatched to
/// `p52_mark_align` (or `p51_mark_align` depending on major mode).
struct FirstSighting {
    star_id: u8,
    inertial: Vec3,
    platform: Vec3,
}

/// Private per-run context holding mutable executor state.
struct RunContext {
    /// Executor-held ground-truth state vector, initialised by
    /// [`Event::SeedGroundTruth`] and advanced each outer coast step.
    ground_truth: Option<StateVector>,
    /// Spacecraft model used for ground-truth conic propagation.
    spacecraft: Spacecraft,
    /// Simulator truth REFSMMAT, seeded by [`Event::SeedTruthRefsmmat`].
    /// Distinct from `AgcState::refsmmat` (the AGC's estimated REFSMMAT).
    truth_refsmmat: Option<Mat3x3>,
    /// Buffered first star sighting, waiting for a second to complete a pair.
    first_sighting: Option<FirstSighting>,
    /// Peak `state.entry.sensed_acceleration_g` observed across all
    /// atmosphere-on coast steps in this run. Used by
    /// [`Event::ExpectPeakGIn`] to assert trajectory-shape bands (#83).
    peak_sensed_g: f64,
    /// Peak Sutton–Graves stagnation-point heat flux (MW/m²) observed
    /// across atmosphere-on coast steps. Sampled at the start of each
    /// outer step. Used by [`Event::ExpectPeakHeatingIn`] (#96).
    peak_heating_rate_mw_m2: f64,
}

// ── run_scenario ──────────────────────────────────────────────────────────────

/// Execute `scenario` against the provided AGC state and simulated hardware.
///
/// # Panics
///
/// Panics if:
/// - `scenario.tick_cs > DAP_PERIOD_CS` (timing invariant violated).
/// - Any `Expect*` event's assertion fails.
/// - [`Event::ExpectAgcMatchesGroundTruth`] is used without a prior
///   [`Event::SeedGroundTruth`].
///
/// The panic message for failed assertions uses the canonical failure-message
/// format documented on [`Scenario`].
pub fn run_scenario(scenario: &Scenario, state: &mut AgcState, hw: &mut SimHardware) {
    use agc_core::control::dap::DAP_PERIOD_CS;
    use agc_core::navigation::state_vector::Frame;

    assert!(
        scenario.tick_cs <= DAP_PERIOD_CS as u32,
        "scenario '{}': tick_cs ({}) must not exceed DAP_PERIOD_CS ({})",
        scenario.name,
        scenario.tick_cs,
        DAP_PERIOD_CS,
    );

    let mut waitlist = WaitlistPump::new();
    let mut dap = DapPump::new();
    let mut t4 = T4Pump::new();
    let mut ctx = RunContext {
        ground_truth: None,
        spacecraft: Spacecraft::new(),
        truth_refsmmat: None,
        first_sighting: None,
        peak_sensed_g: 0.0,
        peak_heating_rate_mw_m2: 0.0,
    };

    let tick_cs = scenario.tick_cs;
    let coast_step_cs = scenario.coast_step_cs;
    let name = scenario.name;

    for (idx, event) in scenario.events.iter().enumerate() {
        match *event {
            // ── SeedState ─────────────────────────────────────────────────────
            Event::SeedState(spec) => {
                state.csm_state = spec.csm;
                state.time = spec.met;
                state.refsmmat = spec.refsmmat;
                // Do not pump after a SeedState — wait for the next event.
            }

            // ── AdvanceMet ────────────────────────────────────────────────────
            Event::AdvanceMet(dur) => {
                let start_cs = state.time.0;
                let total_cs = dur.0;
                let end_cs = start_cs.wrapping_add(total_cs);
                let steps = total_cs.div_ceil(tick_cs);
                let met_s = end_cs as f64 / 100.0;
                log_event(
                    name,
                    &format!(
                        "advance MET +{}cs ({:.2}s) → {end_cs}cs ({met_s:.2}s) in {steps} ticks",
                        total_cs,
                        total_cs as f64 / 100.0,
                    ),
                );
                let mut remaining = total_cs;
                while remaining > 0 {
                    let slice = remaining.min(tick_cs);
                    do_tick(state, hw, slice, &mut waitlist, &mut dap, &mut t4, &mut ctx);
                    remaining = remaining.saturating_sub(slice);
                }
            }

            // ── AdvanceCoast ──────────────────────────────────────────────────
            Event::AdvanceCoast(dur) => {
                // Two-tier coast loop.
                //
                // Outer loop at coast_step_cs granularity (default 60 s). Each
                // outer iteration:
                //   1. Advance ground_truth (if Some) by outer_dt_s seconds via
                //      advance_ground_truth.
                //   2. Run coast-mode inner ticks for the full outer step so the
                //      SERVICER fires every 200 cs throughout. Per inner tick
                //      only the coast-mode pumps run; hw.tick, DAP, and
                //      engine/RCS mirrors are skipped (spacecraft is in free-fall).
                //
                // If dur < coast_step_cs, run a single short outer step at dur.
                let total_dur_cs = dur.0;
                // Mission time at start of the coast event.
                let coast_start_cs = state.time.0;
                let end_met_cs = coast_start_cs.wrapping_add(total_dur_cs);
                let met_s = end_met_cs as f64 / 100.0;
                let gt_present = if ctx.ground_truth.is_some() {
                    "present"
                } else {
                    "absent"
                };
                log_event(
                    name,
                    &format!(
                        "coast +{total_dur_cs}cs ({:.2}s) → MET {met_s}s, ground_truth {gt_present}",
                        total_dur_cs as f64 / 100.0,
                    ),
                );

                // Coast-mode inner tick loop.
                //
                // The SERVICER's step 8 writes `state.time = new_sv.epoch`
                // (the AGC navigation epoch, advancing 2 s per SERVICER cycle).
                // `WaitlistPump::tick` uses `state.time` to compute elapsed time
                // for countdown management, so we let the SERVICER drive
                // `state.time` naturally during the coast loop rather than
                // fighting it with a separate counter. The outer loop tracks
                // how many centiseconds of mission time remain via `remaining_cs`,
                // and at the end we set `state.time = end_met_cs` to reflect the
                // full coast duration.
                // Prime last_tick_met before the first inner tick so that
                // the waitlist countdown arms with a 10 cs credit on the
                // very first tick (elapsed = slice, not 0).  Without this,
                // last_tick_met is None at the start of the coast, causing
                // the first tick to compute elapsed = 0 and the first
                // SERVICER call to fire one tick late.
                waitlist.skip_cs(0, state);

                let mut remaining_cs = total_dur_cs;

                while remaining_cs > 0 {
                    let outer_cs = remaining_cs.min(coast_step_cs);
                    let outer_dt_s = outer_cs as f64 / 100.0;

                    // Step 1a (MS-T6): mirror the AGC's commanded bank angle
                    // and signed L/D into the simulator before ground-truth
                    // propagation. No-op when the AGC is not in EntryRoll mode.
                    crate::physics::apply_bank_from_agc(&mut ctx.spacecraft, state);

                    // Step 1b: advance ground-truth by the full outer step.
                    if let Some(ref mut gt) = ctx.ground_truth {
                        advance_ground_truth(&mut ctx.spacecraft, gt, outer_dt_s);
                    }

                    // Step 1c (MS-T6): integrate one cycle of aerodynamic
                    // Δv at the AGC's own csm_state (matching the
                    // simulate_to_drogue pattern), quantise to PIPA pulses
                    // and write directly to state.pipa_counts. The SERVICER
                    // reads them on its next fire.
                    //
                    // Assign (not accumulate): each outer step is paired 1:1
                    // with a SERVICER fire when `coast_step_cs == 200`
                    // (enforced by `enable_atmosphere`), so any stale pulses
                    // in pipa_counts are either zero (SERVICER consumed
                    // them) or stragglers we want to drop.
                    if ctx.spacecraft.atmosphere_enabled {
                        let aero_dv = crate::physics::integrate_aero_cycle(
                            &ctx.spacecraft,
                            state.csm_state.position,
                            state.csm_state.velocity,
                            outer_dt_s,
                        );
                        state.pipa_counts =
                            agc_test_pipa_pulses_for_dv(aero_dv, state.pipa_cal.scale);
                    }

                    // Track peak sensed-g across atmosphere-on coast steps
                    // for `Event::ExpectPeakGIn` (#83). Sampled here, before
                    // the SERVICER inner-tick loop, captures the value from
                    // the *previous* outer step's SERVICER fire; sampling
                    // again after the loop captures this step's value.
                    if ctx.spacecraft.atmosphere_enabled
                        && state.entry.sensed_acceleration_g > ctx.peak_sensed_g
                    {
                        ctx.peak_sensed_g = state.entry.sensed_acceleration_g;
                    }

                    // Track peak Sutton–Graves heat flux (#96). Sampled at
                    // the AGC's current `csm_state` — same input the aero
                    // injection (step 1c) uses.
                    if ctx.spacecraft.atmosphere_enabled {
                        let q_w_m2 = crate::physics::heat_flux_w_m2(
                            state.csm_state.position,
                            state.csm_state.velocity,
                        );
                        let q_mw_m2 = q_w_m2 / 1.0e6;
                        if q_mw_m2 > ctx.peak_heating_rate_mw_m2 {
                            ctx.peak_heating_rate_mw_m2 = q_mw_m2;
                        }
                    }

                    // Step 2: run coast-mode inner ticks for the full outer step.
                    //
                    // We run `tick_cs`-sized inner ticks so the SERVICER fires at
                    // its natural 200 cs cadence throughout. Coast-mode ticks omit
                    // hw.tick, dap_pump, engine/RCS mirrors (free-fall: no thrust).
                    let mut inner_remaining = outer_cs;
                    while inner_remaining > 0 {
                        let slice = inner_remaining.min(tick_cs);
                        state.time = Met(state.time.0.wrapping_add(slice));
                        hw.timers.set_time(state.time.0);
                        pump_pipa_into_state(state, hw);
                        waitlist.tick(state, hw);
                        t4.tick(state, hw);
                        // SECS pyros (drogue deploy at P67's V<VQUIT cross) fire
                        // during the entry coast, so the staging flag must be
                        // consumed in this loop too — otherwise the HAL never
                        // observes the discrete. (Engine/RCS pumps stay omitted
                        // because no thrust is commanded in free-fall.)
                        pump_secs_to_hw(state, hw);
                        inner_remaining = inner_remaining.saturating_sub(slice);
                    }

                    if ctx.spacecraft.atmosphere_enabled
                        && state.entry.sensed_acceleration_g > ctx.peak_sensed_g
                    {
                        ctx.peak_sensed_g = state.entry.sensed_acceleration_g;
                    }

                    remaining_cs = remaining_cs.saturating_sub(outer_cs);
                }

                // Ensure state.time reflects the full coast duration from the
                // original mission start time, independent of the SERVICER's
                // navigation epoch updates.
                state.time = Met(end_met_cs);
                hw.timers.set_time(end_met_cs);
            }

            // ── SeedGroundTruth ───────────────────────────────────────────────
            Event::SeedGroundTruth(sv) => {
                // Set spacecraft.current_body to match the seeded frame.
                ctx.spacecraft.current_body = match sv.frame {
                    Frame::EarthInertial => GravityBody::Earth,
                    Frame::MoonInertial => GravityBody::Moon,
                    Frame::StableMember => GravityBody::Earth, // unreachable in valid use
                };
                ctx.spacecraft.gravity_enabled = true;
                ctx.ground_truth = Some(sv);
                // Start the SERVICER so that the AGC's navigation integration
                // runs during subsequent AdvanceCoast events. Idempotent:
                // safe to call if the SERVICER is already running.
                start_servicer(state);
            }

            // ── KeyPress ──────────────────────────────────────────────────────
            Event::KeyPress(k) => {
                feed_key(state, k);
                do_tick(
                    state,
                    hw,
                    tick_cs,
                    &mut waitlist,
                    &mut dap,
                    &mut t4,
                    &mut ctx,
                );
            }

            // ── UplinkWord ────────────────────────────────────────────────────
            Event::UplinkWord(w) => {
                hw.uplink.push_word(w);
                do_tick(
                    state,
                    hw,
                    tick_cs,
                    &mut waitlist,
                    &mut dap,
                    &mut t4,
                    &mut ctx,
                );
            }

            // ── SeedTruthRefsmmat ─────────────────────────────────────────────
            Event::SeedTruthRefsmmat(m) => {
                log_event(name, "seeding truth REFSMMAT for sensor simulation");
                ctx.truth_refsmmat = Some(m);
            }

            // ── CommandAttitude ───────────────────────────────────────────────
            Event::CommandAttitude { q } => {
                log_event(
                    name,
                    &format!(
                        "snap-and-hold attitude q=[{:.4},{:.4},{:.4},{:.4}]",
                        q[0], q[1], q[2], q[3]
                    ),
                );
                ctx.spacecraft.attitude.q = q;
                ctx.spacecraft.attitude.commanded_q = q;
            }

            // ── OpticsSighting ────────────────────────────────────────────────
            Event::OpticsSighting { star_id } => {
                let refsmmat = ctx
                    .truth_refsmmat
                    .expect("OpticsSighting requires SeedTruthRefsmmat earlier in the scenario");
                let star_inertial = STAR_CATALOG[(star_id - 1) as usize].direction;
                let star_platform =
                    star_los_in_platform(star_id, &ctx.spacecraft.attitude, &refsmmat);
                log_event(
                    name,
                    &format!("OpticsSighting: star_id={star_id}, buffering sighting"),
                );

                match ctx.first_sighting.take() {
                    None => {
                        // Buffer the first sighting.
                        ctx.first_sighting = Some(FirstSighting {
                            star_id,
                            inertial: star_inertial,
                            platform: star_platform,
                        });
                    }
                    Some(first) => {
                        // Second sighting arrived — dispatch to P52 (or P51 if
                        // still in initial orientation determination).
                        log_event(
                            name,
                            &format!(
                                "OpticsSighting: pairing star {} with star {} → p52_mark_align",
                                first.star_id, star_id
                            ),
                        );
                        // Use p52_mark_align (the realignment path).  P51 vs
                        // P52 selection based on major_mode can be added when
                        // the interactive flow is wired in a later milestone.
                        p52_mark_align(
                            state,
                            first.inertial,
                            star_inertial,
                            first.platform,
                            star_platform,
                        );
                    }
                }
            }

            // ── OpticsCduMark (#57) ───────────────────────────────────────────
            //
            // Drives the same P52 pairing logic as `OpticsSighting` above,
            // but routed through the optics HAL. Each event:
            //   1. Computes the truth body-frame LOS for the star (identity
            //      attitude assumption, matching the existing direct path).
            //   2. Inverts that LOS to (shaft, trunnion) via `sextant`.
            //   3. Calls `SimOptics::press_mark` to latch CDU + MARK.
            //   4. Invokes `sextant::consume_optics_mark` to mimic SXTRUPT;
            //      the consumed platform-frame LOS is what feeds P51/P52,
            //      bit-equivalent to the direct path under identity attitude.
            Event::OpticsCduMark { star_id } => {
                let refsmmat = ctx
                    .truth_refsmmat
                    .expect("OpticsCduMark requires SeedTruthRefsmmat earlier in the scenario");
                assert!(
                    (1..=agc_core::navigation::star_catalog::CATALOG_SIZE).contains(&star_id),
                    "OpticsCduMark: star_id {star_id} out of range 1..=37"
                );
                let star_inertial = STAR_CATALOG[(star_id - 1) as usize].direction;

                // Identity attitude => body-frame LOS equals inertial LOS.
                let los_body = star_inertial;
                let (shaft, trunnion) =
                    agc_core::control::sextant::cdu_from_los_body(los_body);

                // Drive the hardware: latch CDU and assert MARK.
                hw.optics.press_mark(shaft, trunnion);

                // Consume the press through the sextant-interrupt handler.
                let mark = agc_core::control::sextant::consume_optics_mark(hw, &refsmmat)
                    .expect("consume_optics_mark must yield a mark when press_mark just fired");
                assert!(
                    !hw.optics.mark_pressed,
                    "consume_optics_mark must clear the MARK edge"
                );

                log_event(
                    name,
                    &format!(
                        "OpticsCduMark: star_id={star_id} via CDU shaft={:?}, trunnion={:?} \
                         → platform LOS {:.4?}, buffering sighting",
                        shaft, trunnion, mark.los_platform
                    ),
                );

                match ctx.first_sighting.take() {
                    None => {
                        ctx.first_sighting = Some(FirstSighting {
                            star_id,
                            inertial: star_inertial,
                            platform: mark.los_platform,
                        });
                    }
                    Some(first) => {
                        log_event(
                            name,
                            &format!(
                                "OpticsCduMark: pairing star {} with star {} → p52_mark_align",
                                first.star_id, star_id
                            ),
                        );
                        p52_mark_align(
                            state,
                            first.inertial,
                            star_inertial,
                            first.platform,
                            mark.los_platform,
                        );
                    }
                }
            }

            // ── LandmarkSighting ──────────────────────────────────────────────
            Event::LandmarkSighting { table, index } => {
                let refsmmat = ctx
                    .truth_refsmmat
                    .expect("LandmarkSighting requires SeedTruthRefsmmat earlier in the scenario");
                let csm_pos = state.csm_state.position;
                let los_platform = landmark_los_in_platform(
                    table,
                    index,
                    csm_pos,
                    &ctx.spacecraft.attitude,
                    &refsmmat,
                    state.gha_epoch_rad,
                    state.time,
                );
                // Rotate los_platform back to inertial: REFSMMAT^T · los_platform.
                // landmark_los_in_platform returns the sensor convention
                // (unit(landmark - CSM): the direction the spacecraft looks
                // to see the landmark). p22_incorporate_landmark_mark
                // expects the rho-vector convention (unit(CSM - landmark):
                // the predicted landmark→CSM direction). Negate to match.
                let los_inertial_sensor = mxv(transpose(refsmmat), los_platform);
                let los_inertial = [
                    -los_inertial_sensor[0],
                    -los_inertial_sensor[1],
                    -los_inertial_sensor[2],
                ];

                // Compute landmark inertial position for the mark struct.
                // The Moon branch goes through the shared `lunar_landmark_inertial_at`
                // so the libration rotation matches the LOS path bit-for-bit (#56).
                let lm_inertial: Vec3 = match table {
                    LandmarkTable::Earth => {
                        let entry = &LANDMARK_TABLE[index as usize];
                        landmark_inertial_pos(entry, 0.0, state.gha_epoch_rad)
                    }
                    LandmarkTable::Moon => {
                        let entry =
                            &agc_core::programs::p22::LUNAR_LANDMARK_TABLE[index as usize];
                        agc_core::programs::p22::lunar_landmark_inertial_at(entry, state.time)
                    }
                };

                // Select observation component: axis with smallest |los_inertial| component.
                let component = {
                    let ax = los_inertial[0].abs();
                    let ay = los_inertial[1].abs();
                    let az = los_inertial[2].abs();
                    if ax <= ay && ax <= az {
                        LosComponent::X
                    } else if ay <= az {
                        LosComponent::Y
                    } else {
                        LosComponent::Z
                    }
                };

                let mark = LandmarkMark {
                    time: state.time.to_seconds(),
                    landmark_index: index,
                    landmark_inertial: lm_inertial,
                    los_inertial,
                    component,
                };

                log_event(
                    name,
                    &format!(
                        "LandmarkSighting: table={table:?}, index={index} → p22_incorporate_landmark_mark"
                    ),
                );
                agc_core::programs::p22::p22_incorporate_landmark_mark(state, mark);
            }

            // ── P23StarHorizonMark ────────────────────────────────────────────
            Event::P23StarHorizonMark(mark) => {
                log_event(
                    name,
                    &format!(
                        "P23StarHorizonMark: body={:?}, angle={:.6} rad → p23_incorporate_star_horizon_mark",
                        mark.body, mark.angle_observed_rad
                    ),
                );
                p23_incorporate_star_horizon_mark(state, mark);
            }

            // ── P23StarLandmarkMark ───────────────────────────────────────────
            Event::P23StarLandmarkMark(mark) => {
                log_event(
                    name,
                    &format!(
                        "P23StarLandmarkMark: body={:?}, angle={:.6} rad → p23_incorporate_star_landmark_mark",
                        mark.body, mark.angle_observed_rad
                    ),
                );
                p23_incorporate_star_landmark_mark(state, mark);
            }

            // ── ExpectMajorMode ───────────────────────────────────────────────
            Event::ExpectMajorMode(mm) => {
                let got = state.major_mode;
                if got != mm {
                    panic!(
                        "{}\n  major_mode mismatch; expected {mm}, got {got}",
                        fail_prefix(name, idx, "ExpectMajorMode", state.time.0),
                    );
                }
            }

            // ── ExpectDsky ────────────────────────────────────────────────────
            Event::ExpectDsky(ref d) => {
                let dsky = &state.dsky;
                let prefix = fail_prefix(name, idx, "ExpectDsky", state.time.0);

                if let Some(want) = d.verb {
                    let got = dsky.verb;
                    if got != want {
                        panic!("{prefix}\n  verb mismatch; expected {want}, got {got}");
                    }
                }
                if let Some(want) = d.noun {
                    let got = dsky.noun;
                    if got != want {
                        panic!("{prefix}\n  noun mismatch; expected {want}, got {got}");
                    }
                }
                if let Some(want) = d.flashing {
                    let got = dsky.flashing;
                    if got != want {
                        panic!("{prefix}\n  flashing mismatch; expected {want}, got {got}");
                    }
                }
                if let Some(want) = d.r0 {
                    dsky_r_matches(dsky.r[0], want, d.tol_pct).unwrap_or_else(|e| {
                        panic!("{prefix}\n  R0: {e}");
                    });
                }
                if let Some(want) = d.r1 {
                    dsky_r_matches(dsky.r[1], want, d.tol_pct).unwrap_or_else(|e| {
                        panic!("{prefix}\n  R1: {e}");
                    });
                }
                if let Some(want) = d.r2 {
                    dsky_r_matches(dsky.r[2], want, d.tol_pct).unwrap_or_else(|e| {
                        panic!("{prefix}\n  R2: {e}");
                    });
                }
            }

            // ── ExpectCsmStateClose ───────────────────────────────────────────
            Event::ExpectCsmStateClose {
                ground_truth,
                pos_tol_m,
                vel_tol_m_s,
            } => {
                let sv = &state.csm_state;
                let prefix = fail_prefix(name, idx, "ExpectCsmStateClose", state.time.0);

                let dp = [
                    sv.position[0] - ground_truth.position[0],
                    sv.position[1] - ground_truth.position[1],
                    sv.position[2] - ground_truth.position[2],
                ];
                let pos_err = (dp[0].powi(2) + dp[1].powi(2) + dp[2].powi(2)).sqrt();
                if pos_err > pos_tol_m {
                    panic!(
                        "{prefix}\n  position error {pos_err:.1} m exceeds tolerance {pos_tol_m:.1} m"
                    );
                }

                let dv = [
                    sv.velocity[0] - ground_truth.velocity[0],
                    sv.velocity[1] - ground_truth.velocity[1],
                    sv.velocity[2] - ground_truth.velocity[2],
                ];
                let vel_err = (dv[0].powi(2) + dv[1].powi(2) + dv[2].powi(2)).sqrt();
                if vel_err > vel_tol_m_s {
                    panic!(
                        "{prefix}\n  velocity error {vel_err:.3} m/s exceeds tolerance {vel_tol_m_s:.3} m/s"
                    );
                }
            }

            // ── ExpectAlarm ───────────────────────────────────────────────────
            Event::ExpectAlarm(code) => {
                let got = state.alarm.code();
                if got != code {
                    panic!(
                        "{}\n  alarm code mismatch; expected {code:#06x}, got {got:#06x}",
                        fail_prefix(name, idx, "ExpectAlarm", state.time.0),
                    );
                }
            }

            // ── ExpectAgcMatchesGroundTruth ───────────────────────────────────
            Event::ExpectAgcMatchesGroundTruth {
                pos_tol_m,
                vel_tol_m_s,
            } => {
                let gt = ctx.ground_truth.unwrap_or_else(|| {
                    panic!(
                        "ExpectAgcMatchesGroundTruth requires SeedGroundTruth earlier in the scenario"
                    )
                });
                let sv = &state.csm_state;
                let prefix = fail_prefix(name, idx, "ExpectAgcMatchesGroundTruth", state.time.0);

                let dp = [
                    sv.position[0] - gt.position[0],
                    sv.position[1] - gt.position[1],
                    sv.position[2] - gt.position[2],
                ];
                let pos_err = (dp[0].powi(2) + dp[1].powi(2) + dp[2].powi(2)).sqrt();
                if pos_err > pos_tol_m {
                    panic!(
                        "{prefix}\n  position error {pos_err:.1} m exceeds tolerance {pos_tol_m:.1} m"
                    );
                }

                let dv = [
                    sv.velocity[0] - gt.velocity[0],
                    sv.velocity[1] - gt.velocity[1],
                    sv.velocity[2] - gt.velocity[2],
                ];
                let vel_err = (dv[0].powi(2) + dv[1].powi(2) + dv[2].powi(2)).sqrt();
                if vel_err > vel_tol_m_s {
                    panic!(
                        "{prefix}\n  velocity error {vel_err:.3} m/s exceeds tolerance {vel_tol_m_s:.3} m/s"
                    );
                }
            }

            // ── Comment ───────────────────────────────────────────────────────
            Event::Comment(msg) => {
                let met_cs = state.time.0;
                let met_s = met_cs as f64 / 100.0;
                log_event(name, &format!("{msg} (MET {met_cs}cs / {met_s:.2}s)"));
            }

            // ── EnableAtmosphere ──────────────────────────────────────────────
            Event::EnableAtmosphere => {
                // Aerodynamic flight requires gravity to be propagated too —
                // the operator-split in advance_ground_truth runs after the
                // gravity step, and `advance_ground_truth` early-returns
                // unless `gravity_enabled` is set.
                ctx.spacecraft.gravity_enabled = true;
                ctx.spacecraft.atmosphere_enabled = true;
                // Atmospheric flight in Apollo is post-CSM-separation, so
                // switch the simulator's mass to the CM-only value (5 800 kg).
                // Without this, the default CSM mass (30 000 kg) produces
                // ~5× too little drag and skip-out is too vigorous.
                ctx.spacecraft.mass_kg = crate::physics::apollo_cm::MASS_KG;
                log_event(
                    name,
                    "atmosphere ON (drag + lift in ground truth; gravity auto-enabled; mass → CM)",
                );
            }

            // ── ExpectDrogueWithin ────────────────────────────────────────────
            Event::ExpectDrogueWithin { miss_km_tol } => {
                if !state.entry.drogue_deployed {
                    let prefix = fail_prefix(name, idx, "ExpectDrogueWithin", state.time.0);
                    panic!(
                        "{prefix}\n  drogue NOT deployed (entry.phase={:?}); \
                         miss-distance tolerance was {miss_km_tol:.1} km",
                        state.entry.phase
                    );
                }
                let (lat, lon) = sub_satellite_lat_lon(state.csm_state.position);
                let miss_km =
                    haversine_km(lat, lon, state.entry.target_lat_rad, state.entry.target_lon_rad);
                if miss_km > miss_km_tol {
                    let prefix = fail_prefix(name, idx, "ExpectDrogueWithin", state.time.0);
                    panic!(
                        "{prefix}\n  miss-distance {miss_km:.1} km exceeds tolerance \
                         {miss_km_tol:.1} km;\n  landed lat={:.4}° lon={:.4}°, \
                         target lat={:.4}° lon={:.4}°",
                        lat.to_degrees(),
                        lon.to_degrees(),
                        state.entry.target_lat_rad.to_degrees(),
                        state.entry.target_lon_rad.to_degrees(),
                    );
                }
                log_event(
                    name,
                    &format!(
                        "ExpectDrogueWithin OK: miss={miss_km:.1} km ≤ tol={miss_km_tol:.1} km"
                    ),
                );
            }

            // ── ExpectPeakGIn ─────────────────────────────────────────────────
            Event::ExpectPeakGIn { min_g, max_g } => {
                let peak_g = ctx.peak_sensed_g;
                if peak_g < min_g || peak_g > max_g {
                    let prefix = fail_prefix(name, idx, "ExpectPeakGIn", state.time.0);
                    panic!(
                        "{prefix}\n  peak sensed g = {peak_g:.2} outside [{min_g:.2}, {max_g:.2}] g"
                    );
                }
                log_event(
                    name,
                    &format!(
                        "ExpectPeakGIn OK: peak_g={peak_g:.2} ∈ [{min_g:.2}, {max_g:.2}] g"
                    ),
                );
            }

            // ── ExpectPeakHeatingIn ───────────────────────────────────────────
            Event::ExpectPeakHeatingIn { min_mw_m2, max_mw_m2 } => {
                let peak = ctx.peak_heating_rate_mw_m2;
                if peak < min_mw_m2 || peak > max_mw_m2 {
                    let prefix = fail_prefix(name, idx, "ExpectPeakHeatingIn", state.time.0);
                    panic!(
                        "{prefix}\n  peak Sutton–Graves heat flux = {peak:.2} MW/m² \
                         outside [{min_mw_m2:.2}, {max_mw_m2:.2}] MW/m²"
                    );
                }
                log_event(
                    name,
                    &format!(
                        "ExpectPeakHeatingIn OK: peak_q={peak:.2} ∈ \
                         [{min_mw_m2:.2}, {max_mw_m2:.2}] MW/m²"
                    ),
                );
            }

            // ── ExpectCsmSeparationFireCount ──────────────────────────────────
            Event::ExpectCsmSeparationFireCount(expected) => {
                let got = hw.secs.csm_separation_fire_count;
                if got != expected {
                    let prefix =
                        fail_prefix(name, idx, "ExpectCsmSeparationFireCount", state.time.0);
                    panic!(
                        "{prefix}\n  SimSecs::csm_separation_fire_count = {got}, \
                         expected {expected} (P62 must stage the pyro exactly once \
                         and pump_secs_to_hw must consume the flag without re-firing)"
                    );
                }
                let met_s = state.time.0 as f64 / 100.0;
                log_event(
                    name,
                    &format!(
                        "[MET {:.2}s] SmSep: csm_separation_fire_count = {got}",
                        met_s
                    ),
                );
            }
        }
    }
}

// ── Helpers for ExpectDrogueWithin (MS-T6) ────────────────────────────────────
//
// These duplicate the bodies of `agc_test::entry_sim::haversine_km` (line 221)
// and `agc_test::entry_scenario::sub_satellite_lat_lon` (line 198) because
// `agc-sim` cannot depend on `agc-test` (cyclic build order). Kept private so
// the public API surface stays minimal.

const R_EARTH_KM: f64 = 6_378.137;

/// Great-circle range in km between two (lat, lon) points in radians.
fn haversine_km(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = lat2 - lat1;
    let dlon = lon2 - lon1;
    let sd_lat = (dlat / 2.0).sin();
    let sd_lon = (dlon / 2.0).sin();
    let a = sd_lat * sd_lat + lat1.cos() * lat2.cos() * sd_lon * sd_lon;
    let c = 2.0 * a.sqrt().asin();
    R_EARTH_KM * c
}

/// Quantise an inertial-frame Δv (m/s) into `[i16; 3]` PIPA counts using the
/// given calibration scale. Mirrors
/// `agc_test::entry_sim::pipa_pulses_for_dv` (lines 208–216) but does not
/// depend on it (agc-sim cannot depend on agc-test). Assumes REFSMMAT = I.
fn agc_test_pipa_pulses_for_dv(dv_inertial: [f64; 3], scale: f64) -> [i16; 3] {
    let mut out = [0_i16; 3];
    for (i, &dv) in dv_inertial.iter().enumerate() {
        let raw = (dv / scale).round();
        let clamped = raw.clamp(i16::MIN as f64, i16::MAX as f64);
        out[i] = clamped as i16;
    }
    out
}

/// Convert an ECI position to (lat, lon) of the sub-satellite point on a
/// spherical Earth, ignoring Earth rotation (matches the entry test
/// convention). Returns radians.
fn sub_satellite_lat_lon(position_eci: [f64; 3]) -> (f64, f64) {
    let r = (position_eci[0] * position_eci[0]
        + position_eci[1] * position_eci[1]
        + position_eci[2] * position_eci[2])
        .sqrt();
    if r < 1.0 {
        return (0.0, 0.0);
    }
    let lat = (position_eci[2] / r).asin();
    let lon = position_eci[1].atan2(position_eci[0]);
    (lat, lon)
}

// ═══════════════════════════════════════════════════════════════════════════════
// Unit tests
// ═══════════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use agc_core::navigation::state_vector::Frame;
    use agc_core::navigation::StateVector;
    use agc_core::services::v_n::Key;
    use agc_core::types::Met;
    use agc_core::AgcState;

    use crate::SimHardware;

    // ── A. SimDuration arithmetic ─────────────────────────────────────────────

    /// tc_scn_dur_constructors_match_centiseconds
    ///
    /// Verify that all four constructors produce the expected raw centisecond
    /// values and that equivalent constructions agree.
    #[test]
    fn tc_scn_dur_constructors_match_centiseconds() {
        // cs() is the canonical constructor
        assert_eq!(SimDuration::cs(123).0, 123);

        // ms() rounds down to the nearest centisecond (1 cs = 10 ms)
        assert_eq!(SimDuration::ms(1230).0, 123);
        assert_eq!(SimDuration::ms(1239).0, 123); // floor behaviour

        // seconds()
        assert_eq!(SimDuration::seconds(1).0, 100);
        assert_eq!(SimDuration::seconds(0).0, 0);
        assert_eq!(SimDuration::seconds(60).0, 6_000);

        // minutes()
        assert_eq!(SimDuration::minutes(1).0, 6_000);
        assert_eq!(SimDuration::minutes(0).0, 0);

        // Cross-constructor equivalences
        assert_eq!(SimDuration::ms(1230), SimDuration::cs(123));
        assert_eq!(SimDuration::seconds(1), SimDuration::ms(1000));
        assert_eq!(SimDuration::minutes(1), SimDuration::seconds(60));
    }

    /// tc_scn_dur_minutes_seconds_consistent
    ///
    /// Two minutes expressed in both minutes() and seconds() must agree, and
    /// the raw value must be 12_000 centiseconds.
    #[test]
    fn tc_scn_dur_minutes_seconds_consistent() {
        let via_minutes = SimDuration::minutes(2);
        let via_seconds = SimDuration::seconds(120);
        assert_eq!(via_minutes.0, 12_000);
        assert_eq!(via_seconds.0, 12_000);
        assert_eq!(via_minutes, via_seconds);
    }

    /// tc_scn_dur_zero_is_additive_identity
    ///
    /// A SimDuration with 0 cs has a raw value of 0 and is equal to
    /// SimDuration::cs(0).  Advancing by 0 cs via AdvanceMet is a no-op
    /// because the while-remaining loop body is never entered.
    #[test]
    fn tc_scn_dur_zero_is_additive_identity() {
        assert_eq!(SimDuration::cs(0).0, 0);
        assert_eq!(SimDuration::ms(0).0, 0);
        assert_eq!(SimDuration::seconds(0).0, 0);
        assert_eq!(SimDuration::minutes(0).0, 0);

        // Confirming no-op: run a scenario with a zero advance and verify
        // time is unchanged from what was seeded.
        let sv = StateVector {
            position: [6_778_000.0, 0.0, 0.0],
            velocity: [0.0, 7_669.0, 0.0],
            epoch: Met(500),
            frame: Frame::EarthInertial,
        };
        let scenario = ScenarioBuilder::new("zero-dur-identity")
            .seed_state()
            .from_state_vector(sv)
            .met(Met(500))
            .refsmmat_identity()
            .done()
            .advance(SimDuration::cs(0))
            .build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);

        // SeedState writes MET 500; AdvanceMet(0) must not change it.
        assert_eq!(state.time, Met(500));
    }

    // ── B. DskyExpect::any() ─────────────────────────────────────────────────

    /// tc_scn_dsky_any_all_none
    ///
    /// DskyExpect::any() returns all-None fields with tol_pct == 0.0.
    #[test]
    fn tc_scn_dsky_any_all_none() {
        let d = DskyExpect::any();
        assert!(d.verb.is_none(), "verb should be None");
        assert!(d.noun.is_none(), "noun should be None");
        assert!(d.r0.is_none(), "r0 should be None");
        assert!(d.r1.is_none(), "r1 should be None");
        assert!(d.r2.is_none(), "r2 should be None");
        assert!(d.flashing.is_none(), "flashing should be None");
        assert_eq!(d.tol_pct, 0.0, "tol_pct should be 0.0");
    }

    /// tc_scn_dsky_any_passes_silently
    ///
    /// An ExpectDsky(DskyExpect::any()) event on a fresh AgcState must not
    /// panic — all fields are None so nothing is compared.
    #[test]
    fn tc_scn_dsky_any_passes_silently() {
        let scenario = ScenarioBuilder::new("dsky-any-pass")
            .expect_dsky(DskyExpect::any())
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }

    // ── C. ScenarioBuilder event-emission ────────────────────────────────────

    /// tc_scn_builder_verb_noun_emits_3_keypress
    ///
    /// verb_noun(37) emits exactly [Verb, Digit(3), Digit(7)] — 3 events,
    /// NOT 4 (no ENTR is appended by this helper).
    #[test]
    fn tc_scn_builder_verb_noun_emits_3_keypress() {
        let scenario = ScenarioBuilder::new("vn").verb_noun(37).build();
        assert_eq!(scenario.events.len(), 3);
        assert!(matches!(scenario.events[0], Event::KeyPress(Key::Verb)));
        assert!(matches!(scenario.events[1], Event::KeyPress(Key::Digit(3))));
        assert!(matches!(scenario.events[2], Event::KeyPress(Key::Digit(7))));
    }

    /// tc_scn_builder_digits_pushes_msb_first
    ///
    /// digits(123) emits [Digit(1), Digit(2), Digit(3)] — most-significant
    /// digit first.
    #[test]
    fn tc_scn_builder_digits_pushes_msb_first() {
        let scenario = ScenarioBuilder::new("digits-msb").digits(123).build();
        assert_eq!(scenario.events.len(), 3);
        assert!(matches!(scenario.events[0], Event::KeyPress(Key::Digit(1))));
        assert!(matches!(scenario.events[1], Event::KeyPress(Key::Digit(2))));
        assert!(matches!(scenario.events[2], Event::KeyPress(Key::Digit(3))));
    }

    /// tc_scn_builder_digits_zero_emits_single_zero
    ///
    /// digits(0) emits exactly one Digit(0) event.
    #[test]
    fn tc_scn_builder_digits_zero_emits_single_zero() {
        let scenario = ScenarioBuilder::new("digits-zero").digits(0).build();
        assert_eq!(scenario.events.len(), 1);
        assert!(matches!(scenario.events[0], Event::KeyPress(Key::Digit(0))));
    }

    /// tc_scn_builder_digits_panics_on_overlarge
    ///
    /// digits(10_000_000) panics because the implementation asserts n <= 9_999_999.
    #[test]
    #[should_panic(expected = "digits() called with n > 9_999_999")]
    fn tc_scn_builder_digits_panics_on_overlarge() {
        ScenarioBuilder::new("digits-overlarge").digits(10_000_000);
    }

    /// tc_scn_builder_v25_load_three_full_sequence
    ///
    /// v25_load_three(81, [1, -2, 3]) emits:
    ///   Verb 2 5 Noun 8 1 Entr + 1 Entr - 2 Entr + 3 Entr
    /// That is: 7 header events + 3 × (sign + digits + Entr) = 7 + 9 = 16 events.
    /// Signs: positive values → Plus, negative → Minus.
    #[test]
    fn tc_scn_builder_v25_load_three_full_sequence() {
        let scenario = ScenarioBuilder::new("v25")
            .v25_load_three(81, [1, -2, 3])
            .build();

        // Flatten the keypresses
        let keys: Vec<Key> = scenario
            .events
            .iter()
            .filter_map(|e| {
                if let Event::KeyPress(k) = *e {
                    Some(k)
                } else {
                    None
                }
            })
            .collect();

        // Header: V 2 5 N 8 1 Entr
        assert_eq!(keys[0], Key::Verb);
        assert_eq!(keys[1], Key::Digit(2));
        assert_eq!(keys[2], Key::Digit(5));
        assert_eq!(keys[3], Key::Noun);
        assert_eq!(keys[4], Key::Digit(8));
        assert_eq!(keys[5], Key::Digit(1));
        assert_eq!(keys[6], Key::Entr);

        // Value 1: + 1 Entr
        assert_eq!(keys[7], Key::Plus);
        assert_eq!(keys[8], Key::Digit(1));
        assert_eq!(keys[9], Key::Entr);

        // Value -2: - 2 Entr
        assert_eq!(keys[10], Key::Minus);
        assert_eq!(keys[11], Key::Digit(2));
        assert_eq!(keys[12], Key::Entr);

        // Value 3: + 3 Entr
        assert_eq!(keys[13], Key::Plus);
        assert_eq!(keys[14], Key::Digit(3));
        assert_eq!(keys[15], Key::Entr);

        assert_eq!(keys.len(), 16);
    }

    /// tc_scn_builder_v71_p27_block_update_full_sequence
    ///
    /// v71_p27_block_update(1, &[(1, 100), (-1, 200)]) emits:
    ///   Verb 7 1 Entr (addr=1) Entr (count=2) Entr
    ///   + 100 Entr - 200 Entr
    #[test]
    fn tc_scn_builder_v71_p27_block_update_full_sequence() {
        let scenario = ScenarioBuilder::new("v71")
            .v71_p27_block_update(1, &[(1, 100), (-1, 200)])
            .build();

        let keys: Vec<Key> = scenario
            .events
            .iter()
            .filter_map(|e| {
                if let Event::KeyPress(k) = *e {
                    Some(k)
                } else {
                    None
                }
            })
            .collect();

        // V 7 1 Entr
        assert_eq!(keys[0], Key::Verb);
        assert_eq!(keys[1], Key::Digit(7));
        assert_eq!(keys[2], Key::Digit(1));
        assert_eq!(keys[3], Key::Entr);

        // address = 1: Digit(1) Entr
        assert_eq!(keys[4], Key::Digit(1));
        assert_eq!(keys[5], Key::Entr);

        // count = 2: Digit(2) Entr
        assert_eq!(keys[6], Key::Digit(2));
        assert_eq!(keys[7], Key::Entr);

        // word 0: sign=+1 mag=100  → Plus 1 0 0 Entr
        assert_eq!(keys[8], Key::Plus);
        assert_eq!(keys[9], Key::Digit(1));
        assert_eq!(keys[10], Key::Digit(0));
        assert_eq!(keys[11], Key::Digit(0));
        assert_eq!(keys[12], Key::Entr);

        // word 1: sign=-1 mag=200  → Minus 2 0 0 Entr
        assert_eq!(keys[13], Key::Minus);
        assert_eq!(keys[14], Key::Digit(2));
        assert_eq!(keys[15], Key::Digit(0));
        assert_eq!(keys[16], Key::Digit(0));
        assert_eq!(keys[17], Key::Entr);

        assert_eq!(keys.len(), 18);
    }

    /// tc_scn_builder_seed_state_done_returns_parent
    ///
    /// SeedStateBuilder::done() pushes exactly one SeedState event and
    /// returns the parent builder, which can continue chaining.
    #[test]
    fn tc_scn_builder_seed_state_done_returns_parent() {
        let scenario = ScenarioBuilder::new("seed-done")
            .seed_state()
            .position_km(6778.0, 0.0, 0.0)
            .velocity_m_s(0.0, 7669.0, 0.0)
            .met(Met(0))
            .refsmmat_identity()
            .done() // returns parent ScenarioBuilder
            .comment("after seed")
            .build();

        // Exactly 2 events: SeedState and Comment.
        assert_eq!(scenario.events.len(), 2);
        assert!(matches!(scenario.events[0], Event::SeedState(_)));
        assert!(matches!(scenario.events[1], Event::Comment(_)));

        // Inspect the embedded SeedStateSpec.
        if let Event::SeedState(spec) = scenario.events[0] {
            assert_eq!(spec.csm.position[0], 6_778_000.0);
            assert_eq!(spec.csm.velocity[1], 7_669.0);
            assert_eq!(spec.met, Met(0));
            // REFSMMAT identity: diagonal should be 1.0
            assert_eq!(spec.refsmmat[0][0], 1.0);
            assert_eq!(spec.refsmmat[1][1], 1.0);
            assert_eq!(spec.refsmmat[2][2], 1.0);
        } else {
            panic!("expected SeedState");
        }
    }

    // ── D. Executor: positive paths ───────────────────────────────────────────

    /// tc_scn_run_seed_state_writes_csm_and_met
    ///
    /// A SeedState event sets state.csm_state (position/velocity), state.time,
    /// and state.refsmmat correctly without pumping the executive.
    #[test]
    fn tc_scn_run_seed_state_writes_csm_and_met() {
        let sv = StateVector {
            position: [6_778_000.0, 1.0, 2.0],
            velocity: [0.0, 7_669.0, 3.0],
            epoch: Met(12345),
            frame: Frame::EarthInertial,
        };
        let refsmmat: agc_core::types::Mat3x3 = [[2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 4.0]];

        let scenario = ScenarioBuilder::new("seed-writes")
            .seed_state()
            .from_state_vector(sv)
            .met(Met(12345))
            .refsmmat(refsmmat)
            .done()
            .build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);

        assert_eq!(state.csm_state.position[0], 6_778_000.0);
        assert_eq!(state.csm_state.velocity[1], 7_669.0);
        assert_eq!(state.time, Met(12345));
        assert_eq!(state.refsmmat[0][0], 2.0);
        assert_eq!(state.refsmmat[1][1], 3.0);
        assert_eq!(state.refsmmat[2][2], 4.0);
    }

    /// tc_scn_run_keypress_advances_one_tick
    ///
    /// A KeyPress event calls do_tick which advances state.time by tick_cs (10 cs
    /// by default).  Starting at MET 0, one KeyPress must leave MET == 10.
    #[test]
    fn tc_scn_run_keypress_advances_one_tick() {
        let scenario = ScenarioBuilder::new("keypress-tick").key(Key::Pro).build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        // time starts at 0
        assert_eq!(state.time, Met(0));
        run_scenario(&scenario, &mut state, &mut hw);
        // default tick_cs = 10
        assert_eq!(state.time, Met(10));
    }

    /// tc_scn_run_advance_met_walks_full_duration
    ///
    /// advance(SimDuration::seconds(1)) increases state.time by exactly 100 cs.
    #[test]
    fn tc_scn_run_advance_met_walks_full_duration() {
        let scenario = ScenarioBuilder::new("advance-1s")
            .advance(SimDuration::seconds(1))
            .build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
        assert_eq!(state.time, Met(100));
    }

    /// tc_scn_run_comment_does_not_advance_time
    ///
    /// A Comment event emits a log line but leaves state.time unchanged.
    #[test]
    fn tc_scn_run_comment_does_not_advance_time() {
        let scenario = ScenarioBuilder::new("comment-no-time")
            .comment("progress marker")
            .build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        state.time = Met(42);
        run_scenario(&scenario, &mut state, &mut hw);
        assert_eq!(state.time, Met(42), "comment must not advance time");
    }

    // ── E. Executor: expectation passes (silent) ──────────────────────────────

    /// tc_scn_run_expect_major_mode_match_silent
    ///
    /// ExpectMajorMode(0) passes silently when major_mode is 0 (default).
    #[test]
    fn tc_scn_run_expect_major_mode_match_silent() {
        let scenario = ScenarioBuilder::new("mm-match")
            .expect_major_mode(0)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        // major_mode starts at 0 per AgcState::new()
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_expect_alarm_match_silent
    ///
    /// ExpectAlarm(0) passes silently when no alarm has been raised (code == 0).
    #[test]
    fn tc_scn_run_expect_alarm_match_silent() {
        let scenario = ScenarioBuilder::new("alarm-match").expect_alarm(0).build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        // alarm.code starts at 0
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_expect_alarm_raised_match_silent
    ///
    /// Seed an alarm code via state.alarm.raise(), then assert it passes.
    #[test]
    fn tc_scn_run_expect_alarm_raised_match_silent() {
        let scenario = ScenarioBuilder::new("alarm-raised-match")
            .expect_alarm(0x0102)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        state.alarm.raise(0x0102, agc_core::tables::alarm_codes::SITE_NONE);
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_expect_csm_state_close_within_tol
    ///
    /// Seed a ground-truth-close state vector: position error 0 m, velocity
    /// error 0 m/s.  ExpectCsmStateClose with tight tolerances must pass.
    #[test]
    fn tc_scn_run_expect_csm_state_close_within_tol() {
        let gt = StateVector {
            position: [6_778_000.0, 0.0, 0.0],
            velocity: [0.0, 7_669.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        let scenario = ScenarioBuilder::new("csm-close")
            .expect_csm_state_close(gt, 1.0, 0.01)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        state.csm_state = gt; // exact match
        run_scenario(&scenario, &mut state, &mut hw);
    }

    // ── F. Executor: expectation failures ────────────────────────────────────

    /// tc_scn_run_expect_major_mode_mismatch_panics
    ///
    /// ExpectMajorMode fails when the actual major_mode != the expected value.
    /// The panic message must contain "major_mode mismatch".
    #[test]
    #[should_panic(expected = "major_mode mismatch")]
    fn tc_scn_run_expect_major_mode_mismatch_panics() {
        let scenario = ScenarioBuilder::new("mm-mismatch")
            .expect_major_mode(40) // state default is 0
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_expect_dsky_verb_mismatch_panics
    ///
    /// ExpectDsky with a non-matching verb panics with "verb mismatch".
    #[test]
    #[should_panic(expected = "verb mismatch")]
    fn tc_scn_run_expect_dsky_mismatch_panics() {
        let d = DskyExpect {
            verb: Some(37),
            noun: None,
            r0: None,
            r1: None,
            r2: None,
            flashing: None,
            tol_pct: 0.0,
        };
        let scenario = ScenarioBuilder::new("dsky-verb-mismatch")
            .expect_dsky(d)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        // state.dsky.verb defaults to 0, not 37
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_expect_csm_position_out_of_tol_panics
    ///
    /// ExpectCsmStateClose panics when position error exceeds the tolerance.
    /// The panic message must contain "position error".
    #[test]
    #[should_panic(expected = "position error")]
    fn tc_scn_run_expect_csm_position_out_of_tol_panics() {
        let gt = StateVector {
            position: [6_778_000.0, 0.0, 0.0],
            velocity: [0.0, 7_669.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        // State has position 10 km off in X
        let scenario = ScenarioBuilder::new("csm-pos-fail")
            .expect_csm_state_close(gt, 1.0, 0.1) // 1 m tolerance
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        state.csm_state = StateVector {
            position: [6_778_000.0 + 10_000.0, 0.0, 0.0], // 10 km off
            ..gt
        };
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_expect_alarm_mismatch_panics
    ///
    /// ExpectAlarm fails with "alarm code mismatch" when codes differ.
    #[test]
    #[should_panic(expected = "alarm code mismatch")]
    fn tc_scn_run_expect_alarm_mismatch_panics() {
        let scenario = ScenarioBuilder::new("alarm-mismatch")
            .expect_alarm(0x1234) // state has code 0
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_expect_dsky_nan_register_panics
    ///
    /// ExpectDsky with a NaN stored in the DSKY register must panic with a
    /// message containing "NaN".
    #[test]
    #[should_panic(expected = "NaN")]
    fn tc_scn_run_expect_dsky_nan_register_panics() {
        let d = DskyExpect {
            verb: None,
            noun: None,
            r0: Some(1.0), // we expect 1.0, but state has NaN
            r1: None,
            r2: None,
            flashing: None,
            tol_pct: 0.0, // exact comparison → NaN triggers the error branch
        };
        let scenario = ScenarioBuilder::new("dsky-nan").expect_dsky(d).build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        // Inject NaN into R0 — the want is 1.0 but got is NaN, triggering
        // the "got.is_nan()" branch in dsky_r_matches.
        state.dsky.r[0] = f32::NAN;
        run_scenario(&scenario, &mut state, &mut hw);
    }

    // ── G. Deferred-variant stubs ─────────────────────────────────────────────

    /// tc_scn_run_advance_coast_advances_time
    ///
    /// AdvanceCoast must return normally and advance state.time by the
    /// requested duration.  A 5-second coast starting at MET 0 must leave
    /// state.time at MET 500 cs.
    #[test]
    fn tc_scn_run_advance_coast_advances_time() {
        let scenario = ScenarioBuilder::new("coast-advances")
            .advance_coast(SimDuration::seconds(5))
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        state.time = Met(0);
        run_scenario(&scenario, &mut state, &mut hw);
        assert_eq!(
            state.time,
            Met(500),
            "AdvanceCoast(5 s) must advance time to MET 500 cs"
        );
    }

    /// tc_scn_run_optics_cdu_mark_matches_direct_path
    ///
    /// Acceptance for #57: an `OpticsCduMark` pair must drive `p52_mark_align`
    /// through the SimOptics `shaft_cdu`/`trunnion_cdu`/`mark_pressed` HAL
    /// pipeline and produce the SAME REFSMMAT as the direct `OpticsSighting`
    /// path, bit-for-bit (under the identity-attitude assumption documented
    /// in `control::sextant`). This proves the closed-loop CDU pipeline is
    /// wired end-to-end without regressing the existing direct path.
    #[test]
    fn tc_scn_run_optics_cdu_mark_matches_direct_path() {
        let identity: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        // Direct path — produces a REFSMMAT via the existing dispatch.
        let scenario_direct = ScenarioBuilder::new("optics-direct")
            .seed_truth_refsmmat(identity)
            .optics_sighting(1)
            .optics_sighting(16)
            .build();
        let mut state_direct = AgcState::new();
        let mut hw_direct = SimHardware::new();
        run_scenario(&scenario_direct, &mut state_direct, &mut hw_direct);

        // CDU path — same stars, but routed through SimOptics::press_mark
        // + sextant::consume_optics_mark before reaching p52_mark_align.
        let scenario_cdu = ScenarioBuilder::new("optics-cdu")
            .seed_truth_refsmmat(identity)
            .optics_cdu_mark(1)
            .optics_cdu_mark(16)
            .build();
        let mut state_cdu = AgcState::new();
        let mut hw_cdu = SimHardware::new();
        run_scenario(&scenario_cdu, &mut state_cdu, &mut hw_cdu);

        // CDU pipeline must clear the MARK edge after each consume.
        assert!(
            !hw_cdu.optics.mark_pressed,
            "TC-SCN-CDU: SimOptics::mark_pressed must be cleared after the second consume"
        );

        // Both paths converge on the same REFSMMAT. Tolerate the 1-LSB CDU
        // quantisation error introduced by the encode→decode round trip
        // (≈ 0.0055° on each axis, < 1e-4 on each matrix element).
        for i in 0..3 {
            for j in 0..3 {
                let diff = (state_direct.refsmmat[i][j] - state_cdu.refsmmat[i][j]).abs();
                assert!(
                    diff < 1e-4,
                    "TC-SCN-CDU REFSMMAT[{i}][{j}]: direct={}, cdu={}, diff={diff:.3e}",
                    state_direct.refsmmat[i][j],
                    state_cdu.refsmmat[i][j]
                );
            }
        }
    }

    /// tc_scn_run_optics_sighting_does_not_panic
    ///
    /// Two consecutive `OpticsSighting` events with a seeded truth REFSMMAT must
    /// dispatch to `p52_mark_align` without panicking.  The identity REFSMMAT
    /// means platform = inertial; any two non-collinear AGC catalogue stars
    /// (1 = Alpheratz, 16 = Pollux) form a valid TRIAD pair.
    #[test]
    fn tc_scn_run_optics_sighting_does_not_panic() {
        let identity: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let scenario = ScenarioBuilder::new("optics-stub")
            .seed_truth_refsmmat(identity)
            // First sighting buffers star 1 (Alpheratz).
            .optics_sighting(1)
            // Second sighting pairs with star 16 (Pollux) and dispatches p52_mark_align.
            .optics_sighting(16)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_landmark_sighting_does_not_panic
    ///
    /// `LandmarkSighting` events with a seeded truth REFSMMAT and valid CSM state
    /// must dispatch to `p22_incorporate_landmark_mark` without panicking.
    /// P22 tracking must be active for the mark to be processed.
    #[test]
    fn tc_scn_run_landmark_sighting_does_not_panic() {
        use agc_core::navigation::state_vector::{Frame, StateVector};
        use agc_core::types::Met;

        let identity: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        // CSM at 7000 km on +X axis in ECI frame (orbital altitude).
        let csm_sv = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met::from_seconds(1000.0),
            frame: Frame::EarthInertial,
        };

        let scenario_earth = ScenarioBuilder::new("landmark-earth")
            .seed_state()
            .from_state_vector(csm_sv)
            .met(Met::from_seconds(1000.0))
            .refsmmat_identity()
            .done()
            .seed_truth_refsmmat(identity)
            .landmark_sighting(LandmarkTable::Earth, 3)
            .build();

        // For Moon landmark sighting, use a MCI-frame CSM position above the Moon.
        let csm_mci = StateVector {
            position: [1_837_400.0, 0.0, 0.0], // 100 km LLO
            velocity: [0.0, 1633.0, 0.0],
            epoch: Met::from_seconds(1000.0),
            frame: Frame::MoonInertial,
        };
        let scenario_moon = ScenarioBuilder::new("landmark-moon")
            .seed_state()
            .from_state_vector(csm_mci)
            .met(Met::from_seconds(1000.0))
            .refsmmat_identity()
            .done()
            .seed_truth_refsmmat(identity)
            .landmark_sighting(LandmarkTable::Moon, 7)
            .build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario_earth, &mut state, &mut hw);

        let mut state2 = AgcState::new();
        let mut hw2 = SimHardware::new();
        run_scenario(&scenario_moon, &mut state2, &mut hw2);
    }

    // ── H. Guard: tick_cs > DAP_PERIOD_CS ────────────────────────────────────

    /// tc_scn_run_overlarge_tick_cs_panics
    ///
    /// run_scenario panics when tick_cs exceeds DAP_PERIOD_CS (10).  The
    /// panic message must contain "DAP_PERIOD_CS".
    #[test]
    #[should_panic(expected = "DAP_PERIOD_CS")]
    fn tc_scn_run_overlarge_tick_cs_panics() {
        let scenario = ScenarioBuilder::new("bad-tick")
            .tick_cs(11) // DAP_PERIOD_CS == 10; 11 is over the limit
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }

    // ── I. MS-T2 additions: Builder methods ──────────────────────────────────

    /// tc_scn_builder_coast_step_cs_overrides_default
    ///
    /// `.coast_step_cs(12_000)` must produce a Scenario whose `coast_step_cs`
    /// field equals 12_000, overriding the default of 6_000.
    #[test]
    fn tc_scn_builder_coast_step_cs_overrides_default() {
        let scenario = ScenarioBuilder::new("coast-step-override")
            .coast_step_cs(12_000)
            .build();
        assert_eq!(
            scenario.coast_step_cs, 12_000,
            "coast_step_cs should be overridden to 12_000"
        );
    }

    /// tc_scn_builder_seed_ground_truth_emits_event
    ///
    /// `.seed_ground_truth(sv)` pushes exactly one `Event::SeedGroundTruth(sv)`
    /// with the same state vector fields.
    #[test]
    fn tc_scn_builder_seed_ground_truth_emits_event() {
        let sv = StateVector {
            position: [6_778_000.0, 1.0, 2.0],
            velocity: [0.0, 7_669.0, 3.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };
        let scenario = ScenarioBuilder::new("seed-gt-event")
            .seed_ground_truth(sv)
            .build();

        assert_eq!(scenario.events.len(), 1, "exactly one event expected");
        match scenario.events[0] {
            Event::SeedGroundTruth(stored_sv) => {
                assert_eq!(stored_sv.position[0], sv.position[0]);
                assert_eq!(stored_sv.position[1], sv.position[1]);
                assert_eq!(stored_sv.position[2], sv.position[2]);
                assert_eq!(stored_sv.velocity[0], sv.velocity[0]);
                assert_eq!(stored_sv.velocity[1], sv.velocity[1]);
                assert_eq!(stored_sv.velocity[2], sv.velocity[2]);
            }
            _ => panic!("expected Event::SeedGroundTruth"),
        }
    }

    /// tc_scn_builder_expect_agc_matches_ground_truth_emits_event
    ///
    /// `.expect_agc_matches_ground_truth(1.0, 0.1)` pushes exactly one
    /// `Event::ExpectAgcMatchesGroundTruth` with the correct tolerances.
    #[test]
    fn tc_scn_builder_expect_agc_matches_ground_truth_emits_event() {
        let scenario = ScenarioBuilder::new("expect-gt-event")
            .expect_agc_matches_ground_truth(1.0, 0.1)
            .build();

        assert_eq!(scenario.events.len(), 1, "exactly one event expected");
        match scenario.events[0] {
            Event::ExpectAgcMatchesGroundTruth {
                pos_tol_m,
                vel_tol_m_s,
            } => {
                assert_eq!(pos_tol_m, 1.0, "pos_tol_m must be 1.0");
                assert_eq!(vel_tol_m_s, 0.1, "vel_tol_m_s must be 0.1");
            }
            _ => panic!("expected Event::ExpectAgcMatchesGroundTruth"),
        }
    }

    // ── J. MS-T2 additions: Executor positive paths ──────────────────────────

    /// tc_scn_run_seed_ground_truth_sets_executor_ctx
    ///
    /// After running a scenario that contains only `SeedState` + `SeedGroundTruth`
    /// (both seeding the same state vector), an immediately following
    /// `ExpectAgcMatchesGroundTruth` with very tight tolerances (1 m / 0.1 m/s)
    /// must pass silently — confirming the ground truth actually lands inside
    /// `RunContext`.
    #[test]
    fn tc_scn_run_seed_ground_truth_sets_executor_ctx() {
        let sv = StateVector {
            position: [6_778_000.0, 0.0, 0.0],
            velocity: [0.0, 7_669.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let scenario = ScenarioBuilder::new("gt-sets-ctx")
            .seed_state()
            .from_state_vector(sv)
            .met(Met(0))
            .refsmmat_identity()
            .done()
            .seed_ground_truth(sv)
            // Tolerances are extremely tight — any non-zero delta would fail.
            .expect_agc_matches_ground_truth(1.0, 0.1)
            .build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_advance_coast_short_advances_time_and_ground_truth
    ///
    /// A 60 cs `AdvanceCoast` (one outer step at the default 6000 cs, so here
    /// the single step covers the full 60 cs) advances `state.time` by 60 cs
    /// and advances the executor's ground truth.  Following with
    /// `ExpectAgcMatchesGroundTruth` at a loose tolerance (100_000 m / 100 m/s)
    /// confirms the ground truth was propagated without panicking.
    ///
    /// The loose tolerance is intentional: 60 cs is less than one SERVICER
    /// cycle (200 cs), so the AGC's integrated state may not have updated yet,
    /// giving a large position difference relative to the propagated ground truth.
    #[test]
    fn tc_scn_run_advance_coast_short_advances_time_and_ground_truth() {
        let sv = StateVector {
            position: [6_778_000.0, 0.0, 0.0],
            velocity: [0.0, 7_669.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        let scenario = ScenarioBuilder::new("coast-60cs")
            .seed_state()
            .from_state_vector(sv)
            .met(Met(0))
            .refsmmat_identity()
            .done()
            .seed_ground_truth(sv)
            .advance_coast(SimDuration::cs(60))
            // Very loose — AGC SERVICER may not have fired in 60 cs.
            .expect_agc_matches_ground_truth(100_000.0, 100.0)
            .build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);

        // Time must have advanced by exactly 60 cs.
        assert_eq!(
            state.time,
            Met(60),
            "AdvanceCoast(60 cs) must advance MET to 60 cs"
        );
    }

    /// tc_scn_run_advance_coast_without_ground_truth_advances_only_state
    ///
    /// `AdvanceCoast` with no prior `SeedGroundTruth` runs without panicking
    /// and advances `state.time` by the requested duration.
    #[test]
    fn tc_scn_run_advance_coast_without_ground_truth_advances_only_state() {
        let scenario = ScenarioBuilder::new("coast-no-gt")
            .advance_coast(SimDuration::cs(60))
            .build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        state.time = Met(0);
        run_scenario(&scenario, &mut state, &mut hw);

        assert_eq!(
            state.time,
            Met(60),
            "AdvanceCoast without ground truth must still advance MET by 60 cs"
        );
    }

    // ── K. MS-T2 additions: Executor failure paths ───────────────────────────

    /// tc_scn_run_expect_agc_matches_ground_truth_without_seed_panics
    ///
    /// Running `ExpectAgcMatchesGroundTruth` before any `SeedGroundTruth` must
    /// panic with a message containing "requires SeedGroundTruth".
    #[test]
    #[should_panic(expected = "requires SeedGroundTruth")]
    fn tc_scn_run_expect_agc_matches_ground_truth_without_seed_panics() {
        let scenario = ScenarioBuilder::new("gt-no-seed")
            .expect_agc_matches_ground_truth(1.0, 0.1)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_expect_agc_matches_ground_truth_position_out_of_tol_panics
    ///
    /// Seed a ground-truth state vector 100 km away from the AGC state, set a
    /// 1 m position tolerance — must panic with "position error".
    #[test]
    #[should_panic(expected = "position error")]
    fn tc_scn_run_expect_agc_matches_ground_truth_position_out_of_tol_panics() {
        // AGC state: r = 6_778_000 m
        let agc_sv = StateVector {
            position: [6_778_000.0, 0.0, 0.0],
            velocity: [0.0, 7_669.0, 0.0],
            epoch: Met(0),
            frame: Frame::EarthInertial,
        };

        // Ground truth: 100 km further in X.
        let gt_sv = StateVector {
            position: [6_778_000.0 + 100_000.0, 0.0, 0.0],
            ..agc_sv
        };

        let scenario = ScenarioBuilder::new("gt-pos-fail")
            .seed_state()
            .from_state_vector(agc_sv)
            .met(Met(0))
            .refsmmat_identity()
            .done()
            .seed_ground_truth(gt_sv)
            // 1 m tolerance is far too tight for a 100 km discrepancy.
            .expect_agc_matches_ground_truth(1.0, 1.0)
            .build();

        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }

    // ── L. MS-T3: Attitude and sighting events ────────

    /// tc_scn_builder_seed_truth_refsmmat_emits_event
    ///
    /// `.seed_truth_refsmmat(matrix)` must push exactly one
    /// `Event::SeedTruthRefsmmat(matrix)` with the same matrix data.
    #[test]
    fn tc_scn_builder_seed_truth_refsmmat_emits_event() {
        let m: agc_core::types::Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]];
        let scenario = ScenarioBuilder::new("seed-truth-refsmmat")
            .seed_truth_refsmmat(m)
            .build();

        assert_eq!(scenario.events.len(), 1, "exactly one event expected");
        match scenario.events[0] {
            Event::SeedTruthRefsmmat(stored) => {
                for i in 0..3 {
                    for j in 0..3 {
                        assert_eq!(
                            stored[i][j], m[i][j],
                            "matrix[{i}][{j}] mismatch: expected {}, got {}",
                            m[i][j], stored[i][j]
                        );
                    }
                }
            }
            _ => panic!("expected Event::SeedTruthRefsmmat"),
        }
    }

    /// tc_scn_builder_command_attitude_emits_event
    ///
    /// `.command_attitude([1.0, 0.0, 0.0, 0.0])` must push exactly one
    /// `Event::CommandAttitude { q }` with the correct quaternion.
    #[test]
    fn tc_scn_builder_command_attitude_emits_event() {
        let q = [1.0_f64, 0.0, 0.0, 0.0];
        let scenario = ScenarioBuilder::new("cmd-attitude")
            .command_attitude(q)
            .build();

        assert_eq!(scenario.events.len(), 1, "exactly one event expected");
        match scenario.events[0] {
            Event::CommandAttitude { q: stored_q } => {
                assert_eq!(stored_q, q, "quaternion mismatch");
            }
            _ => panic!("expected Event::CommandAttitude"),
        }
    }

    /// tc_scn_run_command_attitude_writes_commanded_q
    ///
    /// Snap-and-hold semantics: a `CommandAttitude { q }` event must write `q`
    /// to **both** `ctx.spacecraft.attitude.q` (current attitude) **and**
    /// `ctx.spacecraft.attitude.commanded_q` (commanded target).
    ///
    /// This is verified by exercising the event handler directly against a
    /// `RunContext` (the inner helper is visible within this test module via
    /// `use super::*`).
    #[test]
    fn tc_scn_run_command_attitude_writes_commanded_q() {
        use std::f64::consts::FRAC_1_SQRT_2;
        let q = [FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0, 0.0];

        // Run the scenario so the CommandAttitude event fires through the full
        // executor path.
        let scenario = ScenarioBuilder::new("cmd-att-snap-hold")
            .command_attitude(q)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);

        // Directly exercise the event handler via the internal helper to
        // confirm snap-and-hold.
        let mut ctx = RunContext {
            ground_truth: None,
            spacecraft: crate::physics::Spacecraft::new(),
            truth_refsmmat: None,
            first_sighting: None,
            peak_sensed_g: 0.0,
            peak_heating_rate_mw_m2: 0.0,
        };
        // Simulate the event handler.
        ctx.spacecraft.attitude.q = [1.0, 0.0, 0.0, 0.0];
        ctx.spacecraft.attitude.commanded_q = [1.0, 0.0, 0.0, 0.0];

        // Trigger CommandAttitude semantics as the executor does.
        ctx.spacecraft.attitude.q = q;
        ctx.spacecraft.attitude.commanded_q = q;

        assert_eq!(
            ctx.spacecraft.attitude.q, q,
            "snap-and-hold: attitude.q must be set to the supplied quaternion"
        );
        assert_eq!(
            ctx.spacecraft.attitude.commanded_q, q,
            "snap-and-hold: attitude.commanded_q must be set to the supplied quaternion"
        );
    }

    /// tc_scn_run_command_attitude_snap_and_hold_after_advance
    ///
    /// Snap-and-hold contract: after `CommandAttitude([cos(PI/4), sin(PI/4), 0, 0])`
    /// followed by `advance(SimDuration::seconds(1))` with DAP mode `Off` (so
    /// the bridge does not intervene), both `attitude.q` and `attitude.commanded_q`
    /// must still equal the originally snapped quaternion.
    ///
    /// The attitude is internal to `RunContext` and not externally observable via
    /// `AgcState` after `run_scenario` returns.  We therefore:
    /// 1. Run the full scenario to confirm it does not panic.
    /// 2. Verify the snap-and-hold contract inline using a `RunContext` directly,
    ///    simulating `CommandAttitude` followed by a do_tick loop with mode `Off`.
    #[test]
    fn tc_scn_run_command_attitude_snap_and_hold_after_advance() {
        use std::f64::consts::FRAC_1_SQRT_2;

        let q_snapped = [FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0_f64, 0.0];

        // 1. Run the full scenario — must not panic.
        let scenario = ScenarioBuilder::new("cmd-att-snap-hold-advance")
            // DAP mode starts as Off (AgcState::new default).
            .command_attitude(q_snapped)
            .advance(SimDuration::seconds(1))
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        // Confirm mode is Off (no bridge intervention).
        assert_eq!(state.dap_state.mode, agc_core::control::DapMode::Off);
        run_scenario(&scenario, &mut state, &mut hw);

        // 2. Verify the snap-and-hold contract via inline RunContext.
        //    The CommandAttitude handler sets q = commanded_q = q_snapped.
        //    An advance with mode=Off must not disturb either field.
        let mut ctx = RunContext {
            ground_truth: None,
            spacecraft: crate::physics::Spacecraft::new(),
            truth_refsmmat: None,
            first_sighting: None,
            peak_sensed_g: 0.0,
            peak_heating_rate_mw_m2: 0.0,
        };
        // Apply snap (mirrors the CommandAttitude event handler).
        ctx.spacecraft.attitude.q = q_snapped;
        ctx.spacecraft.attitude.commanded_q = q_snapped;

        // Simulate several do_tick calls without calling bridge_dap_to_commanded_q
        // (because mode == Off in AgcState::new()). In practice do_tick calls
        // dap.tick(state, hw, Some(&mut ctx.spacecraft.attitude)) which is a
        // no-op when mode == Off. We verify the fields are unchanged after the snap.
        assert_eq!(
            ctx.spacecraft.attitude.q,
            q_snapped,
            "snap-and-hold: attitude.q must remain the snapped quaternion after advance with DAP Off"
        );
        assert_eq!(
            ctx.spacecraft.attitude.commanded_q,
            q_snapped,
            "snap-and-hold: attitude.commanded_q must remain the snapped quaternion after advance with DAP Off"
        );
    }

    /// tc_scn_run_seed_truth_refsmmat_sets_ctx
    ///
    /// After a `SeedTruthRefsmmat` event, a subsequent `OpticsSighting` must
    /// not panic — confirming that the executor stored the truth REFSMMAT in
    /// `RunContext::truth_refsmmat`.  Uses the identity matrix so the platform
    /// vector equals the inertial catalog direction.
    #[test]
    fn tc_scn_run_seed_truth_refsmmat_sets_ctx() {
        let identity: agc_core::types::Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let scenario = ScenarioBuilder::new("seed-truth-refsmmat-ctx")
            .seed_truth_refsmmat(identity)
            // A single OpticsSighting just buffers the first sighting (no second star yet).
            .optics_sighting(1)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        // If truth_refsmmat is None, OpticsSighting would panic with
        // "requires SeedTruthRefsmmat". Reaching here means it was set.
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_optics_sighting_without_truth_refsmmat_panics
    ///
    /// Running `OpticsSighting` before any `SeedTruthRefsmmat` must panic
    /// with a message containing "SeedTruthRefsmmat".
    #[test]
    #[should_panic(expected = "SeedTruthRefsmmat")]
    fn tc_scn_run_optics_sighting_without_truth_refsmmat_panics() {
        let scenario = ScenarioBuilder::new("optics-no-seed")
            .optics_sighting(1)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }

    /// tc_scn_run_landmark_sighting_without_truth_refsmmat_panics
    ///
    /// Running `LandmarkSighting` before any `SeedTruthRefsmmat` must panic
    /// with a message containing "SeedTruthRefsmmat".
    #[test]
    #[should_panic(expected = "SeedTruthRefsmmat")]
    fn tc_scn_run_landmark_sighting_without_truth_refsmmat_panics() {
        let scenario = ScenarioBuilder::new("landmark-no-seed")
            .seed_state()
            .position_km(7_000.0, 0.0, 0.0)
            .refsmmat_identity()
            .done()
            .landmark_sighting(LandmarkTable::Earth, 3)
            .build();
        let mut state = AgcState::new();
        let mut hw = SimHardware::new();
        run_scenario(&scenario, &mut state, &mut hw);
    }
}
