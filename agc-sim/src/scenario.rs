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
//! is not detected.  The `AdvanceCoast` event in this module is a stub today
//! (see the log message in its dispatch arm).  A follow-up issue has been filed
//! to wire `soi_check` into the `AdvanceCoast` executor once MS-T2 lands.
//!
//! ## 3. REFSMMAT propagation after P52
//!
//! `p52_mark_align` in `agc-core/src/programs/p51_p52.rs` writes the new matrix
//! directly to `state.refsmmat`.  `servicer_task` reads `state.refsmmat` on
//! every cycle at step 6 (`mxv(state.refsmmat, delta_v_platform)`); there is no
//! caching or shadow copy.  Conclusion: **a new REFSMMAT written by P52 is
//! consumed by the very next SERVICER cycle — no stale-cache issue exists in the
//! current implementation.**

use agc_core::navigation::StateVector;
use agc_core::services::v_n::{feed_key, Key};
use agc_core::types::{Mat3x3, Met};
use agc_core::AgcState;

use crate::runtime::{
    pump_engine_to_hw, pump_pipa_into_state, pump_rcs_to_hw, DapPump, T4Pump, WaitlistPump,
};
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

    /// Advance mission time by `dur` without SERVICER integration.
    ///
    /// **Stub until MS-T2.** Logs a single line and continues; time is not
    /// advanced (the intent is to add `propagate_coast` integration later).
    AdvanceCoast(SimDuration),

    /// Deliver a single DSKY keypress to the V/N processor, then tick once.
    KeyPress(Key),

    /// Push a raw word onto the simulated uplink FIFO, then tick once.
    UplinkWord(u16),

    /// Log a stub "not-yet-implemented" sighting event and continue.
    OpticsSighting { star_id: u8 },

    /// Log a stub "not-yet-implemented" landmark sighting and continue.
    LandmarkSighting { table: LandmarkTable, index: u8 },

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

    /// Assert `state.alarm.code == code`.
    ExpectAlarm(u16),

    /// Emit a one-line progress message; no state change.
    Comment(&'static str),
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
}

impl ScenarioBuilder {
    /// Create a new builder for a scenario with the given name.
    pub fn new(name: &'static str) -> Self {
        Self {
            name,
            events: Vec::new(),
            tick_cs: 10, // default: 100 ms per tick
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

    /// Advance mission time as a coast phase (stub until MS-T2).
    pub fn advance_coast(mut self, dur: SimDuration) -> Self {
        self.events.push(Event::AdvanceCoast(dur));
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

    /// Push a landmark sighting event (stub).
    pub fn landmark_sighting(mut self, table: LandmarkTable, index: u8) -> Self {
        self.events.push(Event::LandmarkSighting { table, index });
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

    /// Assert `state.alarm.code == code`.
    pub fn expect_alarm(mut self, code: u16) -> Self {
        self.events.push(Event::ExpectAlarm(code));
        self
    }

    /// Emit a progress comment.
    pub fn comment(mut self, msg: &'static str) -> Self {
        self.events.push(Event::Comment(msg));
        self
    }

    /// Consume the builder and produce a [`Scenario`].
    pub fn build(self) -> Scenario {
        Scenario {
            name: self.name,
            events: self.events,
            tick_cs: self.tick_cs,
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
) {
    let tick_s = tick_cs as f64 / 100.0;
    state.time = Met(state.time.0.wrapping_add(tick_cs));
    hw.timers.set_time(state.time.0);
    hw.tick(tick_s);
    pump_pipa_into_state(state, hw);
    dap.tick(state, hw);
    waitlist.tick(state, hw);
    t4.tick(state, hw);
    pump_engine_to_hw(state, hw);
    pump_rcs_to_hw(state, hw);
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

// ── run_scenario ──────────────────────────────────────────────────────────────

/// Execute `scenario` against the provided AGC state and simulated hardware.
///
/// # Panics
///
/// Panics if:
/// - `scenario.tick_cs > DAP_PERIOD_CS` (timing invariant violated).
/// - Any `Expect*` event's assertion fails.
///
/// The panic message for failed assertions uses the canonical failure-message
/// format documented on [`Scenario`].
pub fn run_scenario(scenario: &Scenario, state: &mut AgcState, hw: &mut SimHardware) {
    use agc_core::control::dap::DAP_PERIOD_CS;

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

    let tick_cs = scenario.tick_cs;
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
                    do_tick(state, hw, slice, &mut waitlist, &mut dap, &mut t4);
                    remaining = remaining.saturating_sub(slice);
                }
            }

            // ── AdvanceCoast ──────────────────────────────────────────────────
            Event::AdvanceCoast(dur) => {
                // MS-T2 will wire propagate_coast / soi_check here.
                // Investigation §2 above explains why this is a stub today:
                // soi_check is implemented in navigation::integration but is not
                // yet called from the scenario executor.
                log_event(
                    name,
                    &format!("stub: AdvanceCoast({}cs) is a no-op until MS-T2", dur.0),
                );
            }

            // ── KeyPress ──────────────────────────────────────────────────────
            Event::KeyPress(k) => {
                feed_key(state, k);
                do_tick(state, hw, tick_cs, &mut waitlist, &mut dap, &mut t4);
            }

            // ── UplinkWord ────────────────────────────────────────────────────
            Event::UplinkWord(w) => {
                hw.uplink.push_word(w);
                do_tick(state, hw, tick_cs, &mut waitlist, &mut dap, &mut t4);
            }

            // ── OpticsSighting ────────────────────────────────────────────────
            Event::OpticsSighting { star_id } => {
                log_event(
                    name,
                    &format!("stub: OpticsSighting(star_id={star_id}) not yet implemented"),
                );
            }

            // ── LandmarkSighting ──────────────────────────────────────────────
            Event::LandmarkSighting { table, index } => {
                log_event(
                    name,
                    &format!(
                        "stub: LandmarkSighting(table={table:?}, index={index}) not yet implemented"
                    ),
                );
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
                let got = state.alarm.code;
                if got != code {
                    panic!(
                        "{}\n  alarm code mismatch; expected {code:#06x}, got {got:#06x}",
                        fail_prefix(name, idx, "ExpectAlarm", state.time.0),
                    );
                }
            }

            // ── Comment ───────────────────────────────────────────────────────
            Event::Comment(msg) => {
                let met_cs = state.time.0;
                let met_s = met_cs as f64 / 100.0;
                log_event(name, &format!("{msg} (MET {met_cs}cs / {met_s:.2}s)"));
            }
        }
    }
}
