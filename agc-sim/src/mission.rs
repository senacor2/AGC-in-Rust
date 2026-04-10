//! Mission state tracker: MET, active scenario, scenario scripts.
//!
//! The mission module drives the presentation narrative. Each scenario
//! initialises the state vector, active program, and any scripted events.

use agc_core::navigation::gravity::MU_EARTH;
use agc_core::navigation::state_vector::{Frame, StateVector};
use agc_core::types::Met;

/// Demo scenarios.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scenario {
    /// F1: Launch monitor — P11 with live V06N62 display.
    LaunchMonitor,
    /// F2: Targeted burn — P00 → P30 target → P40 burn → P00.
    TargetedBurn,
    /// F3: Free flight — 200 km LEO, crew drives the DSKY.
    FreeFlight,
}

impl Scenario {
    pub fn label(self) -> &'static str {
        match self {
            Self::LaunchMonitor => "Launch Monitor",
            Self::TargetedBurn => "Targeted Burn",
            Self::FreeFlight => "Free Flight",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "launch" | "launch_monitor" | "f1" => Some(Self::LaunchMonitor),
            "burn" | "targeted_burn" | "f2" => Some(Self::TargetedBurn),
            "free" | "free_flight" | "f3" => Some(Self::FreeFlight),
            _ => None,
        }
    }
}

/// Mission state — one per running simulation.
pub struct Mission {
    pub scenario: Scenario,
    pub sv: StateVector,
    /// Active program number (0, 11, 30, 40, etc).
    pub active_prog: u8,
    /// Current DSKY verb (for auto-load when scenario starts).
    pub default_verb: u8,
    /// Current DSKY noun (for auto-load when scenario starts).
    pub default_noun: u8,
    /// Orbital period in seconds (derived from initial sv).
    pub period_s: f64,
}

impl Mission {
    /// Create a fresh mission for the given scenario.
    pub fn new(scenario: Scenario) -> Self {
        match scenario {
            Scenario::LaunchMonitor => {
                // Simulated post-insertion state: 185 km parking orbit.
                // At this altitude v_circ = sqrt(mu / (R + 185km))
                let r0 = 6_556_000.0_f64; // RE_EARTH + 185 km
                let v = (MU_EARTH / r0).sqrt();
                let sv = StateVector {
                    frame: Frame::Eci,
                    r: [r0, 0.0, 0.0],
                    v: [0.0, v, 0.0],
                    t: Met::ZERO,
                };
                let period = 2.0 * std::f64::consts::PI * r0 / v;
                Self {
                    scenario,
                    sv,
                    active_prog: 11,
                    default_verb: 6,
                    default_noun: 62,
                    period_s: period,
                }
            }
            Scenario::TargetedBurn | Scenario::FreeFlight => {
                // 200 km circular LEO
                let r0 = 6_578_000.0_f64;
                let v = (MU_EARTH / r0).sqrt();
                let sv = StateVector {
                    frame: Frame::Eci,
                    r: [r0, 0.0, 0.0],
                    v: [0.0, v, 0.0],
                    t: Met::ZERO,
                };
                let period = 2.0 * std::f64::consts::PI * r0 / v;
                let (prog, verb, noun) = match scenario {
                    Scenario::TargetedBurn => (0, 0, 0),
                    _ => (0, 37, 0),
                };
                Self {
                    scenario,
                    sv,
                    active_prog: prog,
                    default_verb: verb,
                    default_noun: noun,
                    period_s: period,
                }
            }
        }
    }

    /// Format MET as "+hhh:mm:ss.ss"
    pub fn format_met(&self) -> String {
        let cs = self.sv.t.0;
        let total_s = cs / 100;
        let h = total_s / 3600;
        let m = (total_s / 60) % 60;
        let s = total_s % 60;
        let frac = cs % 100;
        format!("+{:03}:{:02}:{:02}.{:02}", h, m, s, frac)
    }

    /// Format phase label per active program.
    pub fn phase_label(&self) -> &'static str {
        match self.active_prog {
            0 => "P00 — CMC Idle",
            11 => "P11 — Earth Orbit Insertion Monitor",
            30 => "P30 — External Delta-V Targeting",
            37 => "P37 — Return to Earth",
            40 => "P40 — SPS Thrust",
            41 => "P41 — RCS Thrust",
            51 => "P51 — IMU Orientation Determination",
            52 => "P52 — IMU Realign",
            _ => "UNKNOWN",
        }
    }
}
