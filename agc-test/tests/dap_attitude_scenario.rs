// Spec: specs/control-dap.md §TC-DAP-*, specs/control-attitude.md §TC-ATT-*
//
// Integration tests for the Digital AutoPilot (DAP) attitude control loop.
// Tests exercise the full DAP tick cycle:
//   AttitudeTarget → t5rupt_tick → JetDecision → JetCommand
//
// The MockHw simulator stub uses the HAL implementation types.
// No global alarm state is touched; parallel execution is safe (no #[serial]).

use agc_core::{
    control::{
        attitude::{compute_error, phase_plane_decision, JetDecision},
        constants::DEADBAND_DEFAULT_RAD,
        dap::{set_mode, t5rupt_tick, Dap, DapMode},
        rcs_logic::select_jets,
    },
    hal::{
        dsky::{DigitRow, DskyIo, Key, RelayWord},
        engine::EngineImpl,
        imu::{ImuImpl, Unaligned},
        optics::OpticsImpl,
        rcs::{RcsImpl, RcsIo},
        telemetry::TelemetryImpl,
        timers::TimersImpl,
        uplink::UplinkImpl,
        AgcHardware,
    },
    types::{CduAngle, IDENTITY_MAT3},
};

// ── Minimal DSKY stub ─────────────────────────────────────────────────────────

struct StubDsky;

impl DskyIo for StubDsky {
    fn read_key(&mut self) -> Option<Key> {
        None
    }
    fn read_nav_key(&mut self) -> Option<Key> {
        None
    }
    fn write_relay(&mut self, _word: RelayWord) {}
    fn write_lamp_word(&mut self, _bits: u16) {}
    fn write_prog(&mut self, _prog: u8) {}
    fn write_verb(&mut self, _verb: u8) {}
    fn write_noun(&mut self, _noun: u8) {}
    fn write_register(&mut self, _row: usize, _value: &DigitRow) {}
    fn proceed_pressed(&self) -> bool {
        false
    }
    fn set_prog_light(&mut self, _on: bool) {}
}

// ── Full mock hardware ────────────────────────────────────────────────────────

struct MockHw {
    imu: ImuImpl<Unaligned>,
    rcs: RcsImpl,
    engine: EngineImpl,
    timers: TimersImpl,
    dsky: StubDsky,
    optics: OpticsImpl,
    uplink: UplinkImpl,
    telemetry: TelemetryImpl,
}

impl MockHw {
    fn new() -> Self {
        Self {
            imu: ImuImpl::new(),
            rcs: RcsImpl::new(),
            engine: EngineImpl::new(),
            timers: TimersImpl::new(),
            dsky: StubDsky,
            optics: OpticsImpl::new(),
            uplink: UplinkImpl::new(),
            telemetry: TelemetryImpl::new(),
        }
    }
}

impl AgcHardware for MockHw {
    type Timers = TimersImpl;
    type Dsky = StubDsky;
    type Imu = ImuImpl<Unaligned>;
    type Optics = OpticsImpl;
    type Engine = EngineImpl;
    type Rcs = RcsImpl;
    type Uplink = UplinkImpl;
    type Telemetry = TelemetryImpl;

    fn timers(&mut self) -> &mut Self::Timers {
        &mut self.timers
    }
    fn dsky(&mut self) -> &mut Self::Dsky {
        &mut self.dsky
    }
    fn imu(&mut self) -> &mut Self::Imu {
        &mut self.imu
    }
    fn optics(&mut self) -> &mut Self::Optics {
        &mut self.optics
    }
    fn engine(&mut self) -> &mut Self::Engine {
        &mut self.engine
    }
    fn rcs(&mut self) -> &mut Self::Rcs {
        &mut self.rcs
    }
    fn uplink(&mut self) -> &mut Self::Uplink {
        &mut self.uplink
    }
    fn telemetry(&mut self) -> &mut Self::Telemetry {
        &mut self.telemetry
    }
    fn pet_watchdog(&mut self) {}
    fn hardware_restart(&mut self) -> ! {
        loop {}
    }
}

// ── TC-DAP-01: Idle mode never fires jets ─────────────────────────────────────

/// When the DAP is in Idle mode, t5rupt_tick must not command any jets.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc DAPIDLER (page 1019).
#[test]
fn idle_mode_fires_no_jets() {
    let mut dap = Dap::default();
    let mut hw = MockHw::new();

    // Ensure Idle mode
    set_mode(&mut dap, DapMode::Idle, &mut hw);

    // Tick several times
    for _ in 0..5 {
        t5rupt_tick(&mut dap, &mut hw);
    }

    // No jet commands should have been issued
    let cmd = hw.rcs().current_command();
    assert_eq!(cmd.pitch_yaw, 0, "Idle mode must not fire pitch/yaw jets");
    assert_eq!(cmd.roll, 0, "Idle mode must not fire roll jets");
}

// ── TC-DAP-02: Phase-plane on-axis deadband ───────────────────────────────────

/// phase_plane_decision returns None when both error and rate are within deadband.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc PHPLANE (page 1030).
#[test]
fn phase_plane_within_deadband_no_jets() {
    let deadband = DEADBAND_DEFAULT_RAD;
    // Error = 0.001 rad (well within 0.3° = 0.005236 rad deadband)
    // Rate  = 0.0 rad/s
    let decision = phase_plane_decision(0.001, 0.0, deadband);
    assert_eq!(
        decision,
        JetDecision::None,
        "Small error within deadband must give None"
    );

    // Error = 0.0, rate = 0.001 rad/s (well within deadband)
    let decision2 = phase_plane_decision(0.0, 0.001, deadband);
    assert_eq!(
        decision2,
        JetDecision::None,
        "Small rate within deadband must give None"
    );
}

// ── TC-DAP-03: Phase-plane outside deadband fires jet ─────────────────────────

/// phase_plane_decision returns Positive/Negative outside deadband.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc PHPLANE (page 1030).
#[test]
fn phase_plane_outside_deadband_fires_jet() {
    let deadband = DEADBAND_DEFAULT_RAD;

    // Large positive error → Positive decision
    let decision_pos = phase_plane_decision(0.1, 0.0, deadband);
    assert_eq!(
        decision_pos,
        JetDecision::Positive,
        "Large positive error must give Positive"
    );

    // Large negative error → Negative decision
    let decision_neg = phase_plane_decision(-0.1, 0.0, deadband);
    assert_eq!(
        decision_neg,
        JetDecision::Negative,
        "Large negative error must give Negative"
    );
}

// ── TC-DAP-04: RCS jet selection, all-None gives OFF command ─────────────────

/// select_jets(None, None, None) must produce the OFF command (both channels zero).
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc JETSLECT (page 1039).
#[test]
fn jet_select_all_none_gives_off() {
    let cmd = select_jets(JetDecision::None, JetDecision::None, JetDecision::None);
    assert_eq!(cmd.pitch_yaw, 0, "All-None pitch_yaw must be 0");
    assert_eq!(cmd.roll, 0, "All-None roll must be 0");
}

// ── TC-DAP-05: compute_error with identity matrix and zero CDUs gives zero error ──

/// compute_error with the identity REFSMMAT and zero CDU angles must return
/// AttitudeError::ZERO (no correction needed when already at the target attitude).
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc MERUPDAT (p. 1020).
#[test]
fn compute_error_identity_gives_zero() {
    let cdus = [CduAngle::from_counts(0); 3];
    let desired = IDENTITY_MAT3;
    let err = compute_error(&cdus, &desired);
    assert!(
        err.pitch.abs() < 1e-9,
        "pitch error must be ~0 for identity REFSMMAT, got {}",
        err.pitch
    );
    assert!(
        err.yaw.abs() < 1e-9,
        "yaw error must be ~0 for identity REFSMMAT, got {}",
        err.yaw
    );
    assert!(
        err.roll.abs() < 1e-9,
        "roll error must be ~0 for identity REFSMMAT, got {}",
        err.roll
    );
}
