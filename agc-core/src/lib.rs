//! AGC-in-Rust — Comanche055 (Command Module) guidance software.
//!
//! This crate is `no_std` (no heap, no OS). All state lives in [`AgcState`].
//! The entry point for bare-metal hardware is in a separate firmware binary
//! that provides a concrete [`hal::AgcHardware`] implementation and calls
//! the [`services::fresh_start`] sequence.
//!
//! For host-side simulation and testing, use the `agc-sim` crate which
//! provides a software [`hal::AgcHardware`] implementation.

#![cfg_attr(not(test), no_std)]
#![deny(unsafe_op_in_unsafe_fn)]
#![allow(dead_code, unused_variables, unused_imports, clippy::upper_case_acronyms)]

pub mod control;
pub mod executive;
pub mod guidance;
pub mod hal;
pub mod math;
pub mod navigation;
pub mod programs;
pub mod services;
pub mod tables;
pub mod types;

use control::{DapState, TvcFilter, TvcState};
use control::imu_control::{GyroCompensation, ImuAlignmentState};
use control::rcs_logic::RcsConfig;
use executive::{Executive, RestartProtection, Waitlist};
use guidance::maneuver::BurnState;
use guidance::targeting::Maneuver;
use navigation::StateVector;
use programs::p20::RendezvousNavState;
use programs::p61_p67::EntryState;
use services::{AlarmState, DskyState, average_g::PipaCalibration, v_n::VnState};
use types::{CduAngle, Mat3x3, Met};

/// Central mutable state of the guidance computer.
///
/// Passed by `&mut` reference through the entire call hierarchy.
/// There is exactly one instance, allocated statically in the firmware binary.
/// The borrow checker enforces that no two subsystems mutate overlapping fields
/// simultaneously — a compile-time enforcement of the AGC's cooperative
/// scheduling discipline.
pub struct AgcState {
    // ── Scheduler ────────────────────────────────────────────────────────────
    pub executive: Executive,
    pub waitlist: Waitlist,
    pub restart: RestartProtection,

    // ── Navigation ───────────────────────────────────────────────────────────
    /// CSM position (m) and velocity (m/s) in the reference frame.
    pub csm_state: StateVector,
    /// Target vehicle state (LM for rendezvous, or landmark).
    pub target_state: StateVector,
    /// Reference-to-Stable-Member matrix (IMU orientation in inertial frame).
    pub refsmmat: Mat3x3,
    /// Mission elapsed time (centiseconds).
    pub time: Met,

    // ── Guidance and control ─────────────────────────────────────────────────
    /// Currently active major mode (program number, 0–99).
    pub major_mode: u8,
    pub dap_state: DapState,
    pub tvc_state: TvcState,

    /// TVC digital lead-lag filter state (pitch and yaw axes).
    ///
    /// Initialised to nominal Comanche055 coefficients with zeroed memories
    /// by `tvc_init` (called from P40 before ignition). Updated each T5RUPT
    /// cycle by `tvc_step`.
    pub tvc_filter: TvcFilter,

    /// SPS burn execution state.
    ///
    /// Populated by `burn_init` when P40 enters the burn phase. Persists
    /// across Waitlist task boundaries for restart protection (Group 3).
    pub burn: BurnState,

    /// Maneuver computed by the most recently run targeting program (P30,
    /// P31, P34, or P37). Consumed by P40/P41 on entry, after transferring
    /// the maneuver into `BurnState` via `burn_init`.
    ///
    /// Set to `Some(maneuver)` by `p30_load_dv_lvlh` (and analogous functions
    /// in P31/P34/P37). Set to `None` by P40/P41 on entry.
    ///
    /// AGC correspondence: `DELVEET1/2/3` + `TIG` in E3 erasable.
    pub pending_maneuver: Option<Maneuver>,

    // ── IMU ──────────────────────────────────────────────────────────────────
    /// Current IMU platform alignment state.
    ///
    /// Tracks the Caged → CoarseAligned → FineAligned lifecycle.
    /// Read by the SERVICER to gate PIPA integration.
    pub imu_alignment_state: ImuAlignmentState,

    /// Gyro drift compensation constants (NBDX, NBDY, NBDZ).
    ///
    /// Applied by the T4RUPT handler each 120 ms to null platform drift.
    /// Updated by Mission Control uplink or P52 alignment.
    pub gyro_comp: GyroCompensation,

    /// Mission elapsed time of the last gyro drift compensation torque.
    ///
    /// Used by the T4RUPT handler to compute the elapsed interval since the
    /// previous compensation and scale the torque command accordingly.
    pub last_drift_comp_time: Met,

    // ── RCS ───────────────────────────────────────────────────────────────────
    /// RCS jet selection configuration (deadbands, pulse limits, jet counts).
    ///
    /// Loaded from crew V46 entries; defaults to `RcsConfig::NOMINAL` at
    /// FRESH START.
    pub rcs_config: RcsConfig,

    // ── Strategy-D staging fields ─────────────────────────────────────────────
    /// CDU angles staged by the T4/T5 ISR shim before dispatching DAP tasks.
    ///
    /// The ISR shim reads the three CDU channels (CDUX, CDUY, CDUZ) from
    /// hardware and writes them here. `dap_step` reads from this field rather
    /// than calling `hw.imu().read_cdu()` directly, keeping the Waitlist task
    /// signature `fn(&mut AgcState)`.
    pub current_cdu: [CduAngle; 3],

    /// RCS jet command staged by `dap_step` for the T5RUPT ISR shim.
    ///
    /// `dap_step` writes the `u16` jet bitmask from `rcs_logic::select_jets_sm`.
    /// After `dap_step` returns, the ISR shim reads this field and calls
    /// `fire_pulse(hw, jets, counts)`. Reset to `0` at the start of each cycle.
    /// Upper byte = jets_b (channel 06 / ROLLJETS), lower byte = jets_a (05 / PYJETS).
    pub rcs_commanded_jets: u16,

    /// RCS pulse duration staged by `dap_step` for the T5RUPT ISR shim.
    ///
    /// Units: T6 counts (1 count = 0.625 ms). `0` means no pulse this cycle.
    /// Written alongside `rcs_commanded_jets`.
    pub rcs_commanded_pulse_cs: u16,

    /// SPS gimbal command staged by `tvc_step` for the T5RUPT ISR shim.
    ///
    /// `(pitch_counts, yaw_counts)` in CDU error-counter units
    /// (3200 counts = 1 full revolution, ~0.001963 rad/count).
    /// The ISR shim passes these to `hw.engine().sps_gimbal(pitch, yaw)`.
    pub sps_gimbal_cmd: (i16, i16),

    /// Engine thrusting discrete staged by `dap_step` / P40 for the ISR shim.
    ///
    /// `true` while the SPS engine is commanded on. The ISR shim reads this
    /// to decide whether to issue gimbal commands or quench all jets.
    pub engine_thrusting: bool,

    // ── Crew interface ───────────────────────────────────────────────────────
    pub dsky: DskyState,

    // ── Alarms ───────────────────────────────────────────────────────────────
    pub alarm: AlarmState,

    // ── Flags ────────────────────────────────────────────────────────────────
    /// Bit-field flag words (FLAGWRD0–FLAGWRD11). Addressed by bit position,
    /// not by arithmetic value.
    pub flagwords: [u16; 12],

    // ── SERVICER / PIPA ──────────────────────────────────────────────────────
    /// PIPA (accelerometer) calibration constants: scale factor, bias, misalignment.
    /// Initialised to `PipaCalibration::NOMINAL` at FRESH START.
    pub pipa_cal: PipaCalibration,

    /// Raw PIPA counts staging field.
    ///
    /// Written by the T3RUPT handler (Strategy B) or hardware shim before
    /// dispatching `servicer_task`. Consumed by the SERVICER each 2-second cycle.
    /// Units: raw PIPA pulse counts (platform frame, destructive read).
    pub pipa_counts: [i16; 3],

    /// Optional program-specific callback invoked at the end of each SERVICER cycle.
    ///
    /// P40 SPS burns set this to `guidance::maneuver::burn_servicer_exit`.
    /// P00 and programs that do not need a SERVICER exit leave this as `None`.
    pub servicer_exit: Option<fn(&mut AgcState)>,

    /// Inertial-frame delta-V (m/s) integrated by the SERVICER during the
    /// most recent 2-second cycle. Populated by `servicer_task` immediately
    /// before invoking `servicer_exit`. Read by `burn_servicer_exit` during
    /// a P40/P41 burn to advance `BurnState.accumulated_dv_inertial`.
    pub servicer_last_dv_inertial: types::Vec3,

    // ── Rendezvous navigation (P20) ──────────────────────────────────────────
    /// State maintained by the P20 rendezvous navigation program.
    ///
    /// Populated by `programs::p20::p20_init` on entry to P20.
    /// Reset to `Default::default()` on FRESH START.
    pub rendezvous_nav: RendezvousNavState,

    // ── Entry guidance ───────────────────────────────────────────────────────
    /// Atmospheric entry state machine and stub guidance fields.
    ///
    /// Tracks the P61..P67 phase sequence, current sensed acceleration
    /// (g units, test-harness driven in MS4), stub roll command, target
    /// range, and drogue-deployed flag.
    pub entry: EntryState,

    // ── Verb/Noun processor ──────────────────────────────────────────────────
    /// Crew interface (DSKY) Verb/Noun input state machine.
    ///
    /// Updated by `services::v_n::feed_key` each keypress.
    pub vn: VnState,
}

impl AgcState {
    /// Construct a zeroed AgcState for use in FRESH START.
    pub const fn new() -> Self {
        Self {
            executive: Executive::new(),
            waitlist: Waitlist::new(),
            restart: RestartProtection::new(),
            csm_state: StateVector::ZERO,
            target_state: StateVector::ZERO,
            refsmmat: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            time: Met(0),
            major_mode: 0,
            dap_state: DapState {
                mode: control::DapMode::Off,
                attitude_error: [0.0; 3],
                rate_estimate: [0.0; 3],
                prev_cdu: [CduAngle(0); 3],
                deadband: 0.0,
                rate_deadband: 0.0,
                rcs_jet_flags: 0,
                failed_jets: 0,
                num_jets: 2,
                commanded_attitude: [0.0; 3],
                maneuver_rate: [0.0; 3],
                restart_phase: 0,
            },
            tvc_state: TvcState {
                gimbal_pitch: 0.0,
                gimbal_yaw: 0.0,
                trim_pitch: 0.0,
                trim_yaw: 0.0,
            },
            tvc_filter: TvcFilter::new_nominal(),
            burn: BurnState {
                target_dv_inertial: [0.0; 3],
                accumulated_dv_inertial: [0.0; 3],
                tig: Met(0),
                burn_active: false,
                cutoff_time_met: false,
            },
            pending_maneuver: None,
            imu_alignment_state: ImuAlignmentState::Caged,
            gyro_comp: GyroCompensation {
                nbdx: 0.0,
                nbdy: 0.0,
                nbdz: 0.0,
            },
            last_drift_comp_time: Met(0),
            rcs_config: RcsConfig::NOMINAL,
            current_cdu: [CduAngle(0); 3],
            rcs_commanded_jets: 0,
            rcs_commanded_pulse_cs: 0,
            sps_gimbal_cmd: (0, 0),
            engine_thrusting: false,
            dsky: DskyState {
                prog: 0,
                verb: 0,
                noun: 0,
                r: [0.0; 3],
                flashing: false,
                uplink_activity: false,
                no_att: false,
                stby: false,
                key_rel: false,
                opr_err: false,
                restart_flag: false,
                gimbal_lock: false,
                temp: false,
                prog_alarm: false,
                comp_acty: false,
                tracker: false,
                lamp_test_active: false,
            },
            alarm: AlarmState {
                code: 0,
                code2: 0,
                lit: false,
            },
            flagwords: [0u16; 12],
            pipa_cal: PipaCalibration::NOMINAL,
            pipa_counts: [0i16; 3],
            servicer_exit: None,
            servicer_last_dv_inertial: [0.0; 3],
            rendezvous_nav: RendezvousNavState {
                target_pos:              [0.0; 3],
                target_vel:              [0.0; 3],
                target_epoch:            0.0,
                w_matrix:                [[0.0; 6]; 6],
                last_mark_time:          0.0,
                mark_count:              0,
                reject_count:            0,
                consecutive_reject_count: 0,
                lvlh_state: crate::guidance::rendezvous::LvlhState {
                    rho:     [0.0; 3],
                    rho_dot: [0.0; 3],
                },
                tracking_active:         false,
            },
            entry: EntryState::new(),
            vn: VnState::new(),
        }
    }
}
