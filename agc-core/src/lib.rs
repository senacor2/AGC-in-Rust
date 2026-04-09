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

use control::{DapState, TvcState};
use executive::{Executive, RestartProtection, Waitlist};
use navigation::StateVector;
use services::{AlarmState, DskyState, average_g::PipaCalibration};
use types::{Mat3x3, Met};

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
    /// P40 SPS burns set this to `guidance::tvc::cross_product_steering_update`.
    /// P00 and programs that do not need a SERVICER exit leave this as `None`.
    pub servicer_exit: Option<fn(&mut AgcState)>,
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
                deadband: 0.0,
                rcs_jet_flags: 0,
            },
            tvc_state: TvcState {
                gimbal_pitch: 0.0,
                gimbal_yaw: 0.0,
                trim_pitch: 0,
                trim_yaw: 0,
            },
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
        }
    }
}
