// SPDX-License-Identifier: GPL-3.0-or-later
//! T4RUPT handler — periodic I/O at ~120 ms intervals.
//!
//! Cycles through: DSKY display update, IMU status monitoring,
//! uplink processing, telemetry downlink list output.
//!
//! On the bare-metal target the executive's main loop drives the
//! T4-pending tasks inline (see `executive::scheduler::Executive::run`).
//! [`t4rupt_step`] is the host-side / unit-testable entry point that the
//! agc-sim [`T4Pump`](../../../agc_sim/runtime/struct.T4Pump.html) calls
//! at the 120 ms cadence so scripted uplink, future downlink frames, and
//! similar periodic I/O can be exercised without a full executive.
//!
//! ## Downlink cadence
//!
//! The AGC's DOWNRUPT fires every 20 ms (50 Hz); the T4RUPT fires every
//! 120 ms (≈ 8.33 Hz).  To approximate the 20 ms cadence without a
//! dedicated DOWNRUPT timer, `t4rupt_step` calls `downlink_step` six times
//! per T4 tick (6 × 20 ms = 120 ms), evenly advancing the 2-second cycle.

use crate::hal::AgcHardware;
use crate::services::downlink::downlink_step;
use crate::services::lamps::refresh_lamps;
use crate::services::uplink::poll_uplink;
use crate::AgcState;

/// Number of MSFN downlink pairs to drain per T4RUPT tick.
///
/// T4RUPT period = 120 ms; DOWNRUPT period = 20 ms → 6 downrupts per T4 tick.
const DOWNRUPTS_PER_T4: usize = 6;

/// One T4RUPT tick.
///
/// Performs (in order):
/// - Uplink drain ([`poll_uplink`]) — UPRUPT path from HAL to V/N. Any
///   keystrokes delivered here update `state.dsky` and bump
///   `state.pinball_ticks` (via `feed_key` → `note_pinball_activity`).
/// - [`refresh_lamps`] — recompute every indicator-lamp boolean so the
///   DSKY frame that the host scheduler decodes next reflects the
///   AGC's current internal condition. Must run before any
///   `decode_dsky` call in the same tick.
/// - MSFN downlink: 6 word-pairs emitted via `hw.telemetry()`.
///
/// AGC source: `Comanche055/DOWN-TELEMETRY_PROGRAM.agc` — DODOWNTM driven by
/// DOWNRUPT every 20 ms.
pub fn t4rupt_step<H: AgcHardware>(state: &mut AgcState, hw: &mut H) {
    poll_uplink(state, hw.uplink());

    // Refresh indicator lamps before the DSKY frame is decoded by the
    // scheduler (bare-metal) or the sim's display path. Placed after
    // poll_uplink so any uplink-driven V/N state transition is already
    // reflected in `key_rel` / `comp_acty`.
    refresh_lamps(state);

    // Drain 6 downlink word-pairs (≈ 120 ms worth of DOWNRUPT output).
    for _ in 0..DOWNRUPTS_PER_T4 {
        // Split the borrow: copy driver out, call step, write back.
        let mut driver = state.downlink;
        downlink_step(&mut driver, state, hw.telemetry());
        state.downlink = driver;
    }
}
