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

use crate::hal::AgcHardware;
use crate::services::uplink::poll_uplink;
use crate::AgcState;

/// One T4RUPT tick.
///
/// Currently performs:
/// - Uplink drain ([`poll_uplink`]) — UPRUPT path from HAL to V/N.
///
/// To be added in later milestones: DSKY frame emit (parity with the
/// executive inline path), downlink list output.
pub fn t4rupt_step<H: AgcHardware>(state: &mut AgcState, hw: &mut H) {
    poll_uplink(state, hw.uplink());
}
