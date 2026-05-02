//! Remote SPS engine: commands forwarded over the link; thrust-on discrete
//! read from `BridgeState` cache.

use agc_core::hal::engine::Engine;
use agc_protocol::Msg;

use crate::with_bridge_and_link;

/// Zero-sized HAL implementation for the remote SPS engine.
pub struct RemoteEngine;

impl Engine for RemoteEngine {
    fn sps_enable(&mut self, on: bool) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::EngineSpsEnable { on: on as u8 }, seq);
        });
    }

    fn sps_gimbal(&mut self, pitch: i16, yaw: i16) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::EngineSpsGimbal { pitch, yaw }, seq);
        });
    }

    fn thrust_on(&self) -> bool {
        cortex_m::interrupt::free(|cs| crate::BRIDGE.borrow(cs).borrow().engine_thrust_on)
    }
}
