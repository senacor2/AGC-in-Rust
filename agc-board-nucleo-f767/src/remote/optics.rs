//! Remote optics: CDU angles and mark flag come from `BridgeState` cache;
//! drive commands are forwarded over the link.

use agc_core::hal::optics::Optics;
use agc_core::types::CduAngle;
use agc_protocol::Msg;

use crate::with_bridge_and_link;

/// Zero-sized HAL implementation for the remote optics.
pub struct RemoteOptics;

impl Optics for RemoteOptics {
    fn trunnion_angle(&self) -> CduAngle {
        cortex_m::interrupt::free(|cs| {
            CduAngle(crate::BRIDGE.borrow(cs).borrow().optics_cdu_trunnion)
        })
    }

    fn shaft_angle(&self) -> CduAngle {
        cortex_m::interrupt::free(|cs| CduAngle(crate::BRIDGE.borrow(cs).borrow().optics_cdu_shaft))
    }

    fn drive(&mut self, trunnion: i16, shaft: i16) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::OpticsDrive { trunnion, shaft }, seq);
        });
    }

    fn mark_pressed(&self) -> bool {
        // Sticky flag: cleared on first read so the Executive only sees each
        // mark event once.
        cortex_m::interrupt::free(|cs| {
            let mut b = crate::BRIDGE.borrow(cs).borrow_mut();
            let v = b.optics_mark_pending;
            b.optics_mark_pending = false;
            v
        })
    }
}
