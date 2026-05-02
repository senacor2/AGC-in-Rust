//! Remote RCS: jet fire/quench commands forwarded over the link.

use agc_core::hal::rcs::Rcs;
use agc_protocol::Msg;

use crate::with_bridge_and_link;

/// Zero-sized HAL implementation for the remote RCS.
pub struct RemoteRcs;

impl Rcs for RemoteRcs {
    fn fire_sm_jets(&mut self, jets_a: u8, jets_b: u8) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::RcsFireSm { jets_a, jets_b }, seq);
        });
    }

    fn fire_cm_jets(&mut self, jets: u16) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::RcsFireCm { jets }, seq);
        });
    }

    fn quench_all(&mut self) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::RcsQuenchAll, seq);
        });
    }
}
