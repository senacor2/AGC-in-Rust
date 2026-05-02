//! Remote telemetry downlink: words forwarded over the link.

use agc_core::hal::telemetry::Telemetry;
use agc_protocol::Msg;

use crate::with_bridge_and_link;

/// Zero-sized HAL implementation for the remote telemetry downlink.
pub struct RemoteTelemetry;

impl Telemetry for RemoteTelemetry {
    fn send_word(&mut self, word: u16) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::TelemetryWord { word }, seq);
        });
    }
}
