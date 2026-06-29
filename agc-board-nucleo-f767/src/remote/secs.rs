// SPDX-License-Identifier: GPL-3.0-or-later
//! Remote SECS: pyrotechnic discretes forwarded over the link.

use agc_core::hal::secs::Secs;
use agc_protocol::Msg;

use crate::with_bridge_and_link;

/// Zero-sized HAL implementation for the remote SECS pyro driver.
pub struct RemoteSecs;

impl Secs for RemoteSecs {
    fn deploy_drogue(&mut self) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::SecsDeployDrogue, seq);
        });
    }

    fn fire_csm_separation(&mut self) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::SecsFireCsmSeparation, seq);
        });
    }
}
