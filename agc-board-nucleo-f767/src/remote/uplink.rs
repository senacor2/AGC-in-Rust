//! Remote uplink receiver: words are cached in `BridgeState.uplink_queue`.

use agc_core::hal::uplink::Uplink;

/// Zero-sized HAL implementation for the remote uplink receiver.
pub struct RemoteUplink;

impl Uplink for RemoteUplink {
    fn read_word(&mut self) -> Option<u16> {
        cortex_m::interrupt::free(|cs| {
            crate::BRIDGE
                .borrow(cs)
                .borrow_mut()
                .uplink_queue
                .pop_front()
        })
    }
}
