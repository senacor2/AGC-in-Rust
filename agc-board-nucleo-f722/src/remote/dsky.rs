//! Remote DSKY: commands are encoded as `Msg::*` and sent over the UART link;
//! key reads drain the `BridgeState.key_queue`.

use agc_core::hal::dsky::{Dsky, Lamp};
use agc_protocol::Msg;

use crate::with_bridge_and_link;

/// Zero-sized HAL implementation for the remote DSKY.
pub struct RemoteDsky;

impl Dsky for RemoteDsky {
    fn write_row(&mut self, row: u8, data: u16) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::DskyWriteRow { row, data }, seq);
        });
    }

    fn clear_row(&mut self, row: u8) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::DskyClearRow { row }, seq);
        });
    }

    fn set_lamp(&mut self, lamp: Lamp, on: bool) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(
                &Msg::DskySetLamp {
                    lamp: lamp_to_u8(lamp),
                    on: on as u8,
                },
                seq,
            );
        });
    }

    fn set_flash(&mut self, on: bool) {
        with_bridge_and_link(|link, bridge| {
            let seq = bridge.tx_seq;
            bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
            link.send(&Msg::DskySetFlash { on: on as u8 }, seq);
        });
    }

    fn read_key(&mut self) -> Option<u8> {
        cortex_m::interrupt::free(|cs| crate::BRIDGE.borrow(cs).borrow_mut().key_queue.pop_front())
    }
}

fn lamp_to_u8(lamp: Lamp) -> u8 {
    match lamp {
        Lamp::UplinkActivity => 0,
        Lamp::NoAtt => 1,
        Lamp::Stby => 2,
        Lamp::KeyRel => 3,
        Lamp::OprErr => 4,
        Lamp::Restart => 5,
        Lamp::GimbalLock => 6,
        Lamp::Temp => 7,
        Lamp::ProgAlarm => 8,
        Lamp::CompActy => 9,
    }
}
