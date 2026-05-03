//! Inbound message dispatch: bridge → AGC.
//!
//! Called from the USART6 ISR after `UartLink::poll_rx` returns a complete
//! frame.  Updates `BridgeState` cache fields and, for interrupt-generating
//! events, software-pends the corresponding NVIC line.
//!
//! ## IRQ line assignments
//!
//! The AGC Executive interrupt wiring is deferred to a later milestone; this
//! milestone selects conventionally-spare EXTI lines as placeholders:
//!
//! | AGC interrupt | EXTI line | Rationale                                    |
//! |---------------|-----------|----------------------------------------------|
//! | KeyRupt1      | EXTI0     | Spare on Nucleo-F767ZI; no on-board use      |
//! | UplinkRupt    | EXTI1     | Spare; adjacent to KeyRupt1 for easy wiring  |
//!
//! When the Executive interrupt wiring milestone is implemented, these
//! EXTI lines must be replaced with the actual STM32 timer interrupt lines
//! (TIM3/TIM4/TIM5/TIM6) and the EXTI pend calls removed.

use cortex_m::interrupt::CriticalSection;
use cortex_m::peripheral::NVIC;
use stm32f7xx_hal::pac::Interrupt;

use agc_protocol::Msg;

use crate::BRIDGE;

/// Dispatch one inbound message: update `BridgeState` and pend any required
/// software interrupts.  Must be called inside a critical section.
pub fn handle(msg: Msg, cs: &CriticalSection) {
    BRIDGE.borrow(cs).borrow_mut().dispatch(msg);
}

impl crate::state::BridgeState {
    fn dispatch(&mut self, msg: Msg) {
        match msg {
            Msg::DskyKey { code, .. } => {
                // Push into the key queue; drop silently on overflow.
                let _ = self.key_queue.push_back(code);
                // Pend KeyRupt1 via EXTI0.  The Executive ISR stub will consume
                // this when timer wiring is complete.
                // NVIC::pend is safe: it only sets a pending bit; execution
                // of the handler is deferred until the processor exits the
                // critical section and re-enables interrupts.
                NVIC::pend(Interrupt::EXTI0);
            }
            Msg::OpticsCdu { trunnion, shaft } => {
                self.optics_cdu_trunnion = trunnion;
                self.optics_cdu_shaft = shaft;
            }
            Msg::OpticsMark => {
                self.optics_mark_pending = true;
            }
            Msg::EngineThrustOn { on } => {
                self.engine_thrust_on = on != 0;
            }
            Msg::UplinkWord { word } => {
                let _ = self.uplink_queue.push_back(word);
                // Pend UplinkRupt via EXTI1.
                NVIC::pend(Interrupt::EXTI1);
            }
            Msg::BridgeHeartbeat { uptime_ms } => {
                self.last_bridge_heartbeat_ms = uptime_ms;
            }
            // AGC → bridge messages must not arrive inbound; ignore them.
            _ => {}
        }
    }
}
