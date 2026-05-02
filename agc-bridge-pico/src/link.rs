//! UART0 link driver: encodes and decodes AGC protocol frames over GPIO0/1.
//!
//! TX is blocking at 460800 baud; at <10 µs per byte a max-frame (252 B) takes
//! ≈ 2.5 ms — acceptable for stub firmware where outbound traffic is infrequent.

use agc_protocol::{encode, DecodeStatus, FrameDecoder, Msg, MAX_FRAME};
use rp2040_hal::pac::UART0;
use rp2040_hal::uart::ValidUartPinout;
use rp2040_hal::uart::{Enabled, UartPeripheral};

/// UART0 wrapper that owns the peripheral and a frame decoder.
pub struct AgcLink<P: ValidUartPinout<UART0>> {
    uart: UartPeripheral<Enabled, UART0, P>,
    decoder: FrameDecoder,
}

impl<P: ValidUartPinout<UART0>> AgcLink<P> {
    pub fn new(uart: UartPeripheral<Enabled, UART0, P>) -> Self {
        Self {
            uart,
            decoder: FrameDecoder::new(),
        }
    }

    /// Encode `msg` and blocking-write every byte to UART0.
    pub fn send(&mut self, msg: &Msg, seq: u8) {
        let mut buf = [0u8; MAX_FRAME];
        let n = match encode(msg, seq, &mut buf) {
            Ok(n) => n,
            Err(_) => return,
        };
        self.uart.write_full_blocking(&buf[..n]);
    }

    /// Non-blocking RX pump.
    ///
    /// Drains bytes from the UART FIFO into the `FrameDecoder` and returns
    /// the first complete `Msg`, or `None` if no frame is ready yet.
    pub fn poll_rx(&mut self) -> Option<Msg> {
        let mut byte = [0u8; 1];
        while self.uart.uart_is_readable() {
            if self.uart.read_raw(&mut byte).is_ok() {
                match self.decoder.push(byte[0]) {
                    DecodeStatus::Ready { msg, .. } => return Some(msg),
                    DecodeStatus::Error(_) | DecodeStatus::NeedMore => {}
                }
            }
        }
        None
    }

    /// Return the underlying peripheral (C-FREE).
    #[allow(dead_code)]
    pub fn free(self) -> (UartPeripheral<Enabled, UART0, P>, FrameDecoder) {
        (self.uart, self.decoder)
    }
}
