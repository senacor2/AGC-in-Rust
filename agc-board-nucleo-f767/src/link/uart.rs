//! USART6 bridge link driver.
//!
//! Provides `UartLink`, which owns the USART6 peripheral wired to PC6 (TX) /
//! PC7 (RX) via alternate-function 8 at 460800 baud, 8N1.
//!
//! **Blocking TX**: `send` spins on the TXE flag per byte.
//! At 460800 baud a byte is ≈ 21.7 µs; a maximum-size 252-byte frame is
//! ≈ 5.5 ms.  This is acceptable for the current milestone because outbound
//! traffic is infrequent (display updates, heartbeats) and the Executive
//! loop is not yet running.  A future milestone should replace this with
//! DMA or a TX FIFO ring buffer.

use agc_protocol::{encode, DecodeStatus, FrameDecoder, Msg, MAX_FRAME};
use stm32f7xx_hal::{pac::USART6, rcc::Clocks};

/// Thin USART6 wrapper that owns the peripheral register block.
///
/// Safety invariant: at most one `UartLink` exists at a time.
/// Enforced by the `init` function which consumes the PAC `USART6` token.
pub struct UartLink {
    usart: USART6,
    decoder: FrameDecoder,
}

impl UartLink {
    /// Initialise USART6 at 460800 baud on PC6/PC7 (AF8).
    ///
    /// Caller must have already enabled GPIOC and USART6 clocks in RCC
    /// before calling this function.  The function configures the GPIO
    /// alternate-function registers and the USART control registers directly
    /// to avoid a full HAL serial wrapper import that would require generic
    /// pin tokens not needed here.
    ///
    /// RXNEIE (RX-not-empty interrupt enable) is set so the USART6 IRQ fires
    /// on each received byte; the ISR in `bin/agc.rs` calls `poll_rx`.
    pub fn init(usart: USART6, clocks: &Clocks) -> Self {
        // Configure PC6 (TX) and PC7 (RX) as AF8 (USART6).
        // SAFETY: GPIOC clock is enabled by the caller; these registers are
        // write-only configuration; no aliasing with other code paths.
        unsafe {
            let gpioc = &*stm32f7xx_hal::pac::GPIOC::ptr();

            // MODER: set PC6 and PC7 to alternate function (0b10).
            let moder = gpioc.moder.read().bits();
            gpioc.moder.write(|w| {
                w.bits((moder & !(0b11 << 12) & !(0b11 << 14)) | (0b10 << 12) | (0b10 << 14))
            });

            // OSPEEDR: high speed for both pins.
            let ospeedr = gpioc.ospeedr.read().bits();
            gpioc.ospeedr.write(|w| {
                w.bits((ospeedr & !(0b11 << 12) & !(0b11 << 14)) | (0b11 << 12) | (0b11 << 14))
            });

            // AFRL: PC6 = AF8 (bits 27:24), PC7 = AF8 (bits 31:28).
            let afrl = gpioc.afrl.read().bits();
            gpioc
                .afrl
                .write(|w| w.bits((afrl & !(0xF << 24) & !(0xF << 28)) | (8 << 24) | (8 << 28)));

            // Configure USART6.
            // BRR for 460800 baud at PCLK2 = 108 MHz (APB2):
            //   BRR = 108_000_000 / 460800 ≈ 234.375 → 234 (rounds to ≈ 461538 baud, <0.25% error).
            let pclk2 = clocks.pclk2().raw();
            let brr = (pclk2 + 460800 / 2) / 460800;

            usart.cr1.write(|w| w.bits(0)); // reset
            usart.brr.write(|w| w.bits(brr));
            // CR1: TE | RE | RXNEIE | UE
            usart
                .cr1
                .write(|w| w.bits((1 << 3) | (1 << 2) | (1 << 5) | (1 << 0)));
        }

        Self {
            usart,
            decoder: FrameDecoder::new(),
        }
    }

    /// Encode `msg` into a stack buffer and send it byte-by-byte.
    ///
    /// `seq` must be the caller-managed outbound sequence counter (from
    /// `BridgeState.tx_seq`).  The caller increments `tx_seq` after this
    /// returns.
    pub fn send(&mut self, msg: &Msg, seq: u8) {
        let mut buf = [0u8; MAX_FRAME];
        let n = match encode(msg, seq, &mut buf) {
            Ok(n) => n,
            Err(_) => return, // frame too large — should never happen with valid Msg
        };
        for &b in &buf[..n] {
            // Spin on TXE (bit 7 of ISR).
            while self.usart.isr.read().bits() & (1 << 7) == 0 {}
            // SAFETY: `bits()` on the TDR write-register is marked unsafe by
            // the PAC because it allows writing reserved bits.  We write only
            // the 9-bit data field; bits 31:9 are reserved but the USART only
            // samples bits 8:0.  USART6 is exclusively owned here (no aliasing).
            unsafe { self.usart.tdr.write(|w| w.bits(b as u32)) };
        }
    }

    /// Non-blocking RX byte pump.
    ///
    /// Drains the USART6 RXNE flag into the `FrameDecoder` and returns the
    /// first complete `Msg`, or `None` if no frame is ready yet.
    /// The ISR calls this in a loop until it returns `None`.
    pub fn poll_rx(&mut self) -> Option<Msg> {
        while self.usart.isr.read().bits() & (1 << 5) != 0 {
            let byte = self.usart.rdr.read().bits() as u8;
            match self.decoder.push(byte) {
                DecodeStatus::Ready { msg, .. } => return Some(msg),
                DecodeStatus::Error(_) => {} // malformed frame; decoder auto-resets
                DecodeStatus::NeedMore => {}
            }
        }
        None
    }

    /// Return the raw USART6 peripheral (C-FREE).
    pub fn free(self) -> USART6 {
        self.usart
    }
}
