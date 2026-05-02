//! Independent Watchdog (IWDG) wrapper.
//!
//! `init()` programs the IWDG for a ≈1.5 s timeout:
//!   - Prescaler: /256 → LSI ≈ 32 kHz → tick ≈ 8 ms
//!   - Reload: 187 → 187 × 8 ms ≈ 1.496 s
//! This is within the AGC night-watchman spec of 0.64–1.92 s.
//!
//! The `stm32f7xx-hal` crate (v0.8) does not expose a typed IWDG wrapper,
//! so we write the unlock/prescaler/reload sequence directly.

use stm32f7xx_hal::pac::IWDG;

/// Thin wrapper around the IWDG peripheral.
pub struct Watchdog {
    iwdg: IWDG,
}

impl Watchdog {
    /// Consume the PAC IWDG token and start the watchdog.
    ///
    /// After this call the watchdog is running; `pet()` must be called at
    /// least once every ≈1.5 s.
    pub fn init(iwdg: IWDG) -> Self {
        // SAFETY: These are the documented IWDG unlock and configuration
        // sequences from RM0431 §33.3.  No other code touches the IWDG
        // registers; the PAC token is consumed here, preventing aliased access.
        unsafe {
            // Unlock IWDG_PR and IWDG_RLR.
            iwdg.kr.write(|w| w.bits(0x5555));
            // Prescaler /256.
            iwdg.pr.write(|w| w.bits(0b110));
            // Reload = 187 (≈ 1.496 s at 32 kHz / 256).
            iwdg.rlr.write(|w| w.bits(187));
            // Wait for the register-update synchronisation.
            while iwdg.sr.read().bits() != 0 {}
            // Start the watchdog.
            iwdg.kr.write(|w| w.bits(0xCCCC));
        }
        Self { iwdg }
    }

    /// Pet (reload) the watchdog.  Must be called within the timeout period.
    pub fn pet(&self) {
        // SAFETY: Writing the reload key is the only safe IWDG operation after
        // `init`; the value 0xAAAA is the documented reload sequence.
        unsafe {
            self.iwdg.kr.write(|w| w.bits(0xAAAA));
        }
    }
}
