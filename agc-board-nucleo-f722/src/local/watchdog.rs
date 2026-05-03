//! Independent Watchdog (IWDG) wrapper.
//!
//! `init()` programs the IWDG for a 1.024 s timeout:
//!   - Prescaler: /64 → LSI ≈ 32 kHz → tick ≈ 2 ms
//!   - Reload: 512 → 512 × 2 ms = 1.024 s
//! Within the AGC night-watchman spec of 0.64–1.92 s (specs/hal-spec.md §4.3).
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
    /// least once every 1.024 s.
    pub fn init(iwdg: IWDG) -> Self {
        // SAFETY: These are the documented IWDG unlock and configuration
        // sequences from RM0431 §33.3.  No other code touches the IWDG
        // registers; the PAC token is consumed here, preventing aliased access.
        unsafe {
            // Unlock IWDG_PR and IWDG_RLR.
            iwdg.kr.write(|w| w.bits(0x5555));
            // Prescaler /64 (0b100): 32 kHz / 64 = 500 Hz → tick = 2 ms.
            iwdg.pr.write(|w| w.bits(0b100));
            // Reload = 512: 512 × 2 ms = 1.024 s (AGC spec window 0.64–1.92 s).
            iwdg.rlr.write(|w| w.bits(512));
            // Wait for the register-update synchronisation.
            while iwdg.sr.read().bits() != 0 {}
            // Start the watchdog.
            iwdg.kr.write(|w| w.bits(0xCCCC));
        }
        Self { iwdg }
    }

    /// Pet (reload) the watchdog.  Must be called within 1.024 s.
    pub fn pet(&self) {
        // SAFETY: Writing the reload key is the only safe IWDG operation after
        // `init`; the value 0xAAAA is the documented reload sequence.
        unsafe {
            self.iwdg.kr.write(|w| w.bits(0xAAAA));
        }
    }
}
