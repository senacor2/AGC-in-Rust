//! Local timer stubs backed by the SysTick-based millisecond counter.
//!
//! ## Deferred work
//!
//! Full TIM3/TIM5/TIM6 wiring (arm_t3, arm_t5, arm_t6, disarm_t6) is
//! deferred to the timer-wiring milestone.  Each method logs a warning so
//! callers can detect that the stub is being exercised on real hardware.
//! `mission_time` is functional: it reads the `MS_TICKS` atomic that is
//! incremented by the SysTick exception handler in `bin/agc.rs`.

use core::sync::atomic::{AtomicU32, Ordering};

use agc_core::hal::timers::Timers;

/// Global millisecond tick counter.  Incremented by SysTick in `bin/agc.rs`.
pub static MS_TICKS: AtomicU32 = AtomicU32::new(0);

/// Zero-sized timer stub.
pub struct LocalTimers;

impl Timers for LocalTimers {
    fn arm_t3(&mut self, centiseconds: u16) {
        defmt::warn!(
            "LocalTimers::arm_t3({}) — TIM3 wiring deferred",
            centiseconds
        );
    }

    fn arm_t5(&mut self, centiseconds: u16) {
        defmt::warn!(
            "LocalTimers::arm_t5({}) — TIM5 wiring deferred",
            centiseconds
        );
    }

    fn arm_t6(&mut self, counts: u16) {
        defmt::warn!("LocalTimers::arm_t6({}) — TIM6 wiring deferred", counts);
    }

    fn disarm_t6(&mut self) {
        defmt::warn!("LocalTimers::disarm_t6() — TIM6 wiring deferred");
    }

    fn mission_time(&self) -> u32 {
        // Convert milliseconds to centiseconds (÷ 10).
        MS_TICKS.load(Ordering::Relaxed) / 10
    }
}
