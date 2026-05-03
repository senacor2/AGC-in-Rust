//! AGC timer trait implementation backed by STM32F7 hardware timers.
//!
//! ## Timer assignment
//!
//! | AGC interrupt | STM32 timer | Mode          |
//! |---|---|---|
//! | T3RUPT (waitlist, ≤163 s) | TIM2 (32-bit) | one-shot (OPM=1) |
//! | T4RUPT (120 ms periodic)  | TIM3 (16-bit) | periodic (configured once) |
//! | T5RUPT (DAP cycle)        | TIM4 (16-bit) | one-shot (OPM=1) |
//! | T6RUPT (RCS jet pulse)    | TIM5 (32-bit) | one-shot (OPM=1) |
//!
//! All four sit on APB1 → 108 MHz timer clock (APB1 prescaler ≠ 1, so timer
//! clock = 2 × PCLK1 = 2 × 54 MHz = 108 MHz per STM32F7 RM §6.2).
//!
//! ## Prescaler choices
//!
//! TIM2 (arm_t3): PSC=1079 → 100 kHz timer clock.
//! 1 cs = 1000 ticks at 100 kHz; max ARR = 2^32-1 ≈ 42950 s >> 163 s limit.
//!
//! TIM4 (arm_t5): PSC=10799 → 10 kHz timer clock.
//! 1 cs = 100 ticks at 10 kHz; ARR max = 65535 ticks = 6553 cs ≈ 65 s.
//!
//! TIM5 (arm_t6): PSC=0 → 108 MHz timer clock.
//! 1 T6 count = 0.625 ms = 67500 ticks at 108 MHz.
//!
//! TIM3 (T4RUPT, periodic 120 ms): PSC=10799 → 10 kHz → ARR=1199.

use core::sync::atomic::{AtomicU32, Ordering};

use agc_core::hal::runtime::T6_PENDING;
use agc_core::hal::timers::Timers;

use crate::TIMER_HANDLES;

/// Global millisecond tick counter. Incremented by SysTick in `bin/agc.rs`.
pub static MS_TICKS: AtomicU32 = AtomicU32::new(0);

/// Handles to the four timer PAC peripherals.
pub struct TimerHandles {
    pub tim2: stm32f7xx_hal::pac::TIM2,
    pub tim3: stm32f7xx_hal::pac::TIM3,
    pub tim4: stm32f7xx_hal::pac::TIM4,
    pub tim5: stm32f7xx_hal::pac::TIM5,
}

/// Zero-sized timer handle. All timer state lives in `TIMER_HANDLES`.
pub struct LocalTimers;

impl LocalTimers {
    /// Initialise the four AGC timers and return a `LocalTimers`.
    ///
    /// Takes ownership of TIM2/3/4/5 PAC peripherals, enables their APB1 clocks,
    /// and configures them. TIM3 is started immediately in periodic mode at
    /// 120 ms (T4RUPT heartbeat). TIM2/4/5 are configured but not started;
    /// they start when `arm_t3`, `arm_t5`, or `arm_t6` is called.
    ///
    /// Stores the handles in `TIMER_HANDLES` so ISRs and trait methods can reach them.
    ///
    /// # Safety
    /// Must be called exactly once, before any ISRs that access these timers are
    /// unmasked. The RCC pointer dereference is safe because it happens during
    /// single-threaded init before any concurrent access.
    pub fn init(
        tim2: stm32f7xx_hal::pac::TIM2,
        tim3: stm32f7xx_hal::pac::TIM3,
        tim4: stm32f7xx_hal::pac::TIM4,
        tim5: stm32f7xx_hal::pac::TIM5,
    ) -> Self {
        // SAFETY: APB1ENR and timer registers are only written here, before
        // any ISRs touching these timers are unmasked. Single-threaded init.
        unsafe {
            let rcc = &*stm32f7xx_hal::pac::RCC::ptr();

            // Enable APB1 clocks for TIM2/3/4/5.
            rcc.apb1enr.modify(|_, w| {
                w.tim2en()
                    .set_bit()
                    .tim3en()
                    .set_bit()
                    .tim4en()
                    .set_bit()
                    .tim5en()
                    .set_bit()
            });

            // Reset then release each timer to clear any leftover state.
            rcc.apb1rstr.modify(|_, w| {
                w.tim2rst()
                    .set_bit()
                    .tim3rst()
                    .set_bit()
                    .tim4rst()
                    .set_bit()
                    .tim5rst()
                    .set_bit()
            });
            rcc.apb1rstr.modify(|_, w| {
                w.tim2rst()
                    .clear_bit()
                    .tim3rst()
                    .clear_bit()
                    .tim4rst()
                    .clear_bit()
                    .tim5rst()
                    .clear_bit()
            });

            // ── TIM2: T3RUPT one-shot ────────────────────────────────────────
            // PSC=1079 → 108 MHz / 1080 = 100 kHz timer clock.
            // 1 cs = 1000 ticks; max delay = 2^32/1000 cs ≈ 49 days >> 163 s.
            let t2 = &*stm32f7xx_hal::pac::TIM2::ptr();
            t2.psc.write(|w| w.psc().bits(1079));
            // ARR placeholder — overwritten by arm_t3 before each use.
            t2.arr.write(|w| w.bits(9999));
            t2.egr.write(|w| w.ug().set_bit());
            t2.sr.modify(|_, w| w.uif().clear_bit());
            t2.dier.write(|w| w.uie().set_bit());
            // OPM=1 (one-shot), ARPE=1 (buffered), do NOT set CEN yet.
            t2.cr1.write(|w| w.opm().set_bit().arpe().set_bit());

            // ── TIM3: T4RUPT periodic 120 ms ────────────────────────────────
            // PSC=10799 → 108 MHz / 10800 = 10 kHz timer clock.
            // ARR=1199 → period = 1200 / 10000 = 120 ms exactly.
            let t3 = &*stm32f7xx_hal::pac::TIM3::ptr();
            t3.psc.write(|w| w.psc().bits(10799));
            t3.arr.write(|w| w.arr().bits(1199));
            t3.egr.write(|w| w.ug().set_bit());
            t3.sr.modify(|_, w| w.uif().clear_bit());
            t3.dier.write(|w| w.uie().set_bit());
            // Periodic mode (OPM=0, the reset default), ARPE=1, start now.
            t3.cr1.write(|w| w.arpe().set_bit().cen().set_bit());

            // ── TIM4: T5RUPT one-shot ────────────────────────────────────────
            // PSC=10799 → 10 kHz timer clock.
            // 1 cs = 100 ticks; ARR max = 65535 ticks = 655 cs ≈ 6.5 s.
            let t4 = &*stm32f7xx_hal::pac::TIM4::ptr();
            t4.psc.write(|w| w.psc().bits(10799));
            t4.arr.write(|w| w.arr().bits(9999));
            t4.egr.write(|w| w.ug().set_bit());
            t4.sr.modify(|_, w| w.uif().clear_bit());
            t4.dier.write(|w| w.uie().set_bit());
            // OPM=1 (one-shot), ARPE=1, do NOT start yet.
            t4.cr1.write(|w| w.opm().set_bit().arpe().set_bit());

            // ── TIM5: T6RUPT one-shot ────────────────────────────────────────
            // PSC=0 → 108 MHz timer clock.
            // 1 T6 count = 0.625 ms = 67500 ticks at 108 MHz.
            let t5 = &*stm32f7xx_hal::pac::TIM5::ptr();
            t5.psc.write(|w| w.psc().bits(0));
            t5.arr.write(|w| w.bits(67499));
            t5.egr.write(|w| w.ug().set_bit());
            t5.sr.modify(|_, w| w.uif().clear_bit());
            t5.dier.write(|w| w.uie().set_bit());
            // OPM=1 (one-shot), ARPE=1, do NOT start yet.
            t5.cr1.write(|w| w.opm().set_bit().arpe().set_bit());
        }

        // Store handles in the global so ISRs and trait impls can borrow them.
        cortex_m::interrupt::free(|cs| {
            *TIMER_HANDLES.borrow(cs).borrow_mut() = Some(TimerHandles {
                tim2,
                tim3,
                tim4,
                tim5,
            });
        });

        LocalTimers
    }
}

impl Timers for LocalTimers {
    /// Arm TIME3 (TIM2) as a one-shot for `centiseconds` cs.
    ///
    /// PSC=1079 → 100 kHz clock → ARR = cs × 1000 − 1.
    fn arm_t3(&mut self, centiseconds: u16) {
        let arr = (centiseconds as u32) * 1000 - 1;
        cortex_m::interrupt::free(|cs| {
            if let Some(h) = TIMER_HANDLES.borrow(cs).borrow_mut().as_mut() {
                // SAFETY: TIM2 register writes inside a critical section;
                // no concurrent ISR access is possible here.
                unsafe {
                    let t2 = &*stm32f7xx_hal::pac::TIM2::ptr();
                    t2.cr1.modify(|_, w| w.cen().clear_bit());
                    t2.cnt.write(|w| w.bits(0));
                    t2.arr.write(|w| w.bits(arr));
                    t2.sr.modify(|_, w| w.uif().clear_bit());
                    // OPM=1 already set in init; just enable counter.
                    t2.cr1.modify(|_, w| w.cen().set_bit());
                }
                let _ = h; // keep borrow alive
            }
        });
    }

    /// Arm TIME5 (TIM4) as a one-shot for `centiseconds` cs.
    ///
    /// PSC=10799 → 10 kHz clock → ARR = cs × 100 − 1.
    fn arm_t5(&mut self, centiseconds: u16) {
        let arr = centiseconds * 100 - 1;
        cortex_m::interrupt::free(|cs| {
            if let Some(h) = TIMER_HANDLES.borrow(cs).borrow_mut().as_mut() {
                // SAFETY: TIM4 register writes inside a critical section.
                unsafe {
                    let t4 = &*stm32f7xx_hal::pac::TIM4::ptr();
                    t4.cr1.modify(|_, w| w.cen().clear_bit());
                    t4.cnt.write(|w| w.bits(0));
                    t4.arr.write(|w| w.arr().bits(arr));
                    t4.sr.modify(|_, w| w.uif().clear_bit());
                    t4.cr1.modify(|_, w| w.cen().set_bit());
                }
                let _ = h;
            }
        });
    }

    /// Arm TIME6 (TIM5) as a one-shot for `counts` × 0.625 ms.
    ///
    /// PSC=0 → 108 MHz clock → ARR = counts × 67500 − 1.
    fn arm_t6(&mut self, counts: u16) {
        let arr = (counts as u32) * 67500 - 1;
        cortex_m::interrupt::free(|cs| {
            if let Some(h) = TIMER_HANDLES.borrow(cs).borrow_mut().as_mut() {
                // SAFETY: TIM5 register writes inside a critical section.
                unsafe {
                    let t5 = &*stm32f7xx_hal::pac::TIM5::ptr();
                    t5.cr1.modify(|_, w| w.cen().clear_bit());
                    t5.cnt.write(|w| w.bits(0));
                    t5.arr.write(|w| w.bits(arr));
                    t5.sr.modify(|_, w| w.uif().clear_bit());
                    t5.cr1.modify(|_, w| w.cen().set_bit());
                }
                let _ = h;
            }
        });
    }

    /// Disarm TIME6: stop TIM5, clear UIF, clear `T6_PENDING`.
    fn disarm_t6(&mut self) {
        cortex_m::interrupt::free(|cs| {
            if let Some(h) = TIMER_HANDLES.borrow(cs).borrow_mut().as_mut() {
                // SAFETY: TIM5 register writes inside a critical section.
                unsafe {
                    let t5 = &*stm32f7xx_hal::pac::TIM5::ptr();
                    t5.cr1.modify(|_, w| w.cen().clear_bit());
                    t5.sr.modify(|_, w| w.uif().clear_bit());
                }
                let _ = h;
            }
        });
        T6_PENDING.store(false, Ordering::Relaxed);
    }

    /// Read the current mission elapsed time in centiseconds (SysTick-derived).
    fn mission_time(&self) -> u32 {
        // SysTick fires at 1 kHz → 1 ms per tick. Divide by 10 for cs.
        MS_TICKS.load(Ordering::Relaxed) / 10
    }
}
