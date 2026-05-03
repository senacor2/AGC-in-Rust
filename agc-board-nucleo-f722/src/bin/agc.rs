//! AGC firmware entry point for the Nucleo-F722ZE.
//!
//! Boot sequence:
//!   1. `cortex-m-rt` sets up the stack and zero-initialises BSS.
//!   2. Clocks are configured to 216 MHz (SYSCLK) / 54 MHz (APB1) / 108 MHz (APB2).
//!   3. USART6 and IWDG are initialised.
//!   4. RCC CSR reset-cause flags examined: cold POR/BOR → `fresh_start`;
//!      any warm reset (IWDG, software, pin) → `restart` (preserves nav state).
//!   5. SPI3 + BMI088 initialised; 100-sample bias calibration; initial attitude
//!      derived from the measured gravity vector; platform uncaged.
//!   6. TIM7 configured at 1 kHz; NVIC unmasked at priority 0x80.
//!   7. SysTick configured at 1 kHz.
//!   8. TIM2/3/4/5 initialised; NVIC priorities set and unmasked (TIM4 configured
//!      but interrupt not enabled — T5 path retired per ADR-020).
//!   9. CDU pre-read; DAP brought up in AttitudeHold mode.
//!  10. `HelloAck` sent to confirm the bridge link.
//!  11. Memory layout logged via defmt.
//!  12. `Executive::run` entered — never returns.

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::Ordering;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use defmt_rtt as _;
use panic_probe as _;
// `interrupt` re-exported from the PAC verifies the name at compile time (ADR-010).
use stm32f7xx_hal::pac::interrupt;
use stm32f7xx_hal::{pac, prelude::*, rcc::RccExt, spi};

use agc_board_nucleo_f722::{
    link::{dispatch, uart::UartLink},
    local::{timers::MS_TICKS, watchdog::Watchdog},
    register_watchdog_pet, with_timers, BMI088, LINK, PLATFORM,
};
use agc_core::{
    executive::scheduler::Executive,
    hal::runtime::{T3_PENDING, T4_PENDING, T6_PENDING},
    services::fresh_start::fresh_start,
    AgcState,
};
use agc_imu_platform::UnitQuaternion;
use agc_protocol::{Msg, PROTO_VERSION};

// ── Static AgcState ───────────────────────────────────────────────────────────

static mut AGC_STATE: AgcState = AgcState::new();

// ── Watchdog singleton ────────────────────────────────────────────────────────

static WDG: Mutex<RefCell<Option<Watchdog>>> = Mutex::new(RefCell::new(None));

fn pet_wdg() {
    cortex_m::interrupt::free(|cs| {
        if let Some(wdg) = WDG.borrow(cs).borrow().as_ref() {
            wdg.pet();
        }
    });
}

// ── Linker-script symbols (cortex-m-rt 0.7) ──────────────────────────────────

extern "C" {
    static _stack_start: u32;
    static _stack_end: u32;
    static __sdata: u32;
    static __edata: u32;
    static __sbss: u32;
    static __ebss: u32;
}

// ── RCC reset-cause detection (ADR-020) ──────────────────────────────────────

/// Returns `true` if this boot was a cold power-on or brown-out reset.
///
/// Cold boots should run `fresh_start` (all state zeroed). Warm boots
/// (IWDG expiry, software reset, NRST pin, low-power wakeup) should run
/// `restart` so navigation state is preserved.
///
/// # Safety
/// RCC CSR is read and then RMVF is written once during single-threaded
/// init before any ISR touches RCC. No aliased access.
fn was_cold_boot() -> bool {
    // SAFETY: RCC CSR is a single read/modify; no aliasing. Performed
    // exactly once at boot before any other code touches RCC.
    let rcc = unsafe { &*pac::RCC::ptr() };
    let csr = rcc.csr.read();
    let cold = csr.porrstf().is_reset() || csr.borrstf().is_reset();
    // Clear all reset flags so the next boot sees only what just happened.
    rcc.csr.modify(|_, w| w.rmvf().clear());
    cold
}

// ── Memory layout report ──────────────────────────────────────────────────────

/// Log stack / BSS / data sizes via defmt at boot.
///
/// # Safety
/// Only reads the numeric values of linker-script-defined addresses; no
/// pointer is dereferenced as data. Called once before `Executive::run`.
unsafe fn report_memory() {
    let stack_top = &_stack_start as *const _ as usize;
    let stack_bot = &_stack_end as *const _ as usize;
    let bss_size = (&__ebss as *const _ as usize) - (&__sbss as *const _ as usize);
    let data_size = (&__edata as *const _ as usize) - (&__sdata as *const _ as usize);
    defmt::info!(
        "memory: stack {}..{} ({} B), .bss={} B, .data={} B",
        stack_bot,
        stack_top,
        stack_top.saturating_sub(stack_bot),
        bss_size,
        data_size
    );
}

// ── Entry ─────────────────────────────────────────────────────────────────────

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap_or_else(|| cortex_m::peripheral::SCB::sys_reset());
    let mut cp =
        cortex_m::Peripherals::take().unwrap_or_else(|| cortex_m::peripheral::SCB::sys_reset());

    // ── Clocks: 216 MHz SYSCLK, APB1 = 54 MHz, APB2 = 108 MHz ──────────────
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(216.MHz())
        .pclk1(54.MHz())
        .pclk2(108.MHz())
        .freeze();

    // ── USART6 / GPIOC ───────────────────────────────────────────────────────
    let link = UartLink::init(dp.USART6, &clocks);
    cortex_m::interrupt::free(|cs| {
        *LINK.borrow(cs).borrow_mut() = Some(link);
    });

    // SAFETY: no concurrent NVIC access; single-threaded init.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART6);
    }

    // ── Watchdog (≈1.024 s timeout, AGC spec window 0.64–1.92 s) ─────────────
    let wdg = Watchdog::init(dp.IWDG);
    cortex_m::interrupt::free(|cs| {
        *WDG.borrow(cs).borrow_mut() = Some(wdg);
    });
    register_watchdog_pet(pet_wdg);

    // ── SPI3 + BMI088 ─────────────────────────────────────────────────────────
    // Pins:
    //   PB3  = SCK  (AF6)  — CN7 pin 15
    //   PB4  = MISO (AF6)  — CN7 pin 19
    //   PB5  = MOSI (AF6)  — CN7 pin 13
    //   PA15 = CS_ACCEL (push-pull output, idle high) — CN7 pin 17
    //   PB12 = CS_GYRO  (push-pull output, idle high) — CN10 pin 16
    //
    // APB1 peripheral clock = 54 MHz.  SPI3 prescaler /8 → 6.75 MHz, which
    // is within the BMI088's 10 MHz SPI clock limit (datasheet §4.5).
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    let sck = gpiob.pb3.into_alternate::<6>();
    let miso = gpiob.pb4.into_alternate::<6>();
    let mosi = gpiob.pb5.into_alternate::<6>();

    let mut cs_accel = gpioa.pa15.into_push_pull_output();
    let mut cs_gyro = gpiob.pb12.into_push_pull_output();
    cs_accel.set_high();
    cs_gyro.set_high();

    // Build the SPI3 HAL handle.  `stm32f7xx-hal 0.8` does not return
    // `rcc.apb1` from `Clocks::freeze()`, so we construct it from a zeroed
    // ZST (APB1 is `struct APB1(())`; zeroing is a no-op).  The `enable`
    // call inside the HAL writes APB1ENR.spi3en — idempotent if already set.
    //
    // SAFETY: `APB1` is a zero-size wrapper with no invariants; `zeroed()` is
    // identical to `APB1::new()` which is pub(crate).
    let spi3 = {
        use stm32f7xx_hal::spi::Spi;
        let mut apb1: stm32f7xx_hal::rcc::APB1 = unsafe { core::mem::zeroed() };
        Spi::new(dp.SPI3, (sck, miso, mosi)).enable::<u8>(
            spi::Mode {
                polarity: spi::Polarity::IdleLow,
                phase: spi::Phase::CaptureOnFirstTransition,
            },
            6_750.kHz(),
            &clocks,
            &mut apb1,
        )
    };

    use agc_board_nucleo_f722::local::imu::bmi088::Bmi088Driver;

    let bmi = match Bmi088Driver::init(spi3, cs_accel, cs_gyro) {
        Ok(d) => d,
        Err(e) => {
            defmt::error!("BMI088 init failed: {:?}", defmt::Debug2Format(&e));
            // Delay so probe-rs can drain the RTT buffer before reset.
            cortex_m::asm::delay(216_000 * 100);
            cortex_m::peripheral::SCB::sys_reset()
        }
    };

    // ── Bias calibration + initial attitude ───────────────────────────────────
    // Collect 100 samples at ~10 ms spacing.
    // Gyro mean = gyro zero-rate offset.
    // Accel mean direction = gravity body-frame direction.
    let mut bmi = bmi;
    let mut gyro_sum = [0.0f64; 3];
    let mut accel_sum = [0.0f64; 3];
    const CAL_SAMPLES: usize = 100;
    for _ in 0..CAL_SAMPLES {
        let g = bmi.read_gyro_rad_s();
        let a = bmi.read_accel_mps2();
        for i in 0..3 {
            gyro_sum[i] += g[i];
            accel_sum[i] += a[i];
        }
        cortex_m::asm::delay(216_000 * 10);
    }
    let n = CAL_SAMPLES as f64;
    let gyro_bias = [gyro_sum[0] / n, gyro_sum[1] / n, gyro_sum[2] / n];
    let accel_mean = [accel_sum[0] / n, accel_sum[1] / n, accel_sum[2] / n];

    // Derive initial platform attitude from the measured gravity direction rather
    // than assuming +Z alignment (identity attitude). This makes the platform
    // "level" in inertial space regardless of board mounting tilt.
    let mag = libm::sqrt(
        accel_mean[0] * accel_mean[0]
            + accel_mean[1] * accel_mean[1]
            + accel_mean[2] * accel_mean[2],
    );
    let initial_attitude = if mag > 1.0 {
        let g_unit = [
            accel_mean[0] / mag,
            accel_mean[1] / mag,
            accel_mean[2] / mag,
        ];
        UnitQuaternion::from_two_unit_vectors(g_unit, [0.0, 0.0, 1.0])
    } else {
        // Sensor failure or near-weightless environment — fall back to identity.
        UnitQuaternion::IDENTITY
    };
    // With gravity rotated into +Z by attitude, no accel bias correction is needed.
    let accel_bias = [0.0, 0.0, 0.0];

    defmt::info!(
        "imu: initial attitude q=({},{},{},{})",
        initial_attitude.w,
        initial_attitude.x,
        initial_attitude.y,
        initial_attitude.z
    );

    // Store BMI088 and uncage the platform with the gravity-derived attitude.
    cortex_m::interrupt::free(|cs| {
        *BMI088.borrow(cs).borrow_mut() = Some(bmi);
        let mut platform = PLATFORM.borrow(cs).borrow_mut();
        platform.set_bias(gyro_bias, accel_bias);
        platform.uncage(initial_attitude);
    });

    defmt::info!("BMI088: platform uncaged (gravity-vector initial attitude)");

    // ── TIM7 at 1 kHz ────────────────────────────────────────────────────────
    // APB1 timer clock = 108 MHz (2 × PCLK1 because APB1 prescaler ≠ 1,
    // per STM32F7 reference manual §6.2).
    // PSC = 107, ARR = 999  →  108 MHz / 108 / 1000 = 1 kHz exactly.
    //
    // SAFETY: TIM7 registers are written only here (before the ISR is
    // unmasked) and in the TIM7 ISR itself.  No data races.
    unsafe {
        let rcc_rb = &*pac::RCC::ptr();
        rcc_rb.apb1enr.modify(|_, w| w.tim7en().set_bit());
        rcc_rb.apb1rstr.modify(|_, w| w.tim7rst().set_bit());
        rcc_rb.apb1rstr.modify(|_, w| w.tim7rst().clear_bit());

        let tim7 = &*pac::TIM7::ptr();
        tim7.psc.write(|w| w.psc().bits(107));
        tim7.arr.write(|w| w.arr().bits(999));
        // Generate update event to load PSC/ARR into shadow registers.
        tim7.egr.write(|w| w.ug().set_bit());
        // Clear the update interrupt flag raised by the UG event above.
        tim7.sr.modify(|_, w| w.uif().clear_bit());
        // Enable update interrupt (UIE bit).
        tim7.dier.write(|w| w.uie().set_bit());
        // Start the counter.
        tim7.cr1.write(|w| w.cen().set_bit());

        // TIM7 at priority 0x80 — below all AGC timer interrupts.
        cp.NVIC.set_priority(pac::Interrupt::TIM7, 0x80);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM7);
    }

    defmt::info!("TIM7: 1 kHz IMU sample loop started");

    // ── SysTick at 1 kHz ─────────────────────────────────────────────────────
    let mut systick = cp.SYST;
    systick.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    systick.set_reload(215_999); // 216 MHz / 1000 - 1
    systick.clear_current();
    systick.enable_counter();
    systick.enable_interrupt();

    // ── TIM2/3/4/5: AGC scheduling timers ────────────────────────────────────
    use agc_board_nucleo_f722::local::timers::LocalTimers;
    let _timers = LocalTimers::init(dp.TIM2, dp.TIM3, dp.TIM4, dp.TIM5);

    // SAFETY: NVIC writes during single-threaded init before these ISRs fire.
    // TIM4 (T5RUPT) is configured in LocalTimers::init but its interrupt is NOT
    // unmasked here — the T5 path is retired per ADR-020.
    unsafe {
        // Priority order matches AGC Interrupt enum discriminants:
        //   T6RUPT (TIM5) = highest AGC priority → NVIC 0x10
        //   T3RUPT (TIM2)                        → NVIC 0x30
        //   T4RUPT (TIM3)                        → NVIC 0x40
        cp.NVIC.set_priority(pac::Interrupt::TIM5, 0x10);
        cp.NVIC.set_priority(pac::Interrupt::TIM2, 0x30);
        cp.NVIC.set_priority(pac::Interrupt::TIM3, 0x40);

        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM5);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM3);
    }

    defmt::info!("TIM2/3/5: AGC scheduling timers started (TIM4/T5 retired per ADR-020)");

    // ── FRESH START or RESTART ────────────────────────────────────────────────
    // SAFETY: `AGC_STATE` is a `static mut`. We obtain a raw pointer then
    // immediately reborrow as `&mut`.  Interrupt handlers access only the
    // `BRIDGE`, `LINK`, `PLATFORM`, `BMI088`, and `TIMER_HANDLES` statics —
    // never `AGC_STATE` — so there is no aliased mutable access.
    let state: &mut AgcState = unsafe { &mut *core::ptr::addr_of_mut!(AGC_STATE) };
    if was_cold_boot() {
        defmt::info!("agc: COLD BOOT — running FRESH START");
        fresh_start(state);
    } else {
        defmt::info!("agc: WARM BOOT — running RESTART (nav state preserved)");
        agc_core::services::fresh_start::restart(state);
    }

    // ── DAP bootstrap ─────────────────────────────────────────────────────────
    // Read CDU once so dap_init has a valid prev_cdu baseline (its contract).
    // Board is a zero-sized type — constructing a temporary here is a no-op.
    {
        use agc_board_nucleo_f722::Board;
        use agc_core::hal::imu::Imu;
        state.current_cdu = Board::new().imu.read_cdu();
    }
    agc_core::control::dap::dap_init(state, agc_core::control::dap::DapMode::AttitudeHold);
    agc_core::services::average_g::start_servicer(state);

    // ── Hello handshake ───────────────────────────────────────────────────────
    agc_board_nucleo_f722::with_bridge_and_link(|link, bridge| {
        let seq = bridge.tx_seq;
        bridge.tx_seq = bridge.tx_seq.wrapping_add(1);
        link.send(
            &Msg::HelloAck {
                proto_version: PROTO_VERSION,
            },
            seq,
        );
    });

    // ── Memory layout report ─────────────────────────────────────────────────
    // SAFETY: report_memory reads only the addresses of linker-defined symbols;
    // it does not dereference them as data.
    unsafe { report_memory() };

    defmt::info!("agc: init complete, DAP active, executive entering run loop");

    // ── Executive run loop (never returns) ────────────────────────────────────
    use agc_board_nucleo_f722::Board;
    let mut board = Board::new();
    Executive::run(state, &mut board)
}

// ── SysTick handler ───────────────────────────────────────────────────────────

#[exception]
fn SysTick() {
    MS_TICKS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
}

// ── USART6 ISR ────────────────────────────────────────────────────────────────

#[interrupt]
fn USART6() {
    cortex_m::interrupt::free(|cs| {
        let mut link_opt = LINK.borrow(cs).borrow_mut();
        if let Some(link) = link_opt.as_mut() {
            while let Some(msg) = link.poll_rx() {
                dispatch::handle(msg, cs);
            }
        }
    });
}

// ── TIM7 ISR: 1 kHz IMU sample loop ──────────────────────────────────────────
//
// Priority 0x80 — lower than the AGC T6/T3/T4 ISRs (0x10–0x40).
// The SPI reads are inside `interrupt::free` because the `Bmi088Driver` is
// owned by the `BMI088` static which requires a `CriticalSection` token.
// SPI transfer time for 7 + 8 bytes at 6.75 MHz ≈ 18 µs, well within 1 ms.
#[interrupt]
fn TIM7() {
    // SAFETY: Clearing TIM7's UIF flag requires a direct register write.
    // TIM7 registers are only modified here (after init) and in the init
    // block above (before this ISR was unmasked) — no concurrent access.
    unsafe {
        (*pac::TIM7::ptr()).sr.modify(|_, w| w.uif().clear_bit());
    }

    cortex_m::interrupt::free(|cs| {
        let mut bmi_opt = BMI088.borrow(cs).borrow_mut();
        if let Some(bmi) = bmi_opt.as_mut() {
            let gyro = bmi.read_gyro_rad_s();
            let accel = bmi.read_accel_mps2();
            PLATFORM.borrow(cs).borrow_mut().tick(gyro, accel, 0.001);
        }
    });
}

// ── AGC timer ISRs ────────────────────────────────────────────────────────────
//
// Each ISR: clear UIF to acknowledge the interrupt, then set the corresponding
// pending flag. The Executive's main loop (foreground) drains the flags in
// priority order and runs the associated action. ISRs are kept minimal — no
// AgcState access — satisfying ADR-002 and ADR-008.

/// TIM2 → T3RUPT (waitlist dispatch). Priority 0x30.
#[interrupt]
fn TIM2() {
    with_timers(|h| {
        h.tim2.sr.modify(|_, w| w.uif().clear_bit());
    });
    T3_PENDING.store(true, Ordering::Release);
}

/// TIM3 → T4RUPT (periodic I/O, 120 ms). Priority 0x40.
#[interrupt]
fn TIM3() {
    with_timers(|h| {
        h.tim3.sr.modify(|_, w| w.uif().clear_bit());
    });
    T4_PENDING.store(true, Ordering::Release);
}

/// TIM5 → T6RUPT (RCS jet pulse). Priority 0x10 (highest AGC priority).
#[interrupt]
fn TIM5() {
    with_timers(|h| {
        h.tim5.sr.modify(|_, w| w.uif().clear_bit());
    });
    T6_PENDING.store(true, Ordering::Release);
}
