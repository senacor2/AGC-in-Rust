//! AGC firmware entry point for the Nucleo-F722ZE.
//!
//! Boot sequence:
//!   1. `cortex-m-rt` sets up the stack and zero-initialises BSS.
//!   2. Clocks are configured to 216 MHz (SYSCLK) / 54 MHz (APB1) / 108 MHz (APB2).
//!   3. USART6 and IWDG are initialised.
//!   4. SPI3 + BMI088 initialised; 100-sample bias calibration; platform uncaged.
//!   5. TIM7 configured at 1 kHz; NVIC unmasked at priority 8.
//!   6. `AgcState::new()` placed in `static mut`; `fresh_start` runs.
//!   7. `HelloAck` sent to confirm the bridge link.
//!   8. Idle loop pets watchdog and emits a 1 Hz liveness heartbeat with IMU data.

#![no_std]
#![no_main]

use core::cell::RefCell;

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
    register_watchdog_pet, BMI088, LINK, PLATFORM,
};
use agc_core::{services::fresh_start::fresh_start, AgcState};
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

    // ── Watchdog (≈1.5 s timeout) ─────────────────────────────────────────────
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

    let sck  = gpiob.pb3.into_alternate::<6>();
    let miso = gpiob.pb4.into_alternate::<6>();
    let mosi = gpiob.pb5.into_alternate::<6>();

    let mut cs_accel = gpioa.pa15.into_push_pull_output();
    let mut cs_gyro  = gpiob.pb12.into_push_pull_output();
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

    // ── Bias calibration ──────────────────────────────────────────────────────
    // Collect 100 samples at ~10 ms spacing.  Assume board is level on
    // power-up so gravity points along +Z.  Gyro mean = gyro zero-rate offset.
    // Accel bias = measured mean − [0, 0, g].
    // Initial-attitude estimation from the gravity vector is a follow-on item
    // (see tasks.md Phase 2 unchecked items).
    let mut bmi = bmi;
    let mut gyro_sum  = [0.0f64; 3];
    let mut accel_sum = [0.0f64; 3];
    const CAL_SAMPLES: usize = 100;
    for _ in 0..CAL_SAMPLES {
        let g = bmi.read_gyro_rad_s();
        let a = bmi.read_accel_mps2();
        for i in 0..3 {
            gyro_sum[i]  += g[i];
            accel_sum[i] += a[i];
        }
        cortex_m::asm::delay(216_000 * 10);
    }
    let n = CAL_SAMPLES as f64;
    let gyro_bias  = [gyro_sum[0] / n,  gyro_sum[1] / n,  gyro_sum[2] / n];
    let accel_mean = [accel_sum[0] / n, accel_sum[1] / n, accel_sum[2] / n];
    let accel_bias = [accel_mean[0], accel_mean[1], accel_mean[2] - 9.806_65];

    defmt::info!("BMI088 bias cal complete (identity initial attitude, level-board assumption)");

    // Store BMI088 and uncage the platform with identity attitude.
    cortex_m::interrupt::free(|cs| {
        *BMI088.borrow(cs).borrow_mut() = Some(bmi);
        let mut platform = PLATFORM.borrow(cs).borrow_mut();
        platform.set_bias(gyro_bias, accel_bias);
        platform.uncage(UnitQuaternion::IDENTITY);
    });

    defmt::info!("BMI088: platform uncaged");

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

        // Set TIM7 priority to 8 of 16 (0x80 in the 8-bit priority register;
        // STM32F7 uses the upper 4 bits so 0x80 = priority group 8).
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

    // ── FRESH START ───────────────────────────────────────────────────────────
    // SAFETY: `AGC_STATE` is a `static mut`. We obtain a raw pointer then
    // immediately reborrow as `&mut`.  Interrupt handlers access only the
    // `BRIDGE`, `LINK`, `PLATFORM`, and `BMI088` statics — never `AGC_STATE`
    // — so there is no aliased mutable access.
    let state: &mut AgcState = unsafe { &mut *core::ptr::addr_of_mut!(AGC_STATE) };
    fresh_start(state);

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

    defmt::info!("agc: FRESH START complete, entering idle loop");

    // ── Idle loop ─────────────────────────────────────────────────────────────
    let mut last_log_ms: u32 = 0;
    loop {
        pet_wdg();

        let now_ms = MS_TICKS.load(core::sync::atomic::Ordering::Relaxed);
        if now_ms.wrapping_sub(last_log_ms) >= 1_000 {
            last_log_ms = now_ms;

            let cdu = cortex_m::interrupt::free(|cs| {
                PLATFORM.borrow(cs).borrow().read_cdu()
            });

            defmt::info!(
                "imu: cdu=[{}, {}, {}]  MET {} cs",
                cdu[0],
                cdu[1],
                cdu[2],
                state.time.0
            );
        }

        cortex_m::asm::wfi();
    }
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
// Priority 8 (0x80) — lower than the future T6/T5/T3 ISRs.
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
            let gyro  = bmi.read_gyro_rad_s();
            let accel = bmi.read_accel_mps2();
            PLATFORM
                .borrow(cs)
                .borrow_mut()
                .tick(gyro, accel, 0.001);
        }
    });
}
