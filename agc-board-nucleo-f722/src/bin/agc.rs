//! AGC firmware entry point for the Nucleo-F722ZE.
//!
//! Boot sequence:
//!   1. `cortex-m-rt` sets up the stack and zero-initialises BSS.
//!   2. Clocks are configured to 216 MHz (SYSCLK) / 54 MHz (APB1) / 108 MHz (APB2).
//!   3. USART6, GPIOC, and IWDG are initialised.
//!   4. `AgcState::new()` is placed in a `static mut`; a one-time `&mut` borrow
//!      is taken immediately after init — never again.
//!   5. `fresh_start(&mut state)` runs the full re-initialisation sequence.
//!   6. A `HelloAck` is sent to the bridge to confirm the link is alive.
//!   7. The idle loop pets the watchdog and emits a 1 Hz liveness heartbeat.
//!      The Executive scheduler is NOT entered yet (timer wiring deferred).

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use defmt_rtt as _;
use panic_probe as _;
// `interrupt` is re-exported from the PAC so the compiler verifies the name
// exists on the STM32F722 (ADR-010).
use stm32f7xx_hal::pac::interrupt;
use stm32f7xx_hal::{pac, prelude::*, rcc::RccExt};

use agc_board_nucleo_f722::{
    link::{dispatch, uart::UartLink},
    local::{timers::MS_TICKS, watchdog::Watchdog},
    register_watchdog_pet, LINK,
};
use agc_core::{services::fresh_start::fresh_start, AgcState};
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
    let dp = pac::Peripherals::take().unwrap_or_else(|| {
        // `take()` returns None only if called a second time; impossible here.
        cortex_m::peripheral::SCB::sys_reset()
    });
    let cp =
        cortex_m::Peripherals::take().unwrap_or_else(|| cortex_m::peripheral::SCB::sys_reset());

    // ── Clocks: 216 MHz SYSCLK, HSE bypass, APB2 = 108 MHz ──────────────────
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

    // Enable USART6 interrupt in the NVIC.
    // SAFETY: no concurrent NVIC access; this is the only code running at this point.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART6);
    }

    // ── Watchdog (≈1.5 s timeout) ─────────────────────────────────────────────
    let wdg = Watchdog::init(dp.IWDG);
    cortex_m::interrupt::free(|cs| {
        *WDG.borrow(cs).borrow_mut() = Some(wdg);
    });
    register_watchdog_pet(pet_wdg);

    // ── SysTick at 1 kHz ─────────────────────────────────────────────────────
    let mut systick = cp.SYST;
    systick.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    // reload = SYSCLK / tick_rate - 1 = 216_000_000 / 1_000 - 1
    systick.set_reload(215_999);
    systick.clear_current();
    systick.enable_counter();
    systick.enable_interrupt();

    // ── FRESH START ───────────────────────────────────────────────────────────
    // SAFETY: `AGC_STATE` is a `static mut`. We obtain a raw pointer then
    // immediately reborrow it as `&mut`.  This is the only place in the program
    // that creates a reference to `AGC_STATE`; interrupt handlers access only
    // `BRIDGE` and `LINK`, never `AGC_STATE`, so no aliasing can occur.
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
            defmt::info!("agc: idle tick (MET {} cs)", state.time.0);
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

// The `#[interrupt]` attribute is taken from the PAC re-export (ADR-010) so
// the compiler validates that USART6 is a real interrupt on the STM32F722.
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
