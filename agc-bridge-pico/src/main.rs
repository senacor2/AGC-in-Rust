//! RP2040 bridge firmware entry point.
//!
//! Phase 4 stub: speaks the AGC wire protocol over UART0 (GPIO0/1 at 460800
//! baud), exposes a USB-CDC developer console, and generates synthetic
//! BridgeHeartbeat + OpticsCdu traffic so the AGC sees a live bridge.
//!
//! Pin-out:
//!   GPIO0  → UART0 TX  (→ AGC RX)
//!   GPIO1  → UART0 RX  (← AGC TX)
//!   GPIO25 → on-board LED, toggled on each heartbeat

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod console;
mod keymap;
mod link;
mod state;

use agc_protocol::{Msg, PROTO_VERSION};
use console::UsbConsole;
use link::AgcLink;
use state::BridgeState;

use portable_atomic::{AtomicU32, Ordering};

use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    gpio::{FunctionUart, Pins},
    pac,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    usb::UsbBus,
    watchdog::Watchdog,
};
use usb_device::bus::UsbBusAllocator;

use embedded_hal::digital::OutputPin;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

/// Millisecond tick counter, incremented by the SysTick exception.
static MS_TICKS: AtomicU32 = AtomicU32::new(0);

/// SysTick exception handler — increments the millisecond counter.
#[cortex_m_rt::exception]
fn SysTick() {
    MS_TICKS.fetch_add(1, Ordering::Relaxed);
}

// Static allocator for the USB bus; must be 'static because UsbDevice borrows it.
// SAFETY: written exactly once in `main` before the SysTick interrupt is
// enabled, and only read (immutably) after that.  No other code accesses this.
static mut USB_BUS_ALLOC: Option<UsbBusAllocator<UsbBus>> = None;

#[entry]
fn main() -> ! {
    // ── Peripheral take ───────────────────────────────────────────────────────
    let mut dp = pac::Peripherals::take().unwrap_or_else(|| cortex_m::peripheral::SCB::sys_reset());
    let mut cp =
        cortex_m::Peripherals::take().unwrap_or_else(|| cortex_m::peripheral::SCB::sys_reset());

    // ── Watchdog (required by init_clocks_and_plls) ───────────────────────────
    let mut watchdog = Watchdog::new(dp.WATCHDOG);

    // ── Clocks: XOSC 12 MHz, PLL_SYS = 125 MHz, PLL_USB = 48 MHz ────────────
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        dp.XOSC,
        dp.CLOCKS,
        dp.PLL_SYS,
        dp.PLL_USB,
        &mut dp.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap_or_else(|| cortex_m::peripheral::SCB::sys_reset());

    // ── GPIO bank ─────────────────────────────────────────────────────────────
    let sio = Sio::new(dp.SIO);
    let pins = Pins::new(dp.IO_BANK0, dp.PADS_BANK0, sio.gpio_bank0, &mut dp.RESETS);

    // ── LED (GPIO25) ──────────────────────────────────────────────────────────
    let mut led = pins.gpio25.into_push_pull_output();

    // ── UART0: GPIO0 (TX) / GPIO1 (RX) at 460800 baud ────────────────────────
    let uart_tx = pins.gpio0.into_function::<FunctionUart>();
    let uart_rx = pins.gpio1.into_function::<FunctionUart>();
    let uart = UartPeripheral::new(dp.UART0, (uart_tx, uart_rx), &mut dp.RESETS)
        .enable(
            UartConfig::new(460_800.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap_or_else(|_| cortex_m::peripheral::SCB::sys_reset());
    let mut link = AgcLink::new(uart);

    // ── USB CDC-ACM ───────────────────────────────────────────────────────────
    let usb_bus = UsbBus::new(
        dp.USBCTRL_REGS,
        dp.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut dp.RESETS,
    );
    // SAFETY: `USB_BUS_ALLOC` is written here exactly once, while still
    // single-threaded (SysTick not yet started).  We use `addr_of_mut!` to
    // avoid creating any reference to the `static mut`, then immediately
    // cast the initialized inner value to a `'static` shared reference.
    // After this point only `UsbConsole` holds a `&'static` reference and
    // no other code touches this static.
    let bus_alloc: &'static UsbBusAllocator<UsbBus> = unsafe {
        let ptr = core::ptr::addr_of_mut!(USB_BUS_ALLOC);
        ptr.write(Some(UsbBusAllocator::new(usb_bus)));
        match &*ptr {
            Some(b) => b,
            None => core::hint::unreachable_unchecked(),
        }
    };
    let mut console = UsbConsole::new(bus_alloc);

    // ── SysTick at 1 kHz ─────────────────────────────────────────────────────
    // SYSCLK = 125 MHz; reload = 125_000 - 1 → 1 ms period.
    cp.SYST
        .set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    cp.SYST.set_reload(124_999);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    // ── Bridge state ──────────────────────────────────────────────────────────
    let mut state = BridgeState::new();

    // ── Startup: send Hello ───────────────────────────────────────────────────
    {
        let seq = state.next_seq();
        link.send(
            &Msg::Hello {
                proto_version: PROTO_VERSION,
            },
            seq,
        );
        state.last_hello_tx = MS_TICKS.load(Ordering::Relaxed);
    }
    console.write(b"AGC bridge starting...\r\n");

    // ── Main loop ─────────────────────────────────────────────────────────────
    loop {
        let now = MS_TICKS.load(Ordering::Relaxed);

        // ── Poll USB ─────────────────────────────────────────────────────────
        console.poll();

        // ── Process keystrokes ───────────────────────────────────────────────
        {
            // Collect into a fixed-size array to avoid holding borrow during send.
            let mut codes = heapless::Vec::<u8, 16>::new();
            for &byte in console.drain_rx() {
                if let Some(code) = keymap::ascii_to_dsky(byte) {
                    let _ = codes.push(code);
                }
            }
            console.clear_rx();
            for code in codes {
                let seq = state.next_seq();
                link.send(&Msg::DskyKey { code, dsky: 0 }, seq);
            }
        }

        // ── Poll UART RX ─────────────────────────────────────────────────────
        while let Some(msg) = link.poll_rx() {
            match &msg {
                Msg::HelloAck { proto_version } => {
                    if *proto_version == PROTO_VERSION {
                        state.handshake_complete = true;
                        console.write(b"handshake OK\r\n");
                    } else {
                        let seq = state.next_seq();
                        link.send(
                            &Msg::Error {
                                code: 0x01,
                                ctx: *proto_version,
                            },
                            seq,
                        );
                        console.write(b"ERR: proto version mismatch\r\n");
                    }
                }
                Msg::AgcHeartbeat { mission_time_cs } => {
                    state.last_agc_heartbeat = Some(*mission_time_cs);
                }
                other => {
                    console.log_agc_msg(other);
                }
            }
        }

        // ── Hello retry (every 1 s until handshake completes) ────────────────
        if !state.handshake_complete && now.wrapping_sub(state.last_hello_tx) >= 1_000 {
            let seq = state.next_seq();
            link.send(
                &Msg::Hello {
                    proto_version: PROTO_VERSION,
                },
                seq,
            );
            state.last_hello_tx = now;
        }

        // ── BridgeHeartbeat every 200 ms ─────────────────────────────────────
        if now.wrapping_sub(state.last_heartbeat_tx) >= 200 {
            state.last_heartbeat_tx = now;
            state.heartbeat_ms = now;

            let seq = state.next_seq();
            link.send(&Msg::BridgeHeartbeat { uptime_ms: now }, seq);

            // Toggle LED: set high for even 200 ms periods, low for odd.
            if (now / 200) & 1 == 0 {
                let _ = led.set_high();
            } else {
                let _ = led.set_low();
            }
        }

        // ── OpticsCdu every 10 ms ─────────────────────────────────────────────
        if now.wrapping_sub(state.last_cdu_tx) >= 10 {
            // Advance synthetic CDU angles: ~1 count per 100 ms on each axis.
            // Divide by 100 ms: one count per 10 polls.
            let elapsed_100ms = now / 100;
            state.cdu_trunnion = elapsed_100ms as u16;
            state.cdu_shaft = elapsed_100ms.wrapping_add(32768) as u16;

            state.last_cdu_tx = now;

            let seq = state.next_seq();
            link.send(
                &Msg::OpticsCdu {
                    trunnion: state.cdu_trunnion,
                    shaft: state.cdu_shaft,
                },
                seq,
            );
        }
    }
}
