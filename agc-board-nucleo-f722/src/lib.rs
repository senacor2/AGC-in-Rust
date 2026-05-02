//! Bare-metal HAL for the Nucleo-F722ZE board.
//!
//! This crate provides a concrete [`AgcHardware`] implementation for the
//! STM32F722ZE microcontroller.  External peripherals (DSKY, sextant, engines)
//! are reached through a USART6 bridge link using the `agc-protocol` framing.
//! The BMI088 IMU over SPI3 is local (ADR-016); it drives the platform
//! emulator in `agc_imu_platform` which implements the gimballed-platform
//! abstraction expected by flight software.

#![no_std]

pub mod link;
pub mod local;
pub mod remote;
pub mod state;

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;

use agc_core::hal::AgcHardware;

use link::uart::UartLink;
use local::imu::BoardImu;
use local::timers::LocalTimers;
use remote::dsky::RemoteDsky;
use remote::engine::RemoteEngine;
use remote::optics::RemoteOptics;
use remote::rcs::RemoteRcs;
use remote::telemetry::RemoteTelemetry;
use remote::uplink::RemoteUplink;
use state::BridgeState;

// ── Global singletons ─────────────────────────────────────────────────────────

/// Shared bridge state (ADR-008).
/// Written by the USART6 ISR; read by HAL trait impls.
pub static BRIDGE: Mutex<RefCell<BridgeState>> = Mutex::new(RefCell::new(BridgeState::new()));

/// The USART6 link, protected by a critical-section mutex.
pub static LINK: Mutex<RefCell<Option<UartLink>>> = Mutex::new(RefCell::new(None));

/// Virtual stable platform emulator (ADR-016).
/// Written by the TIM7 ISR; read by `BoardImu` trait methods.
pub static PLATFORM: Mutex<RefCell<agc_imu_platform::PlatformEmulator>> =
    Mutex::new(RefCell::new(agc_imu_platform::PlatformEmulator::caged()));

/// BMI088 driver singleton; populated during init, read by TIM7 ISR.
pub static BMI088: Mutex<RefCell<Option<local::imu::bmi088::Bmi088Driver>>> =
    Mutex::new(RefCell::new(None));

// ── Convenience accessors ─────────────────────────────────────────────────────

/// Borrow both `LINK` and `BRIDGE` inside a single critical section and call
/// `f(link, bridge_state)`.  The closure is not called if `LINK` is `None`
/// (i.e. before initialisation).
pub fn with_bridge_and_link<F>(f: F)
where
    F: FnOnce(&mut UartLink, &mut BridgeState),
{
    cortex_m::interrupt::free(|cs| {
        let mut link_opt = LINK.borrow(cs).borrow_mut();
        let mut bridge = BRIDGE.borrow(cs).borrow_mut();
        if let Some(link) = link_opt.as_mut() {
            f(link, &mut bridge);
        }
    });
}

/// Run `f` with a mutable reference to the platform emulator.
pub fn with_platform<F>(f: F)
where
    F: FnOnce(&mut agc_imu_platform::PlatformEmulator),
{
    cortex_m::interrupt::free(|cs| {
        f(&mut crate::PLATFORM.borrow(cs).borrow_mut());
    });
}

// ── Board ─────────────────────────────────────────────────────────────────────

/// Top-level board handle.  Zero-sized fields keep `Board` trivially copyable
/// and avoid any stack allocation for peripheral handles (all state is in the
/// global statics above).
pub struct Board {
    pub dsky: RemoteDsky,
    pub imu: BoardImu,
    pub optics: RemoteOptics,
    pub engine: RemoteEngine,
    pub rcs: RemoteRcs,
    pub uplink: RemoteUplink,
    pub telemetry: RemoteTelemetry,
    pub timers: LocalTimers,
}

impl Board {
    /// Construct the `Board` handle.  Peripherals must already have been
    /// initialised (clocks, USART6, IWDG) and `LINK` populated before calling
    /// any trait methods.
    pub const fn new() -> Self {
        Self {
            dsky: RemoteDsky,
            imu: BoardImu,
            optics: RemoteOptics,
            engine: RemoteEngine,
            rcs: RemoteRcs,
            uplink: RemoteUplink,
            telemetry: RemoteTelemetry,
            timers: LocalTimers,
        }
    }
}

impl AgcHardware for Board {
    type Timers = LocalTimers;
    type Dsky = RemoteDsky;
    type Imu = BoardImu;
    type Optics = RemoteOptics;
    type Engine = RemoteEngine;
    type Rcs = RemoteRcs;
    type Uplink = RemoteUplink;
    type Telemetry = RemoteTelemetry;

    fn timers(&mut self) -> &mut Self::Timers {
        &mut self.timers
    }
    fn dsky(&mut self) -> &mut Self::Dsky {
        &mut self.dsky
    }
    fn imu(&mut self) -> &mut Self::Imu {
        &mut self.imu
    }
    fn optics(&mut self) -> &mut Self::Optics {
        &mut self.optics
    }
    fn engine(&mut self) -> &mut Self::Engine {
        &mut self.engine
    }
    fn rcs(&mut self) -> &mut Self::Rcs {
        &mut self.rcs
    }
    fn uplink(&mut self) -> &mut Self::Uplink {
        &mut self.uplink
    }
    fn telemetry(&mut self) -> &mut Self::Telemetry {
        &mut self.telemetry
    }

    fn pet_watchdog(&mut self) {
        crate::pet_watchdog_global();
    }

    fn hardware_restart(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

// ── Watchdog global shim ──────────────────────────────────────────────────────

use core::sync::atomic::{AtomicBool, Ordering};

static WDG_READY: AtomicBool = AtomicBool::new(false);

static WDG_PET: Mutex<RefCell<Option<fn()>>> = Mutex::new(RefCell::new(None));

/// Register the watchdog pet function.  Called once from `bin/agc.rs` after
/// `Watchdog::init`.
pub fn register_watchdog_pet(f: fn()) {
    cortex_m::interrupt::free(|cs| {
        *WDG_PET.borrow(cs).borrow_mut() = Some(f);
    });
    WDG_READY.store(true, Ordering::Relaxed);
}

fn pet_watchdog_global() {
    if WDG_READY.load(Ordering::Relaxed) {
        cortex_m::interrupt::free(|cs| {
            if let Some(f) = *WDG_PET.borrow(cs).borrow() {
                f();
            }
        });
    }
}
