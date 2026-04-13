//! AGC-Core: Comanche055 Command Module guidance computer, idiomatic Rust port.
//!
//! # `no_std` and embedded constraints
//!
//! This crate targets `thumbv7em-none-eabihf` (Cortex-M4F bare-metal).
//! - No heap: `alloc`, `Vec`, `Box` are forbidden.
//! - No `static mut`: shared mutable state goes through the `sync` module.
//! - Navigation math uses `f64`; CDU / PIPA / channel words use `i16` / `u16`.
//!
//! # Synchronisation facade (`sync` module)
//!
//! Milestone 1 does not yet depend on `cortex-m`. Instead we provide a thin
//! wrapper around the [`critical-section`] crate.
//!
//! For host builds (`cargo test`), a no-op single-threaded implementation is
//! registered via `critical_section::set_impl!` — safe because host tests are
//! single-threaded.
//!
//! For bare-metal builds, a BSP crate in a later milestone will register a
//! real interrupt-disable implementation (e.g. via cortex-m's
//! `critical-section-single-core` feature).
//!
//! Usage pattern (same as `cortex_m::interrupt::free`):
//! ```ignore
//! crate::sync::cs(|cs| {
//!     SOME_MUTEX.borrow(cs).borrow_mut().do_something();
//! });
//! ```
//!
//! # Panic handler
//!
//! On bare-metal (`not(test)`, `not(doc)`), a minimal spin-loop panic handler
//! is registered here.  On host test builds, `std` provides the panic handler.
#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

// ── public modules ────────────────────────────────────────────────────────────
pub mod control;
pub mod executive;
pub mod guidance;
pub mod hal;
pub mod math;
pub mod navigation;
pub mod programs;
pub mod services;
pub mod types;

// ── test helpers (only compiled for host test builds) ─────────────────────────
#[cfg(test)]
pub mod tests;

// ── synchronisation facade ────────────────────────────────────────────────────

/// Thin wrapper around [`critical-section`].
///
/// On host (test builds), a no-op CS implementation is registered.
/// On bare-metal, the BSP crate registers a real implementation.
pub mod sync {
    /// Token proving we are inside a critical section.
    pub use critical_section::CriticalSection as Cs;

    /// Execute `f` inside a critical section.
    ///
    /// On bare-metal this disables interrupts for the duration.
    /// On host (test), it is a no-op (single-threaded host tests).
    #[inline(always)]
    pub fn cs<R>(f: impl FnOnce(Cs<'_>) -> R) -> R {
        critical_section::with(f)
    }

    /// A `Mutex` backed by the critical-section token.
    pub use critical_section::Mutex;
}

/// No-op `critical-section` implementation for host builds.
///
/// # Safety
///
/// This implementation disables no interrupts and acquires no real lock.
/// It is safe ONLY in single-threaded contexts (host unit tests).
/// On the bare-metal target it is replaced by the BSP's implementation.
///
/// AGC source: the AGC's INHINT/RELINT pair is replaced by a real CS on target.
#[cfg(not(any(target_arch = "arm", doc)))]
struct NopCriticalSection;

#[cfg(not(any(target_arch = "arm", doc)))]
// SAFETY: we register a no-op CS only for host (non-ARM) builds, where we
// are guaranteed to be single-threaded (cargo test default).
unsafe impl critical_section::Impl for NopCriticalSection {
    unsafe fn acquire() {}
    unsafe fn release(_token: ()) {}
}

#[cfg(not(any(target_arch = "arm", doc)))]
critical_section::set_impl!(NopCriticalSection);

// ── top-level AgcState ────────────────────────────────────────────────────────
use crate::{
    executive::restart::RestartProtection,
    navigation::state_vector::StateVector,
    services::alarm::AlarmState,
    types::{Mat3x3, Met, Vec3, IDENTITY_MAT3, ZERO_VEC3},
};

/// Flag bits within `AgcState::flags` word.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc FLAGWRD0-8 bit assignments.
#[derive(Clone, Copy, Debug, Default)]
pub struct AgcFlags {
    /// XDELVFLG — external delta-V flag (Flag 2, Bit 8).
    ///
    /// Set by P30 compute_target(); read by P40 on entry.
    /// AGC source: Comanche055/P30-P37.agc XDELVFLG (page 636).
    pub xdelvflg: bool,

    /// UPDATFLG — update-in-progress flag (Flag 1, Bit 7).
    ///
    /// Set during P30/P31 data entry.
    /// AGC source: Comanche055/P30-P37.agc UPDATFLG (page 636).
    pub updatflg: bool,

    /// TRACKFLG — track flag (Flag 1, Bit 5).
    ///
    /// Set during P30/P31 targeting.
    /// AGC source: Comanche055/P30-P37.agc TRACKFLG (page 636).
    pub trackflg: bool,

    /// REFSMFLG — REFSMMAT valid flag.
    ///
    /// Set when REFSMMAT has been computed and is valid for navigation.
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc REFSMFLG.
    pub refsmflg: bool,

    /// .05GSW — 0.05G sensed flag (CM/FLAGS bit 3).
    ///
    /// Set when sensed deceleration reaches 0.05 G during entry.
    /// AGC source: Comanche055/ENTRY_LEXICON.agc `.05GSW = CM/FLAGS bit 3`.
    pub point_05gsw: bool,
}

/// Navigation erasable state used by programs (P11/P30/P37/P40/P61).
///
/// Groups all navigation-related erasable registers that programs read or write.
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (RN, VN, TIG, DELVSLV, etc.).
#[derive(Clone, Copy, Debug)]
pub struct NavState {
    /// Current navigation state vector (RN/VN/PIPTIME).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc RN (B-29 m), VN (B-7 m/cs), PIPTIME.
    pub sv: StateVector,

    /// Time of ignition for P40 (TIG).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc TIG (B-28 cs).
    pub tig: Met,

    /// Delta-V in LVLH frame (DELVSLV).
    ///
    /// AGC source: Comanche055/P30-P37.agc DELVSLV (B+7 m/cs).
    pub delvslv: Vec3,

    /// Delta-V rotated to ECI frame (DELVSIN).
    ///
    /// AGC source: Comanche055/P30-P37.agc DELVSIN (B+7 m/cs).
    pub delvsin: Vec3,

    /// Delta-V magnitude display value (VGDISP), m/s.
    ///
    /// AGC source: Comanche055/P30-P37.agc VGDISP.
    pub vgdisp: f64,

    /// Position at TIG (RTIG), metres.
    ///
    /// AGC source: Comanche055/P30-P37.agc RTIG.
    pub rtig: Vec3,

    /// Velocity at TIG (VTIG), m/s.
    ///
    /// AGC source: Comanche055/P30-P37.agc VTIG.
    pub vtig: Vec3,

    /// Apogee altitude (HAPO), metres above Earth centre.
    ///
    /// AGC source: Comanche055/P30-P37.agc HAPO (B-29 m).
    pub hapo: f64,

    /// Perigee altitude (HPER), metres above Earth centre.
    ///
    /// AGC source: Comanche055/P30-P37.agc HPER (B-29 m).
    pub hper: f64,

    /// Reference Stable-Member Matrix (REFSMMAT).
    ///
    /// 3×3 rotation matrix from stable-member frame to ECI.
    /// AGC source: ERASABLE_ASSIGNMENTS.agc `REFSMMAT ERASE +17D # I(18D)PRM`.
    pub refsmmat: Mat3x3,

    /// Liftoff time (TLIFTOFF), centiseconds.
    ///
    /// AGC source: Comanche055/P11.agc TLIFTOFF.
    pub tliftoff: Met,

    /// Average-G exit hook enable flag.
    ///
    /// When `Some`, the P11 VHHDOT display hook is active.
    /// AGC source: Comanche055/P11.agc AVGEXIT pointer.
    pub avgexit_active: bool,

    /// Vehicle mass, kg.
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc CSMMASS / WEIGHT/G (B-16 kg).
    pub mass_kg: f64,
}

impl NavState {
    /// Construct with default navigation state (zero position/velocity at T=0).
    pub fn new() -> Self {
        Self {
            sv: StateVector::new(ZERO_VEC3, ZERO_VEC3, Met(0)),
            tig: Met(0),
            delvslv: ZERO_VEC3,
            delvsin: ZERO_VEC3,
            vgdisp: 0.0,
            rtig: ZERO_VEC3,
            vtig: ZERO_VEC3,
            hapo: 0.0,
            hper: 0.0,
            refsmmat: IDENTITY_MAT3,
            tliftoff: Met(0),
            avgexit_active: false,
            mass_kg: 28_800.0, // nominal CSM mass at TLI
        }
    }
}

impl Default for NavState {
    fn default() -> Self {
        Self::new()
    }
}

/// The complete AGC erasable-memory state analog.
///
/// All fields correspond to specific erasable memory registers in the AGC.
/// This struct is passed by `&mut` reference through all foreground code.
/// Interrupt-handler-touched state lives in separate `static Mutex` singletons.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (various registers).
pub struct AgcState {
    /// Phase table pairs for restart groups 1–6.
    ///
    /// AGC source: -PHASE1..PHASE6, TBASE1..TBASE6
    /// (ERASABLE_ASSIGNMENTS.agc lines 1759-1847).
    pub restart: RestartProtection,

    /// Alarm history ring buffer (FAILREG, 3 words).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc line 1721.
    pub alarm: AlarmState,

    /// Current major mode register (MODREG).
    ///
    /// `MODREG_NONE` (-1) means "no program" (AGC: -0 / ones-complement zero).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc MODREG.
    pub modreg: i16,

    /// IMU mode flags word 30 (IMODES30).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc IMODES30.
    pub imodes30: u16,

    /// IMU mode flags word 33 (IMODES33).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc IMODES33.
    pub imodes33: u16,

    /// Optics mode flags (OPTMODES).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc OPTMODES.
    pub optmodes: u16,

    /// Restart counter (REDOCTR). Incremented on every GOPROG entry.
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc line 1915.
    pub redoctr: u16,

    /// Erasable-restore flag (ERESTORE). Used by ERASCHK integrity check.
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc ERESTORE.
    pub erestore: u16,

    /// Flag words 0–8 (FLAGWRD0..FLAGWRD8).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc FLAGWRD0-8.
    pub flagwrds: [u16; 9],

    /// Structured program flags (derived from flagwrds for type safety).
    ///
    /// AGC source: ERASABLE_ASSIGNMENTS.agc FLAGWRD1-2 bit assignments.
    pub flags: AgcFlags,

    /// Navigation erasable state (RN, VN, TIG, DELVSLV, REFSMMAT, etc.).
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc navigation area.
    pub nav: NavState,

    /// TEPHEM — ephemeris time base, centiseconds.
    ///
    /// Corrected at liftoff by TLIFTOFF. Used by navigation integration.
    /// AGC source: ERASABLE_ASSIGNMENTS.agc TEPHEM.
    pub tephem: Met,

    /// EXTVBACT — extended verb active flag.
    ///
    /// Set by P61 to lock out extended verbs during reentry guidance.
    /// AGC source: Comanche055/P61-P67.agc EXTVBACT = BIT14.
    pub extvbact: bool,
}

/// Sentinel value for MODREG "no program" (AGC ones-complement -0).
///
/// Written by DOFSTART / MR.KLEAN: `CS ZERO; TS MODREG`.
pub const MODREG_NONE: i16 = -1;

/// IMU mode init value for IMODES30 after fresh start.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc — `IM30INIF = OCT 37411`.
pub const IM30INIF: u16 = 0o37411;

/// IMU mode init value for IMODES33 after fresh start (= PRIO16 = 16).
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc — `IM33INIT = PRIO16`.
pub const IM33INIT: u16 = 16;

/// Optics mode init value.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc — `OPTINITF = OCT 130`.
pub const OPTINITF: u16 = 0o130;

impl AgcState {
    /// Construct a zero-initialised state (safe power-on default).
    pub fn new() -> Self {
        Self {
            restart: RestartProtection::new(),
            alarm: AlarmState::new(),
            modreg: MODREG_NONE,
            imodes30: 0,
            imodes33: 0,
            optmodes: 0,
            redoctr: 0,
            erestore: 0,
            flagwrds: [0; 9],
            flags: AgcFlags::default(),
            nav: NavState::new(),
            tephem: Met(0),
            extvbact: false,
        }
    }
}

impl Default for AgcState {
    fn default() -> Self {
        Self::new()
    }
}

// ── bare-metal panic handler ──────────────────────────────────────────────────
//
// Defined only for ARM bare-metal targets where std is not available.
// Host tests use std's built-in panic handler.
// This is gated on `target_arch = "arm"` rather than `cfg(not(test))` to
// avoid duplicate lang-item conflicts when std is linked for host tests.

#[cfg(all(target_arch = "arm", not(test)))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // Spin. The hardware watchdog will trigger GOJAM.
    // SAFETY: intentional infinite loop — this is the AGC GOJAM equivalent.
    // The real restart path (GOPROG) will be wired in a later milestone.
    loop {
        core::hint::spin_loop();
    }
}
