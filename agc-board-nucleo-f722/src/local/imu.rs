//! Local IMU stub.
//!
//! The STM32F722ZE has no on-chip IMU.  A physical breakout board (e.g.
//! ICM-42688-P) connected over SPI is out of scope for Phase 1.
//! This stub returns zeros and logs a one-time warning on first `read_pipa`
//! call so test sessions on real hardware are immediately obvious.

use core::sync::atomic::{AtomicBool, Ordering};

use agc_core::hal::imu::Imu;
use agc_core::types::CduAngle;

static PIPA_WARNED: AtomicBool = AtomicBool::new(false);

/// Zero-sized stub IMU.
pub struct LocalImu;

impl Imu for LocalImu {
    fn read_pipa(&mut self) -> [i16; 3] {
        if !PIPA_WARNED.swap(true, Ordering::Relaxed) {
            defmt::warn!("LocalImu::read_pipa called — stub returns zeros (Phase-1 board)");
        }
        [0; 3]
    }

    fn read_cdu(&self) -> [CduAngle; 3] {
        [CduAngle(0); 3]
    }

    fn torque_gyro(&mut self, _axis: usize, _pulses: i16) {}

    fn coarse_align(&mut self, _commands: [i16; 3]) {}

    fn is_caged(&self) -> bool {
        true
    }
}
