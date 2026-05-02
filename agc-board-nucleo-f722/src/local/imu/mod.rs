//! Real `Imu` trait impl backed by the BMI088 + `agc_imu_platform` emulator.

pub mod bmi088;

use agc_core::hal::imu::Imu;
use agc_core::types::CduAngle;

/// Zero-sized handle; all state lives in `crate::PLATFORM`.
pub struct BoardImu;

impl Imu for BoardImu {
    fn read_pipa(&mut self) -> [i16; 3] {
        let mut counts = [0i16; 3];
        cortex_m::interrupt::free(|cs| {
            counts = crate::PLATFORM.borrow(cs).borrow_mut().read_pipa().0;
        });
        counts
    }

    fn read_cdu(&self) -> [CduAngle; 3] {
        cortex_m::interrupt::free(|cs| {
            let raw = crate::PLATFORM.borrow(cs).borrow().read_cdu();
            [CduAngle(raw[0]), CduAngle(raw[1]), CduAngle(raw[2])]
        })
    }

    fn torque_gyro(&mut self, axis: usize, pulses: i16) {
        cortex_m::interrupt::free(|cs| {
            crate::PLATFORM
                .borrow(cs)
                .borrow_mut()
                .torque_gyro(axis, pulses);
        });
    }

    fn coarse_align(&mut self, commands: [i16; 3]) {
        cortex_m::interrupt::free(|cs| {
            crate::PLATFORM
                .borrow(cs)
                .borrow_mut()
                .coarse_align(commands);
        });
    }

    fn is_caged(&self) -> bool {
        cortex_m::interrupt::free(|cs| crate::PLATFORM.borrow(cs).borrow().caged)
    }
}
