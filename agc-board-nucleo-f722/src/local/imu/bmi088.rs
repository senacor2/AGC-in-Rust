//! BMI088 SPI3 driver.
//!
//! Owns the SPI3 peripheral and the two chip-select pins (PA15 for the
//! accelerometer, PB12 for the gyroscope).  Each BMI088 die has its own
//! CS line and register map.
//!
//! Bench-tested against the Adafruit BMI088 breakout (#4836). The Bosch
//! shuttle board uses the same silicon and is wire-compatible with this
//! driver via a 1.27→2.54 mm header adapter.
//!
//! ## SPI protocol notes
//! - Mode 0, MSB first, 8-bit words.
//! - Accel reads: assert CS_ACCEL, send address byte (bit 7 = 1 for read),
//!   receive one dummy byte, then receive the data byte(s), deassert.
//! - Gyro reads: assert CS_GYRO, send address byte (bit 7 = 1 for read),
//!   receive data byte(s) directly (no dummy byte), deassert.
//!
//! ## Reference
//! BMI088 datasheet rev 1.9 (Bosch BST-BMI088-DS001-19), §5 (accel) and §6 (gyro).

// embedded-hal 0.2 blocking SPI trait (implemented by stm32f7xx-hal 0.8).
// The board crate depends on both embedded-hal 1.0 and 0.2; the 0.2 version
// is aliased as `embedded-hal-02` in Cargo.toml to avoid name collision.
use embedded_hal_02::blocking::spi::Transfer;

use stm32f7xx_hal::{
    gpio::{Alternate, Output, Pin, PushPull},
    pac::SPI3,
    spi::{Enabled, Spi},
};

// ── Register addresses ────────────────────────────────────────────────────────

// Accelerometer (CS_ACCEL)
const ACC_CHIP_ID: u8 = 0x00;
const ACC_SOFTRESET: u8 = 0x7E;
const ACC_PWR_CTRL: u8 = 0x7D;
const ACC_PWR_CONF: u8 = 0x7C;
const ACC_RANGE: u8 = 0x41;
const ACC_CONF: u8 = 0x40;
const ACC_X_LSB: u8 = 0x12;

// Gyroscope (CS_GYRO)
const GYRO_CHIP_ID: u8 = 0x00;
const GYRO_SOFTRESET: u8 = 0x14;
const GYRO_RANGE: u8 = 0x0F;
const GYRO_BANDWIDTH: u8 = 0x10;
const RATE_X_LSB: u8 = 0x02;

// Expected chip IDs
const ACC_CHIP_ID_EXPECTED: u8 = 0x1E;
const GYRO_CHIP_ID_EXPECTED: u8 = 0x0F;

// ── Scale factors ─────────────────────────────────────────────────────────────

/// LSB-to-m/s² for ACC_RANGE = 0x01 (±6 g).
/// Range = 1.5 × 2^(reg+1) g = 6 g.  Full-scale = 6 × g = 6 × 9.80665 m/s².
/// 16-bit signed → 32768 counts = full scale (positive half).
const LSB_TO_MPS2: f64 = (6.0 * 9.806_65) / 32768.0;

/// LSB-to-rad/s for GYRO_RANGE = 0x01 (±1000 dps).
/// 32768 counts = 1000 deg/s.
const LSB_TO_RAD_PER_S: f64 = (1000.0 * core::f64::consts::PI / 180.0) / 32768.0;

// ── Types ─────────────────────────────────────────────────────────────────────

type Spi3Pins = (
    Pin<'B', 3, Alternate<6>>,
    Pin<'B', 4, Alternate<6>>,
    Pin<'B', 5, Alternate<6>>,
);
pub type Spi3Enabled = Spi<SPI3, Spi3Pins, Enabled<u8>>;
type CsAccel = Pin<'A', 15, Output<PushPull>>;
type CsGyro = Pin<'B', 12, Output<PushPull>>;

/// Error type for BMI088 initialisation.
#[derive(Debug)]
pub enum InitError {
    /// Accelerometer returned unexpected chip ID.
    AccelChipIdMismatch(u8),
    /// Gyroscope returned unexpected chip ID.
    GyroChipIdMismatch(u8),
    /// SPI transfer failed.
    SpiError,
}

/// BMI088 driver owning SPI3 and both chip-select lines.
pub struct Bmi088Driver {
    spi: Spi3Enabled,
    cs_accel: CsAccel,
    cs_gyro: CsGyro,
}

impl Bmi088Driver {
    /// Initialise the BMI088, verify chip IDs, configure power/range/ODR.
    pub fn init(spi: Spi3Enabled, cs_accel: CsAccel, cs_gyro: CsGyro) -> Result<Self, InitError> {
        let mut driver = Self {
            spi,
            cs_accel,
            cs_gyro,
        };

        // ── Soft reset both dies ──────────────────────────────────────────────
        driver.accel_write(ACC_SOFTRESET, 0xB6)?;
        driver.gyro_write(GYRO_SOFTRESET, 0xB6)?;
        // Wait ≥ 30 ms.  At 216 MHz, 216_000 cycles ≈ 1 ms.
        cortex_m::asm::delay(216_000 * 35);

        // ── Accel CS toggle to activate SPI mode (datasheet §6.2) ────────────
        // The first SPI access after power-on must be a CS assert/deassert
        // so the accel recognises the SPI protocol.
        driver.cs_accel.set_high();
        cortex_m::asm::delay(1_000);
        driver.cs_accel.set_low();
        cortex_m::asm::delay(100);
        driver.cs_accel.set_high();
        cortex_m::asm::delay(1_000);

        // ── Verify chip IDs ───────────────────────────────────────────────────
        let acc_id = driver.accel_read_byte(ACC_CHIP_ID)?;
        if acc_id != ACC_CHIP_ID_EXPECTED {
            return Err(InitError::AccelChipIdMismatch(acc_id));
        }

        let gyro_id = driver.gyro_read_byte(GYRO_CHIP_ID)?;
        if gyro_id != GYRO_CHIP_ID_EXPECTED {
            return Err(InitError::GyroChipIdMismatch(gyro_id));
        }

        defmt::info!(
            "BMI088: accel chip_id=0x{:02X} gyro chip_id=0x{:02X}",
            acc_id,
            gyro_id
        );

        // ── Configure accelerometer ───────────────────────────────────────────
        driver.accel_write(ACC_PWR_CTRL, 0x04)?; // enable accel
        cortex_m::asm::delay(216_000 * 5);
        driver.accel_write(ACC_PWR_CONF, 0x00)?; // active mode
        cortex_m::asm::delay(216_000 * 5);
        driver.accel_write(ACC_RANGE, 0x01)?; // ±6 g
        driver.accel_write(ACC_CONF, 0xAB)?; // ODR 1600 Hz, OSR4

        // ── Configure gyroscope ───────────────────────────────────────────────
        driver.gyro_write(GYRO_RANGE, 0x01)?; // ±1000 dps
        driver.gyro_write(GYRO_BANDWIDTH, 0x02)?; // ODR 1000 Hz / 116 Hz BW

        Ok(driver)
    }

    /// Read gyroscope angular rates; returns [x, y, z] in rad/s.
    pub fn read_gyro_rad_s(&mut self) -> [f64; 3] {
        let raw = self.gyro_read_6(RATE_X_LSB);
        [
            i16::from_le_bytes([raw[0], raw[1]]) as f64 * LSB_TO_RAD_PER_S,
            i16::from_le_bytes([raw[2], raw[3]]) as f64 * LSB_TO_RAD_PER_S,
            i16::from_le_bytes([raw[4], raw[5]]) as f64 * LSB_TO_RAD_PER_S,
        ]
    }

    /// Read accelerometer linear acceleration; returns [x, y, z] in m/s².
    pub fn read_accel_mps2(&mut self) -> [f64; 3] {
        let raw = self.accel_read_6(ACC_X_LSB);
        [
            i16::from_le_bytes([raw[0], raw[1]]) as f64 * LSB_TO_MPS2,
            i16::from_le_bytes([raw[2], raw[3]]) as f64 * LSB_TO_MPS2,
            i16::from_le_bytes([raw[4], raw[5]]) as f64 * LSB_TO_MPS2,
        ]
    }

    /// Return the raw peripherals (C-FREE).
    pub fn free(self) -> (Spi3Enabled, CsAccel, CsGyro) {
        (self.spi, self.cs_accel, self.cs_gyro)
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    fn accel_write(&mut self, reg: u8, val: u8) -> Result<(), InitError> {
        let mut buf = [reg & 0x7F, val];
        self.cs_accel.set_low();
        let result = self.spi.transfer(&mut buf);
        self.cs_accel.set_high();
        result.map(|_| ()).map_err(|_| InitError::SpiError)
    }

    fn gyro_write(&mut self, reg: u8, val: u8) -> Result<(), InitError> {
        let mut buf = [reg & 0x7F, val];
        self.cs_gyro.set_low();
        let result = self.spi.transfer(&mut buf);
        self.cs_gyro.set_high();
        result.map(|_| ()).map_err(|_| InitError::SpiError)
    }

    /// Read one byte from an accelerometer register.
    /// Accel SPI reads require one dummy byte: [addr|0x80, dummy, data].
    fn accel_read_byte(&mut self, reg: u8) -> Result<u8, InitError> {
        let mut buf = [reg | 0x80, 0x00, 0x00];
        self.cs_accel.set_low();
        let ok = self.spi.transfer(&mut buf).is_ok();
        self.cs_accel.set_high();
        if ok {
            Ok(buf[2])
        } else {
            Err(InitError::SpiError)
        }
    }

    /// Read one byte from a gyroscope register (no dummy byte).
    fn gyro_read_byte(&mut self, reg: u8) -> Result<u8, InitError> {
        let mut buf = [reg | 0x80, 0x00];
        self.cs_gyro.set_low();
        let ok = self.spi.transfer(&mut buf).is_ok();
        self.cs_gyro.set_high();
        if ok {
            Ok(buf[1])
        } else {
            Err(InitError::SpiError)
        }
    }

    /// Burst-read 6 data bytes from the accelerometer starting at `reg`.
    /// Layout after dummy byte: [XL, XH, YL, YH, ZL, ZH].
    fn accel_read_6(&mut self, reg: u8) -> [u8; 6] {
        // 1 addr + 1 dummy + 6 data = 8 bytes total
        let mut buf = [0u8; 8];
        buf[0] = reg | 0x80;
        self.cs_accel.set_low();
        self.spi.transfer(&mut buf).ok();
        self.cs_accel.set_high();
        [buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]]
    }

    /// Burst-read 6 data bytes from the gyroscope starting at `reg`.
    /// Layout: [XL, XH, YL, YH, ZL, ZH] — no dummy byte.
    fn gyro_read_6(&mut self, reg: u8) -> [u8; 6] {
        // 1 addr + 6 data = 7 bytes total
        let mut buf = [0u8; 7];
        buf[0] = reg | 0x80;
        self.cs_gyro.set_low();
        self.spi.transfer(&mut buf).ok();
        self.cs_gyro.set_high();
        [buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]]
    }
}
