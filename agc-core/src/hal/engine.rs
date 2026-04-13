//! SPS main engine I/O interface.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
//!             EMSD/THRUST = octal 55 (engine on/off discrete)
//!             CDUTCMD = octal 53 = TVCYAW (SPS yaw gimbal)
//!             CDUSCMD = octal 54 = TVCPITCH (SPS pitch gimbal)
//!             CHAN13 = octal 13 (SPS/TVC discrete outputs)

use crate::types::CduAngle;

/// SPS thrust: 91,188.544 N.
/// AGC source: docs/agc-reference-constants.md.
pub const SPS_THRUST_N: f64 = 91_188.544;

/// SPS effective exhaust velocity: 3151.0396 m/s.
/// AGC source: docs/agc-reference-constants.md.
pub const SPS_VE_MS: f64 = 3151.0396;

/// SPS main engine I/O interface.
///
/// Provides engine enable/disable and gimbal trim commands.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
/// EMSD/THRUST = octal 55 (engine on/off).
/// TVCPITCH = CDUSCMD = octal 54 (SPS pitch gimbal).
/// TVCYAW = CDUTCMD = octal 53 (SPS yaw gimbal).
pub trait EngineIo {
    /// Enable (fire) or disable the SPS engine.
    ///
    /// Corresponds to setting/clearing EMSD (octal 55).
    ///
    /// AGC source: Comanche055/P40-P47.agc IGNITION sequence.
    fn set_engine_enable(&mut self, enabled: bool);

    /// True if the engine is currently commanded on.
    fn engine_enabled(&self) -> bool;

    /// Command SPS gimbal trim in pitch axis.
    ///
    /// Units: CDU pulse counts (signed i16). Positive = pitch up.
    /// Writes to TVCPITCH = CDUSCMD (octal 54).
    ///
    /// AGC source: Comanche055/TVC routines, TVCPITCH register.
    fn trim_pitch(&mut self, delta: i16);

    /// Command SPS gimbal trim in yaw axis.
    ///
    /// Units: CDU pulse counts (signed i16). Positive = yaw right.
    /// Writes to TVCYAW = CDUTCMD (octal 53).
    ///
    /// AGC source: Comanche055/TVC routines, TVCYAW register.
    fn trim_yaw(&mut self, delta: i16);

    /// Read back the current pitch gimbal position estimate.
    fn read_tvc_pitch(&self) -> CduAngle;

    /// Read back the current yaw gimbal position estimate.
    fn read_tvc_yaw(&self) -> CduAngle;

    /// Write CHAN11 (DSALMOUT) bits for engine state indication.
    ///
    /// Bit 13 in DSALMOUT = engine on indicator lamp.
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc DSALMOUT = octal 11.
    fn write_dsalmout(&mut self, bits: u16);
}

/// Bare-metal SPS engine implementation skeleton.
///
/// AGC source: Comanche055/P40-P47.agc engine control routines.
pub struct EngineImpl {
    enabled: bool,
    tvc_pitch: CduAngle,
    tvc_yaw: CduAngle,
    dsalmout: u16,
}

impl EngineImpl {
    /// Construct with engine disabled and gimbals at zero.
    pub const fn new() -> Self {
        Self {
            enabled: false,
            tvc_pitch: CduAngle(0),
            tvc_yaw: CduAngle(0),
            dsalmout: 0,
        }
    }

    /// Release the underlying peripheral handle (C-FREE).
    pub fn free(self) -> u16 {
        self.dsalmout
    }
}

impl Default for EngineImpl {
    fn default() -> Self {
        Self::new()
    }
}

impl EngineIo for EngineImpl {
    fn set_engine_enable(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    fn engine_enabled(&self) -> bool {
        self.enabled
    }

    fn trim_pitch(&mut self, delta: i16) {
        self.tvc_pitch = CduAngle(self.tvc_pitch.0.wrapping_add(delta as u16));
    }

    fn trim_yaw(&mut self, delta: i16) {
        self.tvc_yaw = CduAngle(self.tvc_yaw.0.wrapping_add(delta as u16));
    }

    fn read_tvc_pitch(&self) -> CduAngle {
        self.tvc_pitch
    }

    fn read_tvc_yaw(&self) -> CduAngle {
        self.tvc_yaw
    }

    fn write_dsalmout(&mut self, bits: u16) {
        self.dsalmout = bits;
    }
}
