//! IMU (Inertial Measurement Unit) I/O interface with typestate alignment markers.
//!
//! AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (CDUX/Y/Z octal 32-34,
//!             PIPAX/Y/Z octal 37-41, GYROCTR/GYROCMD octal 47)
//!             Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc (IMUCOARS, IMUFINE,
//!             IMUZERO, SETCOARS; pages 1420-1432)
//!             Comanche055/T4RUPT_PROGRAM.agc (IMUMON, pages 139-143)
//! Channels:   CHAN12 (octal 12) — IMU control discretes
//!             CHAN14 (octal 14) — ISS CDU pulse commands
//!             CHAN30 (octal 30) — IMU status bits (input)

use crate::types::{CduAngle, Mat3x3, IDENTITY_MAT3};
use core::marker::PhantomData;

// ── IMU typestate markers ─────────────────────────────────────────────────────

/// Marker: IMU has not yet been coarse-aligned.
///
/// In this state CDU error counters are disabled and gyros are not commanding
/// the stable member.
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO routine.
pub struct Unaligned;

/// Marker: IMU coarse alignment complete; CDU error counters enabled.
///
/// The software iteratively drives COMMAND registers (via channel 14) to
/// rotate gimbals to the desired THETAD orientation.
/// Tolerance: within 2 degrees (COARSTOL in IMU_MODE_SWITCHING_ROUTINES.agc page 1425).
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUCOARS routine, page 1423.
pub struct CoarseAligned;

/// Marker: IMU fine alignment complete; gyro torque available.
///
/// Zero and coarse discrete bits cleared. DAP enabled.
/// Gyro torque commands (GYROCMD, octal 47) available for fine alignment.
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUFINE routine, page 1427.
pub struct FineAligned;

// ── ImuIo trait ───────────────────────────────────────────────────────────────

/// IMU I/O interface.
///
/// Provides access to CDU angles, PIPA delta-V accumulators, and gyro torque.
/// The bare-metal implementation enforces alignment state via typestate
/// parameters on the concrete struct (`ImuImpl<Unaligned>`, etc.).
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (CDUX-Z octal 32-34,
///             PIPAX-Z octal 37-41, GYROCMD octal 47).
///             Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc.
pub trait ImuIo {
    /// Read the three IMU CDU gimbal angles (inner/middle/outer axes X/Y/Z).
    ///
    /// Returns `[CduAngle; 3]` = `[CDUX, CDUY, CDUZ]`.
    /// - CDUX = octal 32, CDUY = octal 33, CDUZ = octal 34.
    ///   The middle gimbal angle (CDUZ) is monitored for gimbal lock (|angle| > 70°).
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc CDU register layout.
    fn read_cdu(&self) -> [CduAngle; 3];

    /// Read and clear the PIPA delta-V counters since the last call.
    ///
    /// Returns `[i16; 3]` = `[PIPAX, PIPAY, PIPAZ]`, in raw pulse counts.
    /// Scale: 1 count = `PIPA_SCALE` = 0.0585 m/s (from SERVICER207.agc KPIP1).
    /// - PIPAX = octal 37, PIPAY = octal 40, PIPAZ = octal 41.
    ///
    /// Reading clears the accumulators. Must be called from SERVICER (2 s cycle).
    ///
    /// AGC source: Comanche055/SERVICER207.agc PIPA read loop.
    fn read_pipa(&mut self) -> [i16; 3];

    /// Send gyro torque pulses for fine alignment (GYROCMD, octal 47).
    ///
    /// `axis`: 0=X, 1=Y, 2=Z.
    /// `pulses`: signed count of torque pulses. Positive = one direction.
    ///
    /// Only meaningful in `FineAligned` state; callers must ensure correct state.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc GYROCMD usage.
    fn torque_gyro(&mut self, axis: usize, pulses: i16);

    /// Read IMU status bits from channel 30.
    ///
    /// Bit 15: temp in limits, Bit 14: ISS turn-on request,
    /// Bit 13: IMU fail, Bit 12: CDU fail, Bit 11: IMU cage, Bit 9: IMU operate.
    ///
    /// AGC source: Comanche055/T4RUPT_PROGRAM.agc IMUMON routine.
    fn read_status(&self) -> u16;

    /// Write IMU control discrete bits to channel 12.
    ///
    /// Bit 4: coarse align enable. Bit 5: ISS CDU zero. Bit 6: error counter enable.
    /// Bit 10: gyro activity inhibit. Bit 15: ISS delay complete.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc SETCOARS, IMUZERO, IMUFINE.
    fn write_control(&mut self, bits: u16);

    /// Write ISS CDU pulse commands to channel 14 (CDUXCMD/CDUYCMD/CDUZCMD).
    ///
    /// Used during coarse alignment to drive the gimbals to the desired angle.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUCOARS SENDPULS label.
    fn write_cdu_commands(&mut self, cmds: [i16; 3]);

    /// True if the IMU is currently in coarse-align mode (channel 12 bit 4).
    ///
    /// Used by fresh_start post-conditions check.
    fn coarse_align_active(&self) -> bool;

    /// Place IMU in coarse align mode (set CHAN12 bit 4).
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc SETCOARS label.
    fn set_coarse_align(&mut self);

    /// Return the current REFSMMAT (Reference Stable-Member Matrix).
    ///
    /// REFSMMAT is the rotation matrix from Stable-Member (SM) frame to ECI.
    /// Its transpose (`transpose(refsmmat)`) converts SM-frame PIPA readings to ECI.
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc `REFSMMAT ERASE +17D # I(18D)PRM`.
    ///             Comanche055/SERVICER207.agc CALCRVG uses REFSMMAT to rotate DELV to ECI.
    fn refsmmat(&self) -> Mat3x3;
}

// ── Typestate concrete implementation skeleton ────────────────────────────────

/// Bare-metal IMU implementation with typestate alignment tracking.
///
/// `State` is one of `Unaligned`, `CoarseAligned`, `FineAligned`.
/// The compiler prevents calling fine-alignment methods before
/// `into_coarse_aligned()` etc.
///
/// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc alignment state machine.
pub struct ImuImpl<State> {
    /// Channel 12 control register shadow (last written value).
    control_shadow: u16,
    /// CDU angle register mirrors (updated by SPI reads).
    cdus: [CduAngle; 3],
    /// PIPA accumulator mirrors.
    pipas: [i16; 3],
    /// Channel 30 status shadow.
    status: u16,
    /// Reference Stable-Member Matrix (SM → ECI rotation).
    ///
    /// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc `REFSMMAT ERASE +17D`.
    refsmmat: Mat3x3,
    _state: PhantomData<State>,
}

impl ImuImpl<Unaligned> {
    /// Construct a new IMU implementation in the `Unaligned` state.
    ///
    /// Called once during hardware initialisation before any alignment sequence.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUZERO entry.
    pub fn new() -> Self {
        Self {
            control_shadow: 0,
            cdus: [CduAngle(0); 3],
            pipas: [0; 3],
            status: 0,
            refsmmat: IDENTITY_MAT3,
            _state: PhantomData,
        }
    }

    /// Begin coarse alignment sequence.
    ///
    /// Sets CHAN12 bit 4 (coarse align enable) and bit 6 (error counter enable).
    /// Returns the IMU in `CoarseAligned` state.
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUCOARS routine, page 1423.
    pub fn into_coarse_aligned(mut self) -> ImuImpl<CoarseAligned> {
        // Set bit 4 (coarse align) and bit 6 (error counter enable).
        self.control_shadow |= (1 << 3) | (1 << 5); // bits 4 and 6 in 1-based = bits 3 and 5 in 0-based
        ImuImpl {
            control_shadow: self.control_shadow,
            cdus: self.cdus,
            pipas: self.pipas,
            status: self.status,
            refsmmat: self.refsmmat,
            _state: PhantomData,
        }
    }

    /// Release the underlying peripheral handle (C-FREE).
    pub fn free(self) -> u16 {
        self.control_shadow
    }
}

impl Default for ImuImpl<Unaligned> {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuImpl<CoarseAligned> {
    /// Transition to fine alignment (clears zero and coarse bits).
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc IMUFINE routine, page 1427.
    pub fn into_fine_aligned(mut self) -> ImuImpl<FineAligned> {
        // Clear coarse align (bit 4) and CDU zero (bit 5) bits.
        self.control_shadow &= !((1 << 3) | (1 << 4));
        ImuImpl {
            control_shadow: self.control_shadow,
            cdus: self.cdus,
            pipas: self.pipas,
            status: self.status,
            refsmmat: self.refsmmat,
            _state: PhantomData,
        }
    }

    /// Release the underlying peripheral handle (C-FREE).
    pub fn free(self) -> u16 {
        self.control_shadow
    }
}

impl ImuImpl<FineAligned> {
    /// Revert to unaligned state (e.g., on IMU fail).
    ///
    /// AGC source: Comanche055/IMU_MODE_SWITCHING_ROUTINES.agc MODABORT path.
    pub fn into_unaligned(mut self) -> ImuImpl<Unaligned> {
        self.control_shadow = 0;
        ImuImpl {
            control_shadow: self.control_shadow,
            cdus: self.cdus,
            pipas: self.pipas,
            status: self.status,
            refsmmat: self.refsmmat,
            _state: PhantomData,
        }
    }

    /// Release the underlying peripheral handle (C-FREE).
    pub fn free(self) -> u16 {
        self.control_shadow
    }
}

/// Shared ImuIo implementation for all states.
macro_rules! impl_imu_io {
    ($state:ty) => {
        impl ImuIo for ImuImpl<$state> {
            fn read_cdu(&self) -> [CduAngle; 3] {
                self.cdus
            }

            fn read_pipa(&mut self) -> [i16; 3] {
                let pipas = self.pipas;
                self.pipas = [0; 3];
                pipas
            }

            fn torque_gyro(&mut self, _axis: usize, _pulses: i16) {
                // No-op on bare-metal skeleton; real impl writes to GYROCMD (octal 47).
            }

            fn read_status(&self) -> u16 {
                self.status
            }

            fn write_control(&mut self, bits: u16) {
                self.control_shadow = bits;
            }

            fn write_cdu_commands(&mut self, _cmds: [i16; 3]) {
                // No-op on bare-metal skeleton; real impl writes to CHAN14.
            }

            fn coarse_align_active(&self) -> bool {
                // Bit 4 in 1-based = bit 3 in 0-based
                (self.control_shadow >> 3) & 1 != 0
            }

            fn set_coarse_align(&mut self) {
                self.control_shadow |= 1 << 3;
            }

            fn refsmmat(&self) -> Mat3x3 {
                self.refsmmat
            }
        }
    };
}

impl_imu_io!(Unaligned);
impl_imu_io!(CoarseAligned);
impl_imu_io!(FineAligned);

/// Shared test/sim helpers for all `ImuImpl<State>` variants.
///
/// These methods are not part of the `ImuIo` trait — they exist only for
/// simulator injection and unit testing.
macro_rules! impl_imu_test_helpers {
    ($state:ty) => {
        impl ImuImpl<$state> {
            /// Inject PIPA counts into the accumulator (for simulator / unit tests).
            ///
            /// The next call to `read_pipa` will return these counts and zero them.
            pub fn inject_pipa(&mut self, counts: [i16; 3]) {
                self.pipas = counts;
            }

            /// Set the REFSMMAT (for simulator / unit tests).
            ///
            /// AGC: REFSMMAT is loaded during P52 IMU alignment.
            pub fn set_refsmmat(&mut self, m: Mat3x3) {
                self.refsmmat = m;
            }
        }
    };
}

impl_imu_test_helpers!(Unaligned);
impl_imu_test_helpers!(CoarseAligned);
impl_imu_test_helpers!(FineAligned);
