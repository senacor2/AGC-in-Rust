//! Backup state — the survives-RESTART subset of [`AgcState`].
//!
//! On the bare-metal target this struct will be placed in the STM32F767ZI's
//! 4 KB BKPSRAM region (`0x4002_4000`, see project memory
//! `project_battery_backed_bkpsram.md`). The BKPSRAM is held by a CR2032 on
//! V_BAT, so its contents survive a hardware RESTART (watchdog timeout, GOJAM,
//! brief power blip). On the host this is just a normal struct used by tests
//! to drive snapshot/restore round-trips.
//!
//! # FRESH START vs RESTART
//!
//! - **FRESH START** scrubs everything. The bare-metal boot path will call
//!   [`invalidate`] on the BKPSRAM region as part of FRESH START so the next
//!   boot can't mistake stale data for a valid backup.
//! - **RESTART** preserves navigation state. The bare-metal boot path will
//!   read the BKPSRAM region, call [`restore_from_backup`], and recover the
//!   fields below. If the magic word, version, or checksum doesn't match,
//!   the boot path falls back to FRESH START.
//!
//! # What's in the backup
//!
//! Only the subset of [`AgcState`] that cannot be recreated from sensors
//! within one cycle:
//!
//! | Field | Why preserved |
//! |---|---|
//! | `csm_state`, `target_state` | nav state — losing it kills the mission |
//! | `refsmmat` | IMU-to-inertial; re-deriving needs P51/P52 alignment |
//! | `time` | mission clock |
//! | `gha_epoch_rad` | Mission Control uplink, expensive to re-acquire |
//! | `restart` (phase registers) | the whole point of restart safety |
//! | `pipa_cal`, `gyro_comp`, `last_drift_comp_time` | uplink calibration |
//! | `imu_alignment_state` | hardware platform's actual state |
//! | `tpi_arrival_epoch` | active rendezvous arrival time |
//! | `major_mode` | so RESTART knows what program to re-enter |
//! | `flagwords` | persistent flags (ENGINEON etc.) |
//!
//! Everything else (scheduler, DAP/TVC, staging fields, V/N input state,
//! display, alarm, marks counters, …) is rebuilt on RESTART by the
//! `restart_with_table` flow.
//!
//! # Header layout (16 bytes)
//!
//! ```text
//! offset | bytes | field
//! -------+-------+----------------------------------------
//!     0  |   4   | magic    = b"AGC1"
//!     4  |   4   | version  (u32; bump on any layout change)
//!     8  |   4   | crc32    (CRC-32/IEEE over everything except this field)
//!    12  |   4   | _pad     (reserved; keeps payload 8-byte-aligned)
//! ```
//!
//! The CRC field is at a fixed offset so the CRC computation can skip it.
//!
//! # Serialisation to BKPSRAM
//!
//! `BackupState` is `#[repr(C)]` so its outer field order is stable. The
//! bare-metal binary will treat the BKPSRAM region as a `*mut BackupState`
//! and read/write fields directly — no separate (de)serialise step. Inner
//! types (`StateVector`, `PipaCalibration`, …) keep the default Rust layout;
//! the `version` field detects layout changes between firmware builds.

use crate::control::imu_control::{GyroCompensation, ImuAlignmentState};
use crate::executive::restart::RestartProtection;
use crate::navigation::state_vector::StateVector;
use crate::services::average_g::PipaCalibration;
use crate::types::{Mat3x3, Met};
use crate::AgcState;

/// Sentinel bytes at offset 0 of [`BackupState`]. Absence of these bytes in
/// BKPSRAM is the signal that the region is uninitialised (first boot, or
/// battery removed), and the boot path should fall back to FRESH START.
pub const MAGIC: [u8; 4] = *b"AGC1";

/// Layout version of [`BackupState`]. Bump on any layout change so a firmware
/// upgrade can detect and reject pre-upgrade backups.
pub const VERSION: u32 = 1;

/// Header at the start of [`BackupState`]. 16 bytes, fixed layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct BackupHeader {
    pub magic: [u8; 4],
    pub version: u32,
    pub crc32: u32,
    pub _pad: u32,
}

impl BackupHeader {
    /// Header for an uninitialised backup region (all zeros). Distinguishable
    /// from a valid backup because `magic != MAGIC`.
    pub const ZERO: Self = Self {
        magic: [0; 4],
        version: 0,
        crc32: 0,
        _pad: 0,
    };
}

/// Survives-RESTART subset of [`AgcState`]. See module docs for the field
/// list and rationale.
#[derive(Clone, Copy)]
#[repr(C)]
pub struct BackupState {
    pub header: BackupHeader,
    pub csm_state: StateVector,
    pub target_state: StateVector,
    pub refsmmat: Mat3x3,
    pub time: Met,
    pub gha_epoch_rad: f64,
    pub restart: RestartProtection,
    pub pipa_cal: PipaCalibration,
    pub gyro_comp: GyroCompensation,
    pub last_drift_comp_time: Met,
    pub imu_alignment_state: ImuAlignmentState,
    pub tpi_arrival_epoch: Option<f64>,
    pub major_mode: u8,
    pub flagwords: [u16; 12],
}

impl BackupState {
    /// Construct a `BackupState` whose header is all zeros. The result is
    /// NOT a valid backup — [`restore_from_backup`] will return
    /// `Err(RestoreError::MagicMismatch)`. Use this as the "uninitialised
    /// region" placeholder on the host; on the bare-metal target the
    /// region is whatever bytes BKPSRAM holds.
    pub const fn zero() -> Self {
        Self {
            header: BackupHeader::ZERO,
            csm_state: StateVector::ZERO,
            target_state: StateVector::ZERO,
            refsmmat: crate::math::linalg::IDENTITY,
            time: Met(0),
            gha_epoch_rad: 0.0,
            restart: RestartProtection::new(),
            pipa_cal: PipaCalibration::NOMINAL,
            gyro_comp: GyroCompensation {
                nbdx: 0.0,
                nbdy: 0.0,
                nbdz: 0.0,
            },
            last_drift_comp_time: Met(0),
            imu_alignment_state: ImuAlignmentState::Caged,
            tpi_arrival_epoch: None,
            major_mode: 0,
            flagwords: [0; 12],
        }
    }
}

/// Reasons [`restore_from_backup`] can refuse to apply a backup.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RestoreError {
    /// Magic bytes don't match — region is uninitialised or battery was
    /// removed. Caller should fall back to FRESH START.
    MagicMismatch,
    /// Version field doesn't match the current firmware's `VERSION`. Caller
    /// should fall back to FRESH START (a firmware upgrade with an
    /// incompatible layout has occurred).
    VersionMismatch { found: u32, expected: u32 },
    /// CRC over the payload doesn't match. Indicates bit-flip corruption in
    /// BKPSRAM. Caller should fall back to FRESH START.
    ChecksumMismatch { found: u32, expected: u32 },
}

/// Snapshot the survives-RESTART fields of `state` into `backup`. Fills in
/// the magic, version, and CRC fields so a subsequent [`restore_from_backup`]
/// will accept it.
pub fn snapshot_for_restart(state: &AgcState, backup: &mut BackupState) {
    backup.csm_state = state.csm_state;
    backup.target_state = state.target_state;
    backup.refsmmat = state.refsmmat;
    backup.time = state.time;
    backup.gha_epoch_rad = state.gha_epoch_rad;
    backup.restart = state.restart;
    backup.pipa_cal = state.pipa_cal;
    backup.gyro_comp = state.gyro_comp;
    backup.last_drift_comp_time = state.last_drift_comp_time;
    backup.imu_alignment_state = state.imu_alignment_state;
    backup.tpi_arrival_epoch = state.tpi_arrival_epoch;
    backup.major_mode = state.major_mode;
    backup.flagwords = state.flagwords;

    backup.header.magic = MAGIC;
    backup.header.version = VERSION;
    backup.header._pad = 0;
    backup.header.crc32 = compute_crc32(backup);
}

/// Apply `backup` to `state`, restoring the survives-RESTART fields.
///
/// Validates the magic word, version, and checksum before touching `state`.
/// On any failure the function returns `Err` and `state` is left unchanged
/// — the caller is expected to fall back to FRESH START.
pub fn restore_from_backup(
    state: &mut AgcState,
    backup: &BackupState,
) -> Result<(), RestoreError> {
    if backup.header.magic != MAGIC {
        return Err(RestoreError::MagicMismatch);
    }
    if backup.header.version != VERSION {
        return Err(RestoreError::VersionMismatch {
            found: backup.header.version,
            expected: VERSION,
        });
    }
    let computed = compute_crc32(backup);
    if backup.header.crc32 != computed {
        return Err(RestoreError::ChecksumMismatch {
            found: backup.header.crc32,
            expected: computed,
        });
    }

    state.csm_state = backup.csm_state;
    state.target_state = backup.target_state;
    state.refsmmat = backup.refsmmat;
    state.time = backup.time;
    state.gha_epoch_rad = backup.gha_epoch_rad;
    state.restart = backup.restart;
    state.pipa_cal = backup.pipa_cal;
    state.gyro_comp = backup.gyro_comp;
    state.last_drift_comp_time = backup.last_drift_comp_time;
    state.imu_alignment_state = backup.imu_alignment_state;
    state.tpi_arrival_epoch = backup.tpi_arrival_epoch;
    state.major_mode = backup.major_mode;
    state.flagwords = backup.flagwords;
    Ok(())
}

/// Mark `backup` as uninitialised. Called from FRESH START so a subsequent
/// boot can't mistake stale BKPSRAM contents for a valid backup. Sets the
/// magic to zero — the rest of the fields are left as-is because they will
/// be overwritten by the next snapshot anyway.
pub fn invalidate(backup: &mut BackupState) {
    backup.header.magic = [0; 4];
}

// ── CRC-32/IEEE (no_std, no table) ────────────────────────────────────────────

/// Compute the CRC-32/IEEE checksum over the bytes of `backup`, skipping the
/// 4-byte CRC field at offset 8..12. This lets the function be used both for
/// computing the CRC to store and for verifying the CRC that was stored.
fn compute_crc32(backup: &BackupState) -> u32 {
    // SAFETY: `BackupState: Copy` and `#[repr(C)]`, so reading its bytes via
    // a `&[u8]` of the right length is sound. The slice does not outlive the
    // borrow of `backup`.
    let bytes = unsafe {
        core::slice::from_raw_parts(
            (backup as *const BackupState).cast::<u8>(),
            core::mem::size_of::<BackupState>(),
        )
    };
    // Header layout: magic[0..4], version[4..8], crc32[8..12], _pad[12..16].
    // Skip bytes 8..12 (the CRC field itself).
    let mut crc: u32 = 0xFFFF_FFFF;
    for (i, &byte) in bytes.iter().enumerate() {
        if (8..12).contains(&i) {
            continue;
        }
        crc = crc32_update(crc, byte);
    }
    !crc
}

fn crc32_update(crc: u32, byte: u8) -> u32 {
    let mut crc = crc ^ (byte as u32);
    let mut i = 0;
    while i < 8 {
        crc = if (crc & 1) == 1 {
            (crc >> 1) ^ 0xEDB8_8320
        } else {
            crc >> 1
        };
        i += 1;
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::Frame;

    /// TC-BACKUP-1: header layout is fixed at 16 bytes.
    #[test]
    fn tc_backup_1_header_is_16_bytes() {
        assert_eq!(core::mem::size_of::<BackupHeader>(), 16);
    }

    /// TC-BACKUP-2: BackupState fits comfortably in the 4 KB BKPSRAM region.
    #[test]
    fn tc_backup_2_fits_in_bkpsram() {
        assert!(
            core::mem::size_of::<BackupState>() < 4096,
            "BackupState ({} B) must fit in 4 KB BKPSRAM",
            core::mem::size_of::<BackupState>()
        );
    }

    /// TC-BACKUP-3: zero-initialised backup fails restore (no magic).
    #[test]
    fn tc_backup_3_zero_is_invalid() {
        let mut state = AgcState::new();
        let backup = BackupState::zero();
        assert_eq!(
            restore_from_backup(&mut state, &backup),
            Err(RestoreError::MagicMismatch)
        );
    }

    /// TC-BACKUP-4: snapshot → restore round-trip preserves every backup field.
    #[test]
    fn tc_backup_4_round_trip_preserves_fields() {
        // Build an AgcState with non-default values in every backup field.
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [1.0e6, 2.0e6, 3.0e6],
            velocity: [100.0, 200.0, 300.0],
            epoch: Met(50_000),
            frame: Frame::EarthInertial,
        };
        state.target_state = StateVector {
            position: [4.0e6, 5.0e6, 6.0e6],
            velocity: [400.0, 500.0, 600.0],
            epoch: Met(50_000),
            frame: Frame::EarthInertial,
        };
        state.refsmmat = [
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        state.time = Met(123_456);
        state.gha_epoch_rad = 1.234_567_8;
        state.restart.set_phase(crate::executive::GROUP_3, crate::executive::Phase::new(2));
        state.pipa_cal = PipaCalibration {
            scale: 0.1,
            bias: [1, -2, 3],
            misalignment: [[1.0, 0.001, 0.0], [-0.001, 1.0, 0.0], [0.0, 0.0, 1.0]],
        };
        state.gyro_comp = GyroCompensation {
            nbdx: 1.0e-7,
            nbdy: -2.0e-7,
            nbdz: 3.0e-7,
        };
        state.last_drift_comp_time = Met(99_999);
        state.imu_alignment_state = ImuAlignmentState::FineAligned;
        state.tpi_arrival_epoch = Some(1_500.0);
        state.major_mode = 31;
        state.flagwords = [0xAAAA, 0x5555, 0xFFFF, 0, 1, 2, 3, 4, 5, 6, 7, 8];

        // Snapshot.
        let mut backup = BackupState::zero();
        snapshot_for_restart(&state, &mut backup);

        // Wipe the AgcState and restore from backup.
        let mut restored = AgcState::new();
        assert_eq!(restore_from_backup(&mut restored, &backup), Ok(()));

        // Every backup field must match.
        assert_eq!(restored.csm_state.position, state.csm_state.position);
        assert_eq!(restored.csm_state.velocity, state.csm_state.velocity);
        assert_eq!(restored.csm_state.epoch, state.csm_state.epoch);
        assert_eq!(restored.target_state.position, state.target_state.position);
        assert_eq!(restored.refsmmat, state.refsmmat);
        assert_eq!(restored.time, state.time);
        assert_eq!(restored.gha_epoch_rad, state.gha_epoch_rad);
        assert_eq!(
            restored.restart.phase(crate::executive::GROUP_3),
            crate::executive::Phase::new(2)
        );
        assert_eq!(restored.pipa_cal.scale, state.pipa_cal.scale);
        assert_eq!(restored.pipa_cal.bias, state.pipa_cal.bias);
        assert_eq!(restored.gyro_comp.nbdx, state.gyro_comp.nbdx);
        assert_eq!(restored.gyro_comp.nbdy, state.gyro_comp.nbdy);
        assert_eq!(restored.gyro_comp.nbdz, state.gyro_comp.nbdz);
        assert_eq!(restored.last_drift_comp_time, state.last_drift_comp_time);
        assert_eq!(restored.imu_alignment_state, state.imu_alignment_state);
        assert_eq!(restored.tpi_arrival_epoch, state.tpi_arrival_epoch);
        assert_eq!(restored.major_mode, state.major_mode);
        assert_eq!(restored.flagwords, state.flagwords);
    }

    /// TC-BACKUP-5: version mismatch is detected and the state is left untouched.
    #[test]
    fn tc_backup_5_version_mismatch_detected() {
        let state = AgcState::new();
        let mut backup = BackupState::zero();
        snapshot_for_restart(&state, &mut backup);
        // Tamper with the version field to simulate a firmware upgrade.
        backup.header.version = VERSION.wrapping_add(1);

        let mut target = AgcState::new();
        target.major_mode = 7; // sentinel — must NOT be overwritten
        let result = restore_from_backup(&mut target, &backup);
        assert!(matches!(result, Err(RestoreError::VersionMismatch { .. })));
        assert_eq!(
            target.major_mode, 7,
            "state must not be modified on version mismatch"
        );
    }

    /// TC-BACKUP-6: CRC mismatch (from a bit-flip in the payload) is detected.
    #[test]
    fn tc_backup_6_checksum_mismatch_detected() {
        let mut state = AgcState::new();
        state.major_mode = 22;
        let mut backup = BackupState::zero();
        snapshot_for_restart(&state, &mut backup);
        // Flip a bit in the payload (without touching the header). This must
        // fail the CRC check — the saved CRC was computed over the un-tampered
        // bytes.
        backup.major_mode ^= 0x01;

        let mut target = AgcState::new();
        let result = restore_from_backup(&mut target, &backup);
        assert!(matches!(result, Err(RestoreError::ChecksumMismatch { .. })));
    }

    /// TC-BACKUP-7: invalidate clears the magic so the next restore fails fast.
    #[test]
    fn tc_backup_7_invalidate_drops_magic() {
        let state = AgcState::new();
        let mut backup = BackupState::zero();
        snapshot_for_restart(&state, &mut backup);

        invalidate(&mut backup);
        assert_eq!(backup.header.magic, [0; 4]);

        let mut target = AgcState::new();
        assert_eq!(
            restore_from_backup(&mut target, &backup),
            Err(RestoreError::MagicMismatch)
        );
    }

    /// TC-BACKUP-8: snapshot is idempotent — calling it twice on the same
    /// state produces a backup that still verifies.
    #[test]
    fn tc_backup_8_double_snapshot_still_valid() {
        let state = AgcState::new();
        let mut backup = BackupState::zero();
        snapshot_for_restart(&state, &mut backup);
        snapshot_for_restart(&state, &mut backup);
        let mut target = AgcState::new();
        assert_eq!(restore_from_backup(&mut target, &backup), Ok(()));
    }
}
