//! RCS (Reaction Control System) jet I/O interface.
//!
//! AGC source: Comanche055/JET_SELECTION_LOGIC.agc (JETSLECT, T6START, pages 1039-1062)
//!             Comanche055/ERASABLE_ASSIGNMENTS.agc
//!             PYJETS = channel 5 (octal 05) — pitch and yaw jet commands
//!             ROLLJETS = channel 6 (octal 06) — roll jet commands

/// AC-roll jets channel 6 mask.
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc ACRJETS = 03760 octal.
pub const ACRJETS_MASK: u16 = 0o03760;

/// BD-roll jets channel 6 mask.
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc BDRJETS = 34017 octal.
pub const BDRJETS_MASK: u16 = 0o34017;

/// Pitch jets channel 5 mask.
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc PJETS = 01417 octal.
pub const PJETS_MASK: u16 = 0o01417;

/// Yaw jets channel 5 mask.
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc YJETS = 06360 octal.
pub const YJETS_MASK: u16 = 0o06360;

/// A pair of jet channel words for one T6 interval.
///
/// Sent simultaneously to the two RCS output channels.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc (T6START, page 1061).
/// Channel 5 (PYJETS, octal 05): pitch and yaw jets.
/// Channel 6 (ROLLJETS, octal 06): roll jets.
#[derive(Clone, Copy, Debug, Default)]
pub struct JetCommand {
    /// Channel 5 word: bits encode pitch (PWORD) and yaw (YWORD) jets.
    /// AGC source: PYJETS = channel 05.
    pub pitch_yaw: u16,
    /// Channel 6 word: bits encode roll jets (RWORD, AC and BD quads).
    /// AGC source: ROLLJETS = channel 06.
    pub roll: u16,
}

impl JetCommand {
    /// All jets off command.
    pub const OFF: Self = Self {
        pitch_yaw: 0,
        roll: 0,
    };
}

/// RCS jet I/O interface.
///
/// Issues 16-jet on/off commands via the two RCS output channels.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc.
/// Channels:   PYJETS (octal 05) — pitch+yaw jets.
///             ROLLJETS (octal 06) — roll jets.
pub trait RcsIo {
    /// Issue a jet command word pair.
    ///
    /// Writes `cmd.pitch_yaw` to channel PYJETS (05) and `cmd.roll` to
    /// channel ROLLJETS (06).  The command takes effect immediately and
    /// persists until overwritten by the next call (typically the next
    /// T6RUPT cycle, 14 ms minimum pulse width).
    ///
    /// AGC source: Comanche055/JET_SELECTION_LOGIC.agc T6START label.
    fn fire_jets(&mut self, cmd: JetCommand);

    /// Turn off all jets (write 0 to both channels).
    ///
    /// AGC source: Comanche055/FRESH_START_AND_RESTART.agc `WRITE CHAN5; WRITE CHAN6`.
    fn all_jets_off(&mut self);

    /// Read back the current jet command state (last written).
    fn current_command(&self) -> JetCommand;

    /// Write channel 5 directly (PYJETS).
    ///
    /// Used by fresh_start to zero the channel.
    fn write_channel5(&mut self, word: u16);

    /// Write channel 6 directly (ROLLJETS).
    ///
    /// Used by fresh_start to zero the channel.
    fn write_channel6(&mut self, word: u16);
}

/// Bare-metal RCS implementation skeleton.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc channel 5/6 output path.
pub struct RcsImpl {
    command: JetCommand,
}

impl RcsImpl {
    /// Construct with all jets off.
    pub const fn new() -> Self {
        Self {
            command: JetCommand::OFF,
        }
    }

    /// Release the underlying peripheral handle (C-FREE).
    pub fn free(self) -> JetCommand {
        self.command
    }
}

impl Default for RcsImpl {
    fn default() -> Self {
        Self::new()
    }
}

impl RcsIo for RcsImpl {
    fn fire_jets(&mut self, cmd: JetCommand) {
        self.command = cmd;
    }

    fn all_jets_off(&mut self) {
        self.command = JetCommand::OFF;
    }

    fn current_command(&self) -> JetCommand {
        self.command
    }

    fn write_channel5(&mut self, word: u16) {
        self.command.pitch_yaw = word;
    }

    fn write_channel6(&mut self, word: u16) {
        self.command.roll = word;
    }
}
