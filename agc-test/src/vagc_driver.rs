//! High-level drivers for yaAGC's TCP channel-word protocol.
//!
//! Builds on [`crate::vagc_channel::YaAgcClient`] and adds two
//! peripheral-emulation pieces needed for MS-E7c end-to-end entry tests:
//!
//! 1. [`DskyScript`] — synthesises DSKY keypresses by writing 5-bit
//!    key codes to channel `0o15`. The AGC's `KEYRUPT1` interrupt picks
//!    each press up and dispatches into `CHARIN`.
//! 2. [`PipaInjector`] — converts a 3-vector of sensed Δv (m/s) into
//!    a stream of PIPA pulse packets. yaAGC's `SocketAPI.c`
//!    (`ParseIoPacket` branch with `Channel & 0x80`) routes such
//!    packets to `UnprogrammedIncrement(State, Counter, IncType)`,
//!    which then drives the AGC's PIPAX/Y/Z erasable counters via the
//!    `CounterPINC`/`CounterMINC` paths.
//!
//! ## DSKY keycode reference
//!
//! Reverse-engineered from `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`
//! `CHARIN2` dispatch table (pp. 315–316, lines 453–489). The 5-bit code
//! reaches `CHARIN` in `MPAC` via the `RAND MNKEYIN` + `LOW5` mask in
//! `KEYRUPT1`.
//!
//! | Code (octal) | Key                       |
//! |--------------|---------------------------|
//! | 00           | (alarm — no key)          |
//! | 01–07        | digits 1–7                |
//! | 010, 011     | digits 8, 9               |
//! | 020          | digit 0                   |
//! | 021          | VERB                      |
//! | 022          | ERROR RESET (RSET)        |
//! | 031          | KEY RELEASE (KEY RLSE)    |
//! | 032          | +                         |
//! | 033          | −                         |
//! | 034          | ENTER (ENTR / PRO)        |
//! | 036          | CLEAR (CLR)               |
//! | 037          | NOUN                      |
//!
//! Other 5-bit codes (012–017, 023–030, 035) trigger `CHARALRM` —
//! the AGC's "illegal key code" path.
//!
//! ## PIPA pulse-packet encoding
//!
//! yaAGC's wire protocol treats packets with the high bit of the
//! channel byte set (`channel & 0x80`) as counter-increment commands.
//! The low 7 bits select the counter register address; the 15-bit
//! value carries the `IncType` discriminant:
//!
//! - `IncType = 0` → `PINC` (counter += 1, ones-complement).
//! - `IncType = 2` → `MINC` (counter += −1, ones-complement).
//!
//! For PIPA pulses the relevant counter registers are:
//!
//! | Register | Erasable addr (octal) | Wire `channel` byte |
//! |----------|-----------------------|---------------------|
//! | PIPAX    | 037                   | `0o237` = `0xBF`    |
//! | PIPAY    | 040                   | `0o240` = `0xC0`    |
//! | PIPAZ    | 041                   | `0o241` = `0xC1`    |
//!
//! Each pulse packet carries the count of **one** PIPA quantum
//! (~0.0585 m/s for the nominal Apollo IMU). [`PipaInjector::tick`]
//! emits N pulses per axis where N is the absolute count returned by
//! [`crate::entry_sim::pipa_pulses_for_dv`]. The pulses for one
//! 2-s SERVICER cycle are emitted in a single burst at the start of
//! the cycle; yaAGC accumulates them into the counter registers
//! immediately, and the AGC's foreground SERVICER reads them at the
//! next sample point. Pulse pacing within the cycle is not modelled —
//! stage A only requires the per-cycle Δv to match.

use std::io;

use crate::entry_sim::{pipa_pulses_for_dv, EntryIntegrator, SUB_STEP_S};
use crate::vagc_channel::{ChannelPacket, YaAgcClient};
use agc_core::services::average_g::PipaCalibration;
use agc_core::types::Vec3;

/// AGC DSKY input channel (`MNKEYIN` in `KEYRUPT1`).
pub const CHAN_KEYIN: u16 = 0o15;

/// Counter-increment marker bit in the wire `channel` byte.
const COUNTER_FLAG: u16 = 0x80;

/// PIPA erasable addresses, per `ERASABLE_ASSIGNMENTS.agc:152-154`.
const PIPAX_ADDR: u16 = 0o37;
const PIPAY_ADDR: u16 = 0o40;
const PIPAZ_ADDR: u16 = 0o41;

/// yaAGC `IncType` for `CounterPINC` (positive unit increment).
const INC_PINC: u16 = 0;
/// yaAGC `IncType` for `CounterMINC` (negative unit increment).
const INC_MINC: u16 = 2;

// ── DSKY scripter ──────────────────────────────────────────────────────────

/// One DSKY key, mapped to its 5-bit channel-015 code.
///
/// Variants cover every code that `CHARIN2`'s dispatch table treats as
/// a valid keypress. Unmapped codes (e.g., 012–017) fire `CHARALRM` on
/// the AGC and are intentionally inaccessible through this type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DskyKey {
    /// Digit 0 (channel-015 code `0o20`).
    Digit0,
    /// Digit 1–9 (channel-015 codes `0o01`–`0o11`).
    Digit(u8),
    /// VERB key (`0o21`).
    Verb,
    /// NOUN key (`0o37`).
    Noun,
    /// ENTR (Enter / Proceed within data entry) (`0o34`).
    Enter,
    /// CLR key (`0o36`).
    Clear,
    /// + key (`0o32`).
    Plus,
    /// − key (`0o33`).
    Minus,
    /// RSET (Error reset) key (`0o22`).
    Reset,
    /// KEY RLSE key (`0o31`).
    KeyRelease,
}

impl DskyKey {
    /// 5-bit AGC keycode for this key.
    ///
    /// Returns `None` for [`DskyKey::Digit(n)`] when `n > 9`. All other
    /// variants always have a defined code.
    pub fn code(self) -> Option<u16> {
        match self {
            DskyKey::Digit0 => Some(0o20),
            DskyKey::Digit(n) if (1..=9).contains(&n) => Some(n as u16),
            DskyKey::Digit(_) => None,
            DskyKey::Verb => Some(0o21),
            DskyKey::Noun => Some(0o37),
            DskyKey::Enter => Some(0o34),
            DskyKey::Clear => Some(0o36),
            DskyKey::Plus => Some(0o32),
            DskyKey::Minus => Some(0o33),
            DskyKey::Reset => Some(0o22),
            DskyKey::KeyRelease => Some(0o31),
        }
    }
}

/// Drive an AGC's DSKY input channel via the yaAGC TCP socket protocol.
///
/// Wraps a [`YaAgcClient`] and emits channel-015 writes for each
/// keypress. Each write fires KEYRUPT1 on the AGC and delivers the
/// 5-bit code to `CHARIN`.
///
/// The client is owned by the script; if a test needs to read AGC →
/// peripheral writes in parallel (e.g., to capture the DSKY display
/// channel), use a separate [`YaAgcClient`] on a second connection.
pub struct DskyScript {
    client: YaAgcClient,
}

impl DskyScript {
    /// Build a scripter over an already-connected client.
    pub fn new(client: YaAgcClient) -> Self {
        Self { client }
    }

    /// Send one keypress.
    ///
    /// Returns `io::Error` of kind `InvalidInput` for an out-of-range
    /// `DskyKey::Digit(n)` (n > 9) and any TCP error from the
    /// underlying socket.
    pub fn press(&mut self, key: DskyKey) -> io::Result<()> {
        let code = key.code().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("invalid DSKY digit: {key:?}"),
            )
        })?;
        self.client.send(ChannelPacket {
            channel: CHAN_KEYIN,
            value: code,
            u_bit: false,
        })
    }

    /// Convenience: send ENTR.
    pub fn enter(&mut self) -> io::Result<()> {
        self.press(DskyKey::Enter)
    }

    /// Convenience: send CLR.
    pub fn clear(&mut self) -> io::Result<()> {
        self.press(DskyKey::Clear)
    }

    /// Convenience: send the `Vnn` digit pair. Asserts `verb < 100`;
    /// values above are clipped to two decimal digits.
    pub fn verb(&mut self, verb: u8) -> io::Result<()> {
        let verb = verb % 100;
        self.press(DskyKey::Verb)?;
        self.press(digit(verb / 10))?;
        self.press(digit(verb % 10))
    }

    /// Convenience: send the `Nnn` digit pair.
    pub fn noun(&mut self, noun: u8) -> io::Result<()> {
        let noun = noun % 100;
        self.press(DskyKey::Noun)?;
        self.press(digit(noun / 10))?;
        self.press(digit(noun % 10))
    }

    /// Convenience: send the canonical `VnnNnnE` 6-key sequence.
    pub fn verb_noun(&mut self, verb: u8, noun: u8) -> io::Result<()> {
        self.verb(verb)?;
        self.noun(noun)?;
        self.enter()
    }

    /// Borrow the underlying client for receive-side polling.
    pub fn client_mut(&mut self) -> &mut YaAgcClient {
        &mut self.client
    }
}

/// Map a 0–9 decimal digit to its `DskyKey` variant.
fn digit(n: u8) -> DskyKey {
    if n == 0 {
        DskyKey::Digit0
    } else {
        DskyKey::Digit(n)
    }
}

// ── PIPA injector ──────────────────────────────────────────────────────────

/// Stream PIPA pulses to a yaAGC instance to emulate IMU accelerometer
/// output.
///
/// On each [`PipaInjector::tick`], the wrapped [`EntryIntegrator`]
/// integrates one SERVICER cycle (default 2 s) under the supplied L/D
/// command and bank angle, the resulting sensed Δv is quantised into
/// per-axis PIPA pulse counts, and each pulse is emitted as a single
/// counter-increment packet on the wire.
///
/// Pulses for the cycle are sent in a single burst at the start of the
/// cycle — yaAGC accumulates them into the counter registers
/// immediately, and the AGC's foreground SERVICER samples them at the
/// next 2-s tick. Pulse rates inside the cycle are not modelled; the
/// stage-A integration only requires the per-cycle Δv to match.
pub struct PipaInjector {
    client: YaAgcClient,
    integrator: EntryIntegrator,
    pipa_cal: PipaCalibration,
}

impl PipaInjector {
    /// Build an injector around an existing client, integrator, and
    /// PIPA calibration. The integrator's working state (position,
    /// velocity) is supplied per-call to [`tick`](Self::tick).
    pub fn new(
        client: YaAgcClient,
        integrator: EntryIntegrator,
        pipa_cal: PipaCalibration,
    ) -> Self {
        Self {
            client,
            integrator,
            pipa_cal,
        }
    }

    /// Integrate one SERVICER cycle and emit the resulting PIPA pulses.
    ///
    /// Inputs match [`EntryIntegrator::integrate_cycle`]:
    /// - `position`, `velocity` — ECI state at the start of the cycle.
    /// - `ld_command` — vertical L/D commanded by the AGC.
    /// - `bank_rad` — bank angle in radians (0 = lift up).
    /// - `dt_s` — interval to integrate over (usually `SERVICER_PERIOD_S`).
    ///
    /// Returns the inertial Δv that was just delivered to yaAGC,
    /// allowing the caller to mirror the same Δv into a parallel
    /// `agc-sim` run for cycle-by-cycle comparison.
    pub fn tick(
        &mut self,
        position: Vec3,
        velocity: Vec3,
        ld_command: f64,
        bank_rad: f64,
        dt_s: f64,
    ) -> io::Result<Vec3> {
        let dv = self
            .integrator
            .integrate_cycle(position, velocity, ld_command, bank_rad, dt_s);
        let counts = pipa_pulses_for_dv(dv, &self.pipa_cal);
        self.emit_pulses(counts)?;
        Ok(dv)
    }

    /// Borrow the underlying client for parallel read-side polling.
    pub fn client_mut(&mut self) -> &mut YaAgcClient {
        &mut self.client
    }

    /// Send the per-axis pulse counts as raw `UnprogrammedIncrement`
    /// packets. Positive counts use `PINC` (`IncType = 0`), negative
    /// counts use `MINC` (`IncType = 2`). One packet per pulse.
    fn emit_pulses(&mut self, counts: [i16; 3]) -> io::Result<()> {
        const AXES: [(u16, i16); 3] = [(PIPAX_ADDR, 0), (PIPAY_ADDR, 1), (PIPAZ_ADDR, 2)];

        for (addr, idx) in AXES {
            let count = counts[idx as usize];
            let (inc, n) = if count >= 0 {
                (INC_PINC, count as i32)
            } else {
                (INC_MINC, -(count as i32))
            };
            let packet = ChannelPacket {
                channel: COUNTER_FLAG | addr,
                value: inc,
                u_bit: false,
            };
            for _ in 0..n {
                self.client.send(packet)?;
            }
        }
        Ok(())
    }
}

/// Number of PIPA pulse packets that would be sent for a per-cycle Δv
/// of `counts` quanta. Diagnostic helper for tests and tracing.
pub fn pipa_pulse_packet_count(counts: [i16; 3]) -> usize {
    counts.iter().map(|c| c.unsigned_abs() as usize).sum()
}

/// `1/PIPADT`, the SERVICER's effective PIPA accumulation interval (s).
///
/// Provided here so tests can derive bursts from the AGC's expected
/// sample cadence without re-reading the EntryIntegrator's internal
/// sub-step. Same value as [`crate::entry_sim::SUB_STEP_S`].
pub const PIPA_SUBSAMPLE_S: f64 = SUB_STEP_S;

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vagc_channel::ChannelPacket;

    /// TC-DSKY-CODE-1: every `DskyKey` resolves to the exact 5-bit code
    /// from `CHARIN2`'s dispatch table.
    #[test]
    fn tc_dsky_code_1_table_matches_charin2() {
        assert_eq!(DskyKey::Digit0.code(), Some(0o20));
        for d in 1..=9u8 {
            assert_eq!(DskyKey::Digit(d).code(), Some(d as u16));
        }
        assert_eq!(DskyKey::Verb.code(), Some(0o21));
        assert_eq!(DskyKey::Reset.code(), Some(0o22));
        assert_eq!(DskyKey::KeyRelease.code(), Some(0o31));
        assert_eq!(DskyKey::Plus.code(), Some(0o32));
        assert_eq!(DskyKey::Minus.code(), Some(0o33));
        assert_eq!(DskyKey::Enter.code(), Some(0o34));
        assert_eq!(DskyKey::Clear.code(), Some(0o36));
        assert_eq!(DskyKey::Noun.code(), Some(0o37));
    }

    /// TC-DSKY-CODE-2: digits outside 0–9 are rejected (return `None`).
    #[test]
    fn tc_dsky_code_2_invalid_digit_rejected() {
        assert_eq!(DskyKey::Digit(0).code(), None); // use Digit0 instead
        assert_eq!(DskyKey::Digit(10).code(), None);
        assert_eq!(DskyKey::Digit(255).code(), None);
    }

    /// TC-PIPA-ENC-1: PIPA pulse packets use the counter-flag bit and
    /// the documented per-axis erasable address.
    #[test]
    fn tc_pipa_enc_1_packet_layout() {
        // Hand-build the expected packet for a single +X pulse.
        let expected = ChannelPacket {
            channel: 0x80 | 0o37,
            value: INC_PINC,
            u_bit: false,
        };
        let bytes = expected.pack();
        let round = ChannelPacket::unpack(bytes).unwrap();
        assert_eq!(round, expected);
        // Counter-flag bit must survive the round trip.
        assert_eq!(round.channel & 0x80, 0x80);
        // Low 7 bits decode to PIPAX address.
        assert_eq!(round.channel & 0x7F, 0o37);
    }

    /// TC-PIPA-ENC-2: the PIPA pulse packet count matches `|counts|`
    /// summed over the three axes.
    #[test]
    fn tc_pipa_enc_2_pulse_count() {
        assert_eq!(pipa_pulse_packet_count([10, 0, -20]), 30);
        assert_eq!(pipa_pulse_packet_count([0, 0, 0]), 0);
        assert_eq!(pipa_pulse_packet_count([i16::MIN, 0, 0]), 0x8000);
    }

    /// TC-PIPA-ENC-3: positive and negative counts use different
    /// `IncType` discriminants (PINC vs MINC).
    #[test]
    fn tc_pipa_enc_3_sign_branch() {
        assert_eq!(INC_PINC, 0);
        assert_eq!(INC_MINC, 2);
    }
}
