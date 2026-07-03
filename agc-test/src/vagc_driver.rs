// SPDX-License-Identifier: GPL-3.0-or-later
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

/// AGC input discrete channel that carries the PROCEED / STBY / etc.
/// hand-controller discretes (see `Comanche055/FRESH_START_AND_RESTART.agc:1990`
/// and `:2061` for the `InputChannel[032] & 020000` PROCEED check).
pub const CHAN_INPUT_DISCRETES: u16 = 0o32;

/// Bit 14 (AGC 1-indexed = 2¹³ = `0o20000`) of channel `0o32` is the
/// PROCEED discrete. Active-low: bit clear = pressed, bit set =
/// released. The AGC's idle value for channel `0o32` is `0o77777`
/// (all discretes released).
pub const PRO_BIT: u16 = 0o20000;

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

/// CDU gimbal counter channel addresses (`EQUALS` values in
/// `Comanche055/ERASABLE_ASSIGNMENTS.agc:145-147`).
/// Wire format: `COUNTER_FLAG | CDU*_ADDR`.
const CDUX_ADDR: u16 = 0o32; // outer  gimbal → wire 0x9A
const CDUY_ADDR: u16 = 0o33; // inner  gimbal → wire 0x9B
const CDUZ_ADDR: u16 = 0o34; // middle gimbal → wire 0x9C

/// yaAGC `IncType` for `+PCDU` (positive CDU increment, slow-rate).
const INC_PCDU: u16 = 1;
/// yaAGC `IncType` for `−MCDU` (negative CDU increment, slow-rate).
const INC_MCDU: u16 = 3;

/// CDU angle resolution: 1 LSB = 360° / 32768 ≈ 0.010986°.
/// Inverse: ≈ 91.02 counts per degree.
///
/// Source: Comanche055 CDU counter register is 15 bits, full-scale 360°,
/// so 1 LSB = 360° / 2^15.
pub const CDU_LSB_DEG: f64 = 360.0 / 32768.0;

/// Maximum CDU injection rate (counts per second). The yaAGC FIFO for
/// unprogrammed increments drains at 400 counts/s (per §8.4). Stay under
/// this limit to prevent FIFO overflow and dropped pulses.
pub const CDU_MAX_RATE_CPS: f64 = 400.0;

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

/// Default delay after each [`DskyScript::press`]. The AGC's KEYRUPT1
/// handler reads channel-015 once per fired interrupt; without a gap
/// between successive keypress packets the second one would
/// overwrite the channel-015 register before the AGC could service
/// the first. 80 ms wall-clock ≈ 1.5 simulated seconds at yaAGC's
/// typical ~20× pace — well above KEYRUPT's handler runtime (a few
/// hundred microseconds) and the CHARIN job-dispatch overhead.
const DEFAULT_INTER_KEY_DELAY: std::time::Duration = std::time::Duration::from_millis(80);

/// Drive an AGC's DSKY input channel via the yaAGC TCP socket protocol.
///
/// Wraps a [`YaAgcClient`] and emits channel-015 writes for each
/// keypress. Each write fires KEYRUPT1 on the AGC and delivers the
/// 5-bit code to `CHARIN`.
///
/// A small post-keystroke delay (default [`DEFAULT_INTER_KEY_DELAY`])
/// is applied inside [`press`](Self::press) so back-to-back calls
/// like `verb_noun(37, 62)` give the AGC time to service KEYRUPT
/// between keystrokes. Use [`with_inter_key_delay`](Self::with_inter_key_delay)
/// to override (e.g., `Duration::ZERO` for a packet-rate test).
///
/// The client is owned by the script; if a test needs to read AGC →
/// peripheral writes in parallel (e.g., to capture the DSKY display
/// channel), use a separate [`YaAgcClient`] on a second connection.
pub struct DskyScript {
    client: YaAgcClient,
    inter_key_delay: std::time::Duration,
}

impl DskyScript {
    /// Build a scripter over an already-connected client. Uses
    /// [`DEFAULT_INTER_KEY_DELAY`] between successive keypresses.
    pub fn new(client: YaAgcClient) -> Self {
        Self {
            client,
            inter_key_delay: DEFAULT_INTER_KEY_DELAY,
        }
    }

    /// Override the post-keypress delay. Pass `Duration::ZERO` for
    /// the protocol-only behaviour useful in unit tests that just
    /// want to count packets.
    pub fn with_inter_key_delay(mut self, delay: std::time::Duration) -> Self {
        self.inter_key_delay = delay;
        self
    }

    /// Send one keypress and sleep for [`Self::inter_key_delay`]
    /// afterwards.
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
        })?;
        if !self.inter_key_delay.is_zero() {
            std::thread::sleep(self.inter_key_delay);
        }
        Ok(())
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

    /// Convenience: send the canonical `VnnNnnE` 7-key sequence
    /// (`VERB n n NOUN n n ENTR`). Suitable for verb-noun pairs like
    /// V16 N36 (monitor time); **not** suitable for V37 (major-mode
    /// request) — use [`verb_major_mode`](Self::verb_major_mode) for
    /// that.
    pub fn verb_noun(&mut self, verb: u8, noun: u8) -> io::Result<()> {
        self.verb(verb)?;
        self.noun(noun)?;
        self.enter()
    }

    /// Send `V37 ENTR <mm digits> ENTR` — the AGC's major-mode
    /// request sequence. V37 uses a verb-then-digits pattern, not
    /// verb-noun: the digits entered after the first ENTR populate
    /// the `MMNUMBER` register (not `NOUNREG`) and the second ENTR
    /// dispatches into the program. Sending `V37 N62 ENTR` (via
    /// [`verb_noun`](Self::verb_noun)) leaves the AGC waiting for
    /// `NOUNREG` data that never arrives, and MMNUMBER stays at 0.
    pub fn verb_major_mode(&mut self, major_mode: u8) -> io::Result<()> {
        let mm = major_mode % 100;
        self.verb(37)?;
        self.enter()?;
        self.press(digit(mm / 10))?;
        self.press(digit(mm % 10))?;
        self.enter()
    }

    /// Send `V33 ENTR` — the AGC's PROCEED-WITHOUT-DATA verb
    /// (`Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc:184`,
    /// `PINBALL_GAME__BUTTONS_AND_LIGHTS.agc:2902` `VBPROC`).
    ///
    /// Used to acknowledge flashing displays (`V06 N61`, etc.) and
    /// advance the AGC through programs that pause for crew input
    /// — e.g., P62 → ROLLC init → P63.
    ///
    /// V33 ENTR is preferred over the hardware PRO discrete on
    /// channel `0o32` bit 14 (`PRO_BIT`):
    /// - The hardware bit is sampled by T4RUPT every ~120 ms
    ///   simulated, so a press has to be held across at least one
    ///   sample edge to be detected — sensitive to yaAGC's pace.
    /// - The same bit doubles as the STANDBY discrete; holding it
    ///   for over 1.28 s simulated puts the AGC into standby
    ///   (`agc_engine.c:2058`).
    /// - V33 ENTR is keyboard-driven (KEYRUPT → CHARIN → VBPROC)
    ///   and has none of those timing pitfalls.
    pub fn proceed(&mut self) -> io::Result<()> {
        self.verb(33)?;
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

// ── CDU injector ───────────────────────────────────────────────────────────

/// Stream CDU gimbal-angle pulses to a yaAGC instance to simulate the
/// IMU's stable-platform attitude drive.
///
/// On each [`CduInjector::tick`], the injector advances each CDU axis
/// by a rate-limited step toward the programmed target angle, emitting
/// individual `+PCDU` (`IncType = 1`) or `−MCDU` (`IncType = 3`) packets.
/// The step size is bounded by both the requested slew rate and the
/// yaAGC FIFO drain limit ([`CDU_MAX_RATE_CPS`] = 400 counts/s).
///
/// ## Wire encoding (§8.4 of `docs/reentry_workflow_spec.md`)
///
/// | Axis          | Counter (octal) | Wire channel (`0x80 | addr`) |
/// |---------------|-----------------|------------------------------|
/// | CDUX (outer)  | `0o32`          | `0x9A`                       |
/// | CDUY (inner)  | `0o33`          | `0x9B`                       |
/// | CDUZ (middle) | `0o34`          | `0x9C`                       |
///
/// ## Rate-continuous requirement (§8.3)
///
/// The entry DAP (EXDAP) derives the body rate `PREL/QREL/RREL` from
/// successive CDU counter differences. Injecting a large step in one
/// tick causes a spurious one-cycle rate spike. Always ramp CDU angles
/// at a physically plausible slew (≤ a few degrees/s) by calling `tick`
/// with an appropriate `slew_rate_deg_per_s`.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc lines 145-147 (counter
/// channel addresses); CM_ENTRY_DIGITAL_AUTOPILOT.agc (EXDAP loop that
/// reads CDU via READGYMB and computes CALFA/CMDAPMOD).
pub struct CduInjector {
    client: YaAgcClient,
    /// Cumulative injected CDU angle `[CDUX, CDUY, CDUZ]` in degrees.
    /// Tracks the total increment we have sent, not the absolute hardware
    /// CDU counter value.
    current_deg: [f64; 3],
    /// Target CDU angle `[CDUX, CDUY, CDUZ]` in degrees.
    target_deg: [f64; 3],
}

impl CduInjector {
    /// Build a CDU injector over an existing client.
    ///
    /// `initial_deg` is the presumed starting CDU angle `[CDUX, CDUY,
    /// CDUZ]` in degrees. Pass `[0.0; 3]` when the core image was
    /// pre-patched to clear the CDU counter registers, or the measured
    /// attitude derived from a prior core dump.
    pub fn new(client: YaAgcClient, initial_deg: [f64; 3]) -> Self {
        Self {
            client,
            current_deg: initial_deg,
            target_deg: initial_deg,
        }
    }

    /// Set the target CDU angle `[CDUX, CDUY, CDUZ]` in degrees.
    pub fn set_target(&mut self, target_deg: [f64; 3]) {
        self.target_deg = target_deg;
    }

    /// Return the current cumulative injected CDU angle in degrees.
    pub fn current_deg(&self) -> [f64; 3] {
        self.current_deg
    }

    /// Advance each CDU axis toward its target by at most
    /// `slew_rate_deg_per_s × tick_s` per axis, emitting PCDU/MCDU
    /// packets. The per-tick burst is also capped at
    /// `CDU_MAX_RATE_CPS × tick_s` counts to respect the FIFO limit.
    ///
    /// Returns the updated cumulative CDU angle after this tick.
    ///
    /// Axes already within one LSB of their target are left undisturbed.
    pub fn tick(&mut self, tick_s: f64, slew_rate_deg_per_s: f64) -> io::Result<[f64; 3]> {
        let max_step_deg = slew_rate_deg_per_s * tick_s;
        let fifo_cap_deg = CDU_MAX_RATE_CPS * tick_s * CDU_LSB_DEG;
        let limit_deg = max_step_deg.min(fifo_cap_deg);

        for axis in 0..3usize {
            let delta = self.target_deg[axis] - self.current_deg[axis];
            // Skip if within half an LSB.
            if delta.abs() < CDU_LSB_DEG * 0.5 {
                continue;
            }
            let step = delta.clamp(-limit_deg, limit_deg);
            let counts = (step / CDU_LSB_DEG).round() as i32;
            if counts == 0 {
                continue;
            }
            self.emit_cdu_axis(axis, counts)?;
            self.current_deg[axis] += counts as f64 * CDU_LSB_DEG;
        }
        Ok(self.current_deg)
    }

    /// Borrow the underlying client for receive-side polling.
    pub fn client_mut(&mut self) -> &mut YaAgcClient {
        &mut self.client
    }

    /// Emit `|counts|` PCDU or MCDU pulses on the given axis index
    /// (0 = CDUX outer, 1 = CDUY inner, 2 = CDUZ middle).
    fn emit_cdu_axis(&mut self, axis: usize, counts: i32) -> io::Result<()> {
        const ADDRS: [u16; 3] = [CDUX_ADDR, CDUY_ADDR, CDUZ_ADDR];
        let (inc, n) = if counts > 0 {
            (INC_PCDU, counts)
        } else {
            (INC_MCDU, -counts)
        };
        let pkt = ChannelPacket {
            channel: COUNTER_FLAG | ADDRS[axis],
            value: inc,
            u_bit: false,
        };
        for _ in 0..n {
            self.client.send(pkt)?;
        }
        Ok(())
    }
}

/// Compute the target CDU gimbal angles (degrees) for the entry trim
/// attitude — heat-shield X-body axis aligned to the velocity direction
/// (AoA ≈ 0°, `CALFA = 1.0 > cos 45°`) — from the ECI state vector and
/// the stable-member REFSMMAT.
///
/// ## Gimbal recipe (§8.2 of `docs/reentry_workflow_spec.md`)
///
/// Inverse of READGYMB (`Comanche055/CM_BODY_ATTITUDE.agc`):
///
/// ```text
/// X_body_sm = REFSMMAT · (−unit(velocity))   # nose = −velocity (heat-shield into wind)
/// Y_body_sm = Gram-Schmidt of Y_SM against X_body_sm
/// Z_body_sm = X_body_sm × Y_body_sm
///
/// CDUZ (middle, AMG) = arcsin( X_body_sm[1] )
/// CDUY (inner, AIG)  = atan2( −X_body_sm[2], X_body_sm[0] )
/// CDUX (outer, AOG)  = atan2( −Z_body_sm[1], Y_body_sm[1] )
/// ```
///
/// The AGC XB axis is the CM **nose** direction, not the heat-shield.
/// For entry (heat-shield forward into the re-entry plasma), the CM flies
/// backward: nose points **away** from the velocity vector, so
/// `X_body = −unit(velocity)` in ECI, mapped through REFSMMAT into SM.
///
/// With this convention the returned CDU angles satisfy CALFA ≈ +1.0
/// once the EXDAP has integrated them, satisfying the WAKEP62 gate
/// condition |CALFA| ≥ cos(45°) AND CALFA > 0 (§3.3).
///
/// Returns `[0.0; 3]` for a degenerate (near-zero velocity) input.
///
/// AGC source: Comanche055/CM_BODY_ATTITUDE.agc, READGYMB routine.
pub fn entry_trim_cdu_deg(
    _position: [f64; 3],
    velocity: [f64; 3],
    refsmmat: [[f64; 3]; 3],
) -> [f64; 3] {
    let v_hat = match unit3(velocity) {
        Some(u) => u,
        None => return [0.0; 3],
    };

    // X_body in SM coordinates: REFSMMAT · (−v_hat).
    //
    // The AGC XB axis is the CM nose direction.  For heat-shield-forward
    // entry the CM is flying backward (nose away from the velocity vector),
    // so the SM representation of the nose is REFSMMAT · (−unit(velocity)).
    // Using +unit(velocity) gives nose-into-wind → CALFA ≈ −1, which keeps
    // EXDAP in the nose-in branch (CMDAPMOD = −0) and WAKEP62 is never
    // scheduled (issue root-caused in tc_e7i_j live run, 2026-07).
    let neg_v_hat = [-v_hat[0], -v_hat[1], -v_hat[2]];
    let xb = matvec3(refsmmat, neg_v_hat);

    // Y_body: Gram-Schmidt of SM Y-axis ([0, 1, 0]) against X_body.
    let dot_xy = xb[1]; // dot([0,1,0], xb) = xb[1]
    let raw_yb = [
        0.0 - dot_xy * xb[0],
        1.0 - dot_xy * xb[1],
        0.0 - dot_xy * xb[2],
    ];
    let yb = match unit3(raw_yb) {
        Some(u) => u,
        // X_body parallel to Y_SM: fall back to SM Z-axis ([0,0,1]).
        None => {
            let dot_xz = xb[2];
            let raw_zb = [
                0.0 - dot_xz * xb[0],
                0.0 - dot_xz * xb[1],
                1.0 - dot_xz * xb[2],
            ];
            unit3(raw_zb).unwrap_or([0.0, 1.0, 0.0])
        }
    };
    let zb = cross3(xb, yb);

    // Gimbal angles — clamp arcsin argument to guard floating-point drift.
    let cduz_rad = xb[1].clamp(-1.0, 1.0).asin();
    let cduy_rad = (-xb[2]).atan2(xb[0]);
    let cdux_rad = (-zb[1]).atan2(yb[1]);

    [cdux_rad.to_degrees(), cduy_rad.to_degrees(), cduz_rad.to_degrees()]
}

/// Normalise a 3-vector; returns `None` if the magnitude is negligible.
fn unit3(v: [f64; 3]) -> Option<[f64; 3]> {
    let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if mag < 1.0e-9 {
        return None;
    }
    Some([v[0] / mag, v[1] / mag, v[2] / mag])
}

/// 3-vector cross product `a × b`.
fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Matrix-vector product `M · v` (row-major 3×3 × column 3-vector).
fn matvec3(m: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

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

    /// TC-PRO-ENC-1: PRO bit position matches the AGC source's
    /// `InputChannel[032] & 020000` PROCEED check
    /// (`Comanche055/FRESH_START_AND_RESTART.agc:1990`). With all
    /// other channel-32 bits at their idle "released" value of 1, the
    /// pressed-PRO packet has value `0o57777` (only bit 14 cleared)
    /// and the released packet has value `0o77777`.
    #[test]
    fn tc_pro_enc_1_pro_bit_position() {
        assert_eq!(PRO_BIT, 0o20000);
        assert_eq!(0o77777 & !PRO_BIT, 0o57777);
        assert_eq!(CHAN_INPUT_DISCRETES, 0o32);
        // The pressed-PRO packet on the wire round-trips through
        // ChannelPacket::pack/unpack.
        let p = ChannelPacket {
            channel: CHAN_INPUT_DISCRETES,
            value: 0o77777 & !PRO_BIT,
            u_bit: false,
        };
        let round = ChannelPacket::unpack(p.pack()).unwrap();
        assert_eq!(round, p);
        assert_eq!(
            round.value & PRO_BIT,
            0,
            "PRO bit must be CLEAR in pressed packet"
        );
    }

    // ── CDU encoder tests ──────────────────────────────────────────────────

    /// TC-CDU-LSB-1: `CDU_LSB_DEG` matches the 360°/32768 specification
    /// from §8.4 of `docs/reentry_workflow_spec.md`.
    #[test]
    fn tc_cdu_lsb_1_scale() {
        let expected = 360.0_f64 / 32768.0;
        assert!(
            (CDU_LSB_DEG - expected).abs() < 1e-12,
            "CDU_LSB_DEG = {CDU_LSB_DEG}, expected {expected}"
        );
        // Implied counts-per-degree: ≈ 91.02.
        let cpd = 1.0 / CDU_LSB_DEG;
        assert!(
            (cpd - 91.022).abs() < 0.001,
            "counts/deg = {cpd}"
        );
    }

    /// TC-CDU-ENC-1: CDU packet encoding uses the counter-flag bit and
    /// the correct per-axis erasable address from ERASABLE_ASSIGNMENTS.
    #[test]
    fn tc_cdu_enc_1_packet_layout() {
        // PCDU on CDUY (inner, 0o33 → wire 0x9B).
        let pkt = ChannelPacket {
            channel: COUNTER_FLAG | CDUY_ADDR,
            value: INC_PCDU,
            u_bit: false,
        };
        let bytes = pkt.pack();
        let round = ChannelPacket::unpack(bytes).unwrap();
        assert_eq!(round, pkt);
        assert_eq!(round.channel & 0x80, 0x80, "counter-flag bit set");
        assert_eq!(round.channel & 0x7F, 0o33, "CDUY address low 7 bits");
        assert_eq!(round.value, INC_PCDU, "PCDU IncType");

        // MCDU on CDUZ (middle, 0o34 → wire 0x9C).
        let pkt2 = ChannelPacket {
            channel: COUNTER_FLAG | CDUZ_ADDR,
            value: INC_MCDU,
            u_bit: false,
        };
        let round2 = ChannelPacket::unpack(pkt2.pack()).unwrap();
        assert_eq!(round2.channel & 0x7F, 0o34, "CDUZ address low 7 bits");
        assert_eq!(round2.value, INC_MCDU, "MCDU IncType");
    }

    /// TC-CDU-ENC-2: CDU IncType constants are distinct and different
    /// from PIPA IncType constants (§8.4: PCDU=1, MCDU=3 vs PINC=0, MINC=2).
    #[test]
    fn tc_cdu_enc_2_inc_type_distinct() {
        assert_eq!(INC_PCDU, 1, "PCDU = slow positive CDU increment");
        assert_eq!(INC_MCDU, 3, "MCDU = slow negative CDU increment");
        // Must not overlap with PIPA IncTypes (0 and 2).
        assert_ne!(INC_PCDU, INC_PINC);
        assert_ne!(INC_PCDU, INC_MINC);
        assert_ne!(INC_MCDU, INC_PINC);
        assert_ne!(INC_MCDU, INC_MINC);
    }

    /// TC-CDU-TRIM-1: `entry_trim_cdu_deg` for a purely circular orbit
    /// (R along ECI-X, V along ECI-Y; FPA = 0°) and the matching entry-
    /// aligned REFSMMAT returns the heat-shield-forward CDU angles.
    ///
    /// At AoA = 0° (heat-shield exactly into wind) with FPA = 0°,
    /// X_body = −unit(V) = [0,−1,0] in ECI. REFSMMAT maps that to
    /// X_body_SM = [−1,0,0] in SM:
    ///   CDUZ = arcsin(0) = 0°
    ///   CDUY = atan2(0, −1) = 180°
    ///   CDUX = 0°
    ///
    /// Note: the actual direct-LEO entry scenario uses FPA = −6°
    /// (velocity has a small radial component), which gives CDUY ≈ 174°.
    /// This test uses the simpler purely-circular geometry for a clean
    /// analytic assertion; `tc_cdu_trim_3` covers the FPA ≠ 0 case.
    #[test]
    fn tc_cdu_trim_1_circular_orbit_zero_aoa() {
        // Purely circular orbit: V perpendicular to R (FPA = 0°).
        let r = [6_493_000.0_f64, 0.0, 0.0];
        let v = [0.0_f64, 7860.0, 0.0];

        // Build REFSMMAT: Y_SM = unit(V×R), Z_SM = unit(-R), X_SM = Y×Z.
        let vxr = [
            v[1] * r[2] - v[2] * r[1],
            v[2] * r[0] - v[0] * r[2],
            v[0] * r[1] - v[1] * r[0],
        ];
        let y_sm = unit3(vxr).unwrap();
        let neg_r = [-r[0], -r[1], -r[2]];
        let z_sm = unit3(neg_r).unwrap();
        let x_sm = unit3(cross3(y_sm, z_sm)).unwrap();
        let refsmmat = [x_sm, y_sm, z_sm];

        let [cdux, cduy, cduz] = entry_trim_cdu_deg(r, v, refsmmat);

        // REFSMMAT rows: X_SM=[0,1,0], Y_SM=[0,0,−1], Z_SM=[−1,0,0].
        // X_body_SM = REFSMMAT·(−V_hat) = −[1,0,0] = [−1,0,0]:
        //   CDUZ = arcsin(0) = 0°, CDUY = atan2(0, −1) = 180°, CDUX = 0°.
        assert!(
            cdux.abs() < 0.1,
            "CDUX = {cdux:.4}°, expected ≈ 0° for FPA=0 circular orbit"
        );
        assert!(
            cduz.abs() < 0.1,
            "CDUZ = {cduz:.4}°, expected ≈ 0° for FPA=0 circular orbit"
        );
        // CDUY = 180° (heat-shield exactly into wind; ±180° are equivalent
        // for a CDU counter so accept both signs).
        assert!(
            cduy.abs() > 179.9,
            "CDUY = {cduy:.4}°, expected ≈ ±180° for FPA=0 heat-shield-forward"
        );
    }

    /// TC-CDU-TRIM-3: `entry_trim_cdu_deg` for the direct-LEO entry
    /// geometry (FPA = −6°, V = [−825, 7860, 0] m/s) gives CDUY ≈ 174°
    /// and AGC CALFA ≈ +1.0 > cos(45°).
    ///
    /// With X_body = −unit(V) (heat-shield-forward) and FPA = −6°:
    ///   X_body_SM ≈ [−0.995, 0, −0.104]
    ///   CDUZ = arcsin(0) ≈ 0°
    ///   CDUY = atan2(0.104, −0.995) ≈ 174° (= 180° − 6°)
    ///
    /// The AGC CALFA = dot(UL, ZB) ≈ +1.0 from the full CM/POSE gimbal
    /// computation (§CM_BODY_ATTITUDE.agc), which satisfies the EXDAP gate
    /// condition |CALFA| ≥ cos(45°) AND CALFA > 0.
    #[test]
    fn tc_cdu_trim_3_direct_leo_fpa_minus6() {
        let r = [6_493_000.0_f64, 0.0, 0.0];
        // FPA = -6° → Vx = 7900 sin(-6°) ≈ -825, Vy = 7900 cos(-6°) ≈ 7856.
        let v = [-825.0_f64, 7860.0, 0.0];

        let vxr = [
            v[1] * r[2] - v[2] * r[1],
            v[2] * r[0] - v[0] * r[2],
            v[0] * r[1] - v[1] * r[0],
        ];
        let y_sm = unit3(vxr).unwrap();
        let neg_r = [-r[0], -r[1], -r[2]];
        let z_sm = unit3(neg_r).unwrap();
        let x_sm = unit3(cross3(y_sm, z_sm)).unwrap();
        let refsmmat = [x_sm, y_sm, z_sm];

        let [cdux, cduy, cduz] = entry_trim_cdu_deg(r, v, refsmmat);

        // CDUX and CDUZ should be ≈ 0° (no roll or middle gimbal tilt).
        assert!(cdux.abs() < 0.5, "CDUX = {cdux:.4}°, expected ≈ 0°");
        assert!(cduz.abs() < 0.5, "CDUZ = {cduz:.4}°, expected ≈ 0°");

        // CDUY ≈ 174° = 180° − 6° (heat-shield-forward, pitched 6° down from
        // anti-velocity = the entry flight-path angle).
        assert!(
            (cduy - 174.0).abs() < 1.0,
            "CDUY = {cduy:.4}°, expected ≈ 174° for FPA=−6° heat-shield-forward"
        );

        // Sanity check: X_body_SM ≈ −unit(V_SM) → dot with V_SM ≈ −1.
        // This confirms heat-shield is into the wind (CALFA ≈ +1 in AGC).
        // Note: AGC CALFA = dot(UL, ZB) from the full CM/POSE computation,
        // not simply cos(CDUY).  The full computation gives CALFA ≈ +1.0
        // (verified analytically in the tc_e7i_j live run, 2026-07).
        let xb_sm = [cduy.to_radians().cos(), 0.0, -cduy.to_radians().sin()];
        // V_hat in SM: REFSMMAT · unit(v) = [0.995, 0, 0.104] for this geometry.
        let v_hat_sm = {
            let v_hat = unit3(v).unwrap();
            matvec3(refsmmat, v_hat)
        };
        let dot_xb_v = xb_sm[0] * v_hat_sm[0] + xb_sm[1] * v_hat_sm[1] + xb_sm[2] * v_hat_sm[2];
        assert!(
            dot_xb_v < -0.9,
            "dot(X_body_SM, V_SM) = {dot_xb_v:.4}, expected ≈ −1 (heat-shield forward)"
        );
    }

    /// TC-CDU-TRIM-2: `entry_trim_cdu_deg` returns `[0; 3]` for a
    /// degenerate (zero-velocity) input rather than NaN.
    #[test]
    fn tc_cdu_trim_2_zero_velocity_safe() {
        let r = [6_493_000.0, 0.0, 0.0];
        let v = [0.0_f64, 0.0, 0.0];
        let refsmmat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let target = entry_trim_cdu_deg(r, v, refsmmat);
        assert_eq!(target, [0.0; 3], "zero velocity must return safe default");
    }
}
