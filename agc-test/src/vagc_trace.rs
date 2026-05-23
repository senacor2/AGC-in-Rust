//! Channel-write recorder, JSON fixture format, and comparator for
//! MS-E7c end-to-end entry-guidance traces.
//!
//! Three building blocks:
//!
//! 1. [`ChannelTraceRecorder`] — owns a [`crate::vagc_channel::YaAgcClient`]
//!    and drains it into a time-stamped, in-memory event list. The
//!    timestamps are wall-clock millisecond offsets from
//!    [`ChannelTraceRecorder::new`]. For cycle-by-cycle comparison the
//!    test driver advances yaAGC and the Rust pipeline in lock-step, so
//!    wall-clock time is sufficient; absolute AGC simulation time can
//!    be recovered later by indexing into the captured SERVICER ticks.
//! 2. [`ChannelTrace`] — the on-disk JSON fixture format. Serde-derived
//!    so it round-trips cleanly through `serde_json`. Field set is
//!    deliberately tight: any later addition lands as a new optional
//!    field so already-committed fixtures still load.
//! 3. [`compare`] — diff two traces under a [`CompareTolerance`]
//!    configuration. Returns a [`CompareReport`] describing the
//!    per-channel discrepancies.

use std::collections::{BTreeMap, BTreeSet};
use std::io;
use std::path::Path;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::vagc_channel::YaAgcClient;

// ── On-disk fixture format ─────────────────────────────────────────────────

/// One AGC → peripheral channel write recorded by [`ChannelTraceRecorder`].
///
/// The `value` is the 15-bit AGC word as delivered on the wire (the
/// sign-bit-in-bit-14 ones-complement form). The recorder does not
/// scale or interpret the value; consumers do their own decoding
/// per channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChannelEvent {
    /// Wall-clock millisecond offset from the recorder's `t0`.
    pub t_ms: u32,
    /// AGC channel number (0–0o57 for documented Comanche055 outputs).
    pub channel: u16,
    /// 15-bit AGC word (sign in bit 14).
    pub value: u16,
}

/// On-disk JSON channel-trace fixture.
///
/// Committed alongside an end-to-end scenario; loaded by Rust-only
/// assertion tests so CI can validate trace structure without needing
/// a local VirtualAGC build.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChannelTrace {
    /// Human-readable scenario name (e.g., `"entry_direct_leo"`).
    pub scenario: String,
    /// Free-form provenance line: how the trace was captured (yaAGC
    /// version, rope hash, capture date). Not parsed by tools.
    pub provenance: String,
    /// Event stream, in capture order. The capture order **is** the
    /// time order: `t_ms` is monotonically non-decreasing.
    pub events: Vec<ChannelEvent>,
}

impl ChannelTrace {
    /// Load a trace from a JSON file.
    pub fn load<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let text = std::fs::read_to_string(path)?;
        serde_json::from_str(&text).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
    }

    /// Save a trace to a JSON file.
    pub fn save<P: AsRef<Path>>(&self, path: P) -> io::Result<()> {
        let text = serde_json::to_string_pretty(self)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        std::fs::write(path, text)
    }

    /// Iterate over the unique channel numbers seen in this trace.
    pub fn channels(&self) -> BTreeSet<u16> {
        self.events.iter().map(|e| e.channel).collect()
    }

    /// Final (most recent) value seen on each channel.
    pub fn final_values(&self) -> BTreeMap<u16, u16> {
        let mut out = BTreeMap::new();
        for ev in &self.events {
            out.insert(ev.channel, ev.value);
        }
        out
    }

    /// Number of writes to a given channel.
    pub fn write_count(&self, channel: u16) -> usize {
        self.events.iter().filter(|e| e.channel == channel).count()
    }
}

// ── Live recorder ──────────────────────────────────────────────────────────

/// Drain channel packets from a `YaAgcClient` and accumulate them into
/// a time-stamped trace.
///
/// The recorder owns the client. To send packets back to yaAGC at the
/// same time, the test driver should hold a *separate* connection
/// (yaAGC delivers every channel write to every connected peripheral).
pub struct ChannelTraceRecorder {
    client: YaAgcClient,
    t0: Instant,
    events: Vec<ChannelEvent>,
}

impl ChannelTraceRecorder {
    /// Wrap a client. The capture clock starts immediately.
    pub fn new(client: YaAgcClient) -> Self {
        Self {
            client,
            t0: Instant::now(),
            events: Vec::new(),
        }
    }

    /// Drain available packets from the socket, blocking up to `budget`
    /// total wall-clock time. Returns the number of packets captured.
    ///
    /// This is the workhorse polling primitive: a test driver typically
    /// calls it once per SERVICER cycle (2 s) with a small `budget`
    /// (e.g., 50 ms) to keep up with yaAGC's output rate without
    /// stalling the test.
    pub fn drain(&mut self, budget: Duration) -> usize {
        let deadline = Instant::now() + budget;
        let mut captured = 0;
        loop {
            let remaining = match deadline.checked_duration_since(Instant::now()) {
                Some(d) if !d.is_zero() => d,
                _ => break,
            };
            // Use a small per-read timeout so we don't block the whole
            // budget on a single recv when the socket is empty.
            let chunk = remaining.min(Duration::from_millis(5));
            match self.client.try_recv(chunk) {
                Ok(pkt) => {
                    let t_ms = Instant::now()
                        .saturating_duration_since(self.t0)
                        .as_millis()
                        .min(u32::MAX as u128) as u32;
                    self.events.push(ChannelEvent {
                        t_ms,
                        channel: pkt.channel,
                        value: pkt.value,
                    });
                    captured += 1;
                }
                Err(e)
                    if e.kind() == io::ErrorKind::TimedOut
                        || e.kind() == io::ErrorKind::WouldBlock =>
                {
                    // No more packets right now; keep polling until
                    // the budget expires in case yaAGC emits more.
                }
                Err(_) => break,
            }
        }
        captured
    }

    /// Consume the recorder and produce a [`ChannelTrace`].
    pub fn into_trace(
        self,
        scenario: impl Into<String>,
        provenance: impl Into<String>,
    ) -> ChannelTrace {
        ChannelTrace {
            scenario: scenario.into(),
            provenance: provenance.into(),
            events: self.events,
        }
    }

    /// Borrow the captured events without consuming the recorder.
    pub fn events(&self) -> &[ChannelEvent] {
        &self.events
    }

    /// Number of events captured so far.
    pub fn len(&self) -> usize {
        self.events.len()
    }

    /// True if no events have been captured.
    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }
}

// ── Comparator ─────────────────────────────────────────────────────────────

/// How strictly to compare two traces, per channel class.
#[derive(Clone, Debug)]
pub struct CompareTolerance {
    /// Channels compared as a bag of `(channel, value)` events ignoring
    /// timing. Two traces match iff the multiset of writes is equal.
    ///
    /// Default population: channel `0o05` (CM RCS), `0o30`–`0o33`
    /// (output discrete groups, including SM RCS and alarms).
    pub event_exact_channels: BTreeSet<u16>,
    /// Channels compared only by their final (last-seen) value. For
    /// DSKY display channels that get rewritten on every T4RUPT.
    ///
    /// Default population: `0o10`–`0o13` (DSKY display + DSALMOUT).
    pub final_value_channels: BTreeSet<u16>,
    /// Channels deliberately ignored in the comparison. Defaults to
    /// empty; populate with channels known to be sensitive to wall-
    /// clock timing or scenario nondeterminism.
    pub ignored_channels: BTreeSet<u16>,
}

impl Default for CompareTolerance {
    fn default() -> Self {
        Self {
            event_exact_channels: [0o05, 0o30, 0o31, 0o32, 0o33].into_iter().collect(),
            final_value_channels: [0o10, 0o11, 0o12, 0o13].into_iter().collect(),
            ignored_channels: BTreeSet::new(),
        }
    }
}

/// One difference found by [`compare`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Difference {
    /// `actual` is missing one or more writes to `channel` that `expected`
    /// contains. The `count` is the size of the missing multiset.
    MissingEvent {
        channel: u16,
        value: u16,
        count: usize,
    },
    /// `actual` contains a write to `channel` that `expected` does not.
    UnexpectedEvent {
        channel: u16,
        value: u16,
        count: usize,
    },
    /// The final-value channel diverges between `expected` and `actual`.
    FinalValueMismatch {
        channel: u16,
        expected: u16,
        actual: u16,
    },
    /// Channel saw activity in `actual` but never in `expected` (or
    /// vice versa) and is outside the configured tolerance.
    UnconfiguredChannel { channel: u16, in_actual: bool },
}

/// Per-channel result of comparing two traces.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CompareReport {
    /// All discrepancies found, in iteration order.
    pub differences: Vec<Difference>,
    /// Channels examined under [`CompareTolerance::event_exact_channels`].
    pub event_exact_channels_examined: usize,
    /// Channels examined under [`CompareTolerance::final_value_channels`].
    pub final_value_channels_examined: usize,
}

impl CompareReport {
    /// True if no discrepancies were found.
    pub fn is_match(&self) -> bool {
        self.differences.is_empty()
    }
}

/// Compare two traces under the given tolerance configuration.
///
/// `expected` is the baseline (e.g., the committed JSON fixture);
/// `actual` is the candidate (e.g., a freshly captured trace, or one
/// projected from the Rust pipeline). Differences are reported
/// directionally — `MissingEvent` means "in expected, not in actual",
/// `UnexpectedEvent` means "in actual, not in expected".
pub fn compare(
    expected: &ChannelTrace,
    actual: &ChannelTrace,
    tol: &CompareTolerance,
) -> CompareReport {
    let mut report = CompareReport::default();

    // Event-exact channels: multiset equality.
    for &channel in &tol.event_exact_channels {
        if tol.ignored_channels.contains(&channel) {
            continue;
        }
        report.event_exact_channels_examined += 1;
        let exp = multiset_for_channel(expected, channel);
        let act = multiset_for_channel(actual, channel);
        for (&value, &exp_count) in &exp {
            let act_count = act.get(&value).copied().unwrap_or(0);
            if act_count < exp_count {
                report.differences.push(Difference::MissingEvent {
                    channel,
                    value,
                    count: exp_count - act_count,
                });
            }
        }
        for (&value, &act_count) in &act {
            let exp_count = exp.get(&value).copied().unwrap_or(0);
            if act_count > exp_count {
                report.differences.push(Difference::UnexpectedEvent {
                    channel,
                    value,
                    count: act_count - exp_count,
                });
            }
        }
    }

    // Final-value channels: just compare the most recent write.
    let exp_final = expected.final_values();
    let act_final = actual.final_values();
    for &channel in &tol.final_value_channels {
        if tol.ignored_channels.contains(&channel) {
            continue;
        }
        report.final_value_channels_examined += 1;
        let exp = exp_final.get(&channel).copied();
        let act = act_final.get(&channel).copied();
        if exp != act {
            report.differences.push(Difference::FinalValueMismatch {
                channel,
                expected: exp.unwrap_or(0),
                actual: act.unwrap_or(0),
            });
        }
    }

    // Channels active in one trace but absent from the tolerance
    // configuration. Helps catch new channels that need a comparison
    // policy decision.
    let configured: BTreeSet<u16> = tol
        .event_exact_channels
        .iter()
        .chain(tol.final_value_channels.iter())
        .chain(tol.ignored_channels.iter())
        .copied()
        .collect();
    for &channel in &actual.channels() {
        if !configured.contains(&channel) {
            report.differences.push(Difference::UnconfiguredChannel {
                channel,
                in_actual: true,
            });
        }
    }
    for &channel in &expected.channels() {
        if !configured.contains(&channel) && !actual.channels().contains(&channel) {
            report.differences.push(Difference::UnconfiguredChannel {
                channel,
                in_actual: false,
            });
        }
    }

    report
}

fn multiset_for_channel(trace: &ChannelTrace, channel: u16) -> BTreeMap<u16, usize> {
    let mut out = BTreeMap::new();
    for ev in &trace.events {
        if ev.channel == channel {
            *out.entry(ev.value).or_insert(0) += 1;
        }
    }
    out
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(t_ms: u32, channel: u16, value: u16) -> ChannelEvent {
        ChannelEvent {
            t_ms,
            channel,
            value,
        }
    }

    /// TC-TRACE-IO-1: a trace round-trips through JSON serialize/deserialize.
    #[test]
    fn tc_trace_io_1_round_trip_json() {
        let original = ChannelTrace {
            scenario: "smoke".into(),
            provenance: "test fixture".into(),
            events: vec![ev(0, 0o10, 0o123), ev(120, 0o11, 0o555)],
        };
        let json = serde_json::to_string(&original).unwrap();
        let back: ChannelTrace = serde_json::from_str(&json).unwrap();
        assert_eq!(original, back);
    }

    /// TC-TRACE-IO-2: load/save round trip via the filesystem.
    #[test]
    fn tc_trace_io_2_round_trip_file() {
        let original = ChannelTrace {
            scenario: "smoke".into(),
            provenance: "test fixture".into(),
            events: vec![ev(0, 0o10, 0o123)],
        };
        let path =
            std::env::temp_dir().join(format!("vagc_trace_io_test_{}.json", std::process::id()));
        original.save(&path).unwrap();
        let loaded = ChannelTrace::load(&path).unwrap();
        assert_eq!(original, loaded);
        let _ = std::fs::remove_file(path);
    }

    /// TC-TRACE-CMP-1: identical traces compare as a match.
    #[test]
    fn tc_trace_cmp_1_identical_match() {
        let trace = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![ev(0, 0o10, 0o123), ev(10, 0o30, 0o400)],
        };
        let report = compare(&trace, &trace, &CompareTolerance::default());
        assert!(report.is_match(), "expected match, got {:?}", report);
    }

    /// TC-TRACE-CMP-2: event-exact mismatch — extra write in `actual`.
    #[test]
    fn tc_trace_cmp_2_event_exact_unexpected() {
        let expected = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![ev(0, 0o30, 0o100)],
        };
        let actual = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![ev(0, 0o30, 0o100), ev(20, 0o30, 0o200)],
        };
        let report = compare(&expected, &actual, &CompareTolerance::default());
        assert!(!report.is_match());
        assert!(
            report.differences.iter().any(|d| matches!(
                d,
                Difference::UnexpectedEvent {
                    channel: 0o30,
                    value: 0o200,
                    count: 1,
                }
            )),
            "expected UnexpectedEvent on 0o30 = 0o200, got {:?}",
            report
        );
    }

    /// TC-TRACE-CMP-3: event-exact mismatch — missing write in `actual`.
    #[test]
    fn tc_trace_cmp_3_event_exact_missing() {
        let expected = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![ev(0, 0o30, 0o100), ev(10, 0o30, 0o100)],
        };
        let actual = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![ev(0, 0o30, 0o100)],
        };
        let report = compare(&expected, &actual, &CompareTolerance::default());
        assert!(!report.is_match());
        assert!(report.differences.iter().any(|d| matches!(
            d,
            Difference::MissingEvent {
                channel: 0o30,
                value: 0o100,
                count: 1,
            }
        )));
    }

    /// TC-TRACE-CMP-4: final-value mismatch on a display channel.
    #[test]
    fn tc_trace_cmp_4_final_value_mismatch() {
        let expected = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![ev(0, 0o10, 0o100), ev(10, 0o10, 0o123)],
        };
        let actual = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![ev(0, 0o10, 0o100), ev(10, 0o10, 0o124)],
        };
        let report = compare(&expected, &actual, &CompareTolerance::default());
        assert!(!report.is_match());
        assert!(report.differences.iter().any(|d| matches!(
            d,
            Difference::FinalValueMismatch {
                channel: 0o10,
                expected: 0o123,
                actual: 0o124,
            }
        )));
    }

    /// TC-TRACE-CMP-5: an unconfigured channel is flagged so the test
    /// author has to make a tolerance decision before merging.
    #[test]
    fn tc_trace_cmp_5_unconfigured_channel_flagged() {
        let expected = ChannelTrace::default();
        let actual = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![ev(0, 0o42, 0o7)],
        };
        let report = compare(&expected, &actual, &CompareTolerance::default());
        assert!(report.differences.iter().any(|d| matches!(
            d,
            Difference::UnconfiguredChannel {
                channel: 0o42,
                in_actual: true,
            }
        )));
    }

    /// TC-TRACE-CMP-6: ignored channels suppress all diagnostics.
    #[test]
    fn tc_trace_cmp_6_ignored_channel_suppresses() {
        let expected = ChannelTrace::default();
        let actual = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![ev(0, 0o42, 0o7)],
        };
        let mut tol = CompareTolerance::default();
        tol.ignored_channels.insert(0o42);
        let report = compare(&expected, &actual, &tol);
        assert!(report.is_match(), "expected match, got {:?}", report);
    }

    /// TC-TRACE-DERIVE-1: helpers `channels`, `final_values`, and
    /// `write_count` aggregate the event stream correctly.
    #[test]
    fn tc_trace_derive_1_aggregation_helpers() {
        let trace = ChannelTrace {
            scenario: "x".into(),
            provenance: "x".into(),
            events: vec![
                ev(0, 0o10, 0o100),
                ev(120, 0o10, 0o123),
                ev(120, 0o11, 0o200),
            ],
        };
        let channels = trace.channels();
        assert!(channels.contains(&0o10));
        assert!(channels.contains(&0o11));
        let finals = trace.final_values();
        assert_eq!(finals.get(&0o10), Some(&0o123));
        assert_eq!(finals.get(&0o11), Some(&0o200));
        assert_eq!(trace.write_count(0o10), 2);
        assert_eq!(trace.write_count(0o11), 1);
    }
}
