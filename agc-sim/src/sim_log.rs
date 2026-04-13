//! Mission log ring buffer for the AGC simulator.
//!
//! Provides a fixed-capacity (256-line) circular log of structured entries
//! that the Mission Log TUI panel can replay.  Heap allocation is permitted
//! in this host-side crate.

use std::time::Instant;

/// Severity level for a log line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogLevel {
    /// Informational message.
    Info,
    /// Non-fatal warning.
    Warn,
    /// Error condition (non-fatal in sim context).
    Error,
}

impl LogLevel {
    /// Short string tag used in rendering.
    pub fn tag(self) -> &'static str {
        match self {
            Self::Info => "INFO",
            Self::Warn => "WARN",
            Self::Error => "ERR ",
        }
    }
}

/// A single structured log entry.
#[derive(Clone, Debug)]
pub struct LogLine {
    /// Wall-clock time at which the entry was produced.
    pub timestamp: Instant,
    /// Severity level.
    pub level: LogLevel,
    /// Human-readable message text.
    pub text: String,
}

/// Maximum number of lines retained in the ring buffer.
const CAPACITY: usize = 256;

/// Fixed-capacity ring buffer of mission log lines.
///
/// When full, the oldest entry is silently dropped to make room for the newest.
pub struct SimLog {
    buffer: Vec<LogLine>,
    /// Write head index (wraps on overflow).
    head: usize,
    /// Total number of entries ever pushed (used to detect full/wrap state).
    total: usize,
}

impl SimLog {
    /// Construct an empty log.
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(CAPACITY),
            head: 0,
            total: 0,
        }
    }

    /// Append an entry at the given severity level.
    fn push(&mut self, level: LogLevel, text: impl Into<String>) {
        let entry = LogLine {
            timestamp: Instant::now(),
            level,
            text: text.into(),
        };
        if self.total < CAPACITY {
            self.buffer.push(entry);
        } else {
            // Wrap: overwrite oldest slot.
            self.buffer[self.head] = entry;
        }
        self.head = (self.head + 1) % CAPACITY;
        self.total = self.total.saturating_add(1);
    }

    /// Append an `Info`-level message.
    pub fn info(&mut self, text: impl Into<String>) {
        self.push(LogLevel::Info, text);
    }

    /// Append a `Warn`-level message.
    pub fn warn(&mut self, text: impl Into<String>) {
        self.push(LogLevel::Warn, text);
    }

    /// Append an `Error`-level message.
    pub fn error(&mut self, text: impl Into<String>) {
        self.push(LogLevel::Error, text);
    }

    /// Iterate over all retained lines in chronological order (oldest first).
    ///
    /// Returns at most `CAPACITY` entries.
    pub fn lines(&self) -> impl Iterator<Item = &LogLine> {
        let len = self.buffer.len();
        if self.total <= CAPACITY {
            // Buffer is not yet full: simple slice from 0..len.
            let (a, b) = self.buffer.split_at(len);
            a.iter().chain(b.iter())
        } else {
            // Buffer has wrapped: oldest entry is at `head`, newest just before.
            let (newer, older) = self.buffer.split_at(self.head);
            older.iter().chain(newer.iter())
        }
    }

    /// Number of entries currently held (capped at `CAPACITY`).
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// True when no entries have been recorded.
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }
}

impl Default for SimLog {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_and_iterate() {
        let mut log = SimLog::new();
        log.info("hello");
        log.warn("world");
        log.error("oops");
        let lines: Vec<_> = log.lines().collect();
        assert_eq!(lines.len(), 3);
        assert_eq!(lines[0].level, LogLevel::Info);
        assert_eq!(lines[1].level, LogLevel::Warn);
        assert_eq!(lines[2].level, LogLevel::Error);
    }

    #[test]
    fn ring_buffer_wraps_at_capacity() {
        let mut log = SimLog::new();
        for i in 0..(CAPACITY + 10) {
            log.info(format!("line {i}"));
        }
        // Should retain exactly CAPACITY entries.
        assert_eq!(log.len(), CAPACITY);
        // The oldest retained line should be entry 10 (lines 0-9 were evicted).
        let lines: Vec<_> = log.lines().collect();
        assert_eq!(lines[0].text, "line 10");
        assert_eq!(lines[CAPACITY - 1].text, format!("line {}", CAPACITY + 9));
    }

    #[test]
    fn is_empty_initially() {
        let log = SimLog::new();
        assert!(log.is_empty());
        assert_eq!(log.len(), 0);
    }
}
