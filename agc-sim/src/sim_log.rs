//! Ring-buffer event log for simulator state changes.
//!
//! Records labelled events (job established, alarm raised, PIPA pulse, etc.)
//! with a monotonic tick counter. The log is displayed in the TUI sidebar.

/// Maximum number of entries retained in the ring buffer.
pub const LOG_CAPACITY: usize = 200;

/// Severity / category of a log entry.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LogLevel {
    Info,
    Warn,
    Alarm,
    Io,
}

impl LogLevel {
    pub fn label(self) -> &'static str {
        match self {
            LogLevel::Info => "INFO ",
            LogLevel::Warn => "WARN ",
            LogLevel::Alarm => "ALARM",
            LogLevel::Io => "I/O  ",
        }
    }
}

/// A single log entry.
#[derive(Clone, Debug)]
pub struct LogEntry {
    pub tick: u64,
    pub level: LogLevel,
    pub message: String,
}

/// Fixed-capacity ring buffer of log entries.
pub struct SimLog {
    entries: Vec<LogEntry>,
    next_tick: u64,
}

impl SimLog {
    pub fn new() -> Self {
        Self {
            entries: Vec::with_capacity(LOG_CAPACITY),
            next_tick: 0,
        }
    }

    pub fn log(&mut self, level: LogLevel, message: impl Into<String>) {
        let entry = LogEntry {
            tick: self.next_tick,
            level,
            message: message.into(),
        };
        self.next_tick += 1;
        if self.entries.len() >= LOG_CAPACITY {
            self.entries.remove(0);
        }
        self.entries.push(entry);
    }

    pub fn info(&mut self, msg: impl Into<String>) {
        self.log(LogLevel::Info, msg);
    }

    pub fn warn(&mut self, msg: impl Into<String>) {
        self.log(LogLevel::Warn, msg);
    }

    pub fn alarm(&mut self, msg: impl Into<String>) {
        self.log(LogLevel::Alarm, msg);
    }

    pub fn io(&mut self, msg: impl Into<String>) {
        self.log(LogLevel::Io, msg);
    }

    /// All entries (oldest first).
    pub fn entries(&self) -> &[LogEntry] {
        &self.entries
    }

    /// The N most recent entries.
    pub fn tail(&self, n: usize) -> &[LogEntry] {
        let len = self.entries.len();
        &self.entries[len.saturating_sub(n)..]
    }
}

impl Default for SimLog {
    fn default() -> Self {
        Self::new()
    }
}
