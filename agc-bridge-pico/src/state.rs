//! Shared bridge state (no peripheral ownership — just data).

pub struct BridgeState {
    /// Outbound sequence counter; wraps 255 → 0.
    pub tx_seq: u8,
    /// Milliseconds accumulated since boot (from SysTick).
    pub heartbeat_ms: u32,
    /// `heartbeat_ms` value at the last BridgeHeartbeat transmission.
    pub last_heartbeat_tx: u32,
    /// `heartbeat_ms` value at the last OpticsCdu transmission.
    pub last_cdu_tx: u32,
    /// Synthetic trunnion CDU angle (slow drift, 1 count ≈ 0.176 arc-sec).
    pub cdu_trunnion: u16,
    /// Synthetic shaft CDU angle.
    pub cdu_shaft: u16,
    /// `mission_time_cs` from the AGC's most recent AgcHeartbeat, if any.
    pub last_agc_heartbeat: Option<u32>,
    /// Whether the startup Hello has been acknowledged by the AGC.
    pub handshake_complete: bool,
    /// `heartbeat_ms` at last Hello send (for retry timeout).
    pub last_hello_tx: u32,
}

impl BridgeState {
    pub const fn new() -> Self {
        Self {
            tx_seq: 0,
            heartbeat_ms: 0,
            last_heartbeat_tx: 0,
            last_cdu_tx: 0,
            cdu_trunnion: 0,
            cdu_shaft: 0,
            last_agc_heartbeat: None,
            handshake_complete: false,
            last_hello_tx: 0,
        }
    }

    /// Increment and return the next sequence number.
    pub fn next_seq(&mut self) -> u8 {
        let s = self.tx_seq;
        self.tx_seq = self.tx_seq.wrapping_add(1);
        s
    }
}
