//! Shared bridge state, protected by a critical-section mutex (ADR-008).

use heapless::Deque;

/// All state that is written by the UART RX ISR and read by the HAL trait impls.
///
/// Wrapped as `static BRIDGE: Mutex<RefCell<BridgeState>>`.
/// Access is always inside `cortex_m::interrupt::free`.
pub struct BridgeState {
    /// Pending DSKY key codes received from the bridge (5-bit codes, AGC encoding).
    pub key_queue: Deque<u8, 16>,

    /// Last-cached optics trunnion CDU angle (bridge → AGC).
    pub optics_cdu_trunnion: i16,

    /// Last-cached optics shaft CDU angle (bridge → AGC).
    pub optics_cdu_shaft: i16,

    /// Sticky mark-pressed flag. Set by `OpticsMark`, cleared on first read.
    pub optics_mark_pending: bool,

    /// Last-cached SPS thrust-on discrete.
    pub engine_thrust_on: bool,

    /// Pending uplink words received from the bridge.
    pub uplink_queue: Deque<u16, 8>,

    /// Outbound sequence counter, incremented by `UartLink::send`.
    pub tx_seq: u8,

    /// Bridge uptime at the last `BridgeHeartbeat` message, in milliseconds.
    /// Used to detect bridge-side link loss; no consumer yet.
    pub last_bridge_heartbeat_ms: u32,
}

impl BridgeState {
    // `new` is `const fn` so it can initialise the global `BRIDGE` static;
    // `Default::default()` isn't const, so a Default impl wouldn't fit here.
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            key_queue: Deque::new(),
            optics_cdu_trunnion: 0,
            optics_cdu_shaft: 0,
            optics_mark_pending: false,
            engine_thrust_on: false,
            uplink_queue: Deque::new(),
            tx_seq: 0,
            last_bridge_heartbeat_ms: 0,
        }
    }
}
