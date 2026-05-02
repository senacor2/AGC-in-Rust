//! USB CDC-ACM developer console.
//!
//! Wraps a `usbd_serial::SerialPort` and the `usb_device::UsbDevice`.
//! Non-blocking on both TX and RX: bytes that cannot be sent immediately
//! (no host connected, TX buffer full) are silently dropped.
//!
//! `poll` must be called every loop iteration to service the USB device stack.
//! Incoming bytes are accumulated in a small ring buffer; `drain_rx` + `clear_rx`
//! return and then consume the buffered bytes.

use agc_protocol::Msg;
use heapless::Vec;
use rp2040_hal::usb::UsbBus;
use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
};
use usbd_serial::SerialPort;

/// Maximum bytes buffered from the USB host between `drain_rx` calls.
const RX_BUF: usize = 64;

pub struct UsbConsole {
    device: UsbDevice<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,
    rx_buf: Vec<u8, RX_BUF>,
}

impl UsbConsole {
    /// Construct a new `UsbConsole`.
    ///
    /// `bus_alloc` must be a `'static` reference so the device and class
    /// can borrow it for the device's lifetime.
    pub fn new(bus_alloc: &'static UsbBusAllocator<UsbBus>) -> Self {
        let serial = SerialPort::new(bus_alloc);

        // usb-device 0.3: string descriptors go through `StringDescriptors`
        // and are attached via `UsbDeviceBuilder::strings`.
        let strings = StringDescriptors::default()
            .manufacturer("AGC-in-Rust")
            .product("AGC Bridge")
            .serial_number("0001");

        let device = UsbDeviceBuilder::new(bus_alloc, UsbVidPid(0x2E8A, 0x000A))
            .device_class(usbd_serial::USB_CLASS_CDC)
            .strings(&[strings])
            .unwrap_or_else(|_| {
                // String registration can only fail if the heapless vec is full.
                // With one descriptor set it never fills; reaching here is a bug.
                cortex_m::peripheral::SCB::sys_reset()
            })
            .build();

        Self {
            device,
            serial,
            rx_buf: Vec::new(),
        }
    }

    /// Drive the USB device state machine and buffer incoming bytes.
    ///
    /// Must be called at least once per millisecond to meet USB timing
    /// requirements.
    pub fn poll(&mut self) {
        if self.device.poll(&mut [&mut self.serial]) {
            let mut tmp = [0u8; RX_BUF];
            match self.serial.read(&mut tmp) {
                Ok(n) if n > 0 => {
                    for &b in &tmp[..n] {
                        // Silently drop on overflow; the developer can re-type.
                        let _ = self.rx_buf.push(b);
                    }
                }
                _ => {}
            }
        }
    }

    /// Return a slice of bytes received since the last `clear_rx` call.
    pub fn drain_rx(&self) -> &[u8] {
        &self.rx_buf
    }

    /// Clear the RX buffer (call after processing `drain_rx`).
    pub fn clear_rx(&mut self) {
        self.rx_buf.clear();
    }

    /// Write `bytes` to the USB serial port.
    ///
    /// Non-blocking: bytes are dropped if the host is not connected or the
    /// TX buffer is full.
    pub fn write(&mut self, bytes: &[u8]) {
        if self.device.state() != UsbDeviceState::Configured {
            return;
        }
        let _ = self.serial.write(bytes);
    }

    /// Pretty-print one decoded AGC→bridge message to the console.
    pub fn log_agc_msg(&mut self, msg: &Msg) {
        use core::fmt::Write as FmtWrite;
        let mut buf = heapless::String::<128>::new();
        let _ = match msg {
            Msg::DskyWriteRow { row, data } => {
                write!(
                    buf,
                    "AGC> DSKY_WRITE_ROW row={} data=0x{:03X}\r\n",
                    row, data
                )
            }
            Msg::DskyClearRow { row } => {
                write!(buf, "AGC> DSKY_CLEAR_ROW row={}\r\n", row)
            }
            Msg::DskySetLamp { lamp, on } => {
                write!(buf, "AGC> DSKY_SET_LAMP lamp={} on={}\r\n", lamp, on)
            }
            Msg::DskySetFlash { on } => {
                write!(buf, "AGC> DSKY_SET_FLASH on={}\r\n", on)
            }
            Msg::OpticsDrive { trunnion, shaft } => {
                write!(
                    buf,
                    "AGC> OPTICS_DRIVE trunnion={} shaft={}\r\n",
                    trunnion, shaft
                )
            }
            Msg::EngineSpsEnable { on } => {
                write!(buf, "AGC> ENGINE_SPS_ENABLE on={}\r\n", on)
            }
            Msg::EngineSpsGimbal { pitch, yaw } => {
                write!(
                    buf,
                    "AGC> ENGINE_SPS_GIMBAL pitch={} yaw={}\r\n",
                    pitch, yaw
                )
            }
            Msg::RcsFireSm { jets_a, jets_b } => {
                write!(
                    buf,
                    "AGC> RCS_FIRE_SM jets_a=0x{:02X} jets_b=0x{:02X}\r\n",
                    jets_a, jets_b
                )
            }
            Msg::RcsFireCm { jets } => {
                write!(buf, "AGC> RCS_FIRE_CM jets=0x{:04X}\r\n", jets)
            }
            Msg::RcsQuenchAll => {
                write!(buf, "AGC> RCS_QUENCH_ALL\r\n")
            }
            Msg::TelemetryWord { word } => {
                write!(buf, "AGC> TELEMETRY_WORD word=0x{:04X}\r\n", word)
            }
            Msg::AgcHeartbeat { mission_time_cs } => {
                write!(buf, "AGC> AGC_HEARTBEAT met={}cs\r\n", mission_time_cs)
            }
            Msg::HelloAck { proto_version } => {
                write!(buf, "AGC> HELLO_ACK proto={}\r\n", proto_version)
            }
            Msg::Error { code, ctx } => {
                write!(buf, "AGC> ERROR code=0x{:02X} ctx=0x{:02X}\r\n", code, ctx)
            }
            _ => {
                write!(buf, "AGC> <unexpected inbound type>\r\n")
            }
        };
        self.write(buf.as_bytes());
    }
}
