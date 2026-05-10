//! Embassy-facing BLE runtime skeleton.
//!
//! This module preserves the useful TMOS event model while giving the Rust BLE
//! stack explicit ownership boundaries:
//!
//! - BB/LLE IRQs keep hard real-time radio work small and signal tasks.
//! - `ble_ll_task` owns advertising, CONNECT_IND parsing, and connection timing.
//! - `ble_host_task` owns GAP/GATT/application events.

use core::cell::Cell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Duration;

/// WCH TMOS message event bit.
pub const SYS_EVENT_MSG: u16 = 0x8000;

/// WCH TMOS logical time unit.
pub const TMOS_TICK_MICROS: u64 = 625;

/// Convert TMOS logical ticks to Embassy time.
///
/// This conversion applies to `tmos_start_task` / `TMOS_GetSystemClock`
/// semantics. Low-level BLE controller clock callbacks use a separate raw
/// counter contract.
#[inline]
pub const fn tmos_ticks_to_duration(ticks: u32) -> Duration {
    Duration::from_micros(ticks as u64 * TMOS_TICK_MICROS)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum BleTaskId {
    Ll = 0,
    Host = 1,
    App = 2,
}

/// TMOS-style 16-bit event mask plus a single wake signal.
///
/// `set()` is safe to call from ISR context on single-core CH32 RISC-V:
/// `CriticalSectionRawMutex` disables interrupts idempotently in ISR.
///
/// The signal only wakes the task. Event identity lives in `bits`, so multiple
/// ISR-side `set()` calls coalesce safely before the task drains them.
pub struct TaskEvents {
    bits: Mutex<CriticalSectionRawMutex, Cell<u16>>,
    wake: Signal<CriticalSectionRawMutex, ()>,
}

impl TaskEvents {
    pub const fn new() -> Self {
        Self {
            bits: Mutex::new(Cell::new(0)),
            wake: Signal::new(),
        }
    }

    /// Equivalent to `tmos_set_event(task, event)`.
    #[inline]
    pub fn set(&self, event: u16) {
        self.bits.lock(|bits| bits.set(bits.get() | event));
        self.wake.signal(());
    }

    /// Equivalent to `tmos_clear_event(task, event)`.
    ///
    /// Use only for explicit cancellation. Dispatch loops should prefer
    /// `take()` and process the local copy to avoid clearing ISR-set bits.
    #[inline]
    pub fn clear(&self, event: u16) {
        self.bits.lock(|bits| bits.set(bits.get() & !event));
    }

    /// Atomically take all pending bits for TMOS-style dispatch.
    #[inline]
    pub fn take(&self) -> u16 {
        self.bits.lock(|bits| {
            let pending = bits.get();
            bits.set(0);
            pending
        })
    }

    #[inline]
    pub async fn wait(&self) {
        self.wake.wait().await;
    }

    /// Wait until at least one event bit is pending, then return the full mask.
    pub async fn wait_and_take(&self) -> u16 {
        loop {
            let bits = self.take();
            if bits != 0 {
                return bits;
            }
            self.wait().await;
        }
    }
}

impl Default for TaskEvents {
    fn default() -> Self {
        Self::new()
    }
}

/// Timer cancel token for `tmos_start_task`-style wrappers.
pub struct BleTimer {
    value: Mutex<CriticalSectionRawMutex, Cell<u32>>,
}

impl BleTimer {
    pub const fn new() -> Self {
        Self {
            value: Mutex::new(Cell::new(0)),
        }
    }

    #[inline]
    pub fn next(&self) -> u32 {
        self.value.lock(|value| {
            let next = value.get().wrapping_add(1);
            value.set(next);
            next
        })
    }

    #[inline]
    pub fn current(&self) -> u32 {
        self.value.lock(|value| value.get())
    }

    #[inline]
    pub fn is_current(&self, generation: u32) -> bool {
        self.current() == generation
    }
}

impl Default for BleTimer {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LlEvent {
    AdvertisingEventDone { channel: u8 },
    ConnectInd(ConnectIndParams),
    RadioWindowMissed,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GapEvent {
    LinkEstablished(ConnectionContext),
    LinkTerminated { reason: u8 },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GattEvent {
    AttPacket { handle: u16, opcode: u8 },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AppEvent {
    Periodic,
    ParamUpdate,
    ReadRssi,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BleMsg {
    Ll(LlEvent),
    Gap(GapEvent),
    Gatt(GattEvent),
    App(AppEvent),
}

pub type BleMsgChannel<const N: usize> = Channel<CriticalSectionRawMutex, BleMsg, N>;

/// LL controller task body placeholder.
///
/// Phase B will move connectable ADV scheduling here. Phase C will add
/// CONNECT_IND capture delivery and connection-event scheduling.
pub async fn ble_ll_task(events: &'static TaskEvents) -> ! {
    loop {
        let _bits = events.wait_and_take().await;
    }
}

/// Host/app task body placeholder.
///
/// This is the `TMOS_SystemProcess` replacement: drain event bits, then drain
/// `SYS_EVENT_MSG`-equivalent messages from the bounded channel.
pub async fn ble_host_task<const N: usize>(
    events: &'static TaskEvents,
    messages: &'static BleMsgChannel<N>,
) -> ! {
    loop {
        let bits = events.wait_and_take().await;
        if bits & SYS_EVENT_MSG != 0 {
            while let Ok(_msg) = messages.try_receive() {}
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectIndParams {
    pub init_a: [u8; 6],
    pub adv_a: [u8; 6],
    pub access_addr: u32,
    pub crc_init: u32,
    pub win_size: u8,
    pub win_offset: u16,
    pub interval: u16,
    pub latency: u16,
    pub timeout: u16,
    pub channel_map: [u8; 5],
    pub hop: u8,
    pub sca: u8,
}

impl ConnectIndParams {
    /// Parse a legacy advertising PDU snapshot containing a CONNECT_IND.
    ///
    /// `pdu` starts at the two-byte advertising header. CONNECT_IND payload is
    /// 34 bytes: InitA, AdvA, and the 22-byte LLData block.
    pub fn parse_legacy_pdu(pdu: &[u8]) -> Option<Self> {
        if pdu.len() < 36 {
            return None;
        }
        if pdu[0] & 0x0f != 0x05 {
            return None;
        }
        if pdu[1] & 0x3f != 34 {
            return None;
        }

        let mut init_a = [0u8; 6];
        init_a.copy_from_slice(&pdu[2..8]);
        let mut adv_a = [0u8; 6];
        adv_a.copy_from_slice(&pdu[8..14]);
        let ll = &pdu[14..36];

        let access_addr = u32::from_le_bytes([ll[0], ll[1], ll[2], ll[3]]);
        let crc_init = u32::from(ll[4]) | (u32::from(ll[5]) << 8) | (u32::from(ll[6]) << 16);
        let win_size = ll[7];
        let win_offset = u16::from_le_bytes([ll[8], ll[9]]);
        let interval = u16::from_le_bytes([ll[10], ll[11]]);
        let latency = u16::from_le_bytes([ll[12], ll[13]]);
        let timeout = u16::from_le_bytes([ll[14], ll[15]]);
        let mut channel_map = [0u8; 5];
        channel_map.copy_from_slice(&ll[16..21]);
        let hop_sca = ll[21];

        Some(Self {
            init_a,
            adv_a,
            access_addr,
            crc_init,
            win_size,
            win_offset,
            interval,
            latency,
            timeout,
            channel_map,
            hop: hop_sca & 0x1f,
            sca: hop_sca >> 5,
        })
    }
}

/// LL connection state owned by `ble_ll_task`.
///
/// FUTURE: Phase 1 is single-link. Multi-link support needs an indexed context
/// pool plus explicit connection-handle allocation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionContext {
    pub handle: u16,
    pub params: ConnectIndParams,
}
