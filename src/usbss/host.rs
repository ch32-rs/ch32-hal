//! USBSS host mode (RM §27.2.5).
//!
//! [`HostDriver`] implements the `embassy-usb` host traits for one SuperSpeed
//! device on the root port. Hubs, streams and isochronous pipes are not
//! supported.
//!
//! The controller has one OUT (`UH_TX`) and one IN (`UH_RX`) transaction
//! engine and one device address register. Pipes take turns on them. A pipe
//! the device answers with NRDY gives them up until the device's ERDY.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{fence, AtomicBool, AtomicU32, AtomicUsize, Ordering};
use core::task::Poll;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb_driver::host::{
    pipe, DeviceEvent, HostError, PipeError, SplitInfo, TimeoutConfig, UsbHostAllocator, UsbHostController, UsbPipe,
};
use embassy_usb_driver::{EndpointInfo, EndpointType, Speed};

use super::regs::{Regs, Role};
use super::{
    dma_capable, dma_memory, enable_interrupts, init_clocks, Buffer, Error, Instance, InterruptHandler,
    LinkInterruptHandler, LinkState, EP0_SIZE, LMP_PORT_CAP, LMP_PORT_CFG_RES, LMP_SUBTYPE_MASK, MODE_HOST,
};
use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Binding;
use crate::pac::usbss::vals::PdMode;
use crate::Peri;

/// Largest SuperSpeed data packet.
const MAX_PACKET: usize = 1024;
/// Most packets one IN arm can take, the width of `HOST_RX_NUMP`.
const MAX_RX_PACKETS: usize = 255;
/// `UH_TX_CTRL`/`UH_RX_CTRL.RES` value that arms a transaction.
const ARM: u8 = 1;
/// Undocumented `UH_TX_CTRL` bit, set on SETUP and data packets.
const TX_NUMP_1: u32 = 1 << 24;
/// `STATUS.HTX_RES`/`HRX_RES`; 1 is a failed transaction.
const ACK: u8 = 0;
const NRDY: u8 = 2;
const STALL: u8 = 3;
/// Failed transactions are retried this many times.
const RETRIES: u32 = 8;
/// Warm Reset signalling time.
const WARM_RESET: Duration = Duration::from_millis(80);
/// How long the link may take to train again after a Warm Reset.
const RETRAIN: Duration = Duration::from_secs(1);
/// Delay before a Status stage, which fails otherwise.
const STATUS_DELAY: Duration = Duration::from_millis(1);

pub(super) struct State {
    regs: Regs,
    /// Held while a transaction is armed.
    engine: Mutex<CriticalSectionRawMutex, ()>,
    done: AtomicWaker,
    /// ERDYs not consumed yet, bit `2 * endpoint + IN`.
    erdy: AtomicU32,
    erdy_wakers: [AtomicWaker; 32],
    /// The device finished the Port Configuration handshake.
    connected: AtomicBool,
    /// Advances when the link goes down or the port resets, which fails the
    /// pipes allocated before.
    generation: AtomicU32,
    port_waker: AtomicWaker,
    /// Packet buffer for the transfers the controller cannot DMA directly.
    bounce: AtomicUsize,
}

impl State {
    pub const fn new(regs: Regs) -> Self {
        Self {
            regs,
            engine: Mutex::new(()),
            done: AtomicWaker::new(),
            erdy: AtomicU32::new(0),
            erdy_wakers: [const { AtomicWaker::new() }; 32],
            connected: AtomicBool::new(false),
            generation: AtomicU32::new(0),
            port_waker: AtomicWaker::new(),
            bounce: AtomicUsize::new(0),
        }
    }

    fn bounce(&self) -> Buffer {
        Buffer {
            address: self.bounce.load(Ordering::Relaxed),
        }
    }

    /// Fails every pipe and wakes everything that waits on the port.
    fn port_lost(&self) {
        self.connected.store(false, Ordering::Release);
        self.generation.fetch_add(1, Ordering::AcqRel);
        self.done.wake();
        wake_all(&self.erdy_wakers);
        self.port_waker.wake();
    }

    /// Stops the transaction engine and drops its completion.
    fn disarm(&self, incoming: bool) {
        let h = self.regs.host();
        if incoming {
            h.uh_rx_ctrl().write(|_| {});
        } else {
            h.uh_tx_ctrl().write(|_| {});
        }
        h.status().write(|w| w.set_uif_transfer(true));
    }

    /// Initializes the block as a SuperSpeed host, also on a warm reset.
    fn init(&self) {
        self.port_lost();
        self.erdy.store(0, Ordering::Relaxed);

        let h = self.regs.host();
        h.usbss_ctrl().modify(|w| w.set_usb_clr_all(true));
        h.usbss_ctrl().modify(|w| w.set_usb_clr_all(false));
        h.uh_tx_ctrl().write(|_| {});
        h.uh_rx_ctrl().write(|_| {});
        self.regs.init(Role::Host);
        h.usbss_ctrl().modify(|w| w.set_force_rst(true));

        // Also releases FORCE_RST.
        h.usbss_ctrl().write(|w| {
            w.set_host_mode(true);
            w.set_itp_en(true);
            w.set_dma_en(true);
            w.set_reg_hp_pend(2);
            w.set_uhie_erdy(true);
        });
        h.uh_tx_ctrl().write(|_| {});
        h.status().write(|w| {
            w.set_uif_transfer(true);
            w.set_uhif_erdy(true);
        });

        // Host mode uses the device view's endpoint enables.
        self.regs.uep_tx_en().write(|w| w.set_ep_tx_en(0, true));
        self.regs.uep_rx_en().write(|w| w.set_ep_rx_en(0, true));
    }

    /// GLOBAL interrupt in host mode: records ERDYs and wakes the task of the
    /// completed transaction, which acknowledges it.
    pub(super) fn on_interrupt(&self) {
        let h = self.regs.host();
        let status = h.status().read();

        if status.uhif_erdy() {
            let host_status = h.host_status().read();
            h.status().write(|w| w.set_uhif_erdy(true));
            let bit = erdy_bit(host_status.uh_rx_erdy_ep(), host_status.uh_rx_erdy_dir());
            self.erdy.fetch_or(1 << bit, Ordering::AcqRel);
            self.erdy_wakers[bit].wake();
        }

        if status.uif_transfer() && h.usbss_ctrl().read().uie_transfer() {
            // Masked until the task acknowledges it, or it fires again.
            h.usbss_ctrl().modify(|w| w.set_uie_transfer(false));
            self.done.wake();
        }
    }

    /// LINK interrupt in host mode. A downstream port sends PORT_CAP and
    /// answers the device's PORT_CAP with PORT_CFG; PORT_CFG_RES means
    /// training completed.
    pub(super) fn on_link_interrupt(&self) {
        let r = self.regs;
        let flags = r.link_int_flag().read();

        if flags.link_if_state_chg() {
            r.link_int_flag().write(|w| w.set_link_if_state_chg(true));
            match r.link_status().read().link_state() {
                // Release the forced state so the LTSSM re-detects.
                LinkState::DISABLED => r.link_ctrl().modify(|w| w.set_link_go_disabled(false)),
                LinkState::RXDET | LinkState::INACTIVE => self.port_lost(),
                LinkState::HOTRST => r.end_hot_reset(),
                _ => {}
            }
        }

        if flags.link_if_tx_lmp() {
            r.link_int_flag().write(|w| w.set_link_if_tx_lmp(true));
            r.tx_port_cap();
        }

        if flags.link_if_rx_lmp() {
            r.link_int_flag().write(|w| w.set_link_if_rx_lmp(true));
            match r.link_lmp_rx_data(0).read() & LMP_SUBTYPE_MASK {
                LMP_PORT_CAP => r.tx_port_cfg(),
                LMP_PORT_CFG_RES => {
                    r.port_configured();
                    self.connected.store(true, Ordering::Release);
                }
                _ => {}
            }
        }

        if flags.link_if_rx_lmp_tout() {
            r.link_int_flag().write(|w| w.set_link_if_rx_lmp_tout(true));
            r.redetect();
        }

        if flags.link_if_warm_rst() {
            r.link_int_flag().write(|w| w.set_link_if_warm_rst(true));
            if r.link_status().read().link_rx_warm_rst() {
                self.init();
            }
        }

        self.port_waker.wake();
    }
}

fn state<T: Instance>() -> &'static State {
    &T::state().host
}

fn erdy_bit(endpoint: u8, incoming: bool) -> usize {
    usize::from(endpoint) * 2 + usize::from(incoming)
}

/// Hides the length, or LLVM unrolls the loop into 4 KiB of wakes.
fn wake_all(wakers: &[AtomicWaker]) {
    for waker in core::hint::black_box(wakers) {
        waker.wake();
    }
}

/// Root-port link status snapshot (`LINK_STATUS`, RM §27.2.6.5).
#[derive(Clone, Copy)]
pub struct LinkStatus {
    pub state: LinkState,
    /// `LINK_STATUS.LINK_RX_TERM_PRES`: receiver termination detected.
    pub term_present: bool,
    /// `LINK_STATUS.LINK_RX_DETECT`: the PHY is in P2 and running RxDetect.
    pub rx_detect: bool,
    /// `LINK_STATUS.LINK_READY`: the link finished initialization in U0.
    pub ready: bool,
    /// `LINK_STATUS.LINK_BUSY`: a `PD_MODE` change is still in progress.
    pub busy: bool,
    /// `LINK_STATUS.LINK_PD_MODE_MASK`: PHY power state.
    pub pd_mode: PdMode,
}

/// SuperSpeed host driver.
pub struct HostDriver<'d, T: Instance> {
    _peri: Peri<'d, T>,
    /// A `Connected` event was reported and no `Disconnected` since.
    attached: bool,
    /// The generation the attached device trained in.
    generation: u32,
}

impl<'d, T: Instance> HostDriver<'d, T> {
    /// `memory` needs to be 1024 bytes aligned to 4 bytes, inside the
    /// controller's DMA range `0x200A_0000..0x2018_0000`. Control transfers
    /// go through it, and so does every packet whose buffer is outside that
    /// range or has room for less than 1024 bytes.
    pub fn new(
        peri: Peri<'d, T>,
        _irqs: impl Binding<T::Interrupt, InterruptHandler<T>> + Binding<T::LinkInterrupt, LinkInterruptHandler<T>> + 'd,
        memory: &'d mut [u8],
    ) -> Result<Self, Error> {
        let state = state::<T>();
        state.bounce.store(dma_memory(memory, MAX_PACKET)?, Ordering::Relaxed);
        T::state().mode.store(MODE_HOST, Ordering::Relaxed);
        unsafe { init_clocks::<T>() };
        state.init();
        unsafe { enable_interrupts::<T>() };

        Ok(Self {
            _peri: peri,
            attached: false,
            generation: 0,
        })
    }

    /// Root-port link status.
    pub fn status(&self) -> LinkStatus {
        let status = state::<T>().regs.link_status().read();
        LinkStatus {
            state: status.link_state(),
            term_present: status.link_rx_term_pres(),
            rx_detect: status.link_rx_detect(),
            ready: status.link_ready(),
            busy: status.link_busy(),
            pd_mode: status.link_pd_mode_mask(),
        }
    }

    /// Warm-resets the port and waits for the link to train again. Returns
    /// whether it did.
    async fn reset(&mut self) -> bool {
        let state = state::<T>();
        let h = state.regs.host();
        state.port_lost();

        h.link_ctrl().write(|w| w.set_link_pd_mode(PdMode::P2));
        h.link_ctrl().modify(|w| w.set_link_tx_warm_rst(true));
        let signalling = OnDrop::new(|| h.link_ctrl().modify(|w| w.set_link_tx_warm_rst(false)));
        Timer::after(WARM_RESET).await;
        drop(signalling);

        let trained = poll_fn(|cx| {
            state.port_waker.register(cx.waker());
            if state.connected.load(Ordering::Acquire) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        });
        if with_timeout(RETRAIN, trained).await.is_err() {
            return false;
        }
        critical_section::with(|_| h.usbss_ctrl().modify(|w| w.set_dev_addr(0)));
        self.generation = state.generation.load(Ordering::Acquire);
        true
    }
}

impl<'d, T: Instance> UsbHostController<'d> for HostDriver<'d, T> {
    type Allocator = Allocator<'d, T>;

    fn allocator(&self) -> Self::Allocator {
        Allocator { _phantom: PhantomData }
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        let state = state::<T>();
        if self.attached {
            poll_fn(|cx| {
                state.port_waker.register(cx.waker());
                if state.generation.load(Ordering::Acquire) != self.generation {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
            self.attached = false;
            return DeviceEvent::Disconnected;
        }

        loop {
            // A link in SS.Inactive leaves it only through a Warm Reset.
            poll_fn(|cx| {
                state.port_waker.register(cx.waker());
                let inactive = state.regs.link_status().read().link_state() == LinkState::INACTIVE;
                if state.connected.load(Ordering::Acquire) || inactive {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
            if self.reset().await {
                self.attached = true;
                return DeviceEvent::Connected(Speed::Super);
            }
        }
    }

    async fn bus_reset(&mut self) {
        // A port that does not train again reports `Disconnected` next.
        self.reset().await;
    }
}

/// Pipe allocator of [`HostDriver`]. Pipes hold no controller resources.
pub struct Allocator<'d, T: Instance> {
    _phantom: PhantomData<&'d T>,
}

impl<'d, T: Instance> Clone for Allocator<'d, T> {
    fn clone(&self) -> Self {
        Self { _phantom: PhantomData }
    }
}

impl<'d, T: Instance> UsbHostAllocator<'d> for Allocator<'d, T> {
    type Pipe<E: pipe::Type, D: pipe::Direction> = Pipe<'d, T, E, D>;

    fn alloc_pipe<E: pipe::Type, D: pipe::Direction>(
        &self,
        address: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<E, D>, HostError> {
        if split.is_some() {
            return Err(HostError::Other("USBSS has no split transactions"));
        }
        if endpoint.companion.is_some_and(|companion| companion.max_streams != 0) {
            return Err(HostError::Other("USBSS host has no streams"));
        }
        let max_packet_size = match E::ep_type() {
            EndpointType::Control => EP0_SIZE,
            EndpointType::Bulk | EndpointType::Interrupt => usize::from(endpoint.max_packet_size),
            EndpointType::Isochronous => return Err(HostError::Other("USBSS host has no isochronous pipes")),
        };
        if !(1..=MAX_PACKET).contains(&max_packet_size) {
            return Err(HostError::InvalidDescriptor);
        }
        let state = state::<T>();
        Ok(Pipe {
            channel: Channel {
                state,
                address,
                endpoint: endpoint.addr.index() as u8,
                max_packet_size,
                seq: 0,
                generation: state.generation.load(Ordering::Acquire),
            },
            timeout: TimeoutConfig::default(),
            _phantom: PhantomData,
        })
    }
}

#[derive(Clone, Copy)]
enum Packet<'a> {
    Setup(&'a [u8; 8]),
    Data(&'a [u8]),
    Status,
}

/// A pipe of [`HostDriver`].
pub struct Pipe<'d, T: Instance, E, D> {
    channel: Channel,
    timeout: TimeoutConfig,
    _phantom: PhantomData<(&'d T, E, D)>,
}

/// The endpoint state of a pipe. Not generic, so the transfer code exists
/// once.
struct Channel {
    state: &'static State,
    address: u8,
    endpoint: u8,
    max_packet_size: usize,
    /// Sequence number of the next data packet.
    seq: u8,
    generation: u32,
}

impl Channel {
    fn check(&self) -> Result<(), PipeError> {
        let state = self.state;
        if state.connected.load(Ordering::Acquire) && state.generation.load(Ordering::Acquire) == self.generation {
            Ok(())
        } else {
            Err(PipeError::Disconnected)
        }
    }

    /// Points the controller at the pipe's device.
    fn select(&self) {
        let h = self.state.regs.host();
        if h.usbss_ctrl().read().dev_addr() != self.address {
            // The interrupt handler also modifies `USBSS_CTRL`.
            critical_section::with(|_| h.usbss_ctrl().modify(|w| w.set_dev_addr(self.address)));
        }
    }

    /// The ERDY bits of the endpoint. Those of EP0 count in either direction.
    fn erdy_mask(&self, incoming: bool) -> u32 {
        if self.endpoint == 0 {
            0b11
        } else {
            1 << erdy_bit(self.endpoint, incoming)
        }
    }

    async fn wait_done(&self, incoming: bool) -> Result<(), PipeError> {
        poll_fn(|cx| {
            let h = self.state.regs.host();
            self.state.done.register(cx.waker());
            let done = if incoming {
                h.uh_rx_ctrl().read().uh_rx_act()
            } else {
                h.uh_tx_ctrl().read().uh_tx_act()
            };
            if done {
                return Poll::Ready(Ok(()));
            }
            if let Err(e) = self.check() {
                return Poll::Ready(Err(e));
            }
            critical_section::with(|_| h.usbss_ctrl().modify(|w| w.set_uie_transfer(true)));
            Poll::Pending
        })
        .await
    }

    async fn wait_erdy(&self, incoming: bool) -> Result<(), PipeError> {
        let state = self.state;
        let mask = self.erdy_mask(incoming);
        poll_fn(|cx| {
            for (bit, waker) in state.erdy_wakers.iter().enumerate() {
                if mask & (1 << bit) != 0 {
                    waker.register(cx.waker());
                }
            }
            if state.erdy.fetch_and(!mask, Ordering::AcqRel) & mask != 0 {
                return Poll::Ready(Ok(()));
            }
            if let Err(e) = self.check() {
                return Poll::Ready(Err(e));
            }
            Poll::Pending
        })
        .await
    }

    /// Runs one OUT transaction. Returns `HTX_RES` and the `NumP` of the
    /// device's ACK.
    async fn transmit(&self, packet: Packet<'_>) -> Result<(u8, u8), PipeError> {
        let state = self.state;
        let h = state.regs.host();
        let _engine = state.engine.lock().await;
        self.check()?;
        self.select();
        state.erdy.fetch_and(!self.erdy_mask(false), Ordering::AcqRel);

        let data: &[u8] = match packet {
            Packet::Setup(setup) => setup,
            Packet::Data(data) => data,
            Packet::Status => &[],
        };
        let address = if dma_capable(data) {
            data.as_ptr() as usize
        } else {
            let bounce = state.bounce();
            unsafe { bounce.write(data) };
            bounce.address
        };
        fence(Ordering::Release);
        h.uh_tx_dma().write(|w| w.set_uh_tx_dma(address as u32));
        h.host_tx_nump().write(|w| w.set_uh_tx_nump(1));
        h.uh_tx_ctrl().write(|w| {
            match packet {
                Packet::Setup(_) => {
                    w.0 |= TX_NUMP_1;
                    w.set_uh_tx_setup(true);
                }
                Packet::Data(_) => {
                    w.0 |= TX_NUMP_1;
                    w.set_uh_tx_ep(self.endpoint);
                    w.set_uh_tx_seq(self.seq);
                }
                Packet::Status => w.set_uh_tx_status(true),
            }
            w.set_uh_tx_len(data.len() as u16);
        });
        let _armed = OnDrop::new(|| state.disarm(false));
        h.uh_tx_ctrl().modify(|w| w.set_uh_tx_res(ARM));

        self.wait_done(false).await?;
        let status = h.status().read();
        Ok((status.htx_res(), status.host_ack_nump()))
    }

    /// Receives into `buf`. The engine asks for one packet after another
    /// until `buf` is full, a short packet arrives or the device answers
    /// otherwise than with data. Returns that answer, the packets received
    /// and their length.
    async fn receive(&self, buf: &mut [u8]) -> Result<(u8, usize, usize), PipeError> {
        let state = self.state;
        let h = state.regs.host();
        let _engine = state.engine.lock().await;
        self.check()?;
        self.select();
        state.erdy.fetch_and(!self.erdy_mask(true), Ordering::AcqRel);

        // Packets land 1024 bytes apart, so each needs that much room,
        // whatever the device sends, and only full-size ones are contiguous.
        let limit = if self.max_packet_size == MAX_PACKET { MAX_RX_PACKETS } else { 1 };
        let packets = (buf.len() / MAX_PACKET).min(limit);
        let direct = packets > 0 && dma_capable(&buf[..packets * MAX_PACKET]);
        let (address, packets) = if direct {
            (buf.as_mut_ptr() as usize, packets)
        } else {
            (state.bounce().address, 1)
        };
        h.uh_rx_dma().write(|w| w.set_uh_rx_dma(address as u32));
        h.host_rx_nump().write(|w| w.set_uh_rx_nump(packets as u8));
        h.uh_rx_ctrl().write(|w| {
            w.set_uh_rx_ep(self.endpoint);
            w.set_uh_rx_seq(self.seq);
            w.set_uh_rx_nump(1);
        });
        let _armed = OnDrop::new(|| state.disarm(true));
        h.uh_rx_ctrl().modify(|w| w.set_uh_rx_res(ARM));

        self.wait_done(true).await?;
        let result = h.status().read().hrx_res();
        let received = usize::from(h.host_rx_nump().read().uh_rx_dpp_num());
        let last = usize::from(h.uh_rx_ctrl().read().uh_rx_len());
        fence(Ordering::Acquire);
        if received == 0 {
            return Ok((result, 0, 0));
        }
        if received > packets || last > self.max_packet_size {
            return Err(PipeError::Babble);
        }
        let length = (received - 1) * MAX_PACKET + last;
        if !direct {
            if length > buf.len() {
                return Err(PipeError::BufferOverflow);
            }
            unsafe { state.bounce().read(&mut buf[..length]) };
        }
        Ok((result, received, length))
    }

    /// Sends one packet once the device takes it. Returns the `NumP` of its
    /// ACK.
    async fn send(&self, packet: Packet<'_>) -> Result<u8, PipeError> {
        let mut failures = 0;
        loop {
            match self.transmit(packet).await? {
                (ACK, nump) => return Ok(nump),
                (NRDY, _) => self.wait_erdy(false).await?,
                (STALL, _) => return Err(PipeError::Stall),
                _ => {
                    failures += 1;
                    if failures > RETRIES {
                        return Err(PipeError::BadResponse);
                    }
                }
            }
        }
    }

    /// Receives data packets until a short one or until `buf` is full. An
    /// NRDY can come after some packets, which stay received.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, PipeError> {
        let mut total = 0;
        let mut failures = 0;
        loop {
            let (result, packets, length) = self.receive(&mut buf[total..]).await?;
            self.seq = ((usize::from(self.seq) + packets) % 32) as u8;
            total += length;
            if packets > 0 && (length < packets * self.max_packet_size || total == buf.len()) {
                return Ok(total);
            }
            if packets > 0 {
                failures = 0;
            }
            match result {
                ACK if packets > 0 => {}
                NRDY => self.wait_erdy(true).await?,
                STALL => return Err(PipeError::Stall),
                _ => {
                    failures += 1;
                    if failures > RETRIES {
                        return Err(PipeError::BadResponse);
                    }
                }
            }
        }
    }

    /// Sends `data` as data packets, ending with a zero-length one if `zlp`
    /// and the last one is full.
    async fn write(&mut self, data: &[u8], zlp: bool) -> Result<(), PipeError> {
        let mut offset = 0;
        loop {
            let chunk = &data[offset..data.len().min(offset + self.max_packet_size)];
            let nump = self.send(Packet::Data(chunk)).await?;
            self.seq = (self.seq + 1) % 32;
            offset += chunk.len();
            if offset == data.len() && !(zlp && chunk.len() == self.max_packet_size) {
                return Ok(());
            }
            if nump == 0 {
                self.wait_erdy(false).await?;
            }
        }
    }

    async fn setup_stage(&mut self, setup: &[u8; 8]) -> Result<(), PipeError> {
        self.seq = 0;
        if self.send(Packet::Setup(setup)).await? == 0 {
            self.wait_erdy(false).await?;
        }
        Ok(())
    }

    async fn status_stage(&self) -> Result<(), PipeError> {
        Timer::after(STATUS_DELAY).await;
        self.send(Packet::Status).await.map(|_| ())
    }
}

impl<'d, T: Instance, E, D> Pipe<'d, T, E, D> {
    fn control_timeout(&self, length: usize) -> Duration {
        let timeout = if length > 0 {
            self.timeout.data_timeout
        } else {
            self.timeout.no_data_timeout
        };
        Duration::from_micros(timeout.as_micros() as u64)
    }
}

impl<'d, T: Instance, E: pipe::Type, D: pipe::Direction> UsbPipe<E, D> for Pipe<'d, T, E, D> {
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        E: pipe::IsControl,
        D: pipe::IsIn,
    {
        let length = usize::from(u16::from_le_bytes([setup[6], setup[7]]));
        let buf = buf.get_mut(..length).ok_or(PipeError::BufferOverflow)?;
        let timeout = self.control_timeout(length);
        let channel = &mut self.channel;
        let transfer = async {
            channel.setup_stage(setup).await?;
            let read = if length > 0 { channel.read(buf).await? } else { 0 };
            channel.status_stage().await?;
            Ok(read)
        };
        with_timeout(timeout, transfer).await.unwrap_or(Err(PipeError::Timeout))
    }

    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        E: pipe::IsControl,
        D: pipe::IsOut,
    {
        let length = usize::from(u16::from_le_bytes([setup[6], setup[7]]));
        let data = buf.get(..length).ok_or(PipeError::BufferOverflow)?;
        let timeout = self.control_timeout(length);
        let channel = &mut self.channel;
        let transfer = async {
            channel.setup_stage(setup).await?;
            if length > 0 {
                channel.write(data, false).await?;
            }
            channel.status_stage().await
        };
        with_timeout(timeout, transfer).await.unwrap_or(Err(PipeError::Timeout))
    }

    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        self.channel.read(buf).await
    }

    async fn request_out(&mut self, buf: &[u8], ensure_transaction_end: bool) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        self.channel.write(buf, ensure_transaction_end).await
    }

    fn set_timeout(&mut self, timeout: TimeoutConfig)
    where
        E: pipe::IsControl,
    {
        self.timeout = timeout;
    }

    fn reset_data_toggle(&mut self)
    where
        E: pipe::IsBulkOrInterrupt,
    {
        self.channel.seq = 0;
    }
}
