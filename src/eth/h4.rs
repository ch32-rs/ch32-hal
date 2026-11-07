//! Ethernet for the CH32H417: gigabit MAC, integrated 10M/100M PHY and RGMII.
//!
//! The internal PHY needs the 500 MHz ETH PLL (`rcc::Config::aux_plls.eth`)
//! and no GPIO. A RGMII PHY additionally needs `rcc::Config::rgmii_on`
//! (`CFGR0.RGMIION`) and `KernelMux::eth1gsrc = Eth1gsrc::ETH_PLL_DIV4` for
//! the 125 MHz clock; [`Ethernet::new_rgmii`] refuses to start without them.
//!
//! DMA resources must be static; RX and TX tokens temporarily grant the CPU
//! ownership of a single frame. Hardware checksums and IEEE 1588 are off.
//!
//! TODO(hw): the RGMII/1G paths follow RM §31 and the EVT driver but have not
//! run on hardware. The nanoCH32H417 only wires the internal 10/100M PHY.

use core::marker::PhantomData;
use core::sync::atomic::{fence, AtomicBool, AtomicU32, Ordering};

use embassy_net_driver::{Capabilities, HardwareAddress};
use embassy_sync::waitqueue::AtomicWaker;

use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::peripheral::{RccPeripheral, SealedRccPeripheral};
use crate::{interrupt, pac, peripherals, Peri};

#[path = "h4_phy.rs"]
mod phy;
#[path = "h4_ring.rs"]
mod ring;

pub use phy::{GenericPhy, InternalPhy, Link, LinkSpeed, Phy, Sma};
pub use ring::{Resources, RxToken, TxToken};

use self::phy::set_pad_speed;

/// `DMASR.PLS`: the integrated PHY reports a link change.
const LINK_INTERRUPT: u32 = 1 << 11;
const IRQ_FLAGS: u32 = 0x0001_cfff;
const INTERRUPTS: u32 = 0x0001_8bfb;
const DATA_ERRORS: u32 = (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 8) | (1 << 9);

/// Poll limit for resets and DMA state changes.
const POLL_LIMIT: u32 = 1_000_000;

static WAKER: AtomicWaker = AtomicWaker::new();
static ACTIVE: AtomicBool = AtomicBool::new(false);
static PENDING: AtomicU32 = AtomicU32::new(0);

/// Driver errors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    InvalidConfig,
    BadClock,
    ResetTimeout,
    SmiTimeout,
    PhyNotFound,
    /// RGMII was requested without `CFGR0.RGMIION` or with the wrong
    /// `CFG2.ETH1GSRC` clock source.
    RgmiiDisabled,
    /// The PHY negotiated 1000BASE-T half duplex; RGMII has no half duplex.
    UnsupportedLink,
    DmaStopped,
}

/// MAC-side RGMII timing.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RgmiiConfig {
    /// `MACCR.TCD`, 0.5 ns per step, `0..=7`. The EVT uses 4.
    ///
    /// The TX clock is also inverted at 1000M, as the EVT does; the polarity
    /// is not configurable because both settings come as a pair.
    pub txc_delay: u8,
}

impl Default for RgmiiConfig {
    fn default() -> Self {
        Self { txc_delay: 4 }
    }
}

/// The twelve RGMII signal pins, with their alternate functions applied.
pub struct RgmiiPins<'d> {
    _pins: [Peri<'d, crate::gpio::AnyPin>; 12],
}

impl<'d> RgmiiPins<'d> {
    /// Configure and take ownership of the RGMII pins.
    ///
    /// Transmit pins are push-pull outputs, receive pins float (RM table 31-1).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        gtxc: Peri<'d, impl GtxcPin<peripherals::ETH>>,
        txen: Peri<'d, impl TxEnPin<peripherals::ETH>>,
        txd0: Peri<'d, impl Txd0Pin<peripherals::ETH>>,
        txd1: Peri<'d, impl Txd1Pin<peripherals::ETH>>,
        txd2: Peri<'d, impl Txd2Pin<peripherals::ETH>>,
        txd3: Peri<'d, impl Txd3Pin<peripherals::ETH>>,
        rxc: Peri<'d, impl RxcPin<peripherals::ETH>>,
        rxdv: Peri<'d, impl RxdvPin<peripherals::ETH>>,
        rd0: Peri<'d, impl Rd0Pin<peripherals::ETH>>,
        rd1: Peri<'d, impl Rd1Pin<peripherals::ETH>>,
        rd2: Peri<'d, impl Rd2Pin<peripherals::ETH>>,
        rd3: Peri<'d, impl Rd3Pin<peripherals::ETH>>,
    ) -> Self {
        let output = crate::gpio::AfType::output(crate::gpio::OutputType::PushPull, crate::gpio::Speed::High);
        let input = crate::gpio::AfType::input(crate::gpio::Pull::None);

        set_as_af!(gtxc, output);
        set_as_af!(txen, output);
        set_as_af!(txd0, output);
        set_as_af!(txd1, output);
        set_as_af!(txd2, output);
        set_as_af!(txd3, output);
        set_as_af!(rxc, input);
        set_as_af!(rxdv, input);
        set_as_af!(rd0, input);
        set_as_af!(rd1, input);
        set_as_af!(rd2, input);
        set_as_af!(rd3, input);

        set_pad_speed(&*gtxc);
        set_pad_speed(&*txen);
        set_pad_speed(&*txd0);
        set_pad_speed(&*txd1);
        set_pad_speed(&*txd2);
        set_pad_speed(&*txd3);

        Self {
            _pins: [
                gtxc.into(),
                txen.into(),
                txd0.into(),
                txd1.into(),
                txd2.into(),
                txd3.into(),
                rxc.into(),
                rxdv.into(),
                rd0.into(),
                rd1.into(),
                rd2.into(),
                rd3.into(),
            ],
        }
    }
}

/// Ethernet instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccPeripheral + 'static {}

trait SealedInstance {}

impl SealedInstance for peripherals::ETH {}
impl Instance for peripherals::ETH {}

pin_trait!(GtxcPin, Instance);
pin_trait!(TxEnPin, Instance);
pin_trait!(Txd0Pin, Instance);
pin_trait!(Txd1Pin, Instance);
pin_trait!(Txd2Pin, Instance);
pin_trait!(Txd3Pin, Instance);
pin_trait!(RxcPin, Instance);
pin_trait!(RxdvPin, Instance);
pin_trait!(Rd0Pin, Instance);
pin_trait!(Rd1Pin, Instance);
pin_trait!(Rd2Pin, Instance);
pin_trait!(Rd3Pin, Instance);
pin_trait!(MdcPin, Instance);
pin_trait!(MdioPin, Instance);

fn eth() -> pac::emac::Eth {
    pac::ETH
}

fn wait(limit: u32, mut done: impl FnMut() -> bool, error: Error) -> Result<(), Error> {
    for _ in 0..limit {
        if done() {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(error)
}

fn stop() {
    critical_section::with(|_| {
        ACTIVE.store(false, Ordering::Release);
        interrupt::typelevel::ETH::disable();
        if !pac::RCC.hbpcenr().read().ethmacen() {
            interrupt::typelevel::ETH::unpend();
            return;
        }
        eth().dmaier().write_value(pac::emac::regs::Dmaier(0));
        eth().dmaomr().write_value(pac::emac::regs::Dmaomr(0));
        eth().maccr().write_value(pac::emac::regs::Maccr(0));
        // Power the integrated PHY down while the MAC is reset; the PHY has no
        // separate clock gate.
        eth().phy_cfgr().write_value(pac::emac::regs::PhyCfgr(1 << 30));
        peripherals::ETH::enable_and_reset();
        fence(Ordering::SeqCst);
        peripherals::ETH::disable();
        interrupt::typelevel::ETH::unpend();
    });
}

fn resume_receive() {
    eth().dmasr().write_value(pac::emac::regs::Dmasr(1 << 7));
    eth().dmarpdr().write_value(pac::emac::regs::Dmarpdr(0));
}

fn resume_transmit() {
    eth().dmasr().write_value(pac::emac::regs::Dmasr((1 << 2) | (1 << 5)));
    eth().dmatpdr().write_value(pac::emac::regs::Dmatpdr(0));
}

/// Interrupt handler.
pub struct InterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::ETH> for InterruptHandler {
    unsafe fn on_interrupt() {
        if !ACTIVE.load(Ordering::Acquire) {
            return;
        }
        critical_section::with(|_| {
            let status = eth().dmasr().read().0 & IRQ_FLAGS;
            PENDING.store(PENDING.load(Ordering::Relaxed) | status, Ordering::Release);
            if status & LINK_INTERRUPT != 0 {
                eth().dmaier().modify(|register| register.set_ple(false));
            }
            eth().dmasr().write_value(pac::emac::regs::Dmasr(status));
        });
        WAKER.wake();
    }
}

#[derive(Clone, Copy)]
enum MacMode {
    Internal,
    Rgmii { delay: u8 },
}

/// Ethernet driver.
pub struct Ethernet<'d, P: Phy> {
    _peri: Peri<'d, peripherals::ETH>,
    phy: P,
    _pins: Option<RgmiiPins<'d>>,
    tx: ring::Ring,
    rx: ring::Ring,
    mac_addr: [u8; 6],
    mode: MacMode,
    link: Link,
    faulted: bool,
    last_error: Option<Error>,
    dma_errors: u32,
    _not_sync: PhantomData<core::cell::Cell<()>>,
}

impl<'d> Ethernet<'d, InternalPhy> {
    /// Create a driver for the integrated 10M/100M PHY.
    pub fn new_internal<const TX: usize, const RX: usize>(
        peri: Peri<'d, peripherals::ETH>,
        irq: impl interrupt::typelevel::Binding<interrupt::typelevel::ETH, InterruptHandler> + 'd,
        resources: &'static mut Resources<TX, RX>,
        mac_addr: [u8; 6],
        phy: InternalPhy,
    ) -> Result<Self, Error> {
        Self::new_impl(peri, irq, resources, mac_addr, phy, MacMode::Internal, None)
    }
}

impl<'d, P: Phy> Ethernet<'d, P> {
    /// Create a driver for an external PHY connected over RGMII.
    pub fn new_rgmii<const TX: usize, const RX: usize>(
        peri: Peri<'d, peripherals::ETH>,
        irq: impl interrupt::typelevel::Binding<interrupt::typelevel::ETH, InterruptHandler> + 'd,
        resources: &'static mut Resources<TX, RX>,
        mac_addr: [u8; 6],
        pins: RgmiiPins<'d>,
        phy: P,
        rgmii: RgmiiConfig,
    ) -> Result<Self, Error> {
        if rgmii.txc_delay > 7 {
            return Err(Error::InvalidConfig);
        }
        let rcc = pac::RCC;
        if !rcc.cfgr0().read().rgmiion() || rcc.cfgr2().read().eth1gsrc() != pac::rcc::vals::Eth1gsrc::ETH_PLL_DIV4 {
            return Err(Error::RgmiiDisabled);
        }
        Self::new_impl(
            peri,
            irq,
            resources,
            mac_addr,
            phy,
            MacMode::Rgmii { delay: rgmii.txc_delay },
            Some(pins),
        )
    }

    fn new_impl<const TX: usize, const RX: usize>(
        peri: Peri<'d, peripherals::ETH>,
        _irq: impl interrupt::typelevel::Binding<interrupt::typelevel::ETH, InterruptHandler> + 'd,
        resources: &'static mut Resources<TX, RX>,
        mac_addr: [u8; 6],
        phy: P,
        mode: MacMode,
        pins: Option<RgmiiPins<'d>>,
    ) -> Result<Self, Error> {
        if TX == 0 || RX == 0 || mac_addr[0] & 1 != 0 || mac_addr == [0; 6] {
            return Err(Error::InvalidConfig);
        }
        if !pac::RCC.ctlr().read().eth_pllrdy() {
            return Err(Error::BadClock);
        }
        let guard = OnDrop::new(stop);
        peripherals::ETH::enable_and_reset();
        let mut driver = Self {
            _peri: peri,
            phy,
            _pins: pins,
            tx: ring::Ring::new(&mut resources.tx_descriptors, &mut resources.tx_buffers, false),
            rx: ring::Ring::new(&mut resources.rx_descriptors, &mut resources.rx_buffers, true),
            mac_addr,
            mode,
            link: Link::Down,
            faulted: false,
            last_error: None,
            dma_errors: 0,
            _not_sync: PhantomData,
        };
        guard.defuse();
        driver.initialize()?;
        Ok(driver)
    }

    fn initialize(&mut self) -> Result<(), Error> {
        let eth = eth();
        eth.dmaier().write_value(pac::emac::regs::Dmaier(0));
        // The integrated PHY must be powered and out of reset before the DMA
        // soft reset: its RX/TX clock domain is part of what `DMABMR.SR`
        // waits for (the EVT releases `PHY_RSTN` before `ETH_SoftwareReset`).
        self.phy.reset()?;

        eth.dmabmr().write_value(pac::emac::regs::Dmabmr(1));
        wait(POLL_LIMIT, || !eth.dmabmr().read().sr(), Error::ResetTimeout)?;

        eth.maccr().write_value(pac::emac::regs::Maccr(0));
        eth.macffr().write(|register| register.set_pam(true));
        eth.macfcr().write_value(pac::emac::regs::Macfcr(0));
        // Mask the PMT and timestamp MAC interrupts; only the DMA interrupt
        // and the internal PHY's link interrupt are used.
        eth.macimr().write_value(pac::emac::regs::Macimr((1 << 3) | (1 << 9)));
        eth.mmcrimr().write(|register| {
            register.set_rfcem(true);
            register.set_rgufm(true);
        });
        eth.mmctimr().write(|register| register.set_tgfm(true));
        eth.maca0hr()
            .write(|register| register.set_maca0h(u16::from_le_bytes([self.mac_addr[4], self.mac_addr[5]])));
        eth.maca0lr()
            .write(|register| register.set_maca0l(u32::from_le_bytes(self.mac_addr[..4].try_into().unwrap())));

        self.tx.reset();
        self.rx.reset();
        eth.dmatdlar().write_value(pac::emac::regs::Dmatdlar(self.tx.address()));
        eth.dmardlar().write_value(pac::emac::regs::Dmardlar(self.rx.address()));
        // Store and forward: a descriptor is only handed back when the whole
        // frame is in the FIFO, so RX never sees a partial frame.
        eth.dmaomr()
            .write_value(pac::emac::regs::Dmaomr((1 << 21) | (1 << 20) | (1 << 13)));
        eth.dmasr().write_value(pac::emac::regs::Dmasr(IRQ_FLAGS));
        self.link = Link::Down;
        self.faulted = false;
        critical_section::with(|_| {
            PENDING.store(0, Ordering::Release);
            ACTIVE.store(true, Ordering::Release);
            interrupt::typelevel::ETH::unpend();
            eth.dmaier().write_value(pac::emac::regs::Dmaier(INTERRUPTS));
            unsafe { interrupt::typelevel::ETH::enable() };
        });
        self.refresh_link()?;
        Ok(())
    }

    /// Re-read the PHY link and reprogram the MAC if it changed.
    fn refresh_link(&mut self) -> Result<(), Error> {
        let Some(link) = self.phy.poll_link() else {
            return Ok(());
        };
        if let Link::Up {
            speed: LinkSpeed::Speed1000,
            full_duplex: false,
        } = link
        {
            return Err(Error::UnsupportedLink);
        }
        self.apply_link(link);
        self.link = link;
        Ok(())
    }

    fn apply_link(&mut self, link: Link) {
        let mode = self.mode;
        critical_section::with(|_| {
            let eth = eth();
            let enabled = eth.dmaier().read();
            eth.dmaier().write_value(pac::emac::regs::Dmaier(enabled.0 & !0x102));
            eth.maccr().modify(|register| {
                register.set_te(false);
                register.set_re(false);
            });
            match link {
                Link::Up { speed, full_duplex } => {
                    let fes = match speed {
                        LinkSpeed::Speed10 => 0,
                        LinkSpeed::Speed100 => 1,
                        LinkSpeed::Speed1000 => 2,
                    };
                    eth.maccr().modify(|register| {
                        register.set_dm(full_duplex);
                        register.set_fes(fes);
                        if let MacMode::Rgmii { delay } = mode {
                            // The EVT inverts GTXC only at gigabit.
                            register.set_tcf(matches!(speed, LinkSpeed::Speed1000));
                            register.set_tcd(delay);
                        }
                        register.set_te(true);
                        register.set_re(true);
                    });
                    eth.dmaomr().modify(|register| {
                        register.set_st(true);
                        register.set_sr(true);
                    });
                    resume_receive();
                    resume_transmit();
                }
                Link::Down => {
                    eth.dmaomr().modify(|register| {
                        register.set_st(false);
                        register.set_sr(false);
                    });
                }
            }
            eth.dmasr().write_value(pac::emac::regs::Dmasr(0x102));
            PENDING.store(PENDING.load(Ordering::Relaxed) & !0x102, Ordering::Release);
            eth.dmaier().write_value(enabled);
        });
    }

    fn fault(&mut self, error: Error) {
        self.last_error = Some(error);
        self.faulted = true;
        self.link = Link::Down;
        eth().dmaier().write_value(pac::emac::regs::Dmaier(0));
        eth().dmaomr().write_value(pac::emac::regs::Dmaomr(0));
        eth().maccr().write_value(pac::emac::regs::Maccr(0));
    }

    fn service(&mut self, context: &mut core::task::Context<'_>) -> bool {
        WAKER.register(context.waker());
        if self.faulted {
            return false;
        }
        let status = critical_section::with(|_| {
            let pending = PENDING.load(Ordering::Acquire);
            PENDING.store(0, Ordering::Release);
            let status = eth().dmasr().read().0 & IRQ_FLAGS;
            eth().dmasr().write_value(pac::emac::regs::Dmasr(status));
            pending | status
        });
        if status & DATA_ERRORS != 0 {
            self.dma_errors = self.dma_errors.saturating_add(1);
            self.last_error = Some(Error::DmaStopped);
            if status & ((1 << 1) | (1 << 8)) != 0 {
                self.fault(Error::DmaStopped);
                return false;
            }
        }
        if status & LINK_INTERRUPT != 0 {
            let result = self.phy.clear_interrupt().and_then(|()| self.refresh_link());
            if let Err(error) = result {
                self.fault(error);
                return false;
            }
            eth().dmaier().modify(|register| register.set_ple(true));
        }
        if status & (1 << 7) != 0 {
            resume_receive();
        }
        if status & ((1 << 2) | (1 << 5)) != 0 {
            resume_transmit();
        }
        self.link != Link::Down
    }

    /// The PHY, for link and ID queries.
    pub fn phy(&self) -> &P {
        &self.phy
    }

    /// The PHY, mutably.
    pub fn phy_mut(&mut self) -> &mut P {
        &mut self.phy
    }

    /// Last link state as seen by the driver.
    pub fn link(&self) -> Link {
        self.link
    }

    pub fn last_error(&self) -> Option<Error> {
        self.last_error
    }

    /// TX, RX and DMA error counters.
    pub fn error_counts(&self) -> (u32, u32, u32) {
        (self.tx.errors, self.rx.errors, self.dma_errors)
    }

    /// Reset the MAC and PHY and drop queued frames after a controller fault.
    pub fn recover(&mut self) -> Result<(), Error> {
        stop();
        peripherals::ETH::enable_and_reset();
        let result = self.initialize();
        if let Err(error) = result {
            stop();
            self.faulted = true;
            self.link = Link::Down;
            self.last_error = Some(error);
        }
        result
    }
}

impl<P: Phy> Drop for Ethernet<'_, P> {
    fn drop(&mut self) {
        stop();
    }
}

impl<P: Phy> embassy_net_driver::Driver for Ethernet<'_, P> {
    type RxToken<'a>
        = RxToken<'a>
    where
        Self: 'a;
    type TxToken<'a>
        = TxToken<'a>
    where
        Self: 'a;

    fn receive(&mut self, context: &mut core::task::Context<'_>) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if !self.service(context) || !self.tx.tx_ready() {
            return None;
        }
        let length = self.rx.receive_length()?;
        Some((
            RxToken {
                ring: &mut self.rx,
                length,
            },
            TxToken { ring: &mut self.tx },
        ))
    }

    fn transmit(&mut self, context: &mut core::task::Context<'_>) -> Option<Self::TxToken<'_>> {
        if !self.service(context) || !self.tx.tx_ready() {
            return None;
        }
        Some(TxToken { ring: &mut self.tx })
    }

    fn link_state(&mut self, context: &mut core::task::Context<'_>) -> embassy_net_driver::LinkState {
        self.service(context);
        if self.faulted || self.link == Link::Down {
            embassy_net_driver::LinkState::Down
        } else {
            embassy_net_driver::LinkState::Up
        }
    }

    fn capabilities(&self) -> Capabilities {
        let mut capabilities = Capabilities::default();
        capabilities.max_transmission_unit = ring::MTU;
        // smoltcp clamps the advertised TCP window to `max_burst_size` full
        // segments (`iface/packet.rs`), so this bounds the receive window, and
        // the ring depth is therefore a throughput knob: a value of 1 makes
        // every connection crawl at one segment per ACK, while 32 deep rings
        // take a 100BASE-TX link to its line rate.
        capabilities.max_burst_size = Some(self.tx.len().min(self.rx.len()));
        capabilities
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac_addr)
    }
}
