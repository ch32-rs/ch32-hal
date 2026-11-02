//! Ethernet for CH32V407/CH32V467, with the integrated 10/100M PHY.
//!
//! Configure the 25 MHz HSE and ETH PLL through RCC before construction.
//! The driver uses dedicated PHY pins and leaves the optional GPIO LEDs alone.
//! DMA resources must be static; RX and TX tokens temporarily grant CPU ownership
//! of a single frame. Hardware checksums and IEEE 1588 are disabled.

use core::marker::PhantomData;
use core::sync::atomic::{fence, AtomicBool, AtomicU32, Ordering};
use core::task::Context;

use embassy_net_driver::{Capabilities, HardwareAddress, LinkState};
use embassy_sync::waitqueue::AtomicWaker;

use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::peripheral::SealedRccPeripheral;
use crate::{interrupt, pac, peripherals, Peri};

#[path = "v4x7_ring.rs"]
mod ring;
pub use ring::{Resources, RxToken, TxToken};

const PHY_INTERRUPT: u32 = 1 << 11;
const IRQ_FLAGS: u32 = 0x0001_cfff;
const INTERRUPTS: u32 = 0x0001_8bfb;
const DATA_ERRORS: u32 = (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 8) | (1 << 9);
static WAKER: AtomicWaker = AtomicWaker::new();
static ACTIVE: AtomicBool = AtomicBool::new(false);
static PENDING: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    InvalidConfig,
    BadClock,
    ResetTimeout,
    SmiTimeout,
    PhyNotFound,
    DmaStopped,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub mac_address: [u8; 6],
    pub phy_address: u8,
    pub poll_limit: u32,
    pub phy_reset_attempts: u16,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mac_address: [2, 0, 0, 0, 0, 1],
            phy_address: 1,
            poll_limit: 1_000_000,
            phy_reset_attempts: 1_000,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Link {
    pub up: bool,
    pub speed_100m: bool,
    pub full_duplex: bool,
}

const DOWN: Link = Link {
    up: false,
    speed_100m: false,
    full_duplex: false,
};

fn clock_range(hz: u32) -> Result<u8, Error> {
    match hz {
        20_000_000..=34_999_999 => Ok(2),
        35_000_000..=59_999_999 => Ok(3),
        60_000_000..=89_999_999 => Ok(0),
        90_000_000..=119_999_999 => Ok(1),
        120_000_000..=149_999_999 => Ok(5),
        150_000_000..=200_000_000 => Ok(4),
        _ => Err(Error::BadClock),
    }
}

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
        eth().phy_cr().write_value(pac::emac::regs::PhyCr(1 << 30));
        <peripherals::ETH as crate::peripheral::SealedRccPeripheral>::enable_and_reset();
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

pub struct InterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::ETH> for InterruptHandler {
    unsafe fn on_interrupt() {
        if !ACTIVE.load(Ordering::Acquire) {
            return;
        }
        critical_section::with(|_| {
            let status = eth().dmasr().read().0 & IRQ_FLAGS;
            PENDING.store(PENDING.load(Ordering::Relaxed) | status, Ordering::Release);
            if status & PHY_INTERRUPT != 0 {
                eth().dmaier().modify(|register| register.set_phylink_ier(false));
            }
            eth().dmasr().write_value(pac::emac::regs::Dmasr(status));
        });
        WAKER.wake();
    }
}

pub struct Ethernet<'d, const TX: usize, const RX: usize> {
    _peri: Peri<'d, peripherals::ETH>,
    tx: ring::Ring,
    rx: ring::Ring,
    config: Config,
    range: u8,
    link: Link,
    faulted: bool,
    last_error: Option<Error>,
    dma_errors: u32,
    phy_id: u32,
    _not_sync: PhantomData<core::cell::Cell<()>>,
}

impl<'d, const TX: usize, const RX: usize> Ethernet<'d, TX, RX> {
    pub fn new(
        peri: Peri<'d, peripherals::ETH>,
        _irq: impl interrupt::typelevel::Binding<interrupt::typelevel::ETH, InterruptHandler> + 'd,
        resources: &'static mut Resources<TX, RX>,
        config: Config,
    ) -> Result<Self, Error> {
        if TX == 0
            || RX == 0
            || config.poll_limit == 0
            || config.phy_reset_attempts == 0
            || config.phy_address > 31
            || config.mac_address[0] & 1 != 0
            || config.mac_address == [0; 6]
        {
            return Err(Error::InvalidConfig);
        }
        let range = clock_range(peripherals::ETH::frequency().0)?;
        let control = pac::RCC.ctlr().read();
        if !control.eth_pllrdy() || !control.hserdy() {
            return Err(Error::BadClock);
        }
        let guard = OnDrop::new(stop);
        peripherals::ETH::enable_and_reset();
        let mut driver = Self {
            _peri: peri,
            tx: ring::Ring::new(&mut resources.tx_descriptors, &mut resources.tx_buffers, false),
            rx: ring::Ring::new(&mut resources.rx_descriptors, &mut resources.rx_buffers, true),
            config,
            range,
            link: DOWN,
            faulted: false,
            last_error: None,
            dma_errors: 0,
            phy_id: 0,
            _not_sync: PhantomData,
        };
        guard.defuse();
        driver.initialize()?;
        Ok(driver)
    }

    fn initialize(&mut self) -> Result<(), Error> {
        eth().dmaier().write_value(pac::emac::regs::Dmaier(0));
        eth().phy_cr().write_value(pac::emac::regs::PhyCr(
            (1 << 31) | (1 << 7) | u32::from(self.config.phy_address),
        ));
        eth().dmabmr().write_value(pac::emac::regs::Dmabmr(1));
        wait(
            self.config.poll_limit,
            || !eth().dmabmr().read().sr(),
            Error::ResetTimeout,
        )?;
        eth().macmiiar().write(|register| register.set_cr(self.range));
        self.smi_write(31, 0)?;
        for _ in 0..self.config.phy_reset_attempts {
            self.phy_id = (u32::from(self.smi_read(2)?) << 16) | u32::from(self.smi_read(3)?);
            if self.phy_id != 0 && self.phy_id != u32::MAX {
                break;
            }
        }
        if self.phy_id == 0 || self.phy_id == u32::MAX {
            return Err(Error::PhyNotFound);
        }
        self.smi_write(0, 0x8000)?;
        let mut reset = true;
        for _ in 0..self.config.phy_reset_attempts {
            if self.smi_read(0)? & 0x8000 == 0 {
                reset = false;
                break;
            }
        }
        if reset {
            return Err(Error::ResetTimeout);
        }
        self.smi_write(31, 7)?;
        let mask = self.smi_read(0x13)?;
        self.smi_write(0x13, (mask | (1 << 13)) & !(1 << 9))?;
        self.smi_write(31, 0)?;
        self.smi_read(0x1e)?;
        self.smi_write(0, 0x1200)?;
        eth().maccr().write_value(pac::emac::regs::Maccr(0));
        eth().macffr().write(|register| register.set_pam(true));
        eth().macfcr().write_value(pac::emac::regs::Macfcr(0));
        eth().macimr().write_value(pac::emac::regs::Macimr((1 << 3) | (1 << 9)));
        eth().mmcrimr().write(|register| {
            register.set_rfcem(true);
            register.set_rfaem(true);
            register.set_rgufm(true);
        });
        eth().mmctimr().write(|register| {
            register.set_tgfscm(true);
            register.set_tgfmscm(true);
            register.set_tgfm(true);
        });
        eth().maca0hr().write(|register| {
            register.set_maca0h(u16::from_le_bytes([
                self.config.mac_address[4],
                self.config.mac_address[5],
            ]))
        });
        eth().maca0lr().write(|register| {
            register.set_maca0l(u32::from_le_bytes(self.config.mac_address[..4].try_into().unwrap()))
        });
        self.tx.reset();
        self.rx.reset();
        eth()
            .dmatdlar()
            .write_value(pac::emac::regs::Dmatdlar(self.tx.address()));
        eth()
            .dmardlar()
            .write_value(pac::emac::regs::Dmardlar(self.rx.address()));
        eth().dmaomr().write_value(pac::emac::regs::Dmaomr(1 << 21));
        eth().dmasr().write_value(pac::emac::regs::Dmasr(IRQ_FLAGS));
        self.link = DOWN;
        self.faulted = false;
        critical_section::with(|_| {
            PENDING.store(0, Ordering::Release);
            ACTIVE.store(true, Ordering::Release);
            interrupt::typelevel::ETH::unpend();
            eth().dmaier().write_value(pac::emac::regs::Dmaier(INTERRUPTS));
            unsafe { interrupt::typelevel::ETH::enable() };
        });
        self.refresh_link()?;
        Ok(())
    }

    fn smi_transfer(&mut self, address: u8, value: Option<u16>) -> Result<u16, Error> {
        wait(
            self.config.poll_limit,
            || !eth().macmiiar().read().mb(),
            Error::SmiTimeout,
        )?;
        if let Some(value) = value {
            eth().macmiidr().write(|register| register.set_md(value));
        }
        eth().macmiiar().write(|register| {
            register.set_cr(self.range);
            register.set_pa(self.config.phy_address);
            register.set_mr(address);
            register.set_mw(value.is_some());
            register.set_mb(true);
        });
        wait(
            self.config.poll_limit,
            || !eth().macmiiar().read().mb(),
            Error::SmiTimeout,
        )?;
        Ok(eth().macmiidr().read().md())
    }

    fn smi_read(&mut self, address: u8) -> Result<u16, Error> {
        self.smi_transfer(address, None)
    }
    fn smi_write(&mut self, address: u8, value: u16) -> Result<(), Error> {
        self.smi_transfer(address, Some(value)).map(|_| ())
    }

    fn refresh_link(&mut self) -> Result<(), Error> {
        self.smi_read(1)?;
        let status = self.smi_read(1)?;
        let phy = self.smi_read(0x1a)?;
        let negotiated = status & 0x20 != 0 || self.smi_read(5)? == 0;
        let link = Link {
            up: status & 4 != 0 && phy & 0x0c != 0 && negotiated,
            speed_100m: phy & 8 != 0,
            full_duplex: phy & 2 != 0,
        };
        if link != self.link {
            critical_section::with(|_| {
                let enabled = eth().dmaier().read();
                eth().dmaier().write_value(pac::emac::regs::Dmaier(enabled.0 & !0x102));
                eth().dmaomr().modify(|register| {
                    register.set_st(false);
                    register.set_sr(false);
                });
                eth().maccr().modify(|register| {
                    register.set_te(false);
                    register.set_re(false);
                });
                wait(
                    self.config.poll_limit,
                    || eth().dmasr().read().0 & 0x007e_0000 == 0,
                    Error::ResetTimeout,
                )?;
                eth().dmasr().write_value(pac::emac::regs::Dmasr(0x102));
                PENDING.store(PENDING.load(Ordering::Relaxed) & !0x102, Ordering::Release);
                eth().maccr().write(|register| {
                    register.set_dm(link.full_duplex);
                    register.set_te(link.up);
                    register.set_re(link.up);
                });
                if link.up {
                    eth().dmaomr().modify(|register| {
                        register.set_st(true);
                        register.set_sr(true);
                    });
                    resume_receive();
                    resume_transmit();
                }
                eth().dmaier().write_value(enabled);
                Ok(())
            })?;
            self.link = link;
        }
        Ok(())
    }

    fn fault(&mut self, error: Error) {
        self.last_error = Some(error);
        self.faulted = true;
        self.link = DOWN;
        eth().dmaier().write_value(pac::emac::regs::Dmaier(0));
        eth().dmaomr().write_value(pac::emac::regs::Dmaomr(0));
        eth().maccr().write_value(pac::emac::regs::Maccr(0));
    }

    fn service(&mut self, context: &mut Context<'_>) -> bool {
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
        if status & PHY_INTERRUPT != 0 {
            let result = self.smi_read(0x1e).and_then(|_| self.refresh_link());
            if let Err(error) = result {
                self.fault(error);
                return false;
            }
            eth().dmaier().modify(|register| register.set_phylink_ier(true));
        }
        if status & (1 << 7) != 0 {
            resume_receive();
        }
        if status & ((1 << 2) | (1 << 5)) != 0 {
            resume_transmit();
        }
        self.link.up
    }

    pub fn phy_id(&self) -> u32 {
        self.phy_id
    }
    pub fn link(&self) -> Link {
        self.link
    }
    pub fn last_error(&self) -> Option<Error> {
        self.last_error
    }
    pub fn error_counts(&self) -> (u32, u32, u32) {
        (self.tx.errors, self.rx.errors, self.dma_errors)
    }

    /// Reset the MAC/PHY and drop queued frames after a controller fault.
    pub fn recover(&mut self) -> Result<(), Error> {
        stop();
        peripherals::ETH::enable_and_reset();
        let result = self.initialize();
        if let Err(error) = result {
            stop();
            self.faulted = true;
            self.link = DOWN;
            self.last_error = Some(error);
        }
        result
    }
}

impl<const TX: usize, const RX: usize> Drop for Ethernet<'_, TX, RX> {
    fn drop(&mut self) {
        stop();
    }
}

impl<const TX: usize, const RX: usize> embassy_net_driver::Driver for Ethernet<'_, TX, RX> {
    type RxToken<'a>
        = RxToken<'a>
    where
        Self: 'a;
    type TxToken<'a>
        = TxToken<'a>
    where
        Self: 'a;

    fn receive(&mut self, context: &mut Context<'_>) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
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

    fn transmit(&mut self, context: &mut Context<'_>) -> Option<Self::TxToken<'_>> {
        if !self.service(context) || !self.tx.tx_ready() {
            return None;
        }
        Some(TxToken { ring: &mut self.tx })
    }

    fn link_state(&mut self, context: &mut Context<'_>) -> LinkState {
        self.service(context);
        if self.faulted {
            return LinkState::Down;
        }
        if self.link.up {
            LinkState::Up
        } else {
            LinkState::Down
        }
    }

    fn capabilities(&self) -> Capabilities {
        let mut capabilities = Capabilities::default();
        capabilities.max_transmission_unit = ring::MTU;
        capabilities.max_burst_size = Some(TX.min(RX));
        capabilities
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.config.mac_address)
    }
}
