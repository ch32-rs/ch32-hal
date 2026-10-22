//! Synopsys DWC Ethernet MAC driver for CH32V305/V307.
//!
//! This driver supports the 10/100M Ethernet MAC found on CH32V305 and CH32V307
//! microcontrollers. The MAC uses the Synopsys DesignWare Cores (DWC) IP, which
//! is register-compatible with STM32's ETH v1a peripheral.
//!
//! The driver uses DMA descriptor rings for zero-copy TX/RX and implements
//! the `embassy-net-driver` traits for integration with `embassy-net`.
//!
//! # Hardware
//!
//! The MAC connects to an external PHY via MII or RMII interface. Pin
//! configuration must be done by the caller before creating the driver.
//!
//! ## RMII mode pins (typical, active-low remapping)
//!
//! | Signal           | Pin  | Direction |
//! |------------------|------|-----------|
//! | RMII_REF_CLK     | PA1  | Input     |
//! | MDIO             | PA2  | Bidir     |
//! | RMII_CRS_DV      | PA7  | Input     |
//! | MDC              | PC1  | Output    |
//! | RMII_RXD0        | PC4  | Input     |
//! | RMII_RXD1        | PC5  | Input     |
//! | RMII_TX_EN       | PB11 | Output    |
//! | RMII_TXD0        | PB12 | Output    |
//! | RMII_TXD1        | PB13 | Output    |

use core::marker::PhantomData;
use core::sync::atomic::{fence, Ordering};
use core::task::Context;

use embassy_net_driver::{Capabilities, HardwareAddress, LinkState};
use embassy_sync::waitqueue::AtomicWaker;

use crate::{pac, Peri};

mod dma;
mod phy;
mod pins;
mod smi;

pub use dma::{PacketQueue, RDes, RDesRing, RxToken, TDes, TDesRing, TxToken};
pub use phy::{GenericPhy, InternalPhy, Phy};
pub use pins::init_rmii_pins;
pub use smi::{Smi, SmiError};

use dma::MTU;

struct State {
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

// ============================================================================
// Driver
// ============================================================================

/// MII/RMII interface mode selection.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PhyInterface {
    /// Media Independent Interface (MII) - 18 pins.
    Mii,
    /// Reduced MII (RMII) - 9 pins.
    Rmii,
    /// Built-in 10M PHY — no external pins needed.
    ///
    /// The CH32V307 has an integrated 10BASE-T PHY connected internally to the
    /// EMAC. TX+/TX-/RX+/RX- go through a transformer on dedicated analog pins.
    Internal10M,
}

/// An error while initializing the Ethernet MAC or PHY.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InitError {
    /// The DMA software reset did not complete within the polling budget.
    DmaResetTimeout,
    /// A PHY management transaction failed.
    Smi(SmiError),
}

impl From<SmiError> for InitError {
    fn from(error: SmiError) -> Self {
        Self::Smi(error)
    }
}

/// Ethernet MAC driver.
///
/// `TX` and `RX` are the number of DMA descriptors (and buffers) in each ring.
pub struct Ethernet<'d, T: Instance, const TX: usize, const RX: usize, P: Phy> {
    tx: TDesRing<'d, T, TX>,
    rx: RDesRing<'d, T, RX>,
    phy: P,
    smi: Smi,
    mac_addr: [u8; 6],
    phy_interface: PhyInterface,
    _phantom: PhantomData<T>,
}

impl<'d, T: Instance, const TX: usize, const RX: usize, P: Phy> Ethernet<'d, T, TX, RX, P> {
    /// Create and initialize a new Ethernet MAC driver.
    ///
    /// # Arguments
    ///
    /// * `queue` - Static packet queue for DMA descriptors and buffers
    /// * `mac_addr` - 6-byte MAC address
    /// * `phy` - PHY driver instance
    /// * `phy_interface` - MII or RMII mode
    ///
    /// # Errors
    ///
    /// Returns an error if DMA reset or a PHY management transaction times out.
    pub fn new(
        _peri: Peri<'d, T>,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        queue: &'d mut PacketQueue<TX, RX>,
        mac_addr: [u8; 6],
        mut phy: P,
        phy_interface: PhyInterface,
    ) -> Result<Self, InitError> {
        T::enable_and_reset();

        let e = T::regs();

        // 2. Interface-specific setup
        if phy_interface == PhyInterface::Internal10M {
            // Configure PLL3 to 60MHz for the 10M PHY clock.
            // HSE=8MHz → PREDIV2=/2 → 4MHz → PLL3*15 → 60MHz.
            // This clock is required for the built-in 10M PHY to operate.
            pac::RCC.ctlr().modify(|w| w.set_pll3on(false));
            pac::RCC.cfgr2().modify(|w| {
                w.set_prediv2(pac::rcc::vals::Prediv::DIV2);
                w.set_pll3mul(pac::rcc::vals::PllxMul::MUL15);
            });
            pac::RCC.ctlr().modify(|w| w.set_pll3on(true));
            while !pac::RCC.ctlr().read().pll3rdy() {}

            // Enable the built-in 10M PHY via EXTEND register
            pac::EXTEND.ctr().modify(|w| w.set_eth_10m_en(true));
        } else {
            // Select MII/RMII interface via AFIO for external PHY
            #[cfg(afio_v3)]
            {
                pac::AFIO.pcfr1().modify(|w| {
                    w.set_mii_rmii_sel(phy_interface == PhyInterface::Rmii);
                });
            }
        }

        // 3. DMA soft reset
        e.dmabmr().write(|w| w.set_sr(true));
        let mut timeout = 10_000u32;
        while e.dmabmr().read().sr() {
            for _ in 0..1000 {
                qingke::riscv::asm::nop();
            }
            timeout -= 1;
            if timeout == 0 {
                return Err(InitError::DmaResetTimeout);
            }
        }

        // 4. Select SMI clock divider — div42 for HCLK 60-100MHz+
        let hclk = crate::rcc::clocks().hclk.0;
        let clock_range: u8 = if hclk < 35_000_000 {
            0b010 // div16
        } else if hclk < 60_000_000 {
            0b011 // div26
        } else {
            0b000 // div42
        };
        e.macmiiar().write(|w| w.set_cr(clock_range));
        let mut smi = Smi::new(T::regs(), clock_range);

        // 5. Configure MAC
        e.maccr().write(|w| {
            w.set_ipco(true); // IPv4 checksum offload
            w.set_dm(true); // Full duplex

            if phy_interface == PhyInterface::Internal10M {
                w.set_ire(true); // Internal 50-ohm pull-up (10M PHY only)
            } else {
                w.set_fes(1); // 100 Mbps for external PHY
                w.set_apcs(true); // Auto pad/CRC strip
                w.set_rd(true); // Retry disable
            }
        });

        // 6. Frame filter
        e.macffr().write(|w| {
            w.set_pam(true); // Pass all multicast (needed for ARP/DHCP)
            if phy_interface == PhyInterface::Internal10M {
                // Promiscuous + receive-all required for 10M PHY polarity detection
                w.set_ra(true);
                w.set_pm(true);
            }
        });

        // 7. Hash table (unused, zero)
        e.machthr().write(|w| w.set_hth(0));
        e.machtlr().write(|w| w.set_htl(0));

        // 8. Flow control: pause time 0x100
        e.macfcr().write(|w| w.set_pt(0x100));

        // 9. Set MAC address
        // MACA0HR: bytes [5:4], MACA0LR: bytes [3:0]
        let mac_high = ((mac_addr[5] as u16) << 8) | (mac_addr[4] as u16);
        let mac_low = ((mac_addr[3] as u32) << 24)
            | ((mac_addr[2] as u32) << 16)
            | ((mac_addr[1] as u32) << 8)
            | (mac_addr[0] as u32);
        e.maca0hr().write(|w| w.set_maca0h(mac_high));
        e.maca0lr().write(|w| w.set_maca0l(mac_low)); // LR write triggers sync

        // 10. DMA bus mode: 32-beat burst length
        e.dmabmr().write(|w| w.set_pbl(32));

        // 11. DMA operation mode: store-and-forward for both TX and RX
        e.dmaomr().write(|w| {
            w.set_rsf(true);
            w.set_tsf(true);
        });

        // Complete fallible PHY setup before DMA can access the packet queue.
        phy.phy_reset(&mut smi)?;
        phy.phy_init(&mut smi)?;

        // 12. Initialize DMA descriptor rings
        let tx = TDesRing::new(&mut queue.tx_desc, &mut queue.tx_buf);
        let rx = RDesRing::new(&mut queue.rx_desc, &mut queue.rx_buf);

        fence(Ordering::SeqCst);

        // 13. Enable MAC receiver and transmitter
        e.maccr().modify(|w| {
            w.set_re(true);
            w.set_te(true);
        });

        // 14. Start DMA: flush TX FIFO, then start TX and RX
        e.dmaomr().modify(|w| w.set_ftf(true));
        e.dmaomr().modify(|w| {
            w.set_st(true);
            w.set_sr(true);
        });

        // Demand poll RX to start receiving
        e.dmarpdr().write(|w| w.set_rpd(0));

        // 15. Enable DMA interrupts
        e.dmaier().write(|w| {
            w.set_tie(true); // TX complete
            w.set_rie(true); // RX complete
            w.set_nise(true); // Normal interrupt summary
            w.set_aise(true); // Abnormal interrupt summary
            w.set_rbuie(true); // RX buffer unavailable
        });

        // 17. Enable ETH interrupt in PFIC
        unsafe {
            qingke::pfic::enable_interrupt(pac::Interrupt::ETH as u8);
        }

        Ok(Self {
            tx,
            rx,
            phy,
            smi,
            mac_addr,
            phy_interface,
            _phantom: PhantomData,
        })
    }

    /// Get the SMI interface for direct PHY register access.
    pub fn smi(&mut self) -> &mut Smi {
        &mut self.smi
    }

    /// Get the configured MAC address.
    pub fn mac_addr(&self) -> [u8; 6] {
        self.mac_addr
    }
}

impl<'d, T: Instance, const TX: usize, const RX: usize, P: Phy> Drop for Ethernet<'d, T, TX, RX, P> {
    fn drop(&mut self) {
        let e = T::regs();

        // Stop DMA TX
        e.dmaomr().modify(|w| w.set_st(false));
        // Disable MAC TX/RX
        e.maccr().modify(|w| {
            w.set_te(false);
            w.set_re(false);
        });
        // Stop DMA RX
        e.dmaomr().modify(|w| w.set_sr(false));

        // Disable ETH interrupt
        unsafe {
            qingke::pfic::disable_interrupt(pac::Interrupt::ETH as u8);
        }

        // Disable built-in 10M PHY if it was enabled
        if self.phy_interface == PhyInterface::Internal10M {
            pac::EXTEND.ctr().modify(|w| w.set_eth_10m_en(false));
        }

        // Disable RCC clocks
        T::disable();
    }
}

trait SealedInstance: crate::peripheral::RccPeripheral {
    fn regs() -> pac::emac::Eth;
    fn state() -> &'static State;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + embassy_hal_internal::PeripheralType + 'static {
    /// Interrupt for this instance
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (emac, $inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> pac::emac::Eth {
                pac::$inst
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let s = T::state();

        s.waker.wake();

        // Clear TX, RX, and normal interrupt summary flags (W1C)
        r.dmasr().write(|w| {
            w.set_ts(true);
            w.set_rs(true);
            w.set_nis(true);
        });

        // Two dummy reads for peripheral clock sync (per reference manual)
        r.dmasr().read();
        r.dmasr().read();
    }
}

// ============================================================================
// embassy-net-driver integration
// ============================================================================

impl<'d, T: Instance, const TX: usize, const RX: usize, P: Phy> embassy_net_driver::Driver
    for Ethernet<'d, T, TX, RX, P>
{
    type RxToken<'a>
        = RxToken<'a, 'd, T, RX>
    where
        Self: 'a;
    type TxToken<'a>
        = TxToken<'a, 'd, T, TX>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let s = T::state();
        s.waker.register(cx.waker());
        if self.rx.available().is_some() && self.tx.available().is_some() {
            // rx and tx are disjoint struct fields, Rust permits simultaneous &mut borrows.
            Some((RxToken { rx: &mut self.rx }, TxToken { tx: &mut self.tx }))
        } else {
            None
        }
    }

    fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
        let s = T::state();
        s.waker.register(cx.waker());
        if self.tx.available().is_some() {
            Some(TxToken { tx: &mut self.tx })
        } else {
            None
        }
    }

    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = Some(self.tx.len());
        caps
    }

    fn link_state(&mut self, cx: &mut Context) -> LinkState {
        if self.phy.poll_link(&mut self.smi, cx).unwrap_or(false) {
            LinkState::Up
        } else {
            LinkState::Down
        }
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac_addr)
    }
}
