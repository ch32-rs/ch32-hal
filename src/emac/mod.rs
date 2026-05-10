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

use core::sync::atomic::{fence, Ordering};
use core::task::Context;

use embassy_net_driver::{Capabilities, HardwareAddress, LinkState};
use embassy_sync::waitqueue::AtomicWaker;

use crate::{pac, Peri};

/// Shorthand for the ETH peripheral instance.
fn eth() -> pac::emac::Eth {
    pac::ETH
}

// ============================================================================
// DMA descriptors
// ============================================================================

const MTU: usize = 1514;
const TX_BUF_SIZE: usize = 1514;
const RX_BUF_SIZE: usize = 1536;

// TX descriptor word 0 (TDES0) bits
const TDES0_OWN: u32 = 1 << 31;
const TDES0_IC: u32 = 1 << 30; // Interrupt on completion
const TDES0_LS: u32 = 1 << 29; // Last segment
const TDES0_FS: u32 = 1 << 28; // First segment
const TDES0_TCH: u32 = 1 << 20; // Second address chained

// RX descriptor word 0 (RDES0) bits
const RDES0_OWN: u32 = 1 << 31;
const RDES0_FS: u32 = 1 << 9; // First descriptor
const RDES0_LS: u32 = 1 << 8; // Last descriptor
const RDES0_ES: u32 = 1 << 15; // Error summary
const RDES0_FL_MASK: u32 = 0x3FFF0000; // Frame length mask
const RDES0_FL_SHIFT: u32 = 16;

// RX descriptor word 1 (RDES1) bits
const RDES1_RCH: u32 = 1 << 14; // Second address chained

/// A single TX DMA descriptor (4 x 32-bit words).
#[repr(C, align(4))]
pub struct TDes {
    tdes0: u32,
    tdes1: u32,
    tdes2: u32,
    tdes3: u32,
}

impl TDes {
    pub const fn new() -> Self {
        Self {
            tdes0: 0,
            tdes1: 0,
            tdes2: 0,
            tdes3: 0,
        }
    }

    #[inline]
    fn tdes0(&self) -> u32 {
        unsafe { core::ptr::read_volatile(&self.tdes0) }
    }

    #[inline]
    fn set_tdes0(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.tdes0, val) }
    }

    #[inline]
    fn set_tdes1(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.tdes1, val) }
    }

    #[inline]
    fn set_tdes2(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.tdes2, val) }
    }

    #[inline]
    fn set_tdes3(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.tdes3, val) }
    }
}

/// A single RX DMA descriptor (4 x 32-bit words).
#[repr(C, align(4))]
pub struct RDes {
    rdes0: u32,
    rdes1: u32,
    rdes2: u32,
    rdes3: u32,
}

impl RDes {
    pub const fn new() -> Self {
        Self {
            rdes0: 0,
            rdes1: 0,
            rdes2: 0,
            rdes3: 0,
        }
    }

    #[inline]
    fn rdes0(&self) -> u32 {
        unsafe { core::ptr::read_volatile(&self.rdes0) }
    }

    #[inline]
    fn set_rdes0(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.rdes0, val) }
    }

    #[inline]
    fn set_rdes1(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.rdes1, val) }
    }

    #[inline]
    fn set_rdes2(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.rdes2, val) }
    }

    #[inline]
    fn set_rdes3(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.rdes3, val) }
    }
}

// ============================================================================
// DMA descriptor rings
// ============================================================================

/// A TX DMA descriptor ring.
pub struct TDesRing<'d, const N: usize> {
    desc: &'d mut [TDes; N],
    buf: &'d mut [[u8; TX_BUF_SIZE]; N],
    index: usize,
}

impl<'d, const N: usize> TDesRing<'d, N> {
    fn new(desc: &'d mut [TDes; N], buf: &'d mut [[u8; TX_BUF_SIZE]; N]) -> Self {
        // Set up chained descriptors
        for i in 0..N {
            let next = if i + 1 < N {
                &desc[i + 1] as *const TDes as u32
            } else {
                &desc[0] as *const TDes as u32
            };

            desc[i].set_tdes0(TDES0_TCH | TDES0_IC | TDES0_FS | TDES0_LS);
            desc[i].set_tdes1(0);
            desc[i].set_tdes2(buf[i].as_ptr() as u32);
            desc[i].set_tdes3(next);
        }
        // Last descriptor chains back to first (already set in loop above).

        // Tell DMA where the TX descriptor ring starts
        eth().dmatdlar().write(|w| w.set_stl(&desc[0] as *const TDes as u32));

        Self { desc, buf, index: 0 }
    }

    /// Returns a mutable slice of the next available TX buffer, or None if DMA owns it.
    pub fn available(&mut self) -> Option<&mut [u8]> {
        let d = &self.desc[self.index];
        if d.tdes0() & TDES0_OWN == 0 {
            Some(&mut self.buf[self.index][..])
        } else {
            None
        }
    }

    /// Submit the current TX buffer to the DMA for transmission.
    pub fn transmit(&mut self, len: usize) {
        let d = &mut self.desc[self.index];

        // Set buffer pointer and length
        d.set_tdes2(self.buf[self.index].as_ptr() as u32);
        d.set_tdes1(len as u32);

        // Set ownership to DMA with fence
        fence(Ordering::Release);
        let base = d.tdes0() & !TDES0_OWN;
        d.set_tdes0(base | TDES0_OWN);

        // Advance ring index
        self.index = (self.index + 1) % N;

        // Poll transmit demand
        eth().dmatpdr().write(|w| w.set_tpd(0));
    }

    /// Number of descriptors in the ring.
    pub fn len(&self) -> usize {
        N
    }
}

/// An RX DMA descriptor ring.
pub struct RDesRing<'d, const N: usize> {
    desc: &'d mut [RDes; N],
    buf: &'d mut [[u8; RX_BUF_SIZE]; N],
    index: usize,
}

impl<'d, const N: usize> RDesRing<'d, N> {
    fn new(desc: &'d mut [RDes; N], buf: &'d mut [[u8; RX_BUF_SIZE]; N]) -> Self {
        for i in 0..N {
            let next = if i + 1 < N {
                &desc[i + 1] as *const RDes as u32
            } else {
                &desc[0] as *const RDes as u32
            };

            desc[i].set_rdes0(RDES0_OWN); // DMA owns initially
            desc[i].set_rdes1(RDES1_RCH | (RX_BUF_SIZE as u32 & 0x1FFF));
            desc[i].set_rdes2(buf[i].as_ptr() as u32);
            desc[i].set_rdes3(next);
        }
        // Last descriptor chains back to first (already set in loop above).

        // Tell DMA where the RX descriptor ring starts
        eth().dmardlar().write(|w| w.set_srl(&desc[0] as *const RDes as u32));

        Self { desc, buf, index: 0 }
    }

    /// Returns a reference to the received frame data if a complete frame is available.
    pub fn available(&mut self) -> Option<&mut [u8]> {
        // If DMA RX is stopped, demand poll to restart
        if eth().dmasr().read().rps() == 0b100 {
            self.demand_poll();
        }

        let d = &self.desc[self.index];
        let rdes0 = d.rdes0();

        // Check if DMA still owns this descriptor
        if rdes0 & RDES0_OWN != 0 {
            return None;
        }

        // Check for first+last segment (single descriptor per frame)
        if rdes0 & (RDES0_FS | RDES0_LS) != (RDES0_FS | RDES0_LS) {
            // Not a single-descriptor frame; skip it and give back to DMA
            self.pop_packet();
            return None;
        }

        // Check for errors
        if rdes0 & RDES0_ES != 0 {
            self.pop_packet();
            return None;
        }

        // Extract frame length (includes CRC, subtract 4)
        let len = ((rdes0 & RDES0_FL_MASK) >> RDES0_FL_SHIFT) as usize;
        let len = if len >= 4 { len - 4 } else { len };
        let len = len.min(RX_BUF_SIZE);

        fence(Ordering::Acquire);

        Some(&mut self.buf[self.index][..len])
    }

    /// Release the current RX descriptor back to DMA ownership.
    pub fn pop_packet(&mut self) {
        let d = &mut self.desc[self.index];
        d.set_rdes0(RDES0_OWN);
        self.demand_poll();
        self.index = (self.index + 1) % N;
    }

    fn demand_poll(&self) {
        eth().dmarpdr().write(|w| w.set_rpd(0));
    }
}

// ============================================================================
// Packet buffer queue (static allocation)
// ============================================================================

/// Static storage for DMA descriptors and packet buffers.
///
/// `TX` and `RX` are the number of descriptors (and buffers) in each ring.
/// Typical values: 2-4 for TX, 4-8 for RX.
///
/// # Usage
///
/// ```ignore
/// static QUEUE: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
/// let queue = QUEUE.init(PacketQueue::new());
/// ```
pub struct PacketQueue<const TX: usize, const RX: usize> {
    tx_desc: [TDes; TX],
    rx_desc: [RDes; RX],
    tx_buf: [[u8; TX_BUF_SIZE]; TX],
    rx_buf: [[u8; RX_BUF_SIZE]; RX],
}

impl<const TX: usize, const RX: usize> PacketQueue<TX, RX> {
    /// Create a new zeroed packet queue.
    pub const fn new() -> Self {
        Self {
            tx_desc: [const { TDes::new() }; TX],
            rx_desc: [const { RDes::new() }; RX],
            tx_buf: [[0u8; TX_BUF_SIZE]; TX],
            rx_buf: [[0u8; RX_BUF_SIZE]; RX],
        }
    }
}

// ============================================================================
// PHY management via SMI (Station Management Interface)
// ============================================================================

/// Station Management Interface for MII PHY register access.
pub struct Smi {
    clock_range: u8,
}

impl Smi {
    /// Read a PHY register.
    pub fn read(&self, phy_addr: u8, reg: u8) -> u16 {
        eth().macmiiar().write(|w| {
            w.set_pa(phy_addr & 0x1F);
            w.set_mr(reg & 0x1F);
            w.set_cr(self.clock_range);
            w.set_mw(false);
            w.set_mb(true);
        });
        while eth().macmiiar().read().mb() {}
        eth().macmiidr().read().md()
    }

    /// Write a PHY register.
    pub fn write(&self, phy_addr: u8, reg: u8, val: u16) {
        eth().macmiidr().write(|w| w.set_md(val));
        eth().macmiiar().write(|w| {
            w.set_pa(phy_addr & 0x1F);
            w.set_mr(reg & 0x1F);
            w.set_cr(self.clock_range);
            w.set_mw(true);
            w.set_mb(true);
        });
        while eth().macmiiar().read().mb() {}
    }
}

// ============================================================================
// PHY trait
// ============================================================================

/// Trait for Ethernet PHY drivers.
pub trait Phy {
    /// Reset the PHY and wait for it to come out of reset.
    fn phy_reset(&mut self, smi: &Smi);
    /// Initialize the PHY (enable autonegotiation, etc).
    fn phy_init(&mut self, smi: &Smi);
    /// Poll the link state. Returns true if link is up.
    fn poll_link(&mut self, smi: &Smi, cx: &mut Context) -> bool;
}

/// MII PHY register constants.
mod phy_regs {
    pub const BCR: u8 = 0x00;
    pub const BSR: u8 = 0x01;
    /// WCH internal 10M PHY MDIX/polarity control register.
    pub const MDIX: u8 = 0x1E;

    pub const BCR_RESET: u16 = 1 << 15;
    pub const BCR_AUTONEG: u16 = 1 << 12;
    pub const BCR_AUTONEG_RESTART: u16 = 1 << 9;

    pub const BSR_LINK_UP: u16 = 1 << 2;

    /// Auto PN polarity detection (bits 3:2 = 0b10).
    pub const MDIX_PN_AUTO: u16 = 2 << 2;
}

/// Generic PHY driver that works with IEEE 802.3 compliant PHYs.
///
/// For the CH32V307 built-in 10M PHY, use SMI address 1.
/// The WCH internal PHY also needs the MDIX auto-polarity register (0x1E)
/// written during init for reliable link establishment.
pub struct GenericPhy {
    phy_addr: u8,
}

impl GenericPhy {
    /// Create a new generic PHY driver.
    ///
    /// `phy_addr` is the SMI address of the PHY (0-31).
    /// For the CH32V307 built-in 10M PHY, use address 1.
    pub fn new(phy_addr: u8) -> Self {
        Self { phy_addr }
    }
}

impl Phy for GenericPhy {
    fn phy_reset(&mut self, smi: &Smi) {
        smi.write(self.phy_addr, phy_regs::BCR, phy_regs::BCR_RESET);
        // Wait for reset bit to self-clear
        while smi.read(self.phy_addr, phy_regs::BCR) & phy_regs::BCR_RESET != 0 {}
    }

    fn phy_init(&mut self, smi: &Smi) {
        // Set auto PN polarity on MDIX register (WCH internal 10M PHY
        // needs this for reliable link establishment; harmless on external PHYs).
        smi.write(self.phy_addr, phy_regs::MDIX, phy_regs::MDIX_PN_AUTO);
        // Start auto-negotiation
        smi.write(
            self.phy_addr,
            phy_regs::BCR,
            phy_regs::BCR_AUTONEG | phy_regs::BCR_AUTONEG_RESTART,
        );
    }

    fn poll_link(&mut self, smi: &Smi, _cx: &mut Context) -> bool {
        let bsr = smi.read(self.phy_addr, phy_regs::BSR);
        bsr & phy_regs::BSR_LINK_UP != 0
    }
}

// ============================================================================
// Interrupt handler
// ============================================================================

static WAKER: AtomicWaker = AtomicWaker::new();

/// Interrupt handler for the Ethernet MAC.
///
/// Users must bind this to the ETH interrupt via `#[interrupt] fn ETH()`.
///
/// # Safety
///
/// This function is called from interrupt context.
pub unsafe fn on_interrupt() {
    WAKER.wake();

    // Clear TX, RX, and normal interrupt summary flags (W1C)
    eth().dmasr().write(|w| {
        w.set_ts(true);
        w.set_rs(true);
        w.set_nis(true);
    });

    // Two dummy reads for peripheral clock sync (per reference manual)
    eth().dmasr().read();
    eth().dmasr().read();
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

/// Configure GPIO pins for RMII mode (default pin mapping, no remap).
///
/// This sets up the 9 RMII pins with appropriate modes:
/// - Input floating: REF_CLK (PA1), CRS_DV (PA7), RXD0 (PC4), RXD1 (PC5)
/// - AF push-pull 50MHz: MDIO (PA2), MDC (PC1), TX_EN (PB11), TXD0 (PB12), TXD1 (PB13)
///
/// Call this before [`Ethernet::new()`].
pub fn init_rmii_pins(
    _ref_clk: Peri<'_, impl crate::gpio::Pin>,
    _mdio: Peri<'_, impl crate::gpio::Pin>,
    _crs_dv: Peri<'_, impl crate::gpio::Pin>,
    _mdc: Peri<'_, impl crate::gpio::Pin>,
    _rxd0: Peri<'_, impl crate::gpio::Pin>,
    _rxd1: Peri<'_, impl crate::gpio::Pin>,
    _tx_en: Peri<'_, impl crate::gpio::Pin>,
    _txd0: Peri<'_, impl crate::gpio::Pin>,
    _txd1: Peri<'_, impl crate::gpio::Pin>,
) {
    use crate::gpio::{AFType, Pull, Speed};

    // Input pins: floating input
    _ref_clk.set_as_input(Pull::None);
    _crs_dv.set_as_input(Pull::None);
    _rxd0.set_as_input(Pull::None);
    _rxd1.set_as_input(Pull::None);

    // Output pins: AF push-pull, 50MHz
    _mdio.set_as_af_output(AFType::OutputPushPull, Speed::High);
    _mdc.set_as_af_output(AFType::OutputPushPull, Speed::High);
    _tx_en.set_as_af_output(AFType::OutputPushPull, Speed::High);
    _txd0.set_as_af_output(AFType::OutputPushPull, Speed::High);
    _txd1.set_as_af_output(AFType::OutputPushPull, Speed::High);
}

/// Ethernet MAC driver.
///
/// `TX` and `RX` are the number of DMA descriptors (and buffers) in each ring.
pub struct Ethernet<'d, const TX: usize, const RX: usize, P: Phy> {
    tx: TDesRing<'d, TX>,
    rx: RDesRing<'d, RX>,
    phy: P,
    smi: Smi,
    mac_addr: [u8; 6],
    phy_interface: PhyInterface,
}

impl<'d, const TX: usize, const RX: usize, P: Phy> Ethernet<'d, TX, RX, P> {
    /// Create and initialize a new Ethernet MAC driver.
    ///
    /// # Arguments
    ///
    /// * `queue` - Static packet queue for DMA descriptors and buffers
    /// * `mac_addr` - 6-byte MAC address
    /// * `phy` - PHY driver instance
    /// * `phy_interface` - MII or RMII mode
    /// * `hclk_mhz` - AHB clock frequency in MHz (for SMI clock divider)
    ///
    /// # Panics
    ///
    /// Panics if the DMA soft reset doesn't complete within ~1ms.
    pub fn new(
        queue: &'d mut PacketQueue<TX, RX>,
        mac_addr: [u8; 6],
        mut phy: P,
        phy_interface: PhyInterface,
        hclk_mhz: u32,
    ) -> Self {
        let e = eth();

        // 1. Enable RCC clocks for ETH MAC, TX, and RX
        pac::RCC.ahbpcenr().modify(|w| {
            w.set_ethmacen(true);
            w.set_ethmactxen(true);
            w.set_ethmacrxen(true);
        });

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
            assert!(timeout > 0, "EMAC DMA soft reset timeout");
        }

        // 4. Select SMI clock divider — div42 for HCLK 60-100MHz+
        let clock_range: u8 = if hclk_mhz < 35 {
            0b010 // div16
        } else if hclk_mhz < 60 {
            0b011 // div26
        } else {
            0b000 // div42
        };
        e.macmiiar().write(|w| w.set_cr(clock_range));
        let smi = Smi { clock_range };

        // 5. Configure MAC
        e.maccr().write(|w| {
            w.set_ipco(true); // IPv4 checksum offload
            w.set_dm(true); // Full duplex

            if phy_interface == PhyInterface::Internal10M {
                w.set_ire(true); // Internal 50-ohm pull-up (10M PHY only)
            } else {
                w.set_fes(true); // 100 Mbps for external PHY
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

        // 16. Reset and initialize PHY
        phy.phy_reset(&smi);
        phy.phy_init(&smi);

        // 17. Enable ETH interrupt in PFIC
        unsafe {
            qingke::pfic::enable_interrupt(pac::Interrupt::ETH as u8);
        }

        Self {
            tx,
            rx,
            phy,
            smi,
            mac_addr,
            phy_interface,
        }
    }

    /// Get the SMI interface for direct PHY register access.
    pub fn smi(&self) -> &Smi {
        &self.smi
    }

    /// Get the configured MAC address.
    pub fn mac_addr(&self) -> [u8; 6] {
        self.mac_addr
    }
}

impl<'d, const TX: usize, const RX: usize, P: Phy> Drop for Ethernet<'d, TX, RX, P> {
    fn drop(&mut self) {
        let e = eth();

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
        pac::RCC.ahbpcenr().modify(|w| {
            w.set_ethmacen(false);
            w.set_ethmactxen(false);
            w.set_ethmacrxen(false);
        });
    }
}

// ============================================================================
// embassy-net-driver integration
// ============================================================================

/// Token for receiving a single packet from the DMA ring.
pub struct RxToken<'a, 'd, const N: usize> {
    rx: &'a mut RDesRing<'d, N>,
}

/// Token for transmitting a single packet via the DMA ring.
pub struct TxToken<'a, 'd, const N: usize> {
    tx: &'a mut TDesRing<'d, N>,
}

impl<'a, 'd, const N: usize> embassy_net_driver::RxToken for RxToken<'a, 'd, N> {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, f: F) -> R {
        let pkt = self.rx.available().expect("RxToken created without available packet");
        let r = f(pkt);
        self.rx.pop_packet();
        r
    }
}

impl<'a, 'd, const N: usize> embassy_net_driver::TxToken for TxToken<'a, 'd, N> {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, len: usize, f: F) -> R {
        let pkt = self.tx.available().expect("TxToken created without available buffer");
        let r = f(&mut pkt[..len]);
        self.tx.transmit(len);
        r
    }
}

impl<'d, const TX: usize, const RX: usize, P: Phy> embassy_net_driver::Driver for Ethernet<'d, TX, RX, P> {
    type RxToken<'a>
        = RxToken<'a, 'd, RX>
    where
        Self: 'a;
    type TxToken<'a>
        = TxToken<'a, 'd, TX>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        WAKER.register(cx.waker());
        if self.rx.available().is_some() && self.tx.available().is_some() {
            // rx and tx are disjoint struct fields, Rust permits simultaneous &mut borrows.
            Some((RxToken { rx: &mut self.rx }, TxToken { tx: &mut self.tx }))
        } else {
            None
        }
    }

    fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
        WAKER.register(cx.waker());
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
        if self.phy.poll_link(&self.smi, cx) {
            LinkState::Up
        } else {
            LinkState::Down
        }
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac_addr)
    }
}
