//! Ethernet MAC+PHY driver for CH32V208 (10M PHY).
//!
//! This module provides an embassy-net compatible driver for the built-in 10M Ethernet MAC+PHY
//! found on CH32V208 microcontrollers. The CH32V208 uses an ENC28J60-inspired register set
//! with register-based buffer management (no DMA descriptors).
//!
//! Uses `embassy-net-driver-channel` to provide multi-buffer RX/TX queues,
//! so packets are not lost while the CPU is processing.

use core::marker::PhantomData;
use core::sync::atomic::{fence, Ordering};

use embassy_net_driver::LinkState;
use embassy_net_driver_channel as ch;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;

use crate::eth::{Instance, StationManagement, PHY};
use crate::interrupt;

const MTU: usize = 1514;

/// The `embassy-net` device type for the CH32V208 10M Ethernet driver.
pub type Device<'d> = ch::Device<'d, MTU>;

/// Buffer state for the channel driver.
///
/// `N_RX` and `N_TX` set the number of RX/TX packet buffers.
/// More buffers improve throughput at the cost of RAM.
pub struct State<const N_RX: usize, const N_TX: usize> {
    ch_state: ch::State<MTU, N_RX, N_TX>,
}

impl<const N_RX: usize, const N_TX: usize> State<N_RX, N_TX> {
    /// Create a new buffer state.
    pub const fn new() -> Self {
        Self {
            ch_state: ch::State::new(),
        }
    }
}

/// ISR → Runner communication channels.
///
/// Using embassy_sync Channels instead of bare flags so the Runner
/// can `.recv().await` without busy-polling.
static RX_CH: Channel<CriticalSectionRawMutex, u16, 1> = Channel::new();
/// Built-in PHY: MDIX register and its receive polarity bits (3:2):
/// 00 positive (reset default), 01 negative.
const PHY_REG_MDIX: u8 = 0x1E;
const PHY_MDIX_PN_MASK: u16 = 3 << 2;
const PHY_MDIX_PN_NEGATIVE: u16 = 1 << 2;
const PHY_REG_BCR: u8 = 0x00;
const PHY_REG_BCR_AN_RESTART: u16 = 1 << 9;
/// Link polls (500 ms each) without link before switching the polarity.
const POLARITY_SWITCH_POLLS: u32 = 4;

/// Timer wake-up latency allowance for the TX completion check (measured
/// ~20 us at 120 MHz: aiming 20 us early needs one re-check for ~3 % of frames).
const TX_WAKE_EARLY_US: u64 = 20;

#[interrupt]
unsafe fn ETH() {
    let mac = crate::pac::ETH;

    // Disable global interrupt while processing
    mac.eie().modify(|w| w.set_intie(false));
    fence(Ordering::SeqCst);

    // Only RX raises this interrupt (see `new()`). A write to EIR, whatever its
    // value, drops any EIR flag the MAC sets at that moment. With RX as the only
    // source, the write follows the RXIF that caused it by the interrupt
    // latency, and the next RXIF cannot come before the next frame ends.
    let flags = mac.eir().read();
    if flags.rxif() {
        mac.eir().write(|w| w.set_rxif(true));
        let len = mac.erxln().read().erxln();
        // Try to send; if full, packet is dropped (better than blocking in ISR)
        let _ = RX_CH.try_send(len);
    }

    fence(Ordering::SeqCst);
    mac.eie().modify(|w| w.set_intie(true));
}

/// Error type for ethernet initialization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitError {
    /// The core frequency must be 60MHz or 120MHz.
    CoreFrequencyInvalid,
}

/// The async runner that drives the ethernet hardware.
///
/// Must be spawned as a separate embassy task via [`Runner::run()`].
pub struct Runner<'d, P: PHY> {
    mac: crate::pac::eth::Eth,
    phy: P,
    ch: ch::Runner<'d, MTU>,
    station_management: EthernetStationManagement<crate::peripherals::ETH>,
}

impl<'d, P: PHY> Runner<'d, P> {
    /// Run the ethernet driver forever.
    ///
    /// This handles RX, TX, and link state monitoring concurrently.
    pub async fn run(self) -> ! {
        let mac = self.mac;
        let mut phy = self.phy;
        let mut station_management = self.station_management;
        let (state_chan, mut rx_chan, mut tx_chan) = self.ch.split();

        // Drain any stale signals
        while RX_CH.try_receive().is_ok() {}

        let rx_fut = async {
            loop {
                // Get a buffer from the channel to receive into
                let buf = rx_chan.rx_buf().await;

                // Point hardware at this buffer and enable reception
                mac.mamxfl().write(|w| w.set_mamxfl(buf.len() as u16));
                mac.erxst().write(|w| w.set_erxst(buf.as_ptr() as u16));
                mac.macon1().write(|w| w.set_marxen(true));
                // RX_EN is set once in `new()`: a read-modify-write of ECON1 here
                // could write back a TX_RTS bit the MAC has just cleared.

                // Wait for RX interrupt - len comes from the ISR
                let len = RX_CH.receive().await as usize;
                let len = len.min(buf.len());

                // Tell the channel we received `len` bytes
                rx_chan.rx_done(len);
            }
        };

        let tx_fut = async {
            loop {
                // Get a buffer with data to transmit
                let buf = tx_chan.tx_buf().await;
                let len = buf.len();

                // Point hardware at this buffer
                mac.etxst().write(|w| w.set_etxst(buf.as_ptr() as u16));
                mac.etxln().write(|w| w.set_etxln(len as u16));

                // Trigger transmission
                mac.econ1().modify(|w| {
                    w.set_tx_rts(true);
                });

                // Wait for completion without TXIF (see `new()`). The frame's time
                // on the 10 Mbit/s wire is known: 0.8 us per byte including padding
                // to 60 bytes, FCS and preamble (the MAC adds the inter-frame gap
                // itself). Wake up TX_WAKE_EARLY_US before, which is about the timer
                // wake-up latency; TX_RTS clears when the transmission is done.
                let wire_us = (len.max(60) as u64 + 4 + 8) * 8 / 10;
                Timer::after_micros(wire_us.saturating_sub(TX_WAKE_EARLY_US)).await;
                while mac.econ1().read().tx_rts() {
                    Timer::after_micros(10).await;
                }

                // Tell the channel we're done with this buffer
                tx_chan.tx_done();
            }
        };

        let link_fut = async {
            let mut down_polls = 0u32;
            loop {
                let up = phy.link_up(&mut station_management);
                state_chan.set_link_state(if up {
                    LinkState::Up
                } else {
                    LinkState::Down
                });

                // The built-in PHY does not detect a reversed receive pair by
                // itself (its automatic polarity mode does not link either).
                // Like WCH's driver, alternate between positive and negative
                // polarity while the link stays down, and keep the one that
                // links. Auto-negotiation takes about 1 s.
                if up {
                    down_polls = 0;
                } else {
                    down_polls += 1;
                    if down_polls >= POLARITY_SWITCH_POLLS {
                        down_polls = 0;
                        let mdix = station_management.smi_read(0, PHY_REG_MDIX);
                        let negative = mdix & PHY_MDIX_PN_MASK == PHY_MDIX_PN_NEGATIVE;
                        let pn = if negative { 0 } else { PHY_MDIX_PN_NEGATIVE };
                        station_management.smi_write(0, PHY_REG_MDIX, (mdix & !PHY_MDIX_PN_MASK) | pn);
                        let bcr = station_management.smi_read(0, PHY_REG_BCR);
                        station_management.smi_write(0, PHY_REG_BCR, bcr | PHY_REG_BCR_AN_RESTART);
                    }
                }

                Timer::after_millis(500).await;
            }
        };

        embassy_futures::join::join3(rx_fut, tx_fut, link_fut).await;
        unreachable!()
    }
}

/// Get the factory-programmed MAC address from chip UID.
///
/// Each CH32 comes with a unique ID that can be used as MAC address.
/// This reads the 6 bytes from flash and returns them in wire order.
pub fn get_mac() -> [u8; 6] {
    const ADDRESS: *const u8 = (0x1FFFF7E8 + 5) as *const u8;
    let mut mac = [0u8; 6];
    unsafe {
        let mac_bytes: &[u8] = core::slice::from_raw_parts(ADDRESS, 6);
        mac.copy_from_slice(mac_bytes);
        mac.reverse();
    };
    // Clear multicast bit (LSB of first octet) for unicast
    mac[0] &= 0xFE;
    mac
}

/// Create a CH32V208 10M MAC+PHY driver for [`embassy-net`](https://crates.io/crates/embassy-net).
///
/// Returns a `(Runner, Device)` pair. The `Device` is passed to `embassy_net::new()`,
/// and the `Runner` must be spawned as a separate embassy task.
///
/// # Arguments
/// * `mac_addr` - The MAC address for this device (6 bytes)
/// * `core_freq` - Core clock frequency in Hz (must be 60MHz or 120MHz)
/// * `phy` - A PHY driver instance (e.g. `GenericSMI::new(0)`)
/// * `state` - A `&'static mut State<N_RX, N_TX>` providing packet buffers
pub async fn new<'d, P: PHY, const N_RX: usize, const N_TX: usize>(
    mac_addr: [u8; 6],
    core_freq: u32,
    phy: P,
    state: &'d mut State<N_RX, N_TX>,
) -> Result<(Runner<'d, P>, Device<'d>), InitError> {
    let mac = crate::pac::ETH;
    let extend = crate::pac::EXTEND;
    let rcc = crate::pac::RCC;

    // Configure ETH clock prescaler
    if core_freq == 60_000_000 {
        rcc.cfgr0()
            .modify(|w| w.set_ethpre(crate::pac::rcc::vals::Ethpre::DIV1));
    } else if core_freq == 120_000_000 {
        rcc.cfgr0()
            .modify(|w| w.set_ethpre(crate::pac::rcc::vals::Ethpre::DIV2));
    } else {
        return Err(InitError::CoreFrequencyInvalid);
    }

    // Enable built-in 10M PHY clock
    extend.ctr().modify(|w| w.set_eth_10m_en(true));

    // Reset RX and TX paths
    mac.econ1().write(|w| {
        w.set_rx_rst(true);
        w.set_tx_rst(true);
    });
    Timer::after_micros(1).await;

    // Release reset
    mac.econ1().write(|_w| {});
    Timer::after_micros(1).await;

    // Clear all interrupt flags
    mac.eir().write(|w| w.0 = 0xff);

    // Configure transmitter
    mac.econ2().write(|w| {
        w.set_tx(false); // enable transmitter (active low)
        w.set_rx_must(0b110); // reserved value, must be written 0b110
    });

    // Configure MAC layer
    mac.macon2().modify(|w| {
        w.set_padcfg(0b001); // pad short packets to 64 bytes
        w.set_txcrcen(true); // append CRC
        w.set_fuldpx(true); // full duplex
    });

    // Configure receive filter
    mac.erxfcon().write(|w| {
        w.set_en(false); // disable receive filtering
        w.set_bcen(true); // receive broadcast
        w.set_hten(true); // hash table filter
        w.set_mcen(true); // receive multicast
        w.set_mpen(true); // magic packet
        w.set_ucen(true); // unicast matching MAC
        w.set_crcen(true); // CRC check
    });

    // Enable the receiver once, before any transmission (see the RX loop).
    mac.econ1().modify(|w| w.set_rx_en(true));

    // Interrupts: RX only. Writing EIR drops any flag the MAC sets at the same
    // moment (seen for TXIF and RXIF), so acknowledging one source can lose
    // another. TX completion is taken from TX_RTS, the link state is polled.
    mac.eie().write(|w| {
        w.set_intie(true); // global interrupt enable
        w.set_rxie(true); // receive interrupt
        w.set_r_en50(true); // enable 50 Ohm termination resistor on RX
    });

    // Set MAC address (register order: maadr0 = byte 5, maadr5 = byte 0)
    mac.maadr0().write(|w| w.set_maadr(mac_addr[5]));
    mac.maadr1().write(|w| w.set_maadr(mac_addr[4]));
    mac.maadr2().write(|w| w.set_maadr(mac_addr[3]));
    mac.maadr3().write(|w| w.set_maadr(mac_addr[2]));
    mac.maadr4().write(|w| w.set_maadr(mac_addr[1]));
    mac.maadr5().write(|w| w.set_maadr(mac_addr[0]));

    fence(Ordering::SeqCst);

    // Enable ETH interrupt in PFIC
    unsafe {
        qingke::pfic::enable_interrupt(crate::pac::Interrupt::ETH as u8);
    };

    let station_management = EthernetStationManagement::<crate::peripherals::ETH>::new();

    // Create the channel-based driver
    let (runner, device) = ch::new(
        &mut state.ch_state,
        embassy_net_driver::HardwareAddress::Ethernet(mac_addr),
    );

    Ok((
        Runner {
            mac,
            phy,
            ch: runner,
            station_management,
        },
        device,
    ))
}

/// Ethernet station management (SMI/MDIO) driver.
struct EthernetStationManagement<T: Instance> {
    _peri: PhantomData<T>,
}

impl<T: Instance> EthernetStationManagement<T> {
    pub fn new() -> Self {
        Self { _peri: PhantomData }
    }
}

unsafe impl<T: Instance> StationManagement for EthernetStationManagement<T> {
    fn smi_read(&mut self, _phy_addr: u8, reg: u8) -> u16 {
        let mac = T::regs();

        mac.miregadr().write(|w| {
            w.set_miregadr(reg & 0x1F);
        });
        mac.mird().read().rd()
    }

    fn smi_write(&mut self, _phy_addr: u8, reg: u8, val: u16) {
        let mac = T::regs();

        mac.miwr().write(|w| {
            w.set_write(true);
            w.set_mirdl(reg & 0x1F);
            w.set_wr(val);
        });
    }
}
