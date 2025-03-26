#![allow(async_fn_in_trait)]
#![warn(missing_docs)]

use core::marker::PhantomData;
use core::sync::atomic::{fence, Ordering};

use embassy_futures::join::join;
use embassy_net_driver_channel::driver::LinkState;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use {ch32_metapac as pac, embassy_net_driver_channel as ch};

use crate::eth::{Instance, StationManagement, PHY};
use crate::interrupt;

// If you change this update the docs of State
const MTU: usize = 1514;

/// Type alias for the embassy-net driver.
pub type Device<'d> = ch::Device<'d, MTU>;

/// Internal state for the embassy-net integration.
///
/// The two generic arguments `N_RX` and `N_TX` set the size of the receive and
/// send packet queue. With a the ethernet MTU of _1514_ this takes up `N_RX +
/// NTX * 1514` bytes. While setting these both to 1 is the minimum this might
/// hurt performance as a packet can not be received while processing another.
///
/// # Warning
/// On devices with a small amount of ram (think ~64k) watch out with the size
/// of there parameters. They will quickly use too much RAM.
pub struct State<const N_RX: usize, const N_TX: usize> {
    ch_state: ch::State<MTU, N_RX, N_TX>,
}

impl<const N_RX: usize, const N_TX: usize> State<N_RX, N_TX> {
    /// Create a new `State`.
    pub const fn new() -> Self {
        Self {
            ch_state: ch::State::new(),
        }
    }
}

/// Error type when initializing a new Wiznet device
#[derive(Debug)]
pub enum InitError {
    /// The core frequency isn't 60 or 120MHz
    CoreFrequencyInvalid,
}

// Interrupt interacts with Runner through 3 event channels
static TX_CH: Channel<CriticalSectionRawMutex, (), 2> = Channel::new();
static RX_CH: Channel<CriticalSectionRawMutex, u16, 1> = Channel::new();
static LINK_CH: Channel<CriticalSectionRawMutex, (), 2> = Channel::new();

#[interrupt]
unsafe fn ETH() {
    let mac = pac::ETH;

    mac.eie().modify(|w| {
        w.set_intie(false);
    });
    fence(Ordering::SeqCst);
    let flags = mac.eir().read();
    mac.eir().write_value(flags);

    // reception
    if flags.rxif() {
        let sender = RX_CH.sender();
        if let Err(_) = sender.try_send(pac::ETH.erxln().read().0) {
            // there is no room in the buffer for this packet, consider it lost and restart
            // reception immediately with same buffer. Otherwise, reception is started from Runner.
            mac.econ1().write(|w| {
                w.set_rx_en(true);
            });
        }
    }

    // transmission done
    if flags.txif() {
        let sender = TX_CH.sender();
        let _ = sender.try_send(());
    }

    // link changed
    if flags.linkif() {
        let sender = LINK_CH.sender();
        let _ = sender.try_send(());
    }
    fence(Ordering::SeqCst);
    mac.eie().modify(|w| {
        w.set_intie(true);
    });
}

// lock to the SMI control registers
static SMI_LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

/// Background runner for the driver.
///
/// You must call `.run()` in a background task for the driver to operate.
pub struct Runner<'d, P: PHY> {
    mac: pac::eth::Eth,
    phy: P,
    ch: ch::Runner<'d, MTU>,
    station_management: EthernetStationManagement<crate::peripherals::ETH>,
}

/// You must call this in a background task for the driver to operate.
impl<'d, P: PHY> Runner<'d, P> {
    /// Run the driver.
    pub async fn run(mut self) -> ! {
        let (state_chan, mut rx_chan, mut tx_chan) = self.ch.split();
        let tx_receiver = TX_CH.receiver();
        let rx_receiver = RX_CH.receiver();
        let link_receiver = LINK_CH.receiver();

        // TX and RX paths use entirely seperate registers and channels, which means that they can
        // fully work in parallel
        loop {
            join(
                join(
                    async {
                        loop {
                            let buf = rx_chan.rx_buf().await;
                            // start receive at buf
                            self.mac.mamxfl().write(|w| w.set_mamxfl(buf.len() as u16));
                            self.mac.erxst().write(|w| w.set_erxst(buf.as_ptr() as u16));
                            self.mac.macon1().write(|w| w.set_marxen(true));
                            self.mac.econ1().write(|w| {
                                w.set_rx_en(true);
                            });
                            rx_receiver.receive().await;
                            let len = self.mac.erxln().read().0;
                            rx_chan.rx_done(len as usize);
                        }
                    },
                    async {
                        loop {
                            let buf = tx_chan.tx_buf().await;
                            // start send buf
                            let address: u16 = buf.as_ptr() as u16;
                            let len: u16 = buf.len() as u16;
                            self.mac.etxst().write(|w| w.set_etxst(address));
                            self.mac.etxln().write(|w| w.set_etxln(len));
                            self.mac.econ1().modify(|w| {
                                w.set_tx_rts(true); // start transmit
                            });
                            tx_receiver.receive().await;
                            tx_chan.tx_done();
                        }
                    },
                ),
                async {
                    loop {
                        link_receiver.receive().await;
                        if self.phy.link_up(&mut self.station_management) {
                            state_chan.set_link_state(LinkState::Up);
                        } else {
                            state_chan.set_link_state(LinkState::Down);
                        }
                    }
                },
            )
            .await;
        }
    }
}

/// Get the factory-programmed MAC
///
/// Each CH32 comes with an Unique ID which can be used as MAC address.
/// This returns the MAC bytes, in the same order as they are transmitted on the wires.
pub fn get_mac() -> [u8; 6] {
    const ADDRESS: *const u8 = (0x1FFFF7E8 + 5) as *const u8;
    let mut mac = [0u8; 6];
    unsafe {
        let mac_bytes: &[u8] = core::slice::from_raw_parts(ADDRESS, 6);
        mac.copy_from_slice(mac_bytes);
        mac.reverse();
    };
    mac[0] = mac[0] & 0xFE; // FIXME! smoltcp thinks the MAC is multicast otherwise...
    mac
}

/// Create a CH32 10M MAC+PHY driver for [`embassy-net`](https://crates.io/crates/embassy-net).
///
/// This returns two structs:
/// - a `Device` that you must pass to the `embassy-net` stack.
/// - a `Runner`. You must call `.run()` on it in a background task.
pub async fn new<'a, const N_RX: usize, const N_TX: usize, P: PHY>(
    mac_addr: [u8; 6],
    state: &'a mut State<N_RX, N_TX>,
    core_freq: u32,
    phy: P,
) -> Result<(Device<'a>, Runner<'a, P>), InitError> {
    let mac = pac::ETH;
    let extend = pac::EXTEND;
    let rcc = pac::RCC;
    let station_management = EthernetStationManagement::new();

    if core_freq == 60_000_000 {
        rcc.cfgr0().modify(|w| w.set_ethpre(pac::rcc::vals::Ethpre::DIV1));
    } else if core_freq == 120_000_000 {
        // divide core clk by 2
        rcc.cfgr0().modify(|w| w.set_ethpre(pac::rcc::vals::Ethpre::DIV2));
    } else {
        return Err(InitError::CoreFrequencyInvalid);
    }
    extend.ctr().modify(|w| w.set_eth_10m_en(true)); // enable MAC+PHY clock

    mac.econ1().write(|w| {
        w.set_rx_rst(true); // reset MAC's RX path
        w.set_rx_rst(true); // reset MAC's TX path
    });
    Timer::after_micros(1).await;
    mac.econ1().write(|w| {
        // no reset
        w.set_rx_en(true); // receive enable
    });
    Timer::after_micros(1).await;
    mac.eir().write(|w| w.0 = 0xff); // clear all interrupt flags
    mac.econ2().write(|w| {
        w.set_tx(false); // enable MAC transmitter
        w.set_rx_must(0b110); // reserved value, must be written 0b110
    });
    mac.macon2().modify(|w| {
        w.set_padcfg(0b001); // short packet padding (0-64B)
        w.set_txcrcen(true); // CRC insertion (4B)
        w.set_fuldpx(true); // full fuplex
    });
    mac.erxfcon().write(|w| {
        w.set_en(false); // disable receive filtering
        w.set_bcen(true); // receive broadcast
        w.set_hten(true); // receive packets with matching  hash
        w.set_mcen(true); // receive multicast
        w.set_mpen(true); // receive magic packet
        w.set_ucen(true); // receive packets with matching MAC
        w.set_crcen(true); // receive packets with bad CRC
    });

    mac.eie().write(|w| {
        w.set_intie(true); // global interrupt enable
        w.set_rxie(true); // receive interrupt
        w.set_linkie(true); // link change interrupt
        w.set_txie(true); // transmit done interrupt
        w.set_r_en50(true); // enable 50 Ohms termination resistor on RX
    });

    // Set MAC address filter
    mac.maadr0().write(|w| w.set_maadr(mac_addr[5]));
    mac.maadr1().write(|w| w.set_maadr(mac_addr[4]));
    mac.maadr2().write(|w| w.set_maadr(mac_addr[3]));
    mac.maadr3().write(|w| w.set_maadr(mac_addr[2]));
    mac.maadr4().write(|w| w.set_maadr(mac_addr[1]));
    mac.maadr5().write(|w| w.set_maadr(mac_addr[0]));

    fence(Ordering::SeqCst);
    unsafe {
        qingke::pfic::enable_interrupt(pac::Interrupt::ETH as u8);
    };

    let (runner, device) = ch::new(&mut state.ch_state, ch::driver::HardwareAddress::Ethernet(mac_addr));

    Ok((
        device,
        Runner {
            ch: runner,
            mac,
            phy,
            station_management,
        },
    ))
}

/// Ethernet SMI driver.
pub struct EthernetStationManagement<T: Instance> {
    peri: PhantomData<T>,
}

impl<T: Instance> EthernetStationManagement<T> {
    pub fn new() -> Self {
        Self { peri: PhantomData {} }
    }
}

unsafe impl<T: Instance> StationManagement for EthernetStationManagement<T> {
    fn smi_read(&mut self, _phy_addr: u8, reg: u8) -> u16 {
        let mac = T::regs();
        let _ = SMI_LOCK.lock();

        mac.miregadr().write(|w| {
            w.set_miregadr(reg & 0x1F);
        });
        mac.mird().read().rd()
    }

    fn smi_write(&mut self, _phy_addr: u8, reg: u8, val: u16) {
        let mac = T::regs();
        let _ = SMI_LOCK.lock();

        mac.miwr().write(|w| {
            w.set_write(true);
            w.set_mirdl(reg & 0x1F);
            w.set_wr(val);
        });
    }
}
