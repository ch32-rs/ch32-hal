//! USBPD, USB Power Delivery
//!
//! Design:
//!
//! - CC Pins:
//! - UsbPdPhy: USBPD PHY layer
//! - UsbPdSniffer: USBPD Sniffer based on PHY layer, no transmit support
//! - [ ] UsbPdSink: USBPD Sink layer
//! - [ ] UsbPdSource: USBPD Source layer

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::AtomicBool;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::delay::DelayNs;
use pac::InterruptNumber;

use crate::gpio::Pull;
use crate::pac::usbpd::vals;
use crate::{interrupt, into_ref, pac, println, Peripheral, RccPeripheral};

#[derive(Debug)]
pub enum Error {
    Rejected,
    Timeout,
    CCNotConnected,
    NotSupported,
    HardReset,
    /// Unexpeted message type
    Protocol(u8),
    MaxRetry,
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let usbpd = T::REGS;

        let status = usbpd.status().read();

        println!("irq 0x{:02x}", status.0);

        if status.if_tx_end() {
            println!(">");
            //         T::REGS.port_cc1().modify(|w| w.set_cc_lve(false));
            // T::REGS.port_cc2().modify(|w| w.set_cc_lve(false));

            T::REGS.config().modify(|w| w.set_ie_tx_end(false));
        }

        if status.if_rx_act() {
            // println!("< {}", T::REGS.bmc_byte_cnt().read().bmc_byte_cnt());
            T::REGS.control().modify(|w| w.set_bmc_start(false)); // stop
            T::REGS.config().modify(|w| w.set_ie_rx_act(false));
        }

        if status.if_rx_reset() {
            T::REGS.config().modify(|w| w.set_ie_rx_reset(false));
            crate::println!("TODO: reset");
        }

        if status.buf_err() {
            crate::println!("TODO: buf_err");
        }

        T::REGS.status().write_value(status);

        // Wake the task to clear and re-enabled interrupts.
        T::state().waker.wake();
    }
}

pub struct UsbPdPhy<'d, T: Instance> {
    _marker: PhantomData<&'d mut T>,
    cc1: vals::CcSel,
    cc2: vals::CcSel,
}

impl<'d, T: Instance> UsbPdPhy<'d, T> {
    /// Create a new SPI driver.
    pub fn new(
        _peri: impl Peripheral<P = T> + 'd,
        cc1: impl Peripheral<P = impl CcPin<T>> + 'd,
        cc2: impl Peripheral<P = impl CcPin<T>> + 'd,
    ) -> Result<Self, Error> {
        into_ref!(cc1, cc2);

        assert!(cc1.port_sel() != cc2.port_sel(), "CC1 and CC2 should be different");

        #[allow(unused)]
        let afio = crate::pac::AFIO;

        T::enable_and_reset();

        cc1.set_as_input(Pull::None);
        cc2.set_as_input(Pull::None);

        // PD 引脚 PC14/PC15 高阈值输入模式
        // PD 收发器 PHY 上拉限幅配置位: USBPD_PHY_V33
        #[cfg(ch32x0)]
        afio.ctlr().modify(|w| {
            w.set_usbpd_in_hvt(true);
            w.set_usbpd_phy_v33(true);
        });
        #[cfg(ch32l1)]
        afio.cr().modify(|w| {
            w.set_usbpd_in_hvt(true);
        });

        T::REGS.config().write(|w| {
            w.set_pd_dma_en(true);
            //    w.set_pd_filt_en(true);
            //  w.set_pd_rst_en(true);
        });
        T::REGS.status().write(|w| w.0 = 0b111111_00); // write 1 to clear

        // pd_phy_reset

        T::port_cc_reg(cc1.port_sel()).write(|w| w.set_cc_ce(vals::PortCcCe::V0_66));
        T::port_cc_reg(cc2.port_sel()).write(|w| w.set_cc_ce(vals::PortCcCe::V0_66));

        let mut this = Self {
            _marker: PhantomData,
            cc1: cc1.port_sel(),
            cc2: cc2.port_sel(),
        };
        this.detect_cc()?;

        Ok(this)
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        T::enable_and_reset();

        T::REGS.config().write(|w| {
            w.set_pd_dma_en(true);
            //    w.set_pd_filt_en(true);
        });
        T::REGS.status().write(|w| w.0 = 0b111111_00); // write 1 to clear

        T::port_cc_reg(self.cc1).modify(|w| w.set_cc_lve(false));
        T::port_cc_reg(self.cc2).modify(|w| w.set_cc_lve(false));

        self.detect_cc()?;

        // pd_phy_reset
        T::port_cc_reg(self.cc1).write(|w| w.set_cc_ce(vals::PortCcCe::V0_66));
        T::port_cc_reg(self.cc2).write(|w| w.set_cc_ce(vals::PortCcCe::V0_66));

        Ok(())
    }

    fn detect_cc(&mut self) -> Result<(), Error> {
        // CH32X035 has no internal CC pull down support
        // The detection voltage is 0.22V, sufficient to detect the default power(500mA/900mA)

        T::port_cc_reg(self.cc1).modify(|w| w.set_cc_ce(vals::PortCcCe::V0_22));
        embassy_time::Delay.delay_us(2);

        if T::port_cc_reg(self.cc1).read().pa_cc_ai() {
            // CC1 is connected
            T::REGS.config().modify(|w| w.set_cc_sel(vals::CcSel::CC1));

            crate::println!("CC1 connected");
            Ok(())
        } else {
            T::port_cc_reg(self.cc2).modify(|w| w.set_cc_ce(vals::PortCcCe::V0_22));
            embassy_time::Delay.delay_us(2);

            if T::port_cc_reg(self.cc2).read().pa_cc_ai() {
                // CC2 is connected
                T::REGS.config().modify(|w| w.set_cc_sel(vals::CcSel::CC2));
                crate::println!("CC2 connected");
                Ok(())
            } else {
                crate::println!("CC not connected");

                Err(Error::CCNotConnected)
            }
        }
    }

    fn enable_rx_interrupt(&mut self) {
        T::REGS.config().modify(|w| {
            w.set_ie_rx_act(true);
            w.set_ie_rx_reset(true);
            w.set_ie_tx_end(false);
        });
    }

    fn enable_tx_interrupt(&mut self) {
        T::REGS.config().modify(|w| {
            w.set_ie_rx_act(false);
            w.set_ie_rx_reset(true);
            w.set_ie_tx_end(true);
        });
    }

    /// Receives a PD message into the provided buffer.
    ///
    /// Returns the number of received bytes or an error.
    pub async fn receive(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        // set rx mode
        // println!("before clear {}", T::REGS.status().read().0);
        T::REGS.config().modify(|w| w.set_pd_all_clr(true));
        // println!("after clear0 {}", T::REGS.status().read().0);
        T::REGS.config().modify(|w| w.set_pd_all_clr(false));
        //        println!("after clear1 {}", T::REGS.status().read().0);

        T::REGS.dma().write_value((buf.as_mut_ptr() as u32 & 0xFFFF) as u16);

        T::REGS.control().modify(|w| w.set_pd_tx_en(false)); // RX
        T::REGS
            .bmc_clk_cnt()
            .modify(|w| w.set_bmc_clk_cnt(calc_bmc_clk_for_rx()));

        self.enable_rx_interrupt();
        T::REGS.control().modify(|w| w.set_bmc_start(true));

        poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            if !T::REGS.config().read().ie_rx_reset() {
                return Poll::Ready(Err(Error::HardReset));
            }

            if !T::REGS.config().read().ie_rx_act() {
                match T::REGS.status().read().bmc_aux() {
                    vals::BmcAux::SOP0 => {
                        // println!("ctrl {}", T::REGS.control().read().0);
                        // println!("=> {}", T::REGS.bmc_byte_cnt().read().bmc_byte_cnt());
                        return Poll::Ready(Ok(T::REGS.bmc_byte_cnt().read().bmc_byte_cnt() as usize));
                    }
                    vals::BmcAux::SOP1 => {
                        // hard reset
                        return Poll::Ready(Err(Error::HardReset));
                    }
                    _ => {
                        self.enable_rx_interrupt();
                        return Poll::Pending;
                    }
                }
            }
            Poll::Pending
        })
        .await
    }

    pub fn blocking_receive(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        // set rx mode
        // println!("before clear {}", T::REGS.status().read().0);
        T::REGS.config().modify(|w| w.set_pd_all_clr(true));
        // println!("after clear0 {}", T::REGS.status().read().0);
        T::REGS.config().modify(|w| w.set_pd_all_clr(false));
        //        println!("after clear1 {}", T::REGS.status().read().0);

        T::REGS.dma().write_value((buf.as_mut_ptr() as u32 & 0xFFFF) as u16);

        T::REGS.control().modify(|w| w.set_pd_tx_en(false)); // RX
        T::REGS
            .bmc_clk_cnt()
            .modify(|w| w.set_bmc_clk_cnt(calc_bmc_clk_for_rx()));

        self.enable_rx_interrupt();
        unsafe {
            qingke::pfic::disable_interrupt(interrupt::USBPD.number() as _);
        }
        println!("begin blocking recv");

        T::REGS.control().modify(|w| w.set_bmc_start(true));

        while !T::REGS.status().read().if_rx_act() {
            // println!("wait");
        }

        unsafe {
            qingke::pfic::enable_interrupt(interrupt::USBPD.number() as _);
        }

        Ok(10)
    }

    fn transmit(&mut self, sop: u8, buf: &[u8]) -> Result<(), Error> {
        T::port_cc_reg(T::REGS.config().read().cc_sel()).modify(|w| w.set_cc_lve(true));

        T::REGS
            .bmc_clk_cnt()
            .write(|w| w.set_bmc_clk_cnt(calc_bmc_clk_for_tx()));

        if buf.is_empty() {
            T::REGS.dma().write_value(0);
        } else {
            T::REGS.dma().write_value(buf.as_ptr() as u16);
        }

        T::REGS.tx_sel().write(|w| w.0 = sop);

        T::REGS.bmc_tx_sz().write(|w| w.set_bmc_tx_sz(buf.len() as _));
        T::REGS.control().modify(|w| w.set_pd_tx_en(true)); // TX

        T::REGS.status().write(|w| w.0 = 0b11111100);

        T::REGS.control().modify(|w| w.set_bmc_start(true));

        Ok(())
    }

    /// Transmit a hard reset.
    pub async fn transmit_hardreset(&mut self) {
        const TX_SEL_HARD_RESET: u8 = 0b10_10_10_01;

        // self.enable_tx_interrupt();
        // self.transmit(TX_SEL_HARD_RESET, &[])?;
        // send_phy_empty_playload

        T::port_cc_reg(T::REGS.config().read().cc_sel()).modify(|w| w.set_cc_lve(true));

        T::REGS
            .bmc_clk_cnt()
            .write(|w| w.set_bmc_clk_cnt(calc_bmc_clk_for_tx()));

        T::REGS.dma().write_value(0);
        T::REGS.tx_sel().write(|w| w.0 = TX_SEL_HARD_RESET);
        T::REGS.bmc_byte_cnt().write(|w| w.set_bmc_byte_cnt(0));

        T::REGS.control().modify(|w| w.set_pd_tx_en(true)); // TX

        T::REGS.status().write(|w| w.0 = 0b11111100);

        T::REGS.control().modify(|w| w.set_bmc_start(true));

        /*

        poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            let config = T::REGS.config().read();
            if !config.ie_tx_end() {
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;

        // T::REGS.port_cc1().modify(|w| w.set_cc_lve(false));
        // T::REGS.port_cc2().modify(|w| w.set_cc_lve(false));

        //        T::REGS.port_cc1().write(|w| w.set_cc_ce(vals::PortCcCe::V0_66));
        //      T::REGS.port_cc2().write(|w| w.set_cc_ce(vals::PortCcCe::V0_66));

        Ok(())
        */
    }
}

struct State {
    waker: AtomicWaker,
    // Inverted logic for a default state of 0 so that the data goes into the .bss section.
    drop_not_ready: AtomicBool,
}

impl State {
    pub const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            drop_not_ready: AtomicBool::new(false),
        }
    }
}

trait SealedInstance {
    const REGS: crate::pac::usbpd::Usbpd;

    fn state() -> &'static State;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccPeripheral {
    type Interrupt: crate::interrupt::typelevel::Interrupt;

    #[allow(dead_code)]
    fn port_cc_reg(cc: vals::CcSel) -> pac::common::Reg<pac::usbpd::regs::PortCc, pac::common::RW> {
        match cc {
            vals::CcSel::CC1 => Self::REGS.port_cc(0),
            vals::CcSel::CC2 => Self::REGS.port_cc(2),
            #[cfg(ch641)]
            vals::CcSel::CC3 => Self::REGS.port_cc(3),
            #[allow(unreachable_patterns)]
            _ => panic!("Invalid CC"),
        }
    }
}

// catch GLOBAL irq
foreach_interrupt!(
    ($inst:ident, usbpd, USBPD, GLOBAL, $irq:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            const REGS: crate::pac::usbpd::Usbpd = crate::pac::$inst;

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
);

pub trait CcPin<T: Instance>: crate::gpio::Pin {
    fn port_sel(&self) -> pac::usbpd::vals::CcSel;
}

#[cfg(ch32x0)]
mod _cc_pin_ch32x0 {
    use super::*;

    impl CcPin<crate::peripherals::USBPD> for crate::peripherals::PC14 {
        #[inline(always)]
        fn port_sel(&self) -> pac::usbpd::vals::CcSel {
            pac::usbpd::vals::CcSel::CC1
        }
    }
    impl CcPin<crate::peripherals::USBPD> for crate::peripherals::PC15 {
        #[inline(always)]
        fn port_sel(&self) -> pac::usbpd::vals::CcSel {
            pac::usbpd::vals::CcSel::CC2
        }
    }
}

#[cfg(ch32l1)]
mod _cc_pin_ch32l1 {
    use super::*;

    impl CcPin<crate::peripherals::USBPD> for crate::peripherals::PB6 {
        #[inline(always)]
        fn port_sel(&self) -> pac::usbpd::vals::CcSel {
            pac::usbpd::vals::CcSel::CC1
        }
    }
    impl CcPin<crate::peripherals::USBPD> for crate::peripherals::PB7 {
        #[inline(always)]
        fn port_sel(&self) -> pac::usbpd::vals::CcSel {
            pac::usbpd::vals::CcSel::CC2
        }
    }
}

#[cfg(ch641)]
mod _cc_pin_ch641 {
    use super::*;

    impl CcPin<crate::peripherals::USBPD> for crate::peripherals::PB0 {
        #[inline(always)]
        fn port_sel(&self) -> pac::usbpd::vals::CcSel {
            pac::usbpd::vals::CcSel::CC1
        }
    }
    impl CcPin<crate::peripherals::USBPD> for crate::peripherals::PB1 {
        #[inline(always)]
        fn port_sel(&self) -> pac::usbpd::vals::CcSel {
            pac::usbpd::vals::CcSel::CC2
        }
    }
    impl CcPin<crate::peripherals::USBPD> for crate::peripherals::PB9 {
        #[inline(always)]
        fn port_sel(&self) -> pac::usbpd::vals::CcSel {
            pac::usbpd::vals::CcSel::CC3
        }
    }
}

#[inline]
fn calc_bmc_clk_for_tx() -> u16 {
    (crate::rcc::clocks().hclk.0 / 1_000_000 * 80 / 48 - 1) as u16
}

#[inline]
fn calc_bmc_clk_for_rx() -> u16 {
    (crate::rcc::clocks().hclk.0 / 1_000_000 * 120 / 48 - 1) as u16
}
