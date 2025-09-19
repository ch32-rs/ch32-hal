//! USBHS device mode peripheral driver
//! Note that this currently only implements device mode
//!
//! <div class="warning">
//! There's a lot of TODOs and panics where things are not implemented
//! </div>
//!
//! The same endpoint index cannot be used for both In/Out transfers.
//! This is a limitation of the implementation and should be fixed.
//!
//! List of things that is tested
//! Untested but expected to work items are noted as well
//!
//! Control Pipe:
//! - [x] Receive `SETUP` packet (Host -> Dev)
//! - [x] Send `IN` packet (Dev -> Host).
//! - [x] Receive `OUT` packet (Host -> Dev).
//!
//! Other Endpoints:
//! - [x] Interrupt Out
//! - [x] Interrupt In
//! - [ ] Bulk Out (Expected to work but not tested)
//! - [ ] Bulk In (Expected to work but not tested)
//! - [ ] Isochronous Out (Does not work)
//! - [ ] Isochronous In (Does not work)
//!
//! Other Features:
//! - [ ] Set endpoint stall
//! - [ ] Get endpoint stall status
//! - [ ] Remote wakeup
//!

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use bitmaps::Bitmap;
use ch32_metapac::usbhs::regs::{EpBufMod, EpConfig, EpType};
use ch32_metapac::usbhs::vals::{EpRxResponse, EpTog, EpTxResponse, SpeedType, UsbToken};
use control::ControlPipe;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointInfo, EndpointType, Event, Unsupported,
};
use endpoint::Endpoint;

use crate::gpio::{AFType, Speed};
use crate::interrupt::typelevel::Interrupt;
use crate::usb::{Dir, EndpointBufferAllocator, EndpointData, EndpointDataBuffer, In, Out};
use crate::{interrupt, Peripheral};

pub mod control;
mod endpoint;

const MAX_NR_EP: usize = 16;
const EP_MAX_PACKET_SIZE: u16 = 64;

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;
static EP_WAKERS: [AtomicWaker; MAX_NR_EP] = [NEW_AW; MAX_NR_EP];

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let flag = r.int_fg().read();

        assert!(!flag.fifo_ov());

        if flag.bus_rst() {
            //mask the interrupt and let the main thread handle it
            r.int_en().modify(|w| w.set_bus_rst(false));
            BUS_WAKER.wake();
        };
        if flag.suspend() {
            //mask the interrupt and let the main thread handle it
            r.int_en().modify(|w| w.set_suspend(false));
            BUS_WAKER.wake();
        };
        if flag.setup_act() {
            //mask the interrupt and let the main thread handle it
            r.int_en().modify(|w| w.set_setup_act(false));
            EP_WAKERS[0].wake();
        };
        if flag.transfer() {
            let status = r.int_st().read();

            match status.token() {
                UsbToken::OUT => {
                    if status.tog_ok() {
                        EP_WAKERS[status.endp() as usize].wake();
                        r.int_en().modify(|w| w.set_transfer(false));
                    } else {
                        // If `tog` is wrong, keep NAK and wait for a "resend"
                        r.int_fg().write(|v| v.set_transfer(true));
                    }
                }
                UsbToken::IN => {
                    EP_WAKERS[status.endp() as usize].wake();
                    r.int_en().modify(|w| w.set_transfer(false));
                }
                // SETUP
                t @ (UsbToken::SOF | UsbToken::SETUP) => {
                    panic!("received token {}", t as u8)
                }
            }
        }
        if flag.hst_sof() {
            r.int_fg().write(|v| v.set_hst_sof(true));
        }
    }
}

pub struct WakeupInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::WakeupInterrupt> for WakeupInterruptHandler<T> {
    unsafe fn on_interrupt() {
        todo!()
    }
}

pub struct Driver<'d, T: Instance, const NR_EP: usize> {
    phantom: PhantomData<&'d T>,
    allocator: EndpointBufferAllocator<'d, NR_EP>,
    allocated: Bitmap<MAX_NR_EP>,
}

impl<'d, T: Instance, const NR_EP: usize> Driver<'d, T, NR_EP> {
    pub fn new(
        _peri: impl Peripheral<P = T> + 'd,
        _irqs: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>
            + interrupt::typelevel::Binding<T::WakeupInterrupt, WakeupInterruptHandler<T>>
            + 'd,
        dp: impl Peripheral<P = impl DpPin<T, 0> + 'd>,
        dm: impl Peripheral<P = impl DmPin<T, 0> + 'd>,
        ep_buffer: &'d mut [EndpointDataBuffer; NR_EP],
    ) -> Self {
        assert!(ep_buffer.len() > 0);
        let dp = dp.into_ref();
        let dm = dm.into_ref();

        dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
        dm.set_as_af_output(AFType::OutputPushPull, Speed::High);

        T::enable_and_reset();

        let allocator = EndpointBufferAllocator::new(ep_buffer);

        let r = T::regs();
        let h = T::hregs();

        r.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });

        // Sleep for 10uS from WCH C code
        embassy_time::block_for(embassy_time::Duration::from_micros(10));

        // Following WCH C code.... unclear why clr_all is not also cleared here
        r.ctrl().modify(|w| w.set_reset_sie(false));
        h.ctrl().write(|w| w.set_phy_suspendm(true));

        r.ctrl().write(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            // TODO: HS, Configurable?
            w.set_speed_type(SpeedType::HIGHSPEED);
        });

        r.int_en().write(|w| {
            w.set_dev_nak(false);
            w.set_fifo_ov(true);
            w.set_setup_act(true);
            w.set_suspend(true);
            w.set_bus_rst(true);
            w.set_transfer(true);
        });

        Self {
            phantom: PhantomData,
            allocator,
            allocated: Bitmap::new(),
        }
    }

    fn find_free_ep_address(&self, _dir: Direction) -> Result<u8, EndpointAllocError> {
        // Skip index 0 which is reserved for control endpoint
        Ok(self.allocated.next_false_index(0).ok_or(EndpointAllocError)? as u8)
    }

    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
        dir: Direction,
    ) -> Result<Endpoint<'d, T, D>, EndpointAllocError> {
        let addr = match ep_addr {
            Some(addr) => {
                // Use the provided endpoint address
                if addr.direction() != dir {
                    return Err(EndpointAllocError);
                }

                let ep_num = addr.index();
                if ep_num >= MAX_NR_EP {
                    return Err(EndpointAllocError);
                }

                // Check if this endpoint is already allocated
                if self.allocated.get(ep_num) {
                    return Err(EndpointAllocError);
                }

                self.allocated.set(ep_num, true);
                addr
            }
            None => {
                // Find a free endpoint address
                let ep_addr = self.find_free_ep_address(dir)?;
                let addr = EndpointAddress::from_parts(ep_addr as usize, dir);

                // Mark as allocated
                self.allocated.set(ep_addr as usize, true);

                addr
            }
        };

        let data = self.allocator.alloc_endpoint(max_packet_size)?;

        Ok(Endpoint::new(
            EndpointInfo {
                addr,
                ep_type,
                max_packet_size,
                interval_ms,
            },
            data,
        ))
    }
}

impl<'d, T: Instance, const NR_EP: usize> embassy_usb_driver::Driver<'d> for Driver<'d, T, NR_EP> {
    type EndpointOut = Endpoint<'d, T, Out>;
    type EndpointIn = Endpoint<'d, T, In>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms, Direction::Out)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms, Direction::In)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let d = T::dregs();

        let ep0_buf = self.allocator.alloc_endpoint(control_max_packet_size).unwrap();
        d.ep0_dma().write_value(ep0_buf.addr() as u32);
        d.ep_max_len(0).write(|w| w.set_len(control_max_packet_size));
        d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
        d.ep_tx_ctrl(0).write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));

        let ep0 = Endpoint::new(
            EndpointInfo {
                // todo fix in embassy usb driver ep_addr should be u8 with top bit unset
                addr: EndpointAddress::from_parts(0, Direction::Out),
                ep_type: EndpointType::Control,
                max_packet_size: EP_MAX_PACKET_SIZE,
                interval_ms: 0,
            },
            ep0_buf,
        );

        critical_section::with(|_| {
            T::Interrupt::unpend();
            unsafe { T::Interrupt::enable() };
        });

        (
            Bus {
                _phantom: PhantomData,
                fake_power_on: false,
            },
            ControlPipe::new(ep0),
        )
    }
}

pub struct Bus<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    fake_power_on: bool,
}

impl<'d, T: Instance> Bus<'d, T> {
    fn bus_reset(&mut self) {
        let regs = T::regs();
        let d = T::dregs();

        // Reset device address
        regs.dev_ad().write(|v| {
            v.set_addr(0);
        });

        // Reset endpoint state on bus reset
        // EP0 get ACK/NAK so it can recieve setup
        d.ep_rx_ctrl(0).write(|v| v.set_mask_uep_r_res(EpRxResponse::ACK));
        d.ep_tx_ctrl(0).write(|v| v.set_mask_uep_t_res(EpTxResponse::NAK));

        // Mark all other EPs as NAK
        for i in 1..=15 {
            d.ep_tx_ctrl(i).write(|v| v.set_mask_uep_t_res(EpTxResponse::NAK));
            d.ep_rx_ctrl(i).write(|v| v.set_mask_uep_r_res(EpRxResponse::NAK));
        }

        // Disable all endpoints [1, 15]
        d.ep_config().write_value(EpConfig::default());
        d.ep_type().write_value(EpType::default());
        d.ep_buf_mod().write_value(EpBufMod::default());
    }
}

impl<'d, T: Instance> embassy_usb_driver::Bus for Bus<'d, T> {
    async fn enable(&mut self) {
        critical_section::with(|_| T::regs().ctrl().modify(|v| v.set_dev_pu_en(true)));
    }

    async fn disable(&mut self) {}

    async fn poll(&mut self) -> Event {
        // TODO: VBUS detection
        if !self.fake_power_on {
            self.fake_power_on = true;
            return Event::PowerDetected;
        }
        poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());

            let r = T::regs();
            let flag = r.int_fg().read();

            if flag.bus_rst() {
                self.bus_reset();
                r.int_fg().write(|w| w.set_bus_rst(true));
                critical_section::with(|_| {
                    r.int_en().modify(|w| w.set_bus_rst(true));
                });
                Poll::Ready(Event::Reset)
            } else if flag.suspend() {
                let mis_st = r.mis_st().read();
                r.int_fg().write(|w| w.set_suspend(true));
                critical_section::with(|_| {
                    r.int_en().modify(|w| w.set_suspend(true));
                });
                if mis_st.suspend() {
                    Poll::Ready(Event::Suspend)
                } else {
                    Poll::Ready(Event::Resume)
                }
            } else {
                Poll::Pending
            }
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let index = ep_addr.index();

        critical_section::with(|_| match ep_addr.direction() {
            Direction::Out => {
                T::dregs().ep_config().modify(|v| v.set_r_en(index - 1, enabled));
                T::dregs().ep_rx_ctrl(index).write(|v| {
                    v.set_mask_uep_r_tog(EpTog::DATA0);
                    v.set_mask_uep_r_res(EpRxResponse::NAK);
                    v.set_r_tog_auto(false);
                });
            }
            Direction::In => {
                T::dregs().ep_config().modify(|v| v.set_t_en(index - 1, enabled));
                T::dregs().ep_tx_ctrl(index).write(|v| {
                    v.set_mask_uep_t_tog(EpTog::DATA0);
                    v.set_mask_uep_t_res(EpTxResponse::NAK);
                    v.set_t_tog_auto(false);
                });
            }
        });
        EP_WAKERS[ep_addr.index()].wake();
    }

    fn endpoint_set_stalled(&mut self, _ep_addr: EndpointAddress, _stalled: bool) {
        todo!()
    }

    fn endpoint_is_stalled(&mut self, _ep_addr: EndpointAddress) -> bool {
        todo!()
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        todo!()
    }
}

/// USB endpoint.
trait SealedInstance: crate::peripheral::RccPeripheral {
    fn regs() -> crate::pac::usbhs::Usb;
    fn dregs() -> crate::pac::usbhs::Usbd;
    fn hregs() -> crate::pac::usbhs::Usbh;
}

/// UsbHs peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    /// Regular interrupt for this instance
    type Interrupt: interrupt::typelevel::Interrupt;
    /// Wakeup interrupt for this instance
    type WakeupInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (usbhs, $inst:ident) => {
        use crate::peripherals;
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::usbhs::Usb {
                crate::pac::$inst
            }

            fn dregs() -> crate::pac::usbhs::Usbd {
                unsafe {
                    crate::pac::usbhs::Usbd::from_ptr(crate::pac::$inst.as_ptr())
                }
            }

            fn hregs() -> crate::pac::usbhs::Usbh {
                unsafe {
                    crate::pac::usbhs::Usbh::from_ptr(crate::pac::$inst.as_ptr())
                }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
            type WakeupInterrupt = crate::_generated::peripheral_interrupts::$inst::WAKEUP;
        }
    };
);

pin_trait!(DmPin, Instance);
pin_trait!(DpPin, Instance);
