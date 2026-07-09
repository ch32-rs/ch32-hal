//! USB_FS and OTG_FS device mode peripheral driver
//! Note that this currently only implements device mode
//!
//! <div class="warning">
//! There's a lot of TODOs and panics where things are not implemented
//! </div>
//!
//! List of things that is tested
//! Untested but expected to work items are noted as well
//!
//! Control Pipe:
//! - [x] Recieve `SETUP` packet (Host -> Dev)
//! - [x] Send `IN` packet (Dev -> Host).
//! - [x] Recieve `OUT` packet (Host -> Dev).
//!
//! Other Endpoints:
//! - [x] Interrupt Out
//! - [x] Interrupt In
//! - [ ] Bulk Out (Expected to work but not tested)
//! - [ ] Bulk In (Expected to work but not tested)
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
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{Direction, EndpointAddress, EndpointInfo, EndpointType, Event};
use endpoint::{ControlPipe, Endpoint};

#[cfg(usb_x0fs)]
use crate::gpio::Pull;
#[cfg(otg)]
use crate::gpio::{AFType, Speed};
use crate::interrupt::typelevel::Interrupt;
use crate::usb::{Dir, EndpointBufferAllocator, EndpointDataBuffer, In, Out};
use crate::{interrupt, peripherals, Peri, PeripheralType, RccPeripheral};

pub mod endpoint;

// TODO: We technically support 16, but we only allow 8 for now (0, 1-7).
const MAX_NR_EP: usize = 8;
const EP_MAX_PACKET_SIZE: u16 = 64;

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;
static EP_WAKERS: [AtomicWaker; MAX_NR_EP] = [NEW_AW; MAX_NR_EP];

#[cfg(otg)]
pub(crate) type EpRxResponse = crate::pac::otg::vals::EpRxResponse;
#[cfg(otg)]
pub(crate) type EpTxResponse = crate::pac::otg::vals::EpTxResponse;
#[cfg(otg)]
type UsbdRegs = crate::pac::otg::Usbd;
#[cfg(otg)]
pub(crate) type UsbToken = crate::pac::otg::vals::UsbToken;

#[cfg(usb_x0fs)]
pub(crate) type EpRxResponse = crate::pac::usb::vals::EpRxResponse;
#[cfg(usb_x0fs)]
pub(crate) type EpTxResponse = crate::pac::usb::vals::EpTxResponse;
#[cfg(usb_x0fs)]
type UsbdRegs = crate::pac::usb::Usbd;
#[cfg(usb_x0fs)]
pub(crate) type UsbToken = crate::pac::usb::vals::UsbToken;

/// USB PHY configuration.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Use direct VDD PHY output and pull-up. Set this to `false` for USB at VDD above 4V.
    #[cfg(usb_x0fs)]
    pub usb_phy_v33: bool,
    /// AFIO pull-up mode for UDP/DP. `0b11` is the USB 1.5k pull-up for 3.3V.
    #[cfg(usb_x0fs)]
    pub udp_pue: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            #[cfg(usb_x0fs)]
            usb_phy_v33: true,
            #[cfg(usb_x0fs)]
            udp_pue: 0b11,
        }
    }
}

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let int_fg = regs.int_fg().read();

        if int_fg.fifo_ov() {
            panic!("overflow");
        }

        // All the interrupt we handle
        if int_fg.suspend() || int_fg.transfer() || int_fg.bus_rst() {
            T::Interrupt::disable();

            // Bus stuff we wakup bus
            if int_fg.bus_rst() || int_fg.suspend() {
                BUS_WAKER.wake();
            }

            if int_fg.transfer() {
                let status = regs.int_st().read();

                let token = status.mask_token();
                match token {
                    UsbToken::IN | UsbToken::OUT => {
                        let ep = status.mask_uis_endp();

                        if ep as usize >= MAX_NR_EP {
                            error!("[USBFS] Unexpected EP: {} got token: {:#x}", ep, token.to_bits());
                        } else {
                            EP_WAKERS[ep as usize].wake();
                        }
                    }
                    UsbToken::SETUP => {
                        EP_WAKERS[0].wake();
                    }
                    UsbToken::RSVD => panic!("rsvd token"),
                }
            }
        }

        // Clear the host sof, we are in device mode.... It's garbage
        if int_fg.hst_sof() {
            regs.int_fg().write(|v| v.set_hst_sof(true));
        }
    }
}

pub struct Driver<'d, T: Instance, const NR_EP: usize, const SIZE: usize> {
    phantom: PhantomData<&'d mut T>,
    allocator: EndpointBufferAllocator<'d, NR_EP, SIZE>,
    allocated: Bitmap<MAX_NR_EP>,
}

impl<'d, T, const NR_EP: usize, const SIZE: usize> Driver<'d, T, NR_EP, SIZE>
where
    T: Instance,
{
    pub fn new(
        _usb: Peri<'d, T>,
        // _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: Peri<'d, impl crate::gpio::Pin>,
        dm: Peri<'d, impl crate::gpio::Pin>,
        ep_buffer: &'d mut [EndpointDataBuffer<SIZE>; NR_EP],
    ) -> Self {
        Self::new_with_config(_usb, dp, dm, ep_buffer, Config::default())
    }

    /// Use this when board-level USB PHY or pull-up settings differ from the default 3.3V setup.
    pub fn new_with_config(
        _usb: Peri<'d, T>,
        // _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: Peri<'d, impl crate::gpio::Pin>,
        dm: Peri<'d, impl crate::gpio::Pin>,
        ep_buffer: &'d mut [EndpointDataBuffer<SIZE>; NR_EP],
        config: Config,
    ) -> Self {
        assert!(ep_buffer.len() > 0);

        T::configure_pins(dp, dm, config);

        T::enable_and_reset();

        let allocator = EndpointBufferAllocator::new(ep_buffer);

        Self {
            phantom: PhantomData,
            allocator,
            allocated: Bitmap::new(),
        }
    }

    fn find_free_ep_address(&self, _dir: Direction) -> Result<u8, embassy_usb_driver::EndpointAllocError> {
        // Skip index 0 which is reserved for control endpoint
        Ok(self
            .allocated
            .next_false_index(0)
            .ok_or(embassy_usb_driver::EndpointAllocError)? as u8)
    }

    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
        dir: Direction,
    ) -> Result<Endpoint<'d, T, D, SIZE>, embassy_usb_driver::EndpointAllocError> {
        let addr = match ep_addr {
            Some(addr) => {
                // Use the provided endpoint address
                if addr.direction() != dir {
                    return Err(embassy_usb_driver::EndpointAllocError);
                }

                let ep_num = addr.index();
                if ep_num >= MAX_NR_EP {
                    return Err(embassy_usb_driver::EndpointAllocError);
                }

                // Check if this endpoint is already allocated
                if self.allocated.get(ep_num) {
                    return Err(embassy_usb_driver::EndpointAllocError);
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

impl<'d, T: Instance, const NR_EP: usize, const SIZE: usize> embassy_usb_driver::Driver<'d>
    for Driver<'d, T, NR_EP, SIZE>
{
    type EndpointOut = Endpoint<'d, T, Out, SIZE>;

    type EndpointIn = Endpoint<'d, T, In, SIZE>;

    type ControlPipe = ControlPipe<'d, T, SIZE>;

    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: embassy_usb_driver::EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, embassy_usb_driver::EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms, Direction::Out)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: embassy_usb_driver::EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, embassy_usb_driver::EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms, Direction::In)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(control_max_packet_size <= EP_MAX_PACKET_SIZE);
        let regs = T::regs();

        T::set_ep_rx_response(0, EpRxResponse::NAK);

        regs.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });

        embassy_time::block_for(embassy_time::Duration::from_micros(10));

        // Clear all
        regs.ctrl().write(|_| {});

        regs.int_en().write(|w| {
            // w.set_dev_nak(true);
            // w.set_fifo_ov(true);

            // Host SOF is ignored, not our usecase here

            w.set_suspend(true);
            w.set_transfer(true);
            w.set_bus_rst(true);
        });

        regs.ctrl().write(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            w.set_dev_pu_en(true);
        });

        let ep0_buf = self.allocator.alloc_endpoint(control_max_packet_size).unwrap();
        T::set_ep_dma(0, ep0_buf.addr() as u32);

        T::set_ep_rx_response(0, EpRxResponse::ACK);
        T::set_ep_tx_response(0, EpTxResponse::NAK);

        critical_section::with(|_cs| {
            T::Interrupt::unpend();
            unsafe {
                T::Interrupt::enable();
            }
        });

        (
            Bus {
                _phantom: PhantomData,
                inited: false,
            },
            ControlPipe::new(ep0_buf),
        )
    }
}

pub struct Bus<'d, T> {
    _phantom: PhantomData<&'d T>,
    inited: bool,
}

impl<'d, T: Instance> Bus<'d, T> {
    fn bus_reset(&mut self) {
        let regs = T::regs();

        // Reset device address
        regs.dev_ad().write(|v| {
            v.set_mask_usb_addr(0);
        });

        // Reset endpoint state on bus reset
        // EP0 get ACK/NAK so it can recieve setup
        T::set_ep_rx_response(0, EpRxResponse::ACK);
        T::set_ep_tx_response(0, EpTxResponse::NAK);

        // Mark all other EPs as NAK
        for i in 1..=7 {
            use embassy_usb_driver::Bus;
            T::set_ep_rx_response(i, EpRxResponse::NAK);
            T::set_ep_tx_response(i, EpTxResponse::NAK);

            // It looks like the HW has a bug, after reset, all EP Tx/Rx
            // are set to enabled. So we disable them here after bus reset.
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::In), false);
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::Out), false);
        }
    }
}

impl<'d, T> embassy_usb_driver::Bus for Bus<'d, T>
where
    T: Instance,
{
    async fn enable(&mut self) {
        let regs = T::regs();

        // Enable the port
        regs.udev_ctrl().write(|w| {
            // Pull Down needs to be disabled because that is for HOST
            w.set_pd_dis(true);
            w.set_port_en(true);
        });

        // Do a bus reset on "enable"
        self.bus_reset();
    }

    async fn disable(&mut self) {
        trace!("disable")
    }

    async fn poll(&mut self) -> embassy_usb_driver::Event {
        // TODO: VBUS detection
        if !self.inited {
            self.inited = true;
            return Event::PowerDetected;
        }

        poll_fn(|ctx| {
            BUS_WAKER.register(ctx.waker());

            let poll_res = {
                let regs = T::regs();
                let interrupt_flags = regs.int_fg().read();

                // Either Suspend or Resume has happened
                if interrupt_flags.suspend() {
                    // Clear suspend flag
                    regs.int_fg().write(|v| v.set_suspend(true));

                    if regs.mis_st().read().suspend() {
                        Poll::Ready(Event::Suspend)
                    } else {
                        Poll::Ready(Event::Resume)
                    }
                } else if interrupt_flags.bus_rst() {
                    trace!("bus: reset");
                    self.bus_reset();

                    regs.int_fg().write(|v| {
                        v.set_bus_rst(true);
                    });

                    Poll::Ready(Event::Reset)
                } else {
                    Poll::Pending
                }
            };
            unsafe { T::Interrupt::enable() };
            poll_res
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        #[cfg(feature = "defmt")]
        trace!(
            "[USBFS] Endpoint: {}, {}: Set enable={}",
            ep_addr.index(),
            ep_addr.direction(),
            enabled
        );
        T::set_ep_enabled(ep_addr.index(), ep_addr.direction(), enabled);
        EP_WAKERS[ep_addr.index() as usize].wake();
    }

    fn endpoint_set_stalled(&mut self, _ep_addr: EndpointAddress, _stalled: bool) {
        todo!()
    }

    fn endpoint_is_stalled(&mut self, _ep_addr: EndpointAddress) -> bool {
        todo!()
    }

    async fn remote_wakeup(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
        todo!()
    }
}

pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

pub(crate) trait SealedInstance: RccPeripheral {
    fn regs() -> UsbdRegs;
    fn configure_pins(dp: Peri<'_, impl crate::gpio::Pin>, dm: Peri<'_, impl crate::gpio::Pin>, config: Config)
    where
        Self: Sized;
    fn set_ep_dma(ep: usize, addr: u32);
    fn set_ep_tx_len(ep: usize, len: u8);
    fn set_ep_tx_response(ep: usize, response: EpTxResponse);
    fn set_ep_rx_response(ep: usize, response: EpRxResponse);
    fn set_ep_tx_toggle_response(ep: usize, toggle: bool, response: EpTxResponse);
    fn set_ep_rx_toggle_response(ep: usize, toggle: bool, response: EpRxResponse);
    fn toggle_ep_tx_response(ep: usize, response: EpTxResponse);
    fn toggle_ep_rx_response(ep: usize, response: EpRxResponse);
    fn ep_rx_len() -> usize;
    fn ep_is_enabled(ep: usize, dir: Direction) -> bool;
    fn set_ep_enabled(ep: usize, dir: Direction, enabled: bool);
}

/// OTG_FS peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (otg, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> UsbdRegs {
                // datasheet
                unsafe { crate::pac::otg::Usbd::from_ptr(crate::pac::OTG_FS.as_ptr()) }
            }

            fn configure_pins(
                dp: Peri<'_, impl crate::gpio::Pin>,
                dm: Peri<'_, impl crate::gpio::Pin>,
                _config: Config,
            ) {
                dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
                dm.set_as_af_output(AFType::OutputPushPull, Speed::High);
            }

            fn set_ep_dma(ep: usize, addr: u32) {
                Self::regs().uep_dma(ep).write_value(addr);
            }

            fn set_ep_tx_len(ep: usize, len: u8) {
                Self::regs().uep_t_len(ep).write_value(len);
            }

            fn set_ep_tx_response(ep: usize, response: EpTxResponse) {
                Self::regs().uep_tx_ctrl(ep).modify(|w| w.set_mask_t_res(response));
            }

            fn set_ep_rx_response(ep: usize, response: EpRxResponse) {
                Self::regs().uep_rx_ctrl(ep).modify(|w| w.set_mask_r_res(response));
            }

            fn set_ep_tx_toggle_response(ep: usize, toggle: bool, response: EpTxResponse) {
                Self::regs().uep_tx_ctrl(ep).modify(|w| {
                    w.set_t_tog(toggle);
                    w.set_mask_t_res(response);
                });
            }

            fn set_ep_rx_toggle_response(ep: usize, toggle: bool, response: EpRxResponse) {
                Self::regs().uep_rx_ctrl(ep).modify(|w| {
                    w.set_r_tog(toggle);
                    w.set_mask_r_res(response);
                });
            }

            fn toggle_ep_tx_response(ep: usize, response: EpTxResponse) {
                Self::regs().uep_tx_ctrl(ep).modify(|w| {
                    w.set_t_tog(!w.t_tog());
                    w.set_mask_t_res(response);
                });
            }

            fn toggle_ep_rx_response(ep: usize, response: EpRxResponse) {
                Self::regs().uep_rx_ctrl(ep).modify(|w| {
                    w.set_r_tog(!w.r_tog());
                    w.set_mask_r_res(response);
                });
            }

            fn ep_rx_len() -> usize {
                Self::regs().rx_len().read().0 as usize
            }

            fn ep_is_enabled(ep: usize, dir: Direction) -> bool {
                let regs = Self::regs();
                match (ep, dir) {
                    (4, Direction::In) => regs.uep4_1_mod().read().uep4_tx_en(),
                    (4, Direction::Out) => regs.uep4_1_mod().read().uep4_rx_en(),
                    (1, Direction::In) => regs.uep4_1_mod().read().uep1_tx_en(),
                    (1, Direction::Out) => regs.uep4_1_mod().read().uep1_rx_en(),
                    (2, Direction::In) => regs.uep2_3_mod().read().uep2_tx_en(),
                    (2, Direction::Out) => regs.uep2_3_mod().read().uep2_rx_en(),
                    (3, Direction::In) => regs.uep2_3_mod().read().uep3_tx_en(),
                    (3, Direction::Out) => regs.uep2_3_mod().read().uep3_rx_en(),
                    (5, Direction::In) => regs.uep5_6_mod().read().uep5_tx_en(),
                    (5, Direction::Out) => regs.uep5_6_mod().read().uep5_rx_en(),
                    (6, Direction::In) => regs.uep5_6_mod().read().uep6_tx_en(),
                    (6, Direction::Out) => regs.uep5_6_mod().read().uep6_rx_en(),
                    (7, Direction::In) => regs.uep7_mod().read().uep7_tx_en(),
                    (7, Direction::Out) => regs.uep7_mod().read().uep7_rx_en(),
                    _ => panic!("Unsupported EP"),
                }
            }

            fn set_ep_enabled(ep: usize, dir: Direction, enabled: bool) {
                let regs = Self::regs();
                match (ep, dir) {
                    (4, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_uep4_tx_en(enabled)),
                    (4, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_uep4_rx_en(enabled)),
                    (1, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_uep1_tx_en(enabled)),
                    (1, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_uep1_rx_en(enabled)),
                    (2, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_uep2_tx_en(enabled)),
                    (2, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_uep2_rx_en(enabled)),
                    (3, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_uep3_tx_en(enabled)),
                    (3, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_uep3_rx_en(enabled)),
                    (5, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_uep5_tx_en(enabled)),
                    (5, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_uep5_rx_en(enabled)),
                    (6, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_uep6_tx_en(enabled)),
                    (6, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_uep6_rx_en(enabled)),
                    (7, Direction::In) => regs.uep7_mod().modify(|v| v.set_uep7_tx_en(enabled)),
                    (7, Direction::Out) => regs.uep7_mod().modify(|v| v.set_uep7_rx_en(enabled)),
                    _ => panic!("setting non-existent endpoint"),
                }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };

    (usb, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> UsbdRegs {
                unsafe { crate::pac::usb::Usbd::from_ptr(crate::pac::$inst.as_ptr()) }
            }

            fn configure_pins(dp: Peri<'_, impl crate::gpio::Pin>, dm: Peri<'_, impl crate::gpio::Pin>, config: Config) {
                dp.set_as_input(Pull::Up);
                dm.set_as_input(Pull::None);

                crate::pac::AFIO.ctlr().modify(|w| {
                    w.set_usb_phy_v33(config.usb_phy_v33);
                    w.set_udp_pue(config.udp_pue);
                    w.set_udm_pue(0);
                    w.set_usb_ioen(true);
                });
            }

            fn set_ep_dma(ep: usize, addr: u32) {
                let regs = Self::regs();
                let dma = crate::pac::usb::regs::UepDma(addr);
                match ep {
                    0..=3 => regs.uep0123_dma(ep).write_value(dma),
                    5..=7 => regs.uep567_dma(ep - 5).write_value(dma),
                    4 => {},
                    _ => panic!("unsupported USBFS endpoint"),
                }
            }

            fn set_ep_tx_len(ep: usize, len: u8) {
                let regs = Self::regs();
                match ep {
                    0..=4 => regs.uep01234_t_len(ep).write_value(crate::pac::usb::regs::UepTLen(len)),
                    5..=7 => regs.uep567_t_len(ep - 5).write_value(crate::pac::usb::regs::UepTLen(len)),
                    _ => panic!("unsupported USBFS endpoint"),
                }
            }

            fn set_ep_tx_response(ep: usize, response: EpTxResponse) {
                Self::modify_ep_ctrl(ep, |w| w.set_t_res(response));
            }

            fn set_ep_rx_response(ep: usize, response: EpRxResponse) {
                Self::modify_ep_ctrl(ep, |w| w.set_r_res(response));
            }

            fn set_ep_tx_toggle_response(ep: usize, toggle: bool, response: EpTxResponse) {
                Self::modify_ep_ctrl(ep, |w| {
                    w.set_t_tog(toggle);
                    w.set_t_res(response);
                });
            }

            fn set_ep_rx_toggle_response(ep: usize, toggle: bool, response: EpRxResponse) {
                Self::modify_ep_ctrl(ep, |w| {
                    w.set_r_tog(toggle);
                    w.set_r_res(response);
                });
            }

            fn toggle_ep_tx_response(ep: usize, response: EpTxResponse) {
                Self::modify_ep_ctrl(ep, |w| {
                    w.set_t_tog(!w.t_tog());
                    w.set_t_res(response);
                });
            }

            fn toggle_ep_rx_response(ep: usize, response: EpRxResponse) {
                Self::modify_ep_ctrl(ep, |w| {
                    w.set_r_tog(!w.r_tog());
                    w.set_r_res(response);
                });
            }

            fn ep_rx_len() -> usize {
                Self::regs().rx_len().read().0 as usize
            }

            fn ep_is_enabled(ep: usize, dir: Direction) -> bool {
                let regs = Self::regs();
                match (ep, dir) {
                    (4, Direction::In) => regs.uep4_1_mod().read().tx_en(0),
                    (4, Direction::Out) => regs.uep4_1_mod().read().rx_en(0),
                    (1, Direction::In) => regs.uep4_1_mod().read().tx_en(1),
                    (1, Direction::Out) => regs.uep4_1_mod().read().rx_en(1),
                    (2, Direction::In) => regs.uep2_3_mod().read().tx_en(0),
                    (2, Direction::Out) => regs.uep2_3_mod().read().rx_en(0),
                    (3, Direction::In) => regs.uep2_3_mod().read().tx_en(1),
                    (3, Direction::Out) => regs.uep2_3_mod().read().rx_en(1),
                    (5, Direction::In) => regs.uep567_mod().read().tx_en(0),
                    (5, Direction::Out) => regs.uep567_mod().read().rx_en(0),
                    (6, Direction::In) => regs.uep567_mod().read().tx_en(1),
                    (6, Direction::Out) => regs.uep567_mod().read().rx_en(1),
                    (7, Direction::In) => regs.uep567_mod().read().tx_en(2),
                    (7, Direction::Out) => regs.uep567_mod().read().rx_en(2),
                    _ => panic!("Unsupported EP"),
                }
            }

            fn set_ep_enabled(ep: usize, dir: Direction, enabled: bool) {
                let regs = Self::regs();
                match (ep, dir) {
                    (4, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_tx_en(0, enabled)),
                    (4, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_rx_en(0, enabled)),
                    (1, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_tx_en(1, enabled)),
                    (1, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_rx_en(1, enabled)),
                    (2, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_tx_en(0, enabled)),
                    (2, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_rx_en(0, enabled)),
                    (3, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_tx_en(1, enabled)),
                    (3, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_rx_en(1, enabled)),
                    (5, Direction::In) => regs.uep567_mod().modify(|v| v.set_tx_en(0, enabled)),
                    (5, Direction::Out) => regs.uep567_mod().modify(|v| v.set_rx_en(0, enabled)),
                    (6, Direction::In) => regs.uep567_mod().modify(|v| v.set_tx_en(1, enabled)),
                    (6, Direction::Out) => regs.uep567_mod().modify(|v| v.set_rx_en(1, enabled)),
                    (7, Direction::In) => regs.uep567_mod().modify(|v| v.set_tx_en(2, enabled)),
                    (7, Direction::Out) => regs.uep567_mod().modify(|v| v.set_rx_en(2, enabled)),
                    _ => panic!("setting non-existent endpoint"),
                }
            }
        }

        impl peripherals::$inst {
            fn modify_ep_ctrl(ep: usize, f: impl FnOnce(&mut crate::pac::usb::regs::UepCtrl)) {
                let regs = <Self as SealedInstance>::regs();
                match ep {
                    0..=4 => regs.uep01234_ctrl(ep).modify(f),
                    5..=7 => regs.uep567_ctrl(ep - 5).modify(f),
                    _ => panic!("unsupported USBFS endpoint"),
                }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);
