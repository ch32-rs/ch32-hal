//! USBFS device mode peripheral driver for CH32V208
//!
//! This driver supports the WCH USBFS peripheral (usb_v2fs) which uses PB6/PB7 pins.
//! Note: This is different from USBD which uses PA11/PA12.
//!
//! ## Key differences from OTG_FS (CH32V307)
//! - DMA address is 16-bit (hardware auto-adds 0x2000_0000 base)
//! - UEP_MOD uses array-indexed API: `tx_en(n)` instead of `uep4_tx_en()`
//!
//! ## Tested features
//! - [x] Control Pipe (SETUP/IN/OUT)
//! - [ ] Interrupt IN/OUT
//! - [ ] Bulk IN/OUT
//!

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use bitmaps::Bitmap;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{Direction, EndpointAddress, EndpointInfo, EndpointType, Event};
use endpoint::{ControlPipe, Endpoint};

// USB response values (no vals module in usb_v2fs)
mod vals {
    /// Endpoint RX response
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[repr(u8)]
    pub enum EpRxResponse {
        ACK = 0b00,
        NAK = 0b10,
        STALL = 0b11,
    }

    impl EpRxResponse {
        pub const fn to_bits(self) -> u8 {
            self as u8
        }
    }

    /// Endpoint TX response
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[repr(u8)]
    pub enum EpTxResponse {
        ACK = 0b00,
        NAK = 0b10,
        STALL = 0b11,
    }

    impl EpTxResponse {
        pub const fn to_bits(self) -> u8 {
            self as u8
        }
    }

    /// USB token types
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[repr(u8)]
    pub enum UsbToken {
        OUT = 0b00,
        RSVD = 0b01,
        IN = 0b10,
        SETUP = 0b11,
    }

    impl UsbToken {
        pub const fn from_bits(val: u8) -> Self {
            match val & 0b11 {
                0b00 => UsbToken::OUT,
                0b01 => UsbToken::RSVD,
                0b10 => UsbToken::IN,
                _ => UsbToken::SETUP,
            }
        }

        pub const fn to_bits(self) -> u8 {
            self as u8
        }
    }
}
use vals::{EpRxResponse, EpTxResponse, UsbToken};

use crate::interrupt::typelevel::Interrupt;
use crate::usb::{Dir, EndpointBufferAllocator, EndpointDataBuffer, In, Out};
use crate::{interrupt, peripherals, Peri, PeripheralType, RccPeripheral};

pub mod endpoint;

const MAX_NR_EP: usize = 8;
const EP_MAX_PACKET_SIZE: u16 = 64;

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;
static EP_WAKERS: [AtomicWaker; MAX_NR_EP] = [NEW_AW; MAX_NR_EP];

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let int_fg = regs.int_fg().read();

        // Debug: print interrupt flags
        crate::println!(
            "[IRQ] fg: rst={} sus={} xfer={} sof={}",
            int_fg.bus_rst(),
            int_fg.suspend(),
            int_fg.transfer(),
            int_fg.hst_sof()
        );

        if int_fg.fifo_ov() {
            panic!("USBFS FIFO overflow");
        }

        if int_fg.suspend() || int_fg.transfer() || int_fg.bus_rst() {
            T::Interrupt::disable();

            if int_fg.bus_rst() {
                crate::println!("[IRQ] Bus Reset!");
                BUS_WAKER.wake();
            }
            if int_fg.suspend() {
                crate::println!("[IRQ] Suspend/Resume");
                BUS_WAKER.wake();
            }

            if int_fg.transfer() {
                let status = regs.int_st().read();
                let token = UsbToken::from_bits(status.mask_token());
                let ep = status.mask_uis_endp();

                crate::println!("[IRQ] Transfer: EP{} token={}", ep, token.to_bits());

                match token {
                    UsbToken::IN | UsbToken::OUT => {
                        if ep as usize >= MAX_NR_EP {
                            error!("[USBFS] Unexpected EP: {} got token: {:#x}", ep, token.to_bits());
                        } else {
                            EP_WAKERS[ep as usize].wake();
                        }
                    }
                    UsbToken::SETUP => {
                        crate::println!("[IRQ] SETUP packet!");
                        EP_WAKERS[0].wake();
                    }
                    UsbToken::RSVD => panic!("USBFS reserved token"),
                }
            }
        }

        // Clear host SOF flag (we're in device mode)
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
    /// Create a new USBFS driver.
    ///
    /// For CH32V208, use PB7 for DP and PB6 for DM.
    pub fn new(
        _usb: Peri<'d, T>,
        _dp: Peri<'d, impl crate::gpio::Pin>,
        _dm: Peri<'d, impl crate::gpio::Pin>,
        ep_buffer: &'d mut [EndpointDataBuffer<SIZE>; NR_EP],
    ) -> Self {
        assert!(ep_buffer.len() > 0);

        T::enable_and_reset();

        // Note: For CH32V208, USBFS uses PB6(DM)/PB7(DP) via internal PHY connection
        // No GPIO AF configuration needed - the USB PHY handles the pins directly
        // The dp/dm parameters are kept for API compatibility and resource tracking

        let allocator = EndpointBufferAllocator::new(ep_buffer);

        Self {
            phantom: PhantomData,
            allocator,
            allocated: Bitmap::new(),
        }
    }

    fn find_free_ep_address(&self, _dir: Direction) -> Result<u8, embassy_usb_driver::EndpointAllocError> {
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
                if addr.direction() != dir {
                    return Err(embassy_usb_driver::EndpointAllocError);
                }

                let ep_num = addr.index();
                if ep_num >= MAX_NR_EP {
                    return Err(embassy_usb_driver::EndpointAllocError);
                }

                if self.allocated.get(ep_num) {
                    return Err(embassy_usb_driver::EndpointAllocError);
                }

                self.allocated.set(ep_num, true);
                addr
            }
            None => {
                let ep_addr = self.find_free_ep_address(dir)?;
                let addr = EndpointAddress::from_parts(ep_addr as usize, dir);
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
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, embassy_usb_driver::EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms, Direction::Out)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, embassy_usb_driver::EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms, Direction::In)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(control_max_packet_size <= EP_MAX_PACKET_SIZE);
        let regs = T::regs();

        crate::println!("[USBFS] start() called, max_pkt={}", control_max_packet_size);

        regs.uep_rx_ctrl(0).write(|v| v.set_mask_r_res(EpRxResponse::NAK.to_bits()));

        // Reset SIE
        regs.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });

        embassy_time::block_for(embassy_time::Duration::from_micros(10));

        // Clear reset
        regs.ctrl().write(|_| {});

        // Enable interrupts
        regs.int_en().write(|w| {
            w.set_suspend(true);
            w.set_transfer(true);
            w.set_bus_rst(true);
        });

        // Enable USB device
        regs.ctrl().write(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            w.set_dev_pu_en(true);
        });

        // Enable USB physical port
        // UDEV_CTRL: UD_PD_DIS (disable pull-down) | UD_PORT_EN (enable port I/O)
        regs.udev_ctrl().write(|w| {
            w.set_pd_dis(true); // Disable pull-down resistor
            w.set_port_en(true); // Enable USB port I/O - CRITICAL!
        });
        crate::println!("[USBFS] UDEV_CTRL set: pd_dis=true, port_en=true");

        // Allocate EP0 buffer
        let ep0_buf = self.allocator.alloc_endpoint(control_max_packet_size).unwrap();
        let ep0_addr = ep0_buf.addr() as u32;

        crate::println!("[USBFS] EP0 buf addr=0x{:08x}", ep0_addr);

        // Write DMA address - full 32-bit address
        regs.uep_dma(0).write_value(ep0_addr);

        regs.uep_rx_ctrl(0).write(|w| w.set_mask_r_res(EpRxResponse::ACK.to_bits()));
        regs.uep_tx_ctrl(0).write(|w| w.set_mask_t_res(EpTxResponse::NAK.to_bits()));

        crate::println!("[USBFS] Enabling interrupt...");

        critical_section::with(|_cs| {
            T::Interrupt::unpend();
            unsafe {
                T::Interrupt::enable();
            }
        });

        crate::println!("[USBFS] USB device started, waiting for host...");

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

        // EP0: ACK for RX, NAK for TX
        regs.uep_rx_ctrl(0).write(|v| v.set_mask_r_res(EpRxResponse::ACK.to_bits()));
        regs.uep_tx_ctrl(0).write(|v| v.set_mask_t_res(EpTxResponse::NAK.to_bits()));

        // All other EPs: NAK and disabled
        for i in 1..=7 {
            use embassy_usb_driver::Bus;
            regs.uep_rx_ctrl(i).write(|v| v.set_mask_r_res(EpRxResponse::NAK.to_bits()));
            regs.uep_tx_ctrl(i).write(|v| v.set_mask_t_res(EpTxResponse::NAK.to_bits()));
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

        // Enable USB port (device mode)
        regs.udev_ctrl().write(|w| {
            w.set_pd_dis(true); // Disable pull-down (that's for host mode)
            w.set_port_en(true);
        });

        self.bus_reset();
    }

    async fn disable(&mut self) {
        trace!("USBFS disable");
    }

    async fn poll(&mut self) -> Event {
        if !self.inited {
            self.inited = true;
            return Event::PowerDetected;
        }

        poll_fn(|ctx| {
            BUS_WAKER.register(ctx.waker());

            let poll_res = {
                let regs = T::regs();
                let interrupt_flags = regs.int_fg().read();

                if interrupt_flags.suspend() {
                    regs.int_fg().write(|v| v.set_suspend(true));

                    if regs.mis_st().read().suspend() {
                        Poll::Ready(Event::Suspend)
                    } else {
                        Poll::Ready(Event::Resume)
                    }
                } else if interrupt_flags.bus_rst() {
                    trace!("USBFS bus reset");
                    self.bus_reset();
                    regs.int_fg().write(|v| v.set_bus_rst(true));
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
        let regs = T::regs();
        let ep = ep_addr.index();
        let dir = ep_addr.direction();

        #[cfg(feature = "defmt")]
        trace!("[USBFS] EP{} {:?}: enabled={}", ep, dir, enabled);

        // USBFS uses array-indexed API for UEP_MOD
        // uep4_1_mod: index 0 = EP4, index 1 = EP1
        // uep2_3_mod: index 0 = EP2, index 1 = EP3
        // uep5_6_mod: index 0 = EP5, index 1 = EP6
        // uep7_mod: EP7 only
        match (ep, dir) {
            // EP4 and EP1 share uep4_1_mod
            (4, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_tx_en(0, enabled)),
            (4, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_rx_en(0, enabled)),
            (1, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_tx_en(1, enabled)),
            (1, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_rx_en(1, enabled)),

            // EP2 and EP3 share uep2_3_mod
            (2, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_tx_en(0, enabled)),
            (2, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_rx_en(0, enabled)),
            (3, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_tx_en(1, enabled)),
            (3, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_rx_en(1, enabled)),

            // EP5 and EP6 share uep5_6_mod
            (5, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_tx_en(0, enabled)),
            (5, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_rx_en(0, enabled)),
            (6, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_tx_en(1, enabled)),
            (6, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_rx_en(1, enabled)),

            // EP7 has its own register
            (7, Direction::In) => regs.uep7_mod().modify(|v| v.set_tx_en(enabled)),
            (7, Direction::Out) => regs.uep7_mod().modify(|v| v.set_rx_en(enabled)),

            _ => {
                #[cfg(feature = "defmt")]
                defmt::panic!("Invalid endpoint {}", ep);
                #[cfg(not(feature = "defmt"))]
                panic!("Invalid endpoint");
            }
        }
        EP_WAKERS[ep].wake();
    }

    fn endpoint_set_stalled(&mut self, _ep_addr: EndpointAddress, _stalled: bool) {
        todo!("USBFS endpoint_set_stalled")
    }

    fn endpoint_is_stalled(&mut self, _ep_addr: EndpointAddress) -> bool {
        todo!("USBFS endpoint_is_stalled")
    }

    async fn remote_wakeup(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
        Err(embassy_usb_driver::Unsupported)
    }
}

pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

trait SealedInstance: RccPeripheral {
    fn regs() -> crate::pac::usb::Usbd;
}

/// USBFS peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (usb, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::usb::Usbd {
                // USBFS peripheral uses usb::Usbd type, but instance is named USBFS
                unsafe { crate::pac::usb::Usbd::from_ptr(crate::pac::$inst.as_ptr()) }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);

