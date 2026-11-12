//! USBFS device-mode driver for the WCH **`usb_l1fs`** layout (CH32L103).
//!
//! Manual name: "USB 全速主机/设备控制器（USBFS）" (CH32L103RM §20).
//! This is one register layout of the shared WCH FS (`UEP*`) IP family;
//! `v1fs` / `v2fs` / `x0fs` / OTG / H4-USBFS are siblings, not this module.
//! Census and rename plan: `ch32-data` `docs/usb_ip.md`.
//!
//! Register layout follows `ch32-data` `usb_l1fs.yaml` / CH32L103RM 表 20-3:
//! combined 16-bit `UEP_CTRL` (TX low byte, RX high byte), 15-bit `UEP_DMA`,
//! and 16-bit `UEP_T_LEN`.
//!
//! Control Pipe:
//! - [x] Receive `SETUP` packet (Host -> Dev)
//! - [x] Send `IN` packet (Dev -> Host)
//! - [x] Receive `OUT` packet (Host -> Dev)
//!
//! Other Endpoints:
//! - [x] Interrupt Out / In (ported from `otg_fs`)
//! - [ ] Bulk Out / In (expected to work, not yet tested on L103)
//!
//! Other Features:
//! - [ ] Set / get endpoint stall
//! - [ ] Remote wakeup

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use bitmaps::Bitmap;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{Direction, EndpointAddress, EndpointInfo, EndpointType, Event};
use endpoint::{ControlPipe, Endpoint};

use crate::gpio::{AFType, Speed};
use crate::interrupt::typelevel::Interrupt;
use crate::pac::usb::vals::{EpRxResponse, EpTxResponse, UsbToken};
use crate::usb::{Dir, EndpointBufferAllocator, EndpointDataBuffer, In, Out};
use crate::{interrupt, peripherals, Peri, PeripheralType, RccPeripheral};

pub mod endpoint;

const MAX_NR_EP: usize = 8;
const EP_MAX_PACKET_SIZE: u16 = 64;

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;
static EP_WAKERS: [AtomicWaker; MAX_NR_EP] = [NEW_AW; MAX_NR_EP];

pub(crate) fn set_ep_dma(regs: crate::pac::usb::Usbd, ep: usize, addr: usize) {
    // CH32L103RM 表 20-3: only bits [14:0] are valid. SRAM lives at
    // 0x2000_0000, so the low 15 bits uniquely identify the buffer.
    regs.uep_dma(ep).write(|w| w.set_addr((addr as u16) & 0x7fff));
}

pub(crate) fn set_ep_tx_len(regs: crate::pac::usb::Usbd, ep: usize, len: usize) {
    regs.uep_t_len(ep).write(|w| w.set_tx_len(len as u16));
}

/// Enable / disable TX or RX for endpoint `ep` (1..=7).
///
/// `UEP4_1_MOD` / `UEP2_3_MOD` / `UEP5_6_MOD` pack two endpoints each; the
/// lower nibble is the even-ish member of the pair (4/2/5), the upper nibble
/// is the odd-ish one (1/3/6). `UEP7_MOD` stands alone.
pub(crate) fn endpoint_set_enabled(regs: crate::pac::usb::Usbd, ep: usize, dir: Direction, enabled: bool) {
    match (ep, dir) {
        (4, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_tx_en(0, enabled)),
        (4, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_rx_en(0, enabled)),
        (1, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_tx_en(1, enabled)),
        (1, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_rx_en(1, enabled)),

        (2, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_tx_en(0, enabled)),
        (2, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_rx_en(0, enabled)),
        (3, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_tx_en(1, enabled)),
        (3, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_rx_en(1, enabled)),

        (5, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_tx_en(0, enabled)),
        (5, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_rx_en(0, enabled)),
        (6, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_tx_en(1, enabled)),
        (6, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_rx_en(1, enabled)),

        (7, Direction::In) => regs.uep7_mod().modify(|v| v.set_tx_en(enabled)),
        (7, Direction::Out) => regs.uep7_mod().modify(|v| v.set_rx_en(enabled)),

        _ => panic!("unsupported endpoint {}", ep),
    }
}

pub(crate) fn endpoint_is_enabled(regs: crate::pac::usb::Usbd, ep: usize, dir: Direction) -> bool {
    match (ep, dir) {
        (4, Direction::In) => regs.uep4_1_mod().read().tx_en(0),
        (4, Direction::Out) => regs.uep4_1_mod().read().rx_en(0),
        (1, Direction::In) => regs.uep4_1_mod().read().tx_en(1),
        (1, Direction::Out) => regs.uep4_1_mod().read().rx_en(1),

        (2, Direction::In) => regs.uep2_3_mod().read().tx_en(0),
        (2, Direction::Out) => regs.uep2_3_mod().read().rx_en(0),
        (3, Direction::In) => regs.uep2_3_mod().read().tx_en(1),
        (3, Direction::Out) => regs.uep2_3_mod().read().rx_en(1),

        (5, Direction::In) => regs.uep5_6_mod().read().tx_en(0),
        (5, Direction::Out) => regs.uep5_6_mod().read().rx_en(0),
        (6, Direction::In) => regs.uep5_6_mod().read().tx_en(1),
        (6, Direction::Out) => regs.uep5_6_mod().read().rx_en(1),

        (7, Direction::In) => regs.uep7_mod().read().tx_en(),
        (7, Direction::Out) => regs.uep7_mod().read().rx_en(),

        _ => panic!("unsupported endpoint {}", ep),
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

        if int_fg.suspend() || int_fg.transfer() || int_fg.bus_rst() {
            T::Interrupt::disable();

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
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: Peri<'d, impl crate::gpio::Pin>,
        dm: Peri<'d, impl crate::gpio::Pin>,
        ep_buffer: &'d mut [EndpointDataBuffer<SIZE>; NR_EP],
    ) -> Self {
        assert!(!ep_buffer.is_empty());

        dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
        dm.set_as_af_output(AFType::OutputPushPull, Speed::High);

        T::enable_and_reset();

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
                if ep_num == 0 || ep_num >= MAX_NR_EP {
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
                max_burst: 0,
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
        config: &embassy_usb_driver::EndpointConfig,
    ) -> Result<Self::EndpointOut, embassy_usb_driver::EndpointAllocError> {
        self.alloc_endpoint(
            config.ep_type,
            config.addr,
            config.max_packet_size,
            config.interval_ms,
            Direction::Out,
        )
    }

    fn alloc_endpoint_in(
        &mut self,
        config: &embassy_usb_driver::EndpointConfig,
    ) -> Result<Self::EndpointIn, embassy_usb_driver::EndpointAllocError> {
        self.alloc_endpoint(
            config.ep_type,
            config.addr,
            config.max_packet_size,
            config.interval_ms,
            Direction::In,
        )
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(control_max_packet_size <= EP_MAX_PACKET_SIZE);
        let regs = T::regs();

        // Combined UEP_CTRL: set both halves explicitly.
        regs.uep_ctrl(0).write(|v| {
            v.set_mask_r_res(EpRxResponse::NAK);
            v.set_mask_t_res(EpTxResponse::NAK);
        });

        regs.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });

        crate::internal::delay::block_for_us(10);

        regs.ctrl().write(|_| {});

        regs.int_en().write(|w| {
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
        set_ep_dma(regs, 0, ep0_buf.addr());

        regs.uep_ctrl(0).write(|w| {
            w.set_mask_r_res(EpRxResponse::ACK);
            w.set_mask_t_res(EpTxResponse::NAK);
        });

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

        regs.dev_ad().write(|v| {
            v.set_mask_usb_addr(0);
        });

        regs.uep_ctrl(0).write(|v| {
            v.set_mask_r_res(EpRxResponse::ACK);
            v.set_mask_t_res(EpTxResponse::NAK);
        });

        for i in 1..=7 {
            use embassy_usb_driver::Bus;

            regs.uep_ctrl(i).write(|v| {
                v.set_mask_r_res(EpRxResponse::NAK);
                v.set_mask_t_res(EpTxResponse::NAK);
            });

            // Hardware leaves endpoints enabled across reset; clear them.
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

        regs.udev_ctrl().write(|w| {
            w.set_pd_dis(true);
            w.set_port_en(true);
        });

        self.bus_reset();
    }

    async fn disable(&mut self) {
        trace!("disable")
    }

    async fn poll(&mut self) -> embassy_usb_driver::Event {
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

        endpoint_set_enabled(T::regs(), ep_addr.index(), ep_addr.direction(), enabled);
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

trait SealedInstance: RccPeripheral {
    fn regs() -> crate::pac::usb::Usbd;
}

/// USBFS peripheral instance (`usb_l1fs`).
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (usb, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::usb::Usbd {
                unsafe { crate::pac::usb::Usbd::from_ptr(crate::pac::$inst.as_ptr()) }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);
