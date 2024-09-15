use core::future::poll_fn;
use core::marker::PhantomData;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{self as driver, Direction, EndpointInfo, EndpointType};

use crate::gpio::Speed;
use crate::{interrupt, peripherals, Peripheral, RccPeripheral};

const NR_EP: usize = 16;
const MAX_EP_OUT_BUFFER: u16 = 64;

pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    ep_0_in: EndpointData,
    ep_0_out: EndpointData,
    ep_alloc: [Option<EndpointData>; NR_EP],
    ep_out_buffer: &'d mut [EpOutBuffer],
    ep_out_buffer_next: usize,
}

#[derive(Debug, Clone, Copy)]
struct EndpointData {
    ep_type: EndpointType,
    max_packet_size: u16,
}

pub struct InterruptHandler<T: Instance> {
    phantom: PhantomData<T>,
}

#[repr(C, align(4))]
pub struct EpOutBuffer {
    data: [u8; MAX_EP_OUT_BUFFER as usize],
}

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;

impl<'d, T> Driver<'d, T>
where
    T: Instance,
{
    pub fn new(
        _usb: impl Peripheral<P = T> + 'd,
        // _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: impl Peripheral<P = impl crate::gpio::Pin> + 'd,
        dm: impl Peripheral<P = impl crate::gpio::Pin> + 'd,
        ep_out_buffer: &'d mut [EpOutBuffer],
    ) -> Self {
        assert!(ep_out_buffer.len() > 0);
        let dp = dp.into_ref();
        let dm = dm.into_ref();

        {
            dp.set_as_output(Speed::High);
            dp.set_low();
            dm.set_as_output(Speed::High);
            dm.set_low();
        }

        T::enable_and_reset();

        let regs = T::regs();

        regs.ctrl().modify(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });

        embassy_time::block_for(embassy_time::Duration::from_micros(10));

        regs.ctrl().modify(|w| {
            w.0 = 0;
        });

        regs.int_en().write(|w| w.0 = 0);

        regs.ctrl().modify(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            w.set_dev_pu_en(true);
        });

        regs.uep_dma(0).write_value(0u32);

        T::regs().udev_ctrl().write(|w| {
            // different from reference code
            // board has no pulldown
            w.set_pd_dis(false);
            w.set_port_en(true);
        });
        // regs.btable().write(|w| w.set_btable(0));

        // Initialize the bus so that it signals that power is available
        // usbd.rs does BUS_WAKER.wake(), but it doesn't seem necessary

        Self {
            phantom: PhantomData,
            ep_alloc: [None; NR_EP],
            ep_out_buffer,
            ep_out_buffer_next: 1,
            ep_0_in: EndpointData {
                ep_type: EndpointType::Control,
                max_packet_size: 64,
            },
            ep_0_out: EndpointData {
                ep_type: EndpointType::Control,
                max_packet_size: 64,
            },
        }
    }
}

/// USB endpoint direction.
trait Dir {
    /// Returns the direction value.
    fn dir() -> Direction;
}

/// Marker type for the "IN" direction.
pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

/// Marker type for the "OUT" direction.
pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}
/// USB endpoint.
pub struct Endpoint<'d, T, D> {
    _phantom: PhantomData<&'d (T, D)>,
    info: EndpointInfo,
    // state: &'d EpState,
}

impl<'d, T> Driver<'d, T>
where
    T: Instance,
{
    fn alloc_endpoint<D: Dir>(&mut self, ep_type: driver::EndpointType, max_packet_size: u16) -> Endpoint<'d, T, D> {
        assert!(max_packet_size <= MAX_EP_OUT_BUFFER);
        todo!()
    }
}

impl<'d, T> driver::Driver<'d> for Driver<'d, T>
where
    T: Instance,
{
    type EndpointOut = Endpoint<'d, T, Out>;

    type EndpointIn = Endpoint<'d, T, In>;

    type ControlPipe = ();

    type Bus = ();

    fn alloc_endpoint_out(
        &mut self,
        ep_type: driver::EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, driver::EndpointAllocError> {
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: driver::EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, driver::EndpointAllocError> {
        todo!()
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let regs = T::regs();
    }
}

#[repr(C, align(4))]
struct EndpointBuffer {
    mem: [u8; 64],
}

const _: () = assert!(core::mem::align_of::<EndpointBuffer>() == 4);
const _: () = assert!(core::mem::size_of::<EndpointBuffer>() == 64);

pub struct Endpoint<'d, T: Instance, D> {
    _phantom: PhantomData<(&'d mut T, D)>,
    info: EndpointInfo,
    buf: EndpointBuffer,
}

impl<'d, T: Instance, D> Endpoint<'d, T, D> {
    fn write_data(&mut self, buf: &[u8]) {
        let index = self.info.addr.index();
        self.buf.write(buf);
        btable::write_in_len::<T>(index, self.buf.addr, buf.len() as _);
    }

    fn read_data(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let index = self.info.addr.index();
        let rx_len = btable::read_out_len::<T>(index) as usize & 0x3FF;
        crate::println!("READ DONE, rx_len = {}", rx_len);
        if rx_len > buf.len() {
            return Err(EndpointError::BufferOverflow);
        }
        self.buf.read(&mut buf[..rx_len]);
        Ok(rx_len)
    }
}

impl<'d, T: Instance> driver::Endpoint for Endpoint<'d, T, In> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        crate::println!("wait_enabled IN WAITING");
        let index = self.info.addr.index();
        poll_fn(|cx| {
            EP_IN_WAKERS[index].register(cx.waker());
            let regs = T::regs();
            if regs.epr(index).read().stat_tx() == Stat::DISABLED {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
        crate::println!("wait_enabled IN OK");
    }
}

impl<'d, T: Instance> driver::Endpoint for Endpoint<'d, T, Out> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        crate::println!("wait_enabled OUT WAITING");
        let index = self.info.addr.index();
        poll_fn(|cx| {
            EP_OUT_WAKERS[index].register(cx.waker());
            let regs = T::regs();
            if regs.epr(index).read().stat_rx() == Stat::DISABLED {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
        crate::println!("wait_enabled OUT OK");
    }
}

impl<'d, T: Instance> driver::EndpointOut for Endpoint<'d, T, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        crate::println!("READ WAITING, buf.len() = {}", buf.len());
        let index = self.info.addr.index();
        let stat = poll_fn(|cx| {
            EP_OUT_WAKERS[index].register(cx.waker());
            let regs = T::regs();
            let stat = regs.epr(index).read().stat_rx();
            if matches!(stat, Stat::NAK | Stat::DISABLED) {
                Poll::Ready(stat)
            } else {
                Poll::Pending
            }
        })
        .await;

        if stat == Stat::DISABLED {
            return Err(EndpointError::Disabled);
        }

        let rx_len = self.read_data(buf)?;

        let regs = T::regs();
        regs.epr(index).write(|w| {
            w.set_ep_type(convert_type(self.info.ep_type));
            w.set_ea(self.info.addr.index() as _);
            w.set_stat_rx(Stat::from_bits(Stat::NAK.to_bits() ^ Stat::VALID.to_bits()));
            w.set_stat_tx(Stat::from_bits(0));
            w.set_ctr_rx(true); // don't clear
            w.set_ctr_tx(true); // don't clear
        });
        crate::println!("READ OK, rx_len = {}", rx_len);

        Ok(rx_len)
    }
}

impl<'d, T: Instance> driver::EndpointIn for Endpoint<'d, T, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        let index = self.info.addr.index();

        crate::println!("WRITE WAITING");
        let stat = poll_fn(|cx| {
            EP_IN_WAKERS[index].register(cx.waker());
            let regs = T::regs();
            let stat = regs.epr(index).read().stat_tx();
            if matches!(stat, Stat::NAK | Stat::DISABLED) {
                Poll::Ready(stat)
            } else {
                Poll::Pending
            }
        })
        .await;

        if stat == Stat::DISABLED {
            return Err(EndpointError::Disabled);
        }

        self.write_data(buf);

        let regs = T::regs();
        regs.epr(index).write(|w| {
            w.set_ep_type(convert_type(self.info.ep_type));
            w.set_ea(self.info.addr.index() as _);
            w.set_stat_tx(Stat::from_bits(Stat::NAK.to_bits() ^ Stat::VALID.to_bits()));
            w.set_stat_rx(Stat::from_bits(0));
            w.set_ctr_rx(true); // don't clear
            w.set_ctr_tx(true); // don't clear
        });

        crate::println!("WRITE OK");

        Ok(())
    }
}

/// USB control pipe.
pub struct ControlPipe<'d, T> {
    _phantom: PhantomData<T>,
    max_packet_size: u16,
    setup_state: &'d ControlPipeSetupState,
    ep_in: Endpoint<'d, T, In>,
    ep_out: Endpoint<'d, T, Out>,
}

impl<'d, T> embassy_usb_driver::ControlPipe for ControlPipe<'d, T>
where
    T: Instance,
{
    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    async fn setup(&mut self) -> [u8; 8] {
        loop {
            let intfg = T::regs().int_fg().read();
            let status = T::regs().int_st().read();

            loop {
                if !intfg.transfer() {
                    break;
                }
                // setup packet
                if status.mask_token() != 0b11 {
                    break;
                }

                T::regs().uep_tx_ctrl(0).write(|w| {
                    w.set_t_tog(true);
                    // NAK
                    w.set_mask_t_res(0b10);
                });
                T::regs().uep_rx_ctrl(0).write(|w| {
                    w.set_r_tog(true);
                    // NAK
                    w.set_mask_r_res(0b10);
                });
            }
            T::regs().int_fg().write_value(intfg);
        }
        poll_fn(|cx| {
            self.ep_out.state.out_waker.register(cx.waker());

            if self.setup_state.setup_ready.load(Ordering::Relaxed) {
                let data = unsafe { *self.setup_state.setup_data.get() };
                self.setup_state.setup_ready.store(false, Ordering::Release);

                // EP0 should not be controlled by `Bus` so this RMW does not need a critical section
                // Receive 1 SETUP packet
                self.regs.doeptsiz(self.ep_out.info.addr.index()).modify(|w| {
                    w.set_rxdpid_stupcnt(1);
                });

                // Clear NAK to indicate we are ready to receive more data
                if !self.quirk_setup_late_cnak {
                    self.regs
                        .doepctl(self.ep_out.info.addr.index())
                        .modify(|w| w.set_cnak(true));
                }

                trace!("SETUP received: {:?}", data);
                Poll::Ready(data)
            } else {
                trace!("SETUP waiting");
                Poll::Pending
            }
        })
        .await
    }

    async fn data_out(&mut self, buf: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        trace!("control: data_out");
        let len = self.ep_out.read(buf).await?;
        trace!("control: data_out read: {:?}", &buf[..len]);
        Ok(len)
    }

    async fn data_in(&mut self, data: &[u8], _first: bool, last: bool) -> Result<(), EndpointError> {
        trace!("control: data_in write: {:?}", data);
        self.ep_in.write(data).await?;

        // wait for status response from host after sending the last packet
        if last {
            trace!("control: data_in waiting for status");
            self.ep_out.read(&mut []).await?;
            trace!("control: complete");
        }

        Ok(())
    }

    async fn accept(&mut self) {
        trace!("control: accept");

        self.ep_in.write(&[]).await.ok();

        trace!("control: accept OK");
    }

    async fn reject(&mut self) {
        trace!("control: reject");

        // EP0 should not be controlled by `Bus` so this RMW does not need a critical section
        self.regs.diepctl(self.ep_in.info.addr.index()).modify(|w| {
            w.set_stall(true);
        });
        self.regs.doepctl(self.ep_out.info.addr.index()).modify(|w| {
            w.set_stall(true);
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        trace!("setting addr: {}", addr);
        critical_section::with(|_| {
            self.regs.dcfg().modify(|w| {
                w.set_dad(addr);
            });
        });

        // synopsys driver requires accept to be sent after changing address
        self.accept().await
    }
}

pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

trait SealedInstance: RccPeripheral {
    fn regs() -> crate::pac::otg::Usbd;
}

/// I2C peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    // /// Event interrupt for this instance
    // type EventInterrupt: interrupt::typelevel::Interrupt;
    // /// Error interrupt for this instance
    // type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (otg, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::otg::Usbd {
                // datasheet
                unsafe { crate::pac::otg::Usbd::from_ptr(crate::pac::OTG_FS.as_ptr()) }
            }
        }

        impl Instance for peripherals::$inst {
            // type EventInterrupt = crate::_generated::peripheral_interrupts::$inst::EV;
            // type ErrorInterrupt = crate::_generated::peripheral_interrupts::$inst::ER;
        }
    };
);
