//! DMA
//!
//! TODO: DMA2 with CH8 to CH11, which are handled in Exxx registers

#![macro_use]

use core::mem;

use embassy_hal_internal::PeripheralType;

use crate::{impl_peripheral, interrupt};

// dma_bdma is the V0/V1/V2/V3-style "single DMA + optional BDMA" driver.
// H4 has a much larger DMA1+DMA2+DMAMUX combo (different `dma_h4`
// version) — gate this driver out until an H4 port lands.
#[cfg(not(ch32h4))]
mod dma_bdma;
#[cfg(all(any(bdma, dma), not(ch32h4)))]
pub use dma_bdma::*;

// On CH32H4 the V3-style `Transfer` / `TransferOptions` aren't available
// (their impls use `pac::dma::vals` which doesn't exist on H4). Provide a
// minimal stub so the driver layer (`util::ChannelAndRequest`, plus the
// USART / SPI / I2C / CAN async paths that reach `ch.read()` / `ch.write()`)
// still compiles. The constructors panic and the future never completes —
// these code paths are unreachable until a real H4 DMA driver lands.
#[cfg(ch32h4)]
mod h4_stub {
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};

    use super::word::Word;
    use super::{AnyChannel, Request};
    use crate::Peri;

    #[derive(Copy, Clone, Default)]
    pub struct TransferOptions;

    pub struct Transfer<'a> {
        _phantom: core::marker::PhantomData<&'a mut ()>,
    }

    impl<'a> Transfer<'a> {
        #[inline]
        fn unimpl() -> ! {
            unimplemented!("CH32H4 DMA driver not yet ported — see ch32-hal#177 followups");
        }

        pub unsafe fn new_read<W: Word>(
            _channel: Peri<'a, AnyChannel>,
            _request: Request,
            _peri_addr: *mut W,
            _buf: &'a mut [W],
            _options: TransferOptions,
        ) -> Self {
            Self::unimpl()
        }

        pub unsafe fn new_read_raw<W: Word>(
            _channel: Peri<'a, AnyChannel>,
            _request: Request,
            _peri_addr: *mut W,
            _buf: *mut [W],
            _options: TransferOptions,
        ) -> Self {
            Self::unimpl()
        }

        pub unsafe fn new_write<W: Word>(
            _channel: Peri<'a, AnyChannel>,
            _request: Request,
            _buf: &'a [W],
            _peri_addr: *mut W,
            _options: TransferOptions,
        ) -> Self {
            Self::unimpl()
        }

        pub unsafe fn new_write_raw<W: Word>(
            _channel: Peri<'a, AnyChannel>,
            _request: Request,
            _buf: *const [W],
            _peri_addr: *mut W,
            _options: TransferOptions,
        ) -> Self {
            Self::unimpl()
        }

        pub unsafe fn new_write_repeated<W: Word>(
            _channel: Peri<'a, AnyChannel>,
            _request: Request,
            _repeated: &'a W,
            _count: usize,
            _peri_addr: *mut W,
            _options: TransferOptions,
        ) -> Self {
            Self::unimpl()
        }

        pub fn get_remaining_transfers(&self) -> u16 {
            Self::unimpl()
        }
    }

    impl Future for Transfer<'_> {
        type Output = ();
        fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
            unreachable!()
        }
    }
}

#[cfg(ch32h4)]
pub use h4_stub::{Transfer, TransferOptions};

pub mod word;

mod util;
pub(crate) use util::*;

pub(crate) mod ringbuffer;

/// "No DMA" placeholder.
///
/// You may pass this in place of a real DMA channel when creating a driver
/// to indicate it should not use DMA.
///
/// This often causes async functionality to not be available on the instance,
/// leaving only blocking functionality.
pub struct NoDma;

impl_peripheral!(NoDma);

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Dir {
    MemoryToPeripheral,
    PeripheralToMemory,
}

pub type Request = ();

pub(crate) trait SealedChannel {
    fn id(&self) -> u8;
}

pub(crate) trait ChannelInterrupt {
    #[cfg_attr(not(feature = "rt"), allow(unused))]
    unsafe fn on_irq();
}

/// DMA channel.
#[allow(private_bounds)]
pub trait Channel: SealedChannel + PeripheralType + Into<AnyChannel> + 'static {
    /// Type-erase (degrade) this pin into an `AnyChannel`.
    ///
    /// This converts DMA channel singletons (`DMA1_CH3`, `DMA2_CH1`, ...), which
    /// are all different types, into the same type. It is useful for
    /// creating arrays of channels, or avoiding generics.
    #[inline]
    fn degrade(self) -> AnyChannel {
        AnyChannel { id: self.id() }
    }
}

macro_rules! dma_channel_impl {
    ($channel_peri:ident, $index:expr) => {
        impl crate::dma::SealedChannel for crate::peripherals::$channel_peri {
            fn id(&self) -> u8 {
                $index
            }
        }
        impl crate::dma::ChannelInterrupt for crate::peripherals::$channel_peri {
            unsafe fn on_irq() {
                crate::dma::AnyChannel { id: $index }.on_irq();
            }
        }

        impl crate::dma::Channel for crate::peripherals::$channel_peri {}

        impl From<crate::peripherals::$channel_peri> for crate::dma::AnyChannel {
            fn from(x: crate::peripherals::$channel_peri) -> Self {
                crate::dma::Channel::degrade(x)
            }
        }
    };
}
/// Type-erased DMA channel.
pub struct AnyChannel {
    pub(crate) id: u8,
}
impl_peripheral!(AnyChannel);

#[cfg(not(ch32h4))]
impl AnyChannel {
    fn info(&self) -> &ChannelInfo {
        &crate::_generated::DMA_CHANNELS[self.id as usize]
    }
}

impl SealedChannel for AnyChannel {
    fn id(&self) -> u8 {
        self.id
    }
}
impl Channel for AnyChannel {}

#[cfg(not(ch32h4))]
const CHANNEL_COUNT: usize = crate::_generated::DMA_CHANNELS.len();
#[cfg(not(ch32h4))]
static STATE: [ChannelState; CHANNEL_COUNT] = [ChannelState::NEW; CHANNEL_COUNT];

// TODO: replace transmutes with core::ptr::metadata once it's stable
#[allow(unused)]
pub(crate) fn slice_ptr_parts<T>(slice: *const [T]) -> (usize, usize) {
    unsafe { mem::transmute(slice) }
}

#[allow(unused)]
pub(crate) fn slice_ptr_parts_mut<T>(slice: *mut [T]) -> (usize, usize) {
    unsafe { mem::transmute(slice) }
}

// safety: must be called only once at startup
pub(crate) unsafe fn init(
    #[cfg_attr(ch32h4, allow(unused))] cs: critical_section::CriticalSection,
    #[cfg_attr(ch32h4, allow(unused))] dma_priority: interrupt::Priority,
) {
    #[cfg(not(ch32h4))]
    dma_bdma::init(cs, dma_priority);
}
