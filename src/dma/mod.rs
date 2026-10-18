//! DMA
//!
//! TODO: DMA2 with CH8 to CH11, which are handled in Exxx registers

#![macro_use]

use core::mem;

use embassy_hal_internal::PeripheralType;

use crate::{impl_peripheral, interrupt};

// dma_bdma covers the V0/V1/V2/V3 "single DMA + optional BDMA" controller
// and the CH32H4 DMA1+DMA2. The register layout is the same; H4 only renames
// the accessors and adds the DMAMUX, both handled inside the driver.
#[cfg(any(bdma, dma))]
mod dma_bdma;
#[cfg(any(bdma, dma))]
pub use dma_bdma::*;

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

/// DMA request selector.
///
/// On chips with fixed channel wiring this carries nothing — the channel
/// itself is the request. On CH32H4 it is the DMAMUX request number
/// (RM table 10-2) programmed into the channel's mux slot.
#[cfg(not(ch32h4))]
pub type Request = ();
#[cfg(ch32h4)]
pub type Request = u8;

/// Marker for a channel routed through the chip's single DMAMUX.
///
/// Implemented for `DMA1_CH1`..`DMA1_CH8` and `DMA2_CH1`..`DMA2_CH8`, and
/// nothing else. Signal traits (`TxDma<USART1>` and friends) blanket-impl
/// over this, which is what keeps a request number from applying to a
/// channel that cannot carry it. It only exists as a bound: the mux slot is
/// derived from the channel's controller and number, not from the type.
#[cfg(ch32h4)]
pub trait MuxChannel: SealedMuxChannel {}

#[cfg(ch32h4)]
pub(crate) trait SealedMuxChannel {}

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

const CHANNEL_COUNT: usize = crate::_generated::DMA_CHANNELS.len();
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
pub(crate) unsafe fn init(cs: critical_section::CriticalSection, dma_priority: interrupt::Priority) {
    dma_bdma::init(cs, dma_priority);
}
