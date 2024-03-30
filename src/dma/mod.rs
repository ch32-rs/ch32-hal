use crate::{impl_peripheral, Peripheral};

pub mod word;

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

pub type Request = u8;

pub(crate) trait SealedChannel {
    fn id(&self) -> u8;
}

pub(crate) trait ChannelInterrupt {
    #[cfg_attr(not(feature = "rt"), allow(unused))]
    unsafe fn on_irq();
}

/// DMA channel.
#[allow(private_bounds)]
pub trait Channel: SealedChannel + Peripheral<P = Self> + Into<AnyChannel> + 'static {
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

/// Type-erased DMA channel.
pub struct AnyChannel {
    pub(crate) id: u8,
}
impl_peripheral!(AnyChannel);

impl AnyChannel {
    //fn info(&self) -> &ChannelInfo {
    //    &crate::_generated::DMA_CHANNELS[self.id as usize]
    //}
}

impl SealedChannel for AnyChannel {
    fn id(&self) -> u8 {
        self.id
    }
}
impl Channel for AnyChannel {}
