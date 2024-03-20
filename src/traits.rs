#![macro_use]

macro_rules! dma_trait {
    ($signal:ident, $instance:path$(, $mode:path)?) => {
        #[doc = concat!(stringify!($signal), " DMA request trait")]
        pub trait $signal<T: $instance $(, M: $mode)?>: crate::dma::Channel {
            #[doc = concat!("Get the DMA request number needed to use this channel as", stringify!($signal))]
            /// Note: in some chips, ST calls this the "channel", and calls channels "streams".
            /// `embassy-stm32` always uses the "channel" and "request number" names.
            fn request(&self) -> crate::dma::Request;
        }
    };
}

macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance, const REMAP: u8>: crate::gpio::Pin {}
    };
}
macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $remap:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance, $remap> for crate::peripherals::$pin {}
    };
}
