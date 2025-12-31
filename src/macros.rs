#![macro_use]
#![allow(unused_macros)]

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
        pub trait $signal<T: $instance, const REMAP: u8 = 0>: crate::gpio::Pin {}
    };
}
macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $remap:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance, $remap> for crate::peripherals::$pin {}
    };
}

#[allow(unused)]
macro_rules! dma_trait_impl {
    // DMA/GPDMA, without DMAMUX
    (crate::$mod:ident::$trait:ident$(<$mode:ident>)?, $instance:ident, {channel: $channel:ident}, $request:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance $(, crate::$mod::$mode)?> for crate::peripherals::$channel {
            fn request(&self) -> crate::dma::Request {
                $request
            }
        }
    };
}

macro_rules! new_dma {
    ($name:ident) => {{
        let request = $name.request();
        Some(crate::dma::ChannelAndRequest {
            channel: $name.into(),
            request,
        })
    }};
}

#[collapse_debuginfo(yes)]
macro_rules! panic {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::panic!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::panic!($($x)*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::trace!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::debug!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::info!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! warn {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::warn!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::error!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! unwrap {
    ($e:expr) => {{
        #[cfg(feature = "defmt")]
        {
            ::defmt::unwrap!($e)
        }
        #[cfg(not(feature = "defmt"))]
        {
            $e.unwrap()
        }
    }};
}
