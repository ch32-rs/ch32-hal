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

// Two pin-trait shapes, selected per chip. They never coexist in one build.
//
// - cfg(not(afio_h4)): PCFR remap. `trait Signal<T, const REMAP: u8 = 0>`.
//   TX and RX share one const, same as the published API. `apply_remap!()`
//   writes the group via `RemapPeripheral::set_remap` (patches cover split
//   fields). Every pin entry is its own impl, so one pin can implement
//   several signals and several groups.
// - cfg(afio_h4): per-pin AF in `AFIO.GPIO_AFR`. The trait has no remap
//   parameter; `af_num()` is the value `build.rs` read from `pin.af`.
//
// `if_remap!(impl TxPin<T, REMAP>)` keeps the const in the signature on
// remap chips and drops it on H4.
macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        #[cfg(not(afio_h4))]
        pub trait $signal<T: $instance, const REMAP: u8 = 0>: crate::gpio::Pin {}

        #[cfg(afio_h4)]
        pub trait $signal<T: $instance>: crate::gpio::Pin {
            #[doc = concat!("AF number for `", stringify!($signal), "`.")]
            fn af_num(&self) -> u8;
        }
    };
}

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $n:expr) => {
        #[cfg(not(afio_h4))]
        impl crate::$mod::$trait<crate::peripherals::$instance, $n> for crate::peripherals::$pin {}

        #[cfg(afio_h4)]
        impl crate::$mod::$trait<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn af_num(&self) -> u8 {
                $n
            }
        }
    };
}

#[cfg(not(afio_h4))]
macro_rules! if_remap {
    ($($t:tt)*) => {
        $($t)*
    };
}

#[cfg(afio_h4)]
macro_rules! if_remap {
    (impl $trait:ident<$a:ty, REMAP>) => {
        impl $trait<$a>
    };
    (impl $trait:ident<$a:ty, $b:ty, REMAP>) => {
        impl $trait<$a, $b>
    };
}

/// Write `AFIO.PCFR*` for the `REMAP` const in scope. No-op on CH32H4,
/// where the mux is the per-pin AF number instead.
macro_rules! apply_remap {
    () => {
        #[cfg(not(afio_h4))]
        {
            fn apply<P: crate::peripheral::RemapPeripheral, const R: u8>() {
                P::set_remap(R);
            }
            apply::<T, REMAP>();
        }
    };
}

/// Configure a pin for AF use and consume it into `Option<Peri<'d, AnyPin>>`.
///
/// Mode/cnf is written on every family. On CH32H4 the AF number is also
/// written to `AFIO.GPIO_AFR`. PCFR remap is `apply_remap!()`, not this macro,
/// because the group is a property of the peripheral, not of one pin.
macro_rules! new_pin {
    ($name:ident, $af_type:expr) => {{
        let pin = $name;
        pin.set_as_af(
            #[cfg(afio_h4)]
            pin.af_num(),
            $af_type,
        );
        Some(pin.into())
    }};
}

/// Like `new_pin!` but leaves the typed pin in place.
macro_rules! set_as_af {
    ($pin:expr, $af_type:expr) => {{
        $pin.set_as_af(
            #[cfg(afio_h4)]
            $pin.af_num(),
            $af_type,
        );
    }};
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
