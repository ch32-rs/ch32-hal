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

// pin_trait!/pin_trait_impl!/pin_trait_afio_impl!/if_afio!
//
// Mirrors embassy-stm32's macros.rs. Two pin-trait shapes coexist:
//
// - cfg(afio): the trait carries a third generic `A` whose only inhabitants
//   are `gpio::Remap<V>` / `gpio::RemapBool<V>` / `gpio::RemapNotApplicable`
//   marker structs. Drivers call `pin.afio_remap()` to write AFIO PCFR for
//   the chosen group; the marker's nominal identity forces every pin of a
//   single peripheral instance to agree on the same group at compile time.
//
// - cfg(not(afio)): the trait has no extra generic and pins expose
//   `pin.af_num()`. Drivers call it together with `set_as_af`.
//
// `if_afio!(...)` lets driver `fn new(...)` signatures keep the `, A>`
// suffix unconditionally — it's stripped out by the macro when
// not(afio).
macro_rules! pin_trait {
    ($signal:ident, $instance:path $(, $mode:path)? $(, @$afio:ident)?) => {
        #[doc = concat!(stringify!($signal), " pin trait")]
        pub trait $signal<T: $instance $(, M: $mode)? $(, #[cfg(afio)] $afio)?>: crate::gpio::Pin {
            #[cfg(not(afio))]
            #[doc = concat!("Get the AF number needed to use this pin as `", stringify!($signal),"`.")]
            fn af_num(&self) -> u8;

            #[cfg(afio)]
            #[doc = concat!("Configure the AFIO PCFR register to route `", stringify!($signal),"` to this pin.")]
            fn afio_remap(&self);
        }
    };
}

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident$(<$mode:ident>)?, $instance:ident, $pin:ident, $af:expr $(, $afio:path)?) => {
        #[cfg(afio)]
        impl crate::$mod::$trait<crate::peripherals::$instance $(, crate::$mod::$mode)? $(, $afio)?> for crate::peripherals::$pin {
            fn afio_remap(&self) {
                // this pin is fixed-function on this AFIO chip — nothing to write
            }
        }

        #[cfg(not(afio))]
        impl crate::$mod::$trait<crate::peripherals::$instance $(, crate::$mod::$mode)?> for crate::peripherals::$pin {
            fn af_num(&self) -> u8 {
                $af
            }
        }
    };
}

#[cfg(afio)]
macro_rules! pin_trait_afio_impl {
    (@set $reg:ident, $setter:ident, $val:expr) => {
        crate::pac::AFIO.$reg().modify(|w| {
            w.$setter($val);
        });
    };
    (crate::$mod:ident::$trait:ident<$mode:ident>, $instance:ident, $pin:ident, {$reg:ident, $setter:ident, $type:ident, [$($val:expr),+]}) => {
        $(
            impl crate::$mod::$trait<crate::peripherals::$instance, crate::$mod::$mode, crate::gpio::$type<$val>> for crate::peripherals::$pin {
                fn afio_remap(&self) {
                    pin_trait_afio_impl!(@set $reg, $setter, $val);
                }
            }
        )+
    };
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, {$reg:ident, $setter:ident, $type:ident, [$($val:expr),+]}) => {
        $(
            impl crate::$mod::$trait<crate::peripherals::$instance, crate::gpio::$type<$val>> for crate::peripherals::$pin {
                fn afio_remap(&self) {
                    pin_trait_afio_impl!(@set $reg, $setter, $val);
                }
            }
        )+
    };
}

#[cfg(afio)]
macro_rules! if_afio {
    ($($t:tt)*) => {
        $($t)*
    };
}

#[cfg(not(afio))]
macro_rules! if_afio {
    (impl $trait:ident<$a:ty, A>) => {
        impl $trait<$a>
    };
    (impl $trait:ident<$a:ty, $b:ty, A>) => {
        impl $trait<$a, $b>
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
