#[cfg(any(qingke_v2, qingke_v4))]
#[path = "./impl_qingke_v2_v4.rs"]
mod delay_impl;

#[cfg(qingke_v3)]
#[path = "./impl_qingke_v3.rs"]
mod delay_impl;

pub use delay_impl::*;
