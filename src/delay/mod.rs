// CH32H4 has its own systick layout (`systick_v3f_v5f`, dual-core), so
// it gets a dedicated impl. The non-H4 cases stay gated on the qingke
// core version because that's how the existing delay shims were
// organised — `impl_qingke_v2_v4` covers `systick_rv2` and `systick_rv4`
// in one file.
#[cfg(systick_v3f_v5f)]
#[path = "./impl_systick_v3f_v5f.rs"]
mod delay_impl;

#[cfg(all(any(qingke_v2, qingke_v4), not(systick_v3f_v5f)))]
#[path = "./impl_qingke_v2_v4.rs"]
mod delay_impl;

#[cfg(qingke_v3)]
#[path = "./impl_qingke_v3.rs"]
mod delay_impl;

pub use delay_impl::*;

pub unsafe fn init() {
    Delay::init();
}
