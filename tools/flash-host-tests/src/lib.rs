#![allow(
    dead_code,
    unused_imports,
    unused_variables,
    unused_macros,
    unused_attributes,
    nonstandard_style,
    unexpected_cfgs,
    static_mut_refs
)]

extern crate self as qingke;
pub use embassy_hal_internal::{impl_peripheral, Peri};

#[path = "../../host-tests-common/common_word.rs"]
pub mod common;
#[path = "../../../src/internal/drop.rs"]
pub mod drop_impl;
pub mod internal {
    pub use crate::drop_impl as drop;
}

pub fn record_read(address: usize) {
    flash_model::read(address);
}

pub fn record_write(address: usize, value: u32) -> bool {
    flash_model::write(address, value)
}

#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/flash_v4x7.rs"]
pub mod flash_registers;

pub mod pac {
    pub use crate::flash_registers as flash;
    pub static mut FLASH_WORDS: [u32; 16] = [0; 16];
    pub const FLASH: flash::Flash = unsafe { flash::Flash::from_ptr(core::ptr::addr_of_mut!(FLASH_WORDS).cast()) };
}

pub mod peripherals {
    pub struct FLASH;
    crate::impl_peripheral!(FLASH);
}

pub mod flash {
    include!(concat!(env!("OUT_DIR"), "/flash.rs"));
}

mod flash_model;
