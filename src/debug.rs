//! The debug module.
//!
//! See-also: https://github.com/openwch/ch32v003/blob/main/EVT/EXAM/SDI_Printf/SDI_Printf/Debug/debug.c

use qingke::riscv;

#[cfg(any(qingke_v3, qingke_v4))]
mod regs {
    pub const DEBUG_DATA0_ADDRESS: *mut u32 = 0xE000_0380 as *mut u32;
    pub const DEBUG_DATA1_ADDRESS: *mut u32 = 0xE000_0384 as *mut u32;
}

#[cfg(qingke_v2)]
mod regs {
    pub const DEBUG_DATA0_ADDRESS: *mut u32 = 0xE00000F4 as *mut u32;
    pub const DEBUG_DATA1_ADDRESS: *mut u32 = 0xE00000F8 as *mut u32;
}

pub struct SDIPrint;

impl SDIPrint {
    pub fn enable() {
        unsafe {
            // Enable SDI print
            core::ptr::write_volatile(regs::DEBUG_DATA0_ADDRESS, 0);
            riscv::asm::delay(100000);
        }
    }

    #[inline]
    fn is_busy() -> bool {
        unsafe { core::ptr::read_volatile(regs::DEBUG_DATA0_ADDRESS) != 0 }
    }
}

impl core::fmt::Write for SDIPrint {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let mut data = [0u8; 8];
        for chunk in s.as_bytes().chunks(7) {
            data[1..chunk.len() + 1].copy_from_slice(chunk);
            data[0] = chunk.len() as u8;

            // data1 is the last 4 bytes of data
            let data1 = u32::from_le_bytes(data[4..].try_into().unwrap());
            let data0 = u32::from_le_bytes(data[..4].try_into().unwrap());

            while SDIPrint::is_busy() {}

            unsafe {
                core::ptr::write_volatile(regs::DEBUG_DATA1_ADDRESS, data1);
                core::ptr::write_volatile(regs::DEBUG_DATA0_ADDRESS, data0);
            }
        }

        Ok(())
    }
}

#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write;
            use core::writeln;

            writeln!(&mut $crate::debug::SDIPrint, $($arg)*).unwrap();
        }
    }
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write;
            use core::write;

            write!(&mut $crate::debug::SDIPrint, $($arg)*).unwrap();
        }
    }
}
