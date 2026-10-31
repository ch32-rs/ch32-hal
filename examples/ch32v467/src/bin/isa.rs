#![no_std]
#![no_main]

use core::arch::asm;
use core::hint::black_box;

use ch32v467_examples as board;
use defmt::{error, info};
use defmt_rtt as _;
use panic_halt as _;
use qingke_rt::entry;

#[cfg(not(all(target_feature = "zve64x", target_feature = "zvl64b", target_feature = "zvbb")))]
compile_error!("Run from examples/ch32v467 with its .cargo/config.toml target features");

#[unsafe(no_mangle)]
pub static mut ISA_RESULT: u32 = 0;

#[unsafe(no_mangle)]
pub extern "C" fn ExceptionHandler() {
    let cause: u32;
    let epc: u32;
    let tval: u32;
    unsafe {
        asm!("csrr {}, mcause", out(reg) cause);
        asm!("csrr {}, mepc", out(reg) epc);
        asm!("csrr {}, mtval", out(reg) tval);
        core::ptr::write_volatile(&raw mut ISA_RESULT, 3);
    }
    error!(
        "ISA TRAP: mcause={=u32:#x} mepc={=u32:#x} mtval={=u32:#x}",
        cause, epc, tval
    );
    loop {
        core::hint::spin_loop();
    }
}

#[inline(never)]
#[unsafe(no_mangle)]
pub extern "C" fn isa_copy_byte(destination: &mut u8, source: &u8) {
    unsafe { core::ptr::write_volatile(destination, core::ptr::read_volatile(source)) };
}

#[inline(never)]
#[unsafe(no_mangle)]
pub extern "C" fn isa_copy_halfword(destination: &mut u16, source: &u16) {
    unsafe { core::ptr::write_volatile(destination, core::ptr::read_volatile(source)) };
}

fn check_xwchc() -> bool {
    let mut words = [0x5555u16; 2];
    let byte: u32;
    let halfword: u32;
    unsafe {
        asm!(
            "qk.c.sb a1, 0(a0)",
            "qk.c.lbu a2, 0(a0)",
            "qk.c.sh a1, 2(a0)",
            "qk.c.lhu a3, 2(a0)",
            in("a0") words.as_mut_ptr(),
            in("a1") 0x89abu32,
            out("a2") byte,
            out("a3") halfword,
            options(nostack),
        );
    }
    let mut copied_byte = 0;
    let mut copied_halfword = 0;
    isa_copy_byte(black_box(&mut copied_byte), black_box(&(byte as u8)));
    isa_copy_halfword(black_box(&mut copied_halfword), black_box(&0x89abu16));
    let passed = byte == 0xab
        && halfword == 0x89ab
        && words == [0x55ab, 0x89ab]
        && copied_byte == 0xab
        && copied_halfword == 0x89ab;
    info!("Xwchc byte/halfword load/store + Rust copies: pass={}", passed);
    passed
}

macro_rules! vector_check {
    ($name:ident, $element:ty, $width:literal, $load:literal, $store:literal) => {
        #[inline(never)]
        fn $name() -> bool {
            let left: [$element; 19] = core::array::from_fn(|index| {
                (<$element>::MAX).wrapping_sub((index as $element).wrapping_mul(17))
            });
            let right: [$element; 19] = core::array::from_fn(|index| {
                (index as $element).wrapping_mul(29).wrapping_add(5)
            });
            let mut sums = [0 as $element; 19];
            let mut rotations = [0 as $element; 19];
            let mut offset = 0;
            let mut chunks = 0;
            while offset < left.len() {
                let remaining = left.len() - offset;
                let length: usize;
                unsafe {
                    asm!(
                        concat!("vsetvli {length}, {remaining}, e", $width, ", m1, ta, ma"),
                        concat!($load, " v8, ({left})"),
                        concat!($load, " v9, ({right})"),
                        "vadd.vv v10, v8, v9",
                        concat!($store, " v10, ({sums})"),
                        "vror.vi v10, v8, 3",
                        concat!($store, " v10, ({rotations})"),
                        length = out(reg) length,
                        remaining = in(reg) remaining,
                        left = in(reg) left.as_ptr().add(offset),
                        right = in(reg) right.as_ptr().add(offset),
                        sums = in(reg) sums.as_mut_ptr().add(offset),
                        rotations = in(reg) rotations.as_mut_ptr().add(offset),
                        out("v8") _,
                        out("v9") _,
                        out("v10") _,
                        options(nostack),
                    );
                }
                if length == 0 || length > remaining {
                    error!("e{}: invalid vl={}", $width, length);
                    return false;
                }
                offset += length;
                chunks += 1;
            }
            for index in 0..left.len() {
                let expected_sum = black_box(left[index]).wrapping_add(black_box(right[index]));
                let expected_rotation = black_box(left[index]).rotate_right(3);
                if sums[index] != expected_sum || rotations[index] != expected_rotation {
                    error!("e{} mismatch at {}: sum={} expected={}, rotate={} expected={}",
                        $width, index, sums[index], expected_sum, rotations[index], expected_rotation);
                    return false;
                }
            }
            info!("Zve64x vadd + Zvbb vror e{}: PASS ({} elements, {} chunks)",
                $width, left.len(), chunks);
            true
        }
    };
}

vector_check!(check_e8, u8, 8, "vle8.v", "vse8.v");
vector_check!(check_e16, u16, 16, "vle16.v", "vse16.v");
vector_check!(check_e32, u32, 32, "vle32.v", "vse32.v");
vector_check!(check_e64, u64, 64, "vle64.v", "vse64.v");

#[entry]
fn main() -> ! {
    board::init();
    board::delay_ms(500);
    unsafe { core::ptr::write_volatile(&raw mut ISA_RESULT, 1) };
    info!("CH32V467 ISA behavior test: +xwchc,+zve64x,+zvl64b,+zvbb");
    let status: u32;
    let vector_bytes: u32;
    unsafe {
        asm!("csrr {}, mstatus", out(reg) status);
        asm!("csrr {}, vlenb", out(reg) vector_bytes);
    }
    info!(
        "mstatus={=u32:#010x}, VS={}, VLEN={} bits",
        status,
        (status >> 9) & 3,
        vector_bytes * 8
    );
    let mut passed = vector_bytes >= 8 && (status >> 9) & 3 != 0;
    passed &= check_xwchc();
    passed &= check_e8();
    passed &= check_e16();
    passed &= check_e32();
    passed &= check_e64();
    unsafe { core::ptr::write_volatile(&raw mut ISA_RESULT, if passed { 2 } else { 3 }) };
    if passed {
        info!("ISA RESULT: PASS");
    } else {
        error!("ISA RESULT: FAIL");
    }
    let mut ticks = 0u32;
    loop {
        ticks = ticks.wrapping_add(1);
        board::heartbeat(ticks);
        board::delay_ms(1000);
    }
}
