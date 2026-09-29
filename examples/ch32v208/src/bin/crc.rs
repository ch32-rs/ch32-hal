#![no_std]
#![no_main]

use ch32_hal as hal;
use hal::crc::Crc;
use hal::println;
use panic_halt as _;
use qingke::riscv;

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    let mut crc = Crc::new(p.CRC);

    // CRC-32 (poly 0x04C11DB7, init 0xFFFF_FFFF, no reflection).
    // Words are fed MSB-first, equivalent to the big-endian byte stream "12345678".
    crc.update_words(&[0x3132_3334, 0x3536_3738]);
    let result = crc.result();
    println!("crc(\"12345678\") = {:#010x}", result);
    assert_eq!(result, 0x49e3_c2fb);

    crc.reset();
    crc.update_word(0x4142_4344); // "ABCD"
    let result = crc.result();
    println!("crc(\"ABCD\") = {:#010x}", result);
    assert_eq!(result, 0xabcf_9a63);

    println!("CRC test success");
    loop {
        riscv::asm::delay(1_000_000);
    }
}
