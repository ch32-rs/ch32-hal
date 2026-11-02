#![no_std]
#![no_main]

use ch32_hal::i2c::{self, Duty, Error, I2c};
use ch32_hal::mode::{Async, Blocking};
use ch32_hal::time::Hertz;
use ch32_hal::{bind_interrupts, pac, peripherals, rcc};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C_EV => i2c::EventInterruptHandler<peripherals::I2C>;
    I2C_ER => i2c::ErrorInterruptHandler<peripherals::I2C>;
});

#[no_mangle]
pub static mut I2C_RESULT: u32 = 0;
#[no_mangle]
pub static mut I2C_DEVICE_RESULT: u32 = 0;
#[no_mangle]
pub static mut I2C_SNAPSHOTS: [[u32; 4]; 6] = [[0; 4]; 6];
#[no_mangle]
pub static mut I2C_PROBES: [u32; 3] = [0; 3];

fn check(condition: bool, code: u32) {
    if !condition {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(I2C_RESULT), code) };
        defmt::panic!("I2C_RESULT=FAIL code={}", code);
    }
}

fn check_timing(index: usize, frequency: u32, duty: Duty) {
    let clock = rcc::clocks().pclk1.0;
    let range = (clock / 1_000_000).min(60);
    let fast = frequency > 100_000;
    let factor = if !fast {
        2
    } else if duty == Duty::Duty2_1 {
        3
    } else {
        25
    };
    let expected = u64::from(clock)
        .div_ceil(u64::from(frequency) * factor)
        .max(if fast { 1 } else { 4 });
    let control = pac::I2C.ctlr2().read();
    let divider = pac::I2C.ckcfgr().read();
    let rise = pac::I2C.rtr().read().trise();
    check(u32::from(control.freq()) == range, 10);
    check(u64::from(divider.ccr()) == expected && divider.f_s() == fast, 11);
    check(divider.duty() == (fast && duty == Duty::Duty16_9), 12);
    check(u32::from(rise) == if fast { range * 3 / 10 + 1 } else { range + 1 }, 13);
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(I2C_SNAPSHOTS[index]),
            [clock, control.0, divider.0, u32::from(rise)],
        )
    };
}

fn check_idle() {
    let control = pac::I2C.ctlr2().read();
    check(
        !control.dmaen() && !control.itevten() && !control.iterren() && !control.itbufen(),
        14,
    );
    check(
        !pac::DMA1.ch(5).cfgr().read().en() && !pac::DMA1.ch(6).cfgr().read().en(),
        15,
    );
}

#[cfg(feature = "i2c-eeprom")]
async fn check_eeprom(bus: &mut I2c<'_, peripherals::I2C, Async>, baseline: &[u8; 32]) {
    let mut buffer = [0; 32];
    for length in [1usize, 2, 3, 4, 16, 32] {
        check(bus.write_read(0x50, &[0], &mut buffer[..length]).await.is_ok(), 25);
        check(buffer[..length] == baseline[..length], 20);
    }
    let mut first = [0; 8];
    let mut second = [0; 1];
    let mut third = [0; 2];
    let result = bus
        .transaction(
            0x50,
            &mut [
                embedded_hal::i2c::Operation::Write(&[0]),
                embedded_hal::i2c::Operation::Read(&mut first),
                embedded_hal::i2c::Operation::Read(&mut second),
                embedded_hal::i2c::Operation::Read(&mut third),
            ],
        )
        .await;
    check(result.is_ok(), 26);
    check(
        first == baseline[..8] && second == baseline[8..9] && third == baseline[9..11],
        21,
    );
    let mut next = [0; 8];
    let result = bus
        .transaction(
            0x50,
            &mut [
                embedded_hal::i2c::Operation::Write(&[0]),
                embedded_hal::i2c::Operation::Read(&mut first),
                embedded_hal::i2c::Operation::Write(&[8]),
                embedded_hal::i2c::Operation::Read(&mut next),
            ],
        )
        .await;
    check(result.is_ok() && first == baseline[..8] && next == baseline[8..16], 30);
    let mut long = [0; 128];
    let _ = select(bus.read(0x50, &mut long), Timer::after_micros(50)).await;
    check_idle();
    check(bus.write_read(0x50, &[0], &mut buffer).await.is_ok(), 27);
    check(&buffer == baseline, 22);
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(I2C_DEVICE_RESULT), 2) };
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(I2C_RESULT), 1) };
    qingke::riscv::asm::delay(2_000_000);
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    }
    if cfg!(feature = "clock-dividers") {
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    }
    let mut peripherals = ch32_hal::init(config);
    #[cfg(feature = "i2c-eeprom")]
    let mut baseline = [0; 32];
    for (index, (frequency, duty)) in [
        (100_000, Duty::Duty2_1),
        (400_000, Duty::Duty2_1),
        (400_000, Duty::Duty16_9),
    ]
    .into_iter()
    .enumerate()
    {
        let mut config = i2c::Config::default();
        config.timeout = Duration::from_millis(10);
        config.duty = duty;
        {
            let mut bus = I2c::<_, Blocking>::new_blocking::<0>(
                peripherals.I2C.reborrow(),
                peripherals.PB6.reborrow(),
                peripherals.PB7.reborrow(),
                Hertz(frequency),
                config,
            );
            check_timing(index, frequency, duty);
            #[cfg(feature = "i2c-eeprom")]
            {
                let mut bytes = [0; 32];
                check(bus.blocking_write_read(0x50, &[0], &mut bytes).is_ok(), 28);
                if index == 0 {
                    baseline = bytes;
                } else {
                    check(bytes == baseline, 23);
                }
                for length in [1usize, 2, 3, 4, 16] {
                    check(bus.blocking_write_read(0x50, &[0], &mut bytes[..length]).is_ok(), 29);
                    check(bytes[..length] == baseline[..length], 24);
                }
            }
            let _ = &mut bus;
        }
        {
            let bus = I2c::<_, Blocking>::new_blocking::<1>(
                peripherals.I2C.reborrow(),
                peripherals.PB8.reborrow(),
                peripherals.PB9.reborrow(),
                Hertz(frequency),
                config,
            );
            check_timing(index + 3, frequency, duty);
            check(pac::AFIO.pcfr1().read().i2c_rm(), 16);
            drop(bus);
        }
        let mut bus = I2c::<_, Async>::new::<0>(
            peripherals.I2C.reborrow(),
            peripherals.PB6.reborrow(),
            peripherals.PB7.reborrow(),
            Irqs,
            peripherals.DMA1_CH6.reborrow(),
            peripherals.DMA1_CH7.reborrow(),
            Hertz(frequency),
            config,
        );
        check(!pac::AFIO.pcfr1().read().i2c_rm(), 17);
        let result = bus.write(0x55, &[]).await;
        let observed = match result {
            Ok(()) => 0,
            Err(Error::Nack) => 1,
            Err(Error::Timeout) => 2,
            Err(_) => 3,
        };
        check(observed != 3, 18);
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(I2C_PROBES[index]), observed) };
        check_idle();
        if let Either::Second(()) = select(bus.write(0x55, &[]), Timer::after_micros(1)).await {
            check_idle();
        }
        #[cfg(feature = "i2c-eeprom")]
        check_eeprom(&mut bus, &baseline).await;
    }
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(I2C_RESULT), 2) };
    defmt::info!(
        "I2C_RESULT=PASS configuration/cleanup; external-device tests enabled={}",
        cfg!(feature = "i2c-eeprom")
    );
    loop {
        Timer::after_secs(1).await;
    }
}
