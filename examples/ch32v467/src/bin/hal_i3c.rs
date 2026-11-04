#![no_std]
#![no_main]

use ch32_hal::i3c::{self, Config, I3c, Target, Timings};
use ch32_hal::{bind_interrupts, pac, peripherals, rcc};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;
use ch32v467_examples::{report_check, report_set, report_static};

bind_interrupts!(struct Irqs {
    I3C_EV => i3c::EventInterruptHandler<peripherals::I3C>;
    I3C_ER => i3c::ErrorInterruptHandler<peripherals::I3C>;
});

report_static!(I3C_RESULT);
report_static!(I3C_DEVICE_RESULT);
report_static!(I3C_DMA_RESULT);
#[no_mangle]
pub static mut I3C_TARGET_COUNT: u32 = 0;
#[no_mangle]
pub static mut I3C_TARGETS: [Target; 4] = [Target {
    dynamic_address: 0,
    pid: [0; 6],
    bcr: 0,
    dcr: 0,
}; 4];

fn check(condition: bool, code: u32) {
    report_check!(I3C_RESULT, condition, code, "I3C_RESULT check failed");
}

#[cfg(feature = "i3c-daa")]
fn raw_word(value: Option<&str>) -> u32 {
    let value = value.expect("i3c-daa requires I3C_TIMINGR0 and I3C_TIMINGR1");
    if let Some(hex) = value.strip_prefix("0x") {
        u32::from_str_radix(hex, 16).expect("invalid hex I3C timing")
    } else {
        value.parse().expect("invalid decimal I3C timing")
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    report_set!(I3C_RESULT, 1);
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-dividers") {
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    }
    let mut peripherals = ch32_hal::init(config);
    let hclk = rcc::clocks().hclk.0;

    #[cfg(feature = "i3c-daa")]
    let (timing0, timing1) = (
        raw_word(option_env!("I3C_TIMINGR0")),
        raw_word(option_env!("I3C_TIMINGR1")),
    );
    #[cfg(not(feature = "i3c-daa"))]
    let (timing0, timing1) = (0x88888889u32, 0x10770200 | (hclk / 1_000_000 - 1));

    check(timing1 & !0x107f03ff == 0, 10);
    let settings = Config::new(Timings {
        hclk_hz: hclk,
        pp_low: timing0 as u8,
        i3c_high: (timing0 >> 8) as u8,
        od_low: (timing0 >> 16) as u8,
        i2c_high: (timing0 >> 24) as u8,
        aval: timing1 as u8,
        free: ((timing1 >> 16) & 127) as u8,
        asncr: ((timing1 >> 8) & 3) as u8,
        sda_hold_1_5: timing1 & (1 << 28) != 0,
    });

    #[cfg(not(feature = "i3c-remap"))]
    let (scl, sda) = (peripherals.PB10, peripherals.PB11);
    #[cfg(feature = "i3c-remap")]
    let (scl, sda) = (peripherals.PB4, peripherals.PB5);
    #[cfg(not(feature = "i3c-dma"))]
    let mut driver = I3c::new(peripherals.I3C.reborrow(), scl, sda, Irqs, settings).unwrap();
    #[cfg(feature = "i3c-dma")]
    let mut driver = I3c::new_with_dma(
        peripherals.I3C.reborrow(),
        scl,
        sda,
        Irqs,
        peripherals.DMA1_CH4,
        peripherals.DMA1_CH5,
        settings,
    )
    .unwrap();

    check(pac::I3C.timingr0().read().0 == timing0, 11);
    check(pac::I3C.timingr1().read().0 == timing1, 12);
    check(pac::I3C.timingr2().read().0 == 0, 13);
    check(pac::I3C.owndevr().read().0 == 0x45, 14);
    check(pac::I3C.intenr().read().0 == 0, 15);
    check(pac::I3C.cfgr().read().0 == (1 << 19) | (1 << 6) | 3, 16);
    driver.reinitialize().unwrap();
    check(!driver.needs_recovery(), 17);

    #[cfg(feature = "i3c-daa")]
    {
        report_set!(I3C_DEVICE_RESULT, 1);
        driver.broadcast_ccc(0x06, &[]).await.unwrap();
        let mut targets = [Target::default(); 4];
        let count = driver
            .assign_addresses(&[0x30, 0x31, 0x32, 0x33], &mut targets)
            .await
            .unwrap();
        check(count > 0, 20);
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(I3C_TARGET_COUNT), count as u32);
            core::ptr::write_volatile(core::ptr::addr_of_mut!(I3C_TARGETS), targets);
        }
        for target in &targets[..count] {
            let mut pid = [0; 6];
            driver
                .direct_ccc(
                    0x8d,
                    target.dynamic_address,
                    embedded_hal::i2c::Operation::Read(&mut pid),
                )
                .await
                .unwrap();
            check(pid == target.pid, 21);
        }
        #[cfg(feature = "i3c-dma-loopback")]
        {
            use embedded_hal::i2c::Operation;
            report_set!(I3C_DMA_RESULT, 1);
            let address = targets[0].dynamic_address;
            let mut transmit = [0u8; 257];
            let mut receive = [0u8; 257];
            for length in [1usize, 7, 8, 9, 31, 257] {
                for (index, byte) in transmit[..length].iter_mut().enumerate() {
                    *byte = (index as u8).wrapping_add(length as u8);
                }
                driver
                    .private_transaction(address, &mut [Operation::Write(&transmit[..length])])
                    .await
                    .unwrap();
                driver
                    .private_transaction(address, &mut [Operation::Read(&mut receive[..length])])
                    .await
                    .unwrap();
                check(transmit[..length] == receive[..length], 30);
                let mut operations = [
                    Operation::Write(&transmit[..length]),
                    Operation::Read(&mut receive[..length]),
                ];
                driver.private_transaction(address, &mut operations).await.unwrap();
                check(transmit[..length] == receive[..length], 31);
                check(
                    !pac::DMA1.ch(3).cfgr().read().en() && !pac::DMA1.ch(4).cfgr().read().en(),
                    32,
                );
                check(pac::I3C.cfgr().read().0 & ((1 << 8) | (1 << 12)) == 0, 33);
            }
            report_set!(I3C_DMA_RESULT, 2);
            defmt::info!("I3C_DMA_RESULT=PASS echo fixture; bus speed remains unqualified");
        }
        report_set!(I3C_DEVICE_RESULT, 2);
        defmt::info!("I3C_DEVICE_RESULT=PASS DAA and GETPID targets={}", count);
    }

    drop(driver);
    check(!pac::RCC.hbpcenr().read().i3cen(), 18);
    report_set!(I3C_RESULT, 2);
    defmt::info!("I3C_RESULT=PASS register/lifecycle checks; hclk={}", hclk);
    if !cfg!(feature = "i3c-daa") {
        defmt::info!("I3C device transfers not run; raw fixture timings are not a qualified bus profile");
    }
    loop {
        Timer::after_secs(1).await;
    }
}
