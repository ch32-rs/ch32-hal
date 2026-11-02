#![no_std]
#![no_main]

use ch32_hal::usbhs::{Driver, InterruptHandler};
use ch32_hal::{bind_interrupts, peripherals, rcc};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use panic_halt as _;

#[cfg(all(feature = "usbhs2", feature = "ch32v467ret6"))]
compile_error!("CH32V467RET6 does not expose USBHS2; use USBHS1");

#[cfg(not(feature = "usbhs2"))]
bind_interrupts!(struct Irqs {
    USBHS1 => InterruptHandler<peripherals::USBHS1>;
});

#[cfg(feature = "usbhs2")]
bind_interrupts!(struct Irqs {
    USBHS2 => InterruptHandler<peripherals::USBHS2>;
});

#[no_mangle]
pub static mut USBHS_STAGE: u32 = 0;
#[no_mangle]
pub static mut USBHS_ECHO_PACKETS: u32 = 0;
#[no_mangle]
pub static mut USBHS_ECHO_BYTES: u32 = 0;
#[no_mangle]
pub static mut USBHS_ERRORS: u32 = 0;

#[repr(align(4))]
struct EndpointMemory([u8; 512]);

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(USBHS_STAGE), 1) };
    let mut config = ch32_hal::Config::default();
    config.rcc.hspll = Some(rcc::HsPllSource::HSI);
    if cfg!(feature = "clock-hse") {
        config.rcc.hse = Some(rcc::Hse {
            freq: ch32_hal::time::Hertz(25_000_000),
            mode: rcc::HseMode::Oscillator,
        });
        config.rcc.hspll = Some(rcc::HsPllSource::HSE);
    }
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    }
    let peripherals = ch32_hal::init(config);
    let mut memory = EndpointMemory([0; 512]);
    #[cfg(not(feature = "usbhs2"))]
    let driver = Driver::new(
        peripherals.USBHS1,
        peripherals.PA12,
        peripherals.PA11,
        Irqs,
        &mut memory.0,
        Default::default(),
    )
    .unwrap();
    #[cfg(feature = "usbhs2")]
    let driver = Driver::new(
        peripherals.USBHS2,
        peripherals.PB7,
        peripherals.PB6,
        Irqs,
        &mut memory.0,
        Default::default(),
    )
    .unwrap();
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("ch32-hal");
    config.product = Some("CH32V4 USBHS CDC echo");
    config.serial_number = Some(if cfg!(feature = "usbhs2") {
        "V4-USBHS2"
    } else {
        "V4-USBHS1"
    });
    config.max_power = 100;
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 64];
    let mut msos_descriptor = [0; 64];
    let mut control = [0; 64];
    let mut state = State::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control,
    );
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(USBHS_STAGE), 2) };
    defmt::info!(
        "USBHS CDC ready: instance={} full-speed",
        if cfg!(feature = "usbhs2") { 2 } else { 1 }
    );
    let echo = async {
        loop {
            class.wait_connection().await;
            unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(USBHS_STAGE), 3) };
            defmt::info!("USBHS CDC connected");
            let mut packet = [0; 64];
            loop {
                let result = async {
                    let length = class.read_packet(&mut packet).await?;
                    class.write_packet(&packet[..length]).await?;
                    if length == packet.len() {
                        class.write_packet(&[]).await?;
                    }
                    Ok::<_, EndpointError>(length)
                }
                .await;
                match result {
                    Ok(length) => unsafe {
                        let packets = core::ptr::addr_of_mut!(USBHS_ECHO_PACKETS);
                        core::ptr::write_volatile(packets, core::ptr::read_volatile(packets).wrapping_add(1));
                        let bytes = core::ptr::addr_of_mut!(USBHS_ECHO_BYTES);
                        core::ptr::write_volatile(bytes, core::ptr::read_volatile(bytes).wrapping_add(length as u32));
                        core::ptr::write_volatile(core::ptr::addr_of_mut!(USBHS_STAGE), 4);
                    },
                    Err(error) => {
                        if error == EndpointError::BufferOverflow {
                            unsafe {
                                let errors = core::ptr::addr_of_mut!(USBHS_ERRORS);
                                core::ptr::write_volatile(errors, core::ptr::read_volatile(errors).wrapping_add(1));
                            }
                            defmt::error!("USBHS CDC buffer overflow");
                        }
                        break;
                    }
                }
            }
            unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(USBHS_STAGE), 2) };
        }
    };
    join(usb.run(), echo).await;
}
