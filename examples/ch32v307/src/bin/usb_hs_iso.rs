//! USBHS isochronous loopback (vendor class).
//!
//! Demonstrates the isochronous support of the `usbhs` driver without any audio stack:
//! - alt setting 0: idle, no endpoints
//! - alt setting 1: isochronous OUT + IN, 64-byte packets, one packet per microframe
//!
//! OUT packets are drained by a task with `read()` (the interrupt fills a 16-slot ring).
//! IN packets are built in the SOF callback (every 125 us at high speed) with
//! `iso_in_fill()` and carry the IN sequence, the last OUT sequence seen, the microframe
//! number, the OUT packet counter and the OUT ring's overrun counter. An
//! isochronous-capable host can set alt setting 1 and check that IN sequences never repeat
//! and that the echoed OUT sequence only moves forward.
//!
//! Build the example with `cargo run --release --bin usb_hs_iso` and connect the board's
//! USBHS port (PB6/PB7). The Linux libusb tool used for the packet checks during
//! development is not part of the repository.

#![no_std]
#![no_main]

use core::cell::Cell;

use ch32_hal::usb::EndpointDataBuffer64;
use ch32_hal::usbhs::{
    self, endpoint_enabled, iso_in_fill, iso_out_overruns, microframe, Driver, InterruptHandler, WakeupInterruptHandler,
};
use ch32_hal::{self as hal, bind_interrupts, peripherals, Config};
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::{Direction, EndpointAddress, EndpointError, EndpointOut};
use embassy_usb::{Builder, EndpointOptions};
use panic_halt as _;

bind_interrupts!(struct Irq {
    USBHS => InterruptHandler<peripherals::USBHS>;
    USBHS_WKUP => WakeupInterruptHandler<peripherals::USBHS>;
});

const VID: u16 = 0xc0de;
const PID: u16 = 0xca51;

/// Endpoint number used for both directions (an endpoint number can be IN and OUT).
const EP_NUM: usize = 1;
const EP_SIZE: u16 = 64;
/// One buffer for EP0, `ISO_OUT_SLOTS` for the OUT ring and `ISO_IN_SLOTS` for the IN ring.
const NR_EP: usize = 1 + usbhs::ISO_OUT_SLOTS + usbhs::ISO_IN_SLOTS;

/// State shared between the SOF callback and the OUT task. The V307 has no atomic
/// read-modify-write, so counters live behind a critical section.
#[derive(Clone, Copy, Default)]
struct Stats {
    /// The IN sequence, bumped for every packet actually queued.
    in_seq: u32,
    /// The last OUT sequence the task has taken from the ring.
    last_out: u32,
    /// OUT packets taken from the ring.
    out_packets: u32,
}

static STATS: Mutex<Cell<Stats>> = Mutex::new(Cell::new(Stats {
    in_seq: 0,
    last_out: 0,
    out_packets: 0,
}));

/// Runs from the USB interrupt on every SOF (125 us at high speed): queue one IN packet
/// for the microframe, when the host has selected the streaming alt setting.
fn on_sof() {
    if !endpoint_enabled(EP_NUM, Direction::In) {
        return;
    }
    let stats = critical_section::with(|cs| STATS.borrow(cs).get());
    let microframe = microframe();
    let out_packets = stats.out_packets as u16;
    let last_out = stats.last_out;
    let overruns = iso_out_overruns(EP_NUM);
    // `iso_in_fill` does not run the closure (and the sequence does not advance) when the
    // ring is full, so no sequence number is ever skipped.
    let _ = iso_in_fill(EP_NUM, |buf| {
        let seq = critical_section::with(|cs| {
            let stats = STATS.borrow(cs).get();
            STATS.borrow(cs).set(Stats {
                in_seq: stats.in_seq.wrapping_add(1),
                ..stats
            });
            stats.in_seq
        });
        buf[0..4].copy_from_slice(&seq.to_le_bytes());
        buf[4..8].copy_from_slice(&last_out.to_le_bytes());
        buf[8..10].copy_from_slice(&microframe.to_le_bytes());
        buf[10..12].copy_from_slice(&out_packets.to_le_bytes());
        buf[12..16].copy_from_slice(&overruns.to_le_bytes());
        for (i, b) in buf[16..].iter_mut().enumerate() {
            *b = (seq as u8).wrapping_add(i as u8);
        }
        EP_SIZE as usize
    });
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE,
        ..Default::default()
    };
    let p = hal::init(cfg);

    // Before `Driver::new`, so the SOF interrupt is enabled from the start.
    usbhs::set_sof_callback(on_sof);

    let mut buffer: [EndpointDataBuffer64; NR_EP] = core::array::from_fn(|_| EndpointDataBuffer64::default());
    let driver = Driver::new(p.USBHS, Irq, p.PB7, p.PB6, &mut buffer);

    let mut config = embassy_usb::Config::new(VID, PID);
    config.manufacturer = Some("ch32-hal");
    config.product = Some("USBHS isochronous loopback");
    config.serial_number = Some("iso-1");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 64];
    let mut msos_descriptor = [0; 64];
    let mut control_buf = [0; 64];

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    let ep_out;
    {
        let mut func = builder.function(0xff, 0x00, 0x00);
        let mut iface = func.interface();
        // alt 0: no endpoints, the default the host sees after SET_CONFIGURATION
        drop(iface.alt_setting(0xff, 0x00, 0x00, None));
        // alt 1: the isochronous stream
        let mut alt = iface.alt_setting(0xff, 0x00, 0x00, None);
        ep_out = alt.endpoint_isochronous_out(
            Some(EndpointAddress::from_parts(EP_NUM, Direction::Out)),
            EP_SIZE,
            1,
            SynchronizationType::NoSynchronization,
            UsageType::DataEndpoint,
            &[],
            EndpointOptions::new(),
        );
        let _ep_in = alt.endpoint_isochronous_in(
            Some(EndpointAddress::from_parts(EP_NUM, Direction::In)),
            EP_SIZE,
            1,
            SynchronizationType::NoSynchronization,
            UsageType::DataEndpoint,
            &[],
            EndpointOptions::new(),
        );
    }

    let mut usb = builder.build();

    join(usb.run(), drain(ep_out)).await;
}

/// Take OUT packets out of the ring the interrupt fills; keep the last sequence for the
/// SOF callback to echo back.
async fn drain<E: EndpointOut>(mut ep: E) {
    let mut buf = [0u8; EP_SIZE as usize];
    loop {
        // alt setting 0 disables the endpoint; `read` fails with `Disabled` then
        ep.wait_enabled().await;
        match ep.read(&mut buf).await {
            Ok(n) if n >= 4 => {
                let seq = u32::from_le_bytes(buf[..4].try_into().unwrap());
                critical_section::with(|cs| {
                    let stats = STATS.borrow(cs).get();
                    STATS.borrow(cs).set(Stats {
                        last_out: seq,
                        out_packets: stats.out_packets.wrapping_add(1),
                        ..stats
                    });
                });
            }
            Ok(_) => {}
            // disabled by a new alt setting or a bus reset: wait for the next enable
            Err(EndpointError::Disabled) => {}
            Err(_) => {}
        }
    }
}
