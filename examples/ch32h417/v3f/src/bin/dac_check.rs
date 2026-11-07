//! DAC self-check (V3F only).
//!
//! Drives **PA4 = DAC OUT1** and **PA5 = DAC OUT2**, checks the dual 12-bit
//! write path, then arms a software trigger on both channels and reads the
//! output registers back. Mailbox (`DacCheck`): stage 1 = init returned,
//! 2 = every check ran; `failures` counts failed checks and `dor1`/`dor2` are
//! the last output values. Only the digital path is checked — the analog
//! output needs a meter or a scope.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::DacCheck;
use hal::dac::{Dac, DualValue, TriggerSel, Value};
use hal::dma::NoDma;
use hal::Peri;

ipc::mailbox!(DacCheck);
use panic_halt as _;

fn check(failures: &mut u32, condition: bool) {
    *failures += u32::from(!condition);
}

fn no_dma() -> Peri<'static, NoDma> {
    unsafe { Peri::new_unchecked(NoDma) }
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let p = hal::init(hal::Config::default());
    let mut failures = 0u32;

    let mut dac = Dac::new(p.DAC, no_dma(), no_dma(), p.PA4, p.PA5);
    let mut delay = hal::delay::Delay;

    // Untriggered writes reach DOR after the next bus clock.
    dac.set(DualValue::Bit12Right(0x123, 0xabc));
    delay.delay_us(10);
    check(&mut failures, dac.ch1().read() == 0x123);
    check(&mut failures, dac.ch2().read() == 0xabc);

    // `set_trigger` disables the channel, so re-enable before triggering.
    dac.ch1().set_trigger(TriggerSel::SOFTWARE);
    dac.ch2().set_trigger(TriggerSel::SOFTWARE);
    dac.ch1().set_triggering(true);
    dac.ch2().set_triggering(true);
    dac.ch1().enable();
    dac.ch2().enable();
    dac.set(DualValue::Bit12Right(0x321, 0x654));
    dac.trigger();
    delay.delay_us(10);
    check(&mut failures, dac.ch1().read() == 0x321);
    check(&mut failures, dac.ch2().read() == 0x654);

    // A single-channel write must not disturb the other output.
    dac.ch1().set(Value::Bit12Left(0x456));
    dac.ch1().trigger();
    delay.delay_us(10);
    check(&mut failures, dac.ch1().read() == 0x456);
    check(&mut failures, dac.ch2().read() == 0x654);

    mailbox.payload.dor1.store(u32::from(dac.ch1().read()), Relaxed);
    mailbox.payload.dor2.store(u32::from(dac.ch2().read()), Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(2, Relaxed);

    loop {
        delay.delay_ms(1000);
    }
}
