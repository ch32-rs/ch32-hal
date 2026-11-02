#![no_std]
#![no_main]

use core::future::Future;
use core::pin::{pin, Pin};
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use ch32_hal::exti::ExtiInput;
use ch32_hal::gpio::{Flex, Input, Level, Output, Pull, Speed};
use ch32_hal::{pac, Peri};
use defmt_rtt as _;
use embassy_futures::join::join3;
use panic_halt as _;
use qingke_rt::entry;

#[no_mangle]
pub static mut GPIO_EXTI_RESULT: u32 = 0;
static WAKE_COUNT: AtomicU32 = AtomicU32::new(0);
const LINES: u32 = (1 << 1) | (1 << 8) | (1 << 13);
const VTABLE: RawWakerVTable = RawWakerVTable::new(clone_waker, wake, wake, drop_waker);

unsafe fn clone_waker(_: *const ()) -> RawWaker {
    RawWaker::new(core::ptr::null(), &VTABLE)
}

unsafe fn wake(_: *const ()) {
    WAKE_COUNT.store(1, Ordering::Relaxed);
}

unsafe fn drop_waker(_: *const ()) {}

#[derive(Clone, Copy)]
enum Edge {
    Rising,
    Falling,
    Any,
}

async fn wait(input: &mut ExtiInput<'_>, edge: Edge) {
    match edge {
        Edge::Rising => input.wait_for_rising_edge().await,
        Edge::Falling => input.wait_for_falling_edge().await,
        Edge::Any => input.wait_for_any_edge().await,
    }
}

fn check(condition: bool) {
    if !condition {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(GPIO_EXTI_RESULT), 3) };
        defmt::panic!("GPIO/EXTI check failed");
    }
}

fn check_pull(pin: Peri<'_, impl ch32_hal::gpio::Pin>) {
    let mut pin = pin;
    for pull in [Pull::Up, Pull::Down] {
        let input = Input::new(pin.reborrow(), pull);
        qingke::riscv::asm::delay(20_000);
        check(input.is_high() == (pull == Pull::Up));
    }
    let _input = Input::new(pin, Pull::None);
}

fn arm(future: Pin<&mut impl Future>, context: &mut Context<'_>) {
    check(future.poll(context).is_pending());
    check(pac::EXTI.intenr().read().0 & LINES == LINES);
    check(pac::EXTI.intfr().read().0 & LINES == 0);
    check(pac::AFIO.exticr(0).read().exti(1) == 1);
    check(pac::AFIO.exticr(2).read().exti(0) == 1);
    check(pac::AFIO.exticr(3).read().exti(1) == 2);
}

fn complete(mut future: Pin<&mut impl Future>, context: &mut Context<'_>, output: &mut Output<'_>, level: Level) {
    WAKE_COUNT.store(0, Ordering::Relaxed);
    output.set_level(level);
    for _ in 0..500_000 {
        if let Poll::Ready(_) = future.as_mut().poll(context) {
            check(WAKE_COUNT.load(Ordering::Relaxed) != 0);
            check(pac::EXTI.intenr().read().0 & LINES == 0);
            return;
        }
    }
    defmt::error!("EXTI timeout: check PA0 -> PB1/PB8/PC13 jumpers");
    check(false);
}

#[entry]
fn main() -> ! {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(GPIO_EXTI_RESULT), 1) };
    qingke::riscv::asm::delay(2_000_000);
    let mut peripherals = ch32_hal::init(Default::default());
    defmt::info!("GPIO pulls: PA0 -> PB1, PB8, PC13; PA0 initially floating");
    let mut output = Flex::new(peripherals.PA0.reborrow());
    output.set_as_input(Pull::None);
    {
        let _input1 = Input::new(peripherals.PB1.reborrow(), Pull::None);
        let _input8 = Input::new(peripherals.PB8.reborrow(), Pull::None);
        let _input13 = Input::new(peripherals.PC13.reborrow(), Pull::None);
    }
    check_pull(peripherals.PB1.reborrow());
    check_pull(peripherals.PB8.reborrow());
    check_pull(peripherals.PC13.reborrow());
    drop(output);
    let mut output = Output::new(peripherals.PA0, Level::Low, Speed::Low);
    let mut input1 = ExtiInput::new(peripherals.PB1, peripherals.EXTI1, Pull::None);
    let mut input8 = ExtiInput::new(peripherals.PB8, peripherals.EXTI8, Pull::None);
    let mut input13 = ExtiInput::new(peripherals.PC13, peripherals.EXTI13, Pull::None);
    let waker = unsafe { Waker::from_raw(clone_waker(core::ptr::null())) };
    let mut context = Context::from_waker(&waker);

    for round in 0..8 {
        for (edge, level) in [
            (Edge::Rising, Level::High),
            (Edge::Falling, Level::Low),
            (Edge::Any, Level::High),
            (Edge::Any, Level::Low),
        ] {
            let mut future = pin!(join3(
                wait(&mut input1, edge),
                wait(&mut input8, edge),
                wait(&mut input13, edge)
            ));
            arm(future.as_mut(), &mut context);
            complete(future.as_mut(), &mut context, &mut output, level);
        }
        {
            let mut future = pin!(join3(
                wait(&mut input1, Edge::Any),
                wait(&mut input8, Edge::Any),
                wait(&mut input13, Edge::Any)
            ));
            arm(future.as_mut(), &mut context);
        }
        check(pac::EXTI.intenr().read().0 & LINES == 0);
        output.set_high();
        qingke::riscv::asm::delay(20_000);
        check(pac::EXTI.intfr().read().0 & LINES == LINES);
        {
            let mut future = pin!(join3(
                wait(&mut input1, Edge::Falling),
                wait(&mut input8, Edge::Falling),
                wait(&mut input13, Edge::Falling)
            ));
            arm(future.as_mut(), &mut context);
            complete(future.as_mut(), &mut context, &mut output, Level::Low);
        }
        check(input1.is_low() && input8.is_low() && input13.is_low());
        defmt::info!("EXTI rising/falling/any/cancel/stale round={} PASS", round);
    }
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(GPIO_EXTI_RESULT), 2) };
    defmt::info!("GPIO_EXTI_RESULT=PASS");
    loop {
        core::hint::spin_loop();
    }
}
