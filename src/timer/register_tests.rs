extern crate std;

use std::sync::Mutex;

use super::*;
use crate::time::Hertz;
use crate::Peri;

static REGISTERS_LOCK: Mutex<()> = Mutex::new(());
static mut REGISTERS: [u32; 24] = [0; 24];

struct TestTimer;
embassy_hal_internal::impl_peripheral!(TestTimer);

impl crate::peripheral::SealedRccPeripheral for TestTimer {
    fn frequency() -> Hertz {
        Hertz(200_000_000)
    }
    fn enable_and_reset_with_cs(_: critical_section::CriticalSection) {
        unsafe { REGISTERS = [0; 24] };
    }
    fn disable_with_cs(_: critical_section::CriticalSection) {}
}
impl crate::peripheral::RccPeripheral for TestTimer {}
impl crate::peripheral::SealedRemapPeripheral for TestTimer {
    fn set_remap(_: u8) {}
}
impl crate::peripheral::RemapPeripheral for TestTimer {}
impl CoreInstance for TestTimer {
    type UpdateInterrupt = crate::interrupt::typelevel::TIM1_UP;
    const BITS: TimerBits = TimerBits::Bits16;
    fn regs() -> *mut () {
        core::ptr::addr_of_mut!(REGISTERS).cast()
    }
}
impl BasicInstance for TestTimer {}
impl SealedGeneralInstance for TestTimer {
    fn enable_outputs(&self) {
        let registers = unsafe { crate::pac::timer::Adtm::from_ptr(Self::regs()) };
        registers.bdtr().modify(|register| register.set_moe(true));
    }
    fn get_counting_mode(&self) -> low_level::CountingMode {
        let control = unsafe { crate::pac::timer::Adtm::from_ptr(Self::regs()) }
            .ctlr1()
            .read();
        (control.cms(), control.dir()).into()
    }
}
impl GeneralInstance16bit for TestTimer {
    type CaptureCompareInterrupt = crate::interrupt::typelevel::TIM1_UP;
    type TriggerInterrupt = crate::interrupt::typelevel::TIM1_UP;
}
impl AdvancedInstance for TestTimer {
    type CommunicationInterrupt = crate::interrupt::typelevel::TIM1_UP;
    type BreakInputInterrupt = crate::interrupt::typelevel::TIM1_UP;
}

#[test]
fn timer_frequency_roundtrips_and_update_source_is_preserved() {
    let _guard = REGISTERS_LOCK.lock().unwrap();
    let timer = low_level::Timer::new(unsafe { Peri::new_unchecked(TestTimer) });
    let registers = timer.regs_basic();
    for source in [
        crate::pac::timer::vals::Urs::ANYEVENT,
        crate::pac::timer::vals::Urs::COUNTERONLY,
    ] {
        registers.ctlr1().modify(|register| register.set_urs(source));
        for frequency in [1, 50, 1000, 200_000_000] {
            timer.set_frequency(Hertz(frequency));
            assert_eq!(timer.get_frequency().0, frequency);
            assert!(registers.ctlr1().read().urs() == source);
        }
    }
    registers.psc().write_value(u16::MAX);
    registers.atrlr().write_value(u16::MAX);
    assert_eq!(timer.get_frequency().0, 0);
}

#[test]
fn clearing_flags_writes_only_the_requested_zero_bits() {
    let _guard = REGISTERS_LOCK.lock().unwrap();
    let timer = low_level::Timer::new(unsafe { Peri::new_unchecked(TestTimer) });
    let registers = timer.regs_gp16();
    registers.intfr().write_value(crate::pac::timer::regs::Intfr(1));
    assert!(timer.clear_update_interrupt());
    assert_eq!(registers.intfr().read().0, 0xfffe);
    assert!(!timer.clear_update_interrupt());
    for channel in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
        registers.intfr().write_value(crate::pac::timer::regs::Intfr(0));
        timer.clear_input_interrupt(channel);
        assert_eq!(registers.intfr().read().0, 0xffff & !(1 << (channel.index() + 1)));
    }
}

#[test]
#[cfg(not(timer_x0))]
fn pwm_modes_and_full_duty_work_at_the_16bit_boundary() {
    let _guard = REGISTERS_LOCK.lock().unwrap();
    for mode in [
        low_level::CountingMode::EdgeAlignedUp,
        low_level::CountingMode::EdgeAlignedDown,
        low_level::CountingMode::CenterAlignedBothInterrupts,
    ] {
        let mut pwm = simple_pwm::SimplePwm::new(
            unsafe { Peri::new_unchecked(TestTimer) },
            None,
            None,
            None,
            None,
            Hertz(3125),
            mode,
        );
        let registers = unsafe { crate::pac::timer::Adtm::from_ptr(TestTimer::regs()) };
        assert!(registers.bdtr().read().moe());
        let reload = u32::from(registers.atrlr().read());
        let period = if mode.is_center_aligned() {
            2 * reload
        } else {
            reload + 1
        };
        assert_eq!(period * (u32::from(registers.psc().read()) + 1), 64_000);
        for channel in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            for duty in [0, pwm.get_max_duty() / 2, pwm.get_max_duty()] {
                pwm.set_duty(channel, duty);
                assert_eq!(pwm.get_duty(channel), duty);
            }
        }
        pwm.set_frequency(Hertz(1));
        pwm.set_duty(Channel::Ch1, pwm.get_max_duty());
    }
}

#[test]
#[cfg(not(timer_x0))]
fn complementary_pwm_disables_the_fourth_main_output() {
    let _guard = REGISTERS_LOCK.lock().unwrap();
    let mut pwm = complementary_pwm::ComplementaryPwm::new(
        unsafe { Peri::new_unchecked(TestTimer) },
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        Hertz(1000),
        low_level::CountingMode::EdgeAlignedUp,
    );
    let registers = unsafe { crate::pac::timer::Adtm::from_ptr(TestTimer::regs()) };
    pwm.enable(Channel::Ch4);
    assert!(registers.ccer().read().cce(3));
    pwm.disable(Channel::Ch4);
    assert!(!registers.ccer().read().cce(3));
    pwm.enable(Channel::Ch1);
    assert!(registers.ccer().read().cce(0) && registers.ccer().read().ccne(0));
    pwm.disable(Channel::Ch1);
    assert!(!registers.ccer().read().cce(0) && !registers.ccer().read().ccne(0));
    pwm.set_duty(Channel::Ch1, pwm.get_max_duty());
}
