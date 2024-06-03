//! GPIO
//!
//! Pin can be configured as:
//! - Floating Input
//! - PullUp Input
//! - PullDown Input
//! - Analog Input
//! - OpenDrain Output
//! - PushPull Output
//! - Alternate Function (input or output)
//!
//! Power On: Floating Input except for some Alternate Function

use core::convert::Infallible;

use critical_section::CriticalSection;
use pac::gpio::vals;

use crate::{exti, impl_peripheral, into_ref, pac, peripherals, Peripheral, PeripheralRef};

/// Speed, for output mode
#[derive(Debug, Eq, PartialEq, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// 10MHz
    Medium = 0b01,
    /// 2MHz
    Low = 0b10,
    /// 50MHz
    #[default]
    High = 0b11,
}

#[cfg(any(gpio_v3, gpio_v0))]
impl From<Speed> for vals::Mode {
    fn from(value: Speed) -> Self {
        use Speed::*;

        match value {
            Medium => vals::Mode::OUTPUT_10MHZ,
            Low => vals::Mode::OUTPUT_2MHZ,
            High => vals::Mode::OUTPUT_50MHZ,
        }
    }
}

#[cfg(gpio_x0)]
impl From<Speed> for vals::Mode {
    fn from(_value: Speed) -> Self {
        vals::Mode::OUTPUT_50MHZ
    }
}

/// Pull setting for an input.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    None,
    Up,
    Down,
}

impl From<Pull> for vals::Cnf {
    fn from(value: Pull) -> Self {
        match value {
            Pull::None => vals::Cnf::FLOATING_IN__OPEN_DRAIN_OUT,
            _ => vals::Cnf::PULL_IN__AF_PUSH_PULL_OUT,
        }
    }
}

impl Pull {
    #[inline]
    fn to_cnf(&self) -> u8 {
        match self {
            Pull::None => 0b01,
            _ => 0b10,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    Low,
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<u8> for Level {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::Low,
            _ => Self::High,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

/// GPIO flexible pin.
///
/// This pin can either be a disconnected, input, or output pin, or both. The level register bit will remain
/// set while not in output mode, so the pin's level will be 'remembered' when it is not in output
/// mode.
pub struct Flex<'d> {
    pub(crate) pin: PeripheralRef<'d, AnyPin>,
}

impl<'d> Flex<'d> {
    /// Wrap the pin in a `Flex`.
    ///
    /// The pin remains disconnected. The initial output level is unspecified, but can be changed
    /// before the pin is put into output mode.
    ///
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd) -> Self {
        into_ref!(pin);
        // Pin will be in disconnected state.
        Self { pin: pin.map_into() }
    }

    /// Put the pin into input mode.
    #[inline]
    pub fn set_as_input(&mut self, pull: Pull) {
        critical_section::with(|_| {
            self.pin.set_mode_cnf(vals::Mode::INPUT, pull.into());
            self.pin.set_pull(pull);
        });
    }

    /// Put the pin into output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn set_as_output(&mut self, speed: Speed) {
        critical_section::with(|_| {
            self.pin.set_as_output(speed);
        });
    }

    /// Put the pin into output mode with open drain.
    ///
    /// Pull resistor is disabled in this mode.
    #[inline]
    pub fn set_as_output_open_drain(&mut self, speed: Speed) {
        // 通用开漏输出模式
        #[cfg(not(gpio_x0))]
        critical_section::with(|_| {
            self.pin
                .set_mode_cnf(speed.into(), vals::Cnf::FLOATING_IN__OPEN_DRAIN_OUT);
        });
        #[cfg(gpio_x0)]
        critical_section::with(|_| {
            self.pin
                .set_mode_cnf(speed.into(), vals::Cnf::PULL_IN__AF_PUSH_PULL_OUT);
            // FIXME: only i2c is auto set to open drain
        });
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.block().indr().read().idr(self.pin.pin() as usize)
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.block().outdr().read().odr(self.pin.pin() as usize)
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.pin.set_low(),
            Level::High => self.pin.set_high(),
        }
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}

/// GPIO input driver.
pub struct Input<'d> {
    pub(crate) pin: Flex<'d>,
}

impl<'d> Input<'d> {
    /// Create GPIO input driver for a [Pin] with the provided [Pull] configuration.
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_input(pull);
        Self { pin }
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }
}

/// GPIO output driver.
///
/// Note that pins will **return to their floating state** when `Output` is dropped.
/// If pins should retain their state indefinitely, either keep ownership of the
/// `Output`, or pass it to [`core::mem::forget`].
pub struct Output<'d> {
    pub(crate) pin: Flex<'d>,
}

impl<'d> Output<'d> {
    /// Create GPIO output driver for a [Pin] with the provided [Level] and [Speed] configuration.
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, initial_output: Level, speed: Speed) -> Self {
        let mut pin = Flex::new(pin);
        match initial_output {
            Level::High => pin.set_high(),
            Level::Low => pin.set_low(),
        }
        pin.set_as_output(speed);
        Self { pin }
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }
}

/// GPIO output open-drain driver.
///
/// Note that pins will **return to their floating state** when `OutputOpenDrain` is dropped.
/// If pins should retain their state indefinitely, either keep ownership of the
/// `OutputOpenDrain`, or pass it to [`core::mem::forget`].
pub struct OutputOpenDrain<'d> {
    pub(crate) pin: Flex<'d>,
}

impl<'d> OutputOpenDrain<'d> {
    /// Create a new GPIO open drain output driver for a [Pin] with the provided [Level] and [Speed], [Pull] configuration.
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, initial_output: Level, speed: Speed) -> Self {
        let mut pin = Flex::new(pin);

        match initial_output {
            Level::High => pin.set_high(),
            Level::Low => pin.set_low(),
        }

        pin.set_as_output_open_drain(speed);
        Self { pin }
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        !self.pin.is_low()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level);
    }

    /// Get whether the output level is set to high.
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Get whether the output level is set to low.
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// Get the current output level.
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }
}

/// GPIO output type
pub enum OutputType {
    /// Drive the pin both high or low.
    PushPull,
    /// Drive the pin low, or don't drive it at all if the output level is high.
    #[cfg(not(gpio_x0))]
    OpenDrain,
}

impl From<OutputType> for AFType {
    fn from(value: OutputType) -> Self {
        match value {
            #[cfg(not(gpio_x0))]
            OutputType::OpenDrain => AFType::OutputOpenDrain,
            OutputType::PushPull => AFType::OutputPushPull,
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AFType {
    /// Output, drive the pin both high or low.
    OutputPushPull = 0b10,
    /// Output, drive the pin low, or don't drive it at all if the output level is high.
    #[cfg(not(gpio_x0))]
    OutputOpenDrain = 0b11,
}

impl From<AFType> for vals::Cnf {
    fn from(value: AFType) -> Self {
        match value {
            AFType::OutputPushPull => vals::Cnf::PULL_IN__AF_PUSH_PULL_OUT,
            #[cfg(not(gpio_x0))]
            AFType::OutputOpenDrain => vals::Cnf::AF_OPEN_DRAIN_OUT,
        }
    }
}

/// Alternate function type settings, CNF, when MODE>0b00

pub(crate) trait SealedPin {
    fn pin_port(&self) -> u8;

    #[inline]
    fn _pin(&self) -> u8 {
        self.pin_port() % 32
    }
    #[inline]
    fn _port(&self) -> u8 {
        self.pin_port() / 32
    }

    #[inline]
    fn block(&self) -> pac::gpio::Gpio {
        pac::GPIO(self._port() as _)
    }

    /// Set the output as high.
    #[inline]
    fn set_high(&self) {
        let n = self._pin() as _;
        #[cfg(not(gpio_x0))]
        self.block().bshr().write(|w| w.set_bs(n, true));
        #[cfg(gpio_x0)]
        {
            if n >= 16 {
                self.block().bsxr().write(|w| w.set_bs(n - 16, true)); // extended BS
            } else {
                self.block().bshr().write(|w| w.set_bs(n, true));
            }
        }
    }

    /// Set the output as low.
    #[inline]
    fn set_low(&self) {
        let n = self._pin() as _;
        #[cfg(not(gpio_x0))]
        self.block().bshr().write(|w| w.set_br(n, true));
        #[cfg(gpio_x0)]
        {
            if n >= 16 {
                self.block().bsxr().write(|w| w.set_br(n - 16, true)); // extended BS
            } else {
                self.block().bshr().write(|w| w.set_br(n, true));
            }
        }
    }

    #[inline]
    fn set_mode_cnf(&self, mode: vals::Mode, cnf: vals::Cnf) {
        let pin = self._pin() as usize;
        let block = self.block();

        match pin / 8 {
            0 => {
                block.cfglr().modify(|w| {
                    w.set_cnf(pin % 8, cnf);
                    w.set_mode(pin % 8, mode);
                });
            }
            #[cfg(not(gpio_v0))]
            1 => {
                block.cfghr().modify(|w| {
                    w.set_cnf(pin % 8, cnf);
                    w.set_mode(pin % 8, mode);
                });
            }
            // only for GPIO with 24 lines
            #[cfg(gpio_x0)]
            2 => {
                block.cfgxr().modify(|w| {
                    w.set_cnf(pin % 8, cnf);
                    w.set_mode(pin % 8, mode);
                });
            }
            _ => unreachable!(),
        }
    }

    #[inline]
    fn set_as_output(&self, speed: Speed) {
        self.set_mode_cnf(speed.into(), vals::Cnf::ANALOG_IN__PUSH_PULL_OUT);
    }

    #[inline]
    fn set_as_input(&self, pull: Pull) {
        self.set_mode_cnf(vals::Mode::INPUT, pull.into());
        self.set_pull(pull);
    }

    #[inline]
    fn set_pull(&self, pull: Pull) {
        let pin = self._pin() as _;

        match pull {
            Pull::Up => self.block().outdr().modify(|w| w.set_odr(pin, true)),
            Pull::Down => self.block().outdr().modify(|w| w.set_odr(pin, false)),
            _ => {}
        }
    }

    /// Only one type, alternate function + push pull
    #[inline]
    fn set_as_af_output(&self, af_type: AFType, speed: Speed) {
        self.set_mode_cnf(speed.into(), af_type.into());
    }

    /// Analog mode, both input and output
    #[inline]
    fn set_as_analog(&self) {
        self.set_mode_cnf(vals::Mode::INPUT, vals::Cnf::ANALOG_IN__PUSH_PULL_OUT);
    }

    /// Set the pin as "disconnected", ie doing nothing and consuming the lowest
    /// amount of power possible.
    ///
    /// This is currently the same as set_as_analog but is semantically different really.
    /// Drivers should set_as_disconnected pins when dropped.
    #[inline]
    fn set_as_disconnected(&self) {
        self.set_as_analog();
    }
}

#[allow(private_bounds)]
pub trait Pin: Peripheral<P = Self> + Into<AnyPin> + SealedPin + Sized + 'static {
    type ExtiChannel: crate::exti::Channel;

    /// Number of the pin within the port (0..31)
    #[inline]
    fn pin(&self) -> u8 {
        self._pin()
    }

    /// Port of the pin
    #[inline]
    fn port(&self) -> u8 {
        self._port()
    }

    /// Convert from concrete pin type PX_XX to type erased `AnyPin`.
    #[inline]
    fn degrade(self) -> AnyPin {
        AnyPin {
            pin_port: self.pin_port(),
        }
    }
}

// Type-erased GPIO pin
pub struct AnyPin {
    pin_port: u8,
}

impl AnyPin {
    #[inline]
    pub unsafe fn steal(pin_port: u8) -> Self {
        Self { pin_port }
    }

    #[inline]
    fn _port(&self) -> u8 {
        self.pin_port / 32
    }
}

impl_peripheral!(AnyPin);
impl Pin for AnyPin {
    #[cfg(exti)]
    type ExtiChannel = exti::AnyChannel;
}
impl SealedPin for AnyPin {
    #[inline]
    fn pin_port(&self) -> u8 {
        self.pin_port
    }
}

foreach_pin!(
    ($pin_name:ident, $port_name:ident, $port_num:expr, $pin_num:expr, $exti_ch:ident) => {
        impl Pin for peripherals::$pin_name {
            #[cfg(exti)]
            type ExtiChannel = peripherals::$exti_ch;
        }

        impl SealedPin for peripherals::$pin_name {
            #[inline]
            fn pin_port(&self) -> u8 {
                $port_num * 32 + $pin_num
            }
        }

        impl From<peripherals::$pin_name> for AnyPin {
            fn from(x: peripherals::$pin_name) -> Self {
                x.degrade()
            }
        }
    };
);

/// Enable the GPIO peripheral clock.

pub(crate) unsafe fn init(_cs: CriticalSection) {
    #[cfg(afio)]
    <crate::peripherals::AFIO as crate::peripheral::SealedRccPeripheral>::enable_and_reset_with_cs(_cs);

    crate::_generated::init_gpio();
}

// eh impls

impl<'d> embedded_hal::digital::ErrorType for Input<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::InputPin for Input<'d> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal::digital::ErrorType for Output<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::OutputPin for Output<'d> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }
}

impl<'d> embedded_hal::digital::StatefulOutputPin for Output<'d> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    /// Is the output pin set as low?
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d> embedded_hal::digital::ErrorType for OutputOpenDrain<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::InputPin for OutputOpenDrain<'d> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal::digital::OutputPin for OutputOpenDrain<'d> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }
}

impl<'d> embedded_hal::digital::StatefulOutputPin for OutputOpenDrain<'d> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    /// Is the output pin set as low?
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d> embedded_hal::digital::InputPin for Flex<'d> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal::digital::OutputPin for Flex<'d> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }
}

impl<'d> embedded_hal::digital::ErrorType for Flex<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::StatefulOutputPin for Flex<'d> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    /// Is the output pin set as low?
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}
