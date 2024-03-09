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

use self::sealed::Pin as _;
use crate::{exti, impl_peripheral, into_ref, pac, peripherals, Peripheral, PeripheralRef};

/// Speed, for output mode
#[derive(Debug, Eq, PartialEq, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// 10MHz
    #[default]
    Medium = 0b01,
    /// 2MHz
    Low = 0b10,
    /// 50MHz
    High = 0b11,
}

/// Pull setting for an input.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    None,
    Up,
    Down,
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
        critical_section::with(|_| self.pin.set_cnf_mode(pull.to_cnf(), 0b00));
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
        critical_section::with(|_| {
            self.pin.set_cnf_mode(0b01, speed as u8);
        });
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.block().indr().read().idr(1 << self.pin.pin())
    }

    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.block().outdr().read().odr(1 << self.pin.pin())
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
    OpenDrain,
}

impl From<OutputType> for sealed::AFType {
    fn from(value: OutputType) -> Self {
        match value {
            OutputType::OpenDrain => sealed::AFType::OutputOpenDrain,
            OutputType::PushPull => sealed::AFType::OutputPushPull,
        }
    }
}

pub(crate) mod sealed {
    use super::*;

    /// Alternate function type settings, CNF, when MODE>0b00
    #[derive(Debug, Copy, Clone)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum AFType {
        /// Output, drive the pin both high or low.
        OutputPushPull = 0b10,
        /// Output, drive the pin low, or don't drive it at all if the output level is high.
        OutputOpenDrain = 0b11,
    }

    pub trait Pin {
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
            self.block().bshr().write(|w| w.set_bs(n, true));
        }

        /// Set the output as low.
        #[inline]
        fn set_low(&self) {
            let n = self._pin() as _;
            self.block().bshr().write(|w| w.set_br(n, true));
        }

        #[inline]
        fn set_cnf_mode(&self, cnf: u8, mode: u8) {
            let pin = self._pin() as usize;
            let block = self.block();

            let cnf = cnf & 0b11;
            let mode = mode & 0b11;

            match pin / 8 {
                0 => {
                    block.cfglr().modify(|w| {
                        w.set_cnf(pin % 8, cnf);
                        w.set_mode(pin % 8, mode);
                    });
                }
                1 => {
                    block.cfghr().modify(|w| {
                        w.set_cnf(pin % 8, cnf);
                        w.set_mode(pin % 8, mode);
                    });
                }
                // only for GPIO with 24 lines
                #[cfg(feature = "gpio_x0")]
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
            let cnf = 0b00;
            let mode = speed as u8;

            self.set_cnf_mode(cnf, mode);
        }

        #[inline]
        fn set_as_input(&self, pull: Pull) {
            let cnf = if pull == Pull::None { 0b01 } else { 0b10 };
            let mode = 0b00;

            self.set_cnf_mode(cnf, mode);
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
            let cnf = af_type as u8;
            let mode = speed as u8;

            self.set_cnf_mode(cnf, mode);
        }

        /// Analog mode, both input and output
        #[inline]
        fn set_as_analog(&self) {
            let cnf = 0b00;
            let mode = 0b00;

            self.set_cnf_mode(cnf, mode);
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
}

pub trait Pin: Peripheral<P = Self> + Into<AnyPin> + sealed::Pin + Sized + 'static {
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
        self.pin_port / 16
    }
}

impl_peripheral!(AnyPin);
impl Pin for AnyPin {
    type ExtiChannel = exti::AnyChannel;
}
impl sealed::Pin for AnyPin {
    #[inline]
    fn pin_port(&self) -> u8 {
        self.pin_port
    }
}

foreach_pin!(
    ($pin_name:ident, $port_name:ident, $port_num:expr, $pin_num:expr, $exti_ch:ident) => {
        impl Pin for peripherals::$pin_name {
            // #[cfg(feature = "exti")]
            type ExtiChannel = peripherals::$exti_ch;
        }

        impl sealed::Pin for peripherals::$pin_name {
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
