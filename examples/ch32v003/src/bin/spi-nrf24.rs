#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
use ch32_hal::gpio::Pin;
use hal::dma::NoDma;
use hal::gpio::{Level, Output};
use hal::peripherals::SPI1;
use hal::prelude::*;
use hal::println;
use hal::spi::Spi;
use qingke::riscv;
use {ch32_hal as hal, panic_halt as _};

/// The following example is an altered verison of https://github.com/rtxm/rust-nrf24l01/tree/master.
/// the code has been altered to avoid `std` and rely on `ch320-hal`'s `Output` and `SPI` impls.

pub struct CEPin {
    gpio: Output<'static>,
    value: Level,
    saved_value: Level,
}

impl CEPin {
    pub fn new(gpio: Output<'static>) -> Result<CEPin, NrfError> {
        let state = gpio.get_output_level();
        Ok(CEPin {
            gpio: gpio,
            value: state,
            saved_value: state,
        })
    }

    pub fn up(&mut self) -> Result<(), NrfError> {
        self.gpio.set_high();
        self.value = Level::High;
        Ok(())
    }

    pub fn down(&mut self) -> Result<(), NrfError> {
        self.gpio.set_low();
        self.value = Level::Low;
        Ok(())
    }

    pub fn save_state(&mut self) -> () {
        self.saved_value = self.value;
    }

    pub fn restore_state(&mut self) -> Result<(), NrfError> {
        self.gpio.set_level(self.saved_value);
        self.value = self.saved_value;
        Ok(())
    }
}

/// Supported air data rates.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum DataRate {
    R250Kbps,
    R1Mbps,
    R2Mbps,
}

impl Default for DataRate {
    fn default() -> DataRate {
        DataRate::R1Mbps
    }
}

/// Supported power amplifier levels.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum PALevel {
    /// -18 dBm, 7.0 mA DC current consumption, few meters range.
    Min,
    /// -12 dBM, 7.5 mA DC current consumption.
    Low,
    /// -6 dBm, 9.0 mA DC current consumption.
    High,
    /// 0 dBm, 11.3 mA DC current consumption, up to 100 meters range.
    Max,
}

impl Default for PALevel {
    fn default() -> PALevel {
        PALevel::Min
    }
}

/// Receiver mode configuration
#[derive(Debug, Default)]
pub struct RXConfig {
    /// data rate, defaults to `DataRate::R1Mbps`.
    pub data_rate: DataRate,
    /// channel, in the range [0, 125], defaults to 0.
    ///
    /// The RF channel frequency F is set according to the formula:
    ///
    /// F = 2400 + `channel` Mhz
    ///
    /// Any `channel` value above 125 is capped to 125.
    pub channel: u8,
    /// Powel level, defaults to `PALevel::Min`.
    pub pa_level: PALevel,
    /// Pipe 0 address
    ///
    /// This is the receiving base address.
    ///
    /// Typically, this is the only address you need to set, unless you
    /// need a multiceiver configuration. In that case, you can enable up to
    /// five additional receiving pipes.
    ///
    /// The address is in little endian order: the first byte is the least significant one.
    ///
    /// You must provide a valid address for Pipe 0.
    pub pipe0_address: [u8; 5],
    /// Pipe 1 address, defaults to None (disabled)
    ///
    /// All pipes 2-5 share the 4 most significant bytes with the pipe 1 address, so
    /// you only need to provide the least significant byte to enable one of those pipes or
    /// set it to None to disable it (default).
    pub pipe1_address: Option<[u8; 5]>,
    /// Pipe 2 LSB, defaults to None (disabled)
    pub pipe2_addr_lsb: Option<u8>,
    /// Pipe 3 LSB, defaults to None (disabled)
    pub pipe3_addr_lsb: Option<u8>,
    /// Pipe 4 LSB, defaults to None (disabled)
    pub pipe4_addr_lsb: Option<u8>,
    /// Pipe 5 LSB, defaults to None (disabled)
    pub pipe5_addr_lsb: Option<u8>,
}

/// Transmitter mode configuration
#[derive(Debug, Default)]
pub struct TXConfig {
    /// data rate, defaults to `DataRate::R1Mbps`
    ///
    /// Both Transmitter and Receiver ends should use the same data rate.
    pub data_rate: DataRate,
    /// channel, in the range [0, 125], defaults to 0.
    ///
    /// The RF channel frequency F is set according to the formula:
    ///
    /// F = 2400 + `channel` Mhz
    ///
    /// Any `channel` value above 125 is capped to 125.
    ///
    /// Both Transmitter and Receiver ends should use the same channel.
    pub channel: u8,
    /// Powel level, defaults to `PALevel::Min`.
    pub pa_level: PALevel,
    /// Max number of retries before giving up when trying to send a packet.
    ///
    /// 0 <= `max_retries` <= 15. Default is 0. Any value above 15 is capped to 15.
    pub max_retries: u8, // [0, 15]
    /// Delay (in steps of 250µs) between retries.
    ///
    /// Actual delay = 250 + `retry_delay` * 250 [µs]
    ///
    /// 0 <= `retry_delay` <= 15. Default is 0, recommended is > 1.
    /// Any value above 15 is capped to 15.
    pub retry_delay: u8, // [0, 15]
    /// Destination address, should match an address on the receiver end.
    ///
    /// This is also the address on which ACK packets are received.
    /// The address is in little endian order: the first byte is the least significant one.
    pub pipe0_address: [u8; 5],
}

/// The Operating mode, either Receiver or Transmitter.
#[derive(Debug)]
pub enum OperatingMode {
    /// Primary receiver
    RX(RXConfig),
    /// Primary transmitter
    TX(TXConfig),
}

type Command = u8;

// Read register
const R_REGISTER: Command = 0;
// Write register
const W_REGISTER: Command = 0b0010_0000;
// Read input FIFO
const R_RX_PAYLOAD: Command = 0b0110_0001;
// Read the size of the packet on top of input FIFO
const R_RX_PL_WID: Command = 0b0110_0000;
// Push a packet to output FIFO
const W_TX_PAYLOAD: Command = 0b1010_0000;
// Push an ACK packet to output FIFO
// Use the last three bits to specify the pipe number
const W_ACK_PAYLOAD: Command = 0b1010_1000;
// Flush commands
const FLUSH_TX: Command = 0b1110_0001;
const FLUSH_RX: Command = 0b1110_0010;

type Register = u8;

// Base config, p 54
const CONFIG: Register = 0;
// Enable auto acknowlegment, p54
const EN_AA: Register = 0x01;
// Enabled RX addresses, p 54
const EN_RXADDR: Register = 0x02;
// Setup of automatic retransmission, p 55
const SETUP_RETR: Register = 0x04;
// Channel, p 55
const RF_CH: Register = 0x05;
// RF data rate and power, p 55
const RF_SETUP: Register = 0x06;
// The status register is returned for each command, so we don't need
// to reed it explicitly.
// We may need to write to it to clear some flags (RX_DR, TX_DS, MAX_RT)
// p 56
const STATUS: Register = 0x07;
// Transmission quality, p 56
const OBSERVE_TX: Register = 0x08;
// Received Power Detector
const RPD: Register = 0x09;
// Pipe 0 address, p 57
const RX_ADDR_P0: Register = 0x0A;
// Pipe 1 address, p 57
const RX_ADDR_P1: Register = 0x0B;
// Pipe 2 address, p 57
const RX_ADDR_P2: Register = 0x0C;
// Pipe 3 address, p 57
const RX_ADDR_P3: Register = 0x0D;
// Pipe 4 address, p 57
const RX_ADDR_P4: Register = 0x0E;
// Pipe 5 address, p 57
const RX_ADDR_P5: Register = 0x0F;
// Destination address, p 57
const TX_ADDR: Register = 0x10;
// FIFO status (RX & TX), p 58
const FIFO_STATUS: Register = 0x17;
// Enable dynamic payload length (requires EN_DPL and ENAA_PX), p 59
const DYNPD: Register = 0x1C;
//  Feature register (content EN_DPL, EN_ACK_PAY...), p 59
const FEATURE: Register = 0x1D;

// Custom Error
#[derive(Debug)]
pub enum NrfError {
    TransferError,
    Timeout,
    CeError,
    PinError,
    WriteZeroQueueFull,
    InvalidDataPacketTooBig,
    MaxRetries,
}

/// The driver
pub struct NRF24L01 {
    ce: CEPin,
    spi: Spi<'static, SPI1, NoDma, NoDma>,
    base_config: u8,
}

impl NRF24L01 {
    // Private methods and functions

    fn send_command(&mut self, data_out: &[u8], data_in: &mut [u8]) -> Result<(), NrfError> {
        self.spi.blocking_transfer(data_in, data_out).map_err(|e| {
            println!("error {:?}", e); // emit a debug error
            NrfError::TransferError
        })
    }

    fn write_register(&mut self, register: Register, byte: u8) -> Result<(), NrfError> {
        // For single byte registers only
        let mut response_buffer = [0u8; 2];
        self.send_command(&[W_REGISTER | register, byte], &mut response_buffer)
    }

    fn read_register(&mut self, register: Register) -> Result<(u8, u8), NrfError> {
        // For single byte registers only.
        // Return (STATUS, register)
        let mut response_buffer = [0u8; 2];
        self.send_command(&[R_REGISTER | register, 0], &mut response_buffer)?;
        Ok((response_buffer[0], response_buffer[1]))
    }

    fn setup_rf(&mut self, rate: DataRate, level: PALevel) -> Result<(), NrfError> {
        let rate_bits: u8 = match rate {
            DataRate::R250Kbps => 0b0010_0000,
            DataRate::R1Mbps => 0,
            DataRate::R2Mbps => 0b0000_1000,
        };
        let level_bits: u8 = match level {
            PALevel::Min => 0,
            PALevel::Low => 0b0000_0010,
            PALevel::High => 0b0000_0100,
            PALevel::Max => 0b0000_0110,
        };
        self.write_register(RF_SETUP, rate_bits | level_bits)
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), NrfError> {
        if channel < 126 {
            self.write_register(RF_CH, channel)
        } else {
            self.write_register(RF_CH, 125)
        }
    }

    fn set_full_address(&mut self, pipe: Register, address: [u8; 5]) -> Result<(), NrfError> {
        let mut response_buffer = [0u8; 6];
        let mut command = [W_REGISTER | pipe, 0, 0, 0, 0, 0];
        command[1..].copy_from_slice(&address);
        self.send_command(&command, &mut response_buffer)
    }

    fn configure_receiver(&mut self, config: &RXConfig) -> Result<u8, NrfError> {
        // set data rate
        // set PA level
        self.setup_rf(config.data_rate, config.pa_level)?;
        // set channel
        self.set_channel(config.channel)?;
        // set Pipe 0 address
        self.set_full_address(RX_ADDR_P0, config.pipe0_address)?;
        let mut enabled = 1u8;
        // Pipe 1
        if let Some(address) = config.pipe1_address {
            self.set_full_address(RX_ADDR_P1, address)?;
            enabled |= 0b0000_0010
        };
        // Pipe 2
        if let Some(lsb) = config.pipe2_addr_lsb {
            self.write_register(RX_ADDR_P2, lsb)?;
            enabled |= 0b0000_0100
        };
        // Pipe 3
        if let Some(lsb) = config.pipe3_addr_lsb {
            self.write_register(RX_ADDR_P3, lsb)?;
            enabled |= 0b0000_1000
        }
        // Pipe 4
        if let Some(lsb) = config.pipe4_addr_lsb {
            self.write_register(RX_ADDR_P4, lsb)?;
            enabled |= 0b0001_0000
        };
        // Pipe 5
        if let Some(lsb) = config.pipe5_addr_lsb {
            self.write_register(RX_ADDR_P5, lsb)?;
            enabled |= 0b0010_0000
        };
        // Enable configured pipes
        self.write_register(EN_RXADDR, enabled)?;
        // base config is 2 bytes for CRC and RX mode on
        // only reflect RX_DR on the IRQ pin
        Ok(0b0011_1101)
    }

    fn configure_transmitter(&mut self, config: &TXConfig) -> Result<u8, NrfError> {
        // set data rate
        // set PA level
        self.setup_rf(config.data_rate, config.pa_level)?;
        // set channel
        self.set_channel(config.channel)?;
        // set destination and Pipe 0 address
        self.set_full_address(RX_ADDR_P0, config.pipe0_address)?;
        self.set_full_address(TX_ADDR, config.pipe0_address)?;
        // disable other pipes
        self.write_register(EN_RXADDR, 1u8)?;
        // retransmission settings
        let retry_bits: u8 = if config.max_retries < 16 {
            config.max_retries
        } else {
            15
        };
        let retry_delay_bits: u8 = if config.retry_delay < 16 {
            config.retry_delay << 4
        } else {
            0xF0
        };
        self.write_register(SETUP_RETR, retry_delay_bits | retry_bits)?;
        // base config is 2 bytes for CRC and TX mode on
        // only reflect TX_DS and MAX_RT on the IRQ pin
        Ok(0b0100_1100)
    }

    // Public API

    /// Construct a new driver instance.
    ///
    /// * `ce_pin`: the GPIO number (Linux SysFS) connected to the CE pin of the transceiver
    /// * `spi_device`: the SPI device number (or channel) the transceiver is connected to.
    ///
    /// We use the spidev linux kernel driver. Ensure you have enabled SPI on your system.
    ///
    /// # Errors
    ///
    /// System IO errors
    ///
    pub fn new(spi_device: Spi<'static, SPI1, NoDma, NoDma>, ce_pin: Output<'static>) -> Result<NRF24L01, NrfError> {
        // pub fn new(ce_pin: u64, spi_device: u8) -> io::Result<NRF24L01> {
        // let mut spi = spidev::Spidev::open(format!("/dev/spidev0.{}", spi_device))?;
        // let options = spidev::SpidevOptions::new()
        //     .bits_per_word(8)
        //     .max_speed_hz(10_000_000)
        //     .mode(spidev::SpiModeFlags::SPI_MODE_0)
        //     .build();
        // spi.configure(&options)?;
        let ce = CEPin::new(ce_pin).map_err(|_| NrfError::CeError)?;
        Ok(NRF24L01 {
            ce,
            spi: spi_device,
            base_config: 0b0000_1101,
        })
    }

    /// Configure the device as Primary Receiver (PRX) or Primary Transmitter (PTX),
    /// set all its properties for proper operation and power it up.
    ///
    /// The device remain in standby until `self.listen()` (RX mode)
    /// or `self.send()` (TX mode) is called.
    ///
    /// All commands work when the device is in standby (recommended) as well as
    /// active state.
    pub fn configure(&mut self, mode: &OperatingMode) -> Result<(), NrfError> {
        self.ce.down().map_err(|_| NrfError::PinError)?;
        // auto acknowlegement
        self.write_register(EN_AA, 0b0011_1111)?;
        // dynamic payload and payload with ACK
        self.write_register(DYNPD, 0b0011_1111)?;
        self.write_register(FEATURE, 0b0000_0110)?;

        // Mode specific configuration
        match *mode {
            OperatingMode::RX(ref config) => self.configure_receiver(config),
            OperatingMode::TX(ref config) => self.configure_transmitter(config),
        }
        .and_then(|base_config| {
            // Go!
            self.base_config = base_config;
            self.power_up()
        })
    }

    /// Scan all channel.
    ///
    /// This function scans all channels ``nb_iter`` times, waiting ``wait_ms``
    /// milliseconds for a carrier on each. The number of time a carrier has been
    /// detected is placed in the ``channel_table`` array at the corresponding index.
    ///
    /// You don't need to call ``.configure`` before using this function, but
    /// you must do it after if you plan to use the device as a receiver or
    /// transmitter.
    pub fn scan(&mut self, nb_iter: u32, wait_ms: u32, channel_table: &mut [u32; 126]) -> Result<(), NrfError> {
        self.write_register(EN_AA, 0u8)?;
        for _ in 0..nb_iter {
            for channel in 0..126 {
                self.set_channel(channel)?;
                self.listen()?;
                // TODO: improve wait time
                // sleep(Duration::from_millis(wait_ms as u64));
                riscv::asm::delay(wait_ms * 1_000);
                self.standby()?;
                let (_, rpd) = self.read_register(RPD)?;
                if rpd > 0 {
                    channel_table[channel as usize] += 1;
                }
            }
        }
        Ok(())
    }

    pub fn is_receiver(&self) -> bool {
        self.base_config & 1u8 == 1u8
    }

    /// Power down the device.
    ///
    /// The power consumption is minimum in this mode, and the device ceases all operation.
    /// It only accepts configuration commands.
    pub fn power_down(&mut self) -> Result<(), NrfError> {
        self.ce.down()?;
        self.write_register(CONFIG, self.base_config)
    }

    /// Power the device up for full operation.
    pub fn power_up(&mut self) -> Result<(), NrfError> {
        println!("Power up NRF24");
        self.write_register(CONFIG, self.base_config | 0b0000_0010)
    }

    /// Put the device in standby (RX Mode)
    ///
    /// Only used in RX mode to suspend active listening.
    /// In TX mode, standby is the default state when not sending data.
    pub fn standby(&mut self) -> Result<(), NrfError> {
        self.ce.down()?;
        Ok(())
    }

    /// (RX mode only) Wake up and activate receiver.
    ///
    /// In RX mode, call this function after a `.configure(...)`, `.standby()` or `power_up()` to
    /// accept incoming packets.
    pub fn listen(&mut self) -> Result<(), NrfError> {
        if self.is_receiver() {
            self.ce.up()?;
        }
        Ok(())
    }

    /// Is there any incoming data to read?
    ///
    /// Works in both RX and TX modes. In TX mode, this function returns true if
    /// a ACK payload has been received.
    pub fn data_available(&mut self) -> Result<bool, NrfError> {
        self.read_register(FIFO_STATUS)
            .and_then(|(_, fifo_status)| Ok(fifo_status.trailing_zeros() >= 1))
    }

    /// Read data from the receiver queue, one packet at a time.
    ///
    /// The `process_packet` callback is fired for each packet, and is
    /// passed a slice into the packet data as argument.
    ///
    /// ``read_all`` returns the number of messages read.
    ///
    /// **Note**: this function puts the device in standby mode during
    /// the processing of the queue and restores operations when it returns *successfully*.
    /// So the `process_packet` callback should better return quickly.
    pub fn read_all<F>(&mut self, mut process_packet: F) -> Result<u8, NrfError>
    where
        F: FnMut(&[u8]) -> (),
    {
        // communication buffers
        let mut pl_wd: [u8; 2] = [0, 0]; // for packet width
        let mut receive_buffer = [0u8; 33]; // for packet
        let out_buffer = [R_RX_PAYLOAD; 33]; // for command
                                             // message counter
        let mut count = 0u8;
        // save CE state
        self.ce.save_state();
        // Standby
        self.ce.down()?;
        // process queue
        while self.data_available()? {
            self.send_command(&[R_RX_PL_WID, 0], &mut pl_wd)?;
            let width = pl_wd[1] as usize;
            if width != 0 {
                // can it be false?
                let ubound = width + 1;
                self.send_command(&out_buffer[..ubound], &mut receive_buffer[..ubound])?;
                process_packet(&receive_buffer[1..ubound]);
                count += 1;
            }
        }
        // Clear interrupt
        self.write_register(STATUS, 0b0100_0000)?;
        // Restore previous CE state
        self.ce.restore_state()?;
        Ok(count)
    }

    /// Queue (FIFO) data to be sent, one packet at a time.
    ///
    /// In TX mode, `pipe_num` is ignored. In RX mode, this function queues an ACK payload
    /// for the next message to arrive on `pipe_num`. if `pipe_num` is bigger than 5,
    /// it is capped to 5.
    ///
    /// The maximum size for a packet is 32 bytes.
    ///
    /// You can store a maximun of 3 payloads in the FIFO queue.
    ///
    /// **Note** : The payloads remain in the queue until the end of an _Enhanced Shockburst_ ™
    /// transaction:
    ///
    /// * In TX mode, a payload is removed from the send queue if and only if
    /// it has been successfully sent, that is, an ACK (with or without payload) has
    /// been received for it.
    ///
    /// * In RX mode, an ACK payload is removed from the queue if and only if
    /// it has been sent AND the pipe receives a new message, different from
    /// the one the ACK payload responded to. This is because the receiver has
    /// no mean to know whether the transmitter has received the ACK until
    /// it receives a new, different message from the same transmitter.
    /// So, it keeps the ACK payload under hand in case the transmitter resends the same
    /// packet over again.
    pub fn push(&mut self, pipe_num: u8, data: &[u8]) -> Result<(), NrfError> {
        let (status, fifo_status) = self.read_register(FIFO_STATUS)?;
        if (status & 1 != 0) || (fifo_status & 0b0010_0000 != 0) {
            // TX_FIFO is full
            println!("Sending queue is full!");
            Err(NrfError::WriteZeroQueueFull)
        } else {
            let command = if self.is_receiver() {
                let actual_pipe_num: u8 = if pipe_num < 6 { pipe_num } else { 5 };
                W_ACK_PAYLOAD | actual_pipe_num
            } else {
                W_TX_PAYLOAD
            };
            if data.len() > 32 {
                println!("Packet too big!");
                Err(NrfError::InvalidDataPacketTooBig)
            } else {
                let mut out_buffer = [command; 33];
                let ubound = data.len() + 1;
                out_buffer[1..ubound].copy_from_slice(data);
                let mut in_buffer = [0u8; 33];
                self.send_command(&out_buffer[..ubound], &mut in_buffer[..ubound])
            }
        }
    }

    /// [TX mode only] Send all packets in the TX FIFO queue.
    ///
    /// The call blocks until all packets are sent or the device reaches
    /// the `max_retries` number of retries after failure.
    /// Return the number of retries in case of success.
    ///
    /// The payloads that failed to be sent remain in the TX queue.
    /// You can call `.send()` again to relaunch a send/retry cycle or
    /// call `.flush_output` to clear the queue.
    ///
    /// # Errors
    /// Return Spidev errors as well as a custom io::ErrorKind::Timeout
    /// when the maximun number of retries has been reached.
    pub fn send(&mut self) -> Result<u8, NrfError> {
        // clear TX_DS and MAX_RT
        self.write_register(STATUS, 0x30)?;
        // init retry counter
        let mut counter = 0u8;
        let (_, fifo_status) = self.read_register(FIFO_STATUS)?;
        let mut packets_left = fifo_status & 0x10 == 0;
        while packets_left {
            // send with a 10us pulse
            self.ce.up()?;
            // sleep(Duration::new(0, 10_000));
            riscv::asm::delay(10_000);
            self.ce.down()?;
            let mut status = 0u8;
            let mut observe = 0u8;
            // wait for ACK
            while status & 0x30 == 0 {
                // wait at least 360us
                // sleep(Duration::new(0, 360_000));
                riscv::asm::delay(360_000);
                let outcome = self.read_register(OBSERVE_TX)?;
                status = outcome.0;
                observe = outcome.1;
            }
            // check MAX_RT
            if status & 0x10 > 0 {
                // failure
                // clear MAX_RT
                self.write_register(STATUS, 0x10)?;
                println!("Max retries reached!");
                // force return
                return Err(NrfError::MaxRetries);
            };
            // Success
            counter += observe & 0x0f;
            let (_, fifo_status) = self.read_register(FIFO_STATUS)?;
            println!("fifo_status: {:?}", fifo_status);
            packets_left = fifo_status & 0x10 == 0;
        }
        // clear TX_DS
        self.write_register(STATUS, 0x20)?;
        // if all sent, return retry counter
        Ok(counter)
    }

    /// Clear input queue.
    ///
    /// In RX mode, use only when device is in standby.
    pub fn flush_input(&mut self) -> Result<(), NrfError> {
        let mut buffer = [0u8];
        self.send_command(&[FLUSH_RX], &mut buffer)?;
        Ok(())
    }

    /// Clear output queue.
    ///
    /// In RX mode, use only when device is in standby.
    pub fn flush_output(&mut self) -> Result<(), NrfError> {
        let mut buffer = [0u8];
        self.send_command(&[FLUSH_TX], &mut buffer)?;
        Ok(())
    }
}

#[allow(dead_code)]
enum Action {
    Listen,
    Emit,
    Scan,
}

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();

    println!("Flash size: {}kb", hal::signature::flash_size_kb());
    println!("Chip UID: {:x?}", hal::signature::unique_id());
    let chip_id = hal::signature::chip_id();
    println!("Chip {}, DevID: 0x{:x}", chip_id.name(), chip_id.dev_id());
    println!("Clocks: {:?}", hal::rcc::clocks());

    let config = hal::Config::default();
    let p = hal::init(config);

    let csn = p.PC0;
    let ce = p.PC4;

    let sck = p.PC5;
    let mosi = p.PC6;
    let miso = p.PC7;

    let mut csn_pin = Output::new(csn, Level::Low, Default::default());
    csn_pin.set_high();

    let mut ce_pin = Output::new(ce.degrade(), Level::Low, Default::default());
    ce_pin.set_low();

    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = Hertz::mhz(10);

    let spi = Spi::new::<0>(p.SPI1, sck, mosi, miso, NoDma, NoDma, spi_config);

    let mut device = NRF24L01::new(spi, ce_pin).unwrap();

    // TODO: set action to perform here
    // let action = Action::Emit;
    // let action = Action::Scan;
    let action = Action::Listen;

    match action {
        Action::Emit => {
            let config = TXConfig {
                channel: 108,
                pa_level: PALevel::Low,
                pipe0_address: *b"abcde",
                max_retries: 3,
                retry_delay: 2,
                ..Default::default()
            };

            let message = b"hello";
            device.configure(&OperatingMode::TX(config)).unwrap();
            device.flush_output().unwrap();

            loop {
                println!("Entering loop");
                device.push(0, message).unwrap();
                match device.send() {
                    Ok(retries) => println!("Message sent, {} retries needed", retries),
                    Err(err) => {
                        println!("Destination unreachable: {:?}", err);
                        device.flush_output().unwrap();
                    }
                };
                riscv::asm::delay(5_000_00);
            }
        }
        Action::Listen => {
            let config = RXConfig {
                channel: 108,
                pa_level: PALevel::Low,
                pipe0_address: *b"abcde",
                ..Default::default()
            };

            device.configure(&OperatingMode::RX(config)).unwrap();
            device.listen().unwrap();

            loop {
                riscv::asm::delay(1_000_00);
                match device.data_available() {
                    Ok(_) => device
                        .read_all(|packet| {
                            println!("Received {:?} bytes", packet.len());
                            println!("Payload {:?}", packet);
                        })
                        .unwrap(),
                    Err(_) => {
                        println!("No data available");
                        0
                    }
                };
            }
        }
        Action::Scan => {
            let rounds = 10;
            let delay = 3;
            loop {
                println!("Starting Scan");
                let mut table = [0u32; 126];
                device.scan(rounds, delay, &mut table).unwrap();
                for (channel, obs) in table.iter().enumerate().take(126) {
                    println!("Channel {:}: {:}", channel, obs);
                }
                riscv::asm::delay(50_000_00);
            }
        }
    }
}
