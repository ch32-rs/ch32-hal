use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiBus as _;
use hal::dma::NoDma;
use hal::exti::ExtiInput;
use hal::gpio::{Input, Output};
use hal::spi::Spi;
use hal::{peripherals, spi};

use crate::hal;

pub mod cmds;
pub mod regs;

pub trait Response {
    fn decode(buf: &[u8]) -> Self;
    fn expected_len() -> usize {
        1
    }
}

pub trait Command {
    type Response: Response;
    fn encode(&self, buf: &mut [u8]) -> usize;
}

impl Response for () {
    fn decode(_buf: &[u8]) -> Self {
        ()
    }
    fn expected_len() -> usize {
        0
    }
}

impl Response for u8 {
    fn decode(buf: &[u8]) -> Self {
        buf[0]
    }
    fn expected_len() -> usize {
        1
    }
}
impl Response for u16 {
    fn decode(buf: &[u8]) -> Self {
        (buf[0] as u16) << 8 | buf[1] as u16
    }
    fn expected_len() -> usize {
        2
    }
}

#[derive(Clone, Copy)]
pub struct Status(pub u8);
impl Response for Status {
    fn decode(buf: &[u8]) -> Self {
        Self(buf[0])
    }
}
impl Status {
    pub fn is_error(&self) -> bool {
        let s = self.command_status();
        s >= 3 && s <= 6
    }

    pub fn is_data_ready(&self) -> bool {
        self.command_status() == 0x02
    }

    pub fn command_status(&self) -> u8 {
        (self.0 & 0b1110) >> 1
    }

    pub fn chip_mode(&self) -> u8 {
        (self.0 & 0b0111_0000) >> 4
    }
}
impl core::fmt::Debug for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Status")
            .field("error", &self.is_error())
            .field("command_status", &self.command_status())
            .field("chip_mode", &self.chip_mode())
            .finish()
    }
}

// ==========

#[derive(Debug, Clone, Copy)]
pub enum SetStandby {
    RC = 0x00,
    XOSC = 0x01,
}
impl Command for SetStandby {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_STANDBY;
        buf[1] = *self as u8;
        return 2;
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SetPacketType {
    GFSK = 0x00,
    LORA = 0x01,
}
impl Command for SetPacketType {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_PACKET_TYPE;
        buf[1] = *self as u8;
        return 2;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SetRfFrequency(u32);
const F_XTAL: u64 = 32_000_000; // 32MHz
impl SetRfFrequency {
    pub const fn hz(freq: u32) -> Self {
        //13.4.1.: RFfrequecy = (RFfreq * Fxtal) / 2^25
        Self((freq as u64 * 33554432 / F_XTAL) as u32)
    }
    pub const fn khz(freq: u32) -> Self {
        Self((freq as u64 * 33554432 * 1_000 / F_XTAL) as u32)
    }
    pub const fn raw(raw: u32) -> Self {
        Self(raw)
    }
}
impl Command for SetRfFrequency {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_RF_FREQUENCY;
        buf[1] = (self.0 >> 24) as u8;
        buf[2] = (self.0 >> 16) as u8;
        buf[3] = (self.0 >> 8) as u8;
        buf[4] = self.0 as u8;
        return 5;
    }
}

#[derive(Debug, Clone, Copy)]
pub enum TxcoVoltage {
    V1_6 = 0x00,
    V1_7 = 0x01,
    V1_8 = 0x02,
    V2_2 = 0x03,
    V2_4 = 0x04,
    V2_7 = 0x05,
    V3_0 = 0x06,
    V3_3 = 0x07,
}

#[derive(Debug, Clone, Copy)]
pub struct SetDIO3AsTCXOCtrl {
    voltage: TxcoVoltage,
    delay: u32,
}
impl SetDIO3AsTCXOCtrl {
    pub const fn new(voltage: TxcoVoltage, delay_ms: u32) -> Self {
        Self {
            voltage,
            delay: delay_ms << 6,
        }
    }
}
impl Command for SetDIO3AsTCXOCtrl {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_DIO3_AS_TCXO_CTRL;
        buf[1] = self.voltage as u8;
        buf[2] = (self.delay >> 16) as u8;
        buf[3] = (self.delay >> 8) as u8;
        buf[4] = (self.delay) as u8;
        return 5;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SetDIO2AsRfSwitchCtrl(pub bool);
impl Command for SetDIO2AsRfSwitchCtrl {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_DIO2_AS_RF_SWITCH_CTRL;
        buf[1] = if self.0 { 0x01 } else { 0x00 };
        return 2;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Calibrate(pub u8);
impl Calibrate {
    pub const ALL: Self = Self(0x7F);
}
impl Command for Calibrate {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::CALIBRATE;
        buf[1] = self.0;
        return 2;
    }
}
impl Default for Calibrate {
    fn default() -> Self {
        Self(0x7F)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CalibrateImage {
    freq1: u8,
    freq2: u8,
}
impl Command for CalibrateImage {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::CALIBRATE_IMAGE;
        buf[1] = self.freq1;
        buf[2] = self.freq2;
        return 3;
    }
}
impl CalibrateImage {
    pub const BAND_430_440: Self = Self {
        freq1: 0x6B,
        freq2: 0x6F,
    };
    pub const BAND_470_510: Self = Self {
        freq1: 0x75,
        freq2: 0x81,
    };
    pub const BAND_779_787: Self = Self {
        freq1: 0xC1,
        freq2: 0xC5,
    };
}

#[derive(Debug, Clone, Copy)]
pub struct SetPaConfig {
    pub pa_duty_cycle: u8,
    pub hp_max: u8,
    pub device_sel: u8,
    pub pa_lut: u8,
}
impl Command for SetPaConfig {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_PA_CONFIG;
        buf[1] = self.pa_duty_cycle;
        buf[2] = self.hp_max;
        buf[3] = self.device_sel;
        buf[4] = self.pa_lut;
        return 5;
    }
}
impl SetPaConfig {
    pub const POWER_22DBM: Self = Self {
        pa_duty_cycle: 0x04,
        hp_max: 0x07,
        device_sel: 0x00,
        pa_lut: 0x01,
    };
    pub const POWER_20DBM: Self = Self {
        pa_duty_cycle: 0x03,
        hp_max: 0x05,
        device_sel: 0x00,
        pa_lut: 0x01,
    };
}

#[derive(Debug, Clone, Copy)]
pub enum RampTime {
    _10U = 0x00,
    _20U = 0x01,
    _40U = 0x02,
    _80U = 0x03,
    _200U = 0x04,
    _800U = 0x05,
    _1700U = 0x06,
    _3400U = 0x07,
}

#[derive(Debug, Clone, Copy)]
pub struct SetTxParams {
    // - 17 (0xEF) to +14 (0x0E) dBm by step of 1 dB if low power PA is selected
    // - 9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
    pub power: i8,
    pub ramp_time: u8,
}
impl Command for SetTxParams {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_TX_PARAMS;
        buf[1] = self.power as u8;
        buf[2] = self.ramp_time;
        return 3;
    }
}
impl Default for SetTxParams {
    fn default() -> Self {
        Self {
            power: 14,
            ramp_time: RampTime::_200U as u8,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SetBufferBaseAddress {
    pub tx_base: u8,
    pub rx_base: u8,
}
impl Command for SetBufferBaseAddress {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_BUFFER_BASE_ADDRESS;
        buf[1] = self.tx_base;
        buf[2] = self.rx_base;
        return 3;
    }
}
impl Default for SetBufferBaseAddress {
    fn default() -> Self {
        Self {
            tx_base: 0x00,
            rx_base: 0x00,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GFSKPulseShape(pub u8);
impl GFSKPulseShape {
    pub const NO_FILTER: Self = Self(0x00);
    pub const GAUSSIAN_0_3: Self = Self(0x08);
    pub const GAUSSIAN_0_5: Self = Self(0x09);
    pub const GAUSSIAN_0_7: Self = Self(0x0A);
    pub const GAUSSIAN_1_0: Self = Self(0x0B);
}
#[derive(Debug, Clone, Copy)]
pub struct GFSKBandwidth(pub u8);
impl GFSKBandwidth {
    pub const RX_BW_4800: Self = Self(0x1F);
    pub const RX_BW_5800: Self = Self(0x17);
    pub const RX_BW_7300: Self = Self(0x0F);
    pub const RX_BW_9700: Self = Self(0x1E);
    pub const RX_BW_11700: Self = Self(0x16);
    pub const RX_BW_14600: Self = Self(0x0E);
    pub const RX_BW_19500: Self = Self(0x1D);
    pub const RX_BW_23400: Self = Self(0x15);
    pub const RX_BW_29300: Self = Self(0x0D);
    pub const RX_BW_39000: Self = Self(0x1C);
    pub const RX_BW_46900: Self = Self(0x14);
    pub const RX_BW_58600: Self = Self(0x0C);
    pub const RX_BW_78000: Self = Self(0x1B);
    pub const RX_BW_93800: Self = Self(0x13);
    pub const RX_BW_117300: Self = Self(0x0B);
    pub const RX_BW_156000: Self = Self(0x1A);
    pub const RX_BW_187200: Self = Self(0x12);
    pub const RX_BW_234300: Self = Self(0x0A);
    pub const RX_BW_312000: Self = Self(0x19);
    pub const RX_BW_373600: Self = Self(0x11);
    pub const RX_BW_467000: Self = Self(0x09);
    //pub const RX_BW_500000: Self = Self(0x18);
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum LoRaSpreadFactor {
    SF5 = 0x05,
    SF6 = 0x06,
    SF7 = 0x07,
    SF8 = 0x08,
    SF9 = 0x09,
    SF10 = 0x0A,
    SF11 = 0x0B,
    SF12 = 0x0C,
}

impl From<u8> for LoRaSpreadFactor {
    fn from(value: u8) -> Self {
        match value {
            0x05 => Self::SF5,
            0x06 => Self::SF6,
            0x07 => Self::SF7,
            0x08 => Self::SF8,
            0x09 => Self::SF9,
            0x0A => Self::SF10,
            0x0B => Self::SF11,
            0x0C => Self::SF12,
            _ => panic!("Invalid LoRa spread factor"),
        }
    }
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum LoRaBandWidth {
    /// 7.81 kHz
    BW7 = 0x00,
    /// 10.42 kHz
    BW10 = 0x08,
    /// 15.63 kHz
    BW15 = 0x01,
    /// 20.83 kHz
    BW20 = 0x09,
    /// 31.25 kHz
    BW31 = 0x02,
    /// 41.67 kHz
    BW41 = 0x0A,
    /// 62.50 kHz
    BW62 = 0x03,
    /// 125 kHz
    BW125 = 0x04,
    /// 250 kHz
    BW250 = 0x05,
    /// 500 kHz
    BW500 = 0x06,
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum LoraCodingRate {
    CR4_5 = 0x01,
    CR4_6 = 0x02,
    CR4_7 = 0x03,
    CR4_8 = 0x04,
}

#[derive(Clone, Copy)]
pub enum SetModulationParams {
    GFSK {
        br: u32, // u24 3 bytes
        pulse_shape: GFSKPulseShape,
        bandwidth: GFSKBandwidth,
        fdev: u32, // u24
    },
    LoRa {
        sf: LoRaSpreadFactor,
        bw: LoRaBandWidth,
        cr: LoraCodingRate,
        low_data_rate_optimize: bool,
    },
}
impl SetModulationParams {
    pub const LORA_DEFAULT: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF7,
        bw: LoRaBandWidth::BW125,
        cr: LoraCodingRate::CR4_5,
        low_data_rate_optimize: false,
    };
    pub const VARIATION1: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF9,
        bw: LoRaBandWidth::BW500,
        cr: LoraCodingRate::CR4_5,
        low_data_rate_optimize: true,
    };
}
impl Command for SetModulationParams {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        match self {
            Self::GFSK {
                br,
                pulse_shape,
                bandwidth,
                fdev,
            } => {
                buf[0] = cmds::SET_MODULATION_PARAMS;
                buf[1] = (*br >> 16) as u8;
                buf[2] = (*br >> 8) as u8;
                buf[3] = *br as u8;
                buf[4] = pulse_shape.0;
                buf[5] = bandwidth.0;
                buf[6] = (*fdev >> 16) as u8;
                buf[7] = (*fdev >> 8) as u8;
                buf[8] = *fdev as u8;
                return 9;
            }
            Self::LoRa {
                sf,
                bw,
                cr,
                low_data_rate_optimize,
            } => {
                buf[0] = cmds::SET_MODULATION_PARAMS;
                buf[1] = 0x01;
                buf[2] = *sf as u8;
                buf[3] = *bw as u8;
                buf[4] = *cr as u8;
                buf[5] = if *low_data_rate_optimize { 0x01 } else { 0x00 };
                return 6;
            }
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum LoRaHeaderType {
    /// Variable length packet (explicit header)
    VarLen = 0x00,
    /// Fixed length packet (implicit header)
    FixedLen = 0x01,
}

#[derive(Clone, Copy)]
pub enum SetPacketParams {
    GFSK {
        preamble_len: u16,
        preamble_detector_len: u8,
        sync_word_len: u8,
        addr_comp: u8,
        packet_type: u8,
        payload_len: u8,
        crc_type: u8,
        whitening: bool,
    },
    LoRa {
        preamble_len: u16,
        header_type: LoRaHeaderType,
        payload_len: u8,
        crc: bool,
        invert_iq: bool,
        // remain is fill with 0
    },
}
impl SetPacketParams {
    pub const LORA_DEFAULT: Self = Self::LoRa {
        preamble_len: 8,
        header_type: LoRaHeaderType::VarLen,
        payload_len: 0,
        crc: false,
        invert_iq: false,
    };
}
impl Command for SetPacketParams {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        match self {
            Self::GFSK {
                preamble_len,
                preamble_detector_len,
                sync_word_len,
                addr_comp,
                packet_type,
                payload_len,
                crc_type,
                whitening,
            } => {
                buf[0] = cmds::SET_PACKET_PARAMS;
                buf[1] = (*preamble_len >> 8) as u8;
                buf[2] = *preamble_len as u8;
                buf[3] = *preamble_detector_len;
                buf[4] = *sync_word_len;
                buf[5] = *addr_comp;
                buf[6] = *packet_type;
                buf[7] = *payload_len;
                buf[8] = *crc_type;
                buf[9] = if *whitening { 0x01 } else { 0x00 };
                return 10;
            }
            Self::LoRa {
                preamble_len,
                header_type,
                payload_len,
                crc,
                invert_iq,
            } => {
                buf[0] = cmds::SET_PACKET_PARAMS;
                buf[1] = (*preamble_len >> 8) as u8;
                buf[2] = *preamble_len as u8;
                buf[3] = *header_type as u8;
                buf[4] = *payload_len;
                buf[5] = if *crc { 0x01 } else { 0x00 };
                buf[6] = if *invert_iq { 0x01 } else { 0x00 };
                return 10;
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SetDioIrqParams {
    pub irq_mask: u16,
    pub dio1_mask: u16,
    pub dio2_mask: u16,
    pub dio3_mask: u16,
}
impl SetDioIrqParams {
    pub const DEFAULT: Self = Self {
        // TxDone, RxDone,
        // Timout
        irq_mask: 0b1_00000011,
        dio1_mask: 0b1_00000011,
        dio2_mask: 0x0000,
        dio3_mask: 0x0000,
    };
}
impl Command for SetDioIrqParams {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_DIO_IRQ_PARAMS;
        buf[1] = (self.irq_mask >> 8) as u8;
        buf[2] = self.irq_mask as u8;
        buf[3] = (self.dio1_mask >> 8) as u8;
        buf[4] = self.dio1_mask as u8;
        buf[5] = (self.dio2_mask >> 8) as u8;
        buf[6] = self.dio2_mask as u8;
        buf[7] = (self.dio3_mask >> 8) as u8;
        buf[8] = self.dio3_mask as u8;
        return 9;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GetIrqStatus;
impl Command for GetIrqStatus {
    type Response = u16;
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::GET_IRQ_STATUS;
        return 1;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GetStatus;
impl Command for GetStatus {
    type Response = Status;
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::GET_STATUS;
        return 1;
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct SetRx {
    pub timeout: u32,
}
impl Command for SetRx {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_RX;
        buf[1] = (self.timeout >> 16) as u8;
        buf[2] = (self.timeout >> 8) as u8;
        buf[3] = self.timeout as u8;
        return 4;
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct SetTx {
    pub timeout: u32,
}
impl Command for SetTx {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_TX;
        buf[1] = (self.timeout >> 16) as u8;
        buf[2] = (self.timeout >> 8) as u8;
        buf[3] = self.timeout as u8;
        return 4;
    }
}

#[derive(Debug)]
pub struct ReadRegister<T> {
    address: u16,
    _phantom: core::marker::PhantomData<T>,
}
impl<T> Command for ReadRegister<T>
where
    T: Response,
{
    type Response = T;
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::READ_REGISTER;
        buf[1] = (self.address >> 8) as u8;
        buf[2] = self.address as u8;
        return 3;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct WriteRegister<'a> {
    address: u16,
    data: &'a [u8],
}
impl<'a> Command for WriteRegister<'a> {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::WRITE_REGISTER;
        buf[1] = (self.address >> 8) as u8;
        buf[2] = self.address as u8;
        buf[3..3 + self.data.len()].copy_from_slice(self.data);
        return 3 + self.data.len();
    }
}

#[derive(Debug, Clone, Copy)]
pub struct WriteBuffer<'a> {
    pub offset: u8,
    pub data: &'a [u8],
}
impl<'a> Command for WriteBuffer<'a> {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::WRITE_BUFFER;
        buf[1] = self.offset;
        buf[2..2 + self.data.len()].copy_from_slice(self.data);
        return 2 + self.data.len();
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Stats {
    pub packet_received: u16,
    pub packet_crc_error: u16,
    pub packet_length_error: u16,
}
impl Response for Stats {
    fn decode(buf: &[u8]) -> Self {
        Self {
            packet_received: (buf[0] as u16) << 8 | buf[1] as u16,
            packet_crc_error: (buf[2] as u16) << 8 | buf[3] as u16,
            packet_length_error: (buf[4] as u16) << 8 | buf[5] as u16,
        }
    }
    fn expected_len() -> usize {
        6
    }
}
#[derive(Debug, Clone, Copy)]
pub struct GetStats;
impl Command for GetStats {
    type Response = Stats;
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::GET_STATS;
        return 1;
    }
}

// ==========
// The device

pub struct Config {}
pub struct SX1268 {
    spi: Spi<'static, peripherals::SPI1, NoDma, NoDma>,
    cs: Output<'static>,
    pub busy: ExtiInput<'static>,
}

impl SX1268 {
    pub fn new(
        spi: Spi<'static, peripherals::SPI1, NoDma, NoDma>,
        cs: Output<'static>,
        busy: ExtiInput<'static>,
    ) -> Self {
        Self { spi, cs, busy }
    }

    pub fn init(&mut self, _config: Config, delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        let status = self.get_status()?;
        if status.chip_mode() == 0x02 {
            // already in standby RC
        } else {
            self.send_command(SetStandby::RC)?;
        }

        // must be first config command
        self.send_command(SetPacketType::LORA)?;
        self.send_command(SetRfFrequency::hz(433_450_000))?;
        self.send_command(SetDIO3AsTCXOCtrl::new(TxcoVoltage::V3_3, 500))?;

        self.send_command(Calibrate::ALL)?;
        self.send_command(CalibrateImage::BAND_430_440)?;

        self.send_command(SetPaConfig::POWER_22DBM)?;

        self.send_command(SetTxParams::default())?;
        self.send_command(SetBufferBaseAddress::default())?;

        self.send_command(SetModulationParams::VARIATION1)?;
        self.send_command(SetPacketParams::LORA_DEFAULT)?;

        // use dio1 as irq
        self.send_command(SetDioIrqParams::DEFAULT)?;
        self.send_command(SetDIO2AsRfSwitchCtrl(true))?;

        Ok(())
    }

    pub fn set_rx(&mut self, delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        delay.delay_ns(32);
        self.send_command(SetRx { timeout: 0 })?;
        Ok(())
    }

    pub fn set_tx(&mut self, delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        todo!();
    }

    pub fn tx_bytes(&mut self, data: &[u8], delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        self.write_buffer(0x00, data)?;

        self.send_command(SetPacketParams::LoRa {
            preamble_len: 8,
            header_type: LoRaHeaderType::VarLen,
            payload_len: data.len() as _,
            crc: false,
            invert_iq: false,
        })?;

        // no timeout
        self.send_command(SetTx::default())?;

        Ok(())
    }

    pub fn get_irq_status(&mut self) -> Result<u16, spi::Error> {
        self.send_command(GetIrqStatus)
    }

    pub fn get_status(&mut self) -> Result<Status, spi::Error> {
        self.send_command(GetStatus)
    }

    pub fn get_stats(&mut self) -> Result<Stats, spi::Error> {
        self.send_command(GetStats)
    }

    fn send_command<C: Command>(&mut self, cmd: C) -> Result<C::Response, spi::Error> {
        let mut buf = [0u8; 255];
        let len = cmd.encode(&mut buf);
        let resp_len = C::Response::expected_len();

        //hal::println!("!! in : {:02x?}", &buf[..len]);
        if resp_len != 0 {
            let total_len = len + 1 + C::Response::expected_len();

            self.cs.set_low();
            self.spi.transfer_in_place(&mut buf[..total_len])?;
            self.cs.set_high();

            let reply = &buf[len + 1..];
            //hal::println!("!! out: {:02x?}", &buf[..total_len]);
            Ok(C::Response::decode(reply))
        } else {
            self.cs.set_low();
            self.spi.transfer_in_place(&mut buf[..len])?;
            self.cs.set_high();
            //hal::println!("!! out: {:02x?}", &buf[..len]);
            Ok(C::Response::decode(&[]))
        }
    }

    pub fn write_register(&mut self, address: u16, data: &[u8]) -> Result<(), spi::Error> {
        self.send_command(WriteRegister { address, data })?;
        Ok(())
    }

    pub fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), spi::Error> {
        self.send_command(WriteBuffer { offset, data })?;
        Ok(())
    }

    pub fn read_register(&mut self, address: u16, data: &mut [u8]) -> Result<(), spi::Error> {
        let mut out = [cmds::READ_REGISTER, (address >> 8) as u8, address as u8, cmds::NOP];

        self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut out[..])?;
        self.spi.blocking_transfer_in_place(data)?;
        self.cs.set_high();

        Ok(())
    }

    pub fn read_buffer(&mut self, offset: u8, data: &mut [u8]) -> Result<(), spi::Error> {
        let mut out = [cmds::READ_BUFFER, offset, cmds::NOP];

        self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut out[..])?;
        self.spi.blocking_transfer_in_place(data)?;
        self.cs.set_high();

        Ok(())
    }
}
