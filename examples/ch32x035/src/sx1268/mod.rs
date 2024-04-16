use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiBus as _;
use hal::dma::NoDma;
use hal::exti::ExtiInput;
use hal::gpio::{Input, Output};
use hal::spi::Spi;
use hal::{peripherals, println, spi};

use crate::hal;

pub mod cmds;
pub mod consts;
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
impl Response for i8 {
    fn decode(buf: &[u8]) -> Self {
        buf[0] as _
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

impl Response for (u8, u8) {
    fn decode(buf: &[u8]) -> Self {
        (buf[0], buf[1])
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
        s >= 3 && s <= 5
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

    pub fn is_standby_rc(&self) -> bool {
        self.chip_mode() == 0x02
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

pub struct SetRegulatorMode(u8);
impl Command for SetRegulatorMode {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_REGULATOR_MODE;
        buf[1] = self.0;
        return 2;
    }
}
impl SetRegulatorMode {
    pub const LDO: Self = Self(0x00);
    pub const DCDC: Self = Self(0x01);
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
    pub freq1: u8,
    pub freq2: u8,
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
    pub fn for_freq(freq: u32) -> Self {
        if freq > 900_000_000 {
            Self::BAND_902_928
        } else if freq > 850_000_000 {
            Self::BAND_864_870
        } else if freq > 770_000_000 {
            Self::BAND_779_787
        } else if freq > 460_000_000 {
            Self::BAND_470_510
        } else {
            Self::BAND_430_440
        }
    }
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
    // for sx1262
    pub const BAND_864_870: Self = Self {
        freq1: 0xD7,
        freq2: 0xD8,
    };
    pub const BAND_902_928: Self = Self {
        freq1: 0xE1,
        freq2: 0xE9,
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
    pub const POWER_17DBM: Self = Self {
        pa_duty_cycle: 0x02,
        hp_max: 0x03,
        device_sel: 0x00,
        pa_lut: 0x01,
    };
    pub const POWER_14DBM: Self = Self {
        pa_duty_cycle: 0x04,
        hp_max: 0x06,
        device_sel: 0x00,
        pa_lut: 0x01,
    };
    pub const POWER_10DBM: Self = Self {
        pa_duty_cycle: 0x00,
        hp_max: 0x03,
        device_sel: 0x00,
        pa_lut: 0x01,
    };
    pub const POWER_15DBM_SX1262: Self = Self {
        pa_duty_cycle: 0x06,
        hp_max: 0x00,
        device_sel: 0x01,
        pa_lut: 0x01,
    };
}

pub struct SetRxTxFallbackMode(u8);
impl SetRxTxFallbackMode {
    pub const STDBY_RC: Self = Self(0x20);
    pub const STDBY_XOSC: Self = Self(0x30);
    pub const FS: Self = Self(0x40);
}
impl Command for SetRxTxFallbackMode {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_RX_TX_FALLBACK_MODE;
        buf[1] = self.0;
        return 2;
    }
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
    pub ramp_time: RampTime,
}
impl SetTxParams {
    pub const fn new(dbm: i8, ramp_time: RampTime) -> Self {
        Self { power: dbm, ramp_time }
    }
}
impl Command for SetTxParams {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_TX_PARAMS;
        buf[1] = self.power as u8;
        buf[2] = self.ramp_time as u8;
        return 3;
    }
}
impl Default for SetTxParams {
    fn default() -> Self {
        Self {
            power: 22,
            ramp_time: RampTime::_3400U,
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
pub struct SetLoRaSymbNumTimeout(pub u8);
impl Command for SetLoRaSymbNumTimeout {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_LORA_SYMB_NUM_TIMEOUT;
        buf[1] = self.0;
        return 2;
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
pub enum LoRaBandwidth {
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

    // for LLCC68
    /// 125 kHz
    BW125 = 0x04,
    /// 250 kHz
    BW250 = 0x05,
    /// 500 kHz
    BW500 = 0x06,
}

#[derive(Copy, Clone)]
pub struct LoraCodingRate(u8);
impl LoraCodingRate {
    pub const CR_4_5: Self = Self(0x01);
    pub const CR_4_6: Self = Self(0x02);
    pub const CR_4_7: Self = Self(0x03);
    pub const CR_4_8: Self = Self(0x04);
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
        bw: LoRaBandwidth,
        cr: LoraCodingRate,
        low_data_rate_optimize: bool,
    },
}
impl SetModulationParams {
    pub const LORA_DEFAULT: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF7,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_5,
        low_data_rate_optimize: false,
    };
    pub const LORA_KBPS_62_5: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF5,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_5,
        low_data_rate_optimize: false,
    };
    pub const LORA_KBPS_38_4: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF5,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_8,
        low_data_rate_optimize: false,
    };
    pub const LORA_KBPS_19_2: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF7,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_6,
        low_data_rate_optimize: false,
    };
    pub const LORA_KBPS_9_6: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF8,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_6,
        low_data_rate_optimize: false,
    };
    pub const LORA_KBPS_4_8: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF8,
        bw: LoRaBandwidth::BW250,
        cr: LoraCodingRate::CR_4_5,
        low_data_rate_optimize: false,
    };
    pub const LORA_KBPS_2_4: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF11,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_5,
        low_data_rate_optimize: false,
    };
    pub const LORA_KBPS_1_2: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF11,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_5,
        low_data_rate_optimize: false,
    };
    pub const LORA_KBPS_0_3: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF11,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_5,
        low_data_rate_optimize: false,
    };
    pub const VARIATION1: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF9,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_5,
        low_data_rate_optimize: false,
    };
    pub const VARIATION2: Self = Self::LoRa {
        sf: LoRaSpreadFactor::SF9,
        bw: LoRaBandwidth::BW500,
        cr: LoraCodingRate::CR_4_7,
        low_data_rate_optimize: false,
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
                buf[1] = *sf as u8;
                buf[2] = *bw as u8;
                buf[3] = cr.0;
                buf[4] = if *low_data_rate_optimize { 0x01 } else { 0x00 };
                return 9;
            }
        }
    }
}

// Only when PacketType = 0x01 (LoRa)
#[derive(Clone, Copy)]
pub struct SetLoRaPacketParams {
    /// 前导码长度
    pub preamble_len: u16,
    pub fixed_len: bool,
    pub payload_len: u8,
    pub crc: bool,
    pub invert_iq: bool,
}
impl SetLoRaPacketParams {
    pub const fn with_payload_len(len: u8) -> Self {
        Self {
            preamble_len: 60,
            fixed_len: false,
            payload_len: len,
            crc: true, // 必须一致
            invert_iq: true,
        }
    }
    pub const DEFAULT: Self = Self {
        preamble_len: 60,
        fixed_len: false,
        payload_len: 0,
        crc: true,
        invert_iq: false,
    };
}
impl Command for SetLoRaPacketParams {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_PACKET_PARAMS;
        buf[1] = (self.preamble_len >> 8) as u8;
        buf[2] = self.preamble_len as u8;
        buf[3] = if self.fixed_len { 0x01 } else { 0x00 };
        buf[4] = self.payload_len;
        buf[5] = if self.crc { 0x01 } else { 0x00 };
        buf[6] = if self.invert_iq { 0x01 } else { 0x00 };
        return 10;
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
        preamble_len: 60,
        header_type: LoRaHeaderType::VarLen,
        payload_len: 0,
        crc: true,
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
pub struct ClearIrqStatus(pub u16);
impl Command for ClearIrqStatus {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::CLEAR_IRQ_STATUS;
        buf[1] = (self.0 >> 8) as u8;
        buf[2] = self.0 as u8;
        return 3;
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
impl SetRx {
    pub const SINGLE: Self = Self { timeout: 0 };
    pub const CONTINUOUS: Self = Self { timeout: 0xFF_FFFF };
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

#[derive(Debug, Clone, Copy)]
pub enum StopTimerOnPreamble {
    YES = 0x01,
    NO = 0x00,
}
impl Command for StopTimerOnPreamble {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::STOP_TIMER_ON_PREAMBLE;
        buf[1] = *self as u8;
        return 2;
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

pub struct SetFs;
impl Command for SetFs {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_FS;
        return 1;
    }
}

pub struct SetTxContinuousWave;
impl Command for SetTxContinuousWave {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_TX_CONTINUOUS_WAVE;
        return 1;
    }
}

pub struct SetTxInfinitePreamble;
impl Command for SetTxInfinitePreamble {
    type Response = ();
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::SET_TX_INFINITE_PREAMBLE;
        return 1;
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

pub struct GetRxBufferStatus;
impl Command for GetRxBufferStatus {
    type Response = (u8, u8);
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::GET_RX_BUFFER_STATUS;
        return 1;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LoRaPacketStatus {
    // Average over last packet received of RSSI Actual signal power is –RssiPkt/2 (dBm)
    pub rssi_pkt_dbm: i8,
    // Estimation of SNR on last packet received in two’s compliment format multiplied by 4. LoRa® Actual SNR in dB =SnrPkt/4
    pub snr_pkt_db: i8,
    // Estimation of RSSI of the LoRa® signal (after despreading) on last packet received. Actual Rssi in dB = -SignalRssiPkt/2
    pub signal_rssi_pkt_dbm: i8,
}
impl Response for LoRaPacketStatus {
    fn decode(buf: &[u8]) -> Self {
        Self {
            rssi_pkt_dbm: -(buf[0] as i8) / 2,
            snr_pkt_db: -(buf[1] as i8) / 4,
            signal_rssi_pkt_dbm: -(buf[2] as i8) / 2,
        }
    }
    fn expected_len() -> usize {
        3
    }
}
pub struct GetLoRaPacketStatus;
impl Command for GetLoRaPacketStatus {
    type Response = LoRaPacketStatus;
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::GET_PACKET_STATUS;
        return 1;
    }
}

pub struct GetRssiInst;
impl Command for GetRssiInst {
    type Response = i8;
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::GET_RSSI_INST;
        return 1;
    }
}

// Miscellaneous
pub struct GetDeviceErrors;
impl Command for GetDeviceErrors {
    type Response = u16;
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::GET_DEVICE_ERRORS;
        return 1;
    }
}

pub struct ClearDeviceErrors;
impl Command for ClearDeviceErrors {
    type Response = Status; // return status
    fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0] = cmds::CLEAR_DEVICE_ERRORS;
        return 1;
    }
}

// ==========
// The module

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoRaModule {
    // SX1262
    STM32WL,
    // SX1268
    EByteE22_400M,
    EByteE22_900M,
    // LLCC68
    EByteE220_400M,
    EByteE220_900M,
}

// ==========
// The chip

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoRaChip {
    /// +22dBm, Continuous frequency coverage from 150 MHz to 960 MHz
    // SF7 to SF9 at 125kHz, SF7 to SF10 at 250kHz, and SF7 to SF11 at 500kHz
    LLCC68,
    /// +15dBm
    SX1261,
    SX1262,
    /// +22dBm, Continuous frequency coverage from 410 MHz to 810 MHz, China frequency bands
    SX1268,
    SX1276,
    SX1277,
    // 137MHz to 525MHz
    SX1278,
    SX1279,
    // TODO:
    //  SX1272/73
}

// ==========
// The device

// Device configuration
#[derive(Default)]
pub struct Config {
    pub freq: u32,
    pub use_dio2_as_rfswitch: bool,
    pub use_dcdc: bool,
    pub dio3_as_txco_ctrl: Option<(TxcoVoltage, u32)>,
    pub xosc_trim: Option<(u8, u8)>,
}

pub struct RfConfig {
    pub freq: u32,
    pub modulation_params: SetModulationParams,
    pub packet_params: SetLoRaPacketParams,
}

pub struct SX1268<CS> {
    spi: Spi<'static, peripherals::SPI1, NoDma, NoDma>,
    cs: CS,
}

impl<CS: embedded_hal::digital::OutputPin> SX1268<CS> {
    pub fn new(spi: Spi<'static, peripherals::SPI1, NoDma, NoDma>, cs: CS) -> Self {
        Self { spi, cs }
    }

    pub fn init_device(&mut self, config: Config, delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        // ensure standby
        let status = self.get_status()?;
        if !status.is_standby_rc() {
            self.send_command(SetStandby::RC)?;
        }

        // init rf switch
        if config.use_dio2_as_rfswitch {
            hal::println!("[init] DIO2 as TXEN");
            self.send_command(SetDIO2AsRfSwitchCtrl(true))?;
        }

        // set lora modem
        self.send_command(SetPacketType::LORA)?;
        self.set_lora_sync_word(consts::LORA_MAC_PUBLIC_SYNCWORD)?;

        // set oscillator
        if let Some((voltage, delay)) = config.dio3_as_txco_ctrl {
            hal::println!("[init] DIO3 as TXCO CTRL");
            self.send_command(SetDIO3AsTCXOCtrl::new(voltage, 500))?;
            Delay.delay_ms(600);
            self.write_register(regs::XTA_TRIM, 0x2F)?; // 补偿33.4pf电容器

            self.send_command(ClearDeviceErrors)?;

            self.send_command(Calibrate::ALL)?; // must for rx, or else IMG_CALIB_ERR
        }

        self.send_command(SetStandby::XOSC)?;

        // set regulator mode
        if config.use_dcdc {
            hal::println!("[init] DCDC");
            self.send_command(SetRegulatorMode::DCDC)?;
        }

        // set buffer base address
        self.send_command(SetBufferBaseAddress::default())?;

        // set_tx_power_and_ramp_time
        {
            self.send_command(SetPaConfig::POWER_14DBM)?;
            self.send_command(SetTxParams::new(14, RampTime::_3400U))?;

            // TODO rx_gain and retention
            // Rx Boosted
            // Set register 0x029F to 0x01
            // Set register 0x02A0 to 0x08
            // Set register 0x02A1 to 0xAC
        }

        Ok(())
    }

    pub fn prepare_for_tx(&mut self, rf_config: &RfConfig) -> Result<(), spi::Error> {
        let status = self.get_status()?;
        if !status.is_standby_rc() {
            self.send_command(SetStandby::RC)?;
        }

        self.send_command(CalibrateImage::for_freq(rf_config.freq))?;
        Delay.delay_ms(100);

        self.send_command(SetRfFrequency::hz(rf_config.freq))?;

        // set_modulation_params
        self.send_command(rf_config.modulation_params)?;
        // TODO: reg write
        self.send_command(rf_config.packet_params)?;

        Delay.delay_ms(100);

        self.get_stats()?;

        //self.send_command(SetDIO3AsTCXOCtrl::new(TxcoVoltage::V3_3, 500))?;
        //self.send_command(SetStandby::XOSC)?;

        Ok(())
    }

    pub fn tx(&mut self, data: &[u8], rf_config: &RfConfig) -> Result<(), spi::Error> {
        let status = self.get_status()?;
        if !status.is_standby_rc() {
            println!("set standby rc");
            self.send_command(SetStandby::RC)?;
        }

        //self.send_command(SetStandby::XOSC)?;
        // Delay.delay_ms(400); // ????

        // set 有效
        self.send_command(SetRfFrequency::hz(rf_config.freq))?;
        // Delay.delay_ms(100);

        let mut set_packet_params = rf_config.packet_params;
        set_packet_params.payload_len = data.len() as u8;
        self.send_command(set_packet_params)?;

        self.write_buffer(0, data)?;

        self.send_command(SetDioIrqParams::DEFAULT)?;

        self.send_command(SetTx::default())?;

        Ok(())
    }

    pub fn prepare_for_rx(&mut self, rf_config: &RfConfig) -> Result<(), spi::Error> {
        let status = self.get_status()?;
        if !status.is_standby_rc() {
            self.send_command(SetStandby::RC)?;
        }

        self.send_command(SetDIO3AsTCXOCtrl::new(TxcoVoltage::V3_3, 500))?;
        self.send_command(SetStandby::XOSC)?;

        self.send_command(CalibrateImage::for_freq(rf_config.freq))?;

        println!("cali {}", rf_config.freq);

        // set_modulation_params
        self.send_command(rf_config.modulation_params)?;
        // TODO: reg write
        self.send_command(rf_config.packet_params)?;

        self.send_command(SetRfFrequency::hz(rf_config.freq))?;

        self.send_command(SetDioIrqParams::DEFAULT)?;

        self.send_command(StopTimerOnPreamble::YES)?;

        self.send_command(SetRx { timeout: 0 })?;

        Ok(())
    }

    pub fn init_e220_1(&mut self, config: Config, delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        let status = self.get_status()?;
        if !status.is_standby_rc() {}

        self.send_command(SetStandby::RC)?;

        if let Some((xta_trim, xtb_trim)) = config.xosc_trim {
            // E220
            self.send_command(SetStandby::XOSC)?;
            self.write_register(regs::XTA_TRIM, xta_trim)?;
            self.write_register(regs::XTB_TRIM, xtb_trim)?;
        }

        if config.use_dio2_as_rfswitch {
            hal::println!("DIO2 as TXEN");
            self.send_command(SetDIO2AsRfSwitchCtrl(true))?;
        }

        self.send_command(SetRegulatorMode::DCDC)?;

        self.send_command(SetBufferBaseAddress::default())?;

        self.send_command(SetPacketType::LORA)?;

        // self.send_command(SetModulationParams::LORA_DEFAULT)?;
        self.send_command(SetModulationParams::LORA_KBPS_2_4)?;

        pub const LORA_SYNC_WORD: u16 = 0x3444; // pub,  0x1424 private
        self.set_lora_sync_word(LORA_SYNC_WORD)?;

        let rf_freq = config.freq;
        hal::println!("rf_freq: {}", rf_freq);
        self.send_command(SetRfFrequency::hz(rf_freq))?;

        //self.send_command(Calibrate::ALL)?;
        //self.send_command(CalibrateImage::BAND_470_510)?;

        {
            // power should match
            self.send_command(SetPaConfig::POWER_14DBM)?;
            self.send_command(SetTxParams::new(14, RampTime::_3400U))?;
        }

        self.send_command(SetLoRaPacketParams::DEFAULT)?;

        // DIO & irq
        self.send_command(SetDioIrqParams::DEFAULT)?;

        Ok(())
    }

    pub fn init_e220(&mut self, config: Config, delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        let status = self.get_status()?;
        if !status.is_standby_rc() {
            self.send_command(SetStandby::RC)?;
        }

        if let Some((xta_trim, xtb_trim)) = config.xosc_trim {
            // E220
            self.send_command(SetStandby::XOSC)?;
            self.write_register(regs::XTA_TRIM, xta_trim)?;
            self.write_register(regs::XTB_TRIM, xtb_trim)?;
        }

        self.send_command(SetStandby::RC)?;

        self.send_command(SetRegulatorMode::DCDC)?;

        if config.use_dio2_as_rfswitch {
            hal::println!("DIO2 as TXEN");
            self.send_command(SetDIO2AsRfSwitchCtrl(true))?;
        }

        self.send_command(SetPacketType::LORA)?;

        let rf_freq = config.freq;
        hal::println!("rf_freq: {}", rf_freq);
        self.send_command(SetRfFrequency::hz(rf_freq))?;

        self.send_command(Calibrate::ALL)?;
        self.send_command(CalibrateImage::BAND_470_510)?;

        self.send_command(SetBufferBaseAddress::default())?;

        {
            // power should match
            self.send_command(SetPaConfig::POWER_20DBM)?;
            self.send_command(SetTxParams::new(22, RampTime::_3400U))?;
        }

        // self.send_command(SetModulationParams::LORA_DEFAULT)?;
        self.send_command(SetModulationParams::LORA_KBPS_2_4)?;
        self.send_command(SetLoRaPacketParams::DEFAULT)?;

        // DIO & irq
        self.send_command(SetDioIrqParams::DEFAULT)?;

        pub const LORA_SYNC_WORD: u16 = 0x3444; // pub,  0x1424 private
        self.set_lora_sync_word(LORA_SYNC_WORD)?;

        Ok(())
    }

    pub fn init_e22(&mut self, config: Config, delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        let status = self.get_status()?;
        if !status.is_standby_rc() {
            self.send_command(SetStandby::RC)?;
        }

        if let Some((voltage, delay)) = config.dio3_as_txco_ctrl {
            hal::println!("DIO3 as TXCO CTRL");
            self.send_command(SetDIO3AsTCXOCtrl::new(voltage, delay))?;
            // self.write_register(regs::XTA_TRIM, 0x2F)?; // this is the default behavior
        }

        // comment this if rx
        //        self.send_command(Calibrate::ALL)?;
        // self.send_command(SetStandby::XOSC)?;

        if config.use_dio2_as_rfswitch {
            hal::println!("DIO2 as TXEN");
            self.send_command(SetDIO2AsRfSwitchCtrl(true))?;
        }
        // use dio1 as irq
        self.send_command(SetDioIrqParams::DEFAULT)?;

        self.send_command(SetRegulatorMode::DCDC)?;

        self.send_command(SetBufferBaseAddress::default())?;

        self.send_command(SetPacketType::LORA)?;

        // self.send_command(SetModulationParams::LORA_DEFAULT)?;
        self.send_command(SetModulationParams::LORA_KBPS_2_4)?;

        pub const LORA_SYNC_WORD: u16 = 0x3444; // pub,  0x1424 private
        self.set_lora_sync_word(LORA_SYNC_WORD)?;

        let rf_freq = config.freq;
        hal::println!("rf_freq: {}", rf_freq);
        self.send_command(SetRfFrequency::hz(rf_freq))?;
        self.send_command(CalibrateImage::BAND_470_510)?;

        self.send_command(Calibrate::ALL)?;
        self.send_command(CalibrateImage::BAND_470_510)?;

        {
            // power should match
            self.send_command(SetPaConfig::POWER_14DBM)?;
            self.send_command(SetTxParams::new(14, RampTime::_3400U))?;
        }

        self.send_command(SetLoRaPacketParams::DEFAULT)?;

        // DIO & irq
        self.send_command(SetDioIrqParams::DEFAULT)?;

        self.send_command(SetStandby::RC)?;

        Ok(())
    }

    pub fn init(&mut self, config: Config, delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        // wake up
        let status = self.get_status()?;
        if status.chip_mode() == 0x02 {
            // already in standby RC
        }
        //self.send_command(SetStandby::XOSC)?;
        self.send_command(SetStandby::RC)?;

        if let Some((voltage, delay)) = config.dio3_as_txco_ctrl {
            hal::println!("DIO3 as TXCO CTRL");
            self.send_command(SetDIO3AsTCXOCtrl::new(voltage, delay))?;
            //self.send_command(SetStandby::XOSC)?;
            self.write_register(regs::XTA_TRIM, 0x2F)?;
            //self.write_register(regs::XTB_TRIM, 0x2F)?;
            //  重新计算修正时钟
            self.send_command(Calibrate::ALL)?;
            self.send_command(SetStandby::XOSC)?;

            // 进入 STDBY_XOSC 待机配置模式
        }

        if let Some((xta_trim, xtb_trim)) = config.xosc_trim {
            // E220
            self.send_command(SetStandby::XOSC)?;
            self.write_register(regs::XTA_TRIM, xta_trim)?;
            self.write_register(regs::XTB_TRIM, xtb_trim)?;
        } else {
            // E22
            // self.send_command(SetStandby::RC)?;
            //  self.send_command(SetStandby::XOSC)?;
        }

        self.send_command(SetRegulatorMode::DCDC)?;

        if config.use_dio2_as_rfswitch {
            hal::println!("DIO2 as TXEN");
            self.send_command(SetDIO2AsRfSwitchCtrl(true))?;
        }
        // use dio1 as irq
        self.send_command(SetDioIrqParams::DEFAULT)?;

        // E220
        // 进入 STDBY_XOSC 待机配置模式
        //        const XTA_TRIM_VALUE: u8 = 0x1C;
        //      self.send_command(SetStandby::XOSC)?;
        //    self.write_register(regs::XTA_TRIM, XTA_TRIM_VALUE)?;
        //  self.write_register(regs::XTB_TRIM, XTA_TRIM_VALUE)?;
        // self.send_command(SetDIO3AsTCXOCtrl::new(TxcoVoltage::V3_3, 500))?;

        self.send_command(SetBufferBaseAddress::default())?;

        // ========== rf part
        let rf_freq = config.freq;
        hal::println!("rf_freq: {}", rf_freq);

        // radio config sequence
        self.send_command(SetPacketType::LORA)?;

        // when tx, callibrate image must after set rf freq
        self.send_command(SetRfFrequency::hz(rf_freq))?;

        self.send_command(CalibrateImage::BAND_470_510)?;

        // must in STDBY_RC mode
        // self.send_command(Calibrate::ALL)?;
        // will busy for 3.5ms

        //self.send_command(SetStandby::XOSC)?;

        // self.send_command(SetModulationParams::LORA_DEFAULT)?;
        self.send_command(SetModulationParams::LORA_KBPS_2_4)?;
        self.send_command(SetLoRaPacketParams::DEFAULT)?;

        pub const LORA_SYNC_WORD: u16 = 0x3444; // pub,  0x1424 private
        self.set_lora_sync_word(LORA_SYNC_WORD)?;

        // power should match
        self.send_command(SetPaConfig::POWER_10DBM)?;
        self.send_command(SetTxParams::new(22, RampTime::_3400U))?;

        Ok(())
    }

    // *****
    pub fn init_as_e22(&mut self, config: Config, delay: &mut impl DelayNs) -> Result<(), spi::Error> {
        let status = self.get_status()?;
        if status.chip_mode() == 0x02 {
            // already in standby RC
        }
        delay.delay_ms(100);

        self.send_command(SetStandby::RC)?;

        if let Some((voltage, delay)) = config.dio3_as_txco_ctrl {
            let error = self.send_command(GetDeviceErrors)?;
            if error & (1 << 5) != 0 {
                hal::println!("!! clear XOSC_START_ERR");
                self.send_command(ClearDeviceErrors)?;
            }
            self.send_command(SetDIO3AsTCXOCtrl::new(voltage, delay))?;

            self.send_command(Calibrate::ALL)?;
            self.send_command(SetStandby::XOSC)?;

            let error = self.send_command(GetDeviceErrors)?;
            println!("!! error: {}", error);
            let status = self.get_status()?;
            println!("!! status: {:?}", status);
        }

        self.send_command(SetStandby::RC)?;

        if config.use_dio2_as_rfswitch {
            hal::println!("DIO2 as TXEN");
            self.send_command(SetDIO2AsRfSwitchCtrl(true))?;
        }

        if config.use_dcdc {
            self.send_command(SetRegulatorMode::DCDC)?;
        }

        self.send_command(SetDioIrqParams::DEFAULT)?;

        self.send_command(SetBufferBaseAddress::default())?;

        // rf part
        let rf_freq = config.freq;
        hal::println!("rf_freq: {}", rf_freq);

        // radio config sequence
        self.send_command(SetPacketType::LORA)?;

        // when tx, callibrate image must after set rf freq
        self.send_command(SetRfFrequency::hz(rf_freq))?;
        self.send_command(CalibrateImage::BAND_470_510)?;

        self.send_command(SetModulationParams::LORA_KBPS_2_4)?;
        self.send_command(SetLoRaPacketParams::DEFAULT)?;

        pub const LORA_SYNC_WORD: u16 = 0x3444; // pub,  0x1424 private
        self.set_lora_sync_word(LORA_SYNC_WORD)?;

        // power should match
        self.send_command(SetPaConfig::POWER_10DBM)?;
        self.send_command(SetTxParams::new(14, RampTime::_800U))?;

        self.send_command(SetRxTxFallbackMode::STDBY_XOSC)?;

        Ok(())
    }

    pub fn set_rf_frequency(&mut self, freq: u32) -> Result<(), spi::Error> {
        //  self.send_command(SetRfFrequency::hz(freq))?;
        //        self.send_command(SetStandby::XOSC)?;
        self.send_command(CalibrateImage::for_freq(freq))?;
        self.send_command(SetRfFrequency::hz(freq))?;
        Ok(())
    }

    pub fn set_fs(&mut self) -> Result<(), spi::Error> {
        self.send_command(SetFs)?;
        Ok(())
    }

    pub fn set_tx_continuous_wave(&mut self) -> Result<(), spi::Error> {
        self.send_command(SetTxContinuousWave)?;
        Ok(())
    }

    pub fn set_tx_infinite_preamble(&mut self) -> Result<(), spi::Error> {
        self.send_command(SetTxInfinitePreamble)?;
        Ok(())
    }

    pub fn rx_bytes(&mut self) -> Result<(), spi::Error> {
        self.send_command(SetStandby::XOSC)?; // ? must
                                              //self.write_register(regs::XTA_TRIM, 0x1C)?;
                                              //self.write_register(regs::XTB_TRIM, 0x1C)?;

        self.send_command(SetDioIrqParams::DEFAULT)?;

        self.send_command(SetLoRaPacketParams::with_payload_len(0xFF))?;
        self.clear_irq_status(0x1FF)?;

        self.send_command(SetRx::SINGLE)?;

        Ok(())
    }

    pub fn tx_bytes(&mut self, data: &[u8]) -> Result<(), spi::Error> {
        self.write_buffer(0x00, data)?;

        self.send_command(SetLoRaPacketParams::with_payload_len(data.len() as u8))?;

        // no timeout
        self.send_command(SetTx::default())?;

        Ok(())
    }

    pub fn get_irq_status(&mut self) -> Result<u16, spi::Error> {
        self.send_command(GetIrqStatus)
    }

    pub fn clear_irq_status(&mut self, mask: u16) -> Result<(), spi::Error> {
        self.send_command(ClearIrqStatus(mask))?;
        Ok(())
    }

    pub fn get_status(&mut self) -> Result<Status, spi::Error> {
        self.send_command(GetStatus)
    }

    pub fn get_stats(&mut self) -> Result<Stats, spi::Error> {
        self.send_command(GetStats)
    }

    pub fn get_rx_buffer_status(&mut self) -> Result<(u8, u8), spi::Error> {
        self.send_command(GetRxBufferStatus)
    }

    pub fn send_command<C: Command>(&mut self, cmd: C) -> Result<C::Response, spi::Error> {
        let mut buf = [0u8; 255];
        let len = cmd.encode(&mut buf);
        let resp_len = C::Response::expected_len();

        //hal::println!("!! in : {:02x?}", &buf[..len]);
        if resp_len != 0 {
            let total_len = len + 1 + C::Response::expected_len();

            let _ = self.cs.set_low();
            self.spi.transfer_in_place(&mut buf[..total_len])?;
            let _ = self.cs.set_high();

            let reply = &buf[len + 1..];
            // hal::println!("!! out: {:02x?}", &buf[..total_len]);
            Ok(C::Response::decode(reply))
        } else {
            let _ = self.cs.set_low();
            self.spi.transfer_in_place(&mut buf[..len])?;
            let _ = self.cs.set_high();
            // hal::println!("!! out: {:02x?}", &buf[..len]);
            Ok(C::Response::decode(&[]))
        }
    }

    pub fn get_lora_sync_word(&mut self) -> Result<u16, spi::Error> {
        let msb = self.read_register(regs::LORA_SYNC_WORD_MSB)?;
        let lsb = self.read_register(regs::LORA_SYNC_WORD_LSB)?;
        Ok((msb as u16) << 8 | lsb as u16)
    }

    pub fn set_lora_sync_word(&mut self, sync_word: u16) -> Result<(), spi::Error> {
        self.write_register(regs::LORA_SYNC_WORD_MSB, (sync_word >> 8) as u8)?;
        self.write_register(regs::LORA_SYNC_WORD_LSB, sync_word as u8)?;

        Ok(())
    }

    pub fn get_rssi_dbm(&mut self) -> Result<i8, spi::Error> {
        // Signal power in dBm = –RssiInst/2 (dBm)
        self.send_command(GetRssiInst).map(|x| x / 2)
    }

    /// 0x18 60mA
    /// 0x38 140mA
    pub fn set_pa_ocp(&mut self, ma: u8) -> Result<(), spi::Error> {
        self.write_register(regs::OCP_CONFIGURATION, ma)?;
        Ok(())
    }

    // helpers
    pub fn write_registers(&mut self, address: u16, data: &[u8]) -> Result<(), spi::Error> {
        self.send_command(WriteRegister { address, data })?;
        Ok(())
    }

    pub fn get_device_errors(&mut self) -> Result<u16, spi::Error> {
        self.send_command(GetDeviceErrors)
    }

    pub fn get_lora_packet_status(&mut self) -> Result<LoRaPacketStatus, spi::Error> {
        self.send_command(GetLoRaPacketStatus)
    }

    pub fn write_register(&mut self, address: u16, data: u8) -> Result<(), spi::Error> {
        self.write_registers(address, &[data])
    }

    pub fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), spi::Error> {
        self.send_command(WriteBuffer { offset, data })?;
        Ok(())
    }

    pub fn read_registers(&mut self, address: u16, data: &mut [u8]) -> Result<(), spi::Error> {
        let mut out = [cmds::READ_REGISTER, (address >> 8) as u8, address as u8, cmds::NOP];

        self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut out[..])?;
        self.spi.blocking_transfer_in_place(data)?;
        self.cs.set_high();

        Ok(())
    }

    pub fn read_register(&mut self, address: u16) -> Result<u8, spi::Error> {
        let mut out = [
            cmds::READ_REGISTER,
            (address >> 8) as u8,
            address as u8,
            cmds::NOP,
            cmds::NOP,
        ];

        self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut out[..])?;
        self.cs.set_high();

        Ok(out[4])
    }

    pub fn read_buffer(&mut self, offset: u8, data: &mut [u8]) -> Result<(), spi::Error> {
        let mut out = [cmds::READ_BUFFER, offset, cmds::NOP];

        let _ = self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut out[..])?;
        self.spi.blocking_transfer_in_place(data)?;
        let _ = self.cs.set_high();

        Ok(())
    }
}
