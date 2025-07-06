#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum CanError {
    /// The peripheral receive buffer was overrun.
    Overrun,
    // MAC sublayer errors
    /// A bit error is detected at that bit time when the bit value that is
    /// monitored differs from the bit value sent.
    Bit,
    /// Bit stuffing error - more than 5 equal bits
    Stuff,
    /// The CRC check sum of a received message was incorrect. The CRC of an
    /// incoming message does not match with the CRC calculated from the received data.
    Crc,
    /// Form error - A fixed format part of a received message has wrong format
    Form,
    /// The message transmitted by the FDCAN was not acknowledged by another node.
    Acknowledge,
    ///  CAN is in Bus_Off state.
    BusOff,
    ///  CAN is in the Error_Passive state.
    BusPassive,
    ///  At least one of error counter has reached the Error_Warning limit of 96.
    BusWarning,
    /// Timeout error.
    Timeout,
}

impl core::fmt::Display for CanError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Overrun => write!(f, "The peripheral receive buffer was overrun"),
            Self::Bit => write!(f, "Bit value that is monitored differs from the bit value sent"),
            Self::Stuff => write!(f, "Sixth consecutive equal bits detected"),
            Self::Crc => write!(f, "Calculated CRC sequence does not equal the received one"),
            Self::Form => write!(f, "A fixed-form bit field contains one or more illegal bits"),
            Self::Acknowledge => write!(f, "Transmitted frame was not acknowledged"),
            Self::BusOff => write!(f, "The peripheral is in Bus Off mode"),
            Self::BusPassive => write!(f, "The peripheral is in Bus Passive mode"),
            Self::BusWarning => write!(f, "A peripheral error counter has reached the Warning threshold"),
            Self::Timeout => write!(f, "Operation timed out"),
        }
    }
}

impl Into<embedded_can::ErrorKind> for CanError {
    fn into(self) -> embedded_can::ErrorKind {
        match self {
            Self::Overrun => embedded_can::ErrorKind::Overrun,
            Self::Bit => embedded_can::ErrorKind::Bit,
            Self::Stuff => embedded_can::ErrorKind::Stuff,
            Self::Crc => embedded_can::ErrorKind::Crc,
            Self::Form => embedded_can::ErrorKind::Form,
            Self::Acknowledge => embedded_can::ErrorKind::Acknowledge,
            Self::BusOff => embedded_can::ErrorKind::Other,
            Self::BusPassive => embedded_can::ErrorKind::Other,
            Self::BusWarning => embedded_can::ErrorKind::Other,
            Self::Timeout => embedded_can::ErrorKind::Other,
        }
    }
}

impl embedded_can::Error for CanError {
    fn kind(&self) -> embedded_can::ErrorKind {
        (*self).into()
    }
}

#[derive(PartialEq)]
pub enum CanMode {
    Normal,
    Silent,
    Loopback,
    SilentLoopback,
}

pub(crate) struct CanModeRegs {
    /// Loopback mode setting
    pub(crate) lbkm: bool,
    /// Silent mode setting
    pub(crate) silm: bool,
}

impl CanMode {
    pub(crate) fn regs(&self) -> CanModeRegs {
        match self {
            CanMode::Normal => CanModeRegs {
                lbkm: false,
                silm: false,
            },
            CanMode::Silent => CanModeRegs {
                lbkm: false,
                silm: true,
            },
            CanMode::Loopback => CanModeRegs {
                lbkm: true,
                silm: false,
            },
            CanMode::SilentLoopback => CanModeRegs { lbkm: true, silm: true },
        }
    }
}

#[derive(Copy, Clone)]
pub enum CanFifo {
    Fifo0,
    Fifo1,
}

impl CanFifo {
    pub(crate) fn val(&self) -> usize {
        match self {
            CanFifo::Fifo0 => 0,
            CanFifo::Fifo1 => 1,
        }
    }

    pub(crate) fn val_bool(&self) -> bool {
        match self {
            CanFifo::Fifo0 => false,
            CanFifo::Fifo1 => true,
        }
    }
}

#[derive(Debug)]
pub enum TxStatus {
    /// Message was sent correctly
    Sent,
    /// Message wasn't sent correctly due to send timeout
    TimeoutError,
    /// Message wasn't sent correctly due to arbitration
    ArbitrationError,
    /// Message wasn't sent correctly due to error
    OtherError,
}
