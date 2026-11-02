use embedded_hal::i2c::Operation;

use super::{Error, Target};

const CFNF: u32 = 1 << 2;
const TXFNF: u32 = 1 << 4;
const RXFNE: u32 = 1 << 5;
const FC: u32 = 1 << 9;
const RXEND: u32 = 1 << 10;
const ERR: u32 = 1 << 11;

pub(super) fn validate_dynamic_address(address: u8) -> Result<(), Error> {
    if !(8..=0x77).contains(&address) || matches!(address, 0x3e | 0x5e | 0x6e | 0x76) {
        Err(Error::InvalidAddress)
    } else {
        Ok(())
    }
}

pub(super) fn hardware_error(status: u32) -> Error {
    if status & (1 << 8) != 0 {
        Error::AddressNack
    } else if status & (1 << 9) != 0 {
        Error::DataNack
    } else if status & ((1 << 6) | (1 << 7)) != 0 {
        Error::Fifo
    } else if status & (1 << 4) != 0 {
        Error::Protocol
    } else if status & (1 << 10) != 0 {
        Error::Data
    } else {
        Error::Controller((status & 15) as u8)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum Progress {
    Advanced,
    Waiting,
    Complete,
}

pub(super) trait Engine {
    fn address(&self) -> u8;
    fn legacy(&self) -> bool;
    fn interrupt_mask(&mut self) -> u32;
    fn step(&mut self, regs: crate::pac::i3c::I3c) -> Result<Progress, Error>;
}

pub(super) struct Frame<'a, 'b> {
    pub address: u8,
    pub legacy: bool,
    ccc: Option<u8>,
    direct: bool,
    prefix_pending: bool,
    operations: &'a mut [Operation<'b>],
    command: usize,
    tx_operation: usize,
    tx_offset: usize,
    rx_operation: usize,
    rx_offset: usize,
}

#[cfg(feature = "embassy")]
pub(super) struct DmaFrame<'a> {
    pub address: u8,
    pub legacy: bool,
    pub controls: [u32; 2],
    pub count: usize,
    pub tx: Option<&'a [u8]>,
    pub rx: Option<&'a mut [u8]>,
}

impl<'a, 'b> Frame<'a, 'b> {
    pub fn new(address: u8, operations: &'a mut [Operation<'b>], legacy: bool) -> Result<Self, Error> {
        if legacy {
            if address > 0x7f {
                return Err(Error::InvalidAddress);
            }
        } else {
            validate_dynamic_address(address)?;
        }
        let mut group_length = 0usize;
        let mut previous = None;
        for operation in operations.iter() {
            let (read, length) = match operation {
                Operation::Read(buffer) => (true, buffer.len()),
                Operation::Write(buffer) => (false, buffer.len()),
            };
            if length == 0 {
                return Err(Error::ZeroLength);
            }
            if previous != Some(read) {
                if legacy && previous.is_some() {
                    return Err(Error::UnsupportedTransaction);
                }
                group_length = 0;
            }
            group_length = group_length.checked_add(length).ok_or(Error::TooLong)?;
            if group_length > u16::MAX as usize {
                return Err(Error::TooLong);
            }
            previous = Some(read);
        }
        Ok(Self {
            address,
            legacy,
            ccc: None,
            direct: false,
            prefix_pending: false,
            operations,
            command: 0,
            tx_operation: 0,
            tx_offset: 0,
            rx_operation: 0,
            rx_offset: 0,
        })
    }

    pub fn is_empty(&self) -> bool {
        self.operations.is_empty()
    }

    #[cfg(feature = "embassy")]
    pub fn into_dma(mut self) -> Result<DmaFrame<'a>, Self> {
        if self.ccc.is_some()
            || self.operations.len() > 2
            || (self.operations.len() == 2
                && matches!(self.operations[0], Operation::Read(_)) == matches!(self.operations[1], Operation::Read(_)))
        {
            return Err(self);
        }
        let count = self.operations.len();
        let mut controls = [0; 2];
        for control in &mut controls[..count] {
            *control = self.next_command();
        }
        let mut tx = None;
        let mut rx = None;
        for operation in self.operations {
            match operation {
                Operation::Write(buffer) => tx = Some(&**buffer),
                Operation::Read(buffer) => rx = Some(&mut **buffer),
            }
        }
        Ok(DmaFrame {
            address: self.address,
            legacy: self.legacy,
            controls,
            count,
            tx,
            rx,
        })
    }

    pub fn broadcast(command: u8, operations: &'a mut [Operation<'b>; 1]) -> Result<Self, Error> {
        if command >= 0x80 || matches!(command, 0x07 | 0x2a) {
            return Err(Error::UnsupportedCcc);
        }
        let Operation::Write(payload) = &operations[0] else {
            return Err(Error::UnsupportedCcc);
        };
        if payload.len() > u16::MAX as usize {
            return Err(Error::TooLong);
        }
        Ok(Self {
            address: 0x7e,
            legacy: false,
            ccc: Some(command),
            direct: false,
            prefix_pending: false,
            operations,
            command: 0,
            tx_operation: 0,
            tx_offset: 0,
            rx_operation: 0,
            rx_offset: 0,
        })
    }

    pub fn direct(command: u8, address: u8, operations: &'a mut [Operation<'b>; 1]) -> Result<Self, Error> {
        if command < 0x80 || command == 0x9a {
            return Err(Error::UnsupportedCcc);
        }
        let mut frame = Self::new(address, operations, false)?;
        frame.ccc = Some(command);
        frame.direct = true;
        frame.prefix_pending = true;
        Ok(frame)
    }

    fn next_command(&mut self) -> u32 {
        if let Some(command) = self.ccc {
            if self.prefix_pending {
                self.prefix_pending = false;
                return (u32::from(command) << 16) | (6 << 27);
            }
            if !self.direct {
                self.command = 1;
                let Operation::Write(payload) = &self.operations[0] else {
                    unreachable!()
                };
                return payload.len() as u32 | (u32::from(command) << 16) | (6 << 27) | (1 << 31);
            }
        }
        let read = matches!(self.operations[self.command], Operation::Read(_));
        let mut length = 0usize;
        while let Some(operation) = self.operations.get(self.command) {
            if matches!(operation, Operation::Read(_)) != read {
                break;
            }
            length += match operation {
                Operation::Read(buffer) => buffer.len(),
                Operation::Write(buffer) => buffer.len(),
            };
            self.command += 1;
        }
        length as u32
            | (u32::from(read) << 16)
            | (u32::from(self.address) << 17)
            | ((if self.legacy {
                4
            } else if self.direct {
                3
            } else {
                2
            }) << 27)
            | (u32::from(self.command == self.operations.len()) << 31)
    }

    fn skip_tx(&mut self) {
        while let Some(operation) = self.operations.get(self.tx_operation) {
            if let Operation::Write(buffer) = operation {
                if self.tx_offset < buffer.len() {
                    break;
                }
            }
            self.tx_operation += 1;
            self.tx_offset = 0;
        }
    }

    fn skip_rx(&mut self) {
        while let Some(operation) = self.operations.get(self.rx_operation) {
            if let Operation::Read(buffer) = operation {
                if self.rx_offset < buffer.len() {
                    break;
                }
            }
            self.rx_operation += 1;
            self.rx_offset = 0;
        }
    }
}

impl Engine for Frame<'_, '_> {
    fn address(&self) -> u8 {
        self.address
    }

    fn legacy(&self) -> bool {
        self.legacy
    }

    fn interrupt_mask(&mut self) -> u32 {
        self.skip_tx();
        self.skip_rx();
        FC | RXEND
            | ERR
            | if self.command < self.operations.len() { CFNF } else { 0 }
            | if self.tx_operation < self.operations.len() {
                TXFNF
            } else {
                0
            }
            | if self.rx_operation < self.operations.len() {
                RXFNE
            } else {
                0
            }
    }

    fn step(&mut self, regs: crate::pac::i3c::I3c) -> Result<Progress, Error> {
        let events = regs.evr().read().0;
        if events & ERR != 0 {
            return Err(hardware_error(regs.stater().read().0));
        }
        if events & RXEND != 0 {
            return Err(Error::ShortRead);
        }
        self.skip_tx();
        self.skip_rx();
        if events & RXFNE != 0 {
            let Some(Operation::Read(buffer)) = self.operations.get_mut(self.rx_operation) else {
                return Err(Error::Fifo);
            };
            buffer[self.rx_offset] = regs.rdr().read().0 as u8;
            self.rx_offset += 1;
            return Ok(Progress::Advanced);
        }
        if events & FC != 0 {
            if self.command != self.operations.len()
                || self.tx_operation != self.operations.len()
                || self.rx_operation != self.operations.len()
            {
                return Err(Error::ShortRead);
            }
            return Ok(Progress::Complete);
        }
        if events & CFNF != 0 && self.command < self.operations.len() {
            regs.ctlr().write(|reg| reg.0 = self.next_command());
            return Ok(Progress::Advanced);
        }
        if events & TXFNF != 0 {
            if let Some(Operation::Write(buffer)) = self.operations.get(self.tx_operation) {
                regs.tdr().write(|reg| reg.0 = u32::from(buffer[self.tx_offset]));
                self.tx_offset += 1;
                return Ok(Progress::Advanced);
            }
        }
        Ok(Progress::Waiting)
    }
}

pub(super) struct Daa<'a> {
    addresses: &'a [u8],
    targets: &'a mut [Target],
    assigned: usize,
    received: usize,
    identity: [u8; 8],
    command_sent: bool,
}

impl<'a> Daa<'a> {
    pub fn new(addresses: &'a [u8], targets: &'a mut [Target], controller_address: u8) -> Result<Self, Error> {
        if addresses.is_empty() || addresses.len() > targets.len() {
            return Err(Error::Capacity);
        }
        for (index, address) in addresses.iter().copied().enumerate() {
            validate_dynamic_address(address)?;
            if address == controller_address || addresses[..index].contains(&address) {
                return Err(Error::InvalidAddress);
            }
        }
        Ok(Self {
            addresses,
            targets,
            assigned: 0,
            received: 0,
            identity: [0; 8],
            command_sent: false,
        })
    }

    pub fn assigned(&self) -> usize {
        self.assigned
    }
}

impl Engine for Daa<'_> {
    fn address(&self) -> u8 {
        0x7e
    }

    fn legacy(&self) -> bool {
        false
    }

    fn interrupt_mask(&mut self) -> u32 {
        ERR | FC
            | RXFNE
            | RXEND
            | if self.received == 8 { TXFNF } else { 0 }
            | if !self.command_sent { CFNF } else { 0 }
    }

    fn step(&mut self, regs: crate::pac::i3c::I3c) -> Result<Progress, Error> {
        let events = regs.evr().read().0;
        if events & ERR != 0 {
            return Err(hardware_error(regs.stater().read().0));
        }
        if events & RXEND != 0 {
            return Err(Error::Protocol);
        }
        if events & RXFNE != 0 {
            if self.received == 8 {
                return Err(Error::Fifo);
            }
            self.identity[self.received] = regs.rdr().read().0 as u8;
            self.received += 1;
            return Ok(Progress::Advanced);
        }
        if events & FC != 0 {
            if !self.command_sent || self.received != 0 || regs.statr().read().0 & 0xffff != self.assigned as u32 {
                return Err(Error::Protocol);
            }
            return Ok(Progress::Complete);
        }
        if !self.command_sent && events & CFNF != 0 {
            regs.ctlr_ccc()
                .write(|reg| reg.0 = (1 << 31) | (6 << 27) | (0x07 << 16));
            self.command_sent = true;
            return Ok(Progress::Advanced);
        }
        if self.received == 8 && events & TXFNF != 0 {
            let Some(address) = self.addresses.get(self.assigned).copied() else {
                return Err(Error::Capacity);
            };
            regs.tdr().write(|reg| reg.0 = u32::from(address));
            self.targets[self.assigned] = Target {
                dynamic_address: address,
                pid: self.identity[..6].try_into().unwrap(),
                bcr: self.identity[6],
                dcr: self.identity[7],
            };
            self.assigned += 1;
            self.received = 0;
            return Ok(Progress::Advanced);
        }
        Ok(Progress::Waiting)
    }
}
