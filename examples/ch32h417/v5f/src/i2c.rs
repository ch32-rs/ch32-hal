//! Minimal I2C1 master for hart 1, on top of metapac.
//!
//! Just enough for the BME280: blocking write and write-then-read, no DMA and
//! no interrupts. The boot core has already programmed the clock tree; this
//! only turns the peripheral clocks on and reads PCLK1 back from
//! `rcc::clocks()` (filled by `ch32_hal::init_secondary`), so it never touches
//! the PLL.
//!
//! Pins are fixed: PB6 = SCL, PB7 = SDA, AF4 (CH32H4 family data). Both are
//! open-drain alternate function, which is what I2C needs; the bus still wants
//! its external pull-ups.

use ch32_hal::pac::gpio::vals::{Cnf, Mode};
use ch32_hal::pac::{AFIO, GPIOB, I2C1, RCC};

/// 7-bit address of the sensor this master talks to.
pub const BME280_ADDR: u8 = 0x76;

/// PB6/PB7's I2C1 function number (`data/family/CH32H4.yaml`).
const I2C1_AF: u8 = 4;

/// Port B in the AFIO AFR indexing (A = 0).
const PORT_B: usize = 1;

/// Spins spent waiting for one bus flag before giving up. Each spin is a
/// peripheral register read, so this is a few milliseconds at most — long
/// enough for a missing device to NACK, short enough that hart 1 cannot wedge
/// itself on a dead bus.
const SPINS: u32 = 200_000;

#[derive(Debug)]
pub enum Error {
    /// A flag never arrived.
    Timeout,
    /// The addressed device did not ACK.
    Nack,
    /// Another master won the bus, or START did not take.
    Arbitration,
    /// A read was asked for with nowhere to put the bytes.
    Overrun,
}

/// Bring up I2C1 on PB6/PB7 at `freq_hz`.
///
/// # Safety
///
/// The caller must be the only user of I2C1, GPIOB pins 6 and 7, and the AFIO
/// AFR entry for those pins. The boot core must already have configured the
/// clock tree; this reads it and does not change it.
pub unsafe fn init(freq_hz: u32) -> Result<(), Error> {
    // GPIO port B, AFIO (the AFR writes are ignored with its clock off) and
    // I2C1 itself. Reset I2C1 so a previous image's configuration is gone.
    RCC.hb2pcenr().modify(|w| {
        w.set_iopben(true);
        w.set_afioen(true);
    });
    RCC.hb1prstr().modify(|w| w.set_i2c1rst(true));
    RCC.hb1prstr().modify(|w| w.set_i2c1rst(false));
    RCC.hb1pcenr().modify(|w| w.set_i2c1en(true));

    // PB6 and PB7 share the low half of port B. Open-drain alternate function,
    // 50 MHz.
    GPIOB.cfglr().modify(|w| {
        for pin in [6usize, 7] {
            w.set_mode(pin, Mode::OUTPUT_50MHZ);
            w.set_cnf(pin, Cnf::AF_OPEN_DRAIN_OUT);
        }
    });
    // Index = port * 2 + (pin / 8); both pins sit in the low half.
    AFIO.gpio_afr(PORT_B * 2).modify(|w| {
        w.set_afr(6, I2C1_AF);
        w.set_afr(7, I2C1_AF);
    });

    let regs = I2C1;
    regs.ctlr1().modify(|w| w.set_pe(false));
    regs.ctlr1().modify(|w| w.set_swrst(true));
    regs.ctlr1().modify(|w| w.set_swrst(false));

    // I2C1's kernel clock is PCLK1 (HB1), which is HCLK divided by PPRE1.
    let pclk1 = pclk1_hz();
    let freq_range = pclk1 / 1_000_000;
    regs.ctlr2().modify(|w| w.set_freq(freq_range as u8));

    if freq_hz <= 100_000 {
        let ccr = (pclk1 / (freq_hz * 2)).max(4);
        regs.rtr().write(|w| w.set_trise((freq_range + 1) as u8));
        regs.ckcfgr().write(|w| w.set_ccr(ccr as u16));
    } else {
        // Fast mode, duty tlow/thigh = 2.
        let ccr = (pclk1 / (freq_hz * 3)).max(1);
        regs.rtr()
            .write(|w| w.set_trise((freq_range * 300 / 1000 + 1) as u8));
        regs.ckcfgr().write(|w| {
            w.set_f_s(true);
            w.set_duty(false);
            w.set_ccr(ccr as u16);
        });
    }

    regs.ctlr1().modify(|w| w.set_pe(true));
    Ok(())
}

/// PCLK1 as the boot core left it.
///
/// `init_secondary` reads the RCC registers back into `rcc::clocks()`, so this
/// follows whatever preset the boot core programmed instead of assuming one.
fn pclk1_hz() -> u32 {
    ch32_hal::rcc::clocks().pclk1.0
}

/// Write `bytes`, then a repeated START and read into `read`.
pub fn write_read(addr: u8, bytes: &[u8], read: &mut [u8]) -> Result<(), Error> {
    if read.is_empty() {
        return Err(Error::Overrun);
    }
    write_bytes(addr, bytes, false)?;
    read_bytes(addr, read)
}

/// Write `bytes` and end the transaction with STOP.
pub fn write(addr: u8, bytes: &[u8]) -> Result<(), Error> {
    write_bytes(addr, bytes, true)
}

fn write_bytes(addr: u8, bytes: &[u8], stop: bool) -> Result<(), Error> {
    let regs = I2C1;
    regs.ctlr1().modify(|w| w.set_start(true));
    wait(|| regs.star1().read().sb())?;
    if regs.ctlr1().read().start() || !regs.star2().read().msl() {
        return Err(Error::Arbitration);
    }

    regs.datar().write(|w| w.set_datar(addr << 1));
    wait_addr()?;
    let _ = regs.star2().read();

    for byte in bytes {
        wait(|| regs.star1().read().tx_e())?;
        regs.datar().write(|w| w.set_datar(*byte));
        wait(|| regs.star1().read().btf())?;
    }

    if stop {
        regs.ctlr1().modify(|w| w.set_stop(true));
    }
    Ok(())
}

fn read_bytes(addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
    let regs = I2C1;
    let Some((last, buffer)) = buffer.split_last_mut() else {
        return Err(Error::Overrun);
    };

    regs.ctlr1().modify(|w| {
        w.set_start(true);
        w.set_ack(true);
    });
    wait(|| regs.star1().read().sb())?;
    if regs.ctlr1().read().start() || !regs.star2().read().msl() {
        return Err(Error::Arbitration);
    }

    regs.datar().write(|w| w.set_datar((addr << 1) | 1));
    wait_addr()?;
    let _ = regs.star2().read();

    for byte in buffer {
        *byte = recv()?;
    }
    regs.ctlr1().modify(|w| {
        w.set_ack(false);
        w.set_stop(true);
    });
    *last = recv()?;
    Ok(())
}

fn recv() -> Result<u8, Error> {
    wait(|| I2C1.star1().read().rx_ne())?;
    Ok(I2C1.datar().read().datar())
}

/// Wait for the address phase, turning a NACK into [`Error::Nack`] instead of
/// burning the whole spin budget on a flag that will never come.
fn wait_addr() -> Result<(), Error> {
    for _ in 0..SPINS {
        check_error()?;
        if I2C1.star1().read().addr() {
            return Ok(());
        }
    }
    // Release the bus so the next attempt starts clean.
    I2C1.ctlr1().modify(|w| w.set_stop(true));
    Err(Error::Timeout)
}

fn wait(mut ready: impl FnMut() -> bool) -> Result<(), Error> {
    for _ in 0..SPINS {
        check_error()?;
        if ready() {
            return Ok(());
        }
    }
    Err(Error::Timeout)
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::Nack => embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Unknown,
            ),
            Error::Arbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Error::Overrun => embedded_hal::i2c::ErrorKind::Overrun,
            Error::Timeout => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

fn check_error() -> Result<(), Error> {
    let star1 = I2C1.star1().read();
    if star1.af() {
        I2C1.star1().modify(|w| w.set_af(false));
        I2C1.ctlr1().modify(|w| w.set_stop(true));
        return Err(Error::Nack);
    }
    if star1.arlo() {
        I2C1.star1().modify(|w| w.set_arlo(false));
        return Err(Error::Arbitration);
    }
    if star1.berr() {
        I2C1.star1().modify(|w| w.set_berr(false));
    }
    Ok(())
}
