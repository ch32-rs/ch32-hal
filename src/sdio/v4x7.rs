//! SD memory cards over the CH32V407/CH32V467 SDIO host.
//!
//! Supports 1/4-bit SDSC, SDHC and SDXC cards, SDR12/SDR25, single/multiple
//! blocks and DMA. Blocking operations have bounded polling. Embassy operations
//! additionally use a software timeout. An interrupted or failed data transfer
//! invalidates the card: call `init_card` again before reuse. The controller has
//! no RCC reset bit; cleanup explicitly stops both state machines and flushes
//! the FIFO. eMMC, SDIO functions and 1.8 V signalling are not implemented.

#[cfg(feature = "embassy")]
use core::future::{poll_fn, Future};
use core::marker::PhantomData;
#[cfg(feature = "embassy")]
use core::pin::Pin;
#[cfg(feature = "embassy")]
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;
use sdio_host::{CardCapacity, CardStatus, SDStatus, SCR};

use crate::dma::{Transfer, TransferOptions};
use crate::gpio::{AFType, AnyPin, SealedPin, Speed};
use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::time::Hertz;
use crate::{interrupt, pac, peripherals, Peri, RccPeripheral, RemapPeripheral};

#[path = "v4x7_types.rs"]
mod types;
use types::{
    block_address, block_count, clock_setting, data_error, response_value, sd_status, status_bytes, transfer_rate,
    Clock,
};
pub use types::{Card, Config, DataBlock, Error, Signalling};

const CMD_FLAGS: u32 = 0xc5;
const DATA_FLAGS: u32 = 0x73a;
const DATA_INTERRUPTS: u32 = 0x33a;
const ALL_FLAGS: u32 = 0x00c0_07ff;
const R1_ERRORS: u32 = 0xfdff_e008;
const DMA_OPTIONS: TransferOptions = TransferOptions {
    priority: crate::dma::Priority::VeryHigh,
    circular: false,
    half_transfer_ir: false,
    complete_transfer_ir: true,
    peripheral_increment: false,
    memory_to_memory: false,
};

#[derive(Clone, Copy, Eq, PartialEq)]
enum Response {
    None,
    R1,
    R2,
    R3,
    R6,
    R7,
}

pub struct InterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::REGS.mask().write_value(pac::sdio::regs::Mask(0));
        T::waker().wake();
    }
}

fn reset_controller<T: Instance>() {
    T::REGS.mask().write_value(pac::sdio::regs::Mask(0));
    T::REGS.dctrl2().write_value(pac::sdio::regs::Dctrl2(0));
    T::REGS.dctrl().write_value(pac::sdio::regs::Dctrl(0));
    T::REGS.cmd().write_value(pac::sdio::regs::Cmd(0));
    T::REGS.power().write_value(pac::sdio::regs::Power(0));
    T::REGS.clkcr().write_value(pac::sdio::regs::Clkcr(0));
    T::REGS.arg().write_value(pac::sdio::regs::Arg(0));
    T::REGS.dtimer().write_value(pac::sdio::regs::Dtimer(0));
    T::REGS.dlen().write_value(pac::sdio::regs::Dlen(0));
    T::REGS.icr().write_value(pac::sdio::regs::Icr(ALL_FLAGS));
    T::Interrupt::unpend();
}

fn stop_data<T: Instance>() {
    T::REGS.mask().write_value(pac::sdio::regs::Mask(0));
    T::REGS.dctrl().write_value(pac::sdio::regs::Dctrl(0));
    T::REGS.icr().write_value(pac::sdio::regs::Icr(DATA_FLAGS));
}

fn wait_idle<T: Instance>(limit: u32, data: bool) -> Result<(), Error> {
    let mask = if data { 0x3800 } else { 0x800 };
    for _ in 0..limit {
        if T::REGS.sta().read().0 & mask == 0 {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(Error::SoftwareTimeout)
}

fn clock_delay<T: Instance>(clock: Hertz, cycles: u32) {
    let iterations = u64::from(T::frequency().0).div_ceil(u64::from(clock.0)) * u64::from(cycles);
    for _ in 0..iterations {
        core::hint::spin_loop();
    }
}

fn retry_delay<T: Instance>() {
    for _ in 0..T::frequency().0.div_ceil(1_000) {
        core::hint::spin_loop();
    }
}

fn configure_clock<T: Instance>(clock: Clock, wide: bool, config: Config) -> Result<(), Error> {
    wait_idle::<T>(config.poll_limit, true)?;
    for _ in 0..7 {
        core::hint::spin_loop();
    }
    T::REGS.clkcr().write(|register| {
        register.set_clkdiv(clock.divisor);
        register.set_bypass(clock.bypass);
        register.set_hwfc_en(clock.bypass);
        register.set_widbus(if wide { 1 } else { 0 });
        register.set_clken(true);
    });
    Ok(())
}

fn command<T: Instance>(index: u8, argument: u32, response: Response, config: Config) -> Result<[u32; 4], Error> {
    wait_idle::<T>(config.poll_limit, false)?;
    T::REGS.icr().write_value(pac::sdio::regs::Icr(CMD_FLAGS));
    T::REGS.arg().write_value(pac::sdio::regs::Arg(argument));
    T::REGS.cmd().write(|register| {
        register.set_cmdindex(index);
        register.set_waitresp(match response {
            Response::None => 0,
            Response::R2 => 3,
            _ => 1,
        });
        register.set_cpsmen(true);
    });
    let completion = if response == Response::None { 0x80 } else { 0x41 };
    for _ in 0..config.poll_limit {
        let status = T::REGS.sta().read().0;
        if status & 4 != 0 {
            return Err(Error::Timeout);
        }
        if status & completion == 0 {
            core::hint::spin_loop();
            continue;
        }
        if response == Response::None {
            return Ok([0; 4]);
        }
        if status & 1 != 0 && response != Response::R3 {
            return Err(Error::Crc);
        }
        if !matches!(response, Response::R2 | Response::R3) && T::REGS.respcmd().read().respcmd() != index {
            return Err(Error::ResponseMismatch);
        }
        let words = core::array::from_fn(|word| T::REGS.resp(word).read().0);
        if response == Response::R1 {
            if words[0] & R1_ERRORS != 0 {
                return Err(Error::CardStatus(words[0]));
            }
            if words[0] & 0x0200_0000 != 0 {
                return Err(Error::CardLocked);
            }
        }
        if response == Response::R6 && words[0] & 0xe000 != 0 {
            return Err(Error::CardStatus(words[0]));
        }
        return Ok(words);
    }
    Err(Error::SoftwareTimeout)
}

fn app_command<T: Instance>(rca: u16, config: Config) -> Result<(), Error> {
    let response = command::<T>(55, u32::from(rca) << 16, Response::R1, config)?;
    if response[0] & 0x20 == 0 {
        return Err(Error::UnsupportedCardType);
    }
    Ok(())
}

fn read_status<T: Instance>(rca: u16, config: Config) -> Result<CardStatus, Error> {
    Ok(command::<T>(13, u32::from(rca) << 16, Response::R1, config)?[0].into())
}

fn ready(status: CardStatus) -> bool {
    status.ready_for_data() && status.state() == sdio_host::CurrentState::Transfer
}

fn blocking_wait_ready<T: Instance>(rca: u16, config: Config) -> Result<(), Error> {
    for _ in 0..config.card_ready_attempts {
        if ready(read_status::<T>(rca, config)?) {
            return Ok(());
        }
        retry_delay::<T>();
    }
    Err(Error::SoftwareTimeout)
}

#[cfg(feature = "embassy")]
async fn wait_ready<T: Instance>(rca: u16, config: Config) -> Result<(), Error> {
    for _ in 0..config.card_ready_attempts {
        if ready(read_status::<T>(rca, config)?) {
            return Ok(());
        }
        embassy_time::Timer::after_millis(1).await;
    }
    Err(Error::SoftwareTimeout)
}

enum Buffer<'a> {
    Read(&'a mut [u32]),
    Write(&'a [u32]),
}

fn start_data<'a, T: Instance, D: SdioDma<T>>(
    dma: Peri<'a, D>,
    buffer: Buffer<'a>,
    index: u8,
    argument: u32,
    block_size: u8,
    config: Config,
) -> Result<Transfer<'a>, Error> {
    let (read, length) = match &buffer {
        Buffer::Read(words) => (true, words.len()),
        Buffer::Write(words) => (false, words.len()),
    };
    if length == 0 || length > u16::MAX as usize || block_size > 14 || length * 4 % (1 << block_size) != 0 {
        return Err(Error::InvalidBufferLength);
    }
    wait_idle::<T>(config.poll_limit, true)?;
    T::REGS.dctrl2().write_value(pac::sdio::regs::Dctrl2(0));
    stop_data::<T>();
    T::REGS
        .dtimer()
        .write_value(pac::sdio::regs::Dtimer(config.data_transfer_timeout));
    T::REGS.dlen().write_value(pac::sdio::regs::Dlen((length * 4) as u32));
    let request = dma.request();
    let transfer = unsafe {
        match buffer {
            Buffer::Read(words) => {
                Transfer::new_read_checked(dma, request, T::REGS.fifo().as_ptr().cast(), words, DMA_OPTIONS)
            }
            Buffer::Write(words) => {
                Transfer::new_write_checked(dma, request, words, T::REGS.fifo().as_ptr().cast(), DMA_OPTIONS)
            }
        }
    };
    let arm = || {
        T::REGS.dctrl().write(|register| {
            register.set_dblocksize(block_size);
            register.set_dtdir(read);
            register.set_dmaen(true);
            register.set_dten(true);
        })
    };
    if read {
        arm();
    }
    command::<T>(index, argument, Response::R1, config)?;
    if !read {
        arm();
    }
    Ok(transfer)
}

fn data_complete<T: Instance>(transfer: &mut Transfer<'_>) -> Result<bool, Error> {
    let status = T::REGS.sta().read().0;
    data_error(status)?;
    if transfer.has_error() {
        return Err(Error::Dma);
    }
    if transfer.is_running() {
        return Ok(false);
    }
    let status = T::REGS.sta().read().0;
    data_error(status)?;
    if transfer.has_error() {
        return Err(Error::Dma);
    }
    if status & 0x100 == 0 || status & 0x3000 != 0 {
        return Ok(false);
    }
    if transfer.get_remaining_transfers() != 0 || T::REGS.dcount().read().0 != 0 {
        return Err(Error::IncompleteTransfer);
    }
    Ok(true)
}

#[cfg(feature = "embassy")]
fn poll_data<T: Instance>(transfer: &mut Transfer<'_>, context: &mut Context<'_>) -> Poll<Result<(), Error>> {
    T::waker().register(context.waker());
    let _ = Pin::new(&mut *transfer).poll(context);
    match data_complete::<T>(transfer) {
        Ok(true) => Poll::Ready(Ok(())),
        Err(error) => Poll::Ready(Err(error)),
        Ok(false) => {
            let finished = T::REGS.sta().read().0 & 0x100 != 0;
            T::REGS.mask().write_value(pac::sdio::regs::Mask(if finished {
                DATA_INTERRUPTS & !0x100
            } else {
                DATA_INTERRUPTS
            }));
            if finished {
                context.waker().wake_by_ref();
            }
            Poll::Pending
        }
    }
}

pub struct Sdmmc<'d, T: Instance, D: SdioDma<T>> {
    _peri: Peri<'d, T>,
    dma: Peri<'d, D>,
    pins: [Option<Peri<'d, AnyPin>>; 6],
    config: Config,
    clock: Hertz,
    signalling: Signalling,
    card: Option<Card>,
}

impl<'d, T: Instance, D: SdioDma<T>> Sdmmc<'d, T, D> {
    pub fn new_1bit<const REMAP: u8>(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dma: Peri<'d, D>,
        clk: Peri<'d, impl CkPin<T>>,
        cmd: Peri<'d, impl CmdPin<T>>,
        d0: Peri<'d, impl D0Pin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, Error> {
        Self::new_inner(
            peri,
            dma,
            [Some(clk.into()), Some(cmd.into()), Some(d0.into()), None, None, None],
            REMAP,
            config,
        )
    }

    pub fn new_4bit<const REMAP: u8>(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dma: Peri<'d, D>,
        clk: Peri<'d, impl CkPin<T>>,
        cmd: Peri<'d, impl CmdPin<T>>,
        d0: Peri<'d, impl D0Pin<T, REMAP>>,
        d1: Peri<'d, impl D1Pin<T, REMAP>>,
        d2: Peri<'d, impl D2Pin<T, REMAP>>,
        d3: Peri<'d, impl D3Pin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, Error> {
        Self::new_inner(
            peri,
            dma,
            [
                Some(clk.into()),
                Some(cmd.into()),
                Some(d0.into()),
                Some(d1.into()),
                Some(d2.into()),
                Some(d3.into()),
            ],
            REMAP,
            config,
        )
    }

    fn new_inner(
        peri: Peri<'d, T>,
        dma: Peri<'d, D>,
        pins: [Option<Peri<'d, AnyPin>>; 6],
        remap: u8,
        config: Config,
    ) -> Result<Self, Error> {
        if config.poll_limit == 0
            || config.card_ready_attempts == 0
            || config.data_transfer_timeout == 0
            || config.software_timeout_ms == 0
        {
            return Err(Error::InvalidConfig);
        }
        let initial = clock_setting(T::frequency(), 400_000, false)?;
        critical_section::with(|_| {
            T::enable_and_reset();
            reset_controller::<T>();
            T::set_remap(remap);
            for pin in pins.iter().flatten() {
                pin.set_as_af_output(AFType::OutputPushPull, Speed::High);
            }
            unsafe { T::Interrupt::enable() };
        });
        Ok(Self {
            _peri: peri,
            dma,
            pins,
            config,
            clock: initial.frequency,
            signalling: Signalling::SDR12,
            card: None,
        })
    }

    fn init_clock(requested: Hertz) -> Result<Clock, Error> {
        if requested.0 == 0 || requested.0 > 50_000_000 {
            return Err(Error::BadClock);
        }
        clock_setting(T::frequency(), 400_000, false)
    }

    fn begin_init(&mut self, initial: Clock) -> Result<bool, Error> {
        self.card = None;
        self.signalling = Signalling::SDR12;
        self.clock = initial.frequency;
        reset_controller::<T>();
        configure_clock::<T>(initial, false, self.config)?;
        T::REGS.power().write_value(pac::sdio::regs::Power(3));
        clock_delay::<T>(initial.frequency, 80);
        command::<T>(0, 0, Response::None, self.config)?;
        match command::<T>(8, 0x1aa, Response::R7, self.config) {
            Ok(words) if words[0] == 0x1aa => Ok(true),
            Ok(words) if words[0] & R1_ERRORS == 0x0040_0000 => Ok(false),
            Err(Error::Timeout) => Ok(false),
            Ok(_) => Err(Error::UnsupportedCardVersion),
            Err(error) => Err(error),
        }
    }

    fn power_up(&self, version2: bool) -> Result<Option<u32>, Error> {
        app_command::<T>(0, self.config)?;
        let response = command::<T>(
            41,
            0x0030_0000 | if version2 { 1 << 30 } else { 0 },
            Response::R3,
            self.config,
        )?[0];
        if response & 0x0030_0000 == 0 {
            return Err(Error::UnsupportedCardType);
        }
        Ok((response & (1 << 31) != 0).then_some(response))
    }

    fn identify(&self, ocr: u32) -> Result<Card, Error> {
        let cid = response_value(command::<T>(2, 0, Response::R2, self.config)?).into();
        let rca = (command::<T>(3, 0, Response::R6, self.config)?[0] >> 16) as u16;
        if rca == 0 {
            return Err(Error::ResponseMismatch);
        }
        let csd_raw = response_value(command::<T>(9, u32::from(rca) << 16, Response::R2, self.config)?);
        let high_capacity = ocr & (1 << 30) != 0;
        if high_capacity != (csd_raw >> 126 == 1) {
            return Err(Error::UnsupportedCardType);
        }
        let block_count = block_count(csd_raw)?;
        command::<T>(7, u32::from(rca) << 16, Response::R1, self.config)?;
        Ok(Card {
            card_type: if high_capacity {
                CardCapacity::SDHC
            } else {
                CardCapacity::SDSC
            },
            ocr: ocr.into(),
            rca,
            cid,
            csd: csd_raw.into(),
            scr: SCR(0),
            status: SDStatus::default(),
            block_count,
        })
    }

    fn set_width(&self, card: &Card) -> Result<bool, Error> {
        let wide = self.pins[5].is_some();
        if !card.scr.bus_width_one() || (wide && !card.scr.bus_width_four()) {
            return Err(Error::UnsupportedBusWidth);
        }
        app_command::<T>(card.rca, self.config)?;
        command::<T>(6, if wide { 2 } else { 0 }, Response::R1, self.config)?;
        Ok(wide)
    }

    fn set_clock(&mut self, requested: u32, wide: bool) -> Result<(), Error> {
        let clock = clock_setting(T::frequency(), requested, self.config.allow_clock_bypass)?;
        configure_clock::<T>(clock, wide, self.config)?;
        self.clock = clock.frequency;
        Ok(())
    }

    fn blocking_data(
        &mut self,
        buffer: Buffer<'_>,
        index: u8,
        argument: u32,
        block_size: u8,
        rca: Option<u16>,
    ) -> Result<(), Error> {
        let card = &mut self.card;
        let guard = OnDrop::new(|| {
            *card = None;
            reset_controller::<T>();
        });
        let mut transfer = start_data::<T, D>(self.dma.reborrow(), buffer, index, argument, block_size, self.config)?;
        let mut completed = false;
        for _ in 0..self.config.poll_limit {
            if data_complete::<T>(&mut transfer)? {
                completed = true;
                break;
            }
            core::hint::spin_loop();
        }
        if !completed {
            return Err(Error::SoftwareTimeout);
        }
        drop(transfer);
        stop_data::<T>();
        if index == 18 || index == 25 {
            command::<T>(12, 0, Response::R1, self.config)?;
        }
        if let Some(rca) = rca {
            blocking_wait_ready::<T>(rca, self.config)?;
        }
        guard.defuse();
        Ok(())
    }

    #[cfg(feature = "embassy")]
    async fn data(
        &mut self,
        buffer: Buffer<'_>,
        index: u8,
        argument: u32,
        block_size: u8,
        rca: Option<u16>,
    ) -> Result<(), Error> {
        let card = &mut self.card;
        let guard = OnDrop::new(|| {
            *card = None;
            reset_controller::<T>();
        });
        let mut transfer = start_data::<T, D>(self.dma.reborrow(), buffer, index, argument, block_size, self.config)?;
        embassy_time::with_timeout(
            embassy_time::Duration::from_millis(u64::from(self.config.software_timeout_ms)),
            poll_fn(|context| poll_data::<T>(&mut transfer, context)),
        )
        .await
        .map_err(|_| Error::SoftwareTimeout)??;
        drop(transfer);
        stop_data::<T>();
        if index == 18 || index == 25 {
            command::<T>(12, 0, Response::R1, self.config)?;
        }
        if let Some(rca) = rca {
            wait_ready::<T>(rca, self.config).await?;
        }
        guard.defuse();
        Ok(())
    }

    pub fn blocking_init_card(&mut self, frequency: Hertz) -> Result<(), Error> {
        let initial = Self::init_clock(frequency)?;
        let guard = OnDrop::new(reset_controller::<T>);
        let version2 = self.begin_init(initial)?;
        let mut ocr = None;
        for _ in 0..self.config.card_ready_attempts {
            ocr = self.power_up(version2)?;
            if ocr.is_some() {
                break;
            }
            retry_delay::<T>();
        }
        let mut card = self.identify(ocr.ok_or(Error::SoftwareTimeout)?)?;
        blocking_wait_ready::<T>(card.rca, self.config)?;
        if matches!(card.card_type, CardCapacity::SDSC) {
            command::<T>(16, 512, Response::R1, self.config)?;
        }
        app_command::<T>(card.rca, self.config)?;
        let mut scr = [0u32; 2];
        self.blocking_data(Buffer::Read(&mut scr), 51, 0, 3, Some(card.rca))?;
        card.scr = SCR((u64::from(u32::from_be(scr[0])) << 32) | u64::from(u32::from_be(scr[1])));
        let wide = self.set_width(&card)?;
        self.set_clock(
            frequency
                .0
                .min(25_000_000)
                .min(transfer_rate(card.csd.transfer_rate())?),
            wide,
        )?;
        if frequency.0 > 25_000_000 && (card.scr.0 >> 56) & 15 != 0 {
            let mut status = [0; 16];
            self.blocking_data(Buffer::Read(&mut status), 6, 0x00ff_fff1, 6, Some(card.rca))?;
            let bytes = status_bytes(&status);
            if bytes[13] & 2 != 0 && bytes[29] & 2 == 0 {
                self.blocking_data(Buffer::Read(&mut status), 6, 0x80ff_fff1, 6, Some(card.rca))?;
                if status_bytes(&status)[16] & 15 != 1 {
                    return Err(Error::SignalingSwitchFailed);
                }
                clock_delay::<T>(self.clock, 8);
                self.set_clock(frequency.0.min(50_000_000), wide)?;
                self.signalling = Signalling::SDR25;
            }
        }
        self.card = Some(card);
        guard.defuse();
        Ok(())
    }

    #[cfg(feature = "embassy")]
    pub async fn init_card(&mut self, frequency: Hertz) -> Result<(), Error> {
        let initial = Self::init_clock(frequency)?;
        let guard = OnDrop::new(reset_controller::<T>);
        let version2 = self.begin_init(initial)?;
        let mut ocr = None;
        for _ in 0..self.config.card_ready_attempts {
            ocr = self.power_up(version2)?;
            if ocr.is_some() {
                break;
            }
            embassy_time::Timer::after_millis(1).await;
        }
        let mut card = self.identify(ocr.ok_or(Error::SoftwareTimeout)?)?;
        wait_ready::<T>(card.rca, self.config).await?;
        if matches!(card.card_type, CardCapacity::SDSC) {
            command::<T>(16, 512, Response::R1, self.config)?;
        }
        app_command::<T>(card.rca, self.config)?;
        let mut scr = [0u32; 2];
        self.data(Buffer::Read(&mut scr), 51, 0, 3, Some(card.rca)).await?;
        card.scr = SCR((u64::from(u32::from_be(scr[0])) << 32) | u64::from(u32::from_be(scr[1])));
        let wide = self.set_width(&card)?;
        self.set_clock(
            frequency
                .0
                .min(25_000_000)
                .min(transfer_rate(card.csd.transfer_rate())?),
            wide,
        )?;
        if frequency.0 > 25_000_000 && (card.scr.0 >> 56) & 15 != 0 {
            let mut status = [0; 16];
            self.data(Buffer::Read(&mut status), 6, 0x00ff_fff1, 6, Some(card.rca))
                .await?;
            let bytes = status_bytes(&status);
            if bytes[13] & 2 != 0 && bytes[29] & 2 == 0 {
                self.data(Buffer::Read(&mut status), 6, 0x80ff_fff1, 6, Some(card.rca))
                    .await?;
                if status_bytes(&status)[16] & 15 != 1 {
                    return Err(Error::SignalingSwitchFailed);
                }
                clock_delay::<T>(self.clock, 8);
                self.set_clock(frequency.0.min(50_000_000), wide)?;
                self.signalling = Signalling::SDR25;
            }
        }
        self.card = Some(card);
        guard.defuse();
        Ok(())
    }

    pub fn blocking_read_blocks(&mut self, block_index: u32, blocks: &mut [DataBlock]) -> Result<(), Error> {
        let card = *self.card()?;
        let address = block_address(&card, block_index, blocks.len())?;
        blocking_wait_ready::<T>(card.rca, self.config)?;
        let index = if blocks.len() == 1 { 17 } else { 18 };
        let words = unsafe { core::slice::from_raw_parts_mut(blocks.as_mut_ptr().cast(), blocks.len() * 128) };
        self.blocking_data(Buffer::Read(words), index, address, 9, Some(card.rca))
    }

    pub fn blocking_write_blocks(&mut self, block_index: u32, blocks: &[DataBlock]) -> Result<(), Error> {
        let card = *self.card()?;
        let address = block_address(&card, block_index, blocks.len())?;
        blocking_wait_ready::<T>(card.rca, self.config)?;
        let index = if blocks.len() == 1 { 24 } else { 25 };
        let words = unsafe { core::slice::from_raw_parts(blocks.as_ptr().cast(), blocks.len() * 128) };
        self.blocking_data(Buffer::Write(words), index, address, 9, Some(card.rca))
    }

    pub fn blocking_read_block(&mut self, block_index: u32, block: &mut DataBlock) -> Result<(), Error> {
        self.blocking_read_blocks(block_index, core::slice::from_mut(block))
    }

    pub fn blocking_write_block(&mut self, block_index: u32, block: &DataBlock) -> Result<(), Error> {
        self.blocking_write_blocks(block_index, core::slice::from_ref(block))
    }

    #[cfg(feature = "embassy")]
    pub async fn read_blocks(&mut self, block_index: u32, blocks: &mut [DataBlock]) -> Result<(), Error> {
        let card = *self.card()?;
        let address = block_address(&card, block_index, blocks.len())?;
        wait_ready::<T>(card.rca, self.config).await?;
        let index = if blocks.len() == 1 { 17 } else { 18 };
        let words = unsafe { core::slice::from_raw_parts_mut(blocks.as_mut_ptr().cast(), blocks.len() * 128) };
        self.data(Buffer::Read(words), index, address, 9, Some(card.rca)).await
    }

    #[cfg(feature = "embassy")]
    pub async fn write_blocks(&mut self, block_index: u32, blocks: &[DataBlock]) -> Result<(), Error> {
        let card = *self.card()?;
        let address = block_address(&card, block_index, blocks.len())?;
        wait_ready::<T>(card.rca, self.config).await?;
        let index = if blocks.len() == 1 { 24 } else { 25 };
        let words = unsafe { core::slice::from_raw_parts(blocks.as_ptr().cast(), blocks.len() * 128) };
        self.data(Buffer::Write(words), index, address, 9, Some(card.rca)).await
    }

    #[cfg(feature = "embassy")]
    pub async fn read_block(&mut self, block_index: u32, block: &mut DataBlock) -> Result<(), Error> {
        self.read_blocks(block_index, core::slice::from_mut(block)).await
    }

    #[cfg(feature = "embassy")]
    pub async fn write_block(&mut self, block_index: u32, block: &DataBlock) -> Result<(), Error> {
        self.write_blocks(block_index, core::slice::from_ref(block)).await
    }

    pub fn blocking_read_sd_status(&mut self) -> Result<SDStatus, Error> {
        let rca = self.card()?.rca;
        app_command::<T>(rca, self.config)?;
        let mut words = [0; 16];
        self.blocking_data(Buffer::Read(&mut words), 13, 0, 6, Some(rca))?;
        let status = sd_status(&words);
        self.card.as_mut().unwrap().status = status;
        Ok(status)
    }

    #[cfg(feature = "embassy")]
    pub async fn read_sd_status(&mut self) -> Result<SDStatus, Error> {
        let rca = self.card()?.rca;
        app_command::<T>(rca, self.config)?;
        let mut words = [0; 16];
        self.data(Buffer::Read(&mut words), 13, 0, 6, Some(rca)).await?;
        let status = sd_status(&words);
        self.card.as_mut().unwrap().status = status;
        Ok(status)
    }

    pub fn card(&self) -> Result<&Card, Error> {
        self.card.as_ref().ok_or(Error::NoCard)
    }
    pub fn clock(&self) -> Hertz {
        self.clock
    }
    pub fn signalling(&self) -> Signalling {
        self.signalling
    }
}

impl<T: Instance, D: SdioDma<T>> Drop for Sdmmc<'_, T, D> {
    fn drop(&mut self) {
        critical_section::with(|_| {
            T::Interrupt::disable();
            reset_controller::<T>();
            for pin in self.pins.iter_mut().flatten() {
                pin.set_as_disconnected();
            }
            T::disable();
        });
    }
}

trait SealedInstance {
    const REGS: pac::sdio::Sdio;
    fn waker() -> &'static AtomicWaker;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccPeripheral + RemapPeripheral + embassy_hal_internal::PeripheralType {
    type Interrupt: Interrupt;
}

impl SealedInstance for peripherals::SDIO {
    const REGS: pac::sdio::Sdio = pac::SDIO;
    fn waker() -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }
}
impl Instance for peripherals::SDIO {
    type Interrupt = crate::_generated::peripheral_interrupts::SDIO::GLOBAL;
}

pin_trait!(CkPin, Instance);
pin_trait!(CmdPin, Instance);
pin_trait!(D0Pin, Instance);
pin_trait!(D1Pin, Instance);
pin_trait!(D2Pin, Instance);
pin_trait!(D3Pin, Instance);
pin_trait!(D4Pin, Instance);
pin_trait!(D5Pin, Instance);
pin_trait!(D6Pin, Instance);
pin_trait!(D7Pin, Instance);
dma_trait!(SdioDma, Instance);

