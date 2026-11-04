use core::sync::atomic::{compiler_fence, fence, Ordering};
use core::task::Poll;

use super::dma_bdma::{priority_bits, size_bits};
use super::word::WordSize;
use super::{AnyChannel, ChannelInfo, Dir, DmaInfo, Request, TransferOptions, STATE};
use crate::pac;
use crate::pac::InterruptNumber;

impl ChannelInfo {
    fn regs(&self) -> pac::dma::Ch {
        match self.dma {
            DmaInfo::Dma(regs) if self.num < 7 => regs.ch(self.num),
            DmaInfo::Dma(regs) => regs.ech(self.num - 7),
            DmaInfo::Dma2(regs) if self.num < 7 => regs.ch(self.num),
            DmaInfo::Dma2(regs) => regs.ech(self.num - 7),
        }
    }

    fn shift(&self) -> usize {
        if self.num < 7 {
            self.num * 4
        } else {
            (self.num - 7) * 4
        }
    }

    fn flags(&self) -> u32 {
        let bits = match self.dma {
            DmaInfo::Dma(regs) if self.num < 7 => regs.intfr().read().0,
            DmaInfo::Dma(regs) => regs.extem_intfr().read().0,
            DmaInfo::Dma2(regs) if self.num < 7 => regs.intfr().read().0,
            DmaInfo::Dma2(regs) => regs.extem_intfr().read().0,
        };
        (bits >> self.shift()) & 0xf
    }

    fn clear_flags(&self, flags: u32) {
        let bits = flags << self.shift();
        match self.dma {
            DmaInfo::Dma(regs) if self.num < 7 => regs.intfcr().write_value(pac::dma::regs::Intfr(bits)),
            DmaInfo::Dma(regs) => regs.extem_intfcr().write_value(pac::dma::regs::ExtemIntfr(bits)),
            DmaInfo::Dma2(regs) if self.num < 7 => regs.intfcr().write_value(pac::dma::regs::Intfr(bits)),
            DmaInfo::Dma2(regs) => regs.extem_intfcr().write_value(pac::dma::regs::ExtemIntfr(bits)),
        }
    }
}

impl AnyChannel {
    pub(super) fn has_error(&self) -> bool {
        STATE[self.id as usize].transfer_error.load(Ordering::Acquire) || self.info().flags() & 8 != 0
    }

    pub(crate) unsafe fn on_irq(&self) {
        let info = self.info();
        let flags = info.flags();
        let config = info.regs().cfgr().read();
        if flags & 8 != 0 && config.teie() {
            self.request_stop();
            let state = &STATE[self.id as usize];
            if state.report_error.load(Ordering::Acquire) {
                state.transfer_error.store(true, Ordering::Release);
                state.waker.wake();
                return;
            }
            panic!("DMA: transfer error on channel {}", self.id);
        }
        let half = flags & 4 != 0 && config.htie();
        let complete = flags & 2 != 0 && config.tcie();
        if !half && !complete {
            return;
        }
        info.clear_flags(if half { 4 } else { 0 } | if complete { 2 } else { 0 });
        let state = &STATE[self.id as usize];
        if complete {
            critical_section::with(|_| {
                let count = state.complete_count.load(Ordering::Acquire);
                state.complete_count.store(count.wrapping_add(1), Ordering::Release);
                if state.stop_at_complete.load(Ordering::Acquire) {
                    state.stop_at_complete.store(false, Ordering::Release);
                    self.request_stop();
                }
            });
        }
        state.waker.wake();
    }

    pub(super) unsafe fn configure(
        &self,
        _request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: WordSize,
        options: TransferOptions,
    ) {
        use pac::dma::vals::{Dir as Direction, Pl, Size};

        assert!(mem_len > 0 && mem_len <= 0xffff);
        let info = self.info();
        let channel = info.regs();
        assert!(!self.is_running());
        self.request_stop();
        STATE[self.id as usize].complete_count.store(0, Ordering::Release);
        STATE[self.id as usize].stop_at_complete.store(false, Ordering::Release);
        STATE[self.id as usize].report_error.store(false, Ordering::Release);
        STATE[self.id as usize].transfer_error.store(false, Ordering::Release);
        channel.paddr().write_value(peri_addr as u32);
        channel.maddr().write_value(mem_addr as u32);
        channel.cntr().write(|register| register.set_ndt(mem_len as u16));
        let size = Size::from_bits(size_bits(data_size));
        channel.cfgr().write(|register| {
            register.set_psize(size);
            register.set_msize(size);
            register.set_dir(match dir {
                Dir::PeripheralToMemory => Direction::FROMPERIPHERAL,
                Dir::MemoryToPeripheral => Direction::FROMMEMORY,
            });
            register.set_pl(Pl::from_bits(priority_bits(options.priority)));
            register.set_minc(incr_mem);
            register.set_pinc(options.peripheral_increment);
            register.set_mem2mem(options.memory_to_memory);
            register.set_circ(options.circular);
            register.set_tcie(options.complete_transfer_ir);
            register.set_htie(options.half_transfer_ir);
            register.set_teie(true);
        });
        if let Some(irq) = info.irq {
            qingke::pfic::unpend_interrupt(irq.number() as u8);
            qingke::pfic::enable_interrupt(irq.number() as u8);
        }
    }

    pub(super) fn start(&self) {
        fence(Ordering::SeqCst);
        self.info().regs().cfgr().modify(|register| register.set_en(true));
    }

    pub(super) fn request_stop(&self) {
        let info = self.info();
        info.regs().cfgr().write(|register| register.set_en(false));
        while info.regs().cfgr().read().en() {}
        info.clear_flags(0xf);
    }

    pub(super) fn is_running(&self) -> bool {
        let info = self.info();
        let config = info.regs().cfgr().read();
        config.en()
            && (config.circ()
                || (STATE[self.id as usize].complete_count.load(Ordering::Acquire) == 0 && info.flags() & 2 == 0))
    }

    pub(super) fn get_remaining_transfers(&self) -> u16 {
        self.info().regs().cntr().read().ndt()
    }

    pub(super) fn disable_circular_mode(&self) {
        critical_section::with(|_| {
            if self.is_running() {
                self.info().clear_flags(2);
                STATE[self.id as usize].stop_at_complete.store(true, Ordering::Release);
            }
        });
    }

    pub(super) fn poll_stop(&self) -> Poll<()> {
        compiler_fence(Ordering::SeqCst);
        if self.is_running() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}
