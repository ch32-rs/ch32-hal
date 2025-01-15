use super::filter::{BitMode, FilterMode};

const CAN_TX_TIMEOUT: u32 = 0xFFF;

pub(crate) struct Registers(pub crate::pac::can::Can);

impl Registers {
    pub fn enter_init_mode(&self) {
        self.0.ctlr().modify(|w| {
            w.set_sleep(false); // Wake up
            w.set_inrq(true); // Request enter init mode
        });

        // Wait until CAN is in init mode
        loop {
            if self.0.statr().read().inak() {
                break;
            }
        }
    }

    pub fn leave_init_mode(&self) {
        self.0.ctlr().modify(|w| w.set_inrq(false)); // Request exit init mode

        // Wait until CAN is no longer in init mode
        loop {
            if !self.0.statr().read().inak() {
                break;
            }
        }
    }

    pub fn set_bit_timing_and_mode(&self, bt: super::util::NominalBitTiming, mode: super::CanMode) {
        let prescaler = u16::from(bt.prescaler) & 0x1FF;
        let seg1 = u8::from(bt.seg1);
        let seg2 = u8::from(bt.seg2) & 0x7F;
        let sync_jump_width = u8::from(bt.sync_jump_width) & 0x7F;
        self.0.btimr().modify(|w| {
            w.set_brp(prescaler - 1); // Set CAN clock prescaler
            w.set_ts1(seg1 - 1); // Set CAN time quantum in bit segment 1
            w.set_ts2(seg2 - 1); // Set CAN time quantum in bit segment 2
            w.set_sjw(sync_jump_width - 1); // Set CAN resync jump width
            w.set_lbkm(mode.regs().lbkm); // Set silent mode bit from mode
            w.set_silm(mode.regs().silm); // Set loopback mode bit from mode
        });
    }

    /// Each filter bank consists of 2 32-bit registers CAN_FxR0 and CAN_FxR1
    pub fn add_filter<BIT: BitMode, MODE: FilterMode>(
        &self,
        filter: super::CanFilter<BIT, MODE>,
        associate_fifo: &super::CanFifo,
    ) {
        self.0.fctlr().modify(|w| w.set_finit(true)); // Enable filter init mode
        self.0.fwr().modify(|w| w.set_fact(filter.bank, true)); // Activate new filter in filter bank
        self.0
            .fscfgr()
            .modify(|w| w.set_fsc(filter.bank, filter.bit_mode.val_bool())); // Set filter scale config (32bit or 16bit mode)
        self.0
            .fr(filter.fr_id_value_reg())
            .write_value(crate::pac::can::regs::Fr(filter.id_value)); // Set filter's id value to match/mask
        self.0
            .fr(filter.fr_id_mask_reg())
            .write_value(crate::pac::can::regs::Fr(filter.id_mask)); // Set filter's id bits to mask
        self.0
            .fmcfgr()
            .modify(|w| w.set_fbm(filter.bank, filter.mode.val_bool())); // Set new filter's operating mode
        self.0
            .fafifor()
            .modify(|w| w.set_ffa(filter.bank, associate_fifo.val_bool())); // Associate CAN's FIFO to new filter
        self.0.fwr().modify(|w| w.set_fact(filter.bank, true)); // Activate new filter
        self.0.fctlr().modify(|w| w.set_finit(false)); // Exit filter init mode
    }

    pub fn find_free_mailbox(&self) -> Option<usize> {
        let tstatr = self.0.tstatr().read();
        if tstatr.tme(0) {
            return Some(0);
        }
        if tstatr.tme(1) {
            return Some(1);
        }
        if tstatr.tme(2) {
            return Some(2);
        }
        return None;
    }

    pub fn write_frame_mailbox(&self, mailbox_num: usize, frame: &super::CanFrame) {
        let tx_data_high: u32 = ((frame.data[7] as u32) << 24)
            | ((frame.data[6] as u32) << 16)
            | ((frame.data[5] as u32) << 8)
            | frame.data[4] as u32;
        let tx_data_low: u32 = ((frame.data[3] as u32) << 24)
            | ((frame.data[2] as u32) << 16)
            | ((frame.data[1] as u32) << 8)
            | frame.data[0] as u32;

        self.0.txmdtr(mailbox_num).modify(|w| w.set_dlc(8)); // Set message length in bytes
        self.0
            .txmdhr(mailbox_num)
            .write_value(crate::pac::can::regs::Txmdhr(tx_data_high));
        self.0
            .txmdlr(mailbox_num)
            .write_value(crate::pac::can::regs::Txmdlr(tx_data_low));
        self.0.txmir(mailbox_num).write_value(crate::pac::can::regs::Txmir(0x0)); // Clear CAN TXMIR register
        self.0.txmir(mailbox_num).modify(|w| {
            w.set_stid(match frame.id {
                embedded_can::Id::Standard(id) => id.as_raw(),
                embedded_can::Id::Extended(_) => unimplemented!(),
            }); // Using CAN Standard ID for message
            w.set_txrq(true); // Initiate mailbox transfer request
        });
    }

    pub fn transmit_status(&self, mailbox_num: usize) -> super::TxStatus {
        let mut wait_status: u32 = 0;
        while !self.0.tstatr().read().txok(mailbox_num) && wait_status < CAN_TX_TIMEOUT {
            wait_status += 1;
        }
        if wait_status == CAN_TX_TIMEOUT {
            return super::TxStatus::TimeoutError;
        }

        let tx_result = self.0.tstatr().read();
        if tx_result.txok(mailbox_num) {
            return super::TxStatus::Sent;
        }
        if tx_result.alst(mailbox_num) {
            return super::TxStatus::ArbitrationError;
        }
        if tx_result.terr(mailbox_num) {
            return super::TxStatus::OtherError;
        }

        super::TxStatus::OtherError
    }
}
