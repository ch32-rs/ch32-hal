mod bit16;
mod bit32;

/// Filter mode, either identifier mask or identifier list
pub trait FilterMode {
    fn val_bool(&self) -> bool;
}

/// Matches the incoming ID to a predefined value after applying a predefined bit mask.
pub struct MaskMode;
pub struct ListMode;

impl FilterMode for MaskMode {
    fn val_bool(&self) -> bool {
        false
    }
}

impl FilterMode for ListMode {
    fn val_bool(&self) -> bool {
        true
    }
}

pub trait BitMode {
    fn val_bool(&self) -> bool;
}

pub struct Bit16Mode;
pub struct Bit32Mode;

impl BitMode for Bit16Mode {
    fn val_bool(&self) -> bool {
        false
    }
}

impl BitMode for Bit32Mode {
    fn val_bool(&self) -> bool {
        true
    }
}

/// See table 24-1 of the reference manual for more details on filtering and modes.
/// Each filter is applied for only one bank and for one register on it bank
pub struct CanFilter<BIT: BitMode, MODE: FilterMode> {
    /// Filter bank number, 0-27
    pub bank: usize,
    /// Values for `STID:EXID:IDE:RTR:0` from msb to lsb to be matched with an incoming message's values.
    /// In IdList mode, value should be a 32-bit id or two 16-bit ids.
    pub id_value: u32,
    /// Bit mask to be applied to incoming message before comparing it to a predefined value.
    /// In IdList mode, this is used in the same way as `id_value` is.
    pub id_mask: u32,
    pub bit_mode: BIT,
    pub mode: MODE,
}

impl<BIT: BitMode, MODE: FilterMode> CanFilter<BIT, MODE> {
    pub fn set_bank(&mut self, bank: usize) -> &mut Self {
        self.bank = bank;

        self
    }

    /// Offset in `usize` for bank `n` filter register 1
    pub(crate) fn fr_id_value_reg(&self) -> usize {
        self.bank * 2 + 0
    }

    /// Offset in `usize` for bank `n` filter register 2
    pub(crate) fn fr_id_mask_reg(&self) -> usize {
        self.bank * 2 + 1
    }
}

/// By default rtr is disabled
#[derive(Default)]
pub struct FilterOptions {
    pub use_rtr: bool,
    pub use_extended_id: bool,
}
