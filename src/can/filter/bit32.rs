use core::marker::PhantomData;

use embedded_can::Id;

use super::{Bit16Mode, Bit32Mode, CanFilter, FilterMode, FilterOptions, ListMode, MaskMode};

impl<MODE: FilterMode> CanFilter<Bit32Mode, MODE> {
    pub fn use_16bit(self) -> CanFilter<Bit16Mode, MODE> {
        CanFilter {
            id_mask: self.id_mask,
            id_value: self.id_value,
            mode: self.mode,
            bank: self.bank,
            bit_mode: PhantomData,
        }
    }
}

impl CanFilter<Bit32Mode, ListMode> {
    /// Creates a filter that accepts all frames
    pub fn accept_all() -> Self {
        CanFilter {
            bank: 0,
            mode: ListMode,
            id_value: 0,
            id_mask: 0,
            bit_mode: PhantomData,
        }
    }

    pub fn new_id_list() -> Self {
        Self {
            bank: 0,
            bit_mode: PhantomData,
            mode: ListMode,

            id_mask: 0,
            id_value: 0,
        }
    }
}

impl CanFilter<Bit32Mode, MaskMode> {
    pub fn new_id_mask() -> Self {
        Self {
            bank: 0,
            bit_mode: PhantomData,
            mode: MaskMode,

            id_mask: 0,
            id_value: 0,
        }
    }
}

pub struct Bit32IdReg<'a>(&'a mut u32);

impl Bit32IdReg<'_> {
    pub fn set(&mut self, id: Id, opts: FilterOptions) {
        *self.0 = 0;
        *self.0 |= (opts.rtr_enabled as u32) << 1;

        match id {
            Id::Standard(id) => {
                let id_bits = id.as_raw() as u32;

                *self.0 |= ((id_bits << 3) << 24) | ((id_bits & 0x7) << 20);
            }

            Id::Extended(id) => {
                let std_id = (id.as_raw() as u16) as u32;
                let ext_id = id.as_raw() >> 11;

                *self.0 |= ((std_id << 3) << 24) | ((std_id & 0x7) << 20) | (ext_id << 3);
            }
        }
    }
}

impl CanFilter<Bit32Mode, ListMode> {
    pub fn get(&mut self, index: usize) -> Option<Bit32IdReg> {
        match index {
            0 => Some(Bit32IdReg(&mut self.id_value)),
            1 => Some(Bit32IdReg(&mut self.id_mask)),
            _ => None,
        }
    }
}
