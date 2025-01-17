use embedded_can::StandardId;

use super::{Bit16Mode, CanFilter, FilterOptions, ListMode, MaskMode};

pub struct Bit16IdReg<'a>(&'a mut u16);

pub struct Bit16MaskReg<'a> {
    mask: &'a mut u16,
    id: &'a mut u16,
}

impl Bit16IdReg<'_> {
    pub fn set(&mut self, id: StandardId, opts: FilterOptions) {
        let bits = (id.as_raw() << 5) | ((opts.use_rtr as u16) << 4) | ((opts.use_extended_id as u16) << 3);
        *self.0 = 0x0000;
        *self.0 |= bits;
    }
}

impl Bit16MaskReg<'_> {
    pub fn set(&mut self, id: u16, mask: u16, opts: FilterOptions) {
        *self.mask &= 0xFFFF;
        *self.id &= 0xFFFF;

        *self.mask |= (id << 5) | ((opts.use_rtr as u16) << 4);
        *self.id |= (mask << 5) | ((opts.use_rtr as u16) << 4);
    }
}

impl CanFilter<Bit16Mode, ListMode> {
    pub fn get(&mut self, index: usize) -> Option<Bit16IdReg> {
        use core::mem;

        match index {
            0 => {
                let bytes = unsafe { &mut *mem::transmute::<*mut u32, *mut [u16; 2]>(&mut self.id_value) };
                Some(Bit16IdReg(&mut bytes[0]))
            }

            1 => {
                let bytes = unsafe { &mut *mem::transmute::<*mut u32, *mut [u16; 2]>(&mut self.id_value) };
                Some(Bit16IdReg(&mut bytes[1]))
            }

            2 => {
                let bytes = unsafe { &mut *mem::transmute::<*mut u32, *mut [u16; 2]>(&mut self.id_mask) };
                Some(Bit16IdReg(&mut bytes[0]))
            }

            3 => {
                let bytes = unsafe { &mut *mem::transmute::<*mut u32, *mut [u16; 2]>(&mut self.id_mask) };

                Some(Bit16IdReg(&mut bytes[0]))
            }

            _ => None,
        }
    }
}

impl From<[(StandardId, FilterOptions); 4]> for CanFilter<Bit16Mode, ListMode> {
    fn from(value: [(StandardId, FilterOptions); 4]) -> Self {
        let mut filter = CanFilter::new_id_list().use_16bit();

        for (index, (id, opts)) in value.into_iter().enumerate() {
            filter.get(index).expect("index less 4").set(id, opts);
        }

        filter
    }
}

impl CanFilter<Bit16Mode, MaskMode> {
    pub fn get(&mut self, index: usize) -> Option<Bit16MaskReg> {
        use core::mem;

        match index {
            0 => {
                let [id, mask] = unsafe { &mut *mem::transmute::<*mut u32, *mut [u16; 2]>(&mut self.id_value) };

                Some(Bit16MaskReg { id, mask })
            }
            1 => {
                let [id, mask] = unsafe { &mut *mem::transmute::<*mut u32, *mut [u16; 2]>(&mut self.id_mask) };

                Some(Bit16MaskReg { id, mask })
            }
            _ => None,
        }
    }
}
