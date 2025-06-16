use embedded_can;

#[derive(Debug)]
pub struct CanFrame {
    pub(crate) id: embedded_can::Id,
    pub(crate) dlc: usize,
    pub(crate) data: [u8; 8],
    pub(crate) is_remote: bool,
}

impl CanFrame {
    pub fn new(id: impl Into<embedded_can::Id>, raw_data: &[u8]) -> Option<Self> {
        if raw_data.len() > 8 {
            return None;
        }

        let mut data = [0; 8];
        data[..raw_data.len()].copy_from_slice(raw_data);

        Some(CanFrame {
            id: id.into(),
            dlc: raw_data.len(),
            data,
            is_remote: false,
        })
    }

    pub(crate) fn new_from_data_registers(
        id: impl Into<embedded_can::Id>,
        frame_data_unordered: u64,
        dlc: usize,
    ) -> Self {
        let mut data: [u8; 8] = [0; 8];

        data.iter_mut().take(dlc).enumerate().for_each(|(i, byte)| {
            *byte = ((frame_data_unordered >> (i * 8)) & 0xFF) as u8;
        });

        Self {
            id: id.into(),
            data,
            dlc,
            is_remote: false,
        }
    }

    /// Return ID
    pub fn id(&self) -> &embedded_can::Id {
        &self.id
    }

    /// Return length of `data`
    pub fn dlc(&self) -> usize {
        self.dlc
    }

    /// Get reference to data
    pub fn data(&self) -> &[u8] {
        &self.data
    }
}

impl embedded_can::Frame for CanFrame {
    fn new(id: impl Into<embedded_can::Id>, raw_data: &[u8]) -> Option<Self> {
        CanFrame::new(id, raw_data)
    }

    #[allow(unused_variables)]
    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        return None; // Not implemented
    }

    fn is_extended(&self) -> bool {
        match self.id {
            embedded_can::Id::Standard(_) => false,
            embedded_can::Id::Extended(_) => true,
        }
    }

    fn is_remote_frame(&self) -> bool {
        false // Not implemented
    }

    fn id(&self) -> embedded_can::Id {
        self.id.into()
    }

    fn dlc(&self) -> usize {
        self.dlc
    }

    fn data(&self) -> &[u8] {
        // Remote frames do not contain data, yet have a value for the dlc so return
        // an empty slice for remote frames.
        match self.is_remote {
            true => &[],
            false => &self.data[0..self.dlc],
        }
    }
}
