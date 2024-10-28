use embassy_usb_driver::Direction;


/// USB Direction Trait
pub trait Dir {
    /// Returns the direction value.
    fn dir() -> Direction;
}

/// Marker type for the "IN" direction.
pub struct In;
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

/// Marker type for the "OUT" direction.
pub struct Out;
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}
