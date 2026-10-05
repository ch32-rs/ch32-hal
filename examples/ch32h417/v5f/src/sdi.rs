//! SDI printf for hart 1, on top of metapac and the core's debug module.
//!
//! Same hardware handshake as `ch32-hal`'s `SDIPrint` — write `DATA1`, then
//! `DATA0`, and the debug module consumes `DATA0` — but the wait is bounded:
//! hart 1 must never be left spinning inside a print. That matters because SDI
//! print is only drained while the debugger has it armed (`wlink
//! --enable-sdi-print`); with it disarmed the first chunk still goes out and the
//! second would spin forever, which is exactly how `sdi_cpuid`'s tick counter
//! used to stall. Use [`SdiPrint::write_str_lossy`] (or `sdi_try_println!`) when
//! losing a line is better than stalling the core, and the blocking
//! [`core::fmt::Write`] impl when the console is known to be armed.

use core::fmt::{self, Write};

use qingke::dm::{DATA0, DATA1};

/// How many spins `write_str_lossy` tolerates per 7-byte chunk before dropping
/// the rest of the message.
const SPINS: u32 = 1_000_000;

/// The 7-byte-per-chunk SDI writer.
pub struct SdiPrint;

impl SdiPrint {
    /// Clear the handshake before the first write, like WCH's `SDI_Printf_Enable`.
    pub fn enable() {
        unsafe { core::ptr::write_volatile(DATA0 as *mut u32, 0) };
    }

    fn is_busy() -> bool {
        unsafe { core::ptr::read_volatile(DATA0 as *const u32) != 0 }
    }

    /// Write, waiting for the debug module after every chunk.
    pub fn write_blocking(s: &str) -> fmt::Result {
        Self::write(s, None)
    }

    /// Write, giving up (and dropping the rest) once a chunk waits too long.
    pub fn write_str_lossy(s: &str) -> fmt::Result {
        Self::write(s, Some(SPINS))
    }

    fn write(s: &str, spins: Option<u32>) -> fmt::Result {
        let mut data = [0u8; 8];
        for chunk in s.as_bytes().chunks(7) {
            data[1..chunk.len() + 1].copy_from_slice(chunk);
            data[0] = chunk.len() as u8;

            // data1 is the last 4 bytes of data
            let data1 = u32::from_le_bytes(data[4..].try_into().unwrap());
            let data0 = u32::from_le_bytes(data[..4].try_into().unwrap());

            let mut waited = 0u32;
            while Self::is_busy() {
                if let Some(limit) = spins {
                    waited += 1;
                    if waited > limit {
                        return Ok(());
                    }
                }
            }

            unsafe {
                core::ptr::write_volatile(DATA1 as *mut u32, data1);
                core::ptr::write_volatile(DATA0 as *mut u32, data0);
            }
        }

        Ok(())
    }
}

impl Write for SdiPrint {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        Self::write_blocking(s)
    }
}

/// `println!` over SDI, blocking — for when the console is armed.
#[macro_export]
macro_rules! sdi_println {
    ($($arg:tt)*) => {{
        use core::fmt::Write as _;
        let _ = writeln!($crate::sdi::SdiPrint, $($arg)*);
    }};
}

/// `println!` over SDI that drops the message rather than stalling hart 1.
#[macro_export]
macro_rules! sdi_try_println {
    ($($arg:tt)*) => {{
        use core::fmt::Write as _;
        let mut message = $crate::sdi::Message::new();
        let _ = write!(message, $($arg)*);
        let _ = $crate::sdi::SdiPrint::write_str_lossy(message.as_str());
    }};
}

/// Fixed-size formatting buffer so the lossy path never needs an allocator.
pub struct Message {
    buf: [u8; 128],
    len: usize,
}

impl Message {
    pub const fn new() -> Self {
        Self {
            buf: [0; 128],
            len: 0,
        }
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl Write for Message {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let room = self.buf.len() - self.len;
        let n = room.min(s.len());
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}
