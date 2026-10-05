//! The debug module.
//!
//! See-also: https://github.com/openwch/ch32v003/blob/main/EVT/EXAM/SDI_Printf/SDI_Printf/Debug/debug.c

use core::sync::atomic::{AtomicBool, Ordering};

use qingke::dm::{DATA0, DATA1};
use qingke::riscv;

const DEBUG_DATA0_ADDRESS: *mut u32 = DATA0 as *mut u32;
const DEBUG_DATA1_ADDRESS: *mut u32 = DATA1 as *mut u32;

pub struct SDIPrint;

impl SDIPrint {
    pub fn enable() {
        unsafe {
            // Enable SDI print
            core::ptr::write_volatile(DEBUG_DATA0_ADDRESS, 0);
            riscv::asm::delay(100000);
        }
    }

    #[inline]
    pub fn is_busy() -> bool {
        unsafe { core::ptr::read_volatile(DEBUG_DATA0_ADDRESS) != 0 }
    }

    /// Write, giving up (and dropping the rest of the message) once a chunk
    /// waits longer than [`SPINS`] iterations.
    ///
    /// SDI print is only drained while the debugger has it armed
    /// (`wlink --enable-sdi-print`). With it disarmed the blocking
    /// [`core::fmt::Write`] impl spins forever on the second chunk — which is
    /// fine for a core that has nothing else to do, but not for a core in a
    /// control loop (or on hart 1, which must not stall behind a console).
    pub fn write_str_lossy(s: &str) -> core::fmt::Result {
        // Once a lossy write has given up, nothing is draining `DATA0`, so every
        // later message would pay the same wait again — skip it outright.
        if GAVE_UP.load(Ordering::Relaxed) {
            return Ok(());
        }
        write(s, Some(SPINS))
    }
}

/// Writer that routes formatting through [`SDIPrint::write_str_lossy`], so a
/// `try_println!` never blocks even though `SDIPrint`'s own [`core::fmt::Write`]
/// impl does.
struct LossyWriter;

impl core::fmt::Write for LossyWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        SDIPrint::write_str_lossy(s)
    }
}

/// Format `args` into the lossy writer and terminate the line.
pub fn write_fmt_lossy(args: core::fmt::Arguments) -> core::fmt::Result {
    use core::fmt::Write as _;

    let mut writer = LossyWriter;
    core::fmt::write(&mut writer, args)?;
    writer.write_str("\n")
}

/// Iterations [`SDIPrint::write_str_lossy`] tolerates per chunk. Kept small
/// because each iteration is a *debug-module* read, which is far slower than a
/// normal load.
const SPINS: u32 = 200_000;

/// Set when a lossy write gives up; see [`SDIPrint::write_str_lossy`].
static GAVE_UP: AtomicBool = AtomicBool::new(false);

impl core::fmt::Write for SDIPrint {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        write(s, None)
    }
}

fn write(s: &str, spins: Option<u32>) -> core::fmt::Result {
    {
        let mut data = [0u8; 8];
        for chunk in s.as_bytes().chunks(7) {
            data[1..chunk.len() + 1].copy_from_slice(chunk);
            data[0] = chunk.len() as u8;

            // data1 is the last 4 bytes of data
            let data1 = u32::from_le_bytes(data[4..].try_into().unwrap());
            let data0 = u32::from_le_bytes(data[..4].try_into().unwrap());

            let mut waited = 0u32;
            while SDIPrint::is_busy() {
                if let Some(limit) = spins {
                    waited += 1;
                    if waited > limit {
                        GAVE_UP.store(true, Ordering::Relaxed);
                        return Ok(());
                    }
                }
            }

            unsafe {
                core::ptr::write_volatile(DEBUG_DATA1_ADDRESS, data1);
                core::ptr::write_volatile(DEBUG_DATA0_ADDRESS, data0);
            }
        }

        Ok(())
    }
}

/// `println!` over SDI that drops the message rather than spinning when the
/// console is not armed. Use it in control loops; use [`println!`] when the
/// console is known to be attached.
#[macro_export]
macro_rules! try_println {
    ($($arg:tt)*) => {
        {
            // Not `writeln!(&mut SDIPrint, ..)`: that goes through the *blocking*
            // `Write` impl, which is exactly what this variant exists to avoid.
            let _ = $crate::debug::write_fmt_lossy(format_args!($($arg)*));
        }
    };
}

#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write;
            use core::writeln;

            writeln!(&mut $crate::debug::SDIPrint, $($arg)*).unwrap();
        }
    }
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write;
            use core::write;

            write!(&mut $crate::debug::SDIPrint, $($arg)*).unwrap();
        }
    }
}
