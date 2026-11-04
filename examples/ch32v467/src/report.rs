//! Debugger-visible result markers shared by the `examples/ch32v467` binaries.
//!
//! Each binary declares its outcome statics with [`report_static!`] and drives
//! them with [`report_set!`] / [`report_check!`] instead of repeating the
//! `write_volatile` + `defmt::panic!` scaffolding. The markers stay readable
//! from RAM after an unattended run, which is how the on-board results are
//! collected (`xtask report` / a debugger read).

/// Declare one `#[no_mangle] u32` result marker.
#[macro_export]
macro_rules! report_static {
    ($name:ident) => {
        #[no_mangle]
        #[used]
        pub static mut $name: u32 = 0;
    };
}

/// Write `code` into `$name` without panicking.
#[macro_export]
macro_rules! report_set {
    ($name:ident, $code:expr) => {{
        unsafe { ::core::ptr::write_volatile(::core::ptr::addr_of_mut!($name), $code) }
    }};
}

/// Set `$name = $code` and panic, echoing the code to the console.
#[macro_export]
macro_rules! report_fail {
    ($name:ident, $code:expr, $msg:literal) => {{
        $crate::report_set!($name, $code);
        ::defmt::panic!("{} code={}", $msg, $code);
    }};
}

/// Assert `$condition`; on failure write `$name = $code` and panic with `$msg`.
#[macro_export]
macro_rules! report_check {
    ($name:ident, $condition:expr, $code:expr, $msg:literal) => {{
        if !$condition {
            $crate::report_fail!($name, $code, $msg);
        }
    }};
}
