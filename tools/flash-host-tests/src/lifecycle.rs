use std::sync::{Mutex, MutexGuard};

use super::*;
use crate::flash_model as model;
use crate::pac;

/// The model and the RAM-backed registers are process-wide singletons.
/// Serialize tests here so a plain `cargo test` cannot race them.
static TEST_LOCK: Mutex<()> = Mutex::new(());

fn lock() -> MutexGuard<'static, ()> {
    TEST_LOCK.lock().unwrap_or_else(|error| error.into_inner())
}

/// `unlock()` writes the two key sequences to `KEYR`/`MODEKEYR` and verifies
/// that both lock bits cleared.
#[test]
fn unlock_clears_lock_and_flock() {
    let _guard = lock();
    model::reset();
    model::set_locked(true);
    assert!(pac::FLASH.ctlr().read().lock());
    assert!(pac::FLASH.ctlr().read().flock());
    assert_eq!(unlock(), Ok(()));
    assert!(!pac::FLASH.ctlr().read().lock());
    assert!(!pac::FLASH.ctlr().read().flock());
}

/// RM 32.5.6: each fast-page 32-bit store waits for `WRBSY`, not `BSY`.
/// `BSY` stays set for the whole 256-byte load, so a `BSY|WRBSY` mask never
/// clears before `PGSTRT`.
#[test]
fn write_buffer_wait_ignores_bsy() {
    let _guard = lock();
    model::reset();
    model::set_busy(true);
    assert_eq!(wait_write_buffer(8), Ok(()));
    // Negative control: the same status must still block `wait_idle`.
    assert_eq!(wait_idle(8), Err(Error::Timeout));

    // Pin the WRBSY bit position: with BSY clear and only WRBSY set, the
    // per-word wait must still time out.
    model::set_busy(false);
    model::set_wrbsy(true);
    assert_eq!(wait_write_buffer(8), Err(Error::Timeout));
}

/// Regression for `flash-write-test` failing with `FLASH_RESULT=FAIL code=21`
/// (`Error::Timeout`) on CH32V467.
#[test]
fn program_page_completes_while_bsy_is_set_during_the_load() {
    let _guard = lock();
    let mut page = [0u32; 64];
    let bytes: [u8; 256] = core::array::from_fn(|index| (index as u8).wrapping_mul(37));
    model::reset();
    let result = unsafe { program_page(page.as_mut_ptr() as usize, &bytes, 64) };
    assert_eq!(result, Ok(()));
    let observed = unsafe { core::slice::from_raw_parts(page.as_ptr() as *const u8, bytes.len()) };
    assert_eq!(observed, bytes);
    // BSY was observed by the driver while it loaded the page; the load only
    // succeeds because the per-word wait looks at WRBSY.
    assert!(model::busy_reads() > 0);
}

/// The per-word wait must still report a real stall, rather than accepting
/// every status. The stall starts at `FTPG`; presetting `WRBSY` would make the
/// initial `BSY|WRBSY` idle wait time out before the load even begins, so the
/// test would never reach `wait_write_buffer`.
#[test]
fn program_page_times_out_when_wrbsy_is_stalled_during_the_load() {
    let _guard = lock();
    let mut page = [0u32; 64];
    let bytes = [0u8; 256];
    model::reset();
    model::stall_wrbsy_after_ftpg(true);
    let result = unsafe { program_page(page.as_mut_ptr() as usize, &bytes, 8) };
    assert_eq!(result, Err(Error::Timeout));
    assert!(model::load_started(), "the fast-page load must have started");
}
