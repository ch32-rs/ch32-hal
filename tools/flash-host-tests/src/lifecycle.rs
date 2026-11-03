use super::*;
use crate::flash_model as model;

/// RM 32.5.6: each fast-page 32-bit store waits for `WRBSY`, not `BSY`.
/// `BSY` stays set for the whole 256-byte load, so a `BSY|WRBSY` mask never
/// clears before `PGSTRT`.
#[test]
fn write_buffer_wait_ignores_bsy() {
    model::reset();
    model::set_busy(true);
    assert_eq!(wait_write_buffer(8), Ok(()));
    // Negative control: the same status must still block `wait_idle`.
    assert_eq!(wait_idle(8), Err(Error::Timeout));
}

/// Regression for `flash-write-test` failing with `FLASH_RESULT=FAIL code=21`
/// (`Error::Timeout`) on CH32V467.
#[test]
fn program_page_completes_while_bsy_is_set_during_the_load() {
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
/// every status.
#[test]
fn program_page_times_out_when_wrbsy_is_stalled() {
    let mut page = [0u32; 64];
    let bytes = [0u8; 256];
    model::reset();
    model::stall_wrbsy(true);
    let result = unsafe { program_page(page.as_mut_ptr() as usize, &bytes, 8) };
    assert_eq!(result, Err(Error::Timeout));
}
