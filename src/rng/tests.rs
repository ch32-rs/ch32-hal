use super::*;

#[test]
fn all_ready_and_error_flags_are_observed() {
    for flags in 0..128 {
        assert_eq!(ready_or_error(&pac::rng::regs::Sr(flags)), flags & 0x67 != 0);
    }
}

#[test]
fn rand_core_errors_have_distinct_nonzero_custom_codes() {
    let seed: rand_core::Error = Error::SeedError.into();
    let clock: rand_core::Error = Error::ClockError.into();
    assert!(seed.code().unwrap().get() >= rand_core::Error::CUSTOM_START);
    assert_ne!(seed.code(), clock.code());
}
