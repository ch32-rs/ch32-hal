fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // Keep RTT symbols from being GC'd by the linker.
    // defmt-rtt's _SEGGER_RTT is only referenced indirectly through
    // the #[global_logger] trait impl chain.
    println!("cargo:rustc-link-arg-bins=--undefined=_SEGGER_RTT");
}
