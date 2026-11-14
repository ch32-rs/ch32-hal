fn main() {
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // `pioc_rainbow` runs firmware out of the top 4 KiB of SRAM, which `memory-x`
    // otherwise hands to the stack. Only that binary gets the reservation.
    println!("cargo:rustc-link-search={}", std::env::var("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rustc-link-arg-bin=pioc_rainbow=-Tpioc.x");
    println!("cargo:rerun-if-changed=pioc.x");
    // println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
