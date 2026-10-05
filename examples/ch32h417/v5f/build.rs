fn main() {
    // Use qingke-rt's standard link.x, but override memory.x
    // with our V5F-specific layout (Flash @ 0x08002000, RAM @ DTCM).
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-search={}", std::env::current_dir().unwrap().display());
}
