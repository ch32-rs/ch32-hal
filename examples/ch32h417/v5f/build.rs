fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlink.x");

    // Ship this crate's linker scripts. qingke-rt's link.x wants both
    // `memory.x` (our V5F FLASH/RAM/DTCM layout, overriding the one metapac
    // generates) and `device.x` (the svd2rust device-crate hook, which a
    // HAL-free crate provides empty). The current crate's build-script search
    // path takes precedence over dependency build-script paths.
    let out_dir = std::env::var("OUT_DIR").unwrap();
    let out_dir = std::path::PathBuf::from(out_dir);
    std::fs::write(out_dir.join("memory.x"), include_bytes!("memory.x")).unwrap();
    std::fs::write(out_dir.join("device.x"), include_bytes!("device.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=device.x");
}
