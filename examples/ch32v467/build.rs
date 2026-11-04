fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=flash-test.x");
    if std::env::var_os("CARGO_FEATURE_FLASH_WRITE_TEST").is_some() {
        let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap();
        println!("cargo:rustc-link-arg-bin=hal_flash=-T{manifest}/flash-test.x");
    }
}
