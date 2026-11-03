use std::path::PathBuf;
use std::{env, fs};

fn strip_inner_docs(source: String) -> String {
    source
        .lines()
        .skip_while(|line| line.is_empty() || line.starts_with("//!"))
        .collect::<Vec<_>>()
        .join("\n")
}

fn main() {
    println!("cargo:rustc-check-cfg=cfg(flash_v4x7)");
    println!("cargo:rustc-cfg=flash_v4x7");

    let harness = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let module_source = harness.join("../../src/flash/mod.rs").canonicalize().unwrap();
    let driver_source = harness.join("../../src/flash/v4x7.rs").canonicalize().unwrap();
    let lifecycle = harness.join("src/lifecycle.rs");
    for path in [&module_source, &driver_source, &lifecycle] {
        println!("cargo:rerun-if-changed={}", path.display());
    }
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // `include!` cannot resolve `mod v4x7;` relative to OUT_DIR, so flatten it
    // into an inline module. Also drop the ELF-only link section, which rustc
    // rejects for Mach-O host objects.
    let inner = output.join("v4x7_inner.rs");
    let driver = fs::read_to_string(driver_source).unwrap();
    // `include!` places these files inside inline modules, where leading `//!`
    // lines are invalid.
    let driver = strip_inner_docs(driver)
        .replace("#[link_section = \".text.ch32_hal_flash\"]\n", "")
        .replace(
            "#[cfg(test)]\n#[path = \"v4x7_tests.rs\"]\nmod tests;",
            &format!("#[cfg(test)]\n#[path = {lifecycle:?}]\nmod lifecycle;"),
        );
    fs::write(&inner, driver).unwrap();

    let mut module = strip_inner_docs(fs::read_to_string(module_source).unwrap());
    module = module.replace(
        "#[cfg(flash_v4x7)]\nmod v4x7;",
        &format!("#[cfg(flash_v4x7)]\npub mod v4x7 {{\n    include!({inner:?});\n}}"),
    );
    // The harness has no generated PAC memory layout. Use the CH32V467WEU6
    // geometry: 992 KiB of application flash and a 256-byte program page.
    let start = module
        .find("#[cfg(flash_v4x7)]\npub const FLASH_SIZE")
        .expect("FLASH_SIZE definition moved");
    let end = module
        .find("\n\n/// Read size (always 1)")
        .expect("READ_SIZE definition moved");
    module.replace_range(
        start..end,
        "#[cfg(flash_v4x7)]\npub const FLASH_SIZE: usize = 992 * 1024;\n\n\
         #[cfg(flash_v4x7)]\npub const WRITE_SIZE: usize = 256;",
    );
    fs::write(output.join("flash.rs"), module).unwrap();
}
