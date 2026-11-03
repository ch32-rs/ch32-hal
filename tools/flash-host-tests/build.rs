use std::path::{Path, PathBuf};
use std::{env, fs};

fn read(path: &Path) -> String {
    fs::read_to_string(path).unwrap_or_else(|error| panic!("cannot read {}: {error}", path.display()))
}

fn canonical(path: &Path) -> PathBuf {
    path.canonicalize()
        .unwrap_or_else(|error| panic!("cannot resolve {}: {error}", path.display()))
}

/// Replace one production-source anchor, failing loudly when the source moves.
fn replace(source: String, from: &str, to: &str) -> String {
    assert_eq!(
        source.matches(from).count(),
        1,
        "production-source anchor moved (expected one match): {from:?}"
    );
    source.replace(from, to)
}

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

    let harness = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is set by cargo"));
    let module_source = canonical(&harness.join("../../src/flash/mod.rs"));
    let driver_source = canonical(&harness.join("../../src/flash/v4x7.rs"));
    let lifecycle = harness.join("src/lifecycle.rs");
    for path in [&module_source, &driver_source, &lifecycle] {
        println!("cargo:rerun-if-changed={}", path.display());
    }
    let output = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by cargo"));

    // `include!` cannot resolve `mod v4x7;` relative to OUT_DIR, so flatten it
    // into an inline module. Also drop the ELF-only link section, which rustc
    // rejects for Mach-O host objects.
    const LINK_SECTION: &str = "#[link_section = \".text.ch32_hal_flash\"]\n";
    let inner = output.join("v4x7_inner.rs");
    let driver = strip_inner_docs(read(&driver_source));
    assert_eq!(
        driver.matches(LINK_SECTION).count(),
        3,
        "expected three ELF link-section attributes in src/flash/v4x7.rs"
    );
    let driver = driver.replace(LINK_SECTION, "");
    let driver = replace(
        driver,
        "#[cfg(test)]\n#[path = \"v4x7_tests.rs\"]\nmod tests;",
        &format!("#[cfg(test)]\n#[path = {lifecycle:?}]\nmod lifecycle;"),
    );
    fs::write(&inner, driver).expect("cannot write the generated v4x7 module");

    let mut module = strip_inner_docs(read(&module_source));
    module = replace(
        module,
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
    fs::write(output.join("flash.rs"), module).expect("cannot write the generated flash module");
}
