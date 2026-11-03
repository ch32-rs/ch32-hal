use std::path::PathBuf;
use std::{env, fs};

fn main() {
    for name in ["dma", "dma_v4x7", "spi_v4x7"] {
        println!("cargo:rustc-check-cfg=cfg({name})");
        println!("cargo:rustc-cfg={name}");
    }
    let harness = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let source = harness.join("../../src/spi.rs").canonicalize().unwrap();
    let lifecycle = harness.join("src/lifecycle.rs");
    for path in [&source, &lifecycle] {
        println!("cargo:rerun-if-changed={}", path.display());
    }
    let driver = fs::read_to_string(source).unwrap();
    // Drop the module-level doc comments: `include!` places this file inside
    // an inline module, where `//!` lines after the include point are invalid.
    let driver = driver
        .lines()
        .skip_while(|line| line.is_empty() || line.starts_with("//!"))
        .collect::<Vec<_>>()
        .join("\n")
        .replace(
            "#[cfg(test)]\n#[path = \"spi/tests.rs\"]\nmod tests;",
            &format!("#[cfg(test)]\n#[path = {lifecycle:?}]\nmod lifecycle;"),
        );
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(output.join("spi.rs"), driver).unwrap();
}
