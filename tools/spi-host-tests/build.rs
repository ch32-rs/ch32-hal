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

fn main() {
    for name in ["dma", "dma_v4x7", "spi_v4x7"] {
        println!("cargo:rustc-check-cfg=cfg({name})");
        println!("cargo:rustc-cfg={name}");
    }
    let harness = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is set by cargo"));
    let source = canonical(&harness.join("../../src/spi.rs"));
    let lifecycle = harness.join("src/lifecycle.rs");
    for path in [&source, &lifecycle] {
        println!("cargo:rerun-if-changed={}", path.display());
    }
    let driver = read(&source);
    // Drop the module-level doc comments: `include!` places this file inside
    // an inline module, where `//!` lines after the include point are invalid.
    let driver = driver
        .lines()
        .skip_while(|line| line.is_empty() || line.starts_with("//!"))
        .collect::<Vec<_>>()
        .join("\n");
    let driver = replace(
        driver,
        "#[cfg(test)]\n#[path = \"spi/tests.rs\"]\nmod tests;",
        &format!("#[cfg(test)]\n#[path = {lifecycle:?}]\nmod lifecycle;"),
    );
    let output = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by cargo"));
    fs::write(output.join("spi.rs"), driver).expect("cannot write the generated SPI module");
}
