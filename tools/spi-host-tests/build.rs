#[path = "../host-tests-common/build_helpers.rs"]
mod build_helpers;
use build_helpers::{canonical, read, strip_inner_docs};

use std::path::PathBuf;
use std::{env, fs};

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
    let driver = strip_inner_docs(read(&source));
    // The production driver no longer carries an inline test module; the
    // harness owns the only tests, so append its lifecycle module.
    let driver = format!("{driver}\n#[cfg(test)]\n#[path = {lifecycle:?}]\nmod lifecycle;\n");
    let output = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by cargo"));
    fs::write(output.join("spi.rs"), driver).expect("cannot write the generated SPI module");
}
