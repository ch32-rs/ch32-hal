use std::path::PathBuf;
use std::{env, fs};

fn main() {
    for name in ["dma", "dma_v4x7", "ch32v4", "i2c_v3"] {
        println!("cargo:rustc-check-cfg=cfg({name})");
        println!("cargo:rustc-cfg={name}");
    }
    let harness = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let source = harness.join("../../src/i2c.rs").canonicalize().unwrap();
    let tests = harness.join("../../src/i2c/tests.rs").canonicalize().unwrap();
    let lifecycle = harness.join("src/lifecycle.rs");
    for path in [&source, &tests, &lifecycle] {
        println!("cargo:rerun-if-changed={}", path.display());
    }
    let driver = fs::read_to_string(source).unwrap();
    let driver = driver
        .strip_prefix("//! Inter-Integrated-Circuit (I2C)\n")
        .unwrap()
        .replace("#[path = \"i2c/tests.rs\"]", &format!("#[path = {tests:?}]"));
    let driver = format!("{driver}\n#[cfg(test)]\n#[path = {lifecycle:?}]\nmod lifecycle;\n");
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(output.join("i2c.rs"), driver).unwrap();
}
