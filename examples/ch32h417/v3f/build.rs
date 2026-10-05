fn main() {
    // Link order matters: link.x INCLUDEs memory.x, so the shared-region
    // fragment (which names SRAM_SHARED) has to come after it.
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tshared.x");

    // Ship this crate's linker scripts. `memory.x` overrides the one metapac's
    // `memory-x` feature generates (which has no `RAM` region that qingke-rt's
    // link.x needs), and `shared.x` places the cross-core mailbox (see the `ipc`
    // crate) at the start of SRAM_SHARED. The current crate's build-script link
    // search path takes precedence over dependency build-script paths.
    let out_dir = std::env::var("OUT_DIR").unwrap();
    let out_dir = std::path::PathBuf::from(out_dir);
    std::fs::write(out_dir.join("memory.x"), include_bytes!("memory.x")).unwrap();
    std::fs::write(out_dir.join("shared.x"), include_bytes!("../ipc/shared.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=../ipc/shared.x");
}
