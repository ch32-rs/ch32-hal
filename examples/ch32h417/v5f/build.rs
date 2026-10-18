fn main() {
    // Link order matters: link.x INCLUDEs memory.x, so the shared-region
    // fragment (which names SRAM_SHARED) has to come after it.
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tshared.x");

    // Ship this crate's linker scripts. qingke-rt's link.x wants `memory.x` (our
    // V5F FLASH/RAM/DTCM layout, overriding the one metapac generates) and
    // `device.x` (the svd2rust device-crate hook, which a HAL-free crate provides
    // empty); `shared.x` *reserves* SRAM_SHARED so nothing of ours lands there —
    // the mailbox itself is defined by the boot core, not by this image.
    let out_dir = std::env::var("OUT_DIR").unwrap();
    let out_dir = std::path::PathBuf::from(out_dir);
    std::fs::write(out_dir.join("memory.x"), include_bytes!("memory.x")).unwrap();
    std::fs::write(out_dir.join("device.x"), include_bytes!("device.x")).unwrap();
    std::fs::write(out_dir.join("shared.x"), include_bytes!("../ipc/shared.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=device.x");
    println!("cargo:rerun-if-changed=../ipc/shared.x");
}
