fn main() {
    // The scripts are this crate's own: `memory.x` gives the single image the
    // whole 960 KiB of flash (no V5F slot to leave alone), `link.x` runs both
    // harts out of the zero-wait shared block. `shared.x` is shared with the
    // two-image examples (`ipc/`) so the mailbox's address cannot drift.
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tshared.x");

    let out_dir = std::env::var("OUT_DIR").unwrap();
    let out_dir = std::path::PathBuf::from(out_dir);
    std::fs::write(out_dir.join("memory.x"), include_bytes!("memory.x")).unwrap();
    std::fs::write(out_dir.join("link.x"), include_bytes!("link.x")).unwrap();
    std::fs::write(out_dir.join("shared.x"), include_bytes!("../ipc/shared.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rerun-if-changed=../ipc/shared.x");
}
