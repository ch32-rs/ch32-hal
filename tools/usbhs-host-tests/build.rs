fn main() {
    for instance in ["peri_usbhs1", "peri_usbhs2"] {
        println!("cargo:rustc-check-cfg=cfg({instance})");
        println!("cargo:rustc-cfg={instance}");
    }
    println!("cargo:rustc-check-cfg=cfg(afio_h4)");
}
