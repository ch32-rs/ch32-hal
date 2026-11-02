fn main() {
    for name in ["dma", "dma_v4x7", "ch32v4", "sdio_v4x7"] {
        println!("cargo:rustc-check-cfg=cfg({name})");
        println!("cargo:rustc-cfg={name}");
    }
}
