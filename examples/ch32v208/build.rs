fn main() {
    // println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Path C: link the WCH BLE library for BLE_IPCoreInit and the BB IRQ sub-handler.
    println!("cargo:rustc-link-search=/Users/mono/Elec/WCH/CH32V20xEVT-2.31/EXAM/BLE/LIB");
    println!("cargo:rustc-link-lib=static=wchble");
    println!("cargo:rustc-link-arg=--undefined=BB_IRQLibHandler");
}
