//! Rust meta-crate for LK

#![no_std]

/*
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
*/

#[unsafe(no_mangle)]
extern "C" fn setup_rust() {
    lk::init();
}
