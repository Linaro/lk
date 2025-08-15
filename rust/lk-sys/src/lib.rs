//! Rust support for LK

#![no_std]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use core::ffi::c_char;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

unsafe impl Sync for lk_init_struct {}

// lk_init_level constants have large gaps between them and some modules
// add or subtract from these constants to indicate that it wants to run
// right before or after other init hooks at a given level. Add add and sub
// functions to lk_init_level to allow this for rust init hooks as well.
impl lk_init_level {
    pub const fn add(mut self, rhs: uint) -> Self {
        self.0 += rhs;
        self
    }
    pub const fn sub(mut self, rhs: uint) -> Self {
        self.0 -= rhs;
        self
    }
}

impl lk_init_struct {
    pub const fn new(
        level: lk_init_level,
        flags: lk_init_flags,
        hook: unsafe extern "C" fn(uint),
        name: *const c_char,
    ) -> Self {
        lk_init_struct { level: level.0 as u16, flags: flags.0 as u16, hook: Option::Some(hook), name }
    }
}

/*

// Return stderr.
#[inline(always)]
pub fn stderr() -> *mut sys::FILE {
    // SAFETY: `__stdio_FILEs` is defined as an `[]` array in C, which doesn't have a rust
    // equivalent. Bindgen generates a zero-element array.  However, Rust does require that static
    // symbols be at an address (even if zero sized) so this symbol will correspond to the first
    // element. We cast this to the first element so that we can then do pointer arithmetic to get
    // to subsequent elements.
    unsafe {
        let stdio = addr_of_mut!(sys::__stdio_FILEs) as *mut sys::FILE;
        stdio.add(2)
    }
}

// This needs to be referenced to make sure this code gets used.
pub fn init() {
    unsafe {
        sys::fputs(b"INIT: Rust lk_sys::init called\n\0".as_ptr() as *const c_char, stderr());
    }
}

extern "C" fn rust_hook(_level: c_uint) {
    unsafe {
        sys::fputs(b"INIT: rust_hook called\n\0".as_ptr() as *const c_char, stderr());
    }
}

// unsafe impl Sync for lk_init_struct { }

#[unsafe(link_section = "lk_init")]
#[used]
// #[no_mangle]  The mangled name is probably just fine
static SETUP_RUST_HOOK: sys::lk_init_struct = sys::lk_init_struct {
    level: sys::LK_INIT_LEVEL_PLATFORM as u16,
    flags: sys::LK_INIT_FLAG_PRIMARY_CPU as u16,
    hook: Some(rust_hook),
    name: "setup_rust\0".as_ptr() as *const c_char,
};
*/
