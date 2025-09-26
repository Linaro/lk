//! Rust PL011 uart driver for LK.

#![no_std]

use core::{
    cell::UnsafeCell,
    ffi::{c_char, c_int, c_void},
    mem, panic, ptr,
};
use lk::{cbuf::Cbuf, sys};

extern crate alloc;

unsafe extern "C" {
    static mut console_input_cbuf: sys::cbuf;
}

fn get_console_input_cbuf() -> Cbuf {
    unsafe { Cbuf::new(&raw mut console_input_cbuf) }
}

/// Init function to ensure crate is linked.
/// This might not be needed once we provide something actually linked.
pub fn init() {
    // Nothing to do yet.
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
enum Pl011UartRegs {
    Dr = 0x00,  // Data Register
    Rsr = 0x04, // Receive Status / Error Clear
    // 0x08 - reserved
    Tfr = 0x18,   // Flag Register
    Iplr = 0x20,  // Interrupt Priority Level Register
    Ibrd = 0x24,  // Integer Baud Rate Divisor
    Fbrd = 0x28,  // Fractional Baud Rate Divisor
    Lcrh = 0x2C,  // Line Control Register
    Cr = 0x30,    // Control Register
    Ifls = 0x34,  // Interrupt FIFO Level Select
    Imsc = 0x38,  // Interrupt Mask Set/Clear
    Tris = 0x3C,  // Raw Interrupt Status
    Tmis = 0x40,  // Masked Interrupt Status
    Icr = 0x44,   // Interrupt Clear Register
    Dmacr = 0x48, // DMA Control Register
}

const CONSOLE_HAS_INPUT_BUFFER: bool = true;
const PL011_FLAG_DEBUG_UART: u32 = 0x1;

const UART_TFR_RXFE: u32 = 1 << 4; // UART Receive FIFO Empty
const UART_TFR_TXFF: u32 = 1 << 5; // UART Transmit FIFO Full
// const UART_TFR_RXFF: u32 = 1 << 6; // UART Receive FIFO Full
// const UART_TFR_TXFE: u32 = 1 << 7; // UART Transmit FIFO Empty

const UART_IMSC_RXIM: u32 = 1 << 4; // Receive interrupt mask

const UART_TMIS_RXMIS: u32 = 1 << 4; // Receive masked interrupt status

const UART_CR_RXEN: u32 = 1 << 9; // Receive enable

/// Unsafe write to a UART register.
fn write_uart_reg(base: usize, reg: Pl011UartRegs, value: u32) {
    unsafe {
        ptr::write_volatile((base + reg as usize) as *mut u32, value);
    }
}

fn set_uart_reg_bits(base: usize, reg: Pl011UartRegs, bits: u32) {
    let val = read_uart_reg(base, reg);
    write_uart_reg(base, reg, val | bits);
}

fn clear_uart_reg_bits(base: usize, reg: Pl011UartRegs, bits: u32) {
    let val = read_uart_reg(base, reg);
    write_uart_reg(base, reg, val & !bits);
}

/// Unsafe read UART register
fn read_uart_reg(base: usize, reg: Pl011UartRegs) -> u32 {
    unsafe { ptr::read_volatile((base + reg as usize) as *const u32) }
}

// TODO: Don't make this manually.
#[repr(C)]
#[derive(Clone)]
#[allow(non_camel_case_types)]
struct pl011_config {
    base: usize,
    irq: u32,
    flag: u32,
}

/// Global state of the uart. Very _non_ rust friendly right now, just a
/// directly translation from the C code.
struct Pl011Uart {
    config: pl011_config,
    rx_buf: sys::cbuf,
}

unsafe impl Sync for Pl011Uart {}

struct SyncUnsafeCell<T>(UnsafeCell<T>);

unsafe impl<T> Sync for SyncUnsafeCell<T> {}

impl<T> SyncUnsafeCell<T> {
    const fn new(value: T) -> Self {
        SyncUnsafeCell(UnsafeCell::new(value))
    }

    #[allow(dead_code)]
    fn get(&self) -> *mut T {
        self.0.get()
    }

    #[allow(dead_code)]
    fn get_mut(&self) -> &mut T {
        unsafe { &mut *self.0.get() }
    }
}

const RXBUF_SIZE: usize = 32;

static PL011_UARTS: SyncUnsafeCell<Pl011Uart> = SyncUnsafeCell::new(unsafe { mem::zeroed() });

extern "C" fn uart_irq(arg: *mut c_void) -> sys::handler_return {
    let uart = unsafe { &mut *(arg as *mut Pl011Uart) };
    let base = uart.config.base;
    let buf = get_console_input_cbuf();

    // uart_pputc(0, b'I');

    let mut resched = false;

    let isr = read_uart_reg(base, Pl011UartRegs::Tmis);

    // Read characters from the fifo until it's empty.
    // TODO: This is wrong, and doesn't handle the cbuf overflow.
    if (isr & UART_TMIS_RXMIS) != 0 {
        while (read_uart_reg(base, Pl011UartRegs::Tfr) & UART_TFR_RXFE) == 0 {
            if CONSOLE_HAS_INPUT_BUFFER {
                if uart.config.flag & PL011_FLAG_DEBUG_UART != 0 {
                    let c = read_uart_reg(base, Pl011UartRegs::Dr) as c_char;
                    buf.write_char(c, false);
                    continue;
                }
            }

            // If we're out of rx buffer, mask the irq instead of handling it.
            if buf.is_full() {
                clear_uart_reg_bits(base, Pl011UartRegs::Imsc, UART_IMSC_RXIM);
                break;
            }
            let c = read_uart_reg(base, Pl011UartRegs::Dr) as c_char;
            buf.write_char(c, false);
        }

        resched = true;
    }

    if resched {
        sys::INT_RESCHEDULE
    } else {
        sys::INT_NO_RESCHEDULE
    }
}

/// Early initialization sets the config so that panic print messages can output
/// characters.
#[unsafe(no_mangle)]
extern "C" fn pl011_init_early(port: isize, config: *const pl011_config) {
    // TODO: Handle more than one port.
    let _ = port;

    unsafe {
        let uarts = PL011_UARTS.get_mut();

        uarts.config = (*config).clone();
    }
}

#[unsafe(no_mangle)]
extern "C" fn pl011_init(port: isize) {
    let _ = port;

    let uart = PL011_UARTS.get_mut();
    let base = uart.config.base;

    unsafe {
        sys::cbuf_initialize(&mut uart.rx_buf, RXBUF_SIZE);
    }

    // Assumes interrupts are contiguous
    unsafe {
        sys::register_int_handler(
            uart.config.irq,
            Some(uart_irq),
            uart as *mut Pl011Uart as *mut c_void,
        );
    }
    log::info!("pl011: registered irq {}", uart.config.irq);
    log::info!("pl011: block at {:p}", uart);
    log::info!("pl011: basse {:#x}", base);

    // Clear all irqs
    write_uart_reg(base, Pl011UartRegs::Icr, 0x3FF);

    // Set fifo trigger level
    write_uart_reg(base, Pl011UartRegs::Ifls, 0); // 1/8 full, 1/8 txfifo

    // Enable rx interrupt
    write_uart_reg(base, Pl011UartRegs::Imsc, UART_IMSC_RXIM);

    // Enable receive
    set_uart_reg_bits(base, Pl011UartRegs::Cr, UART_CR_RXEN);

    // Enable interrupt
    unsafe {
        sys::unmask_interrupt(uart.config.irq);
    }
}

#[unsafe(no_mangle)]
extern "C" fn uart_putc(port: c_int, c: c_char) {
    let _ = port;

    let uart = PL011_UARTS.get_mut();
    let base = uart.config.base;

    // Spin while fifo is full.
    while (read_uart_reg(base, Pl011UartRegs::Tfr) & UART_TFR_TXFF) != 0 {}
    write_uart_reg(base, Pl011UartRegs::Dr, c as u32);
}

#[unsafe(no_mangle)]
extern "C" fn uart_getc(port: c_int, wait: bool) -> c_int {
    let _ = port;

    let uart = PL011_UARTS.get_mut();
    let base = uart.config.base;

    let mut ch = 0;
    if unsafe { sys::cbuf_read_char(&mut uart.rx_buf, &mut ch, wait) } == 1 {
        set_uart_reg_bits(base, Pl011UartRegs::Imsc, UART_IMSC_RXIM);
        return ch as c_int;
    }

    -1
}

/// Panic-time putc.
#[unsafe(no_mangle)]
extern "C" fn uart_pputc(port: c_int, c: c_char) -> c_int {
    let _ = port;

    let uart = PL011_UARTS.get_mut();
    let base = uart.config.base;

    // Spin while fifo is full.
    while (read_uart_reg(base, Pl011UartRegs::Tfr) & UART_TFR_TXFF) != 0 {}
    write_uart_reg(base, Pl011UartRegs::Dr, c as u32);

    return 1;
}

#[unsafe(no_mangle)]
extern "C" fn uart_pgetc(port: c_int) {
    let _ = port;

    panic!("uart_pgetc not implemented");
}
