# Device register access from Rust

There are multiple ways of accessing device registers from within Rust.  These
have different tradeoffs, but more deeply, also deeply differing philosophical
differences behind their design.  This document will go through some of these,
in an exploration of the best way to access devices from within littlekernel
drivers.

## Existing access

Within many operating systems, devices are accessed by combining a base address
with an offset, and using macros or functions provided to read and write from
these values.  The functions take care of the volatile and memory ordering
requirements of accessing the values.

Probably the most usable of these would be the accessors used within the Linux
kernel (from C), which provides a family of macros for accessing registers:

    readl()
    writel()
    readl_relaxed()
    writel_releaxed()

## littlekernel C accessors

In littlekernel there are macros mmio_read32(), mmio_write32() and related
macros.  These are generally arch specific in implementation, as C does not
standardize how to access registers (volatile only solves part of the problem,
there are compiler barriers and memory barriers).  At least on aarch64, lk seems
to implement these with compile barriers, and does not issue memory barrier
operations as part of the I/O.

# Access from Rust

It isn't difficult to provide functions in rust, similar to `mmio_read32` and
`mmio_write32` that work from Rust. These functions are inherently unsafe, as
they work with direct pointer accesses, but this allows for a fairly
straightforward translation of a C driver into Rust, which makes a good starting
point.

But, there is also little benefit to writing the driver in Rust, when the code
must contain numerous unsafe blocks.

## Existing solutions

The rust-embedded community has numerous solutions to this problem, some are
part of various projects to build peripheral access crates (PAC), which are
intended to abstract the registers of an entire SoC. The intent of these is that
beyond initialization, the PAC types allow for safe and unsafe registers, and
these registers usually contain types that address the bitfield aspects of
various registers.

In addition, there are a dozen or so other crates that attempt to be more
general, allowing for easy definition of a set of registers, resulting in a type
to allow for safe access to those particular registers.

### derive-mmio, and safe-mmio

These two crates are similar (and slightly competing) approaches to the problem.
The register block is represented as a regular struct, possibly with special
types for the fields. `derive-mmio` uses proc macros to generate code from a
regular type of struct, whereas `safe-mmio` provides `field` macro for
converting from the register definition to the accessor for an individual
register.

Both of these also suffer from a largely fatal problem.  Within each crate,
there is a write method, that allows a register to be modified:

    fn write(&mut self, value: u32);

At first, it seems obvious that the write method to a register block should take
a `&mut self` as a parameter, after all, it does modify the hardware when it
writes (and ignores the issue that many hardware registers also modify the
hardware when read from).

However, in practice, the rules behind mut references within rust greatly
restrict how these registers can be used. Rust requires that, at any given
point, the entire system can have only a single mutable reference to a given
item. What this means is that in order to write to a register, some kind of
exclusive locking mechanism is needed (see `lk::SpinLock`). Although this works,
it introduces a spinlock (smp) and an interrupt lock for every register access.
It also makes it difficult for an irq handler to get access to the registers
(globals in Rust can't, in general, be mutable unless protected by some kind of
synchronization mechanism.)

Real hardware registers, on the other hand are often designed specifically to be
usable from multiple contexts in just this way. For a uart, the transmit and
receive registers might be completely separate.  For a gpio controller, it is
common to have one register that sets gpio pins, and another that clears them.
Combined with MMU mappings that specify "device" or "strongly ordered" memory
mappings, these devices can freely be written to from multiple contexts.

The problem is a bit deeper, however. The issue of memory safety here isn't due
to whether a particular device coordinates uncoordinated writes or not.
Regardless of the hardware itself, this is an issue of proper operation of a
given piece of hardware, which is the responsibility of the driver itself, not
the language. As such, I believe (strongly) that having these write methods
require a mutable reference to the device, for the most part, makes these types
of register accesses mostly useless.

One way to think of this is with something that rust called interior mutability.
Although rust has this rule to enforce exclusive write access at compile time,
it can be bypassed as needed by something that provides interior mutability. Due
to memory safety, these mechanisms usually also provide some type of exclusive
use (Mutex, Spinlock, etc).  This is necessary for these items to truly be
shared across contexts.

However, hardware block don't have these same constraints. Whether a given set
of interleaving of device registers is proper is a matter of the semantics of
that device. Regardless, it is never "unsafe" in the Rust sense. Even if a
device is used incorrectly, the semantics of that incorrect usage are still
sound. Trying to represent a subset of devices ordering constraints through the
`&mut self` argument to write is misguided at best, and generally results in
drivers with significant inefficiencies.

### rust2svd and chiptool

These are two utilities that are able to read an SVD file (used to describe Arm
register layouts), and generates a peripheral access crate that abstracts the
hardware. Generally, these are fairly complete, and represent not just the
registers, but individual bits.  These tools have taken the position that the
write methods should not require a mutable reference, and have been used to
develop a fairly complete set of HAL drivers for this SoC.

The PACs for these devices do have one significant problem, however. They
currently represent a given peripheral by casting the address of that peripheral
to a simple `&` reference. Unfortunately, for Rust, references are only sound
when they are actually references to memory that obeys Rust's expected
semantics.  The main consequence of this is that there is nothing to stop some
optimization from inserting it's own reads from this memory, beyond those done
explicitly through the volatile accessors.

# Where to go

Unfortunately, this means that for register access within LK, there isn't really
a good workable solution that already exists.  It might be possible to fork
safe-mmio, or mmio-derive, and "fix" the accessor issue on the write.

Another approach is to create a more minimalist PAC type of accessor that,
instead of providing customized generated accessors at the bit level, it
provides accessors that simply abstract the pointer arithmetic for the
underlying devices. As long as the pointers are hidden (and not turned into
references), the access should be sound when used from safe rust, and it can be
written in a way such that no mutable "self" pointer is needed.  In fact, it is
likely that the best representation of the registers will be as a simple type
wrapper around a bare pointer. The pointer itself is not accessible, preventing
unsound memory access, and can only be generated and accessed from an initial
constructor. This wrapper type will have read reference semantics, which can
simply be implemented by implementing Copy and Clone for the type.

What seems to work well for the pl011 driver is to use derive_mmio, and build an
owned instance of it for each context we wish to use it. This creation is unsafe
(as it can't guarantee that our use of the device is correct), but it does allow
for write access to the device registers.
