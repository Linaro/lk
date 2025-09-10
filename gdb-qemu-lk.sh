#! /bin/sh

#! /usr/bin/env nix-shell
#! nix-shell -i bash -p qemu

echo READY TO CONNECT WITH GDB
qemu-system-aarch64 \
	-machine virt \
	-m 1024M \
	-cpu cortex-a57 \
	-nographic \
	-kernel build-qemu-virt-arm64-test/lk.elf \
    -S -s
