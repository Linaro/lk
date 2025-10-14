#! /bin/sh

# This is the build within vanilla Ubuntu in a container
gmake \
	qemu-virt-arm64-test \
	'CC=/usr/bin/clang --target=aarch64-unknown-elf' \
	'CPP=/usr/bin/clang-cpp-14 --target=aarch64-unknown-elf' \
	'CXX=/usr/bin/clang++ --target=aarch64-unknown-elf' \
	'LD=/usr/bin/ld.lld' \
	CPPFILT=/usr/bin/llvm-cxxfilt-14 \
	TOOLCHAIN_PREFIX=/usr/bin/llvm- \
	-j 12 \
    build-qemu-virt-arm64-test/lk.elf
    # -j1



# With nix, all will be set, and we can just build the given target.
# make -j 16 qemu-virt-arm64-test |& tee LOG
