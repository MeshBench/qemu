#!/usr/bin/env bash

set -euo pipefail

TARGET=${TARGET:-xtensa-softmmu}
VERSION=${VERSION:-dev}

echo DBG
./configure --help

# The Windows build, cross-compiled on a Linux runner with MinGW. It replaces
# an msys2 build on a windows-2022 runner that produced the same archive; the
# binary is the same triplet either way, and this one does not need a Windows
# runner to make it.
#
# --disable-werror: QEMU's configure turns -Werror on by itself for a git
# build, and a newer MinGW GCC than the tree was written against warns on
# QEMU's own test sources - tests this build does not run taking down the
# Windows emulator nobody could otherwise have. The msys2 script this replaces
# recorded the same trap.
#
# No SDL, no slirp: MeshBench drives QEMU headless over sockets and never asks
# for a window or for user networking, and each linked library is one more
# thing a downloaded emulator can fail to start without.
#
# Not --static, which the msys2 build used: Fedora's mingw64 packages ship
# DLLs, not static archives, so the libraries travel beside the binary in the
# archive instead. bundle-cross-win-dlls.sh puts them there, and the Package
# step fails if one is missing.
./configure \
    --bindir=bin \
    --cross-prefix=x86_64-w64-mingw32- \
    --datadir=share/qemu \
    --disable-sdl \
    --disable-slirp \
    --disable-werror \
    --enable-gcrypt \
    --enable-pixman \
    --enable-stack-protector \
    --prefix=${PWD}/install/qemu \
    --target-list=${TARGET} \
    --with-pkgversion="${VERSION}" \
    --with-suffix="" \
    --without-default-features \
|| { cat meson-logs/meson-log.txt && false; }

# The esp32 machine references an RSA device gated on gcrypt, so a build
# without it configures, compiles, links, and then dies at machine
# instantiation with "unknown type 'misc.esp32.rsa'" - a long way from the
# cause. Checked here, where the answer is one grep, rather than on a user's
# machine.
# meson writes a bare "#define CONFIG_GCRYPT" for a boolean that is on, with
# no value after it, so match the name and not a 1.
grep -qE '^#define CONFIG_GCRYPT($|[[:space:]])' build/config-host.h || {
    echo "gcrypt is off in this build; the esp32 machine will not instantiate" >&2
    exit 1
}
