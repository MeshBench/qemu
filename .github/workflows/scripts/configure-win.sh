#!/usr/bin/env bash

set -euo pipefail

TARGET=${TARGET:-xtensa-softmmu}
VERSION=${VERSION:-dev}

echo DBG
./configure --help

# --disable-werror, and no SDL. Removing --extra-cflags=-Werror was not
# enough: QEMU's configure turns -Werror on by itself for a git build, and
# it fails on an upstream test
# file (tests/qtest/libqtest.c: 'idx' set but not used) that a newer mingw
# GCC warns about - QEMU's own tests, which this build does not run, taking
# down the Windows emulator nobody could otherwise have. SDL would make the
# binary need a display library it never opens.
./configure \
    --bindir=bin \
    --disable-werror \
    --datadir=share/qemu \
    --enable-gcrypt \
    --disable-sdl \
    --enable-pixman \
    --enable-slirp \
    --enable-stack-protector \
    --prefix=${PWD}/install/qemu \
    --static \
    --target-list=${TARGET} \
    --with-pkgversion="${VERSION}" \
    --with-suffix="" \
    --without-default-features \
|| { cat meson-logs/meson-log.txt && false; }


# Fix: pkg-config for libgcrypt outputs incorrect paths for libiconv and libintl:
# - Unix-style paths (/mingw64/lib/...) instead of Windows paths (D:/a/_temp/msys64/mingw64/lib/...)
# - Dynamic import libraries (.dll.a) instead of static libraries (.a)
# We need to fix both issues in build.ninja for the static build to work correctly.
MSYS_BASE=$(cygpath -w / | sed 's/\\/\//g')
sed -i "s|/mingw64/lib/libintl.dll.a|${MSYS_BASE}/mingw64/lib/libintl.a|g; s|/mingw64/lib/libiconv.dll.a|${MSYS_BASE}/mingw64/lib/libiconv.a|g" build/build.ninja
