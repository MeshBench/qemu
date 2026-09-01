#!/usr/bin/env bash

set -euo pipefail

# Windows is cross-compiled from Linux, not built on a Windows runner.
#
# Fedora rather than the Debian the other Linux legs use: Debian ships the
# MinGW compiler and none of QEMU's dependencies built for it, while Fedora
# ships mingw64- builds of glib2, pixman and libgcrypt. QEMU's own CI
# cross-builds Windows from exactly this base - see
# tests/docker/dockerfiles/fedora-win64-cross.docker, which this list follows.
#
# The image tag is deliberately not pinned. A pinned Fedora reaches end of
# life, its mirrors go with it, and a release job that worked for months
# starts failing in dnf for a reason that has nothing to do with QEMU. The
# cost of tracking is that a newer GCC finds new warnings, which is why the
# configure for this leg turns -Werror off.
#
# No mingw64-libslirp: Fedora does not package one, which is why QEMU's own
# Windows cross image has no slirp either. MeshBench never passes -netdev or
# -nic - the radio is a chardev socket - so user networking is not something
# this archive is missing, and the configure disables it explicitly rather
# than letting the feature detection decide quietly.

dnf -y install \
    bison \
    bzip2 \
    diffutils \
    findutils \
    flex \
    gcc \
    git \
    make \
    meson \
    mingw-w64-tools \
    mingw64-gcc \
    mingw64-gettext \
    mingw64-glib2 \
    mingw64-libgcrypt \
    mingw64-pixman \
    mingw64-winpthreads \
    mingw64-zlib \
    ninja-build \
    python3 \
    python3-pip \
    tar \
    which \
    xz

# mingw64-filesystem, which mingw64-gcc pulls in, is what provides this - and
# QEMU's configure looks it up by exactly this name once --cross-prefix is
# set. If it is absent, every dependency reads as missing and configure fails
# with a list of libraries that are in fact installed.
command -v x86_64-w64-mingw32-pkg-config >/dev/null || {
    echo "x86_64-w64-mingw32-pkg-config is missing; every mingw dependency will read as absent" >&2
    exit 1
}
x86_64-w64-mingw32-gcc --version | head -1
