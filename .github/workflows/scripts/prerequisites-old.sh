#!/usr/bin/env bash

# ubuntu 18.04 for old glibc

set -euo pipefail

export DEBIAN_FRONTEND="noninteractive"

apt-get update -y -q
apt-get install -y -q --no-install-recommends software-properties-common
add-apt-repository ppa:deadsnakes/ppa
apt-get update -y -q
apt-get install -y -q --fix-missing --no-install-recommends python3.8 python3.8-venv python3.8-distutils python3.8-lib2to3
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 100
command -v python3
python3 -V
cd /usr/local/src
python3 -m venv envdir
source envdir/bin/activate
python3 -m pip install meson tomli

apt-get install -y -q --no-install-recommends \
    build-essential \
    git \
    libgcrypt-dev \
    libglib2.0-dev \
    libpixman-1-dev \
    libsdl2-dev \
    ninja-build \
    wget \
    zlib1g-dev \
&& :

# libslirp
libslirp_VER=v4.1.0
libslirp_DIST=libslirp-${libslirp_VER}.tar.bz2
libslirp_SHA256=f423c54c96eb3310bf9519abc8c9c11539801a14327b48b8f36acf407584bbd1
wget --no-verbose https://gitlab.com/qemu-project/libslirp/-/archive/${libslirp_VER}/${libslirp_DIST}
echo "${libslirp_SHA256} *${libslirp_DIST}" | sha256sum --check --strict -
tar -xf ${libslirp_DIST}
cd libslirp-${libslirp_VER}
meson build
ninja -C build install
rm -rf ${libslirp_DIST} libslirp-${libslirp_VER}
