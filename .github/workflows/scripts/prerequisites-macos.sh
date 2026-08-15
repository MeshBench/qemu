#!/usr/bin/env bash

set -euo pipefail

brew install \
  glib \
  libgcrypt \
  libslirp \
  ninja \
  pixman \
  pkg-config \
  sdl2 \
&& :

# workaround if deprecated module 'distutils.version' is missing
# https://peps.python.org/pep-0632/#migration-advice
# https://pypi.org/project/looseversion/
#
# Used for building only, not for qemu run-time
#
# Also if something goes wrong, 'setup-python' action can be used
# as a general solution, https://github.com/actions/runner-images/issues/8932#issuecomment-1836013315)
PYFIX_FILE=/usr/local/Cellar/glib/2.78.1/share/glib-2.0/codegen/utils.py
if [ -f "${PYFIX_FILE}" ] ; then
  echo "Fixing ${PYFIX_FILE}"
  python3 -m pip install --upgrade pip
  python3 -m pip install looseversion

  sed -i '' "s/distutils.version/looseversion/" "${PYFIX_FILE}"
fi

echo "Installing meson and tomli"
# --break-system-packages exists to bypass PEP 668's externally-managed marker,
# and pip only grew the option in 23.0. The Apple Silicon runner ships Xcode's
# python3, whose pip is older than that and which sets no such marker - so the
# flag is both unrecognised and unnecessary there. Passing it unconditionally
# failed the aarch64-apple-darwin legs of every release before the build had
# compiled a line.
pipflags=(--user)
if python3 -m pip install --help 2>/dev/null | grep -q -- --break-system-packages ; then
  pipflags+=(--break-system-packages)
fi
python3 -m pip install "${pipflags[@]}" meson==1.7.0 tomli==2.2.1

# dbg
command -v python3
python3 --version
python3 -m pip freeze
