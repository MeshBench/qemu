#!/usr/bin/env bash

set -euo pipefail

# A self-hosted runner's service environment is not the shell a person gets,
# and Homebrew is routinely absent from its PATH. Look where it installs before
# concluding it is not there: /opt/homebrew on Apple Silicon, /usr/local on
# Intel. GITHUB_PATH rather than PATH alone, because the Configure step that
# needs ninja and pkg-config runs in a different shell from this one.
if ! command -v brew >/dev/null; then
  for p in /opt/homebrew/bin /usr/local/bin; do
    if [ -x "$p/brew" ]; then
      export PATH="$p:$PATH"
      [ -n "${GITHUB_PATH:-}" ] && echo "$p" >> "$GITHUB_PATH"
      break
    fi
  done
fi
if ! command -v brew >/dev/null; then
  echo "::error::brew is not on this runner and is not in either of the two"\
       "places it installs. Nothing below is installed, and the failure"\
       "surfaces three steps later as configure saying 'Cannot find Ninja'." >&2
  exit 1
fi

brew install \
  glib \
  libgcrypt \
  libslirp \
  ninja \
  pixman \
  pkg-config \
  sdl2 \
&& :

# brew install is allowed to come back non-zero for reasons that do not matter
# - a formula already installed, a warning about a link - so what is checked is
# the outcome rather than the exit code. Both of these are what configure looks
# for first, and their absence used to be reported by configure as its own
# failure several steps later.
missing=""
for t in ninja pkg-config; do command -v "$t" >/dev/null || missing="$missing $t"; done
if [ -n "$missing" ]; then
  echo "::error::brew ran and left these missing:$missing" >&2
  exit 1
fi

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
