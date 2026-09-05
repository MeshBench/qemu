#!/usr/bin/env bash
#
# Point this container's apt at the snapshot it was built from.
#
# Debian 11 is here for one reason: it carries the oldest glibc that still
# satisfies QEMU's glib2 requirement, so a binary built against it runs on
# everything newer. That is a deliberate floor rather than an oversight, and it
# is not moving until the floor moves.
#
# What moved is Debian's own hosting. Bullseye is past its end of life, so its
# security pool is no longer served from deb.debian.org: apt-get update
# succeeds against an index that is still published, and then the install fails
# on file after file with a 404. The build reads as a broken toolchain rather
# than as a mirror that no longer has these files.
#
# The image ships the answer commented out in its own sources.list. Every
# official Debian image carries snapshot.debian.org lines pinned to the date it
# was built, for exactly this: a suite that has stopped moving, served from an
# archive that keeps every version. Switching to them is a one-line edit and it
# is more reproducible than what it replaces, because the packages a build gets
# no longer depend on the day it runs.
set -euo pipefail

test -f /etc/apt/sources.list || {
    echo "no /etc/apt/sources.list: this is not the Debian image this expects" >&2
    exit 1
}

# The live lines out, the snapshot lines in. Order matters: commenting first
# would comment the ones just uncommented.
sed -i -e 's|^deb |#deb |' \
       -e 's|^#\s*deb http://snapshot.debian.org|deb http://snapshot.debian.org|' \
       /etc/apt/sources.list

grep -q '^deb http://snapshot.debian.org' /etc/apt/sources.list || {
    echo "this image carries no snapshot lines to fall back to:" >&2
    cat /etc/apt/sources.list >&2
    exit 1
}

# A snapshot's Release file is dated by definition and apt refuses a stale one
# by default. Refusing it here would be refusing the whole point of a snapshot.
echo 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99snapshot
