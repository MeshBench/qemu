#!/usr/bin/env bash
#
# Put the DLLs a cross-built qemu-system-*.exe needs beside it.
#
# The msys2 build this replaces linked statically and needed almost nothing.
# Fedora's mingw64 packages ship DLLs, so the archive carries them instead -
# glib, pixman, gcrypt, and the GCC runtime behind them. Without this the exe
# starts and Windows kills it with a missing-DLL box, which for somebody who
# just downloaded MeshBench is indistinguishable from MeshBench being broken.
#
# Imports are walked recursively: libgcrypt pulls libgpg-error, glib pulls
# libintl and libiconv and pcre2, and none of that is visible from the exe's
# own import table alone.

set -euo pipefail

BIN_DIR="${1:-install/qemu/bin}"
SYSROOT_BIN="/usr/x86_64-w64-mingw32/sys-root/mingw/bin"
OBJDUMP="${OBJDUMP:-x86_64-w64-mingw32-objdump}"

# The GCC runtime (libgcc_s_seh-1.dll, and libstdc++ if anything pulls it) sits
# with the compiler rather than in the sysroot.
GCC_LIB_DIR="$(dirname "$(x86_64-w64-mingw32-gcc -print-libgcc-file-name)")"

search_dirs=("${SYSROOT_BIN}" "${GCC_LIB_DIR}")

find_dll() {
    local name="$1" d
    for d in "${search_dirs[@]}"; do
        # Windows import tables are not consistent about case.
        local hit
        hit=$(find "$d" -maxdepth 1 -iname "$name" -print -quit 2>/dev/null || true)
        [ -n "$hit" ] && { printf '%s\n' "$hit"; return 0; }
    done
    return 1
}

imports_of() {
    "${OBJDUMP}" -p "$1" | awk '/DLL Name:/ { print $3 }'
}

echo "bundling DLLs into ${BIN_DIR}"
queue=()
for exe in "${BIN_DIR}"/*.exe; do
    [ -f "$exe" ] && queue+=("$exe")
done
[ ${#queue[@]} -gt 0 ] || { echo "no .exe in ${BIN_DIR}" >&2; exit 1; }

declare -A seen=()
system_only=()
while [ ${#queue[@]} -gt 0 ]; do
    next=()
    for f in "${queue[@]}"; do
        for dll in $(imports_of "$f"); do
            key=$(printf '%s' "$dll" | tr '[:upper:]' '[:lower:]')
            [ -n "${seen[$key]:-}" ] && continue
            seen[$key]=1
            if src=$(find_dll "$dll"); then
                cp -v "$src" "${BIN_DIR}/$(basename "$src")"
                next+=("${BIN_DIR}/$(basename "$src")")
            else
                # Not ours to ship: KERNEL32, ADVAPI32, ws2_32 and the rest
                # come with Windows, and copying a Fedora stub of one would be
                # worse than leaving it to the operating system.
                system_only+=("$dll")
            fi
        done
    done
    queue=("${next[@]:-}")
    queue=($(printf '%s\n' "${queue[@]}" | sed '/^$/d'))
done

echo
echo "bundled:"
ls -la "${BIN_DIR}"
echo
echo "left to Windows itself: ${system_only[*]:-none}"

# Anything the sysroot had and we failed to copy would show up here as an
# import with no file beside the exe. Checked rather than assumed: the whole
# point of this script is that the archive runs on a machine that has never
# had a toolchain on it.
missing=0
for f in "${BIN_DIR}"/*.exe "${BIN_DIR}"/*.dll; do
    [ -f "$f" ] || continue
    for dll in $(imports_of "$f"); do
        if [ ! -e "${BIN_DIR}/${dll}" ] && find_dll "$dll" >/dev/null; then
            echo "::error::${dll}, needed by $(basename "$f"), exists in the sysroot but was not bundled"
            missing=1
        fi
    done
done
exit "${missing}"
