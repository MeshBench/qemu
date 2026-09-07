#!/usr/bin/env python3
"""Every device the ESP32 clock handler writes to must take the write.

esp32_clk_update() runs when the second-stage bootloader switches to the PLL -
long after every device has been realized - and sets "apb_freq" on the UARTs,
the FRC timers and the timer groups. A property declared with DEFINE_PROP_* is
a static qdev property, and qdev refuses those after realize:

    Attempt to set property 'apb_freq' on anonymous device
    (type 'esp_soc.uart') after it was realized

That is not an error the machine handles. It aborts the emulator at hand-over,
which took every ESP32 board on every platform down at the same instant, and
the console said nothing because the board had not printed anything yet.

So the invariant is checkable and worth checking: bring the machine up stopped,
and set the property from QMP on each of those devices. The QMP path runs the
same qdev guard the machine's own write runs into, so a static property fails
here exactly as it fails there - in under a second, with no firmware, and
naming the device that regressed.
"""

import json
import subprocess
import sys

# The devices esp32_clk_update() writes apb_freq to, by their place in the QOM
# tree. Kept as a list rather than discovered, because a device that stops
# appearing under these names is itself the regression.
DEVICES = ["uart0", "uart1", "uart2", "frc0", "frc1", "timg0", "timg1"]
PROPERTY = "apb_freq"

# 80 MHz: the APB with the PLL up, which is the value the machine actually
# writes. The point is whether the write is accepted, not what it carries.
VALUE = 80000000


def main() -> int:
    qemu = sys.argv[1] if len(sys.argv) > 1 else "./build/qemu-system-xtensa"

    script = [{"execute": "qmp_capabilities"}]
    script += [
        {
            "execute": "qom-set",
            "arguments": {
                "path": "/machine/soc/" + dev,
                "property": PROPERTY,
                "value": VALUE,
            },
            "id": dev,
        }
        for dev in DEVICES
    ]
    script.append({"execute": "quit"})

    # -S so nothing executes: the devices exist and are realized, which is the
    # only state this asks about. Three null serials because the machine wires
    # three ports and QMP has taken stdio.
    proc = subprocess.run(
        [
            qemu, "-machine", "esp32", "-display", "none",
            "-serial", "null", "-serial", "null", "-serial", "null",
            "-S", "-monitor", "none", "-qmp", "stdio",
        ],
        input="\n".join(json.dumps(c) for c in script),
        capture_output=True, text=True, timeout=120,
    )

    failed = []
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        reply = json.loads(line)
        if "error" in reply:
            failed.append((reply.get("id", "?"), reply["error"]["desc"]))

    if failed:
        print("a device the clock handler writes to refuses the write:", file=sys.stderr)
        for dev, desc in failed:
            print(f"  {dev}: {desc}", file=sys.stderr)
        print(
            "\nDeclare the property with object_property_add in the device's "
            "instance_init, as esp32_frc_timer and esp32_timg do, rather than "
            "with DEFINE_PROP_* in its property table.",
            file=sys.stderr,
        )
        return 1

    print(f"{len(DEVICES)} devices take {PROPERTY} after realize")
    return 0


if __name__ == "__main__":
    sys.exit(main())
