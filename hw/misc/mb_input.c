/*
 * The board's buttons, driven from outside.
 *
 * A board's lamps are outputs somebody watches; its buttons are inputs
 * somebody drives, and until now nothing could drive them. This holds one GPIO
 * line per declared button and moves it when a message arrives on a socket, so
 * the application that draws the board can also press it.
 *
 * Press and release rather than click, because the difference matters: a
 * Heltec V3 powers itself off when its program button is held, and a control
 * that could only produce a tap could never reproduce that - nor wake the
 * screen the same firmware puts to sleep.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qemu/main-loop.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define TYPE_MB_INPUT "mb-input"
OBJECT_DECLARE_SIMPLE_TYPE(MBInputState, MB_INPUT)

/* Enough for a trackball's four directions and a handful of buttons. A board
 * needing more than this is a board worth looking at rather than a limit worth
 * raising quietly. */
#define MB_INPUT_MAX 8

/* Every message on this socket is eight bytes: a tag and seven of payload.
 * Fixed width because more than one kind of device listens here - buttons, a
 * keyboard, a touch panel - and each has to be able to skip what is not its
 * own without knowing how long it was.
 *
 * For a button the payload is the pin and the level. The pin rather than an
 * index into the list, so the sender never has to keep in step with the order
 * the lines were declared in. */
#define MB_INPUT_MSG 8
#define MB_INPUT_TAG 'B'

struct MBInputState {
    SysBusDevice parent_obj;

    char *path;
    char *pins; /* comma separated, in the order the lines are declared */

    int fd;
    int npins;
    int pin[MB_INPUT_MAX];
    qemu_irq line[MB_INPUT_MAX];

    uint8_t buf[MB_INPUT_MSG];
    int have;
};

static void mb_input_read(void *opaque)
{
    MBInputState *s = MB_INPUT(opaque);

    for (;;) {
        ssize_t n = recv(s->fd, s->buf + s->have, MB_INPUT_MSG - s->have, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            n = 0;
        }
        if (n == 0) {
            /* The other end went away. Leave every line where it is rather
             * than releasing: a button that springs back when the interface
             * closes would look like somebody let go. */
            qemu_set_fd_handler(s->fd, NULL, NULL, NULL);
            close(s->fd);
            s->fd = -1;
            return;
        }
        s->have += (int)n;
        if (s->have < MB_INPUT_MSG) {
            return;
        }
        s->have = 0;
        if (s->buf[0] != MB_INPUT_TAG) {
            continue;
        }
        int pin = s->buf[1];
        int level = s->buf[2] ? 1 : 0;
        for (int i = 0; i < s->npins; i++) {
            if (s->pin[i] == pin) {
                qemu_set_irq(s->line[i], level);
                break;
            }
        }
    }
}

static void mb_input_connect(MBInputState *s)
{
    struct sockaddr_un addr;

    if (!s->path || !*s->path) {
        return;
    }
    if (strlen(s->path) >= sizeof(addr.sun_path)) {
        warn_report("mb-input: socket path is too long");
        return;
    }
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        return;
    }
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    pstrcpy(addr.sun_path, sizeof(addr.sun_path), s->path);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 &&
        errno != EINPROGRESS) {
        close(fd);
        return;
    }
    s->fd = fd;
    qemu_set_fd_handler(fd, mb_input_read, NULL, s);
}

static void mb_input_realize(DeviceState *dev, Error **errp)
{
    MBInputState *s = MB_INPUT(dev);

    s->fd = -1;
    s->npins = 0;
    if (s->pins && *s->pins) {
        const char *p = s->pins;
        while (*p && s->npins < MB_INPUT_MAX) {
            char *end = NULL;
            long v = strtol(p, &end, 10);
            if (end == p) {
                break;
            }
            s->pin[s->npins++] = (int)v;
            p = (*end == ',') ? end + 1 : end;
        }
    }
    if (s->npins == 0) {
        error_setg(errp, "mb-input: no pins named, so nothing could be pressed");
        return;
    }
    qdev_init_gpio_out_named(dev, s->line, "mb-input", s->npins);
    mb_input_connect(s);
}

static Property mb_input_props[] = {
    DEFINE_PROP_STRING("path", MBInputState, path),
    DEFINE_PROP_STRING("pins", MBInputState, pins),
    DEFINE_PROP_END_OF_LIST(),
};

static void mb_input_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mb_input_realize;
    device_class_set_props(dc, mb_input_props);
}

static const TypeInfo mb_input_info = {
    .name = TYPE_MB_INPUT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MBInputState),
    .class_init = mb_input_class_init,
};

static void mb_input_register_types(void)
{
    type_register_static(&mb_input_info);
}

type_init(mb_input_register_types)
