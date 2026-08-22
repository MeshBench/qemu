/*
 * GT911 capacitive touch panel, as an I2C slave.
 *
 * A register file with a 16-bit address: the driver writes the address it
 * wants and then reads from it. Two things are read in practice - the product
 * identifier, which is how a driver decides the panel is there at all, and the
 * status byte followed by the coordinates of whatever is being touched.
 *
 * Only one point is reported. A real panel tracks five, and modelling the
 * others would mean inventing gestures nobody made: the interface this is
 * driven from has one pointer.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define TYPE_GT911_TOUCH "gt911-touch"
OBJECT_DECLARE_SIMPLE_TYPE(GT911State, GT911_TOUCH)

#define GT_MSG 8
#define GT_TAG 'T'

/* The registers a driver actually reads. */
#define GT_REG_ID     0x8140 /* four bytes, "911" and a nul */
#define GT_REG_STATUS 0x814E /* bit 7 set means a report is ready */
#define GT_REG_POINT  0x8150 /* track id, x, y, size, reserved */

struct GT911State {
    I2CSlave parent_obj;

    char *path;
    int fd;

    /* Where the driver's next read will come from, and how far into it. */
    uint16_t addr;
    int addr_bytes;
    int at;

    /* What is being touched, if anything. */
    bool down;
    uint16_t x, y;

    uint8_t buf[GT_MSG];
    int have;
};

static void gt911_read_socket(void *opaque)
{
    GT911State *s = GT911_TOUCH(opaque);

    for (;;) {
        ssize_t n = recv(s->fd, s->buf + s->have, GT_MSG - s->have, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            n = 0;
        }
        if (n == 0) {
            qemu_set_fd_handler(s->fd, NULL, NULL, NULL);
            close(s->fd);
            s->fd = -1;
            return;
        }
        s->have += (int)n;
        if (s->have < GT_MSG) {
            return;
        }
        s->have = 0;
        if (s->buf[0] != GT_TAG) {
            continue;
        }
        s->x = (uint16_t)(s->buf[1] | s->buf[2] << 8);
        s->y = (uint16_t)(s->buf[3] | s->buf[4] << 8);
        s->down = s->buf[5] != 0;
    }
}

static void gt911_connect(GT911State *s)
{
    struct sockaddr_un addr;

    if (s->fd >= 0 || !s->path || !*s->path) {
        return;
    }
    if (strlen(s->path) >= sizeof(addr.sun_path)) {
        warn_report("gt911: socket path is too long");
        s->path = NULL;
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
    qemu_set_fd_handler(fd, gt911_read_socket, NULL, s);
}

static uint8_t gt911_at(GT911State *s, uint16_t reg, int off)
{
    static const uint8_t id[4] = {'9', '1', '1', 0};

    if (reg >= GT_REG_ID && reg < GT_REG_ID + 4) {
        return id[reg - GT_REG_ID];
    }
    switch (reg) {
    case GT_REG_STATUS:
        /* Bit 7 says a report is waiting, the low bits how many points are
         * in it. Nothing touching is a valid report of zero points, which is
         * what stops a driver waiting for ever. */
        return s->down ? 0x81 : 0x80;
    case GT_REG_POINT + 0:
        return 0; /* track id */
    case GT_REG_POINT + 1:
        return s->x & 0xff;
    case GT_REG_POINT + 2:
        return s->x >> 8;
    case GT_REG_POINT + 3:
        return s->y & 0xff;
    case GT_REG_POINT + 4:
        return s->y >> 8;
    case GT_REG_POINT + 5:
        return s->down ? 20 : 0; /* contact size */
    case GT_REG_POINT + 6:
        return 0;
    default:
        (void)off;
        return 0;
    }
}

static uint8_t gt911_recv(I2CSlave *i2c)
{
    GT911State *s = GT911_TOUCH(i2c);
    uint8_t v = gt911_at(s, (uint16_t)(s->addr + s->at), s->at);
    s->at++;
    return v;
}

static int gt911_send(I2CSlave *i2c, uint8_t data)
{
    GT911State *s = GT911_TOUCH(i2c);

    if (s->addr_bytes == 0) {
        s->addr = (uint16_t)(data << 8);
        s->addr_bytes = 1;
        return 0;
    }
    if (s->addr_bytes == 1) {
        s->addr |= data;
        s->addr_bytes = 2;
        s->at = 0;
        return 0;
    }
    /* Writing past the address is the driver clearing the status flag, which
     * it does after reading a report. Nothing here holds a flag to clear -
     * the state is whatever the pointer is doing now - so this is accepted
     * and ignored rather than refused. */
    return 0;
}

static int gt911_event(I2CSlave *i2c, enum i2c_event event)
{
    GT911State *s = GT911_TOUCH(i2c);

    if (event == I2C_START_SEND) {
        s->addr_bytes = 0;
    }
    if (event == I2C_START_RECV) {
        s->at = 0;
    }
    return 0;
}

static void gt911_realize(DeviceState *dev, Error **errp)
{
    GT911State *s = GT911_TOUCH(dev);

    s->fd = -1;
    s->addr_bytes = 0;
    s->at = 0;
    s->down = false;
    gt911_connect(s);
}

static Property gt911_props[] = {
    DEFINE_PROP_STRING("path", GT911State, path),
    DEFINE_PROP_END_OF_LIST(),
};

static void gt911_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = gt911_realize;
    device_class_set_props(dc, gt911_props);
    sc->recv = gt911_recv;
    sc->send = gt911_send;
    sc->event = gt911_event;
}

static const TypeInfo gt911_info = {
    .name = TYPE_GT911_TOUCH,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(GT911State),
    .class_init = gt911_class_init,
};

static void gt911_register_types(void)
{
    type_register_static(&gt911_info);
}

type_init(gt911_register_types)
