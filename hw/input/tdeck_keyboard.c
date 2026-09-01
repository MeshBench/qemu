/*
 * The T-Deck's keyboard, as an I2C slave.
 *
 * Not a matrix. On this board the keyboard is a second microcontroller with
 * its own firmware, and the application reads it by asking for one byte over
 * I2C: the character last pressed, or zero if nothing has been. That makes it
 * an easy thing to model, because we are modelling the answer rather than the
 * keyboard - which is also why what arrives here is already a character and
 * not a scan code.
 *
 * Keys arrive on the same socket the board's buttons do, tagged so each
 * device can skip what is not its own.
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

/* qemu/osdep.h brings the socket headers on both platforms: sys/socket.h
 * and sys/un.h on POSIX, winsock2 and afunix.h on Windows, where QEMU also
 * wraps socket(), connect(), send(), recv() and close() so that errno says
 * what a Winsock error meant. Including <sys/socket.h> here directly is what
 * stopped this file cross-compiling for Windows at all. */
#include "qemu/sockets.h"

#define TYPE_TDECK_KEYBOARD "tdeck-keyboard"
OBJECT_DECLARE_SIMPLE_TYPE(TDeckKbdState, TDECK_KEYBOARD)

#define KBD_MSG 8
#define KBD_TAG 'K'

/* A short queue rather than one byte. Somebody typing into the interface can
 * outrun a firmware that polls every few hundred milliseconds, and dropping
 * the middle of a word is worse than making them wait for it. */
#define KBD_QUEUE 32

struct TDeckKbdState {
    I2CSlave parent_obj;

    char *path;
    int fd;

    uint8_t q[KBD_QUEUE];
    int head, count;

    uint8_t buf[KBD_MSG];
    int have;
};

static void tdeck_kbd_push(TDeckKbdState *s, uint8_t key)
{
    if (s->count == KBD_QUEUE) {
        /* Drop the oldest. A queue that refuses new keys once full stops
         * responding to the person typing, which is the wrong end to lose. */
        s->head = (s->head + 1) % KBD_QUEUE;
        s->count--;
    }
    s->q[(s->head + s->count) % KBD_QUEUE] = key;
    s->count++;
}

static void tdeck_kbd_read(void *opaque)
{
    TDeckKbdState *s = TDECK_KEYBOARD(opaque);

    for (;;) {
        ssize_t n = recv(s->fd, s->buf + s->have, KBD_MSG - s->have, 0);
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
        if (s->have < KBD_MSG) {
            return;
        }
        s->have = 0;
        if (s->buf[0] == KBD_TAG && s->buf[1] != 0) {
            tdeck_kbd_push(s, s->buf[1]);
        }
    }
}

static void tdeck_kbd_connect(TDeckKbdState *s)
{
    struct sockaddr_un addr;

    if (s->fd >= 0 || !s->path || !*s->path) {
        return;
    }
    if (strlen(s->path) >= sizeof(addr.sun_path)) {
        warn_report("tdeck-keyboard: socket path is too long");
        s->path = NULL;
        return;
    }
    /* SOCK_NONBLOCK is a Linux extension. qemu_socket_set_nonblock() is the
     * same thing said in a way the Windows build also understands. */
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        return;
    }
    qemu_socket_set_nonblock(fd);
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    pstrcpy(addr.sun_path, sizeof(addr.sun_path), s->path);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 &&
        errno != EINPROGRESS && errno != EWOULDBLOCK) {
        close(fd);
        return;
    }
    s->fd = fd;
    qemu_set_fd_handler(fd, tdeck_kbd_read, NULL, s);
}

/* The whole protocol: a one byte read is the next key, and zero means none.
 * A firmware polling an empty keyboard reads zero for ever, which is exactly
 * what a real one does. */
static uint8_t tdeck_kbd_recv(I2CSlave *i2c)
{
    TDeckKbdState *s = TDECK_KEYBOARD(i2c);

    if (s->count == 0) {
        return 0;
    }
    uint8_t key = s->q[s->head];
    s->head = (s->head + 1) % KBD_QUEUE;
    s->count--;
    return key;
}

static int tdeck_kbd_send(I2CSlave *i2c, uint8_t data)
{
    /* Written to only to be probed. Accepting whatever arrives is what lets a
     * driver find the keyboard at all. */
    return 0;
}

static int tdeck_kbd_event(I2CSlave *i2c, enum i2c_event event)
{
    return 0;
}

static void tdeck_kbd_realize(DeviceState *dev, Error **errp)
{
    TDeckKbdState *s = TDECK_KEYBOARD(dev);

    s->fd = -1;
    s->head = s->count = s->have = 0;
    tdeck_kbd_connect(s);
}

static Property tdeck_kbd_props[] = {
    DEFINE_PROP_STRING("path", TDeckKbdState, path),
    DEFINE_PROP_END_OF_LIST(),
};

static void tdeck_kbd_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = tdeck_kbd_realize;
    device_class_set_props(dc, tdeck_kbd_props);
    sc->recv = tdeck_kbd_recv;
    sc->send = tdeck_kbd_send;
    sc->event = tdeck_kbd_event;
}

static const TypeInfo tdeck_kbd_info = {
    .name = TYPE_TDECK_KEYBOARD,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TDeckKbdState),
    .class_init = tdeck_kbd_class_init,
};

static void tdeck_kbd_register_types(void)
{
    type_register_static(&tdeck_kbd_info);
}

type_init(tdeck_kbd_register_types)
