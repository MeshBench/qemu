/*
 * SSD1306 / SH1106 monochrome OLED, as an I2C slave.
 *
 * What thirteen of the boards in this project carry. It models only what a
 * driver actually uses: the page and column pointers, the addressing mode, and
 * the pixel data that follows a 0x40 control byte. Contrast, charge pump,
 * multiplex ratio and the rest are accepted and ignored, because nothing above
 * this can see them.
 *
 * The picture leaves over a socket rather than into a QEMU window. A window
 * works on a desk and fails on a headless runner, on a lab machine, and any
 * time somebody wants to know what a node was showing when something went
 * wrong - and the application that owns the node already has a place to draw
 * it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "qemu/module.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define TYPE_SSD1306_PANEL "ssd1306-panel"
OBJECT_DECLARE_SIMPLE_TYPE(SSD1306PanelState, SSD1306_PANEL)

/* The part is 128 columns by 8 pages of 8 rows. An SH1106 is the same
 * geometry with its columns offset by two, which is why the offset is a
 * property rather than a constant: get it wrong and the whole picture slides
 * sideways, which reads as a driver fault. */
#define SSD_COLS  128
#define SSD_PAGES 8

/* How often the framebuffer is sent, at most. A driver redraws far more often
 * than anybody can look, and the socket is not the place to find that out. */
#define SSD_FRAME_MS 100

struct SSD1306PanelState {
    I2CSlave parent_obj;

    char *path;      /* unix socket the frames go to */
    uint32_t offset; /* column offset: 0 for an SSD1306, 2 for an SH1106 */

    int fd;
    QEMUTimer *tick;

    /* Whether the byte after the address is a command or pixel data, which
     * the control byte says and the driver sets once per transfer. */
    bool data_phase;
    bool have_control;

    /* Multi-byte commands: how many arguments are still owed. */
    int args_left;
    uint8_t pending_cmd;

    uint8_t col, page;
    uint8_t col_start, col_end, page_start, page_end;
    uint8_t addr_mode; /* 0 horizontal, 1 vertical, 2 page */
    bool on;
    bool dirty;

    uint8_t fb[SSD_PAGES][SSD_COLS];
};

/* connect is deliberately quiet about failing. A board whose panel nobody is
 * listening to must still run: the firmware does not know or care, and a
 * refusal here would turn "nothing is watching the screen" into "the board
 * will not start". */
static void ssd1306_connect(SSD1306PanelState *s)
{
    struct sockaddr_un addr;

    if (s->fd >= 0 || !s->path || !*s->path) {
        return;
    }
    if (strlen(s->path) >= sizeof(addr.sun_path)) {
        warn_report("ssd1306: panel socket path is too long: %s", s->path);
        s->path = NULL;
        return;
    }
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        return;
    }
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    pstrcpy(addr.sun_path, sizeof(addr.sun_path), s->path);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return;
    }
    s->fd = fd;
}

/* One frame: a fixed header and the pages as the part stores them, one bit a
 * pixel. Sent whole rather than as damage rectangles - a kilobyte at ten hertz
 * is nothing, and a protocol that can drop an update is a protocol that will. */
static void ssd1306_send_frame(SSD1306PanelState *s)
{
    /* magic, then width and height as 16-bit little-endian, bits per pixel,
     * and whether the panel is switched on. Sixteen bits because a colour
     * panel on the same protocol is 320 across and a byte cannot say so. */
    uint8_t hdr[10] = {'M', 'B', 'F', '2',
                       SSD_COLS & 0xff, SSD_COLS >> 8,
                       (SSD_PAGES * 8) & 0xff, (SSD_PAGES * 8) >> 8,
                       1, 0};

    ssd1306_connect(s);
    if (s->fd < 0) {
        return;
    }
    hdr[9] = s->on ? 1 : 0;

    struct iovec iov[2] = {
        {.iov_base = hdr, .iov_len = sizeof(hdr)},
        {.iov_base = s->fb, .iov_len = sizeof(s->fb)},
    };
    struct msghdr msg = {.msg_iov = iov, .msg_iovlen = 2};
    if (sendmsg(s->fd, &msg, MSG_NOSIGNAL) < 0) {
        close(s->fd);
        s->fd = -1;
    }
}

static void ssd1306_tick(void *opaque)
{
    SSD1306PanelState *s = SSD1306_PANEL(opaque);

    /* Reconnecting counts as a change: a listener that attached after the
     * board drew its screen wants the picture, not the next update. */
    bool was = s->fd >= 0;
    ssd1306_connect(s);
    if (s->dirty || (!was && s->fd >= 0)) {
        s->dirty = false;
        ssd1306_send_frame(s);
    }
    timer_mod(s->tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SSD_FRAME_MS);
}

static void ssd1306_command(SSD1306PanelState *s, uint8_t cmd)
{
    if (s->args_left > 0) {
        switch (s->pending_cmd) {
        case 0x21: /* set column address: start then end */
            if (s->args_left == 2) {
                s->col_start = cmd;
                s->col = cmd;
            } else {
                s->col_end = cmd;
            }
            break;
        case 0x22: /* set page address: start then end */
            if (s->args_left == 2) {
                s->page_start = cmd;
                s->page = cmd;
            } else {
                s->page_end = cmd;
            }
            break;
        case 0x20:
            s->addr_mode = cmd & 3;
            break;
        default:
            break; /* contrast, multiplex, charge pump: accepted, unmodelled */
        }
        s->args_left--;
        return;
    }

    switch (cmd) {
    case 0x20:
    case 0x81:
    case 0x8D:
    case 0xA8:
    case 0xD3:
    case 0xD5:
    case 0xD9:
    case 0xDA:
    case 0xDB:
        s->pending_cmd = cmd;
        s->args_left = 1;
        break;
    case 0x21:
    case 0x22:
        s->pending_cmd = cmd;
        s->args_left = 2;
        break;
    case 0xAE:
        s->on = false;
        s->dirty = true;
        break;
    case 0xAF:
        s->on = true;
        s->dirty = true;
        break;
    default:
        /* Page and column pointers, the SH1106's usual way of addressing. */
        if (cmd >= 0xB0 && cmd <= 0xB7) {
            s->page = cmd - 0xB0;
        } else if (cmd <= 0x0F) {
            s->col = (s->col & 0xF0) | cmd;
        } else if (cmd >= 0x10 && cmd <= 0x1F) {
            s->col = (s->col & 0x0F) | ((cmd - 0x10) << 4);
        }
        break;
    }
}

static void ssd1306_data(SSD1306PanelState *s, uint8_t byte)
{
    int col = (int)s->col - (int)s->offset;

    if (s->page < SSD_PAGES && col >= 0 && col < SSD_COLS) {
        s->fb[s->page][col] = byte;
    }
    /* Dirty on any write, not only on a byte that changed. The first thing a
     * driver does is clear the screen, which writes zeros over a buffer that
     * is already zero - and a panel that reported nothing until a pixel
     * differed sent no frame at all until something was drawn, so a cleared
     * screen looked exactly like a panel nobody had wired. The tick already
     * limits how often this is acted on. */
    s->dirty = true;
    s->col++;

    /* Horizontal addressing wraps the column inside the window and steps the
     * page. Page addressing does not wrap at all, which is what an SH1106
     * driver relies on. */
    if (s->addr_mode == 0) {
        uint8_t end = s->col_end ? s->col_end : SSD_COLS - 1;
        if (s->col > end + s->offset) {
            s->col = s->col_start;
            s->page++;
            if (s->page > (s->page_end ? s->page_end : SSD_PAGES - 1)) {
                s->page = s->page_start;
            }
        }
    }
}

static int ssd1306_send(I2CSlave *i2c, uint8_t data)
{
    SSD1306PanelState *s = SSD1306_PANEL(i2c);

    if (!s->have_control) {
        /* The control byte: bit 6 says the rest of this transfer is pixel
         * data, and bit 7 says only the next byte is. */
        s->have_control = true;
        s->data_phase = (data & 0x40) != 0;
        return 0;
    }
    if (s->data_phase) {
        ssd1306_data(s, data);
    } else {
        ssd1306_command(s, data);
    }
    return 0;
}

static uint8_t ssd1306_recv(I2CSlave *i2c)
{
    /* A driver probes by writing, and reads a status byte at most. Answering
     * zero is answering "not busy", which is true of a panel that redraws in
     * no time at all. */
    return 0;
}

static int ssd1306_event(I2CSlave *i2c, enum i2c_event event)
{
    SSD1306PanelState *s = SSD1306_PANEL(i2c);

    if (event == I2C_START_SEND || event == I2C_START_RECV) {
        s->have_control = false;
    }
    return 0;
}

static void ssd1306_reset_hold(Object *obj, ResetType type)
{
    SSD1306PanelState *s = SSD1306_PANEL(obj);

    s->col = s->page = 0;
    s->col_start = 0;
    s->col_end = SSD_COLS - 1;
    s->page_start = 0;
    s->page_end = SSD_PAGES - 1;
    s->addr_mode = 2;
    s->args_left = 0;
    s->have_control = false;
    s->data_phase = false;
    s->on = false;
    s->dirty = true;
    memset(s->fb, 0, sizeof(s->fb));
}

static void ssd1306_realize(DeviceState *dev, Error **errp)
{
    SSD1306PanelState *s = SSD1306_PANEL(dev);

    s->fd = -1;
    /* Put the registers in their reset state here as well as on reset. A
     * device on a bus is not guaranteed to be reached by the machine's reset,
     * and one that starts with a stale framebuffer and a clear dirty flag
     * sends nothing at all - which looks exactly like a panel nobody wired. */
    ssd1306_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
    ssd1306_connect(s);
    s->tick = timer_new_ms(QEMU_CLOCK_VIRTUAL, ssd1306_tick, s);
    timer_mod(s->tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SSD_FRAME_MS);
}

static Property ssd1306_props[] = {
    DEFINE_PROP_STRING("path", SSD1306PanelState, path),
    DEFINE_PROP_UINT32("column-offset", SSD1306PanelState, offset, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void ssd1306_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = ssd1306_realize;
    device_class_set_props(dc, ssd1306_props);
    rc->phases.hold = ssd1306_reset_hold;
    sc->send = ssd1306_send;
    sc->recv = ssd1306_recv;
    sc->event = ssd1306_event;
}

static const TypeInfo ssd1306_info = {
    .name = TYPE_SSD1306_PANEL,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(SSD1306PanelState),
    .class_init = ssd1306_class_init,
};

static void ssd1306_register_types(void)
{
    type_register_static(&ssd1306_info);
}

type_init(ssd1306_register_types)
