/*
 * ST7789 / ST7735 colour TFT, as an SPI peripheral.
 *
 * What the T-Deck and the tracker boards carry, and unlike the mono OLEDs it
 * does not get a bus to itself: on those boards the display, the radio and the
 * card reader all hang off one controller, told apart only by which chip
 * select is low. So this ignores every byte that arrives while its own select
 * is high, and a driver writing to the radio does not scribble on the screen.
 *
 * A second line decides what a byte means. DC low is a command, DC high is
 * data - which is the whole protocol: set a column window, set a row window,
 * then stream pixels into it.
 *
 * Pixels leave in the format the firmware wrote them, RGB565, down the same
 * socket the mono panel uses. Converting here would be inventing colour the
 * firmware did not ask for.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/ssi/ssi.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define TYPE_ST7789_PANEL "st7789-panel"
OBJECT_DECLARE_SIMPLE_TYPE(ST7789PanelState, ST7789_PANEL)

/* Biggest panel these boards carry. Allocated rather than sized per board so
 * one model serves the 240x240 trackers and the T-Deck's 320x240 alike. */
#define ST_MAX_W 320
#define ST_MAX_H 320
#define ST_FRAME_MS 100

/* The handful of commands that decide what appears. Everything else - gamma,
 * porch control, power settings - is accepted and ignored, because nothing
 * above this model can see any of it. */
enum {
    ST_SWRESET = 0x01,
    ST_SLPOUT  = 0x11,
    ST_INVOFF  = 0x20,
    ST_INVON   = 0x21,
    ST_DISPOFF = 0x28,
    ST_DISPON  = 0x29,
    ST_CASET   = 0x2A,
    ST_RASET   = 0x2B,
    ST_RAMWR   = 0x2C,
    ST_MADCTL  = 0x36,
    ST_COLMOD  = 0x3A,
};

struct ST7789PanelState {
    SSIPeripheral parent_obj;

    char *path;
    uint32_t width, height;

    int fd;
    QEMUTimer *tick;

    bool cs_active;
    bool dc_data;
    bool on;
    bool dirty;

    uint8_t cmd;
    uint8_t args[4];
    int nargs;

    /* The window the next pixels land in, and where in it we are. */
    uint16_t x0, x1, y0, y1;
    uint16_t x, y;
    /* RAMWR sends two bytes a pixel and a transfer can split them. */
    bool have_hi;
    uint8_t hi;

    uint16_t fb[ST_MAX_W * ST_MAX_H];
};

static void st7789_connect(ST7789PanelState *s)
{
    struct sockaddr_un addr;

    if (s->fd >= 0 || !s->path || !*s->path) {
        return;
    }
    if (strlen(s->path) >= sizeof(addr.sun_path)) {
        warn_report("st7789: panel socket path is too long");
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

static void st7789_send_frame(ST7789PanelState *s)
{
    uint8_t hdr[10] = {'M', 'B', 'F', '2',
                       s->width & 0xff, s->width >> 8,
                       s->height & 0xff, s->height >> 8,
                       16, 0};

    st7789_connect(s);
    if (s->fd < 0) {
        return;
    }
    hdr[9] = s->on ? 1 : 0;

    struct iovec iov[2] = {
        {.iov_base = hdr, .iov_len = sizeof(hdr)},
        {.iov_base = s->fb, .iov_len = (size_t)s->width * s->height * 2},
    };
    struct msghdr msg = {.msg_iov = iov, .msg_iovlen = 2};
    if (sendmsg(s->fd, &msg, MSG_NOSIGNAL) < 0) {
        close(s->fd);
        s->fd = -1;
    }
}

static void st7789_tick(void *opaque)
{
    ST7789PanelState *s = ST7789_PANEL(opaque);
    bool was = s->fd >= 0;

    st7789_connect(s);
    if (s->dirty || (!was && s->fd >= 0)) {
        s->dirty = false;
        st7789_send_frame(s);
    }
    timer_mod(s->tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + ST_FRAME_MS);
}

static uint16_t st7789_arg16(const ST7789PanelState *s, int at)
{
    return (uint16_t)((s->args[at] << 8) | s->args[at + 1]);
}

static void st7789_finish_command(ST7789PanelState *s)
{
    switch (s->cmd) {
    case ST_CASET:
        if (s->nargs >= 4) {
            s->x0 = st7789_arg16(s, 0);
            s->x1 = st7789_arg16(s, 2);
            s->x = s->x0;
        }
        break;
    case ST_RASET:
        if (s->nargs >= 4) {
            s->y0 = st7789_arg16(s, 0);
            s->y1 = st7789_arg16(s, 2);
            s->y = s->y0;
        }
        break;
    default:
        break;
    }
}

static void st7789_command(ST7789PanelState *s, uint8_t cmd)
{
    st7789_finish_command(s);
    s->cmd = cmd;
    s->nargs = 0;
    s->have_hi = false;

    switch (cmd) {
    case ST_DISPON:
    case ST_SLPOUT:
        s->on = true;
        s->dirty = true;
        break;
    case ST_DISPOFF:
        s->on = false;
        s->dirty = true;
        break;
    case ST_RAMWR:
        s->x = s->x0;
        s->y = s->y0;
        break;
    case ST_SWRESET:
        memset(s->fb, 0, sizeof(s->fb));
        s->dirty = true;
        break;
    default:
        break;
    }
}

static void st7789_data(ST7789PanelState *s, uint8_t byte)
{
    if (s->cmd == ST_RAMWR) {
        if (!s->have_hi) {
            s->hi = byte;
            s->have_hi = true;
            return;
        }
        s->have_hi = false;
        uint16_t px = (uint16_t)((s->hi << 8) | byte);
        if (s->x < s->width && s->y < s->height) {
            s->fb[(size_t)s->y * s->width + s->x] = px;
            s->dirty = true;
        }
        s->x++;
        if (s->x > s->x1) {
            s->x = s->x0;
            s->y++;
            if (s->y > s->y1) {
                s->y = s->y0;
            }
        }
        return;
    }
    if (s->nargs < (int)sizeof(s->args)) {
        s->args[s->nargs] = byte;
    }
    s->nargs++;
    /* Window commands take four bytes and are acted on as soon as they are
     * complete: a driver may follow them straight with RAMWR without any
     * command in between. */
    if ((s->cmd == ST_CASET || s->cmd == ST_RASET) && s->nargs == 4) {
        st7789_finish_command(s);
        s->nargs = 0;
    }
}

static uint32_t st7789_transfer(SSIPeripheral *dev, uint32_t val)
{
    ST7789PanelState *s = ST7789_PANEL(dev);

    /* Not ours unless our own select is low. The radio is on this bus too. */
    if (!s->cs_active) {
        return 0;
    }
    if (s->dc_data) {
        st7789_data(s, (uint8_t)val);
    } else {
        st7789_command(s, (uint8_t)val);
    }
    return 0;
}

static void st7789_cs(void *opaque, int n, int level)
{
    ST7789PanelState *s = ST7789_PANEL(opaque);
    s->cs_active = !level; /* active low */
}

static void st7789_dc(void *opaque, int n, int level)
{
    ST7789PanelState *s = ST7789_PANEL(opaque);
    s->dc_data = level != 0;
}

static void st7789_reset_hold(Object *obj, ResetType type)
{
    ST7789PanelState *s = ST7789_PANEL(obj);

    s->cs_active = false;
    s->dc_data = false;
    s->on = false;
    s->cmd = 0;
    s->nargs = 0;
    s->have_hi = false;
    s->x0 = s->y0 = 0;
    s->x1 = (uint16_t)(s->width ? s->width - 1 : 0);
    s->y1 = (uint16_t)(s->height ? s->height - 1 : 0);
    s->x = s->y = 0;
    memset(s->fb, 0, sizeof(s->fb));
    s->dirty = true;
}

static void st7789_realize(SSIPeripheral *dev, Error **errp)
{
    ST7789PanelState *s = ST7789_PANEL(dev);

    if (s->width == 0 || s->height == 0 ||
        s->width > ST_MAX_W || s->height > ST_MAX_H) {
        error_setg(errp, "st7789: %ux%u is not a panel this models",
                   s->width, s->height);
        return;
    }
    s->fd = -1;
    st7789_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
    st7789_connect(s);
    s->tick = timer_new_ms(QEMU_CLOCK_VIRTUAL, st7789_tick, s);
    timer_mod(s->tick, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + ST_FRAME_MS);

    qdev_init_gpio_in_named(DEVICE(dev), st7789_cs, "st7789-cs", 1);
    qdev_init_gpio_in_named(DEVICE(dev), st7789_dc, "st7789-dc", 1);
}

static Property st7789_props[] = {
    DEFINE_PROP_STRING("path", ST7789PanelState, path),
    DEFINE_PROP_UINT32("width", ST7789PanelState, width, 320),
    DEFINE_PROP_UINT32("height", ST7789PanelState, height, 240),
    DEFINE_PROP_END_OF_LIST(),
};

static void st7789_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    k->realize = st7789_realize;
    k->transfer = st7789_transfer;
    k->cs_polarity = SSI_CS_NONE;
    rc->phases.hold = st7789_reset_hold;
    device_class_set_props(dc, st7789_props);
}

static const TypeInfo st7789_info = {
    .name = TYPE_ST7789_PANEL,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(ST7789PanelState),
    .class_init = st7789_class_init,
};

static void st7789_register_types(void)
{
    type_register_static(&st7789_info);
}

type_init(st7789_register_types)
