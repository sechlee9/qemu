// include/system/v4l2.h - V4L2 백엔드 헤더
#ifndef QEMU_SYSTEM_V4L2_H
#define QEMU_SYSTEM_V4L2_H

#include "qemu/osdep.h"
#include "qom/object.h"
#include "qapi/error.h"

typedef struct V4L2Backend V4L2Backend;

/* Frame data structure */
typedef struct V4L2Frame {
    uint32_t width;
    uint32_t height;
    uint32_t format;        /* V4L2_PIX_FMT_* */
    uint32_t sequence;
    uint64_t timestamp;
    uint8_t *data;
    size_t size;
    uint32_t bytesperline;
} V4L2Frame;

/* Callback for frame processing */
typedef void (*V4L2FrameCallback)(V4L2Frame *frame, void *opaque);

/* V4L2 Backend API */
V4L2Backend *v4l2_backend_new(const char *device_name, Error **errp);
void v4l2_backend_free(V4L2Backend *v4l2);

int v4l2_backend_start(V4L2Backend *v4l2, Error **errp);
void v4l2_backend_stop(V4L2Backend *v4l2);

void v4l2_backend_set_frame_callback(V4L2Backend *v4l2, 
                                    V4L2FrameCallback callback,
                                    void *opaque);

const char *v4l2_backend_get_device_path(V4L2Backend *v4l2);

/* Format utilities */
static inline const char *v4l2_format_to_string(uint32_t format)
{
    switch (format) {
    case 0x32424752: /* V4L2_PIX_FMT_RGB24 */
        return "RGB24";
    case 0x56595559: /* V4L2_PIX_FMT_YUYV */
        return "YUYV";
    case 0x50424752: /* V4L2_PIX_FMT_RGB565 */
        return "RGB565";
    case 0x30385942: /* V4L2_PIX_FMT_BA81 (RAW8) */
        return "RAW8";
    case 0x41384142: /* V4L2_PIX_FMT_BA10 (RAW10) */
        return "RAW10";
    default:
        return "Unknown";
    }
}

static inline uint32_t v4l2_format_bpp(uint32_t format)
{
    switch (format) {
    case 0x32424752: /* V4L2_PIX_FMT_RGB24 */
        return 24;
    case 0x56595559: /* V4L2_PIX_FMT_YUYV */
        return 16;
    case 0x50424752: /* V4L2_PIX_FMT_RGB565 */
        return 16;
    case 0x30385942: /* V4L2_PIX_FMT_BA81 (RAW8) */
        return 8;
    case 0x41384142: /* V4L2_PIX_FMT_BA10 (RAW10) */
        return 10;
    default:
        return 8;
    }
}

#endif /* QEMU_SYSTEM_V4L2_H */
