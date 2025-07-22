// include/hw/misc/csi2_v4l2_loopback.h - V4L2 루프백 헤더
#ifndef HW_CSI2_V4L2_LOOPBACK_H
#define HW_CSI2_V4L2_LOOPBACK_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qapi/error.h"

#define TYPE_CSI2_V4L2_LOOPBACK "csi2-v4l2-loopback"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2V4L2Loopback, CSI2_V4L2_LOOPBACK)

/* Forward declarations */
typedef struct CSI2RxController CSI2RxController;
typedef struct CSI2V4L2Bridge CSI2V4L2Bridge;

typedef struct CSI2V4L2Loopback {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    
    /* Sub-components */
    CSI2RxController *csi2_controller;
    CSI2V4L2Bridge *v4l2_bridge;
    
    /* Configuration properties */
    char *device_name;
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t pixel_format;
    uint32_t v4l2_frame_rate;
    bool auto_start;
    bool v4l2_enabled;
    
    /* Runtime state */
    bool streaming;
    uint32_t frames_generated;
    QEMUTimer *frame_timer;
    QEMUTimer *status_timer;
    
} CSI2V4L2Loopback;

/* Public API functions */
int csi2_loopback_device_start(CSI2V4L2Loopback *dev);
void csi2_loopback_device_stop(CSI2V4L2Loopback *dev);

/* Configuration helpers */
static inline void csi2_loopback_set_format(CSI2V4L2Loopback *dev,
                                           uint32_t width,
                                           uint32_t height,
                                           uint32_t format)
{
    dev->frame_width = width;
    dev->frame_height = height;
    dev->pixel_format = format;
}

static inline void csi2_loopback_set_rate(CSI2V4L2Loopback *dev,
                                         uint32_t fps)
{
    if (fps > 0 && fps <= 120) {
        dev->v4l2_frame_rate = fps;
    }
}

/* Status query */
static inline bool csi2_loopback_is_streaming(CSI2V4L2Loopback *dev)
{
    return dev ? dev->streaming : false;
}

static inline uint32_t csi2_loopback_get_frame_count(CSI2V4L2Loopback *dev)
{
    return dev ? dev->frames_generated : 0;
}

#endif /* HW_CSI2_V4L2_LOOPBACK_H */
