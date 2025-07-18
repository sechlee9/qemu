// include/hw/misc/csi2_v4l2_integration.h - CSI2 V4L2 통합 헤더
#ifndef HW_CSI2_V4L2_INTEGRATION_H
#define HW_CSI2_V4L2_INTEGRATION_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "include/hw/misc/csi2_rx_controller.h"
#include "include/hw/misc/csi2_v4l2_bridge.h"

#define TYPE_CSI2_V4L2_DEVICE "csi2-v4l2-device"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2V4L2Device, CSI2_V4L2_DEVICE)

/* Forward declarations */
typedef struct V4L2Backend V4L2Backend;

typedef struct CSI2V4L2Device {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    
    /* Sub-components */
    CSI2RxController *csi2_controller;
    CSI2V4L2Bridge *v4l2_bridge;
    V4L2Backend *v4l2_backend;
    
    /* Configuration properties */
    char *device_name;
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t pixel_format;
    bool auto_start;
    
    /* Runtime state */
    bool streaming;
    uint32_t frames_generated;
    QEMUTimer *status_timer;
    
} CSI2V4L2Device;

/* Public API functions */
DeviceState *csi2_v4l2_device_create(const char *device_name,
                                     uint32_t num_lanes,
                                     uint32_t line_rate,
                                     uint32_t frame_width,
                                     uint32_t frame_height);

void csi2_v4l2_device_start_streaming(CSI2V4L2Device *dev);
void csi2_v4l2_device_stop_streaming(CSI2V4L2Device *dev);

/* Configuration helpers */
static inline void csi2_v4l2_device_set_format(CSI2V4L2Device *dev,
                                              uint32_t width,
                                              uint32_t height,
                                              uint32_t format)
{
    dev->frame_width = width;
    dev->frame_height = height;
    dev->pixel_format = format;
}

#endif /* HW_CSI2_V4L2_INTEGRATION_H */
