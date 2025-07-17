#ifndef HW_CSI_CSI2_V4L2_BRIDGE_H
#define HW_CSI_CSI2_V4L2_BRIDGE_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qapi/error.h"

#define TYPE_CSI2_V4L2_BRIDGE "csi2-v4l2-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2V4L2Bridge, CSI2_V4L2_BRIDGE)

typedef struct CSI2V4L2Bridge {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    
    bool streaming;
    uint32_t sequence_number;
    uint32_t frames_captured;
    uint32_t frames_dropped;
    uint32_t buffer_overruns;
    
    QEMUTimer *frame_timer;
    DeviceState *csi2_controller;
    
    uint32_t default_width;
    uint32_t default_height;
    uint32_t default_format;
} CSI2V4L2Bridge;

uint64_t csi2_v4l2_bridge_read(void *opaque, hwaddr offset, unsigned size);
void csi2_v4l2_bridge_write(void *opaque, hwaddr offset, uint64_t value, unsigned size);

#endif
