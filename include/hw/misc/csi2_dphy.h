#ifndef HW_CSI_CSI2_DPHY_H
#define HW_CSI_CSI2_DPHY_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qapi/error.h"

#define TYPE_CSI2_DPHY "csi2-dphy"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2DPhy, CSI2_DPHY)

typedef struct CSI2DPhy {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;

    uint32_t ctrl;
    uint32_t status;
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t default_line_rate;
    bool enable_deskew;
    bool pll_locked;

    QEMUTimer *hs_timer;
    QEMUTimer *settle_timer;
    DeviceState *csi2_controller;
} CSI2DPhy;

uint64_t csi2_dphy_read(void *opaque, hwaddr offset, unsigned size);
void csi2_dphy_write(void *opaque, hwaddr offset, uint64_t value, unsigned size);

#endif

