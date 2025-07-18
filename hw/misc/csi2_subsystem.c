#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/misc/csi2_rx_controller.h"
#include "hw/misc/csi2_dphy.h"
#include "hw/misc/csi2_v4l2_bridge.h"

#define TYPE_CSI2_SUBSYSTEM "csi2-subsystem"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2Subsystem, CSI2_SUBSYSTEM)

typedef struct CSI2Subsystem {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    
    CSI2RxController *csi2_controller;
    CSI2DPhy *dphy;
    CSI2V4L2Bridge *v4l2_bridge;
    
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t frame_width;
    uint32_t frame_height;
    bool enable_v4l2;
} CSI2Subsystem;

static uint64_t csi2_subsystem_read(void *opaque, hwaddr offset, unsigned size)
{
    CSI2Subsystem *s = CSI2_SUBSYSTEM(opaque);
    
    if (offset < 0x1000) {
        return csi2_rx_controller_read(s->csi2_controller, offset, size);
    } else if (offset >= 0x1000 && offset < 0x2000) {
        return csi2_dphy_read(s->dphy, offset - 0x1000, size);
    } else if (offset >= 0x2000 && offset < 0x3000) {
        return csi2_v4l2_bridge_read(s->v4l2_bridge, offset - 0x2000, size);
    }
    
    qemu_log_mask(LOG_GUEST_ERROR, 
                  "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    return 0;
}

static void csi2_subsystem_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CSI2Subsystem *s = CSI2_SUBSYSTEM(opaque);
    
    if (offset < 0x1000) {
        csi2_rx_controller_write(s->csi2_controller, offset, value, size);
    } else if (offset >= 0x1000 && offset < 0x2000) {
        csi2_dphy_write(s->dphy, offset - 0x1000, value, size);
    } else if (offset >= 0x2000 && offset < 0x3000) {
        csi2_v4l2_bridge_write(s->v4l2_bridge, offset - 0x2000, value, size);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    }
}

static const MemoryRegionOps csi2_subsystem_ops = {
    .read = csi2_subsystem_read,
    .write = csi2_subsystem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void csi2_subsystem_realize(DeviceState *dev, Error **errp)
{
    CSI2Subsystem *s = CSI2_SUBSYSTEM(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    Error *local_err = NULL;
    
    /* Create sub-devices */
    s->csi2_controller = CSI2_RX_CONTROLLER(qdev_new(TYPE_CSI2_RX_CONTROLLER));
    s->dphy = CSI2_DPHY(qdev_new(TYPE_CSI2_DPHY));
    s->v4l2_bridge = CSI2_V4L2_BRIDGE(qdev_new(TYPE_CSI2_V4L2_BRIDGE));
    
    /* Realize sub-devices */
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(s->csi2_controller), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(s->dphy), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(s->v4l2_bridge), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    /* Initialize memory region and IRQ */
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_subsystem_ops, s, 
                          "csi2-subsystem", 0x3000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
}

static void csi2_subsystem_unrealize(DeviceState *dev)
{
    CSI2Subsystem *s = CSI2_SUBSYSTEM(dev);
    
    /* Clean up sub-devices */
    if (s->csi2_controller) {
        object_unref(OBJECT(s->csi2_controller));
        s->csi2_controller = NULL;
    }
    
    if (s->dphy) {
        object_unref(OBJECT(s->dphy));
        s->dphy = NULL;
    }
    
    if (s->v4l2_bridge) {
        object_unref(OBJECT(s->v4l2_bridge));
        s->v4l2_bridge = NULL;
    }
}

static void csi2_subsystem_reset(DeviceState *dev)
{
    CSI2Subsystem *s = CSI2_SUBSYSTEM(dev);
    
    /* Reset sub-devices */
    if (s->csi2_controller) {
        device_cold_reset(DEVICE(s->csi2_controller));
    }
    if (s->dphy) {
        device_cold_reset(DEVICE(s->dphy));
    }
    if (s->v4l2_bridge) {
        device_cold_reset(DEVICE(s->v4l2_bridge));
    }
}

static void csi2_subsystem_instance_init(Object *obj)
{
    CSI2Subsystem *s = CSI2_SUBSYSTEM(obj);
    
    /* Set default values */
    s->num_lanes = 4;
    s->line_rate = 1000;
    s->frame_width = 1280;
    s->frame_height = 720;
    s->enable_v4l2 = true;
    
    /* Initialize pointers to NULL */
    s->csi2_controller = NULL;
    s->dphy = NULL;
    s->v4l2_bridge = NULL;
}

static void csi2_subsystem_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = csi2_subsystem_realize;
    dc->unrealize = csi2_subsystem_unrealize;
    device_class_set_legacy_reset(dc, csi2_subsystem_reset);
    dc->desc = "MIPI CSI-2 RX Subsystem";
}

static const TypeInfo csi2_subsystem_info = {
    .name = TYPE_CSI2_SUBSYSTEM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CSI2Subsystem),
    .instance_init = csi2_subsystem_instance_init,
    .class_init = csi2_subsystem_class_init,
};

static void csi2_subsystem_register_types(void)
{
    type_register_static(&csi2_subsystem_info);
}

type_init(csi2_subsystem_register_types)
