#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/misc/csi2_dphy.h"

uint64_t csi2_dphy_read(void *opaque, hwaddr offset, unsigned size)
{
    CSI2DPhy *s = CSI2_DPHY(opaque);
    uint64_t ret = 0;
    
    switch (offset) {
    case 0x00: /* Control */
        ret = s->ctrl;
        break;
    case 0x04: /* Status */
        ret = s->status | (s->pll_locked ? 1 : 0);
        break;
    case 0x08: /* Line rate */
        ret = s->line_rate;
        break;
    case 0x0C: /* Number of lanes */
        ret = s->num_lanes;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
        break;
    }
    return ret;
}

void csi2_dphy_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CSI2DPhy *s = CSI2_DPHY(opaque);
    
    switch (offset) {
    case 0x00: /* Control */
        s->ctrl = value;
        if (value & 1) { /* Enable */
            s->pll_locked = true;
            s->status |= 1;
        } else {
            s->pll_locked = false;
            s->status &= ~1;
        }
        break;
    case 0x08: /* Line rate */
        s->line_rate = value;
        break;
    case 0x0C: /* Number of lanes */
        if (value <= 4) {
            s->num_lanes = value;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
        break;
    }
}

static const MemoryRegionOps csi2_dphy_ops = {
    .read = csi2_dphy_read,
    .write = csi2_dphy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void csi2_dphy_realize(DeviceState *dev, Error **errp)
{
    CSI2DPhy *s = CSI2_DPHY(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_dphy_ops, s, 
                          "csi2-dphy", 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
}

static void csi2_dphy_reset(DeviceState *dev)
{
    CSI2DPhy *s = CSI2_DPHY(dev);
    
    s->ctrl = 0;
    s->status = 0;
    s->pll_locked = false;
    s->line_rate = s->default_line_rate;
}

static void csi2_dphy_instance_init(Object *obj)
{
    CSI2DPhy *s = CSI2_DPHY(obj);
    
    /* Set default values */
    s->num_lanes = 4;
    s->default_line_rate = 1000;
    s->enable_deskew = false;
}

static void csi2_dphy_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = csi2_dphy_realize;
    device_class_set_legacy_reset(dc, csi2_dphy_reset);
    dc->desc = "MIPI D-PHY";
}

static const TypeInfo csi2_dphy_info = {
    .name = TYPE_CSI2_DPHY,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CSI2DPhy),
    .instance_init = csi2_dphy_instance_init,
    .class_init = csi2_dphy_class_init,
};

static void csi2_dphy_register_types(void)
{
    type_register_static(&csi2_dphy_info);
}

type_init(csi2_dphy_register_types)
