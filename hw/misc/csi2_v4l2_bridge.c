#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/misc/csi2_v4l2_bridge.h"

uint64_t csi2_v4l2_bridge_read(void *opaque, hwaddr offset, unsigned size)
{
    CSI2V4L2Bridge *s = CSI2_V4L2_BRIDGE(opaque);
    uint64_t ret = 0;
    
    switch (offset) {
    case 0x00: 
        ret = s->streaming ? 1 : 0;
        break;
    case 0x04: 
        ret = s->frames_captured;
        break;
    case 0x08:
        ret = s->sequence_number;
        break;
    case 0x0C:
        ret = s->frames_dropped;
        break;
    case 0x10:
        ret = s->buffer_overruns;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
        break;
    }
    return ret;
}

void csi2_v4l2_bridge_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CSI2V4L2Bridge *s = CSI2_V4L2_BRIDGE(opaque);
    
    switch (offset) {
    case 0x00: 
        s->streaming = (value & 1) != 0;
        if (s->streaming) {
            printf("V4L2 Bridge: Streaming ENABLED\n");
            /* Start the frame generation immediately */
            timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
        } else {
            printf("V4L2 Bridge: Streaming DISABLED\n");
            timer_del(s->frame_timer);
        }
        break;
    case 0x04:
        /* frames_captured is read-only */
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Write to read-only register 0x%"HWADDR_PRIx"\n", __func__, offset);
        break;
    case 0x08:
        /* Reset sequence number if requested */
        if (value == 0) {
            s->sequence_number = 0;
            printf("V4L2 Bridge: Sequence number reset\n");
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
        break;
    }
}

static void csi2_v4l2_frame_timeout(void *opaque)
{
    CSI2V4L2Bridge *s = CSI2_V4L2_BRIDGE(opaque);
    
    if (s->streaming) {
        /* Increment counters */
        s->frames_captured++;
        s->sequence_number++;
        
        /* Print debug info every 30 frames (1 second at 30fps) */
        if (s->frames_captured % 30 == 1) {
            printf("V4L2 Bridge: Generated frame %d (seq: %d)\n", 
                   s->frames_captured, s->sequence_number);
        }
        
        /* Generate interrupt pulse */
        qemu_set_irq(s->irq, 1);
        qemu_set_irq(s->irq, 0);
        
        /* Schedule next frame (30 FPS = 33.33ms interval) */
        timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
    }
}

static const MemoryRegionOps csi2_v4l2_bridge_ops = {
    .read = csi2_v4l2_bridge_read,
    .write = csi2_v4l2_bridge_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void csi2_v4l2_bridge_realize(DeviceState *dev, Error **errp)
{
    CSI2V4L2Bridge *s = CSI2_V4L2_BRIDGE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_v4l2_bridge_ops, s, 
                          "csi2-v4l2", 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                  csi2_v4l2_frame_timeout, s);
    
    printf("V4L2 Bridge: Device realized\n");
}

static void csi2_v4l2_bridge_unrealize(DeviceState *dev)
{
    CSI2V4L2Bridge *s = CSI2_V4L2_BRIDGE(dev);
    
    if (s->frame_timer) {
        timer_free(s->frame_timer);
        s->frame_timer = NULL;
    }
}

static void csi2_v4l2_bridge_reset(DeviceState *dev)
{
    CSI2V4L2Bridge *s = CSI2_V4L2_BRIDGE(dev);
    
    s->streaming = false;
    s->sequence_number = 0;
    s->frames_captured = 0;
    s->frames_dropped = 0;
    s->buffer_overruns = 0;
    
    if (s->frame_timer) {
        timer_del(s->frame_timer);
    }
    
    printf("V4L2 Bridge: Device reset\n");
}

static void csi2_v4l2_bridge_instance_init(Object *obj)
{
    CSI2V4L2Bridge *s = CSI2_V4L2_BRIDGE(obj);
    
    /* Set default values */
    s->default_width = 1280;
    s->default_height = 720;
    s->default_format = 0x52474742; /* RGGB */
    
    /* Initialize runtime state */
    s->streaming = false;
    s->sequence_number = 0;
    s->frames_captured = 0;
    s->frames_dropped = 0;
    s->buffer_overruns = 0;
    s->frame_timer = NULL;
    s->csi2_controller = NULL;
}

static void csi2_v4l2_bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = csi2_v4l2_bridge_realize;
    dc->unrealize = csi2_v4l2_bridge_unrealize;
    device_class_set_legacy_reset(dc, csi2_v4l2_bridge_reset);
    dc->desc = "MIPI CSI-2 V4L2 Bridge";
}

static const TypeInfo csi2_v4l2_bridge_info = {
    .name = TYPE_CSI2_V4L2_BRIDGE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CSI2V4L2Bridge),
    .instance_init = csi2_v4l2_bridge_instance_init,
    .class_init = csi2_v4l2_bridge_class_init,
};

static void csi2_v4l2_bridge_register_types(void)
{
    type_register_static(&csi2_v4l2_bridge_info);
}

type_init(csi2_v4l2_bridge_register_types)
