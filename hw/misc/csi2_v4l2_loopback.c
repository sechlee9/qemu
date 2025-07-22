// hw/misc/csi2_v4l2_loopback.c - V4L2 루프백 디바이스 구현 (수정됨)
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "include/hw/misc/csi2_rx_controller.h"
#include "include/hw/misc/csi2_v4l2_bridge.h"

#define TYPE_CSI2_V4L2_LOOPBACK "csi2-v4l2-loopback"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2V4L2Loopback, CSI2_V4L2_LOOPBACK)

typedef struct CSI2V4L2Loopback {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    
    /* Sub-components */
    CSI2RxController *csi2_controller;
    CSI2V4L2Bridge *v4l2_bridge;
    
    /* Configuration */
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
    QEMUTimer *frame_timer;
    QEMUTimer *status_timer;
    
    /* V4L2 simulation */
    bool v4l2_enabled;
    uint32_t v4l2_frame_rate;
    
} CSI2V4L2Loopback;

/* Forward declarations */
static int csi2_loopback_start(CSI2V4L2Loopback *dev);
static void csi2_loopback_stop(CSI2V4L2Loopback *dev);
static uint64_t csi2_loopback_read(void *opaque, hwaddr offset, unsigned size);
static void csi2_loopback_write(void *opaque, hwaddr offset, uint64_t value, unsigned size);

/* Frame generation timer callback */
static void csi2_loopback_frame_timeout(void *opaque)
{
    CSI2V4L2Loopback *dev = CSI2_V4L2_LOOPBACK(opaque);
    
    if (!dev->streaming) {
        return;
    }
    
    /* Update CSI2 controller statistics */
    if (dev->csi2_controller) {
        dev->csi2_controller->packet_count++;
        dev->csi2_controller->core_status = 
            (dev->csi2_controller->packet_count << 16) & 0xFFFF0000;
        
        /* Update image info for VC0 */
        dev->csi2_controller->img_info1[0] = 
            (dev->csi2_controller->packet_count << 16) | dev->frame_width;
        dev->csi2_controller->img_info2[0] = dev->pixel_format & 0x3F;
        
        /* Set frame received interrupt */
        dev->csi2_controller->int_status |= CSI2_INT_FRAME_RECEIVED;
        
        /* Generate interrupt if enabled */
        if (dev->csi2_controller->global_int_enable && 
            (dev->csi2_controller->int_status & dev->csi2_controller->int_enable)) {
            qemu_set_irq(dev->csi2_controller->irq, 1);
            qemu_set_irq(dev->csi2_controller->irq, 0); /* Pulse */
        }
    }
    
    /* Update V4L2 bridge statistics */
    if (dev->v4l2_bridge) {
        dev->v4l2_bridge->frames_captured++;
        dev->v4l2_bridge->sequence_number++;
    }
    
    dev->frames_generated++;
    
    /* Print status every 30 frames */
    if (dev->frames_generated % 30 == 1) {
        printf("CSI2-V4L2-Loopback: Generated frame %d (%dx%d)\n",
               dev->frames_generated, dev->frame_width, dev->frame_height);
    }
    
    /* Schedule next frame */
    uint64_t interval_ns = 1000000000ULL / dev->v4l2_frame_rate; /* Convert FPS to nanoseconds */
    timer_mod(dev->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + interval_ns);
}

/* Status monitoring timer */
static void csi2_loopback_status_timeout(void *opaque)
{
    CSI2V4L2Loopback *dev = CSI2_V4L2_LOOPBACK(opaque);
    
    /* Periodic status check every 10 seconds */
    if (dev->streaming && dev->frames_generated % 300 == 0) {
        printf("CSI2-V4L2-Loopback: Status - %d frames generated, streaming=%s\n",
               dev->frames_generated, dev->streaming ? "active" : "inactive");
    }
    
    /* Schedule next check */
    timer_mod(dev->status_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000000000LL);
}

/* Memory mapped register access */
static uint64_t csi2_loopback_read(void *opaque, hwaddr offset, unsigned size)
{
    CSI2V4L2Loopback *dev = CSI2_V4L2_LOOPBACK(opaque);
    
    if (offset < 0x1000) {
        /* CSI2 Controller registers */
        return csi2_rx_controller_read(dev->csi2_controller, offset, size);
    } else if (offset >= 0x2000 && offset < 0x3000) {
        /* V4L2 Bridge registers */
        return csi2_v4l2_bridge_read(dev->v4l2_bridge, offset - 0x2000, size);
    } else if (offset >= 0x3000 && offset < 0x4000) {
        /* Loopback device specific registers */
        switch (offset - 0x3000) {
        case 0x00: /* Device status */
            return (dev->streaming ? 1 : 0) | 
                   (dev->v4l2_enabled ? 2 : 0) |
                   ((dev->v4l2_frame_rate & 0xFF) << 8);
        case 0x04: /* Frame count */
            return dev->frames_generated;
        case 0x08: /* Configuration */
            return (dev->frame_width << 16) | dev->frame_height;
        case 0x0C: /* Pixel format */
            return dev->pixel_format;
        case 0x10: /* Line configuration */
            return (dev->num_lanes << 16) | (dev->line_rate & 0xFFFF);
        default:
            break;
        }
    }
    
    qemu_log_mask(LOG_GUEST_ERROR, 
                  "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    return 0;
}

static void csi2_loopback_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CSI2V4L2Loopback *dev = CSI2_V4L2_LOOPBACK(opaque);
    
    if (offset < 0x1000) {
        /* CSI2 Controller registers */
        csi2_rx_controller_write(dev->csi2_controller, offset, value, size);
    } else if (offset >= 0x2000 && offset < 0x3000) {
        /* V4L2 Bridge registers */
        csi2_v4l2_bridge_write(dev->v4l2_bridge, offset - 0x2000, value, size);
    } else if (offset >= 0x3000 && offset < 0x4000) {
        /* Loopback device control */
        switch (offset - 0x3000) {
        case 0x00: /* Control register */
            if (value & 1) {
                /* Start streaming */
                if (!dev->streaming) {
                    csi2_loopback_start(dev);
                }
            } else {
                /* Stop streaming */
                if (dev->streaming) {
                    csi2_loopback_stop(dev);
                }
            }
            
            if (value & 2) {
                dev->v4l2_enabled = true;
            } else {
                dev->v4l2_enabled = false;
            }
            
            /* Frame rate control */
            uint32_t new_rate = (value >> 8) & 0xFF;
            if (new_rate > 0 && new_rate <= 120) {
                dev->v4l2_frame_rate = new_rate;
            }
            break;
            
        case 0x04: /* Reset frame counter */
            if (value == 0xDEADBEEF) {
                dev->frames_generated = 0;
                if (dev->csi2_controller) {
                    dev->csi2_controller->packet_count = 0;
                }
                if (dev->v4l2_bridge) {
                    dev->v4l2_bridge->frames_captured = 0;
                    dev->v4l2_bridge->sequence_number = 0;
                }
                printf("CSI2-V4L2-Loopback: Statistics reset\n");
            }
            break;
            
        case 0x08: /* Frame size configuration */
            dev->frame_width = (value >> 16) & 0xFFFF;
            dev->frame_height = value & 0xFFFF;
            printf("CSI2-V4L2-Loopback: Frame size set to %dx%d\n",
                   dev->frame_width, dev->frame_height);
            break;
            
        case 0x0C: /* Pixel format */
            dev->pixel_format = value;
            printf("CSI2-V4L2-Loopback: Pixel format set to 0x%08x\n",
                   dev->pixel_format);
            break;
            
        default:
            qemu_log_mask(LOG_GUEST_ERROR, 
                          "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
            break;
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    }
}

/* Start loopback streaming */
static int csi2_loopback_start(CSI2V4L2Loopback *dev)
{
    if (dev->streaming) {
        return 0; /* Already started */
    }
    
    dev->streaming = true;
    dev->frames_generated = 0;
    
    /* Start frame generation timer */
    uint64_t interval_ns = 1000000000ULL / dev->v4l2_frame_rate;
    timer_mod(dev->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + interval_ns);
    
    /* Start status monitoring */
    timer_mod(dev->status_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000000LL);
    
    printf("CSI2-V4L2-Loopback: Streaming started (%dfps, %dx%d)\n",
           dev->v4l2_frame_rate, dev->frame_width, dev->frame_height);
    
    return 0;
}

/* Stop loopback streaming */
static void csi2_loopback_stop(CSI2V4L2Loopback *dev)
{
    if (!dev->streaming) {
        return; /* Already stopped */
    }
    
    dev->streaming = false;
    
    /* Stop timers */
    timer_del(dev->frame_timer);
    timer_del(dev->status_timer);
    
    printf("CSI2-V4L2-Loopback: Streaming stopped (generated %d frames)\n",
           dev->frames_generated);
}

/* Public API functions */
int csi2_loopback_device_start(CSI2V4L2Loopback *dev)
{
    return csi2_loopback_start(dev);
}

void csi2_loopback_device_stop(CSI2V4L2Loopback *dev)
{
    csi2_loopback_stop(dev);
}

/* Memory region operations */
static const MemoryRegionOps csi2_loopback_ops = {
    .read = csi2_loopback_read,
    .write = csi2_loopback_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* Device realization */
static void csi2_loopback_realize(DeviceState *dev, Error **errp)
{
    CSI2V4L2Loopback *s = CSI2_V4L2_LOOPBACK(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    Error *local_err = NULL;
    
    /* Initialize memory region and IRQ */
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_loopback_ops, s, 
                          "csi2-v4l2-loopback", 0x4000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    
    /* Create sub-components */
    s->csi2_controller = CSI2_RX_CONTROLLER(qdev_new(TYPE_CSI2_RX_CONTROLLER));
    s->v4l2_bridge = CSI2_V4L2_BRIDGE(qdev_new(TYPE_CSI2_V4L2_BRIDGE));
    
    /* Configure sub-components */
    s->csi2_controller->num_lanes = s->num_lanes;
    s->csi2_controller->default_line_rate = s->line_rate;
    
    /* Realize sub-components */
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(s->csi2_controller), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(s->v4l2_bridge), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    /* Create timers */
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                  csi2_loopback_frame_timeout, s);
    s->status_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                   csi2_loopback_status_timeout, s);
    
    /* Auto-start if requested */
    if (s->auto_start) {
        csi2_loopback_start(s);
    }
    
    printf("CSI2-V4L2-Loopback: Device realized - %s (%dx%d, %d lanes, %d Mbps)\n",
           s->device_name ?: "QEMU MIPI CSI-2 Loopback",
           s->frame_width, s->frame_height, s->num_lanes, s->line_rate);
}

/* Device unrealization */
static void csi2_loopback_unrealize(DeviceState *dev)
{
    CSI2V4L2Loopback *s = CSI2_V4L2_LOOPBACK(dev);
    
    /* Stop streaming */
    csi2_loopback_stop(s);
    
    /* Free timers */
    if (s->frame_timer) {
        timer_free(s->frame_timer);
        s->frame_timer = NULL;
    }
    
    if (s->status_timer) {
        timer_free(s->status_timer);
        s->status_timer = NULL;
    }
    
    /* Clean up sub-components */
    if (s->csi2_controller) {
        object_unref(OBJECT(s->csi2_controller));
        s->csi2_controller = NULL;
    }
    
    if (s->v4l2_bridge) {
        object_unref(OBJECT(s->v4l2_bridge));
        s->v4l2_bridge = NULL;
    }
    
    printf("CSI2-V4L2-Loopback: Device unrealized\n");
}

/* Device reset */
static void csi2_loopback_reset(DeviceState *dev)
{
    CSI2V4L2Loopback *s = CSI2_V4L2_LOOPBACK(dev);
    
    /* Stop streaming */
    csi2_loopback_stop(s);
    
    /* Reset sub-components */
    if (s->csi2_controller) {
        device_cold_reset(DEVICE(s->csi2_controller));
    }
    if (s->v4l2_bridge) {
        device_cold_reset(DEVICE(s->v4l2_bridge));
    }
    
    /* Reset runtime state */
    s->frames_generated = 0;
    s->streaming = false;
    
    printf("CSI2-V4L2-Loopback: Device reset\n");
}

/* Device properties */
static Property csi2_loopback_properties[] = {
    DEFINE_PROP_STRING("device-name", CSI2V4L2Loopback, device_name),
    DEFINE_PROP_UINT32("num-lanes", CSI2V4L2Loopback, num_lanes, 4),
    DEFINE_PROP_UINT32("line-rate", CSI2V4L2Loopback, line_rate, 1000),
    DEFINE_PROP_UINT32("frame-width", CSI2V4L2Loopback, frame_width, 1280),
    DEFINE_PROP_UINT32("frame-height", CSI2V4L2Loopback, frame_height, 720),
    DEFINE_PROP_UINT32("pixel-format", CSI2V4L2Loopback, pixel_format, 0x32424752), /* RGB24 */
    DEFINE_PROP_UINT32("frame-rate", CSI2V4L2Loopback, v4l2_frame_rate, 30),
    DEFINE_PROP_BOOL("auto-start", CSI2V4L2Loopback, auto_start, true),
    DEFINE_PROP_BOOL("v4l2-enabled", CSI2V4L2Loopback, v4l2_enabled, true),
    {}
};

/* Instance initialization */
static void csi2_loopback_instance_init(Object *obj)
{
    CSI2V4L2Loopback *s = CSI2_V4L2_LOOPBACK(obj);
    
    /* Set default values */
    s->device_name = NULL;
    s->num_lanes = 4;
    s->line_rate = 1000;
    s->frame_width = 1280;
    s->frame_height = 720;
    s->pixel_format = 0x32424752; /* V4L2_PIX_FMT_RGB24 */
    s->v4l2_frame_rate = 30;
    s->auto_start = true;
    s->v4l2_enabled = true;
    
    /* Initialize runtime state */
    s->streaming = false;
    s->frames_generated = 0;
    s->csi2_controller = NULL;
    s->v4l2_bridge = NULL;
    s->frame_timer = NULL;
    s->status_timer = NULL;
}

/* Class initialization */
static void csi2_loopback_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = csi2_loopback_realize;
    dc->unrealize = csi2_loopback_unrealize;
    device_class_set_legacy_reset(dc, csi2_loopback_reset);
    dc->props_ = csi2_loopback_properties;
    dc->desc = "MIPI CSI-2 V4L2 Loopback Device";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

/* Type information */
static const TypeInfo csi2_loopback_info = {
    .name = TYPE_CSI2_V4L2_LOOPBACK,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CSI2V4L2Loopback),
    .instance_init = csi2_loopback_instance_init,
    .class_init = csi2_loopback_class_init,
};

/* Type registration */
static void csi2_loopback_register_types(void)
{
    type_register_static(&csi2_loopback_info);
}

type_init(csi2_loopback_register_types)
