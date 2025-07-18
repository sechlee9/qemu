// hw/misc/csi2_v4l2_integration.c - CSI2와 V4L2 백엔드 통합
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "system/v4l2.h"
#include "include/hw/misc/csi2_rx_controller.h"
#include "include/hw/misc/csi2_v4l2_bridge.h"

#define TYPE_CSI2_V4L2_DEVICE "csi2-v4l2-device"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2V4L2Device, CSI2_V4L2_DEVICE)

typedef struct CSI2V4L2Device {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    
    /* Sub-components */
    CSI2RxController *csi2_controller;
    CSI2V4L2Bridge *v4l2_bridge;
    V4L2Backend *v4l2_backend;
    
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
    QEMUTimer *status_timer;
    
} CSI2V4L2Device;

/* Frame callback from V4L2 backend */
static void csi2_v4l2_frame_callback(V4L2Frame *frame, void *opaque)
{
    CSI2V4L2Device *dev = CSI2_V4L2_DEVICE(opaque);
    
    /* Update CSI2 controller statistics */
    if (dev->csi2_controller) {
        dev->csi2_controller->packet_count++;
        dev->csi2_controller->core_status = 
            (dev->csi2_controller->packet_count << 16) & 0xFFFF0000;
        
        /* Update image info for VC0 */
        dev->csi2_controller->img_info1[0] = 
            (dev->csi2_controller->packet_count << 16) | frame->width;
        dev->csi2_controller->img_info2[0] = frame->format & 0x3F;
        
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
        printf("CSI2-V4L2: Generated frame %d (%dx%d, %s) -> %s\n",
               dev->frames_generated, frame->width, frame->height,
               v4l2_format_to_string(frame->format),
               v4l2_backend_get_device_path(dev->v4l2_backend));
    }
}

/* Status monitoring timer */
static void csi2_v4l2_status_timeout(void *opaque)
{
    CSI2V4L2Device *dev = CSI2_V4L2_DEVICE(opaque);
    
    /* Periodic status check */
    if (dev->streaming && dev->v4l2_backend) {
        const char *device_path = v4l2_backend_get_device_path(dev->v4l2_backend);
        if (device_path && dev->frames_generated % 300 == 0) { /* Every 10 seconds */
            printf("CSI2-V4L2: Active streaming to %s (%d frames)\n",
                   device_path, dev->frames_generated);
        }
    }
    
    /* Schedule next check */
    timer_mod(dev->status_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000000LL);
}

static uint64_t csi2_v4l2_read(void *opaque, hwaddr offset, unsigned size)
{
    CSI2V4L2Device *dev = CSI2_V4L2_DEVICE(opaque);
    
    if (offset < 0x1000) {
        /* CSI2 Controller registers */
        return csi2_rx_controller_read(dev->csi2_controller, offset, size);
    } else if (offset >= 0x2000 && offset < 0x3000) {
        /* V4L2 Bridge registers */
        return csi2_v4l2_bridge_read(dev->v4l2_bridge, offset - 0x2000, size);
    } else if (offset >= 0x3000 && offset < 0x4000) {
        /* V4L2 Device specific registers */
        switch (offset - 0x3000) {
        case 0x00: /* Device status */
            return dev->streaming ? 1 : 0;
        case 0x04: /* Device path length */
            if (dev->v4l2_backend) {
                const char *path = v4l2_backend_get_device_path(dev->v4l2_backend);
                return path ? strlen(path) : 0;
            }
            return 0;
        case 0x08: /* Frames generated */
            return dev->frames_generated;
        case 0x0C: /* Configuration */
            return (dev->frame_width << 16) | dev->frame_height;
        default:
            break;
        }
    }
    
    qemu_log_mask(LOG_GUEST_ERROR, 
                  "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    return 0;
}

static void csi2_v4l2_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CSI2V4L2Device *dev = CSI2_V4L2_DEVICE(opaque);
    Error *local_err = NULL;
    
    if (offset < 0x1000) {
        /* CSI2 Controller registers */
        csi2_rx_controller_write(dev->csi2_controller, offset, value, size);
    } else if (offset >= 0x2000 && offset < 0x3000) {
        /* V4L2 Bridge registers */
        csi2_v4l2_bridge_write(dev->v4l2_bridge, offset - 0x2000, value, size);
    } else if (offset >= 0x3000 && offset < 0x4000) {
        /* V4L2 Device control */
        switch (offset - 0x3000) {
        case 0x00: /* Start/Stop streaming */
            if (value & 1) {
                if (!dev->streaming && dev->v4l2_backend) {
                    if (v4l2_backend_start(dev->v4l2_backend, &local_err) == 0) {
                        dev->streaming = true;
                        printf("CSI2-V4L2: Streaming started to %s\n",
                               v4l2_backend_get_device_path(dev->v4l2_backend));
                    } else {
                        error_report_err(local_err);
                    }
                }
            } else {
                if (dev->streaming && dev->v4l2_backend) {
                    v4l2_backend_stop(dev->v4l2_backend);
                    dev->streaming = false;
                    printf("CSI2-V4L2: Streaming stopped\n");
                }
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
                printf("CSI2-V4L2: Statistics reset\n");
            }
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

static const MemoryRegionOps csi2_v4l2_ops = {
    .read = csi2_v4l2_read,
    .write = csi2_v4l2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void csi2_v4l2_realize(DeviceState *dev, Error **errp)
{
    CSI2V4L2Device *s = CSI2_V4L2_DEVICE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    Error *local_err = NULL;
    
    /* Initialize memory region and IRQ */
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_v4l2_ops, s, 
                          "csi2-v4l2", 0x4000);
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
    
    /* Create V4L2 backend */
    char *device_name = g_strdup_printf("%s (%dx%d)", 
                                       s->device_name ?: "QEMU MIPI CSI-2",
                                       s->frame_width, s->frame_height);
    
    s->v4l2_backend = v4l2_backend_new(device_name, &local_err);
    g_free(device_name);
    
    if (!s->v4l2_backend) {
        error_propagate(errp, local_err);
        return;
    }
    
    /* Set frame callback */
    v4l2_backend_set_frame_callback(s->v4l2_backend, 
                                   csi2_v4l2_frame_callback, s);
    
    /* Create status timer */
    s->status_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                  csi2_v4l2_status_timeout, s);
    
    /* Auto-start if requested */
    if (s->auto_start) {
        if (v4l2_backend_start(s->v4l2_backend, &local_err) == 0) {
            s->streaming = true;
            printf("CSI2-V4L2: Auto-started streaming to %s\n",
                   v4l2_backend_get_device_path(s->v4l2_backend));
        } else {
            error_report_err(local_err);
        }
    }
    
    /* Start status monitoring */
    timer_mod(s->status_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000000LL);
    
    printf("CSI2-V4L2 Device: Realized with V4L2 device: %s\n",
           v4l2_backend_get_device_path(s->v4l2_backend) ?: "pending");
}

static void csi2_v4l2_unrealize(DeviceState *dev)
{
    CSI2V4L2Device *s = CSI2_V4L2_DEVICE(dev);
    
    /* Stop status timer */
    if (s->status_timer) {
        timer_free(s->status_timer);
        s->status_timer = NULL;
    }
    
    /* Stop streaming */
    if (s->streaming && s->v4l2_backend) {
        v4l2_backend_stop(s->v4l2_backend);
        s->streaming = false;
    }
    
    /* Clean up V4L2 backend */
    if (s->v4l2_backend) {
        v4l2_backend_free(s->v4l2_backend);
        s->v4l2_backend = NULL;
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
    
    printf("CSI2-V4L2 Device: Unrealized\n");
}

static void csi2_v4l2_reset(DeviceState *dev)
{
    CSI2V4L2Device *s = CSI2_V4L2_DEVICE(dev);
    
    /* Reset sub-components */
    if (s->csi2_controller) {
        device_cold_reset(DEVICE(s->csi2_controller));
    }
    if (s->v4l2_bridge) {
        device_cold_reset(DEVICE(s->v4l2_bridge));
    }
    
    /* Reset runtime state */
    s->frames_generated = 0;
    
    printf("CSI2-V4L2 Device: Reset\n");
}

static Property csi2_v4l2_properties[] = {
    DEFINE_PROP_STRING("device-name", CSI2V4L2Device, device_name),
    DEFINE_PROP_UINT32("num-lanes", CSI2V4L2Device, num_lanes, 4),
    DEFINE_PROP_UINT32("line-rate", CSI2V4L2Device, line_rate, 1000),
    DEFINE_PROP_UINT32("frame-width", CSI2V4L2Device, frame_width, 1280),
    DEFINE_PROP_UINT32("frame-height", CSI2V4L2Device, frame_height, 720),
    DEFINE_PROP_UINT32("pixel-format", CSI2V4L2Device, pixel_format, 0x32424752), /* RGB24 */
    DEFINE_PROP_BOOL("auto-start", CSI2V4L2Device, auto_start, true),
    {}
};

static void csi2_v4l2_instance_init(Object *obj)
{
    CSI2V4L2Device *s = CSI2_V4L2_DEVICE(obj);
    
    /* Set default values */
    s->device_name = NULL;
    s->num_lanes = 4;
    s->line_rate = 1000;
    s->frame_width = 1280;
    s->frame_height = 720;
    s->pixel_format = 0x32424752; /* V4L2_PIX_FMT_RGB24 */
    s->auto_start = true;
    
    /* Initialize runtime state */
    s->streaming = false;
    s->frames_generated = 0;
    s->csi2_controller = NULL;
    s->v4l2_bridge = NULL;
    s->v4l2_backend = NULL;
    s->status_timer = NULL;
}

static void csi2_v4l2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = csi2_v4l2_realize;
    dc->unrealize = csi2_v4l2_unrealize;
    device_class_set_legacy_reset(dc, csi2_v4l2_reset);
    dc->props_ = csi2_v4l2_properties;
    dc->desc = "MIPI CSI-2 with V4L2 Backend";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo csi2_v4l2_device_info = {
    .name = TYPE_CSI2_V4L2_DEVICE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CSI2V4L2Device),
    .instance_init = csi2_v4l2_instance_init,
    .class_init = csi2_v4l2_class_init,
};

static void csi2_v4l2_register_types(void)
{
    type_register_static(&csi2_v4l2_device_info);
}

type_init(csi2_v4l2_register_types)
