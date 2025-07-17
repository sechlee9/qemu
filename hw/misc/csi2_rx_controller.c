#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/misc/csi2_rx_controller.h"

static void csi2_rx_controller_frame_timeout(void *opaque)
{
    CSI2RxController *s = CSI2_RX_CONTROLLER(opaque);
    
    /* Increment packet count in Core Status register */
    s->packet_count++;
    s->core_status = (s->packet_count << 16) & 0xFFFF0000;
    
    /* Update image info for VC0 */
    s->img_info1[0] = (s->packet_count << 16) | 1280; /* Line count | Byte count */
    s->img_info2[0] = 0x2A; /* RAW10 data type */
    
    /* Set frame received interrupt */
    s->int_status |= CSI2_INT_FRAME_RECEIVED;
    
    /* Generate interrupt if enabled */
    if (s->global_int_enable && (s->int_status & s->int_enable)) {
        qemu_set_irq(s->irq, 1);
    }
    
    /* Print debug info every 30 frames */
    if (s->packet_count % 30 == 1) {
        printf("CSI2 RX: Generated frame %d, Status: 0x%08x\n", 
               s->packet_count, s->core_status);
    }
    
    /* Continue if core is enabled */
    if (s->core_config & 1) {
        timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
    }
}

uint64_t csi2_rx_controller_read(void *opaque, hwaddr offset, unsigned size)
{
    CSI2RxController *s = CSI2_RX_CONTROLLER(opaque);
    uint64_t ret = 0;
    
    switch (offset) {
    case 0x00: /* Core config */
        ret = s->core_config;
        break;
    case 0x04: /* Protocol config */
        ret = s->protocol_config;
        break;
    case 0x10: /* Core status */
        ret = s->core_status;
        break;
    case 0x20: /* Global interrupt enable */
        ret = s->global_int_enable;
        break;
    case 0x24: /* Interrupt status */
        ret = s->int_status;
        break;
    case 0x28: /* Interrupt enable */
        ret = s->int_enable;
        break;
    case 0x2C: /* Dynamic VC selection */
        ret = s->dynamic_vc_sel;
        break;
    case 0x30: /* Generic short packet */
        ret = s->generic_short_pkt;
        break;
    case 0x60: /* Image info 1 VC0 */
        ret = s->img_info1[0];
        break;
    case 0x64: /* Image info 2 VC0 */
        ret = s->img_info2[0];
        break;
    default:
        if (offset >= 0x60 && offset <= 0xDC) {
            /* Image info registers for VC1-VC15 */
            int vc = (offset - 0x60) / 8;
            int reg = (offset - 0x60) % 8;
            if (vc < CSI2_MAX_VC) {
                ret = (reg == 0) ? s->img_info1[vc] : s->img_info2[vc];
            }
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, 
                          "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
        }
        break;
    }
    return ret;
}

void csi2_rx_controller_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CSI2RxController *s = CSI2_RX_CONTROLLER(opaque);
    
    switch (offset) {
    case 0x00: /* Core config */
        s->core_config = value & 0x7;
        if (value & 1) { /* Enable bit */
            printf("CSI2 RX: Core ENABLED\n");
            /* Reset counters when enabling */
            s->packet_count = 0;
            s->core_status = 0;
            /* Start frame generation */
            timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
        } else {
            printf("CSI2 RX: Core DISABLED\n");
            timer_del(s->frame_timer);
        }
        break;
    case 0x04: /* Protocol config */
        s->protocol_config = value;
        printf("CSI2 RX: Protocol config set to 0x%08x\n", (uint32_t)value);
        break;
    case 0x20: /* Global interrupt enable */
        s->global_int_enable = value & 0x1;
        if (s->global_int_enable && (s->int_status & s->int_enable)) {
            qemu_set_irq(s->irq, 1);
        } else {
            qemu_set_irq(s->irq, 0);
        }
        break;
    case 0x24: /* Interrupt status - write 1 to clear */
        s->int_status &= ~value;
        if (s->global_int_enable && (s->int_status & s->int_enable)) {
            qemu_set_irq(s->irq, 1);
        } else {
            qemu_set_irq(s->irq, 0);
        }
        break;
    case 0x28: /* Interrupt enable */
        s->int_enable = value;
        if (s->global_int_enable && (s->int_status & s->int_enable)) {
            qemu_set_irq(s->irq, 1);
        } else {
            qemu_set_irq(s->irq, 0);
        }
        break;
    case 0x2C: /* Dynamic VC selection */
        s->dynamic_vc_sel = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
        break;
    }
}

static const MemoryRegionOps csi2_rx_controller_ops = {
    .read = csi2_rx_controller_read,
    .write = csi2_rx_controller_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void csi2_rx_controller_realize(DeviceState *dev, Error **errp)
{
    CSI2RxController *s = CSI2_RX_CONTROLLER(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_rx_controller_ops, s, 
                          "csi2-rx", 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                  csi2_rx_controller_frame_timeout, s);
    
    printf("CSI2 RX Controller: Device realized\n");
}

static void csi2_rx_controller_unrealize(DeviceState *dev)
{
    CSI2RxController *s = CSI2_RX_CONTROLLER(dev);
    
    if (s->frame_timer) {
        timer_free(s->frame_timer);
        s->frame_timer = NULL;
    }
}

static void csi2_rx_controller_reset(DeviceState *dev)
{
    CSI2RxController *s = CSI2_RX_CONTROLLER(dev);
    int i;
    
    s->core_config = 1; /* Enabled by default */
    s->protocol_config = (s->num_lanes << 3) | s->num_lanes;
    s->core_status = 0;
    s->global_int_enable = 0;
    s->int_status = 0;
    s->int_enable = 0;
    s->dynamic_vc_sel = 0xFFFF;
    s->generic_short_pkt = 0;
    s->packet_count = 0;
    
    for (i = 0; i < CSI2_MAX_VC; i++) {
        s->img_info1[i] = 0;
        s->img_info2[i] = 0;
    }
    
    if (s->frame_timer) {
        timer_del(s->frame_timer);
    }
    
    printf("CSI2 RX Controller: Device reset\n");
}

static void csi2_rx_controller_instance_init(Object *obj)
{
    CSI2RxController *s = CSI2_RX_CONTROLLER(obj);
    
    /* Set default values */
    s->num_lanes = 4;
    s->default_line_rate = 1000;
    s->pixels_per_clock = 1;
    s->vfb_enabled = true;
    s->embedded_enabled = false;
    
    /* Initialize runtime state */
    s->packet_count = 0;
    s->frame_timer = NULL;
}

static void csi2_rx_controller_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = csi2_rx_controller_realize;
    dc->unrealize = csi2_rx_controller_unrealize;
    device_class_set_legacy_reset(dc, csi2_rx_controller_reset);
    dc->desc = "MIPI CSI-2 RX Controller";
}

static const TypeInfo csi2_rx_controller_info = {
    .name = TYPE_CSI2_RX_CONTROLLER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CSI2RxController),
    .instance_init = csi2_rx_controller_instance_init,
    .class_init = csi2_rx_controller_class_init,
};

static void csi2_rx_controller_register_types(void)
{
    type_register_static(&csi2_rx_controller_info);
}

type_init(csi2_rx_controller_register_types)
