// hw/misc/csi2_unified.c - 통합된 CSI2 구현
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "include/hw/misc/csi2_unified.h"

/* PCIe 디바이스 ID */
#define CSI2_VENDOR_ID    0x1234
#define CSI2_DEVICE_ID    0x5678

/* ========================================================================
 * CSI2 통합 디바이스 구현 (SysBus)
 * ======================================================================== */

static void csi2_frame_timeout(void *opaque)
{
    CSI2UnifiedDevice *s = CSI2_UNIFIED_DEVICE(opaque);
    
    /* CSI2 Controller 상태 업데이트 */
    s->packet_count++;
    s->core_status = (s->packet_count << 16) & 0xFFFF0000;
    
    /* VC0 이미지 정보 업데이트 */
    s->img_info1[0] = (s->packet_count << 16) | s->frame_width;
    s->img_info2[0] = s->pixel_format & 0x3F;
    
    /* V4L2 Bridge 상태 업데이트 */
    if (s->v4l2_streaming) {
        s->v4l2_frames_captured++;
        s->v4l2_sequence_number++;
    }
    
    /* 프레임 수신 인터럽트 설정 */
    s->int_status |= CSI2_INT_FRAME_RECEIVED;
    
    /* 인터럽트 생성 */
    if (s->global_int_enable && (s->int_status & s->int_enable)) {
        qemu_set_irq(s->irq, 1);
    }
    
    /* 디버그 출력 (30 프레임마다) */
    if (s->packet_count % 30 == 1) {
        printf("CSI2 Unified: Frame %d generated (V4L2: %s)\n", 
               s->packet_count, s->v4l2_streaming ? "streaming" : "stopped");
    }
    
    /* 다음 프레임 스케줄 (30 FPS) */
    if (s->core_config & 1) {
        timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
    }
}

/* CSI2 Controller 레지스터 읽기 */
static uint64_t csi2_controller_read(CSI2UnifiedDevice *s, hwaddr offset)
{
    switch (offset) {
    case CSI2_CORE_CONFIG:
        return s->core_config;
    case CSI2_PROTOCOL_CONFIG:
        return s->protocol_config;
    case CSI2_CORE_STATUS:
        return s->core_status;
    case CSI2_GLOBAL_INT_ENABLE:
        return s->global_int_enable;
    case CSI2_INT_STATUS:
        return s->int_status;
    case CSI2_INT_ENABLE:
        return s->int_enable;
    case CSI2_DYNAMIC_VC_SEL:
        return s->dynamic_vc_sel;
    case CSI2_GENERIC_SHORT_PKT:
        return s->generic_short_pkt;
    case CSI2_VCX_FRAME_ERROR:
        return s->vcx_frame_error;
    case CSI2_CLOCK_LANE_INFO:
        return s->clock_lane_info;
    default:
        if (offset >= CSI2_LANE0_INFO && offset < CSI2_LANE0_INFO + (CSI2_MAX_LANES * 4)) {
            int lane = (offset - CSI2_LANE0_INFO) / 4;
            return s->lane_info[lane];
        }
        if (offset >= CSI2_IMG_INFO1_VC0 && offset <= 0xDC) {
            int vc = (offset - CSI2_IMG_INFO1_VC0) / 8;
            int reg = (offset - CSI2_IMG_INFO1_VC0) % 8;
            if (vc < CSI2_MAX_VC) {
                return (reg == 0) ? s->img_info1[vc] : s->img_info2[vc];
            }
        }
        break;
    }
    return 0;
}

/* CSI2 Controller 레지스터 쓰기 */
static void csi2_controller_write(CSI2UnifiedDevice *s, hwaddr offset, uint64_t value)
{
    switch (offset) {
    case CSI2_CORE_CONFIG:
        s->core_config = value & 0x7;
        if (value & 1) { /* Enable */
            printf("CSI2 Unified: Controller ENABLED\n");
            s->packet_count = 0;
            s->core_status = 0;
            timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
        } else {
            printf("CSI2 Unified: Controller DISABLED\n");
            timer_del(s->frame_timer);
        }
        break;
    case CSI2_PROTOCOL_CONFIG:
        s->protocol_config = value;
        break;
    case CSI2_GLOBAL_INT_ENABLE:
        s->global_int_enable = value & 0x1;
        break;
    case CSI2_INT_STATUS: /* Write 1 to clear */
        s->int_status &= ~value;
        if (!(s->int_status & s->int_enable)) {
            qemu_set_irq(s->irq, 0);
        }
        break;
    case CSI2_INT_ENABLE:
        s->int_enable = value;
        break;
    case CSI2_DYNAMIC_VC_SEL:
        s->dynamic_vc_sel = value;
        break;
    default:
        break;
    }
}

/* D-PHY 레지스터 읽기 */
static uint64_t csi2_dphy_read(CSI2UnifiedDevice *s, hwaddr offset)
{
    switch (offset - CSI2_DPHY_BASE) {
    case 0x00: /* Control */
        return s->dphy.ctrl;
    case 0x04: /* Status */
        return s->dphy.status | (s->dphy.pll_locked ? 1 : 0);
    case 0x08: /* Line rate */
        return s->dphy.line_rate;
    case 0x0C: /* Number of lanes */
        return s->dphy.num_lanes;
    default:
        break;
    }
    return 0;
}

/* D-PHY 레지스터 쓰기 */
static void csi2_dphy_write(CSI2UnifiedDevice *s, hwaddr offset, uint64_t value)
{
    switch (offset - CSI2_DPHY_BASE) {
    case 0x00: /* Control */
        s->dphy.ctrl = value;
        if (value & 1) { /* Enable */
            s->dphy.pll_locked = true;
            s->dphy.status |= 1;
        } else {
            s->dphy.pll_locked = false;
            s->dphy.status &= ~1;
        }
        break;
    case 0x08: /* Line rate */
        s->dphy.line_rate = value;
        break;
    case 0x0C: /* Number of lanes */
        if (value <= 4) {
            s->dphy.num_lanes = value;
        }
        break;
    default:
        break;
    }
}

/* V4L2 Bridge 레지스터 읽기 */
static uint64_t csi2_v4l2_read(CSI2UnifiedDevice *s, hwaddr offset)
{
    switch (offset - CSI2_V4L2_BASE) {
    case 0x00: /* Streaming */
        return s->v4l2_streaming ? 1 : 0;
    case 0x04: /* Frames captured */
        return s->v4l2_frames_captured;
    case 0x08: /* Sequence number */
        return s->v4l2_sequence_number;
    case 0x0C: /* Frames dropped */
        return s->v4l2_frames_dropped;
    case 0x10: /* Buffer overruns */
        return s->v4l2_buffer_overruns;
    default:
        break;
    }
    return 0;
}

/* V4L2 Bridge 레지스터 쓰기 */
static void csi2_v4l2_write(CSI2UnifiedDevice *s, hwaddr offset, uint64_t value)
{
    switch (offset - CSI2_V4L2_BASE) {
    case 0x00: /* Streaming control */
        s->v4l2_streaming = (value & 1) != 0;
        printf("CSI2 Unified: V4L2 streaming %s\n", 
               s->v4l2_streaming ? "ENABLED" : "DISABLED");
        break;
    case 0x08: /* Reset sequence */
        if (value == 0) {
            s->v4l2_sequence_number = 0;
            printf("CSI2 Unified: V4L2 sequence reset\n");
        }
        break;
    default:
        break;
    }
}

/* Control 레지스터 읽기/쓰기 */
static uint64_t csi2_control_read(CSI2UnifiedDevice *s, hwaddr offset)
{
    switch (offset - CSI2_CONTROL_BASE) {
    case 0x00: /* Global enable */
        return (s->core_config & 1) && s->v4l2_streaming;
    case 0x04: /* Reset status */
        return 0; /* Always ready */
    case 0x08: /* Frame size */
        return (s->frame_width << 16) | s->frame_height;
    case 0x0C: /* Pixel format */
        return s->pixel_format;
    default:
        break;
    }
    return 0;
}

static void csi2_control_write(CSI2UnifiedDevice *s, hwaddr offset, uint64_t value)
{
    switch (offset - CSI2_CONTROL_BASE) {
    case 0x00: /* Global enable */
        if (value & 1) {
            /* Enable both controller and V4L2 */
            s->core_config |= 1;
            s->v4l2_streaming = true;
            timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
            printf("CSI2 Unified: Global ENABLE\n");
        } else {
            s->core_config &= ~1;
            s->v4l2_streaming = false;
            timer_del(s->frame_timer);
            printf("CSI2 Unified: Global DISABLE\n");
        }
        break;
    case 0x04: /* Reset */
        if (value == 0xDEADBEEF) {
            s->packet_count = 0;
            s->v4l2_frames_captured = 0;
            s->v4l2_sequence_number = 0;
            s->int_status = 0;
            printf("CSI2 Unified: System RESET\n");
        }
        break;
    case 0x08: /* Frame size */
        s->frame_width = (value >> 16) & 0xFFFF;
        s->frame_height = value & 0xFFFF;
        printf("CSI2 Unified: Frame size set to %dx%d\n", s->frame_width, s->frame_height);
        break;
    case 0x0C: /* Pixel format */
        s->pixel_format = value;
        break;
    default:
        break;
    }
}

/* 통합 MMIO 읽기 */
uint64_t csi2_unified_read(void *opaque, hwaddr offset, unsigned size)
{
    CSI2UnifiedDevice *s = CSI2_UNIFIED_DEVICE(opaque);
    
    if (CSI2_IS_CONTROLLER_REG(offset)) {
        return csi2_controller_read(s, offset);
    } else if (CSI2_IS_DPHY_REG(offset)) {
        return csi2_dphy_read(s, offset);
    } else if (CSI2_IS_V4L2_REG(offset)) {
        return csi2_v4l2_read(s, offset);
    } else if (CSI2_IS_CONTROL_REG(offset)) {
        return csi2_control_read(s, offset);
    }
    
    qemu_log_mask(LOG_GUEST_ERROR, 
                  "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    return 0;
}

/* 통합 MMIO 쓰기 */
void csi2_unified_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CSI2UnifiedDevice *s = CSI2_UNIFIED_DEVICE(opaque);
    
    if (CSI2_IS_CONTROLLER_REG(offset)) {
        csi2_controller_write(s, offset, value);
    } else if (CSI2_IS_DPHY_REG(offset)) {
        csi2_dphy_write(s, offset, value);
    } else if (CSI2_IS_V4L2_REG(offset)) {
        csi2_v4l2_write(s, offset, value);
    } else if (CSI2_IS_CONTROL_REG(offset)) {
        csi2_control_write(s, offset, value);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    }
}

static const MemoryRegionOps csi2_unified_ops = {
    .read = csi2_unified_read,
    .write = csi2_unified_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* CSI2 통합 디바이스 구현 */
static void csi2_unified_realize(DeviceState *dev, Error **errp)
{
    CSI2UnifiedDevice *s = CSI2_UNIFIED_DEVICE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_unified_ops, s, 
                          "csi2-unified", 0x4000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, csi2_frame_timeout, s);
    
    printf("CSI2 Unified Device: Realized (%d lanes, %d Mb/s, %dx%d)\n",
           s->num_lanes, s->default_line_rate, s->frame_width, s->frame_height);
}

static void csi2_unified_unrealize(DeviceState *dev)
{
    CSI2UnifiedDevice *s = CSI2_UNIFIED_DEVICE(dev);
    
    if (s->frame_timer) {
        timer_free(s->frame_timer);
        s->frame_timer = NULL;
    }
}

static void csi2_unified_reset(DeviceState *dev)
{
    CSI2UnifiedDevice *s = CSI2_UNIFIED_DEVICE(dev);
    int i;
    
    /* CSI2 Controller 리셋 */
    s->core_config = 1; /* Enabled by default */
    s->protocol_config = (s->num_lanes << 3) | s->num_lanes;
    s->core_status = 0;
    s->global_int_enable = 0;
    s->int_status = 0;
    s->int_enable = 0;
    s->dynamic_vc_sel = 0xFFFF;
    s->generic_short_pkt = 0;
    s->vcx_frame_error = 0;
    s->clock_lane_info = 0;
    s->packet_count = 0;
    
    for (i = 0; i < CSI2_MAX_LANES; i++) {
        s->lane_info[i] = 0;
    }
    
    for (i = 0; i < CSI2_MAX_VC; i++) {
        s->img_info1[i] = 0;
        s->img_info2[i] = 0;
    }
    
    /* D-PHY 리셋 */
    s->dphy.ctrl = 0;
    s->dphy.status = 0;
    s->dphy.pll_locked = false;
    s->dphy.line_rate = s->dphy.default_line_rate;
    s->dphy.num_lanes = s->num_lanes;
    
    /* V4L2 Bridge 리셋 */
    s->v4l2_streaming = false;
    s->v4l2_sequence_number = 0;
    s->v4l2_frames_captured = 0;
    s->v4l2_frames_dropped = 0;
    s->v4l2_buffer_overruns = 0;
    
    if (s->frame_timer) {
        timer_del(s->frame_timer);
    }
    
    printf("CSI2 Unified Device: Reset\n");
}

static Property csi2_unified_properties[] = {
    DEFINE_PROP_UINT32("num-lanes", CSI2UnifiedDevice, num_lanes, 4),
    DEFINE_PROP_UINT32("line-rate", CSI2UnifiedDevice, default_line_rate, 1000),
    DEFINE_PROP_UINT32("frame-width", CSI2UnifiedDevice, frame_width, 1280),
    DEFINE_PROP_UINT32("frame-height", CSI2UnifiedDevice, frame_height, 720),
    DEFINE_PROP_UINT32("pixel-format", CSI2UnifiedDevice, pixel_format, 0x32424752), /* RGB24 */
    DEFINE_PROP_BOOL("vfb-enabled", CSI2UnifiedDevice, vfb_enabled, true),
    DEFINE_PROP_BOOL("embedded-enabled", CSI2UnifiedDevice, embedded_enabled, false),
    DEFINE_PROP_UINT32("pixels-per-clock", CSI2UnifiedDevice, pixels_per_clock, 1),
    {}
};

static void csi2_unified_instance_init(Object *obj)
{
    CSI2UnifiedDevice *s = CSI2_UNIFIED_DEVICE(obj);
    
    /* 기본값 설정 */
    s->num_lanes = 4;
    s->default_line_rate = 1000;
    s->frame_width = 1280;
    s->frame_height = 720;
    s->pixel_format = 0x32424752; /* V4L2_PIX_FMT_RGB24 */
    s->vfb_enabled = true;
    s->embedded_enabled = false;
    s->pixels_per_clock = 1;
    
    /* D-PHY 초기화 */
    s->dphy.default_line_rate = 1000;
    s->dphy.enable_deskew = false;
    
    /* 런타임 상태 초기화 */
    s->packet_count = 0;
    s->frame_timer = NULL;
}

static void csi2_unified_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = csi2_unified_realize;
    dc->unrealize = csi2_unified_unrealize;
    device_class_set_legacy_reset(dc, csi2_unified_reset);
    dc->props_ = csi2_unified_properties;
    dc->desc = "MIPI CSI-2 Unified Device";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo csi2_unified_info = {
    .name = TYPE_CSI2_UNIFIED_DEVICE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CSI2UnifiedDevice),
    .instance_init = csi2_unified_instance_init,
    .class_init = csi2_unified_class_init,
};

/* ========================================================================
 * CSI2 PCIe 디바이스 구현
 * ======================================================================== */

static uint64_t csi2_pcie_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(opaque);
    
    if (dev->csi2_device) {
        return csi2_unified_read(dev->csi2_device, addr, size);
    }
    return 0;
}

static void csi2_pcie_mmio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(opaque);
    
    if (dev->csi2_device) {
        csi2_unified_write(dev->csi2_device, addr, value, size);
    }
}

static const MemoryRegionOps csi2_pcie_mmio_ops = {
    .read = csi2_pcie_mmio_read,
    .write = csi2_pcie_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void csi2_pcie_realize(PCIDevice *pci_dev, Error **errp)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(pci_dev);
    Error *local_err = NULL;
    
    pci_config_set_interrupt_pin(pci_dev->config, 1);
    
    memory_region_init_io(&dev->mmio, OBJECT(dev), &csi2_pcie_mmio_ops, dev, 
                          "csi2-pcie", 0x10000);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_MEM_TYPE_64, &dev->mmio);
    
    /* 내장된 CSI2 디바이스 생성 */
    dev->csi2_device = CSI2_UNIFIED_DEVICE(qdev_new(TYPE_CSI2_UNIFIED_DEVICE));
    dev->csi2_device->num_lanes = dev->num_lanes;
    dev->csi2_device->default_line_rate = dev->line_rate;
    dev->csi2_device->frame_width = dev->frame_width;
    dev->csi2_device->frame_height = dev->frame_height;
    
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(dev->csi2_device), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    printf("CSI2 PCIe Device: Initialized (%d lanes, %d Mb/s, %dx%d)\n",
           dev->num_lanes, dev->line_rate, dev->frame_width, dev->frame_height);
}

static void csi2_pcie_exit(PCIDevice *pci_dev)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(pci_dev);
    
    if (dev->csi2_device) {
        object_unref(OBJECT(dev->csi2_device));
        dev->csi2_device = NULL;
    }
}

static void csi2_pcie_reset(DeviceState *qdev)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(qdev);
    
    if (dev->csi2_device) {
        device_cold_reset(DEVICE(dev->csi2_device));
    }
}

static Property csi2_pcie_properties[] = {
    DEFINE_PROP_UINT32("num-lanes", CSI2PCIeDevice, num_lanes, 4),
    DEFINE_PROP_UINT32("line-rate", CSI2PCIeDevice, line_rate, 1000),
    DEFINE_PROP_UINT32("frame-width", CSI2PCIeDevice, frame_width, 1280),
    DEFINE_PROP_UINT32("frame-height", CSI2PCIeDevice, frame_height, 720),
    {}
};

static void csi2_pcie_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);
    
    pc->realize = csi2_pcie_realize;
    pc->exit = csi2_pcie_exit;
    pc->vendor_id = CSI2_VENDOR_ID;
    pc->device_id = CSI2_DEVICE_ID;
    pc->class_id = 0x0400;  /* Multimedia controller */
    pc->revision = 0x01;
    
    dc->desc = "MIPI CSI-2 PCIe Camera Device";
    device_class_set_legacy_reset(dc, csi2_pcie_reset);
    dc->props_ = csi2_pcie_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo csi2_pcie_info = {
    .name = TYPE_CSI2_PCIE_DEVICE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(CSI2PCIeDevice),
    .class_init = csi2_pcie_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

/* 타입 등록 */
static void csi2_unified_register_types(void)
{
    type_register_static(&csi2_unified_info);
    type_register_static(&csi2_pcie_info);
}

type_init(csi2_unified_register_types)
