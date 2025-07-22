// hw/misc/csi2_loopback_pcie.c - CSI2 V4L2 Loopback PCIe Device
#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/pci/msi.h"
#include "include/hw/misc/csi2_v4l2_loopback.h"

#define TYPE_CSI2_LOOPBACK_PCIE "csi2-loopback-pcie"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2LoopbackPCIe, CSI2_LOOPBACK_PCIE)

#define CSI2_LOOPBACK_VENDOR_ID    0x1234
#define CSI2_LOOPBACK_DEVICE_ID    0x5679

typedef struct CSI2LoopbackPCIe {
    PCIDevice parent_obj;
    MemoryRegion mmio;
    
    /* Core loopback device */
    CSI2V4L2Loopback *loopback_dev;
    
    /* PCIe-specific properties */
    char *loopback_device;
    char *device_name;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t pixel_format;
    uint32_t fps;
    uint32_t num_lanes;
    uint32_t line_rate;
    bool auto_start;
    
    /* Interrupt handling */
    QEMUTimer *irq_timer;
    bool msi_enabled;
    
} CSI2LoopbackPCIe;

/* Forward MMI/O operations to the core loopback device */
static uint64_t csi2_loopback_pcie_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    CSI2LoopbackPCIe *pcie_dev = CSI2_LOOPBACK_PCIE(opaque);
    
    if (pcie_dev->loopback_dev) {
        return csi2_loopback_read(pcie_dev->loopback_dev, addr, size);
    }
    
    return 0;
}

static void csi2_loopback_pcie_mmio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    CSI2LoopbackPCIe *pcie_dev = CSI2_LOOPBACK_PCIE(opaque);
    
    if (pcie_dev->loopback_dev) {
        csi2_loopback_write(pcie_dev->loopback_dev, addr, value, size);
    }
}

static const MemoryRegionOps csi2_loopback_pcie_mmio_ops = {
    .read = csi2_loopback_pcie_mmio_read,
    .write = csi2_loopback_pcie_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* IRQ generation timer */
static void csi2_loopback_pcie_irq_timeout(void *opaque)
{
    CSI2LoopbackPCIe *pcie_dev = CSI2_LOOPBACK_PCIE(opaque);
    
    if (pcie_dev->loopback_dev && csi2_loopback_is_active(pcie_dev->loopback_dev)) {
        /* Generate interrupt based on frame activity */
        uint32_t frames_sent = csi2_loopback_get_frames_sent(pcie_dev->loopback_dev);
        
        if (frames_sent > 0 && (frames_sent % 30 == 1)) {
            if (pcie_dev->msi_enabled) {
                msi_notify(&pcie_dev->parent_obj, 0);
            } else {
                pci_set_irq(&pcie_dev->parent_obj, 1);
                /* Clear IRQ after a short time */
                timer_mod(pcie_dev->irq_timer, 
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000); /* 1ms */
            }
        }
    }
    
    /* Schedule next IRQ check */
    timer_mod(pcie_dev->irq_timer, 
             qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333); /* 30 FPS */
}

static void csi2_loopback_pcie_realize(PCIDevice *pci_dev, Error **errp)
{
    CSI2LoopbackPCIe *pcie_dev = CSI2_LOOPBACK_PCIE(pci_dev);
    Error *local_err = NULL;
    
    /* Configure PCI device */
    pci_config_set_interrupt_pin(pci_dev->config, 1);
    
    /* Try to enable MSI */
    if (msi_init(pci_dev, 0x50, 1, true, false, &local_err) == 0) {
        pcie_dev->msi_enabled = true;
        printf("CSI2-Loopback-PCIe: MSI enabled\n");
    } else {
        pcie_dev->msi_enabled = false;
        printf("CSI2-Loopback-PCIe: Using legacy interrupts\n");
        error_free(local_err);
        local_err = NULL;
    }
    
    /* Setup MMIO region */
    memory_region_init_io(&pcie_dev->mmio, OBJECT(pcie_dev), 
                          &csi2_loopback_pcie_mmio_ops, pcie_dev, 
                          "csi2-loopback-pcie", 0x10000);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_MEM_TYPE_64, &pcie_dev->mmio);
    
    /* Create core loopback device */
    pcie_dev->loopback_dev = CSI2_V4L2_LOOPBACK(qdev_new(TYPE_CSI2_V4L2_LOOPBACK));
    
    /* Configure loopback device properties */
    if (pcie_dev->loopback_device) {
        qdev_prop_set_string(DEVICE(pcie_dev->loopback_dev), 
                            "loopback-device", pcie_dev->loopback_device);
    }
    if (pcie_dev->device_name) {
        qdev_prop_set_string(DEVICE(pcie_dev->loopback_dev), 
                            "device-name", pcie_dev->device_name);
    }
    
    qdev_prop_set_uint32(DEVICE(pcie_dev->loopback_dev), "frame-width", pcie_dev->frame_width);
    qdev_prop_set_uint32(DEVICE(pcie_dev->loopback_dev), "frame-height", pcie_dev->frame_height);
    qdev_prop_set_uint32(DEVICE(pcie_dev->loopback_dev), "pixel-format", pcie_dev->pixel_format);
    qdev_prop_set_uint32(DEVICE(pcie_dev->loopback_dev), "fps", pcie_dev->fps);
    qdev_prop_set_uint32(DEVICE(pcie_dev->loopback_dev), "num-lanes", pcie_dev->num_lanes);
    qdev_prop_set_uint32(DEVICE(pcie_dev->loopback_dev), "line-rate", pcie_dev->line_rate);
    qdev_prop_set_bit(DEVICE(pcie_dev->loopback_dev), "auto-start", pcie_dev->auto_start);
    
    /* Realize the loopback device */
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(pcie_dev->loopback_dev), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    /* Create IRQ timer */
    pcie_dev->irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                      csi2_loopback_pcie_irq_timeout, pcie_dev);
    
    /* Start IRQ timer */
    timer_mod(pcie_dev->irq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000000ULL);
    
    printf("CSI2-Loopback-PCIe: Device realized\n");
    printf("  Loopback device: %s\n", pcie_dev->loopback_device ?: "/dev/video0");
    printf("  Resolution: %dx%d @ %dfps\n", 
           pcie_dev->frame_width, pcie_dev->frame_height, pcie_dev->fps);
    printf("  Pixel format: 0x%08x\n", pcie_dev->pixel_format);
    printf("  CSI2: %d lanes @ %d Mbps\n", pcie_dev->num_lanes, pcie_dev->line_rate);
    printf("  Auto-start: %s\n", pcie_dev->auto_start ? "enabled" : "disabled");
}

static void csi2_loopback_pcie_exit(PCIDevice *pci_dev)
{
    CSI2LoopbackPCIe *pcie_dev = CSI2_LOOPBACK_PCIE(pci_dev);
    
    /* Stop IRQ timer */
    if (pcie_dev->irq_timer) {
        timer_free(pcie_dev->irq_timer);
        pcie_dev->irq_timer = NULL;
    }
    
    /* Stop loopback device */
    if (pcie_dev->loopback_dev) {
        csi2_loopback_device_stop(pcie_dev->loopback_dev);
        object_unref(OBJECT(pcie_dev->loopback_dev));
        pcie_dev->loopback_dev = NULL;
    }
    
    /* Cleanup MSI */
    if (pcie_dev->msi_enabled) {
        msi_uninit(pci_dev);
    }
    
    printf("CSI2-Loopback-PCIe: Device removed\n");
}

static void csi2_loopback_pcie_reset(DeviceState *dev)
{
    CSI2LoopbackPCIe *pcie_dev = CSI2_LOOPBACK_PCIE(dev);
    
    /* Reset loopback device */
    if (pcie_dev->loopback_dev) {
        device_cold_reset(DEVICE(pcie_dev->loopback_dev));
    }
    
    /* Clear IRQ */
    if (!pcie_dev->msi_enabled) {
        pci_set_irq(&pcie_dev->parent_obj, 0);
    }
    
    printf("CSI2-Loopback-PCIe: Device reset\n");
}

static Property csi2_loopback_pcie_properties[] = {
    DEFINE_PROP_STRING("loopback-device", CSI2LoopbackPCIe, loopback_device),
    DEFINE_PROP_STRING("device-name", CSI2LoopbackPCIe, device_name),
    DEFINE_PROP_UINT32("frame-width", CSI2LoopbackPCIe, frame_width, 1280),
    DEFINE_PROP_UINT32("frame-height", CSI2LoopbackPCIe, frame_height, 720),
    DEFINE_PROP_UINT32("pixel-format", CSI2LoopbackPCIe, pixel_format, V4L2_PIX_FMT_RGB24),
    DEFINE_PROP_UINT32("fps", CSI2LoopbackPCIe, fps, 30),
    DEFINE_PROP_UINT32("num-lanes", CSI2LoopbackPCIe, num_lanes, 4),
    DEFINE_PROP_UINT32("line-rate", CSI2LoopbackPCIe, line_rate, 1000),
    DEFINE_PROP_BOOL("auto-start", CSI2LoopbackPCIe, auto_start, true),
    {}
};

static void csi2_loopback_pcie_instance_init(Object *obj)
{
    CSI2LoopbackPCIe *pcie_dev = CSI2_LOOPBACK_PCIE(obj);
    
    /* Set default values */
    pcie_dev->loopback_device = NULL;
    pcie_dev->device_name = NULL;
    pcie_dev->frame_width = 1280;
    pcie_dev->frame_height = 720;
    pcie_dev->pixel_format = V4L2_PIX_FMT_RGB24;
    pcie_dev->fps = 30;
    pcie_dev->num_lanes = 4;
    pcie_dev->line_rate = 1000;
    pcie_dev->auto_start = true;
    
    pcie_dev->loopback_dev = NULL;
    pcie_dev->irq_timer = NULL;
    pcie_dev->msi_enabled = false;
}

static void csi2_loopback_pcie_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);
    
    pc->realize = csi2_loopback_pcie_realize;
    pc->exit = csi2_loopback_pcie_exit;
    pc->vendor_id = CSI2_LOOPBACK_VENDOR_ID;
    pc->device_id = CSI2_LOOPBACK_DEVICE_ID;
    pc->class_id = 0x0480;  /* Multimedia controller - Other */
    pc->subsystem_vendor_id = CSI2_LOOPBACK_VENDOR_ID;
    pc->subsystem_id = CSI2_LOOPBACK_DEVICE_ID;
    pc->revision = 0x01;
    
    dc->desc = "MIPI CSI-2 V4L2 Loopback PCIe Device";
    device_class_set_legacy_reset(dc, csi2_loopback_pcie_reset);
    dc->props_ = csi2_loopback_pcie_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo csi2_loopback_pcie_info = {
    .name = TYPE_CSI2_LOOPBACK_PCIE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(CSI2LoopbackPCIe),
    .instance_init = csi2_loopback_pcie_instance_init,
    .class_init = csi2_loopback_pcie_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void csi2_loopback_pcie_register_types(void)
{
    type_register_static(&csi2_loopback_pcie_info);
}

type_init(csi2_loopback_pcie_register_types)
