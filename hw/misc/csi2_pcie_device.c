#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/pci/msi.h"
#include "hw/misc/csi2_rx_controller.h"
#include "hw/misc/csi2_v4l2_bridge.h"

#define TYPE_CSI2_PCIE_DEVICE "mipi-csi-camera-x86"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2PCIeDevice, CSI2_PCIE_DEVICE)

#define CSI2_VENDOR_ID    0x1234
#define CSI2_DEVICE_ID    0x5678

typedef struct CSI2PCIeDevice {
    PCIDevice parent_obj;
    MemoryRegion mmio;
    CSI2RxController *csi2_controller;
    CSI2V4L2Bridge *v4l2_bridge; 
    QEMUTimer *frame_timer;
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t frame_width;
    uint32_t frame_height;
} CSI2PCIeDevice;

static void csi2_pcie_frame_timer_cb(void *opaque)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(opaque);
    if (dev->csi2_controller) {
        dev->csi2_controller->int_status |= CSI2_INT_FRAME_RECEIVED;
        if (dev->csi2_controller->global_int_enable && 
            (dev->csi2_controller->int_status & dev->csi2_controller->int_enable)) {
            qemu_set_irq(dev->csi2_controller->irq, 1);
        }
    }
    timer_mod(dev->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
}

static uint64_t csi2_pcie_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(opaque);
    if (dev->csi2_controller && addr < 0x1000) {
        return csi2_rx_controller_read(dev->csi2_controller, addr, size);
    } else if (addr >= 0x2000 && addr < 0x3000 && dev->v4l2_bridge) {
        return csi2_v4l2_bridge_read(dev->v4l2_bridge, addr - 0x2000, size);
    }
    return 0;
}

static void csi2_pcie_mmio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(opaque);
    if (dev->csi2_controller && addr < 0x1000) {
        csi2_rx_controller_write(dev->csi2_controller, addr, value, size);
    } else if (addr >= 0x2000 && addr < 0x3000 && dev->v4l2_bridge) {
        csi2_v4l2_bridge_write(dev->v4l2_bridge, addr - 0x2000, value, size);
    }

    if (addr == 0x3000 && (value & 1)) {
        timer_mod(dev->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
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
    
    pci_config_set_interrupt_pin(pci_dev->config, 1);
    
    memory_region_init_io(&dev->mmio, OBJECT(dev), &csi2_pcie_mmio_ops, dev, "csi2-pcie", 0x10000);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_MEM_TYPE_64, &dev->mmio);
    
    dev->csi2_controller = CSI2_RX_CONTROLLER(qdev_new(TYPE_CSI2_RX_CONTROLLER));
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(dev->csi2_controller), errp)) {
        return;
    }

    dev->v4l2_bridge = CSI2_V4L2_BRIDGE(qdev_new(TYPE_CSI2_V4L2_BRIDGE));
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(dev->v4l2_bridge), errp)) {
        return;
    }

    dev->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, csi2_pcie_frame_timer_cb, dev);
    
    printf("CSI2 Device initialized: lanes=%d, rate=%d, size=%dx%d\n", 
           dev->num_lanes, dev->line_rate, dev->frame_width, dev->frame_height);
}

static void csi2_pcie_exit(PCIDevice *pci_dev)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(pci_dev);
    if (dev->frame_timer) {
        timer_free(dev->frame_timer);
        dev->frame_timer = NULL;
    }
}

static void csi2_pcie_reset(DeviceState *qdev)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(qdev);
    if (dev->frame_timer) {
        timer_del(dev->frame_timer);
    }
    if (dev->csi2_controller) {
        device_cold_reset(DEVICE(dev->csi2_controller));
    }
}

/* Simple property array - just use empty terminator */
static Property csi2_pcie_properties[] = {
    DEFINE_PROP_UINT32("num-lanes", CSI2PCIeDevice, num_lanes, 4),
    DEFINE_PROP_UINT32("line-rate", CSI2PCIeDevice, line_rate, 1000), 
    DEFINE_PROP_UINT32("frame-width", CSI2PCIeDevice, frame_width, 1280),
    DEFINE_PROP_UINT32("frame-height", CSI2PCIeDevice, frame_height, 720),
    {} /* Empty terminator instead of DEFINE_PROP_END_OF_LIST() */
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
    dc->props_ = csi2_pcie_properties; /* Use dc->props_ instead of device_class_set_props */
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

static void csi2_pcie_register_types(void)
{
    type_register_static(&csi2_pcie_info);
}

type_init(csi2_pcie_register_types)
