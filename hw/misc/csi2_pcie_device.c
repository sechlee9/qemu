// hw/misc/csi2_pcie_device.c - CSI2 Sink Device with Virtual Source (Simplified)
#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/pci/msi.h"
#include "include/hw/misc/csi2_rx_controller.h"
#include "include/hw/misc/csi2_v4l2_bridge.h"

#define TYPE_CSI2_PCIE_DEVICE "mipi-csi-camera-x86"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2PCIeDevice, CSI2_PCIE_DEVICE)

#define CSI2_VENDOR_ID    0x1234
#define CSI2_DEVICE_ID    0x5678

// Virtual Source Types (내장된 가상 카메라 소스)
typedef enum {
    CSI2_VIRTUAL_SOURCE_NONE = 0,
    CSI2_VIRTUAL_SOURCE_TEST_PATTERN,
    CSI2_VIRTUAL_SOURCE_COLOR_BARS,
    CSI2_VIRTUAL_SOURCE_MOVING_PATTERN,
    CSI2_VIRTUAL_SOURCE_NOISE,
    CSI2_VIRTUAL_SOURCE_GRADIENT
} CSI2VirtualSourceType;

typedef struct CSI2PCIeDevice {
    PCIDevice parent_obj;
    MemoryRegion mmio;
    
    /* Sub-components */
    CSI2RxController *csi2_controller;
    CSI2V4L2Bridge *v4l2_bridge;
    
    /* Virtual Source (내장된 가상 카메라) */
    CSI2VirtualSourceType virtual_source_type;
    bool virtual_source_enabled;
    uint32_t virtual_source_pattern;
    QEMUTimer *frame_timer;
    
    /* Configuration */
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t frame_width;
    uint32_t frame_height;
    
    /* Runtime state */
    uint32_t sequence;
    bool streaming;
    
    /* Frame buffer for virtual source */
    uint8_t *frame_buffer;
    size_t frame_buffer_size;
    
} CSI2PCIeDevice;

// Virtual Source Frame Generation Functions
static void generate_test_pattern_x86(CSI2PCIeDevice *dev)
{
    if (!dev->frame_buffer) return;
    
    uint8_t *buf = dev->frame_buffer;
    uint32_t counter = dev->sequence % 256;
    
    for (uint32_t y = 0; y < dev->frame_height; y++) {
        for (uint32_t x = 0; x < dev->frame_width; x++) {
            uint32_t offset = (y * dev->frame_width + x) * 3;
            buf[offset + 0] = (x + counter) % 256;        // R
            buf[offset + 1] = (y + counter) % 256;        // G
            buf[offset + 2] = (x + y + counter) % 256;    // B
        }
    }
}

static void generate_color_bars_x86(CSI2PCIeDevice *dev)
{
    if (!dev->frame_buffer) return;
    
    uint8_t *buf = dev->frame_buffer;
    
    for (uint32_t y = 0; y < dev->frame_height; y++) {
        for (uint32_t x = 0; x < dev->frame_width; x++) {
            uint32_t offset = (y * dev->frame_width + x) * 3;
            uint32_t bar = (x * 8) / dev->frame_width;
            buf[offset + 0] = (bar & 1) ? 255 : 0;        // R
            buf[offset + 1] = (bar & 2) ? 255 : 0;        // G
            buf[offset + 2] = (bar & 4) ? 255 : 0;        // B
        }
    }
}

static void generate_moving_pattern_x86(CSI2PCIeDevice *dev)
{
    if (!dev->frame_buffer) return;
    
    uint8_t *buf = dev->frame_buffer;
    uint32_t counter = dev->sequence;
    
    for (uint32_t y = 0; y < dev->frame_height; y++) {
        for (uint32_t x = 0; x < dev->frame_width; x++) {
            uint32_t offset = (y * dev->frame_width + x) * 3;
            uint32_t wave = ((x + counter * 3) % 80) < 40 ? 255 : 0;
            buf[offset + 0] = wave;                       // R
            buf[offset + 1] = wave;                       // G
            buf[offset + 2] = wave;                       // B
        }
    }
}

static void generate_gradient_x86(CSI2PCIeDevice *dev)
{
    if (!dev->frame_buffer) return;
    
    uint8_t *buf = dev->frame_buffer;
    
    for (uint32_t y = 0; y < dev->frame_height; y++) {
        for (uint32_t x = 0; x < dev->frame_width; x++) {
            uint32_t offset = (y * dev->frame_width + x) * 3;
            buf[offset + 0] = (x * 255) / dev->frame_width;     // R
            buf[offset + 1] = (y * 255) / dev->frame_height;    // G
            buf[offset + 2] = 128;                              // B
        }
    }
}

static void generate_noise_x86(CSI2PCIeDevice *dev)
{
    if (!dev->frame_buffer) return;
    
    uint8_t *buf = dev->frame_buffer;
    static uint32_t seed = 54321;
    
    for (uint32_t y = 0; y < dev->frame_height; y++) {
        for (uint32_t x = 0; x < dev->frame_width; x++) {
            uint32_t offset = (y * dev->frame_width + x) * 3;
            // Simple PRNG
            seed = seed * 1664525 + 1013904223;
            buf[offset + 0] = (seed >> 16) & 0xFF;        // R
            buf[offset + 1] = (seed >> 8) & 0xFF;         // G
            buf[offset + 2] = seed & 0xFF;                // B
        }
    }
}

// Virtual Source Frame Generation
static void csi2_virtual_source_generate_frame_x86(CSI2PCIeDevice *dev)
{
    if (!dev->virtual_source_enabled || !dev->frame_buffer) {
        return;
    }
    
    switch (dev->virtual_source_type) {
    case CSI2_VIRTUAL_SOURCE_TEST_PATTERN:
        generate_test_pattern_x86(dev);
        break;
    case CSI2_VIRTUAL_SOURCE_COLOR_BARS:
        generate_color_bars_x86(dev);
        break;
    case CSI2_VIRTUAL_SOURCE_MOVING_PATTERN:
        generate_moving_pattern_x86(dev);
        break;
    case CSI2_VIRTUAL_SOURCE_GRADIENT:
        generate_gradient_x86(dev);
        break;
    case CSI2_VIRTUAL_SOURCE_NOISE:
        generate_noise_x86(dev);
        break;
    default:
        generate_test_pattern_x86(dev);
        break;
    }
}

// Frame Timer Callback (Virtual Source Frame Reception Simulation)
static void csi2_pcie_frame_timeout(void *opaque)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(opaque);
    
    if (!dev->streaming) {
        return;
    }
    
    dev->sequence++;
    
    // Simulate frame reception from virtual source
    csi2_virtual_source_generate_frame_x86(dev);
    
    // Update CSI2 controller (simulate frame reception)
    if (dev->csi2_controller && dev->virtual_source_enabled) {
        dev->csi2_controller->packet_count = dev->sequence;
        dev->csi2_controller->core_status = 
            (dev->sequence << 16) & 0xFFFF0000;
        dev->csi2_controller->int_status |= CSI2_INT_FRAME_RECEIVED;
    }
    
    // Update V4L2 bridge (simulate frame processing)
    if (dev->v4l2_bridge && dev->virtual_source_enabled) {
        dev->v4l2_bridge->frames_captured++;
        dev->v4l2_bridge->sequence_number = dev->sequence;
    }
    
    // Debug output every 30 frames
    if (dev->sequence % 30 == 1) {
        printf("CSI2-Sink: Received frame %d from virtual source (type: %s)\n", 
               dev->sequence,
               dev->virtual_source_type == CSI2_VIRTUAL_SOURCE_TEST_PATTERN ? "test pattern" :
               dev->virtual_source_type == CSI2_VIRTUAL_SOURCE_COLOR_BARS ? "color bars" :
               dev->virtual_source_type == CSI2_VIRTUAL_SOURCE_MOVING_PATTERN ? "moving pattern" :
               dev->virtual_source_type == CSI2_VIRTUAL_SOURCE_GRADIENT ? "gradient" :
               dev->virtual_source_type == CSI2_VIRTUAL_SOURCE_NOISE ? "noise" : "unknown");
    }
    
    // Continue if streaming is active
    if (dev->streaming) {
        timer_mod(dev->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
    }
}

// MMIO Interface
static uint64_t csi2_pcie_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(opaque);
    
    if (addr < 0x1000 && dev->csi2_controller) {
        return csi2_rx_controller_read(dev->csi2_controller, addr, size);
    } else if (addr >= 0x2000 && addr < 0x3000 && dev->v4l2_bridge) {
        return csi2_v4l2_bridge_read(dev->v4l2_bridge, addr - 0x2000, size);
    } else if (addr >= 0x3000 && addr < 0x4000) {
        /* Extended control registers */
        switch (addr - 0x3000) {
        case 0x00: /* Streaming status */
            return dev->streaming ? 1 : 0;
        case 0x04: /* Sequence counter */
            return dev->sequence;
        case 0x08: /* Frame config */
            return (dev->frame_width << 16) | dev->frame_height;
        case 0x0C: /* Device config */
            return (dev->num_lanes << 16) | dev->line_rate;
        case 0x10: /* Virtual source type */
            return dev->virtual_source_type;
        case 0x14: /* Virtual source enabled */
            return dev->virtual_source_enabled ? 1 : 0;
        case 0x18: /* Virtual source pattern */
            return dev->virtual_source_pattern;
        default:
            break;
        }
    }
    return 0;
}

static void csi2_pcie_mmio_write(void *opaque, hwaddr addr, 
                                 uint64_t value, unsigned size)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(opaque);
    
    if (addr < 0x1000 && dev->csi2_controller) {
        csi2_rx_controller_write(dev->csi2_controller, addr, value, size);
    } else if (addr >= 0x2000 && addr < 0x3000 && dev->v4l2_bridge) {
        csi2_v4l2_bridge_write(dev->v4l2_bridge, addr - 0x2000, value, size);
    } else if (addr >= 0x3000 && addr < 0x4000) {
        /* Extended control registers */
        switch (addr - 0x3000) {
        case 0x00: /* Streaming control */
            if (value & 1) {
                dev->streaming = true;
                dev->sequence = 0;
                timer_mod(dev->frame_timer, 
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
                printf("CSI2-Sink: Streaming started (virtual source: %s)\n",
                       dev->virtual_source_enabled ? "enabled" : "disabled");
            } else {
                dev->streaming = false;
                timer_del(dev->frame_timer);
                printf("CSI2-Sink: Streaming stopped\n");
            }
            break;
        case 0x04: /* Sequence reset */
            if (value == 0) {
                dev->sequence = 0;
                printf("CSI2-Sink: Sequence reset\n");
            }
            break;
        case 0x08: /* Statistics reset */
            if (value == 0xDEADBEEF) {
                dev->sequence = 0;
                if (dev->csi2_controller) {
                    dev->csi2_controller->packet_count = 0;
                }
                if (dev->v4l2_bridge) {
                    dev->v4l2_bridge->frames_captured = 0;
                    dev->v4l2_bridge->sequence_number = 0;
                }
                printf("CSI2-Sink: Statistics reset\n");
            }
            break;
        case 0x10: /* Virtual source type */
            if (value <= CSI2_VIRTUAL_SOURCE_GRADIENT) {
                dev->virtual_source_type = value;
                printf("CSI2-Sink: Virtual source type set to %ld (%s)\n", value,
                       value == CSI2_VIRTUAL_SOURCE_NONE ? "none" :
                       value == CSI2_VIRTUAL_SOURCE_TEST_PATTERN ? "test pattern" :
                       value == CSI2_VIRTUAL_SOURCE_COLOR_BARS ? "color bars" :
                       value == CSI2_VIRTUAL_SOURCE_MOVING_PATTERN ? "moving pattern" :
                       value == CSI2_VIRTUAL_SOURCE_GRADIENT ? "gradient" :
                       value == CSI2_VIRTUAL_SOURCE_NOISE ? "noise" : "unknown");
            }
            break;
        case 0x14: /* Virtual source enable */
            dev->virtual_source_enabled = (value & 1) != 0;
            printf("CSI2-Sink: Virtual source %s\n",
                   dev->virtual_source_enabled ? "enabled" : "disabled");
            break;
        case 0x18: /* Virtual source pattern */
            dev->virtual_source_pattern = value;
            printf("CSI2-Sink: Virtual source pattern set to %ld\n", value);
            break;
        default:
            break;
        }
    }
}

static const MemoryRegionOps csi2_pcie_mmio_ops = {
    .read = csi2_pcie_mmio_read,
    .write = csi2_pcie_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

// Device lifecycle
static void csi2_pcie_realize(PCIDevice *pci_dev, Error **errp)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(pci_dev);
    Error *local_err = NULL;
    
    printf("CSI2-Sink: Device initialization started (mipi-csi-camera-x86)\n");
    
    /* Initialize default values first */
    if (dev->frame_width == 0) dev->frame_width = 1280;
    if (dev->frame_height == 0) dev->frame_height = 720;
    if (dev->num_lanes == 0) dev->num_lanes = 4;
    if (dev->line_rate == 0) dev->line_rate = 1440;
    
    printf("CSI2-Sink: Frame configuration: %dx%d, %d lanes @ %d Mbps\n",
           dev->frame_width, dev->frame_height, dev->num_lanes, dev->line_rate);
    
    /* PCI configuration */
    pci_config_set_interrupt_pin(pci_dev->config, 1);
    memory_region_init_io(&dev->mmio, OBJECT(dev), &csi2_pcie_mmio_ops, 
                          dev, "csi2-sink-x86", 0x10000);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_MEM_TYPE_64, &dev->mmio);
    
    /* Create sub-components */
    dev->csi2_controller = CSI2_RX_CONTROLLER(qdev_new(TYPE_CSI2_RX_CONTROLLER));
    dev->v4l2_bridge = CSI2_V4L2_BRIDGE(qdev_new(TYPE_CSI2_V4L2_BRIDGE));
    
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(dev->csi2_controller), &local_err) ||
        !sysbus_realize_and_unref(SYS_BUS_DEVICE(dev->v4l2_bridge), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    /* Create frame timer for virtual source */
    dev->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                    csi2_pcie_frame_timeout, dev);
    
    /* Initialize frame buffer for virtual source (after frame dimensions are set) */
    dev->frame_buffer_size = dev->frame_width * dev->frame_height * 3; // RGB24
    dev->frame_buffer = g_malloc(dev->frame_buffer_size);
    if (!dev->frame_buffer) {
        error_setg(errp, "Failed to allocate frame buffer (%zu bytes)", dev->frame_buffer_size);
        return;
    }
    printf("CSI2-Sink: Frame buffer allocated: %zu bytes\n", dev->frame_buffer_size);
    
    /* Initialize virtual source */
    dev->virtual_source_type = CSI2_VIRTUAL_SOURCE_TEST_PATTERN;
    dev->virtual_source_enabled = true;  // Enabled by default
    dev->virtual_source_pattern = 0;
    
    /* Initialize state */
    dev->streaming = false;
    dev->sequence = 0;
    
    printf("CSI2-Sink: Device initialized (%dx%d, %d lanes, %d Mbps)\n",
           dev->frame_width, dev->frame_height, dev->num_lanes, dev->line_rate);
    printf("CSI2-Sink: Type: CSI2 Sink Device with Virtual Source\n");
    printf("CSI2-Sink: Virtual source: %s (type: %s)\n", 
           dev->virtual_source_enabled ? "enabled" : "disabled",
           dev->virtual_source_type == CSI2_VIRTUAL_SOURCE_TEST_PATTERN ? "test pattern" : "other");
    printf("CSI2-Sink: Frame buffer: %zu bytes allocated\n", dev->frame_buffer_size);
    printf("CSI2-Sink: Extended Register Map:\n");
    printf("  0x3000: Streaming control (R/W)\n");
    printf("  0x3004: Sequence counter (R/W)\n");
    printf("  0x3008: Frame config (R)\n");
    printf("  0x3010: Virtual source type (R/W)\n");
    printf("  0x3014: Virtual source enabled (R/W)\n");
    printf("  0x3018: Virtual source pattern (R/W)\n");
}

static void csi2_pcie_exit(PCIDevice *pci_dev)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(pci_dev);
    
    printf("CSI2-Sink: Device cleanup started\n");
    
    if (dev->frame_timer) {
        timer_del(dev->frame_timer);
        timer_free(dev->frame_timer);
    }
    
    g_free(dev->frame_buffer);
}

static void csi2_pcie_instance_init(Object *obj)
{
    CSI2PCIeDevice *dev = CSI2_PCIE_DEVICE(obj);
    
    /* Set default values */
    dev->num_lanes = 4;
    dev->line_rate = 1440;
    dev->frame_width = 1280;
    dev->frame_height = 720;
    dev->virtual_source_type = CSI2_VIRTUAL_SOURCE_TEST_PATTERN;
    dev->virtual_source_enabled = true;
    dev->virtual_source_pattern = 0;
    
    /* Initialize runtime state */
    dev->streaming = false;
    dev->sequence = 0;
    dev->frame_buffer = NULL;
    dev->frame_buffer_size = 0;
    dev->frame_timer = NULL;
    
    printf("CSI2-Sink: Instance initialized with defaults (%dx%d)\n", 
           dev->frame_width, dev->frame_height);
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
    if (dev->v4l2_bridge) {
        device_cold_reset(DEVICE(dev->v4l2_bridge));
    }
    
    dev->sequence = 0;
    dev->streaming = false;
    
    // Reset virtual source to default
    dev->virtual_source_type = CSI2_VIRTUAL_SOURCE_TEST_PATTERN;
    dev->virtual_source_enabled = true;
    dev->virtual_source_pattern = 0;
    
    printf("CSI2-Sink: Device reset completed\n");
}

static Property csi2_pcie_properties[] = {
    DEFINE_PROP_UINT32("num-lanes", CSI2PCIeDevice, num_lanes, 4),
    DEFINE_PROP_UINT32("line-rate", CSI2PCIeDevice, line_rate, 1440), 
    DEFINE_PROP_UINT32("frame-width", CSI2PCIeDevice, frame_width, 1280),
    DEFINE_PROP_UINT32("frame-height", CSI2PCIeDevice, frame_height, 720),
    DEFINE_PROP_UINT32("virtual-source-type", CSI2PCIeDevice, virtual_source_type, CSI2_VIRTUAL_SOURCE_TEST_PATTERN),
    DEFINE_PROP_BOOL("virtual-source-enabled", CSI2PCIeDevice, virtual_source_enabled, true),
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
    pc->class_id = 0x0400;
    pc->revision = 0x03;
    
    dc->desc = "MIPI CSI-2 Sink Device with Virtual Source (X86)";
    device_class_set_legacy_reset(dc, csi2_pcie_reset);
    dc->props_ = csi2_pcie_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo csi2_pcie_info = {
    .name = TYPE_CSI2_PCIE_DEVICE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(CSI2PCIeDevice),
    .instance_init = csi2_pcie_instance_init,
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
