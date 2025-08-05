/*
 * AMD CSI-2 RX PCIe Device for QEMU with Ring Buffer
 * Version: 2.0 - Complete rewrite with ring buffer architecture
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include "hw/pci/pci.h"
#include "hw/pci/msix.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/qdev-properties.h"
#include "hw/resettable.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

#define TYPE_AMD_CSI2_RX_PCIE "csi2-pcie-sink"
#define AMD_CSI2_RX_PCIE(obj) OBJECT_CHECK(AmdCsi2RxPcieState, (obj), TYPE_AMD_CSI2_RX_PCIE)

#define XILINX_VENDOR_ID    0x10ee
#define CSI2_DEVICE_ID      0x9024

#define CSI2_MMIO_SIZE      0x2000
#define CSI2_MSIX_VECTORS   4
#define CSI2_RING_SIZE      16

/* Debug macro */
#define CSI2_DEBUG 1

#ifdef CSI2_DEBUG
#define CSI2_DPRINTF(fmt, ...) \
    do { \
        fprintf(stderr, "CSI2: " fmt "\n", ## __VA_ARGS__); \
    } while (0)
#else
#define CSI2_DPRINTF(fmt, ...) do {} while (0)
#endif

/* Register Offsets */
#define CSI2_CORE_CONFIG_REG            0x00
#define CSI2_CORE_STATUS_REG            0x10
#define CSI2_INT_STATUS_REG             0x20
#define CSI2_INT_ENABLE_REG             0x24
#define CSI2_GLOBAL_INT_EN_REG          0x28

/* Ring Buffer Registers */
#define CSI2_RING_BASE_LOW              0x100
#define CSI2_RING_BASE_HIGH             0x104
#define CSI2_RING_SIZE_REG              0x108
#define CSI2_RING_HEAD_REG              0x10C  /* Driver writes, QEMU reads */
#define CSI2_RING_TAIL_REG              0x110  /* QEMU writes, Driver reads */
#define CSI2_RING_CTRL_REG              0x114
#define CSI2_RING_STATUS_REG            0x118

/* Format Registers */
#define CSI2_FORMAT_REG                 0x200
#define CSI2_WIDTH_REG                  0x204
#define CSI2_HEIGHT_REG                 0x208
#define CSI2_FPS_REG                    0x20C

/* Debug Register */
#define CSI2_DEBUG_REG                  0x300
#define CSI2_ERROR_COUNT_REG            0x304
#define CSI2_FRAME_COUNT_REG            0x308

/* Control bits */
#define CSI2_CORE_CONFIG_ENABLE         (1U << 0)
#define CSI2_CORE_CONFIG_RESET          (1U << 1)

/* Ring control bits */
#define RING_CTRL_ENABLE                (1U << 0)
#define RING_CTRL_RESET                 (1U << 1)

/* Ring status bits */
#define RING_STATUS_READY               (1U << 0)
#define RING_STATUS_FULL                (1U << 1)
#define RING_STATUS_EMPTY               (1U << 2)
#define RING_STATUS_ERROR               (1U << 3)

/* Interrupt bits */
#define CSI2_INT_FRAME_DONE             (1U << 0)
#define CSI2_INT_RING_FULL              (1U << 1)
#define CSI2_INT_ERROR                  (1U << 2)

/* Ring buffer entry structure */
typedef struct RingEntry {
    uint64_t dma_addr;
    uint32_t size;
    uint32_t flags;
    uint64_t timestamp;
    uint32_t reserved[2];
} __attribute__((packed)) RingEntry;

/* Ring buffer structure in guest memory */
typedef struct RingBuffer {
    uint32_t magic;      /* 0x43534932 - "CSI2" */
    uint32_t version;    /* 0x00020000 - version 2.0 */
    uint32_t head;       /* Written by driver */
    uint32_t tail;       /* Written by QEMU */
    uint32_t size;       /* Number of entries */
    uint32_t entry_size; /* Size of each entry */
    uint32_t reserved[2];
    RingEntry entries[CSI2_RING_SIZE];
} __attribute__((packed)) RingBuffer;

typedef struct AmdCsi2RxPcieState {
    PCIDevice parent_obj;
    MemoryRegion mmio;
    
    /* Core state */
    bool enabled;
    bool streaming;
    
    /* Ring buffer */
    uint64_t ring_base_addr;
    uint32_t ring_size;
    uint32_t ring_head;  /* Local copy */
    uint32_t ring_tail;  /* Local copy */
    bool ring_enabled;
    
    /* Format */
    uint32_t width;
    uint32_t height;
    uint32_t format;
    uint32_t fps;
    uint32_t frame_size;
    
    /* Timers */
    QEMUTimer *frame_timer;
    
    /* Statistics */
    uint32_t frame_count;
    uint32_t error_count;
    uint32_t dropped_frames;
    
    /* Registers */
    uint32_t int_status;
    uint32_t int_enable;
    uint32_t global_int_en;
} AmdCsi2RxPcieState;

/* Helper functions */
static void csi2_update_irq(AmdCsi2RxPcieState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    bool raise_irq = (s->int_status & s->int_enable) && s->global_int_en;
    
    if (raise_irq) {
        if (msix_enabled(pci_dev)) {
            msix_notify(pci_dev, 0);
        } else if (msi_enabled(pci_dev)) {
            msi_notify(pci_dev, 0);
        } else {
            pci_set_irq(pci_dev, 1);
        }
    } else {
        if (!msix_enabled(pci_dev) && !msi_enabled(pci_dev)) {
            pci_set_irq(pci_dev, 0);
        }
    }
}

static void csi2_set_interrupt(AmdCsi2RxPcieState *s, uint32_t mask)
{
    s->int_status |= mask;
    csi2_update_irq(s);
}

static void csi2_clear_interrupt(AmdCsi2RxPcieState *s, uint32_t mask)
{
    s->int_status &= ~mask;
    csi2_update_irq(s);
}

static int csi2_ring_is_full(uint32_t head, uint32_t tail, uint32_t size)
{
    return ((head + 1) % size) == tail;
}

static int csi2_ring_is_empty(uint32_t head, uint32_t tail)
{
    return head == tail;
}
#if 0
static int csi2_ring_count(uint32_t head, uint32_t tail, uint32_t size)
{
    if (head >= tail) {
        return head - tail;
    } else {
        return size - tail + head;
    }
}
#endif
static void csi2_update_ring_status(AmdCsi2RxPcieState *s)
{
    uint32_t status = 0;
    
    if (s->ring_enabled) {
        status |= RING_STATUS_READY;
    }
    
    if (csi2_ring_is_full(s->ring_head, s->ring_tail, s->ring_size)) {
        status |= RING_STATUS_FULL;
    }
    
    if (csi2_ring_is_empty(s->ring_head, s->ring_tail)) {
        status |= RING_STATUS_EMPTY;
    }
    
    /* Write status - no need to store in regs array */
}

static int csi2_read_ring_head(AmdCsi2RxPcieState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint32_t head;
    
    if (!s->ring_base_addr) {
        return -1;
    }
    
    /* Read head from ring buffer in guest memory */
    if (pci_dma_read(pci_dev, s->ring_base_addr + offsetof(RingBuffer, head),
                     &head, sizeof(head)) != 0) {
        CSI2_DPRINTF("Failed to read ring head");
        return -1;
    }
    
    head = le32_to_cpu(head);
    if (head >= s->ring_size) {
        CSI2_DPRINTF("Invalid ring head: %u (size=%u)", head, s->ring_size);
        return -1;
    }
    
    s->ring_head = head;
    return 0;
}

static int csi2_write_ring_tail(AmdCsi2RxPcieState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint32_t tail_le;
    
    if (!s->ring_base_addr) {
        return -1;
    }
    
    tail_le = cpu_to_le32(s->ring_tail);
    
    /* Write tail to ring buffer in guest memory */
    if (pci_dma_write(pci_dev, s->ring_base_addr + offsetof(RingBuffer, tail),
                      &tail_le, sizeof(tail_le)) != 0) {
        CSI2_DPRINTF("Failed to write ring tail");
        return -1;
    }
    
    return 0;
}

static void generate_test_frame(void *buffer, uint32_t size, uint32_t frame_num)
{
    uint32_t *data = (uint32_t *)buffer;
    uint32_t pixels = size / 4;
    uint32_t pattern = 0x80FF80FF;  /* Simple test pattern */
    
    /* First word is frame number */
    data[0] = cpu_to_le32(frame_num);
    
    /* Fill with test pattern */
    for (uint32_t i = 1; i < pixels; i++) {
        data[i] = cpu_to_le32(pattern);
        pattern = (pattern << 1) | (pattern >> 31);  /* Rotate */
    }
}

static void csi2_process_frame(AmdCsi2RxPcieState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    RingEntry entry;
    void *buffer;
    dma_addr_t len;
    
    /* Check if we have a buffer available */
    if (csi2_ring_is_empty(s->ring_head, s->ring_tail)) {
        s->dropped_frames++;
        CSI2_DPRINTF("No buffer available, frame dropped");
        return;
    }
    
    /* Read entry from ring */
    uint64_t entry_addr = s->ring_base_addr + 
                          offsetof(RingBuffer, entries) + 
                          (s->ring_tail * sizeof(RingEntry));
    
    if (pci_dma_read(pci_dev, entry_addr, &entry, sizeof(entry)) != 0) {
        CSI2_DPRINTF("Failed to read ring entry");
        s->error_count++;
        return;
    }
    
    /* Convert from little endian */
    entry.dma_addr = le64_to_cpu(entry.dma_addr);
    entry.size = le32_to_cpu(entry.size);
    
    CSI2_DPRINTF("Processing frame %u: buffer at 0x%lx, size=%u", 
                 s->frame_count, entry.dma_addr, entry.size);
    
    /* Validate entry */
    if (entry.size != s->frame_size) {
        CSI2_DPRINTF("Invalid buffer size: %u (expected %u)", 
                     entry.size, s->frame_size);
        s->error_count++;
        return;
    }
    
    /* Map buffer for DMA */
    len = entry.size;
    buffer = pci_dma_map(pci_dev, entry.dma_addr, &len, DMA_DIRECTION_FROM_DEVICE);
    
    if (!buffer || len != entry.size) {
        CSI2_DPRINTF("Failed to map buffer");
        s->error_count++;
        if (buffer) {
            pci_dma_unmap(pci_dev, buffer, len, DMA_DIRECTION_FROM_DEVICE, 0);
        }
        return;
    }
    
    /* Generate frame data */
    generate_test_frame(buffer, entry.size, s->frame_count);
    
    /* Unmap buffer */
    pci_dma_unmap(pci_dev, buffer, len, DMA_DIRECTION_FROM_DEVICE, entry.size);
    
    /* Update entry with completion info */
    entry.flags = cpu_to_le32(1);  /* Mark as complete */
    entry.timestamp = cpu_to_le64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    
    /* Write back entry */
    if (pci_dma_write(pci_dev, entry_addr, &entry, sizeof(entry)) != 0) {
        CSI2_DPRINTF("Failed to write back ring entry");
        s->error_count++;
        return;
    }
    
    /* Update tail */
    s->ring_tail = (s->ring_tail + 1) % s->ring_size;
    if (csi2_write_ring_tail(s) < 0) {
        s->error_count++;
        return;
    }
    
    /* Trigger interrupt */
    csi2_set_interrupt(s, CSI2_INT_FRAME_DONE);
    
    s->frame_count++;
    CSI2_DPRINTF("Frame %u completed successfully", s->frame_count - 1);
}

static void csi2_frame_timer(void *opaque)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(opaque);
    
    if (!s->streaming || !s->ring_enabled) {
        return;
    }
    
    /* Read current ring head */
    if (csi2_read_ring_head(s) < 0) {
        s->error_count++;
        return;
    }
    
    /* Process frame */
    csi2_process_frame(s);
    
    /* Update ring status */
    csi2_update_ring_status(s);
    
    /* Schedule next frame */
    if (s->streaming) {
        int64_t interval = 1000000000LL / s->fps;  /* nanoseconds */
        timer_mod_ns(s->frame_timer, 
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + interval);
    }
}

static void csi2_start_streaming(AmdCsi2RxPcieState *s)
{
    if (s->streaming) {
        return;
    }
    
    CSI2_DPRINTF("Starting streaming: %ux%u @ %u fps", 
                 s->width, s->height, s->fps);
    
    s->streaming = true;
    s->frame_count = 0;
    s->dropped_frames = 0;
    
    /* Start frame timer */
    int64_t interval = 1000000000LL / s->fps;
    timer_mod_ns(s->frame_timer, 
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + interval);
}

static void csi2_stop_streaming(AmdCsi2RxPcieState *s)
{
    if (!s->streaming) {
        return;
    }
    
    CSI2_DPRINTF("Stopping streaming");
    
    s->streaming = false;
    timer_del(s->frame_timer);
}

static uint64_t csi2_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(opaque);
    uint64_t val = 0;
    
    switch (addr) {
    case CSI2_CORE_CONFIG_REG:
        val = s->enabled ? CSI2_CORE_CONFIG_ENABLE : 0;
        break;
        
    case CSI2_CORE_STATUS_REG:
        val = (s->streaming << 0) | (s->ring_enabled << 1);
        break;
        
    case CSI2_INT_STATUS_REG:
        val = s->int_status;
        break;
        
    case CSI2_INT_ENABLE_REG:
        val = s->int_enable;
        break;
        
    case CSI2_GLOBAL_INT_EN_REG:
        val = s->global_int_en;
        break;
        
    case CSI2_RING_BASE_LOW:
        val = s->ring_base_addr & 0xFFFFFFFF;
        break;
        
    case CSI2_RING_BASE_HIGH:
        val = s->ring_base_addr >> 32;
        break;
        
    case CSI2_RING_SIZE_REG:
        val = s->ring_size;
        break;
        
    case CSI2_RING_HEAD_REG:
        val = s->ring_head;
        break;
        
    case CSI2_RING_TAIL_REG:
        val = s->ring_tail;
        break;
        
    case CSI2_RING_STATUS_REG:
        val = 0;
        if (s->ring_enabled) val |= RING_STATUS_READY;
        if (csi2_ring_is_full(s->ring_head, s->ring_tail, s->ring_size)) {
            val |= RING_STATUS_FULL;
        }
        if (csi2_ring_is_empty(s->ring_head, s->ring_tail)) {
            val |= RING_STATUS_EMPTY;
        }
        break;
        
    case CSI2_FRAME_COUNT_REG:
        val = s->frame_count;
        break;
        
    case CSI2_ERROR_COUNT_REG:
        val = s->error_count;
        break;
        
    case CSI2_WIDTH_REG:
        val = s->width;
        break;
        
    case CSI2_HEIGHT_REG:
        val = s->height;
        break;
        
    case CSI2_FPS_REG:
        val = s->fps;
        break;
        
    default:
        CSI2_DPRINTF("Read from unknown register 0x%lx", addr);
        break;
    }
    
    return val;
}

static void csi2_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(opaque);
    
    CSI2_DPRINTF("MMIO Write: addr=0x%04lx, val=0x%08lx", addr, val);
    
    switch (addr) {
    case CSI2_CORE_CONFIG_REG:
        if (val & CSI2_CORE_CONFIG_RESET) {
            /* Reset core */
            csi2_stop_streaming(s);
            s->enabled = false;
            s->int_status = 0;
            s->error_count = 0;
            s->frame_count = 0;
        } else if (val & CSI2_CORE_CONFIG_ENABLE) {
            s->enabled = true;
            if (s->ring_enabled) {
                csi2_start_streaming(s);
            }
        } else {
            s->enabled = false;
            csi2_stop_streaming(s);
        }
        break;
        
    case CSI2_INT_STATUS_REG:
        /* Write 1 to clear */
        csi2_clear_interrupt(s, val);
        break;
        
    case CSI2_INT_ENABLE_REG:
        s->int_enable = val;
        csi2_update_irq(s);
        break;
        
    case CSI2_GLOBAL_INT_EN_REG:
        s->global_int_en = val & 1;
        csi2_update_irq(s);
        break;
        
    case CSI2_RING_BASE_LOW:
        s->ring_base_addr = (s->ring_base_addr & 0xFFFFFFFF00000000ULL) | val;
        break;
        
    case CSI2_RING_BASE_HIGH:
        s->ring_base_addr = (s->ring_base_addr & 0xFFFFFFFFULL) | (val << 32);
        break;
        
    case CSI2_RING_SIZE_REG:
        if (val > 0 && val <= CSI2_RING_SIZE) {
            s->ring_size = val;
        }
        break;
        
    case CSI2_RING_CTRL_REG:
        if (val & RING_CTRL_RESET) {
            s->ring_enabled = false;
            s->ring_head = 0;
            s->ring_tail = 0;
            csi2_stop_streaming(s);
        } else if (val & RING_CTRL_ENABLE) {
            if (s->ring_base_addr && s->ring_size > 0) {
                s->ring_enabled = true;
                s->ring_head = 0;
                s->ring_tail = 0;
                CSI2_DPRINTF("Ring buffer enabled: base=0x%lx, size=%u",
                            s->ring_base_addr, s->ring_size);
                if (s->enabled) {
                    csi2_start_streaming(s);
                }
            }
        }
        break;
        
    case CSI2_WIDTH_REG:
        s->width = val;
        s->frame_size = s->width * s->height * 2;  /* Assuming YUV422 */
        break;
        
    case CSI2_HEIGHT_REG:
        s->height = val;
        s->frame_size = s->width * s->height * 2;
        break;
        
    case CSI2_FPS_REG:
        if (val > 0 && val <= 120) {
            s->fps = val;
        }
        break;
        
    default:
        CSI2_DPRINTF("Write to unknown register 0x%lx", addr);
        break;
    }
}

static const MemoryRegionOps csi2_mmio_ops = {
    .read = csi2_mmio_read,
    .write = csi2_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void amd_csi2_rx_pcie_reset(Object *obj, ResetType type)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(obj);
    
    CSI2_DPRINTF("Device reset");
    
    /* Stop streaming */
    csi2_stop_streaming(s);
    
    /* Reset state */
    s->enabled = false;
    s->ring_enabled = false;
    s->ring_base_addr = 0;
    s->ring_size = 0;
    s->ring_head = 0;
    s->ring_tail = 0;
    s->int_status = 0;
    s->int_enable = 0;
    s->global_int_en = 0;
    s->frame_count = 0;
    s->error_count = 0;
    s->dropped_frames = 0;
}

static void amd_csi2_rx_pcie_realize(PCIDevice *pci_dev, Error **errp)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(pci_dev);
    int ret;
    
    CSI2_DPRINTF("Device realize");
    
    /* Initialize state */
    s->enabled = false;
    s->streaming = false;
    s->ring_enabled = false;
    s->width = 1920;
    s->height = 1080;
    s->fps = 30;
    s->frame_size = s->width * s->height * 2;
    
    /* Set up PCI configuration */
    pci_config_set_vendor_id(pci_dev->config, XILINX_VENDOR_ID);
    pci_config_set_device_id(pci_dev->config, CSI2_DEVICE_ID);
    pci_config_set_class(pci_dev->config, PCI_CLASS_MULTIMEDIA_OTHER);
    pci_config_set_interrupt_pin(pci_dev->config, 1);
    
    /* Register MMIO BAR */
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_mmio_ops, s, 
                          "csi2-mmio", CSI2_MMIO_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
    
    /* Initialize MSI-X */
    ret = msix_init_exclusive_bar(pci_dev, CSI2_MSIX_VECTORS, 2, errp);
    if (ret < 0) {
        error_setg(errp, "Failed to initialize MSI-X: %d", ret);
        return;
    }
    
    /* Enable all MSI-X vectors */
    for (int i = 0; i < CSI2_MSIX_VECTORS; i++) {
        msix_vector_use(pci_dev, i);
    }
    
    /* Create frame timer */
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, csi2_frame_timer, s);
    if (!s->frame_timer) {
        error_setg(errp, "Failed to create frame timer");
        msix_uninit_exclusive_bar(pci_dev);
        return;
    }
    
    CSI2_DPRINTF("AMD CSI-2 RX PCIe device realized successfully");
}

static void amd_csi2_rx_pcie_exit(PCIDevice *pci_dev)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(pci_dev);
    
    CSI2_DPRINTF("Device exit");
    
    /* Stop streaming */
    csi2_stop_streaming(s);
    
    /* Delete timer */
    if (s->frame_timer) {
        timer_free(s->frame_timer);
    }
    
    /* Clean up MSI-X */
    msix_uninit_exclusive_bar(pci_dev);
}

static const VMStateDescription vmstate_amd_csi2_rx_pcie = {
    .name = TYPE_AMD_CSI2_RX_PCIE,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, AmdCsi2RxPcieState),
        VMSTATE_BOOL(enabled, AmdCsi2RxPcieState),
        VMSTATE_BOOL(streaming, AmdCsi2RxPcieState),
        VMSTATE_BOOL(ring_enabled, AmdCsi2RxPcieState),
        VMSTATE_UINT64(ring_base_addr, AmdCsi2RxPcieState),
        VMSTATE_UINT32(ring_size, AmdCsi2RxPcieState),
        VMSTATE_UINT32(ring_head, AmdCsi2RxPcieState),
        VMSTATE_UINT32(ring_tail, AmdCsi2RxPcieState),
        VMSTATE_UINT32(width, AmdCsi2RxPcieState),
        VMSTATE_UINT32(height, AmdCsi2RxPcieState),
        VMSTATE_UINT32(fps, AmdCsi2RxPcieState),
        VMSTATE_UINT32(frame_count, AmdCsi2RxPcieState),
        VMSTATE_UINT32(error_count, AmdCsi2RxPcieState),
        VMSTATE_END_OF_LIST()
    }
};

static void amd_csi2_rx_pcie_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    
    pc->realize = amd_csi2_rx_pcie_realize;
    pc->exit = amd_csi2_rx_pcie_exit;
    pc->vendor_id = XILINX_VENDOR_ID;
    pc->device_id = CSI2_DEVICE_ID;
    pc->class_id = PCI_CLASS_MULTIMEDIA_OTHER;
    rc->phases.hold = amd_csi2_rx_pcie_reset;
    dc->vmsd = &vmstate_amd_csi2_rx_pcie;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo amd_csi2_rx_pcie_info = {
    .name = TYPE_AMD_CSI2_RX_PCIE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(AmdCsi2RxPcieState),
    .class_init = amd_csi2_rx_pcie_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

static void amd_csi2_rx_pcie_register_types(void)
{
    type_register_static(&amd_csi2_rx_pcie_info);
}

type_init(amd_csi2_rx_pcie_register_types);
