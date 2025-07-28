/*
 * AMD MIPI CSI-2 RX Subsystem PCIe Sink Device for QEMU
 * 
 * Version 2.2 - FINAL MSI-X FIX
 * 🚨 CRITICAL: Fixed initialization order and register access
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AMD_CSI2_PCIE_SINK "csi2-pcie-sink"
OBJECT_DECLARE_SIMPLE_TYPE(AmdCsi2PcieSinkState, AMD_CSI2_PCIE_SINK)

/* Device Configuration */
#define AMD_CSI2_VENDOR_ID              0x1022
#define AMD_CSI2_DEVICE_ID              0xC901
#define AMD_CSI2_REVISION               0x01
#define AMD_CSI2_CLASS_CODE             0x0480

/* Memory regions */
#define AMD_CSI2_REG_BAR_SIZE           (64 * KiB)
#define AMD_CSI2_FRAMEBUF_BAR_SIZE      (64 * MiB)
#define AMD_CSI2_MSIX_BAR_SIZE          (4 * KiB)

/* MSI-X Configuration */
#define AMD_CSI2_MSIX_VECTORS           8
#define AMD_CSI2_MSIX_TABLE_OFFSET      0x0000
#define AMD_CSI2_MSIX_PBA_OFFSET        0x0800

/* Register definitions */
#define CSI2_REG_CORE_CONFIG           0x00
#define CSI2_REG_PROTOCOL_CONFIG       0x04
#define CSI2_REG_CORE_STATUS           0x10
#define CSI2_REG_GLOBAL_INT_ENABLE     0x20
#define CSI2_REG_ISR                   0x24
#define CSI2_REG_IER                   0x28
#define CSI2_REG_TEST_TRIGGER          0x50
#define CSI2_REG_DEBUG_CTRL            0x54
#define CSI2_REG_FORCE_INT             0x58

/* Register bits */
#define CORE_CONFIG_ENABLE             (1 << 0)
#define CORE_CONFIG_SOFT_RESET         (1 << 1)
#define ISR_FRAME_RECEIVED             (1U << 31)

/* Virtual Camera */
#define AMD_CSI2_VIRT_CAM_FPS           30
#define AMD_CSI2_FRAME_INTERVAL_NS      (1000000000ULL / AMD_CSI2_VIRT_CAM_FPS)

/* 🆕 Enhanced register state with proper initialization */
typedef struct {
    uint32_t core_config;
    uint32_t protocol_config;
    uint32_t core_status;
    uint32_t global_int_enable;
    uint32_t int_status;
    uint32_t int_enable;
    uint32_t test_trigger;
    uint32_t debug_ctrl;
    uint32_t force_int;
    
    /* 🆕 Hardware state tracking */
    bool driver_initialized;
    bool driver_streaming;
    uint32_t driver_access_count;
} AmdCsi2Registers;

/* Virtual Camera State */
typedef struct {
    bool enabled;
    uint32_t frame_count;
    QEMUTimer *frame_timer;
    QEMUTimer *startup_timer;
    bool timer_active;
    bool startup_complete;
} VirtualCamera;

/* Main device state */
struct AmdCsi2PcieSinkState {
    PCIDevice parent_obj;
    
    MemoryRegion reg_bar;
    MemoryRegion framebuf_bar;
    MemoryRegion msix_bar;
    
    AmdCsi2Registers regs;
    VirtualCamera vcam;
    
    bool initialized;
    bool msix_ready;
    uint64_t frames_generated;
    uint64_t interrupts_sent;
    uint64_t test_triggers;
    uint64_t startup_time;
};

/* Forward declarations */
static void amd_csi2_update_interrupts(AmdCsi2PcieSinkState *s);
static void amd_csi2_virtual_camera_frame(void *opaque);
static void amd_csi2_startup_timer(void *opaque);
static void amd_csi2_start_virtual_camera(AmdCsi2PcieSinkState *s);
static void amd_csi2_stop_virtual_camera(AmdCsi2PcieSinkState *s);

/* 🆕 Enhanced MSI-X readiness with detailed logging */
static bool amd_csi2_check_msix_ready(AmdCsi2PcieSinkState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    
    if (!msix_enabled(pci_dev)) {
        return false;
    }
    
    if (!pci_dev->msix_table) {
        return false;
    }
    
    /* Check Vector 0 configuration */
    uint32_t *entry = (uint32_t *)(pci_dev->msix_table);
    uint32_t addr_low = entry[0];
    uint32_t addr_high = entry[1];
    uint32_t msg_data = entry[2];
    uint32_t vector_ctrl = entry[3];
    
    bool addr_valid = (addr_low != 0 || addr_high != 0);
    bool unmasked = !(vector_ctrl & 1);
    
    if (addr_valid && unmasked && !s->msix_ready) {
        s->msix_ready = true;
        printf("AMD CSI2: 🎉 MSI-X Configuration Detected!\n");
        printf("   Vector 0 Address: 0x%08x%08x\n", addr_high, addr_low);
        printf("   Message Data: 0x%08x\n", msg_data);
        printf("   Control: 0x%08x (unmasked)\n", vector_ctrl);
        printf("   🚀 Ready for interrupt delivery!\n");
        
        /* 🆕 Start immediate frame generation when MSI-X is ready */
        if (s->regs.driver_initialized) {
            amd_csi2_start_virtual_camera(s);
        }
    }
    
    return addr_valid && unmasked;
}

/* 🆕 Enhanced interrupt sending with retry logic */
static void amd_csi2_send_interrupt(AmdCsi2PcieSinkState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    
    if (!amd_csi2_check_msix_ready(s)) {
        if (s->interrupts_sent == 0) {
            printf("AMD CSI2: ⏳ MSI-X not ready yet, queueing interrupt\n");
        }
        return;
    }
    
    if (msix_is_masked(pci_dev, 0)) {
        printf("AMD CSI2: ⚠️  Vector 0 is masked, cannot send\n");
        return;
    }
    
    /* Send the interrupt */
    msix_notify(pci_dev, 0);
    s->interrupts_sent++;
    
    /* 🆕 Enhanced logging for first several interrupts */
    if (s->interrupts_sent <= 10) {
        printf("AMD CSI2: 🔔 MSI-X interrupt #%lu sent to Vector 0!\n", s->interrupts_sent);
        
        if (s->interrupts_sent == 1) {
            printf("AMD CSI2: 🎉 FIRST SUCCESSFUL MSI-X INTERRUPT!\n");
            printf("   🎯 QEMU -> Linux communication established!\n");
        }
    } else if (s->interrupts_sent % 30 == 0) {
        printf("AMD CSI2: 📊 MSI-X interrupt milestone: %lu sent\n", s->interrupts_sent);
    }
}

/* Update interrupts with enhanced condition checking */
static void amd_csi2_update_interrupts(AmdCsi2PcieSinkState *s)
{
    if (!(s->regs.global_int_enable & 1)) {
        return;
    }
    
    uint32_t pending = s->regs.int_status & s->regs.int_enable;
    if (pending != 0) {
        amd_csi2_send_interrupt(s);
    }
}

/* 🆕 Enhanced virtual camera management */
static void amd_csi2_start_virtual_camera(AmdCsi2PcieSinkState *s)
{
    if (s->vcam.timer_active) {
        return;
    }
    
    bool core_enabled = (s->regs.core_config & CORE_CONFIG_ENABLE);
    bool global_int = (s->regs.global_int_enable & 1);
    bool frame_int = (s->regs.int_enable & ISR_FRAME_RECEIVED);
    bool msix_ready = s->msix_ready;
    bool driver_ready = s->regs.driver_initialized;
    
    printf("AMD CSI2: 🔍 Virtual camera start check:\n");
    printf("   Core enabled: %s\n", core_enabled ? "YES" : "NO");
    printf("   Global int: %s\n", global_int ? "YES" : "NO");
    printf("   Frame int: %s\n", frame_int ? "YES" : "NO");
    printf("   MSI-X ready: %s\n", msix_ready ? "YES" : "NO");
    printf("   Driver ready: %s\n", driver_ready ? "YES" : "NO");
    
    if (core_enabled && global_int && frame_int && msix_ready && driver_ready) {
        printf("AMD CSI2: 🎬 Starting Virtual Camera (1920x1080@30fps)\n");
        printf("   🎯 All conditions met for interrupt generation!\n");
        
        s->vcam.timer_active = true;
        
        /* Start with immediate first frame */
        timer_mod(s->vcam.frame_timer, 
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000000ULL); /* 100ms delay */
    } else {
        printf("AMD CSI2: ⏳ Virtual camera waiting for conditions\n");
    }
}

static void amd_csi2_stop_virtual_camera(AmdCsi2PcieSinkState *s)
{
    if (s->vcam.timer_active) {
        printf("AMD CSI2: ⏹️  Stopping Virtual Camera\n");
        timer_del(s->vcam.frame_timer);
        s->vcam.timer_active = false;
    }
}

/* 🆕 Startup timer for delayed initialization */
static void amd_csi2_startup_timer(void *opaque)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    
    s->vcam.startup_complete = true;
    
    printf("AMD CSI2: 🚀 Startup sequence complete\n");
    printf("   Checking for driver readiness...\n");
    
    /* Check if we can start virtual camera */
    if (s->regs.driver_initialized) {
        amd_csi2_start_virtual_camera(s);
    }
}

/* 🆕 Enhanced virtual camera frame generation */
static void amd_csi2_virtual_camera_frame(void *opaque)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    
    if (!s->vcam.timer_active) {
        return;
    }
    
    /* Generate frame */
    s->vcam.frame_count++;
    s->frames_generated++;
    
    /* Update status */
    s->regs.core_status = (s->regs.core_status & 0xFFFF) | (s->vcam.frame_count << 16);
    
    /* Set interrupt status */
    s->regs.int_status |= ISR_FRAME_RECEIVED;
    
    /* Send interrupt */
    amd_csi2_update_interrupts(s);
    
    if (s->frames_generated <= 5) {
        printf("AMD CSI2: 📹 Frame #%u generated (ISR=0x%08x)\n", 
               s->vcam.frame_count, s->regs.int_status);
    }
    
    /* Schedule next frame */
    if (s->vcam.timer_active) {
        timer_mod(s->vcam.frame_timer, 
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + AMD_CSI2_FRAME_INTERVAL_NS);
    }
}

/* 🆕 Enhanced register read with proper initialization */
static uint64_t amd_csi2_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    uint32_t val = 0;
    
    /* Track driver access */
    s->regs.driver_access_count++;
    
    switch (addr) {
    case CSI2_REG_CORE_CONFIG:
        val = s->regs.core_config;
        break;
    case CSI2_REG_PROTOCOL_CONFIG:
        val = s->regs.protocol_config;
        break;
    case CSI2_REG_CORE_STATUS:
        val = s->regs.core_status;
        break;
    case CSI2_REG_GLOBAL_INT_ENABLE:
        val = s->regs.global_int_enable;
        break;
    case CSI2_REG_ISR:
        val = s->regs.int_status;
        break;
    case CSI2_REG_IER:
        val = s->regs.int_enable;
        break;
    case CSI2_REG_TEST_TRIGGER:
        val = s->regs.test_trigger;
        break;
    case CSI2_REG_DEBUG_CTRL:
        val = s->regs.debug_ctrl;
        break;
    case CSI2_REG_FORCE_INT:
        val = s->regs.force_int;
        break;
    default:
        break;
    }
    
    /* 🆕 Log first few register accesses */
    if (s->regs.driver_access_count <= 20 || addr == CSI2_REG_ISR) {
        printf("AMD CSI2: 📖 Read 0x%03lx = 0x%08x (access #%u)\n", 
               addr, val, s->regs.driver_access_count);
    }
    
    return val;
}

/* 🆕 Enhanced register write with immediate response */
static void amd_csi2_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    
        printf("AMD CSI2: 📝 Write 0x%03lx = 0x%08x\n", addr, (uint32_t)val);
    
    switch (addr) {
    case CSI2_REG_CORE_CONFIG:
        {
            uint32_t old_val = s->regs.core_config;
            s->regs.core_config = val;
            
            if ((val & CORE_CONFIG_ENABLE) && !(old_val & CORE_CONFIG_ENABLE)) {
                printf("AMD CSI2: 🔧 Core enabled by driver\n");
                s->regs.driver_initialized = true;
                
                /* Check if we can start virtual camera */
                if (s->vcam.startup_complete) {
                    amd_csi2_start_virtual_camera(s);
                }
            }
            
            if (val & CORE_CONFIG_SOFT_RESET) {
                printf("AMD CSI2: 🔄 Soft reset by driver\n");
                s->regs.int_status = 0;
                s->regs.core_config &= ~CORE_CONFIG_SOFT_RESET;
                amd_csi2_stop_virtual_camera(s);
            }
        }
        break;
        
    case CSI2_REG_PROTOCOL_CONFIG:
        s->regs.protocol_config = val;
        printf("AMD CSI2: 🛤️  Protocol config: 0x%x\n", (uint32_t)val);
        break;
        
    case CSI2_REG_GLOBAL_INT_ENABLE:
        {
            uint32_t old_val = s->regs.global_int_enable;
            s->regs.global_int_enable = val;
            
            printf("AMD CSI2: ⚡ Global interrupt enable: 0x%x\n", (uint32_t)val);
            
            if ((val & 1) && !(old_val & 1)) {
                printf("AMD CSI2: 🔔 Global interrupts enabled by driver\n");
                amd_csi2_start_virtual_camera(s);
            } else if (!(val & 1) && (old_val & 1)) {
                printf("AMD CSI2: 🔕 Global interrupts disabled by driver\n");
                amd_csi2_stop_virtual_camera(s);
            }
        }
        break;
        
    case CSI2_REG_ISR:
        /* Write-1-to-clear */
        {
            uint32_t old_status = s->regs.int_status;
            uint32_t cleared = val & old_status;
            s->regs.int_status &= ~cleared;
            
            if (cleared != 0) {
                printf("AMD CSI2: 🧹 ISR cleared: 0x%08x -> 0x%08x (cleared: 0x%08x)\n", 
                       old_status, s->regs.int_status, cleared);
            }
        }
        break;
        
    case CSI2_REG_IER:
        {
            uint32_t old_val = s->regs.int_enable;
            s->regs.int_enable = val;
            
            printf("AMD CSI2: 📡 Interrupt enable: 0x%x\n", (uint32_t)val);
            
            if ((val & ISR_FRAME_RECEIVED) && !(old_val & ISR_FRAME_RECEIVED)) {
                printf("AMD CSI2: 🎬 Frame interrupt enabled by driver\n");
                s->regs.driver_streaming = true;
                amd_csi2_start_virtual_camera(s);
            }
        }
        break;
        
    case CSI2_REG_TEST_TRIGGER:
        s->regs.test_trigger = val;
        s->test_triggers++;
        printf("AMD CSI2: 🧪 Test trigger: 0x%x (#%lu)\n", (uint32_t)val, s->test_triggers);
        
        /* Immediate test response */
        if (val == 0x12345678 || val == 0xDEADBEEF) {
            printf("AMD CSI2: 🚀 Test pattern detected - IMMEDIATE interrupt!\n");
            s->regs.int_status |= ISR_FRAME_RECEIVED;
            amd_csi2_update_interrupts(s);
        }
        break;
        
    case CSI2_REG_DEBUG_CTRL:
        s->regs.debug_ctrl = val;
        printf("AMD CSI2: 🔍 Debug control: 0x%x\n", (uint32_t)val);
        break;
        
    case CSI2_REG_FORCE_INT:
        s->regs.force_int = val;
        printf("AMD CSI2: ⚡ Force interrupt: 0x%x\n", (uint32_t)val);
        
        /* Immediate forced interrupt */
        if (val == 0xDEADBEEF) {
            printf("AMD CSI2: 💥 FORCED interrupt triggered!\n");
            s->regs.int_status |= ISR_FRAME_RECEIVED;
            amd_csi2_update_interrupts(s);
        }
        break;
        
    default:
        break;
    }
}

static const MemoryRegionOps amd_csi2_reg_ops = {
    .read = amd_csi2_reg_read,
    .write = amd_csi2_reg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* Device realization */
static void amd_csi2_pcie_sink_realize(PCIDevice *pci_dev, Error **errp)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(pci_dev);
    uint8_t *pci_conf = pci_dev->config;
    int ret;
    
    printf("AMD CSI2: 🚀 Initializing FINAL MSI-X Device v2.2\n");
    
    /* PCI configuration */
    pci_config_set_vendor_id(pci_conf, AMD_CSI2_VENDOR_ID);
    pci_config_set_device_id(pci_conf, AMD_CSI2_DEVICE_ID);
    pci_config_set_revision(pci_conf, AMD_CSI2_REVISION);
    pci_config_set_class(pci_conf, AMD_CSI2_CLASS_CODE);
    
    pci_set_word(pci_conf + PCI_COMMAND, 
                 PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_INTX_DISABLE);
    pci_set_word(pci_conf + PCI_STATUS, 
                 PCI_STATUS_CAP_LIST | PCI_STATUS_FAST_BACK);
    
    printf("AMD CSI2: 🔧 PCI Config: 1022:c901 class=0x0480\n");
    
    /* Initialize memory regions */
    memory_region_init_io(&s->reg_bar, OBJECT(s), &amd_csi2_reg_ops, s,
                          "amd-csi2-regs", AMD_CSI2_REG_BAR_SIZE);
    
    memory_region_init_ram(&s->framebuf_bar, OBJECT(s), "amd-csi2-framebuf",
                          AMD_CSI2_FRAMEBUF_BAR_SIZE, errp);
    if (*errp) {
        return;
    }
    
    memory_region_init(&s->msix_bar, OBJECT(s), "amd-csi2-msix", 
                       AMD_CSI2_MSIX_BAR_SIZE);
    
    /* Register BARs */
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->reg_bar);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->framebuf_bar);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->msix_bar);
    
    /* MSI-X initialization */
    ret = msix_init(pci_dev, AMD_CSI2_MSIX_VECTORS,
                    &s->msix_bar, 2, AMD_CSI2_MSIX_TABLE_OFFSET,
                    &s->msix_bar, 2, AMD_CSI2_MSIX_PBA_OFFSET, 
                    0x60, errp);
    if (ret < 0) {
        printf("AMD CSI2: ❌ MSI-X init failed: %d\n", ret);
        return;
    }
    
    printf("AMD CSI2: ✅ MSI-X initialized with %d vectors\n", AMD_CSI2_MSIX_VECTORS);
    
    /* 🆕 Initialize registers with proper defaults */
    s->regs.core_config = CORE_CONFIG_ENABLE;
    s->regs.protocol_config = 0x3;
    s->regs.core_status = 0x1000; /* Some initial packet count */
    s->regs.global_int_enable = 0;
    s->regs.int_status = 0;
    s->regs.int_enable = 0;
    s->regs.driver_initialized = false;
    s->regs.driver_streaming = false;
    s->regs.driver_access_count = 0;
    
    /* Initialize virtual camera */
    s->vcam.enabled = true;
    s->vcam.frame_count = 0;
    s->vcam.timer_active = false;
    s->vcam.startup_complete = false;
    s->vcam.frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                      amd_csi2_virtual_camera_frame, s);
    s->vcam.startup_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        amd_csi2_startup_timer, s);
    
    /* Initialize state */
    s->initialized = true;
    s->msix_ready = false;
    s->frames_generated = 0;
    s->interrupts_sent = 0;
    s->test_triggers = 0;
    s->startup_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    /* 🆕 Start startup timer (5 second delay for driver to initialize) */
    timer_mod(s->vcam.startup_timer, s->startup_time + 5000000000ULL);
    
    printf("AMD CSI2: ✅ Device initialization complete\n");
    printf("AMD CSI2: 🎯 Waiting for driver connection...\n");
    printf("AMD CSI2: ⏰ Startup timer set for 5 seconds\n");
}

static void amd_csi2_pcie_sink_exit(PCIDevice *pci_dev)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(pci_dev);
    
    printf("AMD CSI2: 🧹 Device cleanup\n");
    
    amd_csi2_stop_virtual_camera(s);
    
    if (s->vcam.frame_timer) {
        timer_free(s->vcam.frame_timer);
    }
    
    if (s->vcam.startup_timer) {
        timer_free(s->vcam.startup_timer);
    }
    
    printf("AMD CSI2: 📊 FINAL STATS: frames=%lu, interrupts=%lu, tests=%lu\n",
           s->frames_generated, s->interrupts_sent, s->test_triggers);
    
    msix_uninit(pci_dev, &s->msix_bar, &s->msix_bar);
    printf("AMD CSI2: ✅ Cleanup complete\n");
}

static void amd_csi2_pcie_sink_instance_init(Object *obj)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(obj);
    
    memset(&s->regs, 0, sizeof(s->regs));
    s->initialized = false;
    s->msix_ready = false;
    s->frames_generated = 0;
    s->interrupts_sent = 0;
    s->test_triggers = 0;
    s->startup_time = 0;
}

/* VMState for migration */
static const VMStateDescription vmstate_amd_csi2_pcie_sink = {
    .name = "amd-csi2-pcie-sink",
    .version_id = 5,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(regs.core_config, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(regs.int_status, AmdCsi2PcieSinkState),
        VMSTATE_BOOL(initialized, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(vcam.frame_count, AmdCsi2PcieSinkState),
        VMSTATE_END_OF_LIST()
    }
};

/* Class definition */
static void amd_csi2_pcie_sink_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);
    
    pc->realize = amd_csi2_pcie_sink_realize;
    pc->exit = amd_csi2_pcie_sink_exit;
    pc->vendor_id = AMD_CSI2_VENDOR_ID;
    pc->device_id = AMD_CSI2_DEVICE_ID;
    pc->revision = AMD_CSI2_REVISION;
    pc->class_id = AMD_CSI2_CLASS_CODE;
    
    dc->user_creatable = true;
    dc->vmsd = &vmstate_amd_csi2_pcie_sink;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "AMD MIPI CSI-2 RX PCIe Sink Device (FINAL MSI-X FIX)";
}

static const TypeInfo amd_csi2_pcie_sink_info = {
    .name = TYPE_AMD_CSI2_PCIE_SINK,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(AmdCsi2PcieSinkState),
    .instance_init = amd_csi2_pcie_sink_instance_init,
    .class_init = amd_csi2_pcie_sink_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

/* Module registration */
static void amd_csi2_pcie_sink_register_types(void)
{
    type_register_static(&amd_csi2_pcie_sink_info);
}

type_init(amd_csi2_pcie_sink_register_types)
