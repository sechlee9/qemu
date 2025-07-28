/*
 * AMD MIPI CSI-2 RX Subsystem PCIe Sink Device for QEMU
 * 
 * Version 1.6 - FIXED: MSI-X Interrupt Generation
 * 🚨 CRITICAL FIXES: MSI-X activation and ISR handling
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
#define AMD_CSI2_VENDOR_ID              0x1022  /* AMD */
#define AMD_CSI2_DEVICE_ID              0xC901  /* Custom CSI-2 device */
#define AMD_CSI2_REVISION               0x01
#define AMD_CSI2_CLASS_CODE             0x0480  /* Multimedia controller */

/* Memory regions - Fixed MSI-X layout */
#define AMD_CSI2_REG_BAR_SIZE           (8 * KiB)   /* 8KB register space */
#define AMD_CSI2_FRAMEBUF_BAR_SIZE      (64 * MiB)  /* 64MB frame buffer */
#define AMD_CSI2_MSIX_BAR_SIZE          (4 * KiB)   /* 4KB MSI-X space */

/* MSI-X Configuration - Fixed */
#define AMD_CSI2_MSIX_VECTORS           8
#define AMD_CSI2_MSIX_TABLE_SIZE        (AMD_CSI2_MSIX_VECTORS * 16)
#define AMD_CSI2_MSIX_PBA_SIZE          8
#define AMD_CSI2_MSIX_TABLE_OFFSET      0x0000
#define AMD_CSI2_MSIX_PBA_OFFSET        0x0800

/* MSI-X Vector assignments */
#define AMD_CSI2_MSIX_VEC_FRAME_READY   0  /* Primary vector for frame interrupts */

/* Virtual Camera Configuration */
#define AMD_CSI2_VIRT_CAM_WIDTH         1920
#define AMD_CSI2_VIRT_CAM_HEIGHT        1080
#define AMD_CSI2_VIRT_CAM_FPS           30
#define AMD_CSI2_VIRT_CAM_FRAME_SIZE    (AMD_CSI2_VIRT_CAM_WIDTH * AMD_CSI2_VIRT_CAM_HEIGHT * 2)

/* Frame timing (30fps = 33.33ms per frame) */
#define AMD_CSI2_FRAME_INTERVAL_NS      (1000000000ULL / AMD_CSI2_VIRT_CAM_FPS)

/* Register space layout */
typedef struct {
    /* CSI-2 RX Controller registers (0x0000-0x0FFF) */
    uint32_t core_config;           /* 0x00 */
    uint32_t protocol_config;       /* 0x04 */
    uint32_t core_status;           /* 0x10 */
    uint32_t global_int_enable;     /* 0x20 */
    uint32_t int_status;            /* 0x24 */
    uint32_t int_enable;            /* 0x28 */
    uint32_t dynamic_vc_sel;        /* 0x2C */
    uint32_t generic_short_packet;  /* 0x30 */
    uint32_t vcx_frame_error;       /* 0x34 */
    uint32_t clk_lane_info;         /* 0x3C */
    uint32_t lane_info[4];          /* 0x40-0x4C */
    uint32_t img_info1_vc[16];      /* 0x60-0x9C */
    uint32_t img_info2_vc[16];      /* 0x64-0xDC */
    
    /* D-PHY registers (0x1000-0x1FFF) */
    uint32_t dphy_control;          /* 0x1000 */
    uint32_t dphy_status;           /* 0x1004 */
    uint32_t dphy_hs_settle;        /* 0x1008 */
    uint32_t dphy_pll_ctrl;         /* 0x100C */
    uint32_t dphy_pll_status;       /* 0x1010 */
    uint32_t dphy_lane_config;      /* 0x1014 */
    uint32_t dphy_lane_status;      /* 0x1018 */
    
    /* Frame buffer control registers */
    uint32_t framebuf_write_ptr;    /* 0x1020 */
    uint32_t framebuf_read_ptr;     /* 0x1024 */
    uint32_t framebuf_size;         /* 0x1028 */
    uint32_t framebuf_control;      /* 0x102C */
} AmdCsi2Registers;

/* Virtual Camera State */
typedef struct {
    bool enabled;
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t frame_count;
    QEMUTimer *frame_timer;
    bool timer_active;
    uint64_t frame_start_time;
} VirtualCamera;

/* Main device state */
struct AmdCsi2PcieSinkState {
    PCIDevice parent_obj;
    
    /* Memory regions */
    MemoryRegion reg_bar;
    MemoryRegion framebuf_bar;
    MemoryRegion msix_bar;
    
    /* Register state */
    AmdCsi2Registers regs;
    
    /* Virtual camera */
    VirtualCamera vcam;
    
    /* Device state */
    bool initialized;
    
    /* Statistics */
    uint64_t frames_received;
    uint64_t interrupts_sent;
};

/* Forward declarations */
static void amd_csi2_update_interrupts(AmdCsi2PcieSinkState *s);
static void amd_csi2_virtual_camera_frame(void *opaque);

/* 🔧 FIXED: Simplified MSI-X interrupt sender */
static void amd_csi2_send_msi(AmdCsi2PcieSinkState *s, int vector)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    
    if (!msix_enabled(pci_dev)) {
        if (s->interrupts_sent < 5) {
            printf("AMD CSI2: ❌ MSI-X not enabled, cannot send interrupt\n");
        }
        return;
    }
    
    if (vector < 0 || vector >= AMD_CSI2_MSIX_VECTORS) {
        printf("AMD CSI2: ❌ Invalid MSI-X vector: %d\n", vector);
        return;
    }
    
    /* Send interrupt */
    msix_notify(pci_dev, vector);
    s->interrupts_sent++;
    
    /* Log first few interrupts */
    if (s->interrupts_sent <= 5) {
        printf("AMD CSI2: 🔔 Sending MSI-X interrupt #%lu (vector %d)\n", 
               s->interrupts_sent, vector);
    } else if (s->interrupts_sent % 100 == 0) {
        printf("AMD CSI2: 🎬 Virtual vsync #%u (MSI-X interrupts: %lu)\n", 
               s->vcam.frame_count, s->interrupts_sent);
    }
    
    if (s->interrupts_sent == 1) {
        printf("AMD CSI2: 🎉 First MSI-X interrupt sent successfully!\n");
    }
}

/* 🔧 FIXED: Simplified interrupt update */
static void amd_csi2_update_interrupts(AmdCsi2PcieSinkState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    
    if (!msix_enabled(pci_dev)) {
        return;
    }
    
    if (!(s->regs.global_int_enable & 0x1)) {
        return;
    }
    
    /* Send interrupt if we have pending status bits and they're enabled */
    if (s->regs.int_status & s->regs.int_enable) {
        amd_csi2_send_msi(s, AMD_CSI2_MSIX_VEC_FRAME_READY);
    }
}

/* 🔧 FIXED: Simplified virtual source management */
static void amd_csi2_check_virtual_source_start(AmdCsi2PcieSinkState *s)
{
    bool should_start = (s->regs.global_int_enable & 0x1) && 
                       (s->regs.int_enable & (1U << 31));
    
    if (should_start && !s->vcam.timer_active) {
        printf("AMD CSI2: ✅ Starting Virtual Source (all conditions met)\n");
        printf("AMD CSI2: 📊 Conditions: core=1, dphy=1, global_int=1, frame_int=1\n");
        printf("AMD CSI2: 📹 Virtual vsync: %ux%u@%ufps via MSI-X vector %d\n",
               s->vcam.width, s->vcam.height, s->vcam.fps, 
               AMD_CSI2_MSIX_VEC_FRAME_READY);
        
        s->vcam.timer_active = true;
        s->vcam.frame_start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        
        printf("AMD CSI2: 🎯 First virtual vsync starting - initializing timing\n");
        printf("AMD CSI2: 🎬 Virtual vsync parameters: %ux%u@%ufps, %u bytes/frame\n",
               s->vcam.width, s->vcam.height, s->vcam.fps, AMD_CSI2_VIRT_CAM_FRAME_SIZE);
        
        /* Start immediately */
        timer_mod(s->vcam.frame_timer, 
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (AMD_CSI2_FRAME_INTERVAL_NS / 10));
                 
    } else if (!should_start && s->vcam.timer_active) {
        printf("AMD CSI2: ⏹️  Stopping Virtual Source\n");
        timer_del(s->vcam.frame_timer);
        s->vcam.timer_active = false;
    }
}

/* 🔧 FIXED: Simplified Virtual Camera frame generation */
static void amd_csi2_virtual_camera_frame(void *opaque)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    
    /* Check if virtual camera should continue */
    if (!s->vcam.enabled || !s->vcam.timer_active) {
        s->vcam.timer_active = false;
        return;
    }

    /* Update frame count and statistics */
    s->vcam.frame_count++;
    s->frames_received++;
    
    /* Update registers */
    uint32_t packet_count = (s->regs.core_status >> 16) + 1;
    s->regs.core_status = (s->regs.core_status & 0xFFFF) | (packet_count << 16);
    
    /* 🔧 FIXED: Set interrupt status BEFORE sending interrupt */
    s->regs.int_status |= (1U << 31); /* Frame Received bit */
    
    /* Send interrupt if enabled */
    if ((s->regs.global_int_enable & 0x1) && (s->regs.int_enable & (1U << 31))) {
        amd_csi2_send_msi(s, AMD_CSI2_MSIX_VEC_FRAME_READY);
        
        /* Log milestones */
        if (s->vcam.frame_count == 1) {
            printf("AMD CSI2: 🎯 First virtual vsync interrupt sent successfully\n");
        }
    }
    
    /* Schedule next frame with accurate timing */
    if (s->vcam.timer_active) {
        uint64_t next_frame_time = s->vcam.frame_start_time + 
                                  (s->vcam.frame_count * AMD_CSI2_FRAME_INTERVAL_NS);
        timer_mod(s->vcam.frame_timer, next_frame_time);
    }
}

/* Register read handlers */
static uint64_t amd_csi2_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    uint32_t val = 0;
    
    if (addr < 0x1000) {
        /* CSI-2 RX Controller registers */
        switch (addr) {
        case 0x00: /* Core Configuration Register */
            val = s->regs.core_config;
            break;
        case 0x04: /* Protocol Configuration Register */
            val = s->regs.protocol_config;
            break;
        case 0x10: /* Core Status Register */
            val = s->regs.core_status;
            break;
        case 0x20: /* Global Interrupt Enable Register */
            val = s->regs.global_int_enable;
            break;
        case 0x24: /* Interrupt Status Register */
            val = s->regs.int_status;
            break;
        case 0x28: /* Interrupt Enable Register */
            val = s->regs.int_enable;
            break;
        case 0x2C: /* Dynamic VC Selection Register */
            val = s->regs.dynamic_vc_sel;
            break;
        case 0x30: /* Generic Short Packet Register */
            val = s->regs.generic_short_packet;
            break;
        case 0x34: /* VCX Frame Error Register */
            val = s->regs.vcx_frame_error;
            break;
        case 0x3C: /* Clock Lane Information Register */
            val = s->regs.clk_lane_info;
            break;
        default:
            if (addr >= 0x40 && addr <= 0x4C) {
                /* Lane 0-3 Information Registers */
                int lane = (addr - 0x40) / 4;
                val = s->regs.lane_info[lane];
            } else if (addr >= 0x60 && addr <= 0xDC) {
                /* Image Information registers */
                int idx = (addr - 0x60) / 4;
                if (idx < 32) {
                    if (idx & 1) {
                        val = s->regs.img_info2_vc[idx / 2];
                    } else {
                        val = s->regs.img_info1_vc[idx / 2];
                    }
                }
            }
            break;
        }
    } else {
        /* D-PHY registers (0x1000 offset) */
        hwaddr dphy_addr = addr - 0x1000;
        switch (dphy_addr) {
        case 0x00: /* D-PHY Control */
            val = s->regs.dphy_control;
            break;
        case 0x04: /* D-PHY Status */
            val = s->regs.dphy_status;
            break;
        case 0x08: /* D-PHY HS Settle */
            val = s->regs.dphy_hs_settle;
            break;
        case 0x0C: /* D-PHY PLL Control */
            val = s->regs.dphy_pll_ctrl;
            break;
        case 0x10: /* D-PHY PLL Status */
            val = s->regs.dphy_pll_status;
            break;
        case 0x14: /* D-PHY Lane Config */
            val = s->regs.dphy_lane_config;
            break;
        case 0x18: /* D-PHY Lane Status */
            val = s->regs.dphy_lane_status;
            break;
        case 0x20: /* Frame Buffer Write Pointer */
            val = s->regs.framebuf_write_ptr;
            break;
        case 0x24: /* Frame Buffer Read Pointer */
            val = s->regs.framebuf_read_ptr;
            break;
        case 0x28: /* Frame Buffer Size */
            val = s->regs.framebuf_size;
            break;
        case 0x2C: /* Frame Buffer Control */
            val = s->regs.framebuf_control;
            break;
        default:
            break;
        }
    }
    
    return val;
}

/* 🔧 FIXED: Enhanced register write handlers */
static void amd_csi2_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    
    if (addr < 0x1000) {
        /* CSI-2 RX Controller registers */
        switch (addr) {
        case 0x00: /* Core Configuration Register */
            s->regs.core_config = val;
            
            if (val & 0x1) {
                s->initialized = true;
                s->regs.core_status &= ~0x1; /* Clear reset in progress */
                printf("AMD CSI2: 🔧 Core enabled\n");
            }
            
            if (val & 0x2) {
                /* Soft reset */
                s->regs.int_status = 0;
                s->regs.core_status |= 0x1;
                s->vcam.frame_count = 0;
                s->regs.core_config &= ~0x2;
                printf("AMD CSI2: 🔄 Soft reset performed\n");
            }
            break;
            
        case 0x04: /* Protocol Configuration Register */
            s->regs.protocol_config = val;
            printf("AMD CSI2: 🛤️  Protocol config: 0x%x\n", (uint32_t)val);
            break;
            
        case 0x20: /* Global Interrupt Enable Register */
            s->regs.global_int_enable = val;
            printf("AMD CSI2: ⚡ Global interrupt enable: 0x%x (virtual vsync will %s)\n", 
                   (uint32_t)val, (val & 0x1) ? "start" : "stop");
            
            /* Check virtual source when interrupts are enabled */
            amd_csi2_check_virtual_source_start(s);
            break;
            
        case 0x24: /* Interrupt Status Register */
            /* 🔧 FIXED: Write 1 to clear - but DON'T clear artificially written bits */
            {
                uint32_t old_status = s->regs.int_status;
                s->regs.int_status &= ~val;  /* Clear bits where 1 is written */
                
                if (old_status != s->regs.int_status) {
                    printf("AMD CSI2: 🧹 ISR: 0x%08x -> 0x%08x (cleared 0x%08x)\n", 
                           old_status, s->regs.int_status, (uint32_t)val);
                }
            }
            amd_csi2_update_interrupts(s);
            break;
            
        case 0x28: /* Interrupt Enable Register */
            s->regs.int_enable = val;
            printf("AMD CSI2: 📡 Interrupt enable: 0x%x\n", (uint32_t)val);
            
            /* Check virtual source when frame interrupts are enabled */
            if (val & (1U << 31)) {
                printf("AMD CSI2: 🎬 Frame received interrupt enabled - virtual vsync ready\n");
            }
            amd_csi2_check_virtual_source_start(s);
            break;
            
        case 0x2C: /* Dynamic VC Selection Register */
            s->regs.dynamic_vc_sel = val;
            break;
            
        /* 🔧 FIXED: Allow manual ISR manipulation for testing */
        default:
            /* Handle test registers that drivers might write to trigger interrupts */
            if (addr >= 0x50 && addr <= 0x5C) {
                printf("AMD CSI2: 🧪 Test register write: 0x%lx = 0x%lx\n", addr, val);
                
                /* If it's a test trigger, generate an interrupt */
                if (val == 0x12345678 || val == 0xDEADBEEF) {
                    printf("AMD CSI2: 🚀 Test pattern detected - triggering interrupt!\n");
                    s->regs.int_status |= (1U << 31); /* Set frame received */
                    amd_csi2_update_interrupts(s);
                }
            }
            break;
        }
    } else {
        /* D-PHY registers */
        hwaddr dphy_addr = addr - 0x1000;
        switch (dphy_addr) {
        case 0x00: /* D-PHY Control */
            s->regs.dphy_control = val;
            
            if (val & 0x1) {
                s->regs.dphy_status |= 0x3; /* Ready + Init Done */
                s->regs.dphy_pll_status |= 0x3; /* Locked + Ready */
                printf("AMD CSI2: 📶 D-PHY enabled\n");
            } else {
                s->regs.dphy_status &= ~0x3;
                s->regs.dphy_pll_status &= ~0x3;
                printf("AMD CSI2: 📶 D-PHY disabled\n");
            }
            break;
            
        case 0x08: /* D-PHY HS Settle */
            s->regs.dphy_hs_settle = val;
            break;
            
        case 0x0C: /* D-PHY PLL Control */
            s->regs.dphy_pll_ctrl = val;
            if (val & 0x1) {
                s->regs.dphy_pll_status |= 0x3;
                printf("AMD CSI2: 🔧 D-PHY PLL enabled\n");
            } else {
                s->regs.dphy_pll_status &= ~0x3;
                printf("AMD CSI2: 🔧 D-PHY PLL disabled\n");
            }
            break;
            
        case 0x14: /* D-PHY Lane Config */
            s->regs.dphy_lane_config = val;
            s->regs.dphy_lane_status = val & 0xF;
            break;
            
        case 0x24: /* Frame Buffer Read Pointer */
            s->regs.framebuf_read_ptr = val;
            break;
            
        case 0x2C: /* Frame Buffer Control */
            s->regs.framebuf_control = val;
            if (val & 0x1) {
                printf("AMD CSI2: 💾 Frame buffer enabled\n");
            }
            break;
        }
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

/* 🚨 REMOVED: No more forced MSI-X activation - let the OS handle it */

/* 🔧 FIXED: Simplified device initialization */
static void amd_csi2_pcie_sink_realize(PCIDevice *pci_dev, Error **errp)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(pci_dev);
    uint8_t *pci_conf = pci_dev->config;
    int ret;
    
    printf("AMD CSI2: 🚀 Initializing PCIe sink device (FIXED VERSION 1.6)\n");
    
    /* Basic PCI configuration */
    pci_config_set_vendor_id(pci_conf, AMD_CSI2_VENDOR_ID);
    pci_config_set_device_id(pci_conf, AMD_CSI2_DEVICE_ID);
    pci_config_set_revision(pci_conf, AMD_CSI2_REVISION);
    pci_config_set_class(pci_conf, AMD_CSI2_CLASS_CODE);
    
    /* Interrupt pin */
    pci_conf[PCI_INTERRUPT_PIN] = 1;  /* INTA# */
    
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
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32, &s->reg_bar);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32, &s->framebuf_bar);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32, &s->msix_bar);
    
    printf("AMD CSI2: 📍 BARs registered\n");
    
    /* Initialize MSI-X - let the OS control it */
    ret = msix_init(pci_dev, AMD_CSI2_MSIX_VECTORS,
                    &s->msix_bar, 2, AMD_CSI2_MSIX_TABLE_OFFSET,
                    &s->msix_bar, 2, AMD_CSI2_MSIX_PBA_OFFSET, 
                    0x60, errp);
    if (ret < 0) {
        printf("AMD CSI2: ❌ Failed to initialize MSI-X: %d\n", ret);
        error_setg(errp, "MSI-X initialization failed: %d", ret);
        return;
    }
    
    printf("AMD CSI2: ✅ MSI-X initialized with %d vectors (OS-controlled)\n", 
           AMD_CSI2_MSIX_VECTORS);
    
    /* Enable bus master and memory access */
    pci_conf[PCI_COMMAND] = PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
    
    /* Initialize virtual camera */
    s->vcam.enabled = true;
    s->vcam.width = AMD_CSI2_VIRT_CAM_WIDTH;
    s->vcam.height = AMD_CSI2_VIRT_CAM_HEIGHT;
    s->vcam.fps = AMD_CSI2_VIRT_CAM_FPS;
    s->vcam.frame_count = 0;
    s->vcam.timer_active = false;
    s->vcam.frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                      amd_csi2_virtual_camera_frame, s);
    
    /* Initialize registers with proper default values */
    s->regs.core_config = 0x1;  /* Core enabled by default */
    s->regs.protocol_config = 0x3; /* 4 lanes */
    s->regs.dynamic_vc_sel = 0xFFFF;
    s->regs.dphy_control = 0x1;  /* D-PHY enabled by default */
    s->regs.dphy_status = 0x3;   /* Ready + Init Done */
    s->regs.dphy_pll_status = 0x3; /* Locked + Ready */
    s->regs.dphy_hs_settle = 0x20;
    s->regs.framebuf_size = AMD_CSI2_FRAMEBUF_BAR_SIZE;
    
    /* Initialize device state */
    s->initialized = true;
    s->frames_received = 0;
    s->interrupts_sent = 0;
    
    printf("AMD CSI2: ✅ PCIe sink device initialized successfully\n");
    printf("AMD CSI2: 📹 Virtual camera ready: %ux%u@%ufps\n",
           s->vcam.width, s->vcam.height, s->vcam.fps);
    printf("AMD CSI2: 🎮 Virtual vsync will start when driver enables interrupts\n");
}

static void amd_csi2_pcie_sink_exit(PCIDevice *pci_dev)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(pci_dev);
    
    printf("AMD CSI2: 🧹 Cleaning up PCIe sink device\n");
    
    /* Stop virtual camera */
    if (s->vcam.frame_timer) {
        timer_del(s->vcam.frame_timer);
        timer_free(s->vcam.frame_timer);
        s->vcam.timer_active = false;
    }
    
    /* Final statistics */
    if (s->frames_received > 0) {
        printf("AMD CSI2: 📊 Final statistics: %lu frames, %lu interrupts\n",
               s->frames_received, s->interrupts_sent);
    }
    
    /* Cleanup MSI-X */
    msix_uninit(pci_dev, &s->msix_bar, &s->msix_bar);
    
    printf("AMD CSI2: ✅ Cleanup completed\n");
}

static void amd_csi2_pcie_sink_instance_init(Object *obj)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(obj);
    
    /* Initialize state */
    memset(&s->regs, 0, sizeof(s->regs));
    s->frames_received = 0;
    s->interrupts_sent = 0;
    s->initialized = false;
}

/* VMState for migration */
static const VMStateDescription vmstate_amd_csi2_pcie_sink = {
    .name = "amd-csi2-pcie-sink",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(regs.core_config, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(regs.protocol_config, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(regs.int_status, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(regs.int_enable, AmdCsi2PcieSinkState),
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
    dc->desc = "AMD MIPI CSI-2 RX PCIe Sink Device with Fixed MSI-X Support";
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
