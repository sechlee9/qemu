/*
 * AMD MIPI CSI-2 RX Subsystem PCIe Sink Device for QEMU
 * 
 * Version 3.1 - PCI INTERRUPT ROUTING FIX
 * 🎯 FIXED: Added missing PCI interrupt pin configuration
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

/* AMD CSI-2 Complete Register Map (from spec) */
#define CSI2_REG_CORE_CONFIG           0x00
#define CSI2_REG_PROTOCOL_CONFIG       0x04
#define CSI2_REG_CORE_STATUS           0x10
#define CSI2_REG_GLOBAL_INT_ENABLE     0x20
#define CSI2_REG_ISR                   0x24
#define CSI2_REG_IER                   0x28
#define CSI2_REG_DYNAMIC_VC_SEL        0x2C
#define CSI2_REG_GENERIC_SHORT_PACKET  0x30
#define CSI2_REG_VCX_FRAME_ERROR       0x34
#define CSI2_REG_CLK_LANE_INFO         0x3C

/* Lane Information Registers */
#define CSI2_REG_LANE0_INFO            0x40
#define CSI2_REG_LANE1_INFO            0x44
#define CSI2_REG_LANE2_INFO            0x48
#define CSI2_REG_LANE3_INFO            0x4C

/* Test/Debug registers */
#define CSI2_REG_TEST_TRIGGER          0x50
#define CSI2_REG_DEBUG_CTRL            0x54
#define CSI2_REG_FORCE_INT             0x58

/* Image Information Registers (VC0-VC15) */
#define CSI2_REG_IMG_INFO1_VC0         0x60
#define CSI2_REG_IMG_INFO2_VC0         0x64
#define CSI2_REG_IMG_INFO1_VC1         0x68
#define CSI2_REG_IMG_INFO2_VC1         0x6C
/* ... up to VC15 at 0xD8, 0xDC */

/* D-PHY registers (0x1000 offset) */
#define CSI2_REG_DPHY_CONTROL          0x1000
#define CSI2_REG_DPHY_STATUS           0x1004
#define CSI2_REG_DPHY_HS_SETTLE        0x1008
#define CSI2_REG_DPHY_PLL_CTRL         0x100C
#define CSI2_REG_DPHY_PLL_STATUS       0x1010

/* Register bits (from AMD spec) */
#define CORE_CONFIG_ENABLE             BIT(0)
#define CORE_CONFIG_SOFT_RESET         BIT(1)
#define CORE_CONFIG_FULL_RESET         BIT(2)

/* ISR bits (complete from spec) */
#define ISR_FRAME_RECEIVED             BIT(31)
#define ISR_VCX_FRAME_ERROR            BIT(30)
#define ISR_RX_SKEWCALHS               BIT(29)
#define ISR_YUV420_WC_ERROR            BIT(28)
#define ISR_PENDING_WRITE_FIFO         BIT(27)
#define ISR_WC_CORRUPTION              BIT(22)
#define ISR_INCORRECT_LANE_CONFIG      BIT(21)
#define ISR_SHORT_PACKET_FIFO_FULL     BIT(20)
#define ISR_SHORT_PACKET_FIFO_NOT_EMPTY BIT(19)
#define ISR_STREAM_LINE_BUFFER_FULL    BIT(18)
#define ISR_STOP_STATE                 BIT(17)
#define ISR_SOT_ERROR                  BIT(13)
#define ISR_SOT_SYNC_ERROR             BIT(12)
#define ISR_ECC_2_BIT_ERROR            BIT(11)
#define ISR_ECC_1_BIT_ERROR            BIT(10)
#define ISR_CRC_ERROR                  BIT(9)
#define ISR_UNSUPPORTED_DATA_TYPE      BIT(8)

/* Virtual Camera */
#define AMD_CSI2_VIRT_CAM_FPS           30
#define AMD_CSI2_FRAME_INTERVAL_NS      (1000000000ULL / AMD_CSI2_VIRT_CAM_FPS)

/* Complete register state */
typedef struct {
    uint32_t core_config;
    uint32_t protocol_config;
    uint32_t core_status;
    uint32_t global_int_enable;
    uint32_t int_status;
    uint32_t int_enable;
    uint32_t dynamic_vc_sel;
    uint32_t generic_short_packet;
    uint32_t vcx_frame_error;
    uint32_t clk_lane_info;
    uint32_t lane_info[4];
    uint32_t test_trigger;
    uint32_t debug_ctrl;
    uint32_t force_int;
    
    /* Image info for all VCs */
    uint32_t img_info1[16];  /* VC0-VC15 */
    uint32_t img_info2[16];  /* VC0-VC15 */
    
    /* D-PHY registers */
    uint32_t dphy_control;
    uint32_t dphy_status;
    uint32_t dphy_hs_settle;
    uint32_t dphy_pll_ctrl;
    uint32_t dphy_pll_status;
    
    /* State tracking */
    bool driver_connected;
    bool streaming_active;
    uint32_t init_phase;  /* 0=none, 1=core, 2=interrupts, 3=ready */
} AmdCsi2Registers;

/* Virtual Camera State */
typedef struct {
    bool enabled;
    uint32_t frame_count;
    QEMUTimer *frame_timer;
    bool timer_active;
    uint64_t last_frame_time;
} VirtualCamera;

/* Main device state */
struct AmdCsi2PcieSinkState {
    PCIDevice parent_obj;
    
    MemoryRegion reg_bar;
    MemoryRegion framebuf_bar;
    MemoryRegion msix_bar;
    
    AmdCsi2Registers regs;
    VirtualCamera vcam;
    
    bool device_ready;
    uint64_t frames_generated;
    uint64_t interrupts_sent;
    uint64_t driver_accesses;
    
    /* 🆕 PCI 인터럽트 관련 */
    bool pci_interrupt_configured;
    bool msix_interrupt_enabled;
    
    /* Debug and monitoring */
    bool debug_enabled;
    uint64_t last_debug_time;
};

/* Forward declarations */
static void amd_csi2_trigger_interrupt(AmdCsi2PcieSinkState *s, uint32_t isr_bits);
static void amd_csi2_virtual_camera_frame(void *opaque);
static void amd_csi2_check_streaming_conditions(AmdCsi2PcieSinkState *s);

/* 🎯 FIXED: Enhanced interrupt delivery with proper PCI routing */
static void amd_csi2_trigger_interrupt(AmdCsi2PcieSinkState *s, uint32_t isr_bits)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    
    /* Set interrupt status bits */
    s->regs.int_status |= isr_bits;
    
    /* Check global interrupt enable */
    if (!(s->regs.global_int_enable & 1)) {
        if (s->debug_enabled) {
            printf("AMD CSI2: Global interrupts disabled, not sending\n");
        }
        return;
    }
    
    /* Check enabled interrupts */
    uint32_t pending = s->regs.int_status & s->regs.int_enable;
    if (pending == 0) {
        if (s->debug_enabled) {
            printf("AMD CSI2: No pending enabled interrupts\n");
        }
        return;
    }
    static uint64_t last_log_time = 0;
    uint64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL); 

    /* 🎯 FIXED: Proper MSI-X delivery with validation */
    if (msix_enabled(pci_dev)) {
        if (!msix_is_masked(pci_dev, 0)) {
            msix_notify(pci_dev, 0);
            s->interrupts_sent++;
            s->msix_interrupt_enabled = true;
            if (current_time - last_log_time >= 5000000000ULL || s->interrupts_sent <= 5) {
		    printf("AMD CSI2: MSI-X interrupt sent (ISR=0x%08x, pending=0x%08x) #%lu\n",
		 		    s->regs.int_status, pending, s->interrupts_sent);
		    last_log_time = current_time;
	    }
        } else {
            printf("AMD CSI2: MSI-X vector 0 is masked, interrupt not sent\n");
        }
    } else {
        printf("AMD CSI2: MSI-X not enabled, interrupt not sent\n");
    }
}

/* 🎯 FIXED: Immediate streaming start on proper conditions */
static void amd_csi2_check_streaming_conditions(AmdCsi2PcieSinkState *s)
{
    bool core_enabled = (s->regs.core_config & CORE_CONFIG_ENABLE);
    bool interrupts_enabled = (s->regs.global_int_enable & 1);
    bool frame_int_enabled = (s->regs.int_enable & ISR_FRAME_RECEIVED);
    
    bool should_stream = core_enabled && interrupts_enabled && frame_int_enabled;
    
    if (should_stream && !s->vcam.timer_active) {
        /* Start virtual camera immediately */
        s->vcam.timer_active = true;
        s->regs.streaming_active = true;
        
        /* First frame in 1 second */
        timer_mod(s->vcam.frame_timer, 
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000000ULL);
        
        printf("AMD CSI2: Virtual camera started (streaming=true)\n");
        
        /* Send immediate test interrupt to verify connection */
        amd_csi2_trigger_interrupt(s, ISR_FRAME_RECEIVED);
        
    } else if (!should_stream && s->vcam.timer_active) {
        /* Stop virtual camera */
        timer_del(s->vcam.frame_timer);
        s->vcam.timer_active = false;
        s->regs.streaming_active = false;
        
        printf("AMD CSI2: Virtual camera stopped\n");
    }
}

/* Virtual camera frame generation */
static void amd_csi2_virtual_camera_frame(void *opaque)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    
    if (!s->vcam.timer_active) {
        return;
    }
    
    /* Generate frame */
    s->vcam.frame_count++;
    s->frames_generated++;
    s->vcam.last_frame_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    /* Update hardware registers to reflect frame capture */
    s->regs.core_status = (s->regs.core_status & 0xFFFF) | (s->vcam.frame_count << 16);
    s->regs.img_info1[0] = (s->vcam.frame_count << 16) | 1920;  /* Line count + byte count */
    s->regs.img_info2[0] = 0x2A;  /* RAW10 data type */
    
    /* Trigger frame received interrupt */
    amd_csi2_trigger_interrupt(s, ISR_FRAME_RECEIVED);
    
    if (s->debug_enabled && (s->vcam.frame_count <= 5 || s->vcam.frame_count % 30 == 0)) {
        printf("AMD CSI2: Frame #%u generated, interrupt sent\n", s->vcam.frame_count);
    }
    
    /* Schedule next frame */
    if (s->vcam.timer_active) {
        timer_mod(s->vcam.frame_timer, 
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + AMD_CSI2_FRAME_INTERVAL_NS);
    }
}

/* Register read with complete mapping */
static uint64_t amd_csi2_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    uint32_t val = 0;
    
    s->driver_accesses++;
    
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
    case CSI2_REG_DYNAMIC_VC_SEL:
        val = s->regs.dynamic_vc_sel;
        break;
    case CSI2_REG_GENERIC_SHORT_PACKET:
        val = s->regs.generic_short_packet;
        break;
    case CSI2_REG_VCX_FRAME_ERROR:
        val = s->regs.vcx_frame_error;
        break;
    case CSI2_REG_CLK_LANE_INFO:
        val = s->regs.clk_lane_info;
        break;
    case CSI2_REG_LANE0_INFO:
    case CSI2_REG_LANE1_INFO:
    case CSI2_REG_LANE2_INFO:
    case CSI2_REG_LANE3_INFO:
        val = s->regs.lane_info[(addr - CSI2_REG_LANE0_INFO) / 4];
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
        
    /* Image Information Registers for all VCs */
    case CSI2_REG_IMG_INFO1_VC0 ... (CSI2_REG_IMG_INFO1_VC0 + 0x78):
        if ((addr - CSI2_REG_IMG_INFO1_VC0) % 8 == 0) {
            /* IMG_INFO1 registers */
            int vc = (addr - CSI2_REG_IMG_INFO1_VC0) / 8;
            val = s->regs.img_info1[vc];
        } else {
            /* IMG_INFO2 registers */
            int vc = (addr - CSI2_REG_IMG_INFO2_VC0) / 8;
            val = s->regs.img_info2[vc];
        }
        break;
        
    /* D-PHY registers */
    case CSI2_REG_DPHY_CONTROL:
        val = s->regs.dphy_control;
        break;
    case CSI2_REG_DPHY_STATUS:
        val = s->regs.dphy_status;
        break;
    case CSI2_REG_DPHY_HS_SETTLE:
        val = s->regs.dphy_hs_settle;
        break;
    case CSI2_REG_DPHY_PLL_CTRL:
        val = s->regs.dphy_pll_ctrl;
        break;
    case CSI2_REG_DPHY_PLL_STATUS:
        val = s->regs.dphy_pll_status;
        break;
        
    default:
        val = 0;
        break;
    }
    
    /* Debug important register reads */
    if (s->debug_enabled && (addr <= 0x50 || s->driver_accesses <= 20)) {
        printf("AMD CSI2: Read  0x%04lx = 0x%08x\n", addr, val);
    }
    
    return val;
}

/* 🎯 FIXED: Register write with immediate response */
static void amd_csi2_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    uint32_t old_val;
    
    if (s->debug_enabled && (addr <= 0x50 || s->driver_accesses <= 20)) {
        printf("AMD CSI2: Write 0x%04lx = 0x%08x\n", addr, (uint32_t)val);
    }
    
    switch (addr) {
    case CSI2_REG_CORE_CONFIG:
        old_val = s->regs.core_config;
        s->regs.core_config = val;
        
        if (val & CORE_CONFIG_SOFT_RESET) {
            /* Soft reset */
            s->regs.int_status = 0;
            s->regs.core_config &= ~CORE_CONFIG_SOFT_RESET;
            s->regs.init_phase = 0;
            if (s->vcam.timer_active) {
                timer_del(s->vcam.frame_timer);
                s->vcam.timer_active = false;
            }
            printf("AMD CSI2: Soft reset executed\n");
        }
        
        if ((val & CORE_CONFIG_ENABLE) && !(old_val & CORE_CONFIG_ENABLE)) {
            s->regs.driver_connected = true;
            s->regs.init_phase = 1;
            printf("AMD CSI2: Core enabled - driver connected\n");
        }
        
        amd_csi2_check_streaming_conditions(s);
        break;
        
    case CSI2_REG_PROTOCOL_CONFIG:
        s->regs.protocol_config = val;
        break;
        
    case CSI2_REG_GLOBAL_INT_ENABLE:
        old_val = s->regs.global_int_enable;
        s->regs.global_int_enable = val;
        
        if ((val & 1) && !(old_val & 1)) {
            s->regs.init_phase = 2;
            printf("AMD CSI2: Global interrupts enabled\n");
        }
        
        amd_csi2_check_streaming_conditions(s);
        break;
        
    case CSI2_REG_ISR:
        /* Write-1-to-clear */
        s->regs.int_status &= ~val;
        break;
        
    case CSI2_REG_IER:
        old_val = s->regs.int_enable;
        s->regs.int_enable = val;
        
        if ((val & ISR_FRAME_RECEIVED) && !(old_val & ISR_FRAME_RECEIVED)) {
            s->regs.init_phase = 3;
            printf("AMD CSI2: Frame interrupts enabled - ready for streaming\n");
        }
        
        amd_csi2_check_streaming_conditions(s);
        
        /* Check for pending interrupts after enable change */
        if (s->regs.int_status & val) {
            amd_csi2_trigger_interrupt(s, 0);  /* Re-trigger existing pending */
        }
        break;
        
    case CSI2_REG_DYNAMIC_VC_SEL:
        s->regs.dynamic_vc_sel = val;
        break;
        
    case CSI2_REG_VCX_FRAME_ERROR:
        s->regs.vcx_frame_error = val;
        break;
        
    case CSI2_REG_LANE0_INFO:
    case CSI2_REG_LANE1_INFO:
    case CSI2_REG_LANE2_INFO:
    case CSI2_REG_LANE3_INFO:
        s->regs.lane_info[(addr - CSI2_REG_LANE0_INFO) / 4] = val;
        break;
        
    case CSI2_REG_TEST_TRIGGER:
        s->regs.test_trigger = val;
        printf("AMD CSI2: Test trigger = 0x%08x\n", (uint32_t)val);
        
        /* Immediate test interrupt for any non-zero value */
        if (val != 0) {
            amd_csi2_trigger_interrupt(s, ISR_FRAME_RECEIVED);
        }
        break;
        
    case CSI2_REG_DEBUG_CTRL:
        s->regs.debug_ctrl = val;
        s->debug_enabled = (val & 1) ? true : false;
        printf("AMD CSI2: Debug mode %s\n", s->debug_enabled ? "enabled" : "disabled");
        break;
        
    case CSI2_REG_FORCE_INT:
        s->regs.force_int = val;
        printf("AMD CSI2: Force interrupt = 0x%08x\n", (uint32_t)val);
        
        /* Force interrupt for any non-zero value */
        if (val != 0) {
            amd_csi2_trigger_interrupt(s, ISR_FRAME_RECEIVED);
        }
        break;
        
    /* D-PHY registers */
    case CSI2_REG_DPHY_CONTROL:
        s->regs.dphy_control = val;
        if (val & 1) {
            s->regs.dphy_status = 0x3;  /* Ready */
        }
        break;
        
    case CSI2_REG_DPHY_PLL_CTRL:
        s->regs.dphy_pll_ctrl = val;
        if (val & 1) {
            s->regs.dphy_pll_status = 0x3;  /* Locked */
        }
        break;
        
    case CSI2_REG_DPHY_HS_SETTLE:
        s->regs.dphy_hs_settle = val;
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

/* 🎯 FIXED: Device realization with proper PCI interrupt configuration */
static void amd_csi2_pcie_sink_realize(PCIDevice *pci_dev, Error **errp)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(pci_dev);
    uint8_t *pci_conf = pci_dev->config;
    int ret;
    
    printf("AMD CSI2: PCI INTERRUPT ROUTING FIX v3.1\n");
    
    /* PCI configuration */
    pci_config_set_vendor_id(pci_conf, AMD_CSI2_VENDOR_ID);
    pci_config_set_device_id(pci_conf, AMD_CSI2_DEVICE_ID);
    pci_config_set_revision(pci_conf, AMD_CSI2_REVISION);
    pci_config_set_class(pci_conf, AMD_CSI2_CLASS_CODE);
    
    /* 🎯 CRITICAL FIX: Set PCI interrupt configuration */
    pci_set_word(pci_conf + PCI_COMMAND, 
                 PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_INTX_DISABLE);
    pci_set_word(pci_conf + PCI_STATUS, 
                 PCI_STATUS_CAP_LIST | PCI_STATUS_FAST_BACK);
    
    /* 🆕 CRITICAL: Set interrupt pin (INTA) */
    pci_config_set_interrupt_pin(pci_conf, 1);  /* INTA = 1 */
    printf("AMD CSI2: PCI interrupt pin configured (INTA)\n");
    
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
    
    /* 🎯 FIXED: MSI-X initialization with proper capability offset */
    ret = msix_init(pci_dev, AMD_CSI2_MSIX_VECTORS,
                    &s->msix_bar, 2, AMD_CSI2_MSIX_TABLE_OFFSET,
                    &s->msix_bar, 2, AMD_CSI2_MSIX_PBA_OFFSET, 
                    0x60, errp);
    if (ret < 0) {
        printf("AMD CSI2: MSI-X init failed: %d\n", ret);
        return;
    }
    
    /* 🆕 Mark PCI interrupt as configured */
    s->pci_interrupt_configured = true;
    s->msix_interrupt_enabled = false;
    
    printf("AMD CSI2: MSI-X initialized with %d vectors\n", AMD_CSI2_MSIX_VECTORS);
    
    /* Initialize registers with proper defaults */
    memset(&s->regs, 0, sizeof(s->regs));
    s->regs.core_config = 0;  /* Disabled */
    s->regs.protocol_config = 0x3;  /* 4 lanes */
    s->regs.core_status = 0x1000;  /* Base status */
    s->regs.global_int_enable = 0;  /* Disabled */
    s->regs.int_status = 0;
    s->regs.int_enable = 0;  /* Disabled */
    s->regs.dynamic_vc_sel = 0xFFFF;  /* All VCs enabled */
    s->regs.dphy_status = 0x3;  /* Ready */
    s->regs.dphy_pll_status = 0x3;  /* Locked */
    s->regs.init_phase = 0;
    
    /* Initialize virtual camera */
    s->vcam.enabled = true;
    s->vcam.frame_count = 0;
    s->vcam.timer_active = false;
    s->vcam.last_frame_time = 0;
    s->vcam.frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                      amd_csi2_virtual_camera_frame, s);
    
    /* Initialize device state */
    s->device_ready = true;
    s->frames_generated = 0;
    s->interrupts_sent = 0;
    s->driver_accesses = 0;
    s->debug_enabled = true;  /* Start with debug enabled */
    s->last_debug_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    printf("AMD CSI2: Device ready - PCI interrupt configured, MSI-X vectors: %d\n", 
           AMD_CSI2_MSIX_VECTORS);
}

static void amd_csi2_pcie_sink_exit(PCIDevice *pci_dev)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(pci_dev);
    
    if (s->vcam.timer_active) {
        timer_del(s->vcam.frame_timer);
    }
    
    if (s->vcam.frame_timer) {
        timer_free(s->vcam.frame_timer);
    }
    
    printf("AMD CSI2: Device removed - Frames: %lu, Interrupts: %lu, Accesses: %lu\n",
           s->frames_generated, s->interrupts_sent, s->driver_accesses);
    
    msix_uninit(pci_dev, &s->msix_bar, &s->msix_bar);
}

static void amd_csi2_pcie_sink_instance_init(Object *obj)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(obj);
    
    memset(&s->regs, 0, sizeof(s->regs));
    memset(&s->vcam, 0, sizeof(s->vcam));
    s->device_ready = false;
    s->frames_generated = 0;
    s->interrupts_sent = 0;
    s->driver_accesses = 0;
    s->debug_enabled = false;
    s->last_debug_time = 0;
    s->pci_interrupt_configured = false;
    s->msix_interrupt_enabled = false;
}

/* VMState for migration */
static const VMStateDescription vmstate_amd_csi2_pcie_sink = {
    .name = "amd-csi2-pcie-sink",
    .version_id = 11,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(regs.core_config, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(regs.int_status, AmdCsi2PcieSinkState),
        VMSTATE_BOOL(device_ready, AmdCsi2PcieSinkState),
        VMSTATE_UINT32(vcam.frame_count, AmdCsi2PcieSinkState),
        VMSTATE_UINT64(interrupts_sent, AmdCsi2PcieSinkState),
        VMSTATE_BOOL(pci_interrupt_configured, AmdCsi2PcieSinkState),
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
    dc->desc = "AMD MIPI CSI-2 RX PCIe Sink Device (v3.1 - PCI INTERRUPT ROUTING FIX)";
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
