/*
 * AMD MIPI CSI-2 RX Subsystem PCIe Sink Device for QEMU
 * 
 * This device emulates an AMD MIPI CSI-2 receiver connected via PCIe
 * with integrated virtual camera source for testing purposes.
 * 
 * Features:
 * - Full register space emulation (CSI-2 RX Controller + D-PHY)
 * - MSI-X interrupt support with fixed memory layout
 * - Internal virtual camera source with reliable timer
 * - Multiple video formats and resolutions
 * - Frame buffer management
 * - V4L2-compatible interface for Linux drivers
 * 
 * Version 1.5 - COMPILE FIX: MSI-X capability activation
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
#include "include/hw/misc/amd_csi2_rx_controller_core.h"
#include "include/hw/misc/amd_csi2_dphy_core.h"

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

/* Virtual Camera Configuration - Fixed frame size */
#define AMD_CSI2_VIRT_CAM_WIDTH         1920
#define AMD_CSI2_VIRT_CAM_HEIGHT        1080
#define AMD_CSI2_VIRT_CAM_FPS           30
#define AMD_CSI2_VIRT_CAM_FRAME_SIZE    (AMD_CSI2_VIRT_CAM_WIDTH * AMD_CSI2_VIRT_CAM_HEIGHT * 2)

/* Frame timing (30fps = 33.33ms per frame) */
#define AMD_CSI2_FRAME_INTERVAL_NS      (1000000000ULL / AMD_CSI2_VIRT_CAM_FPS)

/* Logging control - reduce verbose output */
#define AMD_CSI2_LOG_FRAME_INTERVAL     100  /* Log every 100 frames */

/* Register space layout - Using AMD header definitions */
typedef struct {
    /* CSI-2 RX Controller registers (0x0000-0x0FFF) */
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
    uint32_t img_info1_vc[16];
    uint32_t img_info2_vc[16];
    
    /* D-PHY registers (0x1000-0x1FFF) */
    uint32_t dphy_control;
    uint32_t dphy_status;
    uint32_t dphy_init;
    uint32_t dphy_wakeup;
    uint32_t dphy_lane_config;
    uint32_t dphy_lane_status;
    uint32_t dphy_hs_settle;
    uint32_t dphy_hs_timeout;
    uint32_t dphy_esc_timeout;
    uint32_t dphy_clk_lane;
    uint32_t dphy_data_lane[4];
    uint32_t dphy_int_status;
    uint32_t dphy_int_enable;
    uint32_t dphy_error_status;
    uint32_t dphy_pll_ctrl;
    uint32_t dphy_pll_status;
    
    /* Frame buffer control registers */
    uint32_t framebuf_write_ptr;
    uint32_t framebuf_read_ptr;
    uint32_t framebuf_size;
    uint32_t framebuf_control;
} AmdCsi2Registers;

/* Virtual Camera State */
typedef struct {
    bool enabled;
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t data_type;
    uint32_t virtual_channel;
    uint32_t frame_count;
    uint32_t line_count;
    QEMUTimer *frame_timer;
    bool timer_active;
    bool log_enabled;
    uint32_t last_log_frame;
    uint64_t frame_start_time;
} VirtualCamera;

/* Frame Buffer Management */
typedef struct {
    uint8_t *data;
    uint32_t size;
    uint32_t write_ptr;
    uint32_t read_ptr;
    bool full;
} FrameBuffer;

/* Main device state */
struct AmdCsi2PcieSinkState {
    PCIDevice parent_obj;
    
    /* Memory regions - Fixed layout */
    MemoryRegion reg_bar;
    MemoryRegion framebuf_bar;
    MemoryRegion msix_bar;
    
    /* Register state */
    AmdCsi2Registers regs;
    
    /* Virtual camera */
    VirtualCamera vcam;
    
    /* Frame buffer */
    FrameBuffer framebuf;
    
    /* Device state */
    bool initialized;
    uint32_t active_lanes;
    uint32_t line_rate;
    
    /* Interrupt state tracking */
    bool interrupts_enabled;
    bool vsync_active;
    
    /* Statistics */
    uint64_t frames_received;
    uint64_t bytes_received;
    uint64_t errors_detected;
    uint64_t interrupts_sent;
};

/* Forward declarations */
static void amd_csi2_update_interrupts(AmdCsi2PcieSinkState *s);
static void amd_csi2_virtual_camera_frame(void *opaque);
static void amd_csi2_process_dummy_frame_data(AmdCsi2PcieSinkState *s, uint32_t size);
static void amd_csi2_check_virtual_source_start(AmdCsi2PcieSinkState *s);

/* 🔧 FIXED: Enhanced MSI-X interrupt helpers */
static void amd_csi2_send_msi(AmdCsi2PcieSinkState *s, int vector)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    
    /* 🔧 FIX: MSI-X 상태 사전 검증 */
    if (!msix_enabled(pci_dev)) {
        if (s->interrupts_sent < 5) {
            printf("AMD CSI2: ❌ MSI-X not enabled, cannot send interrupt\n");
            printf("AMD CSI2: 🔍 MSI-X cap offset: 0x%02x\n", pci_dev->msix_cap);
            if (pci_dev->msix_cap) {
                uint16_t control = pci_get_word(pci_dev->config + pci_dev->msix_cap + PCI_MSIX_FLAGS);
                printf("AMD CSI2: 🔍 MSI-X control: 0x%04x\n", control);
            }
        }
        return;
    }
    
    /* 🔧 FIX: 벡터 범위 검증 */
    if (vector < 0 || vector >= AMD_CSI2_MSIX_VECTORS) {
        printf("AMD CSI2: ❌ Invalid MSI-X vector: %d\n", vector);
        return;
    }
    
    /* 🔧 FIX: 인터럽트 발생 전 디버그 정보 */
    if (s->interrupts_sent < 5) {
        printf("AMD CSI2: 🔔 Sending MSI-X interrupt #%lu (vector %d)\n", 
               s->interrupts_sent + 1, vector);
        printf("AMD CSI2: 📊 MSI-X enabled: %s, vectors: %d\n",
               msix_enabled(pci_dev) ? "YES" : "NO", 
               pci_dev->msix_entries_nr);
    }
    
    /* 실제 인터럽트 발생 */
    msix_notify(pci_dev, vector);
    s->interrupts_sent++;
    
    /* 🔧 FIX: 첫 번째 인터럽트 성공 로그 */
    if (s->interrupts_sent == 1) {
        printf("AMD CSI2: 🎉 First MSI-X interrupt sent successfully!\n");
    } else if (s->interrupts_sent % 1000 == 0) {
        printf("AMD CSI2: 📈 Milestone: %lu MSI-X interrupts sent\n", s->interrupts_sent);
    }
}

static void amd_csi2_update_interrupts(AmdCsi2PcieSinkState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    
    /* Check if interrupts are properly configured */
    if (!msix_enabled(pci_dev)) {
        return;
    }
    
    if (!(s->regs.global_int_enable & 0x1)) {
        return;
    }
    
    /* Send interrupt if conditions are met */
    if (s->regs.int_status & s->regs.int_enable) {
        amd_csi2_send_msi(s, AMD_CSI2_MSIX_VEC_FRAME_READY);
    }
}

/* 🔧 FIXED: Enhanced Virtual Source management with proper timing */
static void amd_csi2_check_virtual_source_start(AmdCsi2PcieSinkState *s)
{
    bool should_start = false;
    
    /* 🔧 FIX: 시작 조건을 더 명확하게 */
    bool core_enabled = (s->regs.core_config & 0x1);
    bool dphy_enabled = (s->regs.dphy_control & 0x1);
    bool global_int_enabled = (s->regs.global_int_enable & 0x1);
    bool frame_int_enabled = (s->regs.int_enable & (1U << 31));
    
    should_start = s->vcam.enabled && core_enabled && dphy_enabled && 
                   global_int_enabled && frame_int_enabled;
    
    if (should_start && !s->vcam.timer_active) {
        printf("AMD CSI2: ✅ Starting Virtual Source (all conditions met)\n");
        printf("AMD CSI2: 📊 Conditions: core=%d, dphy=%d, global_int=%d, frame_int=%d\n",
               core_enabled, dphy_enabled, global_int_enabled, frame_int_enabled);
        printf("AMD CSI2: 📹 Virtual vsync: %ux%u@%ufps via MSI-X vector %d\n",
               s->vcam.width, s->vcam.height, s->vcam.fps, 
               AMD_CSI2_MSIX_VEC_FRAME_READY);
        
        s->interrupts_enabled = true;
        s->vcam.timer_active = true;
        s->vcam.log_enabled = true;
        s->vcam.last_log_frame = 0;
        s->vsync_active = true;
        
        /* 🔧 FIX: 첫 프레임을 즉시 시작하지 않고 약간의 딜레이 */
        timer_mod(s->vcam.frame_timer, 
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (AMD_CSI2_FRAME_INTERVAL_NS / 10));
                 
    } else if (!should_start && s->vcam.timer_active) {
        printf("AMD CSI2: ⏹️  Stopping Virtual Source\n");
        printf("AMD CSI2: 📊 Reason: core=%d, dphy=%d, global_int=%d, frame_int=%d\n",
               core_enabled, dphy_enabled, global_int_enabled, frame_int_enabled);
        printf("AMD CSI2: 📈 Final stats: %u frames, %lu interrupts\n", 
               s->vcam.frame_count, s->interrupts_sent);
        
        timer_del(s->vcam.frame_timer);
        s->interrupts_enabled = false;
        s->vcam.timer_active = false;
        s->vcam.log_enabled = false;
        s->vsync_active = false;
    }
}

/* Optimized dummy test pattern generation */
static void amd_csi2_generate_dummy_frame_metadata(AmdCsi2PcieSinkState *s)
{
    uint32_t frame_num = s->vcam.frame_count;
    
    /* Log key milestones */
    if (s->vcam.log_enabled && (frame_num % AMD_CSI2_LOG_FRAME_INTERVAL == 0)) {
        printf("AMD CSI2: 🎬 Virtual vsync #%u (MSI-X interrupts: %lu)\n", 
               frame_num, s->interrupts_sent);
        s->vcam.last_log_frame = frame_num;
    }
}

/* 🔧 FIXED: Enhanced Virtual Camera frame generation with reliable timing */
static void amd_csi2_virtual_camera_frame(void *opaque)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(opaque);
    uint32_t frame_size = AMD_CSI2_VIRT_CAM_FRAME_SIZE;
    
    /* Check if virtual camera should continue */
    if (!s->vcam.enabled || !s->vcam.timer_active || !s->interrupts_enabled) {
        s->vcam.timer_active = false;
        s->vsync_active = false;
        return;
    }

    /* 🔧 FIX: 첫 프레임 특별 처리 */
    if (s->vcam.frame_count == 0) {
        s->vcam.frame_start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        printf("AMD CSI2: 🎯 First virtual vsync starting - initializing timing\n");
        printf("AMD CSI2: 🎬 Virtual vsync parameters: %ux%u@%ufps, %u bytes/frame\n",
               s->vcam.width, s->vcam.height, s->vcam.fps, frame_size);
    }
    
    /* Generate dummy frame metadata */
    amd_csi2_generate_dummy_frame_metadata(s);
    
    /* Process dummy frame data (register updates only) */
    amd_csi2_process_dummy_frame_data(s, frame_size);
    
    /* Update statistics and registers */
    s->vcam.frame_count++;
    s->frames_received++;
    s->bytes_received += frame_size;
    
    /* 🔧 FIX: 레지스터 업데이트 순서 최적화 */
    uint32_t packet_count = (s->regs.core_status >> 16) + 1;
    s->regs.core_status = (s->regs.core_status & 0xFFFF) | (packet_count << 16);
    
    /* Update image info registers */
    if (s->vcam.virtual_channel < 16) {
        s->regs.img_info1_vc[s->vcam.virtual_channel] = 
            (s->vcam.height << 16) | (frame_size & 0xFFFF);
        s->regs.img_info2_vc[s->vcam.virtual_channel] = s->vcam.data_type;
    }
    
    /* Update frame buffer write pointer (dummy) */
    s->regs.framebuf_write_ptr = (s->regs.framebuf_write_ptr + frame_size) % s->framebuf.size;
    
    /* 🔧 FIX: 인터럽트 발생 최적화 - 조건 재확인 */
    if (s->interrupts_enabled && 
        (s->regs.global_int_enable & 0x1) && 
        (s->regs.int_enable & (1U << 31))) {
        
        /* Set interrupt status */
        s->regs.int_status |= (1U << 31); /* Frame Received bit */
        
        /* Send interrupt */
        amd_csi2_send_msi(s, AMD_CSI2_MSIX_VEC_FRAME_READY);
        
        /* 🔧 FIX: 첫 번째 및 마일스톤 프레임 로깅 */
        if (s->vcam.frame_count == 1) {
            printf("AMD CSI2: 🎯 First virtual vsync interrupt sent successfully\n");
        }
    } else {
        /* 🔧 FIX: 인터럽트 조건 미충족 시 디버그 */
        if (s->vcam.frame_count <= 5) {
            printf("AMD CSI2: ⚠️  Frame %u: interrupt conditions not met\n", s->vcam.frame_count);
            printf("AMD CSI2: 📊 interrupts_enabled=%d, global_enable=0x%x, int_enable=0x%x\n",
                   s->interrupts_enabled, s->regs.global_int_enable, s->regs.int_enable);
        }
    }
    
    /* 🔧 FIX: 정확한 타이밍으로 다음 프레임 스케줄링 */
    if (s->vcam.timer_active && s->interrupts_enabled) {
        uint64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t next_frame_time = s->vcam.frame_start_time + 
                                  (s->vcam.frame_count * AMD_CSI2_FRAME_INTERVAL_NS);
        
        /* 타이밍 드리프트 방지 */
        if (next_frame_time <= current_time) {
            next_frame_time = current_time + AMD_CSI2_FRAME_INTERVAL_NS;
        }
        
        timer_mod(s->vcam.frame_timer, next_frame_time);
    } else {
        printf("AMD CSI2: ⏹️  Virtual vsync stopped (frame %u)\n", s->vcam.frame_count);
        s->vcam.timer_active = false;
        s->vsync_active = false;
    }
}

/* Dummy frame data processing */
static void amd_csi2_process_dummy_frame_data(AmdCsi2PcieSinkState *s, uint32_t size)
{
    /* Update pointers for simulation */
    uint32_t write_ptr = s->framebuf.write_ptr;
    s->framebuf.write_ptr = (write_ptr + size) % s->framebuf.size;
    
    /* Simulate read pointer advancement */
    if (s->vcam.frame_count % 10 == 0) {
        s->framebuf.read_ptr = (s->framebuf.read_ptr + size) % s->framebuf.size;
        s->regs.framebuf_read_ptr = s->framebuf.read_ptr;
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
    
    /* 🔧 FIX: 중요한 레지스터 쓰기 로깅 */
    if (addr == 0x20 || addr == 0x28 || addr == 0x00 || addr == 0x1000) {
        printf("AMD CSI2: 📝 Critical register write: 0x%lx = 0x%lx\n", addr, val);
    }
    
    if (addr < 0x1000) {
        /* CSI-2 RX Controller registers */
        switch (addr) {
        case 0x00: /* Core Configuration Register */
            s->regs.core_config = val;
            
            if (val & 0x1) {
                s->initialized = true;
                s->regs.core_status &= ~0x1; /* Clear reset in progress */
                printf("AMD CSI2: 🔧 Core enabled\n");
            } else {
                s->initialized = false;
                printf("AMD CSI2: 🔧 Core disabled\n");
            }
            
            if (val & 0x2) {
                /* Soft reset */
                s->regs.int_status = 0;
                s->regs.core_status |= 0x1;
                s->vcam.frame_count = 0;
                s->vcam.line_count = 0;
                s->regs.core_config &= ~0x2;
                printf("AMD CSI2: 🔄 Soft reset performed\n");
            }
            
            /* Check virtual source start */
            amd_csi2_check_virtual_source_start(s);
            break;
            
        case 0x04: /* Protocol Configuration Register */
            s->regs.protocol_config = val;
            s->active_lanes = (val & 0x3) + 1;
            printf("AMD CSI2: 🛤️  Protocol config set, active lanes: %u\n", s->active_lanes);
            break;
            
        case 0x20: /* Global Interrupt Enable Register */
            s->regs.global_int_enable = val;
            printf("AMD CSI2: ⚡ Global interrupt enable: 0x%x (virtual vsync will %s)\n", 
                   (uint32_t)val, (val & 0x1) ? "start" : "stop");
            
            /* CRITICAL: Check virtual source when interrupts are enabled */
            amd_csi2_check_virtual_source_start(s);
            amd_csi2_update_interrupts(s);
            break;
            
        case 0x24: /* Interrupt Status Register */
            /* 🔧 FIX: Write 1 to clear 로그 */
            {
                uint32_t cleared_bits = s->regs.int_status & val;
                s->regs.int_status &= ~val;
                if (cleared_bits) {
                    printf("AMD CSI2: 🧹 Cleared interrupt status bits: 0x%x\n", cleared_bits);
                }
            }
            amd_csi2_update_interrupts(s);
            break;
            
        case 0x28: /* Interrupt Enable Register */
            s->regs.int_enable = val;
            printf("AMD CSI2: 📡 Interrupt enable: 0x%x\n", (uint32_t)val);
            
            /* CRITICAL: Check virtual source when frame interrupts are enabled */
            if (val & (1U << 31)) {
                printf("AMD CSI2: 🎬 Frame received interrupt enabled - virtual vsync ready\n");
            } else {
                printf("AMD CSI2: 🎬 Frame received interrupt disabled\n");
            }
            amd_csi2_check_virtual_source_start(s);
            amd_csi2_update_interrupts(s);
            break;
            
        case 0x2C: /* Dynamic VC Selection Register */
            s->regs.dynamic_vc_sel = val;
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
            
            /* Check virtual source when D-PHY state changes */
            amd_csi2_check_virtual_source_start(s);
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
            s->framebuf.read_ptr = val % s->framebuf.size;
            break;
            
        case 0x2C: /* Frame Buffer Control */
            s->regs.framebuf_control = val;
            if (val & 0x1) {
                printf("AMD CSI2: 💾 Frame buffer enabled\n");
            } else {
                printf("AMD CSI2: 💾 Frame buffer disabled\n");
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

/* 🚨 COMPILE FIX: MSI-X capability 강제 활성화 함수 */
static void amd_csi2_force_enable_msix(PCIDevice *pci_dev)
{
    if (!pci_dev->msix_cap) {
        printf("AMD CSI2: ❌ No MSI-X capability found\n");
        return;
    }
    
    uint8_t *config = pci_dev->config;
    uint16_t control = pci_get_word(config + pci_dev->msix_cap + PCI_MSIX_FLAGS);
    
    printf("AMD CSI2: 🔍 Current MSI-X control: 0x%04x\n", control);
    
    /* Force enable MSI-X */
    control |= PCI_MSIX_FLAGS_ENABLE;
    control &= ~PCI_MSIX_FLAGS_MASKALL;  /* Clear function mask */
    
    pci_set_word(config + pci_dev->msix_cap + PCI_MSIX_FLAGS, control);
    
    printf("AMD CSI2: 🔧 Forced MSI-X control to: 0x%04x\n", control);
    
    /* Verify the change */
    control = pci_get_word(config + pci_dev->msix_cap + PCI_MSIX_FLAGS);
    printf("AMD CSI2: ✅ MSI-X control after force: 0x%04x\n", control);
}

/* 🔧 COMPILE FIX: Enhanced device initialization with fixed MSI-X layout */
static void amd_csi2_pcie_sink_realize(PCIDevice *pci_dev, Error **errp)
{
    AmdCsi2PcieSinkState *s = AMD_CSI2_PCIE_SINK(pci_dev);
    uint8_t *pci_conf = pci_dev->config;
    int ret;
    
    printf("AMD CSI2: 🚀 Initializing PCIe sink device (COMPILE FIX VERSION)\n");
    
    /* 🔧 FIX: Enhanced PCI configuration */
    pci_config_set_vendor_id(pci_conf, AMD_CSI2_VENDOR_ID);
    pci_config_set_device_id(pci_conf, AMD_CSI2_DEVICE_ID);
    pci_config_set_revision(pci_conf, AMD_CSI2_REVISION);
    pci_config_set_class(pci_conf, AMD_CSI2_CLASS_CODE);
    
    /* 🔧 FIX: 인터럽트 핀 설정 (MSI-X 필수) */
    pci_conf[PCI_INTERRUPT_PIN] = 1;  /* INTA# */
    
    /* 🔧 FIX: 캐시라인 사이즈 및 레이턴시 설정 */
    pci_conf[PCI_CACHE_LINE_SIZE] = 0x10;
    pci_conf[PCI_LATENCY_TIMER] = 0x40;
    
    /* Enable memory access first */
    pci_conf[PCI_COMMAND] = PCI_COMMAND_MEMORY;

    /* Initialize memory regions with fixed layout */
    memory_region_init_io(&s->reg_bar, OBJECT(s), &amd_csi2_reg_ops, s,
                          "amd-csi2-regs", AMD_CSI2_REG_BAR_SIZE);
    
    memory_region_init_ram(&s->framebuf_bar, OBJECT(s), "amd-csi2-framebuf",
                          AMD_CSI2_FRAMEBUF_BAR_SIZE, errp);
    if (*errp) {
        return;
    }
    
    /* FIXED: Dedicated MSI-X BAR */
    memory_region_init(&s->msix_bar, OBJECT(s), "amd-csi2-msix", 
                       AMD_CSI2_MSIX_BAR_SIZE);
    
    /* 🔧 FIX: BAR 설정 순서 및 타입 명시 */
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32, &s->reg_bar);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32, &s->framebuf_bar);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32, &s->msix_bar);
    
    printf("AMD CSI2: 📍 BARs registered\n");
    
    /* 🔧 FIX: MSI-X 초기화 전에 디바이스 상태 완전 설정 */
    s->interrupts_enabled = false;
    s->vsync_active = false;
    s->interrupts_sent = 0;
    
    /* 🚨 COMPILE FIX: MSI-X 초기화 */
    ret = msix_init(pci_dev, AMD_CSI2_MSIX_VECTORS,
                    &s->msix_bar, 2, AMD_CSI2_MSIX_TABLE_OFFSET,
                    &s->msix_bar, 2, AMD_CSI2_MSIX_PBA_OFFSET, 
                    0x60, errp);  /* position = 0x60 */
    if (ret < 0) {
        printf("AMD CSI2: ❌ Failed to initialize MSI-X: %d\n", ret);
        error_setg(errp, "MSI-X initialization failed: %d", ret);
        return;
    }
    
    printf("AMD CSI2: ✅ MSI-X initialized with %d vectors in dedicated BAR 2\n", 
           AMD_CSI2_MSIX_VECTORS);
    printf("AMD CSI2: 📍 MSI-X table at BAR2+0x%04x, PBA at BAR2+0x%04x\n",
           AMD_CSI2_MSIX_TABLE_OFFSET, AMD_CSI2_MSIX_PBA_OFFSET);
    
    /* 🚨 COMPILE FIX: MSI-X 강제 활성화 */
    amd_csi2_force_enable_msix(pci_dev);
    
    /* Enable bus master after MSI-X setup */
    pci_conf[PCI_COMMAND] |= PCI_COMMAND_MASTER;
    printf("AMD CSI2: 🚌 Bus master enabled\n");
    
    /* Initialize frame buffer */
    s->framebuf.data = memory_region_get_ram_ptr(&s->framebuf_bar);
    s->framebuf.size = AMD_CSI2_FRAMEBUF_BAR_SIZE;
    s->framebuf.write_ptr = 0;
    s->framebuf.read_ptr = 0;
    s->framebuf.full = false;
    
    /* Initialize virtual camera */
    s->vcam.enabled = true;
    s->vcam.width = AMD_CSI2_VIRT_CAM_WIDTH;
    s->vcam.height = AMD_CSI2_VIRT_CAM_HEIGHT;
    s->vcam.fps = AMD_CSI2_VIRT_CAM_FPS;
    s->vcam.data_type = 0x1E; /* YUYV format */
    s->vcam.virtual_channel = 0;
    s->vcam.frame_count = 0;
    s->vcam.line_count = 0;
    s->vcam.timer_active = false;
    s->vcam.log_enabled = false;
    s->vcam.last_log_frame = 0;
    s->vcam.frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                      amd_csi2_virtual_camera_frame, s);
    
    /* Initialize registers with proper default values */
    s->regs.core_config = 0x0;
    s->regs.protocol_config = 0x3; /* 4 lanes */
    s->regs.dynamic_vc_sel = 0xFFFF;
    s->regs.dphy_hs_settle = 0x20;
    s->regs.framebuf_size = AMD_CSI2_FRAMEBUF_BAR_SIZE;
    s->regs.framebuf_write_ptr = 0;
    s->regs.framebuf_read_ptr = 0;
    s->regs.framebuf_control = 0;
    
    /* Initialize device state */
    s->active_lanes = 4;
    s->line_rate = 1440;
    s->initialized = false;
    
    /* 🚨 COMPILE FIX: 최종 MSI-X 상태 검증 */
    if (msix_enabled(pci_dev)) {
        printf("AMD CSI2: ✅ MSI-X capability enabled successfully\n");
    } else {
        printf("AMD CSI2: ⚠️  MSI-X capability not enabled after init\n");
    }
    
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
        printf("AMD CSI2: 📊 Final statistics: %lu frames, %lu interrupts, %.2f MB\n",
               s->frames_received, s->interrupts_sent,
               (double)(s->bytes_received) / (1024 * 1024));
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
    s->bytes_received = 0;
    s->errors_detected = 0;
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
