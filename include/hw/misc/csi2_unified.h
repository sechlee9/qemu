// include/hw/misc/csi2_unified.h - 통합된 CSI2 헤더
#ifndef HW_CSI2_UNIFIED_H
#define HW_CSI2_UNIFIED_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qapi/error.h"

/* CSI2 상수 정의 */
#define CSI2_MAX_LANES      4
#define CSI2_MAX_VC         16
#define CSI2_FIFO_DEPTH     32

/* 인터럽트 비트 */
#define CSI2_INT_FRAME_RECEIVED           (1 << 31)
#define CSI2_INT_ECC_ERROR                (1 << 10)
#define CSI2_INT_CRC_ERROR                (1 << 9)

/* 디바이스 타입 정의 */
#define TYPE_CSI2_UNIFIED_DEVICE "csi2-unified-device"
#define TYPE_CSI2_PCIE_DEVICE "mipi-csi-camera-x86"

OBJECT_DECLARE_SIMPLE_TYPE(CSI2UnifiedDevice, CSI2_UNIFIED_DEVICE)
OBJECT_DECLARE_SIMPLE_TYPE(CSI2PCIeDevice, CSI2_PCIE_DEVICE)

/* Virtual Channel 구조체 */
typedef struct CSI2VirtualChannel {
    uint32_t frame_number;
    uint32_t line_number;
    uint32_t byte_count;
    uint32_t data_type;
    bool frame_active;
} CSI2VirtualChannel;

/* D-PHY 상태 구조체 */
typedef struct CSI2DPhy {
    uint32_t ctrl;
    uint32_t status;
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t default_line_rate;
    bool pll_locked;
    bool enable_deskew;
} CSI2DPhy;

/* CSI2 통합 디바이스 구조체 */
typedef struct CSI2UnifiedDevice {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;

    /* CSI2 Controller 레지스터 */
    uint32_t core_config;
    uint32_t protocol_config;
    uint32_t core_status;
    uint32_t global_int_enable;
    uint32_t int_status;
    uint32_t int_enable;
    uint32_t dynamic_vc_sel;
    uint32_t generic_short_pkt;
    uint32_t vcx_frame_error;
    uint32_t clock_lane_info;
    uint32_t lane_info[CSI2_MAX_LANES];
    uint32_t img_info1[CSI2_MAX_VC];
    uint32_t img_info2[CSI2_MAX_VC];

    /* D-PHY 상태 */
    CSI2DPhy dphy;

    /* Virtual Channel 상태 */
    CSI2VirtualChannel vc[CSI2_MAX_VC];

    /* V4L2 Bridge 상태 */
    bool v4l2_streaming;
    uint32_t v4l2_sequence_number;
    uint32_t v4l2_frames_captured;
    uint32_t v4l2_frames_dropped;
    uint32_t v4l2_buffer_overruns;

    /* 설정 */
    uint32_t num_lanes;
    uint32_t default_line_rate;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t pixel_format;
    bool vfb_enabled;
    bool embedded_enabled;
    uint32_t pixels_per_clock;

    /* 런타임 상태 */
    uint32_t packet_count;
    uint32_t active_lanes;
    QEMUTimer *frame_timer;

} CSI2UnifiedDevice;

/* PCIe wrapper 구조체 */
typedef struct CSI2PCIeDevice {
    PCIDevice parent_obj;
    MemoryRegion mmio;
    
    /* 내장된 CSI2 디바이스 */
    CSI2UnifiedDevice *csi2_device;
    
    /* PCIe 설정 */
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t frame_width;
    uint32_t frame_height;
    
} CSI2PCIeDevice;

/* 공용 함수 선언 */
uint64_t csi2_unified_read(void *opaque, hwaddr offset, unsigned size);
void csi2_unified_write(void *opaque, hwaddr offset, uint64_t value, unsigned size);

/* 메모리 맵 정의 */
#define CSI2_CONTROLLER_BASE    0x0000  /* 0x0000 - 0x0FFF */
#define CSI2_DPHY_BASE          0x1000  /* 0x1000 - 0x1FFF */
#define CSI2_V4L2_BASE          0x2000  /* 0x2000 - 0x2FFF */
#define CSI2_CONTROL_BASE       0x3000  /* 0x3000 - 0x3FFF */

/* 주요 레지스터 오프셋 */
#define CSI2_CORE_CONFIG        0x00
#define CSI2_PROTOCOL_CONFIG    0x04
#define CSI2_CORE_STATUS        0x10
#define CSI2_GLOBAL_INT_ENABLE  0x20
#define CSI2_INT_STATUS         0x24
#define CSI2_INT_ENABLE         0x28
#define CSI2_DYNAMIC_VC_SEL     0x2C
#define CSI2_GENERIC_SHORT_PKT  0x30
#define CSI2_VCX_FRAME_ERROR    0x34
#define CSI2_CLOCK_LANE_INFO    0x3C
#define CSI2_LANE0_INFO         0x40
#define CSI2_IMG_INFO1_VC0      0x60
#define CSI2_IMG_INFO2_VC0      0x64

/* D-PHY 레지스터 */
#define CSI2_DPHY_CTRL          (CSI2_DPHY_BASE + 0x00)
#define CSI2_DPHY_STATUS        (CSI2_DPHY_BASE + 0x04)
#define CSI2_DPHY_LINE_RATE     (CSI2_DPHY_BASE + 0x08)
#define CSI2_DPHY_NUM_LANES     (CSI2_DPHY_BASE + 0x0C)

/* V4L2 Bridge 레지스터 */
#define CSI2_V4L2_STREAMING     (CSI2_V4L2_BASE + 0x00)
#define CSI2_V4L2_FRAMES_CAP    (CSI2_V4L2_BASE + 0x04)
#define CSI2_V4L2_SEQUENCE      (CSI2_V4L2_BASE + 0x08)
#define CSI2_V4L2_FRAMES_DROP   (CSI2_V4L2_BASE + 0x0C)
#define CSI2_V4L2_BUF_OVERRUN   (CSI2_V4L2_BASE + 0x10)

/* 제어 레지스터 */
#define CSI2_CTRL_ENABLE        (CSI2_CONTROL_BASE + 0x00)
#define CSI2_CTRL_RESET         (CSI2_CONTROL_BASE + 0x04)
#define CSI2_CTRL_FRAME_SIZE    (CSI2_CONTROL_BASE + 0x08)
#define CSI2_CTRL_PIXEL_FORMAT  (CSI2_CONTROL_BASE + 0x0C)

/* 헬퍼 매크로 */
#define CSI2_IS_CONTROLLER_REG(offset) ((offset) < CSI2_DPHY_BASE)
#define CSI2_IS_DPHY_REG(offset) ((offset) >= CSI2_DPHY_BASE && (offset) < CSI2_V4L2_BASE)
#define CSI2_IS_V4L2_REG(offset) ((offset) >= CSI2_V4L2_BASE && (offset) < CSI2_CONTROL_BASE)
#define CSI2_IS_CONTROL_REG(offset) ((offset) >= CSI2_CONTROL_BASE)

#endif /* HW_CSI2_UNIFIED_H */
