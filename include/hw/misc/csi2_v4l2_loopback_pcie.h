// hw/misc/CSI2_v4l2_loopback_pcie.h - CSI2 v4l2 loopback PCIe device header
#ifndef HW_MISC_CSI2_V4L2_LOOPBACK_PCIE_H
#define HW_MISC_CSI2_V4L2_LOOPBACK_PCIE_H

#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/pci/msi.h"
#include "qemu/thread.h"
#include "qemu/main-loop.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include <fcntl.h>
#include <unistd.h>

#define TYPE_CSI2_V4L2_LOOPBACK_PCIE "mipi-csi-camera-v4l2"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2V4L2LoopbackPCIeDevice, CSI2_V4L2_LOOPBACK_PCIE)

#define CSI2_VENDOR_ID    0x1234
#define CSI2_DEVICE_ID    0x5679  // Different from original to avoid conflicts

// Register offsets
#define CSI2_REG_CORE_CONFIG        0x0000
#define CSI2_REG_PROTOCOL_CONFIG    0x0004
#define CSI2_REG_CORE_STATUS        0x0010
#define CSI2_REG_INT_ENABLE         0x0020
#define CSI2_REG_INT_STATUS         0x0024
#define CSI2_REG_IMG_INFO1_VC0      0x0060
#define CSI2_REG_IMG_INFO2_VC0      0x0064
#define V4L2_REG_STREAMING          0x2000
#define V4L2_REG_FRAMES_CAPTURED    0x2004
#define V4L2_REG_SEQUENCE_NUMBER    0x2008
#define V4L2_REG_DEVICE_PATH        0x200C
#define V4L2_REG_LOOPBACK_CONTROL   0x2010

// Interrupt bits
#define CSI2_INT_FRAME_RECEIVED     (1 << 31)
#define CSI2_INT_SoT_ERROR          (1 << 13)
#define CSI2_INT_CRC_ERROR          (1 << 9)
#define CSI2_INT_ECC_ERROR          (1 << 10)

// Frame format definitions
#define DEFAULT_FRAME_WIDTH         1280
#define DEFAULT_FRAME_HEIGHT        720
#define DEFAULT_FRAME_RATE          30
#define BYTES_PER_PIXEL             3  // RGB24

// V4L2 device path buffer size
#define V4L2_DEVICE_PATH_MAX        256

typedef struct CSI2Frame {
    uint32_t width;
    uint32_t height;
    uint32_t format;
    uint32_t size;
    uint32_t sequence;
    uint64_t timestamp;
    uint8_t *data;
} CSI2Frame;

typedef struct CSI2V4L2LoopbackPCIeDevice {
    PCIDevice parent_obj;
    
    // MMIO region
    MemoryRegion mmio;
    
    // CSI2 Controller state
    uint32_t core_config;
    uint32_t protocol_config;
    uint32_t core_status;
    uint32_t global_int_enable;
    uint32_t int_status;
    uint32_t int_enable;
    uint32_t packet_count;
    uint32_t active_lanes;
    uint32_t max_lanes;
    uint32_t line_rate;
    
    // Image information registers
    uint32_t img_info1[16];  // VC0-VC15
    uint32_t img_info2[16];  // VC0-VC15
    
    // V4L2 bridge state
    bool streaming;
    uint32_t frames_captured;
    uint32_t sequence_number;
    uint32_t frames_dropped;
    uint32_t buffer_overruns;
    
    // V4L2 loopback integration
    char v4l2_device_path[V4L2_DEVICE_PATH_MAX];
    CharBackend v4l2_chr;
    bool v4l2_loopback_connected;
    
    // Frame generation
    QEMUTimer *frame_timer;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_rate;
    uint32_t num_lanes;
    uint32_t default_line_rate;
    
    // Frame data
    CSI2Frame current_frame;
    QemuMutex frame_mutex;
    
    // Properties
    bool auto_start;
    bool enable_debug;
    char *device_name;
    
    // Statistics
    uint64_t total_frames_generated;
    uint64_t total_bytes_transmitted;
    
} CSI2V4L2LoopbackPCIeDevice;

// Function declarations
void csi2_v4l2_loopback_frame_timer_cb(void *opaque);
uint64_t csi2_v4l2_loopback_mmio_read(void *opaque, hwaddr addr, unsigned size);
void csi2_v4l2_loopback_mmio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size);
void csi2_v4l2_loopback_realize(PCIDevice *pci_dev, Error **errp);
void csi2_v4l2_loopback_exit(PCIDevice *pci_dev);
void csi2_v4l2_loopback_reset(DeviceState *qdev);

// V4L2 loopback specific functions
int csi2_v4l2_loopback_connect(CSI2V4L2LoopbackPCIeDevice *dev, Error **errp);
void csi2_v4l2_loopback_disconnect(CSI2V4L2LoopbackPCIeDevice *dev);
int csi2_v4l2_loopback_send_frame(CSI2V4L2LoopbackPCIeDevice *dev, CSI2Frame *frame);
void csi2_v4l2_loopback_generate_test_frame(CSI2V4L2LoopbackPCIeDevice *dev);

// Frame generation and processing
void csi2_frame_init(CSI2Frame *frame, uint32_t width, uint32_t height);
void csi2_frame_cleanup(CSI2Frame *frame);
void csi2_frame_generate_test_pattern(CSI2Frame *frame, uint32_t sequence);

// Utility functions
static inline uint32_t csi2_calculate_frame_size(uint32_t width, uint32_t height)
{
    return width * height * BYTES_PER_PIXEL;
}

static inline uint32_t csi2_calculate_line_rate(uint32_t width, uint32_t height, uint32_t fps, uint32_t lanes)
{
    // Calculate required line rate in Mb/s for given resolution and fps
    uint64_t bits_per_frame = (uint64_t)width * height * BYTES_PER_PIXEL * 8;
    uint64_t bits_per_second = bits_per_frame * fps;
    return (uint32_t)(bits_per_second / (lanes * 1000000));
}

#endif /* HW_MISC_CSI2_V4L2_LOOPBACK_PCIE_H */
