// csi2_v4l2_loopback_pcie.c - CSI2 PCIe device with V4L2 loopback integration
#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/cutils.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#define TYPE_CSI2_V4L2_LOOPBACK_PCIE "mipi-csi-camera-v4l2"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2V4L2LoopbackPCIe, CSI2_V4L2_LOOPBACK_PCIE)

#define CSI2_VENDOR_ID    0x1234
#define CSI2_DEVICE_ID    0x5678

// CSI2 Register offsets
#define CSI2_REG_CORE_CONFIG        0x0000
#define CSI2_REG_PROTOCOL_CONFIG    0x0004
#define CSI2_REG_CORE_STATUS        0x0010
#define CSI2_REG_INT_ENABLE         0x0020
#define CSI2_REG_INT_STATUS         0x0024
#define CSI2_REG_IMG_INFO1_VC0      0x0060
#define CSI2_REG_IMG_INFO2_VC0      0x0064

// V4L2 Bridge registers
#define V4L2_REG_STREAMING          0x2000
#define V4L2_REG_FRAMES_CAPTURED    0x2004
#define V4L2_REG_SEQUENCE_NUMBER    0x2008

// Extended control registers
#define CSI2_REG_CONTROL            0x3000
#define CSI2_REG_SEQUENCE_RESET     0x3004
#define CSI2_REG_FRAME_CONFIG       0x3008
#define CSI2_REG_STATUS_EXT         0x300C

// Frame configuration
#define DEFAULT_FRAME_WIDTH   1280
#define DEFAULT_FRAME_HEIGHT  720
#define DEFAULT_FPS           30
#define FRAME_SIZE_RGB24      (DEFAULT_FRAME_WIDTH * DEFAULT_FRAME_HEIGHT * 3)

typedef struct CSI2V4L2LoopbackPCIe {
    PCIDevice parent_obj;
    MemoryRegion mmio;
    
    // CSI2 simulation state
    uint32_t core_config;
    uint32_t protocol_config;
    uint32_t core_status;
    uint32_t int_enable;
    uint32_t int_status;
    uint32_t img_info1_vc0;
    uint32_t img_info2_vc0;
    
    // V4L2 bridge state
    uint32_t v4l2_streaming;
    uint32_t frames_captured;
    uint32_t sequence_number;
    
    // Frame generation
    QEMUTimer *frame_timer;
    bool streaming_active;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t fps;
    
    // V4L2 loopback integration
    int v4l2_fd;
    char *v4l2_device_path;
    bool v4l2_initialized;
    
    // Frame buffer
    uint8_t *frame_buffer;
    size_t frame_buffer_size;
    
    // Thread for V4L2 operations
    QemuThread v4l2_thread;
    bool v4l2_thread_running;
    QemuMutex v4l2_mutex;
    QemuCond v4l2_cond;
    bool frame_ready;
    
} CSI2V4L2LoopbackPCIe;

// Forward declarations
static void csi2_frame_timer_cb(void *opaque);
static void csi2_start_streaming(CSI2V4L2LoopbackPCIe *dev);
static void csi2_stop_streaming(CSI2V4L2LoopbackPCIe *dev);
static int csi2_init_v4l2_loopback(CSI2V4L2LoopbackPCIe *dev);
static void csi2_cleanup_v4l2_loopback(CSI2V4L2LoopbackPCIe *dev);
static void csi2_generate_test_frame(CSI2V4L2LoopbackPCIe *dev);
static void *csi2_v4l2_thread_func(void *arg);

// Find available V4L2 loopback device
static char *find_v4l2_loopback_device(void)
{
    for (int i = 0; i < 64; i++) {
        char device_path[64];
        char driver_path[128];
        char driver_name[64];
        FILE *fp;
        
        snprintf(device_path, sizeof(device_path), "/dev/video%d", i);
        snprintf(driver_path, sizeof(driver_path), "/sys/class/video4linux/video%d/device/driver/module", i);
        
        // Check if device exists
        if (access(device_path, F_OK) != 0) {
            continue;
        }
        
        // Read driver name
        fp = fopen(driver_path, "r");
        if (fp) {
            if (fgets(driver_name, sizeof(driver_name), fp)) {
                if (strstr(driver_name, "v4l2loopback")) {
                    fclose(fp);
                    printf("CSI2-V4L2: Found v4l2loopback device: %s\n", device_path);
                    return g_strdup(device_path);
                }
            }
            fclose(fp);
        }
    }
    
    printf("CSI2-V4L2: No v4l2loopback device found\n");
    return NULL;
}

// Initialize V4L2 loopback device
static int csi2_init_v4l2_loopback(CSI2V4L2LoopbackPCIe *dev)
{
    struct v4l2_format fmt;
    struct v4l2_capability cap;
    
    // Find V4L2 loopback device
    dev->v4l2_device_path = find_v4l2_loopback_device();
    if (!dev->v4l2_device_path) {
        printf("CSI2-V4L2: No V4L2 loopback device available\n");
        return -1;
    }
    
    // Open V4L2 device
    dev->v4l2_fd = open(dev->v4l2_device_path, O_RDWR);
    if (dev->v4l2_fd < 0) {
        printf("CSI2-V4L2: Failed to open %s: %s\n", 
               dev->v4l2_device_path, strerror(errno));
        g_free(dev->v4l2_device_path);
        dev->v4l2_device_path = NULL;
        return -1;
    }
    
    // Query capabilities
    if (ioctl(dev->v4l2_fd, VIDIOC_QUERYCAP, &cap) < 0) {
        printf("CSI2-V4L2: Failed to query capabilities: %s\n", strerror(errno));
        goto error;
    }
    
    printf("CSI2-V4L2: Device capabilities:\n");
    printf("  Driver: %s\n", cap.driver);
    printf("  Card: %s\n", cap.card);
    printf("  Bus info: %s\n", cap.bus_info);
    printf("  Capabilities: 0x%08x\n", cap.capabilities);
    
    // Set format
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = dev->frame_width;
    fmt.fmt.pix.height = dev->frame_height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = dev->frame_width * 3;
    fmt.fmt.pix.sizeimage = dev->frame_width * dev->frame_height * 3;
    fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    
    if (ioctl(dev->v4l2_fd, VIDIOC_S_FMT, &fmt) < 0) {
        printf("CSI2-V4L2: Failed to set format: %s\n", strerror(errno));
        goto error;
    }
    
    printf("CSI2-V4L2: V4L2 format set: %dx%d RGB24\n", 
           fmt.fmt.pix.width, fmt.fmt.pix.height);
    
    dev->v4l2_initialized = true;
    return 0;
    
error:
    close(dev->v4l2_fd);
    dev->v4l2_fd = -1;
    g_free(dev->v4l2_device_path);
    dev->v4l2_device_path = NULL;
    return -1;
}

// Cleanup V4L2 loopback
static void csi2_cleanup_v4l2_loopback(CSI2V4L2LoopbackPCIe *dev)
{
    if (dev->v4l2_fd >= 0) {
        close(dev->v4l2_fd);
        dev->v4l2_fd = -1;
    }
    
    g_free(dev->v4l2_device_path);
    dev->v4l2_device_path = NULL;
    dev->v4l2_initialized = false;
}

// Generate test frame with gradient pattern
static void csi2_generate_test_frame(CSI2V4L2LoopbackPCIe *dev)
{
    if (!dev->frame_buffer) {
        return;
    }
    
    uint8_t *buf = dev->frame_buffer;
    uint32_t frame_counter = dev->sequence_number % 256;
    
    // Generate gradient pattern with animation
    for (uint32_t y = 0; y < dev->frame_height; y++) {
        for (uint32_t x = 0; x < dev->frame_width; x++) {
            uint32_t offset = (y * dev->frame_width + x) * 3;
            
            // Animated gradient pattern
            buf[offset + 0] = (x + frame_counter) % 256;        // R
            buf[offset + 1] = (y + frame_counter) % 256;        // G  
            buf[offset + 2] = (x + y + frame_counter) % 256;    // B
        }
    }
}

// V4L2 thread function
static void *csi2_v4l2_thread_func(void *arg)
{
    CSI2V4L2LoopbackPCIe *dev = (CSI2V4L2LoopbackPCIe *)arg;
    
    while (dev->v4l2_thread_running) {
        qemu_mutex_lock(&dev->v4l2_mutex);
        
        // Wait for frame ready signal or thread stop
        while (!dev->frame_ready && dev->v4l2_thread_running) {
            qemu_cond_wait(&dev->v4l2_cond, &dev->v4l2_mutex);
        }
        
        if (!dev->v4l2_thread_running) {
            qemu_mutex_unlock(&dev->v4l2_mutex);
            break;
        }
        
        if (dev->frame_ready && dev->v4l2_initialized && dev->v4l2_fd >= 0) {
            // Write frame to V4L2 loopback device
            ssize_t written = write(dev->v4l2_fd, dev->frame_buffer, dev->frame_buffer_size);
            if (written != (ssize_t)dev->frame_buffer_size) {
                printf("CSI2-V4L2: Failed to write frame: %s (written: %zd/%zu)\n", 
                       strerror(errno), written, dev->frame_buffer_size);
            } else {
                // Update statistics
                dev->frames_captured++;
                if (dev->frames_captured % 30 == 1) {
                    printf("CSI2-V4L2: Wrote frame %d to %s\n", 
                           dev->frames_captured, dev->v4l2_device_path);
                }
            }
            
            dev->frame_ready = false;
        }
        
        qemu_mutex_unlock(&dev->v4l2_mutex);
    }
    
    return NULL;
}

// Frame timer callback
static void csi2_frame_timer_cb(void *opaque)
{
    CSI2V4L2LoopbackPCIe *dev = CSI2_V4L2_LOOPBACK_PCIE(opaque);
    
    if (!dev->streaming_active) {
        return;
    }
    
    // Update sequence number
    dev->sequence_number++;
    
    // Update core status (packet count in upper 16 bits)
    dev->core_status = (dev->sequence_number << 16) & 0xFFFF0000;
    
    // Update image info
    dev->img_info1_vc0 = (dev->sequence_number << 16) | dev->frame_width;
    dev->img_info2_vc0 = 0x2A; // RAW10 data type
    
    // Generate frame data
    csi2_generate_test_frame(dev);
    
    // Signal V4L2 thread that frame is ready
    qemu_mutex_lock(&dev->v4l2_mutex);
    dev->frame_ready = true;
    qemu_cond_signal(&dev->v4l2_cond);
    qemu_mutex_unlock(&dev->v4l2_mutex);
    
    // Schedule next frame (1000000000 ns / fps)
    uint64_t frame_interval_ns = 1000000000ULL / dev->fps;
    timer_mod(dev->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + frame_interval_ns);
}

// Start streaming
static void csi2_start_streaming(CSI2V4L2LoopbackPCIe *dev)
{
    if (dev->streaming_active) {
        return;
    }
    
    printf("CSI2-V4L2: Starting streaming (%dx%d @ %d fps)\n", 
           dev->frame_width, dev->frame_height, dev->fps);
    
    dev->streaming_active = true;
    dev->v4l2_streaming = 1;
    dev->sequence_number = 0;
    dev->frames_captured = 0;
    
    // Start frame timer
    uint64_t frame_interval_ns = 1000000000ULL / dev->fps;
    timer_mod(dev->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + frame_interval_ns);
}

// Stop streaming
static void csi2_stop_streaming(CSI2V4L2LoopbackPCIe *dev)
{
    if (!dev->streaming_active) {
        return;
    }
    
    printf("CSI2-V4L2: Stopping streaming\n");
    
    dev->streaming_active = false;
    dev->v4l2_streaming = 0;
    
    // Stop frame timer
    timer_del(dev->frame_timer);
}

// MMIO read handler
static uint64_t csi2_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    CSI2V4L2LoopbackPCIe *dev = CSI2_V4L2_LOOPBACK_PCIE(opaque);
    uint64_t ret = 0;
    
    switch (addr) {
    case CSI2_REG_CORE_CONFIG:
        ret = dev->core_config;
        break;
    case CSI2_REG_PROTOCOL_CONFIG:
        ret = dev->protocol_config;
        break;
    case CSI2_REG_CORE_STATUS:
        ret = dev->core_status;
        break;
    case CSI2_REG_INT_ENABLE:
        ret = dev->int_enable;
        break;
    case CSI2_REG_INT_STATUS:
        ret = dev->int_status;
        break;
    case CSI2_REG_IMG_INFO1_VC0:
        ret = dev->img_info1_vc0;
        break;
    case CSI2_REG_IMG_INFO2_VC0:
        ret = dev->img_info2_vc0;
        break;
    case V4L2_REG_STREAMING:
        ret = dev->v4l2_streaming;
        break;
    case V4L2_REG_FRAMES_CAPTURED:
        ret = dev->frames_captured;
        break;
    case V4L2_REG_SEQUENCE_NUMBER:
        ret = dev->sequence_number;
        break;
    case CSI2_REG_CONTROL:
        ret = dev->streaming_active ? 1 : 0;
        break;
    case CSI2_REG_FRAME_CONFIG:
        ret = (dev->frame_width << 16) | dev->frame_height;
        break;
    case CSI2_REG_STATUS_EXT:
        ret = (dev->v4l2_initialized ? 0x01 : 0x00) |
              (dev->v4l2_thread_running ? 0x02 : 0x00) |
              (dev->frame_ready ? 0x04 : 0x00);
        break;
    default:
        printf("CSI2-V4L2: Unknown read from offset 0x%lx\n", addr);
        break;
    }
    
    return ret;
}

// MMIO write handler
static void csi2_mmio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    CSI2V4L2LoopbackPCIe *dev = CSI2_V4L2_LOOPBACK_PCIE(opaque);
    
    switch (addr) {
    case CSI2_REG_CORE_CONFIG:
        dev->core_config = value & 0x7;
        if (value & 1) {
            if (!dev->streaming_active) {
                csi2_start_streaming(dev);
            }
        } else {
            if (dev->streaming_active) {
                csi2_stop_streaming(dev);
            }
        }
        break;
        
    case CSI2_REG_PROTOCOL_CONFIG:
        dev->protocol_config = value;
        break;
        
    case CSI2_REG_INT_ENABLE:
        dev->int_enable = value;
        break;
        
    case CSI2_REG_INT_STATUS:
        dev->int_status &= ~value; // Write 1 to clear
        break;
        
    case V4L2_REG_STREAMING:
        if (value & 1) {
            csi2_start_streaming(dev);
        } else {
            csi2_stop_streaming(dev);
        }
        break;
        
    case CSI2_REG_CONTROL:
        if (value & 1) {
            csi2_start_streaming(dev);
        } else {
            csi2_stop_streaming(dev);
        }
        break;
        
    case CSI2_REG_SEQUENCE_RESET:
        if (value == 0xDEADBEEF) {
            dev->sequence_number = 0;
            dev->frames_captured = 0;
            printf("CSI2-V4L2: Statistics reset\n");
        }
        break;
        
    default:
        printf("CSI2-V4L2: Unknown write to offset 0x%lx, value 0x%lx\n", addr, value);
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

// Device realization
static void csi2_pcie_realize(PCIDevice *pci_dev, Error **errp)
{
    CSI2V4L2LoopbackPCIe *dev = CSI2_V4L2_LOOPBACK_PCIE(pci_dev);
    
    printf("CSI2-V4L2: Initializing MIPI CSI2 with V4L2 Loopback\n");
    
    // Set PCI interrupt pin
    pci_config_set_interrupt_pin(pci_dev->config, 1);
    
    // Initialize MMIO region
    memory_region_init_io(&dev->mmio, OBJECT(dev), &csi2_mmio_ops, dev, 
                          "csi2-v4l2-loopback", 0x10000);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_MEM_TYPE_64, &dev->mmio);
    
    // Initialize device state
    dev->core_config = 1; // Enabled by default
    dev->protocol_config = 0x04; // 4 lanes
    dev->core_status = 0;
    dev->int_enable = 0;
    dev->int_status = 0;
    dev->img_info1_vc0 = 0;
    dev->img_info2_vc0 = 0;
    dev->v4l2_streaming = 0;
    dev->frames_captured = 0;
    dev->sequence_number = 0;
    dev->streaming_active = false;
    
    // Frame configuration
    dev->frame_width = DEFAULT_FRAME_WIDTH;
    dev->frame_height = DEFAULT_FRAME_HEIGHT;
    dev->fps = DEFAULT_FPS;
    dev->frame_buffer_size = FRAME_SIZE_RGB24;
    
    // Allocate frame buffer
    dev->frame_buffer = g_malloc(dev->frame_buffer_size);
    if (!dev->frame_buffer) {
        error_setg(errp, "Failed to allocate frame buffer");
        return;
    }
    
    // Initialize frame timer
    dev->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, csi2_frame_timer_cb, dev);
    
    // Initialize V4L2 loopback
    dev->v4l2_fd = -1;
    dev->v4l2_device_path = NULL;
    dev->v4l2_initialized = false;
    
    // Initialize thread synchronization
    qemu_mutex_init(&dev->v4l2_mutex);
    qemu_cond_init(&dev->v4l2_cond);
    dev->frame_ready = false;
    dev->v4l2_thread_running = false;
    
    // Try to initialize V4L2 loopback
    if (csi2_init_v4l2_loopback(dev) == 0) {
        // Start V4L2 thread
        dev->v4l2_thread_running = true;
        qemu_thread_create(&dev->v4l2_thread, "csi2-v4l2", 
                          csi2_v4l2_thread_func, dev, QEMU_THREAD_JOINABLE);
        
        printf("CSI2-V4L2: Device initialized successfully\n");
        printf("  Frame size: %dx%d @ %d fps\n", dev->frame_width, dev->frame_height, dev->fps);
        printf("  V4L2 device: %s\n", dev->v4l2_device_path);
    } else {
        printf("CSI2-V4L2: V4L2 loopback initialization failed, running in CSI2-only mode\n");
    }
}

// Device cleanup
static void csi2_pcie_exit(PCIDevice *pci_dev)
{
    CSI2V4L2LoopbackPCIe *dev = CSI2_V4L2_LOOPBACK_PCIE(pci_dev);
    
    printf("CSI2-V4L2: Cleaning up device\n");
    
    // Stop streaming
    csi2_stop_streaming(dev);
    
    // Stop V4L2 thread
    if (dev->v4l2_thread_running) {
        qemu_mutex_lock(&dev->v4l2_mutex);
        dev->v4l2_thread_running = false;
        qemu_cond_signal(&dev->v4l2_cond);
        qemu_mutex_unlock(&dev->v4l2_mutex);
        
        qemu_thread_join(&dev->v4l2_thread);
    }
    
    // Cleanup V4L2 loopback
    csi2_cleanup_v4l2_loopback(dev);
    
    // Free resources
    if (dev->frame_timer) {
        timer_free(dev->frame_timer);
        dev->frame_timer = NULL;
    }
    
    g_free(dev->frame_buffer);
    dev->frame_buffer = NULL;
    
    qemu_mutex_destroy(&dev->v4l2_mutex);
    qemu_cond_destroy(&dev->v4l2_cond);
    
    printf("CSI2-V4L2: Device cleanup completed\n");
}

// Device reset
static void csi2_pcie_reset(DeviceState *qdev)
{
    CSI2V4L2LoopbackPCIe *dev = CSI2_V4L2_LOOPBACK_PCIE(qdev);
    
    csi2_stop_streaming(dev);
    
    dev->core_config = 1;
    dev->protocol_config = 0x04;
    dev->core_status = 0;
    dev->int_enable = 0;
    dev->int_status = 0;
    dev->v4l2_streaming = 0;
    dev->frames_captured = 0;
    dev->sequence_number = 0;
    
    printf("CSI2-V4L2: Device reset\n");
}

// Device properties
static Property csi2_pcie_properties[] = {
    DEFINE_PROP_UINT32("frame-width", CSI2V4L2LoopbackPCIe, frame_width, DEFAULT_FRAME_WIDTH),
    DEFINE_PROP_UINT32("frame-height", CSI2V4L2LoopbackPCIe, frame_height, DEFAULT_FRAME_HEIGHT),
    DEFINE_PROP_UINT32("fps", CSI2V4L2LoopbackPCIe, fps, DEFAULT_FPS),
    { }
};

// Class initialization
static void csi2_pcie_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);
    
    pc->realize = csi2_pcie_realize;
    pc->exit = csi2_pcie_exit;
    pc->vendor_id = CSI2_VENDOR_ID;
    pc->device_id = CSI2_DEVICE_ID;
    pc->class_id = 0x0400;  // Multimedia controller
    pc->revision = 0x01;
    
    dc->desc = "MIPI CSI-2 PCIe Device with V4L2 Loopback";
    device_class_set_legacy_reset(dc, csi2_pcie_reset);
    dc->props_ = csi2_pcie_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo csi2_pcie_info = {
    .name = TYPE_CSI2_V4L2_LOOPBACK_PCIE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(CSI2V4L2LoopbackPCIe),
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
