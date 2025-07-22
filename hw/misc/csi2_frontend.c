// hw/misc/csi2_frontend.c - Adaptive Backend Selection CSI2 Frontend
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_ids.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "include/hw/misc/csi2_rx_controller.h"
#include "include/hw/misc/csi2_v4l2_bridge.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#define TYPE_CSI2_ADAPTIVE_DEVICE "mipi-csi-camera-x86"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2AdaptiveDevice, CSI2_ADAPTIVE_DEVICE)

typedef enum {
    CSI2_BACKEND_NONE = 0,          /* 백엔드 미결정 */
    CSI2_BACKEND_QEMU_V4L2,         /* QEMU 전용 V4L2 백엔드 */
    CSI2_BACKEND_LOOPBACK,          /* v4l2-loopback 모듈 */
    CSI2_BACKEND_DISABLED           /* 백엔드 비활성화 */
} CSI2BackendType;

typedef struct CSI2AdaptiveDevice {
    PCIDevice parent_obj;
    MemoryRegion mmio;
    
    /* Sub-components */
    CSI2RxController *csi2_controller;
    CSI2V4L2Bridge *v4l2_bridge;
    
    /* Configuration */
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t frame_width;
    uint32_t frame_height;
    char *device_name;
    
    /* Backend detection and selection */
    CSI2BackendType current_backend;
    bool auto_detect_enabled;
    QEMUTimer *detection_timer;
    
    /* QEMU V4L2 Backend (직접 구현) */
    QEMUTimer *qemu_frame_timer;
    uint32_t qemu_frame_sequence;
    bool qemu_backend_active;
    
    /* v4l2-loopback Backend */
    char *loopback_device_path;
    int loopback_fd;
    QEMUTimer *loopback_timer;
    uint32_t loopback_frame_sequence;
    bool loopback_active;
    
    /* Common state */
    bool streaming;
    uint32_t total_frames_generated;
    bool driver_detected;
    
    /* Detection state */
    bool qemu_driver_available;
    bool loopback_driver_available;
    
} CSI2AdaptiveDevice;

/* Forward declarations */
static int csi2_adaptive_start_streaming(CSI2AdaptiveDevice *dev);
static void csi2_adaptive_stop_streaming(CSI2AdaptiveDevice *dev);
static CSI2BackendType csi2_auto_select_backend(CSI2AdaptiveDevice *dev);

/* Driver Detection Functions */
static bool csi2_detect_qemu_driver(void)
{
    /* QEMU 전용 드라이버가 로드되었는지 확인 */
    FILE *fp;
    char line[256];
    bool found = false;
    
    fp = fopen("/proc/modules", "r");
    if (!fp) {
        return false;
    }
    
    while (fgets(line, sizeof(line), fp)) {
        if (strstr(line, "qemu_csi2_vsync") || 
            strstr(line, "qemu-csi2-vsync")) {
            found = true;
            break;
        }
    }
    
    fclose(fp);
    
    if (found) {
        printf("CSI2: Detected QEMU CSI2 V4L2 driver\n");
    }
    
    return found;
}

static bool csi2_detect_loopback_driver(void)
{
    /* v4l2-loopback 드라이버가 로드되었는지 확인 */
    FILE *fp;
    char line[256];
    bool found = false;
    
    fp = fopen("/proc/modules", "r");
    if (!fp) {
        return false;
    }
    
    while (fgets(line, sizeof(line), fp)) {
        if (strstr(line, "v4l2loopback")) {
            found = true;
            break;
        }
    }
    
    fclose(fp);
    
    if (found) {
        printf("CSI2: Detected v4l2-loopback driver\n");
        
        /* 사용 가능한 loopback 디바이스 찾기 */
        for (int i = 4; i < 16; i++) {
            char path[32];
            struct v4l2_capability cap;
            int fd;
            
            snprintf(path, sizeof(path), "/dev/video%d", i);
            fd = open(path, O_RDWR | O_NONBLOCK);
            if (fd < 0) continue;
            
            if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
                if (strstr((char*)cap.driver, "v4l2 loopback") ||
                    strstr((char*)cap.card, "Dummy video device")) {
                    close(fd);
                    printf("CSI2: Found loopback device: %s (%s)\n", path, cap.card);
                    return true;
                }
            }
            close(fd);
        }
    }
    
    return found;
}

static char *csi2_find_loopback_device(void)
{
    /* 사용 가능한 v4l2-loopback 디바이스 경로 반환 */
    for (int i = 4; i < 16; i++) {
        char path[32];
        struct v4l2_capability cap;
        int fd;
        
        snprintf(path, sizeof(path), "/dev/video%d", i);
        fd = open(path, O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;
        
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
            if (strstr((char*)cap.driver, "v4l2 loopback") ||
                strstr((char*)cap.card, "Dummy video device")) {
                close(fd);
                return g_strdup(path);
            }
        }
        close(fd);
    }
    
    return NULL;
}

static CSI2BackendType csi2_auto_select_backend(CSI2AdaptiveDevice *dev)
{
    /* 자동 백엔드 선택 로직 */
    
    /* 1. QEMU 전용 드라이버 우선 확인 */
    if (csi2_detect_qemu_driver()) {
        dev->qemu_driver_available = true;
        printf("CSI2: Selected QEMU V4L2 backend (dedicated driver available)\n");
        return CSI2_BACKEND_QEMU_V4L2;
    }
    
    /* 2. v4l2-loopback 드라이버 확인 */
    if (csi2_detect_loopback_driver()) {
        dev->loopback_driver_available = true;
        g_free(dev->loopback_device_path);
        dev->loopback_device_path = csi2_find_loopback_device();
        
        if (dev->loopback_device_path) {
            printf("CSI2: Selected v4l2-loopback backend (%s)\n", 
                   dev->loopback_device_path);
            return CSI2_BACKEND_LOOPBACK;
        }
    }
    
    /* 3. 드라이버가 없으면 비활성화 */
    printf("CSI2: No compatible V4L2 driver found - backend disabled\n");
    printf("CSI2: Load 'qemu-csi2-vsync' or 'v4l2loopback' module to enable\n");
    return CSI2_BACKEND_DISABLED;
}

/* Backend Detection Timer */
static void csi2_detection_timer_callback(void *opaque)
{
    CSI2AdaptiveDevice *dev = CSI2_ADAPTIVE_DEVICE(opaque);
    CSI2BackendType new_backend;
    bool detection_changed = false;
    
    if (!dev->auto_detect_enabled) {
        return;
    }
    
    /* 드라이버 상태 다시 확인 */
    bool qemu_available = csi2_detect_qemu_driver();
    bool loopback_available = csi2_detect_loopback_driver();
    
    /* 상태 변화 감지 */
    if (qemu_available != dev->qemu_driver_available ||
        loopback_available != dev->loopback_driver_available) {
        detection_changed = true;
        dev->qemu_driver_available = qemu_available;
        dev->loopback_driver_available = loopback_available;
    }
    
    /* 백엔드 재선택 필요한 경우 */
    if (detection_changed || dev->current_backend == CSI2_BACKEND_NONE) {
        new_backend = csi2_auto_select_backend(dev);
        
        if (new_backend != dev->current_backend) {
            printf("CSI2: Backend changed: %d -> %d\n", 
                   dev->current_backend, new_backend);
            
            /* 기존 백엔드 정지 */
            if (dev->streaming) {
                if (dev->qemu_backend_active) {
                    timer_del(dev->qemu_frame_timer);
                    dev->qemu_backend_active = false;
                }
                if (dev->loopback_active) {
                    timer_del(dev->loopback_timer);
                    if (dev->loopback_fd >= 0) {
                        close(dev->loopback_fd);
                        dev->loopback_fd = -1;
                    }
                    dev->loopback_active = false;
                }
            }
            
            dev->current_backend = new_backend;
            
            /* 새 백엔드로 재시작 */
            if (dev->streaming && new_backend != CSI2_BACKEND_DISABLED) {
                csi2_adaptive_start_streaming(dev);
            }
        }
    }
    
    /* 다음 감지 예약 (5초 간격) */
    timer_mod(dev->detection_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 5000000000LL);
}

/* QEMU V4L2 Backend Functions */
static void csi2_qemu_frame_timeout(void *opaque)
{
    CSI2AdaptiveDevice *dev = CSI2_ADAPTIVE_DEVICE(opaque);
    
    if (!dev->qemu_backend_active || !dev->streaming) {
        return;
    }
    
    /* Generate synthetic frame data */
    dev->qemu_frame_sequence++;
    dev->total_frames_generated++;
    
    /* Update CSI2 controller */
    if (dev->csi2_controller) {
        dev->csi2_controller->packet_count = dev->total_frames_generated;
        dev->csi2_controller->core_status = 
            (dev->total_frames_generated << 16) & 0xFFFF0000;
        dev->csi2_controller->int_status |= CSI2_INT_FRAME_RECEIVED;
    }
    
    /* Print status every 30 frames */
    if (dev->qemu_frame_sequence % 30 == 1) {
        printf("CSI2-QEMU: Generated frame %d (total: %d)\n", 
               dev->qemu_frame_sequence, dev->total_frames_generated);
    }
    
    /* Schedule next frame */
    timer_mod(dev->qemu_frame_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
}

static int csi2_start_qemu_backend(CSI2AdaptiveDevice *dev)
{
    if (dev->qemu_backend_active || !dev->qemu_driver_available) {
        return 0;
    }
    
    dev->qemu_frame_sequence = 0;
    dev->qemu_backend_active = true;
    
    timer_mod(dev->qemu_frame_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
    
    printf("CSI2: QEMU V4L2 backend started (dedicated driver mode)\n");
    return 0;
}

/* v4l2-loopback Backend Functions */
static void csi2_loopback_frame_timeout(void *opaque)
{
    CSI2AdaptiveDevice *dev = CSI2_ADAPTIVE_DEVICE(opaque);
    
    if (!dev->loopback_active || !dev->streaming || dev->loopback_fd < 0) {
        return;
    }
    
    /* Generate synthetic RGB24 frame */
    size_t frame_size = dev->frame_width * dev->frame_height * 3;
    uint8_t *frame_data = g_malloc(frame_size);
    
    /* Create test pattern */
    uint8_t color = (dev->loopback_frame_sequence % 256);
    for (size_t i = 0; i < frame_size; i += 3) {
        frame_data[i] = color;           /* R */
        frame_data[i + 1] = 255 - color; /* G */
        frame_data[i + 2] = (color * 2) % 256; /* B */
    }
    
    /* Write to v4l2-loopback device */
    ssize_t written = write(dev->loopback_fd, frame_data, frame_size);
    if (written != frame_size) {
        printf("CSI2-Loopback: Write error: %zd/%zu bytes\n", written, frame_size);
    } else {
        dev->loopback_frame_sequence++;
        dev->total_frames_generated++;
        
        /* Update CSI2 controller */
        if (dev->csi2_controller) {
            dev->csi2_controller->packet_count = dev->total_frames_generated;
            dev->csi2_controller->core_status = 
                (dev->total_frames_generated << 16) & 0xFFFF0000;
            dev->csi2_controller->int_status |= CSI2_INT_FRAME_RECEIVED;
        }
        
        /* Print status every 30 frames */
        if (dev->loopback_frame_sequence % 30 == 1) {
            printf("CSI2-Loopback: Wrote frame %d to %s (total: %d)\n", 
                   dev->loopback_frame_sequence, dev->loopback_device_path,
                   dev->total_frames_generated);
        }
    }
    
    g_free(frame_data);
    
    /* Schedule next frame */
    timer_mod(dev->loopback_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
}

static int csi2_start_loopback_backend(CSI2AdaptiveDevice *dev)
{
    struct v4l2_format format;
    
    if (dev->loopback_active || !dev->loopback_device_path) {
        return 0;
    }
    
    /* Open v4l2-loopback device */
    dev->loopback_fd = open(dev->loopback_device_path, O_RDWR | O_NONBLOCK);
    if (dev->loopback_fd < 0) {
        printf("CSI2: Failed to open %s: %s\n", 
               dev->loopback_device_path, strerror(errno));
        return -1;
    }
    
    /* Set format */
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    format.fmt.pix.width = dev->frame_width;
    format.fmt.pix.height = dev->frame_height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    format.fmt.pix.bytesperline = dev->frame_width * 3;
    format.fmt.pix.sizeimage = dev->frame_width * dev->frame_height * 3;
    
    if (ioctl(dev->loopback_fd, VIDIOC_S_FMT, &format) < 0) {
        printf("CSI2: Failed to set format on %s: %s\n", 
               dev->loopback_device_path, strerror(errno));
        close(dev->loopback_fd);
        dev->loopback_fd = -1;
        return -1;
    }
    
    dev->loopback_frame_sequence = 0;
    dev->loopback_active = true;
    
    timer_mod(dev->loopback_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
    
    printf("CSI2: v4l2-loopback backend started on %s (%dx%d RGB24)\n",
           dev->loopback_device_path, dev->frame_width, dev->frame_height);
    return 0;
}

/* Unified Control Functions */
static int csi2_adaptive_start_streaming(CSI2AdaptiveDevice *dev)
{
    if (dev->streaming) {
        return 0;
    }
    
    /* 백엔드가 결정되지 않았으면 자동 선택 */
    if (dev->current_backend == CSI2_BACKEND_NONE) {
        dev->current_backend = csi2_auto_select_backend(dev);
    }
    
    dev->streaming = true;
    dev->total_frames_generated = 0;
    
    switch (dev->current_backend) {
    case CSI2_BACKEND_QEMU_V4L2:
        return csi2_start_qemu_backend(dev);
        
    case CSI2_BACKEND_LOOPBACK:
        return csi2_start_loopback_backend(dev);
        
    case CSI2_BACKEND_DISABLED:
        printf("CSI2: Backend disabled - no compatible driver found\n");
        return 0;
        
    default:
        printf("CSI2: Unknown backend type: %d\n", dev->current_backend);
        return -1;
    }
}

static void csi2_adaptive_stop_streaming(CSI2AdaptiveDevice *dev)
{
    if (!dev->streaming) {
        return;
    }
    
    /* Stop all backends */
    if (dev->qemu_backend_active) {
        timer_del(dev->qemu_frame_timer);
        dev->qemu_backend_active = false;
    }
    
    if (dev->loopback_active) {
        timer_del(dev->loopback_timer);
        if (dev->loopback_fd >= 0) {
            close(dev->loopback_fd);
            dev->loopback_fd = -1;
        }
        dev->loopback_active = false;
    }
    
    dev->streaming = false;
    
    printf("CSI2: Streaming stopped (total frames: %d)\n", 
           dev->total_frames_generated);
}

/* MMIO Interface */
static uint64_t csi2_adaptive_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    CSI2AdaptiveDevice *dev = CSI2_ADAPTIVE_DEVICE(opaque);
    
    if (addr < 0x1000 && dev->csi2_controller) {
        return csi2_rx_controller_read(dev->csi2_controller, addr, size);
    } else if (addr >= 0x2000 && addr < 0x3000 && dev->v4l2_bridge) {
        return csi2_v4l2_bridge_read(dev->v4l2_bridge, addr - 0x2000, size);
    } else if (addr >= 0x3000 && addr < 0x4000) {
        /* Adaptive control registers */
        switch (addr - 0x3000) {
        case 0x00: /* Current backend type */
            return dev->current_backend;
        case 0x04: /* Streaming status */
            return dev->streaming ? 1 : 0;
        case 0x08: /* Total frames */
            return dev->total_frames_generated;
        case 0x0C: /* Backend status */
            return (dev->qemu_backend_active ? 1 : 0) |
                   (dev->loopback_active ? 2 : 0);
        case 0x10: /* Driver availability */
            return (dev->qemu_driver_available ? 1 : 0) |
                   (dev->loopback_driver_available ? 2 : 0);
        case 0x14: /* Auto detection enabled */
            return dev->auto_detect_enabled ? 1 : 0;
        default:
            break;
        }
    }
    
    return 0;
}

static void csi2_adaptive_mmio_write(void *opaque, hwaddr addr, 
                                    uint64_t value, unsigned size)
{
    CSI2AdaptiveDevice *dev = CSI2_ADAPTIVE_DEVICE(opaque);
    
    if (addr < 0x1000 && dev->csi2_controller) {
        csi2_rx_controller_write(dev->csi2_controller, addr, value, size);
    } else if (addr >= 0x2000 && addr < 0x3000 && dev->v4l2_bridge) {
        csi2_v4l2_bridge_write(dev->v4l2_bridge, addr - 0x2000, value, size);
    } else if (addr >= 0x3000 && addr < 0x4000) {
        /* Adaptive control registers */
        switch (addr - 0x3000) {
        case 0x00: /* Force backend type */
            if (value < CSI2_BACKEND_DISABLED) {
                dev->current_backend = value;
                dev->auto_detect_enabled = false;  /* 수동 설정 시 자동 감지 비활성화 */
                printf("CSI2: Backend manually set to %ld\n", value);
            }
            break;
        case 0x04: /* Start/Stop streaming */
            if (value & 1) {
                csi2_adaptive_start_streaming(dev);
            } else {
                csi2_adaptive_stop_streaming(dev);
            }
            break;
        case 0x08: /* Reset counters */
            if (value == 0xDEADBEEF) {
                dev->total_frames_generated = 0;
                dev->qemu_frame_sequence = 0;
                dev->loopback_frame_sequence = 0;
                printf("CSI2: Statistics reset\n");
            }
            break;
        case 0x0C: /* Force backend detection */
            if (value == 0xDEADBEEF) {
                dev->current_backend = csi2_auto_select_backend(dev);
                printf("CSI2: Backend re-detected: %d\n", dev->current_backend);
            }
            break;
        case 0x14: /* Enable/disable auto detection */
            dev->auto_detect_enabled = (value & 1) != 0;
            printf("CSI2: Auto detection %s\n", 
                   dev->auto_detect_enabled ? "enabled" : "disabled");
            break;
        default:
            break;
        }
    }
}

static const MemoryRegionOps csi2_adaptive_mmio_ops = {
    .read = csi2_adaptive_mmio_read,
    .write = csi2_adaptive_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* Device lifecycle */
static void csi2_adaptive_realize(PCIDevice *pci_dev, Error **errp)
{
    CSI2AdaptiveDevice *dev = CSI2_ADAPTIVE_DEVICE(pci_dev);
    Error *local_err = NULL;
    
    /* PCI configuration */
    pci_config_set_interrupt_pin(pci_dev->config, 1);
    memory_region_init_io(&dev->mmio, OBJECT(dev), &csi2_adaptive_mmio_ops, 
                          dev, "csi2-adaptive", 0x10000);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_MEM_TYPE_64, &dev->mmio);
    
    /* Create sub-components */
    dev->csi2_controller = CSI2_RX_CONTROLLER(qdev_new(TYPE_CSI2_RX_CONTROLLER));
    dev->v4l2_bridge = CSI2_V4L2_BRIDGE(qdev_new(TYPE_CSI2_V4L2_BRIDGE));
    
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(dev->csi2_controller), &local_err) ||
        !sysbus_realize_and_unref(SYS_BUS_DEVICE(dev->v4l2_bridge), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    /* Create timers */
    dev->qemu_frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                         csi2_qemu_frame_timeout, dev);
    dev->loopback_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                       csi2_loopback_frame_timeout, dev);
    dev->detection_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       csi2_detection_timer_callback, dev);
    
    /* Initialize state */
    dev->loopback_fd = -1;
    dev->streaming = false;
    dev->qemu_backend_active = false;
    dev->loopback_active = false;
    dev->total_frames_generated = 0;
    dev->current_backend = CSI2_BACKEND_NONE;
    dev->auto_detect_enabled = true;
    dev->qemu_driver_available = false;
    dev->loopback_driver_available = false;
    
    /* Start backend detection */
    dev->current_backend = csi2_auto_select_backend(dev);
    
    /* Start detection timer */
    timer_mod(dev->detection_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000000LL);
    
    printf("CSI2 Adaptive Device: Initialized (%dx%d, %d lanes, %d Mbps)\n",
           dev->frame_width, dev->frame_height, dev->num_lanes, dev->line_rate);
    printf("CSI2: Initial backend: %s\n", 
           dev->current_backend == CSI2_BACKEND_NONE ? "NONE" :
           dev->current_backend == CSI2_BACKEND_QEMU_V4L2 ? "QEMU_V4L2" :
           dev->current_backend == CSI2_BACKEND_LOOPBACK ? "LOOPBACK" : "DISABLED");
    printf("CSI2: Auto detection: %s\n", 
           dev->auto_detect_enabled ? "enabled" : "disabled");
}

static void csi2_adaptive_exit(PCIDevice *pci_dev)
{
    CSI2AdaptiveDevice *dev = CSI2_ADAPTIVE_DEVICE(pci_dev);
    
    csi2_adaptive_stop_streaming(dev);
    
    if (dev->detection_timer) {
        timer_del(dev->detection_timer);
        timer_free(dev->detection_timer);
    }
    if (dev->qemu_frame_timer) {
        timer_free(dev->qemu_frame_timer);
    }
    if (dev->loopback_timer) {
        timer_free(dev->loopback_timer);
    }
    
    g_free(dev->loopback_device_path);
}

static Property csi2_adaptive_properties[] = {
    DEFINE_PROP_UINT32("num-lanes", CSI2AdaptiveDevice, num_lanes, 4),
    DEFINE_PROP_UINT32("line-rate", CSI2AdaptiveDevice, line_rate, 1440),
    DEFINE_PROP_UINT32("frame-width", CSI2AdaptiveDevice, frame_width, 1280),
    DEFINE_PROP_UINT32("frame-height", CSI2AdaptiveDevice, frame_height, 720),
    DEFINE_PROP_STRING("device-name", CSI2AdaptiveDevice, device_name),
    DEFINE_PROP_BOOL("auto-detect", CSI2AdaptiveDevice, auto_detect_enabled, true),
    {}
};

static void csi2_adaptive_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);
    
    pc->realize = csi2_adaptive_realize;
    pc->exit = csi2_adaptive_exit;
    pc->vendor_id = 0x1234;
    pc->device_id = 0x5678;
    pc->class_id = 0x0400;
    pc->revision = 0x03;
    
    dc->desc = "MIPI CSI-2 Adaptive Camera Device";
    dc->props_ = csi2_adaptive_properties;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo csi2_adaptive_info = {
    .name = TYPE_CSI2_ADAPTIVE_DEVICE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(CSI2AdaptiveDevice),
    .class_init = csi2_adaptive_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void csi2_adaptive_register_types(void)
{
    type_register_static(&csi2_adaptive_info);
}

type_init(csi2_adaptive_register_types)
