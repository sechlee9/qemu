// hw/misc/csi2_v4l2_simple.c - V4L2loopback 연동 단순 CSI2
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "include/hw/misc/csi2_unified.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#define TYPE_CSI2_V4L2_SIMPLE "csi2-v4l2-simple"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2V4L2Simple, CSI2_V4L2_SIMPLE)

typedef struct CSI2V4L2Simple {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    
    /* 내장된 CSI2 디바이스 */
    CSI2UnifiedDevice *csi2_device;
    
    /* V4L2loopback 인터페이스 */
    char *v4l2_device_path;
    int v4l2_fd;
    bool v4l2_active;
    
    /* 프레임 생성 */
    QEMUTimer *v4l2_timer;
    uint32_t v4l2_frames_written;
    
    /* 설정 */
    uint32_t num_lanes;
    uint32_t line_rate;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_format;
    bool auto_start;
    
} CSI2V4L2Simple;

/* V4L2loopback 헬퍼 함수들 */
static int v4l2_open_device(CSI2V4L2Simple *dev)
{
    if (!dev->v4l2_device_path) {
        dev->v4l2_device_path = g_strdup("/dev/video0");
    }
    
    dev->v4l2_fd = open(dev->v4l2_device_path, O_WRONLY);
    if (dev->v4l2_fd < 0) {
        printf("CSI2-V4L2-Simple: Cannot open %s: %s\n", 
               dev->v4l2_device_path, strerror(errno));
        return -1;
    }
    
    printf("CSI2-V4L2-Simple: Opened %s (fd=%d)\n", 
           dev->v4l2_device_path, dev->v4l2_fd);
    return 0;
}

static int v4l2_setup_format(CSI2V4L2Simple *dev)
{
    struct v4l2_format format;
    
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    format.fmt.pix.width = dev->frame_width;
    format.fmt.pix.height = dev->frame_height;
    format.fmt.pix.pixelformat = dev->frame_format;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    format.fmt.pix.bytesperline = dev->frame_width * 3; // RGB24
    format.fmt.pix.sizeimage = dev->frame_width * dev->frame_height * 3;
    
    if (ioctl(dev->v4l2_fd, VIDIOC_S_FMT, &format) < 0) {
        printf("CSI2-V4L2-Simple: VIDIOC_S_FMT failed: %s\n", strerror(errno));
        return -1;
    }
    
    printf("CSI2-V4L2-Simple: Format set to %dx%d\n",
           format.fmt.pix.width, format.fmt.pix.height);
    return 0;
}

static void v4l2_generate_test_frame(CSI2V4L2Simple *dev, uint8_t *buffer)
{
    uint32_t frame_size = dev->frame_width * dev->frame_height;
    uint8_t phase = (dev->v4l2_frames_written * 4) % 256;
    
    /* RGB 그라데이션 패턴 생성 */
    for (uint32_t y = 0; y < dev->frame_height; y++) {
        for (uint32_t x = 0; x < dev->frame_width; x++) {
            uint32_t offset = (y * dev->frame_width + x) * 3;
            
            /* 시간에 따라 변하는 패턴 */
            buffer[offset + 0] = (phase + x * 255 / dev->frame_width) % 256; /* R */
            buffer[offset + 1] = (phase + y * 255 / dev->frame_height) % 256; /* G */
            buffer[offset + 2] = (phase + (x + y) * 128 / (dev->frame_width + dev->frame_height)) % 256; /* B */
        }
    }
}

static void v4l2_write_frame(CSI2V4L2Simple *dev)
{
    if (dev->v4l2_fd < 0 || !dev->v4l2_active) {
        return;
    }
    
    size_t frame_size = dev->frame_width * dev->frame_height * 3; // RGB24
    uint8_t *frame_buffer = g_malloc(frame_size);
    
    /* 테스트 프레임 생성 */
    v4l2_generate_test_frame(dev, frame_buffer);
    
    /* V4L2loopback에 프레임 쓰기 */
    ssize_t written = write(dev->v4l2_fd, frame_buffer, frame_size);
    if (written != (ssize_t)frame_size) {
        if (errno != EAGAIN) { /* EAGAIN은 정상적인 경우 */
            printf("CSI2-V4L2-Simple: Frame write failed: %s (wrote %zd/%zu)\n", 
                   strerror(errno), written, frame_size);
        }
    } else {
        dev->v4l2_frames_written++;
        
        /* 30 프레임마다 상태 출력 */
        if (dev->v4l2_frames_written % 30 == 0) {
            printf("CSI2-V4L2-Simple: %d frames written to %s\n", 
                   dev->v4l2_frames_written, dev->v4l2_device_path);
        }
    }
    
    g_free(frame_buffer);
}

/* V4L2 프레임 생성 타이머 콜백 */
static void v4l2_frame_timeout(void *opaque)
{
    CSI2V4L2Simple *dev = CSI2_V4L2_SIMPLE(opaque);
    
    /* V4L2로 프레임 출력 */
    v4l2_write_frame(dev);
    
    /* CSI2 디바이스 상태도 업데이트 */
    if (dev->csi2_device && dev->v4l2_active) {
        /* CSI2 통계 업데이트 */
        dev->csi2_device->packet_count++;
        dev->csi2_device->core_status = 
            (dev->csi2_device->packet_count << 16) & 0xFFFF0000;
        
        /* V4L2 브리지 통계 업데이트 */
        dev->csi2_device->v4l2_frames_captured = dev->v4l2_frames_written;
        dev->csi2_device->v4l2_sequence_number = dev->v4l2_frames_written;
        
        /* 인터럽트 생성 */
        dev->csi2_device->int_status |= CSI2_INT_FRAME_RECEIVED;
        if (dev->csi2_device->global_int_enable && 
            (dev->csi2_device->int_status & dev->csi2_device->int_enable)) {
            qemu_set_irq(dev->irq, 1);
            qemu_set_irq(dev->irq, 0); /* Pulse */
        }
    }
    
    /* 다음 프레임 스케줄 (30 FPS) */
    if (dev->v4l2_active) {
        timer_mod(dev->v4l2_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
    }
}

/* MMIO 읽기 - CSI2 디바이스와 V4L2 제어 결합 */
static uint64_t csi2_v4l2_simple_read(void *opaque, hwaddr offset, unsigned size)
{
    CSI2V4L2Simple *dev = CSI2_V4L2_SIMPLE(opaque);
    
    if (offset < 0x4000) {
        /* CSI2 디바이스 레지스터 */
        return dev->csi2_device ? 
               csi2_unified_read(dev->csi2_device, offset, size) : 0;
    } else if (offset >= 0x8000 && offset < 0x9000) {
        /* V4L2 전용 레지스터 */
        switch (offset - 0x8000) {
        case 0x00: /* V4L2 active status */
            return dev->v4l2_active ? 1 : 0;
        case 0x04: /* V4L2 frames written */
            return dev->v4l2_frames_written;
        case 0x08: /* V4L2 FD status */
            return dev->v4l2_fd >= 0 ? 1 : 0;
        case 0x0C: /* V4L2 device path length */
            return dev->v4l2_device_path ? strlen(dev->v4l2_device_path) : 0;
        default:
            break;
        }
    }
    
    qemu_log_mask(LOG_GUEST_ERROR, 
                  "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    return 0;
}

/* MMIO 쓰기 */
static void csi2_v4l2_simple_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CSI2V4L2Simple *dev = CSI2_V4L2_SIMPLE(opaque);
    
    if (offset < 0x4000) {
        /* CSI2 디바이스 레지스터 */
        if (dev->csi2_device) {
            csi2_unified_write(dev->csi2_device, offset, value, size);
        }
    } else if (offset >= 0x8000 && offset < 0x9000) {
        /* V4L2 제어 레지스터 */
        switch (offset - 0x8000) {
        case 0x00: /* V4L2 start/stop */
            if (value & 1) {
                if (!dev->v4l2_active) {
                    /* V4L2 디바이스 열기 및 설정 */
                    if (dev->v4l2_fd < 0) {
                        if (v4l2_open_device(dev) < 0 || v4l2_setup_format(dev) < 0) {
                            printf("CSI2-V4L2-Simple: Failed to setup V4L2\n");
                            break;
                        }
                    }
                    
                    dev->v4l2_active = true;
                    if (dev->csi2_device) {
                        dev->csi2_device->v4l2_streaming = true;
                    }
                    
                    /* 프레임 생성 시작 */
                    timer_mod(dev->v4l2_timer, 
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
                    
                    printf("CSI2-V4L2-Simple: V4L2 streaming STARTED\n");
                }
            } else {
                if (dev->v4l2_active) {
                    dev->v4l2_active = false;
                    if (dev->csi2_device) {
                        dev->csi2_device->v4l2_streaming = false;
                    }
                    timer_del(dev->v4l2_timer);
                    printf("CSI2-V4L2-Simple: V4L2 streaming STOPPED\n");
                }
            }
            break;
        case 0x04: /* Reset counters */
            if (value == 0xDEADBEEF) {
                dev->v4l2_frames_written = 0;
                if (dev->csi2_device) {
                    dev->csi2_device->packet_count = 0;
                    dev->csi2_device->v4l2_frames_captured = 0;
                    dev->csi2_device->v4l2_sequence_number = 0;
                }
                printf("CSI2-V4L2-Simple: Counters reset\n");
            }
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, 
                          "%s: Bad V4L2 offset 0x%"HWADDR_PRIx"\n", __func__, offset);
            break;
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    }
}

static const MemoryRegionOps csi2_v4l2_simple_ops = {
    .read = csi2_v4l2_simple_read,
    .write = csi2_v4l2_simple_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void csi2_v4l2_simple_realize(DeviceState *dev, Error **errp)
{
    CSI2V4L2Simple *s = CSI2_V4L2_SIMPLE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    Error *local_err = NULL;
    
    /* 메모리 영역 및 IRQ 초기화 */
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_v4l2_simple_ops, s, 
                          "csi2-v4l2-simple", 0x10000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    
    /* 내장된 CSI2 디바이스 생성 */
    s->csi2_device = CSI2_UNIFIED_DEVICE(qdev_new(TYPE_CSI2_UNIFIED_DEVICE));
    s->csi2_device->num_lanes = s->num_lanes;
    s->csi2_device->default_line_rate = s->line_rate;
    s->csi2_device->frame_width = s->frame_width;
    s->csi2_device->frame_height = s->frame_height;
    s->csi2_device->pixel_format = s->frame_format;
    
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(s->csi2_device), &local_err)) {
        error_propagate(errp, local_err);
        return;
    }
    
    /* V4L2 타이머 생성 */
    s->v4l2_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, v4l2_frame_timeout, s);
    
    /* 자동 시작 */
    if (s->auto_start) {
        if (v4l2_open_device(s) == 0 && v4l2_setup_format(s) == 0) {
            s->v4l2_active = true;
            s->csi2_device->v4l2_streaming = true;
            timer_mod(s->v4l2_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
            printf("CSI2-V4L2-Simple: Auto-started streaming to %s\n", 
                   s->v4l2_device_path);
        }
    }
    
    printf("CSI2-V4L2-Simple: Device realized (V4L2: %s, %dx%d)\n",
           s->v4l2_device_path ?: "not set", s->frame_width, s->frame_height);
}

static void csi2_v4l2_simple_unrealize(DeviceState *dev)
{
    CSI2V4L2Simple *s = CSI2_V4L2_SIMPLE(dev);
    
    /* V4L2 스트리밍 중지 */
    s->v4l2_active = false;
    if (s->v4l2_timer) {
        timer_free(s->v4l2_timer);
        s->v4l2_timer = NULL;
    }
    
    /* V4L2 디바이스 닫기 */
    if (s->v4l2_fd >= 0) {
        close(s->v4l2_fd);
        s->v4l2_fd = -1;
    }
    
    /* CSI2 디바이스 정리 */
    if (s->csi2_device) {
        object_unref(OBJECT(s->csi2_device));
        s->csi2_device = NULL;
    }
    
    printf("CSI2-V4L2-Simple: Device unrealized\n");
}

static void csi2_v4l2_simple_reset(DeviceState *dev)
{
    CSI2V4L2Simple *s = CSI2_V4L2_SIMPLE(dev);
    
    /* CSI2 디바이스 리셋 */
    if (s->csi2_device) {
        device_cold_reset(DEVICE(s->csi2_device));
    }
    
    /* V4L2 상태 리셋 */
    s->v4l2_active = false;
    s->v4l2_frames_written = 0;
    
    if (s->v4l2_timer) {
        timer_del(s->v4l2_timer);
    }
    
    printf("CSI2-V4L2-Simple: Device reset\n");
}

static Property csi2_v4l2_simple_properties[] = {
    DEFINE_PROP_STRING("v4l2-device", CSI2V4L2Simple, v4l2_device_path),
    DEFINE_PROP_UINT32("num-lanes", CSI2V4L2Simple, num_lanes, 4),
    DEFINE_PROP_UINT32("line-rate", CSI2V4L2Simple, line_rate, 1000),
    DEFINE_PROP_UINT32("frame-width", CSI2V4L2Simple, frame_width, 1280),
    DEFINE_PROP_UINT32("frame-height", CSI2V4L2Simple, frame_height, 720),
    DEFINE_PROP_UINT32("frame-format", CSI2V4L2Simple, frame_format, 0x32424752), /* RGB24 */
    DEFINE_PROP_BOOL("auto-start", CSI2V4L2Simple, auto_start, true),
    {}
};

static void csi2_v4l2_simple_instance_init(Object *obj)
{
    CSI2V4L2Simple *s = CSI2_V4L2_SIMPLE(obj);
    
    /* 기본값 설정 */
    s->v4l2_device_path = NULL;
    s->v4l2_fd = -1;
    s->v4l2_active = false;
    s->num_lanes = 4;
    s->line_rate = 1000;
