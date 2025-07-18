// hw/misc/csi2_v4l2_chardev.c - 실제 /dev/video 장치 생성
#include "qemu/osdep.h"
#include "chardev/char.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/option.h"
#include "sysemu/blockdev.h"
#include "hw/qdev-properties.h"
#include "system/v4l2.h"

#define TYPE_V4L2_CHARDEV "v4l2-chardev"
OBJECT_DECLARE_SIMPLE_TYPE(V4L2CharDev, V4L2_CHARDEV)

typedef struct V4L2CharDev {
    Chardev parent;
    V4L2Backend *backend;
    char *device_name;
    bool opened;
    
    /* V4L2 ioctl emulation */
    struct v4l2_capability cap;
    struct v4l2_format current_format;
    bool streaming;
    
} V4L2CharDev;

/* V4L2 IOCTL 에뮬레이션 */
static int v4l2_chardev_handle_ioctl(V4L2CharDev *chr, unsigned long cmd, void *arg)
{
    switch (cmd) {
    case VIDIOC_QUERYCAP: {
        struct v4l2_capability *cap = (struct v4l2_capability *)arg;
        memset(cap, 0, sizeof(*cap));
        
        g_strlcpy((char*)cap->driver, "qemu-v4l2", sizeof(cap->driver));
        g_strlcpy((char*)cap->card, chr->device_name ?: "QEMU MIPI Camera", sizeof(cap->card));
        g_strlcpy((char*)cap->bus_info, "platform:qemu-csi2", sizeof(cap->bus_info));
        
        cap->version = KERNEL_VERSION(5, 10, 0);
        cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
        cap->device_caps = cap->capabilities;
        
        printf("V4L2 CharDev: VIDIOC_QUERYCAP\n");
        return 0;
    }
    
    case VIDIOC_G_FMT: {
        struct v4l2_format *fmt = (struct v4l2_format *)arg;
        if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            return -EINVAL;
        }
        
        *fmt = chr->current_format;
        printf("V4L2 CharDev: VIDIOC_G_FMT (%dx%d)\n", 
               fmt->fmt.pix.width, fmt->fmt.pix.height);
        return 0;
    }
    
    case VIDIOC_S_FMT: {
        struct v4l2_format *fmt = (struct v4l2_format *)arg;
        if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            return -EINVAL;
        }
        
        /* 기본값 설정 */
        if (fmt->fmt.pix.width == 0) fmt->fmt.pix.width = 1280;
        if (fmt->fmt.pix.height == 0) fmt->fmt.pix.height = 720;
        if (fmt->fmt.pix.pixelformat == 0) fmt->fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
        
        fmt->fmt.pix.bytesperline = fmt->fmt.pix.width * 3; /* RGB24 */
        fmt->fmt.pix.sizeimage = fmt->fmt.pix.bytesperline * fmt->fmt.pix.height;
        fmt->fmt.pix.field = V4L2_FIELD_NONE;
        fmt->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
        
        chr->current_format = *fmt;
        
        printf("V4L2 CharDev: VIDIOC_S_FMT (%dx%d, format=0x%08x)\n", 
               fmt->fmt.pix.width, fmt->fmt.pix.height, fmt->fmt.pix.pixelformat);
        return 0;
    }
    
    case VIDIOC_REQBUFS: {
        struct v4l2_requestbuffers *reqbufs = (struct v4l2_requestbuffers *)arg;
        
        if (reqbufs->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
            reqbufs->memory != V4L2_MEMORY_MMAP) {
            return -EINVAL;
        }
        
        /* 최대 8개 버퍼 지원 */
        if (reqbufs->count > 8) {
            reqbufs->count = 8;
        }
        
        printf("V4L2 CharDev: VIDIOC_REQBUFS (count=%d)\n", reqbufs->count);
        return 0;
    }
    
    case VIDIOC_STREAMON: {
        enum v4l2_buf_type *type = (enum v4l2_buf_type *)arg;
        if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            return -EINVAL;
        }
        
        chr->streaming = true;
        if (chr->backend) {
            Error *local_err = NULL;
            v4l2_backend_start(chr->backend, &local_err);
            if (local_err) {
                error_free(local_err);
                return -EIO;
            }
        }
        
        printf("V4L2 CharDev: VIDIOC_STREAMON\n");
        return 0;
    }
    
    case VIDIOC_STREAMOFF: {
        enum v4l2_buf_type *type = (enum v4l2_buf_type *)arg;
        if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            return -EINVAL;
        }
        
        chr->streaming = false;
        if (chr->backend) {
            v4l2_backend_stop(chr->backend);
        }
        
        printf("V4L2 CharDev: VIDIOC_STREAMOFF\n");
        return 0;
    }
    
    default:
        printf("V4L2 CharDev: Unknown ioctl 0x%08lx\n", cmd);
        return -ENOTTY;
    }
}

/* Character device 읽기 구현 */
static int v4l2_chardev_chr_read(Chardev *chr, uint8_t *buf, int len)
{
    V4L2CharDev *v4l2_chr = V4L2_CHARDEV(chr);
    
    if (!v4l2_chr->streaming) {
        return -EAGAIN;
    }
    
    /* 프레임 데이터 시뮬레이션 */
    static uint8_t frame_counter = 0;
    memset(buf, frame_counter++, len);
    
    return len;
}

/* Character device 쓰기 구현 (ioctl 에뮬레이션) */
static int v4l2_chardev_chr_write(Chardev *chr, const uint8_t *buf, int len)
{
    V4L2CharDev *v4l2_chr = V4L2_CHARDEV(chr);
    
    /* 간단한 ioctl 에뮬레이션을 위한 프로토콜 */
    if (len >= 8) {
        uint32_t cmd = *(uint32_t*)buf;
        void *arg = (void*)(buf + 4);
        
        int result = v4l2_chardev_handle_ioctl(v4l2_chr, cmd, arg);
        return result == 0 ? len : result;
    }
    
    return len;
}

static void v4l2_chardev_open(Chardev *chr, ChardevBackend *backend,
                             bool *be_opened, Error **errp)
{
    V4L2CharDev *v4l2_chr = V4L2_CHARDEV(chr);
    
    v4l2_chr->opened = true;
    
    /* V4L2 백엔드 초기화 */
    v4l2_chr->backend = v4l2_backend_new(v4l2_chr->device_name ?: "QEMU V4L2", errp);
    if (!v4l2_chr->backend) {
        v4l2_chr->opened = false;
        return;
    }
    
    /* 기본 포맷 설정 */
    memset(&v4l2_chr->current_format, 0, sizeof(v4l2_chr->current_format));
    v4l2_chr->current_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_chr->current_format.fmt.pix.width = 1280;
    v4l2_chr->current_format.fmt.pix.height = 720;
    v4l2_chr->current_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    v4l2_chr->current_format.fmt.pix.bytesperline = 1280 * 3;
    v4l2_chr->current_format.fmt.pix.sizeimage = 1280 * 720 * 3;
    v4l2_chr->current_format.fmt.pix.field = V4L2_FIELD_NONE;
    
    printf("V4L2 CharDev: Device opened as %s\n", chr->label ?: chr->filename);
    *be_opened = true;
}

static void v4l2_chardev_close(Chardev *chr)
{
    V4L2CharDev *v4l2_chr = V4L2_CHARDEV(chr);
    
    if (v4l2_chr->backend) {
        if (v4l2_chr->streaming) {
            v4l2_backend_stop(v4l2_chr->backend);
        }
        v4l2_backend_free(v4l2_chr->backend);
        v4l2_chr->backend = NULL;
    }
    
    v4l2_chr->opened = false;
    printf("V4L2 CharDev: Device closed\n");
}

static void v4l2_chardev_instance_init(Object *obj)
{
    V4L2CharDev *v4l2_chr = V4L2_CHARDEV(obj);
    
    v4l2_chr->device_name = NULL;
    v4l2_chr->opened = false;
    v4l2_chr->streaming = false;
    v4l2_chr->backend = NULL;
}

static void v4l2_chardev_finalize(Object *obj)
{
    V4L2CharDev *v4l2_chr = V4L2_CHARDEV(obj);
    
    g_free(v4l2_chr->device_name);
}

static void v4l2_chardev_class_init(ObjectClass *oc, const void *data)
{
    ChardevClass *cc = CHARDEV_CLASS(oc);
    
    cc->open = v4l2_chardev_open;
    cc->chr_close = v4l2_chardev_close;
    cc->chr_read = v4l2_chardev_chr_read;
    cc->chr_write = v4l2_chardev_chr_write;
}

static const TypeInfo v4l2_chardev_info = {
    .name = TYPE_V4L2_CHARDEV,
    .parent = TYPE_CHARDEV,
    .instance_size = sizeof(V4L2CharDev),
    .instance_init = v4l2_chardev_instance_init,
    .instance_finalize = v4l2_chardev_finalize,
    .class_init = v4l2_chardev_class_init,
};

static void v4l2_chardev_register_types(void)
{
    type_register_static(&v4l2_chardev_info);
}

type_init(v4l2_chardev_register_types)
