// vm_v4l2_driver.c - 완전히 수정된 QEMU CSI2 V4L2 드라이버
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#define DRIVER_NAME "qemu_csi2_vm_v4l2"
#define DRIVER_VERSION "1.0.0"

// PCI 디바이스 정보
#define QEMU_CSI2_VENDOR_ID 0x1234
#define QEMU_CSI2_DEVICE_ID 0x5678

// 레지스터 오프셋 (QEMU 디바이스와 매칭)
#define CSI2_CTRL_REG           0x00    // CSI2 Controller base
#define CSI2_STATUS_REG         0x10
#define CSI2_INT_STATUS_REG     0x24
#define CSI2_INT_ENABLE_REG     0x28
#define V4L2_BRIDGE_BASE        0x2000  // V4L2 Bridge base
#define V4L2_STREAMING_REG      (V4L2_BRIDGE_BASE + 0x00)
#define V4L2_FRAMES_REG         (V4L2_BRIDGE_BASE + 0x04)
#define V4L2_SEQUENCE_REG       (V4L2_BRIDGE_BASE + 0x08)

// 기본 설정값
#define DEFAULT_WIDTH           1280
#define DEFAULT_HEIGHT          720
#define DEFAULT_FPS             30
#define MAX_BUFFERS             8

// 디바이스 구조체
struct qemu_csi2_vm_device {
    struct pci_dev *pdev;
    struct v4l2_device v4l2_dev;
    struct video_device vdev;
    struct vb2_queue queue;
    struct mutex mutex;
    
    void __iomem *mmio;
    int irq;
    
    // 현재 포맷
    struct v4l2_pix_format format;
    
    // 스트리밍 상태
    bool streaming;
    spinlock_t slock;
    
    // 프레임 생성용
    struct timer_list frame_timer;
    struct work_struct frame_work;
    u32 sequence;
    
    // 버퍼 리스트
    struct list_head buf_list;
};

// 버퍼 구조체
struct qemu_csi2_vm_buffer {
    struct vb2_v4l2_buffer vb;
    struct list_head list;
};

// 지원하는 픽셀 포맷
static struct v4l2_fmtdesc formats[] = {
    {
        .index = 0,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .flags = 0,
        .description = "RGB24",
        .pixelformat = V4L2_PIX_FMT_RGB24,
    },
    {
        .index = 1,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .flags = 0,
        .description = "YUYV",
        .pixelformat = V4L2_PIX_FMT_YUYV,
    },
};

#define NUM_FORMATS ARRAY_SIZE(formats)

// 전방 선언
static void qemu_csi2_vm_frame_work(struct work_struct *work);
static void qemu_csi2_vm_frame_timer(struct timer_list *t);

// ============================================================================
// MMIO 액세스 함수들
// ============================================================================

static inline u32 qemu_csi2_vm_read(struct qemu_csi2_vm_device *dev, u32 offset)
{
    return ioread32(dev->mmio + offset);
}

static inline void qemu_csi2_vm_write(struct qemu_csi2_vm_device *dev, u32 offset, u32 value)
{
    iowrite32(value, dev->mmio + offset);
}

// ============================================================================
// VideoBuf2 Operations
// ============================================================================

static int qemu_csi2_vm_queue_setup(struct vb2_queue *vq,
                                    unsigned int *nbuffers,
                                    unsigned int *nplanes,
                                    unsigned int sizes[],
                                    struct device *alloc_devs[])
{
    struct qemu_csi2_vm_device *dev = vb2_get_drv_priv(vq);
    
    *nplanes = 1;
    sizes[0] = dev->format.sizeimage;
    
    if (*nbuffers < 2)
        *nbuffers = 4;
    if (*nbuffers > MAX_BUFFERS)
        *nbuffers = MAX_BUFFERS;
    
    pr_info("%s: setup %d buffers, size=%u\n", DRIVER_NAME, *nbuffers, sizes[0]);
    return 0;
}

static int qemu_csi2_vm_buf_prepare(struct vb2_buffer *vb)
{
    struct qemu_csi2_vm_device *dev = vb2_get_drv_priv(vb->vb2_queue);
    unsigned long size = dev->format.sizeimage;
    
    if (vb2_plane_size(vb, 0) < size) {
        pr_err("%s: buffer too small (%lu < %lu)\n", 
               DRIVER_NAME, vb2_plane_size(vb, 0), size);
        return -EINVAL;
    }
    
    vb2_set_plane_payload(vb, 0, size);
    return 0;
}

static void qemu_csi2_vm_buf_queue(struct vb2_buffer *vb)
{
    struct qemu_csi2_vm_device *dev = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct qemu_csi2_vm_buffer *buf = container_of(vbuf, struct qemu_csi2_vm_buffer, vb);
    unsigned long flags;
    
    spin_lock_irqsave(&dev->slock, flags);
    list_add_tail(&buf->list, &dev->buf_list);
    spin_unlock_irqrestore(&dev->slock, flags);
}

static int qemu_csi2_vm_start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct qemu_csi2_vm_device *dev = vb2_get_drv_priv(vq);
    
    dev->sequence = 0;
    dev->streaming = true;
    
    // QEMU 디바이스에 스트리밍 시작 신호
    qemu_csi2_vm_write(dev, V4L2_STREAMING_REG, 1);
    
    // 프레임 타이머 시작 (30 FPS)
    mod_timer(&dev->frame_timer, jiffies + msecs_to_jiffies(1000 / DEFAULT_FPS));
    
    pr_info("%s: streaming started\n", DRIVER_NAME);
    return 0;
}

static void qemu_csi2_vm_stop_streaming(struct vb2_queue *vq)
{
    struct qemu_csi2_vm_device *dev = vb2_get_drv_priv(vq);
    struct qemu_csi2_vm_buffer *buf, *tmp;
    unsigned long flags;
    
    dev->streaming = false;
    
    // QEMU 디바이스에 스트리밍 중지 신호
    qemu_csi2_vm_write(dev, V4L2_STREAMING_REG, 0);
    
    // 타이머 중지 (6.15.4에서 사용 가능한 함수 사용)
    try_to_del_timer_sync(&dev->frame_timer);
    
    // 워크 취소
    cancel_work_sync(&dev->frame_work);
    
    // 남은 버퍼들을 ERROR 상태로 반환
    spin_lock_irqsave(&dev->slock, flags);
    list_for_each_entry_safe(buf, tmp, &dev->buf_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    spin_unlock_irqrestore(&dev->slock, flags);
    
    pr_info("%s: streaming stopped\n", DRIVER_NAME);
}

static const struct vb2_ops qemu_csi2_vm_queue_ops = {
    .queue_setup     = qemu_csi2_vm_queue_setup,
    .buf_prepare     = qemu_csi2_vm_buf_prepare,
    .buf_queue       = qemu_csi2_vm_buf_queue,
    .start_streaming = qemu_csi2_vm_start_streaming,
    .stop_streaming  = qemu_csi2_vm_stop_streaming,
    .wait_prepare    = vb2_ops_wait_prepare,
    .wait_finish     = vb2_ops_wait_finish,
};

// ============================================================================
// 프레임 생성 및 처리
// ============================================================================

static void qemu_csi2_vm_generate_frame(struct qemu_csi2_vm_device *dev,
                                        struct qemu_csi2_vm_buffer *buf)
{
    void *vbuf = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
    u32 size = dev->format.sizeimage;
    u32 width = dev->format.width;
    u32 height = dev->format.height;
    u8 *ptr = (u8 *)vbuf;
    u32 i;
    
    if (!vbuf) {
        pr_err("%s: Failed to get buffer address\n", DRIVER_NAME);
        return;
    }
    
    // 간단한 테스트 패턴 생성 (RGB24)
    if (dev->format.pixelformat == V4L2_PIX_FMT_RGB24) {
        for (i = 0; i < width * height; i++) {
            u32 x = i % width;
            u32 y = i / width;
            
            // 컬러 바 패턴
            u8 r = (x * 255) / width;
            u8 g = (y * 255) / height;
            u8 b = (dev->sequence * 10) % 255;
            
            ptr[i * 3 + 0] = r;
            ptr[i * 3 + 1] = g;
            ptr[i * 3 + 2] = b;
        }
    } else {
        // YUYV나 다른 포맷의 경우 간단한 그레이 패턴
        memset(vbuf, (dev->sequence * 10) % 255, size);
    }
}

static void qemu_csi2_vm_frame_work(struct work_struct *work)
{
    struct qemu_csi2_vm_device *dev = container_of(work, struct qemu_csi2_vm_device, frame_work);
    struct qemu_csi2_vm_buffer *buf;
    unsigned long flags;
    
    if (!dev->streaming)
        return;
    
    // 사용 가능한 버퍼 가져오기
    spin_lock_irqsave(&dev->slock, flags);
    if (list_empty(&dev->buf_list)) {
        spin_unlock_irqrestore(&dev->slock, flags);
        return;
    }
    
    buf = list_first_entry(&dev->buf_list, struct qemu_csi2_vm_buffer, list);
    list_del(&buf->list);
    spin_unlock_irqrestore(&dev->slock, flags);
    
    // 프레임 생성
    qemu_csi2_vm_generate_frame(dev, buf);
    
    // 버퍼 메타데이터 설정
    buf->vb.vb2_buf.timestamp = ktime_get_ns();
    buf->vb.sequence = dev->sequence++;
    buf->vb.field = V4L2_FIELD_NONE;
    
    // 버퍼 완료 처리
    vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
    
    // 디버그 정보 (100프레임마다)
    if (dev->sequence % 100 == 1) {
        pr_info("%s: generated frame %u\n", DRIVER_NAME, dev->sequence - 1);
    }
}

static void qemu_csi2_vm_frame_timer(struct timer_list *t)
{
    struct qemu_csi2_vm_device *dev = from_timer(dev, t, frame_timer);
    
    if (dev->streaming) {
        schedule_work(&dev->frame_work);
        mod_timer(&dev->frame_timer, jiffies + msecs_to_jiffies(1000 / DEFAULT_FPS));
    }
}

// ============================================================================
// V4L2 IOCTL Operations
// ============================================================================

static int qemu_csi2_vm_querycap(struct file *file, void *priv,
                                 struct v4l2_capability *cap)
{
    struct qemu_csi2_vm_device *dev = video_drvdata(file);
    
    strscpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
    strscpy(cap->card, "QEMU CSI2 Virtual Camera", sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", pci_name(dev->pdev));
    
    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
    
    return 0;
}

static int qemu_csi2_vm_enum_fmt_vid_cap(struct file *file, void *priv,
                                        struct v4l2_fmtdesc *f)
{
    if (f->index >= NUM_FORMATS)
        return -EINVAL;
    
    *f = formats[f->index];
    return 0;
}

static int qemu_csi2_vm_g_fmt_vid_cap(struct file *file, void *priv,
                                     struct v4l2_format *f)
{
    struct qemu_csi2_vm_device *dev = video_drvdata(file);
    
    f->fmt.pix = dev->format;
    return 0;
}

static int qemu_csi2_vm_try_fmt_vid_cap(struct file *file, void *priv,
                                       struct v4l2_format *f)
{
    struct v4l2_pix_format *pix = &f->fmt.pix;
    int i;
    
    // 지원하는 포맷인지 확인
    for (i = 0; i < NUM_FORMATS; i++) {
        if (formats[i].pixelformat == pix->pixelformat)
            break;
    }
    
    if (i >= NUM_FORMATS) {
        pix->pixelformat = V4L2_PIX_FMT_RGB24;
    }
    
    // 해상도 제한
    if (pix->width < 160)
        pix->width = 160;
    if (pix->width > 1920)
        pix->width = 1920;
    if (pix->height < 120)
        pix->height = 120;
    if (pix->height > 1080)
        pix->height = 1080;
    
    // 정렬
    pix->width &= ~0x1;
    pix->height &= ~0x1;
    
    // 기타 필드 설정
    pix->field = V4L2_FIELD_NONE;
    pix->colorspace = V4L2_COLORSPACE_SRGB;
    
    if (pix->pixelformat == V4L2_PIX_FMT_RGB24) {
        pix->bytesperline = pix->width * 3;
    } else if (pix->pixelformat == V4L2_PIX_FMT_YUYV) {
        pix->bytesperline = pix->width * 2;
    }
    
    pix->sizeimage = pix->bytesperline * pix->height;
    
    return 0;
}

static int qemu_csi2_vm_s_fmt_vid_cap(struct file *file, void *priv,
                                     struct v4l2_format *f)
{
    struct qemu_csi2_vm_device *dev = video_drvdata(file);
    int ret;
    
    if (vb2_is_busy(&dev->queue))
        return -EBUSY;
    
    ret = qemu_csi2_vm_try_fmt_vid_cap(file, priv, f);
    if (ret)
        return ret;
    
    dev->format = f->fmt.pix;
    
    pr_info("%s: format set to %dx%d, %c%c%c%c\n", DRIVER_NAME,
            dev->format.width, dev->format.height,
            (dev->format.pixelformat >>  0) & 0xff,
            (dev->format.pixelformat >>  8) & 0xff,
            (dev->format.pixelformat >> 16) & 0xff,
            (dev->format.pixelformat >> 24) & 0xff);
    
    return 0;
}

static const struct v4l2_ioctl_ops qemu_csi2_vm_ioctl_ops = {
    .vidioc_querycap          = qemu_csi2_vm_querycap,
    .vidioc_enum_fmt_vid_cap  = qemu_csi2_vm_enum_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap     = qemu_csi2_vm_g_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap   = qemu_csi2_vm_try_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap     = qemu_csi2_vm_s_fmt_vid_cap,
    
    .vidioc_reqbufs           = vb2_ioctl_reqbufs,
    .vidioc_querybuf          = vb2_ioctl_querybuf,
    .vidioc_qbuf              = vb2_ioctl_qbuf,
    .vidioc_dqbuf             = vb2_ioctl_dqbuf,
    .vidioc_streamon          = vb2_ioctl_streamon,
    .vidioc_streamoff         = vb2_ioctl_streamoff,
};

// ============================================================================
// File Operations
// ============================================================================

static int qemu_csi2_vm_open(struct file *file)
{
    struct qemu_csi2_vm_device *dev = video_drvdata(file);
    int ret;
    
    // mutex 잠금 제거 - vb2가 자체적으로 관리
    ret = v4l2_fh_open(file);
    if (ret)
        return ret;
    
    pr_info("%s: device opened\n", DRIVER_NAME);
    return ret;
}

static int qemu_csi2_vm_release(struct file *file)
{
    struct qemu_csi2_vm_device *dev = video_drvdata(file);
    int ret;
    
    // mutex 잠금 제거 - vb2가 자체적으로 관리  
    ret = vb2_fop_release(file);
    
    pr_info("%s: device released\n", DRIVER_NAME);
    return ret;
}

static const struct v4l2_file_operations qemu_csi2_vm_fops = {
    .owner          = THIS_MODULE,
    .open           = qemu_csi2_vm_open,
    .release        = qemu_csi2_vm_release,
    .read           = vb2_fop_read,
    .poll           = vb2_fop_poll,
    .mmap           = vb2_fop_mmap,
    .unlocked_ioctl = video_ioctl2,
};

// ============================================================================
// Video Device Operations
// ============================================================================

static void qemu_csi2_vm_video_release(struct video_device *vdev)
{
    pr_info("%s: video device released\n", DRIVER_NAME);
    
    // 추가 정리 작업이 필요하면 여기서 수행
    // dev 변수가 필요하지 않으므로 제거
}

// ============================================================================
// PCI Driver Operations
// ============================================================================

static int qemu_csi2_vm_queue_init(struct qemu_csi2_vm_device *dev)
{
    struct vb2_queue *q = &dev->queue;
    int ret;
    
    memset(q, 0, sizeof(*q));
    q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
    q->drv_priv = dev;
    q->buf_struct_size = sizeof(struct qemu_csi2_vm_buffer);
    q->ops = &qemu_csi2_vm_queue_ops;
    q->mem_ops = &vb2_vmalloc_memops;
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    q->lock = &dev->mutex;
    q->dev = &dev->pdev->dev;
    
    ret = vb2_queue_init(q);
    if (ret) {
        pr_err("%s: Failed to initialize vb2 queue: %d\n", DRIVER_NAME, ret);
        return ret;
    }
    
    return 0;
}

static int qemu_csi2_vm_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct qemu_csi2_vm_device *dev;
    struct video_device *vdev;
    int ret;
    
    pr_info("%s: Probing QEMU CSI2 device %s\n", DRIVER_NAME, pci_name(pdev));
    
    // 디바이스 구조체 할당
    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;
    
    dev->pdev = pdev;
    
    // PCI 리소스 활성화
    ret = pcim_enable_device(pdev);
    if (ret) {
        pr_err("%s: Failed to enable PCI device: %d\n", DRIVER_NAME, ret);
        return ret;
    }
    
    pci_set_master(pdev);
    
    // MMIO 매핑
    ret = pcim_iomap_regions(pdev, BIT(0), DRIVER_NAME);
    if (ret) {
        pr_err("%s: Failed to map PCI regions: %d\n", DRIVER_NAME, ret);
        return ret;
    }
    
    dev->mmio = pcim_iomap_table(pdev)[0];
    if (!dev->mmio) {
        pr_err("%s: Failed to get MMIO address\n", DRIVER_NAME);
        return -ENOMEM;
    }
    
    pr_info("%s: MMIO mapped at %p\n", DRIVER_NAME, dev->mmio);
    
    // 인터럽트 설정 (옵션)
    dev->irq = pdev->irq;
    
    // 동기화 객체 초기화
    mutex_init(&dev->mutex);
    spin_lock_init(&dev->slock);
    INIT_LIST_HEAD(&dev->buf_list);
    
    // 타이머 및 워크 초기화
    timer_setup(&dev->frame_timer, qemu_csi2_vm_frame_timer, 0);
    INIT_WORK(&dev->frame_work, qemu_csi2_vm_frame_work);
    
    // 기본 포맷 설정
    dev->format.width = DEFAULT_WIDTH;
    dev->format.height = DEFAULT_HEIGHT;
    dev->format.pixelformat = V4L2_PIX_FMT_RGB24;
    dev->format.field = V4L2_FIELD_NONE;
    dev->format.bytesperline = DEFAULT_WIDTH * 3;
    dev->format.sizeimage = DEFAULT_WIDTH * DEFAULT_HEIGHT * 3;
    dev->format.colorspace = V4L2_COLORSPACE_SRGB;
    
    // V4L2 디바이스 등록
    ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
    if (ret) {
        pr_err("%s: Failed to register v4l2 device: %d\n", DRIVER_NAME, ret);
        goto err_v4l2;
    }
    
    // VideoBuf2 큐 초기화
    ret = qemu_csi2_vm_queue_init(dev);
    if (ret)
        goto err_queue;
    
    // Video device 설정 (lock을 제거하여 vb2와의 충돌 방지)
    vdev = &dev->vdev;
    *vdev = (struct video_device) {
        .name = "QEMU CSI2 Virtual Camera",
        .vfl_type = VFL_TYPE_VIDEO,
        .vfl_dir = VFL_DIR_RX,
        .minor = -1,
        .release = qemu_csi2_vm_video_release,
        .fops = &qemu_csi2_vm_fops,
        .ioctl_ops = &qemu_csi2_vm_ioctl_ops,
        .device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
        .queue = &dev->queue,
        .v4l2_dev = &dev->v4l2_dev,
    };
    
    video_set_drvdata(vdev, dev);
    
    // Video device 등록
    ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
    if (ret) {
        pr_err("%s: Failed to register video device: %d\n", DRIVER_NAME, ret);
        goto err_video;
    }
    
    // PCI driver data 설정
    pci_set_drvdata(pdev, dev);
    
    pr_info("%s: Successfully registered as %s\n", DRIVER_NAME, video_device_node_name(vdev));
    
    // QEMU 디바이스 초기화 신호
    qemu_csi2_vm_write(dev, CSI2_CTRL_REG, 1); // Enable CSI2 controller
    
    return 0;
    
err_video:
    vb2_queue_release(&dev->queue);
err_queue:
    v4l2_device_unregister(&dev->v4l2_dev);
err_v4l2:
    mutex_destroy(&dev->mutex);
    return ret;
}

static void qemu_csi2_vm_remove(struct pci_dev *pdev)
{
    struct qemu_csi2_vm_device *dev = pci_get_drvdata(pdev);
    
    if (!dev)
        return;
    
    pr_info("%s: Removing device\n", DRIVER_NAME);
    
    // 스트리밍 중지
    if (dev->streaming) {
        dev->streaming = false;
        qemu_csi2_vm_write(dev, V4L2_STREAMING_REG, 0);
    }
    
    // 타이머 및 워크 정리
    try_to_del_timer_sync(&dev->frame_timer);
    cancel_work_sync(&dev->frame_work);
    
    // Video device 해제
    video_unregister_device(&dev->vdev);
    vb2_queue_release(&dev->queue);
    v4l2_device_unregister(&dev->v4l2_dev);
    
    // 동기화 객체 정리
    mutex_destroy(&dev->mutex);
    
    pci_set_drvdata(pdev, NULL);
    
    pr_info("%s: Device removed\n", DRIVER_NAME);
}

// PCI 디바이스 ID 테이블
static const struct pci_device_id qemu_csi2_vm_pci_ids[] = {
    { PCI_DEVICE(QEMU_CSI2_VENDOR_ID, QEMU_CSI2_DEVICE_ID) },
    { 0, }
};

MODULE_DEVICE_TABLE(pci, qemu_csi2_vm_pci_ids);

// PCI 드라이버 구조체
static struct pci_driver qemu_csi2_vm_pci_driver = {
    .name = DRIVER_NAME,
    .id_table = qemu_csi2_vm_pci_ids,
    .probe = qemu_csi2_vm_probe,
    .remove = qemu_csi2_vm_remove,
};

// ============================================================================
// 모듈 초기화 및 정리
// ============================================================================

static int __init qemu_csi2_vm_pci_driver_init(void)
{
    int ret;
    
    pr_info("%s: Loading QEMU CSI2 V4L2 driver v%s\n", DRIVER_NAME, DRIVER_VERSION);
    
    ret = pci_register_driver(&qemu_csi2_vm_pci_driver);
    if (ret) {
        pr_err("%s: Failed to register PCI driver: %d\n", DRIVER_NAME, ret);
        return ret;
    }
    
    pr_info("%s: Driver loaded successfully\n", DRIVER_NAME);
    return 0;
}

static void __exit qemu_csi2_vm_pci_driver_exit(void)
{
    pr_info("%s: Unloading driver\n", DRIVER_NAME);
    pci_unregister_driver(&qemu_csi2_vm_pci_driver);
    pr_info("%s: Driver unloaded\n", DRIVER_NAME);
}

module_init(qemu_csi2_vm_pci_driver_init);
module_exit(qemu_csi2_vm_pci_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("QEMU CSI2 Development Team");
MODULE_DESCRIPTION("QEMU CSI2 Virtual Camera V4L2 Driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS("pci:v00001234d00005678sv*sd*bc*sc*i*");
