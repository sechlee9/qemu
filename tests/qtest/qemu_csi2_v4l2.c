// fixed_qemu_csi2_v4l2.c - V4L2 디바이스 등록 문제 수정 버전
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#define DRIVER_NAME "qemu-csi2-vsync"
#define DEVICE_NAME "QEMU CSI2"

// PCI IDs
#define QEMU_VENDOR_ID 0x1234
#define CSI2_DEVICE_ID 0x5678

// MMIO 레지스터 오프셋
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

// MSI-X 벡터 수
#define CSI2_MSIX_VECTORS 2
#define CSI2_MSIX_FRAME_VEC 0
#define CSI2_MSIX_ERROR_VEC 1

// 최대 버퍼 수
#define MAX_BUFFERS 32

// 기본 포맷
#define DEFAULT_WIDTH  1280
#define DEFAULT_HEIGHT 720
#define DEFAULT_FPS    30

struct csi2_buffer {
    struct vb2_v4l2_buffer vb;
    struct list_head list;
};

struct csi2_v4l2_device {
    struct pci_dev *pdev;
    struct v4l2_device v4l2_dev;
    struct video_device vdev;  // 정적 할당으로 변경
    struct v4l2_ctrl_handler ctrl_handler;
    
    // MMIO
    void __iomem *mmio_base;
    resource_size_t mmio_len;
    
    // MSI-X
    struct msix_entry msix_entries[CSI2_MSIX_VECTORS];
    bool msix_enabled;
    
    // Video buffer queue
    struct vb2_queue queue;
    struct mutex vb_queue_lock;
    struct mutex dev_lock;
    
    // Buffer management
    struct list_head buf_list;
    spinlock_t buf_lock;
    
    // Format and streaming
    struct v4l2_format format;
    bool streaming;
    atomic_t sequence;
    
    // Frame generation
    struct timer_list frame_timer;
    struct work_struct frame_work;
    struct workqueue_struct *frame_wq;
    
    // Statistics
    atomic_t frames_generated;
    atomic_t frames_dropped;
    
    // Device state
    bool initialized;
    bool v4l2_registered;
    bool use_legacy_irq;  // MSI-X 실패시 legacy IRQ 사용
};

// Forward declarations
static int csi2_vb2_queue_setup(struct vb2_queue *vq,
                                unsigned int *nbuffers,
                                unsigned int *nplanes,
                                unsigned int sizes[],
                                struct device *alloc_devs[]);
static void csi2_vb2_buf_queue(struct vb2_buffer *vb);
static int csi2_vb2_start_streaming(struct vb2_queue *vq, unsigned int count);
static void csi2_vb2_stop_streaming(struct vb2_queue *vq);

// VB2 operations
static const struct vb2_ops csi2_vb2_ops = {
    .queue_setup = csi2_vb2_queue_setup,
    .buf_queue = csi2_vb2_buf_queue,
    .start_streaming = csi2_vb2_start_streaming,
    .stop_streaming = csi2_vb2_stop_streaming,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
};

// V4L2 IOCTL operations forward declarations
static int csi2_vidioc_querycap(struct file *file, void *priv,
                                struct v4l2_capability *cap);
static int csi2_vidioc_g_fmt_vid_cap(struct file *file, void *priv,
                                     struct v4l2_format *fmt);
static int csi2_vidioc_s_fmt_vid_cap(struct file *file, void *priv,
                                     struct v4l2_format *fmt);
static int csi2_vidioc_try_fmt_vid_cap(struct file *file, void *priv,
                                       struct v4l2_format *fmt);
static int csi2_vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
                                        struct v4l2_fmtdesc *fmt);
static int csi2_vidioc_reqbufs(struct file *file, void *priv,
                               struct v4l2_requestbuffers *reqbufs);
static int csi2_vidioc_querybuf(struct file *file, void *priv,
                                struct v4l2_buffer *buf);
static int csi2_vidioc_qbuf(struct file *file, void *priv,
                            struct v4l2_buffer *buf);
static int csi2_vidioc_dqbuf(struct file *file, void *priv,
                             struct v4l2_buffer *buf);
static int csi2_vidioc_streamon(struct file *file, void *priv,
                                enum v4l2_buf_type type);
static int csi2_vidioc_streamoff(struct file *file, void *priv,
                                 enum v4l2_buf_type type);
static int csi2_vidioc_g_input(struct file *file, void *priv, unsigned int *i);
static int csi2_vidioc_s_input(struct file *file, void *priv, unsigned int i);
static int csi2_vidioc_enum_input(struct file *file, void *priv, struct v4l2_input *inp);
static int csi2_vidioc_g_parm(struct file *file, void *priv, struct v4l2_streamparm *parm);
static int csi2_vidioc_s_parm(struct file *file, void *priv, struct v4l2_streamparm *parm);

// V4L2 IOCTL operations
static const struct v4l2_ioctl_ops csi2_ioctl_ops = {
    .vidioc_querycap = csi2_vidioc_querycap,
    .vidioc_g_fmt_vid_cap = csi2_vidioc_g_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap = csi2_vidioc_s_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap = csi2_vidioc_try_fmt_vid_cap,
    .vidioc_enum_fmt_vid_cap = csi2_vidioc_enum_fmt_vid_cap,
    .vidioc_reqbufs = csi2_vidioc_reqbufs,
    .vidioc_querybuf = csi2_vidioc_querybuf,
    .vidioc_qbuf = csi2_vidioc_qbuf,
    .vidioc_dqbuf = csi2_vidioc_dqbuf,
    .vidioc_streamon = csi2_vidioc_streamon,
    .vidioc_streamoff = csi2_vidioc_streamoff,
    .vidioc_g_input = csi2_vidioc_g_input,
    .vidioc_s_input = csi2_vidioc_s_input,
    .vidioc_enum_input = csi2_vidioc_enum_input,
    .vidioc_g_parm = csi2_vidioc_g_parm,
    .vidioc_s_parm = csi2_vidioc_s_parm,
};

// File operations forward declarations
static int csi2_open(struct file *file);
static int csi2_release(struct file *file);
static __poll_t csi2_poll(struct file *file, struct poll_table_struct *wait);

// File operations
static const struct v4l2_file_operations csi2_fops = {
    .owner = THIS_MODULE,
    .open = csi2_open,
    .release = csi2_release,
    .poll = csi2_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap = vb2_fop_mmap,
    .read = vb2_fop_read,
};

// Helper functions
static inline u32 csi2_read_reg(struct csi2_v4l2_device *dev, u32 offset)
{
    return ioread32(dev->mmio_base + offset);
}

static inline void csi2_write_reg(struct csi2_v4l2_device *dev, u32 offset, u32 value)
{
    iowrite32(value, dev->mmio_base + offset);
}

// Format validation
static void csi2_fill_pix_format(struct v4l2_pix_format *pix)
{
    if (pix->width < 320) pix->width = 320;
    if (pix->width > 1920) pix->width = 1920;
    if (pix->height < 240) pix->height = 240;
    if (pix->height > 1080) pix->height = 1080;
    
    // Ensure alignment
    pix->width &= ~0x3;
    pix->height &= ~0x1;
    
    switch (pix->pixelformat) {
    case V4L2_PIX_FMT_RGB24:
        pix->bytesperline = pix->width * 3;
        pix->sizeimage = pix->width * pix->height * 3;
        break;
    case V4L2_PIX_FMT_YUYV:
        pix->bytesperline = pix->width * 2;
        pix->sizeimage = pix->width * pix->height * 2;
        break;
    default:
        pix->pixelformat = V4L2_PIX_FMT_RGB24;
        pix->bytesperline = pix->width * 3;
        pix->sizeimage = pix->width * pix->height * 3;
        break;
    }
    
    pix->field = V4L2_FIELD_NONE;
    pix->colorspace = V4L2_COLORSPACE_SRGB;
}

// VB2 callback implementations
static int csi2_vb2_queue_setup(struct vb2_queue *vq,
                                unsigned int *nbuffers,
                                unsigned int *nplanes,
                                unsigned int sizes[],
                                struct device *alloc_devs[])
{
    struct csi2_v4l2_device *dev = vb2_get_drv_priv(vq);
    
    if (*nbuffers < 2)
        *nbuffers = 2;
    if (*nbuffers > MAX_BUFFERS)
        *nbuffers = MAX_BUFFERS;
    
    *nplanes = 1;
    sizes[0] = dev->format.fmt.pix.sizeimage;
    
    dev_info(&dev->pdev->dev, "Queue setup: %u buffers, size %u\n",
             *nbuffers, sizes[0]);
    
    return 0;
}

static void csi2_vb2_buf_queue(struct vb2_buffer *vb)
{
    struct csi2_v4l2_device *dev = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct csi2_buffer *buf = container_of(vbuf, struct csi2_buffer, vb);
    unsigned long flags;
    
    spin_lock_irqsave(&dev->buf_lock, flags);
    list_add_tail(&buf->list, &dev->buf_list);
    spin_unlock_irqrestore(&dev->buf_lock, flags);
}

static int csi2_vb2_start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct csi2_v4l2_device *dev = vb2_get_drv_priv(vq);
    
    dev->streaming = true;
    atomic_set(&dev->sequence, 0);
    
    // Enable frame generation
    mod_timer(&dev->frame_timer, jiffies + msecs_to_jiffies(1000 / DEFAULT_FPS));
    
    // Enable V4L2 bridge in QEMU
    csi2_write_reg(dev, V4L2_REG_STREAMING, 1);
    
    dev_info(&dev->pdev->dev, "Streaming started\n");
    return 0;
}

static void csi2_vb2_stop_streaming(struct vb2_queue *vq)
{
    struct csi2_v4l2_device *dev = vb2_get_drv_priv(vq);
    struct csi2_buffer *buf, *tmp;
    unsigned long flags;
    
    dev->streaming = false;
    
    // Stop frame generation
    timer_delete_sync(&dev->frame_timer);
    flush_workqueue(dev->frame_wq);
    
    // Disable V4L2 bridge in QEMU
    csi2_write_reg(dev, V4L2_REG_STREAMING, 0);
    
    // Return all buffers
    spin_lock_irqsave(&dev->buf_lock, flags);
    list_for_each_entry_safe(buf, tmp, &dev->buf_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    spin_unlock_irqrestore(&dev->buf_lock, flags);
    
    dev_info(&dev->pdev->dev, "Streaming stopped\n");
}

// Frame generation work
static void csi2_frame_work(struct work_struct *work)
{
    struct csi2_v4l2_device *dev = container_of(work, struct csi2_v4l2_device, frame_work);
    struct csi2_buffer *buf;
    unsigned long flags;
    void *vbuf;
    u32 size;
    int i;
    
    if (!dev->streaming)
        return;
    
    // Get next buffer
    spin_lock_irqsave(&dev->buf_lock, flags);
    if (list_empty(&dev->buf_list)) {
        spin_unlock_irqrestore(&dev->buf_lock, flags);
        atomic_inc(&dev->frames_dropped);
        return;
    }
    
    buf = list_first_entry(&dev->buf_list, struct csi2_buffer, list);
    list_del(&buf->list);
    spin_unlock_irqrestore(&dev->buf_lock, flags);
    
    // Fill buffer with test pattern
    vbuf = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
    size = vb2_plane_size(&buf->vb.vb2_buf, 0);
    
    if (vbuf) {
        u8 *data = (u8*)vbuf;
        u8 pattern = atomic_read(&dev->sequence) & 0xFF;
        
        // Simple gradient pattern
        for (i = 0; i < size; i += 3) {
            data[i] = pattern;           // R
            data[i + 1] = pattern >> 1;  // G  
            data[i + 2] = pattern >> 2;  // B
        }
        
        vb2_set_plane_payload(&buf->vb.vb2_buf, 0, size);
    }
    
    // Set timestamp and sequence
    buf->vb.vb2_buf.timestamp = ktime_get_ns();
    buf->vb.sequence = atomic_inc_return(&dev->sequence);
    buf->vb.field = V4L2_FIELD_NONE;
    
    // Return buffer to userspace
    vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
    
    atomic_inc(&dev->frames_generated);
}

// Frame timer callback
static void csi2_frame_timer_callback(struct timer_list *t)
{
    struct csi2_v4l2_device *dev = from_timer(dev, t, frame_timer);
    
    if (dev->streaming) {
        queue_work(dev->frame_wq, &dev->frame_work);
        mod_timer(&dev->frame_timer, jiffies + msecs_to_jiffies(1000 / DEFAULT_FPS));
    }
}

// V4L2 IOCTL implementations
static int csi2_vidioc_querycap(struct file *file, void *priv,
                                struct v4l2_capability *cap)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    
    strscpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
    strscpy(cap->card, DEVICE_NAME, sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", pci_name(dev->pdev));
    
    cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | 
                       V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;
    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
    
    return 0;
}

static int csi2_vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
                                        struct v4l2_fmtdesc *fmt)
{
    if (fmt->index > 1)
        return -EINVAL;
    
    switch (fmt->index) {
    case 0:
        fmt->pixelformat = V4L2_PIX_FMT_RGB24;
        strscpy(fmt->description, "RGB24", sizeof(fmt->description));
        break;
    case 1:
        fmt->pixelformat = V4L2_PIX_FMT_YUYV;
        strscpy(fmt->description, "YUYV", sizeof(fmt->description));
        break;
    }
    
    return 0;
}

static int csi2_vidioc_g_fmt_vid_cap(struct file *file, void *priv,
                                     struct v4l2_format *fmt)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    
    *fmt = dev->format;
    return 0;
}

static int csi2_vidioc_try_fmt_vid_cap(struct file *file, void *priv,
                                       struct v4l2_format *fmt)
{
    if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    
    csi2_fill_pix_format(&fmt->fmt.pix);
    return 0;
}

static int csi2_vidioc_s_fmt_vid_cap(struct file *file, void *priv,
                                     struct v4l2_format *fmt)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    int ret;
    
    ret = csi2_vidioc_try_fmt_vid_cap(file, priv, fmt);
    if (ret)
        return ret;
    
    if (vb2_is_busy(&dev->queue))
        return -EBUSY;
    
    dev->format = *fmt;
    return 0;
}

static int csi2_vidioc_reqbufs(struct file *file, void *priv,
                               struct v4l2_requestbuffers *reqbufs)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    return vb2_reqbufs(&dev->queue, reqbufs);
}

static int csi2_vidioc_querybuf(struct file *file, void *priv,
                                struct v4l2_buffer *buf)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    return vb2_querybuf(&dev->queue, buf);
}

static int csi2_vidioc_qbuf(struct file *file, void *priv,
                            struct v4l2_buffer *buf)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    return vb2_qbuf(&dev->queue, NULL, buf);
}

static int csi2_vidioc_dqbuf(struct file *file, void *priv,
                             struct v4l2_buffer *buf)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    return vb2_dqbuf(&dev->queue, buf, file->f_flags & O_NONBLOCK);
}

static int csi2_vidioc_streamon(struct file *file, void *priv,
                                enum v4l2_buf_type type)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    return vb2_streamon(&dev->queue, type);
}

static int csi2_vidioc_streamoff(struct file *file, void *priv,
                                 enum v4l2_buf_type type)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    return vb2_streamoff(&dev->queue, type);
}

static int csi2_vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
    *i = 0;
    return 0;
}

static int csi2_vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
    if (i != 0)
        return -EINVAL;
    return 0;
}

static int csi2_vidioc_enum_input(struct file *file, void *priv, struct v4l2_input *inp)
{
    if (inp->index != 0)
        return -EINVAL;
    
    strscpy(inp->name, "QEMU CSI2 Camera", sizeof(inp->name));
    inp->type = V4L2_INPUT_TYPE_CAMERA;
    inp->status = 0;
    inp->capabilities = 0;
    
    return 0;
}

static int csi2_vidioc_g_parm(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
    if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    
    parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    parm->parm.capture.timeperframe.numerator = 1;
    parm->parm.capture.timeperframe.denominator = DEFAULT_FPS;
    parm->parm.capture.readbuffers = 4;
    
    return 0;
}

static int csi2_vidioc_s_parm(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
    if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    
    return csi2_vidioc_g_parm(file, priv, parm);
}

// File operations
static int csi2_open(struct file *file)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    int ret;
    
    dev_info(&dev->pdev->dev, "Device opening...\n");
    
    mutex_lock(&dev->dev_lock);
    
    ret = v4l2_fh_open(file);
    if (ret) {
        dev_err(&dev->pdev->dev, "v4l2_fh_open failed: %d\n", ret);
        goto unlock;
    }
    
    if (v4l2_fh_is_singular_file(file)) {
        atomic_set(&dev->sequence, 0);
        atomic_set(&dev->frames_generated, 0);
        atomic_set(&dev->frames_dropped, 0);
    }
    
    dev_info(&dev->pdev->dev, "Device opened successfully\n");

unlock:
    mutex_unlock(&dev->dev_lock);
    return ret;
}

static int csi2_release(struct file *file)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    int ret;
    
    dev_info(&dev->pdev->dev, "Device closing...\n");
    
    mutex_lock(&dev->dev_lock);
    
    if (v4l2_fh_is_singular_file(file) && dev->streaming) {
        vb2_streamoff(&dev->queue, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    }
    
    ret = _vb2_fop_release(file, NULL);
    
    dev_info(&dev->pdev->dev, "Device closed\n");
    
    mutex_unlock(&dev->dev_lock);
    return ret;
}

static __poll_t csi2_poll(struct file *file, struct poll_table_struct *wait)
{
    struct csi2_v4l2_device *dev = video_drvdata(file);
    return vb2_poll(&dev->queue, file, wait);
}

// Legacy IRQ handler (MSI-X가 실패했을 때 사용)
static irqreturn_t csi2_legacy_irq_handler(int irq, void *dev_id)
{
    struct csi2_v4l2_device *dev = dev_id;
    u32 int_status;
    
    int_status = csi2_read_reg(dev, CSI2_REG_INT_STATUS);
    if (!int_status)
        return IRQ_NONE;
    
    // Clear interrupt
    csi2_write_reg(dev, CSI2_REG_INT_STATUS, int_status);
    
    // Handle frame interrupt
    if (int_status & 0x80000000) {
        if (dev->streaming) {
            queue_work(dev->frame_wq, &dev->frame_work);
        }
    }
    
    // Handle error interrupts
    if (int_status & 0x7FFFFFFF) {
        dev_warn(&dev->pdev->dev, "CSI2 error interrupt: 0x%08x\n", int_status);
    }
    
    return IRQ_HANDLED;
}

// MSI-X interrupt handlers
static irqreturn_t csi2_frame_irq_handler(int irq, void *dev_id)
{
    struct csi2_v4l2_device *dev = dev_id;
    u32 int_status;
    
    int_status = csi2_read_reg(dev, CSI2_REG_INT_STATUS);
    if (!(int_status & 0x80000000))
        return IRQ_NONE;
    
    // Clear frame interrupt
    csi2_write_reg(dev, CSI2_REG_INT_STATUS, 0x80000000);
    
    if (dev->streaming) {
        queue_work(dev->frame_wq, &dev->frame_work);
    }
    
    return IRQ_HANDLED;
}

static irqreturn_t csi2_error_irq_handler(int irq, void *dev_id)
{
    struct csi2_v4l2_device *dev = dev_id;
    u32 int_status;
    
    int_status = csi2_read_reg(dev, CSI2_REG_INT_STATUS);
    if (!(int_status & 0x7FFFFFFF))
        return IRQ_NONE;
    
    // Clear error interrupts
    csi2_write_reg(dev, CSI2_REG_INT_STATUS, int_status & 0x7FFFFFFF);
    
    dev_warn(&dev->pdev->dev, "CSI2 error interrupt: 0x%08x\n", int_status);
    
    return IRQ_HANDLED;
}

// MSI-X setup
static int csi2_setup_msix(struct csi2_v4l2_device *dev)
{
    int ret, i;
    
    // Initialize MSI-X entries
    for (i = 0; i < CSI2_MSIX_VECTORS; i++) {
        dev->msix_entries[i].entry = i;
    }
    
    // Enable MSI-X
    ret = pci_enable_msix_exact(dev->pdev, dev->msix_entries, CSI2_MSIX_VECTORS);
    if (ret) {
        dev_warn(&dev->pdev->dev, "Failed to enable MSI-X: %d, will use legacy IRQ\n", ret);
        return ret;
    }
    
    dev_info(&dev->pdev->dev, "Allocated %d MSI-X vectors\n", CSI2_MSIX_VECTORS);
    
    // Request frame interrupt handler
    ret = request_irq(dev->msix_entries[CSI2_MSIX_FRAME_VEC].vector,
                     csi2_frame_irq_handler, 0, "csi2-frame", dev);
    if (ret) {
        dev_err(&dev->pdev->dev, "Failed to request frame IRQ: %d\n", ret);
        goto err_disable_msix;
    }
    
    dev_info(&dev->pdev->dev, "MSI-X frame handler registered (IRQ %d)\n",
             dev->msix_entries[CSI2_MSIX_FRAME_VEC].vector);
    
    // Request error interrupt handler
    ret = request_irq(dev->msix_entries[CSI2_MSIX_ERROR_VEC].vector,
                     csi2_error_irq_handler, 0, "csi2-error", dev);
    if (ret) {
        dev_err(&dev->pdev->dev, "Failed to request error IRQ: %d\n", ret);
        goto err_free_frame_irq;
    }
    
    dev_info(&dev->pdev->dev, "MSI-X error handler registered (IRQ %d)\n",
             dev->msix_entries[CSI2_MSIX_ERROR_VEC].vector);
    
    dev->msix_enabled = true;
    dev->use_legacy_irq = false;
    dev_info(&dev->pdev->dev, "MSI-X setup completed successfully\n");
    
    return 0;
    
err_free_frame_irq:
    free_irq(dev->msix_entries[CSI2_MSIX_FRAME_VEC].vector, dev);
err_disable_msix:
    pci_disable_msix(dev->pdev);
    return ret;
}

// Legacy IRQ setup
static int csi2_setup_legacy_irq(struct csi2_v4l2_device *dev)
{
    int ret;
    
    ret = request_irq(dev->pdev->irq, csi2_legacy_irq_handler, 
                     IRQF_SHARED, DRIVER_NAME, dev);
    if (ret) {
        dev_err(&dev->pdev->dev, "Failed to request legacy IRQ %d: %d\n", 
                dev->pdev->irq, ret);
        return ret;
    }
    
    dev->use_legacy_irq = true;
    dev->msix_enabled = false;
    dev_info(&dev->pdev->dev, "Legacy IRQ %d registered\n", dev->pdev->irq);
    
    return 0;
}

static void csi2_cleanup_interrupts(struct csi2_v4l2_device *dev)
{
    if (dev->msix_enabled) {
        free_irq(dev->msix_entries[CSI2_MSIX_ERROR_VEC].vector, dev);
        free_irq(dev->msix_entries[CSI2_MSIX_FRAME_VEC].vector, dev);
        pci_disable_msix(dev->pdev);
        dev->msix_enabled = false;
        dev_info(&dev->pdev->dev, "MSI-X cleaned up\n");
    } else if (dev->use_legacy_irq) {
        free_irq(dev->pdev->irq, dev);
        dev->use_legacy_irq = false;
        dev_info(&dev->pdev->dev, "Legacy IRQ cleaned up\n");
    }
}

// PCI probe
static int csi2_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct csi2_v4l2_device *dev;
    int ret;
    
    dev_info(&pdev->dev, "Probing QEMU CSI2 device (fixed version)\n");
    
    // Allocate device structure
    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;
    
    dev->pdev = pdev;
    pci_set_drvdata(pdev, dev);
    
    // Enable PCI device
    ret = pci_enable_device(pdev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable PCI device: %d\n", ret);
        return ret;
    }
    
    pci_set_master(pdev);
    
    // Map MMIO
    ret = pci_request_regions(pdev, DRIVER_NAME);
    if (ret) {
        dev_err(&pdev->dev, "Failed to request PCI regions: %d\n", ret);
        goto err_disable_pci;
    }
    
    dev->mmio_base = pci_iomap(pdev, 0, 0);
    if (!dev->mmio_base) {
        dev_err(&pdev->dev, "Failed to map MMIO\n");
        ret = -ENOMEM;
        goto err_release_regions;
    }
    
    dev->mmio_len = pci_resource_len(pdev, 0);
    dev_info(&pdev->dev, "MMIO mapped at %p, length 0x%llx\n", 
             dev->mmio_base, (unsigned long long)dev->mmio_len);
    
    // Initialize locks and lists
    mutex_init(&dev->vb_queue_lock);
    mutex_init(&dev->dev_lock);
    spin_lock_init(&dev->buf_lock);
    INIT_LIST_HEAD(&dev->buf_list);
    
    // Initialize atomic counters
    atomic_set(&dev->sequence, 0);
    atomic_set(&dev->frames_generated, 0);
    atomic_set(&dev->frames_dropped, 0);
    
    // Initialize default format
    dev->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    dev->format.fmt.pix.width = DEFAULT_WIDTH;
    dev->format.fmt.pix.height = DEFAULT_HEIGHT;
    dev->format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    csi2_fill_pix_format(&dev->format.fmt.pix);
    
    // Initialize V4L2 device
    ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to register V4L2 device: %d\n", ret);
        goto err_unmap;
    }
    
    dev_info(&pdev->dev, "V4L2 device registered successfully\n");
    
    // Initialize video buffer queue
    dev->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    dev->queue.io_modes = VB2_MMAP | VB2_READ | VB2_USERPTR;
    dev->queue.drv_priv = dev;
    dev->queue.buf_struct_size = sizeof(struct csi2_buffer);
    dev->queue.ops = &csi2_vb2_ops;
    dev->queue.mem_ops = &vb2_vmalloc_memops;
    dev->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    dev->queue.lock = &dev->vb_queue_lock;
    dev->queue.dev = &pdev->dev;
    
    ret = vb2_queue_init(&dev->queue);
    if (ret) {
        dev_err(&pdev->dev, "Failed to initialize VB2 queue: %d\n", ret);
        goto err_unreg_v4l2;
    }
    
    dev_info(&pdev->dev, "VB2 queue initialized successfully\n");
    
    // Initialize video device (정적 할당)
    strscpy(dev->vdev.name, DEVICE_NAME, sizeof(dev->vdev.name));
    dev->vdev.fops = &csi2_fops;
    dev->vdev.ioctl_ops = &csi2_ioctl_ops;
    dev->vdev.minor = -1;
    dev->vdev.release = video_device_release_empty;  // 정적 할당이므로 빈 함수 사용
    dev->vdev.lock = &dev->dev_lock;
    dev->vdev.queue = &dev->queue;
    dev->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
    dev->vdev.v4l2_dev = &dev->v4l2_dev;
    
    video_set_drvdata(&dev->vdev, dev);
    
    dev_info(&pdev->dev, "Video device structure initialized\n");
    
    // Register video device
    ret = video_register_device(&dev->vdev, VFL_TYPE_VIDEO, -1);
    if (ret) {
        dev_err(&pdev->dev, "Failed to register video device: %d\n", ret);
        goto err_unreg_v4l2;
    }
    
    dev->v4l2_registered = true;
    dev_info(&pdev->dev, "Video device registered as %s\n", 
             video_device_node_name(&dev->vdev));
    
    // Create workqueue for frame processing
    dev->frame_wq = create_singlethread_workqueue("csi2_frames");
    if (!dev->frame_wq) {
        dev_err(&pdev->dev, "Failed to create workqueue\n");
        ret = -ENOMEM;
        goto err_unreg_video;
    }
    
    INIT_WORK(&dev->frame_work, csi2_frame_work);
    
    // Initialize timer
    timer_setup(&dev->frame_timer, csi2_frame_timer_callback, 0);
    
    // Setup interrupts (MSI-X 우선, 실패시 legacy IRQ)
    if (csi2_setup_msix(dev) != 0) {
        dev_info(&pdev->dev, "MSI-X setup failed, trying legacy IRQ\n");
        ret = csi2_setup_legacy_irq(dev);
        if (ret) {
            dev_err(&pdev->dev, "Both MSI-X and legacy IRQ setup failed\n");
            goto err_cleanup_workqueue;
        }
    }
    
    // Enable CSI2 controller
    csi2_write_reg(dev, CSI2_REG_CORE_CONFIG, 1);
    csi2_write_reg(dev, CSI2_REG_INT_ENABLE, 0x80000000);
    
    dev->initialized = true;
    
    dev_info(&pdev->dev, "QEMU CSI2 V4L2 device registered successfully\n");
    if (dev->msix_enabled) {
        dev_info(&pdev->dev, "Interrupt mode: MSI-X (vectors: Frame=%d, Error=%d)\n",
                 dev->msix_entries[CSI2_MSIX_FRAME_VEC].vector,
                 dev->msix_entries[CSI2_MSIX_ERROR_VEC].vector);
    } else {
        dev_info(&pdev->dev, "Interrupt mode: Legacy IRQ %d\n", dev->pdev->irq);
    }
    
    return 0;

err_cleanup_workqueue:
    if (dev->frame_wq) {
        destroy_workqueue(dev->frame_wq);
    }
err_unreg_video:
    if (dev->v4l2_registered) {
        video_unregister_device(&dev->vdev);
        dev->v4l2_registered = false;
    }
err_unreg_v4l2:
    v4l2_device_unregister(&dev->v4l2_dev);
err_unmap:
    pci_iounmap(pdev, dev->mmio_base);
err_release_regions:
    pci_release_regions(pdev);
err_disable_pci:
    pci_disable_device(pdev);
    return ret;
}

static void csi2_pci_remove(struct pci_dev *pdev)
{
    struct csi2_v4l2_device *dev = pci_get_drvdata(pdev);
    
    dev_info(&pdev->dev, "Removing QEMU CSI2 device\n");
    
    if (!dev)
        return;
    
    dev->initialized = false;
    
    // Stop streaming if active
    if (dev->streaming) {
        vb2_streamoff(&dev->queue, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    }
    
    // Stop and cleanup timer
    timer_delete_sync(&dev->frame_timer);
    
    // Cleanup workqueue
    if (dev->frame_wq) {
        flush_workqueue(dev->frame_wq);
        destroy_workqueue(dev->frame_wq);
    }
    
    // Cleanup interrupts
    csi2_cleanup_interrupts(dev);
    
    // Unregister devices
    if (dev->v4l2_registered) {
        video_unregister_device(&dev->vdev);
        dev->v4l2_registered = false;
    }
    
    v4l2_device_unregister(&dev->v4l2_dev);
    
    // Cleanup PCI
    pci_iounmap(pdev, dev->mmio_base);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    
    dev_info(&pdev->dev, "QEMU CSI2 device removed\n");
}

// PCI device table
static const struct pci_device_id csi2_pci_ids[] = {
    { PCI_DEVICE(QEMU_VENDOR_ID, CSI2_DEVICE_ID) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, csi2_pci_ids);

// PCI driver
static struct pci_driver csi2_pci_driver = {
    .name = DRIVER_NAME,
    .id_table = csi2_pci_ids,
    .probe = csi2_pci_probe,
    .remove = csi2_pci_remove,
};

// Module init/exit
static int __init csi2_init(void)
{
    int ret;
    
    pr_info("QEMU CSI2 V4L2 Driver (Fixed Version) loading...\n");
    
    ret = pci_register_driver(&csi2_pci_driver);
    if (ret) {
        pr_err("Failed to register PCI driver: %d\n", ret);
        return ret;
    }
    
    pr_info("QEMU CSI2 V4L2 Driver loaded successfully\n");
    return 0;
}

static void __exit csi2_exit(void)
{
    pr_info("QEMU CSI2 V4L2 Driver unloading...\n");
    pci_unregister_driver(&csi2_pci_driver);
    pr_info("QEMU CSI2 V4L2 Driver unloaded\n");
}

module_init(csi2_init);
module_exit(csi2_exit);

MODULE_AUTHOR("QEMU CSI2 Team");
MODULE_DESCRIPTION("QEMU CSI2 V4L2 Integration Driver (Fixed)");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.2");
