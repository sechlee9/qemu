/*
 * AMD CSI-2 RX PCIe V4L2 Driver (v11.0 - QEMU Static Buffer Compatible)
 *
 * v11.0 Changes:
 * - Removed buffer sequence validation warnings
 * - Added V4L2 buffer flags for FFmpeg/GStreamer compatibility
 * - Reduced debug logging for better performance
 * - Compatible with QEMU static buffer implementation
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#define DRIVER_NAME "amd_csi2_v4l2"
#define DEVICE_NAME "AMD CSI-2 Camera"

/* Standard Register Offsets */
#define CSI2_CORE_CONFIG_REG            0x00
#define CSI2_INT_STATUS_REG             0x24
#define CSI2_INT_ENABLE_REG             0x28
#define CSI2_GLOBAL_INT_EN_REG          0x20
#define CSI2_CORE_STATUS_REG            0x10

/* Dynamic Format Configuration Registers */
#define CSI2_FORMAT_CONFIG_REG          0x40
#define CSI2_WIDTH_REG                  0x44
#define CSI2_HEIGHT_REG                 0x48
#define CSI2_FPS_REG                    0x4C

/* Error Status Register */
#define CSI2_ERROR_STATUS_REG           0x30
#define CSI2_ERROR_DMA_FAILED           (1U << 0)
#define CSI2_ERROR_QUEUE_OVERFLOW       (1U << 1)
#define CSI2_ERROR_FRAME_DROPPED        (1U << 2)

#define CSI2_CORE_CONFIG_CORE_EN_MASK   (1U << 0)
#define CSI2_INT_STATUS_FRAME_RECEIVED  (1U << 31)

/* DMA Queue Registers */
#define CSI2_DMA_QUEUE_DATA_LOW_REG     0x100
#define CSI2_DMA_QUEUE_DATA_HIGH_REG    0x104
#define CSI2_DMA_QUEUE_STATUS_REG       0x108

/* DMA Queue Status bits */
#define DMA_QUEUE_FULL_MASK             (1U << 15)
#define DMA_QUEUE_ENTRIES_SHIFT         8
#define DMA_QUEUE_ENTRIES_MASK          (0x1F << DMA_QUEUE_ENTRIES_SHIFT)
#define DMA_QUEUE_DEPTH_MASK            (0x1F)

#define DEFAULT_WIDTH  1920
#define DEFAULT_HEIGHT 1080
#define DEFAULT_FPS    30
#define BUFFER_TIMEOUT_MS 2000
#define IRQ_INTERVAL_TOLERANCE_MS 50

/* IRQ vector indices */
#define CSI2_IRQ_FRAME_DONE    0
#define CSI2_IRQ_DMA_ERROR     1

static bool use_polling = false;
module_param(use_polling, bool, 0644);
MODULE_PARM_DESC(use_polling, "Use polling instead of interrupts (default: false)");

struct amd_csi2_buffer {
    struct vb2_v4l2_buffer vb2_buf;
    struct list_head list;
    u32 queue_sequence;
    ktime_t queued_time;
};

struct amd_csi2_stats {
    u32 queued_buffers;
    u32 completed_buffers;
    u32 queue_full_errors;
    u32 timeout_errors;
    u32 irq_errors;
    u32 sequence_errors;
    u32 recovery_count;
};

struct amd_csi2_device {
    struct v4l2_device v4l2_dev;
    struct pci_dev *pdev;
    struct video_device *vdev;
    struct vb2_queue queue;
    struct mutex mutex;
    struct list_head buffer_list;
    spinlock_t buffer_lock;
    void __iomem *mmio_base;
    int irq_frame;
    int irq_error;
    unsigned int sequence;
    struct v4l2_pix_format format;
    bool streaming;
    struct tasklet_struct irq_tasklet;
    
    /* Sequence tracking */
    u32 queue_counter;
    u32 expected_sequence;
    
    /* Timeout handling */
    struct delayed_work timeout_work;
    
    /* Statistics */
    struct amd_csi2_stats stats;
    
    /* IRQ timing */
    unsigned long last_irq_jiffies;

    /* Polling mode */
    struct delayed_work poll_work;
};

static void dump_registers(struct amd_csi2_device *csi2_dev)
{
    dev_dbg(&csi2_dev->pdev->dev, "Register dump:\n");
    dev_dbg(&csi2_dev->pdev->dev, "  CORE_CONFIG: 0x%08x\n", 
             readl(csi2_dev->mmio_base + CSI2_CORE_CONFIG_REG));
    dev_dbg(&csi2_dev->pdev->dev, "  CORE_STATUS: 0x%08x\n", 
             readl(csi2_dev->mmio_base + CSI2_CORE_STATUS_REG));
    dev_dbg(&csi2_dev->pdev->dev, "  INT_STATUS: 0x%08x\n", 
             readl(csi2_dev->mmio_base + CSI2_INT_STATUS_REG));
    dev_dbg(&csi2_dev->pdev->dev, "  INT_ENABLE: 0x%08x\n", 
             readl(csi2_dev->mmio_base + CSI2_INT_ENABLE_REG));
    dev_dbg(&csi2_dev->pdev->dev, "  GLOBAL_INT_EN: 0x%08x\n", 
             readl(csi2_dev->mmio_base + CSI2_GLOBAL_INT_EN_REG));
    dev_dbg(&csi2_dev->pdev->dev, "  DMA_QUEUE_STATUS: 0x%08x\n", 
             readl(csi2_dev->mmio_base + CSI2_DMA_QUEUE_STATUS_REG));
}

static void csi2_poll_work(struct work_struct *work)
{
    struct amd_csi2_device *csi2_dev = container_of(to_delayed_work(work),
                                                    struct amd_csi2_device, 
                                                    poll_work);
    u32 status;
    
    if (!csi2_dev->streaming)
        return;
        
    status = readl(csi2_dev->mmio_base + CSI2_INT_STATUS_REG);
    if (status & CSI2_INT_STATUS_FRAME_RECEIVED) {
        /* Clear status */
        writel(CSI2_INT_STATUS_FRAME_RECEIVED, csi2_dev->mmio_base + CSI2_INT_STATUS_REG);
        
        /* Process buffer */
        tasklet_schedule(&csi2_dev->irq_tasklet);
        
        dev_dbg_once(&csi2_dev->pdev->dev, "Polling mode: Frame detected!\n");
    }
    
    /* Reschedule - 30fps = ~33ms */
    schedule_delayed_work(&csi2_dev->poll_work, msecs_to_jiffies(30));
}

static void amd_csi2_irq_tasklet(unsigned long data)
{
    struct amd_csi2_device *csi2_dev = (struct amd_csi2_device *)data;
    struct amd_csi2_buffer *buf;
    unsigned long flags;
    unsigned long now = jiffies;

    /* Check IRQ interval */
    if (csi2_dev->last_irq_jiffies && 
        time_after(now, csi2_dev->last_irq_jiffies + msecs_to_jiffies(IRQ_INTERVAL_TOLERANCE_MS))) {
        dev_warn_ratelimited(&csi2_dev->pdev->dev, 
                           "IRQ interval too long: %ums\n",
                           jiffies_to_msecs(now - csi2_dev->last_irq_jiffies));
        csi2_dev->stats.irq_errors++;
    }
    csi2_dev->last_irq_jiffies = now;

    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    if (list_empty(&csi2_dev->buffer_list)) {
        spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
        dev_err_ratelimited(&csi2_dev->pdev->dev, "IRQ with empty buffer queue!\n");
        csi2_dev->stats.irq_errors++;
        return;
    }
    
    buf = list_first_entry(&csi2_dev->buffer_list, struct amd_csi2_buffer, list);
    
    /* Remove buffer sequence validation - not needed for static buffer */
    
    list_del(&buf->list);
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);

    /* Complete buffer */
    buf->vb2_buf.vb2_buf.timestamp = ktime_get_ns();
    buf->vb2_buf.sequence = csi2_dev->sequence++;
    buf->vb2_buf.field = V4L2_FIELD_NONE;

    /* Clear any error flags for FFmpeg/GStreamer compatibility */
    buf->vb2_buf.flags &= ~V4L2_BUF_FLAG_ERROR;
    buf->vb2_buf.flags |= V4L2_BUF_FLAG_DONE;

    /* Set bytesused */
    vb2_set_plane_payload(&buf->vb2_buf.vb2_buf, 0, csi2_dev->format.sizeimage);
    vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_DONE);
    
    csi2_dev->stats.completed_buffers++;
}

static irqreturn_t amd_csi2_irq_handler(int irq, void *dev_id)
{
    struct amd_csi2_device *csi2_dev = dev_id;
    u32 status;

    if (!csi2_dev || !csi2_dev->mmio_base)
        return IRQ_NONE;

    status = readl(csi2_dev->mmio_base + CSI2_INT_STATUS_REG);

    if ((status == 0) || (status == ~0U))
        return IRQ_NONE;

    /* Handle frame interrupt */
    if (status & CSI2_INT_STATUS_FRAME_RECEIVED) {
        /* Clear interrupt status (W1C) */
        writel(CSI2_INT_STATUS_FRAME_RECEIVED, csi2_dev->mmio_base + CSI2_INT_STATUS_REG);
        tasklet_schedule(&csi2_dev->irq_tasklet);
        return IRQ_HANDLED;
    }

    return IRQ_NONE;
}

static void csi2_timeout_work(struct work_struct *work)
{
    struct amd_csi2_device *csi2_dev = container_of(to_delayed_work(work),
                                                    struct amd_csi2_device, 
                                                    timeout_work);
    struct amd_csi2_buffer *buf, *tmp;
    unsigned long flags;
    ktime_t now = ktime_get();
    LIST_HEAD(timeout_list);

    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_for_each_entry_safe(buf, tmp, &csi2_dev->buffer_list, list) {
        if (ktime_ms_delta(now, buf->queued_time) > BUFFER_TIMEOUT_MS) {
            list_move_tail(&buf->list, &timeout_list);
            dev_warn_ratelimited(&csi2_dev->pdev->dev, 
                    "Buffer timeout: seq=%u, age=%lldms\n", 
                    buf->queue_sequence,
                    ktime_ms_delta(now, buf->queued_time));
        }
    }
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);

    /* Process timed out buffers */
    list_for_each_entry_safe(buf, tmp, &timeout_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
        csi2_dev->stats.timeout_errors++;
    }

    /* Reschedule if streaming */
    if (csi2_dev->streaming)
        schedule_delayed_work(&csi2_dev->timeout_work, HZ);
}

static int queue_setup(struct vb2_queue *vq, unsigned int *nbuffers, unsigned int *nplanes,
                       unsigned int sizes[], struct device *alloc_devs[])
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vq);
    *nplanes = 1;
    sizes[0] = csi2_dev->format.sizeimage;
    return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    size_t size = csi2_dev->format.sizeimage;

    dev_dbg(&csi2_dev->pdev->dev,
            "buffer_prepare: size=%zu, plane_size=%lu\n",
            size, vb2_plane_size(vb, 0));

    if (vb2_plane_size(vb, 0) < size) {
        dev_err(&csi2_dev->pdev->dev,
                "Buffer too small (%lu < %zu)\n",
                vb2_plane_size(vb, 0), size);
        return -EINVAL;
    }

    vb2_set_plane_payload(vb, 0, csi2_dev->format.sizeimage);
    return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct amd_csi2_buffer *buf = container_of(vbuf, struct amd_csi2_buffer, vb2_buf);
    unsigned long flags;
    dma_addr_t dma_addr;
    u32 queue_status;

    dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
    if (!dma_addr) {
        vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
        return;
    }

    buf->queue_sequence = csi2_dev->queue_counter++;
    buf->queued_time = ktime_get();

    /* Critical section: SW queue add and HW transmission */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    
    /* 1. Add to SW queue first */
    list_add_tail(&buf->list, &csi2_dev->buffer_list);
    
    /* 2. Send to HW */
    writel(lower_32_bits(dma_addr), csi2_dev->mmio_base + CSI2_DMA_QUEUE_DATA_LOW_REG);
    writel(upper_32_bits(dma_addr), csi2_dev->mmio_base + CSI2_DMA_QUEUE_DATA_HIGH_REG);
    
    /* 3. Memory barrier */
    wmb();
    
    /* 4. Check HW status */
    queue_status = readl(csi2_dev->mmio_base + CSI2_DMA_QUEUE_STATUS_REG);
    if (queue_status & DMA_QUEUE_FULL_MASK) {
        /* If HW rejected, remove from SW queue */
        list_del(&buf->list);
        spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
        
        vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
        csi2_dev->stats.queue_full_errors++;
        return;
    }
    
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    csi2_dev->stats.queued_buffers++;
}

static void csi2_recover_streaming(struct amd_csi2_device *csi2_dev)
{
    unsigned long flags;
    struct amd_csi2_buffer *buf, *tmp;
    LIST_HEAD(recover_list);

    dev_warn(&csi2_dev->pdev->dev, "Attempting streaming recovery\n");

    /* Collect all pending buffers */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_splice_init(&csi2_dev->buffer_list, &recover_list);
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);

    /* Reset hardware */
    writel(0, csi2_dev->mmio_base + CSI2_GLOBAL_INT_EN_REG);
    writel(0, csi2_dev->mmio_base + CSI2_CORE_CONFIG_REG);
    msleep(10);

    /* Restart hardware */
    writel(CSI2_CORE_CONFIG_CORE_EN_MASK, csi2_dev->mmio_base + CSI2_CORE_CONFIG_REG);
    writel(CSI2_INT_STATUS_FRAME_RECEIVED, csi2_dev->mmio_base + CSI2_INT_ENABLE_REG);
    writel(1, csi2_dev->mmio_base + CSI2_GLOBAL_INT_EN_REG);

    /* Reset sequence tracking */
    csi2_dev->expected_sequence = csi2_dev->queue_counter;

    /* Re-queue buffers */
    list_for_each_entry_safe(buf, tmp, &recover_list, list) {
        list_del(&buf->list);
        buffer_queue(&buf->vb2_buf.vb2_buf);
    }

    csi2_dev->stats.recovery_count++;
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vq);

    dump_registers(csi2_dev);

    /* Reset counters */
    csi2_dev->sequence = 0;
    csi2_dev->queue_counter = 0;
    csi2_dev->expected_sequence = 0;
    csi2_dev->last_irq_jiffies = 0;
    csi2_dev->streaming = true;

    /* Clear any pending interrupts first */
    writel(0xFFFFFFFF, csi2_dev->mmio_base + CSI2_INT_STATUS_REG);

    /* Enable interrupts */
    writel(CSI2_INT_STATUS_FRAME_RECEIVED, csi2_dev->mmio_base + CSI2_INT_ENABLE_REG);
    writel(1, csi2_dev->mmio_base + CSI2_GLOBAL_INT_EN_REG);

    /* Enable core if not already enabled */
    if (!(readl(csi2_dev->mmio_base + CSI2_CORE_CONFIG_REG) &
          CSI2_CORE_CONFIG_CORE_EN_MASK)) {
        writel(CSI2_CORE_CONFIG_CORE_EN_MASK,
               csi2_dev->mmio_base + CSI2_CORE_CONFIG_REG);
    }

    dev_dbg(&csi2_dev->pdev->dev, "Streaming started\n");
    
    /* Start timeout monitoring or polling */
    if (use_polling) {
        INIT_DELAYED_WORK(&csi2_dev->poll_work, csi2_poll_work);
        schedule_delayed_work(&csi2_dev->poll_work, msecs_to_jiffies(30));
    } else {
        schedule_delayed_work(&csi2_dev->timeout_work, HZ);
    }

    return 0;
}

static void stop_streaming(struct vb2_queue *vq)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vq);
    unsigned long flags;

    csi2_dev->streaming = false;

    /* Cancel works */
    cancel_delayed_work_sync(&csi2_dev->timeout_work);
    cancel_delayed_work_sync(&csi2_dev->poll_work);

    /* Disable hardware */
    writel(0, csi2_dev->mmio_base + CSI2_GLOBAL_INT_EN_REG);
    writel(0, csi2_dev->mmio_base + CSI2_CORE_CONFIG_REG);

    tasklet_kill(&csi2_dev->irq_tasklet);

    /* Return all queued buffers */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    while (!list_empty(&csi2_dev->buffer_list)) {
        struct amd_csi2_buffer *buf = list_first_entry(&csi2_dev->buffer_list,
                                                       struct amd_csi2_buffer, list);
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
}

static const struct vb2_ops amd_csi2_vb2_ops = {
    .queue_setup = queue_setup,
    .buf_prepare = buffer_prepare,
    .buf_queue = buffer_queue,
    .start_streaming = start_streaming,
    .stop_streaming = stop_streaming,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
};

static int amd_csi2_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
    struct amd_csi2_device *csi2_dev = video_drvdata(file);
    strscpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
    strscpy(cap->card, DEVICE_NAME, sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", pci_name(csi2_dev->pdev));
    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
    return 0;
}

static int amd_csi2_enum_input(struct file *file, void *priv, struct v4l2_input *inp)
{
    if (inp->index != 0) return -EINVAL;
    inp->type = V4L2_INPUT_TYPE_CAMERA;
    strscpy(inp->name, "Camera 1", sizeof(inp->name));
    return 0;
}

static int amd_csi2_s_input(struct file *file, void *priv, unsigned int i)
{
    if (i != 0) return -EINVAL;
    return 0;
}

static int amd_csi2_g_input(struct file *file, void *priv, unsigned int *i)
{
    *i = 0;
    return 0;
}

static int amd_csi2_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV)
        f->fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    f->fmt.pix.width = clamp_t(u32, f->fmt.pix.width, 320, 1920);
    f->fmt.pix.height = clamp_t(u32, f->fmt.pix.height, 240, 1080);
    f->fmt.pix.bytesperline = f->fmt.pix.width * 2;
    f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;
    f->fmt.pix.field = V4L2_FIELD_NONE;
    f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    return 0;
}

static int amd_csi2_s_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct amd_csi2_device *csi2_dev = video_drvdata(file);
    if (vb2_is_busy(&csi2_dev->queue)) return -EBUSY;
    amd_csi2_try_fmt_vid_cap(file, priv, f);

    /* Send format info to QEMU */
    writel(f->fmt.pix.width, csi2_dev->mmio_base + CSI2_WIDTH_REG);
    writel(f->fmt.pix.height, csi2_dev->mmio_base + CSI2_HEIGHT_REG);
    writel(f->fmt.pix.pixelformat, csi2_dev->mmio_base + CSI2_FORMAT_CONFIG_REG);
    writel(30, csi2_dev->mmio_base + CSI2_FPS_REG); /* Default 30fps */

    csi2_dev->format = f->fmt.pix;
    return 0;
}

static int amd_csi2_g_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct amd_csi2_device *csi2_dev = video_drvdata(file);
    f->fmt.pix = csi2_dev->format;
    return 0;
}

static int amd_csi2_enum_frameintervals(struct file *file, void *priv,
                                       struct v4l2_frmivalenum *fival)
{
    if (fival->index != 0 || fival->pixel_format != V4L2_PIX_FMT_YUYV)
        return -EINVAL;
    fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
    fival->discrete.numerator = 1;
    fival->discrete.denominator = DEFAULT_FPS;
    return 0;
}

static int amd_csi2_g_parm(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
    if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return -EINVAL;
    parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    parm->parm.capture.timeperframe.numerator = 1;
    parm->parm.capture.timeperframe.denominator = DEFAULT_FPS;
    parm->parm.capture.readbuffers = 2;
    return 0;
}

static int amd_csi2_s_parm(struct file *file, void *priv, struct v4l2_streamparm *parm)
{
    return amd_csi2_g_parm(file, priv, parm);
}

static const struct v4l2_ioctl_ops amd_csi2_ioctl_ops = {
    .vidioc_querycap = amd_csi2_querycap,
    .vidioc_enum_input = amd_csi2_enum_input,
    .vidioc_g_input = amd_csi2_g_input,
    .vidioc_s_input = amd_csi2_s_input,
    .vidioc_try_fmt_vid_cap = amd_csi2_try_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap = amd_csi2_s_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap = amd_csi2_g_fmt_vid_cap,
    .vidioc_enum_frameintervals = amd_csi2_enum_frameintervals,
    .vidioc_g_parm = amd_csi2_g_parm,
    .vidioc_s_parm = amd_csi2_s_parm,
    .vidioc_reqbufs = vb2_ioctl_reqbufs,
    .vidioc_querybuf = vb2_ioctl_querybuf,
    .vidioc_qbuf = vb2_ioctl_qbuf,
    .vidioc_dqbuf = vb2_ioctl_dqbuf,
    .vidioc_streamon = vb2_ioctl_streamon,
    .vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations amd_csi2_fops = {
    .owner = THIS_MODULE,
    .open = v4l2_fh_open,
    .release = vb2_fop_release,
    .read = vb2_fop_read,
    .poll = vb2_fop_poll,
    .mmap = vb2_fop_mmap,
    .unlocked_ioctl = video_ioctl2,
};

/* Sysfs attributes for statistics */
static ssize_t stats_show(struct device *dev, 
                         struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    struct amd_csi2_device *csi2_dev = pci_get_drvdata(pdev);
    
    if (!csi2_dev)
        return -ENODEV;
    
    return sprintf(buf, 
                  "queued: %u\n"
                  "completed: %u\n"
                  "queue_full: %u\n"
                  "timeouts: %u\n"
                  "irq_errors: %u\n"
                  "sequence_errors: %u\n"
                  "recovery_count: %u\n",
                  csi2_dev->stats.queued_buffers,
                  csi2_dev->stats.completed_buffers,
                  csi2_dev->stats.queue_full_errors,
                  csi2_dev->stats.timeout_errors,
                  csi2_dev->stats.irq_errors,
                  csi2_dev->stats.sequence_errors,
                  csi2_dev->stats.recovery_count);
}
static DEVICE_ATTR_RO(stats);

static ssize_t recover_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    struct amd_csi2_device *csi2_dev = pci_get_drvdata(pdev);
    
    if (!csi2_dev)
        return -ENODEV;
    
    if (csi2_dev->streaming)
        csi2_recover_streaming(csi2_dev);
    
    return count;
}
static DEVICE_ATTR_WO(recover);

static struct attribute *amd_csi2_attrs[] = {
    &dev_attr_stats.attr,
    &dev_attr_recover.attr,
    NULL
};
ATTRIBUTE_GROUPS(amd_csi2);

static int amd_csi2_init_video_device(struct amd_csi2_device *csi2_dev)
{
    struct video_device *vdev;
    struct vb2_queue *q;
    int ret;

    q = &csi2_dev->queue;
    q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    q->io_modes = VB2_MMAP | VB2_READ;
    q->drv_priv = csi2_dev;
    q->ops = &amd_csi2_vb2_ops;
    q->mem_ops = &vb2_dma_contig_memops;
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    q->lock = &csi2_dev->mutex;
    q->dev = &csi2_dev->pdev->dev;
    q->buf_struct_size = sizeof(struct amd_csi2_buffer);

    ret = vb2_queue_init(q);
    if (ret) {
        dev_err(&csi2_dev->pdev->dev, "vb2_queue_init failed: %d\n", ret);
        return ret;
    }

    vdev = video_device_alloc();
    if (!vdev) {
        dev_err(&csi2_dev->pdev->dev, "video_device_alloc failed\n");
        ret = -ENOMEM;
        goto err_vb2_release;
    }
    csi2_dev->vdev = vdev;

    strscpy(vdev->name, DEVICE_NAME, sizeof(vdev->name));
    vdev->fops = &amd_csi2_fops;
    vdev->ioctl_ops = &amd_csi2_ioctl_ops;
    vdev->release = video_device_release;
    vdev->queue = q;
    vdev->lock = &csi2_dev->mutex;
    vdev->v4l2_dev = &csi2_dev->v4l2_dev;
    vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
    vdev->vfl_type = VFL_TYPE_VIDEO;
    vdev->vfl_dir = VFL_DIR_RX;
    vdev->minor = -1;
    video_set_drvdata(vdev, csi2_dev);

    csi2_dev->format.pixelformat = V4L2_PIX_FMT_YUYV;
    csi2_dev->format.width = DEFAULT_WIDTH;
    csi2_dev->format.height = DEFAULT_HEIGHT;
    csi2_dev->format.field = V4L2_FIELD_NONE;
    csi2_dev->format.colorspace = V4L2_COLORSPACE_SRGB;
    csi2_dev->format.bytesperline = csi2_dev->format.width * 2;
    csi2_dev->format.sizeimage = csi2_dev->format.bytesperline * csi2_dev->format.height;

    ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
    if (ret) {
        dev_err(&csi2_dev->pdev->dev, "video_register_device failed: %d\n", ret);
        goto err_video_release;
    }

    dev_info(&csi2_dev->pdev->dev, "Video device registered as %s\n",
             video_device_node_name(vdev));
    return 0;

err_video_release:
    video_device_release(vdev);
    csi2_dev->vdev = NULL;
err_vb2_release:
    vb2_queue_release(q);
    return ret;
}

static int amd_csi2_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct amd_csi2_device *csi2_dev;
    int ret;

    csi2_dev = kzalloc(sizeof(*csi2_dev), GFP_KERNEL);
    if (!csi2_dev)
        return -ENOMEM;
    pci_set_drvdata(pdev, csi2_dev);
    csi2_dev->pdev = pdev;

    ret = v4l2_device_register(&pdev->dev, &csi2_dev->v4l2_dev);
    if (ret) {
        dev_err(&pdev->dev, "v4l2_device_register failed: %d\n", ret);
        goto err_free_dev;
    }

    mutex_init(&csi2_dev->mutex);
    spin_lock_init(&csi2_dev->buffer_lock);
    INIT_LIST_HEAD(&csi2_dev->buffer_list);
    INIT_DELAYED_WORK(&csi2_dev->timeout_work, csi2_timeout_work);

    ret = pci_enable_device(pdev);
    if (ret) {
        dev_err(&pdev->dev, "pci_enable_device failed: %d\n", ret);
        goto err_v4l2_unregister;
    }

    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (ret) {
        ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (ret) {
            dev_err(&pdev->dev, "DMA mask setup failed: %d\n", ret);
            goto err_pci_disable;
        }
    }

    ret = pci_request_regions(pdev, DRIVER_NAME);
    if (ret) {
        dev_err(&pdev->dev, "pci_request_regions failed: %d\n", ret);
        goto err_pci_disable;
    }
    pci_set_master(pdev);

    csi2_dev->mmio_base = pci_ioremap_bar(pdev, 0);
    if (!csi2_dev->mmio_base) {
        dev_err(&pdev->dev, "pci_ioremap_bar(0) failed\n");
        ret = -ENOMEM;
        goto err_release_regions;
    }

    /* Allocate MSI-X vectors */
    ret = pci_alloc_irq_vectors(pdev, 4, 4, PCI_IRQ_MSIX);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate MSI-X vectors: %d\n", ret);
        goto err_iounmap_mmio;
    }

    /* Request frame interrupt (vector 0) */
    csi2_dev->irq_frame = pci_irq_vector(pdev, CSI2_IRQ_FRAME_DONE);
    ret = request_irq(csi2_dev->irq_frame, amd_csi2_irq_handler, 
                      IRQF_SHARED, "csi2-frame", csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to request frame IRQ: %d\n", ret);
        goto err_free_irq_vectors;
    }

    /* Request error interrupt (vector 1) */
    csi2_dev->irq_error = pci_irq_vector(pdev, CSI2_IRQ_DMA_ERROR);
    ret = request_irq(csi2_dev->irq_error, amd_csi2_irq_handler, 
                      IRQF_SHARED, "csi2-error", csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to request error IRQ: %d\n", ret);
        goto err_free_frame_irq;
    }

    tasklet_init(&csi2_dev->irq_tasklet, amd_csi2_irq_tasklet, (unsigned long)csi2_dev);

    /* Initialize hardware to safe state */
    writel(0, csi2_dev->mmio_base + CSI2_GLOBAL_INT_EN_REG);
    writel(0, csi2_dev->mmio_base + CSI2_CORE_CONFIG_REG);

    /* Create sysfs attributes */
    ret = sysfs_create_group(&pdev->dev.kobj, &amd_csi2_group);
    if (ret) {
        dev_err(&pdev->dev, "Failed to create sysfs attributes\n");
        goto err_free_error_irq;
    }

    ret = amd_csi2_init_video_device(csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "amd_csi2_init_video_device failed: %d\n", ret);
        goto err_remove_sysfs;
    }

    dev_info(&pdev->dev, "AMD CSI-2 device probe completed successfully (v11.0)\n");
    return 0;

err_remove_sysfs:
    sysfs_remove_group(&pdev->dev.kobj, &amd_csi2_group);
err_free_error_irq:
    free_irq(csi2_dev->irq_error, csi2_dev);
err_free_frame_irq:
    tasklet_kill(&csi2_dev->irq_tasklet);
    free_irq(csi2_dev->irq_frame, csi2_dev);
err_free_irq_vectors:
    pci_free_irq_vectors(pdev);
err_iounmap_mmio:
    iounmap(csi2_dev->mmio_base);
err_release_regions:
    pci_release_regions(pdev);
err_pci_disable:
    pci_disable_device(pdev);
err_v4l2_unregister:
    v4l2_device_unregister(&csi2_dev->v4l2_dev);
err_free_dev:
    kfree(csi2_dev);
    return ret;
}

static void amd_csi2_remove(struct pci_dev *pdev)
{
    struct amd_csi2_device *csi2_dev = pci_get_drvdata(pdev);

    if (!csi2_dev)
        return;

    dev_info(&pdev->dev, "Removing AMD CSI-2 device\n");

    /* Cancel any pending work */
    cancel_delayed_work_sync(&csi2_dev->timeout_work);
    cancel_delayed_work_sync(&csi2_dev->poll_work);

    if (csi2_dev->mmio_base) {
        writel(0, csi2_dev->mmio_base + CSI2_CORE_CONFIG_REG);
        writel(0, csi2_dev->mmio_base + CSI2_GLOBAL_INT_EN_REG);
    }

    if (csi2_dev->vdev) {
        video_unregister_device(csi2_dev->vdev);
        csi2_dev->vdev = NULL;
    }

    /* Remove sysfs attributes */
    sysfs_remove_group(&pdev->dev.kobj, &amd_csi2_group);

    v4l2_device_unregister(&csi2_dev->v4l2_dev);

    tasklet_kill(&csi2_dev->irq_tasklet);
    
    /* Free both IRQs */
    if (csi2_dev->irq_error > 0)
        free_irq(csi2_dev->irq_error, csi2_dev);
    if (csi2_dev->irq_frame > 0)
        free_irq(csi2_dev->irq_frame, csi2_dev);
    
    pci_free_irq_vectors(pdev);

    if (csi2_dev->mmio_base)
        iounmap(csi2_dev->mmio_base);

    pci_release_regions(pdev);
    pci_disable_device(pdev);

    kfree(csi2_dev);
    dev_info(&pdev->dev, "AMD CSI-2 device removed successfully\n");
}

static const struct pci_device_id amd_csi2_pci_table[] = {
    { PCI_DEVICE(0x10ee, 0x9024) },
    { 0 }
};
MODULE_DEVICE_TABLE(pci, amd_csi2_pci_table);

static struct pci_driver amd_csi2_pci_driver = {
    .name = DRIVER_NAME,
    .id_table = amd_csi2_pci_table,
    .probe = amd_csi2_probe,
    .remove = amd_csi2_remove,
};

module_pci_driver(amd_csi2_pci_driver);

MODULE_AUTHOR("AMD, Modified by AI");
MODULE_DESCRIPTION("AMD CSI-2 RX PCIe V4L2 Driver (v11.0 - QEMU Static Buffer Compatible)");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("11.0");
