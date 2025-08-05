/*
 * AMD CSI-2 RX PCIe V4L2 Driver with Ring Buffer
 * Version: 2.1 - Fixed deadlock issues with consistent lock ordering
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#define DRIVER_NAME "amd_csi2_v4l2"
#define DEVICE_NAME "AMD CSI-2 Camera"

/* Register Offsets */
#define CSI2_CORE_CONFIG_REG            0x00
#define CSI2_CORE_STATUS_REG            0x10
#define CSI2_INT_STATUS_REG             0x20
#define CSI2_INT_ENABLE_REG             0x24
#define CSI2_GLOBAL_INT_EN_REG          0x28

/* Ring Buffer Registers */
#define CSI2_RING_BASE_LOW              0x100
#define CSI2_RING_BASE_HIGH             0x104
#define CSI2_RING_SIZE_REG              0x108
#define CSI2_RING_HEAD_REG              0x10C
#define CSI2_RING_TAIL_REG              0x110
#define CSI2_RING_CTRL_REG              0x114
#define CSI2_RING_STATUS_REG            0x118

/* Format Registers */
#define CSI2_FORMAT_REG                 0x200
#define CSI2_WIDTH_REG                  0x204
#define CSI2_HEIGHT_REG                 0x208
#define CSI2_FPS_REG                    0x20C

/* Debug Registers */
#define CSI2_DEBUG_REG                  0x300
#define CSI2_ERROR_COUNT_REG            0x304
#define CSI2_FRAME_COUNT_REG            0x308

/* Control bits */
#define CSI2_CORE_CONFIG_ENABLE         (1U << 0)
#define CSI2_CORE_CONFIG_RESET          (1U << 1)

/* Ring control bits */
#define RING_CTRL_ENABLE                (1U << 0)
#define RING_CTRL_RESET                 (1U << 1)

/* Ring status bits */
#define RING_STATUS_READY               (1U << 0)
#define RING_STATUS_FULL                (1U << 1)
#define RING_STATUS_EMPTY               (1U << 2)
#define RING_STATUS_ERROR               (1U << 3)

/* Interrupt bits */
#define CSI2_INT_FRAME_DONE             (1U << 0)
#define CSI2_INT_RING_FULL              (1U << 1)
#define CSI2_INT_ERROR                  (1U << 2)

#define DEFAULT_WIDTH  1920
#define DEFAULT_HEIGHT 1080
#define DEFAULT_FPS    30
#define RING_SIZE      16

/* Debug levels */
static int debug_level = 3;
module_param(debug_level, int, 0644);
MODULE_PARM_DESC(debug_level, "Debug level (0-4)");

#define csi2_dbg(level, dev, fmt, ...) \
    do { \
        if (debug_level >= level) \
            dev_info(dev, "[CSI2] " fmt, ##__VA_ARGS__); \
    } while (0)

/* Ring buffer entry structure - must match QEMU */
struct ring_entry {
    u64 dma_addr;
    u32 size;
    u32 flags;
    u64 timestamp;
    u32 reserved[2];
} __packed;

/* Ring buffer structure - must match QEMU */
struct ring_buffer {
    u32 magic;      /* 0x43534932 - "CSI2" */
    u32 version;    /* 0x00020000 - version 2.0 */
    u32 head;       /* Written by driver */
    u32 tail;       /* Written by QEMU */
    u32 size;       /* Number of entries */
    u32 entry_size; /* Size of each entry */
    u32 reserved[2];
    struct ring_entry entries[RING_SIZE];
} __packed;

struct amd_csi2_buffer {
    struct vb2_v4l2_buffer vb2_buf;
    struct list_head list;
    u32 index;
};

struct amd_csi2_device {
    struct v4l2_device v4l2_dev;
    struct pci_dev *pdev;
    struct video_device *vdev;
    struct vb2_queue queue;
    struct mutex mutex;
    struct list_head buffer_list;
    
    /* Lock ordering: ALWAYS acquire in this order:
     * 1. buffer_lock
     * 2. ring_lock
     * Never acquire buffer_lock while holding ring_lock!
     */
    spinlock_t buffer_lock;  /* Protects buffer_list */
    spinlock_t ring_lock;    /* Protects ring buffer and active_buffers */
    
    void __iomem *mmio_base;
    int irq;
    
    /* Ring buffer */
    struct ring_buffer *ring;
    dma_addr_t ring_dma;
    struct amd_csi2_buffer *active_buffers[RING_SIZE];
    u32 ring_head;
    u32 ring_tail;
    
    /* State */
    bool streaming;
    unsigned int sequence;
    struct v4l2_pix_format format;
    
    /* Statistics */
    u32 queued_buffers;
    u32 completed_buffers;
    u32 dropped_frames;
    u32 errors;
    
    /* Workqueue for bottom half processing */
    struct work_struct irq_work;
    struct list_head done_list;
    
    /* Watchdog */
    struct delayed_work watchdog_work;
};

#ifdef CONFIG_LOCKDEP
/* Define lock classes for lockdep */
static struct lock_class_key buffer_lock_key;
static struct lock_class_key ring_lock_key;
#endif

/* Helper functions */
static inline u32 csi2_read_reg(struct amd_csi2_device *csi2_dev, u32 offset)
{
    return readl(csi2_dev->mmio_base + offset);
}

static inline void csi2_write_reg(struct amd_csi2_device *csi2_dev, u32 offset, u32 value)
{
    writel(value, csi2_dev->mmio_base + offset);
}

static int ring_is_full(u32 head, u32 tail, u32 size)
{
    return ((head + 1) % size) == tail;
}

static int ring_is_empty(u32 head, u32 tail)
{
    return head == tail;
}

static int ring_count(u32 head, u32 tail, u32 size)
{
    if (head >= tail)
        return head - tail;
    else
        return size - tail + head;
}

/* Must be called with ring_lock held */
static void __csi2_queue_buffer_to_ring_locked(struct amd_csi2_device *csi2_dev,
                                               struct amd_csi2_buffer *buf)
{
    struct vb2_buffer *vb = &buf->vb2_buf.vb2_buf;
    u32 head = csi2_dev->ring_head;
    
    /* Check if ring is full */
    if (ring_is_full(head, csi2_dev->ring_tail, RING_SIZE)) {
        csi2_dbg(2, &csi2_dev->pdev->dev, "Ring buffer full\n");
        vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
        csi2_dev->dropped_frames++;
        return;
    }
    
    /* Fill ring entry */
    csi2_dev->ring->entries[head].dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
    csi2_dev->ring->entries[head].size = vb2_plane_size(vb, 0);
    csi2_dev->ring->entries[head].flags = 0;
    csi2_dev->ring->entries[head].timestamp = 0;
    
    /* Store buffer reference */
    csi2_dev->active_buffers[head] = buf;
    
    /* Update head */
    csi2_dev->ring_head = (head + 1) % RING_SIZE;
    csi2_dev->ring->head = csi2_dev->ring_head;
    
    /* Memory barrier to ensure ring update is visible */
    wmb();
    
    csi2_dev->queued_buffers++;
    
    csi2_dbg(4, &csi2_dev->pdev->dev, 
             "Queued buffer %d to ring slot %u (dma: 0x%llx)\n",
             buf->index, head, csi2_dev->ring->entries[head].dma_addr);
}

static void csi2_queue_buffer_to_ring(struct amd_csi2_device *csi2_dev,
                                     struct amd_csi2_buffer *buf)
{
    unsigned long flags;
    
    /* Single lock acquisition */
    spin_lock_irqsave(&csi2_dev->ring_lock, flags);
    __csi2_queue_buffer_to_ring_locked(csi2_dev, buf);
    spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
}

/* Must be called with ring_lock held */
static void __csi2_process_completed_buffers_locked(struct amd_csi2_device *csi2_dev,
                                                   struct list_head *done_list)
{
    u32 tail = csi2_dev->ring->tail;
    
    /* Process all completed buffers */
    while (csi2_dev->ring_tail != tail) {
        struct amd_csi2_buffer *buf;
        struct ring_entry *entry;
        
        buf = csi2_dev->active_buffers[csi2_dev->ring_tail];
        entry = &csi2_dev->ring->entries[csi2_dev->ring_tail];
        
        if (!buf) {
            csi2_dbg(1, &csi2_dev->pdev->dev, 
                     "No buffer in slot %u\n", csi2_dev->ring_tail);
            csi2_dev->ring_tail = (csi2_dev->ring_tail + 1) % RING_SIZE;
            continue;
        }
        
        /* Clear active buffer */
        csi2_dev->active_buffers[csi2_dev->ring_tail] = NULL;
        
        /* Update buffer metadata */
        buf->vb2_buf.vb2_buf.timestamp = entry->timestamp;
        buf->vb2_buf.sequence = csi2_dev->sequence++;
        buf->vb2_buf.field = V4L2_FIELD_NONE;
        
        /* Add to done list for processing outside lock */
        list_add_tail(&buf->list, done_list);
        
        csi2_dev->completed_buffers++;
        
        /* Update tail */
        csi2_dev->ring_tail = (csi2_dev->ring_tail + 1) % RING_SIZE;
    }
}

static void csi2_irq_work_handler(struct work_struct *work)
{
    struct amd_csi2_device *csi2_dev = container_of(work, 
                                                   struct amd_csi2_device, 
                                                   irq_work);
    struct amd_csi2_buffer *buf, *tmp;
    unsigned long flags;
    LIST_HEAD(done_list);
    LIST_HEAD(pending_list);
    
    /* Step 1: Process completed buffers with ring_lock only */
    spin_lock_irqsave(&csi2_dev->ring_lock, flags);
    __csi2_process_completed_buffers_locked(csi2_dev, &done_list);
    spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
    
    /* Step 2: Complete buffers without holding any locks */
    list_for_each_entry_safe(buf, tmp, &done_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_DONE);
    }
    
    /* Step 3: Get pending buffers with buffer_lock only */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_splice_init(&csi2_dev->buffer_list, &pending_list);
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    
    /* Step 4: Queue pending buffers to ring */
    list_for_each_entry_safe(buf, tmp, &pending_list, list) {
        list_del(&buf->list);
        csi2_queue_buffer_to_ring(csi2_dev, buf);
    }
}

static irqreturn_t amd_csi2_irq_handler(int irq, void *dev_id)
{
    struct amd_csi2_device *csi2_dev = dev_id;
    u32 status;
    
    status = csi2_read_reg(csi2_dev, CSI2_INT_STATUS_REG);
    if (!status)
        return IRQ_NONE;
    
    /* Clear interrupts */
    csi2_write_reg(csi2_dev, CSI2_INT_STATUS_REG, status);
    
    if (status & CSI2_INT_FRAME_DONE) {
        /* Schedule work to process buffers */
        schedule_work(&csi2_dev->irq_work);
    }
    
    if (status & CSI2_INT_ERROR) {
        csi2_dev->errors++;
        csi2_dbg(1, &csi2_dev->pdev->dev, "Device error interrupt\n");
    }
    
    return IRQ_HANDLED;
}

static void csi2_watchdog_work(struct work_struct *work)
{
    struct amd_csi2_device *csi2_dev = container_of(to_delayed_work(work),
                                                   struct amd_csi2_device,
                                                   watchdog_work);
    u32 frame_count;
    
    if (!csi2_dev->streaming)
        return;
    
    frame_count = csi2_read_reg(csi2_dev, CSI2_FRAME_COUNT_REG);
    
    csi2_dbg(4, &csi2_dev->pdev->dev,
             "Watchdog: frames=%u, queued=%u, completed=%u, errors=%u\n",
             frame_count, csi2_dev->queued_buffers,
             csi2_dev->completed_buffers, csi2_dev->errors);
    
    /* Reschedule */
    schedule_delayed_work(&csi2_dev->watchdog_work, HZ);
}

/* VB2 operations */
static int queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
                      unsigned int *nplanes, unsigned int sizes[],
                      struct device *alloc_devs[])
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vq);
    
    /* Ensure minimum buffer count */
    if (vq->max_num_buffers + *nbuffers < 4)
        *nbuffers = 4 - vq->max_num_buffers;
    
    if (*nbuffers > RING_SIZE)
        *nbuffers = RING_SIZE;
    
    *nplanes = 1;
    sizes[0] = csi2_dev->format.sizeimage;
    
    csi2_dbg(3, &csi2_dev->pdev->dev,
             "queue_setup: buffers=%u, size=%u\n", *nbuffers, sizes[0]);
    
    return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct amd_csi2_buffer *buf = container_of(vbuf, struct amd_csi2_buffer, vb2_buf);
    
    if (vb2_plane_size(vb, 0) < csi2_dev->format.sizeimage) {
        csi2_dbg(1, &csi2_dev->pdev->dev,
                "Buffer too small (%lu < %u)\n",
                vb2_plane_size(vb, 0), csi2_dev->format.sizeimage);
        return -EINVAL;
    }
    
    vb2_set_plane_payload(vb, 0, csi2_dev->format.sizeimage);
    buf->index = vb->index;
    
    return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct amd_csi2_buffer *buf = container_of(vbuf, struct amd_csi2_buffer, vb2_buf);
    unsigned long flags;
    
    /* If streaming, queue directly to ring */
    if (csi2_dev->streaming) {
        csi2_queue_buffer_to_ring(csi2_dev, buf);
    } else {
        /* Otherwise, add to pending list */
        spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
        list_add_tail(&buf->list, &csi2_dev->buffer_list);
        spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    }
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vq);
    struct amd_csi2_buffer *buf, *tmp;
    unsigned long flags;
    LIST_HEAD(pending_list);
    int ret;
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "start_streaming: count=%u\n", count);
    
    /* Reset device */
    csi2_write_reg(csi2_dev, CSI2_CORE_CONFIG_REG, CSI2_CORE_CONFIG_RESET);
    msleep(10);
    
    /* Configure format */
    csi2_write_reg(csi2_dev, CSI2_WIDTH_REG, csi2_dev->format.width);
    csi2_write_reg(csi2_dev, CSI2_HEIGHT_REG, csi2_dev->format.height);
    csi2_write_reg(csi2_dev, CSI2_FPS_REG, 30);
    
    /* Setup ring buffer */
    csi2_write_reg(csi2_dev, CSI2_RING_BASE_LOW, lower_32_bits(csi2_dev->ring_dma));
    csi2_write_reg(csi2_dev, CSI2_RING_BASE_HIGH, upper_32_bits(csi2_dev->ring_dma));
    csi2_write_reg(csi2_dev, CSI2_RING_SIZE_REG, RING_SIZE);
    
    /* Initialize ring */
    csi2_dev->ring_head = 0;
    csi2_dev->ring_tail = 0;
    csi2_dev->ring->head = 0;
    csi2_dev->ring->tail = 0;
    
    /* Enable ring */
    csi2_write_reg(csi2_dev, CSI2_RING_CTRL_REG, RING_CTRL_ENABLE);
    msleep(10);
    
    /* Check ring status */
    ret = csi2_read_reg(csi2_dev, CSI2_RING_STATUS_REG);
    if (!(ret & RING_STATUS_READY)) {
        csi2_dbg(1, &csi2_dev->pdev->dev, "Ring buffer not ready\n");
        ret = -EIO;
        goto err_stop;
    }
    
    /* Reset statistics */
    csi2_dev->sequence = 0;
    csi2_dev->queued_buffers = 0;
    csi2_dev->completed_buffers = 0;
    csi2_dev->dropped_frames = 0;
    csi2_dev->errors = 0;
    
    /* Get all pending buffers */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_splice_init(&csi2_dev->buffer_list, &pending_list);
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    
    /* Queue all pending buffers */
    list_for_each_entry_safe(buf, tmp, &pending_list, list) {
        list_del(&buf->list);
        csi2_queue_buffer_to_ring(csi2_dev, buf);
    }
    
    /* Enable interrupts */
    csi2_write_reg(csi2_dev, CSI2_INT_ENABLE_REG, 
                   CSI2_INT_FRAME_DONE | CSI2_INT_ERROR);
    csi2_write_reg(csi2_dev, CSI2_GLOBAL_INT_EN_REG, 1);
    
    /* Start streaming */
    csi2_dev->streaming = true;
    csi2_write_reg(csi2_dev, CSI2_CORE_CONFIG_REG, CSI2_CORE_CONFIG_ENABLE);
    
    /* Start watchdog */
    schedule_delayed_work(&csi2_dev->watchdog_work, HZ);
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "Streaming started successfully\n");
    
    return 0;
    
err_stop:
    /* Return all buffers */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_splice_init(&pending_list, &csi2_dev->buffer_list);
    while (!list_empty(&csi2_dev->buffer_list)) {
        buf = list_first_entry(&csi2_dev->buffer_list,
                              struct amd_csi2_buffer, list);
        list_del(&buf->list);
        spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
        vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
        spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    }
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    
    return ret;
}

static void stop_streaming(struct vb2_queue *vq)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vq);
    struct amd_csi2_buffer *buf, *tmp;
    unsigned long flags;
    LIST_HEAD(error_list);
    int i;
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "stop_streaming\n");
    
    csi2_dev->streaming = false;
    
    /* Cancel watchdog */
    cancel_delayed_work_sync(&csi2_dev->watchdog_work);
    
    /* Cancel any pending IRQ work */
    cancel_work_sync(&csi2_dev->irq_work);
    
    /* Disable device */
    csi2_write_reg(csi2_dev, CSI2_GLOBAL_INT_EN_REG, 0);
    csi2_write_reg(csi2_dev, CSI2_CORE_CONFIG_REG, 0);
    csi2_write_reg(csi2_dev, CSI2_RING_CTRL_REG, RING_CTRL_RESET);
    
    /* Collect all buffers maintaining lock order */
    
    /* First, get all queued buffers */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_splice_init(&csi2_dev->buffer_list, &error_list);
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    
    /* Then, get all active buffers from ring */
    spin_lock_irqsave(&csi2_dev->ring_lock, flags);
    for (i = 0; i < RING_SIZE; i++) {
        if (csi2_dev->active_buffers[i]) {
            buf = csi2_dev->active_buffers[i];
            csi2_dev->active_buffers[i] = NULL;
            list_add_tail(&buf->list, &error_list);
        }
    }
    spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
    
    /* Return all buffers without holding any locks */
    list_for_each_entry_safe(buf, tmp, &error_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    
    csi2_dbg(3, &csi2_dev->pdev->dev,
             "Streaming stopped - queued=%u, completed=%u, dropped=%u\n",
             csi2_dev->queued_buffers, csi2_dev->completed_buffers,
             csi2_dev->dropped_frames);
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

/* V4L2 ioctl operations */
static int amd_csi2_querycap(struct file *file, void *priv,
                            struct v4l2_capability *cap)
{
    struct amd_csi2_device *csi2_dev = video_drvdata(file);
    
    strscpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
    strscpy(cap->card, DEVICE_NAME, sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s",
            pci_name(csi2_dev->pdev));
    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
    
    return 0;
}

static int amd_csi2_enum_fmt_vid_cap(struct file *file, void *priv,
                                    struct v4l2_fmtdesc *f)
{
    if (f->index != 0)
        return -EINVAL;
    
    f->pixelformat = V4L2_PIX_FMT_YUYV;
    strscpy(f->description, "YUYV 4:2:2", sizeof(f->description));
    
    return 0;
}

static int amd_csi2_g_fmt_vid_cap(struct file *file, void *priv,
                                 struct v4l2_format *f)
{
    struct amd_csi2_device *csi2_dev = video_drvdata(file);
    
    f->fmt.pix = csi2_dev->format;
    
    return 0;
}

static int amd_csi2_try_fmt_vid_cap(struct file *file, void *priv,
                                   struct v4l2_format *f)
{
    if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV)
        f->fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    
    f->fmt.pix.width = clamp_t(u32, f->fmt.pix.width, 320, 3840);
    f->fmt.pix.height = clamp_t(u32, f->fmt.pix.height, 240, 2160);
    f->fmt.pix.bytesperline = f->fmt.pix.width * 2;
    f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;
    f->fmt.pix.field = V4L2_FIELD_NONE;
    f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    
    return 0;
}

static int amd_csi2_s_fmt_vid_cap(struct file *file, void *priv,
                                 struct v4l2_format *f)
{
    struct amd_csi2_device *csi2_dev = video_drvdata(file);
    
    if (vb2_is_busy(&csi2_dev->queue))
        return -EBUSY;
    
    amd_csi2_try_fmt_vid_cap(file, priv, f);
    csi2_dev->format = f->fmt.pix;
    
    return 0;
}

static int amd_csi2_enum_input(struct file *file, void *priv,
                              struct v4l2_input *inp)
{
    if (inp->index != 0)
        return -EINVAL;
    
    inp->type = V4L2_INPUT_TYPE_CAMERA;
    strscpy(inp->name, "Camera", sizeof(inp->name));
    
    return 0;
}

static int amd_csi2_g_input(struct file *file, void *priv, unsigned int *i)
{
    *i = 0;
    return 0;
}

static int amd_csi2_s_input(struct file *file, void *priv, unsigned int i)
{
    if (i != 0)
        return -EINVAL;
    return 0;
}

static const struct v4l2_ioctl_ops amd_csi2_ioctl_ops = {
    .vidioc_querycap = amd_csi2_querycap,
    .vidioc_enum_fmt_vid_cap = amd_csi2_enum_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap = amd_csi2_g_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap = amd_csi2_try_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap = amd_csi2_s_fmt_vid_cap,
    .vidioc_enum_input = amd_csi2_enum_input,
    .vidioc_g_input = amd_csi2_g_input,
    .vidioc_s_input = amd_csi2_s_input,
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

/* Sysfs attributes */
static ssize_t stats_show(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    struct amd_csi2_device *csi2_dev = pci_get_drvdata(pdev);
    u32 frame_count, error_count;

    if (!csi2_dev)
        return -ENODEV;

    frame_count = csi2_read_reg(csi2_dev, CSI2_FRAME_COUNT_REG);
    error_count = csi2_read_reg(csi2_dev, CSI2_ERROR_COUNT_REG);

    return sprintf(buf,
                  "frames: %u\n"
                  "queued: %u\n"
                  "completed: %u\n"
                  "dropped: %u\n"
                  "errors: %u\n"
                  "hw_frames: %u\n"
                  "hw_errors: %u\n",
                  csi2_dev->sequence,
                  csi2_dev->queued_buffers,
                  csi2_dev->completed_buffers,
                  csi2_dev->dropped_frames,
                  csi2_dev->errors,
                  frame_count,
                  error_count);
}
static DEVICE_ATTR_RO(stats);

static ssize_t ring_status_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    struct amd_csi2_device *csi2_dev = pci_get_drvdata(pdev);
    u32 status;
    int count;

    if (!csi2_dev)
        return -ENODEV;

    status = csi2_read_reg(csi2_dev, CSI2_RING_STATUS_REG);
    count = ring_count(csi2_dev->ring_head, csi2_dev->ring_tail, RING_SIZE);

    return sprintf(buf,
                  "status: 0x%08x\n"
                  "ready: %s\n"
                  "full: %s\n"
                  "empty: %s\n"
                  "head: %u\n"
                  "tail: %u\n"
                  "count: %d\n",
                  status,
                  (status & RING_STATUS_READY) ? "yes" : "no",
                  (status & RING_STATUS_FULL) ? "yes" : "no",
                  (status & RING_STATUS_EMPTY) ? "yes" : "no",
                  csi2_dev->ring_head,
                  csi2_dev->ring_tail,
                  count);
}
static DEVICE_ATTR_RO(ring_status);

static struct attribute *amd_csi2_attrs[] = {
    &dev_attr_stats.attr,
    &dev_attr_ring_status.attr,
    NULL
};

static const struct attribute_group amd_csi2_group = {
    .attrs = amd_csi2_attrs,
};

static const struct attribute_group *amd_csi2_groups[] = {
    &amd_csi2_group,
    NULL,
};

/* Device initialization */
static int amd_csi2_init_video_device(struct amd_csi2_device *csi2_dev)
{
    struct video_device *vdev;
    struct vb2_queue *q;
    int ret;

    /* Initialize VB2 queue */
    q = &csi2_dev->queue;
    q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    q->io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
    q->drv_priv = csi2_dev;
    q->ops = &amd_csi2_vb2_ops;
    q->mem_ops = &vb2_dma_contig_memops;
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    q->lock = &csi2_dev->mutex;
    q->dev = &csi2_dev->pdev->dev;
    q->buf_struct_size = sizeof(struct amd_csi2_buffer);

    ret = vb2_queue_init(q);
    if (ret) {
        csi2_dbg(1, &csi2_dev->pdev->dev, "vb2_queue_init failed: %d\n", ret);
        return ret;
    }

    /* Allocate video device */
    vdev = video_device_alloc();
    if (!vdev) {
        csi2_dbg(1, &csi2_dev->pdev->dev, "video_device_alloc failed\n");
        ret = -ENOMEM;
        goto err_vb2_release;
    }

    csi2_dev->vdev = vdev;

    /* Initialize video device */
    strscpy(vdev->name, DEVICE_NAME, sizeof(vdev->name));
    vdev->fops = &amd_csi2_fops;
    vdev->ioctl_ops = &amd_csi2_ioctl_ops;
    vdev->release = video_device_release;
    vdev->queue = q;
    vdev->lock = &csi2_dev->mutex;
    vdev->v4l2_dev = &csi2_dev->v4l2_dev;
    vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    vdev->vfl_type = VFL_TYPE_VIDEO;
    vdev->vfl_dir = VFL_DIR_RX;
    video_set_drvdata(vdev, csi2_dev);

    /* Set default format */
    csi2_dev->format.pixelformat = V4L2_PIX_FMT_YUYV;
    csi2_dev->format.width = DEFAULT_WIDTH;
    csi2_dev->format.height = DEFAULT_HEIGHT;
    csi2_dev->format.field = V4L2_FIELD_NONE;
    csi2_dev->format.colorspace = V4L2_COLORSPACE_SRGB;
    csi2_dev->format.bytesperline = csi2_dev->format.width * 2;
    csi2_dev->format.sizeimage = csi2_dev->format.bytesperline * csi2_dev->format.height;

    /* Register video device */
    ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
    if (ret) {
        csi2_dbg(1, &csi2_dev->pdev->dev, "video_register_device failed: %d\n", ret);
        goto err_video_release;
    }

    csi2_dbg(3, &csi2_dev->pdev->dev, "Video device registered as %s\n",
             video_device_node_name(vdev));

    return 0;

err_video_release:
    video_device_release(vdev);
    csi2_dev->vdev = NULL;
err_vb2_release:
    vb2_queue_release(q);
    return ret;
}

/* PCI driver */
static int amd_csi2_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct amd_csi2_device *csi2_dev;
    int ret;
    int irq_vectors = 0;
    
    csi2_dbg(3, &pdev->dev, "Probing AMD CSI-2 device\n");
    
    /* Allocate device structure */
    csi2_dev = kzalloc(sizeof(*csi2_dev), GFP_KERNEL);
    if (!csi2_dev)
        return -ENOMEM;
    
    pci_set_drvdata(pdev, csi2_dev);
    csi2_dev->pdev = pdev;
    csi2_dev->irq = -1;  /* Initialize to invalid IRQ */
    
    /* Initialize locks and lists */
    mutex_init(&csi2_dev->mutex);
    spin_lock_init(&csi2_dev->buffer_lock);
    spin_lock_init(&csi2_dev->ring_lock);
    INIT_LIST_HEAD(&csi2_dev->buffer_list);
    INIT_LIST_HEAD(&csi2_dev->done_list);
    INIT_WORK(&csi2_dev->irq_work, csi2_irq_work_handler);
    INIT_DELAYED_WORK(&csi2_dev->watchdog_work, csi2_watchdog_work);
    
#ifdef CONFIG_LOCKDEP
    /* Set lock classes for lockdep */
    lockdep_set_class(&csi2_dev->buffer_lock, &buffer_lock_key);
    lockdep_set_class(&csi2_dev->ring_lock, &ring_lock_key);
#endif
    
    /* Register V4L2 device */
    ret = v4l2_device_register(&pdev->dev, &csi2_dev->v4l2_dev);
    if (ret) {
        csi2_dbg(1, &pdev->dev, "v4l2_device_register failed: %d\n", ret);
        goto err_free_dev;
    }
    
    /* Enable PCI device */
    ret = pci_enable_device(pdev);
    if (ret) {
        csi2_dbg(1, &pdev->dev, "pci_enable_device failed: %d\n", ret);
        goto err_v4l2_unregister;
    }
    
    /* Enable bus mastering */
    pci_set_master(pdev);
    
    /* Set DMA mask */
    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (ret) {
        ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (ret) {
            csi2_dbg(1, &pdev->dev, "DMA mask setup failed: %d\n", ret);
            goto err_pci_disable;
        }
    }
    
    /* Request PCI regions */
    ret = pci_request_regions(pdev, DRIVER_NAME);
    if (ret) {
        csi2_dbg(1, &pdev->dev, "pci_request_regions failed: %d\n", ret);
        goto err_pci_disable;
    }
    
    /* Map MMIO */
    csi2_dev->mmio_base = pci_ioremap_bar(pdev, 0);
    if (!csi2_dev->mmio_base) {
        csi2_dbg(1, &pdev->dev, "pci_ioremap_bar failed\n");
        ret = -ENOMEM;
        goto err_release_regions;
    }
    
    /* Allocate ring buffer */
    csi2_dev->ring = dma_alloc_coherent(&pdev->dev, sizeof(struct ring_buffer),
                                        &csi2_dev->ring_dma, GFP_KERNEL);
    if (!csi2_dev->ring) {
        csi2_dbg(1, &pdev->dev, "Failed to allocate ring buffer\n");
        ret = -ENOMEM;
        goto err_iounmap;
    }
    
    /* Initialize ring buffer */
    memset(csi2_dev->ring, 0, sizeof(struct ring_buffer));
    csi2_dev->ring->magic = 0x43534932;  /* "CSI2" */
    csi2_dev->ring->version = 0x00020000;
    csi2_dev->ring->size = RING_SIZE;
    csi2_dev->ring->entry_size = sizeof(struct ring_entry);
    
    /* Try MSI-X first, then MSI, then legacy */
    irq_vectors = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSIX);
    if (irq_vectors < 0) {
        irq_vectors = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
    }
    
    if (irq_vectors > 0) {
        /* MSI or MSI-X */
        csi2_dev->irq = pci_irq_vector(pdev, 0);
        ret = request_irq(csi2_dev->irq, amd_csi2_irq_handler, 0,
                          DRIVER_NAME, csi2_dev);
        if (ret) {
            csi2_dbg(1, &pdev->dev, "Failed to request MSI/MSI-X IRQ %d: %d\n", 
                     csi2_dev->irq, ret);
            goto err_free_irq_vectors;
        }
        csi2_dbg(3, &pdev->dev, "Using MSI/MSI-X IRQ %d\n", csi2_dev->irq);
    } else {
        /* Legacy interrupt */
        csi2_dev->irq = pdev->irq;
        if (csi2_dev->irq <= 0) {
            csi2_dbg(1, &pdev->dev, "No valid IRQ assigned\n");
            ret = -ENODEV;
            goto err_free_ring;
        }
        ret = request_irq(csi2_dev->irq, amd_csi2_irq_handler, IRQF_SHARED,
                          DRIVER_NAME, csi2_dev);
        if (ret) {
            csi2_dbg(1, &pdev->dev, "Failed to request legacy IRQ %d: %d\n", 
                     csi2_dev->irq, ret);
            goto err_free_ring;
        }
        csi2_dbg(3, &pdev->dev, "Using legacy IRQ %d\n", csi2_dev->irq);
    }
    
    /* Reset device */
    csi2_write_reg(csi2_dev, CSI2_CORE_CONFIG_REG, CSI2_CORE_CONFIG_RESET);
    csi2_write_reg(csi2_dev, CSI2_RING_CTRL_REG, RING_CTRL_RESET);
    csi2_write_reg(csi2_dev, CSI2_GLOBAL_INT_EN_REG, 0);
    csi2_write_reg(csi2_dev, CSI2_INT_STATUS_REG, 0xFFFFFFFF);
    
    /* Create sysfs attributes */
    ret = sysfs_create_groups(&pdev->dev.kobj, amd_csi2_groups);
    if (ret) {
        csi2_dbg(1, &pdev->dev, "Failed to create sysfs attributes\n");
        goto err_free_irq;
    }
    
    /* Initialize video device */
    ret = amd_csi2_init_video_device(csi2_dev);
    if (ret) {
        csi2_dbg(1, &pdev->dev, "Failed to initialize video device: %d\n", ret);
        goto err_remove_sysfs;
    }
    
    csi2_dbg(3, &pdev->dev, "AMD CSI-2 device probed successfully (v2.1)\n");
    
    return 0;
    
err_remove_sysfs:
    sysfs_remove_groups(&pdev->dev.kobj, amd_csi2_groups);
err_free_irq:
    if (csi2_dev->irq > 0) {
        free_irq(csi2_dev->irq, csi2_dev);
        csi2_dev->irq = -1;
    }
err_free_irq_vectors:
    if (irq_vectors > 0)
        pci_free_irq_vectors(pdev);
err_free_ring:
    dma_free_coherent(&pdev->dev, sizeof(struct ring_buffer),
                      csi2_dev->ring, csi2_dev->ring_dma);
err_iounmap:
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
    
    csi2_dbg(3, &pdev->dev, "Removing AMD CSI-2 device\n");
    
    /* Cancel watchdog */
    cancel_delayed_work_sync(&csi2_dev->watchdog_work);
    
    /* Cancel IRQ work */
    cancel_work_sync(&csi2_dev->irq_work);
    
    /* Disable device */
    if (csi2_dev->mmio_base) {
        csi2_write_reg(csi2_dev, CSI2_GLOBAL_INT_EN_REG, 0);
        csi2_write_reg(csi2_dev, CSI2_CORE_CONFIG_REG, 0);
        csi2_write_reg(csi2_dev, CSI2_RING_CTRL_REG, RING_CTRL_RESET);
    }
    
    /* Unregister video device */
    if (csi2_dev->vdev) {
        video_unregister_device(csi2_dev->vdev);
        csi2_dev->vdev = NULL;
    }
    
    /* Remove sysfs attributes */
    sysfs_remove_groups(&pdev->dev.kobj, amd_csi2_groups);
    
    /* Unregister V4L2 device */
    v4l2_device_unregister(&csi2_dev->v4l2_dev);
    
    /* Free IRQ - MUST be done before freeing IRQ vectors */
    if (csi2_dev->irq > 0) {
        synchronize_irq(csi2_dev->irq);  /* Wait for any pending IRQ handlers */
        free_irq(csi2_dev->irq, csi2_dev);
        csi2_dev->irq = -1;
    }
    
    /* Free IRQ vectors if using MSI/MSI-X */
    if (pci_dev_msi_enabled(pdev)) {
        pci_free_irq_vectors(pdev);
    }
    
    /* Free ring buffer */
    if (csi2_dev->ring)
        dma_free_coherent(&pdev->dev, sizeof(struct ring_buffer),
                         csi2_dev->ring, csi2_dev->ring_dma);
    
    /* Unmap MMIO */
    if (csi2_dev->mmio_base)
        iounmap(csi2_dev->mmio_base);
    
    /* Release PCI regions */
    pci_release_regions(pdev);
    
    /* Disable PCI device */
    pci_disable_device(pdev);
    
    /* Clear driver data */
    pci_set_drvdata(pdev, NULL);
    
    /* Free device structure */
    kfree(csi2_dev);
    
    csi2_dbg(3, &pdev->dev, "AMD CSI-2 device removed successfully\n");
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

MODULE_AUTHOR("AMD");
MODULE_DESCRIPTION("AMD CSI-2 RX PCIe V4L2 Driver with Ring Buffer (v2.1)");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("2.1");
