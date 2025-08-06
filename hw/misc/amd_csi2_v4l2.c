/*
 * AMD CSI-2 RX PCIe V4L2 Driver with Ring Buffer
 * Version: 2.5 - Fixed buffer management for stop_streaming
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
#include <linux/ktime.h>
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
#define CSI2_TIMER_STATS_REG            0x310
#define CSI2_TIMER_DRIFT_REG            0x314
#define CSI2_TIMER_CONFIG_REG           0x318

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
#define RING_SIZE      32
#define MIN_BUFFERS    3

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
    bool in_ring;  /* Track if buffer is in ring */
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
    struct workqueue_struct *work_queue;
    struct work_struct irq_work;
    
    /* Performance tuning */
    bool first_frame;
    u32 frame_interval_ns;
    
    /* Timing statistics */
    ktime_t last_irq_time;
    ktime_t min_interval;
    ktime_t max_interval;
    ktime_t total_interval;
    u32 interval_count;
    u32 late_frames;
    ktime_t stream_start_time;
    
    /* Work processing time statistics */
    ktime_t min_work_time;
    ktime_t max_work_time;
    ktime_t total_work_time;
    u32 work_count;
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

static int ring_free_count(u32 head, u32 tail, u32 size)
{
    int count = ring_count(head, tail, size);
    return size - count - 1; /* -1 because we can't fill completely */
}

/* Debug helper - dump all buffer states */
static void csi2_dump_all_buffer_states(struct amd_csi2_device *csi2_dev, const char *context)
{
    struct amd_csi2_buffer *buf;
    unsigned long flags;
    int i;
    
    csi2_dbg(2, &csi2_dev->pdev->dev, "=== Buffer State Dump [%s] ===\n", context);
    
    /* Pending list */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_for_each_entry(buf, &csi2_dev->buffer_list, list) {
        csi2_dbg(2, &csi2_dev->pdev->dev, 
                 "  Pending: buf[%d] state=%d\n",
                 buf->index, buf->vb2_buf.vb2_buf.state);
    }
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    
    /* Ring buffer */
    spin_lock_irqsave(&csi2_dev->ring_lock, flags);
    for (i = 0; i < RING_SIZE; i++) {
        if (csi2_dev->active_buffers[i]) {
            buf = csi2_dev->active_buffers[i];
            csi2_dbg(2, &csi2_dev->pdev->dev,
                     "  Ring[%d]: buf[%d] state=%d in_ring=%d\n",
                     i, buf->index, buf->vb2_buf.vb2_buf.state,
                     buf->in_ring);
        }
    }
    spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
    
    csi2_dbg(2, &csi2_dev->pdev->dev, "  Head=%u, Tail=%u\n",
             csi2_dev->ring_head, csi2_dev->ring_tail);
    csi2_dbg(2, &csi2_dev->pdev->dev, "========================\n");
}

/* Must be called with ring_lock held */
static int __csi2_queue_buffer_to_ring_locked(struct amd_csi2_device *csi2_dev,
                                              struct amd_csi2_buffer *buf)
{
    struct vb2_buffer *vb = &buf->vb2_buf.vb2_buf;
    u32 head = csi2_dev->ring_head;
    u32 hw_tail;
    
    /* Read the current tail from hardware register */
    hw_tail = csi2_read_reg(csi2_dev, CSI2_RING_TAIL_REG);
    
    /* Update cached tail if different */
    if (hw_tail != csi2_dev->ring_tail) {
        csi2_dbg(4, &csi2_dev->pdev->dev, 
                 "Tail updated: cached=%u, hw=%u\n", 
                 csi2_dev->ring_tail, hw_tail);
        csi2_dev->ring_tail = hw_tail;
    }
    
    /* Check if ring is full using the updated tail */
    if (ring_is_full(head, csi2_dev->ring_tail, RING_SIZE)) {
        csi2_dbg(2, &csi2_dev->pdev->dev, 
                 "Ring buffer full, cannot queue buffer %d (head=%u, tail=%u)\n",
                 buf->index, head, csi2_dev->ring_tail);
        return -ENOSPC;
    }
    
    /* Fill ring entry */
    csi2_dev->ring->entries[head].dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
    csi2_dev->ring->entries[head].size = vb2_plane_size(vb, 0);
    csi2_dev->ring->entries[head].flags = 0;
    csi2_dev->ring->entries[head].timestamp = 0;
    
    /* Store buffer reference */
    csi2_dev->active_buffers[head] = buf;
    buf->in_ring = true;
    
    /* Update head */
    csi2_dev->ring_head = (head + 1) % RING_SIZE;
    csi2_dev->ring->head = csi2_dev->ring_head;
    
    /* Memory barrier to ensure ring update is visible */
    wmb();
    
    csi2_dev->queued_buffers++;
    
    csi2_dbg(4, &csi2_dev->pdev->dev, 
             "Queued buffer %d to ring slot %u (dma: 0x%llx)\n",
             buf->index, head, csi2_dev->ring->entries[head].dma_addr);
    
    return 0;
}

static void csi2_queue_buffer_to_ring(struct amd_csi2_device *csi2_dev,
                                     struct amd_csi2_buffer *buf)
{
    unsigned long flags;
    int ret;
    
    spin_lock_irqsave(&csi2_dev->ring_lock, flags);
    ret = __csi2_queue_buffer_to_ring_locked(csi2_dev, buf);
    spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
    
    if (ret) {
        /* Queue failed, return to buffer list */
        spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
        list_add(&buf->list, &csi2_dev->buffer_list);
        spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    }
}

/* Must be called with ring_lock held */
static void __csi2_process_completed_buffers_locked(struct amd_csi2_device *csi2_dev,
                                                   struct list_head *done_list)
{
    u32 tail;
    u32 old_tail = csi2_dev->ring_tail;
    int processed = 0;
    
    /* Check streaming state */
    if (!csi2_dev->streaming) {
        csi2_dbg(3, &csi2_dev->pdev->dev,
                 "Process buffers called but streaming stopped\n");
        return;
    }
    
    /* Memory barrier to ensure we see the latest tail */
    rmb();
    
    /* Read tail directly from hardware register */
    tail = csi2_read_reg(csi2_dev, CSI2_RING_TAIL_REG);
    
    csi2_dbg(3, &csi2_dev->pdev->dev, 
             "Processing buffers: hw_tail=%u, cached_tail=%u, head=%u\n",
             tail, csi2_dev->ring_tail, csi2_dev->ring_head);
    
    /* Process all completed buffers */
    while (csi2_dev->ring_tail != tail) {
        struct amd_csi2_buffer *buf;
        struct ring_entry *entry;
        struct vb2_v4l2_buffer *vbuf;
        u64 timestamp_ns;
        
        buf = csi2_dev->active_buffers[csi2_dev->ring_tail];
        entry = &csi2_dev->ring->entries[csi2_dev->ring_tail];
        
        if (!buf) {
            csi2_dbg(1, &csi2_dev->pdev->dev, 
                     "ERROR: No buffer in slot %u\n", csi2_dev->ring_tail);
            csi2_dev->ring_tail = (csi2_dev->ring_tail + 1) % RING_SIZE;
            continue;
        }

        csi2_dbg(4, &csi2_dev->pdev->dev,
                 "Processing completed buffer at slot %u (index=%d)\n",
                 csi2_dev->ring_tail, buf->index);

        vbuf = &buf->vb2_buf;
        
        /* Clear active buffer and update state */
        csi2_dev->active_buffers[csi2_dev->ring_tail] = NULL;
        buf->in_ring = false;

        /* Set timestamp */
        timestamp_ns = entry->timestamp;
        vbuf->vb2_buf.timestamp = timestamp_ns;

        /* Set metadata */
        vbuf->sequence = csi2_dev->sequence++;
        vbuf->field = V4L2_FIELD_NONE;
        vbuf->flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

        /* Check buffer completion status */
        if (entry->flags & 0x01) {  /* Completion flag */
            vb2_set_plane_payload(&vbuf->vb2_buf, 0, csi2_dev->format.sizeimage);
            csi2_dbg(4, &csi2_dev->pdev->dev, 
                     "Buffer %d marked as complete\n", buf->index);
        } else {
            csi2_dbg(2, &csi2_dev->pdev->dev, 
                     "WARNING: Buffer %d not marked as complete (flags=0x%x)\n", 
                     buf->index, entry->flags);
        }

        /* Add to done list */
        list_add_tail(&buf->list, done_list);
        processed++;

        csi2_dev->completed_buffers++;

        /* Update tail */
        csi2_dev->ring_tail = (csi2_dev->ring_tail + 1) % RING_SIZE;
        
        csi2_dbg(4, &csi2_dev->pdev->dev,
                 "Updated ring_tail to %u\n", csi2_dev->ring_tail);
    }
    
    if (processed > 0) {
        csi2_dbg(3, &csi2_dev->pdev->dev,
                 "Processed %d buffers (tail: %u -> %u)\n",
                 processed, old_tail, csi2_dev->ring_tail);
    } else {
        csi2_dbg(4, &csi2_dev->pdev->dev,
                 "No buffers to process (tail=%u)\n", csi2_dev->ring_tail);
    }
}

static void csi2_irq_work_handler(struct work_struct *work)
{
    struct amd_csi2_device *csi2_dev = container_of(work, 
                                                   struct amd_csi2_device, 
                                                   irq_work);
    struct amd_csi2_buffer *buf;
    unsigned long flags;
    LIST_HEAD(done_list);
    int processed = 0;
    int queued = 0;
    ktime_t work_start, work_end;
    s64 work_time_us;
    
    /* Check streaming state early */
    if (!csi2_dev->streaming) {
        csi2_dbg(3, &csi2_dev->pdev->dev, 
                 "Work handler called but streaming already stopped\n");
        return;
    }
    
    work_start = ktime_get();
    csi2_dev->work_count++;
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "IRQ work handler called (#%d)\n", 
             csi2_dev->work_count);
    
    /* Process completed buffers */
    spin_lock_irqsave(&csi2_dev->ring_lock, flags);
    if (csi2_dev->streaming) {  /* Double check */
        __csi2_process_completed_buffers_locked(csi2_dev, &done_list);
    }
    spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
    
    /* Complete buffers to userspace */
    while (!list_empty(&done_list)) {
        buf = list_first_entry(&done_list, struct amd_csi2_buffer, list);
        list_del_init(&buf->list);
        
        /* Check streaming state before completing */
        if (!csi2_dev->streaming) {
            vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
        } else {
            csi2_dbg(4, &csi2_dev->pdev->dev, 
                     "Completing buffer %d to userspace\n", buf->index);
            vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_DONE);
        }
        processed++;
    }
    
    /* Try to queue pending buffers */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    while (!list_empty(&csi2_dev->buffer_list) && csi2_dev->streaming) {
        buf = list_first_entry(&csi2_dev->buffer_list,
                              struct amd_csi2_buffer, list);
        list_del_init(&buf->list);
        spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
        
        /* Try to queue to ring */
        spin_lock_irqsave(&csi2_dev->ring_lock, flags);
        if (__csi2_queue_buffer_to_ring_locked(csi2_dev, buf) < 0) {
            spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
            /* Ring full, put back to list and stop */
            spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
            list_add(&buf->list, &csi2_dev->buffer_list);
            spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
            break;
        }
        queued++;
        spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
        
        spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    }
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    
    work_end = ktime_get();
    work_time_us = ktime_to_us(ktime_sub(work_end, work_start));
    
    /* Update work time statistics */
    if (csi2_dev->work_count == 1) {
        csi2_dev->min_work_time = ns_to_ktime(work_time_us * 1000);
        csi2_dev->max_work_time = ns_to_ktime(work_time_us * 1000);
    } else {
        if (work_time_us < ktime_to_us(csi2_dev->min_work_time))
            csi2_dev->min_work_time = ns_to_ktime(work_time_us * 1000);
        if (work_time_us > ktime_to_us(csi2_dev->max_work_time))
            csi2_dev->max_work_time = ns_to_ktime(work_time_us * 1000);
    }
    csi2_dev->total_work_time = ktime_add(csi2_dev->total_work_time,
                                         ns_to_ktime(work_time_us * 1000));
    
    if (work_time_us > 5000) { /* 5ms warning threshold */
        csi2_dbg(2, &csi2_dev->pdev->dev,
                 "Work processing took %lld us (too long!)\n",
                 work_time_us);
    }
    
    csi2_dbg(3, &csi2_dev->pdev->dev, 
             "IRQ work completed: processed=%d, queued=%d, time=%lld us\n", 
             processed, queued, work_time_us);
}

static irqreturn_t amd_csi2_irq_handler(int irq, void *dev_id)
{
    struct amd_csi2_device *csi2_dev = dev_id;
    u32 status;
    ktime_t now;
    s64 interval_ns;
    static int irq_count = 0;
    
    status = csi2_read_reg(csi2_dev, CSI2_INT_STATUS_REG);
    if (!status)
        return IRQ_NONE;
    
    now = ktime_get();
    
    /* Measure interrupt interval */
    if (csi2_dev->last_irq_time != 0) {
        interval_ns = ktime_to_ns(ktime_sub(now, csi2_dev->last_irq_time));
        
        /* Update statistics */
        if (csi2_dev->interval_count == 0) {
            csi2_dev->min_interval = ns_to_ktime(interval_ns);
            csi2_dev->max_interval = ns_to_ktime(interval_ns);
        } else {
            if (interval_ns < ktime_to_ns(csi2_dev->min_interval))
                csi2_dev->min_interval = ns_to_ktime(interval_ns);
            if (interval_ns > ktime_to_ns(csi2_dev->max_interval))
                csi2_dev->max_interval = ns_to_ktime(interval_ns);
        }
        
        csi2_dev->total_interval = ktime_add(csi2_dev->total_interval, 
                                           ns_to_ktime(interval_ns));
        csi2_dev->interval_count++;
        
        /* Detect late frames (>40ms for 30fps) */
        if (interval_ns > 40000000) {
            csi2_dev->late_frames++;
            csi2_dbg(2, &csi2_dev->pdev->dev, 
                     "Late frame detected: interval=%lld ms (expected ~33ms)\n",
                     interval_ns / 1000000);
        }
        
        /* Periodic statistics output */
        if (csi2_dev->interval_count % 100 == 0) {
            s64 avg_ns = ktime_to_ns(csi2_dev->total_interval) / 
                         csi2_dev->interval_count;
            csi2_dbg(1, &csi2_dev->pdev->dev, 
                     "IRQ timing stats: avg=%lld ms, min=%lld ms, max=%lld ms, late=%u\n",
                     avg_ns / 1000000,
                     ktime_to_ns(csi2_dev->min_interval) / 1000000,
                     ktime_to_ns(csi2_dev->max_interval) / 1000000,
                     csi2_dev->late_frames);
        }
    }
    
    csi2_dev->last_irq_time = now;
    
    irq_count++;
    csi2_dbg(3, &csi2_dev->pdev->dev, 
             "IRQ #%d: status=0x%08x, interval=%lld us\n", 
             irq_count, status, interval_ns / 1000);
    
    /* Clear interrupts */
    csi2_write_reg(csi2_dev, CSI2_INT_STATUS_REG, status);
    
    if (status & CSI2_INT_FRAME_DONE) {
        csi2_dbg(3, &csi2_dev->pdev->dev, 
                 "Frame done interrupt, scheduling work\n");
        /* Schedule work to process buffers */
        queue_work(csi2_dev->work_queue, &csi2_dev->irq_work);
    }
    
    if (status & CSI2_INT_ERROR) {
        csi2_dev->errors++;
        csi2_dbg(1, &csi2_dev->pdev->dev, "Device error interrupt\n");
    }
    
    return IRQ_HANDLED;
}

/* Return all buffers helper */
static void csi2_return_all_buffers(struct amd_csi2_device *csi2_dev,
                                   enum vb2_buffer_state state)
{
    struct amd_csi2_buffer *buf;
    unsigned long flags;
    int i;
    int returned = 0;
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "Returning all buffers with state %d\n", state);
    
    /* 1. Pending list buffers */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    while (!list_empty(&csi2_dev->buffer_list)) {
        buf = list_first_entry(&csi2_dev->buffer_list,
                              struct amd_csi2_buffer, list);
        list_del_init(&buf->list);
        spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
        
        csi2_dbg(4, &csi2_dev->pdev->dev,
                 "Returning pending buffer %d\n", buf->index);
        vb2_buffer_done(&buf->vb2_buf.vb2_buf, state);
        returned++;
        
        spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    }
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    
    /* 2. Ring buffer active buffers */
    for (i = 0; i < RING_SIZE; i++) {
        spin_lock_irqsave(&csi2_dev->ring_lock, flags);
        buf = csi2_dev->active_buffers[i];
        csi2_dev->active_buffers[i] = NULL;
        spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
        
        if (buf && buf->in_ring) {
            buf->in_ring = false;
            csi2_dbg(4, &csi2_dev->pdev->dev,
                     "Returning active buffer %d from slot %d\n",
                     buf->index, i);
            vb2_buffer_done(&buf->vb2_buf.vb2_buf, state);
            returned++;
        }
    }
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "Returned %d buffers total\n", returned);
}

/* VB2 operations */
static int queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
                      unsigned int *nplanes, unsigned int sizes[],
                      struct device *alloc_devs[])
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vq);
    unsigned int size = csi2_dev->format.sizeimage;
    
    csi2_dbg(4, &csi2_dev->pdev->dev,
             "queue_setup: nbuffers=%u, nplanes=%u\n", 
             *nbuffers, *nplanes);
    
    if (*nplanes) {
        if (*nplanes != 1)
            return -EINVAL;
        if (sizes[0] < size)
            return -EINVAL;
        return 0;
    }
    
    *nplanes = 1;
    sizes[0] = size;
    
    /* Ensure minimum buffers for smooth operation */
    if (*nbuffers < MIN_BUFFERS)
        *nbuffers = MIN_BUFFERS;
    
    /* Don't allocate more than ring size */
    if (*nbuffers > RING_SIZE)
        *nbuffers = RING_SIZE;
    
    if (alloc_devs)
        alloc_devs[0] = &csi2_dev->pdev->dev;
    
    csi2_dbg(3, &csi2_dev->pdev->dev,
             "Allocating %u buffers, size=%u\n", 
             *nbuffers, sizes[0]);
    
    return 0;
}

static int buffer_init(struct vb2_buffer *vb)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct amd_csi2_buffer *buf = container_of(vbuf, struct amd_csi2_buffer, vb2_buf);
    
    INIT_LIST_HEAD(&buf->list);
    buf->index = vb->index;
    buf->in_ring = false;
    
    csi2_dbg(4, &csi2_dev->pdev->dev,
             "buffer_init: index=%d, type=%d\n",
             vb->index, vb->type);
    
    return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct amd_csi2_buffer *buf = container_of(vbuf, struct amd_csi2_buffer, vb2_buf);
    unsigned long size;
    
    size = csi2_dev->format.sizeimage;
    
    if (vb2_plane_size(vb, 0) < size) {
        csi2_dbg(1, &csi2_dev->pdev->dev,
                "Buffer too small (%lu < %lu)\n",
                vb2_plane_size(vb, 0), size);
        return -EINVAL;
    }
    
    vb2_set_plane_payload(vb, 0, size);

    /* Initialize buffer metadata */
    vbuf->flags = 0;
    vbuf->field = V4L2_FIELD_NONE;
    buf->index = vb->index;
    
    /* Clear buffer to avoid "corrupted data" warning */
    if (csi2_dev->first_frame) {
        void *vaddr = vb2_plane_vaddr(vb, 0);
        if (vaddr)
            memset(vaddr, 0, size);
    }
    
    return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct amd_csi2_buffer *buf = container_of(vbuf, struct amd_csi2_buffer, vb2_buf);
    unsigned long flags;
    
    /* Ensure list is properly initialized */
    if (!list_empty(&buf->list)) {
        csi2_dbg(2, &csi2_dev->pdev->dev,
                 "WARNING: Buffer %d list not empty in buffer_queue\n",
                 buf->index);
        INIT_LIST_HEAD(&buf->list);
    }
    
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_add_tail(&buf->list, &csi2_dev->buffer_list);
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    
    csi2_dbg(4, &csi2_dev->pdev->dev,
             "Buffer %d queued\n", buf->index);
    
    if (csi2_dev->streaming) {
        /* Trigger work to process pending buffers */
        queue_work(csi2_dev->work_queue, &csi2_dev->irq_work);
    }
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vq);
    struct amd_csi2_buffer *buf, *tmp;
    unsigned long flags;
    LIST_HEAD(pending_list);
    int ret;
    int queued = 0;
    u32 timer_config;
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "start_streaming: count=%u\n", count);
    
    /* Reset device */
    csi2_write_reg(csi2_dev, CSI2_CORE_CONFIG_REG, CSI2_CORE_CONFIG_RESET);
    msleep(10);
    
    /* Configure format */
    csi2_write_reg(csi2_dev, CSI2_WIDTH_REG, csi2_dev->format.width);
    csi2_write_reg(csi2_dev, CSI2_HEIGHT_REG, csi2_dev->format.height);
    csi2_write_reg(csi2_dev, CSI2_FPS_REG, DEFAULT_FPS);
    
    /* Configure timer (disable batch interrupts for accurate timing) */
    timer_config = 0;  /* bit 0: realtime clock, bit 1: batch interrupts */
    csi2_write_reg(csi2_dev, CSI2_TIMER_CONFIG_REG, timer_config);
    
    /* Setup ring buffer */
    csi2_write_reg(csi2_dev, CSI2_RING_BASE_LOW, lower_32_bits(csi2_dev->ring_dma));
    csi2_write_reg(csi2_dev, CSI2_RING_BASE_HIGH, upper_32_bits(csi2_dev->ring_dma));
    csi2_write_reg(csi2_dev, CSI2_RING_SIZE_REG, RING_SIZE);
    
    /* Initialize ring and statistics */
    csi2_dev->ring_head = 0;
    csi2_dev->ring_tail = 0;
    csi2_dev->ring->head = 0;
    csi2_dev->ring->tail = 0;
    csi2_dev->sequence = 0;
    csi2_dev->queued_buffers = 0;
    csi2_dev->completed_buffers = 0;
    csi2_dev->dropped_frames = 0;
    csi2_dev->errors = 0;
    csi2_dev->first_frame = true;
    csi2_dev->frame_interval_ns = 1000000000 / DEFAULT_FPS;
    
    /* Reset timing statistics */
    csi2_dev->last_irq_time = 0;
    csi2_dev->min_interval = 0;
    csi2_dev->max_interval = 0;
    csi2_dev->total_interval = 0;
    csi2_dev->interval_count = 0;
    csi2_dev->late_frames = 0;
    csi2_dev->stream_start_time = ktime_get();
    
    /* Reset work timing statistics */
    csi2_dev->min_work_time = 0;
    csi2_dev->max_work_time = 0;
    csi2_dev->total_work_time = 0;
    csi2_dev->work_count = 0;
    
    /* Clear active buffers array */
    memset(csi2_dev->active_buffers, 0, sizeof(csi2_dev->active_buffers));
    
    /* Enable ring */
    csi2_write_reg(csi2_dev, CSI2_RING_CTRL_REG, RING_CTRL_ENABLE);
    msleep(10);
    
    /* Check ring status */
    ret = csi2_read_reg(csi2_dev, CSI2_RING_STATUS_REG);
    if (!(ret & RING_STATUS_READY)) {
        csi2_dbg(1, &csi2_dev->pdev->dev, "Ring buffer not ready\n");
        ret = -EIO;
        goto err_return_buffers;
    }
    
    /* Get all pending buffers */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_splice_init(&csi2_dev->buffer_list, &pending_list);
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    
    /* Queue buffers to ring */
    spin_lock_irqsave(&csi2_dev->ring_lock, flags);
    list_for_each_entry_safe(buf, tmp, &pending_list, list) {
        if (__csi2_queue_buffer_to_ring_locked(csi2_dev, buf) < 0) {
            /* Ring full, stop queuing */
            break;
        }
        list_del(&buf->list);
        queued++;
    }
    spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);
    
    /* Return any unqueued buffers to pending list */
    if (!list_empty(&pending_list)) {
        spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
        list_splice(&pending_list, &csi2_dev->buffer_list);
        spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
    }
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "Queued %d buffers to ring\n", queued);
    
    /* Enable interrupts */
    csi2_write_reg(csi2_dev, CSI2_INT_ENABLE_REG, 
                   CSI2_INT_FRAME_DONE | CSI2_INT_ERROR);
    csi2_write_reg(csi2_dev, CSI2_GLOBAL_INT_EN_REG, 1);
    
    /* Start streaming */
    csi2_dev->streaming = true;
    csi2_write_reg(csi2_dev, CSI2_CORE_CONFIG_REG, CSI2_CORE_CONFIG_ENABLE);
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "Streaming started successfully\n");
    
    return 0;
    
err_return_buffers:
    /* Return all buffers */
    spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    list_splice_init(&pending_list, &csi2_dev->buffer_list);
    while (!list_empty(&csi2_dev->buffer_list)) {
        buf = list_first_entry(&csi2_dev->buffer_list,
                              struct amd_csi2_buffer, list);
        list_del(&buf->list);
        spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);
        vb2_buffer_done(&buf->vb2_buf.vb2_buf, VB2_BUF_STATE_QUEUED);
        spin_lock_irqsave(&csi2_dev->buffer_lock, flags);
    }
    spin_unlock_irqrestore(&csi2_dev->buffer_lock, flags);    

    return ret;
}

static void stop_streaming(struct vb2_queue *vq)
{
    struct amd_csi2_device *csi2_dev = vb2_get_drv_priv(vq);
    
    csi2_dbg(3, &csi2_dev->pdev->dev, "stop_streaming: START\n");
    csi2_dump_all_buffer_states(csi2_dev, "stop_streaming start");
    
    /* 1. Immediately stop streaming flag */
    csi2_dev->streaming = false;
    
    /* 2. Completely stop hardware */
    csi2_write_reg(csi2_dev, CSI2_GLOBAL_INT_EN_REG, 0);
    csi2_write_reg(csi2_dev, CSI2_INT_ENABLE_REG, 0);
    csi2_write_reg(csi2_dev, CSI2_CORE_CONFIG_REG, 0);
    csi2_write_reg(csi2_dev, CSI2_RING_CTRL_REG, RING_CTRL_RESET);
    
    /* 3. Wait for any pending IRQ handler to complete */
    if (csi2_dev->irq > 0) {
        synchronize_irq(csi2_dev->irq);
    }
    
    /* 4. Cancel and flush all work */
    cancel_work_sync(&csi2_dev->irq_work);
    flush_workqueue(csi2_dev->work_queue);
    
    /* 5. Memory barrier */
    mb();
    
    /* 6. Return all buffers */
    csi2_return_all_buffers(csi2_dev, VB2_BUF_STATE_ERROR);
    
    /* 7. Final state dump */
    csi2_dump_all_buffer_states(csi2_dev, "stop_streaming end");
    
    /* 8. Print final statistics */
    if (csi2_dev->interval_count > 0) {
        s64 avg_interval = ktime_to_ns(csi2_dev->total_interval) / 
                          csi2_dev->interval_count;
        csi2_dbg(2, &csi2_dev->pdev->dev,
                 "Final IRQ timing stats: avg=%lld ms, min=%lld ms, max=%lld ms, late=%u/%u\n",
                 avg_interval / 1000000,
                 ktime_to_ns(csi2_dev->min_interval) / 1000000,
                 ktime_to_ns(csi2_dev->max_interval) / 1000000,
                 csi2_dev->late_frames, csi2_dev->interval_count);
    }
    
    if (csi2_dev->work_count > 0) {
        s64 avg_work_time = ktime_to_us(csi2_dev->total_work_time) / 
                           csi2_dev->work_count;
        csi2_dbg(2, &csi2_dev->pdev->dev,
                 "Work timing stats: avg=%lld us, min=%lld us, max=%lld us\n",
                 avg_work_time,
                 ktime_to_us(csi2_dev->min_work_time),
                 ktime_to_us(csi2_dev->max_work_time));
    }
    
    csi2_dbg(3, &csi2_dev->pdev->dev,
             "stop_streaming: END - queued=%u, completed=%u, dropped=%u\n",
             csi2_dev->queued_buffers, csi2_dev->completed_buffers,
             csi2_dev->dropped_frames);
}

static const struct vb2_ops amd_csi2_vb2_ops = {
    .queue_setup = queue_setup,
    .buf_init = buffer_init,
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

static int amd_csi2_vidioc_reqbufs(struct file *file, void *priv,
                                   struct v4l2_requestbuffers *rb)
{
    struct amd_csi2_device *csi2_dev = video_drvdata(file);
    
    csi2_dbg(3, &csi2_dev->pdev->dev,
             "VIDIOC_REQBUFS: count=%d, type=%d, memory=%d\n",
             rb->count, rb->type, rb->memory);
    
    /* Clear reserved fields */
    memset(rb->reserved, 0, sizeof(rb->reserved));
    
    if (rb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    /* Limit excessive buffer requests */
    if (rb->count > 32) {
        csi2_dbg(2, &csi2_dev->pdev->dev,
                 "Limiting buffer count from %d to 32\n", rb->count);
        rb->count = 32;
    }
    
    /* Set capabilities */
    rb->capabilities = V4L2_BUF_CAP_SUPPORTS_MMAP | 
                      V4L2_BUF_CAP_SUPPORTS_DMABUF |
                      V4L2_BUF_CAP_SUPPORTS_ORPHANED_BUFS;
    
    return vb2_ioctl_reqbufs(file, priv, rb);
}

static int amd_csi2_vidioc_querybuf(struct file *file, void *priv,
                                   struct v4l2_buffer *b)
{
    /* Clear reserved fields */
    b->reserved2 = 0;
    b->reserved = 0;
    
    return vb2_ioctl_querybuf(file, priv, b);
}

static int amd_csi2_vidioc_g_parm(struct file *file, void *priv,
                                 struct v4l2_streamparm *parm)
{
    if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    
    parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    parm->parm.capture.timeperframe.numerator = 1;
    parm->parm.capture.timeperframe.denominator = DEFAULT_FPS;
    parm->parm.capture.capturemode = 0;
    parm->parm.capture.extendedmode = 0;
    parm->parm.capture.readbuffers = MIN_BUFFERS;
    
    return 0;
}

static int amd_csi2_vidioc_s_parm(struct file *file, void *priv,
                                 struct v4l2_streamparm *parm)
{
    /* Fixed framerate for now */
    return amd_csi2_vidioc_g_parm(file, priv, parm);
}

static int amd_csi2_enum_framesizes(struct file *file, void *priv,
                                   struct v4l2_frmsizeenum *fsize)
{
    if (fsize->index > 0)
        return -EINVAL;
    
    if (fsize->pixel_format != V4L2_PIX_FMT_YUYV)
        return -EINVAL;
    
    fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    fsize->discrete.width = DEFAULT_WIDTH;
    fsize->discrete.height = DEFAULT_HEIGHT;
    
    return 0;
}

static int amd_csi2_enum_frameintervals(struct file *file, void *priv,
                                       struct v4l2_frmivalenum *fival)
{
    if (fival->index > 0)
        return -EINVAL;
    
    if (fival->pixel_format != V4L2_PIX_FMT_YUYV)
        return -EINVAL;
    
    if (fival->width != DEFAULT_WIDTH || fival->height != DEFAULT_HEIGHT)
        return -EINVAL;
    
    fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
    fival->discrete.numerator = 1;
    fival->discrete.denominator = DEFAULT_FPS;
    
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
    .vidioc_reqbufs = amd_csi2_vidioc_reqbufs,
    .vidioc_querybuf = amd_csi2_vidioc_querybuf,
    .vidioc_qbuf = vb2_ioctl_qbuf,
    .vidioc_dqbuf = vb2_ioctl_dqbuf,
    .vidioc_prepare_buf = vb2_ioctl_prepare_buf,
    .vidioc_streamon = vb2_ioctl_streamon,
    .vidioc_streamoff = vb2_ioctl_streamoff,
    .vidioc_enum_framesizes = amd_csi2_enum_framesizes,
    .vidioc_enum_frameintervals = amd_csi2_enum_frameintervals,
    .vidioc_g_parm = amd_csi2_vidioc_g_parm,
    .vidioc_s_parm = amd_csi2_vidioc_s_parm,
};

static const struct v4l2_file_operations amd_csi2_fops = {
    .owner = THIS_MODULE,
    .open = v4l2_fh_open,
    .release = vb2_fop_release,
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
    unsigned long flags;
    int ring_occupancy;

    if (!csi2_dev)
        return -ENODEV;

    frame_count = csi2_read_reg(csi2_dev, CSI2_FRAME_COUNT_REG);
    error_count = csi2_read_reg(csi2_dev, CSI2_ERROR_COUNT_REG);
    
    spin_lock_irqsave(&csi2_dev->ring_lock, flags);
    ring_occupancy = ring_count(csi2_dev->ring_head, csi2_dev->ring_tail, RING_SIZE);
    spin_unlock_irqrestore(&csi2_dev->ring_lock, flags);

    return sprintf(buf,
                  "frames: %u\n"
                  "queued: %u\n"
                  "completed: %u\n"
                  "dropped: %u\n"
                  "errors: %u\n"
                  "hw_frames: %u\n"
                  "hw_errors: %u\n"
                  "ring_occupancy: %d/%d\n",
                  csi2_dev->sequence,
                  csi2_dev->queued_buffers,
                  csi2_dev->completed_buffers,
                  csi2_dev->dropped_frames,
                  csi2_dev->errors,
                  frame_count,
                  error_count,
                  ring_occupancy, RING_SIZE);
}
static DEVICE_ATTR_RO(stats);

static ssize_t timing_stats_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    struct amd_csi2_device *csi2_dev = pci_get_drvdata(pdev);
    s64 avg_ns = 0;
    s64 avg_work_us = 0;
    u32 actual_fps_int, actual_fps_frac;
    
    if (!csi2_dev || csi2_dev->interval_count == 0)
        return -ENODEV;
    
    avg_ns = ktime_to_ns(csi2_dev->total_interval) / csi2_dev->interval_count;
    
    if (csi2_dev->work_count > 0) {
        avg_work_us = ktime_to_us(csi2_dev->total_work_time) / csi2_dev->work_count;
    }
    
    /* Calculate FPS using integer arithmetic */
    if (avg_ns > 0) {
        /* FPS = 1000000000 / avg_ns */
        /* To get two decimal places, multiply by 100 first */
        u64 fps_x100 = 100000000000ULL / avg_ns;
	actual_fps_int = fps_x100 / 100;
 	actual_fps_frac = fps_x100 % 100;
   } else {
       actual_fps_int = 0;
       actual_fps_frac = 0;
   }

   return sprintf(buf,
                 "IRQ Timing Statistics:\n"
                 "Average interval: %lld.%03lld ms\n"
                 "Minimum interval: %lld.%03lld ms\n"
                 "Maximum interval: %lld.%03lld ms\n"
                 "Late frames: %u\n"
                 "Total samples: %u\n"
                 "Expected FPS: %d\n"
                 "Actual FPS: %u.%02u\n"
                 "\nWork Processing Statistics:\n"
                 "Average time: %lld us\n"
                 "Minimum time: %lld us\n"
                 "Maximum time: %lld us\n"
                 "Work count: %u\n",
                 avg_ns / 1000000, (avg_ns / 1000) % 1000,
                 ktime_to_ns(csi2_dev->min_interval) / 1000000,
                 (ktime_to_ns(csi2_dev->min_interval) / 1000) % 1000,
                 ktime_to_ns(csi2_dev->max_interval) / 1000000,
                 (ktime_to_ns(csi2_dev->max_interval) / 1000) % 1000,
                 csi2_dev->late_frames,
                 csi2_dev->interval_count,
                 DEFAULT_FPS,
                 actual_fps_int, actual_fps_frac,
                 avg_work_us,
                 ktime_to_us(csi2_dev->min_work_time),
                 ktime_to_us(csi2_dev->max_work_time),
                 csi2_dev->work_count);
}
static DEVICE_ATTR_RO(timing_stats);

static ssize_t timing_reset_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
   struct pci_dev *pdev = to_pci_dev(dev);
   struct amd_csi2_device *csi2_dev = pci_get_drvdata(pdev);

   if (!csi2_dev)
       return -ENODEV;

   csi2_dev->last_irq_time = 0;
   csi2_dev->min_interval = 0;
   csi2_dev->max_interval = 0;
   csi2_dev->total_interval = 0;
   csi2_dev->interval_count = 0;
   csi2_dev->late_frames = 0;

   csi2_dev->min_work_time = 0;
   csi2_dev->max_work_time = 0;
   csi2_dev->total_work_time = 0;
   csi2_dev->work_count = 0;

   return count;
}
static DEVICE_ATTR_WO(timing_reset);

static struct attribute *amd_csi2_attrs[] = {
   &dev_attr_stats.attr,
   &dev_attr_timing_stats.attr,
   &dev_attr_timing_reset.attr,
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
   q->io_modes = VB2_MMAP | VB2_DMABUF;
   q->drv_priv = csi2_dev;
   q->ops = &amd_csi2_vb2_ops;
   q->mem_ops = &vb2_dma_contig_memops;
   q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
                       V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
   q->lock = &csi2_dev->mutex;
   q->dev = &csi2_dev->pdev->dev;
   q->buf_struct_size = sizeof(struct amd_csi2_buffer);
   q->gfp_flags = GFP_DMA32;
   q->allow_zero_bytesused = 0;
   q->max_num_buffers = 32;

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
   csi2_dev->irq = -1;

   /* Initialize locks and lists */
   mutex_init(&csi2_dev->mutex);
   spin_lock_init(&csi2_dev->buffer_lock);
   spin_lock_init(&csi2_dev->ring_lock);
   INIT_LIST_HEAD(&csi2_dev->buffer_list);
   INIT_WORK(&csi2_dev->irq_work, csi2_irq_work_handler);

#ifdef CONFIG_LOCKDEP
   /* Set lock classes for lockdep */
   lockdep_set_class(&csi2_dev->buffer_lock, &buffer_lock_key);
   lockdep_set_class(&csi2_dev->ring_lock, &ring_lock_key);
#endif

   /* Create dedicated workqueue with high priority */
   csi2_dev->work_queue = alloc_workqueue("amd-csi2",
                                         WQ_HIGHPRI | WQ_CPU_INTENSIVE | WQ_UNBOUND, 1);
   if (!csi2_dev->work_queue) {
       csi2_dbg(1, &pdev->dev, "Failed to create workqueue\n");
       ret = -ENOMEM;
       goto err_free_dev;
   }

   /* Register V4L2 device */
   ret = v4l2_device_register(&pdev->dev, &csi2_dev->v4l2_dev);
   if (ret) {
       csi2_dbg(1, &pdev->dev, "v4l2_device_register failed: %d\n", ret);
       goto err_destroy_wq;
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

   csi2_dbg(3, &pdev->dev, "AMD CSI-2 device probed successfully (v2.5)\n");

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
err_destroy_wq:
   destroy_workqueue(csi2_dev->work_queue);
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

   /* Free IRQ */
   if (csi2_dev->irq > 0) {
       synchronize_irq(csi2_dev->irq);
       free_irq(csi2_dev->irq, csi2_dev);
       csi2_dev->irq = -1;
   }

   /* Free IRQ vectors if using MSI/MSI-X */
   if (pci_dev_msi_enabled(pdev)) {
       pci_free_irq_vectors(pdev);
   }

   /* Destroy workqueue */
   if (csi2_dev->work_queue) {
       flush_workqueue(csi2_dev->work_queue);
       destroy_workqueue(csi2_dev->work_queue);
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
MODULE_DESCRIPTION("AMD CSI-2 RX PCIe V4L2 Driver with Ring Buffer (v2.5)");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("2.5");
