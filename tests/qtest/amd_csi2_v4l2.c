/*
 * AMD MIPI CSI-2 RX Subsystem V4L2 Driver
 * Version 4.0 - MSI-X TIMING AND INITIALIZATION FIX
 * 🎯 FIXED: Proper initialization sequence and MSI-X handling
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/videodev2.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/version.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#define DRIVER_NAME "amd_csi2_v4l2"
#define DRIVER_VERSION "4.0"

/* PCI Device IDs */
#define AMD_VENDOR_ID 0x1022
#define AMD_CSI2_DEVICE_ID 0xC901

/* Complete AMD CSI-2 Register Map (from spec) */
#define CSI2_REG_CORE_CONFIG           0x00
#define CSI2_REG_PROTOCOL_CONFIG       0x04
#define CSI2_REG_CORE_STATUS           0x10
#define CSI2_REG_GLOBAL_INT_ENABLE     0x20
#define CSI2_REG_ISR                   0x24
#define CSI2_REG_IER                   0x28
#define CSI2_REG_DYNAMIC_VC_SEL        0x2C
#define CSI2_REG_GENERIC_SHORT_PACKET  0x30
#define CSI2_REG_VCX_FRAME_ERROR       0x34
#define CSI2_REG_CLK_LANE_INFO         0x3C

/* Lane Information Registers */
#define CSI2_REG_LANE0_INFO            0x40
#define CSI2_REG_LANE1_INFO            0x44
#define CSI2_REG_LANE2_INFO            0x48
#define CSI2_REG_LANE3_INFO            0x4C

/* Test/Debug registers */
#define CSI2_REG_TEST_TRIGGER          0x50
#define CSI2_REG_DEBUG_CTRL            0x54
#define CSI2_REG_FORCE_INT             0x58

/* Image Information Registers */
#define CSI2_REG_IMG_INFO1_VC0         0x60
#define CSI2_REG_IMG_INFO2_VC0         0x64

/* D-PHY registers (0x1000 offset) */
#define CSI2_REG_DPHY_CONTROL          0x1000
#define CSI2_REG_DPHY_STATUS           0x1004
#define CSI2_REG_DPHY_HS_SETTLE        0x1008
#define CSI2_REG_DPHY_PLL_CTRL         0x100C
#define CSI2_REG_DPHY_PLL_STATUS       0x1010

/* Register bit definitions (from AMD spec) */
#define CORE_CONFIG_ENABLE             BIT(0)
#define CORE_CONFIG_SOFT_RESET         BIT(1)
#define CORE_CONFIG_FULL_RESET         BIT(2)

#define ISR_FRAME_RECEIVED             BIT(31)
#define ISR_VCX_FRAME_ERROR            BIT(30)
#define ISR_RX_SKEWCALHS               BIT(29)
#define ISR_YUV420_WC_ERROR            BIT(28)
#define ISR_PENDING_WRITE_FIFO         BIT(27)
#define ISR_WC_CORRUPTION              BIT(22)
#define ISR_INCORRECT_LANE_CONFIG      BIT(21)
#define ISR_SHORT_PACKET_FIFO_FULL     BIT(20)
#define ISR_SHORT_PACKET_FIFO_NOT_EMPTY BIT(19)
#define ISR_STREAM_LINE_BUFFER_FULL    BIT(18)
#define ISR_STOP_STATE                 BIT(17)
#define ISR_SOT_ERROR                  BIT(13)
#define ISR_SOT_SYNC_ERROR             BIT(12)
#define ISR_ECC_2_BIT_ERROR            BIT(11)
#define ISR_ECC_1_BIT_ERROR            BIT(10)
#define ISR_CRC_ERROR                  BIT(9)
#define ISR_UNSUPPORTED_DATA_TYPE      BIT(8)

/* Device configuration */
#define DEFAULT_WIDTH  1920
#define DEFAULT_HEIGHT 1080
#define DEFAULT_FPS    30
#define DEFAULT_FORMAT V4L2_PIX_FMT_YUYV
#define DEFAULT_FIELD  V4L2_FIELD_NONE
#define BYTES_PER_PIXEL 2

/* Buffer management */
#define MIN_BUFFERS    2
#define MAX_BUFFERS    8

/* MSI-X configuration */
#define AMD_CSI2_MSIX_VECTORS 8

struct amd_csi2_buffer {
    struct vb2_v4l2_buffer vb;
    struct list_head list;
    dma_addr_t dma_addr;
    size_t size;
};

struct amd_csi2_dev {
    struct pci_dev *pdev;
    struct v4l2_device v4l2_dev;
    struct video_device vdev;
    
    /* Hardware resources */
    void __iomem *mmio_base;
    int irq_vectors[AMD_CSI2_MSIX_VECTORS];
    int num_irq_vectors;
    int primary_irq;
    bool msix_enabled;
    
    /* Video buffer management */
    struct vb2_queue queue;
    struct list_head buf_list;
    struct amd_csi2_buffer *current_buffer;
    spinlock_t lock;
    struct mutex lock_mutex;
    
    /* Video format */
    struct v4l2_format format;
    struct v4l2_fract timeperframe;
    
    /* Device state */
    bool streaming;
    bool initialized;
    bool hardware_ready;
    u32 sequence;
    u32 frames_captured;
    
    /* Statistics and debugging */
    atomic_t total_interrupts;
    atomic_t frame_interrupts;
    atomic_t error_interrupts;
    atomic_t register_reads;
    atomic_t register_writes;
    ktime_t last_frame_time;
    ktime_t driver_start_time;
    u32 last_isr_value;
    u32 last_core_status;
    u32 init_sequence_step;
    
    /* Capture thread */
    struct task_struct *capture_thread;
    struct completion frame_completion;
    bool thread_should_stop;
    
    /* Hardware monitoring */
    struct timer_list hw_monitor_timer;
    struct work_struct hw_test_work;
    bool test_mode_active;
};

/* Hardware register access helpers */
static inline u32 csi2_read_reg(struct amd_csi2_dev *csi2_dev, u32 offset)
{
    u32 val = readl(csi2_dev->mmio_base + offset);
    atomic_inc(&csi2_dev->register_reads);
    return val;
}

static inline void csi2_write_reg(struct amd_csi2_dev *csi2_dev, u32 offset, u32 value)
{
    writel(value, csi2_dev->mmio_base + offset);
    wmb(); /* Ensure write completion */
    atomic_inc(&csi2_dev->register_writes);
}

/* 🎯 FIXED: Simplified hardware test */
static void amd_csi2_hardware_test(struct work_struct *work)
{
    struct amd_csi2_dev *csi2_dev = container_of(work, struct amd_csi2_dev, hw_test_work);
    u32 test_value = 0x12345678;
    
    dev_info(&csi2_dev->pdev->dev, "🧪 Hardware communication test\n");
    
    /* Enable debug mode */
    csi2_write_reg(csi2_dev, CSI2_REG_DEBUG_CTRL, 1);
    
    /* Write test trigger */
    csi2_write_reg(csi2_dev, CSI2_REG_TEST_TRIGGER, test_value);
    msleep(100);
    
    /* Read back */
    u32 readback = csi2_read_reg(csi2_dev, CSI2_REG_TEST_TRIGGER);
    
    dev_info(&csi2_dev->pdev->dev, 
             "📝 Test: wrote 0x%08x, read 0x%08x %s\n",
             test_value, readback, 
             (readback == test_value) ? "✅ OK" : "❌ FAIL");
    
    /* Force interrupt test */
    dev_info(&csi2_dev->pdev->dev, "🚀 Triggering force interrupt\n");
    csi2_write_reg(csi2_dev, CSI2_REG_FORCE_INT, 0xDEADBEEF);
    msleep(200);
    
    u32 isr = csi2_read_reg(csi2_dev, CSI2_REG_ISR);
    dev_info(&csi2_dev->pdev->dev, "📊 ISR after force: 0x%08x\n", isr);
    
    int total_interrupts = atomic_read(&csi2_dev->total_interrupts);
    dev_info(&csi2_dev->pdev->dev, 
             "📊 Total interrupts received: %d\n", total_interrupts);
}

/* Hardware verification */
static void amd_csi2_verify_hardware(struct amd_csi2_dev *csi2_dev)
{
    u32 core_config, core_status, global_int, ier;
    u32 dphy_status, dphy_pll_status;
    
    dev_info(&csi2_dev->pdev->dev, "🔍 Hardware Status Check\n");
    
    core_config = csi2_read_reg(csi2_dev, CSI2_REG_CORE_CONFIG);
    core_status = csi2_read_reg(csi2_dev, CSI2_REG_CORE_STATUS);
    global_int = csi2_read_reg(csi2_dev, CSI2_REG_GLOBAL_INT_ENABLE);
    ier = csi2_read_reg(csi2_dev, CSI2_REG_IER);
    dphy_status = csi2_read_reg(csi2_dev, CSI2_REG_DPHY_STATUS);
    dphy_pll_status = csi2_read_reg(csi2_dev, CSI2_REG_DPHY_PLL_STATUS);
    
    csi2_dev->last_core_status = core_status;
    
    dev_info(&csi2_dev->pdev->dev,
             "📊 Hardware Status:\n"
             "   Core Config: 0x%08x (enabled: %s)\n"
             "   Core Status: 0x%08x (packet count: %u)\n"
             "   Global Int: 0x%08x (enabled: %s)\n"
             "   IER: 0x%08x (frame int: %s)\n"
             "   D-PHY Status: 0x%08x (ready: %s)\n"
             "   D-PHY PLL: 0x%08x (locked: %s)\n",
             core_config, (core_config & CORE_CONFIG_ENABLE) ? "YES" : "NO",
             core_status, (core_status >> 16),
             global_int, (global_int & 0x1) ? "YES" : "NO",
             ier, (ier & ISR_FRAME_RECEIVED) ? "YES" : "NO",
             dphy_status, (dphy_status & 0x3) ? "YES" : "NO",
             dphy_pll_status, (dphy_pll_status & 0x3) ? "YES" : "NO");
    
    csi2_dev->hardware_ready = (core_config & CORE_CONFIG_ENABLE) &&
                              (dphy_status & 0x3) &&
                              (dphy_pll_status & 0x3);
    
    dev_info(&csi2_dev->pdev->dev, "🎯 Hardware Ready: %s\n",
             csi2_dev->hardware_ready ? "YES" : "NO");
}

/* Hardware monitoring timer */
static void amd_csi2_hw_monitor_timer(struct timer_list *timer)
{
    struct amd_csi2_dev *csi2_dev = from_timer(csi2_dev, timer, hw_monitor_timer);
    
    if (!csi2_dev->streaming) {
        return;
    }
    
    u32 core_status = csi2_read_reg(csi2_dev, CSI2_REG_CORE_STATUS);
    u32 isr = csi2_read_reg(csi2_dev, CSI2_REG_ISR);
    
    bool status_changed = (core_status != csi2_dev->last_core_status);
    bool isr_changed = (isr != csi2_dev->last_isr_value);
    
    if (status_changed || isr_changed) {
        dev_info(&csi2_dev->pdev->dev,
                 "📊 HW Monitor: Status=0x%08x, ISR=0x%08x, Interrupts=%d\n",
                 core_status, isr, atomic_read(&csi2_dev->total_interrupts));
        
        csi2_dev->last_core_status = core_status;
        csi2_dev->last_isr_value = isr;
    }
    
    /* Trigger test if no interrupts received for too long */
    if (atomic_read(&csi2_dev->total_interrupts) == 0 && !csi2_dev->test_mode_active) {
        csi2_dev->test_mode_active = true;
        schedule_work(&csi2_dev->hw_test_work);
    }
    
    /* Reschedule timer */
    if (csi2_dev->streaming) {
        mod_timer(&csi2_dev->hw_monitor_timer, jiffies + msecs_to_jiffies(3000));
    }
}

/* Enhanced MSI-X interrupt handler */
static irqreturn_t amd_csi2_interrupt(int irq, void *dev_id)
{
    struct amd_csi2_dev *csi2_dev = dev_id;
    u32 isr_status;
    unsigned long flags;
    int vector_idx = -1;
    bool handled = false;
    
    /* Find which vector triggered */
    for (int i = 0; i < csi2_dev->num_irq_vectors; i++) {
        if (csi2_dev->irq_vectors[i] == irq) {
            vector_idx = i;
            break;
        }
    }
    
    /* Update interrupt counters */
    atomic_inc(&csi2_dev->total_interrupts);
    int total_count = atomic_read(&csi2_dev->total_interrupts);
    
    dev_info(&csi2_dev->pdev->dev, 
             "🎉 MSI-X IRQ %d (Vector %d) - Interrupt #%d received! 🎉\n",
             irq, vector_idx, total_count);
    
    /* Special celebration for first interrupt */
    if (total_count == 1) {
        dev_info(&csi2_dev->pdev->dev, 
                 "🏆 FIRST INTERRUPT SUCCESS! MSI-X is working! 🏆\n");
    }
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    
    /* Read and process interrupt status */
    isr_status = csi2_read_reg(csi2_dev, CSI2_REG_ISR);
    csi2_dev->last_isr_value = isr_status;
    
    dev_info(&csi2_dev->pdev->dev, "📊 ISR Status: 0x%08x\n", isr_status);
    
    if (isr_status != 0) {
        handled = true;
        
        /* Process frame received interrupt */
        if (isr_status & ISR_FRAME_RECEIVED) {
            atomic_inc(&csi2_dev->frame_interrupts);
            complete(&csi2_dev->frame_completion);
            dev_info(&csi2_dev->pdev->dev, "🎬 Frame interrupt processed\n");
        }
        
        /* Process error interrupts */
        if (isr_status & (ISR_CRC_ERROR | ISR_ECC_1_BIT_ERROR | ISR_ECC_2_BIT_ERROR)) {
            atomic_inc(&csi2_dev->error_interrupts);
            dev_warn(&csi2_dev->pdev->dev, "⚠️  Error interrupt: 0x%08x\n", isr_status);
        }
        
        /* Clear all pending interrupts */
        csi2_write_reg(csi2_dev, CSI2_REG_ISR, isr_status);
        
    } else {
        dev_info(&csi2_dev->pdev->dev, "📌 Spurious interrupt (ISR=0)\n");
        handled = true; /* Still count as handled */
    }
    
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    return handled ? IRQ_HANDLED : IRQ_NONE;
}

/* 🎯 FIXED: Proper hardware initialization sequence */
static int amd_csi2_hw_init(struct amd_csi2_dev *csi2_dev)
{
    u32 val;
    
    dev_info(&csi2_dev->pdev->dev, "🔧 CSI-2 Hardware Initialization Sequence\n");
    
    /* Step 1: Soft reset and clear */
    dev_info(&csi2_dev->pdev->dev, "Step 1: Soft reset\n");
    csi2_write_reg(csi2_dev, CSI2_REG_CORE_CONFIG, CORE_CONFIG_SOFT_RESET);
    msleep(50);
    csi2_write_reg(csi2_dev, CSI2_REG_CORE_CONFIG, 0);
    msleep(50);
    
    /* Step 2: Configure D-PHY first */
    dev_info(&csi2_dev->pdev->dev, "Step 2: D-PHY configuration\n");
    csi2_write_reg(csi2_dev, CSI2_REG_DPHY_CONTROL, 0x1);
    csi2_write_reg(csi2_dev, CSI2_REG_DPHY_PLL_CTRL, 0x1);
    csi2_write_reg(csi2_dev, CSI2_REG_DPHY_HS_SETTLE, 0x20);
    msleep(100);
    
    /* Step 3: Configure protocol (4 lanes) */
    dev_info(&csi2_dev->pdev->dev, "Step 3: Protocol configuration\n");
    csi2_write_reg(csi2_dev, CSI2_REG_PROTOCOL_CONFIG, 0x3);
    
    /* Step 4: Enable core */
    dev_info(&csi2_dev->pdev->dev, "Step 4: Enable core\n");
    csi2_dev->init_sequence_step = 1;
    val = CORE_CONFIG_ENABLE;
    csi2_write_reg(csi2_dev, CSI2_REG_CORE_CONFIG, val);
    msleep(100);
    
    /* Step 5: Setup and enable interrupts carefully */
    dev_info(&csi2_dev->pdev->dev, "Step 5: Configure interrupts\n");
    
    /* Clear any pending interrupts */
    csi2_write_reg(csi2_dev, CSI2_REG_ISR, 0xFFFFFFFF);
    
    /* Enable frame received interrupt */
    csi2_dev->init_sequence_step = 2;
    csi2_write_reg(csi2_dev, CSI2_REG_IER, ISR_FRAME_RECEIVED);
    msleep(50);
    
    /* Enable global interrupts */
    csi2_dev->init_sequence_step = 3;
    csi2_write_reg(csi2_dev, CSI2_REG_GLOBAL_INT_ENABLE, 0x1);
    msleep(100);
    
    /* Step 6: Verify configuration */
    dev_info(&csi2_dev->pdev->dev, "Step 6: Verify configuration\n");
    amd_csi2_verify_hardware(csi2_dev);
    
    /* Initialize monitoring and work queues */
    timer_setup(&csi2_dev->hw_monitor_timer, amd_csi2_hw_monitor_timer, 0);
    INIT_WORK(&csi2_dev->hw_test_work, amd_csi2_hardware_test);
    
    /* Initialize counters */
    atomic_set(&csi2_dev->total_interrupts, 0);
    atomic_set(&csi2_dev->frame_interrupts, 0);
    atomic_set(&csi2_dev->error_interrupts, 0);
    atomic_set(&csi2_dev->register_reads, 0);
    atomic_set(&csi2_dev->register_writes, 0);
    
    csi2_dev->test_mode_active = false;
    csi2_dev->init_sequence_step = 4;  /* Complete */
    csi2_dev->driver_start_time = ktime_get();
    
    dev_info(&csi2_dev->pdev->dev, "✅ Hardware initialization complete\n");
    
    /* Schedule initial hardware test after a delay */
    schedule_work(&csi2_dev->hw_test_work);
    
    return 0;
}

/* 🎯 FIXED: Standard MSI-X setup without unnecessary table access */
static int amd_csi2_setup_msix(struct amd_csi2_dev *csi2_dev)
{
    struct pci_dev *pdev = csi2_dev->pdev;
    int ret, nvec, i;
    
    dev_info(&pdev->dev, "🚀 Standard MSI-X Setup\n");
    
    /* Clear any existing IRQ vectors */
    pci_free_irq_vectors(pdev);
    
    /* Get MSI-X vector count */
    nvec = pci_msix_vec_count(pdev);
    if (nvec < 1) {
        dev_err(&pdev->dev, "❌ No MSI-X vectors available\n");
        return -ENODEV;
    }
    
    dev_info(&pdev->dev, "📊 MSI-X vectors available: %d\n", nvec);
    
    /* Allocate MSI-X vectors */
    ret = pci_alloc_irq_vectors(pdev, 1, min(nvec, AMD_CSI2_MSIX_VECTORS), 
                                PCI_IRQ_MSIX | PCI_IRQ_AFFINITY);
    if (ret < 0) {
        dev_err(&pdev->dev, "❌ Failed to allocate MSI-X vectors: %d\n", ret);
        return ret;
    }
    
    csi2_dev->num_irq_vectors = ret;
    dev_info(&pdev->dev, "✅ Allocated %d MSI-X vectors\n", ret);
    
    /* Get IRQ numbers for all vectors */
    for (i = 0; i < csi2_dev->num_irq_vectors; i++) {
        csi2_dev->irq_vectors[i] = pci_irq_vector(pdev, i);
        dev_info(&pdev->dev, "📍 Vector %d -> IRQ %d\n", i, csi2_dev->irq_vectors[i]);
    }
    
    /* Register interrupt handler for primary vector (Vector 0) */
    csi2_dev->primary_irq = csi2_dev->irq_vectors[0];
    
    ret = request_irq(csi2_dev->primary_irq, amd_csi2_interrupt, 
                      IRQF_SHARED, DRIVER_NAME, csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to register IRQ %d: %d\n", 
                csi2_dev->primary_irq, ret);
        goto cleanup;
    }
    
    dev_info(&pdev->dev, "✅ IRQ handler registered: Vector 0 -> IRQ %d\n", 
             csi2_dev->primary_irq);
    
    csi2_dev->msix_enabled = true;
    
    dev_info(&pdev->dev, 
             "🎯 MSI-X Setup Complete:\n"
             "   Primary IRQ: %d\n"
             "   Total vectors: %d\n"
             "   Ready for interrupts!\n",
             csi2_dev->primary_irq, csi2_dev->num_irq_vectors);
    
    return 0;
    
cleanup:
    pci_free_irq_vectors(pdev);
    return ret;
}

static void amd_csi2_free_msix(struct amd_csi2_dev *csi2_dev)
{
    if (csi2_dev->msix_enabled) {
        dev_info(&csi2_dev->pdev->dev, "🧹 Freeing MSI-X resources\n");
        
        if (csi2_dev->primary_irq > 0) {
            free_irq(csi2_dev->primary_irq, csi2_dev);
        }
        
        pci_free_irq_vectors(csi2_dev->pdev);
        csi2_dev->msix_enabled = false;
        csi2_dev->primary_irq = 0;
        csi2_dev->num_irq_vectors = 0;
    }
}

/* Enhanced capture thread */
static int amd_csi2_capture_thread(void *data)
{
    struct amd_csi2_dev *csi2_dev = data;
    struct amd_csi2_buffer *buf;
    unsigned long flags;
    int frame_count = 0;
    
    dev_info(&csi2_dev->pdev->dev, "🎬 Capture Thread Started\n");
    
    /* Initial hardware verification after a delay */
    msleep(1000);
    amd_csi2_verify_hardware(csi2_dev);
    
    /* Start hardware monitoring */
    mod_timer(&csi2_dev->hw_monitor_timer, jiffies + msecs_to_jiffies(2000));
    
    while (!kthread_should_stop() && !csi2_dev->thread_should_stop) {
        /* Wait for frame completion with timeout */
        long ret = wait_for_completion_interruptible_timeout(&csi2_dev->frame_completion, 
                                                           msecs_to_jiffies(30000));
        
        if (ret <= 0) {
            if (ret == 0) {
                dev_warn(&csi2_dev->pdev->dev, 
                        "⏰ Frame timeout (30s) - triggering test\n");
                schedule_work(&csi2_dev->hw_test_work);
            }
            continue;
        }
        
        /* Reset completion for next frame */
        reinit_completion(&csi2_dev->frame_completion);
        
        spin_lock_irqsave(&csi2_dev->lock, flags);
        
        if (!list_empty(&csi2_dev->buf_list) && csi2_dev->streaming) {
            /* Process frame */
            buf = list_first_entry(&csi2_dev->buf_list, struct amd_csi2_buffer, list);
            list_del(&buf->list);
            
            /* Fill buffer metadata */
            buf->vb.vb2_buf.timestamp = ktime_get_ns();
            buf->vb.sequence = csi2_dev->sequence++;
            buf->vb.field = V4L2_FIELD_NONE;
            
            /* Set payload size */
            vb2_set_plane_payload(&buf->vb.vb2_buf, 0, 
                                  csi2_dev->format.fmt.pix.sizeimage);
            
            /* Complete buffer */
            vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
            
            frame_count++;
            csi2_dev->frames_captured++;
            csi2_dev->last_frame_time = ktime_get();
            
            dev_info(&csi2_dev->pdev->dev, 
                    "🎉 Frame #%d completed (%d bytes) 🎉\n",
                    frame_count, csi2_dev->format.fmt.pix.sizeimage);
        }
        
        spin_unlock_irqrestore(&csi2_dev->lock, flags);
        
        if (try_to_freeze())
            continue;
    }
    
    /* Stop hardware monitoring */
    timer_delete_sync(&csi2_dev->hw_monitor_timer);
    cancel_work_sync(&csi2_dev->hw_test_work);
    
    dev_info(&csi2_dev->pdev->dev, 
        "🎬 Capture thread stopped after %d frames\n", frame_count);
    return 0;
}

/* V4L2 buffer operations */
static int amd_csi2_queue_setup(struct vb2_queue *q,
                                unsigned int *nbuffers,
                                unsigned int *nplanes,
                                unsigned int sizes[],
                                struct device *alloc_devs[])
{
    struct amd_csi2_dev *csi2_dev = vb2_get_drv_priv(q);
    
    *nplanes = 1;
    sizes[0] = csi2_dev->format.fmt.pix.sizeimage;
    
    if (*nbuffers < MIN_BUFFERS)
        *nbuffers = MIN_BUFFERS;
    else if (*nbuffers > MAX_BUFFERS)
        *nbuffers = MAX_BUFFERS;
    
    return 0;
}

static int amd_csi2_buf_prepare(struct vb2_buffer *vb)
{
    struct amd_csi2_dev *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    unsigned long size = csi2_dev->format.fmt.pix.sizeimage;
    
    if (vb2_plane_size(vb, 0) < size) {
        dev_err(&csi2_dev->pdev->dev, "Buffer too small\n");
        return -EINVAL;
    }
    
    vb2_set_plane_payload(vb, 0, size);
    return 0;
}

static void amd_csi2_buf_queue(struct vb2_buffer *vb)
{
    struct amd_csi2_dev *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct amd_csi2_buffer *buf = container_of(vbuf, struct amd_csi2_buffer, vb);
    unsigned long flags;
    
    buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
    buf->size = vb2_plane_size(vb, 0);
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    list_add_tail(&buf->list, &csi2_dev->buf_list);
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
}

static int amd_csi2_start_streaming(struct vb2_queue *q, unsigned int count)
{
    struct amd_csi2_dev *csi2_dev = vb2_get_drv_priv(q);
    unsigned long flags;
    int ret;
    
    dev_info(&csi2_dev->pdev->dev, "🎬 Starting Streaming\n");
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    csi2_dev->streaming = true;
    csi2_dev->sequence = 0;
    csi2_dev->frames_captured = 0;
    csi2_dev->thread_should_stop = false;
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    init_completion(&csi2_dev->frame_completion);
    
    /* Start capture thread */
    csi2_dev->capture_thread = kthread_run(amd_csi2_capture_thread, csi2_dev,
                                          "csi2_capture");
    if (IS_ERR(csi2_dev->capture_thread)) {
        ret = PTR_ERR(csi2_dev->capture_thread);
        dev_err(&csi2_dev->pdev->dev, "❌ Failed to start capture thread: %d\n", ret);
        csi2_dev->streaming = false;
        return ret;
    }
    
    dev_info(&csi2_dev->pdev->dev, "✅ Streaming started\n");
    return 0;
}

static void amd_csi2_stop_streaming(struct vb2_queue *q)
{
    struct amd_csi2_dev *csi2_dev = vb2_get_drv_priv(q);
    struct amd_csi2_buffer *buf, *tmp;
    unsigned long flags;
    
    dev_info(&csi2_dev->pdev->dev, "⏹️  Stopping Streaming\n");
    
    /* Stop capture thread */
    if (csi2_dev->capture_thread) {
        csi2_dev->thread_should_stop = true;
        complete(&csi2_dev->frame_completion);
        kthread_stop(csi2_dev->capture_thread);
        csi2_dev->capture_thread = NULL;
    }
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    csi2_dev->streaming = false;
    
    /* Return all buffers */
    list_for_each_entry_safe(buf, tmp, &csi2_dev->buf_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    /* Final statistics */
    dev_info(&csi2_dev->pdev->dev, 
             "📊 Final Statistics:\n"
             "   Frames captured: %u\n"
             "   Total interrupts: %d\n"
             "   Frame interrupts: %d\n"
             "   Error interrupts: %d\n"
             "   Register reads: %d\n"
             "   Register writes: %d\n",
             csi2_dev->frames_captured,
             atomic_read(&csi2_dev->total_interrupts),
             atomic_read(&csi2_dev->frame_interrupts),
             atomic_read(&csi2_dev->error_interrupts),
             atomic_read(&csi2_dev->register_reads),
             atomic_read(&csi2_dev->register_writes));
}

static const struct vb2_ops amd_csi2_vb2_ops = {
    .queue_setup = amd_csi2_queue_setup,
    .buf_prepare = amd_csi2_buf_prepare,
    .buf_queue = amd_csi2_buf_queue,
    .start_streaming = amd_csi2_start_streaming,
    .stop_streaming = amd_csi2_stop_streaming,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
};

/* V4L2 device operations */
static int amd_csi2_querycap(struct file *file, void *priv,
                             struct v4l2_capability *cap)
{
    struct amd_csi2_dev *csi2_dev = video_drvdata(file);
    
    strscpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
    strscpy(cap->card, "AMD CSI2 Camera", sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", pci_name(csi2_dev->pdev));
    
    return 0;
}

static int amd_csi2_enum_fmt_vid_cap(struct file *file, void *priv,
                                     struct v4l2_fmtdesc *f)
{
    if (f->index > 0)
        return -EINVAL;
    
    f->pixelformat = DEFAULT_FORMAT;
    strscpy(f->description, "YUYV 4:2:2", sizeof(f->description));
    
    return 0;
}

static int amd_csi2_g_fmt_vid_cap(struct file *file, void *priv,
                                  struct v4l2_format *f)
{
    struct amd_csi2_dev *csi2_dev = video_drvdata(file);
    
    *f = csi2_dev->format;
    return 0;
}

static int amd_csi2_s_fmt_vid_cap(struct file *file, void *priv,
                                  struct v4l2_format *f)
{
    struct amd_csi2_dev *csi2_dev = video_drvdata(file);
    
    if (vb2_is_busy(&csi2_dev->queue))
        return -EBUSY;
    
    f->fmt.pix.width = DEFAULT_WIDTH;
    f->fmt.pix.height = DEFAULT_HEIGHT;
    f->fmt.pix.pixelformat = DEFAULT_FORMAT;
    f->fmt.pix.field = DEFAULT_FIELD;
    f->fmt.pix.bytesperline = DEFAULT_WIDTH * BYTES_PER_PIXEL;
    f->fmt.pix.sizeimage = DEFAULT_WIDTH * DEFAULT_HEIGHT * BYTES_PER_PIXEL;
    f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    
    csi2_dev->format = *f;
    
    return 0;
}

static int amd_csi2_try_fmt_vid_cap(struct file *file, void *priv,
                                    struct v4l2_format *f)
{
    f->fmt.pix.width = DEFAULT_WIDTH;
    f->fmt.pix.height = DEFAULT_HEIGHT;
    f->fmt.pix.pixelformat = DEFAULT_FORMAT;
    f->fmt.pix.field = DEFAULT_FIELD;
    f->fmt.pix.bytesperline = DEFAULT_WIDTH * BYTES_PER_PIXEL;
    f->fmt.pix.sizeimage = DEFAULT_WIDTH * DEFAULT_HEIGHT * BYTES_PER_PIXEL;
    f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    
    return 0;
}

static int amd_csi2_enum_input(struct file *file, void *priv,
                               struct v4l2_input *inp)
{
    if (inp->index > 0)
        return -EINVAL;
    
    inp->type = V4L2_INPUT_TYPE_CAMERA;
    strscpy(inp->name, "CSI2 Camera", sizeof(inp->name));
    inp->status = V4L2_IN_ST_NO_SIGNAL;
    
    return 0;
}

static int amd_csi2_g_input(struct file *file, void *priv, unsigned int *i)
{
    *i = 0;
    return 0;
}

static int amd_csi2_s_input(struct file *file, void *priv, unsigned int i)
{
    return i ? -EINVAL : 0;
}

static int amd_csi2_g_parm(struct file *file, void *priv,
                           struct v4l2_streamparm *parm)
{
    struct amd_csi2_dev *csi2_dev = video_drvdata(file);
    
    if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    
    parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    parm->parm.capture.timeperframe = csi2_dev->timeperframe;
    
    return 0;
}

static int amd_csi2_s_parm(struct file *file, void *priv,
                           struct v4l2_streamparm *parm)
{
    struct amd_csi2_dev *csi2_dev = video_drvdata(file);
    
    if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    
    parm->parm.capture.timeperframe.numerator = 1;
    parm->parm.capture.timeperframe.denominator = DEFAULT_FPS;
    
    csi2_dev->timeperframe = parm->parm.capture.timeperframe;
    
    return 0;
}

static const struct v4l2_ioctl_ops amd_csi2_ioctl_ops = {
    .vidioc_querycap = amd_csi2_querycap,
    .vidioc_enum_fmt_vid_cap = amd_csi2_enum_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap = amd_csi2_g_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap = amd_csi2_s_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap = amd_csi2_try_fmt_vid_cap,
    .vidioc_enum_input = amd_csi2_enum_input,
    .vidioc_g_input = amd_csi2_g_input,
    .vidioc_s_input = amd_csi2_s_input,
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

/* Device initialization */
static int amd_csi2_init_device(struct amd_csi2_dev *csi2_dev)
{
    struct video_device *vdev = &csi2_dev->vdev;
    struct vb2_queue *q = &csi2_dev->queue;
    int ret;
    
    spin_lock_init(&csi2_dev->lock);
    mutex_init(&csi2_dev->lock_mutex);
    INIT_LIST_HEAD(&csi2_dev->buf_list);
    
    csi2_dev->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    csi2_dev->format.fmt.pix.width = DEFAULT_WIDTH;
    csi2_dev->format.fmt.pix.height = DEFAULT_HEIGHT;
    csi2_dev->format.fmt.pix.pixelformat = DEFAULT_FORMAT;
    csi2_dev->format.fmt.pix.field = DEFAULT_FIELD;
    csi2_dev->format.fmt.pix.bytesperline = DEFAULT_WIDTH * BYTES_PER_PIXEL;
    csi2_dev->format.fmt.pix.sizeimage = DEFAULT_WIDTH * DEFAULT_HEIGHT * BYTES_PER_PIXEL;
    csi2_dev->format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    
    csi2_dev->timeperframe.numerator = 1;
    csi2_dev->timeperframe.denominator = DEFAULT_FPS;
    
    q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    q->io_modes = VB2_MMAP | VB2_READ;
    q->drv_priv = csi2_dev;
    q->buf_struct_size = sizeof(struct amd_csi2_buffer);
    q->ops = &amd_csi2_vb2_ops;
    q->mem_ops = &vb2_dma_contig_memops;
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    q->lock = &csi2_dev->lock_mutex;
    q->dev = &csi2_dev->pdev->dev;
    
    ret = vb2_queue_init(q);
    if (ret) {
        dev_err(&csi2_dev->pdev->dev, "❌ Failed to initialize VB2 queue: %d\n", ret);
        return ret;
    }
    
    vdev->fops = &amd_csi2_fops;
    vdev->ioctl_ops = &amd_csi2_ioctl_ops;
    vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
    vdev->release = video_device_release_empty;
    vdev->lock = &csi2_dev->lock_mutex;
    vdev->queue = q;
    vdev->v4l2_dev = &csi2_dev->v4l2_dev;
    strscpy(vdev->name, "AMD CSI2 Camera", sizeof(vdev->name));
    video_set_drvdata(vdev, csi2_dev);
    
    return 0;
}

/* PCI probe function */
static int amd_csi2_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct amd_csi2_dev *csi2_dev;
    int ret;
    
    dev_info(&pdev->dev, "🚀 AMD CSI2 MSI-X TIMING FIX v%s\n", DRIVER_VERSION);
    dev_info(&pdev->dev, "🎯 Fixed initialization sequence for Linux 6.15.4\n");
    
    csi2_dev = devm_kzalloc(&pdev->dev, sizeof(*csi2_dev), GFP_KERNEL);
    if (!csi2_dev)
        return -ENOMEM;
        
    csi2_dev->pdev = pdev;
    pci_set_drvdata(pdev, csi2_dev);
    
    ret = pci_enable_device(pdev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to enable PCI device: %d\n", ret);
        return ret;
    }
    
    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (ret) {
        ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (ret) {
            dev_err(&pdev->dev, "❌ Failed to set DMA mask: %d\n", ret);
            goto err_disable_device;
        }
    }
    
    pci_set_master(pdev);
    
    ret = pci_request_regions(pdev, DRIVER_NAME);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to request PCI regions: %d\n", ret);
        goto err_disable_device;
    }
    
    /* Map MMIO region */
    csi2_dev->mmio_base = pci_ioremap_bar(pdev, 0);
    if (!csi2_dev->mmio_base) {
        dev_err(&pdev->dev, "❌ Failed to map MMIO region\n");
        ret = -ENOMEM;
        goto err_release_regions;
    }
    
    dev_info(&pdev->dev, "📍 MMIO mapped at %p\n", csi2_dev->mmio_base);
    
    /* Setup MSI-X */
    ret = amd_csi2_setup_msix(csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to setup MSI-X: %d\n", ret);
        goto err_unmap_mmio;
    }
    
    /* Register V4L2 device */
    ret = v4l2_device_register(&pdev->dev, &csi2_dev->v4l2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to register V4L2 device: %d\n", ret);
        goto err_free_msix;
    }
    
    ret = amd_csi2_init_device(csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to initialize device: %d\n", ret);
        goto err_unregister_v4l2;
    }
    
    ret = video_register_device(&csi2_dev->vdev, VFL_TYPE_VIDEO, -1);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to register video device: %d\n", ret);
        goto err_unregister_v4l2;
    }
    
    /* Initialize hardware */
    ret = amd_csi2_hw_init(csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to initialize hardware: %d\n", ret);
        goto err_unregister_video;
    }
    
    csi2_dev->initialized = true;
    
    dev_info(&pdev->dev, "🎉 AMD CSI2 MSI-X TIMING FIX v%s loaded!\n", DRIVER_VERSION);
    dev_info(&pdev->dev, "📺 Video device: %s\n", video_device_node_name(&csi2_dev->vdev));
    dev_info(&pdev->dev, 
             "🏆 Configuration Summary:\n"
             "   Primary IRQ: %d\n"
             "   MSI-X vectors: %d\n"
             "   Hardware ready: %s\n"
             "   Init sequence: %u/4\n"
             "   Ready for operation! 🚀\n",
             csi2_dev->primary_irq, csi2_dev->num_irq_vectors,
             csi2_dev->hardware_ready ? "YES" : "INITIALIZING",
             csi2_dev->init_sequence_step);
    
    return 0;
    
err_unregister_video:
    video_unregister_device(&csi2_dev->vdev);
err_unregister_v4l2:
    v4l2_device_unregister(&csi2_dev->v4l2_dev);
err_free_msix:
    amd_csi2_free_msix(csi2_dev);
err_unmap_mmio:
    iounmap(csi2_dev->mmio_base);
err_release_regions:
    pci_release_regions(pdev);
err_disable_device:
    pci_disable_device(pdev);
    return ret;
}

/* PCI remove function */
static void amd_csi2_remove(struct pci_dev *pdev)
{
    struct amd_csi2_dev *csi2_dev = pci_get_drvdata(pdev);
    
    dev_info(&pdev->dev, "🧹 Removing AMD CSI2 MSI-X TIMING FIX Driver\n");
    
    if (csi2_dev->initialized) {
        if (csi2_dev->streaming) {
            amd_csi2_stop_streaming(&csi2_dev->queue);
        }
        
        /* Stop hardware monitoring */
        timer_delete_sync(&csi2_dev->hw_monitor_timer);
        cancel_work_sync(&csi2_dev->hw_test_work);
        
        /* Disable interrupts */
        csi2_write_reg(csi2_dev, CSI2_REG_GLOBAL_INT_ENABLE, 0);
        csi2_write_reg(csi2_dev, CSI2_REG_IER, 0);
        
        video_unregister_device(&csi2_dev->vdev);
        v4l2_device_unregister(&csi2_dev->v4l2_dev);
    }
    
    amd_csi2_free_msix(csi2_dev);
    
    if (csi2_dev->mmio_base) {
        iounmap(csi2_dev->mmio_base);
    }
    
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    
    dev_info(&pdev->dev, 
             "✅ AMD CSI2 MSI-X TIMING FIX Driver removed\n"
             "📊 Final stats: %d interrupts, %u frames\n",
             atomic_read(&csi2_dev->total_interrupts),
             csi2_dev->frames_captured);
}

/* PCI device table */
static const struct pci_device_id amd_csi2_pci_tbl[] = {
    { AMD_VENDOR_ID, AMD_CSI2_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, amd_csi2_pci_tbl);

/* PCI driver structure */
static struct pci_driver amd_csi2_pci_driver = {
    .name = DRIVER_NAME,
    .id_table = amd_csi2_pci_tbl,
    .probe = amd_csi2_probe,
    .remove = amd_csi2_remove,
};

static int __init amd_csi2_init(void)
{
    pr_info("🚀 AMD CSI2 MSI-X TIMING FIX V4L2 Driver v%s\n", DRIVER_VERSION);
    pr_info("🎯 Fixed initialization sequence for Linux 6.15.4 + QEMU 10.x\n");
    return pci_register_driver(&amd_csi2_pci_driver);
}

static void __exit amd_csi2_exit(void)
{
    pci_unregister_driver(&amd_csi2_pci_driver);
    pr_info("✅ AMD CSI2 MSI-X TIMING FIX V4L2 Driver v%s unloaded\n", DRIVER_VERSION);
}

module_init(amd_csi2_init);
module_exit(amd_csi2_exit);

MODULE_DESCRIPTION("AMD MIPI CSI-2 RX V4L2 Driver - MSI-X TIMING AND INITIALIZATION FIX");
MODULE_AUTHOR("AMD CSI2 Team");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);
