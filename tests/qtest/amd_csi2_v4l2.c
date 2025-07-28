/*
 * AMD MIPI CSI-2 RX Subsystem V4L2 Driver
 *
 * This driver provides V4L2 interface for AMD MIPI CSI-2 RX subsystem
 * emulated in QEMU. It supports video capture from virtual camera source
 * with MSI-X interrupt-based operation.
 *
 * Version 2.0 - MSI-X ONLY (polling mode removed)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/videodev2.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/time.h>
#include <linux/jiffies.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#define DRIVER_NAME "amd_csi2_v4l2"
#define DRIVER_VERSION "2.0"

/* PCI Device IDs */
#define AMD_VENDOR_ID 0x1022
#define AMD_CSI2_DEVICE_ID 0xC901

/* Missing defines for older kernels */
#ifndef PCI_IRQ_NOLEGACY
#define PCI_IRQ_NOLEGACY	0x00000004  /* Exclude legacy interrupt */
#endif

/* Compatibility functions for different kernel versions */
static inline bool pci_device_is_busmaster(struct pci_dev *pdev)
{
    u16 cmd;
    pci_read_config_word(pdev, PCI_COMMAND, &cmd);
    return !!(cmd & PCI_COMMAND_MASTER);
}

static inline bool pci_device_msix_enabled(struct pci_dev *pdev)
{
    u16 control;
    if (!pdev->msix_cap)
        return false;
    pci_read_config_word(pdev, pdev->msix_cap + PCI_MSIX_FLAGS, &control);
    return !!(control & PCI_MSIX_FLAGS_ENABLE);
}

/* Register Definitions */
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

/* D-PHY Register Definitions (offset 0x1000) */
#define CSI2_DPHY_REG_CONTROL          0x1000
#define CSI2_DPHY_REG_STATUS           0x1004
#define CSI2_DPHY_REG_HS_SETTLE        0x1008
#define CSI2_DPHY_REG_PLL_CTRL         0x100C
#define CSI2_DPHY_REG_PLL_STATUS       0x1010
#define CSI2_DPHY_REG_LANE_CONFIG      0x1014
#define CSI2_DPHY_REG_LANE_STATUS      0x1018
#define CSI2_DPHY_REG_FRAMEBUF_WR_PTR  0x1020
#define CSI2_DPHY_REG_FRAMEBUF_RD_PTR  0x1024
#define CSI2_DPHY_REG_FRAMEBUF_SIZE    0x1028
#define CSI2_DPHY_REG_FRAMEBUF_CTRL    0x102C

/* Control Register Bits */
#define CONTROL_CORE_ENABLE            BIT(0)
#define CONTROL_SOFT_RESET             BIT(1)
#define CONTROL_FULL_RESET             BIT(2)

/* Interrupt Status Register Bits */
#define ISR_FRAME_RECEIVED             BIT(31)
#define ISR_VCX_FRAME_ERROR           BIT(30)
#define ISR_RX_SKEWCALHS              BIT(29)
#define ISR_YUV420_WC_ERROR           BIT(28)
#define ISR_PENDING_WRITE_FIFO        BIT(27)
#define ISR_WC_CORRUPTION             BIT(22)
#define ISR_INCORRECT_LANE_CONFIG     BIT(21)
#define ISR_SHORT_PACKET_FIFO_FULL    BIT(20)
#define ISR_SHORT_PACKET_FIFO_NEMPTY  BIT(19)
#define ISR_STREAM_LINE_BUFFER_FULL   BIT(18)
#define ISR_STOP_STATE                BIT(17)
#define ISR_SOT_ERROR                 BIT(13)
#define ISR_SOT_SYNC_ERROR            BIT(12)
#define ISR_ECC_2BIT_ERROR            BIT(11)
#define ISR_ECC_1BIT_ERROR            BIT(10)
#define ISR_CRC_ERROR                 BIT(9)
#define ISR_UNSUPPORTED_DATA_TYPE     BIT(8)
#define ISR_FRAME_SYNC_ERROR_VC3      BIT(7)
#define ISR_FRAME_LEVEL_ERROR_VC3     BIT(6)
#define ISR_FRAME_SYNC_ERROR_VC2      BIT(5)
#define ISR_FRAME_LEVEL_ERROR_VC2     BIT(4)
#define ISR_FRAME_SYNC_ERROR_VC1      BIT(3)
#define ISR_FRAME_LEVEL_ERROR_VC1     BIT(2)
#define ISR_FRAME_SYNC_ERROR_VC0      BIT(1)
#define ISR_FRAME_LEVEL_ERROR_VC0     BIT(0)

/* D-PHY Control Register Bits */
#define DPHY_CONTROL_ENABLE            BIT(0)
#define DPHY_PLL_CONTROL_ENABLE        BIT(0)

/* Device Defaults */
#define DEFAULT_WIDTH  1920
#define DEFAULT_HEIGHT 1080
#define DEFAULT_FPS    30
#define DEFAULT_FORMAT V4L2_PIX_FMT_YUYV
#define DEFAULT_FIELD  V4L2_FIELD_NONE
#define BYTES_PER_PIXEL 2

/* Buffer Management */
#define MIN_BUFFERS    2
#define MAX_BUFFERS    8
#define MAX_FRAME_SIZE (DEFAULT_WIDTH * DEFAULT_HEIGHT * BYTES_PER_PIXEL)

/* MSI-X Configuration */
#define AMD_CSI2_MSIX_VECTORS 8
#define AMD_CSI2_MSIX_VEC_FRAME 0

/* Forward declarations */
struct amd_csi2_dev;
struct amd_csi2_buffer;

/* Buffer structure */
struct amd_csi2_buffer {
    struct vb2_v4l2_buffer vb;
    struct list_head list;
    dma_addr_t dma_addr;
    size_t size;
};

/* Main device structure */
struct amd_csi2_dev {
    struct pci_dev *pdev;
    struct v4l2_device v4l2_dev;
    struct video_device vdev;
    /* Remove ctrl_handler for now to avoid incomplete type issues */
    
    /* Hardware resources */
    void __iomem *mmio_base;
    void __iomem *msix_base;  /* MSI-X vector table mapping */
    int irq;
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
    u32 sequence;
    u32 frames_captured;
    
    /* Statistics */
    u64 total_interrupts;
    u64 frame_interrupts;
    u64 error_interrupts;
    ktime_t last_frame_time;
    
    /* Capture thread */
    struct task_struct *capture_thread;
    struct completion frame_completion;
    bool thread_should_stop;
};

/* MSI-X setup functions */
static int amd_csi2_setup_msix(struct amd_csi2_dev *csi2_dev)
{
    struct pci_dev *pdev = csi2_dev->pdev;
    void __iomem *msix_base;
    int ret, nvec, i;
    
    dev_info(&pdev->dev, "🔧 Setting up MSI-X interrupts (v2.0)\n");
    
    /* Check MSI-X capability */
    if (!pci_find_capability(pdev, PCI_CAP_ID_MSIX)) {
        dev_err(&pdev->dev, "❌ MSI-X capability not found\n");
        return -ENODEV;
    }
    
    dev_info(&pdev->dev, "✅ MSI-X capability found\n");
    
    /* Get available vectors */
    nvec = pci_msix_vec_count(pdev);
    dev_info(&pdev->dev, "📊 MSI-X vectors available: %d\n", nvec);
    
    if (nvec < 1) {
        dev_err(&pdev->dev, "❌ No MSI-X vectors available\n");
        return -ENODEV;
    }
    
    /* Ensure PCI device is properly enabled */
    if (!pci_is_enabled(pdev)) {
        dev_warn(&pdev->dev, "⚠️  PCI device not enabled, enabling now\n");
        ret = pci_enable_device(pdev);
        if (ret) {
            dev_err(&pdev->dev, "❌ Failed to enable PCI device: %d\n", ret);
            return ret;
        }
    }
    
    /* Ensure bus mastering is enabled */
    if (!pci_device_is_busmaster(pdev)) {
        dev_info(&pdev->dev, "🚌 Enabling bus master\n");
        pci_set_master(pdev);
    }
    
    /* Clean up any existing IRQ vectors */
    pci_free_irq_vectors(pdev);
    
    /* Allocate MSI-X vectors (force MSI-X only, no fallback) */
    dev_info(&pdev->dev, "🎯 Allocating MSI-X vectors (no legacy fallback)\n");
    ret = pci_alloc_irq_vectors(pdev, 1, min(nvec, AMD_CSI2_MSIX_VECTORS), 
                                PCI_IRQ_MSIX | PCI_IRQ_NOLEGACY);
    if (ret < 0) {
        dev_err(&pdev->dev, "❌ Failed to allocate MSI-X vectors: %d\n", ret);
        dev_err(&pdev->dev, "🔄 Attempting single vector allocation\n");
        
        /* Try with just one vector */
        ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSIX | PCI_IRQ_NOLEGACY);
        if (ret < 0) {
            dev_err(&pdev->dev, "❌ Single MSI-X vector allocation failed: %d\n", ret);
            return ret;
        }
    }
    
    dev_info(&pdev->dev, "✅ Successfully allocated %d MSI-X vectors\n", ret);
    
    /* Verify MSI-X is actually enabled */
    if (!pci_device_msix_enabled(pdev)) {
        dev_err(&pdev->dev, "❌ MSI-X not enabled after allocation!\n");
        pci_free_irq_vectors(pdev);
        return -ENODEV;
    }
    
    dev_info(&pdev->dev, "✅ MSI-X is now enabled in hardware\n");
    
    /* Check and clear MSI-X global mask bit */
    int msix_cap = pci_find_capability(pdev, PCI_CAP_ID_MSIX);
    if (msix_cap) {
        u16 control;
        pci_read_config_word(pdev, msix_cap + PCI_MSIX_FLAGS, &control);
        dev_info(&pdev->dev, "📊 MSI-X control register: 0x%04x\n", control);
        
        /* Check if function mask bit is set (bit 14 = 0x4000) - use hardcoded value */
        #define MSIX_FUNCTION_MASK 0x4000
        if (control & MSIX_FUNCTION_MASK) {
            dev_warn(&pdev->dev, "⚠️  MSI-X globally masked (bit 14)! Clearing function mask...\n");
            control &= ~MSIX_FUNCTION_MASK;
            pci_write_config_word(pdev, msix_cap + PCI_MSIX_FLAGS, control);
            
            /* Re-read to verify */
            pci_read_config_word(pdev, msix_cap + PCI_MSIX_FLAGS, &control);
            dev_info(&pdev->dev, "📊 MSI-X control after unmask: 0x%04x\n", control);
            
            if (control & MSIX_FUNCTION_MASK) {
                dev_err(&pdev->dev, "❌ Failed to clear MSI-X function mask!\n");
            } else {
                dev_info(&pdev->dev, "✅ MSI-X function mask cleared successfully!\n");
            }
        } else {
            dev_info(&pdev->dev, "ℹ️  MSI-X function mask already clear (0x%04x)\n", control);
        }
        
        /* Force clear the mask bit even if it wasn't detected */
        dev_info(&pdev->dev, "🔧 Force clearing MSI-X function mask bit (0x4000)...\n");
        u16 old_control = control;
        control = (control & ~MSIX_FUNCTION_MASK) | PCI_MSIX_FLAGS_ENABLE;
        pci_write_config_word(pdev, msix_cap + PCI_MSIX_FLAGS, control);
        
        /* Final verification */
        pci_read_config_word(pdev, msix_cap + PCI_MSIX_FLAGS, &control);
        dev_info(&pdev->dev, "🏁 MSI-X control: 0x%04x -> 0x%04x %s\n", 
                 old_control, control,
                 (control & MSIX_FUNCTION_MASK) ? "❌ STILL MASKED!" : "✅ UNMASKED!");
    } else {
        dev_err(&pdev->dev, "❌ MSI-X capability not found for global unmask!\n");
    }
    
    /* Map MSI-X vector table (BAR 2) for verification */
    msix_base = pci_ioremap_bar(pdev, 2);
    if (!msix_base) {
        dev_err(&pdev->dev, "❌ Failed to map MSI-X vector table - this is critical!\n");
        pci_free_irq_vectors(pdev);
        return -ENOMEM;
    } else {
        dev_info(&pdev->dev, "📍 MSI-X vector table mapped at %p\n", msix_base);
        
        /* Read and display MSI-X vector table entries */
        for (i = 0; i < min(ret, 4); i++) {
            u32 addr_low = readl(msix_base + (i * 16) + 0);
            u32 addr_high = readl(msix_base + (i * 16) + 4);
            u32 msg_data = readl(msix_base + (i * 16) + 8);
            u32 ctrl = readl(msix_base + (i * 16) + 12);
            
            dev_info(&pdev->dev, "📋 Vector %d: addr=0x%x%08x, data=0x%x, ctrl=0x%x\n",
                     i, addr_high, addr_low, msg_data, ctrl);
            
            /* Force unmask ALL vectors regardless of current state */
            dev_info(&pdev->dev, "🔓 BEFORE unmask - Vector %d ctrl=0x%x\n", i, ctrl);
            writel(0, msix_base + (i * 16) + 12);
            
            /* Read back immediately */
            ctrl = readl(msix_base + (i * 16) + 12);
            dev_info(&pdev->dev, "🔓 AFTER unmask - Vector %d ctrl=0x%x\n", i, ctrl);
            
            /* If still masked, try harder */
            if (ctrl & 1) {
                dev_err(&pdev->dev, "❌ Vector %d still masked! Trying harder...\n", i);
                int retry;
                for (retry = 0; retry < 10; retry++) {
                    writel(0x00000000, msix_base + (i * 16) + 12);
                    wmb(); /* Write memory barrier */
                    msleep(1);
                    ctrl = readl(msix_base + (i * 16) + 12);
                    dev_info(&pdev->dev, "🔄 Retry %d: Vector %d ctrl=0x%x\n", retry, i, ctrl);
                    if (!(ctrl & 1)) {
                        dev_info(&pdev->dev, "✅ Vector %d unmasked after %d retries!\n", i, retry + 1);
                        break;
                    }
                }
                if (ctrl & 1) {
                    dev_err(&pdev->dev, "💥 CRITICAL: Vector %d cannot be unmasked! Hardware issue?\n", i);
                }
            } else {
                dev_info(&pdev->dev, "✅ Vector %d successfully unmasked (ctrl=0x%x)\n", i, ctrl);
            }
            
            /* Final verification */
            ctrl = readl(msix_base + (i * 16) + 12);
            dev_info(&pdev->dev, "🏁 FINAL - Vector %d ctrl=0x%x %s\n", 
                     i, ctrl, (ctrl & 1) ? "❌ MASKED" : "✅ UNMASKED");
        }
        
        /* Keep the mapping for potential later use - don't unmap yet */
        /* We'll unmap it in the cleanup function */
        csi2_dev->msix_base = msix_base;
    }
    
    /* Get IRQ numbers for each vector */
    for (i = 0; i < ret; i++) {
        int irq = pci_irq_vector(pdev, i);
        if (irq < 0) {
            dev_err(&pdev->dev, "❌ Failed to get IRQ for vector %d: %d\n", i, irq);
            pci_free_irq_vectors(pdev);
            return irq;
        }
        dev_info(&pdev->dev, "📌 MSI-X vector %d mapped to IRQ %d\n", i, irq);
    }
    
    /* Store primary IRQ (vector 0) */
    csi2_dev->irq = pci_irq_vector(pdev, AMD_CSI2_MSIX_VEC_FRAME);
    if (csi2_dev->irq < 0) {
        dev_err(&pdev->dev, "❌ Failed to get primary IRQ: %d\n", csi2_dev->irq);
        pci_free_irq_vectors(pdev);
        return csi2_dev->irq;
    }
    
    dev_info(&pdev->dev, "🎯 Primary interrupt vector: IRQ %d\n", csi2_dev->irq);
    csi2_dev->msix_enabled = true;
    
    return 0;
}

static void amd_csi2_free_msix(struct amd_csi2_dev *csi2_dev)
{
    if (csi2_dev->msix_enabled) {
        dev_info(&csi2_dev->pdev->dev, "🧹 Freeing MSI-X vectors\n");
        
        /* Unmap MSI-X table if it was mapped */
        if (csi2_dev->msix_base) {
            iounmap(csi2_dev->msix_base);
            csi2_dev->msix_base = NULL;
        }
        
        pci_free_irq_vectors(csi2_dev->pdev);
        csi2_dev->msix_enabled = false;
        csi2_dev->irq = 0;
    }
}

/* Interrupt handler */
static irqreturn_t amd_csi2_interrupt(int irq, void *dev_id)
{
    struct amd_csi2_dev *csi2_dev = dev_id;
    u32 isr_status, global_enable, ier_status;
    unsigned long flags;
    bool handled = false;
    
    /* Increment interrupt counters */
    csi2_dev->total_interrupts++;
    
    /* Always log the first few interrupts */
    if (csi2_dev->total_interrupts <= 5) {
        dev_info(&csi2_dev->pdev->dev, "🎉 MSI-X INTERRUPT #%llu RECEIVED! IRQ=%d\n", 
                 csi2_dev->total_interrupts, irq);
                 
        /* Check MSI-X vector table on first interrupt */
        if (csi2_dev->total_interrupts == 1 && csi2_dev->msix_base) {
            u32 addr_low = readl(csi2_dev->msix_base + 0);
            u32 addr_high = readl(csi2_dev->msix_base + 4);
            u32 msg_data = readl(csi2_dev->msix_base + 8);
            u32 ctrl = readl(csi2_dev->msix_base + 12);
            
            dev_info(&csi2_dev->pdev->dev, 
                     "🔍 MSI-X Vector 0 on first interrupt: addr=0x%x%08x, data=0x%x, ctrl=0x%x\n",
                     addr_high, addr_low, msg_data, ctrl);
        }
    }
    
    /* Read all relevant registers for debugging */
    isr_status = readl(csi2_dev->mmio_base + CSI2_REG_ISR);
    global_enable = readl(csi2_dev->mmio_base + CSI2_REG_GLOBAL_INT_ENABLE);
    ier_status = readl(csi2_dev->mmio_base + CSI2_REG_IER);
    
    /* Log detailed interrupt information */
    if (csi2_dev->total_interrupts <= 10 || (csi2_dev->total_interrupts % 100 == 0)) {
        dev_info(&csi2_dev->pdev->dev, 
                 "🔔 IRQ %d: ISR=0x%08x, Global=0x%08x, IER=0x%08x\n", 
                 irq, isr_status, global_enable, ier_status);
    }
    
    if (!isr_status) {
        if (csi2_dev->total_interrupts <= 5) {
            dev_warn(&csi2_dev->pdev->dev, "⚠️  Spurious interrupt (ISR=0)\n");
        }
        return IRQ_NONE;
    }
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    
    /* Handle frame received interrupt */
    if (isr_status & ISR_FRAME_RECEIVED) {
        csi2_dev->frame_interrupts++;
        dev_info(&csi2_dev->pdev->dev, "🎬 Frame received interrupt (#%llu)\n", 
                csi2_dev->frame_interrupts);
        
        /* Signal capture thread */
        complete(&csi2_dev->frame_completion);
        handled = true;
    }
    
    /* Handle error interrupts */
    if (isr_status & (ISR_CRC_ERROR | ISR_ECC_1BIT_ERROR | ISR_ECC_2BIT_ERROR |
                      ISR_SOT_ERROR | ISR_SOT_SYNC_ERROR | ISR_STREAM_LINE_BUFFER_FULL)) {
        csi2_dev->error_interrupts++;
        dev_warn(&csi2_dev->pdev->dev, "⚠️  Error interrupt: 0x%08x (#%llu)\n", 
                 isr_status, csi2_dev->error_interrupts);
        handled = true;
    }
    
    /* Handle any other interrupts */
    if (isr_status & ~(ISR_FRAME_RECEIVED | ISR_CRC_ERROR | ISR_ECC_1BIT_ERROR | 
                       ISR_ECC_2BIT_ERROR | ISR_SOT_ERROR | ISR_SOT_SYNC_ERROR | 
                       ISR_STREAM_LINE_BUFFER_FULL)) {
        dev_info(&csi2_dev->pdev->dev, "🔔 Other interrupt bits: 0x%08lx\n", 
                 (unsigned long)(isr_status & ~(ISR_FRAME_RECEIVED | ISR_CRC_ERROR | ISR_ECC_1BIT_ERROR | 
                               ISR_ECC_2BIT_ERROR | ISR_SOT_ERROR | ISR_SOT_SYNC_ERROR | 
                               ISR_STREAM_LINE_BUFFER_FULL)));
        handled = true;
    }
    
    /* Clear interrupt status */
    writel(isr_status, csi2_dev->mmio_base + CSI2_REG_ISR);
    
    /* Verify interrupt was cleared */
    if (csi2_dev->total_interrupts <= 5) {
        u32 new_isr = readl(csi2_dev->mmio_base + CSI2_REG_ISR);
        dev_info(&csi2_dev->pdev->dev, "🧹 ISR after clear: 0x%08x\n", new_isr);
    }
    
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    return handled ? IRQ_HANDLED : IRQ_NONE;
}

/* Frame capture thread */
static int amd_csi2_capture_thread(void *data)
{
    struct amd_csi2_dev *csi2_dev = data;
    struct amd_csi2_buffer *buf;
    unsigned long flags;
    int frame_count = 0;
    
    dev_info(&csi2_dev->pdev->dev, "🎬 Capture thread started (MSI-X mode)\n");
    
    while (!kthread_should_stop() && !csi2_dev->thread_should_stop) {
        /* Wait for frame interrupt */
        if (wait_for_completion_interruptible(&csi2_dev->frame_completion)) {
            break;
        }
        
        /* Reset completion for next frame */
        reinit_completion(&csi2_dev->frame_completion);
        
        spin_lock_irqsave(&csi2_dev->lock, flags);
        
        if (!list_empty(&csi2_dev->buf_list) && csi2_dev->streaming) {
            /* Get next buffer */
            buf = list_first_entry(&csi2_dev->buf_list, struct amd_csi2_buffer, list);
            list_del(&buf->list);
            
            /* Fill buffer with frame data */
            buf->vb.vb2_buf.timestamp = ktime_get_ns();
            buf->vb.sequence = csi2_dev->sequence++;
            buf->vb.field = V4L2_FIELD_NONE;
            
            /* Set frame size */
            vb2_set_plane_payload(&buf->vb.vb2_buf, 0, 
                                  csi2_dev->format.fmt.pix.sizeimage);
            
            /* Complete buffer */
            vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
            
            frame_count++;
            csi2_dev->frames_captured++;
            csi2_dev->last_frame_time = ktime_get();
            
            dev_dbg(&csi2_dev->pdev->dev, "📸 Frame #%d completed (%d bytes)\n",
                    frame_count, csi2_dev->format.fmt.pix.sizeimage);
        }
        
        spin_unlock_irqrestore(&csi2_dev->lock, flags);
    }
    
    dev_info(&csi2_dev->pdev->dev, "🎬 Capture thread stopped after %d frames\n", frame_count);
    return 0;
}

/* Hardware initialization */
static int amd_csi2_hw_init(struct amd_csi2_dev *csi2_dev)
{
    u32 val;
    
    dev_info(&csi2_dev->pdev->dev, "🔧 Initializing CSI-2 hardware\n");
    
    /* Read initial register values for debugging */
    val = readl(csi2_dev->mmio_base + CSI2_REG_CORE_CONFIG);
    dev_info(&csi2_dev->pdev->dev, "Initial CORE_CONFIG: 0x%08x\n", val);
    
    /* Soft reset */
    writel(CONTROL_SOFT_RESET, csi2_dev->mmio_base + CSI2_REG_CORE_CONFIG);
    msleep(10);
    
    /* Enable core */
    writel(CONTROL_CORE_ENABLE, csi2_dev->mmio_base + CSI2_REG_CORE_CONFIG);
    
    /* Verify core is enabled */
    val = readl(csi2_dev->mmio_base + CSI2_REG_CORE_CONFIG);
    dev_info(&csi2_dev->pdev->dev, "CORE_CONFIG after enable: 0x%08x\n", val);
    
    /* Configure protocol (4 lanes) */
    writel(0x3, csi2_dev->mmio_base + CSI2_REG_PROTOCOL_CONFIG);
    
    /* Enable D-PHY PLL */
    writel(DPHY_PLL_CONTROL_ENABLE, csi2_dev->mmio_base + CSI2_DPHY_REG_PLL_CTRL);
    
    /* Enable D-PHY */
    writel(DPHY_CONTROL_ENABLE, csi2_dev->mmio_base + CSI2_DPHY_REG_CONTROL);
    
    /* Configure HS settle time */
    writel(0x20, csi2_dev->mmio_base + CSI2_DPHY_REG_HS_SETTLE);
    
    /* Enable frame buffer */
    writel(0x1, csi2_dev->mmio_base + CSI2_DPHY_REG_FRAMEBUF_CTRL);
    
    /* Clear any pending interrupts */
    val = readl(csi2_dev->mmio_base + CSI2_REG_ISR);
    dev_info(&csi2_dev->pdev->dev, "Clearing ISR: 0x%08x\n", val);
    writel(val, csi2_dev->mmio_base + CSI2_REG_ISR);
    
    /* Enable frame received interrupt */
    writel(ISR_FRAME_RECEIVED, csi2_dev->mmio_base + CSI2_REG_IER);
    val = readl(csi2_dev->mmio_base + CSI2_REG_IER);
    dev_info(&csi2_dev->pdev->dev, "IER set to: 0x%08x\n", val);
    
    /* Enable global interrupts */
    writel(0x1, csi2_dev->mmio_base + CSI2_REG_GLOBAL_INT_ENABLE);
    val = readl(csi2_dev->mmio_base + CSI2_REG_GLOBAL_INT_ENABLE);
    dev_info(&csi2_dev->pdev->dev, "Global interrupt enable: 0x%08x\n", val);
    
    /* Test interrupt generation - write to a test register */
    dev_info(&csi2_dev->pdev->dev, "🧪 Testing interrupt generation...\n");
    
    /* Simulate frame received by writing directly to ISR (if supported by QEMU device) */
    writel(ISR_FRAME_RECEIVED, csi2_dev->mmio_base + CSI2_REG_ISR);
    
    /* Check if interrupt was generated */
    msleep(100);
    val = readl(csi2_dev->mmio_base + CSI2_REG_ISR);
    dev_info(&csi2_dev->pdev->dev, "ISR after test write: 0x%08x\n", val);
    
    dev_info(&csi2_dev->pdev->dev, "✅ CSI-2 hardware initialized\n");
    
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
    
    dev_dbg(&csi2_dev->pdev->dev, "Queue setup: %u buffers, size=%u\n",
            *nbuffers, sizes[0]);
    
    return 0;
}

static int amd_csi2_buf_prepare(struct vb2_buffer *vb)
{
    struct amd_csi2_dev *csi2_dev = vb2_get_drv_priv(vb->vb2_queue);
    unsigned long size = csi2_dev->format.fmt.pix.sizeimage;
    
    if (vb2_plane_size(vb, 0) < size) {
        dev_err(&csi2_dev->pdev->dev, "Buffer too small (%lu < %lu)\n",
                vb2_plane_size(vb, 0), size);
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
    
    /* Store DMA address */
    buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
    buf->size = vb2_plane_size(vb, 0);
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    list_add_tail(&buf->list, &csi2_dev->buf_list);
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    dev_dbg(&csi2_dev->pdev->dev, "Buffer queued: DMA=0x%llx, size=%zu\n",
            buf->dma_addr, buf->size);
}

static int amd_csi2_start_streaming(struct vb2_queue *q, unsigned int count)
{
    struct amd_csi2_dev *csi2_dev = vb2_get_drv_priv(q);
    unsigned long flags;
    int ret;
    
    dev_info(&csi2_dev->pdev->dev, "🎬 Starting streaming (MSI-X mode)\n");
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    csi2_dev->streaming = true;
    csi2_dev->sequence = 0;
    csi2_dev->frames_captured = 0;
    csi2_dev->thread_should_stop = false;
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    /* Initialize completion */
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
    
    dev_info(&csi2_dev->pdev->dev, "✅ Streaming started successfully\n");
    return 0;
}

static void amd_csi2_stop_streaming(struct vb2_queue *q)
{
    struct amd_csi2_dev *csi2_dev = vb2_get_drv_priv(q);
    struct amd_csi2_buffer *buf, *tmp;
    unsigned long flags;
    
    dev_info(&csi2_dev->pdev->dev, "⏹️  Stopping streaming\n");
    
    /* Stop capture thread */
    if (csi2_dev->capture_thread) {
        csi2_dev->thread_should_stop = true;
        complete(&csi2_dev->frame_completion);
        kthread_stop(csi2_dev->capture_thread);
        csi2_dev->capture_thread = NULL;
    }
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    csi2_dev->streaming = false;
    
    /* Return all queued buffers */
    list_for_each_entry_safe(buf, tmp, &csi2_dev->buf_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    dev_info(&csi2_dev->pdev->dev, "✅ Streaming stopped (captured %u frames)\n",
             csi2_dev->frames_captured);
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
    strscpy(cap->card, "AMD CSI2 PCIe Camera", sizeof(cap->card));
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
    
    /* Only support our default format */
    f->fmt.pix.width = DEFAULT_WIDTH;
    f->fmt.pix.height = DEFAULT_HEIGHT;
    f->fmt.pix.pixelformat = DEFAULT_FORMAT;
    f->fmt.pix.field = DEFAULT_FIELD;
    f->fmt.pix.bytesperline = DEFAULT_WIDTH * BYTES_PER_PIXEL;
    f->fmt.pix.sizeimage = DEFAULT_WIDTH * DEFAULT_HEIGHT * BYTES_PER_PIXEL;
    f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    
    csi2_dev->format = *f;
    
    dev_dbg(&csi2_dev->pdev->dev, "Format set: %ux%u, size=%u\n",
            f->fmt.pix.width, f->fmt.pix.height, f->fmt.pix.sizeimage);
    
    return 0;
}

static int amd_csi2_try_fmt_vid_cap(struct file *file, void *priv,
                                    struct v4l2_format *f)
{
    /* Only support our default format */
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
    strscpy(inp->name, "CSI2 Virtual Camera", sizeof(inp->name));
    inp->status = V4L2_IN_ST_NO_SIGNAL;  /* Will be updated based on HW status */
    
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
    
    /* Only support 30 FPS */
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
    
    /* Initialize locks */
    spin_lock_init(&csi2_dev->lock);
    mutex_init(&csi2_dev->lock_mutex);
    INIT_LIST_HEAD(&csi2_dev->buf_list);
    
    /* Initialize default format */
    csi2_dev->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    csi2_dev->format.fmt.pix.width = DEFAULT_WIDTH;
    csi2_dev->format.fmt.pix.height = DEFAULT_HEIGHT;
    csi2_dev->format.fmt.pix.pixelformat = DEFAULT_FORMAT;
    csi2_dev->format.fmt.pix.field = DEFAULT_FIELD;
    csi2_dev->format.fmt.pix.bytesperline = DEFAULT_WIDTH * BYTES_PER_PIXEL;
    csi2_dev->format.fmt.pix.sizeimage = DEFAULT_WIDTH * DEFAULT_HEIGHT * BYTES_PER_PIXEL;
    csi2_dev->format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    
    /* Initialize timeperframe */
    csi2_dev->timeperframe.numerator = 1;
    csi2_dev->timeperframe.denominator = DEFAULT_FPS;
    
    /* Initialize VB2 queue */
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
    
    /* Initialize video device */
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
    
    dev_info(&pdev->dev, "🚀 AMD CSI2 device found: %04x:%04x (v%s)\n", 
             pdev->vendor, pdev->device, DRIVER_VERSION);
    
    /* Allocate device structure */
    csi2_dev = devm_kzalloc(&pdev->dev, sizeof(*csi2_dev), GFP_KERNEL);
    if (!csi2_dev)
        return -ENOMEM;
        
    csi2_dev->pdev = pdev;
    pci_set_drvdata(pdev, csi2_dev);
    
    /* Enable PCI device */
    ret = pci_enable_device(pdev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to enable PCI device: %d\n", ret);
        return ret;
    }
    
    /* Set DMA mask */
    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (ret) {
        ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (ret) {
            dev_err(&pdev->dev, "❌ Failed to set DMA mask: %d\n", ret);
            goto err_disable_device;
        }
    }
    
    /* Enable bus mastering */
    pci_set_master(pdev);
    
    /* Request PCI regions */
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
    
    /* Setup MSI-X interrupts */
    ret = amd_csi2_setup_msix(csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to setup MSI-X: %d\n", ret);
        goto err_unmap_mmio;
    }
    
    /* Request IRQ */
    ret = request_irq(csi2_dev->irq, amd_csi2_interrupt, 0, DRIVER_NAME, csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to request IRQ %d: %d\n", csi2_dev->irq, ret);
        goto err_free_msix;
    }
    
    dev_info(&pdev->dev, "✅ IRQ %d registered successfully\n", csi2_dev->irq);
    
    /* Initialize V4L2 device */
    ret = v4l2_device_register(&pdev->dev, &csi2_dev->v4l2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to register V4L2 device: %d\n", ret);
        goto err_free_irq;
    }
    
    /* Initialize device */
    ret = amd_csi2_init_device(csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to initialize device: %d\n", ret);
        goto err_unregister_v4l2;
    }
    
    /* Register video device */
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
    
    dev_info(&pdev->dev, "✅ AMD CSI2 v%s driver loaded successfully\n", DRIVER_VERSION);
    dev_info(&pdev->dev, "📺 Video device: %s\n", video_device_node_name(&csi2_dev->vdev));
    dev_info(&pdev->dev, "🎯 MSI-X interrupt mode ready (IRQ %d)\n", csi2_dev->irq);
    
    return 0;
    
err_unregister_video:
    video_unregister_device(&csi2_dev->vdev);
err_unregister_v4l2:
    v4l2_device_unregister(&csi2_dev->v4l2_dev);
err_free_irq:
    free_irq(csi2_dev->irq, csi2_dev);
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
    
    dev_info(&pdev->dev, "🧹 Removing AMD CSI2 driver\n");
    
    if (csi2_dev->initialized) {
        /* Stop any ongoing streaming */
        if (csi2_dev->streaming) {
            amd_csi2_stop_streaming(&csi2_dev->queue);
        }
        
        /* Disable interrupts */
        writel(0, csi2_dev->mmio_base + CSI2_REG_GLOBAL_INT_ENABLE);
        writel(0, csi2_dev->mmio_base + CSI2_REG_IER);
        
        video_unregister_device(&csi2_dev->vdev);
        v4l2_device_unregister(&csi2_dev->v4l2_dev);
    }
    
    if (csi2_dev->irq) {
        free_irq(csi2_dev->irq, csi2_dev);
    }
    
    amd_csi2_free_msix(csi2_dev);
    
    if (csi2_dev->mmio_base) {
        iounmap(csi2_dev->mmio_base);
    }
    
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    
    dev_info(&pdev->dev, "✅ AMD CSI2 driver removed successfully\n");
    dev_info(&pdev->dev, "📊 Final stats: %llu total interrupts (%llu frame, %llu error)\n",
             csi2_dev->total_interrupts, csi2_dev->frame_interrupts, csi2_dev->error_interrupts);
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

/* Module initialization */
static int __init amd_csi2_init(void)
{
    pr_info("AMD CSI2 V4L2 Driver v%s (MSI-X only)\n", DRIVER_VERSION);
    return pci_register_driver(&amd_csi2_pci_driver);
}

static void __exit amd_csi2_exit(void)
{
    pci_unregister_driver(&amd_csi2_pci_driver);
    pr_info("AMD CSI2 V4L2 Driver v%s unloaded\n", DRIVER_VERSION);
}

module_init(amd_csi2_init);
module_exit(amd_csi2_exit);

MODULE_DESCRIPTION("AMD MIPI CSI-2 RX V4L2 Driver - MSI-X Only");
MODULE_AUTHOR("AMD CSI2 Team");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);
