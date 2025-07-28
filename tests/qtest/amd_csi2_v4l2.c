/*
 * AMD MIPI CSI-2 RX Subsystem V4L2 Driver
 * Version 2.3 - MSI-X Vector Address Debug
 * 🚨 NEW CODE VERSION 1149 - Vector Address Analysis
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
#define DRIVER_VERSION "2.3"

/* PCI Device IDs */
#define AMD_VENDOR_ID 0x1022
#define AMD_CSI2_DEVICE_ID 0xC901

#ifndef PCI_IRQ_NOLEGACY
#define PCI_IRQ_NOLEGACY	0x00000004
#endif

/* Register Definitions */
#define CSI2_REG_CORE_CONFIG           0x00
#define CSI2_REG_PROTOCOL_CONFIG       0x04
#define CSI2_REG_CORE_STATUS           0x10
#define CSI2_REG_GLOBAL_INT_ENABLE     0x20
#define CSI2_REG_ISR                   0x24
#define CSI2_REG_IER                   0x28

/* Test registers for QEMU communication */
#define CSI2_REG_TEST_TRIGGER          0x50
#define CSI2_REG_DEBUG_CTRL            0x54
#define CSI2_REG_FORCE_INT             0x58

/* Control Register Bits */
#define CONTROL_CORE_ENABLE            BIT(0)
#define CONTROL_SOFT_RESET             BIT(1)
#define CONTROL_FULL_RESET             BIT(2)

/* Interrupt Status Register Bits */
#define ISR_FRAME_RECEIVED             BIT(31)

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

/* MSI-X Configuration */
#define AMD_CSI2_MSIX_VECTORS 8
#define AMD_CSI2_MSIX_VEC_FRAME 0

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
    void __iomem *msix_base;
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
    
    /* 🆕 Enhanced interrupt debugging */
    atomic_t total_interrupts;
    atomic_t frame_interrupts;
    atomic_t test_interrupts;
    atomic_t qemu_trigger_count;
    ktime_t last_frame_time;
    u32 last_isr_value;
    unsigned long last_interrupt_jiffies;
    
    /* Capture thread */
    struct task_struct *capture_thread;
    struct completion frame_completion;
    bool thread_should_stop;
    
    /* 🆕 MSI-X Vector debugging */
    u64 vector_address;
    u32 vector_data;
    bool vector_info_valid;
    struct timer_list debug_timer;
};

/* 🆕 MSI-X 벡터 주소 분석 함수 */
static void amd_csi2_analyze_msix_vectors(struct amd_csi2_dev *csi2_dev)
{
    int i;
    
    if (!csi2_dev->msix_base) {
        dev_err(&csi2_dev->pdev->dev, "❌ MSI-X base not mapped for analysis\n");
        return;
    }
    
    dev_info(&csi2_dev->pdev->dev, "🔍 DEEP MSI-X Vector Analysis\n");
    dev_info(&csi2_dev->pdev->dev, "================================\n");
    
    for (i = 0; i < min(4, AMD_CSI2_MSIX_VECTORS); i++) {
        u32 addr_low = readl(csi2_dev->msix_base + (i * 16) + 0);
        u32 addr_high = readl(csi2_dev->msix_base + (i * 16) + 4);
        u32 msg_data = readl(csi2_dev->msix_base + (i * 16) + 8);
        u32 ctrl = readl(csi2_dev->msix_base + (i * 16) + 12);
        
        u64 full_addr = ((u64)addr_high << 32) | addr_low;
        
        dev_info(&csi2_dev->pdev->dev,
                 "🎯 Vector %d:\n"
                 "   Address: 0x%016llx (High=0x%08x, Low=0x%08x)\n"
                 "   Data: 0x%08x\n"
                 "   Control: 0x%08x %s\n",
                 i, full_addr, addr_high, addr_low, msg_data, ctrl,
                 (ctrl & 1) ? "❌ MASKED" : "✅ UNMASKED");
        
        /* Vector 0 정보 저장 */
        if (i == 0) {
            csi2_dev->vector_address = full_addr;
            csi2_dev->vector_data = msg_data;
            csi2_dev->vector_info_valid = (addr_low != 0 || addr_high != 0);
            
            dev_info(&csi2_dev->pdev->dev,
                     "📌 Primary Vector 0 Info Saved:\n"
                     "   Valid: %s\n"
                     "   Address: 0x%016llx\n"
                     "   Data: 0x%08x\n",
                     csi2_dev->vector_info_valid ? "YES" : "NO",
                     csi2_dev->vector_address, csi2_dev->vector_data);
        }
    }
    
    /* IRQ 정보와 비교 */
    dev_info(&csi2_dev->pdev->dev,
             "🔗 Interrupt Correlation:\n"
             "   Primary IRQ: %d\n"
             "   Vector 0 valid: %s\n"
             "   MSI-X enabled: %s\n",
             csi2_dev->irq, 
             csi2_dev->vector_info_valid ? "YES" : "NO",
             csi2_dev->msix_enabled ? "YES" : "NO");
}

/* 🆕 QEMU와 직접 통신하는 인터럽트 트리거 */
static void amd_csi2_trigger_qemu_interrupt(struct amd_csi2_dev *csi2_dev)
{
    u32 trigger_val;
    int trigger_count = atomic_inc_return(&csi2_dev->qemu_trigger_count);
    
    dev_info(&csi2_dev->pdev->dev, 
             "🚀 QEMU Direct Trigger #%d - Testing MSI-X path\n", trigger_count);
    
    /* QEMU 인식 가능한 특별한 패턴으로 트리거 */
    trigger_val = 0x12345678;
    writel(trigger_val, csi2_dev->mmio_base + CSI2_REG_TEST_TRIGGER);
    wmb();
    
    /* 디버그 컨트롤도 설정 */
    writel(0x1, csi2_dev->mmio_base + CSI2_REG_DEBUG_CTRL);
    wmb();
    
    /* 강제 인터럽트 레지스터 */
    writel(0xDEADBEEF, csi2_dev->mmio_base + CSI2_REG_FORCE_INT);
    wmb();
    
    /* 읽기 확인 */
    u32 test_back = readl(csi2_dev->mmio_base + CSI2_REG_TEST_TRIGGER);
    u32 debug_back = readl(csi2_dev->mmio_base + CSI2_REG_DEBUG_CTRL);
    u32 force_back = readl(csi2_dev->mmio_base + CSI2_REG_FORCE_INT);
    
    dev_info(&csi2_dev->pdev->dev,
             "📊 QEMU Register Response:\n"
             "   TEST (wrote 0x%08x, read 0x%08x) %s\n"
             "   DEBUG (wrote 0x00000001, read 0x%08x) %s\n"
             "   FORCE (wrote 0xDEADBEEF, read 0x%08x) %s\n",
             trigger_val, test_back, (test_back == trigger_val) ? "✅ MATCH" : "❌ DIFF",
             debug_back, (debug_back == 0x1) ? "✅ MATCH" : "❌ DIFF",
             force_back, (force_back == 0xDEADBEEF) ? "✅ MATCH" : "❌ DIFF");
    
    /* 추가 대기 후 ISR 확인 */
    msleep(10);
    u32 isr = readl(csi2_dev->mmio_base + CSI2_REG_ISR);
    dev_info(&csi2_dev->pdev->dev, "📋 ISR after QEMU trigger: 0x%08x\n", isr);
}

/* 🆕 타이머 기반 주기적 디버깅 */
static void amd_csi2_debug_timer_callback(struct timer_list *timer)
{
    struct amd_csi2_dev *csi2_dev = from_timer(csi2_dev, timer, debug_timer);
    
    if (!csi2_dev->streaming) {
        return;
    }
    
    /* 매 5초마다 상태 체크 */
    int total_ints = atomic_read(&csi2_dev->total_interrupts);
    int qemu_triggers = atomic_read(&csi2_dev->qemu_trigger_count);
    
    dev_info(&csi2_dev->pdev->dev,
             "⏰ Debug Timer: %d real interrupts, %d QEMU triggers\n",
             total_ints, qemu_triggers);
    
    /* MSI-X 벡터 상태 재확인 */
    if (csi2_dev->msix_base && total_ints == 0 && qemu_triggers < 10) {
        u32 ctrl = readl(csi2_dev->msix_base + 12); /* Vector 0 control */
        dev_info(&csi2_dev->pdev->dev, "📊 Vector 0 control: 0x%08x\n", ctrl);
        
        /* QEMU와 추가 통신 시도 */
        amd_csi2_trigger_qemu_interrupt(csi2_dev);
    }
    
    /* 다음 타이머 (5초 후) */
    if (csi2_dev->streaming) {
        mod_timer(&csi2_dev->debug_timer, jiffies + msecs_to_jiffies(5000));
    }
}

/* 🆕 Enhanced MSI-X 인터럽트 핸들러 */
static irqreturn_t amd_csi2_interrupt(int irq, void *dev_id)
{
    struct amd_csi2_dev *csi2_dev = dev_id;
    u32 isr_status, global_enable, ier_status;
    unsigned long flags;
    bool handled = false;
    
    /* 인터럽트 카운터 증가 */
    atomic_inc(&csi2_dev->total_interrupts);
    csi2_dev->last_interrupt_jiffies = jiffies;
    
    int total_count = atomic_read(&csi2_dev->total_interrupts);
    
    /* 🔥 CRITICAL: 모든 인터럽트를 상세히 로깅 */
    dev_info(&csi2_dev->pdev->dev, 
             "🔥🔥🔥 REAL MSI-X INTERRUPT #%d RECEIVED! IRQ=%d 🔥🔥🔥\n", 
             total_count, irq);
    
    /* 첫 번째 인터럽트에서 벡터 정보 확인 */
    if (total_count == 1) {
        dev_info(&csi2_dev->pdev->dev, "🎉 FIRST REAL INTERRUPT SUCCESS!\n");
        if (csi2_dev->vector_info_valid) {
            dev_info(&csi2_dev->pdev->dev,
                     "🎯 Vector Info: addr=0x%016llx, data=0x%08x\n",
                     csi2_dev->vector_address, csi2_dev->vector_data);
        }
        amd_csi2_analyze_msix_vectors(csi2_dev);
    }
    
    /* 레지스터 상태 읽기 */
    isr_status = readl(csi2_dev->mmio_base + CSI2_REG_ISR);
    global_enable = readl(csi2_dev->mmio_base + CSI2_REG_GLOBAL_INT_ENABLE);
    ier_status = readl(csi2_dev->mmio_base + CSI2_REG_IER);
    csi2_dev->last_isr_value = isr_status;
    
    dev_info(&csi2_dev->pdev->dev, 
             "🔔 IRQ %d (#%d): ISR=0x%08x, Global=0x%08x, IER=0x%08x\n", 
             irq, total_count, isr_status, global_enable, ier_status);
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    
    /* 모든 인터럽트를 프레임 인터럽트로 처리 */
    atomic_inc(&csi2_dev->frame_interrupts);
    int frame_count = atomic_read(&csi2_dev->frame_interrupts);
    
    dev_info(&csi2_dev->pdev->dev, "🎬 Frame interrupt #%d processed\n", frame_count);
    
    /* 캡처 스레드에게 신호 */
    complete(&csi2_dev->frame_completion);
    handled = true;
    
    /* ISR 클리어 */
    writel(0xFFFFFFFF, csi2_dev->mmio_base + CSI2_REG_ISR);
    wmb();
    
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    return IRQ_HANDLED;
}

/* 🆕 Enhanced 캡처 스레드 */
static int amd_csi2_capture_thread(void *data)
{
    struct amd_csi2_dev *csi2_dev = data;
    struct amd_csi2_buffer *buf;
    unsigned long flags;
    int frame_count = 0;
    
    dev_info(&csi2_dev->pdev->dev, "🎬 Capture thread started (Enhanced Debug mode)\n");
    
    /* 🆕 초기 MSI-X 벡터 분석 */
    msleep(100); /* MSI-X 초기화 대기 */
    amd_csi2_analyze_msix_vectors(csi2_dev);
    
    /* 🆕 첫 번째 QEMU 트리거 */
    msleep(500);
    amd_csi2_trigger_qemu_interrupt(csi2_dev);
    
    /* 🆕 디버그 타이머 시작 */
    mod_timer(&csi2_dev->debug_timer, jiffies + msecs_to_jiffies(5000));
    
    while (!kthread_should_stop() && !csi2_dev->thread_should_stop) {
        /* 프레임 인터럽트 대기 (30초 타임아웃) */
        long ret = wait_for_completion_interruptible_timeout(&csi2_dev->frame_completion, 
                                                           msecs_to_jiffies(30000));
        
        if (ret <= 0) {
            if (ret == 0) {
                dev_warn(&csi2_dev->pdev->dev, "⚠️  Frame completion timeout (30s)\n");
                
                /* 타임아웃 시 추가 QEMU 트리거 */
                amd_csi2_trigger_qemu_interrupt(csi2_dev);
            }
            continue;
        }
        
        /* 다음 프레임을 위해 completion 재초기화 */
        reinit_completion(&csi2_dev->frame_completion);
        
        spin_lock_irqsave(&csi2_dev->lock, flags);
        
        if (!list_empty(&csi2_dev->buf_list) && csi2_dev->streaming) {
            /* 다음 버퍼 가져오기 */
            buf = list_first_entry(&csi2_dev->buf_list, struct amd_csi2_buffer, list);
            list_del(&buf->list);
            
            /* 버퍼에 프레임 데이터 채우기 */
            buf->vb.vb2_buf.timestamp = ktime_get_ns();
            buf->vb.sequence = csi2_dev->sequence++;
            buf->vb.field = V4L2_FIELD_NONE;
            
            /* 프레임 크기 설정 */
            vb2_set_plane_payload(&buf->vb.vb2_buf, 0, 
                                  csi2_dev->format.fmt.pix.sizeimage);
            
            /* 버퍼 완료 */
            vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
            
            frame_count++;
            csi2_dev->frames_captured++;
            csi2_dev->last_frame_time = ktime_get();
            
            dev_info(&csi2_dev->pdev->dev, "📸 Frame #%d completed (%d bytes) - REAL INTERRUPT!\n",
                    frame_count, csi2_dev->format.fmt.pix.sizeimage);
                    
        } else {
            dev_warn(&csi2_dev->pdev->dev, 
                    "⚠️  Frame completion but no buffer available\n");
        }
        
        spin_unlock_irqrestore(&csi2_dev->lock, flags);
        
        if (try_to_freeze())
            continue;
    }
    
    /* 디버그 타이머 중지 */
    timer_delete_sync(&csi2_dev->debug_timer);
    
    dev_info(&csi2_dev->pdev->dev, 
        "🎬 Capture thread stopped after %d frames\n", frame_count);
    return 0;
}

/* Hardware initialization */
static int amd_csi2_hw_init(struct amd_csi2_dev *csi2_dev)
{
    u32 val;
    
    dev_info(&csi2_dev->pdev->dev, "🔧 Initializing CSI-2 hardware (Enhanced Debug)\n");
    
    /* 기본 초기화 */
    val = readl(csi2_dev->mmio_base + CSI2_REG_CORE_CONFIG);
    dev_info(&csi2_dev->pdev->dev, "Initial CORE_CONFIG: 0x%08x\n", val);
    
    writel(CONTROL_SOFT_RESET, csi2_dev->mmio_base + CSI2_REG_CORE_CONFIG);
    msleep(10);
    
    writel(CONTROL_CORE_ENABLE, csi2_dev->mmio_base + CSI2_REG_CORE_CONFIG);
    writel(0x3, csi2_dev->mmio_base + CSI2_REG_PROTOCOL_CONFIG);
    
    /* 인터럽트 설정 */
    writel(0xFFFFFFFF, csi2_dev->mmio_base + CSI2_REG_ISR);  /* 모든 비트 클리어 */
    writel(ISR_FRAME_RECEIVED, csi2_dev->mmio_base + CSI2_REG_IER);
    writel(0x1, csi2_dev->mmio_base + CSI2_REG_GLOBAL_INT_ENABLE);
    
    /* 설정 확인 */
    val = readl(csi2_dev->mmio_base + CSI2_REG_GLOBAL_INT_ENABLE);
    dev_info(&csi2_dev->pdev->dev, "Global interrupt enable: 0x%08x\n", val);
    
    val = readl(csi2_dev->mmio_base + CSI2_REG_IER);
    dev_info(&csi2_dev->pdev->dev, "Interrupt enable register: 0x%08x\n", val);
    
    /* 🆕 타이머 초기화 */
    timer_setup(&csi2_dev->debug_timer, amd_csi2_debug_timer_callback, 0);
    
    /* 🆕 카운터 초기화 */
    atomic_set(&csi2_dev->total_interrupts, 0);
    atomic_set(&csi2_dev->frame_interrupts, 0);
    atomic_set(&csi2_dev->test_interrupts, 0);
    atomic_set(&csi2_dev->qemu_trigger_count, 0);
    
    dev_info(&csi2_dev->pdev->dev, "✅ CSI-2 hardware initialized (Enhanced Debug)\n");
    
    return 0;
}

/* MSI-X setup */
static int amd_csi2_setup_msix(struct amd_csi2_dev *csi2_dev)
{
    struct pci_dev *pdev = csi2_dev->pdev;
    void __iomem *msix_base;
    int ret, nvec, i;
    
    dev_info(&pdev->dev, "🔧 Setting up MSI-X interrupts (Enhanced Debug v2.3)\n");
    
    if (!pci_find_capability(pdev, PCI_CAP_ID_MSIX)) {
        dev_err(&pdev->dev, "❌ MSI-X capability not found\n");
        return -ENODEV;
    }
    
    nvec = pci_msix_vec_count(pdev);
    dev_info(&pdev->dev, "📊 MSI-X vectors available: %d\n", nvec);
    
    if (nvec < 1) {
        dev_err(&pdev->dev, "❌ No MSI-X vectors available\n");
        return -ENODEV;
    }
    
    /* Clean up any existing IRQ vectors */
    pci_free_irq_vectors(pdev);
    
    /* Allocate MSI-X vectors */
    ret = pci_alloc_irq_vectors(pdev, 1, min(nvec, AMD_CSI2_MSIX_VECTORS), 
                                PCI_IRQ_MSIX | PCI_IRQ_NOLEGACY);
    if (ret < 0) {
        dev_err(&pdev->dev, "❌ Failed to allocate MSI-X vectors: %d\n", ret);
        return ret;
    }
    
    dev_info(&pdev->dev, "✅ Successfully allocated %d MSI-X vectors\n", ret);
    
    /* Map MSI-X vector table */
    msix_base = pci_ioremap_bar(pdev, 2);
    if (!msix_base) {
        dev_err(&pdev->dev, "❌ Failed to map MSI-X vector table\n");
        pci_free_irq_vectors(pdev);
        return -ENOMEM;
    }
    
    csi2_dev->msix_base = msix_base;
    dev_info(&pdev->dev, "📍 MSI-X vector table mapped at %p\n", msix_base);
    
    /* Unmask all vectors and store info */
    for (i = 0; i < ret; i++) {
        writel(0, msix_base + (i * 16) + 12);
        wmb();
        dev_info(&pdev->dev, "✅ Vector %d unmasked\n", i);
    }
    
    /* Store primary IRQ */
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
        
        if (csi2_dev->msix_base) {
            iounmap(csi2_dev->msix_base);
            csi2_dev->msix_base = NULL;
        }
        
        pci_free_irq_vectors(csi2_dev->pdev);
        csi2_dev->msix_enabled = false;
        csi2_dev->irq = 0;
    }
}

/* V4L2 buffer operations - 기존과 동일하나 간소화 */
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
    
    dev_info(&csi2_dev->pdev->dev, "🎬 Starting streaming (Enhanced Debug mode)\n");
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    csi2_dev->streaming = true;
    csi2_dev->sequence = 0;
    csi2_dev->frames_captured = 0;
    csi2_dev->thread_should_stop = false;
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    init_completion(&csi2_dev->frame_completion);
    
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
    
    /* 디버그 타이머 중지 */
    timer_delete_sync(&csi2_dev->debug_timer);
    
    /* 캡처 스레드 중지 */
    if (csi2_dev->capture_thread) {
        csi2_dev->thread_should_stop = true;
        complete(&csi2_dev->frame_completion);
        kthread_stop(csi2_dev->capture_thread);
        csi2_dev->capture_thread = NULL;
    }
    
    spin_lock_irqsave(&csi2_dev->lock, flags);
    csi2_dev->streaming = false;
    
    /* 대기 중인 버퍼들 반환 */
    list_for_each_entry_safe(buf, tmp, &csi2_dev->buf_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    
    spin_unlock_irqrestore(&csi2_dev->lock, flags);
    
    dev_info(&csi2_dev->pdev->dev, "✅ Streaming stopped (captured %u frames)\n",
             csi2_dev->frames_captured);
    
    /* 🆕 최종 디버그 통계 */
    dev_info(&csi2_dev->pdev->dev, 
             "📊 Final Debug Stats:\n"
             "   Real interrupts: %d\n"
             "   Frame interrupts: %d\n"
             "   QEMU triggers: %d\n"
             "   Vector valid: %s\n",
             atomic_read(&csi2_dev->total_interrupts),
             atomic_read(&csi2_dev->frame_interrupts),
             atomic_read(&csi2_dev->qemu_trigger_count),
             csi2_dev->vector_info_valid ? "YES" : "NO");
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

/* V4L2 device operations - 간소화 */
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
    strscpy(inp->name, "CSI2 Virtual Camera", sizeof(inp->name));
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
    
    dev_info(&pdev->dev, "🚨 NEW CODE VERSION 1149 - Enhanced MSI-X Debug\n");
    dev_info(&pdev->dev, "🚀 AMD CSI2 device found: %04x:%04x (v%s)\n", 
             pdev->vendor, pdev->device, DRIVER_VERSION);
    
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
    
    csi2_dev->mmio_base = pci_ioremap_bar(pdev, 0);
    if (!csi2_dev->mmio_base) {
        dev_err(&pdev->dev, "❌ Failed to map MMIO region\n");
        ret = -ENOMEM;
        goto err_release_regions;
    }
    
    dev_info(&pdev->dev, "📍 MMIO mapped at %p\n", csi2_dev->mmio_base);
    
    ret = amd_csi2_setup_msix(csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to setup MSI-X: %d\n", ret);
        goto err_unmap_mmio;
    }
    
    ret = request_irq(csi2_dev->irq, amd_csi2_interrupt, 0, DRIVER_NAME, csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to request IRQ %d: %d\n", csi2_dev->irq, ret);
        goto err_free_msix;
    }
    
    dev_info(&pdev->dev, "✅ IRQ %d registered successfully\n", csi2_dev->irq);
    
    ret = v4l2_device_register(&pdev->dev, &csi2_dev->v4l2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to register V4L2 device: %d\n", ret);
        goto err_free_irq;
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
    
    ret = amd_csi2_hw_init(csi2_dev);
    if (ret) {
        dev_err(&pdev->dev, "❌ Failed to initialize hardware: %d\n", ret);
        goto err_unregister_video;
    }
    
    csi2_dev->initialized = true;
    
    dev_info(&pdev->dev, "✅ AMD CSI2 v%s driver loaded successfully\n", DRIVER_VERSION);
    dev_info(&pdev->dev, "📺 Video device: %s\n", video_device_node_name(&csi2_dev->vdev));
    dev_info(&pdev->dev, "🎯 Enhanced MSI-X debug mode ready (IRQ %d)\n", csi2_dev->irq);
    
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
        if (csi2_dev->streaming) {
            amd_csi2_stop_streaming(&csi2_dev->queue);
        }
        
        /* 타이머 정리 */
        timer_delete_sync(&csi2_dev->debug_timer);
        
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
    pr_info("AMD CSI2 V4L2 Driver v%s (Enhanced MSI-X Debug)\n", DRIVER_VERSION);
    return pci_register_driver(&amd_csi2_pci_driver);
}

static void __exit amd_csi2_exit(void)
{
    pci_unregister_driver(&amd_csi2_pci_driver);
    pr_info("AMD CSI2 V4L2 Driver v%s unloaded\n", DRIVER_VERSION);
}

module_init(amd_csi2_init);
module_exit(amd_csi2_exit);

MODULE_DESCRIPTION("AMD MIPI CSI-2 RX V4L2 Driver - Enhanced MSI-X Debug");
MODULE_AUTHOR("AMD CSI2 Team");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);
