/*
 * AMD CSI-2 RX PCIe Device for QEMU with Ring Buffer
 * Version: 2.2 - Timer performance improvements
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include "hw/pci/pci.h"
#include "hw/pci/msix.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/qdev-properties.h"
#include "hw/resettable.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

#define TYPE_AMD_CSI2_RX_PCIE "csi2-pcie-sink"
#define AMD_CSI2_RX_PCIE(obj) OBJECT_CHECK(AmdCsi2RxPcieState, (obj), TYPE_AMD_CSI2_RX_PCIE)

#define XILINX_VENDOR_ID    0x10ee
#define CSI2_DEVICE_ID      0x9024

#define CSI2_MMIO_SIZE      0x2000
#define CSI2_MSIX_VECTORS   4
#define CSI2_RING_SIZE      32

/* Debug macro */
#define CSI2_DEBUG 

#ifdef CSI2_DEBUG
#define CSI2_DPRINTF(fmt, ...) \
    do { \
        fprintf(stderr, "CSI2: " fmt "\n", ## __VA_ARGS__); \
    } while (0)
#else
#define CSI2_DPRINTF(fmt, ...) do {} while (0)
#endif

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
#define CSI2_RING_HEAD_REG              0x10C  /* Driver writes, QEMU reads */
#define CSI2_RING_TAIL_REG              0x110  /* QEMU writes, Driver reads */
#define CSI2_RING_CTRL_REG              0x114
#define CSI2_RING_STATUS_REG            0x118

/* P2P DMA Registers */
#define CSI2_P2P_CONFIG_REG             0x400
#define CSI2_P2P_TARGET_LOW             0x404
#define CSI2_P2P_TARGET_HIGH            0x408
#define CSI2_P2P_ENABLE_REG             0x40C
#define CSI2_VERSION_REG                0x410

/* Format Registers */
#define CSI2_FORMAT_REG                 0x200
#define CSI2_WIDTH_REG                  0x204
#define CSI2_HEIGHT_REG                 0x208
#define CSI2_FPS_REG                    0x20C

/* Debug Register */
#define CSI2_DEBUG_REG                  0x300
#define CSI2_ERROR_COUNT_REG            0x304
#define CSI2_FRAME_COUNT_REG            0x308
#define CSI2_TIMER_STATS_REG            0x310
#define CSI2_TIMER_DRIFT_REG            0x314
#define CSI2_TIMER_CONFIG_REG           0x318  /* Timer configuration register */

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

/* P2P control bits */
#define P2P_CONFIG_ENABLE               (1U << 0)
#define P2P_CONFIG_CXL_MODE             (1U << 1)

/* Version */
#define CSI2_VERSION_P2P                0x0300

/* Test pattern types */
typedef enum {
    PATTERN_COLOR_BARS = 0,
    PATTERN_GRADIENT,
    PATTERN_MOVING_BARS,
    PATTERN_SOLID_COLOR,
    PATTERN_CHECKERBOARD
} TestPattern;

/* Ring buffer entry structure */
typedef struct RingEntry {
    uint64_t dma_addr;
    uint32_t size;
    uint32_t flags;
    uint64_t timestamp;
    uint32_t reserved[2];
} __attribute__((packed)) RingEntry;

/* Ring buffer structure in guest memory */
typedef struct RingBuffer {
    uint32_t magic;      /* 0x43534932 - "CSI2" */
    uint32_t version;    /* 0x00020000 - version 2.0 */
    uint32_t head;       /* Written by driver */
    uint32_t tail;       /* Written by QEMU */
    uint32_t size;       /* Number of entries */
    uint32_t entry_size; /* Size of each entry */
    uint32_t reserved[2];
    RingEntry entries[CSI2_RING_SIZE];
} __attribute__((packed)) RingBuffer;

typedef struct AmdCsi2RxPcieState {
    PCIDevice parent_obj;
    MemoryRegion mmio;
    
    /* Core state */
    bool enabled;
    bool streaming;
    
    /* Ring buffer */
    uint64_t ring_base_addr;
    uint32_t ring_size;
    uint32_t ring_head;  /* Local copy */
    uint32_t ring_tail;  /* Local copy */
    bool ring_enabled;
    
    /* Format */
    uint32_t width;
    uint32_t height;
    uint32_t format;
    uint32_t fps;
    uint32_t frame_size;
    
    /* Timers */
    QEMUTimer *frame_timer;
    
    /* Statistics */
    uint32_t frame_count;
    uint32_t error_count;
    uint32_t dropped_frames;
    
    /* Registers */
    uint32_t int_status;
    uint32_t int_enable;
    uint32_t global_int_en;
    
    /* Test pattern */
    TestPattern pattern;
    uint32_t pattern_offset;
    
    /* Performance tuning */
    bool batch_interrupts;
    uint32_t frames_since_interrupt;
    int64_t last_interrupt_time;
    
    /* Timer improvements */
    int64_t next_frame_time;
    int64_t frame_period_ns;
    bool use_realtime_clock;
    uint32_t timer_slack_ns;  /* Changed from int to uint32_t */
    int64_t stream_start_time;

    /* P2P DMA support */
    uint32_t p2p_config;
    uint64_t p2p_target_addr;
    bool p2p_enabled;

    /* Version */
    uint32_t hw_version;
} AmdCsi2RxPcieState;

/* Static buffer for frame generation */
static uint8_t *frame_generation_buffer = NULL;

/* Helper functions */
static void csi2_update_irq(AmdCsi2RxPcieState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    bool raise_irq = (s->int_status & s->int_enable) && s->global_int_en;
    
    if (raise_irq) {
        if (msix_enabled(pci_dev)) {
            msix_notify(pci_dev, 0);
        } else if (msi_enabled(pci_dev)) {
            msi_notify(pci_dev, 0);
        } else {
            pci_set_irq(pci_dev, 1);
        }
    } else {
        if (!msix_enabled(pci_dev) && !msi_enabled(pci_dev)) {
            pci_set_irq(pci_dev, 0);
        }
    }
}

static void csi2_set_interrupt(AmdCsi2RxPcieState *s, uint32_t mask)
{
    s->int_status |= mask;
    csi2_update_irq(s);
}

static void csi2_clear_interrupt(AmdCsi2RxPcieState *s, uint32_t mask)
{
    s->int_status &= ~mask;
    csi2_update_irq(s);
}

static int csi2_ring_is_full(uint32_t head, uint32_t tail, uint32_t size)
{
    return ((head + 1) % size) == tail;
}

static int csi2_ring_is_empty(uint32_t head, uint32_t tail)
{
    return head == tail;
}

static int csi2_ring_count(uint32_t head, uint32_t tail, uint32_t size)
{
    if (head >= tail) {
        return head - tail;
    } else {
        return size - tail + head;
    }
}

static void csi2_update_ring_status(AmdCsi2RxPcieState *s)
{
    uint32_t status = 0;
    
    if (s->ring_enabled) {
        status |= RING_STATUS_READY;
    }
    
    if (csi2_ring_is_full(s->ring_head, s->ring_tail, s->ring_size)) {
        status |= RING_STATUS_FULL;
    }
    
    if (csi2_ring_is_empty(s->ring_head, s->ring_tail)) {
        status |= RING_STATUS_EMPTY;
    }
}

static int csi2_read_ring_head(AmdCsi2RxPcieState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint32_t head;
    uint32_t old_head = s->ring_head;

    if (!s->ring_base_addr) {
        return -1;
    }

    /* Read head from ring buffer in guest memory */
    if (pci_dma_read(pci_dev, s->ring_base_addr + offsetof(RingBuffer, head),
                     &head, sizeof(head)) != 0) {
        CSI2_DPRINTF("Failed to read ring head");
        return -1;
    }

    head = le32_to_cpu(head);
    if (head >= s->ring_size) {
        CSI2_DPRINTF("Invalid ring head: %u (size=%u)", head, s->ring_size);
        return -1;
    }

    if (old_head != head) {
        CSI2_DPRINTF("Ring head updated: %u -> %u", old_head, head);
    }

    s->ring_head = head;
    return 0;
}

static int csi2_write_ring_tail(AmdCsi2RxPcieState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint32_t tail_le;
    
    if (!s->ring_base_addr) {
        return -1;
    }
    
    tail_le = cpu_to_le32(s->ring_tail);
    
    /* Write tail to ring buffer in guest memory */
    if (pci_dma_write(pci_dev, s->ring_base_addr + offsetof(RingBuffer, tail),
                      &tail_le, sizeof(tail_le)) != 0) {
        CSI2_DPRINTF("Failed to write ring tail");
        return -1;
    }
    
    return 0;
}

/* Generate color bars pattern */
static void generate_color_bars(uint8_t *buffer, uint32_t width, uint32_t height)
{
    static const uint32_t colors[] = {
        0xFF0080FF,  /* White */
        0xE10080E1,  /* Yellow */
        0xB200B2B2,  /* Cyan */
        0x950095A0,  /* Green */
        0x69006969,  /* Magenta */
        0x4C004C5A,  /* Red */
        0x1D001D3F,  /* Blue */
        0x00800080   /* Black */
    };
    
    uint32_t bar_width = width / 8;
    uint16_t *data = (uint16_t *)buffer;
    
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            uint32_t bar = x / bar_width;
            if (bar >= 8) bar = 7;
            
            uint32_t color = colors[bar];
            uint8_t y_val = (color >> 16) & 0xFF;
            uint8_t u_val = (color >> 8) & 0xFF;
            uint8_t v_val = color & 0xFF;
            
            /* YUYV format */
            if (x & 1) {
                *data++ = (v_val << 8) | y_val;
            } else {
                *data++ = (u_val << 8) | y_val;
            }
        }
    }
}

/* Generate gradient pattern */
static void generate_gradient(uint8_t *buffer, uint32_t width, uint32_t height, uint32_t offset)
{
    uint16_t *data = (uint16_t *)buffer;
    
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            uint8_t y_val = ((x + offset) * 255 / width) & 0xFF;
            uint8_t u_val = (y * 255 / height) & 0xFF;
            uint8_t v_val = 128;
            
            /* YUYV format */
            if (x & 1) {
                *data++ = (v_val << 8) | y_val;
            } else {
                *data++ = (u_val << 8) | y_val;
            }
        }
    }
}

/* Generate moving bars pattern */
static void generate_moving_bars(uint8_t *buffer, uint32_t width, uint32_t height, uint32_t offset)
{
    uint16_t *data = (uint16_t *)buffer;
    uint32_t bar_width = 64;
    uint32_t shifted_offset = offset % (bar_width * 2);
    
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            uint32_t pos = (x + shifted_offset) % (bar_width * 2);
            uint8_t y_val = (pos < bar_width) ? 235 : 16;
            uint8_t u_val = 128;
            uint8_t v_val = 128;
            
            /* YUYV format */
            if (x & 1) {
                *data++ = (v_val << 8) | y_val;
            } else {
                *data++ = (u_val << 8) | y_val;
            }
        }
    }
}

/* Generate frame with frame number overlay */
static void generate_test_frame(void *buffer, uint32_t size, uint32_t frame_num, AmdCsi2RxPcieState *s)
{
    /* Generate base pattern */
    switch (s->pattern) {
    case PATTERN_COLOR_BARS:
        generate_color_bars(buffer, s->width, s->height);
        break;
    case PATTERN_GRADIENT:
        generate_gradient(buffer, s->width, s->height, s->pattern_offset);
        break;
    case PATTERN_MOVING_BARS:
        generate_moving_bars(buffer, s->width, s->height, s->pattern_offset);
        break;
    default:
        /* Solid gray */
        memset(buffer, 0x80, size);
        break;
    }
    
    /* Update pattern offset for animation */
    s->pattern_offset += 4;
    
    /* Overlay frame number in top-left corner */
    uint16_t *data = (uint16_t *)buffer;
    uint32_t digits[8];
    int num_digits = 0;
    uint32_t temp = frame_num;
    
    /* Extract digits */
    do {
        digits[num_digits++] = temp % 10;
        temp /= 10;
    } while (temp > 0 && num_digits < 8);
    
    /* Draw digits */
    for (int i = 0; i < num_digits && i < 8; i++) {
        int digit = digits[num_digits - 1 - i];
        int x_start = 10 + i * 12;
        int y_start = 10;
        
        /* Simple digit rendering (3x5 font) */
        for (int y = 0; y < 5; y++) {
            for (int x = 0; x < 3; x++) {
                int px = x_start + x * 2;
                int py = y_start + y * 2;
                if (px < s->width && py < s->height) {
                    int offset = py * s->width + px;
                    uint8_t val = (digit == 0 || digit == 8) ? 235 : 
                                 ((x == 1 || y == 2) ? 235 : 16);
                    data[offset] = (128 << 8) | val;
                }
            }
        }
    }
}

static void csi2_process_frame(AmdCsi2RxPcieState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    RingEntry entry;
    int64_t start_time, end_time;
    
    start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    /* Read current ring head from guest memory */
    if (csi2_read_ring_head(s) < 0) {
        s->error_count++;
        return;
    }
    
    /* Check if we have a buffer available */
    if (csi2_ring_is_empty(s->ring_head, s->ring_tail)) {
        s->dropped_frames++;
        CSI2_DPRINTF("No buffer available, frame dropped (head=%u, tail=%u)", 
                     s->ring_head, s->ring_tail);
        return;
    }
    
    /* Read entry from ring */
    uint64_t entry_addr = s->ring_base_addr + 
                          offsetof(RingBuffer, entries) + 
                          (s->ring_tail * sizeof(RingEntry));
    
    if (pci_dma_read(pci_dev, entry_addr, &entry, sizeof(entry)) != 0) {
        CSI2_DPRINTF("Failed to read ring entry");
        s->error_count++;
        return;
    }
    
    /* Convert from little endian */
    entry.dma_addr = le64_to_cpu(entry.dma_addr);
    entry.size = le32_to_cpu(entry.size);
    
    CSI2_DPRINTF("Processing frame %u: buffer at 0x%lx, size=%u (head=%u, tail=%u)", 
                 s->frame_count, entry.dma_addr, entry.size, 
                 s->ring_head, s->ring_tail);
    
    /* Validate entry */
    if (entry.size != s->frame_size) {
        CSI2_DPRINTF("Invalid buffer size: %u (expected %u)", 
                     entry.size, s->frame_size);
        s->error_count++;
        return;
    }
    
    /* Allocate frame buffer if needed */
    if (!frame_generation_buffer) {
        frame_generation_buffer = g_malloc(4096 * 2160 * 2); /* Max 4K resolution */
    }
    
    /* Generate frame data */
    generate_test_frame(frame_generation_buffer, entry.size, s->frame_count, s);

    /* Write frame data based on P2P configuration */
    if (s->p2p_enabled && s->p2p_target_addr) {
        /* P2P DMA 모드 - CXL 메모리에 직접 쓰기 */
        CSI2_DPRINTF("Using P2P DMA to CXL memory at 0x%lx", s->p2p_target_addr);
        
        /* P2P DMA 시뮬레이션
         * 실제로는 PCIe 스위치를 통해 직접 전송되지만,
         * QEMU에서는 메모리 접근으로 시뮬레이션
         */
        if (pci_dma_write(pci_dev, entry.dma_addr, frame_generation_buffer, entry.size) != 0) {
            CSI2_DPRINTF("Failed to write frame data via P2P");
            s->error_count++;
            return;
        }
    } else {
        /* 일반 DMA 모드 */
        if (pci_dma_write(pci_dev, entry.dma_addr, frame_generation_buffer, entry.size) != 0) {
            CSI2_DPRINTF("Failed to write frame data");
            s->error_count++;
            return;
        }
    }    
    
    /* Update entry with completion info */
    entry.flags = cpu_to_le32(1);  /* Mark as complete */
    entry.timestamp = cpu_to_le64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    
    /* Write back entry */
    if (pci_dma_write(pci_dev, entry_addr, &entry, sizeof(entry)) != 0) {
        CSI2_DPRINTF("Failed to write back ring entry");
        s->error_count++;
        return;
    }
    
    /* Update tail */
    s->ring_tail = (s->ring_tail + 1) % s->ring_size;
    if (csi2_write_ring_tail(s) < 0) {
        s->error_count++;
        return;
    }
    
    CSI2_DPRINTF("Updating tail register to %u", s->ring_tail);
    
    s->frame_count++;
    s->frames_since_interrupt++;
    
    /* Batch interrupts for better performance */
    if (s->batch_interrupts) {
        /* Trigger interrupt every 4 frames or if ring is getting full */
        int ring_occupancy = csi2_ring_count(s->ring_head, s->ring_tail, s->ring_size);
        if (s->frames_since_interrupt >= 4 || 
            ring_occupancy >= (s->ring_size * 3 / 4)) {
            csi2_set_interrupt(s, CSI2_INT_FRAME_DONE);
            s->frames_since_interrupt = 0;
        }
    } else {
        /* Trigger interrupt for every frame */
        csi2_set_interrupt(s, CSI2_INT_FRAME_DONE);
    }
    
    end_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    /* Check processing time */
    if ((end_time - start_time) > s->frame_period_ns / 2) {
        CSI2_DPRINTF("Frame processing took too long: %ld ms", 
                     (end_time - start_time) / 1000000);
    }
    
    CSI2_DPRINTF("Frame %u completed successfully", s->frame_count - 1);
}

static void csi2_frame_timer(void *opaque)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(opaque);
    int64_t now;
    
    if (!s->streaming || !s->ring_enabled) {
        return;
    }
    
    /* Get current time */
    if (s->use_realtime_clock) {
        now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    } else {
        now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
    
    /* Read current ring head */
    if (csi2_read_ring_head(s) < 0) {
        s->error_count++;
        goto reschedule;
    }
    
    /* Process frame */
    csi2_process_frame(s);
    
    /* Update ring status */
    csi2_update_ring_status(s);
    
reschedule:
    /* Schedule next frame with drift correction */
    if (s->streaming) {
        /* Initialize next frame time */
        if (s->next_frame_time == 0) {
            s->next_frame_time = now + s->frame_period_ns;
        } else {
            /* Increment by fixed period to avoid drift */
            s->next_frame_time += s->frame_period_ns;
            
            /* Re-sync if too far behind */
            if (s->next_frame_time < now - s->frame_period_ns) {
                CSI2_DPRINTF("Timer drift detected, resyncing");
                s->next_frame_time = now + s->frame_period_ns;
            }
        }
        
        /* Reschedule timer */
        timer_mod_ns(s->frame_timer, s->next_frame_time);
    }
}

static void csi2_start_streaming(AmdCsi2RxPcieState *s)
{
    if (s->streaming) {
        return;
    }
    
    CSI2_DPRINTF("Starting streaming: %ux%u @ %u fps", 
                 s->width, s->height, s->fps);
    
    s->streaming = true;
    s->frame_count = 0;
    s->dropped_frames = 0;
    s->frames_since_interrupt = 0;
    s->pattern_offset = 0;
    s->last_interrupt_time = 0;
    s->next_frame_time = 0;
    s->stream_start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    /* Calculate frame period */
    s->frame_period_ns = 1000000000LL / s->fps;
    
    /* Enable batch interrupts for better performance */
    s->batch_interrupts = false;  /* Disabled for accurate timing test */
    
    /* Start frame timer immediately */
    if (s->use_realtime_clock) {
        timer_mod_ns(s->frame_timer, 
                     qemu_clock_get_ns(QEMU_CLOCK_REALTIME));
    } else {
        timer_mod_ns(s->frame_timer, 
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
}

static void csi2_stop_streaming(AmdCsi2RxPcieState *s)
{
    if (!s->streaming) {
        return;
    }
    
    CSI2_DPRINTF("Stopping streaming");
    
    s->streaming = false;
    timer_del(s->frame_timer);
    
    /* Clear any pending interrupts */
    s->frames_since_interrupt = 0;
}

static uint64_t csi2_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(opaque);
    uint64_t val = 0;
    int64_t now;
    
    switch (addr) {
    case CSI2_CORE_CONFIG_REG:
        val = s->enabled ? CSI2_CORE_CONFIG_ENABLE : 0;
        break;
        
    case CSI2_CORE_STATUS_REG:
        val = (s->streaming << 0) | (s->ring_enabled << 1);
        break;
        
    case CSI2_INT_STATUS_REG:
        val = s->int_status;
        break;
        
    case CSI2_INT_ENABLE_REG:
        val = s->int_enable;
        break;
        
    case CSI2_GLOBAL_INT_EN_REG:
        val = s->global_int_en;
        break;
        
    case CSI2_RING_BASE_LOW:
        val = s->ring_base_addr & 0xFFFFFFFF;
        break;
        
    case CSI2_RING_BASE_HIGH:
        val = s->ring_base_addr >> 32;
        break;
        
    case CSI2_RING_SIZE_REG:
        val = s->ring_size;
        break;
        
    case CSI2_RING_HEAD_REG:
        val = s->ring_head;
        break;
        
    case CSI2_RING_TAIL_REG:
        val = s->ring_tail;
        CSI2_DPRINTF("Read ring tail register: %lu", (unsigned long)val);
        break;
        
    case CSI2_RING_STATUS_REG:
        val = 0;
        if (s->ring_enabled) val |= RING_STATUS_READY;
        if (csi2_ring_is_full(s->ring_head, s->ring_tail, s->ring_size)) {
            val |= RING_STATUS_FULL;
        }
        if (csi2_ring_is_empty(s->ring_head, s->ring_tail)) {
            val |= RING_STATUS_EMPTY;
        }
        break;
        
    case CSI2_TIMER_CONFIG_REG:
        /* Timer configuration register */
        s->use_realtime_clock = (val & 1) ? true : false;
        s->batch_interrupts = (val & 2) ? true : false;
        CSI2_DPRINTF("Timer config: realtime=%d, batch_irq=%d",
                     s->use_realtime_clock, s->batch_interrupts);
        break;
        
    case CSI2_FRAME_COUNT_REG:
        val = s->frame_count;
        break;
        
    case CSI2_ERROR_COUNT_REG:
        val = s->error_count;
        break;
        
    case CSI2_WIDTH_REG:
        val = s->width;
        break;
        
    case CSI2_HEIGHT_REG:
        val = s->height;
        break;
        
    case CSI2_FPS_REG:
        val = s->fps;
        break;
        
    case CSI2_TIMER_STATS_REG:
        /* Calculate actual FPS */
        if (s->frame_count > 0 && s->streaming) {
            now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            int64_t elapsed = now - s->stream_start_time;
            if (elapsed > 0) {
                val = (s->frame_count * 1000000000LL) / elapsed;
            }
        }
        break;
        
    case CSI2_TIMER_DRIFT_REG:
        /* Timer drift in milliseconds */
        if (s->next_frame_time > 0) {
            now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            val = (s->next_frame_time - now) / 1000000;
        }
        break;

    case CSI2_P2P_CONFIG_REG:
        val = s->p2p_config;
        break;

    case CSI2_P2P_TARGET_LOW:
        val = s->p2p_target_addr & 0xFFFFFFFF;
        break;

    case CSI2_P2P_TARGET_HIGH:
        val = s->p2p_target_addr >> 32;
        break;

    case CSI2_P2P_ENABLE_REG:
        val = s->p2p_enabled ? 1 : 0;
        break;

    case CSI2_VERSION_REG:
        val = s->hw_version;
        break;
        
    default:
        CSI2_DPRINTF("Read from unknown register 0x%lx", addr);
        break;
    }
    
    return val;
}

static void csi2_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(opaque);
    
    CSI2_DPRINTF("MMIO Write: addr=0x%04lx, val=0x%08lx", addr, val);
    
    switch (addr) {
    case CSI2_CORE_CONFIG_REG:
        if (val & CSI2_CORE_CONFIG_RESET) {
            /* Reset core */
            csi2_stop_streaming(s);
            s->enabled = false;
            s->int_status = 0;
            s->error_count = 0;
            s->frame_count = 0;
        } else if (val & CSI2_CORE_CONFIG_ENABLE) {
            s->enabled = true;
            if (s->ring_enabled) {
                csi2_start_streaming(s);
            }
        } else {
            s->enabled = false;
            csi2_stop_streaming(s);
        }
        break;
        
    case CSI2_INT_STATUS_REG:
        /* Write 1 to clear */
        csi2_clear_interrupt(s, val);
        break;
        
    case CSI2_INT_ENABLE_REG:
        s->int_enable = val;
        csi2_update_irq(s);
        break;
        
    case CSI2_GLOBAL_INT_EN_REG:
        s->global_int_en = val & 1;
        csi2_update_irq(s);
        break;
        
    case CSI2_RING_BASE_LOW:
        s->ring_base_addr = (s->ring_base_addr & 0xFFFFFFFF00000000ULL) | val;
        break;
        
    case CSI2_RING_BASE_HIGH:
        s->ring_base_addr = (s->ring_base_addr & 0xFFFFFFFFULL) | (val << 32);
        break;
        
    case CSI2_RING_SIZE_REG:
        if (val > 0 && val <= CSI2_RING_SIZE) {
            s->ring_size = val;
        }
        break;
        
    case CSI2_RING_CTRL_REG:
        if (val & RING_CTRL_RESET) {
            s->ring_enabled = false;
            s->ring_head = 0;
            s->ring_tail = 0;
            csi2_stop_streaming(s);
        } else if (val & RING_CTRL_ENABLE) {
            if (s->ring_base_addr && s->ring_size > 0) {
                s->ring_enabled = true;
                s->ring_head = 0;
                s->ring_tail = 0;
                CSI2_DPRINTF("Ring buffer enabled: base=0x%lx, size=%u",
                            s->ring_base_addr, s->ring_size);
                if (s->enabled) {
                    csi2_start_streaming(s);
                }
            }
        }
        break;
        
    case CSI2_WIDTH_REG:
        s->width = val;
        s->frame_size = s->width * s->height * 2;  /* Assuming YUV422 */
        break;
        
    case CSI2_HEIGHT_REG:
        s->height = val;
        s->frame_size = s->width * s->height * 2;
        break;
        
    case CSI2_FPS_REG:
        if (val > 0 && val <= 120) {
            s->fps = val;
            s->frame_period_ns = 1000000000LL / s->fps;
        }
        break;

    /* P2P DMA registers */
    case CSI2_P2P_CONFIG_REG:
        s->p2p_config = val;
        if (val & P2P_CONFIG_ENABLE) {
            s->p2p_enabled = true;
            CSI2_DPRINTF("P2P DMA enabled (CXL mode: %s)",
                        (val & P2P_CONFIG_CXL_MODE) ? "yes" : "no");
        } else {
            s->p2p_enabled = false;
            CSI2_DPRINTF("P2P DMA disabled");
        }
        break;

    case CSI2_P2P_TARGET_LOW:
        s->p2p_target_addr = (s->p2p_target_addr & 0xFFFFFFFF00000000ULL) | val;
        CSI2_DPRINTF("P2P target low: 0x%08lx", val);
        break;

    case CSI2_P2P_TARGET_HIGH:
        s->p2p_target_addr = (s->p2p_target_addr & 0xFFFFFFFFULL) | (val << 32);
        CSI2_DPRINTF("P2P target high: 0x%08lx, full address: 0x%lx",
                    val, s->p2p_target_addr);
        break;

    case CSI2_P2P_ENABLE_REG:
        s->p2p_enabled = (val & 1) ? true : false;
        CSI2_DPRINTF("P2P enable: %d", s->p2p_enabled);
        break;
        
    default:
        CSI2_DPRINTF("Write to unknown register 0x%lx", addr);
        break;
    }
}

static const MemoryRegionOps csi2_mmio_ops = {
    .read = csi2_mmio_read,
    .write = csi2_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void amd_csi2_rx_pcie_reset(Object *obj, ResetType type)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(obj);
    
    CSI2_DPRINTF("Device reset");
    
    /* Stop streaming */
    csi2_stop_streaming(s);
    
    /* Reset state */
    s->enabled = false;
    s->ring_enabled = false;
    s->ring_base_addr = 0;
    s->ring_size = 0;
    s->ring_head = 0;
    s->ring_tail = 0;
    s->int_status = 0;
    s->int_enable = 0;
    s->global_int_en = 0;
    s->frame_count = 0;
    s->error_count = 0;
    s->dropped_frames = 0;
    s->pattern = PATTERN_COLOR_BARS;
    s->pattern_offset = 0;
    s->batch_interrupts = false;
    s->frames_since_interrupt = 0;
    s->last_interrupt_time = 0;
    s->next_frame_time = 0;
    s->stream_start_time = 0;

    /* Reset P2P configuration */
    s->p2p_enabled = false;
    s->p2p_config = 0;
    s->p2p_target_addr = 0;
}

static void amd_csi2_rx_pcie_realize(PCIDevice *pci_dev, Error **errp)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(pci_dev);
    int ret;
    
    CSI2_DPRINTF("Device realize");
    
    /* Initialize state */
    s->enabled = false;
    s->streaming = false;
    s->ring_enabled = false;
    s->width = 1920;
    s->height = 1080;
    s->fps = 30;
    s->frame_size = s->width * s->height * 2;
    s->pattern = PATTERN_COLOR_BARS;
    s->frame_period_ns = 1000000000LL / s->fps;
    s->use_realtime_clock = false;
    s->timer_slack_ns = 1000000;  /* 1ms - ensure this is uint32_t */

    /* Initialize P2P support */
    s->p2p_enabled = false;
    s->p2p_config = 0;
    s->p2p_target_addr = 0;
    s->hw_version = CSI2_VERSION_P2P;  /* Support P2P */
    
    /* Set up PCI configuration */
    pci_config_set_vendor_id(pci_dev->config, XILINX_VENDOR_ID);
    pci_config_set_device_id(pci_dev->config, CSI2_DEVICE_ID);
    pci_config_set_class(pci_dev->config, PCI_CLASS_MULTIMEDIA_OTHER);
    pci_config_set_interrupt_pin(pci_dev->config, 1);
    
    /* Register MMIO BAR */
    memory_region_init_io(&s->mmio, OBJECT(s), &csi2_mmio_ops, s, 
                          "csi2-mmio", CSI2_MMIO_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
    
    /* Initialize MSI-X */
    ret = msix_init_exclusive_bar(pci_dev, CSI2_MSIX_VECTORS, 2, errp);
    if (ret < 0) {
        error_setg(errp, "Failed to initialize MSI-X: %d", ret);
        return;
    }
    
    /* Enable all MSI-X vectors */
    for (int i = 0; i < CSI2_MSIX_VECTORS; i++) {
        msix_vector_use(pci_dev, i);
    }
    
    /* Create frame timer */
    s->frame_timer = timer_new_ns(s->use_realtime_clock ? QEMU_CLOCK_REALTIME : QEMU_CLOCK_VIRTUAL,
                                  csi2_frame_timer, s);
    if (!s->frame_timer) {
        error_setg(errp, "Failed to create frame timer");
        msix_uninit_exclusive_bar(pci_dev);
        return;
    }
    
    CSI2_DPRINTF("AMD CSI-2 RX PCIe device realized successfully");
}

static void amd_csi2_rx_pcie_exit(PCIDevice *pci_dev)
{
    AmdCsi2RxPcieState *s = AMD_CSI2_RX_PCIE(pci_dev);
    
    CSI2_DPRINTF("Device exit");
    
    /* Stop streaming */
    csi2_stop_streaming(s);
    
    /* Delete timer */
    if (s->frame_timer) {
        timer_free(s->frame_timer);
    }
    
    /* Free frame buffer */
    if (frame_generation_buffer) {
        g_free(frame_generation_buffer);
        frame_generation_buffer = NULL;
    }
    
    /* Clean up MSI-X */
    msix_uninit_exclusive_bar(pci_dev);
}

/* Properties are causing issues, so we'll use static configuration for now */
/*
static Property amd_csi2_rx_pcie_properties[] = {
    DEFINE_PROP_BOOL("use-realtime", AmdCsi2RxPcieState, use_realtime_clock, false),
    DEFINE_PROP_UINT32("timer-slack", AmdCsi2RxPcieState, timer_slack_ns, 1000000),
    DEFINE_PROP_BOOL("batch-irq", AmdCsi2RxPcieState, batch_interrupts, false),
    {}
};
*/

static const VMStateDescription vmstate_amd_csi2_rx_pcie = {
    .name = TYPE_AMD_CSI2_RX_PCIE,
    .version_id = 3,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, AmdCsi2RxPcieState),
        VMSTATE_BOOL(enabled, AmdCsi2RxPcieState),
        VMSTATE_BOOL(streaming, AmdCsi2RxPcieState),
        VMSTATE_BOOL(ring_enabled, AmdCsi2RxPcieState),
        VMSTATE_UINT64(ring_base_addr, AmdCsi2RxPcieState),
        VMSTATE_UINT32(ring_size, AmdCsi2RxPcieState),
        VMSTATE_UINT32(ring_head, AmdCsi2RxPcieState),
        VMSTATE_UINT32(ring_tail, AmdCsi2RxPcieState),
        VMSTATE_UINT32(width, AmdCsi2RxPcieState),
        VMSTATE_UINT32(height, AmdCsi2RxPcieState),
        VMSTATE_UINT32(fps, AmdCsi2RxPcieState),
        VMSTATE_UINT32(frame_count, AmdCsi2RxPcieState),
        VMSTATE_UINT32(error_count, AmdCsi2RxPcieState),
	VMSTATE_UINT32(p2p_config, AmdCsi2RxPcieState),
        VMSTATE_UINT64(p2p_target_addr, AmdCsi2RxPcieState),
        VMSTATE_BOOL(p2p_enabled, AmdCsi2RxPcieState),
        VMSTATE_UINT32(hw_version, AmdCsi2RxPcieState),
        VMSTATE_END_OF_LIST()
    }
};

static void amd_csi2_rx_pcie_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    
    pc->realize = amd_csi2_rx_pcie_realize;
    pc->exit = amd_csi2_rx_pcie_exit;
    pc->vendor_id = XILINX_VENDOR_ID;
    pc->device_id = CSI2_DEVICE_ID;
    pc->class_id = PCI_CLASS_MULTIMEDIA_OTHER;
    rc->phases.hold = amd_csi2_rx_pcie_reset;
    dc->vmsd = &vmstate_amd_csi2_rx_pcie;
    /* Properties removed to avoid assertion error */
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo amd_csi2_rx_pcie_info = {
    .name = TYPE_AMD_CSI2_RX_PCIE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(AmdCsi2RxPcieState),
    .class_init = amd_csi2_rx_pcie_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

static void amd_csi2_rx_pcie_register_types(void)
{
    type_register_static(&amd_csi2_rx_pcie_info);
}

type_init(amd_csi2_rx_pcie_register_types);
