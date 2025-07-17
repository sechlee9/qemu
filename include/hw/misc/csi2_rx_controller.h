#ifndef HW_CSI_CSI2_RX_CONTROLLER_H
#define HW_CSI_CSI2_RX_CONTROLLER_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qapi/error.h"

#define TYPE_CSI2_RX_CONTROLLER "csi2-rx-controller"
OBJECT_DECLARE_SIMPLE_TYPE(CSI2RxController, CSI2_RX_CONTROLLER)

#define CSI2_MAX_LANES      4
#define CSI2_MAX_VC         16
#define CSI2_FIFO_DEPTH     32

#define CSI2_INT_FRAME_RECEIVED           (1 << 31)

typedef struct CSI2VirtualChannel {
    uint32_t frame_number;
    uint32_t line_number;
    uint32_t byte_count;
    uint32_t data_type;
    bool frame_active;
} CSI2VirtualChannel;

typedef struct CSI2RxController {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;

    uint32_t core_config;
    uint32_t protocol_config;
    uint32_t core_status;
    uint32_t global_int_enable;
    uint32_t int_status;
    uint32_t int_enable;
    uint32_t dynamic_vc_sel;
    uint32_t generic_short_pkt;
    uint32_t vcx_frame_error;
    uint32_t clock_lane_info;
    uint32_t lane_info[CSI2_MAX_LANES];
    uint32_t img_info1[CSI2_MAX_VC];
    uint32_t img_info2[CSI2_MAX_VC];

    CSI2VirtualChannel vc[CSI2_MAX_VC];
    uint32_t packet_count;
    uint32_t active_lanes;
    uint32_t max_lanes;
    uint32_t line_rate;

    QEMUTimer *frame_timer;

    uint32_t num_lanes;
    uint32_t default_line_rate;
    bool vfb_enabled;
    bool embedded_enabled;
    uint32_t pixels_per_clock;
} CSI2RxController;

uint64_t csi2_rx_controller_read(void *opaque, hwaddr offset, unsigned size);
void csi2_rx_controller_write(void *opaque, hwaddr offset, uint64_t value, unsigned size);

#endif

