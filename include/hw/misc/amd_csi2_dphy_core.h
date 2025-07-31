/*
 * amd_csi2_dphy_core.h - Kernel version  
 * AMD MIPI D-PHY/RX C-PHY/D-PHY Core Register Definitions
 * Based on AMD CSI-2 RX Subsystem Product Guide (PG232)
 * 
 * Register Space: 4KB (0x1000 - 0x1FFF)
 * Base Address Offset: 0x1000 from subsystem base
 */

#ifndef AMD_CSI2_DPHY_CORE_H
#define AMD_CSI2_DPHY_CORE_H

#include <linux/types.h>

/* QEMU compatibility definitions */
#ifndef BIT
#define BIT(nr)                     (1UL << (nr))
#endif

#ifndef GENMASK
#define GENMASK(h, l) \
    (((~0UL) << (l)) & (~0UL >> (sizeof(unsigned long) * 8 - 1 - (h))))
#endif

/* D-PHY Register Base Offset */
#define DPHY_REG_BASE_OFFSET                    0x1000

/* Note: Specific D-PHY register definitions vary by device family
 * For UltraScale+ and Versal devices, refer to MIPI D-PHY LogiCORE IP Product Guide (PG202)
 * For Versal Premium Gen 2 devices, refer to MIPI RX C-PHY/D-PHY LogiCORE IP Product Guide (PG435)
 */

/* Common D-PHY Control and Status Registers */
#define DPHY_CONTROL_REG                        (DPHY_REG_BASE_OFFSET + 0x00)
#define DPHY_STATUS_REG                         (DPHY_REG_BASE_OFFSET + 0x04)
#define DPHY_INIT_REG                           (DPHY_REG_BASE_OFFSET + 0x08)
#define DPHY_WAKEUP_REG                         (DPHY_REG_BASE_OFFSET + 0x0C)

/* Lane Configuration Registers */
#define DPHY_LANE_CONFIG_REG                    (DPHY_REG_BASE_OFFSET + 0x10)
#define DPHY_LANE_STATUS_REG                    (DPHY_REG_BASE_OFFSET + 0x14)

/* Timing Configuration Registers */
#define DPHY_HS_SETTLE_REG                      (DPHY_REG_BASE_OFFSET + 0x20)
#define DPHY_HS_TIMEOUT_REG                     (DPHY_REG_BASE_OFFSET + 0x24)
#define DPHY_ESC_TIMEOUT_REG                    (DPHY_REG_BASE_OFFSET + 0x28)

/* Clock Management Registers */
#define DPHY_CLK_LANE_REG                       (DPHY_REG_BASE_OFFSET + 0x30)
#define DPHY_DATA_LANE0_REG                     (DPHY_REG_BASE_OFFSET + 0x34)
#define DPHY_DATA_LANE1_REG                     (DPHY_REG_BASE_OFFSET + 0x38)
#define DPHY_DATA_LANE2_REG                     (DPHY_REG_BASE_OFFSET + 0x3C)
#define DPHY_DATA_LANE3_REG                     (DPHY_REG_BASE_OFFSET + 0x40)

/* Interrupt and Error Registers */
#define DPHY_INT_STATUS_REG                     (DPHY_REG_BASE_OFFSET + 0x50)
#define DPHY_INT_ENABLE_REG                     (DPHY_REG_BASE_OFFSET + 0x54)
#define DPHY_ERROR_STATUS_REG                   (DPHY_REG_BASE_OFFSET + 0x58)

/* Calibration Registers (7 series specific) */
#define DPHY_IDELAY_TAP_REG                     (DPHY_REG_BASE_OFFSET + 0x60)
#define DPHY_IDELAY_CTRL_REG                    (DPHY_REG_BASE_OFFSET + 0x64)

/* Deskew Registers (UltraScale+ specific for line rates > 1500 Mb/s) */
#define DPHY_DESKEW_CTRL_REG                    (DPHY_REG_BASE_OFFSET + 0x70)
#define DPHY_DESKEW_STATUS_REG                  (DPHY_REG_BASE_OFFSET + 0x74)

/* PLL and Clock Generation Registers */
#define DPHY_PLL_CTRL_REG                       (DPHY_REG_BASE_OFFSET + 0x80)
#define DPHY_PLL_STATUS_REG                     (DPHY_REG_BASE_OFFSET + 0x84)
#define DPHY_MMCM_CTRL_REG                      (DPHY_REG_BASE_OFFSET + 0x88)

/* C-PHY Specific Registers (Versal Premium Gen 2) */
#define CPHY_CTRL_REG                           (DPHY_REG_BASE_OFFSET + 0x90)
#define CPHY_STATUS_REG                         (DPHY_REG_BASE_OFFSET + 0x94)
#define CPHY_LANE_CONFIG_REG                    (DPHY_REG_BASE_OFFSET + 0x98)

/* D-PHY Control Register (0x00) */
#define DPHY_CTRL_ENABLE                        BIT(0)
#define DPHY_CTRL_RESET                         BIT(1)
#define DPHY_CTRL_SHUTDOWN                      BIT(2)
#define DPHY_CTRL_CLOCK_MODE_MASK               BIT(3)
#define DPHY_CTRL_CLOCK_MODE_CONTINUOUS         0
#define DPHY_CTRL_CLOCK_MODE_NON_CONTINUOUS     BIT(3)

/* D-PHY Status Register (0x04) */
#define DPHY_STATUS_INIT_DONE                   BIT(0)
#define DPHY_STATUS_READY                       BIT(1)
#define DPHY_STATUS_ESC_MODE                    BIT(2)
#define DPHY_STATUS_ULP_MODE                    BIT(3)
#define DPHY_STATUS_STOPSTATE_CLK               BIT(4)
#define DPHY_STATUS_STOPSTATE_DATA_MASK         GENMASK(8, 5)
#define DPHY_STATUS_PLL_LOCK                    BIT(9)

/* D-PHY Initialization Register (0x08) */
#define DPHY_INIT_START                         BIT(0)
#define DPHY_INIT_MASTER                        BIT(1)
#define DPHY_INIT_SLAVE                         BIT(2)

/* D-PHY Wakeup Register (0x0C) */
#define DPHY_WAKEUP_REQUEST                     BIT(0)
#define DPHY_WAKEUP_ACK                         BIT(1)

/* Lane Configuration Register (0x10) */
#define DPHY_LANE_CONFIG_NUM_LANES_MASK         GENMASK(1, 0)
#define DPHY_LANE_CONFIG_LANE_ENABLE_MASK       GENMASK(5, 2)
#define DPHY_LANE_CONFIG_SWAP_LANES             BIT(6)

/* Lane Status Register (0x14) */
#define DPHY_LANE_STATUS_CLK_ACTIVE             BIT(0)
#define DPHY_LANE_STATUS_DATA0_ACTIVE           BIT(1)
#define DPHY_LANE_STATUS_DATA1_ACTIVE           BIT(2)
#define DPHY_LANE_STATUS_DATA2_ACTIVE           BIT(3)
#define DPHY_LANE_STATUS_DATA3_ACTIVE           BIT(4)

/* HS Settle Register (0x20) */
#define DPHY_HS_SETTLE_VALUE_MASK               GENMASK(7, 0)
#define DPHY_HS_SETTLE_AUTO_ENABLE              BIT(8)

/* HS Timeout Register (0x24) */
#define DPHY_HS_TIMEOUT_VALUE_MASK              GENMASK(15, 0)

/* Escape Timeout Register (0x28) */
#define DPHY_ESC_TIMEOUT_VALUE_MASK             GENMASK(15, 0)

/* Clock Lane Register (0x30) */
#define DPHY_CLK_LANE_TERM_ENABLE               BIT(0)
#define DPHY_CLK_LANE_HS_RX_ENABLE              BIT(1)
#define DPHY_CLK_LANE_LP_RX_ENABLE              BIT(2)

/* Data Lane Registers (0x34-0x40) */
#define DPHY_DATA_LANE_TERM_ENABLE              BIT(0)
#define DPHY_DATA_LANE_HS_RX_ENABLE             BIT(1)
#define DPHY_DATA_LANE_LP_RX_ENABLE             BIT(2)
#define DPHY_DATA_LANE_FORCE_RX_MODE            BIT(3)

/* Interrupt Status Register (0x50) */
#define DPHY_INT_STATUS_INIT_DONE               BIT(0)
#define DPHY_INT_STATUS_ESC_CMD_ERROR           BIT(1)
#define DPHY_INT_STATUS_ESC_SYNC_ERROR          BIT(2)
#define DPHY_INT_STATUS_CONTROL_ERROR           BIT(3)
#define DPHY_INT_STATUS_LP_TIMEOUT              BIT(4)
#define DPHY_INT_STATUS_HS_TIMEOUT              BIT(5)
#define DPHY_INT_STATUS_ESC_TIMEOUT             BIT(6)
#define DPHY_INT_STATUS_FALSE_CONTROL           BIT(7)

/* Interrupt Enable Register (0x54) - Same bit positions as Status */
#define DPHY_INT_ENABLE_INIT_DONE               BIT(0)
#define DPHY_INT_ENABLE_ESC_CMD_ERROR           BIT(1)
#define DPHY_INT_ENABLE_ESC_SYNC_ERROR          BIT(2)
#define DPHY_INT_ENABLE_CONTROL_ERROR           BIT(3)
#define DPHY_INT_ENABLE_LP_TIMEOUT              BIT(4)
#define DPHY_INT_ENABLE_HS_TIMEOUT              BIT(5)
#define DPHY_INT_ENABLE_ESC_TIMEOUT             BIT(6)
#define DPHY_INT_ENABLE_FALSE_CONTROL           BIT(7)

/* Error Status Register (0x58) */
#define DPHY_ERROR_STATUS_SOT_ERROR             BIT(0)
#define DPHY_ERROR_STATUS_SOT_SYNC_ERROR        BIT(1)
#define DPHY_ERROR_STATUS_EOT_ERROR             BIT(2)
#define DPHY_ERROR_STATUS_ESC_MODE_ERROR        BIT(3)
#define DPHY_ERROR_STATUS_LP_TX_SYNC_ERROR      BIT(4)
#define DPHY_ERROR_STATUS_CONTROL_ERROR         BIT(5)

/* IDELAY Tap Register (0x60) - 7 series specific */
#define DPHY_IDELAY_TAP_VALUE_MASK              GENMASK(4, 0)
#define DPHY_IDELAY_TAP_LOAD                    BIT(5)
#define DPHY_IDELAY_TAP_READY                   BIT(6)

/* IDELAY Control Register (0x64) - 7 series specific */
#define DPHY_IDELAY_CTRL_ENABLE                 BIT(0)
#define DPHY_IDELAY_CTRL_MODE_MASK              GENMASK(2, 1)
#define DPHY_IDELAY_CTRL_MODE_FIXED             0
#define DPHY_IDELAY_CTRL_MODE_VARIABLE          BIT(1)
#define DPHY_IDELAY_CTRL_MODE_VAR_LOAD          BIT(2)

/* Deskew Control Register (0x70) - UltraScale+ specific */
#define DPHY_DESKEW_CTRL_ENABLE                 BIT(0)
#define DPHY_DESKEW_CTRL_PATTERN_LENGTH_MASK    GENMASK(7, 1)
#define DPHY_DESKEW_CTRL_AUTO_ALIGN             BIT(8)

/* Deskew Status Register (0x74) - UltraScale+ specific */
#define DPHY_DESKEW_STATUS_LOCKED               BIT(0)
#define DPHY_DESKEW_STATUS_PATTERN_DETECTED     BIT(1)
#define DPHY_DESKEW_STATUS_ALIGN_ERROR          BIT(2)

/* PLL Control Register (0x80) */
#define DPHY_PLL_CTRL_ENABLE                    BIT(0)
#define DPHY_PLL_CTRL_RESET                     BIT(1)
#define DPHY_PLL_CTRL_BYPASS                    BIT(2)

/* PLL Status Register (0x84) */
#define DPHY_PLL_STATUS_LOCKED                  BIT(0)
#define DPHY_PLL_STATUS_READY                   BIT(1)

/* MMCM Control Register (0x88) */
#define DPHY_MMCM_CTRL_ENABLE                   BIT(0)
#define DPHY_MMCM_CTRL_RESET                    BIT(1)
#define DPHY_MMCM_CTRL_EXTERNAL                 BIT(2)

/* C-PHY Control Register (0x90) - Versal Premium Gen 2 specific */
#define CPHY_CTRL_ENABLE                        BIT(0)
#define CPHY_CTRL_RESET                         BIT(1)
#define CPHY_CTRL_MODE_MASK                     GENMASK(3, 2)

/* C-PHY Status Register (0x94) - Versal Premium Gen 2 specific */
#define CPHY_STATUS_READY                       BIT(0)
#define CPHY_STATUS_WORD_LOCK                   BIT(1)
#define CPHY_STATUS_SYMBOL_LOCK                 BIT(2)

/* C-PHY Lane Configuration Register (0x98) - Versal Premium Gen 2 specific */
#define CPHY_LANE_CONFIG_NUM_LANES_MASK         GENMASK(1, 0)
#define CPHY_LANE_CONFIG_TRIO_ENABLE_MASK       GENMASK(4, 2)

/* Line Rate Calculation Helpers */
#define DPHY_HS_SETTLE_NS_TO_VALUE(ns, bit_period_ns)  ((ns) / (bit_period_ns))
#define DPHY_LINE_RATE_TO_BIT_PERIOD_NS(mbps)          (1000 / (mbps))

/* Utility macros for lane register access */
#define DPHY_DATA_LANE_REG(lane)               (DPHY_DATA_LANE0_REG + ((lane) * 4))

/* Helper functions for field extraction and setting */
static inline u32 dphy_get_num_lanes(u32 config)
{
    return (config & DPHY_LANE_CONFIG_NUM_LANES_MASK) + 1;
}

static inline u32 dphy_set_num_lanes(u32 config, u32 lanes)
{
    config &= ~DPHY_LANE_CONFIG_NUM_LANES_MASK;
    config |= (lanes - 1) & DPHY_LANE_CONFIG_NUM_LANES_MASK;
    return config;
}

static inline bool dphy_is_pll_locked(u32 status)
{
    return !!(status & DPHY_STATUS_PLL_LOCK);
}

static inline bool dphy_is_init_done(u32 status)
{
    return !!(status & DPHY_STATUS_INIT_DONE);
}

static inline bool dphy_is_stopstate_clk(u32 status)
{
    return !!(status & DPHY_STATUS_STOPSTATE_CLK);
}

static inline u32 dphy_get_stopstate_data(u32 status)
{
    return (status & DPHY_STATUS_STOPSTATE_DATA_MASK) >> 5;
}

static inline u32 dphy_calculate_hs_settle(u32 line_rate_mbps)
{
    /* HS_SETTLE = 135ns + 10*UI for typical case */
    u32 bit_period_ns = 1000 / line_rate_mbps;
    u32 settle_time_ns = 135 + (10 * bit_period_ns);
    return DPHY_HS_SETTLE_NS_TO_VALUE(settle_time_ns, bit_period_ns);
}

#endif /* AMD_CSI2_DPHY_CORE_H */
