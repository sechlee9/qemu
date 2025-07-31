/*
 * amd_csi2_rx_controller_core.h - Kernel version
 * AMD MIPI CSI-2 RX Controller Core Register Definitions
 * Based on AMD CSI-2 RX Subsystem Product Guide (PG232)
 * 
 * Register Space: 4KB (0x0000 - 0x0FFF)
 * Base Address Offset: 0x0000 from subsystem base
 */

#ifndef AMD_CSI2_RX_CONTROLLER_CORE_H
#define AMD_CSI2_RX_CONTROLLER_CORE_H

#include <linux/types.h>

/* QEMU compatibility definitions */
#ifndef BIT
#define BIT(nr)                     (1UL << (nr))
#endif

#ifndef GENMASK
#define GENMASK(h, l) \
    (((~0UL) << (l)) & (~0UL >> (sizeof(unsigned long) * 8 - 1 - (h))))
#endif

/* Register Address Offsets */
#define CSI2_CORE_CONFIG_REG                    0x00
#define CSI2_PROTOCOL_CONFIG_REG                0x04
#define CSI2_CORE_STATUS_REG                    0x10
#define CSI2_GLOBAL_INT_EN_REG                  0x20
#define CSI2_INT_STATUS_REG                     0x24
#define CSI2_INT_ENABLE_REG                     0x28
#define CSI2_DYNAMIC_VC_SEL_REG                 0x2C
#define CSI2_GENERIC_SHORT_PACKET_REG           0x30
#define CSI2_VCX_FRAME_ERROR_REG                0x34
#define CSI2_CLK_LANE_INFO_REG                  0x3C

/* Lane Information Registers */
#define CSI2_LANE0_INFO_REG                     0x40
#define CSI2_LANE1_INFO_REG                     0x44
#define CSI2_LANE2_INFO_REG                     0x48
#define CSI2_LANE3_INFO_REG                     0x4C

/* Image Information 1 Registers (VC0-VC15) */
#define CSI2_IMG_INFO1_VC0_REG                  0x60
#define CSI2_IMG_INFO2_VC0_REG                  0x64
#define CSI2_IMG_INFO1_VC1_REG                  0x68
#define CSI2_IMG_INFO2_VC1_REG                  0x6C
#define CSI2_IMG_INFO1_VC2_REG                  0x70
#define CSI2_IMG_INFO2_VC2_REG                  0x74
#define CSI2_IMG_INFO1_VC3_REG                  0x78
#define CSI2_IMG_INFO2_VC3_REG                  0x7C
#define CSI2_IMG_INFO1_VC4_REG                  0x80
#define CSI2_IMG_INFO2_VC4_REG                  0x84
#define CSI2_IMG_INFO1_VC5_REG                  0x88
#define CSI2_IMG_INFO2_VC5_REG                  0x8C
#define CSI2_IMG_INFO1_VC6_REG                  0x90
#define CSI2_IMG_INFO2_VC6_REG                  0x94
#define CSI2_IMG_INFO1_VC7_REG                  0x98
#define CSI2_IMG_INFO2_VC7_REG                  0x9C
#define CSI2_IMG_INFO1_VC8_REG                  0xA0
#define CSI2_IMG_INFO2_VC8_REG                  0xA4
#define CSI2_IMG_INFO1_VC9_REG                  0xA8
#define CSI2_IMG_INFO2_VC9_REG                  0xAC
#define CSI2_IMG_INFO1_VC10_REG                 0xB0
#define CSI2_IMG_INFO2_VC10_REG                 0xB4
#define CSI2_IMG_INFO1_VC11_REG                 0xB8
#define CSI2_IMG_INFO2_VC11_REG                 0xBC
#define CSI2_IMG_INFO1_VC12_REG                 0xC0
#define CSI2_IMG_INFO2_VC12_REG                 0xC4
#define CSI2_IMG_INFO1_VC13_REG                 0xC8
#define CSI2_IMG_INFO2_VC13_REG                 0xCC
#define CSI2_IMG_INFO1_VC14_REG                 0xD0
#define CSI2_IMG_INFO2_VC14_REG                 0xD4
#define CSI2_IMG_INFO1_VC15_REG                 0xD8
#define CSI2_IMG_INFO2_VC15_REG                 0xDC

/* Core Configuration Register (0x00) */
#define CSI2_CORE_CONFIG_CORE_EN_MASK           BIT(0)
#define CSI2_CORE_CONFIG_SOFT_RESET_MASK        BIT(1)
#define CSI2_CORE_CONFIG_FULL_RESET_MASK        BIT(2)

/* Protocol Configuration Register (0x04) */
#define CSI2_PROTOCOL_CONFIG_ACTIVE_LANES_MASK  GENMASK(1, 0)
#define CSI2_PROTOCOL_CONFIG_MAX_LANES_MASK     GENMASK(4, 3)

#define CSI2_LANES_1                            0x0
#define CSI2_LANES_2                            0x1
#define CSI2_LANES_3                            0x2
#define CSI2_LANES_4                            0x3

/* Core Status Register (0x10) */
#define CSI2_CORE_STATUS_PACKET_COUNT_MASK      GENMASK(31, 16)
#define CSI2_CORE_STATUS_SHORT_PACKET_FIFO_FULL BIT(3)
#define CSI2_CORE_STATUS_SHORT_PACKET_FIFO_NE   BIT(2)
#define CSI2_CORE_STATUS_STREAM_LINE_BUF_FULL   BIT(1)
#define CSI2_CORE_STATUS_SOFT_RESET_IN_PROG     BIT(0)

/* Global Interrupt Enable Register (0x20) */
#define CSI2_GLOBAL_INT_EN_MASK                 BIT(0)

/* Interrupt Status Register (0x24) */
#define CSI2_INT_STATUS_FRAME_RECEIVED          BIT(31)
#define CSI2_INT_STATUS_VCX_FRAME_ERROR         BIT(30)
#define CSI2_INT_STATUS_RX_SKEWCALHS            BIT(29)
#define CSI2_INT_STATUS_YUV420_WC_ERROR         BIT(28)
#define CSI2_INT_STATUS_PENDING_WRITE_FIFO      BIT(27)
#define CSI2_INT_STATUS_WC_CORRUPTION           BIT(22)
#define CSI2_INT_STATUS_INCORRECT_LANE_CONFIG   BIT(21)
#define CSI2_INT_STATUS_SHORT_PACKET_FIFO_FULL  BIT(20)
#define CSI2_INT_STATUS_SHORT_PACKET_FIFO_NE    BIT(19)
#define CSI2_INT_STATUS_STREAM_LINE_BUF_FULL    BIT(18)
#define CSI2_INT_STATUS_STOP_STATE              BIT(17)
#define CSI2_INT_STATUS_SOT_ERROR               BIT(13)
#define CSI2_INT_STATUS_SOT_SYNC_ERROR          BIT(12)
#define CSI2_INT_STATUS_ECC_2BIT_ERROR          BIT(11)
#define CSI2_INT_STATUS_ECC_1BIT_ERROR          BIT(10)
#define CSI2_INT_STATUS_CRC_ERROR               BIT(9)
#define CSI2_INT_STATUS_UNSUPPORTED_DATA_TYPE   BIT(8)
#define CSI2_INT_STATUS_FRAME_SYNC_ERROR_VC3    BIT(7)
#define CSI2_INT_STATUS_FRAME_LEVEL_ERROR_VC3   BIT(6)
#define CSI2_INT_STATUS_FRAME_SYNC_ERROR_VC2    BIT(5)
#define CSI2_INT_STATUS_FRAME_LEVEL_ERROR_VC2   BIT(4)
#define CSI2_INT_STATUS_FRAME_SYNC_ERROR_VC1    BIT(3)
#define CSI2_INT_STATUS_FRAME_LEVEL_ERROR_VC1   BIT(2)
#define CSI2_INT_STATUS_FRAME_SYNC_ERROR_VC0    BIT(1)
#define CSI2_INT_STATUS_FRAME_LEVEL_ERROR_VC0   BIT(0)

/* Interrupt Enable Register (0x28) - Same bit positions as Status Register */
#define CSI2_INT_ENABLE_FRAME_RECEIVED          BIT(31)
#define CSI2_INT_ENABLE_VCX_FRAME_ERROR         BIT(30)
#define CSI2_INT_ENABLE_RX_SKEWCALHS            BIT(29)
#define CSI2_INT_ENABLE_YUV420_WC_ERROR         BIT(28)
#define CSI2_INT_ENABLE_PENDING_WRITE_FIFO      BIT(27)
#define CSI2_INT_ENABLE_WC_CORRUPTION           BIT(22)
#define CSI2_INT_ENABLE_INCORRECT_LANE_CONFIG   BIT(21)
#define CSI2_INT_ENABLE_SHORT_PACKET_FIFO_FULL  BIT(20)
#define CSI2_INT_ENABLE_SHORT_PACKET_FIFO_NE    BIT(19)
#define CSI2_INT_ENABLE_STREAM_LINE_BUF_FULL    BIT(18)
#define CSI2_INT_ENABLE_STOP_STATE              BIT(17)
#define CSI2_INT_ENABLE_SOT_ERROR               BIT(13)
#define CSI2_INT_ENABLE_SOT_SYNC_ERROR          BIT(12)
#define CSI2_INT_ENABLE_ECC_2BIT_ERROR          BIT(11)
#define CSI2_INT_ENABLE_ECC_1BIT_ERROR          BIT(10)
#define CSI2_INT_ENABLE_CRC_ERROR               BIT(9)
#define CSI2_INT_ENABLE_UNSUPPORTED_DATA_TYPE   BIT(8)
#define CSI2_INT_ENABLE_FRAME_SYNC_ERROR_VC3    BIT(7)
#define CSI2_INT_ENABLE_FRAME_LEVEL_ERROR_VC3   BIT(6)
#define CSI2_INT_ENABLE_FRAME_SYNC_ERROR_VC2    BIT(5)
#define CSI2_INT_ENABLE_FRAME_LEVEL_ERROR_VC2   BIT(4)
#define CSI2_INT_ENABLE_FRAME_SYNC_ERROR_VC1    BIT(3)
#define CSI2_INT_ENABLE_FRAME_LEVEL_ERROR_VC1   BIT(2)
#define CSI2_INT_ENABLE_FRAME_SYNC_ERROR_VC0    BIT(1)
#define CSI2_INT_ENABLE_FRAME_LEVEL_ERROR_VC0   BIT(0)

/* Dynamic VC Selection Register (0x2C) */
#define CSI2_DYNAMIC_VC_SEL_MASK                GENMASK(15, 0)

/* Generic Short Packet Register (0x30) */
#define CSI2_GENERIC_SHORT_PACKET_DATA_MASK     GENMASK(23, 8)
#define CSI2_GENERIC_SHORT_PACKET_VC_MASK       GENMASK(7, 6)
#define CSI2_GENERIC_SHORT_PACKET_DT_MASK       GENMASK(5, 0)

/* VCX Frame Error Register (0x34) */
#define CSI2_VCX_FRAME_ERROR_SYNC_VC15          BIT(23)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC15         BIT(22)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC14          BIT(21)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC14         BIT(20)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC13          BIT(19)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC13         BIT(18)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC12          BIT(17)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC12         BIT(16)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC11          BIT(15)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC11         BIT(14)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC10          BIT(13)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC10         BIT(12)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC9           BIT(11)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC9          BIT(10)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC8           BIT(9)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC8          BIT(8)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC7           BIT(7)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC7          BIT(6)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC6           BIT(5)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC6          BIT(4)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC5           BIT(3)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC5          BIT(2)
#define CSI2_VCX_FRAME_ERROR_SYNC_VC4           BIT(1)
#define CSI2_VCX_FRAME_ERROR_LEVEL_VC4          BIT(0)

/* Clock Lane Information Register (0x3C) */
#define CSI2_CLK_LANE_INFO_STOP_STATE           BIT(1)

/* Lane Information Register (0x40-0x4C) */
#define CSI2_LANE_INFO_STOP_STATE               BIT(5)
#define CSI2_LANE_INFO_SKEWCALHS                BIT(2)
#define CSI2_LANE_INFO_SOT_ERROR                BIT(1)
#define CSI2_LANE_INFO_SOT_SYNC_ERROR           BIT(0)

/* Image Information 1 Register */
#define CSI2_IMG_INFO1_LINE_COUNT_MASK          GENMASK(31, 16)
#define CSI2_IMG_INFO1_BYTE_COUNT_MASK          GENMASK(15, 0)

/* Image Information 2 Register */
#define CSI2_IMG_INFO2_DATA_TYPE_MASK           GENMASK(5, 0)

/* Data Type Definitions */
#define CSI2_DATA_TYPE_YUV420_8BIT              0x18
#define CSI2_DATA_TYPE_YUV420_10BIT             0x19
#define CSI2_DATA_TYPE_YUV422_8BIT              0x1E
#define CSI2_DATA_TYPE_YUV422_10BIT             0x1F
#define CSI2_DATA_TYPE_RGB444                   0x20
#define CSI2_DATA_TYPE_RGB555                   0x21
#define CSI2_DATA_TYPE_RGB565                   0x22
#define CSI2_DATA_TYPE_RGB666                   0x23
#define CSI2_DATA_TYPE_RGB888                   0x24
#define CSI2_DATA_TYPE_RAW6                     0x28
#define CSI2_DATA_TYPE_RAW7                     0x29
#define CSI2_DATA_TYPE_RAW8                     0x2A
#define CSI2_DATA_TYPE_RAW10                    0x2B
#define CSI2_DATA_TYPE_RAW12                    0x2C
#define CSI2_DATA_TYPE_RAW14                    0x2D
#define CSI2_DATA_TYPE_RAW16                    0x2E
#define CSI2_DATA_TYPE_RAW20                    0x2F
#define CSI2_DATA_TYPE_USER_DEFINED_BASE        0x30
#define CSI2_DATA_TYPE_EMBEDDED_8BIT            0x12

/* Utility macros for register access */
#define CSI2_IMG_INFO1_VC_REG(vc)  (CSI2_IMG_INFO1_VC0_REG + ((vc) * 8))
#define CSI2_IMG_INFO2_VC_REG(vc)  (CSI2_IMG_INFO2_VC0_REG + ((vc) * 8))
#define CSI2_LANE_INFO_REG(lane)   (CSI2_LANE0_INFO_REG + ((lane) * 4))

/* Helper functions for field extraction */
static inline u32 csi2_get_packet_count(u32 status)
{
    return (status & CSI2_CORE_STATUS_PACKET_COUNT_MASK) >> 16;
}

static inline u32 csi2_get_line_count(u32 info1)
{
    return (info1 & CSI2_IMG_INFO1_LINE_COUNT_MASK) >> 16;
}

static inline u32 csi2_get_byte_count(u32 info1)
{
    return info1 & CSI2_IMG_INFO1_BYTE_COUNT_MASK;
}

static inline u32 csi2_get_data_type(u32 info2)
{
    return info2 & CSI2_IMG_INFO2_DATA_TYPE_MASK;
}

static inline u32 csi2_get_short_packet_data(u32 packet)
{
    return (packet & CSI2_GENERIC_SHORT_PACKET_DATA_MASK) >> 8;
}

static inline u32 csi2_get_short_packet_vc(u32 packet)
{
    return (packet & CSI2_GENERIC_SHORT_PACKET_VC_MASK) >> 6;
}

static inline u32 csi2_get_short_packet_dt(u32 packet)
{
    return packet & CSI2_GENERIC_SHORT_PACKET_DT_MASK;
}

#endif /* AMD_CSI2_RX_CONTROLLER_CORE_H */
