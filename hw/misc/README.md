# AMD MIPI CSI-2 RX Subsystem Register Definitions

This repository contains comprehensive register definitions for the AMD MIPI CSI-2 RX Subsystem, extracted from the AMD CSI-2 RX Subsystem Product Guide (PG232). These definitions are designed for use in QEMU device emulation and Linux kernel driver development.

## Overview

The AMD MIPI CSI-2 RX Subsystem implements a CSI-2 receive interface according to the MIPI CSI-2 standard v3.0 with underlying MIPI D-PHY standard v2.0/MIPI C-PHY Standard v2.0. The subsystem captures images from MIPI CSI-2 camera sensors and outputs AXI4-Stream video data ready for image processing.

## Project Structure

```
├── amd_csi2_rx_controller_core.h    # MIPI CSI-2 RX Controller register definitions
├── amd_csi2_dphy_core.h             # MIPI D-PHY/C-PHY core register definitions
└── README.md                        # This file
```

## Register Space Architecture

The AMD MIPI CSI-2 RX Subsystem uses a 8KB register space divided into two main sections:

- **CSI-2 RX Controller**: 4KB space starting at offset 0x0000
- **D-PHY/C-PHY Core**: 4KB space starting at offset 0x1000

## File Descriptions

### 1. amd_csi2_rx_controller_core.h

Contains register definitions for the MIPI CSI-2 RX Controller core, which handles:

#### Key Features:
- **Register Address Space**: 0x0000 - 0x0FFF (4KB)
- **Virtual Channel Support**: Up to 16 virtual channels (VC0-VC15) with VCX extension
- **Data Type Support**: RAW6-RAW20, RGB444-RGB888, YUV420/422 formats
- **Lane Configuration**: 1-4 D-PHY lanes or 1-3 C-PHY lanes support
- **Interrupt Management**: Comprehensive error detection and status reporting

#### Core Registers:
- **Core Configuration (0x00)**: Core enable, soft reset, full reset control
- **Protocol Configuration (0x04)**: Active lanes and maximum lanes configuration
- **Interrupt Status/Enable (0x24/0x28)**: Complete interrupt management
- **Dynamic VC Selection (0x2C)**: Runtime virtual channel filtering
- **Image Information (0x60-0xDC)**: Per-VC line count, byte count, and data type
- **Lane Information (0x40-0x4C)**: Individual lane status and error reporting

#### Supported Data Types:
```c
#define CSI2_DATA_TYPE_RAW8                     0x2A
#define CSI2_DATA_TYPE_RAW10                    0x2B
#define CSI2_DATA_TYPE_RAW12                    0x2C
#define CSI2_DATA_TYPE_RGB888                   0x24
#define CSI2_DATA_TYPE_YUV422_8BIT              0x1E
// ... and more
```

#### Interrupt Sources:
- Frame reception completion
- ECC 1-bit/2-bit errors
- CRC errors
- SoT/SoT sync errors
- Line buffer full conditions
- Frame synchronization errors per VC

### 2. amd_csi2_dphy_core.h

Contains register definitions for the MIPI D-PHY/C-PHY core, which handles:

#### Key Features:
- **Register Address Space**: 0x1000 - 0x1FFF (4KB)
- **Multi-Device Support**: 7 Series, UltraScale+, Versal adaptive SoC
- **Line Rate Support**: 80-4500 Mb/s depending on device family
- **Clock Modes**: Continuous and non-continuous clock operation
- **PHY Standards**: D-PHY v2.0 and C-PHY v2.0 support

#### Core Registers:
- **D-PHY Control/Status (0x00/0x04)**: Basic PHY control and operational status
- **Lane Configuration (0x10)**: Number of lanes, lane enable/disable, lane swapping
- **Timing Configuration (0x20-0x28)**: HS_SETTLE, HS timeout, escape timeout
- **Clock/Data Lane Control (0x30-0x40)**: Individual lane termination and enable
- **PLL/MMCM Control (0x80-0x88)**: Clock generation and management

#### Device-Specific Features:

**7 Series FPGAs:**
- IDELAY calibration support (Fixed/Auto modes)
- External IDELAY tap loading
- 300MHz clock support for IDELAYCTRL

**UltraScale+ Devices:**
- Deskew pattern detection for line rates > 1500 Mb/s
- Automatic lane alignment and center alignment
- Enhanced timing control

**Versal Premium Gen 2:**
- C-PHY trio support (1-3 lanes)
- 16-bit/32-bit PPI data width
- Advanced PHY control features

#### Timing Calculations:
```c
// HS_SETTLE calculation helper
static inline u32 dphy_calculate_hs_settle(u32 line_rate_mbps)
{
    /* HS_SETTLE = 135ns + 10*UI for typical case */
    u32 bit_period_ns = 1000 / line_rate_mbps;
    u32 settle_time_ns = 135 + (10 * bit_period_ns);
    return DPHY_HS_SETTLE_NS_TO_VALUE(settle_time_ns, bit_period_ns);
}
```

## Usage in QEMU Device Implementation

These register definitions are designed for use in QEMU's `csi2-pcie-sink` device implementation:

```c
#include "amd_csi2_rx_controller_core.h"
#include "amd_csi2_dphy_core.h"

// Example: Reading core status
uint32_t status = pci_default_read_config(dev, CSI2_CORE_STATUS_REG, 4);
uint32_t packet_count = csi2_get_packet_count(status);

// Example: Configuring D-PHY
uint32_t dphy_ctrl = DPHY_CTRL_ENABLE | DPHY_CTRL_CLOCK_MODE_CONTINUOUS;
pci_default_write_config(dev, DPHY_CONTROL_REG, dphy_ctrl, 4);
```

## Usage in Linux Kernel Driver

The register definitions can be directly used in V4L2-based kernel drivers:

```c
#include "amd_csi2_rx_controller_core.h"
#include "amd_csi2_dphy_core.h"

// Example: Interrupt handler
static irqreturn_t csi2_irq_handler(int irq, void *data)
{
    struct csi2_device *csi2 = data;
    u32 status = readl(csi2->base + CSI2_INT_STATUS_REG);
    
    if (status & CSI2_INT_STATUS_FRAME_RECEIVED) {
        // Handle frame completion
        v4l2_buffer_done(csi2->current_buffer, VB2_BUF_STATE_DONE);
    }
    
    if (status & CSI2_INT_STATUS_CRC_ERROR) {
        // Handle CRC error
        dev_warn(csi2->dev, "CRC error detected\n");
    }
    
    // Clear interrupts
    writel(status, csi2->base + CSI2_INT_STATUS_REG);
    return IRQ_HANDLED;
}
```

## QEMU Integration

The QEMU script provided supports the `csi2-pcie-sink` device:

```bash
-device csi2-pcie-sink,bus=swport3,id=csi0
```

This creates a PCIe-attached CSI-2 sink device that can be controlled through the register interface defined in these headers.

## Key Features Supported

### Video Format Support
- **RAW Formats**: 6-bit to 20-bit raw Bayer data
- **RGB Formats**: RGB444, RGB555, RGB565, RGB666, RGB888
- **YUV Formats**: YUV420 8-bit, YUV422 8-bit/10-bit
- **User Defined**: Custom data types (0x30-0x37)
- **Embedded Data**: Non-image data packets (0x12)

### Performance Characteristics
- **Line Rates**: 80 Mb/s to 4500 Mb/s (device dependent)
- **Lanes**: 1-4 D-PHY lanes, 1-3 C-PHY lanes
- **Pixel Throughput**: 1, 2, 4, or 8 pixels per clock
- **Virtual Channels**: Up to 16 with VCX extension
- **Latency**: Optimized for real-time video processing

### Error Detection and Recovery
- **PHY Level**: SoT errors, SoT sync errors, control errors
- **Protocol Level**: ECC 1-bit correction, ECC 2-bit detection, CRC validation
- **Frame Level**: Frame sync errors, frame data errors per VC
- **System Level**: Line buffer overflow, lane configuration errors

## Dependencies

- Linux kernel headers (for BIT(), GENMASK() macros)
- Standard integer types (u32, etc.)

## License

These register definitions are derived from AMD's public documentation and are intended for open-source development of QEMU devices and Linux kernel drivers.

## References

- AMD MIPI CSI-2 RX Subsystem Product Guide (PG232)
- MIPI Alliance Standard for Camera Serial Interface CSI-2 v3.0
- MIPI Alliance Physical Layer Specifications D-PHY v2.0/C-PHY v2.0

## Contributing

When contributing to this project:

1. Ensure register definitions match the official AMD documentation
2. Add comprehensive comments for complex bit field operations
3. Include helper functions for common register operations
4. Test with both QEMU device implementation and kernel driver usage

## Support

For questions regarding the AMD MIPI CSI-2 RX Subsystem implementation:
- Refer to the official AMD documentation (PG232)
- Check the MIPI Alliance specifications for protocol details
- Review AMD's reference designs and application examples
