// hw/misc/csi2_machine_integration.c - QEMU 머신에 CSI2-V4L2 디바이스 통합
#include "qemu/osdep.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qemu/error-report.h"
#include "include/hw/misc/csi2_v4l2_integration.h"

/* Machine-specific initialization for CSI2-V4L2 devices */
static void csi2_v4l2_machine_init(MachineState *machine)
{
    DeviceState *dev;
    Error *local_err = NULL;
    
    /* Create CSI2-V4L2 device */
    dev = qdev_new(TYPE_CSI2_V4L2_DEVICE);
    
    /* Configure device properties */
    qdev_prop_set_string(dev, "device-name", "QEMU MIPI Camera");
    qdev_prop_set_uint32(dev, "num-lanes", 4);
    qdev_prop_set_uint32(dev, "line-rate", 1440);
    qdev_prop_set_uint32(dev, "frame-width", 1920);
    qdev_prop_set_uint32(dev, "frame-height", 1080);
    qdev_prop_set_bit(dev, "auto-start", true);
    
    /* Realize device */
    if (!sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &local_err)) {
        error_report("Failed to create CSI2-V4L2 device: %s", 
                    error_get_pretty(local_err));
        error_free(local_err);
        return;
    }
    
    /* Map device to system memory */
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0xA0000000);
    
    printf("Machine: CSI2-V4L2 device mapped at 0xA0000000\n");
}

/* Register machine initialization hook */
static void csi2_v4l2_machine_register(void)
{
    /* This function is called but currently just registers the type */
    /* Actual machine initialization would be called from machine-specific code */
}

type_init(csi2_v4l2_machine_register)
