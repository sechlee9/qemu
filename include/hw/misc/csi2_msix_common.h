// include/hw/misc/csi2_msix_common.h - CSI2 MSI-X 공통 헤더 (빌드 오류 수정)
#ifndef HW_MISC_CSI2_MSIX_COMMON_H
#define HW_MISC_CSI2_MSIX_COMMON_H

#include "qemu/osdep.h"

/* Forward declarations */
typedef struct PCIDevice PCIDevice;

/* MSI-X 벡터 수 */
#define CSI2_MSIX_MAX_VECTORS    4

/* MSI-X 상태 구조체 */
typedef struct CSI2MSIXState {
    bool initialized;
    bool enabled;
    unsigned int num_vectors;
    uint32_t pending_vectors;
    uint32_t masked_vectors;
} CSI2MSIXState;

/* MSI-X 정보 구조체 */
typedef struct CSI2MSIXInfo {
    bool enabled;
    bool present;
    unsigned int num_vectors;
    bool vector_masked[CSI2_MSIX_MAX_VECTORS];
} CSI2MSIXInfo;

/* 함수 선언 */
int csi2_msix_init(PCIDevice *pci_dev, unsigned int nvectors);
void csi2_msix_cleanup(PCIDevice *pci_dev);

/* MSI-X 상태 관리 */
void csi2_msix_state_init(CSI2MSIXState *state);
void csi2_msix_state_reset(CSI2MSIXState *state);
void csi2_msix_state_update(CSI2MSIXState *state, PCIDevice *pci_dev);

void csi2_msix_notify_frame_received(PCIDevice *pci_dev);
void csi2_msix_notify_error(PCIDevice *pci_dev, uint32_t error_type);
void csi2_msix_notify_status_change(PCIDevice *pci_dev, uint32_t status);
void csi2_msix_notify_v4l2_event(PCIDevice *pci_dev, uint32_t event_type);

bool csi2_msix_vector_enabled(PCIDevice *pci_dev, unsigned int vector);
void csi2_msix_set_mask_all(PCIDevice *pci_dev, bool mask);

CSI2MSIXInfo csi2_msix_get_info(PCIDevice *pci_dev);
void csi2_msix_debug_print(PCIDevice *pci_dev);

#endif /* HW_MISC_CSI2_MSIX_COMMON_H */
