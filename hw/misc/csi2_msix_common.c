// hw/misc/csi2_msix_common.c - CSI2 MSI-X 공통 구현 (빌드 오류 수정)
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "include/hw/misc/csi2_msix_common.h"

/* MSI-X 벡터 정의 */
#define CSI2_MSIX_FRAME_VECTOR      0
#define CSI2_MSIX_ERROR_VECTOR      1
#define CSI2_MSIX_STATUS_VECTOR     2
#define CSI2_MSIX_V4L2_VECTOR       3

/* MSI-X 초기화 */
int csi2_msix_init(PCIDevice *pci_dev, unsigned int nvectors)
{
    int ret;
    
    if (nvectors == 0 || nvectors > CSI2_MSIX_MAX_VECTORS) {
        qemu_log_mask(LOG_GUEST_ERROR, "CSI2: Invalid MSI-X vector count: %u\n", nvectors);
        return -EINVAL;
    }
    
    /* MSI-X 테이블과 PBA를 위한 메모리 영역 할당 */
    ret = msix_init(pci_dev, nvectors,
                    &pci_dev->msix_table_mmio, 0, 0,    /* table */
                    &pci_dev->msix_pba_mmio, 0, 0x1000, /* pba */
                    0, NULL);
    
    if (ret < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "CSI2: MSI-X initialization failed: %d\n", ret);
        return ret;
    }
    
    qemu_log_mask(LOG_TRACE, "CSI2: MSI-X initialized with %u vectors\n", nvectors);
    return 0;
}

/* MSI-X 정리 */
void csi2_msix_cleanup(PCIDevice *pci_dev)
{
    if (msix_present(pci_dev)) {
        msix_uninit(pci_dev, &pci_dev->msix_table_mmio, &pci_dev->msix_pba_mmio);
        qemu_log_mask(LOG_TRACE, "CSI2: MSI-X cleaned up\n");
    }
}

/* MSI-X 상태 초기화 */
void csi2_msix_state_init(CSI2MSIXState *state)
{
    if (!state) {
        return;
    }
    
    memset(state, 0, sizeof(CSI2MSIXState));
    state->num_vectors = CSI2_MSIX_MAX_VECTORS;
    qemu_log_mask(LOG_TRACE, "CSI2: MSI-X state initialized\n");
}

/* MSI-X 상태 리셋 */
void csi2_msix_state_reset(CSI2MSIXState *state)
{
    if (!state) {
        return;
    }
    
    state->enabled = false;
    state->pending_vectors = 0;
    state->masked_vectors = 0xFFFFFFFF; /* 모든 벡터 마스크 */
    qemu_log_mask(LOG_TRACE, "CSI2: MSI-X state reset\n");
}

/* MSI-X 상태 업데이트 */
void csi2_msix_state_update(CSI2MSIXState *state, PCIDevice *pci_dev)
{
    if (!state || !pci_dev) {
        return;
    }
    
    state->enabled = msix_enabled(pci_dev);
    state->initialized = msix_present(pci_dev);
    
    /* 마스크 상태 업데이트 */
    state->masked_vectors = 0;
    for (unsigned int i = 0; i < state->num_vectors; i++) {
        if (state->enabled && msix_is_masked(pci_dev, i)) {
            state->masked_vectors |= (1U << i);
        }
    }
}

/* 프레임 수신 인터럽트 전송 */
void csi2_msix_notify_frame_received(PCIDevice *pci_dev)
{
    if (msix_enabled(pci_dev)) {
        msix_notify(pci_dev, CSI2_MSIX_FRAME_VECTOR);
        qemu_log_mask(LOG_TRACE, "CSI2: Frame received MSI-X sent (vector %d)\n", 
                     CSI2_MSIX_FRAME_VECTOR);
    }
}

/* 오류 인터럽트 전송 */
void csi2_msix_notify_error(PCIDevice *pci_dev, uint32_t error_type)
{
    if (msix_enabled(pci_dev)) {
        msix_notify(pci_dev, CSI2_MSIX_ERROR_VECTOR);
        qemu_log_mask(LOG_TRACE, "CSI2: Error MSI-X sent (vector %d, type 0x%x)\n", 
                     CSI2_MSIX_ERROR_VECTOR, error_type);
    }
}

/* 상태 변경 인터럽트 전송 */
void csi2_msix_notify_status_change(PCIDevice *pci_dev, uint32_t status)
{
    if (msix_enabled(pci_dev)) {
        msix_notify(pci_dev, CSI2_MSIX_STATUS_VECTOR);
        qemu_log_mask(LOG_TRACE, "CSI2: Status change MSI-X sent (vector %d, status 0x%x)\n", 
                     CSI2_MSIX_STATUS_VECTOR, status);
    }
}

/* V4L2 관련 인터럽트 전송 */
void csi2_msix_notify_v4l2_event(PCIDevice *pci_dev, uint32_t event_type)
{
    if (msix_enabled(pci_dev)) {
        msix_notify(pci_dev, CSI2_MSIX_V4L2_VECTOR);
        qemu_log_mask(LOG_TRACE, "CSI2: V4L2 event MSI-X sent (vector %d, event 0x%x)\n", 
                     CSI2_MSIX_V4L2_VECTOR, event_type);
    }
}

/* MSI-X 벡터 활성화 상태 확인 */
bool csi2_msix_vector_enabled(PCIDevice *pci_dev, unsigned int vector)
{
    if (!msix_enabled(pci_dev) || vector >= CSI2_MSIX_MAX_VECTORS) {
        return false;
    }
    
    return !msix_is_masked(pci_dev, vector);
}

/* 모든 MSI-X 벡터 마스크 설정/해제 */
void csi2_msix_set_mask_all(PCIDevice *pci_dev, bool mask)
{
    if (!msix_enabled(pci_dev)) {
        return;
    }
    
    for (unsigned int i = 0; i < CSI2_MSIX_MAX_VECTORS; i++) {
        if (mask) {
            msix_set_mask(pci_dev, i, true);
        } else {
            msix_set_mask(pci_dev, i, false);
        }
    }
    
    qemu_log_mask(LOG_TRACE, "CSI2: All MSI-X vectors %s\n", 
                 mask ? "masked" : "unmasked");
}

/* MSI-X 상태 정보 얻기 */
CSI2MSIXInfo csi2_msix_get_info(PCIDevice *pci_dev)
{
    CSI2MSIXInfo info = {0};
    
    info.enabled = msix_enabled(pci_dev);
    info.present = msix_present(pci_dev);
    info.num_vectors = CSI2_MSIX_MAX_VECTORS;
    
    if (info.enabled) {
        for (unsigned int i = 0; i < CSI2_MSIX_MAX_VECTORS; i++) {
            info.vector_masked[i] = msix_is_masked(pci_dev, i);
        }
    }
    
    return info;
}

/* MSI-X 디버그 정보 출력 */
void csi2_msix_debug_print(PCIDevice *pci_dev)
{
    CSI2MSIXInfo info = csi2_msix_get_info(pci_dev);
    
    printf("CSI2 MSI-X Debug Information:\n");
    printf("  Present: %s\n", info.present ? "Yes" : "No");
    printf("  Enabled: %s\n", info.enabled ? "Yes" : "No");
    printf("  Vectors: %u\n", info.num_vectors);
    
    if (info.enabled) {
        printf("  Vector Status:\n");
        printf("    Frame (0):  %s\n", info.vector_masked[0] ? "Masked" : "Enabled");
        printf("    Error (1):  %s\n", info.vector_masked[1] ? "Masked" : "Enabled");
        printf("    Status (2): %s\n", info.vector_masked[2] ? "Masked" : "Enabled");
        printf("    V4L2 (3):   %s\n", info.vector_masked[3] ? "Masked" : "Enabled");
    }
}
