// csi2_to_v4l2_bridge_debug_fixed.c - 자동 V4L2 장치 탐지 버전
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>
#include <dirent.h>
#include <linux/videodev2.h>

/* CSI-2 Register Definitions */
#define CSI2_CORE_CONFIG        0x00
#define CSI2_PROTOCOL_CONFIG    0x04
#define CSI2_CORE_STATUS        0x10
#define CSI2_GLOBAL_INT_ENABLE  0x20
#define CSI2_INT_STATUS         0x24
#define CSI2_INT_ENABLE         0x28
#define V4L2_STREAMING          0x2000
#define V4L2_FRAMES_CAPTURED    0x2004
#define V4L2_SEQUENCE_NUMBER    0x2008

/* PCIe device info */
#define CSI2_VENDOR_ID          0x1234
#define CSI2_DEVICE_ID          0x5678

typedef struct {
    int pci_fd;
    void *mmio_base;
    size_t mmio_size;
    volatile int running;
    char pci_path[256];
    
    // V4L2 관련
    int v4l2_fd;
    char v4l2_device[32];
    bool v4l2_found;
    
    // 프레임 버퍼
    uint8_t *frame_buffer;
    size_t frame_size;
} csi2_device_t;

static csi2_device_t g_csi2_dev = {0};

void signal_handler(int sig) {
    printf("\nReceived signal %d, stopping...\n", sig);
    g_csi2_dev.running = 0;
}

// V4L2 loopback 장치 자동 탐지
static int find_v4l2_loopback_device(char *device_path, size_t path_size) {
    DIR *video_dir;
    struct dirent *entry;
    char sys_path[256];
    char driver_link[256];
    char driver_name[64];
    ssize_t link_len;
    
    printf("Searching for V4L2 loopback devices...\n");
    
    // /sys/class/video4linux 디렉토리 탐색
    video_dir = opendir("/sys/class/video4linux");
    if (!video_dir) {
        printf("Failed to open /sys/class/video4linux\n");
        return -1;
    }
    
    while ((entry = readdir(video_dir)) != NULL) {
        if (strncmp(entry->d_name, "video", 5) != 0) {
            continue;
        }
        
        // 드라이버 링크 경로 구성
        snprintf(sys_path, sizeof(sys_path), 
                "/sys/class/video4linux/%s/device/driver", entry->d_name);
        
        // 드라이버 링크 읽기
        link_len = readlink(sys_path, driver_link, sizeof(driver_link) - 1);
        if (link_len == -1) {
            continue;
        }
        
        driver_link[link_len] = '\0';
        
        // 드라이버 이름 추출 (경로에서 마지막 부분)
        char *driver_basename = strrchr(driver_link, '/');
        if (driver_basename) {
            driver_basename++; // '/' 다음 문자부터
        } else {
            driver_basename = driver_link;
        }
        
        printf("  %s: driver = %s\n", entry->d_name, driver_basename);
        
        // v4l2loopback 드라이버 확인
        if (strstr(driver_basename, "v4l2loopback") != NULL) {
            snprintf(device_path, path_size, "/dev/%s", entry->d_name);
            closedir(video_dir);
            
            // 장치 파일 존재 확인
            if (access(device_path, F_OK) == 0) {
                printf("Found V4L2 loopback device: %s\n", device_path);
                return 0;
            }
        }
    }
    
    closedir(video_dir);
    printf("No V4L2 loopback device found\n");
    return -1;
}

// V4L2 장치 초기화
static int init_v4l2_device(csi2_device_t *dev) {
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    
    if (find_v4l2_loopback_device(dev->v4l2_device, sizeof(dev->v4l2_device)) < 0) {
        return -1;
    }
    
    // V4L2 장치 열기
    dev->v4l2_fd = open(dev->v4l2_device, O_RDWR);
    if (dev->v4l2_fd < 0) {
        printf("Failed to open %s: %s\n", dev->v4l2_device, strerror(errno));
        return -1;
    }
    
    // 장치 capabilities 확인
    if (ioctl(dev->v4l2_fd, VIDIOC_QUERYCAP, &cap) < 0) {
        printf("Failed to query capabilities: %s\n", strerror(errno));
        close(dev->v4l2_fd);
        return -1;
    }
    
    printf("V4L2 Device Info:\n");
    printf("  Driver: %s\n", cap.driver);
    printf("  Card: %s\n", cap.card);
    printf("  Capabilities: 0x%08x\n", cap.capabilities);
    
    // 출력 장치인지 확인
    if (!(cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)) {
        printf("Device does not support video output\n");
        close(dev->v4l2_fd);
        return -1;
    }
    
    // 포맷 설정
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    
    // 현재 포맷 확인
    if (ioctl(dev->v4l2_fd, VIDIOC_G_FMT, &fmt) < 0) {
        printf("Failed to get format: %s\n", strerror(errno));
        close(dev->v4l2_fd);
        return -1;
    }
    
    printf("Current V4L2 format: %dx%d, fourcc: %c%c%c%c\n",
           fmt.fmt.pix.width, fmt.fmt.pix.height,
           fmt.fmt.pix.pixelformat & 0xff,
           (fmt.fmt.pix.pixelformat >> 8) & 0xff,
           (fmt.fmt.pix.pixelformat >> 16) & 0xff,
           (fmt.fmt.pix.pixelformat >> 24) & 0xff);
    
    // 프레임 버퍼 크기 계산
    dev->frame_size = fmt.fmt.pix.sizeimage;
    dev->frame_buffer = malloc(dev->frame_size);
    if (!dev->frame_buffer) {
        printf("Failed to allocate frame buffer\n");
        close(dev->v4l2_fd);
        return -1;
    }
    
    printf("V4L2 device initialized: %s (frame size: %zu bytes)\n", 
           dev->v4l2_device, dev->frame_size);
    
    dev->v4l2_found = true;
    return 0;
}

// CSI2 장치 찾기 및 초기화
static int find_csi2_device(csi2_device_t *dev) {
    DIR *pci_dir;
    struct dirent *entry;
    char vendor_path[512];
    char device_path[512];
    char resource_path[512];
    FILE *fp;
    uint16_t vendor, device;
    
    printf("Searching for CSI2 device (%04x:%04x)...\n", CSI2_VENDOR_ID, CSI2_DEVICE_ID);
    
    pci_dir = opendir("/sys/bus/pci/devices");
    if (!pci_dir) {
        perror("Failed to open /sys/bus/pci/devices");
        return -1;
    }
    
    while ((entry = readdir(pci_dir)) != NULL) {
        if (entry->d_name[0] == '.') continue;
        
        // vendor ID 읽기
        snprintf(vendor_path, sizeof(vendor_path), 
                "/sys/bus/pci/devices/%s/vendor", entry->d_name);
        fp = fopen(vendor_path, "r");
        if (!fp) continue;
        if (fscanf(fp, "0x%hx", &vendor) != 1) {
            fclose(fp);
            continue;
        }
        fclose(fp);
        
        // device ID 읽기
        snprintf(device_path, sizeof(device_path), 
                "/sys/bus/pci/devices/%s/device", entry->d_name);
        fp = fopen(device_path, "r");
        if (!fp) continue;
        if (fscanf(fp, "0x%hx", &device) != 1) {
            fclose(fp);
            continue;
        }
        fclose(fp);
        
        if (vendor == CSI2_VENDOR_ID && device == CSI2_DEVICE_ID) {
            printf("Found CSI2 device: %s (%04x:%04x)\n", entry->d_name, vendor, device);
            
            // resource0 파일 경로
            snprintf(resource_path, sizeof(resource_path), 
                    "/sys/bus/pci/devices/%s/resource0", entry->d_name);
            
            dev->pci_fd = open(resource_path, O_RDWR | O_SYNC);
            if (dev->pci_fd >= 0) {
                strncpy(dev->pci_path, entry->d_name, sizeof(dev->pci_path) - 1);
                closedir(pci_dir);
                return 0;
            }
            
            printf("Failed to open %s: %s\n", resource_path, strerror(errno));
        }
    }
    
    closedir(pci_dir);
    return -1;
}

/* Helper functions */
static uint32_t csi2_read32(csi2_device_t *dev, uint32_t offset) {
    if (!dev->mmio_base || offset >= dev->mmio_size) {
        return 0;
    }
    return *(volatile uint32_t*)((char*)dev->mmio_base + offset);
}

static void csi2_write32(csi2_device_t *dev, uint32_t offset, uint32_t value) {
    if (!dev->mmio_base || offset >= dev->mmio_size) {
        return;
    }
    *(volatile uint32_t*)((char*)dev->mmio_base + offset) = value;
}

/* Initialize CSI2 device */
static int csi2_init(csi2_device_t *dev) {
    struct stat st;
    
    if (find_csi2_device(dev) < 0) {
        return -1;
    }
    
    if (fstat(dev->pci_fd, &st) < 0) {
        perror("fstat");
        return -1;
    }
    
    dev->mmio_size = st.st_size;
    if (dev->mmio_size == 0) {
        dev->mmio_size = 0x10000; /* Default 64KB */
    }
    
    dev->mmio_base = mmap(NULL, dev->mmio_size, PROT_READ | PROT_WRITE, 
                          MAP_SHARED, dev->pci_fd, 0);
    if (dev->mmio_base == MAP_FAILED) {
        perror("mmap");
        return -1;
    }
    
    printf("CSI2 device mapped at %p, size: 0x%zx\n", dev->mmio_base, dev->mmio_size);
    return 0;
}

/* Write frame to V4L2 device */
static int write_frame_to_v4l2(csi2_device_t *dev) {
    if (!dev->v4l2_found || dev->v4l2_fd < 0 || !dev->frame_buffer) {
        return -1;
    }
    
    // 간단한 테스트 패턴 생성 (그라디언트)
    uint8_t *buf = dev->frame_buffer;
    static uint32_t frame_counter = 0;
    frame_counter++;
    
    // RGB24 형식으로 그라디언트 패턴 생성
    for (size_t i = 0; i < dev->frame_size; i += 3) {
        buf[i]   = (i + frame_counter) % 256;      // R
        buf[i+1] = (i/2 + frame_counter) % 256;    // G
        buf[i+2] = (i/3 + frame_counter) % 256;    // B
    }
    
    ssize_t written = write(dev->v4l2_fd, dev->frame_buffer, dev->frame_size);
    if (written != (ssize_t)dev->frame_size) {
        return -1;
    }
    
    return 0;
}

/* Monitor CSI2 status and bridge to V4L2 */
static void csi2_monitor_and_bridge(csi2_device_t *dev) {
    uint32_t int_status, core_status;
    uint32_t v4l2_frames, v4l2_seq;
    static uint32_t last_seq = 0;
    static uint32_t frame_count = 0;
    
    /* Read CSI2 status */
    int_status = csi2_read32(dev, CSI2_INT_STATUS);
    core_status = csi2_read32(dev, CSI2_CORE_STATUS);
    v4l2_frames = csi2_read32(dev, V4L2_FRAMES_CAPTURED);
    v4l2_seq = csi2_read32(dev, V4L2_SEQUENCE_NUMBER);
    
    /* Clear interrupts */
    if (int_status) {
        csi2_write32(dev, CSI2_INT_STATUS, int_status);
    }
    
    /* Check for new frames */
    if (v4l2_seq != last_seq) {
        printf("New frame detected! seq=%d -> %d\n", last_seq, v4l2_seq);
        
        if (dev->v4l2_found) {
            if (write_frame_to_v4l2(dev) == 0) {
                frame_count++;
                printf("Frame %d (seq:%d) -> V4L2 OK\n", frame_count, v4l2_seq);
            } else {
                printf("Frame %d (seq:%d) -> V4L2 FAILED (wrote %zd/%zu bytes)\n", 
                       frame_count, v4l2_seq, 
                       (ssize_t)write(dev->v4l2_fd, dev->frame_buffer, dev->frame_size), 
                       dev->frame_size);
            }
        }
        
        last_seq = v4l2_seq;
    }
}

int main(int argc, char *argv[]) {
    printf("CSI2 to V4L2 Bridge Starting (Auto-detect Version)...\n");
    
    /* Set up signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* Initialize CSI2 device */
    if (csi2_init(&g_csi2_dev) < 0) {
        fprintf(stderr, "Failed to initialize CSI2 device\n");
        return 1;
    }
    
    /* Initialize V4L2 loopback device */
    if (init_v4l2_device(&g_csi2_dev) < 0) {
        printf("V4L2 loopback device not available, running CSI2-only mode\n");
        g_csi2_dev.v4l2_found = false;
    }
    
    printf("Bridge initialized successfully\n");
    
    /* Read initial status */
    printf("Initial CSI2 device status:\n");
    printf("- Core config: 0x%08x\n", csi2_read32(&g_csi2_dev, CSI2_CORE_CONFIG));
    printf("- Core status: 0x%08x\n", csi2_read32(&g_csi2_dev, CSI2_CORE_STATUS));
    printf("- Streaming: 0x%08x\n", csi2_read32(&g_csi2_dev, V4L2_STREAMING));
    printf("- Frames captured: %d\n", csi2_read32(&g_csi2_dev, V4L2_FRAMES_CAPTURED));
    printf("- Sequence number: %d\n", csi2_read32(&g_csi2_dev, V4L2_SEQUENCE_NUMBER));
    
    /* Start streaming */
    printf("Starting CSI2 streaming...\n");
    csi2_write32(&g_csi2_dev, V4L2_STREAMING, 1);
    
    /* Check status after starting */
    printf("After starting streaming:\n");
    printf("- Streaming: 0x%08x\n", csi2_read32(&g_csi2_dev, V4L2_STREAMING));
    printf("- Frames captured: %d\n", csi2_read32(&g_csi2_dev, V4L2_FRAMES_CAPTURED));
    printf("- Sequence number: %d\n", csi2_read32(&g_csi2_dev, V4L2_SEQUENCE_NUMBER));
    
    /* Monitor loop */
    printf("Bridge running (Ctrl+C to stop)...\n");
    g_csi2_dev.running = 1;
    
    while (g_csi2_dev.running) {
        csi2_monitor_and_bridge(&g_csi2_dev);
        usleep(100000); /* 100ms */
    }
    
    /* Stop streaming */
    printf("\nStopping streaming...\n");
    csi2_write32(&g_csi2_dev, V4L2_STREAMING, 0);
    
    /* Cleanup */
    if (g_csi2_dev.v4l2_found && g_csi2_dev.v4l2_fd >= 0) {
        close(g_csi2_dev.v4l2_fd);
    }
    if (g_csi2_dev.frame_buffer) {
        free(g_csi2_dev.frame_buffer);
    }
    if (g_csi2_dev.mmio_base && g_csi2_dev.mmio_base != MAP_FAILED) {
        munmap(g_csi2_dev.mmio_base, g_csi2_dev.mmio_size);
    }
    if (g_csi2_dev.pci_fd >= 0) {
        close(g_csi2_dev.pci_fd);
    }
    
    printf("CSI2 to V4L2 bridge completed\n");
    return 0;
}
