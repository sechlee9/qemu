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
#include <signal.h>
#include <time.h>
#include <dirent.h>

/* CSI-2 Register Definitions (based on AMD spec) */
#define CSI2_CORE_CONFIG        0x00
#define CSI2_PROTOCOL_CONFIG    0x04
#define CSI2_CORE_STATUS        0x10
#define CSI2_GLOBAL_INT_ENABLE  0x20
#define CSI2_INT_STATUS         0x24
#define CSI2_INT_ENABLE         0x28
#define CSI2_VC_SELECTION       0x2C
#define CSI2_SHORT_PACKET       0x30
#define CSI2_IMG_INFO1_VC0      0x60
#define CSI2_IMG_INFO2_VC0      0x64

/* Interrupt bits */
#define CSI2_INT_FRAME_RECEIVED (1 << 31)
#define CSI2_INT_SoT_ERROR      (1 << 13)
#define CSI2_INT_CRC_ERROR      (1 << 9)
#define CSI2_INT_ECC_ERROR      (1 << 10)

/* V4L2 Bridge registers */
#define V4L2_BRIDGE_BASE        0x2000
#define V4L2_STREAMING          (V4L2_BRIDGE_BASE + 0x00)
#define V4L2_FRAMES_CAPTURED    (V4L2_BRIDGE_BASE + 0x04)
#define V4L2_SEQUENCE_NUMBER    (V4L2_BRIDGE_BASE + 0x08)

/* PCIe device info */
#define CSI2_VENDOR_ID          0x1234
#define CSI2_DEVICE_ID          0x5678

typedef struct {
    int pci_fd;
    void *mmio_base;
    size_t mmio_size;
    volatile int running;
    char pci_path[256];
} csi2_device_t;

static csi2_device_t g_csi2_dev = {0};

void signal_handler(int sig) {
    printf("\nReceived signal %d, stopping...\n", sig);
    g_csi2_dev.running = 0;
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

/* Enhanced PCI device search */
int find_csi2_device_enhanced(csi2_device_t *dev) {
    DIR *pci_dir;
    struct dirent *entry;
    char path[512];
    char vendor_path[512];
    char device_path[512];
    char resource_path[512];
    FILE *fp;
    uint16_t vendor, device;
    
    printf("Searching for CSI2 device in /sys/bus/pci/devices/...\n");
    
    pci_dir = opendir("/sys/bus/pci/devices");
    if (!pci_dir) {
        perror("Failed to open /sys/bus/pci/devices");
        return -1;
    }
    
    while ((entry = readdir(pci_dir)) != NULL) {
        if (entry->d_name[0] == '.') continue;
        
        /* Read vendor ID */
        snprintf(vendor_path, sizeof(vendor_path), 
                "/sys/bus/pci/devices/%s/vendor", entry->d_name);
        fp = fopen(vendor_path, "r");
        if (!fp) continue;
        if (fscanf(fp, "0x%hx", &vendor) != 1) {
            fclose(fp);
            continue;
        }
        fclose(fp);
        
        /* Read device ID */
        snprintf(device_path, sizeof(device_path), 
                "/sys/bus/pci/devices/%s/device", entry->d_name);
        fp = fopen(device_path, "r");
        if (!fp) continue;
        if (fscanf(fp, "0x%hx", &device) != 1) {
            fclose(fp);
            continue;
        }
        fclose(fp);
        
        printf("Found PCI device %s: %04x:%04x\n", entry->d_name, vendor, device);
        
        if (vendor == CSI2_VENDOR_ID && device == CSI2_DEVICE_ID) {
            printf("Found CSI2 device: %s\n", entry->d_name);
            strncpy(dev->pci_path, entry->d_name, sizeof(dev->pci_path) - 1);
            
            /* Try different resource access methods */
            
            /* Method 1: resource0 file */
            snprintf(resource_path, sizeof(resource_path), 
                    "/sys/bus/pci/devices/%s/resource0", entry->d_name);
            dev->pci_fd = open(resource_path, O_RDWR | O_SYNC);
            if (dev->pci_fd >= 0) {
                printf("Opened resource0: %s\n", resource_path);
                closedir(pci_dir);
                return 0;
            }
            
            /* Method 2: Try /dev/mem with base address */
            snprintf(path, sizeof(path), 
                    "/sys/bus/pci/devices/%s/resource", entry->d_name);
            fp = fopen(path, "r");
            if (fp) {
                unsigned long start, end, flags;
                if (fscanf(fp, "0x%lx 0x%lx 0x%lx", &start, &end, &flags) == 3) {
                    printf("Resource info: start=0x%lx, end=0x%lx, flags=0x%lx\n", 
                           start, end, flags);
                    if (start > 0) {
                        dev->pci_fd = open("/dev/mem", O_RDWR | O_SYNC);
                        if (dev->pci_fd >= 0) {
                            printf("Using /dev/mem access at 0x%lx\n", start);
                            dev->mmio_size = end - start;
                            dev->mmio_base = mmap(NULL, dev->mmio_size, 
                                                PROT_READ | PROT_WRITE, MAP_SHARED, 
                                                dev->pci_fd, start);
                            if (dev->mmio_base != MAP_FAILED) {
                                printf("Mapped device memory at %p, size: 0x%zx\n", 
                                       dev->mmio_base, dev->mmio_size);
                                fclose(fp);
                                closedir(pci_dir);
                                return 0;
                            }
                            close(dev->pci_fd);
                            dev->pci_fd = -1;
                        }
                    }
                }
                fclose(fp);
            }
            
            printf("Failed to access device resources\n");
        }
    }
    
    closedir(pci_dir);
    return -1;
}

/* Original PCI device search method */
int find_csi2_device_legacy(csi2_device_t *dev) {
    char path[256];
    char line[256];
    FILE *fp;
    int bus, slot, func;
    uint16_t vendor, device;
    
    fp = fopen("/proc/bus/pci/devices", "r");
    if (!fp) {
        printf("Cannot open /proc/bus/pci/devices, trying alternative method\n");
        return find_csi2_device_enhanced(dev);
    }
    
    while (fgets(line, sizeof(line), fp)) {
        if (sscanf(line, "%02x%02x\t%04x%04x", &bus, &slot, &vendor, &device) == 4) {
            if (vendor == CSI2_VENDOR_ID && device == CSI2_DEVICE_ID) {
                func = slot & 0x7;
                slot = (slot >> 3) & 0x1f;
                
                printf("Found CSI2 device at %02x:%02x.%d\n", bus, slot, func);
                
                /* Try multiple resource access patterns */
                snprintf(path, sizeof(path), 
                        "/sys/bus/pci/devices/0000:%02x:%02x.%d/resource0", 
                        bus, slot, func);
                
                dev->pci_fd = open(path, O_RDWR | O_SYNC);
                if (dev->pci_fd >= 0) {
                    fclose(fp);
                    return 0;
                }
                
                /* Alternative path format */
                snprintf(path, sizeof(path), 
                        "/sys/bus/pci/devices/%04x:%02x:%02x.%d/resource0", 
                        0, bus, slot, func);
                
                dev->pci_fd = open(path, O_RDWR | O_SYNC);
                if (dev->pci_fd >= 0) {
                    fclose(fp);
                    return 0;
                }
                
                printf("Failed to open resource file: %s\n", path);
                printf("Error: %s\n", strerror(errno));
            }
        }
    }
    
    fclose(fp);
    return find_csi2_device_enhanced(dev);
}

/* Initialize CSI2 device */
int csi2_init(csi2_device_t *dev) {
    struct stat st;
    
    if (find_csi2_device_legacy(dev) < 0) {
        return -1;
    }
    
    /* If we already mapped memory via /dev/mem, we're done */
    if (dev->mmio_base && dev->mmio_base != MAP_FAILED) {
        printf("Device already mapped successfully\n");
        return 0;
    }
    
    if (fstat(dev->pci_fd, &st) < 0) {
        perror("fstat");
        return -1;
    }
    
    dev->mmio_size = st.st_size;
    if (dev->mmio_size == 0) {
        dev->mmio_size = 0x10000; /* Default 64KB */
        printf("Using default MMIO size: 0x%zx\n", dev->mmio_size);
    }
    
    dev->mmio_base = mmap(NULL, dev->mmio_size, PROT_READ | PROT_WRITE, 
                          MAP_SHARED, dev->pci_fd, 0);
    if (dev->mmio_base == MAP_FAILED) {
        perror("mmap");
        printf("Trying alternative mapping...\n");
        
        /* Try smaller size */
        dev->mmio_size = 0x1000; /* 4KB */
        dev->mmio_base = mmap(NULL, dev->mmio_size, PROT_READ | PROT_WRITE, 
                              MAP_SHARED, dev->pci_fd, 0);
        if (dev->mmio_base == MAP_FAILED) {
            perror("mmap (alternative)");
            return -1;
        }
    }
    
    printf("CSI2 device mapped at %p, size: 0x%zx\n", dev->mmio_base, dev->mmio_size);
    return 0;
}

/* Cleanup CSI2 device */
void csi2_cleanup(csi2_device_t *dev) {
    printf("Cleaning up CSI2 device...\n");
    
    // V4L2 스트리밍 중지
    printf("Stopping V4L2 streaming...\n");
    csi2_write32(dev, V4L2_STREAMING, 0);
    
    // CSI2 Core 비활성화
    printf("Disabling CSI2 core...\n");
    csi2_write32(dev, CSI2_CORE_CONFIG, 0);
    
    // 인터럽트 비활성화
    printf("Disabling interrupts...\n");
    csi2_write32(dev, CSI2_GLOBAL_INT_ENABLE, 0);
    csi2_write32(dev, CSI2_INT_ENABLE, 0);
    
    // 상태 확인
    uint32_t core_config = csi2_read32(dev, CSI2_CORE_CONFIG);
    uint32_t v4l2_streaming = csi2_read32(dev, V4L2_STREAMING);
    
    printf("Final device state: Core=0x%x, V4L2=0x%x\n", 
           core_config, v4l2_streaming);
    
    if (core_config == 0 && v4l2_streaming == 0) {
        printf("✅ Device successfully stopped\n");
    } else {
        printf("⚠️ Device may still be active\n");
    }
}

/* Debug function to show system info */
void show_system_info() {
    printf("\n=== System Information ===\n");
    
    printf("Available PCI devices:\n");
    system("lspci 2>/dev/null | head -10");
    
    printf("\nPCI devices in sysfs:\n");
    system("ls -la /sys/bus/pci/devices/ 2>/dev/null | head -5");
    
    printf("\nLooking for our device:\n");
    system("find /sys/bus/pci/devices/ -name 'vendor' -exec grep -l '0x1234' {} \\; 2>/dev/null | head -3");
    
    printf("\nMemory info:\n");
    system("cat /proc/meminfo | grep -E '(MemTotal|MemFree)' 2>/dev/null");
    
    printf("\nKernel version:\n");
    system("uname -a 2>/dev/null");
    
    printf("\n");
}

/* Configure CSI2 device */
void csi2_configure(csi2_device_t *dev) {
    uint32_t val;
    
    printf("Configuring CSI2 device...\n");
    
    /* Read initial status */
    val = csi2_read32(dev, CSI2_CORE_CONFIG);
    printf("Initial Core Config: 0x%08x\n", val);
    
    val = csi2_read32(dev, CSI2_PROTOCOL_CONFIG);
    printf("Protocol Config: 0x%08x\n", val);
    
    /* Enable global interrupts */
    csi2_write32(dev, CSI2_GLOBAL_INT_ENABLE, 1);
    
    /* Enable frame received interrupt */
    csi2_write32(dev, CSI2_INT_ENABLE, CSI2_INT_FRAME_RECEIVED | 
                                       CSI2_INT_SoT_ERROR | 
                                       CSI2_INT_CRC_ERROR |
                                       CSI2_INT_ECC_ERROR);
    
    /* Enable all virtual channels */
    csi2_write32(dev, CSI2_VC_SELECTION, 0xFFFF);
    
    /* Enable core */
    csi2_write32(dev, CSI2_CORE_CONFIG, 1);
    
    printf("CSI2 device configured\n");
}

/* Start V4L2 streaming */
void csi2_start_streaming(csi2_device_t *dev) {
    printf("Starting V4L2 streaming...\n");
    csi2_write32(dev, V4L2_STREAMING, 1);
}

/* Stop V4L2 streaming */
void csi2_stop_streaming(csi2_device_t *dev) {
    printf("Stopping V4L2 streaming...\n");
    csi2_write32(dev, V4L2_STREAMING, 0);
}

/* Monitor CSI2 status */
void csi2_monitor(csi2_device_t *dev) {
    uint32_t int_status, core_status, frame_count;
    uint32_t v4l2_frames, v4l2_seq;
    static uint32_t last_frame_count = 0;
    static uint32_t last_v4l2_frames = 0;
    
    /* Read interrupt status */
    int_status = csi2_read32(dev, CSI2_INT_STATUS);
    
    /* Read core status */
    core_status = csi2_read32(dev, CSI2_CORE_STATUS);
    frame_count = (core_status >> 16) & 0xFFFF;
    
    /* Read V4L2 bridge status */
    v4l2_frames = csi2_read32(dev, V4L2_FRAMES_CAPTURED);
    v4l2_seq = csi2_read32(dev, V4L2_SEQUENCE_NUMBER);
    
    /* Clear interrupts */
    if (int_status) {
        csi2_write32(dev, CSI2_INT_STATUS, int_status);
    }
    
    /* Print status if there are changes */
    if (frame_count != last_frame_count || v4l2_frames != last_v4l2_frames || int_status) {
        printf("[%ld] INT: 0x%08x, Frames: %d->%d, V4L2: %d (seq:%d), Status: 0x%08x\n",
               time(NULL), int_status, last_frame_count, frame_count, 
               v4l2_frames, v4l2_seq, core_status);
        
        if (int_status & CSI2_INT_FRAME_RECEIVED) {
            printf("  -> Frame received interrupt\n");
        }
        if (int_status & CSI2_INT_SoT_ERROR) {
            printf("  -> SoT error detected\n");
        }
        if (int_status & CSI2_INT_CRC_ERROR) {
            printf("  -> CRC error detected\n");
        }
        if (int_status & CSI2_INT_ECC_ERROR) {
            printf("  -> ECC error detected\n");
        }
        
        last_frame_count = frame_count;
        last_v4l2_frames = v4l2_frames;
    }
}

/* Test register access */
void csi2_test_registers(csi2_device_t *dev) {
    uint32_t val;
    
    printf("\n=== Register Test ===\n");
    
    printf("Core Config (0x00): 0x%08x\n", csi2_read32(dev, CSI2_CORE_CONFIG));
    printf("Protocol Config (0x04): 0x%08x\n", csi2_read32(dev, CSI2_PROTOCOL_CONFIG));
    printf("Core Status (0x10): 0x%08x\n", csi2_read32(dev, CSI2_CORE_STATUS));
    printf("Global Int Enable (0x20): 0x%08x\n", csi2_read32(dev, CSI2_GLOBAL_INT_ENABLE));
    printf("Int Status (0x24): 0x%08x\n", csi2_read32(dev, CSI2_INT_STATUS));
    printf("Int Enable (0x28): 0x%08x\n", csi2_read32(dev, CSI2_INT_ENABLE));
    printf("VC Selection (0x2C): 0x%08x\n", csi2_read32(dev, CSI2_VC_SELECTION));
    printf("Short Packet (0x30): 0x%08x\n", csi2_read32(dev, CSI2_SHORT_PACKET));
    printf("Image Info1 VC0 (0x60): 0x%08x\n", csi2_read32(dev, CSI2_IMG_INFO1_VC0));
    printf("Image Info2 VC0 (0x64): 0x%08x\n", csi2_read32(dev, CSI2_IMG_INFO2_VC0));
    
    printf("\n=== V4L2 Bridge Registers ===\n");
    printf("V4L2 Streaming: 0x%08x\n", csi2_read32(dev, V4L2_STREAMING));
    printf("V4L2 Frames: 0x%08x\n", csi2_read32(dev, V4L2_FRAMES_CAPTURED));
    printf("V4L2 Sequence: 0x%08x\n", csi2_read32(dev, V4L2_SEQUENCE_NUMBER));
    
    /* Test write/read */
    printf("\n=== Write/Read Test ===\n");
    uint32_t test_val = 0xDEADBEEF;
    printf("Testing VC Selection register...\n");
    uint32_t orig = csi2_read32(dev, CSI2_VC_SELECTION);
    csi2_write32(dev, CSI2_VC_SELECTION, test_val);
    uint32_t readback = csi2_read32(dev, CSI2_VC_SELECTION);
    printf("Original: 0x%08x, Wrote: 0x%08x, Read: 0x%08x\n", orig, test_val, readback);
    csi2_write32(dev, CSI2_VC_SELECTION, orig); /* Restore */
}

void print_usage(const char *progname) {
    printf("Usage: %s [options]\n", progname);
    printf("Options:\n");
    printf("  -h, --help     Show this help\n");
    printf("  -t, --test     Run register test only\n");
    printf("  -m, --monitor  Monitor mode (default)\n");
    printf("  -s, --stream   Enable streaming\n");
    printf("  -d, --duration <seconds>  Run duration (default: 30)\n");
    printf("  -i, --info     Show system information\n");
}

int main(int argc, char *argv[]) {
    int test_only = 0;
    int enable_streaming = 0;
    int show_info = 0;
    int duration = 30;
    int i;
    
    /* Parse command line arguments */
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--test") == 0) {
            test_only = 1;
        } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--stream") == 0) {
            enable_streaming = 1;
        } else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--info") == 0) {
            show_info = 1;
        } else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--duration") == 0) {
            if (i + 1 < argc) {
                duration = atoi(argv[++i]);
            }
        }
    }
    
    printf("CSI-2 Device Test Application\n");
    printf("=============================\n");
    
    if (show_info) {
        show_system_info();
        if (test_only) return 0;
    }
    
    /* Initialize device */
    if (csi2_init(&g_csi2_dev) < 0) {
        fprintf(stderr, "Failed to initialize CSI2 device\n");
        show_system_info();
        return 1;
    }
    
    /* Set up signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* Configure device */
    csi2_configure(&g_csi2_dev);
    
    /* Run register test */
    csi2_test_registers(&g_csi2_dev);
    
    if (test_only) {
        printf("Register test completed\n");
        goto cleanup;
    }
    
    /* Start streaming if requested */
    if (enable_streaming) {
        csi2_start_streaming(&g_csi2_dev);
    }
    
    /* Monitor loop */
    printf("\nMonitoring CSI2 device for %d seconds (Ctrl+C to stop)...\n", duration);
    g_csi2_dev.running = 1;
    
    time_t start_time = time(NULL);
    while (g_csi2_dev.running && (time(NULL) - start_time) < duration) {
        csi2_monitor(&g_csi2_dev);
        usleep(100000); /* 100ms */
    }
    
    /* Stop streaming */
    if (enable_streaming) {
        csi2_stop_streaming(&g_csi2_dev);
    }
    
    printf("\nFinal status:\n");
    csi2_test_registers(&g_csi2_dev);
    
cleanup:
    csi2_cleanup(&g_csi2_dev);
    printf("CSI2 test completed\n");
    return 0;
}
