// v4l2_streaming_test.c - V4L2 스트리밍 테스트
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <time.h>
#include <sys/select.h>
#include <stdbool.h>

#define MAX_BUFFERS 8
#define DEVICE_PATH "/dev/video0"

typedef struct {
    int fd;
    struct v4l2_capability cap;
    struct v4l2_format format;
    struct v4l2_buffer buffers[MAX_BUFFERS];
    void *buffer_addrs[MAX_BUFFERS];
    int buffer_count;
    bool streaming;
} V4L2TestDevice;

static void print_capabilities(struct v4l2_capability *cap)
{
    printf("=== V4L2 Device Capabilities ===\n");
    printf("Driver: %s\n", cap->driver);
    printf("Card: %s\n", cap->card);
    printf("Bus: %s\n", cap->bus_info);
    printf("Version: %u.%u.%u\n", 
           (cap->version >> 16) & 0xFF,
           (cap->version >> 8) & 0xFF,
           cap->version & 0xFF);
    printf("Capabilities: 0x%08x\n", cap->capabilities);
    
    if (cap->capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        printf("  ✓ Video Capture\n");
    }
    if (cap->capabilities & V4L2_CAP_STREAMING) {
        printf("  ✓ Streaming I/O\n");
    }
    if (cap->capabilities & V4L2_CAP_READWRITE) {
        printf("  ✓ Read/Write I/O\n");
    }
    printf("\n");
}

static void print_format(struct v4l2_format *format)
{
    printf("=== Current Format ===\n");
    printf("Type: %u\n", format->type);
    printf("Width: %u\n", format->fmt.pix.width);
    printf("Height: %u\n", format->fmt.pix.height);
    printf("Pixel Format: 0x%08x (%.4s)\n", 
           format->fmt.pix.pixelformat,
           (char*)&format->fmt.pix.pixelformat);
    printf("Bytes per line: %u\n", format->fmt.pix.bytesperline);
    printf("Image size: %u\n", format->fmt.pix.sizeimage);
    printf("Field: %u\n", format->fmt.pix.field);
    printf("\n");
}

static int open_device(V4L2TestDevice *dev, const char *device_path)
{
    dev->fd = open(device_path, O_RDWR | O_NONBLOCK);
    if (dev->fd < 0) {
        printf("❌ Cannot open device %s: %s\n", device_path, strerror(errno));
        return -1;
    }
    
    printf("✅ Successfully opened device: %s\n", device_path);
    return 0;
}

static int query_capabilities(V4L2TestDevice *dev)
{
    if (ioctl(dev->fd, VIDIOC_QUERYCAP, &dev->cap) < 0) {
        printf("❌ VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
        return -1;
    }
    
    print_capabilities(&dev->cap);
    
    if (!(dev->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        printf("❌ Device does not support video capture\n");
        return -1;
    }
    
    if (!(dev->cap.capabilities & V4L2_CAP_STREAMING)) {
        printf("❌ Device does not support streaming\n");
        return -1;
    }
    
    return 0;
}

static int enumerate_formats(V4L2TestDevice *dev)
{
    struct v4l2_fmtdesc fmt;
    int index = 0;
    
    printf("=== Supported Formats ===\n");
    
    while (1) {
        memset(&fmt, 0, sizeof(fmt));
        fmt.index = index;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        
        if (ioctl(dev->fd, VIDIOC_ENUM_FMT, &fmt) < 0) {
            break;
        }
        
        printf("[%d] %s (0x%08x) - %.4s\n", 
               index, fmt.description, fmt.pixelformat,
               (char*)&fmt.pixelformat);
        
        index++;
    }
    
    printf("Total formats: %d\n\n", index);
    return index;
}

static int get_current_format(V4L2TestDevice *dev)
{
    memset(&dev->format, 0, sizeof(dev->format));
    dev->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (ioctl(dev->fd, VIDIOC_G_FMT, &dev->format) < 0) {
        printf("❌ VIDIOC_G_FMT failed: %s\n", strerror(errno));
        return -1;
    }
    
    print_format(&dev->format);
    return 0;
}

static int setup_buffers(V4L2TestDevice *dev)
{
    struct v4l2_requestbuffers reqbufs;
    
    printf("=== Setting up buffers ===\n");
    
    memset(&reqbufs, 0, sizeof(reqbufs));
    reqbufs.count = 4;
    reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbufs.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(dev->fd, VIDIOC_REQBUFS, &reqbufs) < 0) {
        printf("❌ VIDIOC_REQBUFS failed: %s\n", strerror(errno));
        return -1;
    }
    
    dev->buffer_count = reqbufs.count;
    printf("✅ Allocated %d buffers\n", dev->buffer_count);
    
    /* Map buffers */
    for (int i = 0; i < dev->buffer_count; i++) {
        memset(&dev->buffers[i], 0, sizeof(dev->buffers[i]));
        dev->buffers[i].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        dev->buffers[i].memory = V4L2_MEMORY_MMAP;
        dev->buffers[i].index = i;
        
        if (ioctl(dev->fd, VIDIOC_QUERYBUF, &dev->buffers[i]) < 0) {
            printf("❌ VIDIOC_QUERYBUF failed for buffer %d: %s\n", 
                   i, strerror(errno));
            return -1;
        }
        
        dev->buffer_addrs[i] = mmap(NULL, dev->buffers[i].length,
                                   PROT_READ | PROT_WRITE, MAP_SHARED,
                                   dev->fd, dev->buffers[i].m.offset);
        
        if (dev->buffer_addrs[i] == MAP_FAILED) {
            printf("❌ mmap failed for buffer %d: %s\n", i, strerror(errno));
            return -1;
        }
        
        printf("  Buffer %d: length=%u, offset=0x%08x, mapped=%p\n",
               i, dev->buffers[i].length, dev->buffers[i].m.offset, 
               dev->buffer_addrs[i]);
    }
    
    printf("\n");
    return 0;
}

static int start_streaming(V4L2TestDevice *dev)
{
    printf("=== Starting streaming ===\n");
    
    /* Queue all buffers */
    for (int i = 0; i < dev->buffer_count; i++) {
        if (ioctl(dev->fd, VIDIOC_QBUF, &dev->buffers[i]) < 0) {
            printf("❌ VIDIOC_QBUF failed for buffer %d: %s\n", 
                   i, strerror(errno));
            return -1;
        }
        printf("  ✓ Queued buffer %d\n", i);
    }
    
    /* Start streaming */
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(dev->fd, VIDIOC_STREAMON, &type) < 0) {
        printf("❌ VIDIOC_STREAMON failed: %s\n", strerror(errno));
        return -1;
    }
    
    dev->streaming = true;
    printf("✅ Streaming started\n\n");
    return 0;
}

static void analyze_frame_data(uint8_t *data, uint32_t size, uint32_t width, uint32_t height)
{
    if (size < width * height * 3) {
        printf("    ⚠️ Frame size too small: %u bytes\n", size);
        return;
    }
    
    uint64_t sum_r = 0, sum_g = 0, sum_b = 0;
    uint32_t pixel_count = (size / 3);
    uint8_t min_r = 255, max_r = 0;
    uint8_t min_g = 255, max_g = 0;
    uint8_t min_b = 255, max_b = 0;
    
    for (uint32_t i = 0; i < pixel_count; i++) {
        uint8_t r = data[i * 3];
        uint8_t g = data[i * 3 + 1];
        uint8_t b = data[i * 3 + 2];
        
        sum_r += r; sum_g += g; sum_b += b;
        
        if (r < min_r) min_r = r; 
        if (r > max_r) max_r = r;
        if (g < min_g) min_g = g; 
        if (g > max_g) max_g = g;
        if (b < min_b) min_b = b; 
        if (b > max_b) max_b = b;
    }
    
    printf("    📊 Pixel analysis (%u pixels):\n", pixel_count);
    printf("    Average RGB: (%lu, %lu, %lu)\n", 
           sum_r / pixel_count, sum_g / pixel_count, sum_b / pixel_count);
    printf("    Range R: %u-%u, G: %u-%u, B: %u-%u\n", 
           min_r, max_r, min_g, max_g, min_b, max_b);
}

static int capture_frames(V4L2TestDevice *dev, int frame_count)
{
    struct v4l2_buffer buf;
    fd_set fds;
    struct timeval tv;
    int frames_captured = 0;
    struct timespec start_time, current_time;
    
    printf("=== Capturing %d frames ===\n", frame_count);
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    
    while (frames_captured < frame_count) {
        FD_ZERO(&fds);
        FD_SET(dev->fd, &fds);
        
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        
        int ret = select(dev->fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            printf("❌ select() failed: %s\n", strerror(errno));
            break;
        } else if (ret == 0) {
            printf("⏰ Timeout waiting for frame %d\n", frames_captured + 1);
            continue;
        }
        
        /* Dequeue buffer */
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(dev->fd, VIDIOC_DQBUF, &buf) < 0) {
            printf("❌ VIDIOC_DQBUF failed: %s\n", strerror(errno));
            break;
        }
        
        frames_captured++;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed = (current_time.tv_sec - start_time.tv_sec) + 
                        (current_time.tv_nsec - start_time.tv_nsec) / 1e9;
        
        printf("📸 Frame %d captured:\n", frames_captured);
        printf("    Buffer index: %d\n", buf.index);
        printf("    Bytes used: %d\n", buf.bytesused);
        printf("    Sequence: %d\n", buf.sequence);
        printf("    Timestamp: %ld.%06ld\n", buf.timestamp.tv_sec, buf.timestamp.tv_usec);
        printf("    Elapsed: %.2f sec (%.1f fps)\n", elapsed, frames_captured / elapsed);
        
        /* Analyze frame data */
        uint8_t *data = (uint8_t*)dev->buffer_addrs[buf.index];
        analyze_frame_data(data, buf.bytesused, 
                          dev->format.fmt.pix.width, 
                          dev->format.fmt.pix.height);
        
        /* Re-queue buffer */
        if (ioctl(dev->fd, VIDIOC_QBUF, &buf) < 0) {
            printf("❌ VIDIOC_QBUF failed: %s\n", strerror(errno));
            break;
        }
        
        printf("\n");
        
        /* Short delay between frames for readability */
        usleep(100000); // 100ms
    }
    
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    double total_time = (current_time.tv_sec - start_time.tv_sec) + 
                       (current_time.tv_nsec - start_time.tv_nsec) / 1e9;
    
    printf("=== Capture Summary ===\n");
    printf("Frames captured: %d/%d\n", frames_captured, frame_count);
    printf("Total time: %.2f seconds\n", total_time);
    printf("Average FPS: %.1f\n", frames_captured / total_time);
    printf("\n");
    
    return frames_captured;
}

static void stop_streaming(V4L2TestDevice *dev)
{
    if (!dev->streaming) {
        return;
    }
    
    printf("=== Stopping streaming ===\n");
    
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(dev->fd, VIDIOC_STREAMOFF, &type) < 0) {
        printf("❌ VIDIOC_STREAMOFF failed: %s\n", strerror(errno));
    } else {
        printf("✅ Streaming stopped\n");
    }
    
    dev->streaming = false;
}

static void cleanup_device(V4L2TestDevice *dev)
{
    stop_streaming(dev);
    
    /* Unmap buffers */
    for (int i = 0; i < dev->buffer_count; i++) {
        if (dev->buffer_addrs[i] != MAP_FAILED) {
            munmap(dev->buffer_addrs[i], dev->buffers[i].length);
        }
    }
    
    if (dev->fd >= 0) {
        close(dev->fd);
        dev->fd = -1;
    }
    
    printf("✅ Cleanup completed\n");
}

static void print_usage(const char *prog_name)
{
    printf("Usage: %s [options]\n", prog_name);
    printf("Options:\n");
    printf("  -d <device>    V4L2 device path (default: /dev/video0)\n");
    printf("  -f <frames>    Number of frames to capture (default: 5)\n");
    printf("  -i             Show device info only\n");
    printf("  -h             Show this help\n");
}

int main(int argc, char *argv[])
{
    V4L2TestDevice dev = {0};
    const char *device_path = DEVICE_PATH;
    int frame_count = 5;
    bool info_only = false;
    
    /* Parse command line arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            device_path = argv[++i];
        } else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            frame_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-i") == 0) {
            info_only = true;
        } else if (strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }
    
    printf("🎥 V4L2 CSI2 Camera Streaming Test 🎥\n");
    printf("=====================================\n");
    printf("Device: %s\n", device_path);
    printf("Frames to capture: %d\n", frame_count);
    printf("\n");
    
    /* Open device */
    if (open_device(&dev, device_path) < 0) {
        return 1;
    }
    
    /* Query capabilities */
    if (query_capabilities(&dev) < 0) {
        cleanup_device(&dev);
        return 1;
    }
    
    /* Enumerate formats */
    enumerate_formats(&dev);
    
    /* Get current format */
    if (get_current_format(&dev) < 0) {
        cleanup_device(&dev);
        return 1;
    }
    
    if (info_only) {
        cleanup_device(&dev);
        return 0;
    }
    
    /* Setup buffers */
    if (setup_buffers(&dev) < 0) {
        cleanup_device(&dev);
        return 1;
    }
    
    /* Start streaming */
    if (start_streaming(&dev) < 0) {
        cleanup_device(&dev);
        return 1;
    }
    
    /* Capture frames */
    int captured = capture_frames(&dev, frame_count);
    
    /* Cleanup */
    cleanup_device(&dev);
    
    printf("🏁 Final Results 🏁\n");
    printf("==================\n");
    printf("Device: %s\n", device_path);
    printf("Requested frames: %d\n", frame_count);
    printf("Captured frames: %d\n", captured);
    printf("Success rate: %.1f%%\n", 
           frame_count > 0 ? (100.0 * captured / frame_count) : 0.0);
    
    if (captured >= frame_count * 0.8) { // 80% success rate
        printf("✅ Test PASSED\n");
        return 0;
    } else {
        printf("❌ Test FAILED\n");
        return 1;
    }
}
