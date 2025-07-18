// advanced_v4l2_test.c - 고급 V4L2 성능 및 기능 테스트
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
#include <signal.h>

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
    
    // Performance metrics
    uint64_t total_frames;
    uint64_t total_bytes;
    double start_time;
    double last_frame_time;
    uint32_t dropped_frames;
    uint32_t sequence_errors;
    uint32_t last_sequence;
} V4L2Device;

static volatile bool running = true;

static void signal_handler(int sig)
{
    printf("\n🛑 Signal %d received, stopping...\n", sig);
    running = false;
}

static double get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void print_performance_stats(V4L2Device *dev)
{
    double current_time = get_time();
    double elapsed = current_time - dev->start_time;
    double fps = dev->total_frames / elapsed;
    double bandwidth = (dev->total_bytes / 1024.0 / 1024.0) / elapsed; // MB/s
    
    printf("\n📊 Performance Statistics:\n");
    printf("  Runtime: %.2f seconds\n", elapsed);
    printf("  Total frames: %lu\n", dev->total_frames);
    printf("  Average FPS: %.1f\n", fps);
    printf("  Data rate: %.1f MB/s\n", bandwidth);
    printf("  Dropped frames: %u\n", dev->dropped_frames);
    printf("  Sequence errors: %u\n", dev->sequence_errors);
    printf("  Frame rate stability: %.1f%%\n", 
           dev->total_frames > 0 ? 100.0 * (dev->total_frames - dev->dropped_frames) / dev->total_frames : 0.0);
}

static int setup_device(V4L2Device *dev, const char *device_path)
{
    // Open device
    dev->fd = open(device_path, O_RDWR | O_NONBLOCK);
    if (dev->fd < 0) {
        printf("❌ Cannot open %s: %s\n", device_path, strerror(errno));
        return -1;
    }
    
    // Query capabilities
    if (ioctl(dev->fd, VIDIOC_QUERYCAP, &dev->cap) < 0) {
        printf("❌ VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
        return -1;
    }
    
    printf("✅ Device: %s (%s)\n", dev->cap.card, device_path);
    
    // Get format
    memset(&dev->format, 0, sizeof(dev->format));
    dev->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (ioctl(dev->fd, VIDIOC_G_FMT, &dev->format) < 0) {
        printf("❌ VIDIOC_G_FMT failed: %s\n", strerror(errno));
        return -1;
    }
    
    printf("📷 Format: %dx%d %.4s (%u bytes/line)\n",
           dev->format.fmt.pix.width,
           dev->format.fmt.pix.height,
           (char*)&dev->format.fmt.pix.pixelformat,
           dev->format.fmt.pix.bytesperline);
    
    return 0;
}

static int setup_buffers(V4L2Device *dev, int buffer_count)
{
    struct v4l2_requestbuffers reqbufs;
    
    memset(&reqbufs, 0, sizeof(reqbufs));
    reqbufs.count = buffer_count;
    reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbufs.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(dev->fd, VIDIOC_REQBUFS, &reqbufs) < 0) {
        printf("❌ VIDIOC_REQBUFS failed: %s\n", strerror(errno));
        return -1;
    }
    
    dev->buffer_count = reqbufs.count;
    printf("🗂️  Allocated %d buffers\n", dev->buffer_count);
    
    // Map buffers
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
    }
    
    return 0;
}

static int start_streaming(V4L2Device *dev)
{
    // Queue all buffers
    for (int i = 0; i < dev->buffer_count; i++) {
        if (ioctl(dev->fd, VIDIOC_QBUF, &dev->buffers[i]) < 0) {
            printf("❌ VIDIOC_QBUF failed for buffer %d: %s\n", 
                   i, strerror(errno));
            return -1;
        }
    }
    
    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(dev->fd, VIDIOC_STREAMON, &type) < 0) {
        printf("❌ VIDIOC_STREAMON failed: %s\n", strerror(errno));
        return -1;
    }
    
    dev->streaming = true;
    dev->start_time = get_time();
    dev->last_frame_time = dev->start_time;
    printf("🎬 Streaming started\n");
    
    return 0;
}

static void analyze_frame_quality(V4L2Device *dev, uint8_t *data, uint32_t size, uint32_t sequence)
{
    // Check sequence continuity
    if (dev->total_frames > 0) {
        uint32_t expected_seq = dev->last_sequence + 1;
        if (sequence != expected_seq) {
            dev->sequence_errors++;
            if (sequence > expected_seq) {
                dev->dropped_frames += (sequence - expected_seq);
            }
        }
    }
    dev->last_sequence = sequence;
    
    // Sample pixel analysis (check first few pixels)
    if (size >= 12) { // At least 4 RGB pixels
        uint8_t *rgb = data;
        printf("    🎨 First pixels: RGB(%d,%d,%d) RGB(%d,%d,%d) RGB(%d,%d,%d) RGB(%d,%d,%d)\n",
               rgb[0], rgb[1], rgb[2], rgb[3], rgb[4], rgb[5],
               rgb[6], rgb[7], rgb[8], rgb[9], rgb[10], rgb[11]);
    }
    
    // Check for pattern
    if (size >= 3) {
        uint8_t r = data[0], g = data[1], b = data[2];
        printf("    📊 Pattern check: R=%d, G=%d, B=%d (seq=%d)\n", r, g, b, sequence);
    }
}

static int capture_performance_test(V4L2Device *dev, int duration_seconds)
{
    struct v4l2_buffer buf;
    fd_set fds;
    struct timeval tv;
    double end_time = dev->start_time + duration_seconds;
    
    printf("\n🚀 Starting %d-second performance test...\n", duration_seconds);
    printf("Press Ctrl+C to stop early\n\n");
    
    while (running && get_time() < end_time) {
        FD_ZERO(&fds);
        FD_SET(dev->fd, &fds);
        
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        
        int ret = select(dev->fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            printf("❌ select() failed: %s\n", strerror(errno));
            break;
        } else if (ret == 0) {
            printf("⏰ Timeout waiting for frame\n");
            continue;
        }
        
        // Dequeue buffer
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(dev->fd, VIDIOC_DQBUF, &buf) < 0) {
            printf("❌ VIDIOC_DQBUF failed: %s\n", strerror(errno));
            break;
        }
        
        dev->total_frames++;
        dev->total_bytes += buf.bytesused;
        double current_time = get_time();
        double frame_interval = current_time - dev->last_frame_time;
        dev->last_frame_time = current_time;
        
        // Print progress every 30 frames
        if (dev->total_frames % 30 == 1 || dev->total_frames <= 5) {
            printf("📸 Frame %lu: seq=%d, size=%d, interval=%.3fs (%.1f fps)\n",
                   dev->total_frames, buf.sequence, buf.bytesused, 
                   frame_interval, 1.0 / frame_interval);
            
            uint8_t *data = (uint8_t*)dev->buffer_addrs[buf.index];
            analyze_frame_quality(dev, data, buf.bytesused, buf.sequence);
        }
        
        // Re-queue buffer
        if (ioctl(dev->fd, VIDIOC_QBUF, &buf) < 0) {
            printf("❌ VIDIOC_QBUF failed: %s\n", strerror(errno));
            break;
        }
        
        // Print stats every 5 seconds
        if ((int)(current_time - dev->start_time) % 5 == 0 && 
            current_time - dev->start_time >= 5.0 && 
            dev->total_frames % 150 == 0) {
            print_performance_stats(dev);
        }
    }
    
    return 0;
}

static void stop_streaming(V4L2Device *dev)
{
    if (!dev->streaming) return;
    
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(dev->fd, VIDIOC_STREAMOFF, &type) < 0) {
        printf("❌ VIDIOC_STREAMOFF failed: %s\n", strerror(errno));
    } else {
        printf("🛑 Streaming stopped\n");
    }
    
    dev->streaming = false;
}

static void cleanup_device(V4L2Device *dev)
{
    stop_streaming(dev);
    
    // Unmap buffers
    for (int i = 0; i < dev->buffer_count; i++) {
        if (dev->buffer_addrs[i] != MAP_FAILED) {
            munmap(dev->buffer_addrs[i], dev->buffers[i].length);
        }
    }
    
    if (dev->fd >= 0) {
        close(dev->fd);
        dev->fd = -1;
    }
}

static void print_usage(const char *prog_name)
{
    printf("Usage: %s [options]\n", prog_name);
    printf("Options:\n");
    printf("  -d <device>    V4L2 device path (default: /dev/video0)\n");
    printf("  -t <seconds>   Test duration in seconds (default: 10)\n");
    printf("  -b <count>     Number of buffers (default: 4)\n");
    printf("  -h             Show this help\n");
}

int main(int argc, char *argv[])
{
    V4L2Device dev = {0};
    const char *device_path = DEVICE_PATH;
    int duration = 10;
    int buffer_count = 4;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            device_path = argv[++i];
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            duration = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            buffer_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }
    
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("🎯 Advanced V4L2 CSI2 Performance Test\n");
    printf("=====================================\n");
    printf("Device: %s\n", device_path);
    printf("Duration: %d seconds\n", duration);
    printf("Buffers: %d\n", buffer_count);
    printf("\n");
    
    // Initialize device
    dev.fd = -1;
    for (int i = 0; i < MAX_BUFFERS; i++) {
        dev.buffer_addrs[i] = MAP_FAILED;
    }
    
    // Setup device
    if (setup_device(&dev, device_path) < 0) {
        cleanup_device(&dev);
        return 1;
    }
    
    // Setup buffers
    if (setup_buffers(&dev, buffer_count) < 0) {
        cleanup_device(&dev);
        return 1;
    }
    
    // Start streaming
    if (start_streaming(&dev) < 0) {
        cleanup_device(&dev);
        return 1;
    }
    
    // Run performance test
    capture_performance_test(&dev, duration);
    
    // Final statistics
    print_performance_stats(&dev);
    
    // Cleanup
    cleanup_device(&dev);
    
    printf("\n🏁 Test completed!\n");
    printf("Quality: %s\n", 
           dev.sequence_errors == 0 ? "✅ Perfect" : 
           dev.sequence_errors < 5 ? "⚠️  Good" : "❌ Poor");
    
    return (dev.total_frames > 0 && dev.sequence_errors < dev.total_frames * 0.1) ? 0 : 1;
}
