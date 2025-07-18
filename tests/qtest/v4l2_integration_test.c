// tests/qtest/v4l2_integration_test.c - V4L2 통합 테스트
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
    printf("V4L2 Device Capabilities:\n");
    printf("  Driver: %s\n", cap->driver);
    printf("  Card: %s\n", cap->card);
    printf("  Bus: %s\n", cap->bus_info);
    printf("  Version: %u.%u.%u\n", 
           (cap->version >> 16) & 0xFF,
           (cap->version >> 8) & 0xFF,
           cap->version & 0xFF);
    printf("  Capabilities: 0x%08x\n", cap->capabilities);
    
    if (cap->capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        printf("    - Video Capture\n");
    }
    if (cap->capabilities & V4L2_CAP_VIDEO_OUTPUT) {
        printf("    - Video Output\n");
    }
    if (cap->capabilities & V4L2_CAP_STREAMING) {
        printf("    - Streaming I/O\n");
    }
    if (cap->capabilities & V4L2_CAP_READWRITE) {
        printf("    - Read/Write I/O\n");
    }
}

static void print_format(struct v4l2_format *format)
{
    printf("V4L2 Format:\n");
    printf("  Type: %u\n", format->type);
    printf("  Width: %u\n", format->fmt.pix.width);
    printf("  Height: %u\n", format->fmt.pix.height);
    printf("  Pixel Format: 0x%08x (%.4s)\n", 
           format->fmt.pix.pixelformat,
           (char*)&format->fmt.pix.pixelformat);
    printf("  Bytes per line: %u\n", format->fmt.pix.bytesperline);
    printf("  Image size: %u\n", format->fmt.pix.sizeimage);
    printf("  Field: %u\n", format->fmt.pix.field);
}

static int v4l2_test_open_device(V4L2TestDevice *dev, const char *device_path)
{
    dev->fd = open(device_path, O_RDWR | O_NONBLOCK);
    if (dev->fd < 0) {
        printf("Error: Cannot open device %s: %s\n", device_path, strerror(errno));
        return -1;
    }
    
    printf("Successfully opened device: %s\n", device_path);
    return 0;
}

static int v4l2_test_query_caps(V4L2TestDevice *dev)
{
    if (ioctl(dev->fd, VIDIOC_QUERYCAP, &dev->cap) < 0) {
        printf("Error: VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
        return -1;
    }
    
    print_capabilities(&dev->cap);
    return 0;
}

static int v4l2_test_set_format(V4L2TestDevice *dev, uint32_t width, uint32_t height)
{
    memset(&dev->format, 0, sizeof(dev->format));
    dev->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    /* Get current format first */
    if (ioctl(dev->fd, VIDIOC_G_FMT, &dev->format) < 0) {
        printf("Error: VIDIOC_G_FMT failed: %s\n", strerror(errno));
        return -1;
    }
    
    printf("Current format:\n");
    print_format(&dev->format);
    
    /* Set desired format */
    dev->format.fmt.pix.width = width;
    dev->format.fmt.pix.height = height;
    dev->format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    dev->format.fmt.pix.field = V4L2_FIELD_NONE;
    
    if (ioctl(dev->fd, VIDIOC_S_FMT, &dev->format) < 0) {
        printf("Error: VIDIOC_S_FMT failed: %s\n", strerror(errno));
        return -1;
    }
    
    printf("Set format:\n");
    print_format(&dev->format);
    return 0;
}

static int v4l2_test_setup_buffers(V4L2TestDevice *dev)
{
    struct v4l2_requestbuffers reqbufs;
    
    memset(&reqbufs, 0, sizeof(reqbufs));
    reqbufs.count = 4;
    reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbufs.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(dev->fd, VIDIOC_REQBUFS, &reqbufs) < 0) {
        printf("Error: VIDIOC_REQBUFS failed: %s\n", strerror(errno));
        return -1;
    }
    
    dev->buffer_count = reqbufs.count;
    printf("Allocated %d buffers\n", dev->buffer_count);
    
    /* Map buffers */
    for (int i = 0; i < dev->buffer_count; i++) {
        memset(&dev->buffers[i], 0, sizeof(dev->buffers[i]));
        dev->buffers[i].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        dev->buffers[i].memory = V4L2_MEMORY_MMAP;
        dev->buffers[i].index = i;
        
        if (ioctl(dev->fd, VIDIOC_QUERYBUF, &dev->buffers[i]) < 0) {
            printf("Error: VIDIOC_QUERYBUF failed for buffer %d: %s\n", 
                   i, strerror(errno));
            return -1;
        }
        
        dev->buffer_addrs[i] = mmap(NULL, dev->buffers[i].length,
                                   PROT_READ | PROT_WRITE, MAP_SHARED,
                                   dev->fd, dev->buffers[i].m.offset);
        
        if (dev->buffer_addrs[i] == MAP_FAILED) {
            printf("Error: mmap failed for buffer %d: %s\n", i, strerror(errno));
            return -1;
        }
        
        printf("Buffer %d: length=%u, offset=0x%08x, mapped=%p\n",
               i, dev->buffers[i].length, dev->buffers[i].m.offset, 
               dev->buffer_addrs[i]);
    }
    
    return 0;
}

static int v4l2_test_start_streaming(V4L2TestDevice *dev)
{
    /* Queue all buffers */
    for (int i = 0; i < dev->buffer_count; i++) {
        if (ioctl(dev->fd, VIDIOC_QBUF, &dev->buffers[i]) < 0) {
            printf("Error: VIDIOC_QBUF failed for buffer %d: %s\n", 
                   i, strerror(errno));
            return -1;
        }
    }
    
    /* Start streaming */
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(dev->fd, VIDIOC_STREAMON, &type) < 0) {
        printf("Error: VIDIOC_STREAMON failed: %s\n", strerror(errno));
        return -1;
    }
    
    dev->streaming = true;
    printf("Streaming started\n");
    return 0;
}

static int v4l2_test_capture_frames(V4L2TestDevice *dev, int frame_count)
{
    struct v4l2_buffer buf;
    fd_set fds;
    struct timeval tv;
    int frames_captured = 0;
    
    printf("Capturing %d frames...\n", frame_count);
    
    while (frames_captured < frame_count) {
        FD_ZERO(&fds);
        FD_SET(dev->fd, &fds);
        
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        
        int ret = select(dev->fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            printf("Error: select() failed: %s\n", strerror(errno));
            break;
        } else if (ret == 0) {
            printf("Warning: Timeout waiting for frame\n");
            continue;
        }
        
        /* Dequeue buffer */
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(dev->fd, VIDIOC_DQBUF, &buf) < 0) {
            printf("Error: VIDIOC_DQBUF failed: %s\n", strerror(errno));
            break;
        }
        
        frames_captured++;
        printf("Frame %d: index=%d, bytesused=%d, sequence=%d, timestamp=%ld.%06ld\n",
               frames_captured, buf.index, buf.bytesused, buf.sequence,
               buf.timestamp.tv_sec, buf.timestamp.tv_usec);
        
        /* Analyze frame data */
        uint8_t *data = (uint8_t*)dev->buffer_addrs[buf.index];
        uint32_t avg_r = 0, avg_g = 0, avg_b = 0;
        int pixel_count = buf.bytesused / 3;
        
        for (int i = 0; i < pixel_count; i++) {
            avg_r += data[i * 3];
            avg_g += data[i * 3 + 1];
            avg_b += data[i * 3 + 2];
        }
        
        if (pixel_count > 0) {
            avg_r /= pixel_count;
            avg_g /= pixel_count;
            avg_b /= pixel_count;
            printf("  Average RGB: (%d, %d, %d)\n", avg_r, avg_g, avg_b);
        }
        
        /* Re-queue buffer */
        if (ioctl(dev->fd, VIDIOC_QBUF, &buf) < 0) {
            printf("Error: VIDIOC_QBUF failed: %s\n", strerror(errno));
            break;
        }
    }
    
    printf("Captured %d frames total\n", frames_captured);
    return frames_captured;
}

static void v4l2_test_stop_streaming(V4L2TestDevice *dev)
{
    if (!dev->streaming) {
        return;
    }
    
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(dev->fd, VIDIOC_STREAMOFF, &type) < 0) {
        printf("Error: VIDIOC_STREAMOFF failed: %s\n", strerror(errno));
    }
    
    dev->streaming = false;
    printf("Streaming stopped\n");
}

static void v4l2_test_cleanup(V4L2TestDevice *dev)
{
    v4l2_test_stop_streaming(dev);
    
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
    
    printf("Cleanup completed\n");
}

static void v4l2_test_enumerate_formats(V4L2TestDevice *dev)
{
    struct v4l2_fmtdesc fmt;
    int index = 0;
    
    printf("Supported formats:\n");
    
    while (1) {
        memset(&fmt, 0, sizeof(fmt));
        fmt.index = index;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        
        if (ioctl(dev->fd, VIDIOC_ENUM_FMT, &fmt) < 0) {
            break;
        }
        
        printf("  [%d] %s (0x%08x) - %.4s\n", 
               index, fmt.description, fmt.pixelformat,
               (char*)&fmt.pixelformat);
        
        /* Enumerate frame sizes for this format */
        struct v4l2_frmsizeenum frmsize;
        int size_index = 0;
        
        while (1) {
            memset(&frmsize, 0, sizeof(frmsize));
            frmsize.index = size_index;
            frmsize.pixel_format = fmt.pixelformat;
            
            if (ioctl(dev->fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) < 0) {
                break;
            }
            
            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                printf("    Size: %dx%d\n", 
                       frmsize.discrete.width, frmsize.discrete.height);
            }
            
            size_index++;
        }
        
        index++;
    }
}

static void print_usage(const char *prog_name)
{
    printf("Usage: %s [options]\n", prog_name);
    printf("Options:\n");
    printf("  -d <device>    V4L2 device path (default: /dev/video0)\n");
    printf("  -w <width>     Frame width (default: 1280)\n");
    printf("  -h <height>    Frame height (default: 720)\n");
    printf("  -f <frames>    Number of frames to capture (default: 10)\n");
    printf("  -i             Show device info only\n");
    printf("  -e             Enumerate formats only\n");
    printf("  --help         Show this help\n");
}

int main(int argc, char *argv[])
{
    V4L2TestDevice dev = {0};
    const char *device_path = DEVICE_PATH;
    uint32_t width = 1280;
    uint32_t height = 720;
    int frame_count = 10;
    bool info_only = false;
    bool enum_only = false;
    
    /* Parse command line arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            device_path = argv[++i];
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            width = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
            height = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            frame_count = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-i") == 0) {
            info_only = true;
        } else if (strcmp(argv[i], "-e") == 0) {
            enum_only = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }
    
    printf("=== V4L2 Integration Test ===\n");
    printf("Device: %s\n", device_path);
    printf("Resolution: %dx%d\n", width, height);
    printf("Frames: %d\n", frame_count);
    printf("\n");
    
    /* Open device */
    if (v4l2_test_open_device(&dev, device_path) < 0) {
        return 1;
    }
    
    /* Query capabilities */
    if (v4l2_test_query_caps(&dev) < 0) {
        v4l2_test_cleanup(&dev);
        return 1;
    }
    
    if (info_only) {
        v4l2_test_cleanup(&dev);
        return 0;
    }
    
    /* Enumerate formats */
    v4l2_test_enumerate_formats(&dev);
    
    if (enum_only) {
        v4l2_test_cleanup(&dev);
        return 0;
    }
    
    /* Set format */
    if (v4l2_test_set_format(&dev, width, height) < 0) {
        v4l2_test_cleanup(&dev);
        return 1;
    }
    
    /* Setup buffers */
    if (v4l2_test_setup_buffers(&dev) < 0) {
        v4l2_test_cleanup(&dev);
        return 1;
    }
    
    /* Start streaming */
    if (v4l2_test_start_streaming(&dev) < 0) {
        v4l2_test_cleanup(&dev);
        return 1;
    }
    
    /* Capture frames */
    int captured = v4l2_test_capture_frames(&dev, frame_count);
    
    /* Cleanup */
    v4l2_test_cleanup(&dev);
    
    printf("\n=== Test Summary ===\n");
    printf("Device: %s\n", device_path);
    printf("Requested frames: %d\n", frame_count);
    printf("Captured frames: %d\n", captured);
    printf("Success rate: %.1f%%\n", 
           frame_count > 0 ? (100.0 * captured / frame_count) : 0.0);
    
    if (captured == frame_count) {
        printf("✅ Test PASSED\n");
        return 0;
    } else {
        printf("❌ Test FAILED\n");
        return 1;
    }
}
