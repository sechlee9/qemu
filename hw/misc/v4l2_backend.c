// hw/misc/v4l2_backend.c - V4L2 백엔드 (hw/misc에 위치)
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "system/v4l2.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#ifdef CONFIG_LINUX
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#endif

typedef struct V4L2Backend {
    Object parent;
    
    char *device_path;
    char *device_name;
    int video_fd;
    bool opened;
    
#ifdef CONFIG_LINUX
    /* V4L2 capabilities */
    struct v4l2_capability cap;
    struct v4l2_format format;
#endif
    
    /* Frame handling */
    QemuMutex frame_mutex;
    QEMUTimer *frame_timer;
    uint32_t frame_sequence;
    
    /* Callbacks */
    V4L2FrameCallback frame_callback;
    void *callback_opaque;
    
} V4L2Backend;

#define TYPE_V4L2_BACKEND "v4l2-backend"
OBJECT_DECLARE_SIMPLE_TYPE(V4L2Backend, V4L2_BACKEND)

static void v4l2_backend_frame_timeout(void *opaque)
{
    V4L2Backend *v4l2 = V4L2_BACKEND(opaque);
    
    if (!v4l2->frame_callback) {
        goto reschedule;
    }
    
    qemu_mutex_lock(&v4l2->frame_mutex);
    
    V4L2Frame frame = {
        .width = 1280,
        .height = 720,
        .format = 0x32424752, /* RGB24 */
        .sequence = v4l2->frame_sequence++,
        .timestamp = qemu_clock_get_ns(QEMU_CLOCK_REALTIME),
        .size = 1280 * 720 * 3,
        .bytesperline = 1280 * 3
    };
    
    frame.data = g_malloc(frame.size);
    
    /* Generate test pattern - simple gradient */
    uint8_t *data = frame.data;
    for (int y = 0; y < frame.height; y++) {
        for (int x = 0; x < frame.width; x++) {
            int pixel_offset = (y * frame.width + x) * 3;
            data[pixel_offset + 0] = (x * 255) / frame.width;     /* R */
            data[pixel_offset + 1] = (y * 255) / frame.height;    /* G */
            data[pixel_offset + 2] = ((x + y) * 255) / (frame.width + frame.height); /* B */
        }
    }
    
    v4l2->frame_callback(&frame, v4l2->callback_opaque);
    g_free(frame.data);
    
    qemu_mutex_unlock(&v4l2->frame_mutex);

reschedule:
    /* 30 FPS = 33.33ms interval */
    timer_mod(v4l2->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
}

V4L2Backend *v4l2_backend_new(const char *device_name, Error **errp)
{
    V4L2Backend *v4l2 = V4L2_BACKEND(object_new(TYPE_V4L2_BACKEND));
    
    v4l2->device_name = g_strdup(device_name);
    v4l2->device_path = g_strdup("/dev/video0");
    v4l2->frame_sequence = 0;
    v4l2->video_fd = -1;
    v4l2->opened = false;
    
    qemu_mutex_init(&v4l2->frame_mutex);
    v4l2->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                    v4l2_backend_frame_timeout, v4l2);
    
    printf("V4L2 Backend: Created for %s\n", device_name);
    return v4l2;
}

void v4l2_backend_free(V4L2Backend *v4l2)
{
    if (!v4l2) return;
    
    v4l2_backend_stop(v4l2);
    
    if (v4l2->frame_timer) {
        timer_free(v4l2->frame_timer);
        v4l2->frame_timer = NULL;
    }
    
    qemu_mutex_destroy(&v4l2->frame_mutex);
    g_free(v4l2->device_name);
    g_free(v4l2->device_path);
    
    if (v4l2->video_fd >= 0) {
        close(v4l2->video_fd);
        v4l2->video_fd = -1;
    }
    
    object_unref(OBJECT(v4l2));
}

int v4l2_backend_start(V4L2Backend *v4l2, Error **errp)
{
    if (!v4l2) {
        error_setg(errp, "V4L2 backend is NULL");
        return -1;
    }

    /* Start frame generation timer */
    timer_mod(v4l2->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 33333333);
    printf("V4L2 Backend: Started (device: %s)\n", v4l2->device_path);
    return 0;
}

void v4l2_backend_stop(V4L2Backend *v4l2)
{
    if (!v4l2) return;
    
    timer_del(v4l2->frame_timer);
    
    if (v4l2->video_fd >= 0) {
        close(v4l2->video_fd);
        v4l2->video_fd = -1;
        v4l2->opened = false;
    }
    
    printf("V4L2 Backend: Stopped\n");
}

void v4l2_backend_set_frame_callback(V4L2Backend *v4l2, 
                                    V4L2FrameCallback callback,
                                    void *opaque)
{
    if (!v4l2) return;
    
    qemu_mutex_lock(&v4l2->frame_mutex);
    v4l2->frame_callback = callback;
    v4l2->callback_opaque = opaque;
    qemu_mutex_unlock(&v4l2->frame_mutex);
}

const char *v4l2_backend_get_device_path(V4L2Backend *v4l2)
{
    return v4l2 ? v4l2->device_path : NULL;
}

static void v4l2_backend_class_init(ObjectClass *klass, const void *data)
{
    /* Object class initialization */
}

static void v4l2_backend_instance_init(Object *obj)
{
    V4L2Backend *v4l2 = V4L2_BACKEND(obj);
    v4l2->video_fd = -1;
    v4l2->opened = false;
    v4l2->device_name = NULL;
    v4l2->device_path = NULL;
    v4l2->frame_callback = NULL;
    v4l2->callback_opaque = NULL;
}

static void v4l2_backend_finalize(Object *obj)
{
    V4L2Backend *v4l2 = V4L2_BACKEND(obj);
    
    v4l2_backend_stop(v4l2);
    
    g_free(v4l2->device_name);
    g_free(v4l2->device_path);
    
    if (v4l2->frame_timer) {
        timer_free(v4l2->frame_timer);
    }
}

static const TypeInfo v4l2_backend_info = {
    .name = TYPE_V4L2_BACKEND,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(V4L2Backend),
    .instance_init = v4l2_backend_instance_init,
    .instance_finalize = v4l2_backend_finalize,
    .class_init = v4l2_backend_class_init,
};

static void register_types(void)
{
    type_register_static(&v4l2_backend_info);
}

type_init(register_types)
