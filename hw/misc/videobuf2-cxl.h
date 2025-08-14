/* videobuf2-cxl.h - VideoBuf2 CXL memory allocator header */
#ifndef _MEDIA_VIDEOBUF2_CXL_H
#define _MEDIA_VIDEOBUF2_CXL_H

#include <media/videobuf2-v4l2.h>
#include <linux/pci.h>

struct vb2_cxl_pool;

/* Pool information structure */
struct vb2_cxl_pool_info {
    phys_addr_t phys_addr;
    size_t size;
    struct pci_dev *pci_dev;
    int cache_mode;
};

/* Extended vb2_queue context for CXL pool */
struct vb2_cxl_queue_ctx {
    struct vb2_queue queue;
    struct vb2_cxl_pool *pool;
};

/* CXL memory operations */
extern const struct vb2_mem_ops vb2_cxl_memops;

/* Pool management API */
struct vb2_cxl_pool *vb2_cxl_pool_create(const char *name, int cache_mode);
void vb2_cxl_pool_destroy(struct vb2_cxl_pool *pool);
int vb2_cxl_pool_get_info(struct vb2_cxl_pool *pool, struct vb2_cxl_pool_info *info);

/* Legacy API */
int vb2_cxl_init(const char *dax_device_name);
void vb2_cxl_cleanup(void);

#endif /* _MEDIA_VIDEOBUF2_CXL_H */
