/* videobuf2-cxl.c - VideoBuf2 CXL memory allocator */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/memremap.h>
#include <linux/pci.h>
#include <linux/pci-p2pdma.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-memops.h>
#include "videobuf2-cxl.h"

MODULE_DESCRIPTION("VideoBuf2 CXL memory allocator");
MODULE_AUTHOR("Your Name");
MODULE_LICENSE("GPL v2");

/* Module parameters */
static int default_cache_mode = MEMREMAP_WC;
module_param(default_cache_mode, int, 0644);
MODULE_PARM_DESC(default_cache_mode, "Default cache mode: 1=WB (default), 4=WC");

/* CXL Memory Device Class Code - already defined in pci_ids.h */
#ifndef PCI_CLASS_MEMORY_CXL
#define PCI_CLASS_MEMORY_CXL    0x0502
#endif

/* DMA mapping types */
enum vb2_cxl_dma_type {
    DMA_MAPPING_NONE,
    DMA_MAPPING_P2P,
    DMA_MAPPING_IOMMU
};

/* Statistics structure */
struct vb2_cxl_stats {
    atomic64_t alloc_count;
    atomic64_t free_count;
    atomic64_t cache_flush_count;
    atomic64_t cache_invalidate_count;
    atomic64_t p2p_success;
    atomic64_t iommu_fallback;
    atomic64_t alloc_failures;
};

/* CXL memory pool structure */
struct vb2_cxl_pool {
    void __iomem *virt_addr;
    phys_addr_t phys_addr;
    size_t size;
    struct gen_pool *pool;
    spinlock_t lock;
    struct pci_dev *cxl_pdev;
    int cxl_bar;
    int cache_mode;
    bool initialized;
    struct kref kref;
    struct list_head list;
    struct vb2_cxl_stats stats;
    char name[32];
    struct device *dev;  /* For sysfs */
};

/* Buffer structure */
struct vb2_cxl_buf {
    void *vaddr;
    phys_addr_t paddr;
    dma_addr_t dma_addr;
    size_t size;
    unsigned long pool_addr;
    struct vb2_vmarea_handler handler;
    refcount_t refcount;
    struct device *dev;
    enum vb2_cxl_dma_type dma_mapping_type;
    struct vb2_cxl_pool *pool;  /* Reference to pool */
};

/* Global pool management */
static LIST_HEAD(cxl_pools);
static DEFINE_MUTEX(cxl_pools_lock);
static struct class *vb2_cxl_class;
static int pool_index;

/* Cache management helpers for x86 */
#ifdef CONFIG_X86
static void cxl_flush_cache(void *addr, size_t size)
{
    unsigned long start = (unsigned long)addr;
    unsigned long end = start + size;
    
    for (; start < end; start += boot_cpu_data.x86_clflush_size)
        clflush((void *)start);
}

static void cxl_invalidate_cache(void *addr, size_t size)
{
    cxl_flush_cache(addr, size);
}
#else
static void cxl_flush_cache(void *addr, size_t size)
{
    flush_cache_vmap((unsigned long)addr, (unsigned long)addr + size);
}

static void cxl_invalidate_cache(void *addr, size_t size)
{
    invalidate_kernel_vmap_range(addr, size);
}
#endif

/* Find CXL device and get memory resource info */
static struct pci_dev *find_cxl_device_with_memory(phys_addr_t *phys, size_t *size, int *bar)
{
    struct pci_dev *pdev = NULL;
    int i;
    
    /* First try to find by class code (CXL memory device) */
    while ((pdev = pci_get_class(PCI_CLASS_MEMORY_CXL << 8, pdev)) != NULL) {
        pr_info("vb2-cxl: Found CXL memory device: %s\n", pci_name(pdev));
        
        /* Check each BAR for memory resource */
        for (i = 0; i < PCI_STD_NUM_BARS; i++) {
            if (pci_resource_flags(pdev, i) & IORESOURCE_MEM) {
                *phys = pci_resource_start(pdev, i);
                *size = pci_resource_len(pdev, i);
                
                /* Skip small BARs (likely control registers) */
                if (*size >= SZ_256M) {
                    pr_info("vb2-cxl: Found memory at BAR%d: 0x%llx, size %zu MB\n",
                            i, *phys, *size >> 20);
                    *bar = i;
                    return pdev;
                }
            }
        }
    }
    
    /* Fallback: try specific vendor devices */
    pdev = pci_get_device(0x8086, 0x0d93, NULL);  /* Intel CXL */
    if (!pdev) {
        pdev = pci_get_device(0x1022, 0x1a00, NULL);  /* AMD CXL (example) */
    }
    
    if (pdev) {
        /* Find memory BAR */
        for (i = 0; i < PCI_STD_NUM_BARS; i++) {
            if ((pci_resource_flags(pdev, i) & IORESOURCE_MEM) &&
                pci_resource_len(pdev, i) >= SZ_256M) {
                *phys = pci_resource_start(pdev, i);
                *size = pci_resource_len(pdev, i);
                *bar = i;
                pr_info("vb2-cxl: Found CXL device %s with memory at 0x%llx\n",
                        pci_name(pdev), *phys);
                return pdev;
            }
        }
        pci_dev_put(pdev);
    }
    
    return NULL;
}

/* Pool reference counting */
static void vb2_cxl_pool_release(struct kref *kref)
{
    struct vb2_cxl_pool *pool = container_of(kref, struct vb2_cxl_pool, kref);
    
    if (pool->pool) {
        size_t used = gen_pool_size(pool->pool) - gen_pool_avail(pool->pool);
        if (used > 0) {
            pr_warn("vb2-cxl: Pool %s: Warning: %zu bytes still in use\n", 
                    pool->name, used);
        }
        gen_pool_destroy(pool->pool);
    }
    
    if (pool->cxl_pdev) {
        pci_dev_put(pool->cxl_pdev);
    }
    
    if (pool->virt_addr) {
        memunmap(pool->virt_addr);
    }
    
    if (pool->dev) {
        device_unregister(pool->dev);
    }
    
    pr_info("vb2-cxl: Pool %s destroyed\n", pool->name);
    kfree(pool);
}

/* Sysfs attributes */
static ssize_t stats_show(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    struct vb2_cxl_pool *pool = dev_get_drvdata(dev);
    
    return sprintf(buf,
        "allocations: %lld\n"
        "frees: %lld\n"
        "cache_flushes: %lld\n"
        "cache_invalidates: %lld\n"
        "p2p_mappings: %lld\n"
        "iommu_mappings: %lld\n"
        "alloc_failures: %lld\n"
        "pool_size: %zu MB\n"
        "pool_available: %zu MB\n",
        atomic64_read(&pool->stats.alloc_count),
        atomic64_read(&pool->stats.free_count),
        atomic64_read(&pool->stats.cache_flush_count),
        atomic64_read(&pool->stats.cache_invalidate_count),
        atomic64_read(&pool->stats.p2p_success),
        atomic64_read(&pool->stats.iommu_fallback),
        atomic64_read(&pool->stats.alloc_failures),
        pool->size >> 20,
        gen_pool_avail(pool->pool) >> 20);
}
static DEVICE_ATTR_RO(stats);

static ssize_t cache_mode_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    struct vb2_cxl_pool *pool = dev_get_drvdata(dev);
    const char *mode_str;
    
    switch (pool->cache_mode) {
    case MEMREMAP_WB:
        mode_str = "write-back";
        break;
    case MEMREMAP_WT:
        mode_str = "write-through";
        break;
    case MEMREMAP_WC:
        mode_str = "write-combining";
        break;
    default:
        mode_str = "unknown";
    }
    
    return sprintf(buf, "%s\n", mode_str);
}
static DEVICE_ATTR_RO(cache_mode);

static struct attribute *vb2_cxl_attrs[] = {
    &dev_attr_stats.attr,
    &dev_attr_cache_mode.attr,
    NULL
};
ATTRIBUTE_GROUPS(vb2_cxl);

/* Get pool from vb2_queue context */
static struct vb2_cxl_pool *get_pool_from_vb(struct vb2_buffer *vb)
{
    /* Check if this is an extended queue context */
    struct vb2_cxl_queue_ctx *ctx = container_of(vb->vb2_queue, 
                                                 struct vb2_cxl_queue_ctx, 
                                                 queue);
    
    /* Validate by checking if the pool pointer looks valid */
    if (ctx && ctx->pool && ctx->pool->initialized) {
        return ctx->pool;
    }
    
    /* Fallback to default pool if available */
    return NULL;
}

/* vb2_cxl_alloc 함수 수정 - 직접 pool 사용 */
static void *vb2_cxl_alloc(struct vb2_buffer *vb, struct device *dev,
                          unsigned long size)
{
    struct vb2_cxl_pool *pool = NULL;
    struct vb2_cxl_buf *buf;
    unsigned long vaddr;
    
    pr_info("vb2-cxl: alloc called - size=%lu\n", size);
    pr_info("vb2-cxl: vb=%p, vb->vb2_queue=%p\n", vb, vb ? vb->vb2_queue : NULL);
    pr_info("vb2-cxl: dev=%p, dev name=%s\n", dev, dev ? dev_name(dev) : "null");
    
    /* 직접 pool 리스트에서 찾기 */
    mutex_lock(&cxl_pools_lock);
    list_for_each_entry(pool, &cxl_pools, list) {
        if (pool->initialized) {
            pr_info("vb2-cxl: Using pool: %s, addr=%p\n", pool->name, pool);
            break;
        }
    }
    mutex_unlock(&cxl_pools_lock);
    
    if (!pool || !pool->initialized) {
        pr_err("vb2-cxl: No initialized pool found\n");
        return ERR_PTR(-ENODEV);
    }
    
    /* 버퍼 구조체 할당 */
    buf = kzalloc(sizeof(*buf), GFP_KERNEL);
    if (!buf) {
        atomic64_inc(&pool->stats.alloc_failures);
        pr_err("vb2-cxl: Failed to allocate buffer structure\n");
        return ERR_PTR(-ENOMEM);
    }
    
    /* Page align size */
    size = PAGE_ALIGN(size);
    pr_info("vb2-cxl: Aligned size: %lu bytes\n", size);
    
    /* pool에서 메모리 할당 */
    spin_lock(&pool->lock);
    vaddr = gen_pool_alloc(pool->pool, size);
    spin_unlock(&pool->lock);
    
    if (!vaddr) {
        kfree(buf);
        atomic64_inc(&pool->stats.alloc_failures);
        pr_err("vb2-cxl: Failed to allocate %lu bytes from pool %s (available: %zu)\n", 
               size, pool->name, gen_pool_avail(pool->pool));
        return ERR_PTR(-ENOMEM);
    }
    
    buf->pool_addr = vaddr;
    buf->vaddr = (void *)vaddr;
    buf->size = size;
    buf->paddr = pool->phys_addr + (vaddr - (unsigned long)pool->virt_addr);
    buf->dma_addr = buf->paddr;  /* Default to physical address */
    buf->dev = dev;
    buf->pool = pool;
    buf->dma_mapping_type = DMA_MAPPING_NONE;
    
    refcount_set(&buf->refcount, 1);
    kref_get(&pool->kref);  /* Pool reference */
    
    /* Clear allocated memory */
    //memset(buf->vaddr, 0, size);
    
    /* Setup DMA mapping if needed - 간단히 처리 */
    if (dev) {
        buf->dma_addr = buf->paddr;  /* CXL 메모리는 직접 접근 가능 */
        pr_info("vb2-cxl: Using direct physical address for DMA: 0x%llx\n", 
                buf->dma_addr);
    }
    
    atomic64_inc(&pool->stats.alloc_count);
    pr_info("vb2-cxl: Successfully allocated %zu bytes at vaddr=%p, paddr=%llx, dma=%llx\n",
            size, buf->vaddr, buf->paddr, buf->dma_addr);
    
    return buf;
}

static void vb2_cxl_put(void *buf_priv)
{
    struct vb2_cxl_buf *buf = buf_priv;
    struct vb2_cxl_pool *pool = buf->pool;
    
    if (refcount_dec_and_test(&buf->refcount)) {
        pr_debug("vb2-cxl: Freeing buffer at %p\n", buf->vaddr);
        
        /* DMA mapping type-specific cleanup */
        switch (buf->dma_mapping_type) {
        case DMA_MAPPING_IOMMU:
            if (buf->dev && buf->dma_addr != buf->paddr) {
                dma_unmap_resource(buf->dev, buf->dma_addr, 
                                 buf->size, DMA_BIDIRECTIONAL, 0);
            }
            break;
        case DMA_MAPPING_P2P:
            /* P2P mapping doesn't need explicit cleanup */
            break;
        case DMA_MAPPING_NONE:
        default:
            break;
        }
        
        spin_lock(&pool->lock);
        gen_pool_free(pool->pool, buf->pool_addr, buf->size);
        spin_unlock(&pool->lock);
        
        atomic64_inc(&pool->stats.free_count);
        kref_put(&pool->kref, vb2_cxl_pool_release);
        kfree(buf);
    }
}

static void *vb2_cxl_get_userptr(struct vb2_buffer *vb, struct device *dev,
                                unsigned long vaddr, unsigned long size)
{
    /* CXL memory doesn't support userptr */
    return ERR_PTR(-EOPNOTSUPP);
}

static void vb2_cxl_put_userptr(void *buf_priv)
{
    /* Not implemented as get_userptr is not supported */
}

static void vb2_cxl_prepare(void *buf_priv)
{
    struct vb2_cxl_buf *buf = buf_priv;
    struct vb2_cxl_pool *pool = buf->pool;
    
    /* Skip cache operations for write-combining mode */
    if (pool->cache_mode != MEMREMAP_WC && buf->vaddr) {
        cxl_flush_cache(buf->vaddr, buf->size);
        atomic64_inc(&pool->stats.cache_flush_count);
    }
}

static void vb2_cxl_finish(void *buf_priv)
{
    struct vb2_cxl_buf *buf = buf_priv;
    struct vb2_cxl_pool *pool = buf->pool;
    
    /* Skip cache operations for write-combining mode */
    if (pool->cache_mode != MEMREMAP_WC && buf->vaddr) {
        cxl_invalidate_cache(buf->vaddr, buf->size);
        atomic64_inc(&pool->stats.cache_invalidate_count);
    }
}

static void *vb2_cxl_attach_dmabuf(struct vb2_buffer *vb, struct device *dev,
                                  struct dma_buf *dbuf, unsigned long size)
{
    /* CXL memory doesn't support dmabuf import */
    return ERR_PTR(-EOPNOTSUPP);
}

static void vb2_cxl_detach_dmabuf(void *buf_priv)
{
    /* Not implemented as attach_dmabuf is not supported */
}

static int vb2_cxl_map_dmabuf(void *buf_priv)
{
    /* Not implemented as dmabuf is not supported */
    return -EOPNOTSUPP;
}

static void vb2_cxl_unmap_dmabuf(void *buf_priv)
{
    /* Not implemented as dmabuf is not supported */
}

static void *vb2_cxl_cookie(struct vb2_buffer *vb, void *buf_priv)
{
    struct vb2_cxl_buf *buf = buf_priv;
    
    /* Return DMA address as a value casted to pointer */
    return (void *)(uintptr_t)buf->dma_addr;
}

static void *vb2_cxl_vaddr(struct vb2_buffer *vb, void *buf_priv)
{
    struct vb2_cxl_buf *buf = buf_priv;
    return buf->vaddr;
}

static unsigned int vb2_cxl_num_users(void *buf_priv)
{
    struct vb2_cxl_buf *buf = buf_priv;
    return refcount_read(&buf->refcount);
}

static int vb2_cxl_mmap(void *buf_priv, struct vm_area_struct *vma)
{
    struct vb2_cxl_buf *buf = buf_priv;
    struct vb2_cxl_pool *pool = buf->pool;
    unsigned long size = vma->vm_end - vma->vm_start;
    
    pr_debug("vb2-cxl: mmap called - size=%lu, buf_size=%zu\n", 
             size, buf->size);
    
    if (size > buf->size) {
        pr_err("vb2-cxl: mmap size (%lu) exceeds buffer size (%zu)\n",
               size, buf->size);
        return -EINVAL;
    }
    
    /* CXL 메모리의 cache mode에 따라 page protection 설정 */
    switch (pool->cache_mode) {
    case MEMREMAP_WB:
        /* Write-back: 일반 메모리처럼 캐시 사용 */
        vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
        break;
    case MEMREMAP_WC:
        /* Write-combining: 순차 쓰기에 최적화 */
        vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
        break;
    case MEMREMAP_WT:
        /* Write-through */
        vma->vm_page_prot = pgprot_writethrough(vma->vm_page_prot);
        break;
    default:
        /* Uncached */
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
        break;
    }
    
    /* VM flags 설정 - PFNMAP이 중요 */
    vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
    
    pr_debug("vb2-cxl: Mapping PFN 0x%llx to VMA 0x%lx-0x%lx (prot: 0x%lx)\n",
             (unsigned long long)(buf->paddr >> PAGE_SHIFT), 
             vma->vm_start, vma->vm_end, pgprot_val(vma->vm_page_prot));
    
    /* remap_pfn_range를 사용하여 물리 메모리 매핑 */
    return remap_pfn_range(vma, vma->vm_start,
                          buf->paddr >> PAGE_SHIFT,
                          size, vma->vm_page_prot);
}

/* Export vb2_mem_ops structure */
const struct vb2_mem_ops vb2_cxl_memops = {
    .alloc = vb2_cxl_alloc,
    .put = vb2_cxl_put,
    .get_userptr = vb2_cxl_get_userptr,
    .put_userptr = vb2_cxl_put_userptr,
    .prepare = vb2_cxl_prepare,
    .finish = vb2_cxl_finish,
    .attach_dmabuf = vb2_cxl_attach_dmabuf,
    .detach_dmabuf = vb2_cxl_detach_dmabuf,
    .map_dmabuf = vb2_cxl_map_dmabuf,
    .unmap_dmabuf = vb2_cxl_unmap_dmabuf,
    .cookie = vb2_cxl_cookie,
    .vaddr = vb2_cxl_vaddr,
    .mmap = vb2_cxl_mmap,
    .num_users = vb2_cxl_num_users,
};
EXPORT_SYMBOL_GPL(vb2_cxl_memops);

/* Get pool information */
int vb2_cxl_pool_get_info(struct vb2_cxl_pool *pool, struct vb2_cxl_pool_info *info)
{
    if (!pool || !info)
        return -EINVAL;
    
    if (!pool->initialized)
        return -ENODEV;
    
    info->phys_addr = pool->phys_addr;
    info->size = pool->size;
    info->pci_dev = pool->cxl_pdev;
    info->cache_mode = pool->cache_mode;

    pr_info("vb2-cxl: pool_get_info - pci_dev: %s\n",
            pool->cxl_pdev ? pci_name(pool->cxl_pdev) : "none");
    
    return 0;
}
EXPORT_SYMBOL_GPL(vb2_cxl_pool_get_info);

/* Pool creation */
struct vb2_cxl_pool *vb2_cxl_pool_create(const char *name, int cache_mode)
{
    struct vb2_cxl_pool *pool;
    phys_addr_t phys_addr = 0;
    size_t size = 0;
    int bar = 0;
    int ret;
    
    pool = kzalloc(sizeof(*pool), GFP_KERNEL);
    if (!pool)
        return ERR_PTR(-ENOMEM);
    
    spin_lock_init(&pool->lock);
    kref_init(&pool->kref);
    strscpy(pool->name, name ? name : "default", sizeof(pool->name));
    pool->cache_mode = cache_mode ? cache_mode : default_cache_mode;
    
    /* Try to find CXL device and get memory info */
    pool->cxl_pdev = find_cxl_device_with_memory(&phys_addr, &size, &bar);
    
    if (pool->cxl_pdev) {
        /* Use dynamically discovered values */
        pool->phys_addr = phys_addr;
        pool->size = size;
        pool->cxl_bar = bar;
        pr_info("vb2-cxl: Pool %s using discovered CXL memory at 0x%llx, size: %zu MB\n",
                pool->name, pool->phys_addr, pool->size >> 20);
    } else {
        /* Fallback to hardcoded values for testing */
        pr_warn("vb2-cxl: Pool %s: No CXL device found, using fallback address\n",
                pool->name);
        pool->phys_addr = 0xa90000000ULL;
        pool->size = SZ_512M;
    }
    
    /* Map CXL memory with selected cache mode */
    pool->virt_addr = memremap(pool->phys_addr, pool->size, pool->cache_mode);
    if (!pool->virt_addr) {
        pr_err("vb2-cxl: Pool %s: Failed to map CXL memory\n", pool->name);
        if (pool->cxl_pdev) {
            pci_dev_put(pool->cxl_pdev);
        }
        kfree(pool);
        return ERR_PTR(-ENOMEM);
    }
    
    pr_info("vb2-cxl: Pool %s: Memory mapped successfully (cache mode: %d)\n",
            pool->name, pool->cache_mode);
    
    /* Create memory pool */
    pool->pool = gen_pool_create(PAGE_SHIFT, NUMA_NO_NODE);
    if (!pool->pool) {
        memunmap(pool->virt_addr);
        if (pool->cxl_pdev) {
            pci_dev_put(pool->cxl_pdev);
        }
        kfree(pool);
        return ERR_PTR(-ENOMEM);
    }
    
    ret = gen_pool_add_virt(pool->pool,
                           (unsigned long)pool->virt_addr,
                           pool->phys_addr,
                           pool->size, NUMA_NO_NODE);
    if (ret) {
        gen_pool_destroy(pool->pool);
        memunmap(pool->virt_addr);
        if (pool->cxl_pdev) {
            pci_dev_put(pool->cxl_pdev);
        }
        kfree(pool);
        return ERR_PTR(ret);
    }
    
    /* Create sysfs device */
    if (vb2_cxl_class) {
        pool->dev = device_create(vb2_cxl_class, NULL, 
                                 MKDEV(0, pool_index++),
                                 pool, "cxl_pool_%s", pool->name);
        if (IS_ERR(pool->dev)) {
            pool->dev = NULL;
        } else {
            dev_set_drvdata(pool->dev, pool);
        }
    }
    
    pool->initialized = true;
    
    /* Add to global list */
    mutex_lock(&cxl_pools_lock);
    list_add(&pool->list, &cxl_pools);
    mutex_unlock(&cxl_pools_lock);
    
    pr_info("vb2-cxl: Pool %s created successfully\n", pool->name);
    pr_info("vb2-cxl: Available memory: %zu MB\n", 
            gen_pool_avail(pool->pool) >> 20);

    /* pool 생성 완료 후 상태 출력 */
    pr_info("vb2-cxl: Pool %s initialization complete:\n", pool->name);
    pr_info("  Physical address: 0x%llx\n", pool->phys_addr);
    pr_info("  Virtual address: %p\n", pool->virt_addr);
    pr_info("  Size: %zu MB\n", pool->size >> 20);
    pr_info("  Available: %zu MB\n", gen_pool_avail(pool->pool) >> 20);
    pr_info("  Pool pointer: %p\n", pool);
    
    return pool;
}
EXPORT_SYMBOL_GPL(vb2_cxl_pool_create);

/* Pool destruction */
void vb2_cxl_pool_destroy(struct vb2_cxl_pool *pool)
{
    if (!pool)
        return;
    
    mutex_lock(&cxl_pools_lock);
    list_del(&pool->list);
    mutex_unlock(&cxl_pools_lock);
    
    pool->initialized = false;
    kref_put(&pool->kref, vb2_cxl_pool_release);
}
EXPORT_SYMBOL_GPL(vb2_cxl_pool_destroy);

/* Legacy single pool API for backward compatibility */
static struct vb2_cxl_pool *default_pool;

int vb2_cxl_init(const char *dax_device_name)
{
    if (default_pool)
        return 0;
    
    default_pool = vb2_cxl_pool_create("default", default_cache_mode);
    if (IS_ERR(default_pool)) {
        int ret = PTR_ERR(default_pool);
        default_pool = NULL;
        return ret;
    }
    
    return 0;
}
EXPORT_SYMBOL_GPL(vb2_cxl_init);

void vb2_cxl_cleanup(void)
{
    if (default_pool) {
        vb2_cxl_pool_destroy(default_pool);
        default_pool = NULL;
    }
}
EXPORT_SYMBOL_GPL(vb2_cxl_cleanup);

/* Module initialization/cleanup */
static int __init vb2_cxl_module_init(void)
{
    /* Create sysfs class */
    vb2_cxl_class = class_create("vb2_cxl");
    if (IS_ERR(vb2_cxl_class)) {
        pr_err("vb2-cxl: Failed to create class\n");
        vb2_cxl_class = NULL;
    } else {
        vb2_cxl_class->dev_groups = vb2_cxl_groups;
    }
    
    pr_info("vb2-cxl: VideoBuf2 CXL allocator loaded\n");
    return 0;
}

static void __exit vb2_cxl_module_exit(void)
{
    struct vb2_cxl_pool *pool, *tmp;
    
    /* Clean up all pools */
    mutex_lock(&cxl_pools_lock);
    list_for_each_entry_safe(pool, tmp, &cxl_pools, list) {
        list_del(&pool->list);
        pool->initialized = false;
        kref_put(&pool->kref, vb2_cxl_pool_release);
    }
    mutex_unlock(&cxl_pools_lock);
    
    if (vb2_cxl_class) {
        class_destroy(vb2_cxl_class);
    }
    
    pr_info("vb2-cxl: VideoBuf2 CXL allocator unloaded\n");
}

module_init(vb2_cxl_module_init);
module_exit(vb2_cxl_module_exit);
