#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

KSYMTAB_DATA(vb2_cxl_memops, "_gpl", "");
KSYMTAB_FUNC(vb2_cxl_pool_get_info, "_gpl", "");
KSYMTAB_FUNC(vb2_cxl_pool_create, "_gpl", "");
KSYMTAB_FUNC(vb2_cxl_pool_destroy, "_gpl", "");
KSYMTAB_FUNC(vb2_cxl_init, "_gpl", "");
KSYMTAB_FUNC(vb2_cxl_cleanup, "_gpl", "");

SYMBOL_CRC(vb2_cxl_memops, 0x7194e38f, "_gpl");
SYMBOL_CRC(vb2_cxl_pool_get_info, 0xfb963da7, "_gpl");
SYMBOL_CRC(vb2_cxl_pool_create, 0xc9a01190, "_gpl");
SYMBOL_CRC(vb2_cxl_pool_destroy, 0x9e063844, "_gpl");
SYMBOL_CRC(vb2_cxl_init, 0x80569fa0, "_gpl");
SYMBOL_CRC(vb2_cxl_cleanup, 0x6608c4c9, "_gpl");

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x6c224cda, "gen_pool_destroy" },
	{ 0x68c89da1, "pci_dev_put" },
	{ 0x9e9fdd9d, "memunmap" },
	{ 0x7c2b3c70, "device_unregister" },
	{ 0x037a0cba, "kfree" },
	{ 0x9c00e6df, "class_create" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0x57f5e89c, "kmalloc_caches" },
	{ 0x691e08da, "__kmalloc_cache_noprof" },
	{ 0xa916b694, "strnlen" },
	{ 0x476b165a, "sized_strscpy" },
	{ 0xeb3fc6d6, "pci_get_class" },
	{ 0x4d924f20, "memremap" },
	{ 0xced0f4d4, "gen_pool_create" },
	{ 0xbefa51a3, "gen_pool_add_owner" },
	{ 0x028d945d, "device_create" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x6a6c8425, "pci_get_device" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x19dee613, "__fortify_panic" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x2b593aa8, "gen_pool_alloc_algo_owner" },
	{ 0x4202864b, "pv_ops" },
	{ 0xd4ec10e6, "BUG_func" },
	{ 0x0296695f, "refcount_warn_saturate" },
	{ 0x900be4e5, "class_destroy" },
	{ 0x060ba97c, "gen_pool_free_owner" },
	{ 0xad459de2, "dma_unmap_resource" },
	{ 0x192c61a9, "param_ops_int" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xef45223d, "boot_cpu_data" },
	{ 0x170c7785, "__vma_start_write" },
	{ 0x8ff76340, "remap_pfn_range" },
	{ 0x2cf56265, "__dynamic_pr_debug" },
	{ 0x494e3393, "vm_get_page_prot" },
	{ 0x50d1f870, "pgprot_writecombine" },
	{ 0x93dc2586, "pgprot_writethrough" },
	{ 0x46cf10eb, "cachemode2protval" },
	{ 0x122c3a7e, "_printk" },
	{ 0x2464da17, "gen_pool_size" },
	{ 0xd0d3f0a4, "gen_pool_avail" },
	{ 0x359ce760, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "92CEDB5E5753BE097061AA2");
