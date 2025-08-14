# AMD CSI2 V4L2 Driver Makefile with VideoBuf2-CXL and NVMe-CMB support
# For QEMU Guest with custom kernel source

obj-m += amd_csi2_v4l2.o

# Kernel source directory - QEMU guest kernel
KERNEL_SRC := ../../generic/linux-6.15.4
KDIR := $(KERNEL_SRC)
PWD := $(shell pwd)

# VideoBuf2 module directories - use absolute paths
VB2_CXL_DIR := $(abspath ../videobuf2-cxl)
VB2_NVME_CMB_DIR := $(abspath ../videobuf2-nvme-cmb)

# Check if modules exist
VB2_CXL_EXISTS := $(wildcard $(VB2_CXL_DIR)/Makefile)
VB2_NVME_CMB_EXISTS := $(wildcard $(VB2_NVME_CMB_DIR)/Makefile)

# Module symbol files
CXL_SYMVERS := $(VB2_CXL_DIR)/Module.symvers
NVME_SYMVERS := $(VB2_NVME_CMB_DIR)/Module.symvers

# Build mode compiler flags
ifeq ($(POLLING_MODE), 1)
    ccflags-y += -DCSI2_POLLING_MODE=1
    BUILD_MODE_STR := polling
else
    ccflags-y += -DCSI2_POLLING_MODE=0
    BUILD_MODE_STR := interrupt
endif

# Include paths for videobuf2 modules
ifneq ($(VB2_CXL_EXISTS),)
    ccflags-y += -I$(VB2_CXL_DIR)
endif

ifneq ($(VB2_NVME_CMB_EXISTS),)
    ccflags-y += -I$(VB2_NVME_CMB_DIR)
endif

# Check if we're in QEMU guest
IS_QEMU := $(shell if dmesg 2>/dev/null | grep -qi "qemu\|kvm"; then echo "yes"; else echo "no"; fi)

# Default target
all: check-env build-cxl build-nvme-cmb build-csi2

# Environment check
check-env:
	@echo "================================"
	@echo "AMD CSI2 V4L2 Driver Build"
	@echo "================================"
	@echo "Build mode: $(BUILD_MODE_STR)"
	@echo "Kernel source: $(KERNEL_SRC)"
	@echo "Kernel directory: $(KDIR)"
	@echo "Build directory: $(PWD)"
	@echo "Target architecture: $(shell uname -m)"
	@echo "Running kernel: $(shell uname -r)"
	@echo "QEMU Guest: $(IS_QEMU)"
	@if [ -d "$(VB2_CXL_DIR)" ]; then \
		echo "VideoBuf2-CXL: Found at $(VB2_CXL_DIR)"; \
	else \
		echo "VideoBuf2-CXL: Not found - REQUIRED!"; \
		echo "  Expected location: $(VB2_CXL_DIR)"; \
	fi
	@if [ -d "$(VB2_NVME_CMB_DIR)" ]; then \
		echo "VideoBuf2-NVMe-CMB: Found at $(VB2_NVME_CMB_DIR)"; \
	else \
		echo "VideoBuf2-NVMe-CMB: Not found"; \
		echo "  Expected location: $(VB2_NVME_CMB_DIR)"; \
	fi
	@echo "================================"
	@if [ ! -d "$(KERNEL_SRC)" ]; then \
		echo "❌ ERROR: Kernel source not found at $(KERNEL_SRC)"; \
		exit 1; \
	else \
		echo "✅ Kernel source found"; \
		if [ -f "$(KERNEL_SRC)/.config" ]; then \
			echo "✅ Kernel appears to be configured"; \
		else \
			echo "⚠️  Warning: Kernel may not be configured"; \
		fi; \
	fi

# Build VideoBuf2-CXL
build-cxl:
	@if [ -d "$(VB2_CXL_DIR)" ] && [ -f "$(VB2_CXL_DIR)/Makefile" ]; then \
		echo "================================"; \
		echo "Step 1: Building VideoBuf2-CXL module..."; \
		echo "================================"; \
		$(MAKE) -C $(VB2_CXL_DIR) KDIR=$(KDIR) all || exit 1; \
		echo "✅ VideoBuf2-CXL built successfully"; \
		echo ""; \
	else \
		echo "❌ ERROR: VideoBuf2-CXL directory not found"; \
		exit 1; \
	fi

# Build VideoBuf2-NVMe-CMB
build-nvme-cmb:
	@if [ -d "$(VB2_NVME_CMB_DIR)" ] && [ -f "$(VB2_NVME_CMB_DIR)/Makefile" ]; then \
		echo "================================"; \
		echo "Step 2: Building VideoBuf2-NVMe-CMB module..."; \
		echo "================================"; \
		$(MAKE) -C $(VB2_NVME_CMB_DIR) KDIR=$(KDIR) all || exit 1; \
		echo "✅ VideoBuf2-NVMe-CMB built successfully"; \
		echo ""; \
	fi

# Build AMD CSI2 module
build-csi2:
	@echo "================================"
	@echo "Step 3: Building AMD CSI2 V4L2 module..."
	@echo "================================"
	@# Check which symbol files exist
	@SYMVERS_FILES=""; \
	if [ -f "$(CXL_SYMVERS)" ]; then \
		echo "  Using CXL symbols from $(CXL_SYMVERS)"; \
		SYMVERS_FILES="$$SYMVERS_FILES $(CXL_SYMVERS)"; \
	fi; \
	if [ -f "$(NVME_SYMVERS)" ]; then \
		echo "  Using NVMe-CMB symbols from $(NVME_SYMVERS)"; \
		SYMVERS_FILES="$$SYMVERS_FILES $(NVME_SYMVERS)"; \
	fi; \
	if [ -n "$$SYMVERS_FILES" ]; then \
		echo "  Building with external symbols..."; \
		$(MAKE) -C $(KDIR) M=$(PWD) modules KBUILD_EXTRA_SYMBOLS="$$SYMVERS_FILES"; \
	else \
		echo "  Building without external symbols..."; \
		$(MAKE) -C $(KDIR) M=$(PWD) modules; \
	fi
	@echo "✅ AMD CSI2 V4L2 module built successfully"

# Clean all modules
clean:
	@if [ -d "$(VB2_CXL_DIR)" ]; then \
		echo "Cleaning VideoBuf2-CXL module..."; \
		$(MAKE) -C $(VB2_CXL_DIR) KDIR=$(KDIR) clean; \
	fi
	@if [ -d "$(VB2_NVME_CMB_DIR)" ]; then \
		echo "Cleaning VideoBuf2-NVMe-CMB module..."; \
		$(MAKE) -C $(VB2_NVME_CMB_DIR) KDIR=$(KDIR) clean; \
	fi
	@if [ -d "$(KDIR)" ]; then \
		$(MAKE) -C $(KDIR) M=$(PWD) clean; \
	fi
	rm -f Module.symvers modules.order
	rm -rf ./modules
	rm -f *.o *.mod.c *.mod *.cmd .*.cmd
	rm -rf .tmp_versions
	@echo "Build directory cleaned"

# Install modules
install: all
	@echo "Installing modules..."
	@mkdir -p ./modules
	@cp *.ko ./modules/ 2>/dev/null || echo "No .ko files to copy"
	@if [ -f "$(VB2_CXL_DIR)/videobuf2-cxl.ko" ]; then \
		echo "Installing VideoBuf2-CXL module..."; \
		cp $(VB2_CXL_DIR)/videobuf2-cxl.ko ./modules/; \
	fi
	@if [ -f "$(VB2_NVME_CMB_DIR)/videobuf2-nvme-cmb.ko" ]; then \
		echo "Installing VideoBuf2-NVMe-CMB module..."; \
		cp $(VB2_NVME_CMB_DIR)/videobuf2-nvme-cmb.ko ./modules/; \
	fi
	@echo "Modules ready in ./modules/"
	@ls -la ./modules/*.ko

# Load V4L2 dependencies
load-deps:
	@echo "🔧 Loading required V4L2 dependencies..."
	@for mod in videodev videobuf2-core videobuf2-v4l2 videobuf2-dma-contig videobuf2-memops; do \
		if lsmod | grep -q "^$$mod "; then \
			echo "✅ $$mod already loaded"; \
		else \
			if sudo modprobe $$mod 2>/dev/null; then \
				echo "✅ $$mod loaded via modprobe"; \
			else \
				echo "⚠️  $$mod not available (may be built-in)"; \
			fi; \
		fi; \
	done

# Load VideoBuf2-CXL
load-vb2-cxl:
	@echo "🔧 Loading VideoBuf2-CXL module..."
	@if [ -f "./modules/videobuf2-cxl.ko" ]; then \
		if lsmod | grep -q "^videobuf2_cxl "; then \
			echo "✅ videobuf2_cxl already loaded"; \
		else \
			echo "📦 Loading videobuf2-cxl.ko..."; \
			if sudo insmod ./modules/videobuf2-cxl.ko; then \
				echo "✅ Successfully loaded videobuf2-cxl"; \
				sleep 1; \
			else \
				echo "❌ Failed to load videobuf2-cxl"; \
				dmesg | tail -10; \
				exit 1; \
			fi; \
		fi; \
	else \
		echo "❌ ERROR: VideoBuf2-CXL module not found!"; \
		exit 1; \
	fi

# Load VideoBuf2-NVMe-CMB
load-vb2-nvme-cmb:
	@echo "🔧 Loading VideoBuf2-NVMe-CMB module..."
	@if [ -f "./modules/videobuf2-nvme-cmb.ko" ]; then \
		if lsmod | grep -q "^videobuf2_nvme_cmb "; then \
			echo "✅ videobuf2_nvme_cmb already loaded"; \
		else \
			echo "📦 Loading videobuf2-nvme-cmb.ko..."; \
			if sudo insmod ./modules/videobuf2-nvme-cmb.ko; then \
				echo "✅ Successfully loaded videobuf2-nvme-cmb"; \
				sleep 1; \
			else \
				echo "⚠️  Failed to load videobuf2-nvme-cmb (continuing)"; \
			fi; \
		fi; \
	fi

# Load all modules
load: load-params
	@echo "Loading AMD CSI2 V4L2 driver ($(BUILD_MODE_STR) mode)..."
	@if [ ! -f "./modules/amd_csi2_v4l2.ko" ]; then \
		echo "Modules not found, building first..."; \
		$(MAKE) install; \
	fi
	@$(MAKE) load-deps
	@$(MAKE) load-vb2-cxl
	@$(MAKE) load-vb2-nvme-cmb
	@if lsmod | grep -q "^amd_csi2_v4l2 "; then \
		echo "Unloading existing amd_csi2_v4l2..."; \
		sudo rmmod amd_csi2_v4l2 || true; \
		sleep 1; \
	fi
	@echo "📦 Loading amd_csi2_v4l2.ko..."
	@MODULE_PARAMS=""; \
	if [ "$(USE_CXL)" = "1" ] || [ "$(USE_CXL)" = "" ]; then \
		MODULE_PARAMS="$$MODULE_PARAMS use_cxl_memory=1"; \
	fi; \
	if [ "$(USE_NVME)" = "1" ]; then \
		MODULE_PARAMS="$$MODULE_PARAMS use_nvme_cmb=1"; \
	fi; \
	if [ -n "$(DEBUG)" ]; then \
		MODULE_PARAMS="$$MODULE_PARAMS debug_level=$(DEBUG)"; \
	fi; \
	echo "  Parameters: $$MODULE_PARAMS"; \
	if sudo insmod ./modules/amd_csi2_v4l2.ko $$MODULE_PARAMS; then \
		echo "✅ Driver loaded successfully"; \
		echo ""; \
		lsmod | grep -E "(amd_csi2|videobuf2_cxl|videobuf2_nvme)"; \
		echo ""; \
		ls -la /dev/video* 2>/dev/null || echo "No video devices yet"; \
		echo ""; \
		dmesg | grep -E "(CSI2|CXL|NVMe)" | tail -10; \
	else \
		echo "❌ Driver load failed"; \
		dmesg | tail -20; \
		exit 1; \
	fi

# Load parameters help
load-params:
	@echo "Module parameters:"
	@echo "  USE_CXL=$(USE_CXL) (default: 1)"
	@echo "  USE_NVME=$(USE_NVME) (default: 0)"
	@echo "  DEBUG=$(DEBUG) (default: 3)"
	@echo ""

# Unload all modules
unload:
	@echo "Unloading AMD CSI2 V4L2 driver..."
	@if lsmod | grep -q "^amd_csi2_v4l2 "; then \
		sudo rmmod amd_csi2_v4l2; \
		echo "✅ AMD CSI2 driver unloaded"; \
	fi
	@if lsmod | grep -q "^videobuf2_nvme_cmb "; then \
		sudo rmmod videobuf2-nvme-cmb; \
		echo "✅ VideoBuf2-NVMe-CMB unloaded"; \
	fi
	@if lsmod | grep -q "^videobuf2_cxl "; then \
		sudo rmmod videobuf2-cxl; \
		echo "✅ VideoBuf2-CXL unloaded"; \
	fi

# Reload modules
reload: unload load

# Status check
status:
	@echo "================================"
	@echo "AMD CSI2 Driver Status"
	@echo "================================"
	@echo "Build mode: $(BUILD_MODE_STR)"
	@echo "Kernel source: $(KERNEL_SRC)"
	@echo "QEMU Guest: $(IS_QEMU)"
	@echo ""
	@echo "📋 Module Status:"
	@lsmod | grep -E "(amd_csi2|videobuf2_cxl|videobuf2_nvme)" || echo "  No modules loaded"
	@echo ""
	@echo "🔌 PCI Devices:"
	@echo "  CSI2 Device:"
	@lspci -d 10ee:9024 2>/dev/null | head -1 || echo "    Not found"
	@echo "  CXL Device:"
	@lspci | grep -i "CXL\|0d93" | head -1 || echo "    Not found"
	@echo "  NVMe Device:"
	@lspci | grep -i "NVMe" | head -1 || echo "    Not found"
	@echo ""
	@echo "📹 Video Devices:"
	@ls -la /dev/video* 2>/dev/null || echo "  No video devices found"
	@echo ""
	@echo "🔧 Memory Status:"
	@if [ -d "/sys/class/vb2_cxl/" ]; then \
		echo "  CXL Pools:"; \
		for pool in /sys/class/vb2_cxl/cxl_pool_*; do \
			if [ -e "$$pool/stats" ]; then \
				echo "    $$(basename $$pool):"; \
				cat $$pool/stats | sed 's/^/      /'; \
			fi; \
		done; \
	else \
		echo "  CXL pools not available"; \
	fi

# Test CXL functionality
test-cxl:
	@echo "Testing CXL functionality..."
	@echo "=== CXL Device Check ==="
	@lspci -vvv -d 8086:0d93 2>/dev/null || echo "Intel CXL device not found"
	@echo ""
	@echo "=== Module Parameters ==="
	@if [ -d "/sys/module/amd_csi2_v4l2/parameters/" ]; then \
		for param in /sys/module/amd_csi2_v4l2/parameters/*; do \
			if [ -f "$$param" ]; then \
				echo "$$(basename $$param): $$(cat $$param 2>/dev/null)"; \
			fi; \
		done; \
	else \
		echo "Module not loaded"; \
	fi

# Test video capture
test-capture:
	@echo "Testing video capture..."
	@if [ -e "/dev/video0" ]; then \
		v4l2-ctl -d /dev/video0 --all; \
		echo ""; \
		echo "Attempting capture test (5 frames)..."; \
		v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=5 --stream-to=/dev/null; \
	else \
		echo "No video device found"; \
	fi

# Performance test
perf-test:
	@echo "Running performance test..."
	@if [ -e "/dev/video0" ]; then \
		echo "Capturing 300 frames (10 seconds at 30fps)..."; \
		time v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=300 --stream-to=/dev/null; \
	else \
		echo "No video device found"; \
	fi

# Show kernel messages
dmesg:
	@echo "=== Recent kernel messages ==="
	@dmesg | grep -E "(CSI2|CXL|NVMe|videobuf|P2P)" | tail -30

# Full sequence
full: clean all install load
	@echo "✅ Full build and load completed"

# Help
help:
	@echo "AMD CSI2 V4L2 Driver Makefile"
	@echo "============================="
	@echo ""
	@echo "Build targets:"
	@echo "  all              - Build all modules"
	@echo "  clean            - Clean all build files"
	@echo "  install          - Copy modules to ./modules/"
	@echo ""
	@echo "Module management:"
	@echo "  load             - Load all modules"
	@echo "  unload           - Unload all modules"
	@echo "  reload           - Reload all modules"
	@echo "  full             - Clean, build, install and load"
	@echo ""
	@echo "Parameters for 'make load':"
	@echo "  USE_CXL=1        - Enable CXL memory (default: 1)"
	@echo "  USE_NVME=1       - Enable NVMe CMB (default: 0)"
	@echo "  DEBUG=<0-4>      - Set debug level (default: 3)"
	@echo ""
	@echo "Testing:"
	@echo "  status           - Show driver status"
	@echo "  test-cxl         - Test CXL functionality"
	@echo "  test-capture     - Test video capture"
	@echo "  perf-test        - Performance test"
	@echo "  dmesg            - Show kernel messages"
	@echo ""
	@echo "Example: make load USE_CXL=1 USE_NVME=1 DEBUG=4"

.PHONY: all clean install load unload reload status help full \
        check-env build-cxl build-nvme-cmb build-csi2 \
        load-deps load-vb2-cxl load-vb2-nvme-cmb load-params \
        test-cxl test-capture perf-test dmesg
