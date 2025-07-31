# AMD CSI2 V4L2 Driver Makefile

obj-m += amd_csi2_v4l2.o

# Build mode selection - NEW FEATURE
POLLING_MODE ?= 0

# Build mode compiler flags - NEW
ifeq ($(POLLING_MODE), 1)
    ccflags-y += -DCSI2_POLLING_MODE=1
    BUILD_MODE_STR := polling
else
    ccflags-y += -DCSI2_POLLING_MODE=0
    BUILD_MODE_STR := interrupt
endif

# QEMU Environment Detection and Kernel Directory Setup
# Check if we're in QEMU with 9p filesystem (host development)
KERNEL_SRC_HOST := ../../generic/linux-6.15.4
KERNEL_SRC_GUEST := /lib/modules/$(shell uname -r)/build

# Determine which kernel source to use
ifneq ($(wildcard $(KERNEL_SRC_HOST)),)
    # Host development environment (9p shared filesystem)
    KDIR := $(KERNEL_SRC_HOST)
    $(info Building with host kernel source: $(KDIR))
else ifneq ($(wildcard $(KERNEL_SRC_GUEST)),)
    # Guest environment with installed headers
    KDIR := $(KERNEL_SRC_GUEST)
    $(info Building with guest kernel headers: $(KDIR))
else
    # Fallback
    $(error Neither host kernel source nor guest headers found!)
endif

# Build directory
PWD := $(shell pwd)

# Default target - now with environment detection
all: check-env
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# NEW: Polling mode target
polling:
	@echo "Building AMD CSI2 V4L2 Driver in polling mode..."
	$(MAKE) POLLING_MODE=1 all

# NEW: Interrupt mode target (explicit)
interrupt:
	@echo "Building AMD CSI2 V4L2 Driver in interrupt mode..."
	$(MAKE) POLLING_MODE=0 all

# Environment check target - MODIFIED
check-env:
	@echo "================================"
	@echo "AMD CSI2 V4L2 Driver Build"
	@echo "================================"
	@echo "Build mode: $(BUILD_MODE_STR)"
	@echo "Kernel source directory: $(KDIR)"
	@echo "Build directory: $(PWD)"
	@echo "Target architecture: $(shell uname -m)"
	@echo "Kernel version: $(shell uname -r)"
	@echo "================================"
	@if [ ! -d "$(KDIR)" ]; then \
		echo "ERROR: Kernel source directory not found!"; \
		echo "Expected: $(KDIR)"; \
		exit 1; \
	fi
	@if [ ! -f "$(KDIR)/Makefile" ]; then \
		echo "ERROR: Kernel Makefile not found in $(KDIR)"; \
		exit 1; \
	fi

# Clean target - enhanced
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f Module.symvers modules.order
	rm -rf ./modules
	rm -f *.o *.mod.c *.mod *.cmd .*.cmd
	rm -rf .tmp_versions
	@echo "Build directory cleaned"

# Install modprobe configuration
install-config:
	@echo "📋 Installing modprobe configuration..."
	@if [ -f "amd_csi2.conf" ]; then \
		sudo cp amd_csi2.conf /etc/modprobe.d/amd_csi2.conf; \
		echo "✅ Configuration installed to /etc/modprobe.d/amd_csi2.conf"; \
	else \
		echo "⚠️  Configuration file not found, creating temporary one..."; \
		echo "# AMD CSI2 V4L2 Driver Dependencies" > /tmp/amd_csi2.conf; \
		echo "softdep amd_csi2_v4l2 pre: videodev videobuf2-core videobuf2-v4l2 videobuf2-dma-contig" >> /tmp/amd_csi2.conf; \
		sudo cp /tmp/amd_csi2.conf /etc/modprobe.d/amd_csi2.conf; \
		rm /tmp/amd_csi2.conf; \
		echo "✅ Temporary configuration installed"; \
	fi

# Remove modprobe configuration
remove-config:
	@echo "🗑️  Removing modprobe configuration..."
	@sudo rm -f /etc/modprobe.d/amd_csi2.conf
	@echo "✅ Configuration removed"

# Check kernel config for V4L2 support
check-kernel-config:
	@echo "🔍 Checking Kernel V4L2 Configuration"
	@echo "===================================="
	@if [ -f /proc/config.gz ]; then \
		echo "📋 V4L2 related config options:"; \
		zcat /proc/config.gz | grep -E "CONFIG_(VIDEO|V4L2)" | head -10; \
	elif [ -f /boot/config-$(shell uname -r) ]; then \
		echo "📋 V4L2 related config options:"; \
		grep -E "CONFIG_(VIDEO|V4L2)" /boot/config-$(shell uname -r) | head -10; \
	else \
		echo "❌ Kernel config not available"; \
	fi

# Try loading without dependencies (for testing)
load-no-deps:
	@echo "🚀 Loading driver without dependencies (test mode)..."
	@if [ ! -f "./modules/amd_csi2_v4l2.ko" ]; then \
		echo "Module not found, building first..."; \
		$(MAKE) install-manual; \
	fi
	@if lsmod | grep -q amd_csi2_v4l2; then \
		echo "Driver already loaded, unloading first..."; \
		sudo rmmod amd_csi2_v4l2 || true; \
		sleep 1; \
	fi
	@echo "📦 Loading driver module (ignoring dependencies)..."
	@if sudo insmod ./modules/amd_csi2_v4l2.ko; then \
		echo "✅ Driver loaded successfully without dependencies"; \
		echo "   This suggests the dependencies might be built-in"; \
	else \
		echo "❌ Driver load failed even without explicit dependency loading"; \
		dmesg | tail -10; \
	fi

# Force load with dependency resolution
force-load:
	@echo "🚀 Force loading with all methods..."
	@$(MAKE) debug-symbols
	@$(MAKE) force-load-deps
	@echo ""
	@echo "📦 Attempting driver load..."
	@if sudo insmod ./modules/amd_csi2_v4l2.ko; then \
		echo "✅ Success!"; \
	else \
		echo "❌ Failed - trying alternative approach..."; \
		$(MAKE) load-no-deps; \
	fi

# Reload module (unload + load)
reload: unload load

# Show module info
info:
	modinfo amd_csi2_v4l2.ko

# Show device status - enhanced for QEMU - MODIFIED
status:
	@echo "================================"
	@echo "AMD CSI2 Driver Status"
	@echo "================================"
	@echo "Build mode: $(BUILD_MODE_STR)"
	@echo "📋 Module Status:"
	@if lsmod | grep -q amd_csi2; then \
		lsmod | grep amd_csi2; \
	else \
		echo "  Driver not loaded"; \
	fi
	@echo ""
	@echo "🔌 PCI Device Status:"
	@if lspci | grep -q "AMD.*c901"; then \
		lspci | grep "AMD.*c901"; \
		echo "  Device found - OK"; \
	else \
		echo "  AMD CSI2 device not found"; \
		echo "  All PCI devices:"; \
		lspci | grep -i multimedia || echo "  No multimedia devices found"; \
	fi
	@echo ""
	@echo "📹 Video Devices:"
	@ls -la /dev/video* 2>/dev/null || echo "  No video devices found"
	@echo ""
	@echo "📊 Recent Kernel Messages:"
	@dmesg | grep -i csi2 | tail -5 || echo "  No CSI2 messages in dmesg"
	@echo ""
	@echo "🔧 Build Files:"
	@ls -la *.ko ./modules/*.ko 2>/dev/null || echo "  No module files found"

# Debug target for troubleshooting
debug:
	@echo "================================"
	@echo "AMD CSI2 Debug Information"
	@echo "================================"
	@echo "🏗️  Build Environment:"
	@echo "  Build mode: $(BUILD_MODE_STR)"
	@echo "  Kernel source: $(KDIR)"
	@echo "  PWD: $(PWD)"
	@echo "  Architecture: $(shell uname -m)"
	@echo "  Kernel version: $(shell uname -r)"
	@echo ""
	@echo "📁 File System Info:"
	@mount | grep -E "(9p|virtio)" || echo "  No 9p/virtio mounts found"
	@echo ""
	@echo "🔍 PCI Detailed Info:"
	@lspci -vvv | grep -A 20 "AMD.*c901" || echo "  AMD CSI2 device not found"
	@echo ""
	@echo "📋 All Loaded Modules:"
	@lsmod | head -10
	@echo ""
	@echo "⚠️  System Messages (last 10):"
	@dmesg | tail -10

# Test PCI device presence
test-pci:
	@echo "Testing PCI device presence..."
	@if lspci -d 1022:c901 >/dev/null 2>&1; then \
		echo "✅ AMD CSI2 device found"; \
		lspci -d 1022:c901 -vvv; \
	else \
		echo "❌ AMD CSI2 device not found"; \
		echo "Available PCI devices:"; \
		lspci; \
		exit 1; \
	fi

# Install target - adapted for QEMU environment
install: all
	@echo "Installing AMD CSI2 V4L2 driver..."
	@if [ -w /lib/modules/$(shell uname -r)/extra ]; then \
		sudo $(MAKE) -C $(KDIR) M=$(PWD) modules_install; \
		sudo depmod -a; \
		echo "Driver installed to system"; \
	else \
		echo "Creating local module directory..."; \
		mkdir -p ./modules; \
		cp *.ko ./modules/; \
		echo "Driver copied to ./modules/ (manual insmod required)"; \
		echo "Use: sudo insmod ./modules/amd_csi2_v4l2.ko"; \
	fi

# Manual install for QEMU environments
install-manual: all
	@echo "Manual installation for QEMU environment..."
	mkdir -p ./modules
	cp *.ko ./modules/
	@echo "Module ready: ./modules/amd_csi2_v4l2.ko"
	@echo "To load: sudo insmod ./modules/amd_csi2_v4l2.ko"
	@echo "To unload: sudo rmmod amd_csi2_v4l2"

# Uninstall target
uninstall:
	sudo rm -f /lib/modules/$(shell uname -r)/extra/amd_csi2_v4l2.ko
	sudo depmod -a

# Helper target to try loading a single module with multiple methods
try-load-module:
	@echo "🔍 Trying to load $(MODULE)..."
	@if lsmod | grep -q "^$(MODULE) "; then \
		echo "✅ $(MODULE) already loaded"; \
	elif modprobe $(MODULE) 2>/dev/null; then \
		echo "✅ $(MODULE) loaded via modprobe"; \
	elif find /lib/modules/$(shell uname -r) -name "$(MODULE).ko*" 2>/dev/null | head -1 | xargs -r -I {} sh -c 'echo "Trying insmod {}" && insmod {} 2>/dev/null'; then \
		echo "✅ $(MODULE) loaded via insmod"; \
	else \
		echo "⚠️  $(MODULE) not found or failed to load"; \
		return 1; \
	fi

# Enhanced dependency loading with fallback options
load-deps:
	@echo "🔧 Loading required V4L2 dependencies..."
	@echo "📍 Checking module availability..."
	@# Check if modules exist in the system
	@find /lib/modules/$(shell uname -r) -name "*video*" -type f 2>/dev/null | head -5 || echo "No video modules found in standard location"
	@echo ""
	@echo "🔄 Attempting to load dependencies with fallbacks..."
	@# Try loading essential V4L2 modules
	@for mod in videodev videobuf2-core videobuf2-v4l2 videobuf2-dma-contig; do \
		$(MAKE) try-load-module MODULE=$$mod || echo "⚠️  $$mod not available (continuing)"; \
	done
	@echo ""
	@echo "📋 Final dependency status:"
	@lsmod | grep -E "(videodev|videobuf)" || echo "⚠️  No V4L2 modules loaded (driver may still work)"
	@echo "✅ Dependency loading completed"

# Enhanced load target with better error handling - MODIFIED
load:
	@echo "Loading AMD CSI2 V4L2 driver ($(BUILD_MODE_STR) mode)..."
	@if [ ! -f "./modules/amd_csi2_v4l2.ko" ]; then \
		echo "Module not found, building first..."; \
		$(MAKE) install-manual; \
	fi
	@if lsmod | grep -q amd_csi2_v4l2; then \
		echo "Driver already loaded, unloading first..."; \
		sudo rmmod amd_csi2_v4l2 || true; \
		sleep 1; \
	fi
	@# Load dependencies with improved error handling
	@$(MAKE) load-deps
	@echo ""
	@echo "📦 Loading driver module..."
	@if sudo insmod ./modules/amd_csi2_v4l2.ko; then \
		echo "✅ Driver loaded successfully in $(BUILD_MODE_STR) mode"; \
		echo "   Check with: lsmod | grep amd_csi2"; \
		echo "   Device: ls -la /dev/video*"; \
		dmesg | tail -5 | grep -i csi2 || echo "   No CSI2 messages in dmesg yet"; \
	else \
		echo "❌ Driver load failed"; \
		echo "📋 Recent kernel messages:"; \
		dmesg | tail -10; \
		echo ""; \
		echo "🔍 Detailed error analysis:"; \
		$(MAKE) debug-load-failure; \
		exit 1; \
	fi

# New target for analyzing load failures
debug-load-failure:
	@echo "🔍 Analyzing driver load failure..."
	@echo "=================================="
	@echo "📋 Missing symbols analysis:"
	@if dmesg | tail -20 | grep -i "unknown symbol"; then \
		echo "Found unknown symbol errors:"; \
		dmesg | tail -20 | grep -i "unknown symbol"; \
	else \
		echo "No unknown symbol errors found"; \
	fi
	@echo ""
	@echo "📦 Available V4L2 symbols:"
	@if [ -d /proc/modules ]; then \
		for mod in videodev videobuf2-core videobuf2-v4l2; do \
			if lsmod | grep -q $$mod; then \
				echo "✅ $$mod is loaded"; \
			else \
				echo "❌ $$mod is NOT loaded"; \
			fi; \
		done; \
	fi
	@echo ""
	@echo "🔧 Suggested fixes:"
	@echo "1. Try: make force-load-deps"
	@echo "2. Try: make check-kernel-config"
	@echo "3. Check: dmesg | grep -i v4l"

# Force load dependencies from built-in modules
force-load-deps:
	@echo "🚀 Force loading V4L2 dependencies..."
	@echo "Trying built-in kernel modules..."
	@# Try to find and load modules from different locations
	@for mod in videodev videobuf2-core videobuf2-v4l2 videobuf2-dma-contig; do \
		echo "Force loading $$mod..."; \
		if find /lib/modules/$(shell uname -r) -name "$$mod.ko*" -exec insmod {} \; 2>/dev/null; then \
			echo "✅ $$mod loaded"; \
		elif modprobe $$mod 2>/dev/null; then \
			echo "✅ $$mod loaded via modprobe"; \
		else \
			echo "⚠️  $$mod not available"; \
		fi; \
	done
	@echo "✅ Force loading completed"

# Unload module
unload:
	@echo "Unloading AMD CSI2 V4L2 driver..."
	@if lsmod | grep -q amd_csi2_v4l2; then \
		sudo rmmod amd_csi2_v4l2; \
		echo "Driver unloaded successfully"; \
	else \
		echo "Driver not currently loaded"; \
	fi

# Debug missing symbols - enhanced
debug-symbols:
	@echo "🔍 Debugging Missing Symbols"
	@echo "=========================="
	@echo "📋 Current V4L2 modules status:"
	@lsmod | grep -E "(videodev|videobuf)" || echo "  No V4L2 modules loaded"
	@echo ""
	@echo "📦 Checking available modules in kernel:"
	@find /lib/modules/$(shell uname -r) -name "*video*" -o -name "*v4l*" | grep -E '\.ko' | sort || echo "  No V4L2 modules found"
	@echo ""
	@echo "🔧 Attempting to load V4L2 modules..."
	@$(MAKE) force-load-deps
	@echo ""
	@echo "📋 Final V4L2 modules status:"
	@lsmod | grep -E "(videodev|videobuf)" || echo "  Still no V4L2 modules loaded"
	@echo ""
	@echo "🔍 Kernel V4L2 support check:"
	@if grep -q "CONFIG_VIDEO_DEV=y" /proc/config.gz 2>/dev/null || grep -q "CONFIG_VIDEO_DEV=y" /boot/config-$(shell uname -r) 2>/dev/null; then \
		echo "✅ V4L2 support is built-in"; \
	elif grep -q "CONFIG_VIDEO_DEV=m" /proc/config.gz 2>/dev/null || grep -q "CONFIG_VIDEO_DEV=m" /boot/config-$(shell uname -r) 2>/dev/null; then \
		echo "✅ V4L2 support available as modules"; \
	else \
		echo "❌ V4L2 support may not be available"; \
	fi

.PHONY: all polling interrupt clean install install-manual install-config remove-config uninstall load load-deps load-no-deps unload reload info status debug check-env test-pci debug-symbols force-load force-load-deps check-kernel-config debug-load-failure try-load-module

# Help target - MODIFIED
help:
	@echo "AMD CSI2 V4L2 Driver Makefile"
	@echo "============================="
	@echo ""
	@echo "Build modes:"
	@echo "  all              - Build in interrupt mode (default)"
	@echo "  polling          - Build in polling mode"
	@echo "  interrupt        - Build in interrupt mode (explicit)"
	@echo ""
	@echo "Available targets:"
	@echo "  clean            - Clean build files"
	@echo "  install          - Install driver (automatic detection)"
	@echo "  install-manual   - Manual install for QEMU"
	@echo "  load             - Load the driver module (with dependencies)"
	@echo "  load-no-deps     - Load driver without explicit dependencies"
	@echo "  force-load       - Force load with all available methods"
	@echo "  unload           - Unload the driver module"
	@echo "  reload           - Unload and load the driver"
	@echo "  status           - Show driver and device status"
	@echo "  debug            - Show detailed debug information"
	@echo "  debug-symbols    - Debug V4L2 dependency issues"
	@echo "  test-pci         - Test PCI device presence"
	@echo "  info             - Show module information"
	@echo "  help             - Show this help"
	@echo ""
	@echo "Build modes:"
	@echo "  Interrupt mode: Uses MSI-X/legacy interrupts (default)"
	@echo "  Polling mode:   Uses timer-based polling for debugging"
	@echo ""
	@echo "QEMU Development Workflow:"
	@echo "  1. make clean"
	@echo "  2. make all          (or make polling for polling mode)"
	@echo "  3. make load         (or make load-no-deps if deps fail)"
	@echo "  4. make status"
	@echo ""
	@echo "Troubleshooting:"
	@echo "  make debug-symbols   - Analyze dependency issues"
	@echo "  make force-load      - Try all loading methods"
	@echo "  make check-kernel-config - Check V4L2 kernel support"
	@echo ""
	@echo "Examples:"
	@echo "  make polling load    - Build in polling mode and load"
	@echo "  make interrupt load  - Build in interrupt mode and load"
