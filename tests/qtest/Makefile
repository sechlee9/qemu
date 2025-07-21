# Kernel module Makefile for qemu_csi2_v4l2 driver
obj-m := qemu_csi2_v4l2.o

# Kernel source directory
KERNEL_SRC ?= ../../../generic/linux-6.15.4

# Build flags
ccflags-y := -Wall -Wextra

all:
	make -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean

install:
	sudo insmod qemu_csi2_v4l2.ko

uninstall:
	sudo rmmod qemu_csi2_v4l2

load:
	@echo "Loading V4L2 dependencies..."
	sudo modprobe videobuf2-vmalloc
	sudo modprobe videobuf2-v4l2
	sudo modprobe videodev
	@echo "Loading qemu_csi2_v4l2 module..."
	sudo insmod qemu_csi2_v4l2.ko

unload:
	@echo "Unloading module..."
	sudo rmmod qemu_csi2_v4l2 || true

# 중복된 reload 타겟 수정
reload: unload
	sleep 2
	make load

force-unload:
	sudo fuser -k /dev/video0 2>/dev/null || true
	sudo pkill -f v4l2 || true
	sleep 1
	sudo rmmod -f qemu_csi2_v4l2 || true

# 추가 유용한 타겟들
status:
	@echo "=== Module Status ==="
	@if lsmod | grep -q qemu_csi2_v4l2; then \
		echo "✅ Module is loaded"; \
		lsmod | grep qemu_csi2_v4l2; \
	else \
		echo "❌ Module is not loaded"; \
	fi
	@echo ""
	@echo "=== V4L2 Devices ==="
	@ls -la /dev/video* 2>/dev/null || echo "No video devices found"
	@echo ""
	@echo "=== Recent Kernel Messages ==="
	@dmesg | grep -i csi2 | tail -10 || echo "No CSI2 messages found"

debug:
	@echo "=== Debug Information ==="
	@echo "Kernel source: $(KERNEL_SRC)"
	@echo "Module file: qemu_csi2_v4l2.ko"
	@echo "Source file: qemu_csi2_v4l2.c"
	@if [ -f qemu_csi2_v4l2.ko ]; then \
		echo "✅ Module built"; \
		modinfo qemu_csi2_v4l2.ko | head -10; \
	else \
		echo "❌ Module not built"; \
	fi

monitor:
	@echo "Monitoring kernel logs for CSI2 messages..."
	@echo "Press Ctrl+C to stop"
	sudo dmesg -w | grep --line-buffered -i csi2

.PHONY: all clean install uninstall load unload reload force-unload status debug monitor
