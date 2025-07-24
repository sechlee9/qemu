# Makefile for QEMU CSI2 V4L2 Driver
# Linux Kernel 6.15.4

# 커널 소스 경로 - 여기를 수정하세요
KERNEL_SRC := ../../../generic/linux-6.15.4

# 드라이버 모듈 이름
MODULE_NAME := qemu_csi2_v4l2
obj-m := $(MODULE_NAME).o

# 소스 파일들 - circular dependency 방지를 위해 다른 이름 사용
# vm_v4l2_driver.c -> vm_v4l2_driver.o -> vm_v4l2_driver.ko
# 별도의 object 파일명 지정 불필요

# 컴파일 플래그
ccflags-y := -Wall -Wextra -g
ccflags-y += -DDEBUG

# 크로스 컴파일 설정 (필요시)
# ARCH := x86_64
# CROSS_COMPILE := 

# 기본 타겟
all: modules

# 모듈 빌드
modules:
	@echo "Building QEMU CSI2 V4L2 Driver..."
	@if [ ! -d "$(KERNEL_SRC)" ]; then \
		echo "Error: Kernel source directory not found: $(KERNEL_SRC)"; \
		echo "Please edit KERNEL_SRC in Makefile"; \
		exit 1; \
	fi
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

# 모듈 정리
clean:
	@echo "Cleaning..."
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
	rm -f modules.order Module.symvers *.mod.c *.o *.ko
	rm -f .*.cmd *.mod

# 모듈 설치 (root 권한 필요)
install: 
	@echo "Installing module..."
	sudo $(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install
	sudo depmod -a

# 모듈 로드 (root 권한 필요)
load: 
	@echo "Loading module..."
	sudo modprobe videodev || true
	sudo modprobe mc || true
	#sudo modprobe videobuf2-core || true
	sudo modprobe videobuf2-memops || true
	sudo modprobe videobuf2-vmalloc || true
	sudo modprobe videobuf2-v4l2 || true
	sudo insmod $(MODULE_NAME).ko

# 모듈 언로드 (root 권한 필요)
unload:
	@echo "Unloading module..."
	-sudo rmmod $(MODULE_NAME)

# 모듈 재로드 (root 권한 필요)
reload: unload load

# 모듈 정보 확인
info: modules
	@echo "Module information:"
	modinfo $(MODULE_NAME).ko

# 커널 로그 확인
log:
	@echo "Recent kernel messages:"
	dmesg | tail -20

# 디바이스 확인
check:
	@echo "Checking video devices:"
	ls -la /dev/video* 2>/dev/null || echo "No video devices found"
	@echo ""
	@echo "Checking loaded modules:"
	lsmod | grep -E "(videodev|videobuf|$(MODULE_NAME))"
	@echo ""
	@echo "Checking PCI devices:"
	lspci | grep -i "1234:5678\|multimedia\|camera"

# 개발용 - 빠른 테스트
test: reload check
	@echo "Module test completed"

# 디버깅 정보
debug:
	@echo "Debug information:"
	@echo "PWD: $(PWD)"
	@echo "KERNEL_SRC: $(KERNEL_SRC)"
	@echo "MODULE_NAME: $(MODULE_NAME)"
	@echo ""
	@if [ -f "$(MODULE_NAME).ko" ]; then \
		echo "Module file exists: $(MODULE_NAME).ko"; \
		file $(MODULE_NAME).ko; \
	else \
		echo "Module file not found: $(MODULE_NAME).ko"; \
	fi

# V4L2 테스트 프로그램 빌드 (있는 경우)
test-prog:
	@if [ -f "v4l2_test.c" ]; then \
		echo "Building V4L2 test program..."; \
		gcc -o v4l2_test v4l2_test.c -Wall -g; \
	else \
		echo "v4l2_test.c not found"; \
	fi

# 모든 것 정리
distclean: clean unload
	@echo "Complete cleanup done"

# 헬프
help:
	@echo "Available targets:"
	@echo "  all       - Build the module (default)"
	@echo "  modules   - Build the module"
	@echo "  clean     - Clean build files"
	@echo "  install   - Install module to system"
	@echo "  load      - Load module into kernel"
	@echo "  unload    - Unload module from kernel"
	@echo "  reload    - Unload and load module"
	@echo "  info      - Show module information"
	@echo "  log       - Show recent kernel messages"
	@echo "  check     - Check devices and modules"
	@echo "  test      - Quick test (reload + check)"
	@echo "  debug     - Show debug information"
	@echo "  test-prog - Build V4L2 test program"
	@echo "  distclean - Complete cleanup"
	@echo "  help      - Show this help"
	@echo ""
	@echo "Usage examples:"
	@echo "  make                    # Build module"
	@echo "  make KERNEL_SRC=/path   # Build with custom kernel path"
	@echo "  sudo make load          # Load module"
	@echo "  make test               # Quick test"

.PHONY: all modules clean install load unload reload info log check test debug test-prog distclean help
