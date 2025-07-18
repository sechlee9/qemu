#!/bin/bash
# scripts/test-csi2-v4l2.sh - CSI2-V4L2 통합 테스트 스크립트

echo "=== QEMU CSI2-V4L2 Integration Test ==="

# 1. 필수 모듈 확인 및 설치
echo "Checking v4l2loopback module..."
if ! lsmod | grep -q v4l2loopback; then
    echo "Installing v4l2loopback module..."
    sudo apt update
    sudo apt install -y v4l2loopback-dkms v4l2loopback-utils
fi

# 2. QEMU 실행 (V4L2 백엔드 포함)
echo "Starting QEMU with CSI2-V4L2 device..."

CSI2_NUM_LANES=4
CSI2_LINE_RATE=1440
CSI2_FRAME_WIDTH=1920
CSI2_FRAME_HEIGHT=1080

qemu-system-x86_64 \
    -s \
    -kernel ../../generic/linux-6.15.4/arch/x86_64/boot/bzImage \
    -append "root=/dev/vda2 rw console=ttyS0,115200 v4l2loopback.devices=1" \
    -smp 4 \
    -accel kvm \
    -serial file:qemu-csi2-v4l2.log \
    -nographic \
    -drive file=qemu.img,if=none,id=disk1,format=raw \
    -device virtio-blk-pci,drive=disk1 \
    -netdev user,id=net0,hostfwd=tcp::2222-:22,hostfwd=tcp::8080-:8080 \
    -device virtio-net-pci,netdev=net0 \
    -machine q35,cxl=on -m 8G,maxmem=32G,slots=8 \
    -device csi2-v4l2-device,num-lanes=$CSI2_NUM_LANES,line-rate=$CSI2_LINE_RATE,frame-width=$CSI2_FRAME_WIDTH,frame-height=$CSI2_FRAME_HEIGHT,auto-start=true \
    -virtfs local,path=/lib/modules,mount_tag=modshare,security_model=mapped \
    -virtfs local,path=/home/sechlee,mount_tag=homeshare,security_model=mapped

echo "QEMU started. Check /dev/video* devices in guest OS."

# 3. 게스트에서 실행할 테스트 명령어들
cat << 'EOF' > guest_test_commands.sh
#!/bin/bash
echo "=== Guest OS V4L2 Device Test ==="

# V4L2 디바이스 확인
echo "Available video devices:"
ls -la /dev/video*

# V4L2 디바이스 정보 조회
echo "V4L2 device capabilities:"
v4l2-ctl --list-devices

# 디바이스 포맷 정보
echo "V4L2 device formats:"
v4l2-ctl -d /dev/video0 --list-formats-ext

# 프레임 캡처 테스트
echo "Capturing test frames..."
ffmpeg -f v4l2 -i /dev/video0 -frames:v 10 -y test_capture_%03d.jpg

# GStreamer 테스트 (설치되어 있는 경우)
if command -v gst-launch-1.0 &> /dev/null; then
    echo "Testing with GStreamer..."
    gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! jpegenc ! filesink location=gstreamer_test.jpg
fi

# V4L2 스트리밍 테스트
echo "Testing V4L2 streaming..."
timeout 10s ffmpeg -f v4l2 -i /dev/video0 -c:v libx264 -t 5 test_video.mp4

echo "=== Test completed ==="
EOF

chmod +x guest_test_commands.sh
echo "Created guest_test_commands.sh for testing in guest OS"

# 4. 호스트에서 V4L2 디바이스 모니터링
echo "You can also monitor V4L2 devices on host:"
echo "watch -n 1 'ls -la /dev/video* && echo && v4l2-ctl --list-devices'"
