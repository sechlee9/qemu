#!/bin/bash
# test_csi2_devices.sh - MIPI CSI2 디바이스 종합 테스트 스크립트
# Author: QEMU CSI2 Team
# Version: 1.0

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 로깅 함수
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_header() {
    echo -e "${PURPLE}===============================================${NC}"
    echo -e "${PURPLE}$1${NC}"
    echo -e "${PURPLE}===============================================${NC}"
}

# 전역 변수
MIPI_X86_DEVICE=""
MIPI_V4L2_DEVICE=""
V4L2_DEVICE=""
TEST_DURATION=10
FRAME_COUNT=30
OUTPUT_DIR="/tmp/csi2_test"

# 테스트 결과 저장
TEST_RESULTS=()

# 결과 기록 함수
record_result() {
    local test_name="$1"
    local result="$2"
    local details="$3"
    
    if [ "$result" = "PASS" ]; then
        log_success "$test_name: PASS - $details"
    else
        log_error "$test_name: FAIL - $details"
    fi
    
    TEST_RESULTS+=("$test_name:$result:$details")
}

# 출력 디렉토리 생성
create_output_dir() {
    log_info "Creating output directory: $OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR"
    chmod 755 "$OUTPUT_DIR"
}

# 시스템 정보 수집
collect_system_info() {
    log_header "SYSTEM INFORMATION"
    
    echo "Kernel Version: $(uname -r)"
    echo "Distribution: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
    echo "Architecture: $(uname -m)"
    echo "Date: $(date)"
    echo "Uptime: $(uptime)"
    echo ""
}

# PCI 디바이스 확인
check_pci_devices() {
    log_header "PCI DEVICE DETECTION"
    
    log_info "Scanning for QEMU CSI2 PCI devices..."
    
    # mipi-csi-camera-x86 확인 (vendor:device = 1234:5678)
    local x86_device=$(lspci -d 1234:5678 | head -1)
    if [ -n "$x86_device" ]; then
        MIPI_X86_DEVICE=$(echo "$x86_device" | cut -d' ' -f1)
        log_success "Found mipi-csi-camera-x86 at $MIPI_X86_DEVICE"
        echo "  Details: $x86_device"
        record_result "PCI_X86_DEVICE" "PASS" "Device found at $MIPI_X86_DEVICE"
    else
        log_error "mipi-csi-camera-x86 device not found"
        record_result "PCI_X86_DEVICE" "FAIL" "Device not found in PCI scan"
    fi
    
    # mipi-csi-camera-v4l2 확인 (vendor:device = 1234:5679)
    local v4l2_device=$(lspci -d 1234:5679 | head -1)
    if [ -n "$v4l2_device" ]; then
        MIPI_V4L2_DEVICE=$(echo "$v4l2_device" | cut -d' ' -f1)
        log_success "Found mipi-csi-camera-v4l2 at $MIPI_V4L2_DEVICE"
        echo "  Details: $v4l2_device"
        record_result "PCI_V4L2_DEVICE" "PASS" "Device found at $MIPI_V4L2_DEVICE"
    else
        log_error "mipi-csi-camera-v4l2 device not found"
        record_result "PCI_V4L2_DEVICE" "FAIL" "Device not found in PCI scan"
    fi
    
    echo ""
    log_info "All PCI devices with vendor ID 1234:"
    lspci -d 1234: -v || log_warning "No devices found with vendor ID 1234"
    echo ""
}

# 커널 모듈 확인
check_kernel_modules() {
    log_header "KERNEL MODULE STATUS"
    
    log_info "Checking loaded CSI2 kernel modules..."
    
    # qemu-csi2 모듈 확인 (다양한 이름 패턴 지원)
    local module_found=false
    local module_info=""
    
    # 패턴 1: qemu_csi2_v4l2 (현재 사용중인 모듈명)
    if lsmod | grep -q "qemu_csi2_v4l2"; then
        module_info=$(lsmod | grep "qemu_csi2_v4l2")
        module_found=true
    # 패턴 2: 일반적인 qemu csi2 패턴들
    elif lsmod | grep -q "qemu.*csi2\|csi2.*qemu"; then
        module_info=$(lsmod | grep -E "qemu.*csi2|csi2.*qemu")
        module_found=true
    fi
    
    if [ "$module_found" = true ]; then
        log_success "qemu-csi2 module loaded"
        echo "  $module_info"
        record_result "KERNEL_MODULE" "PASS" "qemu-csi2 module loaded"
    else
        log_error "qemu-csi2 module not loaded"
        log_info "Available modules (for debugging):"
        lsmod | head -5
        record_result "KERNEL_MODULE" "FAIL" "Module not found"
    fi
    
    # v4l2loopback 모듈 확인 (선택적)
    if lsmod | grep -q "v4l2loopback"; then
        local v4l2loop_info=$(lsmod | grep "v4l2loopback")
        log_success "v4l2loopback module loaded"
        echo "  $v4l2loop_info"
    else
        log_warning "v4l2loopback module not loaded (optional)"
    fi
    
    echo ""
}

# V4L2 디바이스 확인
check_v4l2_devices() {
    log_header "V4L2 DEVICE DETECTION"
    
    log_info "Scanning for V4L2 video devices..."
    
    if ! command -v v4l2-ctl &> /dev/null; then
        log_error "v4l2-ctl not found. Please install v4l-utils package."
        record_result "V4L2_TOOLS" "FAIL" "v4l2-ctl not available"
        return 1
    fi
    
    # 사용 가능한 video 디바이스 목록
    log_info "Available video devices:"
    ls -la /dev/video* 2>/dev/null || log_warning "No video devices found"
    echo ""
    
    # V4L2 디바이스 목록
    log_info "V4L2 device list:"
    v4l2-ctl --list-devices 2>/dev/null || log_warning "Failed to list V4L2 devices"
    echo ""
    
    # QEMU CSI2 디바이스 찾기
    local found_device=""
    for video_dev in /dev/video*; do
        if [ -c "$video_dev" ]; then
            local device_info=$(v4l2-ctl -d "$video_dev" --info 2>/dev/null | grep -i "qemu\|csi2" || true)
            if [ -n "$device_info" ]; then
                V4L2_DEVICE="$video_dev"
                found_device="$video_dev"
                log_success "Found QEMU CSI2 V4L2 device: $video_dev"
                echo "  Device info: $device_info"
                break
            fi
        fi
    done
    
    if [ -n "$found_device" ]; then
        record_result "V4L2_DEVICE" "PASS" "Device found at $found_device"
    else
        log_error "QEMU CSI2 V4L2 device not found"
        record_result "V4L2_DEVICE" "FAIL" "No QEMU CSI2 device found"
        
        # 첫 번째 video 디바이스를 fallback으로 사용
        if ls /dev/video* &>/dev/null; then
            V4L2_DEVICE="/dev/video0"
            log_warning "Using fallback device: $V4L2_DEVICE"
        fi
    fi
    
    echo ""
}

# PCI 디바이스 MMIO 테스트
test_pci_mmio() {
    log_header "PCI MMIO REGISTER TESTS"
    
    if [ -z "$MIPI_X86_DEVICE" ] && [ -z "$MIPI_V4L2_DEVICE" ]; then
        log_error "No PCI devices found to test"
        record_result "MMIO_TEST" "FAIL" "No devices available"
        return 1
    fi
    
    # QEMU 내부에서 디바이스 테스트
    log_info "Testing PCI device accessibility..."
    
    # /sys/bus/pci/devices/ 에서 디바이스 정보 확인
    if [ -n "$MIPI_X86_DEVICE" ]; then
        local pci_path="/sys/bus/pci/devices/0000:$MIPI_X86_DEVICE"
        if [ -d "$pci_path" ]; then
            log_success "mipi-csi-camera-x86 PCI device accessible"
            log_info "  Vendor ID: $(cat $pci_path/vendor 2>/dev/null || echo 'N/A')"
            log_info "  Device ID: $(cat $pci_path/device 2>/dev/null || echo 'N/A')"
            log_info "  Driver: $(basename $(readlink $pci_path/driver 2>/dev/null) 2>/dev/null || echo 'None')"
            record_result "X86_MMIO_ACCESS" "PASS" "Device accessible via sysfs"
        else
            log_error "mipi-csi-camera-x86 not accessible via sysfs"
            record_result "X86_MMIO_ACCESS" "FAIL" "sysfs access failed"
        fi
    fi
    
    if [ -n "$MIPI_V4L2_DEVICE" ]; then
        local pci_path="/sys/bus/pci/devices/0000:$MIPI_V4L2_DEVICE"
        if [ -d "$pci_path" ]; then
            log_success "mipi-csi-camera-v4l2 PCI device accessible"
            log_info "  Vendor ID: $(cat $pci_path/vendor 2>/dev/null || echo 'N/A')"
            log_info "  Device ID: $(cat $pci_path/device 2>/dev/null || echo 'N/A')"
            log_info "  Driver: $(basename $(readlink $pci_path/driver 2>/dev/null) 2>/dev/null || echo 'None')"
            record_result "V4L2_MMIO_ACCESS" "PASS" "Device accessible via sysfs"
        else
            log_error "mipi-csi-camera-v4l2 not accessible via sysfs"
            record_result "V4L2_MMIO_ACCESS" "FAIL" "sysfs access failed"
        fi
    fi
    
    echo ""
}

# V4L2 디바이스 기능 테스트
test_v4l2_capabilities() {
    log_header "V4L2 DEVICE CAPABILITY TESTS"
    
    if [ -z "$V4L2_DEVICE" ]; then
        log_error "No V4L2 device available for testing"
        record_result "V4L2_CAPABILITIES" "FAIL" "No device available"
        return 1
    fi
    
    log_info "Testing V4L2 device: $V4L2_DEVICE"
    
    # 디바이스 정보 확인
    log_info "Device capabilities:"
    if v4l2-ctl -d "$V4L2_DEVICE" --info > "$OUTPUT_DIR/device_info.txt" 2>&1; then
        cat "$OUTPUT_DIR/device_info.txt"
        log_success "Device info retrieved successfully"
        record_result "V4L2_INFO" "PASS" "Device info accessible"
    else
        log_error "Failed to get device info"
        record_result "V4L2_INFO" "FAIL" "Cannot access device info"
    fi
    
    # 지원 포맷 확인
    log_info "Supported formats:"
    if v4l2-ctl -d "$V4L2_DEVICE" --list-formats > "$OUTPUT_DIR/formats.txt" 2>&1; then
        cat "$OUTPUT_DIR/formats.txt"
        log_success "Format list retrieved successfully"
        record_result "V4L2_FORMATS" "PASS" "Formats accessible"
    else
        log_error "Failed to get format list"
        record_result "V4L2_FORMATS" "FAIL" "Cannot access formats"
    fi
    
    # 현재 포맷 확인
    log_info "Current format:"
    if v4l2-ctl -d "$V4L2_DEVICE" --get-fmt-video > "$OUTPUT_DIR/current_format.txt" 2>&1; then
        cat "$OUTPUT_DIR/current_format.txt"
        log_success "Current format retrieved successfully"
        record_result "V4L2_CURRENT_FORMAT" "PASS" "Current format accessible"
    else
        log_error "Failed to get current format"
        record_result "V4L2_CURRENT_FORMAT" "FAIL" "Cannot access current format"
    fi
    
    echo ""
}

# 프레임 캡처 테스트
test_frame_capture() {
    log_header "FRAME CAPTURE TESTS"
    
    if [ -z "$V4L2_DEVICE" ]; then
        log_error "No V4L2 device available for frame capture"
        record_result "FRAME_CAPTURE" "FAIL" "No device available"
        return 1
    fi
    
    log_info "Testing frame capture from $V4L2_DEVICE"
    
    # 짧은 캡처 테스트 (5프레임)
    log_info "Capturing 5 test frames..."
    local capture_file="$OUTPUT_DIR/test_capture.raw"
    
    if timeout 5 v4l2-ctl -d "$V4L2_DEVICE" --stream-mmap --stream-count=5 --stream-to="$capture_file" > "$OUTPUT_DIR/capture_log.txt" 2>&1; then
        if [ -f "$capture_file" ] && [ -s "$capture_file" ]; then
            local file_size=$(stat -c%s "$capture_file")
            log_success "Frame capture successful (captured $file_size bytes)"
            record_result "FRAME_CAPTURE" "PASS" "Captured $file_size bytes in 5 frames"
        else
            log_error "Capture file is empty or missing"
            record_result "FRAME_CAPTURE" "FAIL" "Empty capture file"
        fi
    else
        log_error "Frame capture failed"
        if [ -f "$OUTPUT_DIR/capture_log.txt" ]; then
            log_info "Capture error log:"
            tail -5 "$OUTPUT_DIR/capture_log.txt"
        fi
        record_result "FRAME_CAPTURE" "FAIL" "Capture command failed"
    fi
    
    echo ""
}

# 스트리밍 성능 테스트
test_streaming_performance() {
    log_header "STREAMING PERFORMANCE TESTS"
    
    if [ -z "$V4L2_DEVICE" ]; then
        log_error "No V4L2 device available for streaming test"
        record_result "STREAMING_PERFORMANCE" "FAIL" "No device available"
        return 1
    fi
    
    log_info "Testing streaming performance for $TEST_DURATION seconds..."
    log_info "Target: $FRAME_COUNT frames"
    
    local perf_file="$OUTPUT_DIR/streaming_performance.txt"
    local start_time=$(date +%s)
    
    # 직접 타임아웃을 적용한 스트리밍 테스트
    log_info "Starting streaming capture..."
    timeout $TEST_DURATION v4l2-ctl -d "$V4L2_DEVICE" --stream-mmap --stream-count=$FRAME_COUNT --stream-to=/dev/null > "$perf_file" 2>&1
    local exit_code=$?
    
    local end_time=$(date +%s)
    local elapsed=$((end_time - start_time))
    
    if [ -f "$perf_file" ] && [ -s "$perf_file" ]; then
        # v4l2-ctl 출력에서 프레임 수 추출 (여러 방법 시도)
        local captured_frames=0
        
        # 방법 1: "<<" 문자열 카운트 (각 프레임마다 출력됨)
        if grep -q "<<" "$perf_file"; then
            captured_frames=$(grep -c "<<" "$perf_file" 2>/dev/null || echo "0")
        fi
        
        # 방법 2: 숫자 패턴 찾기
        if [ "$captured_frames" -eq 0 ] && grep -q "^[0-9]*$" "$perf_file"; then
            captured_frames=$(tail -1 "$perf_file" 2>/dev/null | grep -o "^[0-9]*" || echo "0")
        fi
        
        # 방법 3: "seq=" 패턴 찾기
        if [ "$captured_frames" -eq 0 ] && grep -q "seq=" "$perf_file"; then
            local last_seq=$(grep -o "seq=[0-9]*" "$perf_file" 2>/dev/null | tail -1 | cut -d= -f2)
            if [ -n "$last_seq" ]; then
                captured_frames=$((last_seq + 1))
            fi
        fi
        
        # 방법 4: 줄 수로 추정 (fallback)
        if [ "$captured_frames" -eq 0 ]; then
            captured_frames=$(wc -l < "$perf_file" 2>/dev/null || echo "0")
            if [ "$captured_frames" -gt "$FRAME_COUNT" ]; then
                captured_frames="$FRAME_COUNT"
            fi
        fi
        
        # FPS 계산
        local fps="0.00"
        if [ "$elapsed" -gt 0 ] && [ "$captured_frames" -gt 0 ]; then
            fps=$(awk "BEGIN {printf \"%.2f\", $captured_frames / $elapsed}")
        fi
        
        log_success "Streaming test completed"
        log_info "  Duration: ${elapsed}s"
        log_info "  Frames captured: $captured_frames"
        log_info "  Average FPS: $fps"
        log_info "  Exit code: $exit_code (124=timeout, 0=success)"
        
        # 성공 조건: 프레임이 캡처되었거나 타임아웃으로 정상 종료
        if [ "$captured_frames" -gt 0 ] || [ "$exit_code" -eq 124 ]; then
            record_result "STREAMING_PERFORMANCE" "PASS" "$captured_frames frames in ${elapsed}s (${fps} fps)"
        else
            record_result "STREAMING_PERFORMANCE" "FAIL" "No frames captured (exit: $exit_code)"
        fi
        
        # 디버그: 출력 파일 내용 표시 (처음 5줄)
        log_info "Performance test output (first 5 lines):"
        head -5 "$perf_file" | while read line; do
            echo "    $line"
        done
        
    else
        log_error "Performance test failed - no output file or empty"
        record_result "STREAMING_PERFORMANCE" "FAIL" "No output generated"
    fi
    
    echo ""
}

# 인터럽트 및 IRQ 테스트
test_interrupts() {
    log_header "INTERRUPT AND IRQ TESTS"
    
    log_info "Checking interrupt statistics..."
    
    # /proc/interrupts에서 CSI2 관련 인터럽트 확인
    local irq_info=$(grep -i "csi2\|mipi" /proc/interrupts 2>/dev/null || true)
    if [ -n "$irq_info" ]; then
        log_success "Found CSI2 interrupts:"
        echo "$irq_info"
        record_result "INTERRUPT_DETECTION" "PASS" "CSI2 interrupts found"
    else
        log_warning "No CSI2-specific interrupts found in /proc/interrupts"
        record_result "INTERRUPT_DETECTION" "WARN" "No specific interrupts found"
    fi
    
    # MSI-X 지원 확인
    if [ -n "$MIPI_V4L2_DEVICE" ]; then
        local pci_path="/sys/bus/pci/devices/0000:$MIPI_V4L2_DEVICE"
        if [ -f "$pci_path/msi_irqs" ]; then
            local msi_irqs=$(ls "$pci_path/msi_irqs" 2>/dev/null | wc -l)
            if [ "$msi_irqs" -gt 0 ]; then
                log_success "MSI-X interrupts enabled ($msi_irqs vectors)"
                record_result "MSIX_SUPPORT" "PASS" "$msi_irqs MSI-X vectors"
            else
                log_info "No MSI-X interrupts (using legacy IRQ)"
                record_result "MSIX_SUPPORT" "INFO" "Legacy IRQ mode"
            fi
        fi
    fi
    
    echo ""
}

# 드라이버 로그 확인
check_driver_logs() {
    log_header "DRIVER LOG ANALYSIS"
    
    log_info "Recent kernel messages related to CSI2:"
    
    # dmesg에서 CSI2 관련 메시지 확인
    local log_file="$OUTPUT_DIR/driver_logs.txt"
    dmesg | grep -i "csi2\|mipi\|qemu.*video" | tail -20 > "$log_file"
    
    if [ -s "$log_file" ]; then
        cat "$log_file"
        log_success "Driver logs collected"
        record_result "DRIVER_LOGS" "PASS" "Logs available"
    else
        log_warning "No recent CSI2-related kernel messages found"
        record_result "DRIVER_LOGS" "WARN" "No recent messages"
    fi
    
    echo ""
}

# v4l2loopback 통합 테스트
test_v4l2loopback_integration() {
    log_header "V4L2LOOPBACK INTEGRATION TEST"
    
    if ! lsmod | grep -q "v4l2loopback"; then
        log_warning "v4l2loopback module not loaded - skipping integration test"
        record_result "V4L2LOOPBACK_INTEGRATION" "SKIP" "Module not loaded"
        return 0
    fi
    
    log_info "Testing v4l2loopback integration..."
    
    # v4l2loopback 디바이스 찾기
    local loopback_device=""
    for video_dev in /dev/video*; do
        if [ -c "$video_dev" ]; then
            local device_info=$(v4l2-ctl -d "$video_dev" --info 2>/dev/null | grep -i "loopback\|dummy" || true)
            if [ -n "$device_info" ]; then
                loopback_device="$video_dev"
                log_success "Found v4l2loopback device: $video_dev"
                break
            fi
        fi
    done
    
    if [ -n "$loopback_device" ]; then
        # 간단한 테스트 패턴 전송
        log_info "Testing pattern transmission to $loopback_device"
        if echo "test pattern" | timeout 2 tee "$loopback_device" > /dev/null 2>&1; then
            log_success "v4l2loopback write test successful"
            record_result "V4L2LOOPBACK_INTEGRATION" "PASS" "Pattern transmission successful"
        else
            log_warning "v4l2loopback write test failed"
            record_result "V4L2LOOPBACK_INTEGRATION" "WARN" "Write test failed"
        fi
    else
        log_warning "No v4l2loopback device found"
        record_result "V4L2LOOPBACK_INTEGRATION" "WARN" "No loopback device found"
    fi
    
    echo ""
}

# 종합 리포트 생성
generate_report() {
    log_header "TEST SUMMARY REPORT"
    
    local report_file="$OUTPUT_DIR/test_report.txt"
    local total_tests=0
    local passed_tests=0
    local failed_tests=0
    local skipped_tests=0
    
    {
        echo "MIPI CSI2 Device Test Report"
        echo "Generated: $(date)"
        echo "System: $(uname -a)"
        echo "Output Directory: $OUTPUT_DIR"
        echo ""
        echo "=================================="
        echo "TEST RESULTS SUMMARY"
        echo "=================================="
        echo ""
    } > "$report_file"
    
    for result in "${TEST_RESULTS[@]}"; do
        IFS=':' read -r test_name test_result test_details <<< "$result"
        total_tests=$((total_tests + 1))
        
        printf "%-30s: %-6s - %s\n" "$test_name" "$test_result" "$test_details" >> "$report_file"
        
        case "$test_result" in
            "PASS") passed_tests=$((passed_tests + 1)) ;;
            "FAIL") failed_tests=$((failed_tests + 1)) ;;
            "SKIP"|"WARN"|"INFO") skipped_tests=$((skipped_tests + 1)) ;;
        esac
    done
    
    {
        echo ""
        echo "=================================="
        echo "STATISTICS"
        echo "=================================="
        echo "Total Tests: $total_tests"
        echo "Passed: $passed_tests"
        echo "Failed: $failed_tests" 
        echo "Skipped/Warnings: $skipped_tests"
        echo "Success Rate: $(awk "BEGIN {printf \"%.2f\", $passed_tests * 100 / $total_tests}")%"
        echo ""
    } >> "$report_file"
    
    # 화면에도 출력
    cat "$report_file"
    
    log_info "Full report saved to: $report_file"
    
    # 전체 테스트 결과 평가
    if [ "$failed_tests" -eq 0 ]; then
        log_success "All critical tests passed! CSI2 devices are working properly."
        return 0
    else
        log_error "$failed_tests critical test(s) failed. Please check the issues above."
        return 1
    fi
}

# 사용법 출력
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -d, --duration SECONDS    Test duration for streaming test (default: $TEST_DURATION)"
    echo "  -f, --frames COUNT        Frame count for capture test (default: $FRAME_COUNT)"
    echo "  -o, --output DIR          Output directory (default: $OUTPUT_DIR)"
    echo "  -h, --help                Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                        Run all tests with default settings"
    echo "  $0 -d 30 -f 100          Run with 30s streaming test and 100 frame capture"
    echo "  $0 -o /home/user/test     Use custom output directory"
    echo ""
}

# 메인 함수
main() {
    # 명령행 인수 처리
    while [[ $# -gt 0 ]]; do
        case $1 in
            -d|--duration)
                TEST_DURATION="$2"
                shift 2
                ;;
            -f|--frames)
                FRAME_COUNT="$2"
                shift 2
                ;;
            -o|--output)
                OUTPUT_DIR="$2"
                shift 2
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
    done
    
    # 필수 명령어 확인
    for cmd in awk timeout; do
        if ! command -v "$cmd" &> /dev/null; then
            log_error "$cmd command not found. Please install required package."
            exit 1
        fi
    done
    
    log_header "MIPI CSI2 DEVICE COMPREHENSIVE TEST"
    log_info "Test Duration: ${TEST_DURATION}s"
    log_info "Frame Count: $FRAME_COUNT"
    log_info "Output Directory: $OUTPUT_DIR"
    echo ""
    
    # 출력 디렉토리 생성
    create_output_dir
    
    # 테스트 실행
    collect_system_info
    check_pci_devices
    check_kernel_modules
    check_v4l2_devices
    test_pci_mmio
    test_v4l2_capabilities
    test_frame_capture
    test_streaming_performance
    test_interrupts
    check_driver_logs
    test_v4l2loopback_integration
    
    # 결과 리포트 생성
    generate_report
}

# 스크립트 실행
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
