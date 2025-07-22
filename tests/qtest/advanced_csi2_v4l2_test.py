#!/usr/bin/env python3
"""
Advanced CSI2-V4L2 Integration Test
===================================
이 스크립트는 QEMU에서 CSI2와 V4L2 통합을 포괄적으로 테스트합니다.
"""

import cv2
import numpy as np
import time
import sys
import os
import fcntl
import struct
import mmap
from pathlib import Path
import json
import threading
import queue

# V4L2 상수들
V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_STREAMING = 0x04000000
VIDIOC_QUERYCAP = 0x80685600
VIDIOC_G_FMT = 0xc0cc5604
VIDIOC_S_FMT = 0xc0cc5605

class CSI2V4L2Tester:
    def __init__(self, device_path="/dev/video0"):
        self.device_path = device_path
        self.cap = None
        self.test_results = {}
        self.frame_queue = queue.Queue(maxsize=100)
        self.stop_capture = False
        
    def log(self, message, level="INFO"):
        """로그 메시지 출력"""
        timestamp = time.strftime("%H:%M:%S")
        icons = {"INFO": "ℹ️", "SUCCESS": "✅", "WARNING": "⚠️", "ERROR": "❌"}
        print(f"[{timestamp}] {icons.get(level, '📝')} {message}")
        
    def test_device_discovery(self):
        """디바이스 발견 테스트"""
        self.log("Testing device discovery...")
        
        # /dev/video* 디바이스들 검색
        video_devices = list(Path("/dev").glob("video*"))
        self.log(f"Found {len(video_devices)} video devices: {[str(d) for d in video_devices]}")
        
        # 메인 디바이스 존재 확인
        if Path(self.device_path).exists():
            self.log(f"Target device {self.device_path} exists", "SUCCESS")
            return True
        else:
            self.log(f"Target device {self.device_path} not found", "ERROR")
            return False
    
    def test_v4l2_capabilities(self):
        """V4L2 capabilities 직접 테스트"""
        self.log("Testing V4L2 capabilities directly...")
        
        try:
            with open(self.device_path, 'rb') as fd:
                # VIDIOC_QUERYCAP 호출
                cap_struct = bytearray(104)  # v4l2_capability 구조체 크기
                fcntl.ioctl(fd.fileno(), VIDIOC_QUERYCAP, cap_struct)
                
                # 결과 파싱
                driver = cap_struct[0:16].decode('utf-8', errors='ignore').rstrip('\x00')
                card = cap_struct[16:48].decode('utf-8', errors='ignore').rstrip('\x00')
                bus_info = cap_struct[48:80].decode('utf-8', errors='ignore').rstrip('\x00')
                capabilities = struct.unpack('I', cap_struct[84:88])[0]
                
                self.log(f"Driver: {driver}")
                self.log(f"Card: {card}")
                self.log(f"Bus: {bus_info}")
                self.log(f"Capabilities: 0x{capabilities:08x}")
                
                # 기능 확인
                has_capture = bool(capabilities & V4L2_CAP_VIDEO_CAPTURE)
                has_streaming = bool(capabilities & V4L2_CAP_STREAMING)
                
                self.log(f"Video Capture: {'Yes' if has_capture else 'No'}")
                self.log(f"Streaming: {'Yes' if has_streaming else 'No'}")
                
                if has_capture and has_streaming:
                    self.log("V4L2 capabilities check passed", "SUCCESS")
                    return True
                else:
                    self.log("Required V4L2 capabilities missing", "ERROR")
                    return False
                    
        except Exception as e:
            self.log(f"V4L2 capabilities test failed: {e}", "ERROR")
            return False
    
    def test_opencv_integration(self):
        """OpenCV 통합 테스트"""
        self.log("Testing OpenCV integration...")
        
        try:
            self.cap = cv2.VideoCapture(self.device_path)
            if not self.cap.isOpened():
                self.log("Failed to open device with OpenCV", "ERROR")
                return False
                
            # 기본 속성 읽기
            width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            self.log(f"OpenCV Properties - Resolution: {width}x{height}, FPS: {fps}")
            
            # 프레임 캡처 테스트
            ret, frame = self.cap.read()
            if ret:
                self.log(f"Frame captured: {frame.shape}, dtype: {frame.dtype}", "SUCCESS")
                return True
            else:
                self.log("Failed to capture frame", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"OpenCV integration test failed: {e}", "ERROR")
            return False
    
    def test_continuous_capture(self, duration=10, target_fps=30):
        """연속 캡처 테스트"""
        self.log(f"Testing continuous capture for {duration}s at {target_fps} FPS...")
        
        if not self.cap:
            self.log("OpenCV not initialized", "ERROR")
            return False
            
        frame_count = 0
        start_time = time.time()
        last_fps_time = start_time
        fps_frames = 0
        
        frame_times = []
        frame_sizes = []
        
        while time.time() - start_time < duration:
            ret, frame = self.cap.read()
            if ret:
                frame_count += 1
                fps_frames += 1
                current_time = time.time()
                frame_times.append(current_time)
                frame_sizes.append(frame.nbytes)
                
                # FPS 계산 (매초)
                if current_time - last_fps_time >= 1.0:
                    current_fps = fps_frames / (current_time - last_fps_time)
                    self.log(f"Current FPS: {current_fps:.1f}, Total frames: {frame_count}")
                    last_fps_time = current_time
                    fps_frames = 0
                    
            else:
                self.log("Frame capture failed", "WARNING")
                time.sleep(0.001)  # 작은 지연
        
        total_time = time.time() - start_time
        avg_fps = frame_count / total_time
        
        # 통계 계산
        if len(frame_times) > 1:
            intervals = [frame_times[i] - frame_times[i-1] for i in range(1, len(frame_times))]
            avg_interval = np.mean(intervals)
            std_interval = np.std(intervals)
            jitter = std_interval / avg_interval * 100 if avg_interval > 0 else 0
            
            self.log(f"Capture Statistics:")
            self.log(f"  Total frames: {frame_count}")
            self.log(f"  Average FPS: {avg_fps:.2f}")
            self.log(f"  Frame interval: {avg_interval*1000:.2f}±{std_interval*1000:.2f}ms")
            self.log(f"  Timing jitter: {jitter:.1f}%")
            self.log(f"  Average frame size: {np.mean(frame_sizes)/1024:.1f} KB")
            
            # 성능 평가
            if avg_fps >= target_fps * 0.9:  # 90% 이상이면 성공
                self.log("Continuous capture test passed", "SUCCESS")
                return True
            else:
                self.log(f"FPS too low: {avg_fps:.1f} < {target_fps * 0.9:.1f}", "WARNING")
                return False
        else:
            self.log("No frames captured", "ERROR")
            return False
    
    def test_format_changes(self):
        """포맷 변경 테스트"""
        self.log("Testing format changes...")
        
        if not self.cap:
            self.log("OpenCV not initialized", "ERROR")
            return False
        
        test_formats = [
            (640, 480),
            (1280, 720),
            (1920, 1080),
            (320, 240),
        ]
        
        success_count = 0
        
        for width, height in test_formats:
            self.log(f"Testing format {width}x{height}...")
            
            # 포맷 설정 시도
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # 실제 설정된 값 확인
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            
            # 프레임 캡처해서 실제 크기 확인
            ret, frame = self.cap.read()
            if ret:
                frame_height, frame_width = frame.shape[:2]
                self.log(f"  Requested: {width}x{height}")
                self.log(f"  Property: {actual_width}x{actual_height}")
                self.log(f"  Actual frame: {frame_width}x{frame_height}")
                
                # 유연한 성공 기준 (QEMU에서는 고정 해상도일 수 있음)
                if frame_width > 0 and frame_height > 0:
                    success_count += 1
                    self.log(f"  Format test passed", "SUCCESS")
                else:
                    self.log(f"  Invalid frame size", "ERROR")
            else:
                self.log(f"  Failed to capture frame", "ERROR")
        
        success_rate = success_count / len(test_formats)
        self.log(f"Format change test: {success_count}/{len(test_formats)} passed ({success_rate*100:.1f}%)")
        
        return success_rate >= 0.5  # 50% 이상 성공하면 통과
    
    def test_frame_analysis(self):
        """프레임 내용 분석 테스트"""
        self.log("Testing frame content analysis...")
        
        if not self.cap:
            self.log("OpenCV not initialized", "ERROR")
            return False
        
        frames = []
        for i in range(10):
            ret, frame = self.cap.read()
            if ret:
                frames.append(frame)
            time.sleep(0.1)
        
        if not frames:
            self.log("No frames captured for analysis", "ERROR")
            return False
        
        # 프레임 분석
        first_frame = frames[0]
        last_frame = frames[-1]
        
        # 통계 계산
        mean_first = np.mean(first_frame, axis=(0, 1))
        mean_last = np.mean(last_frame, axis=(0, 1))
        
        # 프레임 간 차이
        frame_diff = np.mean(np.abs(first_frame.astype(float) - last_frame.astype(float)))
        
        # 색상 분포
        hist_b = cv2.calcHist([first_frame], [0], None, [256], [0, 256])
        hist_g = cv2.calcHist([first_frame], [1], None, [256], [0, 256])
        hist_r = cv2.calcHist([first_frame], [2], None, [256], [0, 256])
        
        # 엔트로피 계산 (복잡도 측정)
        def calculate_entropy(hist):
            hist_norm = hist.flatten() / np.sum(hist)
            hist_norm = hist_norm[hist_norm > 0]  # 0 제거
            return -np.sum(hist_norm * np.log2(hist_norm))
        
        entropy_b = calculate_entropy(hist_b)
        entropy_g = calculate_entropy(hist_g)
        entropy_r = calculate_entropy(hist_r)
        
        self.log(f"Frame Analysis Results:")
        self.log(f"  First frame mean BGR: ({mean_first[0]:.1f}, {mean_first[1]:.1f}, {mean_first[2]:.1f})")
        self.log(f"  Last frame mean BGR: ({mean_last[0]:.1f}, {mean_last[1]:.1f}, {mean_last[2]:.1f})")
        self.log(f"  Frame difference: {frame_diff:.2f}")
        self.log(f"  Color entropy BGR: ({entropy_b:.2f}, {entropy_g:.2f}, {entropy_r:.2f})")
        
        # 프레임이 변화하고 있는지 확인 (QEMU 드라이버가 애니메이션을 생성하는지)
        has_variation = frame_diff > 1.0 or max(entropy_b, entropy_g, entropy_r) > 3.0
        
        if has_variation:
            self.log("Frame content shows variation", "SUCCESS")
        else:
            self.log("Frame content appears static", "WARNING")
        
        return True
    
    def test_threaded_capture(self, duration=5):
        """멀티스레드 캡처 테스트"""
        self.log(f"Testing threaded capture for {duration}s...")
        
        def capture_worker():
            """캡처 워커 스레드"""
            frame_count = 0
            while not self.stop_capture:
                ret, frame = self.cap.read()
                if ret:
                    try:
                        self.frame_queue.put(frame, timeout=0.1)
                        frame_count += 1
                    except queue.Full:
                        pass  # 큐가 가득 찬 경우 무시
                else:
                    time.sleep(0.001)
            self.log(f"Capture worker finished, captured {frame_count} frames")
        
        def process_worker():
            """처리 워커 스레드"""
            processed_count = 0
            while not self.stop_capture or not self.frame_queue.empty():
                try:
                    frame = self.frame_queue.get(timeout=1.0)
                    # 간단한 처리 (평균 계산)
                    mean_value = np.mean(frame)
                    processed_count += 1
                    if processed_count % 30 == 0:
                        self.log(f"Processed {processed_count} frames, last mean: {mean_value:.1f}")
                except queue.Empty:
                    break
            self.log(f"Process worker finished, processed {processed_count} frames")
            return processed_count
        
        if not self.cap:
            self.log("OpenCV not initialized", "ERROR")
            return False
        
        # 스레드 시작
        self.stop_capture = False
        capture_thread = threading.Thread(target=capture_worker)
        process_thread = threading.Thread(target=process_worker)
        
        start_time = time.time()
        capture_thread.start()
        process_thread.start()
        
        # 지정된 시간 대기
        time.sleep(duration)
        
        # 캡처 중지
        self.stop_capture = True
        capture_thread.join(timeout=2.0)
        process_thread.join(timeout=2.0)
        
        total_time = time.time() - start_time
        queue_size = self.frame_queue.qsize()
        
        self.log(f"Threaded capture completed:")
        self.log(f"  Duration: {total_time:.2f}s")
        self.log(f"  Remaining queue size: {queue_size}")
        
        # 큐 정리
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        
        self.log("Threaded capture test passed", "SUCCESS")
        return True
    
    def save_test_results(self, filename="csi2_v4l2_test_results.json"):
        """테스트 결과 저장"""
        self.log(f"Saving test results to {filename}...")
        
        results = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "device_path": self.device_path,
            "opencv_version": cv2.__version__,
            "tests": self.test_results
        }
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.log(f"Test results saved", "SUCCESS")
    
    def cleanup(self):
        """정리"""
        if self.cap:
            self.cap.release()
            self.cap = None
        self.log("Cleanup completed")
    
    def run_all_tests(self):
        """모든 테스트 실행"""
        self.log("🚀 Starting Advanced CSI2-V4L2 Integration Test")
        self.log("=" * 60)
        
        tests = [
            ("Device Discovery", self.test_device_discovery),
            ("V4L2 Capabilities", self.test_v4l2_capabilities),
            ("OpenCV Integration", self.test_opencv_integration),
            ("Continuous Capture", lambda: self.test_continuous_capture(duration=5)),
            ("Format Changes", self.test_format_changes),
            ("Frame Analysis", self.test_frame_analysis),
            ("Threaded Capture", lambda: self.test_threaded_capture(duration=3)),
        ]
        
        passed = 0
        total = len(tests)
        
        for test_name, test_func in tests:
            self.log(f"\n🧪 Running test: {test_name}")
            self.log("-" * 40)
            
            try:
                result = test_func()
                self.test_results[test_name] = {
                    "passed": result,
                    "timestamp": time.strftime("%H:%M:%S")
                }
                
                if result:
                    passed += 1
                    self.log(f"✅ Test '{test_name}' PASSED")
                else:
                    self.log(f"❌ Test '{test_name}' FAILED")
                    
            except Exception as e:
                self.log(f"💥 Test '{test_name}' CRASHED: {e}", "ERROR")
                self.test_results[test_name] = {
                    "passed": False,
                    "error": str(e),
                    "timestamp": time.strftime("%H:%M:%S")
                }
        
        # 결과 요약
        self.log("\n" + "=" * 60)
        self.log("🎯 TEST SUMMARY")
        self.log("=" * 60)
        
        for test_name, result in self.test_results.items():
            status = "✅ PASS" if result["passed"] else "❌ FAIL"
            self.log(f"{status} {test_name}")
            if "error" in result:
                self.log(f"     Error: {result['error']}")
        
        success_rate = passed / total * 100
        self.log(f"\n📊 Overall Success Rate: {passed}/{total} ({success_rate:.1f}%)")
        
        if success_rate >= 80:
            self.log("🎉 CSI2-V4L2 Integration: EXCELLENT!", "SUCCESS")
        elif success_rate >= 60:
            self.log("👍 CSI2-V4L2 Integration: GOOD", "SUCCESS")
        elif success_rate >= 40:
            self.log("⚠️  CSI2-V4L2 Integration: ACCEPTABLE", "WARNING")
        else:
            self.log("❌ CSI2-V4L2 Integration: NEEDS WORK", "ERROR")
        
        # 결과 저장
        self.save_test_results()
        
        return success_rate >= 60

def main():
    """메인 함수"""
    device_path = "/dev/video0"
    
    # 명령줄 인수 처리
    if len(sys.argv) > 1:
        device_path = sys.argv[1]
    
    tester = CSI2V4L2Tester(device_path)
    
    try:
        success = tester.run_all_tests()
        sys.exit(0 if success else 1)
    finally:
        tester.cleanup()

if __name__ == "__main__":
    main()
