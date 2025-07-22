#!/usr/bin/env python3
"""
GStreamer CSI2-V4L2 Integration Test
====================================
GStreamer를 사용한 V4L2 장치 테스트
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject
import time
import sys
import threading
import json
from pathlib import Path

class GStreamerCSI2Tester:
    def __init__(self, device_path="/dev/video0"):
        self.device_path = device_path
        self.pipeline = None
        self.loop = None
        self.frame_count = 0
        self.test_results = {}
        self.running = False
        
        # GStreamer 초기화
        Gst.init(None)
        
    def log(self, message, level="INFO"):
        """로그 메시지 출력"""
        timestamp = time.strftime("%H:%M:%S")
        icons = {"INFO": "ℹ️", "SUCCESS": "✅", "WARNING": "⚠️", "ERROR": "❌"}
        print(f"[{timestamp}] {icons.get(level, '📝')} {message}")
    
    def test_gstreamer_basic(self):
        """기본 GStreamer 파이프라인 테스트"""
        self.log("Testing basic GStreamer pipeline...")
        
        try:
            # 기본 테스트 파이프라인
            pipeline_str = f"v4l2src device={self.device_path} ! fakesink"
            self.log(f"Pipeline: {pipeline_str}")
            
            pipeline = Gst.parse_launch(pipeline_str)
            
            # 파이프라인 시작
            pipeline.set_state(Gst.State.PLAYING)
            
            # 잠시 실행
            time.sleep(2)
            
            # 상태 확인
            state = pipeline.get_state(Gst.CLOCK_TIME_NONE)[1]
            self.log(f"Pipeline state: {state}")
            
            # 정리
            pipeline.set_state(Gst.State.NULL)
            
            if state == Gst.State.PLAYING:
                self.log("Basic GStreamer test passed", "SUCCESS")
                return True
            else:
                self.log("Pipeline failed to reach PLAYING state", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"GStreamer basic test failed: {e}", "ERROR")
            return False
    
    def test_caps_negotiation(self):
        """캡스 협상 테스트"""
        self.log("Testing caps negotiation...")
        
        test_caps = [
            "video/x-raw,format=RGB,width=640,height=480,framerate=30/1",
            "video/x-raw,format=BGR,width=1280,height=720,framerate=25/1",
            "video/x-raw,format=RGB,width=1920,height=1080,framerate=30/1",
            "video/x-raw,width=320,height=240",
        ]
        
        success_count = 0
        
        for caps_str in test_caps:
            self.log(f"Testing caps: {caps_str}")
            
            try:
                pipeline_str = f"v4l2src device={self.device_path} ! {caps_str} ! fakesink"
                pipeline = Gst.parse_launch(pipeline_str)
                
                # 파이프라인 시작
                pipeline.set_state(Gst.State.PLAYING)
                time.sleep(1)
                
                # 상태 확인
                state = pipeline.get_state(Gst.CLOCK_TIME_NONE)[1]
                
                if state == Gst.State.PLAYING:
                    self.log(f"  ✅ Caps accepted")
                    success_count += 1
                else:
                    self.log(f"  ❌ Caps rejected")
                
                pipeline.set_state(Gst.State.NULL)
                
            except Exception as e:
                self.log(f"  💥 Caps test error: {e}")
        
        success_rate = success_count / len(test_caps)
        self.log(f"Caps negotiation: {success_count}/{len(test_caps)} passed ({success_rate*100:.1f}%)")
        
        return success_rate >= 0.5
    
    def test_format_conversion(self):
        """포맷 변환 테스트"""
        self.log("Testing format conversion...")
        
        conversions = [
            ("videoconvert ! video/x-raw,format=RGB", "RGB conversion"),
            ("videoconvert ! video/x-raw,format=BGR", "BGR conversion"),
            ("videoconvert ! video/x-raw,format=YUY2", "YUY2 conversion"),
            ("videoscale ! video/x-raw,width=320,height=240", "Scale down"),
            ("videoscale ! video/x-raw,width=1920,height=1080", "Scale up"),
        ]
        
        success_count = 0
        
        for conversion, desc in conversions:
            self.log(f"Testing {desc}...")
            
            try:
                pipeline_str = f"v4l2src device={self.device_path} ! {conversion} ! fakesink"
                pipeline = Gst.parse_launch(pipeline_str)
                
                pipeline.set_state(Gst.State.PLAYING)
                time.sleep(1.5)
                
                state = pipeline.get_state(Gst.CLOCK_TIME_NONE)[1]
                
                if state == Gst.State.PLAYING:
                    self.log(f"  ✅ {desc} successful")
                    success_count += 1
                else:
                    self.log(f"  ❌ {desc} failed")
                
                pipeline.set_state(Gst.State.NULL)
                
            except Exception as e:
                self.log(f"  💥 {desc} error: {e}")
        
        success_rate = success_count / len(conversions)
        self.log(f"Format conversion: {success_count}/{len(conversions)} passed ({success_rate*100:.1f}%)")
        
        return success_rate >= 0.6
    
    def test_file_recording(self, duration=5):
        """파일 녹화 테스트"""
        self.log(f"Testing file recording for {duration}s...")
        
        output_file = "gstreamer_test_recording.mp4"
        
        try:
            # 기존 파일 삭제
            if Path(output_file).exists():
                Path(output_file).unlink()
            
            # 녹화 파이프라인
            pipeline_str = (f"v4l2src device={self.device_path} ! "
                           f"videoconvert ! "
                           f"x264enc ! "
                           f"mp4mux ! "
                           f"filesink location={output_file}")
            
            self.log(f"Recording pipeline: {pipeline_str}")
            
            pipeline = Gst.parse_launch(pipeline_str)
            
            # 녹화 시작
            start_time = time.time()
            pipeline.set_state(Gst.State.PLAYING)
            
            # 지정된 시간 동안 녹화
            while time.time() - start_time < duration:
                state = pipeline.get_state(Gst.CLOCK_TIME_NONE)[1]
                if state != Gst.State.PLAYING:
                    self.log("Pipeline stopped unexpectedly", "ERROR")
                    break
                time.sleep(0.1)
            
            # 녹화 중지
            pipeline.set_state(Gst.State.NULL)
            
            # 파일 확인
            if Path(output_file).exists():
                file_size = Path(output_file).stat().st_size
                self.log(f"Recording completed: {output_file} ({file_size} bytes)", "SUCCESS")
                
                # 파일 크기가 합리적인지 확인
                if file_size > 1024:  # 최소 1KB
                    return True
                else:
                    self.log("Recording file too small", "WARNING")
                    return False
            else:
                self.log("Recording file not created", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"File recording test failed: {e}", "ERROR")
            return False
    
    def test_streaming_server(self, duration=10):
        """스트리밍 서버 테스트"""
        self.log(f"Testing streaming server for {duration}s...")
        
        try:
            # RTSP 스트리밍 서버 파이프라인
            pipeline_str = (f"v4l2src device={self.device_path} ! "
                           f"videoconvert ! "
                           f"x264enc tune=zerolatency bitrate=1000 ! "
                           f"rtph264pay ! "
                           f"udpsink host=127.0.0.1 port=5000")
            
            self.log(f"Streaming pipeline: {pipeline_str}")
            
            pipeline = Gst.parse_launch(pipeline_str)
            
            # 스트리밍 시작
            start_time = time.time()
            pipeline.set_state(Gst.State.PLAYING)
            
            # 파이프라인 상태 모니터링
            stable_time = 0
            while time.time() - start_time < duration:
                state = pipeline.get_state(Gst.CLOCK_TIME_NONE)[1]
                
                if state == Gst.State.PLAYING:
                    stable_time += 0.5
                else:
                    self.log(f"Pipeline state: {state}", "WARNING")
                
                time.sleep(0.5)
            
            # 스트리밍 중지
            pipeline.set_state(Gst.State.NULL)
            
            stability_ratio = stable_time / duration
            self.log(f"Streaming stability: {stability_ratio*100:.1f}%")
            
            if stability_ratio >= 0.8:
                self.log("Streaming server test passed", "SUCCESS")
                return True
            else:
                self.log("Streaming was unstable", "WARNING")
                return False
                
        except Exception as e:
            self.log(f"Streaming server test failed: {e}", "ERROR")
            return False
    
    def test_frame_callback(self, duration=5):
        """프레임 콜백 테스트"""
        self.log(f"Testing frame callback for {duration}s...")
        
        self.frame_count = 0
        
        def on_new_sample(sink):
            """새 샘플(프레임) 콜백"""
            sample = sink.emit('pull-sample')
            if sample:
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    self.log(f"Received {self.frame_count} frames")
            return Gst.FlowReturn.OK
        
        try:
            # 콜백 테스트 파이프라인
            pipeline_str = f"v4l2src device={self.device_path} ! appsink name=sink"
            pipeline = Gst.parse_launch(pipeline_str)
            
            # appsink 설정
            sink = pipeline.get_by_name('sink')
            sink.set_property('emit-signals', True)
            sink.connect('new-sample', on_new_sample)
            
            # 파이프라인 시작
            start_time = time.time()
            pipeline.set_state(Gst.State.PLAYING)
            
            # 메인 루프 실행
            loop = GObject.MainLoop()
            
            def stop_loop():
                time.sleep(duration)
                loop.quit()
            
            stop_thread = threading.Thread(target=stop_loop)
            stop_thread.start()
            
            loop.run()
            
            # 정리
            pipeline.set_state(Gst.State.NULL)
            stop_thread.join()
            
            total_time = time.time() - start_time
            fps = self.frame_count / total_time if total_time > 0 else 0
            
            self.log(f"Callback test results:")
            self.log(f"  Frames received: {self.frame_count}")
            self.log(f"  Average FPS: {fps:.2f}")
            
            if self.frame_count > 0 and fps >= 10:  # 최소 10 FPS
                self.log("Frame callback test passed", "SUCCESS")
                return True
            else:
                self.log("Frame callback test failed", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"Frame callback test failed: {e}", "ERROR")
            return False
    
    def test_device_properties(self):
        """디바이스 속성 테스트"""
        self.log("Testing device properties...")
        
        try:
            # 속성 쿼리 파이프라인
            pipeline_str = f"v4l2src device={self.device_path} name=src ! fakesink"
            pipeline = Gst.parse_launch(pipeline_str)
            
            src = pipeline.get_by_name('src')
            
            # 파이프라인 시작 (PAUSED 상태로)
            pipeline.set_state(Gst.State.PAUSED)
            pipeline.get_state(Gst.CLOCK_TIME_NONE)
            
            # 속성들 확인
            properties = [
                'device',
                'device-name', 
                'norm',
                'io-mode',
                'extra-controls',
            ]
            
            self.log("Device properties:")
            for prop in properties:
                try:
                    value = src.get_property(prop)
                    self.log(f"  {prop}: {value}")
                except Exception as e:
                    self.log(f"  {prop}: N/A ({e})")
            
            # 캡스 정보
            pad = src.get_static_pad('src')
            caps = pad.query_caps(None)
            self.log(f"  Available caps: {caps.to_string()}")
            
            pipeline.set_state(Gst.State.NULL)
            
            self.log("Device properties test completed", "SUCCESS")
            return True
            
        except Exception as e:
            self.log(f"Device properties test failed: {e}", "ERROR")
            return False
    
    def save_test_results(self, filename="gstreamer_csi2_test_results.json"):
        """테스트 결과 저장"""
        self.log(f"Saving test results to {filename}...")
        
        results = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "device_path": self.device_path,
            "gstreamer_version": Gst.version_string(),
            "tests": self.test_results
        }
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.log(f"Test results saved", "SUCCESS")
    
    def run_all_tests(self):
        """모든 테스트 실행"""
        self.log("🚀 Starting GStreamer CSI2-V4L2 Integration Test")
        self.log("=" * 60)
        
        tests = [
            ("GStreamer Basic", self.test_gstreamer_basic),
            ("Device Properties", self.test_device_properties),
            ("Caps Negotiation", self.test_caps_negotiation),
            ("Format Conversion", self.test_format_conversion),
            ("File Recording", lambda: self.test_file_recording(duration=3)),
            ("Frame Callback", lambda: self.test_frame_callback(duration=3)),
            ("Streaming Server", lambda: self.test_streaming_server(duration=5)),
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
        self.log("🎯 GSTREAMER TEST SUMMARY")
        self.log("=" * 60)
        
        for test_name, result in self.test_results.items():
            status = "✅ PASS" if result["passed"] else "❌ FAIL"
            self.log(f"{status} {test_name}")
            if "error" in result:
                self.log(f"     Error: {result['error']}")
        
        success_rate = passed / total * 100
        self.log(f"\n📊 GStreamer Success Rate: {passed}/{total} ({success_rate:.1f}%)")
        
        if success_rate >= 80:
            self.log("🎉 GStreamer Integration: EXCELLENT!", "SUCCESS")
        elif success_rate >= 60:
            self.log("👍 GStreamer Integration: GOOD", "SUCCESS")
        elif success_rate >= 40:
            self.log("⚠️  GStreamer Integration: ACCEPTABLE", "WARNING")
        else:
            self.log("❌ GStreamer Integration: NEEDS WORK", "ERROR")
        
        # 결과 저장
        self.save_test_results()
        
        return success_rate >= 60

def main():
    """메인 함수"""
    device_path = "/dev/video0"
    
    # 명령줄 인수 처리
    if len(sys.argv) > 1:
        device_path = sys.argv[1]
    
    tester = GStreamerCSI2Tester(device_path)
    
    try:
        success = tester.run_all_tests()
        sys.exit(0 if success else 1)
    finally:
        pass  # GStreamer는 자동으로 정리됨

if __name__ == "__main__":
    main()
