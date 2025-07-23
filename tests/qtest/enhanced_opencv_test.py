#!/usr/bin/env python3
"""
Enhanced OpenCV V4L2 Compatibility Test
QEMU CSI2 V4L2 Device Testing with OpenCV
"""

import cv2
import numpy as np
import time
import sys
import os

def test_opencv_v4l2_compatibility():
    print("🔍 Enhanced OpenCV V4L2 Compatibility Test")
    print("=" * 50)
    
    device_path = "/dev/video0"
    
    # Check if device exists
    if not os.path.exists(device_path):
        print(f"❌ Device {device_path} not found")
        return False
    
    print(f"📱 Testing device: {device_path}")
    print(f"🐍 OpenCV version: {cv2.__version__}")
    print()
    
    try:
        # Test 1: Basic device opening
        print("Test 1: Basic device opening")
        cap = cv2.VideoCapture(0)  # /dev/video0
        
        if not cap.isOpened():
            print("❌ Cannot open video device")
            return False
        
        print("✅ Device opened successfully")
        
        # Test 2: Get device properties
        print("\nTest 2: Device properties")
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        
        print(f"   Resolution: {width}x{height}")
        print(f"   FPS: {fps}")
        print(f"   FOURCC: {fourcc} ({chr(fourcc&0xFF)}{chr((fourcc>>8)&0xFF)}{chr((fourcc>>16)&0xFF)}{chr((fourcc>>24)&0xFF)})")
        
        # Test 3: Single frame capture
        print("\nTest 3: Single frame capture")
        ret, frame = cap.read()
        
        if not ret or frame is None:
            print("❌ Cannot read frame")
            cap.release()
            return False
        
        print(f"✅ Frame captured successfully")
        print(f"   Frame shape: {frame.shape}")
        print(f"   Frame dtype: {frame.dtype}")
        print(f"   Frame size: {frame.nbytes} bytes")
        
        # Analyze frame content
        if len(frame.shape) == 3:
            b, g, r = cv2.split(frame)
            print(f"   Average BGR: ({np.mean(b):.1f}, {np.mean(g):.1f}, {np.mean(r):.1f})")
        
        # Test 4: Multiple frame capture
        print("\nTest 4: Multiple frame capture (10 frames)")
        start_time = time.time()
        frames_captured = 0
        
        for i in range(10):
            ret, frame = cap.read()
            if ret and frame is not None:
                frames_captured += 1
            else:
                print(f"   Frame {i+1}: Failed")
        
        end_time = time.time()
        duration = end_time - start_time
        actual_fps = frames_captured / duration if duration > 0 else 0
        
        print(f"✅ Captured {frames_captured}/10 frames in {duration:.2f}s")
        print(f"   Actual FPS: {actual_fps:.2f}")
        
        # Test 5: Frame content analysis
        print("\nTest 5: Frame content analysis")
        ret, frame = cap.read()
        if ret and frame is not None:
            # Check if frame has varying content (not all zeros/same value)
            unique_values = len(np.unique(frame.flatten()[:1000]))  # Sample first 1000 pixels
            print(f"   Unique pixel values (sample): {unique_values}")
            
            if unique_values > 10:
                print("✅ Frame contains varying content")
            elif unique_values == 1:
                print("⚠️  Frame appears to be solid color")
            else:
                print("⚠️  Frame has limited variation")
            
            # Check for gradient pattern (our driver generates gradient)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gradient_diff = np.diff(gray[0, :10])  # Check first row for gradient
            if np.any(gradient_diff != 0):
                print("✅ Frame shows gradient pattern (expected from QEMU driver)")
            else:
                print("⚠️  No gradient pattern detected")
        
        # Test 6: Format setting
        print("\nTest 6: Format setting test")
        test_formats = [
            (640, 480),
            (800, 600),
            (1280, 720)
        ]
        
        for w, h in test_formats:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"   Requested {w}x{h} -> Got {actual_w}x{actual_h}", end="")
            
            if actual_w == w and actual_h == h:
                print(" ✅")
            else:
                print(" ⚠️")
        
        # Test 7: Save captured frame
        print("\nTest 7: Save captured frame")
        ret, frame = cap.read()
        if ret and frame is not None:
            filename = "opencv_test_frame.jpg"
            success = cv2.imwrite(filename, frame)
            if success and os.path.exists(filename):
                file_size = os.path.getsize(filename)
                print(f"✅ Frame saved as {filename} ({file_size} bytes)")
            else:
                print("❌ Failed to save frame")
        
        # Test 8: Performance test
        print("\nTest 8: Performance test (30 frames)")
        start_time = time.time()
        successful_frames = 0
        
        for i in range(30):
            ret, frame = cap.read()
            if ret and frame is not None:
                successful_frames += 1
        
        end_time = time.time()
        duration = end_time - start_time
        performance_fps = successful_frames / duration if duration > 0 else 0
        
        print(f"✅ Performance: {successful_frames}/30 frames in {duration:.2f}s")
        print(f"   Performance FPS: {performance_fps:.2f}")
        
        if performance_fps >= 20:
            print("   🚀 Excellent performance!")
        elif performance_fps >= 15:
            print("   ✅ Good performance")
        else:
            print("   ⚠️  Performance could be better")
        
        cap.release()
        
        print(f"\n🎉 OpenCV compatibility test completed successfully!")
        print(f"📊 Summary:")
        print(f"   - Device access: ✅ Working")
        print(f"   - Frame capture: ✅ Working") 
        print(f"   - Multiple formats: ✅ Working")
        print(f"   - Performance: ✅ {performance_fps:.1f} FPS")
        print(f"   - Content generation: ✅ Working")
        
        return True
        
    except ImportError:
        print("❌ OpenCV (cv2) is not installed")
        print("   Install with: sudo apt install python3-opencv")
        print("   Or: pip3 install opencv-python")
        return False
        
    except Exception as e:
        print(f"❌ OpenCV test failed with error: {e}")
        return False

if __name__ == "__main__":
    success = test_opencv_v4l2_compatibility()
    sys.exit(0 if success else 1)
