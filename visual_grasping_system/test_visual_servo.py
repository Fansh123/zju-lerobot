"""
视觉伺服诊断测试 - 带舵机控制
摄像头旋转90度，用Y偏差控制第一舵机
"""

import numpy as np
import time
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from wrist_camera import WristCamera
from soarm101_sdk_urdf import SOARM101Controller


def main():
    print("="*60)
    print("视觉伺服诊断测试 - 带舵机控制")
    print("="*60)
    
    print("\n[初始化] 摄像头...")
    camera = WristCamera(camera_id=1)
    
    if not camera.is_ready():
        print("✗ 摄像头初始化失败")
        return
    
    print("✓ 摄像头初始化成功")
    
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
    print("\n[初始化] 机械臂...")
    arm = SOARM101Controller('COM18', urdf_path=urdf_path)
    
    if not arm.connect():
        print("✗ 机械臂连接失败")
        camera.release()
        return
    
    print("✓ 机械臂连接成功")
    
    CENTER_THRESHOLD_Y = 20
    JOINT_STEP = 0.02
    
    print("\n" + "="*60)
    print("视觉伺服测试 - 摄像头旋转90度")
    print("="*60)
    print("\n按键说明:")
    print("  'q' - 退出")
    print("  's' - 保存画面")
    print("  'c' - 开始/停止自动居中")
    print("  'r' - 重置第一舵机到0位置")
    print("\n居中逻辑:")
    print("  物块在红线上方(Y<240) -> 第一舵机向右转")
    print("  物块在红线下方(Y>240) -> 第一舵机向左转")
    
    cv2 = camera.cv2
    cv2.namedWindow("Visual Servo Test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Visual Servo Test", 640, 480)
    
    auto_center = False
    detection_count = 0
    center_count = 0
    
    while True:
        frame = camera.get_frame()
        if frame is None:
            continue
        
        display = frame.copy()
        
        cv2.line(display, (0, 240), (640, 240), (0, 0, 255), 2)
        cv2.line(display, (320, 0), (320, 480), (255, 0, 0), 1)
        
        cube = camera.detect_red_cube(frame)
        
        if cube:
            detection_count += 1
            cx, cy = cube['center']
            pixel_size = cube['pixel_size']
            
            cv2.circle(display, (cx, cy), 10, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 3, (0, 255, 0), -1)
            
            error_y = cy - 240
            
            if abs(error_y) < CENTER_THRESHOLD_Y:
                color = (0, 255, 0)
                status = f"CENTERED! Y error: {error_y}px"
                center_count += 1
            else:
                color = (0, 165, 255)
                if error_y < 0:
                    direction = "UP -> Turn RIGHT"
                else:
                    direction = "DOWN -> Turn LEFT"
                status = f"Y error: {error_y}px ({direction})"
            
            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(display, f"Center: ({cx}, {cy})", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display, f"Size: {pixel_size:.0f}px", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            if auto_center and abs(error_y) >= CENTER_THRESHOLD_Y:
                angles = arm.get_joint_angles()
                if angles is not None:
                    if error_y < 0:
                        angles[0] += JOINT_STEP
                    else:
                        angles[0] -= JOINT_STEP
                    
                    angles[0] = np.clip(angles[0], -1.5, 1.5)
                    arm.set_joint_angles(angles, duration=0.2)
                    cv2.putText(display, "AUTO CENTERING...", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(display, "No red cube detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        mode = "AUTO" if auto_center else "MANUAL"
        cv2.putText(display, f"Mode: {mode}", (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display, f"Centered: {center_count}", (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("Visual Servo Test", display)
        
        key = cv2.waitKey(50) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"capture_{int(time.time())}.jpg"
            cv2.imwrite(filename, frame)
            print(f"保存画面: {filename}")
        elif key == ord('c'):
            auto_center = not auto_center
            print(f"自动居中: {'开启' if auto_center else '关闭'}")
        elif key == ord('r'):
            angles = arm.get_joint_angles()
            if angles is not None:
                angles[0] = 0
                arm.set_joint_angles(angles, duration=0.5)
                print("第一舵机重置到0位置")
    
    camera.release()
    arm.disconnect()
    cv2.destroyAllWindows()
    
    print(f"\n测试统计:")
    print(f"  检测成功: {detection_count} 次")
    print(f"  居中成功: {center_count} 次")


if __name__ == "__main__":
    main()
