"""
手眼标定测试程序
测试图像坐标到机械臂坐标的转换精度
"""

import numpy as np
import os
import sys
import time

from soarm101_sdk_urdf import SOARM101Controller
from wrist_camera import WristCamera
from coordinate_transformer import CoordinateTransformer


def test_hand_eye_calibration():
    print("="*60)
    print("手眼标定测试")
    print("="*60)
    
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
    arm = SOARM101Controller('COM18', urdf_path=urdf_path)
    camera = WristCamera(camera_id=1)
    transformer = CoordinateTransformer('calibration_data')
    
    if not arm.connect():
        print("无法连接机械臂")
        return
    
    if not camera.is_ready():
        print("摄像头未就绪")
        arm.disconnect()
        return
    
    if not transformer.is_calibrated():
        print("手眼标定未完成")
        camera.release()
        arm.disconnect()
        return
    
    print("\n测试步骤:")
    print("1. 将红色物块放在桌面上")
    print("2. 程序检测物块位置")
    print("3. 机械臂移动到计算位置")
    print("4. 观察误差并记录")
    print("\n按 'q' 退出测试")
    print("按 't' 开始测试")
    print("按 'm' 手动输入坐标测试")
    
    test_count = 0
    total_error = [0, 0, 0]
    
    while True:
        frame = camera.get_frame()
        if frame is None:
            continue
        
        display = frame.copy()
        
        cube = camera.detect_red_cube(frame)
        if cube:
            display = camera.draw_cube_detection(display, cube)
            
            ee_pos, ee_rot = arm.forward_kinematics()
            if ee_pos is not None and ee_rot is not None:
                depth = transformer.estimate_depth(cube['pixel_size'])
                point_base = transformer.image_to_base(
                    cube['center'], depth, ee_pos, ee_rot
                )
                if point_base is not None:
                    text = f"Base: ({point_base[0]*1000:.1f}, {point_base[1]*1000:.1f}, {point_base[2]*1000:.1f}) mm"
                    camera.cv2.putText(display, text, (10, 30), 
                                      camera.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        camera.cv2.imshow("Hand-Eye Test", display)
        
        key = camera.cv2.waitKey(100) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('t'):
            if cube is None:
                print("\n未检测到物块，请将红色物块放在摄像头视野中")
                continue
            
            test_count += 1
            print(f"\n{'='*40}")
            print(f"测试 #{test_count}")
            print(f"{'='*40}")
            
            print(f"图像坐标: {cube['center']}")
            print(f"像素尺寸: {cube['pixel_size']:.1f}px")
            
            ee_pos, ee_rot = arm.forward_kinematics()
            if ee_pos is None or ee_rot is None:
                print("无法获取末端位姿")
                continue
            
            depth = transformer.estimate_depth(cube['pixel_size'])
            print(f"深度估计: {depth*1000:.1f}mm")
            
            point_base = transformer.image_to_base(
                cube['center'], depth, ee_pos, ee_rot
            )
            
            if point_base is None:
                print("坐标转换失败")
                continue
            
            target_x = point_base[0]
            target_y = point_base[1]
            target_z = 0.011
            
            print(f"目标位置: ({target_x*1000:.1f}, {target_y*1000:.1f}, {target_z*1000:.1f}) mm")
            
            print("\n移动机械臂到目标位置...")
            success = arm.move_to_xyz([target_x, target_y, target_z], duration=2.0)
            
            if success:
                print("✓ 机械臂已移动到目标位置")
                print("\n请观察机械臂末端与物块的位置偏差")
                
                error_x = input("输入X方向误差(mm, 正值=偏前, 负值=偏后): ")
                error_y = input("输入Y方向误差(mm, 正值=偏左, 负值=偏右): ")
                error_z = input("输入Z方向误差(mm, 正值=偏高, 负值=偏低): ")
                
                try:
                    ex = float(error_x) if error_x else 0
                    ey = float(error_y) if error_y else 0
                    ez = float(error_z) if error_z else 0
                    total_error[0] += ex
                    total_error[1] += ey
                    total_error[2] += ez
                    print(f"本次误差: X={ex}mm, Y={ey}mm, Z={ez}mm")
                except:
                    pass
            else:
                print("✗ 无法到达目标位置")
            
            print("\n按 't' 继续下一次测试，或按 'q' 退出")
        
        elif key == ord('m'):
            print("\n手动输入坐标测试")
            try:
                x = float(input("输入目标X坐标(mm): ")) / 1000
                y = float(input("输入目标Y坐标(mm): ")) / 1000
                z = float(input("输入目标Z坐标(mm, 默认50): ") or "50") / 1000
                
                print(f"移动到 ({x*1000}, {y*1000}, {z*1000}) mm...")
                success = arm.move_to_xyz([x, y, z], duration=2.0)
                
                if success:
                    print("✓ 移动成功")
                else:
                    print("✗ 移动失败")
            except:
                print("输入无效")
    
    if test_count > 0:
        print(f"\n{'='*40}")
        print("测试总结")
        print(f"{'='*40}")
        print(f"测试次数: {test_count}")
        print(f"平均误差: X={total_error[0]/test_count:.1f}mm, Y={total_error[1]/test_count:.1f}mm, Z={total_error[2]/test_count:.1f}mm")
        total_dist = np.sqrt(total_error[0]**2 + total_error[1]**2 + total_error[2]**2) / test_count
        print(f"平均总误差: {total_dist:.1f}mm")
    
    camera.release()
    arm.disconnect()
    print("\n测试完成")


if __name__ == "__main__":
    test_hand_eye_calibration()
