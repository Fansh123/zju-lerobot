"""
获取摄像头在机械臂中立姿态下的位置
"""

import numpy as np
import os
import yaml

from soarm101_sdk_urdf import SOARM101Controller
from coordinate_transformer import CoordinateTransformer


def main():
    print("="*60)
    print("摄像头位置计算")
    print("="*60)
    
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
    arm = SOARM101Controller('COM18', urdf_path=urdf_path)
    
    if not arm.connect():
        print("无法连接机械臂")
        return
    
    print("\n1. 移动到中立姿态...")
    arm.move_to_neutral(duration=2.0)
    
    import time
    time.sleep(1.0)
    
    print("\n2. 获取末端位姿...")
    ee_pos, ee_rot = arm.forward_kinematics()
    
    if ee_pos is None or ee_rot is None:
        print("无法获取末端位姿")
        arm.disconnect()
        return
    
    print(f"\n末端位置 (基座坐标系):")
    print(f"  X: {ee_pos[0]*1000:.2f} mm")
    print(f"  Y: {ee_pos[1]*1000:.2f} mm")
    print(f"  Z: {ee_pos[2]*1000:.2f} mm")
    
    print(f"\n末端旋转矩阵:")
    print(ee_rot)
    
    print("\n3. 读取手眼标定结果...")
    calib_file = os.path.join('calibration_data', 'hand_eye.yaml')
    
    if not os.path.exists(calib_file):
        print("手眼标定文件不存在")
        arm.disconnect()
        return
    
    with open(calib_file, 'r') as f:
        data = yaml.safe_load(f)
        hand_eye_R = np.array(data['rotation'])
        hand_eye_t = np.array(data['translation'])
    
    print(f"\n手眼变换 (相机到末端):")
    print(f"  旋转矩阵:")
    print(f"    {hand_eye_R}")
    print(f"  平移向量: {hand_eye_t.flatten()} mm")
    
    print("\n4. 计算摄像头位置...")
    
    t_m = hand_eye_t.flatten() / 1000.0
    cam_pos_in_ee = t_m
    
    cam_pos_in_base = ee_rot @ cam_pos_in_ee + ee_pos
    
    print(f"\n摄像头位置 (基座坐标系):")
    print(f"  X: {cam_pos_in_base[0]*1000:.2f} mm")
    print(f"  Y: {cam_pos_in_base[1]*1000:.2f} mm")
    print(f"  Z: {cam_pos_in_base[2]*1000:.2f} mm")
    
    print(f"\n摄像头相对于末端的偏移:")
    print(f"  X: {hand_eye_t.flatten()[0]:.2f} mm")
    print(f"  Y: {hand_eye_t.flatten()[1]:.2f} mm")
    print(f"  Z: {hand_eye_t.flatten()[2]:.2f} mm")
    
    print("\n" + "="*60)
    print("总结")
    print("="*60)
    print(f"末端位置: ({ee_pos[0]*1000:.1f}, {ee_pos[1]*1000:.1f}, {ee_pos[2]*1000:.1f}) mm")
    print(f"摄像头位置: ({cam_pos_in_base[0]*1000:.1f}, {cam_pos_in_base[1]*1000:.1f}, {cam_pos_in_base[2]*1000:.1f}) mm")
    print(f"摄像头高度: {cam_pos_in_base[2]*1000:.1f} mm (相对于桌面)")
    
    arm.disconnect()


if __name__ == "__main__":
    main()
