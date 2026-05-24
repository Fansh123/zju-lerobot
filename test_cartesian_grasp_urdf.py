"""
笛卡尔空间抓取测试 (基于URDF)
假设：前方20cm桌面上有一个22*22*22mm的方形物块
"""

import sys
sys.path.insert(0, '.')

from visual_grasping_system.soarm101_sdk_urdf import SOARM101Controller
import time
import numpy as np


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    import os
    urdf_path = os.path.join(os.path.dirname(__file__), 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
    if not os.path.exists(urdf_path):
        urdf_path = None
    
    print("="*60)
    print("笛卡尔空间抓取测试 (URDF版)")
    print("="*60)
    print("\n假设条件:")
    print("- 物块位置: 前方约20cm")
    print("- 物块尺寸: 22x22x22mm")
    
    arm = SOARM101Controller(port, urdf_path=urdf_path)
    
    if not arm.connect():
        print("连接失败!")
        return
    
    print("\n扫描舵机...")
    arm.scan_servos()
    
    if not arm.urdf:
        print("[ERROR] URDF未加载，无法进行笛卡尔空间控制")
        arm.disconnect()
        return
    
    try:
        print("\n" + "="*60)
        print("步骤1: 移动到中立位置")
        print("="*60)
        arm.move_to_neutral(duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_current_xyz()
        print(f"当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
        
        print("\n" + "="*60)
        print("步骤2: 打开夹爪")
        print("="*60)
        arm.release(duration=1.0)
        time.sleep(0.5)
        
        print("\n" + "="*60)
        print("步骤3: 移动到物块上方 (前20cm, 左0cm, 上15cm)")
        print("="*60)
        target = [0.20, 0.0, 0.15]
        arm.move_to_xyz(target, duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_current_xyz()
        print(f"当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
        
        print("\n" + "="*60)
        print("步骤4: 下降到抓取高度 (上5cm)")
        print("="*60)
        target = [0.20, 0.0, 0.05]
        arm.move_to_xyz(target, duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_current_xyz()
        print(f"当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
        
        print("\n" + "="*60)
        print("步骤5: 闭合夹爪抓取物块")
        print("="*60)
        arm.grasp(duration=1.0)
        time.sleep(1.0)
        
        print("\n" + "="*60)
        print("步骤6: 抬起物块 (上15cm)")
        print("="*60)
        target = [0.20, 0.0, 0.15]
        arm.move_to_xyz(target, duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_current_xyz()
        print(f"当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
        
        print("\n" + "="*60)
        print("步骤7: 移动到放置位置 (前15cm, 左10cm)")
        print("="*60)
        target = [0.15, 0.10, 0.15]
        arm.move_to_xyz(target, duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_current_xyz()
        print(f"当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
        
        print("\n" + "="*60)
        print("步骤8: 放置物块")
        print("="*60)
        arm.release(duration=1.0)
        time.sleep(0.5)
        
        print("\n" + "="*60)
        print("步骤9: 返回中立位置")
        print("="*60)
        arm.move_to_neutral(duration=2.0)
        
        print("\n" + "="*60)
        print("✓ 笛卡尔空间抓取测试完成!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    
    arm.disconnect()
    print("\n断开连接")


if __name__ == "__main__":
    main()
