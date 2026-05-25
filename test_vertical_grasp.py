"""
竖直向下夹取物块测试
物块位置: (200mm, 0mm, 0mm)
"""

import sys
sys.path.insert(0, '.')

from visual_grasping_system.soarm101_sdk_urdf import SOARM101Controller
import time
import numpy as np
import os


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    urdf_path = os.path.join(os.path.dirname(__file__), 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
    
    print("="*60)
    print("竖直向下夹取物块测试")
    print("="*60)
    print("\n物块位置: (200mm, 0mm, 0mm)")
    
    arm = SOARM101Controller(port, urdf_path=urdf_path)
    
    if not arm.connect():
        print("连接失败!")
        return
    
    print("\n扫描舵机...")
    arm.scan_servos()
    
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
        print("步骤3: 移动到物块上方 (200mm, 0mm, 100mm)")
        print("="*60)
        arm.move_to_xyz([0.200, 0.0, 0.100], duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_current_xyz()
        print(f"当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
        
        print("\n" + "="*60)
        print("步骤4: 竖直下降到物块位置 (200mm, 0mm, 0mm)")
        print("="*60)
        arm.move_to_xyz([0.200, 0.1, 0.100], duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_current_xyz()
        print(f"当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
        
        print("\n" + "="*60)
        print("步骤5: 闭合夹爪抓取物块")
        print("="*60)
        arm.grasp(duration=1.0)
        time.sleep(1.0)
        
        print("\n" + "="*60)
        print("步骤6: 竖直抬起物块 (200mm, 0mm, 150mm)")
        print("="*60)
        arm.move_to_xyz([0.200, 0.0, 0.150], duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_current_xyz()
        print(f"当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
        
        print("\n" + "="*60)
        print("步骤7: 移动到放置位置 (100mm, 100mm, 150mm)")
        print("="*60)
        arm.move_to_xyz([0.100, 0.100, 0.150], duration=2.0)
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
        print("✓ 竖直向下夹取测试完成!")
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
