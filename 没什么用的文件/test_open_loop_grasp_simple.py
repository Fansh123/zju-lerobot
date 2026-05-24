"""
开环抓取测试 - 关节空间控制
假设：前方20cm桌面上有一个22*22*22mm的方形物块
"""

import sys
sys.path.insert(0, '.')

from visual_grasping_system.soarm101_sdk_simple import SOARM101Controller
import time
import numpy as np


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("开环抓取测试 (关节空间控制)")
    print("="*60)
    print("\n假设条件:")
    print("- 物块位置: 前方约20cm")
    print("- 物块尺寸: 22x22x22mm")
    
    arm = SOARM101Controller(port)
    
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
        
        print("\n" + "="*60)
        print("步骤2: 打开夹爪")
        print("="*60)
        arm.release(duration=1.0)
        time.sleep(0.5)
        
        print("\n" + "="*60)
        print("步骤3: 移动到物块上方")
        print("="*60)
        angles = [
            0.0,      # 肩部旋转: 0°
            -0.8,     # 肩部抬升: -45°
            1.2,      # 肘部弯曲: 70°
            0.0,      # 腕部俯仰: 0°
            0.0,      # 腕部旋转: 0°
            arm.GRIPPER_OPEN_ANGLE
        ]
        print(f"目标角度: {[f'{np.degrees(a):.1f}°' for a in angles[:5]]}")
        arm.set_joint_angles(angles, duration=2.0)
        time.sleep(0.5)
        
        print("\n" + "="*60)
        print("步骤4: 下降到抓取位置")
        print("="*60)
        angles = [
            0.0,      # 肩部旋转: 0°
            -0.5,     # 肩部抬升: -30°
            0.8,      # 肘部弯曲: 45°
            -0.3,     # 腕部俯仰: -15°
            0.0,      # 腕部旋转: 0°
            arm.GRIPPER_OPEN_ANGLE
        ]
        print(f"目标角度: {[f'{np.degrees(a):.1f}°' for a in angles[:5]]}")
        arm.set_joint_angles(angles, duration=2.0)
        time.sleep(0.5)
        
        print("\n" + "="*60)
        print("步骤5: 闭合夹爪抓取物块")
        print("="*60)
        arm.grasp(duration=1.0)
        time.sleep(1.0)
        
        print("\n" + "="*60)
        print("步骤6: 抬起物块")
        print("="*60)
        angles = [
            0.0,      # 肩部旋转: 0°
            -0.8,     # 肩部抬升: -45°
            1.2,      # 肘部弯曲: 70°
            0.0,      # 腕部俯仰: 0°
            0.0,      # 腕部旋转: 0°
            arm.GRIPPER_CLOSE_ANGLE
        ]
        print(f"目标角度: {[f'{np.degrees(a):.1f}°' for a in angles[:5]]}")
        arm.set_joint_angles(angles, duration=2.0)
        time.sleep(0.5)
        
        print("\n" + "="*60)
        print("步骤7: 旋转到放置位置 (向右90°)")
        print("="*60)
        angles = [
            1.57,     # 肩部旋转: 90°
            -0.8,     # 肩部抬升: -45°
            1.2,      # 肘部弯曲: 70°
            0.0,      # 腕部俯仰: 0°
            0.0,      # 腕部旋转: 0°
            arm.GRIPPER_CLOSE_ANGLE
        ]
        print(f"目标角度: {[f'{np.degrees(a):.1f}°' for a in angles[:5]]}")
        arm.set_joint_angles(angles, duration=2.0)
        time.sleep(0.5)
        
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
        print("✓ 开环抓取测试完成!")
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
