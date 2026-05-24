"""
开环抓取测试
假设：前方20cm桌面上有一个22*22*22mm的方形物块
"""

import sys
sys.path.insert(0, '.')

from visual_grasping_system.soarm101_sdk import SOARM101Controller
import time
import numpy as np


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("开环抓取测试")
    print("="*60)
    print("\n假设条件:")
    print("- 物块位置: 前方200mm, 左侧0mm")
    print("- 物块尺寸: 22x22x22mm")
    print("- 桌面高度: 假设末端需要下降到Z=80mm")
    
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
        
        pos = arm.get_user_position()
        print(f"当前位置: 前{pos['forward']:.1f}mm, 左{pos['left']:.1f}mm, 上{pos['up']:.1f}mm")
        
        print("\n" + "="*60)
        print("步骤2: 打开夹爪")
        print("="*60)
        arm.release(duration=1.0)
        time.sleep(0.5)
        
        print("\n" + "="*60)
        print("步骤3: 移动到物块上方 (前200mm, 左0mm, 上150mm)")
        print("="*60)
        target_x = 0.200  # 前200mm
        target_y = 0.0    # 左0mm
        target_z = 0.150  # 上150mm (物块上方)
        
        arm.move_to_xyz([target_x, target_y, target_z], duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_user_position()
        print(f"当前位置: 前{pos['forward']:.1f}mm, 左{pos['left']:.1f}mm, 上{pos['up']:.1f}mm")
        
        print("\n" + "="*60)
        print("步骤4: 下降到抓取高度 (上80mm)")
        print("="*60)
        target_z = 0.080  # 上80mm
        arm.move_to_xyz([target_x, target_y, target_z], duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_user_position()
        print(f"当前位置: 前{pos['forward']:.1f}mm, 左{pos['left']:.1f}mm, 上{pos['up']:.1f}mm")
        
        print("\n" + "="*60)
        print("步骤5: 闭合夹爪抓取物块")
        print("="*60)
        arm.grasp(duration=1.0)
        time.sleep(1.0)
        
        print("\n" + "="*60)
        print("步骤6: 抬起物块 (上150mm)")
        print("="*60)
        target_z = 0.150
        arm.move_to_xyz([target_x, target_y, target_z], duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_user_position()
        print(f"当前位置: 前{pos['forward']:.1f}mm, 左{pos['left']:.1f}mm, 上{pos['up']:.1f}mm")
        
        print("\n" + "="*60)
        print("步骤7: 移动到放置位置 (前150mm, 左100mm)")
        print("="*60)
        place_x = 0.150
        place_y = 0.100
        arm.move_to_xyz([place_x, place_y, target_z], duration=2.0)
        time.sleep(0.5)
        
        pos = arm.get_user_position()
        print(f"当前位置: 前{pos['forward']:.1f}mm, 左{pos['left']:.1f}mm, 上{pos['up']:.1f}mm")
        
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
