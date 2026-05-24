"""
测试夹爪角度范围
找出夹爪完全闭合和完全打开的位置值
"""

import sys
sys.path.insert(0, '.')

from visual_grasping_system.soarm101_sdk import SOARM101Controller, FeetechSTS
import time
import numpy as np


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*50)
    print("夹爪角度范围测试")
    print("="*50)
    
    arm = SOARM101Controller(port)
    
    if not arm.connect():
        print("连接失败!")
        return
    
    print("\n当前舵机状态:")
    arm.scan_servos()
    
    print("\n" + "="*50)
    print("测试夹爪位置范围")
    print("="*50)
    
    test_positions = [
        (500, "位置 500"),
        (800, "位置 800"),
        (1000, "位置 1000"),
        (1200, "位置 1200"),
        (1400, "位置 1400"),
        (1600, "位置 1600"),
        (1800, "位置 1800"),
        (2000, "位置 2000 (中心)"),
        (2200, "位置 2200"),
        (2400, "位置 2400"),
        (2600, "位置 2600"),
        (2800, "位置 2800"),
        (3000, "位置 3000"),
    ]
    
    print("\n输入 'y' 确认夹爪状态, 'n' 继续, 'q' 退出")
    
    for pos, desc in test_positions:
        print(f"\n移动到 {desc}...")
        arm.bus.set_position(6, pos)
        time.sleep(1.0)
        
        actual_pos = arm.bus.read_position(6)
        angle = FeetechSTS.position_to_angle(actual_pos) if actual_pos else None
        
        print(f"  实际位置: {actual_pos}")
        if angle is not None:
            print(f"  对应角度: {np.degrees(angle):.1f}°")
        
        cmd = input("夹爪状态 (y=确认/n=继续/q=退出): ").strip().lower()
        if cmd == 'q':
            break
    
    print("\n" + "="*50)
    print("请输入夹爪的最佳闭合位置和打开位置:")
    print("="*50)
    
    try:
        close_pos = int(input("夹爪完全闭合位置 (建议 800-1400): "))
        open_pos = int(input("夹爪完全打开位置 (建议 2400-3000): "))
        
        close_angle = FeetechSTS.position_to_angle(close_pos)
        open_angle = FeetechSTS.position_to_angle(open_pos)
        
        print(f"\n建议的夹爪角度限制:")
        print(f"  闭合角度: {np.degrees(close_angle):.1f}° ({close_angle:.3f} rad)")
        print(f"  打开角度: {np.degrees(open_angle):.1f}° ({open_angle:.3f} rad)")
        
        print(f"\n修改 JOINT_LIMITS[5] 为:")
        print(f"  [{close_angle:.3f}, {open_angle:.3f}]")
        
    except ValueError:
        print("无效输入")
    
    arm.disconnect()
    print("\n测试完成!")


if __name__ == "__main__":
    main()
