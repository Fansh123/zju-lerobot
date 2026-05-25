"""
手动控制测试 - 测试SDK运动控制准确性
支持关节空间和笛卡尔空间控制
"""

import sys
sys.path.insert(0, '.')

from visual_grasping_system.soarm101_sdk_urdf import SOARM101Controller
import time
import numpy as np
import os


def print_help():
    print("\n" + "="*60)
    print("手动控制命令")
    print("="*60)
    print("\n关节控制 (直接输入角度):")
    print("  j1 <角度>  - 肩部旋转 (度)")
    print("  j2 <角度>  - 肩部抬升 (度)")
    print("  j3 <角度>  - 肘部弯曲 (度)")
    print("  j4 <角度>  - 腕部俯仰 (度)")
    print("  j5 <角度>  - 腕部旋转 (度)")
    print("\n笛卡尔控制 (输入毫米):")
    print("  x <值>     - X轴移动 (前)")
    print("  y <值>     - Y轴移动 (左)")
    print("  z <值>     - Z轴移动 (上)")
    print("  xyz <x> <y> <z> - 移动到指定位置")
    print("\n其他命令:")
    print("  pos        - 显示当前位置")
    print("  angles     - 显示当前关节角度")
    print("  neutral    - 移动到中立位置")
    print("  open       - 打开夹爪")
    print("  close      - 闭合夹爪")
    print("  help       - 显示帮助")
    print("  quit       - 退出")
    print("="*60)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    urdf_path = os.path.join(os.path.dirname(__file__), 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
    
    print("="*60)
    print("SO-ARM101 手动控制测试")
    print("="*60)
    
    arm = SOARM101Controller(port, urdf_path=urdf_path)
    
    if not arm.connect():
        print("连接失败!")
        return
    
    print("\n扫描舵机...")
    arm.scan_servos()
    
    arm.move_to_neutral(duration=1.5)
    
    print_help()
    
    if arm.urdf:
        pos = arm.get_current_xyz()
        print(f"\n当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
    
    angles = arm.get_joint_angles()
    print(f"当前关节角度: {[f'{np.degrees(a):.1f}°' for a in angles[:5]]}")
    
    running = True
    while running:
        try:
            cmd = input("\n> ").strip().lower()
            
            if not cmd:
                continue
            
            parts = cmd.split()
            action = parts[0]
            
            if action == 'quit' or action == 'q':
                print("退出...")
                running = False
            
            elif action == 'help' or action == 'h' or action == '?':
                print_help()
            
            elif action == 'pos':
                if arm.urdf:
                    pos = arm.get_current_xyz()
                    print(f"末端位置: X={pos[0]*1000:.1f}mm, Y={pos[1]*1000:.1f}mm, Z={pos[2]*1000:.1f}mm")
                else:
                    print("URDF未加载，无法获取笛卡尔位置")
            
            elif action == 'angles':
                angles = arm.get_joint_angles()
                print(f"关节角度:")
                print(f"  J1 (肩部旋转): {np.degrees(angles[0]):.1f}°")
                print(f"  J2 (肩部抬升): {np.degrees(angles[1]):.1f}°")
                print(f"  J3 (肘部弯曲): {np.degrees(angles[2]):.1f}°")
                print(f"  J4 (腕部俯仰): {np.degrees(angles[3]):.1f}°")
                print(f"  J5 (腕部旋转): {np.degrees(angles[4]):.1f}°")
            
            elif action == 'neutral':
                print("移动到中立位置...")
                arm.move_to_neutral(duration=1.5)
                print("完成")
            
            elif action == 'open':
                print("打开夹爪...")
                arm.release(duration=0.8)
                print("完成")
            
            elif action == 'close':
                print("闭合夹爪...")
                arm.grasp(duration=0.8)
                print("完成")
            
            elif action in ['j1', 'j2', 'j3', 'j4', 'j5']:
                if len(parts) < 2:
                    print(f"用法: {action} <角度(度)>")
                    continue
                
                try:
                    angle_deg = float(parts[1])
                    angle_rad = np.radians(angle_deg)
                    
                    joint_idx = int(action[1]) - 1
                    angles = arm.get_joint_angles()
                    angles[joint_idx] = angle_rad
                    
                    print(f"设置 J{joint_idx+1} = {angle_deg:.1f}°")
                    arm.set_joint_angles(angles, duration=1.0)
                    print("完成")
                    
                except ValueError:
                    print("无效的角度值")
            
            elif action == 'x' or action == 'y' or action == 'z':
                if not arm.urdf:
                    print("URDF未加载，无法进行笛卡尔控制")
                    continue
                
                if len(parts) < 2:
                    print(f"用法: {action} <值(mm)>")
                    continue
                
                try:
                    value = float(parts[1]) / 1000.0  # mm -> m
                    
                    pos = arm.get_current_xyz()
                    
                    if action == 'x':
                        target = [value, pos[1], pos[2]]
                    elif action == 'y':
                        target = [pos[0], value, pos[2]]
                    else:
                        target = [pos[0], pos[1], value]
                    
                    print(f"移动到 ({target[0]*1000:.1f}, {target[1]*1000:.1f}, {target[2]*1000:.1f}) mm...")
                    arm.move_to_xyz(target, duration=2.0)
                    print("完成")
                    
                except ValueError:
                    print("无效的数值")
            
            elif action == 'xyz':
                if not arm.urdf:
                    print("URDF未加载，无法进行笛卡尔控制")
                    continue
                
                if len(parts) < 4:
                    print("用法: xyz <x> <y> <z> (单位: mm)")
                    continue
                
                try:
                    x = float(parts[1]) / 1000.0
                    y = float(parts[2]) / 1000.0
                    z = float(parts[3]) / 1000.0
                    
                    print(f"移动到 ({x*1000:.1f}, {y*1000:.1f}, {z*1000:.1f}) mm...")
                    arm.move_to_xyz([x, y, z], duration=2.0)
                    print("完成")
                    
                except ValueError:
                    print("无效的数值")
            
            else:
                print(f"未知命令: {action}")
                print("输入 'help' 显示帮助")
        
        except KeyboardInterrupt:
            print("\n用户中断")
            running = False
        except Exception as e:
            print(f"错误: {e}")
    
    print("\n返回中立位置...")
    arm.move_to_neutral(duration=1.5)
    arm.disconnect()
    print("再见!")


if __name__ == "__main__":
    main()
