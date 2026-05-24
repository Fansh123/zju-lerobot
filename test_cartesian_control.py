"""
交互式笛卡尔空间控制测试
========================
使用键盘控制机械臂在笛卡尔空间中移动

命令:
  q/e - 前/后移动
  w/s - 右/左移动
  a/d - 下/上移动
  p   - 打印当前位置
  h   - 回到中立位置
  x   - 退出
"""

import sys
sys.path.insert(0, '.')

from visual_grasping_system.soarm101_sdk import SOARM101Controller
import time


def clear_screen():
    print("\033[2J\033[H", end="")


def print_help():
    print("\n" + "="*50)
    print("交互式笛卡尔空间控制")
    print("="*50)
    print("\n命令:")
    print("  q/e - 前/后移动 (当前步长)")
    print("  w/s - 右/左移动 (当前步长)")
    print("  a/d - 下/上移动 (当前步长)")
    print("  r/f - 快速前/后 (5cm)")
    print("  t/b - 快速下/上 (5cm)")
    print("  o/c - 夹爪开/合")
    print("  p   - 打印当前位置")
    print("  h   - 回到中立位置")
    print("  m   - 修改步长")
    print("  g   - 移动到指定坐标")
    print("  ?   - 显示帮助")
    print("  x   - 退出")
    print("="*50)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    step_size = 0.02  # 默认步长 2cm
    
    print("="*50)
    print("SO-ARM101 交互式笛卡尔控制")
    print("="*50)
    print(f"\n连接到 {port}...")
    
    arm = SOARM101Controller(port)
    
    if not arm.connect():
        print("连接失败!")
        return
    
    print("\n扫描舵机...")
    arm.scan_servos()
    
    print("\n移动到中立位置...")
    arm.move_to_neutral(duration=1.5)
    time.sleep(0.5)
    
    print_help()
    
    pos = arm.get_current_xyz()
    print(f"\n当前末端位置: x={pos[0]*1000:.1f}mm, y={pos[1]*1000:.1f}mm, z={pos[2]*1000:.1f}mm")
    print(f"步长: {step_size*1000:.0f}mm")
    print("\n输入命令 (按回车确认): ")
    
    running = True
    while running:
        try:
            cmd = input("> ").strip().lower()
            
            if not cmd:
                continue
            
            if cmd == 'x':
                print("\n退出...")
                running = False
                
            elif cmd == 'w':
                print(f"向前移动 {step_size*1000:.0f}mm...")
                arm.move_forward(step_size, duration=1.0)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 's':
                print(f"向后移动 {step_size*1000:.0f}mm...")
                arm.move_backward(step_size, duration=1.0)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 'a':
                print(f"向左移动 {step_size*1000:.0f}mm...")
                arm.move_left(step_size, duration=1.0)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 'd':
                print(f"向右移动 {step_size*1000:.0f}mm...")
                arm.move_right(step_size, duration=1.0)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 'q':
                print(f"向上移动 {step_size*1000:.0f}mm...")
                arm.move_up(step_size, duration=1.0)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 'e':
                print(f"向下移动 {step_size*1000:.0f}mm...")
                arm.move_down(step_size, duration=1.0)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 'r':
                print("快速向前 5cm...")
                arm.move_forward(0.05, duration=1.5)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 'f':
                print("快速向后 5cm...")
                arm.move_backward(0.05, duration=1.5)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 't':
                print("快速向上 5cm...")
                arm.move_up(0.05, duration=1.5)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 'b':
                print("快速向下 5cm...")
                arm.move_down(0.05, duration=1.5)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 'o':
                print("夹爪打开...")
                arm.release(duration=0.8)
                
            elif cmd == 'c':
                print("夹爪闭合...")
                arm.grasp(duration=0.8)
                
            elif cmd == 'p':
                pos = arm.get_current_xyz()
                print(f"\n当前末端位置:")
                print(f"  X: {pos[0]*1000:.1f} mm")
                print(f"  Y: {pos[1]*1000:.1f} mm")
                print(f"  Z: {pos[2]*1000:.1f} mm")
                
            elif cmd == 'h':
                print("\n回到中立位置...")
                arm.move_to_neutral(duration=1.5)
                pos = arm.get_current_xyz()
                print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                
            elif cmd == 'm':
                try:
                    new_step = float(input("输入新步长 (mm): "))
                    step_size = new_step / 1000.0
                    print(f"步长已设置为: {step_size*1000:.0f}mm")
                except ValueError:
                    print("无效输入")
                    
            elif cmd == 'g':
                try:
                    print("\n输入目标坐标 (单位: mm)")
                    x = float(input("X: ")) / 1000.0
                    y = float(input("Y: ")) / 1000.0
                    z = float(input("Z: ")) / 1000.0
                    print(f"\n移动到 ({x*1000:.1f}, {y*1000:.1f}, {z*1000:.1f}) mm...")
                    arm.move_to_xyz([x, y, z], duration=2.0)
                    pos = arm.get_current_xyz()
                    print(f"当前位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
                except ValueError:
                    print("无效输入")
                    
            elif cmd == '?':
                print_help()
                
            else:
                print(f"未知命令: {cmd}")
                print("输入 '?' 显示帮助")
                
        except KeyboardInterrupt:
            print("\n\n用户中断")
            running = False
        except Exception as e:
            print(f"错误: {e}")
    
    print("\n断开连接...")
    arm.disconnect()
    print("再见!")


if __name__ == "__main__":
    main()
