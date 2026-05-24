#!/usr/bin/env python3
"""
SO-ARM100 快速测试脚本
无需 ROS2，直接串口控制
"""

from soarm100_sdk import SOARM100
import time

def main():
    print("="*60)
    print("SO-ARM100 独立控制测试")
    print("="*60)
    
    # 设置串口端口
    port = input("请输入串口端口 (默认 COM3): ").strip()
    if not port:
        port = 'COM3'
    
    # 创建控制器
    arm = SOARM100(port)
    
    # 连接机械臂
    if not arm.connect():
        print(f"无法连接到 {port}")
        print("请检查:")
        print("  1. 串口端口是否正确")
        print("  2. 机械臂是否已通电")
        print("  3. USB线是否连接")
        return
    
    print("\n连接成功!")
    print("="*60)
    
    try:
        while True:
            print("\n菜单:")
            print("1. 移动到中立位置")
            print("2. 肩部旋转测试")
            print("3. 夹爪测试")
            print("4. 完整抓取演示")
            print("5. 退出")
            
            choice = input("\n请选择操作 (1-5): ")
            
            if choice == '1':
                arm.move_to_neutral()
            
            elif choice == '2':
                print("向右旋转...")
                arm.set_joint_angles([0.785, 0.0, 0.0, 0.0, 0.0, 2.0], duration=1.0)
                time.sleep(1)
                print("向左旋转...")
                arm.set_joint_angles([-0.785, 0.0, 0.0, 0.0, 0.0, 2.0], duration=1.0)
                time.sleep(1)
                arm.move_to_neutral()
            
            elif choice == '3':
                print("闭合夹爪...")
                arm.grasp()
                time.sleep(1)
                print("打开夹爪...")
                arm.release()
            
            elif choice == '4':
                print("\n演示流程:")
                print("1. 移动到抓取位置")
                arm.set_joint_angles([0.0, 1.0, -1.5, 0.5, 0.0, 2.0], duration=1.5)
                
                print("2. 闭合夹爪")
                arm.grasp()
                time.sleep(1)
                
                print("3. 肩部旋转90度")
                arm.set_joint_angles([1.57, 1.0, -1.5, 0.5, 0.0, 0.0], duration=2.0)
                
                print("4. 等待5秒")
                time.sleep(5)
                
                print("5. 打开夹爪")
                arm.release()
                time.sleep(1)
                
                print("6. 返回中立位置")
                arm.move_to_neutral()
            
            elif choice == '5':
                print("退出程序")
                break
            
            else:
                print("无效选择")
    
    except KeyboardInterrupt:
        print("\n\n用户中断")
        arm.move_to_neutral()
    
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()