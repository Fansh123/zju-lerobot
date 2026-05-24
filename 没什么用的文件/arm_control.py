import pinocchio as pin
import numpy as np
from pinocchio.visualize import MeshcatVisualizer
import os
import time

print("=== SO-ARM100 机械臂交互式控制 ===")

current_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100", "so100.urdf")
mesh_dir = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100")

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_path,
    package_dirs=[mesh_dir]
)

viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()

q = pin.neutral(model)
viz.display(q)

joint_names = [model.names[i] for i in range(1, len(model.names))]
print(f"\n机械臂关节: {joint_names}")
print("关节范围:")
for i, name in enumerate(joint_names):
    print(f"  {i}: {name}  [{model.lowerPositionLimit[i]:.2f}, {model.upperPositionLimit[i]:.2f}]")

print("\n" + "="*60)
print("交互式控制说明:")
print("="*60)
print("键盘操作:")
print("  0-5: 选择关节 (对应 shoulder_pan 到 gripper)")
print("  w/s: 增加/减少选中关节的角度")
print("  q: 退出程序")
print("  r: 重置到初始位置")
print("  p: 打印当前关节角度")
print("\n角度增量: 0.1 弧度 (~5.7度)")
print("="*60)

selected_joint = 0
step_size = 0.1

print(f"\n当前选中关节: {selected_joint} ({joint_names[selected_joint]})")
print("开始控制... (按 'q' 退出)")

try:
    import sys
    import tty
    import termios
    
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setraw(sys.stdin.fileno())
        
        while True:
            key = sys.stdin.read(1)
            
            if key == 'q':
                print("\n退出程序...")
                break
            elif key == 'r':
                q = pin.neutral(model)
                viz.display(q)
                print("\n已重置到初始位置")
            elif key == 'p':
                print("\n当前关节角度:")
                for i, name in enumerate(joint_names):
                    print(f"  {name}: {q[i]:.4f}")
            elif key in '012345':
                selected_joint = int(key)
                print(f"\n选中关节: {selected_joint} ({joint_names[selected_joint]})")
            elif key == 'w':
                q[selected_joint] = min(q[selected_joint] + step_size, model.upperPositionLimit[selected_joint])
                viz.display(q)
                print(f"\r{joint_names[selected_joint]}: {q[selected_joint]:.4f}", end='')
            elif key == 's':
                q[selected_joint] = max(q[selected_joint] - step_size, model.lowerPositionLimit[selected_joint])
                viz.display(q)
                print(f"\r{joint_names[selected_joint]}: {q[selected_joint]:.4f}", end='')
                
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        
except ImportError:
    print("警告: 无法导入 tty/termios 模块，使用简化控制模式")
    print("按 Enter 继续下一步...")
    
    while True:
        print(f"\n当前选中关节: {selected_joint} ({joint_names[selected_joint]})")
        print(f"当前角度: {q[selected_joint]:.4f}")
        action = input("输入操作 (0-5选择关节, w增加, s减少, r重置, q退出): ")
        
        if action == 'q':
            print("退出程序...")
            break
        elif action == 'r':
            q = pin.neutral(model)
            viz.display(q)
            print("已重置到初始位置")
        elif action in '012345':
            selected_joint = int(action)
        elif action == 'w':
            q[selected_joint] = min(q[selected_joint] + step_size, model.upperPositionLimit[selected_joint])
            viz.display(q)
        elif action == 's':
            q[selected_joint] = max(q[selected_joint] - step_size, model.lowerPositionLimit[selected_joint])
            viz.display(q)

print("\n程序结束")
