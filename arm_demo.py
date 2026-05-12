import pinocchio as pin
import numpy as np
from pinocchio.visualize import MeshcatVisualizer
import os
import time

print("=== SO-ARM100 机械臂自动演示 ===")

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

joint_names = [model.names[i] for i in range(1, len(model.names))]
print(f"机械臂关节: {joint_names}")

q = pin.neutral(model)
viz.display(q)

print("\n开始自动演示...")
print("机械臂将执行一系列预设动作")
print("按 Ctrl+C 停止")

try:
    while True:
        print("\n动作 1: 肩部旋转")
        for i in range(50):
            q[0] = 0.8 * np.sin(i * 0.1)
            viz.display(q)
            time.sleep(0.05)
        
        print("\n动作 2: 肘部弯曲")
        for i in range(50):
            q[2] = 0.5 + 0.5 * np.sin(i * 0.1)
            viz.display(q)
            time.sleep(0.05)
        
        print("\n动作 3: 腕部运动")
        for i in range(50):
            q[3] = 0.5 * np.sin(i * 0.1)
            q[4] = 0.3 * np.cos(i * 0.15)
            viz.display(q)
            time.sleep(0.05)
        
        print("\n动作 4: 夹爪开合")
        for i in range(30):
            q[5] = 1.745 * (i / 30)
            viz.display(q)
            time.sleep(0.05)
        time.sleep(1)
        for i in range(30):
            q[5] = 1.745 * (1 - i / 30)
            viz.display(q)
            time.sleep(0.05)
        
        print("\n动作 5: 回到初始位置")
        q = pin.neutral(model)
        viz.display(q)
        time.sleep(1)
        
except KeyboardInterrupt:
    print("\n演示结束")
