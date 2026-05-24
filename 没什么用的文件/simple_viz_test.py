
import pinocchio as pin
import numpy as np
from pinocchio.visualize import MeshcatVisualizer
import os
import time

print("=== 简单机械臂可视化测试 ===")

current_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100", "so100.urdf")
mesh_dir = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100")

print("Loading model...")
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_path,
    package_dirs=[mesh_dir]
)

print(f"Model joints: {model.nq}")
print(f"Joint names: {[model.names[i] for i in range(1, len(model.names))]}")

print("\nInitializing MeshcatVisualizer...")
viz = MeshcatVisualizer(model, collision_model, visual_model)

try:
    viz.initViewer(open=True)
    print("Viewer initialized")
except Exception as e:
    print(f"Error initializing viewer: {e}")
    import traceback
    traceback.print_exc()

print("Loading model to viewer...")
viz.loadViewerModel()
print("Model loaded to viewer")

print("\nDisplaying neutral position...")
q = pin.neutral(model)
viz.display(q)

print("\nWaiting 5 seconds to see if visualization appears...")
print("If you see the robot in the Meshcat window, the issue might be in the main simulation.")
print("If not, there might be a visualization configuration issue.")

for i in range(10):
    q[0] = 0.5 * np.sin(i * 0.3)
    viz.display(q)
    time.sleep(0.5)
    print(f"Step {i+1}/10 - joint 0 = {q[0]:.3f}")

print("\nTest complete!")
input("Press Enter to exit...")
