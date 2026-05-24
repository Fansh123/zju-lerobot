
import pinocchio as pin
import meshcat
import os

print("=== SO-ARM100 机械臂测试 ===")

current_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100", "so100.urdf")
mesh_dir = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100")

print(f"URDF路径: {urdf_path}")
print(f"网格目录: {mesh_dir}")

try:
    # 加载模型
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_path,
        package_dirs=[mesh_dir]
    )
    print("\n✓ 模型加载成功!")
    print(f"  关节数量: {model.nq}")
    print(f"  自由度: {model.nv}")
    
    # 创建视觉器
    from pinocchio.visualize import MeshcatVisualizer
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    
    # 显示初始位置
    q = pin.neutral(model)
    viz.display(q)
    
    print("\n✓ 可视化器已启动!")
    print("  浏览器应该自动打开 Meshcat 窗口")
    print("  请访问: http://127.0.0.1:7000/static/")
    
    # 简单运动测试
    print("\n=== 执行简单运动测试 ===")
    import time
    
    for i in range(50):
        q[0] = 0.5 * np.sin(i * 0.1)
        q[1] = -0.3 * np.sin(i * 0.15)
        viz.display(q)
        time.sleep(0.05)
    
    print("\n✓ 测试完成!")
    input("按回车键退出...")
    
except Exception as e:
    print(f"\n✗ 错误: {type(e).__name__}: {e}")
    import traceback
    traceback.print_exc()
