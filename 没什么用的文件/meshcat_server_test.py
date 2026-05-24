import meshcat
import meshcat.geometry as g
import numpy as np
import time

print("=== Meshcat 服务器测试（持续运行）===")

print("Creating Meshcat server...")
viz = meshcat.Visualizer()
print("Server created")

print("Opening viewer in browser...")
viz.open()
print("Viewer should be open now")
print("URL: http://127.0.0.1:7000/static/")

print("\nAdding objects to the scene...")

box_geom = g.Box([0.1, 0.1, 0.1])
viz['meshcat/box'].set_object(box_geom, g.MeshPhongMaterial(color=0xff0000))
print("  - Red box added")

sphere_geom = g.Sphere(0.05)
viz['meshcat/sphere'].set_object(sphere_geom, g.MeshPhongMaterial(color=0x0000ff))
print("  - Blue sphere added")

cylinder_geom = g.Cylinder(0.03, 0.15)
viz['meshcat/cylinder'].set_object(cylinder_geom, g.MeshPhongMaterial(color=0x00ff00))
print("  - Green cylinder added")

print("\nApplying transforms...")
transform = np.array([
    [1, 0, 0, 0.2],
    [0, 1, 0, 0],
    [0, 0, 1, 0.3],
    [0, 0, 0, 1]
])
viz['meshcat/box'].set_transform(transform)
print("  - Box transformed")

print("\n" + "="*60)
print("Meshcat 服务器已启动！")
print("请在浏览器中访问: http://127.0.0.1:7000/static/")
print("你应该能看到：")
print("  - 红色方块")
print("  - 蓝色球体")
print("  - 绿色圆柱体")
print("="*60)

print("\n保持服务器运行 60 秒...")
for i in range(60):
    time.sleep(1)
    if i % 10 == 0:
        print(f"  已运行 {i+1}/60 秒")
    
print("\n测试完成！")
