import meshcat
import meshcat.geometry as g
import numpy as np

print("=== Meshcat 服务器测试 ===")

print("Creating Meshcat server...")
viz = meshcat.Visualizer()
print("Server created")

print("Opening viewer in browser...")
viz.open()
print("Viewer should be open now")

print("\nAdding a red box to the scene...")
box_geom = g.Box([0.1, 0.1, 0.1])
viz['meshcat/box'].set_object(box_geom, g.MeshPhongMaterial(color=0xff0000))
print("Red box added!")

print("\nAdding a blue sphere...")
sphere_geom = g.Sphere(0.05)
viz['meshcat/sphere'].set_object(sphere_geom, g.MeshPhongMaterial(color=0x0000ff))
print("Blue sphere added!")

print("\nAdding a green cylinder...")
cylinder_geom = g.Cylinder(0.03, 0.15)
viz['meshcat/cylinder'].set_object(cylinder_geom, g.MeshPhongMaterial(color=0x00ff00))
print("Green cylinder added!")

print("\nTransforming the box...")
transform = np.array([
    [1, 0, 0, 0.2],
    [0, 1, 0, 0],
    [0, 0, 1, 0.3],
    [0, 0, 0, 1]
])
viz['meshcat/box'].set_transform(transform)
print("Box transformed!")

print("\n" + "="*50)
print("Meshcat test complete!")
print("Please check your browser at: http://127.0.0.1:7000/static/")
print("You should see:")
print("  - A red box")
print("  - A blue sphere")
print("  - A green cylinder")
print("="*50)

input("Press Enter to exit...")