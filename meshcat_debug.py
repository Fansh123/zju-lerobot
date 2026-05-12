import sys
print("Python version:", sys.version)
print("Python executable:", sys.executable)

try:
    print("\nStep 1: Importing meshcat...")
    import meshcat
    print("  SUCCESS: meshcat imported")
except Exception as e:
    print(f"  FAILED: {type(e).__name__}: {e}")
    sys.exit(1)

try:
    print("\nStep 2: Creating Visualizer...")
    viz = meshcat.Visualizer()
    print("  SUCCESS: Visualizer created")
except Exception as e:
    print(f"  FAILED: {type(e).__name__}: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

try:
    print("\nStep 3: Opening viewer...")
    viz.open()
    print("  SUCCESS: Viewer opened")
except Exception as e:
    print(f"  FAILED: {type(e).__name__}: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

try:
    print("\nStep 4: Adding box geometry...")
    import meshcat.geometry as g
    box = g.Box([0.1, 0.1, 0.1])
    print("  SUCCESS: Box geometry created")
except Exception as e:
    print(f"  FAILED: {type(e).__name__}: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

try:
    print("\nStep 5: Setting object in scene...")
    viz['test/box'].set_object(box)
    print("  SUCCESS: Object set in scene")
except Exception as e:
    print(f"  FAILED: {type(e).__name__}: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "="*60)
print("ALL TESTS PASSED!")
print("Meshcat server should be running at: http://127.0.0.1:7000")
print("="*60)

print("\nKeeping process alive for 30 seconds...")
import time
time.sleep(30)

print("Done!")
