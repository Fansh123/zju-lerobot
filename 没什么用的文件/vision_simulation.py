import numpy as np
import time
import meshcat.geometry as g
import meshcat.transformations as tf


class VisionSimulator:
    def __init__(self, target_position=(0.3, 0.0, 0.15)):
        self.target_position = np.array(target_position)
        self.detected = False
        self.detection_delay = 2.0
        self.start_time = None

    def start_detection(self, timeout=15):
        print("="*60)
        print("视觉识别模拟模式")
        print("="*60)
        print(f"目标位置: ({self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f})")
        print(f"模拟检测延迟: {self.detection_delay}秒")
        print("="*60)
        self.start_time = time.time()
        self.detected = False
        return True

    def get_detection_result(self, frame=None):
        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time

        if not self.detected and elapsed >= self.detection_delay:
            self.detected = True
            cx, cy = self._position_to_image_coords()
            area = 2500
            print(f"\n检测到红色方块! 模拟位置: ({cx}, {cy}), 面积: {area}")
            return cx, cy, area, True

        remaining = self.detection_delay - elapsed
        if remaining > 0:
            print(f"\r检测中... ({elapsed:.1f}s / {self.detection_delay}s)", end='', flush=True)

        return None, None, 0, False

    def _position_to_image_coords(self):
        x, y, z = self.target_position
        frame_width, frame_height = 640, 480
        scale_x = frame_width / 0.6
        scale_y = frame_height / 0.6
        cx = int((x + 0.3) * scale_x)
        cy = int((0.3 - y) * scale_y)
        cx = np.clip(cx, 50, frame_width - 50)
        cy = np.clip(cy, 50, frame_height - 50)
        return cx, cy

    def wait_for_detection(self, timeout=15):
        start_time = time.time()
        while True:
            cx, cy, area, detected = self.get_detection_result()
            elapsed = time.time() - start_time

            if detected:
                return cx, cy, True

            if elapsed > timeout:
                print(f"\n检测超时 ({timeout}s)")
                return None, None, False

            time.sleep(0.1)


class GripperObject:
    def __init__(self, visualizer, name="red_block"):
        self.vis = visualizer
        self.name = name
        self.position = None
        self.attached = False
        self.mesh_created = False

    def spawn(self, position, size=0.03):
        self.position = np.array(position)
        self.vis[self.name].set_object(
            g.Box([size, size, size]),
            g.MeshLambertMaterial(color=0xff0000, opacity=1.0)
        )
        self._update_transform()
        self.mesh_created = True
        print(f"红色方块已生成: 位置({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}), 大小{size}")

    def _update_transform(self):
        if self.position is None:
            return
        T = tf.translation_matrix(self.position)
        self.vis[self.name].set_transform(T)

    def update_position(self, new_position):
        if self.position is None or not self.mesh_created:
            return
        self.position = np.array(new_position)
        self._update_transform()

    def attach_to_gripper(self, gripper_position):
        self.attached = True
        self.position = np.array(gripper_position)
        self._update_transform()
        print(f"物块已附着到夹爪: ({gripper_position[0]:.3f}, {gripper_position[1]:.3f}, {gripper_position[2]:.3f})")

    def detach(self, drop_position):
        self.attached = False
        self.position = np.array(drop_position)
        self._update_transform()
        print(f"物块已放下: ({drop_position[0]:.3f}, {drop_position[1]:.3f}, {drop_position[2]:.3f})")

    def remove(self):
        self.position = None
        self.attached = False
        self.mesh_created = False