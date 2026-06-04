"""
测试脚本：抓取蓝色物块 → 移动到放置起始姿态 → 放置到红色方框
运行: python test_blue_grasp_place.py [COM端口]
"""

import sys
import os
import time
import yaml
import numpy as np

from soarm101_sdk_urdf import SOARM101Controller
from wrist_camera import WristCamera
from grasping_strategy import VisualServoGrasp
from placing_strategy import PlacingStrategy

CONFIG_DIR = os.path.dirname(__file__)


def _load_sys_cfg():
    path = os.path.join(CONFIG_DIR, 'system_config.yaml')
    if os.path.exists(path):
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}
    return {}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None

    sys_cfg = _load_sys_cfg()
    arm_cfg = sys_cfg.get('arm', {})
    cam_cfg = sys_cfg.get('camera', {})

    if port is None:
        port = arm_cfg.get('port', 'COM18')

    urdf_rel = arm_cfg.get('urdf_path', '../SO-ARM100/Simulation/SO101/so101_new_calib.urdf')
    urdf_path = os.path.join(CONFIG_DIR, urdf_rel)
    if not os.path.exists(urdf_path):
        urdf_path = None
        print("[WARN] URDF未找到")

    camera_id = cam_cfg.get('camera_id', 1)

    print("=" * 60)
    print("蓝色物块抓取 → 起始姿态 → 红色方框放置 测试")
    print("=" * 60)

    arm = SOARM101Controller(port, urdf_path=urdf_path)
    camera = WristCamera(camera_id=camera_id)

    if not arm.connect():
        print("无法连接机械臂")
        return

    if not camera.is_ready():
        print("警告: 摄像头未就绪")

    # 蓝色抓取
    blue_grasp = VisualServoGrasp(arm, camera, target_color='blue')
    # 红色放置 (复用同一 camera，放置策略始终检测红色方框)
    placing = PlacingStrategy(arm, camera)

    print("\n" + "=" * 60)
    print("[Phase 1] 抓取蓝色物块")
    print("=" * 60)

    success = blue_grasp.execute_grasp(show_display=True)
    if not success:
        print("\n抓取蓝色物块失败")
        camera.release()
        arm.disconnect()
        return

    print("\n" + "=" * 60)
    print("[Phase 2] 移动到放置起始姿态")
    print("=" * 60)

    placing.go_to_start_pose()

    print("\n" + "=" * 60)
    print("[Phase 3] 放置物块到红色方框")
    print("=" * 60)

    placing.place_object(show_display=True)

    camera.release()
    arm.disconnect()
    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)


if __name__ == "__main__":
    main()
