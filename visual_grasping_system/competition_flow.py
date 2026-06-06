"""
比赛自动化流程
=======
取货区(蓝色) → 装货区(红色方框) → 救援区(蓝色→红色方框)

运行: python competition_flow.py [COM端口]
"""

import sys
import os
import time
import yaml
import numpy as np
from typing import Optional, List

from soarm101_sdk_urdf import SOARM101Controller
from wrist_camera import WristCamera
from grasping_strategy import VisualServoGrasp
from placing_strategy import PlacingStrategy

CONFIG_DIR = os.path.dirname(__file__)
COMPETITION_CONFIG_PATH = os.path.join(CONFIG_DIR, 'competition_config.yaml')


def _load_comp_config():
    if os.path.exists(COMPETITION_CONFIG_PATH):
        with open(COMPETITION_CONFIG_PATH, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}
    return {}


def _save_comp_config(cfg):
    with open(COMPETITION_CONFIG_PATH, 'w', encoding='utf-8') as f:
        yaml.dump(cfg, f, default_flow_style=False)
    print(f"\n[OK] 配置已保存: {COMPETITION_CONFIG_PATH}")


def _load_sys_cfg():
    path = os.path.join(CONFIG_DIR, 'system_config.yaml')
    if os.path.exists(path):
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}
    return {}


class CompetitionFlow:
    """比赛自动化流程控制器"""

    def __init__(self, arm: SOARM101Controller, camera: WristCamera):
        self.arm = arm
        self.camera = camera

        self.cfg = _load_comp_config()
        self.pickup_poses: List[List[float]] = self.cfg.get('pickup_poses', [[0]*6]*3)
        self.loading_place_pose: List[float] = self.cfg.get('loading_place_pose', [0]*6)
        self.rescue_pickup_pose: List[float] = self.cfg.get('rescue_pickup_pose', [0]*6)
        self.rescue_place_pose: List[float] = self.cfg.get('rescue_place_pose', [0]*6)

        # 各区域独立的高度
        self.pickup_grasp_z = self.cfg.get('pickup_grasp_z', None)
        self.loading_place_z = self.cfg.get('loading_place_z', None)
        self.rescue_grasp_z = self.cfg.get('rescue_grasp_z', None)
        self.rescue_place_z = self.cfg.get('rescue_place_z', None)

        # 取货抓取实例 (蓝色，取货高度)
        self.pickup_grasp = VisualServoGrasp(arm, camera, target_color='blue',
                                             grasp_z=self.pickup_grasp_z)
        # 装货放置实例 (装货高度)
        self.loading_placer = PlacingStrategy(arm, camera,
                                              place_z=self.loading_place_z)
        # 救援抓取实例 (蓝色，救援抓取高度)
        self.rescue_grasp = VisualServoGrasp(arm, camera, target_color='blue',
                                             grasp_z=self.rescue_grasp_z)
        # 救援放置实例 (救援放置高度)
        self.rescue_placer = PlacingStrategy(arm, camera,
                                             place_z=self.rescue_place_z)

    # -------- 保存位置 --------

    def _save_current_as(self, key: str):
        ang = self.arm.get_joint_angles()
        if ang is None:
            print("[ERROR] 无法获取当前关节角度")
            return
        self.cfg[key] = [float(a) for a in ang]
        _save_comp_config(self.cfg)
        self._reload()
        print(f"[OK] 已保存 {key}: {[f'{a*57.3:.1f}deg' for a in ang]}")

    def save_pickup(self, idx: int):
        ang = self.arm.get_joint_angles()
        if ang is None:
            print("[ERROR] 无法获取当前关节角度")
            return
        i = idx - 1  # 1-based to 0-based
        while len(self.pickup_poses) <= i:
            self.pickup_poses.append([0.0]*6)
        self.pickup_poses[i] = [float(a) for a in ang]
        self.cfg['pickup_poses'] = self.pickup_poses
        _save_comp_config(self.cfg)
        self._reload()
        print(f"[OK] 已保存取货位置{idx}: {[f'{a*57.3:.1f}deg' for a in ang]}")

    def save_loading_place(self):
        self._save_current_as('loading_place_pose')

    def save_rescue_pickup(self):
        self._save_current_as('rescue_pickup_pose')

    def save_rescue_place(self):
        self._save_current_as('rescue_place_pose')

    def _reload(self):
        self.cfg = _load_comp_config()
        self.pickup_poses = self.cfg.get('pickup_poses', [[0]*6]*3)
        self.loading_place_pose = self.cfg.get('loading_place_pose', [0]*6)
        self.rescue_pickup_pose = self.cfg.get('rescue_pickup_pose', [0]*6)
        self.rescue_place_pose = self.cfg.get('rescue_place_pose', [0]*6)
        self.pickup_grasp_z = self.cfg.get('pickup_grasp_z', None)
        self.loading_place_z = self.cfg.get('loading_place_z', None)
        self.rescue_grasp_z = self.cfg.get('rescue_grasp_z', None)
        self.rescue_place_z = self.cfg.get('rescue_place_z', None)
        # 更新实例参数
        if self.pickup_grasp_z is not None:
            self.pickup_grasp.GRASP_Z = self.pickup_grasp_z
        if self.rescue_grasp_z is not None:
            self.rescue_grasp.GRASP_Z = self.rescue_grasp_z
        if self.loading_place_z is not None:
            self.loading_placer.PLACE_Z = self.loading_place_z
        if self.rescue_place_z is not None:
            self.rescue_placer.PLACE_Z = self.rescue_place_z

    # -------- 辅助: 移动到姿态(保持夹爪) --------

    def _go_to_pose_keep_gripper(self, pose: List[float], label: str):
        print(f"\n[{label}] 移动到预设姿态 (保持夹爪)...")
        current_angles = self.arm.get_joint_angles()
        if current_angles is not None:
            target = list(pose[:5]) + [current_angles[5]]
        else:
            target = list(pose)
        self.arm.set_joint_angles(target, duration=2.0)
        time.sleep(0.5)
        print(f"[{label}] [OK] 已到达")

    # -------- Phase 1: 取货区 --------

    def phase1_pickup(self) -> bool:
        """取货区: 依次访问3个位置，找到蓝色物块后执行抓取"""
        print("\n" + "=" * 60)
        print("[Phase 1] 取货区 - 从立体货架抓取蓝色物块")
        print("=" * 60)

        for i, pose in enumerate(self.pickup_poses):
            print(f"\n--- 取货位置 {i+1}/3 ---")
            self._go_to_pose_keep_gripper(pose, f"取货位置{i+1}")

            # 快速检测
            frame = self.camera.get_frame()
            if frame is None:
                print(f"  取货位置{i+1}: 摄像头未就绪")
                continue

            cube = self.camera.detect_blue_cube(frame)
            if cube is None:
                print(f"  取货位置{i+1}: 未检测到蓝色物块，尝试下一个...")
                continue

            print(f"  取货位置{i+1}: 检测到蓝色物块! 中心={cube['center']}")
            success = self.pickup_grasp.execute_grasp(show_display=True)
            if success:
                print(f"\n[Phase 1] [OK] 取货完成")
                return True
            else:
                print(f"  取货位置{i+1}: 抓取失败，尝试下一个...")

        print("\n[Phase 1] [FAIL] 所有位置均未找到蓝色物块")
        return False

    # -------- Phase 2: 装货区 --------

    def phase2_loading(self) -> bool:
        """装货区: 移动到装货位置，放置物块到红色方框"""
        print("\n" + "=" * 60)
        print("[Phase 2] 装货区 - 放置物块到红色方框")
        print("=" * 60)

        self._go_to_pose_keep_gripper(self.loading_place_pose, "装货区")

        return self.loading_placer.place_object(show_display=True)

    # -------- Phase 3: 救援区 --------

    def phase3_rescue(self) -> bool:
        """救援区: 抓取蓝色物块 → 放置到红色方框"""
        print("\n" + "=" * 60)
        print("[Phase 3] 救援区 - 抓取蓝色物块并放置到红色方框")
        print("=" * 60)

        # 3a: 前往救援抓取位置
        print("\n--- 救援区 抓取 ---")
        self._go_to_pose_keep_gripper(self.rescue_pickup_pose, "救援抓取")

        # 检测蓝色物块
        frame = self.camera.get_frame()
        if frame is None:
            print("[Phase 3] 摄像头未就绪")
            return False

        cube = self.camera.detect_blue_cube(frame)
        if cube is None:
            print("[Phase 3] 未检测到蓝色物块，尝试搜索...")
            if not self.rescue_grasp.search_for_object(show_display=True):
                print("[Phase 3] [FAIL] 搜索失败")
                return False

        success = self.rescue_grasp.execute_grasp(show_display=True)
        if not success:
            print("[Phase 3] [FAIL] 抓取失败")
            return False

        # 3b: 前往救援放置位置
        print("\n--- 救援区 放置 ---")
        self._go_to_pose_keep_gripper(self.rescue_place_pose, "救援放置")

        return self.rescue_placer.place_object(show_display=True)

    # -------- 完整流程 --------

    def run_full(self) -> bool:
        print("\n" + "=" * 60)
        print("比赛自动化流程 - 完整运行")
        print("=" * 60)

        if not self.phase1_pickup():
            return False
        if not self.phase2_loading():
            return False
        if not self.phase3_rescue():
            return False

        print("\n" + "=" * 60)
        print("[OK] 比赛自动化流程全部完成!")
        print("=" * 60)
        return True

    # -------- 显示当前配置 --------

    def show_config(self):
        print("\n" + "=" * 60)
        print("当前比赛配置")
        print("=" * 60)
        for i, pose in enumerate(self.pickup_poses):
            print(f"  取货位置{i+1}: {[f'{a*57.3:.1f}deg' for a in pose]}")
        print(f"  装货放置:     {[f'{a*57.3:.1f}deg' for a in self.loading_place_pose]}")
        print(f"  救援抓取:     {[f'{a*57.3:.1f}deg' for a in self.rescue_pickup_pose]}")
        print(f"  救援放置:     {[f'{a*57.3:.1f}deg' for a in self.rescue_place_pose]}")
        print()
        print(f"  取货Z高度:    {self.pickup_grasp_z*1000 if self.pickup_grasp_z else '默认'}mm")
        print(f"  装货Z高度:    {self.loading_place_z*1000 if self.loading_place_z else '默认'}mm")
        print(f"  救援抓取Z:    {self.rescue_grasp_z*1000 if self.rescue_grasp_z else '默认'}mm")
        print(f"  救援放置Z:    {self.rescue_place_z*1000 if self.rescue_place_z else '默认'}mm")


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
    print("比赛自动化流程")
    print("=" * 60)

    arm = SOARM101Controller(port, urdf_path=urdf_path)
    camera = WristCamera(camera_id=camera_id)

    if not arm.connect():
        print("无法连接机械臂")
        return

    if not camera.is_ready():
        print("警告: 摄像头未就绪")

    flow = CompetitionFlow(arm, camera)
    flow.show_config()

    print("\n" + "=" * 60)
    print("菜单选项")
    print("=" * 60)
    print("  保存位置:")
    print("    1. 保存当前姿态 → 取货区位置1")
    print("    2. 保存当前姿态 → 取货区位置2")
    print("    3. 保存当前姿态 → 取货区位置3")
    print("    4. 保存当前姿态 → 装货区放置位置")
    print("    5. 保存当前姿态 → 救援区抓取位置")
    print("    6. 保存当前姿态 → 救援区放置位置")
    print("  设置高度 (单位: 米):")
    print("    a. 设置取货区抓取Z高度")
    print("    b. 设置装货区放置Z高度")
    print("    c. 设置救援区抓取Z高度")
    print("    d. 设置救援区放置Z高度")
    print("  运行:")
    print("    7. 运行完整流程")
    print("    8. 仅运行取货区(Phase 1)")
    print("    9. 仅运行装货区(Phase 2)")
    print("    10. 仅运行救援区(Phase 3)")
    print("    c. 显示当前配置")
    print("    q. 退出")

    while True:
        cmd = input("\n请选择: ").strip().lower()
        if cmd == 'q':
            break
        elif cmd == '1':
            flow.save_pickup(1)
        elif cmd == '2':
            flow.save_pickup(2)
        elif cmd == '3':
            flow.save_pickup(3)
        elif cmd == '4':
            flow.save_loading_place()
        elif cmd == '5':
            flow.save_rescue_pickup()
        elif cmd == '6':
            flow.save_rescue_place()
        elif cmd == 'a':
            try:
                v = float(input("取货区抓取Z高度(米): ").strip())
                flow.cfg['pickup_grasp_z'] = v
                _save_comp_config(flow.cfg)
                flow._reload()
            except ValueError:
                print("[ERROR] 请输入有效数字")
        elif cmd == 'b':
            try:
                v = float(input("装货区放置Z高度(米): ").strip())
                flow.cfg['loading_place_z'] = v
                _save_comp_config(flow.cfg)
                flow._reload()
            except ValueError:
                print("[ERROR] 请输入有效数字")
        elif cmd == 'c':
            try:
                v = float(input("救援区抓取Z高度(米): ").strip())
                flow.cfg['rescue_grasp_z'] = v
                _save_comp_config(flow.cfg)
                flow._reload()
            except ValueError:
                print("[ERROR] 请输入有效数字")
        elif cmd == 'd':
            try:
                v = float(input("救援区放置Z高度(米): ").strip())
                flow.cfg['rescue_place_z'] = v
                _save_comp_config(flow.cfg)
                flow._reload()
            except ValueError:
                print("[ERROR] 请输入有效数字")
        elif cmd == '7':
            flow.run_full()
        elif cmd == '8':
            flow.phase1_pickup()
        elif cmd == '9':
            flow.phase2_loading()
        elif cmd == '10':
            flow.phase3_rescue()
        elif cmd == 'c':
            flow.show_config()
        else:
            print("未知命令")

    camera.release()
    arm.disconnect()
    print("\n退出")


if __name__ == "__main__":
    main()