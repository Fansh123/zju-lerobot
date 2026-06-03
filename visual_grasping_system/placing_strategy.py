"""
放置策略模块 - 视觉伺服版
摄像头向下看桌面，通过线延长检测方框中心，
先左右居中，再向前平移至中心位置
"""

import numpy as np
import time
import os
import yaml
from typing import Optional, Dict, Tuple
from wrist_camera import WristCamera
from soarm101_sdk_urdf import SOARM101Controller


CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'placing_config.yaml')


def _load_config():
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, 'r') as f:
            return yaml.safe_load(f) or {}
    return {}


def _save_config(cfg):
    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False)
    print(f"✓ 配置已保存: {CONFIG_PATH}")


class PlacingStrategy:
    """视觉伺服放置策略"""

    CENTER_THRESHOLD_Y = 8

    MAX_ITER = 60
    JOINT_STEP = 0.025
    FORWARD_COEFFICIENT = 0.0015

    PLACE_Z = 0.015
    RETRACT_Z = 0.12

    GRIPPER_OPEN = 1.1

    def __init__(self, arm: SOARM101Controller, camera: WristCamera = None,
                 camera_id: int = 0):
        self.arm = arm
        self.camera = camera if camera else WristCamera(camera_id)

        self.img_cx = 320
        self.img_cy = 240

        self._load_placing_config()

    def _load_placing_config(self):
        cfg = _load_config()
        self.start_pose = cfg.get('start_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.87])
        self.FORWARD_COEFFICIENT = cfg.get('forward_coefficient', self.FORWARD_COEFFICIENT)
        self.center_offset_rad = np.deg2rad(cfg.get('center_offset_deg', 3.0))
        print(f"[配置] 起始姿态: {[f'{a*57.3:.1f}°' for a in self.start_pose]}")
        print(f"[配置] 前进系数: {self.FORWARD_COEFFICIENT*1000:.1f}mm/px")
        print(f"[配置] 居中后偏移: {np.rad2deg(self.center_offset_rad):.1f}°")

    def save_current_as_start_pose(self):
        ang = self.arm.get_joint_angles()
        if ang is None:
            print("✗ 无法获取当前关节角度")
            return False
        cfg = _load_config()
        cfg['start_pose'] = [float(a) for a in ang]
        _save_config(cfg)
        self.start_pose = cfg['start_pose']
        print(f"\n✓ 已保存当前姿态为起始姿态:")
        print(f"  {[f'{a*57.3:.1f}°' for a in self.start_pose]}")
        return True

    def set_center_offset(self, deg: float):
        cfg = _load_config()
        cfg['center_offset_deg'] = float(deg)
        _save_config(cfg)
        self.center_offset_rad = np.deg2rad(deg)
        print(f"\n✓ 居中后偏移已更新: {deg:.1f}°")
        return True

    def go_to_start_pose(self):
        print(f"\n[起始姿态] 移动到保存的起始位置...")
        self.arm.set_joint_angles(self.start_pose, duration=2.0)
        time.sleep(0.5)
        print("[起始姿态] ✓ 已到达起始位置")

    def camera_debug_preview(self):
        print("\n[调试] 摄像头预览 - 按 'q' 退出, 按 's' 保存当前帧")
        print("  显示: 原始画面 | 红色mask | 边缘检测")
        wname = "Camera Debug"
        self.camera.cv2.namedWindow(wname)

        while True:
            frame = self.camera.get_frame()
            if frame is None:
                continue

            hsv = self.camera.cv2.cvtColor(frame, self.camera.cv2.COLOR_BGR2HSV)
            mask1 = self.camera.cv2.inRange(hsv, self.camera.red_hsv_low1, self.camera.red_hsv_high1)
            mask2 = self.camera.cv2.inRange(hsv, self.camera.red_hsv_low2, self.camera.red_hsv_high2)
            mask = self.camera.cv2.bitwise_or(mask1, mask2)

            k3 = np.ones((3, 3), np.uint8)
            mask_closed = self.camera.cv2.morphologyEx(mask, self.camera.cv2.MORPH_CLOSE, k3, iterations=2)
            edges_disp = self.camera.cv2.Canny(mask_closed, 20, 60)

            r = self.camera.detect_red_frame_rect(frame)
            r2 = self.camera.detect_red_frame_lines(frame, min_line_len=8, max_line_gap=10)
            r3 = self.camera.detect_red_frame(frame)

            overlay = frame.copy()
            self.camera.cv2.line(overlay, (self.img_cx, 0), (self.img_cx, 480), (0, 255, 0), 1)
            self.camera.cv2.line(overlay, (0, self.img_cy), (640, self.img_cy), (0, 255, 0), 1)

            if r is not None:
                overlay = self.camera.draw_frame_rect(overlay, r)
                cx, cy = r['center']
                self.camera.cv2.putText(overlay, f"Rect OK ({cx},{cy})", (10, 25),
                                        self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            elif r2 is not None:
                overlay = self.camera.draw_frame_lines(overlay, r2)
                cx, cy = r2['center']
                self.camera.cv2.putText(overlay, f"Hough OK ({cx},{cy})", (10, 25),
                                        self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            elif r3 is not None:
                overlay = self.camera.draw_frame_detection(overlay, r3)
                cx, cy = r3['center']
                self.camera.cv2.putText(overlay, f"Contour OK ({cx},{cy})", (10, 25),
                                        self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                self.camera.cv2.putText(overlay, "NO DETECTION", (10, 25),
                                        self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            mask_color = self.camera.cv2.cvtColor(mask, self.camera.cv2.COLOR_GRAY2BGR)
            edges_color = self.camera.cv2.cvtColor(edges_disp, self.camera.cv2.COLOR_GRAY2BGR)

            h_overlay = self.camera.cv2.resize(overlay, (320, 240))
            h_mask = self.camera.cv2.resize(mask_color, (320, 240))
            h_edges = self.camera.cv2.resize(edges_color, (320, 240))

            top = np.hstack([h_overlay, h_mask])
            bot = np.hstack([h_edges, np.zeros_like(h_edges)])
            panel = np.vstack([top, bot])

            self.camera.cv2.putText(panel, "Camera", (5, 15), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            self.camera.cv2.putText(panel, "Red Mask", (325, 15), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            self.camera.cv2.putText(panel, "Edges", (5, 255), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.camera.cv2.imshow(wname, panel)
            key = self.camera.cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                import os
                os.makedirs('debug_images', exist_ok=True)
                n = len(os.listdir('debug_images'))
                self.camera.cv2.imwrite(f'debug_images/frame_{n}.png', panel)
                print(f"  保存: debug_images/frame_{n}.png")

        self.camera.cv2.destroyWindow(wname)

    def detect_frame_center(self, frame: np.ndarray = None) -> Optional[Tuple[int, int]]:
        r = self.camera.detect_red_frame_lines(frame)
        if r is not None:
            return r['center']
        r2 = self.camera.detect_red_frame(frame)
        if r2 is not None:
            return r2['center']
        return None

    def _detect_with_fallback(self, frame):
        r = self.camera.detect_red_frame_rect(frame)
        if r is not None:
            return r
        r2 = self.camera.detect_red_frame_lines(frame, min_line_len=8, max_line_gap=10)
        if r2 is not None:
            return r2
        r3 = self.camera.detect_red_frame(frame)
        if r3 is not None:
            return {
                'center': r3['center'],
                'corners': None,
                'diags': [],
                'bounds': []
            }
        return None

    def servo_frame_center(self, show_display: bool = True) -> bool:
        print("\n" + "=" * 60)
        print("[放置伺服] 方框居中流程")
        print("=" * 60)

        wname = "Frame Servo - Press 'q' to abort"
        if show_display:
            self.camera.cv2.namedWindow(wname)

        try:
            # --- Phase 1: Y-axis centering (same as grasping)
            print(f"\n[Phase1] Y轴居中 (关节0), 目标Y={self.img_cy}...")
            for i in range(self.MAX_ITER):
                frame = self.camera.get_frame()
                if frame is None:
                    continue

                r = self._detect_with_fallback(frame)
                if r is None:
                    print(f"  [{i+1}] 未检测到方框")
                    disp = frame.copy()
                    self.camera.cv2.putText(disp, "No frame", (10, 30),
                                            self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    if show_display:
                        self.camera.cv2.imshow(wname, disp)
                        if self.camera.cv2.waitKey(30) & 0xFF == ord('q'):
                            return False
                    continue

                cx, cy = r['center']
                if r['corners'] is not None and len(r['corners']) == 4:
                    disp = self.camera.draw_frame_rect(frame, r)
                else:
                    disp = frame.copy()
                    self.camera.cv2.circle(disp, (cx, cy), 12, (0, 0, 255), 2)
                    self.camera.cv2.circle(disp, (cx, cy), 5, (0, 0, 255), -1)
                self.camera.cv2.line(disp, (0, self.img_cy), (640, self.img_cy), (0, 255, 0), 1)

                error_y = cy - self.img_cy
                if error_y < 0:
                    direction = "UP -> Turn RIGHT"
                else:
                    direction = "UP -> Turn LEFT" if error_y > 0 else "CENTERED"

                self.camera.cv2.putText(disp, f"Y err={error_y}px ({direction})", (10, 30),
                                        self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                if abs(error_y) < self.CENTER_THRESHOLD_Y:
                    print(f"[Phase1] ✓ Y轴居中完成 (误差={error_y}px)")
                    self.camera.cv2.putText(disp, "Y CENTERED", (10, 60),
                                            self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    if show_display:
                        self.camera.cv2.imshow(wname, disp)
                        self.camera.cv2.waitKey(300)
                    break

                print(f"  [{i+1}] Y偏差: {error_y}px ({direction})")
                ang = self.arm.get_joint_angles()
                if ang is not None:
                    if error_y < 0:
                        ang[0] += self.JOINT_STEP
                    else:
                        ang[0] -= self.JOINT_STEP
                    ang[0] = np.clip(ang[0], -1.5, 1.5)
                    self.arm.set_joint_angles(ang, duration=0.15)

                if show_display:
                    self.camera.cv2.imshow(wname, disp)
                    if self.camera.cv2.waitKey(30) & 0xFF == ord('q'):
                        return False
                time.sleep(0.1)
            else:
                print("[Phase1] ⚠ Y轴居中超时")
                return False

            if abs(self.center_offset_rad) > 0.001:
                print(f"\n[Phase1] 应用居中偏移: {np.rad2deg(self.center_offset_rad):.1f}° → 关节0")
                ang = self.arm.get_joint_angles()
                if ang is not None:
                    ang[0] += self.center_offset_rad
                    ang[0] = np.clip(ang[0], -1.5, 1.5)
                    self.arm.set_joint_angles(ang, duration=0.3)
                    time.sleep(0.3)

            current_cx_error = 0.0
            frame = self.camera.get_frame()
            if frame is not None:
                r_post = self._detect_with_fallback(frame)
                if r_post is not None:
                    current_cx_error = r_post['center'][0] - self.img_cx
                    print(f"[Phase2] 居中后方框X偏差: {current_cx_error:.0f}px (中心={r_post['center']})")
                else:
                    print("[Phase2] ⚠ 居中后未检测到方框, 使用偏差=0")
            else:
                print("[Phase2] ⚠ 无法获取画面, 使用偏差=0")

            # --- Phase 2: forward approach (single move_linear, keep Z + wrist Z) ---
            if self.arm.urdf is None:
                print("[Phase2] ⚠ URDF未加载, 跳过前进阶段")
                return False

            forward_dist = abs(current_cx_error) * self.FORWARD_COEFFICIENT
            print(f"\n[Phase2] X偏差={current_cx_error:.0f}px → 前进距离={forward_dist*1000:.1f}mm (系数={self.FORWARD_COEFFICIENT*1000:.1f}mm/px)")

            ang_before = self.arm.get_joint_angles()
            wrist_z_current = None
            if ang_before is not None:
                wrist_z_current = self.arm.get_wrist_position(ang_before[:5])[2]
                print(f"[Phase2] 当前腕部Z高度: {wrist_z_current*1000:.1f}mm, 将保持此高度前进")

            current_pos = self.arm.get_current_xyz()
            if current_pos is None:
                print("[Phase2] ✗ 无法获取当前位置")
                return False

            target_x = current_pos[0] + forward_dist
            target_y = current_pos[1]
            target_z = current_pos[2]

            print(f"[Phase2] 向前直线运动:")
            print(f"  起点: ({current_pos[0]*1000:.1f}, {current_pos[1]*1000:.1f}, {current_pos[2]*1000:.1f}) mm")
            print(f"  终点: ({target_x*1000:.1f}, {target_y*1000:.1f}, {target_z*1000:.1f}) mm")
            if wrist_z_current is not None:
                print(f"  腕部Z: {wrist_z_current*1000:.1f}mm (保持不变)")

            ok = self.arm.move_linear([target_x, target_y, target_z],
                                      wrist_z=wrist_z_current,
                                      duration=1.5,
                                      num_steps=30,
                                      free_joints=[0, 1, 2, 3])
            if ok:
                print("[Phase2] ✓ 前进完成")
            else:
                print("[Phase2] ⚠ move_linear失败")

            print("[Phase2] ✓ 方框居中流程完成")

            if show_display:
                frame = self.camera.get_frame()
                if frame is not None:
                    r = self._detect_with_fallback(frame)
                    if r is not None:
                        disp = self.camera.draw_frame_rect(frame, r) if r['corners'] is not None else frame
                        cx, cy = r['center']
                        self.camera.cv2.line(disp, (self.img_cx, 0), (self.img_cx, 480), (0, 255, 0), 1)
                        self.camera.cv2.line(disp, (0, self.img_cy), (640, self.img_cy), (0, 255, 0), 1)
                        self.camera.cv2.putText(disp, f"Final: ({cx},{cy}) err_x={cx-self.img_cx} err_y={cy-self.img_cy}",
                                                (10, 30), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        self.camera.cv2.imshow(wname, disp)
                        self.camera.cv2.waitKey(1000)

            return True

        finally:
            if show_display:
                self.camera.cv2.destroyWindow(wname)

    def place_object(self, show_display: bool = True) -> bool:
        print("\n" + "=" * 60)
        print("[放置] 视觉伺服放置流程")
        print("=" * 60)

        if not self.arm.connected:
            print("[放置] 错误: 机械臂未连接")
            return False

        if not self.servo_frame_center(show_display):
            print("[放置] ✗ 方框居中失败")
            return False

        print("\n[放置] 下降到放置高度...")
        pos = self.arm.get_current_xyz()
        if pos is not None:
            self.arm.move_to_xyz([pos[0], pos[1], self.PLACE_Z], duration=1.0)
        time.sleep(0.5)

        print("[放置] 释放物体...")
        ang = self.arm.get_joint_angles()
        if ang is not None:
            ang[5] = self.GRIPPER_OPEN
            self.arm.set_joint_angles(ang, duration=0.5)
        time.sleep(0.5)

        print("[放置] 提升离开...")
        pos = self.arm.get_current_xyz()
        if pos is not None:
            self.arm.move_to_xyz([pos[0], pos[1], pos[2] + self.RETRACT_Z], duration=1.0)
        time.sleep(0.3)

        print("\n" + "=" * 60)
        print("[放置] ✓ 放置完成")
        print("=" * 60)
        return True


def main():
    import sys
    import os
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'

    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
    if not os.path.exists(urdf_path):
        urdf_path = None
        print("[WARN] URDF未找到, 放置功能不可用")

    print("=" * 60)
    print("放置策略测试 - 视觉伺服版")
    print("=" * 60)

    arm = SOARM101Controller(port, urdf_path=urdf_path)
    camera = WristCamera(camera_id=1)

    if not arm.connect():
        print("无法连接机械臂")
        return

    if not camera.is_ready():
        print("警告: 摄像头未就绪")

    placing = PlacingStrategy(arm, camera)

    print("\n测试选项:")
    print("0. 摄像头调试预览 (看画面+红色mask)")
    print("1. 检测方框")
    print("2. 方框视觉伺服居中")
    print("3. 执行完整放置")
    print("4. 返回初始位置")
    print("5. 前往起始姿态 (摄像头朝下)")
    print("6. 保存当前姿态为起始姿态")
    print("7. 设置居中后偏移角度 (当前: {:.1f}°)".format(np.rad2deg(placing.center_offset_rad)))
    print("q. 退出")

    while True:
        cmd = input("\n请选择: ").strip().lower()
        if cmd == 'q':
            break
        elif cmd == '0':
            placing.camera_debug_preview()
        elif cmd == '1':
            center = placing.detect_frame_center()
            if center:
                print(f"检测到方框中心: {center}")
            else:
                print("未检测到方框")
        elif cmd == '2':
            placing.servo_frame_center(show_display=True)
        elif cmd == '3':
            placing.place_object(show_display=True)
        elif cmd == '4':
            arm.move_to_neutral()
        elif cmd == '5':
            placing.go_to_start_pose()
        elif cmd == '6':
            placing.save_current_as_start_pose()
        elif cmd == '7':
            try:
                deg = float(input("输入偏移角度(度, 正值=向右): ").strip())
                placing.set_center_offset(deg)
            except ValueError:
                print("✗ 请输入有效数字")

    camera.release()
    arm.disconnect()
    print("\n测试完成")


if __name__ == "__main__":
    main()
