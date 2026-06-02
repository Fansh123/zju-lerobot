"""
放置策略模块 - 视觉伺服版
摄像头向下看桌面，通过线延长检测方框中心，
先左右居中，再向前平移至中心位置
"""

import numpy as np
import time
from typing import Optional, Dict, Tuple
from wrist_camera import WristCamera
from soarm101_sdk_urdf import SOARM101Controller


class PlacingStrategy:
    """视觉伺服放置策略"""

    CENTER_THRESHOLD_X = 15
    APPROACH_THRESHOLD_Y = 25

    MAX_ITER = 60
    LR_STEP = 0.025
    FWD_STEP = 0.006

    PLACE_Z = 0.015
    RETRACT_Z = 0.12

    GRIPPER_OPEN = 1.1

    def __init__(self, arm: SOARM101Controller, camera: WristCamera = None,
                 camera_id: int = 0):
        self.arm = arm
        self.camera = camera if camera else WristCamera(camera_id)

        self.img_cx = 320
        self.img_cy = 240
        self.approach_target_y = 400

    def detect_frame_center(self, frame: np.ndarray = None) -> Optional[Tuple[int, int]]:
        result = self.camera.detect_red_frame_lines(frame)
        if result is None:
            return None
        return result['center']

    def servo_frame_center(self, show_display: bool = True) -> bool:
        print("\n" + "=" * 60)
        print("[放置伺服] 方框居中流程")
        print("=" * 60)

        wname = "Frame Servo - Press 'q' to abort"
        if show_display:
            self.camera.cv2.namedWindow(wname)

        try:
            # --- Phase 1: left-right centering ---
            print("\n[Phase1] 左右居中 (关节0)...")
            for i in range(self.MAX_ITER):
                frame = self.camera.get_frame()
                if frame is None:
                    continue

                r = self.camera.detect_red_frame_lines(frame)
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
                disp = self.camera.draw_frame_lines(frame, r)
                self.camera.cv2.line(disp, (self.img_cx, 0), (self.img_cx, 480), (0, 255, 0), 1)
                self.camera.cv2.line(disp, (0, self.img_cy), (640, self.img_cy), (0, 255, 0), 1)

                err_x = cx - self.img_cx
                self.camera.cv2.putText(disp, f"X err={err_x}px", (10, 30),
                                        self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                if abs(err_x) < self.CENTER_THRESHOLD_X:
                    print(f"[Phase1] ✓ 左右居中完成 (X误差={err_x}px)")
                    self.camera.cv2.putText(disp, "LR CENTERED", (10, 60),
                                            self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    if show_display:
                        self.camera.cv2.imshow(wname, disp)
                        self.camera.cv2.waitKey(300)
                    break

                ang = self.arm.get_joint_angles()
                if ang is not None:
                    ang[0] += -self.LR_STEP if err_x > 0 else self.LR_STEP
                    ang[0] = np.clip(ang[0], -1.5, 1.5)
                    self.arm.set_joint_angles(ang, duration=0.15)
                    print(f"  [{i+1}] X误差={err_x}px -> 调整关节0")

                if show_display:
                    self.camera.cv2.imshow(wname, disp)
                    if self.camera.cv2.waitKey(30) & 0xFF == ord('q'):
                        return False
                time.sleep(0.1)
            else:
                print("[Phase1] ⚠ 左右居中超时")
                return False

            # --- Phase 2: forward approach ---
            print("\n[Phase2] 向前平移, 方框Y中心 → 画面底部...")
            for i in range(self.MAX_ITER):
                frame = self.camera.get_frame()
                if frame is None:
                    continue

                r = self.camera.detect_red_frame_lines(frame)
                if r is None:
                    print(f"  [{i+1}] 未检测到方框")
                    time.sleep(0.1)
                    continue

                cx, cy = r['center']
                disp = self.camera.draw_frame_lines(frame, r)
                self.camera.cv2.line(disp, (0, self.approach_target_y), (640, self.approach_target_y),
                                     (0, 0, 255), 2)
                self.camera.cv2.line(disp, (0, self.img_cy), (640, self.img_cy), (0, 255, 0), 1)

                err_y = self.approach_target_y - cy
                self.camera.cv2.putText(disp, f"Y target={self.approach_target_y}, cy={cy}, err={err_y}",
                                        (10, 30), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if abs(err_y) < self.APPROACH_THRESHOLD_Y:
                    print(f"[Phase2] ✓ 到达目标 Y (误差={err_y}px)")
                    self.camera.cv2.putText(disp, "APPROACH DONE", (10, 60),
                                            self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    if show_display:
                        self.camera.cv2.imshow(wname, disp)
                        self.camera.cv2.waitKey(500)
                    return True

                pos = self.arm.get_current_xyz()
                if pos is None:
                    continue

                target_x = pos[0] + self.FWD_STEP
                target_y = pos[1]
                target_z = pos[2]
                self.arm.move_to_xyz([target_x, target_y, target_z], duration=0.25)
                print(f"  [{i+1}] cy={cy} → 前进 {self.FWD_STEP*1000:.0f}mm")

                if show_display:
                    self.camera.cv2.imshow(wname, disp)
                    if self.camera.cv2.waitKey(30) & 0xFF == ord('q'):
                        return False
                time.sleep(0.15)
            else:
                print("[Phase2] ⚠ 向前平移超时")
                return False

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
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'

    print("=" * 60)
    print("放置策略测试 - 视觉伺服版")
    print("=" * 60)

    arm = SOARM101Controller(port)
    camera = WristCamera(camera_id=1)

    if not arm.connect():
        print("无法连接机械臂")
        return

    if not camera.is_ready():
        print("警告: 摄像头未就绪")

    placing = PlacingStrategy(arm, camera)

    print("\n测试选项:")
    print("1. 检测方框 (线延长法)")
    print("2. 方框视觉伺服居中")
    print("3. 执行完整放置")
    print("4. 返回初始位置")
    print("q. 退出")

    while True:
        cmd = input("\n请选择: ").strip().lower()
        if cmd == 'q':
            break
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

    camera.release()
    arm.disconnect()
    print("\n测试完成")


if __name__ == "__main__":
    main()
