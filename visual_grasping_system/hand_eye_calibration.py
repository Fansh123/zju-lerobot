"""
手眼标定模块
用于标定腕部摄像头与末端执行器之间的变换关系
"""

import numpy as np
import time
import os
import yaml
from typing import Optional, List, Tuple, Dict

from wrist_camera import WristCamera
from soarm101_sdk_urdf import SOARM101Controller


class HandEyeCalibration:
    """手眼标定类"""
    
    def __init__(self, arm: SOARM101Controller, camera: WristCamera,
                 calibration_dir: str = 'calibration_data'):
        self.arm = arm
        self.camera = camera
        self.calibration_dir = calibration_dir
        
        self.chessboard_size = (8, 5)
        self.square_size = 25
        
        self.calibration_data = {
            'camera_matrix': None,
            'dist_coeffs': None,
            'hand_eye_transform': None,
            'calibrated': False
        }
        
        self._ensure_calibration_dir()
    
    def _ensure_calibration_dir(self):
        if not os.path.exists(self.calibration_dir):
            os.makedirs(self.calibration_dir)
    
    def calibrate_camera(self, num_images: int = 20) -> bool:
        print("\n" + "="*60)
        print("相机内参标定")
        print("="*60)
        print(f"\n需要采集 {num_images} 张棋盘格图像")
        print("请在不同角度和距离拍摄标定板\n")
        
        if not self.camera.is_ready():
            print("错误: 摄像头未就绪")
            return False
        
        obj_points = []
        img_points = []
        
        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        
        images_captured = 0
        
        print("按 'c' 捕获图像，按 'q' 完成标定")
        print("\n⚠️  请点击弹出的摄像头窗口，然后按键操作！")
        print("⚠️  必须检测到棋盘格（显示彩色角点）才能按 'c' 拍照！\n")
        
        while images_captured < num_images:
            frame = self.camera.get_frame()
            if frame is None:
                continue
            
            display = frame.copy()
            gray = self.camera.cv2.cvtColor(frame, self.camera.cv2.COLOR_BGR2GRAY)
            
            ret, corners = self.camera.cv2.findChessboardCorners(gray, self.chessboard_size, None)
            
            if ret:
                corners = self._refine_corners(gray, corners)
                self.camera.cv2.drawChessboardCorners(display, self.chessboard_size, corners, ret)
                status = "CHESSBOARD DETECTED - Press 'c' now!"
                status_color = (0, 255, 0)
            else:
                status = "NO CHESSBOARD - Move the board into view"
                status_color = (0, 0, 255)
            
            self.camera.cv2.putText(display, f"Captured: {images_captured}/{num_images}", 
                                    (10, 30), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 255, 0), 2)
            self.camera.cv2.putText(display, status, 
                                    (10, 70), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.8, status_color, 2)
            
            self.camera.cv2.imshow("Camera Calibration", display)
            
            key = self.camera.cv2.waitKey(50) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('c'):
                if ret:
                    obj_points.append(objp)
                    img_points.append(corners)
                    images_captured += 1
                    print(f"  ✓ 采集图像 {images_captured}/{num_images}")
                else:
                    print("  ✗ 未检测到棋盘格，请调整标定板位置！")
        
        self.camera.cv2.destroyWindow("Camera Calibration")
        
        if images_captured < 10:
            print("错误: 采集的图像数量不足")
            return False
        
        print("\n计算相机内参...")
        
        frame_size = (frame.shape[1], frame.shape[0])
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = \
            self.camera.cv2.calibrateCamera(obj_points, img_points, frame_size, None, None)
        
        if not ret:
            print("标定失败")
            return False
        
        self.calibration_data['camera_matrix'] = camera_matrix
        self.calibration_data['dist_coeffs'] = dist_coeffs
        
        print("\n相机内参:")
        print(camera_matrix)
        print("\n畸变系数:")
        print(dist_coeffs)
        
        self._save_camera_calibration()
        
        print("\n✓ 相机内参标定完成")
        return True
    
    def _refine_corners(self, gray, corners):
        """亚像素精化角点"""
        criteria = (self.camera.cv2.TERM_CRITERIA_EPS + self.camera.cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        refined = self.camera.cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        return refined

    def calibrate_hand_eye(self, num_poses: int = 15) -> bool:
        print("\n" + "="*60)
        print("手眼标定")
        print("="*60)
        print(f"\n需要采集 {num_poses} 组数据, 建议20-30组")
        print("姿态变化越大（位置+角度），标定越准\n")
        
        if not self.camera.is_ready():
            print("错误: 摄像头未就绪")
            return False
        
        if not self.arm.connected:
            print("错误: 机械臂未连接")
            return False
        
        print("\n⚠️  重要提示:")
        print("  1. 标定板必须固定不动")
        print("  2. 棋盘格必须在视野中央附近")
        print("  3. 每次采集前尽量改变机械臂姿态（位置和角度都要变）")
        print("  4. 按 'm' 会自动移动到较远的不同位置\n")
        
        raw_R_gripper2base = []
        raw_t_gripper2base = []
        raw_R_target2cam = []
        raw_t_target2cam = []
        
        poses_captured = 0
        torque_enabled = True
        
        print("按 'c' 捕获当前姿态，按 'q' 完成标定")
        print("按 'd' 断开扭矩（可手动拖动机械臂），按 'r' 恢复扭矩")
        print("按 'm' 自动移动机械臂到新位置")
        print("\n⚠️  请点击弹出的摄像头窗口，然后按键操作！\n")
        
        while poses_captured < num_poses:
            frame = self.camera.get_frame()
            if frame is None:
                continue
            
            display = frame.copy()
            gray = self.camera.cv2.cvtColor(frame, self.camera.cv2.COLOR_BGR2GRAY)
            
            ret, corners = self.camera.cv2.findChessboardCorners(gray, self.chessboard_size, None)
            
            if ret:
                corners = self._refine_corners(gray, corners)
                self.camera.cv2.drawChessboardCorners(display, self.chessboard_size, corners, ret)
                status = "CHESSBOARD OK - Press 'c' now!"
                status_color = (0, 255, 0)
            else:
                status = "NO CHESSBOARD - Adjust arm position"
                status_color = (0, 0, 255)
            
            torque_status = "TORQUE: ON" if torque_enabled else "TORQUE: OFF (can drag arm)"
            torque_color = (0, 255, 0) if torque_enabled else (0, 165, 255)
            
            self.camera.cv2.putText(display, f"Poses: {poses_captured}/{num_poses}", 
                                    (10, 30), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 255, 0), 2)
            self.camera.cv2.putText(display, status, 
                                    (10, 70), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.7, status_color, 2)
            self.camera.cv2.putText(display, torque_status, 
                                    (10, 100), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.7, torque_color, 2)
            self.camera.cv2.putText(display, "c=capture  d=disable torque  r=enable  m=move  q=quit", 
                                    (10, 130), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.5, (255, 255, 0), 2)
            
            self.camera.cv2.imshow("Hand-Eye Calibration", display)
            
            key = self.camera.cv2.waitKey(50) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('d'):
                for i in range(6):
                    self.arm.bus.enable_torque(i + 1, False)
                    time.sleep(0.05)
                torque_enabled = False
                print("  ⚠️ 扭矩已断开，现在可以手动拖动机械臂！")
            elif key == ord('r'):
                for i in range(6):
                    self.arm.bus.enable_torque(i + 1, True)
                    time.sleep(0.05)
                torque_enabled = True
                print("  ✓ 扭矩已恢复")
            elif key == ord('m'):
                if not torque_enabled:
                    for i in range(6):
                        self.arm.bus.enable_torque(i + 1, True)
                        time.sleep(0.05)
                    torque_enabled = True
                print("  移动机械臂到新位置...")
                self._move_to_diverse_pose()
            elif key == ord('c'):
                if ret:
                    if not torque_enabled:
                        for i in range(6):
                            self.arm.bus.enable_torque(i + 1, True)
                            time.sleep(0.05)
                        torque_enabled = True
                        print("  ✓ 扭矩已恢复")
                        time.sleep(0.3)
                    
                    joint_angles = self.arm.get_joint_angles()
                    
                    R_ee, t_ee = self._forward_kinematics(joint_angles)
                    raw_R_gripper2base.append(R_ee)
                    raw_t_gripper2base.append(t_ee)
                    
                    _, rvec, tvec = self.camera.cv2.solvePnP(
                        self._get_object_points(), 
                        corners,
                        self.calibration_data['camera_matrix'],
                        self.calibration_data['dist_coeffs'],
                        flags=self.camera.cv2.SOLVEPNP_IPPE
                    )
                    
                    R, _ = self.camera.cv2.Rodrigues(rvec)
                    raw_R_target2cam.append(R)
                    raw_t_target2cam.append(tvec.astype(np.float64) / 1000.0)
                    
                    poses_captured += 1
                    print(f"  ✓ 采集姿态 {poses_captured}/{num_poses}")
                    print(f"     末端位置: ({t_ee[0]*1000:.1f}, {t_ee[1]*1000:.1f}, {t_ee[2]*1000:.1f}) mm")
                    print("  按 'd' 断开扭矩手动调整姿态，或按 'm' 自动移动...")
                else:
                    print("  ✗ 未检测到棋盘格，请调整机械臂位置！")
        
        self.camera.cv2.destroyWindow("Hand-Eye Calibration")
        
        if poses_captured < 10:
            print("错误: 采集的数据不足")
            return False
        
        print(f"\n共采集 {poses_captured} 组数据")
        print("计算手眼变换矩阵...")
        
        R_cam2gripper, t_cam2gripper = self._calibrate_with_filter(
            raw_R_gripper2base, raw_t_gripper2base,
            raw_R_target2cam, raw_t_target2cam
        )
        
        if R_cam2gripper is None:
            print("错误: 标定失败")
            return False
        
        self.calibration_data['hand_eye_transform'] = {
            'rotation': R_cam2gripper,
            'translation': t_cam2gripper
        }
        self.calibration_data['calibrated'] = True
        
        print("\n" + "="*60)
        print("手眼标定结果")
        print("="*60)
        print("旋转矩阵:")
        print(R_cam2gripper)
        print("\n平移向量 (mm):")
        print(t_cam2gripper.flatten() * 1000)
        
        translation = t_cam2gripper.flatten() * 1000
        print(f"\n  物理意义: 相机在末端坐标系下的偏移")
        print(f"  X: {translation[0]:.1f} mm")
        print(f"  Y: {translation[1]:.1f} mm")
        print(f"  Z: {translation[2]:.1f} mm")
        
        self._verify_calibration(
            raw_R_gripper2base, raw_t_gripper2base,
            raw_R_target2cam, raw_t_target2cam,
            R_cam2gripper, t_cam2gripper
        )
        
        self._save_hand_eye_calibration()
        
        self.arm.move_to_neutral()
        
        print("\n✓ 手眼标定完成")
        return True
    
    def _compute_board_in_base(self, R_g2b, t_g2b, R_t2c, t_t2c, R_c2g, t_c2g):
        """
        计算标定板在基座坐标系下的位置
        标定板固定不动，所有测量值应该一致
        """
        t_board2base = R_g2b @ (R_c2g @ t_t2c.reshape(3, 1) + t_c2g.reshape(3, 1)) + t_g2b.reshape(3, 1)
        return t_board2base.flatten()
    
    def _calibrate_with_filter(self, R_g2b, t_g2b, R_t2c, t_t2c):
        """
        带离群值剔除的标定：
        1. 用全部数据初步标定
        2. 计算每组数据推算出的标定板位置
        3. 去掉偏离标定板平均位置最远的25%数据
        4. 用剩下的数据重新标定
        """
        all_R_g2b = list(R_g2b)
        all_t_g2b = list(t_g2b)
        all_R_t2c = list(R_t2c)
        all_t_t2c = list(t_t2c)
        
        for method_name, method in [
            ("TSAI", self.camera.cv2.CALIB_HAND_EYE_TSAI),
            ("PARK", self.camera.cv2.CALIB_HAND_EYE_PARK),
            ("HORAUD", self.camera.cv2.CALIB_HAND_EYE_HORAUD),
        ]:
            try:
                R, t = self.camera.cv2.calibrateHandEye(
                    all_R_g2b, all_t_g2b,
                    all_R_t2c, all_t_t2c,
                    method=method
                )
                print(f"  {method_name} 方法: t=({t[0]*1000:.1f}, {t[1]*1000:.1f}, {t[2]*1000:.1f}) mm")
            except Exception as e:
                print(f"  {method_name} 方法失败: {e}")
        
        R_best, t_best = self.camera.cv2.calibrateHandEye(
            all_R_g2b, all_t_g2b,
            all_R_t2c, all_t_t2c,
            method=self.camera.cv2.CALIB_HAND_EYE_TSAI
        )
        
        board_positions = []
        for i in range(len(all_R_g2b)):
            pos = self._compute_board_in_base(
                all_R_g2b[i], all_t_g2b[i],
                all_R_t2c[i], all_t_t2c[i],
                R_best, t_best
            )
            board_positions.append(pos)
        board_positions = np.array(board_positions)
        
        mean_pos = np.mean(board_positions, axis=0)
        errors = np.linalg.norm(board_positions - mean_pos, axis=1)
        threshold = np.percentile(errors, 75)
        
        valid_indices = [i for i, e in enumerate(errors) if e <= threshold]
        removed_count = len(errors) - len(valid_indices)
        
        if removed_count > 0:
            print(f"\n  剔除 {removed_count} 组离群数据 (偏差阈值: {threshold*1000:.2f}mm)")
            print(f"  使用剩余 {len(valid_indices)} 组数据重新标定...")
            
            filtered_R_g2b = [all_R_g2b[i] for i in valid_indices]
            filtered_t_g2b = [all_t_g2b[i] for i in valid_indices]
            filtered_R_t2c = [all_R_t2c[i] for i in valid_indices]
            filtered_t_t2c = [all_t_t2c[i] for i in valid_indices]
            
            R_final, t_final = self.camera.cv2.calibrateHandEye(
                filtered_R_g2b, filtered_t_g2b,
                filtered_R_t2c, filtered_t_t2c,
                method=self.camera.cv2.CALIB_HAND_EYE_TSAI
            )
            
            final_board_positions = []
            for i in range(len(filtered_R_g2b)):
                pos = self._compute_board_in_base(
                    filtered_R_g2b[i], filtered_t_g2b[i],
                    filtered_R_t2c[i], filtered_t_t2c[i],
                    R_final, t_final
                )
                final_board_positions.append(pos)
            final_board_positions = np.array(final_board_positions)
            final_mean = np.mean(final_board_positions, axis=0)
            final_errors = np.linalg.norm(final_board_positions - final_mean, axis=1)
            
            print(f"  过滤前标定板位置标准差: {np.std(errors):.1f}mm")
            print(f"  过滤后标定板位置标准差: {np.std(final_errors):.1f}mm")
            
            return R_final, t_final
        
        return R_best, t_best
    
    def _verify_calibration(self, R_g2b, t_g2b, R_t2c, t_t2c, R_c2g, t_c2g):
        """验证标定结果"""
        print("\n" + "="*60)
        print("标定验证")
        print("="*60)
        
        board_positions = []
        for i in range(len(R_g2b)):
            pos = self._compute_board_in_base(
                R_g2b[i], t_g2b[i], R_t2c[i], t_t2c[i], R_c2g, t_c2g
            )
            board_positions.append(pos)
        board_positions = np.array(board_positions)
        
        mean_pos = np.mean(board_positions, axis=0)
        errors = np.linalg.norm(board_positions - mean_pos, axis=1) * 1000
        
        print(f"  标定板平均位置: ({mean_pos[0]*1000:.1f}, {mean_pos[1]*1000:.1f}, {mean_pos[2]*1000:.1f}) mm")
        print(f"  标定板位置偏差 - 平均: {np.mean(errors):.2f} mm")
        print(f"  标定板位置偏差 - 最大: {np.max(errors):.2f} mm")
        print(f"  标定板位置偏差 - 最小: {np.min(errors):.2f} mm")
        print(f"  标准偏差: {np.std(errors):.2f} mm")
        
        if np.mean(errors) < 10:
            print("\n  ✓ 标定质量良好")
        elif np.mean(errors) < 30:
            print("\n  ⚠️ 标定质量一般，建议增加更多姿态变化")
        else:
            print("\n  ✗ 标定质量差，建议重新采集数据")
            print("    可能原因: URDF模型不准确 / 相机内参不准 / 数据采集问题")

    def _forward_kinematics(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.arm.urdf is None:
            print("[ERROR] URDF未加载，无法计算正运动学")
            return np.eye(3), np.zeros(3)
        
        position, rotation = self.arm.forward_kinematics(joint_angles[:5])
        
        if position is None or rotation is None:
            return np.eye(3), np.zeros(3)
        
        return rotation, position
    
    def _get_object_points(self) -> np.ndarray:
        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        return objp
    
    def _move_to_diverse_pose(self, max_retries: int = 5):
        """
        从当前位置做增量式小步随机移动
        保留上一帧画面检查棋盘格是否可见，不可见则退回重试
        """
        current_angles = self.arm.get_joint_angles()
        if current_angles is None:
            return
        
        for attempt in range(max_retries):
            delta = np.random.uniform(-0.08, 0.08, 5)
            
            joint_ranges = [
                (0, -0.8, 0.8),    # shoulder_pan: 左右摆动
                (1, -1.2, -0.2),   # shoulder_lift: 小范围升降
                (2, 0.6, 1.5),     # elbow_flex: 肘部微调
                (3, -0.6, 0.3),    # wrist_flex: 腕部俯仰
                (4, -0.8, 0.8),    # wrist_roll: 腕部旋转
            ]
            
            new_angles = current_angles.copy()
            for idx, lower, upper in joint_ranges:
                val = current_angles[idx] + delta[idx]
                new_angles[idx] = np.clip(val, lower, upper)
            
            new_angles[5] = current_angles[5]
            
            self.arm.set_joint_angles(new_angles, duration=1.0)
            time.sleep(0.5)
            
            frame = self.camera.get_frame()
            if frame is not None:
                gray = self.camera.cv2.cvtColor(frame, self.camera.cv2.COLOR_BGR2GRAY)
                ret, _ = self.camera.cv2.findChessboardCorners(gray, self.chessboard_size, None)
                if ret:
                    return
            
            print("  ⚠️ 移动后丢失标定板视野，退回重试...")
            self.arm.set_joint_angles(current_angles, duration=0.8)
            time.sleep(0.5)
        
        print("  ⚠️ 多次重试后仍无法找到标定板，请手动调整")
    
    def _save_camera_calibration(self):
        filepath = os.path.join(self.calibration_dir, 'camera_params.yaml')
        
        data = {
            'camera_matrix': self.calibration_data['camera_matrix'].tolist(),
            'dist_coeffs': self.calibration_data['dist_coeffs'].tolist()
        }
        
        with open(filepath, 'w') as f:
            yaml.dump(data, f)
        
        print(f"相机参数已保存: {filepath}")
    
    def _save_hand_eye_calibration(self):
        filepath = os.path.join(self.calibration_dir, 'hand_eye.yaml')
        
        data = {
            'rotation': self.calibration_data['hand_eye_transform']['rotation'].tolist(),
            'translation': self.calibration_data['hand_eye_transform']['translation'].tolist(),
            'calibrated': True
        }
        
        with open(filepath, 'w') as f:
            yaml.dump(data, f)
        
        print(f"手眼标定数据已保存: {filepath}")
    
    def load_calibration(self) -> bool:
        camera_file = os.path.join(self.calibration_dir, 'camera_params.yaml')
        hand_eye_file = os.path.join(self.calibration_dir, 'hand_eye.yaml')
        
        if os.path.exists(camera_file):
            with open(camera_file, 'r') as f:
                data = yaml.safe_load(f)
                self.calibration_data['camera_matrix'] = np.array(data['camera_matrix'])
                self.calibration_data['dist_coeffs'] = np.array(data['dist_coeffs'])
            print("✓ 相机参数已加载")
        
        if os.path.exists(hand_eye_file):
            with open(hand_eye_file, 'r') as f:
                data = yaml.safe_load(f)
                self.calibration_data['hand_eye_transform'] = {
                    'rotation': np.array(data['rotation']),
                    'translation': np.array(data['translation'])
                }
                self.calibration_data['calibrated'] = data.get('calibrated', False)
            print("✓ 手眼标定数据已加载")
        
        return self.calibration_data['calibrated']
    
    def is_calibrated(self) -> bool:
        return self.calibration_data['calibrated']
    
    def get_hand_eye_transform(self) -> Optional[Dict]:
        return self.calibration_data['hand_eye_transform']
    
    def transform_point(self, point_cam: np.ndarray) -> Optional[np.ndarray]:
        if not self.is_calibrated():
            return None
        
        R = self.calibration_data['hand_eye_transform']['rotation']
        t = self.calibration_data['hand_eye_transform']['translation']
        
        point_ee = R @ point_cam + t.flatten()
        
        return point_ee


def main():
    import argparse
    import os
    
    parser = argparse.ArgumentParser(description='手眼标定工具')
    parser.add_argument('port', nargs='?', default='COM18', help='串口端口')
    parser.add_argument('--camera', type=int, default=0, help='摄像头ID (默认0)')
    
    args = parser.parse_args()
    
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
    if not os.path.exists(urdf_path):
        urdf_path = None
        print("[WARN] 未找到URDF文件，正运动学将不可用")
    
    print("="*60)
    print("手眼标定工具")
    print("="*60)
    
    arm = SOARM101Controller(args.port, urdf_path=urdf_path)
    camera = WristCamera(camera_id=args.camera)
    
    if not arm.connect():
        print("无法连接机械臂")
        return
    
    if not camera.is_ready():
        print("摄像头未就绪")
        arm.disconnect()
        return
    
    calib = HandEyeCalibration(arm, camera)
    
    print("\n标定选项:")
    print("1. 相机内参标定")
    print("2. 手眼标定")
    print("3. 完整标定流程")
    print("4. 加载已有标定数据")
    print("q. 退出")
    
    while True:
        cmd = input("\n请选择: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == '1':
            calib.calibrate_camera()
        elif cmd == '2':
            if calib.calibration_data['camera_matrix'] is None:
                print("请先进行相机内参标定")
            else:
                calib.calibrate_hand_eye()
        elif cmd == '3':
            if calib.calibrate_camera():
                calib.calibrate_hand_eye()
        elif cmd == '4':
            calib.load_calibration()
    
    camera.release()
    arm.disconnect()
    print("\n标定完成")


if __name__ == "__main__":
    main()
