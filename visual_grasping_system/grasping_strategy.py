"""
视觉伺服抓取策略
摄像头旋转90度，用Y偏差控制第一舵机
"""

import numpy as np
import time
from typing import Optional, Dict
from soarm101_sdk_urdf import SOARM101Controller
from wrist_camera import WristCamera


class VisualServoGrasp:
    """视觉伺服抓取类"""
    
    CENTER_THRESHOLD_Y = 20
    
    MAX_ITERATIONS = 50
    JOINT_STEP = 0.03
    
    GRASP_Z = 0.010
    FORWARD_DISTANCE = 0.1
    
    GRIPPER_OPEN = 1.1
    GRIPPER_CLOSE = -1.0
    WRIST_ROLL_ANGLE = -np.pi / 2
    
    CENTER_ADJUST_ANGLE = -0.13
    
    SEARCH_JOINT_LIMITS = (-1.5, 1.5)
    SEARCH_STEP = 0.15
    SEARCH_SWEEP_COUNT = 3
    SEARCH_FORWARD_STEP = 0.03
    
    def __init__(self, arm: SOARM101Controller, camera: WristCamera):
        self.arm = arm
        self.camera = camera
        
        self.image_center_y = 240
        self.search_window_name = "Object Search - Press 'q' to abort"
    
    def visual_servo_center(self, show_display: bool = True) -> bool:
        """
        视觉伺服居中：用Y偏差控制第一舵机
        摄像头旋转90度，物块应在画面上下中心线(Y=240)
        
        Args:
            show_display: 是否显示摄像头画面
            
        Returns:
            是否成功居中
        """
        print("\n[视觉伺服] 开始左右调整...")
        
        if show_display:
            window_name = "Visual Servo - Press 'q' to stop"
            self.camera.cv2.namedWindow(window_name)
        
        try:
            for i in range(self.MAX_ITERATIONS):
                frame = self.camera.get_frame()
                if frame is None:
                    continue
                
                display = frame.copy()
                
                self.camera.cv2.line(display, (0, self.image_center_y), (640, self.image_center_y), (0, 0, 255), 2)
                
                cube = self.camera.detect_red_cube(frame)
                
                if cube is None:
                    print(f"  [{i+1}] 未检测到物块")
                    
                    self.camera.cv2.putText(display, "No cube detected", (10, 30),
                                           self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                else:
                    cx, cy = cube['center']
                    error_y = cy - self.image_center_y
                    
                    self.camera.cv2.circle(display, (cx, cy), 10, (0, 255, 0), 2)
                    self.camera.cv2.circle(display, (cx, cy), 3, (0, 255, 0), -1)
                    
                    if abs(error_y) < self.CENTER_THRESHOLD_Y:
                        color = (0, 255, 0)
                        status = f"CENTERED! Y error: {error_y}px"
                        print(f"[视觉伺服] ✓ 居中成功！偏差={error_y}px")
                        
                        self.camera.cv2.putText(display, status, (10, 30),
                                               self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                        
                        if show_display:
                            self.camera.cv2.imshow(window_name, display)
                            self.camera.cv2.waitKey(300)
                        
                        return True
                    else:
                        color = (0, 165, 255)
                        if error_y < 0:
                            direction = "UP -> Turn RIGHT"
                        else:
                            direction = "DOWN -> Turn LEFT"
                        status = f"Y error: {error_y}px ({direction})"
                    
                    self.camera.cv2.putText(display, status, (10, 30),
                                           self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    self.camera.cv2.putText(display, f"Center: ({cx}, {cy})", (10, 60),
                                           self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    print(f"  [{i+1}] Y偏差: {error_y:.0f}px ({direction})")
                    
                    angles = self.arm.get_joint_angles()
                    if angles is not None:
                        if error_y < 0:
                            angles[0] += self.JOINT_STEP
                        else:
                            angles[0] -= self.JOINT_STEP
                        
                        angles[0] = np.clip(angles[0], -1.5, 1.5)
                        self.arm.set_joint_angles(angles, duration=0.2)
                
                if show_display:
                    self.camera.cv2.imshow(window_name, display)
                    key = self.camera.cv2.waitKey(50) & 0xFF
                    if key == ord('q'):
                        break
                
                time.sleep(0.1)
            
            print(f"[视觉伺服] ✗ 居中超时")
            return False
            
        finally:
            if show_display:
                self.camera.cv2.destroyWindow(window_name)
    
    def search_for_object(self, show_display: bool = True) -> bool:
        """
        自主搜索物块：当视野中没有物块时，自动扫描寻找
        
        搜索策略：
        1. 在当前位置左右摆动扫描
        2. 如果没找到，向前移动一小步
        3. 重复扫描，直到找到物块或达到最大次数
        
        Args:
            show_display: 是否显示摄像头画面
            
        Returns:
            是否找到物块
        """
        print("\n" + "="*60)
        print("[搜索] 开始自主搜索物块...")
        print("="*60)
        
        if show_display:
            self.camera.cv2.namedWindow(self.search_window_name)
        
        try:
            initial_angles = self.arm.get_joint_angles()
            if initial_angles is None:
                print("[搜索] 无法获取当前关节角度")
                return False
            
            current_joint0 = initial_angles[0]
            min_angle, max_angle = self.SEARCH_JOINT_LIMITS
            
            for sweep in range(self.SEARCH_SWEEP_COUNT):
                print(f"\n[搜索] 第 {sweep+1}/{self.SEARCH_SWEEP_COUNT} 轮扫描")
                
                for direction in ['left', 'right']:
                    print(f"  [搜索] 向{direction}扫描...")
                    start_angle = current_joint0 if direction == 'left' else min_angle
                    end_angle = min_angle if direction == 'left' else max_angle
                    
                    step_sign = -1 if direction == 'left' else 1
                    current_angle = start_angle
                    
                    while (direction == 'left' and current_angle >= end_angle) or \
                          (direction == 'right' and current_angle <= end_angle):
                        
                        angles = self.arm.get_joint_angles()
                        if angles is None:
                            break
                        
                        angles[0] = np.clip(current_angle, min_angle, max_angle)
                        self.arm.set_joint_angles(angles, duration=0.15)
                        time.sleep(0.1)
                        
                        frame = self.camera.get_frame()
                        if frame is not None:
                            cube = self.camera.detect_red_cube(frame)
                            
                            if cube is not None:
                                cx, cy = cube['center']
                                print(f"  ✓ [搜索] 找到物块！位置: ({cx}, {cy})")
                                
                                if show_display:
                                    display = frame.copy()
                                    self.camera.cv2.circle(display, (cx, cy), 10, (0, 255, 0), 2)
                                    self.camera.cv2.putText(display, "OBJECT FOUND!", (10, 30),
                                                           self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                    self.camera.cv2.imshow(self.search_window_name, display)
                                    self.camera.cv2.waitKey(500)
                                
                                return True
                            
                            if show_display:
                                display = frame.copy()
                                self.camera.cv2.putText(display, f"Searching... ({current_angle*57.3:.1f}°)", 
                                                       (10, 30), self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                                self.camera.cv2.imshow(self.search_window_name, display)
                                key = self.camera.cv2.waitKey(30) & 0xFF
                                if key == ord('q'):
                                    print("[搜索] 用户中止")
                                    return False
                        
                        current_angle += step_sign * self.SEARCH_STEP
                    
                    current_joint0 = angles[0] if angles is not None else current_joint0
                
                if sweep < self.SEARCH_SWEEP_COUNT - 1:
                    print(f"  [搜索] 本轮未找到，向前移动 {self.SEARCH_FORWARD_STEP*1000:.0f}mm...")
                    current_pos = self.arm.get_current_xyz()
                    if current_pos is not None:
                        target_pos = [
                            current_pos[0] + self.SEARCH_FORWARD_STEP,
                            current_pos[1],
                            current_pos[2]
                        ]
                        self.arm.move_to_xyz(target_pos, duration=0.8)
                        time.sleep(0.3)
            
            print("[搜索] ✗ 搜索完毕，未找到物块")
            return False
            
        finally:
            if show_display:
                self.camera.cv2.destroyWindow(self.search_window_name)
    
    def execute_grasp(self, show_display: bool = True) -> bool:
        """
        执行视觉伺服抓取
        
        Args:
            show_display: 是否显示摄像头画面
            
        Returns:
            是否成功
        """
        if not self.arm.connected:
            print("错误: 机械臂未连接")
            return False
        
        if self.arm.urdf is None:
            print("错误: URDF未加载")
            return False
        
        print("\n" + "="*60)
        print("视觉伺服抓取流程")
        print("="*60)
        
        try:
            print("\n[步骤1] 打开夹爪...")
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[5] = self.GRIPPER_OPEN
                self.arm.set_joint_angles(angles, duration=0.5)
            time.sleep(0.3)
            
            print("\n[步骤2] 设置横向抓取姿态...")
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[4] = self.WRIST_ROLL_ANGLE
                self.arm.set_joint_angles(angles, duration=0.8)
            time.sleep(0.5)
            
            print("\n[步骤3] 视觉伺服居中...")
            if not self.visual_servo_center(show_display):
                print("居中失败，开始自主搜索物块...")
                if not self.search_for_object(show_display):
                    print("[步骤3] ✗ 搜索失败，无法找到物块")
                    return False
                print("[步骤3] ✓ 搜索成功，再次尝试居中...")
                if not self.visual_servo_center(show_display):
                    print("[步骤3] ✗ 搜索后居中仍失败")
                    return False
            
            print("\n[步骤3.5] 向左微调10度...")
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[0] += self.CENTER_ADJUST_ANGLE
                angles[0] = np.clip(angles[0], -1.5, 1.5)
                self.arm.set_joint_angles(angles, duration=0.5)
            time.sleep(0.3)
            
            print("\n[步骤4] Z下降到10mm...")
            current_pos = self.arm.get_current_xyz()
            if current_pos is not None:
                self.arm.move_to_xyz([current_pos[0], current_pos[1], self.GRASP_Z], duration=1.5)
            time.sleep(0.5)
            
            angles_after_descend = self.arm.get_joint_angles()
            wrist_z_at_grasp = None
            if angles_after_descend is not None:
                wrist_z_at_grasp = self.arm.get_wrist_position(angles_after_descend[:5])[2]
                print(f"[步骤5] 当前腕部Z高度: {wrist_z_at_grasp*1000:.1f}mm, 将保持此高度前进")

            print("\n[步骤4.5] 降低腕部高度至50mm (保持末端位置不变)...")
            current_end_pos = self.arm.get_current_xyz()
            if current_end_pos is not None and angles_after_descend is not None:
                new_angles_45 = self.arm.inverse_kinematics_constrained(
                    current_end_pos,
                    q_init=angles_after_descend[:5],
                    wrist_z_target=0.050,
                    wrist_z_weight=8.0,
                    max_iter=200,
                    eps=1e-4,
                    damping=0.15,
                    free_joints=[0,1,2,3]
                )
                if new_angles_45 is not None:
                    new_angles_full = np.zeros(6)
                    new_angles_full[:5] = new_angles_45
                    new_angles_full[5] = angles_after_descend[5]
                    self.arm.set_joint_angles(new_angles_full, duration=1.5)
                    time.sleep(0.5)
                    wrist_z_at_grasp = 0.050
                    new_wrist_z = self.arm.get_wrist_position(new_angles_45)[2]
                    print(f"  ✓ 腕部高度已降至 {new_wrist_z*1000:.1f}mm")
                else:
                    print("  ⚠ IK无解, 保持当前腕部高度继续")

            print("\n[步骤5] 向前移动100mm (保持腕部Z不变, 末端直线运动)...")
            current_pos = self.arm.get_current_xyz()
            if current_pos is not None:
                target_x = current_pos[0] + self.FORWARD_DISTANCE
                target_y = current_pos[1]
                target_z = self.GRASP_Z
                self.arm.move_linear([target_x, target_y, target_z],
                                     wrist_z=wrist_z_at_grasp,
                                     duration=1.5,
                                     num_steps=30,
                                     free_joints=[0,1,2,3])
            time.sleep(0.5)
            
            print("\n[步骤6] 闭合夹爪...")
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[5] = self.GRIPPER_CLOSE
                self.arm.set_joint_angles(angles, duration=0.8)
            time.sleep(0.5)
            
            print("\n[步骤7] 提升物体...")
            self.arm.move_relative(dz=0.1, duration=1.5)
            time.sleep(0.5)
            
            print("\n" + "="*60)
            print("✓ 视觉伺服抓取完成！")
            print("="*60)
            return True
            
        except Exception as e:
            print(f"\n抓取失败: {e}")
            return False
    
    def release_object(self) -> bool:
        """释放物体"""
        print("释放物体...")
        angles = self.arm.get_joint_angles()
        if angles is not None:
            angles[5] = self.GRIPPER_OPEN
            self.arm.set_joint_angles(angles, duration=0.5)
        time.sleep(0.3)
        return True
    
    def return_home(self) -> bool:
        """返回初始位置"""
        print("返回初始位置...")
        self.arm.move_to_neutral(duration=1.5)
        return True


class GraspExecutor:
    """抓取执行器"""
    
    def __init__(self, arm: SOARM101Controller, camera: WristCamera = None, 
                 calibration_dir: str = 'calibration_data'):
        if camera is None:
            camera = WristCamera(camera_id=1)
        self.visual_servo = VisualServoGrasp(arm, camera)
        self.arm = arm
        self.camera = camera
    
    def auto_grasp(self, grasp_info: Dict = None) -> bool:
        """自动抓取"""
        return self.visual_servo.execute_grasp(show_display=True)
    
    def test_grasp(self):
        """测试抓取"""
        return self.visual_servo.execute_grasp(show_display=True)
    
    def test_release(self):
        """测试释放"""
        return self.visual_servo.release_object()


def main():
    import sys
    import os
    
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("视觉伺服抓取测试")
    print("="*60)
    
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
    arm = SOARM101Controller(port, urdf_path=urdf_path)
    camera = WristCamera(camera_id=1)
    
    if not arm.connect():
        print("无法连接机械臂")
        return
    
    if not camera.is_ready():
        print("摄像头未就绪")
        arm.disconnect()
        return
    
    executor = GraspExecutor(arm, camera)
    
    print("\n测试选项:")
    print("1. 测试居中")
    print("2. 测试完整抓取")
    print("3. 测试释放")
    print("4. 返回初始位置")
    print("5. 测试自主搜索")
    print("q. 退出")
    
    while True:
        cmd = input("\n请选择: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == '1':
            executor.visual_servo.visual_servo_center(show_display=True)
        elif cmd == '2':
            executor.test_grasp()
        elif cmd == '3':
            executor.test_release()
        elif cmd == '4':
            arm.move_to_neutral()
        elif cmd == '5':
            executor.visual_servo.search_for_object(show_display=True)
    
    camera.release()
    arm.disconnect()
    print("\n测试完成")


if __name__ == "__main__":
    main()
