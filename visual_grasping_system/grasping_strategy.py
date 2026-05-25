"""
抓取策略模块
提供横向抓取和纵向抓取两种策略
"""

import numpy as np
import time
from typing import Optional, Dict, Tuple
from soarm101_sdk_urdf import SOARM101Controller
from coordinate_transformer import CoordinateTransformer


class GraspingStrategy:
    """抓取策略类"""
    
    GRASP_HORIZONTAL = 'horizontal'
    GRASP_VERTICAL = 'vertical'
    
    HORIZONTAL_APPROACH = {
        'shoulder_pan': 0,
        'shoulder_lift': 0.3,
        'elbow_flex': -0.5,
        'wrist_flex': 1.2,
        'wrist_roll': 0,
        'gripper': 1.5
    }
    
    VERTICAL_APPROACH = {
        'shoulder_pan': 0,
        'shoulder_lift': 0.5,
        'elbow_flex': -0.8,
        'wrist_flex': 0,
        'wrist_roll': 0,
        'gripper': 1.5
    }
    
    def __init__(self, arm: SOARM101Controller):
        self.arm = arm
        
        self.approach_height = 0.08
        self.grasp_height = 0.02
        self.lift_height = 0.15
        
        self.gripper_open = 1.5
        self.gripper_close_horizontal = 0.2
        self.gripper_close_vertical = 0.3
    
    def execute_grasp(self, target_pos: Tuple[float, float], grasp_type: str = 'vertical',
                      orientation: float = 0) -> bool:
        if not self.arm.connected:
            print("错误: 机械臂未连接")
            return False
        
        print(f"\n执行{grasp_type}抓取...")
        print(f"目标位置: {target_pos}, 方向: {orientation:.1f}°")
        
        if grasp_type == self.GRASP_HORIZONTAL:
            return self._horizontal_grasp(target_pos, orientation)
        else:
            return self._vertical_grasp(target_pos, orientation)
    
    def _horizontal_grasp(self, target_pos: Tuple[float, float], orientation: float) -> bool:
        print("\n[横向抓取流程]")
        
        try:
            print("1. 移动到物体上方...")
            self._move_above_target(target_pos, self.approach_height)
            time.sleep(0.5)
            
            print("2. 调整腕部角度（横向）...")
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[3] = 1.2
                angles[4] = np.radians(orientation)
                angles[5] = self.gripper_open
                self.arm.set_joint_angles(angles, duration=1.0)
            time.sleep(0.5)
            
            print("3. 下降到抓取高度...")
            self._move_to_grasp_height(target_pos, self.grasp_height)
            time.sleep(0.5)
            
            print("4. 闭合夹爪...")
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[5] = self.gripper_close_horizontal
                self.arm.set_joint_angles(angles, duration=0.5)
            time.sleep(0.5)
            
            print("5. 提升物体...")
            self._lift_object(self.lift_height)
            time.sleep(0.5)
            
            print("✓ 横向抓取完成")
            return True
            
        except Exception as e:
            print(f"抓取失败: {e}")
            return False
    
    def _vertical_grasp(self, target_pos: Tuple[float, float], orientation: float) -> bool:
        print("\n[纵向抓取流程]")
        
        try:
            print("1. 移动到物体上方...")
            self._move_above_target(target_pos, self.approach_height)
            time.sleep(0.5)
            
            print("2. 调整腕部角度（纵向）...")
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[3] = 0
                angles[4] = 0
                angles[5] = self.gripper_open
                self.arm.set_joint_angles(angles, duration=1.0)
            time.sleep(0.5)
            
            print("3. 下降到抓取高度...")
            self._move_to_grasp_height(target_pos, self.grasp_height)
            time.sleep(0.5)
            
            print("4. 闭合夹爪...")
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[5] = self.gripper_close_vertical
                self.arm.set_joint_angles(angles, duration=0.5)
            time.sleep(0.5)
            
            print("5. 提升物体...")
            self._lift_object(self.lift_height)
            time.sleep(0.5)
            
            print("✓ 纵向抓取完成")
            return True
            
        except Exception as e:
            print(f"抓取失败: {e}")
            return False
    
    def _move_above_target(self, target_pos: Tuple[float, float], height: float):
        angles = self.arm.get_joint_angles()
        if angles is None:
            return
        
        target_x, target_y = target_pos
        
        shoulder_pan = np.arctan2(target_y, target_x) if abs(target_x) > 0.01 else 0
        
        distance = np.sqrt(target_x**2 + target_y**2)
        
        shoulder_lift = 0.3 + height * 2
        elbow_flex = -0.5 - distance * 0.3
        
        angles[0] = np.clip(shoulder_pan, -1.92, 1.92)
        angles[1] = np.clip(shoulder_lift, -1.75, 1.75)
        angles[2] = np.clip(elbow_flex, -1.69, 1.69)
        
        self.arm.set_joint_angles(angles, duration=1.5)
    
    def _move_to_grasp_height(self, target_pos: Tuple[float, float], height: float):
        angles = self.arm.get_joint_angles()
        if angles is None:
            return
        
        angles[1] = angles[1] - 0.2
        angles[2] = angles[2] + 0.3
        
        self.arm.set_joint_angles(angles, duration=1.0)
    
    def _lift_object(self, height: float):
        angles = self.arm.get_joint_angles()
        if angles is None:
            return
        
        angles[1] = angles[1] + 0.3
        angles[2] = angles[2] - 0.2
        
        self.arm.set_joint_angles(angles, duration=1.0)
    
    def release_object(self) -> bool:
        if not self.arm.connected:
            return False
        
        print("释放物体...")
        angles = self.arm.get_joint_angles()
        if angles is not None:
            angles[5] = self.gripper_open
            self.arm.set_joint_angles(angles, duration=0.5)
        
        return True
    
    def return_to_home(self) -> bool:
        if not self.arm.connected:
            return False
        
        print("返回初始位置...")
        self.arm.move_to_neutral(duration=1.5)
        return True
    
    def set_grasp_parameters(self, approach_height: float = None, 
                             grasp_height: float = None,
                             lift_height: float = None,
                             gripper_open: float = None,
                             gripper_close: float = None):
        if approach_height is not None:
            self.approach_height = approach_height
        if grasp_height is not None:
            self.grasp_height = grasp_height
        if lift_height is not None:
            self.lift_height = lift_height
        if gripper_open is not None:
            self.gripper_open = gripper_open
        if gripper_close is not None:
            self.gripper_close_horizontal = gripper_close
            self.gripper_close_vertical = gripper_close


class GraspExecutor:
    """抓取执行器 - 提供更高级的抓取控制"""
    
    def __init__(self, arm: SOARM101Controller, calibration_dir: str = 'calibration_data'):
        self.strategy = GraspingStrategy(arm)
        self.arm = arm
        self.transformer = CoordinateTransformer(calibration_dir)
        self.default_depth = 0.25
    
    def auto_grasp(self, obj_info: Dict) -> bool:
        if not obj_info:
            print("错误: 无物体信息")
            return False
        
        grasp_type = obj_info.get('grasp_type', 'vertical')
        position = obj_info.get('center', (320, 240))
        orientation = obj_info.get('orientation', 0)
        
        target_3d = self._image_to_3d(position)
        
        if target_3d is None:
            print("错误: 无法计算目标位置")
            return False
        
        return self.strategy.execute_grasp(target_3d, grasp_type, orientation)
    
    def _image_to_3d(self, image_pos: Tuple[int, int], depth: float = None) -> Optional[Tuple[float, float]]:
        if depth is None:
            depth = self.default_depth
        
        if self.transformer.is_calibrated():
            ee_pos, ee_rot = self.arm.forward_kinematics()
            if ee_pos is not None and ee_rot is not None:
                point_base = self.transformer.image_to_base(
                    image_pos, depth, ee_pos, ee_rot
                )
                if point_base is not None:
                    print(f"[坐标转换] 图像{image_pos} -> 基座({point_base[0]*1000:.1f}, {point_base[1]*1000:.1f}, {point_base[2]*1000:.1f})mm")
                    return (point_base[0], point_base[1])
        
        cx, cy = image_pos
        x = (cx - 320) * 0.001 * depth
        y = (cy - 240) * 0.001 * depth
        return (x + 0.2, y)
    
    def test_grasp_horizontal(self):
        print("\n测试横向抓取...")
        return self.strategy.execute_grasp((0.25, 0), 'horizontal', 0)
    
    def test_grasp_vertical(self):
        print("\n测试纵向抓取...")
        return self.strategy.execute_grasp((0.25, 0), 'vertical', 0)
    
    def test_release(self):
        return self.strategy.release_object()


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("抓取策略测试")
    print("="*60)
    
    arm = SOARM101Controller(port)
    
    if not arm.connect():
        print("无法连接机械臂")
        return
    
    executor = GraspExecutor(arm)
    
    print("\n测试选项:")
    print("1. 测试横向抓取")
    print("2. 测试纵向抓取")
    print("3. 测试释放")
    print("4. 返回初始位置")
    print("q. 退出")
    
    while True:
        cmd = input("\n请选择: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == '1':
            executor.test_grasp_horizontal()
        elif cmd == '2':
            executor.test_grasp_vertical()
        elif cmd == '3':
            executor.test_release()
        elif cmd == '4':
            arm.move_to_neutral()
    
    arm.disconnect()
    print("\n测试完成")


if __name__ == "__main__":
    main()
