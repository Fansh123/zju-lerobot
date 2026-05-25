"""
放置策略模块
提供红色方框检测和物体放置功能
"""

import numpy as np
import time
from typing import Optional, Dict, Tuple
from wrist_camera import WristCamera
from soarm101_sdk_urdf import SOARM101Controller
from coordinate_transformer import CoordinateTransformer


class PlacingStrategy:
    """放置策略类"""
    
    def __init__(self, arm: SOARM101Controller, camera: WristCamera = None, camera_id: int = 0,
                 calibration_dir: str = 'calibration_data'):
        self.arm = arm
        self.camera = camera if camera else WristCamera(camera_id)
        self.transformer = CoordinateTransformer(calibration_dir)
        self.default_depth = 0.25
        
        self.frame_size_mm = (100, 100)
        self.place_height = 0.02
        self.approach_height = 0.1
        
        self.gripper_open = 1.5
    
    def detect_place_area(self, frame: np.ndarray = None) -> Optional[Dict]:
        if frame is None:
            frame = self.camera.get_frame()
        
        if frame is None:
            return None
        
        frame_obj = self.camera.detect_red_frame(frame, self.frame_size_mm)
        
        if frame_obj is None:
            return None
        
        center = frame_obj['center']
        bbox = frame_obj['bbox']
        
        target_3d = self._image_to_3d(center)
        
        return {
            'image_center': center,
            'world_position': target_3d,
            'bbox': bbox,
            'area': frame_obj['area'],
            'valid': self._validate_place_area(frame_obj)
        }
    
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
    
    def _validate_place_area(self, frame_obj: Dict) -> bool:
        _, _, w, h = frame_obj['bbox']
        
        min_size = 50
        max_size = 200
        
        if w < min_size or h < min_size:
            return False
        
        if w > max_size or h > max_size:
            return False
        
        aspect_ratio = w / h if h > 0 else 1
        if aspect_ratio < 0.7 or aspect_ratio > 1.3:
            return False
        
        return True
    
    def place_object(self, place_area: Dict = None, frame: np.ndarray = None) -> bool:
        if not self.arm.connected:
            print("错误: 机械臂未连接")
            return False
        
        if place_area is None:
            place_area = self.detect_place_area(frame)
        
        if place_area is None:
            print("错误: 未检测到放置区域")
            return False
        
        if not place_area.get('valid', False):
            print("警告: 放置区域可能不合适")
        
        print("\n[放置流程]")
        
        try:
            target_pos = place_area['world_position']
            print(f"目标位置: {target_pos}")
            
            print("1. 移动到放置区域上方...")
            self._move_above_place(target_pos, self.approach_height)
            time.sleep(0.5)
            
            print("2. 精确定位...")
            self._fine_adjust(place_area)
            time.sleep(0.5)
            
            print("3. 下降到放置高度...")
            self._move_to_place_height(target_pos, self.place_height)
            time.sleep(0.5)
            
            print("4. 释放物体...")
            self._release_object()
            time.sleep(0.5)
            
            print("5. 提升并离开...")
            self._lift_and_leave()
            time.sleep(0.5)
            
            print("✓ 放置完成")
            return True
            
        except Exception as e:
            print(f"放置失败: {e}")
            return False
    
    def _move_above_place(self, target_pos: Tuple[float, float], height: float):
        angles = self.arm.get_joint_angles()
        if angles is None:
            return
        
        target_x, target_y = target_pos
        
        shoulder_pan = np.arctan2(target_y, target_x) if abs(target_x) > 0.01 else 0
        
        distance = np.sqrt(target_x**2 + target_y**2)
        
        shoulder_lift = 0.5 + height * 2
        elbow_flex = -0.8 - distance * 0.3
        
        angles[0] = np.clip(shoulder_pan, -1.92, 1.92)
        angles[1] = np.clip(shoulder_lift, -1.75, 1.75)
        angles[2] = np.clip(elbow_flex, -1.69, 1.69)
        
        self.arm.set_joint_angles(angles, duration=1.5)
    
    def _fine_adjust(self, place_area: Dict):
        current_frame = self.camera.get_frame()
        if current_frame is None:
            return
        
        current_place = self.detect_place_area(current_frame)
        if current_place is None:
            return
        
        current_center = current_place['image_center']
        target_center = place_area['image_center']
        
        dx = current_center[0] - target_center[0]
        dy = current_center[1] - target_center[1]
        
        if abs(dx) > 10 or abs(dy) > 10:
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[0] += dx * 0.001
                angles[1] += dy * 0.001
                self.arm.set_joint_angles(angles, duration=0.5)
    
    def _move_to_place_height(self, target_pos: Tuple[float, float], height: float):
        angles = self.arm.get_joint_angles()
        if angles is None:
            return
        
        angles[1] = angles[1] - 0.15
        angles[2] = angles[2] + 0.2
        
        self.arm.set_joint_angles(angles, duration=1.0)
    
    def _release_object(self):
        angles = self.arm.get_joint_angles()
        if angles is not None:
            angles[5] = self.gripper_open
            self.arm.set_joint_angles(angles, duration=0.5)
    
    def _lift_and_leave(self):
        angles = self.arm.get_joint_angles()
        if angles is None:
            return
        
        angles[1] = angles[1] + 0.2
        angles[2] = angles[2] - 0.1
        
        self.arm.set_joint_angles(angles, duration=1.0)
    
    def verify_placement(self, frame: np.ndarray = None) -> bool:
        if frame is None:
            frame = self.camera.get_frame()
        
        if frame is None:
            return False
        
        place_area = self.detect_place_area(frame)
        if place_area is None:
            return False
        
        objects = self.camera.detect_objects(frame)
        
        place_center = place_area['image_center']
        place_bbox = place_area['bbox']
        
        for obj in objects:
            obj_center = obj['center']
            
            if (place_bbox[0] < obj_center[0] < place_bbox[0] + place_bbox[2] and
                place_bbox[1] < obj_center[1] < place_bbox[1] + place_bbox[3]):
                return True
        
        return False
    
    def set_place_parameters(self, frame_size_mm: Tuple[float, float] = None,
                             place_height: float = None,
                             approach_height: float = None):
        if frame_size_mm is not None:
            self.frame_size_mm = frame_size_mm
        if place_height is not None:
            self.place_height = place_height
        if approach_height is not None:
            self.approach_height = approach_height


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("放置策略测试")
    print("="*60)
    
    arm = SOARM101Controller(port)
    camera = WristCamera(camera_id=0)
    
    if not arm.connect():
        print("无法连接机械臂")
        return
    
    if not camera.is_ready():
        print("警告: 摄像头未就绪")
    
    placing = PlacingStrategy(arm, camera)
    
    print("\n测试选项:")
    print("1. 检测放置区域")
    print("2. 执行放置")
    print("3. 验证放置结果")
    print("4. 返回初始位置")
    print("q. 退出")
    
    while True:
        cmd = input("\n请选择: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == '1':
            place_area = placing.detect_place_area()
            if place_area:
                print(f"检测到放置区域: 中心={place_area['image_center']}, 有效={place_area['valid']}")
            else:
                print("未检测到放置区域")
        elif cmd == '2':
            placing.place_object()
        elif cmd == '3':
            result = placing.verify_placement()
            print(f"放置验证: {'成功' if result else '失败'}")
        elif cmd == '4':
            arm.move_to_neutral()
    
    camera.release()
    arm.disconnect()
    print("\n测试完成")


if __name__ == "__main__":
    main()
