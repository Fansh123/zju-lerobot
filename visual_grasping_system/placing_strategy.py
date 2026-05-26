"""
放置策略模块 - 优化版
物块在桌面上（z=0），固定放置高度
"""

import numpy as np
import time
from typing import Optional, Dict, Tuple
from wrist_camera import WristCamera
from soarm101_sdk_urdf import SOARM101Controller
from coordinate_transformer import CoordinateTransformer


class PlacingStrategy:
    """放置策略类"""
    
    TABLE_HEIGHT = 0.0
    OBJECT_HEIGHT = 0.022
    PLACE_HEIGHT = 0.015
    
    def __init__(self, arm: SOARM101Controller, camera: WristCamera = None, 
                 camera_id: int = 0, calibration_dir: str = 'calibration_data'):
        self.arm = arm
        self.camera = camera if camera else WristCamera(camera_id)
        self.transformer = CoordinateTransformer(calibration_dir)
        
        self.approach_height = 0.08
        self.gripper_open = 1.1
    
    def detect_place_area(self, frame: np.ndarray = None) -> Optional[Dict]:
        """检测放置区域（红色方框）"""
        if frame is None:
            frame = self.camera.get_frame()
        
        if frame is None:
            return None
        
        frame_obj = self.camera.detect_red_frame(frame)
        
        if frame_obj is None:
            return None
        
        return {
            'image_center': frame_obj['center'],
            'pixel_size': frame_obj['pixel_size'],
            'bbox': frame_obj['bbox'],
            'area': frame_obj['area']
        }
    
    def place_object(self, place_area: Dict = None, frame: np.ndarray = None) -> bool:
        """
        放置物体到红色方框（z坐标固定为桌面高度）
        
        Args:
            place_area: 放置区域信息
            frame: 图像帧
            
        Returns:
            是否成功
        """
        if not self.arm.connected:
            print("错误: 机械臂未连接")
            return False
        
        if self.arm.urdf is None:
            print("错误: URDF未加载，无法使用笛卡尔控制")
            return False
        
        if place_area is None:
            place_area = self.detect_place_area(frame)
        
        if place_area is None:
            print("错误: 未检测到放置区域")
            return False
        
        target_x, target_y = self._calculate_target_xy(place_area)
        
        if target_x is None or target_y is None:
            print("错误: 无法计算放置位置")
            return False
        
        place_z = self.TABLE_HEIGHT + self.PLACE_HEIGHT
        approach_z = place_z + self.approach_height
        
        print(f"\n[放置流程]")
        print(f"目标位置: ({target_x*1000:.1f}, {target_y*1000:.1f}, {place_z*1000:.1f}) mm")
        
        try:
            print(f"1. 移动到放置区域上方 (z={approach_z*1000:.1f}mm)...")
            success = self.arm.move_to_xyz([target_x, target_y, approach_z], duration=2.0)
            if not success:
                print("  警告: 无法到达目标位置上方，尝试继续...")
            time.sleep(0.5)
            
            print(f"2. 下降到放置高度 (z={place_z*1000:.1f}mm)...")
            success = self.arm.move_to_xyz([target_x, target_y, place_z], duration=1.5)
            if not success:
                print("  警告: 无法到达放置高度，尝试继续...")
            time.sleep(0.5)
            
            print("3. 释放物体...")
            angles = self.arm.get_joint_angles()
            if angles is not None:
                angles[5] = self.gripper_open
                self.arm.set_joint_angles(angles, duration=0.5)
            time.sleep(0.5)
            
            print("4. 提升并离开...")
            current_pos = self.arm.get_current_xyz()
            if current_pos is not None:
                self.arm.move_to_xyz([current_pos[0], current_pos[1], current_pos[2] + 0.1], duration=1.0)
            time.sleep(0.5)
            
            print("✓ 放置完成")
            return True
            
        except Exception as e:
            print(f"放置失败: {e}")
            return False
    
    def _calculate_target_xy(self, place_area: Dict) -> Tuple[Optional[float], Optional[float]]:
        """计算放置目标x, y坐标（z固定为桌面高度）"""
        image_center = place_area.get('image_center')
        pixel_size = place_area.get('pixel_size')
        
        if image_center is None or pixel_size is None:
            return None, None
        
        if self.transformer.is_calibrated():
            ee_pos, ee_rot = self.arm.forward_kinematics()
            if ee_pos is not None and ee_rot is not None:
                depth = self.transformer.estimate_depth(pixel_size)
                print(f"[坐标转换] 深度估计: {depth*1000:.1f}mm (像素尺寸={pixel_size:.1f}px)")
                
                point_base = self.transformer.image_to_base(
                    image_center, depth, ee_pos, ee_rot
                )
                if point_base is not None:
                    print(f"[坐标转换] 图像{image_center} -> 基座({point_base[0]*1000:.1f}, {point_base[1]*1000:.1f}, {point_base[2]*1000:.1f})mm")
                    return (point_base[0], point_base[1])
        
        depth = 0.20
        cx, cy = image_center
        fx = 534.0
        x = (cx - 320) * depth / fx
        y = (cy - 240) * depth / fx
        return (x + 0.15, y)


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("放置策略测试 - 优化版（物块在桌面上）")
    print("="*60)
    
    arm = SOARM101Controller(port)
    camera = WristCamera(camera_id=1)
    
    if not arm.connect():
        print("无法连接机械臂")
        return
    
    if not camera.is_ready():
        print("警告: 摄像头未就绪")
    
    placing = PlacingStrategy(arm, camera)
    
    print("\n测试选项:")
    print("1. 检测放置区域")
    print("2. 执行放置")
    print("3. 返回初始位置")
    print("q. 退出")
    
    while True:
        cmd = input("\n请选择: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == '1':
            place_area = placing.detect_place_area()
            if place_area:
                print(f"检测到放置区域: 中心={place_area['image_center']}")
            else:
                print("未检测到放置区域")
        elif cmd == '2':
            placing.place_object()
        elif cmd == '3':
            arm.move_to_neutral()
    
    camera.release()
    arm.disconnect()
    print("\n测试完成")


if __name__ == "__main__":
    main()
