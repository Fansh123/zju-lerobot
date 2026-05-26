"""
坐标转换模块
使用标定结果将图像坐标转换为机械臂基座坐标
支持基于物块尺寸的深度估计
"""

import numpy as np
import os
import yaml
from typing import Optional, Tuple


class CoordinateTransformer:
    """坐标转换器"""
    
    REAL_OBJECT_SIZE = 0.022
    
    def __init__(self, calibration_dir: str = 'calibration_data'):
        self.calibration_dir = calibration_dir
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.hand_eye_R = None
        self.hand_eye_t = None
        self.calibrated = False
        
        self._load_calibration()
    
    def _load_calibration(self):
        camera_file = os.path.join(self.calibration_dir, 'camera_params.yaml')
        hand_eye_file = os.path.join(self.calibration_dir, 'hand_eye.yaml')
        
        if os.path.exists(camera_file):
            with open(camera_file, 'r') as f:
                data = yaml.safe_load(f)
                self.camera_matrix = np.array(data['camera_matrix'])
                self.dist_coeffs = np.array(data['dist_coeffs'])
            print(f"[Transformer] 相机内参已加载")
            print(f"  焦距: fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}")
        
        if os.path.exists(hand_eye_file):
            with open(hand_eye_file, 'r') as f:
                data = yaml.safe_load(f)
                self.hand_eye_R = np.array(data['rotation'])
                self.hand_eye_t = np.array(data['translation'])
                self.calibrated = data.get('calibrated', False)
            
            if self.calibrated:
                print(f"[Transformer] 手眼标定已加载")
                print(f"  平移向量: {self.hand_eye_t.flatten()} mm")
            else:
                print(f"[Transformer] 警告: 手眼标定未完成")
    
    def is_calibrated(self) -> bool:
        return self.calibrated and self.camera_matrix is not None
    
    def estimate_depth(self, pixel_size: float, real_size: float = None) -> float:
        """
        基于物块尺寸估计深度
        
        公式: depth = real_size * focal_length / pixel_size
        
        Args:
            pixel_size: 物块在图像中的像素尺寸
            real_size: 物块实际尺寸 (米)，默认22mm
            
        Returns:
            估计的深度 (米)
        """
        if real_size is None:
            real_size = self.REAL_OBJECT_SIZE
        
        if self.camera_matrix is not None:
            fx = self.camera_matrix[0, 0]
        else:
            fx = 534.0
        
        depth = real_size * fx / pixel_size
        
        return depth
    
    def image_to_camera(self, image_point: Tuple[int, int], 
                        depth: float) -> np.ndarray:
        """
        图像坐标 -> 相机坐标
        
        Args:
            image_point: 图像坐标
            depth: 深度 (米)
            
        Returns:
            相机坐标系下的3D点 [x, y, z] (米)
        """
        if self.camera_matrix is None:
            return np.array([0, 0, depth])
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        u, v = image_point
        
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return np.array([x, y, z])
    
    def camera_to_end_effector(self, point_cam: np.ndarray) -> np.ndarray:
        """
        相机坐标 -> 末端坐标
        
        Args:
            point_cam: 相机坐标系下的点 (米)
            
        Returns:
            末端坐标系下的点 (米)
        """
        if self.hand_eye_R is None or self.hand_eye_t is None:
            return point_cam
        
        t_mm = self.hand_eye_t.flatten()
        
        point_ee = self.hand_eye_R @ point_cam + t_mm / 1000.0
        
        return point_ee
    
    def end_effector_to_base(self, point_ee: np.ndarray,
                             ee_position: np.ndarray,
                             ee_rotation: np.ndarray) -> np.ndarray:
        """
        末端坐标 -> 基座坐标
        
        Args:
            point_ee: 末端坐标系下的点 (米)
            ee_position: 末端位置 (米)
            ee_rotation: 末端旋转矩阵 (3x3)
            
        Returns:
            基座坐标系下的点 (米)
        """
        point_base = ee_rotation @ point_ee + ee_position
        
        return point_base
    
    def image_to_base(self, image_point: Tuple[int, int],
                      depth: float,
                      ee_position: np.ndarray,
                      ee_rotation: np.ndarray) -> Optional[np.ndarray]:
        """
        图像坐标 -> 基座坐标 (完整转换)
        
        Args:
            image_point: 图像坐标
            depth: 深度 (米)
            ee_position: 当前末端位置 (米)
            ee_rotation: 当前末端旋转矩阵 (3x3)
            
        Returns:
            基座坐标系下的点 (米) 或 None
        """
        if not self.is_calibrated():
            print("[Transformer] 错误: 未完成标定")
            return None
        
        point_cam = self.image_to_camera(image_point, depth)
        
        point_ee = self.camera_to_end_effector(point_cam)
        
        point_base = self.end_effector_to_base(point_ee, ee_position, ee_rotation)
        
        return point_base
    
    def get_grasp_position_3d(self, image_center: Tuple[int, int],
                               pixel_size: float,
                               ee_position: np.ndarray,
                               ee_rotation: np.ndarray,
                               real_size: float = None) -> Optional[np.ndarray]:
        """
        获取抓取点的3D位置（基于尺寸估计深度）
        
        Args:
            image_center: 图像中心坐标
            pixel_size: 物块像素尺寸
            ee_position: 当前末端位置 (米)
            ee_rotation: 当前末端旋转矩阵 (3x3)
            real_size: 物块实际尺寸 (米)
            
        Returns:
            抓取点在基座坐标系下的位置 (米)
        """
        depth = self.estimate_depth(pixel_size, real_size)
        print(f"[Transformer] 深度估计: {depth*1000:.1f}mm (像素尺寸={pixel_size:.1f}px)")
        
        return self.image_to_base(image_center, depth, ee_position, ee_rotation)


def test_transformer():
    print("="*50)
    print("坐标转换测试")
    print("="*50)
    
    transformer = CoordinateTransformer('calibration_data')
    
    if not transformer.is_calibrated():
        print("错误: 未完成标定")
        return
    
    print("\n测试深度估计:")
    pixel_sizes = [30, 40, 50, 60, 80, 100]
    for ps in pixel_sizes:
        depth = transformer.estimate_depth(ps)
        print(f"  像素尺寸 {ps}px -> 深度 {depth*1000:.1f}mm")
    
    print("\n测试图像坐标 -> 相机坐标:")
    image_point = (320, 240)
    depth = 0.25
    point_cam = transformer.image_to_camera(image_point, depth)
    print(f"  图像: {image_point}, 深度: {depth}m")
    print(f"  相机坐标: {point_cam}")
    
    print("\n测试完整转换 (假设末端位姿):")
    ee_pos = np.array([0.15, 0.0, 0.25])
    ee_rot = np.eye(3)
    pixel_size = 50
    point_base = transformer.get_grasp_position_3d(image_point, pixel_size, ee_pos, ee_rot)
    print(f"  末端位置: {ee_pos}")
    print(f"  基座坐标: {point_base}")


if __name__ == "__main__":
    test_transformer()
