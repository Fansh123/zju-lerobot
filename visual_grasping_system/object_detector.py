"""
物体检测模块 - 简化版
只检测红色正方体（边长22mm）
"""

import numpy as np
import os
import yaml
from typing import Optional, Dict
from wrist_camera import WristCamera


class ObjectDetector:
    """物体检测类 - 只检测红色正方体"""
    
    def __init__(self, camera: WristCamera = None, camera_id: int = None):
        self.camera = camera if camera else WristCamera(camera_id)
    
    def detect_cube(self, frame: np.ndarray = None) -> Optional[Dict]:
        """
        检测红色正方体
        
        Returns:
            Dict with keys:
            - 'center': (cx, cy) 图像中心坐标
            - 'pixel_size': 物块在图像中的像素尺寸
            - 'bbox': (x, y, w, h) 边界框
            - 'area': 面积
        """
        return self.camera.detect_red_cube(frame)
    
    def get_grasp_info(self, cube: Dict = None, frame: np.ndarray = None) -> Optional[Dict]:
        """
        获取抓取信息
        
        Returns:
            Dict with keys:
            - 'image_center': 图像中心坐标
            - 'pixel_size': 像素尺寸（用于深度估计）
            - 'grasp_type': 抓取类型（固定为 'horizontal'）
        """
        if cube is None:
            cube = self.detect_cube(frame)
        
        if cube is None:
            return None
        
        return {
            'image_center': cube['center'],
            'pixel_size': cube['pixel_size'],
            'grasp_type': 'horizontal',
            'bbox': cube['bbox']
        }
    
    def draw_detection(self, frame: np.ndarray, cube: Dict) -> np.ndarray:
        return self.camera.draw_cube_detection(frame, cube)


def main():
    print("="*60)
    print("物体检测测试 - 简化版")
    print("="*60)
    
    detector = ObjectDetector(camera_id=1)
    
    if not detector.camera.is_ready():
        print("摄像头未就绪，退出测试")
        return
    
    print("\n按 'q' 退出")
    print("按 'd' 检测红色正方体")
    
    while True:
        frame = detector.camera.get_frame()
        if frame is None:
            continue
        
        display = frame.copy()
        
        key = detector.camera.cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('d'):
            cube = detector.detect_cube(frame)
            if cube:
                grasp_info = detector.get_grasp_info(cube)
                print(f"\n检测到红色正方体:")
                print(f"  中心: {cube['center']}")
                print(f"  像素尺寸: {cube['pixel_size']:.1f}px")
                print(f"  抓取类型: {grasp_info['grasp_type']}")
                display = detector.draw_detection(display, cube)
            else:
                print("\n未检测到红色正方体")
        
        detector.camera.cv2.imshow("Object Detector", display)
    
    detector.camera.release()
    detector.camera.cv2.destroyAllWindows()
    print("\n测试完成")


if __name__ == "__main__":
    main()
