"""
腕部摄像头模块 - 简化版
只检测红色正方体（边长22mm）
"""

import numpy as np
from typing import Optional, Tuple, Dict


class WristCamera:
    """腕部摄像头类"""
    
    REAL_OBJECT_SIZE = 0.022
    
    def __init__(self, camera_id: int = 0, resolution: Tuple[int, int] = (640, 480)):
        self.camera_id = camera_id
        self.resolution = resolution
        self.cap = None
        self.cv2 = None
        self.initialized = False
        
        self._init_camera()
        
        self.red_hsv_low1 = np.array([0, 100, 100])
        self.red_hsv_high1 = np.array([10, 255, 255])
        self.red_hsv_low2 = np.array([160, 100, 100])
        self.red_hsv_high2 = np.array([180, 255, 255])
        
    def _init_camera(self):
        try:
            import cv2
            self.cv2 = cv2
            
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                print(f"警告: 无法打开摄像头 {self.camera_id}")
                self.initialized = False
                return
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            self.initialized = True
            print(f"✓ 摄像头初始化成功: {self.resolution[0]}x{self.resolution[1]}")
            
        except ImportError:
            print("警告: OpenCV未安装，摄像头功能不可用")
            self.initialized = False
    
    def is_ready(self) -> bool:
        return self.initialized and self.cap is not None and self.cap.isOpened()
    
    def get_frame(self) -> Optional[np.ndarray]:
        if not self.is_ready():
            return None
        
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        return frame
    
    def detect_red_cube(self, frame: np.ndarray = None, min_area: int = 200) -> Optional[Dict]:
        """
        检测红色正方体
        
        Returns:
            Dict with keys:
            - 'center': (cx, cy) 图像中心坐标
            - 'pixel_size': 物块在图像中的像素尺寸（用于深度估计）
            - 'bbox': (x, y, w, h) 边界框
            - 'area': 面积
        """
        if frame is None:
            frame = self.get_frame()
        
        if frame is None:
            return None
        
        hsv = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2HSV)
        
        mask1 = self.cv2.inRange(hsv, self.red_hsv_low1, self.red_hsv_high1)
        mask2 = self.cv2.inRange(hsv, self.red_hsv_low2, self.red_hsv_high2)
        mask = self.cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_OPEN, kernel)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_CLOSE, kernel)
        
        contours, _ = self.cv2.findContours(mask, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        valid_cubes = []
        for contour in contours:
            area = self.cv2.contourArea(contour)
            if area < min_area:
                continue
            
            peri = self.cv2.arcLength(contour, True)
            approx = self.cv2.approxPolyDP(contour, 0.04 * peri, True)
            
            if len(approx) >= 4 and len(approx) <= 6:
                x, y, w, h = self.cv2.boundingRect(contour)
                aspect_ratio = w / h if h > 0 else 1
                
                if 0.6 < aspect_ratio < 1.7:
                    M = self.cv2.moments(contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        pixel_size = (w + h) / 2
                        
                        valid_cubes.append({
                            'center': (cx, cy),
                            'pixel_size': pixel_size,
                            'bbox': (x, y, w, h),
                            'area': area,
                            'contour': contour
                        })
        
        if not valid_cubes:
            return None
        
        return max(valid_cubes, key=lambda x: x['area'])
    
    def detect_red_frame(self, frame: np.ndarray = None) -> Optional[Dict]:
        """检测红色方框（放置区域）"""
        if frame is None:
            frame = self.get_frame()
        
        if frame is None:
            return None
        
        hsv = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2HSV)
        
        mask1 = self.cv2.inRange(hsv, self.red_hsv_low1, self.red_hsv_high1)
        mask2 = self.cv2.inRange(hsv, self.red_hsv_low2, self.red_hsv_high2)
        mask = self.cv2.bitwise_or(mask1, mask2)
        
        contours, _ = self.cv2.findContours(mask, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE)
        
        frame_contours = []
        
        for contour in contours:
            peri = self.cv2.arcLength(contour, True)
            approx = self.cv2.approxPolyDP(contour, 0.02 * peri, True)
            
            if len(approx) == 4:
                area = self.cv2.contourArea(contour)
                if area > 2000:
                    x, y, w, h = self.cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h if h > 0 else 0
                    
                    if 0.7 < aspect_ratio < 1.4:
                        M = self.cv2.moments(contour)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            
                            frame_contours.append({
                                'center': (cx, cy),
                                'area': area,
                                'bbox': (x, y, w, h),
                                'pixel_size': (w + h) / 2
                            })
        
        if not frame_contours:
            return None
        
        return max(frame_contours, key=lambda x: x['area'])
    
    def draw_cube_detection(self, frame: np.ndarray, cube: Dict) -> np.ndarray:
        result = frame.copy()
        
        cx, cy = cube['center']
        x, y, w, h = cube['bbox']
        pixel_size = cube['pixel_size']
        
        self.cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
        self.cv2.circle(result, (cx, cy), 5, (0, 255, 0), -1)
        
        label = f"Red Cube ({pixel_size:.1f}px)"
        self.cv2.putText(result, label, (x, y - 10), self.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return result
    
    def draw_frame_detection(self, frame: np.ndarray, frame_obj: Dict) -> np.ndarray:
        result = frame.copy()
        
        cx, cy = frame_obj['center']
        x, y, w, h = frame_obj['bbox']
        
        self.cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 3)
        self.cv2.circle(result, (cx, cy), 8, (0, 0, 255), -1)
        
        self.cv2.putText(result, "Place Area", (x, y - 10), self.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return result
    
    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.initialized = False


def main():
    print("="*60)
    print("腕部摄像头测试 - 简化版")
    print("="*60)
    
    camera = WristCamera(camera_id=1)
    
    if not camera.is_ready():
        print("摄像头未就绪，退出测试")
        return
    
    print("\n按 'q' 退出")
    print("按 'c' 检测红色正方体")
    print("按 'f' 检测红色方框")
    
    while True:
        frame = camera.get_frame()
        if frame is None:
            continue
        
        display = frame.copy()
        
        key = camera.cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('c'):
            cube = camera.detect_red_cube(frame)
            if cube:
                print(f"\n检测到红色正方体:")
                print(f"  中心: {cube['center']}")
                print(f"  像素尺寸: {cube['pixel_size']:.1f}px")
                display = camera.draw_cube_detection(display, cube)
            else:
                print("\n未检测到红色正方体")
        elif key == ord('f'):
            frame_obj = camera.detect_red_frame(frame)
            if frame_obj:
                print(f"\n检测到红色方框: 中心={frame_obj['center']}")
                display = camera.draw_frame_detection(display, frame_obj)
            else:
                print("\n未检测到红色方框")
        
        camera.cv2.imshow("Wrist Camera", display)
    
    camera.release()
    camera.cv2.destroyAllWindows()
    print("\n测试完成")


if __name__ == "__main__":
    main()
