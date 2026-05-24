"""
腕部摄像头模块
支持树莓派摄像头和USB摄像头
提供图像采集、目标检测、红色方框检测等功能
"""

import numpy as np
from typing import Optional, Tuple, List, Dict
import time


class WristCamera:
    """腕部摄像头类"""
    
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
        
        self.focal_length = 500
        self.pixel_size = 0.001
        
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
    
    def detect_red_object(self, frame: np.ndarray = None, min_area: int = 500) -> Optional[Dict]:
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
        
        largest = max(contours, key=self.cv2.contourArea)
        area = self.cv2.contourArea(largest)
        
        if area < min_area:
            return None
        
        M = self.cv2.moments(largest)
        if M["m00"] == 0:
            return None
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        x, y, w, h = self.cv2.boundingRect(largest)
        
        return {
            'center': (cx, cy),
            'area': area,
            'bbox': (x, y, w, h),
            'contour': largest
        }
    
    def detect_red_frame(self, frame: np.ndarray = None, expected_size_mm: Tuple[float, float] = (100, 100)) -> Optional[Dict]:
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
                if area > 1000:
                    x, y, w, h = self.cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h if h > 0 else 0
                    
                    if 0.8 < aspect_ratio < 1.2:
                        M = self.cv2.moments(contour)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            
                            frame_contours.append({
                                'center': (cx, cy),
                                'area': area,
                                'bbox': (x, y, w, h),
                                'aspect_ratio': aspect_ratio,
                                'contour': contour,
                                'corners': approx
                            })
        
        if not frame_contours:
            return None
        
        return max(frame_contours, key=lambda x: x['area'])
    
    def classify_shape(self, contour) -> str:
        peri = self.cv2.arcLength(contour, True)
        approx = self.cv2.approxPolyDP(contour, 0.04 * peri, True)
        
        if len(approx) == 4:
            return 'square'
        elif len(approx) > 8:
            area = self.cv2.contourArea(contour)
            (x, y), radius = self.cv2.minEnclosingCircle(contour)
            circle_area = np.pi * radius * radius
            
            if abs(area - circle_area) / circle_area < 0.2:
                return 'cylinder'
        
        return 'unknown'
    
    def detect_objects(self, frame: np.ndarray = None, min_area: int = 500) -> List[Dict]:
        if frame is None:
            frame = self.get_frame()
        
        if frame is None:
            return []
        
        hsv = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2HSV)
        
        mask1 = self.cv2.inRange(hsv, self.red_hsv_low1, self.red_hsv_high1)
        mask2 = self.cv2.inRange(hsv, self.red_hsv_low2, self.red_hsv_high2)
        mask = self.cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_OPEN, kernel)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_CLOSE, kernel)
        
        contours, _ = self.cv2.findContours(mask, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        for contour in contours:
            area = self.cv2.contourArea(contour)
            if area < min_area:
                continue
            
            M = self.cv2.moments(contour)
            if M["m00"] == 0:
                continue
            
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            x, y, w, h = self.cv2.boundingRect(contour)
            
            shape = self.classify_shape(contour)
            
            objects.append({
                'center': (cx, cy),
                'area': area,
                'bbox': (x, y, w, h),
                'shape': shape,
                'contour': contour
            })
        
        return objects
    
    def get_object_pose_2d(self, obj: Dict) -> Dict:
        cx, cy = obj['center']
        x, y, w, h = obj['bbox']
        
        angle = 0
        if obj['shape'] == 'square' and 'contour' in obj:
            rect = self.cv2.minAreaRect(obj['contour'])
            angle = rect[-1]
        
        return {
            'center': (cx, cy),
            'width': w,
            'height': h,
            'angle': angle,
            'shape': obj['shape']
        }
    
    def suggest_grasp_type(self, obj: Dict) -> str:
        shape = obj.get('shape', 'unknown')
        
        if shape == 'cylinder':
            return 'horizontal'
        elif shape == 'square':
            _, _, w, h = obj['bbox']
            aspect_ratio = w / h if h > 0 else 1
            
            if aspect_ratio > 1.5 or aspect_ratio < 0.67:
                return 'horizontal'
            else:
                return 'vertical'
        
        return 'vertical'
    
    def draw_detection(self, frame: np.ndarray, obj: Dict, color: Tuple[int, int, int] = (0, 255, 0)) -> np.ndarray:
        result = frame.copy()
        
        cx, cy = obj['center']
        x, y, w, h = obj['bbox']
        
        self.cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
        self.cv2.circle(result, (cx, cy), 5, color, -1)
        
        shape = obj.get('shape', 'unknown')
        grasp = self.suggest_grasp_type(obj)
        label = f"{shape} ({grasp})"
        self.cv2.putText(result, label, (x, y - 10), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
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
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()


def main():
    print("="*60)
    print("腕部摄像头测试")
    print("="*60)
    
    camera = WristCamera(camera_id=0)
    
    if not camera.is_ready():
        print("摄像头未就绪，退出测试")
        return
    
    print("\n按 'q' 退出")
    print("按 's' 截图保存")
    print("按 'd' 检测物体")
    print("按 'f' 检测红色方框")
    
    while True:
        frame = camera.get_frame()
        if frame is None:
            continue
        
        display = frame.copy()
        
        key = camera.cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"capture_{int(time.time())}.jpg"
            camera.cv2.imwrite(filename, frame)
            print(f"截图保存: {filename}")
        elif key == ord('d'):
            objects = camera.detect_objects(frame)
            print(f"\n检测到 {len(objects)} 个物体:")
            for i, obj in enumerate(objects):
                print(f"  {i+1}. 形状: {obj['shape']}, 位置: {obj['center']}, 建议抓取: {camera.suggest_grasp_type(obj)}")
                display = camera.draw_detection(display, obj)
        elif key == ord('f'):
            frame_obj = camera.detect_red_frame(frame)
            if frame_obj:
                print(f"\n检测到红色方框: 中心={frame_obj['center']}, 尺寸={frame_obj['bbox'][2:]}")
                display = camera.draw_frame_detection(display, frame_obj)
            else:
                print("\n未检测到红色方框")
        
        camera.cv2.imshow("Wrist Camera", display)
    
    camera.release()
    camera.cv2.destroyAllWindows()
    print("\n测试完成")


if __name__ == "__main__":
    main()
