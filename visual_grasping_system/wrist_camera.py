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
                if area > 100:
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
    
    def detect_red_frame_rect(self, frame: np.ndarray = None,
                              min_area: int = 200, max_area: int = 200000) -> Optional[Dict]:
        if frame is None:
            frame = self.get_frame()
        if frame is None:
            return None

        hsv = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2HSV)
        mask1 = self.cv2.inRange(hsv, self.red_hsv_low1, self.red_hsv_high1)
        mask2 = self.cv2.inRange(hsv, self.red_hsv_low2, self.red_hsv_high2)
        mask = self.cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_CLOSE, kernel, iterations=3)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = self.cv2.findContours(mask, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        candidates = []
        for contour in contours:
            area = self.cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue
            candidates.append(contour)

        if not candidates:
            return None

        contour = max(candidates, key=lambda c: self.cv2.contourArea(c))

        rect = self.cv2.minAreaRect(contour)
        center = (int(rect[0][0]), int(rect[0][1]))
        corners = self.cv2.boxPoints(rect)

        w, h = rect[1]
        if min(w, h) < 5:
            return None
        aspect = max(w, h) / (min(w, h) + 1e-6)
        if aspect > 2.5:
            return None

        if not (0 <= center[0] < self.resolution[0] and 0 <= center[1] < self.resolution[1]):
            return None

        return {
            'center': center,
            'corners': corners,
            'diags': [(corners[0], corners[2]), (corners[1], corners[3])],
            'bounds': [],
            'area': area,
            'aspect': aspect
        }

    def draw_frame_rect(self, frame: np.ndarray, result: Dict) -> np.ndarray:
        d = frame.copy()
        cs = result['corners'].astype(int)
        self.cv2.drawContours(d, [cs], 0, (255, 0, 0), 2)
        self.cv2.line(d, tuple(cs[0]), tuple(cs[2]), (0, 255, 255), 1)
        self.cv2.line(d, tuple(cs[1]), tuple(cs[3]), (0, 255, 255), 1)
        cx, cy = result['center']
        self.cv2.circle(d, (cx, cy), 8, (0, 0, 255), -1)
        self.cv2.circle(d, (cx, cy), 12, (0, 0, 255), 2)
        area = result.get('area', '?')
        self.cv2.putText(d, f"({cx},{cy}) a={area}", (cx - 50, cy - 20),
                         self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        for i, c in enumerate(cs):
            self.cv2.circle(d, tuple(c), 5, (0, 255, 0), -1)
        return d

    def detect_red_frame_lines(self, frame: np.ndarray = None,
                               min_line_len: int = 10, max_line_gap: int = 8) -> Optional[Dict]:
        if frame is None:
            frame = self.get_frame()
        if frame is None:
            return None

        hsv = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2HSV)
        mask1 = self.cv2.inRange(hsv, self.red_hsv_low1, self.red_hsv_high1)
        mask2 = self.cv2.inRange(hsv, self.red_hsv_low2, self.red_hsv_high2)
        mask = self.cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((3, 3), np.uint8)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_CLOSE, kernel, iterations=2)

        edges = self.cv2.Canny(mask, 20, 60, apertureSize=3)

        lines = self.cv2.HoughLinesP(edges, 1, np.pi/180, threshold=15,
                                      minLineLength=min_line_len, maxLineGap=max_line_gap)
        if lines is None or len(lines) < 4:
            return None

        group_a = []
        group_b = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            dy = y2 - y1
            length = np.sqrt(dx*dx + dy*dy)
            if length < 1:
                continue
            angle_ratio = abs(dy) / length
            if angle_ratio < 0.5:
                group_a.append(line[0])
            else:
                group_b.append(line[0])

        if len(group_a) < 2 or len(group_b) < 2:
            return None

        group_a = sorted(group_a, key=lambda l: (l[1] + l[3]) / 2.0)
        group_b = sorted(group_b, key=lambda l: (l[0] + l[2]) / 2.0)

        la_top = self._extend_line(group_a[0])
        la_bot = self._extend_line(group_a[-1])
        lb_lef = self._extend_line(group_b[0])
        lb_rig = self._extend_line(group_b[-1])

        corners = []
        for la, lb in [(la_top, lb_lef), (la_top, lb_rig),
                        (la_bot, lb_lef), (la_bot, lb_rig)]:
            pt = self._intersect_lines(la, lb)
            if pt is not None:
                corners.append(pt)

        if len(corners) != 4:
            return None

        corners = np.array(corners, dtype=np.float32)
        center = np.mean(corners, axis=0)
        cx, cy = int(center[0]), int(center[1])

        if not (0 <= cx < self.resolution[0] and 0 <= cy < self.resolution[1]):
            return None

        return {
            'center': (cx, cy),
            'corners': corners,
            'diags': [(corners[0], corners[3]), (corners[1], corners[2])],
            'bounds': [la_top, la_bot, lb_lef, lb_rig]
        }

    @staticmethod
    def _extend_line(seg):
        x1, y1, x2, y2 = seg
        dx = x2 - x1
        dy = y2 - y1
        dn = np.sqrt(dx*dx + dy*dy)
        if dn < 1e-6:
            return (x1, y1, x1 + 1, y1)
        s = 5000.0 / dn
        return (x1 - dx*s, y1 - dy*s, x2 + dx*s, y2 + dy*s)

    @staticmethod
    def _intersect_lines(l1, l2):
        x1, y1, x2, y2 = l1
        x3, y3, x4, y4 = l2
        d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if abs(d) < 1e-8:
            return None
        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / d
        return (x1 + t*(x2-x1), y1 + t*(y3-y1))

    def draw_frame_lines(self, frame: np.ndarray, result: Dict) -> np.ndarray:
        d = frame.copy()
        cs = result['corners'].astype(int)
        for i in range(4):
            self.cv2.line(d, tuple(cs[i]), tuple(cs[(i+1)%4]), (255, 0, 0), 2)
        for di in result['diags']:
            self.cv2.line(d, tuple(di[0].astype(int)), tuple(di[1].astype(int)), (0, 255, 255), 1)
        cx, cy = result['center']
        self.cv2.circle(d, (cx, cy), 8, (0, 0, 255), -1)
        self.cv2.circle(d, (cx, cy), 12, (0, 0, 255), 2)
        self.cv2.putText(d, f"({cx},{cy})", (cx-40, cy-20),
                         self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        for i, c in enumerate(cs):
            self.cv2.circle(d, tuple(c), 5, (0, 255, 0), -1)
        return d

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
