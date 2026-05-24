"""
物体检测模块
提供物体检测、形状分类、抓取点计算等功能
"""

import numpy as np
from typing import Optional, Tuple, List, Dict
from wrist_camera import WristCamera


class ObjectDetector:
    """物体检测类"""
    
    SHAPE_SQUARE = 'square'
    SHAPE_CYLINDER = 'cylinder'
    SHAPE_UNKNOWN = 'unknown'
    
    GRASP_HORIZONTAL = 'horizontal'
    GRASP_VERTICAL = 'vertical'
    
    def __init__(self, camera: WristCamera = None, camera_id: int = 0):
        self.camera = camera if camera else WristCamera(camera_id)
        
        self.min_object_area = 500
        self.max_object_area = 50000
        
        self.square_aspect_ratio_range = (0.7, 1.3)
        self.cylinder_circularity_threshold = 0.75
    
    def detect_all_objects(self, frame: np.ndarray = None) -> List[Dict]:
        if frame is None:
            frame = self.camera.get_frame()
        
        if frame is None:
            return []
        
        raw_objects = self.camera.detect_objects(frame, min_area=self.min_object_area)
        
        processed_objects = []
        for obj in raw_objects:
            if obj['area'] > self.max_object_area:
                continue
            
            processed = self._process_object(obj)
            processed_objects.append(processed)
        
        return processed_objects
    
    def _process_object(self, obj: Dict) -> Dict:
        result = obj.copy()
        
        result['shape'] = self._classify_shape_detailed(obj)
        result['grasp_type'] = self.suggest_grasp_type(obj)
        result['grasp_pose'] = self._calculate_grasp_pose(obj)
        result['orientation'] = self._calculate_orientation(obj)
        
        return result
    
    def _classify_shape_detailed(self, obj: Dict) -> str:
        contour = obj.get('contour')
        if contour is None:
            return self.SHAPE_UNKNOWN
        
        peri = self.camera.cv2.arcLength(contour, True)
        approx = self.camera.cv2.approxPolyDP(contour, 0.04 * peri, True)
        
        if len(approx) == 4:
            x, y, w, h = obj['bbox']
            aspect_ratio = w / h if h > 0 else 1
            
            if self.square_aspect_ratio_range[0] <= aspect_ratio <= self.square_aspect_ratio_range[1]:
                return self.SHAPE_SQUARE
            else:
                return self.SHAPE_SQUARE
        
        area = self.camera.cv2.contourArea(contour)
        hull = self.camera.cv2.convexHull(contour)
        hull_area = self.camera.cv2.contourArea(hull)
        
        if hull_area > 0:
            solidity = float(area) / hull_area
            
            (x, y), radius = self.camera.cv2.minEnclosingCircle(contour)
            circle_area = np.pi * radius * radius
            
            if circle_area > 0:
                circularity = area / circle_area
                
                if circularity > self.cylinder_circularity_threshold:
                    return self.SHAPE_CYLINDER
        
        return self.SHAPE_UNKNOWN
    
    def suggest_grasp_type(self, obj: Dict) -> str:
        shape = obj.get('shape', self.SHAPE_UNKNOWN)
        
        if shape == self.SHAPE_CYLINDER:
            return self.GRASP_HORIZONTAL
        
        if shape == self.SHAPE_SQUARE:
            _, _, w, h = obj['bbox']
            aspect_ratio = w / h if h > 0 else 1
            
            if aspect_ratio > 1.5 or aspect_ratio < 0.67:
                return self.GRASP_HORIZONTAL
            else:
                return self.GRASP_VERTICAL
        
        return self.GRASP_VERTICAL
    
    def _calculate_grasp_pose(self, obj: Dict) -> Dict:
        cx, cy = obj['center']
        x, y, w, h = obj['bbox']
        grasp_type = obj.get('grasp_type', self.GRASP_VERTICAL)
        
        if grasp_type == self.GRASP_HORIZONTAL:
            approach_angle = self._calculate_orientation(obj)
            grasp_width = min(w, h) * 0.8
        else:
            approach_angle = 0
            grasp_width = min(w, h) * 0.9
        
        return {
            'center': (cx, cy),
            'approach_angle': approach_angle,
            'grasp_width': grasp_width,
            'grasp_type': grasp_type
        }
    
    def _calculate_orientation(self, obj: Dict) -> float:
        contour = obj.get('contour')
        if contour is None:
            return 0
        
        rect = self.camera.cv2.minAreaRect(contour)
        angle = rect[-1]
        
        if angle > 45:
            angle = angle - 90
        elif angle < -45:
            angle = angle + 90
        
        return angle
    
    def get_closest_object(self, objects: List[Dict] = None, frame: np.ndarray = None) -> Optional[Dict]:
        if objects is None:
            objects = self.detect_all_objects(frame)
        
        if not objects:
            return None
        
        frame_center_x = self.camera.resolution[0] / 2
        frame_center_y = self.camera.resolution[1] / 2
        
        def distance_to_center(obj):
            cx, cy = obj['center']
            return np.sqrt((cx - frame_center_x)**2 + (cy - frame_center_y)**2)
        
        return min(objects, key=distance_to_center)
    
    def get_largest_object(self, objects: List[Dict] = None, frame: np.ndarray = None) -> Optional[Dict]:
        if objects is None:
            objects = self.detect_all_objects(frame)
        
        if not objects:
            return None
        
        return max(objects, key=lambda x: x['area'])
    
    def filter_by_shape(self, objects: List[Dict], shape: str) -> List[Dict]:
        return [obj for obj in objects if obj.get('shape') == shape]
    
    def filter_by_grasp_type(self, objects: List[Dict], grasp_type: str) -> List[Dict]:
        return [obj for obj in objects if obj.get('grasp_type') == grasp_type]
    
    def get_grasp_info(self, obj: Dict) -> Dict:
        return {
            'position': obj['center'],
            'shape': obj.get('shape', self.SHAPE_UNKNOWN),
            'grasp_type': obj.get('grasp_type', self.GRASP_VERTICAL),
            'grasp_width': obj.get('grasp_pose', {}).get('grasp_width', 30),
            'orientation': obj.get('orientation', 0),
            'bbox': obj['bbox']
        }
    
    def is_object_graspable(self, obj: Dict) -> bool:
        shape = obj.get('shape', self.SHAPE_UNKNOWN)
        
        if shape == self.SHAPE_UNKNOWN:
            return False
        
        _, _, w, h = obj['bbox']
        min_size = 20
        max_size = 150
        
        if w < min_size or h < min_size:
            return False
        
        if w > max_size or h > max_size:
            return False
        
        return True
    
    def draw_object_info(self, frame: np.ndarray, obj: Dict) -> np.ndarray:
        result = frame.copy()
        
        cx, cy = obj['center']
        x, y, w, h = obj['bbox']
        
        color = (0, 255, 0) if obj.get('grasp_type') == self.GRASP_VERTICAL else (255, 165, 0)
        
        self.camera.cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
        self.camera.cv2.circle(result, (cx, cy), 5, color, -1)
        
        shape = obj.get('shape', 'unknown')
        grasp_type = obj.get('grasp_type', 'unknown')
        graspable = "✓" if self.is_object_graspable(obj) else "✗"
        
        label = f"{shape} | {grasp_type} | {graspable}"
        self.camera.cv2.putText(result, label, (x, y - 10), 
                                self.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        grasp_pose = obj.get('grasp_pose', {})
        if grasp_pose:
            angle = grasp_pose.get('approach_angle', 0)
            angle_rad = np.radians(angle)
            line_length = 30
            
            x1 = int(cx - line_length * np.cos(angle_rad))
            y1 = int(cy - line_length * np.sin(angle_rad))
            x2 = int(cx + line_length * np.cos(angle_rad))
            y2 = int(cy + line_length * np.sin(angle_rad))
            
            self.camera.cv2.line(result, (x1, y1), (x2, y2), (255, 0, 255), 2)
        
        return result


def main():
    print("="*60)
    print("物体检测测试")
    print("="*60)
    
    detector = ObjectDetector(camera_id=0)
    
    if not detector.camera.is_ready():
        print("摄像头未就绪，退出测试")
        return
    
    print("\n按 'q' 退出")
    print("按 'd' 检测物体")
    print("按 'f' 过滤方形物体")
    print("按 'c' 过滤圆柱形物体")
    
    current_objects = []
    filter_shape = None
    
    while True:
        frame = detector.camera.get_frame()
        if frame is None:
            continue
        
        display = frame.copy()
        
        key = detector.camera.cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('d'):
            current_objects = detector.detect_all_objects(frame)
            filter_shape = None
            print(f"\n检测到 {len(current_objects)} 个物体:")
            for i, obj in enumerate(current_objects):
                graspable = "可抓取" if detector.is_object_graspable(obj) else "不可抓取"
                print(f"  {i+1}. 形状: {obj['shape']}, 抓取方式: {obj['grasp_type']}, {graspable}")
        elif key == ord('f'):
            filter_shape = ObjectDetector.SHAPE_SQUARE
            print(f"\n过滤: 只显示方形物体")
        elif key == ord('c'):
            filter_shape = ObjectDetector.SHAPE_CYLINDER
            print(f"\n过滤: 只显示圆柱形物体")
        
        objects_to_draw = current_objects
        if filter_shape:
            objects_to_draw = detector.filter_by_shape(current_objects, filter_shape)
        
        for obj in objects_to_draw:
            display = detector.draw_object_info(display, obj)
        
        info_text = f"Objects: {len(objects_to_draw)}"
        if filter_shape:
            info_text += f" (filtered: {filter_shape})"
        detector.camera.cv2.putText(display, info_text, (10, 30), 
                                    detector.camera.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        detector.camera.cv2.imshow("Object Detector", display)
    
    detector.camera.release()
    detector.camera.cv2.destroyAllWindows()
    print("\n测试完成")


if __name__ == "__main__":
    main()
