"""
视觉抓取主程序
整合摄像头、物体检测、抓取策略、放置策略
实现完整的视觉抓取-放置流程
"""

import numpy as np
import time
import argparse
from typing import Optional, Dict, List, Tuple

from soarm101_sdk import SOARM101Controller
from wrist_camera import WristCamera
from object_detector import ObjectDetector
from grasping_strategy import GraspingStrategy, GraspExecutor
from placing_strategy import PlacingStrategy


class VisualGrasping:
    """视觉抓取主类"""
    
    def __init__(self, arm_port: str = 'COM18', camera_id: int = 0,
                 calibration_file: str = None):
        self.arm_port = arm_port
        self.camera_id = camera_id
        self.calibration_file = calibration_file
        
        self.arm = SOARM101Controller(arm_port)
        self.camera = WristCamera(camera_id)
        self.detector = ObjectDetector(self.camera)
        self.grasp_executor = None
        self.placing_strategy = None
        
        self.connected = False
    
    def connect(self) -> bool:
        print("="*60)
        print("初始化视觉抓取系统")
        print("="*60)
        
        if not self.arm.connect():
            print("错误: 无法连接机械臂")
            return False
        
        if not self.camera.is_ready():
            print("警告: 摄像头未就绪，部分功能受限")
        else:
            print("✓ 摄像头就绪")
        
        self.grasp_executor = GraspExecutor(self.arm)
        self.placing_strategy = PlacingStrategy(self.arm, self.camera)
        
        self.connected = True
        print("✓ 系统初始化完成\n")
        return True
    
    def disconnect(self):
        if self.connected:
            self.arm.disconnect()
            self.camera.release()
            self.connected = False
            print("✓ 系统已断开连接")
    
    def scan_and_detect(self, frame: np.ndarray = None) -> Dict:
        if not self.camera.is_ready():
            print("错误: 摄像头未就绪")
            return {'objects': [], 'place_area': None}
        
        objects = self.detector.detect_all_objects(frame)
        place_area = self.placing_strategy.detect_place_area(frame)
        
        return {
            'objects': objects,
            'place_area': place_area,
            'object_count': len(objects)
        }
    
    def select_target_object(self, objects: List[Dict] = None, 
                             frame: np.ndarray = None,
                             shape_filter: str = None,
                             grasp_type_filter: str = None) -> Optional[Dict]:
        if objects is None:
            detection = self.scan_and_detect(frame)
            objects = detection['objects']
        
        if not objects:
            return None
        
        filtered = objects
        
        if shape_filter:
            filtered = self.detector.filter_by_shape(filtered, shape_filter)
        
        if grasp_type_filter:
            filtered = self.detector.filter_by_grasp_type(filtered, grasp_type_filter)
        
        if not filtered:
            return None
        
        graspable = [obj for obj in filtered if self.detector.is_object_graspable(obj)]
        
        if graspable:
            return self.detector.get_closest_object(graspable)
        
        return filtered[0]
    
    def grasp_object(self, target: Dict = None, grasp_type: str = 'auto',
                     frame: np.ndarray = None) -> bool:
        if not self.connected:
            print("错误: 系统未连接")
            return False
        
        if target is None:
            target = self.select_target_object(frame=frame)
        
        if target is None:
            print("错误: 未找到可抓取的物体")
            return False
        
        if grasp_type == 'auto':
            grasp_type = target.get('grasp_type', 'vertical')
        
        print(f"\n目标物体信息:")
        print(f"  形状: {target.get('shape', 'unknown')}")
        print(f"  位置: {target.get('center')}")
        print(f"  抓取方式: {grasp_type}")
        
        return self.grasp_executor.auto_grasp(target)
    
    def place_in_frame(self, frame: np.ndarray = None) -> bool:
        if not self.connected:
            print("错误: 系统未连接")
            return False
        
        return self.placing_strategy.place_object(frame=frame)
    
    def run_full_task(self, shape_filter: str = None, 
                      grasp_type: str = 'auto',
                      verify: bool = True) -> bool:
        if not self.connected:
            print("错误: 系统未连接")
            return False
        
        print("\n" + "="*60)
        print("开始执行完整抓取-放置任务")
        print("="*60)
        
        try:
            print("\n[步骤1] 移动到初始位置")
            self.arm.move_to_neutral(duration=1.5)
            time.sleep(0.5)
            
            print("\n[步骤2] 扫描工作区域")
            detection = self.scan_and_detect()
            
            if detection['object_count'] == 0:
                print("错误: 未检测到物体")
                return False
            
            print(f"检测到 {detection['object_count']} 个物体")
            
            if detection['place_area'] is None:
                print("警告: 未检测到放置区域")
            else:
                print(f"放置区域: {detection['place_area']['image_center']}")
            
            print("\n[步骤3] 选择目标物体")
            target = self.select_target_object(
                objects=detection['objects'],
                shape_filter=shape_filter
            )
            
            if target is None:
                print("错误: 未找到合适的目标物体")
                return False
            
            print(f"目标: 形状={target.get('shape')}, 抓取方式={target.get('grasp_type')}")
            
            print("\n[步骤4] 执行抓取")
            if not self.grasp_object(target, grasp_type):
                print("抓取失败")
                return False
            
            time.sleep(1)
            
            print("\n[步骤5] 检测放置区域")
            place_area = self.placing_strategy.detect_place_area()
            
            if place_area is None:
                print("错误: 未检测到放置区域")
                self.arm.move_to_neutral()
                return False
            
            print(f"放置区域中心: {place_area['image_center']}")
            
            print("\n[步骤6] 执行放置")
            if not self.place_in_frame():
                print("放置失败")
                return False
            
            time.sleep(1)
            
            if verify:
                print("\n[步骤7] 验证放置结果")
                result = self.placing_strategy.verify_placement()
                print(f"放置验证: {'成功' if result else '失败'}")
            
            print("\n[步骤8] 返回初始位置")
            self.arm.move_to_neutral(duration=1.5)
            
            print("\n" + "="*60)
            print("✓ 任务完成!")
            print("="*60)
            return True
            
        except KeyboardInterrupt:
            print("\n用户中断")
            self.arm.move_to_neutral()
            return False
        except Exception as e:
            print(f"\n任务执行错误: {e}")
            self.arm.move_to_neutral()
            return False
    
    def run_demo(self):
        if not self.connected:
            print("错误: 系统未连接")
            return
        
        print("\n" + "="*60)
        print("演示模式")
        print("="*60)
        
        print("\n1. 测试摄像头和物体检测")
        frame = self.camera.get_frame()
        if frame is not None:
            objects = self.detector.detect_all_objects(frame)
            print(f"   检测到 {len(objects)} 个物体")
            for i, obj in enumerate(objects):
                print(f"   - {i+1}: {obj['shape']}, {obj['grasp_type']}")
        
        print("\n2. 测试机械臂运动")
        self.arm.move_to_neutral(duration=1.5)
        time.sleep(1)
        
        print("\n3. 测试夹爪")
        self.arm.grasp()
        time.sleep(1)
        self.arm.release()
        time.sleep(0.5)
        
        print("\n✓ 演示完成")
    
    def interactive_mode(self):
        if not self.connected:
            print("错误: 系统未连接")
            return
        
        print("\n" + "="*60)
        print("交互模式")
        print("="*60)
        print("\n命令:")
        print("  1 - 扫描检测物体")
        print("  2 - 选择并抓取物体")
        print("  3 - 放置物体到红色方框")
        print("  4 - 执行完整任务")
        print("  5 - 返回初始位置")
        print("  6 - 测试横向抓取")
        print("  7 - 测试纵向抓取")
        print("  d - 演示模式")
        print("  q - 退出")
        
        while True:
            cmd = input("\n请输入命令: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == '1':
                detection = self.scan_and_detect()
                print(f"\n检测到 {detection['object_count']} 个物体")
                for i, obj in enumerate(detection['objects']):
                    print(f"  {i+1}. {obj['shape']} - {obj['grasp_type']} - 位置{obj['center']}")
                if detection['place_area']:
                    print(f"\n放置区域: {detection['place_area']['image_center']}")
            elif cmd == '2':
                self.grasp_object()
            elif cmd == '3':
                self.place_in_frame()
            elif cmd == '4':
                self.run_full_task()
            elif cmd == '5':
                self.arm.move_to_neutral()
            elif cmd == '6':
                self.grasp_executor.test_grasp_horizontal()
            elif cmd == '7':
                self.grasp_executor.test_grasp_vertical()
            elif cmd == 'd':
                self.run_demo()
        
        print("\n退出交互模式")
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


def main():
    parser = argparse.ArgumentParser(description='SO-ARM101 视觉抓取系统')
    parser.add_argument('port', nargs='?', default='COM18', help='串口端口')
    parser.add_argument('--camera', type=int, default=0, help='摄像头ID')
    parser.add_argument('--interactive', '-i', action='store_true', help='交互模式')
    parser.add_argument('--demo', action='store_true', help='演示模式')
    parser.add_argument('--task', action='store_true', help='执行完整任务')
    parser.add_argument('--shape', choices=['square', 'cylinder'], help='过滤物体形状')
    parser.add_argument('--grasp', choices=['horizontal', 'vertical', 'auto'], 
                        default='auto', help='抓取方式')
    
    args = parser.parse_args()
    
    vg = VisualGrasping(
        arm_port=args.port,
        camera_id=args.camera
    )
    
    if not vg.connect():
        return
    
    try:
        if args.interactive:
            vg.interactive_mode()
        elif args.demo:
            vg.run_demo()
        elif args.task:
            vg.run_full_task(
                shape_filter=args.shape,
                grasp_type=args.grasp
            )
        else:
            vg.interactive_mode()
    finally:
        vg.disconnect()


if __name__ == "__main__":
    main()
