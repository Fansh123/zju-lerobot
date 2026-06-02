"""
视觉抓取主程序 - 视觉伺服版
使用视觉伺服方式抓取红色正方体
"""

import numpy as np
import time
import argparse
import os
from typing import Optional, Dict

from soarm101_sdk_urdf import SOARM101Controller
from wrist_camera import WristCamera
from object_detector import ObjectDetector
from grasping_strategy import GraspExecutor, VisualServoGrasp
from placing_strategy import PlacingStrategy


class VisualGrasping:
    """视觉抓取主类"""
    
    def __init__(self, arm_port: str = 'COM18', camera_id: int = 0):
        self.arm_port = arm_port
        self.camera_id = camera_id
        
        urdf_path = os.path.join(os.path.dirname(__file__), '..', 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
        if not os.path.exists(urdf_path):
            urdf_path = None
            print("[WARN] 未找到URDF文件")
        
        print("[INIT] 创建机械臂控制器...")
        self.arm = SOARM101Controller(arm_port, urdf_path=urdf_path)
        print("[INIT] 初始化摄像头...")
        self.camera = WristCamera(camera_id)
        print("[INIT] 创建物体检测器...")
        self.detector = ObjectDetector(self.camera)
        self.grasp_executor = None
        self.visual_servo = None
        self.placing_strategy = None
        
        self.connected = False
    
    def connect(self) -> bool:
        if not self.arm.connect():
            print("无法连接机械臂")
            return False
        
        if not self.camera.is_ready():
            print("警告: 摄像头未就绪")
        
        self.grasp_executor = GraspExecutor(self.arm, self.camera)
        self.visual_servo = VisualServoGrasp(self.arm, self.camera)
        self.placing_strategy = PlacingStrategy(self.arm, self.camera)
        
        self.connected = True
        return True
    
    def disconnect(self):
        self.camera.release()
        self.arm.disconnect()
        self.connected = False
    
    def detect_cube(self) -> Optional[Dict]:
        """检测红色正方体"""
        frame = self.camera.get_frame()
        if frame is None:
            print("无法获取图像")
            return None
        
        cube = self.detector.detect_cube(frame)
        
        if cube is None:
            print("未检测到红色正方体")
            return None
        
        print(f"检测到红色正方体:")
        print(f"  中心: {cube['center']}")
        print(f"  像素尺寸: {cube['pixel_size']:.1f}px")
        
        return self.detector.get_grasp_info(cube)
    
    def visual_servo_grasp(self) -> bool:
        """视觉伺服抓取"""
        return self.visual_servo.execute_grasp()
    
    def place_object(self) -> bool:
        """放置物体到红色方框"""
        return self.placing_strategy.place_object()
    
    def run_full_task(self) -> bool:
        """执行完整任务：检测 -> 视觉伺服抓取 -> 放置"""
        print("\n" + "="*60)
        print("开始执行完整任务")
        print("="*60)
        
        print("\n[步骤1] 视觉伺服抓取...")
        if not self.visual_servo_grasp():
            print("任务失败: 抓取失败")
            return False
        
        print("\n[步骤2] 视觉伺服放置...")
        if not self.place_object():
            print("任务失败: 放置失败")
            return False
        
        print("\n" + "="*60)
        print("✓ 完整任务执行成功!")
        print("="*60)
        return True
    
    def return_home(self):
        """返回初始位置"""
        self.arm.move_to_neutral()


def main():
    parser = argparse.ArgumentParser(description='视觉抓取 - 视觉伺服版')
    parser.add_argument('port', nargs='?', default='COM18', help='串口端口')
    parser.add_argument('--camera', type=int, default=1, help='摄像头ID')
    
    args = parser.parse_args()
    
    print("="*60)
    print("视觉抓取系统 - 视觉伺服版")
    print("使用视觉伺服方式抓取红色正方体")
    print("="*60)
    
    vg = VisualGrasping(args.port, args.camera)
    
    if not vg.connect():
        return
    
    print("\n命令选项:")
    print("1. 检测红色正方体")
    print("2. 视觉伺服抓取")
    print("3. 放置物体")
    print("4. 执行完整任务（抓取->放置）")
    print("5. 返回初始位置")
    print("q. 退出")
    
    while True:
        cmd = input("\n请输入命令: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == '1':
            vg.detect_cube()
        elif cmd == '2':
            vg.visual_servo_grasp()
        elif cmd == '3':
            vg.place_object()
        elif cmd == '4':
            vg.run_full_task()
        elif cmd == '5':
            vg.return_home()
        else:
            print("未知命令")
    
    vg.disconnect()
    print("\n程序结束")


if __name__ == "__main__":
    main()
