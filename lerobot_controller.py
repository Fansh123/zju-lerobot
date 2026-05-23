"""
LeRobot SDK 控制示例 - SO-ARM100/SO-ARM101 机械臂
官方文档: https://huggingface.co/docs/lerobot
"""

import numpy as np
import time


class LeRobotController:
    def __init__(self, robot_name="so100"):
        self.robot_name = robot_name
        self.robot = None
        self.connected = False

    def initialize(self):
        """初始化 LeRobot 连接"""
        try:
            from lerobot import Robot
            
            print(f"正在连接 {self.robot_name} 机械臂...")
            self.robot = Robot(self.robot_name)
            
            if self.robot is not None:
                self.connected = True
                print(f"成功连接到 {self.robot_name}!")
                print(f"关节数量: {self.robot.n_joints}")
                print(f"关节名称: {self.robot.joint_names}")
                return True
                
        except ImportError:
            print("错误: 未安装 lerobot 库")
            print("请运行: pip install lerobot")
            return False
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def get_joint_positions(self):
        """获取当前关节角度"""
        if not self.connected or self.robot is None:
            return None
        return self.robot.get_joint_positions()

    def set_joint_positions(self, positions, duration=1.0):
        """设置关节角度"""
        if not self.connected or self.robot is None:
            print("错误: 未连接机械臂")
            return False
            
        try:
            self.robot.set_joint_positions(positions, duration=duration)
            return True
        except Exception as e:
            print(f"设置关节角度失败: {e}")
            return False

    def move_to_neutral(self):
        """移动到中立位置"""
        if not self.connected:
            return False
        print("移动到中立位置")
        return self.set_joint_positions(np.zeros(self.robot.n_joints), duration=1.0)

    def grasp(self):
        """闭合夹爪"""
        if not self.connected or self.robot is None:
            return False
        print("夹爪闭合")
        try:
            self.robot.close_gripper()
            return True
        except Exception as e:
            print(f"夹爪控制失败: {e}")
            return False

    def release(self):
        """打开夹爪"""
        if not self.connected or self.robot is None:
            return False
        print("夹爪打开")
        try:
            self.robot.open_gripper()
            return True
        except Exception as e:
            print(f"夹爪控制失败: {e}")
            return False

    def move_to_grasp_position(self):
        """移动到抓取位置"""
        if not self.connected or self.robot is None:
            return False
            
        # SO-ARM100/SO-ARM101 典型抓取姿态
        grasp_q = np.array([0.0, 1.0, -1.5, 0.5, 0.0, 1.5])
        print(f"移动到抓取位置: {grasp_q}")
        return self.set_joint_positions(grasp_q, duration=1.5)

    def rotate_shoulder(self, angle_deg=90):
        """肩部旋转"""
        if not self.connected or self.robot is None:
            return False
            
        current_q = self.get_joint_positions()
        if current_q is None:
            return False
            
        target_q = current_q.copy()
        target_q[0] += np.radians(angle_deg)
        
        print(f"肩部旋转 {angle_deg}度")
        return self.set_joint_positions(target_q, duration=2.0)

    def close(self):
        """关闭连接"""
        if self.robot is not None:
            self.robot.close()
            self.connected = False
            print("已断开连接")


class VisionControlWithLeRobot:
    def __init__(self):
        self.controller = LeRobotController()
        self.vision_sim = None
        self.cap = None

    def initialize(self, use_real_camera=True):
        """初始化控制器和视觉"""
        print("=== LeRobot 视觉识别控制 ===")
        
        if not self.controller.initialize():
            return False
            
        try:
            from vision import initialize_camera
            self.cap = initialize_camera(0)
            print("摄像头初始化成功")
        except Exception as e:
            print(f"摄像头不可用，使用模拟视觉: {e}")
            from vision_simulation import VisionSimulator
            self.vision_sim = VisionSimulator(target_position=(0.3, 0.0, 0.15))
        
        return True

    def detect_red_block(self, timeout=15):
        """检测红色方块"""
        if self.cap is not None:
            from vision import detect_red_block
            print("正在检测红色方块...")
            start_time = time.time()
            
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                cx, cy, area, detected = detect_red_block(frame)
                elapsed = time.time() - start_time
                
                if elapsed > timeout:
                    print(f"\n检测超时 ({timeout}s)")
                    return False
                
                if detected:
                    print(f"\n检测到红色方块!")
                    return True
                
                print(f"\r检测中... ({elapsed:.1f}s)", end='', flush=True)
        else:
            if self.vision_sim:
                _, _, detected = self.vision_sim.wait_for_detection(timeout=timeout)
                return detected
        
        return False

    def run(self):
        """运行完整流程"""
        print("\n" + "="*60)
        print("开始 LeRobot 视觉识别演示")
        print("="*60)
        
        try:
            print("\n步骤1: 视觉识别红色方块")
            if not self.detect_red_block(timeout=15):
                print("未检测到红色方块")
                return
            
            print("\n步骤2: 移动到抓取位置")
            self.controller.move_to_grasp_position()
            
            print("\n步骤3: 闭合夹爪")
            self.controller.grasp()
            
            print("\n步骤4: 肩部旋转90度")
            self.controller.rotate_shoulder(90)
            
            print("\n步骤5: 等待5秒")
            time.sleep(5)
            
            print("\n步骤6: 打开夹爪")
            self.controller.release()
            
            print("\n步骤7: 返回初始位置")
            self.controller.move_to_neutral()
            
            print("\n" + "="*60)
            print("演示完成!")
            print("="*60)
            
        except KeyboardInterrupt:
            print("\n用户中断")
            self.controller.move_to_neutral()
        finally:
            if self.cap:
                self.cap.release()
            self.controller.close()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="LeRobot 视觉识别控制")
    parser.add_argument('--robot', default='so100', choices=['so100', 'so101'], 
                        help='机械臂型号')
    args = parser.parse_args()
    
    controller = VisionControlWithLeRobot()
    
    if not controller.initialize():
        print("初始化失败")
        return
    
    controller.run()


if __name__ == "__main__":
    main()