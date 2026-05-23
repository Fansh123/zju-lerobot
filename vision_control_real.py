import numpy as np
import time
import os

try:
    import pinocchio as pin
    from pinocchio.visualize import MeshcatVisualizer
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("警告: pinocchio未安装，将跳过可视化")

from vision import initialize_camera, detect_red_block
from vision_simulation import VisionSimulator, GripperObject


class VisionControlRealArm:
    def __init__(self, use_real_arm=False, serial_port='COM3'):
        self.use_real_arm = use_real_arm
        self.serial_port = serial_port
        self.real_arm = None
        self.viz = None
        self.model = None
        self.vision_sim = None
        self.red_block = None
        self.cap = None
        self.current_q = np.zeros(6)

    def initialize(self):
        print("=== 视觉识别机械臂控制 ===")
        print(f"模式: {'真实机械臂' if self.use_real_arm else '仿真'}")
        
        if self.use_real_arm:
            from real_arm_controller import RealArmController
            self.real_arm = RealArmController(port=self.serial_port)
            if not self.real_arm.initialize():
                print("错误: 无法连接真实机械臂")
                return False
            self.current_q = self.real_arm.current_q
        else:
            if PINOCCHIO_AVAILABLE:
                current_dir = os.path.dirname(os.path.abspath(__file__))
                urdf_path = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100", "so100.urdf")
                mesh_dir = os.path.join(current_dir, "SO-ARM100", "Simulation", "SO100")
                
                self.model, collision_model, visual_model = pin.buildModelsFromUrdf(
                    urdf_path, package_dirs=[mesh_dir]
                )
                
                self.viz = MeshcatVisualizer(self.model, collision_model, visual_model)
                self.viz.initViewer(open=True)
                self.viz.loadViewerModel()
                
                self.current_q = pin.neutral(self.model)
                self.viz.display(self.current_q)
                
                self.red_block = GripperObject(self.viz.viewer["objects"])
                self.red_block.spawn(position=(0.3, 0.0, 0.15), size=0.03)
            
            self.vision_sim = VisionSimulator(target_position=(0.3, 0.0, 0.15))
        
        try:
            self.cap = initialize_camera(0)
            print("摄像头初始化成功")
        except Exception as e:
            print(f"摄像头初始化失败: {e}")
            self.cap = None
        
        print("初始化完成!")
        return True

    def animate_movement(self, q_target, duration=1.0):
        q_start = self.current_q.copy()
        n_steps = 20
        
        for i in range(n_steps):
            t = i / n_steps
            t_smooth = t * t * t * (t * (t * 6 - 15) + 10)
            q_interp = q_start + (q_target - q_start) * t_smooth
            
            if self.use_real_arm and self.real_arm:
                for j in range(6):
                    self.real_arm.servo.set_joint_angle(j, q_interp[j])
            elif self.viz:
                self.viz.display(q_interp)
            
            self.current_q = q_interp.copy()
            time.sleep(duration / n_steps)

    def move_to_grasp_position(self):
        q_target = np.array([0.0, 1.2, -1.8, 0.3, 0.0, 2.0])
        print(f"移动到抓取位置: {q_target}")
        self.animate_movement(q_target, duration=1.5)

    def grasp_object(self):
        print("下降到抓取位置")
        q_down = self.current_q.copy()
        q_down[1] -= 0.15
        q_down[2] += 0.1
        self.animate_movement(q_down, duration=0.5)
        
        print("夹爪闭合")
        if self.use_real_arm and self.real_arm:
            self.real_arm.grasp()
        else:
            q_grip = q_down.copy()
            q_grip[5] = 0.0
            self.animate_movement(q_grip, duration=0.3)
            if self.red_block:
                self.red_block.attach_to_gripper(np.array([0.3, 0.0, 0.12]))
        
        print("提升物块")
        q_up = self.current_q.copy()
        q_up[1] += 0.15
        q_up[2] -= 0.1
        self.animate_movement(q_up, duration=0.5)

    def rotate_shoulder(self, angle_deg=90):
        angle_rad = np.radians(angle_deg)
        q_target = self.current_q.copy()
        q_target[0] += angle_rad
        
        if self.model:
            q_target[0] = np.clip(q_target[0], 
                                 self.model.lowerPositionLimit[0], 
                                 self.model.upperPositionLimit[0])
        
        print(f"肩部旋转 {angle_deg}度: {np.degrees(self.current_q[0]):.1f} -> {np.degrees(q_target[0]):.1f}")
        self.animate_movement(q_target, duration=2.0)
        
        if self.red_block and self.red_block.attached:
            self.red_block.update_position(np.array([0.3 * np.cos(q_target[0]), 
                                                     0.3 * np.sin(q_target[0]), 
                                                     0.18]))

    def release_object(self):
        print("下降到释放位置")
        q_down = self.current_q.copy()
        q_down[1] -= 0.15
        q_down[2] += 0.1
        self.animate_movement(q_down, duration=0.5)
        
        print("夹爪打开")
        if self.use_real_arm and self.real_arm:
            self.real_arm.release()
        else:
            q_release = q_down.copy()
            q_release[5] = 2.0
            self.animate_movement(q_release, duration=0.3)
            if self.red_block:
                self.red_block.detach(np.array([0.3 * np.cos(self.current_q[0]), 
                                                0.3 * np.sin(self.current_q[0]), 
                                                0.12]))
        
        time.sleep(0.5)
        
        print("提升")
        q_up = self.current_q.copy()
        q_up[1] += 0.15
        q_up[2] -= 0.1
        self.animate_movement(q_up, duration=0.5)

    def return_to_initial(self):
        q_target = np.zeros(6) if not self.model else pin.neutral(self.model)
        print("返回初始位置")
        self.animate_movement(q_target, duration=2.0)

    def detect_red_block(self, timeout=15):
        if self.cap is not None:
            print("正在使用真实摄像头检测红色方块...")
            start_time = time.time()
            
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("无法读取摄像头帧")
                    break
                
                cx, cy, area, detected = detect_red_block(frame)
                elapsed = time.time() - start_time
                
                if elapsed > timeout:
                    print(f"\n检测超时 ({timeout}s)")
                    return False
                
                if detected:
                    print(f"\n检测到红色方块! 位置: ({cx}, {cy}), 面积: {area}")
                    return True
                
                print(f"\r检测中... ({elapsed:.1f}s)", end='', flush=True)
        else:
            print("使用模拟视觉检测")
            if self.vision_sim:
                _, _, detected = self.vision_sim.wait_for_detection(timeout=timeout)
                return detected
        
        return False

    def run(self):
        print("\n" + "="*60)
        print("开始视觉识别与机械臂控制演示")
        print("="*60)
        
        try:
            print("\n步骤1: 视觉识别红色方块")
            if not self.detect_red_block(timeout=15):
                print("未检测到红色方块")
                return
            
            print("\n步骤2: 移动到抓取位置")
            self.move_to_grasp_position()
            
            print("\n步骤3: 夹取物块")
            self.grasp_object()
            
            print("\n步骤4: 肩部旋转90度")
            self.rotate_shoulder(90)
            
            print("\n步骤5: 等待5秒")
            time.sleep(5)
            
            print("\n步骤6: 放下物块")
            self.release_object()
            
            print("\n步骤7: 返回初始位置")
            self.return_to_initial()
            
            print("\n" + "="*60)
            print("演示完成!")
            print("="*60)
            
        except KeyboardInterrupt:
            print("\n用户中断")
            if self.use_real_arm and self.real_arm:
                self.real_arm.move_to_neutral()
        
        finally:
            if self.cap:
                self.cap.release()
            if self.use_real_arm and self.real_arm:
                self.real_arm.close()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="视觉识别机械臂控制")
    parser.add_argument('--real', action='store_true', help='使用真实机械臂')
    parser.add_argument('--port', default='COM3', help='串口端口')
    args = parser.parse_args()
    
    controller = VisionControlRealArm(use_real_arm=args.real, serial_port=args.port)
    
    if not controller.initialize():
        return
    
    controller.run()


if __name__ == "__main__":
    main()