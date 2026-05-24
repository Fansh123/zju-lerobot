"""
SO-ARM101 机械臂控制 SDK
========================
支持视觉识别红色方块、抓取、旋转、放置等完整功能

功能:
- 机械臂运动控制
- 视觉识别红色方块
- 夹爪控制
- 完整的抓取-旋转-放置流程
"""

import serial
import time
import numpy as np
from typing import Optional, List, Tuple


class FeetechSTS:
    """Feetech STS3215 舵机通信协议"""
    
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04
    INST_ACTION = 0x05
    INST_SYNC_WRITE = 0x83
    
    REG_TORQUE_ENABLE = 40
    REG_GOAL_ACCELERATION = 44
    REG_GOAL_SPEED = 46
    REG_GOAL_POSITION = 42
    REG_PRESENT_POSITION = 56
    REG_PRESENT_SPEED = 58
    REG_PRESENT_LOAD = 60
    REG_PRESENT_VOLTAGE = 62
    REG_PRESENT_TEMPERATURE = 63
    REG_MODE = 33
    
    POS_CENTER = 2048
    POS_MIN = 0
    POS_MAX = 4095
    
    def __init__(self, port: str, baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
    
    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                self.port, self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            time.sleep(0.3)
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    @staticmethod
    def _checksum(data: List[int]) -> int:
        return (~sum(data)) & 0xFF
    
    def _build_packet(self, servo_id: int, instruction: int, parameters: List[int] = None) -> bytes:
        if parameters is None:
            parameters = []
        length = len(parameters) + 2
        data = [servo_id, length, instruction] + parameters
        chk = self._checksum(data)
        return bytes([0xFF, 0xFF] + data + [chk])
    
    def _send_only(self, packet: bytes):
        self.ser.reset_input_buffer()
        self.ser.write(packet)
    
    def _send_and_read(self, packet: bytes, timeout: float = 0.1) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        time.sleep(timeout)
        return self.ser.read(50)
    
    def ping(self, servo_id: int) -> bool:
        packet = self._build_packet(servo_id, self.INST_PING)
        response = self._send_and_read(packet)
        return response and len(response) >= 6 and response[2] == servo_id
    
    def enable_torque(self, servo_id: int, enable: bool = True):
        packet = self._build_packet(
            servo_id, self.INST_WRITE,
            [self.REG_TORQUE_ENABLE, 1 if enable else 0]
        )
        self._send_only(packet)
    
    def set_acceleration(self, servo_id: int, acceleration: int):
        packet = self._build_packet(
            servo_id, self.INST_WRITE,
            [self.REG_GOAL_ACCELERATION, acceleration & 0xFF]
        )
        self._send_only(packet)
    
    def set_speed(self, servo_id: int, speed: int):
        spd_low = speed & 0xFF
        spd_high = (speed >> 8) & 0xFF
        packet = self._build_packet(
            servo_id, self.INST_WRITE,
            [self.REG_GOAL_SPEED, spd_low, spd_high]
        )
        self._send_only(packet)
    
    def set_position(self, servo_id: int, position: int, speed: int = None):
        position = max(self.POS_MIN, min(self.POS_MAX, int(position)))
        pos_low = position & 0xFF
        pos_high = (position >> 8) & 0xFF
        
        if speed is not None:
            spd_low = speed & 0xFF
            spd_high = (speed >> 8) & 0xFF
            packet = self._build_packet(
                servo_id, self.INST_WRITE,
                [self.REG_GOAL_POSITION, pos_low, pos_high, spd_low, spd_high]
            )
        else:
            packet = self._build_packet(
                servo_id, self.INST_WRITE,
                [self.REG_GOAL_POSITION, pos_low, pos_high]
            )
        self._send_only(packet)
    
    def read_position(self, servo_id: int) -> Optional[int]:
        packet = self._build_packet(
            servo_id, self.INST_READ,
            [self.REG_PRESENT_POSITION, 2]
        )
        response = self._send_and_read(packet, timeout=0.15)
        
        if response and len(response) >= 8:
            if response[0] == 0xFF and response[1] == 0xFF:
                length = response[3]
                if length >= 4:
                    pos_low = response[5]
                    pos_high = response[6]
                    return pos_low + (pos_high << 8)
        return None
    
    def read_voltage(self, servo_id: int) -> Optional[float]:
        packet = self._build_packet(
            servo_id, self.INST_READ,
            [self.REG_PRESENT_VOLTAGE, 1]
        )
        response = self._send_and_read(packet, timeout=0.15)
        
        if response and len(response) >= 6:
            return response[5] / 10.0
        return None
    
    def read_temperature(self, servo_id: int) -> Optional[int]:
        packet = self._build_packet(
            servo_id, self.INST_READ,
            [self.REG_PRESENT_TEMPERATURE, 1]
        )
        response = self._send_and_read(packet, timeout=0.15)
        
        if response and len(response) >= 6:
            return response[5]
        return None
    
    def sync_write_positions(self, positions: List[int], speed: int = None):
        addr = self.REG_GOAL_POSITION
        data_len = 4 if speed else 2
        
        params = [addr, data_len]
        for i, pos in enumerate(positions):
            pos = max(self.POS_MIN, min(self.POS_MAX, int(pos)))
            pos_low = pos & 0xFF
            pos_high = (pos >> 8) & 0xFF
            if speed:
                spd_low = speed & 0xFF
                spd_high = (speed >> 8) & 0xFF
                params.extend([i + 1, pos_low, pos_high, spd_low, spd_high])
            else:
                params.extend([i + 1, pos_low, pos_high])
        
        length = len(params) + 2
        data = [0xFE, length, self.INST_SYNC_WRITE] + params
        chk = self._checksum(data)
        packet = bytes([0xFF, 0xFF] + data + [chk])
        self._send_only(packet)
    
    @staticmethod
    def angle_to_position(angle_rad: float) -> int:
        return int(angle_rad / (2 * np.pi) * 4096 + FeetechSTS.POS_CENTER)
    
    @staticmethod
    def position_to_angle(position: int) -> float:
        return (position - FeetechSTS.POS_CENTER) / 4096 * 2 * np.pi


class SOARM101Controller:
    """SO-ARM101 机械臂控制器"""
    
    JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 
                   'wrist_flex', 'wrist_roll', 'gripper']
    
    JOINT_LIMITS = np.array([
        [-1.92, 1.92], [-1.75, 1.75], [-1.69, 1.69],
        [-1.66, 1.66], [-2.74, 2.84], [-0.17, 1.75]
    ])
    
    DEFAULT_SPEED = 500
    DEFAULT_ACCELERATION = 50
    
    def __init__(self, port: str = 'COM18'):
        self.bus = FeetechSTS(port, baudrate=1000000)
        self.connected = False
        self.current_positions = np.full(6, FeetechSTS.POS_CENTER)
        self._speed = self.DEFAULT_SPEED
        self._acceleration = self.DEFAULT_ACCELERATION
    
    def connect(self) -> bool:
        if not self.bus.connect():
            return False
        
        for i in range(6):
            self.bus.enable_torque(i + 1, True)
            self.bus.set_acceleration(i + 1, self._acceleration)
            self.bus.set_speed(i + 1, self._speed)
            time.sleep(0.03)
        
        self.connected = True
        print("✓ 机械臂已连接")
        return True
    
    def disconnect(self):
        if self.connected:
            for i in range(6):
                self.bus.enable_torque(i + 1, False)
                time.sleep(0.02)
            self.bus.disconnect()
            self.connected = False
            print("✓ 机械臂已断开")
    
    def set_speed(self, speed: int):
        self._speed = speed
        if self.connected:
            for i in range(6):
                self.bus.set_speed(i + 1, speed)
    
    def set_acceleration(self, acceleration: int):
        self._acceleration = acceleration
        if self.connected:
            for i in range(6):
                self.bus.set_acceleration(i + 1, acceleration)
    
    def get_joint_angles(self) -> np.ndarray:
        if not self.connected:
            return None
        angles = np.zeros(6)
        for i in range(6):
            pos = self.bus.read_position(i + 1)
            if pos is not None:
                self.current_positions[i] = pos
                angles[i] = FeetechSTS.position_to_angle(pos)
        return angles
    
    def get_joint_positions(self) -> np.ndarray:
        return self.current_positions.copy()
    
    def move_to_angles(self, angles: List[float], duration: float = 1.0, 
                       blocking: bool = True) -> bool:
        if not self.connected:
            return False
        
        angles = np.array(angles, dtype=float)
        for i in range(6):
            angles[i] = np.clip(angles[i], self.JOINT_LIMITS[i][0], self.JOINT_LIMITS[i][1])
        
        target_positions = np.array([FeetechSTS.angle_to_position(a) for a in angles])
        start_positions = self.current_positions.copy()
        
        if not blocking:
            self.bus.sync_write_positions(target_positions.tolist(), self._speed)
            self.current_positions = target_positions
            return True
        
        num_steps = max(10, int(duration * 30))
        dt = duration / num_steps
        
        for step in range(num_steps + 1):
            t = step / num_steps
            t_smooth = t * t * t * (t * (t * 6 - 15) + 10)
            
            current_pos = start_positions + (target_positions - start_positions) * t_smooth
            
            for j in range(6):
                self.bus.set_position(j + 1, int(current_pos[j]))
            
            self.current_positions = current_pos.astype(int)
            time.sleep(dt)
        
        return True
    
    def move_to_position(self, positions: List[int], duration: float = 1.0) -> bool:
        if not self.connected:
            return False
        
        positions = np.array(positions, dtype=int)
        for i in range(6):
            positions[i] = np.clip(positions[i], FeetechSTS.POS_MIN, FeetechSTS.POS_MAX)
        
        start_positions = self.current_positions.copy()
        target_positions = positions
        
        num_steps = max(10, int(duration * 30))
        dt = duration / num_steps
        
        for step in range(num_steps + 1):
            t = step / num_steps
            t_smooth = t * t * t * (t * (t * 6 - 15) + 10)
            
            current_pos = start_positions + (target_positions - start_positions) * t_smooth
            
            for j in range(6):
                self.bus.set_position(j + 1, int(current_pos[j]))
            
            self.current_positions = current_pos.astype(int)
            time.sleep(dt)
        
        return True
    
    def move_to_neutral(self, duration: float = 1.0) -> bool:
        return self.move_to_angles([0, 0, 0, 0, 0, 0.87], duration)
    
    def move_to_home(self, duration: float = 1.5) -> bool:
        return self.move_to_angles([0, -1.5, 1.5, 0, 0, 0.87], duration)
    
    def grasp(self, duration: float = 0.5) -> bool:
        angles = self.get_joint_angles()
        if angles is None:
            return False
        angles[5] = -0.1
        return self.move_to_angles(angles, duration)
    
    def release(self, duration: float = 0.5) -> bool:
        angles = self.get_joint_angles()
        if angles is None:
            return False
        angles[5] = 1.5
        return self.move_to_angles(angles, duration)
    
    def rotate_shoulder(self, angle_rad: float, duration: float = 1.0) -> bool:
        angles = self.get_joint_angles()
        if angles is None:
            return False
        angles[0] = np.clip(angle_rad, self.JOINT_LIMITS[0][0], self.JOINT_LIMITS[0][1])
        return self.move_to_angles(angles, duration)
    
    def wait(self, seconds: float):
        time.sleep(seconds)
    
    def scan_servos(self) -> dict:
        result = {}
        print("\n扫描舵机...")
        for i in range(1, 7):
            if self.bus.ping(i):
                pos = self.bus.read_position(i)
                voltage = self.bus.read_voltage(i)
                temp = self.bus.read_temperature(i)
                result[i] = {
                    'online': True,
                    'position': pos,
                    'angle': FeetechSTS.position_to_angle(pos) if pos else None,
                    'voltage': voltage,
                    'temperature': temp
                }
                status = f"位置={pos}" if pos else "位置读取失败"
                print(f"  ✓ 舵机 ID={i}: {status}")
            else:
                result[i] = {'online': False}
                print(f"  ✗ 舵机 ID={i}: 无响应")
        return result
    
    def get_voltage(self) -> float:
        return self.bus.read_voltage(1)
    
    def get_temperature(self, servo_id: int = 1) -> int:
        return self.bus.read_temperature(servo_id)
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


class VisionDetector:
    """视觉识别模块"""
    
    def __init__(self, camera_id: int = 0, use_simulation: bool = False):
        self.camera_id = camera_id
        self.use_simulation = use_simulation
        self.cap = None
        
        if not use_simulation:
            try:
                import cv2
                self.cv2 = cv2
                self.cap = cv2.VideoCapture(camera_id)
                if not self.cap.isOpened():
                    print("警告: 无法打开摄像头，使用模拟模式")
                    self.use_simulation = True
            except ImportError:
                print("警告: OpenCV未安装，使用模拟模式")
                self.use_simulation = True
    
    def detect_red_block(self) -> Optional[Tuple[int, int]]:
        if self.use_simulation:
            time.sleep(0.5)
            return (320, 240)
        
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        hsv = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = self.cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = self.cv2.inRange(hsv, lower_red2, upper_red2)
        mask = self.cv2.bitwise_or(mask1, mask2)
        
        contours, _ = self.cv2.findContours(mask, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=self.cv2.contourArea)
            if self.cv2.contourArea(largest) > 500:
                M = self.cv2.moments(largest)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)
        
        return None
    
    def is_centered(self, center: Tuple[int, int], threshold: int = 30) -> bool:
        if center is None:
            return False
        cx, cy = center
        return abs(cx - 320) < threshold and abs(cy - 240) < threshold
    
    def release(self):
        if self.cap:
            self.cap.release()
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()


class SOARM101TaskRunner:
    """SO-ARM101 任务执行器"""
    
    def __init__(self, port: str = 'COM18', camera_id: int = 0, use_simulation: bool = False):
        self.arm = SOARM101Controller(port)
        self.vision = VisionDetector(camera_id, use_simulation)
        self.use_simulation = use_simulation
    
    def connect(self) -> bool:
        return self.arm.connect()
    
    def disconnect(self):
        self.arm.disconnect()
        self.vision.release()
    
    def run_grasp_rotate_place(self, rotate_angle: float = np.pi/2, wait_time: float = 5.0) -> bool:
        """
        执行完整的抓取-旋转-放置任务
        
        Args:
            rotate_angle: 肩部旋转角度（弧度），默认90度
            wait_time: 旋转后等待时间（秒），默认5秒
        
        Returns:
            bool: 任务是否成功完成
        """
        if not self.arm.connected:
            print("错误: 机械臂未连接")
            return False
        
        print("\n" + "="*50)
        print("开始执行: 视觉识别 → 抓取 → 旋转 → 放置")
        print("="*50)
        
        try:
            print("\n[1/7] 移动到初始位置...")
            self.arm.move_to_neutral(duration=1.5)
            
            print("\n[2/7] 视觉识别红色方块...")
            center = self.vision.detect_red_block()
            if center:
                print(f"  检测到红色方块位置: {center}")
            else:
                print("  未检测到红色方块，使用默认位置")
            
            print("\n[3/7] 移动到抓取位置...")
            self.arm.move_to_angles([0, 0.8, -1.2, 0.4, 0, 1.5], duration=1.5)
            time.sleep(0.5)
            
            print("\n[4/7] 下降并抓取...")
            self.arm.move_to_angles([0, 0.3, -0.5, 0.2, 0, 1.5], duration=1.0)
            time.sleep(0.3)
            self.arm.grasp(duration=0.5)
            time.sleep(0.5)
            
            print("\n[5/7] 提升并旋转肩部...")
            self.arm.move_to_angles([0, 0.8, -1.2, 0.4, 0, -0.1], duration=1.0)
            time.sleep(0.3)
            
            print(f"  肩部旋转 {np.degrees(rotate_angle):.1f}°...")
            self.arm.rotate_shoulder(rotate_angle, duration=1.5)
            
            print(f"\n[6/7] 等待 {wait_time} 秒...")
            self.arm.wait(wait_time)
            
            print("\n[7/7] 放下物块...")
            self.arm.move_to_angles([rotate_angle, 0.3, -0.5, 0.2, 0, -0.1], duration=1.0)
            time.sleep(0.3)
            self.arm.release(duration=0.5)
            time.sleep(0.5)
            
            self.arm.move_to_neutral(duration=1.5)
            
            print("\n" + "="*50)
            print("✓ 任务完成!")
            print("="*50)
            return True
            
        except KeyboardInterrupt:
            print("\n用户中断")
            return False
        except Exception as e:
            print(f"\n错误: {e}")
            return False
    
    def run_simple_test(self):
        """运行简单测试"""
        if not self.arm.connected:
            print("错误: 机械臂未连接")
            return
        
        print("\n开始简单测试...")
        
        print("\n1. 移动到中立位置")
        self.arm.move_to_neutral(duration=1.5)
        time.sleep(0.5)
        
        print("\n2. 肩部旋转测试")
        self.arm.rotate_shoulder(0.5, duration=1.0)
        time.sleep(0.5)
        self.arm.rotate_shoulder(-0.5, duration=1.0)
        time.sleep(0.5)
        
        print("\n3. 夹爪测试")
        self.arm.grasp()
        time.sleep(1)
        self.arm.release()
        time.sleep(0.5)
        
        print("\n4. 返回中立位置")
        self.arm.move_to_neutral()
        
        print("\n✓ 测试完成!")
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


def main():
    import sys
    import argparse
    
    parser = argparse.ArgumentParser(description='SO-ARM101 机械臂控制')
    parser.add_argument('port', nargs='?', default='COM18', help='串口端口')
    parser.add_argument('--test', action='store_true', help='运行简单测试')
    parser.add_argument('--task', action='store_true', help='运行完整抓取任务')
    parser.add_argument('--rotate', type=float, default=90, help='旋转角度（度）')
    parser.add_argument('--wait', type=float, default=5, help='等待时间（秒）')
    parser.add_argument('--sim', action='store_true', help='使用模拟视觉')
    
    args = parser.parse_args()
    
    print("="*60)
    print("SO-ARM101 机械臂控制 SDK")
    print("="*60)
    
    runner = SOARM101TaskRunner(
        port=args.port,
        use_simulation=args.sim
    )
    
    if runner.connect():
        runner.arm.scan_servos()
        
        if args.task:
            rotate_rad = np.radians(args.rotate)
            runner.run_grasp_rotate_place(rotate_angle=rotate_rad, wait_time=args.wait)
        elif args.test:
            runner.run_simple_test()
        else:
            print("\n使用方法:")
            print("  python soarm101_sdk.py COM18 --test     # 简单测试")
            print("  python soarm101_sdk.py COM18 --task     # 完整抓取任务")
            print("  python soarm101_sdk.py COM18 --task --rotate 90 --wait 5")
        
        runner.disconnect()


if __name__ == "__main__":
    main()
