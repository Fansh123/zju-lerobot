"""
SO-ARM101 机械臂控制 SDK (基于URDF)
===================================
使用yourdfpy解析URDF，实现精确的正运动学和逆运动学
"""

import serial
import time
import numpy as np
import os
import yourdfpy
from typing import List, Tuple, Optional


class FeetechSTS:
    """Feetech STS3215 舵机通信"""
    
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    
    REG_TORQUE_ENABLE = 40
    REG_GOAL_ACCELERATION = 44
    REG_GOAL_SPEED = 46
    REG_GOAL_POSITION = 42
    REG_PRESENT_POSITION = 56
    REG_PRESENT_VOLTAGE = 62
    REG_PRESENT_TEMPERATURE = 65
    
    POS_CENTER = 2048
    POS_MIN = 0
    POS_MAX = 4095
    
    def __init__(self, port, baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
    
    def connect(self):
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
            print(f"[ERROR] 连接失败: {e}")
            return False
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    @staticmethod
    def _checksum(data):
        return (~sum(data)) & 0xFF
    
    def _build_packet(self, servo_id, instruction, parameters=None):
        if parameters is None:
            parameters = []
        length = len(parameters) + 2
        data = [servo_id, length, instruction] + parameters
        chk = self._checksum(data)
        return bytes([0xFF, 0xFF] + data + [chk])
    
    def _send_only(self, packet):
        self.ser.reset_input_buffer()
        self.ser.write(packet)
    
    def _send_and_read(self, packet, timeout=0.1):
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        time.sleep(timeout)
        return self.ser.read(50)
    
    def ping(self, servo_id):
        packet = self._build_packet(servo_id, self.INST_PING)
        response = self._send_and_read(packet)
        return response and len(response) >= 6 and response[2] == servo_id
    
    def enable_torque(self, servo_id, enable=True):
        packet = self._build_packet(
            servo_id, self.INST_WRITE,
            [self.REG_TORQUE_ENABLE, 1 if enable else 0]
        )
        self._send_only(packet)
    
    def set_acceleration(self, servo_id, acceleration):
        packet = self._build_packet(
            servo_id, self.INST_WRITE,
            [self.REG_GOAL_ACCELERATION, acceleration & 0xFF]
        )
        self._send_only(packet)
    
    def set_speed(self, servo_id, speed):
        spd_low = speed & 0xFF
        spd_high = (speed >> 8) & 0xFF
        packet = self._build_packet(
            servo_id, self.INST_WRITE,
            [self.REG_GOAL_SPEED, spd_low, spd_high]
        )
        self._send_only(packet)
    
    def set_position(self, servo_id, position, speed=None):
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
    
    def read_position(self, servo_id):
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
    
    def read_voltage(self, servo_id):
        packet = self._build_packet(
            servo_id, self.INST_READ,
            [self.REG_PRESENT_VOLTAGE, 1]
        )
        response = self._send_and_read(packet)
        if response and len(response) >= 7:
            return response[5] / 10.0
        return None
    
    def read_temperature(self, servo_id):
        packet = self._build_packet(
            servo_id, self.INST_READ,
            [self.REG_PRESENT_TEMPERATURE, 1]
        )
        response = self._send_and_read(packet)
        if response and len(response) >= 7:
            return response[5]
        return None
    
    @staticmethod
    def angle_to_position(angle_rad):
        return int(angle_rad / (2 * np.pi) * 4096 + FeetechSTS.POS_CENTER)
    
    @staticmethod
    def position_to_angle(position):
        return (position - FeetechSTS.POS_CENTER) / 4096 * 2 * np.pi


class SOARM101Controller:
    """SO-ARM101 机械臂控制器 (基于URDF)"""
    
    JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 
                   'wrist_flex', 'wrist_roll', 'gripper']
    
    JOINT_LIMITS = np.array([
        [-1.92, 1.92], [-1.75, 1.75], [-1.69, 1.69],
        [-1.66, 1.66], [-2.74, 2.84], [-1.06, 1.22]
    ])
    
    GRIPPER_CLOSE_ANGLE = -1.0
    GRIPPER_OPEN_ANGLE = 1.1
    
    DEFAULT_SPEED = 500
    DEFAULT_ACCELERATION = 50
    
    def __init__(self, port: str = 'COM18', urdf_path: str = None):
        self.bus = FeetechSTS(port, baudrate=1000000)
        self.connected = False
        self.current_positions = np.full(6, FeetechSTS.POS_CENTER)
        self._speed = self.DEFAULT_SPEED
        self._acceleration = self.DEFAULT_ACCELERATION
        
        self.urdf = None
        self.joint_names_urdf = None
        self.ee_link_name = "gripper_frame_link"
        
        if urdf_path:
            self._load_urdf(urdf_path)
    
    def _load_urdf(self, urdf_path: str):
        try:
            print(f"[URDF] 加载URDF: {urdf_path}")
            self.urdf = yourdfpy.URDF.load(urdf_path)
            
            self.joint_names_urdf = list(self.urdf.actuated_joint_names)
            self.joint_names_urdf = [j for j in self.joint_names_urdf if 'gripper' not in j.lower()]
            
            self.joint_order = {
                'shoulder_pan': 0,
                'shoulder_lift': 1,
                'elbow_flex': 2,
                'wrist_flex': 3,
                'wrist_roll': 4,
            }
            
            print(f"[URDF] 驱动关节: {self.urdf.actuated_joint_names}")
            print(f"[URDF] 使用关节: {self.joint_names_urdf}")
            print(f"[URDF] 末端链接: {self.ee_link_name}")
            print(f"[URDF] 加载成功!")
            
        except Exception as e:
            print(f"[ERROR] 加载URDF失败: {e}")
            import traceback
            traceback.print_exc()
            self.urdf = None
    
    def connect(self) -> bool:
        print(f"[CONNECT] 尝试连接到 {self.bus.port} @ {self.bus.baudrate}")
        if not self.bus.connect():
            print("[CONNECT] 串口连接失败")
            return False
        
        print("[CONNECT] 串口连接成功, 开始配置舵机...")
        for i in range(6):
            self.bus.enable_torque(i + 1, True)
            self.bus.set_acceleration(i + 1, self._acceleration)
            self.bus.set_speed(i + 1, self._speed)
            time.sleep(0.03)
        
        self.connected = True
        print("[CONNECT] ✓ 机械臂已连接")
        return True
    
    def disconnect(self):
        for i in range(6):
            self.bus.enable_torque(i + 1, False)
            time.sleep(0.02)
        self.bus.disconnect()
        self.connected = False
        print("✓ 机械臂已断开")
    
    def scan_servos(self) -> dict:
        result = {}
        print("\n[SCAN] 扫描舵机...")
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
                if pos:
                    angle = FeetechSTS.position_to_angle(pos)
                    print(f"  ✓ 舵机 ID={i}: 位置={pos}, 角度={np.degrees(angle):.1f}°, 电压={voltage}V, 温度={temp}°C")
                else:
                    print(f"  ✓ 舵机 ID={i}: 在线 (位置读取失败)")
            else:
                result[i] = {'online': False}
                print(f"  ✗ 舵机 ID={i}: 无响应")
        return result
    
    def get_joint_angles(self) -> np.ndarray:
        angles = np.zeros(6)
        for i in range(6):
            pos = self.bus.read_position(i + 1)
            if pos is not None:
                self.current_positions[i] = pos
                angles[i] = FeetechSTS.position_to_angle(pos)
        return angles
    
    def set_joint_angles(self, angles, duration: float = 1.0) -> bool:
        if not self.connected:
            print("[ERROR] 机械臂未连接")
            return False
        
        angles = np.array(angles, dtype=float)
        for i in range(6):
            angles[i] = np.clip(angles[i], self.JOINT_LIMITS[i][0], self.JOINT_LIMITS[i][1])
        
        target_positions = np.array([FeetechSTS.angle_to_position(a) for a in angles])
        start_positions = self.current_positions.copy()
        
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
    
    def forward_kinematics(self, angles: np.ndarray = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        正运动学: 从关节角度计算末端位置和姿态
        
        Args:
            angles: 5个关节角度 (弧度), 顺序为 [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
            
        Returns:
            position: 末端位置 (米) [x, y, z]
            rotation: 末端姿态 (旋转矩阵) [3x3]
        """
        if not self.urdf:
            print("[ERROR] URDF未加载")
            return None, None
        
        if angles is None:
            angles = self.get_joint_angles()[:5]
        
        urdf_order = [0, 0, 0, 0, 0, 0]
        for name, idx in self.joint_order.items():
            urdf_idx = list(self.urdf.actuated_joint_names).index(name)
            urdf_order[urdf_idx] = angles[idx]
        
        cfg = np.array(urdf_order)
        
        self.urdf.update_cfg(cfg)
        
        link_frame = self.urdf.get_transform(self.ee_link_name)
        
        position = link_frame[:3, 3].copy()
        rotation = link_frame[:3, :3].copy()
        
        return position, rotation
    
    def inverse_kinematics(self, target_pos: np.ndarray, q_init: np.ndarray = None,
                           max_iter: int = 100, eps: float = 1e-4, 
                           damping: float = 0.1) -> Optional[np.ndarray]:
        """
        逆运动学: 从目标位置计算关节角度 (阻尼最小二乘法)
        
        Args:
            target_pos: 目标位置 (米) [x, y, z]
            q_init: 初始关节角度 (5个)
            max_iter: 最大迭代次数
            eps: 收敛阈值 (米)
            damping: 阻尼系数
            
        Returns:
            关节角度 (5个) 或 None (无解)
        """
        if not self.urdf:
            print("[ERROR] URDF未加载")
            return None
        
        target = np.array(target_pos)
        
        if q_init is None:
            q = np.zeros(5)
        else:
            q = np.array(q_init[:5])
        
        lower = np.array([self.JOINT_LIMITS[i][0] for i in range(5)])
        upper = np.array([self.JOINT_LIMITS[i][1] for i in range(5)])
        
        for iteration in range(max_iter):
            pos, _ = self.forward_kinematics(q)
            error = target - pos
            distance = np.linalg.norm(error)
            
            if distance < eps:
                print(f"[IK] 收敛于第 {iteration} 次迭代, 误差={distance*1000:.2f}mm")
                return np.clip(q, lower, upper)
            
            delta = 1e-5
            J = np.zeros((3, 5))
            
            for j in range(5):
                q_plus = q.copy()
                q_plus[j] += delta
                pos_plus, _ = self.forward_kinematics(q_plus)
                J[:, j] = (pos_plus - pos) / delta
            
            try:
                JJT = J @ J.T
                damped = JJT + damping**2 * np.eye(3)
                dq = J.T @ np.linalg.solve(damped, error)
                q = q + dq
                q = np.clip(q, lower, upper)
            except np.linalg.LinAlgError:
                print("[IK] 雅可比矩阵奇异")
                return None
        
        final_pos, _ = self.forward_kinematics(q)
        final_error = np.linalg.norm(target - final_pos)
        print(f"[IK] 未收敛, 最终误差={final_error*1000:.2f}mm")
        return None if final_error > 0.05 else np.clip(q, lower, upper)
    
    def get_current_xyz(self) -> np.ndarray:
        """获取当前末端位置 (米)"""
        angles = self.get_joint_angles()
        pos, _ = self.forward_kinematics(angles[:5])
        return pos
    
    def move_to_neutral(self, duration: float = 1.0) -> bool:
        print("[MOVE] 移动到中立位置")
        return self.set_joint_angles([0, 0, 0, 0, 0, 0.0], duration)
    
    def move_to_home(self, duration: float = 1.5) -> bool:
        print("[MOVE] 移动到初始位置")
        return self.set_joint_angles([0, -1.5, 1.5, 0, 0, self.GRIPPER_OPEN_ANGLE], duration)
    
    def grasp(self, duration: float = 0.5) -> bool:
        print("[GRIP] 夹爪闭合")
        angles = self.get_joint_angles()
        angles[5] = self.GRIPPER_CLOSE_ANGLE
        return self.set_joint_angles(angles, duration)
    
    def release(self, duration: float = 0.5) -> bool:
        print("[GRIP] 夹爪打开")
        angles = self.get_joint_angles()
        angles[5] = self.GRIPPER_OPEN_ANGLE
        return self.set_joint_angles(angles, duration)
    
    def move_to_xyz(self, target_xyz: List[float], duration: float = 1.5) -> bool:
        """
        移动到目标笛卡尔坐标
        
        Args:
            target_xyz: 目标位置 (x, y, z) 米
            duration: 运动时间
            
        Returns:
            是否成功
        """
        if not self.connected or not self.urdf:
            print("[ERROR] move_to_xyz(): 机械臂未连接或URDF未加载")
            return False
        
        target = np.array(target_xyz)
        print(f"[MOVE_XYZ] 目标位置: ({target[0]*1000:.1f}, {target[1]*1000:.1f}, {target[2]*1000:.1f}) mm")
        
        current_angles = self.get_joint_angles()
        q_init = current_angles[:5]
        
        q_result = self.inverse_kinematics(target, q_init)
        if q_result is None:
            print("[MOVE_XYZ] 无法到达目标位置")
            return False
        
        full_angles = np.zeros(6)
        full_angles[:5] = q_result
        full_angles[5] = current_angles[5]
        
        return self.set_joint_angles(full_angles, duration)
    
    def move_relative(self, dx: float = 0, dy: float = 0, dz: float = 0, 
                      duration: float = 1.0) -> bool:
        """
        相对移动 (笛卡尔空间)
        
        Args:
            dx: X方向移动量 (米)
            dy: Y方向移动量 (米)
            dz: Z方向移动量 (米)
            duration: 运动时间
        """
        if not self.urdf:
            print("[ERROR] URDF未加载")
            return False
        
        current_pos = self.get_current_xyz()
        target_pos = current_pos + np.array([dx, dy, dz])
        
        print(f"[MOVE_REL] 相对移动: dx={dx*1000:.1f}mm, dy={dy*1000:.1f}mm, dz={dz*1000:.1f}mm")
        print(f"[MOVE_REL] 当前: ({current_pos[0]*1000:.1f}, {current_pos[1]*1000:.1f}, {current_pos[2]*1000:.1f}) mm")
        print(f"[MOVE_REL] 目标: ({target_pos[0]*1000:.1f}, {target_pos[1]*1000:.1f}, {target_pos[2]*1000:.1f}) mm")
        
        return self.move_to_xyz(target_pos, duration)


def main():
    import sys
    import argparse
    
    parser = argparse.ArgumentParser(description='SO-ARM101 机械臂控制')
    parser.add_argument('port', nargs='?', default='COM18', help='串口端口')
    parser.add_argument('--test', action='store_true', help='运行测试')
    parser.add_argument('--urdf', type=str, default=None, help='URDF文件路径')
    
    args = parser.parse_args()
    
    urdf_path = args.urdf
    if urdf_path is None:
        default_urdf = os.path.join(os.path.dirname(__file__), '..', 'SO-ARM100', 'Simulation', 'SO101', 'so101_new_calib.urdf')
        if os.path.exists(default_urdf):
            urdf_path = os.path.abspath(default_urdf)
    
    print("="*60)
    print("SO-ARM101 机械臂控制 SDK (URDF版)")
    print("="*60)
    
    arm = SOARM101Controller(args.port, urdf_path=urdf_path)
    
    if arm.connect():
        arm.scan_servos()
        
        if args.test:
            print("\n开始测试...")
            
            if arm.urdf:
                pos = arm.get_current_xyz()
                print(f"\n当前末端位置: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
            
            arm.move_to_neutral(duration=1.5)
            time.sleep(0.5)
            
            arm.grasp()
            time.sleep(1)
            
            arm.release()
            time.sleep(0.5)
            
            arm.move_to_neutral(duration=1.5)
            print("\n✓ 测试完成!")
        
        arm.disconnect()


if __name__ == "__main__":
    main()
