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
        [-1.66, 1.66], [-2.74, 2.84], [-1.06, 1.22]
    ])
    
    GRIPPER_CLOSE_ANGLE = -1.0
    GRIPPER_OPEN_ANGLE = 1.1
    
    DEFAULT_SPEED = 500
    DEFAULT_ACCELERATION = 50
    
    LINK_LENGTHS = {
        'd1': 0.0624,
        'a2': 0.11257,
        'a3': 0.1349,
        'd4': 0.0611,
        'd5': 0.0981,
    }
    
    def __init__(self, port: str = 'COM18'):
        self.bus = FeetechSTS(port, baudrate=1000000)
        self.connected = False
        self.current_positions = np.full(6, FeetechSTS.POS_CENTER)
        self._speed = self.DEFAULT_SPEED
        self._acceleration = self.DEFAULT_ACCELERATION
        self._current_xyz = None
    
    def connect(self) -> bool:
        print(f"[CONNECT] 尝试连接到 {self.bus.port} @ {self.bus.baudrate}")
        if not self.bus.connect():
            print("[CONNECT] 串口连接失败")
            return False
        
        print("[CONNECT] 串口连接成功, 开始配置舵机...")
        for i in range(6):
            print(f"[CONNECT] 配置舵机 {i+1}/{6}: 启用扭矩, 设置速度={self._speed}, 加速度={self._acceleration}")
            self.bus.enable_torque(i + 1, True)
            self.bus.set_acceleration(i + 1, self._acceleration)
            self.bus.set_speed(i + 1, self._speed)
            time.sleep(0.03)
        
        self.connected = True
        print("[CONNECT] ✓ 机械臂已连接")
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
    
    def forward_kinematics(self, angles: np.ndarray = None) -> Tuple[float, float, float]:
        """
        正运动学: 从关节角度计算末端位置 (基于SO101 URDF)
        
        坐标系定义:
        - 原点: 基座底部中心
        - X轴: 机械臂前方
        - Y轴: 机械臂左方  
        - Z轴: 垂直向上
        
        Args:
            angles: 5个关节角度 (弧度), 不包括夹爪. 如果为None则使用当前位置
            
        Returns:
            (x, y, z): 末端位置 (米)
        """
        if angles is None:
            angles = np.array([FeetechSTS.position_to_angle(p) for p in self.current_positions[:5]])
        
        q1, q2, q3, q4, q5 = angles[:5]
        
        L = self.LINK_LENGTHS
        
        def rot_z(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        def rot_y(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
        
        def rot_x(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
        
        def trans(x, y, z):
            return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
        
        T_base_shoulder = trans(0.0388, 0, 0.0624) @ rot_z(q1)
        
        T_shoulder_upper = trans(-0.0304, -0.0183, -0.0542) @ rot_x(-np.pi/2) @ rot_y(-np.pi/2) @ rot_z(q2)
        
        T_upper_lower = trans(-0.11257, -0.028, 0) @ rot_z(np.pi/2) @ rot_z(q3)
        
        T_lower_wrist = trans(-0.1349, 0.0052, 0) @ rot_z(-np.pi/2) @ rot_z(q4)
        
        T_wrist_gripper = trans(0, -0.0611, 0.0181) @ rot_x(np.pi/2) @ rot_y(0.0487) @ rot_z(np.pi) @ rot_z(q5)
        
        T_gripper_ee = trans(-0.0079, 0, -0.0981)
        
        T = T_base_shoulder @ T_shoulder_upper @ T_upper_lower @ T_lower_wrist @ T_wrist_gripper @ T_gripper_ee
        
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        
        return np.array([x, y, z])
    
    def inverse_kinematics(self, target_xyz: np.ndarray, current_angles: np.ndarray = None) -> Optional[np.ndarray]:
        """
        逆运动学: 从目标位置计算关节角度 (数值解法)
        
        Args:
            target_xyz: 目标位置 (x, y, z) 米
            current_angles: 当前关节角度 (作为初始猜测)
            
        Returns:
            关节角度 (5个) 或 None (无解)
        """
        if current_angles is None:
            current_angles = np.array([FeetechSTS.position_to_angle(p) for p in self.current_positions[:5]])
        
        target = np.array(target_xyz)
        angles = current_angles.copy()
        
        max_iterations = 100
        learning_rate = 0.5
        tolerance = 0.001
        
        for iteration in range(max_iterations):
            current_pos = self.forward_kinematics(angles)
            error = target - current_pos
            distance = np.linalg.norm(error)
            
            if distance < tolerance:
                print(f"[IK] 收敛于第 {iteration} 次迭代, 误差={distance*1000:.2f}mm")
                return angles
            
            delta = 0.001
            jacobian = np.zeros((3, 5))
            
            for j in range(5):
                angles_plus = angles.copy()
                angles_plus[j] += delta
                pos_plus = self.forward_kinematics(angles_plus)
                jacobian[:, j] = (pos_plus - current_pos) / delta
            
            try:
                jacobian_pinv = np.linalg.pinv(jacobian)
                delta_angles = jacobian_pinv @ error * learning_rate
                angles += delta_angles
                
                for i in range(5):
                    angles[i] = np.clip(angles[i], self.JOINT_LIMITS[i][0], self.JOINT_LIMITS[i][1])
                    
            except np.linalg.LinAlgError:
                print("[IK] 雅可比矩阵奇异，无法求解")
                return None
        
        final_error = np.linalg.norm(target - self.forward_kinematics(angles))
        print(f"[IK] 未收敛, 最终误差={final_error*1000:.2f}mm")
        return angles if final_error < 0.02 else None
    
    def get_current_xyz(self) -> np.ndarray:
        """获取当前末端位置 (米) - URDF坐标系"""
        angles = np.array([FeetechSTS.position_to_angle(p) for p in self.current_positions[:5]])
        self._current_xyz = self.forward_kinematics(angles)
        return self._current_xyz
    
    def get_user_position(self) -> dict:
        """
        获取用户友好的末端位置
        
        用户坐标系 (站在机械臂后方):
        - forward: 前方距离 (正值表示向前)
        - left: 左侧距离 (正值表示向左)
        - up: 高度 (正值表示向上)
        
        Returns:
            dict: {'forward': mm, 'left': mm, 'up': mm}
        """
        urdf_pos = self.get_current_xyz()
        return {
            'forward': urdf_pos[2] * 1000,   # Z轴 → 前
            'left': -urdf_pos[0] * 1000,     # -X轴 → 左
            'up': -urdf_pos[1] * 1000        # -Y轴 → 上
        }
    
    def move_to_xyz(self, target_xyz: List[float], duration: float = 1.5) -> bool:
        """
        移动到目标笛卡尔坐标 (URDF坐标系)
        
        Args:
            target_xyz: 目标位置 (x, y, z) 米 - URDF坐标系
            duration: 运动时间
            
        Returns:
            是否成功
        """
        if not self.connected:
            print("[ERROR] move_to_xyz(): 机械臂未连接")
            return False
        
        target = np.array(target_xyz)
        
        target_user = {
            'forward': target[2] * 1000,
            'left': -target[0] * 1000,
            'up': -target[1] * 1000
        }
        print(f"[MOVE_XYZ] 目标: 前{target_user['forward']:.1f}mm, 左{target_user['left']:.1f}mm, 上{target_user['up']:.1f}mm")
        
        current_angles = np.array([FeetechSTS.position_to_angle(p) for p in self.current_positions[:5]])
        current_pos = self.forward_kinematics(current_angles)
        
        target_angles = self.inverse_kinematics(target, current_angles)
        if target_angles is None:
            print("[MOVE_XYZ] 无法到达目标位置")
            return False
        
        full_angles = np.zeros(6)
        full_angles[:5] = target_angles
        full_angles[5] = FeetechSTS.position_to_angle(self.current_positions[5])
        
        result = self.move_to_angles(full_angles, duration)
        if result:
            self._current_xyz = target
        return result
    
    def move_relative(self, dx: float = 0, dy: float = 0, dz: float = 0, duration: float = 1.0) -> bool:
        """
        相对移动 (笛卡尔空间)
        
        用户坐标系 (站在机械臂后方):
        - dx: 前/后移动 (前为正)
        - dy: 左/右移动 (左为正)
        - dz: 上/下移动 (上为正)
        
        URDF坐标系映射:
        - 前/后 → Z轴
        - 左/右 → X轴 (反向)
        - 上/下 → Y轴 (反向)
        
        Args:
            dx: 前后方向移动量 (米)
            dy: 左右方向移动量 (米)
            dz: 上下方向移动量 (米)
            duration: 运动时间
            
        Returns:
            是否成功
        """
        if not self.connected:
            print("[ERROR] move_relative(): 机械臂未连接")
            return False
        
        current_pos = self.get_current_xyz()
        
        target_pos = current_pos + np.array([
            -dy,
            -dz,
            dx
        ])
        
        print(f"[MOVE_REL] 相对移动: 前{dx*1000:.1f}mm, 左{dy*1000:.1f}mm, 上{dz*1000:.1f}mm")
        
        user_pos = self.get_user_position()
        print(f"[MOVE_REL] 当前位置: 前{user_pos['forward']:.1f}mm, 左{user_pos['left']:.1f}mm, 上{user_pos['up']:.1f}mm")
        
        result = self.move_to_xyz(target_pos, duration)
        
        new_pos = self.get_user_position()
        print(f"[MOVE_REL] 移动后: 前{new_pos['forward']:.1f}mm, 左{new_pos['left']:.1f}mm, 上{new_pos['up']:.1f}mm")
        
        return result
    
    def move_up(self, distance: float = 0.05, duration: float = 1.0) -> bool:
        """向上移动 (米)"""
        return self.move_relative(dz=abs(distance), duration=duration)
    
    def move_down(self, distance: float = 0.05, duration: float = 1.0) -> bool:
        """向下移动 (米)"""
        return self.move_relative(dz=-abs(distance), duration=duration)
    
    def move_left(self, distance: float = 0.05, duration: float = 1.0) -> bool:
        """向左移动 (米)"""
        return self.move_relative(dy=abs(distance), duration=duration)
    
    def move_right(self, distance: float = 0.05, duration: float = 1.0) -> bool:
        """向右移动 (米)"""
        return self.move_relative(dy=-abs(distance), duration=duration)
    
    def move_forward(self, distance: float = 0.05, duration: float = 1.0) -> bool:
        """向前移动 (米)"""
        return self.move_relative(dx=abs(distance), duration=duration)
    
    def move_backward(self, distance: float = 0.05, duration: float = 1.0) -> bool:
        """向后移动 (米)"""
        return self.move_relative(dx=-abs(distance), duration=duration)
    
    def move_to_angles(self, angles: List[float], duration: float = 1.0, 
                       blocking: bool = True) -> bool:
        if not self.connected:
            print("[ERROR] move_to_angles(): 机械臂未连接")
            return False
        
        angles = np.array(angles, dtype=float)
        print(f"[MOVE] 目标角度: {[f'{np.degrees(a):.1f}°' for a in angles]}")
        
        for i in range(6):
            original = angles[i]
            angles[i] = np.clip(angles[i], self.JOINT_LIMITS[i][0], self.JOINT_LIMITS[i][1])
            if abs(original - angles[i]) > 0.01:
                print(f"[MOVE] 关节{i+1}({self.JOINT_NAMES[i]}) 角度被限制: {np.degrees(original):.1f}° -> {np.degrees(angles[i]):.1f}°")
        
        target_positions = np.array([FeetechSTS.angle_to_position(a) for a in angles])
        start_positions = self.current_positions.copy()
        
        print(f"[MOVE] 目标位置: {target_positions.tolist()}")
        print(f"[MOVE] 起始位置: {start_positions.tolist()}")
        print(f"[MOVE] duration={duration}s, blocking={blocking}")
        
        if not blocking:
            self.bus.sync_write_positions(target_positions.tolist(), self._speed)
            self.current_positions = target_positions
            print("[MOVE] 非阻塞模式: 命令已发送")
            return True
        
        num_steps = max(10, int(duration * 30))
        dt = duration / num_steps
        print(f"[MOVE] 开始运动, 共{num_steps}步, 每步{dt*1000:.1f}ms")
        
        for step in range(num_steps + 1):
            t = step / num_steps
            t_smooth = t * t * t * (t * (t * 6 - 15) + 10)
            
            current_pos = start_positions + (target_positions - start_positions) * t_smooth
            
            for j in range(6):
                self.bus.set_position(j + 1, int(current_pos[j]))
            
            self.current_positions = current_pos.astype(int)
            time.sleep(dt)
        
        print("[MOVE] 运动完成")
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
        print(f"[NEUTRAL] 移动到中立位置, duration={duration}s")
        print("[NEUTRAL] 目标姿态: 所有关节0°, 夹爪微开")
        result = self.move_to_angles([0, 0, 0, 0, 0, 0.0], duration)
        print(f"[NEUTRAL] {'成功' if result else '失败'}")
        return result
    
    def move_to_home(self, duration: float = 1.5) -> bool:
        print(f"[HOME] 移动到初始位置, duration={duration}s")
        print("[HOME] 目标姿态: 肩部0°, 肩抬-86°, 肘部86°, 腕部0°, 腕旋0°, 夹爪打开")
        result = self.move_to_angles([0, -1.5, 1.5, 0, 0, self.GRIPPER_OPEN_ANGLE], duration)
        print(f"[HOME] {'成功' if result else '失败'}")
        return result
    
    def grasp(self, duration: float = 0.5) -> bool:
        if not self.connected:
            print("[ERROR] grasp(): 机械臂未连接")
            return False
        
        print(f"[GRASP] 开始闭合夹爪, duration={duration}s")
        
        angles = np.array([FeetechSTS.position_to_angle(p) for p in self.current_positions])
        print(f"[GRASP] 当前角度: {[f'{np.degrees(a):.1f}°' for a in angles]}")
        
        angles[5] = self.GRIPPER_CLOSE_ANGLE
        print(f"[GRASP] 目标夹爪角度: {np.degrees(angles[5]):.1f}°")
        
        result = self.move_to_angles(angles, duration)
        print(f"[GRASP] 夹爪闭合 {'成功' if result else '失败'}")
        return result
    
    def release(self, duration: float = 0.5) -> bool:
        if not self.connected:
            print("[ERROR] release(): 机械臂未连接")
            return False
        
        print(f"[RELEASE] 开始打开夹爪, duration={duration}s")
        
        angles = np.array([FeetechSTS.position_to_angle(p) for p in self.current_positions])
        print(f"[RELEASE] 当前角度: {[f'{np.degrees(a):.1f}°' for a in angles]}")
        
        angles[5] = self.GRIPPER_OPEN_ANGLE
        print(f"[RELEASE] 目标夹爪角度: {np.degrees(angles[5]):.1f}°")
        
        result = self.move_to_angles(angles, duration)
        print(f"[RELEASE] 夹爪打开 {'成功' if result else '失败'}")
        return result
    
    def rotate_shoulder(self, angle_rad: float, duration: float = 1.0) -> bool:
        if not self.connected:
            print("[ERROR] rotate_shoulder(): 机械臂未连接")
            return False
        
        print(f"[ROTATE] 肩部旋转, 目标角度={np.degrees(angle_rad):.1f}°, duration={duration}s")
        
        angles = np.array([FeetechSTS.position_to_angle(p) for p in self.current_positions])
        print(f"[ROTATE] 当前肩部角度: {np.degrees(angles[0]):.1f}°")
        
        angles[0] = np.clip(angle_rad, self.JOINT_LIMITS[0][0], self.JOINT_LIMITS[0][1])
        print(f"[ROTATE] 限制后目标角度: {np.degrees(angles[0]):.1f}° (限制: ±{np.degrees(self.JOINT_LIMITS[0][1]):.1f}°)")
        
        result = self.move_to_angles(angles, duration)
        print(f"[ROTATE] 肩部旋转 {'成功' if result else '失败'}")
        return result
    
    def wait(self, seconds: float):
        time.sleep(seconds)
    
    def scan_servos(self) -> dict:
        result = {}
        print("\n[SCAN] 扫描舵机...")
        for i in range(1, 7):
            print(f"[SCAN] 检测舵机 ID={i}...", end=" ")
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
                    print(f"✓ 在线, 位置={pos}, 角度={np.degrees(angle):.1f}°, 电压={voltage}V, 温度={temp}°C")
                else:
                    print(f"✓ 在线, 位置读取失败")
            else:
                result[i] = {'online': False}
                print(f"✗ 无响应")
        print(f"[SCAN] 扫描完成, {sum(1 for r in result.values() if r.get('online'))}/6 舵机在线")
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
        """运行完整测试 - 测试所有运动控制命令"""
        if not self.arm.connected:
            print("[ERROR] 机械臂未连接")
            return
        
        print("\n" + "="*60)
        print("开始完整测试 - 测试所有运动控制命令")
        print("="*60)
        
        try:
            print("\n" + "-"*60)
            print("[TEST 1/8] 移动到中立位置")
            print("-"*60)
            self.arm.move_to_neutral(duration=1.5)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[TEST 2/8] 肩部旋转测试 (向右 +28.6°)")
            print("-"*60)
            self.arm.rotate_shoulder(0.5, duration=1.0)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[TEST 3/8] 肩部旋转测试 (向左 -28.6°)")
            print("-"*60)
            self.arm.rotate_shoulder(-0.5, duration=1.0)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[TEST 4/8] 肩部旋转测试 (回到中心 0°)")
            print("-"*60)
            self.arm.rotate_shoulder(0.0, duration=1.0)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[TEST 5/8] 夹爪闭合测试")
            print("-"*60)
            self.arm.grasp(duration=0.8)
            time.sleep(1)
            
            print("\n" + "-"*60)
            print("[TEST 6/8] 夹爪打开测试")
            print("-"*60)
            self.arm.release(duration=0.8)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[TEST 7/8] 移动到初始位置 (Home)")
            print("-"*60)
            self.arm.move_to_home(duration=1.5)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[TEST 8/8] 返回中立位置")
            print("-"*60)
            self.arm.move_to_neutral(duration=1.5)
            
            print("\n" + "="*60)
            print("✓ 所有测试完成!")
            print("="*60)
            
        except KeyboardInterrupt:
            print("\n[INTERRUPT] 用户中断测试")
        except Exception as e:
            print(f"\n[ERROR] 测试出错: {e}")
            import traceback
            traceback.print_exc()
    
    def run_cartesian_test(self):
        """测试笛卡尔空间控制"""
        if not self.arm.connected:
            print("[ERROR] 机械臂未连接")
            return
        
        print("\n" + "="*60)
        print("笛卡尔空间控制测试")
        print("="*60)
        
        try:
            print("\n" + "-"*60)
            print("[CARTESIAN 1/7] 移动到中立位置")
            print("-"*60)
            self.arm.move_to_neutral(duration=1.5)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[CARTESIAN 2/7] 获取当前末端位置")
            print("-"*60)
            pos = self.arm.get_current_xyz()
            print(f"当前末端位置: x={pos[0]*1000:.1f}mm, y={pos[1]*1000:.1f}mm, z={pos[2]*1000:.1f}mm")
            time.sleep(0.3)
            
            print("\n" + "-"*60)
            print("[CARTESIAN 3/7] 向上移动 5cm")
            print("-"*60)
            self.arm.move_up(0.05, duration=1.5)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[CARTESIAN 4/7] 向下移动 5cm (回到原高度)")
            print("-"*60)
            self.arm.move_down(0.05, duration=1.5)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[CARTESIAN 5/7] 向左移动 5cm")
            print("-"*60)
            self.arm.move_left(0.05, duration=1.5)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[CARTESIAN 6/7] 向右移动 5cm (回到原位置)")
            print("-"*60)
            self.arm.move_right(0.05, duration=1.5)
            time.sleep(0.5)
            
            print("\n" + "-"*60)
            print("[CARTESIAN 7/7] 返回中立位置")
            print("-"*60)
            self.arm.move_to_neutral(duration=1.5)
            
            print("\n" + "="*60)
            print("✓ 笛卡尔空间控制测试完成!")
            print("="*60)
            
        except KeyboardInterrupt:
            print("\n[INTERRUPT] 用户中断测试")
        except Exception as e:
            print(f"\n[ERROR] 测试出错: {e}")
            import traceback
            traceback.print_exc()
    
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
    parser.add_argument('--cartesian', action='store_true', help='运行笛卡尔空间控制测试')
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
        elif args.cartesian:
            runner.run_cartesian_test()
        elif args.test:
            runner.run_simple_test()
        else:
            print("\n使用方法:")
            print("  python soarm101_sdk.py COM18 --test        # 关节空间测试")
            print("  python soarm101_sdk.py COM18 --cartesian   # 笛卡尔空间测试")
            print("  python soarm101_sdk.py COM18 --task        # 完整抓取任务")
            print("  python soarm101_sdk.py COM18 --task --rotate 90 --wait 5")
        
        runner.disconnect()


if __name__ == "__main__":
    main()
