"""
SO-ARM101 Waveshare 控制器 (修复版 v3)
- 修复指令码: READ=0x02, WRITE=0x03
- 添加速度和加速度设置
- 确保舵机能正常运动
"""

import serial
import time
import numpy as np


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
            print(f"✓ 连接到 {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
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
    
    def angle_to_position(self, angle_rad):
        return int(angle_rad / (2 * np.pi) * 4096 + self.POS_CENTER)
    
    def position_to_angle(self, position):
        return (position - self.POS_CENTER) / 4096 * 2 * np.pi


class SOARM101:
    """SO-ARM101 机械臂控制器"""
    
    JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 
                   'wrist_flex', 'wrist_roll', 'gripper']
    
    JOINT_LIMITS = np.array([
        [-1.92, 1.92], [-1.75, 1.75], [-1.69, 1.69],
        [-1.66, 1.66], [-2.74, 2.84], [0.0, 1.75]
    ])
    
    DEFAULT_SPEED = 500
    DEFAULT_ACCELERATION = 50
    
    def __init__(self, port='COM18'):
        self.bus = FeetechSTS(port, baudrate=1000000)
        self.connected = False
        self.current_positions = np.full(6, FeetechSTS.POS_CENTER)
    
    def connect(self):
        if not self.bus.connect():
            return False
        
        for i in range(6):
            self.bus.enable_torque(i + 1, True)
            self.bus.set_acceleration(i + 1, self.DEFAULT_ACCELERATION)
            self.bus.set_speed(i + 1, self.DEFAULT_SPEED)
            time.sleep(0.03)
        
        self.connected = True
        print("✓ 机械臂已连接，扭矩已启用，速度/加速度已设置")
        return True
    
    def disconnect(self):
        for i in range(6):
            self.bus.enable_torque(i + 1, False)
            time.sleep(0.02)
        self.bus.disconnect()
        self.connected = False
    
    def get_joint_angles(self):
        if not self.connected:
            return None
        angles = np.zeros(6)
        for i in range(6):
            pos = self.bus.read_position(i + 1)
            if pos is not None:
                self.current_positions[i] = pos
                angles[i] = self.bus.position_to_angle(pos)
        return angles
    
    def set_joint_angles(self, angles, duration=1.0):
        if not self.connected:
            return False
        
        angles = np.array(angles, dtype=float)
        for i in range(6):
            angles[i] = np.clip(angles[i], self.JOINT_LIMITS[i][0], self.JOINT_LIMITS[i][1])
        
        target_positions = np.array([self.bus.angle_to_position(a) for a in angles])
        start_positions = self.current_positions.copy()
        
        num_steps = max(10, int(duration * 20))
        
        for step in range(num_steps + 1):
            t = step / num_steps
            t_smooth = t * t * t * (t * (t * 6 - 15) + 10)
            
            current_pos = start_positions + (target_positions - start_positions) * t_smooth
            
            for j in range(6):
                self.bus.set_position(j + 1, int(current_pos[j]))
            
            self.current_positions = current_pos.astype(int)
            time.sleep(duration / num_steps)
        
        return True
    
    def move_to_neutral(self, duration=1.0):
        print("移动到中立位置...")
        return self.set_joint_angles(np.zeros(6), duration)
    
    def grasp(self):
        print("夹爪闭合")
        angles = np.zeros(6)
        angles[5] = 0.0
        return self.set_joint_angles(angles, duration=0.5)
    
    def release(self):
        print("夹爪打开")
        angles = np.zeros(6)
        angles[5] = 1.5
        return self.set_joint_angles(angles, duration=0.5)
    
    def scan_servos(self):
        print("\n扫描舵机...")
        for i in range(1, 7):
            if self.bus.ping(i):
                pos = self.bus.read_position(i)
                if pos is not None:
                    angle = self.bus.position_to_angle(pos)
                    print(f"  ✓ 舵机 ID={i}: 位置={pos}, 角度={np.degrees(angle):.1f}°")
                else:
                    print(f"  ✓ 舵机 ID={i}: 在线 (位置读取失败)")
            else:
                print(f"  ✗ 舵机 ID={i}: 无响应")


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("SO-ARM101 Waveshare 控制器 v3")
    print("="*60)
    
    arm = SOARM101(port)
    
    if arm.connect():
        arm.scan_servos()
        
        try:
            print("\n开始测试运动...")
            
            print("\n1. 移动到中立位置")
            arm.move_to_neutral(duration=1.5)
            time.sleep(0.5)
            
            print("\n2. 肩部旋转测试 (向右)")
            angles = np.zeros(6)
            angles[0] = 0.5
            arm.set_joint_angles(angles, duration=1.0)
            time.sleep(0.5)
            
            print("\n3. 肩部旋转测试 (向左)")
            angles[0] = -0.5
            arm.set_joint_angles(angles, duration=1.0)
            time.sleep(0.5)
            
            print("\n4. 返回中立位置")
            arm.move_to_neutral(duration=1.0)
            time.sleep(0.5)
            
            print("\n5. 夹爪测试")
            arm.grasp()
            time.sleep(1)
            arm.release()
            time.sleep(0.5)
            
            print("\n6. 最终返回中立位置")
            arm.move_to_neutral()
            
            print("\n✓ 测试完成!")
            
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            arm.disconnect()
    else:
        print("无法连接到机械臂")


if __name__ == "__main__":
    main()
