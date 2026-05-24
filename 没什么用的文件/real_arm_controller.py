import serial
import time
import numpy as np


class STS3215Servo:
    def __init__(self, serial_port, baud_rate=115200):
        self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
        time.sleep(0.1)
        self.joint_limits = [
            (-2.0, 2.0),   
            (-2.0, 2.0),   
            (-2.0, 2.0),   
            (-2.0, 2.0),   
            (-2.0, 2.0),   
            (0.0, 2.0)     
        ]

    def _angle_to_pulse(self, angle_rad, joint_idx):
        min_angle, max_angle = self.joint_limits[joint_idx]
        angle_rad = np.clip(angle_rad, min_angle, max_angle)
        pulse_min = 500
        pulse_max = 2500
        angle_deg = np.degrees(angle_rad)
        angle_deg = np.clip(angle_deg, -90, 90)
        pulse = pulse_min + (angle_deg + 90) * (pulse_max - pulse_min) / 180
        return int(pulse)

    def _pulse_to_angle(self, pulse):
        angle_deg = ((pulse - 500) / (2500 - 500)) * 180 - 90
        return np.radians(angle_deg)

    def set_joint_angle(self, joint_idx, angle_rad, speed=100):
        if joint_idx < 0 or joint_idx > 5:
            print(f"错误: 关节索引 {joint_idx} 无效")
            return False

        pulse = self._angle_to_pulse(angle_rad, joint_idx)
        servo_id = joint_idx + 1
        low_byte = pulse & 0xFF
        high_byte = (pulse >> 8) & 0xFF
        
        checksum = (servo_id + 3 + high_byte + low_byte) & 0xFF
        
        command = bytearray([0xFF, 0xFF, servo_id, 3, high_byte, low_byte, checksum])
        self.ser.write(command)
        
        time.sleep(0.05)
        return True

    def set_joint_angles(self, angles, speed=100, duration=0.5):
        num_steps = 20
        start_angles = self.get_joint_angles()
        if start_angles is None:
            start_angles = np.zeros(6)
        
        for i in range(num_steps):
            t = i / num_steps
            t_smooth = t * t * t * (t * (t * 6 - 15) + 10)
            current_angles = start_angles + (angles - start_angles) * t_smooth
            
            for j in range(6):
                self.set_joint_angle(j, current_angles[j])
            
            time.sleep(duration / num_steps)
        
        return True

    def get_joint_angle(self, joint_idx):
        servo_id = joint_idx + 1
        command = bytearray([0xFF, 0xFF, servo_id, 2, 0x02, 0x00])
        self.ser.write(command)
        
        time.sleep(0.05)
        response = self.ser.read(7)
        
        if len(response) == 7 and response[0] == 0xFF and response[1] == 0xFF:
            pulse = (response[4] << 8) | response[5]
            return self._pulse_to_angle(pulse)
        
        return None

    def get_joint_angles(self):
        angles = []
        for i in range(6):
            angle = self.get_joint_angle(i)
            if angle is None:
                return None
            angles.append(angle)
        return np.array(angles)

    def set_gripper(self, position):
        position = np.clip(position, 0.0, 2.0)
        return self.set_joint_angle(5, position)

    def open_gripper(self):
        return self.set_joint_angle(5, 2.0)

    def close_gripper(self):
        return self.set_joint_angle(5, 0.0)

    def move_to_neutral(self):
        neutral_angles = np.zeros(6)
        return self.set_joint_angles(neutral_angles, duration=1.0)

    def test_connection(self):
        try:
            angles = self.get_joint_angles()
            if angles is not None:
                print("成功连接到机械臂!")
                print(f"当前关节角度: {angles}")
                return True
            else:
                print("无法读取关节角度")
                return False
        except Exception as e:
            print(f"连接测试失败: {e}")
            return False

    def close(self):
        if self.ser.is_open:
            self.ser.close()


class RealArmController:
    def __init__(self, port='COM3', baud_rate=115200):
        self.servo = STS3215Servo(port, baud_rate)
        self.current_q = np.zeros(6)
        self.connected = False

    def initialize(self):
        print(f"尝试连接机械臂: {self.servo.ser.port}")
        self.connected = self.servo.test_connection()
        if self.connected:
            self.current_q = self.servo.get_joint_angles() or np.zeros(6)
            print(f"当前关节角度: {self.current_q}")
        return self.connected

    def move_to_q(self, target_q, duration=1.0):
        if not self.connected:
            print("错误: 机械臂未连接")
            return False
        
        print(f"移动到: {target_q}")
        success = self.servo.set_joint_angles(target_q, duration=duration)
        if success:
            self.current_q = target_q.copy()
        return success

    def grasp(self):
        if not self.connected:
            print("错误: 机械臂未连接")
            return False
        print("夹爪闭合")
        return self.servo.close_gripper()

    def release(self):
        if not self.connected:
            print("错误: 机械臂未连接")
            return False
        print("夹爪打开")
        return self.servo.open_gripper()

    def move_to_neutral(self):
        return self.move_to_q(np.zeros(6), duration=1.0)

    def close(self):
        self.servo.close()
        self.connected = False


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = 'COM3'
    
    arm = RealArmController(port=port)
    
    if arm.initialize():
        print("\n机械臂控制测试")
        print("="*50)
        print("1. 移动到初始位置")
        arm.move_to_neutral()
        
        print("\n2. 测试肩部旋转")
        test_q = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 2.0])
        arm.move_to_q(test_q, duration=1.0)
        
        print("\n3. 返回初始位置")
        arm.move_to_neutral()
        
        print("\n4. 测试夹爪")
        arm.close_gripper()
        time.sleep(1)
        arm.open_gripper()
        
        arm.close()
        print("\n测试完成!")
    else:
        print("无法连接到机械臂，请检查串口设置")