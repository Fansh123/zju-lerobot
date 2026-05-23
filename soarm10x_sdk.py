"""
SO-ARM100/SO-ARM101 独立控制 SDK
无需 ROS2，直接通过串口控制 STS3215 舵机

依赖:
    pip install pyserial numpy

支持的机械臂型号:
    - SO-100 (旧版本)
    - SO-101 (新版本/专业版)

使用示例:
    from soarm10x_sdk import SOARM10X
    
    arm = SOARM10X('COM3', model='so101')
    if arm.connect():
        arm.move_to_neutral()
        arm.grasp()
        arm.move_to_position([0.5, 1.0, -1.0, 0.5, 0.0, 0.0])
        arm.release()
        arm.disconnect()
"""

import serial
import time
import numpy as np


class SOARM10X:
    """SO-ARM100/SO-ARM101 机械臂控制器"""
    
    # SO-100 关节限制 (弧度)
    SO100_LIMITS = np.array([
        [-2.0, 2.0],       # shoulder_pan
        [-1.5, 1.5],       # shoulder_lift
        [-2.0, 0.5],       # elbow_flex
        [-1.5, 1.5],       # wrist_flex
        [-2.0, 2.0],       # wrist_roll
        [0.0, 2.0]         # gripper
    ])
    
    # SO-101 关节限制 (弧度) - 新标定版本
    SO101_LIMITS = np.array([
        [-1.91986, 1.91986],   # shoulder_pan (~-110° ~ +110°)
        [-1.74533, 1.74533],   # shoulder_lift (~-100° ~ +100°)
        [-1.69, 1.69],         # elbow_flex (~-97° ~ +97°)
        [-1.65806, 1.65806],   # wrist_flex (~-95° ~ +95°)
        [-2.74385, 2.84121],   # wrist_roll (~-157° ~ +163°)
        [-0.17453, 1.74533]    # gripper (~-10° ~ +100°)
    ])
    
    # 关节名称
    JOINT_NAMES = [
        'shoulder_pan',    # 底座旋转
        'shoulder_lift',   # 肩部抬升
        'elbow_flex',      # 肘部弯曲
        'wrist_flex',      # 腕部俯仰
        'wrist_roll',      # 腕部旋转
        'gripper'          # 夹爪
    ]
    
    def __init__(self, port='COM3', baud_rate=115200, model='so101'):
        """
        初始化控制器
        
        参数:
            port: 串口端口 (默认 COM3)
            baud_rate: 波特率 (默认 115200)
            model: 机械臂型号 'so100' 或 'so101' (默认 'so101')
        """
        self.port = port
        self.baud_rate = baud_rate
        self.model = model.lower()
        self.ser = None
        self.connected = False
        
        # 根据型号设置关节限制
        if self.model == 'so101':
            self.joint_limits = self.SO101_LIMITS
            print(f"使用 SO-101 关节限制")
        else:
            self.joint_limits = self.SO100_LIMITS
            print(f"使用 SO-100 关节限制")
        
        # 当前关节角度
        self.current_joints = np.zeros(6)
    
    def _angle_to_pulse(self, angle_rad, joint_idx):
        """将弧度转换为舵机脉冲宽度 (500-2500 us)"""
        min_angle, max_angle = self.joint_limits[joint_idx]
        angle_rad = np.clip(angle_rad, min_angle, max_angle)
        
        if joint_idx == 5:  # 夹爪特殊处理
            # 夹爪范围映射到 500-2500
            range_total = max_angle - min_angle
            normalized = (angle_rad - min_angle) / range_total
            return int(500 + normalized * 2000)
        else:
            # 标准关节: -90° ~ +90° 映射到 500 ~ 2500
            angle_deg = np.degrees(angle_rad)
            angle_deg = np.clip(angle_deg, -90, 90)
            return int(500 + (angle_deg + 90) * 2000 / 180)
    
    def _pulse_to_angle(self, pulse):
        """将脉冲宽度转换为弧度"""
        angle_deg = ((pulse - 500) / 2000) * 180 - 90
        return np.radians(angle_deg)
    
    def _send_command(self, servo_id, pulse):
        """发送单舵机控制指令"""
        if self.ser is None or not self.ser.is_open:
            return False
        
        low_byte = pulse & 0xFF
        high_byte = (pulse >> 8) & 0xFF
        checksum = (servo_id + 3 + high_byte + low_byte) & 0xFF
        
        command = bytearray([0xFF, 0xFF, servo_id, 3, high_byte, low_byte, checksum])
        
        try:
            self.ser.write(command)
            time.sleep(0.02)
            return True
        except Exception as e:
            print(f"发送指令失败: {e}")
            return False
    
    def _read_angle(self, servo_id):
        """读取单个舵机角度"""
        if self.ser is None or not self.ser.is_open:
            return None
        
        command = bytearray([0xFF, 0xFF, servo_id, 2, 0x02, 0x00])
        
        try:
            self.ser.write(command)
            time.sleep(0.05)
            response = self.ser.read(7)
            
            if len(response) == 7 and response[0] == 0xFF and response[1] == 0xFF:
                pulse = (response[4] << 8) | response[5]
                return self._pulse_to_angle(pulse)
            return None
        except Exception as e:
            print(f"读取角度失败: {e}")
            return None
    
    def connect(self):
        """连接机械臂"""
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            time.sleep(0.2)
            
            if self.ser.is_open:
                angles = self.get_joint_angles()
                if angles is not None:
                    self.current_joints = angles
                    self.connected = True
                    print(f"成功连接到 SO-ARM{self.model[-3:]}!")
                    print(f"当前关节角度 (度): {np.degrees(self.current_joints).round(1)}")
                    return True
                else:
                    self.ser.close()
                    print("无法读取关节角度")
                    return False
            else:
                print("串口无法打开")
                return False
                
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.connected = False
            print("已断开连接")
    
    def get_joint_angles(self):
        """获取所有关节角度"""
        if not self.connected:
            print("错误: 未连接")
            return None
        
        angles = []
        for i in range(6):
            angle = self._read_angle(i + 1)
            if angle is None:
                return None
            angles.append(angle)
        
        return np.array(angles)
    
    def set_joint_angle(self, joint_idx, angle_rad, wait=True):
        """设置单个关节角度"""
        if not self.connected:
            print("错误: 未连接")
            return False
        
        if joint_idx < 0 or joint_idx >= 6:
            print(f"错误: 关节索引 {joint_idx} 无效")
            return False
        
        pulse = self._angle_to_pulse(angle_rad, joint_idx)
        success = self._send_command(joint_idx + 1, pulse)
        
        if success:
            self.current_joints[joint_idx] = angle_rad
            if wait:
                time.sleep(0.1)
        
        return success
    
    def set_joint_angles(self, angles, duration=1.0, smooth=True):
        """设置所有关节角度（平滑运动）"""
        if not self.connected:
            print("错误: 未连接")
            return False
        
        angles = np.array(angles)
        if len(angles) != 6:
            print("错误: 必须提供6个关节角度")
            return False
        
        # 限制角度范围
        for i in range(6):
            angles[i] = np.clip(angles[i], self.joint_limits[i][0], self.joint_limits[i][1])
        
        num_steps = max(10, int(duration * 20))
        start_angles = self.current_joints.copy()
        
        for i in range(num_steps):
            t = i / num_steps
            
            if smooth:
                t = t * t * t * (t * (t * 6 - 15) + 10)
            
            current = start_angles + (angles - start_angles) * t
            
            for j in range(6):
                self._send_command(j + 1, self._angle_to_pulse(current[j], j))
            
            self.current_joints = current.copy()
            time.sleep(duration / num_steps)
        
        return True
    
    def move_to_position(self, angles, duration=1.0):
        """移动到指定位置"""
        return self.set_joint_angles(angles, duration)
    
    def move_to_neutral(self, duration=1.0):
        """移动到中立位置"""
        print("移动到中立位置")
        return self.set_joint_angles(np.zeros(6), duration)
    
    def grasp(self):
        """闭合夹爪"""
        print("夹爪闭合")
        min_grip, max_grip = self.joint_limits[5]
        return self.set_joint_angle(5, min_grip)
    
    def release(self):
        """打开夹爪"""
        print("夹爪打开")
        min_grip, max_grip = self.joint_limits[5]
        return self.set_joint_angle(5, max_grip)
    
    def set_gripper_position(self, position):
        """设置夹爪位置 (0=闭合, 1=打开)"""
        min_grip, max_grip = self.joint_limits[5]
        position = np.clip(position, 0.0, 1.0)
        angle = min_grip + position * (max_grip - min_grip)
        return self.set_joint_angle(5, angle)
    
    def get_joint_info(self):
        """打印关节信息"""
        print("\n" + "="*60)
        print(f"SO-ARM{self.model[-3:]} 关节信息")
        print("="*60)
        print(f"{'索引':<5} {'关节名称':<15} {'最小角度':<12} {'最大角度':<12}")
        print("-"*60)
        for i, name in enumerate(self.JOINT_NAMES):
            min_deg = np.degrees(self.joint_limits[i][0])
            max_deg = np.degrees(self.joint_limits[i][1])
            print(f"{i:<5} {name:<15} {min_deg:>8.1f}° ~ {max_deg:>8.1f}°")
        print("="*60)
    
    def test(self):
        """运行测试序列"""
        if not self.connected:
            print("错误: 未连接")
            return
        
        print("\n" + "="*50)
        print(f"SO-ARM{self.model[-3:]} 功能测试")
        print("="*50)
        
        print("\n1. 移动到中立位置")
        self.move_to_neutral()
        
        print("\n2. 测试肩部旋转")
        self.set_joint_angles([0.785, 0.0, 0.0, 0.0, 0.0, 0.5], duration=1.0)
        time.sleep(1)
        self.set_joint_angles([-0.785, 0.0, 0.0, 0.0, 0.0, 0.5], duration=1.0)
        time.sleep(1)
        
        print("\n3. 测试肩部抬升")
        self.set_joint_angles([0.0, 1.0, 0.0, 0.0, 0.0, 0.5], duration=1.0)
        time.sleep(1)
        self.set_joint_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.5], duration=1.0)
        
        print("\n4. 测试肘部弯曲")
        self.set_joint_angles([0.0, 0.5, -1.0, 0.0, 0.0, 0.5], duration=1.0)
        time.sleep(1)
        self.set_joint_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.5], duration=1.0)
        
        print("\n5. 测试夹爪")
        self.grasp()
        time.sleep(1)
        self.release()
        
        print("\n6. 返回中立位置")
        self.move_to_neutral()
        
        print("\n" + "="*50)
        print("测试完成!")
        print("="*50)


def main():
    import sys
    
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM3'
    model = sys.argv[2] if len(sys.argv) > 2 else 'so101'
    
    print(f"SO-ARM10X 独立控制器")
    print(f"端口: {port}")
    print(f"型号: {model}")
    print("-"*40)
    
    arm = SOARM10X(port, model=model)
    
    if arm.connect():
        arm.get_joint_info()
        
        try:
            arm.test()
        except KeyboardInterrupt:
            print("\n用户中断，返回中立位置")
            arm.move_to_neutral()
        finally:
            arm.disconnect()
    else:
        print("无法连接到机械臂")


if __name__ == "__main__":
    main()