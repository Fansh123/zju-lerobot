#!/usr/bin/env python3
"""
舵机极限位置测试
逐个测试每个舵机的最小和最大位置
"""

import serial
import time
import numpy as np


def checksum(data):
    return (~sum(data)) & 0xFF


def build_packet(servo_id, instruction, parameters=None):
    if parameters is None:
        parameters = []
    length = len(parameters) + 2
    data = [servo_id, length, instruction] + parameters
    chk = checksum(data)
    return bytes([0xFF, 0xFF] + data + [chk])


def read_position(ser, servo_id):
    packet = build_packet(servo_id, 0x02, [56, 2])
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.1)
    resp = ser.read(50)
    if resp and len(resp) >= 8:
        return resp[5] + (resp[6] << 8)
    return None


def write_position(ser, servo_id, position, speed=100):
    pos_low = position & 0xFF
    pos_high = (position >> 8) & 0xFF
    spd_low = speed & 0xFF
    spd_high = (speed >> 8) & 0xFF
    packet = build_packet(servo_id, 0x03, [42, pos_low, pos_high, spd_low, spd_high])
    ser.reset_input_buffer()
    ser.write(packet)


def enable_torque(ser, servo_id, enable=True):
    packet = build_packet(servo_id, 0x03, [40, 1 if enable else 0])
    ser.reset_input_buffer()
    ser.write(packet)


def set_speed_accel(ser, servo_id, speed=200, accel=30):
    packet = build_packet(servo_id, 0x03, [44, accel])
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.02)
    spd_low = speed & 0xFF
    spd_high = (speed >> 8) & 0xFF
    packet = build_packet(servo_id, 0x03, [46, spd_low, spd_high])
    ser.reset_input_buffer()
    ser.write(packet)


def test_servo_limits(ser, servo_id, servo_name):
    """测试单个舵机的极限位置"""
    print(f"\n{'='*50}")
    print(f"测试舵机 ID={servo_id} ({servo_name})")
    print('='*50)
    
    enable_torque(ser, servo_id, True)
    set_speed_accel(ser, servo_id, speed=200, accel=30)
    time.sleep(0.1)
    
    current_pos = read_position(ser, servo_id)
    print(f"当前位置: {current_pos}")
    
    input(f"\n按Enter开始测试舵机{servo_id}的最小位置...")
    print("正在寻找最小位置...")
    print("⚠️ 如果舵机发出异响或到达机械极限，请立即按Ctrl+C停止！")
    
    min_pos = current_pos
    test_positions = [current_pos - 100, current_pos - 200, current_pos - 300,
                      current_pos - 500, current_pos - 800, current_pos - 1000,
                      current_pos - 1500, current_pos - 2000]
    
    for target in test_positions:
        if target < 0:
            target = 0
        print(f"  尝试位置: {target}")
        write_position(ser, servo_id, target, speed=100)
        time.sleep(1.5)
        actual_pos = read_position(ser, servo_id)
        print(f"  实际位置: {actual_pos}")
        if actual_pos:
            min_pos = actual_pos
        time.sleep(0.5)
    
    print(f"\n最小位置: {min_pos}")
    
    input(f"\n按Enter开始测试舵机{servo_id}的最大位置...")
    print("正在寻找最大位置...")
    print("⚠️ 如果舵机发出异响或到达机械极限，请立即按Ctrl+C停止！")
    
    max_pos = current_pos
    test_positions = [current_pos + 100, current_pos + 200, current_pos + 300,
                      current_pos + 500, current_pos + 800, current_pos + 1000,
                      current_pos + 1500, current_pos + 2000]
    
    for target in test_positions:
        if target > 4095:
            target = 4095
        print(f"  尝试位置: {target}")
        write_position(ser, servo_id, target, speed=100)
        time.sleep(1.5)
        actual_pos = read_position(ser, servo_id)
        print(f"  实际位置: {actual_pos}")
        if actual_pos:
            max_pos = actual_pos
        time.sleep(0.5)
    
    print(f"\n最大位置: {max_pos}")
    
    center = (min_pos + max_pos) // 2
    print(f"\n返回中心位置: {center}")
    write_position(ser, servo_id, center, speed=200)
    time.sleep(1)
    
    return min_pos, max_pos


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("舵机极限位置测试")
    print("="*60)
    print("\n⚠️ 警告: 此测试会让舵机移动到极限位置")
    print("   请确保机械臂周围有足够空间，无障碍物")
    print("   如有异常请立即按 Ctrl+C 停止\n")
    
    input("准备好后按Enter开始...")
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.5)
        time.sleep(0.5)
        print(f"✓ 连接到 {port}\n")
        
        servo_names = [
            'shoulder_pan (底座旋转)',
            'shoulder_lift (肩部抬升)',
            'elbow_flex (肘部弯曲)',
            'wrist_flex (腕部俯仰)',
            'wrist_roll (腕部旋转)',
            'gripper (夹爪)'
        ]
        
        results = {}
        
        for i, name in enumerate(servo_names):
            servo_id = i + 1
            try:
                min_pos, max_pos = test_servo_limits(ser, servo_id, name)
                results[servo_id] = {
                    'name': name,
                    'min': min_pos,
                    'max': max_pos,
                    'range': max_pos - min_pos
                }
            except KeyboardInterrupt:
                print(f"\n用户中断，跳过舵机{servo_id}")
                continue
        
        ser.close()
        
        print("\n" + "="*60)
        print("测试结果汇总")
        print("="*60)
        print(f"{'舵机':<6} {'名称':<25} {'最小':<8} {'最大':<8} {'范围':<8}")
        print("-"*60)
        for servo_id, data in results.items():
            print(f"ID={servo_id:<4} {data['name']:<25} {data['min']:<8} {data['max']:<8} {data['range']:<8}")
        
        print("\n建议的关节限制（弧度）:")
        print("JOINT_LIMITS = np.array([")
        for i in range(6):
            servo_id = i + 1
            if servo_id in results:
                data = results[servo_id]
                min_angle = (data['min'] - 2048) / 4096 * 2 * np.pi
                max_angle = (data['max'] - 2048) / 4096 * 2 * np.pi
                print(f"    [{min_angle:.2f}, {max_angle:.2f}],  # {data['name']}")
        print("])")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
