#!/usr/bin/env python3
"""
舵机极限位置完整测试
先让所有舵机回到安全位置，然后逐个测试每个舵机的极限
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


def write_position(ser, servo_id, position, speed=150):
    position = max(0, min(4095, int(position)))
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


def move_all_to_position(ser, positions, speed=150):
    """移动所有舵机到指定位置"""
    for i, pos in enumerate(positions):
        write_position(ser, i + 1, pos, speed)
        time.sleep(0.02)
    time.sleep(2)


def find_limit(ser, servo_id, direction, start_pos, step=50, max_steps=40):
    """寻找舵机的极限位置"""
    pos = start_pos
    last_valid_pos = start_pos
    
    for _ in range(max_steps):
        test_pos = pos + (step * direction)
        if test_pos < 0 or test_pos > 4095:
            break
        
        write_position(ser, servo_id, test_pos, speed=80)
        time.sleep(0.3)
        actual = read_position(ser, servo_id)
        
        if actual is None:
            break
        
        diff = abs(actual - test_pos)
        if diff > 100:
            break
        
        last_valid_pos = actual
        pos = test_pos
    
    return last_valid_pos


def test_servo_limits(ser, servo_id, servo_name, safe_positions):
    """测试单个舵机的极限位置"""
    print(f"\n测试舵机 ID={servo_id} ({servo_name})")
    print("-" * 50)
    
    print("  移动到安全位置...")
    move_all_to_position(ser, safe_positions, speed=150)
    
    current_pos = read_position(ser, servo_id)
    print(f"  当前位置: {current_pos}")
    
    print("  寻找最小位置...")
    min_pos = find_limit(ser, servo_id, -1, current_pos, step=30, max_steps=60)
    print(f"  最小位置: {min_pos}")
    
    write_position(ser, servo_id, 2048, speed=100)
    time.sleep(1)
    
    print("  寻找最大位置...")
    max_pos = find_limit(ser, servo_id, 1, 2048, step=30, max_steps=60)
    print(f"  最大位置: {max_pos}")
    
    print("  返回中心位置...")
    write_position(ser, servo_id, 2048, speed=150)
    time.sleep(1)
    
    return min_pos, max_pos


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("舵机极限位置完整测试")
    print("="*60)
    print("\n⚠️ 此测试会让每个舵机单独移动到极限位置")
    print("   请确保机械臂周围有足够空间！\n")
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.5)
        time.sleep(0.5)
        print(f"✓ 连接到 {port}")
        
        for i in range(1, 7):
            enable_torque(ser, i, True)
            set_speed_accel(ser, i, speed=200, accel=30)
            time.sleep(0.02)
        
        print("\n移动所有舵机到中立位置...")
        move_all_to_position(ser, [2048]*6, speed=150)
        
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
            safe_positions = [2048] * 6
            min_pos, max_pos = test_servo_limits(ser, servo_id, name, safe_positions)
            results[servo_id] = {
                'name': name,
                'min': min_pos,
                'max': max_pos,
                'range': (max_pos - min_pos) if min_pos and max_pos else 0
            }
        
        print("\n" + "="*60)
        print("测试结果汇总")
        print("="*60)
        print(f"{'舵机':<6} {'名称':<25} {'最小':<8} {'最大':<8} {'范围':<8} {'角度范围'}")
        print("-"*80)
        for servo_id, data in results.items():
            if data['min'] and data['max']:
                min_deg = (data['min'] - 2048) / 4096 * 360
                max_deg = (data['max'] - 2048) / 4096 * 360
                range_deg = data['range'] / 4096 * 360
                print(f"ID={servo_id:<4} {data['name']:<25} {data['min']:<8} {data['max']:<8} {data['range']:<8} {min_deg:.1f}°~{max_deg:.1f}° ({range_deg:.1f}°)")
        
        print("\n建议的关节限制（弧度）:")
        print("JOINT_LIMITS = np.array([")
        for i in range(6):
            servo_id = i + 1
            if servo_id in results:
                data = results[servo_id]
                if data['min'] and data['max']:
                    min_angle = (data['min'] - 2048) / 4096 * 2 * np.pi
                    max_angle = (data['max'] - 2048) / 4096 * 2 * np.pi
                    print(f"    [{min_angle:.2f}, {max_angle:.2f}],  # {data['name']}")
        print("])")
        
        print("\n返回中立位置...")
        move_all_to_position(ser, [2048]*6, speed=150)
        
        ser.close()
        print("\n✓ 测试完成!")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
