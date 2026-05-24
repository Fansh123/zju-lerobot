#!/usr/bin/env python3
"""
舵机极限位置简单测试
直接测试每个舵机能否到达指定位置
"""

import serial
import time


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


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("舵机极限位置简单测试")
    print("="*60)
    
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
        
        # 初始化所有舵机
        print("初始化所有舵机...")
        for i in range(1, 7):
            enable_torque(ser, i, True)
            set_speed_accel(ser, i, speed=200, accel=30)
        time.sleep(0.5)
        
        # 先让所有舵机回到中心位置
        print("移动到中心位置...")
        for i in range(1, 7):
            write_position(ser, i, 2048, speed=150)
        time.sleep(2)
        
        results = {}
        
        # 测试每个舵机
        for i, name in enumerate(servo_names):
            servo_id = i + 1
            print(f"\n{'='*50}")
            print(f"测试舵机 ID={servo_id} ({name})")
            print('='*50)
            
            # 读取当前位置
            current = read_position(ser, servo_id)
            print(f"当前位置: {current}")
            
            # 测试几个关键位置
            test_positions = [500, 1000, 1500, 2048, 2500, 3000, 3500]
            valid_positions = []
            
            for pos in test_positions:
                print(f"  测试位置 {pos}...", end=" ")
                write_position(ser, servo_id, pos, speed=100)
                time.sleep(1)
                actual = read_position(ser, servo_id)
                
                if actual:
                    diff = abs(actual - pos)
                    if diff < 50:
                        print(f"✓ 到达 {actual}")
                        valid_positions.append(pos)
                    else:
                        print(f"✗ 实际 {actual} (差 {diff})")
                else:
                    print("✗ 无响应")
            
            if valid_positions:
                results[servo_id] = {
                    'name': name,
                    'min': min(valid_positions),
                    'max': max(valid_positions),
                    'valid': valid_positions
                }
            else:
                results[servo_id] = {
                    'name': name,
                    'min': None,
                    'max': None,
                    'valid': []
                }
            
            # 返回中心
            write_position(ser, servo_id, 2048, speed=150)
            time.sleep(1)
        
        # 打印结果汇总
        print("\n" + "="*60)
        print("测试结果汇总")
        print("="*60)
        print(f"{'舵机':<6} {'名称':<25} {'最小':<8} {'最大':<8} {'有效位置'}")
        print("-"*80)
        for servo_id, data in results.items():
            valid_str = str(data['valid']) if data['valid'] else "无"
            print(f"ID={servo_id:<4} {data['name']:<25} {data['min']:<8} {data['max']:<8} {valid_str}")
        
        ser.close()
        print("\n✓ 测试完成!")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
