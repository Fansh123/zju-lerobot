#!/usr/bin/env python3
"""
夹爪闭合测试 - 测试更小的闭合值
"""

import serial
import time
import sys


def checksum(data):
    return (~sum(data)) & 0xFF


def build_packet(servo_id, instruction, parameters=None):
    if parameters is None:
        parameters = []
    length = len(parameters) + 2
    data = [servo_id, length, instruction] + parameters
    chk = checksum(data)
    return bytes([0xFF, 0xFF] + data + [chk])


def set_position(ser, servo_id, position, speed=200):
    position = max(0, min(4095, int(position)))
    pos_low = position & 0xFF
    pos_high = (position >> 8) & 0xFF
    spd_low = speed & 0xFF
    spd_high = (speed >> 8) & 0xFF
    
    packet = build_packet(servo_id, 0x03, [42, pos_low, pos_high, spd_low, spd_high])
    ser.write(packet)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*50)
    print("夹爪闭合测试")
    print("="*50)
    
    ser = serial.Serial(port, 1000000, timeout=0.5)
    time.sleep(0.3)
    print(f"✓ 连接到 {port}")
    
    # 启用扭矩
    for i in range(1, 7):
        packet = build_packet(i, 0x03, [40, 1])
        ser.write(packet)
        time.sleep(0.02)
    
    # 设置速度
    for i in range(1, 7):
        packet = build_packet(i, 0x03, [46, 200, 0])
        ser.write(packet)
        time.sleep(0.02)
    
    print("\n开始测试...")
    
    # 先打开
    print("\n1. 夹爪打开 (位置2500)...")
    set_position(ser, 6, 2500, speed=200)
    time.sleep(1.5)
    
    # 测试更小的闭合值
    print("\n2. 夹爪闭合 (位置200)...")
    set_position(ser, 6, 200, speed=150)
    time.sleep(1.5)
    
    print("\n3. 夹爪打开 (位置2500)...")
    set_position(ser, 6, 2500, speed=200)
    time.sleep(1.5)
    
    print("\n4. 夹爪闭合 (位置100)...")
    set_position(ser, 6, 100, speed=150)
    time.sleep(1.5)
    
    print("\n5. 夹爪打开 (位置2500)...")
    set_position(ser, 6, 2500, speed=200)
    time.sleep(1.5)
    
    print("\n6. 夹爪闭合 (位置0)...")
    set_position(ser, 6, 0, speed=150)
    time.sleep(1.5)
    
    print("\n7. 夹爪打开 (位置2500)...")
    set_position(ser, 6, 2500, speed=200)
    time.sleep(1.5)
    
    ser.close()
    print("\n✓ 测试完成!")


if __name__ == "__main__":
    main()
