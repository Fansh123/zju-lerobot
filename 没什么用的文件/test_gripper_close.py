#!/usr/bin/env python3
"""
夹爪闭合位置测试
测试不同的闭合位置值
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
    print("夹爪闭合位置测试")
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
    
    # 先打开
    print("\n先打开夹爪...")
    set_position(ser, 6, 2500, speed=200)
    time.sleep(1.5)
    
    # 测试不同的闭合位置
    close_positions = [500, 400, 300, 200, 100, 50, 0]
    
    for pos in close_positions:
        print(f"\n测试闭合位置: {pos}")
        set_position(ser, 6, pos, speed=150)
        time.sleep(1.5)
        
        resp = input("夹爪是否完全闭合? (y/n/q): ").strip().lower()
        if resp == 'y':
            print(f"✓ 最佳闭合位置: {pos}")
            break
        elif resp == 'q':
            break
    
    # 最后打开
    print("\n打开夹爪...")
    set_position(ser, 6, 2500, speed=200)
    time.sleep(1)
    
    ser.close()
    print("\n测试完成")


if __name__ == "__main__":
    main()
