#!/usr/bin/env python3
"""
直接测试舵机运动
不经过角度转换，直接发送位置值
"""

import serial
import time


def checksum(data):
    return (~sum(data)) & 0xFF


def build_write_packet(servo_id, address, values):
    """构建写指令包"""
    params = [address] + values
    length = len(params) + 2
    data = [servo_id, length, 0x28] + params
    chk = checksum(data)
    return bytes([0xFF, 0xFF] + data + [chk])


def test_direct(port='COM18'):
    print("="*60)
    print("直接舵机运动测试")
    print("="*60)
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.1)
        time.sleep(0.3)
        print(f"✓ 连接到 {port}")
        
        # 测试1: 启用舵机1扭矩
        print("\n测试1: 启用舵机1扭矩")
        packet = build_write_packet(1, 40, [1])  # 地址40=扭矩使能, 值1=启用
        print(f"  发送: {packet.hex()}")
        ser.reset_input_buffer()
        ser.write(packet)
        time.sleep(0.2)
        resp = ser.read(20)
        print(f"  响应: {resp.hex() if resp else '(无)'}")
        
        # 测试2: 设置舵机1位置到1500
        print("\n测试2: 设置舵机1位置到1500")
        pos = 1500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        speed = 200
        spd_low = speed & 0xFF
        spd_high = (speed >> 8) & 0xFF
        
        packet = build_write_packet(1, 42, [pos_low, pos_high, spd_low, spd_high])
        print(f"  发送: {packet.hex()}")
        ser.reset_input_buffer()
        ser.write(packet)
        time.sleep(1.5)  # 等待运动完成
        resp = ser.read(20)
        print(f"  响应: {resp.hex() if resp else '(无)'}")
        print("  舵机1应该动了！")
        
        # 测试3: 设置舵机1位置到2500
        print("\n测试3: 设置舵机1位置到2500")
        pos = 2500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        
        packet = build_write_packet(1, 42, [pos_low, pos_high, spd_low, spd_high])
        print(f"  发送: {packet.hex()}")
        ser.reset_input_buffer()
        ser.write(packet)
        time.sleep(1.5)
        resp = ser.read(20)
        print(f"  响应: {resp.hex() if resp else '(无)'}")
        print("  舵机1应该转到另一个位置！")
        
        # 测试4: 设置舵机1位置到2048（中心）
        print("\n测试4: 设置舵机1位置到2048（中心）")
        pos = 2048
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        
        packet = build_write_packet(1, 42, [pos_low, pos_high, spd_low, spd_high])
        print(f"  发送: {packet.hex()}")
        ser.reset_input_buffer()
        ser.write(packet)
        time.sleep(1.5)
        resp = ser.read(20)
        print(f"  响应: {resp.hex() if resp else '(无)'}")
        
        # 测试5: 启用所有舵机并测试
        print("\n测试5: 启用所有舵机扭矩")
        for i in range(1, 7):
            packet = build_write_packet(i, 40, [1])
            ser.write(packet)
            time.sleep(0.05)
        print("  所有舵机扭矩已启用")
        
        # 测试6: 同时移动所有舵机
        print("\n测试6: 同时移动所有舵机到1500")
        for i in range(1, 7):
            pos = 1500
            pos_low = pos & 0xFF
            pos_high = (pos >> 8) & 0xFF
            packet = build_write_packet(i, 42, [pos_low, pos_high, 0, 0])
            ser.write(packet)
            time.sleep(0.02)
        print("  等待运动...")
        time.sleep(2)
        
        print("\n测试7: 同时移动所有舵机到2048")
        for i in range(1, 7):
            pos = 2048
            pos_low = pos & 0xFF
            pos_high = (pos >> 8) & 0xFF
            packet = build_write_packet(i, 42, [pos_low, pos_high, 0, 0])
            ser.write(packet)
            time.sleep(0.02)
        print("  等待运动...")
        time.sleep(2)
        
        ser.close()
        print("\n测试完成!")
        
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    test_direct(port)
    input("\n按Enter退出...")