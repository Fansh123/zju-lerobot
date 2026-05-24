#!/usr/bin/env python3
"""
详细协议测试
检查每个步骤是否正确
"""

import serial
import time


def checksum(data):
    return (~sum(data)) & 0xFF


def send_and_show(ser, name, packet, wait=0.3):
    """发送并显示响应"""
    print(f"\n{name}:")
    print(f"  发送: {packet.hex()}")
    
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(wait)
    
    response = ser.read(50)
    print(f"  收到: {response.hex() if response else '(无)'}")
    
    if response and len(response) >= 6:
        print(f"  解析: ID={response[2]}, Len={response[3]}, Err={response[4]}")
    
    return response


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("详细协议测试")
    print("="*60)
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.3)
        time.sleep(0.3)
        print(f"✓ 连接到 {port}")
        
        # ===== 舵机1测试 =====
        servo_id = 1
        
        # 1. PING
        data = [servo_id, 2, 0x01]
        packet = bytes([0xFF, 0xFF] + data + [checksum(data)])
        send_and_show(ser, "1. PING", packet)
        
        # 2. 读取扭矩使能状态 (地址40)
        data = [servo_id, 4, 0x2A, 40, 1]  # READ, 地址40, 1字节
        packet = bytes([0xFF, 0xFF] + data + [checksum(data)])
        send_and_show(ser, "2. 读扭矩状态 (地址40)", packet)
        
        # 3. 写扭矩使能=1 (地址40)
        data = [servo_id, 4, 0x28, 40, 1]  # WRITE, 地址40, 值1
        packet = bytes([0xFF, 0xFF] + data + [checksum(data)])
        send_and_show(ser, "3. 写扭矩使能=1", packet)
        
        # 4. 再次读取扭矩状态
        data = [servo_id, 4, 0x2A, 40, 1]
        packet = bytes([0xFF, 0xFF] + data + [checksum(data)])
        resp = send_and_show(ser, "4. 再次读扭矩状态", packet)
        
        # 5. 读取当前位置 (地址56, 2字节)
        data = [servo_id, 4, 0x2A, 56, 2]
        packet = bytes([0xFF, 0xFF] + data + [checksum(data)])
        resp = send_and_show(ser, "5. 读当前位置 (地址56)", packet)
        
        # 6. 写目标位置 (地址42)
        # 位置 2048 = 中心
        pos = 2048
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        data = [servo_id, 5, 0x28, 42, pos_low, pos_high]  # 只写位置，不写速度
        packet = bytes([0xFF, 0xFF] + data + [checksum(data)])
        send_and_show(ser, f"6. 写目标位置={pos} (地址42, 2字节)", packet)
        
        time.sleep(1)
        
        # 7. 写目标位置到 1500
        pos = 1500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        data = [servo_id, 5, 0x28, 42, pos_low, pos_high]
        packet = bytes([0xFF, 0xFF] + data + [checksum(data)])
        send_and_show(ser, f"7. 写目标位置={pos}", packet)
        
        time.sleep(2)
        
        # 8. 写目标位置到 2500
        pos = 2500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        data = [servo_id, 5, 0x28, 42, pos_low, pos_high]
        packet = bytes([0xFF, 0xFF] + data + [checksum(data)])
        send_and_show(ser, f"8. 写目标位置={pos}", packet)
        
        time.sleep(2)
        
        # 9. 返回中心
        pos = 2048
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        data = [servo_id, 5, 0x28, 42, pos_low, pos_high]
        packet = bytes([0xFF, 0xFF] + data + [checksum(data)])
        send_and_show(ser, f"9. 返回中心位置={pos}", packet)
        
        ser.close()
        print("\n测试完成!")
        
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()
    input("\n按Enter退出...")